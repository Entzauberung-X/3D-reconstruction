/**
 * @file pose_solver.cpp
 * @brief 平面标定板位姿解算与二义性消解的实现 (见 pose_solver.h 的说明)
 */

#include "core/pose_solver.h"

#include <opencv2/calib3d.hpp>
#include <cmath>
#include <limits>

namespace Calib {

namespace {

constexpr double kPi = 3.14159265358979323846;

/** cv::Vec3d → 3x1 CV_64F 的 cv::Mat */
cv::Mat toMat(const cv::Vec3d& v)
{
    return cv::Mat(3, 1, CV_64F, const_cast<double*>(&v[0])).clone();
}

/** 3x1 CV_64F 的 cv::Mat → cv::Vec3d */
cv::Vec3d toVec(const cv::Mat& m)
{
    cv::Vec3d v(0, 0, 0);
    for (int i = 0; i < 3 && i < m.total(); ++i) v[i] = m.at<double>(i);
    return v;
}

/** 绕 z 轴 180° 的旋转矩阵 */
cv::Matx33d rz180()
{
    return cv::Matx33d(-1, 0, 0,
                       0, -1, 0,
                       0,  0, 1);
}

/** 旋转向量 → 旋转矩阵 */
cv::Matx33d rodriguesToMat(const cv::Vec3d& rvec)
{
    cv::Mat R;
    cv::Rodrigues(toMat(rvec), R);
    cv::Matx33d out;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out(r, c) = R.at<double>(r, c);
    return out;
}

/** 旋转矩阵 → 旋转向量 */
cv::Vec3d matToRodrigues(const cv::Matx33d& R)
{
    cv::Mat m(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m.at<double>(r, c) = R(r, c);
    cv::Mat rvec;
    cv::Rodrigues(m, rvec);
    return toVec(rvec);
}

/** 用与现有代码完全相同的调用精化位姿 (useExtrinsicGuess=true) */
bool refineWithGuess(const std::vector<cv::Point3f>& obj,
                     const std::vector<cv::Point2f>& img,
                     cv::InputArray K, cv::InputArray D,
                     cv::Vec3d& rvec, cv::Vec3d& tvec)
{
    cv::Mat rv = toMat(rvec), tv = toMat(tvec);
    if (!cv::solvePnP(obj, img, K, D, rv, tv, true, cv::SOLVEPNP_ITERATIVE))
        return false;
    rvec = toVec(rv);
    tvec = toVec(tv);
    return true;
}

/** 全部角点变换后是否都在相机前方 (z > 0)。IPPE 不做深度过滤, 必须自检 */
bool cheiralityOk(const std::vector<cv::Point3f>& obj,
                  const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
    const cv::Matx33d R = rodriguesToMat(rvec);
    for (const auto& p : obj) {
        const double z = R(2,0) * p.x + R(2,1) * p.y + R(2,2) * p.z + tvec[2];
        if (z <= 0.0) return false;
    }
    return true;
}

/** 板法向 (R·e3) 与"由相机指向板心的向量"(t) 的带符号点积。
 *  符号取决于 objectPoints 的排布手性: 本工程用 (c*sq, r*sq, 0), X 向右、Y 向下
 *  (图像式), 右手定则下 Z 指向板内, 故"看得到正面"的位姿其 n·t 为**正**。
 *  但**不要把这个结论写死** —— 换一种角点排布 (或换标定板约定) 符号就会反过来。
 *  调用方一律以 legacy 解的符号作参考, 见 enumeratePlanarCandidates。 */
double normalDotT(const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
    const cv::Matx33d R = rodriguesToMat(rvec);
    const cv::Vec3d n(R(0,2), R(1,2), R(2,2));
    return n.dot(tvec);
}

/** 由 objectPoints 推断棋盘格的行列数。要求是规则行列网格, 否则返回 false */
bool inferGridSize(const std::vector<cv::Point3f>& obj, int& W, int& H)
{
    if (obj.empty()) return false;
    std::vector<float> xs, ys;
    for (const auto& p : obj) {
        bool seen = false;
        for (float v : xs) if (std::fabs(v - p.x) < 1e-4f) { seen = true; break; }
        if (!seen) xs.push_back(p.x);
        seen = false;
        for (float v : ys) if (std::fabs(v - p.y) < 1e-4f) { seen = true; break; }
        if (!seen) ys.push_back(p.y);
    }
    W = static_cast<int>(xs.size());
    H = static_cast<int>(ys.size());
    return W > 0 && H > 0 && static_cast<size_t>(W) * H == obj.size();
}

/** 两个候选是否实质相同 (相对转角 < 阈值) */
bool samePose(const cv::Vec3d& a, const cv::Vec3d& b, double tolDeg = 2.0)
{
    return relativeRotationDeg(a, b) < tolDeg;
}

/// 序列消歧用的位姿节点
struct Node {
    bool          valid = false;
    int           idx = 0;          // 候选池下标
    cv::Vec3d     rvec, tvec;
    double        reproj = 0.0;
    bool          isLegacy = false;
    bool          permuted = false;
};

} // namespace

// ---------------------------------------------------------------------------
// 基础工具
// ---------------------------------------------------------------------------

void makeChessboardObjectPoints(const cv::Size& boardSize, float squareSize,
                               std::vector<cv::Point3f>& out)
{
    out.clear();
    out.reserve(static_cast<size_t>(boardSize.width) * boardSize.height);
    for (int r = 0; r < boardSize.height; ++r)
        for (int c = 0; c < boardSize.width; ++c)
            out.push_back(cv::Point3f(c * squareSize, r * squareSize, 0.0f));
}

double reprojRms(const std::vector<cv::Point3f>& obj,
                 const std::vector<cv::Point2f>& img,
                 cv::InputArray rvec, cv::InputArray tvec,
                 cv::InputArray K, cv::InputArray D)
{
    if (obj.empty() || img.size() != obj.size()) return -1.0;
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, K, D, proj);
    double sum = 0.0;
    int    n   = 0;
    for (size_t i = 0; i < proj.size() && i < img.size(); ++i) {
        const double dx = proj[i].x - img[i].x;
        const double dy = proj[i].y - img[i].y;
        sum += dx * dx + dy * dy;
        ++n;
    }
    return n > 0 ? std::sqrt(sum / n) : -1.0;
}

double relativeRotationDeg(const cv::Vec3d& rvecA, const cv::Vec3d& rvecB)
{
    const cv::Matx33d dR = rodriguesToMat(rvecA) * rodriguesToMat(rvecB).t();
    const double tr = dR(0,0) + dR(1,1) + dR(2,2);
    const double cosv = std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0));
    return std::acos(cosv) * 180.0 / kPi;
}

bool isInPlaneSelfSymmetric(const std::vector<cv::Point3f>& obj, double tolMm)
{
    if (obj.empty()) return false;
    double cx = 0.0, cy = 0.0;
    for (const auto& p : obj) { cx += p.x; cy += p.y; }
    cx /= obj.size(); cy /= obj.size();

    // 对每个点, 检查"绕格心 180° 旋转"后的位置是否仍有点与之重合
    for (const auto& p : obj) {
        const double fx = 2.0 * cx - p.x;
        const double fy = 2.0 * cy - p.y;
        bool found = false;
        for (const auto& q : obj) {
            if (std::fabs(q.x - fx) <= tolMm && std::fabs(q.y - fy) <= tolMm &&
                std::fabs(q.z - p.z) <= tolMm) {
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

bool makeFlipCandidate(const std::vector<cv::Point3f>& obj,
                       const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                       cv::Vec3d& rvecOut, cv::Vec3d& tvecOut)
{
    if (obj.empty()) return false;

    // 只有自对称的板才存在这个翻转二义性
    if (!isInPlaneSelfSymmetric(obj)) return false;

    double cx = 0.0, cy = 0.0;
    for (const auto& p : obj) { cx += p.x; cy += p.y; }
    cx /= obj.size(); cy /= obj.size();

    // 绕格心 (cx,cy) 转 180°:  Q(p) = Rz(pi)·p + d,  d = (2cx, 2cy, 0)
    // 翻转对应关系下 R_f·Q(P) + t_f = R·P + t
    //   => R_f = R·Rz(pi)⁻¹ = R·Rz(pi)
    //   => t_f = t - R_f·d
    const cv::Matx33d R   = rodriguesToMat(rvec);
    const cv::Matx33d Rf  = R * rz180();
    const cv::Vec3d   d(2.0 * cx, 2.0 * cy, 0.0);

    rvecOut = matToRodrigues(Rf);
    tvecOut = tvec - cv::Vec3d(Rf(0,0)*d[0] + Rf(0,1)*d[1] + Rf(0,2)*d[2],
                               Rf(1,0)*d[0] + Rf(1,1)*d[1] + Rf(1,2)*d[2],
                               Rf(2,0)*d[0] + Rf(2,1)*d[1] + Rf(2,2)*d[2]);
    return true;
}

// ---------------------------------------------------------------------------
// 阶段A: 单帧候选枚举
// ---------------------------------------------------------------------------

bool permuteChessboardImagePoints(const std::vector<cv::Point3f>& obj,
                                  const std::vector<cv::Point2f>& img,
                                  std::vector<cv::Point2f>& out)
{
    int W = 0, H = 0;
    if (!inferGridSize(obj, W, H) || img.size() != obj.size()) return false;
    out.resize(img.size());
    for (size_t i = 0; i < img.size(); ++i) {
        const int r = static_cast<int>(i) / W;
        const int c = static_cast<int>(i) % W;
        const int j = (H - 1 - r) * W + (W - 1 - c);
        out[i] = img[j];
    }
    return true;
}

bool enumeratePlanarCandidates(const std::vector<cv::Point3f>& obj,
                              const std::vector<cv::Point2f>& img,
                              cv::InputArray K, cv::InputArray D,
                              const cv::Vec3d* legacyRvec, const cv::Vec3d* legacyTvec,
                              std::vector<PoseCandidate>& out,
                              bool* ippeThrew)
{
    out.clear();
    if (ippeThrew) *ippeThrew = false;
    if (obj.size() < 4 || img.size() != obj.size() || K.empty()) return false;

    // 可见面判据的参考符号: 由 legacy 解**自校准**, 不写死约定。
    //
    // 这里踩过一个坑, 值得记下来: 早先按"相机在板正面一侧"直接写死 n·t < 0, 结果在真实
    // 数据上 legacy 与配对B的 n·t 同为正号(+132/+159), 判据对两者都判 false; 而 legacy
    // 为了保持基线逐位不变被豁免过滤、配对B却被真过滤掉 —— 于是整个消歧在真实数据上
    // 静默失效(合成测试因位姿恰好满足 n·t<0 而漏检)。改用"与 legacy 同号"即可:
    // 数学上翻转位姿与 legacy 的 n·t 逐位相同(可证 n·R_f·d = 0), 所以主失效模式的
    // 正确候选永不会被误滤, 而法向真的反了的 IPPE 解仍会被挡掉。
    double refNtSign = 0.0;   // 0 = 未能自校准, 此时不做可见面过滤

    // 把候选的相对残差/几何校验都按"它自己所属的配对"填充。
    // 这一步是必须的: 配对 B 的位姿对**原始**图点残差极大(那不是它的数据),
    // 若按原始图点评分会被误当成"拟合很差的解"而丢弃。
    auto fill = [&](PoseCandidate& c, const std::vector<cv::Point2f>& pts) {
        c.reprojRmsPx  = reprojRms(obj, pts, toMat(c.rvec), toMat(c.tvec), K, D);
        c.cheiralityOk = cheiralityOk(obj, c.rvec, c.tvec);
        const double nt = normalDotT(c.rvec, c.tvec);
        c.frontFaceOk = (refNtSign == 0.0) ? true : (nt * refNtSign > 0.0);
    };

    auto push = [&](const cv::Vec3d& rv, const cv::Vec3d& tv, const std::vector<cv::Point2f>& pts,
                    bool fromLegacy, bool fromIppe, bool permuted) {
        if (!std::isfinite(rv[0]) || !std::isfinite(tv[0])) return;
        for (const auto& e : out)
            if (samePose(e.rvec, rv) && e.permuted == permuted) return;   // 去重
        PoseCandidate c;
        c.rvec = rv; c.tvec = tv;
        c.fromLegacy = fromLegacy; c.fromIppe = fromIppe; c.permuted = permuted;
        fill(c, pts);
        out.push_back(c);
    };

    // IPPE 候选: 平面目标的解析解, 通常 2 个。
    // 注意 OpenCV 4.5.4 对 SOLVEPNP_IPPE 只承诺"返回 2 个解", 无排序承诺
    // (排序承诺属于 solveP3P 的文档块), 因此不依赖顺序, 一律自算误差后比较。
    auto addIppe = [&](const std::vector<cv::Point2f>& pts, bool permuted) {
        std::vector<cv::Mat> rvecs, tvecs;
        try {
            cv::solvePnPGeneric(obj, pts, K, D, rvecs, tvecs, false,
                                cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), cv::noArray());
        } catch (const cv::Exception&) {
            // 退化配置/IPPE 失败: 吞掉并降级, 绝不让上层整体失败
            if (ippeThrew) *ippeThrew = true;
            return;
        }
        for (size_t i = 0; i < rvecs.size() && i < tvecs.size(); ++i) {
            cv::Vec3d rv = toVec(rvecs[i]), tv = toVec(tvecs[i]);
            cv::Vec3d rr = rv, tt = tv;
            if (!refineWithGuess(obj, pts, K, D, rr, tt)) continue;   // IPPE 原始解精度偶有不足
            push(rr, tt, pts, false, true, permuted);
        }
    };

    // ---- 配对 A: 检测顺序正常 ----
    // legacy 必须是"与改动前完全相同的调用"的原样结果, 这样选中它时输出才逐位不变
    {
        cv::Vec3d lr(0,0,0), lt(0,0,0);
        bool have = false;
        if (legacyRvec && legacyTvec) {
            lr = *legacyRvec; lt = *legacyTvec; have = true;
        } else {
            cv::Mat rv, tv;
            if (cv::solvePnP(obj, img, K, D, rv, tv)) { lr = toVec(rv); lt = toVec(tv); have = true; }
        }
        if (have) {
            refNtSign = normalDotT(lr, lt) >= 0.0 ? 1.0 : -1.0;   // 以 legacy 自校准
            PoseCandidate c;
            c.rvec = lr; c.tvec = lt;
            c.fromLegacy = true; c.fromIppe = false; c.permuted = false;
            fill(c, img);
            out.push_back(c);
        }
        addIppe(img, false);
    }

    // ---- 配对 B: 检测顺序被翻转 ----
    // 把图点置换回规范顺序后重新解算 —— 这时解出的才是**物理正确**的位姿。
    // (直接用 flip(legacy) 当初始猜测亦可, 但对原始图点精化会把它拉回配对 A,
    //  必须对置换后的图点精化, 见 fill 的说明。)
    std::vector<cv::Point2f> imgPerm;
    if (permuteChessboardImagePoints(obj, img, imgPerm)) {
        cv::Mat rv, tv;
        if (cv::solvePnP(obj, imgPerm, K, D, rv, tv))
            push(toVec(rv), toVec(tv), imgPerm, false, false, true);
        addIppe(imgPerm, true);
    }

    return !out.empty();
}

// ---------------------------------------------------------------------------
// 阶段B: 序列消歧 (Viterbi)
// ---------------------------------------------------------------------------

bool disambiguatePlanarSequence(const std::vector<std::vector<cv::Point2f>>& imgPerFrame,
                               const std::vector<cv::Point3f>& obj,
                               cv::InputArray K, cv::InputArray D,
                               const PlanarSequenceOptions& opt,
                               std::vector<FramePoseResult>& out)
{
    out.assign(imgPerFrame.size(), FramePoseResult());
    if (imgPerFrame.empty() || obj.empty()) return false;

    // ---- 逐帧枚举候选并成节点 ----
    std::vector<std::vector<Node>> nodes(imgPerFrame.size());
    for (size_t i = 0; i < imgPerFrame.size(); ++i) {
        FramePoseResult& res = out[i];
        if (imgPerFrame[i].size() != obj.size()) continue;   // 该帧未检测到棋盘格

        std::vector<PoseCandidate> cands;
        bool ippeThrew = false;
        if (!enumeratePlanarCandidates(obj, imgPerFrame[i], K, D, nullptr, nullptr,
                                       cands, &ippeThrew))
            continue;

        for (const auto& c : cands) {
            if (!opt.useIppeCandidates && !c.fromLegacy) continue;   // 行为等价模式
            if (c.permuted && !opt.allowFlipCandidate) continue;     // 关闭翻转假设
            if (!c.fromLegacy) {
                if (!c.cheiralityOk) continue;   // 相机背后的解不能用于连续性比较
                if (!c.frontFaceOk)  continue;   // 看到板背面的解无物理意义
            }
            Node n;
            n.valid = true;
            n.idx = static_cast<int>(nodes[i].size());
            n.rvec = c.rvec; n.tvec = c.tvec;
            n.reproj = c.reprojRmsPx;
            n.isLegacy = c.fromLegacy;
            n.permuted = c.permuted;
            nodes[i].push_back(n);
        }

        // 没有可用候选则退回 legacy 原样解, 保证不因消歧而丢帧
        if (nodes[i].empty()) {
            std::vector<PoseCandidate> raw;
            if (enumeratePlanarCandidates(obj, imgPerFrame[i], K, D, nullptr, nullptr, raw)
                && !raw.empty() && raw.front().fromLegacy) {
                Node n;
                n.valid = true; n.idx = 0; n.isLegacy = true;
                n.rvec = raw.front().rvec; n.tvec = raw.front().tvec;
                n.reproj = raw.front().reprojRmsPx;
                nodes[i].push_back(n);
            }
        }
        res.solved = !nodes[i].empty();
        res.candidateCount = static_cast<int>(nodes[i].size());
        if (ippeThrew && res.solved) res.note = "IPPE 求解异常, 已退回常规 PnP";
    }

    // 参与链式消歧的帧 (已解算成功)
    std::vector<int> chain;
    for (size_t i = 0; i < nodes.size(); ++i)
        if (!nodes[i].empty()) chain.push_back(static_cast<int>(i));
    if (chain.empty()) return false;

    // ---- 节点/边代价 ----
    auto nodeCost = [&](const Node& n) -> double {
        return opt.wReproj * n.reproj + (n.isLegacy ? 0.0 : opt.wGauge);
    };
    auto edgeCost = [&](const Node& prev, const Node& cur) -> double {
        const double step = relativeRotationDeg(cur.rvec, prev.rvec);
        if (step > opt.maxStepDeg) {
            const double over = step - opt.maxStepDeg;
            return opt.wEdge * over * over;          // 真翻转 ≈180° → 巨额罚
        }
        if (step < opt.minStepDeg) {
            const double under = opt.minStepDeg - step;
            return opt.wEdge * under * under;        // 重复帧
        }
        return 0.0;
    };

    // ---- Viterbi (每帧状态数 ≤ 候选数, 通常 1~3) ----
    const int NF = static_cast<int>(chain.size());
    std::vector<std::vector<double>> dp(NF), prevCost(NF);
    std::vector<std::vector<int>>    prevIdx(NF);
    for (int f = 0; f < NF; ++f) {
        const int fi = chain[f];
        const int M  = static_cast<int>(nodes[fi].size());
        dp[f].assign(M, std::numeric_limits<double>::max());
        prevCost[f].assign(M, 0.0);
        prevIdx[f].assign(M, -1);
    }
    for (int k = 0; k < static_cast<int>(nodes[chain[0]].size()); ++k)
        dp[0][k] = nodeCost(nodes[chain[0]][k]);

    for (int f = 1; f < NF; ++f) {
        const auto& prevNodes = nodes[chain[f - 1]];
        const auto& curNodes  = nodes[chain[f]];
        for (int l = 0; l < static_cast<int>(curNodes.size()); ++l) {
            for (int k = 0; k < static_cast<int>(prevNodes.size()); ++k) {
                if (dp[f - 1][k] == std::numeric_limits<double>::max()) continue;
                const double c = dp[f - 1][k] + edgeCost(prevNodes[k], curNodes[l]) + nodeCost(curNodes[l]);
                if (c < dp[f][l]) { dp[f][l] = c; prevIdx[f][l] = k; }
            }
        }
    }

    // ---- 回溯最优路径 ----
    std::vector<int> pick(NF, 0);
    {
        int best = 0;
        for (int l = 1; l < static_cast<int>(dp[NF - 1].size()); ++l)
            if (dp[NF - 1][l] < dp[NF - 1][best]) best = l;
        pick[NF - 1] = best;
        for (int f = NF - 1; f > 0; --f)
            pick[f - 1] = prevIdx[f][pick[f]] >= 0 ? prevIdx[f][pick[f]] : 0;
    }

    // ---- 回填结果 ----
    for (int f = 0; f < NF; ++f) {
        const int  fi = chain[f];
        const auto& ns = nodes[fi];
        FramePoseResult& res = out[fi];

        const int chosen = pick[f];
        res.solved = true;
        res.chosen = chosen;
        res.rvec = ns[chosen].rvec;
        res.tvec = ns[chosen].tvec;
        res.reprojRmsPx = ns[chosen].reproj;
        res.permuted = ns[chosen].permuted;

        if (f > 0) {
            res.stepChosenDeg = relativeRotationDeg(ns[chosen].rvec, nodes[chain[f-1]][pick[f-1]].rvec);
        }

        // 辅助解: 取与选中解差异最大的那个候选, 用于暴露"曾发生过翻转"
        int alt = -1;
        double altSep = 0.0;
        for (int k = 0; k < static_cast<int>(ns.size()); ++k) {
            if (k == chosen) continue;
            const double sep = relativeRotationDeg(ns[k].rvec, ns[chosen].rvec);
            if (sep > altSep) { altSep = sep; alt = k; }
        }
        if (alt >= 0) {
            res.ambiguous = true;
            res.reprojRmsAltPx = ns[alt].reproj;
            if (f > 0)
                res.stepAltDeg = relativeRotationDeg(ns[alt].rvec, nodes[chain[f-1]][pick[f-1]].rvec);
            else
                res.stepAltDeg = altSep;
        }

        // ---- 判级 ----
        // 只有"与选中解相差 >90°"的候选才算真正的翻转候选; 小于该量级只是同类解的
        // 微小分歧, 不值得记录。
        // 注意: 对自对称标定板, **每一帧都必然存在**一个 180° 翻转伙伴, 所以
        // "存在翻转候选"本身零信息量, 不能据此报警 (否则干净序列会帧帧告警)。
        // 真正值得报的是"配对方式相对序列起始发生了改变" —— 检测器中途翻转了顺序。
        //   flag=0  该帧配对与序列起始一致 (无论是否 permuted)
        //   flag=1  该帧配对与序列起始不同, 已被连续性纠正
        //   flag=2  两个候选的步长都落在合理区间 —— 无法消解, 需人工介入
        // "已消解"绝不能标 2, 否则"异常帧数量"检查会永久失败。
        if (alt >= 0 && altSep > 90.0 &&
            (f > 0) && res.stepAltDeg >= opt.minStepDeg && res.stepAltDeg <= opt.maxStepDeg) {
            res.flipFlag = 2;
            res.note = QString("位姿二义性无法消解: 两候选步长 %1° / %2° 均落在 [%3°,%4°] 内")
                           .arg(res.stepChosenDeg, 0, 'f', 2).arg(res.stepAltDeg, 0, 'f', 2)
                           .arg(opt.minStepDeg, 0, 'f', 2).arg(opt.maxStepDeg, 0, 'f', 1);
        }
    }

    // 相对序列起始的配对基准: 检测器若从头就翻转顺序, 属全局相位(规范自由度),
    // 不构成问题; 只有"中途改变"才是需要报出来的真实故障。
    bool refPermuted = false;
    for (int idx : chain)
        if (out[idx].solved) { refPermuted = out[idx].permuted; break; }

    for (int idx : chain) {
        FramePoseResult& res = out[idx];
        if (!res.solved || res.flipFlag == 2) continue;
        if (res.permuted != refPermuted) {
            res.flipFlag = 1;
            res.note = QString("检测顺序相对序列起始发生翻转, 已按连续性纠正回规范配对 (翻转解步长 %1° 越界)")
                           .arg(res.stepAltDeg, 0, 'f', 1);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// 阶段C: 绝对校验
// ---------------------------------------------------------------------------

void annotateWithAxis(bool normalSignAgrees, double normalDevDeg, double tolDeg,
                      FramePoseResult& io)
{
    if (!io.solved) return;

    if (!normalSignAgrees) {
        // 用 max 语义合并, 不吃掉已有告警
        io.flipFlag = std::max(io.flipFlag, 2);
        if (!io.note.isEmpty()) io.note += "; ";
        io.note += "棋盘法向相对转台轴反向 (带符号校验)";
        return;
    }
    if (normalDevDeg > tolDeg) {
        io.flipFlag = std::max(io.flipFlag, 1);
        if (!io.note.isEmpty()) io.note += "; ";
        io.note += QString("棋盘法向偏离轴 %1°").arg(normalDevDeg, 0, 'f', 2);
    }
}

} // namespace Calib
