// ==================== 重建调试包 ====================
// 与「转台轴标定调试包」(exportDebugPackage) 对应的一套重建侧取证材料。
//
// 为什么要单独一套: 轴标定那个包里的重建内容只有一份原始日志, 而"点云大量丢失"这类问题
// 恰恰不是日志能回答的 —— 日志里只有聚合计数 (S3 剔除 26256 个), 分不清是
// 【S2 匹配错了】/【光平面不准】/【S1 根本没提出光条】/【阈值偏紧】, 四者处理方式完全不同。
// 所以这里除了把逐帧计数落盘, 还要对若干样本帧重跑一遍 S1~S3, 把**每一对匹配的几何量**
// 导出来 —— 包括一个不依赖判据的独立真值 (双目 DLT 点到光平面的距离), 用来反查判据本身。
//
// 数据来源分工:
//   · 逐帧计数 / 逐视角点云 / 最终点云  ← 上次那轮重建留在 worker 上的结果, 零成本
//   · 样本帧逐对匹配明细 / 光条叠加图    ← 导出时按**上次提交的 JobInput**重跑, 几秒钟
// 重跑必须用 m_lastReconJob 而不是当前界面控件值: 用户在跑完之后动一下滑块,
// 导出的样本就会和被诊断的那次对不上, 整包结论都会跑偏。
#include "ui/mainwindow.h"
#include "ui/theme.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QDateTime>
#include <QMessageBox>
#include <QFileDialog>
#include <QApplication>
#include <QDebug>

#include <opencv2/opencv.hpp>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

/** 导出期间把 OpenCV 压成单线程, 退出时恢复。
 *  理由: worker 在并行重建时对每个视角都强制过单线程, 串行重跑要复现同一套数值就得同环境。 */
struct CvThreadGuard {
    int prev;
    CvThreadGuard() : prev(cv::getNumThreads()) { cv::setNumThreads(1); }
    ~CvThreadGuard() { cv::setNumThreads(prev); }
};

/** CSV 文本单元格: 加引号并把内部引号翻倍、换行换成空格, 否则一个带逗号的路径就能破整行 */
QString csvCell(const QString& s)
{
    QString t = s;
    t.replace('"', "\"\"");
    t.replace('\n', ' ').replace('\r', ' ');
    return '"' + t + '"';
}

/** 重投影误差 8 桶的列名, 与 ViewOutcomeStats::reprojBucket 一一对应 */
const char* kHistHeads[8] = {"<1px", "1-2px", "2-5px", "5-10px",
                             "10-20px", "20-50px", "50-100px", ">=100px"};

/** 拒绝原因的中文名 (MatchDiag::RejectReason) */
QString rejectReasonName(int r)
{
    switch (r) {
    case 0:  return "接受";
    case 1:  return "空间残差超限";       // 旧判据(右图重投影), 已不再产生
    case 2:  return "交点在相机后方";
    case 3:  return "深度越界";
    case 4:  return "两射线接近平行";
    case 5:  return "DLT求解失败";
    case 6:  return "光平面一致性超限";
    default: return "未走到判据";
    }
}

/** S5 配准自检: 在最终点云上取物体的圆柱中段, 最小二乘拟合横截面圆心, 与标定轴点比较。
 *
 *  为什么这个数值得自动算出来:
 *  S5 融合后的点云 = ∪ᵢ 把视角 i 绕「标定轴点 C」转 −i×步长。所以任意高度上的横截面
 *  必然是**一族以 C 为圆心的同心圆** —— 拟合出来的圆心就该精确等于 C。
 *  实测偏离多少, 物体就被径向摊开多少(±该值): 偏 7mm 就是外壁外面多出一圈十几毫米宽的
 *  稀疏拖尾, 在 3D 视图里看起来像"物体旁边多了一层不存在的曲面", 而且一开 ICP 就被揉掉。
 *  这个数在融合结果里看一眼点云是看不出来的(点云本身看起来是"对的"), 必须算。
 *
 *  为什么取"中段": 物体是圆柱+顶上方形盒, 只有圆柱那一段的横截面才是圆。取高度范围的
 *  55%~85% 那一段 —— 这段是圆柱, 盒子和近轴的内壁都不在里面。 */
struct RingFit {
    bool   ok    = false;
    double cx    = 0.0, cy    = 0.0;   // 拟合出的横截面圆心 (基准系 XY)
    double r     = 0.0;                // 拟合半径 (mm)
    double devMm = 0.0;                // 与标定轴点的距离 (mm) —— 关键指标
    double spreadMm = 0.0;             // 环本身的径向离散 (p90-p10), 用来判断这个拟合可不可信
    int    n     = 0;
};

RingFit fitCrossSectionCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              const Eigen::Vector3f& axisPoint,
                              const Eigen::Vector3f& axisDir)
{
    RingFit out;
    if (!cloud || cloud->size() < 200) return out;

    // 沿轴向的高度 (轴方向理论上≈单位向量, 但参数可能没归一化)
    Eigen::Vector3f u = axisDir.norm() > 1e-6f ? axisDir.normalized() : Eigen::Vector3f(0, 0, 1);
    std::vector<std::pair<double, const pcl::PointXYZ*>> tmp;
    tmp.reserve(cloud->size());
    for (const auto& p : cloud->points)
        tmp.emplace_back(double(u.dot(Eigen::Vector3f(p.x, p.y, p.z) - axisPoint)), &p);
    std::sort(tmp.begin(), tmp.end(),
              [](const std::pair<double, const pcl::PointXYZ*>& a,
                 const std::pair<double, const pcl::PointXYZ*>& b) { return a.first < b.first; });
    const double h0 = tmp.front().first, h1 = tmp.back().first;
    if (h1 - h0 < 5.0) return out;
    const double lo = h0 + (h1 - h0) * 0.55, hi = h0 + (h1 - h0) * 0.85;

    std::vector<cv::Point2d> band;
    for (const auto& t : tmp)
        if (t.first >= lo && t.first <= hi) band.emplace_back(t.second->x, t.second->y);
    if (band.size() < 100) return out;

    // Kasa 圆拟合 + 迭代收紧: 先用标定轴点起手取环, 再反复用新圆心重取。
    // 不直接对着"全部中段点"拟合 —— 那里面混着盒子与近轴内壁, 会把圆心拽偏。
    auto kasa = [](const std::vector<cv::Point2d>& v, cv::Point2d& c, double& r) {
        const size_t n = v.size();
        if (n < 3) return false;
        double sx = 0, sy = 0;
        for (const auto& p : v) { sx += p.x; sy += p.y; }
        sx /= n; sy /= n;
        double suu = 0, svv = 0, suv = 0, suuu = 0, svvv = 0, suvv = 0, svuu = 0;
        for (const auto& p : v) {
            const double a = p.x - sx, b = p.y - sy;
            suu += a * a; svv += b * b; suv += a * b;
            suuu += a * a * a; svvv += b * b * b; suvv += a * b * b; svuu += b * a * a;
        }
        suu /= n; svv /= n; suv /= n; suuu /= n; svvv /= n; suvv /= n; svuu /= n;
        const double det = 2.0 * (suu * svv - suv * suv);
        if (std::fabs(det) < 1e-12) return false;
        const double uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / det;
        const double vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / det;
        c = cv::Point2d(sx + uc, sy + vc);
        r = std::sqrt(uc * uc + vc * vc + suu + svv);
        return std::isfinite(c.x) && std::isfinite(c.y) && std::isfinite(r);
    };

    cv::Point2d c(axisPoint.x(), axisPoint.y());
    double r = [&] {
        std::vector<double> d;
        d.reserve(band.size());
        for (const auto& p : band) d.push_back(cv::norm(p - c));
        std::sort(d.begin(), d.end());
        return d[d.size() / 2];
    }();
    std::vector<cv::Point2d> ring;
    for (int it = 0; it < 8; ++it) {
        ring.clear();
        for (const auto& p : band)
            if (std::fabs(cv::norm(p - c) - r) < 9.0) ring.push_back(p);
        if (ring.size() < 50) return out;
        if (!kasa(ring, c, r)) return out;
    }

    std::vector<double> d;
    d.reserve(ring.size());
    for (const auto& p : ring) d.push_back(cv::norm(p - c));
    std::sort(d.begin(), d.end());

    out.ok       = true;
    out.cx       = c.x;
    out.cy       = c.y;
    out.r        = r;
    out.n        = int(ring.size());
    out.devMm    = std::hypot(c.x - axisPoint.x(), c.y - axisPoint.y());
    out.spreadMm = d[size_t(d.size() * 0.9)] - d[size_t(d.size() * 0.1)];
    return out;
}

} // namespace

// ==================== 参数快照 ====================
// 轴调试包与重建调试包共用同一份生成逻辑: 两处各写一遍迟早会因为漏改一处而
// 出现"报告里的参数和实际跑的参数对不上", 那比没有参数更糟。
QString MainWindow::buildReconParamsCsv() const
{
    // 优先用本次作业的参数快照, 而不是当前界面控件值: 用户在跑完之后动一下滑块、
    // 重框一次 ROI, 导出的"实际参数"就和被诊断的那次对不上了 —— 那是比没有参数更糟的情况。
    ReconstructionParams p = m_hasReconJob ? m_lastReconJob.params : buildReconstructionParams();
    QString csv = "参数,数值\n";
    auto row = [&](const QString& k, const QString& v) { csv += csvCell(k) + "," + csvCell(v) + "\n"; };

    row("S1 方法(0=Steger/1=重心/2=列极值)", QString::number(p.s1_method));
    row("LAB A 阈值", QString::number(p.lab_a_threshold));
    row("红色掩膜色相容差(°)", QString::number(p.laser_hue_tol_deg));
    row("红色掩膜饱和度下限", QString::number(p.laser_min_sat));
    row("Steger sigma", QString::number(p.steger_sigma, 'f', 4));
    row("Steger t_max(px)", QString::number(p.steger_t_max, 'f', 3));
    row("Steger 暗边阈值", QString::number(p.steger_edge_dark_thresh));
    row("过曝 L 阈值", QString::number(p.steger_overexposed_l_thresh));
    row("红通道饱和阈值", QString::number(p.sat_r_thresh));
    row("红通道边缘阈值", QString::number(p.edge_dark_r_thresh));
    row("最低点置信度", QString::number(p.min_point_confidence, 'f', 4));
    // ROI 必须留档: 它不只是空间裁剪 —— S1 的红色门限剖面历史上是按 ROI 行号锚定的
    // (见 extractLaserCenter 里 adaptive_split 的说明), 同一张图框不同大小的框,
    // 掩膜本身就不同。不记下来, 事后看到两份结果对不上就无从解释。
    {
        auto rectStr = [](const cv::Rect& r) {
            return (r.width > 0 && r.height > 0)
                       ? QString("x=%1 y=%2 w=%3 h=%4").arg(r.x).arg(r.y).arg(r.width).arg(r.height)
                       : QString("全图(未框选)");
        };
        row("左图 ROI", rectStr(p.roi_left));
        row("右图 ROI", rectStr(p.roi_right));
    }
    row("并行线程", QString::number(p.parallel_threads));
    row("极线容差(px)", QString::number(p.epipolar_threshold, 'f', 3));
    row("DP 跳过惩罚", QString::number(p.dp_skip_penalty, 'f', 3));
    row("DP 视差平滑", QString::number(p.dp_smooth_weight, 'f', 3));
    row("视差跳变切断阈值(px)", QString::number(p.disparity_break_threshold, 'f', 3));
    row("深度范围(mm)", QString("%1 ~ %2").arg(p.depth_min, 0, 'f', 1).arg(p.depth_max, 0, 'f', 1));
    row("S3 右图一致性阈值(mm)", QString::number(p.reproj_reject_mm, 'f', 3));
    row("使用 ICP", p.use_icp ? "是" : "否(纯轴旋转)");
    // 回环校正会改变各视角的位姿, 关/开给出的是**两份不同的点云** —— 不写进参数档就没法回溯
    row("回环校正", p.loop_closure == ReconstructionParams::LoopClosureOff ? "关闭(纯增量累加)"
                  : (p.loop_closure == ReconstructionParams::LoopClosureIcp ? "启用(ICP回环)" : "启用(均摊)")) ;
    row("ICP 最大对应距离(mm)", QString::number(p.icp_max_correspondence_distance, 'f', 3));
    row("ICP 最大迭代", QString::number(p.icp_max_iterations));
    row("ICP 轴信任度(%)", QString::number(p.icp_axis_trust, 'f', 2));
    row("SOR K / std", QString("%1 / %2").arg(p.sor_mean_k).arg(p.sor_std_dev_mul, 0, 'f', 3));
    row("体素尺寸(mm)", QString::number(p.voxel_leaf_size, 'f', 4));
    row("网格方法", p.mesh_method == 0 ? "泊松" : "GP3");
    row("泊松深度", QString::number(p.poisson_depth));
    row("泊松点权重", QString::number(p.poisson_point_weight, 'f', 3));
    row("网格截断距离(mm)", QString::number(p.mesh_truncation_distance, 'f', 3));

    // 重建侧真正依赖的标定量: 光平面直接决定 S3 的深度, 换过一次就全错, 必须留档
    row("光平面(a,b,c,d)", QString("[%1, %2, %3, %4]")
        .arg(m_LaserPlaneEquation[0], 0, 'f', 6).arg(m_LaserPlaneEquation[1], 0, 'f', 6)
        .arg(m_LaserPlaneEquation[2], 0, 'f', 6).arg(m_LaserPlaneEquation[3], 0, 'f', 6));
    row("双目标定 RMS(px)", QString::number(m_StereoRms, 'f', 4));
    row("基线(mm)", QString::number(m_T.empty() ? 0.0 : cv::norm(m_T), 'f', 3));
    row("是否立体校正", m_IsRectified ? "是" : "否");

    // S5 的三个位姿输入必须全部留档。此前只记了步长, 轴点和基准系变换都没记 ——
    // 而"点云外侧多出一圈不存在的曲面"那类问题恰恰出在轴点上, 事后只能靠拟合去猜,
    // 拟合本身又有偏差, 根本没法回溯。
    if (m_hasReconJob) {
        const Eigen::Vector3f& ap = m_lastReconJob.axisPoint;
        const Eigen::Vector3f& ad = m_lastReconJob.axisDir;
        row("旋转轴点(基准系)", QString("[%1, %2, %3]").arg(ap.x(), 0, 'f', 3)
                                                  .arg(ap.y(), 0, 'f', 3).arg(ap.z(), 0, 'f', 3));
        row("旋转轴方向(基准系)", QString("[%1, %2, %3]").arg(ad.x(), 0, 'f', 4)
                                                    .arg(ad.y(), 0, 'f', 4).arg(ad.z(), 0, 'f', 4));
        row("轴标定是否有效", m_lastReconJob.axisValid ? "是" : "否(安全降级: 轴点/方向是猜的)");
    }
    // 基准系→相机系: 没有它就无法把导出的 3D 点反投影回原图核对
    // (曾想用它定位"外圈那层点打在物体上还是轮廓外", 结果因为没有而卡住)。
    if (!m_R_base.empty() && m_R_base.rows == 3 && m_R_base.cols == 3) {
        auto m3 = [&](const cv::Mat& m) {
            cv::Mat d;
            if (m.type() == CV_32F)      m.convertTo(d, CV_64F);
            else if (m.type() == CV_64F) d = m;
            else                         return QString("(非数值矩阵)");
            QString t;
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                t += QString("%1%2").arg(j ? "," : "").arg(d.at<double>(i, j), 0, 'f', 6);
            return t;
        };
        row("R_base(基准系→相机系)", m3(m_R_base));
    }
    if (!m_T_base.empty()) {
        cv::Mat d;
        if (m_T_base.type() == CV_32F)      m_T_base.convertTo(d, CV_64F);
        else if (m_T_base.type() == CV_64F) d = m_T_base;
        if (!d.empty())
            row("T_base(基准系→相机系)", QString("[%1, %2, %3]")
                .arg(d.at<double>(0), 0, 'f', 4).arg(d.at<double>(1), 0, 'f', 4)
                .arg(d.at<double>(2), 0, 'f', 4));
    }

    // S6 底面填充会往点云里灌**合成点**, 必须留档 —— 排查"点云里有不存在的平面"时
    // 这是第一个该看的数。0 = 没造。
    if (m_reconWorker)
        row("S6 底面填充合成点数", QString::number(m_reconWorker->bottomFilledCount()));

    row("本次重建步长(°)", QString::number(m_hasReconJob ? m_lastReconJob.angleStepDeg : 0.0, 'f', 4));
    row("本次重建对数", QString::number(m_hasReconJob ? m_lastReconJob.leftPaths.size() : 0));
    return csv;
}

// ==================== 诊断报告 ====================
QString MainWindow::buildReconDiagReport(const QVector<ReconSampleStat>& samples,
                                        int imageHeight) const
{
    if (!m_reconWorker) return QString("（无重建记录）\n");
    const std::vector<ViewRecord>& recs = m_reconWorker->viewRecords();
    const ViewOutcomeStats& st = m_reconWorker->mergedStats();
    const bool cancelled = m_reconWorker->wasCancelled();

    QString s;
    s += "# 重建诊断报告\n\n";
    s += QString("- 生成时间: %1\n").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
    s += QString("- 图像序列: %1 对 ｜ 步长 %2°\n")
             .arg(recs.size()).arg(m_hasReconJob ? m_lastReconJob.angleStepDeg : 0.0, 0, 'f', 3);
    if (m_hasReconJob && !m_lastReconJob.leftPaths.isEmpty())
        s += QString("- 序列目录: %1\n").arg(QFileInfo(m_lastReconJob.leftPaths.first()).absolutePath());
    if (cancelled) s += "- ⚠️ **本次重建被取消**: 下面的数据只覆盖取消前已处理的视角\n";
    s += "\n";

    // ---- 逐阶段漏斗 ----
    long s1L = 0, s1R = 0, s2Sum = 0, s3Sum = 0;
    for (const ViewRecord& r : recs) { s1L += r.s1Left; s1R += r.s1Right; s2Sum += r.s2Match; s3Sum += r.s3Points; }
    const double yield = (s2Sum > 0) ? 100.0 * s3Sum / s2Sum : 0.0;
    const double ratioLR = (s1L > 0) ? double(s1R) / s1L : 0.0;

    s += "## 1. 结论\n\n";
    s += QString("**S2 匹配 %1 对 → S3 有效点 %2 个，产出率 %3%**")
             .arg(s2Sum).arg(s3Sum).arg(yield, 0, 'f', 1);
    if (st.reprojReject > 0 && s2Sum > 0 && double(st.reprojReject) / s2Sum > 0.5)
        s += " —— 绝大多数损失发生在 **S3 的右图一致性判据**上（见 §3）。";
    else if (yield > 0 && yield < 40)
        s += " —— 产出率偏低，见 §3 的分项。";
    else if (yield >= 60)
        s += " —— 产出率正常，没有大面积丢失。";
    else
        s += " —— 产出率偏低但不极端，结合 §4 / §6 判断。";
    s += "\n\n";

    s += "## 2. 逐阶段漏斗\n\n";
    s += "| 阶段 | 数量 | 占上一阶段 |\n|---|---|---|\n";
    s += QString("| S1 左图提取光条点 | %1 | — |\n").arg(s1L);
    s += QString("| S1 右图提取光条点 | %1 | 右/左比 **%2** |\n").arg(s1R).arg(ratioLR, 0, 'f', 2);
    long s2Dup = 0;
    for (const ViewRecord& r : recs) s2Dup += r.s2DupMerged;
    s += QString("| S2 极线匹配对数 | %1 | %2% |\n")
             .arg(s2Sum).arg(s1L > 0 ? 100.0 * s2Sum / s1L : 0.0, 0, 'f', 1);
    s += QString("| └ 其中「多个左点共用一个右点」被合并掉 | %1 | %2% |\n")
             .arg(s2Dup).arg(s2Sum + s2Dup > 0 ? 100.0 * s2Dup / (s2Sum + s2Dup) : 0.0, 0, 'f', 1);
    s += QString("| S3 有效点数 | %1 | **%2%** |\n").arg(s3Sum).arg(yield, 0, 'f', 1);
    s += QString("| 最终点云 (S5/S6 后) | %1 | — |\n")
             .arg(m_reconWorker ? m_reconWorker->resultCloud()->size() : 0);
    s += "\n";

    s += "## 3. 关键判读\n\n";
    s += "**S3 剔除分项**（四项相加 = 匹配对数 - 有效点数）\n\n";
    s += "| 剔除项 | 数量 | 占匹配对数 |\n|---|---|---|\n";
    auto rejRow = [&](const QString& name, int n) {
        s += QString("| %1 | %2 | %3% |\n").arg(name).arg(n)
                 .arg(s2Sum > 0 ? 100.0 * n / s2Sum : 0.0, 0, 'f', 1);
    };
    rejRow("光平面一致性超限", st.reprojReject);
    rejRow("交点在相机后方", st.negDepth);
    rejRow("深度越界", st.tooFar);
    rejRow("两射线接近平行", st.degenerate);
    s += "\n";
    // 只给"最粗的那根柱子"开药方。四项各写一条建议看着全面, 实际是把判断责任推回给读者
    const int rejTotal = st.reprojReject + st.negDepth + st.tooFar + st.degenerate;
    if (rejTotal <= 0) {
        s += "- 本段没有剔除记录。\n";
    } else if (st.reprojReject * 2 >= rejTotal) {
        s += "- **空间残差**占了大头（四项里的 "
             + QString::number(100.0 * st.reprojReject / rejTotal, 'f', 0) + "%）：匹配与光平面「对不上」。"
             "先看 §4 —— 若右/左提取比明显偏离 1，是一侧光条没提全；两侧都提全了才轮到怀疑光平面本身。\n";
    } else if (st.tooFar * 2 >= rejTotal) {
        s += "- **深度越界**占了大头：检查 Tab5 的「深度范围」是否把有效点挡掉了"
             "（本段观测到的深度范围是 "
             + QString("%1 ~ %2 mm").arg(st.depthMinMm, 0, 'f', 1).arg(st.depthMaxMm, 0, 'f', 1) + "）。\n";
    } else if (st.negDepth * 2 >= rejTotal) {
        s += "- **交点在相机后方**占了大头：射线方向或光平面的符号约定有问题，"
             "通常是标定被换过（比如光平面是在另一个坐标系下标的）。\n";
    } else if (st.degenerate * 2 >= rejTotal) {
        s += "- **射线与光平面平行**占了大头：光平面几乎过光心，该配置下平面无法约束深度，"
             "需要重新标定光平面。\n";
    } else {
        s += "- 四项剔除量级相当，没有单一主因；结合 §4（提取）与 §6（逐对匹配）判断。\n";
    }
    s += "\n";

    s += "## 4. 光条提取质量 (S1)\n\n";
    if (s1L > 0 && ratioLR > 0) {
        s += QString("- 左图提取 %1 点 / 右图提取 %2 点，**右/左 = %3**（两侧看的是同一条光条，理应接近 1）。\n")
                 .arg(s1L).arg(s1R).arg(ratioLR, 0, 'f', 2);
        // 注意这个比值本身就偏保守: 两台相机看光条的视角不同, 端头被遮挡掉一截是正常的,
        // 0.75~1.0 属于健康范围 (本工程实测"掩膜完好"时该比值也在 0.77 左右)。
        // 所以只在**极端**不对称时才报警, 真正的判据是 `overlays/` 里的叠加图。
        if (ratioLR < 0.5 || ratioLR > 2.0) {
            s += "  ⚠️ 两侧差了 2 倍以上，说明光条在一侧被整段判掉了 —— 最常见的原因是"
                 "红光掩膜的色相窗太窄：光条越亮越白，色相会从暗处的 ~174°（OpenCV 0-180 标度）"
                 "漂到亮处的 ~155°，而掩膜只认 170~180° 时就只能提出暗处那一段。"
                 "请检查 Tab5 的「红色色相容差 / 饱和度下限」，并对照 `overlays/` 里的叠加图。\n";
        } else {
            s += "  两侧在同一量级（端头遮挡导致的差异是正常的），**但比值正常不代表提全了** ——"
                 "两侧同时丢一段时光看比值发现不了。要看 `overlays/` 里样本帧的叠加图："
                 "绿点应当连续覆盖整条光条。\n";
        }
    }
    // 饱和恢复占比: 点是"脊线拟合"来的还是"两侧边缘反推"来的, 决定的精度不一样。
    // 激光削顶(红通道)时脊线在平台区是没有意义的, 只能靠边缘中点。这个比例必须看得见。
    {
        long satL = 0, satR = 0;
        for (const ViewRecord& r : recs) { satL += r.s1LeftSatRecover; satR += r.s1RightSatRecover; }
        if (satL + satR > 0) {
            s += QString("- **饱和恢复点占比**：左 %1/%2 (%3%) / 右 %4/%5 (%6%)。"
                         "激光削顶处无法用脊线定中心，这些点的中心取自两侧边缘的中点。\n")
                     .arg(satL).arg(s1L).arg(s1L > 0 ? 100.0 * satL / s1L : 0.0, 0, 'f', 0)
                     .arg(satR).arg(s1R).arg(s1R > 0 ? 100.0 * satR / s1R : 0.0, 0, 'f', 0);
            if (s1L > 0 && double(satL) / s1L > 0.9)
                s += "  ⚠️ 九成以上都靠恢复反推：多半是曝光偏高或激光功率偏大，"
                     "光条几乎整条削顶。此时中心完全由边缘决定，精度对边缘阈值很敏感。\n";
        }
    }
    // 行范围/间隙: 只看点数分不出"整条稀疏"和"中间断了一截"
    int gapL = 0, gapR = 0; double avgRowSpanL = 0, avgRowSpanR = 0; int nSpan = 0;
    int rminL = 1 << 30, rmaxL = -1, rminR = 1 << 30, rmaxR = -1;
    for (const ViewRecord& r : recs) {
        if (r.maxGapLeft  > gapL) gapL = r.maxGapLeft;
        if (r.maxGapRight > gapR) gapR = r.maxGapRight;
        if (r.rowMinLeft >= 0 && r.rowMaxLeft >= 0) {
            avgRowSpanL += r.rowMaxLeft - r.rowMinLeft; ++nSpan;
            rminL = std::min(rminL, r.rowMinLeft);  rmaxL = std::max(rmaxL, r.rowMaxLeft);
        }
        if (r.rowMinRight >= 0 && r.rowMaxRight >= 0) {
            avgRowSpanR += r.rowMaxRight - r.rowMinRight;
            rminR = std::min(rminR, r.rowMinRight); rmaxR = std::max(rmaxR, r.rowMaxRight);
        }
    }
    if (nSpan > 0) {
        s += QString("- 提取点在原图上的行跨度：左图平均 %1 行 / 右图平均 %2 行；"
                     "全部帧的行范围并集：左 [%3, %4] / 右 [%5, %6]；"
                     "最大的单段空缺：左 %7 行 / 右 %8 行。\n")
                 .arg(avgRowSpanL / nSpan, 0, 'f', 0).arg(avgRowSpanR / nSpan, 0, 'f', 0)
                 .arg(rminL).arg(rmaxL).arg(rminR).arg(rmaxR).arg(gapL).arg(gapR);
        /**
         * 大空缺有两种完全不同的成因，必须分开说 —— 否则会误导：
         *   ① 提取点**铺满整幅图**（行范围接近图高、起点贴到 0），说明掩膜把背景里的红色
         *      物体也算成了光条；此时那个"空缺"只是背景杂点到真光条之间的距离，光条本身
         *      可能一点没丢。解决办法是框 ROI，不是调掩膜。
         *   ② 行范围本身正常（就是光条的长度），但中间断了一截 —— 那才是真丢了光条中段。
         */
        const bool spansWholeImage = (imageHeight > 0)
            && (double(std::max(rmaxL - rminL, rmaxR - rminR)) > 0.9 * imageHeight);
        const int gapThresh = 80;   // 一行像素约 0.5mm; 连续 80 行没有点已经明显不正常
        if (spansWholeImage) {
            s += QString("  ⚠️ 提取点的行范围几乎铺满整幅图（图高 %1），而光条本身不可能贯穿全图 ——"
                         " 掩膜多半把**背景里的红色物体**也算进来了。到 `overlays/` 核对："
                         "若绿点除光条之外还有一簇游离在背景上，就是这种情况。\n"
                         "  处理：在 Tab5 用「左图/右图 ROI 框选」把背景排掉即可。"
                         "这些杂点虽然会被 S3 的几何判据剔掉，但会挤占 S2 的匹配路径、"
                         "并让上面那个「空缺」数字虚高。\n").arg(imageHeight);
        } else if (gapL > gapThresh || gapR > gapThresh) {
            s += QString("  ⚠️ 存在超过 %1 行的连续空缺（左 %2 / 右 %3）：光条中段丢了一截。"
                         "该段多半是过曝成白（色相漂出掩膜窗）或被遮挡；"
                         "到 `overlays/` 看一眼就能确认是哪种。\n").arg(gapThresh).arg(gapL).arg(gapR);
        } else {
            s += "  没有出现大段空缺，光条是连续的。\n";
        }
    }
    s += QString("- 逐帧数值见 `02_逐帧产出.csv`；`overlays/` 下有样本帧的原图 + 提取点叠加图，可直接看图核对。\n");
    s += "\n";

    s += "## 5. 右图重投影误差分布\n\n";
    {
        const int* h = st.reprojHist;
        const int total = h[0]+h[1]+h[2]+h[3]+h[4]+h[5]+h[6]+h[7];
        if (total > 0) {
            s += "| 桶 | 数量 | 占比 |\n|---|---|---|\n";
            for (int b = 0; b < 8; ++b)
                s += QString("| %1 | %2 | %3% |\n").arg(kHistHeads[b]).arg(h[b])
                         .arg(100.0 * h[b] / total, 0, 'f', 1);
            s += "\n";
            // 先算"紧贴阈值"与"远离阈值"的量, 而不是把两种建议都念一遍
            const int huge = h[6] + h[7];          // 50px 以上: 几何上不成立
            const int tiny = h[0] + h[1] + h[2];   // 5px 以内: 基本正确
            s += QString("- 5px 以内 %1%（基本正确）、50px 以上 %2%（几何上不成立）。\n")
                     .arg(100.0 * tiny / total, 0, 'f', 1).arg(100.0 * huge / total, 0, 'f', 1);
            // 三档: 误差"远在阈值之外"(错配) / "紧贴阈值"(阈值或光平面略偏) / "基本干净"
            if (huge > 2 * tiny) {
                s += "  → **成片落在阈值之外**：这些匹配在几何上根本不成立（伪匹配），"
                     "放宽阈值只会把错点放进去。应先解决 §4 的光条提取，再回来调阈值。\n";
            } else if (tiny > huge) {
                s += "  → 大部分匹配是干净的。\n";
            } else {
                s += "  → 质量主要挤在阈值附近那一档：属于光平面略有偏差、或阈值偏紧、"
                     "或工作距离过近，可小幅放宽 Tab5 的「右图一致性阈值」，"
                     "更彻底的办法是重标光平面。结合 §6 的「几何自洽比例」确认。\n";
            }
        } else {
            s += "（无数据）\n";
        }
    }
    s += "\n";

    s += "## 6. 样本帧逐对匹配验证\n\n";
    if (samples.isEmpty()) {
        s += "（未采集：样本帧重跑失败或序列不可读）\n";
    } else {
        s += "对下列样本帧重跑了 S1~S3，逐对匹配的几何量见 `04_样本帧匹配明细.csv`。\n\n";
        s += "| 帧号 | 角度(°) | S1左/右 | S2 | S3 | 产出率 | 几何自洽比例 | DLT平面残差中位数(mm) |\n";
        s += "|---|---|---|---|---|---|---|---|\n";
        for (const ReconSampleStat& q : samples) {
            s += QString("| %1 | %2 | %3/%4 | %5 | %6 | %7% | %8% | %9 |\n")
                     .arg(q.frameIdx).arg(q.angleDeg, 0, 'f', 2)
                     .arg(q.s1L).arg(q.s1R).arg(q.s2).arg(q.s3)
                     .arg(q.s2 > 0 ? 100.0 * q.s3 / q.s2 : 0.0, 0, 'f', 1)
                     .arg(q.nMatch > 0 ? 100.0 * q.nGood / q.nMatch : 0.0, 0, 'f', 1)
                     .arg(q.residMedianMm >= 0 ? QString::number(q.residMedianMm, 'f', 2) : QString("—"));
        }
        s += "\n";
        s += "「几何自洽」= 双目 DLT 三角化点到光平面的距离 ≤ 5mm。**这个量与 S3 的判据无关**，"
             "是用来反查判据的独立真值：\n";
        s += "- 自洽比例高、但产出率低 → 判据（阈值）太紧，是关键路径；\n";
        s += "- 自洽比例本身很低 → 这些匹配在几何上就不成立，放宽阈值没用，得回到 §4；\n";
        s += "- 若某帧「有自洽候选」的比例高却仍被大量剔除，才需要去查 S2 的匹配策略。\n";
    }
    s += "\n";

    s += "## 7. S5 位姿核对（步长 / 轴点）\n\n";
    s += QString("- 本次重建使用的步长: **%1°**（Tab5「每视角转角」）。\n")
             .arg(m_hasReconJob ? m_lastReconJob.angleStepDeg : 0.0, 0, 'f', 4);
    if (m_axisDiag.valid && m_axisDiag.stepMeanDeg > 0) {
        const double rel = m_hasReconJob && m_lastReconJob.angleStepDeg > 0
                               ? 100.0 * std::fabs(m_lastReconJob.angleStepDeg - m_axisDiag.stepMeanDeg)
                                     / m_lastReconJob.angleStepDeg
                               : -1.0;
        s += QString("- 转台标定序列实测步长: **%1°**（标准差 %2°，累计转角 %3°）。\n")
                 .arg(m_axisDiag.stepMeanDeg, 0, 'f', 4).arg(m_axisDiag.stepStdDeg, 0, 'f', 4)
                 .arg(m_axisDiag.angleSpanDeg, 0, 'f', 1);
        if (rel >= 0) {
            s += QString("- 相对偏差: **%1%**。").arg(rel, 0, 'f', 1);
            if (rel > 5.0) {
                s += " ⚠️ 超过 5%，**必须核对**：S5 若未启用 ICP，整个点云完全靠 `i×步长` 的"
                     "纯轴旋转摆放，步长错多少整圈就累积错多少（本段 N 帧即累积 "
                     + QString("%1°").arg(std::fabs(m_lastReconJob.angleStepDeg - m_axisDiag.stepMeanDeg)
                                          * m_lastReconJob.leftPaths.size(), 0, 'f', 0) + "）。\n";
            } else {
                s += " 在 5% 以内，可接受。\n";
            }
        }
        s += "- 注意：上面的实测值来自**转台标定序列**。若重建序列是另一次采集，"
             "两者的实际步长未必相同 —— 以重建序列自身的实测为准。\n";
    } else {
        s += "- 还没有可用的转台轴诊断结果，无法核对（到 Tab6「运行转台标定诊断」跑一次即可）。\n";
    }
    if (!m_hasReconJob || !m_lastReconJob.params.use_icp)
        s += "- ⚠️ 本次重建**未启用 ICP**（Tab5「使用 ICP」未勾选）：S5 只做纯轴旋转，"
             "不做任何数据驱动的修正，因此步长与轴参数的误差会原样进入点云。\n";
    // 回环校正与步长之间有个反直觉的耦合, 必须写清楚: 开着它时步长偏差会被整圈约束
    // 悄悄补偿掉一部分, 于是"步长错了"这件事在看点云时反而不明显。
    if (m_hasReconJob && m_lastReconJob.params.loop_closure == ReconstructionParams::LoopClosureOff) {
        s += "- 本次**回环校正已关闭**（Tab5「启用回环校正」未勾选）：各视角位姿纯增量累加，"
             "点云反映的是标定与步长的**原始**结果 —— 排查参数问题时这是正确的基线。\n";
    } else if (m_hasReconJob) {
        s += "- ⚠️ 本次**启用了回环校正**：整圈累计漂移已沿标定轴摊回各帧。若步长本身有偏差，"
             "它会被整圈约束补偿掉一部分，**点云看起来对、实际尺寸不对** —— "
             "怀疑步长时请关掉重跑一次作对照。\n";
    }

    // ---- 轴点自检: 融合后的横截面圆心应当精确落在标定轴点上 ----
    if (m_reconWorker) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr fin = m_reconWorker->resultCloud();
        Eigen::Vector3f ap = m_hasReconJob ? m_lastReconJob.axisPoint
                                           : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f ad = m_hasReconJob ? m_lastReconJob.axisDir
                                           : Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        const RingFit rf = fitCrossSectionCenter(fin, ap, ad);
        s += "\n**轴点自检**（融合后的横截面圆心 vs 标定轴点）\n\n";
        if (!rf.ok) {
            s += "- 点云太小或缺轴参数，没算出来。\n";
        } else {
            s += QString("- 圆柱中段拟合圆心: **(%1, %2)**，半径 %3 mm，用掉 %4 个点"
                         "（环本身径向离散 p90−p10 = %5 mm）。\n")
                     .arg(rf.cx, 0, 'f', 2).arg(rf.cy, 0, 'f', 2).arg(rf.r, 0, 'f', 1)
                     .arg(rf.n).arg(rf.spreadMm, 0, 'f', 1);
            s += QString("- 标定轴点: **(%1, %2)**。\n")
                     .arg(ap.x(), 0, 'f', 2).arg(ap.y(), 0, 'f', 2);
            // 判据不能只看"偏差 > 2mm": 物体在转台上**单纯装偏**同样会让偏差很大,
            // 而那种情况下点云本身是对的, 不该让用户去重标定。
            //
            // 而且这个自检**只能在小环厚时**才是在量轴点。理由:
            //   绕轴旋转**保持到轴的距离不变** ⇒ 单帧的径向铺开量, 合并后原样保留。
            //   所以只要逐视角数据(05_逐视角点云.csv)在单帧里就已经铺开了,
            //   合并后的"厚环"就与 S5 用什么轴、轴点准不准**无关** —— 那是 S1/S2/S3 的事。
            //   实测: 视角 0 在 h85-90 这一格里 r 就从 49.1 铺到 58.0 (该帧方位角只跨 4.7°)。
            // 因此这里**不能**由"环厚"反推"轴点错", 那会把排查引到错误的方向。
            s += QString("- 偏差: **%1 mm**。").arg(rf.devMm, 0, 'f', 2);
            if (rf.devMm <= 2.0) {
                s += " 在 2mm 以内，配准的轴点没有问题。\n";
            } else if (rf.spreadMm <= 3.0) {
                s += QString(" 超过 2mm，但**环本身很薄（%1 mm）**。\n"
                             "  → 此时这个圆心是可信的：物体在转台上装偏了 %2 mm，**不是轴点标定错**。\n"
                             "     各视角反旋转后精确重合，点云本身是对的，只是整体偏离了转轴。**不必处理。**\n")
                         .arg(rf.spreadMm, 0, 'f', 1).arg(rf.devMm, 0, 'f', 1);
            } else {
                s += QString(" 超过 2mm，**但环本身铺开了 %1 mm —— 所以这个圆心不可信，"
                             "上面的偏差不能当成「轴点错了」。**\n"
                             "  → 绕轴旋转保持到轴的距离不变，单帧铺开多少，合并后就是多少。\n"
                             "     环厚说明**逐视角数据本身就铺开了**（见 `05_逐视角点云.csv`：\n"
                             "     单帧、单个 5mm 高度格内 r 就横跨 10mm 左右，而整帧方位角只跨约 5°）。\n"
                             "  → **这与 S5 用哪根轴无关，去查 S1/S2/S3（光条宽度、光平面、匹配），"
                             "不要急着重标定轴点。**\n")
                         .arg(rf.spreadMm, 0, 'f', 1);
            }
        }
    }
    s += "\n";

    s += "## 8. 文件说明\n\n";
    s += "| 文件 | 内容 |\n|---|---|\n";
    s += "| `02_逐帧产出.csv` | 每个视角的 S1/S2/S3 计数、各类剔除、掩膜像素、光条行范围与最大空缺 |\n";
    s += "| `03_逐帧重投影直方图.csv` | 每个视角的右图重投影误差 8 桶分布（看损失是均匀还是集中） |\n";
    s += "| `04_样本帧匹配明细.csv` | 样本帧**每一对匹配**的视差/深度/像素残差/空间残差/DLT 平面残差/拒绝原因 |\n";
    s += "| `05_逐视角点云.csv` | 每个视角的 3D 点（转台基准系，**未施加视角旋转**），可离线做配准/步长分析 |\n";
    s += "| `06_重建参数.csv` | 本次重建实际使用的参数与关键标定量（光平面、旋转轴点/方向、R_base/T_base） |\n";
    s += "| `07_重建日志.txt` | Tab5 重建日志全文 |\n";
    s += "| `08_最终点云.ply` | S5/S6 后的最终点云（二进制） |\n";
    s += "| `overlays/` | 样本帧原图 + S1 提取点叠加（绿点），可直接看出光条提出到哪一段 |\n";
    return s;
}

// ==================== 导出 ====================
bool MainWindow::exportReconDebugPackage(const QString& dir, QString* err,
                                         const std::function<void(const QString&, int)>& progress)
{
    // 各阶段占的百分比区间。样本帧重跑 (要跑 S1~S3) 是大头, 逐视角点云次之,
    // 其余几个文件都是纯序列化。这个分配只影响进度条的观感, 不影响正确性。
    const int kPctFrameTable = 4, kPctCloudEnd = 30, kPctMiscEnd = 34;
    const int kPctOverlayEnd = 97;   // 样本帧重跑与叠加图共同占用 [kPctMiscEnd, kPctOverlayEnd]
    auto report = [&](const QString& stage, int pct) {
        if (progress) progress(stage, pct);
    };

    if (!m_reconWorker || m_reconWorker->viewRecords().empty()) {
        if (err) *err = "还没有可导出的重建记录，请先在 Tab5（或 Tab6 的完整重建测试）跑一次重建";
        return false;
    }
    if (!m_hasReconJob) {
        if (err) *err = "缺少本次重建的作业快照，无法按同一套参数重跑样本帧";
        return false;
    }
    if (!QDir().mkpath(dir)) { if (err) *err = QString("无法创建目录 %1").arg(dir); return false; }
    report("准备中", 0);

    auto writeText = [&](const QString& name, const QString& content, bool utf8Bom = false) -> bool {
        QFile f(QDir(dir).filePath(name));
        if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            if (err) *err = QString("无法写入 %1").arg(f.fileName());
            return false;
        }
        QTextStream ts(&f);
        ts.setCodec("UTF-8");
        ts.setGenerateByteOrderMark(utf8Bom);
        ts << content;
        ts.flush();
        f.close();
        return true;
    };

    const std::vector<ViewRecord>& recs = m_reconWorker->viewRecords();

    // ---------- 02 逐帧产出 ----------
    {
        // 「掩膜像素」已是**扣除伪连通域之后**的数; 紧邻的「掩膜剔除」是被扣掉的那部分。
        // 两者相加 = 筛选前的原始掩膜。分两列是为了让"伪光条剔除动过多少手"始终可见 ——
        // 一段会默默改变结果的筛选, 不能只留一个合并后的数字。
        QString csv = "帧号,角度(deg),S1左,S1右,左掩膜像素,右掩膜像素,左掩膜剔除,右掩膜剔除,"
                      "左行起,左行止,左最大空缺,右行起,右行止,右最大空缺,"
                      "左饱和恢复点,右饱和恢复点,"
                      "S2匹配,S2右点复用合并,S3有效点,平面残差剔除,负深度,深度越界,射线平行,产出率(%),状态\n";
        for (const ViewRecord& r : recs) {
            const double y = (r.s2Match > 0) ? 100.0 * r.s3Points / r.s2Match : 0.0;
            csv += QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16,%17,%18,%19,%20,%21,%22,%23,%24,%25\n")
                       .arg(r.frameIdx).arg(r.angleDeg, 0, 'f', 3)
                       .arg(r.s1Left).arg(r.s1Right).arg(r.s1LeftMask).arg(r.s1RightMask)
                       .arg(r.s1LeftMaskDropped).arg(r.s1RightMaskDropped)
                       .arg(r.rowMinLeft).arg(r.rowMaxLeft).arg(r.maxGapLeft)
                       .arg(r.rowMinRight).arg(r.rowMaxRight).arg(r.maxGapRight)
                       .arg(r.s1LeftSatRecover).arg(r.s1RightSatRecover)
                       .arg(r.s2Match).arg(r.s2DupMerged).arg(r.s3Points).arg(r.reprojReject)
                       .arg(r.negDepth).arg(r.tooFar).arg(r.degenerate)
                       .arg(y, 0, 'f', 1)
                       .arg(r.ok ? "正常" : "零产出");
        }
        if (!writeText("02_逐帧产出.csv", csv, true)) return false;
    }
    report("逐帧产出表", kPctFrameTable);

    // ---------- 03 逐帧重投影直方图 ----------
    {
        QString csv = "帧号";
        for (const char* h : kHistHeads) csv += QString(",%1").arg(h);
        csv += "\n";
        for (const ViewRecord& r : recs) {
            csv += QString::number(r.frameIdx);
            for (int b = 0; b < 8; ++b) csv += QString(",%1").arg(r.reprojHist[b]);
            csv += "\n";
        }
        if (!writeText("03_逐帧重投影直方图.csv", csv, true)) return false;
    }

    // ---------- 05 逐视角点云 ----------
    // 这份是包里最大的文件 (每帧几百点 × 上百帧 = 十万行量级), 所以**直接流式写盘**,
    // 不在内存里攒一个几十 MB 的 QString —— 顺带也让进度能按点汇报。
    {
        report("逐视角点云", kPctFrameTable);
        const auto& clouds = m_reconWorker->viewClouds();
        QFile f(QDir(dir).filePath("05_逐视角点云.csv"));
        if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            if (err) *err = QString("无法写入 %1").arg(f.fileName());
            return false;
        }
        QTextStream ts(&f);
        ts.setCodec("UTF-8");
        ts.setGenerateByteOrderMark(true);   // Excel 打开中文表头需要
        // 必须用 QString 而不是裸的 const char*: QTextStream::operator<<(const char*)
        // 在 Qt5 里按 **Latin-1** 解释输入 (只有 QString 才走上面设的 UTF-8 codec),
        // 中文表头会被二次编码成乱码 —— 而数据列全是数字, 只有表头坏, 极易被当成
        // "Excel 的编码设置问题" 而查不出来。
        ts << QStringLiteral("视角号,角度(deg),x,y,z\n");
        const size_t nView = std::min(clouds.size(), recs.size());
        for (size_t i = 0; i < nView; ++i) {
            if (!clouds[i]) continue;
            for (const auto& p : clouds[i]->points)
                ts << recs[i].frameIdx << ',' << QString::number(recs[i].angleDeg, 'f', 3) << ','
                   << QString::number(p.x, 'f', 4) << ',' << QString::number(p.y, 'f', 4) << ','
                   << QString::number(p.z, 'f', 4) << '\n';
            // 每 1/8 报一次: 报得太密会让 processEvents 抢走比写盘还多的时间
            if (nView >= 8 && (i % (nView / 8) == 0))
                report("逐视角点云", kPctFrameTable
                       + (kPctCloudEnd - kPctFrameTable) * static_cast<int>(i) / static_cast<int>(nView));
        }
        ts.flush();
        f.close();
    }

    // ---------- 06 参数快照 / 07 日志 ----------
    report("参数快照与日志", kPctCloudEnd);
    if (!writeText("06_重建参数.csv", buildReconParamsCsv(), true)) return false;
    if (txtReconLog) {
        if (!writeText("07_重建日志.txt", txtReconLog->toPlainText())) return false;
    }

    // ---------- 08 最终点云 ----------
    {
        report("最终点云", kPctMiscEnd);
        pcl::PointCloud<pcl::PointXYZ>::Ptr c = m_reconWorker->resultCloud();
        if (c && !c->empty()) {
            const QString ply = QDir(dir).filePath("08_最终点云.ply");
            // 二进制: ASCII 的 PLY 在十万级点云上体积会翻好几倍
            if (pcl::io::savePLYFileBinary(ply.toStdString(), *c) != 0) {
                if (err) *err = QString("无法写入 %1").arg(ply);
                return false;
            }
        }
    }

    // ---------- 样本帧选择 ----------
    // 优先挑「产出率最低」的帧 —— 诊断要在最坏的地方做, 平均帧看不出病因。
    // 再补一帧产出率最高的作对照, 否则无法区分"这一帧特殊"还是"整段都这样"。
    const int kMaxSamples = 6;
    QVector<int> sampleIdx;
    {
        QVector<int> byYield;
        for (int i = 0; i < static_cast<int>(recs.size()); ++i)
            if (recs[i].s2Match > 0) byYield.push_back(i);
        std::sort(byYield.begin(), byYield.end(), [&](int a, int b) {
            return double(recs[a].s3Points) / recs[a].s2Match
                 < double(recs[b].s3Points) / recs[b].s2Match;
        });
        for (int i = 0; i < byYield.size() && sampleIdx.size() < kMaxSamples - 1; ++i)
            if (!sampleIdx.contains(byYield[i])) sampleIdx.push_back(byYield[i]);
        if (!byYield.isEmpty() && !sampleIdx.contains(byYield.back()))
            sampleIdx.push_back(byYield.back());   // 对照帧
        std::sort(sampleIdx.begin(), sampleIdx.end());
    }

    // ---------- 样本帧重跑 + 叠加图 + 04 明细 ----------
    QVector<ReconSampleStat> samples;
    int probeImageHeight = 0;   // 样本帧的原图高度 (给报告判断"是否铺满整图")
    QString matchCsv = "帧号,序号,左u,左v,右u,右v,视差(px),深度(mm),像素残差(px),"
                       "空间残差(mm),DLT平面残差(mm),是否接受,拒绝原因\n";
    {
        const QString ovDir = QDir(dir).filePath("overlays");
        QDir().mkpath(ovDir);
        CvThreadGuard guard;   // 与并行重建时的 OpenCV 线程设置对齐, 见结构体说明

        PointCloudBuilder probe;
        probe.setCalibrationData(m_lastReconJob.calibData);
        probe.setReconstructionParams(m_lastReconJob.params);

        // 每个样本帧占两个进度格: 先重跑 S1~S3, 再画叠加图。
        // 必须把它们摊在**同一条递增的格序列**上 —— 曾把叠加图报在区间高位(88%+)、
        // 下一帧的重跑又回到低位(43%), 进度条会来回跳, 看着像卡死在反复重试。
        const int nSample = std::max(1, static_cast<int>(sampleIdx.size()));
        auto samplePct = [&](int step) {
            return kPctMiscEnd + (kPctOverlayEnd - kPctMiscEnd) * step / (2 * nSample);
        };
        for (int si = 0; si < sampleIdx.size(); ++si) {
            const int fi = sampleIdx[si];
            // 重跑是整个导出里最慢的一段 (每帧都要走完整的 S1~S3), 所占进度区间也最大
            report(QString("重跑样本帧 %1/%2").arg(si + 1).arg(nSample), samplePct(2 * si));
            if (fi < 0 || fi >= m_lastReconJob.leftPaths.size()
                || fi >= m_lastReconJob.rightPaths.size()) continue;
            cv::Mat imgL = cv::imread(m_lastReconJob.leftPaths[fi].toStdString());
            cv::Mat imgR = cv::imread(m_lastReconJob.rightPaths[fi].toStdString());
            if (imgL.empty() || imgR.empty()) continue;

            ViewDiag diag;
            const double ang = fi * m_lastReconJob.angleStepDeg;
            probe.processSingleView(imgL, imgR, ang, fi, &diag);

            ReconSampleStat qs;
            qs.frameIdx = fi;
            qs.angleDeg = ang;
            qs.ran = true;
            if (probeImageHeight <= 0) probeImageHeight = imgL.rows;
            qs.s1L = static_cast<int>(diag.ptsLeftRaw.size());
            qs.s1R = static_cast<int>(diag.ptsRightRaw.size());
            qs.s2 = static_cast<int>(diag.matches.size());
            // 重跑得到的 S3 有效点数取自该 builder 的最后一条 records (就是刚跑的这一帧)
            if (!probe.viewRecords().empty()) qs.s3 = probe.viewRecords().back().s3Points;

            QVector<double> resids;
            for (int k = 0; k < static_cast<int>(diag.matches.size()); ++k) {
                const MatchDiag& m = diag.matches[k];
                if (m.accepted) ++qs.nAccepted;
                if (m.dltPlaneResidMm >= 0) {
                    ++qs.nMatch;
                    resids.push_back(m.dltPlaneResidMm);
                    if (m.dltPlaneResidMm <= 5.0) ++qs.nGood;
                }
                matchCsv += QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13\n")
                                .arg(fi).arg(k)
                                .arg(m.left.x, 0, 'f', 2).arg(m.left.y, 0, 'f', 2)
                                .arg(m.right.x, 0, 'f', 2).arg(m.right.y, 0, 'f', 2)
                                .arg(m.dispPx, 0, 'f', 2).arg(m.depthMm, 0, 'f', 3)
                                .arg(m.reprojPx, 0, 'f', 3).arg(m.reprojMm, 0, 'f', 3)
                                .arg(m.dltPlaneResidMm, 0, 'f', 3)
                                .arg(m.accepted ? "是" : "否")
                                .arg(csvCell(rejectReasonName(m.rejectReason)));
            }
            if (!resids.isEmpty()) {
                std::sort(resids.begin(), resids.end());
                qs.residMedianMm = resids[resids.size() / 2];
            }
            samples.push_back(qs);

            // 叠加图 (JPEG 而不是 PNG: 原图 1280x720, PNG 一张上兆, 包里十几张就太臃肿)
            report(QString("生成叠加图 %1/%2").arg(si + 1).arg(nSample), samplePct(2 * si + 1));
            for (int side = 0; side < 2; ++side) {
                const cv::Mat& src = (side == 0) ? imgL : imgR;
                const std::vector<cv::Point2f>& pts = (side == 0) ? diag.ptsLeftRaw : diag.ptsRightRaw;
                cv::Mat canvas = src.clone();
                for (const cv::Point2f& p : pts)
                    cv::circle(canvas, cv::Point(cvRound(p.x), cvRound(p.y)), 2,
                               cv::Scalar(0, 255, 0), -1);
                // cv::putText 的 Hershey 字库不支持中文, 图内文字一律 ASCII
                const std::string cap =
                    QString("frame %1  S1 pts: %2").arg(fi).arg(pts.size()).toStdString();
                cv::putText(canvas, cap, cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                            cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
                cv::putText(canvas, cap, cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                            cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                const QString fn = QDir(ovDir).filePath(
                    QString("frame%1_%2.jpg").arg(fi, 4, 10, QChar('0')).arg(side == 0 ? "L" : "R"));
                std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 90};
                cv::imwrite(fn.toStdString(), canvas, params);
            }
        }
    }
    if (!writeText("04_样本帧匹配明细.csv", matchCsv, true)) return false;

    // ---------- 01 报告 (必须放在样本帧重跑之后: 报告里要嵌入样本结论) ----------
    report("生成诊断报告", kPctOverlayEnd);
    if (!writeText("01_重建诊断报告.md", buildReconDiagReport(samples, probeImageHeight))) return false;

    report("导出完成", 100);
    return true;
}

// ==================== UI 包装 ====================
void MainWindow::onExportReconDebugPackage()
{
    if (!m_reconWorker || m_reconWorker->viewRecords().empty()) {
        QMessageBox::information(this, "无法导出",
            "还没有可导出的重建记录。\n请先在 Tab5 跑一次重建，或到 Tab6 点「完整重建测试」。");
        return;
    }

    const QString parent = QFileDialog::getExistingDirectory(this, "选择导出目录", dirDebug);
    if (parent.isEmpty()) return;
    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString dir = QDir(parent).filePath("recon_debug_" + stamp);
    if (!QDir().mkpath(dir)) {
        QMessageBox::warning(this, "导出失败", QString("无法创建目录:\n%1").arg(dir));
        return;
    }

    // 导出期间禁用入口按钮, 避免重入 (下面每条退出路径都要恢复 ——
    // 漏一条就会出现"导出一次之后再也不能导出"的假死)
    if (btnReconDebugExport) btnReconDebugExport->setEnabled(false);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (progressReconDebug) {
        progressReconDebug->setRange(0, 100);
        progressReconDebug->setValue(0);
        progressReconDebug->setFormat("准备中 0%");
    }
    QApplication::processEvents();

    QString err;
    const bool ok = exportReconDebugPackage(dir, &err,
        [this](const QString& stage, int percent) {
            if (!progressReconDebug) return;
            progressReconDebug->setValue(percent);
            progressReconDebug->setFormat(QString("%1  %2%").arg(stage).arg(percent));
            // 同步执行时必须主动泵事件, 否则重绘要等整个导出跑完才发生。
            // 排除用户输入: 防止进度条重绘期间又点一次按钮造成重入。
            QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        });

    QApplication::restoreOverrideCursor();
    if (progressReconDebug) {
        progressReconDebug->setValue(ok ? 100 : 0);
        progressReconDebug->setFormat(ok ? "导出完成 100%" : "导出失败");
    }
    updateDebugStatus();   // 恢复按钮的可用状态 (以本次记录是否还在为准)

    if (!ok) {
        appendDebugLog(QString("❌ 重建调试包导出失败: %1").arg(err));
        QMessageBox::warning(this, "导出失败", err);
        return;
    }
    appendDebugLog(QString("✅ 重建调试包已导出: %1").arg(dir));
    appendDebugLog("   含: 逐帧产出 / 逐帧直方图 / 样本帧匹配明细 / 逐视角点云 / 参数 / 日志 / 最终点云 / overlays");
    QMessageBox::information(this, "导出完成",
        QString("重建调试包已导出到:\n%1\n\n"
                "其中 01_重建诊断报告.md 是入口，先看它的 §1 结论与 §3 关键判读。").arg(dir));
}
