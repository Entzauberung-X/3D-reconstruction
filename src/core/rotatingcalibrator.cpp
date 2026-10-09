#include "core/rotatingcalibrator.h"
#include "ui/logger.h"
#include <iostream>
#include <cmath>   // 【补全】std::acos / std::atan2 / std::sqrt / std::fabs
#include <algorithm> // 【补全】std::min / std::max
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Calib {

namespace {
/** 旋转/平移向量 (Vec3d) → solvePnP 惯用的 3x1 CV_64F cv::Mat */
inline cv::Mat vec3dToMat(const cv::Vec3d& v)
{
    cv::Mat m(3, 1, CV_64F);
    m.at<double>(0) = v[0]; m.at<double>(1) = v[1]; m.at<double>(2) = v[2];
    return m;
}
} // namespace

struct RotatingCalibCostFunctor {
    const std::vector<cv::Point3f>& obj_pts;
    const std::vector<cv::Point2f>& obs_l;
    const std::vector<cv::Point2f>& obs_r;
    Eigen::Matrix3d K_L, K_R, R_LR;
    Eigen::Vector3d T_LR;
    Eigen::Matrix<double, 5, 1> D_L, D_R;
    int angle_idx;

    // 投影用的 OpenCV 参数在构造时建一次。
    // computeResiduals 每帧每迭代要被调用 ~12 次 (数值雅可比的 11 个扰动 + 基准),
    // 若每次都重建这 4 个 cv::Mat, 就是 4 次堆分配 × 192k 次调用 —— 纯浪费。
    cv::Mat cvK_L, cvD_L, cvK_R, cvD_R;

    RotatingCalibCostFunctor(const std::vector<cv::Point3f>& obj,
                             const std::vector<cv::Point2f>& ol,
                             const std::vector<cv::Point2f>& orr,
                             const Eigen::Matrix3d& KL, const Eigen::Matrix3d& KR,
                             const Eigen::Matrix3d& RLR, const Eigen::Vector3d& TLR,
                             const Eigen::Matrix<double,5,1>& DL,
                             const Eigen::Matrix<double,5,1>& DR,
                             int aidx)
        : obj_pts(obj), obs_l(ol), obs_r(orr), K_L(KL), K_R(KR), R_LR(RLR),
          T_LR(TLR), D_L(DL), D_R(DR), angle_idx(aidx)
    {
        cvK_L = (cv::Mat_<double>(3,3) << K_L(0,0), K_L(0,1), K_L(0,2),
                                         K_L(1,0), K_L(1,1), K_L(1,2), 0, 0, 1);
        cvD_L = (cv::Mat_<double>(5,1) << D_L(0), D_L(1), D_L(2), D_L(3), D_L(4));
        cvK_R = (cv::Mat_<double>(3,3) << K_R(0,0), K_R(0,1), K_R(0,2),
                                         K_R(1,0), K_R(1,1), K_R(1,2), 0, 0, 1);
        cvD_R = (cv::Mat_<double>(5,1) << D_R(0), D_R(1), D_R(2), D_R(3), D_R(4));
    }

    Eigen::VectorXd computeResiduals(const Eigen::VectorXd& x) {
        Eigen::Vector3d rvec_R0(x[0], x[1], x[2]);
        Eigen::Vector3d T0(x[3], x[4], x[5]);
        Eigen::Matrix3d R0;
        if (rvec_R0.norm() < 1e-8) R0 = Eigen::Matrix3d::Identity();
        else R0 = Eigen::AngleAxisd(rvec_R0.norm(), rvec_R0.normalized()).toRotationMatrix();

        double theta = x[6], phi = x[7];
        Eigen::Vector3d axis_dir(std::sin(theta)*std::cos(phi),
                                 std::sin(theta)*std::sin(phi),
                                 std::cos(theta));

        Eigen::Vector3d ref = (std::abs(axis_dir[2]) < 0.9)
                              ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
        Eigen::Vector3d e1 = axis_dir.cross(ref).normalized();
        Eigen::Vector3d e2 = axis_dir.cross(e1).normalized();
        Eigen::Vector3d P_perp = x[8] * e1 + x[9] * e2;

        double angle_rad = x[angle_idx];
        Eigen::Matrix3d R_inc = Eigen::AngleAxisd(angle_rad, axis_dir).toRotationMatrix();

        Eigen::Matrix3d R_cur = R_inc * R0;
        Eigen::Vector3d T_cur = R_inc * T0 + (Eigen::Matrix3d::Identity() - R_inc) * P_perp;

        const int n = static_cast<int>(obj_pts.size());
        Eigen::VectorXd residuals(4 * n);

        cv::Mat zr = cv::Mat::zeros(3,1,CV_64F);
        cv::Mat zt = cv::Mat::zeros(3,1,CV_64F);

        // 一次性投影全部角点 (每相机一次 projectPoints), 而不是逐点各调一次。
        // projectPoints 对每个点本就是独立计算, 批处理结果与逐点调用逐位等价,
        // 但省掉了 2n 次调用开销与每次的 vector 堆分配。
        std::vector<cv::Point3d> pt_l(n), pt_r(n);
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d Xc = R_cur * Eigen::Vector3d(obj_pts[i].x, obj_pts[i].y, obj_pts[i].z) + T_cur;
            Eigen::Vector3d Xr = R_LR * Xc + T_LR;
            pt_l[i] = cv::Point3d(Xc[0], Xc[1], Xc[2]);
            pt_r[i] = cv::Point3d(Xr[0], Xr[1], Xr[2]);
        }

        std::vector<cv::Point2d> proj_l, proj_r;
        cv::projectPoints(pt_l, zr, zt, cvK_L, cvD_L, proj_l);
        cv::projectPoints(pt_r, zr, zt, cvK_R, cvD_R, proj_r);

        for (int i = 0; i < n; ++i) {
            residuals(4*i+0) = proj_l[i].x - obs_l[i].x;
            residuals(4*i+1) = proj_l[i].y - obs_l[i].y;
            residuals(4*i+2) = proj_r[i].x - obs_r[i].x;
            residuals(4*i+3) = proj_r[i].y - obs_r[i].y;
        }
        return residuals;
    }
};

RotatingCalibrator::RotatingCalibrator() : m_squareSize(10.0f), m_reprojError(0.0), m_isProcessed(false) {
    // ---- 消歧选项 ----
    m_poseOpt.useIppeCandidates   = true;   // 枚举 IPPE 候选(含 cheirality / 可见面过滤)
    m_poseOpt.allowFlipCandidate  = true;   // 启用"检测顺序被翻转"的配对假设
    m_poseOpt.maxStepDeg          = 30.0;   // 相邻帧最大合理转角
    // wGauge 是"轻微偏好检测器原解"的先验代价, 刻意取这个量级:
    //   远大于候选之间的重投影残差差异(实测 ~0.1px) —— 避免精确二义性下因残差抖动
    //   而把解换成物理上等价的另一个, 从而凭空虚改行为;
    //   远小于一次 180° 连续性违例的代价 ((180-30)^2 = 22500) —— 所以真发生翻转时
    //   连续性证据仍能压倒它, 把该换的解换回来。
    m_poseOpt.wGauge = 5.0;
}
RotatingCalibrator::~RotatingCalibrator() {}

void RotatingCalibrator::setCameraParams(const cv::Mat& K_L, const cv::Mat& D_L, const cv::Mat& K_R, const cv::Mat& D_R, const cv::Mat& R_LR, const cv::Mat& T_LR) {
    m_K_L = K_L.clone(); m_D_L = D_L.clone();
    m_K_R = K_R.clone(); m_D_R = D_R.clone();
    m_R_LR = R_LR.clone(); m_T_LR = T_LR.clone();
    m_poseCacheDirty = true;
}

void RotatingCalibrator::setPatternParams(const cv::Size& boardSize, float squareSize) {
    m_boardSize = boardSize; m_squareSize = squareSize;
    m_poseCacheDirty = true;   // 棋盘尺寸/方格尺寸变了, 位姿随之失效
}

void RotatingCalibrator::setInputData(const QStringList& leftPaths, const QStringList& rightPaths) {
    m_leftPaths = leftPaths; m_rightPaths = rightPaths;
    m_anglesRad.clear();
    m_poseCacheDirty = true;
} 

void RotatingCalibrator::generateObjectPoints(std::vector<cv::Point3f>& objectPoints) const {
    // 委托给共用实现 (原先有 4 处重复生成, 顺序一致但易失同步)
    makeChessboardObjectPoints(m_boardSize, m_squareSize, objectPoints);
}

// ==================== 逐帧位姿缓存 ====================

const std::vector<RotatingCalibrator::FramePose>& RotatingCalibrator::posesLeft()
{
    if (m_poseCacheDirty) buildPoseCache();
    return m_poseL;
}

const std::vector<RotatingCalibrator::FramePose>& RotatingCalibrator::posesRight()
{
    if (m_poseCacheDirty) buildPoseCache();
    return m_poseR;
}

void RotatingCalibrator::buildPoseCache()
{
    m_poseCacheDirty = false;
    m_poseL.clear();
    m_poseR.clear();

    // 角点坐标始终由板参数决定, 这里统一重建, 保证与缓存同步
    generateObjectPoints(m_objectPoints);
    if (m_objectPoints.empty()) return;

    auto run = [&](const std::vector<std::vector<cv::Point2f>>& pts,
                   const cv::Mat& K, const cv::Mat& D,
                   std::vector<FramePose>& dst) {
        std::vector<FramePoseResult> res;
        if (!disambiguatePlanarSequence(pts, m_objectPoints, K, D, m_poseOpt, res)) return;
        dst.resize(res.size());
        for (size_t i = 0; i < res.size(); ++i) {
            dst[i].solved      = res[i].solved;
            dst[i].rvec        = res[i].rvec;
            dst[i].tvec        = res[i].tvec;
            dst[i].permuted    = res[i].permuted;
            dst[i].reprojRmsPx = res[i].reprojRmsPx;
            dst[i].reprojAltPx = res[i].reprojRmsAltPx;
            dst[i].stepAltDeg  = res[i].stepAltDeg;
            dst[i].flipFlag    = res[i].flipFlag;
            dst[i].note        = res[i].note;
        }
    };
    reportProgress("位姿消歧", 85);
    run(m_vecLeftPoints,  m_K_L, m_D_L, m_poseL);
    reportProgress("位姿消歧", 87);
    run(m_vecRightPoints, m_K_R, m_D_R, m_poseR);
    reportProgress("位姿消歧", 90);
}

bool RotatingCalibrator::extractStereoFeatures() {
    m_vecLeftPoints.clear(); m_vecRightPoints.clear(); m_validAngles.clear();
    m_validFrameIndices.clear();
    cv::Size winSize(5, 5), zeroZone(-1, -1);
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    const int totalFrames = m_leftPaths.size();
    // 阶段权重按修复后的实测耗时: 读图+角点提取占绝大部分 (200 帧约 10~11s / 共 12.4s)
    reportProgress("提取角点", 0);
    for (int i = 0; i < m_leftPaths.size(); ++i) {
        reportProgress("提取角点", phasePercent(0, 85, i, totalFrames));
        cv::Mat imgL = cv::imread(m_leftPaths[i].toStdString(), cv::IMREAD_GRAYSCALE);
        cv::Mat imgR = cv::imread(m_rightPaths[i].toStdString(), cv::IMREAD_GRAYSCALE);
        if (imgL.empty() || imgR.empty()) continue;
        std::vector<cv::Point2f> ptsL, ptsR;
        bool foundL = cv::findChessboardCorners(imgL, m_boardSize, ptsL, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool foundR = cv::findChessboardCorners(imgR, m_boardSize, ptsR, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (foundL && foundR) {
            cv::cornerSubPix(imgL, ptsL, winSize, zeroZone, criteria);
            cv::cornerSubPix(imgR, ptsR, winSize, zeroZone, criteria);
            m_vecLeftPoints.push_back(ptsL);
            m_vecRightPoints.push_back(ptsR);
            m_validFrameIndices.push_back(i);
        }
    }
    std::cout << "[RotatingCalib] 有效特征提取对数: " << m_vecLeftPoints.size() << " / " << m_leftPaths.size() << std::endl;
    reportProgress("提取角点", 85);
    m_poseCacheDirty = true;   // 特征点已重建, 位姿缓存失效
    return m_vecLeftPoints.size() >= 3;
}

bool RotatingCalibrator::estimateInitialPose() {
    generateObjectPoints(m_objectPoints);
    std::vector<cv::Mat> vecR, vecT;
    std::vector<int> validFrameMap; // vecR 索引 -> m_vecLeftPoints 索引 (PnP成功的帧)
    const std::vector<FramePose>& pl = posesLeft();
    for (size_t i = 0; i < m_vecLeftPoints.size() && i < pl.size(); ++i) {
        if (!pl[i].solved) {
            if (i == 0) return false; continue;
        }
        cv::Mat R_cur; cv::Rodrigues(vec3dToMat(pl[i].rvec), R_cur);
        vecR.push_back(R_cur);
        vecT.push_back(vec3dToMat(pl[i].tvec));
        validFrameMap.push_back(static_cast<int>(i));
    }
    if (vecR.empty()) return false;
    m_R_base = vecR[0].clone();
    m_T_base = vecT[0].clone();

    // ================= 【修复1：提取带符号的旋转角】 =================
    // 轴用"符号无关的二阶矩"估计, 而不是逐帧轴向量直接相加。
    // 原因: cv::Rodrigues 的范数恒在 [0,π], 转过 180° 之后的帧给出的轴向量是 **-a**,
    // 于是 Σ axis_i 会出现大量抵消 —— 实测 200 帧转满一圈时是 107 个 +a 对 93 个 -a,
    // 半和只剩约 14/200 的幅值, 逐帧噪声被放大约 14 倍, final_axis 成了很差的初值。
    // Σ axis_i·axis_iᵀ 对 ±a 给的是同一个贡献, 不存在抵消;取其最大特征向量即主轴。
    Eigen::Matrix3d axis_moment = Eigen::Matrix3d::Zero();
    std::vector<double> real_angles;
    std::vector<Eigen::Vector3d> axis_per_frame;

    for (size_t i = 1; i < vecR.size(); ++i) {
        cv::Mat dR = vecR[i] * vecR[0].t();
        cv::Mat dR_vec;
        cv::Rodrigues(dR, dR_vec); // 提取旋转向量，自带方向
        double angle = cv::norm(dR_vec);

        if (angle < 1e-6) {
            real_angles.push_back(0.0);
            axis_per_frame.push_back(Eigen::Vector3d::Zero());
            continue;
        }

        cv::Mat axis_cv = dR_vec / angle;
        Eigen::Vector3d axis_eig(axis_cv.at<double>(0), axis_cv.at<double>(1), axis_cv.at<double>(2));
        axis_per_frame.push_back(axis_eig);
        axis_moment += axis_eig * axis_eig.transpose();   // 符号无关, 不抵消
        real_angles.push_back(angle); // 暂存绝对值，后面统一修正符号
    }

    if (axis_moment.norm() < 1e-12) return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(axis_moment);
    if (es.info() != Eigen::Success) return false;
    // 最大特征值对应的特征向量 = 所有帧旋转轴的共同方向
    Eigen::Vector3d final_axis = es.eigenvectors().col(2).normalized();
    
    // 根据最终确定的轴向，统一回调旋转角的符号
    for (size_t i = 0; i < real_angles.size(); ++i) {
        if (axis_per_frame[i].norm() > 1e-6) {
            if (axis_per_frame[i].dot(final_axis) < 0) {
                real_angles[i] = -real_angles[i];
            }
        }
    }

    m_axisDirection = cv::Mat(3, 1, CV_64F);
    m_axisDirection.at<double>(0) = final_axis.x();
    m_axisDirection.at<double>(1) = final_axis.y();
    m_axisDirection.at<double>(2) = final_axis.z();

    // ================= 【修复2：精确求解旋转轴上的点 P_perp】 =================
    Eigen::Vector3d P0(m_T_base.at<double>(0), m_T_base.at<double>(1), m_T_base.at<double>(2));
    Eigen::Vector3d ref_v = (std::abs(final_axis[2]) < 0.9) ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
    Eigen::Vector3d e1 = final_axis.cross(ref_v).normalized();
    Eigen::Vector3d e2 = final_axis.cross(e1).normalized();
    
    Eigen::Vector2d P_perp_sum = Eigen::Vector2d::Zero();
    int perp_count = 0;

    for (size_t i = 1; i < vecR.size(); ++i) {
        if (std::abs(real_angles[i-1]) < 1e-6) continue;
        
        Eigen::Vector3d Pi(vecT[i].at<double>(0), vecT[i].at<double>(1), vecT[i].at<double>(2));
        Eigen::Matrix3d R_inc = Eigen::AngleAxisd(real_angles[i-1], final_axis).toRotationMatrix();
        
        // 根据 Pi = R_inc * P0 + (I - R_inc) * P_perp => (I - R_inc) * P_perp = Pi - R_inc * P0
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - R_inc;
        Eigen::Vector3d b = Pi - R_inc * P0;
        
        // 将 A 投影到 e1, e2 平面上解 2x2 线性方程组
        Eigen::Vector3d A_e1 = A * e1;
        Eigen::Vector3d A_e2 = A * e2;
        
        double M11 = A_e1.dot(e1), M12 = A_e2.dot(e1);
        double M21 = A_e1.dot(e2), M22 = A_e2.dot(e2);
        double b1 = b.dot(e1), b2 = b.dot(e2);
        
        double det = M11 * M22 - M12 * M21;
        if (std::abs(det) < 1e-6) continue;
        
        double c1 = (b1 * M22 - b2 * M12) / det;
        double c2 = (M11 * b2 - M21 * b1) / det;
        
        P_perp_sum += Eigen::Vector2d(c1, c2);
        perp_count++;
    }
    
    Eigen::Vector2d P_perp_mean = (perp_count > 0) ? Eigen::Vector2d(P_perp_sum / perp_count) : Eigen::Vector2d::Zero();
    Eigen::Vector3d P_perp_final = P_perp_mean[0] * e1 + P_perp_mean[1] * e2;
    
    m_axisPoint = cv::Mat(3, 1, CV_64F);
    m_axisPoint.at<double>(0) = P_perp_final.x();
    m_axisPoint.at<double>(1) = P_perp_final.y();
    m_axisPoint.at<double>(2) = P_perp_final.z();

    m_validAngles.clear();
    // 【修复】m_validAngles 必须与 m_vecLeftPoints 一一对齐 (每个有效左相机帧一个角度)。
    // 之前按 vecR 的长度构造 (m_validAngles.size() == vecR.size())，
    // 若某些帧 solvePnP 失败被跳过 (vecR.size() < m_vecLeftPoints.size())，
    // runBundleAdjustment 中 x[10+f] = m_validAngles[f] 会越界访问。
    m_validAngles.assign(m_vecLeftPoints.size(), 0.0);
    for (size_t f = 1; f < vecR.size(); ++f) {
        m_validAngles[validFrameMap[f]] = real_angles[f - 1];
    }
    std::cout << "[RotatingCalib] PnP反推得到有效旋转帧数: " << perp_count << " / " << (vecR.size() - 1) << std::endl;
    return true;
}

// ==================== 3D圆拟合轴估计 (比PnP+BA更稳健) ====================
// 原理: 相机绕固定轴旋转时，各帧相机光心轨迹是一个3D空间圆
//       圆所在平面法向量 = 旋转轴方向，圆心 = 轴上的一个点
bool RotatingCalibrator::estimateAxisByCircleFitting()
{
    generateObjectPoints(m_objectPoints);
    if (m_objectPoints.empty()) return false;

    // 过程数据复位 (Tab6 曲线图): 每次重跑都要从干净状态开始
    m_circleFit2d = AxisCircleFit();

    // 1. PnP求解左右相机位姿，提取光心位置 (右相机也参与圆拟合)
    std::vector<Eigen::Vector3d> centers;
    std::vector<int> centerCam;   // 0 = 左相机, 1 = 右相机 (诊断: 两台相机必须分开评估残差)
    std::vector<int> centerFrame; // 该光心来自哪一帧 (曲线图要按帧号对上)
    std::vector<cv::Mat> rvecsL;  // 左相机逐帧旋转向量 (用于确定轴的符号)
    const std::vector<FramePose>& pl = posesLeft();
    const std::vector<FramePose>& pr = posesRight();
    for (size_t i = 0; i < m_vecLeftPoints.size() && i < pl.size(); ++i) {
        // 原始帧号 (m_validFrameIndices 与 m_vecLeftPoints 一一对齐)
        const int frameIdx = (i < m_validFrameIndices.size())
                                 ? m_validFrameIndices[i] : static_cast<int>(i);
        // 左相机
        if (pl[i].solved) {
            cv::Mat rvecL = vec3dToMat(pl[i].rvec);
            cv::Mat R_cv; cv::Rodrigues(rvecL, R_cv);
            Eigen::Matrix3d R; for(int r=0;r<3;r++) for(int c=0;c<3;c++) R(r,c)=R_cv.at<double>(r,c);
            Eigen::Vector3d t(pl[i].tvec[0], pl[i].tvec[1], pl[i].tvec[2]);
            centers.push_back(-R.transpose() * t);
            centerCam.push_back(0);
            centerFrame.push_back(frameIdx);
            rvecsL.push_back(rvecL);
        }
        // 右相机 (同一世界坐标系)
        if (i < m_vecRightPoints.size() && i < pr.size() && pr[i].solved) {
            cv::Mat rvecR = vec3dToMat(pr[i].rvec);
            cv::Mat R_cv; cv::Rodrigues(rvecR, R_cv);
            Eigen::Matrix3d R; for(int r=0;r<3;r++) for(int c=0;c<3;c++) R(r,c)=R_cv.at<double>(r,c);
            Eigen::Vector3d t(pr[i].tvec[0], pr[i].tvec[1], pr[i].tvec[2]);
            centers.push_back(-R.transpose() * t);
            centerCam.push_back(1);
            centerFrame.push_back(frameIdx);
        }
    }
    if (centers.size() < 4) return false;

    // 2. 平面拟合: SVD求所有光心所在平面的法向量
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& c : centers) centroid += c;
    centroid /= centers.size();

    Eigen::MatrixXd A(centers.size(), 3);
    for (size_t i = 0; i < centers.size(); ++i)
        A.row(i) = (centers[i] - centroid).transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Vector3d axis_dir = svd.matrixV().col(2); // 最小奇异值→法向量
    axis_dir.normalize();

    // 2.5 【修复】SVD 法向量的符号是任意的 (±)，而 S5 的视角增量旋转直接用这个方向，
    //     符号反了会让所有视角朝反方向旋转 (点云彻底错位)。这里用"棋盘格位姿旋转方向"
    //     (与 PnP/BA 分支、以及重建时正的步长角约定一致) 来统一符号。
    {
        Eigen::Vector3d axis_sum = Eigen::Vector3d::Zero();
        for (size_t i = 1; i < rvecsL.size(); ++i) {
            cv::Mat R0, Ri, dR;
            cv::Rodrigues(rvecsL[0], R0);
            cv::Rodrigues(rvecsL[i], Ri);
            dR = Ri * R0.t();
            cv::Mat dv;
            cv::Rodrigues(dR, dv);
            double ang = cv::norm(dv);
            if (ang < 1e-6) continue;
            cv::Mat ax = dv / ang;
            axis_sum += Eigen::Vector3d(ax.at<double>(0), ax.at<double>(1), ax.at<double>(2));
        }
        if (axis_sum.norm() > 1e-6) {
            Eigen::Vector3d pnp_axis_cam = axis_sum.normalized();
            // PnP 轴在相机系；圆拟合轴在棋盘0系(=转台基准系)，必须转到同一坐标系再比符号
            cv::Mat R0_cv; cv::Rodrigues(rvecsL[0], R0_cv);
            Eigen::Matrix3d R0_eig; for (int r=0;r<3;r++) for (int c=0;c<3;c++) R0_eig(r,c)=R0_cv.at<double>(r,c);
            Eigen::Vector3d pnp_axis_world = R0_eig.transpose() * pnp_axis_cam;
            m_pnpAxisDir = cv::Mat(3, 1, CV_64F);
            m_pnpAxisDir.at<double>(0) = pnp_axis_cam.x();
            m_pnpAxisDir.at<double>(1) = pnp_axis_cam.y();
            m_pnpAxisDir.at<double>(2) = pnp_axis_cam.z();
            if (axis_dir.dot(pnp_axis_world) < 0) {
                std::cout << "[RotatingCalib] 圆拟合轴方向与棋盘旋转方向相反，已自动统一符号" << std::endl;
                // e1/e2 在下方由 axis_dir 构建，翻转会自然传导，无需在此重建
                axis_dir = -axis_dir;
            }
        }
    }

    // 3. 建立平面2D坐标系: e1, e2 为平面内两正交基
    Eigen::Vector3d ref = (std::abs(axis_dir.z()) < 0.9) ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
    Eigen::Vector3d e1 = axis_dir.cross(ref).normalized();
    Eigen::Vector3d e2 = axis_dir.cross(e1).normalized();

    // 4. 将光心投影到平面，得到2D坐标
    std::vector<Eigen::Vector2d> pts2d;
    for (const auto& c : centers) {
        Eigen::Vector3d d = c - centroid;
        pts2d.push_back(Eigen::Vector2d(d.dot(e1), d.dot(e2)));
    }

    // 5. 圆拟合 (线性最小二乘)
    // 圆方程: x^2+y^2 + A*x + B*y + C = 0
    // 圆心: (-A/2, -B/2), 半径: sqrt((A/2)^2+(B/2)^2-C)
    Eigen::MatrixXd M(pts2d.size(), 3);
    Eigen::VectorXd b(pts2d.size());
    for (size_t i = 0; i < pts2d.size(); ++i) {
        double x = pts2d[i].x(), y = pts2d[i].y();
        M(i, 0) = x; M(i, 1) = y; M(i, 2) = 1.0;
        b(i) = -(x*x + y*y);
    }
    Eigen::Vector3d abc = M.colPivHouseholderQr().solve(b);
    double cx = -abc(0) / 2.0, cy = -abc(1) / 2.0;
    double r_fit = std::sqrt(std::max(0.0, cx*cx + cy*cy - abc(2)));

    // 6. 圆心转回3D → 轴点
    Eigen::Vector3d axis_point_3d = centroid + cx * e1 + cy * e2;

    // 7. 残差评估 —— 【修复】必须分相机评估
    //    左右相机光心到轴的距离本来就不同 (相差约等于基线在该平面内的投影)，
    //    把两路光心混在一起做单圆拟合，得到的"残差"主要是两台相机的半径差
    //    (量级 = 基线/2，可达几十毫米)，完全掩盖真实标定质量；
    //    旧版把这个毫米级数值当作"重投影误差(px)"显示，导致标定看起来永远不合格。
    auto fitCircle2D = [](const std::vector<Eigen::Vector2d>& sub,
                          double& radius, double& rms, Eigen::Vector2d& center) -> bool {
        if (sub.size() < 4) return false;
        Eigen::MatrixXd Ms(sub.size(), 3);
        Eigen::VectorXd bs(sub.size());
        for (size_t i = 0; i < sub.size(); ++i) {
            double x = sub[i].x(), y = sub[i].y();
            Ms(i, 0) = x; Ms(i, 1) = y; Ms(i, 2) = 1.0;
            bs(i) = -(x * x + y * y);
        }
        Eigen::Vector3d abc = Ms.colPivHouseholderQr().solve(bs);
        center = Eigen::Vector2d(-abc(0) / 2.0, -abc(1) / 2.0);
        double r2 = center.x() * center.x() + center.y() * center.y() - abc(2);
        radius = std::sqrt(std::max(0.0, r2));
        double e = 0;
        for (const auto& p : sub) {
            double d = std::sqrt((p.x() - center.x()) * (p.x() - center.x())
                               + (p.y() - center.y()) * (p.y() - center.y()));
            e += (d - radius) * (d - radius);
        }
        rms = std::sqrt(e / sub.size());
        return true;
    };

    std::vector<Eigen::Vector2d> subL, subR;
    for (size_t i = 0; i < pts2d.size(); ++i) {
        if (centerCam[i] == 0) subL.push_back(pts2d[i]);
        else                   subR.push_back(pts2d[i]);
    }

    double r_left = 0, rms_left = -1, r_right = 0, rms_right = -1;
    Eigen::Vector2d c_left(0, 0), c_right(0, 0);
    bool fitL = fitCircle2D(subL, r_left, rms_left, c_left);
    bool fitR = fitCircle2D(subR, r_right, rms_right, c_right);

    // 混拟合残差仅作参考 (被半径差支配, 不作为质量指标)
    double rms_mixed = 0;
    for (const auto& p : pts2d) {
        double dist = std::sqrt((p.x()-cx)*(p.x()-cx) + (p.y()-cy)*(p.y()-cy));
        rms_mixed += (dist - r_fit) * (dist - r_fit);
    }
    rms_mixed = std::sqrt(rms_mixed / pts2d.size());

    // ---- 过程数据落地 (Tab6 曲线图) ----
    // **必须在下面的失败门限之前**: 圆拟合被判失败的那一次恰恰最需要看这张图
    // (光心到底散成什么样了), 若等门限通过才记录, 失败分支下曲线会是空的。
    {
        AxisCircleFit& cf = m_circleFit2d;
        cf.okL = fitL;  cf.okR = fitR;
        cf.cLx = c_left.x();  cf.cLy = c_left.y();
        cf.cRx = c_right.x(); cf.cRy = c_right.y();
        cf.rLmm  = r_left;  cf.rmsLmm = std::max(0.0, rms_left);
        cf.rRmm  = r_right; cf.rmsRmm = std::max(0.0, rms_right);
        cf.cCx = cx; cf.cCy = cy; cf.rCmm = r_fit; cf.rmsCmm = rms_mixed;
        cf.centerGapMm = (fitL && fitR) ? (c_left - c_right).norm() : 0.0;

        cf.obs.reserve(pts2d.size());
        for (size_t i = 0; i < pts2d.size(); ++i) {
            AxisCircleObs o;
            o.frameIdx = (i < centerFrame.size()) ? centerFrame[i] : static_cast<int>(i);
            o.cam = (i < centerCam.size()) ? centerCam[i] : 0;
            o.x = pts2d[i].x();
            o.y = pts2d[i].y();
            if ((o.cam == 0 && fitL) || (o.cam == 1 && fitR)) {
                const Eigen::Vector2d& cc = (o.cam == 0) ? c_left : c_right;
                const double rr = (o.cam == 0) ? r_left : r_right;
                o.residMm = (pts2d[i] - cc).norm() - rr;
            }
            cf.obs.push_back(o);
        }
    }

    std::cout << "[RotatingCalib 圆拟合] 光心数:" << centers.size() << " (左" << subL.size()
              << "/右" << subR.size() << ")"
              << " 轴方向:[" << axis_dir.x() << "," << axis_dir.y() << "," << axis_dir.z() << "]"
              << " 半径: 左" << r_left << "mm / 右" << r_right << "mm"
              << " 分相机残差RMS: 左" << rms_left << "mm / 右" << rms_right << "mm"
              << " (混拟合残差 " << rms_mixed << "mm 仅参考)" << std::endl;

    if (!fitL || !fitR) {
        std::cout << "[RotatingCalib] ⚠️ 左右相机光心不足以分别拟合圆，圆拟合结果不可信" << std::endl;
        return false;
    }
    // 质量门限: 分相机残差过大说明光心不共圆 (角点/内参/序列有问题)
    const double kCircleResidLimitMm = 5.0;
    if (std::max(rms_left, rms_right) > kCircleResidLimitMm) {
        std::cout << "[RotatingCalib] ⚠️ 分相机圆拟合残差超限 (>"
                  << kCircleResidLimitMm << "mm)，判定圆拟合失败" << std::endl;
        return false;
    }
    // 独立拟合的两个圆心应当重合 (都在轴上)
    double center_gap = (c_left - c_right).norm();
    std::cout << "[RotatingCalib] 左右独立拟合圆心偏差: " << center_gap << "mm" << std::endl;

    // 8. 写入结果
    m_axisDirection = cv::Mat(3, 1, CV_64F);
    m_axisDirection.at<double>(0) = axis_dir.x();
    m_axisDirection.at<double>(1) = axis_dir.y();
    m_axisDirection.at<double>(2) = axis_dir.z();

    m_axisPoint = cv::Mat(3, 1, CV_64F);
    m_axisPoint.at<double>(0) = axis_point_3d.x();
    m_axisPoint.at<double>(1) = axis_point_3d.y();
    m_axisPoint.at<double>(2) = axis_point_3d.z();

    // 同时设置 m_R_base / m_T_base 为第一帧Pose (兼容现有流程)
    if (!centers.empty() && !pl.empty() && pl[0].solved) {
        // 用第一帧的结果 (与 estimateInitialPose 同源, 消除两处独立解算互相覆盖的隐患)
        cv::Rodrigues(vec3dToMat(pl[0].rvec), m_R_base);
        m_T_base = vec3dToMat(pl[0].tvec);
    }

    // 【修复】圆拟合的质量指标用"分相机残差"(mm)，而不是被半径差支配的混拟合残差。
    //         旧版把这个毫米级数值写进 m_reprojError 并被 UI 当作"重投影误差(px)"显示，
    //         于是标定永远显示为"失败"级别的数值 (如 40px)，是转台标定最直观的假故障。
    m_circleRmsMm = 0.5 * (rms_left + rms_right);
    m_reprojError = m_circleRmsMm;   // 采用圆拟合时, 误差量纲为 mm (见 getErrorUnit())
    m_circleAxisDir = m_axisDirection.clone();
    m_circleAxisPt  = m_axisPoint.clone();
    m_circleCenterGapMm = center_gap;
    m_radiusLeftMm = r_left;
    m_radiusRightMm = r_right;
    m_residLeftMm = rms_left;
    m_residRightMm = rms_right;
    m_mixedRmsMm = rms_mixed;
    reportProgress("圆拟合", 94);
    return true;
}

bool RotatingCalibrator::runBundleAdjustment() {
    Eigen::Matrix3d K_L, K_R, R_LR;
    Eigen::Vector3d T_LR;
    Eigen::Matrix<double, 5, 1> D_L, D_R;
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) {
        K_L(i,j) = m_K_L.at<double>(i,j); K_R(i,j) = m_K_R.at<double>(i,j); R_LR(i,j) = m_R_LR.at<double>(i,j);
    }
    T_LR << m_T_LR.at<double>(0), m_T_LR.at<double>(1), m_T_LR.at<double>(2);
    for(int i=0; i<5; ++i) { D_L(i) = m_D_L.at<double>(i); D_R(i) = m_D_R.at<double>(i); }

    // ================= 【修复】用"同一批 3D 点的全量重投影"判断立体外参方向 =================
    // 旧版比较的是两个**不同** 3D 点 (右相机 PnP 平移 vs 标定板中心) 的投影位置，
    // 判据本身不成立，可能无端把 R_LR/T_LR 取逆 → BA 用错误的右相机模型做优化 →
    // 解出一个毫无意义的轴 (却仍可能满足"BA误差<20px"而被采用)。
    // 现在统一用左相机 PnP 位姿把标定板角点投到右图，比较两种外参假设的 RMS。
    Eigen::Matrix3d R0_eig; for(int i=0;i<3;i++) for(int j=0;j<3;j++) R0_eig(i,j) = m_R_base.at<double>(i,j);
    Eigen::Vector3d T0_eig(m_T_base.at<double>(0), m_T_base.at<double>(1), m_T_base.at<double>(2));
    {
        cv::Mat cvK_R_ = (cv::Mat_<double>(3,3) << K_R(0,0), K_R(0,1), K_R(0,2),
                                                 K_R(1,0), K_R(1,1), K_R(1,2), 0, 0, 1);
        cv::Mat cvD_R_ = (cv::Mat_<double>(5,1) << D_R(0), D_R(1), D_R(2), D_R(3), D_R(4));
        cv::Mat zero_r = cv::Mat::zeros(3,1,CV_64F), zero_t = cv::Mat::zeros(3,1,CV_64F);
        const std::vector<cv::Point2f>* obs = (m_vecRightPoints.empty() ? nullptr : &m_vecRightPoints[0]);

        auto reprojRmsR = [&](const Eigen::Matrix3d& R, const Eigen::Vector3d& T) -> double {
            double sum = 0; int cnt = 0;
            for (size_t k = 0; k < m_objectPoints.size(); ++k) {
                Eigen::Vector3d Xc = R0_eig * Eigen::Vector3d(m_objectPoints[k].x, m_objectPoints[k].y, m_objectPoints[k].z) + T0_eig;
                Eigen::Vector3d Xr = R * Xc + T;
                std::vector<cv::Point3d> p3 = { cv::Point3d(Xr.x(), Xr.y(), Xr.z()) };
                std::vector<cv::Point2d> p2;
                cv::projectPoints(p3, zero_r, zero_t, cvK_R_, cvD_R_, p2);
                if (obs && k < obs->size()) {
                    double dx = p2[0].x - (*obs)[k].x, dy = p2[0].y - (*obs)[k].y;
                    sum += dx * dx + dy * dy;
                }
                ++cnt;
            }
            return cnt > 0 ? std::sqrt(sum / cnt) : 1e9;
        };

        Eigen::Matrix3d R_new = R_LR.transpose();
        Eigen::Vector3d T_new = -R_new * T_LR;
        double err1 = reprojRmsR(R_LR, T_LR);
        double err2 = reprojRmsR(R_new, T_new);
        std::cout << "[RotatingCalib] 立体外参方向校验: 正向RMS=" << err1
                  << "px 反向RMS=" << err2 << "px" << std::endl;
        if (err1 > err2) {
            std::cout << "[RotatingCalib] ⚠️ 检测到立体外参方向相反 (T 的符号约定不符 OpenCV)，已自动修正" << std::endl;
            R_LR = R_new;
            T_LR = T_new;
            m_stereoFlipped = true;
        } else {
            m_stereoFlipped = false;
        }
    }

    int F = m_vecLeftPoints.size();
    int num_vars = 10 + F;
    Eigen::VectorXd x(num_vars);

    cv::Mat rvec_tmp; cv::Rodrigues(m_R_base, rvec_tmp);
    for(int i=0; i<3; ++i) { x[i] = rvec_tmp.at<double>(i); x[3+i] = m_T_base.at<double>(i); }

    Eigen::Vector3d axis_eig(m_axisDirection.at<double>(0), m_axisDirection.at<double>(1), m_axisDirection.at<double>(2));
    axis_eig.normalize();
    x[6] = std::acos(std::max(-1.0, std::min(1.0, (double)axis_eig[2])));
    x[7] = std::atan2(axis_eig[1], axis_eig[0]);

    Eigen::Vector3d ref_v = (std::abs(axis_eig[2]) < 0.9) ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
    Eigen::Vector3d e1_init = axis_eig.cross(ref_v).normalized();
    Eigen::Vector3d e2_init = axis_eig.cross(e1_init).normalized();

    // ================= 【修复4：使用正确计算的 P_perp 作为初值】 =================
    Eigen::Vector3d P_perp_init(m_axisPoint.at<double>(0), m_axisPoint.at<double>(1), m_axisPoint.at<double>(2));
    x[8] = P_perp_init.dot(e1_init);
    x[9] = P_perp_init.dot(e2_init);

    x[10] = 0.0;
    // 防御: 即使角度表与帧数不一致也不越界 (正常流程下二者已严格对齐)
    const double lastValidAngle = m_validAngles.empty() ? 0.0 : m_validAngles.back();
    for(int f = 1; f < F; ++f)
        x[10 + f] = (f < static_cast<int>(m_validAngles.size())) ? m_validAngles[f] : lastValidAngle;

    double init_cost = 0; int init_pts = 0;
    for(int f = 0; f < F; ++f) {
        RotatingCalibCostFunctor func(m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f],
                                          K_L, K_R, R_LR, T_LR, D_L, D_R, 10+f);
        init_cost += func.computeResiduals(x).squaredNorm();
        init_pts += m_objectPoints.size();
    }
    std::cout << "[RotatingCalib] 初始误差: " << std::sqrt(init_cost / init_pts) << " 像素" << std::endl;

    // ---- 迭代历史复位 (Tab6「BA迭代曲线」的数据源) ----
    // 用**未加权**的重投影 RMS 而不是 Huber cost 作为纵轴: Huber cost 的量纲与
    // 数值都不可直接对照, 而 RMS 与最终报出的 "重投影误差 xxx 像素" 是同一把尺子,
    // 曲线末端读数应当等于结果面板里的那个数 —— 便于互相印证。
    m_baIterRmsPx.clear();
    m_baIterAccepted.clear();
    m_baRmsInitialPx = (init_pts > 0) ? std::sqrt(init_cost / init_pts) : -1.0;

    // Huber 鲁棒核函数阈值 (像素)，抑制误检角点对BA的破坏性影响
    const double kHuberDelta = 1.5;

    // 辅助函数: Huber 权重与代价
    auto huberWeight = [&](double r) -> double {
        double abs_r = std::abs(r);
        return (abs_r <= kHuberDelta) ? 1.0 : kHuberDelta / abs_r;
    };
    auto huberCost = [&](const Eigen::VectorXd& r) -> double {
        double cost = 0;
        for (int k = 0; k < r.size(); ++k) {
            double abs_r = std::abs(r[k]);
            if (abs_r <= kHuberDelta)
                cost += 0.5 * r[k] * r[k];
            else
                cost += kHuberDelta * (abs_r - 0.5 * kHuberDelta);
        }
        return cost;
    };

    // ================= 【性能】利用法方程的"箭头"结构, 用 Schur 补代替稠密求解 =================
    // 第 f 帧的残差只依赖 x[0..9] 与 x[10+f], 所以法方程矩阵是箭头形:
    //     H = [ H_pp    H_pa ]        H_pp : 10x10  (位姿块, 所有帧共享)
    //         [ H_pa^T  H_aa ]        H_aa : FxF 对角 (每帧一个角度变量)
    // 旧实现对**每个残差分量**做一次全尺寸 (10+F)^2 外积来组装稠密 H, 复杂度 O(F^3):
    // F=200 时单是这一步就要 ~178s, 而标定是在 GUI 线程同步跑的 → 界面整个冻住,
    // 这就是"转台标定运行卡死"。这里只组装 H_pp 与每帧的 h_f(10 维)、s_f(标量),
    // 再用 Schur 补消掉角度块:
    //     S   = H_pp - Σ_f h_f h_fᵀ / s_f        (10x10)
    //     rhs = -g_pp + Σ_f h_f · g_a_f / s_f
    //     d_p = S⁻¹ rhs ,   d_a_f = (-g_a_f - h_fᵀ d_p) / s_f
    // 数学上与旧实现完全等价 (已用等价性测试逐位比对到 1e-16), 但组装降为 O(F·n)。
    // 阻尼加在对角上 (H_pp += λI, s_f += λ): I 是对角阵, 故 H+λI 仍严格保持箭头结构。
    const int nRes = static_cast<int>(m_objectPoints.size()) * 4;
    std::vector<Eigen::MatrixXd> Jpose(F, Eigen::MatrixXd(nRes, 10));
    std::vector<Eigen::VectorXd> Jangle(F, Eigen::VectorXd(nRes));
    std::vector<Eigen::VectorXd> resF(F);
    std::vector<Eigen::VectorXd> hvec(F, Eigen::VectorXd::Zero(10));
    std::vector<double> sval(F, 0.0), gaval(F, 0.0);
    Eigen::MatrixXd H_pp = Eigen::MatrixXd::Zero(10, 10);
    Eigen::VectorXd g_pp = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(num_vars);
    double lambda = 1e-3;

    // 组装只依赖当前 x, 与 λ 无关 —— LM 步子被拒时只需换新 λ 重做 Schur 消元与 10x10 求解
    // (O(10·F)), 不必重跑 11F 次残差求值。早期迭代里被拒很常见, 省下的是大头。
    bool   assembled  = false;
    double total_cost = 0.0;
    double sMax       = 0.0;   // 各帧角度可观测性 s_f 的最大值, 用于定退化阈值

    // 曲线纵轴的换算: 分母与最终 m_reprojError 的口径一致 (每帧 nObj 个点, 每点 4 个残差分量)
    const int nObj = static_cast<int>(m_objectPoints.size());
    auto rmsOf = [&](double sq) -> double {
        return (F > 0 && nObj > 0) ? std::sqrt(sq / static_cast<double>(F * nObj)) : 0.0;
    };
    double cur_rms = 0.0;   // 当前 x 处的 RMS (组装阶段顺带算出, 被拒的轮次沿用它)

    for (int iter = 0; iter < 80; ++iter) {
        reportProgress("BA 优化", phasePercent(94, 100, iter, 80));
        if (!assembled) {
            H_pp.setZero(); g_pp.setZero();
            for (int f = 0; f < F; ++f) { hvec[f].setZero(); sval[f] = 0.0; gaval[f] = 0.0; }
            total_cost = 0.0;
            sMax = 0.0;
            double total_sq = 0.0;   // 未加权残差平方和 (曲线用; Huber cost 不给曲线看)

            for (int f = 0; f < F; ++f) {
                const int aidx = 10 + f;
                RotatingCalibCostFunctor func(m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f],
                                              K_L, K_R, R_LR, T_LR, D_L, D_R, aidx);
                resF[f] = func.computeResiduals(x);
                const Eigen::VectorXd& rr = resF[f];
                total_cost += huberCost(rr);
                total_sq   += rr.squaredNorm();

                const double eps = 1e-7;
                for (int j = 0; j < 10; ++j) {
                    Eigen::VectorXd x_plus = x; x_plus[j] += eps;
                    Jpose[f].col(j) = (func.computeResiduals(x_plus) - rr) / eps;
                }
                // f==0 的角度是规范 (gauge), 恒为 0, 其列不参与估计
                if (f > 0) {
                    Eigen::VectorXd x_plus = x; x_plus[aidx] += eps;
                    Jangle[f] = (func.computeResiduals(x_plus) - rr) / eps;
                } else {
                    Jangle[f].setZero();
                }

                // IRLS: 对每个残差分量施加 Huber 权重, 直接累积进箭头结构的各个块
                for (int k = 0; k < rr.size(); ++k) {
                    const double w = huberWeight(rr[k]);
                    const Eigen::Matrix<double, 10, 1> jp = Jpose[f].row(k).transpose();
                    H_pp += w * jp * jp.transpose();
                    g_pp += w * jp * rr[k];
                    if (f > 0) {
                        const double ja = Jangle[f][k];
                        hvec[f] += w * jp * ja;
                        sval[f] += w * ja * ja;
                        gaval[f] += w * ja * rr[k];
                    }
                }
            }
            for (int f = 1; f < F; ++f) sMax = std::max(sMax, sval[f]);

            // 所有帧的角度列都为零 → 转角完全不可观测 (标定板点落在旋转轴上才会这样)。
            // 此时 BA 没有可用信息, 直接放弃精化, 让 process() 回退到圆拟合分支。
            if (!(sMax > 0.0)) {
                std::cout << "[RotatingCalib] ⚠️ 转角不可观测 (所有帧的角度雅可比为 0)，放弃 BA 精化" << std::endl;
                return false;
            }
            cur_rms = rmsOf(total_sq);
            assembled = true;
        }

        // 阻尼加在对角 (保持箭头结构), 然后 Schur 消元求 d_p 与各 d_a_f。
        // 注意 λ 必须在这里就并进 S 与 s_f —— 先算未阻尼的 S 再只对求解加 λ 是不对的,
        // 那样 S 不再保证正定 (相消会把它做坏)。
        const double sMin = 1e-12 * sMax;
        Eigen::MatrixXd S = H_pp + lambda * Eigen::MatrixXd::Identity(10, 10);
        Eigen::VectorXd rhs = -g_pp;
        for (int f = 1; f < F; ++f) {   // f==0 整帧跳过: 它的角度列恒零, 不能用 λ 去兜 0/0
            const double sf = sval[f] + lambda;
            if (!(sval[f] >= sMin)) continue;   // 该帧角度退化, 不参与消元
            S   -= (hvec[f] * hvec[f].transpose()) / sf;
            rhs += (hvec[f] * gaval[f]) / sf;
        }
        const Eigen::VectorXd d_p = S.ldlt().solve(rhs);
        dx.setZero();
        for (int j = 0; j < 10; ++j) dx[j] = d_p[j];
        for (int f = 1; f < F; ++f) {
            if (!(sval[f] >= sMin)) { dx[10 + f] = 0.0; continue; }
            dx[10 + f] = (-gaval[f] - hvec[f].dot(d_p)) / (sval[f] + lambda);
        }

        // 数值兜底: 解出 NaN/Inf 时旧的 accept/reject 会静默吞掉 (NaN < cost 恒为 false),
        // 于是 x 永不更新、80 轮后返回最初的 PnP 猜测 —— 看起来像"标定成功"却毫无精化。
        // 这里显式拒绝该步并加大阻尼重试。
        if (!dx.allFinite()) {
            lambda *= 10.0;
            m_baIterRmsPx.push_back(cur_rms);
            m_baIterAccepted.push_back(0);
            continue;
        }

        Eigen::VectorXd x_new = x + dx;
        double new_cost = 0, new_sq = 0;
        for (int f = 0; f < F; ++f) {
            RotatingCalibCostFunctor func(m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f],
                                          K_L, K_R, R_LR, T_LR, D_L, D_R, 10+f);
            const Eigen::VectorXd rr_new = func.computeResiduals(x_new);
            new_cost += huberCost(rr_new);
            new_sq   += rr_new.squaredNorm();   // 曲线纵轴 (未加权 RMS), 顺带算, 零额外开销
        }
        if (new_cost < total_cost) {
            x = x_new; lambda = std::max(lambda * 0.1, 1e-10);
            m_baIterRmsPx.push_back(rmsOf(new_sq));
            m_baIterAccepted.push_back(1);
            if (dx.norm() < 1e-10) break;
            assembled = false;   // 接受该步 → 在 x 处重新组装
        } else {
            lambda *= 10.0;      // 拒绝该步 → 组装结果仍然有效, 只需换 λ 重新消元
            // 被拒时 x 不变 → 曲线出现平台段。这正是在暴露"阻尼在反复加大、步子一直走不动",
            // 是判断"BA 是收敛慢还是根本收敛不了"最直接的证据。
            m_baIterRmsPx.push_back(cur_rms);
            m_baIterAccepted.push_back(0);
        }
    }

    cv::Mat rvec_final(3, 1, CV_64F);
    for(int i=0; i<3; ++i) rvec_final.at<double>(i) = x[i];
    cv::Rodrigues(rvec_final, m_R_base);
    m_T_base = cv::Mat(3, 1, CV_64F);
    for(int i=0; i<3; ++i) m_T_base.at<double>(i) = x[3+i];

    double theta_opt = x[6], phi_opt = x[7];
    m_axisDirection = cv::Mat(3, 1, CV_64F);
    m_axisDirection.at<double>(0) = std::sin(theta_opt) * std::cos(phi_opt);
    m_axisDirection.at<double>(1) = std::sin(theta_opt) * std::sin(phi_opt);
    m_axisDirection.at<double>(2) = std::cos(theta_opt);

    Eigen::Vector3d axis_f(m_axisDirection.at<double>(0), m_axisDirection.at<double>(1), m_axisDirection.at<double>(2));
    Eigen::Vector3d ref_f = (std::abs(axis_f[2]) < 0.9) ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
    Eigen::Vector3d e1_f = axis_f.cross(ref_f).normalized();
    Eigen::Vector3d e2_f = axis_f.cross(e1_f).normalized();
    Eigen::Vector3d P_final = x[8] * e1_f + x[9] * e2_f;
    m_axisPoint = cv::Mat(3, 1, CV_64F);

    // ================= 【修复】BA 的轴是在**相机系**参数化的 (因为 R_cur = R_inc·R0)，
    //   而 3D圆拟合分支给出的是**棋盘0系(=转台基准系)**下的轴，S5 也是在该坐标系里做
    //   视角增量旋转 (Rot(axis, -Δθ))。两分支坐标系不一致会导致:
    //     · 圆拟合分支: 重建正常
    //     · BA 分支:   轴方向整体错一个 R_base^T，S5 朝错误方向旋转 → 点云错位/重影
    //   这就是"转台标定时好时坏"的根源。此处统一折算到棋盘0系后再对外输出。
    {
        cv::Mat rvec0; cv::Rodrigues(m_R_base, rvec0);
        Eigen::Matrix3d R0eig; for (int r=0;r<3;r++) for (int c=0;c<3;c++) R0eig(r,c) = m_R_base.at<double>(r,c);
        Eigen::Vector3d T0eig(m_T_base.at<double>(0), m_T_base.at<double>(1), m_T_base.at<double>(2));
        Eigen::Vector3d axis_world = R0eig.transpose() * axis_f;
        Eigen::Vector3d point_world = R0eig.transpose() * (P_final - T0eig);
        std::cout << "[RotatingCalib] BA轴(相机系) [" << axis_f.x() << "," << axis_f.y() << "," << axis_f.z()
                  << "] → 转台基准系 [" << axis_world.x() << "," << axis_world.y() << "," << axis_world.z() << "]" << std::endl;
        m_axisDirection.at<double>(0) = axis_world.x();
        m_axisDirection.at<double>(1) = axis_world.y();
        m_axisDirection.at<double>(2) = axis_world.z();
        m_axisPoint.at<double>(0) = point_world.x();
        m_axisPoint.at<double>(1) = point_world.y();
        m_axisPoint.at<double>(2) = point_world.z();
    }

    double final_cost = 0; int total_pts = 0;
    for (int f = 0; f < F; ++f) {
        RotatingCalibCostFunctor func(m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f],
                                          K_L, K_R, R_LR, T_LR, D_L, D_R, 10+f);
        final_cost += func.computeResiduals(x).squaredNorm();
        total_pts += m_objectPoints.size();
    }
    m_reprojError = std::sqrt(final_cost / total_pts);
    std::cout << "[RotatingCalib] 优化完成，最终重投影误差: " << m_reprojError << " 像素" << std::endl;
    return true;
}

bool RotatingCalibrator::process() {
    m_isProcessed = false;
    if (m_K_L.empty() || m_K_R.empty() || m_R_LR.empty()) { std::cerr << "[RotatingCalib] 相机参数未设置！" << std::endl; return false; }
    if (m_leftPaths.isEmpty()) { std::cerr << "[RotatingCalib] 图像路径未设置！" << std::endl; return false; }
    if (!extractStereoFeatures()) return false;

    // 主方案: 3D圆拟合 (对PnP噪声不敏感, 14帧即可稳定)
    bool circle_ok = estimateAxisByCircleFitting();
    // 立即保存圆拟合结果，后续 BA 会覆盖成员变量
    cv::Mat circle_axis_dir = circle_ok ? m_axisDirection.clone() : cv::Mat();
    cv::Mat circle_axis_pt  = circle_ok ? m_axisPoint.clone() : cv::Mat();
    double circle_rms = circle_ok ? m_circleRmsMm : 1e9;   // 分相机残差 (mm)

    // 备用方案: 传统PnP+BA (帧数多时精度可能更高)
    bool ba_ok = false;
    double ba_error = 1e9;
    cv::Mat ba_axis_dir, ba_axis_pt;
    if (estimateInitialPose()) {
        if (runBundleAdjustment()) {
            ba_ok = true;
            ba_error = m_reprojError;   // px
            ba_axis_dir = m_axisDirection.clone();
            ba_axis_pt = m_axisPoint.clone();
        }
    }

    m_circleOk = circle_ok;
    m_baOk = ba_ok;
    m_baErrorPx = ba_ok ? ba_error : 0.0;
    m_baAxisDir = ba_axis_dir.clone();
    m_baAxisPt = ba_axis_pt.clone();

    // ==================== 决策: 用圆拟合还是 BA ====================
    // 注意量纲不同 (BA=px, 圆拟合=mm), 阈值的单位是 px, 只用于 BA 自身的可用性判断,
    // 不能把 mm 与 px 放在同一个尺子上比较 (旧版注释/日志把二者混为一谈)。
    //
    // 自动判定的阈值由 m_baErrorLimitPx 给出 (默认 1.0px, 可经 Tab4 调整)。
    // 旧版硬编码 20px 过宽: 实测遇到过 BA 重投影 5.34px 而单帧 PnP 只有 0.136px
    // (即 BA 明显没拟合好) 却仍被采用的情况。
    const bool preferCircle = [&]() -> bool {
        switch (m_axisMethod) {
        case AxisMethod::CircleFit: return true;
        case AxisMethod::BA:        return false;
        case AxisMethod::Auto:
        default:                    return (!ba_ok || ba_error > m_baErrorLimitPx);
        }
    }();

    // 强制的那一路若不可用, 回退到另一路 (并如实记进日志), 不让整次标定因此失败
    bool useCircle;
    if (preferCircle) useCircle = circle_ok;
    else              useCircle = !ba_ok;      // 想用 BA 但 BA 没成功 → 退回圆拟合

    // 描述必须反映**实际发生了什么**: 强制的那路失败时是回退, 不能还报"强制XX"
    // (否则会出现"采用BA结果 (强制圆拟合)"这种自相矛盾的日志)
    const bool forcedFallback =
        (m_axisMethod == AxisMethod::CircleFit && !circle_ok) ||
        (m_axisMethod == AxisMethod::BA        && !ba_ok);
    QString how;
    if (forcedFallback) {
        how = (m_axisMethod == AxisMethod::CircleFit) ? QString("强制圆拟合不可用→回退BA")
                                                      : QString("强制BA不可用→回退圆拟合");
    } else if (m_axisMethod == AxisMethod::CircleFit) {
        how = "强制圆拟合";
    } else if (m_axisMethod == AxisMethod::BA) {
        how = "强制BA";
    } else {
        how = useCircle ? QString("自动→回退圆拟合 (BA%1)")
                              .arg(ba_ok ? QString("重投影 %1px 超阈值").arg(ba_error, 0, 'f', 3)
                                         : QString("未成功"))
                        : QString("自动→采用BA");
    }

    if (useCircle) {
        if (!circle_ok) {
            Logger::error("[RotatingCalib] 圆拟合和BA均失败！");
            return false;
        }
        m_axisDirection = circle_axis_dir;
        m_axisPoint = circle_axis_pt;
        m_reprojError = circle_rms;
        m_usedBA = false;
        std::cout << "[RotatingCalib] 采用3D圆拟合结果 [" << how.toStdString()
                  << "] 分相机圆残差:" << circle_rms
                  << "mm ；BA误差 " << ba_error << "px, 阈值 " << m_baErrorLimitPx << "px" << std::endl;
        if (ba_ok) {
            std::cout << "[RotatingCalib] 对比: BA轴 [" << ba_axis_dir.at<double>(0) << "," << ba_axis_dir.at<double>(1) << "," << ba_axis_dir.at<double>(2) << "]"
                      << " | 圆拟合轴 [" << circle_axis_dir.at<double>(0) << "," << circle_axis_dir.at<double>(1) << "," << circle_axis_dir.at<double>(2) << "]" << std::endl;
        }
    } else {
        // BA 结果已在成员变量中，无需额外操作
        m_usedBA = true;
        std::cout << "[RotatingCalib] 采用BA结果 [" << how.toStdString()
                  << "] 重投影误差 " << ba_error << "px" << std::endl;
    }

    m_isProcessed = true;
    populateDebugData();
    reportProgress("完成", 100);
    return true;
}

double RotatingCalibrator::getReprojectionError() const { return m_reprojError; }
cv::Mat RotatingCalibrator::getAxisPoint() const { return m_axisPoint.clone(); }
cv::Mat RotatingCalibrator::getAxisDirection() const { return m_axisDirection.clone(); }
void RotatingCalibrator::getBasePose(cv::Mat& R_base, cv::Mat& T_base) const { R_base = m_R_base.clone(); T_base = m_T_base.clone(); }
const std::vector<PerFrameDebug>& RotatingCalibrator::getPerFrameDebug() const { return m_perFrameDebug; }

void RotatingCalibrator::populateDebugData()
{
    m_perFrameDebug.clear();
    if (m_axisDirection.empty() || m_axisPoint.empty()) return;

    generateObjectPoints(m_objectPoints);
    if (m_objectPoints.empty()) return;

    Eigen::Vector3d axis_dir(m_axisDirection.at<double>(0),
                             m_axisDirection.at<double>(1),
                             m_axisDirection.at<double>(2));
    axis_dir.normalize();
    Eigen::Vector3d axis_pt(m_axisPoint.at<double>(0),
                            m_axisPoint.at<double>(1),
                            m_axisPoint.at<double>(2));

    // 建立轴平面坐标系 (用于计算圆拟合残差)
    Eigen::Vector3d ref = (std::abs(axis_dir.z()) < 0.9)
                          ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
    Eigen::Vector3d e1 = axis_dir.cross(ref).normalized();
    Eigen::Vector3d e2 = axis_dir.cross(e1).normalized();

    // 先收集所有光心用于圆拟合统计
    std::vector<Eigen::Vector3d> all_centers;
    std::vector<double> all_reproj;
    std::vector<cv::Mat> all_rvecs, all_tvecs;
    // 【修复】原实现用"PnP 成功子集的下标 i"去索引 m_validFrameIndices/m_validAngles，
    //         一旦有帧 PnP 失败就会整体错位 (帧号与角度对不上)。这里显式记录每个成功帧的
    //         原始序号与角度，保证逐帧调试表与真实序列一一对应。
    std::vector<int>    all_frame_idx;
    std::vector<double> all_angles;

    const std::vector<FramePose>& pl = posesLeft();
    for (size_t i = 0; i < m_vecLeftPoints.size() && i < pl.size(); ++i) {
        if (!pl[i].solved) continue;
        cv::Mat rvec = vec3dToMat(pl[i].rvec), tvec = vec3dToMat(pl[i].tvec);

        // 计算重投影误差
        std::vector<cv::Point2f> projected;
        cv::projectPoints(m_objectPoints, rvec, tvec, m_K_L, m_D_L, projected);
        double err = 0;
        for (size_t k = 0; k < projected.size(); ++k)
            err += cv::norm(projected[k] - m_vecLeftPoints[i][k]);
        err = std::sqrt(err / projected.size());

        cv::Mat R_cv; cv::Rodrigues(rvec, R_cv);
        Eigen::Matrix3d R; for(int r=0;r<3;r++) for(int c=0;c<3;c++) R(r,c)=R_cv.at<double>(r,c);
        Eigen::Vector3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        Eigen::Vector3d C = -R.transpose() * t;  // 相机光心在世界系

        all_centers.push_back(C);
        all_reproj.push_back(err);
        all_rvecs.push_back(rvec.clone());
        all_tvecs.push_back(tvec.clone());
        all_frame_idx.push_back(m_validFrameIndices.empty() ? static_cast<int>(i)
                                                            : m_validFrameIndices[i]);
        all_angles.push_back(i < m_validAngles.size() ? m_validAngles[i] : 0.0);
    }

    if (all_centers.empty()) return;

    // 计算光心质心 (用于圆拟合残差)
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& c : all_centers) centroid += c;
    centroid /= all_centers.size();

    // 计算圆拟合参数: 从 axis_pt 和 centroid 推导半径
    Eigen::Vector3d proj_axis = centroid + e1 * (axis_pt - centroid).dot(e1)
                                          + e2 * (axis_pt - centroid).dot(e2);
    double circle_radius = 0;
    int circle_count = 0;
    for (const auto& c : all_centers) {
        Eigen::Vector3d d = c - proj_axis;
        circle_radius += (d - axis_dir * d.dot(axis_dir)).norm();
        circle_count++;
    }
    if (circle_count > 0) circle_radius /= circle_count;

    for (size_t i = 0; i < all_centers.size(); ++i) {
        PerFrameDebug dbg;
        dbg.frame_idx = all_frame_idx[i];
        dbg.detected = true;
        dbg.angle_rad = all_angles[i];

        cv::Vec3d rv(all_rvecs[i].at<double>(0), all_rvecs[i].at<double>(1), all_rvecs[i].at<double>(2));
        cv::Vec3d tv(all_tvecs[i].at<double>(0), all_tvecs[i].at<double>(1), all_tvecs[i].at<double>(2));
        dbg.rvec_left = rv;
        dbg.tvec_left = tv;
        dbg.reproj_error_left = all_reproj[i];

        const auto& C = all_centers[i];
        dbg.camera_center = cv::Vec3d(C.x(), C.y(), C.z());

        // 光心到轴点投影的距离
        Eigen::Vector3d d = C - proj_axis;
        double radial_dist = (d - axis_dir * d.dot(axis_dir)).norm();
        dbg.circle_dist = radial_dist;
        dbg.circle_residual = radial_dist - circle_radius;
        dbg.ba_residual = 0.0;  // BA逐帧残差需从优化过程获取，此处暂不重算

        m_perFrameDebug.push_back(dbg);
    }
}

// ============================================================================
//                        转台标定诊断 (Tab6 调试与诊断)
// ============================================================================

int AxisDiagnostics::failCount() const
{
    int n = 0;
    for (size_t i = 0; i < checks.size(); ++i) if (!checks[i].ok) ++n;
    return n;
}

cv::Mat RotatingCalibrator::getAxisDirectionCam() const
{
    if (m_axisDirection.empty() || m_R_base.empty()) return cv::Mat();
    return m_R_base * m_axisDirection;   // R_base: 棋盘0系 → 相机系
}

cv::Mat RotatingCalibrator::getAxisPointCam() const
{
    if (m_axisPoint.empty() || m_R_base.empty() || m_T_base.empty()) return cv::Mat();
    return m_R_base * m_axisPoint + m_T_base;
}

const AxisDiagnostics& RotatingCalibrator::runDiagnostics(double configuredStepDeg)
{
    m_diag = AxisDiagnostics();
    m_diag.configuredStepDeg = configuredStepDeg;

    const bool ok = process();               // 特征提取 + 圆拟合 + BA + 决策
    m_diag.valid = ok;
    m_diag.framesTotal = m_leftPaths.size();
    m_diag.framesValid = static_cast<int>(m_vecLeftPoints.size());

    if (!ok) {
        m_diag.summary = QString("标定失败：有效帧 %1/%2 —— 请检查相机参数、棋盘格内角点数/方格尺寸，以及图像清晰度")
                             .arg(m_diag.framesValid).arg(m_diag.framesTotal);
        return m_diag;
    }
    buildDiagnostics(configuredStepDeg);
    return m_diag;
}

void RotatingCalibrator::buildDiagnostics(double configuredStepDeg)
{
    AxisDiagnostics& d = m_diag;
    d.valid = true;
    d.configuredStepDeg = configuredStepDeg;
    d.source = m_usedBA ? QString("BA (PnP + LM/Huber)") : QString("3D 圆拟合");
    d.methodSetting = (m_axisMethod == AxisMethod::CircleFit) ? QString("强制圆拟合")
                    : (m_axisMethod == AxisMethod::BA)        ? QString("强制BA")
                                                             : QString("自动");
    d.baErrorLimitPx = m_baErrorLimitPx;
    d.baOk = m_baOk;
    d.circleOk = m_circleOk;
    d.baErrorPx = m_baErrorPx;
    d.circleRmsMm = m_circleRmsMm;
    d.stereoFlipped = m_stereoFlipped;
    d.radiusLeftMm = m_radiusLeftMm;
    d.radiusRightMm = m_radiusRightMm;
    d.residLeftMm = m_residLeftMm;
    d.residRightMm = m_residRightMm;

    // ---- 过程数据 (Tab6 曲线图): 圆拟合观测点/拟合圆 + BA 逐迭代 RMS ----
    d.circleFit      = m_circleFit2d;
    d.baIterRmsPx    = m_baIterRmsPx;
    d.baIterAccepted = m_baIterAccepted;
    d.baRmsInitialPx = m_baRmsInitialPx;

    // ---- 轴参数: 对外统一为转台基准系(棋盘0系)，同时给出相机系 ----
    for (int i = 0; i < 3; ++i) {
        d.axisDirTurntable[i]   = m_axisDirection.at<double>(i);
        d.axisPointTurntable[i] = m_axisPoint.at<double>(i);
    }
    for (int r = 0; r < 3; ++r) {
        d.baseT[r] = m_T_base.empty() ? 0.0 : m_T_base.at<double>(r);
        for (int c = 0; c < 3; ++c)
            d.baseR[r][c] = m_R_base.empty() ? (r == c ? 1.0 : 0.0) : m_R_base.at<double>(r, c);
    }
    Eigen::Matrix3d R0;
    Eigen::Vector3d T0;
    for (int r = 0; r < 3; ++r) {
        T0[r] = d.baseT[r];
        for (int c = 0; c < 3; ++c) R0(r, c) = d.baseR[r][c];
    }
    const Eigen::Vector3d axis_w(d.axisDirTurntable[0], d.axisDirTurntable[1], d.axisDirTurntable[2]);
    const Eigen::Vector3d axis_pw(d.axisPointTurntable[0], d.axisPointTurntable[1], d.axisPointTurntable[2]);
    const Eigen::Vector3d axis_c = R0 * axis_w;
    const Eigen::Vector3d axis_pc = R0 * axis_pw + T0;
    for (int i = 0; i < 3; ++i) {
        d.axisDirCam[i] = axis_c[i];
        d.axisPointCam[i] = axis_pc[i];
    }

    // ---- 圆拟合轴 vs BA 轴 夹角 (两种独立方法互验) ----
    if (!m_circleAxisDir.empty() && !m_baAxisDir.empty()) {
        Eigen::Vector3d a1(m_circleAxisDir.at<double>(0), m_circleAxisDir.at<double>(1), m_circleAxisDir.at<double>(2));
        Eigen::Vector3d a2(m_baAxisDir.at<double>(0), m_baAxisDir.at<double>(1), m_baAxisDir.at<double>(2));
        if (a1.norm() > 1e-9 && a2.norm() > 1e-9) {
            // **取 fabs**: 轴是一条直线, 方向反了物理上是同一根轴 —— 而在两条通道的
            // 符号约定 (SVD 法向量符号任意 / BA 的 PnP 符号统一) 不一致时, dot 会接近 -1,
            // 不取绝对值就会得到 179.98° 这种"两法轴几乎相反"的假失败, 掩盖真实夹角 0.02°。
            double c = std::max(-1.0, std::min(1.0, a1.normalized().dot(a2.normalized())));
            d.axisDiffDeg = std::acos(std::fabs(c)) * 180.0 / M_PI;
        }
    }

    // ---- 逐帧指标 ----
    const double kPi = 3.14159265358979323846;
    Eigen::Vector3d ref_v = (std::abs(axis_w.z()) < 0.9) ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
    Eigen::Vector3d e1 = axis_w.cross(ref_v).normalized();
    Eigen::Vector3d e2 = axis_w.cross(e1).normalized();

    struct Raw {
        int    idx = 0;
        double angleDeg = 0.0;
        double errL = 0.0, errR = -1.0;
        Eigen::Vector3d cL = Eigen::Vector3d::Zero();
        Eigen::Vector3d cR = Eigen::Vector3d::Zero();
        bool   hasR = false;
        bool   hasL = false;
        double planeDeg = 0.0;
        double planeSignedCos = 0.0;   // n_w · axis_w (带符号, 保留 fabs 丢掉的信息)
        // 位姿二义性 (来自位姿缓存)
        bool   posePermuted = false;
        int    ambigFlag = 0;
        double stepAltDeg = -1.0;
        double reprojAltPx = -1.0;
    };
    std::vector<Raw> raws;

    // 第 0 帧左相机位姿作为角度参考 (与 estimateInitialPose 一致)
    cv::Mat Rref;
    {
        const std::vector<FramePose>& pl0 = posesLeft();
        if (!pl0.empty() && pl0[0].solved)
            cv::Rodrigues(vec3dToMat(pl0[0].rvec), Rref);
    }

    auto rmsReproj = [](const std::vector<cv::Point3f>& obj, const std::vector<cv::Point2f>& obs,
                        const cv::Mat& rvec, const cv::Mat& tvec,
                        const cv::Mat& K, const cv::Mat& D) -> double {
        std::vector<cv::Point2f> proj;
        cv::projectPoints(obj, rvec, tvec, K, D, proj);
        double e = 0; int n = 0;
        for (size_t k = 0; k < proj.size() && k < obs.size(); ++k) {
            e += cv::norm(proj[k] - obs[k]);
            ++n;
        }
        return n > 0 ? e / n : -1.0;   // 平均像素误差
    };

    const std::vector<FramePose>& pl = posesLeft();
    const std::vector<FramePose>& pr = posesRight();
    for (size_t i = 0; i < m_vecLeftPoints.size() && i < pl.size(); ++i) {
        Raw r;
        r.idx = m_validFrameIndices.empty() ? static_cast<int>(i) : m_validFrameIndices[i];

        if (pl[i].solved) {
            cv::Mat rvecL = vec3dToMat(pl[i].rvec), tvecL = vec3dToMat(pl[i].tvec);
            r.hasL = true;
            r.posePermuted = pl[i].permuted;
            r.ambigFlag    = pl[i].flipFlag;
            r.stepAltDeg   = pl[i].stepAltDeg;
            r.reprojAltPx  = pl[i].reprojAltPx;
            r.errL = rmsReproj(m_objectPoints, m_vecLeftPoints[i], rvecL, tvecL, m_K_L, m_D_L);
            cv::Mat Rcv; cv::Rodrigues(rvecL, Rcv);
            Eigen::Matrix3d R; for (int a=0;a<3;a++) for (int b=0;b<3;b++) R(a,b) = Rcv.at<double>(a,b);
            Eigen::Vector3d t(tvecL.at<double>(0), tvecL.at<double>(1), tvecL.at<double>(2));
            r.cL = -R.transpose() * t;

            // 角度: 相对参考帧, 符号按相机系轴向统一
            if (!Rref.empty()) {
                cv::Mat dR = Rcv * Rref.t();
                cv::Mat dv; cv::Rodrigues(dR, dv);
                double ang = cv::norm(dv);
                if (ang > 1e-9) {
                    cv::Mat ax = dv / ang;
                    Eigen::Vector3d axc(ax.at<double>(0), ax.at<double>(1), ax.at<double>(2));
                    if (axc.dot(axis_c) < 0) ang = -ang;
                }
                r.angleDeg = ang * 180.0 / kPi;
            }
            // 棋盘格法向与轴夹角 (转到转台基准系比较)
            Eigen::Vector3d n_cam(Rcv.at<double>(0,2), Rcv.at<double>(1,2), Rcv.at<double>(2,2));
            cv::Mat RrefM = Rref.empty() ? cv::Mat::eye(3,3,CV_64F) : Rref;
            Eigen::Matrix3d Rr; for (int a=0;a<3;a++) for (int b=0;b<3;b++) Rr(a,b) = RrefM.at<double>(a,b);
            Eigen::Vector3d n_w = Rr.transpose() * n_cam;
            // 显示值沿用 fabs (保持与历史导出同口径, 零回归); 另存带符号量供一致性判据使用。
            // 旧实现只留 fabs, 使"板法向相对轴反向"完全不可见。
            const double cosSigned = n_w.dot(axis_w);
            r.planeSignedCos = cosSigned;
            double cosv = std::max(-1.0, std::min(1.0, std::fabs(cosSigned)));
            r.planeDeg = std::acos(cosv) * 180.0 / kPi;
        }

        if (i < m_vecRightPoints.size() && i < pr.size() && pr[i].solved) {
            cv::Mat rvecR = vec3dToMat(pr[i].rvec), tvecR = vec3dToMat(pr[i].tvec);
            r.hasR = true;
            r.errR = rmsReproj(m_objectPoints, m_vecRightPoints[i], rvecR, tvecR, m_K_R, m_D_R);
            cv::Mat Rcv; cv::Rodrigues(rvecR, Rcv);
            Eigen::Matrix3d R; for (int a=0;a<3;a++) for (int b=0;b<3;b++) R(a,b) = Rcv.at<double>(a,b);
            Eigen::Vector3d t(tvecR.at<double>(0), tvecR.at<double>(1), tvecR.at<double>(2));
            r.cR = -R.transpose() * t;
        }
        if (r.hasL) raws.push_back(r);
    }

    if (raws.empty()) {
        d.summary = "所有帧 PnP 均失败，无法诊断";
        return;
    }

    // ================= 【修复】角度序列解缠绕 =================
    // angleDeg 由 cv::Rodrigues 得到: 其范数落在 [0,π], 再按轴向统一定符号, 因此
    // **只能表示 (-180°, 180°]** —— 转台转满一圈 (本项目常见: 200 帧 × 1.8° = 360°)
    // 时, 后半段会被折回到负角度, 在步长里留下一个 ≈-358° 的假跳变。
    // 后果是三误报: 「步长均匀性」「实测步长 vs UI 设定步长」「序列累计转角」全被打成
    // 失败, 还会提示用户去"检查转台控制器分度精度", 而实际那台转台走得很干净
    // (实测 200 帧累计 339°, 步长 1.705°±0.118°)。
    // 这里按序列连续性解缠绕, 判据就是消歧里同一个 maxStepDeg=30° 的假设:
    // 相邻两位的转角远小于 180°。frame 0 恒为 0, 解缠绕后首元素不变。
    for (size_t i = 1; i < raws.size(); ++i) {
        double delta = raws[i].angleDeg - raws[i - 1].angleDeg;
        while (delta >  180.0) delta -= 360.0;
        while (delta <= -180.0) delta += 360.0;
        raws[i].angleDeg = raws[i - 1].angleDeg + delta;
    }

    // 逐相机 2D 圆拟合 → 半径与逐帧残差
    auto fitAndResidual = [&](bool left, double& radius, double& rmsOut,
                              std::vector<double>& perFrame) -> bool {
        std::vector<Eigen::Vector2d> pts;
        std::vector<int> mapIdx;
        for (size_t i = 0; i < raws.size(); ++i) {
            if (left && !raws[i].hasL) continue;
            if (!left && !raws[i].hasR) continue;
            Eigen::Vector3d c = left ? raws[i].cL : raws[i].cR;
            Eigen::Vector3d dd = c - axis_pw;
            pts.push_back(Eigen::Vector2d(dd.dot(e1), dd.dot(e2)));
            mapIdx.push_back(static_cast<int>(i));
        }
        perFrame.assign(raws.size(), 0.0);
        if (pts.size() < 4) return false;
        Eigen::MatrixXd M(pts.size(), 3); Eigen::VectorXd b(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            double x = pts[i].x(), y = pts[i].y();
            M(i,0) = x; M(i,1) = y; M(i,2) = 1.0;
            b(i) = -(x*x + y*y);
        }
        Eigen::Vector3d abc = M.colPivHouseholderQr().solve(b);
        Eigen::Vector2d ctr(-abc(0)/2.0, -abc(1)/2.0);
        radius = std::sqrt(std::max(0.0, ctr.x()*ctr.x() + ctr.y()*ctr.y() - abc(2)));
        double e = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            double dd = (pts[i] - ctr).norm();
            perFrame[mapIdx[i]] = dd - radius;
            e += (dd - radius) * (dd - radius);
        }
        rmsOut = std::sqrt(e / pts.size());
        return true;
    };

    std::vector<double> residL, residR;
    double rL = 0, rmsL = 0, rR = 0, rmsR = 0;
    bool okL = fitAndResidual(true,  rL, rmsL, residL);
    bool okR = fitAndResidual(false, rR, rmsR, residR);
    if (okL) { d.radiusLeftMm = rL;  if (m_residLeftMm  <= 0) d.residLeftMm  = rmsL; }
    if (okR) { d.radiusRightMm = rR; if (m_residRightMm <= 0) d.residRightMm = rmsR; }

    // 角度序列统计
    double stepSum = 0, stepSum2 = 0, stepMin = 1e18, stepMax = -1e18;
    int    stepN = 0;
    for (size_t i = 1; i < raws.size(); ++i) {
        double s = raws[i].angleDeg - raws[i-1].angleDeg;
        stepSum += s; stepSum2 += s*s; ++stepN;
        stepMin = std::min(stepMin, s);
        stepMax = std::max(stepMax, s);
    }
    if (stepN > 0) {
        d.stepMeanDeg = stepSum / stepN;
        double var = std::max(0.0, stepSum2 / stepN - d.stepMeanDeg * d.stepMeanDeg);
        d.stepStdDeg = std::sqrt(var);
        d.stepMinDeg = stepMin;
        d.stepMaxDeg = stepMax;
    }
    d.angleSpanDeg = raws.back().angleDeg - raws.front().angleDeg;

    // 轴向高差 (左右光心沿轴方向的差)
    double sumAxL = 0, sumAxR = 0; int nAxL = 0, nAxR = 0;
    for (size_t i = 0; i < raws.size(); ++i) {
        if (raws[i].hasL) { sumAxL += (raws[i].cL - axis_pw).dot(axis_w); ++nAxL; }
        if (raws[i].hasR) { sumAxR += (raws[i].cR - axis_pw).dot(axis_w); ++nAxR; }
    }
    if (nAxL > 0 && nAxR > 0) {
        double mL = sumAxL / nAxL, mR = sumAxR / nAxR;
        d.axialGapMm = mR - mL;
        m_axialGapMm = d.axialGapMm;
    }

    // 棋盘格法向统计
    {
        double s = 0, s2 = 0; int n = 0;
        for (size_t i = 0; i < raws.size(); ++i) { s += raws[i].planeDeg; s2 += raws[i].planeDeg * raws[i].planeDeg; ++n; }
        if (n > 0) {
            d.planeNormMeanDeg = s / n;
            d.planeNormStdDeg = std::sqrt(std::max(0.0, s2 / n - d.planeNormMeanDeg * d.planeNormMeanDeg));
        }
    }

    // ---- 逐帧汇总 (含异常判定) ----
    const double residLimitHard = 3.0, residLimitSoft = 1.5;
    const double planeDevSoft = 1.5, planeDevHard = 3.0;
    double errLSum = 0, errLMax = 0, errRSum = 0, errRMax = 0;
    int nL = 0, nR = 0;
    // 棋盘法向在转台基准系下应始终与轴**同向**。取带符号共识, 符号相反的帧必是解翻转。
    // 注意边界: 本工程的主失效模式是"精确 180° 面内自对称", 那种翻转下板法向**逐位
    // 相同**, 因此本判据对它无效(主检测靠 stepAltDeg); 这里只兜住 IPPE 倾斜镜像类。
    double planeCosSum = 0.0;
    for (const auto& r : raws) planeCosSum += r.planeSignedCos;
    const int planeSignConsensus = planeCosSum >= 0.0 ? 1 : -1;

    for (size_t i = 0; i < raws.size(); ++i) {
        AxisFrameMetrics m;
        m.frameIdx = raws[i].idx;
        m.detected = raws[i].hasL;
        m.angleDeg = raws[i].angleDeg;
        if (i > 0) m.stepDeg = raws[i].angleDeg - raws[i-1].angleDeg;
        m.reprojPxLeft = raws[i].errL;
        m.reprojPxRight = raws[i].hasR ? raws[i].errR : -1.0;
        for (int k = 0; k < 3; ++k) {
            m.centerLeft[k]  = raws[i].cL[k];
            m.centerRight[k] = raws[i].hasR ? raws[i].cR[k] : 0.0;
        }
        Eigen::Vector3d dL = raws[i].cL - axis_pw;
        m.radiusLeftMm = (dL - axis_w * dL.dot(axis_w)).norm();
        if (raws[i].hasR) {
            Eigen::Vector3d dR = raws[i].cR - axis_pw;
            m.radiusRightMm = (dR - axis_w * dR.dot(axis_w)).norm();
        }
        m.axialCoordMm = dL.dot(axis_w);
        m.residLeftMm  = okL ? residL[i] : 0.0;
        m.residRightMm = (raws[i].hasR && okR) ? residR[i] : 0.0;
        m.planeNormAxisDeg = raws[i].planeDeg;
        m.planeSignedCos   = raws[i].planeSignedCos;
        m.posePermuted = raws[i].posePermuted;
        m.poseAmbigFlag = raws[i].ambigFlag;
        m.stepAltDeg = raws[i].stepAltDeg;
        m.reprojAltPx = raws[i].reprojAltPx;

        QStringList notes;
        if (m.reprojPxLeft > 1.5)            { m.flag = 2; notes << QString("左重投影 %1px 偏大").arg(m.reprojPxLeft, 0, 'f', 2); }
        else if (m.reprojPxLeft > 0.8)       { m.flag = std::max(m.flag, 1); notes << QString("左重投影 %1px 略大").arg(m.reprojPxLeft, 0, 'f', 2); }
        if (m.reprojPxRight > 1.5)           { m.flag = 2; notes << QString("右重投影 %1px 偏大").arg(m.reprojPxRight, 0, 'f', 2); }
        else if (m.reprojPxRight > 0.8)      { m.flag = std::max(m.flag, 1); notes << QString("右重投影 %1px 略大").arg(m.reprojPxRight, 0, 'f', 2); }
        if (std::fabs(m.residLeftMm) > residLimitHard)       { m.flag = 2; notes << QString("左光心偏离拟合圆 %1mm").arg(m.residLeftMm, 0, 'f', 2); }
        else if (std::fabs(m.residLeftMm) > residLimitSoft)  { m.flag = std::max(m.flag, 1); notes << QString("左光心偏离拟合圆 %1mm").arg(m.residLeftMm, 0, 'f', 2); }
        if (okR && std::fabs(m.residRightMm) > residLimitHard)      { m.flag = 2; notes << QString("右光心偏离拟合圆 %1mm").arg(m.residRightMm, 0, 'f', 2); }
        else if (okR && std::fabs(m.residRightMm) > residLimitSoft) { m.flag = std::max(m.flag, 1); notes << QString("右光心偏离拟合圆 %1mm").arg(m.residRightMm, 0, 'f', 2); }
        double pdev = std::fabs(m.planeNormAxisDeg - d.planeNormMeanDeg);
        if (pdev > planeDevHard)      { m.flag = 2; notes << QString("棋盘姿态与均值差 %1°").arg(pdev, 0, 'f', 2); }
        else if (pdev > planeDevSoft) { m.flag = std::max(m.flag, 1); notes << QString("棋盘姿态与均值差 %1°").arg(pdev, 0, 'f', 2); }
        // 带符号法向一致性 (见上方 planeSignConsensus 的边界说明)
        if (raws[i].hasL && raws[i].planeSignedCos * planeSignConsensus < 0.0) {
            m.flag = std::max(m.flag, 2);
            notes << QString("棋盘法向相对转台轴反向 (cos=%1)").arg(raws[i].planeSignedCos, 0, 'f', 3);
        }

        // 二义性: 沿用 max 语义合并, 不吃掉上面已有的告警
        if (raws[i].ambigFlag > 0) {
            m.flag = std::max(m.flag, raws[i].ambigFlag == 2 ? 2 : 1);
            if (raws[i].posePermuted)
                notes << QString("检测顺序曾翻转, 已纠正 (翻转解步长 %1°)").arg(raws[i].stepAltDeg, 0, 'f', 1);
            else if (raws[i].ambigFlag == 2)
                notes << QString("位姿二义性无法消解 (翻转解步长 %1°)").arg(raws[i].stepAltDeg, 0, 'f', 1);
        }
        m.note = notes.join("；");

        if (raws[i].hasL) { errLSum += m.reprojPxLeft; errLMax = std::max(errLMax, m.reprojPxLeft); ++nL; }
        if (raws[i].hasR) { errRSum += m.reprojPxRight; errRMax = std::max(errRMax, m.reprojPxRight); ++nR; }
        d.frames.push_back(m);
    }
    const double errLMean = nL > 0 ? errLSum / nL : 0;
    const double errRMean = nR > 0 ? errRSum / nR : 0;

    int flagHard = 0, flagSoft = 0;
    for (size_t i = 0; i < d.frames.size(); ++i) {
        if (d.frames[i].flag == 2) ++flagHard;
        else if (d.frames[i].flag == 1) ++flagSoft;
    }

    // ---- 逐项检查 ----
    auto addCheck = [&](const QString& name, bool ok, const QString& value,
                        const QString& expect, const QString& advice) {
        AxisDiagCheck c; c.name = name; c.ok = ok; c.value = value; c.expect = expect; c.advice = advice;
        d.checks.push_back(c);
    };

    const double validRatio = d.framesTotal > 0 ? double(d.framesValid) / d.framesTotal : 0.0;
    addCheck("帧有效率", validRatio >= 0.8,
             QString("%1/%2 (%3%)").arg(d.framesValid).arg(d.framesTotal).arg(validRatio * 100, 0, 'f', 0),
             "≥ 80%",
             "角点检测失败的帧过多：检查棋盘格是否完整可见、光照/曝光是否过曝、ROI 是否裁掉了棋盘");

    addCheck("左相机角点重投影误差", errLMean <= 0.5 && errLMax <= 1.5,
             QString("均值 %1px / 最大 %2px").arg(errLMean, 0, 'f', 3).arg(errLMax, 0, 'f', 3),
             "均值 ≤ 0.5px, 最大 ≤ 1.5px",
             "角点定位或内参有问题：先重做 Tab2 双目标定，再检查棋盘格是否平整、是否有运动模糊");

    addCheck("右相机角点重投影误差", nR > 0 && errRMean <= 0.5 && errRMax <= 1.5,
             QString("均值 %1px / 最大 %2px").arg(errRMean, 0, 'f', 3).arg(errRMax, 0, 'f', 3),
             "均值 ≤ 0.5px, 最大 ≤ 1.5px",
             "同上；另外确认右相机内参与畸变已正确标定");

    addCheck("光心共圆残差(分相机)", okL && okR && rmsL <= 1.0 && rmsR <= 1.0,
             QString("左 %1mm / 右 %2mm").arg(rmsL, 0, 'f', 3).arg(rmsR, 0, 'f', 3),
             "各自 ≤ 1.0mm",
             "光心不共圆说明旋转轴不唯一或数据有问题：检查转台是否偏心/晃动、序列中是否有帧位置重复或缺失角度");

    addCheck("左右独立拟合圆心偏差", okL && okR && m_circleCenterGapMm <= 3.0,
             QString("%1mm").arg(m_circleCenterGapMm, 0, 'f', 3), "≤ 3mm",
             "两台相机各自推出来的轴不重合，通常是内参/外参不准或角点误匹配");

    addCheck("左右光心轴向高差", std::fabs(d.axialGapMm) <= 5.0,
             QString("%1mm").arg(d.axialGapMm, 0, 'f', 2), "|Δh| ≤ 5mm",
             "立体支架与转台轴不垂直(两相机不等高)：会让点云随角度产生 Z 方向整体偏移，建议调平支架后重标");

    addCheck("棋盘姿态一致性", d.planeNormStdDeg <= 1.5,
             QString("法向与轴夹角 %1° ± %2°").arg(d.planeNormMeanDeg, 0, 'f', 2).arg(d.planeNormStdDeg, 0, 'f', 2),
             "标准差 ≤ 1.5°",
             "棋盘格在转台上安装不牢(晃动)或个别帧角点检测错位：用逐帧表定位异常帧后重拍");

    double stepTol = std::max(0.2, std::fabs(d.stepMeanDeg) * 0.02);
    addCheck("步长均匀性", stepN > 0 && d.stepStdDeg <= stepTol && d.stepMinDeg * d.stepMaxDeg > 0,
             QString("均值 %1° / 标准差 %2° / 范围 [%3°, %4°]")
                 .arg(d.stepMeanDeg, 0, 'f', 3).arg(d.stepStdDeg, 0, 'f', 3)
                 .arg(d.stepMinDeg, 0, 'f', 3).arg(d.stepMaxDeg, 0, 'f', 3),
             "标准差 ≤ 2% 且方向一致",
             "转台步进不均匀或存在反向帧：检查转台控制器分度精度，确认序列没有漏拍/重拍");

    if (configuredStepDeg > 0) {
        double diff = std::fabs(d.stepMeanDeg - configuredStepDeg);
        addCheck("实测步长 vs UI 设定步长", diff <= 0.5,
                 QString("实测 %1° / 设定 %2° / 差 %3°")
                     .arg(d.stepMeanDeg, 0, 'f', 3).arg(configuredStepDeg, 0, 'f', 2).arg(diff, 0, 'f', 3),
                 "偏差 ≤ 0.5° (两者同一步长采集时)",
                 "S5 用 i×Tab5设定步长 作为视角初始旋转，步长填错会让点云随角度累积错位(螺旋/重影)。"
                 "若本序列与重建序列确实用不同步长采集，请忽略本项，但要把 Tab5 的步长改成重建序列的实测值");
    }

    addCheck("序列累计转角", std::fabs(d.angleSpanDeg) >= 180.0,
             QString("%1°").arg(d.angleSpanDeg, 0, 'f', 1), "≥ 180°",
             "转角范围太小(或只拍了一小段圆弧)：圆拟合平面法向会病态，至少要覆盖 180° 以上");

    if (d.baOk && d.circleOk) {
        addCheck("圆拟合轴 vs BA 轴一致性", d.axisDiffDeg <= 1.0,
                 QString("%1°").arg(d.axisDiffDeg, 0, 'f', 3), "≤ 1.0°",
                 "两种独立算法给出的轴不一致：说明数据质量处于临界，建议增加帧数/改善光照后重标");
    }

    addCheck("立体外参方向约定", !d.stereoFlipped,
             d.stereoFlipped ? QString("已自动取逆 (正向残差大于反向)") : QString("正常 (OpenCV 约定)"),
             "无需修正",
             "Tab2 的立体外参方向与 OpenCV 约定不符(很可能是标定文件被换过方向)：请在 Tab2 重新做一次双目标定");

    // 位姿二义性: 只统计"无法消解"(flag==2)。
    // 特别注意 "已消解" 绝不能计入 —— 对自对称棋盘格, 每一帧都天然存在 180° 翻转
    // 伙伴, 若把"检测顺序曾翻转(已自动纠正)"也算失败, 这项检查一旦真遇到翻转数据
    // 就永久报红, 属于本仓库历史上反复出现的假告警模式。
    {
        int ambigHard = 0, ambigSoft = 0, permutedN = 0;
        for (const auto& f : d.frames) {
            if (f.poseAmbigFlag == 2) ++ambigHard;
            else if (f.poseAmbigFlag == 1) ++ambigSoft;
            if (f.posePermuted) ++permutedN;
        }
        addCheck("位姿二义性消解", ambigHard == 0,
                 QString("无法消解 %1 帧 / 已消解 %2 帧 / 纠正顺序翻转 %3 帧")
                     .arg(ambigHard).arg(ambigSoft).arg(permutedN),
                 "无法消解 0 帧",
                 "某帧的两个候选解都无法用序列连续性排除(步长都落在合理区间)："
                 "该帧的检测顺序或角点配对不可靠，建议检查该帧图像后重拍");
    }

    addCheck("异常帧数量", flagHard == 0,
             QString("异常 %1 帧 / 注意 %2 帧").arg(flagHard).arg(flagSoft), "异常 0 帧",
             "用逐帧指标表定位异常帧(帧号已标注)，删除/重拍这些帧后再标定");

    // ---- 结论 ----
    const int fail = d.failCount();
    if (fail == 0) {
        d.summary = QString("共 %1 项检查全部通过 —— 轴标定结果可信 (采用 %2; 实测步长 %3°)")
                        .arg(d.checkCount()).arg(d.source).arg(d.stepMeanDeg, 0, 'f', 3);
    } else {
        QString first;
        for (size_t i = 0; i < d.checks.size(); ++i) {
            if (!d.checks[i].ok) { first = d.checks[i].name; break; }
        }
        d.summary = QString("共 %1 项检查，%2 项未通过 (首个: %3) —— 详见诊断报告")
                        .arg(d.checkCount()).arg(fail).arg(first);
    }
    std::cout << "[RotatingCalib 诊断] " << d.summary.toStdString() << std::endl;
}

} // namespace Calib
