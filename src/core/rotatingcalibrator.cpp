#include "core/rotatingcalibrator.h"
#include "ui/logger.h"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Calib {

struct RotatingCalibCostFunctor {
    const std::vector<cv::Point3f>& obj_pts;
    const std::vector<cv::Point2f>& obs_l;
    const std::vector<cv::Point2f>& obs_r;
    Eigen::Matrix3d K_L, K_R, R_LR;
    Eigen::Vector3d T_LR;
    Eigen::Matrix<double, 5, 1> D_L, D_R;
    int angle_idx;

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

        int n = obj_pts.size();
        Eigen::VectorXd residuals(4 * n);

        cv::Mat cvK_L = (cv::Mat_<double>(3,3) << K_L(0,0), K_L(0,1), K_L(0,2), K_L(1,0), K_L(1,1), K_L(1,2), 0, 0, 1);
        cv::Mat cvD_L = (cv::Mat_<double>(5,1) << D_L(0), D_L(1), D_L(2), D_L(3), D_L(4));
        cv::Mat cvK_R = (cv::Mat_<double>(3,3) << K_R(0,0), K_R(0,1), K_R(0,2), K_R(1,0), K_R(1,1), K_R(1,2), 0, 0, 1);
        cv::Mat cvD_R = (cv::Mat_<double>(5,1) << D_R(0), D_R(1), D_R(2), D_R(3), D_R(4));
        cv::Mat zr = cv::Mat::zeros(3,1,CV_64F);
        cv::Mat zt = cv::Mat::zeros(3,1,CV_64F);

        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d Xc = R_cur * Eigen::Vector3d(obj_pts[i].x, obj_pts[i].y, obj_pts[i].z) + T_cur;
            Eigen::Vector3d Xr = R_LR * Xc + T_LR;

            std::vector<cv::Point3d> pt_l = { cv::Point3d(Xc[0], Xc[1], Xc[2]) };
            std::vector<cv::Point2d> proj_l;
            cv::projectPoints(pt_l, zr, zt, cvK_L, cvD_L, proj_l);

            std::vector<cv::Point3d> pt_r = { cv::Point3d(Xr[0], Xr[1], Xr[2]) };
            std::vector<cv::Point2d> proj_r;
            cv::projectPoints(pt_r, zr, zt, cvK_R, cvD_R, proj_r);

            residuals(4*i+0) = proj_l[0].x - obs_l[i].x;
            residuals(4*i+1) = proj_l[0].y - obs_l[i].y;
            residuals(4*i+2) = proj_r[0].x - obs_r[i].x;
            residuals(4*i+3) = proj_r[0].y - obs_r[i].y;
        }
        return residuals;
    }
};

RotatingCalibrator::RotatingCalibrator() : m_squareSize(10.0f), m_reprojError(0.0), m_isProcessed(false) {}
RotatingCalibrator::~RotatingCalibrator() {}

void RotatingCalibrator::setCameraParams(const cv::Mat& K_L, const cv::Mat& D_L, const cv::Mat& K_R, const cv::Mat& D_R, const cv::Mat& R_LR, const cv::Mat& T_LR) {
    m_K_L = K_L.clone(); m_D_L = D_L.clone();
    m_K_R = K_R.clone(); m_D_R = D_R.clone();
    m_R_LR = R_LR.clone(); m_T_LR = T_LR.clone();
}

void RotatingCalibrator::setPatternParams(const cv::Size& boardSize, float squareSize) {
    m_boardSize = boardSize; m_squareSize = squareSize;
}

void RotatingCalibrator::setInputData(const QStringList& leftPaths, const QStringList& rightPaths) {
    m_leftPaths = leftPaths; m_rightPaths = rightPaths;
    m_anglesRad.clear();
} 

void RotatingCalibrator::generateObjectPoints(std::vector<cv::Point3f>& objectPoints) const {
    objectPoints.clear();
    for (int r = 0; r < m_boardSize.height; ++r)
        for (int c = 0; c < m_boardSize.width; ++c)
            objectPoints.push_back(cv::Point3f(c * m_squareSize, r * m_squareSize, 0.0f));
}

bool RotatingCalibrator::extractStereoFeatures() {
    m_vecLeftPoints.clear(); m_vecRightPoints.clear(); m_validAngles.clear();
    m_validFrameIndices.clear();
    cv::Size winSize(5, 5), zeroZone(-1, -1);
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    for (int i = 0; i < m_leftPaths.size(); ++i) {
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
    return m_vecLeftPoints.size() >= 3;
}

bool RotatingCalibrator::estimateInitialPose() {
    generateObjectPoints(m_objectPoints);
    std::vector<cv::Mat> vecR, vecT;
    for (size_t i = 0; i < m_vecLeftPoints.size(); ++i) {
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(m_objectPoints, m_vecLeftPoints[i], m_K_L, m_D_L, rvec, tvec)) {
            if (i == 0) return false; continue;
        }
        cv::Mat R_cur; cv::Rodrigues(rvec, R_cur);
        vecR.push_back(R_cur); vecT.push_back(tvec);
    }
    if (vecR.empty()) return false;
    m_R_base = vecR[0].clone();
    m_T_base = vecT[0].clone();

    // ================= 【修复1：提取带符号的旋转角】 =================
    Eigen::Vector3d axis_sum = Eigen::Vector3d::Zero();
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
        axis_sum += axis_eig;
        real_angles.push_back(angle); // 暂存绝对值，后面统一修正符号
    }
    
    if (axis_sum.norm() < 1e-6) return false;
    Eigen::Vector3d final_axis = axis_sum.normalized();
    
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

    m_validAngles.clear(); m_validAngles.push_back(0.0);
    m_validAngles.insert(m_validAngles.end(), real_angles.begin(), real_angles.end());
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

    // 1. PnP求解左右相机位姿，提取光心位置 (右相机也参与圆拟合)
    std::vector<Eigen::Vector3d> centers;
    for (size_t i = 0; i < m_vecLeftPoints.size(); ++i) {
        // 左相机
        cv::Mat rvecL, tvecL;
        if (cv::solvePnP(m_objectPoints, m_vecLeftPoints[i], m_K_L, m_D_L, rvecL, tvecL)) {
            cv::Mat R_cv; cv::Rodrigues(rvecL, R_cv);
            Eigen::Matrix3d R; for(int r=0;r<3;r++) for(int c=0;c<3;c++) R(r,c)=R_cv.at<double>(r,c);
            Eigen::Vector3d t(tvecL.at<double>(0), tvecL.at<double>(1), tvecL.at<double>(2));
            centers.push_back(-R.transpose() * t);
        }
        // 右相机 (同一世界坐标系)
        cv::Mat rvecR, tvecR;
        if (i < m_vecRightPoints.size() &&
            cv::solvePnP(m_objectPoints, m_vecRightPoints[i], m_K_R, m_D_R, rvecR, tvecR)) {
            cv::Mat R_cv; cv::Rodrigues(rvecR, R_cv);
            Eigen::Matrix3d R; for(int r=0;r<3;r++) for(int c=0;c<3;c++) R(r,c)=R_cv.at<double>(r,c);
            Eigen::Vector3d t(tvecR.at<double>(0), tvecR.at<double>(1), tvecR.at<double>(2));
            centers.push_back(-R.transpose() * t);
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

    // 7. 计算残差: 各光心到拟合圆心的距离与半径的偏差
    double rms_dist = 0;
    for (const auto& p : pts2d) {
        double dist = std::sqrt((p.x()-cx)*(p.x()-cx) + (p.y()-cy)*(p.y()-cy));
        rms_dist += (dist - r_fit) * (dist - r_fit);
    }
    rms_dist = std::sqrt(rms_dist / pts2d.size());

    std::cout << "[RotatingCalib 圆拟合] 光心数:" << centers.size() << " (含左右相机)"
              << " 轴方向:[" << axis_dir.x() << "," << axis_dir.y() << "," << axis_dir.z() << "]"
              << " 半径:" << r_fit << "mm"
              << " 圆心残差RMS:" << rms_dist << "mm" << std::endl;

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
    if (!centers.empty()) {
        // 用第一帧的结果
        cv::Mat rvec0, tvec0;
        cv::solvePnP(m_objectPoints, m_vecLeftPoints[0], m_K_L, m_D_L, rvec0, tvec0);
        cv::Rodrigues(rvec0, m_R_base);
        m_T_base = tvec0.clone();
    }

    m_reprojError = rms_dist; // 暂存圆拟合残差
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

    // ================= 【修复3：去除标定板尺寸硬编码】 =================
    cv::Mat rvec_R0, tvec_R0;
    cv::solvePnP(m_objectPoints, m_vecRightPoints[0], m_K_R, m_D_R, rvec_R0, tvec_R0);
    Eigen::Vector3d P_right_cam(tvec_R0.at<double>(0), tvec_R0.at<double>(1), tvec_R0.at<double>(2));
    Eigen::Vector3d proj_R_true = K_R * P_right_cam; 
    
    Eigen::Matrix3d R0_eig; for(int i=0;i<3;i++) for(int j=0;j<3;j++) R0_eig(i,j) = m_R_base.at<double>(i,j);
    Eigen::Vector3d T0_eig(m_T_base.at<double>(0), m_T_base.at<double>(1), m_T_base.at<double>(2));
    
    // 使用真实的标定板中心点做测试
    cv::Point3f center_obj(m_boardSize.width * m_squareSize / 2.0f, m_boardSize.height * m_squareSize / 2.0f, 0.0f);
    Eigen::Vector3d P_left_cam = R0_eig * Eigen::Vector3d(center_obj.x, center_obj.y, center_obj.z) + T0_eig;
    
    Eigen::Vector3d test1_cam = R_LR * P_left_cam + T_LR;
    Eigen::Vector3d proj1 = K_R * test1_cam;
    
    Eigen::Matrix3d R_new = R_LR.transpose();
    Eigen::Vector3d T_new = -R_new * T_LR;
    Eigen::Vector3d test2_cam = R_new * P_left_cam + T_new;
    Eigen::Vector3d proj2 = K_R * test2_cam;

    double err1 = (proj1.head<2>() - proj_R_true.head<2>()).norm();
    double err2 = (proj2.head<2>() - proj_R_true.head<2>()).norm();

    if (err1 > err2) {
        std::cout << "==================================================" << std::endl;
        std::cout << "[RotatingCalib] 检测到立体外参方向相反，已自动修正！" << std::endl;
        std::cout << "==================================================" << std::endl;
        R_LR = R_new;
        T_LR = T_new;
    } else {
        std::cout << "[RotatingCalib] 立体外参方向正确，无需修正。" << std::endl;
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
    for(int f = 1; f < F; ++f) x[10 + f] = m_validAngles[f];

    double init_cost = 0; int init_pts = 0;
    for(int f = 0; f < F; ++f) {
        RotatingCalibCostFunctor func{m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f], K_L, K_R, R_LR, T_LR, D_L, D_R, 10+f};
        init_cost += func.computeResiduals(x).squaredNorm();
        init_pts += m_objectPoints.size();
    }
    std::cout << "[RotatingCalib] 初始误差: " << std::sqrt(init_cost / init_pts) << " 像素" << std::endl;

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

    Eigen::MatrixXd H_total = Eigen::MatrixXd::Zero(num_vars, num_vars);
    Eigen::VectorXd g_total = Eigen::VectorXd::Zero(num_vars);
    double lambda = 1e-3;

    for (int iter = 0; iter < 80; ++iter) {
        H_total.setZero(); g_total.setZero();
        double total_cost = 0;
        for (int f = 0; f < F; ++f) {
            int aidx = 10 + f;
            RotatingCalibCostFunctor func{m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f], K_L, K_R, R_LR, T_LR, D_L, D_R, aidx};
            Eigen::VectorXd r = func.computeResiduals(x);
            total_cost += huberCost(r);
            Eigen::MatrixXd J(r.size(), num_vars);
            J.setZero();
            double eps = 1e-7;
            for (int j = 0; j < 10; ++j) {
                Eigen::VectorXd x_plus = x; x_plus[j] += eps;
                J.col(j) = (func.computeResiduals(x_plus) - r) / eps;
            }
            if (f > 0) {
                Eigen::VectorXd x_plus = x; x_plus[aidx] += eps;
                J.col(aidx) = (func.computeResiduals(x_plus) - r) / eps;
            }
            // IRLS: 对每个残差分量施加 Huber 权重
            for (int k = 0; k < r.size(); ++k) {
                double w = huberWeight(r[k]);
                H_total += w * J.row(k).transpose() * J.row(k);
                g_total += w * J.row(k).transpose() * r[k];
            }
        }
        Eigen::MatrixXd H_lm = H_total + lambda * Eigen::MatrixXd::Identity(num_vars, num_vars);
        Eigen::VectorXd dx = -H_lm.ldlt().solve(g_total);
        Eigen::VectorXd x_new = x + dx;
        double new_cost = 0;
        for (int f = 0; f < F; ++f) {
            RotatingCalibCostFunctor func{m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f], K_L, K_R, R_LR, T_LR, D_L, D_R, 10+f};
            new_cost += huberCost(func.computeResiduals(x_new));
        }
        if (new_cost < total_cost) {
            x = x_new; lambda = std::max(lambda * 0.1, 1e-10);
            if (dx.norm() < 1e-10) break;
        } else {
            lambda *= 10.0;
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
    m_axisPoint.at<double>(0) = P_final.x();
    m_axisPoint.at<double>(1) = P_final.y();
    m_axisPoint.at<double>(2) = P_final.z();

    double final_cost = 0; int total_pts = 0;
    for (int f = 0; f < F; ++f) {
        RotatingCalibCostFunctor func{m_objectPoints, m_vecLeftPoints[f], m_vecRightPoints[f], K_L, K_R, R_LR, T_LR, D_L, D_R, 10+f};
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
    double circle_rms = circle_ok ? m_reprojError : 1e9;

    // 备用方案: 传统PnP+BA (帧数多时精度可能更高)
    bool ba_ok = false;
    double ba_error = 1e9;
    cv::Mat ba_axis_dir, ba_axis_pt;
    if (estimateInitialPose()) {
        if (runBundleAdjustment()) {
            ba_ok = true;
            ba_error = m_reprojError;
            ba_axis_dir = m_axisDirection.clone();
            ba_axis_pt = m_axisPoint.clone();
        }
    }

    // 决策: BA误差>20px时用圆拟合; 否则用BA
    if (circle_ok && (!ba_ok || ba_error > 20.0)) {
        m_axisDirection = circle_axis_dir;
        m_axisPoint = circle_axis_pt;
        m_reprojError = circle_rms;
        std::cout << "[RotatingCalib] ⚠️ BA误差" << ba_error << "px > 20px → 采用3D圆拟合结果 (圆RMS:" << circle_rms << "mm)" << std::endl;
        if (ba_ok) {
            std::cout << "[RotatingCalib] 对比: BA轴 [" << ba_axis_dir.at<double>(0) << "," << ba_axis_dir.at<double>(1) << "," << ba_axis_dir.at<double>(2) << "]"
                      << " | 圆拟合轴 [" << circle_axis_dir.at<double>(0) << "," << circle_axis_dir.at<double>(1) << "," << circle_axis_dir.at<double>(2) << "]" << std::endl;
        }
    } else if (ba_ok) {
        // BA结果已在成员变量中，无需额外操作
        std::cout << "[RotatingCalib] BA误差" << ba_error << "px ≤ 20px → 采用BA结果" << std::endl;
    } else if (!circle_ok) {
        Logger::error("[RotatingCalib] 圆拟合和BA均失败！");
        return false;
    }

    m_isProcessed = true;
    populateDebugData();
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

    for (size_t i = 0; i < m_vecLeftPoints.size(); ++i) {
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(m_objectPoints, m_vecLeftPoints[i], m_K_L, m_D_L, rvec, tvec)) continue;

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
        dbg.frame_idx = m_validFrameIndices.empty() ? static_cast<int>(i)
                                                     : m_validFrameIndices[i];
        dbg.detected = true;
        dbg.angle_rad = m_validAngles.empty() ? 0.0 : m_validAngles[i];

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

} // namespace Calib
