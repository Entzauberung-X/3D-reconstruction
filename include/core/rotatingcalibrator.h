#ifndef ROTATINGCALIBRATOR_H
#define ROTATINGCALIBRATOR_H

/**
 * @file rotatingcalibrator.h
 * @brief 旋转平面标定算法头文件 (纯算法层)
 * @details 负责接收外部已知的双目内外参，通过旋转平面图像序列，计算旋转轴方程及基准位姿。
 *          核心算法基于 Ceres Solver 构建耦合旋转轴约束的全局 BA 优化模型。
 */

#pragma once

#include <opencv2/core.hpp>
#include <QStringList>
#include <vector>

namespace Calib {

// 每帧调试信息
struct PerFrameDebug {
    int frame_idx = 0;
    bool detected = false;
    double angle_rad = 0.0;           // 转台角度 (弧度)
    cv::Vec3d rvec_left;              // 左相机PnP旋转向量
    cv::Vec3d tvec_left;              // 左相机PnP平移向量 (mm)
    double reproj_error_left = 0.0;   // 左相机重投影误差 (px)
    cv::Vec3d camera_center;          // 左相机光心在世界系 (mm)
    double circle_dist = 0.0;         // 光心到拟合圆心的距离 (mm)
    double circle_residual = 0.0;     // 距离与拟合半径的偏差 (mm)
    double ba_residual = 0.0;         // BA重投影误差 (px, 若执行BA)
};

class RotatingCalibrator {
public:
    RotatingCalibrator();
    ~RotatingCalibrator();

    // ==================== 输入设置接口 ====================

    /**
     * @brief 设置双目相机的内外参 (强制要求外部传入)
     * @note 内部会自动进行深拷贝，不会修改外部传入的Mat对象
     */
    void setCameraParams(const cv::Mat& K_L, const cv::Mat& D_L,
                         const cv::Mat& K_R, const cv::Mat& D_R,
                         const cv::Mat& R_LR, const cv::Mat& T_LR);

    /**
     * @brief 设置标定板物理参数
     */
    void setPatternParams(const cv::Size& boardSize, float squareSize);

    /**
     * @brief 设置待处理的图像数据和旋转角度信息
     * @param stepAngleDeg 转台每次旋转的步长角度 (度)
     */
    void setInputData(const QStringList& leftPaths, const QStringList& rightPaths);
    
    // ==================== 核心算法执行接口 ====================

    /**
     * @brief 执行旋转平面标定算法主流程
     * @return true 标定优化收敛成功, false 失败
     */
    bool process();

    // ==================== 结果获取接口 ====================

    double getReprojectionError() const;
    cv::Mat getAxisPoint() const;      // 轴上的点 (3x1)
    cv::Mat getAxisDirection() const;  // 轴方向向量 (3x1, 单位向量)
    void getBasePose(cv::Mat& R_base, cv::Mat& T_base) const;
    const std::vector<PerFrameDebug>& getPerFrameDebug() const;  // 获取逐帧调试信息

private:
    // ==================== 内部私有算法函数 ====================
    bool extractStereoFeatures();
    bool estimateInitialPose();
    bool estimateAxisByCircleFitting();  // 3D圆拟合：比BA更稳健的轴估计
    bool runBundleAdjustment();
    void generateObjectPoints(std::vector<cv::Point3f>& objectPoints) const;
    void populateDebugData();  // 填充逐帧调试信息

    // ==================== 内部成员变量 ====================
    
    // 相机参数 (深拷贝存储)
    cv::Mat m_K_L, m_D_L;
    cv::Mat m_K_R, m_D_R;
    cv::Mat m_R_LR, m_T_LR;

    // 标定板参数
    cv::Size m_boardSize;
    float m_squareSize;

    // 图像与角度数据
    QStringList m_leftPaths;
    QStringList m_rightPaths;
    std::vector<double> m_anglesRad; // 转换为弧度后的绝对角度序列

    // 缓存的特征点提取结果 (只保留左右都成功提取的帧)
    std::vector<std::vector<cv::Point2f>> m_vecLeftPoints;
    std::vector<std::vector<cv::Point2f>> m_vecRightPoints;
    std::vector<cv::Point3f> m_objectPoints;
    std::vector<double> m_validAngles; // 与特征点对应的绝对角度
    std::vector<int> m_validFrameIndices; // 有效帧的原始索引

    // 算法输出结果
    cv::Mat m_axisPoint;      
    cv::Mat m_axisDirection;  
    cv::Mat m_R_base;         
    cv::Mat m_T_base;         
    double m_reprojError;
    bool m_isProcessed;
    std::vector<PerFrameDebug> m_perFrameDebug;       
};

} // namespace Calib

#endif