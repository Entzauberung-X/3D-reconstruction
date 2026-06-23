#ifndef LASERCALIBRATION_H
#define LASERCALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <QString>
#include <string>      // 【补全】用于 LineAnalysisResult
#include <functional>  // 【补全】用于 std::function

// LAB 算法参数结构体
struct LABParams {
    int minL = 80;
    int minA = 130;
    int maxB = 147;
    int whiteThresh = 232;
    int blockSize = 15;
    int C = -2;
};

// 斯蒂格算法参数
struct StegerParams {
    int gaussKernel = 5;        // 高斯模糊窗口大小 (必须为奇数，如 3, 5, 7)
    double gaussSigma = 1.5;    // 高斯模糊标准差 (激光线越宽，该值可适当调大至 2.0)
    float threshold = -20.0f;   // Hessian 阈值 (注意：亮线的最大特征值为负数，绝对值越大越严格)
    float maxT = 1.5f;          // 泰勒展开偏移限制 (极值点距离当前像素不能超过此值)
    int maskDilateRadius = 2;   // 掩膜膨胀半径 (防止宽带边缘在骨化时被截断遗漏)
};

// 激光对数据结构
struct LaserPair {
    std::vector<cv::Point2f> ptsL;  // 左相机激光点
    std::vector<cv::Point2f> ptsR;  // 右相机激光点
    cv::Rect roiL;
    cv::Rect roiR;

    // 记录位姿检测结果
    cv::Vec3d rvecL; 
    cv::Vec3d tvecL;
    bool poseValidL = false;

    cv::Vec3d rvecR;
    cv::Vec3d tvecR;
    bool poseValidR = false;
};

// 极线匹配线段结构体
struct LineSegment {
    cv::Point2f p1, p2;      // 线段端点
    cv::Vec4f lineParams;    // cv::fitLine 返回的
    // 【修正】将 originalPoints 改为 pts，与 .cpp 文件中的实现保持严格一致
    std::vector<cv::Point2f> pts; 
};

// 【补全】缺失的直线分析结果结构体
struct LineAnalysisResult {
    std::string equationStr;
    int pointCount = 0;
    float rmse = 0.0f;
    std::vector<cv::Point2f> pts;  // 【新增】存储RANSAC拟合的内点
};

// 激光平面标定结果结构体
struct LaserPlaneCalibrationResult {
    bool success = false;
    std::vector<cv::Point3f> worldPoints;  // 三维点云
    cv::Mat planeCoeffs;                  // 平面方程系数 [a, b, c, d]
    int totalPoints = 0;                  // 总点数
    int failedMatches = 0;                // 匹配失败点数
    QString resultMessage;                // 结果消息
};

// 处理结果结构体
struct LaserProcessingResult {
    bool success = false;
    cv::Rect roi;
    std::vector<cv::Point2f> laserPoints; // 2D 激光点 (像素坐标)
    
    // 位姿信息
    cv::Vec3d rvec; 
    cv::Vec3d tvec; 
    bool poseValid = false; 

    // 3D 坐标 (相机坐标系下)
    std::vector<cv::Point3f> points3d; 
    
    // 【新增】直线分析结果
    LineAnalysisResult lineAnalysis;
};

struct CalibrationParams {
    float epipolarThreshold = 2.0f;    // 极线匹配容差(像素)
    float depthMin = 10.0f;           // 最小有效深度
    float depthMax = 2000.0f;          // 最大有效深度
    int minPointsPerSegment = 5;       // 极线分段匹配时，单段最少点数
    float segmentBreakDist = 8.0f;     // 切分线段的最大间断距离(像素)
    
    // RANSAC 平面去噪参数
    float ransacDistThreshold = 0.5f;  // RANSAC 内点距离阈值
    int ransacMaxIterations = 200;     // RANSAC 最大迭代次数
    int minPointsForPlane = 10;        // 拟合平面最少点数
};

class LaserCalibration {
public:
    LaserCalibration();

    // 设置参数
    void setBoardSize(int w, int h);
    void setSquareSize(float size);
    void setCameraParams(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    // 核心处理函数
    LaserProcessingResult processLaserPair(const cv::Mat& imgNoLaser,
                                           const cv::Mat& imgLaser,
                                           const cv::Mat& cameraMatrix,
                                           const cv::Mat& distCoeffs,
                                           const StegerParams& stegerParams,
                                           const LABParams& labParams = LABParams());

    // 直线分析辅助函数
    LineAnalysisResult analyzeLaserLine(const std::vector<cv::Point2f>& points);

    // 激光平面标定函数
    LaserPlaneCalibrationResult calibrateLaserPlane(
        const std::vector<LaserPair>& laserData,
        const cv::Mat& P1, const cv::Mat& P2,
        float depthMin = 100.0f, float depthMax = 2000.0f);

    LaserPlaneCalibrationResult calibrateLaserPlaneWithProgress(
        const std::vector<LaserPair>& laserData,
        const cv::Mat& P1, const cv::Mat& P2,
        const CalibrationParams& params,
        std::function<void(int, int)> progressCallback = nullptr
    );

private:
    cv::Size m_boardSize;
    float m_squareSize;
    
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

    std::vector<cv::Point2f> detectLaserCenter_LAB_Steger(
        const cv::Mat& roiLaser, 
        const cv::Mat& roiMask, 
        const LABParams& labParams, 
        const StegerParams& stegerParams, // <-- 新增的 Steger 参数
        float downScale = 1.0f
    );
    
    static float pointToLineDistance(const cv::Point2f& pt, const cv::Point2f& linePt1, const cv::Point2f& linePt2);
    std::vector<cv::Point2f> detectLaserCenter_Fast(const cv::Mat& channel, const StegerParams& params);
    
    // 【新增】RANSAC直线拟合函数声明
    LineAnalysisResult ransacLineFitting(const std::vector<cv::Point2f>& points, 
                                      float distanceThreshold = 2.0f, 
                                      int maxIterations = 100);
};

#endif
