#ifndef POINTCALIBRATOR_H
#define POINTCALIBRATOR_H

#include <opencv2/opencv.hpp>
#include <QString>

struct SiftMeasureResult {
    bool success = false;
    cv::Point2f pt_left;
    cv::Point2f pt_right;
    cv::Point3d point_3d;
    double distance = 0.0;
    QString error_msg;
};

class PointCalibrator {
public:
    PointCalibrator() = default;
    
    void setStereoParams(const cv::Mat& K_L, const cv::Mat& D_L, const cv::Mat& K_R, const cv::Mat& D_R,
                         const cv::Mat& R, const cv::Mat& T);

    SiftMeasureResult measureSinglePoint(const cv::Mat& imgL, const cv::Mat& imgR);

private:
    // 只保留测量实际用到的参数 (P1/P2/is_rectified 未使用，已移除)
    cv::Mat m_K_L, m_D_L, m_K_R, m_D_R, m_R, m_T;
};

#endif // POINTCALIBRATOR_H
