#include "core/pointcalibrator.h"

void PointCalibrator::setStereoParams(const cv::Mat& K_L, const cv::Mat& D_L, const cv::Mat& K_R, const cv::Mat& D_R,
                                      const cv::Mat& R, const cv::Mat& T, const cv::Mat& P1, const cv::Mat& P2, bool is_rectified) {
    m_K_L = K_L.clone(); m_D_L = D_L.clone(); m_K_R = K_R.clone(); m_D_R = D_R.clone();
    m_R = R.clone(); m_T = T.clone(); m_P1 = P1.clone(); m_P2 = P2.clone();
    m_is_rectified = is_rectified;
}

SiftMeasureResult PointCalibrator::measureSinglePoint(const cv::Mat& imgL, const cv::Mat& imgR) {
    SiftMeasureResult res;
    if (imgL.empty() || imgR.empty() || m_K_L.empty() || m_R.empty() || m_T.empty()) {
        res.error_msg = "图像或相机参数为空"; return res;
    }

    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(500);
    std::vector<cv::KeyPoint> kpL, kpR;
    cv::Mat descL, descR;
    sift->detectAndCompute(imgL, cv::Mat(), kpL, descL);
    sift->detectAndCompute(imgR, cv::Mat(), kpR, descR);

    if (descL.empty() || descR.empty()) {
        res.error_msg = "未提取到SIFT特征"; return res;
    }

    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(descL, descR, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i].size() >= 2 && knn_matches[i][0].distance < 0.75f * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    if (good_matches.empty()) {
        res.error_msg = "无有效匹配点"; return res;
    }

    // 取距离最小的最优匹配
    auto min_it = std::min_element(good_matches.begin(), good_matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
        return a.distance < b.distance;
    });

    res.pt_left = kpL[min_it->queryIdx].pt;
    res.pt_right = kpR[min_it->trainIdx].pt;

    // ================= 核心算法修复区 =================
    
    // 1. 将原始像素坐标 -> 去畸变的归一化相机坐标 (此时去除了内参K的影响)
    std::vector<cv::Point2f> ptL_vec = {res.pt_left};
    std::vector<cv::Point2f> ptR_vec = {res.pt_right};
    std::vector<cv::Point2f> ptL_undist, ptR_undist;
    
    cv::undistortPoints(ptL_vec, ptL_undist, m_K_L, m_D_L);
    cv::undistortPoints(ptR_vec, ptR_undist, m_K_R, m_D_R);

    // 2. 构造严格 3x4 的投影矩阵 (因为坐标已归一化，不再需要乘 K 矩阵)
    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F); // 左相机: [I | 0]
    
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F); // 右相机: [R | T]
    m_R.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    m_T.copyTo(P2(cv::Rect(3, 0, 1, 3)));

    // 3. 执行三角化 (输出的三维点直接处于左相机坐标系下，单位与标定时的 T 一致，通常是 mm)
    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, ptL_undist, ptR_undist, pts4D);

    float w = pts4D.at<float>(3, 0);
    if (std::abs(w) < 1e-6) { res.error_msg = "三角化权重异常"; return res; }

    res.point_3d.x = pts4D.at<float>(0, 0) / w;
    res.point_3d.y = pts4D.at<float>(1, 0) / w;
    res.point_3d.z = pts4D.at<float>(2, 0) / w;
    res.distance = cv::norm(res.point_3d);
    
    if (res.point_3d.z < 0) { res.error_msg = "深度为负(可能在相机后方)"; return res; }
    
    res.success = true;
    return res;
}
