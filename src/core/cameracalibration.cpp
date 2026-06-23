#include "core/cameracalibration.h"
#include <QDebug>
#include <fstream>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include <map> // 用于双目匹配

CameraCalibration::CameraCalibration()
    : squareSize(10.0f)
{
    boardSize = cv::Size(11, 8);
}

void CameraCalibration::setBoardSize(int width, int height)
{
    boardSize.width = width;
    boardSize.height = height;
}

void CameraCalibration::setSquareSize(float size)
{
    squareSize = size;
}

void CameraCalibration::removeImage(int index)
{
    if (index >= 0 && index < imageItems.size()) {
        imageItems.erase(imageItems.begin() + index);
        // 保持底层向量的同步 (按照 imageItems 同步清理)
        if (!imagePoints.empty() && index < imagePoints.size()) imagePoints.erase(imagePoints.begin() + index);
        if (!objectPoints.empty() && index < objectPoints.size()) objectPoints.erase(objectPoints.begin() + index);
    }
}

void CameraCalibration::clear()
{
    imageItems.clear();
    objectPoints.clear();
    imagePoints.clear();
    resultStr.clear();
    imageSize = cv::Size();
    cameraMatrix.release();
    distCoeffs.release();
    stereoR.release();
    stereoT.release();
    stereoE.release();
    stereoF.release();
}

// =====================================================================
// 核心私有方法：统一的角点检测与精细化
// =====================================================================
bool CameraCalibration::detectAndRefineCorners(const cv::Mat& grayImg, std::vector<cv::Point2f>& outCorners)
{
    if (grayImg.empty()) return false;

    // 1. 统一的下采样策略 (基于最长边，无论横拍竖拍都能稳定加速)
    cv::Mat workingImg = grayImg;
    double scale = 1.0;
    const int MAX_SIDE = 800;
    int maxDim = std::max(grayImg.cols, grayImg.rows);
    
    if (maxDim > MAX_SIDE) {
        scale = (double)MAX_SIDE / maxDim;
        cv::resize(grayImg, workingImg, cv::Size(), scale, scale, cv::INTER_LINEAR);
    }

    // 2. 粗定位
    bool found = cv::findChessboardCorners(workingImg, boardSize, outCorners,
                        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (!found) return false;

    // 3. 坐标映射回原图
    if (scale != 1.0) {
        for (auto& pt : outCorners) {
            pt.x /= scale;
            pt.y /= scale;
        }
    }

    // 4. 安全的亚像素精细化 (直接在原图全图做)
    // 废弃之前的 ROI 截断法：因为当角点贴近图片边缘时，ROI 会导致 cornerSubPix 搜索窗口越界报错。
    // 现代OpenCV在全图做11x11的cornerSubPix速度极快，无需用ROI优化。
    cv::cornerSubPix(grayImg, outCorners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));

    return true;
}

// =====================================================================
// 统一图片处理入口
// =====================================================================

// 支持直接传入 Mat
bool CameraCalibration::processImage(const cv::Mat &image, const std::string &identifier)
{
    if (image.empty()) return false;
    if (imageSize.empty()) imageSize = image.size();

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    std::vector<cv::Point2f> corners;
    if (!detectAndRefineCorners(gray, corners)) {
        // 检测失败，记录空项保持索引一致（可选），或直接返回 false
        return false; 
    }

    // 构造对应的 3D 物理坐标点
    std::vector<cv::Point3f> objPts;
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            objPts.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));

    // 统一存储，解决之前 addImage 和 processImage 数据不同步的致命问题
    CalibImageItem item;
    item.filePath = identifier;
    item.found = true;
    item.corners = corners;

    imageItems.push_back(item);
    imagePoints.push_back(corners);
    objectPoints.push_back(objPts);

    return true;
}

// 支持传入文件路径
bool CameraCalibration::processImage(const std::string &filePath)
{
    cv::Mat image = cv::imread(filePath);
    if (image.empty()) return false;
    return processImage(image, filePath);
}

// =====================================================================
// 标定执行与统计
// =====================================================================
bool CameraCalibration::runCalibration()
{
    std::vector<int> validIndices;
    std::vector<std::vector<cv::Point2f>> validImagePoints;
    std::vector<std::vector<cv::Point3f>> validObjectPoints;
    
    for(size_t i = 0; i < imageItems.size(); ++i) {
        if(imageItems[i].found) {
            validIndices.push_back(i);
            validImagePoints.push_back(imageItems[i].corners);
            validObjectPoints.push_back(objectPoints[i]);
        }
    }

    if (validImagePoints.size() < 3) {
        resultStr = "错误：有效标定图片不足 3 张，无法标定。";
        return false;
    }

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    
    // 执行标定
    double totalRMS = cv::calibrateCamera(validObjectPoints, validImagePoints, imageSize,
                                     cameraMatrix, distCoeffs, rvecs, tvecs, 0);

    // 计算每张图的详细误差与统计
    for (size_t i = 0; i < validIndices.size(); ++i) {
        int originalIdx = validIndices[i];
        cv::Rodrigues(rvecs[i], imageItems[originalIdx].R); // 存储旋转矩阵
        imageItems[originalIdx].t = tvecs[i].clone();

        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(validObjectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projectedPoints);
        
        double ssRes = 0, ssTot = 0;
        int count = projectedPoints.size();
        double sumX = 0, sumY = 0;
        for(int k=0; k<count; ++k) { 
            sumX += imageItems[originalIdx].corners[k].x; 
            sumY += imageItems[originalIdx].corners[k].y; 
        }
        cv::Point2f meanObs(sumX/count, sumY/count);
        
        for(int k=0; k<count; ++k) {
            double dx = imageItems[originalIdx].corners[k].x - projectedPoints[k].x;
            double dy = imageItems[originalIdx].corners[k].y - projectedPoints[k].y;
            ssRes += dx*dx + dy*dy;
            double dxt = imageItems[originalIdx].corners[k].x - meanObs.x;
            double dyt = imageItems[originalIdx].corners[k].y - meanObs.y;
            ssTot += dxt*dxt + dyt*dyt;
        }
        imageItems[originalIdx].reprojError = sqrt(ssRes / count);
        imageItems[originalIdx].rSquared = (ssTot > 0) ? (1.0 - (ssRes / ssTot)) : 0;
    }

    // === 生成详细的统计字符串 ===
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "标定完成! 有效图片: " << validImagePoints.size() << " 张\n";
    ss << "整体RMS误差: " << totalRMS << " 像素\n\n";
    ss << "=== 相机内参矩阵 ===\n" << cameraMatrix << "\n\n";
    ss << "=== 畸变系数 ===\n" << distCoeffs << "\n\n";

    ss << "=== 各图片重投影误差 ===\n";
    for (int idx : validIndices) {
        std::string filename = imageItems[idx].filePath.substr(imageItems[idx].filePath.find_last_of("/\\") + 1);
        ss << filename << ": " << imageItems[idx].reprojError << " px\n";
    }
    ss << "\n";

    // 修正：位姿丰富度 (计算真正的标准差)
    ss << "=== 位姿丰富度分析 ===\n";
    cv::Mat meanC = cv::Mat::zeros(3, 1, CV_64F);
    std::vector<cv::Mat> cameraCenters;
    for (int idx : validIndices) {
        cv::Mat C = -imageItems[idx].R.t() * imageItems[idx].t; // 相机光心坐标
        cameraCenters.push_back(C);
        meanC += C;
    }
    meanC /= validIndices.size();

    double variance = 0;
    for (const auto& C : cameraCenters) {
        cv::Mat diff = C - meanC;
        variance += diff.dot(diff); // 真实的方差公式：差的平方和
    }
    double stdDev = std::sqrt(variance / validIndices.size()); // 标准差 = sqrt(方差)
    // 修正单位提示：不再默认写 cm，取决于用户的 squareSize 单位
    ss << "相机位置标准差: " << stdDev << " (单位与 squareSize 一致)\n\n";
    
    resultStr = ss.str();
    qDebug().noquote() << QString::fromStdString(resultStr);
    
    return true;
}

// =====================================================================
// 单张图位姿计算
// =====================================================================
bool CameraCalibration::computeSingleImagePose(const cv::Mat &image, cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &corners)
{
    if (cameraMatrix.empty() || distCoeffs.empty()) return false;
    if (image.empty()) return false;

    cv::Mat gray;
    if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else gray = image;

    // 复用核心检测逻辑
    if (!detectAndRefineCorners(gray, corners)) return false;

    std::vector<cv::Point3f> objectCorners;
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            objectCorners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));

    // 输出旋转向量，不在这里转矩阵，交由调用者决定
    return cv::solvePnP(objectCorners, corners, cameraMatrix, distCoeffs, rvec, tvec);
}

bool CameraCalibration::computeSingleImagePose(const std::string &filePath, cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &corners)
{
    cv::Mat img = cv::imread(filePath);
    return computeSingleImagePose(img, rvec, tvec, corners);
}

// =====================================================================
// 双目标定执行
// =====================================================================
bool CameraCalibration::runStereoCalibration(const CameraCalibration& rightCamera)
{
    if (cameraMatrix.empty() || rightCamera.cameraMatrix.empty()) {
        qDebug() << "双目标定失败：请确保左右相机均已单独完成标定！";
        return false;
    }

    // 根据 filePath 进行严格匹配，确保左右图是对应的
    std::map<std::string, size_t> leftMap;
    for(size_t i=0; i<imageItems.size(); ++i) {
        if(imageItems[i].found) leftMap[imageItems[i].filePath] = i;
    }

    std::vector<std::vector<cv::Point2f>> stereoLeftPoints;
    std::vector<std::vector<cv::Point2f>> stereoRightPoints;
    std::vector<std::vector<cv::Point3f>> stereoObjectPoints;

    for(const auto& rItem : rightCamera.imageItems) {
        if(!rItem.found) continue;
        auto it = leftMap.find(rItem.filePath);
        if(it != leftMap.end()) {
            stereoLeftPoints.push_back(imageItems[it->second].corners);
            stereoRightPoints.push_back(rItem.corners);
            // 2. 新增：直接复用之前 processImage 时存好的 objectPoints
            stereoObjectPoints.push_back(objectPoints[it->second]);
        }
    }

    if (stereoLeftPoints.size() < 5) { // 双目通常需要更多同步图片
        qDebug() << "双目标定失败：匹配到的有效双目图片对不足 5 对。请确保左右图片文件名完全一致。";
        return false;
    }

    // 假设双目图片大小一致，取左图的 imageSize
    cv::Size stereoSize = imageSize; 

    // 执行双目标定 (固定内参，只求 R, T, E, F)
    double rms = cv::stereoCalibrate(
        stereoObjectPoints,            // <-- 补上遗漏的 3D 点
        stereoLeftPoints, stereoRightPoints,
        cameraMatrix, distCoeffs,
        rightCamera.cameraMatrix, rightCamera.distCoeffs,
        stereoSize, 
        stereoR, stereoT, stereoE, stereoF,
        cv::CALIB_FIX_INTRINSIC, 
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1e-6)
    );

    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "=== 双目标定完成 ===\n";
    ss << "使用图片对数: " << stereoLeftPoints.size() << "\n";
    ss << "双目RMS重投影误差: " << rms << " 像素\n\n";
    ss << "=== 旋转矩阵 (R) ===\n" << stereoR << "\n\n";
    ss << "=== 平移向量 (T) ===\n" << stereoT << "\n\n";
    
    qDebug().noquote() << QString::fromStdString(ss.str());
    resultStr += "\n" + ss.str(); // 追加到原有结果后

    return true;
}
