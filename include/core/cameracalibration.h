#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// 存储每一张标定图片的信息
struct CalibImageItem {
    std::string filePath;
    bool found = false; // 初始化默认值
    std::vector<cv::Point2f> corners;
    cv::Mat R; // 旋转矩阵 (3x3) - 注意：不是旋转向量
    cv::Mat t; // 平移向量 (3x1)
    double reprojError = 0.0;
    double rSquared = 0.0;
};

class CameraCalibration
{
public:
    CameraCalibration();

    // ==================== 参数设置与获取 ====================
    void setBoardSize(int width, int height);
    void setSquareSize(float size);

    cv::Size getBoardSize() const { return boardSize; }
    float getSquareSize() const { return squareSize; }
    cv::Size getImageSize() const { return imageSize; }

    // ==================== 单目结果获取 ====================
    const std::vector<CalibImageItem>& getImageItems() const { return imageItems; }
    cv::Mat getCameraMatrix() const { return cameraMatrix; }
    cv::Mat getDistCoeffs() const { return distCoeffs; }
    std::string getResultString() const { return resultStr; }

    // ==================== 双目结果获取 ====================
    // 修改命名：避免与局部变量、结构体成员冲突
    cv::Mat getStereoR() const { return stereoR; }
    cv::Mat getStereoT() const { return stereoT; }
    cv::Mat getStereoE() const { return stereoE; }
    cv::Mat getStereoF() const { return stereoF; }

    // ==================== 核心操作 ====================
    
    // 统一的图片处理入口 (替换掉之前有逻辑漏洞的 addImage)
    bool processImage(const std::string &filePath);
    bool processImage(const cv::Mat &image, const std::string &identifier = "raw_image"); // 新增：支持直接传入Mat
    
    void removeImage(int index);
    void clear();
    
    // 标定执行
    bool runCalibration();
    bool runStereoCalibration(const CameraCalibration& rightCamera);

    // 计算单张图片位姿 (参数名改为 rvec/tvec 避免与类成员冲突)
    bool computeSingleImagePose(const cv::Mat &image, cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &corners);
    bool computeSingleImagePose(const std::string &filePath, cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &corners);

private:
    // ==================== 标定板参数 ====================
    cv::Size boardSize;
    float squareSize;
        
    // ==================== 内部核心算法 (DRY 原则) ====================
    // 统一的角点检测与亚像素精细化逻辑
    bool detectAndRefineCorners(const cv::Mat& grayImg, std::vector<cv::Point2f>& outCorners);

    // ==================== 数据存储 ====================
    std::vector<CalibImageItem> imageItems;
    std::vector<std::vector<cv::Point2f>> imagePoints;   // 与 imageItems 同步
    std::vector<std::vector<cv::Point3f>> objectPoints;  // 与 imageItems 同步
    cv::Size imageSize; // 假设所有标定图片尺寸一致

    // ==================== 单目标定结果 ====================
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::string resultStr;

    // ==================== 双目标定结果 (重命名防冲突) ====================
    cv::Mat stereoR; // 双目旋转矩阵
    cv::Mat stereoT; // 双目平移向量
    cv::Mat stereoE; // 本质矩阵
    cv::Mat stereoF; // 基础矩阵
};

#endif // CAMERACALIBRATION_H
