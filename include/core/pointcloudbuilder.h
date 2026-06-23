#ifndef POINTCLOUD_BUILDER_H
#define POINTCLOUD_BUILDER_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Core>
#include <vector>
#include <QString>

// ============================================================================
// 三维重建参数统一管理结构体
// ============================================================================
enum S1Method {
    S1_STEGER_MASK   = 0,  // Steger脊线检测 + A通道掩膜 (实验3最优)
    S1_CENTROID_MASK = 1,  // 灰度重心 + 掩膜 (当前)
    S1_COLUMN_MAX    = 2   // 纯列极值 (实验6, 无掩膜)
};

struct ReconstructionParams
{
    // ==================== S1: 光条中心提取参数 ====================
    // --- 通用掩膜参数 ---
    int s1_method = 0;                  // S1方法: 0=Steger+掩膜 1=灰度重心+掩膜 2=列极值
    int lab_a_threshold = 160;          // LAB A通道阈值 (红光判定)
    double min_segment_length = 5.0;    // 最小线段长度 (水平切片降级时使用)
    int sample_step = 1;                // 行降采样步长 (水平切片降级时使用)

    // --- Steger 算法核心参数 ---
    bool use_steger = true;             // 兼容旧版 (被s1_method替代)
    float steger_sigma = 1.2f;          // Hessian 高斯窗口
    float steger_t_max = 0.6f;          // 泰勒偏移距离阈值 (像素)，越小越严格防跨线

    // --- Steger 高反光过曝恢复参数 ---
    bool steger_edge_offset_enable = true;      // 是否启用”高亮中心由两侧边缘恢复”
    int steger_overexposed_l_thresh = 235;      // LAB L通道(亮度)过曝判定阈值
    float steger_edge_offset_sigma = 1.2f;      // 过曝时法线偏移系数
    float steger_overexposed_max_offset = 6.0f;  // 过曝恢复最大搜索步数 (迭代步长0.5px)

    // --- 兼容旧版 UI 残留字段 ---
    double gray_deadzone = 0.0;
    double weight_sum_threshold = 0.0;

    cv::Rect roi_left;                   
    cv::Rect roi_right;   

    // ==================== S2: 双目极线约束匹配参数 ====================
    double epipolar_threshold = 5.0;            // 极线匹配容差 (像素)
    float  depth_min = 10.0f;                  // 最小有效深度
    float  depth_max = 500.0f;                 // 最大有效深度
    int    min_points_per_segment = 5;         // 单段最少点数
    float  segment_break_dist = 8.0f;          // 切分线段的最大间断距离
    double dp_skip_penalty = 15.0;             // DP 跳过代价
    double dp_smooth_weight = 0.5;             // 视差平滑系数
    float disparity_break_threshold = 15.0f;   // 视差跳变切断阈值 (单位：像素)

    // ==================== S5: 多视角ICP配准参数 ====================
    bool   use_icp = false;                          // false=纯轴旋转, true=ICP精配准
    double icp_max_correspondence_distance = 3.0;   // ICP 最大对应点距离 (稀疏点云需 ≥2mm)
    double icp_axis_trust = 95.0;                   // ICP轴信任度(%): 100=完全信任轴(ICP仅修正平移), 0=完全信任ICP
    double icp_euclidean_fitness_epsilon = 0.001;   // ICP 欧式适应度收敛阈值
    double icp_translation_epsilon = 1e-8;          // ICP 平移收敛阈值
    double icp_rotation_epsilon = 1e-8;             // ICP 旋转收敛阈值
    int    icp_max_iterations = 20;                 // ICP 最大迭代次数

    // ==================== S6: 去噪/滤波与网格化参数 ====================
    int    sor_mean_k = 10;                   // 统计滤波 K 邻域
    double sor_std_dev_mul = 3.0;             // 统计滤波标准差倍数
    double voxel_leaf_size = 0.0;             // 体素大小 (设为 0.0 则不进行体素滤波)
    int    poisson_depth = 9;                 // 泊松重建深度 (9=512³, 高分辨率捕捉曲面细节)
    float  poisson_point_weight = 3.0f;       // 泊松点权重
    double gp3_radius = 2.0;                  // 贪婪投影三角化 (GP3) 搜索半径
    double mesh_truncation_distance = 3.0;    // 网格截断距离 (0=不截断): 移除离点云超过此距离的网格面
    int    mesh_method = 1;                   // 网格方法: 0=泊松(水密), 1=GP3(按点云,适合非闭合)
};

// ============================================================================
// 标定参数结构体 (由外部/文件载入传入)
// ============================================================================
struct CalibrationData 
{
    // 双目相机参数
    cv::Mat cameraMatrixL;   
    cv::Mat distCoeffL;      
    cv::Mat cameraMatrixR;   
    cv::Mat distCoeffR;      
    cv::Mat R_stereo;        // 双目旋转矩阵 (右相对左)
    cv::Mat T_stereo;        // 双目平移向量 (右相对左)

    // 光平面参数 (方程: ax + by + cz + d = 0)
    Eigen::Vector4d laser_plane_coeff; 

    cv::Mat P1_rectified;  // 左相机校正后投影矩阵
    cv::Mat P2_rectified;  // 右相机校正后投影矩阵
    bool is_rectified;     // 是否已校正标志
    
    cv::Mat P1; // 立体校正后的左投影矩阵
    cv::Mat P2; // 立体校正后的右投影矩阵

    // 转台标定参数
    cv::Mat R_rect_L;        // 左相机校正旋转 (原始→校正), 用于S1畸变校正
    cv::Mat R_rect_R;        // 右相机校正旋转 (原始→校正)
    cv::Mat R_cam2turntable; // 相机到世界坐标系的旋转
    cv::Mat T_cam2turntable; // 相机到世界坐标系的平移
    Eigen::Vector3d turntable_axis; // 转台旋转轴在转台坐标系下的单位向量
};

// ============================================================================
// PointCloudBuilder 核心算法类
// ============================================================================
class PointCloudBuilder
{
public:
    PointCloudBuilder();
    ~PointCloudBuilder();

    // 初始化与配置
    void setCalibrationData(const CalibrationData& calib_data);
    void setReconstructionParams(const ReconstructionParams& params);

    QString lastDetailLog;

    // ================= 核心流水线暴露接口 =================

    // 单视角处理: 封装 S1 -> S2 -> S3 -> S4
    // 输入：左右图像 + 当前转台旋转角度(度)
    // 输出：转台基准坐标系下的单线点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr processSingleView(
        const cv::Mat& img_left, 
        const cv::Mat& img_right, 
        double current_angle_deg);

    // 全局处理: 封装 S5 -> S6
    // 输入：所有单视角点云集合
    // 输出：最终的全局点云与网格 (存入内部，通过getter获取)
    void processGlobal(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& multi_view_clouds,
        const std::vector<double>& view_angles_deg,
        const Eigen::Vector3f& axis_point,
        const Eigen::Vector3f& axis_dir);

    // 获取处理结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFinalPointCloud() const;
    pcl::PolygonMesh getFinalMesh() const;

    // ================= 单帧调试专用暴露接口 (放行私有方法) =================
    
    // S1: 光条中心提取
    void extractLaserCenter(const cv::Mat& img, const cv::Rect& roi, std::vector<cv::Point2f>& center_points, int* mask_pixels_out = nullptr);

    // S2: 双目极线约束匹配
    void epipolarConstraintMatch(
        const std::vector<cv::Point2f>& pts_left, 
        const std::vector<cv::Point2f>& pts_right,
        std::vector<cv::Point2f>& matched_pts_left, 
        std::vector<cv::Point2f>& matched_pts_right);

    // S3: 三角化得单线轮廓点云 (相机坐标系)
    pcl::PointCloud<pcl::PointXYZ>::Ptr triangulatePoints(
        const std::vector<cv::Point2f>& pts_left,
        const std::vector<cv::Point2f>& pts_right);

    // S4: 坐标变换到转台基准坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformToTurntableFrame(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cam,
        double angle_deg);

private:
    CalibrationData m_calib_data;
    ReconstructionParams m_params;

    // 内部缓存结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_global_cloud;
    pcl::PolygonMesh m_global_mesh;

    // ================= S5, S6 具体算法实现声明 (保持私有) =================

    // S5: 多视角点云配准 (ICP)
    pcl::PointCloud<pcl::PointXYZ>::Ptr multiViewRegistration(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& multi_view_clouds,
        const std::vector<double>& view_angles_deg,
        const Eigen::Vector3f& axis_point,
        const Eigen::Vector3f& axis_dir);

    // S6-1: 去噪与滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoiseAndFilter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

    // S6-2: 网格化 (泊松重建)
    pcl::PolygonMesh meshReconstruction(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud);

    // 底面间隙填充：RANSAC检测转台平面，在稀疏区补点辅助泊松闭合底面
    pcl::PointCloud<pcl::PointXYZ>::Ptr fillBottomGaps(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

#endif // POINTCLOUD_BUILDER_H