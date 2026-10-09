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

    // --- 红光掩膜 (HSV), 仅 s1_method 0/1 使用 ---
    /**
     * 色相容差 (OpenCV 色相 0-180 标度)。红色在标度上首尾相接 (0 ≡ 180),
     * 所以窗口写成 [0, tol] ∪ [180-tol, 180], 即以纯红为中心向品红方向张开。
     *
     * 为什么必须能张开: 激光线越亮越白, 色相会从暗处的 ~174° 一路漂到 ~155°
     * (打在蓝色转盘上时更明显)。原先固定 tol=10 的窗只覆盖 170~180°,
     * 于是光条中下段整段被判成非红光而丢弃 —— 实测同一批 40 帧里,
     * 把 tol 从 10 放宽到 30 后 S1 左/右提取点数从 6555/4638 变成 12456/9990,
     * 且多出来的点能通过 S3 的光平面几何校验, 不是噪声。
     */
    int laser_hue_tol_deg = 30;
    int laser_min_sat     = 40;         // 最低饱和度 (0-255); 原为硬编码 60, 光条在亮处会掉到 50~70

    // --- Steger 算法核心参数 ---
    bool use_steger = true;             // 兼容旧版 (被s1_method替代)
    float steger_sigma = 1.2f;          // Hessian 高斯窗口
    float steger_t_max = 0.6f;          // 泰勒偏移距离阈值 (像素)，越小越严格防跨线

    // --- Steger 高反光过曝恢复参数 ---
    bool steger_edge_offset_enable = true;      // 是否启用”高亮中心由两侧边缘恢复”
    int steger_overexposed_l_thresh = 235;      // LAB L通道(亮度)过曝判定阈值
    float steger_edge_offset_sigma = 1.2f;      // 过曝时法线偏移系数
    float steger_overexposed_max_offset = 6.0f;  // 过曝恢复最大搜索步数 (迭代步长0.5px)

    // --- 【P0-3】S1 阈值参数化 (原先是散落在算法里的硬编码值) ---
    int steger_edge_dark_thresh = 200;      // 过曝恢复: 法线两侧"暗区/边缘"亮度上限
    int centroid_overexposed_l = 250;       // 灰度重心 & 列极值: 过曝判定亮度
    int centroid_edge_dark_thresh = 200;    // 灰度重心 & 列极值: 边缘扫描亮度上限

    // --- 红通道饱和恢复 (★ 见 pointcloudbuilder.cpp 里 extractLaserCenter 的说明) ---
    // 背景: 过曝判定原先只看 LAB 的 L 通道。但激光是**纯红**的 —— 红通道早就削顶到
    // 255 了, L 却只有 130~180 (红色在亮度上本来就暗), 于是"过曝恢复"整段不触发,
    // 平台区的中心改由绿/蓝通道(环境光、蓝色转台的反光)决定, 逐行乱跳。
    // 实测: 124 行里有 84 行红通道饱和; 饱和行的相邻行列号跳变 4.56px, 不饱和行 1.21px,
    // 差 3.8 倍。列号跳 4.5px ≈ 深度差 3.0mm ≈ 圆柱面的径向散布。
    // 所以饱和判定必须**分通道**: 谁削顶就按谁算。
    int sat_r_thresh = 250;                 // 红通道削顶判定 (激光在红通道)
    int edge_dark_r_thresh = 128;           // 红通道下的"边缘"水平 (取 255 的一半)
    int column_peak_min_l = 70;             // 纯列极值: 最小峰值亮度 (低于视为无光条)

    // --- 【P0-3】S1 逐点置信度 ---
    // 置信度定义: clamp(L/255) × (过曝恢复点 ? 0.75 : 1.0)，取值 (0,1]
    float min_point_confidence = 0.0f;      // 最低点置信度，0=不过滤 (过滤在 S2 匹配之前)
    float overexposed_conf_weight = 0.75f;  // 过曝恢复点的置信度折扣系数

    // --- 兼容旧版 UI 残留字段 ---
    double gray_deadzone = 0.0;
    double weight_sum_threshold = 0.0;

    cv::Rect roi_left;                   
    cv::Rect roi_right;   

    // ==================== S2: 双目极线约束匹配参数 ====================
    double epipolar_threshold = 5.0;            // 极线匹配容差 (像素)
    float  depth_min = 10.0f;                  // 最小有效深度
    float  depth_max = 500.0f;                 // 最大有效深度
    /**
     * S3 右图重投影一致性阈值, **单位毫米**。光平面只约束深度, 用右图再剔除 S2 错配点。
     * 判据是「光平面给出的 3D 点到右图那条匹配射线的空间距离」(= 像素误差 × z / f)。
     *
     * 为什么用 mm 而不是 px: 光平面偏差 δ 引起的重投影误差是 f·B·δ/(d·z), 随深度 1/z 衰减,
     * 所以固定像素阈值在工作距离 113~255mm 这段范围里相当于松紧差 2.3 倍
     * (同一个 40px 阈值, 近端等于 4.5mm、远端等于 10mm)。换成 mm 就与深度无关了。
     *
     * 默认 8mm 来自实测: 匹配按「双目 DLT 三角化点到光平面的距离」分得很干净 ——
     * 自洽匹配 ≤5mm, 伪匹配 ≥34mm, 中间是空的。落在空档里的任何值结果都一样。
     */
    double reproj_reject_mm = 8.0;
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

    /**
     * 回环校正方式 (S5 阶段2)。转台转满一圈时, 末帧理论上应与首帧重合,
     * 二者的偏差就是整圈的累计漂移, 需要摊回各帧。
     *  - LoopClosureSpread = 0 (默认): 用"增量累计变换偏离恒等的量"沿标定轴均摊。
     *                                  不依赖点云质量, 当前生产行为。
     *  - LoopClosureIcp    = 1: 用末帧↔首帧的 ICP 残差作误差再均摊 (数据好时更准,
     *                          但 360° 重合时 ICP 可能收敛到 0° 而失效)。
     *  - LoopClosureOff    = 2: 不做回环校正 (纯增量累加)。
     */
    enum LoopClosure { LoopClosureSpread = 0, LoopClosureIcp = 1, LoopClosureOff = 2 };
    int loop_closure = LoopClosureSpread;           // 回环校正方式

    // ==================== S6: 去噪/滤波与网格化参数 ====================
    int    sor_mean_k = 10;                   // 统计滤波 K 邻域
    double sor_std_dev_mul = 3.0;             // 统计滤波标准差倍数
    double voxel_leaf_size = 0.0;             // 体素大小 (0.0=自动按点云中位间距估计)
    int    poisson_depth = 9;                 // 泊松重建深度 (9=512³, 高分辨率捕捉曲面细节)
    float  poisson_point_weight = 3.0f;       // 泊松点权重
    double gp3_radius = 2.0;                  // 贪婪投影三角化 (GP3) 搜索半径
    double mesh_truncation_distance = 3.0;    // 网格截断距离 (0=不截断): 移除离点云超过此距离的网格面
    int    mesh_method = 1;                   // 网格方法: 0=泊松(水密), 1=GP3(按点云,适合非闭合)

    // ==================== 工程: 并行 ====================
    int parallel_threads = 0;                 // S1~S4 逐视角并行线程数 (0=按CPU核心自动)
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
    // 【必须零初始化】Eigen 默认构造不置零；若某条注入路径(如 Tab4 棋盘格3D验证)
    // 未显式赋值，随机值会被 has_plane 误判为"存在光平面"，使 S3 走错分支。
    Eigen::Vector4d laser_plane_coeff = Eigen::Vector4d::Zero();

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
    Eigen::Vector3d turntable_axis = Eigen::Vector3d::Zero(); // 转台旋转轴在转台坐标系下的单位向量
};

// ============================================================================
// PointCloudBuilder 核心算法类
// ============================================================================

/**
 * 逐视角处理结果的**汇总**统计 (S1~S4)。
 *
 * 为什么要有它: 这些计数原来是在每个视角的终端日志里逐条打印的, 200 视角就是
 * 200 遍, 纯刷屏。收进这里的目的是让调用方在整段重建结束后**汇总打印一次**,
 * 既保住"多少视角失败了、失败在哪一步"的诊断能力, 又不产生逐视角噪声。
 *
 * 每个 PointCloudBuilder 实例只被一个线程使用 (worker 分块私有实例),
 * 所以这里用普通整数即可, 不需要原子量。
 */
/**
 * 单个视角的处理明细 (S1~S4)。
 *
 * 为什么在"汇总统计"之外还要逐视角留一份: 汇总只说明"整段丢了多少",
 * 回答不了"是均匀地丢, 还是某几帧整段没出点"。Tab6 的「快速产出测试」要画的
 * 逐帧产出曲线就依赖它 —— 例如某帧 S2 匹配数正常但 S3 有效点数为 0,
 * 那就不是提取/匹配的问题, 而是该帧三角化后全被剔除。
 */
struct ViewRecord {
    int    frameIdx  = -1;   // 视角序号 (-1 = 调用方未提供)
    double angleDeg  = 0.0;  // 该视角的转台角
    int    s1Left    = 0;    // S1 左图提取的候选点数
    int    s1Right   = 0;    // S1 右图提取的候选点数
    int    s2Match   = 0;    // S2 匹配对数
    /**
     * 该视角被合并掉的**多余右点匹配数** (S2)。
     *
     * 背景: 同一个右图光条点曾经被 2~7 个左点各配一次, 每次算出一个不同的 3D 点,
     * 组内深度中位相差 2.6mm —— 这是"圆柱面被抹厚"的一个直接来源。
     * 现在 S2 收口成严格一对一, 这里记的就是收掉了多少。
     * 留这个数是为了让"收口"这件事**看得见**: 它应当长期为 0 (根因在阶段 3 已解决),
     * 一旦重新变大, 说明分段窗口又开始重叠了。
     */
    int    s2DupMerged = 0;
    int    s3Points  = 0;    // S3+S4 后该视角的有效点数
    int    reprojReject = 0; // 该视角被光平面一致性(DLT点到光平面的距离)剔除的点数
    int    negDepth  = 0;    // 该视角交点在相机后方
    int    tooFar    = 0;    // 该视角深度越界
    int    degenerate = 0;   // 该视角射线与光平面平行
    bool   ok        = false;// 是否产出至少一个点

    /**
     * 逐视角的右图重投影误差直方图 (桶同 ViewOutcomeStats::reprojBucket)。
     * 汇总直方图会把"每帧都差一点"和"少数帧差很多"混成同一个形状,
     * 而这两种情况的处理完全不同, 所以逐帧也留一份。
     */
    int    reprojHist[8] = {0,0,0,0,0,0,0,0};

    // --- 光条提取质量 (S1) ---
    // 光条点数是"提了多少", 掩膜像素数是"本来有多少红光", 两者相差悬殊就说明掩膜把人判掉了。
    // 注意: 只有 Steger/灰度重心 (s1_method 0/1) 才有红光掩膜; 列极值法下恒为 0。
    int    s1LeftMask  = 0;   // 左图红光掩膜的像素数, 已扣除伪连通域 (0 = 该方法不做掩膜)
    int    s1RightMask = 0;   // 右图红光掩膜的像素数, 已扣除伪连通域
    // 被"伪光条剔除"丢掉的掩膜像素数 (见 extractLaserCenter)。
    // 为什么要单独记: 它是一段**会默默改变结果**的筛选, 必须能从留档里看见它动过手。
    // 若某帧这个数突然变大, 多半是那一帧主光条被遮得很短, 相对阈值跟着缩了。
    int    s1LeftMaskDropped  = 0;
    int    s1RightMaskDropped = 0;
    // 其中有多少点的中心是**由饱和恢复反推**的 (两侧边缘取中点), 而不是脊线拟合。
    // 实测约占七成 —— 这个比例必须能从留档里看见: 一旦恢复逻辑失效(例如激光功率变了、
    // 饱和阈值设偏了), 症状只是"点云变厚", 不记下来就查不出是它。
    int    s1LeftSatRecover  = 0;
    int    s1RightSatRecover = 0;
    // 提取点在**原始图**上的行范围与最大行间隙。
    // 行范围回答"光条只提了哪一段"(例如只提到上半段);
    // 最大间隙回答"中间是不是断了一截"—— 两者都是"点数不够"背后的真实形态,
    // 光看点数分不出"整条稀疏"和"中间缺失"。
    int    rowMinLeft  = -1;  // -1 = 左图没提到点
    int    rowMaxLeft  = -1;
    int    maxGapLeft  = 0;   // 相邻提取点之间最大的空行数
    int    rowMinRight = -1;
    int    rowMaxRight = -1;
    int    maxGapRight = 0;
};

struct ViewOutcomeStats {
    int views        = 0;   // 进入处理的视角数
    int extractEmpty = 0;   // S1 光条提取为空
    int matchEmpty   = 0;   // S2 匹配 0 对
    int s2DupMerged  = 0;   // S2 因"多个左点共用同一个右点"而合并掉的多余匹配数 (见 ViewRecord)
    int triEmpty     = 0;   // S3 三角化后为空
    int negDepth     = 0;   // 交点在相机后方 / 负深度
    int tooFar       = 0;   // 深度越出 [depth_min, depth_max]
    int degenerate   = 0;   // 两射线接近平行, DLT 无解
    int reprojReject = 0;   // 光平面一致性超限剔除 (DLT 点到光平面的距离)
    int points       = 0;   // 三角化有效点数
    /**
     * 一致性残差直方图 (px), 桶: <1 / 1-2 / 2-5 / 5-10 / 10-20 / 20-50 / 50-100 / ≥100。
     *
     * 口径: DLT 点到光平面的距离(mm) 折算成"该深度处的等效重投影误差" —— 保留 px 是为了
     * 与历史日志同口径可比。所以它反映的是**光平面与双目的偏离程度**, 不再是重投影误差。
     *
     * 为什么需要这个分布: 只报一个"剔除 23309" 无法判断病根 ——
     *   · 残差集中在 5~10px (约 1~2mm) → 光平面标定略有偏差, 属正常噪声
     *   · 残差高达几十~几百 px → 光平面基本是错的, 或 S2 大量错配
     * 两种情况的处理完全不同, 所以把分布留下来。
     */
    int reprojHist[8] = {0,0,0,0,0,0,0,0};

    static int reprojBucket(double e) {
        if (e <  1.0) return 0;
        if (e <  2.0) return 1;
        if (e <  5.0) return 2;
        if (e < 10.0) return 3;
        if (e < 20.0) return 4;
        if (e < 50.0) return 5;
        if (e < 100.0) return 6;
        return 7;
    }
    // 观测到的深度范围 (mm): 用来判断 depth_min/depth_max 参数是否把有效点挡掉了
    double depthMinMm = 1e30;
    double depthMaxMm = -1e30;

    // 逐视角明细 (见 ViewRecord 说明)
    std::vector<ViewRecord> records;

    void merge(const ViewOutcomeStats& o) {
        views += o.views; extractEmpty += o.extractEmpty; matchEmpty += o.matchEmpty;
        s2DupMerged += o.s2DupMerged;
        triEmpty += o.triEmpty; negDepth += o.negDepth; tooFar += o.tooFar;
        degenerate += o.degenerate; reprojReject += o.reprojReject; points += o.points;
        for (int i = 0; i < 8; ++i) reprojHist[i] += o.reprojHist[i];
        if (o.depthMinMm < depthMinMm) depthMinMm = o.depthMinMm;
        if (o.depthMaxMm > depthMaxMm) depthMaxMm = o.depthMaxMm;
        // 注意: 线程分块并行时 merge 进来的 records 顺序是"按块交错"的, 不是全局帧序。
        // 调用方若要按帧号画曲线, 必须自己按 frameIdx 排序 (worker 已在其内部按视角序号收集)。
        records.insert(records.end(), o.records.begin(), o.records.end());
    }
};

/**
 * 单对匹配的几何明细 —— **只在调用方显式要求时才填**(诊断包用), 正常重建不产生任何开销。
 *
 * 为什么需要它: 正式重建路径上这些量算完就丢, 只留聚合计数。于是"点云大量丢失"时
 * 只能看到"S3 剔除了 26256 个", 分不清是【匹配本身错了】还是【光平面不准】还是
 * 【阈值偏紧】—— 三者的处理完全不同。把每个匹配的几何量落盘, 才能用
 * `dltPlaneResidMm`(独立于判据的几何真值)去反查实际判据是否合理。
 */
struct MatchDiag {
    cv::Point2f left, right;         // 立体校正系下的匹配对 (px)
    double depthMm        = 0.0;     // 左射线∩光平面 得到的深度
    double dispPx         = 0.0;     // 视差 (u_L - u_R)
    double reprojPx       = 0.0;     // 该 3D 点投回右图的像素误差
    double reprojMm       = 0.0;     // = reprojPx × z / f, S3 实际使用的判据量
    /**
     * 双目 DLT 三角化得到的点**到光平面的距离**(mm)。
     * 这是不依赖判据的独立几何真值: 光线求交用的光平面若准, 两条射线交出的点就该落在平面上。
     * 实测数据里它分得很干净 —— 自洽匹配 ≤5mm, 伪匹配 ≥34mm, 中间是空的。
     * <0 表示 DLT 求解失败(射线接近平行)。
     */
    double dltPlaneResidMm = -1.0;
    int    rejectReason   = -1;      // 见 RejectReason; -1 = 未走到判据(求交就失败了)
    bool   accepted       = false;
    // PlaneResid: DLT 点到光平面的距离超限 (现在的错配判据)。
    // ReprojMm 是旧判据(右图重投影)留下的取值, 已不再产生, 保留以免历史 CSV 解析错位。
    enum RejectReason { Accepted = 0, ReprojMm = 1, NegativeDepth = 2, TooFar = 3,
                        Degenerate = 4, DltFailed = 5, PlaneResid = 6 };
};

/**
 * 单个视角的过程明细。填充它需要在 S3 内部多留一份数据, 所以只对**样本帧**开启 ——
 * 212 帧全开会白白占内存, 而诊断只需要几帧就够看出病因。
 */
struct ViewDiag {
    // --- S1 提取点 ---
    std::vector<cv::Point2f> ptsLeftRaw,  ptsRightRaw;   // 原始畸变图坐标 (叠加图用)
    std::vector<cv::Point2f> ptsLeftRect, ptsRightRect;  // 立体校正系坐标 (几何分析用)
    // --- S2 的全部候选配对 (含最终被 S3 拒绝的) ---
    std::vector<MatchDiag> matches;
};

class PointCloudBuilder
{
public:
    PointCloudBuilder();
    ~PointCloudBuilder();

    /** 本实例累计的逐视角统计 (调用方可跨实例 merge 后汇总打印) */
    const ViewOutcomeStats& viewStats() const { return m_viewStats; }
    /** 逐视角明细 (Tab6「快速产出测试」曲线用; 单实例串行处理时即全局帧序) */
    const std::vector<ViewRecord>& viewRecords() const { return m_viewStats.records; }
    /** 清零统计 (新一轮重建开始时调用) */
    void resetViewStats() { m_viewStats = ViewOutcomeStats(); }

    // 初始化与配置
    void setCalibrationData(const CalibrationData& calib_data);
    void setReconstructionParams(const ReconstructionParams& params);

    QString lastDetailLog;

    // ================= 核心流水线暴露接口 =================

    // 单视角处理: 封装 S1 -> S2 -> S3 -> S4
    // 输入：左右图像 + 当前转台旋转角度(度)
    // 输出：转台基准坐标系下的单线点云
    // frameIdx: 视角序号, 仅用于逐视角明细 (viewRecords) 里标记来源帧; -1 = 不提供。
    //           并行分块时各块的 records 顺序是交错的, 有这个序号才能还原全局帧序。
    // diag:     非空时额外采集本视角的过程明细 (见 ViewDiag)。**只给样本帧用** ——
    //           它多留一份逐点数据, 全序列开启纯属浪费。正常重建传 nullptr 即零开销。
    pcl::PointCloud<pcl::PointXYZ>::Ptr processSingleView(
        const cv::Mat& img_left,
        const cv::Mat& img_right,
        double current_angle_deg,
        int frameIdx = -1,
        ViewDiag* diag = nullptr);

    // 全局处理: 封装 S5 -> S6
    // 输入：所有单视角点云集合
    // 输出：最终的全局点云与网格 (存入内部，通过getter获取)
    void processGlobal(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& multi_view_clouds,
        const std::vector<double>& view_angles_deg,
        const Eigen::Vector3f& axis_point,
        const Eigen::Vector3f& axis_dir);

    // 获取处理结果
    /**
     * @brief 上一次 S6 底面填充往点云里灌进去的**合成点**数量。
     *
     * 为什么值得单独暴露: 这些点不是测出来的, 是照着拟合平面按 3mm 网格造出来的。
     * 它们一旦进了 m_global_cloud 就会写进最终 PLY —— 排查"点云里有不存在的平面"时,
     * 第一步就该知道这一趟到底造了多少个。0 表示没造。
     */
    int bottomFilledCount() const { return m_bottomFilledCount; }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getFinalPointCloud() const;
    pcl::PolygonMesh getFinalMesh() const;

    // 结果回写 (后台线程完成后，将结果存入本对象，供保存/独立视图复用)
    void storeResult(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PolygonMesh& mesh);

    // ================= 单帧调试专用暴露接口 (放行私有方法) =================
    
    // S1: 光条中心提取
    // 【P0-3】confidence_out: 可选输出，逐点置信度 (0,1]，与 center_points 一一对应；
    //          定义 = clamp(L/255) × (过曝恢复点 ? overexposed_conf_weight : 1.0)
    // mask_pixels_out   : 参与提取的掩膜像素数 (**已扣除**被剔除的伪连通域)
    // mask_dropped_out  : 被"伪光条剔除"丢掉的掩膜像素数 (见函数内那段说明)
    //                     两者相加 = 筛选前的原始掩膜像素数。
    // sat_recovered_out : 其中有多少点是**由饱和恢复**算出来的 (中心取自两侧边缘的
    //                     中点, 不是脊线)。这个数要留档: 它回答"点云里有多少几何
    //                     是靠恢复反推的"。本序列实测约七成 —— 这个比例必须看得见,
    //                     否则一旦恢复逻辑失效, 只会表现为"点云变厚", 看不出原因。
    void extractLaserCenter(const cv::Mat& img, const cv::Rect& roi,
                            std::vector<cv::Point2f>& center_points,
                            int* mask_pixels_out = nullptr,
                            std::vector<float>* confidence_out = nullptr,
                            int* mask_dropped_out = nullptr,
                            int* sat_recovered_out = nullptr);

    // S2: 双目极线约束匹配
    void epipolarConstraintMatch(
        const std::vector<cv::Point2f>& pts_left, 
        const std::vector<cv::Point2f>& pts_right,
        std::vector<cv::Point2f>& matched_pts_left, 
        std::vector<cv::Point2f>& matched_pts_right);

    // S3: 三角化得单线轮廓点云 (相机坐标系)
    // diag: 非空时把每一对匹配的几何量 (含被拒绝的) 记进 diag->matches, 见 MatchDiag。
    pcl::PointCloud<pcl::PointXYZ>::Ptr triangulatePoints(
        const std::vector<cv::Point2f>& pts_left,
        const std::vector<cv::Point2f>& pts_right,
        ViewDiag* diag = nullptr);

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
    int m_bottomFilledCount = 0;   // S6 底面填充造出来的合成点数, 见 bottomFilledCount()

    ViewOutcomeStats m_viewStats;   // 逐视角统计汇总 (见 ViewOutcomeStats 说明)

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

    /**
     * @brief 底面间隙填充：RANSAC 检测转台平面，在稀疏区补合成点，辅助网格闭合底面。
     *
     * @param axisDir **必须传真实的转台轴方向** —— 它是这整件事的"竖直方向"。
     *   早期版本写死成 (0,1,0)（转台轴=Y 的约定），但本工程标定出来的轴在基准系里是
     *   **+Z**，于是"取底面 12%"实际取成了沿 Y 侧切的一片竖切片，RANSAC 对着一片竖切片
     *   拟合出平面，再往那张平面上灌合成点 —— 最终点云里就凭空多出一张穿过物体的薄片。
     *   实测该薄片上的点**精确共面**（到拟合平面 p50 = p90 = 0.000mm），法向 ≈ +Y，
     *   与真实扫描点一眼可分。
     *
     * @param filledOut 非空时回写补出的合成点数量。这些点是**造出来的**，不是测出来的，
     *   调用方需要据此判断要不要保留它们。
     * @param axisPoint 转台轴上的一个点。**必须有**: "底座要比物体宽"这条判据要用
     *   "点到转台轴的垂距", 而只拿到方向是不够的 —— 用"过原点的平行线"算出来的半径
     *   会大得离谱(实测 109mm vs 真实 55mm), 判据就永远不通过。
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fillBottomGaps(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3f& axisDir,
        const Eigen::Vector3f& axisPoint,
        int* filledOut = nullptr);
};

#endif // POINTCLOUD_BUILDER_H