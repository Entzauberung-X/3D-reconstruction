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
#include "core/pose_solver.h"
#include <QString>
#include <QStringList>
#include <algorithm>
#include <functional>
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

// ==================== 标定诊断 (Tab6「调试与诊断」使用) ====================

/** 单项诊断检查 */
struct AxisDiagCheck {
    QString name;      // 检查项名称
    bool    ok = true; // 是否通过
    QString value;     // 实测值
    QString expect;    // 期望 / 阈值
    QString advice;    // 不通过时的建议动作
};

/** 逐帧指标 (左右相机各一套, 用于定位异常帧) */
struct AxisFrameMetrics {
    int    frameIdx = 0;              // 原始序列帧号 (0 起)
    bool   detected = false;          // 该帧是否成功检测到棋盘格
    double angleDeg = 0.0;            // 相对第 0 帧的转角
    double stepDeg = 0.0;             // 相对上一有效帧的步长
    double reprojPxLeft = 0.0;        // 左相机 PnP 重投影 RMS (px)
    double reprojPxRight = 0.0;       // 右相机 PnP 重投影 RMS (px)
    double centerLeft[3]  = {0,0,0};  // 左光心 (棋盘系, mm)
    double centerRight[3] = {0,0,0};  // 右光心 (棋盘系, mm)
    double radiusLeftMm  = 0.0;       // 左光心到旋转轴的距离 (mm)
    double radiusRightMm = 0.0;
    double residLeftMm   = 0.0;       // 相对本相机圆半径的偏差 (mm)
    double residRightMm  = 0.0;
    double axialCoordMm  = 0.0;       // 左光心沿轴向坐标 (mm)
    double planeNormAxisDeg = 0.0;    // 棋盘格法向与旋转轴夹角 (°, 取绝对值)
    double planeSignedCos = 0.0;      // 法向·轴 (带符号; 与共识符号相反=解翻转)
    // ---- 位姿二义性 (对自对称棋盘格, 每一帧都必然存在一个 180° 翻转伙伴,
    //      所以"存在翻转候选"零信息量; 真正报出来的是"配对相对序列起始变了") ----
    bool   posePermuted = false;      // 该帧检测顺序曾翻转并已纠正回规范配对
    int    poseAmbigFlag = 0;         // 0=正常 1=已消解 2=无法消解
    double stepAltDeg = -1.0;         // 翻转候选相对上一帧的转角 (≈180 表示发生过翻转)
    double reprojAltPx = -1.0;        // 翻转候选的重投影 RMS (px)
    int    flag = 0;                  // 0=正常 1=注意 2=异常
    QString note;                     // 异常说明
};

// ==================== 过程数据 (Tab6 曲线图使用) ====================
//
// 为什么单独留原始观测而不只是留标量指标: 只报"左残差 0.42mm"无法判断光心是
// 均匀散布在圆附近、还是有个别帧整段飞出去。把观测点本身留下来, 才能在 Tab6 里
// 直接看到分布形状 —— 这才是判断"共圆质量"的依据。

/** 3D 圆拟合的单个观测: 某帧某台相机的光心在拟合平面内的投影 */
struct AxisCircleObs {
    int    frameIdx = 0;        // 原始序列帧号
    int    cam = 0;             // 0 = 左相机, 1 = 右相机
    double x = 0.0, y = 0.0;    // 拟合平面内 2D 坐标 (mm)
    double residMm = 0.0;       // 到本相机拟合圆的半径偏差 (mm, 有符号)
};

/** 3D 圆拟合的完整结果 (含原始观测点与拟合圆参数) */
struct AxisCircleFit {
    bool   okL = false, okR = false;
    double cLx = 0, cLy = 0, rLmm = 0, rmsLmm = 0;   // 左相机拟合圆 (平面2D)
    double cRx = 0, cRy = 0, rRmm = 0, rmsRmm = 0;   // 右相机拟合圆
    double cCx = 0, cCy = 0, rCmm = 0, rmsCmm = 0;   // 左右混合拟合 (被半径差支配, 仅参考)
    double centerGapMm = 0;                          // 左右独立拟合圆心偏差 (应在轴上, 趋 0)
    std::vector<AxisCircleObs> obs;                  // 全部观测点 (左右各一帧一个)
};

/** 转台(旋转轴)标定完整诊断结果 */
struct AxisDiagnostics {
    bool    valid = false;          // 诊断是否成功执行
    QString source;                 // 最终采用: "BA" / "圆拟合"
    QString methodSetting;          // 轴估计方式设定: "自动" / "强制圆拟合" / "强制BA"
    double  baErrorLimitPx = -1;    // 自动模式下的 BA 重投影阈值 (px), <0 = 未提供
    QString summary;                // 一句话结论
    int     framesTotal = 0;        // 序列总帧数
    int     framesValid = 0;        // 角点检测成功帧数
    // ---- 角度序列 ----
    double  stepMeanDeg = 0, stepStdDeg = 0, stepMinDeg = 0, stepMaxDeg = 0;
    double  angleSpanDeg = 0;       // 累计转角 (°)
    double  configuredStepDeg = -1; // UI 标称步长 (Tab5「每视角转角」), <0 = 未提供
    // ---- 轴参数 (相机系 / 转台基准系) ----
    double  axisDirCam[3] = {0,0,0},       axisPointCam[3] = {0,0,0};
    double  axisDirTurntable[3] = {0,0,0}, axisPointTurntable[3] = {0,0,0};
    double  baseR[3][3] = {{1,0,0},{0,1,0},{0,0,1}};   // R_base (棋盘0系←相机系)
    double  baseT[3] = {0,0,0};
    double  baErrorPx = 0;          // BA 重投影误差 (px)
    double  circleRmsMm = 0;        // 圆拟合残差 (mm, 混左右光心, 仅参考)
    double  axisDiffDeg = 0;        // 圆拟合轴 与 BA轴 夹角 (°)
    bool    baOk = false, circleOk = false;
    bool    stereoFlipped = false;  // BA 中是否触发了立体外参方向"自动修正"
    // ---- 分相机圆拟合 ----
    double  radiusLeftMm = 0, radiusRightMm = 0;
    double  residLeftMm = 0, residRightMm = 0;   // 各自圆拟合残差 RMS (mm)
    double  axialGapMm = 0;         // 左右光心沿轴向的高差 (mm)
    // ---- 棋盘格法向与轴夹角 ----
    double  planeNormMeanDeg = 0, planeNormStdDeg = 0;

    std::vector<AxisFrameMetrics> frames;
    std::vector<AxisDiagCheck>    checks;

    // ---- 过程数据 (Tab6 曲线图; 见 AxisCircleFit / baIterRmsPx 的说明) ----
    AxisCircleFit circleFit;              // 3D 圆拟合的观测点与拟合圆
    std::vector<double> baIterRmsPx;      // BA 每轮 LM 迭代结束时的重投影 RMS (px)
    std::vector<char>   baIterAccepted;   // 该轮步子是否被接受 (0=被拒, x 不变 → 曲线走平)
    double baRmsInitialPx = -1.0;         // BA 迭代前 (PnP 初值) 的 RMS (px), <0 = 未运行

    int  failCount() const;
    int  checkCount() const { return static_cast<int>(checks.size()); }
};

class RotatingCalibrator {
public:
    RotatingCalibrator();
    ~RotatingCalibrator();

    // ==================== 进度回调 ====================
    /**
     * @brief 进度回调: (阶段名, 总进度百分比 0~100)
     * @note  process() 是**同步**执行的, 回调在调用线程 (即 GUI 线程) 上被调用,
     *        所以 UI 侧更新进度条后需要自己 processEvents() 才能刷出来。
     *        未设置回调时行为与加此接口前完全一致 (零开销)。
     *        阶段划分与权重按修复后的实测耗时占比: 提取角点是大头。
     */
    using ProgressFn = std::function<void(const QString& stage, int percent)>;
    void setProgressFn(ProgressFn fn) { m_progressFn = std::move(fn); }

    // ==================== 轴估计方式 (Tab4「算法参数设置」) ====================
    /**
     * 圆拟合与 BA 是两条独立通道, 各有失效场景, 所以既提供自动判定也允许强制指定。
     *  - Auto:      BA 可用且其重投影误差 ≤ baErrorLimitPx 时用 BA, 否则回退圆拟合
     *  - CircleFit: 强制用 3D 圆拟合
     *  - BA:        强制用 PnP+LM
     * 强制的那一路若失败, 会回退到另一路并在 source 里标注, 不会让整次标定失败。
     */
    enum class AxisMethod { Auto = 0, CircleFit = 1, BA = 2 };

    void setAxisMethod(AxisMethod m) { m_axisMethod = m; }
    AxisMethod getAxisMethod() const { return m_axisMethod; }

    /**
     * @brief 自动模式下 BA 的重投影误差上限 (px), 超过则回退圆拟合。
     * @note  默认 1.0px: 健康数据的 BA 重投影在 0.2px 量级, 留约 5 倍余量;
     *        旧值 20px 过宽 —— 实测遇到过 BA 重投影 5.34px (而单帧 PnP 仅 0.136px,
     *        即 BA 明显没拟合好) 却仍被判为"可用"而采用。
     */
    void setBaErrorLimitPx(double px) { m_baErrorLimitPx = px; }
    double getBaErrorLimitPx() const { return m_baErrorLimitPx; }

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
    // 轴参数统一在**转台基准系(棋盘0系)**下返回 —— 与 S4 输出坐标系、S5 视角增量旋转坐标系一致
    cv::Mat getAxisPoint() const;      // 轴上的点 (3x1)
    cv::Mat getAxisDirection() const;  // 轴方向向量 (3x1, 单位向量)
    // 相机系下的轴 (供在原始相机系里做的验证使用, 如 Tab6 多帧棋盘格 3D)
    cv::Mat getAxisDirectionCam() const;
    cv::Mat getAxisPointCam() const;
    void getBasePose(cv::Mat& R_base, cv::Mat& T_base) const;
    const std::vector<PerFrameDebug>& getPerFrameDebug() const;  // 获取逐帧调试信息

    /**
     * @brief 最终误差的量纲: "px" (BA 重投影) 或 "mm" (圆拟合残差)
     * @note  getReprojectionError() 在两个分支下量纲不同, UI 必须用本接口标注单位,
     *        否则会把毫米级的圆拟合残差当成像素级重投影误差显示 (旧版 Tab4 的显示缺陷)。
     */
    const char* getErrorUnit() const { return m_usedBA ? "px" : "mm"; }
    double getBaErrorPx() const { return m_baErrorPx; }      // BA 重投影误差 (px)
    double getCircleRmsMm() const { return m_circleRmsMm; }  // 圆拟合残差均值 (mm)
    double getCircleResidualLeftMm() const { return m_residLeftMm; }    // 左相机光心共圆残差 RMS
    double getCircleResidualRightMm() const { return m_residRightMm; }  // 右相机光心共圆残差 RMS
    double getAxialGapMm() const { return m_axialGapMm; }    // 左右光心轴向高差
    bool   isBaUsed() const { return m_usedBA; }

    // ==================== 诊断接口 (Tab6) ====================
    /**
     * @brief 执行完整转台标定诊断: 重新提取特征 → 圆拟合 + BA → 逐帧指标 + 一致性检查
     * @param configuredStepDeg Tab5「每视角转角」的标称步长 (用于比较实测步长), <0 表示未知
     * @note  会重新执行一次完整标定 (含特征提取), 耗时与轴标定相当。
     */
    const AxisDiagnostics& runDiagnostics(double configuredStepDeg = -1.0);
    const AxisDiagnostics& getDiagnostics() const { return m_diag; }

private:
    // ==================== 内部私有算法函数 ====================
    bool extractStereoFeatures();
    bool estimateInitialPose();
    bool estimateAxisByCircleFitting();  // 3D圆拟合：比BA更稳健的轴估计
    bool runBundleAdjustment();
    void generateObjectPoints(std::vector<cv::Point3f>& objectPoints) const;
    void populateDebugData();  // 填充逐帧调试信息

    /** 进度上报 (回调为空时是空操作) */
    void reportProgress(const QString& stage, int percent) const {
        if (m_progressFn) m_progressFn(stage, percent);
    }
    /** 把 [from,to] 这段百分比按 cur/total 线性映射 */
    static int phasePercent(int from, int to, int cur, int total) {
        if (total <= 0) return from;
        const double r = static_cast<double>(cur) / total;
        return from + static_cast<int>((to - from) * std::min(1.0, std::max(0.0, r)));
    }

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

    // ---- 诊断用: 两种轴估计各自的原始结果 (process() 中记录) ----
    bool   m_usedBA = false;          // 最终采用 BA (true) 还是圆拟合 (false)
    bool   m_baOk = false, m_circleOk = false;
    double m_baErrorPx = 0.0;         // BA 重投影误差 (px)
    double m_circleRmsMm = 0.0;       // 圆拟合残差 (mm)
    cv::Mat m_baAxisDir, m_baAxisPt;  // BA 轴
    cv::Mat m_circleAxisDir, m_circleAxisPt;
    bool   m_stereoFlipped = false;   // BA 中是否触发立体外参方向修正
    AxisDiagnostics m_diag;           // 最近一次诊断结果

    // ---- 圆拟合质量指标 (分相机, 单位 mm) ----
    cv::Mat m_pnpAxisDir;             // PnP 旋转方向推出的轴 (仅用于统一符号/诊断)
    double  m_circleCenterGapMm = 0.0;   // 左右相机独立拟合的圆心偏差
    double  m_radiusLeftMm = 0.0, m_radiusRightMm = 0.0;
    double  m_residLeftMm = 0.0, m_residRightMm = 0.0;
    double  m_mixedRmsMm = 0.0;       // 混拟合残差 (仅参考, 被半径差支配)
    double  m_axialGapMm = 0.0;       // 左右光心沿轴向的高差 (mm)

    // ---- 过程数据 (Tab6 曲线图): 由圆拟合/BA 在运行中记录, buildDiagnostics 拷进 m_diag ----
    // 注意这些必须在各自的**失败分支之前**就填好 —— 恰恰是出问题的那次最需要看曲线,
    // 若等到判定成功才记录, 失败时曲线就是空的。
    AxisCircleFit       m_circleFit2d;      // 圆拟合原始观测 + 拟合圆
    std::vector<double> m_baIterRmsPx;      // BA 逐轮 RMS (px)
    std::vector<char>   m_baIterAccepted;   // 逐轮是否接受
    double              m_baRmsInitialPx = -1.0;

    // ==================== 逐帧位姿缓存 (全类唯一的位姿解算来源) ====================
    /**
     * 单帧解算结果。
     *
     * 为什么要有这层缓存: 改动前 estimateInitialPose / estimateAxisByCircleFitting /
     * populateDebugData / buildDiagnostics 各自独立跑一遍 solvePnP (共 4 条通道),
     * 主路径与诊断路径的一致性靠"两次独立解算碰巧相同"来维持, 一旦某帧的 PnP 落入
     * 不同局部极小, "当前生效轴 vs 诊断轴"检查就会无故失败。
     * 现在 4 条通道全部改为消费 posesLeft()/posesRight(), 一致性由构造保证,
     * 也顺带使逐帧二义性消解只需做一次。
     */
    struct FramePose {
        bool      solved = false;
        cv::Vec3d rvec, tvec;
        bool      permuted = false;    // 检测顺序曾被翻转并已纠正回规范配对
        double    reprojRmsPx = 0.0;   // 相对本帧所属配对的重投影 RMS (px)
        double    reprojAltPx = -1.0;  // 翻转候选的重投影 RMS (px, -1 = 无)
        double    stepAltDeg = -1.0;   // 翻转候选相对上一帧的转角, ≈180 表示发生过翻转
        int       flipFlag = 0;        // 0 正常 / 1 已消解(注意) / 2 无法消解(异常)
        QString   note;
    };

    /** 逐帧位姿 (惰性构建 + 缓存)。绝不读 m_axisDirection, 故与轴估计无循环依赖。 */
    const std::vector<FramePose>& posesLeft();
    const std::vector<FramePose>& posesRight();
    void buildPoseCache();

    std::vector<FramePose> m_poseL, m_poseR;
    bool                   m_poseCacheDirty = true;
    PlanarSequenceOptions  m_poseOpt;      // 消歧选项 (按计划分阶段启用)

    // ---- 诊断内部辅助 ----
    void buildDiagnostics(double configuredStepDeg);

    // ---- 进度回调 (空 = 不上报) ----
    ProgressFn m_progressFn;

    // ---- 轴估计方式与阈值 ----
    AxisMethod m_axisMethod = AxisMethod::Auto;
    double     m_baErrorLimitPx = 1.0;
};

} // namespace Calib

#endif