#ifndef POSE_SOLVER_H
#define POSE_SOLVER_H

/**
 * @file pose_solver.h
 * @brief 平面标定板位姿解算与二义性消解 (纯算法层, 不依赖 Qt 界面)
 *
 * @details 平面棋盘格存在两类位姿二义性, 单帧解算无法区分:
 *
 *  **① 精确 180° 面内自对称 (本工程棋盘格的主失效模式)**
 *  11x8 内角点的坐标集在"绕格心转 180°"下严格自映射 (映射 (c,r)→(10-c,7-r)
 *  给出 (100-X, 70-Y), 指标集仍是自身)。因此 findChessboardCorners 的角点顺序
 *  只可能有两种取值, 对应位姿 R_flip = R·Rz(pi), t_flip = t - R_flip·d。
 *  因为 Rz(pi)·e3 = e3, **R 的第三列(板法向)逐位相同**, 且重投影误差在数学上
 *  完全相等 —— 法向/轴/重投影/planeDeg 的 fabs 全都零灵敏度。
 *  唯一有分辨力的量是相邻帧的相对**转角**(翻转 ≈180° vs 正常步长)。
 *  推论: 单帧从原理上不可能消解, 只能靠序列连续性。
 *
 *  **② IPPE 倾斜镜像**
 *  平面目标的经典二解, 两者重投影误差接近但不相等, 法向有可观测差异。
 *
 * 消解策略 (三段式, 消歧不依赖旋转轴, 避免"用轴定标号、轴又由标号决定"的循环):
 *   阶段A enumeratePlanarCandidates  单帧枚举候选 + cheirality/可见面过滤
 *   阶段B disambiguatePlanarSequence 序列连续性 DP 定标号 (核心判据)
 *   阶段C annotateWithAxis           用估出的轴做绝对校验, 只写 flag/note 不改选择
 *
 * 注意: 根治手段是改用非对称标定板 (OpenCV 的 CALIB_CB_MARKER +
 * findChessboardCornersSB 可获得规范角点顺序), 使二义性从源头不存在。本模块
 * 处理的是"已经采到对称板数据"的兜底。
 */

#pragma once

#include <opencv2/core.hpp>
#include <QString>
#include <vector>

namespace Calib {

/**
 * 单帧的一个候选位姿。
 *
 * 重要概念: 二义性有两种**不同性质**, 不能混为一谈。
 *  - 配对内部的歧义 (IPPE 倾斜镜像): 同一份 obj↔img 配对下有多个解, 都拟合数据。
 *  - 配对本身的歧义 (本工程主失效模式): 检测器把角点顺序反过来输出, 于是
 *    obj↔img 的**配对关系**变了。此时"正确位姿"并不拟合所报告的配对, 而是拟合
 *    **把图点置换回规范顺序后**的配对。所以这类候选的相对残差必须对其**自己所属
 *    的配对**计算, 否则代价不可比 (误当"拟合很差的解"而被丢弃)。
 *    见 permuted 字段。
 */
struct PoseCandidate {
    cv::Vec3d rvec;                 // 旋转向量 (棋盘系 → 相机系)
    cv::Vec3d tvec;                 // 平移向量 (mm)
    double    reprojRmsPx = 0.0;    // 相对**本候选所属配对**的重投影 RMS (px)
    bool      cheiralityOk = true;  // 全部角点变换后 z > 0
    bool      frontFaceOk = true;   // 相机在标定板正面一侧
    bool      fromIppe = true;      // false = 手工补的 legacy·Rz(pi)
    bool      fromLegacy = false;   // true = 现有 solvePnP(默认参数) 的原样结果
    bool      permuted = false;     // true = 假设检测顺序被翻转(图点已置换回规范序)
};

/** 序列消歧后单帧的结果 */
struct FramePoseResult {
    bool   solved = false;          // 该帧是否解算成功
    int    chosen = 0;              // 选中候选在候选池中的下标
    int    candidateCount = 0;      // 候选池大小
    bool   ambiguous = false;       // 有辅助解通过了过滤 (存在二义性风险)
    double reprojRmsPx = 0.0;       // 选中解的重投影 RMS (px)
    double reprojRmsAltPx = -1.0;   // 辅助解的重投影 RMS (-1 = 无辅助解)
    double stepChosenDeg = 0.0;     // 选中解相对上一有效帧的转角 (°)
    double stepAltDeg = -1.0;       // 辅助解相对上一有效帧的转角 (°), ≈180 即"发生过翻转"
    int    flipFlag = 0;            // 0=正常 1=已消解(注意) 2=无法消解(异常)
    bool   permuted = false;        // 选中解来自"检测顺序被翻转"的假设
    QString note;                   // 说明 (写入逐帧指标表)
    cv::Vec3d rvec;                 // 选中位姿
    cv::Vec3d tvec;
};

/** 序列消歧的可调项 (分阶段启用, 便于把"结构调整"与"行为改变"分开验证) */
struct PlanarSequenceOptions {
    double minStepDeg = 0.05;        // 相邻帧最小合理转角
    double maxStepDeg = 30.0;        // 相邻帧最大合理转角 (真翻转 ≈180°, 区分度 ~6 倍)
    bool   useIppeCandidates = true; // false = 只用 legacy (行为等价模式)
    bool   allowFlipCandidate = true;// 是否加入 legacy·Rz(pi) 手工候选
    double wReproj = 1.0;            // 节点代价: 重投影 RMS 权重
    double wGauge  = 0.0;            // 节点代价: 偏离 legacy 的相位锚定权重
    double wEdge   = 1.0;            // 边代价: 转角越界罚权重
};

// ---------------------------------------------------------------------------
// 基础工具
// ---------------------------------------------------------------------------

/** 生成棋盘格角点坐标。顺序/类型与 rotatingcalibrator.cpp 原实现逐位一致:
 *  行优先, X = 列号 × squareSize, Y = 行号 × squareSize, Z = 0 */
void makeChessboardObjectPoints(const cv::Size& boardSize, float squareSize,
                               std::vector<cv::Point3f>& out);

/** 重投影 RMS (px), 定义 sqrt(Σd²/N), 与 OpenCV 的 reprojectionError 同口径。
 *  rvec/tvec 用 InputArray 以便同时接受 cv::Mat(solvePnP 输出) 与 cv::Vec3d */
double reprojRms(const std::vector<cv::Point3f>& obj,
                 const std::vector<cv::Point2f>& img,
                 cv::InputArray rvec, cv::InputArray tvec,
                 cv::InputArray K, cv::InputArray D);

/** 两个位姿之间的相对旋转角 (°), 即 |angle(R_a·R_bᵀ)| ∈ [0,180] */
double relativeRotationDeg(const cv::Vec3d& rvecA, const cv::Vec3d& rvecB);

/** 把棋盘格图点置换回"规范顺序"。
 *  检测器顺序被翻转时报告的是 img[i] = p[flipIndex(i)], 置换后 out[i] = p[i],
 *  即恢复出真实的角点顺序。obj 不是规则行列网格时返回 false。 */
bool permuteChessboardImagePoints(const std::vector<cv::Point3f>& obj,
                                  const std::vector<cv::Point2f>& img,
                                  std::vector<cv::Point2f>& out);

/** 棋盘格角点集是否在"绕格心 180° 面内旋转"下自映射 (是则可产生翻转二义性) */
bool isInPlaneSelfSymmetric(const std::vector<cv::Point3f>& obj, double tolMm = 1e-3);

/** 由 legacy 位姿构造 180° 面内翻转候选: R_f = R·Rz(pi), t_f = t - R_f·d */
bool makeFlipCandidate(const std::vector<cv::Point3f>& obj,
                       const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                       cv::Vec3d& rvecOut, cv::Vec3d& tvecOut);

// ---------------------------------------------------------------------------
// 阶段A: 单帧候选枚举
// ---------------------------------------------------------------------------

/**
 * 枚举单帧的位姿候选, 不依赖任何序列上下文。
 * 候选池 = [legacy 原样解] + [IPPE 各解] + [legacy·Rz(pi)] (去重后), 并各自标注
 * cheirality / 可见面 / 重投影 RMS。
 * legacyRvec/legacyTvec 传 nullptr 时内部用 solvePnP 默认参数自行解算。
 * ippeThrew 在 solvePnPGeneric 抛异常时置 true (调用方据此降级, 不应让整体失败)。
 */
bool enumeratePlanarCandidates(const std::vector<cv::Point3f>& obj,
                              const std::vector<cv::Point2f>& img,
                              cv::InputArray K, cv::InputArray D,
                              const cv::Vec3d* legacyRvec, const cv::Vec3d* legacyTvec,
                              std::vector<PoseCandidate>& out,
                              bool* ippeThrew = nullptr);

// ---------------------------------------------------------------------------
// 阶段B: 序列消歧
// ---------------------------------------------------------------------------

/**
 * 对整段序列做消歧。核心判据是序列连续性: 转台步进很小(典型 2°), 而翻转候选
 * 相对上一帧的转角 ≈180°, 因此用 [minStep,maxStep] 的越界罚即可干净区分。
 * wGauge 项把全局相位锚定在 legacy 解上, 保证干净数据输出与改动前逐位相同,
 * 且不会偷偷改变 R_base 的约定(那会连带改变点云与 S5 视角旋转方向)。
 * imgPerFrame 中解算失败的帧在 out 中对应 solved=false。
 */
bool disambiguatePlanarSequence(const std::vector<std::vector<cv::Point2f>>& imgPerFrame,
                               const std::vector<cv::Point3f>& obj,
                               cv::InputArray K, cv::InputArray D,
                               const PlanarSequenceOptions& opt,
                               std::vector<FramePoseResult>& out);

// ---------------------------------------------------------------------------
// 阶段C: 绝对校验 (只标注, 不改选择)
// ---------------------------------------------------------------------------

/**
 * 用已估出的旋转轴校验位姿, 只写 flipFlag/note (沿用 max 语义合并), 不改变 rvec/tvec。
 * 判据由调用方算好后传入: 棋盘法向在转台基准系下应始终与轴同向(带符号, 不用 fabs)。
 * 注意: 这对 ① 类翻转无效(法向逐位相同), 主检测靠 stepAltDeg; 只对 ② 类有效。
 */
void annotateWithAxis(bool normalSignAgrees, double normalDevDeg, double tolDeg,
                      FramePoseResult& io);

} // namespace Calib

#endif // POSE_SOLVER_H
