/**
 * @file lasercalibration.cpp
 * @brief 激光平面标定算法实现
 * @details 包含激光线提取、极线匹配、射线-平面求交、RANSAC与SVD平面拟合等核心算法。
 *          主要用于双目结构光三维扫描系统中的光平面标定。
 */

#include "core/lasercalibration.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <QDebug>

// ================= 构造与参数设置 =================

/**
 * @brief 构造函数，初始化默认参数
 * @details 默认标定板尺寸为 11x8，格子尺寸为 10.0mm。
 *          相机内参初始化为单位阵和零畸变（占位，实际需调用 setCameraParams 传入真实标定结果）。
 */
LaserCalibration::LaserCalibration() : m_boardSize(11, 8), m_squareSize(10.0f) {
    m_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    m_distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
}

void LaserCalibration::setBoardSize(int w, int h) { m_boardSize = cv::Size(w, h); }
void LaserCalibration::setSquareSize(float size) { m_squareSize = size; }

/**
 * @brief 设置相机的内参矩阵和畸变系数
 */
void LaserCalibration::setCameraParams(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    m_cameraMatrix = cameraMatrix.clone();
    m_distCoeffs = distCoeffs.clone();
}


// ====================================================================
// 1. 直线分析辅助函数
// ====================================================================

/**
 * @brief 分析激光线点集的直线度和方程
 * @param points 输入的2D点集
 * @return LineAnalysisResult 包含点数、直线方程字符串和均方根误差(RMSE)
 */
LineAnalysisResult LaserCalibration::analyzeLaserLine(const std::vector<cv::Point2f>& points) {
    LineAnalysisResult res;
    res.pointCount = points.size();
    if (points.size() < 2) {
        res.equationStr = "N/A";
        return res;
    }

    // 使用最小二乘法拟合直线，返回 (vx, vy, x0, y0)
    cv::Vec4f lineParams;
    cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);
    
    // 将方向向量和点坐标转换为斜截式 y = kx + b
    float k = lineParams[1] / (lineParams[0] + 1e-6f); // 防止除以0
    float b = lineParams[3] - k * lineParams[2];
    res.equationStr = QString("y = %1x + %2").arg(k, 0, 'f', 2).arg(b, 0, 'f', 2).toStdString();

    // 计算拟合的均方根误差
    double error_sum = 0;
    for (const auto& pt : points) {
        double y_pred = k * pt.x + b;
        error_sum += (pt.y - y_pred) * (pt.y - y_pred);
    }
    res.rmse = static_cast<float>(std::sqrt(error_sum / points.size()));
    return res;
}

// ====================================================================
// 2. 极线匹配辅助函数 (用于处理激光线断开的情况)
// ====================================================================

/**
 * @brief 将离散点集按X坐标排序并分割成多条连续的线段
 * @param pts 输入点集
 * @param breakDist 判定为断开的最大距离阈值
 * @param minPts 构成有效线段的最少点数
 * @return 线段集合
 */
static std::vector<LineSegment> fitLineSegments(const std::vector<cv::Point2f>& pts, float breakDist, int minPts) {
    std::vector<LineSegment> segments;
    if (pts.empty()) return segments;

    // 1. 按X坐标从小到大排序
    std::vector<cv::Point2f> sortedPts = pts;
    std::sort(sortedPts.begin(), sortedPts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });

    std::vector<cv::Point2f> currentSeg = {sortedPts[0]};
    for (size_t i = 1; i < sortedPts.size(); ++i) {
        float dist = cv::norm(sortedPts[i] - sortedPts[i-1]);
        // 2. 如果两点间距超过阈值，认为线段断开
        if (dist > breakDist) {
            if ((int)currentSeg.size() >= minPts) {
                LineSegment seg;
                seg.pts = currentSeg;
                cv::fitLine(currentSeg, seg.lineParams, cv::DIST_L2, 0, 0.01, 0.01);
                // 【核心修复】lineParams 顺序是，前两个是方向，后两个是点坐标
                float len = cv::norm(currentSeg.front() - currentSeg.back());
                float dx = seg.lineParams[0] * (len / 2.0f);
                float dy = seg.lineParams[1] * (len / 2.0f);
                seg.p1 = cv::Point2f(seg.lineParams[2] - dx, seg.lineParams[3] - dy);
                seg.p2 = cv::Point2f(seg.lineParams[2] + dx, seg.lineParams[3] + dy);
                segments.push_back(seg);
            }
            currentSeg.clear();
        }
        currentSeg.push_back(sortedPts[i]);
    }

    // 处理最后一段
    if ((int)currentSeg.size() >= minPts) {
        LineSegment seg;
        seg.pts = currentSeg;
        cv::fitLine(currentSeg, seg.lineParams, cv::DIST_L2, 0, 0.01, 0.01);
        // 【核心修复】lineParams 顺序是，前两个是方向，后两个是点坐标
        float len = cv::norm(currentSeg.front() - currentSeg.back());
        float dx = seg.lineParams[0] * (len / 2.0f);
        float dy = seg.lineParams[1] * (len / 2.0f);
        seg.p1 = cv::Point2f(seg.lineParams[2] - dx, seg.lineParams[3] - dy);
        seg.p2 = cv::Point2f(seg.lineParams[2] + dx, seg.lineParams[3] + dy);
        segments.push_back(seg);
    }
    return segments;
}

/**
 * @brief 计算2D空间中两条直线的交点
 * @param p1 直线1上的点
 * @param v1 直线1的方向向量
 * @param p2 直线2上的点
 * @param v2 直线2的方向向量
 * @return 交点坐标，如果平行则返回 (-1, -1)
 */
static cv::Point2f lineLineIntersection(cv::Point2f p1, cv::Vec2f v1, cv::Point2f p2, cv::Vec2f v2) {
    float cross = v1[0] * v2[1] - v1[1] * v2[0];
    if (std::abs(cross) < 1e-6f) return cv::Point2f(-1, -1); // 叉积为0，平行
    cv::Point2f d = p2 - p1;
    float t = (d.x * v2[1] - d.y * v2[0]) / cross;
    return cv::Point2f(p1.x + t * v1[0], p1.y + t * v1[1]);
}


// ====================================================================
// 3. 核心标定算法实现 (方法A：左射线与右平面求交)
// ====================================================================

/**
 * @brief 将 cv::fitLine 的 (vx, vy, x0, y0) 转换为齐次直线方程 ax + by + c = 0
 */
static cv::Vec3d getLineEquation(const cv::Vec4f& lineParams) {
    double vx = lineParams[0], vy = lineParams[1];
    double x0 = lineParams[2], y0 = lineParams[3];
    // 法向量 = (-vy, vx)，常数项 c = vy*x0 - vx*y0
    double a = -vy;
    double b = vx;
    double c = vy * x0 - vx * y0;
    return cv::Vec3d(a, b, c);
}

LineAnalysisResult LaserCalibration::ransacLineFitting(const std::vector<cv::Point2f>& points, 
                                                   float distanceThreshold, 
                                                   int maxIterations)
{
    LineAnalysisResult result;
    result.pointCount = 0;
    result.rmse = 0.0f;
    
    if (points.size() < 2) {
        return result;
    }

    // RANSAC参数
    const int iterations = maxIterations;
    std::vector<cv::Point2f> bestInliers;
    cv::Vec4f bestLine;
    int bestInlierCount = 0;
    
    // 随机数生成器
    cv::RNG rng(cv::getTickCount());
    
    for (int i = 0; i < iterations; ++i) {
        // 随机选择两个点来定义一条直线
        std::vector<cv::Point2f> randomPoints;
        std::vector<int> indices;
        
        while (randomPoints.size() < 2) {
            int idx = rng.uniform(0, static_cast<int>(points.size()));
            if (std::find(indices.begin(), indices.end(), idx) == indices.end()) {
                randomPoints.push_back(points[idx]);
                indices.push_back(idx);
            }
        }
        
        // 计算直线参数 (ax + by + c = 0)
        double x1 = randomPoints[0].x, y1 = randomPoints[0].y;
        double x2 = randomPoints[1].x, y2 = randomPoints[1].y;
        
        double a = y2 - y1;
        double b = x1 - x2;
        double c = x2 * y1 - x1 * y2;
        
        // 归一化直线参数
        double norm = sqrt(a * a + b * b);
        if (norm > 0) {
            a /= norm;
            b /= norm;
            c /= norm;
        }
        
        // 计算内点数量
        std::vector<cv::Point2f> currentInliers;
        int inlierCount = 0;
        
        for (const auto& point : points) {
            double distance = fabs(a * point.x + b * point.y + c);
            if (distance < distanceThreshold) {
                currentInliers.push_back(point);
                inlierCount++;
            }
        }
        
        // 更新最佳直线
        if (inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestInliers = currentInliers;
            bestLine = cv::Vec4f(a, b, c, 0); // 存储直线参数
        }
        
        // 如果内点数量足够，提前终止
        if (inlierCount >= points.size() * 0.8) { // 80%的点都是内点时提前终止
            break;
        }
    }
    
    // 如果找到了足够的内点，进行直线拟合
    if (bestInlierCount >= 2) {
        // 使用所有内点进行最终的直线拟合
        if (!bestInliers.empty()) {
            cv::Vec4f finalLine;
            cv::fitLine(bestInliers, finalLine, cv::DIST_L2, 0, 0.01, 0.01);
            
            // 【核心修复】将 转换为 ax + by + c = 0 形式
            double a = -finalLine[1];
            double b = finalLine[0];
            double c = finalLine[1] * finalLine[2] - finalLine[0] * finalLine[3];
            double norm_l = std::sqrt(a * a + b * b);
            if (norm_l > 1e-6) {
                a /= norm_l; b /= norm_l; c /= norm_l;
            }

            // 计算正确的 RMSE
            double rmse = 0.0;
            for (const auto& pt : bestInliers) {
                double distance = std::abs(a * pt.x + b * pt.y + c);
                rmse += distance * distance;
            }
            rmse = std::sqrt(rmse / bestInliers.size());
            
            // 生成正确的直线方程字符串
            std::string equationStr = "y = ";
            if (std::abs(b) > 1e-6) {
                double slope = -a / b;
                double intercept = -c / b;
                equationStr += std::to_string(slope) + "x + " + std::to_string(intercept);
            } else {
                equationStr = "x = " + std::to_string(-c / a);
            }
            
            result.pts = bestInliers;
            result.pointCount = bestInliers.size();
            result.rmse = static_cast<float>(rmse);
            result.equationStr = equationStr;
        }
    }
    
    return result;
}

/**
 * @brief 带进度回调的激光平面标定（核心算法1）
 * @details 算法原理：
 * 1. 提取左相机投影矩阵P1的3x3部分M并求逆，计算左相机光心C1。
 * 2. 右图激光线反投影为空间平面 (pi = l_R^T * P2)。
 * 3. 左图像素点反投影为空间射线 (基于M_inv，起点C1，方向D)。
 * 4. 计算射线与平面的交点，得到空间三维点。
 * 5. 使用 RANSAC 剔除飞点，最后用 SVD 拟合纯净点云得到光平面方程。
 */
LaserPlaneCalibrationResult LaserCalibration::calibrateLaserPlaneWithProgress(
    const std::vector<LaserPair>& laserData,
    const cv::Mat& P1, const cv::Mat& P2,
    const CalibrationParams& params,
    std::function<void(int, int)> progressCallback)
{
    LaserPlaneCalibrationResult result;
    result.success = false;
    if (laserData.empty() || P1.empty() || P2.empty()) return result;

    // =========================================================================
    // 1. 预计算核心矩阵与光心
    // =========================================================================
    // P1 = [M | p4]，M是左相机内参*旋转，p4是平移
    cv::Mat M = P1.colRange(0, 3);
    cv::Mat p4 = P1.col(3);
    cv::Mat M_inv;
    cv::invert(M, M_inv, cv::DECOMP_LU); // 3x3方阵求逆，非常稳定

    // 利用 SVD 求解齐次线性方程组 P1 * C1 = 0，得到左相机光心 C1
    cv::Mat C1_h;
    cv::SVD::solveZ(P1, C1_h);
    cv::Point3d C1(C1_h.at<double>(0)/C1_h.at<double>(3), 
                   C1_h.at<double>(1)/C1_h.at<double>(3), 
                   C1_h.at<double>(2)/C1_h.at<double>(3));

    std::vector<cv::Point3f> rawWorldPoints;
    int cnt_killed_by_depth = 0;
    int cnt_killed_by_parallel = 0;
    int cnt_killed_by_verify = 0;
    double min_z_found = 1e9, max_z_found = -1e9;

    // =========================================================================
    // 2. 逐张处理：基于 3x3 逆矩阵的绝对稳定射线求交
    // =========================================================================
    for (int i = 0; i < laserData.size(); ++i) {
        if (progressCallback) progressCallback(i + 1, laserData.size());
        
        const auto& ptsL = laserData[i].ptsL;
        const auto& ptsR = laserData[i].ptsR;
        if (ptsL.empty() || ptsR.empty()) continue;

        // 分割线段，处理激光线遮挡/断开情况
        auto segsL = fitLineSegments(ptsL, params.segmentBreakDist, params.minPointsPerSegment);
        auto segsR = fitLineSegments(ptsR, params.segmentBreakDist, params.minPointsPerSegment);
        if (segsL.empty() || segsR.empty()) continue;

        // 如果左右都只有一段线，可以跳过后续的耗时重投影验证，提高速度
        bool isSingleSegPair = (segsL.size() == 1 && segsR.size() == 1);

        for (const auto& segL : segsL) {
            for (const auto& segR : segsR) {
                // 右图直线反投影的空间平面方程 pi = l_R^T * P2
                cv::Vec3d l_R = getLineEquation(segR.lineParams);
                cv::Mat l_R_mat = (cv::Mat_<double>(1, 3) << l_R[0], l_R[1], l_R[2]);
                cv::Mat pi = l_R_mat * P2; // 结果为 1x4: [pa, pb, pc, pd]
                double pa = pi.at<double>(0), pb = pi.at<double>(1);
                double pc = pi.at<double>(2), pd = pi.at<double>(3);
                cv::Point3d pi_vec(pa, pb, pc);

                for (const auto& ptL : segL.pts) {
                    // 【关键数学推导】左图像素点反投影为射线方向 D
                    // 射线起点为光心 C1，方向 D = M_inv * (x_L - p4) - C1 
                    cv::Mat xL_h = (cv::Mat_<double>(3, 1) << ptL.x, ptL.y, 1.0);
                    cv::Mat diff = xL_h - p4;
                    cv::Mat P0_mat = M_inv * diff;
                    cv::Point3d P0(P0_mat.at<double>(0), P0_mat.at<double>(1), P0_mat.at<double>(2));
                    cv::Point3d D = P0 - C1;

                    // 射线与平面求交: P(t) = C1 + t*D 代入平面方程 pa*x + pb*y + pc*z + pd = 0
                    double dot_C = pi_vec.dot(C1) + pd;
                    double dot_D = pi_vec.dot(D);
                    
                    if (std::abs(dot_D) < 1e-6) { // 射线与平面平行
                        cnt_killed_by_parallel++;
                        continue;
                    }
                    double t = -dot_C / dot_D;
                    if (t < 0) continue; // 交点在相机背后，剔除

                    cv::Point3d pt3D(C1.x + t*D.x, C1.y + t*D.y, C1.z + t*D.z);

                    // 统计Z范围
                    if (pt3D.z < min_z_found) min_z_found = pt3D.z;
                    if (pt3D.z > max_z_found) max_z_found = pt3D.z;

                    // 深度过滤
                    if (pt3D.z < params.depthMin || pt3D.z > params.depthMax) {
                        cnt_killed_by_depth++;
                        continue;
                    }

                    // 多段线防错配验证：将求出的3D点重投影回右图，检查是否真的落在当前右图线段上
                    if (!isSingleSegPair) {
                        cv::Mat pt3D_h = (cv::Mat_<double>(4, 1) << pt3D.x, pt3D.y, pt3D.z, 1.0);
                        cv::Mat pt2D_R_h = P2 * pt3D_h;
                        cv::Point2f proj_R(pt2D_R_h.at<double>(0)/pt2D_R_h.at<double>(2), 
                                          pt2D_R_h.at<double>(1)/pt2D_R_h.at<double>(2));
                        float dist = pointToLineDistance(proj_R, segR.p1, segR.p2);
                        if (dist > 15.0f) { // 距离右图线段太远，说明错配
                            cnt_killed_by_verify++;
                            continue;
                        }
                    }
                    rawWorldPoints.push_back(cv::Point3f(static_cast<float>(pt3D.x), 
                                                         static_cast<float>(pt3D.y), 
                                                         static_cast<float>(pt3D.z)));
                }
            }
        }
    }

    // =========================================================================
    // 3. 诊断日志
    // =========================================================================
    qDebug() << "====== [光平面诊断] 求交完成 ======";
    qDebug() << "有效点云总数:" << rawWorldPoints.size();
    qDebug() << "被深度过滤杀掉:" << cnt_killed_by_depth << "(阈值:" << params.depthMin << "~" << params.depthMax << ")";
    if (rawWorldPoints.size() > 0) {
        qDebug() << "实际有效点云 Z 坐标范围: [" << min_z_found << " , " << max_z_found << "]";
    }
    qDebug() << "===================================";

    if (rawWorldPoints.size() < params.minPointsForPlane) {
        result.resultMessage = QString("失败：有效三维点不足(仅%1个)。请查看终端日志排查原因！").arg(rawWorldPoints.size());
        return result;
    }

    // =========================================================================
    // 4. RANSAC 剔除飞点
    // =========================================================================
    std::vector<cv::Point3f> inliers;
    int bestInlierCount = 0;
    cv::Vec4f bestPlane;
    srand(42); // 固定随机种子以保证结果可复现

    for (int iter = 0; iter < params.ransacMaxIterations; ++iter) {
        // 随机采样3个点
        int i1 = rand() % rawWorldPoints.size();
        int i2 = rand() % rawWorldPoints.size();
        int i3 = rand() % rawWorldPoints.size();
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        cv::Point3f p1 = rawWorldPoints[i1], p2 = rawWorldPoints[i2], p3 = rawWorldPoints[i3];
        // 叉积求平面法向量
        cv::Point3f normal = (p2 - p1).cross(p3 - p1);
        float normLen = cv::norm(normal);
        if (normLen < 1e-6) continue; // 三点共线，跳过
        normal /= normLen;
        float d = -normal.dot(p1);

        // 统计满足距离阈值的内点数量
        int currentInliers = 0;
        for (const auto& pt : rawWorldPoints) {
            if (std::abs(normal.dot(pt) + d) < params.ransacDistThreshold)
                currentInliers++;
        }

        if (currentInliers > bestInlierCount) {
            bestInlierCount = currentInliers;
            bestPlane = cv::Vec4f(normal.x, normal.y, normal.z, d);
        }
    }

    // 筛选出所有内点
    for (const auto& pt : rawWorldPoints) {
        if (std::abs(bestPlane[0]*pt.x + bestPlane[1]*pt.y + bestPlane[2]*pt.z + bestPlane[3]) < params.ransacDistThreshold) {
            inliers.push_back(pt);
        }
    }

    if (inliers.size() < 3) {
        qDebug() << "====== [光平面诊断] RANSAC 失败 ======";
        qDebug() << "输入给RANSAC的点数:" << rawWorldPoints.size() << " | 最大内点数:" << bestInlierCount;
        result.resultMessage = QString("失败：RANSAC剔除后有效点不足。请查看终端日志！");
        return result;
    }

    // =========================================================================
    // 5. 纯净内点 SVD 高精度拟合
    // =========================================================================
    // 构造 Nx3 矩阵
    cv::Mat A(inliers.size(), 3, CV_64F);
    for (size_t i = 0; i < inliers.size(); ++i) {
        A.at<double>(i, 0) = inliers[i].x;
        A.at<double>(i, 1) = inliers[i].y;
        A.at<double>(i, 2) = inliers[i].z;
    }
    
    // 去质心化（消除平移分量，防止平面不过原点时SVD产生奇点）
    cv::Mat mean_pt;
    cv::reduce(A, mean_pt, 0, cv::REDUCE_AVG);
    cv::Mat A_centered = A - cv::repeat(mean_pt, A.rows, 1);

    // SVD分解：最小奇异值对应的右奇异向量即为平面法向量
    cv::Mat w, u, vt;
    cv::SVD::compute(A_centered, w, u, vt);
    double a = vt.at<double>(2, 0), b = vt.at<double>(2, 1), c = vt.at<double>(2, 2);
    // 计算 D 系数：D = -(n · 质心)
    double d = -(a * mean_pt.at<double>(0) + b * mean_pt.at<double>(1) + c * mean_pt.at<double>(2));
    
    // 归一化法向量
    double norm = std::sqrt(a*a + b*b + c*c);
    a/=norm; b/=norm; c/=norm; d/=norm;

    // 组装结果
    result.planeCoeffs = (cv::Mat_<float>(4, 1) << a, b, c, d);
    result.worldPoints = inliers;
    
    // 计算拟合RMSE
    double error_sum = 0;
    for(const auto& pt : inliers) error_sum += std::pow(a*pt.x + b*pt.y + c*pt.z + d, 2);
    double rmse = std::sqrt(error_sum / inliers.size());

    result.success = true;
    result.totalPoints = inliers.size();
    result.failedMatches = rawWorldPoints.size() - inliers.size();
    
    // =========================================================================
    // 【核心修改处】将方程系数格式化输出到字符串
    // =========================================================================
    result.resultMessage = QString("标定成功！(直线求交法)\n"
                                  "纯净点数: %1\n"
                                  "RANSAC剔除飞点: %2\n"
                                  "拟合误差(RMSE): %3 mm\n"
                                  "------------------------\n"
                                  "光平面方程:\n"
                                  "%4x + %5y + %6z + %7 = 0")
                                  .arg(result.totalPoints)
                                  .arg(result.failedMatches)
                                  .arg(rmse, 0, 'f', 4)
                                  .arg(a, 0, 'f', 6)  // 保留6位小数保证方程精度
                                  .arg(b, 0, 'f', 6)
                                  .arg(c, 0, 'f', 6)
                                  .arg(d, 0, 'f', 6);
                                  
    return result;
}

/**
 * @brief 计算点到由两点定义的直线的距离 (全局静态辅助函数)
 */
static float pointToLineDistance(const cv::Point2f& pt, const cv::Point2f& linePt1, const cv::Point2f& linePt2) {
    float dx = linePt2.x - linePt1.x;
    float dy = linePt2.y - linePt1.y;
    float norm = std::sqrt(dx * dx + dy * dy);
    if (norm < 1e-6f) return std::sqrt(std::pow(pt.x - linePt1.x, 2) + std::pow(pt.y - linePt1.y, 2));
    // 利用叉积公式求距离：|dy*x - dx*y + x2*y1 - y2*x1| / sqrt(dx^2+dy^2)
    return std::abs(dy * pt.x - dx * pt.y + linePt2.x * linePt1.y - linePt2.y * linePt1.x) / norm;
}


// =====================================================================
// 函数 1：激光中心提取（基于LAB色彩空间，支持降采样加速）
// =====================================================================

/**
 * @brief 核心激光中心线提取算法
 * @param roiLaser 感兴趣区域的激光图像
 * @param roiMask 掩膜（保留接口兼容，内部未使用）
 * @param params LAB阈值参数
 * @param downScale 降采样比例。标定时传1.0保证精度，实时扫描时可传0.25~0.5提高帧率
 * @return 按扫描线顺序排列的亚像素激光中心点集
 */
std::vector<cv::Point2f> LaserCalibration::detectLaserCenter_LAB_Steger(
    const cv::Mat& roiLaser, const cv::Mat& roiMask, 
    const LABParams& labParams, const StegerParams& stegerParams, float downScale) 
{
    std::vector<cv::Point2f> finalPoints;
    if (roiLaser.empty()) return finalPoints;

    downScale = std::clamp(downScale, 0.25f, 1.0f); // 边界保护
    cv::Mat bgr_small;
    if (downScale < 0.99f) {
        cv::resize(roiLaser, bgr_small, cv::Size(), downScale, downScale, cv::INTER_LINEAR);
    } else {
        bgr_small = roiLaser; 
    }

    // ==================================================================
    // 1. 【完全保留原逻辑】 LAB 色彩空间阈值分割，生成高质量掩膜
    // ==================================================================
    cv::Mat labImg;
    cv::cvtColor(bgr_small, labImg, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> labChannels;
    cv::split(labImg, labChannels);
    cv::Mat L = labChannels[0], A = labChannels[1], B_ch = labChannels[2];

    // 自适应阈值提取高亮区域
    cv::Mat maskL, maskA;
    int block = labParams.blockSize > 1 ? labParams.blockSize : 3;
    if (block % 2 == 0) block++;
    int small_block = std::max(3, static_cast<int>(block * downScale));
    if (small_block % 2 == 0) small_block++;
    
    cv::adaptiveThreshold(L, maskL, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, small_block, labParams.C);
    cv::adaptiveThreshold(A, maskA, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, small_block, labParams.C);
    
    // 排除黄色（B通道低通常偏黄）
    cv::Mat maskB_not_yellow;
    cv::inRange(B_ch, 0, labParams.maxB, maskB_not_yellow);

    // 综合LAB掩膜
    cv::Mat maskLAB;
    cv::bitwise_and(maskL, maskA, maskLAB);
    cv::bitwise_and(maskLAB, maskB_not_yellow, maskLAB);

    // 补充：提取纯白色区域（应对反光过曝变成纯白的情况）
    std::vector<cv::Mat> bgrChannels;
    cv::split(bgr_small, bgrChannels);
    cv::Mat minRB, minRGB;
    cv::min(bgrChannels[0], bgrChannels[1], minRB);
    cv::min(minRB, bgrChannels[2], minRGB);
    cv::Mat maskWhite = (minRGB > labParams.whiteThresh) & (bgrChannels[2] >= bgrChannels[1]) & (bgrChannels[2] >= bgrChannels[0]);
    
    // 合并掩膜并形态学闭运算去孔洞
    cv::Mat maskFinal;
    cv::bitwise_or(maskLAB, maskWhite, maskFinal);
    int kernelSize = std::max(3, static_cast<int>(3 * downScale));
    if (kernelSize % 2 == 0) kernelSize++;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(maskFinal, maskFinal, cv::MORPH_CLOSE, kernel);

    // 为 Steger 稍微膨胀掩膜，防止宽带边缘截断
    cv::Mat maskDilated;
    cv::dilate(maskFinal, maskDilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    // ==================================================================
    // 2. Steger 算法：基于局部法线的亚像素骨化
    // ==================================================================
    cv::Mat gray;
    cv::cvtColor(bgr_small, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray, CV_32F);
    
    // 高斯平滑
    int ksize = stegerParams.gaussKernel > 1 && (stegerParams.gaussKernel % 2 == 1) ? stegerParams.gaussKernel : 5;
    cv::GaussianBlur(gray, gray, cv::Size(ksize, ksize), stegerParams.gaussSigma);

    // 计算一阶和二阶偏导数
    cv::Mat dx, dy, dxx, dxy, dyy;
    cv::Sobel(gray, dx, CV_32F, 1, 0, 3);
    cv::Sobel(gray, dy, CV_32F, 0, 1, 3);
    cv::Sobel(dx, dxx, CV_32F, 1, 0, 3);
    cv::Sobel(dy, dyy, CV_32F, 0, 1, 3);
    cv::Sobel(dx, dxy, CV_32F, 0, 1, 3);

    // 获取掩膜内的像素坐标以加速遍历
    cv::Mat maskLocations;
    cv::findNonZero(maskDilated, maskLocations);

    // 【核心优化】使用哈希表进行 NMS 去重，彻底解决虚拟机 std::bad_alloc 内存爆炸问题
    struct Candidate { cv::Point2f pt; float resp; };
    std::unordered_map<size_t, Candidate> nmsMap;
    nmsMap.reserve(5000); // 预分配空间减少哈希冲突

    for (int i = 0; i < maskLocations.rows; ++i) {
        int x = maskLocations.at<cv::Point>(i).x;
        int y = maskLocations.at<cv::Point>(i).y;

        float ixx = dxx.at<float>(y, x);
        float ixy = dxy.at<float>(y, x);
        float iyy = dyy.at<float>(y, x);
        float ix  = dx.at<float>(y, x);
        float iy  = dy.at<float>(y, x);

        // 计算 Hessian 矩阵的最大特征值 (对于亮线，最大特征值应为负数)
        float trace = ixx + iyy;
        float det = ixx * iyy - ixy * ixy;
        float discriminant = trace * trace / 4.0f - det;
        if (discriminant < 0) discriminant = 0;
        
        float lambda1 = trace / 2.0f - std::sqrt(discriminant);

        // 判断阈值 (注意：亮线的 lambda1 是负数，阈值也应设为负数)
        if (lambda1 >= stegerParams.threshold) continue;

        // 计算法向量 (对应最大特征值 lambda1 的特征向量)
        // 择优两公式策略: (H - λI)v = 0 有两种等价形式，取范数较大者避免数值退化
        float nx_a = ixy,           ny_a = lambda1 - ixx;
        float nx_b = lambda1 - iyy, ny_b = ixy;
        float norm_a = nx_a * nx_a + ny_a * ny_a;
        float norm_b = nx_b * nx_b + ny_b * ny_b;
        float nx = (norm_a >= norm_b) ? nx_a : nx_b;
        float ny = (norm_a >= norm_b) ? ny_a : ny_b;
        float norm = std::sqrt(nx * nx + ny * ny);
        if (norm < 1e-6f) continue;
        nx /= norm;
        ny /= norm;

        // 泰勒展开求极值点偏移量 t
        float denom = ixx * nx * nx + 2.0f * ixy * nx * ny + iyy * ny * ny;
        if (std::abs(denom) < 1e-6f) continue;
        float t = -(ix * nx + iy * ny) / denom;

        // 极值点不能离当前像素太远
        if (std::abs(t) > stegerParams.maxT) continue;

        // 计算最终的亚像素坐标
        float px = x + t * nx;
        float py = y + t * ny;

        // 边界检查
        int px_int = cvRound(px);
        int py_int = cvRound(py);
        if (px_int < 0 || px_int >= gray.cols || py_int < 0 || py_int >= gray.rows) continue;

        // 哈希表 NMS：将二维坐标展平为一维 key
        float resp = std::abs(lambda1);
        size_t key = static_cast<size_t>(py_int) * static_cast<size_t>(gray.cols) + static_cast<size_t>(px_int);
        
        auto it = nmsMap.find(key);
        if (it == nmsMap.end() || resp > it->second.resp) {
            nmsMap[key] = {cv::Point2f(px, py), resp};
        }
    }

    // ==================================================================
    // 3. 从哈希表提取去重后的点，并还原缩放比例
    // ==================================================================
    for (const auto& pair : nmsMap) {
        finalPoints.push_back(cv::Point2f(pair.second.pt.x / downScale, 
                                          pair.second.pt.y / downScale));
    }
    
    return finalPoints;
}

// =====================================================================
// 函数 2：激光图像对前处理（定位棋盘格、提取ROI、提取激光线）
// =====================================================================

/**
 * @brief 处理一对图像（无激光和有激光），输出单张图的激光线点集和棋盘格位姿
 * @details 强制禁止降采样，确保标定时的极高精度。
 */
LaserProcessingResult LaserCalibration::processLaserPair(
    const cv::Mat& imgNoLaser, const cv::Mat& imgLaser, 
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, 
    const StegerParams& stegerParams, const LABParams& labParams)
{
    LaserProcessingResult result;
    result.success = false;
    result.poseValid = false;

    cv::Size useBoardSize = m_boardSize;
    if (useBoardSize.width < 3 || useBoardSize.height < 3) return result;
    if (imgNoLaser.empty() || imgLaser.empty()) return result;

    if (cameraMatrix.empty()) {
        qDebug() << "[solvePnP 调试] 相机内参矩阵为空！请检查单目标定是否成功。";
        return result;
    }
    
    // 标定流程中，彻底禁止外部缩放
    cv::Mat imgNoLaserProc = imgNoLaser;
    cv::Mat imgLaserProc = imgLaser;
    
    // 1. 在无激光图上提取棋盘格角点
    cv::Mat grayNoLaser;
    if (imgNoLaserProc.channels() == 3) cv::cvtColor(imgNoLaserProc, grayNoLaser, cv::COLOR_BGR2GRAY);
    else grayNoLaser = imgNoLaserProc;

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(grayNoLaser, useBoardSize, corners, 
                                           cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) return result;

    // 亚像素角点精化
    cv::cornerSubPix(grayNoLaser, corners, cv::Size(5, 5), cv::Size(-1, -1), 
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

    // 2. PnP 求解标定板相对于相机的位姿
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < useBoardSize.height; ++i)
        for (int j = 0; j < useBoardSize.width; ++j)
            objectPoints.push_back(cv::Point3f(j * m_squareSize, i * m_squareSize, 0.0f));

    // 输出世界坐标（转换为字符串）
    if (!objectPoints.empty()) {
        QString worldStr = QString("世界坐标首点: (%1, %2, %3), 末点: (%4, %5, %6)")
            .arg(objectPoints[0].x, 0, 'f', 4).arg(objectPoints[0].y, 0, 'f', 4).arg(objectPoints[0].z, 0, 'f', 4)
            .arg(objectPoints.back().x, 0, 'f', 4).arg(objectPoints.back().y, 0, 'f', 4).arg(objectPoints.back().z, 0, 'f', 4);
        qDebug() << "[solvePnP 调试] 世界坐标：" << worldStr;
    }

    if (!cameraMatrix.empty()) {
        try {
            result.poseValid = cv::solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, 
                                         result.rvec, result.tvec, false, cv::SOLVEPNP_ITERATIVE);
            // ==================== 调试输出：solvePnP 结果 ====================
            // 输出求解结果（转换为字符串）
            qDebug() << "[solvePnP 调试] 求解结果：" << (result.poseValid ? "成功" : "失败");
            if (result.poseValid) {
                QString rvecStr = QString("[%1, %2, %3]")
                    .arg(result.rvec[0], 0, 'f', 4)
                    .arg(result.rvec[1], 0, 'f', 4)
                    .arg(result.rvec[2], 0, 'f', 4);
                QString tvecStr = QString("[%1, %2, %3]")
                    .arg(result.tvec[0], 0, 'f', 4)
                    .arg(result.tvec[1], 0, 'f', 4)
                    .arg(result.tvec[2], 0, 'f', 4);
                
                qDebug() << "[solvePnP 调试] 旋转向量：" << rvecStr;
                qDebug() << "[solvePnP 调试] 平移向量：" << tvecStr;
            }
        } catch (const cv::Exception& e) {
            qDebug() << "[solvePnP 调试] 异常：" << e.what();
            result.poseValid = false;
        }
    }

    
    // 3. 计算包含标定板的 ROI 区域 (扩大1.4倍，确保激光线在ROI内)
    cv::Rect bbox = cv::boundingRect(corners);
    float roiScale = 1.5f;
    int paddingW = static_cast<int>((bbox.width * roiScale - bbox.width) / 2);
    int paddingH = static_cast<int>((bbox.height * roiScale - bbox.height) / 2);
    cv::Rect roi(bbox.x - paddingW, bbox.y - paddingH, 
                 bbox.width + 2 * paddingW, bbox.height + 2 * paddingH);
    
    // 防止ROI越界
    roi = roi & cv::Rect(0, 0, imgLaserProc.cols, imgLaserProc.rows);
    if (roi.width <= 0 || roi.height <= 0) return result;

    // 4. 在有激光图的 ROI 内提取激光中心
    cv::Mat roiLaser = imgLaserProc(roi);
    // 【核心】标定阶段强制传入 downScale = 1.0f，禁止缩放，保证亚像素级别精度！
    std::vector<cv::Point2f> localPts = detectLaserCenter_LAB_Steger(roiLaser, cv::Mat(), labParams, stegerParams, 1.0f);


    // 坐标从局部 ROI 还原到全图
    for (const auto& pt : localPts) {
        result.laserPoints.push_back(cv::Point2f(pt.x + roi.x, pt.y + roi.y));
    }
    
    result.roi = roi;
    result.success = !result.laserPoints.empty();
    
    // 【新增】调用RANSAC直线拟合函数
    if (result.success && result.laserPoints.size() > 2) {
        LineAnalysisResult lineResult = ransacLineFitting(result.laserPoints, 2.0f, 100);
        
        // 只保留RANSAC拟合的内点（符合直线模型的点）
        result.laserPoints = lineResult.pts;
        result.success = !result.laserPoints.empty();
        
        // 可选：存储直线分析结果
        result.lineAnalysis = lineResult;
    }
    
    return result;
}

// =====================================================================
// 函数 3：激光平面标定（方法B：基于极线约束与三角测量）
// =====================================================================

/**
 * @brief 传统的激光平面标定方法
 * @details 算法原理：
 * 1. 计算左相机光心及在右图的极点。
 * 2. 对左图每个激光点，反投影射线并求出在右图上的物理极线。
 * 3. 在右图中寻找距离该极线最近的点作为匹配点。
 * 4. 使用 cv::triangulatePoints 三角测量恢复三维点。
 * 5. SVD 拟合空间平面。
 */
LaserPlaneCalibrationResult LaserCalibration::calibrateLaserPlane(
    const std::vector<LaserPair>& laserData,
    const cv::Mat& P1, const cv::Mat& P2,
    float depthMin, float depthMax) 
{
    LaserPlaneCalibrationResult result;
    result.success = false;
    if (laserData.empty() || P1.empty() || P2.empty()) {
        result.resultMessage = "错误：数据或投影矩阵为空。";
        return result;
    }

    // 预计算矩阵
    cv::Mat P1_pinv;
    cv::invert(P1, P1_pinv, cv::DECOMP_SVD);
    
    // 左相机光心齐次坐标 (P * C = 0)
    cv::Mat C1_h = cv::Mat::zeros(4, 1, CV_64F);
    C1_h.at<double>(3) = 1.0;
    cv::Mat C1 = P1_pinv * C1_h;
    
    // 左光心投影到右图，即右图极点 epipole2
    cv::Mat epipole2_h = P2 * C1;
    cv::Point2f epipole2(static_cast<float>(epipole2_h.at<double>(0) / epipole2_h.at<double>(2)), 
                         static_cast<float>(epipole2_h.at<double>(1) / epipole2_h.at<double>(2)));

    std::vector<cv::Point3f> worldPoints;
    int totalPoints = 0;
    int failedMatches = 0;
    const float EPIPOLAR_THRESHOLD = 2.0f; // 极线距离容差（像素）

    for (size_t i = 0; i < laserData.size(); ++i) {
        const auto& ptsL = laserData[i].ptsL;
        const auto& ptsR = laserData[i].ptsR;
        if (ptsL.empty() || ptsR.empty()) continue;

        std::vector<cv::Point2f> matchedL, matchedR;

        // 极线匹配
        for (const auto& ptL : ptsL) {
            cv::Mat xL_h = (cv::Mat_<double>(3, 1) << ptL.x, ptL.y, 1.0);
            // 反投影到空间射线
            cv::Mat X_dir_h = P1_pinv * xL_h;
            // 射线上的点投影到右图，得到极线上的另一个点
            cv::Mat xR_dir_h = P2 * X_dir_h;
            if (std::abs(xR_dir_h.at<double>(2)) < 1e-6) continue;
            cv::Point2f pt_dir(static_cast<float>(xR_dir_h.at<double>(0) / xR_dir_h.at<double>(2)), 
                               static_cast<float>(xR_dir_h.at<double>(1) / xR_dir_h.at<double>(2)));

            // 遍历右图激光点，找距离极线 (epipole2 -> pt_dir) 最近的
            float minDist = std::numeric_limits<float>::max();
            int bestIdx = -1;
            for (int j = 0; j < ptsR.size(); ++j) {
                float dist = pointToLineDistance(ptsR[j], epipole2, pt_dir);
                if (dist < minDist) {
                    minDist = dist;
                    bestIdx = j;
                }
            }

            if (bestIdx != -1 && minDist < EPIPOLAR_THRESHOLD) {
                matchedL.push_back(ptL);
                matchedR.push_back(ptsR[bestIdx]);
            }
        }

        // 批量三角测量
        if (matchedL.size() >= 5) {
            cv::Mat points4D;
            cv::triangulatePoints(P1, P2, matchedL, matchedR, points4D);
            for (int j = 0; j < points4D.cols; ++j) {
                float w = points4D.at<float>(3, j);
                if (w > 1e-6 && std::abs(w) < 1e6) { // 齐次坐标合理性检查
                    cv::Point3f pt;
                    pt.x = points4D.at<float>(0, j) / w;
                    pt.y = points4D.at<float>(1, j) / w;
                    pt.z = points4D.at<float>(2, j) / w;
                    // 深度过滤
                    if (pt.z > depthMin && pt.z < depthMax) {
                        worldPoints.push_back(pt);
                        totalPoints++;
                    }
                } else {
                    failedMatches++;
                }
            }
        }
    }

    if (worldPoints.size() < 10) {
        result.resultMessage = QString("失败：有效三维点不足(仅%1个)。").arg(worldPoints.size());
        return result;
    }

    // SVD 特征值法拟合平面
    cv::Mat A(worldPoints.size(), 3, CV_64F);
    for (size_t i = 0; i < worldPoints.size(); ++i) {
        A.at<double>(i, 0) = worldPoints[i].x;
        A.at<double>(i, 1) = worldPoints[i].y;
        A.at<double>(i, 2) = worldPoints[i].z;
    }
    cv::Mat mean_pt;
    cv::reduce(A, mean_pt, 0, cv::REDUCE_AVG);
    cv::Mat A_centered = A - cv::repeat(mean_pt, A.rows, 1);
    
    cv::Mat w, u, vt;
    cv::SVD::compute(A_centered, w, u, vt);
    
    cv::Mat normal = vt.row(2);
    double A_coef = normal.at<double>(0);
    double B_coef = normal.at<double>(1);
    double C_coef = normal.at<double>(2);
    double D_coef = -(A_coef * mean_pt.at<double>(0) + B_coef * mean_pt.at<double>(1) + C_coef * mean_pt.at<double>(2));
    
    double norm = std::sqrt(A_coef*A_coef + B_coef*B_coef + C_coef*C_coef);
    A_coef /= norm; B_coef /= norm; C_coef /= norm; D_coef /= norm;

    result.success = true;
    result.worldPoints = worldPoints;
    result.planeCoeffs = (cv::Mat_<float>(4, 1) << static_cast<float>(A_coef), static_cast<float>(B_coef), 
                                                 static_cast<float>(C_coef), static_cast<float>(D_coef));
    result.totalPoints = totalPoints;
    result.failedMatches = failedMatches;
    
    double error_sum = 0.0;
    for(size_t i=0; i<worldPoints.size(); i++) {
        double dist = A_coef * worldPoints[i].x + B_coef * worldPoints[i].y + C_coef * worldPoints[i].z + D_coef;
        error_sum += dist * dist;
    }
    double rmse = std::sqrt(error_sum / worldPoints.size());
    result.resultMessage = QString("光平面标定完成！\n有效点数: %1\n方程: %2x + %3y + %4z + %5 = 0\n拟合中误差(RMSE): %6 mm")
                            .arg(worldPoints.size())
                            .arg(A_coef, 0, 'f', 4).arg(B_coef, 0, 'f', 4)
                            .arg(C_coef, 0, 'f', 4).arg(D_coef, 0, 'f', 4)
                            .arg(rmse, 0, 'f', 4);
    return result;
}

/**
 * @brief 类成员函数版本的点到直线距离计算
 */
float LaserCalibration::pointToLineDistance(const cv::Point2f& pt, const cv::Point2f& linePt1, const cv::Point2f& linePt2) {
    float dx = linePt2.x - linePt1.x;
    float dy = linePt2.y - linePt1.y;
    float norm = std::sqrt(dx * dx + dy * dy);
    if (norm < 1e-6f) return std::sqrt(std::pow(pt.x - linePt1.x, 2) + std::pow(pt.y - linePt1.y, 2));
    return std::abs(dy * pt.x - dx * pt.y + linePt2.x * linePt1.y - linePt2.y * linePt1.x) / norm;
}
