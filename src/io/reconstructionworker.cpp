#include "io/reconstructionworker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QThread>
#include <QDebug>
#include <QFile>
#include <atomic>
#include <algorithm>
#ifdef __linux__
#include <unistd.h>
#endif

namespace {
/**
 * 当前进程常驻内存 (MB)。
 *
 * 为什么要打这个数: 这台机器只有 3.8GB 内存, 重建一跑就被内核 OOM 杀掉
 * (实测 RSS 涨到 2.5GB, 现象是"卡死")。光看日志里的规则/参数猜不出内存花在哪一段,
 * 而 S1~S4 / S5 配准 / S6 去噪 / 网格化 四段的省法完全不同, 所以每段都留一个读数。
 * 读不到时返回 -1, 不影响流程。
 */
double rssMb()
{
#ifdef __linux__
    QFile f(QStringLiteral("/proc/self/statm"));
    if (!f.open(QIODevice::ReadOnly)) return -1.0;
    const QList<QByteArray> parts = f.readAll().trimmed().split(' ');
    if (parts.size() < 2) return -1.0;
    bool ok = false;
    const qlonglong pages = parts[1].toLongLong(&ok);   // 第 2 个字段 = 常驻页数
    if (!ok) return -1.0;
    return double(pages) * (double(sysconf(_SC_PAGESIZE)) / (1024.0 * 1024.0));
#else
    return -1.0;
#endif
}

// 线程内将 cv::Mat 转为 QImage 供 UI 预览 (与 MainWindow::mat2QImage 等价)
QImage matToQImage(const cv::Mat &mat)
{
    if (mat.empty() || mat.cols <= 0 || mat.rows <= 0) return QImage();
    if (mat.type() == CV_8UC1)
        return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8).copy();
    if (mat.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
    return QImage();
}
} // namespace

ReconstructionWorker::ReconstructionWorker(QObject *parent)
    : QObject(parent), m_cancel(false)
{
    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void ReconstructionWorker::cancel()
{
    m_cancel = true;
}

void ReconstructionWorker::run(const JobInput &job)
{
    m_cancel = false;
    m_viewImagePending = false;   // 上一轮最后一帧可能没人取走, 不清掉这轮就一张预览都不给
    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_mesh = pcl::PolygonMesh();
    m_viewRecords.clear();
    m_viewClouds.clear();
    m_mergedStats = ViewOutcomeStats();

    const int totalViews = qMin(job.leftPaths.size(), job.rightPaths.size());
    if (totalViews <= 0) {
        emit error("图像序列为空，无法重建！");
        emit finished();
        return;
    }

    m_builder.setCalibrationData(job.calibData);
    m_builder.setReconstructionParams(job.params);

    // ========================================================================
    // S1~S4: 逐视角处理 —— 【P0-4】并行化
    //   各视角之间完全独立 (只共享只读的标定/参数)，按视角分块并行执行。
    //   注意: PointCloudBuilder::lastDetailLog 是成员变量、非线程安全，
    //         因此每个并行分块使用各自的 builder 实例，结果按视角序号汇总。
    // ========================================================================
    emit logMessage(QString("====== [流水线] 开始 S1~S4 逐视角处理 ======  [内存 %1 MB]")
                        .arg(rssMb(), 0, 'f', 0));

    int threads = job.params.parallel_threads;
    if (threads <= 0) threads = QThread::idealThreadCount();   // 0 = 自动
    threads = std::max(1, std::min(threads, 32));
    threads = std::min(threads, totalViews);
    emit logMessage(QString(">>> 并行线程数: %1 (视角数 %2, 0=自动按CPU核心)").arg(threads).arg(totalViews));

    struct ViewResult {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        double angle = 0.0;
        QString log;
        bool ok = false;
        ViewRecord record;   // 逐视角明细 (见 ReconstructionWorker::viewRecords())
    };
    std::vector<ViewResult> results(totalViews);
    std::atomic<int> doneCount(0);
    // 每块(线程)一份逐视角统计, 并行结束后 merge 成一条汇总
    // (逐视角打印这些计数就是刷屏主力, 见 ViewOutcomeStats 说明)
    std::vector<ViewOutcomeStats> threadStats(threads);

    // 视角级并行时把 OpenCV 内部线程数压到 1，避免与视角并行叠加造成线程超订
    const int prevCvThreads = cv::getNumThreads();
    cv::setNumThreads(1);

    QVector<QFuture<void>> futures;
    futures.reserve(threads);
    for (int t = 0; t < threads; ++t) {
        futures.append(QtConcurrent::run([&, t]() {
            PointCloudBuilder builder;   // 分块私有实例
            builder.setCalibrationData(job.calibData);
            builder.setReconstructionParams(job.params);

            // 统一推进进度 + 周期性状态日志
            // (逐视角详情日志在并行结束后按顺序统一输出，避免多线程交错)
            auto advance = [&]() {
                const int done = doneCount.fetch_add(1) + 1;
                emit progress(done, totalViews);
                const int stride = std::max(1, totalViews / 10);
                if (totalViews >= 20 && done % stride == 0)
                    emit logMessage(QString(">>> S1~S4 进度 %1/%2 视角").arg(done).arg(totalViews));
            };

            for (int i = t; i < totalViews; i += threads) {
                if (m_cancel) break;

                cv::Mat imgL = cv::imread(job.leftPaths[i].toStdString());
                cv::Mat imgR = cv::imread(job.rightPaths[i].toStdString());
                if (imgL.empty() || imgR.empty()) {
                    results[i].log = QString("[%1/%2] ⚠️ 读取失败").arg(i + 1).arg(totalViews);
                    results[i].record.frameIdx = i;      // 读图失败也要占位, 否则曲线上会缺帧
                    results[i].record.angleDeg = i * job.angleStepDeg;
                    advance();
                    continue;
                }

                // 注意: 此处【不要】对图像做 remap 立体校正!
                // processSingleView 的设计输入是“原始畸变图像”: S1 在原始图上提取光条
                // → 内部 undistortPoints(K,D,R_rect,P_rect) 转到校正系 → S2 匹配 → S3 三角化。
                // 若先 remap 再交给 processSingleView，校正会被施加两次，3D 点坐标错误。
                // ROI (m_roiLeft/m_roiRight) 也是按原始图框选的，直接作用于原始图才一致。
                // 背压: 主线程还没画完上一对就不再交新的。
                // 交付是排队的, 每对是两张 1280x960 的 QImage (7MB), 主线程一忙
                // (转 `QPixmap` 本身就慢) 这些图就在事件队列里堆积 —— 200 个视角
                // 能堆到 1.4GB, 是"重建跑着跑着把机器吃爆"的另一半原因。
                // 预览少刷几帧没人看得出来, 堆队列会要命。
                if (!m_viewImagePending.exchange(true)) {
                    emit viewImage(i, matToQImage(imgL), matToQImage(imgR));
                }

                const double currentAngle = i * job.angleStepDeg;
                pcl::PointCloud<pcl::PointXYZ>::Ptr singleCloud =
                    builder.processSingleView(imgL, imgR, currentAngle, i);

                // 本块 builder 刚处理的就是第 i 帧, 它的最后一条 records 即本帧明细。
                // 在此逐帧回收 (而不是等并行结束后 merge 各块), 顺序才与视角号一致。
                if (!builder.viewRecords().empty())
                    results[i].record = builder.viewRecords().back();

                if (singleCloud && !singleCloud->empty()) {
                    results[i].cloud = singleCloud;
                    results[i].angle = currentAngle;
                    results[i].ok = true;
                    results[i].log = QString("[%1/%2] %3").arg(i + 1).arg(totalViews).arg(builder.lastDetailLog);
                } else {
                    results[i].log = QString("[%1/%2] ❌ %3").arg(i + 1).arg(totalViews).arg(builder.lastDetailLog);
                }
                advance();
            }
            threadStats[t] = builder.viewStats();   // 本块统计交回主流程汇总
        }));
    }
    for (QFuture<void> &f : futures) f.waitForFinished();
    cv::setNumThreads(prevCvThreads);

    if (m_cancel) {
        emit logMessage(">>> 用户取消重建 (S1~S4)");
        emit finished();
        return;
    }

    // 按视角顺序汇总 (保证日志与点云顺序与串行版本一致)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> allViewClouds;
    std::vector<double> viewAngles;
    allViewClouds.reserve(totalViews);
    for (int i = 0; i < totalViews; ++i) {
        if (!results[i].log.isEmpty()) emit logMessage(results[i].log);
        if (results[i].ok && results[i].cloud && !results[i].cloud->empty()) {
            allViewClouds.push_back(results[i].cloud);
            viewAngles.push_back(results[i].angle);
        }
    }

    // 逐视角明细/点云按视角序号回收 (即使后面因"无有效点云"提前返回, 这份数据也仍然有用:
    // 它正是判断"点为什么没出来"的依据, 所以放在下面的 early-return 之前)。
    // 点云**按索引**填, 失败帧留 nullptr —— push_back 会让帧号与点云错位。
    m_viewRecords.reserve(totalViews);
    m_viewClouds.assign(totalViews, pcl::PointCloud<pcl::PointXYZ>::Ptr());
    for (int i = 0; i < totalViews; ++i) {
        m_viewRecords.push_back(results[i].record);
        if (results[i].ok && results[i].cloud && !results[i].cloud->empty())
            m_viewClouds[i] = results[i].cloud;
    }

    emit logMessage(QString("====== S1~S4 结束: 有效视角 %1 个 ======  [内存 %2 MB]\n")
                        .arg(allViewClouds.size()).arg(rssMb(), 0, 'f', 0));

    // ---- 逐视角统计汇总 (一条, 替代原先每个视角各打一遍的终端噪声) ----
    {
        ViewOutcomeStats st;
        for (const auto& s : threadStats) st.merge(s);
        m_mergedStats = st;   // 留给调试包: depthMin/Max 与 8 桶直方图无法由逐视角 records 还原
        QString depth = (st.depthMaxMm > st.depthMinMm)
            ? QString("[%1, %2]").arg(st.depthMinMm, 0, 'f', 1).arg(st.depthMaxMm, 0, 'f', 1)
            : QString("(无)");
        emit logMessage(QString(">>> [逐视角汇总] 处理 %1 | S1提取空 %2 | S2匹配零 %3 | S3三角化空 %4\n"
                                "                   S3剔除: 负深度 %5, 深度越界 %6, 射线平行 %7, 平面残差 %8\n"
                                "                   有效点数 %9 | 深度范围 %10 mm")
            .arg(st.views).arg(st.extractEmpty).arg(st.matchEmpty).arg(st.triEmpty)
            .arg(st.negDepth).arg(st.tooFar).arg(st.degenerate).arg(st.reprojReject)
            .arg(st.points).arg(depth));

        // 右图重投影误差分布 (px): 判断错配是"差一点点" (误差挤在阈值上方一小段)
        // 还是光平面/匹配基本是错的 (误差高达几十~几百 px) —— 两者处理完全不同。
        // 注意接受/拒绝的判据已经换成毫米(见 reproj_reject_mm), 这里只作诊断口径:
        // 阈值处在新旧单位交界上, 用 px 直方图能一眼看出错配是集中还是一泻千里。
        const int* h = st.reprojHist;
        const int histTotal = h[0]+h[1]+h[2]+h[3]+h[4]+h[5]+h[6]+h[7];
        if (histTotal > 0) {
            auto pct = [&](int n) { return 100.0 * n / histTotal; };
            emit logMessage(QString(">>> [右图重投影误差分布] 共 %1 个匹配 (判据: 空间残差 ≤ %2mm)\n"
                                    "      <1px %3 (%4%) | 1-2px %5 (%6%) | 2-5px %7 (%8%) | 5-10px %9 (%10%)\n"
                                    "      10-20px %11 (%12%) | 20-50px %13 (%14%) | 50-100px %15 (%16%) | ≥100px %17 (%18%)")
                .arg(histTotal).arg(job.params.reproj_reject_mm, 0, 'f', 1)
                .arg(h[0]).arg(pct(h[0]), 0, 'f', 1).arg(h[1]).arg(pct(h[1]), 0, 'f', 1)
                .arg(h[2]).arg(pct(h[2]), 0, 'f', 1).arg(h[3]).arg(pct(h[3]), 0, 'f', 1)
                .arg(h[4]).arg(pct(h[4]), 0, 'f', 1).arg(h[5]).arg(pct(h[5]), 0, 'f', 1)
                .arg(h[6]).arg(pct(h[6]), 0, 'f', 1).arg(h[7]).arg(pct(h[7]), 0, 'f', 1));
        }
    }
    if (allViewClouds.empty()) {
        emit error("无有效点云！请检查标定与图像质量。");
        emit finished();
        return;
    }
    if (m_cancel) {
        emit logMessage(">>> 用户取消重建 (S5 前)");
        emit finished();
        return;
    }

    // ===== S5: 多视角配准 =====
    // 标题必须随模式变: 关闭 ICP 时仍打「S5 ICP 配准」会让人以为下面的数字来自 ICP。
    emit logMessage(job.params.use_icp
        ? "====== [S5 ICP 配准] 开始 ======"
        : "====== [S5 纯轴旋转拼接] 开始 (ICP 已关闭) ======");
    Eigen::Vector3f axisPoint = job.axisPoint;
    Eigen::Vector3f axisDir   = job.axisDir;
    if (job.axisValid) {
        // 坐标系约定: S4 已把各视角点云变到"转台基准系(棋盘0系)"(= R_base^T)，
        // 因此 S5 的视角增量旋转必须使用**同一坐标系**下的轴。
        // 轴标定模块 (RotatingCalibrator) 已统一在该坐标系下输出轴方向/轴点，
        // 这里直接使用，切勿再做 R_base^T 之类的二次变换。
        emit logMessage(">>> [精准模式] 使用完整标定结果 (坐标系: 转台基准系 = S4 输出系)");
        emit logMessage(QString(">>> 轴方向[%1,%2,%3] 轴点[%4,%5,%6]mm")
            .arg(axisDir.x(), 0, 'f', 4).arg(axisDir.y(), 0, 'f', 4).arg(axisDir.z(), 0, 'f', 4)
            .arg(axisPoint.x(), 0, 'f', 2).arg(axisPoint.y(), 0, 'f', 2).arg(axisPoint.z(), 0, 'f', 2));
    } else {
        // 安全降级: 轴标定缺失时, 就地估一条轴, 且**必须给在转台基准系里** ——
        // S5 绕的就是这个系 (见上面"坐标系约定")。
        // 早先的默认值 (30,0,250) / (0,1,0) 是按**相机系**写的: 相机系里转台轴确实大致是
        // +Y、转台中心确实在光轴前方 250mm 处, 但这两个数被直接喂给基准系下的旋转,
        // 等于绕一条差了约 90° 的轴、转过一个 250mm 外的点 —— 拼出来的云必然是散的,
        // 而日志还写着"若拼接呈螺旋状错位…", 把一次坐标系错用说成了参数没调好。
        //
        // 就地估法:
        //   方向 —— 本工程标定约定的竖直轴 +Z
        //           (轴方向(转台基准系) 历来是 [-0.0008, 0.0016, 1.0000])
        //   轴点 —— 全部装配点云的 XY 形心。物体就摆在转台上, 这是手里能拿到的最好猜测;
        //           轴点只有**垂直于轴的分量**参与运算, 所以 z 取什么都无所谓。
        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        size_t cnt = 0;
        for (const auto& c : allViewClouds)
            for (const auto& p : c->points) { sum += Eigen::Vector3d(p.x, p.y, p.z); ++cnt; }
        if (cnt > 0) {
            const Eigen::Vector3d c = sum / double(cnt);
            axisPoint = Eigen::Vector3f(float(c.x()), float(c.y()), float(c.z()));
        } else {
            axisPoint = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        }
        axisDir = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

        emit logMessage(">>> [安全降级] 未检测到旋转轴标定结果！");
        emit logMessage(QString(">>> 降级策略: 轴点取装配点云 XY 形心 [%1, %2] mm (由 %3 个点估出)")
            .arg(axisPoint.x(), 0, 'f', 2).arg(axisPoint.y(), 0, 'f', 2).arg(cnt));
        emit logMessage(">>> 降级策略: 旋转方向取本工程约定中的竖直轴 +Z [0, 0, 1]");
        emit logMessage(">>> ⚠️ 这只是保底估算, 拼接结果仅供预览。请到 Tab4 执行旋转轴自动标定后重跑。");
    }

    m_builder.processGlobal(allViewClouds, viewAngles, axisPoint, axisDir);   // 内部逐段打 [内存]
    if (!m_builder.lastDetailLog.isEmpty())
        emit logMessage(m_builder.lastDetailLog);

    m_cloud = m_builder.getFinalPointCloud();
    m_mesh  = m_builder.getFinalMesh();

    emit logMessage(QString("====== [S6 结束] 点云: %1 | 网格: %2 ======  [内存 %3 MB]")
        .arg(m_cloud ? m_cloud->size() : 0).arg(m_mesh.polygons.size()).arg(rssMb(), 0, 'f', 0));
    emit finished();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ReconstructionWorker::resultCloud() const { return m_cloud; }
pcl::PolygonMesh ReconstructionWorker::resultMesh() const { return m_mesh; }
