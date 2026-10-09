#ifndef RECONSTRUCTIONWORKER_H
#define RECONSTRUCTIONWORKER_H

#include <QObject>
#include <QStringList>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <atomic>
#include "core/pointcloudbuilder.h"

/**
 * @brief 三维重建后台工作线程 (与 LaserWorker 同模式)
 * @details 在独立 QThread 中执行 S1~S6 重建流水线，避免大场景重建阻塞 UI。
 *          通过信号槽向主线程回报进度/日志/预览图，完成后再由主线程读取结果。
 *          拥有独立的 PointCloudBuilder 实例，与主线程 (Tab4/预览) 共用对象无冲突。
 */
class ReconstructionWorker : public QObject
{
    Q_OBJECT
public:
    // 一次性传入的作业数据 (在主线程组装完毕后经队列传递给工作线程)
    struct JobInput {
        QStringList leftPaths;
        QStringList rightPaths;
        double angleStepDeg = 1.8;
        bool rectified = false;
        cv::Mat mapL1, mapL2, mapR1, mapR2;   // 立体校正映射表
        CalibrationData calibData;
        ReconstructionParams params;
        // 轴点/方向都是**转台基准系** (= S4 输出系) 下的量, 不是相机系。
        // 默认值取本工程标定约定的竖直轴 +Z; axisValid=false 时 worker 会用装配点云的
        // XY 形心覆盖轴点 (见 reconstructionworker.cpp 里安全降级那段)。
        Eigen::Vector3f axisPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisDir   = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        bool axisValid = false;
    };

    explicit ReconstructionWorker(QObject *parent = nullptr);

    // 结果读取 (收到 finished 信号后由主线程调用，工作线程已空闲，读取安全)
    pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud() const;
    pcl::PolygonMesh resultMesh() const;

    /**
     * @brief 逐视角明细 (S1~S4), 已按视角序号 0..N-1 排好序。
     *
     * 为什么要在这里收集而不是让调用方读 builder 的: 视角级并行把工作切成若干块,
     * 每块一个 PointCloudBuilder, 各块的 records 拼起来的顺序是"按块交错"的,
     * 不是全局帧序 —— 直接拿去画逐帧曲线会得到一条乱序的折线。
     * worker 在每块内处理完一帧就立刻取该块最后一条记录, 按视角序号回收, 于是天然有序。
     */
    const std::vector<ViewRecord>& viewRecords() const { return m_viewRecords; }

    /**
     * @brief 本次运行是否被取消。
     * 为什么需要它: finished() 在两条路径上都会发出, 而"取消"与"跑挂了"
     * 在结果上看起来一样 (点云为空)。调用方 (Tab6 重建测试) 若不区分, 会把用户主动取消
     * 报成一次"未通过"的失败。
     */
    bool wasCancelled() const { return m_cancel.load(); }

    /**
     * @brief 逐视角点云 (S4 输出, 转台基准系), **按视角序号索引**, 该视角无产出时为 nullptr。
     *
     * 为什么要按索引而不是 push_back: 下游要把点云和 viewRecords()[i] 的计数按帧对齐
     * (调试包里的「逐视角点云」要带角度/产出率), 丢帧后顺序就错位了。
     * 为什么要在 S5 之前留一份: S5 会按 i×步长 把每个视角旋转后融合, 融合完就再也分不出
     * "哪一片来自哪一帧" —— 而出问题时要问的恰恰是"某一帧的点跑到哪去了"。
     */
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& viewClouds() const { return m_viewClouds; }

    /**
     * @brief 整段 S1~S4 的汇总统计 (直方图/深度范围/各阶段剔除总数)。
     *
     * 为什么要单独暴露: worker 用它打完汇总日志后就丢掉了, 而其中 depthMin/Max 与
     * 8 桶直方图**无法由逐视角 records 还原**。调试包要给出权威的总量, 只能从这里取。
     */
    const ViewOutcomeStats& mergedStats() const { return m_mergedStats; }

    /**
     * @brief 本次 S6 底面填充往最终点云里灌进去的**合成点**数量。
     *
     * 为什么要透出来: 这些点是照着拟合平面按 3mm 网格造出来的, 不是测出来的, 但会写进
     * 最终 PLY。排查"点云里有不存在的平面"时, 这是第一个该看的数。
     */
    int bottomFilledCount() const { return m_builder.bottomFilledCount(); }

    /**
     * @brief 主线程画完一帧预览后必须调用, 否则不再交付新的预览图。
     *
     * 交付是**带背压**的 (同时最多一对图在队列里): 每帧预览是两张 1280x960 的 QImage
     * (7MB), 跨线程信号排队时主线程一忙就会在事件队列里堆积 —— 200 个视角就是 1.4GB。
     */
    void framePreviewConsumed() { m_viewImagePending = false; }

public slots:
    void run(const ReconstructionWorker::JobInput &job);
    void cancel();

signals:
    void progress(int current, int total);
    void logMessage(const QString &line);
    void viewImage(int index, const QImage &left, const QImage &right);
    void finished();
    void error(const QString &message);

private:
    PointCloudBuilder m_builder;   // 线程内专用，避免与主线程共享
    // m_cancel 在 run() 开头清零、cancel() 置位, 所以本次运行结束后它恰好记录了
    // "这一轮是否被取消" —— wasCancelled() 直接读它即可, 不需要第二个标志位。
    std::atomic<bool> m_cancel;
    std::atomic<bool> m_viewImagePending{false};  // 上一对预览图是否还没被主线程画完 (背压闸门)
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PolygonMesh m_mesh;
    std::vector<ViewRecord> m_viewRecords;   // 逐视角明细 (按视角序号排序, 见 viewRecords())
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_viewClouds;  // 逐视角点云 (按视角序号索引, 见 viewClouds())
    ViewOutcomeStats m_mergedStats;          // 整段汇总 (见 mergedStats())
};

Q_DECLARE_METATYPE(ReconstructionWorker::JobInput)

#endif // RECONSTRUCTIONWORKER_H
