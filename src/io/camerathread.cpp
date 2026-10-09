#include "io/camerathread.h"
#include <QDebug>

namespace {
/** 把 OpenCV 的 fourcc 整数还原成可读四字符 (排查格式协商问题用) */
QString fourccToString(double v)
{
    const int f = static_cast<int>(v);
    if (f == 0) return "未设置";
    QString s;
    for (int i = 0; i < 4; ++i) {
        const char c = static_cast<char>((f >> (8 * i)) & 0xFF);
        s.append((c >= 32 && c < 127) ? QChar(c) : QChar('?'));
    }
    return s;
}
} // namespace

CameraThread::CameraThread(int cameraIndex, QObject *parent)
    : QThread(parent), running(true), m_cameraIndex(cameraIndex)
{
}

void CameraThread::startStreaming()
{
    if (isRunning()) return;
    running = true;          // 先复位再启线程: 否则"关闭相机"之后再"打开"只会 open() 到设备, 一帧都不取
    QThread::start();
}

void CameraThread::stop()
{
    running = false;
}

void CameraThread::run()
{
    // 使用索引打开摄像头
    if (!cap.open(m_cameraIndex, cv::CAP_V4L2)) {
        emit cameraFailed(m_cameraIndex, QString("无法打开摄像头 %1").arg(m_cameraIndex));
        return;
    }

    // ---- 像素格式: 必须优先设为 MJPEG ----
    // 本机是两台同型号 UVC 相机共用一条 USB 2.0 总线。未压缩(YUYV)在 1280x960 下
    // 每帧 1280*960*2 ≈ 2.46MB, 30fps 需要约 590Mbps, 已超过 USB 2.0 的 480Mbps 上限,
    // 于是第二台相机在建立流式接口时拿不到等时带宽, 表现为
    //   open() 成功但 read() 永远失败, 内核日志出现
    //   "usb X-Y: 3:0: usb_set_interface failed (-75)"  (-75 = EOVERFLOW)
    // OpenCV 的 V4L2 后端默认走 YUYV, 因此这里必须显式指定 MJPG。
    // 注意: 像素格式会改变设备**可用分辨率列表**, 所以要在设分辨率之前设格式。
    const int mjpg = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    const bool fourccOk = cap.set(cv::CAP_PROP_FOURCC, mjpg);

    // 设置分辨率 (旧值保留为注释, 便于回退)
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 2592);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1944);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 960);

    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // ---- 把实际协商结果读回来 ----
    // 驱动对不支持的尺寸/格式通常不报错而是"就近取", 静默降级会导致采集分辨率与
    // 标定内参不一致(整条重建链路都会错), 而且过去只会显示一句"读取中断", 无法排查。
    const int gotW = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int gotH = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const QString gotFmt = fourccToString(cap.get(cv::CAP_PROP_FOURCC));
    qDebug() << QString("[相机 %1] 请求 MJPG 1280x960 -> 实际 %2 %3x%4 (fourcc 设置%5)")
                    .arg(m_cameraIndex).arg(gotFmt).arg(gotW).arg(gotH)
                    .arg(fourccOk ? "成功" : "失败");

    cv::Mat frame;
    int okFrames = 0;

    while (running) {
        if (!cap.read(frame)) {
            // 摄像头断开或读取失败：上报 UI 而不是静默退出。
            // 带上"已成功读过多少帧"与协商结果 —— 一帧都没读到基本就是带宽/格式问题,
            // 读到过再断则是拔线或掉电, 两者处理方式完全不同。
            const QString detail = okFrames > 0
                ? QString("第 %1 帧后中断").arg(okFrames)
                : QString("始终无法取帧 (协商为 %1 %2x%3, 请检查 USB 带宽/端口)")
                      .arg(gotFmt).arg(gotW).arg(gotH);
            emit cameraFailed(m_cameraIndex, QString("摄像头 %1 %2").arg(m_cameraIndex).arg(detail));
            break;
        }
        ++okFrames;

        if (frame.empty()) continue;

        // 上一帧还没被主线程取走就不再交新的。
        //
        // 跨线程信号是**排队**的: 主线程一忙(重建/弹窗/拖窗口), 每个 matReady 里
        // 3.5MB 的 clone 就堆在事件队列里等, 两路相机 60 帧/秒 = 210MB/秒 ——
        // 十几秒就能把 3.8GB 的机器顶到 OOM(实测进程涨到 2.5GB 被内核杀掉,
        // 表现就是"重建跑着跑着卡死")。预览丢帧没人看得出来, 堆队列会要命。
        if (!m_framePending.exchange(true)) {
            emit matReady(frame.clone());
        }

        QThread::msleep(10);
    }

    cap.release();
}
