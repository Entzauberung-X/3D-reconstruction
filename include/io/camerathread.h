#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QThread>
#include <QString>
#include <atomic>
#include <opencv2/opencv.hpp>

class CameraThread : public QThread
{
    Q_OBJECT
public:
    explicit CameraThread(int cameraIndex, QObject *parent = nullptr);
    /**
     * 启动取流线程。**必须用它代替 QThread::start()**。
     * stop() 只把 running 置 false 就不再复位, 直接 start() 的话 run() 里
     * 摄像机 open() 成功、while(running) 却立刻退出 —— 现象是"相机打开了、
     * 按钮也显示已打开, 但一帧画面都没有"。自动采集在这种情况下会一路
     * 回 0x03 却一张图都存不下 (见 MainWindow::onStartAutoCollect)。
     */
    void startStreaming();
    void stop();

    /**
     * 主线程取走一帧后必须调用, 否则取流线程会一直认为"上一帧还没被消费"而停止交付。
     * 交付是**有背压的**: 同时最多只有一帧在事件队列里 (见 run() 里的说明)。
     */
    void frameConsumed() { m_framePending = false; }

signals:
    void matReady(const cv::Mat &mat);
    // 摄像头打开失败或读取中断时上报，供 UI 显示错误提示
    void cameraFailed(int cameraIndex, const QString &reason);

protected:
    void run() override;

private:
    cv::VideoCapture cap;
    std::atomic<bool> running; // 跨线程 stop() 需原子操作
    std::atomic<bool> m_framePending{false};  // 上一帧是否还没被主线程取走 (背压闸门)
    int m_cameraIndex; // 新增：摄像头索引
};
    

#endif // CAMERATHREAD_H
