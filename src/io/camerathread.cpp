#include "io/camerathread.h"
#include <QDebug>
#include <QImage>

CameraThread::CameraThread(int cameraIndex, QObject *parent)
    : QThread(parent), running(true), m_cameraIndex(cameraIndex)
{
}

void CameraThread::stop()
{
    running = false;
}

void CameraThread::run()
{
    // 使用索引打开摄像头
    if (!cap.open(m_cameraIndex, cv::CAP_V4L2)) {
        qDebug() << "无法打开摄像头" << m_cameraIndex;
        return;
    }

    // 设置分辨率
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 2592);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1944);
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 960);

    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    cv::Mat frame;

    while (running) {
        if (!cap.read(frame)) {
            break;
        }

        if (frame.empty()) continue;

        emit matReady(frame.clone());

        cv::Mat rgbFrame;
        cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);

        QImage qimg(rgbFrame.data,
                    rgbFrame.cols,
                    rgbFrame.rows,
                    static_cast<int>(rgbFrame.step),
                    QImage::Format_RGB888);

        emit frameReady(qimg.copy());
        
        QThread::msleep(10);
    }

    cap.release();
}
