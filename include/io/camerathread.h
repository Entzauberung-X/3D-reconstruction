#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QThread>
#include <opencv2/opencv.hpp>

class CameraThread : public QThread
{
    Q_OBJECT
public:
    explicit CameraThread(int cameraIndex, QObject *parent = nullptr);
    void stop();

signals:
    void matReady(const cv::Mat &mat); 
    void frameReady(const QImage &img);

protected:
    void run() override;

private:
    cv::VideoCapture cap;
    bool running;
    int m_cameraIndex; // 新增：摄像头索引
};
    

#endif // CAMERATHREAD_H
