#ifndef LASERWORKER_H
#define LASERWORKER_H

#include <QObject>
#include <QVector>
#include <QString>
#include <opencv2/opencv.hpp>
#include "core/lasercalibration.h"

class LaserWorker : public QObject {
    Q_OBJECT
public:
    explicit LaserWorker(QObject *parent = nullptr);

    void setFiles(const QVector<QString>& noLaserL, const QVector<QString>& laserL,
                const QVector<QString>& noLaserR, const QVector<QString>& laserR,
                const LaserCalibration& calib,
                const cv::Mat& camMatL, const cv::Mat& distL,
                const cv::Mat& camMatR, const cv::Mat& distR,
                const StegerParams& params,
                const LABParams& labParams); 

public slots:
    void process();

signals:
    // 【重构】废除原来的无参 finished，改为打包抛出所有数据
    // Qt 的隐式共享机制保证了这里传参是“零拷贝”的，非常高效
    void finished(QVector<LaserPair> validPairs, 
                  QVector<LaserProcessingResult> cacheL, 
                  QVector<LaserProcessingResult> cacheR, 
                  int totalProcessed);
                  
    void error(QString err);
    void progress(int current, int total);
    
    // 废弃: void resultReady(int idx, LaserProcessingResult resL, LaserProcessingResult resR);

private:
    QVector<QString> m_noLaserL, m_LaserL;
    QVector<QString> m_noLaserR, m_LaserR;
    
    LaserCalibration m_calibrator; 
    StegerParams m_params;
    LABParams m_labParams;

    cv::Mat m_camMatL, m_distL;
    cv::Mat m_camMatR, m_distR;
};

#endif // LASERWORKER_H
