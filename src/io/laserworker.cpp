#include "io/laserworker.h"
#include <QDebug>

LaserWorker::LaserWorker(QObject *parent) : QObject(parent) {}

void LaserWorker::setFiles(const QVector<QString>& noLaserL, const QVector<QString>& laserL,
                           const QVector<QString>& noLaserR, const QVector<QString>& laserR,
                           const LaserCalibration& calib,
                           const cv::Mat& camMatL, const cv::Mat& distL,
                           const cv::Mat& camMatR, const cv::Mat& distR,
                           const StegerParams& params,
                           const LABParams& labParams)
{
    m_noLaserL = noLaserL; m_LaserL = laserL;
    m_noLaserR = noLaserR; m_LaserR = laserR;
    m_calibrator = calib;
    m_params = params;
    m_labParams = labParams;
    
    // ✅ 加固 1：深拷贝前先判空，防止把“空矩阵”合法化
    m_camMatL = camMatL.empty() ? cv::Mat() : camMatL.clone();
    m_distL   = distL.empty()   ? cv::Mat() : distL.clone();
    m_camMatR = camMatR.empty() ? cv::Mat() : camMatR.clone();
    m_distR   = distR.empty()   ? cv::Mat() : distR.clone();
}

void LaserWorker::process() {
    int total = m_noLaserL.size();
    if (total == 0) {
        emit finished(QVector<LaserPair>(), QVector<LaserProcessingResult>(), QVector<LaserProcessingResult>(), 0);
        return;
    }

    // ✅ 加固 2：循环前直接拦截！如果内参是空的，直接报错返回，绝不浪费时间去读图
    if (m_camMatL.empty() || m_distL.empty() || m_camMatR.empty() || m_distR.empty()) {
        QString errMsg = "工作线程启动失败：接收到的相机内参或畸变系数为空！";
        qDebug() << "[Worker 错误]" << errMsg;
        emit error(errMsg);
        // 抛出空结果，防止主线程死锁或崩溃
        emit finished(QVector<LaserPair>(), QVector<LaserProcessingResult>(), QVector<LaserProcessingResult>(), 0);
        return; 
    }

    QVector<LaserPair> localValidPairs;
    // 预分配空间，提升性能
    QVector<LaserProcessingResult> localCacheL(total);
    QVector<LaserProcessingResult> localCacheR(total);

    for (int i = 0; i < total; ++i) {
        emit progress(i + 1, total);

        cv::Mat imgNoL_L = cv::imread(m_noLaserL[i].toStdString());
        cv::Mat imgL_L   = cv::imread(m_LaserL[i].toStdString());
        cv::Mat imgNoL_R = cv::imread(m_noLaserR[i].toStdString());
        cv::Mat imgL_R   = cv::imread(m_LaserR[i].toStdString());

        if (imgNoL_L.empty() || imgL_L.empty() || imgNoL_R.empty() || imgL_R.empty()) {
            emit error(QString("无法读取图像对 %1").arg(i));
            localCacheL[i] = LaserProcessingResult();
            localCacheR[i] = LaserProcessingResult();
            continue;
        }

        // 参数完美匹配
        LaserProcessingResult resL = m_calibrator.processLaserPair(
            imgNoL_L, imgL_L, m_camMatL, m_distL, m_params, m_labParams);
            
        LaserProcessingResult resR = m_calibrator.processLaserPair(
            imgNoL_R, imgL_R, m_camMatR, m_distR, m_params, m_labParams);

        localCacheL[i] = resL;
        localCacheR[i] = resR;

        // 只要左右都提取到了激光点，就收集起来
        if (resL.success && resR.success && !resL.laserPoints.empty() && !resR.laserPoints.empty()) {
            LaserPair pair;
            pair.ptsL = resL.laserPoints;
            pair.ptsR = resR.laserPoints;
            
            // 位姿传递
            pair.rvecL = resL.rvec; pair.tvecL = resL.tvec; pair.poseValidL = resL.poseValid;
            pair.rvecR = resR.rvec; pair.tvecR = resR.tvec; pair.poseValidR = resR.poseValid;
            
            localValidPairs.append(pair);
        }
    }

    // 抛出最终结果
    emit finished(localValidPairs, localCacheL, localCacheR, total);
}
