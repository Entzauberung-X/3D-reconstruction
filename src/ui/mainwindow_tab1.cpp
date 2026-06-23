// ==================== Tab1: 双目采集 ====================
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QDebug>
#include <QStatusBar>
#include <QDir>
#include <QDateTime>
#include <QFileInfo>

void MainWindow::initTab1(QTabWidget* tabWidget) {
    QWidget *page = new QWidget();
    QHBoxLayout *mainTabLayout = new QHBoxLayout(page);
    mainTabLayout->setContentsMargins(Theme::MARGIN, Theme::MARGIN, Theme::MARGIN, Theme::MARGIN);
    mainTabLayout->setSpacing(Theme::SPACING);

    // === 左侧控制面板 ===
    QWidget *ctrlPanel = new QWidget();
    ctrlPanel->setFixedWidth(260);
    QVBoxLayout *ctrlLayout = new QVBoxLayout(ctrlPanel);
    ctrlLayout->setContentsMargins(0, 0, 0, 0);
    ctrlLayout->setSpacing(Theme::SPACING);

    // --- 相机开关 ---
    QGroupBox *grpCamera = new QGroupBox("相机控制");
    QVBoxLayout *camLayout = new QVBoxLayout(grpCamera);

    btnOpenLeftCam = new QPushButton("打开左相机");
    btnOpenLeftCam->setMinimumWidth(Theme::BTN_MIN_W);
    btnCaptureLeft = new QPushButton("拍摄左图");
    btnCaptureLeft->setEnabled(false);
    btnCaptureLeft->setMinimumWidth(Theme::BTN_MIN_W);

    btnOpenRightCam = new QPushButton("打开右相机");
    btnOpenRightCam->setMinimumWidth(Theme::BTN_MIN_W);
    btnCaptureRight = new QPushButton("拍摄右图");
    btnCaptureRight->setEnabled(false);
    btnCaptureRight->setMinimumWidth(Theme::BTN_MIN_W);

    camLayout->addWidget(btnOpenLeftCam);
    camLayout->addWidget(btnCaptureLeft);
    camLayout->addWidget(btnOpenRightCam);
    camLayout->addWidget(btnCaptureRight);
    ctrlLayout->addWidget(grpCamera);

    // --- 拍摄模式 ---
    QGroupBox *grpMode = new QGroupBox("拍摄模式");
    QVBoxLayout *modeLayout = new QVBoxLayout(grpMode);

    chkCalib = new QCheckBox("标定图");
    chkLaser = new QCheckBox("激光图");
    chkPlatform = new QCheckBox("平台图");
    chkCalib->setChecked(true);
    saveModeGroup = new QButtonGroup(this);
    saveModeGroup->setExclusive(true);
    saveModeGroup->addButton(chkCalib, 0);
    saveModeGroup->addButton(chkLaser, 1);
    saveModeGroup->addButton(chkPlatform, 2);
    modeLayout->addWidget(chkCalib);
    modeLayout->addWidget(chkLaser);
    modeLayout->addWidget(chkPlatform);

    btnCaptureBoth = new QPushButton("同时拍摄");
    btnCaptureBoth->setMinimumWidth(Theme::BTN_MIN_W);
    btnCaptureBoth->setStyleSheet(Theme::boldButton());
    modeLayout->addWidget(btnCaptureBoth);
    ctrlLayout->addWidget(grpMode);

    // --- 自动采集 ---
    QGroupBox *grpAuto = new QGroupBox("自动采集 (串口触发)");
    QVBoxLayout *autoLayout = new QVBoxLayout(grpAuto);
    autoLayout->setSpacing(Theme::SPACING);

    btnStartAutoCollect = new QPushButton("开始采集");
    btnPauseAutoCollect = new QPushButton("跳过/解救下位机");
    btnEndAutoCollect = new QPushButton("强制终止");
    btnPauseAutoCollect->setEnabled(false);
    btnEndAutoCollect->setEnabled(false);
    btnStartAutoCollect->setMinimumWidth(Theme::BTN_MIN_W);
    btnPauseAutoCollect->setMinimumWidth(Theme::BTN_MIN_W + 10);
    btnEndAutoCollect->setMinimumWidth(Theme::BTN_MIN_W);
    btnStartAutoCollect->setStyleSheet(Theme::successButton());
    btnPauseAutoCollect->setStyleSheet(Theme::warningButton());
    btnEndAutoCollect->setStyleSheet(Theme::dangerButton());

    autoLayout->addWidget(btnStartAutoCollect);
    autoLayout->addWidget(btnPauseAutoCollect);
    autoLayout->addWidget(btnEndAutoCollect);
    ctrlLayout->addWidget(grpAuto);
    ctrlLayout->addStretch();

    // === 右侧视频预览 ===
    QWidget *videoPanel = new QWidget();
    QVBoxLayout *videoLayout = new QVBoxLayout(videoPanel);
    videoLayout->setContentsMargins(0, 0, 0, 0);
    videoLayout->setSpacing(Theme::SPACING);

    auto createVideoPanel = [](const QString& title, VideoWidget*& vw) -> QWidget* {
        QWidget *w = new QWidget();
        QVBoxLayout *lay = new QVBoxLayout(w);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(2);
        QLabel *lbl = new QLabel(title);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setStyleSheet(Theme::sectionTitleStyle());
        vw = new VideoWidget();
        vw->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
        vw->setStyleSheet(Theme::videoBorderStyle());
        lay->addWidget(lbl);
        lay->addWidget(vw, 1);
        return w;
    };

    videoLayout->addWidget(createVideoPanel("左相机视图", videoLeft), 1);
    videoLayout->addWidget(createVideoPanel("右相机视图", videoRight), 1);

    // --- 组装 ---
    mainTabLayout->addWidget(ctrlPanel);
    mainTabLayout->addWidget(videoPanel, 1);
    tabWidget->addTab(page, "双目采集");

    // ==================== Tab 1 对象实例化与核心连接 ====================
    threadLeftCam = new CameraThread(0, this);
    threadRightCam = new CameraThread(2, this);

    m_serialManager = new SerialPortManager(this);
    m_autoCollectTimer = new QTimer(this);
    m_autoCollectTimer->setSingleShot(true); // 【核心】必须设为单次触发，用于防死锁超时

    m_autoCollectCount = 0;
    m_isAutoCollecting = false;
    m_isPaused = false;

    // 在构造函数中，紧邻 m_autoCollectTimer 初始化之后添加：
    m_stabilizeTimer = new QTimer(this);
    m_stabilizeTimer->setSingleShot(true);
    connect(m_stabilizeTimer, &QTimer::timeout, this, &MainWindow::onStabilizeTimeout);

    // 相机图像信号连接 (不直连UI，统一走Mat槽函数处理丢帧逻辑)
    connect(threadLeftCam, &CameraThread::matReady, this, &MainWindow::onUpdateLeftMat);
    connect(threadRightCam, &CameraThread::matReady, this, &MainWindow::onUpdateRightMat);

    // 基础按钮信号
    connect(btnOpenLeftCam, &QPushButton::clicked, this, &MainWindow::onOpenLeftCameraClicked);
    connect(btnCaptureLeft, &QPushButton::clicked, this, &MainWindow::onCaptureLeftClicked);
    connect(btnOpenRightCam, &QPushButton::clicked, this, &MainWindow::onOpenRightCameraClicked);
    connect(btnCaptureRight, &QPushButton::clicked, this, &MainWindow::onCaptureRightClicked);
    connect(btnCaptureBoth, &QPushButton::clicked, this, &MainWindow::onCaptureBothClicked);

    // 自动采集按钮信号
    connect(btnStartAutoCollect, &QPushButton::clicked, this, &MainWindow::onStartAutoCollect);
    connect(btnPauseAutoCollect, &QPushButton::clicked, this, &MainWindow::onPauseAutoCollect);
    connect(btnEndAutoCollect, &QPushButton::clicked, this, &MainWindow::onEndAutoCollect);

    connect(m_autoCollectTimer, &QTimer::timeout, this, &MainWindow::onCaptureTimeout);

    connect(m_serialManager, &SerialPortManager::dataReceived, this, &MainWindow::onSerialDataReceived);
}

// ==================== Tab 1 槽函数 ====================

void MainWindow::onOpenLeftCameraClicked() {
    if (threadLeftCam->isRunning()) {
        threadLeftCam->stop(); threadLeftCam->wait();
        btnOpenLeftCam->setText("打开左相机");
        btnCaptureLeft->setEnabled(false);
        m_statusCamera->setText("相机: 左✗ 右" + QString(threadRightCam->isRunning() ? "✓" : "✗"));
        m_statusCamera->setStyleSheet(QString("color: %1; padding: 0 12px;").arg(threadRightCam->isRunning() ? Theme::WARNING : Theme::DANGER));
    } else {
        threadLeftCam->start(); btnOpenLeftCam->setText("关闭左相机"); btnCaptureLeft->setEnabled(true);
        m_statusCamera->setText("相机: 左✓ 右" + QString(threadRightCam->isRunning() ? "✓" : "✗"));
        m_statusCamera->setStyleSheet(QString("color: %1; padding: 0 12px;").arg(threadRightCam->isRunning() ? Theme::SUCCESS : Theme::WARNING));
    }
}

void MainWindow::onOpenRightCameraClicked() {
    if (threadRightCam->isRunning()) {
        threadRightCam->stop(); threadRightCam->wait();
        btnOpenRightCam->setText("打开右相机");
        btnCaptureRight->setEnabled(false);
        m_statusCamera->setText("相机: 左" + QString(threadLeftCam->isRunning() ? "✓" : "✗") + " 右✗");
        m_statusCamera->setStyleSheet(QString("color: %1; padding: 0 12px;").arg(threadLeftCam->isRunning() ? Theme::WARNING : Theme::DANGER));
    } else {
        threadRightCam->start(); btnOpenRightCam->setText("关闭右相机");
        btnCaptureRight->setEnabled(true);
        m_statusCamera->setText("相机: 左" + QString(threadLeftCam->isRunning() ? "✓" : "✗") + " 右✓");
        m_statusCamera->setStyleSheet(QString("color: %1; padding: 0 12px;").arg(threadLeftCam->isRunning() ? Theme::SUCCESS : Theme::WARNING));
    }
}

void MainWindow::onUpdateLeftMat(const cv::Mat &mat)
{
    {
        QMutexLocker locker(&m_matMutex);
        latestFrameLeft = mat.clone();
    }

    if (tabWidget->currentIndex() == 0) {
        videoLeft->setFrame(mat2QImage(mat));
    }
}

void MainWindow::onUpdateRightMat(const cv::Mat &mat) {
    {
        QMutexLocker locker(&m_matMutex);
        latestFrameRight = mat.clone();
    }

    if (tabWidget->currentIndex() == 0) {
        videoRight->setFrame(mat2QImage(mat));
    }
}

void MainWindow::onStabilizeTimeout()
{
    if (!m_isAutoCollecting) {
        qDebug() << ">>> 稳定延时结束时自动采集已终止，忽略拍照";
        return;
    }

    qDebug() << ">>> 稳定延时结束，准备拍照保存（第" << (m_autoCollectCount + 1) << "次）";

    cv::Mat matL, matR;
    {
        QMutexLocker locker(&m_matMutex);
        matL = latestFrameLeft.clone();
        matR = latestFrameRight.clone();
    }

    // 停止超时定时器
    m_autoCollectTimer->stop();

    if (matL.empty() || matR.empty()) {
        qDebug() << "!!! 错误：延时后图像仍为空，强制发送0x03跳过";
        if (m_serialManager && m_serialManager->isOpen()) {
            m_serialManager->sendData("\x03");
        }
        return;
    }

    // 直接保存（内部会发送0x03给下位机）
    performAutoSave(0, matL, matR);
}

void MainWindow::onCaptureLeftClicked() {
    QString fileName;
    cv::Mat matCopy;
    // 锁内仅拷贝数据
    QMutexLocker locker(&m_matMutex);
    if (latestFrameLeft.empty()) return;
    fileName = QString("%1/left_%2.jpg").arg(dirLeftCalib).arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz"));
    matCopy = latestFrameLeft.clone();

    // 锁外执行 IO 和 UI
    if (cv::imwrite(fileName.toStdString(), matCopy)) {
        QMessageBox::information(this, "成功", "已保存左图");
    }
}

void MainWindow::onCaptureRightClicked() {
    QString fileName;
    cv::Mat matCopy;

    QMutexLocker locker(&m_matMutex);
    if (latestFrameRight.empty()) return;
    fileName = QString("%1/right_%2.jpg").arg(dirRightCalib).arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz"));
    matCopy = latestFrameRight.clone();

    // 锁外执行 IO 和 UI
    if (cv::imwrite(fileName.toStdString(), matCopy)) {
        QMessageBox::information(this, "成功", "已保存右图");
    }
}

void MainWindow::onCaptureBothClicked() {
    // 锁内仅拷贝最新帧数据
    QString currentLeftDir, currentRightDir, modeStr;
    cv::Mat matLeft, matRight;
    {
        QMutexLocker locker(&m_matMutex);

        if (chkPlatform->isChecked()) {
            currentLeftDir = dirLeftPlatform; currentRightDir = dirRightPlatform; modeStr = "平台图";
        } else if (chkLaser->isChecked()) {
            currentLeftDir = dirLeftLaser; currentRightDir = dirRightLaser; modeStr = "激光图";
        } else {
            currentLeftDir = dirLeftCalib; currentRightDir = dirRightCalib; modeStr = "标定图";
        }

        if (!latestFrameLeft.empty())
            matLeft = latestFrameLeft.clone();
        if (!latestFrameRight.empty())
            matRight = latestFrameRight.clone();
    }

    // 锁外执行文件IO
    QDir().mkpath(currentLeftDir);
    QDir().mkpath(currentRightDir);

    QString timestamp = QString::number(QDateTime::currentMSecsSinceEpoch());
    bool leftOk = false, rightOk = false;
    QString leftPath, rightPath;

    if (!matLeft.empty()) {
        leftPath = QString("%1/left_%2.jpg").arg(currentLeftDir).arg(timestamp);
        leftOk = cv::imwrite(leftPath.toStdString(), matLeft);
    }

    if (!matRight.empty()) {
        rightPath = QString("%1/right_%2.jpg").arg(currentRightDir).arg(timestamp);
        rightOk = cv::imwrite(rightPath.toStdString(), matRight);
    }

    if (leftOk && rightOk) {
        statusBar()->showMessage(QString("已保存%1: 左%2 | 右%3")
            .arg(modeStr)
            .arg(QFileInfo(leftPath).fileName())
            .arg(QFileInfo(rightPath).fileName()), 1000);
    } else {
        QString errMsg = QString("同步拍摄%1失败:\n").arg(modeStr);
        if (!leftOk) errMsg += "- 左相机图像为空或保存失败\n";
        if (!rightOk) errMsg += "- 右相机图像为空或保存失败\n";
        QMessageBox::warning(this, "拍摄失败", errMsg);
    }
}

void MainWindow::performAutoSave(int index, const cv::Mat& leftMat, const cv::Mat& rightMat)
{
    Q_UNUSED(index);

    QString currentDirLeft = m_cachedDirLeft;
    QString currentDirRight = m_cachedDirRight;
    QDir().mkpath(currentDirLeft);
    QDir().mkpath(currentDirRight);

    if (leftMat.empty() || rightMat.empty()) {
        qDebug() << "!!! 图像为空，直接发0x03跳过";
        if (m_serialManager && m_serialManager->isOpen()) {
            m_serialManager->sendData("\x03");
        }
        return;
    }

    // ✅✅✅ 核心：先发 0x03 救活下位机，哪怕后面的存盘卡住也不影响流程！
    m_autoCollectCount++;
    qDebug() << ">>> ✅ 完成第" << m_autoCollectCount << "/200次采集";

    if (m_serialManager && m_serialManager->isOpen()) {
        m_serialManager->sendData("\x03");
        qDebug() << ">>> 已发送完成信号 (0x03)";
    }

    // 存盘放最后，爱卡多久卡多久，下位机早就去转下一步了
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
    QString leftPath = currentDirLeft + "/left_" + timestamp + ".jpg";
    QString rightPath = currentDirRight + "/right_" + timestamp + ".jpg";

    cv::imwrite(leftPath.toStdString(), leftMat);
    cv::imwrite(rightPath.toStdString(), rightMat);
}

// ==================== 自动采集状态机补充 ====================

void MainWindow::onStartAutoCollect() {
    QStringList availablePorts;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        availablePorts << info.portName();
    }

    if (availablePorts.isEmpty()) {
        QMessageBox::warning(this, "错误", "未检测到任何串口设备！请检查硬件连接。");
        return;
    }

    QString portToOpen = availablePorts.first();
    if (!m_serialManager->openPort(portToOpen, 115200)) {
        QMessageBox::warning(this, "串口错误", QString("无法打开串口 %1！").arg(portToOpen));
        return;
    }

    if (!threadLeftCam->isRunning()) threadLeftCam->start();
    if (!threadRightCam->isRunning()) threadRightCam->start();

    m_cachedDirLeft = dirLeftPointCloud;
    m_cachedDirRight = dirRightPointCloud;

    m_isAutoCollecting = true;
    m_isWaitingForCapture = false;
    m_autoCollectCount = 0; // 重置计数器

    // 界面状态锁定
    btnStartAutoCollect->setEnabled(false);
    btnPauseAutoCollect->setEnabled(true);
    btnEndAutoCollect->setEnabled(true);
    btnOpenLeftCam->setEnabled(false); btnOpenRightCam->setEnabled(false);
    btnCaptureLeft->setEnabled(false); btnCaptureRight->setEnabled(false);
    btnCaptureBoth->setEnabled(false);
    chkCalib->setEnabled(false); chkLaser->setEnabled(false); chkPlatform->setEnabled(false);

    m_serialManager->sendData("\x01"); // 唤醒下位机
    qDebug() << ">>> 已发送启动信号 (0x01)";
}

void MainWindow::onPauseAutoCollect() {
    // 手动解救按钮，发0x03打破下位机的while死等
    if (m_isWaitingForCapture) {
        m_autoCollectTimer->stop();
        m_isWaitingForCapture = false;
        m_serialManager->sendData("\x03");
        qDebug() << ">>> 用户手动干预，发送跳过信号 (0x03)";
    }
}

void MainWindow::onEndAutoCollect() {
    m_autoCollectTimer->stop();
    m_stabilizeTimer->stop();
    m_isAutoCollecting = false;
    m_isWaitingForCapture = false;

    // 如果下位机正在死等0x03，必须发一个救活它
    if (m_serialManager && m_serialManager->isOpen()) {
        m_serialManager->sendData("\x03");
    }

    // 恢复界面
    btnStartAutoCollect->setEnabled(true);
    btnPauseAutoCollect->setEnabled(false);
    btnEndAutoCollect->setEnabled(false);
    btnOpenLeftCam->setEnabled(true); btnOpenRightCam->setEnabled(true);
    btnCaptureLeft->setEnabled(threadLeftCam->isRunning());
    btnCaptureRight->setEnabled(threadRightCam->isRunning());
    btnCaptureBoth->setEnabled(true);
    chkCalib->setEnabled(true); chkLaser->setEnabled(true); chkPlatform->setEnabled(true);
}

// 保留空实现防止编译报错
void MainWindow::onAutoCollectStep() {}

// ==================== 新增的状态机槽函数实现 ====================
void MainWindow::onSerialDataReceived(const QByteArray &data)
{
    for (int i = 0; i < data.size(); i++) {
        uint8_t cmd = static_cast<uint8_t>(data[i]);

        // ---------- 0x04：采集结束 ----------
        if (cmd == 0x04) {
            qDebug() << ">>> 收到结束信号 (0x04)，总计采集" << m_autoCollectCount << "次";
            onEndAutoCollect();
            QMessageBox::information(this, "完成", "200次点云自动采集已完成！");
        }
        // ---------- 0x02：触发拍照 ----------
        else if (cmd == 0x02) {
            if (m_isAutoCollecting) {
                qDebug() << ">>> 收到第" << (m_autoCollectCount + 1) << "/200次触发 (0x02)，启动3000ms稳定延时...";
                m_captureStartTimeMs = QDateTime::currentMSecsSinceEpoch();  // 记录时间戳（可选，用于调试）
                m_stabilizeTimer->start(3000);   // 稳定延时 3000ms 后拍照
                m_autoCollectTimer->start(5000); // 5 秒超时保护，防止下位机死等
            } else {
                qDebug() << ">>> 收到 0x02 但未处于自动采集状态，忽略";
            }
        }
        // ---------- 其他命令可在此扩展 ----------
        else {
            qDebug() << ">>> 收到未知命令:" << QString::number(cmd, 16);
        }
    }
}

void MainWindow::onCaptureTimeout()
{
    if (m_isAutoCollecting) {
        qDebug() << "!!! ⚠️ 拍照流程超时（5秒），强制发送0x03解救下位机！";
        m_stabilizeTimer->stop();   // 停止稳定延时，避免重复拍照
        if (m_serialManager && m_serialManager->isOpen()) {
            m_serialManager->sendData("\x03");
        }
    }
}
