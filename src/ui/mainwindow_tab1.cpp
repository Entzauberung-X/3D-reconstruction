// ==================== Tab1: 双目采集 ====================
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QCheckBox>
#include <QButtonGroup>
#include <QAbstractButton>
#include <QSpinBox>
#include <QProgressBar>
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
    chkWorkpiece = new QCheckBox("工件图");
    chkCalib->setChecked(true);
    // 这一组同时决定「同时拍摄」和「自动采集」存到哪个序列目录
    chkPlatform->setToolTip("存入 Resources/Left_Platform、Right_Platform\n用于 Tab4 转台(旋转轴)标定序列");
    chkWorkpiece->setToolTip("存入 Resources/Left_PointCloud、Right_PointCloud\n用于 Tab5 三维重建序列");
    saveModeGroup = new QButtonGroup(this);
    saveModeGroup->setExclusive(true);
    saveModeGroup->addButton(chkCalib, 0);
    saveModeGroup->addButton(chkLaser, 1);
    saveModeGroup->addButton(chkPlatform, 2);
    saveModeGroup->addButton(chkWorkpiece, 3);
    modeLayout->addWidget(chkCalib);
    modeLayout->addWidget(chkLaser);
    modeLayout->addWidget(chkPlatform);
    modeLayout->addWidget(chkWorkpiece);

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

    // 多采余量: 目标之外额外允许的组数 (下位机多转几格也不丢图/进度条不"溢出")
    QHBoxLayout *extraRow = new QHBoxLayout();
    extraRow->setSpacing(Theme::SPACING);
    QLabel *lblExtra = new QLabel("多采余量:");
    spinExtraShots = new QSpinBox();
    spinExtraShots->setRange(0, 100);
    spinExtraShots->setValue(10);
    spinExtraShots->setSuffix(" 组");
    spinExtraShots->setToolTip(
        "在目标组数之外额外允许多采的组数。\n"
        "下位机偶尔会多转一格或多发一次触发, 这些图本来就会存下来,\n"
        "此值只决定进度条量程, 不影响何时结束 (结束仍由下位机 0x04 决定)。");
    extraRow->addWidget(lblExtra);
    extraRow->addWidget(spinExtraShots, 1);
    autoLayout->addLayout(extraRow);

    // 采集进度条 (样式与 Tab5 重建进度条一致, 量程 = 目标 + 多采余量)
    progressAutoCollect = new QProgressBar();
    progressAutoCollect->setRange(0, autoCollectTarget());
    progressAutoCollect->setValue(0);
    progressAutoCollect->setFormat(QString("0 / %1 组").arg(autoCollectTarget()));
    progressAutoCollect->setToolTip("自动采集进度: 每次触发保存左+右各一张为 1 组");
    autoLayout->addWidget(progressAutoCollect);

    lblAutoCollectStatus = new QLabel();
    lblAutoCollectStatus->setStyleSheet(Theme::metricsStyle());
    lblAutoCollectStatus->setWordWrap(true);
    autoLayout->addWidget(lblAutoCollectStatus);

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
    addPage(tabWidget, page, "01 硬件采集", "01", "硬件与采集",
            "相机连接 / 实时预览 / 手动与自动采集 / 串口触发 —— 把标定与重建需要的数据拍下来");

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

    // 等待相机首帧的超时: 到点还没画面就不唤醒下位机 (见 onStartAutoCollect)
    m_kickoffTimer = new QTimer(this);
    m_kickoffTimer->setSingleShot(true);
    connect(m_kickoffTimer, &QTimer::timeout, this, &MainWindow::onKickoffTimeout);

    // 相机图像信号连接 (不直连UI，统一走Mat槽函数处理丢帧逻辑)
    connect(threadLeftCam, &CameraThread::matReady, this, &MainWindow::onUpdateLeftMat);
    connect(threadRightCam, &CameraThread::matReady, this, &MainWindow::onUpdateRightMat);
    // 相机打开失败/读取中断时上报 UI（跨线程，自动 Queued）
    connect(threadLeftCam, &CameraThread::cameraFailed, this, &MainWindow::onCameraFailed);
    connect(threadRightCam, &CameraThread::cameraFailed, this, &MainWindow::onCameraFailed);

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

    // 拍摄模式切换: 刷新"自动采集会存到哪个序列目录"的提示
    // (目录已在 initDirectories() 中初始化)
    auto refreshAutoTargetHint = [this]() {
        QString l, r, name;
        currentSaveDirs(l, r, name);
        lblAutoCollectStatus->setText(QString("目标: %1\n左: %2\n右: %3").arg(name).arg(l).arg(r));
    };
    connect(saveModeGroup, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked),
            this, [refreshAutoTargetHint](QAbstractButton*) { refreshAutoTargetHint(); });
    refreshAutoTargetHint();

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
        threadLeftCam->startStreaming(); btnOpenLeftCam->setText("关闭左相机"); btnCaptureLeft->setEnabled(true);
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
        threadRightCam->startStreaming(); btnOpenRightCam->setText("关闭右相机");
        btnCaptureRight->setEnabled(true);
        m_statusCamera->setText("相机: 左" + QString(threadLeftCam->isRunning() ? "✓" : "✗") + " 右✓");
        m_statusCamera->setStyleSheet(QString("color: %1; padding: 0 12px;").arg(threadLeftCam->isRunning() ? Theme::SUCCESS : Theme::WARNING));
    }
}

void MainWindow::onUpdateLeftMat(const cv::Mat &mat)
{
    // 放开取流线程的背压闸门(必须在拷贝之前: 拷贝这段时间里来的新帧才不会被卡住)
    threadLeftCam->frameConsumed();

    {
        QMutexLocker locker(&m_matMutex);
        latestFrameLeft = mat.clone();
        ++m_leftFrameSeq;
    }

    if (tabWidget->currentIndex() == 0) {
        videoLeft->setFrame(mat2QImage(mat));
    }

    // 等的就是这一帧: 两台都出图了才唤醒下位机 (见 onStartAutoCollect)
    if (m_pendingKickoff && framePairReady()) kickoffSerial();
}

void MainWindow::onUpdateRightMat(const cv::Mat &mat) {
    threadRightCam->frameConsumed();

    {
        QMutexLocker locker(&m_matMutex);
        latestFrameRight = mat.clone();
        ++m_rightFrameSeq;
    }

    if (tabWidget->currentIndex() == 0) {
        videoRight->setFrame(mat2QImage(mat));
    }

    if (m_pendingKickoff && framePairReady()) kickoffSerial();
}

void MainWindow::onCameraFailed(int cameraIndex, const QString &reason)
{
    statusBar()->showMessage(QString("相机 %1 错误: %2").arg(cameraIndex).arg(reason), 5000);

    // 复位对应相机的按钮状态（线程已结束，不能再依赖 isRunning()）
    if (cameraIndex == 0) {
        btnOpenLeftCam->setText("打开左相机");
        btnCaptureLeft->setEnabled(false);
    } else if (cameraIndex == 2) {
        btnOpenRightCam->setText("打开右相机");
        btnCaptureRight->setEnabled(false);
    }

    QString status = "相机: 左" + QString(threadLeftCam && threadLeftCam->isRunning() ? "✓" : "✗")
                   + " 右" + QString(threadRightCam && threadRightCam->isRunning() ? "✓" : "✗");
    m_statusCamera->setText(status);
    m_statusCamera->setStyleSheet(QString("color: %1; padding: 0 12px;").arg(Theme::DANGER));

    // 采集途中掉一台相机 = 后面每一格都取不到新画面, 不如当场停下。
    // (线程 read() 失败即退出, 不会自愈; 继续跑只会让下位机白转一整轮。)
    if (m_isAutoCollecting) {
        QMessageBox::warning(this, "相机中断",
            QString("相机 %1 采集途中中断:\n%2\n\n已终止本次自动采集。").arg(cameraIndex).arg(reason));
        onEndAutoCollect();
    }
}

void MainWindow::onStabilizeTimeout()
{
    // 采集已终止 / 窗口已经收尾: 丢弃本次延时回调 (不再打印, 避免刷屏)
    if (!m_isAutoCollecting || !m_isWaitingForCapture) return;

    m_autoCollectTimer->stop();   // 正常路径, 撤回死线

    cv::Mat matL, matR;
    quint64 seqL = 0, seqR = 0;
    {
        QMutexLocker locker(&m_matMutex);
        matL = latestFrameLeft.clone();
        matR = latestFrameRight.clone();
        seqL = m_leftFrameSeq;
        seqR = m_rightFrameSeq;
    }

    // 稳定延时到了, 但这段时间里两台相机必须各自至少出过一帧新画面。
    // 旧版只判"图像非空"——而 latestFrame 一旦有值就永不清空, 相机掉线/线程退出后
    // 它一直是非空的: 于是接下来每一格都把**同一张旧图**当新组存一遍, 计数一路满,
    // 序列却全是重复帧。判新帧才能发现"这一格其实没采到"。
    const bool fresh = (seqL > m_leftSeqAtTrigger) && (seqR > m_rightSeqAtTrigger);

    if (matL.empty() || matR.empty() || !fresh) {
        qDebug() << "[采集] 本组丢弃: 稳定延时内没有新画面 (左 +"
                 << (seqL - m_leftSeqAtTrigger) << "帧 / 右 +"
                 << (seqR - m_rightSeqAtTrigger) << "帧)";
        if (m_serialManager && m_serialManager->isOpen()) {
            m_serialManager->sendData("\x03");
        }
        closeCaptureWindow(false);   // 计入"跳过", 进度条仍然前进, 不会静默少组
        return;
    }

    // 直接保存（内部会发送0x03给下位机）
    performAutoSave(0, matL, matR);
}

void MainWindow::onCaptureLeftClicked() {
    cv::Mat matCopy;
    {
        // 锁内仅拷贝数据
        QMutexLocker locker(&m_matMutex);
        if (latestFrameLeft.empty()) return;
        matCopy = latestFrameLeft.clone();
    }
    // 锁外执行 IO 和 UI (避免 JPEG 编码阻塞相机线程的 onUpdateLeftMat)
    QString fileName = QString("%1/left_%2.jpg").arg(dirLeftCalib)
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz"));
    if (cv::imwrite(fileName.toStdString(), matCopy)) {
        QMessageBox::information(this, "成功", "已保存左图");
    }
}

void MainWindow::onCaptureRightClicked() {
    cv::Mat matCopy;

    {
        QMutexLocker locker(&m_matMutex);
        if (latestFrameRight.empty()) return;
        matCopy = latestFrameRight.clone();
    }

    // 锁外执行 IO 和 UI (避免 JPEG 编码阻塞相机线程的 onUpdateRightMat)
    QString fileName = QString("%1/right_%2.jpg").arg(dirRightCalib)
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz"));
    if (cv::imwrite(fileName.toStdString(), matCopy)) {
        QMessageBox::information(this, "成功", "已保存右图");
    }
}

// 「拍摄模式」→ 存档目录的唯一路由。手动拍摄与自动采集都走这里, 避免两处各写一份 if/else
void MainWindow::currentSaveDirs(QString& leftDir, QString& rightDir, QString& modeName) const
{
    if (chkPlatform->isChecked()) {
        leftDir = dirLeftPlatform;  rightDir = dirRightPlatform;  modeName = "平台图";
    } else if (chkWorkpiece->isChecked()) {
        leftDir = dirLeftPointCloud; rightDir = dirRightPointCloud; modeName = "工件图";
    } else if (chkLaser->isChecked()) {
        leftDir = dirLeftLaser;     rightDir = dirRightLaser;     modeName = "激光图";
    } else {
        leftDir = dirLeftCalib;     rightDir = dirRightCalib;     modeName = "标定图";
    }
}

void MainWindow::onCaptureBothClicked() {
    // 目录解析不碰共享帧数据, 放在锁外
    QString currentLeftDir, currentRightDir, modeStr;
    currentSaveDirs(currentLeftDir, currentRightDir, modeStr);

    cv::Mat matLeft, matRight;
    {
        QMutexLocker locker(&m_matMutex);
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

    // ✅✅✅ 核心：先发 0x03 救活下位机，哪怕后面的存盘卡住也不影响流程！
    if (m_serialManager && m_serialManager->isOpen()) {
        m_serialManager->sendData("\x03");
    }

    bool saved = false;
    if (leftMat.empty() || rightMat.empty()) {
        qDebug() << "[采集] 图像为空, 本组跳过";
    } else {
        QDir().mkpath(currentDirLeft);
        QDir().mkpath(currentDirRight);

        // 存盘放最后，爱卡多久卡多久，下位机早就去转下一步了
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        QString leftPath  = currentDirLeft  + "/left_"  + timestamp + ".jpg";
        QString rightPath = currentDirRight + "/right_" + timestamp + ".jpg";

        // 写盘结果必须看: 磁盘满/目录无权限时 imwrite 只是返回 false 不抛异常,
        // 旧版照样 ++计数, 于是"采到 200 组"而目录里 0 张。
        const bool leftOk  = cv::imwrite(leftPath.toStdString(),  leftMat);
        const bool rightOk = cv::imwrite(rightPath.toStdString(), rightMat);
        saved = leftOk && rightOk;
        if (!saved) {
            qDebug() << "[采集] 存盘失败 (左" << leftOk << "右" << rightOk << "):" << leftPath;
        }
    }

    closeCaptureWindow(saved);
}

// ==================== 自动采集状态机补充 ====================

bool MainWindow::framePairReady()
{
    QMutexLocker locker(&m_matMutex);
    return !latestFrameLeft.empty() && !latestFrameRight.empty();
}

void MainWindow::kickoffSerial()
{
    m_pendingKickoff = false;
    if (m_kickoffTimer) m_kickoffTimer->stop();
    if (m_serialManager && m_serialManager->isOpen()) {
        m_serialManager->sendData("\x01");   // 唤醒下位机: 可以开始转第一格了
    }
    qDebug() << "[采集] 相机已出图, 唤醒下位机 (0x01)";
}

void MainWindow::startCaptureWindow(int settleMs)
{
    m_isWaitingForCapture = true;
    m_leftSeqAtTrigger  = m_leftFrameSeq;
    m_rightSeqAtTrigger = m_rightFrameSeq;
    if (settleMs <= 0) settleMs = 1;   // start(0) 也能触发, 但留 1ms 语义更清楚
    m_stabilizeTimer->start(settleMs);                 // 稳定延时后存盘
    m_autoCollectTimer->start(AUTO_WINDOW_DEADMAN_MS); // 死线: 万一稳定回调没来也不让下位机干等
}

void MainWindow::closeCaptureWindow(bool saved)
{
    m_isWaitingForCapture = false;
    if (saved) ++m_autoCollectCount; else ++m_autoCollectSkipped;
    updateAutoCollectProgress();

    if (!m_isAutoCollecting) return;

    // 窗口期间又来过触发: 补一个窗口。不补的话下位机转过的那些格就等于丢了 ——
    // 这正是"采不满目标组数"的另一半原因。
    //
    // 补采只用很短的稳定延时: 那个触发早就到了(排在窗口里等), 转台从那时起就没再动过
    // —— 它要动也得等刚才那记 0x03。拖满 3 秒再拍的话, 转台已经走到下一格,
    // 这一组的画面就串位了。
    if (m_pendingTriggers > 0) {
        --m_pendingTriggers;
        startCaptureWindow(AUTO_QUEUED_SETTLE_MS);
    }
}

void MainWindow::updateAutoCollectProgress()
{
    const int consumed = m_autoCollectCount + m_autoCollectSkipped;
    if (progressAutoCollect) {
        progressAutoCollect->setRange(0, autoCollectTarget());
        progressAutoCollect->setValue(qMin(consumed, autoCollectTarget()));
        progressAutoCollect->setFormat(QString("%1 / %2 组   跳过 %3")
            .arg(m_autoCollectCount).arg(autoCollectTarget()).arg(m_autoCollectSkipped));
    }
    if (lblAutoCollectStatus) {
        QString l, r, name;
        currentSaveDirs(l, r, name);   // 采集中勾选被锁, 取到的就是本次目标
        lblAutoCollectStatus->setText(QString("目标: %1\n成功 %2 | 跳过 %3 | 重复触发 %4\n左: %5\n右: %6")
            .arg(name).arg(m_autoCollectCount).arg(m_autoCollectSkipped)
            .arg(m_autoDupTriggers).arg(m_cachedDirLeft).arg(m_cachedDirRight));
    }
}

void MainWindow::onStartAutoCollect() {
    // 工具栏那个「开始采集」不受按钮 enable 约束, 采集途中还能点 —— 再走一遍会重开串口、
    // 清零计数、又发一记 0x01, 下位机等于被唤醒第二次, 组数直接对不上。拦住。
    if (m_isAutoCollecting) {
        qDebug() << "[采集] 已在采集中, 忽略重复启动";
        return;
    }

    // 自动采集沿用「拍摄模式」的勾选, 但只有平台图/工件图两个目标说得通。
    // 若停在标定图/激光图就开跑, 会把整段序列倒进标定目录, 冲掉已有标定图并让重建序列为空 —— 拦下。
    if (chkCalib->isChecked() || chkLaser->isChecked()) {
        QMessageBox::warning(this, "采集目标不对",
            "自动采集只能导入「平台图」(转台标定序列) 或「工件图」(三维重建序列)。\n"
            "请在「拍摄模式」中改选其中之一再开始。");
        return;
    }

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

    // 用 startStreaming() 而不是 QThread::start(): 前者会复位 running,
    // 否则"关过相机再打开"得到的线程只是 open() 到设备就退出, 一帧都不出。
    threadLeftCam->startStreaming();
    threadRightCam->startStreaming();

    // 采集目标沿用「拍摄模式」当前勾选 (平台图 → 转台序列目录; 工件图 → 重建序列目录)
    QString targetName;
    currentSaveDirs(m_cachedDirLeft, m_cachedDirRight, targetName);
    QDir().mkpath(m_cachedDirLeft);
    QDir().mkpath(m_cachedDirRight);

    m_isAutoCollecting = true;
    m_isWaitingForCapture = false;
    m_pendingTriggers = 0;
    m_autoCollectCount = 0;   // 重置计数器
    m_autoCollectSkipped = 0;
    m_autoDupTriggers = 0;

    // 进度条复位
    progressAutoCollect->setRange(0, autoCollectTarget());
    progressAutoCollect->setValue(0);
    progressAutoCollect->setFormat(QString("0 / %1 组").arg(autoCollectTarget()));
    updateAutoCollectProgress();

    // 界面状态锁定
    btnStartAutoCollect->setEnabled(false);
    btnPauseAutoCollect->setEnabled(true);
    btnEndAutoCollect->setEnabled(true);
    btnOpenLeftCam->setEnabled(false); btnOpenRightCam->setEnabled(false);
    btnCaptureLeft->setEnabled(false); btnCaptureRight->setEnabled(false);
    btnCaptureBoth->setEnabled(false);
    // 采集中不允许切换目标
    chkCalib->setEnabled(false); chkLaser->setEnabled(false);
    chkPlatform->setEnabled(false); chkWorkpiece->setEnabled(false);

    // 唤醒下位机之前先确认两台相机真的在出图。
    // 旧版无条件 sendData(0x01): 相机没画面时下位机照样转完一整轮, 我们每格都回 0x03
    // 却一张都没存下 —— 界面上就是"进度条不动 / 一组也采不到"。
    if (framePairReady()) {
        kickoffSerial();
    } else {
        m_pendingKickoff = true;
        m_kickoffTimer->start(KICKOFF_TIMEOUT_MS);
        qDebug() << "[采集] 尚未收到相机画面, 暂不唤醒下位机";
    }
    qDebug() << "[采集] 自动采集启动 →" << targetName << "目录:" << m_cachedDirLeft;
}

void MainWindow::onKickoffTimeout()
{
    if (!m_pendingKickoff) return;
    m_pendingKickoff = false;
    qDebug() << "[采集] 等待相机首帧超时, 未唤醒下位机, 终止";
    QMessageBox::warning(this, "相机无画面",
        QString("等待 %1 秒仍没有收到相机画面, 已取消本次自动采集。\n"
                "请检查相机连接/预览后再试。").arg(KICKOFF_TIMEOUT_MS / 1000));
    onEndAutoCollect();
}

void MainWindow::onPauseAutoCollect() {
    if (!m_isAutoCollecting) return;

    // 只在窗口进行中(下位机正死等 0x03)才发 0x03 —— 平时乱发会让下位机多转一格。
    if (!m_isWaitingForCapture) {
        qDebug() << "[采集] 当前没有等待中的触发, 忽略跳过";
        return;
    }

    // 手动解救: 0x03 打破下位机的 while 死等。
    // 稳定定时器**必须一起停**, 否则 3 秒后它照样存一组图 —— 那组是下一格的画面,
    // 计数和角度都对不上。
    m_stabilizeTimer->stop();
    m_autoCollectTimer->stop();
    if (m_serialManager && m_serialManager->isOpen()) {
        m_serialManager->sendData("\x03");
    }
    qDebug() << "[采集] 手动跳过当前位, 发送 0x03";
    closeCaptureWindow(false);
}

void MainWindow::onEndAutoCollect() {
    const bool wasCollecting = m_isAutoCollecting;

    // 先落旗: 下面收尾时 closeCaptureWindow 才不会又去开下一个排队窗口
    m_isAutoCollecting = false;
    m_stabilizeTimer->stop();
    m_autoCollectTimer->stop();
    m_pendingKickoff = false;
    if (m_kickoffTimer) m_kickoffTimer->stop();

    // 窗口还没收尾就收到结束信号/被强制终止: 先把手上这组存下来。
    // 旧版直接 stop() 稳定定时器, 最后一格永远丢掉 —— 24 个位置只落 23 组,
    // 看起来就是"采不到目标组数"。
    bool acked = false;   // 下面这段自己会回 0x03, 别让末尾再补一记(两记 0x03 下位机转两格)
    if (wasCollecting && m_isWaitingForCapture) {
        cv::Mat matL, matR;
        quint64 seqL = 0, seqR = 0;
        {
            QMutexLocker locker(&m_matMutex);
            matL = latestFrameLeft.clone();
            matR = latestFrameRight.clone();
            seqL = m_leftFrameSeq;
            seqR = m_rightFrameSeq;
        }
        const bool fresh = (seqL > m_leftSeqAtTrigger) && (seqR > m_rightSeqAtTrigger);
        if (!matL.empty() && !matR.empty() && fresh) {
            performAutoSave(0, matL, matR);   // 内部发 0x03 并 closeCaptureWindow
        } else {
            if (m_serialManager && m_serialManager->isOpen()) {
                m_serialManager->sendData("\x03");
            }
            closeCaptureWindow(false);
        }
        acked = true;
    }

    m_isWaitingForCapture = false;
    m_pendingTriggers = 0;

    // 如果下位机正在死等0x03，必须发一个救活它。
    // 本来就没在采集时不发: 空闲的下位机收到 0x03 只会多转一格。
    if (!acked && wasCollecting && m_serialManager && m_serialManager->isOpen()) {
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
    chkCalib->setEnabled(true); chkLaser->setEnabled(true);
    chkPlatform->setEnabled(true); chkWorkpiece->setEnabled(true);

    // 进度条收尾 + 状态提示 (数字全留着: 少了几组要能一眼看出少在哪)
    if (progressAutoCollect) {
        progressAutoCollect->setFormat(QString("%1 / %2 组   跳过 %3")
            .arg(m_autoCollectCount).arg(autoCollectTarget()).arg(m_autoCollectSkipped));
    }
    if (lblAutoCollectStatus) {
        lblAutoCollectStatus->setText(QString("已结束: 成功 %1 / 跳过 %2 / 重复触发 %3")
            .arg(m_autoCollectCount).arg(m_autoCollectSkipped).arg(m_autoDupTriggers));
    }
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
            qDebug() << "[采集] 收到结束信号 (0x04): 成功" << m_autoCollectCount
                     << "跳过" << m_autoCollectSkipped << "重复触发" << m_autoDupTriggers;
            onEndAutoCollect();   // 内部会把还没落盘的最后一组补存下来
            QMessageBox::information(this, "完成",
                QString("自动采集已完成。\n成功 %1 组 / 跳过 %2 组 / 重复触发 %3 次\n(每组 = 左 + 右各一张)")
                    .arg(m_autoCollectCount).arg(m_autoCollectSkipped).arg(m_autoDupTriggers));
        }
        // ---------- 0x02：触发拍照 ----------
        else if (cmd == 0x02) {
            if (!m_isAutoCollecting) {
                qDebug() << "[采集] 收到 0x02 但未处于采集状态, 忽略";
            } else if (m_isWaitingForCapture) {
                // 窗口进行中又来的触发: 下位机多发了一次/多转了一格。
                // 绝不能拿它去重启本窗口的定时器 —— 3 秒的稳定延时会被一路后推,
                // 触发间隔一小于 3s 这个窗口就永远不落盘(死线也一起被推)。
                // 正确做法是排队, 等本窗口收尾时补采一组。
                ++m_pendingTriggers;
                ++m_autoDupTriggers;
                qDebug() << "[采集] 窗口进行中的触发, 排队等补采: 第" << m_pendingTriggers << "个";
            } else {
                startCaptureWindow();
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
    if (!m_isAutoCollecting || !m_isWaitingForCapture) return;

    qDebug() << "[采集] ⚠️ 窗口死线到, 本组按跳过处理, 发送 0x03 解救下位机";
    m_stabilizeTimer->stop();   // 停止稳定延时，避免重复拍照
    if (m_serialManager && m_serialManager->isOpen()) {
        m_serialManager->sendData("\x03");
    }
    closeCaptureWindow(false);
}
