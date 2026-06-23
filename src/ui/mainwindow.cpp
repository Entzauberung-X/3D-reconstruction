#include "ui/mainwindow.h"
#include "ui/logger.h"
#include "ui/theme.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QDialog>
#include <QDir>
#include <QDateTime>
#include <QStatusBar>
#include <QMenuBar>
#include <QToolBar>
#include <QAction>
#include <QSettings>
#include <QCloseEvent>
#include <QFuture>
#include <QScrollArea>
#include <QHeaderView>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QClipboard>
#include <numeric>
#include <vector>
#include <algorithm>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QPainter>
#include <QScrollArea>
#include <functional>
#include <iomanip>
#include <QtConcurrent/QtConcurrent>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <thread>
#include <sstream>
#include <fstream>

// ==================== 构造函数 ====================
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), captureCount(0), m_IsRectified(false)
{
    setWindowTitle("双目线激光转台三维重建系统");
    resize(1400, 900);

    // ── 菜单栏 ──
    QMenuBar *mb = menuBar();
    QMenu *fileMenu = mb->addMenu("文件(&F)");
    fileMenu->addAction("保存全部标定", this, &MainWindow::onSaveCalibrationClicked);
    fileMenu->addAction("加载标定...", this, &MainWindow::onLoadCalibrationClicked);
    fileMenu->addSeparator();
    fileMenu->addAction("退出(&Q)", this, &QWidget::close);

    QMenu *viewMenu = mb->addMenu("视图(&V)");
    viewMenu->addAction("弹出3D视图", this, &MainWindow::onOpenStandaloneViewer);

    QMenu *calibMenu = mb->addMenu("标定(&C)");
    calibMenu->addAction("自动加载图片", this, &MainWindow::onAutoLoadAllImages);
    calibMenu->addSeparator();
    calibMenu->addAction("相机标定", this, &MainWindow::onCalibrateClicked);
    calibMenu->addAction("立体标定", this, &MainWindow::onCalculateStereoParams);
    calibMenu->addAction("光平面标定", this, &MainWindow::onToolbarLaserCalib);
    calibMenu->addAction("转台标定", this, &MainWindow::onExecAxisCalibClicked);

    QMenu *toolsMenu = mb->addMenu("工具(&T)");
    toolsMenu->addAction("棋盘格3D验证", this, &MainWindow::onVerifyChessboard3D);
    toolsMenu->addAction("多帧棋盘格3D重建", this, &MainWindow::onMultiFrameChessboard3D);
    toolsMenu->addAction("逐帧调试信息", this, &MainWindow::onDebugAxisCalibClicked);
    toolsMenu->addSeparator();
    toolsMenu->addAction("开始重建", this, &MainWindow::onStartReconstructionClicked);

    QMenu *settingsMenu = mb->addMenu("设置(&S)");
    settingsMenu->addAction("标定板参数...", this, [this]() {
        QDialog dlg(this);
        dlg.setWindowTitle("标定板参数设置");
        dlg.setFixedSize(320, 200);
        QVBoxLayout *lay = new QVBoxLayout(&dlg);
        lay->setSpacing(10);

        QFormLayout *form = new QFormLayout();
        QSpinBox *spinCols = new QSpinBox(); spinCols->setRange(3, 20); spinCols->setValue(m_boardSize.width);
        QSpinBox *spinRows = new QSpinBox(); spinRows->setRange(3, 20); spinRows->setValue(m_boardSize.height);
        QDoubleSpinBox *spinSize = new QDoubleSpinBox(); spinSize->setRange(1.0, 100.0); spinSize->setValue(m_squareSize); spinSize->setSingleStep(1.0);
        form->addRow("角点列数:", spinCols);
        form->addRow("角点行数:", spinRows);
        form->addRow("方格大小(mm):", spinSize);
        lay->addLayout(form);

        QHBoxLayout *btnLayout = new QHBoxLayout();
        QPushButton *btnOk = new QPushButton("确定");
        QPushButton *btnCancel = new QPushButton("取消");
        btnOk->setStyleSheet(Theme::successButton());
        btnCancel->setStyleSheet(QString("background: %1; color: %2; padding: 6px 16px; border-radius: 3px;").arg(Theme::BG_HOVER, Theme::TEXT_PRIMARY));
        btnLayout->addStretch();
        btnLayout->addWidget(btnOk);
        btnLayout->addWidget(btnCancel);
        lay->addLayout(btnLayout);

        connect(btnOk, &QPushButton::clicked, &dlg, &QDialog::accept);
        connect(btnCancel, &QPushButton::clicked, &dlg, &QDialog::reject);

        if (dlg.exec() == QDialog::Accepted) {
            m_boardSize.width = spinCols->value();
            m_boardSize.height = spinRows->value();
            m_squareSize = (float)spinSize->value();
            Logger::info(QString("标定板参数已更新: %1x%2, %3mm")
                .arg(m_boardSize.width).arg(m_boardSize.height).arg(m_squareSize));
        }
    });

    QMenu *helpMenu = mb->addMenu("帮助(&H)");
    helpMenu->addAction("工作流程", this, [this]() {
        QMessageBox::information(this, "推荐工作流程",
            "① 自动加载图片 — 加载全部默认路径图像\n"
            "② 相机标定 — 单目标定左右相机内参\n"
            "③ 立体标定 — 双目标定外参R/T\n"
            "④ 光平面标定 — 激光线检测→光平面拟合\n"
            "⑤ 转台标定 — 旋转轴自动标定\n"
            "⑥ 开始重建 — S1-S6三维重建流水线\n\n"
            "提示: 第①步可一键加载全部图片，后续步骤无需手动选择文件。\n"
            "状态栏底部实时显示各步骤标定状态和误差值。");
    });
    helpMenu->addSeparator();
    helpMenu->addAction("关于", this, [this]() {
        QMessageBox::about(this, "关于",
            "双目线激光转台三维重建系统 v1.0\n\n"
            "基于 Qt5 + OpenCV + PCL + Eigen3\n"
            "S1-S6 流水线: 光条提取→极线匹配→三角化→坐标变换→多视角配准→网格重建");
    });

    // ── 工具栏 ──
    QToolBar *toolbar = addToolBar("主工具栏");
    toolbar->setMovable(false);
    toolbar->setIconSize(QSize(20, 20));
    toolbar->setStyleSheet(QString("QToolBar { border-bottom: 1px solid %1; padding: 2px 4px; spacing: 4px; }").arg(Theme::BORDER));

    QAction *actOpenLeft  = toolbar->addAction("打开左相机");
    actOpenLeft->setToolTip("打开左相机");
    QAction *actOpenRight = toolbar->addAction("打开右相机");
    actOpenRight->setToolTip("打开右相机");
    QAction *actCapture   = toolbar->addAction("同时拍摄");
    actCapture->setToolTip("同时拍摄左右相机");
    toolbar->addSeparator();
    QAction *actAutoLoad  = toolbar->addAction("自动加载图片");
    actAutoLoad->setToolTip("从默认路径加载全部图片: 标定/激光/转台/重建序列");
    QAction *actCalib     = toolbar->addAction("相机标定");
    actCalib->setToolTip("执行单目相机标定");
    QAction *actStereo    = toolbar->addAction("立体标定");
    actStereo->setToolTip("执行双目立体标定");
    QAction *actLaserCal  = toolbar->addAction("光平面标定");
    actLaserCal->setToolTip("自动检测激光线→标定光平面");
    QAction *actAxisCal   = toolbar->addAction("转台标定");
    actAxisCal->setToolTip("执行旋转轴自动标定");
    toolbar->addSeparator();
    QAction *actAutoStart = toolbar->addAction("开始采集");
    actAutoStart->setToolTip("开始自动采集 (串口触发模式)");
    QAction *actAutoPause = toolbar->addAction("暂停采集");
    actAutoPause->setToolTip("跳过当前/解救下位机");
    QAction *actAutoStop  = toolbar->addAction("终止采集");
    actAutoStop->setToolTip("强制终止自动采集");
    toolbar->addSeparator();
    QAction *actRecon     = toolbar->addAction("开始重建");
    actRecon->setToolTip("开始三维重建流水线 S1-S6");

    connect(actOpenLeft,  &QAction::triggered, this, &MainWindow::onOpenLeftCameraClicked);
    connect(actOpenRight, &QAction::triggered, this, &MainWindow::onOpenRightCameraClicked);
    connect(actCapture,   &QAction::triggered, this, &MainWindow::onCaptureBothClicked);
    connect(actAutoLoad,  &QAction::triggered, this, &MainWindow::onAutoLoadAllImages);
    connect(actCalib,     &QAction::triggered, this, &MainWindow::onCalibrateClicked);
    connect(actStereo,    &QAction::triggered, this, &MainWindow::onCalculateStereoParams);
    connect(actLaserCal,  &QAction::triggered, this, &MainWindow::onToolbarLaserCalib);
    connect(actAxisCal,   &QAction::triggered, this, &MainWindow::onExecAxisCalibClicked);
    connect(actAutoStart, &QAction::triggered, this, &MainWindow::onStartAutoCollect);
    connect(actAutoPause, &QAction::triggered, this, &MainWindow::onPauseAutoCollect);
    connect(actAutoStop,  &QAction::triggered, this, &MainWindow::onEndAutoCollect);
    connect(actRecon,     &QAction::triggered, this, &MainWindow::onStartReconstructionClicked);

    // ── Tab 容器 ──
    tabWidget = new QTabWidget(this);
    initTab1(tabWidget);
    initTab2(tabWidget);
    initTab3(tabWidget);
    initTab4(tabWidget);
    initTab5(tabWidget);
    setCentralWidget(tabWidget);

    // ── 状态栏 ──
    QStatusBar *sb = statusBar();
    sb->setStyleSheet(QString("QStatusBar { background: %1; border-top: 1px solid %2; padding: 2px 8px; }"
                              "QStatusBar::item { border: none; }").arg(Theme::BG_HOVER, Theme::BORDER));

    auto mkStatus = [](const QString& text) -> QLabel* {
        QLabel *lbl = new QLabel(text);
        lbl->setStyleSheet(QString("color: %1; padding: 0 8px;").arg(Theme::DANGER));
        return lbl;
    };

    m_statusCamera   = mkStatus("相机: 未连接");
    m_statusCamera->setToolTip("左/右相机连接状态");
    m_statusCamCalib = mkStatus("单目: 未标定");
    m_statusCamCalib->setToolTip("单目标定: 计算左右相机内参和畸变系数");
    m_statusStereo   = mkStatus("立体: 未标定");
    m_statusStereo->setToolTip("立体标定: 计算双目外参 R/T 和校正矩阵");
    m_statusLaser    = mkStatus("光平面: 未标定");
    m_statusLaser->setToolTip("光平面标定: 拟合激光平面方程 (需要先执行激光线检测)");
    m_statusAxis     = mkStatus("转台: 未标定");
    m_statusAxis->setToolTip("转台标定: 标定旋转轴方向和轴点位置");
    m_statusSerial   = mkStatus("串口: 关闭");
    m_statusSerial->setToolTip("STM32 串口状态 (115200bps)");

    sb->addWidget(m_statusCamera);
    sb->addWidget(m_statusCamCalib);
    sb->addWidget(m_statusStereo);
    sb->addWidget(m_statusLaser);
    sb->addWidget(m_statusAxis);
    sb->addWidget(m_statusSerial);
    sb->addPermanentWidget(new QLabel("v1.0"));

    // ── 初始化 ──
    initDirectories();
    Logger::instance().setLogFile("Resources/app.log");
    Logger::info("=== 应用启动 ===");
    tryAutoLoadCalibration();
    m_boardSize = cv::Size(11, 8);
    m_squareSize = 10.0f;

    // ── 恢复窗口状态 ──
    QSettings settings("3DRecon", "Camera");
    if (settings.contains("geometry"))
        restoreGeometry(settings.value("geometry").toByteArray());
    if (settings.contains("windowState"))
        restoreState(settings.value("windowState").toByteArray());
}

MainWindow::~MainWindow()
{
    // 停止相机线程 (parent=this, Qt 自动删除)
    if (threadLeftCam && threadLeftCam->isRunning()) { threadLeftCam->stop(); threadLeftCam->wait(); }
    if (threadRightCam && threadRightCam->isRunning()) { threadRightCam->stop(); threadRightCam->wait(); }
    // 停止激光工作线程
    if (m_laserThread && m_laserThread->isRunning()) { m_laserThread->quit(); m_laserThread->wait(); }
    // 手动删除无 parent 的堆对象
    delete m_laserWorker;
    delete m_builder;
    delete m_viewer3D;
    delete m_rotatingCalibrator;
    delete m_pointCalibrator;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QSettings settings("3DRecon", "Camera");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    QMainWindow::closeEvent(event);
}

// ==================== 共享辅助函数 ====================

void MainWindow::initDirectories()
{
    QDir resDir("Resources");
    if (!resDir.exists()) resDir.mkpath(".");

    dirLeftCalib = "Resources/Left";
    dirRightCalib = "Resources/Right";
    dirLeftLaser = "Resources/Left_laser";
    dirRightLaser = "Resources/Right_laser";
    dirLeftPlatform = "Resources/Left_Platform";
    dirRightPlatform = "Resources/Right_Platform";
    dirLeftPointCloud = "Resources/Left_PointCloud";
    dirRightPointCloud = "Resources/Right_PointCloud";

    QDir().mkpath(dirLeftCalib);
    QDir().mkpath(dirRightCalib);
    QDir().mkpath(dirLeftLaser);
    QDir().mkpath(dirRightLaser);
    QDir().mkpath(dirLeftPlatform);
    QDir().mkpath(dirRightPlatform);
    QDir().mkpath(dirLeftPointCloud);
    QDir().mkpath(dirRightPointCloud);
}

QStringList MainWindow::getFilesInFolder(const QString& folderPath, int maxCount)
{
    QDir dir(folderPath);
    QStringList filters; filters << "*.jpg" << "*.jpeg" << "*.JPG" << "*.JPEG" << "*.png" << "*.PNG" << "*.bmp" << "*.BMP" << "*.tiff" << "*.tif";
    dir.setNameFilters(filters);
    dir.setSorting(QDir::Name);
    QStringList fileList = dir.entryList();
    if (maxCount > 0 && fileList.size() > maxCount) fileList = fileList.mid(0, maxCount);
    QStringList fullPaths;
    for (const QString &file : fileList) fullPaths.append(dir.absoluteFilePath(file));
    return fullPaths;
}

QImage MainWindow::mat2QImage(const cv::Mat &mat) {
    if(mat.empty() || mat.cols <= 0 || mat.rows <= 0) return QImage();
    if(mat.type() == CV_8UC1) return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8).copy();
    if(mat.type() == CV_8UC3) {
        cv::Mat rgb; cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
    return QImage();
}

void MainWindow::onAutoLoadAllImages()
{
    Logger::info(">>> 自动加载全部图片 (默认路径)...");
    int totalLoaded = 0;
    int nCalibL = 0, nCalibR = 0, nLaserL = 0, nLaserR = 0, nRot = 0, nRecon = 0;

    // --- Tab2: 标定图 ---
    QStringList calibL = getFilesInFolder(dirLeftCalib, 0);
    QStringList calibR = getFilesInFolder(dirRightCalib, 0);
    nCalibL = calibL.size(); nCalibR = calibR.size();
    if (!calibL.isEmpty() || !calibR.isEmpty()) {
        calibratorLeft.setBoardSize(m_boardSize.width, m_boardSize.height);
        calibratorLeft.setSquareSize(m_squareSize);
        calibratorRight.setBoardSize(m_boardSize.width, m_boardSize.height);
        calibratorRight.setSquareSize(m_squareSize);
        listCalibration->clear();
        for (const QString &f : calibL) {
            bool ok = calibratorLeft.processImage(f.toStdString());
            QListWidgetItem *item = new QListWidgetItem("[左] " + QFileInfo(f).fileName());
            item->setData(Qt::UserRole, f); item->setData(Qt::UserRole + 1, 0);
            item->setForeground(ok ? Qt::darkGreen : Qt::red);
            listCalibration->addItem(item);
        }
        for (const QString &f : calibR) {
            bool ok = calibratorRight.processImage(f.toStdString());
            QListWidgetItem *item = new QListWidgetItem("[右] " + QFileInfo(f).fileName());
            item->setData(Qt::UserRole, f); item->setData(Qt::UserRole + 1, 1);
            item->setForeground(ok ? Qt::darkGreen : Qt::red);
            listCalibration->addItem(item);
        }
        // 同时填充立体对列表
        m_StereoFilesL = calibL;
        m_StereoFilesR = calibR;
        listStereoPairs->clear();
        int pairCount = qMin(calibL.size(), calibR.size());
        for (int i = 0; i < pairCount; ++i) {
            listStereoPairs->addItem(QString("Pair %1: %2 <-> %3")
                .arg(i+1).arg(QFileInfo(calibL[i]).fileName()).arg(QFileInfo(calibR[i]).fileName()));
        }
        totalLoaded += calibL.size() + calibR.size();
        Logger::info(QString("  标定图: 左%1 右%2, 立体对%3").arg(calibL.size()).arg(calibR.size()).arg(pairCount));
    } else {
        Logger::warning(QString("  标定图未找到 (左:%1 右:%2)").arg(calibL.size()).arg(calibR.size()));
    }

    // --- Tab3: 无激光图 + 有激光图 ---
    QStringList noLaserL = getFilesInFolder(dirLeftCalib, 0);  // 无激光图与标定图共用目录
    QStringList noLaserR = getFilesInFolder(dirRightCalib, 0);
    if (!noLaserL.isEmpty() && !noLaserR.isEmpty()) {
        m_NoLaserFilesL = noLaserL;
        m_NoLaserFilesR = noLaserR;
        Logger::info(QString("  无激光图: 左%1 右%2").arg(noLaserL.size()).arg(noLaserR.size()));
    }
    QStringList laserL = getFilesInFolder(dirLeftLaser, 0);
    QStringList laserR = getFilesInFolder(dirRightLaser, 0);
    nLaserL = laserL.size(); nLaserR = laserR.size();
    if (!laserL.isEmpty() && !laserR.isEmpty()) {
        m_LaserFilesL = laserL;
        m_LaserFilesR = laserR;
        updateLaserPairList();
        totalLoaded += laserL.size() + laserR.size();
        Logger::info(QString("  激光图: 左%1 右%2").arg(laserL.size()).arg(laserR.size()));
    } else {
        Logger::warning(QString("  激光图未找到 (左:%1 右:%2)").arg(laserL.size()).arg(laserR.size()));
    }

    // --- Tab4: 旋转序列 ---
    QStringList rotL = getFilesInFolder(dirLeftPlatform, 0);
    QStringList rotR = getFilesInFolder(dirRightPlatform, 0);
    if (!rotL.isEmpty() && !rotR.isEmpty()) {
        m_rotSeqLeftPaths = rotL;
        m_rotSeqRightPaths = rotR;
        int cnt = qMin(rotL.size(), rotR.size());
        txtAxisCalibResult->clear();
        txtAxisCalibResult->append(QString(">>> [自动] 已加载旋转序列: %1 对").arg(cnt));
        txtAxisCalibResult->append(QString("    左: %1").arg(dirLeftPlatform));
        txtAxisCalibResult->append(QString("    右: %1").arg(dirRightPlatform));
        txtAxisCalibResult->append(">>> 点击\"执行轴标定\"或工具栏\"转台标定\"");
        btnExecAxisCalib->setEnabled(true);
        btnDebugAxisCalib->setEnabled(false);
        nRot = cnt;
        totalLoaded += cnt * 2;
        Logger::info(QString("  转台序列: %1 对").arg(cnt));
    } else {
        Logger::warning(QString("  转台序列未找到 (左:%1 右:%2)").arg(rotL.size()).arg(rotR.size()));
    }

    // --- Tab5: 重建序列 ---
    QStringList reconL = getFilesInFolder(dirLeftPointCloud, 999);
    QStringList reconR = getFilesInFolder(dirRightPointCloud, 999);
    if (!reconL.isEmpty() && !reconR.isEmpty()) {
        nRecon = qMin(reconL.size(), reconR.size());
        m_reconLeftPaths = reconL.mid(0, nRecon);
        m_reconRightPaths = reconR.mid(0, nRecon);
        spinViewCount->setValue(nRecon);
        txtReconLog->clear();
        txtReconLog->append(QString(">>> [自动] 已加载重建序列: %1 对").arg(nRecon));
        txtReconLog->append(QString("    左: %1").arg(dirLeftPointCloud));
        txtReconLog->append(QString("    右: %1").arg(dirRightPointCloud));
        totalLoaded += nRecon * 2;
        Logger::info(QString("  重建序列: %1 对").arg(nRecon));
    } else {
        Logger::warning(QString("  重建序列未找到 (左:%1 右:%2)").arg(reconL.size()).arg(reconR.size()));
    }

    Logger::info(QString(">>> 自动加载完成: 共 %1 张").arg(totalLoaded));
    statusBar()->showMessage(QString("自动加载完成: 标定%1+%2张 | 激光%3+%4张 | 转台%5对 | 重建%6对 — 就绪")
        .arg(nCalibL).arg(nCalibR).arg(nLaserL).arg(nLaserR).arg(nRot).arg(nRecon), 8000);
}
