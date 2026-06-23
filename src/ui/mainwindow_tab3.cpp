// ==================== Tab3: 激光标定 ====================
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include "ui/logger.h"
#include <QSplitter>
#include "io/laserworker.h"
#include "core/lasercalibration.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>
#include <QDialog>
#include <QStatusBar>
#include <QThread>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QDebug>
#include <QFormLayout>

void MainWindow::initTab3(QTabWidget* tabWidget) {
    QWidget *page = new QWidget();
    QVBoxLayout *tab3Layout = new QVBoxLayout(page);
    tab3Layout->setContentsMargins(Theme::MARGIN, Theme::MARGIN, Theme::MARGIN, Theme::MARGIN);
    QSplitter *splitter = new QSplitter(Qt::Horizontal);
    splitter->setHandleWidth(1);
    tab3Layout->addWidget(splitter);

    m_laserThread = new QThread(this);
    m_laserWorker = new LaserWorker();

    // 将 Worker 移动到新线程中执行
    m_laserWorker->moveToThread(m_laserThread);

    // 连接 Worker 的信号
    connectLaserWorker();

    // 启动线程（让它进入事件循环待命）
    m_laserThread->start();

    // 1. 左侧控制面板
    QWidget *tab3Control = new QWidget();
    QVBoxLayout *tab3CtrlLayout = new QVBoxLayout(tab3Control);
    tab3Control->setFixedWidth(Theme::CONTROL_PANEL_W);

    QGroupBox *grpLaser = new QGroupBox("激光标定");
    QVBoxLayout *grpLaserLayout = new QVBoxLayout(grpLaser);

    btnSelectNoLaser = new QPushButton("选择无激光图 (棋盘格)");
    btnSelectNoLaser->setStyleSheet(Theme::boldButton());

    btnSelectLaser = new QPushButton("选择有激光图 (激光线)");
    btnSelectLaser->setStyleSheet(Theme::boldButton());

    QPushButton *btnT3Detect = new QPushButton("执行激光线检测");
    btnT3Detect->setStyleSheet(Theme::successButton());

    QPushButton *btnT3Calib = new QPushButton("执行光平面标定");
    btnT3Calib->setStyleSheet(Theme::primaryButton());

    // ================= 修复：必须 new 按钮才能使用 =================
    QPushButton *btnShowAnalysis = new QPushButton("显示评价与拟合结果");
    btnShowAnalysis->setStyleSheet(Theme::purpleButton());

    btnShowLaserResult = new QPushButton("显示光平面结果");

    grpLaserLayout->addWidget(btnSelectNoLaser);
    grpLaserLayout->addWidget(btnSelectLaser);
    grpLaserLayout->addWidget(btnT3Detect);
    grpLaserLayout->addWidget(btnT3Calib);
    grpLaserLayout->addWidget(btnShowAnalysis); // 现在可以安全使用
    grpLaserLayout->addWidget(btnShowLaserResult);

    tab3CtrlLayout->addWidget(grpLaser);
    tab3CtrlLayout->addStretch();

    // 2. 中间列表区域
    QWidget *tab3ListPanel = new QWidget();
    QVBoxLayout *tab3ListLayout = new QVBoxLayout(tab3ListPanel);
    tab3ListLayout->addWidget(new QLabel("图像对列表:"));
    listLaserPairs = new QListWidget();
    tab3ListLayout->addWidget(listLaserPairs, 1);

    // 3. 右侧显示区域
    QWidget *tab3Display = new QWidget();
    QGridLayout *tab3DispLayout = new QGridLayout(tab3Display);

    tab3ViewLeftUndistort = new VideoWidget();
    tab3ViewRightUndistort = new VideoWidget();
    tab3ViewLeftLaser = new VideoWidget();
    tab3ViewRightLaser = new VideoWidget();

    tab3ViewLeftUndistort->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
    tab3ViewRightUndistort->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
    tab3ViewLeftLaser->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
    tab3ViewRightLaser->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);

    tab3DispLayout->addWidget(new QLabel("左相机(带ROI)"), 0, 0, Qt::AlignCenter);
    tab3DispLayout->addWidget(new QLabel("右相机(带ROI)"), 0, 1, Qt::AlignCenter);
    tab3DispLayout->addWidget(tab3ViewLeftUndistort, 1, 0);
    tab3DispLayout->addWidget(tab3ViewRightUndistort, 1, 1);
    tab3DispLayout->addWidget(new QLabel("左激光细节"), 2, 0, Qt::AlignCenter);
    tab3DispLayout->addWidget(new QLabel("右激光细节"), 2, 1, Qt::AlignCenter);
    tab3DispLayout->addWidget(tab3ViewLeftLaser, 3, 0);
    tab3DispLayout->addWidget(tab3ViewRightLaser, 3, 1);
    tab3DispLayout->setRowStretch(1, 1);
    tab3DispLayout->setRowStretch(3, 1);

    splitter->addWidget(tab3Control);
    splitter->addWidget(tab3ListPanel);
    splitter->addWidget(tab3Display);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setStretchFactor(2, 2);

    tabWidget->addTab(page, "光平面标定");

    // 信号连接
    connect(btnSelectNoLaser, &QPushButton::clicked, this, &MainWindow::onSelectNoLaserImagesClicked);
    connect(btnSelectLaser, &QPushButton::clicked, this, &MainWindow::onSelectLaserImagesClicked);
    connect(btnT3Detect, &QPushButton::clicked, this, &MainWindow::onTab3DetectLaser);
    connect(btnT3Calib, &QPushButton::clicked, this, &MainWindow::onTab3CalibrateLaser);
    connect(btnShowLaserResult, &QPushButton::clicked, this, &MainWindow::onTab3ShowLaserResult);
    connect(btnShowAnalysis, &QPushButton::clicked, this, &MainWindow::onTab3ShowLaserAnalysis);
    connect(listLaserPairs, &QListWidget::itemClicked, this, &MainWindow::onLaserListItemClicked);
}

void MainWindow::connectLaserWorker() {
    qDebug() << "[调试] connectLaserWorker 开始";
    qDebug() << "[调试] m_laserWorker:" << (void*)m_laserWorker;
    qDebug() << "[调试] m_laserThread:" << (void*)m_laserThread;

    if (!m_laserWorker || !m_laserThread) {
        qDebug() << "[调试-错误] worker 或 thread 为空！";
        return;
    }

    connect(m_laserWorker, &LaserWorker::progress, this, [this](int current, int total){
        qDebug() << "[调试-进度]" << current << "/" << total;
        statusBar()->showMessage(QString("正在处理: %1/%2 ...").arg(current).arg(total));
    });

    // 接收一次性抛出的结果
    connect(m_laserWorker, &LaserWorker::finished, this, [this](
        QVector<LaserPair> validPairs,
        QVector<LaserProcessingResult> cacheL,
        QVector<LaserProcessingResult> cacheR,
        int totalProcessed)
    {
        qDebug() << "[调试-finished信号] 收到结果 - validPairs:" << validPairs.size()
                 << " cacheL:" << cacheL.size() << " cacheR:" << cacheR.size()
                 << " totalProcessed:" << totalProcessed;

        // 【调试：检查数据一致性】
        if (totalProcessed != cacheL.size() || totalProcessed != cacheR.size()) {
            qDebug() << "[调试-警告] 数据大小不一致！";
        }

        // 直接替换 UI 层的全局数据
        m_AllLaserData = std::vector<LaserPair>(validPairs.begin(), validPairs.end());
        m_cachedResultsL = std::vector<LaserProcessingResult>(cacheL.begin(), cacheL.end());
        m_cachedResultsR = std::vector<LaserProcessingResult>(cacheR.begin(), cacheR.end());

        qDebug() << "[调试-finished信号] 数据赋值完成，开始更新列表颜色";

        // 更新左侧列表的颜色状态
        for (int i = 0; i < totalProcessed; ++i) {
            if (listLaserPairs && listLaserPairs->item(i)) {
                bool isValid = (i < cacheL.size() && cacheL[i].success &&
                                i < cacheR.size() && cacheR[i].success);
                listLaserPairs->item(i)->setForeground(isValid ? Qt::darkGreen : Qt::red);
            }
        }

        qDebug() << "[调试-finished信号] 列表颜色更新完成";
        statusBar()->showMessage(QString("检测完成！有效对: %1").arg(validPairs.size()), 5000);
        m_isLaserProcessing = false;

        // 工具栏链式调用: 检测完成后自动执行光平面标定
        if (m_pendingLaserCalib) {
            m_pendingLaserCalib = false;
            if (!m_AllLaserData.empty()) {
                qDebug() << "[调试] 工具栏链式调用 → 光平面标定";
                onTab3CalibrateLaser();
                return;  // 跳过独立的完成弹窗
            }
        }
        QMessageBox::information(this, "完成", QString("批量检测完成！\n有效数据: %1 对").arg(validPairs.size()));
        qDebug() << "[调试-finished信号] 处理完毕";
    });

    connect(m_laserWorker, &LaserWorker::error, this, [this](const QString& msg){
        qDebug() << "[调试-error信号]" << msg;
    });

    connect(m_laserThread, &QThread::finished, m_laserWorker, &QObject::deleteLater);

    qDebug() << "[调试] connectLaserWorker 完成";
}

void MainWindow::onSelectNoLaserImagesClicked() {
    QStringList filesL = QFileDialog::getOpenFileNames(this, "选择无激光左图", dirLeftCalib, "Images (*.jpg *.png *.bmp)");
    if (filesL.isEmpty()) return;
    m_NoLaserFilesL = filesL;

    QStringList filesR = QFileDialog::getOpenFileNames(this, "选择无激光右图", dirRightCalib, "Images (*.jpg *.png *.bmp)");
    if (filesR.isEmpty()) return;
    m_NoLaserFilesR = filesR;

    updateLaserPairList();
}

void MainWindow::onSelectLaserImagesClicked() {
    QStringList filesL = QFileDialog::getOpenFileNames(this, "选择有激光左图", dirLeftLaser, "Images (*.jpg *.png *.bmp)");
    if (filesL.isEmpty()) return;
    m_LaserFilesL = filesL;

    QStringList filesR = QFileDialog::getOpenFileNames(this, "选择有激光右图", dirRightLaser, "Images (*.jpg *.png *.bmp)");
    if (filesR.isEmpty()) return;
    m_LaserFilesR = filesR;

    updateLaserPairList();
}

void MainWindow::updateLaserPairList() {
    listLaserPairs->clear();
    // 【Qt5修改】显式转换为 int，避免 size_t 隐式转换警告
    int count = static_cast<int>(std::min({m_NoLaserFilesL.size(), m_NoLaserFilesR.size(),
                          m_LaserFilesL.size(), m_LaserFilesR.size()}));

    if (count == 0) return;

    for(int i = 0; i < count; ++i) {
        listLaserPairs->addItem(QString("Pair %1: NoLaser <-> Laser").arg(i + 1));
    }
}

void MainWindow::onLaserListItemClicked(QListWidgetItem *item) {
    if (!item) return;

    int idx = listLaserPairs->row(item);

    if (idx < 0 || idx >= static_cast<int>(m_cachedResultsL.size())) {
        if (idx >= 0 && idx < m_LaserFilesL.size()) {
            cv::Mat img = cv::imread(m_LaserFilesL[idx].toStdString());
            if (!img.empty() && tab3ViewLeftUndistort)
                tab3ViewLeftUndistort->setFrame(mat2QImage(img));
        }
        if (tab3ViewRightUndistort) tab3ViewRightUndistort->setFrame(QImage());
        if (tab3ViewLeftLaser) tab3ViewLeftLaser->setFrame(QImage());
        if (tab3ViewRightLaser) tab3ViewRightLaser->setFrame(QImage());
        return;
    }

    LaserProcessingResult resL = m_cachedResultsL[idx];
    LaserProcessingResult resR = m_cachedResultsR[idx];

    cv::Mat imgL = cv::imread(m_LaserFilesL[idx].toStdString());
    cv::Mat imgR = cv::imread(m_LaserFilesR[idx].toStdString());

    // ================= 左相机可视化 =================
    if (!imgL.empty() && tab3ViewLeftUndistort) {
        cv::Mat vis = imgL.clone();

        if (resL.success) {
            for (const auto &pt : resL.laserPoints) cv::circle(vis, pt, 2, cv::Scalar(0, 255, 255), -1);
        }
        tab3ViewLeftUndistort->setFrame(mat2QImage(vis));
    }

    // ================= 右相机可视化 =================
    if (!imgR.empty() && tab3ViewRightUndistort) {
        cv::Mat vis = imgR.clone();

        if (resR.success) {
            for (const auto &pt : resR.laserPoints) cv::circle(vis, pt, 2, cv::Scalar(0, 255, 255), -1);
        }
        tab3ViewRightUndistort->setFrame(mat2QImage(vis));
    }

    // ================= 左激光细节 (ROI放大) =================
    if (resL.success && !imgL.empty() && tab3ViewLeftLaser) {
        cv::Rect safeRoi = resL.roi & cv::Rect(0, 0, imgL.cols, imgL.rows);
        if (safeRoi.width > 0 && safeRoi.height > 0) {
            cv::Mat roiImg = imgL(safeRoi).clone();
            for (const auto &pt : resL.laserPoints) {
                cv::circle(roiImg, cv::Point(pt.x - safeRoi.x, pt.y - safeRoi.y), 3, cv::Scalar(255, 0, 0), -1);
            }
            if (roiImg.cols < 300) cv::resize(roiImg, roiImg, cv::Size(), 2.0, 2.0);
            tab3ViewLeftLaser->setFrame(mat2QImage(roiImg));
        }
    } else if (tab3ViewLeftLaser) {
        tab3ViewLeftLaser->setFrame(QImage());
    }

    // ================= 右激光细节 (ROI放大) =================
    if (resR.success && !imgR.empty() && tab3ViewRightLaser) {
        cv::Rect safeRoi = resR.roi & cv::Rect(0, 0, imgR.cols, imgR.rows);
        if (safeRoi.width > 0 && safeRoi.height > 0) {
            cv::Mat roiImg = imgR(safeRoi).clone();
            for (const auto &pt : resR.laserPoints) {
                cv::circle(roiImg, cv::Point(pt.x - safeRoi.x, pt.y - safeRoi.y), 3, cv::Scalar(255, 0, 0), -1);
            }
            if (roiImg.cols < 300) cv::resize(roiImg, roiImg, cv::Size(), 2.0, 2.0);
            tab3ViewRightLaser->setFrame(mat2QImage(roiImg));
        }
    } else if (tab3ViewRightLaser) {
        tab3ViewRightLaser->setFrame(QImage());
    }
}

void MainWindow::onTab3DetectLaser() {
    qDebug() << "========== [激光检测] 开始 ==========";

    // 【调试 1：检查指针状态】
    qDebug() << "[调试] m_laserThread 指针:" << (void*)m_laserThread;
    qDebug() << "[调试] m_laserWorker 指针:" << (void*)m_laserWorker;

    if (m_laserThread) {
        qDebug() << "[调试] 线程运行状态:" << m_laserThread->isRunning()
                 << "完成状态:" << m_laserThread->isFinished();
    }

    // ====================================================================
    // 【修复 1：防止重复点击与野指针访问】
    // ====================================================================
    if (m_isLaserProcessing) {
        qDebug() << "[调试] 正在处理上一批任务，拦截重复点击";
        QMessageBox::warning(this, "提示", "后台正在处理中，请勿重复点击！");
        m_pendingLaserCalib = false;
        return;
    }

    // 【调试 2：检查标定状态】
    qDebug() << "[调试] m_IsRectified:" << m_IsRectified;
    qDebug() << "[调试] 左相机矩阵空:" << m_CameraMatrixL.empty()
             << "尺寸:" << (m_CameraMatrixL.empty() ? -1 : m_CameraMatrixL.rows) << "x"
             << (m_CameraMatrixL.empty() ? -1 : m_CameraMatrixL.cols);
    qDebug() << "[调试] 右相机矩阵空:" << m_CameraMatrixR.empty()
             << "尺寸:" << (m_CameraMatrixR.empty() ? -1 : m_CameraMatrixR.rows) << "x"
             << (m_CameraMatrixR.empty() ? -1 : m_CameraMatrixR.cols);
    qDebug() << "[调试] 左畸变系数空:" << m_DistCoeffsL.empty()
             << "尺寸:" << (m_DistCoeffsL.empty() ? -1 : m_DistCoeffsL.rows) << "x"
             << (m_DistCoeffsL.empty() ? -1 : m_DistCoeffsL.cols);
    qDebug() << "[调试] 右畸变系数空:" << m_DistCoeffsR.empty()
             << "尺寸:" << (m_DistCoeffsR.empty() ? -1 : m_DistCoeffsR.rows) << "x"
             << (m_DistCoeffsR.empty() ? -1 : m_DistCoeffsR.cols);

    // 基础条件检查
    if (!m_IsRectified) {
        qDebug() << "[调试] 失败：未完成立体标定";
        QMessageBox::warning(this, "错误", "请先完成立体标定。");
        m_pendingLaserCalib = false;
        return;
    }

    if (m_CameraMatrixL.empty() || m_DistCoeffsL.empty()) {
        qDebug() << "[调试] 失败：左相机内参为空";
        QMessageBox::warning(this, "错误", "左相机内参为空，请重新执行单目标定！");
        m_pendingLaserCalib = false;
        return;
    }
    if (m_CameraMatrixR.empty() || m_DistCoeffsR.empty()) {
        qDebug() << "[调试] 失败：右相机内参为空";
        QMessageBox::warning(this, "错误", "右相机内参为空，请重新执行单目标定！");
        m_pendingLaserCalib = false;
        return;
    }

    // 【调试 3：检查文件列表】
    qDebug() << "[调试] 无激光左图数量:" << m_NoLaserFilesL.size();
    qDebug() << "[调试] 无激光右图数量:" << m_NoLaserFilesR.size();
    qDebug() << "[调试] 有激光左图数量:" << m_LaserFilesL.size();
    qDebug() << "[调试] 有激光右图数量:" << m_LaserFilesR.size();

    if (m_NoLaserFilesL.empty() || m_LaserFilesL.empty() ||
        m_NoLaserFilesR.empty() || m_LaserFilesR.empty()) {
        qDebug() << "[调试] 失败：图片列表为空";
        QMessageBox::warning(this, "提示", "请先选择无激光图和有激光图。");
        m_pendingLaserCalib = false;
        return;
    }

    if (m_boardSize.width == 0 || m_boardSize.height == 0) {
        qDebug() << "[调试] 失败：棋盘格尺寸未设置" << m_boardSize.width << m_boardSize.height;
        QMessageBox::warning(this, "错误", "棋盘格尺寸未设置。");
        m_pendingLaserCalib = false;
        return;
    }

    qDebug() << "[调试] 棋盘格尺寸:" << m_boardSize.width << "x" << m_boardSize.height;
    qDebug() << "[调试] 方格大小:" << m_squareSize;

    // ====================================================================
    // 【修复 2：截取等长数据】
    // ====================================================================
    int count = static_cast<int>(std::min({m_NoLaserFilesL.size(), m_NoLaserFilesR.size(),
                                          m_LaserFilesL.size(), m_LaserFilesR.size()}));
    qDebug() << "[调试] 有效图片对数:" << count;

    if (count == 0) {
        qDebug() << "[调试] 失败：有效图片对数为0";
        QMessageBox::warning(this, "错误", "有效图片对数为0，请检查选择的图片数量是否匹配！");
        m_pendingLaserCalib = false;
        return;
    }

    QStringList safeNoL = m_NoLaserFilesL.mid(0, count);
    QStringList safeLaserL = m_LaserFilesL.mid(0, count);
    QStringList safeNoR = m_NoLaserFilesR.mid(0, count);
    QStringList safeLaserR = m_LaserFilesR.mid(0, count);

    qDebug() << "[调试] 截取后数量 - 无激光L:" << safeNoL.size()
             << " 有激光L:" << safeLaserL.size()
             << " 无激光R:" << safeNoR.size()
             << " 有激光R:" << safeLaserR.size();

    // ====================================================================
    // 【修复 3：增加 UI 控件空指针保护】
    // ====================================================================
    m_AllLaserData.clear();
    m_cachedResultsL.clear();
    m_cachedResultsR.clear();

    qDebug() << "[调试] listLaserPairs 指针:" << (void*)listLaserPairs;
    if (listLaserPairs) {
        for(int i = 0; i < listLaserPairs->count(); ++i) {
            QListWidgetItem* item = listLaserPairs->item(i);
            if (item) {
                item->setForeground(Qt::black);
            }
        }
    }

    // ====================================================================
    // 【关键调试：检查 m_laserWorker 并安全调用】
    // ====================================================================
    if (!m_laserWorker) {
        qDebug() << "[调试-致命错误] m_laserWorker 为空指针！";
        QMessageBox::critical(this, "严重错误", "工作线程对象丢失，请重启软件！");
        return;
    }

    m_isLaserProcessing = true; // <-- 添加这一行：标记开始处理
    qDebug() << "[调试] 准备调用 setFiles...";

    try {
        qDebug() << "[调试] 准备调用 setFiles...";

        // 逐步测试每个参数的拷贝，看卡在哪一步
        qDebug() << "[调试] 1. 测试字符串列表...";
        auto safeNoL_v = safeNoL.toVector();
        auto safeLaserL_v = safeLaserL.toVector();
        auto safeNoR_v = safeNoR.toVector();
        auto safeLaserR_v = safeLaserR.toVector();
        qDebug() << "[调试] 字符串列表正常";

        qDebug() << "[调试] 2. 测试 m_laserCalib...";
        auto calibCopy = m_laserCalib; // 如果这里崩，说明这个类的拷贝构造有问题
        qDebug() << "[调试] m_laserCalib 正常";

        qDebug() << "[调试] 3. 测试左相机矩阵 clone...";
        auto matL = m_CameraMatrixL.clone();
        qDebug() << "[调试] 左相机矩阵正常";

        qDebug() << "[调试] 4. 测试左畸变系数 clone...";
        auto distL = m_DistCoeffsL.clone();
        qDebug() << "[调试] 左畸变系数正常";

        qDebug() << "[调试] 5. 测试右相机矩阵 clone...";
        auto matR = m_CameraMatrixR.clone();
        qDebug() << "[调试] 右相机矩阵正常";

        qDebug() << "[调试] 6. 测试右畸变系数 clone...";
        auto distR = m_DistCoeffsR.clone();
        qDebug() << "[调试] 右畸变系数正常";

        qDebug() << "[调试] 7. 测试 StegerParams...";
        auto stegerCopy = m_StegerParams;
        qDebug() << "[调试] StegerParams 正常";

        qDebug() << "[调试] 8. 测试 LABParams...";
        auto labCopy = m_LABParams;
        qDebug() << "[调试] LABParams 正常";

        qDebug() << "[调试] 9. 所有参数准备完毕，调用 setFiles...";
        m_laserWorker->setFiles(
            safeNoL_v, safeLaserL_v, safeNoR_v, safeLaserR_v,
            calibCopy,
            matL, distL,
            matR, distR,
            stegerCopy,
            labCopy
        );
        qDebug() << "[调试] setFiles 调用成功";

    } catch (const std::exception& e) {
        qDebug() << "[调试-异常] setFiles 抛出异常:" << e.what();
        QMessageBox::critical(this, "异常", QString("setFiles 异常: %1").arg(e.what()));
        return;
    } catch (...) {
        qDebug() << "[调试-异常] setFiles 抛出未知异常";
        QMessageBox::critical(this, "异常", "setFiles 发生未知异常");
        return;
    }

    qDebug() << "[调试] 准备调用 process()...";
    qDebug() << "[调试] m_laserWorker 线程亲和性:"
             << (m_laserWorker->thread() == QThread::currentThread() ? "主线程" : "工作线程");

    try {
        // ====================================================================
        // 【重要修复：正确触发子线程】
        // 如果 worker 被 moveToThread，必须通过信号触发，不能直接调用！
        // ====================================================================
        if (m_laserWorker->thread() != QThread::currentThread()) {
            qDebug() << "[调试] Worker 在工作线程，使用信号触发";
            QMetaObject::invokeMethod(m_laserWorker, "process", Qt::QueuedConnection);
        } else {
            qDebug() << "[调试] Worker 在主线程，直接调用（可能会卡UI）";
            m_laserWorker->process();
        }
        qDebug() << "[调试] process 调用成功";
    } catch (const std::exception& e) {
        qDebug() << "[调试-异常] process 抛出异常:" << e.what();
        QMessageBox::critical(this, "异常", QString("process 异常: %1").arg(e.what()));
    } catch (...) {
        qDebug() << "[调试-异常] process 抛出未知异常";
        QMessageBox::critical(this, "异常", "process 发生未知异常");
    }

    qDebug() << "========== [激光检测] 函数返回 ==========";
}

void MainWindow::onLaserProgress(int current, int total) {
    statusBar()->showMessage(QString("正在处理: %1/%2 ...").arg(current).arg(total));
    QApplication::processEvents(); // 允许UI响应
}

void MainWindow::onLaserResultReady(const std::vector<LaserPair>& results) {
    m_AllLaserData = results;
    updateLaserPairList();
}

void MainWindow::onLaserFinished() {
    statusBar()->showMessage(QString("检测完成！成功 %1 对").arg(static_cast<int>(m_AllLaserData.size())), 5000);
    if (m_pendingLaserCalib) {
        m_pendingLaserCalib = false;
        if (!m_AllLaserData.empty()) {
            onTab3CalibrateLaser();
        }
    } else {
        QMessageBox::information(this, "完成",
            QString("批量检测完成！\n成功处理 %1 对图像。").arg(static_cast<int>(m_AllLaserData.size())));
    }
}

void MainWindow::onLaserError(const QString& message) {
    m_pendingLaserCalib = false;
    QMessageBox::warning(this, "错误", message);
    statusBar()->showMessage("激光检测出错: " + message, 5000);
}

void MainWindow::onToolbarLaserCalib()
{
    Logger::info(">>> [工具栏] 光平面标定 触发");
    // 如果已有检测数据，直接标定
    if (!m_AllLaserData.empty()) {
        onTab3CalibrateLaser();
        return;
    }
    // 自动加载无激光图 (棋盘格)
    if (m_NoLaserFilesL.isEmpty() || m_NoLaserFilesR.isEmpty()) {
        QStringList noLaserL = getFilesInFolder(dirLeftCalib, 0);
        QStringList noLaserR = getFilesInFolder(dirRightCalib, 0);
        if (!noLaserL.isEmpty() && !noLaserR.isEmpty()) {
            m_NoLaserFilesL = noLaserL;
            m_NoLaserFilesR = noLaserR;
            Logger::info(QString("自动加载无激光图: 左%1 右%2").arg(noLaserL.size()).arg(noLaserR.size()));
        }
    }
    // 自动加载有激光图
    if (m_LaserFilesL.isEmpty() || m_LaserFilesR.isEmpty()) {
        QStringList laserL = getFilesInFolder(dirLeftLaser, 0);
        QStringList laserR = getFilesInFolder(dirRightLaser, 0);
        if (laserL.isEmpty() || laserR.isEmpty()) {
            QMessageBox::warning(this, "错误", QString("激光图路径为空!\n左: %1 (%2张)\n右: %3 (%4张)")
                .arg(dirLeftLaser).arg(laserL.size()).arg(dirRightLaser).arg(laserR.size()));
            return;
        }
        m_LaserFilesL = laserL;
        m_LaserFilesR = laserR;
        Logger::info(QString("自动加载激光图: 左%1 右%2").arg(laserL.size()).arg(laserR.size()));
    }
    // 前置条件检查 (与 onTab3DetectLaser 一致)
    if (!m_IsRectified || m_P1.empty() || m_P2.empty()) {
        QMessageBox::warning(this, "缺少参数", "请先在 Tab2 完成双目立体标定！");
        return;
    }
    if (m_NoLaserFilesL.empty() || m_LaserFilesL.empty()) {
        QMessageBox::warning(this, "缺少图片", "自动加载图片失败，请手动选择无激光图和有激光图。");
        return;
    }

    m_pendingLaserCalib = true;
    Logger::info(">>> 工具栏: 自动激光检测 → 光平面标定");
    onTab3DetectLaser();
}

void MainWindow::onTab3ShowLaserAnalysis() {
    if (m_AllLaserData.empty()) {
        QMessageBox::warning(this, "提示", "没有检测数据，请先执行激光线检测。");
        return;
    }

    LaserCalibration lc;
    QDialog *dialog = new QDialog(this);
    dialog->setWindowTitle("激光线评价与位姿检测结果");
    dialog->resize(1100, 650);

    QVBoxLayout *layout = new QVBoxLayout(dialog);
    QTableWidget *table = new QTableWidget(dialog);

    table->setColumnCount(9);
    table->setHorizontalHeaderLabels(QStringList()
        << "图片ID" << "左相机方程" << "左点数" << "左直线度" << "左相机位姿"
        << "右相机方程" << "右点数" << "右直线度" << "右相机位姿"
    );

    // 【Qt5修改】显式转换 size_t 为 int
    int rowCount = static_cast<int>(m_AllLaserData.size());
    table->setRowCount(rowCount);

    auto formatPose = [](const cv::Vec3d& rvec, const cv::Vec3d& tvec, bool valid) -> QString {
        if (!valid) return QString("无效");
        double angle_deg = cv::norm(rvec) * 180.0 / CV_PI;
        return QString("Rot: %1°\nTrans: [%2, %3, %4]")
               .arg(angle_deg, 0, 'f', 1)
               .arg(tvec[0], 0, 'f', 1).arg(tvec[1], 0, 'f', 1).arg(tvec[2], 0, 'f', 1);
    };

    int validPoseLCount = 0;
    int validPoseRCount = 0;

    // 【Qt5修改】使用 int 类型的循环变量
    for (int i = 0; i < rowCount; ++i) {
        const auto &pair = m_AllLaserData[i];
        LineAnalysisResult resL = lc.analyzeLaserLine(pair.ptsL);
        LineAnalysisResult resR = lc.analyzeLaserLine(pair.ptsR);

        // 统计有效位姿数量
        if (pair.poseValidL) validPoseLCount++;
        if (pair.poseValidR) validPoseRCount++;

        table->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));

        QTableWidgetItem *itemEqL = new QTableWidgetItem(QString::fromStdString(resL.equationStr));
        itemEqL->setTextAlignment(Qt::AlignCenter);
        table->setItem(i, 1, itemEqL);

        table->setItem(i, 2, new QTableWidgetItem(QString::number(resL.pointCount)));

        QTableWidgetItem *itemRmseL = new QTableWidgetItem(QString::number(resL.rmse, 'f', 4));
        itemRmseL->setTextAlignment(Qt::AlignCenter);
        if (resL.rmse > 0.5 && resL.rmse > 0) {
            itemRmseL->setForeground(QBrush(QColor(231, 76, 60)));
            itemRmseL->setToolTip("直线度误差较大，建议检查该图片！");
        }
        table->setItem(i, 3, itemRmseL);

        QString poseStrL = formatPose(pair.rvecL, pair.tvecL, pair.poseValidL);
        QTableWidgetItem *itemPoseL = new QTableWidgetItem(poseStrL);
        itemPoseL->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        if (!pair.poseValidL) {
            itemPoseL->setForeground(QBrush(QColor(231, 76, 60)));
            itemPoseL->setToolTip("PnP求解失败");
        }
        table->setItem(i, 4, itemPoseL);

        // 右侧同理填充
        QTableWidgetItem *itemEqR = new QTableWidgetItem(QString::fromStdString(resR.equationStr));
        itemEqR->setTextAlignment(Qt::AlignCenter);
        table->setItem(i, 5, itemEqR);
        table->setItem(i, 6, new QTableWidgetItem(QString::number(resR.pointCount)));

        QTableWidgetItem *itemRmseR = new QTableWidgetItem(QString::number(resR.rmse, 'f', 4));
        itemRmseR->setTextAlignment(Qt::AlignCenter);
        if (resR.rmse > 0.5 && resR.rmse > 0) {
            itemRmseR->setForeground(QBrush(QColor(231, 76, 60)));
            itemRmseR->setToolTip("直线度误差较大，建议检查该图片！");
        }
        table->setItem(i, 7, itemRmseR);

        QString poseStrR = formatPose(pair.rvecR, pair.tvecR, pair.poseValidR);
        QTableWidgetItem *itemPoseR = new QTableWidgetItem(poseStrR);
        itemPoseR->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        if (!pair.poseValidR) {
            itemPoseR->setForeground(QBrush(QColor(231, 76, 60)));
            itemPoseR->setToolTip("PnP求解失败");
        }
        table->setItem(i, 8, itemPoseR);
    }

    table->resizeColumnsToContents();
    table->setColumnWidth(4, 140);
    table->setColumnWidth(8, 140);

    layout->addWidget(table);

    // 智能诊断统计
    QString summary = QString("注：直线度(RMSE)越小越好。位姿统计 -> 左图有效: %1/%2, 右图有效: %3/%4")
                      .arg(validPoseLCount).arg(rowCount)
                      .arg(validPoseRCount).arg(rowCount);
    QLabel *lblNote = new QLabel(summary, dialog);
    lblNote->setStyleSheet("color: gray; font-style: italic;");
    layout->addWidget(lblNote);

    dialog->exec();
    delete dialog;
}

void MainWindow::onTab3CalibrateLaser() {
    if (m_AllLaserData.empty()) {
        QMessageBox::warning(this, "错误", "未检测到激光数据。");
        return;
    }

    if (!m_IsRectified || m_P1.empty() || m_P2.empty()) {
        QMessageBox::warning(this, "错误", "请先完成立体标定并确保投影矩阵已生成。");
        return;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    LaserCalibration lc;

    auto progressCb = [this](int current, int total) {
        statusBar()->showMessage(QString("正在处理第 %1/%2 张图片...").arg(current).arg(total));
        QApplication::processEvents();
    };

    // 使用结构体传入参数
    CalibrationParams params;

    LaserPlaneCalibrationResult result = lc.calibrateLaserPlaneWithProgress(
        m_AllLaserData, m_P1, m_P2, params, progressCb
    );

    QApplication::restoreOverrideCursor();

    if (result.success) {
        m_LaserPlaneEquation[0] = result.planeCoeffs.at<float>(0);
        m_LaserPlaneEquation[1] = result.planeCoeffs.at<float>(1);
        m_LaserPlaneEquation[2] = result.planeCoeffs.at<float>(2);
        m_LaserPlaneEquation[3] = result.planeCoeffs.at<float>(3);

        m_LaserResultStr = result.resultMessage.toStdString();
        // 从结果消息中提取RMSE
        double laserRmse = 0;
        int rmseIdx = result.resultMessage.indexOf("RMSE):");
        if (rmseIdx >= 0) {
            QString rmseStr = result.resultMessage.mid(rmseIdx + 6).trimmed();
            rmseStr = rmseStr.left(rmseStr.indexOf(" mm"));
            laserRmse = rmseStr.toDouble();
        }
        m_statusLaser->setText(QString("光平面: RMSE %1mm").arg(laserRmse, 0, 'f', 2));
        m_statusLaser->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(laserRmse < 0.5 ? Theme::SUCCESS : Theme::WARNING));
        QMessageBox::information(this, "成功", result.resultMessage);
    } else {
        m_statusLaser->setText("光平面: 失败");
        m_statusLaser->setStyleSheet(QString("color: %1; padding: 0 8px;").arg(Theme::DANGER));
        QMessageBox::warning(this, "失败", result.resultMessage);
    }

    statusBar()->showMessage("光平面标定完成", 3000);
}

void MainWindow::onTab3ShowLaserResult() {
    if (m_LaserResultStr.empty()) {
         QMessageBox::warning(this, "提示", "尚未标定。");
         return;
    }
    QDialog *dialog = new QDialog(this);

    dialog->setWindowTitle("结果");
    dialog->resize(400, 200);

    QVBoxLayout *lay = new QVBoxLayout(dialog);
    QTextEdit *txt = new QTextEdit();

    txt->setReadOnly(true);
    txt->setText(QString::fromStdString(m_LaserResultStr));
    lay->addWidget(txt);
    dialog->exec();
    delete dialog;
}
