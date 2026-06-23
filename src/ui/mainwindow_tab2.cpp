// ==================== Tab2: 相机标定 ====================
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QDialog>
#include <QDebug>
#include <QApplication>
#include <sstream>
#include <iomanip>
#include <cmath>

void MainWindow::initTab2(QTabWidget* tabWidget) {
    QWidget *tab2 = new QWidget();
    QVBoxLayout *mainLayout = new QVBoxLayout(tab2);
    mainLayout->setContentsMargins(Theme::MARGIN, Theme::MARGIN, Theme::MARGIN, Theme::MARGIN);
    QSplitter *splitter = new QSplitter(Qt::Horizontal);
    splitter->setHandleWidth(1);
    mainLayout->addWidget(splitter);

    // 1. 左侧控制面板
    QWidget *controlPanel = new QWidget();
    QVBoxLayout *controlLayout = new QVBoxLayout(controlPanel);
    controlPanel->setFixedWidth(Theme::CONTROL_PANEL_W);

    // 单独标定组
    QGroupBox *grpSingle = new QGroupBox("单独标定");
    QVBoxLayout *grpSingleLayout = new QVBoxLayout(grpSingle);
    btnLoadLeft = new QPushButton("加载标定图 (左+右)");
    btnLoadLeft->setMinimumWidth(Theme::BTN_MIN_W);
    btnCalibrate = new QPushButton("执行单独标定");
    btnCalibrate->setStyleSheet(Theme::successButton());
    grpSingleLayout->addWidget(btnLoadLeft);
    grpSingleLayout->addWidget(btnCalibrate);

    // 立体标定组
    QGroupBox *grpStereo = new QGroupBox("立体标定");
    QVBoxLayout *grpStereoLayout = new QVBoxLayout(grpStereo);
    btnSelectStereoLeft = new QPushButton("选择立体对 (左+右)");
    btnSelectStereoLeft->setMinimumWidth(Theme::BTN_MIN_W);
    QPushButton *btnCalcStereo = new QPushButton("执行全局立体标定");
    btnCalcStereo->setStyleSheet(Theme::primaryButton());
    grpStereoLayout->addWidget(btnSelectStereoLeft);

    QHBoxLayout *rotCapLayout = new QHBoxLayout();
    chkCapRotation = new QCheckBox("限制R旋转角 ≤");
    chkCapRotation->setToolTip("勾选后将stereoCalibrate算出的R的旋转角限制在指定范围内。\n相机近似平行时，标定可能算出虚假大旋转(3-5°)，限制到1-2°可保留真实微小旋转。");
    spinMaxRotationDeg = new QDoubleSpinBox();
    spinMaxRotationDeg->setRange(0.1, 10.0);
    spinMaxRotationDeg->setValue(2.0);
    spinMaxRotationDeg->setSuffix("°");
    spinMaxRotationDeg->setDecimals(1);
    spinMaxRotationDeg->setSingleStep(0.5);
    rotCapLayout->addWidget(chkCapRotation);
    rotCapLayout->addWidget(spinMaxRotationDeg);
    rotCapLayout->addStretch();
    grpStereoLayout->addLayout(rotCapLayout);

    grpStereoLayout->addWidget(btnCalcStereo);

    // 结果组
    QGroupBox *grpResult = new QGroupBox("结果显示");
    QVBoxLayout *resLayout = new QVBoxLayout(grpResult);
    btnShowLeftParams = new QPushButton("显示左相机参数");
    btnShowRightParams = new QPushButton("显示右相机参数");
    btnShowRelative = new QPushButton("显示相对位姿");
    resLayout->addWidget(btnShowLeftParams);
    resLayout->addWidget(btnShowRightParams);
    resLayout->addWidget(btnShowRelative);

    controlLayout->addWidget(grpSingle);
    controlLayout->addWidget(grpStereo);
    controlLayout->addWidget(grpResult);

    // 标定持久化
    QGroupBox *grpPersistence = new QGroupBox("标定持久化");
    QVBoxLayout *persistLayout = new QVBoxLayout(grpPersistence);
    btnSaveCalib = new QPushButton("保存全部标定");
    btnSaveCalib->setStyleSheet(Theme::successButton());
    btnLoadCalib = new QPushButton("加载全部标定");
    btnLoadCalib->setStyleSheet(Theme::primaryButton());
    connect(btnSaveCalib, &QPushButton::clicked, this, &MainWindow::onSaveCalibrationClicked);
    connect(btnLoadCalib, &QPushButton::clicked, this, &MainWindow::onLoadCalibrationClicked);
    persistLayout->addWidget(btnSaveCalib);
    persistLayout->addWidget(btnLoadCalib);
    controlLayout->addWidget(grpPersistence);

    controlLayout->addStretch();

    // 2. 中间列表区域
    QWidget *listPanel = new QWidget();
    QVBoxLayout *listPanelLayout = new QVBoxLayout(listPanel);

    listPanelLayout->addWidget(new QLabel("单独标定图片列表:"));
    listCalibration = new QListWidget();
    listCalibration->setSelectionMode(QAbstractItemView::ExtendedSelection);
    listPanelLayout->addWidget(listCalibration, 1);

    QPushButton *btnDelete = new QPushButton("批量删除选中 (上方)");
    connect(btnDelete, &QPushButton::clicked, this, &MainWindow::onDeleteSelectedClicked);
    listPanelLayout->addWidget(btnDelete);

    listPanelLayout->addWidget(new QLabel("立体对图片列表 (需一一对应):"));
    listStereoPairs = new QListWidget();
    listPanelLayout->addWidget(listStereoPairs, 1);

    // 3. 右侧显示区域
    QWidget *displayPanel = new QWidget();
    QGridLayout *displayLayout = new QGridLayout(displayPanel);

    displayOriginal = new VideoWidget();
    displayCorners = new VideoWidget();
    displayPairLeft = new VideoWidget();
    displayPairRight = new VideoWidget();

    displayOriginal->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
    displayCorners->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
    displayPairLeft->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
    displayPairRight->setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);

    displayLayout->addWidget(new QLabel("当前查看图片"), 0, 0, Qt::AlignCenter);
    displayLayout->addWidget(new QLabel("角点检测结果"), 0, 1, Qt::AlignCenter);
    displayLayout->addWidget(displayOriginal, 1, 0);
    displayLayout->addWidget(displayCorners, 1, 1);
    displayLayout->addWidget(new QLabel("立体对左图"), 2, 0, Qt::AlignCenter);
    displayLayout->addWidget(new QLabel("立体对右图"), 2, 1, Qt::AlignCenter);
    displayLayout->addWidget(displayPairLeft, 3, 0);
    displayLayout->addWidget(displayPairRight, 3, 1);
    displayLayout->setRowStretch(1, 1);
    displayLayout->setRowStretch(3, 1);

    splitter->addWidget(controlPanel);
    splitter->addWidget(listPanel);
    splitter->addWidget(displayPanel);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setStretchFactor(2, 2);
    tabWidget->addTab(tab2, "标定与参数");

    // Tab2 信号连接
    connect(btnLoadLeft, &QPushButton::clicked, this, &MainWindow::onLoadCalibImagesClicked);
    connect(btnCalibrate, &QPushButton::clicked, this, &MainWindow::onCalibrateClicked);
    connect(btnShowLeftParams, &QPushButton::clicked, this, &MainWindow::onShowLeftParamsClicked);
    connect(btnShowRightParams, &QPushButton::clicked, this, &MainWindow::onShowRightParamsClicked);

    connect(btnSelectStereoLeft, &QPushButton::clicked, this, &MainWindow::onSelectStereoPairClicked);
    connect(btnCalcStereo, &QPushButton::clicked, this, &MainWindow::onCalculateStereoParams);
    connect(btnShowRelative, &QPushButton::clicked, this, &MainWindow::onShowRelativePoseClicked);

    connect(listCalibration, &QListWidget::itemClicked, this, &MainWindow::onListItemClicked);
    connect(listStereoPairs, &QListWidget::itemClicked, this, &MainWindow::onStereoListItemClicked);
}

void MainWindow::onLoadCalibImagesClicked() {
    QStringList filesL = QFileDialog::getOpenFileNames(this, "选择左相机标定图", dirLeftCalib, "Images (*.jpg *.png *.bmp)");
    QStringList filesR = QFileDialog::getOpenFileNames(this, "选择右相机标定图", dirRightCalib, "Images (*.jpg *.png *.bmp)");
    if (filesL.isEmpty() && filesR.isEmpty()) return;

    calibratorLeft.setBoardSize(m_boardSize.width, m_boardSize.height);
    calibratorLeft.setSquareSize(m_squareSize);
    calibratorRight.setBoardSize(m_boardSize.width, m_boardSize.height);
    calibratorRight.setSquareSize(m_squareSize);

    for (const QString &file : filesL) {
        bool success = calibratorLeft.processImage(file.toStdString());
        QListWidgetItem *item = new QListWidgetItem("[左] " + QFileInfo(file).fileName());
        item->setData(Qt::UserRole, file); item->setData(Qt::UserRole + 1, 0);
        item->setForeground(success ? Qt::darkGreen : Qt::red);
        listCalibration->addItem(item);
    }
    for (const QString &file : filesR) {
        bool success = calibratorRight.processImage(file.toStdString());
        QListWidgetItem *item = new QListWidgetItem("[右] " + QFileInfo(file).fileName());
        item->setData(Qt::UserRole, file); item->setData(Qt::UserRole + 1, 1);
        item->setForeground(success ? Qt::darkGreen : Qt::red);
        listCalibration->addItem(item);
    }
}

void MainWindow::onDeleteSelectedClicked() {
    QList<QListWidgetItem*> items = listCalibration->selectedItems();
    // 倒序排序，防止删除时索引错乱
    std::sort(items.begin(), items.end(), [](QListWidgetItem *a, QListWidgetItem *b) {
        return a->listWidget()->row(a) > b->listWidget()->row(b);
    });

    // 修复: 将 Qt 的 foreach 宏替换为标准的 C++11 范围 for 循环
    for (QListWidgetItem *item : items) {
        delete listCalibration->takeItem(listCalibration->row(item));
    }
}

void MainWindow::onCalibrateClicked() {
    if (calibratorLeft.getImageItems().empty() || calibratorRight.getImageItems().empty()) {
        QMessageBox::warning(this, "错误", "请先加载图片");
        return;
    }

    // 1. 执行标定计算
    bool okL = calibratorLeft.runCalibration();
    bool okR = calibratorRight.runCalibration();

    // 2. ✅ 核心修复：将计算结果赋值给主窗口的成员变量
    if (okL) {
        m_CameraMatrixL = calibratorLeft.getCameraMatrix().clone();
        m_DistCoeffsL = calibratorLeft.getDistCoeffs().clone();
        qDebug() << "左相机内参已提取至主窗口！";
    }

    if (okR) {
        m_CameraMatrixR = calibratorRight.getCameraMatrix().clone();
        m_DistCoeffsR = calibratorRight.getDistCoeffs().clone();
        qDebug() << "右相机内参已提取至主窗口！";
    }

    // 3. 更新状态栏
    if (okL && okR) {
        double rmsL = 0, rmsR = 0;
        for (const auto& it : calibratorLeft.getImageItems()) rmsL += it.reprojError * it.reprojError;
        for (const auto& it : calibratorRight.getImageItems()) rmsR += it.reprojError * it.reprojError;
        int nL = (int)calibratorLeft.getImageItems().size();
        int nR = (int)calibratorRight.getImageItems().size();
        rmsL = nL > 0 ? std::sqrt(rmsL / nL) : 0;
        rmsR = nR > 0 ? std::sqrt(rmsR / nR) : 0;
        double rms = (rmsL + rmsR) * 0.5;
        m_statusCamCalib->setText(QString("单目: L%1 R%2px").arg(rmsL, 0, 'f', 2).arg(rmsR, 0, 'f', 2));
        m_statusCamCalib->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(rms < 0.5 ? Theme::SUCCESS : Theme::WARNING));
        QMessageBox::information(this, "完成", "单独标定完成！");
    } else {
        QString errMsg;
        if (!okL) errMsg += "左相机标定失败！\n";
        if (!okR) errMsg += "右相机标定失败！";
        QMessageBox::warning(this, "警告", errMsg);
    }
}

void MainWindow::onShowLeftParamsClicked() {
    QDialog *dialog = new QDialog(this); dialog->setWindowTitle("左相机参数"); dialog->resize(600, 500);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    QVBoxLayout *layout = new QVBoxLayout(dialog); QTextEdit *text = new QTextEdit(); text->setReadOnly(true);
    text->setText(QString::fromStdString(calibratorLeft.getResultString())); layout->addWidget(text); dialog->exec();
}

void MainWindow::onShowRightParamsClicked() {
    QDialog *dialog = new QDialog(this); dialog->setWindowTitle("右相机参数"); dialog->resize(600, 500);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    QVBoxLayout *layout = new QVBoxLayout(dialog); QTextEdit *text = new QTextEdit(); text->setReadOnly(true);
    text->setText(QString::fromStdString(calibratorRight.getResultString())); layout->addWidget(text); dialog->exec();
}

void MainWindow::onListItemClicked(QListWidgetItem *item) {
    if (!item) return;
    QString filePath = item->data(Qt::UserRole).toString();
    int side = item->data(Qt::UserRole + 1).toInt();
    CameraCalibration* currentCalib = (side == 0) ? &calibratorLeft : &calibratorRight;
    const auto& items = currentCalib->getImageItems();
    const CalibImageItem* targetItem = nullptr;
    for(const auto& imgItem : items) {
        if (imgItem.filePath == filePath.toStdString()) {
            targetItem = &imgItem;
            break;
        }
    }
    if (!targetItem) return;
    cv::Mat originalImg = cv::imread(targetItem->filePath);
    if (originalImg.empty()) return;
    displayOriginal->setFrame(mat2QImage(originalImg));
    if (targetItem->found && !targetItem->corners.empty()) {
        cv::Mat cornerImg = originalImg.clone();
        cv::drawChessboardCorners(cornerImg, currentCalib->getBoardSize(), targetItem->corners, true);
        displayCorners->setFrame(mat2QImage(cornerImg));
    } else {
        displayCorners->setFrame(mat2QImage(originalImg));
    }
}

// --- 立体对选择逻辑 ---

void MainWindow::onSelectStereoPairClicked() {
    QStringList filesL = QFileDialog::getOpenFileNames(this, "选择立体对左图", dirLeftCalib, "Images (*.jpg *.png *.bmp)");
    if (filesL.isEmpty()) return;
    QStringList filesR = QFileDialog::getOpenFileNames(this, "选择立体对右图", dirRightCalib, "Images (*.jpg *.png *.bmp)");
    if (filesR.isEmpty()) return;

    m_StereoFilesL = filesL;
    m_StereoFilesR = filesR;
    listStereoPairs->clear();
    int count = std::min(m_StereoFilesL.size(), m_StereoFilesR.size());
    for (int i = 0; i < count; ++i) {
        listStereoPairs->addItem(QString("Pair %1: %2 <-> %3")
            .arg(i+1).arg(QFileInfo(m_StereoFilesL[i]).fileName()).arg(QFileInfo(m_StereoFilesR[i]).fileName()));
    }
    displayPairLeft->setFrame(mat2QImage(cv::imread(filesL.last().toStdString())));
    displayPairRight->setFrame(mat2QImage(cv::imread(filesR.last().toStdString())));
}

void MainWindow::onStereoListItemClicked(QListWidgetItem *item) {
    int idx = listStereoPairs->row(item);
    if (idx >= 0 && idx < m_StereoFilesL.size() && idx < m_StereoFilesR.size()) {
        displayPairLeft->setFrame(mat2QImage(cv::imread(m_StereoFilesL[idx].toStdString())));
        displayPairRight->setFrame(mat2QImage(cv::imread(m_StereoFilesR[idx].toStdString())));
    }
}

void MainWindow::onCalculateStereoParams() {
    // 1. 检查单目标定是否完成
    if (calibratorLeft.getCameraMatrix().empty() || calibratorRight.getCameraMatrix().empty()) {
        QMessageBox::warning(this, "错误", "请先完成左右相机的单独标定。");
        return;
    }

    // 2. 获取已处理的数据
    const auto& leftItems = calibratorLeft.getImageItems();
    const auto& rightItems = calibratorRight.getImageItems();

    std::vector<std::vector<cv::Point3f>> allObjectPoints;
    std::vector<std::vector<cv::Point2f>> allCornersL, allCornersR;

    int validPairs = 0;
    int count = std::min(leftItems.size(), rightItems.size());

    for (int i = 0; i < count; ++i) {
        if (leftItems[i].found && rightItems[i].found) {
            allCornersL.push_back(leftItems[i].corners);
            allCornersR.push_back(rightItems[i].corners);

            std::vector<cv::Point3f> objCorners;
            cv::Size boardSize = calibratorLeft.getBoardSize();
            float squareSize = calibratorLeft.getSquareSize();
            for (int r = 0; r < boardSize.height; ++r)
                for (int c = 0; c < boardSize.width; ++c)
                    objCorners.push_back(cv::Point3f(c * squareSize, r * squareSize, 0));

            allObjectPoints.push_back(objCorners);
            validPairs++;
        }
    }

    if (validPairs < 3) {
        QMessageBox::warning(this, "错误", "有效的立体匹配图片对少于3对，请确保左右图片数量一致且均检测成功。");
        return;
    }

    // 3. 准备参数
    cv::Size imageSize = calibratorLeft.getImageSize();
    cv::Mat K1 = calibratorLeft.getCameraMatrix(), D1 = calibratorLeft.getDistCoeffs();
    cv::Mat K2 = calibratorRight.getCameraMatrix(), D2 = calibratorRight.getDistCoeffs();

    // 4. 执行立体标定
    int flags = cv::CALIB_FIX_INTRINSIC;

    QApplication::setOverrideCursor(Qt::WaitCursor);

    double rms = cv::stereoCalibrate(
        allObjectPoints, allCornersL, allCornersR,
        K1, D1, K2, D2, imageSize,
        m_R, m_T, m_E, m_F,
        flags,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5)
    );
    m_StereoRms = rms;

    // 5. 立体校正 (Stereo Rectify)
    // 若启用旋转角限制，将R的旋转角裁剪到上限内（直接修改m_R，下游自动生效）
    if (chkCapRotation->isChecked()) {
        cv::Mat rvec;
        cv::Rodrigues(m_R, rvec);
        double angle_deg = cv::norm(rvec) * 180.0 / CV_PI;
        double max_deg = spinMaxRotationDeg->value();
        if (angle_deg > max_deg) {
            rvec *= (max_deg / angle_deg);
            cv::Rodrigues(rvec, m_R);
        }
        std::stringstream ssR;
        ssR << "[R限制] 原旋转角=" << std::fixed << std::setprecision(2) << angle_deg
            << "° → 限制到≤" << max_deg << "°"
            << (angle_deg > max_deg ? " (已裁剪)" : " (未超限)")
            << " | 基线=" << cv::norm(m_T) << " mm";
        qDebug() << QString::fromStdString(ssR.str());
    }
    cv::stereoRectify(K1, D1, K2, D2, imageSize, m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q, cv::CALIB_ZERO_DISPARITY, 0, imageSize);

    // 6. 计算映射表
    cv::initUndistortRectifyMap(K1, D1, m_R1, m_P1, imageSize, CV_16SC2, m_MapL1, m_MapL2);
    cv::initUndistortRectifyMap(K2, D2, m_R2, m_P2, imageSize, CV_16SC2, m_MapR1, m_MapR2);
    m_IsRectified = true;
    m_statusStereo->setText(QString("立体: RMS %1px").arg(m_StereoRms, 0, 'f', 3));
    m_statusStereo->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(m_StereoRms < 0.5 ? Theme::SUCCESS : Theme::WARNING));

    QApplication::restoreOverrideCursor();

    // 7. 逐对极线误差 (存储到成员变量, 在"显示相对位姿"弹窗查看)
    m_perPairEpiErrors.clear();
    double total_epi_err = 0;
    for (int i = 0; i < validPairs; ++i) {
        std::vector<cv::Point2f> rectL, rectR;
        cv::undistortPoints(allCornersL[i], rectL, K1, D1, m_R1, m_P1);
        cv::undistortPoints(allCornersR[i], rectR, K2, D2, m_R2, m_P2);
        double pair_err = 0;
        int n = std::min(rectL.size(), rectR.size());
        for (int j = 0; j < n; ++j)
            pair_err += std::fabs(rectL[j].y - rectR[j].y);
        pair_err /= n;
        total_epi_err += pair_err;
        m_perPairEpiErrors.push_back(pair_err);
    }
    double avgEpiErr = total_epi_err / validPairs;
    int badCount = 0;
    for (double e : m_perPairEpiErrors) if (e > 1.0) ++badCount;

    std::stringstream ss;
    ss << "立体标定完成!\n";
    ss << "有效图片对: " << validPairs << "\n";
    ss << "RMS 重投影误差: " << rms << " 像素\n";
    ss << "基线距离: " << cv::norm(m_T) << " 单位\n";
    ss << "平均极线Y误差: " << std::fixed << std::setprecision(2) << avgEpiErr << "px";
    if (badCount > 0)
        ss << "  (" << badCount << "/" << validPairs << "对 >1px)";
    ss << "\n\n点击「显示相对位姿」查看逐对误差详情";
    QMessageBox::information(this, "完成", QString::fromStdString(ss.str()));
}

void MainWindow::onShowRelativePoseClicked() {
    if (!m_IsRectified) { QMessageBox::warning(this, "提示", "请先完成立体标定。"); return; }
    QDialog *dialog = new QDialog(this); dialog->setWindowTitle("立体标定结果"); dialog->resize(600, 700);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    QVBoxLayout *layout = new QVBoxLayout(dialog);
    QTextEdit *text = new QTextEdit(); text->setReadOnly(true);
    text->setStyleSheet("font-family: Consolas, monospace; font-size: 13px;");
    std::stringstream ss;
    ss << "========== 立体标定结果 ==========\n\n";
    ss << "RMS 重投影误差: " << m_StereoRms << " 像素\n";
    ss << "基线距离: " << cv::norm(m_T) << " mm\n";
    ss << "有效图片对: " << m_perPairEpiErrors.size() << "\n\n";
    ss << "R (旋转矩阵):\n" << m_R << "\n";
    ss << "T (平移向量):\n" << m_T << "\n\n";
    ss << "P1 (左校正投影矩阵):\n" << m_P1 << "\n";
    ss << "P2 (右校正投影矩阵):\n" << m_P2 << "\n\n";
    ss << "K1 (左相机内参):\n" << m_CameraMatrixL << "\n";
    ss << "K2 (右相机内参):\n" << m_CameraMatrixR << "\n\n";

    // 逐对极线Y误差
    if (!m_perPairEpiErrors.empty()) {
        double total = 0;
        int bad = 0;
        ss << "===== 校正后逐对极线Y误差 (应<1px) =====\n";
        ss << std::fixed << std::setprecision(2);
        for (size_t i = 0; i < m_perPairEpiErrors.size(); ++i) {
            double e = m_perPairEpiErrors[i];
            total += e;
            if (e > 1.0) ++bad;
            ss << "  Pair " << std::setw(3) << (i + 1) << ": " << e << "px"
               << (e > 5 ? " ❌" : e > 1 ? " ⚠️" : " ✅") << "\n";
        }
        ss << "\n平均极线Y误差: " << (total / m_perPairEpiErrors.size()) << "px";
        if (bad > 0) ss << "  (" << bad << "/" << m_perPairEpiErrors.size() << "对 >1px)";
        ss << "\n";
    }
    text->setText(QString::fromStdString(ss.str()));
    layout->addWidget(text);
    dialog->exec();
}

// ==================== 标定持久化 ====================

void MainWindow::onSaveCalibrationClicked()
{
    if (m_CameraMatrixL.empty() || m_CameraMatrixR.empty()) {
        QMessageBox::warning(this, "保存失败", "请先完成单目标定和立体标定！");
        return;
    }
    if (saveCalibration()) {
        QMessageBox::information(this, "保存成功",
            "全部标定参数已保存到 Resources/calibration/calib_data.yml\n"
            "下次启动时将自动加载。");
    }
}

void MainWindow::onLoadCalibrationClicked()
{
    if (loadCalibration()) {
        QMessageBox::information(this, "加载成功",
            QString("标定参数已从 calib_data.yml 恢复。\n"
                    "立体标定 RMS: %1 px\n基线: %2 mm")
            .arg(m_StereoRms, 0, 'f', 4).arg(cv::norm(m_T), 0, 'f', 2));
    }
}
