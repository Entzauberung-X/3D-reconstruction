// ==================== Tab4: 转台标定 & 诊断工具 ====================
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>
#include <QDialog>
#include <QTextEdit>
#include <QScrollArea>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>
#include <QProgressBar>
#include <QEventLoop>
#include <QDebug>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm> // 【补全】std::min (旋转序列/角点数量截取)

void MainWindow::initTab4(QTabWidget* tabWidget) {
    QWidget *page = new QWidget();
    page->setObjectName("转台");
    QHBoxLayout *tab4MainLayout = new QHBoxLayout(page);

    // 左侧控制面板 (单列布局，宽度确保中文按钮不截断)
    QScrollArea *tab4ScrollArea = new QScrollArea();
    tab4ScrollArea->setFixedWidth(320);
    tab4ScrollArea->setWidgetResizable(true);
    tab4ScrollArea->setStyleSheet("QScrollArea { border: none; }");

    QWidget *tab4CtrlPanel = new QWidget();
    QVBoxLayout *tab4CtrlLayout = new QVBoxLayout(tab4CtrlPanel);
    tab4CtrlPanel->setFixedWidth(300);

    // ===== 组1: 旋转轴自动标定 =====
    QGroupBox *grpAxisCalib = new QGroupBox("旋转轴自动标定");
    QVBoxLayout *grpAxisCalibLayout = new QVBoxLayout(grpAxisCalib);

    btnLoadRotSeq = new QPushButton("1. 加载旋转序列");
    btnLoadRotSeq->setStyleSheet(Theme::boldButton());
    grpAxisCalibLayout->addWidget(btnLoadRotSeq);

    btnExecAxisCalib = new QPushButton("2. 执行轴标定");
    btnExecAxisCalib->setStyleSheet(Theme::warningButton());
    btnExecAxisCalib->setEnabled(false);
    grpAxisCalibLayout->addWidget(btnExecAxisCalib);

    // ---- 算法参数设置 (写法与 Tab5 的参数面板一致) ----
    QFormLayout *formAxisAlgo = new QFormLayout();
    formAxisAlgo->setLabelAlignment(Qt::AlignLeft);
    formAxisAlgo->setContentsMargins(0, 0, 0, 0);

    comboAxisMethod = new QComboBox();
    comboAxisMethod->addItem("自动");       // index 0 == RotatingCalibrator::AxisMethod::Auto
    comboAxisMethod->addItem("圆拟合");     // index 1 == CircleFit
    comboAxisMethod->addItem("BA");         // index 2 == BA
    comboAxisMethod->setCurrentIndex(0);
    comboAxisMethod->setToolTip(
        "自动: BA 可用且重投影 ≤ 阈值时用 BA, 否则回退圆拟合\n"
        "圆拟合: 强制用 3D 圆拟合 (对 PnP 噪声不敏感, 帧少也稳)\n"
        "BA: 强制用 PnP+LM/Huber (帧多时精度可能更高)\n"
        "强制的那一路若失败会自动回退到另一路, 不会让标定失败");
    formAxisAlgo->addRow("轴估计方式:", comboAxisMethod);

    spinBaErrorLimit = new QDoubleSpinBox();
    spinBaErrorLimit->setRange(0.05, 20.0);
    spinBaErrorLimit->setValue(1.0);        // 健康数据 BA ~0.2px, 留 5 倍余量
    spinBaErrorLimit->setSingleStep(0.1);
    spinBaErrorLimit->setDecimals(2);
    spinBaErrorLimit->setSuffix(" px");
    spinBaErrorLimit->setToolTip(
        "仅「自动」模式生效: BA 重投影误差超过此值就回退圆拟合。\n"
        "实测健康数据 BA 在 0.2px 量级; 旧版硬编码 20px 过宽, 会放过明显没拟合好的 BA。");
    formAxisAlgo->addRow("BA 误差上限:", spinBaErrorLimit);

    grpAxisCalibLayout->addLayout(formAxisAlgo);

    // 标定进度条 (控件样式与 Tab5 重建进度条保持一致)
    progressAxisCalib = new QProgressBar();
    progressAxisCalib->setRange(0, 100);
    progressAxisCalib->setValue(0);
    progressAxisCalib->setFormat("待机");
    progressAxisCalib->setToolTip("转台标定进度: 提取角点 → 位姿消歧 → 3D 圆拟合 → BA 优化");
    grpAxisCalibLayout->addWidget(progressAxisCalib);

    txtAxisCalibResult = new QTextEdit();
    txtAxisCalibResult->setReadOnly(true); txtAxisCalibResult->setFont(QFont(Theme::MONO, 9));
    txtAxisCalibResult->setMaximumHeight(180);
    txtAxisCalibResult->setStyleSheet(Theme::terminalStyle());
    grpAxisCalibLayout->addWidget(txtAxisCalibResult);

    lblAxisCamFrame = new QLabel("轴(相机系): 未标定");
    lblAxisCamFrame->setStyleSheet(QString("color: %1; font-family: %2; font-size: 11px; padding: 4px;").arg(Theme::TEXT_SECONDARY, Theme::MONO));
    lblAxisCamFrame->setWordWrap(true);
    grpAxisCalibLayout->addWidget(lblAxisCamFrame);
    tab4CtrlLayout->addWidget(grpAxisCalib);

    // ===== 组2: SIFT 特征点测距 =====
    QGroupBox *grpSift = new QGroupBox("SIFT 特征点测距");
    QVBoxLayout *grpSiftLayout = new QVBoxLayout(grpSift);
    btnLoadSiftImages = new QPushButton("1. 打开左右测试图");
    btnLoadSiftImages->setStyleSheet(Theme::boldButton());
    btnExecSiftMeasure = new QPushButton("2. 执行 SIFT 测距");
    btnExecSiftMeasure->setStyleSheet(Theme::successButton());
    btnSelectSiftRoiLeft = new QPushButton("3.1 左图ROI框选");
    btnSelectSiftRoiRight = new QPushButton("3.2 右图ROI框选");

    grpSiftLayout->addWidget(btnLoadSiftImages);
    grpSiftLayout->addWidget(btnExecSiftMeasure);
    grpSiftLayout->addWidget(btnSelectSiftRoiLeft);
    grpSiftLayout->addWidget(btnSelectSiftRoiRight);

    tab4CtrlLayout->addWidget(grpSift);
    tab4CtrlLayout->addStretch();

    tab4ScrollArea->setWidget(tab4CtrlPanel);

    // 右侧显示面板
    QWidget *tab4DispPanel = new QWidget();
    QVBoxLayout *tab4DispLayout = new QVBoxLayout(tab4DispPanel);
    tab4DispLayout->setContentsMargins(6, 6, 6, 6);
    QLabel *lblSiftTitle = new QLabel("特征点匹配结果预览");
    lblSiftTitle->setStyleSheet("font-weight: bold; font-size: 13px;");
    tab4DispLayout->addWidget(lblSiftTitle);

    QHBoxLayout *imgLayout = new QHBoxLayout();
    lblSiftLeftImg = new QLabel(); lblSiftLeftImg->setStyleSheet(Theme::imageLabelStyle(300, 225)); lblSiftLeftImg->setMinimumSize(300, 225); lblSiftLeftImg->setAlignment(Qt::AlignCenter); lblSiftLeftImg->setText("左图");
    lblSiftRightImg = new QLabel(); lblSiftRightImg->setStyleSheet(Theme::imageLabelStyle(300, 225)); lblSiftRightImg->setMinimumSize(300, 225); lblSiftRightImg->setAlignment(Qt::AlignCenter); lblSiftRightImg->setText("右图");
    imgLayout->addWidget(lblSiftLeftImg); imgLayout->addWidget(lblSiftRightImg);
    tab4DispLayout->addLayout(imgLayout);

    txtSiftResult = new QTextEdit();
    txtSiftResult->setReadOnly(true); txtSiftResult->setFont(QFont(Theme::MONO, 10)); txtSiftResult->setMaximumHeight(200);
    txtSiftResult->setStyleSheet(Theme::terminalStyle());
    tab4DispLayout->addWidget(txtSiftResult);
    tab4DispLayout->addStretch();

    tab4MainLayout->addWidget(tab4ScrollArea); tab4MainLayout->addWidget(tab4DispPanel, 1);
    addPage(tabWidget, page, "04 转台标定", "04", "转台标定",
            "旋转轴方向/轴点与基准位姿 (PnP → 3D 圆拟合 / BA)，另含 SIFT 特征测距辅助验证");

    m_rotatingCalibrator = new Calib::RotatingCalibrator();
    m_pointCalibrator = new PointCalibrator();
    connect(btnLoadRotSeq, &QPushButton::clicked, this, &MainWindow::onLoadRotSeqClicked);
    connect(btnExecAxisCalib, &QPushButton::clicked, this, &MainWindow::onExecAxisCalibClicked);
    // 诊断类按钮的连接在 initTab6() 中建立
    connect(btnLoadSiftImages, &QPushButton::clicked, this, &MainWindow::onLoadSiftImagesClicked);
    connect(btnExecSiftMeasure, &QPushButton::clicked, this, &MainWindow::onExecSiftMeasureClicked);
    connect(btnSelectSiftRoiLeft, &QPushButton::clicked, this, &MainWindow::onSelectSiftRoiLeftClicked);
    connect(btnSelectSiftRoiRight, &QPushButton::clicked, this, &MainWindow::onSelectSiftRoiRightClicked);
}

void MainWindow::onLoadRotSeqClicked()
{
    // 选择包含旋转序列图像的文件夹 (左右分开选)
    QString dirL = QFileDialog::getExistingDirectory(this, "选择左相机旋转序列文件夹", dirLeftPlatform);
    if (dirL.isEmpty()) return;
    QString dirR = QFileDialog::getExistingDirectory(this, "选择右相机旋转序列文件夹", dirRightPlatform);
    if (dirR.isEmpty()) return;

    m_rotSeqLeftPaths = getFilesInFolder(dirL, 0);
    m_rotSeqRightPaths = getFilesInFolder(dirR, 0);

    if (m_rotSeqLeftPaths.isEmpty() || m_rotSeqRightPaths.isEmpty()) {
        QMessageBox::warning(this, "错误", "未在所选文件夹中找到图像文件！");
        return;
    }

    int count = qMin(m_rotSeqLeftPaths.size(), m_rotSeqRightPaths.size());
    txtAxisCalibResult->clear();
    txtAxisCalibResult->append(QString(">>> 已加载旋转序列: %1 对图像").arg(count));
    txtAxisCalibResult->append(QString("    左: %1").arg(dirL));
    txtAxisCalibResult->append(QString("    右: %1").arg(dirR));
    txtAxisCalibResult->append(">>> 点击\"执行轴标定\"开始自动标定");

    btnExecAxisCalib->setEnabled(true);
    // 诊断只依赖"旋转序列 + 相机参数"，因此加载序列后即可运行
    // (标定失败时恰恰最需要诊断，不能因为标定没成功就把诊断入口灰掉)
    btnAxisDiagRun->setEnabled(!(m_CameraMatrixL.empty() || m_CameraMatrixR.empty() || m_R.empty()));
    btnMultiFrame3D->setEnabled(false);
    btnVerify3D->setEnabled(false);
}

void MainWindow::onExecAxisCalibClicked()
{
    // 检查前置条件
    if (m_CameraMatrixL.empty() || m_CameraMatrixR.empty() || m_R.empty()) {
        QMessageBox::warning(this, "缺少参数", "请先在 Tab2 完成双目相机标定和立体标定！");
        return;
    }

    if (m_rotSeqLeftPaths.isEmpty()) {
        QMessageBox::warning(this, "缺少数据", "请先加载旋转序列图像！");
        return;
    }

    int count = qMin(m_rotSeqLeftPaths.size(), m_rotSeqRightPaths.size());
    if (count < 3) {
        QMessageBox::warning(this, "数据不足", "至少需要3对旋转序列图像！");
        return;
    }

    btnExecAxisCalib->setEnabled(false);
    btnLoadRotSeq->setEnabled(false);
    btnAxisDiagRun->setEnabled(false);
    comboAxisMethod->setEnabled(false);      // 标定期间锁定算法参数
    spinBaErrorLimit->setEnabled(false);
    btnMultiFrame3D->setEnabled(false);
    btnVerify3D->setEnabled(false);
    btnMultiFrameStats->setEnabled(false);
    btnMultiFrame3DView->setEnabled(false);
    txtAxisCalibResult->append(">>> 开始旋转轴自动标定...");
    txtAxisCalibResult->append(QString("    标定板: %1 x %2, 方格 %3 mm")
        .arg(m_boardSize.width).arg(m_boardSize.height).arg(m_squareSize));

    // 进度条复位 + 绑定算法层进度回调 (process() 为同步调用, 回调在 GUI 线程触发)
    progressAxisCalib->setRange(0, 100);
    progressAxisCalib->setValue(0);
    progressAxisCalib->setFormat("准备中 0%");
    m_rotatingCalibrator->setProgressFn([this](const QString& stage, int percent) {
        progressAxisCalib->setValue(percent);
        progressAxisCalib->setFormat(QString("%1  %2%").arg(stage).arg(percent));
        // 只刷新界面绘制、不放行用户输入, 避免同步标定期间重入点击/菜单触发
        QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    });
    QApplication::processEvents();

    // 配置标定器 (含「算法参数设置」)
    m_rotatingCalibrator->setAxisMethod(
        static_cast<Calib::RotatingCalibrator::AxisMethod>(comboAxisMethod->currentIndex()));
    m_rotatingCalibrator->setBaErrorLimitPx(spinBaErrorLimit->value());
    m_rotatingCalibrator->setCameraParams(
        m_CameraMatrixL, m_DistCoeffsL,
        m_CameraMatrixR, m_DistCoeffsR,
        m_R, m_T
    );
    m_rotatingCalibrator->setPatternParams(
        cv::Size(m_boardSize.width, m_boardSize.height),
        static_cast<float>(m_squareSize)
    );
    m_rotatingCalibrator->setInputData(m_rotSeqLeftPaths, m_rotSeqRightPaths);

    // 执行标定
    bool ok = m_rotatingCalibrator->process();

    // 解除回调 (避免 Tab6 诊断流程误刷本页进度条) 并收尾进度显示
    m_rotatingCalibrator->setProgressFn(nullptr);
    if (ok) { progressAxisCalib->setValue(100); progressAxisCalib->setFormat("标定完成 100%"); }
    else    { progressAxisCalib->setValue(0);   progressAxisCalib->setFormat("标定失败"); }

    if (ok) {
        m_rotAxisDirection = m_rotatingCalibrator->getAxisDirection();
        m_rotAxisPoint = m_rotatingCalibrator->getAxisPoint();
        m_rotatingCalibrator->getBasePose(m_R_base, m_T_base);
        double err = m_rotatingCalibrator->getReprojectionError();
        // 【修复】错误量纲随所采用的方案变化: BA=px, 圆拟合=mm。
        //   旧版一律显示 "px"，把毫米级的圆拟合残差当成几十像素的重投影误差，
        //   让用户误以为标定失败 (转台标定最典型的"假故障")。
        const char* unit = m_rotatingCalibrator->getErrorUnit();
        const QString source = (QString(unit) == "px") ? "BA (PnP+LM)" : "3D 圆拟合";

        txtAxisCalibResult->append(QString("✅ 标定成功！采用方案: %1, 误差: %2 %3")
            .arg(source).arg(err, 0, 'f', 3).arg(unit));
        // 算法设置 + 两条通道各自的成绩, 便于判断这次为什么选了它
        txtAxisCalibResult->append(QString("    算法设置: %1 (BA 阈值 %2px)")
            .arg(comboAxisMethod->currentText())
            .arg(spinBaErrorLimit->value(), 0, 'f', 2));
        txtAxisCalibResult->append(QString("    圆拟合残差(分相机) 左 %1mm / 右 %2mm | BA重投影 %3px | 轴向高差 %4mm")
            .arg(m_rotatingCalibrator->getCircleResidualLeftMm(), 0, 'f', 3)
            .arg(m_rotatingCalibrator->getCircleResidualRightMm(), 0, 'f', 3)
            .arg(m_rotatingCalibrator->getBaErrorPx(), 0, 'f', 3)
            .arg(m_rotatingCalibrator->getAxialGapMm(), 0, 'f', 2));
        txtAxisCalibResult->append(QString(">>> 轴方向(转台基准系): [%1, %2, %3]")
            .arg(m_rotAxisDirection.at<double>(0), 0, 'f', 4)
            .arg(m_rotAxisDirection.at<double>(1), 0, 'f', 4)
            .arg(m_rotAxisDirection.at<double>(2), 0, 'f', 4));
        txtAxisCalibResult->append(QString(">>> 轴点坐标(转台基准系): [%1, %2, %3] mm")
            .arg(m_rotAxisPoint.at<double>(0), 0, 'f', 2)
            .arg(m_rotAxisPoint.at<double>(1), 0, 'f', 2)
            .arg(m_rotAxisPoint.at<double>(2), 0, 'f', 2));
        txtAxisCalibResult->append(">>> 结果已自动保存，三维重建时将使用精确轴参数");

        // 轴在相机坐标系下的表示 (用于物理验证)
        if (!m_R_base.empty() && !m_T_base.empty()) {
            cv::Mat axis_world = m_rotAxisDirection.clone();
            cv::Mat axis_cam = m_R_base * axis_world; // 方向: R_base * a_world
            txtAxisCalibResult->append(QString(">>> 轴方向(相机系): [%1, %2, %3]")
                .arg(axis_cam.at<double>(0), 0, 'f', 4)
                .arg(axis_cam.at<double>(1), 0, 'f', 4)
                .arg(axis_cam.at<double>(2), 0, 'f', 4));
            cv::Mat q_world = m_rotAxisPoint.clone();
            cv::Mat q_cam = m_R_base * q_world + m_T_base; // 点: R_base * Q + T_base
            txtAxisCalibResult->append(QString(">>> 轴点(相机系): [%1, %2, %3] mm")
                .arg(q_cam.at<double>(0), 0, 'f', 2)
                .arg(q_cam.at<double>(1), 0, 'f', 2)
                .arg(q_cam.at<double>(2), 0, 'f', 2));

            lblAxisCamFrame->setText(QString("轴(相机系): 方向[%1,%2,%3] 点[%4,%5,%6]mm")
                .arg(axis_cam.at<double>(0),0,'f',3).arg(axis_cam.at<double>(1),0,'f',3).arg(axis_cam.at<double>(2),0,'f',3)
                .arg(q_cam.at<double>(0),0,'f',1).arg(q_cam.at<double>(1),0,'f',1).arg(q_cam.at<double>(2),0,'f',1));
            lblAxisCamFrame->setStyleSheet(QString("color: %1; font-family: %2; font-size: 11px; padding: 4px;").arg(Theme::ACCENT, Theme::MONO));
        }

        // 启用逐帧调试按钮
        btnAxisDiagRun->setEnabled(true);
        btnVerify3D->setEnabled(true);
        btnMultiFrame3D->setEnabled(true);

        // 同步刷新「调试与诊断」页的前置条件与标定质量指标条
        updateDebugStatus();
        appendDebugLog(QString("[标定] 旋转轴标定完成 (方案 %1, 误差 %2 %3) — "
                               "可到「调试与诊断」页做转台标定诊断/棋盘格 3D 验证")
                           .arg(source).arg(err, 0, 'f', 3).arg(unit));

        m_statusAxis->setText(QString("转台: %1 %2%3").arg(err, 0, 'f', 2).arg(unit).arg(source.left(2)));
        m_statusAxis->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;")
            .arg((QString(unit) == "px" ? err < 0.5 : err < 1.0) ? Theme::SUCCESS : Theme::WARNING));
        QMessageBox::information(this, "轴标定完成",
            QString("旋转轴标定成功！\n\n"
                    "采用方案: %1\n"
                    "误差: %2 %3\n"
                    "圆拟合残差(左/右): %4 / %5 mm\n"
                    "BA 重投影误差: %6 px\n"
                    "左右光心轴向高差: %7 mm\n"
                    "轴方向(转台基准系): [%8, %9, %10]\n"
                    "轴点(转台基准系): [%11, %12, %13] mm\n\n"
                    "提示: 到「调试与诊断」页可做完整转台标定诊断并导出调试包。")
                .arg(source).arg(err, 0, 'f', 3).arg(unit)
                .arg(m_rotatingCalibrator->getCircleResidualLeftMm(), 0, 'f', 3)
                .arg(m_rotatingCalibrator->getCircleResidualRightMm(), 0, 'f', 3)
                .arg(m_rotatingCalibrator->getBaErrorPx(), 0, 'f', 3)
                .arg(m_rotatingCalibrator->getAxialGapMm(), 0, 'f', 2)
                .arg(m_rotAxisDirection.at<double>(0), 0, 'f', 4)
                .arg(m_rotAxisDirection.at<double>(1), 0, 'f', 4)
                .arg(m_rotAxisDirection.at<double>(2), 0, 'f', 4)
                .arg(m_rotAxisPoint.at<double>(0), 0, 'f', 2)
                .arg(m_rotAxisPoint.at<double>(1), 0, 'f', 2)
                .arg(m_rotAxisPoint.at<double>(2), 0, 'f', 2));

    } else {
        m_statusAxis->setText("转台: 失败");
        m_statusAxis->setStyleSheet(QString("color: %1; padding: 0 8px;").arg(Theme::DANGER));
        txtAxisCalibResult->append("   1. 标定板角点数是否正确");
        txtAxisCalibResult->append("   2. 所有旋转角度下标定板是否都可完整检测");
        txtAxisCalibResult->append("   3. 相机内外参是否已正确标定");
    }

    btnExecAxisCalib->setEnabled(true);
    btnLoadRotSeq->setEnabled(true);
    comboAxisMethod->setEnabled(true);
    spinBaErrorLimit->setEnabled(true);
}

// ================= Tab 4: SIFT 测距 槽函数 =================
void MainWindow::onLoadSiftImagesClicked() {
    QString pathL = QFileDialog::getOpenFileName(this, "选择左图", dirLeftPointCloud, "Images (*.jpg *.jpeg *.png *.bmp);;All Files (*)");
    if(pathL.isEmpty()) return;
    QString pathR = QFileDialog::getOpenFileName(this, "选择右图", dirRightPointCloud, "Images (*.jpg *.jpeg *.png *.bmp);;All Files (*)");
    if(pathR.isEmpty()) return;
    m_siftLeftPath = pathL;
    m_siftRightPath = pathR;
    cv::Mat matL = cv::imread(pathL.toStdString());
    cv::Mat matR = cv::imread(pathR.toStdString());
    if(matL.empty()) {
        txtSiftResult->append(QString(">>> 错误：无法打开左图 %1").arg(pathL));
        return;
    }
    if(matR.empty()) {
        txtSiftResult->append(QString(">>> 错误：无法打开右图 %1").arg(pathR));
        return;
    }
    lblSiftLeftImg->setPixmap(QPixmap::fromImage(mat2QImage(matL)).scaled(lblSiftLeftImg->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    lblSiftRightImg->setPixmap(QPixmap::fromImage(mat2QImage(matR)).scaled(lblSiftRightImg->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    txtSiftResult->append(">>> 成功加载左右测试图。");

    m_siftRoiLeft = cv::Rect();
    m_siftRoiRight = cv::Rect();
    if(btnSelectSiftRoiLeft) btnSelectSiftRoiLeft->setText("3. 左图ROI框选");
    if(btnSelectSiftRoiRight) btnSelectSiftRoiRight->setText("4. 右图ROI框选");
}

void MainWindow::onSelectSiftRoiLeftClicked() {
    if (m_siftLeftPath.isEmpty()) { QMessageBox::warning(this, "提示", "请先加载左图！"); return; }
    if (selectRoiOnImage(m_siftLeftPath, "框选左图 SIFT ROI", m_siftRoiLeft, btnSelectSiftRoiLeft))
        txtSiftResult->append(">>> 左图 ROI 已设置。");
}

void MainWindow::onSelectSiftRoiRightClicked() {
    if (m_siftRightPath.isEmpty()) { QMessageBox::warning(this, "提示", "请先加载右图！"); return; }
    if (selectRoiOnImage(m_siftRightPath, "框选右图 SIFT ROI", m_siftRoiRight, btnSelectSiftRoiRight))
        txtSiftResult->append(">>> 右图 ROI 已设置。");
}

void MainWindow::onExecSiftMeasureClicked() {
    if(m_siftLeftPath.isEmpty() || m_siftRightPath.isEmpty()) {
        QMessageBox::warning(this, "错误", "请先加载图片！");
        return;
    }
    if(m_CameraMatrixL.empty() || m_CameraMatrixR.empty()) {
        QMessageBox::warning(this, "错误", "请先完成双目标定！");
        return;
    }

    // 1. 读取原始图像 (用于最后画图显示)
    cv::Mat imgL = cv::imread(m_siftLeftPath.toStdString());
    cv::Mat imgR = cv::imread(m_siftRightPath.toStdString());
    if (imgL.empty() || imgR.empty()) {
        txtSiftResult->append(QString(">>> 错误：无法读取图片 %1 或 %2").arg(m_siftLeftPath, m_siftRightPath));
        return;
    }

    // 2. 复制一份用于算法处理 (在ROI外置黑，欺骗SIFT只提取ROI内特征点)
    cv::Mat processImgL = imgL.clone();
    cv::Mat processImgR = imgR.clone();

    if (m_siftRoiLeft.width > 0 && m_siftRoiLeft.height > 0) {
        cv::Rect safeRoiL = m_siftRoiLeft & cv::Rect(0, 0, processImgL.cols, processImgL.rows);
        if (safeRoiL.area() > 0) {
            cv::Mat blackL = cv::Mat::zeros(processImgL.size(), processImgL.type());
            processImgL(safeRoiL).copyTo(blackL(safeRoiL));
            processImgL = blackL;
            txtSiftResult->append(">>> 已启用左图 ROI 掩膜。");
        }
    }
    if (m_siftRoiRight.width > 0 && m_siftRoiRight.height > 0) {
        cv::Rect safeRoiR = m_siftRoiRight & cv::Rect(0, 0, processImgR.cols, processImgR.rows);
        if (safeRoiR.area() > 0) {
            cv::Mat blackR = cv::Mat::zeros(processImgR.size(), processImgR.type());
            processImgR(safeRoiR).copyTo(blackR(safeRoiR));
            processImgR = blackR;
            txtSiftResult->append(">>> 已启用右图 ROI 掩膜。");
        }
    }

    // 3. 调用底层算法
    m_pointCalibrator->setStereoParams(m_CameraMatrixL, m_DistCoeffsL, m_CameraMatrixR, m_DistCoeffsR, m_R, m_T);
    SiftMeasureResult res = m_pointCalibrator->measureSinglePoint(processImgL, processImgR);

    // 4. 在【原始图像】上画出特征点位置，并更新UI显示
    cv::Mat drawL = imgL.clone();
    cv::Mat drawR = imgR.clone();

        if (res.success) {
        // 在左右图上画绿色的最优匹配点圆圈和红色十字
        cv::circle(drawL, res.pt_left, 10, cv::Scalar(0, 255, 0), 2);
        cv::circle(drawL, res.pt_left, 10, cv::Scalar(0, 0, 255), 1);
        cv::drawMarker(drawL, res.pt_left, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);

        cv::circle(drawR, res.pt_right, 10, cv::Scalar(0, 255, 0), 2);
        cv::circle(drawR, res.pt_right, 10, cv::Scalar(0, 0, 255), 1);
        cv::drawMarker(drawR, res.pt_right, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);

        lblSiftLeftImg->setPixmap(QPixmap::fromImage(mat2QImage(drawL)));
        lblSiftRightImg->setPixmap(QPixmap::fromImage(mat2QImage(drawR))); // <-- 修复1：补全了 Right

        // 5. 输出左相机坐标系下的三维坐标
        txtSiftResult->append("====== SIFT 测距成功 ======");
        txtSiftResult->append(QString("左图特征点像素坐标: [X: %1, Y: %2]")
            .arg(res.pt_left.x, 0, 'f', 2).arg(res.pt_left.y, 0, 'f', 2));
        txtSiftResult->append(QString("右图特征点像素坐标: [X: %1, Y: %2]")
            .arg(res.pt_right.x, 0, 'f', 2).arg(res.pt_right.y, 0, 'f', 2));
        txtSiftResult->append("------ 左相机坐标系三维坐标 ------");
        txtSiftResult->append(QString("X: %1 mm").arg(res.point_3d.x, 0, 'f', 3));
        txtSiftResult->append(QString("Y: %1 mm").arg(res.point_3d.y, 0, 'f', 3));
        txtSiftResult->append(QString("Z: %1 mm").arg(res.point_3d.z, 0, 'f', 3));
        txtSiftResult->append(QString("距离(模长): %1 mm").arg(res.distance, 0, 'f', 3));
    } else {
        lblSiftLeftImg->setPixmap(QPixmap::fromImage(mat2QImage(drawL)));
        lblSiftRightImg->setPixmap(QPixmap::fromImage(mat2QImage(drawR))); // <-- 修复1：补全了 Right
        txtSiftResult->append("====== SIFT 测距失败 ======");
        //txtSiftResult->append(QString("错误原因: %1").arg(res.error_msg.c_str())); // <-- 修复2：加了 .c_str()
    }
}
