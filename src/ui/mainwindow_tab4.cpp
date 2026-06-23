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
#include <QDebug>
#include <sstream>
#include <iomanip>
#include <cmath>

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

    btnDebugAxisCalib = new QPushButton("3. 逐帧调试信息");
    btnDebugAxisCalib->setStyleSheet(Theme::primaryButton());
    btnDebugAxisCalib->setEnabled(false);
    grpAxisCalibLayout->addWidget(btnDebugAxisCalib);

    btnVerify3D = new QPushButton("4. 棋盘格3D重建验证");
    btnVerify3D->setStyleSheet(Theme::successButton());
    btnVerify3D->setEnabled(false);
    grpAxisCalibLayout->addWidget(btnVerify3D);

    btnMultiFrame3D = new QPushButton("5. 多帧棋盘格3D重建");
    btnMultiFrame3D->setStyleSheet(Theme::purpleButton());
    btnMultiFrame3D->setEnabled(false);
    btnMultiFrame3D->setToolTip("对所有旋转序列帧执行完整S3→S4→S5三维重建，验证全流水线精度。");
    grpAxisCalibLayout->addWidget(btnMultiFrame3D);

    QHBoxLayout *reopenLayout = new QHBoxLayout();
    btnMultiFrameStats = new QPushButton("查看统计");
    btnMultiFrameStats->setEnabled(false);
    btnMultiFrameStats->setMaximumWidth(100);
    btnMultiFrame3DView = new QPushButton("查看3D");
    btnMultiFrame3DView->setEnabled(false);
    btnMultiFrame3DView->setMaximumWidth(100);
    reopenLayout->addWidget(btnMultiFrameStats);
    reopenLayout->addWidget(btnMultiFrame3DView);
    reopenLayout->addStretch();
    grpAxisCalibLayout->addLayout(reopenLayout);

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
    tabWidget->addTab(page, "转台标定 & SIFT");

    m_rotatingCalibrator = new Calib::RotatingCalibrator();
    m_pointCalibrator = new PointCalibrator();
    connect(btnLoadRotSeq, &QPushButton::clicked, this, &MainWindow::onLoadRotSeqClicked);
    connect(btnExecAxisCalib, &QPushButton::clicked, this, &MainWindow::onExecAxisCalibClicked);
    connect(btnDebugAxisCalib, &QPushButton::clicked, this, &MainWindow::onDebugAxisCalibClicked);
    connect(btnVerify3D, &QPushButton::clicked, this, &MainWindow::onVerifyChessboard3D);
    connect(btnMultiFrame3D, &QPushButton::clicked, this, &MainWindow::onMultiFrameChessboard3D);
    connect(btnMultiFrameStats, &QPushButton::clicked, this, &MainWindow::onReopenChessboardStats);
    connect(btnMultiFrame3DView, &QPushButton::clicked, this, &MainWindow::onReopenChessboard3DView);
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
    btnDebugAxisCalib->setEnabled(false); // 新序列加载，旧调试数据失效
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
    btnDebugAxisCalib->setEnabled(false);
    btnMultiFrame3D->setEnabled(false);
    btnVerify3D->setEnabled(false);
    btnMultiFrameStats->setEnabled(false);
    btnMultiFrame3DView->setEnabled(false);
    txtAxisCalibResult->append(">>> 开始旋转轴自动标定...");
    txtAxisCalibResult->append(QString("    标定板: %1 x %2, 方格 %3 mm")
        .arg(m_boardSize.width).arg(m_boardSize.height).arg(m_squareSize));
    QApplication::processEvents();

    // 配置标定器
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

    if (ok) {
        m_rotAxisDirection = m_rotatingCalibrator->getAxisDirection();
        m_rotAxisPoint = m_rotatingCalibrator->getAxisPoint();
        m_rotatingCalibrator->getBasePose(m_R_base, m_T_base);
        double rms = m_rotatingCalibrator->getReprojectionError();

        txtAxisCalibResult->append(QString("✅ 标定成功！重投影误差: %1 px").arg(rms, 0, 'f', 3));
        txtAxisCalibResult->append(QString(">>> 轴方向: [%1, %2, %3]")
            .arg(m_rotAxisDirection.at<double>(0), 0, 'f', 4)
            .arg(m_rotAxisDirection.at<double>(1), 0, 'f', 4)
            .arg(m_rotAxisDirection.at<double>(2), 0, 'f', 4));
        txtAxisCalibResult->append(QString(">>> 轴点坐标: [%1, %2, %3] mm")
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
        btnDebugAxisCalib->setEnabled(true);
        btnVerify3D->setEnabled(true);
        btnMultiFrame3D->setEnabled(true);

        m_statusAxis->setText(QString("转台: RMS %1px").arg(rms, 0, 'f', 2));
        m_statusAxis->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(rms < 0.5 ? Theme::SUCCESS : Theme::WARNING));
        QMessageBox::information(this, "轴标定完成",
            QString("旋转轴标定成功！\n\n"
                    "重投影误差: %1 px\n"
                    "轴方向: [%2, %3, %4]\n"
                    "轴点: [%5, %6, %7] mm")
                .arg(rms, 0, 'f', 3)
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
}

// ==============================================================================
// Tab 4 槽函数实现 (逐帧调试弹窗)
// ==============================================================================

void MainWindow::onDebugAxisCalibClicked()
{
    if (!m_rotatingCalibrator) return;

    const auto& debug = m_rotatingCalibrator->getPerFrameDebug();
    if (debug.empty()) {
        QMessageBox::information(this, "调试信息", "暂无逐帧调试数据。请先执行轴标定。");
        return;
    }

    // 统计
    double sum_reproj = 0, max_reproj = 0, min_reproj = 1e9;
    double sum_circle_res = 0, max_circle_res = 0;
    int valid_count = 0;
    for (const auto& d : debug) {
        if (!d.detected) continue;
        sum_reproj += d.reproj_error_left;
        if (d.reproj_error_left > max_reproj) max_reproj = d.reproj_error_left;
        if (d.reproj_error_left < min_reproj) min_reproj = d.reproj_error_left;
        double abs_cr = std::abs(d.circle_residual);
        sum_circle_res += abs_cr;
        if (abs_cr > max_circle_res) max_circle_res = abs_cr;
        valid_count++;
    }
    double avg_reproj = valid_count > 0 ? sum_reproj / valid_count : 0;
    double avg_circle_res = valid_count > 0 ? sum_circle_res / valid_count : 0;

    // 构建 HTML 弹窗
    QString html;
    html += QString("<style>"
            "body { font-family: %1, monospace; font-size: 12px; color: #222; background: #fff; margin: 10px; }"
            "h2 { color: #1565c0; margin: 0 0 8px 0; }"
            ".summary { background: #f5f5f5; border: 1px solid %2; border-radius: 4px; padding: 10px; margin-bottom: 12px; }"
            ".summary td { padding: 2px 12px; }"
            ".summary .lbl { color: %3; }"
            ".summary .val { color: #222; font-weight: bold; }"
            ".summary .warn { color: #e65100; }"
            "table.data { border-collapse: collapse; width: 100%; font-size: 11px; }"
            "table.data th { background: #e0e0e0; color: #1565c0; padding: 4px 8px; border: 1px solid #bbb; position: sticky; top: 0; }"
            "table.data td { padding: 3px 8px; border: 1px solid %2; text-align: right; }"
            "table.data td.idx { text-align: center; color: #999; }"
            "table.data td.pass { color: #2e7d32; }"
            "table.data td.warn { color: #e65100; }"
            "table.data td.fail { color: #c62828; }"
            "table.data tr:nth-child(even) { background: #fafafa; }"
            "table.data tr:hover { background: #e3f2fd; }"
            "</style>").arg(Theme::MONO, Theme::TEXT_PRIMARY, Theme::TEXT_SECONDARY);

    html += "<h2>旋转轴标定 — 逐帧调试信息</h2>";

    // 总览
    html += "<div class='summary'><table>";
    html += QString("<tr><td class='lbl'>有效帧数:</td><td class='val'>%1 / %2</td>"
                    "<td class='lbl'>平均重投影误差:</td><td class='val'>%3 px</td></tr>")
                .arg(valid_count).arg(debug.size()).arg(avg_reproj, 0, 'f', 3);
    html += QString("<tr><td class='lbl'>重投影误差范围:</td><td class='val'>%1 ~ %2 px</td>"
                    "<td class='lbl'>平均圆心残差:</td><td class='val'>%3 mm</td></tr>")
                .arg(min_reproj, 0, 'f', 3).arg(max_reproj, 0, 'f', 3).arg(avg_circle_res, 0, 'f', 3);
    html += QString("<tr><td class='lbl'>最大圆心残差:</td><td class='val %1'>%2 mm</td>"
                    "<td class='lbl'>轴方向:</td><td class='val'>[%3, %4, %5]</td></tr>")
                .arg(max_circle_res > 5.0 ? "warn" : "").arg(max_circle_res, 0, 'f', 3)
                .arg(m_rotAxisDirection.at<double>(0), 0, 'f', 4)
                .arg(m_rotAxisDirection.at<double>(1), 0, 'f', 4)
                .arg(m_rotAxisDirection.at<double>(2), 0, 'f', 4);
    html += "</table></div>";

    // 逐帧表头
    html += "<table class='data'><tr>"
            "<th>帧号</th><th>角度(°)</th><th>重投影(px)</th>"
            "<th>光心 X</th><th>光心 Y</th><th>光心 Z</th>"
            "<th>圆距离</th><th>圆残差</th>"
            "<th>Tx</th><th>Ty</th><th>Tz</th>"
            "</tr>";

    for (const auto& d : debug) {
        double angle_deg = d.angle_rad * 180.0 / M_PI;
        // 重投影误差颜色
        QString reproj_class = d.reproj_error_left < 0.5 ? "pass" :
                               d.reproj_error_left < 1.5 ? "warn" : "fail";
        // 圆残差颜色
        double abs_cr = std::abs(d.circle_residual);
        QString circle_class = abs_cr < 1.0 ? "pass" :
                               abs_cr < 3.0 ? "warn" : "fail";

        html += QString("<tr>"
                        "<td class='idx'>%1</td>"
                        "<td>%2</td>"
                        "<td class='%3'>%4</td>"
                        "<td>%5</td><td>%6</td><td>%7</td>"
                        "<td>%8</td>"
                        "<td class='%9'>%10</td>"
                        "<td>%11</td><td>%12</td><td>%13</td>"
                        "</tr>")
                    .arg(d.frame_idx)
                    .arg(angle_deg, 0, 'f', 1)
                    .arg(reproj_class).arg(d.reproj_error_left, 0, 'f', 3)
                    .arg(d.camera_center[0], 0, 'f', 1)
                    .arg(d.camera_center[1], 0, 'f', 1)
                    .arg(d.camera_center[2], 0, 'f', 1)
                    .arg(d.circle_dist, 0, 'f', 2)
                    .arg(circle_class).arg(d.circle_residual, 0, 'f', 3)
                    .arg(d.tvec_left[0], 0, 'f', 1)
                    .arg(d.tvec_left[1], 0, 'f', 1)
                    .arg(d.tvec_left[2], 0, 'f', 1);
    }
    html += "</table>";

    // 弹窗
    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle("旋转轴标定 — 逐帧调试信息");
    dlg->resize(1100, 650);
    dlg->setStyleSheet("background-color: #fff;");

    QVBoxLayout *layout = new QVBoxLayout(dlg);
    layout->setContentsMargins(4, 4, 4, 4);

    QTextEdit *textEdit = new QTextEdit();
    textEdit->setReadOnly(true);
    textEdit->setHtml(html);
    textEdit->setStyleSheet(QString("background-color: #fff; border: 1px solid %1;").arg(Theme::TEXT_PRIMARY));
    layout->addWidget(textEdit);

    QPushButton *btnClose = new QPushButton("关闭");
    btnClose->setStyleSheet("background-color: #e0e0e0; color: #222; padding: 4px 16px;");
    btnClose->setFixedWidth(100);
    connect(btnClose, &QPushButton::clicked, dlg, &QDialog::close);
    QHBoxLayout *btnLayout = new QHBoxLayout();
    btnLayout->addStretch();
    btnLayout->addWidget(btnClose);
    layout->addLayout(btnLayout);

    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->show();
}

// ==============================================================================
// Tab 4 槽函数实现 (棋盘格3D重建验证)
// ==============================================================================

void MainWindow::onVerifyChessboard3D()
{
    if (m_rotSeqLeftPaths.isEmpty() || m_rotSeqRightPaths.isEmpty()) {
        QMessageBox::warning(this, "缺少数据", "请先加载旋转序列图像并执行轴标定！");
        return;
    }
    if (m_CameraMatrixL.empty() || m_R.empty()) {
        QMessageBox::warning(this, "缺少参数", "请先在Tab2完成双目立体标定！");
        return;
    }
    if (m_rotAxisDirection.empty() || m_rotAxisPoint.empty()) {
        QMessageBox::warning(this, "缺少轴参数", "请先执行轴标定！");
        return;
    }

    // 取中间一帧 (避免首帧PnP偏差大)
    int idx = m_rotSeqLeftPaths.size() / 2;
    cv::Mat imgL_orig = cv::imread(m_rotSeqLeftPaths[idx].toStdString());
    cv::Mat imgR_orig = cv::imread(m_rotSeqRightPaths[idx].toStdString());
    if (imgL_orig.empty() || imgR_orig.empty()) { QMessageBox::warning(this, "错误", "无法读取图像"); return; }

    // 校正
    cv::Mat imgL = imgL_orig, imgR = imgR_orig;
    if (m_IsRectified && !m_MapL1.empty()) {
        cv::Mat rL, rR;
        cv::remap(imgL_orig, rL, m_MapL1, m_MapL2, cv::INTER_LINEAR);
        cv::remap(imgR_orig, rR, m_MapR1, m_MapR2, cv::INTER_LINEAR);
        imgL = rL; imgR = rR;
    }

    // 棋盘格角点检测 (校正后)
    cv::Size boardSize(m_boardSize.width, m_boardSize.height);
    std::vector<cv::Point2f> cornersL, cornersR, cornersL_orig, cornersR_orig;
    bool foundL = cv::findChessboardCorners(imgL, boardSize, cornersL,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    bool foundR = cv::findChessboardCorners(imgR, boardSize, cornersR,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    // 原始图像角点
    bool foundL_orig = cv::findChessboardCorners(imgL_orig, boardSize, cornersL_orig,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    bool foundR_orig = cv::findChessboardCorners(imgR_orig, boardSize, cornersR_orig,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!foundL || !foundR) {
        QMessageBox::warning(this, "检测失败", QString("帧#%1 校正图检测失败").arg(idx)); return;
    }

    // 亚像素精化
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::Mat grayL, grayR, grayL_orig, grayR_orig;
    cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(grayL, cornersL, cv::Size(5,5), cv::Size(-1,-1), criteria);
    cv::cornerSubPix(grayR, cornersR, cv::Size(5,5), cv::Size(-1,-1), criteria);
    if (foundL_orig && foundR_orig) {
        cv::cvtColor(imgL_orig, grayL_orig, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR_orig, grayR_orig, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(grayL_orig, cornersL_orig, cv::Size(5,5), cv::Size(-1,-1), criteria);
        cv::cornerSubPix(grayR_orig, cornersR_orig, cv::Size(5,5), cv::Size(-1,-1), criteria);
    }

    // 校正图极线误差 + 角点对应性检查
    double epi_err_sum = 0, epi_err_max = 0;
    double dx_sum = 0, dy_sum = 0, dx_min = 1e9, dx_max = -1e9;
    for (size_t i = 0; i < cornersL.size() && i < cornersR.size(); ++i) {
        double dy = std::fabs(cornersL[i].y - cornersR[i].y);
        double dx = cornersL[i].x - cornersR[i].x; // 视差
        epi_err_sum += dy; if (dy > epi_err_max) epi_err_max = dy;
        dx_sum += dx; dy_sum += dy;
        if (dx < dx_min) dx_min = dx; if (dx > dx_max) dx_max = dx;
    }
    epi_err_sum /= cornersL.size();
    double dx_mean = dx_sum / cornersL.size();
    double dy_mean = dy_sum / cornersL.size();

    // === 角点顺序一致性诊断 ===
    // OpenCV findChessboardCorners 在左右视图中可能以不同角点为原点（如左图左上、右图右下），
    // 导致索引i对应的不是同一物理角点 → 立体标定结果为错误的R/T。
    int cbCols = m_boardSize.width;
    int cbRows = m_boardSize.height;
    int cbN = cbCols * cbRows;
    if ((int)cornersL.size() >= cbN && (int)cornersR.size() >= cbN) {
        // corner order: OpenCV returns row-major: row*cols + col
        auto cornerAt = [&](const std::vector<cv::Point2f>& v, int r, int c) -> cv::Point2f {
            int idx = r * cbCols + c;
            return (idx >= 0 && idx < (int)v.size()) ? v[idx] : cv::Point2f(NAN, NAN);
        };
        auto calcYErr = [&](const std::vector<cv::Point2f>& cl,
                             const std::vector<cv::Point2f>& cr) -> double {
            double sum = 0; int n = 0;
            for (size_t i = 0; i < cl.size() && i < cr.size(); ++i)
                { sum += std::fabs(cl[i].y - cr[i].y); ++n; }
            return n > 0 ? sum / n : 999.0;
        };

        struct Variant { QString label; std::vector<cv::Point2f> cr_remap; };
        std::vector<Variant> variants;
        variants.push_back({"原序(0°)", cornersR});

        // 行翻转 (镜像于垂直轴 — 每行逆序)
        {
            std::vector<cv::Point2f> v = cornersR;
            for (int r = 0; r < cbRows; ++r)
                for (int c = 0; c < cbCols; ++c)
                    v[r * cbCols + c] = cornersR[r * cbCols + (cbCols - 1 - c)];
            variants.push_back({"行翻转", v});
        }
        // 列翻转 (镜像于水平轴)
        {
            std::vector<cv::Point2f> v = cornersR;
            for (int r = 0; r < cbRows; ++r)
                for (int c = 0; c < cbCols; ++c)
                    v[r * cbCols + c] = cornersR[(cbRows - 1 - r) * cbCols + c];
            variants.push_back({"列翻转", v});
        }
        // 180°翻转
        {
            std::vector<cv::Point2f> v = cornersR;
            for (int r = 0; r < cbRows; ++r)
                for (int c = 0; c < cbCols; ++c)
                    v[r * cbCols + c] = cornersR[(cbRows - 1 - r) * cbCols + (cbCols - 1 - c)];
            variants.push_back({"180°翻转", v});
        }
        // 转置 (行列交换 — 90°旋转场景)
        if (cbRows == cbCols) {
            std::vector<cv::Point2f> v = cornersR;
            for (int r = 0; r < cbRows; ++r)
                for (int c = 0; c < cbCols; ++c)
                    v[r * cbCols + c] = cornersR[c * cbCols + r];
            variants.push_back({"转置", v});
        }

        // 逐角点Y差网格 (可视化分布模式)
        QString yGrid;
        double bestErr = epi_err_sum; int bestIdx = 0;
        for (size_t vi = 0; vi < variants.size(); ++vi) {
            double err = calcYErr(cornersL, variants[vi].cr_remap);
            if (err < bestErr) { bestErr = err; bestIdx = (int)vi; }
        }

        // 只有差异显著(>10×)时才报警
        if (bestErr < epi_err_sum * 0.1) {
            txtAxisCalibResult->append(QString(
                "❌❌ 角点顺序不一致！右图'%1'对应→极线Y误差从%2px降至%3px。\n"
                "左右图 findChessboardCorners 检测的起点/方向不同！\n"
                "→ 立体标定的 allCornersL[i]/allCornersR[i] 可能不对应同一物理角点\n"
                "→ stereoCalibrate 得到的 R/T 是错误匹配点对算出的！")
                .arg(variants[bestIdx].label).arg(epi_err_sum,0,'f',1).arg(bestErr,0,'f',1));
        }

        // 构建直观的角点位置矩阵: 显示左右图各行Y值
        QString orderDiag;
        orderDiag += "角点网格 (行Y均值 L→R):";
        for (int r = 0; r < cbRows && r < 8; ++r) {
            double yL_sum = 0, yR_sum = 0; int n = 0;
            for (int c = 0; c < cbCols; ++c) {
                auto pL = cornerAt(cornersL, r, c);
                auto pR = cornerAt(cornersR, r, c);
                if (!std::isnan(pL.y) && !std::isnan(pR.y))
                    { yL_sum += pL.y; yR_sum += pR.y; ++n; }
            }
            if (n > 0) {
                orderDiag += QString("\n  行%1: L_y=%2 R_y=%3 Δ=%4px")
                    .arg(r).arg(yL_sum/n,0,'f',1).arg(yR_sum/n,0,'f',1)
                    .arg(std::fabs(yL_sum/n - yR_sum/n),0,'f',2);
            }
        }
        txtAxisCalibResult->append(orderDiag);

        // 记录恢复后的最佳结果
        if (bestErr < epi_err_sum * 0.9) {
            double impct = (1.0 - bestErr / epi_err_sum) * 100;
            txtAxisCalibResult->append(QString(
                "→ 信息: 最佳对应'%1', Y误差改善%2% (%3→%4px)")
                .arg(variants[bestIdx].label).arg(impct,0,'f',0)
                .arg(epi_err_sum,0,'f',2).arg(bestErr,0,'f',2));
        }
    }

    // === 三角化: 校正系 (P1_rect, P2_rect) ===
    cv::Mat P1_rect = m_P1;
    cv::Mat P2_rect = m_P2;
    cv::Mat pts_rect;
    cv::triangulatePoints(P1_rect, P2_rect,
        cv::Mat(cornersL).reshape(2,1), cv::Mat(cornersR).reshape(2,1), pts_rect);

    float zr_mean=0, zr_min=1e9, zr_max=-1e9;
    for (int i=0; i<pts_rect.cols; ++i) {
        float w=pts_rect.at<float>(3,i); if(std::fabs(w)<1e-6) continue;
        float z=pts_rect.at<float>(2,i)/w;
        zr_mean+=z; if(z<zr_min)zr_min=z; if(z>zr_max)zr_max=z;
    }
    zr_mean/=pts_rect.cols;

    // === 三角化: 原始系 (K1[I|0], K2[R|T]) ===
    float zo_mean=-999, zo_min=-999, zo_max=-999;
    if (foundL_orig && foundR_orig) {
        cv::Mat P1_orig = (cv::Mat_<double>(3,4) <<
            m_CameraMatrixL.at<double>(0,0),0,m_CameraMatrixL.at<double>(0,2),0,
            0,m_CameraMatrixL.at<double>(1,1),m_CameraMatrixL.at<double>(1,2),0,
            0,0,1,0);
        cv::Mat Rt; cv::hconcat(m_R, m_T, Rt);
        cv::Mat P2_orig = m_CameraMatrixR * Rt;
        cv::Mat pts_orig;
        cv::triangulatePoints(P1_orig, P2_orig,
            cv::Mat(cornersL_orig).reshape(2,1), cv::Mat(cornersR_orig).reshape(2,1), pts_orig);

        zo_mean=0; zo_min=1e9; zo_max=-1e9;
        for (int i=0; i<pts_orig.cols; ++i) {
            float w=pts_orig.at<float>(3,i); if(std::fabs(w)<1e-6) continue;
            float z=pts_orig.at<float>(2,i)/w;
            zo_mean+=z; if(z<zo_min)zo_min=z; if(z>zo_max)zo_max=z;
        }
        zo_mean/=pts_orig.cols;
    }

    // 输出对比
    txtAxisCalibResult->append(QString("[棋盘格验证] 帧#%1 校正后极线Y误差: 均值%2px 最大%3px")
        .arg(idx).arg(epi_err_sum,0,'f',2).arg(epi_err_max,0,'f',2));
    txtAxisCalibResult->append(QString("[角点对应] 视差dx: 均值%1 范围[%2,%3] | Y差均值:%4px (应≈0)")
        .arg(dx_mean,0,'f',1).arg(dx_min,0,'f',1).arg(dx_max,0,'f',1).arg(dy_mean,0,'f',2));
    // 单目fx验证: 数10格(100mm)的像素距, 反推fx
    // 棋盘格是11×8, 取第0列和第10列(间隔100mm)的两角点
    int nCols = m_boardSize.width;
    if (nCols >= 11 && cornersL.size() >= (size_t)(10 * m_boardSize.height + 1)) {
        double pix_dist = cv::norm(cornersL[0] - cornersL[10]); // 同行, 间隔10列=100mm
        double fx_from_image = pix_dist * zr_mean / 100.0;
        txtAxisCalibResult->append(QString("[fx验证] 10格像素距=%1px @Z=%2mm → fx≈%3 (标称%4) %5")
            .arg(pix_dist,0,'f',1).arg(zr_mean,0,'f',1).arg(fx_from_image,0,'f',1)
            .arg(m_CameraMatrixL.at<double>(0,0),0,'f',1)
            .arg(std::fabs(fx_from_image - m_CameraMatrixL.at<double>(0,0)) < 50 ? "✅fx正确" : "❌fx偏差大"));
    }
    txtAxisCalibResult->append(QString("[棋盘格3D-校正系] Z均值%1 Z范围[%2,%3] 展宽%4mm")
        .arg(zr_mean,0,'f',1).arg(zr_min,0,'f',1).arg(zr_max,0,'f',1).arg(zr_max-zr_min,0,'f',1));
    txtAxisCalibResult->append(QString("[棋盘格3D-原始系] Z均值%1 Z范围[%2,%3] 展宽%4mm%5")
        .arg(zo_mean,0,'f',1).arg(zo_min,0,'f',1).arg(zo_max,0,'f',1).arg(zo_max-zo_min,0,'f',1)
        .arg(foundL_orig?"":" (原始图检测失败)"));

    // 弹窗显示校正系点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i<pts_rect.cols; ++i) {
        float w=pts_rect.at<float>(3,i); if(std::fabs(w)<1e-6) continue;
        cloud->push_back(pcl::PointXYZ(pts_rect.at<float>(0,i)/w, pts_rect.at<float>(1,i)/w, pts_rect.at<float>(2,i)/w));
    }

    // === 对比测试: 用立体标定图像对验证 (诊断32px Y误差是验证代码bug还是图像问题) ===
    if (!m_StereoFilesL.isEmpty() && !m_StereoFilesR.isEmpty()) {
        cv::Mat calL = cv::imread(m_StereoFilesL[0].toStdString());
        cv::Mat calR = cv::imread(m_StereoFilesR[0].toStdString());
        if (!calL.empty() && !calR.empty()) {
            if (m_IsRectified && !m_MapL1.empty()) {
                cv::Mat rL, rR;
                cv::remap(calL, rL, m_MapL1, m_MapL2, cv::INTER_LINEAR);
                cv::remap(calR, rR, m_MapR1, m_MapR2, cv::INTER_LINEAR);
                calL = rL; calR = rR;
            }
            std::vector<cv::Point2f> cL, cR;
            bool fL = cv::findChessboardCorners(calL, boardSize, cL,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            bool fR = cv::findChessboardCorners(calR, boardSize, cR,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            if (fL && fR) {
                cv::Mat gL, gR;
                cv::cvtColor(calL, gL, cv::COLOR_BGR2GRAY);
                cv::cvtColor(calR, gR, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gL, cL, cv::Size(5,5), cv::Size(-1,-1), criteria);
                cv::cornerSubPix(gR, cR, cv::Size(5,5), cv::Size(-1,-1), criteria);
                double calYErr = 0, calYMax = 0;
                for (size_t i = 0; i < cL.size() && i < cR.size(); ++i) {
                    double dy = std::fabs(cL[i].y - cR[i].y);
                    calYErr += dy; if (dy > calYMax) calYMax = dy;
                }
                calYErr /= cL.size();
                cv::Mat ptsCal;
                cv::triangulatePoints(m_P1, m_P2,
                    cv::Mat(cL).reshape(2,1), cv::Mat(cR).reshape(2,1), ptsCal);
                float zCal=0, zCalMin=1e9, zCalMax=-1e9;
                for (int i=0; i<ptsCal.cols; ++i) {
                    float w=ptsCal.at<float>(3,i); if(std::fabs(w)<1e-6) continue;
                    float z=ptsCal.at<float>(2,i)/w;
                    zCal+=z; if(z<zCalMin)zCalMin=z; if(z>zCalMax)zCalMax=z;
                }
                zCal/=ptsCal.cols;
                txtAxisCalibResult->append(QString(
                    "\n[标定对对比] 第1对标定图 极线Y误差: %1px (max %2px) | Z均值%3 Z展宽%4mm %5")
                    .arg(calYErr,0,'f',2).arg(calYMax,0,'f',2)
                    .arg(zCal,0,'f',1).arg(zCalMax-zCalMin,0,'f',1)
                    .arg((calYErr < 1.0 && (zCalMax-zCalMin) < 5.0) ? "✅标定对正常→旋转序列图像有问题" : "❌标定对也异常→立体标定本身错误"));
                if (calYErr < 1.0 && epi_err_sum > 5.0) {
                    txtAxisCalibResult->append("⚠️ 标定对Y误差<1px但旋转序列帧>5px → 旋转序列图像与标定图像不一致(分辨率/相机位置/对焦)");
                }
            }
        }
    }

    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle(QString("棋盘格3D #%1 校正系(%2点) Z展宽%3mm").arg(idx).arg(cloud->size()).arg(zr_max-zr_min,0,'f',1));
    dlg->resize(800, 600);
    QVBoxLayout *lay = new QVBoxLayout(dlg);
    PointCloudViewer *viewer = new PointCloudViewer(dlg);
    lay->addWidget(viewer);
    viewer->showPointCloud(cloud, "chessboard3d");
    viewer->resetCamera();
    dlg->show();
}

// ==============================================================================
// 多帧棋盘格3D重建: 全流水线验证 (S3→S4→S5)
// ==============================================================================
// 多帧棋盘格3D重建 — 使用完整S1-S6流水线, PnP反推旋转角, 纯轴旋转(关ICP)
// ==============================================================================
void MainWindow::onMultiFrameChessboard3D()
{
    if (m_rotSeqLeftPaths.isEmpty()) {
        QMessageBox::warning(this, "缺少数据", "请先加载旋转序列图像！");
        return;
    }
    if (!m_IsRectified || m_P1.empty()) {
        QMessageBox::warning(this, "缺少参数", "请先在Tab2完成双目立体标定！");
        return;
    }
    if (m_rotAxisDirection.empty() || m_rotAxisPoint.empty()) {
        QMessageBox::warning(this, "缺少轴参数", "请先执行轴标定！");
        return;
    }
    if (m_R_base.empty() || m_T_base.empty()) {
        QMessageBox::warning(this, "缺少基座位姿", "轴标定数据不完整，请重新执行轴标定！");
        return;
    }
    if (!m_builder) {
        QMessageBox::warning(this, "缺少重建器", "PointCloudBuilder未初始化");
        return;
    }

    // 检查校正矩阵完整性
    if (m_R1.empty() || m_R2.empty() || m_P1.empty() || m_P2.empty()) {
        QMessageBox::warning(this, "参数不完整", "R1/R2/P1/P2为空，请重新执行立体标定！");
        return;
    }

    // 注入标定数据到m_builder (与Tab5一致)
    {
        CalibrationData calibData;
        calibData.cameraMatrixL = m_CameraMatrixL.clone();
        calibData.distCoeffL = m_DistCoeffsL.clone();
        calibData.cameraMatrixR = m_CameraMatrixR.clone();
        calibData.distCoeffR = m_DistCoeffsR.clone();
        calibData.R_stereo = m_R.clone();
        calibData.T_stereo = m_T.clone();
        calibData.P1 = m_P1.clone();
        calibData.P2 = m_P2.clone();
        calibData.is_rectified = m_IsRectified;
        calibData.R_rect_L = m_R1.clone();
        calibData.R_rect_R = m_R2.clone();
        calibData.P1_rectified = m_P1.clone();
        calibData.P2_rectified = m_P2.clone();
        cv::Mat R_base_t = m_R_base.t();
        calibData.R_cam2turntable = R_base_t.clone();
        calibData.T_cam2turntable = -m_R_base.t() * m_T_base;
        m_builder->setCalibrationData(calibData);
    }

    // 获取PnP反推的每帧角度: 从perFrameDebug的rvec提取
    std::vector<double> pnp_angles_deg;
    if (m_rotatingCalibrator) {
        const auto& debug = m_rotatingCalibrator->getPerFrameDebug();
        // 收集所有帧的角度 (rvec的范数即旋转角, 符号由与轴的点积决定)
        Eigen::Vector3d ax(m_rotAxisDirection.at<double>(0),
                           m_rotAxisDirection.at<double>(1),
                           m_rotAxisDirection.at<double>(2));
        for (size_t i = 0; i < debug.size(); ++i) {
            if (!debug[i].detected) continue;
            Eigen::Vector3d rv(debug[i].rvec_left[0],
                               debug[i].rvec_left[1],
                               debug[i].rvec_left[2]);
            double ang = rv.dot(ax) * 180.0 / CV_PI;
            pnp_angles_deg.push_back(ang);
        }
        // 转换为相对于帧0的角度
        if (!pnp_angles_deg.empty()) {
            double a0 = pnp_angles_deg[0];
            for (double& a : pnp_angles_deg) a -= a0;
        }
    }
    int N = std::min(m_rotSeqLeftPaths.size(), m_rotSeqRightPaths.size());
    if (pnp_angles_deg.empty()) {
        for (int i = 0; i < N; ++i) pnp_angles_deg.push_back(i * 3.0);
    }
    while ((int)pnp_angles_deg.size() < N) pnp_angles_deg.push_back(pnp_angles_deg.back() + 3.0);

    // 检测异常角度 (非单调递增超过2帧则降级)
    int badAngles = 0;
    for (size_t i = 1; i < pnp_angles_deg.size(); ++i)
        if (pnp_angles_deg[i] < pnp_angles_deg[i-1] - 1.0) ++badAngles;
    if (badAngles > 2) {
        for (int i = 0; i < N; ++i) pnp_angles_deg[i] = i * 3.0;
    }
    QString angleSrc = (badAngles > 2) ? "均匀步长(角度异常降级)" : "PnP rvec反推";

    cv::Size boardSize(m_boardSize.width, m_boardSize.height);
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    // 轴参数
    Eigen::Vector3f axis_dir(m_rotAxisDirection.at<double>(0),
                              m_rotAxisDirection.at<double>(1),
                              m_rotAxisDirection.at<double>(2));
    Eigen::Vector3f axis_pt(m_rotAxisPoint.at<double>(0),
                             m_rotAxisPoint.at<double>(1),
                             m_rotAxisPoint.at<double>(2));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> viewClouds;
    std::vector<double> viewAngles;
    QString frameStats;
    int totalFrames = 0;

    for (int fi = 0; fi < N; ++fi) {
        // 读取原始图像
        cv::Mat imgL_orig = cv::imread(m_rotSeqLeftPaths[fi].toStdString());
        cv::Mat imgR_orig = cv::imread(m_rotSeqRightPaths[fi].toStdString());
        if (imgL_orig.empty() || imgR_orig.empty()) continue;

        // 校正 (remap)
        cv::Mat imgL = imgL_orig, imgR = imgR_orig;
        cv::remap(imgL_orig, imgL, m_MapL1, m_MapL2, cv::INTER_LINEAR);
        cv::remap(imgR_orig, imgR, m_MapR1, m_MapR2, cv::INTER_LINEAR);

        // S1替代: 棋盘格角点检测 (在校正后图像上)
        std::vector<cv::Point2f> cL, cR;
        bool fL = cv::findChessboardCorners(imgL, boardSize, cL,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool fR = cv::findChessboardCorners(imgR, boardSize, cR,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!fL || !fR) continue;

        cv::Mat gL, gR;
        cv::cvtColor(imgL, gL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR, gR, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gL, cL, cv::Size(5,5), cv::Size(-1,-1), criteria);
        cv::cornerSubPix(gR, cR, cv::Size(5,5), cv::Size(-1,-1), criteria);

        // S2替代: 按索引配对 (校正后角点已有序, 直接配对)
        size_t nPts = std::min(cL.size(), cR.size());
        std::vector<cv::Point2f> matchL(cL.begin(), cL.begin() + nPts);
        std::vector<cv::Point2f> matchR(cR.begin(), cR.begin() + nPts);

        // S3: 三角化 (复用m_builder, 点已在校正系无需undistort)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam = m_builder->triangulatePoints(matchL, matchR);
        if (cloud_cam->empty()) continue;

        // S4: 坐标变换 (复用m_builder)
        double angle_deg = pnp_angles_deg[fi];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world = m_builder->transformToTurntableFrame(cloud_cam, angle_deg);
        if (cloud_world->empty()) continue;

        // S5替代: 纯轴旋转对齐到帧0 (关ICP)
        double angle_rad = -angle_deg * CV_PI / 180.0;
        Eigen::AngleAxisf rot(angle_rad, axis_dir);
        Eigen::Matrix3f R = rot.toRotationMatrix();
        Eigen::Vector3f T = (Eigen::Matrix3f::Identity() - R) * axis_pt;

        double zMean = 0, zMin = 1e9, zMax = -1e9;
        for (auto& pt : cloud_world->points) {
            float wx = R(0,0)*pt.x + R(0,1)*pt.y + R(0,2)*pt.z + T(0);
            float wy = R(1,0)*pt.x + R(1,1)*pt.y + R(1,2)*pt.z + T(1);
            float wz = R(2,0)*pt.x + R(2,1)*pt.y + R(2,2)*pt.z + T(2);
            pt.x = wx; pt.y = wy; pt.z = wz;
            zMean += wz; if (wz < zMin) zMin = wz; if (wz > zMax) zMax = wz;
        }
        zMean /= cloud_world->size();

        viewClouds.push_back(cloud_world);
        viewAngles.push_back(0.0);

        frameStats += QString("  帧%1(%2°): %3角点→S3:%4点→S4→纯轴旋转 Z均值%5 Z展宽%6mm\n")
            .arg(fi).arg(angle_deg, 0, 'f', 1).arg((int)nPts).arg((int)cloud_world->size())
            .arg(zMean, 0, 'f', 2).arg(zMax - zMin, 0, 'f', 2);
        ++totalFrames;
    }

    if (totalFrames == 0) {
        QMessageBox::warning(this, "检测失败", "所有帧均未检测到棋盘格角点！");
        return;
    }

    // 合并所有对齐后的点云用于统计
    pcl::PointCloud<pcl::PointXYZ>::Ptr allCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& c : viewClouds)
        *allCloud += *c;

    // 全局统计
    double gzMean = 0, gzMin = 1e9, gzMax = -1e9;
    for (const auto& pt : allCloud->points) {
        gzMean += pt.z;
        if (pt.z < gzMin) gzMin = pt.z;
        if (pt.z > gzMax) gzMax = pt.z;
    }
    gzMean /= allCloud->size();

    // 平面拟合 (SVD)
    Eigen::MatrixXf ptsMat(allCloud->size(), 3);
    for (size_t i = 0; i < allCloud->size(); ++i) {
        ptsMat(i,0)=allCloud->points[i].x; ptsMat(i,1)=allCloud->points[i].y; ptsMat(i,2)=allCloud->points[i].z;
    }
    Eigen::RowVector3f centroid = ptsMat.colwise().mean();
    Eigen::MatrixXf centered = ptsMat.rowwise() - centroid;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(centered, Eigen::ComputeThinV);
    Eigen::Vector3f plane_n = svd.matrixV().col(2);
    if (plane_n(2) < 0) plane_n = -plane_n;
    float plane_d = -plane_n.dot(centroid.transpose());
    float plane_rms = 0;
    for (size_t i = 0; i < allCloud->size(); ++i) {
        float dist = std::fabs(plane_n(0)*allCloud->points[i].x + plane_n(1)*allCloud->points[i].y + plane_n(2)*allCloud->points[i].z + plane_d);
        plane_rms += dist * dist;
    }
    plane_rms = std::sqrt(plane_rms / allCloud->size());

    // ==== 弹窗显示 (替代txtAxisCalibResult) ====
    QDialog *dlgResult = new QDialog(this, Qt::Window);
    dlgResult->setWindowTitle(QString("多帧棋盘格3D %1帧 %2点").arg(totalFrames).arg((int)allCloud->size()));
    dlgResult->resize(750, 650);
    QVBoxLayout *dlgLay = new QVBoxLayout(dlgResult);

    QTextEdit *txtResult = new QTextEdit();
    txtResult->setReadOnly(true);
    txtResult->setFont(QFont(Theme::MONO, 10));
    txtResult->setStyleSheet(QString("background-color: %1; color: %2;").arg(Theme::BG_CARD, Theme::TEXT_PRIMARY));

    bool pass = (gzMax - gzMin < 5.0 && plane_rms < 3.0);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "====== 多帧棋盘格3D重建 ======\n\n";
    ss << "流水线: remap校正 → 角点检测 → S2(index pair) → S3(m_builder) → S4(m_builder) → S5(纯轴旋转,关ICP)\n";
    ss << "旋转角来源: " << angleSrc.toStdString() << " (" << pnp_angles_deg.size() << "帧有效)\n\n";
    ss << "--- 逐帧统计 ---\n" << frameStats.toStdString() << "\n";
    ss << "--- 全局 ---\n";
    ss << "总帧数: " << totalFrames << "  总角点数: " << allCloud->size() << "\n";
    ss << "Z均值: " << gzMean << "  Z范围: [" << gzMin << "," << gzMax << "]  展宽: " << (gzMax-gzMin) << "mm\n";
    ss << "平面拟合RMS: " << plane_rms << "mm  法向量: [" << plane_n(0) << "," << plane_n(1) << "," << plane_n(2) << "]\n";
    ss << "\n判定: " << (pass ? "✅ Z展宽<5mm 流水线正确" : "❌ Z展宽>5mm 流水线存在误差") << "\n";

    txtResult->setText(QString::fromStdString(ss.str()));
    dlgLay->addWidget(txtResult);

    // 角点坐标简要
    QTextEdit *txtCoord = new QTextEdit();
    txtCoord->setReadOnly(true);
    txtCoord->setFont(QFont(Theme::MONO, 9));
    txtCoord->setMaximumHeight(150);
    std::stringstream cs;
    cs << std::fixed << std::setprecision(3);
    int showN = std::min(30, (int)allCloud->size());
    for (int i = 0; i < showN; ++i)
        cs << allCloud->points[i].x << "," << allCloud->points[i].y << "," << allCloud->points[i].z << "\n";
    cs << "... (共" << allCloud->size() << "点)";
    txtCoord->setText(QString::fromStdString(cs.str()));
    dlgLay->addWidget(txtCoord);

    dlgResult->show();

    // 单独弹出3D点云窗口
    QDialog *dlg3D = new QDialog(this, Qt::Window);
    dlg3D->setWindowTitle(QString("棋盘格3D点云 %1点 Z展宽%2mm").arg((int)allCloud->size()).arg(gzMax-gzMin,0,'f',1));
    dlg3D->resize(800, 650);
    QVBoxLayout *lay3D = new QVBoxLayout(dlg3D);
    PointCloudViewer *viewer = new PointCloudViewer(dlg3D);
    lay3D->addWidget(viewer);
    viewer->showPointCloud(allCloud, "multi_chessboard_full");
    viewer->resetCamera();
    dlg3D->show();

    // 存储结果供重开按钮使用
    m_lastChessboardCloud = allCloud;
    m_lastChessboardStats = QString::fromStdString(ss.str());
    btnMultiFrameStats->setEnabled(true);
    btnMultiFrame3DView->setEnabled(true);

    // 同步输出到日志窗口 (简略版)
    txtAxisCalibResult->append(QString("[多帧棋盘格] %1帧%2点 Z展宽%3mm 平面RMS %4mm %5")
        .arg(totalFrames).arg((int)allCloud->size()).arg(gzMax-gzMin,0,'f',2).arg(plane_rms,0,'f',3)
        .arg(pass ? "✅" : "❌"));
}

void MainWindow::onReopenChessboardStats()
{
    if (m_lastChessboardStats.isEmpty()) return;
    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle("多帧棋盘格3D — 统计结果");
    dlg->resize(750, 600);
    QVBoxLayout *lay = new QVBoxLayout(dlg);
    QTextEdit *txt = new QTextEdit();
    txt->setReadOnly(true);
    txt->setFont(QFont(Theme::MONO, 10));
    txt->setStyleSheet(QString("background-color: %1; color: %2;").arg(Theme::BG_CARD, Theme::TEXT_PRIMARY));
    txt->setText(m_lastChessboardStats);
    lay->addWidget(txt);
    dlg->show();
}

void MainWindow::onReopenChessboard3DView()
{
    if (!m_lastChessboardCloud || m_lastChessboardCloud->empty()) return;
    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle(QString("棋盘格3D点云 %1点").arg((int)m_lastChessboardCloud->size()));
    dlg->resize(800, 650);
    QVBoxLayout *lay = new QVBoxLayout(dlg);
    PointCloudViewer *viewer = new PointCloudViewer(dlg);
    lay->addWidget(viewer);
    viewer->showPointCloud(m_lastChessboardCloud, "chessboard_replay");
    viewer->resetCamera();
    dlg->show();
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
    m_pointCalibrator->setStereoParams(m_CameraMatrixL, m_DistCoeffsL, m_CameraMatrixR, m_DistCoeffsR, m_R, m_T, m_P1, m_P2, m_IsRectified);
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
