// ==================== Tab5: 三维重建 ====================
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include <QSplitter>
#include "core/pointcloudbuilder.h"
#include "core/pointcloudviewer.h"
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
#include <QDebug>
#include <QFormLayout>
#include <QScrollArea>
#include <QProgressBar>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QDateTime>
#include <QTextEdit>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

void MainWindow::initTab5(QTabWidget* tabWidget) {
    // ========================== Tab 5: 三维重建 ==========================
    QWidget *tab5 = new QWidget();
    QVBoxLayout *tab5MainLayout = new QVBoxLayout(tab5);
    tab5MainLayout->setContentsMargins(Theme::MARGIN, Theme::MARGIN, Theme::MARGIN, Theme::MARGIN);
    QSplitter *splitter = new QSplitter(Qt::Horizontal);
    splitter->setHandleWidth(1);
    tab5MainLayout->addWidget(splitter);

    // --- 初始化核心算法与显示对象 ---
    m_builder = new PointCloudBuilder();
    m_viewer3D = new PointCloudViewer(); // 绝不传 this，防止 VTK 弹窗
    m_viewer3D->setFocusPolicy(Qt::StrongFocus);
    m_roiLeft = cv::Rect();   // 强制初始化为空 (算法层会自动降级为全图)
    m_roiRight = cv::Rect();  // 强制初始化为空

    // 1. 左侧控制面板
    QWidget *tab5CtrlPanel = new QWidget();
    QVBoxLayout *tab5CtrlVLayout = new QVBoxLayout(tab5CtrlPanel);
    tab5CtrlPanel->setFixedWidth(360);

    // ==================== 1. 核心流程按钮组 ====================
    QGroupBox *grpReconCtrl = new QGroupBox("重建流程控制");
    QVBoxLayout *grpReconCtrlLayout = new QVBoxLayout(grpReconCtrl);

    QFormLayout *formReconBasic = new QFormLayout();
    spinAngleStep = new QDoubleSpinBox();
    spinAngleStep->setRange(0.1, 360.0);
    spinAngleStep->setValue(1.8);  // 默认1.8度
    spinAngleStep->setSuffix("°");

    spinViewCount = new QSpinBox();
    spinViewCount->setRange(1, 9999);
    spinViewCount->setValue(200);  // 默认200个视角

    formReconBasic->addRow("旋转步长:", spinAngleStep);
    formReconBasic->addRow("视角数量:", spinViewCount);
    grpReconCtrlLayout->addLayout(formReconBasic);

    QHBoxLayout *layoutPreviewFrame = new QHBoxLayout();
    layoutPreviewFrame->addWidget(new QLabel("预览帧号:"));

    spinPreviewIndex = new QSpinBox();
    spinPreviewIndex->setRange(0, 9999);
    spinPreviewIndex->setValue(0);
    spinPreviewIndex->setFixedWidth(70);
    layoutPreviewFrame->addWidget(spinPreviewIndex);
    layoutPreviewFrame->addStretch();

    btnPreviewSingleFrame = new QPushButton("预览单帧点云");
    btnPreviewSingleFrame->setMinimumWidth(Theme::BTN_MIN_W);
    btnPreviewSingleFrame->setStyleSheet(Theme::warningButton());
    btnPreviewSingleFrame->setToolTip("使用当前参数实时计算并显示指定帧的点云，不执行全局拼接");
    layoutPreviewFrame->addWidget(btnPreviewSingleFrame);

    grpReconCtrlLayout->addLayout(layoutPreviewFrame);

    btnLoadReconSeq = new QPushButton("1. 导入旋转图像序列");
    btnLoadReconSeq->setStyleSheet(Theme::boldButton());
    btnStartRecon = new QPushButton("2. 开始三维重建");
    btnStartRecon->setStyleSheet(Theme::successButton());
    btnSaveRecon = new QPushButton("3. 保存结果");
    // 【新增】点云窗口按钮
    btnShowPointCloudWindow = new QPushButton("4. 显示点云窗口");
    btnShowPointCloudWindow->setToolTip("弹出新窗口仅显示点云（不含网格）");
    // 【新增】独立3D视图测试按钮
    btnOpenViewer = new QPushButton("5. 弹出独立3D视图");
    btnOpenViewer->setToolTip("弹出新窗口测试鼠标左键旋转是否正常");
    btnSaveRecon->setEnabled(false);
    grpReconCtrlLayout->addWidget(btnLoadReconSeq);
    grpReconCtrlLayout->addWidget(btnStartRecon);
    grpReconCtrlLayout->addWidget(btnSaveRecon);
    grpReconCtrlLayout->addWidget(btnShowPointCloudWindow);
    grpReconCtrlLayout->addWidget(btnOpenViewer);

    // ==================== 2. 算法参数滚动面板 ====================
    QScrollArea *paramScroll = new QScrollArea();
    paramScroll->setWidgetResizable(true);
    QWidget *paramContainer = new QWidget();
    QVBoxLayout *paramMainLayout = new QVBoxLayout(paramContainer);
    paramMainLayout->setSpacing(Theme::SPACING);

    // --- S1: 光条中心提取 (支持 Steger / 水平切片 切换) ---
    QGroupBox *grpS1 = new QGroupBox("S1: 光条中心提取");
    QFormLayout *formS1 = new QFormLayout(grpS1);
    formS1->setLabelAlignment(Qt::AlignLeft);

    btnSelectRoiLeft = new QPushButton("左图 ROI 框选");
    btnSelectRoiRight = new QPushButton("右图 ROI 框选");

    // 【S1方法选择】
    cmbS1Method = new QComboBox();
    cmbS1Method->addItem("Steger + 掩膜 (推荐)");
    cmbS1Method->addItem("灰度重心 + 掩膜");
    cmbS1Method->addItem("纯列极值");
    cmbS1Method->setCurrentIndex(0); // 默认Steger+掩膜
    cmbS1Method->setToolTip("切换S1光条提取算法");

    // 【算法切换开关】
    chkUseSteger = new QCheckBox("使用 Steger 亚像素算法");
    chkUseSteger->setChecked(true);
    chkUseSteger->setToolTip("勾选：Hessian矩阵抗过曝提取(推荐)\n不勾选：降级为水平切片+灰度重心");

    spinLabA = new QSpinBox();
    spinLabA->setRange(80, 200);
    spinLabA->setValue(160);
    spinLabA->setSingleStep(5);
    spinLabA->setToolTip("低于此值判定为非红光噪声");

    // Steger 核心参数
    spinStegerSigma = new QDoubleSpinBox();
    spinStegerSigma->setRange(0.5, 5.0);
    spinStegerSigma->setValue(1.2);
    spinStegerSigma->setSingleStep(0.1);
    spinStegerSigma->setDecimals(1);
    spinStegerSigma->setToolTip("高斯平滑系数。光条越粗/噪声越大，此值应越大(如2.0)");

    spinStegerTMax = new QDoubleSpinBox();
    spinStegerTMax->setRange(0.1, 1.5);
    spinStegerTMax->setValue(0.6); // 默认 0.6
    spinStegerTMax->setSingleStep(0.1);
    spinStegerTMax->setDecimals(1);
    spinStegerTMax->setToolTip("泰勒偏移距离上限(像素)。越小越严格，防止跨光条错配");

    // 过曝恢复专用参数
    chkOverexposedEnable = new QCheckBox("启用高反光过曝恢复");
    chkOverexposedEnable->setChecked(true); // 默认开启
    chkOverexposedEnable->setToolTip("当光条中心因反光变成白色时，通过两侧红色边缘反推真实中心");

    spinOverexposedL = new QSpinBox();
    spinOverexposedL->setRange(200, 255);
    spinOverexposedL->setValue(235); // 默认 235
    spinOverexposedL->setToolTip("LAB空间L通道(亮度)阈值。像素亮度大于此值且失去红色，视为过曝");

    spinEdgeOffsetSigma = new QDoubleSpinBox();
    spinEdgeOffsetSigma->setRange(0.5, 3.0);
    spinEdgeOffsetSigma->setValue(1.2); // 默认 1.2
    spinEdgeOffsetSigma->setSingleStep(0.1);
    spinEdgeOffsetSigma->setDecimals(1);
    spinEdgeOffsetSigma->setToolTip("过曝时向法线两侧寻找红边缘的距离 = sigma × 此系数");

    // 原有水平切片的参数 (Steger 开启时会被禁用变灰)
    spinMorphSigma = new QDoubleSpinBox();
    spinMorphSigma->setRange(2.0, 50.0);
    spinMorphSigma->setValue(5.0);
    spinMorphSigma->setSingleStep(1.0);
    spinMorphSigma->setDecimals(0);
    spinMorphSigma->setToolTip("(水平切片用)单行连续红光的最小像素数");

    spinSampleStep = new QSpinBox();
    spinSampleStep->setRange(1, 10);
    spinSampleStep->setValue(1);
    spinSampleStep->setToolTip("(水平切片用)行降采样步长");

    // 添加到表单布局
    formS1->addRow("", btnSelectRoiLeft);
    formS1->addRow("", btnSelectRoiRight);
    formS1->addRow("S1 算法:", cmbS1Method);
    formS1->addRow(chkUseSteger);
    formS1->addRow("红色阈值(LAB A):", spinLabA);
    formS1->addRow("高斯平滑系数:", spinStegerSigma);
    formS1->addRow("泰勒偏移阈值:", spinStegerTMax);
    formS1->addRow(chkOverexposedEnable);
    formS1->addRow("过曝亮度阈值(L):", spinOverexposedL);
    formS1->addRow("边缘偏移系数:", spinEdgeOffsetSigma);

    // 分隔线提示
    QLabel* lblDeprecated = new QLabel("── 以下为降级算法参数 ──");
    lblDeprecated->setStyleSheet("color: gray; font-size: 10px;");
    lblDeprecated->setAlignment(Qt::AlignCenter);
    formS1->addRow(lblDeprecated);

    formS1->addRow("最小线段长度:", spinMorphSigma);
    formS1->addRow("行降采样步长:", spinSampleStep);

    paramMainLayout->addWidget(grpS1);

    // 【UI 联动逻辑】：勾选 Steger 时，禁用水平切片参数
    auto updateS1UIState = [this](bool useSteger) {
        spinMorphSigma->setEnabled(!useSteger);
        spinSampleStep->setEnabled(!useSteger);
    };

    connect(chkUseSteger, &QCheckBox::toggled, this, updateS1UIState);
    updateS1UIState(chkUseSteger->isChecked()); // 初始化一次状态

    // --- S2: 双目极线匹配 ---
    QGroupBox *grpS2 = new QGroupBox("S2: 双目极线匹配");
    QFormLayout *formS2 = new QFormLayout(grpS2);
    formS2->setLabelAlignment(Qt::AlignLeft);

    spinEpipolarThresh = new QDoubleSpinBox();
    spinEpipolarThresh->setRange(0.1, 20.0);
    spinEpipolarThresh->setValue(5.0);
    spinEpipolarThresh->setSingleStep(0.1);
    spinEpipolarThresh->setDecimals(1);

    spinDepthMin = new QDoubleSpinBox();
    spinDepthMin->setRange(0.0, 5000.0);
    spinDepthMin->setValue(10.0);
    spinDepthMin->setSingleStep(1.0);
    spinDepthMin->setDecimals(1);

    spinDepthMax = new QDoubleSpinBox();
    spinDepthMax->setRange(10.0, 10000.0);
    spinDepthMax->setValue(500.0);
    spinDepthMax->setSingleStep(10.0);
    spinDepthMax->setDecimals(1);

    spinMinPtsSeg = new QSpinBox();
    spinMinPtsSeg->setRange(1, 100);
    spinMinPtsSeg->setValue(5);
    spinMinPtsSeg->setSingleStep(1);

    spinBreakDist = new QDoubleSpinBox();
    spinBreakDist->setRange(0.1, 50.0);
    spinBreakDist->setValue(8.0);
    spinBreakDist->setSingleStep(0.5);
    spinBreakDist->setDecimals(1);

    spinDpSkipPenalty = new QDoubleSpinBox();
    spinDpSkipPenalty->setRange(0.0, 200.0);
    spinDpSkipPenalty->setValue(15.0);   // 默认值，与 DP 逻辑匹配
    spinDpSkipPenalty->setSingleStep(0.5);
    spinDpSkipPenalty->setDecimals(1);

    spinDpSmoothWeight = new QDoubleSpinBox();
    spinDpSmoothWeight->setRange(0.0, 20.0);
    spinDpSmoothWeight->setValue(0.5);
    spinDpSmoothWeight->setSingleStep(0.1);
    spinDpSmoothWeight->setDecimals(1);

    spinDisparityBreak = new QDoubleSpinBox();
    spinDisparityBreak->setRange(0.1, 100.0);
    spinDisparityBreak->setValue(15.0);    // 默认值，代表视差突变超过15像素就切断
    spinDisparityBreak->setSingleStep(1.0);
    spinDisparityBreak->setDecimals(1);

    formS2->addRow("极线匹配容差:", spinEpipolarThresh);
    formS2->addRow("最小有效深度:", spinDepthMin);
    formS2->addRow("最大有效深度:", spinDepthMax);
    formS2->addRow("单段最少点数:", spinMinPtsSeg);
    formS2->addRow("线段间断距离:", spinBreakDist);
    formS2->addRow("DP跳过惩罚:", spinDpSkipPenalty);
    formS2->addRow("DP视差平滑度:", spinDpSmoothWeight);
    formS2->addRow("视差跳变切断阈值:", spinDisparityBreak);
    paramMainLayout->addWidget(grpS2);

    // --- S5: 多视角ICP配准 ---
    QGroupBox *grpS5 = new QGroupBox("S5: 多视角 ICP 配准");
    QFormLayout *formS5 = new QFormLayout(grpS5);
    formS5->setLabelAlignment(Qt::AlignLeft);

    chkUseIcp = new QCheckBox("启用ICP精配准");
    chkUseIcp->setChecked(false);
    chkUseIcp->setToolTip("纯轴旋转(默认): 依赖精确轴标定, 适合对称物体。ICP: 可修正微小平移, 但可能加剧对称面形变。");

    spinIcpMaxDist = new QDoubleSpinBox();
    spinIcpMaxDist->setRange(0.001, 100);
    spinIcpMaxDist->setValue(3.0);
    spinIcpMaxDist->setSingleStep(0.5);
    spinIcpMaxDist->setDecimals(1);

    spinIcpIter = new QSpinBox();
    spinIcpIter->setRange(1, 200);
    spinIcpIter->setValue(20);
    spinIcpIter->setSingleStep(1);

    spinIcpEpsilon = new QDoubleSpinBox();
    spinIcpEpsilon->setRange(1e-10, 0.1);
    spinIcpEpsilon->setValue(0.001);
    spinIcpEpsilon->setSingleStep(0.00001);
    spinIcpEpsilon->setDecimals(7);
    spinIcpEpsilon->setToolTip("ICP欧式适应度收敛阈值");

    spinIcpTransEps = new QDoubleSpinBox();
    spinIcpTransEps->setRange(1e-10, 0.1);
    spinIcpTransEps->setValue(1e-8);
    spinIcpTransEps->setSingleStep(0.000001);
    spinIcpTransEps->setDecimals(8);
    spinIcpTransEps->setToolTip("ICP平移向量收敛阈值");

    spinIcpAxisTrust = new QDoubleSpinBox();
    spinIcpAxisTrust->setRange(0.0, 100.0);
    spinIcpAxisTrust->setValue(95.0);
    spinIcpAxisTrust->setSingleStep(1.0);
    spinIcpAxisTrust->setSuffix("%");
    spinIcpAxisTrust->setToolTip("ICP对转台轴的信任度。100%=完全信任轴(ICP仅修正非轴误差,适合对称物体)。0%=完全信任ICP。");

    formS5->addRow("启用ICP:", chkUseIcp);
    formS5->addRow("ICP轴信任度:", spinIcpAxisTrust);
    formS5->addRow("最大对应距离:", spinIcpMaxDist);
    formS5->addRow("最大迭代次数:", spinIcpIter);
    formS5->addRow("适应度阈值:", spinIcpEpsilon);
    formS5->addRow("平移收敛阈值:", spinIcpTransEps);
    paramMainLayout->addWidget(grpS5);

    // --- S6: 去噪/滤波与网格化 ---
    QGroupBox *grpS6 = new QGroupBox("S6: 去噪/滤波与网格化");
    QFormLayout *formS6 = new QFormLayout(grpS6);
    formS6->setLabelAlignment(Qt::AlignLeft);

    spinSorMeanK = new QDoubleSpinBox();
    spinSorMeanK->setRange(1, 200);
    spinSorMeanK->setValue(10);
    spinSorMeanK->setSingleStep(1);
    spinSorMeanK->setDecimals(0);

    spinSorStdMul = new QDoubleSpinBox();
    spinSorStdMul->setRange(0.01, 5.0);
    spinSorStdMul->setValue(3.0);
    spinSorStdMul->setSingleStep(0.05);
    spinSorStdMul->setDecimals(2);

    spinVoxelSize = new QDoubleSpinBox();
    spinVoxelSize->setDecimals(4);
    spinVoxelSize->setValue(0.0);
    spinVoxelSize->setSingleStep(0.001);

    // GP3搜索半径
    spinGp3Radius = new QDoubleSpinBox();
    spinGp3Radius->setRange(0.001, 5.0);
    spinGp3Radius->setValue(2.0);
    spinGp3Radius->setSingleStep(0.001);
    spinGp3Radius->setDecimals(3);
    spinGp3Radius->setToolTip("贪婪投影三角化的搜索半径，物体越大该值应越大");

    // 网格截断距离
    spinMeshTruncationDist = new QDoubleSpinBox();
    spinMeshTruncationDist->setRange(0.0, 50.0);
    spinMeshTruncationDist->setValue(3.0);
    spinMeshTruncationDist->setSingleStep(0.5);
    spinMeshTruncationDist->setDecimals(1);
    spinMeshTruncationDist->setToolTip("截断距离(0=不截断)：移除离点云超过此距离的网格面，防止Poisson在无数据区域产生闭合气泡");

    // Poisson 深度
    spinPoissonDepth = new QSpinBox();
    spinPoissonDepth->setRange(6, 12);
    spinPoissonDepth->setValue(8);
    spinPoissonDepth->setToolTip("泊松八叉树深度(6~12)：值越小表面越光滑但细节越少，毛刺严重时降到7");

    // Poisson 点权重
    spinPoissonPointWeight = new QDoubleSpinBox();
    spinPoissonPointWeight->setRange(0.5, 8.0);
    spinPoissonPointWeight->setValue(3.0);
    spinPoissonPointWeight->setSingleStep(0.5);
    spinPoissonPointWeight->setDecimals(1);
    spinPoissonPointWeight->setToolTip("泊松插值权重(0.5~8.0)：值越大拟合越紧密");

    // 网格方法选择
    cmbMeshMethod = new QComboBox();
    cmbMeshMethod->addItem("泊松 (水密闭合)");
    cmbMeshMethod->addItem("GP3 (按点云, 适合非闭合)");
    cmbMeshMethod->setCurrentIndex(1); // 默认GP3
    cmbMeshMethod->setToolTip("GP3仅在有点云处生成三角面,适合转台扫描的天然非闭合数据。泊松生成水密闭合面。");

    formS6->addRow("网格方法:", cmbMeshMethod);
    formS6->addRow("统计滤波K邻域:", spinSorMeanK);
    formS6->addRow("标准差倍数:", spinSorStdMul);
    formS6->addRow("体素大小(0禁):", spinVoxelSize);
    formS6->addRow("GP3搜索半径:", spinGp3Radius);
    formS6->addRow("泊松深度(6~12):", spinPoissonDepth);
    formS6->addRow("泊松点权重:", spinPoissonPointWeight);
    formS6->addRow("网格截断距离(0关):", spinMeshTruncationDist);
    paramMainLayout->addWidget(grpS6);

    paramScroll->setWidget(paramContainer);

    // ==================== 3. 组装左侧面板 (核心修复) ====================
    tab5CtrlVLayout->addWidget(grpReconCtrl);  // 必须先加按钮组
    tab5CtrlVLayout->addWidget(new QLabel("算法参数设置 (已内置安全阈值):"));
    tab5CtrlVLayout->addWidget(paramScroll, 1); // 再加滚动面板

    // 进度条
    progressRecon = new QProgressBar(); progressRecon->setValue(0);
    tab5CtrlVLayout->addWidget(progressRecon);

    // 4. 右侧辅助显示区 (图像预览 + 日志)
    QWidget *tab5DispPanel = new QWidget();
    QVBoxLayout *tab5DispLayout = new QVBoxLayout(tab5DispPanel);
    tab5DispLayout->setContentsMargins(6, 6, 6, 6);
    tab5DispLayout->setSpacing(6);

    tab5DispLayout->addWidget(new QLabel("当前处理图像对预览:"));
    QHBoxLayout *previewLayout = new QHBoxLayout();
    QString imgLabelStyle2 = Theme::imageLabelStyle(160, 120);
    lblReconLeftView = new QLabel(); lblReconLeftView->setAlignment(Qt::AlignCenter); lblReconLeftView->setStyleSheet(imgLabelStyle2); lblReconLeftView->setMinimumSize(160, 120); lblReconLeftView->setText("左"); lblReconLeftView->setScaledContents(true);
    lblReconRightView = new QLabel(); lblReconRightView->setAlignment(Qt::AlignCenter); lblReconRightView->setStyleSheet(imgLabelStyle2); lblReconRightView->setMinimumSize(160, 120); lblReconRightView->setText("右"); lblReconRightView->setScaledContents(true);
    previewLayout->addWidget(lblReconLeftView);
    previewLayout->addWidget(lblReconRightView);
    tab5DispLayout->addLayout(previewLayout);
    txtReconLog = new QTextEdit(); txtReconLog->setReadOnly(true); txtReconLog->setFont(QFont(Theme::MONO, 9)); txtReconLog->setStyleSheet(Theme::terminalStyle());
    tab5DispLayout->addWidget(txtReconLog, 1);

    // --- 5. 最终组合 ---
    splitter->addWidget(tab5CtrlPanel);
    splitter->addWidget(m_viewer3D);
    splitter->addWidget(tab5DispPanel);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 3);
    splitter->setStretchFactor(2, 1);
    tabWidget->addTab(tab5, "三维重建");

    // Tab 5 信号连接
    connect(btnLoadReconSeq, &QPushButton::clicked, this, &MainWindow::onLoadReconSeqClicked);
    connect(btnPreviewSingleFrame, &QPushButton::clicked, this, &MainWindow::onPreviewSingleFrameClicked); // 新增
    connect(btnStartRecon, &QPushButton::clicked, this, &MainWindow::onStartReconstructionClicked);
    connect(btnSaveRecon, &QPushButton::clicked, this, &MainWindow::onSaveReconResultClicked);
    connect(btnSelectRoiLeft, &QPushButton::clicked, this, &MainWindow::onSelectRoiLeftClicked);
    connect(btnSelectRoiRight, &QPushButton::clicked, this, &MainWindow::onSelectRoiRightClicked);
    connect(btnOpenViewer, &QPushButton::clicked, this, &MainWindow::onOpenStandaloneViewer);
    connect(btnShowPointCloudWindow, &QPushButton::clicked, this, &MainWindow::onShowPointCloudWindowClicked);
    connect(tabWidget, &QTabWidget::currentChanged, this, [this](int index) {
        if (index == 4) { m_viewer3D->setFocus(); }
    });
}

// ================= Tab 5: 三维重建 槽函数 =================

void MainWindow::applyCalibAndParamsToBuilder()
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
    calibData.R_rect_L = m_R1.empty() ? cv::Mat::eye(3, 3, CV_64F) : m_R1.clone();
    calibData.R_rect_R = m_R2.empty() ? cv::Mat::eye(3, 3, CV_64F) : m_R2.clone();

    if (m_IsRectified && !m_P1.empty()) {
        calibData.P1_rectified = m_P1.clone();
        calibData.P2_rectified = m_P2.clone();
    }

    calibData.laser_plane_coeff = Eigen::Vector4d(
        m_LaserPlaneEquation[0], m_LaserPlaneEquation[1],
        m_LaserPlaneEquation[2], m_LaserPlaneEquation[3]);

    bool axisDirValid = !m_rotAxisDirection.empty() && (cv::norm(m_rotAxisDirection) > 0.1);
    bool axisPointValid = !m_rotAxisPoint.empty() && (cv::norm(m_rotAxisPoint) > 0.01);
    bool axisValid = axisDirValid && axisPointValid;
    calibData.turntable_axis = axisDirValid
        ? Eigen::Vector3d(m_rotAxisDirection.at<double>(0,0),
                          m_rotAxisDirection.at<double>(1,0),
                          m_rotAxisDirection.at<double>(2,0))
        : Eigen::Vector3d(0, 1, 0);

    if (axisValid && !m_R_base.empty() && !m_T_base.empty()) {
        cv::Mat R_base_inv = m_R_base.t();
        calibData.R_cam2turntable = R_base_inv.clone();
        calibData.T_cam2turntable = -R_base_inv * m_T_base;
    } else {
        calibData.R_cam2turntable = cv::Mat::eye(3, 3, CV_64F);
        calibData.T_cam2turntable = cv::Mat::zeros(3, 1, CV_64F);
    }
    m_builder->setCalibrationData(calibData);

    ReconstructionParams params;
    params.s1_method = cmbS1Method->currentIndex();
    params.lab_a_threshold = spinLabA->value();
    params.use_steger = chkUseSteger->isChecked();
    params.steger_sigma = spinStegerSigma->value();
    params.steger_t_max = spinStegerTMax->value();
    params.steger_edge_offset_enable = chkOverexposedEnable->isChecked();
    params.steger_overexposed_l_thresh = spinOverexposedL->value();
    params.steger_edge_offset_sigma = spinEdgeOffsetSigma->value();
    params.min_segment_length = spinMorphSigma->value();
    params.sample_step = spinSampleStep->value();
    params.epipolar_threshold = spinEpipolarThresh->value();
    params.depth_min = spinDepthMin->value();
    params.depth_max = spinDepthMax->value();
    params.disparity_break_threshold = spinDisparityBreak->value();
    params.dp_skip_penalty = spinDpSkipPenalty->value();
    params.dp_smooth_weight = spinDpSmoothWeight->value();
    params.use_icp = chkUseIcp->isChecked();
    params.icp_max_correspondence_distance = spinIcpMaxDist->value();
    params.icp_max_iterations = spinIcpIter->value();
    params.icp_axis_trust = spinIcpAxisTrust->value();
    params.icp_euclidean_fitness_epsilon = spinIcpEpsilon->value();
    params.icp_translation_epsilon = spinIcpTransEps->value();
    params.sor_mean_k = spinSorMeanK->value();
    params.sor_std_dev_mul = spinSorStdMul->value();
    params.voxel_leaf_size = spinVoxelSize->value();
    params.gp3_radius = spinGp3Radius->value();
    params.mesh_truncation_distance = spinMeshTruncationDist->value();
    params.mesh_method = cmbMeshMethod->currentIndex();
    params.poisson_depth = spinPoissonDepth->value();
    params.poisson_point_weight = spinPoissonPointWeight->value();

    params.roi_left = (m_roiLeft.width > 0 && m_roiRight.width > 0) ? m_roiLeft : cv::Rect();
    params.roi_right = (m_roiRight.width > 0 && m_roiLeft.width > 0) ? m_roiRight : cv::Rect();

    m_builder->setReconstructionParams(params);
}

void MainWindow::onLoadReconSeqClicked()
{
    QString folderL = QFileDialog::getExistingDirectory(this, "选择左图序列文件夹", dirLeftPointCloud);
    if(folderL.isEmpty()) return;
    QString folderR = QFileDialog::getExistingDirectory(this, "选择右图序列文件夹", dirRightPointCloud);
    if(folderR.isEmpty()) return;

    // 借用原有的工具函数获取文件列表
    m_reconLeftPaths = getFilesInFolder(folderL, 999);
    m_reconRightPaths = getFilesInFolder(folderR, 999);

    if(m_reconLeftPaths.size() != m_reconRightPaths.size() || m_reconLeftPaths.isEmpty()) {
        QMessageBox::warning(this, "文件错误", "左右图像数量不一致或文件夹为空！");
        return;
    }

    spinViewCount->setValue(m_reconLeftPaths.size());
    txtReconLog->append(QString("[%1] 成功加载 %2 对图像序列。")
                        .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
                        .arg(m_reconLeftPaths.size()));
}

void MainWindow::onStartReconstructionClicked()
{
    if (m_reconLeftPaths.isEmpty()) {
        QMessageBox::warning(this, "错误", "请先导入旋转图像序列！");
        return;
    }

    txtReconLog->clear();
    m_viewer3D->clearViewer();
    btnStartRecon->setEnabled(false);
    btnSaveRecon->setEnabled(false);

    txtReconLog->append("====== [前置诊断] 开始三维重建检查 ======");
    double angleStep = spinAngleStep->value();
    bool axisDirValid = !m_rotAxisDirection.empty() && (cv::norm(m_rotAxisDirection) > 0.1);
    bool axisPointValid = !m_rotAxisPoint.empty() && (cv::norm(m_rotAxisPoint) > 0.01);
    bool axisValid = axisDirValid && axisPointValid;  // 完整标定 = 方向 + 轴点

    txtReconLog->append(QString("1. 图像序列: %1 对 | 步长: %2°").arg(m_reconLeftPaths.size()).arg(angleStep));
    txtReconLog->append(QString("2. 相机参数: %1 | 立体外参: %2").arg(m_CameraMatrixL.empty() ? "❌" : "✅").arg(m_R.empty() ? "❌" : "✅"));
    txtReconLog->append(QString("3. 旋转轴标定: %1").arg(axisValid ? "✅ 完整" : (axisDirValid ? "⚠️ 仅方向" : "❌ 未标定")));

    applyCalibAndParamsToBuilder();

    // ======================== 完整参数与误差总览 ========================
    txtReconLog->append("\n====== [参数总览] 全部标定与重建参数 ======");

    // -- 标定误差 --
    txtReconLog->append(QString("\n--- 标定精度 ---"));
    txtReconLog->append(QString("双目标定 RMS: %1 px  |  基线: %2 mm")
        .arg(m_StereoRms, 0, 'f', 4).arg(cv::norm(m_T), 0, 'f', 2));
    if (cv::norm(m_LaserPlaneEquation) > 1e-6f) {
        txtReconLog->append(QString("光平面: %1x + %2y + %3z + %4 = 0")
            .arg(m_LaserPlaneEquation[0], 0, 'f', 4)
            .arg(m_LaserPlaneEquation[1], 0, 'f', 4)
            .arg(m_LaserPlaneEquation[2], 0, 'f', 4)
            .arg(m_LaserPlaneEquation[3], 0, 'f', 4));
    } else {
        txtReconLog->append("光平面: ❌ 未标定");
    }
    if (axisDirValid) {
        double nx = m_rotAxisDirection.at<double>(0,0);
        double ny = m_rotAxisDirection.at<double>(1,0);
        double nz = m_rotAxisDirection.at<double>(2,0);
        txtReconLog->append(QString("旋转轴方向: [%1, %2, %3]")
            .arg(nx, 0, 'f', 4).arg(ny, 0, 'f', 4).arg(nz, 0, 'f', 4));
        if (axisPointValid) {
            txtReconLog->append(QString("旋转轴点: [%1, %2, %3] mm")
                .arg(m_rotAxisPoint.at<double>(0,0), 0, 'f', 2)
                .arg(m_rotAxisPoint.at<double>(1,0), 0, 'f', 2)
                .arg(m_rotAxisPoint.at<double>(2,0), 0, 'f', 2));
        }
    }

    // -- 相机内参 --
    auto fmtMat33 = [](const cv::Mat& M) -> QString {
        if (M.empty() || M.rows < 3 || M.cols < 3) return "未标定";
        return QString("[%1 %2 %3]\n               [%4 %5 %6]\n               [%7 %8 %9]")
            .arg(M.at<double>(0,0), 6, 'f', 1).arg(M.at<double>(0,1), 6, 'f', 1).arg(M.at<double>(0,2), 6, 'f', 1)
            .arg(M.at<double>(1,0), 6, 'f', 1).arg(M.at<double>(1,1), 6, 'f', 1).arg(M.at<double>(1,2), 6, 'f', 1)
            .arg(M.at<double>(2,0), 6, 'f', 1).arg(M.at<double>(2,1), 6, 'f', 1).arg(M.at<double>(2,2), 6, 'f', 1);
    };
    auto fmtDist = [](const cv::Mat& D) -> QString {
        if (D.empty() || D.total() < 5) return "未标定";
        return QString("[%1, %2, %3, %4, %5]")
            .arg(D.at<double>(0), 0, 'f', 4).arg(D.at<double>(1), 0, 'f', 4)
            .arg(D.at<double>(2), 0, 'f', 4).arg(D.at<double>(3), 0, 'f', 4)
            .arg(D.at<double>(4), 0, 'f', 4);
    };
    txtReconLog->append(QString("\n--- 相机内参 ---"));
    txtReconLog->append(QString("左内参 K_L:\n               %1").arg(fmtMat33(m_CameraMatrixL)));
    txtReconLog->append(QString("左畸变 D_L: %1").arg(fmtDist(m_DistCoeffsL)));
    txtReconLog->append(QString("右内参 K_R:\n               %1").arg(fmtMat33(m_CameraMatrixR)));
    txtReconLog->append(QString("右畸变 D_R: %1").arg(fmtDist(m_DistCoeffsR)));
    txtReconLog->append(QString("立体校正: %1").arg(m_IsRectified ? "✅ 已校正" : "❌ 未校正"));
    if (!m_R.empty()) {
        txtReconLog->append(QString("立体 R:\n               %1").arg(fmtMat33(m_R)));
    }
    if (!m_T.empty()) {
        txtReconLog->append(QString("立体 T: [%1, %2, %3]")
            .arg(m_T.at<double>(0), 0, 'f', 2).arg(m_T.at<double>(1), 0, 'f', 2).arg(m_T.at<double>(2), 0, 'f', 2));
    }

    // -- S1 光条提取参数 --
    txtReconLog->append(QString("\n--- S1 光条提取 ---"));
    txtReconLog->append(QString("算法: %1 | sigma: %2 | t_max: %3 px | LAB_A: %4")
        .arg(chkUseSteger->isChecked() ? "Steger" : "灰度重心")
        .arg(spinStegerSigma->value()).arg(spinStegerTMax->value()).arg(spinLabA->value()));
    txtReconLog->append(QString("过曝恢复: %1 | L阈值: %2 | 偏移系数: %3")
        .arg(chkOverexposedEnable->isChecked() ? "开" : "关")
        .arg(spinOverexposedL->value()).arg(spinEdgeOffsetSigma->value()));

    // -- S2 匹配参数 --
    txtReconLog->append(QString("\n--- S2 极线匹配 ---"));
    txtReconLog->append(QString("极线容差: %1 px | 视差跳变: %2 px | DP跳过: %3 | DP平滑: %4")
        .arg(spinEpipolarThresh->value()).arg(spinDisparityBreak->value())
        .arg(spinDpSkipPenalty->value()).arg(spinDpSmoothWeight->value()));
    txtReconLog->append(QString("深度范围: %1 ~ %2 mm")
        .arg(spinDepthMin->value()).arg(spinDepthMax->value()));

    // -- S5 ICP 参数 --
    txtReconLog->append(QString("\n--- S5 ICP 配准 ---"));
    txtReconLog->append(QString("最大对应距离: %1 mm | 最大迭代: %2 | 收敛阈值: %3")
        .arg(spinIcpMaxDist->value()).arg(spinIcpIter->value())
        .arg(spinIcpEpsilon->value()));

    // -- S6 网格参数 --
    txtReconLog->append(QString("\n--- S6 网格重建 ---"));
    txtReconLog->append(QString("泊松深度: %1 | 点权重: %2 | 截断距离: %3 mm")
        .arg(spinPoissonDepth->value()).arg(spinPoissonPointWeight->value()).arg(spinMeshTruncationDist->value()));
    txtReconLog->append(QString("SOR K: %1 | SOR std: %2 | 体素: %3 mm")
        .arg(spinSorMeanK->value()).arg(spinSorStdMul->value()).arg(spinVoxelSize->value()));

    // -- 精度预估 --
    txtReconLog->append(QString("\n--- 精度预估 ---"));
    double baseline = cv::norm(m_T);
    double rough_f = m_CameraMatrixL.empty() ? 0 : m_CameraMatrixL.at<double>(0,0);
    if (baseline > 0 && rough_f > 0) {
        double z_est = 400.0; // 典型工作距
        double disparity_per_px = z_est * z_est / (rough_f * baseline);
        txtReconLog->append(QString("基线/焦距: B=%1 f≈%2 → Z=%3mm 时 1px≈%4mm 深度误差")
            .arg(baseline, 0, 'f', 1).arg(rough_f, 0, 'f', 0)
            .arg(z_est, 0, 'f', 0).arg(disparity_per_px, 0, 'f', 2));
    }
    if (m_StereoRms > 0.5) {
        txtReconLog->append(QString("⚠️ 双目标定 RMS > 0.5px，建议重新标定"));
    }
    if (baseline < 80) {
        txtReconLog->append(QString("⚠️ 基线 < 80mm，深度精度受限"));
    }
    txtReconLog->append("========================================\n");

    txtReconLog->append("====== [流水线] 开始 S1~S4 逐视角处理 ======");
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> allViewClouds;
    std::vector<double> viewAngles;
    int totalViews = m_reconLeftPaths.size();

    for (int i = 0; i < totalViews; ++i) {
        if (!this->isVisible()) {
            btnStartRecon->setEnabled(true);
            return;
        }

        cv::Mat imgL = cv::imread(m_reconLeftPaths[i].toStdString()), imgR = cv::imread(m_reconRightPaths[i].toStdString());
        if (imgL.empty() || imgR.empty()) {
            txtReconLog->append(QString("[视角 %1] ⚠️ 读取失败").arg(i+1));
            continue;
        }

        if (m_IsRectified && !m_MapL1.empty()) {
            cv::remap(imgL, imgL, m_MapL1, m_MapL2, cv::INTER_LINEAR);
            cv::remap(imgR, imgR, m_MapR1, m_MapR2, cv::INTER_LINEAR);
        }

        lblReconLeftView->setPixmap(QPixmap::fromImage(mat2QImage(imgL)));
        lblReconRightView->setPixmap(QPixmap::fromImage(mat2QImage(imgR)));

        double currentAngle = i * angleStep;
        pcl::PointCloud<pcl::PointXYZ>::Ptr singleCloud = m_builder->processSingleView(imgL, imgR, currentAngle);

        if (!singleCloud->empty()) {
            allViewClouds.push_back(singleCloud);
            viewAngles.push_back(currentAngle);
            txtReconLog->append(QString("[%1/%2] %3").arg(i+1).arg(totalViews).arg(m_builder->lastDetailLog));
        } else {
            txtReconLog->append(QString("[%1/%2] ❌ %3").arg(i+1).arg(totalViews).arg(m_builder->lastDetailLog));
        }

        progressRecon->setValue(static_cast<int>((i + 1) * 100.0 / totalViews));
        QApplication::processEvents();
    }

    txtReconLog->append(QString("====== S1~S4 结束: 有效视角 %1 个 ======\n").arg(allViewClouds.size()));

    if (allViewClouds.empty()) {
        txtReconLog->append("❌ 致命错误: 无有效点云！");
        btnStartRecon->setEnabled(true);
        return;
    }

    // ====================================================================
    // 旋转轴基准点与方向提取 (支持完整标定 / 仅方向手动设置 / 安全降级)
    // ====================================================================
    txtReconLog->append("====== [S5 ICP 配准] 开始 ======");

    Eigen::Vector3f axis_point = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis_dir = Eigen::Vector3f(0.0f, 1.0f, 0.0f); // 默认 Y 轴

    if (axisValid) {
        // 【完整标定模式】：使用自动标定的方向 + 轴点
        axis_dir = Eigen::Vector3f(
            static_cast<float>(m_rotAxisDirection.at<double>(0,0)),
            static_cast<float>(m_rotAxisDirection.at<double>(1,0)),
            static_cast<float>(m_rotAxisDirection.at<double>(2,0))
        );
        axis_point = Eigen::Vector3f(
            static_cast<float>(m_rotAxisPoint.at<double>(0,0)),
            static_cast<float>(m_rotAxisPoint.at<double>(1,0)),
            static_cast<float>(m_rotAxisPoint.at<double>(2,0))
        );
        txtReconLog->append(QString(">>> [精准模式] 使用完整标定结果"));
        txtReconLog->append(QString("    轴方向: [%1, %2, %3]")
            .arg(axis_dir.x(), 0, 'f', 4).arg(axis_dir.y(), 0, 'f', 4).arg(axis_dir.z(), 0, 'f', 4));
        txtReconLog->append(QString("    轴点: [%1, %2, %3] mm")
            .arg(axis_point.x(), 0, 'f', 2).arg(axis_point.y(), 0, 'f', 2).arg(axis_point.z(), 0, 'f', 2));
    } else {
        // 【安全降级模式】：无任何标定结果
        axis_point = Eigen::Vector3f(30.0f, 0.0f, 250.0f);

        txtReconLog->append(">>> [安全降级] 未检测到旋转轴标定结果！");
        txtReconLog->append(QString(">>> 降级策略: 假设转台中心在左相机坐标系 Z 轴前方 %1 mm 处").arg(250.0f));
        txtReconLog->append(">>> 降级策略: 旋转方向默认为竖直 Y 轴 [0, 1, 0]");
        txtReconLog->append(">>> 提示: 若拼接呈螺旋状错位，说明实际距离偏差大，请在Tab4进行旋转轴自动标定");
    }

    // 2. 调用修改后的多视角配准接口 (传入基准点和方向)
    m_builder->processGlobal(allViewClouds, viewAngles, axis_point, axis_dir);
    // 输出 S5 ICP 诊断
    if (!m_builder->lastDetailLog.isEmpty())
        txtReconLog->append(m_builder->lastDetailLog);
    // ====================================================================

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud = m_builder->getFinalPointCloud();
    pcl::PolygonMesh finalMesh = m_builder->getFinalMesh();

    txtReconLog->append(QString("====== [S6 结束] 点云: %1 | 网格: %2 ======").arg(finalCloud->size()).arg(finalMesh.polygons.size()));

    if (!finalCloud->empty()) {
        m_viewer3D->showPointCloud(finalCloud, "final_cloud");
        m_viewer3D->showMesh(finalMesh, "final_mesh");
        m_viewer3D->resetCamera();
    } else {
        txtReconLog->append("❌ 警告: 最终点云为空！");
    }

    txtReconLog->append("三维重建流程执行完毕！");
    btnStartRecon->setEnabled(true);
    btnSaveRecon->setEnabled(true);

    // 自动弹出重建结果窗口 (点云 + 网格)
    if (!finalCloud->empty()) {
        QDialog *resultDlg = new QDialog(this);
        resultDlg->resize(1000, 750);
        resultDlg->setWindowTitle("三维重建结果 — 点云 + 泊松网格");
        resultDlg->setStyleSheet("background-color: " + QString(Theme::BG_CARD) + ";");
        resultDlg->setAttribute(Qt::WA_DeleteOnClose);

        QVBoxLayout *dlgLayout = new QVBoxLayout(resultDlg);
        dlgLayout->setContentsMargins(0, 0, 0, 0);

        PointCloudViewer *resultViewer = new PointCloudViewer(resultDlg);
        dlgLayout->addWidget(resultViewer);

        resultViewer->showPointCloud(finalCloud, "result_cloud");
        resultViewer->resetCamera();

        resultDlg->show();
        txtReconLog->append(">>> 已弹出重建结果3D视图窗口");
    }
}

void MainWindow::onSaveReconResultClicked()
{
    if (!m_builder || m_builder->getFinalPointCloud()->empty()) {
        QMessageBox::warning(this, "保存失败", "没有可保存的点云数据。");
        return;
    }

    QString pathPly = QFileDialog::getSaveFileName(this, "保存点云文件", "reconstruction_result.ply", "PLY Files (*.ply)");
    if (!pathPly.isEmpty()) {
        pcl::io::savePLYFile(pathPly.toStdString(), *m_builder->getFinalPointCloud());
        txtReconLog->append(QString("点云已保存至: %1").arg(pathPly));

        QString pathVtk = QFileDialog::getSaveFileName(this, "保存网格文件", "reconstruction_result.vtk", "VTK Files (*.vtk)");
        if (!pathVtk.isEmpty()) {
            pcl::io::saveVTKFile(pathVtk.toStdString(), m_builder->getFinalMesh());
            txtReconLog->append(QString("网格已保存至: %1").arg(pathVtk));
        }
    }
}

void MainWindow::onSelectRoiLeftClicked() {
    if (m_reconLeftPaths.isEmpty()) { QMessageBox::warning(this, "提示", "请先导入旋转图像序列！"); return; }
    if (selectRoiOnImage(m_reconLeftPaths[0], "框选左图 ROI (原始分辨率)", m_roiLeft, btnSelectRoiLeft))
        txtReconLog->append(QString("[ROI设置] 左图 ROI 已更新: %1x%2 起始(%3,%4)")
                             .arg(m_roiLeft.width).arg(m_roiLeft.height).arg(m_roiLeft.x).arg(m_roiLeft.y));
}

void MainWindow::onSelectRoiRightClicked() {
    if (m_reconRightPaths.isEmpty()) { QMessageBox::warning(this, "提示", "请先导入旋转图像序列！"); return; }
    if (selectRoiOnImage(m_reconRightPaths[0], "框选右图 ROI (原始分辨率)", m_roiRight, btnSelectRoiRight))
        txtReconLog->append(QString("[ROI设置] 右图 ROI 已更新: %1x%2 起始(%3,%4)")
                             .arg(m_roiRight.width).arg(m_roiRight.height).arg(m_roiRight.x).arg(m_roiRight.y));
}

void MainWindow::onOpenStandaloneViewer()
{
    // 创建独立的3D视图弹窗
    QDialog *dlg = new QDialog(this);
    dlg->resize(1000, 750);
    dlg->setWindowTitle("独立3D视图 — 点云 + 网格");
    dlg->setStyleSheet("background-color: " + QString(Theme::BG_CARD) + ";");
    dlg->setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout *layout = new QVBoxLayout(dlg);
    layout->setContentsMargins(0, 0, 0, 0);

    PointCloudViewer *standaloneViewer = new PointCloudViewer(dlg);
    layout->addWidget(standaloneViewer);

    if (m_builder) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m_builder->getFinalPointCloud();
        pcl::PolygonMesh mesh = m_builder->getFinalMesh();
        if (cloud && !cloud->empty()) {
            standaloneViewer->showPointCloud(cloud, "test_cloud");
            if (mesh.polygons.size() > 0) {
                standaloneViewer->showMesh(mesh, "test_mesh");
            }
            standaloneViewer->resetCamera();
        }
    }

    dlg->exec();
}

void MainWindow::onShowPointCloudWindowClicked()
{
    QDialog *dlg = new QDialog(this);
    dlg->resize(1000, 750);
    dlg->setWindowTitle("点云重建窗口 — 仅点云");
    dlg->setStyleSheet("background-color: " + QString(Theme::BG_CARD) + ";");
    dlg->setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout *layout = new QVBoxLayout(dlg);
    layout->setContentsMargins(0, 0, 0, 0);

    PointCloudViewer *viewer = new PointCloudViewer(dlg);
    layout->addWidget(viewer);

    if (m_builder) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m_builder->getFinalPointCloud();
        if (cloud && !cloud->empty()) {
            viewer->showPointCloud(cloud, "pointcloud_only");
            viewer->resetCamera();
        }
    }

    dlg->exec();
}

void MainWindow::onPreviewSingleFrameClicked()
{
    txtReconLog->append("====== [单帧深度调试] 开始执行 ======");
    if (m_reconLeftPaths.isEmpty() || m_reconRightPaths.isEmpty()) {
        QMessageBox::warning(this, "预览失败", "请先点击 [1. 导入旋转图像序列]！");
        return;
    }
    int idx = spinPreviewIndex->value();
    int maxIdx = qMin(m_reconLeftPaths.size(), m_reconRightPaths.size()) - 1;
    if (idx > maxIdx) {
        QMessageBox::warning(this, "预览失败", QString("帧号 %1 超出范围！最大帧号为 %2").arg(idx).arg(maxIdx));
        return;
    }
    cv::Mat imgL = cv::imread(m_reconLeftPaths[idx].toStdString());
    cv::Mat imgR = cv::imread(m_reconRightPaths[idx].toStdString());
    if (imgL.empty() || imgR.empty()) return;

    // ==================== 前置准备 ====================
    if (m_IsRectified && !m_MapL1.empty()) {
        cv::Mat imgL_rect, imgR_rect;
        cv::remap(imgL, imgL_rect, m_MapL1, m_MapL2, cv::INTER_LINEAR);
        cv::remap(imgR, imgR_rect, m_MapR1, m_MapR2, cv::INTER_LINEAR);
        imgL = imgL_rect; imgR = imgR_rect;
    }

    applyCalibAndParamsToBuilder();

    // ==================== 创建调试弹窗 (精简版：S1提取 + S3重投影) ====================
    static QWidget *s_debugDlg = nullptr;
    if (s_debugDlg) { s_debugDlg->close(); delete s_debugDlg; }
    QWidget *debugDialog = new QWidget(nullptr, Qt::Window);
    debugDialog->setAttribute(Qt::WA_DeleteOnClose);
    s_debugDlg = debugDialog;
    debugDialog->setWindowTitle(QString("调试 #%1").arg(idx));
    debugDialog->setFixedSize(1200, 820); debugDialog->setStyleSheet("background-color: " + QString(Theme::BG_CARD) + "; color: " + QString(Theme::TEXT_PRIMARY) + ";");
    QVBoxLayout *dlgOuter = new QVBoxLayout(debugDialog); dlgOuter->setContentsMargins(4, 4, 4, 4);
    QHBoxLayout *dlgMainLayout = new QHBoxLayout(); dlgMainLayout->setSpacing(4);
    QWidget *imgContainerWidget = new QWidget(); imgContainerWidget->setFixedSize(820, 780);
    QGridLayout *gridLayout = new QGridLayout(imgContainerWidget); gridLayout->setSpacing(2);
    QString titleStyle = "border: 1px solid " + QString(Theme::BORDER) + "; background-color: " + QString(Theme::BG_INPUT) + "; color: " + QString(Theme::ACCENT) + "; font-weight: bold; padding: 2px; font-size: 11px;";
    QString imgStyle = "border: 1px solid " + QString(Theme::BG_HOVER) + "; background-color: #000;";
    QLabel *lblTitleS1L = new QLabel("S1 光条提取 L"); lblTitleS1L->setStyleSheet(titleStyle); lblTitleS1L->setAlignment(Qt::AlignCenter);
    QLabel *lblTitleS1R = new QLabel("S1 光条提取 R"); lblTitleS1R->setStyleSheet(titleStyle); lblTitleS1R->setAlignment(Qt::AlignCenter);
    QLabel *lblTitleS2L = new QLabel("S2 极线匹配 L"); lblTitleS2L->setStyleSheet(titleStyle + " color: #ffdd44;"); lblTitleS2L->setAlignment(Qt::AlignCenter);
    QLabel *lblTitleS2R = new QLabel("S2 极线匹配 R"); lblTitleS2R->setStyleSheet(titleStyle + " color: #ffdd44;"); lblTitleS2R->setAlignment(Qt::AlignCenter);
    QLabel *lblTitleS3L = new QLabel("S3 深度重投影 L"); lblTitleS3L->setStyleSheet(titleStyle + " color: #1eff00;"); lblTitleS3L->setAlignment(Qt::AlignCenter);
    QLabel *lblTitleS3R = new QLabel("S3 深度重投影 R"); lblTitleS3R->setStyleSheet(titleStyle + " color: #1eff00;"); lblTitleS3R->setAlignment(Qt::AlignCenter);
    QLabel *lblImgS1L = new QLabel(); lblImgS1L->setStyleSheet(imgStyle); lblImgS1L->setScaledContents(true); lblImgS1L->setFixedSize(400, 220);
    QLabel *lblImgS1R = new QLabel(); lblImgS1R->setStyleSheet(imgStyle); lblImgS1R->setScaledContents(true); lblImgS1R->setFixedSize(400, 220);
    QLabel *lblImgS2L = new QLabel(); lblImgS2L->setStyleSheet(imgStyle); lblImgS2L->setScaledContents(true); lblImgS2L->setFixedSize(400, 220);
    QLabel *lblImgS2R = new QLabel(); lblImgS2R->setStyleSheet(imgStyle); lblImgS2R->setScaledContents(true); lblImgS2R->setFixedSize(400, 220);
    QLabel *lblImgS3L = new QLabel(); lblImgS3L->setStyleSheet(imgStyle); lblImgS3L->setScaledContents(true); lblImgS3L->setFixedSize(400, 220);
    QLabel *lblImgS3R = new QLabel(); lblImgS3R->setStyleSheet(imgStyle); lblImgS3R->setScaledContents(true); lblImgS3R->setFixedSize(400, 220);
    gridLayout->addWidget(lblTitleS1L, 0, 0); gridLayout->addWidget(lblImgS1L, 1, 0);
    gridLayout->addWidget(lblTitleS1R, 0, 1); gridLayout->addWidget(lblImgS1R, 1, 1);
    gridLayout->addWidget(lblTitleS2L, 2, 0); gridLayout->addWidget(lblImgS2L, 3, 0);
    gridLayout->addWidget(lblTitleS2R, 2, 1); gridLayout->addWidget(lblImgS2R, 3, 1);
    gridLayout->addWidget(lblTitleS3L, 4, 0); gridLayout->addWidget(lblImgS3L, 5, 0);
    gridLayout->addWidget(lblTitleS3R, 4, 1); gridLayout->addWidget(lblImgS3R, 5, 1);
    dlgMainLayout->addWidget(imgContainerWidget);
    PointCloudViewer *debugViewer = new PointCloudViewer(debugDialog); dlgMainLayout->addWidget(debugViewer, 1);
    dlgOuter->addLayout(dlgMainLayout);
    // 导航按钮行
    QHBoxLayout *navLayout = new QHBoxLayout();
    QPushButton *btnPrev = new QPushButton("◀ 上一帧"); btnPrev->setStyleSheet("background-color: " + QString(Theme::BG_HOVER) + "; color: #fff; padding: 6px 12px;");
    QPushButton *btnNext = new QPushButton("下一帧 ▶"); btnNext->setStyleSheet("background-color: " + QString(Theme::BG_HOVER) + "; color: #fff; padding: 6px 12px;");
    QLabel *lblNavInfo = new QLabel(QString("当前: #%1 / %2 帧").arg(idx).arg(maxIdx));
    lblNavInfo->setStyleSheet("color: " + QString(Theme::TEXT_SECONDARY) + "; padding: 0 12px;");
    navLayout->addWidget(btnPrev); navLayout->addWidget(lblNavInfo); navLayout->addWidget(btnNext); navLayout->addStretch();
    dlgOuter->addLayout(navLayout);
    debugDialog->show();

    // ==================== 核心：生成黑底红线底图 (修复 channel_morph 报错) ====================
    auto getBlackBgRedLaser = [&](const cv::Mat& img) -> cv::Mat {
        cv::Mat lab_img; cv::cvtColor(img, lab_img, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> channels; cv::split(lab_img, channels);
        cv::Mat mask = (channels[1] > spinLabA->value());
        cv::Mat black_bg = cv::Mat::zeros(img.size(), CV_8UC3);
        img.copyTo(black_bg, mask); return black_bg;
    };
    cv::Mat bgL = getBlackBgRedLaser(imgL); cv::Mat bgR = getBlackBgRedLaser(imgR);
    cv::Scalar pointColor(0, 255, 255);

    // ==================== S1 调试 ====================
    cv::Mat drawS1L = bgL.clone(); cv::Mat drawS1R = bgR.clone();
    std::vector<cv::Point2f> pts_left, pts_right;
    cv::Rect roiL = (m_roiLeft.width > 0) ? m_roiLeft : cv::Rect();
    cv::Rect roiR = (m_roiRight.width > 0) ? m_roiRight : cv::Rect();
    m_builder->extractLaserCenter(drawS1L, roiL, pts_left);
    m_builder->extractLaserCenter(drawS1R, roiR, pts_right);
    for (const auto& pt : pts_left) cv::circle(drawS1L, pt, 2, pointColor, -1);
    for (const auto& pt : pts_right) cv::circle(drawS1R, pt, 2, pointColor, -1);
    lblImgS1L->setPixmap(QPixmap::fromImage(mat2QImage(drawS1L))); lblImgS1R->setPixmap(QPixmap::fromImage(mat2QImage(drawS1R)));
    QApplication::processEvents();

    // ==================== S2 调试：极线匹配可视化 ====================
    std::vector<cv::Point2f> match_l, match_r;
    m_builder->epipolarConstraintMatch(pts_left, pts_right, match_l, match_r);
    cv::Mat drawS2L = bgL.clone(); cv::Mat drawS2R = bgR.clone();
    // 给匹配对分配颜色 (HSV色相循环)
    int nMatches = static_cast<int>(match_l.size());
    for (int i = 0; i < nMatches; ++i) {
        cv::Scalar color = cv::Scalar(0, 255, 255); // 黄色
        cv::circle(drawS2L, match_l[i], 3, color, -1);
        cv::circle(drawS2R, match_r[i], 3, color, -1);
        // 序号标签 (每10个标一个避免拥挤)
        if (i % 5 == 0) {
            cv::putText(drawS2L, std::to_string(i), match_l[i] + cv::Point2f(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
            cv::putText(drawS2R, std::to_string(i), match_r[i] + cv::Point2f(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        }
    }
    cv::putText(drawS2L, QString("Matched: %1/%2").arg(nMatches).arg(pts_left.size()).toStdString(),
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(drawS2R, QString("Matched: %1/%2").arg(nMatches).arg(pts_right.size()).toStdString(),
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    lblImgS2L->setPixmap(QPixmap::fromImage(mat2QImage(drawS2L)));
    lblImgS2R->setPixmap(QPixmap::fromImage(mat2QImage(drawS2R)));
    QApplication::processEvents();

    // ==================== S3 调试：深度伪彩色重投影 ====================
    cv::Mat drawS3L = bgL.clone(); cv::Mat drawS3R = bgR.clone();
    pcl::PointCloud<pcl::PointXYZ>::Ptr single_cloud = m_builder->triangulatePoints(match_l, match_r);
    double s3_err_L = -1, s3_err_R = -1;
    int s3_pts = 0;
    if (single_cloud && !single_cloud->empty()) {
        cv::Mat P_left = (m_IsRectified && !m_P1.empty()) ? m_P1 : m_CameraMatrixL;
        cv::Mat P_right = (m_IsRectified && !m_P2.empty()) ? m_P2 : m_CameraMatrixR;
        float z_min = FLT_MAX, z_max = -FLT_MAX;
        for (const auto& pt : single_cloud->points) { if (pt.z < z_min) z_min = pt.z; if (pt.z > z_max) z_max = pt.z; }
        auto depthToColor = [&](float z) -> cv::Scalar { float ratio = (z_max - z_min < 1e-3f) ? 0.5f : (z - z_min) / (z_max - z_min); return cv::Scalar(255, static_cast<int>(255 * (1.0f - ratio)), static_cast<int>(255 * ratio)); };
        double total_error_L = 0, total_error_R = 0;
        for (size_t i = 0; i < single_cloud->size(); ++i) {
            pcl::PointXYZ pt3d = single_cloud->points[i]; cv::Mat p3d_h = (cv::Mat_<double>(4, 1) << pt3d.x, pt3d.y, pt3d.z, 1.0); cv::Scalar color = depthToColor(pt3d.z);
            if (!P_left.empty()) { cv::Mat p2d_L = P_left * p3d_h; double u = p2d_L.at<double>(0,0) / p2d_L.at<double>(2,0), v = p2d_L.at<double>(1,0) / p2d_L.at<double>(2,0); if (u >= 0 && u < drawS3L.cols && v >= 0 && v < drawS3L.rows) { cv::drawMarker(drawS3L, cv::Point(u, v), color, cv::MARKER_CROSS, 8, 2); if (i < match_l.size()) total_error_L += std::sqrt(std::pow(u - match_l[i].x, 2) + std::pow(v - match_l[i].y, 2)); } }
            if (!P_right.empty()) { cv::Mat p2d_R = P_right * p3d_h; double u = p2d_R.at<double>(0,0) / p2d_R.at<double>(2,0), v = p2d_R.at<double>(1,0) / p2d_R.at<double>(2,0); if (u >= 0 && u < drawS3R.cols && v >= 0 && v < drawS3R.rows) { cv::drawMarker(drawS3R, cv::Point(u, v), color, cv::MARKER_CROSS, 8, 2); if (i < match_r.size()) total_error_R += std::sqrt(std::pow(u - match_r[i].x, 2) + std::pow(v - match_r[i].y, 2)); } }
        }
        s3_err_L = total_error_L / single_cloud->size(); s3_err_R = total_error_R / single_cloud->size();
        s3_pts = single_cloud->size();
        cv::putText(drawS3L, QString("Err: %1 px").arg(s3_err_L, 0, 'f', 2).toStdString(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(drawS3R, QString("Err: %1 px").arg(s3_err_R, 0, 'f', 2).toStdString(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        debugViewer->showPointCloud(single_cloud, "debug_cloud"); debugViewer->resetCamera(); debugViewer->update();
    } else {
        cv::putText(drawS3L, "Triangulation Failed!", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    lblImgS3L->setPixmap(QPixmap::fromImage(mat2QImage(drawS3L))); lblImgS3R->setPixmap(QPixmap::fromImage(mat2QImage(drawS3R)));
    lblReconLeftView->setPixmap(QPixmap::fromImage(mat2QImage(drawS3L)).scaled(lblReconLeftView->size(), Qt::KeepAspectRatio));
    lblReconRightView->setPixmap(QPixmap::fromImage(mat2QImage(drawS3R)).scaled(lblReconRightView->size(), Qt::KeepAspectRatio));

    // 导航按钮
    QObject::connect(btnPrev, &QPushButton::clicked, [this, idx, maxIdx]() {
        if (idx > 0) { spinPreviewIndex->setValue(idx - 1); onPreviewSingleFrameClicked(); }
    });
    QObject::connect(btnNext, &QPushButton::clicked, [this, idx, maxIdx]() {
        if (idx < maxIdx) { spinPreviewIndex->setValue(idx + 1); onPreviewSingleFrameClicked(); }
    });

    txtReconLog->append(QString("调试 #%1: S1左%2右%3 → S2%4对 → S3%5点 | 重投影左%6右%7 px")
        .arg(idx).arg(pts_left.size()).arg(pts_right.size()).arg(match_l.size()).arg(s3_pts)
        .arg(s3_err_L > 0 ? QString::number(s3_err_L, 'f', 2) : "-")
        .arg(s3_err_R > 0 ? QString::number(s3_err_R, 'f', 2) : "-"));
}
