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
#include <cfloat> // 【补全】FLT_MAX (S3 深度伪彩色重投影)

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

    // --- 重建后台线程 (与 LaserWorker 同模式: Worker 移入独立 QThread) ---
    qRegisterMetaType<ReconstructionWorker::JobInput>("ReconstructionWorker::JobInput");
    m_reconThread = new QThread(this);
    m_reconWorker = new ReconstructionWorker();
    m_reconWorker->moveToThread(m_reconThread);
    connect(m_reconWorker, &ReconstructionWorker::progress, this, &MainWindow::onReconProgress);
    connect(m_reconWorker, &ReconstructionWorker::logMessage, this, &MainWindow::onReconLog);
    connect(m_reconWorker, &ReconstructionWorker::viewImage, this, &MainWindow::onReconViewImage);
    connect(m_reconWorker, &ReconstructionWorker::finished, this, &MainWindow::onReconstructionFinished);
    connect(m_reconWorker, &ReconstructionWorker::error, this, &MainWindow::onReconstructionError);
    m_reconThread->start();

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

    // 【P0-4】S1~S4 逐视角并行线程数
    spinParallelThreads = new QSpinBox();
    spinParallelThreads->setRange(0, 32);
    spinParallelThreads->setValue(0);
    spinParallelThreads->setSpecialValueText("自动");
    spinParallelThreads->setToolTip("S1~S4 逐视角处理线程数。0=自动(按CPU核心数)。\n"
                                    "各视角相互独立，多线程可显著缩短重建耗时。");

    formReconBasic->addRow("旋转步长:", spinAngleStep);
    formReconBasic->addRow("视角数量:", spinViewCount);
    formReconBasic->addRow("并行线程数:", spinParallelThreads);
    grpReconCtrlLayout->addLayout(formReconBasic);

    btnLoadReconSeq = new QPushButton("1. 导入旋转图像序列");
    btnLoadReconSeq->setStyleSheet(Theme::boldButton());
    btnStartRecon = new QPushButton("2. 开始三维重建");
    btnStartRecon->setStyleSheet(Theme::successButton());
    btnSaveRecon = new QPushButton("3. 保存结果");
    btnSaveRecon->setEnabled(false);
    grpReconCtrlLayout->addWidget(btnLoadReconSeq);
    grpReconCtrlLayout->addWidget(btnStartRecon);
    grpReconCtrlLayout->addWidget(btnSaveRecon);

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

    // 红光掩膜 (HSV) 的另外两道门限。原先是写死的 [0,10]∪[170,180] 与 S>60,
    // 结果亮起来偏白的光条 (色相漂到 ~160°) 被判成非红光而整段丢失。
    spinLaserHueTol = new QSpinBox();
    spinLaserHueTol->setRange(0, 90);
    spinLaserHueTol->setValue(30);
    spinLaserHueTol->setSingleStep(5);
    spinLaserHueTol->setSuffix(" °");
    spinLaserHueTol->setToolTip(
        "红色色相容差 (OpenCV 色相 0-180 标度)。窗口 = [0,tol] ∪ [180-tol,180]。\n"
        "光条越亮越白, 色相会从暗处的 ~174° 漂到亮处的 ~155°;\n"
        "容差 10° 只覆盖 170~180°, 光条中下段会被整段判掉 (实测丢失一半以上提取点)。\n"
        "调到 30° 覆盖 150~180°, 多出来的点能通过 S3 的几何校验。");

    spinLaserMinSat = new QSpinBox();
    spinLaserMinSat->setRange(0, 255);
    spinLaserMinSat->setValue(40);
    spinLaserMinSat->setSingleStep(5);
    spinLaserMinSat->setToolTip(
        "红色掩膜的最低饱和度 (0-255)。原为 60;\n"
        "光条打在高反光/蓝色表面上时饱和度会掉到 50~70, 门限过高同样会整段丢光条。");

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

    spinSatRThresh = new QSpinBox();
    spinSatRThresh->setRange(200, 255);
    spinSatRThresh->setValue(250);
    spinSatRThresh->setToolTip("红色通道削顶判定。激光是纯红的，R 早就到 255 了而 L 只有 130~180，"
                               "所以过曝必须按 R 判 —— 否则平台区的中心会被绿蓝通道的噪声带走，逐行乱跳");

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

    // 【P0-3】最低点置信度过滤
    spinMinConfidence = new QDoubleSpinBox();
    spinMinConfidence->setRange(0.0, 1.0);
    spinMinConfidence->setValue(0.0);
    spinMinConfidence->setSingleStep(0.05);
    spinMinConfidence->setDecimals(2);
    spinMinConfidence->setToolTip("低于此置信度的光条点直接丢弃 (0=不过滤)。\n"
                                  "置信度 = 亮度归一化(L/255) × 提取方式系数(过曝恢复点×0.75)。\n"
                                  "过滤发生在极线匹配之前，可减少弱响应点引起的错配。");

    // 添加到表单布局
    formS1->addRow("", btnSelectRoiLeft);
    formS1->addRow("", btnSelectRoiRight);
    formS1->addRow("S1 算法:", cmbS1Method);
    formS1->addRow(chkUseSteger);
    formS1->addRow("红色阈值(LAB A):", spinLabA);
    formS1->addRow("红色色相容差:", spinLaserHueTol);
    formS1->addRow("红色饱和度下限:", spinLaserMinSat);
    formS1->addRow("高斯平滑系数:", spinStegerSigma);
    formS1->addRow("泰勒偏移阈值:", spinStegerTMax);
    formS1->addRow(chkOverexposedEnable);
    formS1->addRow("过曝亮度阈值(L):", spinOverexposedL);
    formS1->addRow("红通道饱和阈值:", spinSatRThresh);
    formS1->addRow("边缘偏移系数:", spinEdgeOffsetSigma);
    formS1->addRow("最低点置信度:", spinMinConfidence);

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

    // S3 光平面一致性阈值, 单位毫米。判据是"双目 DLT 点到光平面的距离"。
    // 单位取 mm 而不是 px, 是因为同一份平面误差折算到不同深度上是不同的 px 数 ——
    // 固定 px 阈值在 113~255mm 的工作范围内松紧差 2.3 倍, 距离一改就得重调。
    spinReprojReject = new QDoubleSpinBox();
    spinReprojReject->setRange(0.2, 100.0);
    spinReprojReject->setValue(8.0);
    spinReprojReject->setSingleStep(0.5);
    spinReprojReject->setDecimals(2);
    spinReprojReject->setSuffix(" mm");
    spinReprojReject->setToolTip(
        "S3 剔除 S2 错配点的阈值, 单位毫米。\n"
        "3D 点由双目 DLT 给出; 判据 = 该点到光平面的距离。\n\n"
        "实测匹配分得很干净: 自洽的 ≤5mm, 伪匹配 ≥34mm, 中间是空的 ——\n"
        "所以 8mm 落在空档里, 再往上放宽只会放进伪匹配。\n"
        "标定良好的系统残差中位应在 1mm 上下; 若日志里「一致性残差分布」显示大量点\n"
        "紧贴在阈值上方, 说明光平面略有偏差, 可适当放宽; 若成片落在几十 px 以上,\n"
        "多半是光平面标定本身有问题, 放宽只会污染点云。");

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
    formS2->addRow("光平面一致性阈值:", spinReprojReject);
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

    // 回环校正: 转台转满一圈时末帧应与首帧重合, 二者偏差即整圈累计漂移
    chkLoopClosure = new QCheckBox("启用回环校正");
    chkLoopClosure->setChecked(false);   // 默认关闭: 不在未被告知的情况下改动点云位姿
    chkLoopClosure->setToolTip(
        "关闭(默认): 各视角位姿纯增量累加, 不做任何回环修正 —— 点云里看到的就是\n"
        "         标定参数与步长的原始结果, 排查问题时先看这一档。\n"
        "启用: 用整圈累计漂移反推误差, 再沿标定轴摊回各帧。步长偏大/偏小造成的\n"
        "     方位向拉伸会被它拉回整圈 360°, 但也会把真实误差一并抹掉。");

    comboLoopClosure = new QComboBox();
    comboLoopClosure->addItem("均摊(增量)");   // index 0 == LoopClosureSpread
    comboLoopClosure->addItem("ICP回环");      // index 1 == LoopClosureIcp
    comboLoopClosure->setCurrentIndex(0);
    comboLoopClosure->setEnabled(false);       // 未启用时无意义, 随总开关联动
    comboLoopClosure->setToolTip(
        "均摊(增量): 用增量累计偏离恒等的量沿标定轴摊回各帧。不依赖点云质量。\n"
        "ICP回环: 用末帧↔首帧的 ICP 残差作误差再均摊。数据好时更准,\n"
        "         但 360° 完全重合时 ICP 可能收敛到 0° 而失效。");
    // 总开关驱动方式下拉框的可用性: 关闭时它一个值都不会被读到, 灰掉才不误导
    connect(chkLoopClosure, &QCheckBox::toggled, comboLoopClosure, &QComboBox::setEnabled);
    formS5->addRow("回环校正:", chkLoopClosure);
    formS5->addRow("回环校正方式:", comboLoopClosure);
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
    spinVoxelSize->setToolTip("S6 去噪体素降采样尺寸(mm)。0=不降采样；\n"
                              "网格化前的密度均匀化在 0 时按点云自身中位点间距自适应。");

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
    spinPoissonDepth->setValue(9);  // 与算法层 ReconstructionParams 默认值(9=512³)保持一致
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
    formS6->addRow("体素大小(0=自动):", spinVoxelSize);
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
    addPage(tabWidget, tab5, "05 三维重建", "05", "三维重建",
            "S1 光条提取 → S2 极线匹配 → S3 三角化 → S4 坐标系变换 → S5 多视角配准 → S6 去噪网格化");

    // Tab 5 信号连接
    connect(btnLoadReconSeq, &QPushButton::clicked, this, &MainWindow::onLoadReconSeqClicked);
    connect(btnStartRecon, &QPushButton::clicked, this, &MainWindow::onStartReconstructionClicked);
    connect(btnSaveRecon, &QPushButton::clicked, this, &MainWindow::onSaveReconResultClicked);
    connect(btnSelectRoiLeft, &QPushButton::clicked, this, &MainWindow::onSelectRoiLeftClicked);
    connect(btnSelectRoiRight, &QPushButton::clicked, this, &MainWindow::onSelectRoiRightClicked);
    // 诊断类按钮(单帧预览 / 点云窗口 / 独立3D视图)的连接在 initTab6() 中建立
    connect(tabWidget, &QTabWidget::currentChanged, this, [this](int index) {
        if (index == 4) { m_viewer3D->setFocus(); }   // Tab5 = 三维重建
    });
}

// ================= Tab 5: 三维重建 槽函数 =================

void MainWindow::applyCalibAndParamsToBuilder()
{
    m_builder->setCalibrationData(buildCalibrationData());
    m_builder->setReconstructionParams(buildReconstructionParams());
}

CalibrationData MainWindow::buildCalibrationData() const
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
    return calibData;
}

ReconstructionParams MainWindow::buildReconstructionParams() const
{
    ReconstructionParams params;
    params.s1_method = cmbS1Method->currentIndex();
    params.lab_a_threshold = spinLabA->value();
    params.laser_hue_tol_deg = spinLaserHueTol->value();
    params.laser_min_sat = spinLaserMinSat->value();
    params.use_steger = chkUseSteger->isChecked();
    params.steger_sigma = spinStegerSigma->value();
    params.steger_t_max = spinStegerTMax->value();
    params.steger_edge_offset_enable = chkOverexposedEnable->isChecked();
    params.steger_overexposed_l_thresh = spinOverexposedL->value();
    params.sat_r_thresh = spinSatRThresh->value();
    params.steger_edge_offset_sigma = spinEdgeOffsetSigma->value();
    params.min_point_confidence = static_cast<float>(spinMinConfidence->value());   // 【P0-3】
    params.min_segment_length = spinMorphSigma->value();
    params.sample_step = spinSampleStep->value();
    params.epipolar_threshold = spinEpipolarThresh->value();
    params.depth_min = spinDepthMin->value();
    params.depth_max = spinDepthMax->value();
    params.reproj_reject_mm = spinReprojReject->value();
    params.disparity_break_threshold = spinDisparityBreak->value();
    params.dp_skip_penalty = spinDpSkipPenalty->value();
    params.dp_smooth_weight = spinDpSmoothWeight->value();
    params.use_icp = chkUseIcp->isChecked();
    params.icp_max_correspondence_distance = spinIcpMaxDist->value();
    params.icp_max_iterations = spinIcpIter->value();
    params.icp_axis_trust = spinIcpAxisTrust->value();
    params.icp_euclidean_fitness_epsilon = spinIcpEpsilon->value();
    params.icp_translation_epsilon = spinIcpTransEps->value();
    // 下拉框顺序与 ReconstructionParams::LoopClosure 前两项一一对应 (均摊=0 / ICP=1);
    // 第三项 LoopClosureOff 不在下拉框里 —— "关不关"由上面的总开关决定, 只留一个入口。
    params.loop_closure = chkLoopClosure->isChecked()
                        ? comboLoopClosure->currentIndex()
                        : int(ReconstructionParams::LoopClosureOff);
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
    params.parallel_threads = spinParallelThreads->value();   // 【P0-4】0=自动
    return params;
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

// ============================================================================
// 重建作业组装 / 运行态 —— **Tab5 与 Tab6「完整重建测试」共用**
//   抽出来的理由: 两条入口若各写一遍组装逻辑, 迟早会因为漏改一处而出现
//   "测试通过、正式跑失败"这种最难查的分歧; 运行态 (m_reconRunning + 按钮文案)
//   也必须共用, 否则 Tab6 触发时 Tab5 的按钮还亮着, 用户能对忙碌的 worker 再排一个 job。
// ============================================================================
bool MainWindow::buildReconJob(ReconstructionWorker::JobInput& job, QString* err)
{
    if (m_reconLeftPaths.isEmpty() || m_reconRightPaths.isEmpty()) {
        if (err) *err = "请先到「三维重建」页加载旋转图像序列（左右各一个文件夹）";
        return false;
    }
    if (m_reconLeftPaths.size() != m_reconRightPaths.size()) {
        // worker 内部按 qMin 取长度会静默截断, 这里显式拦下 —— 否则"少跑了几帧"没人会发现
        if (err) *err = QString("左右序列数量不一致（%1 / %2），请重新加载")
                            .arg(m_reconLeftPaths.size()).arg(m_reconRightPaths.size());
        return false;
    }
    const double angleStep = spinAngleStep ? spinAngleStep->value() : 1.8;
    if (!(angleStep > 0.0)) {
        if (err) *err = "「旋转步长」必须大于 0";
        return false;
    }

    const bool axisDirValid   = !m_rotAxisDirection.empty() && (cv::norm(m_rotAxisDirection) > 0.1);
    const bool axisPointValid = !m_rotAxisPoint.empty()     && (cv::norm(m_rotAxisPoint) > 0.01);
    const bool axisValid      = axisDirValid && axisPointValid;   // 完整标定 = 方向 + 轴点

    // 轴标定缺失时的兜底值**不在这里给** —— 这两个字段在 S5 里是按**转台基准系**解释的,
    // 而"相机系里转台在光轴前方 250mm、竖直方向是 +Y"那套默认值属于**相机系**, 混用会
    // 拼出一团散点(见 reconstructionworker.cpp 里安全降级那段)。真正的兜底在 worker 里
    // 就地估: 方向取 +Z, 轴点取装配点云的 XY 形心。这里只放中性占位。
    Eigen::Vector3f axis_point(0.0f, 0.0f, 0.0f);   // 占位, axisValid=false 时由 worker 覆盖
    Eigen::Vector3f axis_dir(0.0f, 0.0f, 1.0f);     // 占位, 同上
    if (axisValid) {
        axis_point = Eigen::Vector3f(
            static_cast<float>(m_rotAxisPoint.at<double>(0,0)),
            static_cast<float>(m_rotAxisPoint.at<double>(1,0)),
            static_cast<float>(m_rotAxisPoint.at<double>(2,0)));
        axis_dir = Eigen::Vector3f(
            static_cast<float>(m_rotAxisDirection.at<double>(0,0)),
            static_cast<float>(m_rotAxisDirection.at<double>(1,0)),
            static_cast<float>(m_rotAxisDirection.at<double>(2,0)));
    }

    job = ReconstructionWorker::JobInput();
    job.leftPaths    = m_reconLeftPaths;
    job.rightPaths   = m_reconRightPaths;
    job.angleStepDeg = angleStep;
    job.rectified    = m_IsRectified && !m_MapL1.empty();
    job.mapL1 = m_MapL1;
    job.mapL2 = m_MapL2;
    job.mapR1 = m_MapR1;
    job.mapR2 = m_MapR2;
    job.calibData = buildCalibrationData();
    job.params    = buildReconstructionParams();
    job.axisPoint = axis_point;
    job.axisDir   = axis_dir;
    job.axisValid = axisValid;
    return true;
}

void MainWindow::markReconRunning()
{
    m_reconRunning = true;
    if (!btnStartRecon) return;
    btnStartRecon->setText("取消重建");
    btnStartRecon->setStyleSheet(Theme::dangerButton());
    // 必须重新启用: 提交前的准备阶段按钮是禁用的(避免参数总览还没打完就被再点一次),
    // 而"重建进行中再点一次 = 请求取消"这个分支要靠它可点才起作用。
    // 旧版直到这里都没恢复启用, 于是取消路径整个是死代码 —— 按钮从头到尾按不动。
    btnStartRecon->setEnabled(true);
}

void MainWindow::onStartReconstructionClicked()
{
    if (m_reconLeftPaths.isEmpty()) {
        QMessageBox::warning(this, "错误", "请先导入旋转图像序列！");
        return;
    }

    // 重建进行中：再次点击 = 请求取消
    if (m_reconRunning) {
        if (m_reconWorker) {
            m_reconWorker->cancel();
            btnStartRecon->setText("正在取消...");
            btnStartRecon->setEnabled(false);
            txtReconLog->append(">>> 已请求取消重建，等待当前视角处理结束...");
        }
        return;
    }

    txtReconLog->clear();
    m_viewer3D->clearViewer();
    btnStartRecon->setEnabled(false);
    btnSaveRecon->setEnabled(false);
    progressRecon->setValue(0);

    txtReconLog->append("====== [前置诊断] 开始三维重建检查 ======");
    double angleStep = spinAngleStep->value();
    bool axisDirValid = !m_rotAxisDirection.empty() && (cv::norm(m_rotAxisDirection) > 0.1);
    bool axisPointValid = !m_rotAxisPoint.empty() && (cv::norm(m_rotAxisPoint) > 0.01);
    bool axisValid = axisDirValid && axisPointValid;  // 完整标定 = 方向 + 轴点

    txtReconLog->append(QString("1. 图像序列: %1 对 | 步长: %2°").arg(m_reconLeftPaths.size()).arg(angleStep));
    txtReconLog->append(QString("2. 相机参数: %1 | 立体外参: %2").arg(m_CameraMatrixL.empty() ? "❌" : "✅").arg(m_R.empty() ? "❌" : "✅"));
    txtReconLog->append(QString("3. 旋转轴标定: %1").arg(axisValid ? "✅ 完整" : (axisDirValid ? "⚠️ 仅方向" : "❌ 未标定")));

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
        .arg(cmbS1Method->currentText())
        .arg(spinStegerSigma->value()).arg(spinStegerTMax->value()).arg(spinLabA->value()));
    txtReconLog->append(QString("红光掩膜: 色相容差 %1° (窗 %2~180 ∪ 0~%3) | 饱和度下限 %4")
        .arg(spinLaserHueTol->value())
        .arg(180 - spinLaserHueTol->value()).arg(spinLaserHueTol->value())
        .arg(spinLaserMinSat->value()));
    txtReconLog->append(QString("过曝恢复: %1 | L阈值: %2 | 红通道饱和阈值: %3 | 偏移系数: %4")
        .arg(chkOverexposedEnable->isChecked() ? "开" : "关")
        .arg(spinOverexposedL->value()).arg(spinSatRThresh->value())
        .arg(spinEdgeOffsetSigma->value()));
    txtReconLog->append(QString("最低点置信度: %1 %2 | 并行线程: %3")
        .arg(spinMinConfidence->value(), 0, 'f', 2)
        .arg(spinMinConfidence->value() <= 0.0 ? "(不过滤)" : "(先过滤再匹配)")
        .arg(spinParallelThreads->value() == 0 ? "自动" : QString::number(spinParallelThreads->value())));

    // -- S2 匹配参数 --
    txtReconLog->append(QString("\n--- S2 极线匹配 ---"));
    txtReconLog->append(QString("极线容差: %1 px | 视差跳变: %2 px | DP跳过: %3 | DP平滑: %4")
        .arg(spinEpipolarThresh->value()).arg(spinDisparityBreak->value())
        .arg(spinDpSkipPenalty->value()).arg(spinDpSmoothWeight->value()));
    txtReconLog->append(QString("深度范围: %1 ~ %2 mm")
        .arg(spinDepthMin->value()).arg(spinDepthMax->value()));
    txtReconLog->append(QString("S3 光平面一致性阈值: %1 mm (DLT点到光平面的距离)").arg(spinReprojReject->value(), 0, 'f', 2));

    // -- S5 参数 --
    if (chkUseIcp->isChecked()) {
        txtReconLog->append(QString("\n--- S5 ICP 配准 ---"));
        txtReconLog->append(QString("最大对应距离: %1 mm | 最大迭代: %2 | 收敛阈值: %3")
            .arg(spinIcpMaxDist->value()).arg(spinIcpIter->value())
            .arg(spinIcpEpsilon->value()));
    } else {
        // 关闭时不再罗列 ICP 参数: 它们这一趟一个都不会被读到, 列出来只会让人以为生效了。
        txtReconLog->append(QString("\n--- S5 纯轴旋转拼接 (ICP 已关闭) ---"));
        txtReconLog->append(QString("各视角按「序号 × %1°」绕标定轴直接拼接 | 步长误差会原样累积进点云")
            .arg(spinAngleStep->value(), 0, 'f', 4));
    }
    // 回环校正: 关与不关会给出**位姿不同**的点云, 是排查步长/轴误差时最该先确认的一档
    if (chkLoopClosure->isChecked()) {
        txtReconLog->append(QString("回环校正: 启用 (%1) | 整圈累计漂移将沿标定轴摊回各帧")
            .arg(comboLoopClosure->currentText()));
    } else {
        txtReconLog->append(QString("回环校正: 已关闭 | 位姿纯增量累加, 未做整圈修正"));
    }

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

    // ====================================================================
    // 组装作业并投递到后台工作线程 (S1~S6 在独立线程执行，UI 不再卡顿)
    //   组装走 buildReconJob(), 与 Tab6「完整重建测试」共用同一份代码
    // ====================================================================
    ReconstructionWorker::JobInput job;
    QString jobErr;
    if (!buildReconJob(job, &jobErr)) {
        txtReconLog->append("❌ " + jobErr);
        // 上面已经把按钮禁用了, 这条提前返回必须恢复 —— 否则一次参数错误就把入口永久卡灰
        btnStartRecon->setEnabled(true);
        QMessageBox::warning(this, "无法开始重建", jobErr);
        progressRecon->setValue(0);
        return;
    }

    markReconRunning();
    // 留一份作业快照: 「导出重建调试包」要用同一套标定/参数重跑样本帧,
    // 不能读当时的界面控件值 (用户可能在跑完之后动过滑块)
    m_lastReconJob = job;
    m_hasReconJob = true;
    txtReconLog->append("====== 已提交重建任务到后台线程，UI 保持响应 (再次点击按钮可取消) ======");

    QMetaObject::invokeMethod(m_reconWorker, "run", Qt::QueuedConnection,
                              Q_ARG(ReconstructionWorker::JobInput, job));
}

// ================= Tab 5: 重建工作线程信号槽 =================

void MainWindow::onReconstructionFinished()
{
    // 本次是否由 Tab6「完整重建测试」发起: 取一次就清掉, 后续按普通重建收尾
    const bool smokeTest = m_reconSmoke;
    m_reconSmoke = false;

    m_reconRunning = false;
    btnStartRecon->setEnabled(true);
    btnStartRecon->setText("2. 开始三维重建");
    btnStartRecon->setStyleSheet(Theme::successButton());

    if (!m_reconWorker) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud = m_reconWorker->resultCloud();
    pcl::PolygonMesh finalMesh = m_reconWorker->resultMesh();

    // 结果写回 m_builder，供保存/独立视图复用
    m_builder->storeResult(finalCloud, finalMesh);

    // 同步刷新「调试与诊断」页的前置条件与重建规模指标条
    updateDebugStatus();
    appendDebugLog(QString("[重建] 流水线执行完毕: 点云 %1 点 — 可到「调试与诊断」页回看结果")
        .arg(finalCloud ? static_cast<int>(finalCloud->size()) : 0));

    if (finalCloud && !finalCloud->empty()) {
        m_viewer3D->showPointCloud(finalCloud, "final_cloud");
        m_viewer3D->showMesh(finalMesh, "final_mesh");
        m_viewer3D->resetCamera();
        btnSaveRecon->setEnabled(true);
    } else {
        txtReconLog->append("❌ 警告: 最终点云为空！");
    }

    txtReconLog->append("三维重建流程执行完毕！");

    // 测试模式: 只出通过/不通过报告, 不再弹一个 3D 结果窗口 (测试要的是结论, 不是又一张图)。
    // 结果已在上方 storeResult 写回 m_builder, 仍可在 Tab6「点云+网格」里回看。
    if (smokeTest) {
        appendDebugLog(buildReconTestReport());
        return;
    }

    // 自动弹出重建结果窗口 (点云 + 网格)
    if (finalCloud && !finalCloud->empty()) {
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

void MainWindow::onReconstructionError(const QString &message)
{
    txtReconLog->append("❌ 错误: " + message);
}

void MainWindow::onReconProgress(int current, int total)
{
    if (total > 0) {
        progressRecon->setValue(static_cast<int>(current * 100.0 / total));
    }
}

void MainWindow::onReconLog(const QString &line)
{
    txtReconLog->append(line);
}

void MainWindow::onReconViewImage(int index, const QImage &left, const QImage &right)
{
    Q_UNUSED(index);
    if (!left.isNull())  lblReconLeftView->setPixmap(QPixmap::fromImage(left));
    if (!right.isNull()) lblReconRightView->setPixmap(QPixmap::fromImage(right));
    // 画完了才放开下一帧 (背压闸门), 见 ReconstructionWorker::framePreviewConsumed()
    if (m_reconWorker) m_reconWorker->framePreviewConsumed();
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

