// ==================== Tab6: 调试与诊断 ====================
//
// 设计说明 (UI 优化):
//   功能分区沿用"两栏诊断"结构, 本次只做界面/交互层的打磨:
//     · ① 前置条件检查 (通栏): 左右两块与 ②③ 一一对应, 每项显示 ✓/✗ + 当前取值,
//        并带「去标定 / 去加载 / 去重建」跳转按钮 —— 缺什么、去哪一页补, 一步可达。
//     · ② 标定诊断 (相机·光平面·转台) / ③ 重建诊断 (S1→S6 与结果):
//        每栏 = 说明框 + 指标条 + 若干小节; 每栏只保留 1 个实心主操作按钮,
//        其余次要/回看入口统一为描边(ghost)按钮, 避免"全是重点 = 没有重点"。
//     · 下部统一调试输出: 通栏 + 清空/复制/保存/自动滚动 工具条, 可拖动放大。
//
//   各调试入口内部仍保留前置条件校验: 缺数据时给出"去哪一页做什么"的具体指引，
//   而不是简单地灰掉按钮。
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include "ui/curveplotwidget.h"
#include "core/pointcloudbuilder.h"
#include "core/pointcloudviewer.h"
#include <opencv2/imgcodecs.hpp>

#include <QWidget>
#include <QSplitter>
#include <QGroupBox>
#include <QFrame>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QCheckBox>
#include <QSpinBox>
#include <QTextEdit>
#include <QTextCursor>
#include <QPointer>   // s_debugDlg: 窗口被销毁后自动置空, 避免悬垂指针
#include <QTimer>     // 上一帧/下一帧: 把重建推迟到当前槽返回之后
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QBrush>
#include <QColor>
#include <QTabWidget>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTextStream>
#include <QFont>
#include <QMessageBox>
#include <QApplication>
#include <QEventLoop>
#include <QDialog>
#include <QDateTime>
#include <QStringList>
#include <QList>
#include <QPixmap>
#include <QDebug>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cfloat>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>

namespace {

// ① 前置条件检查项: 显示名 / 缺失时跳转的页索引 (-1 = 在本页生成, 不跳转) / 跳转按钮文字
struct DebugCheckRow {
    const char* name;
    int         tab;
    const char* action;
};

const DebugCheckRow kCalibChecks[3] = {
    { "双目标定参数",  1, "去标定" },
    { "旋转序列",      3, "去加载" },
    { "旋转轴标定",    3, "去标定" }
};

const DebugCheckRow kReconChecks[3] = {
    { "重建序列",       4, "去加载"   },
    { "重建结果点云",   4, "去重建"   },
    { "多帧棋盘格统计", -1, "本页生成" }
};

} // namespace

void MainWindow::initTab6(QTabWidget* tabWidget) {
    QWidget *page = new QWidget();
    QVBoxLayout *mainLayout = new QVBoxLayout(page);
    mainLayout->setContentsMargins(Theme::MARGIN, Theme::MARGIN, Theme::MARGIN, Theme::MARGIN);
    mainLayout->setSpacing(Theme::SPACING);

    // 上下可拖动: 上 = 入口区 (① 前置条件 + ②③ 两栏诊断), 下 = 统一调试输出
    QSplitter *vSplit = new QSplitter(Qt::Vertical);
    vSplit->setHandleWidth(4);
    vSplit->setChildrenCollapsible(false);
    vSplit->setStyleSheet(QString("QSplitter::handle { background-color: %1; }").arg(Theme::BORDER));
    mainLayout->addWidget(vSplit);

    QWidget *topPanel = new QWidget();
    QVBoxLayout *topLay = new QVBoxLayout(topPanel);
    topLay->setContentsMargins(0, 0, 0, 0);
    topLay->setSpacing(Theme::SPACING);

    // ========================================================================
    // ① 前置条件检查 (通栏: 左"标定"块 / 右"重建"块, 与 ②③ 两栏一一对应)
    //    每行 = ✓/✗ + 名称 + 当前取值 + 跳转按钮("缺什么就去哪一页补")
    // ========================================================================
    QGroupBox *grpReady = new QGroupBox("① 前置条件检查");
    grpReady->setStyleSheet(Theme::cardGroupStyle());
    QHBoxLayout *readyLay = new QHBoxLayout(grpReady);
    readyLay->setContentsMargins(8, 4, 8, 8);
    readyLay->setSpacing(Theme::SPACING);

    QGridLayout *readyGrid = new QGridLayout();
    readyGrid->setContentsMargins(0, 0, 0, 0);
    readyGrid->setHorizontalSpacing(8);
    readyGrid->setVerticalSpacing(4);
    readyGrid->setColumnStretch(1, 1);
    readyGrid->setColumnStretch(5, 1);

    // 生成一块检查表: 表头 + 3 行 (名称 / 当前取值 / 跳转按钮)
    auto buildReadyBlock = [&](const DebugCheckRow* rows, int colBase,
                               QVector<QLabel*>& titles, QVector<QLabel*>& values,
                               const QString& head) {
        QLabel *h = new QLabel(head);
        h->setStyleSheet(Theme::subTitleStyle());
        readyGrid->addWidget(h, 0, colBase, 1, 3);

        for (int r = 0; r < 3; ++r) {
            QLabel *t = new QLabel();
            t->setTextFormat(Qt::RichText);
            t->setStyleSheet(QString("font-size: 12px; color: %1;").arg(Theme::TEXT_PRIMARY));

            QLabel *v = new QLabel("—");
            v->setTextFormat(Qt::RichText);
            v->setStyleSheet(QString("font-size: 12px; color: %1;").arg(Theme::TEXT_SECONDARY));

            readyGrid->addWidget(t, r + 1, colBase);
            readyGrid->addWidget(v, r + 1, colBase + 1);

            if (rows[r].tab >= 0) {
                QPushButton *b = new QPushButton(rows[r].action);
                b->setStyleSheet(Theme::miniGhostButton());
                b->setCursor(Qt::PointingHandCursor);
                b->setToolTip(QString("切换到「%1」页处理").arg(tabWidget->tabText(rows[r].tab)));
                const int target = rows[r].tab;
                connect(b, &QPushButton::clicked, this, [this, target]() {
                    if (this->tabWidget) this->tabWidget->setCurrentIndex(target);
                });
                readyGrid->addWidget(b, r + 1, colBase + 2);
            } else {
                QLabel *b = new QLabel(rows[r].action);
                b->setStyleSheet(QString("font-size: 11px; color: %1;").arg(Theme::TEXT_DIM));
                readyGrid->addWidget(b, r + 1, colBase + 2);
            }

            titles.push_back(t);
            values.push_back(v);
        }
    };

    buildReadyBlock(kCalibChecks, 0, m_dbgCalibTitle, m_dbgCalibValue, "标定  (供 ② 使用)");

    // 竖分隔线: 固定 1px, 同时给出 Sunken 边框与背景色, 保证任何风格下都可见
    QFrame *vline = new QFrame();
    vline->setFrameShape(QFrame::VLine);
    vline->setFrameShadow(QFrame::Sunken);
    vline->setFixedWidth(1);
    vline->setStyleSheet(QString("background-color: %1;").arg(Theme::BORDER));
    readyGrid->addWidget(vline, 0, 3, 4, 1);

    buildReadyBlock(kReconChecks, 4, m_dbgReconTitle, m_dbgReconValue, "重建  (供 ③ 使用)");

    readyLay->addLayout(readyGrid, 1);

    btnRefreshDebugStatus = new QPushButton("刷新");
    btnRefreshDebugStatus->setStyleSheet(Theme::ghostButton());
    btnRefreshDebugStatus->setCursor(Qt::PointingHandCursor);
    btnRefreshDebugStatus->setToolTip("重新检查前置条件 (切换到本页时也会自动刷新)");
    readyLay->addWidget(btnRefreshDebugStatus, 0, Qt::AlignTop);

    topLay->addWidget(grpReady);

    // ========================================================================
    // ②③ 两栏: 标定诊断 | 重建诊断
    // ========================================================================
    QHBoxLayout *cols = new QHBoxLayout();
    cols->setSpacing(Theme::SPACING);

    // ---------------- ② 标定诊断 (相机 / 光平面 / 转台) ----------------
    QGroupBox *grpCalib = new QGroupBox("② 标定诊断  (相机·光平面·转台)");
    grpCalib->setStyleSheet(Theme::cardGroupStyle());
    QVBoxLayout *calibLay = new QVBoxLayout(grpCalib);
    calibLay->setContentsMargins(8, 4, 8, 8);
    calibLay->setSpacing(6);

    lblCalibMetrics = new QLabel("—");
    lblCalibMetrics->setTextFormat(Qt::RichText);
    lblCalibMetrics->setWordWrap(true);
    lblCalibMetrics->setStyleSheet(Theme::metricsStyle());
    calibLay->addWidget(lblCalibMetrics);

    QLabel *lblSubAxis = new QLabel("转台旋转轴 — 标定诊断");
    lblSubAxis->setStyleSheet(Theme::subTitleStyle());
    calibLay->addWidget(lblSubAxis);

    // 本栏主操作 (实心): 转台标定诊断
    btnAxisDiagRun = new QPushButton("运行转台标定诊断");
    btnAxisDiagRun->setStyleSheet(Theme::successButton());
    btnAxisDiagRun->setCursor(Qt::PointingHandCursor);
    btnAxisDiagRun->setEnabled(false);
    btnAxisDiagRun->setToolTip("需先在「转台标定」页加载旋转序列。\n输出: 逐帧指标表 + 一致性检查 + 可导出调试包。");
    calibLay->addWidget(btnAxisDiagRun);

    // 诊断进度条 (与 Tab4/Tab5 同一风格; 复用 process() 的进度回调)
    progressAxisDiag = new QProgressBar();
    progressAxisDiag->setRange(0, 100);
    progressAxisDiag->setValue(0);
    progressAxisDiag->setFormat("待机");
    progressAxisDiag->setToolTip("诊断进度: 内部会重跑一次完整标定 (提取角点 → 位姿消歧 → 圆拟合 → BA)");
    calibLay->addWidget(progressAxisDiag);

    QHBoxLayout *axisDiagRow = new QHBoxLayout();
    axisDiagRow->setSpacing(6);
    btnAxisDiagTable = new QPushButton("逐帧指标表");
    btnAxisDiagTable->setStyleSheet(Theme::miniGhostButton());
    btnAxisDiagTable->setCursor(Qt::PointingHandCursor);
    btnAxisDiagTable->setEnabled(false);
    btnAxisDiagTable->setToolTip("重新打开最近一次诊断的逐帧指标表");
    btnAxisDiagExport = new QPushButton("导出调试包");
    btnAxisDiagExport->setStyleSheet(Theme::miniGhostButton(Theme::PURPLE));
    btnAxisDiagExport->setCursor(Qt::PointingHandCursor);
    btnAxisDiagExport->setToolTip("导出核心调试内容: 诊断报告 + 逐帧指标CSV + 参数快照 + 日志 + 异常帧图像");
    axisDiagRow->addWidget(btnAxisDiagTable);
    axisDiagRow->addWidget(btnAxisDiagExport);
    axisDiagRow->addStretch();
    calibLay->addLayout(axisDiagRow);

    // 过程曲线: 光心是否真的共圆、BA 到底收敛到哪 —— 这两件事只有看图才知道,
    // 汇总指标(残差RMS/重投影px)都只是把分布压成了一个数。
    QHBoxLayout *curveRow = new QHBoxLayout();
    curveRow->setSpacing(6);
    btnCircleFitCurve = new QPushButton("圆拟合曲线");
    btnCircleFitCurve->setStyleSheet(Theme::miniGhostButton(Theme::PURPLE));
    btnCircleFitCurve->setCursor(Qt::PointingHandCursor);
    btnCircleFitCurve->setEnabled(false);
    btnCircleFitCurve->setToolTip("光心在拟合平面上的投影 + 拟合圆 + 逐点半径偏差曲线");
    btnBaIterCurve = new QPushButton("BA迭代曲线");
    btnBaIterCurve->setStyleSheet(Theme::miniGhostButton(Theme::PURPLE));
    btnBaIterCurve->setCursor(Qt::PointingHandCursor);
    btnBaIterCurve->setEnabled(false);
    btnBaIterCurve->setToolTip("BA 每轮迭代的重投影 RMS(px) 与误差阈值参考线");
    curveRow->addWidget(btnCircleFitCurve);
    curveRow->addWidget(btnBaIterCurve);
    curveRow->addStretch();
    calibLay->addLayout(curveRow);

    lblAxisDiagSummary = new QLabel("尚未运行诊断");
    lblAxisDiagSummary->setTextFormat(Qt::RichText);
    lblAxisDiagSummary->setWordWrap(true);
    lblAxisDiagSummary->setStyleSheet(Theme::metricsStyle());
    calibLay->addWidget(lblAxisDiagSummary);

    QLabel *lblSubChess = new QLabel("棋盘格全流程精度");
    lblSubChess->setStyleSheet(Theme::subTitleStyle());
    calibLay->addWidget(lblSubChess);

    QGridLayout *chessGrid = new QGridLayout();
    chessGrid->setContentsMargins(0, 0, 0, 0);
    chessGrid->setHorizontalSpacing(6);
    chessGrid->setVerticalSpacing(6);

    // 本栏主操作 (实心)
    btnVerify3D = new QPushButton("单帧 3D 验证");
    btnVerify3D->setStyleSheet(Theme::successButton());
    btnVerify3D->setCursor(Qt::PointingHandCursor);
    btnVerify3D->setEnabled(false);
    btnVerify3D->setToolTip("需先完成双目标定 + 旋转轴标定。\n检查极线Y误差、视差、fx 反推与 Z 展宽。");

    btnMultiFrame3D = new QPushButton("多帧 3D 重建");
    btnMultiFrame3D->setStyleSheet(Theme::ghostButton(Theme::PURPLE));
    btnMultiFrame3D->setCursor(Qt::PointingHandCursor);
    btnMultiFrame3D->setEnabled(false);
    btnMultiFrame3D->setToolTip("对所有旋转序列帧执行完整 S3→S4→S5 三维重建，验证全流水线精度。");

    chessGrid->addWidget(btnVerify3D, 0, 0);
    chessGrid->addWidget(btnMultiFrame3D, 0, 1);
    chessGrid->setColumnStretch(0, 1);
    chessGrid->setColumnStretch(1, 1);
    calibLay->addLayout(chessGrid);

    QHBoxLayout *reopenLayout = new QHBoxLayout();
    reopenLayout->setSpacing(6);
    QLabel *lblReplay = new QLabel("回看上次结果");
    lblReplay->setStyleSheet(QString("font-size: 11px; color: %1;").arg(Theme::TEXT_SECONDARY));
    reopenLayout->addWidget(lblReplay);

    btnMultiFrameStats = new QPushButton("统计文本");
    btnMultiFrameStats->setStyleSheet(Theme::miniGhostButton());
    btnMultiFrameStats->setCursor(Qt::PointingHandCursor);
    btnMultiFrameStats->setEnabled(false);
    btnMultiFrameStats->setToolTip("重新打开上一次多帧重建的文本统计结果");

    btnMultiFrame3DView = new QPushButton("点云窗口");
    btnMultiFrame3DView->setStyleSheet(Theme::miniGhostButton());
    btnMultiFrame3DView->setCursor(Qt::PointingHandCursor);
    btnMultiFrame3DView->setEnabled(false);
    btnMultiFrame3DView->setToolTip("重新打开上一次多帧重建的点云窗口");

    reopenLayout->addWidget(btnMultiFrameStats);
    reopenLayout->addWidget(btnMultiFrame3DView);
    reopenLayout->addStretch();
    calibLay->addLayout(reopenLayout);
    calibLay->addStretch();
    cols->addWidget(grpCalib, 1);

    // ---------------- ③ 重建诊断 (S1→S6 与结果) ----------------
    QGroupBox *grpRecon = new QGroupBox("③ 重建诊断  (S1→S6 流水线与结果)");
    grpRecon->setStyleSheet(Theme::cardGroupStyle());
    QVBoxLayout *reconLay = new QVBoxLayout(grpRecon);
    reconLay->setContentsMargins(8, 4, 8, 8);
    reconLay->setSpacing(6);

    lblReconMetrics = new QLabel("—");
    lblReconMetrics->setTextFormat(Qt::RichText);
    lblReconMetrics->setWordWrap(true);
    lblReconMetrics->setStyleSheet(Theme::metricsStyle());
    reconLay->addWidget(lblReconMetrics);

    QLabel *lblSubSingle = new QLabel("单帧流水线调试");
    lblSubSingle->setStyleSheet(Theme::subTitleStyle());
    reconLay->addWidget(lblSubSingle);

    QHBoxLayout *frameRow = new QHBoxLayout();
    frameRow->setSpacing(4);

    QLabel *lblFrame = new QLabel("调试帧号");
    lblFrame->setStyleSheet(QString("font-size: 12px; color: %1;").arg(Theme::TEXT_PRIMARY));
    frameRow->addWidget(lblFrame);

    spinPreviewIndex = new QSpinBox();
    spinPreviewIndex->setRange(0, 0);   // 实际范围在 updateDebugStatus() 里按序列长度设置
    spinPreviewIndex->setFixedWidth(70);
    spinPreviewIndex->setToolTip("要调试的序列帧序号 (0 起)");
    frameRow->addWidget(spinPreviewIndex);

    btnPreviewPrev = new QPushButton("◀");
    btnPreviewPrev->setStyleSheet(Theme::miniGhostButton());
    btnPreviewPrev->setFixedSize(28, 24);
    btnPreviewPrev->setCursor(Qt::PointingHandCursor);
    btnPreviewPrev->setToolTip("上一帧");
    btnPreviewPrev->setEnabled(false);
    frameRow->addWidget(btnPreviewPrev);

    btnPreviewNext = new QPushButton("▶");
    btnPreviewNext->setStyleSheet(Theme::miniGhostButton());
    btnPreviewNext->setFixedSize(28, 24);
    btnPreviewNext->setCursor(Qt::PointingHandCursor);
    btnPreviewNext->setToolTip("下一帧");
    btnPreviewNext->setEnabled(false);
    frameRow->addWidget(btnPreviewNext);

    lblDbgSeqInfo = new QLabel("序列未加载");
    lblDbgSeqInfo->setStyleSheet(QString("font-size: 11px; color: %1;").arg(Theme::TEXT_SECONDARY));
    frameRow->addWidget(lblDbgSeqInfo);
    frameRow->addStretch();
    reconLay->addLayout(frameRow);

    // 本栏主操作 (实心)
    btnPreviewSingleFrame = new QPushButton("开始单帧调试");
    btnPreviewSingleFrame->setStyleSheet(Theme::warningButton());
    btnPreviewSingleFrame->setCursor(Qt::PointingHandCursor);
    btnPreviewSingleFrame->setToolTip("使用「三维重建」页当前参数与序列，实时计算并可视化单帧 S1/S2/S3 中间结果");
    reconLay->addWidget(btnPreviewSingleFrame);

    QLabel *lblSubView = new QLabel("结果查看");
    lblSubView->setStyleSheet(Theme::subTitleStyle());
    reconLay->addWidget(lblSubView);

    QHBoxLayout *viewRow = new QHBoxLayout();
    viewRow->setSpacing(6);

    btnOpenViewer = new QPushButton("点云 + 网格");
    btnOpenViewer->setStyleSheet(Theme::ghostButton());
    btnOpenViewer->setCursor(Qt::PointingHandCursor);
    btnOpenViewer->setToolTip("把当前重建结果弹出到独立窗口（点云 + 网格），需先在「三维重建」页跑完");

    btnShowPointCloudWindow = new QPushButton("仅点云");
    btnShowPointCloudWindow->setStyleSheet(Theme::ghostButton(Theme::SUCCESS));
    btnShowPointCloudWindow->setCursor(Qt::PointingHandCursor);
    btnShowPointCloudWindow->setToolTip("弹出独立窗口仅显示点云（不含网格），便于观察噪声点");

    viewRow->addWidget(btnOpenViewer);
    viewRow->addWidget(btnShowPointCloudWindow);
    reconLay->addLayout(viewRow);

    QLabel *lblSubTest = new QLabel("重建测试");
    lblSubTest->setStyleSheet(Theme::subTitleStyle());
    reconLay->addWidget(lblSubTest);

    QGridLayout *testGrid = new QGridLayout();
    testGrid->setContentsMargins(0, 0, 0, 0);
    testGrid->setHorizontalSpacing(6);
    testGrid->setVerticalSpacing(6);

    btnQuickYield = new QPushButton("快速产出测试");
    btnQuickYield->setStyleSheet(Theme::ghostButton(Theme::ACCENT));
    btnQuickYield->setCursor(Qt::PointingHandCursor);
    btnQuickYield->setToolTip("取序列前 N 帧跑 S1~S4 (不做 S5/S6 配准与网格化)，\n"
                              "输出逐帧有效点数曲线 + 产出率直方图 + 右图重投影误差分布，秒级完成");

    btnFullRecon = new QPushButton("完整重建测试");
    btnFullRecon->setStyleSheet(Theme::ghostButton(Theme::PURPLE));
    btnFullRecon->setCursor(Qt::PointingHandCursor);
    btnFullRecon->setToolTip("调用「三维重建」页同一套后台流水线跑完整 S1~S6，\n结束后给出通过/不通过判定与报告");

    // 导出重建调试包: 与「转台标定调试包」对应的一套重建侧取证材料。
    // 需要上次重建留下的记录, 所以由 updateDebugStatus() 门控 (跑之前点了只会得到空包)。
    btnReconDebugExport = new QPushButton("导出重建调试包");
    btnReconDebugExport->setStyleSheet(Theme::ghostButton(Theme::PURPLE));
    btnReconDebugExport->setCursor(Qt::PointingHandCursor);
    btnReconDebugExport->setEnabled(false);
    btnReconDebugExport->setToolTip(
        "导出重建侧取证材料: 逐帧产出表 / 逐帧重投影直方图 / 参数快照 / 重建日志 / 最终点云,\n"
        "并对若干「产出率最低」的样本帧重跑 S1~S3, 导出**每一对匹配的几何量**\n"
        "(视差/深度/像素残差/空间残差/双目DLT到光平面的残差/拒绝原因) 与光条提取叠加图。\n"
        "需要先跑过一次重建。");

    testGrid->addWidget(btnQuickYield, 0, 0);
    testGrid->addWidget(btnFullRecon, 0, 1);
    testGrid->addWidget(btnReconDebugExport, 1, 0, 1, 2);
    testGrid->setColumnStretch(0, 1);
    testGrid->setColumnStretch(1, 1);
    reconLay->addLayout(testGrid);

    // 导出进度: 导出要重跑样本帧 + 写逐视角点云 + 编码叠加图, 整段几秒, 没进度条只能干等。
    // 走的是与「转台标定诊断」同一套机制 (同步执行 + 进度回调 + processEvents 泵重绘)。
    progressReconDebug = new QProgressBar();
    progressReconDebug->setRange(0, 100);
    progressReconDebug->setValue(0);
    progressReconDebug->setFormat("待机");
    progressReconDebug->setToolTip("导出进度: 逐帧表 → 逐视角点云 → 样本帧重跑 → 叠加图 → 报告");
    reconLay->addWidget(progressReconDebug);

    reconLay->addStretch();
    cols->addWidget(grpRecon, 1);

    topLay->addLayout(cols, 1);
    vSplit->addWidget(topPanel);

    // ========================================================================
    // 统一调试输出 (两栏的结果都汇总到这里, 便于对照) + 工具条
    // ========================================================================
    QWidget *outPanel = new QWidget();
    QVBoxLayout *outLay = new QVBoxLayout(outPanel);
    outLay->setContentsMargins(0, 0, 0, 0);
    outLay->setSpacing(4);

    QHBoxLayout *outBar = new QHBoxLayout();
    outBar->setSpacing(6);

    QLabel *lblOutTitle = new QLabel("调试输出");
    lblOutTitle->setStyleSheet(Theme::sectionTitleStyle());
    outBar->addWidget(lblOutTitle);
    outBar->addStretch();

    chkDbgAutoScroll = new QCheckBox("自动滚动");
    chkDbgAutoScroll->setChecked(true);
    chkDbgAutoScroll->setStyleSheet(QString("font-size: 11px; color: %1;").arg(Theme::TEXT_SECONDARY));
    chkDbgAutoScroll->setToolTip("追加日志后自动滚动到最新一行");
    outBar->addWidget(chkDbgAutoScroll);

    QPushButton *btnLogExport = new QPushButton("导出调试包");
    btnLogExport->setStyleSheet(Theme::miniGhostButton(Theme::PURPLE));
    btnLogExport->setCursor(Qt::PointingHandCursor);
    btnLogExport->setToolTip("导出核心调试内容: 转台标定诊断报告 + 逐帧指标CSV + 标定/重建参数快照\n"
                             "+ Tab6/Tab5/Tab4 全部日志 + 异常帧图像");
    outBar->addWidget(btnLogExport);

    QPushButton *btnLogCopy = new QPushButton("复制");
    btnLogCopy->setStyleSheet(Theme::miniGhostButton());
    btnLogCopy->setCursor(Qt::PointingHandCursor);
    btnLogCopy->setToolTip("复制全部调试输出到剪贴板");

    QPushButton *btnLogSave = new QPushButton("保存");
    btnLogSave->setStyleSheet(Theme::miniGhostButton());
    btnLogSave->setCursor(Qt::PointingHandCursor);
    btnLogSave->setToolTip("把调试输出另存为文本文件");

    QPushButton *btnLogClear = new QPushButton("清空");
    btnLogClear->setStyleSheet(Theme::miniGhostButton(Theme::DANGER));
    btnLogClear->setCursor(Qt::PointingHandCursor);
    btnLogClear->setToolTip("清空调试输出");

    outBar->addWidget(btnLogCopy);
    outBar->addWidget(btnLogSave);
    outBar->addWidget(btnLogClear);
    outLay->addLayout(outBar);

    txtDebugLog = new QTextEdit();
    txtDebugLog->setReadOnly(true);
    txtDebugLog->setFont(QFont(Theme::MONO, 9));
    txtDebugLog->setStyleSheet(Theme::terminalStyle());
    txtDebugLog->setPlaceholderText("调试/验证结果将输出到此处...");
    outLay->addWidget(txtDebugLog, 1);
    vSplit->addWidget(outPanel);

    vSplit->setStretchFactor(0, 0);
    vSplit->setStretchFactor(1, 1);
    // 顶部两栏各新增了控件 (② 曲线两钮 / ③ 重建测试两钮), 不把顶部分配调大
    // 会把下方的统一调试输出区挤没
    // 顶部多一行「导出重建调试包」按钮, 尺寸跟着抬一点, 否则下方调试输出区被挤没
    vSplit->setSizes(QList<int>() << 412 << 380);

    // 页头 + 内容包装 (工业软件风格: [06] 调试与诊断 —— 该页职责)
    QWidget *pageWrap = addPage(tabWidget, page, "06 调试诊断", "06", "调试与诊断",
                                "前置条件检查 / 转台标定诊断 / 重建诊断 / 统一调试输出与调试包导出");

    // ============ 信号连接 ============
    connect(btnRefreshDebugStatus, &QPushButton::clicked, this, &MainWindow::onRefreshDebugStatus);
    connect(btnPreviewSingleFrame, &QPushButton::clicked, this, &MainWindow::onPreviewSingleFrameClicked);
    connect(btnAxisDiagRun, &QPushButton::clicked, this, &MainWindow::onRunAxisDiagnosis);
    connect(btnAxisDiagTable, &QPushButton::clicked, this, &MainWindow::onShowAxisDiagTable);
    connect(btnAxisDiagExport, &QPushButton::clicked, this, &MainWindow::onExportDebugPackage);
    connect(btnCircleFitCurve, &QPushButton::clicked, this, &MainWindow::onShowCircleFitCurve);
    connect(btnBaIterCurve, &QPushButton::clicked, this, &MainWindow::onShowBaIterCurve);
    connect(btnQuickYield, &QPushButton::clicked, this, &MainWindow::onRunQuickYieldTest);
    connect(btnFullRecon, &QPushButton::clicked, this, &MainWindow::onRunFullReconTest);
    connect(btnReconDebugExport, &QPushButton::clicked, this, &MainWindow::onExportReconDebugPackage);
    connect(btnLogExport, &QPushButton::clicked, this, &MainWindow::onExportDebugPackage);
    connect(btnVerify3D, &QPushButton::clicked, this, &MainWindow::onVerifyChessboard3D);
    connect(btnMultiFrame3D, &QPushButton::clicked, this, &MainWindow::onMultiFrameChessboard3D);
    connect(btnMultiFrameStats, &QPushButton::clicked, this, &MainWindow::onReopenChessboardStats);
    connect(btnMultiFrame3DView, &QPushButton::clicked, this, &MainWindow::onReopenChessboard3DView);
    connect(btnOpenViewer, &QPushButton::clicked, this, &MainWindow::onOpenStandaloneViewer);
    connect(btnShowPointCloudWindow, &QPushButton::clicked, this, &MainWindow::onShowPointCloudWindowClicked);

    // ③ 帧号步进 (spinPreviewIndex 自带上下限, 越界自动夹住)
    connect(btnPreviewPrev, &QPushButton::clicked, this, [this]() {
        spinPreviewIndex->setValue(spinPreviewIndex->value() - 1);
    });
    connect(btnPreviewNext, &QPushButton::clicked, this, [this]() {
        spinPreviewIndex->setValue(spinPreviewIndex->value() + 1);
    });

    // 输出区工具条
    connect(btnLogClear, &QPushButton::clicked, this, [this]() {
        txtDebugLog->clear();
    });
    connect(btnLogCopy, &QPushButton::clicked, this, [this]() {
        if (txtDebugLog->toPlainText().isEmpty()) return;
        txtDebugLog->selectAll();
        txtDebugLog->copy();
        QTextCursor c = txtDebugLog->textCursor();
        c.movePosition(QTextCursor::End);
        txtDebugLog->setTextCursor(c);
        appendDebugLog(QString("[%1] 已复制全部调试输出到剪贴板")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
    });
    connect(btnLogSave, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getSaveFileName(
            this, "保存调试输出", "debug_log.txt", "文本文件 (*.txt);;所有文件 (*)");
        if (path.isEmpty()) return;
        QFile f(path);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "保存失败", QString("无法写入文件:\n%1").arg(path));
            return;
        }
        { QTextStream ts(&f); ts << txtDebugLog->toPlainText(); ts.flush(); }
        f.close();
        appendDebugLog(QString("[%1] 调试输出已保存至: %2")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss"), path));
    });

    // 切换到本页时自动刷新前置条件 (用页面包装指针判断, 不依赖写死的 Tab 索引)
    connect(tabWidget, &QTabWidget::currentChanged, this, [this, pageWrap](int index) {
        if (this->tabWidget && this->tabWidget->widget(index) == pageWrap) updateDebugStatus();
    });

    appendDebugLog("=== 调试与诊断页就绪 (左: 标定诊断 / 右: 重建诊断) ===");
    appendDebugLog("提示: 顶部「① 前置条件检查」可直接点「去标定 / 去加载 / 去重建」跳转到对应页面补齐数据。");
    updateDebugStatus();
}

// ==================== Tab6 辅助 ====================

void MainWindow::appendDebugLog(const QString &line)
{
    if (!txtDebugLog) { qDebug().noquote() << "[debug]" << line; return; }
    txtDebugLog->append(line);
    if (!chkDbgAutoScroll || chkDbgAutoScroll->isChecked()) {
        QTextCursor c = txtDebugLog->textCursor();
        c.movePosition(QTextCursor::End);
        txtDebugLog->setTextCursor(c);
        txtDebugLog->ensureCursorVisible();
    }
}

void MainWindow::updateDebugStatus()
{
    if (m_dbgCalibValue.isEmpty() || m_dbgReconValue.isEmpty()) return;   // Tab6 尚未创建

    auto mark = [](bool ok) -> QString {
        return ok ? QString("<span style='color:%1; font-weight:bold'>✓</span>").arg(Theme::SUCCESS)
                  : QString("<span style='color:%1; font-weight:bold'>✗</span>").arg(Theme::DANGER);
    };
    auto tone = [](const QString& text, bool ok) -> QString {
        return QString("<span style='color:%1'>%2</span>")
                   .arg(ok ? Theme::SUCCESS : Theme::DANGER).arg(text);
    };
    // 把一块检查表的 3 行文字按状态刷进去
    auto fill = [&](QVector<QLabel*>& titles, QVector<QLabel*>& values, const DebugCheckRow* rows,
                    const bool* oks, const QString* texts) {
        const int n = qMin(titles.size(), values.size());
        for (int i = 0; i < n && i < 3; ++i) {
            titles[i]->setText(QString("%1&nbsp;%2").arg(mark(oks[i]), QString::fromUtf8(rows[i].name)));
            values[i]->setText(tone(texts[i], oks[i]));
        }
    };

    // ---------------- 数据就绪判定 ----------------
    const bool hasCalib = !m_CameraMatrixL.empty() && !m_CameraMatrixR.empty()
                          && !m_R.empty() && !m_T.empty();
    const int rotPairs = qMin(m_rotSeqLeftPaths.size(), m_rotSeqRightPaths.size());
    const bool hasRotSeq = rotPairs > 0;
    const bool hasAxis = !m_rotAxisDirection.empty() && !m_rotAxisPoint.empty()
                         && cv::norm(m_rotAxisDirection) > 0.1;
    const int reconPairs = qMin(m_reconLeftPaths.size(), m_reconRightPaths.size());
    const bool hasReconSeq = reconPairs > 0;

    int pointCount = 0, faceCount = 0;
    if (m_builder) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr c = m_builder->getFinalPointCloud();
        if (c && !c->empty()) pointCount = static_cast<int>(c->size());
        pcl::PolygonMesh mesh = m_builder->getFinalMesh();
        if (!mesh.polygons.empty()) faceCount = static_cast<int>(mesh.polygons.size());
    }
    const bool hasResult = pointCount > 0;
    const bool hasStats = !m_lastChessboardStats.isEmpty();

    bool cOk[3] = { hasCalib, hasRotSeq, hasAxis };
    QString cTxt[3] = {
        hasCalib ? (m_StereoRms > 1e-9 ? QString("已就绪 · RMS %1 px").arg(m_StereoRms, 0, 'f', 3)
                                       : QString("已就绪"))
                 : QString("未标定"),
        hasRotSeq ? QString("%1 对").arg(rotPairs) : QString("未加载"),
        hasAxis   ? QString("已标定") : QString("未标定")
    };
    bool rOk[3] = { hasReconSeq, hasResult, hasStats };
    QString rTxt[3] = {
        hasReconSeq ? QString("%1 对").arg(reconPairs) : QString("未加载"),
        hasResult   ? QString("%1 点").arg(pointCount) : QString("尚未生成"),
        hasStats    ? QString("已生成") : QString("未生成")
    };
    fill(m_dbgCalibTitle, m_dbgCalibValue, kCalibChecks, cOk, cTxt);
    fill(m_dbgReconTitle, m_dbgReconValue, kReconChecks, rOk, rTxt);

    const QString sep = QString(" <span style='color:%1'>·</span> ").arg(Theme::BORDER);

    // ---------------- ② 标定质量指标条 ----------------
    if (lblCalibMetrics) {
        const cv::Vec4f& pl = m_LaserPlaneEquation;
        const bool hasPlane = (std::fabs(static_cast<double>(pl[0]))
                             + std::fabs(static_cast<double>(pl[1]))
                             + std::fabs(static_cast<double>(pl[2]))) > 1e-9;

        QString epiStr = "—";
        if (!m_perPairEpiErrors.empty()) {
            double worst = 0.0;
            for (size_t i = 0; i < m_perPairEpiErrors.size(); ++i)
                worst = std::max(worst, std::fabs(m_perPairEpiErrors[i]));
            epiStr = QString("%1 px").arg(worst, 0, 'f', 3);
        }

        QString axisStr = "—";
        if (m_rotatingCalibrator) {
            const double r = m_rotatingCalibrator->getReprojectionError();
            if (r > 1e-9) axisStr = QString("%1").arg(r, 0, 'f', 4);
        }

        lblCalibMetrics->setText(
            QString("光平面 %1").arg(hasPlane ? tone("已标定", true) : tone("未标定", false))
            + sep + QString("立体 RMS %1").arg(m_StereoRms > 1e-9
                        ? QString("%1 px").arg(m_StereoRms, 0, 'f', 3) : QString("—"))
            + sep + QString("极线最大误差 %1").arg(epiStr)
            + sep + QString("轴标定残差 %1").arg(axisStr));
    }

    // ---------------- ③ 重建规模指标条 ----------------
    if (lblReconMetrics) {
        // 转台一圈覆盖率: S5 的闭环均摊假设"扫完一圈(360°)", 步长×帧数不符会直接造成错位
        const double stepDeg = spinAngleStep ? spinAngleStep->value() : 0.0;
        QString angStr;
        if (hasReconSeq && stepDeg > 0.0) {
            const double total = reconPairs * stepDeg;
            const bool full = (std::fabs(total - 360.0) <= 5.0) || (std::fabs(total - 720.0) <= 10.0);
            angStr = QString("%1对 × %2° = %3° %4")
                         .arg(reconPairs).arg(stepDeg, 0, 'f', 1).arg(total, 0, 'f', 0)
                         .arg(full ? QString("✓") : QString("<span style='color:%1'>⚠ 非整圈</span>").arg(Theme::WARNING));
        } else {
            angStr = "—";
        }
        lblReconMetrics->setText(
            QString("点云 %1").arg(hasResult ? tone(QString("%1 点").arg(pointCount), true)
                                             : tone(QString("尚未生成"), false))
            + sep + QString("网格 %1").arg(faceCount > 0 ? QString("%1 面").arg(faceCount) : QString("—"))
            + sep + QString("序列 %1").arg(hasReconSeq ? QString("%1 对").arg(reconPairs)
                                                       : QString("未加载"))
            + sep + QString("转角 %1").arg(angStr));
    }

    // ---------------- ③ 调试帧号范围随序列长度自适应 ----------------
    if (btnAxisDiagRun) {
        // 诊断只依赖"旋转序列 + 相机参数"：标定失败时反而更需要它，因此不作为"轴已标定"的附属功能
        btnAxisDiagRun->setEnabled(hasRotSeq && hasCalib);
    }

    if (lblDbgSeqInfo && spinPreviewIndex && btnPreviewPrev && btnPreviewNext) {
        if (hasReconSeq) {
            spinPreviewIndex->setRange(0, reconPairs - 1);
            if (spinPreviewIndex->value() > reconPairs - 1) spinPreviewIndex->setValue(reconPairs - 1);
            lblDbgSeqInfo->setText(QString("共 %1 对").arg(reconPairs));
            btnPreviewPrev->setEnabled(true);
            btnPreviewNext->setEnabled(true);
        } else {
            spinPreviewIndex->setRange(0, 0);
            lblDbgSeqInfo->setText("序列未加载");
            btnPreviewPrev->setEnabled(false);
            btnPreviewNext->setEnabled(false);
        }
    }

    // ---- ② 过程曲线按钮: 由"数据是否已经产生"决定, 而不是由当前前置条件决定 ----
    // 诊断跑过一次后 m_axisDiag 会一直留着, 此时即使用户换了序列/清了标定,
    // 上次的曲线仍然是有意义的存档 —— 所以这里只跟着数据走, 不跟着前置条件走。
    if (btnCircleFitCurve) btnCircleFitCurve->setEnabled(!m_axisDiag.circleFit.obs.empty());
    if (btnBaIterCurve)    btnBaIterCurve->setEnabled(!m_axisDiag.baIterRmsPx.empty());

    // ---- ③ 重建测试按钮 ----
    if (btnQuickYield) btnQuickYield->setEnabled(hasReconSeq);
    if (btnFullRecon)  btnFullRecon->setEnabled(hasReconSeq && !m_reconRunning);
    // 重建调试包要读"上次重建留下的记录", 跑之前点了只能写出一个空包 —— 显式门控
    if (btnReconDebugExport)
        btnReconDebugExport->setEnabled(m_reconWorker && !m_reconWorker->viewRecords().empty()
                                        && m_hasReconJob && !m_reconRunning);
}

void MainWindow::onRefreshDebugStatus()
{
    updateDebugStatus();
    appendDebugLog(QString("[%1] 已刷新前置条件状态")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
}

// ==================== [由 Tab4/Tab5 迁移] onReopenChessboard3DView ====================
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

// ==================== [由 Tab4/Tab5 迁移] onReopenChessboardStats ====================
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

// ==================== [由 Tab4/Tab5 迁移] onMultiFrameChessboard3D ====================
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
        // dst 必须是空 Mat: 写成 `cv::Mat imgL = imgL_orig;` 再 remap 会让 remap 原地改写
        // imgL_orig (尺寸类型一致时不重新分配)。此处 imgL_orig 后面没再用, 所以现在没症状,
        // 但同样的写法已经在「单帧调试」里造成过匹配点整体偏出光条的 bug —— 一并按正确写法来。
        cv::Mat imgL, imgR;
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

    // ==== 弹窗显示 (替代原先写入 Tab4 结果框) ====
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
    appendDebugLog(QString("[多帧棋盘格] %1帧%2点 Z展宽%3mm 平面RMS %4mm %5")
        .arg(totalFrames).arg((int)allCloud->size()).arg(gzMax-gzMin,0,'f',2).arg(plane_rms,0,'f',3)
        .arg(pass ? "✅" : "❌"));
}

// ==================== [由 Tab4/Tab5 迁移] onVerifyChessboard3D ====================
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
            appendDebugLog(QString(
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
        appendDebugLog(orderDiag);

        // 记录恢复后的最佳结果
        if (bestErr < epi_err_sum * 0.9) {
            double impct = (1.0 - bestErr / epi_err_sum) * 100;
            appendDebugLog(QString(
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
    if (pts_rect.cols > 0) zr_mean/=pts_rect.cols;

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
        if (pts_orig.cols > 0) zo_mean/=pts_orig.cols;
    }

    // 输出对比
    appendDebugLog(QString("[棋盘格验证] 帧#%1 校正后极线Y误差: 均值%2px 最大%3px")
        .arg(idx).arg(epi_err_sum,0,'f',2).arg(epi_err_max,0,'f',2));
    appendDebugLog(QString("[角点对应] 视差dx: 均值%1 范围[%2,%3] | Y差均值:%4px (应≈0)")
        .arg(dx_mean,0,'f',1).arg(dx_min,0,'f',1).arg(dx_max,0,'f',1).arg(dy_mean,0,'f',2));
    // 单目fx验证: 数10格(100mm)的像素距, 反推fx
    // 棋盘格是11×8, 取第0列和第10列(间隔100mm)的两角点
    int nCols = m_boardSize.width;
    if (nCols >= 11 && cornersL.size() >= (size_t)(10 * m_boardSize.height + 1)) {
        double pix_dist = cv::norm(cornersL[0] - cornersL[10]); // 同行, 间隔10列=100mm
        double fx_from_image = pix_dist * zr_mean / 100.0;
        appendDebugLog(QString("[fx验证] 10格像素距=%1px @Z=%2mm → fx≈%3 (标称%4) %5")
            .arg(pix_dist,0,'f',1).arg(zr_mean,0,'f',1).arg(fx_from_image,0,'f',1)
            .arg(m_CameraMatrixL.at<double>(0,0),0,'f',1)
            .arg(std::fabs(fx_from_image - m_CameraMatrixL.at<double>(0,0)) < 50 ? "✅fx正确" : "❌fx偏差大"));
    }
    appendDebugLog(QString("[棋盘格3D-校正系] Z均值%1 Z范围[%2,%3] 展宽%4mm")
        .arg(zr_mean,0,'f',1).arg(zr_min,0,'f',1).arg(zr_max,0,'f',1).arg(zr_max-zr_min,0,'f',1));
    appendDebugLog(QString("[棋盘格3D-原始系] Z均值%1 Z范围[%2,%3] 展宽%4mm%5")
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
                appendDebugLog(QString(
                    "\n[标定对对比] 第1对标定图 极线Y误差: %1px (max %2px) | Z均值%3 Z展宽%4mm %5")
                    .arg(calYErr,0,'f',2).arg(calYMax,0,'f',2)
                    .arg(zCal,0,'f',1).arg(zCalMax-zCalMin,0,'f',1)
                    .arg((calYErr < 1.0 && (zCalMax-zCalMin) < 5.0) ? "✅标定对正常→旋转序列图像有问题" : "❌标定对也异常→立体标定本身错误"));
                if (calYErr < 1.0 && epi_err_sum > 5.0) {
                    appendDebugLog("⚠️ 标定对Y误差<1px但旋转序列帧>5px → 旋转序列图像与标定图像不一致(分辨率/相机位置/对焦)");
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
// ============================================================================
//                转台标定诊断 (Tab6 核心调试功能)
//   流程: 一键诊断 → 逐帧指标表 → 一致性检查 → 导出核心调试内容
//   逐项核对本工程中转台标定/重建最易出问题的环节:
//     · 角点与内参      → 左右相机逐帧 PnP 重投影误差
//     · 轴解的稳健性    → 圆拟合 vs BA 两法互验、分相机光心共圆残差
//     · 安装几何        → 棋盘姿态一致性、左右光心轴向高差
//     · 角度序列        → 步长均匀性、实测步长 vs Tab5 设定步长
//     · 坐标系一致性    → 轴在相机系/转台基准系下的表达 (S4 输出与 S5 输入必须同系)
// ============================================================================

// 矩阵格式化 (报告/CSV 用; 兼容 CV_32F/CV_64F, 避免标定文件里是 float 时崩溃)
static QString fmtMatrix(const cv::Mat& src, int prec = 4)
{
    if (src.empty()) return "(空)";
    cv::Mat m;
    if (src.type() == CV_32F)      src.convertTo(m, CV_64F);
    else if (src.type() == CV_64F) m = src;
    else                           return "(非数值矩阵)";
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec);
    for (int r = 0; r < m.rows; ++r) {
        for (int c = 0; c < m.cols; ++c) {
            os << (c ? ", " : "") << m.at<double>(r, c);
        }
        if (r + 1 < m.rows) os << " ; ";
    }
    return QString::fromStdString(os.str());
}

static QString fmtVec3d(const double* v, int prec = 4)
{
    return QString("[%1, %2, %3]")
        .arg(v[0], 0, 'f', prec).arg(v[1], 0, 'f', prec).arg(v[2], 0, 'f', prec);
}

// ---------------------------------------------------------------------------
// 运行诊断
// ---------------------------------------------------------------------------
void MainWindow::onRunAxisDiagnosis()
{
    if (!m_rotatingCalibrator) { QMessageBox::warning(this, "无法诊断", "标定器未初始化"); return; }
    if (m_rotSeqLeftPaths.isEmpty() || m_rotSeqRightPaths.isEmpty()) {
        QMessageBox::warning(this, "缺少数据", "请先到「转台标定」页加载旋转序列图像！");
        return;
    }
    if (m_CameraMatrixL.empty() || m_CameraMatrixR.empty() || m_R.empty() || m_T.empty()) {
        QMessageBox::warning(this, "缺少参数", "请先到「相机标定」页完成双目标定！");
        return;
    }

    const int pairs = qMin(m_rotSeqLeftPaths.size(), m_rotSeqRightPaths.size());
    appendDebugLog("====== [转台标定诊断] 开始 ======");
    appendDebugLog(QString("序列: %1 对 | 棋盘格: %2 x %3 内角点, 方格 %4 mm")
        .arg(pairs).arg(m_boardSize.width).arg(m_boardSize.height).arg(m_squareSize, 0, 'f', 2));

    QApplication::setOverrideCursor(Qt::WaitCursor);
    m_rotatingCalibrator->setCameraParams(m_CameraMatrixL, m_DistCoeffsL,
                                          m_CameraMatrixR, m_DistCoeffsR, m_R, m_T);
    m_rotatingCalibrator->setPatternParams(m_boardSize, m_squareSize);
    m_rotatingCalibrator->setInputData(m_rotSeqLeftPaths, m_rotSeqRightPaths);

    // 诊断内部会重跑一次完整 process(); 接上进度回调让界面有反馈 (与 Tab4 同一套机制)
    if (progressAxisDiag) {
        progressAxisDiag->setRange(0, 100);
        progressAxisDiag->setValue(0);
        progressAxisDiag->setFormat("准备中 0%");
    }
    m_rotatingCalibrator->setProgressFn([this](const QString& stage, int percent) {
        if (!progressAxisDiag) return;
        progressAxisDiag->setValue(percent);
        progressAxisDiag->setFormat(QString("%1  %2%").arg(stage).arg(percent));
        // 同步执行时必须主动泵事件, 否则重绘要等整个诊断跑完才发生
        QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    });
    // 诊断期间禁用入口按钮, 避免重入 (下面每条退出路径都要恢复)
    btnAxisDiagRun->setEnabled(false);
    QApplication::processEvents();

    const double cfgStep = spinAngleStep ? spinAngleStep->value() : -1.0;
    m_axisDiag = m_rotatingCalibrator->runDiagnostics(cfgStep);
    m_axisDiagTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    m_rotatingCalibrator->setProgressFn(nullptr);
    QApplication::restoreOverrideCursor();
    btnAxisDiagRun->setEnabled(true);
    if (progressAxisDiag) {
        progressAxisDiag->setValue(m_axisDiag.valid ? 100 : 0);
        progressAxisDiag->setFormat(m_axisDiag.valid ? "诊断完成 100%" : "诊断未完成");
    }

    if (!m_axisDiag.valid) {
        appendDebugLog("❌ " + m_axisDiag.summary);
        if (lblAxisDiagSummary) {
            lblAxisDiagSummary->setText(QString("<b><span style='color:%1'>诊断未完成</span></b><br>%2")
                .arg(Theme::DANGER, m_axisDiag.summary));
        }
        QMessageBox::warning(this, "诊断未完成", m_axisDiag.summary);
        return;
    }

    // ---- 界面层附加检查: 当前生效轴 vs 本次诊断轴 (坐标系/符号一致性) ----
    {
        Calib::AxisDiagCheck c;
        c.name = "当前生效轴 vs 诊断轴";
        c.expect = "夹角 ≤ 0.5°";
        if (!m_rotAxisDirection.empty() && cv::norm(m_rotAxisDirection) > 0.1) {
            Eigen::Vector3d cur(m_rotAxisDirection.at<double>(0),
                                m_rotAxisDirection.at<double>(1),
                                m_rotAxisDirection.at<double>(2));
            Eigen::Vector3d dia(m_axisDiag.axisDirTurntable[0],
                                m_axisDiag.axisDirTurntable[1],
                                m_axisDiag.axisDirTurntable[2]);
            double dot = cur.normalized().dot(dia.normalized());
            dot = std::max(-1.0, std::min(1.0, dot));
            double ang = std::acos(dot) * 180.0 / M_PI;
            c.value = QString("%1°").arg(ang, 0, 'f', 3);
            c.ok = (ang <= 0.5);
            if (std::fabs(ang - 180.0) < 1.0) {
                c.ok = false;
                c.advice = "轴向符号相反！S5 会用相反方向旋转各视角 → 点云彻底错位。"
                           "请在「转台标定」页重新执行轴标定（或重新加载标定文件）";
            } else if (!c.ok) {
                c.advice = "当前使用的轴与本次诊断结果不一致：可能是旧标定文件、"
                           "或轴标定后又改过相机参数，建议重新执行轴标定";
            } else {
                c.advice = "一致";
            }
        } else {
            c.value = "未标定";
            c.ok = false;
            c.advice = "还没有可用的轴参数：请到「转台标定」页执行轴标定";
        }
        m_axisDiag.checks.push_back(c);
    }
    // ---- 坐标系说明行 (恒通过, 用于提醒 S4/S5 的坐标系约定) ----
    {
        Calib::AxisDiagCheck c;
        c.name = "轴坐标系 (S4 输出 / S5 输入)";
        c.ok = true;
        c.value = "转台基准系(棋盘0系)";
        c.expect = "与 S4 输出同系";
        c.advice = QString("相机系下为 %1（仅在原始相机系里做验证时才使用）")
                       .arg(fmtVec3d(m_axisDiag.axisDirCam));
        m_axisDiag.checks.push_back(c);
    }

    // ---- 输出到统一调试日志 ----
    for (size_t i = 0; i < m_axisDiag.checks.size(); ++i) {
        const Calib::AxisDiagCheck& c = m_axisDiag.checks[i];
        appendDebugLog(QString("  %1 %2: %3 (期望 %4)")
            .arg(c.ok ? "✓" : "✗").arg(c.name, c.value, c.expect));
        if (!c.ok && !c.advice.isEmpty())
            appendDebugLog(QString("      ↳ 建议: %1").arg(c.advice));
    }
    appendDebugLog(QString("采用方案: %1 | 实测步长: %2° (设定 %3°) | 累计转角: %4°")
        .arg(m_axisDiag.source)
        .arg(m_axisDiag.stepMeanDeg, 0, 'f', 3)
        .arg(cfgStep > 0 ? QString::number(cfgStep, 'f', 2) : QString("未设置"))
        .arg(m_axisDiag.angleSpanDeg, 0, 'f', 1));
    appendDebugLog(QString("轴方向(转台系): %1 | 轴点(转台系): %2 mm")
        .arg(fmtVec3d(m_axisDiag.axisDirTurntable), fmtVec3d(m_axisDiag.axisPointTurntable, 2)));
    appendDebugLog(QString("光心共圆残差: 左 %1mm / 右 %2mm | BA重投影: %3px | 两法轴夹角: %4°")
        .arg(m_axisDiag.residLeftMm, 0, 'f', 3).arg(m_axisDiag.residRightMm, 0, 'f', 3)
        .arg(m_axisDiag.baErrorPx, 0, 'f', 3).arg(m_axisDiag.axisDiffDeg, 0, 'f', 3));

    int hard = 0, soft = 0;
    for (size_t i = 0; i < m_axisDiag.frames.size(); ++i) {
        if (m_axisDiag.frames[i].flag == 2) ++hard;
        else if (m_axisDiag.frames[i].flag == 1) ++soft;
    }
    if (hard > 0) {
        QStringList bad;
        for (size_t i = 0; i < m_axisDiag.frames.size() && bad.size() < 12; ++i)
            if (m_axisDiag.frames[i].flag == 2)
                bad << QString("#%1").arg(m_axisDiag.frames[i].frameIdx);
        appendDebugLog(QString("⚠️ 异常帧 %1 个: %2").arg(hard).arg(bad.join(", ")));
    }
    appendDebugLog(QString("诊断结论: %1").arg(m_axisDiag.summary));
    appendDebugLog("提示: 点「导出调试包」可保存报告/逐帧CSV/参数快照/日志/异常帧图像。");

    if (lblAxisDiagSummary) {
        const int fails = m_axisDiag.failCount();
        QString html = QString("<b>%1</b> · %2 项检查通过 / %3 项未通过<br>")
            .arg(m_axisDiagTime)
            .arg(m_axisDiag.checkCount() - fails).arg(fails);
        html += QString("实测步长 %1° · 残差 左%2/右%3 mm · BA %4px · 异常帧 %5")
            .arg(m_axisDiag.stepMeanDeg, 0, 'f', 3)
            .arg(m_axisDiag.residLeftMm, 0, 'f', 3).arg(m_axisDiag.residRightMm, 0, 'f', 3)
            .arg(m_axisDiag.baErrorPx, 0, 'f', 3).arg(hard);
        lblAxisDiagSummary->setText(html);
        lblAxisDiagSummary->setStyleSheet(fails == 0 ? Theme::metricsStyle()
            : QString("QLabel { background-color: #fff6f5; border: 1px solid %1; border-radius: %2px;"
                      " padding: 6px 8px; color: %3; font-size: 11px; }")
                  .arg(Theme::DANGER).arg(Theme::BORDER_RADIUS).arg(Theme::TEXT_PRIMARY));
    }
    if (btnAxisDiagTable) btnAxisDiagTable->setEnabled(true);

    updateDebugStatus();
    showAxisDiagDialog();
}

void MainWindow::onShowAxisDiagTable()
{
    showAxisDiagDialog();
}

// ---------------------------------------------------------------------------
// 诊断结果窗口: 结论与检查项 | 逐帧指标表 | 轴参数与坐标系
// ---------------------------------------------------------------------------
void MainWindow::showAxisDiagDialog()
{
    const Calib::AxisDiagnostics& d = m_axisDiag;
    if (!d.valid && d.checks.empty()) {
        QMessageBox::information(this, "暂无诊断结果", "请先点击「运行转台标定诊断」。");
        return;
    }

    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle(QString("转台标定诊断 — %1").arg(m_axisDiagTime));
    dlg->resize(1180, 720);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    QVBoxLayout *lay = new QVBoxLayout(dlg);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(6);

    const int fails = d.failCount();
    QLabel *lblSum = new QLabel(QString(
        "<span style='font-size:13px;font-weight:bold;color:%1'>%2</span>"
        "<span style='color:%3'>　采用方案 %4 ｜ 有效帧 %5/%6 ｜ 实测步长 %7° (设定 %8) ｜ 累计转角 %9°</span>")
        .arg(fails == 0 ? Theme::SUCCESS : Theme::DANGER)
        .arg(d.summary)
        .arg(Theme::TEXT_SECONDARY)
        .arg(d.source).arg(d.framesValid).arg(d.framesTotal)
        .arg(d.stepMeanDeg, 0, 'f', 3)
        .arg(d.configuredStepDeg > 0 ? QString::number(d.configuredStepDeg, 'f', 2) : QString("未设置"))
        .arg(d.angleSpanDeg, 0, 'f', 1));
    lblSum->setWordWrap(true);
    lblSum->setStyleSheet(Theme::hintBoxStyle());
    lay->addWidget(lblSum);

    QTabWidget *tabs = new QTabWidget();
    tabs->setStyleSheet("QTabWidget::pane { border: 1px solid " + QString(Theme::BORDER) +
                        "; } QTabBar::tab { padding: 5px 14px; }");

    // ---------- 页1: 检查项 ----------
    {
        QTableWidget *t = new QTableWidget(static_cast<int>(d.checks.size()), 5);
        t->setHorizontalHeaderLabels(QStringList() << "状态" << "检查项" << "实测" << "期望" << "不通过时的建议");
        t->verticalHeader()->setVisible(false);
        t->setEditTriggers(QAbstractItemView::NoEditTriggers);
        t->setSelectionBehavior(QAbstractItemView::SelectRows);
        t->setWordWrap(true);
        t->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        t->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        t->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        t->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        t->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
        for (int i = 0; i < static_cast<int>(d.checks.size()); ++i) {
            const Calib::AxisDiagCheck& c = d.checks[i];
            QTableWidgetItem *it0 = new QTableWidgetItem(c.ok ? "✓ 通过" : "✗ 未通过");
            it0->setForeground(QBrush(QColor(c.ok ? Theme::SUCCESS : Theme::DANGER)));
            it0->setTextAlignment(Qt::AlignCenter);
            t->setItem(i, 0, it0);
            t->setItem(i, 1, new QTableWidgetItem(c.name));
            t->setItem(i, 2, new QTableWidgetItem(c.value));
            t->setItem(i, 3, new QTableWidgetItem(c.expect));
            t->setItem(i, 4, new QTableWidgetItem(c.ok ? QString("—") : c.advice));
            if (!c.ok) {
                for (int k = 0; k < 5; ++k)
                    if (t->item(i, k)) t->item(i, k)->setBackground(QBrush(QColor("#fff5f4")));
            }
        }
        t->resizeRowsToContents();
        tabs->addTab(t, QString("一致性检查 (%1)").arg(d.checks.size()));
    }

    // ---------- 页2: 逐帧指标 ----------
    {
        QWidget *page = new QWidget();
        QVBoxLayout *vl = new QVBoxLayout(page);
        vl->setContentsMargins(4, 4, 4, 4);
        QCheckBox *chkOnlyBad = new QCheckBox("仅显示 异常/注意 帧");
        chkOnlyBad->setStyleSheet(QString("font-size: 11px; color: %1;").arg(Theme::TEXT_SECONDARY));
        vl->addWidget(chkOnlyBad);

        QStringList heads;
        heads << "帧号" << "角度(°)" << "步长(°)" << "左重投影(px)" << "右重投影(px)"
              << "光心X(mm)" << "光心Y(mm)" << "光心Z(mm)" << "到轴距离(mm)"
              << "圆残差(mm)" << "棋盘-轴夹角(°)" << "状态" << "说明";
        QTableWidget *t = new QTableWidget(static_cast<int>(d.frames.size()), heads.size());
        t->setHorizontalHeaderLabels(heads);
        t->verticalHeader()->setVisible(false);
        t->setEditTriggers(QAbstractItemView::NoEditTriggers);
        t->setSelectionBehavior(QAbstractItemView::SelectRows);
        t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        t->horizontalHeader()->setSectionResizeMode(heads.size() - 1, QHeaderView::Stretch);

        auto fillTable = [&](bool onlyBad) {
            std::vector<const Calib::AxisFrameMetrics*> show;
            for (size_t i = 0; i < d.frames.size(); ++i)
                if (!onlyBad || d.frames[i].flag != 0) show.push_back(&d.frames[i]);
            t->setRowCount(0);                       // 先清空，避免过滤时残留旧行
            t->setRowCount(static_cast<int>(show.size()));
            for (size_t r = 0; r < show.size(); ++r) {
                const Calib::AxisFrameMetrics& m = *show[r];
                QStringList vals;
                vals << QString::number(m.frameIdx)
                     << QString::number(m.angleDeg, 'f', 2)
                     << QString::number(m.stepDeg, 'f', 3)
                     << QString::number(m.reprojPxLeft, 'f', 3)
                     << (m.reprojPxRight < 0 ? QString("—") : QString::number(m.reprojPxRight, 'f', 3))
                     << QString::number(m.centerLeft[0], 'f', 1)
                     << QString::number(m.centerLeft[1], 'f', 1)
                     << QString::number(m.centerLeft[2], 'f', 1)
                     << QString::number(m.radiusLeftMm, 'f', 2)
                     << QString::number(m.residLeftMm, 'f', 3)
                     << QString::number(m.planeNormAxisDeg, 'f', 2)
                     << (m.flag == 2 ? QString("✗ 异常") : (m.flag == 1 ? QString("⚠ 注意") : QString("✓ 正常")))
                     << (m.note.isEmpty() ? QString("—") : m.note);
                for (int k = 0; k < vals.size(); ++k) {
                    QTableWidgetItem *it = new QTableWidgetItem(vals[k]);
                    if (k > 0 && k < 11) it->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
                    if (k == 11) it->setTextAlignment(Qt::AlignCenter);
                    if (m.flag == 2) it->setBackground(QBrush(QColor("#ffeceb")));
                    else if (m.flag == 1) it->setBackground(QBrush(QColor("#fff8e6")));
                    if (k == 11)
                        it->setForeground(QBrush(QColor(m.flag == 2 ? Theme::DANGER
                                                       : (m.flag == 1 ? Theme::WARNING : Theme::SUCCESS))));
                    t->setItem(static_cast<int>(r), k, it);
                }
            }
        };
        fillTable(false);
        connect(chkOnlyBad, &QCheckBox::toggled, chkOnlyBad, [fillTable](bool on) { fillTable(on); });
        vl->addWidget(t, 1);
        tabs->addTab(page, QString("逐帧指标 (%1)").arg(d.frames.size()));
    }

    // ---------- 页3: 轴参数与坐标系 ----------
    {
        QTextEdit *txt = new QTextEdit();
        txt->setReadOnly(true);
        txt->setFont(QFont(Theme::MONO, 10));
        txt->setStyleSheet(Theme::terminalStyle());
        QString s;
        auto line = [&](const QString& k, const QString& v) { s += QString("%1: %2\n").arg(k, -26).arg(v); };
        line("诊断时间", m_axisDiagTime);
        line("采用方案", d.source);
        line("轴方向(转台基准系)", fmtVec3d(d.axisDirTurntable));
        line("轴点(转台基准系)", fmtVec3d(d.axisPointTurntable, 2) + " mm");
        line("轴方向(相机系)", fmtVec3d(d.axisDirCam));
        line("轴点(相机系)", fmtVec3d(d.axisPointCam, 2) + " mm");
        s += "\n-- 立体外参 / 基准位姿 (R_base: 棋盘0系 → 相机系) --\n";
        line("R_base", fmtMatrix(m_R_base));
        line("T_base", fmtMatrix(m_T_base));
        line("R (立体)", fmtMatrix(m_R));
        line("T (立体)", fmtMatrix(m_T));
        s += "\n-- 圆拟合质量 (分相机) --\n";
        line("半径 左/右", QString("%1 / %2 mm").arg(d.radiusLeftMm, 0, 'f', 3).arg(d.radiusRightMm, 0, 'f', 3));
        line("共圆残差 左/右", QString("%1 / %2 mm").arg(d.residLeftMm, 0, 'f', 3).arg(d.residRightMm, 0, 'f', 3));
        line("左右光心轴向高差", QString("%1 mm").arg(d.axialGapMm, 0, 'f', 3));
        line("BA 重投影误差", QString("%1 px").arg(d.baErrorPx, 0, 'f', 4));
        line("圆拟合 vs BA 轴夹角", QString("%1°").arg(d.axisDiffDeg, 0, 'f', 4));
        line("立体外参方向修正", d.stereoFlipped ? "已触发(异常!)" : "未触发(正常)");
        s += "\n-- 角度序列 --\n";
        line("实测步长 均值/标准差", QString("%1° / %2°").arg(d.stepMeanDeg, 0, 'f', 4).arg(d.stepStdDeg, 0, 'f', 4));
        line("实测步长 最小/最大", QString("%1° / %2°").arg(d.stepMinDeg, 0, 'f', 4).arg(d.stepMaxDeg, 0, 'f', 4));
        line("累计转角", QString("%1°").arg(d.angleSpanDeg, 0, 'f', 3));
        line("UI 设定步长", d.configuredStepDeg > 0 ? QString("%1°").arg(d.configuredStepDeg, 0, 'f', 3)
                                                    : QString("未提供"));
        s += "\n-- 棋盘格姿态 --\n";
        line("法向与轴夹角 均值/标准差",
             QString("%1° / %2°").arg(d.planeNormMeanDeg, 0, 'f', 3).arg(d.planeNormStdDeg, 0, 'f', 3));
        s += "\n-- 相机内参 --\n";
        line("K_L", fmtMatrix(m_CameraMatrixL, 3));
        line("D_L", fmtMatrix(m_DistCoeffsL, 4));
        line("K_R", fmtMatrix(m_CameraMatrixR, 3));
        line("D_R", fmtMatrix(m_DistCoeffsR, 4));
        txt->setPlainText(s);
        tabs->addTab(txt, "轴参数与坐标系");
    }

    lay->addWidget(tabs, 1);

    QHBoxLayout *btnRow = new QHBoxLayout();
    btnRow->addStretch();
    QPushButton *btnExp = new QPushButton("导出调试包");
    btnExp->setStyleSheet(Theme::warningButton());
    btnExp->setCursor(Qt::PointingHandCursor);
    btnExp->setToolTip("导出诊断报告 + 逐帧指标 + 参数快照 + 全部日志 + 异常帧图像");
    QPushButton *btnClose = new QPushButton("关闭");
    btnClose->setStyleSheet(Theme::ghostButton());
    btnClose->setCursor(Qt::PointingHandCursor);
    connect(btnExp, &QPushButton::clicked, this, &MainWindow::onExportDebugPackage);
    connect(btnClose, &QPushButton::clicked, dlg, &QDialog::close);
    btnRow->addWidget(btnExp);
    btnRow->addWidget(btnClose);
    lay->addLayout(btnRow);

    dlg->show();
}

// ---------------------------------------------------------------------------
// 诊断报告 (Markdown)
// ---------------------------------------------------------------------------
QString MainWindow::buildAxisDiagReport() const
{
    const Calib::AxisDiagnostics& d = m_axisDiag;
    QString s;
    s += "# 转台标定诊断报告\n\n";
    s += QString("- 生成时间: %1\n").arg(m_axisDiagTime.isEmpty()
             ? QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") : m_axisDiagTime);
    s += QString("- 旋转序列: %1 对\n").arg(qMin(m_rotSeqLeftPaths.size(), m_rotSeqRightPaths.size()));
    s += QString("- 棋盘格: %1 x %2 内角点, 方格 %3 mm\n")
             .arg(m_boardSize.width).arg(m_boardSize.height).arg(m_squareSize, 0, 'f', 2);
    s += QString("- 双目标定 RMS: %1 px\n").arg(m_StereoRms, 0, 'f', 4);
    s += "\n## 1. 结论\n\n";
    s += QString("**%1**\n\n").arg(d.summary);
    if (!d.valid) {
        s += "> 诊断未能完成，请先解决有效帧不足 / 相机参数缺失的问题。\n";
        return s;
    }
    s += QString("采用方案: %1 ｜ 有效帧 %2/%3 ｜ 实测步长 %4° (设定 %5°) ｜ 累计转角 %6°\n")
             .arg(d.source).arg(d.framesValid).arg(d.framesTotal)
             .arg(d.stepMeanDeg, 0, 'f', 3)
             .arg(d.configuredStepDeg > 0 ? QString::number(d.configuredStepDeg, 'f', 2) : "未设置")
             .arg(d.angleSpanDeg, 0, 'f', 1);
    s += QString("算法设置: %1 ｜ BA 误差上限 %2 px\n")
             .arg(d.methodSetting.isEmpty() ? QString("—") : d.methodSetting)
             .arg(d.baErrorLimitPx >= 0 ? QString::number(d.baErrorLimitPx, 'f', 2) : QString("—"));
    s += "> 实测步长与累计转角已按序列连续性解缠绕（相邻帧转角差归一到 (-180°,180°]）。"
         "转台转满一圈时原始角度会被 Rodrigues 折回到 (-180°,180°]，未解缠绕会误报"
         "「步长均匀性 / 累计转角」不通过。\n";

    s += "\n## 2. 一致性检查\n\n";
    s += "| 状态 | 检查项 | 实测 | 期望 | 不通过时的建议 |\n|---|---|---|---|---|\n";
    for (size_t i = 0; i < d.checks.size(); ++i) {
        const Calib::AxisDiagCheck& c = d.checks[i];
        s += QString("| %1 | %2 | %3 | %4 | %5 |\n")
                 .arg(c.ok ? "✅" : "❌").arg(c.name).arg(c.value).arg(c.expect)
                 .arg(c.ok ? QString("—") : c.advice);
    }

    s += "\n## 3. 关键数值\n\n";
    s += QString("- 轴方向(转台基准系): %1\n").arg(fmtVec3d(d.axisDirTurntable));
    s += QString("- 轴点(转台基准系): %1 mm\n").arg(fmtVec3d(d.axisPointTurntable, 2));
    s += QString("- 轴方向(相机系): %1\n").arg(fmtVec3d(d.axisDirCam));
    s += QString("- 轴点(相机系): %1 mm\n").arg(fmtVec3d(d.axisPointCam, 2));
    s += QString("- 圆拟合半径 左/右: %1 / %2 mm\n").arg(d.radiusLeftMm, 0, 'f', 3).arg(d.radiusRightMm, 0, 'f', 3);
    s += QString("- 光心共圆残差 左/右: %1 / %2 mm\n").arg(d.residLeftMm, 0, 'f', 3).arg(d.residRightMm, 0, 'f', 3);
    s += QString("- 左右光心轴向高差 Δh: %1 mm\n").arg(d.axialGapMm, 0, 'f', 3);
    s += QString("- 轴估计方式设定: %1 (BA 误差上限 %2 px)\n")
             .arg(d.methodSetting.isEmpty() ? QString("—") : d.methodSetting)
             .arg(d.baErrorLimitPx >= 0 ? QString::number(d.baErrorLimitPx, 'f', 2) : QString("—"));
    s += QString("- BA 重投影误差: %1 px (仅用于判断 BA 是否可信, 与圆拟合的 mm 不同量纲)\n")
             .arg(d.baErrorPx, 0, 'f', 4);
    s += QString("- 圆拟合轴 与 BA 轴夹角: %1°\n").arg(d.axisDiffDeg, 0, 'f', 4);
    s += QString("- 立体外参方向自动修正: %1\n").arg(d.stereoFlipped ? "已触发 (异常)" : "未触发 (正常)");
    s += QString("- 棋盘格法向与轴夹角: %1° ± %2°\n").arg(d.planeNormMeanDeg, 0, 'f', 3).arg(d.planeNormStdDeg, 0, 'f', 3);
    s += QString("- 实测步长: %1° ± %2° (范围 %3° ~ %4°)\n")
             .arg(d.stepMeanDeg, 0, 'f', 4).arg(d.stepStdDeg, 0, 'f', 4)
             .arg(d.stepMinDeg, 0, 'f', 4).arg(d.stepMaxDeg, 0, 'f', 4);

    s += "\n## 4. 异常帧\n\n";
    QStringList bad, warn;
    for (size_t i = 0; i < d.frames.size(); ++i) {
        const Calib::AxisFrameMetrics& f = d.frames[i];
        if (f.flag == 2) bad << QString("#%1(%2)").arg(f.frameIdx).arg(f.note);
        else if (f.flag == 1) warn << QString("#%1(%2)").arg(f.frameIdx).arg(f.note);
    }
    s += bad.isEmpty() ? QString("- 无异常帧\n") : QString("- 异常帧: %1\n").arg(bad.join("; "));
    if (!warn.isEmpty()) s += QString("- 注意帧: %1\n").arg(warn.join("; "));
    if (!m_reconLeftPaths.isEmpty())
        s += QString("- 重建序列(参考): %1 对, Tab5 设定步长 %2°\n")
                 .arg(qMin(m_reconLeftPaths.size(), m_reconRightPaths.size()))
                 .arg(spinAngleStep ? spinAngleStep->value() : 0.0, 0, 'f', 2);

    s += "\n## 5. 复现与排查建议\n\n";
    s += "1. 参数快照见 `04_标定参数.csv` / `05_重建参数.csv`，逐帧数据见 `02_逐帧指标.csv`；\n";
    s += "2. 异常帧图像已复制到 `frames/` 目录，可直接核对角点检测/曝光/棋盘是否出画；\n";
    s += "3. `06_Tab6调试输出.txt` / `07_Tab5重建日志.txt` 为 Tab6 / Tab5 的完整日志；\n";
    s += "4. 轴参数与坐标系定义见 `03_轴与坐标系.csv`（S4 输出与 S5 输入必须同为"
         "「转台基准系」，即棋盘第 0 帧坐标系）；\n";
    s += "5. 逐项检查明细见 `09_检查项.csv`；\n";
    s += "6. 圆拟合观测点见 `10_圆拟合观测.csv`（plane 内 x/y 与逐点半径偏差），"
         "BA 逐迭代重投影见 `11_BA迭代.csv`（iter, rms_px, accepted）。"
         "这两份就是 Tab6「圆拟合曲线 / BA迭代曲线」两个弹窗所画的数据，"
         "可在任意工具里重绘或换判据分析。\n";

    s += "\n## 6. 重建侧提示 (S1/S2/S3)\n\n";
    s += "轴标定只决定视角旋转；点云的稠密程度取决于 S1 光条提取、S2 极线匹配与 S3 三角化。\n";
    s += "**先看 S1**：「逐视角汇总」的「S1提取空」为 0 只说明每帧都提得到点，"
         "不说明提得够。若「S1:左N/右M」两边差得很多（例如右图只有左图的一半），"
         "多半是红光掩膜把一部分光条判掉了 —— 检查 Tab5 的「红色色相容差 / 饱和度下限」，"
         "光条越亮越白，色相会从暗处的 ~174° 漂到亮处的 ~155°。\n";
    s += "再看 **S3 剔除里的「平面残差」一项**：它在四项剔除中占绝对多数时，"
         "说明「左射线∩光平面」求出的深度与右图对不上 —— 光平面有偏差或工作距离过近时都会这样。"
         "次行「右图重投影误差分布」给出误差直方图，用来区分"
         "「大片误差挤在阈值上方一小段」(放宽阈值/重标光平面即可) 与"
         "「误差成片落在几十上百像素」(光平面基本是错的，放宽只会放进错点)。\n";
    s += "若要定位到具体帧，用 Tab6「快速产出测试」跑前 N 帧，"
         "其「逐帧有效点数」曲线可直接看出损失是均匀的还是集中在少数帧。\n";
    s += "本报告只覆盖转台轴标定，重建侧指标请对照上述日志与快速产出测试。\n";
    return s;
}

// ---------------------------------------------------------------------------
// 导出核心调试内容
// ---------------------------------------------------------------------------
void MainWindow::onExportDebugPackage()
{
    // 默认落在 Resources/debug，避免每次都手动导航到导出目录
    const QString parent = QFileDialog::getExistingDirectory(this, "选择导出目录", dirDebug);
    if (parent.isEmpty()) return;

    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString dir = QDir(parent).filePath("axis_debug_" + stamp);
    if (!QDir().mkpath(dir)) {
        QMessageBox::warning(this, "导出失败", QString("无法创建目录:\n%1").arg(dir));
        return;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    QString err;
    const bool ok = exportDebugPackage(dir, &err);
    QApplication::restoreOverrideCursor();

    if (!ok) {
        QMessageBox::warning(this, "导出失败", err);
        appendDebugLog("❌ 调试包导出失败: " + err);
        return;
    }
    appendDebugLog(QString("✅ 调试包已导出: %1").arg(dir));
    appendDebugLog("   内容: 01_诊断报告.md / 02_逐帧指标.csv / 03_轴与坐标系.csv / 04_标定参数.csv "
                   "/ 05_重建参数.csv / 06_Tab6调试输出.txt / 07_Tab5重建日志.txt / 08_Tab4轴标定日志.txt "
                   "/ 09_检查项.csv / 10_圆拟合观测.csv / 11_BA迭代.csv / frames/ 异常帧图像");

    QMessageBox::information(this, "导出完成",
        QString("核心调试内容已导出到:\n%1\n\n"
                "包含诊断报告、逐帧指标、标定/重建参数快照、圆拟合观测与BA迭代数据、"
                "各页日志与异常帧图像。").arg(dir));
}

bool MainWindow::exportDebugPackage(const QString& dir, QString* err)
{
    auto writeText = [&](const QString& name, const QString& content, bool utf8Bom = false) -> bool {
        QFile f(QDir(dir).filePath(name));
        if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            if (err) *err = QString("无法写入 %1").arg(f.fileName());
            return false;
        }
        QTextStream ts(&f);
        ts.setCodec("UTF-8");
        if (utf8Bom) ts.setGenerateByteOrderMark(true);
        ts << content;
        ts.flush();
        f.close();
        return true;
    };

    const Calib::AxisDiagnostics& d = m_axisDiag;

    // 01 诊断报告
    if (!writeText("01_诊断报告.md", buildAxisDiagReport())) return false;

    // 02 逐帧指标
    {
        // 前 19 列保持原 schema 不变(便于与历史导出逐列对照), 位姿二义性的两个数值列
        // 追加在末尾; 结论本身写在已有的「说明」列里。
        QString csv = "帧号,角度(deg),步长(deg),左重投影(px),右重投影(px),光心X(mm),光心Y(mm),光心Z(mm),"
                      "到轴距离(mm),圆残差(mm),右光心X(mm),右光心Y(mm),右光心Z(mm),右到轴距离(mm),"
                      "右圆残差(mm),轴向坐标(mm),棋盘法向与轴夹角(deg),状态,说明,"
                      "翻转候选步长(deg),翻转候选重投影(px)\n";
        for (size_t i = 0; i < d.frames.size(); ++i) {
            const Calib::AxisFrameMetrics& m = d.frames[i];
            csv += QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16,%17,%18,\"%19\",%20,%21\n")
                .arg(m.frameIdx)
                .arg(m.angleDeg, 0, 'f', 4).arg(m.stepDeg, 0, 'f', 4)
                .arg(m.reprojPxLeft, 0, 'f', 4)
                .arg(m.reprojPxRight < 0 ? QString("") : QString::number(m.reprojPxRight, 'f', 4))
                .arg(m.centerLeft[0], 0, 'f', 3).arg(m.centerLeft[1], 0, 'f', 3).arg(m.centerLeft[2], 0, 'f', 3)
                .arg(m.radiusLeftMm, 0, 'f', 3).arg(m.residLeftMm, 0, 'f', 4)
                .arg(m.centerRight[0], 0, 'f', 3).arg(m.centerRight[1], 0, 'f', 3).arg(m.centerRight[2], 0, 'f', 3)
                .arg(m.radiusRightMm, 0, 'f', 3).arg(m.residRightMm, 0, 'f', 4)
                .arg(m.axialCoordMm, 0, 'f', 3).arg(m.planeNormAxisDeg, 0, 'f', 4)
                .arg(m.flag == 2 ? "异常" : (m.flag == 1 ? "注意" : "正常"))
                .arg(m.note)
                .arg(m.stepAltDeg < 0 ? QString("") : QString::number(m.stepAltDeg, 'f', 4))
                .arg(m.reprojAltPx < 0 ? QString("") : QString::number(m.reprojAltPx, 'f', 4));
        }
        if (!writeText("02_逐帧指标.csv", csv, true)) return false;
    }

    // 03 轴与坐标系
    {
        QString csv = "项目,数值,单位/说明\n";
        auto row = [&](const QString& k, const QString& v, const QString& u) {
            csv += QString("\"%1\",\"%2\",\"%3\"\n").arg(k, v, u);
        };
        row("轴方向(转台基准系)", fmtVec3d(d.axisDirTurntable), "单位向量; S5 视角旋转使用的坐标系");
        row("轴点(转台基准系)", fmtVec3d(d.axisPointTurntable, 3), "mm");
        row("轴方向(相机系)", fmtVec3d(d.axisDirCam), "= R_base * 轴(转台系)");
        row("轴点(相机系)", fmtVec3d(d.axisPointCam, 3), "mm");
        row("R_base", fmtMatrix(m_R_base), "棋盘0系 → 相机系");
        row("T_base", fmtMatrix(m_T_base), "mm");
        row("R_cam2turntable", fmtMatrix(m_R_base.empty() ? cv::Mat() : cv::Mat(m_R_base.t())), "= R_base^T; S4 使用");
        row("R(立体)", fmtMatrix(m_R), "右相机 ← 左相机");
        row("T(立体)", fmtMatrix(m_T), "mm");
        row("圆拟合半径 左", QString::number(d.radiusLeftMm, 'f', 4), "mm");
        row("圆拟合半径 右", QString::number(d.radiusRightMm, 'f', 4), "mm");
        row("共圆残差 左", QString::number(d.residLeftMm, 'f', 4), "mm");
        row("共圆残差 右", QString::number(d.residRightMm, 'f', 4), "mm");
        row("左右光心轴向高差", QString::number(d.axialGapMm, 'f', 4), "mm; 应接近 0");
        row("BA 重投影误差", QString::number(d.baErrorPx, 'f', 4), "px");
        row("圆拟合轴 vs BA 轴夹角", QString::number(d.axisDiffDeg, 'f', 4), "deg");
        row("立体外参方向修正", d.stereoFlipped ? "已触发" : "未触发", "触发=Tab2外参方向约定异常");
        row("实测步长均值", QString::number(d.stepMeanDeg, 'f', 4), "deg");
        row("实测步长标准差", QString::number(d.stepStdDeg, 'f', 4), "deg");
        row("累计转角", QString::number(d.angleSpanDeg, 'f', 4), "deg");
        row("Tab5 设定步长", d.configuredStepDeg > 0 ? QString::number(d.configuredStepDeg, 'f', 4) : "未设置", "deg");
        row("采用方案", d.source, "圆拟合 / BA");
        row("轴估计方式设定", d.methodSetting.isEmpty() ? "—" : d.methodSetting, "自动 / 强制圆拟合 / 强制BA");
        row("BA 误差上限", d.baErrorLimitPx >= 0 ? QString::number(d.baErrorLimitPx, 'f', 2) : "—", "px; 自动模式下超过即回退圆拟合");
        if (!writeText("03_轴与坐标系.csv", csv, true)) return false;
    }

    // 04 标定参数
    {
        QString csv = "项目,数值\n";
        auto row = [&](const QString& k, const QString& v) { csv += QString("\"%1\",\"%2\"\n").arg(k, v); };
        row("左相机内参 K_L", fmtMatrix(m_CameraMatrixL, 4));
        row("左相机畸变 D_L", fmtMatrix(m_DistCoeffsL, 6));
        row("右相机内参 K_R", fmtMatrix(m_CameraMatrixR, 4));
        row("右相机畸变 D_R", fmtMatrix(m_DistCoeffsR, 6));
        row("立体 R", fmtMatrix(m_R, 6));
        row("立体 T (mm)", fmtMatrix(m_T, 4));
        row("立体标定 RMS (px)", QString::number(m_StereoRms, 'f', 4));
        row("是否已立体校正", m_IsRectified ? "是" : "否");
        row("光平面方程 (a,b,c,d)", QString("[%1, %2, %3, %4]")
                .arg(m_LaserPlaneEquation[0], 0, 'f', 6).arg(m_LaserPlaneEquation[1], 0, 'f', 6)
                .arg(m_LaserPlaneEquation[2], 0, 'f', 6).arg(m_LaserPlaneEquation[3], 0, 'f', 6));
        row("棋盘格内角点", QString("%1 x %2").arg(m_boardSize.width).arg(m_boardSize.height));
        row("棋盘格方格 (mm)", QString::number(m_squareSize, 'f', 3));
        row("旋转序列对数", QString::number(qMin(m_rotSeqLeftPaths.size(), m_rotSeqRightPaths.size())));
        row("重建序列对数", QString::number(qMin(m_reconLeftPaths.size(), m_reconRightPaths.size())));
        row("旋转序列目录", m_rotSeqLeftPaths.isEmpty() ? QString("(未加载)") : QFileInfo(m_rotSeqLeftPaths[0]).absolutePath());
        row("重建序列目录", m_reconLeftPaths.isEmpty() ? QString("(未加载)") : QFileInfo(m_reconLeftPaths[0]).absolutePath());
        if (!writeText("04_标定参数.csv", csv, true)) return false;
    }

    // 05 重建参数 (与「重建调试包」共用同一份生成逻辑, 免得两处各写一遍后对不上)
    if (!writeText("05_重建参数.csv", buildReconParamsCsv(), true)) return false;

    // 06/07/08 日志
    if (!writeText("06_Tab6调试输出.txt",
                   txtDebugLog ? txtDebugLog->toPlainText() : QString())) return false;
    if (!writeText("07_Tab5重建日志.txt",
                   txtReconLog ? txtReconLog->toPlainText() : QString())) return false;
    if (!writeText("08_Tab4轴标定日志.txt",
                   txtAxisCalibResult ? txtAxisCalibResult->toPlainText() : QString())) return false;

    // 09 检查项
    {
        QString csv = "状态,检查项,实测,期望,建议\n";
        for (size_t i = 0; i < d.checks.size(); ++i) {
            const Calib::AxisDiagCheck& c = d.checks[i];
            csv += QString("\"%1\",\"%2\",\"%3\",\"%4\",\"%5\"\n")
                       .arg(c.ok ? "通过" : "未通过").arg(c.name).arg(c.value)
                       .arg(c.expect).arg(c.advice);
        }
        if (!writeText("09_检查项.csv", csv, true)) return false;
    }

    // 10 圆拟合观测点 (Tab6「圆拟合曲线」画的就是这份数据)
    {
        QString csv = "frameIdx,cam,x_mm,y_mm,resid_mm\n";
        for (const Calib::AxisCircleObs& o : d.circleFit.obs)
            csv += QString("%1,%2,%3,%4,%5\n").arg(o.frameIdx).arg(o.cam)
                       .arg(o.x, 0, 'f', 4).arg(o.y, 0, 'f', 4).arg(o.residMm, 0, 'f', 4);
        if (!writeText("10_圆拟合观测.csv", csv, true)) return false;
    }

    // 11 BA 逐迭代重投影 (Tab6「BA迭代曲线」画的就是这份数据)
    {
        QString csv = "iter,rms_px,accepted\n";
        for (size_t i = 0; i < d.baIterRmsPx.size(); ++i) {
            const bool acc = (i < d.baIterAccepted.size()) && d.baIterAccepted[i];
            csv += QString("%1,%2,%3\n").arg(static_cast<int>(i))
                       .arg(d.baIterRmsPx[i], 0, 'f', 5).arg(acc ? 1 : 0);
        }
        if (!writeText("11_BA迭代.csv", csv, true)) return false;
    }

    // frames/ 异常帧图像
    {
        QStringList toCopy;
        for (size_t i = 0; i < d.frames.size(); ++i) {
            const Calib::AxisFrameMetrics& m = d.frames[i];
            if (m.flag != 0 && m.frameIdx >= 0 && m.frameIdx < m_rotSeqLeftPaths.size())
                toCopy << m_rotSeqLeftPaths[m.frameIdx];
            if (toCopy.size() >= 20) break;
        }
        if (!toCopy.isEmpty()) {
            QDir().mkpath(QDir(dir).filePath("frames"));
            for (const QString& src : toCopy) {
                QFileInfo fi(src);
                if (!fi.exists()) continue;
                const QString dst = QDir(dir).filePath("frames/" + fi.fileName());
                if (QFile::exists(dst)) continue;
                QFile::copy(src, dst);
            }
        }
    }
    return true;
}

// ==================== [由 Tab4/Tab5 迁移] onPreviewSingleFrameClicked ====================
void MainWindow::onPreviewSingleFrameClicked()
{
    appendDebugLog("====== [单帧深度调试] 开始执行 ======");
    if (m_reconLeftPaths.isEmpty() || m_reconRightPaths.isEmpty()) {
        QMessageBox::warning(this, "预览失败",
            "尚未导入旋转图像序列。\n请先到「三维重建」页执行 [1. 导入旋转图像序列]。");
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
    // 【两套底图, 不要混用】S1 与真实重建路径一样, 在**原始畸变图**上提取光条;
    // 只有 S2/S3 才需要校正图 (极线按行匹配、三角化用 P1/P2_rectified)。
    // 这里曾经是"把 imgL/imgR 整体 remap 掉, 后面全用它们", 有两个后果:
    //   1) ROI (m_roiLeft/m_roiRight) 是按原始图框选的, 套到校正图上位置就偏了 —— 裁错区域;
    //   2) 更要紧: 弹窗里显示的 S1 点不是真实流水线看到的那批点, 拿它去查"光条/点数"问题会被带偏。
    //      (reconstructionworker.cpp 里也写了同一条约束, 那里是反过来的 —— 不能 remap)
    // 所以保持 imgL/imgR 为原图, 另存一对校正图 imgL_rect/imgR_rect 专供 S2/S3 显示。
    //
    // 【P0 踩过的坑】校正图必须是**全新的空 Mat**, 绝不能先 `cv::Mat imgL_rect = imgL;` 再 remap。
    // remap 内部只做 dst.create(尺寸, 类型): 尺寸类型一致时它**不会重新分配**, 而是直接往
    // dst 那块内存里写。此时 dst 与 src 是同一块内存 —— 于是原图 imgL 被校正图覆盖掉,
    // 后面 S1 拿到的"原图"其实已经是校正图, S2 再 undistort 一次就成了二次校正, 匹配点整体
    // 偏出光条(实测偏 ~90px)。
    // 之所以有时候看不出问题: 若映射表是按 1257x714 建的而图像是 1280x720, 尺寸不同会触发
    // 重新分配, 恰好掩盖了这个错误 —— 换个尺寸就发作, 属于最难查的一类。
    // 正确写法: 拿空的 dst 让 remap 自己分配。
    const bool rectOk = (m_IsRectified && !m_MapL1.empty());
    cv::Mat imgL_rect, imgR_rect;          // 留着空, 不要赋值成 imgL/imgR
    if (rectOk) {
        cv::remap(imgL, imgL_rect, m_MapL1, m_MapL2, cv::INTER_LINEAR);
        cv::remap(imgR, imgR_rect, m_MapR1, m_MapR2, cv::INTER_LINEAR);
    } else {
        imgL_rect = imgL; imgR_rect = imgR;   // 未校正: 原图即校正图 (引用计数, 不拷数据)
    }

    applyCalibAndParamsToBuilder();

    // ==================== 创建调试弹窗 (精简版：S1提取 + S3重投影) ====================
    // 【P0】这里曾经写成 `static QWidget* s_debugDlg` + `close(); delete s_debugDlg;`, 两个坑:
    //
    //  坑一 · 悬垂指针: 窗口带 WA_DeleteOnClose, 用户点 × 关掉时 Qt 已经把它销毁了,
    //        而 s_debugDlg 仍指着那块内存 —— 再点一次按钮就是 s_debugDlg->close() 访问已释放对象。
    //        改用 QPointer: 对象被销毁时它自动变 nullptr。
    //
    //  坑二 · 自杀式重建(闪退的直接原因): 「上一帧 / 下一帧」按钮是这个窗口的子控件,
    //        它的槽**同步**回调本函数, 而本函数第一件事就是把上一个窗口 close+delete ——
    //        删掉的正是"正在执行那个槽的按钮"所在的整棵控件树。槽返回时 Qt 还要继续
    //        遍历那个按钮的信号, 于是 use-after-free。
    //        两处一起改: 这里只 close (交给 WA_DeleteOnClose 走 deleteLater, 不在当前调用栈里析构),
    //        导航按钮那边改成 QTimer::singleShot(0, ...) 把重建推迟到槽返回之后。
    static QPointer<QWidget> s_debugDlg;
    if (s_debugDlg) s_debugDlg->close();
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
    // S1 底图 = 原图; S2/S3 底图 = 校正图 (它们的点活在矫正系里, 画在原图上对不齐)
    cv::Mat bgL = getBlackBgRedLaser(imgL); cv::Mat bgR = getBlackBgRedLaser(imgR);
    // 未校正时 imgL_rect 就是 imgL, 直接复用 (cv::Mat 赋值只加引用计数, 不拷数据)
    cv::Mat bgLr = rectOk ? getBlackBgRedLaser(imgL_rect) : bgL;
    cv::Mat bgRr = rectOk ? getBlackBgRedLaser(imgR_rect) : bgR;
    cv::Scalar pointColor(0, 255, 255);

    // ==================== S1 调试 ====================
    // 原图 + 原图 ROI —— 与 processSingleView 入口完全一致, 这里出来的才是流水线的那批点。
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
    // 进 S2 之前必须把点从原图系转到校正系 —— 与 processSingleView 里那一步是同一步。
    // 不转的话: 极线匹配是靠"行号相近"配对的, 畸变图的行号对不上, 匹配会整体乱掉;
    // 后面的三角化也会拿畸变坐标去乘 P1/P2_rectified。
    if (rectOk && !m_R1.empty() && !m_P1.empty()
        && !m_CameraMatrixL.empty() && !m_DistCoeffsL.empty()) {
        std::vector<cv::Point2f> pl_rect, pr_rect;
        cv::undistortPoints(pts_left,  pl_rect, m_CameraMatrixL, m_DistCoeffsL, m_R1, m_P1);
        cv::undistortPoints(pts_right, pr_rect, m_CameraMatrixR, m_DistCoeffsR, m_R2, m_P2);
        pts_left.swap(pl_rect); pts_right.swap(pr_rect);
    }
    std::vector<cv::Point2f> match_l, match_r;
    m_builder->epipolarConstraintMatch(pts_left, pts_right, match_l, match_r);
    cv::Mat drawS2L = bgLr.clone(); cv::Mat drawS2R = bgRr.clone();
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
    cv::Mat drawS3L = bgLr.clone(); cv::Mat drawS3R = bgRr.clone();
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
    // (原实现此处同步刷新 Tab5 的预览标签; 迁移到 Tab6 后由本页弹窗承担显示)

    // 导航按钮
    // 【必须推迟, 不能直接调】onPreviewSingleFrameClicked 的第一步就是把当前这个窗口关掉,
    // 而本 lambda 正是运行在那个窗口的子按钮上 —— 同步调用 = 删掉自己脚下的东西。
    // singleShot(0) 让重建在本槽返回、事件循环重新转起来之后才发生 (见函数开头 坑二 的说明)。
    QObject::connect(btnPrev, &QPushButton::clicked, [this, idx]() {
        if (idx > 0) {
            spinPreviewIndex->setValue(idx - 1);
            QTimer::singleShot(0, this, [this]() { onPreviewSingleFrameClicked(); });
        }
    });
    QObject::connect(btnNext, &QPushButton::clicked, [this, idx, maxIdx]() {
        if (idx < maxIdx) {
            spinPreviewIndex->setValue(idx + 1);
            QTimer::singleShot(0, this, [this]() { onPreviewSingleFrameClicked(); });
        }
    });

    appendDebugLog(QString("调试 #%1: S1左%2右%3 → S2%4对 → S3%5点 | 重投影左%6右%7 px")
        .arg(idx).arg(pts_left.size()).arg(pts_right.size()).arg(match_l.size()).arg(s3_pts)
        .arg(s3_err_L > 0 ? QString::number(s3_err_L, 'f', 2) : "-")
        .arg(s3_err_R > 0 ? QString::number(s3_err_R, 'f', 2) : "-"));
}

// ==================== [由 Tab4/Tab5 迁移] onShowPointCloudWindowClicked ====================
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

// ==================== [由 Tab4/Tab5 迁移] onOpenStandaloneViewer ====================
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

// ============================================================================
// 过程曲线: 3D圆拟合 (观测点 + 拟合圆 + 残差)
//
// 为什么值得单独开窗看: 汇总指标只给"残差 RMS = 0.42mm"这样的一个数, 而真正要判断的是
// 光心**是否均匀分布在圆附近** —— 全部帧都差 0.4mm, 和"绝大多数帧贴合、个别帧飞出去",
// 前者是系统性误差(可能是转台本身或标定板), 后者是个别帧的角点检测坏了, 处理方式完全不同。
// ============================================================================
void MainWindow::onShowCircleFitCurve()
{
    const Calib::AxisCircleFit& cf = m_axisDiag.circleFit;
    if (cf.obs.empty()) {
        QMessageBox::information(this, "暂无数据",
            "还没有圆拟合观测数据。\n请先点击「运行转台标定诊断」——观测点只在诊断过程中产生。");
        return;
    }

    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle(QString("3D圆拟合 — 观测点与残差   %1").arg(m_axisDiagTime));
    dlg->resize(920, 780);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    QVBoxLayout *lay = new QVBoxLayout(dlg);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(6);

    int cntL = 0, cntR = 0;
    for (const Calib::AxisCircleObs& o : cf.obs) { if (o.cam == 0) ++cntL; else ++cntR; }

    QLabel *lbl = new QLabel(QString(
        "光心 <b>%1</b> 个 (左 %2 / 右 %3) &nbsp;·&nbsp; 半径 左 %4 / 右 %5 mm "
        "&nbsp;·&nbsp; 残差RMS 左 %6 / 右 %7 mm &nbsp;·&nbsp; 左右圆心偏差 %8 mm")
        .arg(static_cast<int>(cf.obs.size()))
        .arg(cntL).arg(cntR)
        .arg(cf.rLmm, 0, 'f', 2).arg(cf.rRmm, 0, 'f', 2)
        .arg(cf.rmsLmm, 0, 'f', 3).arg(cf.rmsRmm, 0, 'f', 3)
        .arg(cf.centerGapMm, 0, 'f', 3));
    lbl->setTextFormat(Qt::RichText);
    lbl->setWordWrap(true);
    lbl->setStyleSheet(Theme::metricsStyle());
    lay->addWidget(lbl);

    QSplitter *sp = new QSplitter(Qt::Vertical);

    // ---- 上: 拟合平面内的散点 + 拟合圆 ----
    CurvePlotWidget *plotCircle = new CurvePlotWidget();
    plotCircle->setTitle("光心在拟合平面上的投影 (mm)");
    plotCircle->setAxisLabels("平面 X (mm)", "平面 Y (mm)");
    plotCircle->setEqualAspect(true);   // 不设的话圆会被画成椭圆, 半径比例看不出问题

    CurvePlotWidget::Series sL, sR;
    sL.kind = CurvePlotWidget::Kind::Scatter;
    sL.color = QColor(Theme::ACCENT);  sL.name = "左光心";
    sL.mark = CurvePlotWidget::Mark::Circle; sL.markSize = 3.0;
    sR = sL;
    sR.color = QColor(Theme::WARNING); sR.name = "右光心";
    sR.mark = CurvePlotWidget::Mark::Square;

    // ---- 下: 逐点半径偏差 ----
    CurvePlotWidget *plotResid = new CurvePlotWidget();
    plotResid->setTitle("逐点半径偏差 (观测点到本相机拟合圆的距离 − 半径, mm)");
    plotResid->setAxisLabels("帧号", "偏差 (mm)");
    CurvePlotWidget::Series rL, rR;
    rL.kind = CurvePlotWidget::Kind::Line;
    rL.color = QColor(Theme::ACCENT);  rL.name = "左残差";
    rL.mark = CurvePlotWidget::Mark::Circle; rL.markSize = 2.6; rL.width = 1.2;
    rR = rL;
    rR.color = QColor(Theme::WARNING); rR.name = "右残差";
    rR.mark = CurvePlotWidget::Mark::Square;

    for (const Calib::AxisCircleObs& o : cf.obs) {
        if (o.cam == 0) {
            sL.pts << QPointF(o.x, o.y);
            rL.pts << QPointF(o.frameIdx, o.residMm);
        } else {
            sR.pts << QPointF(o.x, o.y);
            rR.pts << QPointF(o.frameIdx, o.residMm);
        }
    }
    plotCircle->addSeries(sL);
    plotCircle->addSeries(sR);

    CurvePlotWidget::Circle cL, cR;
    if (cf.okL) {
        cL.center = QPointF(cf.cLx, cf.cLy); cL.radius = cf.rLmm;
        cL.color = QColor(Theme::ACCENT); cL.name = "左拟合圆";
        plotCircle->addCircle(cL);
    }
    if (cf.okR) {
        cR.center = QPointF(cf.cRx, cf.cRy); cR.radius = cf.rRmm;
        cR.color = QColor(Theme::WARNING); cR.name = "右拟合圆";
        plotCircle->addCircle(cR);
    }
    // 某一路拟合失败时不画那条圆即可, 散点仍然照画 —— 恰恰是这种场景最需要看图
    plotResid->addSeries(rL);
    plotResid->addSeries(rR);
    CurvePlotWidget::RefLine zero; zero.y = 0.0; zero.color = QColor(Theme::TEXT_DIM); zero.name = "零偏差";
    plotResid->addRefLine(zero);

    sp->addWidget(plotCircle);
    sp->addWidget(plotResid);
    sp->setSizes(QList<int>() << 430 << 280);
    lay->addWidget(sp, 1);

    QHBoxLayout *btnRow = new QHBoxLayout();
    btnRow->addStretch();
    QPushButton *btnExp = new QPushButton("导出观测CSV");
    btnExp->setStyleSheet(Theme::miniGhostButton(Theme::PURPLE));
    btnExp->setCursor(Qt::PointingHandCursor);
    connect(btnExp, &QPushButton::clicked, this, [this]() {
        const Calib::AxisCircleFit& fit = m_axisDiag.circleFit;
        const QString path = QFileDialog::getSaveFileName(
            this, "导出圆拟合观测", "circle_fit_obs.csv", "CSV 文件 (*.csv);;所有文件 (*)");
        if (path.isEmpty()) return;
        QFile f(path);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "导出失败", QString("无法写入:\n%1").arg(path));
            return;
        }
        QTextStream ts(&f);
        ts.setCodec("UTF-8");
        ts.setGenerateByteOrderMark(true);
        ts << "frameIdx,cam,x_mm,y_mm,resid_mm\n";
        for (const Calib::AxisCircleObs& o : fit.obs)
            ts << o.frameIdx << "," << o.cam << ","
               << QString::number(o.x, 'f', 4) << "," << QString::number(o.y, 'f', 4) << ","
               << QString::number(o.residMm, 'f', 4) << "\n";
        ts.flush();
        f.close();
        appendDebugLog(QString("[圆拟合] 观测点已导出: %1").arg(path));
    });
    QPushButton *btnClose = new QPushButton("关闭");
    btnClose->setStyleSheet(Theme::ghostButton());
    btnClose->setCursor(Qt::PointingHandCursor);
    connect(btnClose, &QPushButton::clicked, dlg, &QDialog::close);
    btnRow->addWidget(btnExp);
    btnRow->addWidget(btnClose);
    lay->addLayout(btnRow);

    dlg->show();
}

// ============================================================================
// 过程曲线: BA 逐迭代重投影误差
//
// 要回答的问题: BA 报出的那个重投影误差, 是"已经收敛到那里", 还是"还没收敛就被 80 轮截断"?
// 曲线走平 → 收敛; 曲线还在降 → 迭代次数不够; 大量被拒的台阶 → 阻尼策略在挣扎。
// 叠加误差阈值参考线后, 可以直接看出它离"可用"差多少。
// ============================================================================
void MainWindow::onShowBaIterCurve()
{
    const std::vector<double>& hist = m_axisDiag.baIterRmsPx;
    if (hist.empty()) {
        QMessageBox::information(this, "暂无数据",
            "没有 BA 迭代数据。可能原因:\n"
            "· 未运行「运行转台标定诊断」\n"
            "· 转角不可观测 (棋盘格点落在旋转轴附近)，BA 被直接放弃\n"
            "· PnP 初值解算失败");
        return;
    }

    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle(QString("BA迭代 — 重投影误差   %1").arg(m_axisDiagTime));
    dlg->resize(880, 560);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    QVBoxLayout *lay = new QVBoxLayout(dlg);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(6);

    int accepted = 0;
    for (size_t i = 0; i < m_axisDiag.baIterAccepted.size(); ++i)
        if (m_axisDiag.baIterAccepted[i]) ++accepted;

    QLabel *lbl = new QLabel(QString(
        "初值 %1 px &nbsp;→&nbsp; 迭代后 %2 px &nbsp;·&nbsp; 迭代 %3 轮 (接受 %4 / 拒绝 %5) "
        "&nbsp;·&nbsp; 阈值 %6 px &nbsp;·&nbsp; 实际采用: %7")
        .arg(m_axisDiag.baRmsInitialPx, 0, 'f', 3)
        .arg(hist.back(), 0, 'f', 3)
        .arg(static_cast<int>(hist.size()))
        .arg(accepted).arg(static_cast<int>(hist.size()) - accepted)
        .arg(m_axisDiag.baErrorLimitPx, 0, 'f', 2)
        .arg(m_axisDiag.source));
    lbl->setTextFormat(Qt::RichText);
    lbl->setWordWrap(true);
    lbl->setStyleSheet(Theme::metricsStyle());
    lay->addWidget(lbl);

    CurvePlotWidget *plot = new CurvePlotWidget();
    plot->setTitle("BA (LM/Huber) 逐迭代重投影 RMS (px)");
    plot->setAxisLabels("迭代轮次", "重投影 RMS (px)");

    CurvePlotWidget::Series line;
    line.kind = CurvePlotWidget::Kind::Line;
    line.color = QColor(Theme::ACCENT);
    line.name = "RMS";
    line.width = 1.6;
    for (size_t i = 0; i < hist.size(); ++i)
        line.pts << QPointF(static_cast<double>(i), hist[i]);
    plot->addSeries(line);

    // 接受/拒绝分开标注: 连续一串拒绝点 = 那几轮在反复加大阻尼却没走成, 曲线表现为平台段
    CurvePlotWidget::Series okS, rejS;
    okS.kind = CurvePlotWidget::Kind::Scatter;
    okS.color = QColor(Theme::SUCCESS); okS.name = "接受步";
    okS.mark = CurvePlotWidget::Mark::Circle; okS.markSize = 3.0;
    rejS.kind = CurvePlotWidget::Kind::Scatter;
    rejS.color = QColor(Theme::DANGER); rejS.name = "拒绝步";
    rejS.mark = CurvePlotWidget::Mark::Square; rejS.markSize = 3.0;
    for (size_t i = 0; i < hist.size(); ++i) {
        const bool acc = (i < m_axisDiag.baIterAccepted.size()) && m_axisDiag.baIterAccepted[i];
        (acc ? okS : rejS).pts << QPointF(static_cast<double>(i), hist[i]);
    }
    plot->addSeries(rejS);
    plot->addSeries(okS);

    if (m_axisDiag.baErrorLimitPx > 0) {
        CurvePlotWidget::RefLine th;
        th.y = m_axisDiag.baErrorLimitPx;
        th.color = QColor(Theme::DANGER);
        th.name = QString("自动模式阈值 %1 px").arg(m_axisDiag.baErrorLimitPx, 0, 'f', 2);
        plot->addRefLine(th);
    }
    lay->addWidget(plot, 1);

    QHBoxLayout *btnRow = new QHBoxLayout();
    btnRow->addStretch();
    QPushButton *btnClose = new QPushButton("关闭");
    btnClose->setStyleSheet(Theme::ghostButton());
    btnClose->setCursor(Qt::PointingHandCursor);
    connect(btnClose, &QPushButton::clicked, dlg, &QDialog::close);
    btnRow->addWidget(btnClose);
    lay->addLayout(btnRow);

    dlg->show();
}

// ============================================================================
// 重建测试 ①: 快速产出测试 (前 N 帧 S1~S4)
//
// 定位: 完整重建跑一次要几十秒到几分钟, 而调参时想知道的往往只是"点是从哪一步开始丢的"。
// S1~S4 逐视角独立、无跨视角状态, 所以前 N 帧在统计上可以代表整段 (例外: 异常只出现在
// 序列尾段的情况看不到 —— 所以界面上会标明 N / 总帧数, 让人知道这只看了个开头)。
//
// **这里绝不能对图像做 remap**: processSingleView 的设计输入是原始畸变图, 内部自己用
// undistortPoints(K,D,R_rect,P_rect) 转到校正系。Tab6「单帧调试」是先 remap 再交给 builder
// 的另一条路径, 照抄那套会二次校正, 3D 点全错 —— 曲线就成了"看起来很精细的垃圾"。
// ============================================================================
void MainWindow::onRunQuickYieldTest()
{
    if (m_reconLeftPaths.isEmpty() || m_reconRightPaths.isEmpty()) {
        QMessageBox::warning(this, "缺少数据", "请先到「三维重建」页加载旋转图像序列！");
        return;
    }
    const int totalPairs = qMin(m_reconLeftPaths.size(), m_reconRightPaths.size());

    QDialog *dlg = new QDialog(this, Qt::Window);
    dlg->setWindowTitle("快速产出测试 — 前 N 帧 S1~S4");
    dlg->resize(900, 760);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    QVBoxLayout *lay = new QVBoxLayout(dlg);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(6);

    QHBoxLayout *topRow = new QHBoxLayout();
    topRow->setSpacing(6);
    QLabel *lblN = new QLabel("测试帧数 N");
    lblN->setStyleSheet(QString("font-size: 12px; color: %1;").arg(Theme::TEXT_PRIMARY));
    QSpinBox *spinN = new QSpinBox();
    spinN->setRange(1, qMax(1, totalPairs));
    spinN->setValue(qMin(20, totalPairs));
    spinN->setToolTip("S1~S4 逐视角独立, 前 N 帧可代表整段; 数值越大越慢");
    QLabel *lblTotal = new QLabel(QString("/ 共 %1 对").arg(totalPairs));
    lblTotal->setStyleSheet(QString("font-size: 11px; color: %1;").arg(Theme::TEXT_SECONDARY));
    QPushButton *btnRun = new QPushButton("开始测试");
    btnRun->setStyleSheet(Theme::successButton());
    btnRun->setCursor(Qt::PointingHandCursor);
    topRow->addWidget(lblN);
    topRow->addWidget(spinN);
    topRow->addWidget(lblTotal);
    topRow->addSpacing(10);
    topRow->addWidget(btnRun);
    topRow->addStretch();
    lay->addLayout(topRow);

    QProgressBar *bar = new QProgressBar();
    bar->setRange(0, 1);
    bar->setValue(0);
    bar->setFormat("待机");
    lay->addWidget(bar);

    CurvePlotWidget *plotYield = new CurvePlotWidget();
    plotYield->setTitle("逐帧有效点数 (S1→S4 之后)");
    plotYield->setAxisLabels("帧号", "点数");

    CurvePlotWidget *plotHist = new CurvePlotWidget();
    plotHist->setTitle("右图重投影误差分布");
    plotHist->setAxisLabels("误差区间 (px)", "匹配数");
    plotHist->setXTickLabels(QStringList() << "<1" << "1-2" << "2-5" << "5-10"
                                           << "10-20" << "20-50" << "50-100" << "≥100");

    QLabel *lblSum = new QLabel("尚未测试");
    lblSum->setTextFormat(Qt::RichText);
    lblSum->setWordWrap(true);
    lblSum->setStyleSheet(Theme::metricsStyle());

    QSplitter *sp = new QSplitter(Qt::Vertical);
    sp->addWidget(plotYield);
    sp->addWidget(plotHist);
    sp->setSizes(QList<int>() << 240 << 240);
    lay->addWidget(sp, 1);
    lay->addWidget(lblSum);

    // 与 worker 用同一套参数来源: 手搓参数会让"测试结果"和"正式重建"对不上
    connect(btnRun, &QPushButton::clicked, this, [=]() {
        const int n = qMin(spinN->value(), totalPairs);
        if (n <= 0) return;
        btnRun->setEnabled(false);
        spinN->setEnabled(false);
        bar->setRange(0, n);
        bar->setValue(0);

        PointCloudBuilder builder;
        builder.setCalibrationData(buildCalibrationData());
        builder.setReconstructionParams(buildReconstructionParams());

        // 与 worker 的逐视角并行路径保持一致的执行环境 (OpenCV 压到单线程),
        // 否则线程数差异会让同一份数据给出不同的匹配结果, 测试就失去可比性
        const int prevCvThreads = cv::getNumThreads();
        cv::setNumThreads(1);
        const double step = spinAngleStep ? spinAngleStep->value() : 1.8;
        int readFail = 0;
        for (int i = 0; i < n; ++i) {
            const cv::Mat imgL = cv::imread(m_reconLeftPaths[i].toStdString());
            const cv::Mat imgR = cv::imread(m_reconRightPaths[i].toStdString());
            if (imgL.empty() || imgR.empty()) ++readFail;
            else builder.processSingleView(imgL, imgR, i * step, i);
            bar->setValue(i + 1);
            bar->setFormat(QString("已处理 %1 / %2").arg(i + 1).arg(n));
            QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        }
        cv::setNumThreads(prevCvThreads);

        // ---- 曲线 ----
        plotYield->clear();
        CurvePlotWidget::Series sy;
        sy.kind = CurvePlotWidget::Kind::Line;
        sy.color = QColor(Theme::ACCENT);
        sy.name = "有效点数";
        sy.mark = CurvePlotWidget::Mark::Circle;
        sy.markSize = 3.0;
        int sumMatch = 0, sumPts = 0, zeroFrames = 0;
        for (const ViewRecord& r : builder.viewRecords()) {
            sy.pts << QPointF(r.frameIdx, r.s3Points);
            sumMatch += r.s2Match;
            sumPts   += r.s3Points;
            if (r.s3Points == 0) ++zeroFrames;
        }
        plotYield->addSeries(sy);

        plotHist->clear();
        const ViewOutcomeStats& st = builder.viewStats();
        CurvePlotWidget::Series sh;
        sh.kind = CurvePlotWidget::Kind::Bar;
        sh.color = QColor(Theme::ACCENT);
        sh.name = "匹配数";
        for (int b = 0; b < 8; ++b) sh.pts << QPointF(b, st.reprojHist[b]);
        plotHist->addSeries(sh);

        // ---- 汇总 (口径与 worker 的 [逐视角汇总] 完全一致) ----
        const double yieldPct = sumMatch > 0 ? 100.0 * sumPts / sumMatch : 0.0;
        lblSum->setText(QString(
            "处理 <b>%1</b> 帧 (总 %2) &nbsp;·&nbsp; 读取失败 %3 &nbsp;·&nbsp; 零产出帧 <b>%4</b><br>"
            "S1提取空 %5 &nbsp;|&nbsp; S2匹配零 %6 &nbsp;|&nbsp; S3三角化空 %7<br>"
            "S3剔除: 负深度 %8 / 深度越界 %9 / 射线平行 %10 / <b>重投影 %11</b><br>"
            "匹配 %12 对 → 有效点 <b>%13</b> (产出率 %14%) &nbsp;·&nbsp; 深度范围 [%15, %16] mm")
            .arg(n).arg(totalPairs).arg(readFail).arg(zeroFrames)
            .arg(st.extractEmpty).arg(st.matchEmpty).arg(st.triEmpty)
            .arg(st.negDepth).arg(st.tooFar).arg(st.degenerate).arg(st.reprojReject)
            .arg(sumMatch).arg(sumPts).arg(yieldPct, 0, 'f', 1)
            .arg(st.depthMaxMm > st.depthMinMm ? QString::number(st.depthMinMm, 'f', 1) : QString("—"))
            .arg(st.depthMaxMm > st.depthMinMm ? QString::number(st.depthMaxMm, 'f', 1) : QString("—")));

        appendDebugLog(QString("[重建测试/快速] 前 %1 帧: 有效点 %2 (匹配 %3, 产出率 %4%), 零产出帧 %5, 重投影剔除 %6")
            .arg(n).arg(sumPts).arg(sumMatch).arg(yieldPct, 0, 'f', 1).arg(zeroFrames).arg(st.reprojReject));

        btnRun->setEnabled(true);
        spinN->setEnabled(true);
        bar->setFormat("完成");
    });

    QHBoxLayout *btnRow = new QHBoxLayout();
    btnRow->addStretch();
    QPushButton *btnClose = new QPushButton("关闭");
    btnClose->setStyleSheet(Theme::ghostButton());
    btnClose->setCursor(Qt::PointingHandCursor);
    connect(btnClose, &QPushButton::clicked, dlg, &QDialog::close);
    btnRow->addWidget(btnClose);
    lay->addLayout(btnRow);

    dlg->show();
}

// ============================================================================
// 重建测试 ②: 完整重建 (S1~S6) 冒烟测试
//
// 复用 Tab5 常驻的同一个 worker 与同一份 buildReconJob(): 这是"测试"能代表"正式跑"的前提。
// 走 m_reconSmoke 标志而不是再挂一个 finished 连接 —— 后者依赖"连接顺序在 Tab5 之后"这种
// 隐性约定, 一旦有人调整 init 顺序或改用 DirectConnection 就会静默失效。
// ============================================================================
void MainWindow::onRunFullReconTest()
{
    if (m_reconRunning) {
        QMessageBox::information(this, "正在重建",
            "已有重建任务在运行。\n可在「三维重建」页取消后再发起测试。");
        return;
    }
    ReconstructionWorker::JobInput job;
    QString err;
    if (!buildReconJob(job, &err)) {
        QMessageBox::warning(this, "无法测试", err);
        return;
    }

    m_reconSmoke = true;
    markReconRunning();   // 与 Tab5 共用运行态: 避免 Tab5 那边还能再排一个 job
    if (btnFullRecon) btnFullRecon->setEnabled(false);   // 结束后由 updateDebugStatus 恢复

    appendDebugLog(QString("====== [重建测试] 开始完整 S1~S6 重建: %1 对, 转角 %2° ======")
        .arg(job.leftPaths.size()).arg(job.angleStepDeg, 0, 'f', 3));
    appendDebugLog(QString(">>> 预估总转角 %1° (转满一圈约需 360°)")
        .arg(job.angleStepDeg * job.leftPaths.size(), 0, 'f', 1));

    m_lastReconJob = job;   // 调试包要按同一套参数重跑样本帧, 见 mainwindow.h 的说明
    m_hasReconJob = true;
    QMetaObject::invokeMethod(m_reconWorker, "run", Qt::QueuedConnection,
                              Q_ARG(ReconstructionWorker::JobInput, job));
}

// 完整重建测试的通过/不通过判定 (在 onReconstructionFinished 收尾时调用)
QString MainWindow::buildReconTestReport() const
{
    if (!m_reconWorker) return "[重建测试] worker 不存在";

    const std::vector<ViewRecord>& recs = m_reconWorker->viewRecords();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m_reconWorker->resultCloud();
    const pcl::PolygonMesh mesh = m_reconWorker->resultMesh();

    const int n = static_cast<int>(recs.size());
    int sumMatch = 0, sumPts = 0, zeroFrames = 0;
    for (const ViewRecord& r : recs) {
        sumMatch += r.s2Match;
        sumPts   += r.s3Points;
        if (r.s3Points == 0) ++zeroFrames;
    }
    const int pts   = cloud ? static_cast<int>(cloud->size()) : 0;
    const int faces = static_cast<int>(mesh.polygons.size());
    const double yieldPct = sumMatch > 0 ? 100.0 * sumPts / sumMatch : 0.0;

    QStringList fail;
    if (n == 0)                    fail << "没有任何视角记录 (序列为空或读图全部失败)";
    if (sumPts == 0)               fail << "S1~S4 阶段一个点都没产出";
    else if (zeroFrames * 2 > n)   fail << QString("超过一半视角零产出 (%1/%2)").arg(zeroFrames).arg(n);
    if (sumMatch > 0 && yieldPct < 20.0)
        fail << QString("匹配→有效点产出率仅 %1% (大量匹配被剔除)").arg(yieldPct, 0, 'f', 1);
    if (pts == 0)                  fail << "S6 之后最终点云为空";
    else if (n > 0 && pts < sumPts / 2)
        fail << QString("最终点数 %1 远小于各视角之和 %2 (配准/滤波丢了大量点)").arg(pts).arg(sumPts);
    if (faces == 0)                fail << "网格面数为 0";

    // 取消与"跑挂了"在结果上长得一样(点云为空), 但含义完全不同 —— 用户主动取消不该被报成失败
    if (m_reconWorker->wasCancelled()) {
        return QString("====== [重建测试] 已取消 ======\n"
                       "已处理 %1 个视角, 未走到 S5/S6, 不构成一次有效测试。\n").arg(n);
    }

    QString s;
    s += QString("====== [重建测试] 完整重建结果: %1 ======\n")
             .arg(fail.isEmpty() ? QString("✅ 通过") : QString("❌ 未通过 (%1 项)").arg(fail.size()));
    s += QString("视角 %1 | 零产出帧 %2 | 匹配 %3 对 → 视角点数合计 %4 (产出率 %5%)\n")
             .arg(n).arg(zeroFrames).arg(sumMatch).arg(sumPts).arg(yieldPct, 0, 'f', 1);
    s += QString("最终点云 %1 点 | 网格 %2 面\n").arg(pts).arg(faces);
    for (const QString& f : fail) s += QString("  ✗ %1\n").arg(f);
    if (fail.isEmpty())
        s += "  ✓ 各阶段均有产出, 最终点云与网格非空\n";
    return s;
}
