#pragma once
#include <QString>
#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>

namespace Theme {

// ══════════════════════════════════════════════════════════════════════════
//  工业软件风格 (浅色) —— 设计要点
//   · 冷灰钢铁底色 + 白色数据面板 + 硬朗直角(小圆角) + 细分隔线
//   · 数据/参数一律等宽字体, 便于逐位对齐核对
//   · 分组用"卡片 + 标题条", 层级靠灰阶与线框而不是靠颜色
//   · 颜色只用于语义(通过/警告/危险/主操作), 不用于装饰
// ══════════════════════════════════════════════════════════════════════════
inline constexpr auto BG_MAIN        = "#e9ecef";   // 窗口底 (冷灰)
inline constexpr auto BG_CARD        = "#ffffff";   // 数据面板
inline constexpr auto BG_INPUT       = "#ffffff";
inline constexpr auto BG_HOVER       = "#dde2e7";
inline constexpr auto BG_STRIP       = "#f4f6f8";   // 表头/工具条底
inline constexpr auto BG_SEL         = "#dce9f6";   // 选中行
inline constexpr auto BORDER         = "#aeb6bf";   // 主分隔线 (偏硬)
inline constexpr auto BORDER_LIGHT   = "#d5dae0";   // 次级分隔线
inline constexpr auto BORDER_FOCUS   = "#1f6fb2";
inline constexpr auto TEXT_PRIMARY   = "#1f2933";
inline constexpr auto TEXT_SECONDARY = "#5b6672";
inline constexpr auto TEXT_DIM       = "#8b95a1";
inline constexpr auto ACCENT         = "#1f6fb2";   // 工业蓝 (主操作)
inline constexpr auto SUCCESS        = "#1e8e4e";
inline constexpr auto WARNING        = "#c97a12";
inline constexpr auto DANGER         = "#c0392b";
inline constexpr auto PURPLE         = "#6f42c1";

// ── Terminal colors (keep dark for log/data display) ──
inline constexpr auto TERM_BG        = "#15181c";
inline constexpr auto TERM_TEXT      = "#c7ccd1";
inline constexpr auto TERM_BORDER    = "#3a4048";

// ── Fonts ──
inline const char* const MONO = "Consolas";
inline const char* const UI   = "Segoe UI";

// ── Dimensions (工业软件偏紧凑) ──
inline constexpr int BTN_MIN_W      = 104;
inline constexpr int BTN_H          = 26;
inline constexpr int CONTROL_PANEL_W = 280;
inline constexpr int VIDEO_MIN_W    = 320;
inline constexpr int VIDEO_MIN_H    = 240;
inline constexpr int BORDER_RADIUS  = 3;
inline constexpr int MARGIN         = 6;
inline constexpr int SPACING        = 5;

// ── Reusable Style Sheets ──

/** Dark terminal-style text edit for log/data output */
inline QString terminalStyle() {
    return QString(
        "QTextEdit {"
        "  background-color: %1; border: 1px solid %2; border-radius: %6px;"
        "  color: %3; font-family: %4; font-size: 10px;"
        "}"
    ).arg(TERM_BG, TERM_BORDER, TERM_TEXT, MONO).arg(BORDER_RADIUS);
}

/** Image label with border */
inline QString imageLabelStyle(int minW = 120, int minH = 90) {
    return QString(
        "QLabel {"
        "  border: 2px solid %1; background-color: %2;"
        "  border-radius: %4px; color: %3;"
        "  min-width: %5px; min-height: %6px;"
        "}"
    ).arg(BORDER, BG_INPUT, TEXT_DIM).arg(BORDER_RADIUS).arg(minW).arg(minH);
}

/** Semantic action button */
inline QString actionButton(const QString& bgColor) {
    return QString(
        "QPushButton {"
        "  background-color: %1; color: white; font-weight: bold;"
        "  border: none; border-radius: %2px; padding: 6px 14px;"
        "  min-height: %3px;"
        "}"
        "QPushButton:hover { opacity: 0.85; }"
        "QPushButton:disabled { background-color: #bbb; color: #888; }"
    ).arg(bgColor).arg(BORDER_RADIUS).arg(BTN_H);
}

inline QString successButton()   { return actionButton(SUCCESS); }
inline QString primaryButton()   { return actionButton(ACCENT); }
inline QString warningButton()   { return actionButton(WARNING); }
inline QString dangerButton()    { return actionButton(DANGER); }
inline QString purpleButton()    { return actionButton(PURPLE); }

/** Bold text button (no background, native look) */
inline QString boldButton() {
    return "QPushButton { font-weight: bold; }";
}

/** Video widget border */
inline QString videoBorderStyle() {
    return QString(
        "background-color: #000; border: 2px solid %1; border-radius: %2px;"
    ).arg(BORDER).arg(BORDER_RADIUS);
}

/** Title label for video sections */
inline QString sectionTitleStyle() {
    return QString(
        "font-weight: bold; font-size: 14px; color: %1;"
    ).arg(TEXT_PRIMARY);
}

// ==========================================================================
// UI 层次化辅助样式 (Tab6 调试与诊断页优化时引入, 其余页面可复用)
// 目的: 一页里只保留少量"实心"主操作按钮, 次要/回看操作用描边按钮,
//       配合统一的 小标题 / 提示框 / 指标条, 让信息层级一眼可辨。
// ==========================================================================

/** Card-style group box: 白色卡片 + 圆角 + 顶部标题 (作为一栏的容器) */
inline QString cardGroupStyle() {
    return QString(
        "QGroupBox {"
        "  background-color: %1; border: 1px solid %2; border-radius: %4px;"
        "  margin-top: 16px;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin; subcontrol-position: top left;"
        "  left: 8px; padding: 0 6px;"
        "  color: %3; font-weight: bold; font-size: 12px;"
        "}"
    ).arg(BG_CARD, BORDER, TEXT_PRIMARY).arg(BORDER_RADIUS);
}

/** 小节标题: 左侧强调色竖条 */
inline QString subTitleStyle() {
    return QString(
        "font-weight: bold; font-size: 12px; color: %1;"
        " border-left: 3px solid %2; padding-left: 6px;"
    ).arg(TEXT_PRIMARY, ACCENT);
}

/** 说明性文字块: 浅底 + 细边, 用于解释"这一栏回答什么问题" */
inline QString hintBoxStyle() {
    return QString(
        "QLabel { background-color: #f7f9fb; border: 1px solid #e4e9ef;"
        "  border-radius: %1px; padding: 6px 8px; color: %2; font-size: 11px; }"
    ).arg(BORDER_RADIUS).arg(TEXT_SECONDARY);
}

/** 指标条: 等宽字体展示关键数值 (标定质量 / 结果规模) */
inline QString metricsStyle() {
    return QString(
        "QLabel { background-color: #f2f6fa; border: 1px solid #dde5ee;"
        "  border-radius: %1px; padding: 6px 8px; color: %2;"
        "  font-family: %3; font-size: 11px; }"
    ).arg(BORDER_RADIUS).arg(TEXT_PRIMARY).arg(MONO);
}

// 注意: 同一个样式里出现两次的颜色必须用不同占位符 (%1 与 %5)。
//       QString::arg() 对"重复出现的同一编号占位符"只保证替换最低编号的那一处
//       (Qt4/5 文档表述)，写成同一个编号会导致颜色漏替换。
/** 描边(次要)动作按钮: 保留语义色的同时避免与主操作抢视线 */
inline QString ghostButton(const QString& color = ACCENT) {
    return QString(
        "QPushButton {"
        "  background-color: transparent; color: %1;"
        "  border: 1px solid %5; border-radius: %2px;"
        "  padding: 5px 12px; min-height: 26px; font-weight: bold;"
        "}"
        "QPushButton:hover { background-color: %3; }"
        "QPushButton:pressed { background-color: %4; }"
        "QPushButton:disabled { color: #b8bfc7; border-color: #dfe3e8; background-color: transparent; }"
    ).arg(color).arg(BORDER_RADIUS).arg(BG_HOVER).arg(BG_HOVER).arg(color);
}

/** 迷你描边按钮: 用于工具条/回看等低权重入口 */
inline QString miniGhostButton(const QString& color = ACCENT) {
    return QString(
        "QPushButton {"
        "  background-color: transparent; color: %1;"
        "  border: 1px solid %4; border-radius: %2px;"
        "  padding: 2px 8px; font-size: 11px;"
        "}"
        "QPushButton:hover { background-color: %3; }"
        "QPushButton:disabled { color: #b8bfc7; border-color: #dfe3e8; }"
    ).arg(color).arg(BORDER_RADIUS).arg(BG_HOVER).arg(color);
}

// ══════════════════════════════════════════════════════════════════════════
//  全局样式表 (工业软件风格)
//  只对"没有自带样式表"的控件生效；各页已显式设过样式的控件保持原样。
// ══════════════════════════════════════════════════════════════════════════
inline QString globalStyleSheet()
{
    // 说明: QSS 里同一颜色会在多处出现，而 QString::arg() 对"重复出现的同一编号占位符"
    //       的处理在不同 Qt 版本/文档表述并不一致 (Qt4/5 文档为"最低编号的那一处")，
    //       为避免颜色漏替换，这里改用**具名 token + QString::replace()**(替换全部出现)。
    QString s = QString(
        // ---------- 基础 ----------
        "QMainWindow, QDialog { background-color: @BGMN@; }"
        "QWidget { font-family: '@UIF@'; font-size: 12px; color: @TXT1@; }"
        "QToolTip { background-color: #2b3138; color: #f0f2f5; border: 1px solid #000; padding: 3px; }"

        // ---------- 菜单栏 / 工具栏 / 状态栏 ----------
        "QMenuBar { background-color: @STRIP@; border-bottom: 1px solid @BRDR@; padding: 1px 2px; }"
        "QMenuBar::item { padding: 4px 10px; background: transparent; }"
        "QMenuBar::item:selected { background-color: @HOVR@; }"
        "QMenu { background-color: @CARD@; border: 1px solid @BRDR@; padding: 3px; }"
        "QMenu::item { padding: 5px 22px 5px 14px; }"
        "QMenu::item:selected { background-color: @ACNT@; color: #ffffff; }"
        "QMenu::separator { height: 1px; background: @BRLT@; margin: 3px 6px; }"
        "QToolBar { background-color: @STRIP@; border-bottom: 1px solid @BRDR@; spacing: 4px; padding: 3px; }"
        "QToolButton { padding: 4px 10px; border: 1px solid transparent; border-radius: @RAD@px; }"
        "QToolButton:hover { background-color: @HOVR@; border-color: @BRLT@; }"
        "QStatusBar { background-color: @STRIP@; border-top: 1px solid @BRDR@; }"

        // ---------- Tab 页签 (工业软件: 平直页签 + 顶部强调线) ----------
        "QTabWidget::pane { border: 1px solid @BRDR@; background-color: @BGMN@; top: -1px; }"
        "QTabBar { background-color: @STRIP@; }"
        "QTabBar::tab { background-color: @STRIP@; color: @TXT2@; padding: 6px 16px;"
        "  border: 1px solid @BRDR@; border-bottom: none; margin-right: 1px; }"
        "QTabBar::tab:selected { background-color: @CARD@; color: @TXT1@; font-weight: bold;"
        "  border-top: 2px solid @ACNT@; padding-top: 5px; }"
        "QTabBar::tab:hover:!selected { background-color: @HOVR@; }"

        // ---------- 分组框 (卡片 + 标题) ----------
        "QGroupBox { background-color: @CARD@; border: 1px solid @BRDR@; border-radius: @RAD@px;"
        "  margin-top: 14px; padding-top: 2px; }"
        "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left;"
        "  left: 8px; padding: 0 5px; color: @TXT1@; font-weight: bold; }"

        // ---------- 按钮 ----------
        "QPushButton { background-color: @STRIP@; color: @TXT1@; border: 1px solid @BRLT@;"
        "  border-radius: @RAD@px; padding: 3px 12px; min-height: 22px; }"
        "QPushButton:hover { background-color: @HOVR@; border-color: @BRDR@; }"
        "QPushButton:pressed { background-color: @SELB@; }"
        "QPushButton:checked { background-color: @ACNT@; color: #ffffff; border-color: @ACNT@; }"
        "QPushButton:disabled { color: @DIMT@; background-color: @DSBG@; border-color: @BRLT@; }"

        // ---------- 输入控件 ----------
        "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit {"
        "  background-color: @CARD@; border: 1px solid @BRLT@; border-radius: @RAD@px;"
        "  padding: 2px 6px; min-height: 20px; selection-background-color: @ACNT@; }"
        "QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus, QPlainTextEdit:focus {"
        "  border-color: @ACNT@; }"
        "QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {"
        "  background-color: @DSBG@; color: @DIMT@; }"
        "QComboBox::drop-down { width: 18px; border-left: 1px solid @BRLT@; }"
        "QComboBox QAbstractItemView { background-color: @CARD@; border: 1px solid @BRDR@;"
        "  selection-background-color: @ACNT@; selection-color: #ffffff; }"
        "QSpinBox::up-button, QDoubleSpinBox::up-button, QSpinBox::down-button, QDoubleSpinBox::down-button {"
        "  width: 15px; background-color: @STRIP@; border-left: 1px solid @BRLT@; }"

        // ---------- 复选/单选 ----------
        "QCheckBox, QRadioButton { spacing: 6px; }"
        "QCheckBox::indicator, QRadioButton::indicator { width: 14px; height: 14px; }"
        "QCheckBox::indicator:unchecked { border: 1px solid @BRDR@; background: @CARD@; }"
        "QCheckBox::indicator:checked { border: 1px solid @ACNT@; background: @ACNT@; }"
        "QCheckBox:disabled, QRadioButton:disabled { color: @DIMT@; }"

        // ---------- 列表 / 表格 (数据区: 等宽 + 交替行) ----------
        "QListWidget, QTableWidget, QTreeWidget { background-color: @CARD@; border: 1px solid @BRDR@;"
        "  gridline-color: @BRLT@; alternate-background-color: @ALTB@; font-family: '@MONO@'; font-size: 11px; }"
        "QListWidget::item, QTableWidget::item, QTreeWidget::item { padding: 2px 4px; }"
        "QListWidget::item:selected, QTableWidget::item:selected, QTreeWidget::item:selected {"
        "  background-color: @ACNT@; color: #ffffff; }"
        "QHeaderView::section { background-color: @STRIP@; color: @TXT1@; padding: 3px 6px;"
        "  border: none; border-right: 1px solid @BRLT@; border-bottom: 1px solid @BRDR@; font-weight: bold; }"
        "QTableCornerButton::section { background-color: @STRIP@; border: 1px solid @BRLT@; }"

        // ---------- 进度条 ----------
        "QProgressBar { background-color: @DSBG@; border: 1px solid @BRLT@; border-radius: @RAD@px;"
        "  text-align: center; color: @TXT1@; height: 16px; }"
        "QProgressBar::chunk { background-color: @ACNT@; border-radius: @RAD@px; }"

        // ---------- 滚动条 (细窄工业风) ----------
        "QScrollBar:vertical { background: @BGMN@; width: 11px; margin: 0; }"
        "QScrollBar::handle:vertical { background: @SBR@; min-height: 24px; border-radius: 2px; }"
        "QScrollBar::handle:vertical:hover { background: @BRDR@; }"
        "QScrollBar:horizontal { background: @BGMN@; height: 11px; margin: 0; }"
        "QScrollBar::handle:horizontal { background: @SBR@; min-width: 24px; border-radius: 2px; }"
        "QScrollBar::handle:horizontal:hover { background: @BRDR@; }"
        "QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }"
        "QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }"

        // ---------- 分隔条 ----------
        "QSplitter::handle { background-color: @BRLT@; }"
        "QSplitter::handle:hover { background-color: @ACNT@; }"
    );

    // 用具名 token 替换: 参数强制为 QString, 避免 QString/QLatin1String 重载歧义
    auto rep = [&s](const char* token, const QString& value) {
        s.replace(QString::fromLatin1(token), value);
    };
    rep("@BGMN@", BG_MAIN);
    rep("@UIF@",  UI);
    rep("@TXT1@", TEXT_PRIMARY);
    rep("@TXT2@", TEXT_SECONDARY);
    rep("@DIMT@", TEXT_DIM);
    rep("@STRIP@", BG_STRIP);
    rep("@CARD@", BG_CARD);
    rep("@HOVR@", BG_HOVER);
    rep("@SELB@", BG_SEL);
    rep("@ALTB@", "#fbfcfd");
    rep("@DSBG@", "#f0f2f4");
    rep("@BRDR@", BORDER);
    rep("@BRLT@", BORDER_LIGHT);
    rep("@ACNT@", ACCENT);
    rep("@MONO@", MONO);
    rep("@SBR@",  "#9aa4ae");
    rep("@RAD@",  QString::number(BORDER_RADIUS));
    return s;
}

// ══════════════════════════════════════════════════════════════════════════
//  页面页头: [序号] 标题 —— 副标题 (说明该页职责)
// ══════════════════════════════════════════════════════════════════════════
inline QWidget* pageHeader(const QString& stepNo, const QString& title, const QString& subtitle)
{
    QWidget *head = new QWidget();
    head->setObjectName("pageHeader");
    QString qss = QString(
        "#pageHeader { background-color: @STRIP@; border-bottom: 2px solid @ACNT@; }"
        "#pageStep { background-color: @ACNT@; color: #ffffff; font-weight: bold;"
        "  font-size: 12px; padding: 2px 8px; border-radius: 2px; }"
        "#pageTitle { font-size: 15px; font-weight: bold; color: @TXT1@; }"
        "#pageSub { font-size: 11px; color: @TXT2@; }");
    auto rep = [&qss](const char* token, const QString& value) {
        qss.replace(QString::fromLatin1(token), value);
    };
    rep("@STRIP@", BG_STRIP);
    rep("@ACNT@",  ACCENT);
    rep("@TXT1@",  TEXT_PRIMARY);
    rep("@TXT2@",  TEXT_SECONDARY);
    head->setStyleSheet(qss);

    QHBoxLayout *lay = new QHBoxLayout(head);
    lay->setContentsMargins(8, 6, 8, 6);
    lay->setSpacing(10);

    QLabel *lblStep = new QLabel(stepNo);
    lblStep->setObjectName("pageStep");
    lblStep->setAlignment(Qt::AlignCenter);
    lblStep->setFixedHeight(22);
    lay->addWidget(lblStep);

    QLabel *lblTitle = new QLabel(title);
    lblTitle->setObjectName("pageTitle");
    lay->addWidget(lblTitle);

    QFrame *vline = new QFrame();
    vline->setFrameShape(QFrame::VLine);
    vline->setFrameShadow(QFrame::Plain);
    vline->setFixedWidth(1);
    vline->setStyleSheet(QString("background-color: %1;").arg(BORDER_LIGHT));
    lay->addWidget(vline);

    QLabel *lblSub = new QLabel(subtitle);
    lblSub->setObjectName("pageSub");
    lblSub->setWordWrap(true);
    lay->addWidget(lblSub, 1);

    return head;
}

} // namespace Theme