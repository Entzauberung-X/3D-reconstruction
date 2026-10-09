#include <QSurfaceFormat>
#include <QApplication>
#include "ui/mainwindow.h"
#include "ui/theme.h"

namespace {

/**
 * 静音全部终端输出。
 *
 * 工程里有一百多处输出调用, 但它们走的是**三条互不相干的通路**, 只堵一条会漏掉另外两条:
 *   1. qDebug/qWarning/qInfo/qCritical —— Qt 消息系统 (默认直接写 stderr)
 *   2. std::cout / std::cerr           —— C++ 流 (默认与 stdio 同步)
 *   3. printf/fprintf + PCL/OpenCV 等第三方库内部 —— C stdio (stdout/stderr)
 * 所以这里两条都堵:
 *   · freopen 到 /dev/null —— 覆盖 stdio 全家 (含与 stdio 同步的 cout/cerr 以及第三方库)
 *   · qInstallMessageHandler —— 覆盖 Qt 消息系统 (它在 Unix 上可能直接写 fd, 不经 stdio)
 * 两者叠加才是完整的, 单靠任一条都可能漏。
 *
 * **不影响 Resources/app.log**: Logger 自己持有 QFile/QTextStream 写入 (见 ui/logger.h),
 * 与终端通路完全无关, 所以日志文件照常记录。
 *
 * 需要排查问题时设环境变量 CAMERA_VERBOSE=1 可整体恢复终端输出。
 */
void silenceTerminalOutput()
{
    if (!qEnvironmentVariableIsEmpty("CAMERA_VERBOSE")) return;   // 调试逃生口

#ifdef Q_OS_WIN
    FILE* sink = nullptr;                                          // Windows 没有 /dev/null
    freopen_s(&sink, "NUL", "w", stdout);
    freopen_s(&sink, "NUL", "w", stderr);
#else
    freopen("/dev/null", "w", stdout);
    freopen("/dev/null", "w", stderr);
#endif

    qInstallMessageHandler([](QtMsgType, const QMessageLogContext&, const QString&) {});
}

} // namespace

int main(int argc, char *argv[])
{
    silenceTerminalOutput();   // 必须最早调用: 晚于它的一切输出都早已打出去了

    // 解决 QVTKOpenGLWidget 在某些平台下的兼容性问题
    QSurfaceFormat fmt;
    fmt.setVersion(3, 2);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    qRegisterMetaType< QVector<LaserPair> >("QVector<LaserPair>");
    qRegisterMetaType< QVector<LaserProcessingResult> >("QVector<LaserProcessingResult>");
    qRegisterMetaType<cv::Mat>("cv::Mat");

    QApplication app(argc, argv);

    // ── 全局工业白色主题 QSS (仅覆盖需要统一的控件) ──
    app.setStyleSheet(QString(
        // QTabBar
        "QTabBar::tab { padding: 8px 20px; }"
        "QTabBar::tab:selected { font-weight: bold; }"
        // QGroupBox
        "QGroupBox { font-weight: bold; border: 1px solid %1; border-radius: 4px;"
        "  margin-top: 12px; padding-top: 16px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; color: %2; }"
        // Base button
        "QPushButton { border: 1px solid %1; border-radius: 4px;"
        "  padding: 5px 14px; min-height: 28px; }"
        "QPushButton:hover { background-color: %3; }"
        "QPushButton:disabled { color: #999; }"
        // Input widgets
        "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {"
        "  border: 1px solid %1; border-radius: 3px; padding: 3px 6px; min-height: 24px; }"
        // Lists
        "QListWidget, QTableWidget { border: 1px solid %1; border-radius: 3px; }"
        // ScrollArea
        "QScrollArea { border: none; }"
        // ProgressBar
        "QProgressBar { border: 1px solid %1; border-radius: 3px; text-align: center; min-height: 20px; }"
        "QProgressBar::chunk { background-color: %2; border-radius: 2px; }"
        // Header
        "QHeaderView::section { padding: 4px 8px; border: 1px solid %1; font-weight: bold; }"
    ).arg(Theme::BORDER, Theme::ACCENT, Theme::BG_HOVER));

    MainWindow w;
    w.show();
    return app.exec();
}
