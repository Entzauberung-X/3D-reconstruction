#include <QSurfaceFormat>
#include <QApplication>
#include "ui/mainwindow.h"
#include "ui/theme.h"

int main(int argc, char *argv[])
{
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
