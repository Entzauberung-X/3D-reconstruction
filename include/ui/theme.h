#pragma once
#include <QString>

namespace Theme {

// ── Color Palette (Light Industrial) ──
inline constexpr auto BG_MAIN        = "#f0f2f5";
inline constexpr auto BG_CARD        = "#ffffff";
inline constexpr auto BG_INPUT       = "#ffffff";
inline constexpr auto BG_HOVER       = "#e2e6ea";
inline constexpr auto BORDER         = "#c0c4cc";
inline constexpr auto BORDER_FOCUS   = "#2980b9";
inline constexpr auto TEXT_PRIMARY   = "#2c3e50";
inline constexpr auto TEXT_SECONDARY = "#6b7b8d";
inline constexpr auto TEXT_DIM       = "#999999";
inline constexpr auto ACCENT         = "#2980b9";
inline constexpr auto SUCCESS        = "#27ae60";
inline constexpr auto WARNING        = "#f39c12";
inline constexpr auto DANGER         = "#e74c3c";
inline constexpr auto PURPLE         = "#8e44ad";

// ── Terminal colors (keep dark for log/data display) ──
inline constexpr auto TERM_BG        = "#1e1e1e";
inline constexpr auto TERM_TEXT      = "#cccccc";
inline constexpr auto TERM_BORDER    = "#555555";

// ── Fonts ──
inline const char* const MONO = "Consolas";
inline const char* const UI   = "Segoe UI";

// ── Dimensions ──
inline constexpr int BTN_MIN_W      = 110;
inline constexpr int BTN_H          = 32;
inline constexpr int CONTROL_PANEL_W = 260;
inline constexpr int VIDEO_MIN_W    = 320;
inline constexpr int VIDEO_MIN_H    = 240;
inline constexpr int BORDER_RADIUS  = 4;
inline constexpr int MARGIN         = 8;
inline constexpr int SPACING        = 6;

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

} // namespace Theme
