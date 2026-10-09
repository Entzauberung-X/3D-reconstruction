#include "ui/curveplotwidget.h"
#include "ui/theme.h"

#include <QPainter>
#include <QPolygonF>
#include <QPaintEvent>
#include <QFontMetrics>
#include <cmath>
#include <limits>
#include <algorithm>

namespace {

/** 取一个"好看"的刻度步长 (1 / 2 / 5 × 10^k) */
double niceStep(double range, int targetTicks)
{
    if (!(range > 0.0) || targetTicks < 1) return 1.0;
    const double raw = range / targetTicks;
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double n   = raw / mag;
    // 阈值取 1.5 / 3 / 7 (而不是 1 / 2 / 5): 用后者时 n=2.1 会跳到 5, 刻度数只剩目标的一半,
    // 直方图这类量程大的图会稀到只剩两三根刻度线
    const double m   = (n <= 1.5) ? 1.0 : (n <= 3.0 ? 2.0 : (n <= 7.0 ? 5.0 : 10.0));
    return m * mag;
}

/** 按步长决定标签小数位, 避免出现 "0.30000000000000004" 这类值 */
QString fmtTick(double v, double step)
{
    int prec = 0;
    if (step < 1.0)   prec = (step < 0.1) ? 2 : 1;
    if (step < 0.01)  prec = 3;
    QString s = QString::number(v, 'f', prec);
    if (s == "-0" || s.startsWith("-0.0")) s.remove(0, 1);   // 抹掉轴上的 "-0"
    return s;
}

/** 画一个点标记 */
void drawMark(QPainter& p, const QPointF& c, CurvePlotWidget::Mark m, double size)
{
    const double h = size;   // 半径
    switch (m) {
    case CurvePlotWidget::Mark::Square:
        p.drawRect(QRectF(c.x() - h, c.y() - h, 2 * h, 2 * h));
        break;
    case CurvePlotWidget::Mark::Triangle: {
        QPolygonF t;
        t << QPointF(c.x(), c.y() - h) << QPointF(c.x() - h, c.y() + h) << QPointF(c.x() + h, c.y() + h);
        p.drawPolygon(t);
        break;
    }
    case CurvePlotWidget::Mark::Circle:
    default:
        p.drawEllipse(c, h, h);
        break;
    }
}

} // namespace

CurvePlotWidget::CurvePlotWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(120);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void CurvePlotWidget::clear()
{
    m_series.clear();
    m_refs.clear();
    m_circles.clear();
    update();
}

void CurvePlotWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(Theme::BG_CARD));

    // ---------------- 数据范围 ----------------
    double xmin =  std::numeric_limits<double>::max(), xmax = -std::numeric_limits<double>::max();
    double ymin =  std::numeric_limits<double>::max(), ymax = -std::numeric_limits<double>::max();
    bool   hasData = false;

    auto grow = [&](double x, double y) {
        xmin = std::min(xmin, x); xmax = std::max(xmax, x);
        ymin = std::min(ymin, y); ymax = std::max(ymax, y);
        hasData = true;
    };
    for (const Series& s : m_series) {
        for (const QPointF& q : s.pts) {
            grow(q.x(), q.y());
            if (s.kind == Kind::Bar) grow(q.x(), 0.0);   // 柱状要从基线长上来
        }
    }
    for (const Circle& c : m_circles) {
        grow(c.center.x() - c.radius, c.center.y() - c.radius);
        grow(c.center.x() + c.radius, c.center.y() + c.radius);
    }

    // ---------------- 版面 ----------------
    const bool hasTitle = !m_title.isEmpty();
    const int  mL = 60, mR = 16;
    const int  mT = hasTitle ? 32 : 14;
    const int  mB = m_xLabel.isEmpty() ? 30 : 46;
    QRectF plot = QRectF(rect()).adjusted(mL, mT, -mR, -mB);
    if (plot.width() < 40 || plot.height() < 30) return;

    // ---------------- 标题 ----------------
    if (hasTitle) {
        p.setPen(QColor(Theme::TEXT_PRIMARY));
        QFont f(Theme::UI, 10);
        f.setBold(true);
        p.setFont(f);
        p.drawText(QRectF(0, 6, width(), 20), Qt::AlignHCenter | Qt::AlignVCenter, m_title);
    }

    if (!hasData) {
        p.setPen(QColor(Theme::TEXT_DIM));
        p.setFont(QFont(Theme::UI, 10));
        p.drawText(plot, Qt::AlignCenter, "无数据");
        return;
    }

    // 退化: 所有点共线 / 只有一个点 → 给一个人造跨度, 否则除零
    if (xmax - xmin < 1e-12) { xmin -= 1.0; xmax += 1.0; }
    if (ymax - ymin < 1e-12) { ymin -= 1.0; ymax += 1.0; }

    double padX = 0.04 * (xmax - xmin), padY = 0.08 * (ymax - ymin);
    xmin -= padX; xmax += padX;
    ymin -= padY; ymax += padY;

    // 参考线略微超出数据范围时把它拉进来, 否则阈值线看不到。
    // 但**最多只扩展一个数据跨度**: 若阈值远在数据之上 (例如 BA 收敛到 0.3px 而阈值设成 20px),
    // 无条件并入会把曲线压成贴底的一条直线 —— 那比"看不见阈值线"糟糕得多。
    // 阈值本身在弹窗顶部的数值条里已经写明, 不依赖这根线来传达。
    const double dataSpanY = ymax - ymin;
    for (const RefLine& r : m_refs) {
        ymin = std::min(ymin, std::max(r.y, ymin - dataSpanY));
        ymax = std::max(ymax, std::min(r.y, ymax + dataSpanY));
    }
    if (ymin > 0.0 && ymin < 0.5 * (ymax - ymin)) ymin = 0.0;   // 贴 0 的图从 0 起更直观

    // 等比例: 用同一 scale 决定两轴跨度, 保证圆是圆
    if (m_equalAspect) {
        const double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
        const double scale = std::max((xmax - xmin) / plot.width(), (ymax - ymin) / plot.height());
        const double hx = 0.5 * scale * plot.width(), hy = 0.5 * scale * plot.height();
        xmin = cx - hx; xmax = cx + hx;
        ymin = cy - hy; ymax = cy + hy;
    }

    const double sx = plot.width()  / (xmax - xmin);
    const double sy = plot.height() / (ymax - ymin);
    auto px = [&](double x) { return plot.left() + (x - xmin) * sx; };
    auto py = [&](double y) { return plot.bottom() - (y - ymin) * sy; };

    // ---------------- 网格与刻度 ----------------
    const bool  catX  = !m_xTickLabels.isEmpty();   // 类别型横轴 (直方图)
    const double stx  = niceStep(xmax - xmin, 6);
    const double sty  = niceStep(ymax - ymin, 5);
    QFont tickFont(Theme::MONO, 8);
    p.setFont(tickFont);

    p.setPen(QPen(QColor(Theme::BORDER_LIGHT), 1, Qt::DotLine));
    if (catX) {
        for (int i = 0; i < m_xTickLabels.size(); ++i) {
            const double vx = i;
            if (vx < xmin || vx > xmax) continue;
            p.drawLine(QPointF(px(vx), plot.top()), QPointF(px(vx), plot.bottom()));
        }
    } else {
        for (double v = std::ceil(xmin / stx) * stx; v <= xmax + 1e-9; v += stx)
            p.drawLine(QPointF(px(v), plot.top()), QPointF(px(v), plot.bottom()));
    }
    for (double v = std::ceil(ymin / sty) * sty; v <= ymax + 1e-9; v += sty)
        p.drawLine(QPointF(plot.left(), py(v)), QPointF(plot.right(), py(v)));

    p.setPen(QColor(Theme::TEXT_SECONDARY));
    p.setBrush(Qt::NoBrush);
    if (catX) {
        for (int i = 0; i < m_xTickLabels.size(); ++i) {
            const double vx = i;
            if (vx < xmin || vx > xmax) continue;
            p.drawText(QRectF(px(vx) - 24, plot.bottom() + 3, 48, 14), Qt::AlignCenter, m_xTickLabels[i]);
        }
    } else {
        for (double v = std::ceil(xmin / stx) * stx; v <= xmax + 1e-9; v += stx)
            p.drawText(QRectF(px(v) - 40, plot.bottom() + 3, 80, 14), Qt::AlignCenter, fmtTick(v, stx));
    }
    for (double v = std::ceil(ymin / sty) * sty; v <= ymax + 1e-9; v += sty)
        p.drawText(QRectF(plot.left() - 56, py(v) - 8, 52, 16), Qt::AlignRight | Qt::AlignVCenter,
                   fmtTick(v, sty));

    // 边框
    p.setPen(QColor(Theme::BORDER));
    p.drawRect(plot);

    // ---------------- 绘图区内容 (裁剪) ----------------
    p.save();
    p.setClipRect(plot.adjusted(-1, -1, 1, 1));

    // 参考线 (画在最底层)
    for (const RefLine& r : m_refs) {
        if (r.y < ymin || r.y > ymax) continue;
        p.setPen(QPen(r.color, 1.2, r.dashed ? Qt::DashLine : Qt::SolidLine));
        p.drawLine(QPointF(plot.left(), py(r.y)), QPointF(plot.right(), py(r.y)));
    }

    // 系列
    for (const Series& s : m_series) {
        if (s.pts.isEmpty()) continue;

        if (s.kind == Kind::Bar) {
            const double bw = std::max(2.0, 0.62 * plot.width() / std::max(1, s.pts.size()));
            p.setPen(QPen(s.color.darker(120), 1));
            p.setBrush(s.color);
            for (const QPointF& q : s.pts) {
                const double x0 = px(q.x()) - bw / 2.0;
                const double y0 = py(0.0), y1 = py(q.y());
                p.drawRect(QRectF(x0, std::min(y0, y1), bw, std::fabs(y1 - y0)));
            }
            continue;
        }

        if (s.kind == Kind::Line && s.pts.size() >= 2) {
            QPolygonF poly;
            poly.reserve(s.pts.size());
            for (const QPointF& q : s.pts) poly << QPointF(px(q.x()), py(q.y()));
            p.setPen(QPen(s.color, s.width, s.dashed ? Qt::DashLine : Qt::SolidLine,
                          Qt::RoundCap, Qt::RoundJoin));
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(poly);
        }

        if (s.kind == Kind::Scatter || s.mark != Mark::None) {
            p.setPen(QPen(s.color.darker(130), 1));
            p.setBrush(s.color);
            for (const QPointF& q : s.pts) drawMark(p, QPointF(px(q.x()), py(q.y())), s.mark, s.markSize);
        }
    }

    // 拟合圆 (虚线整圆)
    for (const Circle& c : m_circles) {
        p.setPen(QPen(c.color, c.width, c.dashed ? Qt::DashLine : Qt::SolidLine));
        p.setBrush(Qt::NoBrush);
        const double rpx = c.radius * sx;
        p.drawEllipse(QPointF(px(c.center.x()), py(c.center.y())), rpx, rpx);
    }
    p.restore();

    // ---------------- 轴标签 ----------------
    p.setPen(QColor(Theme::TEXT_SECONDARY));
    p.setFont(QFont(Theme::UI, 9));
    if (!m_xLabel.isEmpty())
        p.drawText(QRectF(plot.left(), plot.bottom() + 20, plot.width(), 18),
                   Qt::AlignHCenter | Qt::AlignVCenter, m_xLabel);
    if (!m_yLabel.isEmpty()) {
        p.save();
        p.translate(12, plot.center().y());
        p.rotate(-90);
        p.drawText(QRectF(-plot.height() / 2, -10, plot.height(), 18),
                   Qt::AlignCenter, m_yLabel);
        p.restore();
    }

    // ---------------- 图例 / 参考线标注 ----------------
    QFont legFont(Theme::UI, 8);
    p.setFont(legFont);
    const QFontMetrics legFm(legFont);
    double legY = plot.top() + 6;
    const double legX = plot.right() - 8;

    auto legendRow = [&](const QColor& col, const QString& text, bool dashed, bool mark,
                         const QColor& textColor) {
        if (text.isEmpty()) return;
        const int tw = legFm.horizontalAdvance(text);
        const double x0 = legX - tw - 22;
        p.setPen(QPen(col, 1.6, dashed ? Qt::DashLine : Qt::SolidLine));
        p.drawLine(QPointF(x0, legY + 6), QPointF(x0 + 14, legY + 6));
        if (mark) {
            p.setBrush(col);
            p.drawEllipse(QPointF(x0 + 7, legY + 6), 2.5, 2.5);
        }
        p.setPen(textColor);
        p.drawText(QRectF(x0 + 18, legY, tw + 4, 12), Qt::AlignLeft | Qt::AlignVCenter, text);
        legY += 13;
    };

    for (const Series& s : m_series)
        legendRow(s.color, s.name, s.dashed, s.kind == Kind::Scatter || s.mark != Mark::None,
                  QColor(Theme::TEXT_PRIMARY));
    for (const Circle& c : m_circles)
        legendRow(c.color, c.name, c.dashed, false, QColor(Theme::TEXT_PRIMARY));
    for (const RefLine& r : m_refs) {
        // 被裁在可视范围外的参考线不画也不列; 给它留一行图例会让人以为图上真有那么一条线
        if (r.y < ymin || r.y > ymax) continue;
        legendRow(r.color, r.name, r.dashed, false, QColor(Theme::TEXT_SECONDARY));
    }
}
