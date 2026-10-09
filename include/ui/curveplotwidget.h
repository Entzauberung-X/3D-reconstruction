#ifndef CURVEPLOTWIDGET_H
#define CURVEPLOTWIDGET_H

/**
 * @file curveplotwidget.h
 * @brief 轻量自绘曲线控件 (QPainter)
 *
 * 为什么自己写: 本工程没有引入任何绘图库 (无 QtCharts / QCustomPlot, 全仓零命中),
 * 而 Tab6 需要展示「圆拟合观测点+拟合圆」「圆拟合残差」「BA逐迭代重投影」「逐帧点数」
 * 「误差直方图」五类二维图。引入一个新依赖只为画几张静态图不划算, 故直接用 QPainter 实现。
 *
 * 设计边界: **只画不交互** —— 没有缩放/平移/悬停取值。图表尺寸小、用途是"看一眼分布"，
 * 交互带来的复杂度与收益不成比例。
 */

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QColor>
#include <QString>
#include <QStringList>

class CurvePlotWidget : public QWidget
{
public:
    /** 系列类型: 折线 / 散点 / 柱状 (柱状基线恒为 y=0) */
    enum class Kind { Line, Scatter, Bar };
    /** 折线的点标记 (散点系列忽略此项, 恒画标记) */
    enum class Mark { None, Circle, Square, Triangle };

    struct Series {
        Kind             kind  = Kind::Line;
        QVector<QPointF> pts;
        QColor           color = QColor(31, 111, 178);
        QString          name;
        double           width = 1.6;
        Mark             mark  = Mark::None;
        double           markSize = 4.0;
        bool             dashed = false;
    };

    /** 横向参考线 (y = const): BA 的误差阈值、残差的 0 基线 */
    struct RefLine {
        double  y = 0.0;
        QColor  color = QColor(192, 57, 43);
        QString name;
        bool    dashed = true;
    };

    /** 叠加参考圆 (虚线): 圆拟合结果的完整圆。要求 setEqualAspect(true), 否则显示成椭圆 */
    struct Circle {
        QPointF center;
        double  radius = 0.0;
        QColor  color = QColor(111, 66, 193);
        QString name;
        bool    dashed = true;
        double  width = 1.6;
    };

    explicit CurvePlotWidget(QWidget* parent = nullptr);

    void setTitle(const QString& t)      { m_title = t; update(); }
    void setAxisLabels(const QString& x, const QString& y) { m_xLabel = x; m_yLabel = y; update(); }
    /** 等比例坐标轴 (圆必须开; 直线/柱状关掉可充分利用画布) */
    void setEqualAspect(bool on)         { m_equalAspect = on; update(); }
    void setLegendVisible(bool on)       { m_legend = on; update(); }
    /**
     * 自定义 X 轴刻度标签, 依次对齐到 x = 0, 1, 2, ...
     * 用于类别型横轴 (如直方图的 8 个误差区间): 此时横轴不是连续量, 画数值刻度没有意义,
     * 而把区间边界画成数值 (1/2/5/10...) 又会因为线性轴把左侧几根柱子挤成一堆。
     */
    void setXTickLabels(const QStringList& labels) { m_xTickLabels = labels; update(); }

    void addSeries(const Series& s)      { m_series.push_back(s); update(); }
    void addRefLine(const RefLine& r)    { m_refs.push_back(r); update(); }
    void addCircle(const Circle& c)      { m_circles.push_back(c); update(); }

    /** 清空全部系列 (保留标题/轴标签等设置) */
    void clear();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QVector<Series>  m_series;
    QVector<RefLine> m_refs;
    QVector<Circle>  m_circles;
    QString m_title, m_xLabel, m_yLabel;
    QStringList m_xTickLabels;   // 非空时用类别标签代替 X 数值刻度
    bool    m_equalAspect = false;
    bool    m_legend = true;
};

#endif // CURVEPLOTWIDGET_H
