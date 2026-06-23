#include "ui/videowidget.h"
#include "ui/theme.h"
#include <QPainter>

VideoWidget::VideoWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(Theme::VIDEO_MIN_W, Theme::VIDEO_MIN_H);
    setStyleSheet(Theme::videoBorderStyle());
}

void VideoWidget::setFrame(const QImage &frame)
{
    QMutexLocker locker(&frameMutex);
    
    // 只在有新帧时更新SS
    if (!frame.isNull() && currentFrame != frame) {
        currentFrame = frame;
        update(); // 触发重绘
    }
}

void VideoWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    
    QPainter painter(this);
    
    // 填充黑色背景
    painter.fillRect(rect(), Qt::black);
    
    QMutexLocker locker(&frameMutex);
    
    // 如果有当前帧，绘制它
    if (!currentFrame.isNull()) {
        // 计算保持宽高比的缩放
        QRect targetRect;
        QSize frameSize = currentFrame.size();
        QSize widgetSize = size();
        
        double frameAspect = (double)frameSize.width() / frameSize.height();
        double widgetAspect = (double)widgetSize.width() / widgetSize.height();
        
        if (widgetAspect > frameAspect) {
            // 以高度为基准
            int height = widgetSize.height();
            int width = height * frameAspect;
            targetRect = QRect((widgetSize.width() - width) / 2, 0, width, height);
        } else {
            // 以宽度为基准
            int width = widgetSize.width();
            int height = width / frameAspect;
            targetRect = QRect(0, (widgetSize.height() - height) / 2, width, height);
        }
        
        // 绘制图像
        painter.drawImage(targetRect, currentFrame);
    } else {
        // 没有图像时显示文本
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "无视频信号");
    }
}
