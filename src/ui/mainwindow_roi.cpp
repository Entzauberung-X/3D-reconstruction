// ==================== ROI 框选共享辅助 ====================
// Tab4 SIFT ROI (m_siftRoiLeft/Right) 和 Tab5 激光 ROI (m_roiLeft/Right)
// 各自维护独立的成员变量，互不干扰。对话框打开时显示已有选区，取消保留原值。
#include "ui/mainwindow.h"
#include "ui/videowidget.h"
#include "ui/theme.h"

#include <QDialog>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRubberBand>
#include <QMouseEvent>
#include <QMessageBox>
#include <opencv2/opencv.hpp>

namespace {

class LambdaEventFilter : public QObject {
public:
    explicit LambdaEventFilter(std::function<bool(QEvent*)> filterFunc, QObject* parent = nullptr)
        : QObject(parent), m_filterFunc(std::move(filterFunc)) {}
protected:
    bool eventFilter(QObject *watched, QEvent *event) override {
        if (m_filterFunc) { return m_filterFunc(event); }
        return QObject::eventFilter(watched, event);
    }
private:
    std::function<bool(QEvent*)> m_filterFunc;
};

} // anonymous namespace

bool MainWindow::selectRoiOnImage(const QString& imagePath, const QString& windowTitle,
                                   cv::Rect& outRect, QPushButton* btnLabel)
{
    cv::Mat mat = cv::imread(imagePath.toStdString());
    if (mat.empty()) return false;
    cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
    QImage baseImage = QImage(mat.data, mat.cols, mat.rows,
                              static_cast<int>(mat.step), QImage::Format_RGB888).copy();
    if (baseImage.isNull()) return false;

    QDialog dlg(nullptr);
    dlg.setWindowTitle(windowTitle);
    dlg.setStyleSheet(QString("background-color: %1;").arg(Theme::TEXT_PRIMARY));
    dlg.resize(baseImage.size().boundedTo(QSize(1920, 1080)));

    QVBoxLayout *layout = new QVBoxLayout(&dlg);
    QLabel *imgLabel = new QLabel(&dlg);
    imgLabel->setPixmap(QPixmap::fromImage(baseImage));
    imgLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(imgLabel);

    QPushButton *btnConfirm = new QPushButton("确认当前选区");
    btnConfirm->setStyleSheet(Theme::successButton());
    layout->addWidget(btnConfirm);

    // 使用已有ROI作为初始选区，若无则默认居中半幅
    QRect finalRoi;
    if (outRect.width > 0 && outRect.height > 0) {
        finalRoi = QRect(outRect.x, outRect.y, outRect.width, outRect.height);
    } else {
        finalRoi = QRect(baseImage.width()/4, baseImage.height()/4,
                         baseImage.width()/2, baseImage.height()/2);
    }
    QPoint origin;
    QRubberBand *rubberBand = nullptr;
    // 初始时显示已有选区
    rubberBand = new QRubberBand(QRubberBand::Rectangle, imgLabel);
    rubberBand->setGeometry(finalRoi);
    rubberBand->show();

    LambdaEventFilter *filter = new LambdaEventFilter(
        [&](QEvent *event) -> bool {
            if (event->type() == QEvent::MouseButtonPress) {
                QMouseEvent *me = static_cast<QMouseEvent*>(event);
                if (me->button() == Qt::LeftButton) {
                    origin = me->pos();
                    if (rubberBand) delete rubberBand;
                    rubberBand = new QRubberBand(QRubberBand::Rectangle, imgLabel);
                    rubberBand->setGeometry(QRect(origin, QSize()));
                    rubberBand->show();
                }
                return true;
            } else if (event->type() == QEvent::MouseMove) {
                QMouseEvent *me = static_cast<QMouseEvent*>(event);
                if (rubberBand) {
                    rubberBand->setGeometry(QRect(origin, me->pos()).normalized());
                }
                return true;
            } else if (event->type() == QEvent::MouseButtonRelease) {
                QMouseEvent *me = static_cast<QMouseEvent*>(event);
                if (me->button() == Qt::LeftButton && rubberBand) {
                    rubberBand->hide();
                    finalRoi = QRect(origin, me->pos()).normalized();
                }
                return true;
            }
            return false;
        }, imgLabel
    );
    imgLabel->installEventFilter(filter);
    QObject::connect(btnConfirm, &QPushButton::clicked, &dlg, &QDialog::accept);

    if (dlg.exec() == QDialog::Accepted) {
        outRect = cv::Rect(finalRoi.x(), finalRoi.y(), finalRoi.width(), finalRoi.height());
        if (btnLabel) {
            btnLabel->setText(QString("ROI: [%1,%2,%3,%4]")
                              .arg(outRect.x).arg(outRect.y).arg(outRect.width).arg(outRect.height));
        }
        return true;
    }
    // 取消时保留原有ROI不变
    return false;
}
