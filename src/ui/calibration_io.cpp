// ==================== 标定参数持久化 (YAML) ====================
#include "ui/mainwindow.h"
#include "ui/theme.h"
#include <QMessageBox>
#include <QFileInfo>
#include <QDir>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QDialog>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>

static const char* CALIB_FILE = "Resources/calibration/calib_data.yml";

bool MainWindow::saveCalibration()
{
    QDir().mkpath("Resources/calibration");

    cv::FileStorage fs(CALIB_FILE, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        QMessageBox::warning(this, "保存失败", "无法创建标定文件！");
        return false;
    }

    fs << "stereo_rms" << m_StereoRms;

    fs << "cameraMatrixL" << m_CameraMatrixL;
    fs << "distCoeffL" << m_DistCoeffsL;
    fs << "cameraMatrixR" << m_CameraMatrixR;
    fs << "distCoeffR" << m_DistCoeffsR;

    fs << "R_stereo" << m_R;
    fs << "T_stereo" << m_T;
    fs << "R1" << m_R1;
    fs << "R2" << m_R2;
    fs << "P1" << m_P1;
    fs << "P2" << m_P2;

    fs << "laser_plane" << "{:" << "a" << m_LaserPlaneEquation[0]
                       << "b" << m_LaserPlaneEquation[1]
                       << "c" << m_LaserPlaneEquation[2]
                       << "d" << m_LaserPlaneEquation[3] << "}";

    fs << "axis_direction" << m_rotAxisDirection;
    fs << "axis_point" << m_rotAxisPoint;
    fs << "R_base" << m_R_base;
    fs << "T_base" << m_T_base;
    fs << "image_width" << (!m_P1.empty() ? (int)(m_P1.at<double>(0,2) * 2 + 0.5) : 1280);
    fs << "image_height" << (!m_P1.empty() ? (int)(m_P1.at<double>(1,2) * 2 + 0.5) : 960);

    fs.release();

    QString saved = "立体标定(Tab2)";
    if (cv::norm(m_LaserPlaneEquation) > 1e-6f) saved += " + 光平面(Tab3)";
    if (!m_rotAxisDirection.empty() && cv::norm(m_rotAxisDirection) > 0.1) saved += " + 旋转轴(Tab4)";
    txtReconLog->append(QString("[标定保存] 已写入 %1 (%2)").arg(CALIB_FILE, saved));
    return true;
}

bool MainWindow::loadCalibration(bool showDialog)
{
    if (!QFileInfo::exists(CALIB_FILE)) {
        QMessageBox::information(this, "加载标定", "未找到标定文件，请先完成标定流程并保存。");
        return false;
    }

    cv::FileStorage fs(CALIB_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        QMessageBox::warning(this, "加载失败", "无法读取标定文件！");
        return false;
    }

    fs["stereo_rms"] >> m_StereoRms;

    fs["cameraMatrixL"] >> m_CameraMatrixL;
    fs["distCoeffL"] >> m_DistCoeffsL;
    fs["cameraMatrixR"] >> m_CameraMatrixR;
    fs["distCoeffR"] >> m_DistCoeffsR;

    fs["R_stereo"] >> m_R;
    fs["T_stereo"] >> m_T;
    fs["R1"] >> m_R1;
    fs["R2"] >> m_R2;
    fs["P1"] >> m_P1;
    fs["P2"] >> m_P2;

    // 光平面标定 (Tab3, 可选: 可能尚未标定)
    cv::FileNode plane = fs["laser_plane"];
    if (!plane.empty() && !plane["a"].empty()) {
        m_LaserPlaneEquation = cv::Vec4f(
            static_cast<float>((double)plane["a"]),
            static_cast<float>((double)plane["b"]),
            static_cast<float>((double)plane["c"]),
            static_cast<float>((double)plane["d"])
        );
    }

    // 旋转轴标定 (Tab4, 可选: 可能尚未标定)
    cv::FileNode axisDir = fs["axis_direction"];
    cv::FileNode axisPt  = fs["axis_point"];
    cv::FileNode rBase   = fs["R_base"];
    cv::FileNode tBase   = fs["T_base"];
    if (!axisDir.empty()) fs["axis_direction"] >> m_rotAxisDirection;
    if (!axisPt.empty())  fs["axis_point"] >> m_rotAxisPoint;
    if (!rBase.empty())   fs["R_base"] >> m_R_base;
    if (!tBase.empty())   fs["T_base"] >> m_T_base;

    // 读取图像尺寸 (必须在 fs.release() 之前)
    int imgW = 1280, imgH = 960;
    cv::FileNode iw = fs["image_width"];
    cv::FileNode ih = fs["image_height"];
    if (!iw.empty()) iw >> imgW;
    if (!ih.empty()) ih >> imgH;

    fs.release();

    // 参数完整性校验
    if (m_CameraMatrixL.empty() || m_CameraMatrixR.empty() || m_R.empty() || m_T.empty()) {
        QMessageBox::warning(this, "加载失败", "标定文件数据不完整！");
        return false;
    }

    // 重新生成校正映射表
    if (!m_P1.empty() && !m_P2.empty()) {
        cv::initUndistortRectifyMap(m_CameraMatrixL, m_DistCoeffsL, m_R1, m_P1,
                                     cv::Size(imgW, imgH), CV_16SC2, m_MapL1, m_MapL2);
        cv::initUndistortRectifyMap(m_CameraMatrixR, m_DistCoeffsR, m_R2, m_P2,
                                     cv::Size(imgW, imgH), CV_16SC2, m_MapR1, m_MapR2);
        m_IsRectified = true;
    }

    // 更新 UI 显示
    if (!m_R.empty()) {
        std::stringstream ss;
        ss << "立体 RMS: " << std::fixed << std::setprecision(4) << m_StereoRms << " px\n"
           << "基线: " << std::fixed << std::setprecision(2) << cv::norm(m_T) << " mm";
        m_RelativeResultStr = ss.str();
    }

    QString loaded = "立体标定";
    if (cv::norm(m_LaserPlaneEquation) > 1e-6f) loaded += " + 光平面";
    if (!m_rotAxisDirection.empty() && cv::norm(m_rotAxisDirection) > 0.1) loaded += " + 旋转轴";
    txtReconLog->append(QString("[标定加载] 已从 %1 恢复: %2 (RMS=%3px, 基线=%4mm)")
        .arg(CALIB_FILE, loaded).arg(m_StereoRms, 0, 'f', 4).arg(cv::norm(m_T), 0, 'f', 2));

    // 更新状态栏
    m_statusCamCalib->setText("单目: ✓ (已加载)");
    m_statusCamCalib->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(Theme::SUCCESS));
    m_statusStereo->setText(QString("立体: RMS %1px").arg(m_StereoRms, 0, 'f', 3));
    m_statusStereo->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(m_StereoRms < 0.5 ? Theme::SUCCESS : Theme::WARNING));
    if (cv::norm(m_LaserPlaneEquation) > 1e-6f) {
        m_statusLaser->setText("光平面: ✓ (已加载)");
        m_statusLaser->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(Theme::SUCCESS));
    }
    if (!m_rotAxisDirection.empty() && cv::norm(m_rotAxisDirection) > 0.1) {
        m_statusAxis->setText("转台: ✓ (已加载)");
        m_statusAxis->setStyleSheet(QString("color: %1; padding: 0 8px; font-weight: bold;").arg(Theme::SUCCESS));
    }

    // 弹窗显示完整标定参数 (自动加载时静默, 手动加载时弹窗)
    if (showDialog)
        showCalibrationReport();

    return true;
}

void MainWindow::showCalibrationReport()
{
    auto fmtMat = [](const cv::Mat& M, int prec = 4) -> QString {
        if (M.empty()) return "—";
        QString s;
        for (int r = 0; r < M.rows; ++r) {
            if (r > 0) s += "\n";
            for (int c = 0; c < M.cols; ++c)
                s += QString::number(M.at<double>(r, c), 'f', prec) + "\t";
        }
        return s;
    };

    auto fmtDist = [](const cv::Mat& D) -> QString {
        if (D.empty() || D.total() < 5) return "—";
        return QString("[%1, %2, %3, %4, %5]")
            .arg(D.at<double>(0), 0, 'f', 4).arg(D.at<double>(1), 0, 'f', 4)
            .arg(D.at<double>(2), 0, 'f', 4).arg(D.at<double>(3), 0, 'f', 4)
            .arg(D.at<double>(4), 0, 'f', 4);
    };

    auto fmtAxisDir = [&]() -> QString {
        if (m_rotAxisDirection.empty()) return "—";
        return QString("[%1, %2, %3]")
            .arg(m_rotAxisDirection.at<double>(0,0), 0, 'f', 6)
            .arg(m_rotAxisDirection.at<double>(1,0), 0, 'f', 6)
            .arg(m_rotAxisDirection.at<double>(2,0), 0, 'f', 6);
    };

    auto fmtAxisPt = [&]() -> QString {
        if (m_rotAxisPoint.empty()) return "—";
        return QString("[%1, %2, %3] mm")
            .arg(m_rotAxisPoint.at<double>(0,0), 0, 'f', 2)
            .arg(m_rotAxisPoint.at<double>(1,0), 0, 'f', 2)
            .arg(m_rotAxisPoint.at<double>(2,0), 0, 'f', 2);
    };

    auto fmtPlane = [&]() -> QString {
        float n = cv::norm(m_LaserPlaneEquation);
        if (n < 1e-6f) return "— (未标定)";
        return QString("%1 x + %2 y + %3 z + %4 = 0")
            .arg(m_LaserPlaneEquation[0], 0, 'f', 4)
            .arg(m_LaserPlaneEquation[1], 0, 'f', 4)
            .arg(m_LaserPlaneEquation[2], 0, 'f', 4)
            .arg(m_LaserPlaneEquation[3], 0, 'f', 4);
    };

    auto section = [](const QString& title) -> QString {
        return QString("\n══ %1 ══\n").arg(title);
    };

    QString report;
    report += section("立体标定");
    report += QString("RMS 重投影误差: %1 px\n").arg(m_StereoRms, 0, 'f', 4);
    report += QString("基线距离:        %1 mm\n").arg(cv::norm(m_T), 0, 'f', 2);

    report += section("左相机内参 (K_L)");
    report += fmtMat(m_CameraMatrixL, 2) + "\n";
    report += QString("畸变 D_L: %1\n").arg(fmtDist(m_DistCoeffsL));

    report += section("右相机内参 (K_R)");
    report += fmtMat(m_CameraMatrixR, 2) + "\n";
    report += QString("畸变 D_R: %1\n").arg(fmtDist(m_DistCoeffsR));

    report += section("立体外参");
    report += "R (右相对左):\n" + fmtMat(m_R) + "\n";
    report += QString("T (右相对左): %1 mm\n").arg(fmtMat(m_T, 2));

    report += section("校正参数");
    report += "R1 (左校正旋转):\n" + fmtMat(m_R1) + "\n";
    report += "R2 (右校正旋转):\n" + fmtMat(m_R2) + "\n";
    report += "P1 (左校正投影):\n" + fmtMat(m_P1, 2) + "\n";
    report += "P2 (右校正投影):\n" + fmtMat(m_P2, 2) + "\n";

    report += section("光平面标定 (Tab3)");
    report += fmtPlane() + "\n";

    report += section("旋转轴标定 (Tab4)");
    report += QString("轴方向 (相机系): %1\n").arg(fmtAxisDir());
    report += QString("轴点   (相机系): %1\n").arg(fmtAxisPt());
    report += "R_base:\n" + fmtMat(m_R_base) + "\n";
    report += QString("T_base: %1\n").arg(fmtMat(m_T_base, 2));

    QDialog dlg(this);
    dlg.setWindowTitle("标定参数报告 — 已从 calib_data.yml 加载");
    dlg.resize(640, 680);
    dlg.setStyleSheet(QString("background-color: %1;").arg(Theme::BG_CARD));

    QVBoxLayout *lay = new QVBoxLayout(&dlg);

    QTextEdit *text = new QTextEdit();
    text->setReadOnly(true);
    text->setFont(QFont(Theme::MONO, 9));
    text->setStyleSheet(Theme::terminalStyle());
    text->setPlainText(report);
    lay->addWidget(text);

    QPushButton *btnOk = new QPushButton("确定");
    btnOk->setStyleSheet(Theme::successButton());
    btnOk->setFixedWidth(100);
    connect(btnOk, &QPushButton::clicked, &dlg, &QDialog::accept);

    QHBoxLayout *btnLayout = new QHBoxLayout();
    btnLayout->addStretch();
    btnLayout->addWidget(btnOk);
    lay->addLayout(btnLayout);

    dlg.exec();
}

bool MainWindow::tryAutoLoadCalibration()
{
    if (!QFileInfo::exists(CALIB_FILE)) return false;
    return loadCalibration(false);  // 启动时不弹窗, 仅在状态栏和日志显示
}
