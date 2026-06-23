#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTabWidget>
#include <QPushButton>
#include <QTextEdit>
#include <QListWidget>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QString>
#include <QVector>
#include <QButtonGroup>
#include <QProgressBar>
#include <QDateTime>
#include <QTimer>
#include <QMutex>
#include "ui/videowidget.h"
#include "io/camerathread.h"
#include "core/cameracalibration.h"
#include "core/lasercalibration.h"
#include "core/rotatingcalibrator.h"
#include "core/pointcloudbuilder.h"
#include "core/pointcloudviewer.h"
#include "io/serialportmanager.h"
#include <opencv2/opencv.hpp>
#include "io/laserworker.h"
#include "core/pointcalibrator.h"

class LaserWorker;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    // 工具栏
    void onAutoLoadAllImages();

    // Tab1: 采集
    void onOpenLeftCameraClicked();
    void onCaptureLeftClicked();
    void onOpenRightCameraClicked();
    void onCaptureRightClicked();
    void onCaptureBothClicked();
    void onUpdateLeftMat(const cv::Mat &mat);
    void onUpdateRightMat(const cv::Mat &mat);
    void onStartAutoCollect();
    void onPauseAutoCollect();
    void onEndAutoCollect();
    void onAutoCollectStep();
    void performAutoSave(int index, const cv::Mat& leftMat, const cv::Mat& rightMat);
    void onSerialDataReceived(const QByteArray &data);
    void onCaptureTimeout();
    void onStabilizeTimeout();  // 稳定延时结束，执行拍照保存

    // Tab2: 标定
    void onLoadCalibImagesClicked();
    void onCalibrateClicked();
    void onShowLeftParamsClicked();
    void onShowRightParamsClicked();
    void onSelectStereoPairClicked();
    void onCalculateStereoParams();
    void onStereoListItemClicked(QListWidgetItem *item);
    void onShowRelativePoseClicked();
    void onListItemClicked(QListWidgetItem *item);
    void onDeleteSelectedClicked();
    void onSaveCalibrationClicked();
    void onLoadCalibrationClicked();

    // Tab3: 激光
    void onToolbarLaserCalib();
    void onSelectNoLaserImagesClicked();
    void onSelectLaserImagesClicked();
    void onLaserListItemClicked(QListWidgetItem *item);
    void onTab3DetectLaser();
    void onTab3CalibrateLaser();
    void onTab3ShowLaserResult();
    void onTab3ShowLaserAnalysis();
    void connectLaserWorker();
    void onLaserProgress(int current, int total);
    void onLaserResultReady(const std::vector<LaserPair>& results);
    void onLaserFinished();
    void onLaserError(const QString& message);

    // Tab4: 旋转平面标定
    void onLoadRotSeqClicked();
    void onExecAxisCalibClicked();
    void onDebugAxisCalibClicked();
    void onVerifyChessboard3D();
    void onMultiFrameChessboard3D();
    void onReopenChessboardStats();
    void onReopenChessboard3DView();
    void onLoadSiftImagesClicked();
    void onExecSiftMeasureClicked();
    void onSelectSiftRoiLeftClicked();
    void onSelectSiftRoiRightClicked();

    // Tab5: 三维重建
    void onLoadReconSeqClicked();
    void onStartReconstructionClicked();
    void onSaveReconResultClicked();
    void onSelectRoiLeftClicked();
    void onSelectRoiRightClicked();
    void onOpenStandaloneViewer();
    void onShowPointCloudWindowClicked();
    void onPreviewSingleFrameClicked();

private:
    QTimer *m_stabilizeTimer;   // 稳定延时定时器
    QImage mat2QImage(const cv::Mat &mat);
    void initDirectories();
    void updateLaserPairList();
    QStringList getFilesInFolder(const QString& folderPath, int maxCount);
    bool selectRoiOnImage(const QString& imagePath, const QString& windowTitle,
                          cv::Rect& outRect, QPushButton* btnLabel);
    void initTab1(QTabWidget* tabWidget);
    void initTab2(QTabWidget* tabWidget);
    void initTab3(QTabWidget* tabWidget);
    void initTab4(QTabWidget* tabWidget);
    void initTab5(QTabWidget* tabWidget);
    bool saveCalibration();
    bool loadCalibration(bool showDialog = true);
    bool tryAutoLoadCalibration();
    void showCalibrationReport();
    void applyCalibAndParamsToBuilder();

    QTabWidget *tabWidget;
    QLabel *m_statusCamera, *m_statusSerial;
    QLabel *m_statusCamCalib, *m_statusStereo, *m_statusLaser, *m_statusAxis;
    qint64 m_leftReadyTime = 0;
    qint64 m_rightReadyTime = 0;

    // --- 路径 ---
    QString dirLeftCalib;
    QString dirRightCalib;
    QString dirLeftLaser;
    QString dirRightLaser;
    QString dirLeftPlatform;
    QString dirRightPlatform;
    QString dirLeftPointCloud;
    QString dirRightPointCloud;

    // --- Tab1 控件 ---
    VideoWidget *videoLeft;
    VideoWidget *videoRight;
    QPushButton *btnOpenLeftCam;
    QPushButton *btnCaptureLeft;
    QPushButton *btnOpenRightCam;
    QPushButton *btnCaptureRight;
    QPushButton *btnCaptureBoth;
    QPushButton *btnStartAutoCollect;
    QPushButton *btnPauseAutoCollect;
    QPushButton *btnEndAutoCollect;
    QCheckBox *chkCalib;
    QCheckBox *chkLaser;
    QCheckBox *chkPlatform;
    QButtonGroup *saveModeGroup;
    CameraThread *threadLeftCam;
    CameraThread *threadRightCam;
    cv::Mat latestFrameLeft;
    cv::Mat latestFrameRight;
    int captureCount;
    QTimer *m_autoCollectTimer;
    int m_autoCollectCount;
    bool m_isAutoCollecting;
    bool m_isPaused;
    SerialPortManager *m_serialManager;
    QMutex m_matMutex;
    bool m_isWaitingForCapture;
    int m_framesToDiscard;
    bool m_leftFrameReady;
    bool m_rightFrameReady;

    qint64 m_captureStartTimeMs = 0;     // 收到0x02时的时间戳
    static constexpr int DISCARD_DURATION_MS = 150;  // 丢弃150ms内的帧（约5帧@30fps）

    // --- Tab2 控件 ---
    QPushButton *btnLoadLeft;
    QPushButton *btnCalibrate;
    QPushButton *btnShowLeftParams;
    QPushButton *btnShowRightParams;
    QPushButton *btnSelectStereoLeft;
    QPushButton *btnSaveCalib;
    QPushButton *btnLoadCalib;
    QListWidget *listStereoPairs;
    QStringList m_StereoFilesL;
    QStringList m_StereoFilesR;
    QString m_cachedDirLeft;
    QString m_cachedDirRight;
    QPushButton *btnShowRelative;
    QCheckBox *chkCapRotation;
    QDoubleSpinBox *spinMaxRotationDeg;
    QListWidget *listCalibration;
    VideoWidget *displayOriginal;
    VideoWidget *displayCorners;
    VideoWidget *displayPairLeft;
    VideoWidget *displayPairRight;
    CameraCalibration calibratorLeft;
    CameraCalibration calibratorRight;
    cv::Mat m_CameraMatrixL, m_DistCoeffsL;
    cv::Mat m_CameraMatrixR, m_DistCoeffsR;
    cv::Mat m_R, m_T, m_E, m_F;
    cv::Mat m_R1, m_R2, m_P1, m_P2, m_Q;
    cv::Mat m_MapL1, m_MapL2;
    cv::Mat m_MapR1, m_MapR2;
    bool m_IsRectified;
    double m_StereoRms = 0.0;
    std::string m_RelativeResultStr;
    std::vector<double> m_perPairEpiErrors;  // 每对图像的校正极线Y误差
    float m_squareSize;

    // --- Tab3 控件 ---
    VideoWidget *tab3ViewLeftUndistort;
    VideoWidget *tab3ViewRightUndistort;
    VideoWidget *tab3ViewLeftLaser;
    VideoWidget *tab3ViewRightLaser;
    QPushButton *btnSelectNoLaser;
    QPushButton *btnSelectLaser;
    QPushButton *btnShowAnalysis;
    QListWidget *listLaserPairs;
    QStringList m_NoLaserFilesL, m_NoLaserFilesR;
    QStringList m_LaserFilesL, m_LaserFilesR;
    std::vector<LaserPair> m_AllLaserData;
    QPushButton *btnShowLaserResult;
    cv::Size m_boardSize;
    cv::Vec4f m_LaserPlaneEquation;
    std::string m_LaserResultStr;
    StegerParams m_StegerParams;
    LABParams m_LABParams;
    QThread* m_laserThread = nullptr;
    LaserWorker* m_laserWorker = nullptr;
    bool m_isLaserProcessing = false;
    bool m_pendingLaserCalib = false;
    LaserCalibration m_laserCalib;
    std::vector<LaserProcessingResult> m_cachedResultsL;
    std::vector<LaserProcessingResult> m_cachedResultsR;
    QLabel *lblPlatLeftAxis;
    QLabel *lblPlatRightAxis;

    // --- Tab4 控件 ---
    cv::Mat m_rotAxisDirection;
    cv::Mat m_rotAxisPoint;
    cv::Mat m_R_base;          // 相机在角度0时的位姿 (世界→相机)
    cv::Mat m_T_base;

    // --- Tab4 旋转轴自动标定 ---
    QPushButton *btnLoadRotSeq;
    QPushButton *btnExecAxisCalib;
    QPushButton *btnDebugAxisCalib;
    QPushButton *btnVerify3D;
    QPushButton *btnMultiFrame3D;
    QPushButton *btnMultiFrameStats;
    QPushButton *btnMultiFrame3DView;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_lastChessboardCloud;
    QString m_lastChessboardStats;
    QTextEdit *txtAxisCalibResult;
    QLabel *lblAxisCamFrame;
    Calib::RotatingCalibrator* m_rotatingCalibrator;
    QStringList m_rotSeqLeftPaths;
    QStringList m_rotSeqRightPaths;

    PointCalibrator* m_pointCalibrator;
    QPushButton *btnLoadSiftImages;
    QPushButton *btnExecSiftMeasure;
    QLabel *lblSiftLeftImg;
    QLabel *lblSiftRightImg;
    QTextEdit *txtSiftResult;
    QString m_siftLeftPath;
    QString m_siftRightPath;

    // --- Tab4 SIFT ROI 新增 ---
    QPushButton *btnSelectSiftRoiLeft;
    QPushButton *btnSelectSiftRoiRight;
    cv::Rect m_siftRoiLeft;
    cv::Rect m_siftRoiRight;

    // ==================== Tab5: 三维重建 控件与参数 ====================
    QPushButton *btnLoadReconSeq;
    QPushButton *btnStartRecon;
    QPushButton* btnOpenViewer;
    QPushButton *btnSaveRecon;
    QPushButton *btnShowPointCloudWindow;
    
    QSpinBox *spinViewCount;
    QDoubleSpinBox *spinAngleStep;
    
    QProgressBar *progressRecon;
    QTextEdit *txtReconLog;
    QLabel *lblReconLeftView;
    QLabel *lblReconRightView;

    PointCloudBuilder* m_builder;
    PointCloudViewer* m_viewer3D;
    QStringList m_reconLeftPaths;
    QStringList m_reconRightPaths;

    QSpinBox* spinPreviewIndex;
    QPushButton* btnPreviewSingleFrame;

    // --- S1: 光条中心提取参数 (距离变换+灰度重心法) ---
    QDoubleSpinBox* spinMorphSigma;
    QComboBox* cmbS1Method;         // S1方法选择
    QSpinBox* spinLabA;             // 红色阈值 (LAB A通道)
    QSpinBox* spinSampleStep;       // 降采样步长
    QSpinBox *spinOverexposedThresh;  
    QSpinBox *spinMaxOverexposedGap;
    
    // --- S1 Steger 新增参数控件 ---
    QCheckBox* chkUseSteger;
    QDoubleSpinBox* spinStegerSigma;
    QDoubleSpinBox* spinStegerTMax;
    QCheckBox* chkOverexposedEnable;
    QSpinBox* spinOverexposedL;
    QDoubleSpinBox* spinEdgeOffsetSigma;

    // --- ROI 框选 ---
    QPushButton *btnSelectRoiLeft;
    QPushButton *btnSelectRoiRight;
    cv::Rect m_roiLeft;
    cv::Rect m_roiRight;

    // --- S2: 双目极线匹配参数 ---
    QDoubleSpinBox *spinEpipolarThresh;
    QDoubleSpinBox *spinDepthMin;
    QDoubleSpinBox *spinDepthMax;
    QSpinBox *spinMinPtsSeg;
    QDoubleSpinBox *spinBreakDist;
    QDoubleSpinBox *spinDisparityBreak;

    // --- S5: 多视角ICP配准参数 ---
    QCheckBox *chkUseIcp;
    QDoubleSpinBox *spinIcpMaxDist;
    QSpinBox *spinIcpIter;
    QDoubleSpinBox *spinIcpEpsilon;
    QDoubleSpinBox *spinIcpAxisTrust;
    QDoubleSpinBox *spinIcpTransEps;
    QDoubleSpinBox *spinDpSkipPenalty;
    QDoubleSpinBox *spinDpSmoothWeight;

    // --- S6: 去噪/滤波与网格化参数 ---
    QDoubleSpinBox *spinSorMeanK;
    QDoubleSpinBox *spinSorStdMul;
    QDoubleSpinBox *spinVoxelSize;
    QDoubleSpinBox* spinGp3Radius;
    QDoubleSpinBox* spinMeshTruncationDist;
    QSpinBox* spinPoissonDepth;
    QComboBox* cmbMeshMethod;
    QDoubleSpinBox* spinPoissonPointWeight;
};

#endif // MAINWINDOW_H
