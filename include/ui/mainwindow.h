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
#include <functional>
#include "ui/videowidget.h"
#include "io/camerathread.h"
#include "core/cameracalibration.h"
#include "core/lasercalibration.h"
#include "core/rotatingcalibrator.h"
#include "core/pointcloudbuilder.h"
#include "core/pointcloudviewer.h"
#include "io/serialportmanager.h"
#include "io/reconstructionworker.h"
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
    /** 按「拍摄模式」当前勾选解析出存档目录 (手动拍摄与自动采集共用同一套路由) */
    void currentSaveDirs(QString& leftDir, QString& rightDir, QString& modeName) const;
    void onUpdateLeftMat(const cv::Mat &mat);
    void onUpdateRightMat(const cv::Mat &mat);
    void onCameraFailed(int cameraIndex, const QString &reason);
    void onStartAutoCollect();
    void onPauseAutoCollect();
    void onEndAutoCollect();
    void onAutoCollectStep();
    void performAutoSave(int index, const cv::Mat& leftMat, const cv::Mat& rightMat);
    void onSerialDataReceived(const QByteArray &data);
    void onCaptureTimeout();
    void onStabilizeTimeout();  // 稳定延时结束，执行拍照保存
    void onKickoffTimeout();    // 等待相机首帧超时 → 不唤醒下位机, 直接收摊

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
    // Tab5: 重建工作线程信号
    void onReconstructionFinished();
    void onReconstructionError(const QString &message);
    void onReconProgress(int current, int total);
    void onReconLog(const QString &line);
    void onReconViewImage(int index, const QImage &left, const QImage &right);

    // ==================== Tab6: 调试与诊断 ====================
    // 均为自 Tab4/Tab5 迁移的调试工具入口 (实现在 mainwindow_tab6.cpp)
    void onRunAxisDiagnosis();              // 转台标定完整诊断 (逐帧指标 + 一致性检查)
    void onShowAxisDiagTable();             // 查看转台标定逐帧指标表
    void onExportDebugPackage();            // 导出核心调试内容 (报告/CSV/日志/异常帧图)
    void onVerifyChessboard3D();            // 单帧棋盘格 3D 验证
    void onMultiFrameChessboard3D();        // 多帧棋盘格 3D 重建
    void onReopenChessboardStats();         // 重开统计结果
    void onReopenChessboard3DView();        // 重开点云窗口
    void onOpenStandaloneViewer();          // 点云+网格 独立3D视图
    void onShowPointCloudWindowClicked();   // 仅点云 窗口
    void onPreviewSingleFrameClicked();     // 单帧 S1/S2/S3 流水线调试
    void onRefreshDebugStatus();            // 刷新前置条件状态
    void onShowCircleFitCurve();            // 弹窗: 3D圆拟合观测点 + 拟合圆 + 残差曲线
    void onShowBaIterCurve();               // 弹窗: BA 逐迭代重投影误差曲线
    void onRunQuickYieldTest();             // 前 N 帧 S1~S4 快速产出测试 (逐帧点数 + 误差分布)
    void onRunFullReconTest();              // 完整 S1~S6 重建冒烟测试
    void onExportReconDebugPackage();       // 导出重建调试包 (逐帧产出/匹配明细/点云/叠加图)

private:
    QTimer *m_stabilizeTimer;   // 稳定延时定时器
    QImage mat2QImage(const cv::Mat &mat);
    void appendDebugLog(const QString &line);   // Tab6 统一调试输出
    void updateDebugStatus();                   // 刷新 Tab6 前置条件指示
    void showAxisDiagDialog();                  // 弹出转台标定诊断窗口
    QString buildAxisDiagReport() const;        // 生成 Markdown 诊断报告
    bool exportDebugPackage(const QString& dir, QString* err);   // 导出核心调试内容
    QString buildReconTestReport() const;       // Tab6 完整重建测试的通过/不通过报告

    // ---- 重建调试包 (实现在 mainwindow_recondebug.cpp) ----
    /**
     * 样本帧重跑出来的统计 (重建调试包报告用)。
     * 「几何自洽」= 双目 DLT 三角化点到光平面的距离 ≤5mm —— 这个量与 S3 的判据无关,
     * 所以能拿来反查判据本身是否合理。
     */
    struct ReconSampleStat {
        int    frameIdx = -1;
        double angleDeg = 0.0;
        int    s1L = 0, s1R = 0, s2 = 0, s3 = 0;
        int    nAccepted = 0;      // S3 实际接受的点数
        int    nMatch    = 0;      // DLT 求解成功的匹配数 (残差统计的分母)
        int    nGood     = 0;      // 其中几何自洽(≤5mm)的
        double residMedianMm = -1.0;
        bool   ran = false;
    };
    QString buildReconParamsCsv() const;        // 重建参数 + 关键标定量快照 (两个调试包共用)
    // imageHeight: 原图高度, 用来判断"提取点是否铺满了整幅图"(背景误提取的特征); 0 = 未知
    QString buildReconDiagReport(const QVector<ReconSampleStat>& samples, int imageHeight) const;
    /**
     * 导出重建调试包。
     * progress: 可选的进度回调 (阶段名, 0-100)。导出要重跑样本帧、写逐视角点云、编码叠加图,
     *           整段要几秒, 没有回调时界面只能干等。签名与 RotatingCalibrator::ProgressFn 一致。
     */
    bool exportReconDebugPackage(const QString& dir, QString* err,
                                 const std::function<void(const QString&, int)>& progress = nullptr);
    // 重建作业组装与运行态 (Tab5/Tab6 共用, 实现在 mainwindow_tab5.cpp)
    bool buildReconJob(ReconstructionWorker::JobInput& job, QString* err);
    void markReconRunning();
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
    void initTab6(QTabWidget* tabWidget);   // 调试与诊断
    // 统一的"页头 + 内容"包装: [序号] 标题 —— 该页职责说明 (工业软件风格)
    QWidget* addPage(QTabWidget* tabs, QWidget* content, const QString& tabTitle,
                     const QString& stepNo, const QString& title, const QString& subtitle);
    bool saveCalibration();
    bool loadCalibration(bool showDialog = true);
    bool tryAutoLoadCalibration();
    void showCalibrationReport();
    void applyCalibAndParamsToBuilder();
    // 参数构建辅助 (供 applyCalibAndParamsToBuilder 与重建工作线程复用)
    CalibrationData buildCalibrationData() const;
    ReconstructionParams buildReconstructionParams() const;

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
    QString dirDebug;              // 调试包导出默认目录

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
    QProgressBar *progressAutoCollect;   // 自动采集进度条 (风格同 Tab5)
    QSpinBox *spinExtraShots = nullptr;  // 目标之外额外允许多采的组数 (下位机多转几格也不丢图)
    QLabel *lblAutoCollectStatus;        // 采集目标 / 进度文字提示
    QCheckBox *chkCalib;
    QCheckBox *chkLaser;
    QCheckBox *chkPlatform;
    QCheckBox *chkWorkpiece;             // 工件图 (三维重建序列), 与其余拍摄模式互斥
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

    // ---- 自动采集状态机 ----
    // 一次「收到 0x02 → 存下这一组」称为一个**窗口**。窗口一旦开就不再被后续触发重置,
    // 这是能采满组数的关键: 旧版每收到一个 0x02 就把 3s 稳定定时器(和 5s 死线)重起一遍,
    // 下位机连发触发时窗口会被无限后推、永远不落盘, 整轮可能一组都存不下来。
    bool m_isWaitingForCapture = false;   // 窗口进行中: 0x02 已收到, 图还没存
    int  m_pendingTriggers = 0;           // 窗口进行中又收到的触发, 排队等下一个窗口
    int  m_autoCollectSkipped = 0;        // 取不到新画面/存盘失败而丢掉的组数 (可见, 不静默)
    int  m_autoDupTriggers = 0;           // 重复触发计数 (下位机"多发一次触发"时能看见)
    quint64 m_leftFrameSeq = 0;           // 左相机累计出帧数 —— 用来判"这一格里有没有新画面"
    quint64 m_rightFrameSeq = 0;
    quint64 m_leftSeqAtTrigger = 0;       // 开窗口时的帧计数快照
    quint64 m_rightSeqAtTrigger = 0;
    bool m_pendingKickoff = false;        // 已点开始但还没等到首帧 → 暂不唤醒下位机
    QTimer *m_kickoffTimer = nullptr;     // 首帧等待超时

    /** 两台相机都有画面 (判断"能不能唤醒下位机") */
    bool framePairReady();
    /** 开一个采集窗口 (settleMs = 转台稳定延时; 补采排队的触发时用很短的值) */
    void startCaptureWindow(int settleMs = AUTO_SETTLE_MS);
    /** 收尾当前窗口: 计数、刷进度、把排队的触发补上 (saved=false 计入跳过) */
    void closeCaptureWindow(bool saved);
    /** 刷新进度条 + 指标行 (只放数字, 不放说明文字) */
    void updateAutoCollectProgress();
    /** 唤醒下位机 (0x01) —— 只有确认相机出图之后才允许调用 */
    void kickoffSerial();

    static constexpr int AUTO_COLLECT_TARGET = 200;  // 下位机自动采集目标次数 (进度条量程基准)
    static constexpr int AUTO_SETTLE_MS = 3000;      // 转台停稳/相机曝光稳定的延时
    static constexpr int AUTO_QUEUED_SETTLE_MS = 300; // 补采排队的触发时的短延时 (那一格早就停稳了)
    static constexpr int AUTO_WINDOW_DEADMAN_MS = 8000;  // 窗口死线 (稳定延时 3s + 5s 余量)
    static constexpr int KICKOFF_TIMEOUT_MS = 5000;      // 等待相机首帧的上限
    /**
     * 本次自动采集实际允许的组数 = 目标 + 多采余量。
     * 下位机偶尔会多转一格/多发一次触发, 旧版把进度条写死在 200 会让超出的那张
     * 看起来"没被记上"(其实图是存下来的)。进度条量程统一走这里。
     */
    int autoCollectTarget() const {
        return AUTO_COLLECT_TARGET + (spinExtraShots ? spinExtraShots->value() : 0);
    }

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
    // 立体校正**真正用的图像尺寸** (stereoRectify/initUndistortRectifyMap 的 imageSize)。
    // 必须单独存一份: P1 的主点会因 alpha=0 裁剪而不在图像正中, 拿 P1.at(0,2)*2 反推尺寸
    // 会得到 1257x714 这种偏小的值, 用它建映射表就把校正图右边/下边裁掉一截。
    cv::Size m_CalibImageSize;
    bool m_IsRectified;
    double m_StereoRms = 0.0;
    std::string m_RelativeResultStr;
    std::vector<double> m_perPairEpiErrors;  // 每对图像的校正极线Y误差
    float m_squareSize = 10.0f;              // 棋盘格方格尺寸 (mm)，默认 10mm

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
    cv::Size m_boardSize{11, 8};  // 棋盘格内角点数 (列x行)，默认 11x8
    // 光平面方程 (0,0,0,0 = 未标定)。必须显式置零:
    // 该值直接进入 S3 的"射线-光平面求交"分支判定，未初始化会导致误走上该分支
    cv::Vec4f m_LaserPlaneEquation{0.f, 0.f, 0.f, 0.f};
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
    QProgressBar *progressAxisCalib;  // 转台标定进度条 (风格同 Tab5)
    QComboBox   *comboAxisMethod = nullptr;   // Tab4 算法参数: 自动 / 圆拟合 / BA
    QDoubleSpinBox *spinBaErrorLimit = nullptr; // Tab4 算法参数: BA 重投影阈值 (px)
    QPushButton *btnAxisDiagRun;      // Tab6: 运行转台标定诊断 (原 btnDebugAxisCalib)
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
    QSpinBox *spinParallelThreads;   // 【P0-4】S1~S4 并行线程数 (0=自动)
    
    QProgressBar *progressRecon;
    QTextEdit *txtReconLog;
    QLabel *lblReconLeftView;
    QLabel *lblReconRightView;

    PointCloudBuilder* m_builder;
    PointCloudViewer* m_viewer3D;
    QStringList m_reconLeftPaths;
    QStringList m_reconRightPaths;

    // --- Tab5 重建后台线程 ---
    QThread* m_reconThread = nullptr;
    ReconstructionWorker* m_reconWorker = nullptr;
    /**
     * 上一次提交给 worker 的作业。
     *
     * 为什么要留一份: 「导出重建调试包」要对样本帧重跑 S1~S3, 而重跑必须用**同一套**标定与
     * 参数 —— 若直接调 buildCalibrationData()/buildReconstructionParams(), 拿到的是**当前
     * 界面控件值**。用户在跑完重建筑之后顺手动一下滑块(比如刚才改的重投影阈值),
     * 导出的样本就和被诊断的那一次对不上, 整包结论都会跑偏。
     */
    ReconstructionWorker::JobInput m_lastReconJob;
    bool m_hasReconJob = false;   // m_lastReconJob 是否有效
    bool m_reconRunning = false;   // 防止重建重复触发
    // 本次重建是否由 Tab6「完整重建测试」发起: 只影响收尾(出测试报告 / 抑制结果弹窗),
    // 靠这个标志而不是在 worker 上再挂一个 finished 连接 —— 后者依赖"连接顺序"这种
    // 隐性约定, 一旦有人调整 init 顺序或改用 DirectConnection 就会静默失效。
    bool m_reconSmoke = false;

    QSpinBox *spinPreviewIndex;
    QPushButton* btnPreviewSingleFrame;

    // ==================== Tab6: 调试与诊断 控件 ====================
    // ① 前置条件: 左(标定)/右(重建) 各 3 项, 每项 = 名称(带✓/✗) + 当前取值 + "去哪一页补"跳转按钮
    QVector<QLabel*> m_dbgCalibTitle, m_dbgCalibValue;   // 标定块 (供 ② 使用)
    QVector<QLabel*> m_dbgReconTitle, m_dbgReconValue;   // 重建块 (供 ③ 使用)
    QLabel *lblCalibMetrics = nullptr;      // ② 标定质量指标条 (光平面/立体RMS/极线误差/轴残差)
    QLabel *lblReconMetrics = nullptr;      // ③ 重建结果规模指标条 (点数/面数)
    QLabel *lblDbgSeqInfo = nullptr;        // ③ 调试帧号旁的"序列长度"提示
    QPushButton *btnPreviewPrev = nullptr;  // ③ 上一帧
    QPushButton *btnPreviewNext = nullptr;  // ③ 下一帧
    QPushButton *btnRefreshDebugStatus = nullptr;   // 刷新前置条件
    QCheckBox *chkDbgAutoScroll = nullptr;  // 输出区: 追加日志后自动滚动到底部
    QTextEdit *txtDebugLog = nullptr;       // 统一调试输出区
    // --- 转台标定诊断 (② 栏) ---
    QPushButton *btnAxisDiagTable = nullptr;    // 查看逐帧指标表
    QPushButton *btnAxisDiagExport = nullptr;   // 导出调试包 (诊断栏内)
    QPushButton *btnReconDebugExport = nullptr; // 导出重建调试包 (重建诊断栏内)
    QProgressBar *progressReconDebug = nullptr; // 重建调试包导出进度
    QPushButton *btnCircleFitCurve = nullptr;   // 弹窗: 3D圆拟合观测点/拟合圆/残差曲线
    QPushButton *btnBaIterCurve = nullptr;      // 弹窗: BA 逐迭代重投影误差曲线
    QPushButton *btnQuickYield = nullptr;       // ③ 快速产出测试 (前 N 帧 S1~S4)
    QPushButton *btnFullRecon = nullptr;        // ③ 完整重建测试 (S1~S6)
    QLabel      *lblAxisDiagSummary = nullptr;  // 最近一次诊断结论
    QProgressBar *progressAxisDiag = nullptr;   // 诊断进度条 (风格同 Tab5; 内部复用 process() 的进度回调)
    Calib::AxisDiagnostics m_axisDiag;          // 最近一次诊断结果 (导出用)
    QString     m_axisDiagTime;                 // 最近一次诊断时间

    // --- S1: 光条中心提取参数 (距离变换+灰度重心法) ---
    QDoubleSpinBox* spinMinConfidence;   // 【P0-3】最低点置信度 (0=不过滤)
    QDoubleSpinBox* spinMorphSigma;
    QComboBox* cmbS1Method;         // S1方法选择
    QSpinBox* spinLabA;             // 红色阈值 (LAB A通道)
    QSpinBox* spinLaserHueTol;      // 红色掩膜色相容差 (OpenCV 色相 0-180 标度)
    QSpinBox* spinLaserMinSat;      // 红色掩膜最低饱和度
    QSpinBox* spinSampleStep;       // 降采样步长
    QSpinBox *spinOverexposedThresh;  
    QSpinBox *spinMaxOverexposedGap;
    
    // --- S1 Steger 新增参数控件 ---
    QCheckBox* chkUseSteger;
    QDoubleSpinBox* spinStegerSigma;
    QDoubleSpinBox* spinStegerTMax;
    QCheckBox* chkOverexposedEnable;
    QSpinBox* spinOverexposedL;
    QSpinBox* spinSatRThresh;   // 红通道饱和判定 (激光在红通道, 削顶发生在它身上)
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
    QDoubleSpinBox *spinReprojReject;   // S3 光平面一致性阈值 (mm)
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
    QCheckBox *chkLoopClosure;        // 回环校正总开关 (默认关闭)
    QComboBox *comboLoopClosure;      // 回环校正方式: 均摊 / ICP回环 (仅在总开关勾选时有效)
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
