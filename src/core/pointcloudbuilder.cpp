#include "core/pointcloudbuilder.h"
#include "ui/logger.h"
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h> // 用于矩阵变换点云
#include <pcl/common/io.h>           // pcl::copyPointCloud
#include <pcl/kdtree/kdtree_flann.h> // KdTreeFLANN (间距估计/网格截断)
#include <pcl/registration/icp_nl.h> // 包含点到面ICP
#include <pcl/features/normal_3d_omp.h> // 包含并行法线计算，加快速度
#include <algorithm>
#include <array>    // std::array (S2 分段候选配对)
#include <atomic>   // 【P0-4】并行化后的原子标志
#include <cmath>
#include <cfloat>   // 【补全】FLT_MAX (fillBottomGaps)
#include <cstdio>   // 【补全】fprintf (S4 首帧诊断输出)
#include <cstdlib> // std::srand / std::rand (底面RANSAC)
#include <map>      // std::map (S2 阶段 3.5 按右点分组合并)
#include <set>
#include <Eigen/Eigenvalues>  // SelfAdjointEigenSolver for PCA viewpoint
#include <QDebug>
#include <QFile>    // [内存] 逐段 RSS 读数 (读 /proc/self/statm)
#include <QMutex>
#include <QMutexLocker>
#include <QSet>
#ifdef __linux__
#include <unistd.h>  // sysconf(_SC_PAGESIZE)
#endif

namespace {
/**
 * 只打印一次的终端日志。
 *
 * S1~S4 是**逐视角**跑的 (200 视角就跑 200 遍)，但"这次重建走了哪个算法分支"
 * 完全由标定/参数决定，每个视角都相同 —— 逐视角打印纯粹是刷屏。
 * 逐视角的点数明细另有去路: 已由 PointCloudBuilder::lastDetailLog 汇总,
 * 经 ReconstructionWorker::logMessage 送到 Tab5 的日志面板。
 *
 * 同样地, 逐视角路径上的告警 (缺参数/点数不一致/降级回退) 一旦成立就是每个视角
 * 都成立, 打 200 遍同样的字也不增加信息量 —— 按"每种情况报一次"处理, 但保留
 * 原来的告警级别 (走 qWarning, 依然进 stderr)。
 *
 * S1~S4 并行时本函数会被多线程并发调用，故加锁。
 */
void logOnceImpl(const char* key, const QString& msg, bool warning)
{
    static QMutex mutex;
    static QSet<QString> seen;
    QMutexLocker lock(&mutex);
    const QString k = QString::fromLatin1(key);
    if (seen.contains(k)) return;
    seen.insert(k);
    if (warning) qWarning().noquote()  << msg;
    else         qDebug().noquote()    << msg;
}
inline void qDebugOnce  (const char* key, const QString& msg) { logOnceImpl(key, msg, false); }
inline void qWarningOnce(const char* key, const QString& msg) { logOnceImpl(key, msg, true);  }

// 【P0-3】单点置信度 (0,1]: 亮度归一化 × 提取方式系数。
// 过曝恢复点 (中心由两侧边缘反推) 精度略低，按 overexposed_conf_weight 折扣。
inline float pointConfidence(float brightness, bool overexposed, float ov_weight)
{
    float base = std::min(1.0f, std::max(0.0f, brightness / 255.0f));
    return base * (overexposed ? std::max(0.0f, std::min(1.0f, ov_weight)) : 1.0f);
}

// 【P0-2】采样估计点云的中位最近邻间距 (单位与点云一致, 通常为 mm)。
// 用于自适应体素尺寸 / MLS 搜索半径；点数不足或估计失败返回 0。
double estimateMedianSpacing(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int kNeighbors = 6)
{
    if (!cloud || cloud->size() < 10) return 0.0;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    std::vector<int> nn_idx(kNeighbors);
    std::vector<float> nn_sqdist(kNeighbors);
    std::vector<float> spacings;
    const size_t max_samples = 500;
    const size_t step = std::max<size_t>(1, cloud->size() / max_samples);
    spacings.reserve(std::min(max_samples, cloud->size()));

    for (size_t i = 0; i < cloud->size(); i += step) {
        int found = tree.nearestKSearch(cloud->points[i], kNeighbors, nn_idx, nn_sqdist);
        if (found > 1) {
            float sum = 0.0f;
            for (int k = 1; k < found; ++k) sum += std::sqrt(nn_sqdist[k]);
            spacings.push_back(sum / static_cast<float>(found - 1));
        }
    }
    if (spacings.empty()) return 0.0;
    std::sort(spacings.begin(), spacings.end());
    return static_cast<double>(spacings[spacings.size() / 2]);
}
} // anonymous namespace

PointCloudBuilder::PointCloudBuilder() {
    m_global_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

PointCloudBuilder::~PointCloudBuilder() {}

void PointCloudBuilder::setCalibrationData(const CalibrationData& calib_data) {
    m_calib_data = calib_data;
}

void PointCloudBuilder::setReconstructionParams(const ReconstructionParams& params) {
    m_params = params;
}

void PointCloudBuilder::extractLaserCenter(const cv::Mat& img,
                                           const cv::Rect& roi,
                                           std::vector<cv::Point2f>& center_points,
                                           int* mask_pixels_out,
                                           std::vector<float>* confidence_out,
                                           int* mask_dropped_out,
                                           int* sat_recovered_out)
{
    center_points.clear();
    if (confidence_out) confidence_out->clear();   // 【P0-3】与 center_points 严格同步
    if (sat_recovered_out) *sat_recovered_out = 0;
    if (img.empty() || img.channels() != 3) return;

    // 1. ROI 安全处理
    cv::Rect safe_roi = (roi.width == 0 || roi.height == 0)
                            ? cv::Rect(0, 0, img.cols, img.rows) : roi;
    safe_roi &= cv::Rect(0, 0, img.cols, img.rows);
    cv::Mat roi_img = img(safe_roi);

    // 2. LAB + HSV 红光掩膜 (精简版)
    cv::Mat lab_img;
    cv::cvtColor(roi_img, lab_img, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_channels;
    cv::split(lab_img, lab_channels);
    cv::Mat l_channel = lab_channels[0];
    cv::Mat a_channel = lab_channels[1];
    // 红通道单独留一份: 激光是纯红的, 削顶发生在它身上而不是 L 上 (见 ReconstructionParams
    // 里 sat_r_thresh 的说明)。BGR 的第 2 个通道是 R。
    std::vector<cv::Mat> bgr_channels;
    cv::split(roi_img, bgr_channels);
    cv::Mat r_channel = bgr_channels[2];
    cv::Mat hsv_img;
    cv::cvtColor(roi_img, hsv_img, cv::COLOR_BGR2HSV);
    if (mask_pixels_out) *mask_pixels_out = l_channel.rows * l_channel.cols;

    // ===== 掩膜构建 (STEGER_MASK / CENTROID_MASK 共用) =====
    cv::Mat mask_a;
    bool need_mask = (m_params.s1_method == 0 || m_params.s1_method == 1);
    if (need_mask) {
        int base_a_thresh = m_params.lab_a_threshold;
        int bottom_a_thresh = std::max(60, base_a_thresh - 60);
        // 【必须按整幅图的行号定义这个剖面, 不能按 ROI 的行号】
        // 下面这条阈值斜坡的意图是: 画面上半段用严格的 A 门限, 下 35% 逐行放松到
        // bottom_a_thresh (下半场照明弱、光条暗, 补偿一下)。它是一条**图像位置**的
        // 补偿曲线, 所以自变量必须是绝对行号。
        // 原实现用 a_channel.rows (= ROI 高度) 算分割点, 于是框了 ROI 之后整条斜坡
        // 被重新锚定: 斜坡的最后一行恒等于 bottom_a_thresh (160→100), 而它在原图上的
        // 位置完全由用户随手框的框决定。实测把左图框到 538 行、右图框到 493 行后,
        // 同一行的门限从 143 掉到 100 —— 低于 LAB 中性点 128, 等于「只要不偏绿偏蓝
        // 就算红」, 光条周围的红色散斑于是成片通过掩膜, 变成噪声点。
        // 改成绝对行号后, 掩膜严格等于「整幅图掩膜 ∩ ROI」, 框 ROI 退化为纯空间裁剪。
        const int full_h = img.rows;
        const int adaptive_split = static_cast<int>(full_h * 0.65);
        mask_a = cv::Mat(a_channel.size(), CV_8U);
        for (int ry = 0; ry < a_channel.rows; ++ry) {
            uchar* mask_row = mask_a.ptr<uchar>(ry);
            const uchar* a_row = a_channel.ptr<uchar>(ry);
            const int y = ry + safe_roi.y;              // ROI 局部行号 → 整幅图行号
            int thresh = (y < adaptive_split) ? base_a_thresh
                         : base_a_thresh - static_cast<int>((base_a_thresh - bottom_a_thresh)
                             * (y - adaptive_split) / static_cast<float>(full_h - adaptive_split));
            for (int x = 0; x < a_channel.cols; ++x)
                mask_row[x] = (a_row[x] > thresh) ? 255 : 0;
        }
        // 色相窗/饱和度门限由参数给出 (见 ReconstructionParams::laser_hue_tol_deg 的说明):
        // 红色在 OpenCV 的 0-180 色相标度上首尾相接, 所以窗口必须写成 [0,tol] ∪ [180-tol,180]
        // 两段再取并; 原来写死的 [0,10]∪[170,180] 只覆盖 20° 宽的红色,
        // 亮起来后色相漂到 160° 左右的光条会被整段判掉。
        const int hueTol = std::min(90, std::max(0, m_params.laser_hue_tol_deg));
        const int satMin = std::max(0, std::min(255, m_params.laser_min_sat));
        cv::Mat mask_hsv_low, mask_hsv_high, mask_hsv;
        cv::inRange(hsv_img, cv::Scalar(0, satMin, 20), cv::Scalar(hueTol, 255, 255), mask_hsv_low);
        cv::inRange(hsv_img, cv::Scalar(180 - hueTol, satMin, 20), cv::Scalar(180, 255, 255), mask_hsv_high);
        cv::bitwise_or(mask_hsv_low, mask_hsv_high, mask_hsv);
        cv::bitwise_and(mask_a, mask_hsv, mask_a);
        cv::Mat strict_dilated;
        cv::dilate(mask_a, strict_dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)));
        cv::Mat mask_bright, mask_hsv_b2, mask_hsv_bl, mask_hsv_bh;
        cv::inRange(l_channel, 160, 255, mask_bright);
        cv::inRange(hsv_img, cv::Scalar(0, satMin, 120), cv::Scalar(hueTol, 255, 255), mask_hsv_bl);
        cv::inRange(hsv_img, cv::Scalar(180 - hueTol, satMin, 120), cv::Scalar(180, 255, 255), mask_hsv_bh);
        cv::bitwise_or(mask_hsv_bl, mask_hsv_bh, mask_hsv_b2);
        cv::bitwise_and(mask_bright, mask_hsv_b2, mask_bright);
        cv::bitwise_and(mask_bright, strict_dilated, mask_bright);
        cv::bitwise_or(mask_a, mask_bright, mask_a);
        cv::dilate(mask_a, mask_a, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        // ===== 【伪光条剔除】只留下"够长"的连通域 =====
        // 为什么必须在掩膜这一层做: Steger 的非极大值抑制是**纯 3×3 局部**的 ——
        // 一个孤立亮斑在自己邻域里天然就是极大值, 所以它**必然输出候选点**,
        // 任何逐点阈值 (包括"最低点置信度") 都拦不住。唯一的拦截点就是掩膜。
        //
        // 伪连通域从哪来 (探针实测, 本序列 212 帧逐帧可见):
        //   · 盒子顶边的**镜面反光斑** —— 位置固定、面积逐帧变 (18~303 px)
        //   · 背景里静止的红色物件 (框了 ROI 之后多半已被挡在外面)
        // 它们都能通过红色掩膜 + A 通道阈值, 于是被当成光条提取。
        // 危险之处在于它们**不跟着转台转**, 而 S5 只会按 i×步长 整体旋转每一帧 ——
        // 于是这些同一小片像素, 在成品点云里被扫成一圈稀疏的点。
        //
        // 判据用"长边": 激光光条是一条**贯穿被测物的长脊**, 反光斑只是一小团。
        // 本序列实测: 主光条长边 p50 = 255px, 伪斑 p50 = 22px / p90 = 39px, 分得很开。
        // 阈值写成"相对最大连通域"+"绝对下限"两条:
        //   · 相对量保证主光条永远留得下, 且被遮挡合法截断的光条段
        //     (通常仍有主光条的 25% 以上) 不会被误伤;
        //   · 绝对下限 (24px) 防止某帧主光条本身就很短时, 相对阈值被一起拖没。
        // 这两个量都**不含任何图像位置依赖** —— 不重蹈 adaptive_split 按 ROI 行号
        // 锚定的覆辙 (见上面那段说明)。
        int dropped = 0;
        if (cv::countNonZero(mask_a) > 0) {
            cv::Mat labels, stats, centroids;
            const int n = cv::connectedComponentsWithStats(mask_a, labels, stats, centroids, 8);
            int longest = 0;
            for (int i = 1; i < n; ++i)
                longest = std::max(longest, std::max(stats.at<int>(i, cv::CC_STAT_WIDTH),
                                                     stats.at<int>(i, cv::CC_STAT_HEIGHT)));
            const int keep_min = std::max(24, static_cast<int>(longest * 0.25));
            for (int i = 1; i < n; ++i) {
                const int span = std::max(stats.at<int>(i, cv::CC_STAT_WIDTH),
                                          stats.at<int>(i, cv::CC_STAT_HEIGHT));
                if (span < keep_min) {
                    dropped += stats.at<int>(i, cv::CC_STAT_AREA);
                    mask_a.setTo(0, labels == i);
                }
            }
        }
        if (mask_pixels_out) *mask_pixels_out = cv::countNonZero(mask_a);
        if (mask_dropped_out) *mask_dropped_out = dropped;
    }

    const int w = l_channel.cols, h = l_channel.rows;

    // ===== 方法选择 =====
    if (m_params.s1_method == 0) {
        // -------- Steger + 掩膜 (实验3) --------
        const float sigma = std::max(0.5f, m_params.steger_sigma);
        int ksize = cvRound(sigma * 3.0f) * 2 + 1;
        if (ksize % 2 == 0) ksize += 1;
        cv::Mat blurred;
        cv::GaussianBlur(l_channel, blurred, cv::Size(ksize, ksize), sigma, sigma);
        cv::Mat dx, dy, dxx, dyy, dxy;
        cv::Sobel(blurred, dx, CV_32F, 1, 0, 3);
        cv::Sobel(blurred, dy, CV_32F, 0, 1, 3);
        cv::Sobel(dx, dxx, CV_32F, 1, 0, 3);
        cv::Sobel(dy, dyy, CV_32F, 0, 1, 3);
        cv::Sobel(dx, dxy, CV_32F, 0, 1, 3);
        cv::Mat resp(h, w, CV_32F, cv::Scalar(0));
        cv::Mat smx(h, w, CV_32F, cv::Scalar(0));
        cv::Mat smy(h, w, CV_32F, cv::Scalar(0));
        cv::Mat ovx(h, w, CV_8U, cv::Scalar(0));   // 【P0-3】该格最优解是否来自过曝恢复
        const float t_max = std::max(0.1f, m_params.steger_t_max);
        const int edge_dark_thresh = m_params.steger_edge_dark_thresh;   // 【P0-3】原硬编码 200
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                if (mask_a.at<uchar>(y, x) == 0) continue;
                float gx = dx.at<float>(y, x), gy = dy.at<float>(y, x);
                float l_px = static_cast<float>(l_channel.at<uchar>(y, x));
                float cpx, cpy; bool ok = false;
                bool from_overexposed = false;   // 【P0-3】置信度折扣依据
                // 过曝/饱和判定必须**分通道**:
                //   · L 削顶 (l_px > L 阈值) —— 白热区, 按 L 找边缘;
                //   · 红通道削顶 (r_px ≥ R 阈值) —— 红激光削顶而 L 不高的情形,
                //     此时 L 在平台区是被绿蓝通道(环境光/蓝色转台反光)牵着走的,
                //     用它找边缘会走到环境光上, 必须改用红通道。
                const int r_px = static_cast<int>(r_channel.at<uchar>(y, x));
                const bool l_clip = l_px > static_cast<float>(m_params.steger_overexposed_l_thresh);
                const bool r_clip = (r_px >= m_params.sat_r_thresh);
                if (m_params.steger_edge_offset_enable && (l_clip || r_clip)) {
                    // 哪条通道削顶, 就用哪条通道找边缘 (阈值同量纲)
                    const cv::Mat& edge_ch = r_clip ? r_channel : l_channel;
                    const int edge_thr = r_clip ? m_params.edge_dark_r_thresh : edge_dark_thresh;
                    float gn = std::sqrt(gx*gx+gy*gy);
                    if (gn > 0.5f) {
                        // 搜索半径: 红通道削顶时要一直走到"红通道掉到边缘阈值"为止,
                        // 平台半宽实测可达 8~10px, 用原来的 sigma*6 (=7.2) 会走不到边、
                        // 退化成单边外推。红通道的剖面从 250+ 单调降到背景,
                        // 向外第一个 <128 的穿越点必然是光条自己的边, 放宽不会串到别处。
                        float nxg=gx/gn, nyg=gy/gn;
                        float ms = r_clip ? 20.0f : std::max(3.0f, sigma*6.0f);
                        float ld=-1, rd=-1;
                        for (float d=0.5f; d<=ms && (ld<0||rd<0); d+=0.5f) {
                            if (ld<0) { int ex=cvRound(x-d*nxg), ey=cvRound(y-d*nyg);
                                if (ex>=0&&ex<w&&ey>=0&&ey<h&&edge_ch.at<uchar>(ey,ex)<edge_thr) ld=d; }
                            if (rd<0) { int ex=cvRound(x+d*nxg), ey=cvRound(y+d*nyg);
                                if (ex>=0&&ex<w&&ey>=0&&ey<h&&edge_ch.at<uchar>(ey,ex)<edge_thr) rd=d; }
                        }
                        if (ld>0&&rd>0) { float hw=(ld+rd)*0.5f;
                            if (hw>=1.5f&&hw<=12.0f) { cpx=x+(rd-ld)*0.5f*nxg; cpy=y+(rd-ld)*0.5f*nyg; ok=true; } }
                        else if (ld>0) { cpx=x-(ld+3.0f)*nxg; cpy=y-(ld+3.0f)*nyg; ok=true; }
                        else if (rd>0) { cpx=x+(rd+3.0f)*nxg; cpy=y+(rd+3.0f)*nyg; ok=true; }
                    }
                    if (ok) { from_overexposed = true; goto ste_rec; }
                }
                {
                    float gxx=dxx.at<float>(y,x), gyy=dyy.at<float>(y,x), gxy=dxy.at<float>(y,x);
                    float trace=gxx+gyy, det=gxx*gyy-gxy*gxy;
                    if (det<=0||trace>=0) continue;
                    float sq=std::sqrt(std::max(0.0f, trace*trace-4.0f*det));
                    float lambda1=(trace-sq)*0.5f;
                    if (std::fabs(lambda1)<1e-4f) continue;
                    if (lambda1>-2.0f) continue;
                    float nxa=gxy, nya=lambda1-gxx, nxb=lambda1-gyy, nyb=gxy;
                    float na=nxa*nxa+nya*nya, nb=nxb*nxb+nyb*nyb;
                    float nx=(na>=nb)?nxa:nxb, ny=(na>=nb)?nya:nyb;
                    float nrm=std::sqrt(nx*nx+ny*ny); if (nrm<1e-6f) continue;
                    nx/=nrm; ny/=nrm;
                    float denom=gxx*nx*nx+2.0f*gxy*nx*ny+gyy*ny*ny;
                    if (std::fabs(denom)<1e-6f) continue;
                    float t=-(gx*nx+gy*ny)/denom;
                    if (std::fabs(t)>t_max) continue;
                    cpx=x+t*nx; cpy=y+t*ny; ok=true;
                }
                if (!ok) continue;
                ste_rec:
                float px=cpx+safe_roi.x, py=cpy+safe_roi.y;
                if (l_px>resp.at<float>(y,x)) {
                    resp.at<float>(y,x)=l_px; smx.at<float>(y,x)=px; smy.at<float>(y,x)=py;
                    ovx.at<uchar>(y,x) = from_overexposed ? 1 : 0;
                }
            }
        }
        for (int y=1; y<h-1; ++y) for (int x=1; x<w-1; ++x) {
            float v=resp.at<float>(y,x); if (v<1e-4f) continue;
            bool im=true;
            for (int dy=-1;dy<=1&&im;++dy) for (int dx=-1;dx<=1&&im;++dx)
                if ((dx||dy)&&resp.at<float>(y+dy,x+dx)>v) im=false;
            if (im) {
                center_points.emplace_back(smx.at<float>(y,x), smy.at<float>(y,x));
                const bool ov = (ovx.at<uchar>(y,x) != 0);
                if (ov && sat_recovered_out) ++(*sat_recovered_out);
                if (confidence_out) {
                    // 【P0-3】置信度 = 亮度归一化 × 提取方式系数
                    confidence_out->push_back(
                        pointConfidence(v, ov, m_params.overexposed_conf_weight));
                }
            }
        }

    } else if (m_params.s1_method == 1) {
        // -------- 灰度重心 + 掩膜 --------
        std::vector<int> rough_y(w, -1);
        std::vector<uchar> peak_l(w, 0), peak_r(w, 0);
        for (int x = 1; x < w - 1; ++x) {
            uchar max_l = 0, max_r = 0; int max_y = -1;
            for (int y = 1; y < h - 1; ++y) {
                if (mask_a.at<uchar>(y, x) == 0) continue;
                uchar v = l_channel.at<uchar>(y, x);
                if (v > max_l) { max_l = v; max_y = y; max_r = r_channel.at<uchar>(y, x); }
            }
            if (max_y >= 0) { rough_y[x] = max_y; peak_l[x] = max_l; peak_r[x] = max_r; }
        }
        for (int x = 1; x < w - 1; ++x) {
            if (rough_y[x] < 0) continue;
            int cy = rough_y[x]; float sx = static_cast<float>(x), sy;
            bool from_overexposed = false;
            // 饱和判定分通道: 红通道削顶而 L 不高时, 必须按红通道找边缘 (见 sat_r_thresh)
            const bool r_clip = (peak_r[x] >= m_params.sat_r_thresh);
            if ((peak_l[x] >= m_params.centroid_overexposed_l || r_clip)
                && m_params.steger_edge_offset_enable) {  // 【P0-3】原硬编码250
                const cv::Mat& edge_ch = r_clip ? r_channel : l_channel;
                const int dark = r_clip ? m_params.edge_dark_r_thresh
                                        : m_params.centroid_edge_dark_thresh;  // 【P0-3】原硬编码200
                int lx=x, rx=x; while (lx>0 && edge_ch.at<uchar>(cy,lx)>dark) --lx;
                while (rx<w-1 && edge_ch.at<uchar>(cy,rx)>dark) ++rx;
                int ty=cy, by=cy; while (ty>0 && edge_ch.at<uchar>(ty,x)>dark) --ty;
                while (by<h-1 && edge_ch.at<uchar>(by,x)>dark) ++by;
                int hx=(rx-lx)/2, hy=(by-ty)/2;
                if (hx>=2&&hx<=12&&hy>=2&&hy<=12) {
                    sx=float(lx+rx)*0.5f; sy=float(ty+by)*0.5f; from_overexposed = true;
                } else {
                    sy=float(cy);
                }
            } else {
                int y0=std::max(4,cy-8), y1=std::min(h-5,cy+8);
                float sl=0,sly=0;
                for (int yy=y0; yy<=y1; ++yy) {
                    if (mask_a.at<uchar>(yy,x)==0) continue;
                    float v=float(l_channel.at<uchar>(yy,x)); sl+=v; sly+=v*yy;
                }
                sy=(sl>0)?sly/sl:float(cy);
            }
            center_points.emplace_back(sx+safe_roi.x, sy+safe_roi.y);
            if (from_overexposed && sat_recovered_out) ++(*sat_recovered_out);
            if (confidence_out) {
                confidence_out->push_back(pointConfidence(static_cast<float>(peak_l[x]),
                                                          from_overexposed,
                                                          m_params.overexposed_conf_weight));
            }
        }

    } else {
        // -------- 纯列极值 (无掩膜) --------
        for (int x = 1; x < w - 1; ++x) {
            uchar max_l = 0, max_r = 0; int max_y = -1;
            for (int y = 1; y < h - 1; ++y) {
                uchar v = l_channel.at<uchar>(y, x);
                if (v > max_l) { max_l = v; max_y = y; max_r = r_channel.at<uchar>(y, x); }
            }
            // 【P0-3】最小峰值亮度改为参数 (原硬编码 70)
            if (max_y < 0 || max_l < m_params.column_peak_min_l) continue;
            float sx = static_cast<float>(x), sy;
            bool from_overexposed = false;
            const bool r_clip = (max_r >= m_params.sat_r_thresh);
            if ((max_l >= m_params.centroid_overexposed_l || r_clip)
                && m_params.steger_edge_offset_enable) {  // 【P0-3】原硬编码250
                const cv::Mat& edge_ch = r_clip ? r_channel : l_channel;
                const int dark = r_clip ? m_params.edge_dark_r_thresh
                                        : m_params.centroid_edge_dark_thresh;  // 【P0-3】原硬编码200
                int lx=x, rx=x; while (lx>0 && edge_ch.at<uchar>(max_y,lx)>dark) --lx;
                while (rx<w-1 && edge_ch.at<uchar>(max_y,rx)>dark) ++rx;
                int ty=max_y, by=max_y; while (ty>0 && edge_ch.at<uchar>(ty,x)>dark) --ty;
                while (by<h-1 && edge_ch.at<uchar>(by,x)>dark) ++by;
                sx=float(lx+rx)*0.5f; sy=float(ty+by)*0.5f;
                from_overexposed = true;
            } else {
                int y0=std::max(4,max_y-8), y1=std::min(h-5,max_y+8);
                float sl=0,sly=0;
                for (int yy=y0; yy<=y1; ++yy) {
                    float v=float(l_channel.at<uchar>(yy,x)); sl+=v; sly+=v*yy;
                }
                sy=(sl>0)?sly/sl:float(max_y);
            }
            center_points.emplace_back(sx+safe_roi.x, sy+safe_roi.y);
            if (from_overexposed && sat_recovered_out) ++(*sat_recovered_out);
            if (confidence_out) {
                confidence_out->push_back(pointConfidence(static_cast<float>(max_l),
                                                          from_overexposed,
                                                          m_params.overexposed_conf_weight));
            }
        }
    }

    // Pauta异常剔除 (所有方法共用)
    // 【P0-3】置信度向量必须与点集同步增删，否则下游按索引取置信度会错位
    if (center_points.size() > 20) {
        std::vector<float> local_dy;
        for (size_t i = 1; i + 1 < center_points.size(); ++i)
            local_dy.push_back(std::fabs(center_points[i].y - (center_points[i-1].y + center_points[i+1].y) * 0.5f));
        if (!local_dy.empty()) {
            float mean = 0, sq_sum = 0;
            for (float d : local_dy) { mean += d; sq_sum += d * d; }
            mean /= local_dy.size();
            float stdv = std::sqrt(std::max(0.0f, sq_sum / local_dy.size() - mean * mean));
            float thresh = mean + 3.0f * stdv;
            const bool has_conf = (confidence_out && confidence_out->size() == center_points.size());
            std::vector<cv::Point2f> filtered;
            std::vector<float> filtered_conf;
            filtered.push_back(center_points.front());
            if (has_conf) filtered_conf.push_back((*confidence_out)[0]);
            for (size_t i = 1; i + 1 < center_points.size(); ++i) {
                if (local_dy[i-1] <= thresh) {
                    filtered.push_back(center_points[i]);
                    if (has_conf) filtered_conf.push_back((*confidence_out)[i]);
                }
            }
            filtered.push_back(center_points.back());
            if (has_conf) filtered_conf.push_back(confidence_out->back());
            center_points.swap(filtered);
            if (has_conf) confidence_out->swap(filtered_conf);
        }
    }
    return;
}

// ======================== S2: 极线约束匹配 (视差跳变切断 + 分段 DP) ========================
void PointCloudBuilder::epipolarConstraintMatch(
    const std::vector<cv::Point2f>& pts_left,
    const std::vector<cv::Point2f>& pts_right,
    std::vector<cv::Point2f>& matched_pts_left,
    std::vector<cv::Point2f>& matched_pts_right)
{
    matched_pts_left.clear();
    matched_pts_right.clear();
    if (pts_left.empty() || pts_right.empty()) return;

    // --------------------------------------------------
    // 校正后模式：视差跳变切断 + 分段独立 DP 匹配
    // --------------------------------------------------
    if (m_calib_data.is_rectified) {
        qDebugOnce("s2_rect", "====== [S2 极线匹配] 模式: 视差跳变切断 + 分段 DP ======");

        std::vector<cv::Point2f> L = pts_left;
        std::vector<cv::Point2f> R = pts_right;
        std::sort(L.begin(), L.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        std::sort(R.begin(), R.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

        const int M = L.size();
        const int N = R.size();
        const double search_range_y = std::max(5.0, m_params.epipolar_threshold * 3.0);
        const double skip_penalty  = m_params.dp_skip_penalty;

        float min_x = L[0].x, max_x = L[0].x;
        for (const auto& p : L) { if (p.x < min_x) min_x = p.x; if (p.x > max_x) max_x = p.x; }
        for (const auto& p : R) { if (p.x < min_x) min_x = p.x; if (p.x > max_x) max_x = p.x; }
        const float span_x = std::max(50.0f, max_x - min_x);

        // ========================================================================
        // 【阶段 1】：粗匹配以计算视差梯度，寻找切断点 (修复版)
        // ========================================================================
        std::vector<double> rough_disp(M, NAN);
        std::vector<bool> r_used(N, false);
        
        // 工业物理约束：视差(左右图X坐标差)绝对不可能超过图像宽度的一半
        double max_physical_disp_limit = span_x * 0.8; 

        for (int i = 0; i < M; ++i) {
            double best_dy = search_range_y; 
            int best_j = -1;
            for (int j = 0; j < N; ++j) {
                if (r_used[j]) continue;
                double dy = std::fabs(L[i].y - R[j].y);
                if (dy < best_dy) { 
                    // 【核心修复】：计算视差，如果视差异常巨大，说明是乱配，直接拒绝
                    double disp = std::fabs(L[i].x - R[j].x);
                    if (disp < max_physical_disp_limit) {
                        best_dy = dy; 
                        best_j = j;
                    }
                }
            }
            if (best_j >= 0) {
                rough_disp[i] = L[i].x - R[best_j].x; // 记录合法的粗视差
                r_used[best_j] = true;
            }
        }

        // 记录切断点索引 (在 L 中的索引)
        std::vector<int> seg_starts;
        seg_starts.push_back(0);
        double break_thresh = m_params.disparity_break_threshold;
        for (int i = 1; i < M; ++i) {
            if (std::isnan(rough_disp[i]) || std::isnan(rough_disp[i-1]) ||
                std::fabs(rough_disp[i] - rough_disp[i-1]) > break_thresh) {
                if (seg_starts.back() != i) {
                    seg_starts.push_back(i);
                }
            }
        }
        // 光条分段数逐视角打印会是刷屏主力, 已由 lastDetailLog 汇总到 UI

        // ========================================================================
        // 【阶段 2】：定义分段 DP 执行器 (Lambda 函数)
        // ========================================================================
        // 各分段的候选配对先收在这里, 全部段跑完后再做**全局右点去重** (见阶段 3.5)。
        // 元素: (左全局下标, 右全局下标, 极线距离 dy)
        std::vector<std::array<int, 3>> segMatches;
        // segRidx 是**右图全局下标**的列表, 不再是连续区间:
        // 阶段 3 会把已经被前面分段用掉的右点剔掉, 保证每个右点只归一个分段。
        auto runSegmentDP = [&](const std::vector<cv::Point2f>& segL, const std::vector<int>& segRidx,
                                int lOffset) {
            int m = segL.size(), n = segRidx.size();
            if (m < 1 || n < 1) return; // 只要有点就尝试

            std::vector<std::vector<double>> dp(m + 1, std::vector<double>(n + 1, 1e18));
            std::vector<std::vector<int>> back_i(m + 1, std::vector<int>(n + 1, -1));
            std::vector<std::vector<int>> back_j(m + 1, std::vector<int>(n + 1, -1));
            std::vector<std::vector<int>> back_type(m + 1, std::vector<int>(n + 1, 0));
            std::vector<std::vector<int>> last_match(m + 1, std::vector<int>(n + 1, -1));

            dp[0][0] = 0.0;
            for (int i = 1; i <= m; ++i) { dp[i][0] = dp[i-1][0] + skip_penalty; back_i[i][0] = i-1; back_j[i][0] = 0; back_type[i][0] = 0; }
            for (int j = 1; j <= n; ++j) { dp[0][j] = dp[0][j-1] + skip_penalty; back_i[0][j] = 0; back_j[0][j] = j-1; back_type[0][j] = 1; }

            for (int i = 1; i <= m; ++i) {
                const cv::Point2f& pi = segL[i-1];
                for (int j = 1; j <= n; ++j) {
                    const cv::Point2f& pj = R[segRidx[j-1]];

                    if (dp[i-1][j] + skip_penalty < dp[i][j]) { dp[i][j] = dp[i-1][j] + skip_penalty; back_i[i][j] = i-1; back_j[i][j] = j; back_type[i][j] = 0; last_match[i][j] = last_match[i-1][j]; }
                    if (dp[i][j-1] + skip_penalty < dp[i][j]) { dp[i][j] = dp[i][j-1] + skip_penalty; back_i[i][j] = i; back_j[i][j] = j-1; back_type[i][j] = 1; last_match[i][j] = last_match[i][j-1]; }

                    const double dy = std::fabs(pi.y - pj.y);
                    if (dy <= search_range_y) {
                        double dx = std::fabs(pi.x - pj.x);
                        double match_cost = dy + 10.0 * (dx * dx) / ((span_x + 1.0) * (span_x + 1.0));
                        double smooth_cost = 0.0; 

                        double total = dp[i-1][j-1] + match_cost + smooth_cost;
                        if (total < dp[i][j]) {
                            dp[i][j] = total;
                            back_i[i][j] = i-1; back_j[i][j] = j-1; back_type[i][j] = 2;
                            last_match[i][j] = j-1;
                        }
                    }
                }
            }

            // 回溯
            std::vector<std::pair<int,int>> matches;
            int i = m, j = n;
            while (i > 0 || j > 0) {
                int type = back_type[i][j];
                if (type == 2) { matches.emplace_back(i-1, j-1); i--; j--; }
                else if (type == 1) { j--; }
                else { i--; }
            }
            std::reverse(matches.begin(), matches.end());

            for (const auto& match : matches) {
                const int li = lOffset + match.first;
                const int rj = segRidx[match.second];
                segMatches.push_back({ li, rj,
                    static_cast<int>(std::lround(std::fabs(L[li].y - R[rj].y) * 1000.0)) });
            }
        };

        // ========================================================================
        // 【阶段 3】：利用 Y 轴单调性，快速切片并执行分段 DP
        //   右点**只归一个分段**: 曾经每个分段各取一段独立的 Y 窗口, 相邻窗口重叠,
        //   同一个右点被多个分段各配一次 (实测吃掉 65~85% 的匹配对)。
        //   现在按分段先后把已用掉的右点剔出后续窗口 —— DP 在每个分段内本来就是
        //   一对一的, 加上这一条, 全局也就是一对一了。
        // ========================================================================
        std::vector<bool> r_taken(N, false);
        for (size_t s = 0; s < seg_starts.size(); ++s) {
            int start_idx = seg_starts[s];
            int end_idx = (s + 1 < seg_starts.size()) ? seg_starts[s+1] : M;

            // 【核心修复】：将 < 2 改为 < 1，宁可让 DP 自己处理短段，也不能误杀有效点
            if (end_idx - start_idx < 1) continue;

            std::vector<cv::Point2f> segL(L.begin() + start_idx, L.begin() + end_idx);

            double y_min = segL.front().y - search_range_y;
            double y_max = segL.back().y + search_range_y;

            auto it_low = std::lower_bound(R.begin(), R.end(), y_min, [](const cv::Point2f& p, double val) { return p.y < val; });
            auto it_high = std::upper_bound(R.begin(), R.end(), y_max, [](double val, const cv::Point2f& p) { return val < p.y; });

            std::vector<int> segRidx;
            for (auto it = it_low; it != it_high; ++it) {
                const int rj = static_cast<int>(it - R.begin());
                if (!r_taken[rj]) segRidx.push_back(rj);
            }

            if (!segL.empty() && !segRidx.empty()) {
                const size_t before = segMatches.size();
                runSegmentDP(segL, segRidx, start_idx);
                for (size_t k = before; k < segMatches.size(); ++k) r_taken[segMatches[k][1]] = true;
            }
        }

        // ========================================================================
        // 【阶段 3.5：候选汇总 + 强制一对一 (兜底)】
        //
        //   为什么必须在这里收口 —— 一段被删掉的旧注释写的是"交给 S3 判":
        //   那时 S3 用「光平面给的 3D 点投回右图」做判据, 同一个右点的多个候选里
        //   只有对的那个投影得上。**这个前提已经不成立了**: S3 现在用双目 DLT 出点,
        //   而 DLT 点天然落在两条射线上, "投影回右图"等于自己判自己, 拦不住任何东西
        //   (实测 33575 对候选全部通过)。于是"多个左点共用一个右点"直接进了点云。
        //
        //   它有多严重 (212 视角逐帧统计): 65~85% 的匹配对是重复的, 最极端一帧
        //   142 对匹配只用掉 88 个右点 (平均 2.4 个左点挤一个右点, 最多 7 个)。
        //   这些是**同一个物理点的多次重建**, 组内深度中位相差 2.6mm ——
        //   单目测不出、多目一叠就是几毫米厚的一层壳。
        //
        //   收口分两步: 阶段 3 让每个右点只归一个分段 (治根), 这里再兜一次底 (保底),
        //   正常情况下这里合并数应当是 0。合并取组内左点坐标的平均值 ——
        //   按"匹配后视差曲线的平滑性"评价, 平均优于"取 |dy| 最小"(最差, 印证了旧注释
        //   的判断)也优于取中位。
        // ========================================================================
        {
            std::map<int, std::vector<int>> by_right;   // 右点下标 → segMatches 下标
            for (size_t k = 0; k < segMatches.size(); ++k)
                by_right[segMatches[k][1]].push_back(static_cast<int>(k));

            std::vector<cv::Point2f> outL, outR;
            outL.reserve(by_right.size());
            outR.reserve(by_right.size());
            int dup_merged = 0;
            for (auto& kv : by_right) {
                const int rj = kv.first;
                if (kv.second.size() == 1) {
                    outL.push_back(L[segMatches[kv.second[0]][0]]);
                } else {
                    cv::Point2f sum(0.f, 0.f);
                    for (int k : kv.second) sum += L[segMatches[k][0]];
                    outL.push_back(sum * (1.0f / static_cast<float>(kv.second.size())));
                    dup_merged += static_cast<int>(kv.second.size()) - 1;
                }
                outR.push_back(R[rj]);
            }
            matched_pts_left.swap(outL);
            matched_pts_right.swap(outR);
            m_viewStats.s2DupMerged += dup_merged;   // 留痕: 收口动过手必须看得见
        }

        // (匹配前后点数逐视角打印属于刷屏; 结果已汇总进 lastDetailLog)

        // ========================================================================
        // 【阶段 4：真正的降级保底逻辑】(原代码此处为空导致返回0)
        // ========================================================================
        if (matched_pts_left.empty()) {
            qWarningOnce("s2_dp_fallback", "[S2 分段DP] 无匹配，降级为严格贪心最近邻");
            std::vector<bool> r_used_fallback(N, false);
            for (int i = 0; i < M; ++i) {
                double min_cost = 1e9;
                int best_j = -1;
                for (int j = 0; j < N; ++j) {
                    if (r_used_fallback[j]) continue;
                    // 严格限制极线距离
                    double dy = std::fabs(L[i].y - R[j].y);
                    if (dy > m_params.epipolar_threshold) continue; 
                    
                    // 综合代价评估 (Y轴权重高，X轴适度惩罚防止错配)
                    double dx = std::fabs(L[i].x - R[j].x);
                    double cost = dy * 10.0 + dx * 0.5; 
                    
                    if (cost < min_cost) {
                        min_cost = cost;
                        best_j = j;
                    }
                }
                if (best_j >= 0) {
                    matched_pts_left.push_back(L[i]);
                    matched_pts_right.push_back(R[best_j]);
                    r_used_fallback[best_j] = true;
                }
            }
        }
        return;
    }

    // ==================== 未校正模式 ====================
    qDebugOnce("s2_rawF", "====== [S2 极线匹配] 模式: 原始F矩阵 ======");
    if (m_calib_data.cameraMatrixL.empty() || m_calib_data.cameraMatrixR.empty() ||
        m_calib_data.R_stereo.empty() || m_calib_data.T_stereo.empty() ||
        m_calib_data.T_stereo.rows != 3 || m_calib_data.T_stereo.cols < 1) {
        qWarningOnce("s2_noparams", "[S2] 未校正模式缺少相机参数，无法进行极线匹配");
        return;
    }
    cv::Mat E = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat R_mat = m_calib_data.R_stereo, T = m_calib_data.T_stereo;
    E.at<double>(0, 1) = -T.at<double>(2, 0); E.at<double>(0, 2) =  T.at<double>(1, 0);
    E.at<double>(1, 0) =  T.at<double>(2, 0); E.at<double>(1, 2) = -T.at<double>(0, 0);
    E.at<double>(2, 0) = -T.at<double>(1, 0); E.at<double>(2, 1) =  T.at<double>(0, 0);
    E = E * R_mat;
    cv::Mat F = m_calib_data.cameraMatrixR.inv().t() * E * m_calib_data.cameraMatrixL.inv();
    double thresh = m_params.epipolar_threshold;

    std::vector<cv::Point2f> pts_right_sorted = pts_right;
    std::vector<size_t> sort_indices(pts_right_sorted.size());
    std::iota(sort_indices.begin(), sort_indices.end(), 0);
    std::sort(sort_indices.begin(), sort_indices.end(), [&](size_t i1, size_t i2) {
        return pts_right_sorted[i1].y < pts_right_sorted[i2].y;
    });
    std::vector<bool> is_right_matched(pts_right.size(), false);

    for (const auto& pt_l : pts_left) {
        cv::Mat p_l = (cv::Mat_<double>(3, 1) << pt_l.x, pt_l.y, 1.0);
        cv::Mat epiline = F * p_l;
        double a = epiline.at<double>(0), b = epiline.at<double>(1), c = epiline.at<double>(2);
        double denom = sqrt(a * a + b * b);
        if (denom < 1e-6) continue;

        double min_dist = thresh;
        int best_match_idx = -1; 
        auto it_start = std::lower_bound(sort_indices.begin(), sort_indices.end(), pt_l.y,
            [&](size_t idx, double val) { return pts_right_sorted[idx].y < val - thresh; });
        for (auto it = it_start; it != sort_indices.end(); ++it) {
            size_t r_idx = *it;
            double dy = pts_right_sorted[r_idx].y - pt_l.y;
            if (dy > thresh) break;
            double dist = std::fabs(a * pts_right_sorted[r_idx].x + b * pts_right_sorted[r_idx].y + c) / denom;
            if (dist < min_dist && !is_right_matched[r_idx]) {
                min_dist = dist; best_match_idx = static_cast<int>(r_idx);
            }
        }
        if (best_match_idx >= 0) {
            matched_pts_left.push_back(pt_l);
            matched_pts_right.push_back(pts_right_sorted[best_match_idx]);
            is_right_matched[best_match_idx] = true;
        }
    }
    // (逐视角匹配点数已汇总进 lastDetailLog, 不再逐视角打终端)
}

// ======================== S3: 三角化 (P0: 优先射线-光平面直接求交) ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::triangulatePoints(
    const std::vector<cv::Point2f>& pts_left,
    const std::vector<cv::Point2f>& pts_right,
    ViewDiag* diag)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pts_left.empty() || pts_right.empty()) return cloud;
    if (pts_left.size() != pts_right.size()) {   // 防御: 匹配点必须一一对应
        qWarningOnce("s3_size_mismatch", "[S3] 左右点数不一致，放弃三角化");
        return cloud;
    }

    cv::Mat P1, P2;
    bool rect_mode = false;   // P1/P2 是否来自立体校正矩阵 (决定光平面是否需要旋转到校正系)
    if (m_calib_data.is_rectified && !m_calib_data.P1_rectified.empty() && !m_calib_data.P2_rectified.empty()) {
        P1 = m_calib_data.P1_rectified;
        P2 = m_calib_data.P2_rectified;
        rect_mode = true;
        qDebugOnce("s3_rect", "====== [S3 三角化] 使用校正后P1/P2 ======");
    } else {
        if (m_calib_data.cameraMatrixL.empty() || m_calib_data.cameraMatrixR.empty() ||
            m_calib_data.R_stereo.empty() || m_calib_data.T_stereo.empty()) {
            qWarningOnce("s3_noparams", "[S3] 未校正模式缺少相机参数，无法三角化");
            return cloud;
        }
        cv::Mat K_L = m_calib_data.cameraMatrixL, K_R = m_calib_data.cameraMatrixR;
        cv::Mat R = m_calib_data.R_stereo, T = m_calib_data.T_stereo;
        P1 = (cv::Mat_<double>(3,4) << K_L.at<double>(0,0), K_L.at<double>(0,1), K_L.at<double>(0,2), 0,
                                       K_L.at<double>(1,0), K_L.at<double>(1,1), K_L.at<double>(1,2), 0,
                                       K_L.at<double>(2,0), K_L.at<double>(2,1), K_L.at<double>(2,2), 0);
        cv::Mat Rt; cv::hconcat(R, T, Rt);
        P2 = K_R * Rt;
        qDebugOnce("s3_raw", "====== [S3 三角化] 使用原始参数 ======");
    }

    // =========================================================================
    // 光平面准备: 统一到 S3 的工作坐标系
    //   Tab3 标定的光平面在【原始左相机系】; 若本函数使用校正投影矩阵 P1_rect,
    //   则点云处于校正系, 需把平面法向旋转到校正系 (纯旋转, d 不变)。
    // =========================================================================
    Eigen::Vector4d plane = m_calib_data.laser_plane_coeff;
    double plane_norm = std::sqrt(plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2]);
    bool has_plane = (plane_norm > 1e-9);
    // 仅在确实使用"校正系"P1/P2 时才把平面法向旋到校正系 (纯旋转, d 不变)。
    // 若使用原始系 K[I|0]，平面本就处于原始系，不能再旋转。
    if (has_plane && rect_mode && !m_calib_data.R_rect_L.empty()) {
        Eigen::Matrix3d R1;
        for (int r=0;r<3;r++) for (int c=0;c<3;c++) R1(r,c)=m_calib_data.R_rect_L.at<double>(r,c);
        Eigen::Vector3d n_orig(plane[0], plane[1], plane[2]);
        Eigen::Vector3d n_rect = R1 * n_orig;
        plane = Eigen::Vector4d(n_rect[0], n_rect[1], n_rect[2], plane[3]);
        plane_norm = n_rect.norm();
    }
    if (has_plane && plane_norm > 1e-9) plane /= plane_norm;

    // =========================================================================
    // 【P0-2】3D 点用双目 DLT 三角化; 光平面改作"一致性判据"
    //
    // 原来这里是"左射线 ∩ 光平面"。单目约束本身没错, 但在本装置上它是病态的:
    // 实测左射线与光平面的夹角**中位只有 12.3°、p10 只有 2.34°** —— 极度掠射。
    // 掠射时深度对平面误差的放大是 1/sin(夹角): 平面偏 1mm, 夹角 2° 处点沿射线跑
    // 28.7mm、6° 处 9.6mm、12° 处 4.8mm。而实测光平面与双目三角化点相距
    // 中位 0.99mm / p90 1.90mm / 最大 3.21mm —— 这么点误差被放大成 9~21mm 的
    // **径向铺开**, 正是"圆柱面被抹成 ~10mm 厚的环"那个伪影的成因。
    // 同一批匹配改用 DLT 后, 径向铺开降到 3.5~7mm (缩小 2~4 倍)。
    //
    // 光平面没有被丢掉, 它换了个更合适的岗位: **错配判据**。
    // DLT 点完全不依赖光平面, 所以"点到光平面的距离"是一个与三角化无关的独立真值;
    // 实测该量双峰分明 (自洽 ≤5mm、伪匹配 ≥34mm、中间是空的)。
    // 这比原来的"右图重投影一致性"硬得多: DLT 点天然落在两条射线上, 拿重投影去判
    // 等于自己判自己, 错配根本拦不住。
    //
    // plane[3] ≈ 0 (平面过光心) 时单目求交会退化到光心, 所以保留这个分支条件 ——
    // 此时走下面的纯 DLT 回退路径, 只是少一道判据。
    // =========================================================================
    cv::Mat M = P1.colRange(0, 3);
    const bool basis_ok = (std::abs(cv::determinant(M)) > 1e-12);
    const bool plane_usable = has_plane && basis_ok && std::abs(plane[3]) > 1e-3;
    // 光平面一致性阈值(mm): DLT 点到光平面的距离超过它就判为 S2 错配。
    // 实测该量双峰分明 (自洽 ≤5mm、伪匹配 ≥34mm、中间是空的), 所以 8mm 这个默认值
    // 余量很宽 —— 它只砍伪匹配, 碰不到好点。
    // 单位取 mm 而不是 px: 同一份平面误差折算到不同深度上是不同的 px 数,
    // 固定 px 阈值在 113~255mm 的工作范围内松紧会差 2.3 倍。
    const double kReprojRejectMm = m_params.reproj_reject_mm;
    // 右相机校正系的等效焦距: 只用于把 mm 残差折算回 px 喂给诊断直方图,
    // 好让这条曲线的口径与历史日志保持一致。
    const double fxRect = (P2.at<double>(0, 0) > 1e-9) ? P2.at<double>(0, 0) : 0.0;

    int valid_count = 0, neg_count = 0, too_far_count = 0, degenerate_count = 0, reject_count = 0;
    double z_min = 1e10, z_max = -1e10;

    if (plane_usable) {
        // 诊断采集: 组一条匹配明细。只在 diag 非空时被调用 (正常重建 zero-cost)。
        auto mkDiag = [&](size_t i, MatchDiag::RejectReason reason) {
            MatchDiag d;
            d.left  = pts_left[i];
            d.right = pts_right[i];
            d.dispPx = pts_left[i].x - pts_right[i].x;
            d.rejectReason = static_cast<int>(reason);
            return d;
        };
        // 批量 DLT: 一次算完所有匹配对。逐对调用要为每对分配一次 Mat, 慢且不会更准。
        const int nMatched = static_cast<int>(pts_left.size());
        cv::Mat pts4d;
        {
            cv::Mat mL(2, nMatched, CV_32F), mR(2, nMatched, CV_32F);
            for (int k = 0; k < nMatched; ++k) {
                mL.at<float>(0, k) = pts_left[k].x;  mL.at<float>(1, k) = pts_left[k].y;
                mR.at<float>(0, k) = pts_right[k].x; mR.at<float>(1, k) = pts_right[k].y;
            }
            cv::triangulatePoints(P1, P2, mL, mR, pts4d);
        }
        for (size_t i = 0; i < pts_left.size(); ++i) {
            const float w = pts4d.at<float>(3, static_cast<int>(i));
            if (std::fabs(w) < 1e-9f) {   // 两射线接近平行, DLT 无解
                ++degenerate_count;
                if (diag) diag->matches.push_back(mkDiag(i, MatchDiag::Degenerate));
                continue;
            }
            const double x = pts4d.at<float>(0, static_cast<int>(i)) / w;
            const double y = pts4d.at<float>(1, static_cast<int>(i)) / w;
            const double z = pts4d.at<float>(2, static_cast<int>(i)) / w;
            if (z <= 0) {                 // 交点在相机后方
                ++neg_count;
                if (diag) diag->matches.push_back(mkDiag(i, MatchDiag::NegativeDepth));
                continue;
            }
            if (z < z_min) z_min = z;
            if (z > z_max) z_max = z;
            if (z <= m_params.depth_min || z >= m_params.depth_max) {
                ++too_far_count;
                if (diag) { MatchDiag d = mkDiag(i, MatchDiag::TooFar); d.depthMm = z; diag->matches.push_back(d); }
                continue;
            }

            // 光平面一致性检查 (剔除 S2 错配): 量 DLT 点到光平面的距离。
            const double planeResidMm =
                std::fabs(plane[0]*x + plane[1]*y + plane[2]*z + plane[3]);
            MatchDiag md = mkDiag(i, MatchDiag::Accepted);
            md.depthMm = z;
            md.dltPlaneResidMm = planeResidMm;
            // 直方图仍按 px 口径统计: 把 mm 残差折算成"该深度处的等效重投影误差",
            // 这样逐帧曲线的口径与历史日志一致, 老数据还能对照。
            const double residPx = (fxRect > 0.0) ? planeResidMm * fxRect / z : planeResidMm;
            m_viewStats.reprojHist[ViewOutcomeStats::reprojBucket(residPx)]++;
            if (diag) { md.reprojPx = residPx; md.reprojMm = planeResidMm; }
            if (planeResidMm > kReprojRejectMm) {
                ++reject_count;
                md.rejectReason = MatchDiag::PlaneResid;
                if (diag) diag->matches.push_back(md);
                continue;
            }
            if (diag) { md.accepted = true; diag->matches.push_back(md); }
            cloud->push_back(pcl::PointXYZ(static_cast<float>(x),
                                           static_cast<float>(y),
                                           static_cast<float>(z)));
            ++valid_count;
        }
        // (原此处另跑一遍批量 DLT 只为量"点到光平面的独立真值"。现在 DLT 点**就是**
        //  3D 点、该残差**就是**判据本身, 上面循环里已经逐点算过, 这一段自然消掉。)
        // 分支由标定决定, 每视角相同 → 只报一次; 逐视角的 有效/负深/越界/平行/剔除
        // 明细属于刷屏主力, 改为累计进 m_viewStats, 由调用方在整段结束后汇总打印一次
        qDebugOnce("s3_dlt_plane", "====== [S3 三角化] 模式: 双目 DLT + 光平面一致性判据 ======");
        m_viewStats.negDepth     += neg_count;
        m_viewStats.tooFar       += too_far_count;
        m_viewStats.degenerate   += degenerate_count;
        m_viewStats.reprojReject += reject_count;
        m_viewStats.points       += valid_count;
        if (z_min < m_viewStats.depthMinMm) m_viewStats.depthMinMm = z_min;
        if (z_max > m_viewStats.depthMaxMm) m_viewStats.depthMaxMm = z_max;
        return cloud;
    }

    // =========================================================================
    // 回退路径: 双目 DLT 三角化
    //   适用: ① 未标定光平面 (如 Tab4 棋盘格3D验证, 角点不在光平面上)
    //         ② laser_plane_coeff 未初始化/全零
    //         ③ 光平面退化(过光心)
    // =========================================================================
    cv::Mat pts_left_mat  = cv::Mat(pts_left).reshape(2, 1);
    cv::Mat pts_right_mat = cv::Mat(pts_right).reshape(2, 1);

    cv::Mat pts4d;
    cv::triangulatePoints(P1, P2, pts_left_mat, pts_right_mat, pts4d);

    for (int i = 0; i < pts4d.cols; ++i) {
        float w = pts4d.at<float>(3, i);
        if (std::fabs(w) < 1e-6) { neg_count++; continue; }
        float x = pts4d.at<float>(0, i) / w;
        float y = pts4d.at<float>(1, i) / w;
        float z = pts4d.at<float>(2, i) / w;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
        if (z > m_params.depth_min && z < m_params.depth_max) {
            cloud->push_back(pcl::PointXYZ(x, y, z));
            valid_count++;
        } else if (z <= 0) {
            neg_count++;
        } else {
            too_far_count++;
        }
    }
    // 同上: 分支只报一次, 逐视角计数累计进 m_viewStats 由调用方汇总
    qDebugOnce("s3_dlt", QString("====== [S3 三角化] 模式: 双目DLT三角化 ====== %1")
                            .arg(has_plane ? "(光平面退化, 过光心)" : "(无光平面)"));
    m_viewStats.negDepth += neg_count;
    m_viewStats.tooFar   += too_far_count;
    m_viewStats.points   += valid_count;
    if (z_min < m_viewStats.depthMinMm) m_viewStats.depthMinMm = z_min;
    if (z_max > m_viewStats.depthMaxMm) m_viewStats.depthMaxMm = z_max;
    return cloud;
}

// ======================== S5: 多视角 Point-to-Plane ICP 配准 (引入旋转轴标定版) ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::multiViewRegistration(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& multi_view_clouds, 
    const std::vector<double>& view_angles_deg,
    const Eigen::Vector3f& axis_point,  // 新增：旋转轴经过的基准点 (如转台中心)
    const Eigen::Vector3f& axis_dir)    // 新增：旋转轴的单位方向向量
{
    if (multi_view_clouds.empty()) return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (multi_view_clouds.size() != view_angles_deg.size() || view_angles_deg.empty()) {
        qDebug() << "错误: 视角数量与角度数量不匹配！";
        *accumulated_cloud = *multi_view_clouds[0];
        return accumulated_cloud;
    }

    // 确保旋转轴方向向量已归一化
    Eigen::Vector3f norm_axis_dir = axis_dir.normalized();

    size_t N = multi_view_clouds.size();
    // ====== 初始化 Point-to-Plane ICP ======
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setMaxCorrespondenceDistance(m_params.icp_max_correspondence_distance);
    icp.setTransformationEpsilon(m_params.icp_translation_epsilon);
    icp.setEuclideanFitnessEpsilon(m_params.icp_euclidean_fitness_epsilon);
    icp.setMaximumIterations(m_params.icp_max_iterations);

    // 法线估计对象
    // 【P0-2】改用 OpenMP 并行版本 (点云较大时法线估计是 S5 的主要耗时项之一)
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
    normal_est.setNumberOfThreads(0);   // 0 = 使用全部可用核心
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_est.setSearchMethod(tree);
    normal_est.setKSearch(20);

    // 辅助函数：将 XYZ 点云转换为带法线的 PointNormal 点云
    auto computeNormals = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) -> pcl::PointCloud<pcl::PointNormal>::Ptr {
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normal_est.setInputCloud(cloud);
        normal_est.compute(*normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        return cloud_with_normals;
    };

    // ====== 阶段 1：相邻帧增量配准 ======
    std::vector<Eigen::Matrix4f> all_transforms(N, Eigen::Matrix4f::Identity());
    size_t first_valid_idx = 0;
    for (size_t i = 0; i < N; ++i) {
        if (!multi_view_clouds[i]->empty()) {
            first_valid_idx = i;
            break;
        }
    }

    int icp_converged = 0, icp_failed = 0;
    // 诊断: 记录首帧和每50帧的初始对齐误差
    std::vector<QString> diag_lines;
    for (size_t i = first_valid_idx + 1; i < N; ++i) {
        if (multi_view_clouds[i]->empty() || multi_view_clouds[i-1]->empty()) {
            all_transforms[i] = all_transforms[i-1];
            continue;
        }

        // 1. 计算理论旋转步长 (基于标定的任意旋转轴)
        double delta_angle_deg = view_angles_deg[i] - view_angles_deg[i-1];
        float delta_angle_rad = -static_cast<float>(delta_angle_deg * M_PI / 180.0);
        Eigen::Matrix4f T_delta_guess = Eigen::Matrix4f::Identity();

        if (fabs(delta_angle_rad) > 1e-6) {
            Eigen::AngleAxisf rotation(delta_angle_rad, norm_axis_dir);
            Eigen::Matrix3f R = rotation.toRotationMatrix();
            T_delta_guess.block<3, 3>(0, 0) = R;
            T_delta_guess.block<3, 1>(0, 3) = (Eigen::Matrix3f::Identity() - R) * axis_point;
        }

        // 2. 点云转法线
        auto curr_normals = computeNormals(multi_view_clouds[i]);
        auto prev_normals = computeNormals(multi_view_clouds[i-1]);

        // 3. 粗拼接
        pcl::PointCloud<pcl::PointNormal>::Ptr rough_aligned(new pcl::PointCloud<pcl::PointNormal>);
        pcl::transformPointCloudWithNormals(*curr_normals, *rough_aligned, T_delta_guess);

        // 【诊断】计算粗配准后的最近邻距离 (判断初始猜测质量)
        double init_mean_dist = 0;
        int init_sample = 0;
        if ((i <= first_valid_idx + 5) || (i % 50 == 0)) {
            pcl::search::KdTree<pcl::PointNormal> kdtree;
            kdtree.setInputCloud(prev_normals);
            for (const auto& pt : rough_aligned->points) {
                std::vector<int> idx(1);
                std::vector<float> dist(1);
                if (kdtree.nearestKSearch(pt, 1, idx, dist) > 0) {
                    init_mean_dist += std::sqrt(dist[0]);
                    init_sample++;
                }
            }
            if (init_sample > 0) init_mean_dist /= init_sample;
        }

        // 4. Point-to-Plane ICP 精配准
        //    关闭时**根本不跑 ICP**: 既不浪费一次逐帧迭代, 也避免日志里出现
        //    "收敛 0 / 降级 N" 这种像是"跑了但全失败"的假象 —— 它其实是"没跑"。
        const bool icpEnabled = m_params.use_icp;   // UI 可切换纯轴旋转/ICP
        Eigen::Matrix4f T_delta_final = T_delta_guess;
        Eigen::Matrix4f T_icp_delta = Eigen::Matrix4f::Identity();
        double icp_score = 0.0;
        if (icpEnabled) {
        icp.setInputSource(rough_aligned);
        icp.setInputTarget(prev_normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr icp_aligned(new pcl::PointCloud<pcl::PointNormal>);
        icp.align(*icp_aligned);
        }
        if (icpEnabled && icp.hasConverged()) {
            Eigen::Matrix4f T_icp = icp.getFinalTransformation();
            // 约束: ICP轴信任度控制绕轴旋转分量的保留比例
            // trust=100% → 完全信任轴, ICP仅修正非轴旋转+平移 (适合对称物体)
            // trust=0%   → 完全信任ICP, 不做轴约束
            Eigen::Matrix3f R_icp = T_icp.block<3,3>(0,0);
            Eigen::AngleAxisf aa_icp(R_icp);
            Eigen::Vector3f rvec_icp = aa_icp.angle() * aa_icp.axis();
            float axis_keep = 1.0f - m_params.icp_axis_trust / 100.0f; // 0% trust → keep=1.0, 100% trust → keep=0.0
            float axial = rvec_icp.dot(norm_axis_dir);
            Eigen::Vector3f rvec_constrained = rvec_icp - axial * (1.0f - axis_keep) * norm_axis_dir;
            float angle_c = rvec_constrained.norm();
            Eigen::Matrix4f T_icp_c = Eigen::Matrix4f::Identity();
            if (angle_c > 1e-6f) {
                T_icp_c.block<3,3>(0,0) = Eigen::AngleAxisf(angle_c, rvec_constrained/angle_c).toRotationMatrix();
            }
            // 平移: 同比例阻尼轴方向分量
            Eigen::Vector3f t_icp = T_icp.block<3,1>(0,3);
            float t_axial = t_icp.dot(norm_axis_dir);
            Eigen::Vector3f t_constrained = t_icp - t_axial * (1.0f - axis_keep) * norm_axis_dir;
            T_icp_c.block<3,1>(0,3) = t_constrained;
            T_delta_final = T_icp_c * T_delta_guess;
            ++icp_converged;
            icp_score = icp.getFitnessScore();
            T_icp_delta  = T_icp;
        } else if (icpEnabled) {
            T_delta_final = T_delta_guess;
            ++icp_failed;
        }
        all_transforms[i] = all_transforms[i-1] * T_delta_final;

        // 【诊断】记录采样帧。ICP 关闭时不留这些行 —— 全部是 ICP 量, 报出来只会
        // 让人以为"ICP 跑了但失败了"; 关闭状态下真正该看的只有融合点数。
        if (!init_sample || !icpEnabled) continue;
        Eigen::AngleAxisf aa(T_icp_delta.block<3,3>(0,0));
        float icp_rot_deg = aa.angle() * 180.0 / M_PI;
        float icp_trans = T_icp_delta.block<3,1>(0,3).norm();
        bool would_accept = (icp_rot_deg < 2.0 && icp_trans < 2.0);
        // 【S5诊断】首5帧输出理论vs实际平移量
        if (i <= first_valid_idx + 5) {
            Eigen::Vector3f t_theory = T_delta_guess.block<3,1>(0,3);
            Eigen::Vector3f t_actual = T_delta_final.block<3,1>(0,3);
            diag_lines.push_back(QString("  [S5平移] 帧%1→%2 理论t:[%3,%4,%5] 实际t:[%6,%7,%8]")
                .arg(i-1).arg(i)
                .arg(t_theory.x(),0,'f',1).arg(t_theory.y(),0,'f',1).arg(t_theory.z(),0,'f',1)
                .arg(t_actual.x(),0,'f',1).arg(t_actual.y(),0,'f',1).arg(t_actual.z(),0,'f',1));
        }
        // 仅在前5帧和每50帧输出
        if (i <= first_valid_idx + 5 || i % 50 == 0) {
            diag_lines.push_back(QString("  [%1→%2] 粗配准均值距离:%3 mm | ICP收敛:%4 score:%5 rot:%6° t:%7 mm %8")
                .arg(i-1).arg(i)
                .arg(init_mean_dist, 0, 'f', 2)
                .arg(icp.hasConverged() ? "是" : "否")
                .arg(icp_score, 0, 'f', 4)
                .arg(icp_rot_deg, 0, 'f', 2)
                .arg(icp_trans, 0, 'f', 2)
                .arg(would_accept ? "✅" : ""));
        }
    }

    // ====== 阶段 2：闭环误差检测与沿标定轴均摊 ======
    size_t last_valid_idx = 0;
    for (int i = N - 1; i >= 1; --i) {
        if (!multi_view_clouds[i]->empty()) {
            last_valid_idx = i;
            break;
        }
    }

    if (last_valid_idx > first_valid_idx + 10) {
        auto last_normals = computeNormals(multi_view_clouds[last_valid_idx]);
        auto first_normals = computeNormals(multi_view_clouds[first_valid_idx]);

        pcl::PointCloud<pcl::PointNormal>::Ptr last_mapped(new pcl::PointCloud<pcl::PointNormal>);
        pcl::transformPointCloudWithNormals(*last_normals, *last_mapped, all_transforms[last_valid_idx]);

        icp.setInputSource(last_mapped);
        icp.setInputTarget(first_normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr loop_closed(new pcl::PointCloud<pcl::PointNormal>);
        icp.align(*loop_closed);

        // 始终计算闭环误差（即使ICP不收敛），用于诊断轴标定质量
        Eigen::Matrix4f loop_error = icp.getFinalTransformation();
        Eigen::Matrix3f R_err_mat = loop_error.block<3,3>(0,0);
        Eigen::Vector3f t_err = loop_error.block<3,1>(0,3);
        float rot_err_deg = Eigen::AngleAxisf(R_err_mat).angle() * 180.0 / M_PI;

        // 同时计算增量累加的终点与理论360°的偏差
        float total_inc_angle = Eigen::AngleAxisf(
            all_transforms[last_valid_idx].block<3,3>(0,0)).angle() * 180.0 / M_PI;
        Eigen::Vector3f total_inc_trans = all_transforms[last_valid_idx].block<3,1>(0,3);

        diag_lines.push_back(QString("  [闭环诊断] 增量累计rot:%1° t:[%2,%3,%4]mm →全量均摊(每帧-%5°)")
            .arg(total_inc_angle, 0, 'f', 2)
            .arg(total_inc_trans.x(), 0, 'f', 1)
            .arg(total_inc_trans.y(), 0, 'f', 1)
            .arg(total_inc_trans.z(), 0, 'f', 1)
            .arg(total_inc_angle / (last_valid_idx - first_valid_idx), 0, 'f', 3));

        // 三种回环方式 (见 ReconstructionParams::LoopClosure):
        //   Spread: T_err = all_transforms[last]⁻¹  —— 用增量链自身的累计量作为漂移
        //   Icp   : T_err = loop_error              —— 用末帧↔首帧的点云配准量作为漂移
        //   Off   : 完全跳过
        //
        // 关于 ICP 的取值: loop_error 满足 loop_error·(all_transforms[last]·末帧点云) ≈ 首帧点云,
        // 所以 loop_error·all_transforms[last] 是 ICP 拟合出的"末帧→首帧"总变换, 它应当等于 I。
        // 换言之 loop_error 本身就是所需的修正量 (与 Spread 的 all_transforms[last]⁻¹ 同量纲,
        // 二者在 ICP 收敛良好时一致)。**不能**再取 (loop_error·all_transforms[last])⁻¹ ——
        // 那个量恒 ≈ I⁻¹ = I, 等于什么都没修正。
        // 注意 Spread 正是 Icp 在 loop_error == I (闭合完美) 时的特例, 两者量纲一致:
        // 都是"作用在首帧坐标系、施于左侧"的修正量, ratio=1 时把末帧打到恒等。
        Eigen::Matrix4f T_err = Eigen::Matrix4f::Identity();
        bool doCorrect = (m_params.loop_closure != ReconstructionParams::LoopClosureOff);
        const bool useIcp = (m_params.loop_closure == ReconstructionParams::LoopClosureIcp);

        if (doCorrect) {
            // ICP 只在其确实收敛时才可信 (点云太少/重叠不足时会返回无意义变换),
            // 否则退回增量链自身的估计。
            const bool icp_ok = useIcp && icp.hasConverged();
            if (icp_ok) {
                T_err = loop_error;
            } else {
                if (useIcp)
                    qDebug() << "====== [闭环优化] ICP 未收敛，回退增量均摊 ======";
                T_err = all_transforms[last_valid_idx].inverse();
            }

            Eigen::Matrix3f R_err_final = T_err.block<3,3>(0,0);
            Eigen::Vector3f t_err_final = T_err.block<3,1>(0,3);
            Eigen::AngleAxisf err_aa(R_err_final);
            Eigen::Vector3f rvec_err = err_aa.angle() * err_aa.axis();

            for (size_t i = first_valid_idx + 1; i <= last_valid_idx; ++i) {
                float ratio = static_cast<float>(i - first_valid_idx)
                            / static_cast<float>(last_valid_idx - first_valid_idx);

                Eigen::Vector3f rvec_frac = rvec_err * ratio;
                float angle = rvec_frac.norm();
                Eigen::Matrix3f R_correction = (angle < 1e-6f)
                    ? Eigen::Matrix3f::Identity()
                    : Eigen::AngleAxisf(angle, rvec_frac.normalized()).toRotationMatrix();
                Eigen::Vector3f t_correction = t_err_final * ratio;

                Eigen::Matrix4f T_correction = Eigen::Matrix4f::Identity();
                T_correction.block<3, 3>(0, 0) = R_correction;
                T_correction.block<3, 1>(0, 3) = t_correction;

                all_transforms[i] = T_correction * all_transforms[i];
            }
            float final_rot = Eigen::AngleAxisf(R_err_final).angle() * 180.0 / M_PI;
            qDebug() << "====== [闭环优化]" << (useIcp ? "(ICP)" : "(增量均摊)")
                     << "旋转误差:" << final_rot << "° 平移误差:" << t_err_final.norm()
                     << "mm，已全量SE(3)均摊 ======";
        } else {
            qDebug() << "====== [闭环优化] 已关闭，不做回环校正 ======";
        }
    }

    // ====== 阶段 3：应用变换，融合点云 ======
    *accumulated_cloud = *multi_view_clouds[first_valid_idx];
    for (size_t i = first_valid_idx + 1; i < N; ++i) {
        if (multi_view_clouds[i]->empty()) continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*multi_view_clouds[i], *transformed_cloud, all_transforms[i]);
        *accumulated_cloud += *transformed_cloud;
    }
    qDebug() << "====== [S5 结束] Point-to-Plane配准完成，点云总数:" << accumulated_cloud->size() << " ======";
    if (m_params.use_icp) {
        lastDetailLog = QString("[S5 ICP] 相邻帧: 收敛 %1 / 降级 %2 | max_correspondence_distance: %3 mm | 融合点云: %4")
            .arg(icp_converged).arg(icp_failed).arg(m_params.icp_max_correspondence_distance).arg(accumulated_cloud->size());
    } else {
        // 关闭 ICP 时旧文案是"相邻帧: 收敛 0 / 降级 211", 看起来像 ICP 跑了 211 次全失败,
        // 实际一次都没跑 —— 纯轴旋转拼接本就是这个模式的正常行为, 不是降级。
        lastDetailLog = QString("[S5 纯轴旋转] ICP 已关闭: 各视角按「视角序号 × 步长」绕标定轴直接拼接, 不做数据驱动修正 | 融合点云: %1")
            .arg(accumulated_cloud->size());
    }
    if (!diag_lines.empty()) {
        lastDetailLog += "\n[S5 诊断] 采样帧初始对齐 (粗配准均值距离=初始猜测质量, ICP收敛=Yes则精配准成功, rot/t=ICP修正量):";
        for (const auto& line : diag_lines)
            lastDetailLog += "\n" + line;
    }
    return accumulated_cloud;
}


// ======================== S6: 滤波与网格化 (架构重构：泊松替换为贪婪投影) ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::denoiseAndFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *filtered_cloud = *input_cloud;
    if (filtered_cloud->empty()) return filtered_cloud;

    if (filtered_cloud->size() > static_cast<size_t>(m_params.sor_mean_k)) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(m_params.sor_mean_k);
        sor.setStddevMulThresh(m_params.sor_std_dev_mul);
        sor.filter(*filtered_cloud);
    }
    if (m_params.voxel_leaf_size > 0.001) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(m_params.voxel_leaf_size, m_params.voxel_leaf_size, m_params.voxel_leaf_size);
        voxel.setInputCloud(filtered_cloud);
        voxel.filter(*filtered_cloud);
    }
    return filtered_cloud;
}

pcl::PolygonMesh PointCloudBuilder::meshReconstruction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {

    pcl::PolygonMesh mesh;
    if (filtered_cloud->size() < 50) return mesh;

    // 0. 密度均匀化：泊松重建对非均匀采样敏感，多视角重叠区密度偏高
    //    【P0-2】体素尺寸自适应: 用户显式给定(>0)则尊重用户值; 否则按点云自身的
    //            中位最近邻间距估计 (原实现硬编码回退到 1.0mm, 与点云实际尺度无关)。
    pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const double median_spacing = estimateMedianSpacing(filtered_cloud);
    double leaf = m_params.voxel_leaf_size;
    bool leaf_auto = (leaf < 0.001);
    if (leaf_auto) {
        leaf = (median_spacing > 1e-6) ? median_spacing : 1.0;
        leaf = std::max(0.05, std::min(leaf, 20.0));   // 合理区间保护
    }
    pcl::VoxelGrid<pcl::PointXYZ> density_voxel;
    density_voxel.setLeafSize(leaf, leaf, leaf);
    density_voxel.setInputCloud(filtered_cloud);
    density_voxel.filter(*uniform_cloud);
    if (uniform_cloud->size() < 50) {
        qWarning() << "[S6] 体素降采样后点数不足，回退原始点云";
        *uniform_cloud = *filtered_cloud;
    }
    qDebug() << "[S6 密度均匀化] 中位点间距:" << median_spacing
             << "mm | 体素:" << leaf << (leaf_auto ? "mm (自适应)" : "mm (用户指定)")
             << "|" << filtered_cloud->size() << "→" << uniform_cloud->size() << "点";

    // 1. MLS 平滑重采样 + 法线计算
    //    MLS 对局部邻域拟合多项式曲面，输出的法线比原始 NormalEstimation 更平滑，
    //    同时上采样稀疏区域、均匀化点密度，有效抑制泊松毛刺和棱角。
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    // 自适应 MLS 半径: 基于点云中位数点间距，稀疏区放大半径以桥接间隙
    // (中位间距在上一步已估计, 此处直接复用, 避免重复 KNN)
    double mls_radius = leaf * 3.0;
    if (median_spacing > 1e-6) {
        // 稀疏区需要更大半径 (4x中位数间距)，密集区保持 3x leaf
        mls_radius = std::max(mls_radius, median_spacing * 4.0);
    }
    if (mls_radius < 1.5) mls_radius = 1.5;
    if (mls_radius > 20.0) mls_radius = 20.0;  // 上限放宽以桥接底部稀疏区与密集区间隙

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud(uniform_cloud);
    mls.setSearchRadius(mls_radius);
    mls.setPolynomialOrder(2);
    mls.setComputeNormals(true);
    mls.setSqrGaussParam(mls_radius * mls_radius);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    mls.setSearchMethod(mls_tree);
    mls.process(*cloud_with_normals);

    bool mls_ok = (cloud_with_normals->size() >= 30);
    if (!mls_ok) {
        qWarning() << "[S6] MLS 输出点数不足，降级为原始法线估计";
    }

    // 2. 法线方向一致性修正：以质心外推包围盒深度为虚拟视点
    //    改用 PCA 主方向替代固定 Z+，对任意朝向物体均适用
    //
    // 这里**不再**为了求质心/PCA 单独复制一份 PointXYZ 点云:
    // PointNormal 的前三个 float 就是 x/y/z, 直接在上面算即可。
    // 这份拷贝在百万点级是几十~上百 MB, 而且峰值内存正好卡在泊松网格化之前 ——
    // 本机 3.8GB 内存, 重建被 OOM 杀掉时它就是压死骆驼的那一份。
    float max_dist = 0;
    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    // PCA 求主方向 (最大方差方向 ≈ 主要表面朝向)
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    {
        // MLS 失败时点还只有位置 (法线在后面才估), 两条路径的源点云不同, 用泛型 lambda 抹平
        auto accumulate = [&](const auto& points, size_t n) {
            Eigen::Vector3f c = Eigen::Vector3f::Zero();
            if (n) {
                Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                for (size_t i = 0; i < n; ++i) {
                    const Eigen::Vector3f p = points[i].getVector3fMap();
                    sum += p.cast<double>();
                }
                c = (sum / double(n)).cast<float>();
            }
            centroid.head<3>() = c;
            for (size_t i = 0; i < n; ++i) {
                const Eigen::Vector3f d = points[i].getVector3fMap() - c;
                cov += d * d.transpose();
                const float sq = d.squaredNorm();
                if (sq > max_dist) max_dist = sq;
            }
        };
        if (mls_ok) accumulate(cloud_with_normals->points, cloud_with_normals->size());
        else        accumulate(uniform_cloud->points,      uniform_cloud->size());
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    Eigen::Vector3f pca_dir = eig.eigenvectors().col(2); // 最大特征值对应方向
    if (pca_dir.z() < 0) pca_dir = -pca_dir;             // 保持朝向 Z+
    Eigen::Vector3f viewpoint = centroid.head<3>() + pca_dir * 3.0f * std::sqrt(max_dist);

    if (mls_ok) {
        for (auto& pt : cloud_with_normals->points) {
            Eigen::Vector3f dir_to_view = viewpoint - pt.getVector3fMap();
            if (pt.getNormalVector3fMap().dot(dir_to_view) < 0) {
                pt.getNormalVector3fMap() *= -1.0f;
            }
        }
    } else {
        // 降级：手动估计法线 (【P0-2】使用 OpenMP 并行版)
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
        n.setNumberOfThreads(0);   // 0 = 全部可用核心
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(uniform_cloud);
        n.setInputCloud(uniform_cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);
        for (size_t i = 0; i < normals->size(); ++i) {
            Eigen::Vector3f dir_to_view = viewpoint - uniform_cloud->points[i].getVector3fMap();
            if (normals->points[i].getNormalVector3fMap().dot(dir_to_view) < 0) {
                normals->points[i].getNormalVector3fMap() *= -1.0f;
            }
        }
        pcl::concatenateFields(*uniform_cloud, *normals, *cloud_with_normals);
    }

    // 3. 表面重建 (按mesh_method选择)
    if (m_params.mesh_method == 1) {
        // GP3: 贪婪投影三角化——只在有点云处生成三角面，天然适合非闭合的转台数据
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        gp3.setSearchRadius(m_params.gp3_radius);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 3);
        gp3.setMinimumAngle(M_PI / 18);
        gp3.setMaximumAngle(2 * M_PI / 3);
        gp3.setNormalConsistency(true);
        gp3.setInputCloud(cloud_with_normals);
        gp3.reconstruct(mesh);
        qDebug() << "[S6 网格] GP3重建完成，面:" << mesh.polygons.size() << " r=" << m_params.gp3_radius;
    } else {
        // Poisson: 水密泊松重建
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth(m_params.poisson_depth);
        poisson.setPointWeight(m_params.poisson_point_weight);
        poisson.setInputCloud(cloud_with_normals);
        poisson.reconstruct(mesh);
        qDebug() << "[S6 网格] 泊松重建完成，面:" << mesh.polygons.size();

        if (mesh.polygons.empty() && cloud_with_normals->size() >= 10) {
            qWarning() << "[S6] 泊松重建失败(0面)，降级为贪婪投影三角化";
            m_params.mesh_method = 1; // 临时切换以复用GP3代码
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            gp3.setSearchRadius(mls_radius * 2.0);
            gp3.setMu(2.5);
            gp3.setMaximumNearestNeighbors(100);
            gp3.setMaximumSurfaceAngle(M_PI / 3);
            gp3.setMinimumAngle(M_PI / 18);
            gp3.setMaximumAngle(2 * M_PI / 3);
            gp3.setNormalConsistency(true);
            gp3.setInputCloud(cloud_with_normals);
            gp3.reconstruct(mesh);
            qWarning() << "[S6] GP3 降级重建完成，面:" << mesh.polygons.size();
        }
    }

    // 4. Laplacian 网格平滑：对泊松输出的网格顶点进行局部平均，
    //    消除细碎毛刺和八叉树离散化棱角，同时保持整体形状。
    if (!mesh.cloud.data.empty() && !mesh.polygons.empty()) {
        pcl::PointCloud<pcl::PointXYZ> verts;
        pcl::fromPCLPointCloud2(mesh.cloud, verts);

        // 构建顶点邻接表
        std::vector<std::vector<size_t>> neighbors(verts.size());
        for (const auto& poly : mesh.polygons) {
            for (size_t k = 0; k < poly.vertices.size(); ++k) {
                size_t v0 = poly.vertices[k];
                size_t v1 = poly.vertices[(k + 1) % poly.vertices.size()];
                if (v0 < verts.size() && v1 < verts.size()) {
                    neighbors[v0].push_back(v1);
                }
            }
        }

        const int smooth_iters = 15;
        const float relax = 0.20f;
        std::vector<Eigen::Vector3f> orig(verts.size());
        for (int iter = 0; iter < smooth_iters; ++iter) {
            for (size_t i = 0; i < verts.size(); ++i)
                orig[i] = verts[i].getVector3fMap();

            for (size_t i = 0; i < verts.size(); ++i) {
                if (neighbors[i].empty()) continue;
                Eigen::Vector3f avg = Eigen::Vector3f::Zero();
                for (size_t nb : neighbors[i])
                    avg += orig[nb];
                avg /= static_cast<float>(neighbors[i].size());
                verts[i].getVector3fMap() = orig[i] + relax * (avg - orig[i]);
            }
        }

        pcl::toPCLPointCloud2(verts, mesh.cloud);
        qDebug() << "[S6 网格] Laplacian 平滑完成，迭代:" << smooth_iters << " 松弛:" << relax;
    }

    // 5. 网格截断：移除远离原始点云数据的虚假闭合面 (自适应阈值)
    if (m_params.mesh_truncation_distance > 0.0 && !mesh.cloud.data.empty()) {
        double global_cutoff_sq = m_params.mesh_truncation_distance * m_params.mesh_truncation_distance;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(uniform_cloud);

        pcl::PointCloud<pcl::PointXYZ> vertices;
        pcl::fromPCLPointCloud2(mesh.cloud, vertices);

        // 自适应截断: 对每个顶点查询K近邻，局部稀疏区放宽容差
        std::vector<bool> vertex_valid(vertices.size(), false);
        std::vector<int> nn_idx(8);
        std::vector<float> nn_sqdist(8);
        int kept_vertices = 0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            int found = kdtree.nearestKSearch(vertices[i], 8, nn_idx, nn_sqdist);
            if (found > 0) {
                // 局部平均点间距 → 自适应阈值
                float local_avg = 0;
                for (int k = 0; k < found; ++k) local_avg += std::sqrt(nn_sqdist[k]);
                local_avg /= found;
                double adaptive_cutoff_sq = std::max(global_cutoff_sq,
                    static_cast<double>(local_avg * local_avg * 16.0)); // 4x 局部间距, 对曲面更宽容
                if (nn_sqdist[0] < adaptive_cutoff_sq) {
                    vertex_valid[i] = true;
                    ++kept_vertices;
                }
            }
        }

        std::vector<pcl::Vertices> filtered_polygons;
        for (const auto& poly : mesh.polygons) {
            if (poly.vertices.size() < 3) continue;
            int valid_count = 0;
            for (const auto& vi : poly.vertices) {
                if (vi < vertex_valid.size() && vertex_valid[vi]) ++valid_count;
            }
            if (valid_count >= 2) {
                filtered_polygons.push_back(poly);
            }
        }

        size_t removed_faces = mesh.polygons.size() - filtered_polygons.size();
        mesh.polygons.swap(filtered_polygons);

        qDebug() << "[S6 网格] 泊松重建完成，原始:" << filtered_cloud->size()
                 << " -> 均匀化:" << uniform_cloud->size()
                 << " -> MLS(r=" << mls_radius << "):" << cloud_with_normals->size()
                 << " depth:" << m_params.poisson_depth
                 << " pw:" << m_params.poisson_point_weight
                 << " 面:" << mesh.polygons.size()
                 << " 截断(自适应,基值):" << m_params.mesh_truncation_distance << "mm"
                 << " 顶点保留率:" << (vertices.size() ? kept_vertices * 100.0 / vertices.size() : 100) << "%"
                 << " 移除面:" << removed_faces;
    } else {
        qDebug() << "[S6 网格] 泊松重建完成，原始:" << filtered_cloud->size()
                 << " -> 均匀化:" << uniform_cloud->size()
                 << " -> MLS(r=" << mls_radius << "):" << cloud_with_normals->size()
                 << " depth:" << m_params.poisson_depth
                 << " pw:" << m_params.poisson_point_weight
                 << " 面:" << mesh.polygons.size();
    }

    return mesh;
}

// ======================== S4: 相机坐标系 → 转台基准坐标系 ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::transformToTurntableFrame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cam,
    double angle_deg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_turntable(new pcl::PointCloud<pcl::PointXYZ>);
    if (cloud_cam->empty()) return cloud_turntable;

    cv::Mat R_ct = m_calib_data.R_cam2turntable;
    cv::Mat T_ct = m_calib_data.T_cam2turntable;

    // 若转台标定数据为空，降级使用单位变换
    if (R_ct.empty() || T_ct.empty()) {
        Logger::warning("[S4] R_cam2turntable 或 T_cam2turntable 为空，点云保持相机坐标系");
        *cloud_turntable = *cloud_cam;
        return cloud_turntable;
    }

    // 校正坐标系回退：S3 三角化在立体校正后的相机系 (P_rect = R1 * P_orig)
    // 轴标定 PnP 在原始相机系。此处先施加 R1^T 将校正系点云转回原始相机系
    cv::Mat R_rect_inv;
    if (!m_calib_data.R_rect_L.empty()) {
        R_rect_inv = m_calib_data.R_rect_L.t(); // R1^T: 校正系→原始系
    } else {
        R_rect_inv = cv::Mat::eye(3, 3, CV_64F);
    }
    cv::Mat R_final = R_ct * R_rect_inv;  // R_base^T * R1^T = (R1*R_base)^T
    cv::Mat T_final = T_ct;               // 平移不变

    // 仅首帧输出变换矩阵
    // 【P0-4】S1~S4 并行后本函数会被多线程并发调用，静态标志必须原子化，
    //         否则存在数据竞争 (可能重复打印或漏打印)。
    static std::atomic<bool> s4_first_logged{false};
    if (!s4_first_logged.exchange(true)) {
        fprintf(stderr, "[S4 诊断] 校正系→世界系变换 (首帧):\n");
        fprintf(stderr, "  R_final = R_base^T * R1^T = [%.4f %.4f %.4f; %.4f %.4f %.4f; %.4f %.4f %.4f]\n",
                R_final.at<double>(0,0), R_final.at<double>(0,1), R_final.at<double>(0,2),
                R_final.at<double>(1,0), R_final.at<double>(1,1), R_final.at<double>(1,2),
                R_final.at<double>(2,0), R_final.at<double>(2,1), R_final.at<double>(2,2));
        fprintf(stderr, "  T_final = [%.2f, %.2f, %.2f] mm\n",
                T_final.at<double>(0), T_final.at<double>(1), T_final.at<double>(2));
        if (!cloud_cam->empty()) {
            const auto& p = cloud_cam->points[0];
            // 先 R1^T 回原始系
            double ox = R_rect_inv.at<double>(0,0)*p.x + R_rect_inv.at<double>(0,1)*p.y + R_rect_inv.at<double>(0,2)*p.z;
            double oy = R_rect_inv.at<double>(1,0)*p.x + R_rect_inv.at<double>(1,1)*p.y + R_rect_inv.at<double>(1,2)*p.z;
            double oz = R_rect_inv.at<double>(2,0)*p.x + R_rect_inv.at<double>(2,1)*p.y + R_rect_inv.at<double>(2,2)*p.z;
            double wx = R_ct.at<double>(0,0)*ox + R_ct.at<double>(0,1)*oy + R_ct.at<double>(0,2)*oz + T_ct.at<double>(0);
            double wy = R_ct.at<double>(1,0)*ox + R_ct.at<double>(1,1)*oy + R_ct.at<double>(1,2)*oz + T_ct.at<double>(1);
            double wz = R_ct.at<double>(2,0)*ox + R_ct.at<double>(2,1)*oy + R_ct.at<double>(2,2)*oz + T_ct.at<double>(2);
            fprintf(stderr, "  采样点: 校正系(%.1f,%.1f,%.1f) → 原始系(%.1f,%.1f,%.1f) → 世界系(%.1f,%.1f,%.1f) mm\n",
                    p.x, p.y, p.z, ox, oy, oz, wx, wy, wz);
        }
    }

    // P_world = R_final * P_rectified + T_final
    for (const auto& pt : cloud_cam->points) {
        double x = R_final.at<double>(0,0)*pt.x + R_final.at<double>(0,1)*pt.y + R_final.at<double>(0,2)*pt.z + T_final.at<double>(0);
        double y = R_final.at<double>(1,0)*pt.x + R_final.at<double>(1,1)*pt.y + R_final.at<double>(1,2)*pt.z + T_final.at<double>(1);
        double z = R_final.at<double>(2,0)*pt.x + R_final.at<double>(2,1)*pt.y + R_final.at<double>(2,2)*pt.z + T_final.at<double>(2);
        cloud_turntable->push_back(pcl::PointXYZ(static_cast<float>(x),
                                                  static_cast<float>(y),
                                                  static_cast<float>(z)));
    }
    return cloud_turntable;
}

// ======================== 底面间隙填充 ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::fillBottomGaps(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& axisDir,
    const Eigen::Vector3f& axisPoint,
    int* filledOut)
{
    if (filledOut) *filledOut = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    *out = *cloud;
    if (cloud->size() < 100) return out;

    // "竖直方向"必须来自真实的转台轴, 不能写死。
    // 写死 (0,1,0) 的后果见头文件里的说明: 取到的不是底面而是一片竖切片, 拟合出的平面
    // 与补出的合成点会横穿整个物体。
    Eigen::Vector3f up = axisDir.norm() > 1e-6f ? axisDir.normalized()
                                                : Eigen::Vector3f(0.0f, 1.0f, 0.0f);

    auto heightOf = [&](const pcl::PointXYZ& pt) { return up.dot(pt.getVector3fMap()); };

    // 1. 沿轴方向的两个极值
    float h_min = FLT_MAX, h_max = -FLT_MAX;
    for (const auto& pt : cloud->points) {
        const float h = heightOf(pt);
        if (h < h_min) h_min = h;
        if (h > h_max) h_max = h;
    }
    const float h_range = h_max - h_min;
    if (h_range < 1.0f) return out;

    // 1b. 中段最大半径 —— 下面"底座宽度判据"的基准。
    //
    // ★ 为什么必须要有这条判据: 原实现的闸门只有一句"这一段内点率 ≥ 0.5",
    //   它挡不住**物体自己某个接近水平的面**。12% 带本身就有 7mm 厚,
    //   在 1.5mm 容差下随便一张平面都能凑够 50% 内点 —— 实测本序列两端
    //   内点率都过关(-51.9~-44.8 端点 2051 个, z 标准差 1.42mm;
    //   另一端点 2621 个, 1.52mm), 判据完全没有区分度。
    //
    //   真正的"底面"是物体**立在上面**的那张面, 它必然比物体宽。
    //   这条性质与平面度无关, 是原闸门缺的那一维。
    float mid_rmax = 0.0f;
    {
        const float mid_lo = h_min + h_range * 0.35f, mid_hi = h_min + h_range * 0.65f;
        for (const auto& pt : cloud->points) {
            const float h = heightOf(pt);
            if (h < mid_lo || h > mid_hi) continue;
            // 到**转台轴**的垂距: 先减去轴上的点, 再取垂直于轴的分量。
            // 不能省掉 axisPoint —— 用"过原点的平行线"量, 半径会大得离谱
            // (实测 109mm vs 真实 55mm), 判据就永远不通过。
            const Eigen::Vector3f v = pt.getVector3fMap() - axisPoint;
            const float r = (v - up * up.dot(v)).norm();
            if (r > mid_rmax) mid_rmax = r;
        }
    }
    const float base_min_width = mid_rmax * 1.25f;        // 底座至少要比物体宽 25%

    // 2. 对**两端**各拟合一次, 选出真正像底座的那一端
    //
    // ★ 原实现写死"底面在沿轴方向的极小端", 这个前提在本标定下是**反的**:
    //   基准系的 +z 指向下方(转台盘 z≈+7, 物体顶 z≈-52), 于是 h_min 拿到的是
    //   物体的**顶端**。它拿盒子顶部拟出一张平面, 再按 3mm 网格铺满整个外接范围,
    //   凭空造出一张 115×120mm 的合成板飘在物体外面 —— 用户看到的"物体外那层
    //   规则散点"就是它。轴方向的符号是标定给出来的, 没有理由认为它一定指"上",
    //   所以不能写死, 必须让数据自己选。
    const int   ransac_iters = 100;
    const float ransac_dist  = 1.5f; // mm
    auto fitEnd = [&](bool lowEnd, Eigen::Vector4f& plane, double& ratio) -> bool {
        const float thr = lowEnd ? (h_min + h_range * 0.12f) : (h_max - h_range * 0.12f);
        std::vector<Eigen::Vector3f> pts;
        for (const auto& pt : cloud->points) {
            const float h = heightOf(pt);
            if (lowEnd ? (h < thr) : (h > thr)) pts.push_back(pt.getVector3fMap());
        }
        if (pts.size() < 30) return false;

        // 手动 RANSAC 平面拟合 (ax + by + cz + d = 0)
        // 固定随机种子，保证结果可复现 (与激光平面标定的 srand(42) 保持一致)
        std::srand(42);
        Eigen::Vector4f best(up.x(), up.y(), up.z(), lowEnd ? -h_min : -h_max);
        int best_inliers = 0;
        for (int iter = 0; iter < ransac_iters; ++iter) {
            int i1 = rand() % pts.size();
            int i2 = rand() % pts.size();
            int i3 = rand() % pts.size();
            if (i1 == i2 || i2 == i3 || i1 == i3) continue;
            Eigen::Vector3f n = (pts[i2] - pts[i1]).cross(pts[i3] - pts[i1]);
            float len = n.norm();
            if (len < 1e-6f) continue;
            n /= len;
            float d_val = -n.dot(pts[i1]);
            int inliers = 0;
            for (const auto& p : pts) {
                if (std::abs(n.dot(p) + d_val) < ransac_dist) ++inliers;
            }
            if (inliers > best_inliers) {
                best_inliers = inliers;
                best = Eigen::Vector4f(n.x(), n.y(), n.z(), d_val);
            }
        }
        ratio = double(best_inliers) / double(pts.size());
        // 法线统一朝轴的正方向, 便于后面算"内点到轴的距离"
        Eigen::Vector3f n(best[0], best[1], best[2]);
        if (n.dot(up) < 0.0f) { best = Eigen::Vector4f(-n.x(), -n.y(), -n.z(), -best[3]); }
        plane = best;
        return true;
    };

    // 3. 选端 + 闸门: 内点要够密(是平面) 且 要够宽(是底座)。两端都不像就不填。
    Eigen::Vector3f plane_n(up);
    float plane_d = -h_min;
    float chosen_width = 0.0f;
    int   chosen_end = -1;                 // 0 = 轴极小端, 1 = 轴极大端, -1 = 都不像
    double chosen_ratio = 0.0;
    for (int end = 0; end < 2; ++end) {
        Eigen::Vector4f p;
        double rt = 0.0;
        if (!fitEnd(end == 0, p, rt)) continue;
        if (rt < 0.5) continue;                       // 不像平面
        Eigen::Vector3f n(p[0], p[1], p[2]);
        const float d = p[3];
        float rmax = 0.0f;
        for (const auto& pt : cloud->points) {
            const Eigen::Vector3f v = pt.getVector3fMap();
            if (std::abs(n.dot(v) + d) >= ransac_dist) continue;
            const Eigen::Vector3f w = v - axisPoint;
            const float r = (w - up * up.dot(w)).norm();   // 到转台轴的垂距
            if (r > rmax) rmax = r;
        }
        if (rmax < base_min_width) continue;          // 比物体窄 → 不是底座, 是物体自己的面
        if (rmax > chosen_width) {
            chosen_end = end; chosen_width = rmax; chosen_ratio = rt;
            plane_n = n; plane_d = d;
        }
    }
    if (chosen_end < 0) {
        qDebug() << "[底面填充] 跳过: 沿轴两端都找不到「比物体更宽」的平面"
                 << "(中段最大半径" << mid_rmax << "mm, 要求底面内点半径 >=" << base_min_width
                 << "mm)。这一段多半是物体自己的面, 不是它立在上面的那张底座。"
                 << "不往点云里灌合成点。";
        return out;
    }

    // 4. 构建平面上的 2D 网格，填充稀疏单元格
    // 选两个切向量: u = n × up (若 n 接近 up 则该叉积退化, 换一个与 n 不平行的方向), v = n × u
    Eigen::Vector3f u = plane_n.cross(up);
    if (u.norm() < 0.1f) {
        const Eigen::Vector3f alt = (std::fabs(plane_n.z()) < 0.9f) ? Eigen::Vector3f(0, 0, 1)
                                                                   : Eigen::Vector3f(1, 0, 0);
        u = plane_n.cross(alt);
    }
    u.normalize();
    Eigen::Vector3f v = plane_n.cross(u).normalized();

    // 计算 UV 投影范围 —— 同样只用贴着平面的点, 网格才框在真实平面区域上
    const float occ_dist = 2.0f * ransac_dist;   // 与下面占用统计用同一道距离门
    float u_min = FLT_MAX, u_max = -FLT_MAX, v_min = FLT_MAX, v_max = -FLT_MAX;
    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        if (std::fabs(plane_n.dot(p) + plane_d) > occ_dist) continue;
        float up_uv = p.dot(u), vp = p.dot(v);
        if (up_uv < u_min) u_min = up_uv; if (up_uv > u_max) u_max = up_uv;
        if (vp < v_min) v_min = vp; if (vp > v_max) v_max = vp;
    }
    float u_range = u_max - u_min, v_range = v_max - v_min;
    if (u_range < 1.0f || v_range < 1.0f) return out;

    float grid_size = 3.0f; // mm
    float u_pad = u_range * 0.05f, v_pad = v_range * 0.05f;
    int nu = std::max(2, static_cast<int>((u_range + 2 * u_pad) / grid_size));
    int nv = std::max(2, static_cast<int>((v_range + 2 * v_pad) / grid_size));

    // 将 3D 点投影到 UV 网格，标记已占用单元格
    //
    // ★ 只统计**贴着平面**的点。原实现把**所有**点正交投影到平面上, 于是
    //   离平面 50mm 的圆柱壁、转台盘也各自"占用"了一格 —— 填充于是从真实的
    //   平面区域蔓延到整个外接范围, 这也正是那张合成板能铺到 115×120mm 的原因。
    //   正交投影本来就会把远处的东西投到平面上, 所以这个距离门是必需的。
    std::vector<std::vector<int>> grid_count(nu, std::vector<int>(nv, 0));
    int occ_pts = 0;
    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        float dist = plane_n.dot(p) + plane_d;
        if (std::fabs(dist) > occ_dist) continue;
        Eigen::Vector3f p_proj = p - dist * plane_n;  // 正交投影到平面
        int iu = static_cast<int>((p_proj.dot(u) - (u_min - u_pad)) / grid_size);
        int iv = static_cast<int>((p_proj.dot(v) - (v_min - v_pad)) / grid_size);
        if (iu >= 0 && iu < nu && iv >= 0 && iv < nv) {
            grid_count[iu][iv]++;
            ++occ_pts;
        }
    }
    if (occ_pts < 30) {
        qDebug() << "[底面填充] 跳过: 贴着候选底面的点只有" << occ_pts
                 << "个, 不足以判断哪里是空洞。不往点云里灌合成点。";
        return out;
    }

    // 填充空单元格: 在底面上添加点
    int filled = 0;
    for (int iu = 0; iu < nu; ++iu) {
        for (int iv = 0; iv < nv; ++iv) {
            if (grid_count[iu][iv] > 0) continue;
            // 检查邻居是否有占用的 (避免在孤立区域填充)
            int neighbor_sum = 0;
            for (int du = -1; du <= 1; ++du) {
                for (int dv = -1; dv <= 1; ++dv) {
                    int nu_idx = iu + du, nv_idx = iv + dv;
                    if (nu_idx >= 0 && nu_idx < nu && nv_idx >= 0 && nv_idx < nv)
                        neighbor_sum += (grid_count[nu_idx][nv_idx] > 0 ? 1 : 0);
                }
            }
            if (neighbor_sum < 2) continue;

            // UV 坐标 → 3D 平面点: P = u*u_coord + v*v_coord - d*n
            float u_coord = (iu + 0.5f) * grid_size + (u_min - u_pad);
            float v_coord = (iv + 0.5f) * grid_size + (v_min - v_pad);
            Eigen::Vector3f fill_pt = u * u_coord + v * v_coord - plane_d * plane_n;

            out->push_back(pcl::PointXYZ(fill_pt.x(), fill_pt.y(), fill_pt.z()));
            ++filled;
        }
    }
    if (filledOut) *filledOut = filled;
    qDebug() << "[底面填充] 轴方向:(" << up.x() << "," << up.y() << "," << up.z() << ")"
             << " 选中的端:" << (chosen_end == 0 ? "轴极小端" : "轴极大端")
             << " 内点率:" << chosen_ratio
             << " 底面半径:" << chosen_width << "/ 物体中段半径:" << mid_rmax
             << " 平面法向:(" << plane_n.x() << "," << plane_n.y() << "," << plane_n.z() << ")"
             << " 网格:" << nu << "x" << nv
             << " 平面内的点:" << occ_pts
             << " 填充点:" << filled << "(合成点, 不是测出来的)";
    return out;
}

// ======================== 对外暴露的组合接口 ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::processSingleView(
    const cv::Mat& img_left, const cv::Mat& img_right, double current_angle_deg,
    int frameIdx, ViewDiag* diag) {

    // 逐视角表头已去掉: 进度由 Tab5 进度条 + 每视角状态呈现,
    // 详细结果走 lastDetailLog → UI 日志面板
    m_viewStats.views++;
    std::vector<cv::Point2f> pts_left, pts_right, match_l, match_r;
    std::vector<float> conf_left, conf_right;   // 【P0-3】逐点置信度
    int mask_left = 0, mask_right = 0;

    // ---- 逐视角明细 (Tab6 曲线用; 见 ViewRecord 说明) ----
    // 逐视角的剔除计数由 triangulatePoints 直接累加进 m_viewStats (整段汇总),
    // 这里用"入口快照 + 出口做差"取本视角增量, 不必改动 triangulatePoints 内部。
    ViewRecord rec;
    rec.frameIdx = frameIdx;
    rec.angleDeg = current_angle_deg;
    const int s0_neg    = m_viewStats.negDepth;
    const int s0_far    = m_viewStats.tooFar;
    const int s0_degen  = m_viewStats.degenerate;
    const int s0_reject = m_viewStats.reprojReject;
    const int s0_dup    = m_viewStats.s2DupMerged;
    int s0_hist[8];
    for (int b = 0; b < 8; ++b) s0_hist[b] = m_viewStats.reprojHist[b];   // 同样靠做差取本视角增量
    // 每个 return 点都必须经由 done(), 漏一个就会在逐帧曲线上缺一帧
    auto done = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
                    -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
        rec.reprojReject = m_viewStats.reprojReject - s0_reject;
        rec.negDepth     = m_viewStats.negDepth     - s0_neg;
        rec.tooFar       = m_viewStats.tooFar       - s0_far;
        rec.degenerate   = m_viewStats.degenerate   - s0_degen;
        rec.s2DupMerged  = m_viewStats.s2DupMerged  - s0_dup;
        for (int b = 0; b < 8; ++b) rec.reprojHist[b] = m_viewStats.reprojHist[b] - s0_hist[b];
        rec.s3Points     = cloud ? static_cast<int>(cloud->size()) : 0;
        rec.ok           = (rec.s3Points > 0);
        m_viewStats.records.push_back(rec);
        return cloud;
    };

    int mask_left_dropped = 0, mask_right_dropped = 0;
    int sat_left = 0, sat_right = 0;
    extractLaserCenter(img_left, m_params.roi_left, pts_left, &mask_left, &conf_left,
                       &mask_left_dropped, &sat_left);
    extractLaserCenter(img_right, m_params.roi_right, pts_right, &mask_right, &conf_right,
                       &mask_right_dropped, &sat_right);
    rec.s1LeftSatRecover  = sat_left;
    rec.s1RightSatRecover = sat_right;
    // 记录的是**置信度过滤之前**的提取数: 这才是"S1 到底找到多少光条点"。
    // 过滤后归零与"压根没提取到"是两回事, 混在一起会让曲线看不出病因。
    rec.s1Left  = static_cast<int>(pts_left.size());
    rec.s1Right = static_cast<int>(pts_right.size());
    // 列极值法 (s1_method==2) 根本不做掩膜, 此时 extractLaserCenter 的 mask 出参
    // 只是 ROI 的像素数, 拿去算"覆盖率"会得出荒唐的结论 —— 直接置 0 表示"无掩膜"。
    const bool hasMask = (m_params.s1_method == 0 || m_params.s1_method == 1);
    rec.s1LeftMask  = hasMask ? mask_left  : 0;
    rec.s1RightMask = hasMask ? mask_right : 0;
    rec.s1LeftMaskDropped  = hasMask ? mask_left_dropped  : 0;
    rec.s1RightMaskDropped = hasMask ? mask_right_dropped : 0;
    // 行范围 / 最大行间隙: 在**原图**坐标上统计 (undistort 之前), 才对应人眼看图的位置。
    // 只看点数分不出"整条稀疏"与"中间断了一截", 这两个数才分得出。
    auto rowStats = [](const std::vector<cv::Point2f>& pts, int& rmin, int& rmax, int& gap) {
        rmin = rmax = -1;
        gap = 0;
        if (pts.empty()) return;
        std::vector<int> rows;
        rows.reserve(pts.size());
        for (const cv::Point2f& p : pts) rows.push_back(cvRound(p.y));
        std::sort(rows.begin(), rows.end());
        rmin = rows.front();
        rmax = rows.back();
        for (size_t i = 1; i < rows.size(); ++i) {
            const int d = rows[i] - rows[i - 1] - 1;
            if (d > gap) gap = d;
        }
    };
    rowStats(pts_left,  rec.rowMinLeft,  rec.rowMaxLeft,  rec.maxGapLeft);
    rowStats(pts_right, rec.rowMinRight, rec.rowMaxRight, rec.maxGapRight);
    // 诊断采集: 留一份**原始图坐标**的点 —— 叠加图要画在原图上, 校正系的点画上去是对不齐的
    if (diag) { diag->ptsLeftRaw = pts_left; diag->ptsRightRaw = pts_right; }

    if (pts_left.empty() || pts_right.empty()) {
        ++m_viewStats.extractEmpty;
        lastDetailLog = QString("S1: 左%1 右%2 → ❌ 提取为空").arg(pts_left.size()).arg(pts_right.size());
        return done(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    }

    // 【P0-3】按置信度预过滤低质量光条点 (在 S2 匹配之前，避免弱响应点参与错配)
    double conf_sum = 0.0;
    size_t conf_n = 0;
    for (float c : conf_left)  { conf_sum += c; ++conf_n; }
    for (float c : conf_right) { conf_sum += c; ++conf_n; }
    const double mean_conf = (conf_n > 0) ? (conf_sum / conf_n) : 0.0;

    if (m_params.min_point_confidence > 0.0f) {
        auto keepHighConfidence = [](std::vector<cv::Point2f>& pts, std::vector<float>& conf, float thr) {
            if (conf.size() != pts.size()) return;   // 置信度不可用时不做过滤
            std::vector<cv::Point2f> kept_pts;
            std::vector<float> kept_conf;
            kept_pts.reserve(pts.size());
            kept_conf.reserve(conf.size());
            for (size_t i = 0; i < pts.size(); ++i) {
                if (conf[i] >= thr) { kept_pts.push_back(pts[i]); kept_conf.push_back(conf[i]); }
            }
            pts.swap(kept_pts);
            conf.swap(kept_conf);   // 保持点与置信度仍一一对应
        };
        keepHighConfidence(pts_left, conf_left, m_params.min_point_confidence);
        keepHighConfidence(pts_right, conf_right, m_params.min_point_confidence);
        if (pts_left.empty() || pts_right.empty()) {
            ++m_viewStats.extractEmpty;
            lastDetailLog = QString("S1: 置信度阈值%1过滤后左%2 右%3 → ❌ 无有效点")
                .arg(m_params.min_point_confidence, 0, 'f', 2).arg(pts_left.size()).arg(pts_right.size());
            return done(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
        }
    }

    // S1 提取在原始畸变图像上，但 S2 匹配和 S3 三角化使用校正投影矩阵。
    // 必须先 undistort 将点转到校正坐标系。
    if (m_calib_data.is_rectified) {
        if (!m_calib_data.R_rect_L.empty() && !m_calib_data.P1_rectified.empty()) {
            std::vector<cv::Point2f> ptsL_undist, ptsR_undist;
            cv::undistortPoints(pts_left, ptsL_undist,
                m_calib_data.cameraMatrixL, m_calib_data.distCoeffL,
                m_calib_data.R_rect_L, m_calib_data.P1_rectified);
            cv::undistortPoints(pts_right, ptsR_undist,
                m_calib_data.cameraMatrixR, m_calib_data.distCoeffR,
                m_calib_data.R_rect_R, m_calib_data.P2_rectified);
            pts_left.swap(ptsL_undist);
            pts_right.swap(ptsR_undist);
        }
    }
    // 诊断采集: 校正系的全量光条点 —— 这是"右图到底有没有对应点"这类问题的判据来源
    if (diag) { diag->ptsLeftRect = pts_left; diag->ptsRightRect = pts_right; }

    epipolarConstraintMatch(pts_left, pts_right, match_l, match_r);
    rec.s2Match = static_cast<int>(match_l.size());
    if (match_l.empty()) {
        ++m_viewStats.matchEmpty;
        lastDetailLog = QString("S1→S2: 左%1 右%2 → ❌ 匹配0对").arg(pts_left.size()).arg(pts_right.size());
        return done(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam = triangulatePoints(match_l, match_r, diag);
    if (cloud_cam->empty()) {
        ++m_viewStats.triEmpty;
        lastDetailLog = QString("S1→S3: 左%1 右%2 → S2匹配%3对 → ❌ 三角化后为空").arg(pts_left.size()).arg(pts_right.size()).arg(match_l.size());
        return done(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    }

    // S4: 相机坐标系 → 世界坐标系 (使用标定的R_base/T_base)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world = transformToTurntableFrame(cloud_cam, current_angle_deg);
    lastDetailLog = QString("S1:%1(M:%5,conf%7)/%2(M:%6) → S2:%3 → S3:%4")
        .arg(pts_left.size()).arg(pts_right.size()).arg(match_l.size()).arg(cloud_world->size())
        .arg(mask_left).arg(mask_right)
        .arg(mean_conf, 0, 'f', 2);
    return done(cloud_world);
}

// 【接口修改】：新增 view_angles_deg 参数传入
void PointCloudBuilder::processGlobal(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& multi_view_clouds, 
    const std::vector<double>& view_angles_deg,
    const Eigen::Vector3f& axis_point, 
    const Eigen::Vector3f& axis_dir)
{
    // 逐段打常驻内存: 这台机器只有 3.8GB, 重建被内核 OOM 杀掉时, 只有知道
    // "是配准还是网格化吃掉的"才谈得上省。四段的省法完全不同。
    auto rss = [](const char* tag) {
        QFile f(QStringLiteral("/proc/self/statm"));
        if (!f.open(QIODevice::ReadOnly)) return;
        const QList<QByteArray> parts = f.readAll().trimmed().split(' ');
        if (parts.size() < 2) return;
        const double mb = double(parts[1].toLongLong()) * (double(sysconf(_SC_PAGESIZE)) / 1048576.0);
        qDebug() << "[内存]" << tag << QString::number(mb, 'f', 0) << "MB";
    };
    rss("S5 配准前");

    // 将标定得到的轴信息和点云传入配准算法
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_merged = multiViewRegistration(multi_view_clouds, view_angles_deg, axis_point, axis_dir);
    rss("S5 配准后");
    m_global_cloud = denoiseAndFilter(raw_merged);
    raw_merged.reset();   // 去噪是"读旧写新", 旧的那份上百万点别留着占内存
    rss("S6 去噪后");
    // 底面间隙填充。必须把真实轴方向传进去 —— 这一步会**往点云里灌合成点**,
    // 轴方向给错就会在物体旁边凭空多出一张平面(见 fillBottomGaps 的说明)。
    m_global_cloud = fillBottomGaps(m_global_cloud, axis_dir, axis_point, &m_bottomFilledCount);
    rss("S6 填底后");
    m_global_mesh = meshReconstruction(m_global_cloud);
    rss("S6 网格化后");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::getFinalPointCloud() const {
    return m_global_cloud;
}

pcl::PolygonMesh PointCloudBuilder::getFinalMesh() const {
    return m_global_mesh;
}

void PointCloudBuilder::storeResult(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PolygonMesh& mesh) {
    m_global_cloud = cloud;
    m_global_mesh = mesh;
}
