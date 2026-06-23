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
#include <pcl/registration/icp_nl.h> // 包含点到面ICP
#include <pcl/features/normal_3d_omp.h> // 包含并行法线计算，加快速度
#include <algorithm>
#include <cmath>
#include <set>
#include <Eigen/Eigenvalues>  // SelfAdjointEigenSolver for PCA viewpoint
#include <QDebug>

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
                                           int* mask_pixels_out)
{
    center_points.clear();
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
    cv::Mat hsv_img;
    cv::cvtColor(roi_img, hsv_img, cv::COLOR_BGR2HSV);
    if (mask_pixels_out) *mask_pixels_out = l_channel.rows * l_channel.cols;

    // ===== 掩膜构建 (STEGER_MASK / CENTROID_MASK 共用) =====
    cv::Mat mask_a;
    bool need_mask = (m_params.s1_method == 0 || m_params.s1_method == 1);
    if (need_mask) {
        int base_a_thresh = m_params.lab_a_threshold;
        int bottom_a_thresh = std::max(60, base_a_thresh - 60);
        int adaptive_split = static_cast<int>(a_channel.rows * 0.65);
        mask_a = cv::Mat(a_channel.size(), CV_8U);
        for (int y = 0; y < a_channel.rows; ++y) {
            uchar* mask_row = mask_a.ptr<uchar>(y);
            const uchar* a_row = a_channel.ptr<uchar>(y);
            int thresh = (y < adaptive_split) ? base_a_thresh
                         : base_a_thresh - static_cast<int>((base_a_thresh - bottom_a_thresh)
                             * (y - adaptive_split) / static_cast<float>(a_channel.rows - adaptive_split));
            for (int x = 0; x < a_channel.cols; ++x)
                mask_row[x] = (a_row[x] > thresh) ? 255 : 0;
        }
        cv::Mat mask_hsv_low, mask_hsv_high, mask_hsv;
        cv::inRange(hsv_img, cv::Scalar(0, 60, 20), cv::Scalar(10, 255, 255), mask_hsv_low);
        cv::inRange(hsv_img, cv::Scalar(170, 60, 20), cv::Scalar(180, 255, 255), mask_hsv_high);
        cv::bitwise_or(mask_hsv_low, mask_hsv_high, mask_hsv);
        cv::bitwise_and(mask_a, mask_hsv, mask_a);
        cv::Mat strict_dilated;
        cv::dilate(mask_a, strict_dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)));
        cv::Mat mask_bright, mask_hsv_b2, mask_hsv_bl, mask_hsv_bh;
        cv::inRange(l_channel, 160, 255, mask_bright);
        cv::inRange(hsv_img, cv::Scalar(0, 60, 120), cv::Scalar(10, 255, 255), mask_hsv_bl);
        cv::inRange(hsv_img, cv::Scalar(170, 60, 120), cv::Scalar(180, 255, 255), mask_hsv_bh);
        cv::bitwise_or(mask_hsv_bl, mask_hsv_bh, mask_hsv_b2);
        cv::bitwise_and(mask_bright, mask_hsv_b2, mask_bright);
        cv::bitwise_and(mask_bright, strict_dilated, mask_bright);
        cv::bitwise_or(mask_a, mask_bright, mask_a);
        cv::dilate(mask_a, mask_a, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        if (mask_pixels_out) *mask_pixels_out = cv::countNonZero(mask_a);
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
        const float t_max = std::max(0.1f, m_params.steger_t_max);
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                if (mask_a.at<uchar>(y, x) == 0) continue;
                float gx = dx.at<float>(y, x), gy = dy.at<float>(y, x);
                float l_px = static_cast<float>(l_channel.at<uchar>(y, x));
                float cpx, cpy; bool ok = false;
                if (m_params.steger_edge_offset_enable && l_px > 235) {
                    float gn = std::sqrt(gx*gx+gy*gy);
                    if (gn > 0.5f) {
                        float nxg=gx/gn, nyg=gy/gn, ms=std::max(3.0f, sigma*6.0f);
                        float ld=-1, rd=-1;
                        for (float d=0.5f; d<=ms && (ld<0||rd<0); d+=0.5f) {
                            if (ld<0) { int ex=cvRound(x-d*nxg), ey=cvRound(y-d*nyg);
                                if (ex>=0&&ex<w&&ey>=0&&ey<h&&l_channel.at<uchar>(ey,ex)<200) ld=d; }
                            if (rd<0) { int ex=cvRound(x+d*nxg), ey=cvRound(y+d*nyg);
                                if (ex>=0&&ex<w&&ey>=0&&ey<h&&l_channel.at<uchar>(ey,ex)<200) rd=d; }
                        }
                        if (ld>0&&rd>0) { float hw=(ld+rd)*0.5f;
                            if (hw>=1.5f&&hw<=12.0f) { cpx=x+(rd-ld)*0.5f*nxg; cpy=y+(rd-ld)*0.5f*nyg; ok=true; } }
                        else if (ld>0) { cpx=x-(ld+3.0f)*nxg; cpy=y-(ld+3.0f)*nyg; ok=true; }
                        else if (rd>0) { cpx=x+(rd+3.0f)*nxg; cpy=y+(rd+3.0f)*nyg; ok=true; }
                    }
                    if (ok) goto ste_rec;
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
                if (l_px>resp.at<float>(y,x)) { resp.at<float>(y,x)=l_px; smx.at<float>(y,x)=px; smy.at<float>(y,x)=py; }
            }
        }
        for (int y=1; y<h-1; ++y) for (int x=1; x<w-1; ++x) {
            float v=resp.at<float>(y,x); if (v<1e-4f) continue;
            bool im=true;
            for (int dy=-1;dy<=1&&im;++dy) for (int dx=-1;dx<=1&&im;++dx)
                if ((dx||dy)&&resp.at<float>(y+dy,x+dx)>v) im=false;
            if (im) center_points.emplace_back(smx.at<float>(y,x), smy.at<float>(y,x));
        }

    } else if (m_params.s1_method == 1) {
        // -------- 灰度重心 + 掩膜 --------
        std::vector<int> rough_y(w, -1);
        std::vector<uchar> peak_l(w, 0);
        for (int x = 1; x < w - 1; ++x) {
            uchar max_l = 0; int max_y = -1;
            for (int y = 1; y < h - 1; ++y) {
                if (mask_a.at<uchar>(y, x) == 0) continue;
                uchar v = l_channel.at<uchar>(y, x);
                if (v > max_l) { max_l = v; max_y = y; }
            }
            if (max_y >= 0) { rough_y[x] = max_y; peak_l[x] = max_l; }
        }
        for (int x = 1; x < w - 1; ++x) {
            if (rough_y[x] < 0) continue;
            int cy = rough_y[x]; float sx = static_cast<float>(x), sy;
            if (peak_l[x] >= 250 && m_params.steger_edge_offset_enable) {
                int lx=x, rx=x; while (lx>0 && l_channel.at<uchar>(cy,lx)>200) --lx;
                while (rx<w-1 && l_channel.at<uchar>(cy,rx)>200) ++rx;
                int ty=cy, by=cy; while (ty>0 && l_channel.at<uchar>(ty,x)>200) --ty;
                while (by<h-1 && l_channel.at<uchar>(by,x)>200) ++by;
                int hx=(rx-lx)/2, hy=(by-ty)/2;
                if (hx>=2&&hx<=12&&hy>=2&&hy<=12) { sx=float(lx+rx)*0.5f; sy=float(ty+by)*0.5f; }
                else sy=float(cy);
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
        }

    } else {
        // -------- 纯列极值 (无掩膜) --------
        for (int x = 1; x < w - 1; ++x) {
            uchar max_l = 0; int max_y = -1;
            for (int y = 1; y < h - 1; ++y) {
                uchar v = l_channel.at<uchar>(y, x);
                if (v > max_l) { max_l = v; max_y = y; }
            }
            if (max_y < 0 || max_l < 70) continue;
            float sx = static_cast<float>(x), sy;
            if (max_l >= 250 && m_params.steger_edge_offset_enable) {
                int lx=x, rx=x; while (lx>0 && l_channel.at<uchar>(max_y,lx)>200) --lx;
                while (rx<w-1 && l_channel.at<uchar>(max_y,rx)>200) ++rx;
                int ty=max_y, by=max_y; while (ty>0 && l_channel.at<uchar>(ty,x)>200) --ty;
                while (by<h-1 && l_channel.at<uchar>(by,x)>200) ++by;
                sx=float(lx+rx)*0.5f; sy=float(ty+by)*0.5f;
            } else {
                int y0=std::max(4,max_y-8), y1=std::min(h-5,max_y+8);
                float sl=0,sly=0;
                for (int yy=y0; yy<=y1; ++yy) {
                    float v=float(l_channel.at<uchar>(yy,x)); sl+=v; sly+=v*yy;
                }
                sy=(sl>0)?sly/sl:float(max_y);
            }
            center_points.emplace_back(sx+safe_roi.x, sy+safe_roi.y);
        }
    }

    // Pauta异常剔除 (所有方法共用)
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
            std::vector<cv::Point2f> filtered;
            filtered.push_back(center_points.front());
            for (size_t i = 1; i + 1 < center_points.size(); ++i)
                if (local_dy[i-1] <= thresh) filtered.push_back(center_points[i]);
            filtered.push_back(center_points.back());
            center_points.swap(filtered);
        }

    return;
}
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
        qDebug() << "====== [S2 极线匹配] 模式: 视差跳变切断 + 分段 DP ======";

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
        qDebug() << " 检测到光条分段数:" << seg_starts.size();

        // ========================================================================
        // 【阶段 2】：定义分段 DP 执行器 (Lambda 函数)
        // ========================================================================
        auto runSegmentDP = [&](const std::vector<cv::Point2f>& segL, const std::vector<cv::Point2f>& segR) {
            int m = segL.size(), n = segR.size();
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
                    const cv::Point2f& pj = segR[j-1];

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
                matched_pts_left.push_back(segL[match.first]);
                matched_pts_right.push_back(segR[match.second]);
            }
        };

        // ========================================================================
        // 【阶段 3】：利用 Y 轴单调性，快速切片并执行分段 DP
        // ========================================================================
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

            std::vector<cv::Point2f> segR(it_low, it_high);

            if (!segL.empty() && !segR.empty()) {
                runSegmentDP(segL, segR);
            }
        }

        qDebug() << " 匹配前 左:" << M << " 右:" << N;
        qDebug() << " 切断分段匹配后 成功对数:" << matched_pts_left.size();

        // ========================================================================
        // 【阶段 4：真正的降级保底逻辑】(原代码此处为空导致返回0)
        // ========================================================================
        if (matched_pts_left.empty()) {
            qWarning() << "[S2 分段DP] 无匹配，降级为严格贪心最近邻";
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
            qDebug() << " 降级贪心匹配成功对数:" << matched_pts_left.size();
        }
        return;
    }

    // ==================== 未校正模式 ====================
    qDebug() << "====== [S2 极线匹配] 模式: 原始F矩阵 ======";
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
    qDebug() << " 匹配前 左:" << pts_left.size() << " 右:" << pts_right.size();
    qDebug() << " 匹配后 成功对数:" << matched_pts_left.size();
}

// ======================== S3: 三角化 ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::triangulatePoints(
    const std::vector<cv::Point2f>& pts_left,
    const std::vector<cv::Point2f>& pts_right)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pts_left.empty() || pts_right.empty()) return cloud;

    cv::Mat P1, P2;
    if (m_calib_data.is_rectified && !m_calib_data.P1_rectified.empty() && !m_calib_data.P2_rectified.empty()) {
        P1 = m_calib_data.P1_rectified;
        P2 = m_calib_data.P2_rectified;
        qDebug() << "====== [S3 三角化] 使用校正后P1/P2 ======";
    } else {
        cv::Mat K_L = m_calib_data.cameraMatrixL, K_R = m_calib_data.cameraMatrixR;
        cv::Mat R = m_calib_data.R_stereo, T = m_calib_data.T_stereo;
        P1 = (cv::Mat_<double>(3,4) << K_L.at<double>(0,0), K_L.at<double>(0,1), K_L.at<double>(0,2), 0,
                                       K_L.at<double>(1,0), K_L.at<double>(1,1), K_L.at<double>(1,2), 0,
                                       K_L.at<double>(2,0), K_L.at<double>(2,1), K_L.at<double>(2,2), 0);
        cv::Mat Rt; cv::hconcat(R, T, Rt);
        P2 = K_R * Rt;
        qDebug() << "====== [S3 三角化] 使用原始参数 ======";
    }

    // 转换为 OpenCV 需要的 1 行 N 列双通道矩阵
    cv::Mat pts_left_mat  = cv::Mat(pts_left).reshape(2, 1);
    cv::Mat pts_right_mat = cv::Mat(pts_right).reshape(2, 1);

    cv::Mat pts4d;
    cv::triangulatePoints(P1, P2, pts_left_mat, pts_right_mat, pts4d);

    int neg_count = 0, too_far_count = 0, valid_count = 0;
    double z_min = 1e10, z_max = -1e10;
    double sum_correction = 0;
    int corrected_count = 0;
    Eigen::Vector4d plane = m_calib_data.laser_plane_coeff;
    bool has_plane = (std::fabs(plane[0]) > 1e-6 || std::fabs(plane[1]) > 1e-6
                   || std::fabs(plane[2]) > 1e-6);
    // 光平面R1修正: Tab3标定在原始系, S3在校正系, 将光平面法向量转到校正系
    if (has_plane && !m_calib_data.R_rect_L.empty()) {
        Eigen::Matrix3d R1;
        for (int r=0;r<3;r++) for (int c=0;c<3;c++) R1(r,c)=m_calib_data.R_rect_L.at<double>(r,c);
        Eigen::Vector3d n_orig(plane[0], plane[1], plane[2]);
        Eigen::Vector3d n_rect = R1 * n_orig;
        plane = Eigen::Vector4d(n_rect[0], n_rect[1], n_rect[2], plane[3]);
    }
    for (int i = 0; i < pts4d.cols; ++i) {
        float w = pts4d.at<float>(3, i);
        if (std::fabs(w) < 1e-6) { neg_count++; continue; }
        float x = pts4d.at<float>(0, i) / w;
        float y = pts4d.at<float>(1, i) / w;
        float z = pts4d.at<float>(2, i) / w;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
        if (z > m_params.depth_min && z < m_params.depth_max) {
            if (has_plane) {
                double signed_dist = plane[0]*x + plane[1]*y + plane[2]*z + plane[3];
                sum_correction += std::fabs(signed_dist);
                corrected_count++;
                x -= static_cast<float>(signed_dist * plane[0]);
                y -= static_cast<float>(signed_dist * plane[1]);
                z -= static_cast<float>(signed_dist * plane[2]);
            }
            cloud->push_back(pcl::PointXYZ(x, y, z));
            valid_count++;
        } else if (z <= 0) {
            neg_count++;
        } else {
            too_far_count++;
        }
    }
    double mean_corr = corrected_count > 0 ? sum_correction / corrected_count : 0;
    qDebug() << " 对数:" << pts4d.cols << " 有效:" << valid_count
             << " 负深:" << neg_count << " 过远:" << too_far_count
             << " 范围:[" << z_min << "," << z_max << "]"
             << (has_plane ? QString(" [光平面均修正:%1mm]").arg(mean_corr, 0, 'f', 1).toStdString().c_str() : " [无光平面]");
    return cloud;
}

// ======================== S5: 多视角 Point-to-Plane ICP 配准 (无旋转轴依赖版) ========================
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
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
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
        icp.setInputSource(rough_aligned);
        icp.setInputTarget(prev_normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr icp_aligned(new pcl::PointCloud<pcl::PointNormal>);
        icp.align(*icp_aligned);

        Eigen::Matrix4f T_delta_final;
        bool skip_icp = !m_params.use_icp;  // UI可切换纯轴旋转/ICP
        if (!skip_icp && icp.hasConverged()) {
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
        } else {
            T_delta_final = T_delta_guess;
            ++icp_failed;
        }
        all_transforms[i] = all_transforms[i-1] * T_delta_final;

        // 【诊断】记录采样帧
        if (!init_sample) continue;
        double icp_score = icp.getFitnessScore();
        Eigen::Matrix4f T_icp_delta = icp.getFinalTransformation();
        Eigen::AngleAxisf aa(T_icp_delta.block<3,3>(0,0));
        float icp_rot_deg = aa.angle() * 180.0 / M_PI;
        float icp_trans = T_icp_delta.block<3,1>(0,3).norm();
        bool would_accept = (icp_rot_deg < 2.0 && icp_trans < 2.0);
        // 【S5诊断】首5帧输出理论vs实际平移量
        if (i <= first_valid_idx + 5) {
            Eigen::Vector3f t_theory = T_delta_guess.block<3,1>(0,3);
            Eigen::Matrix4f T_final_delta = all_transforms[i].inverse() * all_transforms[i-1]; // not quite right
            // Actually use the stored T_delta_final
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

        // 始终用增量累计偏离恒等的量作为均摊误差
        // 理论: 360°扫描后累计变换应为I, 偏离量即系统误差
        // 闭环ICP在累计rot=63°时仍可能收敛到0°(360°对齐), 不可依赖
        Eigen::Matrix4f T_err = all_transforms[last_valid_idx].inverse();
        bool use_incremental_fallback = true;

        {
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
            qDebug() << "====== [闭环优化]" << (use_incremental_fallback ? "(增量回退)" : "(ICP)")
                     << "旋转误差:" << final_rot << "° 平移误差:" << t_err_final.norm()
                     << "mm，已全量SE(3)均摊 ======";
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
    lastDetailLog = QString("[S5 ICP] 相邻帧: 收敛 %1 / 降级 %2 | max_correspondence_distance: %3 mm | 融合点云: %4")
        .arg(icp_converged).arg(icp_failed).arg(m_params.icp_max_correspondence_distance).arg(accumulated_cloud->size());
    if (!diag_lines.empty()) {
        lastDetailLog += "\n[S5 诊断] 采样帧初始对齐 (粗配准均值距离=初始猜测质量, ICP收敛=Yes则精配准成功, rot/t=ICP修正量):";
        for (const auto& line : diag_lines)
            lastDetailLog += "\n" + line;
    }
    return accumulated_cloud;
}


// ======================== S6: 滤波与网格化 (架构重构：泊松替换为贪懒投影) ========================
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    double leaf = m_params.voxel_leaf_size;
    if (leaf < 0.001) leaf = 1.0;
    pcl::VoxelGrid<pcl::PointXYZ> density_voxel;
    density_voxel.setLeafSize(leaf, leaf, leaf);
    density_voxel.setInputCloud(filtered_cloud);
    density_voxel.filter(*uniform_cloud);
    if (uniform_cloud->size() < 50) {
        qWarning() << "[S6] 体素降采样后点数不足，回退原始点云";
        *uniform_cloud = *filtered_cloud;
    }

    // 1. MLS 平滑重采样 + 法线计算
    //    MLS 对局部邻域拟合多项式曲面，输出的法线比原始 NormalEstimation 更平滑，
    //    同时上采样稀疏区域、均匀化点密度，有效抑制泊松毛刺和棱角。
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    // 自适应 MLS 半径: 基于点云中位数点间距，稀疏区放大半径以桥接间隙
    double mls_radius = leaf * 3.0;
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> spacing_tree;
        spacing_tree.setInputCloud(uniform_cloud);
        std::vector<float> avg_spacings;
        avg_spacings.reserve(std::min(500UL, uniform_cloud->size()));
        std::vector<int> nn_idx(6);
        std::vector<float> nn_sqdist(6);
        size_t step = std::max(1UL, uniform_cloud->size() / 500);
        for (size_t i = 0; i < uniform_cloud->size(); i += step) {
            int found = spacing_tree.nearestKSearch(uniform_cloud->points[i], 6, nn_idx, nn_sqdist);
            if (found > 1) {
                float sum = 0; for (int k = 1; k < found; ++k) sum += std::sqrt(nn_sqdist[k]);
                avg_spacings.push_back(sum / (found - 1));
            }
        }
        if (!avg_spacings.empty()) {
            std::sort(avg_spacings.begin(), avg_spacings.end());
            double median_spacing = avg_spacings[avg_spacings.size() / 2];
            // 稀疏区需要更大半径 (4x中位数间距)，密集区保持 3x leaf
            mls_radius = std::max(mls_radius, median_spacing * 4.0);
        }
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (mls_ok) {
        pcl::copyPointCloud(*cloud_with_normals, *centroid_cloud);
    } else {
        *centroid_cloud = *uniform_cloud;
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*centroid_cloud, centroid);
    float max_dist = 0;
    // PCA 求主方向 (最大方差方向 ≈ 主要表面朝向)
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& pt : centroid_cloud->points) {
        Eigen::Vector3f d = pt.getVector3fMap() - centroid.head<3>();
        cov += d * d.transpose();
        float sq = d.squaredNorm();
        if (sq > max_dist) max_dist = sq;
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
        // 降级：手动估计法线
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
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
    static bool s4_first_logged = false;
    if (!s4_first_logged) {
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
        s4_first_logged = true;
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
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    *out = *cloud;
    if (cloud->size() < 100) return out;

    // 1. 找底面极值 (转台轴默认 Y 向上，底面在 Y 最小处)
    float y_min = FLT_MAX, y_max = -FLT_MAX;
    float x_min = FLT_MAX, x_max = -FLT_MAX;
    float z_min = FLT_MAX, z_max = -FLT_MAX;
    for (const auto& pt : cloud->points) {
        if (pt.y < y_min) y_min = pt.y;
        if (pt.y > y_max) y_max = pt.y;
        if (pt.x < x_min) x_min = pt.x;
        if (pt.x > x_max) x_max = pt.x;
        if (pt.z < z_min) z_min = pt.z;
        if (pt.z > z_max) z_max = pt.z;
    }
    float y_range = y_max - y_min;
    if (y_range < 1.0f) return out;

    // 2. 收集底部 12% 点用于平面拟合
    float y_thresh = y_min + y_range * 0.12f;
    std::vector<Eigen::Vector3f> bottom_pts;
    for (const auto& pt : cloud->points) {
        if (pt.y < y_thresh)
            bottom_pts.push_back(pt.getVector3fMap());
    }
    if (bottom_pts.size() < 30) return out;

    // 3. 手动 RANSAC 平面拟合 (ax + by + cz + d = 0)
    const int ransac_iters = 100;
    const float ransac_dist = 1.5f; // mm
    Eigen::Vector4f best_plane(0, 1, 0, -y_min); // 初始猜测: 水平面
    int best_inliers = 0;
    for (int iter = 0; iter < ransac_iters; ++iter) {
        int i1 = rand() % bottom_pts.size();
        int i2 = rand() % bottom_pts.size();
        int i3 = rand() % bottom_pts.size();
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;
        Eigen::Vector3f n = (bottom_pts[i2] - bottom_pts[i1]).cross(bottom_pts[i3] - bottom_pts[i1]);
        float len = n.norm();
        if (len < 1e-6f) continue;
        n /= len;
        float d_val = -n.dot(bottom_pts[i1]);
        int inliers = 0;
        for (const auto& p : bottom_pts) {
            if (std::abs(n.dot(p) + d_val) < ransac_dist) ++inliers;
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_plane = Eigen::Vector4f(n.x(), n.y(), n.z(), d_val);
        }
    }

    Eigen::Vector3f plane_n(best_plane[0], best_plane[1], best_plane[2]);
    float plane_d = best_plane[3];
    // 确保法线朝 Y+ (上方)
    if (plane_n.y() < 0) { plane_n = -plane_n; plane_d = -plane_d; }

    // 4. 构建平面上的 2D 网格，填充稀疏单元格
    // 选两个切向量: u = n × world_up 的归一化 (若 n 接近 Y，用 Z), v = n × u
    Eigen::Vector3f world_up(0, 1, 0);
    Eigen::Vector3f u = plane_n.cross(world_up);
    if (u.norm() < 0.1f) u = plane_n.cross(Eigen::Vector3f(0, 0, 1));
    u.normalize();
    Eigen::Vector3f v = plane_n.cross(u).normalized();

    // 计算点云在 UV 切空间的投影范围
    float u_min = FLT_MAX, u_max = -FLT_MAX, v_min = FLT_MAX, v_max = -FLT_MAX;
    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        float up = p.dot(u), vp = p.dot(v);
        if (up < u_min) u_min = up; if (up > u_max) u_max = up;
        if (vp < v_min) v_min = vp; if (vp > v_max) v_max = vp;
    }
    float u_range = u_max - u_min, v_range = v_max - v_min;
    if (u_range < 1.0f || v_range < 1.0f) return out;

    float grid_size = 3.0f; // mm
    float u_pad = u_range * 0.05f, v_pad = v_range * 0.05f;
    int nu = std::max(2, static_cast<int>((u_range + 2 * u_pad) / grid_size));
    int nv = std::max(2, static_cast<int>((v_range + 2 * v_pad) / grid_size));

    // 将 3D 点投影到 UV 网格，标记已占用单元格
    std::vector<std::vector<int>> grid_count(nu, std::vector<int>(nv, 0));
    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        float dist = plane_n.dot(p) + plane_d;
        Eigen::Vector3f p_proj = p - dist * plane_n;  // 正交投影到平面
        int iu = static_cast<int>((p_proj.dot(u) - (u_min - u_pad)) / grid_size);
        int iv = static_cast<int>((p_proj.dot(v) - (v_min - v_pad)) / grid_size);
        if (iu >= 0 && iu < nu && iv >= 0 && iv < nv)
            grid_count[iu][iv]++;
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
    qDebug() << "[底面填充] RANSAC内点数:" << best_inliers
             << " 平面法向:(" << plane_n.x() << "," << plane_n.y() << "," << plane_n.z() << ")"
             << " 网格:" << nu << "x" << nv << " 填充点:" << filled;
    return out;
}

// ======================== 对外暴露的组合接口 ========================
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::processSingleView(
    const cv::Mat& img_left, const cv::Mat& img_right, double current_angle_deg) {

    qDebug() << "====== 开始处理单视角: " << current_angle_deg << "° ======";
    std::vector<cv::Point2f> pts_left, pts_right, match_l, match_r;
    int mask_left = 0, mask_right = 0;
    extractLaserCenter(img_left, m_params.roi_left, pts_left, &mask_left);
    extractLaserCenter(img_right, m_params.roi_right, pts_right, &mask_right);

    if (pts_left.empty() || pts_right.empty()) {
        lastDetailLog = QString("S1: 左%1 右%2 → ❌ 提取为空").arg(pts_left.size()).arg(pts_right.size());
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
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

    epipolarConstraintMatch(pts_left, pts_right, match_l, match_r);
    if (match_l.empty()) {
        lastDetailLog = QString("S1→S2: 左%1 右%2 → ❌ 匹配0对").arg(pts_left.size()).arg(pts_right.size());
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam = triangulatePoints(match_l, match_r);
    if (cloud_cam->empty()) {
        lastDetailLog = QString("S1→S3: 左%1 右%2 → S2匹配%3对 → ❌ 三角化后为空").arg(pts_left.size()).arg(pts_right.size()).arg(match_l.size());
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // S4: 相机坐标系 → 世界坐标系 (使用标定的R_base/T_base)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world = transformToTurntableFrame(cloud_cam, current_angle_deg);
    lastDetailLog = QString("S1:%1(M:%5)/%2(M:%6) → S2:%3 → S3:%4")
        .arg(pts_left.size()).arg(pts_right.size()).arg(match_l.size()).arg(cloud_world->size())
        .arg(mask_left).arg(mask_right);
    return cloud_world;
}

// 【接口修改】：新增 view_angles_deg 参数传入
void PointCloudBuilder::processGlobal(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& multi_view_clouds, 
    const std::vector<double>& view_angles_deg,
    const Eigen::Vector3f& axis_point, 
    const Eigen::Vector3f& axis_dir)
{
    // 将标定得到的轴信息和点云传入配准算法
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_merged = multiViewRegistration(multi_view_clouds, view_angles_deg, axis_point, axis_dir);
    m_global_cloud = denoiseAndFilter(raw_merged);
    m_global_cloud = fillBottomGaps(m_global_cloud);   // 底面间隙填充，辅助泊松闭合底面
    m_global_mesh = meshReconstruction(m_global_cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuilder::getFinalPointCloud() const {
    return m_global_cloud;
}

pcl::PolygonMesh PointCloudBuilder::getFinalMesh() const {
    return m_global_mesh;
}
