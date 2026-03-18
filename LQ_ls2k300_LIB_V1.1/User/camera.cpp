// 摄像头模块实现：
// 负责图像采集、二值化、八邻域搜线、赛道元素识别/状态机处理，以及补线后的中线/偏差计算。
#include "camera.h"
#include "displayer.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>

namespace
{
// 参考工程使用的固定分辨率标定参数：
// 当前工程允许任意输出分辨率，因此需要把这些经验参数按比例缩放后再使用。
constexpr int kLegacyRefWidth = 188;
constexpr int kLegacyRefHeight = 120;
constexpr int kLegacyTrackWidthCount = 90;
constexpr int kLegacyTrackWidths[kLegacyTrackWidthCount] = {
    136, 135, 134, 133, 132, 131, 130, 129, 128, 127,
    126, 125, 124, 123, 122, 121, 120, 119, 118, 117,
    116, 115, 114, 113, 112, 111, 110, 109, 108, 107,
    106, 105, 104, 103, 102, 101, 100, 99, 98, 97,
    96, 95, 94, 93, 92, 91, 90, 89, 88, 87,
    86, 85, 84, 83, 82, 81, 80, 79, 78, 77,
    76, 75, 74, 73, 72, 71, 70, 69, 68, 67,
    60, 59, 58, 57, 56, 55, 54, 53, 52, 51,
    50, 45, 41, 40, 39, 37, 33, 31, 30, 29
};
constexpr double kLegacyOffsetWeights[15] = {
    0.96, 0.92, 0.88, 0.83, 0.77,
    0.71, 0.65, 0.59, 0.53, 0.47,
    0.47, 0.47, 0.47, 0.47, 0.47
};

// -------------------- 通用数学/尺寸辅助函数 --------------------
int ClampInt(int value, int minValue, int maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

int ScaleLegacyRow(int legacyRow, int imgH)
{
    if (imgH <= 1)
        return 0;
    return ClampInt(static_cast<int>(std::lround(
                        legacyRow * (imgH - 1) / static_cast<double>(kLegacyRefHeight - 1))),
                    0, imgH - 1);
}

int ScaleLegacyCount(int legacyCount, int imgH)
{
    const int scaled = static_cast<int>(std::lround(
        legacyCount * imgH / static_cast<double>(kLegacyRefHeight)));
    return std::max(1, scaled);
}

int ScaleLegacyX(int legacyX, int imgW)
{
    if (imgW <= 1)
        return 0;
    return ClampInt(static_cast<int>(std::lround(
                        legacyX * (imgW - 1) / static_cast<double>(kLegacyRefWidth - 1))),
                    0, imgW - 1);
}

// 初始化补线缓存的尺寸，保证 neo/second 线数组与图像高度一致。
void EnsureLineToolResultSize(Camera::LineToolResult& toolResult, int imgW, int imgH)
{
    (void)imgW;
    const size_t H = (imgH > 0) ? static_cast<size_t>(imgH) : 0U;
    toolResult.neo_l_line_x.resize(H, 0);
    toolResult.neo_r_line_x.resize(H, 0);
    toolResult.l_second_line_x.resize(H, 0);
    toolResult.r_second_line_x.resize(H, 0);
}

// 通过现有线坐标反推图像宽度，主要用于只拿到搜线结果时的辅助计算。
int InferImageWidthFromLines(const Camera::LineSearchResult& lineResult)
{
    int maxX = 0;

    for (size_t i = 0; i < lineResult.left_x.size(); ++i)
        maxX = std::max(maxX, lineResult.left_x[i]);
    for (size_t i = 0; i < lineResult.right_x.size(); ++i)
        maxX = std::max(maxX, lineResult.right_x[i]);
    for (size_t i = 0; i < lineResult.center_x.size(); ++i)
        maxX = std::max(maxX, static_cast<int>(std::lround(lineResult.center_x[i])));

    return std::max(1, maxX + 1);
}

// 左/右边线查询接口统一了“丢线值”的表示，便于后续逻辑复用。
bool LeftLineFoundAt(const Camera::LineSearchResult& lineResult, int y)
{
    return y >= 0 &&
           y < static_cast<int>(lineResult.left_x.size()) &&
           lineResult.left_x[static_cast<size_t>(y)] != 0;
}

bool RightLineFoundAt(const Camera::LineSearchResult& lineResult, int y, int imgW)
{
    return y >= 0 &&
           y < static_cast<int>(lineResult.right_x.size()) &&
           lineResult.right_x[static_cast<size_t>(y)] != imgW - 1;
}

int LeftLineXAt(const Camera::LineSearchResult& lineResult, int y)
{
    if (y < 0 || y >= static_cast<int>(lineResult.left_x.size()))
        return 0;
    return lineResult.left_x[static_cast<size_t>(y)];
}

int RightLineXAt(const Camera::LineSearchResult& lineResult, int y)
{
    if (y < 0 || y >= static_cast<int>(lineResult.right_x.size()))
        return 0;
    return lineResult.right_x[static_cast<size_t>(y)];
}

// 三点曲率：用于区分直道、弯道以及一些特殊元素入口形态。
double SignedCurvature(double x1, double y1, double x2, double y2, double x3, double y3)
{
    const double ax = x2 - x1;
    const double ay = y2 - y1;
    const double bx = x3 - x2;
    const double by = y3 - y2;
    const double cx = x3 - x1;
    const double cy = y3 - y1;

    const double cross = ax * cy - ay * cx;
    const double a = std::hypot(x2 - x1, y2 - y1);
    const double b = std::hypot(x3 - x2, y3 - y2);
    const double c = std::hypot(x3 - x1, y3 - y1);
    if (a <= 1e-6 || b <= 1e-6 || c <= 1e-6)
        return 0.0;

    return 2.0 * cross / (a * b * c);
}

// 每帧开始前清理“当前帧识别结果”；
// 与 roadType_/阶段变量这类跨帧状态机数据不同，这里只存本帧观测值。
void ResetFrameTrackResult(Camera::TrackElementResult& result)
{
    result.current = Camera::TrackElementResult::None;
    result.line_found = false;
    result.row_hits = 0;
    result.left_roundabout_detected = false;
    result.right_roundabout_detected = false;
    result.left_cross_detected = false;
    result.right_cross_detected = false;
    result.cross_detected = false;
    result.fork_detected = false;
    result.straight_detected = false;
    result.bend_detected = false;
    result.bend_direction = 0;
    result.straight_factor = 0;
    result.outside_protected = false;
    result.left_diuxian_hang = 0;
    result.left_budiuxian_hang = 0;
    result.right_diuxian_hang = 0;
    result.right_budiuxian_hang = 0;
    result.l_guaidain_x1 = 0;
    result.l_guaidain_y1 = 0;
    result.r_guaidain_x1 = 0;
    result.r_guaidain_y1 = 0;
    result.l_guaidain_x2 = 0;
    result.l_guaidain_y2 = 0;
    result.r_guaidain_x2 = 0;
    result.r_guaidain_y2 = 0;
    result.l_guaidain_x = 0;
    result.l_guaidain_y = 0;
    result.r_guaidain_x = 0;
    result.r_guaidain_y = 0;
    result.bianxian_guaidian_l = 0;
    result.bianxian_guaidian_r = 0;
    result.sancha_x = 0;
    result.sancha_y = 0;
    result.sancha_x_zhengque = 0;
    result.sancha_y_zhengque = 0;
    result.qianzhang = 0;
    result.cirque_or_cross_count = 0;
    result.left_curvature = 0.0;
    result.right_curvature = 0.0;
    result.track_length = 0;
    result.left_lose_value = 0;
    result.right_lose_value = 0;
    result.left_losemax = 0;
    result.right_losemax = 0;
    result.roundabout_stage = 0;
    result.fork_stage = 0;
    result.cross_stage = 0;
}

// 统计左右丢线数和首次丢线位置，是元素识别的基础特征之一。
void ComputeLoseMetrics(const Camera::LineSearchResult& lineResult, int imgW,
                        int& leftLoseValue, int& rightLoseValue,
                        int& leftLoseMax, int& rightLoseMax)
{
    leftLoseValue = 0;
    rightLoseValue = 0;
    leftLoseMax = 0;
    rightLoseMax = 0;

    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    if (H <= 0)
        return;

    bool leftMaxFound = false;
    bool rightMaxFound = false;
    for (int y = H - 1; y >= 10; --y)
    {
        if (!LeftLineFoundAt(lineResult, y))
        {
            leftLoseValue++;
            if (!leftMaxFound)
            {
                leftLoseMax = y;
                leftMaxFound = true;
            }
        }

        if (!RightLineFoundAt(lineResult, y, imgW))
        {
            rightLoseValue++;
            if (!rightMaxFound)
            {
                rightLoseMax = y;
                rightMaxFound = true;
            }
        }
    }
}

int& LostRowsRef(uint8_t type, Camera::TrackElementResult& result)
{
    return (type == 1) ? result.left_diuxian_hang : result.right_diuxian_hang;
}

int& FoundRowsRef(uint8_t type, Camera::TrackElementResult& result)
{
    return (type == 1) ? result.left_budiuxian_hang : result.right_budiuxian_hang;
}
}

// -------------------- 相机生命周期与基础采集 --------------------
Camera::Camera(int deviceId)
    : deviceId_(deviceId)
{
}

Camera::~Camera()
{
    close();
}

bool Camera::open()
{
    if (cap_.isOpened())
        return true;

    if (!cap_.open(deviceId_))
    {
        std::printf("Open camera failed.\n");
        return false;
    }
    return true;
}

void Camera::close()
{
    if (cap_.isOpened())
    {
        cap_.release();
    }
}

bool Camera::isOpened() const
{
    return cap_.isOpened();
}

bool Camera::grabGrayFrame(cv::Mat& gray, int outWidth, int outHeight)
{
    if (!cap_.isOpened())
        return false;

    cv::Mat frame;
    cv::Mat resized;

    if (!cap_.read(frame) || frame.empty())
    {
        std::printf("Read frame failed.\n");
        return false;
    }

    // 缩放到指定分辨率
    cv::resize(frame, resized, cv::Size(outWidth, outHeight));
    // 转为灰度图（只负责图像处理，不负责显示）
    cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

    return true;
}

void Camera::flipVertical(const cv::Mat& src, cv::Mat& dst) const
{
    // 使用 OpenCV 提供的 flip 接口，flipCode = 0 表示上下颠倒
    cv::flip(src, dst, 0);
}

double Camera::otsuBinarize(const cv::Mat& gray, cv::Mat& binary) const
{
    if (gray.empty())
    {
        binary.release();
        return 0.0;
    }

    // 确保输入是单通道 8bit 图像
    CV_Assert(gray.type() == CV_8UC1);

    // 每隔固定帧数才下采样并重新计算大津阈值，其余帧直接沿用缓存阈值，避免无效 resize
    otsuFrameCount_++;
    if (!otsuInited_ || otsuFrameCount_ >= 10)
    {
        int proc_w = gray.cols / 2;
        int proc_h = gray.rows / 2;
        cv::Mat smallGray;
        if (proc_w > 0 && proc_h > 0)
            cv::resize(gray, smallGray, cv::Size(proc_w, proc_h));
        else
            smallGray = gray;

        cv::Mat tmpBin;
        double thresh = cv::threshold(
            smallGray, tmpBin,
            0, 255,
            cv::THRESH_BINARY | cv::THRESH_OTSU
        );
        otsuThresh_ = thresh;
        otsuInited_ = true;
        otsuFrameCount_ = 0;
    }

    // 使用缓存的大津阈值，对原始分辨率图像做二值化
    cv::threshold(gray, binary, otsuThresh_, 255, cv::THRESH_BINARY);
    return otsuThresh_;
}

void Camera::pixelFilterErode(cv::Mat& binary) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;
    const int H = binary.rows;
    const int W = binary.cols;
    if (H < 21 || W < 21)
        return;
    cv::Mat tmp = binary.clone();
    const int yEnd = H - 10;
    const int xEnd = W - 10;
    for (int y = 10; y < yEnd; y++)
    {
        const uchar* up   = binary.ptr<uchar>(y - 1);
        const uchar* cur  = binary.ptr<uchar>(y);
        const uchar* down = binary.ptr<uchar>(y + 1);
        uchar* out = tmp.ptr<uchar>(y);
        for (int x = 10; x < xEnd; x++)
        {
            int sum = static_cast<int>(up[x]) + static_cast<int>(down[x])
                    + static_cast<int>(cur[x - 1]) + static_cast<int>(cur[x + 1]);
            if (cur[x] == 0 && sum >= 3 * 255)
                out[x] = 255;
            else if (cur[x] != 0 && sum < 2 * 255)
                out[x] = 0;
            else
                out[x] = cur[x];
        }
    }
    tmp.copyTo(binary);
}

double Camera::updateFps()
{
    int64_t now = cv::getTickCount();

    if (lastTick_ == 0)
    {
        lastTick_ = now;
        fps_ = 0.0;
        frameCount_ = 0;
        return fps_;
    }

    frameCount_++;
    double elapsed = static_cast<double>(now - lastTick_) / cv::getTickFrequency();
    if (elapsed >= 1.0)
    {
        fps_ = frameCount_ / elapsed;
        frameCount_ = 0;
        lastTick_ = now;
    }

    return fps_;
}

// -------------------- 图像预处理与搜线 --------------------
void Camera::searchLineEightNeighbor(const cv::Mat& binary, LineSearchResult& result, int type) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
    {
        result.left_x.clear();
        result.right_x.clear();
        result.search_line_end = 0;
        return;
    }

    const int H = binary.rows;
    const int W = binary.cols;
    const int mid = W / 2;

    result.left_x.assign(static_cast<size_t>(H), 0);
    result.right_x.assign(static_cast<size_t>(H), W - 1);
    result.search_line_end = 10;

    int search_line_end = 10;
    const int edge_margin_left = 10;
    const int edge_margin_right = 7;
    const int neighbor_offset = 7;

    for (int row = H - 1; row > search_line_end; row--)
    {
        int l_search_start, r_search_start;

        if (type == 0)
        {
            const int row_below = row + 1;
            const bool has_below = (row_below < H);
            const int l_below = has_below ? result.left_x[static_cast<size_t>(row_below)] : 0;
            const int r_below = has_below ? result.right_x[static_cast<size_t>(row_below)] : W - 1;
            const bool left_ok = (l_below != 0);
            const bool right_ok = (r_below != W - 1);

            if (row >= H - 5 || (!left_ok && !right_ok && row < H - 4))
            {
                l_search_start = mid;
                r_search_start = mid;
            }
            else if (left_ok && right_ok && row < H - 4)
            {
                l_search_start = l_below + neighbor_offset;
                r_search_start = r_below - neighbor_offset;
            }
            else if (left_ok && !right_ok && row < H - 4)
            {
                l_search_start = l_below + neighbor_offset;
                r_search_start = mid;
            }
            else if (!left_ok && right_ok && row < H - 4)
            {
                l_search_start = mid;
                r_search_start = r_below - neighbor_offset;
            }
            else
            {
                l_search_start = mid;
                r_search_start = mid;
            }
        }
        else
        {
            l_search_start = mid;
            r_search_start = mid;
        }

        if (l_search_start >= W - 2)
            l_search_start = mid;
        if (r_search_start <= 2)
            r_search_start = mid;

        if (row >= 2 && row < H - 40)
        {
            const uchar c0 = binary.at<uchar>(row, mid);
            const uchar c1 = binary.at<uchar>(row - 1, mid);
            const uchar c2 = binary.at<uchar>(row - 2, mid);
            if (c0 == 0 && c1 == 0 && c2 == 0)
            {
                search_line_end = row + 1;
                result.search_line_end = search_line_end;
                break;
            }
        }

        bool l_found = false;
        bool r_found = false;

        for (int l_width = l_search_start; l_width > 1; l_width--)
        {
            if (l_width > 2 && l_width - 2 >= 0)
            {
                uchar a = binary.at<uchar>(row, l_width - 2);
                uchar b = binary.at<uchar>(row, l_width - 1);
                uchar c = binary.at<uchar>(row, l_width);
                if (a == 0 && b == 0 && c != 0)
                {
                    int lx = l_width - 1;
                    if (lx <= edge_margin_left)
                        result.left_x[static_cast<size_t>(row)] = 0;
                    else
                    {
                        result.left_x[static_cast<size_t>(row)] = lx;
                        l_found = true;
                    }
                    break;
                }
            }
            if (l_width == 2)
            {
                result.left_x[static_cast<size_t>(row)] = 0;
                break;
            }
        }
        if (result.left_x[static_cast<size_t>(row)] != 0)
            l_found = true;

        for (int r_width = r_search_start; r_width < W - 2; r_width++)
        {
            if (r_width < W - 3 && r_width + 2 < W)
            {
                uchar a = binary.at<uchar>(row, r_width);
                uchar b = binary.at<uchar>(row, r_width + 1);
                uchar c = binary.at<uchar>(row, r_width + 2);
                if (a != 0 && b == 0 && c == 0)
                {
                    int rx = r_width + 1;
                    if (rx >= W - edge_margin_right)
                        result.right_x[static_cast<size_t>(row)] = W - 1;
                    else
                    {
                        result.right_x[static_cast<size_t>(row)] = rx;
                        r_found = true;
                    }
                    break;
                }
            }
            if (r_width == W - 3)
            {
                result.right_x[static_cast<size_t>(row)] = W - 1;
                break;
            }
        }
        if (result.right_x[static_cast<size_t>(row)] != W - 1)
            r_found = true;

        int& lx = result.left_x[static_cast<size_t>(row)];
        int& rx = result.right_x[static_cast<size_t>(row)];
        if (rx <= lx)
        {
            const int default_width = TrackWidthAt(row, W, H);
            if (r_found && !l_found)
                lx = (rx > default_width) ? (rx - default_width) : 0;
            else if (!r_found && l_found)
                rx = (lx + default_width < W) ? (lx + default_width) : (W - 1);
            else if (!r_found && !l_found)
            {
                lx = 0;
                rx = W - 1;
            }
        }
    }

    result.search_line_end = search_line_end;
}

// 将参考工程的“每行赛道宽度表”按当前分辨率映射为像素宽度。
int Camera::TrackWidthAt(int y, int imgW, int imgH) const
{
    if (imgW <= 1 || imgH <= 1)
        return std::max(1, imgW / 2);

    const int rowFromBottom = ClampInt(imgH - 1 - y, 0, kLegacyTrackWidthCount - 1);
    const int legacyWidth = kLegacyTrackWidths[rowFromBottom];
    return ClampInt(ScaleLegacyX(legacyWidth, imgW), 1, imgW - 1);
}

int Camera::EstimateDistanceMeasure(const LineSearchResult& lineResult) const
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    if (H <= 0)
        return 40;

    const int visibleRows = std::max(0, H - lineResult.search_line_end);
    return ClampInt(visibleRows * 2, 40, 180);
}

// 每帧搜线后先把原始边线同步到补线缓存，后续元素状态机可在此基础上做补线覆盖。
void Camera::initializeLineTool(const LineSearchResult& lineResult, int imgW, int imgH)
{
    EnsureLineToolResultSize(lineToolResult_, imgW, imgH);
    lineToolResult_.rightx_1 = { imgW - 1, imgW - 1 };
    lineToolResult_.righty_1 = { imgH - 1, imgH - 1 };

    for (int y = 0; y < imgH; ++y)
    {
        const size_t idx = static_cast<size_t>(y);
        lineToolResult_.neo_l_line_x[idx] =
            (idx < lineResult.left_x.size()) ? lineResult.left_x[idx] : 0;
        lineToolResult_.neo_r_line_x[idx] =
            (idx < lineResult.right_x.size()) ? lineResult.right_x[idx] : (imgW - 1);
        lineToolResult_.l_second_line_x[idx] = 0;
        lineToolResult_.r_second_line_x[idx] = imgW - 1;
    }
}

// 对外保留原接口名，内部统一走“含补线结果”的中线/偏差计算。
void Camera::calculateCenterLine(LineSearchResult& result, int imgW, int imgH) const
{
    calculateCenterLineWithSupplement(result, imgW, imgH);
}

// 计算中线与偏差时优先使用 neo 补线结果；
// 这样环岛/十字等处理后的引导线能直接作用于最终偏差输出。
void Camera::calculateCenterLineWithSupplement(LineSearchResult& result, int imgW, int imgH) const
{
    result.offset = 0.0;
    const int H = imgH;
    const int W = imgW;
    const int mid = W / 2;

    if (result.left_x.size() != static_cast<size_t>(H) ||
        result.right_x.size() != static_cast<size_t>(H))
    {
        return;
    }

    const bool hasSupplementLines =
        lineToolResult_.neo_l_line_x.size() == static_cast<size_t>(H) &&
        lineToolResult_.neo_r_line_x.size() == static_cast<size_t>(H);
    const std::vector<int>& leftSource = hasSupplementLines ? lineToolResult_.neo_l_line_x : result.left_x;
    const std::vector<int>& rightSource = hasSupplementLines ? lineToolResult_.neo_r_line_x : result.right_x;

    result.center_x.assign(static_cast<size_t>(H), 0.0);
    for (int y = H - 1; y >= 10; --y)
    {
        result.center_x[static_cast<size_t>(y)] =
            0.5 * (leftSource[static_cast<size_t>(y)] + rightSource[static_cast<size_t>(y)]);
    }

    const int smoothWindow = 20;
    std::vector<double> tmp(result.center_x);
    for (int y = 0; y < H; ++y)
    {
        double sum = 0.0;
        int count = 0;
        const int half = smoothWindow / 2;
        const int lower = std::max(0, y - half);
        const int upper = std::min(H - 1, y + half);
        for (int yy = lower; yy <= upper; ++yy)
        {
            sum += tmp[static_cast<size_t>(yy)];
            count++;
        }
        if (count > 0)
            result.center_x[static_cast<size_t>(y)] = sum / count;
    }

    const int weightBottom = H - 15;
    const int weightTop = H - 25;
    for (int y = weightBottom; y >= weightTop && y >= 0; --y)
    {
        const int idx = ClampInt((H - 20) - y, 0, 14);
        result.offset += kLegacyOffsetWeights[idx] *
                         (result.center_x[static_cast<size_t>(y)] - mid);
    }
    result.offset /= 2.0;
}

// TFT 绘制也优先显示补线结果，便于现场直接看到状态机修改后的边线。
void Camera::drawLineResultOnTFT(const LineSearchResult& result, int imgW, int imgH) const
{
    static constexpr uint16_t RGB565_BLUE  = 0x001F;
    static constexpr uint16_t RGB565_GREEN = 0x07E0;
    static constexpr uint16_t RGB565_RED   = 0xF800;
    const int halfW = 1;
    const bool hasSupplementLines =
        lineToolResult_.neo_l_line_x.size() == static_cast<size_t>(imgH) &&
        lineToolResult_.neo_r_line_x.size() == static_cast<size_t>(imgH);
    const std::vector<int>& leftSource = hasSupplementLines ? lineToolResult_.neo_l_line_x : result.left_x;
    const std::vector<int>& rightSource = hasSupplementLines ? lineToolResult_.neo_r_line_x : result.right_x;

    for (int y = 0; y < imgH; y++)
    {
        int lx = leftSource[static_cast<size_t>(y)];
        int rx = rightSource[static_cast<size_t>(y)];
        for (int dx = -halfW; dx <= halfW; dx++)
        {
            int xl = lx + dx;
            if (xl >= 0 && xl < imgW)
                TFT_DrawPixel(static_cast<uint8_t>(xl), static_cast<uint8_t>(y), RGB565_BLUE);
            if (rx != lx)
            {
                int xr = rx + dx;
                if (xr >= 0 && xr < imgW)
                    TFT_DrawPixel(static_cast<uint8_t>(xr), static_cast<uint8_t>(y), RGB565_GREEN);
            }
        }
        if (y >= 10 && y < static_cast<int>(result.center_x.size()))
        {
            int mx = static_cast<int>(result.center_x[static_cast<size_t>(y)] + 0.5);
            for (int dx = -halfW; dx <= halfW; dx++)
            {
                int xm = mx + dx;
                if (xm >= 0 && xm < imgW)
                    TFT_DrawPixel(static_cast<uint8_t>(xm), static_cast<uint8_t>(y), RGB565_RED);
            }
        }
    }
    TFT_Flush();
}

// -------------------- 元素检测入口 --------------------
void Camera::detectTrackElements(const cv::Mat& binary, LineSearchResult& lineResult,
                                 TrackElementResult& result)
{
    ResetFrameTrackResult(result);

    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    const int startRow = std::max(0, H - 10);
    const int endRow = std::max(0, H / 2);
    const int strength = std::max(4, W / 24);

    detectZebraCrossing(binary, result, startRow, endRow, strength);
    Element_Test(binary, lineResult, result);
    Element_Handle(lineResult, result);
    syncRoadStateToResult(result);

    result.roundabout_stage = huandaoStage_;
    result.fork_stage = sanchaStage_;
    result.cross_stage = crossStage_;

    if (result.current == TrackElementResult::None && result.zebra_detected)
        result.current = TrackElementResult::ZebraCrossing;
}

// 斑马线检测基本沿用参考工程逻辑，并保留冷却期避免重复触发。
void Camera::detectZebraCrossing(const cv::Mat& binary, TrackElementResult& result,
                                 int startRow, int endRow, int strength) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H <= 0 || W <= 0)
        return;

    int start = std::max(0, std::min(startRow, H - 1));
    int end = std::max(0, std::min(endRow, H - 1));
    if (start < end)
        std::swap(start, end);

    const int qiangdu = std::max(1, strength);

    // 按原逻辑保留冷却期：冷却期间不再重新检测，只维护计数器。
    if (result.in_cooldown)
    {
        result.cooldown_counter++;
        if (result.cooldown_counter >= 200)
        {
            result.in_cooldown = false;
            result.cooldown_counter = 0;
        }
        result.line_found = false;
        result.row_hits = 0;
        return;
    }

    int times = 0;
    bool lineFound = false;

    for (int height = start; height >= end; --height)
    {
        int black_blocks_l = 0;
        int black_blocks_r = 0;
        int cursor_l = 0;
        int cursor_r = 0;

        for (int width_l = W / 2, width_r = W / 2;
             width_l >= 1 && width_r < W - 2;
             --width_l, ++width_r)
        {
            if (binary.at<uchar>(height, width_l) == 0)
            {
                if (cursor_l > 40)
                    break;
                cursor_l++;
            }
            else
            {
                if (cursor_l >= qiangdu && cursor_l <= qiangdu + 8)
                    black_blocks_l++;
                cursor_l = 0;
            }

            if (binary.at<uchar>(height, width_r) == 0)
            {
                if (cursor_r >= 20)
                    break;
                cursor_r++;
            }
            else
            {
                if (cursor_r >= qiangdu && cursor_r <= qiangdu + 8)
                    black_blocks_r++;
                cursor_r = 0;
            }
        }

        const int blackBlockCount = black_blocks_l + black_blocks_r;
        if (blackBlockCount >= 3 && blackBlockCount <= 20)
            times++;
    }

    result.row_hits = times;

    int hitThreshold = (start - end - 5) / 3;
    if (hitThreshold < 1)
        hitThreshold = 1;

    if (times >= hitThreshold)
    {
        lineFound = true;
        result.zebra_detected = true;

        if (result.stop_count == 0)
        {
            result.stop_count = 1;
            result.in_cooldown = true;
            result.cooldown_counter = 0;
        }
        else if (result.stop_count == 1)
        {
            result.stop_count = 2;
        }
    }
    else
    {
        result.zebra_detected = false;
    }

    result.line_found = lineFound;
    result.last_line_found = lineFound;
}

// -------------------- 元素识别辅助函数 --------------------
void Camera::Sancha_didian(const cv::Mat& binary, TrackElementResult& result) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H < 12 || W < 8)
        return;

    double k1 = 0.35;
    double k2 = 0.65;
    if (result.sancha_y_zhengque >= 60)
    {
        k1 = 0.10;
        k2 = 0.90;
    }

    result.sancha_x = 0;
    result.sancha_y = 0;

    const int xStart = ClampInt(static_cast<int>(std::lround(k1 * W)), 0, W - 1);
    const int xEnd = ClampInt(static_cast<int>(std::lround(k2 * W)), xStart + 1, W - 1);
    for (int x = xStart; x < xEnd; ++x)
    {
        int candidateY = 0;
        for (int y = H - 1; y > 10; --y)
        {
            if (binary.at<uchar>(y, x) == 0 &&
                binary.at<uchar>(y - 1, x) == 0 &&
                binary.at<uchar>(y - 2, x) == 0)
            {
                result.sancha_x = x;
                result.sancha_y = y;
                candidateY = y;
                break;
            }
        }

        if (candidateY > result.sancha_y_zhengque)
        {
            result.sancha_y_zhengque = candidateY;
            result.sancha_x_zhengque = result.sancha_x;
        }
    }
}

void Camera::Qianzhang(const cv::Mat& binary, TrackElementResult& result) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H < 12 || W < 3)
        return;

    const int mid = W / 2;
    result.qianzhang = 0;
    for (int y = H - 1; y > 10; --y)
    {
        if (binary.at<uchar>(y, mid) == 0 &&
            binary.at<uchar>(y - 1, mid) == 0 &&
            binary.at<uchar>(y - 2, mid) == 0)
        {
            result.qianzhang = y;
        }
    }
}

int Camera::Cirque_or_Cross(uint8_t type, int startline, const cv::Mat& binary,
                            const LineSearchResult& lineResult, TrackElementResult& result) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return 0;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H <= 0 || W <= 0)
        return 0;

    int start = ClampInt(startline, 0, H - 1);
    if (start >= H - 15)
        start = H - 16;

    int num = 0;
    if (type == 1)
    {
        for (int y = start; y < std::min(start + 25, H - 1); ++y)
        {
            const int lx = LeftLineXAt(lineResult, y);
            for (int x = lx; x > 0 && x > lx - 50; --x)
            {
                if (binary.at<uchar>(y, x) != 0)
                    num++;
            }
        }
    }
    else if (type == 2)
    {
        for (int y = start; y < std::min(start + 25, H - 1); ++y)
        {
            const int rx = RightLineXAt(lineResult, y);
            for (int x = rx; x < W && x < rx + 50; ++x)
            {
                if (binary.at<uchar>(y, x) != 0)
                    num++;
            }
        }
    }

    result.cirque_or_cross_count = num;
    return num;
}

// 将跨帧 roadType_ 状态同步回当前帧结果，供显示层和上层控制统一读取。
void Camera::syncRoadStateToResult(TrackElementResult& result) const
{
    result.left_roundabout_detected = roadType_.LeftCirque;
    result.right_roundabout_detected = roadType_.RightCirque;
    result.left_cross_detected = roadType_.L_Cross;
    result.right_cross_detected = roadType_.R_Cross;
    result.cross_detected = roadType_.Cross;
    result.fork_detected = roadType_.Fork;
    result.straight_detected = roadType_.straight;
    result.bend_detected = roadType_.bend;

    if (roadType_.LeftCirque)
        result.current = TrackElementResult::LeftRoundabout;
    else if (roadType_.RightCirque)
        result.current = TrackElementResult::RightRoundabout;
    else if (roadType_.L_Cross)
        result.current = TrackElementResult::LeftCross;
    else if (roadType_.R_Cross)
        result.current = TrackElementResult::RightCross;
    else if (roadType_.Fork)
        result.current = TrackElementResult::Fork;
    else if (roadType_.Cross)
        result.current = TrackElementResult::Crossroad;
    else if (roadType_.bend)
    {
        if (result.bend_direction < 0)
            result.current = TrackElementResult::BendLeft;
        else if (result.bend_direction > 0)
            result.current = TrackElementResult::BendRight;
        else
            result.current = TrackElementResult::Bend;
    }
    else if (roadType_.straight)
    {
        result.current = TrackElementResult::Straight;
    }

    result.sancha_x_zhengque = sanchaXBest_;
    result.sancha_y_zhengque = sanchaYBest_;
}

// 参考工程中的“直道长度”估计，用于后续直/弯判定。
void Camera::Check_Zhidao(const LineSearchResult& lineResult, TrackElementResult& result)
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    const int imgW = InferImageWidthFromLines(lineResult);
    int inc = 0;
    int dec = 0;

    for (int y = H - 1; y > 1; --y)
    {
        if (LeftLineFoundAt(lineResult, y) &&
            LeftLineFoundAt(lineResult, y - 1) &&
            LeftLineXAt(lineResult, y) <= LeftLineXAt(lineResult, y - 1))
        {
            inc++;
        }
        if (RightLineFoundAt(lineResult, y, imgW) &&
            RightLineFoundAt(lineResult, y - 1, imgW) &&
            RightLineXAt(lineResult, y) >= RightLineXAt(lineResult, y - 1))
        {
            dec++;
        }
    }

    result.straight_factor = (inc >= dec) ? dec : inc;
}

// 在图像中线位置向上搜索黑段，用于估计可视赛道长度。
void Camera::Mid_Col(const cv::Mat& binary, TrackElementResult& result) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int mid = binary.cols / 2;
    int y = H - 1;
    for (; y > 1; --y)
    {
        if (binary.at<uchar>(y, mid) == 0 &&
            binary.at<uchar>(y - 1, mid) == 0 &&
            binary.at<uchar>(y - 2, mid) == 0)
        {
            break;
        }
    }
    result.track_length = H - y;
}

// 简单出界保护：若底部赛道区间内出现过宽黑块，则置保护标记。
void Camera::Outside_protect(const cv::Mat& binary, const LineSearchResult& lineResult,
                             TrackElementResult& result) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H <= 0 || W <= 0)
        return;

    const int left = ClampInt(LeftLineXAt(lineResult, H - 1), 0, W - 1);
    const int right = ClampInt(RightLineXAt(lineResult, H - 1), 0, W - 1);
    int blackCount = 0;
    for (int x = left; x < right; ++x)
    {
        if (binary.at<uchar>(H - 1, x) == 0)
        {
            blackCount++;
            if (blackCount > H)
            {
                result.outside_protected = true;
                break;
            }
        }
    }
}

// 参考工程中 Tututu 的等价实现，用于抑制某些误识别场景。
bool Camera::Tututu(uint8_t type, const LineSearchResult& lineResult, int imgW) const
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    if (H < 32)
        return false;

    const int yStart = H - 1;
    const int yStop = ScaleLegacyRow(30, H);
    if (type == 1)
    {
        for (int y = yStart; y >= yStop; y -= 4)
        {
            if (y - 10 < 0)
                break;
            if (!RightLineFoundAt(lineResult, y, imgW) ||
                !RightLineFoundAt(lineResult, y - 1, imgW) ||
                !RightLineFoundAt(lineResult, y - 10, imgW))
            {
                continue;
            }

            if (std::abs(RightLineXAt(lineResult, y) - RightLineXAt(lineResult, y - 1)) < 4 &&
                (RightLineXAt(lineResult, y) - RightLineXAt(lineResult, y - 10)) < -ScaleLegacyX(8, imgW))
            {
                return true;
            }
        }
    }
    else if (type == 2)
    {
        for (int y = yStart; y >= yStop; y -= 4)
        {
            if (y - 10 < 0)
                break;
            if (!LeftLineFoundAt(lineResult, y) ||
                !LeftLineFoundAt(lineResult, y - 1) ||
                !LeftLineFoundAt(lineResult, y - 10))
            {
                continue;
            }

            if (std::abs(LeftLineXAt(lineResult, y) - LeftLineXAt(lineResult, y - 1)) < 4 &&
                (LeftLineXAt(lineResult, y) - LeftLineXAt(lineResult, y - 10)) > ScaleLegacyX(8, imgW))
            {
                return true;
            }
        }
    }
    return false;
}

// 元素识别主逻辑：
// 根据丢线、曲率、前瞻量和圆环弧线等特征，更新跨帧 roadType_ 状态。
void Camera::Element_Test(const cv::Mat& binary, const LineSearchResult& lineResult,
                          TrackElementResult& result)
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H < 50 || W < 20)
        return;

    ComputeLoseMetrics(lineResult, W, result.left_lose_value, result.right_lose_value,
                       result.left_losemax, result.right_losemax);
    Mid_Col(binary, result);
    Outside_protect(binary, lineResult, result);
    Check_Zhidao(lineResult, result);

    const int y1 = H - 10;
    const int y2 = H - 25;
    const int y3 = H - 40;
    if (y3 >= 0 &&
        LeftLineFoundAt(lineResult, y1) && LeftLineFoundAt(lineResult, y2) && LeftLineFoundAt(lineResult, y3))
    {
        result.left_curvature = 1000.0 * SignedCurvature(
            LeftLineXAt(lineResult, y1), y1,
            LeftLineXAt(lineResult, y2), y2,
            LeftLineXAt(lineResult, y3), y3);
    }
    if (y3 >= 0 &&
        RightLineFoundAt(lineResult, y1, W) && RightLineFoundAt(lineResult, y2, W) && RightLineFoundAt(lineResult, y3, W))
    {
        result.right_curvature = 1000.0 * SignedCurvature(
            RightLineXAt(lineResult, y1), y1,
            RightLineXAt(lineResult, y2), y2,
            RightLineXAt(lineResult, y3), y3);
    }

    Sancha_didian(binary, result);
    Qianzhang(binary, result);

    const bool leftRoundaboutBase = detectRoundabout(binary, lineResult, result, 1);
    const bool rightRoundaboutBase = detectRoundabout(binary, lineResult, result, 2);
    const bool leftArcStrong = RoundaboutGetArc(1, ScaleLegacyCount(15, H), lineResult, result);
    const bool rightArcStrong = RoundaboutGetArc(2, ScaleLegacyCount(15, H), lineResult, result);
    const bool leftTututu = Tututu(1, lineResult, W);
    const bool rightTututu = Tututu(2, lineResult, W);

    if (!roadType_.Fork && !roadType_.LeftCirque && !roadType_.RightCirque &&
        !roadType_.L_Cross && !roadType_.R_Cross && !roadType_.Cross &&
        !roadType_.Barn_l_in && !roadType_.Barn_r_in)
    {
        if (result.track_length > ScaleLegacyCount(50, H) &&
            result.left_lose_value > ScaleLegacyCount(30, H) &&
            result.right_lose_value < ScaleLegacyCount(14, H) &&
            std::fabs(result.right_curvature) < 15.0 &&
            !leftTututu)
        {
            if ((Cirque_or_Cross(1, result.left_losemax, binary, lineResult, result) >= ScaleLegacyCount(15, H) ||
                 leftRoundaboutBase) &&
                !roadType_.LeftCirque && !roadType_.L_Cross)
            {
                roadType_.LeftCirque = true;
                roadType_.straight = false;
                roadType_.bend = false;
            }
            else if (!roadType_.L_Cross && !roadType_.LeftCirque && !roadType_.Fork &&
                     RightLineXAt(lineResult, ClampInt(ScaleLegacyRow(50, H), 0, H - 1)) < ScaleLegacyX(100, W) &&
                     RightLineXAt(lineResult, ClampInt(ScaleLegacyRow(53, H), 0, H - 1)) < ScaleLegacyX(100, W))
            {
                roadType_.L_Cross = true;
                roadType_.straight = false;
                roadType_.bend = false;
            }
        }
        else if (result.track_length > ScaleLegacyCount(50, H) &&
                 result.right_lose_value > ScaleLegacyCount(30, H) &&
                 result.left_lose_value < ScaleLegacyCount(14, H) &&
                 std::fabs(result.left_curvature) < 15.0 &&
                 !rightTututu)
        {
            if ((Cirque_or_Cross(2, result.right_losemax, binary, lineResult, result) >= ScaleLegacyCount(10, H) ||
                 rightRoundaboutBase) &&
                !roadType_.RightCirque && !roadType_.R_Cross)
            {
                roadType_.RightCirque = true;
                roadType_.straight = false;
                roadType_.bend = false;
            }
            else if (!roadType_.R_Cross && !roadType_.RightCirque && !roadType_.Fork &&
                     LeftLineXAt(lineResult, ClampInt(ScaleLegacyRow(50, H), 0, H - 1)) > ScaleLegacyX(100, W) &&
                     LeftLineXAt(lineResult, ClampInt(ScaleLegacyRow(53, H), 0, H - 1)) > ScaleLegacyX(100, W))
            {
                roadType_.R_Cross = true;
                roadType_.straight = false;
                roadType_.bend = false;
            }
        }
        else if (leftArcStrong && rightArcStrong)
        {
            roadType_.Fork = true;
            roadType_.straight = false;
            roadType_.bend = false;
        }
        else if (result.track_length > ScaleLegacyCount(60, H) &&
                 result.right_lose_value > ScaleLegacyCount(20, H) &&
                 result.left_lose_value > ScaleLegacyCount(20, H))
        {
            roadType_.Cross = true;
            roadType_.straight = false;
            roadType_.bend = false;
        }
        else if (result.straight_factor <= ScaleLegacyCount(65, H))
        {
            roadType_.straight = false;
            roadType_.bend = true;

            const int quarterY = ClampInt(H * 2 / 3, 0, H - 1);
            const int bottomY = ClampInt(H * 5 / 6, 0, H - 1);
            const int quarterPoint = (LeftLineXAt(lineResult, quarterY) + RightLineXAt(lineResult, quarterY)) / 2;
            const int bottomPoint = (LeftLineXAt(lineResult, bottomY) + RightLineXAt(lineResult, bottomY)) / 2;

            if (quarterPoint < bottomPoint)
                result.bend_direction = -1;
            else if (quarterPoint > bottomPoint)
                result.bend_direction = 1;
            else
                result.bend_direction = 0;
        }
        else
        {
            roadType_.straight = true;
            roadType_.bend = false;
        }
    }

    syncRoadStateToResult(result);

    if (roadType_.Fork && result.sancha_y > 0)
    {
        if (result.sancha_y > sanchaYBest_)
        {
            sanchaYBest_ = result.sancha_y;
            sanchaXBest_ = result.sancha_x;
        }
        result.sancha_y_zhengque = sanchaYBest_;
        result.sancha_x_zhengque = sanchaXBest_;
    }
}

// -------------------- 环岛/十字/三岔等特征提取 --------------------
void Camera::Diuxian_weizhi_test(uint8_t type, int startline, int endline,
                                 const LineSearchResult& lineResult, TrackElementResult& result) const
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    if (H <= 0)
        return;

    const int imgW = InferImageWidthFromLines(lineResult);
    int start = ClampInt(startline, 0, H - 1);
    int end = ClampInt(endline, 0, H - 1);
    if (start < end)
        std::swap(start, end);

    int foundRows = 0;
    for (int y = start; y >= end; --y)
    {
        if (type == 1)
            foundRows += LeftLineFoundAt(lineResult, y) ? 1 : 0;
        else if (type == 2)
            foundRows += RightLineFoundAt(lineResult, y, imgW) ? 1 : 0;
    }

    const int totalRows = start - end + 1;
    const int lostRows = totalRows - foundRows;
    if (type == 1)
    {
        result.left_budiuxian_hang = foundRows;
        result.left_diuxian_hang = lostRows;
    }
    else if (type == 2)
    {
        result.right_budiuxian_hang = foundRows;
        result.right_diuxian_hang = lostRows;
    }
}

bool Camera::Bianxian_guaidian_num(uint8_t type, int startline, int endline,
                                   const LineSearchResult& lineResult, TrackElementResult& result) const
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    if (H < 12)
        return false;

    if (type == 1)
    {
        result.l_guaidain_x1 = 0;
        result.l_guaidain_y1 = 0;
        result.l_guaidain_x2 = 0;
        result.l_guaidain_y2 = 0;

        int start = ClampInt(startline, 11, H - 11);
        int end = ClampInt(endline, 6, H - 11);
        if (start <= end)
            return false;

        for (int y = start; y > end; --y)
        {
            if (!LeftLineFoundAt(lineResult, y) ||
                !LeftLineFoundAt(lineResult, y - 1) ||
                !LeftLineFoundAt(lineResult, y + 10))
                continue;

            if (std::abs(LeftLineXAt(lineResult, y) - LeftLineXAt(lineResult, y - 1)) < 4 &&
                (LeftLineXAt(lineResult, y) - LeftLineXAt(lineResult, y + 10) > 10))
            {
                result.l_guaidain_x1 = LeftLineXAt(lineResult, y - 5);
                result.l_guaidain_y1 = y - 5;
                return true;
            }
        }
    }
    else if (type == 2)
    {
        result.r_guaidain_x1 = 0;
        result.r_guaidain_y1 = 0;
        result.r_guaidain_x2 = 0;
        result.r_guaidain_y2 = 0;

        int start = ClampInt(startline, 10, H - 11);
        int end = ClampInt(endline, 0, H - 11);
        if (start <= end)
            return false;

        const int imgW = InferImageWidthFromLines(lineResult);
        for (int y = start; y > end; --y)
        {
            if (!RightLineFoundAt(lineResult, y, imgW) ||
                !RightLineFoundAt(lineResult, y + 1, imgW) ||
                !RightLineFoundAt(lineResult, y + 10, imgW))
                continue;

            if (std::abs(RightLineXAt(lineResult, y) - RightLineXAt(lineResult, y + 1)) < 4 &&
                (RightLineXAt(lineResult, y) - RightLineXAt(lineResult, y + 10) < -10))
            {
                result.r_guaidain_x1 = RightLineXAt(lineResult, y);
                result.r_guaidain_y1 = y;
                return true;
            }
        }
    }

    return false;
}

void Camera::Bianxian_guaidian(uint8_t type, int startline, int endline,
                               const LineSearchResult& lineResult, TrackElementResult& result) const
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    if (H < 5)
        return;

    if (type == 1)
    {
        result.l_guaidain_x = 0;
        result.l_guaidain_y = 0;

        if (!LeftLineFoundAt(lineResult, H - 1))
            return;

        int start = ClampInt(startline, 4, H - 2);
        int end = ClampInt(endline, 3, H - 2);
        for (int y = start; y > end; --y)
        {
            if (!LeftLineFoundAt(lineResult, y) ||
                !LeftLineFoundAt(lineResult, y + 1) ||
                !LeftLineFoundAt(lineResult, y - 3))
                continue;

            if (std::abs(LeftLineXAt(lineResult, y) - LeftLineXAt(lineResult, y + 1)) < 4 &&
                (LeftLineXAt(lineResult, y) - LeftLineXAt(lineResult, y - 3) > 8))
            {
                result.l_guaidain_y = y;
                result.l_guaidain_x = LeftLineXAt(lineResult, y);
                break;
            }
        }
    }
    else if (type == 2)
    {
        result.r_guaidain_x = 0;
        result.r_guaidain_y = 0;

        const int imgW = InferImageWidthFromLines(lineResult);
        if (!RightLineFoundAt(lineResult, H - 1, imgW))
            return;

        int start = ClampInt(startline, 4, H - 2);
        int end = ClampInt(endline, 3, H - 2);
        for (int y = start; y > end; --y)
        {
            if (!RightLineFoundAt(lineResult, y, imgW) ||
                !RightLineFoundAt(lineResult, y + 1, imgW) ||
                !RightLineFoundAt(lineResult, y - 3, imgW))
                continue;

            if (std::abs(RightLineXAt(lineResult, y) - RightLineXAt(lineResult, y + 1)) < 4 &&
                (RightLineXAt(lineResult, y) - RightLineXAt(lineResult, y - 3) < -8))
            {
                result.r_guaidain_y = y;
                result.r_guaidain_x = RightLineXAt(lineResult, y);
                break;
            }
        }
    }
}

bool Camera::RoundaboutGetArc(uint8_t type, int num, const LineSearchResult& lineResult,
                              TrackElementResult& result) const
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    if (H < 12 || num <= 0)
        return false;

    int inc = 0;
    int dec = 0;
    const int imgW = InferImageWidthFromLines(lineResult);

    if (type == 1)
    {
        result.bianxian_guaidian_l = 0;
        for (int i = H - 1; i > 10; --i)
        {
            if (LeftLineFoundAt(lineResult, i) && LeftLineFoundAt(lineResult, i - 1))
            {
                if (inc < num)
                {
                    if (LeftLineXAt(lineResult, i) < LeftLineXAt(lineResult, i - 1))
                        inc++;
                }
                else if (LeftLineXAt(lineResult, i) > LeftLineXAt(lineResult, i - 1))
                {
                    dec++;
                }

                if (inc >= num && dec >= num)
                {
                    result.bianxian_guaidian_l = ClampInt(i + num, 0, H - 1);
                    return true;
                }
            }
            else
            {
                inc = 0;
                dec = 0;
            }
        }
    }
    else if (type == 2)
    {
        result.bianxian_guaidian_r = 0;
        for (int i = H - 1; i > 10; --i)
        {
            if (RightLineFoundAt(lineResult, i, imgW) && RightLineFoundAt(lineResult, i - 1, imgW))
            {
                if (inc < num)
                {
                    if (RightLineXAt(lineResult, i) > RightLineXAt(lineResult, i - 1))
                        inc++;
                }
                else if (RightLineXAt(lineResult, i) < RightLineXAt(lineResult, i - 1))
                {
                    dec++;
                }

                if (inc >= num && dec >= num)
                {
                    result.bianxian_guaidian_r = ClampInt(i + num, 0, H - 1);
                    return true;
                }
            }
            else
            {
                inc = 0;
                dec = 0;
            }
        }
    }

    return false;
}

bool Camera::detectRoundabout(const cv::Mat& binary, const LineSearchResult& lineResult,
                              TrackElementResult& result, uint8_t type) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return false;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H < 20 || W < 8)
        return false;

    Diuxian_weizhi_test(type, H - 1, std::max(10, H / 2), lineResult, result);
    const bool hasCorner1 = Bianxian_guaidian_num(type, H - 11, 10, lineResult, result);
    Bianxian_guaidian(type, H - 2, 10, lineResult, result);
    const bool hasArc = RoundaboutGetArc(type, 3, lineResult, result);

    bool basePattern = false;
    if (type == 1)
    {
        int y1 = -1;
        for (int i = H - 1; i > std::max(1, H - 20); --i)
        {
            if (!LeftLineFoundAt(lineResult, i) && !LeftLineFoundAt(lineResult, i - 1))
            {
                y1 = i;
                break;
            }
        }

        if (y1 >= 12)
        {
            for (int i = ClampInt(y1 - 10, 12, H - 1); i > 10; --i)
            {
                if (binary.at<uchar>(i, 0) == 0 &&
                    binary.at<uchar>(i - 1, 0) == 0 &&
                    binary.at<uchar>(i - 2, 0) == 0)
                {
                    y1 = i;
                    break;
                }
            }

            for (int i = y1; i > 10; --i)
            {
                if (i - 6 < 0)
                    break;
                if (!LeftLineFoundAt(lineResult, i) ||
                    !LeftLineFoundAt(lineResult, i - 2) ||
                    !LeftLineFoundAt(lineResult, i - 4) ||
                    !LeftLineFoundAt(lineResult, i - 6))
                    continue;

                if ((LeftLineXAt(lineResult, i - 2) - LeftLineXAt(lineResult, i) < 10) &&
                    (LeftLineXAt(lineResult, i - 4) - LeftLineXAt(lineResult, i) < 20) &&
                    (LeftLineXAt(lineResult, i - 6) - LeftLineXAt(lineResult, i) < 30))
                {
                    basePattern = true;
                    break;
                }
            }
        }

        result.left_roundabout_detected =
            basePattern && (hasArc || hasCorner1 || result.l_guaidain_y != 0);
        return result.left_roundabout_detected;
    }

    if (type == 2)
    {
        const int imgW = InferImageWidthFromLines(lineResult);
        int y1 = -1;
        for (int i = H - 1; i > std::max(1, H - 20); --i)
        {
            if (!RightLineFoundAt(lineResult, i, imgW) && !RightLineFoundAt(lineResult, i - 1, imgW))
            {
                y1 = i;
                break;
            }
        }

        if (y1 >= 12)
        {
            for (int i = ClampInt(y1 - 10, 12, H - 1); i > 10; --i)
            {
                if (binary.at<uchar>(i, W - 1) == 0 &&
                    binary.at<uchar>(i - 1, W - 1) == 0 &&
                    binary.at<uchar>(i - 2, W - 1) == 0)
                {
                    y1 = i;
                    break;
                }
            }

            for (int i = y1; i > 10; --i)
            {
                if (i - 6 < 0)
                    break;
                if (!RightLineFoundAt(lineResult, i, imgW) ||
                    !RightLineFoundAt(lineResult, i - 2, imgW) ||
                    !RightLineFoundAt(lineResult, i - 4, imgW) ||
                    !RightLineFoundAt(lineResult, i - 6, imgW))
                    continue;

                if ((RightLineXAt(lineResult, i) - RightLineXAt(lineResult, i - 2) < 10) &&
                    (RightLineXAt(lineResult, i) - RightLineXAt(lineResult, i - 4) < 20) &&
                    (RightLineXAt(lineResult, i) - RightLineXAt(lineResult, i - 6) < 30))
                {
                    basePattern = true;
                    break;
                }
            }
        }

        result.right_roundabout_detected =
            basePattern && (hasArc || hasCorner1 || result.r_guaidain_y != 0);
        return result.right_roundabout_detected;
    }

    return false;
}

// -------------------- 元素处理状态机与补线 --------------------
void Camera::drawTrackElementsOnTFT(const TrackElementResult& result) const
{
    char buf[32];
    const char* elementName = "None";

    if (result.current == TrackElementResult::LeftRoundabout)
        elementName = "L-Round";
    else if (result.current == TrackElementResult::RightRoundabout)
        elementName = "R-Round";
    else if (result.current == TrackElementResult::LeftCross)
        elementName = "L-Cross";
    else if (result.current == TrackElementResult::RightCross)
        elementName = "R-Cross";
    else if (result.current == TrackElementResult::Crossroad)
        elementName = "Cross";
    else if (result.current == TrackElementResult::Fork)
        elementName = "Fork";
    else if (result.current == TrackElementResult::Straight)
        elementName = "Straight";
    else if (result.current == TrackElementResult::BendLeft)
        elementName = "Bend-L";
    else if (result.current == TrackElementResult::BendRight)
        elementName = "Bend-R";
    else if (result.current == TrackElementResult::Bend)
        elementName = "Bend";
    else if (result.current == TrackElementResult::ZebraCrossing)
        elementName = "Zebra";

    std::snprintf(buf, sizeof(buf), "Elem:%-10s", elementName);
    TFT_ShowTextBottomLeft(buf, 0);
}

// 清理上一阶段生成的补线，让 neo 数组重新与原始搜线结果同步。
void Camera::Clear_Supplement_Lines(LineSearchResult& lineResult)
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    const int W = InferImageWidthFromLines(lineResult);
    EnsureLineToolResultSize(lineToolResult_, W, H);

    for (int y = 0; y < H; ++y)
    {
        const size_t idx = static_cast<size_t>(y);
        lineToolResult_.l_second_line_x[idx] = 0;
        lineToolResult_.r_second_line_x[idx] = W - 1;
        lineToolResult_.neo_l_line_x[idx] = lineResult.left_x[idx];
        lineToolResult_.neo_r_line_x[idx] = lineResult.right_x[idx];
    }
}

// 十字拐点检测：寻找上下两个关键拐点，为十字补线提供连接端点。
void Camera::Check_Cross_Guaidian(uint8_t type, const LineSearchResult& lineResult)
{
    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    const int W = InferImageWidthFromLines(lineResult);
    if (H < 20 || W < 20)
        return;

    if (type == 1)
        crossLeft_ = { 0, 0 };
    else if (type == 2)
        crossRight_ = { 0, 0 };

    const int mid = W / 2;
    const int lowerStart = ClampInt(ScaleLegacyRow(105, H), 0, H - 1);
    const int lowerStop = ClampInt(ScaleLegacyRow(70, H), 0, H - 1);
    const int upperStart = ClampInt(ScaleLegacyRow(25, H), 0, H - 1);

    if (type == 1)
    {
        for (int y1 = lowerStart; y1 >= lowerStop; --y1)
        {
            if (y1 + 1 >= H || y1 - 5 < 0)
                continue;
            for (int x = mid; x > ScaleLegacyX(5, W); --x)
            {
                if (LeftLineXAt(lineResult, y1) == x &&
                    LeftLineFoundAt(lineResult, y1 + 1) &&
                    std::abs(LeftLineXAt(lineResult, y1) - LeftLineXAt(lineResult, y1 + 1)) < 4 &&
                    LeftLineFoundAt(lineResult, y1 - 5) &&
                    (LeftLineXAt(lineResult, y1) - LeftLineXAt(lineResult, y1 - 5)) > ScaleLegacyX(10, W))
                {
                    crossLeft_[0] = y1;
                    break;
                }
            }
            if (crossLeft_[0] != 0)
                break;
        }

        const int upperStop = (crossLeft_[0] != 0) ? std::max(upperStart, crossLeft_[0] - ScaleLegacyCount(15, H))
                                                   : ClampInt(ScaleLegacyRow(95, H), 0, H - 1);
        for (int y2 = upperStart; y2 < upperStop; ++y2)
        {
            if (y2 - 1 < 0 || y2 + 5 >= H || y2 + 3 >= H)
                continue;
            for (int x = mid; x > ScaleLegacyX(5, W); --x)
            {
                if (LeftLineXAt(lineResult, y2) == x &&
                    std::abs(LeftLineXAt(lineResult, y2) - LeftLineXAt(lineResult, y2 - 1)) < 4 &&
                    ((LeftLineFoundAt(lineResult, y2 + 5) &&
                      (LeftLineXAt(lineResult, y2) - LeftLineXAt(lineResult, y2 + 5)) > ScaleLegacyX(20, W)) ||
                     !LeftLineFoundAt(lineResult, y2 + 3)))
                {
                    crossLeft_[1] = ClampInt(y2 + ScaleLegacyCount(2, H), 0, H - 1);
                    break;
                }
            }
            if (crossLeft_[1] != 0)
                break;
        }
    }
    else if (type == 2)
    {
        for (int y1 = lowerStart; y1 >= lowerStop; --y1)
        {
            if (y1 + 1 >= H || y1 - 5 < 0)
                continue;
            for (int x = mid; x < W - ScaleLegacyX(5, W); ++x)
            {
                if (RightLineXAt(lineResult, y1) == x &&
                    RightLineFoundAt(lineResult, y1 + 1, W) &&
                    std::abs(RightLineXAt(lineResult, y1) - RightLineXAt(lineResult, y1 + 1)) < 4 &&
                    RightLineFoundAt(lineResult, y1 - 5, W) &&
                    (RightLineXAt(lineResult, y1) - RightLineXAt(lineResult, y1 - 5)) < -ScaleLegacyX(10, W))
                {
                    crossRight_[0] = y1;
                    break;
                }
            }
            if (crossRight_[0] != 0)
                break;
        }

        const int upperStop = (crossRight_[0] != 0) ? std::max(upperStart, crossRight_[0] - ScaleLegacyCount(15, H))
                                                    : ClampInt(ScaleLegacyRow(95, H), 0, H - 1);
        for (int y2 = upperStart; y2 < upperStop; ++y2)
        {
            if (y2 - 10 < 0 || y2 + 5 >= H || y2 + 3 >= H)
                continue;
            for (int x = mid; x < W - ScaleLegacyX(5, W); ++x)
            {
                if (RightLineXAt(lineResult, y2) == x &&
                    ((RightLineFoundAt(lineResult, y2, W) &&
                      std::abs(RightLineXAt(lineResult, y2) - RightLineXAt(lineResult, y2 - 1)) < 4 &&
                      RightLineFoundAt(lineResult, y2 + 5, W) &&
                      (RightLineXAt(lineResult, y2) - RightLineXAt(lineResult, y2 + 5)) < -ScaleLegacyX(20, W)) ||
                     (RightLineFoundAt(lineResult, y2 - 10, W) &&
                      std::abs(RightLineXAt(lineResult, y2) - RightLineXAt(lineResult, y2 - 10)) < 4 &&
                      !RightLineFoundAt(lineResult, y2 + 3, W))))
                {
                    crossRight_[1] = y2;
                    break;
                }
            }
            if (crossRight_[1] != 0)
                break;
        }
    }
}

// 按参考工程优先级依次处理车库、环岛、斜十字、三岔和十字状态机。
void Camera::Element_Handle(LineSearchResult& lineResult, TrackElementResult& result)
{
    if (roadType_.Barn_l_in)
    {
        Handle_Barn_in(1, lineResult, result);
        return;
    }
    if (roadType_.Barn_r_in)
    {
        Handle_Barn_in(2, lineResult, result);
        return;
    }
    if (roadType_.LeftCirque)
    {
        Handle_Left_Cirque(lineResult, result);
        return;
    }
    if (roadType_.RightCirque)
    {
        Handle_Right_Cirque(lineResult, result);
        return;
    }
    if (roadType_.L_Cross)
    {
        Handle_L_Cross(lineResult, result);
        return;
    }
    if (roadType_.R_Cross)
    {
        Handle_R_Cross(lineResult, result);
        return;
    }
    if (roadType_.Fork)
    {
        Handle_Fork(lineResult, result);
        return;
    }
    if (roadType_.Cross)
        Handle_Cross(lineResult, result);
}

// 左环岛状态机：判断进环、环内、出环各阶段，并写入左/右补线。
void Camera::Handle_Left_Cirque(LineSearchResult& lineResult, TrackElementResult& result)
{
    if (pauseCameraProcess_)
        return;

    const int H = static_cast<int>(lineResult.left_x.size());
    const int W = InferImageWidthFromLines(lineResult);
    if (H <= 0 || W <= 0)
        return;

    if (inCarHahaMode_ && huandaoStage_ < 5)
    {
        huandaoStage_ = 5;
        annulusS1_ = 0;
    }

    switch (huandaoStage_)
    {
    case 1:
        Diuxian_weizhi_test(1, H - 1, ScaleLegacyRow(100, H), lineResult, result);
        if (FoundRowsRef(1, result) >= 1)
        {
            Diuxian_weizhi_test(1, ScaleLegacyRow(100, H), ScaleLegacyRow(70, H), lineResult, result);
            if (LostRowsRef(1, result) >= ScaleLegacyCount(10, H))
                huandaoStage_ = 2;
        }
        break;
    case 2:
        Diuxian_weizhi_test(1, H - 1, ScaleLegacyRow(90, H), lineResult, result);
        if (LostRowsRef(1, result) >= ScaleLegacyCount(20, H))
        {
            Diuxian_weizhi_test(1, ScaleLegacyRow(90, H), ScaleLegacyRow(50, H), lineResult, result);
            if (FoundRowsRef(1, result) >= ScaleLegacyCount(30, H))
                huandaoStage_ = 3;
        }
        break;
    case 3:
        Diuxian_weizhi_test(1, H - 1, ScaleLegacyRow(90, H), lineResult, result);
        if (Bianxian_guaidian_num(1, ScaleLegacyRow(100, H), ScaleLegacyRow(30, H), lineResult, result))
            huandaoStage_ = 4;
        else if (FoundRowsRef(1, result) >= ScaleLegacyCount(10, H))
        {
            Diuxian_weizhi_test(1, ScaleLegacyRow(100, H), ScaleLegacyRow(60, H), lineResult, result);
            if (LostRowsRef(1, result) >= ScaleLegacyCount(25, H))
                huandaoStage_ = 4;
        }
        break;
    case 4:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > 1000)
        {
            huandaoStage_ = 5;
            annulusS1_ = 0;
        }
        break;
    case 5:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > (inCarHahaMode_ ? 300 : 700))
        {
            Diuxian_weizhi_test(1, ScaleLegacyRow(50, H), ScaleLegacyRow(30, H), lineResult, result);
            if (LostRowsRef(1, result) >= ScaleLegacyCount(15, H))
            {
                Diuxian_weizhi_test(2, ScaleLegacyRow(50, H), ScaleLegacyRow(30, H), lineResult, result);
                if (LostRowsRef(2, result) >= ScaleLegacyCount(10, H))
                    huandaoStage_ = 6;
            }
        }
        break;
    case 6:
    {
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        const bool leftCorner = Bianxian_guaidian_num(1, ScaleLegacyRow(100, H), ScaleLegacyRow(30, H),
                                                      lineResult, result);
        static int leftValidConditionCount = 0;
        if (leftCorner)
            leftValidConditionCount++;
        else
            leftValidConditionCount = 0;

        if (((leftValidConditionCount >= 2) && annulusS1_ > 500) ||
            annulusS1_ > (inCarHahaMode_ ? 30000 : 1200))
        {
            huandaoStage_ = 7;
            annulusS1_ = 0;
            leftValidConditionCount = 0;
        }
        break;
    }
    case 7:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > 1000)
        {
            Clear_Supplement_Lines(lineResult);
            roadType_.LeftCirque = false;
            huandaoStage_ = 1;
            annulusS1_ = 0;
            inCarHahaMode_ = false;
            pushedBoxInRoundabout_ = false;
            disableElementDetection_ = false;
        }
        break;
    default:
        huandaoStage_ = 1;
        break;
    }

    if (huandaoStage_ == 1)
    {
        for (int y = ScaleLegacyRow(110, H); y >= ScaleLegacyRow(30, H); --y)
        {
            const int newLeft = ClampInt(lineResult.right_x[static_cast<size_t>(y)] -
                                             TrackWidthAt(y, W, H) + ScaleLegacyX(5, W),
                                         0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
        }
    }
    else if (huandaoStage_ == 2 || huandaoStage_ == 3)
    {
        for (int y = H - 1; y >= ScaleLegacyRow(60, H); --y)
        {
            const int newLeft = ClampInt(lineResult.right_x[static_cast<size_t>(y)] -
                                             TrackWidthAt(y, W, H) + ScaleLegacyX(5, W),
                                         0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
        }
    }
    else if (huandaoStage_ == 4)
    {
        Bianxian_guaidian_num(1, ScaleLegacyRow(80, H), ScaleLegacyRow(20, H), lineResult, result);
        int rightPointY = ScaleLegacyRow(10, H);
        if (result.l_guaidain_x1 != 0 && result.l_guaidain_y1 != 0)
        {
            const int quarterHeight = H - 1 - (H - 1 - ScaleLegacyRow(10, H)) / 3;
            const int rightPointX = ClampInt(lineResult.right_x[static_cast<size_t>(quarterHeight)] -
                                                 ScaleLegacyX(30, W),
                                             0, W - 1);
            rightPointY = quarterHeight;
            if (rightPointX > 0 && rightPointX < W)
                La_zhixian(2, result.l_guaidain_x1, result.l_guaidain_y1,
                           rightPointX, rightPointY, lineResult, lineToolResult_);
        }
        for (int y = H - 1; y >= rightPointY; --y)
        {
            const int newRight = ClampInt(lineResult.right_x[static_cast<size_t>(y)] - ScaleLegacyX(35, W),
                                          0, W - 1);
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = newRight;
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = newRight;
        }
        if (rightPointY < H - 1 &&
            Regression(2, rightPointY, H - 1, lineResult, lineToolResult_))
        {
            Hua_Xian(2, rightPointY, H - 1,
                     lineToolResult_.parameterB, lineToolResult_.parameterA,
                     lineToolResult_, W, H);
        }
    }
    else if (huandaoStage_ == 5)
    {
        for (int y = H - 1; y > ScaleLegacyRow(10, H); --y)
        {
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = lineResult.left_x[static_cast<size_t>(y)];
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
        }
    }
    else if (huandaoStage_ == 6)
    {
        for (int y = ClampInt(H / 6, 0, H - 1); y < H; ++y)
        {
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] =
                ClampInt(lineResult.left_x[static_cast<size_t>(y)] + TrackWidthAt(y, W, H) - ScaleLegacyX(20, W),
                         0, W - 1);
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = ScaleLegacyX(128, W);
        }
    }
    else if (huandaoStage_ == 7)
    {
        for (int y = H - 1; y > ScaleLegacyRow(20, H); --y)
        {
            const int newLeft = ClampInt(lineResult.right_x[static_cast<size_t>(y)] - TrackWidthAt(y, W, H),
                                         0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
        }
    }
}

// 右环岛状态机：逻辑与左环岛镜像，补线方向相反。
void Camera::Handle_Right_Cirque(LineSearchResult& lineResult, TrackElementResult& result)
{
    if (pauseCameraProcess_)
        return;

    const int H = static_cast<int>(lineResult.right_x.size());
    const int W = InferImageWidthFromLines(lineResult);
    if (H <= 0 || W <= 0)
        return;

    if (inCarHahaMode_ && huandaoStage_ < 5)
    {
        huandaoStage_ = 5;
        annulusS1_ = 0;
    }

    switch (huandaoStage_)
    {
    case 1:
        Diuxian_weizhi_test(2, H - 1, ScaleLegacyRow(100, H), lineResult, result);
        if (FoundRowsRef(2, result) >= 1)
        {
            Diuxian_weizhi_test(2, ScaleLegacyRow(100, H), ScaleLegacyRow(70, H), lineResult, result);
            if (LostRowsRef(2, result) >= ScaleLegacyCount(10, H))
                huandaoStage_ = 2;
        }
        break;
    case 2:
        Diuxian_weizhi_test(2, H - 1, ScaleLegacyRow(90, H), lineResult, result);
        if (LostRowsRef(2, result) >= ScaleLegacyCount(10, H))
        {
            Diuxian_weizhi_test(2, ScaleLegacyRow(90, H), ScaleLegacyRow(50, H), lineResult, result);
            if (FoundRowsRef(2, result) >= ScaleLegacyCount(5, H))
                huandaoStage_ = 3;
        }
        break;
    case 3:
        Diuxian_weizhi_test(2, H - 1, ScaleLegacyRow(90, H), lineResult, result);
        if (Bianxian_guaidian_num(2, ScaleLegacyRow(100, H), ScaleLegacyRow(30, H), lineResult, result))
            huandaoStage_ = 4;
        else if (FoundRowsRef(2, result) >= ScaleLegacyCount(10, H))
        {
            Diuxian_weizhi_test(2, ScaleLegacyRow(100, H), ScaleLegacyRow(60, H), lineResult, result);
            if (LostRowsRef(2, result) >= 1)
                huandaoStage_ = 4;
        }
        break;
    case 4:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > 1000)
        {
            huandaoStage_ = 5;
            annulusS1_ = 0;
        }
        break;
    case 5:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > (inCarHahaMode_ ? 200 : 500))
        {
            Diuxian_weizhi_test(1, ScaleLegacyRow(50, H), ScaleLegacyRow(30, H), lineResult, result);
            if (LostRowsRef(1, result) >= ScaleLegacyCount(15, H))
            {
                Diuxian_weizhi_test(2, ScaleLegacyRow(50, H), ScaleLegacyRow(30, H), lineResult, result);
                if (LostRowsRef(2, result) >= ScaleLegacyCount(10, H))
                    huandaoStage_ = 6;
            }
        }
        break;
    case 6:
    {
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        const bool rightCorner = Bianxian_guaidian_num(2, ScaleLegacyRow(100, H), ScaleLegacyRow(30, H),
                                                       lineResult, result);
        static int rightValidConditionCount = 0;
        if (rightCorner)
            rightValidConditionCount++;
        else
            rightValidConditionCount = 0;

        if (((rightValidConditionCount >= 2) && annulusS1_ > 1500) ||
            annulusS1_ > (inCarHahaMode_ ? 30000 : 1200))
        {
            huandaoStage_ = 7;
            annulusS1_ = 0;
            rightValidConditionCount = 0;
        }
        break;
    }
    case 7:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > 1000)
        {
            Clear_Supplement_Lines(lineResult);
            roadType_.RightCirque = false;
            huandaoStage_ = 1;
            annulusS1_ = 0;
            inCarHahaMode_ = false;
            pushedBoxInRoundabout_ = false;
            disableElementDetection_ = false;
        }
        break;
    default:
        huandaoStage_ = 1;
        break;
    }

    if (huandaoStage_ == 1)
    {
        for (int y = ScaleLegacyRow(110, H); y >= ScaleLegacyRow(30, H); --y)
        {
            const int newRight = ClampInt(lineResult.left_x[static_cast<size_t>(y)] +
                                              TrackWidthAt(y, W, H) - ScaleLegacyX(5, W),
                                          0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = lineResult.left_x[static_cast<size_t>(y)];
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = newRight;
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = lineResult.left_x[static_cast<size_t>(y)];
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = newRight;
        }
    }
    else if (huandaoStage_ == 2 || huandaoStage_ == 3)
    {
        for (int y = H - 1; y >= ScaleLegacyRow(60, H); --y)
        {
            const int newRight = ClampInt(lineResult.left_x[static_cast<size_t>(y)] +
                                              TrackWidthAt(y, W, H) - ScaleLegacyX(5, W),
                                          0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = lineResult.left_x[static_cast<size_t>(y)];
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = newRight;
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = lineResult.left_x[static_cast<size_t>(y)];
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = newRight;
        }
    }
    else if (huandaoStage_ == 4)
    {
        Bianxian_guaidian_num(2, ScaleLegacyRow(80, H), ScaleLegacyRow(20, H), lineResult, result);
        int leftPointY = ScaleLegacyRow(10, H);
        if (result.r_guaidain_x1 != 0 && result.r_guaidain_y1 != 0)
        {
            const int quarterHeight = H - 1 - (H - 1 - ScaleLegacyRow(10, H)) / 3;
            const int leftPointX = ClampInt(lineResult.left_x[static_cast<size_t>(quarterHeight)] +
                                                ScaleLegacyX(40, W),
                                            0, W - 1);
            leftPointY = quarterHeight;
            if (leftPointX > 0 && leftPointX < W)
                La_zhixian(1, result.r_guaidain_x1, result.r_guaidain_y1,
                           leftPointX, leftPointY, lineResult, lineToolResult_);
        }
        for (int y = H - 1; y >= leftPointY; --y)
        {
            const int newLeft = ClampInt(lineResult.left_x[static_cast<size_t>(y)] + ScaleLegacyX(45, W),
                                         0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = newLeft;
        }
        if (leftPointY < H - 1 &&
            Regression(1, leftPointY, H - 1, lineResult, lineToolResult_))
        {
            Hua_Xian(1, leftPointY, H - 1,
                     lineToolResult_.parameterB, lineToolResult_.parameterA,
                     lineToolResult_, W, H);
        }
    }
    else if (huandaoStage_ == 5)
    {
        for (int y = H - 1; y > ScaleLegacyRow(10, H); --y)
        {
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = lineResult.left_x[static_cast<size_t>(y)];
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = lineResult.right_x[static_cast<size_t>(y)];
        }
    }
    else if (huandaoStage_ == 6)
    {
        for (int y = ClampInt(H / 6, 0, H - 1); y < H; ++y)
        {
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] =
                ClampInt(lineResult.right_x[static_cast<size_t>(y)] - TrackWidthAt(y, W, H) - ScaleLegacyX(20, W),
                         0, W - 1);
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = ScaleLegacyX(70, W);
        }
    }
    else if (huandaoStage_ == 7)
    {
        for (int y = H - 1; y > ScaleLegacyRow(20, H); --y)
        {
            const int newRight = ClampInt(lineResult.left_x[static_cast<size_t>(y)] + TrackWidthAt(y, W, H),
                                          0, W - 1);
            const int newLeft = ClampInt(lineResult.left_x[static_cast<size_t>(y)] + ScaleLegacyX(15, W),
                                         0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = newRight;
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = newRight;
        }
    }
}

// 三岔状态机：记录最佳底点并对右边线做拉线/拟合补线。
void Camera::Handle_Fork(LineSearchResult& lineResult, TrackElementResult& result)
{
    const int H = static_cast<int>(lineResult.left_x.size());
    const int W = InferImageWidthFromLines(lineResult);
    if (H <= 0 || W <= 0)
        return;

    switch (sanchaStage_)
    {
    case 1:
        if (RoundaboutGetArc(1, ScaleLegacyCount(15, H), lineResult, result) &&
            RoundaboutGetArc(2, ScaleLegacyCount(15, H), lineResult, result))
        {
            if (result.bianxian_guaidian_l >= ScaleLegacyRow(55, H) &&
                result.bianxian_guaidian_r >= ScaleLegacyRow(55, H))
            {
                sanchaStage_ = 2;
                forkS1_ = 0;
            }
        }
        break;
    case 2:
        forkS1_ += EstimateDistanceMeasure(lineResult);
        if (result.sancha_y > 0 && result.sancha_y > sanchaYBest_)
        {
            sanchaYBest_ = result.sancha_y;
            sanchaXBest_ = result.sancha_x;
        }
        if (forkS1_ >= 2500)
        {
            forkS1_ = 0;
            sanchaYBest_ = 0;
            sanchaXBest_ = 0;
            sanchaStage_ = 3;
        }
        break;
    case 3:
        if (RoundaboutGetArc(1, ScaleLegacyCount(10, H), lineResult, result) &&
            RoundaboutGetArc(2, ScaleLegacyCount(10, H), lineResult, result))
        {
            if (result.bianxian_guaidian_l >= ScaleLegacyRow(50, H) &&
                result.bianxian_guaidian_r >= ScaleLegacyRow(50, H))
            {
                sanchaStage_ = 4;
                forkS1_ = 0;
            }
        }
        break;
    case 4:
        forkS1_ += EstimateDistanceMeasure(lineResult);
        if (result.sancha_y > 0 && result.sancha_y > sanchaYBest_)
        {
            sanchaYBest_ = result.sancha_y;
            sanchaXBest_ = result.sancha_x;
        }
        if (forkS1_ >= 4000)
        {
            forkS1_ = 0;
            sanchaYBest_ = 0;
            sanchaXBest_ = 0;
            sanchaStage_ = 5;
        }
        break;
    case 5:
        roadType_.Fork = false;
        sanchaStage_ = 1;
        break;
    default:
        sanchaStage_ = 1;
        break;
    }

    if ((sanchaStage_ == 2 || sanchaStage_ == 4) && sanchaYBest_ > ScaleLegacyCount(5, H))
    {
        int targetY = sanchaYBest_;
        if (targetY > ScaleLegacyRow(80, H))
            targetY = ScaleLegacyRow(57, H);
        La_zhixian(2, W - 1, H - 1, sanchaXBest_, targetY, lineResult, lineToolResult_);
        if (targetY < H - 1 &&
            Regression(2, targetY, H - 1, lineResult, lineToolResult_))
        {
            Hua_Xian(2, targetY, H - 1,
                     lineToolResult_.parameterB, lineToolResult_.parameterA,
                     lineToolResult_, W, H);
        }
    }
}

// 左/右斜十字处理主要通过补丢失侧边线引导车辆继续通过。
void Camera::Handle_L_Cross(LineSearchResult& lineResult, TrackElementResult& result)
{
    const int H = static_cast<int>(lineResult.left_x.size());
    const int W = InferImageWidthFromLines(lineResult);
    if (H <= 0 || W <= 0)
        return;

    if (roadType_.LeftCirque || roadType_.RightCirque)
    {
        roadType_.L_Cross = false;
        leftCrossStage_ = 1;
        Clear_Supplement_Lines(lineResult);
        return;
    }

    switch (leftCrossStage_)
    {
    case 1:
        if (result.left_lose_value < ScaleLegacyCount(20, H))
            leftCrossStage_ = 2;
        break;
    case 2:
        if (std::abs(result.left_lose_value - result.right_lose_value) < ScaleLegacyCount(20, H))
            leftCrossStage_ = 3;
        break;
    case 3:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > 2000 ||
            std::abs(result.left_lose_value - result.right_lose_value) < ScaleLegacyCount(10, H))
        {
            Clear_Supplement_Lines(lineResult);
            roadType_.L_Cross = false;
            annulusS1_ = 0;
            leftCrossStage_ = 1;
        }
        break;
    default:
        leftCrossStage_ = 1;
        break;
    }

    if (leftCrossStage_ == 1 || leftCrossStage_ == 3)
    {
        for (int y = H - 1; y > ScaleLegacyRow(20, H); --y)
        {
            const int newLeft = ClampInt(lineResult.right_x[static_cast<size_t>(y)] - TrackWidthAt(y, W, H),
                                         0, W - 1);
            lineToolResult_.l_second_line_x[static_cast<size_t>(y)] = newLeft;
            lineToolResult_.neo_l_line_x[static_cast<size_t>(y)] = newLeft;
        }
    }
}

void Camera::Handle_R_Cross(LineSearchResult& lineResult, TrackElementResult& result)
{
    const int H = static_cast<int>(lineResult.right_x.size());
    const int W = InferImageWidthFromLines(lineResult);
    if (H <= 0 || W <= 0)
        return;

    if (roadType_.LeftCirque || roadType_.RightCirque)
    {
        roadType_.R_Cross = false;
        rightCrossStage_ = 1;
        Clear_Supplement_Lines(lineResult);
        return;
    }

    switch (rightCrossStage_)
    {
    case 1:
        if (result.right_lose_value < ScaleLegacyCount(20, H))
            rightCrossStage_ = 2;
        break;
    case 2:
        if (std::abs(result.left_lose_value - result.right_lose_value) < ScaleLegacyCount(20, H))
            rightCrossStage_ = 3;
        break;
    case 3:
        annulusS1_ += EstimateDistanceMeasure(lineResult);
        if (annulusS1_ > 2000 ||
            std::abs(result.left_lose_value - result.right_lose_value) < ScaleLegacyCount(10, H))
        {
            Clear_Supplement_Lines(lineResult);
            roadType_.R_Cross = false;
            annulusS1_ = 0;
            rightCrossStage_ = 1;
        }
        break;
    default:
        rightCrossStage_ = 1;
        break;
    }

    if (rightCrossStage_ == 1 || rightCrossStage_ == 3)
    {
        for (int y = H - 1; y > ScaleLegacyRow(20, H); --y)
        {
            const int newRight = ClampInt(lineResult.left_x[static_cast<size_t>(y)] + TrackWidthAt(y, W, H),
                                          0, W - 1);
            lineToolResult_.r_second_line_x[static_cast<size_t>(y)] = newRight;
            lineToolResult_.neo_r_line_x[static_cast<size_t>(y)] = newRight;
        }
    }
}

// 十字处理会先找拐点，再按阶段连接上下拐点或拉向屏幕下边界。
void Camera::Handle_Cross(LineSearchResult& lineResult, TrackElementResult& result)
{
    const int H = static_cast<int>(lineResult.left_x.size());
    const int W = InferImageWidthFromLines(lineResult);
    if (H <= 0 || W <= 0)
        return;

    crossS_ += EstimateDistanceMeasure(lineResult);
    if (crossS_ > 2000)
    {
        crossS_ = 0;
        roadType_.Cross = false;
        crossStage_ = 1;
    }

    if (roadType_.LeftCirque || roadType_.RightCirque)
    {
        roadType_.Cross = false;
        return;
    }

    Check_Cross_Guaidian(1, lineResult);
    Check_Cross_Guaidian(2, lineResult);

    switch (crossStage_)
    {
    case 1:
        if (crossLeft_[0] != 0 && crossRight_[0] != 0)
        {
            if (crossLeft_[1] != 0 && lineResult.left_x[static_cast<size_t>(crossLeft_[1])] > 0)
            {
                La_zhixian(1, lineResult.left_x[static_cast<size_t>(crossLeft_[0])], crossLeft_[0],
                           lineResult.left_x[static_cast<size_t>(crossLeft_[1])], crossLeft_[1],
                           lineResult, lineToolResult_);
            }
            if (crossRight_[1] != 0 &&
                lineResult.right_x[static_cast<size_t>(crossRight_[1])] < W - 1)
            {
                La_zhixian(2, lineResult.right_x[static_cast<size_t>(crossRight_[0])], crossRight_[0],
                           lineResult.right_x[static_cast<size_t>(crossRight_[1])], crossRight_[1],
                           lineResult, lineToolResult_);
            }
        }
        else
        {
            crossStage_ = 2;
        }
        break;
    case 2:
        if (crossLeft_[1] != 0 && lineResult.left_x[static_cast<size_t>(crossLeft_[1])] > 0)
        {
            La_zhixian(1, ScaleLegacyX(1, W), H - 1,
                       lineResult.left_x[static_cast<size_t>(crossLeft_[1])], crossLeft_[1],
                       lineResult, lineToolResult_);
        }
        if (crossRight_[1] != 0 &&
            lineResult.right_x[static_cast<size_t>(crossRight_[1])] < W - 1)
        {
            La_zhixian(2, W - 2, H - 1,
                       lineResult.right_x[static_cast<size_t>(crossRight_[1])], crossRight_[1],
                       lineResult, lineToolResult_);
        }
        if (crossLeft_[0] != 0 && crossRight_[0] != 0)
            crossStage_ = 1;
        break;
    default:
        crossStage_ = 1;
        break;
    }

    Diuxian_weizhi_test(1, H - 1, ScaleLegacyRow(5, H), lineResult, result);
    Diuxian_weizhi_test(2, H - 1, ScaleLegacyRow(5, H), lineResult, result);

    if ((result.left_lose_value < ScaleLegacyCount(5, H) &&
         result.right_lose_value < ScaleLegacyCount(5, H)) ||
        (crossLeft_[1] == 0 && crossRight_[1] == 0))
    {
        crossStage_ = 1;
        roadType_.Cross = false;
        Clear_Supplement_Lines(lineResult);
    }

    if (result.right_lose_value > ScaleLegacyCount(30, H))
    {
        roadType_.LeftCirque = false;
        roadType_.RightCirque = false;
        huandaoStage_ = 1;
    }
}

// 车库入库处理目前保留参考工程思路：按目标侧补线并以距离近似推进阶段。
void Camera::Handle_Barn_in(uint8_t type, LineSearchResult& lineResult, TrackElementResult& result)
{
    (void)result;

    const int H = std::max(static_cast<int>(lineResult.left_x.size()),
                           static_cast<int>(lineResult.right_x.size()));
    const int W = InferImageWidthFromLines(lineResult);
    if (H <= 0 || W <= 0)
        return;

    if (type == 1)
    {
        if (passBarn_ == 1)
        {
            for (int y = H - 1; y > ScaleLegacyRow(20, H); --y)
                lineResult.left_x[static_cast<size_t>(y)] =
                    ClampInt(lineResult.right_x[static_cast<size_t>(y)] - TrackWidthAt(y, W, H), 0, W - 1);
            closeCheckKuS_ += EstimateDistanceMeasure(lineResult);
            if (closeCheckKuS_ > 4000)
            {
                passBarn_++;
                roadType_.Barn_l_in = false;
                closeCheckKuS_ = 0;
            }
        }
        else if (passBarn_ == 2)
        {
            jinkuS_ += EstimateDistanceMeasure(lineResult);
            if (jinkuS_ > 4000)
                jinkuS_ = 0;
        }
    }
    else if (type == 2)
    {
        if (passBarn_ == 1)
        {
            for (int y = H - 1; y > ScaleLegacyRow(20, H); --y)
                lineResult.right_x[static_cast<size_t>(y)] =
                    ClampInt(lineResult.left_x[static_cast<size_t>(y)] + TrackWidthAt(y, W, H), 0, W - 1);
            closeCheckKuS_ += EstimateDistanceMeasure(lineResult);
            if (closeCheckKuS_ > 4000)
            {
                passBarn_++;
                roadType_.Barn_r_in = false;
                closeCheckKuS_ = 0;
            }
        }
        else if (passBarn_ == 2)
        {
            jinkuS_ += EstimateDistanceMeasure(lineResult);
            if (jinkuS_ > 4000)
                jinkuS_ = 0;
        }
    }

    initializeLineTool(lineResult, W, H);
    if (type == 1 &&
        Regression(1, ScaleLegacyRow(20, H), H - 1, lineResult, lineToolResult_))
    {
        Hua_Xian(1, ScaleLegacyRow(20, H), H - 1,
                 lineToolResult_.parameterB, lineToolResult_.parameterA,
                 lineToolResult_, W, H);
    }
    else if (type == 2 &&
             Regression(2, ScaleLegacyRow(20, H), H - 1, lineResult, lineToolResult_))
    {
        Hua_Xian(2, ScaleLegacyRow(20, H), H - 1,
                 lineToolResult_.parameterB, lineToolResult_.parameterA,
        lineToolResult_, W, H);
    }
}

// -------------------- 补线工具函数 --------------------
void Camera::Check_guaidian_cheku_1(const cv::Mat& binary, LineToolResult& toolResult, uint8_t type) const
{
    (void)type;

    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H < 3 || W < 3)
        return;

    EnsureLineToolResultSize(toolResult, W, H);
    const int defaultX = W - 1;
    const int defaultY = H - 1;
    toolResult.rightx_1[0] = defaultX;
    toolResult.rightx_1[1] = defaultX;
    toolResult.righty_1[0] = defaultY;
    toolResult.righty_1[1] = defaultY;

    const int x = W - 3;
    for (int y = H - 1; y > 10; --y)
    {
        if (binary.at<uchar>(y, x) == 0 &&
            binary.at<uchar>(y - 1, x) == 0 &&
            binary.at<uchar>(y - 2, x) == 0)
        {
            toolResult.rightx_1[1] = x;
            toolResult.righty_1[1] = y;
            break;
        }
    }
}

// 两点拉直线：同时覆盖原始边线和 neo/second 缓存，便于后续直接参与中线计算。
void Camera::La_zhixian(uint8_t type, int x_down, int y_down, int x_up, int y_up,
                        LineSearchResult& lineResult, LineToolResult& toolResult) const
{
    std::vector<int>* lineArray = nullptr;
    std::vector<int>* neoArray = nullptr;
    std::vector<int>* secondLineArray = nullptr;

    if (type == 1)
    {
        lineArray = &lineResult.left_x;
        neoArray = &toolResult.neo_l_line_x;
        secondLineArray = &toolResult.l_second_line_x;
    }
    else if (type == 2)
    {
        lineArray = &lineResult.right_x;
        neoArray = &toolResult.neo_r_line_x;
        secondLineArray = &toolResult.r_second_line_x;
    }
    else
    {
        return;
    }

    if (!lineArray || lineArray->empty())
        return;

    const int H = static_cast<int>(lineArray->size());
    const int W = InferImageWidthFromLines(lineResult);

    EnsureLineToolResultSize(toolResult, W, H);
    *neoArray = *lineArray;

    if (y_down < 0 || y_up < 0 || y_down >= H || y_up >= H)
        return;

    const int k = (y_down == y_up) ? 1 : (y_down - y_up);
    const float K = static_cast<float>(x_down - x_up) / static_cast<float>(k);
    float X = static_cast<float>(x_down);

    if (y_down > y_up)
    {
        for (int y = y_down - 1; y > y_up; --y)
        {
            X -= K;
            const int x = ClampInt(static_cast<int>(std::lround(X)), 0, W - 1);
            (*secondLineArray)[static_cast<size_t>(y)] = x;
            (*neoArray)[static_cast<size_t>(y)] = x;
        }
    }
    else
    {
        for (int y = y_down + 1; y < y_up; ++y)
        {
            X += K;
            const int x = ClampInt(static_cast<int>(std::lround(X)), 0, W - 1);
            (*secondLineArray)[static_cast<size_t>(y)] = x;
            (*neoArray)[static_cast<size_t>(y)] = x;
        }
    }

    const int xDownClamped = ClampInt(x_down, 0, W - 1);
    const int xUpClamped = ClampInt(x_up, 0, W - 1);
    (*secondLineArray)[static_cast<size_t>(y_down)] = xDownClamped;
    (*secondLineArray)[static_cast<size_t>(y_up)] = xUpClamped;
    (*neoArray)[static_cast<size_t>(y_down)] = xDownClamped;
    (*neoArray)[static_cast<size_t>(y_up)] = xUpClamped;

    if (y_down > y_up)
    {
        X = static_cast<float>(x_down);
        for (int y = y_down - 1; y > y_up; --y)
        {
            X -= K;
            (*lineArray)[static_cast<size_t>(y)] = ClampInt(static_cast<int>(std::lround(X)), 0, W - 1);
        }
    }
    else
    {
        X = static_cast<float>(x_down);
        for (int y = y_down + 1; y < y_up; ++y)
        {
            X += K;
            (*lineArray)[static_cast<size_t>(y)] = ClampInt(static_cast<int>(std::lround(X)), 0, W - 1);
        }
    }
}

// 最小二乘拟合常用于把阶段性补线再做一次平滑，减少锯齿感。
bool Camera::Regression(uint8_t type, int startline, int endline,
                        const LineSearchResult& lineResult, LineToolResult& toolResult) const
{
    const int H = std::max({ static_cast<int>(lineResult.left_x.size()),
                             static_cast<int>(lineResult.right_x.size()),
                             static_cast<int>(lineResult.center_x.size()) });
    if (H <= 0 || startline < 0 || endline < 0 || startline >= H || endline >= H || startline >= endline)
        return false;

    int sumlines = endline - startline;
    if (sumlines <= 0)
        return false;

    double sumX = 0.0;
    double sumY = 0.0;
    double averageX = 0.0;
    double averageY = 0.0;
    double sumUp = 0.0;
    double sumDown = 0.0;

    if (type == 0)
    {
        if (lineResult.center_x.size() < static_cast<size_t>(endline))
            return false;
        for (int i = startline; i < endline; ++i)
        {
            sumX += i;
            sumY += lineResult.center_x[static_cast<size_t>(i)];
        }
        averageX = sumX / sumlines;
        averageY = sumY / sumlines;
        for (int i = startline; i < endline; ++i)
        {
            const double y = lineResult.center_x[static_cast<size_t>(i)];
            sumUp += (y - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
    }
    else if (type == 1)
    {
        if (lineResult.left_x.size() < static_cast<size_t>(endline))
            return false;
        for (int i = startline; i < endline; ++i)
        {
            sumX += i;
            sumY += lineResult.left_x[static_cast<size_t>(i)];
        }
        averageX = sumX / sumlines;
        averageY = sumY / sumlines;
        for (int i = startline; i < endline; ++i)
        {
            const double y = lineResult.left_x[static_cast<size_t>(i)];
            sumUp += (y - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
    }
    else if (type == 2)
    {
        if (lineResult.right_x.size() < static_cast<size_t>(endline))
            return false;
        for (int i = startline; i < endline; ++i)
        {
            sumX += i;
            sumY += lineResult.right_x[static_cast<size_t>(i)];
        }
        averageX = sumX / sumlines;
        averageY = sumY / sumlines;
        for (int i = startline; i < endline; ++i)
        {
            const double y = lineResult.right_x[static_cast<size_t>(i)];
            sumUp += (y - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
    }
    else
    {
        return false;
    }

    if (sumDown == 0.0)
        return false;

    toolResult.parameterB = static_cast<float>(sumUp / sumDown);
    toolResult.parameterA = static_cast<float>(averageY - toolResult.parameterB * averageX);
    return true;
}

// 根据拟合得到的直线参数回填补线结果。
void Camera::Hua_Xian(uint8_t type, int startline, int endline, float parameterB, float parameterA,
                      LineToolResult& toolResult, int imgW, int imgH) const
{
    if (imgW <= 0 || imgH <= 0)
        return;

    const int start = ClampInt(startline, 0, imgH - 1);
    const int end = ClampInt(endline, 0, imgH - 1);
    if (start > end)
        return;

    EnsureLineToolResultSize(toolResult, imgW, imgH);

    for (int i = start; i <= end; ++i)
    {
        const int x = ClampInt(static_cast<int>(std::lround(parameterB * i + parameterA)), 0, imgW - 1);
        if (type == 1)
        {
            toolResult.l_second_line_x[static_cast<size_t>(i)] = x;
            toolResult.neo_l_line_x[static_cast<size_t>(i)] = x;
        }
        else if (type == 2)
        {
            toolResult.r_second_line_x[static_cast<size_t>(i)] = x;
            toolResult.neo_r_line_x[static_cast<size_t>(i)] = x;
        }
    }
}

// -------------------- 单帧总流程 --------------------
bool Camera::grabProcessAndDisplayFrame(cv::Mat& gray, cv::Mat& binary, LineSearchResult& lineResult,
                                        TrackElementResult& trackResult,
                                        int outWidth, int outHeight, int searchType)
{
    if (!grabGrayFrame(gray, outWidth, outHeight))
        return false;
    flipVertical(gray, gray);
    otsuBinarize(gray, binary);
    pixelFilterErode(binary);
    searchLineEightNeighbor(binary, lineResult, searchType);
    initializeLineTool(lineResult, outWidth, outHeight);
    detectTrackElements(binary, lineResult, trackResult);
    calculateCenterLine(lineResult, outWidth, outHeight);
    TFT_ShowFullGray8(binary.data);
    drawLineResultOnTFT(lineResult, outWidth, outHeight);
    drawTrackElementsOnTFT(trackResult);
    return true;
}
