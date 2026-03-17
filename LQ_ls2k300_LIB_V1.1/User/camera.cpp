#include "camera.h"
#include "displayer.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>

namespace
{
int ClampInt(int value, int minValue, int maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

void EnsureLineToolResultSize(Camera::LineToolResult& toolResult, int imgW, int imgH)
{
    (void)imgW;
    const size_t H = (imgH > 0) ? static_cast<size_t>(imgH) : 0U;
    toolResult.neo_l_line_x.resize(H, 0);
    toolResult.neo_r_line_x.resize(H, 0);
    toolResult.l_second_line_x.resize(H, 0);
    toolResult.r_second_line_x.resize(H, 0);
}

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

bool TututuGuard(uint8_t type, const Camera::LineSearchResult& lineResult, int imgW)
{
    const std::vector<int>* line = nullptr;
    int lostValue = 0;
    if (type == 1)
    {
        line = &lineResult.left_x;
        lostValue = 0;
    }
    else if (type == 2)
    {
        line = &lineResult.right_x;
        lostValue = imgW - 1;
    }
    else
    {
        return false;
    }

    if (!line || line->size() < 20)
        return false;

    int oscillationCount = 0;
    int lastSign = 0;
    for (int y = static_cast<int>(line->size()) - 2; y >= std::max(10, static_cast<int>(line->size()) - 25); --y)
    {
        const int cur = (*line)[static_cast<size_t>(y)];
        const int next = (*line)[static_cast<size_t>(y + 1)];
        if (cur == lostValue || next == lostValue)
            continue;

        const int diff = cur - next;
        if (std::abs(diff) < 3)
            continue;

        const int sign = (diff > 0) ? 1 : -1;
        if (lastSign != 0 && sign != lastSign)
            oscillationCount++;
        lastSign = sign;
    }

    return oscillationCount >= 4;
}

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
}

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
            const int default_width = std::min(W / 2, 80);
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

void Camera::calculateCenterLine(LineSearchResult& result, int imgW, int imgH) const
{
    result.offset = 0.0;
    const int H = imgH;
    const int W = imgW;
    const int mid = W / 2;

    if (result.left_x.size() != static_cast<size_t>(H) || result.right_x.size() != static_cast<size_t>(H))
        return;

    result.center_x.resize(static_cast<size_t>(H), 0.0);

    // 中线 = (左线 + 右线) / 2，从底向上到第 10 行
    for (int y = H - 1; y >= 10; y--)
    {
        int l = result.left_x[static_cast<size_t>(y)];
        int r = result.right_x[static_cast<size_t>(y)];
        result.center_x[static_cast<size_t>(y)] = 1.0 * (l + r) / 2.0;
    }

    // 对中线平滑滤波（5 点滑动平均，类似 HDPJ_lvbo）
    const int win = 2;  // 左右各 2 点，共 5 点
    std::vector<double> tmp(static_cast<size_t>(H), 0.0);
    for (int y = 0; y < H; y++)
        tmp[static_cast<size_t>(y)] = result.center_x[static_cast<size_t>(y)];
    for (int y = 20; y <= H - 1; y++)
    {
        double sum = 0.0;
        int n = 0;
        for (int d = -win; d <= win; d++)
        {
            int yy = y + d;
            if (yy >= 0 && yy < H)
            {
                sum += tmp[static_cast<size_t>(yy)];
                n++;
            }
        }
        if (n > 0)
            result.center_x[static_cast<size_t>(y)] = sum / n;
    }

    // 计算偏移量：取底部长条区域加权（y 从 H-15 到 H-25），权重取 1，最后 /2
    static const int OFFSET_WEIGHT_COUNT = 11;
    double weights[OFFSET_WEIGHT_COUNT] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
    for (int y = H - 15; y >= H - 25 && y >= 0; y--)
    {
        int idx = (H - 15) - y;
        if (idx >= 0 && idx < OFFSET_WEIGHT_COUNT)
            result.offset += weights[idx] * (result.center_x[static_cast<size_t>(y)] - mid);
    }
    result.offset = result.offset / 2.0;
}

void Camera::drawLineResultOnTFT(const LineSearchResult& result, int imgW, int imgH) const
{
    static constexpr uint16_t RGB565_BLUE  = 0x001F;
    static constexpr uint16_t RGB565_GREEN = 0x07E0;
    static constexpr uint16_t RGB565_RED   = 0xF800;
    const int halfW = 1;

    for (int y = 0; y < imgH; y++)
    {
        int lx = result.left_x[static_cast<size_t>(y)];
        int rx = result.right_x[static_cast<size_t>(y)];
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

void Camera::detectTrackElements(const cv::Mat& binary, const LineSearchResult& lineResult,
                                 TrackElementResult& result) const
{
    result.current = TrackElementResult::None;
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
    result.qianzhang = 0;
    result.cirque_or_cross_count = 0;
    result.left_curvature = 0.0;
    result.right_curvature = 0.0;
    result.track_length = 0;
    result.left_lose_value = 0;
    result.right_lose_value = 0;
    result.left_losemax = 0;
    result.right_losemax = 0;

    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    const int startRow = std::max(0, H - 10);
    const int endRow = std::max(0, H / 2);
    const int strength = std::max(4, W / 24);

    detectZebraCrossing(binary, result, startRow, endRow, strength);
    Element_Test(binary, lineResult, result);

    if (result.current == TrackElementResult::None && result.zebra_detected)
        result.current = TrackElementResult::ZebraCrossing;
    else if (result.current == TrackElementResult::None && result.left_roundabout_detected)
        result.current = TrackElementResult::LeftRoundabout;
    else if (result.current == TrackElementResult::None && result.right_roundabout_detected)
        result.current = TrackElementResult::RightRoundabout;
}

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

void Camera::Element_Test(const cv::Mat& binary, const LineSearchResult& lineResult,
                          TrackElementResult& result) const
{
    if (binary.empty() || binary.type() != CV_8UC1)
        return;

    const int H = binary.rows;
    const int W = binary.cols;
    if (H < 50 || W < 20 || lineResult.center_x.size() < static_cast<size_t>(H))
        return;

    ComputeLoseMetrics(lineResult, W, result.left_lose_value, result.right_lose_value,
                       result.left_losemax, result.right_losemax);
    result.track_length = std::max(0, H - lineResult.search_line_end);

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
    const bool leftArcStrong = RoundaboutGetArc(1, 15, lineResult, result);
    const bool rightArcStrong = RoundaboutGetArc(2, 15, lineResult, result);
    const bool leftTututu = TututuGuard(1, lineResult, W);
    const bool rightTututu = TututuGuard(2, lineResult, W);
    result.left_roundabout_detected = false;
    result.right_roundabout_detected = false;

    // 保持与原 Element_Test 一致的判定顺序。
    if (result.track_length > 50 &&
        result.left_lose_value > 30 &&
        result.right_lose_value < 14 &&
        std::fabs(result.right_curvature) < 15.0 &&
        !leftTututu)
    {
        if (Cirque_or_Cross(1, result.left_losemax, binary, lineResult, result) >= 15 || leftRoundaboutBase)
        {
            result.left_roundabout_detected = true;
            result.current = TrackElementResult::LeftRoundabout;
        }
        else if (RightLineXAt(lineResult, ClampInt(50, 0, H - 1)) < std::min(100, W - 1) &&
                 RightLineXAt(lineResult, ClampInt(53, 0, H - 1)) < std::min(100, W - 1))
        {
            result.left_cross_detected = true;
            result.current = TrackElementResult::LeftCross;
        }
    }
    else if (result.track_length > 50 &&
             result.right_lose_value > 30 &&
             result.left_lose_value < 14 &&
             std::fabs(result.left_curvature) < 15.0 &&
             !rightTututu)
    {
        if (Cirque_or_Cross(2, result.right_losemax, binary, lineResult, result) >= 10 || rightRoundaboutBase)
        {
            result.right_roundabout_detected = true;
            result.current = TrackElementResult::RightRoundabout;
        }
        else if (LeftLineXAt(lineResult, ClampInt(50, 0, H - 1)) > std::min(100, W - 1) &&
                 LeftLineXAt(lineResult, ClampInt(53, 0, H - 1)) > std::min(100, W - 1))
        {
            result.right_cross_detected = true;
            result.current = TrackElementResult::RightCross;
        }
    }
    else if (leftArcStrong && rightArcStrong)
    {
        result.fork_detected = true;
        result.current = TrackElementResult::Fork;
    }
    else if (result.track_length > 60 &&
             result.right_lose_value > 20 &&
             result.left_lose_value > 20)
    {
        result.cross_detected = true;
        result.current = TrackElementResult::Crossroad;
    }
    else if (result.qianzhang <= 65)
    {
        result.bend_detected = true;
        result.straight_detected = false;

        const int quarterY = ClampInt(H * 2 / 3, 0, H - 1);
        const int bottomY = ClampInt(H * 5 / 6, 0, H - 1);
        const int quarterPoint = static_cast<int>(std::lround(lineResult.center_x[static_cast<size_t>(quarterY)]));
        const int bottomPoint = static_cast<int>(std::lround(lineResult.center_x[static_cast<size_t>(bottomY)]));

        if (quarterPoint < bottomPoint)
        {
            result.bend_direction = -1;
            result.current = TrackElementResult::BendLeft;
        }
        else if (quarterPoint > bottomPoint)
        {
            result.bend_direction = 1;
            result.current = TrackElementResult::BendRight;
        }
        else
        {
            result.bend_direction = 0;
            result.current = TrackElementResult::Bend;
        }
    }
    else
    {
        result.straight_detected = true;
        result.bend_detected = false;
        result.current = TrackElementResult::Straight;
    }

    // 三岔底点作为辅助诊断信息；若已经识别出 Fork，则保留当前状态。
    if (result.current == TrackElementResult::Fork && result.sancha_y > 0)
    {
        result.sancha_y_zhengque = std::max(result.sancha_y_zhengque, result.sancha_y);
        result.sancha_x_zhengque = result.sancha_x;
    }
}

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
    calculateCenterLine(lineResult, outWidth, outHeight);
    detectTrackElements(binary, lineResult, trackResult);
    TFT_ShowFullGray8(binary.data);
    drawLineResultOnTFT(lineResult, outWidth, outHeight);
    drawTrackElementsOnTFT(trackResult);
    return true;
}
