#include "camera.h"
#include "displayer.h"
#include <algorithm>
#include <cstdio>
#include <cstdint>

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



