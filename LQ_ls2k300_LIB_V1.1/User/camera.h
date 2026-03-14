#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

// 摄像头图像采集与处理封装类（只做图像获取和处理，不负责屏幕显示）
class Camera
{
public:
    explicit Camera(int deviceId = 0);
    ~Camera();

    bool open();
    void close();
    bool isOpened() const;

    // 采集一帧，缩放到指定宽高并转换为灰度图
    // gray 将被输出为 CV_8UC1，大小为 outWidth x outHeight
    // 返回 false 表示采集失败或结束
    bool grabGrayFrame(cv::Mat& gray, int outWidth, int outHeight);

    // 对输入灰度图像做上下颠倒（垂直翻转）
    // src 和 dst 可以是同一张图（原地翻转）
    void flipVertical(const cv::Mat& src, cv::Mat& dst) const;

    // 使用大津法对灰度图进行二值化
    // gray 为输入的单通道 CV_8UC1 图像，binary 为输出的二值图
    // 返回选取的阈值（若 gray 为空则返回 0）
    double otsuBinarize(const cv::Mat& gray, cv::Mat& binary) const;

    // 腐蚀/像素滤波：去除孤立噪点，二值图 0 黑 255 白，原地修改 binary
    void pixelFilterErode(cv::Mat& binary) const;

    // 更新并返回当前帧率（单位：fps），每调用一次视为处理了一帧
    double updateFps();

    // 获取最近一次计算得到的帧率
    double currentFps() const { return fps_; }

    // 八邻域搜线结果：每行的左右边线横坐标（与二值图行对应）
    struct LineSearchResult
    {
        std::vector<int> left_x;    // 左线 x，0 表示丢线
        std::vector<int> right_x;   // 右线 x，cols-1 表示丢线
        std::vector<double> center_x;  // 中线横坐标（由 calculateCenterLine 填充）
        double offset = 0.0;        // 偏移量（由 calculateCenterLine 填充）
        int search_line_end = 0;   // 搜线结束行（从下往上搜时的终止行）
    };

    // 根据左右边线计算中线并平滑、求偏移（逻辑同 Calculate_Offset_1）
    void calculateCenterLine(LineSearchResult& result, int imgW, int imgH) const;

    // 八邻域搜线：在二值图上从底向上搜左右边线（黑黑白/白黑黑）
    // binary 为 CV_8UC1，0 黑 255 白；type 0 普通模式，1 斑马线模式（仅起始列不同）
    void searchLineEightNeighbor(const cv::Mat& binary, LineSearchResult& result, int type = 0) const;

    // 使用龙邱库逐点绘制边线到 TFT（左蓝、右绿、中线红），各 3 像素宽，内部会 TFT_Flush
    void drawLineResultOnTFT(const LineSearchResult& result, int imgW, int imgH) const;

    /**
     * 一帧完整流程：采集 → 翻转 → 二值化 → 滤波 → 八邻域搜线 → 中线 → 显示二值图与边线。
     * @return false 表示采集失败（调用方应 break 主循环）
     */
    bool grabProcessAndDisplayFrame(cv::Mat& gray, cv::Mat& binary, LineSearchResult& lineResult,
                                   int outWidth, int outHeight, int searchType = 0);

private:
    int deviceId_;
    cv::VideoCapture cap_;

    mutable double fps_ = 0.0;
    mutable int frameCount_ = 0;
    mutable int64_t lastTick_ = 0;

    // 大津法二值化相关状态：缓存阈值并控制计算频率
    mutable double otsuThresh_ = 0.0;
    mutable int otsuFrameCount_ = 0;
    mutable bool otsuInited_ = false;
};


