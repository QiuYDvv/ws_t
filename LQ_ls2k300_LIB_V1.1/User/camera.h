#pragma once

#include <array>
#include <opencv2/opencv.hpp>
#include <vector>

// 摄像头模块公共接口：
// 负责采集图像、二值化、搜线、元素识别/处理，并向主流程输出中线与偏差结果。

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

    // 赛道元素检测结果：先实现斑马线，后续可继续扩展环岛、十字等元素
    struct TrackElementResult
    {
        enum ElementType
        {
            None = 0,
            ZebraCrossing = 1,
            LeftRoundabout = 2,
            RightRoundabout = 3,
            LeftCross = 4,
            RightCross = 5,
            Crossroad = 6,
            Fork = 7,
            Straight = 8,
            BendLeft = 9,
            BendRight = 10,
            Bend = 11
        };

        ElementType current = None;
        bool zebra_detected = false;   // 对应原 flag_starting_line
        bool line_found = false;       // 本帧原始检测结果
        int row_hits = 0;              // 对应原 times
        int stop_count = 0;            // 对应原 stop_1
        bool in_cooldown = false;      // 对应原 in_cooldown
        int cooldown_counter = 0;      // 对应原 cooldown_counter
        bool last_line_found = false;  // 对应原 last_line_found

        bool left_roundabout_detected = false;
        bool right_roundabout_detected = false;

        int left_diuxian_hang = 0;
        int left_budiuxian_hang = 0;
        int right_diuxian_hang = 0;
        int right_budiuxian_hang = 0;

        int l_guaidain_x1 = 0;
        int l_guaidain_y1 = 0;
        int r_guaidain_x1 = 0;
        int r_guaidain_y1 = 0;
        int l_guaidain_x2 = 0;
        int l_guaidain_y2 = 0;
        int r_guaidain_x2 = 0;
        int r_guaidain_y2 = 0;

        int l_guaidain_x = 0;
        int l_guaidain_y = 0;
        int r_guaidain_x = 0;
        int r_guaidain_y = 0;

        int bianxian_guaidian_l = 0;
        int bianxian_guaidian_r = 0;

        int sancha_x = 0;
        int sancha_y = 0;
        int sancha_x_zhengque = 0;
        int sancha_y_zhengque = 0;
        int qianzhang = 0;

        int cirque_or_cross_count = 0;
        double left_curvature = 0.0;
        double right_curvature = 0.0;
        int track_length = 0;
        int left_lose_value = 0;
        int right_lose_value = 0;
        int left_losemax = 0;
        int right_losemax = 0;

        bool left_cross_detected = false;
        bool right_cross_detected = false;
        bool cross_detected = false;
        bool fork_detected = false;
        bool straight_detected = false;
        bool bend_detected = false;
        int bend_direction = 0; // -1 左弯，1 右弯，0 未知/近直
        int straight_factor = 0;
        bool outside_protected = false;
        int roundabout_stage = 0;
        int fork_stage = 0;
        int cross_stage = 0;
    };

    // 车库/补线工具结果：尽量保留原工程里的变量命名，便于后续移植
    struct LineToolResult
    {
        std::array<int, 2> rightx_1 { 0, 0 };
        std::array<int, 2> righty_1 { 0, 0 };
        std::vector<int> neo_l_line_x;
        std::vector<int> neo_r_line_x;
        std::vector<int> l_second_line_x;
        std::vector<int> r_second_line_x;
        float parameterA = 0.0f;
        float parameterB = 0.0f;
    };

    // 根据左右边线计算中线并平滑、求偏移（逻辑同 Calculate_Offset_1）
    void calculateCenterLine(LineSearchResult& result, int imgW, int imgH) const;

    // 八邻域搜线：在二值图上从底向上搜左右边线（黑黑白/白黑黑）
    // binary 为 CV_8UC1，0 黑 255 白；type 0 普通模式，1 斑马线模式（仅起始列不同）
    void searchLineEightNeighbor(const cv::Mat& binary, LineSearchResult& result, int type = 0) const;

    // 使用龙邱库逐点绘制边线到 TFT（左蓝、右绿、中线红），各 3 像素宽，内部会 TFT_Flush
    void drawLineResultOnTFT(const LineSearchResult& result, int imgW, int imgH) const;

    // 赛道元素检测入口：内部先检测斑马线/环岛，并更新结果结构体
    void detectTrackElements(const cv::Mat& binary, LineSearchResult& lineResult,
                             TrackElementResult& result);

    // 按原 check_cheku 逻辑检测斑马线
    void detectZebraCrossing(const cv::Mat& binary, TrackElementResult& result,
                             int startRow, int endRow, int strength) const;

    // 按原 CircleTest 思路检测环岛：type 1 左，2 右
    bool detectRoundabout(const cv::Mat& binary, const LineSearchResult& lineResult,
                          TrackElementResult& result, uint8_t type) const;

    // 找三岔 V 字底点
    void Sancha_didian(const cv::Mat& binary, TrackElementResult& result) const;

    // 计算前瞻量（中线）
    void Qianzhang(const cv::Mat& binary, TrackElementResult& result) const;

    // 判断圆环还是十字：type 1 左，2 右
    int Cirque_or_Cross(uint8_t type, int startline, const cv::Mat& binary,
                        const LineSearchResult& lineResult, TrackElementResult& result) const;

    // 元素识别主逻辑
    void Element_Test(const cv::Mat& binary, const LineSearchResult& lineResult,
                      TrackElementResult& result);

    // 判断丢线行位置和数目：type 1 左，2 右
    void Diuxian_weizhi_test(uint8_t type, int startline, int endline,
                             const LineSearchResult& lineResult, TrackElementResult& result) const;

    // 找边线拐点（第一种）
    bool Bianxian_guaidian_num(uint8_t type, int startline, int endline,
                               const LineSearchResult& lineResult, TrackElementResult& result) const;

    // 找边线拐点（单拐点）
    void Bianxian_guaidian(uint8_t type, int startline, int endline,
                           const LineSearchResult& lineResult, TrackElementResult& result) const;

    // 获取圆环弧线：type 1 左，2 右
    bool RoundaboutGetArc(uint8_t type, int num, const LineSearchResult& lineResult,
                          TrackElementResult& result) const;

    // 在屏幕左下角显示当前检测到的赛道元素名称
    void drawTrackElementsOnTFT(const TrackElementResult& result) const;

    // 找车库拐点（简单版）：对应原 Check_guaidian_cheku_1
    void Check_guaidian_cheku_1(const cv::Mat& binary, LineToolResult& toolResult, uint8_t type = 0) const;

    // 两点拉直线并同步更新 neo/second_line 数组：type 1 左线，2 右线
    void La_zhixian(uint8_t type, int x_down, int y_down, int x_up, int y_up,
                    LineSearchResult& lineResult, LineToolResult& toolResult) const;

    // 最小二乘拟合：type 0 中线，1 左线，2 右线；结果写回 toolResult.parameterA/B
    bool Regression(uint8_t type, int startline, int endline,
                    const LineSearchResult& lineResult, LineToolResult& toolResult) const;

    // 根据最小二乘结果划线：type 1 左线，2 右线
    void Hua_Xian(uint8_t type, int startline, int endline, float parameterB, float parameterA,
                  LineToolResult& toolResult, int imgW, int imgH) const;

    /**
     * 一帧完整流程：采集 → 翻转 → 二值化 → 滤波 → 八邻域搜线 → 中线 → 显示二值图与边线。
     * @return false 表示采集失败（调用方应 break 主循环）
     */
    bool grabProcessAndDisplayFrame(cv::Mat& gray, cv::Mat& binary, LineSearchResult& lineResult,
                                    TrackElementResult& trackResult,
                                    int outWidth, int outHeight, int searchType = 0);

private:
    // 赛道元素持久化状态：
    // 与 TrackElementResult 的“当前帧结果”不同，这里保存跨帧的状态机状态。
    struct RoadTypeState
    {
        bool straight = false;
        bool bend = false;
        bool Ramp = false;
        bool Cross = false;
        bool L_Cross = false;
        bool R_Cross = false;
        bool LeftCirque = false;
        bool RightCirque = false;
        bool Fork = false;
        bool Barn_l_out = false;
        bool Barn_r_out = false;
        bool Barn_l_in = false;
        bool Barn_r_in = false;
    };

    void initializeLineTool(const LineSearchResult& lineResult, int imgW, int imgH);
    void calculateCenterLineWithSupplement(LineSearchResult& result, int imgW, int imgH) const;
    void syncRoadStateToResult(TrackElementResult& result) const;
    void Check_Zhidao(const LineSearchResult& lineResult, TrackElementResult& result);
    void Mid_Col(const cv::Mat& binary, TrackElementResult& result) const;
    void Outside_protect(const cv::Mat& binary, const LineSearchResult& lineResult,
                         TrackElementResult& result) const;
    bool Tututu(uint8_t type, const LineSearchResult& lineResult, int imgW) const;
    void Element_Handle(LineSearchResult& lineResult, TrackElementResult& result);
    void Handle_Left_Cirque(LineSearchResult& lineResult, TrackElementResult& result);
    void Handle_Right_Cirque(LineSearchResult& lineResult, TrackElementResult& result);
    void Handle_Fork(LineSearchResult& lineResult, TrackElementResult& result);
    void Handle_L_Cross(LineSearchResult& lineResult, TrackElementResult& result);
    void Handle_R_Cross(LineSearchResult& lineResult, TrackElementResult& result);
    void Handle_Cross(LineSearchResult& lineResult, TrackElementResult& result);
    void Handle_Barn_in(uint8_t type, LineSearchResult& lineResult, TrackElementResult& result);
    void Check_Cross_Guaidian(uint8_t type, const LineSearchResult& lineResult);
    void Clear_Supplement_Lines(LineSearchResult& lineResult);
    int TrackWidthAt(int y, int imgW, int imgH) const;
    int EstimateDistanceMeasure(const LineSearchResult& lineResult) const;

    int deviceId_;
    cv::VideoCapture cap_;

    mutable double fps_ = 0.0;
    mutable int frameCount_ = 0;
    mutable int64_t lastTick_ = 0;

    // 大津法二值化相关状态：缓存阈值并控制计算频率
    mutable double otsuThresh_ = 0.0;
    mutable int otsuFrameCount_ = 0;
    mutable bool otsuInited_ = false;

    RoadTypeState roadType_;
    LineToolResult lineToolResult_;
    bool pauseCameraProcess_ = false;
    bool disableElementDetection_ = false;
    bool inCarHahaMode_ = false;
    bool pushedBoxInRoundabout_ = false;
    int huandaoStage_ = 1;
    int sanchaStage_ = 1;
    int leftCrossStage_ = 1;
    int rightCrossStage_ = 1;
    int crossStage_ = 1;
    int passBarn_ = 1;
    int closeCheckKuS_ = 0;
    int jinkuS_ = 0;
    int annulusS1_ = 0;
    int forkS1_ = 0;
    int forkS2_ = 0;
    int crossS_ = 0;
    int sanchaXBest_ = 0;
    int sanchaYBest_ = 0;
    std::array<int, 2> crossLeft_ { 0, 0 };
    std::array<int, 2> crossRight_ { 0, 0 };
};
