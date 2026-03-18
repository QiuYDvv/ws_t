// 显示模块实现：
// 对龙邱 TFT 驱动做薄封装，统一提供灰度图显示、文本显示和逐点绘制接口。
#include "displayer.h"
#include "camera.h"
#include "LQ_TFT18_dri.hpp"
#include <cstdio>

// 初始化 TFT，type = 0 横屏，1 竖屏
void TFT_DisplayerInit(uint8_t type)
{
    TFTSPI_dri_init(type);
}

// 直接返回底层驱动宏中的宽度配置。
uint8_t TFT_DisplayerWidth()
{
    return TFT18W;
}

// 直接返回底层驱动宏中的高度配置。
uint8_t TFT_DisplayerHeight()
{
    return TFT18H;
}

void TFT_ShowFullGray8(const uint8_t* gray)
{
    if (gray == nullptr)
        return;

    // 直接使用库提供的灰度显示接口
    TFTSPI_dir_road(0, 0, TFT18H, TFT18W, (uint8_t*)gray);
}

void TFT_ShowFullRGB565(const uint16_t* rgb565)
{
    if (rgb565 == nullptr)
        return;

    // 逐像素刷新 RGB565 缓冲，适合已经完成颜色编码的整帧图像。
    uint32_t idx = 0;
    for (uint8_t y = 0; y < TFT18H; y++)
    {
        for (uint8_t x = 0; x < TFT18W; x++)
        {
            TFTSPI_dri_data_mod(x, y, rgb565[idx++]);
        }
    }
    TFTSPI_dir_flush();
}

void TFT_DrawPixel(uint8_t x, uint8_t y, uint16_t color)
{
    // 这里只写显存，不立即刷新；批量绘制结束后由调用方统一 flush。
    TFTSPI_dri_data_mod(x, y, color);
}

void TFT_Flush()
{
    TFTSPI_dir_flush();
}

void TFT_ShowTextTopLeft(const char* text)
{
    if (text == nullptr)
        return;

    // 使用库提供的 6x8 字符串输出函数，在左上角(行0,列0)显示
    TFTSPI_dir_P6X8Str(0, 1, text, u16YELLOW, u16BLACK);
}

void TFT_ShowTextBottomLeft(const char* text, uint8_t lineFromBottom)
{
    if (text == nullptr)
        return;

    const uint8_t textRows = static_cast<uint8_t>(TFT18H / 8);
    if (textRows == 0)
        return;

    uint8_t y = static_cast<uint8_t>(textRows - 1);
    if (lineFromBottom < textRows)
        y = static_cast<uint8_t>(textRows - 1 - lineFromBottom);

    // 将“距底部第几行”转换为驱动使用的字符行号。
    TFTSPI_dir_P6X8Str(0, y, text, u16YELLOW, u16BLACK);
}

double TFT_UpdateAndShowFps(Camera& cam)
{
    double fps = cam.updateFps();
    char buf[32];
    std::snprintf(buf, sizeof(buf), "FPS: %.1f", fps);
    TFT_ShowTextTopLeft(buf);
    return fps;
}
