#include "displayer.h"
#include "LQ_TFT18_dri.hpp"

// 初始化 TFT，type = 0 横屏，1 竖屏
void TFT_DisplayerInit(uint8_t type)
{
    TFTSPI_dri_init(type);
}

uint8_t TFT_DisplayerWidth()
{
    return TFT18W;
}

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

