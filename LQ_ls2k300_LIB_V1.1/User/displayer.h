#pragma once

#include <stdint.h>
#include <stddef.h>

// 前向声明，避免 displayer 与 camera 互相包含
class Camera;

// 简单封装：直接调用 LQ_TFT18_dri 提供的函数，不自己写底层驱动

// 初始化 TFT，type = 0 横屏，1 竖屏
void TFT_DisplayerInit(uint8_t type);

// 获取屏幕宽高（宏直接来自 LQ_TFT18_dri.hpp）
uint8_t TFT_DisplayerWidth();
uint8_t TFT_DisplayerHeight();

// 全屏显示 8bit 灰度原始图像（长度 >= 宽*高，按行从左到右、从上到下）
void TFT_ShowFullGray8(const uint8_t* gray);

// 全屏显示 RGB565 原始图像（长度 >= 宽*高，按行从左到右、从上到下）
void TFT_ShowFullRGB565(const uint16_t* rgb565);

// 在指定坐标画一个点（RGB565 颜色），需配合 TFT_Flush 生效
void TFT_DrawPixel(uint8_t x, uint8_t y, uint16_t color);

// 将显存刷新到屏幕（在逐点绘制后调用）
void TFT_Flush();

// 在屏幕左上角显示一行文本（使用 6x8 字体）
void TFT_ShowTextTopLeft(const char* text);

// 在屏幕左下角显示一行文本（lineFromBottom = 0 表示最底下一行）
void TFT_ShowTextBottomLeft(const char* text, uint8_t lineFromBottom = 0);

// 计算帧率并在屏幕左上角显示，返回当前 FPS（内部调用 cam.updateFps()）
double TFT_UpdateAndShowFps(Camera& cam);
