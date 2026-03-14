/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-02-26
@修改内容：
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "main.hpp"
#include "camera.h"
#include "debuger.h"
#include "displayer.h"
#include "imu.h"
#include "serial.h"
#include <termios.h>
#include <csignal>
#include <cstdio>

// Ctrl+C 时置位，电机线程检测后 stop 并退出，从而析构 PWM 并 unexport，第二次运行才能重新 export
static volatile sig_atomic_t g_exitRequested = 0;
static void onSigInt(int) { g_exitRequested = 1; }

int main()
{

    // 加载所需内核模块
    std::system("insmod /lib/modules/4.19.190/TFT18_dri.ko");
    std::system("insmod /lib/modules/4.19.190/TFT18_dev.ko");
    std::system("insmod /lib/modules/4.19.190/LQ_MPU6050_dri.ko");
    std::system("insmod /lib/modules/4.19.190/LQ_MPU6050_dev.ko");

    // 初始化 TFT（0：横屏）
    TFT_DisplayerInit(0);

    std::signal(SIGINT, onSigInt);

    // 串口提前打开，用于正弦速度测试与主循环共用
    Serial serial;
    if (serial.open("/dev/ttyS1", B115200))
        StartDebuggerThreads(serial, &g_exitRequested);

    // 打开摄像头（只负责图像采集与处理）
    Camera cam(0);
    if (!cam.open())
    {
        return 1;
    }

    // 初始化 IMU（MPU6050）
    Imu imu;
    imu.init();

    // 串口已在前面打开；若未打开则重新打开（供主循环 VOFA+ 等使用）
    if (!serial.isOpen() && !serial.open("/dev/ttyS1", B115200))
        { /* 可选：记录打开失败 */ }
    if (serial.isOpen())
        DebuggerSendStr(serial, "main start\n");

    // 显示分辨率
    const int tft_w = TFT_DisplayerWidth();
    const int tft_h = TFT_DisplayerHeight();

    cv::Mat gray;        // 原始分辨率灰度图
    cv::Mat binFull;     // 原始分辨率二值图（用于显示）
    Camera::LineSearchResult lineResult;

    int frameCount = 0;  // 用于串口发送节流

    // 主循环：在 main 中同时调用 camera 和 display；Ctrl+C 后也会退出
    while (cam.isOpened() && !g_exitRequested)
    {
        if (!cam.grabProcessAndDisplayFrame(gray, binFull, lineResult, tft_w, tft_h, 0))
            break;

        // 读取 IMU 并显示解算姿态角（R/P/Y：横滚/俯仰/航向，单位度）
        DebuggerUpdateAndDisplayImu(imu);

        // 计算帧率并显示在屏幕左上角
        double fps = TFT_UpdateAndShowFps(cam);

        // VOFA+ JustFloat：每 period 帧发一帧浮点数据（FPS, R, P, Y）
        // DebuggerSendVofaIfNeeded(serial, frameCount, 15, fps, imu);
    }

    return 0;
}
