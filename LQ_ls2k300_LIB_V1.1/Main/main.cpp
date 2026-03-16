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
#include <fstream>
#include <string>
#include <thread>

// Ctrl+C 时置位，主循环与控制线程检测后主动停机并退出
static volatile sig_atomic_t g_exitRequested = 0;
static void onSigInt(int) { g_exitRequested = 1; }

static void JoinThread(std::thread& thread)
{
    if (thread.joinable())
        thread.join();
}

static bool IsKernelModuleLoaded(const char* moduleName)
{
    if (!moduleName || moduleName[0] == '\0')
        return false;

    std::ifstream modules("/proc/modules");
    if (!modules.is_open())
        return false;

    std::string line;
    const std::string prefix(moduleName);
    while (std::getline(modules, line))
    {
        if (line.compare(0, prefix.size(), prefix) == 0 &&
            (line.size() == prefix.size() || line[prefix.size()] == ' '))
            return true;
    }
    return false;
}

static void LoadKernelModuleIfNeeded(const char* moduleName, const char* modulePath)
{
    if (!moduleName || !modulePath || IsKernelModuleLoaded(moduleName))
        return;
    std::string cmd = "insmod ";
    cmd += modulePath;
    std::system(cmd.c_str());
}

int main()
{

    // 加载所需内核模块
    LoadKernelModuleIfNeeded("TFT18_dri", "/lib/modules/4.19.190/TFT18_dri.ko");
    LoadKernelModuleIfNeeded("TFT18_dev", "/lib/modules/4.19.190/TFT18_dev.ko");
    LoadKernelModuleIfNeeded("LQ_MPU6050_dri", "/lib/modules/4.19.190/LQ_MPU6050_dri.ko");
    LoadKernelModuleIfNeeded("LQ_MPU6050_dev", "/lib/modules/4.19.190/LQ_MPU6050_dev.ko");

    // 初始化 TFT（0：横屏）
    TFT_DisplayerInit(0);

    std::signal(SIGINT, onSigInt);

    // 打开摄像头（只负责图像采集与处理）
    Camera cam(0);
    if (!cam.open())
        return 1;

    // 初始化 IMU（MPU6050）
    Imu imu;
    imu.init();

    // 串口在业务初始化完成后再打开，避免初始化失败时线程已经启动
    Serial serial;
    std::thread cmdThread, motorThread;
    if (serial.open("/dev/ttyS1", B115200))
        StartDebuggerThreads(serial, &g_exitRequested, &cmdThread, &motorThread);

    // 显示分辨率
    const int tft_w = TFT_DisplayerWidth();
    const int tft_h = TFT_DisplayerHeight();

    cv::Mat gray;        // 原始分辨率灰度图
    cv::Mat binFull;     // 原始分辨率二值图（用于显示）
    Camera::LineSearchResult lineResult;
    int frameCount = 0;  // 预留给 VOFA 调试发送节流

    // 主循环：在 main 中同时调用 camera 和 display；Ctrl+C 后也会退出
    while (cam.isOpened() && !g_exitRequested)
    {
        if (!cam.grabProcessAndDisplayFrame(gray, binFull, lineResult, tft_w, tft_h, 0))
            break;

        // 读取 IMU 并显示解算姿态角（R/P/Y：横滚/俯仰/航向，单位度）
        DebuggerUpdateAndDisplayImu(imu);
        DebuggerPrintPidIfChanged();
        DebuggerPrintMotorStateIfChanged();

        // 计算帧率并显示在屏幕左上角
        double fps = TFT_UpdateAndShowFps(cam);
        (void)fps;

        // VOFA+ JustFloat：每 period 帧发一帧浮点数据（FPS, R, P, Y）
        // DebuggerSendVofaIfNeeded(serial, frameCount, 15, fps, imu);
        (void)frameCount;
    }

    g_exitRequested = 1;
    JoinThread(motorThread);
    JoinThread(cmdThread);
    return 0;
}
