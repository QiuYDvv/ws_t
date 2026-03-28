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

// 主程序入口：
// 负责平台初始化、设备打开、后台线程启动，以及驱动 camera/imu/debugger 的主循环。
#include "main.hpp"
#include "camera.h"
#include "debuger.h"
#include "displayer.h"
#include "imu.h"
#include "serial.h"
#include "thread_timing.hpp"
#include <termios.h>
#include <csignal>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

// Ctrl+C 时置位，主循环与控制线程检测后主动停机并退出
static volatile sig_atomic_t g_exitRequested = 0;
static void onSigInt(int) { g_exitRequested = 1; }

// 统一封装 join，避免主流程里重复写 joinable 判定。
static void JoinThread(std::thread& thread)
{
    if (thread.joinable())
        thread.join();
}

// IMU 更新线程：
// 按固定周期调用 Imu::update，让姿态解算与主显示循环解耦。
static void ImuUpdateThread(Imu* imu, volatile sig_atomic_t* exitFlag)
{
    if (!imu || !exitFlag)
        return;

    lq::SetThisThreadName("imu");
    lq::ThreadTiming timing("imu");

    const std::chrono::microseconds period(5000); // 200Hz 解算线程（LoongOS 下更稳）
    std::chrono::steady_clock::time_point nextWake = std::chrono::steady_clock::now();

    while (!(*exitFlag))
    {
        const auto loopStart = std::chrono::steady_clock::now();
        if (imu->isInited())
            imu->update();
        const auto afterWork = std::chrono::steady_clock::now();

        nextWake += period;

        const auto workNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(afterWork - loopStart).count());
        const bool overrun = afterWork > nextWake;
        timing.RecordWork(workNs, overrun);
        lq::ThreadTimingReport report;
        if (timing.TryMakeReport(&report))
            lq::PrintThreadTimingLine(timing.name(), report);

        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (nextWake > now)
            std::this_thread::sleep_until(nextWake);
        else
            nextWake = now;
    }
}

// 通过 /proc/modules 判断驱动是否已加载，避免重复 insmod。
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

// 仅在驱动未加载时执行 insmod，减少无意义的系统调用与错误输出。
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
    lq::SetThisThreadName("main");

    // -------------------- 平台初始化 --------------------
    // 加载所需内核模块
    LoadKernelModuleIfNeeded("TFT18_dri", "/lib/modules/4.19.190/TFT18_dri.ko");
    LoadKernelModuleIfNeeded("TFT18_dev", "/lib/modules/4.19.190/TFT18_dev.ko");
    LoadKernelModuleIfNeeded("LQ_MPU6050_dri", "/lib/modules/4.19.190/LQ_MPU6050_dri.ko");
    LoadKernelModuleIfNeeded("LQ_MPU6050_dev", "/lib/modules/4.19.190/LQ_MPU6050_dev.ko");

    // 初始化 TFT（0：横屏）
    TFT_DisplayerInit(0);

    std::signal(SIGINT, onSigInt);

    // -------------------- 业务模块初始化 --------------------
    // 打开摄像头（只负责图像采集与处理）
    Camera cam(0);
    if (!cam.open())
        return 1;

    // 初始化 IMU（MPU6050）
    Imu imu;
    const bool imuReady = imu.init();
    std::thread imuThread;
    if (imuReady)
        imuThread = std::thread(ImuUpdateThread, &imu, &g_exitRequested);

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
    Camera::TrackElementResult trackResult;
    int frameCount = 0;  // 预留给 VOFA 调试发送节流

    lq::ThreadTiming mainTiming("main");
    double lastFps = 0.0;

#if LQ_PROFILE_THREADS
    uint64_t tftLastReportNs = lq::NowWallNs();
    uint64_t tftFrames = 0;
    uint64_t tftFlushSumNs = 0;
    uint64_t tftFlushMaxNs = 0;
#endif

    // -------------------- 主循环 --------------------
    // 主循环：在 main 中同时调用 camera 和 display；Ctrl+C 后也会退出
    while (cam.isOpened() && !g_exitRequested)
    {
        const auto loopStart = std::chrono::steady_clock::now();
        if (!cam.grabProcessAndDisplayFrame(gray, binFull, lineResult, trackResult, tft_w, tft_h, 0))
            break;

        // IMU 在独立线程中更新，主线程只显示当前姿态角。
        DebuggerDisplayImu(imu);
        DebuggerPrintPidIfChanged();
        DebuggerPrintMotorStateIfChanged();

        // 计算帧率并显示在屏幕左上角
        lastFps = TFT_UpdateAndShowFps_NoFlush(cam);

        // 合并一帧内所有绘制，只 flush 一次（灰度图/线条/元素/FPS/IMU）。
#if LQ_PROFILE_THREADS
        const uint64_t flushStart = lq::NowWallNs();
#endif
        TFT_Flush();
#if LQ_PROFILE_THREADS
        const uint64_t flushEnd = lq::NowWallNs();
        if (flushStart && flushEnd && flushEnd >= flushStart)
        {
            const uint64_t flushNs = flushEnd - flushStart;
            tftFlushSumNs += flushNs;
            if (flushNs > tftFlushMaxNs)
                tftFlushMaxNs = flushNs;
        }

        ++tftFrames;
        if (tftLastReportNs && flushEnd && flushEnd - tftLastReportNs >= 1000000000ULL && tftFrames)
        {
            const double invFrames = 1.0 / static_cast<double>(tftFrames);
            const double flushAvgMs = (static_cast<double>(tftFlushSumNs) / 1e6) * invFrames;
            const double flushMaxMs = static_cast<double>(tftFlushMaxNs) / 1e6;
            std::printf("[stage][tft] flush_avg=%.2fms flush_max=%.2fms\n", flushAvgMs, flushMaxMs);
            std::fflush(stdout);
            tftLastReportNs = flushEnd;
            tftFrames = 0;
            tftFlushSumNs = 0;
            tftFlushMaxNs = 0;
        }
#endif

        // VOFA+ JustFloat：每 period 帧发一帧浮点数据（FPS, R, P, Y）
        // DebuggerSendVofaIfNeeded(serial, frameCount, 15, fps, imu);
        (void)frameCount;

        const auto loopEnd = std::chrono::steady_clock::now();
        const auto workNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(loopEnd - loopStart).count());
        mainTiming.RecordWork(workNs, false);
        lq::ThreadTimingReport report;
        if (mainTiming.TryMakeReport(&report))
        {
            char extra[64];
            std::snprintf(extra, sizeof(extra), "fps=%.1f", lastFps);
            lq::PrintThreadTimingLine(mainTiming.name(), report, extra);
        }
    }

    // -------------------- 退出清理 --------------------
    g_exitRequested = 1;
    JoinThread(imuThread);
    JoinThread(motorThread);
    JoinThread(cmdThread);
    return 0;
}
