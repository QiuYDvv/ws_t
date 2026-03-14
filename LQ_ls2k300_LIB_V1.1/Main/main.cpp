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
#include "controller.hpp"
#include "displayer.h"
#include "imu.h"
#include "serial.h"
#include <termios.h>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <sys/time.h>
#include <thread>
#include <unistd.h>  // sleep(), usleep()

// 串口被电机线程与主线程共用，加互斥锁保证收发不交叉
static std::mutex g_serialMutex;

// Ctrl+C 时置位，电机线程检测后 stop 并退出，从而析构 PWM 并 unexport，第二次运行才能重新 export
static volatile sig_atomic_t g_exitRequested = 0;
static void onSigInt(int) { g_exitRequested = 1; }

// 计算帧率并在屏幕左上角显示，返回当前 FPS
static double updateAndShowFps(Camera& cam)
{
    double fps = cam.updateFps();
    char buf[32];
    std::snprintf(buf, sizeof(buf), "FPS: %.1f", fps);
    TFT_ShowTextTopLeft(buf);
    return fps;
}

// 按周期通过串口发送 VOFA+ JustFloat 一帧（FPS, R, P, Y）
static void sendVofaIfNeeded(Serial& serial, int& frameCount, int period,
                             double fps, const Imu& imu)
{
    if (!serial.isOpen() || (++frameCount % period != 0))
        return;
    float vofa[4] = {
        static_cast<float>(fps),
        static_cast<float>(imu.rollDeg()),
        static_cast<float>(imu.pitchDeg()),
        static_cast<float>(imu.yawDeg())
    };
    std::lock_guard<std::mutex> lock(g_serialMutex);
    serial.sendVofaJustFloat(vofa, 4);
}

// 将目标速度与编码器实际速度发送到上位机（VOFA+ JustFloat：targetL, targetR, actualL, actualR）
static void sendSpeedToHost(Serial& serial,
                            float targetLeft, float targetRight,
                            float actualLeft, float actualRight)
{
    if (!serial.isOpen())
        return;
    float vofa[4] = { targetLeft, targetRight, actualLeft, actualRight };
    std::lock_guard<std::mutex> lock(g_serialMutex);
    serial.sendVofaJustFloat(vofa, 4);
}

// 为「目标速度」是否用正弦：false 时用下面两个 override 值（如 0 0 即停止）
static bool g_useSineTarget = true;
static float g_overrideTargetL = 0.0f;
static float g_overrideTargetR = 0.0f;

// 上位机下发的 PID 参数，电机线程每拍从这些全局量同步到 motor（不改龙邱库，故用独立命令线程写全局）
static float g_kpL = 0.1f, g_kiL = 0.0f, g_kdL = 0.0f;
static float g_kpR = 0.1f, g_kiR = 0.0f, g_kdR = 0.0f;

/**
 * 独立线程：阻塞读串口，解析一行后更新全局 PID/目标。
 * 协议（文本，以 \\n 结尾）：P/L/R/S/G 同前，只写全局，电机线程每拍把全局同步到 motor。
 */
static void commandReceiverThread(Serial* serial)
{
    if (!serial || !serial->isOpen())
        return;
    char lineBuf[128];
    size_t lineLen = 0;

    while (!g_exitRequested)
    {
        char chunk[32];
        int n;
        {
            std::lock_guard<std::mutex> lock(g_serialMutex);
            n = serial->receive(chunk, sizeof(chunk));
        }
        if (n <= 0)
        {
            usleep(10000);
            continue;
        }

        for (int i = 0; i < n && lineLen < sizeof(lineBuf) - 1; i++)
        {
            char c = chunk[i];
            if (c == '\n' || c == '\r')
            {
                lineBuf[lineLen] = '\0';
                if (lineLen == 0)
                {
                    lineLen = 0;
                    continue;
                }

                float a = 0, b = 0, c1 = 0, d = 0, e = 0, f = 0;
                int cnt = 0;

                if (lineBuf[0] == 'P' && lineBuf[1] == ' ')
                {
                    cnt = std::sscanf(lineBuf + 2, "%f %f %f %f %f %f", &a, &b, &c1, &d, &e, &f);
                    if (cnt == 6)
                        { g_kpL = a; g_kiL = b; g_kdL = c1; g_kpR = d; g_kiR = e; g_kdR = f; }
                    else if (cnt >= 3)
                        { g_kpL = g_kpR = a; g_kiL = g_kiR = b; g_kdL = g_kdR = c1; }
                }
                else if (lineBuf[0] == 'L' && lineBuf[1] == ' ')
                {
                    cnt = std::sscanf(lineBuf + 2, "%f %f %f", &a, &b, &c1);
                    if (cnt == 3)
                        { g_kpL = a; g_kiL = b; g_kdL = c1; }
                }
                else if (lineBuf[0] == 'R' && lineBuf[1] == ' ')
                {
                    cnt = std::sscanf(lineBuf + 2, "%f %f %f", &a, &b, &c1);
                    if (cnt == 3)
                        { g_kpR = a; g_kiR = b; g_kdR = c1; }
                }
                else if (lineBuf[0] == 'S' && lineBuf[1] == ' ')
                {
                    cnt = std::sscanf(lineBuf + 2, "%f %f", &a, &b);
                    if (cnt >= 2)
                    {
                        g_overrideTargetL = a;
                        g_overrideTargetR = b;
                        g_useSineTarget = false;
                    }
                }
                else if (lineBuf[0] == 'G' && (lineBuf[1] == '\0' || lineBuf[1] == ' '))
                    g_useSineTarget = true;

                lineLen = 0;
                continue;
            }
            lineBuf[lineLen++] = c;
        }
        if (lineLen >= sizeof(lineBuf) - 1)
            lineLen = 0;
    }
}

// 正弦速度命令闭环测试：约 10 秒，目标速度为正弦，编码器+PID 闭环，并向上位机发送目标与实际速度
static void runSineSpeedTest(Serial& serial)
{
    MotorController motor;
    motor.init();
    motor.setPID(0.1f, 0.00f, 0.00f);

    const double duration  = 300.0;   // 测试时长（秒）
    const double amplitude = 30.0;   // 正弦幅值（与编码器速度同单位）
    const double freq      = 0.15;    // 正弦频率 Hz
    const double dt        = 0.01;   // 控制周期 10ms
    const int    usDt      = 10000;  // usleep 微秒

    struct timeval t0, t1;
    gettimeofday(&t0, nullptr);
    double t = 0.0;

    while (t < duration && !g_exitRequested)
    {
        motor.setPIDLeft(g_kpL, g_kiL, g_kdL);
        motor.setPIDRight(g_kpR, g_kiR, g_kdR);

        if (g_exitRequested)
            break;

        float targetL, targetR;
        if (g_useSineTarget)
        {
            double target = amplitude * std::sin(2.0 * M_PI * freq * t);
            targetL = targetR = static_cast<float>(target);
            motor.setTargetSpeed(targetL, targetR);
        }
        else
        {
            targetL = g_overrideTargetL;
            targetR = g_overrideTargetR;
            motor.setTargetSpeed(targetL, targetR);
        }

        motor.updateClosedLoop(dt);

        float actualL = motor.getEncoderSpeedLeft();
        float actualR = motor.getEncoderSpeedRight();
        sendSpeedToHost(serial, targetL, targetR, actualL, actualR);

        usleep(usDt);
        gettimeofday(&t1, nullptr);
        t = (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) * 1e-6;
    }

    motor.stop();
    // 仅当正常跑满 duration 时清零，否则保留 g_exitRequested 让主循环也能退出（如 Ctrl+C）
    if (t >= duration)
        g_exitRequested = 0;
}

// RGB565：蓝(左线)、绿(右线)、红(中线)
static constexpr uint16_t RGB565_BLUE  = 0x001F;
static constexpr uint16_t RGB565_GREEN = 0x07E0;
static constexpr uint16_t RGB565_RED   = 0xF800;

// 使用龙邱库逐点绘制边线（左蓝、右绿、中线红），各 3 像素宽，内部自动刷新 TFT
static void drawLineResultOnTFT(const Camera::LineSearchResult& result, int imgW, int imgH)
{
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
        // 中线用红色绘制（仅在有有效中线的行，即 y >= 10）
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
    {
        std::lock_guard<std::mutex> lock(g_serialMutex);
        serial.sendStr("sine speed test start (motor in thread)\n");
    }

    // 命令线程：阻塞读串口，解析 P/L/R/S/G 写全局 PID 与目标（不改龙邱库，无 select）
    std::thread cmdThread(commandReceiverThread, &serial);
    cmdThread.detach();
    // 电机线程：每拍从全局同步 PID/目标到 motor，闭环与发 VOFA
    std::thread motorThread(runSineSpeedTest, std::ref(serial));
    motorThread.detach();

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
        serial.sendStr("main start\n");

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
        // 采集原始分辨率图像
        if (!cam.grabGrayFrame(gray, tft_w, tft_h))
        {
            break;
        }

        // 对原图做上下颠倒
        cam.flipVertical(gray, gray);

        // 使用 Camera 内部封装好的：下采样 + 帧间采样 + 大津二值化逻辑
        cam.otsuBinarize(gray, binFull);
        cam.pixelFilterErode(binFull);  // 二值化后、搜线前做腐蚀/像素滤波去噪
        // 先显示二值化图像，再算中线，再用龙邱库逐点刷新边线（左蓝、右绿、中线红）
        cam.searchLineEightNeighbor(binFull, lineResult, 0);
        cam.calculateCenterLine(lineResult, tft_w, tft_h);
        TFT_ShowFullGray8(binFull.data);
        drawLineResultOnTFT(lineResult, tft_w, tft_h);

        // 读取 IMU 并显示解算姿态角（R/P/Y：横滚/俯仰/航向，单位度）
        if (imu.isInited() && imu.update())
        {
            imu.displayAttitude(0, 2);
        }

        // 计算帧率并显示在屏幕左上角
        double fps = updateAndShowFps(cam);

        // VOFA+ JustFloat：每 period 帧发一帧浮点数据（FPS, R, P, Y）
        //sendVofaIfNeeded(serial, frameCount, 15, fps, imu);
    }

    return 0;
}
