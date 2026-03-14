#include "debuger.h"
#include "controller.hpp"
#include "imu.h"
#include "serial.h"
#include <cmath>
#include <cstdio>
#include <mutex>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

namespace {

static std::mutex g_serialMutex;

// 为「目标速度」是否用正弦：false 时用下面两个 override 值（如 0 0 即停止）
static bool g_useSineTarget = true;
static float g_overrideTargetL = 0.0f;
static float g_overrideTargetR = 0.0f;

// 上位机下发的 PID 参数，电机线程每拍从这些全局量同步到 motor
static float g_kpL = 0.1f, g_kiL = 0.0f, g_kdL = 0.0f;
static float g_kpR = 0.1f, g_kiR = 0.0f, g_kdR = 0.0f;

static void commandReceiverThread(Serial* serial, volatile sig_atomic_t* exitFlag)
{
    if (!serial || !serial->isOpen())
        return;
    char lineBuf[128];
    size_t lineLen = 0;

    while (exitFlag && !*exitFlag)
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

static void runSineSpeedTest(Serial& serial, volatile sig_atomic_t* exitFlag)
{
    MotorController motor;
    motor.init();
    motor.setPID(0.1f, 0.00f, 0.00f);

    const double duration  = 300.0;   // 测试时长（秒）
    const double amplitude = 30.0;    // 正弦幅值（与编码器速度同单位）
    const double freq      = 0.15;    // 正弦频率 Hz
    const double dt        = 0.01;    // 控制周期 10ms
    const int    usDt      = 10000;   // usleep 微秒

    struct timeval t0, t1;
    gettimeofday(&t0, nullptr);
    double t = 0.0;

    while (exitFlag && t < duration && !*exitFlag)
    {
        motor.setPIDLeft(g_kpL, g_kiL, g_kdL);
        motor.setPIDRight(g_kpR, g_kiR, g_kdR);

        if (exitFlag && *exitFlag)
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
        DebuggerSendSpeedToHost(serial, targetL, targetR, actualL, actualR);

        usleep(usDt);
        gettimeofday(&t1, nullptr);
        t = (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) * 1e-6;
    }

    motor.stop();
    if (exitFlag && t >= duration)
        *exitFlag = 0;
}

} // namespace

void DebuggerSendStr(Serial& serial, const char* str)
{
    if (!str)
        return;
    std::lock_guard<std::mutex> lock(g_serialMutex);
    serial.sendStr(str);
}

void DebuggerSendVofaIfNeeded(Serial& serial, int& frameCount, int period,
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

void DebuggerSendSpeedToHost(Serial& serial,
                             float targetLeft, float targetRight,
                             float actualLeft, float actualRight)
{
    if (!serial.isOpen())
        return;
    float vofa[4] = { targetLeft, targetRight, actualLeft, actualRight };
    std::lock_guard<std::mutex> lock(g_serialMutex);
    serial.sendVofaJustFloat(vofa, 4);
}

void StartDebuggerThreads(Serial& serial, volatile sig_atomic_t* exitFlag)
{
    if (serial.isOpen())
        DebuggerSendStr(serial, "sine speed test start (motor in thread)\n");

    std::thread cmdThread(commandReceiverThread, &serial, exitFlag);
    cmdThread.detach();
    std::thread motorThread(runSineSpeedTest, std::ref(serial), exitFlag);
    motorThread.detach();
}

void DebuggerUpdateAndDisplayImu(Imu& imu, uint8_t row, uint8_t col)
{
    if (imu.isInited() && imu.update())
        imu.displayAttitude(row, col);
}
