// 调试模块实现：
// 负责上位机串口命令解析、电机测试线程、VOFA 数据发送以及终端/TFT 调试输出。
#include "debuger.h"
#include "controller.hpp"
#include "imu.h"
#include "serial.h"
#include "thread_timing.hpp"

#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

namespace {

// 调试线程运行参数：
// 控制命令轮询周期、电机控制周期、遥测发送节流和堵转保护阈值。
// 说明：在 LoongOS 上，10ms 的 sleep_until 往往跑不满 100Hz，实际会落在 ~60Hz 左右；
// 这里保持“期望 100Hz”的 10ms 周期，但 PID 的 dt 使用实际测量值（见 MotorControlThread），
// 因此即便实际频率偏低，也能按真实 dt 计算积分/微分项。
constexpr double kControlPeriodSec = 0.01;
constexpr auto kControlPeriod = std::chrono::milliseconds(10);
constexpr int kCommandPollTimeoutMs = 20;
constexpr int kCommandFlushTimeoutMs = 100;
constexpr int kTelemetryDivider = 1;
constexpr int kOutputRampStep = 6;
// 先关闭堵转保护，便于现场继续联调；后续确认阈值后再打开。
constexpr bool kEnableStallProtection = false;
constexpr int kStallOutputThreshold = 70;
constexpr float kStallTargetThreshold = 8.0f;
constexpr float kStallSpeedThreshold = 2.0f;
constexpr int kStallDetectCycles = 80;
constexpr bool kSendTextFeedback = false;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kSineAmplitude = 30.0;
constexpr double kSineFreqHz = 0.15;
constexpr float kStopTargetEpsilon = 1e-3f;

static std::mutex g_serialTxMutex;

// 调试模块共享状态：
// 命令接收线程修改，电机控制线程读取，因此统一受 mtx 保护。
struct SharedDebuggerState {
    std::mutex mtx;
    bool running = false;
    bool useSineTarget = false;
    bool faultActive = false;
    float manualTargetL = 0.0f;
    float manualTargetR = 0.0f;
    float kpL = 2.4f;
    float kiL = 0.2f;
    float kdL = 0.0f;
    float kpR = 2.4f;
    float kiR = 0.2f;
    float kdR = 0.0f;
};

struct ControlSnapshot {
    bool running = false;
    bool useSineTarget = false;
    bool faultActive = false;
    float targetL = 0.0f;
    float targetR = 0.0f;
    float kpL = 2.4f;
    float kiL = 0.2f;
    float kdL = 0.0f;
    float kpR = 2.4f;
    float kiR = 0.2f;
    float kdR = 0.0f;
};

static SharedDebuggerState g_sharedState;

// 判断当前目标速度是否可视为“停车”命令。
static bool IsStopTarget(float left, float right)
{
    return std::fabs(left) < kStopTargetEpsilon && std::fabs(right) < kStopTargetEpsilon;
}

// 恢复调试模块到开机默认状态。
static void ResetSharedState()
{
    std::lock_guard<std::mutex> lock(g_sharedState.mtx);
    g_sharedState.running = false;
    g_sharedState.useSineTarget = false;
    g_sharedState.faultActive = false;
    g_sharedState.manualTargetL = 0.0f;
    g_sharedState.manualTargetR = 0.0f;
    g_sharedState.kpL = 2.4f;
    g_sharedState.kiL = 0.2f;
    g_sharedState.kdL = 0.0f;
    g_sharedState.kpR = 2.4f;
    g_sharedState.kiR = 0.2f;
    g_sharedState.kdR = 0.0f;
}

// 将共享状态复制为当前控制线程可安全使用的一份快照，减少持锁时间。
static ControlSnapshot SnapshotSharedState()
{
    std::lock_guard<std::mutex> lock(g_sharedState.mtx);
    ControlSnapshot snapshot;
    snapshot.running = g_sharedState.running;
    snapshot.useSineTarget = g_sharedState.useSineTarget;
    snapshot.faultActive = g_sharedState.faultActive;
    snapshot.targetL = g_sharedState.manualTargetL;
    snapshot.targetR = g_sharedState.manualTargetR;
    snapshot.kpL = g_sharedState.kpL;
    snapshot.kiL = g_sharedState.kiL;
    snapshot.kdL = g_sharedState.kdL;
    snapshot.kpR = g_sharedState.kpR;
    snapshot.kiR = g_sharedState.kiR;
    snapshot.kdR = g_sharedState.kdR;
    return snapshot;
}

// 停车命令只清运行态与目标值，不把其视为故障。
static void ApplyStopCommandLocked()
{
    g_sharedState.running = false;
    g_sharedState.useSineTarget = false;
    g_sharedState.manualTargetL = 0.0f;
    g_sharedState.manualTargetR = 0.0f;
}

// 故障停车在普通停车基础上额外置 faultActive，提示控制线程保持停机。
static void LatchFaultStopLocked()
{
    ApplyStopCommandLocked();
    g_sharedState.faultActive = true;
}

// 将当前目标模式映射为便于打印的简短状态字符串。
static const char* DescribeMotorMotion(const ControlSnapshot& snapshot)
{
    if (snapshot.faultActive)
        return "fault_stop";
    if (!snapshot.running || IsStopTarget(snapshot.targetL, snapshot.targetR))
        return "stopped";
    if (snapshot.useSineTarget)
        return "sine";

    const bool leftForward = snapshot.targetL > kStopTargetEpsilon;
    const bool leftReverse = snapshot.targetL < -kStopTargetEpsilon;
    const bool rightForward = snapshot.targetR > kStopTargetEpsilon;
    const bool rightReverse = snapshot.targetR < -kStopTargetEpsilon;

    if (leftForward && rightForward)
        return "forward";
    if (leftReverse && rightReverse)
        return "reverse";
    if ((leftForward && rightReverse) || (leftReverse && rightForward))
        return "spin";
    return "differential";
}

// 解析上位机命令：
// P/L/R 调 PID，S 设置手动目标，G 开正弦目标，X/STOP 停车。
static void HandleCommandLine(const char* line)
{
    if (!line || line[0] == '\0')
        return;

    const unsigned char rawCmd = static_cast<unsigned char>(line[0]);
    const char cmd = static_cast<char>(std::toupper(rawCmd));
    const char* args = line + 1;
    while (*args == ' ')
        ++args;

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
    float e = 0.0f;
    float f = 0.0f;

    std::lock_guard<std::mutex> lock(g_sharedState.mtx);

    if (cmd == 'P')
    {
        const int cnt = std::sscanf(args, "%f %f %f %f %f %f", &a, &b, &c, &d, &e, &f);
        if (cnt == 6)
        {
            g_sharedState.kpL = a;
            g_sharedState.kiL = b;
            g_sharedState.kdL = c;
            g_sharedState.kpR = d;
            g_sharedState.kiR = e;
            g_sharedState.kdR = f;
        }
        else if (cnt >= 3)
        {
            g_sharedState.kpL = a;
            g_sharedState.kiL = b;
            g_sharedState.kdL = c;
            g_sharedState.kpR = a;
            g_sharedState.kiR = b;
            g_sharedState.kdR = c;
        }
        return;
    }

    if (cmd == 'L')
    {
        if (std::sscanf(args, "%f %f %f", &a, &b, &c) == 3)
        {
            g_sharedState.kpL = a;
            g_sharedState.kiL = b;
            g_sharedState.kdL = c;
        }
        return;
    }

    if (cmd == 'R')
    {
        if (std::sscanf(args, "%f %f %f", &a, &b, &c) == 3)
        {
            g_sharedState.kpR = a;
            g_sharedState.kiR = b;
            g_sharedState.kdR = c;
        }
        return;
    }

    if (cmd == 'S')
    {
        if (std::sscanf(args, "%f %f", &a, &b) >= 2)
        {
            if (IsStopTarget(a, b))
            {
                ApplyStopCommandLocked();
                g_sharedState.faultActive = false;
            }
            else
            {
                g_sharedState.manualTargetL = a;
                g_sharedState.manualTargetR = b;
                g_sharedState.useSineTarget = false;
                g_sharedState.running = true;
                g_sharedState.faultActive = false;
            }
        }
        return;
    }

    if (cmd == 'G')
    {
        g_sharedState.useSineTarget = true;
        g_sharedState.running = true;
        g_sharedState.faultActive = false;
        return;
    }

    if (cmd == 'X' || std::strcmp(line, "STOP") == 0 || std::strcmp(line, "stop") == 0)
    {
        ApplyStopCommandLocked();
        g_sharedState.faultActive = false;
    }
}

// 串口遥测发送节流，避免 10ms 控制周期下把串口打满。
static void MaybeSendSpeedTelemetry(Serial* serial, int& telemetryCounter,
                                    float targetLeft, float targetRight,
                                    float actualLeft, float actualRight,
                                    float pwmLeft, float pwmRight)
{
    if (!serial || !serial->isOpen())
        return;

    ++telemetryCounter;
    if (telemetryCounter < kTelemetryDivider)
        return;

    telemetryCounter = 0;
    DebuggerSendSpeedToHost(*serial, targetLeft, targetRight, actualLeft, actualRight, pwmLeft, pwmRight);
}

// 命令接收线程：
// 按行接收串口命令；若一行长时间未收到换行，则按超时自动提交。
static void CommandReceiverThread(Serial* serial, volatile sig_atomic_t* exitFlag)
{
    if (!serial || !serial->isOpen())
        return;

    lq::SetThisThreadName("cmd");
    lq::ThreadTiming timing("cmd");
    uint64_t bytesInInterval = 0;
    uint64_t linesInInterval = 0;

    char lineBuf[128];
    size_t lineLen = 0;
    int idleWaitMs = 0;

    while (exitFlag && !*exitFlag)
    {
        const bool readable = serial->waitReadable(kCommandPollTimeoutMs);
        const auto workStart = std::chrono::steady_clock::now();

        if (!readable)
        {
            if (lineLen > 0)
            {
                idleWaitMs += kCommandPollTimeoutMs;
                if (idleWaitMs >= kCommandFlushTimeoutMs)
                {
                    lineBuf[lineLen] = '\0';
                    DebuggerEchoCommand(*serial, lineBuf);
                    HandleCommandLine(lineBuf);
                    ++linesInInterval;
                    lineLen = 0;
                    idleWaitMs = 0;
                }
            }
        }
        else
        {
            idleWaitMs = 0;
            char chunk[32];
            const int n = serial->receive(chunk, sizeof(chunk));
            if (n > 0)
            {
                bytesInInterval += static_cast<uint64_t>(n);
                for (int i = 0; i < n; ++i)
                {
                    const char ch = chunk[i];
                    if (ch == '\n' || ch == '\r')
                    {
                        lineBuf[lineLen] = '\0';
                        if (lineLen > 0)
                        {
                            DebuggerEchoCommand(*serial, lineBuf);
                            HandleCommandLine(lineBuf);
                            ++linesInInterval;
                        }
                        lineLen = 0;
                        idleWaitMs = 0;
                        continue;
                    }

                    if (lineLen < sizeof(lineBuf) - 1)
                        lineBuf[lineLen++] = ch;
                    else
                    {
                        lineLen = 0;
                        idleWaitMs = 0;
                    }
                }
            }
        }

        const auto workEnd = std::chrono::steady_clock::now();
        const auto workNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(workEnd - workStart).count());
        timing.RecordWork(workNs, false);
        lq::ThreadTimingReport report;
        if (timing.TryMakeReport(&report))
        {
            const double invSec = (report.wall_s > 1e-9) ? (1.0 / report.wall_s) : 0.0;
            char extra[96];
            std::snprintf(extra, sizeof(extra), "bytes/s=%.0f lines/s=%.1f",
                          static_cast<double>(bytesInInterval) * invSec,
                          static_cast<double>(linesInInterval) * invSec);
            lq::PrintThreadTimingLine(timing.name(), report, extra);
            bytesInInterval = 0;
            linesInInterval = 0;
        }
    }
}

// 电机控制线程：
// 周期读取共享状态，更新目标速度/PID，执行闭环控制并回传遥测。
static void MotorControlThread(Serial* serial, volatile sig_atomic_t* exitFlag)
{
    lq::SetThisThreadName("motor");
    lq::ThreadTiming timing("motor");

    MotorController* motor = new MotorController();
    motor->init();
    motor->setRampStep(kOutputRampStep);

    auto lastTick = std::chrono::steady_clock::now();
    auto sineStart = lastTick;
    bool pidApplied = false;
    bool wasRunning = false;
    bool wasSineTarget = false;
    bool wasFaultActive = false;
    float appliedKpL = 2.4f;
    float appliedKiL = 0.2f;
    float appliedKdL = 0.0f;
    float appliedKpR = 2.4f;
    float appliedKiR = 0.2f;
    float appliedKdR = 0.0f;
    int stallCounterLeft = 0;
    int stallCounterRight = 0;
    int telemetryCounter = 0;

    while (exitFlag && !*exitFlag)
    {
        const auto now = std::chrono::steady_clock::now();
        const auto cycleStart = now;
        const auto cycleDeadline = now + kControlPeriod;

        double dt = std::chrono::duration<double>(now - lastTick).count();
        lastTick = now;
        if (dt <= 0.0 || dt > 0.1)
            dt = kControlPeriodSec;

        ControlSnapshot snapshot = SnapshotSharedState();
        if (!pidApplied ||
            snapshot.kpL != appliedKpL || snapshot.kiL != appliedKiL || snapshot.kdL != appliedKdL ||
            snapshot.kpR != appliedKpR || snapshot.kiR != appliedKiR || snapshot.kdR != appliedKdR)
        {
            // 仅在参数变化时刷新 PID，避免每个周期重复写入。
            motor->setPID(snapshot.kpL, snapshot.kiL, snapshot.kdL,
                          snapshot.kpR, snapshot.kiR, snapshot.kdR);
            appliedKpL = snapshot.kpL;
            appliedKiL = snapshot.kiL;
            appliedKdL = snapshot.kdL;
            appliedKpR = snapshot.kpR;
            appliedKiR = snapshot.kiR;
            appliedKdR = snapshot.kdR;
            pidApplied = true;
        }

        if (kEnableStallProtection && snapshot.faultActive)
        {
            if (!wasFaultActive)
            {
                motor->stop();
                if (kSendTextFeedback && serial && serial->isOpen())
                    DebuggerSendStr(*serial, "motor protect: stall stop\n");
            }

            float actualL = 0.0f;
            float actualR = 0.0f;
            float appliedOutL = 0.0f;
            float appliedOutR = 0.0f;
            motor->getEncoderSpeed(actualL, actualR);
            motor->getAppliedOutput(appliedOutL, appliedOutR);
            MaybeSendSpeedTelemetry(serial, telemetryCounter, 0.0f, 0.0f, actualL, actualR,
                                    appliedOutL, appliedOutR);

            wasRunning = false;
            wasSineTarget = false;
            wasFaultActive = true;
            stallCounterLeft = 0;
            stallCounterRight = 0;

            const auto cycleEnd = std::chrono::steady_clock::now();
            const auto workNs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(cycleEnd - cycleStart).count());
            timing.RecordWork(workNs, cycleEnd > cycleDeadline);
            lq::ThreadTimingReport report;
            if (timing.TryMakeReport(&report))
                lq::PrintThreadTimingLine(timing.name(), report);

            std::this_thread::sleep_until(now + kControlPeriod);
            continue;
        }

        wasFaultActive = false;

        if (!snapshot.running)
        {
            if (wasRunning)
            {
                motor->stop();
            }

            float actualL = 0.0f;
            float actualR = 0.0f;
            float appliedOutL = 0.0f;
            float appliedOutR = 0.0f;
            motor->getEncoderSpeed(actualL, actualR);
            motor->getAppliedOutput(appliedOutL, appliedOutR);
            MaybeSendSpeedTelemetry(serial, telemetryCounter, snapshot.targetL, snapshot.targetR,
                                    actualL, actualR,
                                    appliedOutL, appliedOutR);

            wasRunning = false;
            wasSineTarget = false;
            stallCounterLeft = 0;
            stallCounterRight = 0;

            const auto cycleEnd = std::chrono::steady_clock::now();
            const auto workNs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(cycleEnd - cycleStart).count());
            timing.RecordWork(workNs, cycleEnd > cycleDeadline);
            lq::ThreadTimingReport report;
            if (timing.TryMakeReport(&report))
                lq::PrintThreadTimingLine(timing.name(), report);

            std::this_thread::sleep_until(now + kControlPeriod);
            continue;
        }

        if (snapshot.useSineTarget)
        {
            if (!wasRunning || !wasSineTarget)
                sineStart = now;

            // 正弦模式用于现场快速验证闭环跟踪、换向和编码器反馈。
            const double t = std::chrono::duration<double>(now - sineStart).count();
            const float target = static_cast<float>(kSineAmplitude * std::sin(kTwoPi * kSineFreqHz * t));
            snapshot.targetL = target;
            snapshot.targetR = target;
        }

        motor->setTargetSpeed(snapshot.targetL, snapshot.targetR);
        motor->updateClosedLoop(dt);

        float actualL = 0.0f;
        float actualR = 0.0f;
        float appliedOutL = 0.0f;
        float appliedOutR = 0.0f;
        motor->getLastMeasuredSpeed(actualL, actualR);
        motor->getAppliedOutput(appliedOutL, appliedOutR);

        if (kEnableStallProtection)
        {
            const bool leftStalled =
                std::fabs(snapshot.targetL) >= kStallTargetThreshold &&
                std::fabs(appliedOutL) >= kStallOutputThreshold &&
                std::fabs(actualL) <= kStallSpeedThreshold;
            const bool rightStalled =
                std::fabs(snapshot.targetR) >= kStallTargetThreshold &&
                std::fabs(appliedOutR) >= kStallOutputThreshold &&
                std::fabs(actualR) <= kStallSpeedThreshold;

            stallCounterLeft = leftStalled ? (stallCounterLeft + 1) : 0;
            stallCounterRight = rightStalled ? (stallCounterRight + 1) : 0;

            if (stallCounterLeft >= kStallDetectCycles || stallCounterRight >= kStallDetectCycles)
            {
                {
                    std::lock_guard<std::mutex> lock(g_sharedState.mtx);
                    LatchFaultStopLocked();
                }
                motor->stop();
                wasRunning = false;
                wasSineTarget = false;
                wasFaultActive = true;
                stallCounterLeft = 0;
                stallCounterRight = 0;

                const auto cycleEnd = std::chrono::steady_clock::now();
                const auto workNs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(cycleEnd - cycleStart).count());
                timing.RecordWork(workNs, cycleEnd > cycleDeadline);
                lq::ThreadTimingReport report;
                if (timing.TryMakeReport(&report))
                    lq::PrintThreadTimingLine(timing.name(), report);

                std::this_thread::sleep_until(now + kControlPeriod);
                continue;
            }
        }
        else
        {
            stallCounterLeft = 0;
            stallCounterRight = 0;
        }

        MaybeSendSpeedTelemetry(serial, telemetryCounter, snapshot.targetL, snapshot.targetR,
                                actualL, actualR,
                                appliedOutL, appliedOutR);

        wasRunning = true;
        wasSineTarget = snapshot.useSineTarget;

        const auto cycleEnd = std::chrono::steady_clock::now();
        const auto workNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(cycleEnd - cycleStart).count());
        timing.RecordWork(workNs, cycleEnd > cycleDeadline);
        lq::ThreadTimingReport report;
        if (timing.TryMakeReport(&report))
            lq::PrintThreadTimingLine(timing.name(), report);

        std::this_thread::sleep_until(now + kControlPeriod);
    }

    motor->shutdown();
    if (serial && serial->isOpen())
        DebuggerSendSpeedToHost(*serial, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

} // namespace

// 对串口发送统一加锁，避免命令回显、VOFA 和文本输出互相穿插。
void DebuggerSendStr(Serial& serial, const char* str)
{
    if (!str)
        return;
    std::lock_guard<std::mutex> lock(g_serialTxMutex);
    serial.sendStr(str);
}

// 将收到的一行命令回显给上位机，方便确认解析前的原始输入。
void DebuggerEchoCommand(Serial& serial, const char* line)
{
    if (!line || line[0] == '\0')
        return;

    char buf[160];
    std::snprintf(buf, sizeof(buf), "cmd: %s\n", line);
    DebuggerSendStr(serial, buf);
}

// 按固定帧间隔发送 IMU + FPS 四个浮点量到 VOFA。
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
    std::lock_guard<std::mutex> lock(g_serialTxMutex);
    serial.sendVofaJustFloat(vofa, 4);
}

// 发送速度闭环关键量，便于上位机做目标/反馈对比曲线。
void DebuggerSendSpeedToHost(Serial& serial,
                             float targetLeft, float targetRight,
                             float actualLeft, float actualRight,
                             float pwmLeft, float pwmRight)
{
    if (!serial.isOpen())
        return;
    float vofa[4] = { targetLeft, targetRight, actualLeft, actualRight };
    std::lock_guard<std::mutex> lock(g_serialTxMutex);
    serial.sendVofaJustFloat(vofa, 4);
}

// 对外统一启动两个后台线程，并在启动前重置共享状态。
void StartDebuggerThreads(Serial& serial, volatile sig_atomic_t* exitFlag,
                          std::thread* outCmdThread, std::thread* outMotorThread)
{
    ResetSharedState();

    if (kSendTextFeedback && serial.isOpen())
        DebuggerSendStr(serial, "debug ready: idle, G=sine, S l r=manual, X=stop, stall protect on\n");

    if (outCmdThread)
        *outCmdThread = std::thread(CommandReceiverThread, &serial, exitFlag);
    if (outMotorThread)
        *outMotorThread = std::thread(MotorControlThread, &serial, exitFlag);
}

// 仅当 PID 参数相对上次发生变化时打印，减少终端刷屏。
void DebuggerPrintPidIfChanged()
{
    const ControlSnapshot snapshot = SnapshotSharedState();

    static bool initialized = false;
    static float lastKpL = 0.0f;
    static float lastKiL = 0.0f;
    static float lastKdL = 0.0f;
    static float lastKpR = 0.0f;
    static float lastKiR = 0.0f;
    static float lastKdR = 0.0f;

    if (!initialized)
    {
        lastKpL = snapshot.kpL;
        lastKiL = snapshot.kiL;
        lastKdL = snapshot.kdL;
        lastKpR = snapshot.kpR;
        lastKiR = snapshot.kiR;
        lastKdR = snapshot.kdR;
        initialized = true;
        return;
    }

    if (snapshot.kpL == lastKpL && snapshot.kiL == lastKiL && snapshot.kdL == lastKdL &&
        snapshot.kpR == lastKpR && snapshot.kiR == lastKiR && snapshot.kdR == lastKdR)
    {
        return;
    }

    std::printf("PID updated: L(%.4f, %.4f, %.4f) R(%.4f, %.4f, %.4f)\n",
                snapshot.kpL, snapshot.kiL, snapshot.kdL,
                snapshot.kpR, snapshot.kiR, snapshot.kdR);
    std::fflush(stdout);

    lastKpL = snapshot.kpL;
    lastKiL = snapshot.kiL;
    lastKdL = snapshot.kdL;
    lastKpR = snapshot.kpR;
    lastKiR = snapshot.kiR;
    lastKdR = snapshot.kdR;
}

// 仅在运行模式或目标速度变化时打印电机状态，便于跟踪命令切换。
void DebuggerPrintMotorStateIfChanged()
{
    const ControlSnapshot snapshot = SnapshotSharedState();

    static bool initialized = false;
    static bool lastRunning = false;
    static bool lastUseSineTarget = false;
    static bool lastFaultActive = false;
    static float lastTargetL = 0.0f;
    static float lastTargetR = 0.0f;

    if (!initialized)
    {
        lastRunning = snapshot.running;
        lastUseSineTarget = snapshot.useSineTarget;
        lastFaultActive = snapshot.faultActive;
        lastTargetL = snapshot.targetL;
        lastTargetR = snapshot.targetR;
        initialized = true;
        return;
    }

    const bool stateChanged =
        snapshot.running != lastRunning ||
        snapshot.useSineTarget != lastUseSineTarget ||
        snapshot.faultActive != lastFaultActive ||
        snapshot.targetL != lastTargetL ||
        snapshot.targetR != lastTargetR;
    if (!stateChanged)
        return;

    if (snapshot.useSineTarget)
    {
        std::printf("Motor state: %s\n", DescribeMotorMotion(snapshot));
    }
    else
    {
        std::printf("Motor state: %s, targetL=%.2f, targetR=%.2f\n",
                    DescribeMotorMotion(snapshot), snapshot.targetL, snapshot.targetR);
    }
    std::fflush(stdout);

    lastRunning = snapshot.running;
    lastUseSineTarget = snapshot.useSineTarget;
    lastFaultActive = snapshot.faultActive;
    lastTargetL = snapshot.targetL;
    lastTargetR = snapshot.targetR;
}

// 若 IMU 已初始化，则在 TFT 上显示当前姿态角。
void DebuggerDisplayImu(const Imu& imu, uint8_t row, uint8_t col)
{
    if (imu.isInited())
        imu.displayAttitude(row, col);
}
