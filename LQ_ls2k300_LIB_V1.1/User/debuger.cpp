#include "debuger.h"
#include "controller.hpp"
#include "imu.h"
#include "serial.h"

#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

namespace {

constexpr double kControlPeriodSec = 0.01;
constexpr auto kControlPeriod = std::chrono::milliseconds(10);
constexpr int kCommandPollTimeoutMs = 20;
constexpr int kCommandFlushTimeoutMs = 100;
constexpr int kTelemetryDivider = 2;
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

struct SharedDebuggerState {
    std::mutex mtx;
    bool running = false;
    bool useSineTarget = false;
    bool faultActive = false;
    float manualTargetL = 0.0f;
    float manualTargetR = 0.0f;
    float kpL = 0.1f;
    float kiL = 0.0f;
    float kdL = 0.0f;
    float kpR = 0.1f;
    float kiR = 0.0f;
    float kdR = 0.0f;
};

struct ControlSnapshot {
    bool running = false;
    bool useSineTarget = false;
    bool faultActive = false;
    float targetL = 0.0f;
    float targetR = 0.0f;
    float kpL = 0.1f;
    float kiL = 0.0f;
    float kdL = 0.0f;
    float kpR = 0.1f;
    float kiR = 0.0f;
    float kdR = 0.0f;
};

static SharedDebuggerState g_sharedState;

static bool IsStopTarget(float left, float right)
{
    return std::fabs(left) < kStopTargetEpsilon && std::fabs(right) < kStopTargetEpsilon;
}

static void ResetSharedState()
{
    std::lock_guard<std::mutex> lock(g_sharedState.mtx);
    g_sharedState.running = false;
    g_sharedState.useSineTarget = false;
    g_sharedState.faultActive = false;
    g_sharedState.manualTargetL = 0.0f;
    g_sharedState.manualTargetR = 0.0f;
    g_sharedState.kpL = 1.0f;
    g_sharedState.kiL = 0.0f;
    g_sharedState.kdL = 0.0f;
    g_sharedState.kpR = 1.0f;
    g_sharedState.kiR = 0.0f;
    g_sharedState.kdR = 0.0f;
}

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

static void ApplyStopCommandLocked()
{
    g_sharedState.running = false;
    g_sharedState.useSineTarget = false;
    g_sharedState.manualTargetL = 0.0f;
    g_sharedState.manualTargetR = 0.0f;
}

static void LatchFaultStopLocked()
{
    ApplyStopCommandLocked();
    g_sharedState.faultActive = true;
}

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

static void MaybeSendSpeedTelemetry(Serial* serial, int& telemetryCounter,
                                    float targetLeft, float targetRight,
                                    float actualLeft, float actualRight)
{
    if (!serial || !serial->isOpen())
        return;

    ++telemetryCounter;
    if (telemetryCounter < kTelemetryDivider)
        return;

    telemetryCounter = 0;
    DebuggerSendSpeedToHost(*serial, targetLeft, targetRight, actualLeft, actualRight);
}

static void CommandReceiverThread(Serial* serial, volatile sig_atomic_t* exitFlag)
{
    if (!serial || !serial->isOpen())
        return;

    char lineBuf[128];
    size_t lineLen = 0;
    int idleWaitMs = 0;

    while (exitFlag && !*exitFlag)
    {
        if (!serial->waitReadable(kCommandPollTimeoutMs))
        {
            if (lineLen > 0)
            {
                idleWaitMs += kCommandPollTimeoutMs;
                if (idleWaitMs >= kCommandFlushTimeoutMs)
                {
                    lineBuf[lineLen] = '\0';
                    HandleCommandLine(lineBuf);
                    lineLen = 0;
                    idleWaitMs = 0;
                }
            }
            continue;
        }

        idleWaitMs = 0;
        char chunk[32];
        const int n = serial->receive(chunk, sizeof(chunk));
        if (n <= 0)
            continue;

        for (int i = 0; i < n; ++i)
        {
            const char ch = chunk[i];
            if (ch == '\n' || ch == '\r')
            {
                lineBuf[lineLen] = '\0';
                if (lineLen > 0)
                    HandleCommandLine(lineBuf);
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

static void MotorControlThread(Serial* serial, volatile sig_atomic_t* exitFlag)
{
    MotorController* motor = new MotorController();
    motor->init();
    motor->setRampStep(kOutputRampStep);

    auto lastTick = std::chrono::steady_clock::now();
    auto sineStart = lastTick;
    bool pidApplied = false;
    bool wasRunning = false;
    bool wasSineTarget = false;
    bool wasFaultActive = false;
    float appliedKpL = 1.0f;
    float appliedKiL = 0.0f;
    float appliedKdL = 0.0f;
    float appliedKpR = 1.0f;
    float appliedKiR = 0.0f;
    float appliedKdR = 0.0f;
    int stallCounterLeft = 0;
    int stallCounterRight = 0;
    int telemetryCounter = 0;

    while (exitFlag && !*exitFlag)
    {
        const auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTick).count();
        lastTick = now;
        if (dt <= 0.0 || dt > 0.1)
            dt = kControlPeriodSec;

        ControlSnapshot snapshot = SnapshotSharedState();
        if (!pidApplied ||
            snapshot.kpL != appliedKpL || snapshot.kiL != appliedKiL || snapshot.kdL != appliedKdL ||
            snapshot.kpR != appliedKpR || snapshot.kiR != appliedKiR || snapshot.kdR != appliedKdR)
        {
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
            motor->getEncoderSpeed(actualL, actualR);
            MaybeSendSpeedTelemetry(serial, telemetryCounter, 0.0f, 0.0f, actualL, actualR);

            wasRunning = false;
            wasSineTarget = false;
            wasFaultActive = true;
            stallCounterLeft = 0;
            stallCounterRight = 0;
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
            motor->getEncoderSpeed(actualL, actualR);
            MaybeSendSpeedTelemetry(serial, telemetryCounter, snapshot.targetL, snapshot.targetR,
                                    actualL, actualR);

            wasRunning = false;
            wasSineTarget = false;
            stallCounterLeft = 0;
            stallCounterRight = 0;
            std::this_thread::sleep_until(now + kControlPeriod);
            continue;
        }

        if (snapshot.useSineTarget)
        {
            if (!wasRunning || !wasSineTarget)
                sineStart = now;

            const double t = std::chrono::duration<double>(now - sineStart).count();
            const float target = static_cast<float>(kSineAmplitude * std::sin(kTwoPi * kSineFreqHz * t));
            snapshot.targetL = target;
            snapshot.targetR = target;
        }

        motor->setTargetSpeed(snapshot.targetL, snapshot.targetR);
        motor->updateClosedLoop(dt);

        float actualL = 0.0f;
        float actualR = 0.0f;
        int appliedOutL = 0;
        int appliedOutR = 0;
        motor->getLastMeasuredSpeed(actualL, actualR);
        motor->getAppliedOutput(appliedOutL, appliedOutR);

        if (kEnableStallProtection)
        {
            const bool leftStalled =
                std::fabs(snapshot.targetL) >= kStallTargetThreshold &&
                std::abs(appliedOutL) >= kStallOutputThreshold &&
                std::fabs(actualL) <= kStallSpeedThreshold;
            const bool rightStalled =
                std::fabs(snapshot.targetR) >= kStallTargetThreshold &&
                std::abs(appliedOutR) >= kStallOutputThreshold &&
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
                                actualL, actualR);

        wasRunning = true;
        wasSineTarget = snapshot.useSineTarget;
        std::this_thread::sleep_until(now + kControlPeriod);
    }

    motor->shutdown();
    if (serial && serial->isOpen())
        DebuggerSendSpeedToHost(*serial, 0.0f, 0.0f, 0.0f, 0.0f);
}

} // namespace

void DebuggerSendStr(Serial& serial, const char* str)
{
    if (!str)
        return;
    std::lock_guard<std::mutex> lock(g_serialTxMutex);
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
    std::lock_guard<std::mutex> lock(g_serialTxMutex);
    serial.sendVofaJustFloat(vofa, 4);
}

void DebuggerSendSpeedToHost(Serial& serial,
                             float targetLeft, float targetRight,
                             float actualLeft, float actualRight)
{
    if (!serial.isOpen())
        return;
    float vofa[4] = { targetLeft, targetRight, actualLeft, actualRight };
    std::lock_guard<std::mutex> lock(g_serialTxMutex);
    serial.sendVofaJustFloat(vofa, 4);
}

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

void DebuggerUpdateAndDisplayImu(Imu& imu, uint8_t row, uint8_t col)
{
    if (imu.isInited() && imu.update())
        imu.displayAttitude(row, col);
}
