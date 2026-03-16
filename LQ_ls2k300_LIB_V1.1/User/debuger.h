#pragma once

#include <csignal>
#include <cstdint>
#include <thread>

// 前向声明，避免 debuger 与主程序互相包含过多头文件
class Serial;
class Imu;

// 串口发送时加锁（与命令线程、电机线程共用串口时调用）
void DebuggerSendStr(Serial& serial, const char* str);

// 将收到的命令原样文本回显到串口，便于联调确认上位机发包内容
void DebuggerEchoCommand(Serial& serial, const char* line);

// 按周期通过串口发送 VOFA+ JustFloat 一帧（FPS, R, P, Y）
void DebuggerSendVofaIfNeeded(Serial& serial, int& frameCount, int period,
                              double fps, const Imu& imu);

// 将目标速度、编码器实际速度、左右电机当前输出 PWM 命令发送到上位机
// （VOFA+ JustFloat：targetL, targetR, actualL, actualR, pwmL, pwmR）
void DebuggerSendSpeedToHost(Serial& serial,
                             float targetLeft, float targetRight,
                             float actualLeft, float actualRight,
                             float pwmLeft, float pwmRight);

/**
 * 启动调试线程：命令接收线程 + 电机控制线程。
 * 默认待机不输出，G 启动正弦目标，S left right 切手动目标，X/STOP 或 S 0 0 停车。
 * exitFlag 由主程序在 SIGINT 时置 1；主程序退出前必须对 outCmdThread、outMotorThread 调用 join()，
 * 以避免 use-after-free（serial/exitFlag 被销毁后线程仍访问）。
 */
void StartDebuggerThreads(Serial& serial, volatile sig_atomic_t* exitFlag,
                         std::thread* outCmdThread, std::thread* outMotorThread);

// 若 PID 参数相对上次调用发生变化，则在终端打印当前左右轮 PID
void DebuggerPrintPidIfChanged();

// 若电机运行状态或目标速度相对上次调用发生变化，则在终端打印当前状态
void DebuggerPrintMotorStateIfChanged();

// 读取 IMU 并显示解算姿态角到 TFT（R/P/Y：横滚/俯仰/航向，单位度），row/col 为显示位置
void DebuggerUpdateAndDisplayImu(Imu& imu, uint8_t row = 0, uint8_t col = 2);
