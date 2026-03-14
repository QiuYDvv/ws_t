#pragma once

#include <csignal>
#include <cstdint>
#include <thread>

// 前向声明，避免 debuger 与主程序互相包含过多头文件
class Serial;
class Imu;

// 串口发送时加锁（与命令线程、电机线程共用串口时调用）
void DebuggerSendStr(Serial& serial, const char* str);

// 按周期通过串口发送 VOFA+ JustFloat 一帧（FPS, R, P, Y）
void DebuggerSendVofaIfNeeded(Serial& serial, int& frameCount, int period,
                              double fps, const Imu& imu);

// 将目标速度与编码器实际速度发送到上位机
// （VOFA+ JustFloat：targetL, targetR, actualL, actualR）
void DebuggerSendSpeedToHost(Serial& serial,
                             float targetLeft, float targetRight,
                             float actualLeft, float actualRight);

/**
 * 启动调试线程：命令接收线程 + 电机控制线程。
 * 默认待机不输出，G 启动正弦目标，S left right 切手动目标，X/STOP 或 S 0 0 停车。
 * exitFlag 由主程序在 SIGINT 时置 1；主程序退出前必须对 outCmdThread、outMotorThread 调用 join()，
 * 以避免 use-after-free（serial/exitFlag 被销毁后线程仍访问）。
 */
void StartDebuggerThreads(Serial& serial, volatile sig_atomic_t* exitFlag,
                         std::thread* outCmdThread, std::thread* outMotorThread);

// 读取 IMU 并显示解算姿态角到 TFT（R/P/Y：横滚/俯仰/航向，单位度），row/col 为显示位置
void DebuggerUpdateAndDisplayImu(Imu& imu, uint8_t row = 0, uint8_t col = 2);
