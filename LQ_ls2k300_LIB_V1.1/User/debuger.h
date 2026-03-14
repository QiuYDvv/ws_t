#pragma once

#include <csignal>

// 前向声明，避免 debuger 与主程序互相包含过多头文件
class Serial;
class Imu;

// 串口发送时加锁（与命令线程、电机线程共用串口时调用）
void DebuggerSendStr(Serial& serial, const char* str);

// 按周期通过串口发送 VOFA+ JustFloat 一帧（FPS, R, P, Y）
void DebuggerSendVofaIfNeeded(Serial& serial, int& frameCount, int period,
                              double fps, const Imu& imu);

// 将目标速度与编码器实际速度发送到上位机（VOFA+ JustFloat：targetL, targetR, actualL, actualR）
void DebuggerSendSpeedToHost(Serial& serial,
                              float targetLeft, float targetRight,
                              float actualLeft, float actualRight);

// 启动调试线程：命令接收线程 + 正弦速度闭环测试线程；exitFlag 由主程序在 SIGINT 时置 1，测试正常结束时置 0
void StartDebuggerThreads(Serial& serial, volatile sig_atomic_t* exitFlag);
