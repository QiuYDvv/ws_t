#pragma once

#include "main.hpp"

// 电机测试程序
void MotorDemo();

// 舵机测试程序
void ServoDemo();

// 久久派 21 个 GPIO 翻转测试
void GpioDemo();

// GPIO 输出测试(设备文件)
void GpioOutputDemo1(uint8_t gpio);

// GPIO 输出测试(硬件)
void GpioOutputDemo2(uint8_t gpio);

// GPIO 输入测试(设备文件)
void GpioInputDemo1();

// GPIO 输入测试(硬件)
void GpioInputDemo2();

// PWM 测试(设备文件)
void PwmDevDemo();

// PWM 测试(寄存器)
void PwmHWDemo();

// Gtim PWM 测试(硬件)
void GtimPwmDemo();

// 编码器测试(硬件)
void EncoderDemo();

// 摄像头测试
void CameraDemo();

// ADC 功能测试
void AdcFunDemo();

// tft屏幕测试程序
void TFTDemo();

// TFT 屏幕驱动测试程序
void TFT_Dri_Demo();

// 读取MPU6050或者ICM42605原始数据 测试
void Demo_IIC_Gyro();   // 模拟 IIC
void Demo_I2C_Gyro();   // 硬件 IIC

// TCP 测试程序
void TCP_Demo();

// UDP 测试程序
void UDP_Demo();

// 串口功能测试
void Uart_Demo();

// sleep()函数测试 -- 以秒为单位延时
void sleepDemo();

// usleep()函数测试 -- 以微秒为单位延时
void usleepDemo();

// nanosleep()函数测试 -- 以纳秒为单位延时
void nanosleepDemo();

// clock_nanosleep()函数测试 -- 以纳秒为单位延时
void clock_nanosleepDemo();

// 计算帧率
void CalculateFrameRate();

// 时间戳打印测试
void GetTimeDemo();

// 获取当前毫秒值(日历时间)
void GetCurrentMillisecond();

// 打印当前微秒值
void printMicTimestamp();

/*!
 * @brief   setitimer 它能设置周期性的定时器，还可指定不同类型的定时器（如真实时间、虚拟时间等）
 */
void setitimerDemo();

/*!
 * @brief   this_thread::sleep_for 使用线程休眠函数
 */
void HibernateDemo();
