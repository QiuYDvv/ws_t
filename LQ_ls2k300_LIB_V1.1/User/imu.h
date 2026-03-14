#pragma once

#include <stdint.h>
#include <string>

#include "LQ_MPU6050.hpp"

// 简单的 IMU 解算类：使用 MPU6050 原始数据计算俯仰角/横滚角
class Imu
{
public:
    Imu() = default;

    // 初始化 IMU（默认使用 LS_IIC_PATH 设备）
    bool init(const std::string& devPath = LS_IIC_PATH);

    // 读取一次原始数据并更新姿态角
    // 返回 true 表示读取成功
    bool update();

    // 原始传感器读数
    int16_t ax() const { return ax_; }
    int16_t ay() const { return ay_; }
    int16_t az() const { return az_; }
    int16_t gx() const { return gx_; }
    int16_t gy() const { return gy_; }
    int16_t gz() const { return gz_; }

    // 由加速度解算得到的姿态角（单位：度）
    double rollDeg()  const { return rollDeg_; }   // 横滚角（绕 X 轴）
    double pitchDeg() const { return pitchDeg_; }  // 俯仰角（绕 Y 轴）
    double yawDeg()   const { return yawDeg_; }    // 航向角（绕 Z 轴，由陀螺仪积分，会漂移）

    bool isInited() const { return inited_; }

    // 将当前解算的姿态角（R/P/Y）显示到 TFT 屏幕指定位置（行 row，列 col）
    void displayAttitude(uint8_t row = 0, uint8_t col = 2) const;

private:
    LS_I2C_DEV dev_;
    bool inited_ = false;

    int16_t ax_ = 0, ay_ = 0, az_ = 0;
    int16_t gx_ = 0, gy_ = 0, gz_ = 0;

    double rollDeg_  = 0.0;
    double pitchDeg_ = 0.0;
    double yawDeg_   = 0.0;   // 由 gz 积分得到，无磁力计故仅相对航向

    uint64_t lastUpdateUs_ = 0;  // 上次 update 的时刻（微秒），用于积分 dt
};

