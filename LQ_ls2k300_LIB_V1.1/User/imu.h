#pragma once

#include <mutex>
#include <stdint.h>
#include <string>

#include "LQ_MPU6050.hpp"

// IMU 模块公共接口：
// 封装 MPU6050 初始化、陀螺仪零偏标定、姿态积分解算以及姿态角读取/显示。

// IMU 姿态解算类：
// 仅使用 MPU6050 陀螺仪角速度，按“零偏补偿 -> 死区 -> 低通 -> 四元数积分”解算姿态。
class Imu
{
public:
    Imu() = default;

    // 初始化 IMU（默认使用 LS_IIC_PATH 设备）
    bool init(const std::string& devPath = LS_IIC_PATH);

    // 读取一次原始数据并更新姿态角，返回 true 表示读取成功
    bool update();

    // 原始传感器读数
    int16_t ax() const;
    int16_t ay() const;
    int16_t az() const;
    int16_t gx() const;
    int16_t gy() const;
    int16_t gz() const;

    // 由四元数转换得到的姿态角（单位：度）
    // 仅使用陀螺仪时，三个角均为相对上电姿态的积分结果，长期会随零偏误差漂移。
    double rollDeg() const;
    double pitchDeg() const;
    double yawDeg() const;

    bool isInited() const;

    // 将当前解算的姿态角（R/P/Y）显示到 TFT 屏幕指定位置（行 row，列 col）
    void displayAttitude(uint8_t row = 0, uint8_t col = 2) const;

private:
    struct Vec3
    {
        double x;
        double y;
        double z;
    };

    struct Quaternion
    {
        double w;
        double x;
        double y;
        double z;
    };

    bool calibrateGyroBias();
    uint64_t nowUs() const;
    double rawGyroToRadPerSec(int16_t raw) const;
    Vec3 applyDeadband(const Vec3& omega) const;
    Vec3 lowPassFilter(const Vec3& omega, double dtSec);
    void integrateQuaternion(const Vec3& omega, double dtSec);
    void normalizeQuaternion();
    void updateEulerFromQuaternion();

    static double clamp(double value, double minValue, double maxValue);
    static double wrapAngleDeg(double angleDeg);

    mutable std::mutex stateMutex_;
    LS_I2C_DEV dev_;
    bool inited_ = false;
    bool biasReady_ = false;
    bool filterReady_ = false;

    int16_t ax_ = 0, ay_ = 0, az_ = 0;
    int16_t gx_ = 0, gy_ = 0, gz_ = 0;

    Vec3 gyroBiasRad_ { 0.0, 0.0, 0.0 };
    Vec3 gyroFilteredRad_ { 0.0, 0.0, 0.0 };
    Quaternion attitude_ { 1.0, 0.0, 0.0, 0.0 };

    double rollDeg_  = 0.0;
    double pitchDeg_ = 0.0;
    double yawDeg_   = 0.0;

    uint64_t lastUpdateUs_ = 0;

    static constexpr int kBiasSampleCount = 500;
    static constexpr int kBiasSampleIntervalUs = 2000;
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kDegToRad = kPi / 180.0;
    static constexpr double kRadToDeg = 180.0 / kPi;
    static constexpr double kGyroScaleLsbPerDegPerSec = 16.4;   // MPU6050 ±2000dps
    static constexpr double kGyroScaleRadPerSecPerLsb = kDegToRad / kGyroScaleLsbPerDegPerSec;
    static constexpr double kDeadbandRadPerSec = 0.02;          // 约 1.15 deg/s
    static constexpr double kGyroLpfCutoffHz = 8.0;
    static constexpr double kMaxDtSec = 0.10;
    static constexpr double kQuatNormEps = 1e-12;
};
