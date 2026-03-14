#include "imu.h"
#include "LQ_TFT18_dri.hpp"
#include <cmath>
#include <cstdio>
#include <sys/time.h>

bool Imu::init(const std::string& devPath)
{
    if (inited_)
        return true;

    if (dev_.I2C_Init(devPath) != 0)
    {
        std::printf("IMU I2C_Init failed, path=%s\n", devPath.c_str());
        inited_ = false;
        return false;
    }

    inited_ = true;
    return true;
}

bool Imu::update()
{
    if (!inited_)
        return false;

    int16_t ax, ay, az, gx, gy, gz;
    if (dev_.I2C_Get_RawData(&ax, &ay, &az, &gx, &gy, &gz) != 0)
    {
        return false;
    }

    ax_ = ax;
    ay_ = ay;
    az_ = az;
    gx_ = gx;
    gy_ = gy;
    gz_ = gz;

    // 使用加速度数据估算姿态角（简单重力解算）
    // 公式参考通用 MPU6050 姿态解算（单位：度）
    const double axf = static_cast<double>(ax_);
    const double ayf = static_cast<double>(ay_);
    const double azf = static_cast<double>(az_);

    const double rad2deg = 180.0 / M_PI;

    // 横滚角：绕 X 轴，反映左右倾斜
    rollDeg_ = std::atan2(ayf, azf) * rad2deg;

    // 俯仰角：绕 Y 轴，反映前后倾斜
    const double denom = std::sqrt(ayf * ayf + azf * azf);
    if (denom > 1e-6)
        pitchDeg_ = std::atan2(-axf, denom) * rad2deg;
    else
        pitchDeg_ = 0.0;

    // 航向角 yaw：对陀螺仪 Z 轴角速度积分（MPU6050 无磁力计，仅相对航向，会漂移）
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint64_t nowUs = static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + static_cast<uint64_t>(tv.tv_usec);
    if (lastUpdateUs_ > 0)
    {
        double dtSec = static_cast<double>(nowUs - lastUpdateUs_) / 1e6;
        // MPU6050 典型量程 ±2000°/s，灵敏度约 16.4 LSB/(°/s)
        const double gyroScale = 16.4;
        double gzDegPerSec = static_cast<double>(gz_) / gyroScale;
        yawDeg_ += gzDegPerSec * dtSec;
        // 限制在 [-180, 180] 方便显示
        while (yawDeg_ > 180.0)  yawDeg_ -= 360.0;
        while (yawDeg_ < -180.0) yawDeg_ += 360.0;
    }
    lastUpdateUs_ = nowUs;

    return true;
}

void Imu::displayAttitude(uint8_t row, uint8_t col) const
{
    char buf[40];
    std::snprintf(buf, sizeof(buf), "R:%5.1f P:%5.1f Y:%5.1f",
                 rollDeg_, pitchDeg_, yawDeg_);
    TFTSPI_dir_P6X8Str(row, col, buf, u16WHITE, u16BLACK);
}
