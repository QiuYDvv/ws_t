#include "imu.h"
#include "LQ_TFT18_dri.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <unistd.h>

int16_t Imu::ax() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return ax_;
}

int16_t Imu::ay() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return ay_;
}

int16_t Imu::az() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return az_;
}

int16_t Imu::gx() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return gx_;
}

int16_t Imu::gy() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return gy_;
}

int16_t Imu::gz() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return gz_;
}

double Imu::rollDeg() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return rollDeg_;
}

double Imu::pitchDeg() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return pitchDeg_;
}

double Imu::yawDeg() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return yawDeg_;
}

bool Imu::isInited() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return inited_;
}

bool Imu::init(const std::string& devPath)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (inited_)
        return true;

    if (dev_.I2C_Init(devPath) != 0)
    {
        std::printf("IMU I2C_Init failed, path=%s\n", devPath.c_str());
        inited_ = false;
        return false;
    }

    attitude_ = { 1.0, 0.0, 0.0, 0.0 };
    gyroBiasRad_ = { 0.0, 0.0, 0.0 };
    gyroFilteredRad_ = { 0.0, 0.0, 0.0 };
    rollDeg_ = 0.0;
    pitchDeg_ = 0.0;
    yawDeg_ = 0.0;
    lastUpdateUs_ = 0;
    biasReady_ = false;
    filterReady_ = false;

    if (!calibrateGyroBias())
    {
        std::printf("IMU gyro bias calibration failed\n");
        inited_ = false;
        return false;
    }

    lastUpdateUs_ = nowUs();
    inited_ = true;
    return true;
}

bool Imu::update()
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!inited_ || !biasReady_)
        return false;

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    if (dev_.I2C_Get_RawData(&ax, &ay, &az, &gx, &gy, &gz) != 0)
        return false;

    ax_ = ax;
    ay_ = ay;
    az_ = az;
    gx_ = gx;
    gy_ = gy;
    gz_ = gz;

    const uint64_t now = nowUs();
    if (lastUpdateUs_ == 0 || now <= lastUpdateUs_)
    {
        lastUpdateUs_ = now;
        updateEulerFromQuaternion();
        return true;
    }

    double dtSec = static_cast<double>(now - lastUpdateUs_) / 1e6;
    lastUpdateUs_ = now;
    if (dtSec <= 0.0)
        return true;
    if (dtSec > kMaxDtSec)
        dtSec = kMaxDtSec;

    // 第一步：读取陀螺仪原始角速度，并统一转换为 rad/s。
    const Vec3 omegaRaw = {
        rawGyroToRadPerSec(gx_),
        rawGyroToRadPerSec(gy_),
        rawGyroToRadPerSec(gz_)
    };

    // 第二步：零偏补偿。
    const Vec3 omegaBiasComp = {
        omegaRaw.x - gyroBiasRad_.x,
        omegaRaw.y - gyroBiasRad_.y,
        omegaRaw.z - gyroBiasRad_.z
    };

    // 第三步：死区处理。
    const Vec3 omegaDeadband = applyDeadband(omegaBiasComp);

    // 第四步：一阶低通滤波。
    const Vec3 omegaFiltered = lowPassFilter(omegaDeadband, dtSec);

    // 第五步：四元数积分更新姿态。
    integrateQuaternion(omegaFiltered, dtSec);

    // 第六步：四元数归一化。
    normalizeQuaternion();

    // 第七步：仅在输出时转欧拉角。
    updateEulerFromQuaternion();
    return true;
}

bool Imu::calibrateGyroBias()
{
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    for (int i = 0; i < kBiasSampleCount; ++i)
    {
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;
        if (dev_.I2C_Get_Ang(&gx, &gy, &gz) != 0)
            return false;

        sumX += static_cast<double>(gx);
        sumY += static_cast<double>(gy);
        sumZ += static_cast<double>(gz);
        usleep(kBiasSampleIntervalUs);
    }

    const double invSampleCount = 1.0 / static_cast<double>(kBiasSampleCount);
    gyroBiasRad_ = {
        sumX * invSampleCount * kGyroScaleRadPerSecPerLsb,
        sumY * invSampleCount * kGyroScaleRadPerSecPerLsb,
        sumZ * invSampleCount * kGyroScaleRadPerSecPerLsb
    };

    biasReady_ = true;
    filterReady_ = false;
    return true;
}

uint64_t Imu::nowUs() const
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0;

    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec / 1000ULL);
}

double Imu::rawGyroToRadPerSec(int16_t raw) const
{
    return static_cast<double>(raw) * kGyroScaleRadPerSecPerLsb;
}

Imu::Vec3 Imu::applyDeadband(const Vec3& omega) const
{
    Vec3 out = omega;
    if (std::fabs(out.x) < kDeadbandRadPerSec)
        out.x = 0.0;
    if (std::fabs(out.y) < kDeadbandRadPerSec)
        out.y = 0.0;
    if (std::fabs(out.z) < kDeadbandRadPerSec)
        out.z = 0.0;
    return out;
}

Imu::Vec3 Imu::lowPassFilter(const Vec3& omega, double dtSec)
{
    if (!filterReady_)
    {
        gyroFilteredRad_ = omega;
        filterReady_ = true;
        return gyroFilteredRad_;
    }

    if (dtSec <= 0.0 || kGyroLpfCutoffHz <= 0.0)
        return gyroFilteredRad_;

    const double tau = 1.0 / (2.0 * kPi * kGyroLpfCutoffHz);
    const double alpha = dtSec / (tau + dtSec);

    gyroFilteredRad_.x += alpha * (omega.x - gyroFilteredRad_.x);
    gyroFilteredRad_.y += alpha * (omega.y - gyroFilteredRad_.y);
    gyroFilteredRad_.z += alpha * (omega.z - gyroFilteredRad_.z);
    return gyroFilteredRad_;
}

void Imu::integrateQuaternion(const Vec3& omega, double dtSec)
{
    // q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
    // 离散时使用增量四元数 q(k+1) = q(k) ⊗ dq，比直接积分欧拉角更稳定。
    const double omegaNorm = std::sqrt(omega.x * omega.x + omega.y * omega.y + omega.z * omega.z);

    Quaternion delta = { 1.0, 0.0, 0.0, 0.0 };
    if (omegaNorm > 1e-9)
    {
        const double halfTheta = 0.5 * omegaNorm * dtSec;
        const double sinHalfThetaOverNorm = std::sin(halfTheta) / omegaNorm;
        delta.w = std::cos(halfTheta);
        delta.x = omega.x * sinHalfThetaOverNorm;
        delta.y = omega.y * sinHalfThetaOverNorm;
        delta.z = omega.z * sinHalfThetaOverNorm;
    }
    else
    {
        const double halfDt = 0.5 * dtSec;
        delta.x = omega.x * halfDt;
        delta.y = omega.y * halfDt;
        delta.z = omega.z * halfDt;
    }

    const Quaternion current = attitude_;
    attitude_ = {
        current.w * delta.w - current.x * delta.x - current.y * delta.y - current.z * delta.z,
        current.w * delta.x + current.x * delta.w + current.y * delta.z - current.z * delta.y,
        current.w * delta.y - current.x * delta.z + current.y * delta.w + current.z * delta.x,
        current.w * delta.z + current.x * delta.y - current.y * delta.x + current.z * delta.w
    };
}

void Imu::normalizeQuaternion()
{
    const double norm = std::sqrt(attitude_.w * attitude_.w + attitude_.x * attitude_.x +
                                  attitude_.y * attitude_.y + attitude_.z * attitude_.z);
    if (norm <= kQuatNormEps)
    {
        attitude_ = { 1.0, 0.0, 0.0, 0.0 };
        return;
    }

    const double invNorm = 1.0 / norm;
    attitude_.w *= invNorm;
    attitude_.x *= invNorm;
    attitude_.y *= invNorm;
    attitude_.z *= invNorm;
}

void Imu::updateEulerFromQuaternion()
{
    const double sinrCosp = 2.0 * (attitude_.w * attitude_.x + attitude_.y * attitude_.z);
    const double cosrCosp = 1.0 - 2.0 * (attitude_.x * attitude_.x + attitude_.y * attitude_.y);
    const double sinp = 2.0 * (attitude_.w * attitude_.y - attitude_.z * attitude_.x);
    const double sinyCosp = 2.0 * (attitude_.w * attitude_.z + attitude_.x * attitude_.y);
    const double cosyCosp = 1.0 - 2.0 * (attitude_.y * attitude_.y + attitude_.z * attitude_.z);

    rollDeg_ = wrapAngleDeg(std::atan2(sinrCosp, cosrCosp) * kRadToDeg);
    pitchDeg_ = std::asin(clamp(sinp, -1.0, 1.0)) * kRadToDeg;
    yawDeg_ = wrapAngleDeg(std::atan2(sinyCosp, cosyCosp) * kRadToDeg);
}

double Imu::clamp(double value, double minValue, double maxValue)
{
    if (value < minValue)
        return minValue;
    if (value > maxValue)
        return maxValue;
    return value;
}

double Imu::wrapAngleDeg(double angleDeg)
{
    while (angleDeg > 180.0)
        angleDeg -= 360.0;
    while (angleDeg < -180.0)
        angleDeg += 360.0;
    return angleDeg;
}

void Imu::displayAttitude(uint8_t row, uint8_t col) const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    char buf[40];
    std::snprintf(buf, sizeof(buf), "R:%5.1f P:%5.1f Y:%5.1f",
                  rollDeg_, pitchDeg_, yawDeg_);
    TFTSPI_dir_P6X8Str(row, col, buf, u16WHITE, u16BLACK);
}
