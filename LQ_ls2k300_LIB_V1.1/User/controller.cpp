/* 电机速度控制器实现：开环 PWM + 方向 GPIO + 编码器 + PID 闭环 */

#include "controller.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

constexpr int MotorController::DEFAULT_PWM_CYCLE;
constexpr int MotorController::DEFAULT_MAX_DUTY;

MotorController::MotorController()
    : leftPWM_(PWM1, DEFAULT_PWM_CYCLE, 0, "inversed")
    , rightPWM_(PWM2, DEFAULT_PWM_CYCLE, 0, "inversed")
    , leftDir_(74, GPIO_Mode_Out)    // CAN3_RX 左电机方向
    , rightDir_(75, GPIO_Mode_Out)   // CAN3_TX 右电机方向
    , leftEncoder_(0, 73)            // 左编码器：PWM 通道 0，方向 GPIO73（与 LQ_Encoder_Demo 一致）
    , rightEncoder_(3, 72)           // 右编码器：PWM 通道 3，方向 GPIO72
    , maxDuty_(DEFAULT_MAX_DUTY)
    , maxOutput_(100)                // 速度输出限幅，默认 ±100
    , inited_(false)
    , kpL_(0.1f), kiL_(0.0f), kdL_(0.0f)
    , kpR_(0.1f), kiR_(0.0f), kdR_(0.0f)
    , targetLeft_(0.0f), targetRight_(0.0f)
    , integralLeft_(0.0f), integralRight_(0.0f)
    , lastErrorLeft_(0.0f), lastErrorRight_(0.0f)
{
}

void MotorController::init()
{
    leftPWM_.Enable();
    rightPWM_.Enable();
    stop();
    inited_ = true;
}

void MotorController::setMaxDuty(int maxDuty)
{
    maxDuty_ = std::max(0, std::min(maxDuty, DEFAULT_PWM_CYCLE));
}

void MotorController::setMaxOutput(int maxOutput)
{
    maxOutput_ = std::max(1, std::min(100, maxOutput));
}

void MotorController::setOneMotor(SetPWM& pwm, HWGpio& dir, int speed)
{
    speed = std::max(-maxOutput_, std::min(maxOutput_, speed));
    int duty = (std::abs(speed) * maxDuty_) / 100;
    pwm.SetDutyCycle(duty);
    dir.SetGpioValue(speed >= 0 ? 1 : 0);
}

void MotorController::setSpeed(int left, int right)
{
    setOneMotor(leftPWM_, leftDir_, left);
    setOneMotor(rightPWM_, rightDir_, right);
}

void MotorController::stop()
{
    setSpeed(0, 0);
}

// 编码器无脉冲时龙邱库会令 val=1，导致 100000000/1/512≈195000，PID 误判为极大速度并饱和输出
// 超过此阈值视为无效（静止/无脉冲），按 0 处理，避免一上电就全速
static constexpr float ENCODER_SPEED_INVALID_THRESHOLD = 2000.0f;

float MotorController::getEncoderSpeedLeft()
{
    float v = leftEncoder_.Update();
    return (v > ENCODER_SPEED_INVALID_THRESHOLD || v < -ENCODER_SPEED_INVALID_THRESHOLD) ? 0.0f : v;
}

float MotorController::getEncoderSpeedRight()
{
    float v = rightEncoder_.Update();
    return (v > ENCODER_SPEED_INVALID_THRESHOLD || v < -ENCODER_SPEED_INVALID_THRESHOLD) ? 0.0f : v;
}

void MotorController::getEncoderSpeed(float& out_left, float& out_right)
{
    out_left  = getEncoderSpeedLeft();
    out_right = getEncoderSpeedRight();
}

void MotorController::setPID(float kp, float ki, float kd)
{
    kpL_ = kpR_ = kp;
    kiL_ = kiR_ = ki;
    kdL_ = kdR_ = kd;
    if (kp == 0.0f && ki == 0.0f && kd == 0.0f)
        integralLeft_ = integralRight_ = 0.0f;
}

void MotorController::setPID(float kpL, float kiL, float kdL, float kpR, float kiR, float kdR)
{
    kpL_ = kpL; kiL_ = kiL; kdL_ = kdL;
    kpR_ = kpR; kiR_ = kiR; kdR_ = kdR;
    if (kpL == 0.0f && kiL == 0.0f && kdL == 0.0f)
        integralLeft_ = 0.0f;
    if (kpR == 0.0f && kiR == 0.0f && kdR == 0.0f)
        integralRight_ = 0.0f;
}

void MotorController::setPIDLeft(float kp, float ki, float kd)
{
    kpL_ = kp; kiL_ = ki; kdL_ = kd;
    if (kp == 0.0f && ki == 0.0f && kd == 0.0f)
        integralLeft_ = 0.0f;
}

void MotorController::setPIDRight(float kp, float ki, float kd)
{
    kpR_ = kp; kiR_ = ki; kdR_ = kd;
    if (kp == 0.0f && ki == 0.0f && kd == 0.0f)
        integralRight_ = 0.0f;
}

void MotorController::setTargetSpeed(float target_left, float target_right)
{
    targetLeft_  = target_left;
    targetRight_ = target_right;
}

void MotorController::updateOnePID(float target, float actual, double dt,
                                   float kp, float ki, float kd,
                                   float& integral, float& lastError, int& outSpeed)
{
    float error = target - actual;
    integral += error * static_cast<float>(dt);
    float derivative = (dt > 1e-9f) ? static_cast<float>((error - lastError) / dt) : 0.0f;
    lastError = error;

    const float maxIntegral = 500.0f;
    integral = std::max(-maxIntegral, std::min(maxIntegral, integral));

    float out = kp * error + ki * integral + kd * derivative;
    outSpeed = static_cast<int>(std::round(out));
    outSpeed = std::max(-maxOutput_, std::min(maxOutput_, outSpeed));
}

void MotorController::updateClosedLoop(double dt)
{
    float actualLeft  = getEncoderSpeedLeft();
    float actualRight = getEncoderSpeedRight();

    int leftSpeed  = 0;
    int rightSpeed = 0;
    updateOnePID(targetLeft_,  actualLeft,  dt, kpL_, kiL_, kdL_, integralLeft_,  lastErrorLeft_,  leftSpeed);
    updateOnePID(targetRight_, actualRight, dt, kpR_, kiR_, kdR_, integralRight_, lastErrorRight_, rightSpeed);

    setSpeed(leftSpeed, rightSpeed);
}
