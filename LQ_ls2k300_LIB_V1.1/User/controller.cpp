/* 电机速度控制器实现：开环 PWM + 方向 GPIO + 编码器 + PID 闭环 */

#include "controller.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

constexpr int MotorController::DEFAULT_PWM_CYCLE;
constexpr int MotorController::DEFAULT_MAX_DUTY;

namespace {

// 电机/编码器软件校准：
// 1. 左电机实际方向与逻辑命令相反，因此对左电机输出取反；
// 2. 如有接线换边，可打开左右输出交换；
// 3. 编码器符号保持与逻辑速度同向。
constexpr int kLeftMotorOutputSign = -1;
constexpr int kRightMotorOutputSign = 1;
constexpr bool kSwapMotorOutputChannels = false;
constexpr float kLeftEncoderSign = 1.0f;
constexpr float kRightEncoderSign = 1.0f;

static float ApplyEncoderSign(float value, float sign)
{
    return value * sign;
}

static int ApplyMotorOutputSign(int value, int sign)
{
    return value * sign;
}

} // namespace

MotorController::MotorController()
    : leftPWM_(PWM1, DEFAULT_PWM_CYCLE, 0, "inversed")
    , rightPWM_(PWM2, DEFAULT_PWM_CYCLE, 0, "inversed")
    , leftDir_(74, GPIO_Mode_Out)    // CAN3_RX 左电机方向
    , rightDir_(75, GPIO_Mode_Out)   // CAN3_TX 右电机方向
    , leftEncoder_(0, 73)            // 左编码器：PWM 通道 0，方向 GPIO73（与 LQ_Encoder_Demo 一致）
    , rightEncoder_(3, 72)           // 右编码器：PWM 通道 3，方向 GPIO72
    , maxDuty_(DEFAULT_MAX_DUTY)
    , maxOutput_(100)                // 速度输出限幅，默认 ±100
    , rampStep_(6)                   // 10ms 控制周期下每拍最多变化 6%，约 170ms 从 0 到满量程
    , inited_(false)
    , kpL_(2.4f), kiL_(0.2f), kdL_(0.0f)
    , kpR_(2.4f), kiR_(0.2f), kdR_(0.0f)
    , targetLeft_(0.0f), targetRight_(0.0f)
    , integralLeft_(0.0f), integralRight_(0.0f)
    , lastErrorLeft_(0.0f), lastErrorRight_(0.0f)
    , lastMeasuredLeft_(0.0f), lastMeasuredRight_(0.0f)
    , currentOutputLeft_(0), currentOutputRight_(0)
{
}

void MotorController::init()
{
    if (inited_)
    {
        stop();
        return;
    }

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

void MotorController::setRampStep(int rampStep)
{
    rampStep_ = std::max(1, std::min(100, rampStep));
}

int MotorController::rampOutputToward(int targetSpeed, int currentSpeed) const
{
    targetSpeed = std::max(-maxOutput_, std::min(maxOutput_, targetSpeed));

    // 方向切换必须先回零，避免电机与驱动桥瞬间反向冲击。
    if (targetSpeed != 0 && currentSpeed != 0 && ((targetSpeed > 0) != (currentSpeed > 0)))
        targetSpeed = 0;

    if (targetSpeed > currentSpeed + rampStep_)
        return currentSpeed + rampStep_;
    if (targetSpeed < currentSpeed - rampStep_)
        return currentSpeed - rampStep_;
    return targetSpeed;
}

void MotorController::setOneMotor(SetPWM& pwm, HWGpio& dir, int speed)
{
    speed = std::max(-maxOutput_, std::min(maxOutput_, speed));
    int duty = (std::abs(speed) * maxDuty_) / 100;
    pwm.SetDutyCycle(duty);
    dir.SetGpioValue(speed >= 0 ? 1 : 0);
}

void MotorController::applyRawSpeed(int left, int right)
{
    const int leftOutput = ApplyMotorOutputSign(left, kLeftMotorOutputSign);
    const int rightOutput = ApplyMotorOutputSign(right, kRightMotorOutputSign);

    if (kSwapMotorOutputChannels)
    {
        setOneMotor(rightPWM_, rightDir_, leftOutput);
        setOneMotor(leftPWM_, leftDir_, rightOutput);
        return;
    }

    setOneMotor(leftPWM_, leftDir_, leftOutput);
    setOneMotor(rightPWM_, rightDir_, rightOutput);
}

void MotorController::setSpeed(int left, int right)
{
    currentOutputLeft_ = std::max(-maxOutput_, std::min(maxOutput_, left));
    currentOutputRight_ = std::max(-maxOutput_, std::min(maxOutput_, right));
    applyRawSpeed(currentOutputLeft_, currentOutputRight_);
}

void MotorController::stop()
{
    resetPIDState();
    setSpeed(0, 0);
}

void MotorController::resetPIDState()
{
    targetLeft_ = 0.0f;
    targetRight_ = 0.0f;
    integralLeft_ = 0.0f;
    integralRight_ = 0.0f;
    lastErrorLeft_ = 0.0f;
    lastErrorRight_ = 0.0f;
}

void MotorController::shutdown()
{
    stop();
    if (!inited_)
        return;

    leftPWM_.Disable();
    rightPWM_.Disable();
    leftPWM_.UnexportPWM();
    rightPWM_.UnexportPWM();
    inited_ = false;
}

// 编码器无脉冲时龙邱库会令 val=1，导致 100000000/1/512≈195000，PID 误判为极大速度并饱和输出
// 超过此阈值视为无效（静止/无脉冲），按 0 处理，避免一上电就全速
static constexpr float ENCODER_SPEED_INVALID_THRESHOLD = 2000.0f;

float MotorController::getEncoderSpeedLeft()
{
    float v = ApplyEncoderSign(leftEncoder_.Update(), kLeftEncoderSign);
    lastMeasuredLeft_ =
        (v > ENCODER_SPEED_INVALID_THRESHOLD || v < -ENCODER_SPEED_INVALID_THRESHOLD) ? 0.0f : v;
    return lastMeasuredLeft_;
}

float MotorController::getEncoderSpeedRight()
{
    float v = ApplyEncoderSign(rightEncoder_.Update(), kRightEncoderSign);
    lastMeasuredRight_ =
        (v > ENCODER_SPEED_INVALID_THRESHOLD || v < -ENCODER_SPEED_INVALID_THRESHOLD) ? 0.0f : v;
    return lastMeasuredRight_;
}

void MotorController::getEncoderSpeed(float& out_left, float& out_right)
{
    out_left  = getEncoderSpeedLeft();
    out_right = getEncoderSpeedRight();
}

void MotorController::getLastMeasuredSpeed(float& out_left, float& out_right) const
{
    out_left = lastMeasuredLeft_;
    out_right = lastMeasuredRight_;
}

void MotorController::getAppliedOutput(int& out_left, int& out_right) const
{
    out_left = currentOutputLeft_;
    out_right = currentOutputRight_;
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

    const float maxIntegral = 100.0f;
    integral = std::max(-maxIntegral, std::min(maxIntegral, integral));

    float out = kp * error + ki * integral + kd * derivative;
    outSpeed = static_cast<int>(std::round(out));
    outSpeed = std::max(-maxOutput_, std::min(maxOutput_, outSpeed));
}

void MotorController::updateClosedLoop(double dt)
{
    float actualLeft  = getEncoderSpeedLeft();
    float actualRight = getEncoderSpeedRight();

    int targetOutputLeft = 0;
    int targetOutputRight = 0;
    updateOnePID(targetLeft_,  actualLeft,  dt, kpL_, kiL_, kdL_, integralLeft_,  lastErrorLeft_,  targetOutputLeft);
    updateOnePID(targetRight_, actualRight, dt, kpR_, kiR_, kdR_, integralRight_, lastErrorRight_, targetOutputRight);

    currentOutputLeft_ = rampOutputToward(targetOutputLeft, currentOutputLeft_);
    currentOutputRight_ = rampOutputToward(targetOutputRight, currentOutputRight_);
    applyRawSpeed(currentOutputLeft_, currentOutputRight_);
}
