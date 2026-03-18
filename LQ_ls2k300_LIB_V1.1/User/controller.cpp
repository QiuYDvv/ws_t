/* 电机速度控制器实现：开环 PWM + 方向 GPIO + 编码器 + PID 闭环 */

#include "controller.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

constexpr int MotorController::DEFAULT_PWM_CYCLE;
constexpr int MotorController::DEFAULT_MAX_DUTY;

namespace {

// 方向/接线适配常量：
// 用于把“逻辑速度方向”映射到当前实际硬件接线方向，减少业务层关心接线差异。
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

template <typename T>
static T ApplyMotorOutputSign(T value, int sign)
{
    return value * static_cast<T>(sign);
}

constexpr float kOutputZeroEpsilon = 0.05f;
constexpr float kFilteredSpeedSnapEpsilon = 0.2f;

} // namespace

// 构造时完成底层外设对象、PID 初值和各种限幅参数初始化。
MotorController::MotorController()
    : leftPWM_(PWM1, DEFAULT_PWM_CYCLE, 0, "inversed")
    , rightPWM_(PWM2, DEFAULT_PWM_CYCLE, 0, "inversed")
    , leftDir_(74, GPIO_Mode_Out)    // CAN3_RX 左电机方向
    , rightDir_(75, GPIO_Mode_Out)   // CAN3_TX 右电机方向
    , leftEncoder_(0, 73)            // 左编码器：PWM 通道 0，方向 GPIO73（与 LQ_Encoder_Demo 一致）
    , rightEncoder_(3, 72)           // 右编码器：PWM 通道 3，方向 GPIO72
    , maxDuty_(DEFAULT_MAX_DUTY)
    , minStartDuty_(1)
    , maxOutput_(100)                // 速度输出限幅，默认 ±100
    , rampStep_(6)                   // 10ms 控制周期下每拍最多变化 6%，约 170ms 从 0 到满量程
    , inited_(false)
    , speedFilterAlpha_(0.25f)       // 一阶低通，抑制低速瞬时测速跳变
    , kpL_(2.4f), kiL_(0.2f), kdL_(0.0f)
    , kpR_(2.4f), kiR_(0.2f), kdR_(0.0f)
    , targetLeft_(0.0f), targetRight_(0.0f)
    , integralLeft_(0.0f), integralRight_(0.0f)
    , lastErrorLeft_(0.0f), lastErrorRight_(0.0f)
    , lastMeasuredLeft_(0.0f), lastMeasuredRight_(0.0f)
    , currentOutputLeft_(0.0f), currentOutputRight_(0.0f)
    , appliedOutputLeft_(0.0f), appliedOutputRight_(0.0f)
    , lastAppliedDutyLeft_(0), lastAppliedDutyRight_(0)
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

// 对外部给定的占空比上限做约束，避免超过 PWM 周期。
void MotorController::setMaxDuty(int maxDuty)
{
    maxDuty_ = std::max(0, std::min(maxDuty, DEFAULT_PWM_CYCLE));
}

// 对外部给定的速度百分比上限做约束。
void MotorController::setMaxOutput(int maxOutput)
{
    maxOutput_ = std::max(1, std::min(100, maxOutput));
}

// 斜坡限制越小，输出变化越柔和；越大，则响应更快。
void MotorController::setRampStep(int rampStep)
{
    rampStep_ = std::max(1, std::min(100, rampStep));
}

// 将目标输出按“斜坡 + 换向先回零”策略收敛到当前输出，减少机械和电气冲击。
float MotorController::rampOutputToward(float targetSpeed, float currentSpeed) const
{
    const float maxOutput = static_cast<float>(maxOutput_);
    const float rampStep = static_cast<float>(rampStep_);
    targetSpeed = std::max(-maxOutput, std::min(maxOutput, targetSpeed));

    // 方向切换必须先回零，避免电机与驱动桥瞬间反向冲击。
    if (std::fabs(targetSpeed) > kOutputZeroEpsilon &&
        std::fabs(currentSpeed) > kOutputZeroEpsilon &&
        ((targetSpeed > 0) != (currentSpeed > 0)))
    {
        targetSpeed = 0.0f;
    }

    float nextSpeed = targetSpeed;
    if (targetSpeed > currentSpeed + rampStep)
        nextSpeed = currentSpeed + rampStep;
    else if (targetSpeed < currentSpeed - rampStep)
        nextSpeed = currentSpeed - rampStep;

    return (std::fabs(nextSpeed) < kOutputZeroEpsilon && std::fabs(targetSpeed) < kOutputZeroEpsilon)
               ? 0.0f
               : nextSpeed;
}

// 将单个电机的逻辑速度换算为方向引脚和占空比，并返回真实下发后的速度近似值。
float MotorController::setOneMotor(SetPWM& pwm, HWGpio& dir, float speed, int& lastAppliedDuty)
{
    const float maxOutput = static_cast<float>(maxOutput_);
    speed = std::max(-maxOutput, std::min(maxOutput, speed));
    dir.SetGpioValue(speed >= 0.0f ? 1 : 0);

    if (maxDuty_ <= 0 || std::fabs(speed) < kOutputZeroEpsilon)
    {
        pwm.SetDutyCycle(0);
        lastAppliedDuty = 0;
        return 0.0f;
    }

    const float dutyFloat = std::fabs(speed) * static_cast<float>(maxDuty_) / maxOutput;
    int duty = static_cast<int>(std::round(dutyFloat));
    duty = std::max(0, std::min(maxDuty_, duty));

    const int minStartDuty = std::max(0, std::min(minStartDuty_, maxDuty_));
    if (lastAppliedDuty == 0 && duty > 0 && duty < minStartDuty)
        duty = minStartDuty;

    pwm.SetDutyCycle(duty);
    lastAppliedDuty = duty;

    const float appliedSpeed = static_cast<float>(duty) * maxOutput / static_cast<float>(maxDuty_);
    return speed >= 0.0f ? appliedSpeed : -appliedSpeed;
}

// 将左右轮逻辑速度统一映射到物理 PWM/方向口，兼容接线符号与左右通道交换。
void MotorController::applyRawSpeed(float left, float right)
{
    const float leftOutput = ApplyMotorOutputSign(left, kLeftMotorOutputSign);
    const float rightOutput = ApplyMotorOutputSign(right, kRightMotorOutputSign);

    if (kSwapMotorOutputChannels)
    {
        const float appliedLeftPhysical = setOneMotor(rightPWM_, rightDir_, leftOutput, lastAppliedDutyLeft_);
        const float appliedRightPhysical = setOneMotor(leftPWM_, leftDir_, rightOutput, lastAppliedDutyRight_);
        appliedOutputLeft_ = ApplyMotorOutputSign(appliedLeftPhysical, kLeftMotorOutputSign);
        appliedOutputRight_ = ApplyMotorOutputSign(appliedRightPhysical, kRightMotorOutputSign);
        return;
    }

    const float appliedLeftPhysical = setOneMotor(leftPWM_, leftDir_, leftOutput, lastAppliedDutyLeft_);
    const float appliedRightPhysical = setOneMotor(rightPWM_, rightDir_, rightOutput, lastAppliedDutyRight_);
    appliedOutputLeft_ = ApplyMotorOutputSign(appliedLeftPhysical, kLeftMotorOutputSign);
    appliedOutputRight_ = ApplyMotorOutputSign(appliedRightPhysical, kRightMotorOutputSign);
}

// 开环速度接口：直接按百分比输出，不使用编码器闭环。
void MotorController::setSpeed(int left, int right)
{
    currentOutputLeft_ = static_cast<float>(std::max(-maxOutput_, std::min(maxOutput_, left)));
    currentOutputRight_ = static_cast<float>(std::max(-maxOutput_, std::min(maxOutput_, right)));
    applyRawSpeed(currentOutputLeft_, currentOutputRight_);
}

// 停止时同时清目标和 PID 累积状态，避免再次启动时带旧积分。
void MotorController::stop()
{
    resetPIDState();
    setSpeed(0, 0);
}

// 清理 PID 积分、误差和目标速度缓存。
void MotorController::resetPIDState()
{
    targetLeft_ = 0.0f;
    targetRight_ = 0.0f;
    integralLeft_ = 0.0f;
    integralRight_ = 0.0f;
    lastErrorLeft_ = 0.0f;
    lastErrorRight_ = 0.0f;
}

// 关闭 PWM 输出并撤销导出，通常用于程序退出时的硬件清理。
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

// 左右轮编码器读取接口内部都会走统一的滤波逻辑。
float MotorController::getEncoderSpeedLeft()
{
    return readFilteredEncoderSpeed(leftEncoder_, kLeftEncoderSign, lastMeasuredLeft_);
}

float MotorController::getEncoderSpeedRight()
{
    return readFilteredEncoderSpeed(rightEncoder_, kRightEncoderSign, lastMeasuredRight_);
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

// 编码器原始测速会有跳变和无脉冲异常值，这里统一做合法性过滤与一阶低通。
float MotorController::readFilteredEncoderSpeed(LS_PwmEncoder& encoder, float sign, float& lastMeasured)
{
    float v = ApplyEncoderSign(encoder.Update(), sign);
    float validSpeed =
        (v > ENCODER_SPEED_INVALID_THRESHOLD || v < -ENCODER_SPEED_INVALID_THRESHOLD) ? 0.0f : v;

    lastMeasured += speedFilterAlpha_ * (validSpeed - lastMeasured);
    if (std::fabs(validSpeed) < kOutputZeroEpsilon && std::fabs(lastMeasured) < kFilteredSpeedSnapEpsilon)
        lastMeasured = 0.0f;

    return lastMeasured;
}

void MotorController::getAppliedOutput(float& out_left, float& out_right) const
{
    out_left = appliedOutputLeft_;
    out_right = appliedOutputRight_;
}

// 设置 PID 时支持“共用一组参数”或“左右独立参数”两种模式。
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

// 单轮 PID 计算：误差、积分、微分和输出限幅都在这里完成。
void MotorController::updateOnePID(float target, float actual, double dt,
                                   float kp, float ki, float kd,
                                   float& integral, float& lastError, float& outSpeed)
{
    float error = target - actual;
    integral += error * static_cast<float>(dt);
    float derivative = (dt > 1e-9f) ? static_cast<float>((error - lastError) / dt) : 0.0f;
    lastError = error;

    const float maxIntegral = 100.0f;
    integral = std::max(-maxIntegral, std::min(maxIntegral, integral));

    float out = kp * error + ki * integral + kd * derivative;
    const float maxOutput = static_cast<float>(maxOutput_);
    outSpeed = std::max(-maxOutput, std::min(maxOutput, out));
}

// 闭环主入口：读取编码器、计算 PID、做斜坡限制，再把结果下发给电机。
void MotorController::updateClosedLoop(double dt)
{
    float actualLeft  = getEncoderSpeedLeft();
    float actualRight = getEncoderSpeedRight();

    float targetOutputLeft = 0.0f;
    float targetOutputRight = 0.0f;
    updateOnePID(targetLeft_,  actualLeft,  dt, kpL_, kiL_, kdL_, integralLeft_,  lastErrorLeft_,  targetOutputLeft);
    updateOnePID(targetRight_, actualRight, dt, kpR_, kiR_, kdR_, integralRight_, lastErrorRight_, targetOutputRight);

    currentOutputLeft_ = rampOutputToward(targetOutputLeft, currentOutputLeft_);
    currentOutputRight_ = rampOutputToward(targetOutputRight, currentOutputRight_);
    applyRawSpeed(currentOutputLeft_, currentOutputRight_);
}
