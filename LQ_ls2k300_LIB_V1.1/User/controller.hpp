/* 电机速度控制器：开环/闭环（编码器 + PID）
 * 基于龙邱 LQ_PWM、LQ_HW_GPIO、LQ_PWM_ENCODER，与 LQ_Motor_Demo / LQ_Encoder_Demo 引脚一致
 */
#pragma once

// 电机控制模块公共接口：
// 暴露 PWM/方向控制、编码器读取和闭环 PID 速度控制能力，供调试线程或后续业务控制层调用。

#include "LQ_PWM.hpp"
#include "LQ_HW_GPIO.hpp"
#include "LQ_PWM_ENCODER.hpp"
#include <cmath>

/** 电机速度控制器：左右轮 PWM + 方向 GPIO + 编码器 + PID 闭环 */
class MotorController {
public:
    /** 默认 PWM 周期 100kHz，与 LQ_Motor_Demo 一致 */
    static constexpr int DEFAULT_PWM_CYCLE = 100000;
    /** 默认最大占空比（0~DEFAULT_PWM_CYCLE），限制最大速度 */
    static constexpr int DEFAULT_MAX_DUTY = 50000;

    MotorController();
    ~MotorController() = default;

    /** 初始化 PWM、方向 GPIO、编码器，使能输出 */
    void init();

    /**
     * 设置左右轮速度（开环）
     * @param left  左轮速度，范围 [-100, 100]，正数前进、负数后退
     * @param right 右轮速度，范围 [-100, 100]
     */
    void setSpeed(int left, int right);

    /** 停止：左右轮速度置 0 */
    void stop();

    /** 清空 PID 积分/微分状态并将目标速度归零 */
    void resetPIDState();

    /** 停机：输出占空比清零并关闭 PWM 使能 */
    void shutdown();

    /** 设置最大占空比上限，用于限制最高速度（0 ~ PWM 周期） */
    void setMaxDuty(int maxDuty);

    /** 速度输出限幅：设定 [-maxOutput, maxOutput]，默认 100，可设为 80 等以保护电机 */
    void setMaxOutput(int maxOutput);

    /** 每个控制周期允许的最大输出变化量，限制电流冲击与突变反转 */
    void setRampStep(int rampStep);

    /** 是否已初始化 */
    bool isInited() const { return inited_; }

    // ---------- 编码器（参考 LQ_Encoder_Demo：左 0/73，右 3/72）----------
    /** 读取左轮编码器瞬时速度（带方向，与龙邱 Update() 同单位） */
    float getEncoderSpeedLeft();
    /** 读取右轮编码器瞬时速度（带方向） */
    float getEncoderSpeedRight();
    /** 读取左右轮编码器速度到 out_left、out_right */
    void getEncoderSpeed(float& out_left, float& out_right);
    /** 读取上一次闭环更新中缓存的速度反馈 */
    void getLastMeasuredSpeed(float& out_left, float& out_right) const;
    /** 读取当前真正输出到电机的百分比命令（含量化/最小启动占空比后的实际值） */
    void getAppliedOutput(float& out_left, float& out_right) const;

    // ---------- PID 闭环控速 ----------
    /** 设置 PID 参数（左右轮共用一组参数） */
    void setPID(float kp, float ki, float kd);
    /** 设置左右轮独立 PID 参数：左 (kpL,kiL,kdL)，右 (kpR,kiR,kdR) */
    void setPID(float kpL, float kiL, float kdL, float kpR, float kiR, float kdR);
    /** 仅设置左轮 PID */
    void setPIDLeft(float kp, float ki, float kd);
    /** 仅设置右轮 PID */
    void setPIDRight(float kp, float ki, float kd);
    /** 设置目标速度（与编码器速度同单位，用于闭环） */
    void setTargetSpeed(float target_left, float target_right);
    /**
     * 执行一步闭环：读编码器、算 PID、输出到电机
     * @param dt 距上次调用的时间间隔（秒）
     */
    void updateClosedLoop(double dt);

private:
    void applyRawSpeed(float left, float right);
    float rampOutputToward(float targetSpeed, float currentSpeed) const;
    float setOneMotor(SetPWM& pwm, HWGpio& dir, float speed, int& lastAppliedDuty);
    float readFilteredEncoderSpeed(LS_PwmEncoder& encoder, float sign, float& lastMeasured);
    void updateOnePID(float target, float actual, double dt,
                      float kp, float ki, float kd,
                      float& integral, float& lastError, float& outSpeed);

    SetPWM leftPWM_;
    SetPWM rightPWM_;
    HWGpio leftDir_;
    HWGpio rightDir_;
    LS_PwmEncoder leftEncoder_;   // channel 0, GPIO 73（与 LQ_Encoder_Demo 一致）
    LS_PwmEncoder rightEncoder_;  // channel 3, GPIO 72
    int maxDuty_;
    int minStartDuty_;  // 静止起步时的最小占空比，避免命令太小只抖不转
    int maxOutput_;  // 速度输出限幅 [-maxOutput_, maxOutput_]
    int rampStep_;
    bool inited_;
    float speedFilterAlpha_;

    float kpL_, kiL_, kdL_;
    float kpR_, kiR_, kdR_;
    float targetLeft_, targetRight_;
    float integralLeft_, integralRight_;
    float lastErrorLeft_, lastErrorRight_;
    float lastMeasuredLeft_, lastMeasuredRight_;
    float currentOutputLeft_, currentOutputRight_;
    float appliedOutputLeft_, appliedOutputRight_;
    int lastAppliedDutyLeft_, lastAppliedDutyRight_;
};
