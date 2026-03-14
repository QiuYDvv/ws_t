/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-02-26
@修改内容：
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "main.hpp"
#include "LQ_demo.hpp"

// 蜂鸣器引脚初始化
// HWGpio beep(61, GPIO_Mode_Out);

int main()
{
    // beep.SetGpioValue(0);   // 关闭蜂鸣器

    // GpioOutputDemo1(88);    // 测试GPIO输出功能(设备文件)
    // GpioOutputDemo2(88);    // 测试GPIO输出功能(寄存器)
    // GpioInputDemo1();       // 测试GPIO输入功能(设备文件)
    // GpioInputDemo2();       // 测试GPIO输入功能(寄存器)
    // PwmDevDemo();           // 测试 PWM 功能(设备文件)
    // PwmHWDemo();            // 测试 PWM 功能(寄存器)
    // GtimPwmDemo();          // 测试 Gtim 的 PWM 功能(寄存器)
    // AtimPwmDemo();
    // EncoderDemo();          // 测试编码器功能(寄存器)
    // CameraDemo();           // 测试摄像头功能
    // AdcFunDemo();           // 测试 ADC 功能
    // TFTDemo();              // 测试 TFT 屏幕功能
    // TFT_Dri_Demo();         // 测试 TFT 屏幕驱动功能
    // Demo_IIC_Gyro();        // 测试陀螺仪功能
    // Demo_I2C_Gyro();        // 测试陀螺仪驱动功能 
    // Uart_Demo();            // 测试串口功能
    // TCP_Demo();             // 测试 TCP 程序
    // UDP_Demo();             // 测试 UDP 程序
    // GetTimeDemo();          // 测试时间戳打印功能
    // sleepDemo();            // 测试 sleep() 函数 ------------ 以秒为单位延时
    // usleepDemo();           // 测试 usleep() 函数 ----------- 以微秒为单位延时
    // nanosleepDemo();        // 测试 nanosleep() 函数 -------- 以纳秒为单位延时
    // clock_nanosleepDemo();  // 测试 clock_nanosleep() 函数 -- 以纳秒为单位延时
    // MotorDemo();            // 测试电机功能
    // ServoDemo();            // 测试舵机功能
    // GpioDemo();             // 测试久久派22个GPIO翻转功能

    // setitimerDemo();        // 信号模拟5ms中断
    // HibernateDemo();        // 线程模拟5ms中断

    while(1)
    {
    }
    
    return 0;
}
