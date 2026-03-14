#include "LQ_demo.hpp"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void GpioInputDemo1()
 * @功能说明：GPIO 输入功能测试(设备文件)
 * @参数说明：无
 * @函数返回：无
 * @调用方法：GpioInputDemo1();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void GpioInputDemo1()
{
    SetGPIO Gpio1(44, "in");
    SetGPIO Gpio2(45, "in");
    while(1)
    {
        if(Gpio1.GetGpioValue() == 0){
            printf("Button 0 is pressed\n");
            usleep(500000);
        }
        if(Gpio2.GetGpioValue() == 0){
            printf("Button 1 is pressed\n");
            usleep(500000);
        }
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void GpioInputDemo2()
 * @功能说明：GPIO 输入功能测试(寄存器)
 * @参数说明：无
 * @函数返回：无
 * @调用方法：GpioInputDemo2();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void GpioInputDemo2()
{
    HWGpio Gpio1(44, GPIO_Mode_In);
    HWGpio Gpio2(45, GPIO_Mode_In);
    while(1)
    {
        if(Gpio1.GetGpioValue() == 0){
            printf("Button 0 is pressed\n");
            usleep(500000);
        }
        if(Gpio2.GetGpioValue() == 0){
            printf("Button 1 is pressed\n");
            usleep(500000);
        }
    }
}
