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

@修改日期：2025-03-16
@修改内容：
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#pragma once

// 工程公共平台头：
// 集中包含当前主程序会直接依赖的标准库、系统头、OpenCV 与龙邱库头文件。
// 该文件偏“总入口”，适合平台初始化或兼容旧工程；新业务文件可按需只包含自己需要的头。

//////////////// C++标准库 //////////////////////////////
#include <iostream>
#include <string>

//////////////// C标准库 ////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>    // 共享内存
#include <sys/sem.h>    // 信号量集

//////////////// OpenCV头文件 ///////////////////////////
#include <opencv2/opencv.hpp>

//////////////// 龙邱库文件 /////////////////////////////
#include "LQ_PWM.hpp"
#include "LQ_GTIM_PWM.hpp"
#include "LQ_GPIO.hpp"
#include "LQ_PWM_ENCODER.hpp"
#include "LQ_TFT18.hpp"
#include "LQ_SOFT_I2C.hpp"
#include "LQ_HW_GPIO.hpp"
#include "LQ_HW_PWM.hpp"
#include "LQ_HW_SPI.hpp"
#include "LQ_SOFT_I2C_Gyro.hpp"
#include "LQ_Uart.hpp"
#include "LQ_MPU6050.hpp"
#include "LQ_TCP_Client.hpp"
#include "LQ_UDP_Client.hpp"
#include "LQ_TFT18_dri.hpp"

using namespace cv;
using namespace std;
