#pragma once

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <linux/interrupt.h>

enum
{
    I2C_GET_GYRO_ID,    // 获取陀螺仪 ID
    I2C_GET_GYRO_TEM,   // 获取温度值
    I2C_GET_GYRO_NONE,
    I2C_GET_GYRO_ANG,   // 获取角速度值
    I2C_GET_GYRO_ACC    // 获取加速度值
};

// 自定义结构体，存储了一些 iic 设备所需要的成员变量
struct ls_i2c_dev {
    struct i2c_client *client;  // 表示连接到 iic 总线上的客户端设备，包含了设备的各种信息和操作方式
    struct cdev        cdev;    // 表示字符设备的核心结构体，包含字符设备的各种属性和操作方法
    struct class      *class;   // 表示设备类的核心结构体
    struct device     *device;  // 表示系统中的一个具体设备
    dev_t              dev_id;  // 表示该设备的设备号-
};

/************************************* I2C读写相关函数 *************************************/
/*!
 * @brief   从寄存器读取数据
 * @param   dev : 自定义 I2C 相关结构体 
 * @param   reg : 寄存器地址
 * @param   val : 读取数据缓冲区
 * @param   len : 缓冲区长度
 * @return  成功返回 0，失败返回错误码
 */
int i2c_read_regs(struct ls_i2c_dev *dev, u8 reg, void *val, int len);

/*!
 * @brief   使用 I2C 向寄存器读取一个字节
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @return  返回获取到的数据
 */
u8 i2c_read_reg_byte(struct ls_i2c_dev *dev, u8 reg);

/*!
 * @brief   向寄存器写数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @param   buf : 写入数据缓冲区
 * @param   len : 缓冲区长度
 * @return  返回 i2c_transfer 函数返回值
 */
s32 i2c_write_regs(struct ls_i2c_dev *dev, u8 reg, u8 *buf, u8 len);

/*!
 * @brief   向寄存器写入一个字节数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @param   buf : 写入的数据
 * @return  返回 i2c_transfer 函数返回值
 */
s32 i2c_write_reg(struct ls_i2c_dev *dev, u8 reg, u8 buf);

/************************************* 陀螺仪相关函数 **************************************/
/*!
 * @brief   读取陀螺仪的设备ID
 * @param   dev : 自定义 I2C 相关结构体
 * @return  陀螺仪的设备ID
 */
uint8_t Obt_Gyro_Dev_ID(struct ls_i2c_dev *dev);

/*!
 * @brief   设置陀螺仪测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 0 --> ±250dps    1 --> ±500dps
 *                2 --> ±1000dps   3 --> ±2000dps
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 */
uint8_t MPU_Set_Gyro_Fsr(struct ls_i2c_dev *dev, uint8_t fsr);

/*!
 * @brief   设置加速度计测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 0 --> ±2g    1 --> ±4g
 *                2 --> ±8g    3 --> ±16g
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 */
uint8_t MPU_Set_Accel_Fsr(struct ls_i2c_dev *dev, uint8_t fsr);

/*!
 * @brief   设置数字低通滤波
 * @param   dev : 自定义 I2C 相关结构体
 * @param   lpf : 数字低通滤波频率(Hz)
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 */
uint8_t MPU_Set_LPF(struct ls_i2c_dev *dev, uint16_t lpf);

/*!
 * @brief   设置采样率
 * @param   dev : 自定义 I2C 相关结构体
 * @param   rate: 4 ~ 1000(Hz)
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2019/6/12
 */
uint8_t MPU_Set_Rate(struct ls_i2c_dev *dev, uint16_t rate);

/*!
 * @brief    获取温度值
 * @param    dev : 自定义 I2C 相关结构体
 * @return   温度值(扩大了100倍)
 * @date     2019/6/12
 */
int16_t MPU_Get_Temperature(struct ls_i2c_dev *dev);

/*!
 * @brief   获取陀螺仪值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   gx,gy,gz: 陀螺仪 x,y,z 轴的原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 */
uint8_t MPU_Get_Gyroscope(struct ls_i2c_dev *dev, int16_t *gx, int16_t *gy, int16_t *gz);

/*!
 * @brief   获取加速度值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   ax,ay,az: 陀螺仪 x,y,z 轴的原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2019/6/12
 */
uint8_t MPU_Get_Accelerometer(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az);

/*!
 * @brief   获取 加速度值 角速度值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   ax,ay,az: 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
 * @param   gx,gy,gz: 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2019/6/12
 */
uint8_t MPU_Get_Raw_data(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

/*!
 * @brief   内核毫秒级延时函数
 * @param   ms : 毫秒值
 * @return  无
 * @date    2025/3/20
 */
void Delay_Ms(uint16_t ms);
