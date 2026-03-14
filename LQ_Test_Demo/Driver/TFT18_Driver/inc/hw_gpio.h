#ifndef __HW_GPIO_H__
#define __HW_GPIO_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#define LS_GPIO_BASE_ADDR  0x16104000   // GPIO 基地址
#define LS_GPIO_REUSE_ADDR 0x16000490   // GPIO复用配置寄存器基地址

#define LS_GPIO_REUSE_OFS  0x04         // 复用配置寄存器偏移量

#define LS_GPIO_OEN_OFFSET(n) (0x00 + (n) / 8 * 0x01)  // 方向寄存器：0 --> 输出；1 --> 输入
#define LS_GPIO_O_OFFSET(n)   (0x10 + (n) / 8 * 0x01)  // 输出寄存器
#define LS_GPIO_I_OFFSET(n)   (0x20 + (n) / 8 * 0x01)  // 输入寄存器

#define GPIO_Mode_In  1 // 配置为输入模式
#define GPIO_Mode_Out 0 // 配置为输出模式

#define PAGESIZE 0x1000 // 映射地址时的页大小

// 修改寄存器配置 GPIO
struct HardwareGPIO {
    void __iomem *gpio_base;    // 基础寄存器
    void __iomem *gpio_reuse;   // 复用寄存器
    uint8_t gpio;               // GPIO引脚号
};

/*!
 * @brief   初始化 GPIO
 * @param   gpio : GPIO 的引脚号
 * @param   Gpio : GPIO 的自定义类
 * @return  失败返回错误码，成功返回 0
 */
int my_gpio_init(uint8_t gpio, struct HardwareGPIO *Gpio);

/*!
 * @brief   设置 GPIO 模式
 * @param   mode : GPIO 的模式。GPIO_Mode_In 为输入模式，GPIO_Mode_Out 为输出模式
 * @param   Gpio : GPIO 的自定义类
 * @return  失败返回 -1，成功返回 0
 */
int my_gpio_mode(uint8_t mode, struct HardwareGPIO *Gpio);

/*!
 * @brief   设置 GPIO 高低电平
 * @param   val  : GPIO 的电平值
 * @param   Gpio : GPIO 的自定义类
 * @return  失败返回 -1，成功返回设置的电平值
 */
int my_gpio_set_value(uint8_t val, struct HardwareGPIO *Gpio);

/*!
 * @brief   获取 GPIO 高低电平
 * @param   Gpio : GPIO 的自定义类
 * @return  返回获取的电平值
 */
int my_gpio_get_value(struct HardwareGPIO *Gpio);

/*!
 * @brief   清理 GPIO
 * @param   GPIO : GPIO 的自定义类
 * @return  无
 */
void my_gpio_release(struct HardwareGPIO *Gpio);

#endif
