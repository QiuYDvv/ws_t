#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>

// 该结构体用于描述连接到 I2C 总线上的设备信息
static struct i2c_board_info    IIC_Equipment;
// 用于表示一个连接到 I2C 总线上的客户端设备，包含了设备的各种信息和操作方式+++
static struct i2c_client *      client;

// 设备名称
#define DEV_ID_NAME "loongson,IIC_Eqm"

// 当需要注册多个 I2C 设备时，可以在此添加
static const unsigned short All_Addr[] = {0x68, 0x71, I2C_CLIENT_END};

/*!
 * @brief   加载函数
 */
static int __init dev_init(void)
{
    // 创建 I2C 总线适配器
    struct i2c_adapter *adapter = NULL;
    // 将 I2C 总线上的设备信息结构体清空
    memset(&IIC_Equipment, 0, sizeof(struct i2c_board_info));
    // 复制设备名称
    strlcpy(IIC_Equipment.type, DEV_ID_NAME, I2C_NAME_SIZE);
    // 根据指定的适配器编号，从内核中获取对应的 I2C 总线适配器
    adapter = i2c_get_adapter(1);
    // 用于在 I2C 总线上探测并注册新设备的函数
    client = i2c_new_probed_device(adapter, &IIC_Equipment, All_Addr, NULL);
    // 检测 I2C 设备是否存在
    i2c_put_adapter(adapter);
    printk("LQ_ls_i2c_dev init function\r\n");

    if (client)
        return 0;
    else
        return -ENODEV;
}

/*!
 * @brief   卸载函数
 */
static void __exit dev_exit(void)
{
    // 从 Linux 内核的 I2C 子系统中注销一个已注册的 I2C 设备
    i2c_unregister_device(client);
    printk("LQ_ls_i2c_dev exit function\r\n");
}

module_init(dev_init);                      // 指定加载函数
module_exit(dev_exit);                      // 指定卸载函数
MODULE_AUTHOR("LQ_012 <chiusir@163.com>");  // 作者以及邮箱
MODULE_DESCRIPTION("一个陀螺仪的设备端模块"); // 模块简单介绍
MODULE_VERSION("1.0");                      // 版本号
MODULE_LICENSE("GPL");                      // 许可证声明
