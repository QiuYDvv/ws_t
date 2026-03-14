#include "LQ_ls_i2c_dri.h"
#include "LQ_ls_i2c_gyro.h"

#define DEVICE_CNT      1                   // 设备数量
#define DEVICE_NAME     "ls_iic"            // 设备文件名称
#define DEVICE_ID_NAME  "loongson,IIC_Eqm"  // 设备匹配名称

uint8_t I2C_MPU6050  = 0;   // 可用
uint8_t I2C_errorid  = 0;

/************************************ 上层接口的函数声明 ************************************/
// 对应上层 open 函数
static int i2c_open(struct inode *inode, struct file *f);
// 对应上层 read 函数
static ssize_t i2c_read(struct file *f, char __user *buf, size_t cnt, loff_t *off);
// 对应上层 write 函数
static ssize_t i2c_write(struct file *f, const char __user *buf, size_t cnt, loff_t *off);
// 对应上层 close 函数
static int i2c_release(struct inode *inode, struct file *f);
// 对应上层 ioctl 函数
static long i2c_ioctl(struct file *f, unsigned int cmd, unsigned long arg);

/********************************** 设备驱动的相关函数声明 **********************************/
// I2C 设备驱动的探测函数
static int i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
// I2C设备驱动的移除函数
static int i2c_remove(struct i2c_client *c);

/*************************************** 全局变量定义 **************************************/
// 创建一个全局的 iic 客户端设备结构体
static struct i2c_client *main_client;
// 存放当前陀螺仪的ID值
uint8_t Gyro_ID = 0;

/***************************** 文件操作指针集 -- 提供给上层接口 *****************************/
static const struct file_operations i2c_ops = {
    .owner = THIS_MODULE,       // 该文件操作集合属于当前的内核模块
    .open = i2c_open,           // 指向一个处理文件打开操作的函数
    .read = i2c_read,           // 指向一个处理文件读取操作的函数
    .write = i2c_write,         // 指向一个处理文件写入操作的函数
    .unlocked_ioctl = i2c_ioctl,// 指向一个处理文件自定义操作的函数
    .release = i2c_release,     // 指向一个处理文件关闭操作的函数
};

/********************************** 设备与驱动的匹配结构体 **********************************/
static const struct i2c_device_id i2c_dev_id[] = {
    { DEVICE_ID_NAME, 0 },
    {  }
};

/********************************* I2C设备驱动的核心结构体 **********************************/
static struct i2c_driver i2c_drv = {
    .driver = {
        .name = "i2c_gyro_drv", // 驱动的名称
        .owner = THIS_MODULE,   // 代表当前内核模块
    },
    .probe = i2c_probe,     // 探测函数
    .remove = i2c_remove,   // 移除函数
    .id_table = i2c_dev_id  // 数组，包含了iic设备的ID信息
};

/************************************* I2C读写相关函数 *************************************/
/*!
 * @brief   从寄存器读取数据
 * @param   dev : 自定义 I2C 相关结构体 
 * @param   reg : 寄存器地址
 * @param   val : 读取数据缓冲区
 * @param   len : 缓冲区长度
 * @return  成功返回 0，失败返回错误码
 * @date    2025/3/20
 */
int i2c_read_regs(struct ls_i2c_dev *dev, u8 reg, void *val, int len)
{
    int ret;
    // 用于描述 I2C 消息的基本数据结构
    struct i2c_msg msg[2];
    // 表示一个连接到 I2C 总线上的客户端设备
    struct i2c_client *cli = (struct i2c_client*)dev->client;
    // 配置第一个消息(写消息)
    msg[0].addr = cli->addr;// cli 指向的 I2C 设备的地址
    msg[0].flags = 0;       // 消息标志位，表示一个写操作
    msg[0].buf = &reg;      // 要读取的寄存器地址
    msg[0].len = 1;         // 消息的数据长度 1 字节，因为只需要发送一个寄存器地址
    // 配置第二个消息(读消息)
    msg[1].addr = cli->addr;// cli 指向的 I2C 设备的地址
    msg[1].flags = I2C_M_RD;// 消息标志位，表示一个读操作
    msg[1].buf = val;       // 消息缓冲区指向 val
    msg[1].len = len;       // 消息长度为 len，即要从寄存器读取的字节数
    // 在 I2C 总线上进行数据传输
    ret = i2c_transfer(cli->adapter, msg, 2);
    if (ret == 2)
        ret = 0;
    else
    {
        printk("i2c read error. ret = %d, reg = %06x, len = %d\r\n", ret, reg, len);
        ret = -EREMOTEIO;
    }
    return ret;
}

/*!
 * @brief   使用 I2C 向寄存器读取一个字节
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @return  返回获取到的数据
 * @date    2025/3/20
 */
u8 i2c_read_reg_byte(struct ls_i2c_dev *dev, u8 reg)
{
    // 定义一个变量存储读取的数据
    u8 data = 0;
    // 调用上面的读取函数，将自定义结构体、要读取的寄存器地址、存储缓冲区、缓冲区长度传入函数中
    i2c_read_regs(dev, reg, &data, 1);
    // 返回数据
    return data;
}

/*!
 * @brief   向寄存器写数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @param   buf : 写入数据缓冲区
 * @param   len : 缓冲区长度
 * @return  返回 i2c_transfer 函数返回值
 * @date    2025/3/20
 */
s32 i2c_write_regs(struct ls_i2c_dev *dev, u8 reg, u8 *buf, u8 len)
{
    // 存储要发送的完整数据，包括寄存器地址和要写入的数据
    u8 b[256];
    // 用于描述 I2C 消息的基本数据结构
    struct i2c_msg msg;
    // 表示一个连接到 I2C 总线上的客户设备
    struct i2c_client *cli = (struct i2c_client*)dev->client;

    b[0] = reg;             // 将寄存器地址 reg 存储在数组 b 的第一个元素中
    memcpy(&b[1], buf, len);// 将 buf 中长度为 len 的数据存储到 b 数组中，从第二个元素开始存储

    msg.addr = cli->addr;   // cli 指向的 I2C 设备的地址
    msg.flags = 0;          // 消息标志位设备为 0，表示一个写操作

    msg.buf = b;            // 将消息的数据缓冲区指针指向数组 b，即要发送的完整数据
    msg.len = len + 1;      // 设置消息的数据长度为 len + 1，因为除了要写入的数据len字节，好包含一个字节的寄存器地址

    // 执行 I2C 数据传输的函数
    return i2c_transfer(cli->adapter, &msg, 1);
}

/*!
 * @brief   向寄存器写入一个字节数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @param   buf : 写入的数据
 * @return  返回 i2c_transfer 函数返回值
 * @date    2025/3/20
 */
s32 i2c_write_reg(struct ls_i2c_dev *dev, u8 reg, u8 buf)
{
    return i2c_write_regs(dev, reg, &buf, 1);
}

/************************************* 陀螺仪相关函数 **************************************/
/*!
 * @brief   读取陀螺仪的设备ID
 * @param   dev : 自定义 I2C 相关结构体
 * @return  陀螺仪的设备ID
 * @date    2025/3/20
 */
uint8_t Obt_Gyro_Dev_ID(struct ls_i2c_dev *dev)
{
    Gyro_ID = i2c_read_reg_byte(dev, WHO_AM_I); //获取陀螺仪设备 ID
    switch (Gyro_ID)
    {
        // case 0x12:IIC_ICM20602 = 1;break;
        // case 0x71:IIC_MPU9250  = 1;break;
        // case 0x98:IIC_ICM20689 = 1;break;
        // case 0x42:IIC_ICM42605 = 1;break;
        case 0x68:I2C_MPU6050  = 1;break;
        default:  I2C_errorid  = 1;//return 0;
    }
    return Gyro_ID;
}

/*!
 * @brief   设置陀螺仪测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 0 --> ±250dps    1 --> ±500dps
 *                2 --> ±1000dps   3 --> ±2000dps
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 */
uint8_t MPU_Set_Gyro_Fsr(struct ls_i2c_dev *dev, uint8_t fsr)
{
    return i2c_write_reg(dev, MPU_GYRO_CFG_REG, fsr << 3);
}

/*!
 * @brief   设置加速度计测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 0 --> ±2g    1 --> ±4g
 *                2 --> ±8g    3 --> ±16g
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 */
uint8_t MPU_Set_Accel_Fsr(struct ls_i2c_dev *dev, uint8_t fsr)
{
    return i2c_write_reg(dev, MPU_ACCEL_CFG_REG, fsr << 3);
}

/*!
 * @brief   设置数字低通滤波
 * @param   dev : 自定义 I2C 相关结构体
 * @param   lpf : 数字低通滤波频率(Hz)
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 */
uint8_t MPU_Set_LPF(struct ls_i2c_dev *dev, uint16_t lpf)
{
    uint8_t dat = 0;
    if (lpf >= 188)
        dat = 1;
    else if (lpf >= 98)
        dat = 2;
    else if (lpf >= 42)
        dat = 3;
    else if (lpf >= 20)
        dat = 4;
    else if (lpf >= 10)
        dat = 5;
    else
        dat = 6;
    return i2c_write_reg(dev, MPU_CFG_REG, dat); // 设置数字低通滤波器
}

/*!
 * @brief   设置采样率
 * @param   dev : 自定义 I2C 相关结构体
 * @param   rate: 4 ~ 1000(Hz)
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 */
uint8_t MPU_Set_Rate(struct ls_i2c_dev *dev, uint16_t rate)
{
    uint8_t dat;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    dat = 1000 / rate - 1;
    i2c_write_reg(dev, MPU_SAMPLE_RATE_REG, dat); // 设置数字低通滤波器
    return MPU_Set_LPF(dev, rate / 2);            // 自动设置LPF为采样率的一半
}

/*!
 * @brief    获取温度值
 * @param    dev : 自定义 I2C 相关结构体
 * @return   温度值(扩大了100倍)
 * @date     2019/6/12
 */
int16_t MPU_Get_Temperature(struct ls_i2c_dev *dev)
{
    uint8_t buf[2];
    int16_t raw;
    i2c_read_regs(dev, MPU_TEMP_OUTH_REG, buf, 2);
    raw = (((uint16_t)buf[0] << 8) | buf[1]) * 100;
    return (2100 + raw / 33387);
}

/*!
 * @brief   获取陀螺仪值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   gx,gy,gz: 陀螺仪 x,y,z 轴的原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2025/3/20
 */
uint8_t MPU_Get_Gyroscope(struct ls_i2c_dev *dev, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6], res;
    res = i2c_read_regs(dev, MPU_GYRO_XOUTH_REG, buf, 6);
    if (res == 0)
    {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/*!
 * @brief   获取加速度值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   ax,ay,az: 陀螺仪 x,y,z 轴的原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2025/3/20
 */
uint8_t MPU_Get_Accelerometer(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6], res;
    res = i2c_read_regs(dev, MPU_ACCEL_XOUTH_REG, buf, 6);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/*!
 * @brief   获取 加速度值 角速度值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   ax,ay,az: 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
 * @param   gx,gy,gz: 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2025/3/20
 */
uint8_t MPU_Get_Raw_data(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14], res;
    res = i2c_read_regs(dev, MPU_ACCEL_XOUTH_REG, buf, 14);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
        *gx = ((uint16_t)buf[8] << 8) | buf[9];
        *gy = ((uint16_t)buf[10] << 8) | buf[11];
        *gz = ((uint16_t)buf[12] << 8) | buf[13];
    }
    return res;
}

/*!
 * @brief   初始化陀螺仪
 * @param   dev : 自定义 I2C 相关结构体
 * @return  成功返回 0，失败返回 1
 * @date    2025/3/20
 */
uint8_t I2C_Gyro_init(struct ls_i2c_dev *dev)
{
    u8 res;
    res = dev->client->addr; // 读取 MPU6050 的 ID
    if (res != Gyro_ID)      // 器件 ID 正确
    {
        printk("Gyro_ID not MPU6050 0x%x\r\n", res);
        return 1;
    }
    printk("Gyro_ID yes MPU6050\r\n");
    res = 0;
    res += i2c_write_reg(dev, MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    Delay_Ms(100);                                      // 延时100ms
    res += i2c_write_reg(dev, MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    res += MPU_Set_Gyro_Fsr(dev, 3);                    // 陀螺仪传感器,±2000dps
    res += MPU_Set_Accel_Fsr(dev, 1);                   // 加速度传感器,±4g
    res += MPU_Set_Rate(dev, 1000);                     // 设置采样率1000Hz
    res += i2c_write_reg(dev, MPU_CFG_REG, 0x02);       // 设置数字低通滤波器   98hz
    res += i2c_write_reg(dev, MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    res += i2c_write_reg(dev, MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    res += i2c_write_reg(dev, MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
    res += i2c_write_reg(dev, MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
    return 0;
}

/************************************ 上层接口的函数实现 ************************************/
/*!
 * @brief   上层 open 函数相关实现
 * @param   inode: 指向 inode 结构体的指针，包括文件系统中文件或目录的元数据结构
 * @param   f    : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @return  成功返回 0，失败返回错误码
 * @date    2025/3/20
 */
static int i2c_open(struct inode *inode, struct file *f)
{
    // 从 file 结构体中获取对应的字符设备结构体 cdev，cdev 结构体代表了字符设备在内核中的抽象表现
    struct cdev *cdev = f->f_path.dentry->d_inode->i_cdev;
    // 使用 container_of 宏根据 cdev 结构体的地址找到自定义结构体的地址
    struct ls_i2c_dev *dev = container_of(cdev, struct ls_i2c_dev, cdev);
    // 获取陀螺仪 ID
    Obt_Gyro_Dev_ID(dev);
    printk("Gyro_ID = 0x%x, yuanshi = 0x%x\r\n", Gyro_ID, dev->client->addr);
    // 初始化陀螺仪
    if (I2C_Gyro_init(dev) != 0)
        return -1;
    return 0;
}

/*!
 * @brief   上层 read 函数相关实现
 * @param   f   : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @param   buf : 指向用户空间缓冲区的指针
 * @param   cnt : 表示用户期望从 I2C 设备读取的字节数
 * @param   off : 指向文件偏移量的指针
 * @return  大于零表示成功复制到用户空间的字节数；等于零表示已到文件末尾；失败返回错误码
 * @date    2025/3/20
 */
static ssize_t i2c_read(struct file *f, char __user *buf, size_t cnt, loff_t *off)
{
    int16_t data[6];
    ssize_t err = 0;
    // 从 file 结构体中获取对应的字符设备结构体 cdev，cdev 结构体代表了字符设备在内核中的抽象表现
    struct cdev *cdev = f->f_path.dentry->d_inode->i_cdev;
    // 使用 container_of 宏根据 cdev 结构体的地址找到自定义结构体的地址
    struct ls_i2c_dev *dev = container_of(cdev, struct ls_i2c_dev, cdev);
    // 从陀螺仪获取数据
    MPU_Get_Raw_data(dev, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
    // 将数据从内核层发送到用户层
    err = copy_to_user(buf, data, sizeof(data));
    return err;
}

/*!
 * @brief   上层 write 函数相关实现
 * @param   f   : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @param   buf : 指向用户空间缓冲区的指针
 * @param   cnt : 表示用户期望从 I2C 设备写入的字节数
 * @param   off : 指向文件偏移量的指针
 * @return  大于零表示成功写入到用户空间的字节数；失败返回错误码
 * @date    2025/3/20
 */
static ssize_t i2c_write(struct file *f, const char __user *buf, size_t cnt, loff_t *off)
{
    return 0;
}

/*!
 * @brief   上层 close 函数相关实现
 * @param   inode: 指向 inode 结构体的指针，包括文件系统中文件或目录的元数据结构
 * @param   f    : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @return  0 表示释放成功，负数表示释放操作失败
 * @date    2025/3/20
 */
static int i2c_release(struct inode *inode, struct file *f)
{
    return 0;
}

/*!
 * @brief   上层 ioctl 函数相关实现
 * @param   f   : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @param   cmd : 控制命令码，用于指定要执行的操作类型
 * @param   arg : 与命令码相关的参数，可传递额外的数据
 * @return  0 表示操作成功，负数表示操作失败
 * @date    2025/3/20
 */
static long i2c_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    uint8_t ID = 0;
    int16_t data[3] = {0};
    // 从 file 结构体中获取对应的字符设备结构体 cdev，cdev 结构体代表了字符设备在内核中的抽象表现
    struct cdev *cdev = f->f_path.dentry->d_inode->i_cdev;
    // 使用 container_of 宏根据 cdev 结构体的地址找到自定义结构体的地址
    struct ls_i2c_dev *dev = container_of(cdev, struct ls_i2c_dev, cdev);
    switch (cmd)
    {
        case I2C_GET_GYRO_ID:   // 获取ID
            ID = Obt_Gyro_Dev_ID(dev);
            if (copy_to_user((uint8_t*)arg, &ID, sizeof(ID)))
                return -EFAULT;
            break;
        case I2C_GET_GYRO_TEM:  // 获取温度
            data[0] = MPU_Get_Temperature(dev);
            if (copy_to_user((int16_t*)arg, &data[0], sizeof(int16_t)))
                return -EFAULT;
            break;
        case I2C_GET_GYRO_ANG:  // 获取角速度值
            MPU_Get_Gyroscope(dev, &data[0], &data[1], &data[2]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data)))
                return -EFAULT;
            break;
        case I2C_GET_GYRO_ACC:  // 获取加速度值
            MPU_Get_Accelerometer(dev, &data[0], &data[1], &data[2]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data)))
                return -EFAULT;
            break;
        default:
            return -EFAULT;
    }
    return 0;
}

/*!
 * @brief   内核毫秒级延时函数
 * @param   ms : 毫秒值
 * @return  无
 * @date    2025/3/20
 */
void Delay_Ms(uint16_t ms)
{
    mdelay(ms);
}

/*********************************** 设备驱动相关核心函数 ***********************************/
/*!
 * @brief   设备驱动的探测函数
 * @param   client: 指向 I2C 客户端结构体，包含设备的地址、适配器等信息
 * @param   id    : 指向 I2C 设备 ID 结构体，包含设备的标识符，用于判断设备是否与驱动程序匹配
 * @return  0 表示设备探测和初始化成功，负数表示操作失败
 * @date    2025/3/20
 */
static int i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;                  // 存储各种操作的返回值
    struct ls_i2c_dev *ls_i2c;// 创建一个自定义结构体
    main_client = client;     // 将传入的client参数保存到全局变量main_client中
    // devm_kzalloc 是一个内存分配函数，会给 client->dev 分配内存，并初始化为 0
    ls_i2c = devm_kzalloc(&client->dev, sizeof(*ls_i2c), GFP_KERNEL);
    if (!ls_i2c)
        return -ENOMEM;
    // 获取字符设备的设备编号
    ret = alloc_chrdev_region(&ls_i2c->dev_id, 0, DEVICE_CNT, DEVICE_NAME);
    if (ret < 0)
    {
        pr_err("%s Couldn't alloc_chrdev_regin, ret = %d\r\n", DEVICE_NAME, ret);
        return -ENOMEM;
    }
    // 将字符设备的所有者设备为当前模块
    ls_i2c->cdev.owner = THIS_MODULE;
    // 初始化字符设备结构体
    cdev_init(&ls_i2c->cdev, &i2c_ops);
    // 将字符设备添加到内核的字符设备管理系统中
    ret = cdev_add(&ls_i2c->cdev, ls_i2c->dev_id, DEVICE_CNT);
    if (ret < 0)
        goto del_unregister;
    // 创建一个设备类，设备类用于在/sys/class目录下创建对应的目录，方便用户空间对设备进行管理
    ls_i2c->class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(ls_i2c->class))
        goto del_cdev;
    // 用于在 /dev 目录下创建一个设备节点，用户空间程序可以通过该节点与设备进行交互
    ls_i2c->device = device_create(ls_i2c->class, NULL, ls_i2c->dev_id, NULL, DEVICE_NAME);
    if (IS_ERR(ls_i2c->device))
        goto destroy_class;
    // 将 I2C 客户端指针保存到自定义结构体中
    ls_i2c->client = client;
    // 用于将自定义结构体与 I2C 客户端关联起来，方便在其他函数中通过 i2c_get_clientdata 获取设备的私有数据
    i2c_set_clientdata(client, ls_i2c);
    printk("LQ_ls_i2c_dri module probe function\r\n");
    return 0;
destroy_class:
    // 注销之前创建的设备节点
    device_destroy(ls_i2c->class, ls_i2c->dev_id);
del_cdev:
    // 从内核的字符设备管理系统中删除指定的字符设备
    cdev_del(&ls_i2c->cdev);
del_unregister:
    // 用于注销之前分配的字符设备编号
    unregister_chrdev_region(ls_i2c->dev_id, DEVICE_CNT);
    return -EIO;
}

/*!
 * @brief   设备驱动的移除函数
 * @param   c : 指向 I2C 客户端结构体，包含设备的地址、适配器等信息
 * @return  0 表示设备移除操作成功，负数表示操作失败
 * @date    2025/3/20
 */
static int i2c_remove(struct i2c_client *c)
{
    // 获取与 i2c_client 结构体关联的设备私有数据
    struct ls_i2c_dev *ls_i2c = i2c_get_clientdata(c);
    // 从内核的字符设备管理系统中删除指定的字符设备
    cdev_del(&ls_i2c->cdev);
    // 用于注销之前分配的字符设备编号
    unregister_chrdev_region(ls_i2c->dev_id, DEVICE_CNT);
    // 注销之前创建的设备节点
    device_destroy(ls_i2c->class, ls_i2c->dev_id);
    // 用于销毁之前创建的设备类
    class_destroy(ls_i2c->class);
    printk("LQ_ls_i2c_dri module remove function\r\n");
    return 0;
}

/*!
 * @brief   设备驱动的加载函数
 * @date    2025/3/20
 */
static int __init i2c_drv_init(void)
{
    // IIC 设备驱动注册入口函数
    i2c_add_driver(&i2c_drv);
    return 0;
}

/*!
 * @brief   设备驱动的卸载函数
 * @date    2025/3/20
 */
static void __exit i2c_drv_exit(void)
{
    // IIC 设备驱动注销入口函数
    i2c_del_driver(&i2c_drv);
}

/************************************ 内核模块相关宏函数 ************************************/
module_init(i2c_drv_init);                  // 指定加载函数
module_exit(i2c_drv_exit);                  // 指定卸载函数
MODULE_AUTHOR("LQ_012 <chiusir@163.com>");  // 作者以及邮箱
MODULE_DESCRIPTION("一个陀螺仪的驱动端模块"); // 模块简单介绍
MODULE_VERSION("1.0");                      // 版本号
MODULE_LICENSE("GPL");                      // 许可证声明
