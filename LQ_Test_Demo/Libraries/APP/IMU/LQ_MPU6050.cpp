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

@修改日期：2025-03-25
@修改内容：
@注意事项：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "LQ_MPU6050.hpp"

/*!
 * @brief   1.如果想要使用该文件中的MPU6050相关函数，需要先将对应模块加载到内核中
 * @brief   2.需要加载的模块：LQ_MPU6050_dev.ko 和 LQ_MPU6050_dri.ko
 * @brief   3.在加载这两个模块的时候，需要先将陀螺仪安装好，不然IIC总线检测不到设备，加载模块会失败
 * @brief   4.安装模块：insmod LQ_MPU6050_dev.ko
 * @brief   5.卸载模块：rmmod LQ_MPU6050_dev.ko
 * @brief   6.查看当前模块：lsmod
 */

/*!
 * @brief   I2C 驱动相关类无参构造函数
 * @param   无
 * @return  无
 * @date    2025/3/20
 */
LS_I2C_DEV::LS_I2C_DEV()
{

}

/*!
 * @brief   I2C 驱动相关类有参构造函数
 * @param   path : I2C 驱动生成的设备文件路径
 * @return  无
 * @date    2025/3/20
 */
LS_I2C_DEV::LS_I2C_DEV(const string &path)
{
    // 初始化
    this->I2C_Init(path);
}

/*!
 * @brief   初始化陀螺仪
 * @param   path : I2C 驱动生成的设备文件路径
 * @return  成功返回 0，失败返回 -1
 * @date    2025/3/20
 */
int8_t LS_I2C_DEV::I2C_Init(const string &path)
{
    this->I2C_fd = open(path.c_str(), O_RDWR);
    if (this->I2C_fd < 0)
    {
        printf("Open I2C failed %d\n", errno);
        return -1;
    }
    return 0;
}

/*!
 * @brief   获取陀螺仪 ID
 * @param   无
 * @return  返回陀螺仪 ID
 * @date    2025/3/20
 */
uint8_t LS_I2C_DEV::I2C_Get_ID(void)
{
    uint8_t id = 0;
    ioctl(this->I2C_fd, I2C_GET_GYRO_ID, &id);
    return id;
}

/*!
 * @brief   获取温度值
 * @param   无
 * @return  温度值
 * @date    2025/3/20
 */
int16_t LS_I2C_DEV::I2C_Get_Tem(void)
{
    int16_t tem = 0;
    ioctl(this->I2C_fd, I2C_GET_GYRO_TEM, &tem);
    return tem;
}

/*!
 * @brief   获取角速度值
 * @param   gx,gy,gz : 
 * @return
 * @date    2025/3/20
 */
int8_t LS_I2C_DEV::I2C_Get_Ang(int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[3] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_GYRO_ANG, data) != 0)
        return -1;
    *gx = data[0];
    *gy = data[1];
    *gz = data[2];
    return 0;
}

/*!
 * @brief   获取加速度值
 * @param   ax,ay,az : 
 * @return
 * @date    2025/3/20
 */
int8_t LS_I2C_DEV::I2C_Get_Acc(int16_t *ax, int16_t *ay, int16_t *az)
{
    int16_t data[3] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_GYRO_ACC, data) != 0)
        return -1;
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    return 0;
}

/*!
 * @brief   获取陀螺仪加速度值、角速度值
 * @param   ax,ay,az : 
 * @return
 * @date    2025/3/20
 */
int8_t LS_I2C_DEV::I2C_Get_RawData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[6] = {0};
    read(this->I2C_fd, data, 6 * sizeof(int16_t));
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    *gx = data[3];
    *gy = data[4];
    *gz = data[5];
    return 0;
}

/*!
 * @brief   I2C 驱动相关类析构函数
 * @param   无
 * @return  无
 * @date    2025/3/20
 */
LS_I2C_DEV::~LS_I2C_DEV()
{

}
