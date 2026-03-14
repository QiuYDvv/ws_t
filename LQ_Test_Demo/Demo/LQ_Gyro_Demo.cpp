#include "LQ_demo.hpp"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void Demo_IIC_Gyro(void)
 * @功能说明：测试陀螺仪模块
 * @参数说明：无
 * @函数返回：无
 * @调用方法：Demo_IIC_Gyro();
 * @备注说明：检测陀螺仪加速度计模块
 *           型号：6050 20602 20689 9250 42605（AD0接GND）自动识别
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Demo_IIC_Gyro()
{
    char txt[30];
    int16_t aacx, aacy, aacz;    // 加速度传感器原始数据
    int16_t gyrox, gyroy, gyroz; // 陀螺仪原始数据
    int16_t magx, magy, magz;    // 地磁原始数据
    TFTSPI_Init(1);  // 屏幕初始化
    cout << "TFTSPI_Init" << endl;
    TFTSPI_CLS(u16BLACK);
    cout << "TFTSPI_CLS" << endl;

    IIC_Init(); // IIC初始化
    usleep(100000);
    uint8_t res = Gyro_Chose(); // 判断陀螺仪型号
    sprintf(txt, "ID:0x%2x", res);
    TFTSPI_P8X16Str(5, 0, txt, u16WHITE, u16BLACK); // 字符串显示

    IIC_Gyro_init();    // 陀螺仪初始化
    
    while (1)
    {
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
        printf("Ang : %d, %d, %d\n", gyrox, gyroy, gyroz);
        MPU_Get_Raw_data(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz); // 得到加速度传感器数据
        if (IIC_MPU9250)
            MPU9250_Get_Magnetometer(&magx, &magy, &magz);  // 获取磁力计值

        sprintf((char *)txt, "ax:%06d", aacx);
        TFTSPI_P8X16Str(4, 1, txt, u16RED, u16BLUE); // 字符串显示
        sprintf((char *)txt, "ay:%06d", aacy);
        TFTSPI_P8X16Str(4, 2, txt, u16WHITE, u16BLACK); // 字符串显示
        sprintf((char *)txt, "az:%06d", aacz);
        TFTSPI_P8X16Str(4, 3, txt, u16RED, u16BLUE); // 字符串显示
        sprintf((char *)txt, "gx:%06d", gyrox);
        TFTSPI_P8X16Str(4, 4, txt, u16WHITE, u16BLACK); // 字符串显示
        sprintf((char *)txt, "gy:%06d", gyroy);
        TFTSPI_P8X16Str(4, 5, txt, u16RED, u16BLUE); // 字符串显示
        sprintf((char *)txt, "gz:%06d", gyroz);
        TFTSPI_P8X16Str(4, 6, txt, u16WHITE, u16BLACK); // 字符串显示

        if (IIC_MPU9250)
        {
            sprintf((char *)txt, "mx:%06d", magx);
            TFTSPI_P8X16Str(4, 7, txt, u16RED, u16BLUE); // 字符串显示
            sprintf((char *)txt, "my:%06d", magy);
            TFTSPI_P8X16Str(4, 8, txt, u16WHITE, u16BLACK); // 字符串显示
            sprintf((char *)txt, "mz:%06d", magz);
            TFTSPI_P8X16Str(4, 9, txt, u16RED, u16BLUE); // 字符串显示
        }

        Delay_Ms(30);
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：vvoid Demo_I2C_Gyro(void)
 * @功能说明：测试陀螺仪驱动模块
 * @参数说明：无
 * @函数返回：无
 * @调用方法：Demo_I2C_Gyro();
 * @备注说明：检测陀螺仪加速度计模块，暂时只支持MPU6050
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Demo_I2C_Gyro()
{
    char txt[30];
    TFTSPI_Init(1);  // 屏幕初始化
    cout << "TFTSPI_Init" << endl;
    TFTSPI_CLS(u16BLACK);
    cout << "TFTSPI_CLS" << endl;

    LS_I2C_DEV app(LS_IIC_PATH);    // 创建变量(初始化)

    usleep(100000);

    int16_t data[6] = {0};
    while (1)
    {
        printf("ID : 0x%x\n", app.I2C_Get_ID());
        printf("Tem : %d\n", app.I2C_Get_Tem());
        app.I2C_Get_Ang(&data[0], &data[1], &data[2]);
        app.I2C_Get_Acc(&data[3], &data[4], &data[5]);
        printf("Ang : %d, %d, %d\n", data[0], data[1], data[2]);
        printf("Acc : %d, %d, %d\n", data[3], data[4], data[5]);
        app.I2C_Get_RawData(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
        printf("data : %d, %d, %d, %d, %d, %d\n", data[0],data[1],data[2],data[3],data[4],data[5]);
        // sprintf((char *)txt, "ax:%06d", data[0]);
        // TFTSPI_P8X16Str(4, 1, txt, u16RED, u16BLUE); // 字符串显示
        // sprintf((char *)txt, "ay:%06d", data[1]);
        // TFTSPI_P8X16Str(4, 2, txt, u16WHITE, u16BLACK); // 字符串显示
        // sprintf((char *)txt, "az:%06d", data[2]);
        // TFTSPI_P8X16Str(4, 3, txt, u16RED, u16BLUE); // 字符串显示
        // sprintf((char *)txt, "gx:%06d", data[3]);
        // TFTSPI_P8X16Str(4, 4, txt, u16WHITE, u16BLACK); // 字符串显示
        // sprintf((char *)txt, "gy:%06d", data[4]);
        // TFTSPI_P8X16Str(4, 5, txt, u16RED, u16BLUE); // 字符串显示
        // sprintf((char *)txt, "gz:%06d", data[5]);
        // TFTSPI_P8X16Str(4, 6, txt, u16WHITE, u16BLACK); // 字符串显示
        Delay_Ms(30);
    }
}
