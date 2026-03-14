#include "LQ_demo.hpp"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void TFTDemo()
 * @功能说明：TFT 屏幕测试程序
 * @参数说明：无
 * @函数返回：无
 * @调用方法：TFTDemo();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TFTDemo()
{
    TFTSPI_Init(0); // LCD初始化    0：横屏     1：竖屏
    cout << "TFTSPI_Init" << endl;
    TFTSPI_CLS(u16BLUE);    // 蓝色屏幕
    cout << "TFTSPI_CLS" << endl;
    TFTSPI_P8X16Str(0, 2, (char*)"Beijing Longqiu", u16RED, u16BLUE);   // 字符串显示
    TFTSPI_P8X16Str(0, 4, (char*)"Long Qiu i.s.t.", u16WHITE, u16BLACK);   // 字符串显示
    cout << "TFTSPI_P8x16Str" << endl;
    sleep(1);
    char txt[32];
    unsigned short count = 1;
    while(1) 
    {
        memset(txt, 0, sizeof(txt));
        sprintf(txt, "variate:%05d", count);
        TFTSPI_P8X16Str(0, 6, txt, u16RED, u16BLUE);
        usleep(500000);
        count++;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void TFT_Dri_Demo()
 * @功能说明：TFT 屏幕驱动测试程序
 * @参数说明：无
 * @函数返回：无
 * @调用方法：TFT_Dri_Demo();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TFT_Dri_Demo()
{
    TFTSPI_dri_init(1);

    TFTSPI_dri_draw_line(10, 5, 10, 50, u16RED);
    TFTSPI_dri_draw_line(10, 5, 100, 5, u16RED);
    TFTSPI_dri_draw_line(10, 5, 100, 50, u16RED);
    TFTSPI_dri_draw_circle(10, 10, 15, u16RED);
    TFTSPI_dri_draw_rectangle(1, 60, 120, 120, u16RED);
    TFTSPI_dri_fill_area(20, 40, 80, 80, u16RED);
    TFTSPI_dir_P6X8Str(2, 2, "LongQiu", u16BLACK, u16GREEN);
    TFTSPI_dir_P8X8Str(2, 3, "LongQiu", u16BLACK, u16GREEN);
    TFTSPI_dir_P8X16Str(2, 4, "LongQiu", u16BLACK, u16GREEN);

    sleep(1);
    char txt[32];
    unsigned short count = 1;

    while(1)
    {
        memset(txt, 0, sizeof(txt));
        sprintf(txt, "variate:%05d", count);
        TFTSPI_dir_P8X16Str(0, 6, txt, u16RED, u16BLUE);
        usleep(500000);
        count++;
    }
}
