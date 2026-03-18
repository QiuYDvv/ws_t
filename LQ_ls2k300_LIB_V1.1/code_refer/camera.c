#include "camera.h"
#include "function.h"
#include "math.h"

// 定义颜色常量
#define COLOR_BLACK 0x0000
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_WHITE 0xFFFF

uint8 search_line_end = 10; // 搜线终止行

int16 X1 = 0, X2 = 0, Y1 = 0, Y2 = 0; // 显示一些变量(调元素的时候用来看一些变量的值)
uint8 image_01[MT9V03X_H][MT9V03X_W];
uint8 image_yuanshi[MT9V03X_H][MT9V03X_W];

float offset_quanzhong[15] = {0.96, 0.92, 0.88, 0.83, 0.77,
                              0.71, 0.65, 0.59, 0.53, 0.47,
                              0.47, 0.47, 0.47, 0.47, 0.47}; // 偏差权重

uint8 kuandu_saidao[MT9V03X_H - 1] = {
    136, 135, 134, 133, 132, 131, 130, 129, 128, 127,
    126, 125, 124, 123, 122, 121, 120, 119, 118, 117,
    116, 115, 114, 113, 112, 111, 110, 109, 108, 107,
    106, 105, 104, 103, 102, 101, 100, 99, 98, 97,
    96, 95, 94, 93, 92, 91, 90, 89, 88, 87,
    86, 85, 84, 83, 82, 81, 80, 79, 78, 77,
    76, 75, 74, 73, 72, 71, 70, 69, 68, 67,
    60, 59, 58, 57, 56, 55, 54, 53, 52, 51,
    50, 45, 41, 40, 39, 37, 33, 31, 30, 29}; // 前80行的赛道宽度

struct ROAD_TYPE road_type = {
    .straight = 0,    // 直道
    .bend = 0,        // 弯道
    .Ramp = 0,        // 坡道
    .Cross = 0,       // 十字
    .L_Cross = 0,     // 入左十字
    .R_Cross = 0,     // 入右十字
    .LeftCirque = 0,  // 左环岛
    .RightCirque = 0, // 右环岛
    .Fork = 0,        // 三岔
    .Barn_l_out = 0,
    .Barn_r_out = 0,
    .Barn_l_in = 0,
    .Barn_r_in = 0,
};

// 在文件顶部添加此全局变量声明
uint8 in_car_haha_mode = 0; // 标记是否正在执行car_haha函数

// 在文件顶部添加全局变量
int is_bend_global = 0;        // 是否是弯道
int bend_direction_global = 0; // 弯道方向：-1左弯，1右弯，0未定义

// 摄像头处理全流程
void Camera_Display(void)
{
    // if(mt9v03x_finish_flag==1)
    {
        Transfer_Camera();
        mt9v03x_finish_flag = 0; // 在图像使用完毕后  务必清除标志位，否则不会开始采集下一幅图像
        Get01change_Dajin();     // 二值化
                                 //        turn_to_bin();      // 图像二值化
        Pixle_Filter();          // 腐蚀（像素滤波）
        Search_Line(0);          // 初次搜线
        Check_Zhidao();          // 用于检测直道的长度

        check_cheku(110, 50, 2);

        // 如果pause_camera_process为0且disable_element_detection为0，则执行元素识别和处理
        if (!pause_camera_process && !disable_element_detection)
        {
            Element_Test(); // 元素识别
        }
        Element_Handle(); // 元素处理

        HDPJ_lvbo(m_line_x, 20, MT9V03X_H - 1); // 中线滤波
        //        Calculate_Offset();                        //偏差计算
        Calculate_Offset_1(); // 偏差计算
        // Blacking();           // 画出边界中线
        //          // 横向显示 road_type 状态
        //          // 第一行显示
        //          // 第一行显示
        //          ips200_show_string(0, 0, "S:"); // Straight
        //          ips200_show_int(15, 0, road_type.straight, 1);
        //          ips200_show_string(30, 0, "B:"); // Bend
        //          ips200_show_int(45, 0, road_type.bend, 1);
        //          ips200_show_string(60, 0, "Rmp:"); // Ramp
        //          ips200_show_int(95, 0, road_type.Ramp, 1);
        //          ips200_show_string(110, 0, "C:"); // Cross
        //          ips200_show_int(125, 0, road_type.Cross, 1);
        //          ips200_show_string(140, 0, "LC:"); // L_Cross
        //          ips200_show_int(170, 0, road_type.L_Cross, 1);
        //          ips200_show_string(185, 0, "RC:"); // R_Cross
        //          ips200_show_int(215, 0, road_type.R_Cross, 1);

        //        // 第二行显示
        // ips200_show_string(0, 15, "LCq:"); // LeftCirque
        // ips200_show_int(40, 15, road_type.LeftCirque, 1);
        // ips200_show_string(55, 15, "RCq:"); // RightCirque
        // ips200_show_int(95, 15, road_type.RightCirque, 1);
        //        ips200_show_string(110, 15, "Fk:"); // Fork
        //        ips200_show_int(140, 15, road_type.Fork, 1);
        //        ips200_show_string(155, 15, "BLo:"); // Barn_l_out
        //        ips200_show_int(195, 15, road_type.Barn_l_out, 1);
        //
        //        // 第三行显示
        //        ips200_show_string(0, 30, "BRo:"); // Barn_r_out
        //        ips200_show_int(40, 30, road_type.Barn_r_out, 1);
        //        ips200_show_string(55, 30, "BLi:"); // Barn_l_in
        //        ips200_show_int(95, 30, road_type.Barn_l_in, 1);
        //        ips200_show_string(110, 30, "BRi:"); // Barn_r_in
        //        ips200_show_int(150, 30, road_type.Barn_r_in, 1);

        // ips200_show_int(16 * 15 + 10, 130, offset, 10);
        // ips200_show_int(60, 130, stop_1, 10);
        //         ips200_show_int(60, 130, road_type.bend, 10);
        //         ips200_show_int(75, 130, road_type.straight, 10);
        // ips200_show_uint(0, 150, huandao_stage, 10);
        for (uint8 i = (MT9V03X_H - 1); i > search_line_end; i--)
        {
            m_line_x[i] = (l_line_x[i] + r_line_x[i]) / 2;
        }
        //        if(flag.image_show)
        //        {            //屏幕显示
        //            Blacking();
        //        }
    }
}

/****************************END*****************************************/

// 元素处理
uint16 distance_L_Cross_out = 0;
uint16 distance_R_Cross_out = 0;
uint8 flag_L_shizi_R_turn = 0;
uint8 flag_R_shizi_L_turn = 0;

int16 annulus_s1 = 0;
int16 jiaodu_jifen = 0;
int16 annulus_jinku_s = 0;
void Element_Handle()
{
    /***************圆环*********************/
    if (road_type.LeftCirque)
    {
        Handle_Left_Cirque();
        return; // 处理完左环岛后直接返回
    }
    else if (road_type.RightCirque)
    {
        Handle_Right_Cirque();
        return; // 处理完右环岛后直接返回
    }

    /****************斜十字********************/
    if (road_type.L_Cross)
    {
        Handle_L_Cross();
        return; // 处理完左斜十字后直接返回
    }
    else if (road_type.R_Cross)
    {
        Handle_R_Cross();
        return; // 处理完右斜十字后直接返回
    }

    /****************三岔********************/
    if (road_type.Fork)
    {
        // Handle_Fork();
        road_type.Fork = 0;
        return; // 处理完三岔后直接返回
    }

    /*************十字***********************/
    if (road_type.Cross)
    {
        Handle_Cross();
        // 不需要return，因为已经是最后一个元素处理
    }
}
/****************************END*****************************************/

// 元素识别
int16 l_line_qulv = 0, r_line_qulv = 0; // 用于存储左右线的曲率
void Element_Test(void)
{ // 调用 Process_Curvity 函数计算左车道线的曲率，并存储在变量中
    l_line_qulv = 1000 * Process_Curvity(l_line_x[MT9V03X_H - 10], MT9V03X_H - 10, l_line_x[MT9V03X_H - 25], MT9V03X_H - 25, l_line_x[MT9V03X_H - 40], MT9V03X_H - 40);
    r_line_qulv = 1000 * Process_Curvity(r_line_x[MT9V03X_H - 10], MT9V03X_H - 10, r_line_x[MT9V03X_H - 25], MT9V03X_H - 25, r_line_x[MT9V03X_H - 40], MT9V03X_H - 40);

    Mid_Col();
    Outside_protect(); // 出界停车

    // 这一行是一系列条件判断，判断当前的车道类型，例如左圆环、右圆环、左斜十字、右斜十字、三岔路、十字路口等
    if (!road_type.Fork && !road_type.LeftCirque && !road_type.RightCirque && !road_type.L_Cross && !road_type.R_Cross && !road_type.Cross && !road_type.Barn_l_in && !road_type.Barn_r_in)
    {
        // 判断左斜十字和左圆环
        if (length > 50 && l_lose_value > 30 && r_lose_value < 14 && fabs(r_line_qulv) < 15 && !Tututu(1))
        {
            if (Cirque_or_Cross(1, l_losemax) >= 15 && road_type.LeftCirque == 0 && road_type.L_Cross == 0) // 左侧丢线位置y的下方有较多白点,判断为圆环
            {
                road_type.LeftCirque = 1;
                road_type.straight = 0;
                road_type.bend = 0;
            }
            else if (road_type.L_Cross == 0 && road_type.LeftCirque == 0 && road_type.Fork == 0 && r_line_x[50] < 100 && r_line_x[53] < 100)
            {
                road_type.L_Cross = 1;
                road_type.straight = 0;
                road_type.bend = 0;
            }
        }
        // 右斜十字和右圆环
        else if (length > 50 && r_lose_value > 30 && l_lose_value < 14 && abs(l_line_qulv) < 15 && !Tututu(2))
        {
            // 降低白点检测阈值，增加识别率
            if (Cirque_or_Cross(2, r_losemax) >= 10 && road_type.RightCirque == 0 && road_type.R_Cross == 0)
            {
                road_type.RightCirque = 1;
                road_type.straight = 0;
                road_type.bend = 0;
            }
            else if (road_type.R_Cross == 0 && road_type.RightCirque == 0 && road_type.Fork == 0 && l_line_x[50] > 100 && l_line_x[53] > 100) //
            {
                road_type.R_Cross = 1;
                road_type.straight = 0;
                road_type.bend = 0;
            }
        }
        // 检测入三岔
        else if (RoundaboutGetArc(1, 15) && RoundaboutGetArc(2, 15))
        {
            //            BUZZER_ON;
            road_type.Fork = 1;
            road_type.straight = 0;
            road_type.bend = 0;
        }
        // 检测十字
        else if (length > 60 && r_lose_value > 20 && l_lose_value > 20)
        {
            BUZZER_ON;
            road_type.Cross = 1;
            road_type.straight = 0;
            road_type.bend = 0;
            BUZZER_OFF;
        }
        // 排除特殊元素，判断直道和弯道
        else if (sudu_yingzi <= 65)
        {
            road_type.straight = 0;
            road_type.bend = 1; // 弯道

            // 新增：通过判断中线发展方向确定左弯或右弯
            // 取下1/4点和最底点的中线位置进行比较
            int quarter_point = m_line_x[MT9V03X_H * 2/3];  // 取下1/4点
            int bottom_point = m_line_x[MT9V03X_H * 5/6];     // 取最底点

            // 计算中线发展方向
            if (quarter_point < bottom_point)
            {
                // 中线向左发展，判断为左弯
                is_bend_global = 1;
                bend_direction_global = -1;
            }
            else if (quarter_point > bottom_point)
            {
                // 中线向右发展，判断为右弯
                is_bend_global = 1;
                bend_direction_global = 1;
            }
            else
            {
                // 中线垂直，可能是直道或其他元素
                is_bend_global = 1;
                bend_direction_global = 0;
            }
        }
        else // 如果以上条件都不满足，则判断为直道
        {
            road_type.straight = 1; // 直道
            road_type.bend = 0;
        }
    }
}
/****************************END*****************************************/

//*******************图像数组转存函数************************************//
void Transfer_Camera()
{
    for (uint8 y = 0; y < MT9V03X_H; y++) // 存储到一个新数组，后续处理（High为120，Width为188，刷新率为50）
    {
        for (uint8 x = 0; x < MT9V03X_W; x++)
        {
            image_yuanshi[y][x] = mt9v03x_image[y][x];
        }
    }
}
/****************************END*****************************************/

uint8 kuan[MT9V03X_H];  // 基础搜线函数
uint8 m_line_x[MT9V03X_H];                                    // 储存赛道中线的列
uint8 r_line_y[MT9V03X_H], l_line_y[MT9V03X_H];               // 储存左右边界的行数
uint8 r_line_x[MT9V03X_H], l_line_x[MT9V03X_H];               // 储存原始图像的左右边界的列数
uint8 r_second_line_x[MT9V03X_H], l_second_line_x[MT9V03X_H]; // 储存补线之后的左右边界值（还未用到）
uint8 m_second_line_x[MT9V03X_H];                             // 储存赛道中线的列
uint8 r_lose_value = 0, l_lose_value = 0;                     // 左右丢线数，后面注意要走一次清零
uint8 r_search_flag[MT9V03X_H], l_search_flag[MT9V03X_H];     // 是否搜到线的标志
uint8 height, r_width, l_width;                               // 循环变量名
uint8 r_losemax, l_losemax;


void Search_Line(uint8 type) // 0为普通模式，1为斑马线模式（只有搜线起始横坐标不同）
{
    uint8 l_flag = 0, r_flag = 0;
    uint8 l_search_start, r_search_start;       // 搜线起始列坐标
    uint8 r_searchget_flage, l_searchget_flage; // 搜到线时的标志位
    r_searchget_flage = 1;
    l_searchget_flage = 1; // 开始搜索是默认为上次搜到线
    r_lose_value = 0;
    l_lose_value = 0; // 左右丢线数，后面注意要走一次清零

    // 初始化边线数组，避免出现未定义值
    for (uint8 i = 0; i < MT9V03X_H; i++)
    {
        l_line_x[i] = 0;
        r_line_x[i] = MT9V03X_W - 1;
        l_search_flag[i] = 0;
        r_search_flag[i] = 0;
        neo_l_line_x[i] = 0;
        neo_r_line_x[i] = MT9V03X_W - 1;
        l_second_line_x[i] = 0;
        r_second_line_x[i] = MT9V03X_W - 1;
    }

    // 从图像的底部向上搜索横线。search_line_end 是一个全局变量，它记录了上一次搜索的结束位置
    for (height = (MT9V03X_H - 1); height > search_line_end; height--)
    {
        // 确定每行的搜线起始横坐标
        if (type == 0)                                                                                                                            // 普通模式
        {                                                                                                                                         // 根据上一行搜到的线、丢线情况以及当前行的位置来确定左右搜线的起始位置
            if ((height > MT9V03X_H - 5) || ((l_line_x[height + 1] == 0) && (r_line_x[height + 1] == MT9V03X_W - 1) && (height < MT9V03X_H - 4))) // 前四行，或者左右都丢线的行
            {
                l_search_start = MT9V03X_W / 2;
                r_search_start = MT9V03X_W / 2;
            }
            else if ((l_line_x[height + 1] != 0) && (r_line_x[height + 1] != MT9V03X_W - 1) && (height < MT9V03X_H - 4)) // 左右都不丢线
            {
                l_search_start = l_line_x[height + 1] + 7;
                r_search_start = r_line_x[height + 1] - 7;
            }
            else if ((l_line_x[height + 1] != 0 && r_line_x[height + 1] == MT9V03X_W - 1) && (height < MT9V03X_H - 4)) // 左不丢线,右丢线
            {
                l_search_start = l_line_x[height + 1] + 7;
                r_search_start = MT9V03X_W / 2;
            }
            else if ((l_line_x[height + 1] == 0 && r_line_x[height + 1] != MT9V03X_W - 1) && (height < MT9V03X_H - 4)) // 右不丢线,左丢线
            {
                l_search_start = MT9V03X_W / 2;
                r_search_start = r_line_x[height + 1] - 7;
            }
        }

        // 确保搜线起始点在有效范围内
        if (l_search_start >= MT9V03X_W - 2)
            l_search_start = MT9V03X_W / 2;
        if (r_search_start <= 2)
            r_search_start = MT9V03X_W / 2;

        // 如果当前行及上一行和上两行的中间列都是黑色，并且当前行位置在图像底部之上 40 行
        if ((image_01[height][MT9V03X_W / 2] == 0) && (image_01[height - 1][MT9V03X_W / 2] == 0) && (image_01[height - 2][MT9V03X_W / 2] == 0) && (height < MT9V03X_H - 40)) // 搜线终止条件
        {
            search_line_end = height + 1;
            break;
        }
        else
        {
            search_line_end = 10;
        }

        // 搜线起始位置向左搜索
        for (l_width = l_search_start; l_width > 1; l_width--) // 左边搜线
        {                                                      // 如果遇到黑黑白的情况，则认为找到了左边的线。
            if (image_01[height][l_width - 2] == 0 && image_01[height][l_width - 1] == 0 && image_01[height][l_width] != 0 && l_width > 2)
            { // 黑黑白
                l_line_x[height] = l_width - 1;
                l_line_y[height] = height;
                l_search_flag[height] = 1;
                l_searchget_flage = 1;

                // 新增：如果检测到的左边线太靠左（紧贴左边沿10像素以内），也判断为丢线
                if (l_line_x[height] <= 10)
                {
                    if (l_flag == 0)
                    {
                        l_flag = 1;
                        l_losemax = height;
                    }
                    l_line_x[height] = 0;
                    l_line_y[height] = height;
                    l_search_flag[height] = 0;
                    l_searchget_flage = 0;
                    l_lose_value++;
                }
                break;
            }
            // 如果搜索到图像边缘还没找到线，则记录丢线情况并更新相关变量
            else if (l_width == 2)
            {
                if (l_flag == 0)
                {
                    l_flag = 1;
                    l_losemax = height;
                }
                l_line_x[height] = 0;
                l_line_y[height] = height;
                l_search_flag[height] = 0;
                l_searchget_flage = 0;
                l_lose_value++;
                break;
            }
        }
        // 从搜线起始位置向右搜索
        for (r_width = r_search_start; r_width < (MT9V03X_W - 2); r_width++) // 右边搜线
        {                                                                    // 如果遇到白黑黑的情况，则认为找到了右边的线。
            if (image_01[height][r_width] != 0 && image_01[height][r_width + 1] == 0 && image_01[height][r_width + 2] == 0 && r_width < MT9V03X_W - 3)
            { // 白黑黑
                r_line_x[height] = r_width + 1;
                r_line_y[height] = height;
                r_search_flag[height] = 1;
                r_searchget_flage = 1;

                // 新增：检查右边线是否太靠右边缘
                if (r_line_x[height] >= MT9V03X_W - 7) // 紧贴右边沿7像素以内
                {
                    if (r_flag == 0)
                    {
                        r_flag = 1;
                        r_losemax = height;
                    }
                    r_line_x[height] = MT9V03X_W - 1; // 设置为丢线标记
                    r_line_y[height] = height;
                    r_search_flag[height] = 0;
                    r_searchget_flage = 0;
                    r_lose_value++;
                }
                break;
            }
            else if (r_width == MT9V03X_W - 3)
            {
                if (r_flag == 0)
                {
                    r_flag = 1;
                    r_losemax = height;
                }
                r_line_x[height] = MT9V03X_W - 1;
                r_line_y[height] = height;
                r_search_flag[height] = 0;
                r_searchget_flage = 0;
                r_lose_value++;
                break;
            }
        }

        // 检查并修正异常值
        if (r_line_x[height] <= l_line_x[height])
        {
            // 如果右边线在左边线左侧，说明有异常
            if (r_search_flag[height] == 1 && l_search_flag[height] == 0)
            {
                // 右边线有效，左边线无效，使用右边线和固定宽度来估计左边线
                l_line_x[height] = (r_line_x[height] > kuandu_saidao[MT9V03X_H - 1 - height]) ? (r_line_x[height] - kuandu_saidao[MT9V03X_H - 1 - height]) : 0;
            }
            else if (r_search_flag[height] == 0 && l_search_flag[height] == 1)
            {
                // 左边线有效，右边线无效，使用左边线和固定宽度来估计右边线
                r_line_x[height] = (l_line_x[height] + kuandu_saidao[MT9V03X_H - 1 - height] < MT9V03X_W - 1) ? (l_line_x[height] + kuandu_saidao[MT9V03X_H - 1 - height]) : (MT9V03X_W - 1);
            }
            else if (r_search_flag[height] == 0 && l_search_flag[height] == 0)
            {
                // 两边都无效，使用默认值
                l_line_x[height] = 0;
                r_line_x[height] = MT9V03X_W - 1;
            }
        }

        kuan[height] = r_line_x[height] - l_line_x[height]; // 计算每一行搜到的线的宽度
    }
    // 在Search_Line函数末尾添加
    // 将搜索到的边线复制到neo数组
    for (uint8 y = MT9V03X_H - 1; y > search_line_end; y--)
    {
        neo_l_line_x[y] = l_line_x[y];
        neo_r_line_x[y] = r_line_x[y];
    }
}
/****************************END*****************************************/

// 二值化
int16 yuzhi = 0;               // 按键修改阈值变量
uint32 Threshold;              // 阈值
uint32 Threshold_static = 180; // 阈值静态下限  //180
uint32 Threshold_detach = 300; // 阳光算法分割阈值(光强越强,该值越大)  // 300
void Get01change_Dajin(void)
{ // 调用 Threshold_Deal() 函数来计算阈值，并将结果存储在 Threshold 变量中
    // 求阈值的方法有问题，待解决！！！
    //    Threshold = Threshold_Deal(image_yuanshi[0], MT9V03X_W, MT9V03X_H, Threshold_detach+yangguang);
    //
    //    // 如果计算得到的阈值小于阈值的静态下限 (Threshold_static)，则将阈值设为静态下限值
    //    if (Threshold < Threshold_static+ yuzhi)
    //    {
    //        Threshold = Threshold_static+ yuzhi;
    //    }
    //    ips200_show_uint(0, 200, Threshold, 10);
    //    uint8 thre;
    //    for(uint8 y = 0; y < MT9V03X_H; y++)      // 遍历图像的每一行
    //    {
    //        for(uint8 x = 0; x < MT9V03X_W; x++)  // 遍历图像的每一列
    //        {   // 根据像素在图像中的位置不同，调整阈值的大小
    ////            if (x <= 15)
    ////                thre = Threshold - 10;  // 图像的边缘可能存在一些干扰，因此在图像边缘处，将阈值适当减小
    ////            else if (x >= MT9V03X_W-15)
    ////                thre = Threshold - 10;
    ////            else
    ////                thre = Threshold;

    //            if (image_yuanshi[y][x] >thre)         //数值越大，显示的内容越多，较浅的图像也能显示出来
    //                image_01[y][x] = 255;  //白
    //            else
    //                image_01[y][x] = 0;  //黑
    //        }
    //    }

    uint8 i, j;
    Threshold = otsuThreshold(image_yuanshi[0], MT9V03X_W, MT9V03X_H);
    for (i = 0; i < MT9V03X_H; i++)
    {
        for (j = 0; j < MT9V03X_W; j++)
        { // 检查其灰度值是否大于计算得到的阈值
            if (image_yuanshi[i][j] > Threshold)
                image_01[i][j] = 255; // 白
            else
                image_01[i][j] = 0; // 黑
        }
    }
}
/****************************END*****************************************/

/// 求二值化阈值
//-------------------------------------------------------------------------------------------------------------------
//  @param      image  图像数组，存储了图像的像素数据。
//  @param      clo    宽
//  @param      row    高
//  @param      pixel_threshold 阈值分离参数，用于计算二值化阈值
//-------------------------------------------------------------------------------------------------------------------
uint8 Threshold_Deal(uint8 *image, uint16 col, uint16 row, uint32 pixel_threshold)
{
#define GrayScale 256 // 定义灰度级
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale]; // 存储图像中每个像素值的出现次数
    float pixelPro[GrayScale]; // 存储图像中每个像素值的比例
    int i, j;
    int pixelSum = width * height; // 计算了图像中像素的总数量
    uint8 threshold = 0;
    uint8 *data = image; // 指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    { // 初始化数组，将其元素全部设为 0。
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    // 用于存储图像中所有像素值的总和
    uint32 gray_sum = 0;
    // 统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 1) // 遍历图像的每一行
    {
        for (j = 0; j < width; j += 1) // 遍历图像的每一列
        {
            pixelCount[(int)data[i * width + j]]++; // 将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];   // 灰度值总和
        }
    }

    // 计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++) // 遍历灰度级
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    // 遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < pixel_threshold; j++)
    {
        w0 += pixelPro[j];        // 背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
        u0tmp += j * pixelPro[j]; // 背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;                                          // 背景平均灰度
        u1 = u1tmp / w1;                                          // 前景平均灰度
        u = u0tmp + u1tmp;                                        // 全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2); // 加权方差
        if (deltaTmp > deltaMax)                                  // 如果当前计算得到的加权方差大于之前记录的最大加权方差，则更新阈值为当前像素值 j
        {
            deltaMax = deltaTmp;
            threshold = (uint8)j; // 本来这里没有强制类型转换的,我自己加的
        }
        if (deltaTmp < deltaMax) // 如果当前计算得到的加权方差小于最大加权方差，则跳出循环
        {
            break;
        }
    }
    return threshold;
}
/****************************END*****************************************/

// 减少运算量的二值化
uint8 my_adapt_threshold(uint8 *image, uint16 width, uint16 height) // 注意计算阈值的一定要是原图像
{
#define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height / 4;
    uint8 threshold = 0;
    uint8 *data = image; // 指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum = 0;
    // 统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 2)
    {
        for (j = 0; j < width; j += 2)
        {
            pixelCount[(int)data[i * width + j]]++; // 将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];   // 灰度值总和
        }
    }

    // 计算每个像素值的点在整幅图像中的比例

    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    // 遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (uint16 j = 0; j < GrayScale; j++)
    {

        w0 += pixelPro[j];        // 背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j]; // 背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;   // 背景平均灰度
        u1 = u1tmp / w1;   // 前景平均灰度
        u = u0tmp + u1tmp; // 全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return threshold;
}
/****************************END*****************************************/

//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width = col;
    uint16 Image_Height = row;
    int X;
    uint16 Y;
    uint8 *data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack = 0, OmegaFore = 0, MicroBack = 0, MicroFore = 0, SigmaB = 0, Sigma = 0; // 类间方差;
    uint32 MinValue = 0, MaxValue = 0;
    uint8 Threshold = 0;

    for (Y = 0; Y < Image_Height; Y++) // Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        // Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y * Image_Width + X]]++; // 统计每个灰度值的个数信息
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++)
        ; // 获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--)
        ; // 获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue; // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue; // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y]; //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y; // 灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];                                               // 前景像素点数
        PixelFore = Amount - PixelBack;                                                    // 背景像素点数
        OmegaBack = (double)PixelBack / Amount;                                            // 前景像素百分比
        OmegaFore = (double)PixelFore / Amount;                                            // 背景像素百分比
        PixelIntegralBack += HistGram[Y] * Y;                                              // 前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;                             // 背景灰度值
        MicroBack = (double)PixelIntegralBack / PixelBack;                                 // 前景灰度百分比
        MicroFore = (double)PixelIntegralFore / PixelFore;                                 // 背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // g
        if (Sigma > SigmaB)                                                                // 遍历最大的类间方差g
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------

uint8 image_thereshold; // 图像分割阈值
// uint8 bin_image[image_h][image_w];//二维数组，用于存储经过二值化处理后的图像
void turn_to_bin(void)
{
    uint8 i, j;
    image_thereshold = otsuThreshold(image_yuanshi[0], MT9V03X_W, MT9V03X_H);
    for (i = 0; i < MT9V03X_H; i++)
    {
        for (j = 0; j < MT9V03X_W; j++)
        { // 检查其灰度值是否大于计算得到的阈值
            if (image_yuanshi[i][j] > image_thereshold)
                image_01[i][j] = 255; // 白
            else
                image_01[i][j] = 0; // 黑
        }
    }
}

extern uint8 cross_stage;
// 画边线和中线(液晶画点,如果屏幕显示的图像进行了缩放就不能适用)
void Blacking()
{
    // 1. 显示原始二值化图像
    ips200_show_gray_image(0, 0, image_01[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);

    // 2. 添加两条参考红线，用于显示弯道判断使用的采样点位置
    for(uint8 x = 0; x < MT9V03X_W; x++)
    {
        // 画第一条红线 - 位置在MT9V03X_H * 5/6
        ips200_draw_point(x, MT9V03X_H * 5/6, COLOR_RED);
        
        // 画第二条红线 - 位置在MT9V03X_H - 5
        ips200_draw_point(x, MT9V03X_H * 2/3, COLOR_RED);
    }

    // 3. 绘制赛道边界线和中线（彩色）
    for (uint8 y = (MT9V03X_H - 1); y > search_line_end; y--)
    {
        // 确保坐标有效再绘制
        // 左边线（红色）
        if (l_line_x[y] < MT9V03X_W && y < MT9V03X_H)
            ips200_draw_point(l_line_x[y], y, COLOR_RED);

        // 右边线（蓝色）- 确保右边线不会画在左侧
        if (r_line_x[y] < MT9V03X_W && r_line_x[y] > l_line_x[y] && y < MT9V03X_H)
            ips200_draw_point(r_line_x[y], y, COLOR_BLUE);

        // 中线（绿色）
        if (m_line_x[y] < MT9V03X_W && m_line_x[y] > 0 && y < MT9V03X_H)
            ips200_draw_point(m_line_x[y], y, COLOR_GREEN);

        // 4. 添加补线部分显示（与对应边线颜色一致）
        // 左边补线（红色）
        if (l_second_line_x[y] < MT9V03X_W && l_second_line_x[y] != l_line_x[y])
            ips200_draw_point(l_second_line_x[y], y, COLOR_RED);

        // 右边补线（蓝色）
        if (r_second_line_x[y] < MT9V03X_W && r_second_line_x[y] != r_line_x[y])
            ips200_draw_point(r_second_line_x[y], y, COLOR_BLUE);
    }

    // 6. 显示调试信息
    ips200_show_uint(30, 130, Threshold, 10);
    ips200_show_uint(20, 150, cross_stage, 10);
    ips200_show_uint(40, 150, road_type.LeftCirque, 10);
    ips200_show_uint(60, 150, road_type.RightCirque, 10);
    ips200_show_uint(0, 170, l_lose_value, 10);  // 左丢线
    ips200_show_uint(30, 170, r_lose_value, 10); // 右丢线
    ips200_show_uint(0, 190, annulus_s1, 10);

    ips200_show_uint(0, 220, cross_left[1], 10);
    ips200_show_uint(30, 220, cross_right[1], 10);
    ips200_show_uint(60, 220, cross_left[0], 10);
    ips200_show_uint(90, 220, cross_right[0], 10);
}

/****************************END*****************************************/

//****************************偏差计算*******************************//
int16 offset; // 摄像头处理得到的偏差
// int16 shizi_s1;
// void Calculate_Offset()
//{
//
//     uint16 total_midcourt_line =0;            //计算中线值
//     uint16 mid_value = MT9V03X_W/2 -1 ;       //屏幕中间
//
//     for(uint8 y=74; y<78; y++)                //累加控制行的中线值
//     {
//         m_line_x[y] = (l_line_x[y] + r_line_x[y])/2;
//         total_midcourt_line = total_midcourt_line + m_line_x[y];
//     }
//     offset = (total_midcourt_line - 4*mid_value);
//     if((road_type.Barn_r_in == 1) && (pass_barn == 2))
//     {
//         offset =200;
//     }
//     if((road_type.Barn_l_in == 1) && (pass_barn == 2))
//     {
//         offset =-200;
//     }
// }
/****************************END*****************************************/

void Calculate_Offset_1()
{
    offset = 0;

    // 使用neo数组计算中线而不是原始边线数组
    for (uint8 y = MT9V03X_H - 1; y >= 10; y--)
    {
        // 原来的代码：
        // m_line_x[y] = 1.0 * (l_line_x[y] + r_line_x[y]) / 2;

        // 修改为使用neo数组:
        m_line_x[y] = 1.0 * (neo_l_line_x[y] + neo_r_line_x[y]) / 2;
    }

    // 对中线进行平滑滤波
    HDPJ_lvbo(m_line_x, 20, MT9V03X_H - 1);

    // 计算偏移量（这部分不变）
    for (uint8 y = MT9V03X_H - 15; y >= MT9V03X_H - 25; y--)
    {
        offset += offset_quanzhong[MT9V03X_H - 20 - y] * (m_line_x[y] - MT9V03X_W / 2);
    }

    offset = offset / 2;
}

// 三点法计算赛道曲率
float Process_Curvity(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
    float K;
    int S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    // 面积的符号表示方向
    int16 q1 = (int16)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    uint8 AB = My_Sqrt(q1);
    q1 = (int16)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    uint8 BC = My_Sqrt(q1);
    q1 = (int16)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    uint8 AC = My_Sqrt(q1);
    if (AB * BC * AC == 0)
    {
        K = 0;
    }
    else
        K = (float)4 * S_of_ABC / (AB * BC * AC);
    return K;
}
/****************************END*****************************************/

// 自己写的开方函数
uint8 My_Sqrt(int16 x)
{
    uint8 ans = 0, p = 0x80;
    while (p != 0)
    {
        ans += p;
        if (ans * ans > x)
        {
            ans -= p;
        }
        p = (uint8)(p / 2);
    }
    return (ans);
}
/****************************END*****************************************/

// 识别斑马线
uint8 flag_starting_line = 0; // 用于标志是否检测到了斑马线
int stop_1 = 0;               // 遇到斑马线计数
// 车库检测 - 判断斑马线部分
//-------------------------------------------------------------------------------------------------------------------
//  @param      start_point  要检查的起始高度（行数）
//  @param      end_point    要检查的结束高度（行数）
//  @param      qiangdu      黑色块的宽度阈值
void check_cheku(uint8 start_point, uint8 end_point, uint8 qiangdu)
{
    static uint8 last_line_found = 0;   // 保留以兼容其他可能使用此变量的代码
    static uint16 cooldown_counter = 0; // 冷却期计数器
    static uint8 in_cooldown = 0;       // 是否在冷却期的标志
    uint8 times = 0;
    uint8 line_found = 0; // 本次是否检测到斑马线

    // 如果在冷却期内，增加计数器并检查是否结束冷却
    if (in_cooldown)
    {
        cooldown_counter++;
        if (cooldown_counter >= 200)
        {
            in_cooldown = 0;
            cooldown_counter = 0;
        }
        // 在冷却期内不进行斑马线检测
        return;
    }

    // 进行正常的斑马线检测
    for (uint8 height = start_point; height >= end_point; height--)
    {
        uint8 black_blocks_l = 0, black_blocks_r = 0;
        uint8 cursor_l = 0, cursor_r = 0;

        for (uint8 width_l = MT9V03X_W / 2, width_r = MT9V03X_W / 2; width_l >= 1 && width_r < MT9V03X_W - 2; width_l--, width_r++)
        {
            if (image_01[height][width_l] == 0)
            {
                if (cursor_l > 40)
                {
                    break;
                }
                else
                {
                    cursor_l++;
                }
            }
            else
            {
                if (cursor_l >= qiangdu && cursor_l <= qiangdu + 8)
                {
                    black_blocks_l++;
                    cursor_l = 0;
                }
                else
                {
                    cursor_l = 0;
                }
            }

            if (image_01[height][width_r] == 0)
            {
                if (cursor_r >= 20)
                {
                    break;
                }
                else
                {
                    cursor_r++;
                }
            }
            else
            {
                if (cursor_r >= qiangdu && cursor_r <= qiangdu + 8)
                {
                    black_blocks_r++;
                    cursor_r = 0;
                }
                else
                {
                    cursor_r = 0;
                }
            }
        }
        if ((black_blocks_l + black_blocks_r) >= 3 && (black_blocks_l + black_blocks_r) <= 20)
        {
            times++;
        }
    }

    // 判断本次是否检测到斑马线
    if (times >= (start_point - end_point - 5) / 3)
    {
        line_found = 1;
        flag_starting_line = 1;

        // 根据当前stop_1值决定如何更新
        if (stop_1 == 0)
        {
            stop_1 = 1;
            in_cooldown = 1; // 进入冷却期
            cooldown_counter = 0;
        }
        else if (stop_1 == 1)
        {
            stop_1 = 2;
        }
    }
    else
    {
        line_found = 0;
        flag_starting_line = 0;
    }

    // 保存当前状态用于下一次检测
    last_line_found = line_found;
}

/****************************END*****************************************/

// 腐蚀（像素滤波,去除偶尔出现的噪点,效果有限）(这个函数有问题,因为当时二值化时用1表示的白,现在用255表示白,得改一下)
void Pixle_Filter()
{ // 遍历图像的高度，从第 10 行开始到 MT9V03X_H-10 行结束，避免处理图像边缘
    for (uint8 height = 10; height < MT9V03X_H - 10; height++)
    { // 遍历图像的宽度，从第 10 列开始到 MT9V03X_W-10 列结束，同样避免处理图像边缘
        for (uint8 width = 10; width < MT9V03X_W - 10; width = width + 1)
        {
            if ((image_01[height][width] == 0) && (image_01[height - 1][width] + image_01[height + 1][width] + image_01[height][width + 1] + image_01[height][width - 1] >= 3 * 255))
            { // 一个黑点的上下左右的白点大于等于三个，令这个点为白
                image_01[height][width] = 1;
            } // 如果当前像素不为黑色（值不为 0），并且当前像素上下左右四个像素中至多有两个为白色(值为 255),则将当前像素设为黑色（值为 0）
            else if ((image_01[height][width] != 0) && (image_01[height - 1][width] + image_01[height + 1][width] + image_01[height][width + 1] + image_01[height][width - 1] < 2 * 255))
            {
                image_01[height][width] = 0;
            }
        }
    }
}
/****************************END*****************************************/

// 找车库拐点(简单粗暴版)
// uint8 leftx_1[2],left_1[2];
uint8 rightx_1[2] = {MT9V03X_W - 1, MT9V03X_W - 1}, righty_1[2] = {MT9V03X_H - 1, MT9V03X_H - 1};
void Check_guaidian_cheku_1(uint8 type)
{
    for (uint8 y = MT9V03X_H - 1; y > 10; y--)
    {
        if (image_01[y][MT9V03X_W - 3] == 0 && image_01[y - 1][MT9V03X_W - 3] == 0 && image_01[y - 2][MT9V03X_W - 3] == 0)
        { // y<MT9V03X_H-30 &&
            rightx_1[1] = MT9V03X_W - 3;
            righty_1[1] = y;
            break;
        }
    }
}
/****************************END*****************************************/

// 本来就是同一边界上的两点拉线
void La_zhixian(uint8 x_down, uint8 y_down, uint8 x_up, uint8 y_up, uint8 *array) // 输入两个点的横纵坐标
{
    int16 k = ((int16)y_down - (int16)y_up);
    if (k == 0)
        k = 1;
    float K = ((float)x_down - (float)x_up) / k;
    float X = (float)x_down;

    // 判断是左边线还是右边线
    uint8 is_left_line = 0;
    if (array == l_line_x)
    {
        is_left_line = 1;
    }

    // 保存原始边线数据到neo数组
    for (uint8 i = 0; i < MT9V03X_H; i++)
    {
        if (is_left_line)
        {
            neo_l_line_x[i] = l_line_x[i];
        }
        else
        {
            neo_r_line_x[i] = r_line_x[i];
        }
    }

    // 执行拉直线操作
    if (y_down > y_up)
    {
        for (uint8 y = y_down - 1; y > y_up; y--)
        {
            X -= K;
            // 保存补线到second_line数组
            if (is_left_line)
            {
                l_second_line_x[y] = (uint8)X;
                // 同时更新neo数组
                neo_l_line_x[y] = (uint8)X;
            }
            else
            {
                r_second_line_x[y] = (uint8)X;
                // 同时更新neo数组
                neo_r_line_x[y] = (uint8)X;
            }
        }
    }
    else
    {
        for (uint8 y = y_down + 1; y < y_up; y++)
        {
            X += K;
            // 保存补线到second_line数组
            if (is_left_line)
            {
                l_second_line_x[y] = (uint8)X;
                // 同时更新neo数组
                neo_l_line_x[y] = (uint8)X;
            }
            else
            {
                r_second_line_x[y] = (uint8)X;
                // 同时更新neo数组
                neo_r_line_x[y] = (uint8)X;
            }
        }
    }

    // 端点也记录为补线
    if (is_left_line)
    {
        l_second_line_x[y_down] = x_down;
        l_second_line_x[y_up] = x_up;
    }
    else
    {
        r_second_line_x[y_down] = x_down;
        r_second_line_x[y_up] = x_up;
    }

    // 修改原数组（为了保持与原函数兼容）
    if (y_down > y_up)
    {
        X = (float)x_down;
        for (uint8 y = y_down - 1; y > y_up; y--)
        {
            X -= K;
            array[y] = (uint8)X;
        }
    }
    else
    {
        X = (float)x_down;
        for (uint8 y = y_down + 1; y < y_up; y++)
        {
            X += K;
            array[y] = (uint8)X;
        }
    }
}
/****************************END*****************************************/

// 最小二乘法计算斜率和截距
float parameterA = 0, parameterB = 0;
void Regression(uint8 type, uint8 startline, uint8 endline)
{ // 一左，二右，0中间
    // 添加行范围检查
    if (startline >= MT9V03X_H || endline >= MT9V03X_H || startline > endline)
        return;

    uint8 i = 0;
    uint8 sumlines = endline - startline;
    int16 sumX = 0;
    int16 sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;

    if (type == 0) // 拟合中线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += m_line_x[i];
        }
        if (sumlines != 0)
        {
            averageX = sumX * 1.0 / sumlines; // x的平均值
            averageY = sumY * 1.0 / sumlines; // y的平均值
        }
        for (i = startline; i < endline; i++)
        {
            sumUp += (m_line_x[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 1) // 拟合左线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += l_line_x[i];
        }
        averageX = sumX * 1.0 / sumlines; // x的平均值
        averageY = sumY * 1.0 / sumlines; // y的平均值
        for (i = startline; i < endline; i++)
        {
            sumUp += (l_line_x[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2) // 拟合右线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += r_line_x[i];
        }
        averageX = sumX * 1.0 / sumlines; // x的平均值
        averageY = sumY * 1.0 / sumlines; // y的平均值
        for (i = startline; i < endline; i++)
        {
            sumUp += (r_line_x[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}
/****************************END*****************************************/

// 最小二乘法划线
void Hua_Xian(uint8 type, uint8 startline, uint8 endline, float parameterB, float parameterA)
{
    for (uint8 i = startline; i <= endline; i++)
    {
        if (type == 1)
        {
            // 保存到补线数组
            l_second_line_x[i] = (uint8)(parameterB * i + parameterA);
            // 更新neo数组
            neo_l_line_x[i] = (uint8)(parameterB * i + parameterA);
        }
        else if (type == 2)
        {
            // 保存到补线数组
            r_second_line_x[i] = (uint8)(parameterB * i + parameterA);
            // 更新neo数组
            neo_r_line_x[i] = (uint8)(parameterB * i + parameterA);
        }
    }
}
/****************************END*****************************************/

// 环岛检测
void CircleTest(uint8 type)
{                  // 1左，2右
    if (type == 1) // 左环岛检测
    {
        uint8 y1;
        for (uint8 i = MT9V03X_H - 1; i > MT9V03X_H - 20; i--)
        {
            if (l_line_x[i] == 0 && l_line_x[i - 1] == 0) // 跨过下方黑块
            {
                y1 = i;
                break;
            }
        }
        for (uint8 i = y1 - 10; i > 10; i--) // y1 -10
        {
            if (image_01[i][0] == 0 && image_01[i - 1][0] == 0 && image_01[i - 2][0] == 0) // 找到上方黑点
            {
                y1 = i;
                break;
            }
        } // 找到黑色圆环的起始高度坐标

        for (uint8 i = y1; y1 > 10; i--)
        {
            if ((l_line_x[y1 - 2] - l_line_x[y1] < 10) && (l_line_x[y1 - 4] - l_line_x[y1] < 20) && (l_line_x[y1 - 6] - l_line_x[y1] < 30)) // 利用圆环横坐标增加没有十字快的特征（猜想）
            {
                road_type.straight = 0;
                road_type.bend = 0;
                road_type.Ramp = 0;
                //                road_type.Cross         = 0;
                road_type.L_Cross = 0;
                road_type.R_Cross = 0;
                road_type.LeftCirque = 1;
                road_type.RightCirque = 0;
                //                road_type.Fork_in       = 0;
                //                road_type.Fork_on       = 0;
                //                road_type.Fork_out      = 0;
                road_type.Barn_l_out = 0;
                road_type.Barn_r_out = 0;
                road_type.Barn_l_in = 0;
                road_type.Barn_r_in = 0;
            }
        }
    }
}
/****************************END*****************************************/

// 判断丢线行位置和数目
uint8 diuxian_hang = 0, budiuxian_hang = 0; // 初始化丢线行和未丢线行的计数器
void Diuxian_weizhi_test(uint8 type, uint8 startline, uint8 endline)
{                                         // 1左2右
    diuxian_hang = 0, budiuxian_hang = 0; // 函数开始时，重新初始化计数器
    if (type == 1)
    {
        for (uint8 y = startline; y >= endline; y--) // 从startline到endline反向遍历行
        {
            budiuxian_hang += l_search_flag[y];
            diuxian_hang = (startline - endline + 1) - budiuxian_hang; // 计算丢线行数目
        }
    }
    if (type == 2)
    {
        for (uint8 y = startline; y >= endline; y--)
        {
            budiuxian_hang += r_search_flag[y];
            diuxian_hang = (startline - endline + 1) - budiuxian_hang;
        }
    }
}
/****************************END*****************************************/

// 找边线拐点
uint8 l_guaidain_x1 = 0, l_guaidain_y1 = 0; // 一拐点
uint8 r_guaidain_x1 = 0, r_guaidain_y1 = 0;
uint8 l_guaidain_x2 = 0, l_guaidain_y2 = 0; // 二拐点
uint8 r_guaidain_x2 = 0, r_guaidain_y2 = 0;
uint8 Bianxian_guaidian_num(uint8 type, uint8 startline, uint8 endline)
{
    uint8 has_guaidian = 0; // 返回值，0表示没有拐点，1表示有拐点

    // 初始化拐点坐标
    l_guaidain_x1 = 0;
    l_guaidain_y1 = 0;
    r_guaidain_x1 = 0;
    r_guaidain_y1 = 0;

    // 不再需要第二个拐点的变量，但保留初始化以防止其他函数访问时出错
    l_guaidain_x2 = 0;
    l_guaidain_y2 = 0;
    r_guaidain_x2 = 0;
    r_guaidain_y2 = 0;

    // 左边线拐点检测
    if (type == 1)
    {
        // 搜索左边线拐点
        for (uint8 y = startline; y > endline; y--)
        {
            if (fabs(l_line_x[y] - l_line_x[y - 1]) < 4 && (l_line_x[y] - l_line_x[y + 10] > 10))
            {
                l_guaidain_x1 = l_line_x[y - 5];
                l_guaidain_y1 = y - 5;
                has_guaidian = 1; // 找到拐点
                break;
            }
        }
    }

    // 右边线拐点检测
    else if (type == 2)
    {
        // 搜索右边线拐点
        for (uint8 y = startline; y > endline; y--)
        {
            if (fabs(r_line_x[y] - r_line_x[y + 1]) < 4 && (r_line_x[y] - r_line_x[y + 10] < -10)) // 这个条件要更加严格一点
            {
                r_guaidain_x1 = r_line_x[y];
                r_guaidain_y1 = y;
                has_guaidian = 1; // 找到拐点
                break;
            }
        }
    }

    return has_guaidian; // 返回是否找到拐点
}
/****************************END*****************************************/

uint8 l_guaidain_x = 0, l_guaidain_y = 0;
uint8 r_guaidain_x = 0, r_guaidain_y = 0;
void Bianxian_guaidian(uint8 type, uint8 startline, uint8 endline)
{ // 每边只有一个拐点的情况
    l_guaidain_x = 0;
    l_guaidain_y = 0;
    r_guaidain_x = 0;
    r_guaidain_y = 0;

    // 左边线拐点
    if (type == 1)
    {
        if (l_line_x[MT9V03X_H - 1] != 0) // 左下拐点存在
        {
            for (uint8 y = startline; y > endline; y--)
            {
                if (fabs(l_line_x[y] - l_line_x[y + 1]) < 4 && (l_line_x[y] - l_line_x[y - 3] > 8))
                {
                    l_guaidain_y = y;
                    l_guaidain_x = l_line_x[y];
                    break;
                }
            }
        }
    }

    // 右边线拐点
    if (type == 2)
    {
        if (r_line_x[MT9V03X_H - 1] != MT9V03X_W - 1) // 右下拐点存在
        {
            for (uint8 y = startline; y > endline; y--)
            {
                if (fabs(r_line_x[y] - r_line_x[y + 1]) < 4 && (r_line_x[y] - r_line_x[y - 3] < -8)) // 这个条件要更加严格一点
                {
                    r_guaidain_y = y;
                    r_guaidain_x = r_line_x[y];
                    break;
                }
            }
        }
    }
}
/****************************END*****************************************/

// 获取圆环的弧线
//-------------------------------------------------------------------------------------------------------------------
//  @param      type  模式
//  @param      num    判断的强度
uint8 bianxian_guaidian_l = 0; // 存储圆环弧线的位置
uint8 bianxian_guaidian_r = 0;
uint8 RoundaboutGetArc(uint8 type, uint8 num)
{                           // array待处理数组，type模式，num判断的强度
    uint8 inc = 0, dec = 0; // 用于记录向右和向左延申的数量
    switch (type)           // 左边线
    {
    case 1: // 从图像的最低行开始（MT9V03X_H - 1），向上遍历，直到第11行
        for (uint8 i = MT9V03X_H - 1; i > 10; i--)
        {                                                 // 检查当前行和上一行的左边线是否都存在，即未丢线
            if (l_line_x[i] != 0 && l_line_x[i - 1] != 0) // 最低行未丢线
            {
                if (inc < num)
                {
                    if (l_line_x[i] < l_line_x[i - 1]) // 左边线向右延申
                    {
                        inc++;
                    }
                }
                else
                {
                    if (l_line_x[i] > l_line_x[i - 1]) // 左边线向左延申
                    {
                        dec++;
                    }
                }

                /* 有弧线 */
                if (inc >= num && dec >= num)
                {
                    bianxian_guaidian_l = i + num;
                    return 1;
                }
            }
            else
            {
                inc = 0;
                dec = 0;
            }
        }
        break;

    case 2: // 右边线
        for (uint8 i = MT9V03X_H - 1; i > 10; i--)
        {
            if (r_line_x[i] != MT9V03X_W - 1 && r_line_x[i - 1] != MT9V03X_W - 1) // 最低行未丢线
            {
                if (inc < num)
                {
                    if (r_line_x[i] > r_line_x[i - 1]) // 右边线向左延申
                    {
                        inc++;
                    }
                }
                else
                {
                    if (r_line_x[i] < r_line_x[i - 1]) // 右边线向右延申
                    {
                        dec++;
                    }
                }
                /* 有弧线 */
                if (inc >= num && dec >= num)
                { // 如果向左和向右的延伸都达到了指定数量 num，则说明存在圆弧线
                    bianxian_guaidian_r = i + num;
                    return 1;
                }
            }
            else // 如果当前行或上一行的右边线丢失，重置 inc 和 dec
            {
                inc = 0;
                dec = 0;
            }
        }
        break; // 跳出 switch 语句
    }

    return 0; // 没有找到圆弧线，则返回0
}
/****************************END*****************************************/

// 三岔找V字底点
uint8 sancha_x = 0, sancha_x_zhengque = 0;
uint8 sancha_y = 0, sancha_y_zhengque = 0;
void Sancha_didian()
{
    float k1, k2;
    sancha_x = 0;
    sancha_y = 0;

    if (sancha_y_zhengque < 60)
    {
        k1 = 0.35;
        k2 = 0.65;
    }
    else
    {
        k1 = 0.1;
        k2 = 0.9;
    }

    for (uint8 x = k1 * MT9V03X_W; x < k2 * MT9V03X_W; x++)
    {
        for (uint8 y = MT9V03X_H - 1; y > 10; y--)
        {
            if (image_01[y][x] == 0 && image_01[y - 1][x] == 0 && image_01[y - 2][x] == 0)
            {
                sancha_x = x;
                sancha_y = y;
                break;
            }
        }
        if (sancha_y > sancha_y_zhengque)
        {
            sancha_y_zhengque = sancha_y;
            sancha_x_zhengque = sancha_x;
        }
    }
}
/****************************END*****************************************/

// 计算前瞻量(中线)
uint8 qianzhang = 0;
void Qianzhang(void) // 不够用再改
{
    for (uint8 y = MT9V03X_H - 1; y > 10; y--)
    {
        if (image_01[y][MT9V03X_W / 2] == 0 && image_01[y - 1][MT9V03X_W / 2] == 0 && image_01[y - 2][MT9V03X_W / 2] == 0)
        {
            qianzhang = y;
        }
    }
}
/****************************END*****************************************/

uint8 Num = 0; // 存储计数结果
// 判断圆环还是十字（利用圆环的第一段圆弧）
uint8 Cirque_or_Cross(uint8 type, uint8 startline)
{
    uint8 num = 0;

    // 防止startline越界
    if (startline >= MT9V03X_H - 15)
        startline = MT9V03X_H - 16;

    if (type == 1) // 左圆环
    {
        // 扩大检测范围
        for (uint8 y = startline; y < startline + 25 && y < MT9V03X_H - 1; y++)
        {
            for (uint8 x = l_line_x[y]; x > 0 && x > l_line_x[y] - 50; x--) // 限制检测范围
            {
                if (image_01[y][x] != 0)
                {
                    num++;
                }
            }
        }
    }
    else if (type == 2) // 右圆环
    {
        // 扩大检测范围
        for (uint8 y = startline; y < startline + 25 && y < MT9V03X_H - 1; y++)
        {
            for (uint8 x = r_line_x[y]; x < MT9V03X_W && x < r_line_x[y] + 50; x++)
            {
                if (image_01[y][x] != 0)
                {
                    num++;
                }
            }
        }
    }

    Num = num; // 保存计数结果
    return num;
}
/*********************END*****************************************/

extern uint8 pushed_box_in_roundabout;

// 处理左环岛
uint8 huandao_stage = 1; // 环岛阶段标志位
uint8 point;
void Handle_Left_Cirque()
{
    if (pause_camera_process)
    {
        return;
    }

    // 检查是否在car_haha模式下，如果是且不在环岛内状态，强制进入环岛内
    if (in_car_haha_mode && huandao_stage < 5)
    {
        huandao_stage = 5; // 强制进入环岛内状态
        annulus_s1 = 0;    // 重置距离积分
    }

    // 判断在圆环的位置
    switch (huandao_stage)
    {
    case 1:
        Diuxian_weizhi_test(1, MT9V03X_H - 1, MT9V03X_H - 20);
        if (budiuxian_hang >= 1)
        {
            Diuxian_weizhi_test(1, MT9V03X_H - 20, MT9V03X_H - 50);
            if (diuxian_hang >= 10)
            {
                huandao_stage = 2; // 黑段加白段（1阶段）（补线直行）
            }
        }
        break;
    case 2:
        Diuxian_weizhi_test(1, MT9V03X_H - 1, MT9V03X_H - 30);
        if (diuxian_hang >= 20 && huandao_stage == 2)
        {
            Diuxian_weizhi_test(1, MT9V03X_H - 30, MT9V03X_H - 70);
            if (budiuxian_hang >= 30)
            {
                huandao_stage = 3; // 白段加黑段（2阶段）（补线直行）
            }
        }
        break;
    case 3:
        Diuxian_weizhi_test(1, MT9V03X_H - 1, MT9V03X_H - 30);
        point = Bianxian_guaidian_num(1, MT9V03X_H - 20, 30);
        if (budiuxian_hang >= 10 && huandao_stage == 3)
        {
            Diuxian_weizhi_test(1, MT9V03X_H - 20, MT9V03X_H - 60);
            if (diuxian_hang >= 25)
            {
                huandao_stage = 4; // 黑段加白段（3阶段）（拉线进环）
            }
            else if (point != 0) // 左边拐点存在
            {
                huandao_stage = 4; // 进入case 4
            }
        }

        break;
    case 4:
        annulus_s1 += Distance_Measure();
        if (annulus_s1 > 1000) // 编码器积分大于一个值，进入环岛里面（这个值可以给小一些，车身路径不好）
        {
            huandao_stage = 5; // 进入圆环内
            annulus_s1 = 0;
        }
        break;
    case 5:
        annulus_s1 += Distance_Measure();
        // 根据是否在car_haha模式下调整阈值
        if (annulus_s1 > (in_car_haha_mode ? 300 : 700)) // 在搬运模式下使用较小值
        {
            Diuxian_weizhi_test(1, MT9V03X_H - 70, MT9V03X_H - 90); // 40 60
            if (diuxian_hang >= 15 && huandao_stage == 5)
            {
                Diuxian_weizhi_test(2, MT9V03X_H - 70, MT9V03X_H - 90);
                if (diuxian_hang >= 10)
                {
                    huandao_stage = 6; // 出环
                }
            }
        }
        break;
    case 6:
        annulus_s1 += Distance_Measure();

        // 检查从中间到最下边的右边线是否有丢线
        uint8 right_line_no_lost = 1; // 默认为没有丢线
        for (uint8 y = MT9V03X_H / 2; y < MT9V03X_H; y++)
        {
            if (r_line_x[y] == MT9V03X_W - 1)
            { // 发现丢线
                right_line_no_lost = 0;
                break;
            }
        }

        // 检测左边是否有拐点，类似case 3的拐点检测
        uint8 left_corner_point = Bianxian_guaidian_num(1, MT9V03X_H - 20, 30);

        // 使用静态变量跟踪连续满足条件的次数
        static uint8 left_valid_condition_count = 0;

        // 检查当前帧是否满足条件 - 右边线没有丢线且左边有拐点
        if (left_corner_point)
        {
            left_valid_condition_count++;
        }
        else
        {
            left_valid_condition_count = 0; // 重置计数器
        }

        // 满足条件时进入case 7 - 需要连续5次满足或距离达到阈值
        if (((left_valid_condition_count >= 2) && annulus_s1 > 500)|| annulus_s1 > (in_car_haha_mode ? 30000 : 1200))
        {
            huandao_stage = 7;
            annulus_s1 = 0;
            left_valid_condition_count = 0; // 重置计数器
        }
        break;
    case 7:
        annulus_s1 += Distance_Measure();
        if (annulus_s1 > 1000) // 积分一段距离并补线环岛处理结束(所有标志位清零)
        {
            Clear_Supplement_Lines(); // 清除补线和恢复neo线
            road_type.LeftCirque = 0;
            huandao_stage = 1;
            annulus_s1 = 0;
            l_guaidain_x = 0, l_guaidain_y = 0;
            r_guaidain_x = 0, r_guaidain_y = 0;
            BUZZER_OFF;
            // 重置car_haha模式标志
            in_car_haha_mode = 0;
            // 重置环岛中推箱子标志
            pushed_box_in_roundabout = 0;
            disable_element_detection = 0; // 恢复元素检测
        }
        break;
    }

    // 处理
    if (huandao_stage == 1) // 半宽补线补左线
    {
        for (uint8 y = MT9V03X_H - 10; y >= MT9V03X_H - 90; y--) // 只有80行的赛道宽度，够用了
        {
            // 计算补线位置
            uint8 new_l_x = r_line_x[y] - (kuandu_saidao[MT9V03X_H - 1 - y]) + 5;
            uint8 new_r_x = r_line_x[y];

            // 存入补线数组
            l_second_line_x[y] = new_l_x;
            r_second_line_x[y] = new_r_x;

            // 存入neo数组用于中线计算
            neo_l_line_x[y] = new_l_x;
            neo_r_line_x[y] = new_r_x;
        }
    }
    else if ((huandao_stage == 2) || (huandao_stage == 3))
    {
        for (uint8 y = MT9V03X_H - 1; y >= MT9V03X_H - 60; y--) // 只有80行的赛道宽度，够用了
        {
            // 计算补线位置
            uint8 new_l_x = r_line_x[y] - (kuandu_saidao[MT9V03X_H - 1 - y]) + 5;
            uint8 new_r_x = r_line_x[y];

            // 存入补线数组
            l_second_line_x[y] = new_l_x;
            r_second_line_x[y] = new_r_x;

            // 存入neo数组用于中线计算
            neo_l_line_x[y] = new_l_x;
            neo_r_line_x[y] = new_r_x;
        }
    }
    else if (huandao_stage == 4) // 赛道宽度补线
    {
        Bianxian_guaidian_num(1, 80, 20);
        uint8 right_point_y;
        // 如果找到左拐点
        if (l_guaidain_x1 != 0 && l_guaidain_y1 != 0)
        {
            // 计算从下往上1/3处的位置
            uint8 quarter_height = MT9V03X_H - 1 - (MT9V03X_H - 1 - 10) / 3;

            // 获取右边线上对应位置的点
            uint8 right_point_x = r_line_x[quarter_height] - 30;
            right_point_y = quarter_height;

            // 确保右边线上的点有效
            if (right_point_x != 0 && right_point_x < MT9V03X_W)
            {
                // 拉直线 - 从左拐点到右边线上的点
                La_zhixian(l_guaidain_x1, l_guaidain_y1, right_point_x, right_point_y, r_line_x);
            }
        }
        else
        {
            right_point_y = 10;
        }
        for (uint8 y = MT9V03X_H - 1; y >= right_point_y; y--)
        {
            uint8 new_r_x = r_line_x[y] - 35;
            r_second_line_x[y] = new_r_x;
            neo_r_line_x[y] = new_r_x;
        }
    }
    else if (huandao_stage == 5) // 在环岛
    {
        // 无需操作，但确保neo数组与原始数组同步
        for (uint8 y = MT9V03X_H - 1; y > 10; y--)
        {
            neo_l_line_x[y] = l_line_x[y];
            neo_r_line_x[y] = r_line_x[y];
        }
    }
    else if (huandao_stage == 6) // 出环岛
    {
        // 计算1/6的位置
        uint8 high = MT9V03X_H * 1 / 6;
        uint8 low = MT9V03X_H - 1;

        // 处理high到low之间的行
        for (uint8 y = high; y <= low; y++)
        {
            uint8 new_r_x = 128;
            r_second_line_x[y] = l_line_x[y] + kuandu_saidao[MT9V03X_H - 1 - y] - 20;
            neo_r_line_x[y] = new_r_x;
        }
    }
    else if (huandao_stage == 7)
    {
        for (uint8 y = MT9V03X_H - 1; y > 20; y--)
        {
            // 计算补线位置
            uint8 new_l_x = r_line_x[y] - kuandu_saidao[MT9V03X_H - 1 - y];
            uint8 new_r_x = r_line_x[y];

            // 存入补线数组
            l_second_line_x[y] = new_l_x;
            r_second_line_x[y] = new_r_x;

            // 存入neo数组用于中线计算
            neo_l_line_x[y] = new_l_x;
            neo_r_line_x[y] = new_r_x;
        }
    }
}
/*****************************左环岛处理结束*****************************************/

// 处理右环岛
void Handle_Right_Cirque()
{
    if (pause_camera_process)
    {
        return;
    }

    // 检查是否在car_haha模式下，如果是且不在环岛内状态，强制进入环岛内
    if (in_car_haha_mode && huandao_stage < 5)
    {
        huandao_stage = 5; // 强制进入环岛内状态
        annulus_s1 = 0;    // 重置距离积分
    }

    // 判断在圆环的位置
    switch (huandao_stage)
    {
    case 1:
        Diuxian_weizhi_test(2, MT9V03X_H - 1, MT9V03X_H - 20);
        if (budiuxian_hang >= 1)
        {
            Diuxian_weizhi_test(2, MT9V03X_H - 20, MT9V03X_H - 50);
            if (diuxian_hang >= 10)
            {
                huandao_stage = 2; // 黑段加白段（1阶段）（补线直行）
            }
        }
        break;
    case 2:
        Diuxian_weizhi_test(2, MT9V03X_H - 1, MT9V03X_H - 30);
        if (diuxian_hang >= 10 && huandao_stage == 2)
        {
            Diuxian_weizhi_test(2, MT9V03X_H - 30, MT9V03X_H - 70);
            if (budiuxian_hang >= 5)
            {
                huandao_stage = 3; // 白段加黑段（2阶段）（补线直行）
            }
        }
        break;
    case 3:
        Diuxian_weizhi_test(2, MT9V03X_H - 1, MT9V03X_H - 30);
        point = Bianxian_guaidian_num(2, MT9V03X_H - 20, 30);
        if (budiuxian_hang >= 10 && huandao_stage == 3)
        {
            Diuxian_weizhi_test(2, MT9V03X_H - 20, MT9V03X_H - 60);
            if (diuxian_hang >= 1)
            {
                huandao_stage = 4; // 黑段加白段（3阶段）（拉线进环）
            }
            else if (point != 0) // 左边拐点存在
            {
                huandao_stage = 4; // 进入case 4
            }
        }

        break;
    case 4:
        annulus_s1 += Distance_Measure();
        if (annulus_s1 > 1000) // 编码器积分大于一个值，进入环岛里面（这个值可以给小一些，车身路径不好）
        {
            huandao_stage = 5; // 进入圆环内
            annulus_s1 = 0;
        }
        break;
    case 5:
        annulus_s1 += Distance_Measure();
        // 根据是否在car_haha模式下调整阈值
        if (annulus_s1 > (in_car_haha_mode ? 200 : 500)) // 在搬运模式下使用较小值
        {
            Diuxian_weizhi_test(1, MT9V03X_H - 70, MT9V03X_H - 90); // 40 60
            if (diuxian_hang >= 15 && huandao_stage == 5)
            {
                Diuxian_weizhi_test(2, MT9V03X_H - 70, MT9V03X_H - 90);
                if (diuxian_hang >= 10)
                {
                    huandao_stage = 6; // 出环
                }
            }
        }
        break;
    case 6:
        annulus_s1 += Distance_Measure();

        // 检查从中间到最下边的左边线是否有丢线
        uint8 left_line_no_lost = 1; // 默认为没有丢线
        for (uint8 y = MT9V03X_H / 2; y < MT9V03X_H; y++)
        {
            if (l_line_x[y] == 0)
            { // 发现丢线
                left_line_no_lost = 0;
                break;
            }
        }

        // 检测右边是否有拐点，类似case 3的拐点检测
        uint8 right_corner_point = Bianxian_guaidian_num(2, MT9V03X_H - 20, 30);

        // 使用静态变量跟踪连续满足条件的次数
        static uint8 right_valid_condition_count = 0;

        // 检查当前帧是否满足条件 - 左边线没有丢线且右边有拐点
        if (right_corner_point)
        {
            right_valid_condition_count++;
        }
        else
        {
            right_valid_condition_count = 0; // 重置计数器
        }

        // 满足条件时进入case 7 - 需要连续5次满足或距离达到阈值
        if (((right_valid_condition_count >= 2) && annulus_s1 > 1500)|| annulus_s1 > (in_car_haha_mode ? 30000 : 1200))
        {
            huandao_stage = 7;
            annulus_s1 = 0;
            right_valid_condition_count = 0; // 重置计数器
        }
        break;
    case 7:
        annulus_s1 += Distance_Measure();
        if (annulus_s1 > 1000) // 积分一段距离并补线环岛处理结束(所有标志位清零)
        {
            Clear_Supplement_Lines(); // 清除补线和恢复neo线
            road_type.RightCirque = 0;
            huandao_stage = 1;
            annulus_s1 = 0;
            l_guaidain_x = 0, l_guaidain_y = 0;
            r_guaidain_x = 0, r_guaidain_y = 0;
            BUZZER_OFF;
            // 重置car_haha模式标志
            in_car_haha_mode = 0;
            // 重置环岛中推箱子标志
            pushed_box_in_roundabout = 0;
            disable_element_detection = 0; // 恢复元素检测
        }
        break;
    }

    // 处理
    if (huandao_stage == 1) // 半宽补线补右线
    {
        for (uint8 y = MT9V03X_H - 10; y >= MT9V03X_H - 90; y--) // 只有80行的赛道宽度，够用了
        {
            // 计算补线位置
            uint8 new_r_x = l_line_x[y] + (kuandu_saidao[MT9V03X_H - 1 - y]) - 5;
            uint8 new_l_x = l_line_x[y];

            // 存入补线数组
            l_second_line_x[y] = new_l_x;
            r_second_line_x[y] = new_r_x;

            // 存入neo数组用于中线计算
            neo_l_line_x[y] = new_l_x;
            neo_r_line_x[y] = new_r_x;
        }
    }
    else if ((huandao_stage == 2) || (huandao_stage == 3))
    {
        for (uint8 y = MT9V03X_H - 1; y >= MT9V03X_H - 60; y--) // 只有80行的赛道宽度，够用了
        {
            // 计算补线位置
            uint8 new_r_x = l_line_x[y] + (kuandu_saidao[MT9V03X_H - 1 - y]) - 5;
            uint8 new_l_x = l_line_x[y];

            // 存入补线数组
            l_second_line_x[y] = new_l_x;
            r_second_line_x[y] = new_r_x;

            // 存入neo数组用于中线计算
            neo_l_line_x[y] = new_l_x;
            neo_r_line_x[y] = new_r_x;
        }
    }
    else if (huandao_stage == 4) // 赛道宽度补线
    {
        Bianxian_guaidian_num(2, 80, 20);
        uint8 left_point_y;
        // 如果找到左拐点
        if (r_guaidain_x1 != 0 && r_guaidain_y1 != 0)
        {
            // 计算从下往上1/3处的位置
            uint8 quarter_height = MT9V03X_H - 1 - (MT9V03X_H - 1 - 10) / 3;

            // 获取右边线上对应位置的点
            uint8 left_point_x = l_line_x[quarter_height] + 40;
            left_point_y = quarter_height;

            // 确保右边线上的点有效
            if (left_point_x != 0 && left_point_x < MT9V03X_W)
            {
                // 拉直线 - 从右拐点到左边线上的点
                La_zhixian(r_guaidain_x1, r_guaidain_y1, left_point_x, left_point_y, l_line_x);
            }
        }
        else
        {
            left_point_y = 10;
        }
        for (uint8 y = MT9V03X_H - 1; y >= left_point_y; y--)
        {
            uint8 new_l_x = l_line_x[y] + 45;
            l_second_line_x[y] = new_l_x;
            neo_l_line_x[y] = new_l_x;
        }
    }
    else if (huandao_stage == 5) // 在环岛
    {
        // 无需操作，但确保neo数组与原始数组同步
        for (uint8 y = MT9V03X_H - 1; y > 10; y--)
        {
            neo_l_line_x[y] = l_line_x[y];
            neo_r_line_x[y] = r_line_x[y];
        }
    }
    else if (huandao_stage == 6) // 出环岛
    {
        // 计算1/6的位置
        uint8 high = MT9V03X_H * 1 / 6;
        uint8 low = MT9V03X_H - 1;

        // 处理high到low之间的行
        for (uint8 y = high; y <= low; y++)
        {
            uint8 new_l_x = 70;
            l_second_line_x[y] = r_line_x[y] - kuandu_saidao[MT9V03X_H - 1 - y] - 20;
            neo_l_line_x[y] = new_l_x;
        }
    }
    else if (huandao_stage == 7)
    {
        for (uint8 y = MT9V03X_H - 1; y > 20; y--)
        {
            // 计算补线位置
            uint8 new_r_x = l_line_x[y] + kuandu_saidao[MT9V03X_H - 1 - y];
            uint8 new_l_x = l_line_x[y] + 15;

            // 存入补线数组
            l_second_line_x[y] = new_l_x;
            r_second_line_x[y] = new_r_x;

            // 存入neo数组用于中线计算
            neo_l_line_x[y] = new_l_x;
            neo_r_line_x[y] = new_r_x;
        }
    }
}
/*****************************右环岛处理结束*****************************************/

int16 fork_s1 = 0;
int16 fork_s2 = 0;
// 处理三岔
uint8 sancha_stage = 1; // 三岔阶段标志位
void Handle_Fork()
{
    // 判断在三岔口的位置
    switch (sancha_stage)
    {
    case 1:
        if (RoundaboutGetArc(1, 15) && RoundaboutGetArc(2, 15))
        {
            if (bianxian_guaidian_l >= 55 && bianxian_guaidian_r >= 55)
            {
                bianxian_guaidian_l = 0;
                bianxian_guaidian_r = 0;
                sancha_stage = 2;
            }
        }
        break;
    case 2:
        Sancha_didian();
        //            fork_s1 += Distance_Measure();
        if (fork_s1 >= 2500)
        {
            fork_s1 = 0;
            sancha_y_zhengque = 0;
            sancha_stage = 3;
        }
        break;
    case 3:
        if (RoundaboutGetArc(1, 10) && RoundaboutGetArc(2, 10))
        {
            if (bianxian_guaidian_l >= 50 && bianxian_guaidian_r >= 50) // 避免车身不稳
            {
                sancha_stage = 4;
            }
        }

        break;
    case 4:
        Sancha_didian();
        //            fork_s1 += Distance_Measure();
        if (fork_s1 >= 4000)
        {
            fork_s1 = 0;
            sancha_y_zhengque = 0;
            bianxian_guaidian_l = 0;
            bianxian_guaidian_r = 0;
            sancha_stage = 5;
        }
        break;
    }

    // 处理
    if (sancha_stage == 1)
    {
        ; // 无需操作
    }
    else if (sancha_stage == 2)
    {
        if (sancha_y_zhengque > 5)
        {
            if (sancha_y_zhengque > 80)
            {
                sancha_y_zhengque = 57;
            }
            //            l_line_x[y] = r_line_x[y] - kuandu_saidao[MT9V03X_H-1 -y];
            La_zhixian(MT9V03X_W - 1, MT9V03X_H - 1, sancha_x_zhengque, sancha_y_zhengque, r_line_x); // 拉右线
        }
    }
    else if (sancha_stage == 3)
    {
        ; // 无需操作
    }
    else if (sancha_stage == 4)
    {
        if (sancha_y_zhengque > 5)
        {
            if (sancha_y_zhengque > 80)
            {
                sancha_y_zhengque = 57;
            }
            La_zhixian(MT9V03X_W - 1, MT9V03X_H - 1, sancha_x_zhengque, sancha_y_zhengque, r_line_x); // 拉右线
        }
    }
    else if (sancha_stage == 5)
    {
        BUZZER_OFF;
        road_type.Fork = 0;
        sancha_stage = 1;
    }
}
/****************************三岔处理结束****************************************/
uint8 l_xieshizi_stage = 1; // 左斜十字状态机变量
uint8 r_xieshizi_stage = 1; // 右斜十字状态机变量
// 处理左斜十字
void Handle_L_Cross()
{
    // 添加保护：如果进入了环岛状态，退出斜十字处理
    if (road_type.LeftCirque || road_type.RightCirque)
    {
        road_type.L_Cross = 0;
        l_xieshizi_stage = 1;
        Clear_Supplement_Lines();
        return;
    }

    switch (l_xieshizi_stage)
    {
    case 1:
    {
        if (l_lose_value < 20)
        {
            l_xieshizi_stage = 2;
        }
    }
    break;
    case 2:
    {
        if (abs(l_lose_value - r_lose_value) < 20) // 前瞻量需要调
        {
            l_xieshizi_stage = 3;
        }
    }
    break;
    case 3:
    {
        //            annulus_s1 += Distance_Measure();
        if (annulus_s1 > 2000 || abs(l_lose_value - r_lose_value) < 10)
        {
            Clear_Supplement_Lines(); // 清除补线和恢复neo线
            road_type.L_Cross = 0;
            annulus_s1 = 0;
            l_xieshizi_stage = 1;
            r_guaidain_x = 0;
            r_guaidain_y = 0;
            l_guaidain_x = 0;
            l_guaidain_y = 0;
            BUZZER_OFF;
        }
    }
    break;
    }
    // 处理
    if (l_xieshizi_stage == 1 || l_xieshizi_stage == 3)
    {
        for (uint8 y = MT9V03X_H - 1; y > 20; y--)
        {
            // 计算补线坐标
            uint8 new_x = r_line_x[y] - kuandu_saidao[MT9V03X_H - 1 - y];

            // 存入补线数组
            l_second_line_x[y] = new_x;

            // 更新neo数组
            neo_l_line_x[y] = new_x;
        }
    }
    else if (l_xieshizi_stage == 2)
    {
        ; // 不做处理
    }
    //    else if(xieshizi_stage ==3)
    //    {
    //        if(r_line_x[MT9V03X_H -1] != 0)//右下拐点存在，取拐点横坐标
    //        {
    //            for(uint8 y=MT9V03X_H -1; y>30; y--)
    //            {
    //                if(fabs(r_line_x[y] -r_line_x[y+1])<4 && (r_line_x[y]- r_line_x[y-6] <-8))
    //                {
    //                    r_guaidain_x = r_line_x[y];
    //                    r_guaidain_y = y;
    //                    break;
    //                }
    //            }
    //        }
    //        else
    //        {
    //            r_guaidain_x = MT9V03X_W-5;
    //            r_guaidain_y = 60;
    //        }
    //
    //        for(uint8 y = (r_guaidain_y-10); y>10; y--)
    //        {
    //            if(image_01[y][r_guaidain_x] ==0 && image_01[y-1][r_guaidain_x]==0 && image_01[y-2][r_guaidain_x]==0)
    //            {
    //                r_guaidain_y = y;
    //                break;
    //            }
    //        }
    //        La_zhixian(1, MT9V03X_H -1, r_guaidain_x, r_guaidain_y, l_line_x);
    //    }
}
/****************************左斜十字处理结束*****************************************/

// 处理右斜十字
void Handle_R_Cross()
{
    switch (r_xieshizi_stage)
    {
        // 添加保护：如果进入了环岛状态，退出斜十字处理
        if (road_type.LeftCirque || road_type.RightCirque)
        {
            road_type.R_Cross = 0;
            r_xieshizi_stage = 1;
            Clear_Supplement_Lines();
            return;
        }
    case 1:
    {
        if (r_lose_value < 20)
        {
            r_xieshizi_stage = 2;
        }
    }
    break;
    case 2:
    {
        if (abs(l_lose_value - r_lose_value) < 20) // 前瞻量需要调
        {
            r_xieshizi_stage = 3;
        }
    }
    break;
    case 3:
    {
        //            annulus_s1 += Distance_Measure();
        if (annulus_s1 > 2000 || abs(l_lose_value - r_lose_value) < 10)
        {
            Clear_Supplement_Lines(); // 清除补线和恢复neo线
            road_type.R_Cross = 0;
            annulus_s1 = 0;
            r_xieshizi_stage = 1;
            r_guaidain_x = 0;
            r_guaidain_y = 0;
            l_guaidain_x = 0;
            l_guaidain_y = 0;
            BUZZER_OFF;
        }
    }
    break;
    }
    // 处理
    if (r_xieshizi_stage == 1 || r_xieshizi_stage == 3)
    {
        for (uint8 y = MT9V03X_H - 1; y > 20; y--)
        {

            // 计算补线坐标
            uint8 new_x = l_line_x[y] + kuandu_saidao[MT9V03X_H - 1 - y];

            // 存入补线数组
            r_second_line_x[y] = new_x;

            // 更新neo数组
            neo_r_line_x[y] = new_x;
        }
    }
    else if (r_xieshizi_stage == 2)
    {
        ; // 不做处理
    }
    //        else if(xieshizi_stage ==3)
    //        {
    //            if(l_line_x[MT9V03X_H -1] != 0)//右下拐点存在，取拐点横坐标
    //            {
    //                for(uint8 y=MT9V03X_H -1; y>30; y--)
    //                {
    //                    if(fabs(l_line_x[y] -l_line_x[y+1])<4 && (l_line_x[y]- l_line_x[y-6] > 8))
    //                    {
    //                        l_guaidain_x = l_line_x[y];
    //                        l_guaidain_y = y;
    //                        break;
    //                    }
    //                }
    //            }
    //            else
    //            {
    //                l_guaidain_x = 5;
    //                l_guaidain_y = 60;
    //            }
    //
    //            for(uint8 y = (r_guaidain_y-10); y>10; y--)
    //            {
    //                if(image_01[y][l_guaidain_x] ==0 && image_01[y-1][l_guaidain_x]==0 && image_01[y-2][l_guaidain_x]==0)
    //                {
    //                    l_guaidain_y = y;
    //                    break;
    //                }
    //            }
    //            La_zhixian(MT9V03X_W-1, MT9V03X_H -1, l_guaidain_x, l_guaidain_y, r_line_x);
    //        }
}
/****************************右斜十字处理结束*****************************************/

int16 cross_s = 0;
uint8 cross_stage = 1; // 十字阶段标志位

void Handle_Cross(void)
{
    // 判断是否需要退出十字路口处理
    if (cross_s > 2000)
    {
        cross_s = 0;
        road_type.Cross = 0;
        cross_stage = 1; // 重置十字阶段
    }

    // 防止与环岛状态机冲突 - 添加下面的保护代码
    if (road_type.LeftCirque || road_type.RightCirque)
    {
        // 如果已经进入环岛状态，就不执行十字处理
        road_type.Cross = 0;
        return;
    }

    // 更新拐点信息
    Check_Cross_Guaidian(1); // 检测左侧拐点
    Check_Cross_Guaidian(2); // 检测右侧拐点

    // 根据十字路口阶段进行处理
    switch (cross_stage)
    {
    case 1: // 两边都有拐点的情况
        // 检查是否两边都有下拐点
        if (cross_left[0] != 0 && cross_right[0] != 0)
        {
            // 左侧拐点处理：左下连接左上
            if (cross_left[1] != 0 && l_line_x[cross_left[1]] > 0)
            {
                La_zhixian(l_line_x[cross_left[0]], cross_left[0],
                           l_line_x[cross_left[1]], cross_left[1], l_line_x);
            }

            // 右侧拐点处理：右下连接右上
            if (cross_right[1] != 0 && r_line_x[cross_right[1]] < MT9V03X_W - 1)
            {
                La_zhixian(r_line_x[cross_right[0]], cross_right[0],
                           r_line_x[cross_right[1]], cross_right[1], r_line_x);
            }
        }
        else
        {
            // 如果左下拐点或右下拐点消失，进入case 2
            cross_stage = 2;
        }
        break;

    case 2: // 下拐点消失的情况
        // 左侧拐点处理：左上拐点拉到左下
        if (cross_left[1] != 0 && l_line_x[cross_left[1]] > 0)
        {
            // 拉线到屏幕左下角
            La_zhixian(1, MT9V03X_H - 1, l_line_x[cross_left[1]], cross_left[1], l_line_x);
        }

        // 右侧拐点处理：右上拐点拉到右下
        if (cross_right[1] != 0 && r_line_x[cross_right[1]] < MT9V03X_W - 1)
        {
            // 拉线到屏幕右下角
            La_zhixian(MT9V03X_W - 2, MT9V03X_H - 1, r_line_x[cross_right[1]], cross_right[1], r_line_x);
        }

        // 检测是否两边都有下拐点了，如果有就回到case 1
        if (cross_left[0] != 0 && cross_right[0] != 0)
        {
            cross_stage = 1;
        }
        break;
    }

    // 检测是否要退出十字处理
    // 再次检测丢线情况
    Diuxian_weizhi_test(1, MT9V03X_H - 1, 5);
    Diuxian_weizhi_test(2, MT9V03X_H - 1, 5);

    // 如果丢线情况减少或拐点消失，退出十字处理
    if ((l_lose_value < 5 && r_lose_value < 5) ||    // 丢线减少
        (cross_left[1] == 0 && cross_right[1] == 0)) // 上拐点都消失
    {
        cross_stage = 1;          // 退回到十字检测阶段
        road_type.Cross = 0;      // 清除十字标记
        Clear_Supplement_Lines(); // 清除补线和恢复neo线
    }

    // 添加保护代码：如果右侧丢线特别多，但仍在十字中，确保不进入环岛状态
    if (r_lose_value > 30)
    {
        // 强制清除可能的环岛标志
        road_type.LeftCirque = 0;
        road_type.RightCirque = 0;
        huandao_stage = 1; // 重置环岛状态机
    }
}

uint8 cross_left[2] = {0, 0}; // 第一个下标存图像下方的点的y值
uint8 cross_right[2] = {0, 0};
void Check_Cross_Guaidian(uint8 type) // 找十字拐点
{
    // 确保先清零，避免使用旧值
    if (type == 1)
    {
        cross_left[0] = 0;
        cross_left[1] = 0;
    }
    else if (type == 2)
    {
        cross_right[0] = 0;
        cross_right[1] = 0;
    }

    uint8 mid_col = MT9V03X_W / 2; // 图像中间列

    if (type == 1) // 左边拐点检测
    {
        // 检测左下拐点 - 从中间向左寻找
        for (uint8 y1 = MT9V03X_H - 15; y1 >= 70; y1--)
        {
            for (uint8 x = mid_col; x > 5; x--) // 从中间列开始向左搜索
            {
                if (l_line_x[y1] == x &&                         // 如果该行左边线位置就在当前x
                    fabs(l_line_x[y1] - l_line_x[y1 + 1]) < 4 && // 与下一行左边线位置接近
                    (l_line_x[y1] - l_line_x[y1 - 5]) > 10)      // 向上偏移明显
                {
                    cross_left[0] = y1;
                    break;
                }
            }
            if (cross_left[0] != 0)
                break; // 如果找到了拐点，跳出循环
        }

        // 寻找左上拐点 - 从中间向左寻找
        for (uint8 y2 = 25; y2 < (cross_left[0] ? cross_left[0] - 15 : MT9V03X_H - 25); y2++)
        {
            for (uint8 x = mid_col; x > 5; x--) // 从中间列开始向左搜索
            {
                if (l_line_x[y2] == x && // 如果该行左边线位置就在当前x
                    ((fabs(l_line_x[y2] - l_line_x[y2 - 1]) < 4 && (l_line_x[y2] - l_line_x[y2 + 5]) > 20) ||
                     (fabs(l_line_x[y2] - l_line_x[y2 - 1]) < 4 && l_line_x[y2 + 3] == 0)))
                {
                    cross_left[1] = y2 + 2;
                    break;
                }
            }
            if (cross_left[1] != 0)
                break; // 如果找到了拐点，跳出循环
        }
    }
    else if (type == 2) // 右边拐点检测
    {
        // 检测右下拐点 - 从中间向右寻找
        for (uint8 y1 = MT9V03X_H - 15; y1 >= 70; y1--)
        {
            for (uint8 x = mid_col; x < MT9V03X_W - 5; x++) // 从中间列开始向右搜索
            {
                if (r_line_x[y1] == x &&                         // 如果该行右边线位置就在当前x
                    fabs(r_line_x[y1] - r_line_x[y1 + 1]) < 4 && // 与下一行右边线位置接近
                    (r_line_x[y1] - r_line_x[y1 - 5]) < -10)     // 向上偏移明显
                {
                    cross_right[0] = y1;
                    break;
                }
            }
            if (cross_right[0] != 0)
                break; // 如果找到了拐点，跳出循环
        }

        // 寻找右上拐点 - 从中间向右寻找
        for (uint8 y2 = 25; y2 < (cross_right[0] ? cross_right[0] - 15 : MT9V03X_H - 25); y2++)
        {
            for (uint8 x = mid_col; x < MT9V03X_W - 5; x++) // 从中间列开始向右搜索
            {
                if (r_line_x[y2] == x && // 如果该行右边线位置就在当前x
                    ((r_line_x[y2] != MT9V03X_W - 1 && fabs(r_line_x[y2] - r_line_x[y2 - 1]) < 4 && (r_line_x[y2] - r_line_x[y2 + 5]) < -20) ||
                     (fabs(r_line_x[y2] - r_line_x[y2 - 10]) < 4 && r_line_x[y2 + 3] == 0)))
                {
                    cross_right[1] = y2;
                    break;
                }
            }
            if (cross_right[1] != 0)
                break; // 如果找到了拐点，跳出循环
        }
    }
}

/****************************十字处理结束*****************************************/

////出库函数
// uint8 Garage_T=0;  //出库计时
// void Handle_Barn_Out(uint8 type)
//{//1左2右
//     if(type ==1)
//     {
//         while (Garage_T <40)//直走
//         {
//             Set_Steer(0);
//             Set_Motor(3000, 3000);
//             Garage_T++;
//             systick_delay_ms(STM0,10);
//         }
//         while (Garage_T<80)//转弯
//         {
//             Set_Steer(STEER_LIM);
//             Set_Motor(3000, 3000);
//             Garage_T++;
//             systick_delay_ms(STM0,10);
//         }
//         if(Garage_T>=80)   //出库结束,标志位清零
//         {
//             Garage_T =0;
//             flag.start =1;
//         }
//     }

//    else if(type ==2)
//    {
//        while (Garage_T <40)//直走
//        {
//            Set_Steer(0);
//            Set_Motor(3000, 3000);
//            Garage_T++;
//            systick_delay_ms(STM0,10);
//        }
//        while (Garage_T <80)   //转弯
//        {
//            Set_Steer(-STEER_LIM);
//            Set_Motor(3000, 3000);
//            Garage_T++;
//            systick_delay_ms(STM0,10);
//        }
//        if(Garage_T>=80)   //出库结束,标志位清零
//        {
//            Set_Steer(0);
//            Garage_T =0;
//            flag.start =1;
//        }
//    }
//}

int8 pass_barn = 1;
int16 close_check_ku_s = 0;
int16 jinku_s = 0;
// 入库函数
void Handle_Barn_in(uint8 type)
{ // 1左入库,2右入库
    if (type == 1)
    {
        if (pass_barn == 1) // 第一次经过库
        {
            for (uint8 y = MT9V03X_H - 1; y > 20; y--)
            {
                l_line_x[y] = r_line_x[y] - kuandu_saidao[MT9V03X_H - 1 - y]; // 补左线
            }
            //            close_check_ku_s += Distance_Measure();
            if (close_check_ku_s > 4000) // 积分一段距离，打开库检测标志位
            {
                pass_barn++;
                road_type.Barn_l_in = 0;
                flag.open_check_ku = 1;
                close_check_ku_s = 0;
            }
        }
        else if (pass_barn == 2) // 第二次经过库
        {
            //            jinku_s += Distance_Measure();
            if (jinku_s > 4000) // 积分一段距离停车
            {
                flag.stop = 1;
                jinku_s = 0;
            }
        }
    }

    else if (type == 2)
    {
        if (pass_barn == 1) // 第一次经过库
        {
            for (uint8 y = MT9V03X_H - 1; y > 20; y--)
            {
                r_line_x[y] = l_line_x[y] + kuandu_saidao[MT9V03X_H - 1 - y]; // 补右线
            }
            //            close_check_ku_s += Distance_Measure();
            if (close_check_ku_s > 4000) // 积分一段距离，打开库检测标志位
            {
                pass_barn++;
                road_type.Barn_r_in = 0;
                flag.open_check_ku = 1;
                close_check_ku_s = 0;
            }
        }
        else if (pass_barn == 2) // 第二次经过库
        {
            //            jinku_s += Distance_Measure();
            if (jinku_s > 4000) // 积分一段距离停车
            {
                flag.stop = 1;
                jinku_s = 0;
            }
        }
    }
}

uint8 length = 0; // 存储计算得到的长度值
void  Mid_Col(void)
{
    int i;
    for (i = MT9V03X_H; i > 1; i--) // 从图像的底部开始向上遍历
    {                               // 在每一行的第 80 列（假设该列为图像的中间列）进行判断，如果该列的当前行和上两行都是黑色（像素值为0），则跳出循环
        if (image_01[i][80] == 0 && image_01[i - 1][80] == 0 && image_01[i - 2][80] == 0)
        {
            break;
        }
    }
    length = MT9V03X_H - i; // 计算得到的长度值为图像的高度减去当前行号 i，赋值给全局变量 length
    //        return length;
}
// 出界停车
void Outside_protect(void)
{
    uint8 j = 0;

    for (uint8 i = l_line_x[MT9V03X_H - 1]; i < r_line_x[MT9V03X_H - 1]; i++) // 左边搜线
    {
        if (image_01[MT9V03X_H - 1][i] == 0)
        {
            j++;
            if (j > MT9V03X_H)
            {
                flag.stop = 1;
            }
        }
    }
}

int16 Sum1;
void HDPJ_lvbo(uint8 data[], uint8 N, uint8 size)
{
    Sum1 = 0;
    for (uint8 j = 0; j < size; j++)
    {
        if (j < N / 2)
        {
            for (uint8 k = 0; k < N; k++)
            {
                Sum1 += data[j + k];
            }
            data[j] = Sum1 / N;
        }
        else if (j < size - N / 2)
        {
            for (uint8 k = 0; k < N / 2; k++)
            {
                Sum1 += (data[j + k] + data[j - k]);
            }
            data[j] = Sum1 / N;
        }
        else
        {
            for (uint8 k = 0; k < size - j; k++)
            {
                Sum1 += data[j + k];
            }
            for (uint8 k = 0; k < (N - size + j); k++)
            {
                Sum1 += data[j - k];
            }
            data[j] = Sum1 / N;
        }
        Sum1 = 0;
    }
}

uint8 sudu_yingzi = 0;      // 速度因子
void Check_Zhidao(void)     // 用于检测直道的长度
{                           // 这里要考虑搜线终止行
    uint8 inc = 0, dec = 0; // 用于记录向左和向右延伸的像素数量
    sudu_yingzi = 0;
    // 从图像的最底部开始向上遍历，遍历到第2行
    for (uint8 y = MT9V03X_H - 1; y > 1; y--)
    {                                                             // 检查左边线是否不丢失，并且当前行的左边线位置不小于上一行的左边线位置
        if ((l_line_x[y] <= l_line_x[y - 1]) && l_line_x[y] != 0) // 两边不丢线才计算直道长度
        {
            inc++;
        }

        if ((r_line_x[y] >= r_line_x[y - 1]) && r_line_x[y] != MT9V03X_W - 1)
        {
            dec++;
        }
    }
    // 判断向左延伸的像素数量 inc 是否大于等于向右延伸的像素数量 dec
    if (inc >= dec)
    {
        sudu_yingzi = dec; // 直道的长度取 dec
    }
    else
    {
        sudu_yingzi = inc;
    }
}

uint8 Tututu(uint8 type)
{
    if (type == 1)
    {
        for (uint8 y1 = MT9V03X_H - 1; y1 >= 30; y1 = y1 - 4)
        {
            if (fabs(r_line_x[y1] - r_line_x[y1 - 1]) < 4 && (r_line_x[y1] - r_line_x[y1 - 10]) < -8) // 条件改严格，更难返回1
            {
                return 1;
            }
        }
    }
    else if (type == 2)
    {
        for (uint8 y1 = MT9V03X_H - 1; y1 >= 30; y1 = y1 - 4)
        {
            if (fabs(l_line_x[y1] - l_line_x[y1 - 1]) < 4 && (l_line_x[y1] - l_line_x[y1 - 10]) > 8) // 条件改严格，更难返回1
            {
                return 1;
            }
        }
    }
    return 0;
}

extern uint8 cross_stage;

// 新增的边线数组，用于保存包含补线的完整边线信息
uint8 neo_l_line_x[MT9V03X_H], neo_r_line_x[MT9V03X_H];

// 清除补线和恢复neo线的函数
void Clear_Supplement_Lines(void)
{
    for (uint8 y = 0; y < MT9V03X_H; y++)
    {
        // 清除补线数组
        l_second_line_x[y] = 0;
        r_second_line_x[y] = MT9V03X_W - 1;

        // 恢复neo数组与原始边线一致
        neo_l_line_x[y] = l_line_x[y];
        neo_r_line_x[y] = r_line_x[y];
    }
}
