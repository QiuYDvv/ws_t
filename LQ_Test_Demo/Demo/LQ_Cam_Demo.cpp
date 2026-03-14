#include "LQ_demo.hpp"

/*
        该例程是使用OpenCV以及USB摄像头进行实现，久久派
    中，默认是没有USB摄像头的驱动的，需自行配置。可参考龙
    邱B站视频教程进行配置。
    视频链接：https://www.bilibili.com/video/BV1zRKHeTEDm
*/

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void CameraDemo()
 * @功能说明：摄像头测试程序
 * @参数说明：无
 * @函数返回：无
 * @调用方法：CameraDemo();
 * @备注说明：该函数会将设置的帧率、像素大小以及后续获取的图像数据从串口打印
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void CameraDemo()
{
    TFTSPI_dri_init(0);
    // 创建一个摄像头
    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cerr << "Error open video stream" << endl;
        return;
    }
    // 设置视频流编码器
    cap.set(cv::CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // 设置摄像头图像宽高和帧率
    cap.set(CAP_PROP_FRAME_WIDTH, 160);
    cap.set(CAP_PROP_FRAME_HEIGHT, 120);
    cap.set(CAP_PROP_FPS, 120);
    // 获取摄像头图像宽高和帧率
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double frame_fps = cap.get(cv::CAP_PROP_FPS);
    printf("frame:%d*%d, fps:%3f", frame_width, frame_height, frame_fps);
    sleep(1);
    // 获取视频流
    Mat frame;
    uint8_t data[120][160];
    printf("data sizeof is %d", sizeof(data));
    while (1)
    {
        // 读取摄像头一帧图像
        cap.read(frame);    // cap >> frame;
        if (frame.empty())
        {
            cerr << "Error read frame" << endl;
            break;
        }
        // 转化为灰度
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        // 二值化处理
        Mat binary;
        threshold(gray, binary, 127, 255, THRESH_BINARY);
        // 获取图像数据
        for (int i = 0; i < 120; i++)
        {
            for (int j = 0; j < 160; j++)
            {
                // data[i][j] = gray.at<unsigned char>(i, j);
                data[i][j] = binary.at<unsigned char>(i, j);
            }
        }
        // TFTSPI_dir_road(0, 0, 120, 160, (uint8_t*)data);
        TFTSPI_dir_binRoad(0, 0, 120, 160, (uint8_t*)data);
    }
} 
