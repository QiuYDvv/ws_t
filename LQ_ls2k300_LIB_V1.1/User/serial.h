#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>
#include <termios.h>

// 串口模块公共接口：
// 负责对龙邱串口对象做 RAII 封装，并提供文本、原始字节和 VOFA 浮点帧发送接口。

// 前向声明，避免 serial.cpp 外暴露龙邱头文件
struct LS_UART;

// 串口与上位机通信封装类，基于龙邱 LQ_Uart（LS_UART）库
class Serial
{
public:
    Serial() = default;
    ~Serial();

    Serial(const Serial&) = delete;
    Serial& operator=(const Serial&) = delete;

    // 打开串口，device 如 "/dev/ttyS1"，baudrate 使用 termios 宏如 B115200
    // 返回 true 表示打开成功
    bool open(const std::string& device, speed_t baudrate);

    void close();

    bool isOpen() const { return uart_ != nullptr; }

    // 发送数据，返回实际发送字节数，失败返回 -1
    int send(const char* buf, size_t len);
    int send(const uint8_t* buf, size_t len);

    // 接收数据，返回实际读取字节数，无数据或失败返回 0 或 -1
    int receive(char* buf, size_t maxLen);
    int receive(uint8_t* buf, size_t maxLen);

    // 获取底层串口 fd；失败返回 -1
    int nativeHandle() const;

    // 等待串口可读；超时或失败返回 false
    bool waitReadable(int timeoutMs) const;

    // 发送字符串（不含结尾 '\0'）
    int sendStr(const char* str);
    int sendStr(const std::string& str);

    // VOFA+ JustFloat 协议：发送一帧浮点数据，上位机选择 JustFloat 引擎即可绘图
    // data 为小端浮点数组，count 为浮点个数；帧尾自动添加 0x00 0x00 0x80 0x7f
    int sendVofaJustFloat(const float* data, int count);

private:
    LS_UART* uart_ = nullptr;
};
