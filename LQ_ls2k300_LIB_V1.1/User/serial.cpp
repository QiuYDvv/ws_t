#include "serial.h"
#include "LQ_Uart.hpp"

#include <cstring>

Serial::~Serial()
{
    close();
}

bool Serial::open(const std::string& device, speed_t baudrate)
{
    close();
    // 直接调用龙邱库：LS_UART 带参构造函数完成打开与配置
    uart_ = new LS_UART(device, baudrate, LS_UART_STOP1, LS_UART_DATA8, LS_UART_NONE);
    return true;
}

void Serial::close()
{
    if (uart_)
    {
        delete uart_;
        uart_ = nullptr;
    }
}

int Serial::send(const char* buf, size_t len)
{
    if (!uart_ || !buf)
        return -1;
    ssize_t n = uart_->WriteData(buf, static_cast<ssize_t>(len));
    return static_cast<int>(n);
}

int Serial::send(const uint8_t* buf, size_t len)
{
    return send(reinterpret_cast<const char*>(buf), len);
}

int Serial::receive(char* buf, size_t maxLen)
{
    if (!uart_ || !buf || maxLen == 0)
        return -1;
    ssize_t n = uart_->ReadData(buf, static_cast<ssize_t>(maxLen));
    return static_cast<int>(n);
}

int Serial::receive(uint8_t* buf, size_t maxLen)
{
    return receive(reinterpret_cast<char*>(buf), maxLen);
}

int Serial::sendStr(const char* str)
{
    if (!str)
        return -1;
    return send(str, std::strlen(str));
}

int Serial::sendStr(const std::string& str)
{
    return send(str.c_str(), str.size());
}

int Serial::sendVofaJustFloat(const float* data, int count)
{
    if (!uart_ || !data || count <= 0)
        return -1;
    // JustFloat 格式：小端 float 数组 + 帧尾 0x00 0x00 0x80 0x7f（+Inf，小端）
    const uint8_t tail[4] = { 0x00, 0x00, 0x80, 0x7f };
    ssize_t n1 = uart_->WriteData(reinterpret_cast<const char*>(data), count * sizeof(float));
    if (n1 != static_cast<ssize_t>(count * sizeof(float)))
        return static_cast<int>(n1);
    ssize_t n2 = uart_->WriteData(reinterpret_cast<const char*>(tail), 4);
    return (n2 == 4) ? static_cast<int>(n1) + 4 : static_cast<int>(n1 + n2);
}
