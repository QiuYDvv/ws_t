#include "LQ_demo.hpp"

#include <signal.h>
#include <sys/time.h>

#include <thread>

using namespace std;

// 信号处理函数
void timer_handler(int signum)
{
    static uint64_t val = 0;
    printf("Timer signal received! %ld\n", val++);
    // 添加自定义逻辑
}

/*!
 * @brief   setitimer 它能设置周期性的定时器，还可指定不同类型的定时器（如真实时间、虚拟时间等）
 */
void setitimerDemo()
{
    // 注册信号处理函数
    signal(SIGALRM, timer_handler);
    // 设置定时器间隔和初始延迟
    struct itimerval timer;
    timer.it_value.tv_sec = 2;          // 设置初始延迟 2s
    timer.it_value.tv_usec = 0;         // 设置初始延迟 0us
    timer.it_interval.tv_sec = 0;       // 设置间隔延迟 0s
    timer.it_interval.tv_usec = 5000;   // 设置间隔延迟 5ms
    // 设置定时器
    if (setitimer(ITIMER_REAL, &timer, NULL) == -1)
    {
        perror("setitimer");
        return;
    }
    printf("Waiting for timer...\n");
    while(1)
    {
        printf("setitimerDemo!\n");
        sleep(1);
    }
}

// 线程函数
void timer_thread()
{
    static uint64_t val = 0;
    while(true)
    {
        // 添加自定义逻辑
        printf("timer_thread! %ld\n", val++);
        // 等待 5ms
        this_thread::sleep_for(chrono::milliseconds(5));
    }
}

/*!
 * @brief   this_thread::sleep_for 使用线程休眠函数
 */
void HibernateDemo()
{
    // 创建线程
    thread t(timer_thread);

    while(1)
    {
        printf("HibernateDemo!\n");
        sleep(1);
    }
}
