#include "LQ_demo.hpp"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void UDP_Demo()
 * @功能说明：UDP 测试程序
 * @参数说明：无
 * @函数返回：无
 * @调用方法：UDP_Demo();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void UDP_Demo()
{
    UDP_Client cli("127.0.0.1", 9999);

    char buf[255] = {0};
    
    while(1)
    {
        memset(buf, 0, sizeof(buf));    // 清空数组
        fgets(buf, sizeof(buf), stdin); // 从终端获取数据
        cli.UDP_Send(buf, strlen(buf)); // 发送数据
        cli.UDP_Recv(buf, sizeof(buf)); // 接收数据
        printf("cli Recv : %s\n", buf); // 显示
    }
}