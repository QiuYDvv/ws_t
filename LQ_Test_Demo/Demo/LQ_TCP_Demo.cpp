#include "LQ_demo.hpp"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void TCP_Demo()
 * @功能说明：TCP 测试程序
 * @参数说明：无
 * @函数返回：无
 * @调用方法：TCP_Demo();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TCP_Demo()
{
    TCP_Client cli("127.0.0.1", 9999);

    char buf[255] = {0};
    cli.TCP_Recv(buf, sizeof(buf));
    printf("%s\n", buf);

    while (1)
    {
        memset(buf, 0, sizeof(buf));    // 清空数组
        fgets(buf, sizeof(buf), stdin); // 从终端获取数据
        cli.TCP_Send(buf, strlen(buf)); // 发送数据
        cli.TCP_Recv(buf, sizeof(buf)); // 获取数据
        printf("cli Recv:%s\n", buf);   // 显示
    }
}
