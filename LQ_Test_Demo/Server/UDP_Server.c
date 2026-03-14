#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define IP  "127.0.0.1"
#define PORT 9999

/*!
 * UDP 单播测试服务器端
 */

int main()
{
    // 创建 UDP 套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        perror("socket:");
        return -1;
    }
    printf("UDP 套接字创建成功\n");
    // 配置服务器地址结构
    struct sockaddr_in addr, cliaddr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);
    // 绑定
    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == -1)
    {
        perror("bind:");
        close(sockfd);
        return -1;
    }
    printf("绑定成功\n");

    char buf[255] = {0};
    ssize_t recvret = 0;
    while(1)
    {
        memset(buf, 0, sizeof(buf));
        socklen_t clilen = sizeof(cliaddr);
        // 接收数据
        recvret = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr*)&cliaddr, &clilen);
        if (recvret == -1)
        {
            perror("recvfrom error");
            continue;
        }
        buf[recvret] = '\0';
        printf("UDP_Server : %s\n", buf);
        // 回显
        if (sendto(sockfd, buf, recvret, 0, (struct sockaddr*)&cliaddr, clilen) == -1)
        {
            perror("sendto error");
        }
    }

    return 0;
}