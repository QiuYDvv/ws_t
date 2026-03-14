#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define IP  "127.0.0.1"
#define PORT 9999

int main()
{
    // 创建连接套接字
    int connectFd = socket(AF_INET, SOCK_STREAM, 0);
    if (connectFd == -1)
    {
        perror("socket:");
        return -1;
    }
    printf("connectFd = %d\n", connectFd);
    // 绑定
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = inet_addr(IP);
    if (bind(connectFd, (struct sockaddr*)&addr, sizeof(addr)) == -1)
    {
        perror("bind:");
        return -1;
    }
    printf("绑定成功\n");
    // 监听
    if (listen(connectFd, 3) == -1)
    {
        perror("listen:");
        return -1;
    }
    printf("监听成功\n");

    struct sockaddr_in cliaddr;
    int len = sizeof(cliaddr);
    // 连接客户端
    int cfd = accept(connectFd, (struct sockaddr*)&cliaddr, &len);
    char buf[255] = "Successfully connected to the server\n";
    int sendret = send(cfd, buf, sizeof(buf), 0);

    int recvret = 0;
    while(1)
    {
        memset(buf, 0, sizeof(buf));
        recvret = recv(cfd, buf, sizeof(buf), 0);
        if (recvret == 0)
        {
            printf("客户端已断开连接\n");
            return 0;
        }
        send(cfd, buf, recvret, 0);
        printf("Ser Recv:%s\n", buf);
    }
    return 0;
}
