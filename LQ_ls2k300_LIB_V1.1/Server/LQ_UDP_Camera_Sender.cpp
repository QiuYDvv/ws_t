#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>
#include <fcntl.h>

/*注：此文件是部署到久久派的发送端代码*/

using namespace cv;
using namespace std;

class ImageClient {
private:
    int client_socket;
    struct sockaddr_in server_addr;
    std::atomic<bool> running{true};
    const int IMG_WIDTH = 320;
    const int IMG_HEIGHT = 240;
    const int MAX_PACKET_SIZE = 65507; // UDP最大包大小

public:
    bool connectToServer(const std::string& server_ip, int port) {
        client_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (client_socket == -1) {
            std::cerr << "Failed to create UDP socket" << std::endl;
            return false;
        }

        // 设置socket为非阻塞
        int flags = fcntl(client_socket, F_GETFL, 0);
        fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);

        // 设置目标服务器地址
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        
        if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid address/Address not supported" << std::endl;
            return false;
        }

        std::cout << "UDP Client initialized. Target: " << server_ip << ":" << port << std::endl;
        return true;
    }

    // 发送图像的函数接口
    bool sendImage(const cv::Mat& image) {
        if (!running) {
            return false;
        }

        cv::Mat processed_frame = image.clone();

        // 旋转图像
        rotate(processed_frame, processed_frame, ROTATE_180);

        // 调整图像大小
        if (processed_frame.cols != IMG_WIDTH || processed_frame.rows != IMG_HEIGHT) {
            cv::resize(processed_frame, processed_frame, cv::Size(IMG_WIDTH, IMG_HEIGHT));
        }

        // 编码为JPEG
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(30); // 低质量以减少数据量

        std::vector<uchar> buffer;
        if (!cv::imencode(".jpg", processed_frame, buffer, compression_params)) {
            std::cerr << "Failed to encode image" << std::endl;
            return false;
        }

        // 检查数据大小是否超过UDP限制
        if (buffer.size() > MAX_PACKET_SIZE) {
            std::cerr << "Image too large for UDP: " << buffer.size() << " bytes" << std::endl;
            return false;
        }

        // 发送图像数据
        int sent = sendto(client_socket, buffer.data(), buffer.size(), 0,
                         (struct sockaddr*)&server_addr, sizeof(server_addr));
        
        if (sent < 0) {
            // 非阻塞socket，EAGAIN是正常的
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Failed to send image data: " << strerror(errno) << std::endl;
            }
            return false;
        }

        return true;
    }

    bool initializeCamera(cv::VideoCapture& cap) {
        // 尝试打开摄像头
        cap.open(0, cv::CAP_V4L2);
        
        if (!cap.isOpened()) {
            cap.open(0);
            if (!cap.isOpened()) {
                std::cerr << "Failed to open camera" << std::endl;
                return false;
            }
        }

        // 设置摄像头参数
        cap.set(cv::CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
        cap.set(cv::CAP_PROP_FPS, 60);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 2);

        std::cout << "Camera initialized: " 
                  << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" 
                  << cap.get(cv::CAP_PROP_FRAME_HEIGHT) 
                  << " at " << cap.get(cv::CAP_PROP_FPS) << " FPS" << std::endl;

        return true;
    }

    void captureAndSend() {
        cv::VideoCapture cap;
        if (!initializeCamera(cap)) {
            return;
        }

        cv::Mat frame;
        auto frame_interval = std::chrono::microseconds(1000000 / 60);
        auto next_frame_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        auto start_time = std::chrono::steady_clock::now();

        while (running) {
            auto current_time = std::chrono::steady_clock::now();
            
            // 控制帧率
            if (current_time < next_frame_time) {
                std::this_thread::sleep_until(next_frame_time);
            }
            next_frame_time += frame_interval;

            // 捕获帧
            if (!cap.read(frame)) {
                std::cerr << "Failed to capture frame" << std::endl;
                break;
            }

            // 使用sendImage函数发送
            sendImage(frame);

            frame_count++;
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();
            
            if (elapsed >= 1) {
                std::cout << "Sent FPS: " << frame_count << std::endl;
                frame_count = 0;
                start_time = std::chrono::steady_clock::now();
            }
        }

        cap.release();
        std::cout << "Camera released" << std::endl;
    }

    void stop() {
        running = false;
        if (client_socket != -1) {
            close(client_socket);
        }
    }

    ~ImageClient() {
        stop();
    }
};

/**
 * @brief UDP图像传输客户端主函数
 * 
 * 程序入口点，负责初始化UDP客户端连接并启动图像捕获和传输
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组，argv[0]为程序名，argv[1]为服务器IP地址
 * @return int 程序退出状态码：0表示成功，-1表示失败
 * 
 * @note 使用方式: 程序名 <服务器IP>
 * @note 服务器默认端口为8080
 */
/**
 * @brief UDP图像传输客户端主函数
 * 
 * 负责初始化UDP客户端连接，捕获图像并通过UDP协议发送到指定服务器
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 *        - argv[0]: 程序名称
 *        - argv[1]: 服务器IP地址
 * 
 * @return int 程序退出状态码
 *         - 0: 正常退出
 *         - -1: 参数错误或连接失败
 */
int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <server_ip>" << std::endl;
        return -1;
    }

    ImageClient client;
    
    if (!client.connectToServer(argv[1], 8080)) {
        std::cerr << "Failed to initialize UDP client" << std::endl;
        return -1;
    }

    std::cout << "Starting UDP image transmission..." << std::endl;
    client.captureAndSend();
    
    std::cout << "Client stopped" << std::endl;
    return 0;
}