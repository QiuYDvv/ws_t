#include <iostream>
#include <opencv2/opencv.hpp>
#include <winsock2.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>

/*注：此文件是windows下的接收端代码*/

#pragma comment(lib, "ws2_32.lib")

using namespace cv;
using namespace std;

class FrameBuffer {
private:
    queue<vector<uchar>> frame_queue;
    mutex queue_mutex;
    const size_t max_size = 2;

public:
    void addFrame(const vector<uchar>& frame_data) {
        lock_guard<mutex> lock(queue_mutex);

        if (frame_queue.size() >= max_size) {
            frame_queue.pop();
        }

        frame_queue.push(frame_data);
    }

    bool getLatestFrame(vector<uchar>& frame_data) {
        lock_guard<mutex> lock(queue_mutex);

        if (frame_queue.empty()) {
            return false;
        }

        frame_data = frame_queue.back();

        while (frame_queue.size() > 1) {
            frame_queue.pop();
        }

        return true;
    }

    size_t size() {
        lock_guard<mutex> lock(queue_mutex);
        return frame_queue.size();
    }
};

class UDPServer {
private:
    SOCKET server_socket;
    atomic<bool> running;
    FrameBuffer frame_buffer;
    thread receiver_thread;
    thread processor_thread;
    const int MAX_PACKET_SIZE = 65507;

public:
    UDPServer() : server_socket(INVALID_SOCKET), running(false) {}

    bool initialize(int port) {
        // 初始化Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            cerr << "WSAStartup failed" << endl;
            return false;
        }

        // 创建UDP socket
        server_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (server_socket == INVALID_SOCKET) {
            cerr << "Failed to create UDP socket: " << WSAGetLastError() << endl;
            WSACleanup();
            return false;
        }

        // 设置socket选项
        int opt = 1;
        if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR,
            reinterpret_cast<const char*>(&opt), sizeof(opt)) == SOCKET_ERROR) {
            cerr << "setsockopt failed: " << WSAGetLastError() << endl;
            closesocket(server_socket);
            WSACleanup();
            return false;
        }

        // 设置非阻塞模式
        u_long mode = 1;
        if (ioctlsocket(server_socket, FIONBIO, &mode) == SOCKET_ERROR) {
            cerr << "ioctlsocket failed: " << WSAGetLastError() << endl;
            closesocket(server_socket);
            WSACleanup();
            return false;
        }

        // 绑定socket
        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);

        int bindResult = ::bind(server_socket,
            reinterpret_cast<sockaddr*>(&server_addr),
            static_cast<int>(sizeof(server_addr)));

        if (bindResult == SOCKET_ERROR) {
            cerr << "Bind failed: " << WSAGetLastError() << endl;
            closesocket(server_socket);
            WSACleanup();
            return false;
        }

        running = true;
        cout << "UDP Server started on port " << port << endl;
        return true;
    }

    void start() {
        receiver_thread = thread(&UDPServer::receiverLoop, this);
        processor_thread = thread(&UDPServer::processorLoop, this);

        cout << "Server threads started. Press 'q' in the image window to quit." << endl;

        // 等待线程结束
        if (receiver_thread.joinable()) {
            receiver_thread.join();
        }
        if (processor_thread.joinable()) {
            processor_thread.join();
        }
    }

private:
    void receiverLoop() {
        vector<char> buffer(MAX_PACKET_SIZE);
        sockaddr_in client_addr;
        int client_len = sizeof(client_addr);

        int consecutive_empty = 0;
        auto last_packet_time = chrono::steady_clock::now();

        while (running) {
            // 接收数据
            int recv_size = recvfrom(server_socket,
                buffer.data(),
                static_cast<int>(buffer.size()),
                0,
                reinterpret_cast<sockaddr*>(&client_addr),
                &client_len);

            if (recv_size > 0) {
                // 验证并处理接收到的数据
                vector<uchar> frame_data(buffer.begin(), buffer.begin() + recv_size);

                // 检查JPEG文件头
                if (frame_data.size() >= 2 && frame_data[0] == 0xFF && frame_data[1] == 0xD8) {
                    frame_buffer.addFrame(frame_data);
                    last_packet_time = chrono::steady_clock::now();
                    consecutive_empty = 0;
                }
            }
            else {
                int error = WSAGetLastError();
                if (error != WSAEWOULDBLOCK) {
                    cerr << "recvfrom error: " << error << endl;
                    break;
                }

                // 检查连接超时
                consecutive_empty++;
                if (consecutive_empty > 1000) {
                    auto now = chrono::steady_clock::now();
                    if (chrono::duration_cast<chrono::seconds>(now - last_packet_time).count() > 5) {
                        cout << "No data received for 5 seconds" << endl;
                        last_packet_time = now;
                    }
                    consecutive_empty = 500;
                }

                this_thread::sleep_for(chrono::microseconds(100));
            }
        }
    }

    void processorLoop() {
        namedWindow("UDP Image Transmission", WINDOW_AUTOSIZE);

        int frame_count = 0;
        auto start_time = chrono::steady_clock::now();
        auto last_stat_time = start_time;

        while (running) {
            vector<uchar> frame_data;
            if (frame_buffer.getLatestFrame(frame_data)) {
                try {
                    Mat image = imdecode(frame_data, IMREAD_COLOR);
                    if (!image.empty()) {
                        imshow("UDP Image Transmission", image);
                        frame_count++;
                    }
                }
                catch (const exception& e) {
                    cerr << "Image decode error: " << e.what() << endl;
                }
            }

            // 显示统计信息
            auto now = chrono::steady_clock::now();
            if (chrono::duration_cast<chrono::seconds>(now - last_stat_time).count() >= 2) {
                cout << "FPS: " << frame_count / 2 << ", Buffer: " << frame_buffer.size() << endl;
                frame_count = 0;
                last_stat_time = now;
            }

            // 检查退出键
            int key = waitKey(1);
            if (key == 'q' || key == 'Q' || key == 27) {
                running = false;
                break;
            }

            this_thread::sleep_for(chrono::milliseconds(1));
        }

        destroyAllWindows();
    }

public:
    void stop() {
        running = false;

        if (server_socket != INVALID_SOCKET) {
            closesocket(server_socket);
            WSACleanup();
        }

        cout << "Server stopped" << endl;
    }

    ~UDPServer() {
        stop();
    }
};

int main() {
    cout << "=== UDP Image Transmission Server ===" << endl;
    cout << "Features:" << endl;
    cout << "- UDP protocol for lower latency" << endl;
    cout << "- Frame dropping for real-time performance" << endl;
    cout << "- Non-blocking socket operations" << endl;
    cout << "=====================================" << endl;

    UDPServer server;

    if (!server.initialize(8080)) {
        cerr << "Server initialization failed" << endl;
        return -1;
    }

    server.start();

    return 0;
}