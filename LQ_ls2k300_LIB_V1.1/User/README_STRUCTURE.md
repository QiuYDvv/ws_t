# 项目代码结构优化建议

## 1. 命名与风格统一

| 现状 | 建议 | 说明 |
|------|------|------|
| `controller.hpp` / 其余 `*.h` | 统一为 `.h` 或 `.hpp` | 便于维护和搜索 |
| `debuger` | 改为 `debugger` | 拼写修正（若重命名需改 main 与 CMake 引用） |
| `TFT_*` 全局函数 / `Camera` 类 | 可逐步统一为「模块类 + 方法」 | 例如 `Displayer::showFps()`，减少全局命名空间 |

## 2. 依赖与头文件

- **main.hpp 过重**：当前 main.cpp 仅需 `Serial`、`Camera`、`Imu`、displayer 接口、OpenCV 的 `cv::Mat` 等，但通过 `#include "main.hpp"` 会拖入整库（LQ_PWM、LQ_GPIO、LQ_TFT18、LQ_Uart 等）。  
  **建议**：main.cpp 只包含业务所需头文件（camera.h、displayer.h、serial.h、imu.h、debuger.h、opencv、termios、csignal 等），不再包含 main.hpp；若龙邱库某处依赖 main.hpp，可单独为库保留一个 platform.hpp，main 不包含它。

- **Camera 与显示耦合**：`drawLineResultOnTFT()` 在 Camera 里直接调 TFT，导致 camera 依赖 displayer。  
  **建议**：将「把 LineSearchResult 画到屏幕」挪到 displayer（例如 `Displayer::drawLineResult(const LineSearchResult&, int w, int h)`），Camera 只产出 `LineSearchResult` 数据，由 main 或 displayer 负责绘制。

## 3. 配置集中

- 主流程中的魔数（内核版本 `4.19.190`、设备 `/dev/ttyS1`、波特率 `B115200`、TFT 横竖屏等）建议集中到**配置头文件或单例**（如 `config.h` / `BoardConfig`），便于换板子或环境时只改一处。

## 4. 初始化与平台逻辑

- **insmod + TFT 初始化** 目前全在 main 里，可收到一个「平台初始化」模块（如 `platform.cpp` / `Board::init()`），内部依次加载内核模块、初始化 TFT 等，main 只调一次 `Platform::init()`，结构更清晰，也方便做条件编译（如仅桌面调试时不 insmod）。

## 5. CMake

- `aux_source_directory(../Demo SRC)` 会把 Demo 下所有 .cpp 都链进 main，若 Demo 仅作参考或独立可执行，建议从 SRC 中去掉 Demo，或为 Demo 单独建 target，避免误链未用代码。
- 交叉编译工具链、OpenCV 路径等可放入 `cmake/toolchain.cmake` 或 `cmake/opencv.cmake`，主 CMakeLists.txt 只做 `include()`，便于多环境切换。

## 6. 模块职责小结

| 模块 | 当前职责 | 可优化方向 |
|------|----------|------------|
| **main.cpp** | 流程编排 + 平台初始化 + 主循环 | 平台初始化抽到 Platform；只包含必要头文件 |
| **camera** | 采集 + 二值化 + 搜线 + 中线 + **画到 TFT** | 绘制挪到 displayer，camera 只做图像与几何结果 |
| **displayer** | TFT 初始化、显示、FPS 显示 | 增加「画 LineSearchResult」接口，统一显示入口 |
| **debuger** | 串口命令、VOFA 发送、电机测试线程 | 命名改为 debugger；可考虑「是否启动电机测试」由配置或命令行控制 |
| **controller** | 电机 + 编码器 + PID | 保持单一职责，接口已清晰 |

以上为建议，可按优先级分步做（例如先统一命名与配置、再抽平台初始化、最后解耦 Camera/displayer）。
