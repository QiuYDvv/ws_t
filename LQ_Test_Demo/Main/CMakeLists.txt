cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

# 设置交叉编译工具路径 --> (需根据自己实际路径进行修改)
set(CROSS_COMPILER_DIR /opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1)
# 指定交叉编译工具所使用的C语言编译器
set(CMAKE_C_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-gcc)
# 指定交叉编译工具所使用的C++编译器
set(CMAKE_CXX_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-g++)

# 项目名称
project(main)

# 指定所用的源文件路径
aux_source_directory(${PROJECT_SOURCE_DIR}/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../Libraries/Driver SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../Libraries/Hal SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../Libraries/APP/Display SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../Libraries/APP/IMU SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../User SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../Demo SRC)

# 指定所用的头文件路径
include_directories(${PROJECT_SOURCE_DIR}/)
include_directories(${PROJECT_SOURCE_DIR}/../Libraries/Driver/inc)
include_directories(${PROJECT_SOURCE_DIR}/../Libraries/Hal/inc)
include_directories(${PROJECT_SOURCE_DIR}/../Libraries/APP/Display)
include_directories(${PROJECT_SOURCE_DIR}/../Libraries/APP/IMU)
include_directories(${PROJECT_SOURCE_DIR}/../User)
include_directories(${PROJECT_SOURCE_DIR}/../Demo)

# 指定OpenCV所用头文件路径 --> (需根据自己实际路径进行修改)
include_directories(/usr/local/include/opencv4/)
# 找到opencv所需的库文件并保存到变量中 --> (需根据自己实际路径进行修改)
find_library(OPENCV_CORE      opencv_core      /usr/local/lib/)
find_library(OPENCV_IMGPROC   opencv_imgproc   /usr/local/lib/)
find_library(OPENCV_HIGHGUI   opencv_highgui   /usr/local/lib/)
find_library(OPENCV_VIDEOIO   opencv_videoio   /usr/local/lib/)
find_library(OPENCV_IMGCODECS opencv_imgcodecs /usr/local/lib/)

# 若需使用静态库，可带上下面两行程序，具体可参考龙邱B站视频教程
# link_directories(${PROJECT_SOURCE_DIR}/a)
# link_libraries(ss)

# 由源文件生成一个可执行程序
add_executable(main ${SRC})

# 连接OpenCV动态库
target_link_libraries(main PUBLIC ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_HIGHGUI} ${OPENCV_VIDEOIO} ${OPENCV_IMGCODECS})

# 连接多线程pthread库
target_link_libraries(main PUBLIC pthread)
