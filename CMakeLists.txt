cmake_minimum_required(VERSION 3.10)
project(arm_control)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /home/anyone/DynamixelSDK/c++/include
)
link_directories(
    /home/anyone/DynamixelSDK/c++/build
)
add_executable(arm_control main.cpp Arm.cpp Camera.cpp)
find_package(Threads REQUIRED)
target_link_libraries(arm_control
    ${OpenCV_LIBS}
    realsense2
    dxl_x64_cpp
    Threads::Threads
)
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)