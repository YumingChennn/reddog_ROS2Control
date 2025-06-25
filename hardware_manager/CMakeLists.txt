cmake_minimum_required(VERSION 3.5)
project(can_test)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/can
)

# 設定 lib 搜尋路徑
link_directories(${PROJECT_SOURCE_DIR}/can)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# 主節點編譯

add_executable(can_node_motor_imu
    src/can_node_motor_imu.cpp   
    src/usb2can_motor_imu.cpp
)

target_link_libraries(can_node_motor_imu
    pthread
    rt
    usb_can
    unitree_sdk2
)


