cmake_minimum_required(VERSION 3.5)
project(reddog_hardware)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 設定 SDK 路徑（預設為 /usr/local/xsens，可透過 cmake -D 指定）
set(XSENS_INSTALL_PREFIX "/usr/local/xsens" CACHE PATH "Path to Xsens SDK")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/can
    /usr/include/eigen3
)

# include 與 lib 路徑
include_directories(${XSENS_INSTALL_PREFIX}/include)
include_directories(${XSENS_INSTALL_PREFIX}/include/xsensdeviceapi)
link_directories(${XSENS_INSTALL_PREFIX}/lib)

# 設定 lib 搜尋路徑
link_directories(${PROJECT_SOURCE_DIR}/can)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# 共用的 linker flags
set(COMMON_LIBS
    xsensdeviceapi
    xstypes
    pthread
    m
)

# 主節點編譯
add_executable(can_node_motor_imu
    src/can_node_motor_imu.cpp   
    src/usb2can_motor_imu.cpp
    src/callback_handler.cpp
)

target_include_directories(can_node_motor_imu PRIVATE include)
target_link_libraries(can_node_motor_imu
    pthread
    rt
    usb_can
    unitree_sdk2
    ${COMMON_LIBS}
)

add_executable(example_mti_reddog
    src/example_mti_reddog.cpp
    src/callback_handler.cpp
)
target_include_directories(example_mti_reddog PRIVATE include)
target_link_libraries(example_mti_reddog ${COMMON_LIBS})



