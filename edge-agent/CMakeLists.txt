#基本设置
cmake_minimum_required(VERSION 3.16)
project(edge_agent C)
#编译器
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
#编译文件位置
aux_source_directory(./src SRC_LIST)
aux_source_directory(../config CONFIG)
set(EXECUTABLE_OUTPUT_PATH ./bin)
include_directories(./inc ../config)

#编译
add_executable(edge-agent ${SRC_LIST} ${CONFIG})

# open62541 amalgamation / processing 需要 pthread 和 math
find_package(Threads REQUIRED)
target_link_libraries(edge-agent PRIVATE gpiod Threads::Threads m)
