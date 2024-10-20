cmake_minimum_required(VERSION 3.8)
project(vins)

########################################## 编译器设置 ##########################################
# 检查当前C++编译器是否为 GNU Compiler Collection 或者 检查 CMAKE_CXX_COMPILER_ID 是否匹配字符串 "Clang"
# 如果条件为真（即使用的是 GCC 或 Clang 编译器）
# -Wall 启用了大多数警告消息，-Wextra 启用了额外的警告消息，-Wpedantic 启用了更严格的警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 添加realse版本的优化选项
# -O1 它启用一些通用的优化，但不会引入太多的编译时间延迟。它通常用于提高代码的执行速度，同时保留一定的可读性和可调试性。
# -O2 它启用更多的优化，可能会花费更多的编译时间，但通常会提高代码的执行速度。
# -O3 它启用了最大程度的优化，会应用更多的优化技术，以提高代码的执行速度。
add_compile_options(-O2)

# 内参检查工具
set(ENABLE_ASAN off)
if (ENABLE_ASAN)
  message(STATUS "build with ASAN")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
endif ()

# 没有指定CMAKE_BUILD_TYPE 默认为 Release
set(CMAKE_BUILD_TYPE Release)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

# debug模式下检查内存泄漏
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")

# 定义输出的assets的路径参考
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

include_directories(
        "${PROJECT_SOURCE_DIR}/include/"
)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARY_DIRS})
if (${OpenCV_FOUND})
  message(STATUS "OpenCV's Version is:${OpenCV_VERSION}")
else()
  message(FATAL_ERROR "OpenCV not found!")
endif ()

# 查找 RealSense2 库
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})
link_libraries(${realsense2_LIBRARY_DIRS})
if(${realsense2_FOUND})
  message(STATUS "realsense2 SDK Version is:${realsense2_VERSION}")
else()
  message(FATAL_ERROR "realsense2 SDK not found!")
endif()

# 查找yaml-cpp库
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})
# 检查是否找到库
if (yaml-cpp_FOUND)
  message(STATUS "yaml-cpp's Version is: ${yaml-cpp_VERSION}")
else()
  message(FATAL_ERROR "yaml-cpp not found!")
endif()


add_executable(${PROJECT_NAME}_node
        src/realsense_pipline_node.cpp
        src/realsense_pipline.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBRARIES}
        ${yaml-cpp_LIBRARIES}
        realsense2
        yaml-cpp
)


