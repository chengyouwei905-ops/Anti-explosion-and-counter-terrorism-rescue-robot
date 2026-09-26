cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32H723VGT6test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# ---------------------------------------------------------------------------
# 用户代码按模块目录组织，每个模块统一为 <模块>/Src/*.c + <模块>/Inc/*.h，
# 新增文件只要放进对应目录即可，无需改 CMake（CONFIGURE_DEPENDS 会自动重新扫描）。
#   App/       应用层：业务逻辑（视觉、底盘、机械臂、任务状态机）
#   Bsp/       板级支持层：HAL 外设封装（uart / can / gpio / tim ...）
#   Common/    通用工具：环形缓冲、类型定义、日志等，不依赖硬件
#   Protocol/  通信协议层：帧打包/解包（vision / can ...）
#   Algorithm/ 算法层：PID、运动学、滤波等纯计算代码（用到时再建）
# ---------------------------------------------------------------------------
foreach(user_module App Bsp Common Protocol Algorithm)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${user_module}/Src)
        file(GLOB_RECURSE ${user_module}_SOURCES CONFIGURE_DEPENDS
             ${CMAKE_CURRENT_SOURCE_DIR}/${user_module}/Src/*.c)
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${${user_module}_SOURCES})
        target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/${user_module}/Inc)
    endif()
endforeach()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
