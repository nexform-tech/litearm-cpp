cmake_minimum_required(VERSION 3.14)
project(litearm-cpp VERSION 2.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# LiteArm STM32 直连后端 —— 薄协议 SDK (C++17, 零外部依赖)
#
# 唯一依赖是 C++ 标准库与平台自带的串口 API (POSIX termios / Win32) —— 刻意不引
# libserialport 之类: PC 侧只做帧编解码, 重型计算 (S 曲线/IK/动力学) 本来就在固件里。
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(LITEARM_BUILD_EXAMPLES "构建 examples/" ON)
option(LITEARM_WERROR "把编译警告当错误" OFF)

add_library(litearm
    src/protocol.cpp
    src/clock.cpp
    src/framing.cpp
    src/errors.cpp
    src/rot.cpp
    src/state.cpp
    src/transport.cpp
    src/ack.cpp
    src/params.cpp
    src/model.cpp
    src/log.cpp
    src/diagnostics.cpp
    src/cart.cpp
    src/arm.cpp
    src/testing.cpp
)
add_library(litearm::litearm ALIAS litearm)

target_include_directories(litearm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# POSIX 下用 termios/poll/flock; Windows 下用 Win32 串口 API。
if(WIN32)
    target_link_libraries(litearm PUBLIC setupapi)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(litearm PUBLIC Threads::Threads)
endif()

if(MSVC)
    target_compile_options(litearm PRIVATE /W4)
else()
    target_compile_options(litearm PRIVATE -Wall -Wextra)
    if(LITEARM_WERROR)
        target_compile_options(litearm PRIVATE -Werror)
    endif()
endif()

# ---------------------------------------------------------------------------
# CLI (巡检/冒烟) —— 库的薄壳, 零额外依赖, 默认随库构建。
# ---------------------------------------------------------------------------
add_executable(litearm-cpp cli/main.cpp)
target_link_libraries(litearm-cpp PRIVATE litearm)

# ---------------------------------------------------------------------------
# examples
# ---------------------------------------------------------------------------
if(LITEARM_BUILD_EXAMPLES)
    file(GLOB LITEARM_EXAMPLE_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")
    list(REMOVE_ITEM LITEARM_EXAMPLE_SOURCES
         "${CMAKE_CURRENT_SOURCE_DIR}/examples/_common.cpp")
    add_library(litearm_examples_common STATIC examples/_common.cpp)
    target_link_libraries(litearm_examples_common PUBLIC litearm)
    foreach(src ${LITEARM_EXAMPLE_SOURCES})
        get_filename_component(name ${src} NAME_WE)
        add_executable(${name} ${src})
        target_link_libraries(${name} PRIVATE litearm_examples_common)
    endforeach()
endif()

# ---------------------------------------------------------------------------
# install / 打包
# ---------------------------------------------------------------------------
# 客户两种用法都支持:
#     find_package(litearm CONFIG REQUIRED)
#     target_link_libraries(my_app PRIVATE litearm::litearm)
#   —— 或 ——
#     pkg-config --cflags --libs litearm
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ⚠ 只对**共享**产物有意义; 静态库上设它无害。
#   SOVERSION 取主版本号 ⇒ 同一主版本内承诺 ABI 兼容 (这是本仓的**承诺**, 不是默认值)。
set_target_properties(litearm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR})

install(TARGETS litearm
        EXPORT litearmTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        # ⚠ INCLUDES DESTINATION 让导出的 target 自带 include 路径 —— 少了它, 客户
        #   `find_package` 成功、`target_link_libraries` 也成功, 但**编译时找不到头**,
        #   报错指向 "litearm/litearm.hpp: No such file" 而完全看不出是打包漏了一行。
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# ⚠ 装的是**整个目录** (`include/litearm`), 不是逐个头文件列的清单 —— 目录形式会自动
#   跟上新增的头 (本仓头文件加得频繁: clock.hpp / framing.hpp 都是后加的)。
install(DIRECTORY include/litearm DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# ⚠ CLI 也装到 bin/ —— README 承诺 "The SDK also ships a small CLI", 而只把库装出去、
#   把 CLI 留在构建树里, 那句 "ships" 就是空头支票 (现场巡检的人手上没有源码 checkout)。
# ⚠ 刻意**不并进 `litearmTargets` 导出**: 那是给 `find_package` 消费者 `link` 用的,
#   可执行文件进去没有意义。故单独一条 install, 不带 EXPORT。
install(TARGETS litearm-cpp RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# --- CMake 包配置 ---
install(EXPORT litearmTargets
        FILE litearmTargets.cmake
        NAMESPACE litearm::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litearm)

configure_package_config_file(
    cmake/litearmConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/litearmConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litearm)

# ⚠ `SameMajorVersion`: 主版本相同的请求都算兼容 (与上面的 SOVERSION 承诺一致)。
#   换成 `ExactVersion` 会让客户在 2.1.1 上必须写死 2.1.1 —— 对一个小版本修 bug 的
#   库来说是自找麻烦。
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/litearmConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/litearmConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/litearmConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litearm)

# --- pkg-config ---
configure_file(cmake/litearm.pc.in ${CMAKE_CURRENT_BINARY_DIR}/litearm.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/litearm.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
