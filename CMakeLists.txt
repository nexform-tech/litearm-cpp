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
option(LITEARM_BUILD_TESTS "构建 tests/" ON)
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
# tests
# ---------------------------------------------------------------------------
# ⚠⚠ 交叉编译时**不许**开测试。
#   注意本仓的测试**不依赖 gtest** ⇒ 它们**编得过** —— 问题在**跑**: ctest 会去执行
#   aarch64 二进制, 而本机跑不了 ⇒ 一整排 Failed, 报错是 "Exec format error" 这种
#   离"你交叉编译了"很远的消息。明确拦下比留一堆跑不了的 target 好。
#   ⚠ toolchain 文件里已经 FORCE 成 OFF; 这里再守一道, 是因为那条约束只活在一个
#     将来可能被改的文件里, 而失败现场在这里 (走本仓的 toolchain 时它其实求值不到 ——
#     它守的是别的交叉配置: 手动设 CMAKE_SYSTEM_NAME、或不含 FORCE 的 toolchain)。
if(CMAKE_CROSSCOMPILING AND LITEARM_BUILD_TESTS)
    message(FATAL_ERROR
        "交叉编译时不允许 LITEARM_BUILD_TESTS=ON: 测试二进制在构建机上跑不了, "
        "ctest 会整排失败。请加 -DLITEARM_BUILD_TESTS=OFF, 或配置 "
        "CROSSCOMPILING_EMULATOR (如 qemu-aarch64) 后自行放开本守卫。")
endif()

if(LITEARM_BUILD_TESTS)
    enable_testing()
    add_library(litearm_test_support STATIC tests/test_framework.cpp)
    target_include_directories(litearm_test_support PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(litearm_test_support PUBLIC litearm)

    file(GLOB LITEARM_TEST_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp")
    # test_framework.cpp 是框架自身 (它提供 main), 已经被编进 litearm_test_support ——
    # 别再把它当成一个"用例文件"注册成一个 0 用例的 ctest 条目。
    list(REMOVE_ITEM LITEARM_TEST_SOURCES
         "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_framework.cpp")
    foreach(src ${LITEARM_TEST_SOURCES})
        get_filename_component(name ${src} NAME_WE)
        add_executable(${name} ${src})
        target_link_libraries(${name} PRIVATE litearm_test_support)
        add_test(NAME ${name} COMMAND ${name})
        # 有用例 SKIP 时框架返回 77 —— 让 ctest 报 `***Skipped` 而不是 "Passed"。
        # ⚠ 这不是洁癖: ctest 默认**吞掉通过用例的全部输出**, 于是"11 个用例被跳过"在
        #   报告里长得和"全跑过了"一模一样 (实测)。假绿比没测更糟。
        set_tests_properties(${name} PROPERTIES SKIP_RETURN_CODE 77)
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
