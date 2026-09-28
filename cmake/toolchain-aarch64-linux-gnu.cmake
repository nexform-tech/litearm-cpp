# aarch64 (Linux) 交叉编译工具链文件。
#
# 用法:
#     cmake -S . -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64-linux-gnu.cmake
#     cmake --build build-arm64 -j
#
# ⚠⚠ **本仓从未在 aarch64 上运行过** —— 只做过"能否编译"这一层。对齐、端序、串口行为
#   在 arm64 上**无人守** (端序这一条风险其实很低: 线协议是逐字节组装的, 见 protocol.cpp;
#   但"很低"不是"验过")。这条限制写在 README 的"平台"一节里, 别读成"已支持 arm64"。
#
# ⚠ 依赖: `aarch64-linux-gnu-g++` (Debian/Ubuntu 上是 `g++-aarch64-linux-gnu`)。
#   本机实测 (2026-09-28): **未安装** ⇒ 本文件在本机跑不通, 属于"写好了但没验过"。
#   ⚠ 这句话留着是**刻意的**: 它是登记册里那种"未验证"的诚实记录, 不要因为某天装上
#     工具链跑通了就把它删掉 —— 要删就把它改成"实测通过 + 日期 + 机器"。

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 只让 find_* 在**目标**根下找库/头, 但程序 (如工具) 仍在本机找。
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# ⚠ 交叉时必须关测试。本仓的测试**不依赖 gtest**, 所以它们**编得过** —— 问题在**跑**:
#   ctest 会去执行 aarch64 二进制, 而本机跑不了 ⇒ 一堆 Failed, 且报错形状是
#   "Exec format error" 这种离"你交叉了"很远的消息。
#   明确拦下比留一堆跑不了的 target 好 (顶层 CMakeLists 里还有第二道, 见那里)。
set(LITEARM_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# ⚠ FORCE 的副作用: toolchain 文件在命令行 `-D` **之后**处理 ⇒ 显式
#   `-DLITEARM_BUILD_TESTS=ON` 会被静默覆盖成 OFF。方向是安全的 (宁可少编也不产出
#   跑不了的 target), 故保留 FORCE —— 但要接交叉跑测试, 请用 CROSSCOMPILING_EMULATOR
#   (qemu-aarch64) 而不是把这个 FORCE 去掉。
