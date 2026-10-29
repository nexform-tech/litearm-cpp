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

# 本仓不包含测试套件；交叉构建只产出库 / CLI / examples。
