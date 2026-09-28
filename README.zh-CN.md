# litearm-cpp

LiteArm 机械臂的 **C++ SDK** —— 经由 USB 串口**直连固件**, 中间没有服务器、没有中间件。
轨迹规划、运动学与动力学全部由固件承担, PC 侧只发点、收结果。

本仓是 [litearm-python](https://github.com/nexform-tech/litearm-python) 2.1.0 的 C++ 移植:
同一套线协议、同一套语义、同一套安全判据, 逐条对照移植; 测试同样全离线可跑。

> 接口全景见 [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md);
> 现场排查 (现象 → 原因 → 怎么办) 见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。

## 特点

- **零外部依赖**: 只用 C++17 标准库 + 平台自带串口 API (POSIX termios / Win32)。
  不引 libserialport, 不引 numpy —— 重型计算本来就在固件里。
- **单一定位**: 头文件 `include/litearm/`, 一个静态库 `liblitearm.a`。
- **直连 USB**: 一根线到固件; VID:PID `1d50:606f` 自动发现。
- **固件干重活**: 规划/运动学/动力学都在固件; PC 侧只发点、判到位。
- **完整运动 API**: 关节运动、笛卡尔直线/圆弧/多路点、拖动示教。
- **安全内建**: 子进程 fail-closed、独立急停通道、每条不可逆命令都标出来。

## 构建

| 项目 | 要求 |
| --- | --- |
| 编译器 | 支持 C++17 (g++ 7+ / clang 6+ / MSVC 2019+) |
| 构建 | CMake ≥ 3.14 |
| 依赖 | 无 (POSIX 用 termios/poll/flock, Windows 用 Win32 串口 API) |
| 固件 | `Litearm1.5.0` 或更高 |
| 连接 | USB CDC 串口, VID:PID `1d50:606f` |

```bash
./build.sh                 # 配置 + 构建 + 跑测试 (全离线)
source env.sh              # 把 build/ 放进 PATH (首次会自动构建)
./run_example.sh 01_hello
```

手工构建:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CMake 开关: `-DLITEARM_BUILD_EXAMPLES=OFF` / `-DLITEARM_BUILD_TESTS=OFF` /
`-DLITEARM_WERROR=ON`。

Linux 需要串口权限:

```bash
sudo usermod -aG dialout $USER      # 重新登录后生效
```

作为子目录接入:

```cmake
add_subdirectory(third_party/litearm-cpp)
target_link_libraries(your_app PRIVATE litearm::litearm)
```

### 安装与打包

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j && cmake --install build
```

装出来的东西 (三种消费方式都验过 —— 见下面「怎么验的」):

```
include/litearm/*.hpp              # 全部公开头 (含 testing.hpp / framing.hpp)
lib/liblitearm.a                   # 或 .so (加 -DBUILD_SHARED_LIBS=ON)
lib/cmake/litearm/litearmConfig.cmake
lib/cmake/litearm/litearmConfigVersion.cmake
lib/cmake/litearm/litearmTargets.cmake
lib/pkgconfig/litearm.pc
```

**消费方式一 —— CMake**:

```cmake
find_package(litearm CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE litearm::litearm)
```

**消费方式二 —— pkg-config**:

```bash
g++ my_app.cpp $(pkg-config --cflags --libs litearm)
# 静态链接要带 --static (它会带出 Libs.private 里的 -lpthread)
```

**消费方式三 —— 子目录** (`add_subdirectory`, 见上)。

⚠ 伞头 `litearm.hpp` **刻意不含** `testing.hpp` 与传输实现细节 —— 用假件要自己
`#include <litearm/testing.hpp>`。

#### 怎么验的 (别只看"装上了")

在临时 prefix 上真装一遍, 再写**真消费者**分别用 `find_package` 与 `pkg-config` 去编、去跑:

```bash
cmake -S . -B build-inst -DCMAKE_INSTALL_PREFIX=/tmp/litearm-prefix \
      -DLITEARM_BUILD_TESTS=OFF -DLITEARM_BUILD_EXAMPLES=OFF
cmake --build build-inst -j && cmake --install build-inst
# 然后: cmake 消费者 find_package(litearm CONFIG REQUIRED) → 编过 → 跑通
#       pkg-config 消费者 g++ main.cpp $(pkg-config --cflags --libs litearm) → 编过 → 跑通
```

`tests/test_packaging.cpp` 另外守着**卖出去之前**就能发现的那几类:
版本只有一个说法 (CMakeLists ↔ 头里的宏 ↔ `version()`)、公开头引用的兄弟头都在
`include/litearm/` 里、两个打包模板文件在、`.pc` 的版本不是写死的。

#### aarch64 交叉编译

```bash
cmake -S . -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64-linux-gnu.cmake
cmake --build build-arm64 -j
```

⚠⚠ **本仓从未在 aarch64 上运行过** —— 只做过"能否编译"这一层, 而**编译这一层在本机也没验成**
(没装 `aarch64-linux-gnu-g++`): 工具链文件被正确解析、干净地失败在"找不到编译器"。
对齐、端序、串口行为在 arm64 上**无人守**。别把本节读成"已支持 arm64"。

⚠ 那个工具链文件**强制** `LITEARM_BUILD_TESTS=OFF`: 本仓的测试不依赖 gtest 所以**编得过**,
但 ctest 会在构建机上执行 aarch64 二进制 ⇒ 一整排 "Exec format error"。
要交叉**跑**测试请配置 `CROSSCOMPILING_EMULATOR` (qemu-aarch64)。

## 快速上手

```cpp
#include <litearm/litearm.hpp>
using namespace litearm;

int main() {
    Arm arm;                   // 不指定端口则自动发现 (VID:PID 1d50:606f)
    arm.connect();             // 校验固件版本约定

    std::printf("固件 %s, %d 关节\n", arm.firmware().c_str(), arm.n());

    arm.enable();              // 任何运动之前必须先使能

    // 关节运动: 固件规划 S 曲线并自完成
    arm.movej({0.1, 0, 0, 0, 0, 0, 0}, 0.3);

    // 笛卡尔直线: 位姿 = 3 个位置 (m) + 3 个姿态 (rad)
    arm.move_l({0.30, 0.0, 0.40, 3.1416, 0.0, 0.0}, 0.5);

    // 读状态 —— 注意读值要 .value (返回的是 Msg 信封, 见下)
    auto st = arm.get_state().value;
    if (st) std::printf("q = %s\n", fmt(st->q()).c_str());

    auto tcp = arm.get_tcp().value;
    if (tcp) std::printf("TCP = %s\n", fmt(*tcp).c_str());

    // 逆运动学: 位姿 -> 关节角
    auto q = arm.ik({0.30, 0.0, 0.35, 3.1416, 0.0, 0.0});

    arm.home();                // 回零位舒展姿
    arm.close();               // 断开 (析构也会兜底调它)
}
```

`Arm::connect()` 是唯一入口。每条会话都带一条**读线程**, 所以**必须 `close()`** ——
C++ 的析构会兜底, 但显式关掉更清楚 (析构只在对象真被析构时才跑)。
不传端口则自动发现; SDK **不读任何环境变量**。

## 接口参考

### 连接

```cpp
Arm arm;                              // 自动发现
Arm arm2("/dev/ttyACM0");             // 指定端口
arm.connect();                        // 幂等: 已连着同一目标则 no-op
arm.reconnect();                      // 强制重建 (改过 move_timeout 之后要用它)
arm.close();                          // 断开 (幂等)
printf("%s / %d\n", arm.firmware().c_str(), arm.n());
```

### 关节运动

三条都需要先 `enable()`: `movej` 是单发, `movej_sync` 是同步 PTP, `home` 回零位。

```cpp
arm.enable();
arm.movej({0.1,0,0,0,0,0,0}, 0.3);            // 每轴独立 S 曲线 (先到先停)
arm.movej_sync({0.1,0,0,0,0,0,0});            // 一条路径标量驱动全轴, 同时到达 (更慢但可预测)
arm.home();                                    // 固件速度写死 0.10
```

### 笛卡尔运动

```cpp
std::array<double,6> P1{0.30, 0.0, 0.40, 3.1416, 0.0, 0.0};
std::array<double,6> P2{0.32, 0.0, 0.42, 3.1416, 0.0, 0.0};
std::array<double,6> P3{0.34, 0.0, 0.44, 3.1416, 0.0, 0.0};

CartPlan a = arm.move_l(P2, 0.5);              // 直线
CartPlan b = arm.move_p(P2);                   // 关节空间点到点 —— 不是直线!
CartPlan c = arm.move_path({P1, P2, P3}, 0.5); // 依次经过, 尖角
CartPlan d = arm.move_c(arm.get_tcp().value.value(), P2, P3);   // 圆弧 (起点须为实测 TCP)

CartPlan e = arm.move_l(P2, 0.5, /*wait=*/false);   // 不阻塞; 之后自己轮询
auto claimed = arm.poll_cart();                      // 认领一条没被取走的结果
arm.set_speed(50);                                   // 全局调速器: 整数百分比 0..100
```

位姿也收 `(pos[3], R[3x3])` 与 4x4 齐次矩阵:

```cpp
rot::Vec3 pos{0.32, 0.0, 0.42};
rot::Mat3 R = rot::rpy_to_mat({3.1416, 0.0, 0.0});
arm.move_l(rot::PoseInput::from_pos_rot(pos, R), 0.5);
```

### 状态 / 运动学

```cpp
auto st = arm.get_state().value;               // std::optional<RobotState>
if (st) {
    fmt(st->q()); fmt(st->dq()); fmt(st->tau());
    bool en = st->enabled(), busy = st->cart_busy(), bad = st->faulted();
    std::string detail = st->fault_detail();
    std::vector<int> axes = st->fault_axes();   // 0 基
}
auto tcp = arm.get_tcp().value;                 // std::optional<std::array<double,6>>
arm.get_status_now();                           // 主动要一帧
arm.ik({0.30, 0.0, 0.35, 3.1416, 0.0, 0.0});    // 位姿 -> q[7]
```

### 生活 / 安全

```cpp
arm.emergency_stop();       // 急停: 没有任何前置条件
arm.reset();                // 清故障 + 重新锚定 (不是 MCU 重启)
arm.clear_faults();         // 只清 RAM 故障位
arm.disable();              // ⚠ 失能后位置环不再抱住臂
```

### 前馈 / 动力学

```cpp
arm.set_payload(1.0);                                   // 换负载后**总是**要调
arm.set_gravity_scale({1,1,1,1,1,1,1});
arm.ff_preset(1);                                       // 0 全关 / 1 出厂 / 2 全开
double m = arm.get_ff_scalar(4).value;                  // 读回 payload_mass
arm.get_ff_mask();                                      // ff_mask (0x2C item 9, 只读)
```

### 拖动示教

```cpp
{
    auto zg = arm.zero_g();        // 进入 + 启动保活
    std::this_thread::sleep_for(std::chrono::seconds(10));
}                                  // 退出作用域自动收尾 (不抛)
// 或显式配对: arm.zero_g_start(); ...; arm.zero_g_stop();
```

### 关节级参数 (`arm.params()`)

```cpp
JointParam p = arm.params().get_joint_param(0).value;   // J1 的 kp/kd/tau_max/软限位
arm.params().set_joint_param(0, 30.0, 1.0, 20.0);       // idx, kp, kd, tau_max
arm.params().set_joint_limits(0, -1.0, 1.0);            // 只许收窄
for (const auto& jp : arm.params().all_joint_params()) {
    printf("J%d kp=%.1f q=[%.3f, %.3f]\n", jp.idx + 1, jp.kp, jp.q_min, jp.q_max);
}
```

### 动力学模型 (`arm.model()`)

```cpp
if (arm.model().probe()) {                              // 固件支持在线导入吗
    for (int i = 1; i <= 7; ++i) arm.model().set_body(i, body10);
    arm.model().set_jm({...});
    arm.model().commit(MODEL_MASK_WRITTEN);             // 0x2FE; 须失能
}
auto s = arm.model().status().value;                    // override / staged_mask / dirty
auto g = arm.model().get_gravity({0,0,0,0,0,0,0}).value;  // G(q)
```

### 数据采集 (`arm.log()`)

```cpp
arm.log().start(300);                                   // 录 300 拍后自停
LogReader r = arm.log().reader();
uint32_t total = r.total();                             // 已录字节数
auto samples = r.samples();                             // 逐拍解析
// 便捷: 采 + 等录满 + 读回
auto s2 = arm.log().capture(600);
arm.log().dump("trace.bin", /*wait=*/true);             // 原始字节落盘
```

### 固件自检 (`arm.diag()`)

```cpp
KinBenchResult kb = arm.diag().kin_bench().value;
printf("crc 坏帧 %lld, 应答丢弃 %lld, CAN TX 失败 %lld\n",
       kb.crc_errors(), kb.reply_dropped(), kb.can_tx_fail());
```

### 授权 / 激活

```cpp
LicenseInfo lic = arm.license();                        // 未激活也正常返回, 不抛
printf("%s uid=%s\n", lic.state_name().c_str(), lic.uid_hex().c_str());
arm.disable();
arm.activate(cust_id, issued, flags, mac16, 16);        // 须失能; mac 由厂商签发
```

### 持久化

```cpp
arm.disable();
arm.save_params();          // ⚠ 写 flash, 不可逆
```

### 只读属性

```cpp
arm.n();                    // 关节数
arm.firmware();             // 版本串, 如 "Litearm1.8.0-7J" (来自握手 0x41, 权威)
arm.fw_version();           // std::optional<FirmwareVersion>
arm.cart_supported();       // 固件是否编进了 LITEARM_CART_PLAN
arm.last_reset_reason();    // "normal" / "iwdg-rst" / "" (未收到开机签名)
arm.zero_g_active();        // 保活是否仍在维持
arm.port_string();          // **实际**连着的端口 (未连接时回退到构造入参)
```

诊断用的加法 (**全部与 Python 参照分叉**, 参照侧一个都没有; 都是纯本地快照,
**终态下也不抛** —— 现场最需要它们的时候正是链路已经废掉的时候):

```cpp
arm.is_connected();         // 读线程活着 **且** 状态帧没过期 (两个条件缺一不可)
arm.is_in_dfu();            // 是否已进 DFU 终态
arm.status_seq();           // 成功解码的状态帧累计计数
arm.msg_hz(0x40);           // 该类上行帧的平均到达频率 (Hz; 分母是实际到达间隔)
arm.banner_version();       // std::optional —— 开机横幅里的版本串 (横幅只在开机发一次)
arm.host_stats();           // 宿主机侧计数快照, 见 `struct HostStats`
arm.options();              // 当前生效的选项 (**现值**, 不是构造期快照)
```

### 可调旋钮

```cpp
arm.q_tol = 0.03;           // 到位判据: 关节角容差 (rad)
arm.dq_tol = 0.10;          // 到位判据: 关节速度容差 (rad/s)
arm.arrive_frames = 3;      // 连续静止拍数
arm.move_timeout = 15.0;    // 动作等待窗口 (秒) —— ⚠ 改它要配 reconnect()
```

## 返回值信封 `Msg<T>`

11 个"读一帧"型 getter 返回 `Msg<T> { value, hz, timestamp }`:

| 字段 | 含义 |
| --- | --- |
| `value` | 原返回值。取不到帧的入口 (`get_state` / `get_tcp`) 用 `std::nullopt` 表达 |
| `hz` | 该类帧在本会话里的**平均到达频率** (自首次收到起算, 不随链路空闲衰减) |
| `timestamp` | 该类帧**最近一帧**的本地单调时刻; 从没收到过则是 `0.0` |

被包的 11 个: `get_state` / `get_status_now` / `get_tcp` / `get_ff_vec` /
`get_ff_scalar` / `params().get_joint_param` / `model().get_body` / `model().get_jm` /
`model().status` / `model().get_gravity` / `diag().kin_bench`。

**刻意不包的**: `move_*` / `home` (它们是**动作结果**, 不是"读一帧");
纯本地量 (`n()` / `firmware()` / `last_reset_reason()`); `license()` (没有固件发起的
流量 —— 那样 `hz` 只会是"调用方自己的轮询频率"); 以及两个派生 getter
(`get_ff_mask` / `all_joint_params`, 它们是聚合或标量投影)。

`hz == 0.0` 的两种情况: ① 该类帧**从没到过** (此时 `timestamp` 也是 0);
② 只到过一帧 —— 一个样本定不出频率。单发请求/应答式的入口 (`get_joint_param` /
`get_body` / `get_jm` / `get_gravity`) **第一次调用必然 `hz == 0.0`**, 第二次起等于
**调用方自己的轮询频率**, 不是固件的什么周期。

## 命令行

```bash
./01_hello                      # 等价于 CLI 的 status
./04_ik_tcp
./02_movej --go 0.1 0 -0.1 0 0 0 0
```

SDK 里也带一个简易 CLI (`litearm::main_impl`), 便于巡检:

```bash
litearm-cpp status              # 只读
litearm-cpp fw                  # 版本串 + 轴数
litearm-cpp tcp                 # 当前位姿 + 帧率
litearm-cpp movej -0.1 0 0 0 0 0 0 --speed 0.3
litearm-cpp home
```

## 注意事项

### 多进程: fork 出来的子进程不能沿用父进程的 `Arm`

命令**真的会发到线上**, 但父进程的读线程会把应答吃掉 —— 你只会看到"无应答"超时,
而重试就意味着**把命令发第二遍**。读状态更隐蔽: 它不报错, 只是**永远返回继承来的陈旧值**。

所以本库是 **fail-closed** 的: 子进程里任何命令立刻抛 `ForkedSessionError`,
一个字节都不会出去。

子进程要用臂, **父进程必须先 `close()` 释放端口**, 再 fork, 然后在子进程里新建 `Arm`。

⚠ 子进程里 `close()` **仍可调** (它只清会话状态、不碰传输层, 所以不会挂死), 但别指望它
释放串口。

C++ 里这一条的落法比 Python 更需要说清楚 —— 子进程里 `close()` **故意泄漏两个句柄**:

- 那层 `Ack` 不能被销毁: 它的析构会 `join` 一条**在子进程里不存在**的线程, 而
  `std::thread::join()` 对不存在的线程会**永久阻塞**、`~std::thread` 对没 join 的线程会
  `std::terminate`。故把它 `release()` 掉, 交给内核回收。
- 传输层同理: `SerialTransport::close()` 要取的那把读锁, 在 fork 那一刻**很可能正被父进程的
  读线程持着**, 而能解锁的那个线程不在子进程里 —— 取它就是死锁。

子进程要么很快退出, 要么会新建自己的 `Arm`, 所以这份泄漏是有界的。

**⚠ 真正会挂死的那一步, 是析构一个"有人停在上面的" `std::condition_variable`**
(实测定位, glibc 2.35): 父进程的读线程正停在 `Ack` 的 `stop_cv_` 上, `fork()` 之后在子进程里
析构那个 condvar —— glibc 会去取 condvar 自己的内部锁 (组切换), 而那把锁与已经**不存在**的
等待者绑在一起 ⇒ 永久阻塞。最小复现: 一个线程 `cv.wait_for(...)` 停在 condvar 上, `fork()`
之后删掉那个 condvar。 (同一个实验里: 析构**没人在等**的 condvar / mutex 都没事; `detach()`
继承来的线程句柄也不抛; `malloc`/`free` 也安全 —— 问题**只**出在"有人停在上面的 condvar"。)

所以本库的规则是: **子进程里绝不析构继承来的会话对象**。走支持的路径
(`close()` / 离开作用域) 时这条自动成立; 万一有人绕过去直接析构, 库会**出声告警**
(而不是静默挂死) —— 与全仓"丢帧不静默"同一口径。

(测试: `tests/test_fork_guard.cpp` 真的 `fork()` 出来验, 7 条用例 —— 文件描述符继承、
"线程不被复制"、以及"condvar 析构会挂"都是操作系统的事实, 任何桩都模拟不出来。)

### 安全规则

1. **`disable()` 之后位置环不再抱住臂** —— 有负载会下坠。
2. **`movej` 不检查关节限位** —— 越界目标被固件钳到限位, 臂**仍会走完全程**。
3. **`move_js` / `send_mit` 需要 ≥10Hz 保活** —— 否则 0.1s 看门狗丢掉刚度, 臂慢慢软下来。
4. **`enter_dfu()` 是终态操作** —— 之后每个入口都失效, 烧完固件要新建 `Arm`。

### 不可逆命令: 别在已标定的臂上跑

下面这些会覆盖或抹掉**这台臂逐台标定**的动力学模型, **没有撤销**:

| 入口 | 效果 |
| --- | --- |
| `save_params()` | 把当前 RAM 写进 flash |
| `arm.model().commit()` | 让 staged 的动力学模型改动生效 |
| `arm.model().revert()` | 回退动力学模型 (不动 flash, 所以重新上电会复活) |
| `arm.params().reset_factory()` | 恢复出厂设置 |

只在一台标定没有任何价值的板子上做这些事。
**`arm.model().set_jm()` 永远不该被调用** —— 错的关节映射会让臂乱摆, 且没有可靠的回退路径。

压测 CAN 链路时只跑 `candump` (只读), 绝不 `cangen`: `can0` **就是**电机总线。

## 例子

见 [examples/README.md](examples/README.md):

- `01_hello` —— 握手 + 固件版本 + 读状态
- `02_movej` —— 关节运动
- `03_move_p` —— 笛卡尔点到点
- `04_ik_tcp` —— 逆运动学与当前位姿
- `05_ff_tune` —— 动力学 / 控制律调参
- `06_cartesian` —— 笛卡尔直线 / 圆弧 / 多路点
- `07_vel_jitter_trace` —— 100Hz 状态流 + 300Hz 固件日志的双路逐拍采集

样例**默认只读**; 任何会动的都要 `--go`:

```bash
source env.sh
./run_example.sh 01_hello
./run_example.sh 02_movej --go
```

## 测试

```bash
./build.sh                  # 配置 + 构建 + ctest (全离线, 从不碰硬件)
ctest --test-dir build -R test_cart -V
```

21 个测试二进制, 覆盖协议/CRC/旋转/错误码/状态帧/串口(真 pty)/会话装配/命令面/
**帧归属**/**命令覆盖**/**固件协议同步**/
前馈/关节参数/模型/采集/自检/笛卡尔/零重力/授权/并发/**fork 守卫**/CLI。
**不需要任何硬件** —— 测试用一份脚本化的假传输 (`litearm::testing::FakeTransport`),
而串口本身用**真 pty** 测 (不是桩: 桩会把 `timeout` 整个忽略掉, 于是"`read_frame(0)`
其实一个字节都不碰"这类缺陷在桩上全绿)。

### 未验证登记册

仓库根目录的 `UNVERIFIED_REGISTER.md` 逐条登记「**没验证的 / 已知不完美的 / 有意与参照实现
分叉的**」—— 每条**必须带出处**（`文件:行号`），否则等于没登记（现场复核不了）。
报告里说"测过了"之前先扫一眼它。

⚠ 这条纪律**本身有机器判据**（`tests/test_register.cpp`）：列数、出处形状、id 连号、
以及"**丢了行首 `|` 的隐形行**"（那种行读起来还像一条登记，而机器完全看不见它）。
判据只查**形状**，查不了出处是否**还指得到那句话** —— 这条限制也登记在册（第 18 条）。

### 与固件的协议同步 (`test_protocol_sync`)

这一套**需要固件仓库**才能跑, 默认 **SKIP**(不是通过 —— 假绿比没测更糟):

```bash
LITEARM_FW_DIR=/path/to/litearm-stm32 ctest --test-dir build -R test_protocol_sync -V
```

它直接解析固件源码 (**不需要编译固件**), 双向强制比对:

- 固件每条 `CMD_*`/`RSP_*` 在 SDK `protocol.hpp` 里**同名同值** (反向亦然 —— SDK 不许声明
  固件没有的命令);
- 固件每条已实现命令在 `COMMAND_COVERAGE` 里都有登记, 且**没有过期登记**;
- 三张豁免表 (`FIRMWARE_ONLY_CMDS`/`FIRMWARE_ONLY_RSPS`/`PREEXISTING_GAPS`) 仍然成立;
- 状态帧布局仍是 `6 + N*21`、每关节步长仍是 21B;
- `joint_cfg.h` 的 `LITEARM_BENCH_MODEL_AXIS` 与 SDK 常量一致;
- `ERR{cmd,0x00}` 仍**只**由固件的 `default` 分支产生 (SDK 的能力判定全押在这条上);
- 固件自报版本不低于 `MIN_FW`。

**为什么必须有它**: 固件 1.5.x 把状态帧从 `4+21N` 改成 `6+21N` 时漏同步了 SDK, 而桩测试
整替换 transport、自造旧布局 —— **离线全绿、真机必挂**。人工核对挡不住这类事。

⚠ **SKIP 会在报告里单列**: 跑完若看到 `<N> 跳过`, 说明同步那一半**没测**, 别读成测过了。

## 与原版 (litearm-python 2.1.0) 的关系

**逐条对照移植**, 线协议与语义完全一致; 差异只在语言层面:

| 主题 | Python | C++ |
| --- | --- | --- |
| 位姿入参 | 运行期判形态 (`as_pose` 收 list/tuple) | `rot::PoseInput`, 三种写法各有具名工厂; 6 向量可隐式转换 |
| 关节数 | `arm.n` 属性 | `arm.n()` 方法 |
| 可调旋钮 | `arm.move_timeout = ...` | 同名公开成员, 赋值即可 |
| 取不到的值 | `None` | `std::nullopt` |
| 零重力上下文 | `with arm.zero_g():` | `{ auto zg = arm.zero_g(); ... }` (RAII) |
| 收尾 | `__del__` 兜底 | 析构兜底 (同样只委托给 `close()`) |
| 测试框架 | pytest | 自研极简框架 (零依赖, 每个 `test_*.cpp` 一个可执行文件) |

### 本移植**有意**与原版分叉的四组 (2026-09-28 起)

> 这三组都是照着另一套独立 C++ 实现 (Gitee `yudao_hz_1/litearm-cpp` 的 `feat/p0-protocol-layer`)
> 对齐的结果。**它与本仓在共同覆盖的范围内与 Python 参照逐字节一致** (用它的金样本驱动本仓
> 跑过 19/19), 分歧只在下面这三处取舍上。

#### ① 同帧节流: **整套删除** (原版有, 本仓不做)

原版的 `_tx_allowed` / `set_tx_repeat_min_interval` 允许节流**任何**帧, 后果是: 一条
**等 ACK** 的命令被丢掉之后帧压根没上 USB ⇒ 固件永远不会应答 ⇒ 调用方只能**等满窗口**再报
"无应答", 归因**指向反了**。

本仓**早先**的做法是把射程收窄到"声明为可丢弃"的写口 (只有零重力保活那样写)。**现在改成
整套删除**, 理由是那套机制在参照实现里**私有且默认全关**(行为上等价于无), 不值得为它留一条
可能被误用的路径:

- `Arm::set_tx_repeat_min_interval()` 与 `Arm::tx_throttled_frames()` **已移除**;
- `raw_write` 的 `droppable` 形参**已移除**;
- ⇒ `raw_write` 的新不变式更强: **进了这个函数就一定会写出去**, 没有"清队了但帧没发"
  这种半状态。判据在 `tests/test_threading.cpp` 的
  `threading_every_raw_write_reaches_the_wire_there_is_no_drop_path`。

#### ② 客户端预检: **新增四条** (原版没有, 由固件钳制)

全部在**发帧之前**本地拒发。共同理由只有一条: 参数错到"比较全为假"时, 静默放行的后果是
**臂真的动了**而调用方无从察觉。

| 预检 | 判据 | 落地 |
|---|---|---|
| 非有限目标 | 目标含 NaN / ±inf ⇒ 抛 | `precheck_q_`, **排在"缓存空 ⇒ 放行"之前** |
| 软限越限 | 超出 `[q_min, q_max]` ⇒ 抛 | `precheck_q_`; 缓存来自 `connect()` 末尾读的 `0x24` |
| 速度 | `!(speed >= 0 && speed <= 1)` | `precheck_speed_`; **`move_p` 那只是六个运动入口里唯一原版没有的** |
| 容差 | `!(pos_tol > 0) \|\| !(rpy_tol > 0)` | `move_p` 内联; `dp >= NaN` 为假会**让位置判定被静默跳过** |

⚠ 两条**刻意的不拦**: `home()` **绝不**预检 (臂漂出软限位后, 恢复路径不该被自己的预检堵死);
`move_js` / `send_mit` / `send_mit_all` 是透传命令, 不做预检。

⚠ 代价要说清: `connect()` 因此**多出 n 条 `0x24`** (参照实现的 connect 不读软限), 读不到时
**fail-open**(不预检、放行, 绝不让连接失败)。判据在 `tests/test_precheck.cpp` (13 条)。

#### ③ 诊断访问器: **新增七个** (原版没有)

`is_connected()` / `is_in_dfu()` / `status_seq()` / `msg_hz()` / `banner_version()` /
`host_stats()` / `options()`。全是**纯本地快照、终态下也不抛** —— 现场最需要它们的时候正是
链路已经废掉的时候。判据在 `tests/test_accessors.cpp` (11 条)。

⚠ `options()` 返回**现值**而不是构造期快照 (与本仓"旋钮是公开成员、运行期可改"的设计一致);
本仓的 `port_string()` 也顺带修好了 —— 它原先返回**构造入参**, 走自动发现时恒为空串,
**而头文件注释承诺的是"或自动发现到的"**(注释与实现脱节; 真机复现过)。

#### ④ 时间源: **可注入** (原版没有)

```cpp
auto clk = litearm::testing::FakeClock::make();
litearm::ArmOptions opts;
opts.port = "fake";
opts.clock = clk;                  // 空 = SteadyClock (生产默认)
```

⚠⚠ **本仓的时钟只管"现在几点", 不管"怎么等"** —— 这是与 A 侧（Gitee 那份）的一处
**刻意不同**: A 的 `Clock` 还有 `sleep_until(t)`（假钟 = 把虚拟时间跳到 t 并立刻返回），
本仓**不提供**它。理由: 本仓的等待**清一色**是 `condition_variable::wait_for`, 靠的是
**有帧到达时被 notify 立刻唤醒**; 把睡眠也搬进时钟就会丢掉这条, 每次等待都要等满一整片。

⇒ 本仓的语义是: **deadline / 到期比较 / 时间戳**走注入的钟; **等待**走条件变量的**真实**时间。

⚠⚠ **由此有一条必须知道的后果**: 注入一个**不会自己前进**的假钟时, `now + timeout`
这类 deadline **永远到不了** ⇒ 凡"等超时真的发生"的判据会**挂死**（不是失败，是挂死）。
所以假钟只给**纯比较**类判据用（到期比较、时间戳、TTL 水位，那些不等待）。
A 侧的结论一致（它的注释也写"凡'超时真的发生'的判据要用 SteadyClock"），
只是那边退化成 skip，这边是挂死。

**它的价值在一条具体的用例上**：「多久没收到帧算失联」（`STATUS_STALE_MAX_S` = 2.0s）
本来要**真睡满 2 秒**才验得到；用假钟 `advance(3.0)` 是**瞬间**的，而且断言里不再混进
"调度器会不会按时醒来"。判据在 `tests/test_clock.cpp`。

⚠ **不跟着注入的**: 传输层（`transport.cpp`）的时间源**保持真钟** —— 那里的
`poll` / `tcdrain` 超时是**硬件**超时，注入假钟是**错的**，不只是没用。

### 已知继承的差异/缺口 (都**不是**移植引入的, 而是在原版里就存在):

- `license()` 在**低于 1.8.0** 的固件上报的是"**无应答超时**"而不是"固件没有这条命令":
  它调 `expect` 时没传 `echo_cmd`, 于是那条 `ERR{0x2F,0x00}` 落到别的队列里, 本入口看不到。
  逐条证据与修法见 `tests/test_license.cpp` 里那条用例。移植时刻意**保留原语义**。
- `get_status_now()` 在固件**拒绝** `0x40` 时可能**报成功** (返回一帧状态, 而非
  `UnsupportedByFirmwareError`)。原因是它的结论判据后半条是"**有任何**新状态帧到" ——
  而固件的 100Hz 被动流与我们的 `0x40` 是两条独立的线: 若在 `seq0` 采样与首次判据之间
  落进一帧状态帧, 判据当场成立, 而那条 `ERR{0x40,code}` 可能还没被投递, 于是函数查一眼
  ERR 队列 (空) 便落到"返回成功"。
  **量化** (2026-09-28, 实验见下): 调用期间**一帧状态帧都没交付** ⇒ 150/150 全部正确抛出;
  交付了恰好 1 帧 ⇒ 150 次里 26~35 次报成功。TSan 下 `tests/test_commands.cpp` 的
  `cmd_unsupported_during_status_read_...` 因此约 1/15 概率翻红。

  ⚠⚠ **同一根因还有第二处表现, 后果比上面重得多 —— 它在安全守卫上 fail-open**:
  `Arm::reject_if_cart_in_flight()` (零重力入口的反向守卫) 靠 `get_status_now()` 现取一帧
  判 `cart_busy`; 取不到时**保守拒绝** (`CART_IN_FLIGHT_UNCONFIRMED_MESSAGE`)。而上面这个
  窗口让它"误以为取到了" —— 拿到的是**一帧陈旧的状态帧**, `cart_busy()` 为 0 ⇒ **守卫放行**。
  正是该守卫存在的理由那一侧: "轨迹中途进场 ⇒ 臂靠摩擦滑停 (coast)"。
  实测 TSan 下 `zero_g_is_refused_conservatively_when_the_status_is_unavailable`
  30 次里翻红 3 次。

  两条用例现已用 `lt::quiesce()` (关被动流 + 等在途帧落定) 把变量摘掉, 只钉各自要钉的
  那件事 (ERR→异常映射 / 保守拒绝)。**别把它们改回"直接在活跃链路上断言"** ——
  那会重新变成两条测时序的用例。

  **为什么不修**: 状态帧里**没有命令回显** (只有 ACK/ERR 有), 所以"这一帧是不是我们那条
  命令的应答"在协议层面**不可判定**; 给失败分支加宽限窗口只能把窗口压小、关不严 ——
  而一个"看起来修好了"的半修, 比一个明写着的已知限制更危险。加上这条路径在受支持的
  固件上**不可达**: `connect()` 的版本门已经挡掉了没有 `0x40` 的旧固件。
  ⇒ 保留原语义, 记在这里 + `src/arm.cpp` 的 `done` 谓词旁。
- `linux` 之外**不做 CDC 自动发现**: macOS 没有 sysfs、Windows 需要 SetupAPI 才能可靠读到
  VID:PID, 猜一个会给出假阳性。请显式传端口。这是刻意的取舍, 不是遗漏。
- **USB 重新枚举之后的第一次 `connect()` 会握手超时**（`get_firmware 无应答`），等几秒重试即好。
  真机实测 (2026-09-28): 当天 **4 次**重新插上/重新直通之后**每一次首次连接都失败**，
  重试全部成功；热链路下从未出现。
  ⚠ 参照实现**同样不重试**（它的 `connect()` 也是发一次 `0x41`、等 1.5s、失败就关链路抛）
  ⇒ 这是**继承行为**，不是移植引入的。移植时刻意保留原语义。
  现场用法: 连不上就**隔一两秒重试一次**，别急着怀疑固件或线。
  （登记册第 21 条；要不要给它加一次有界重试是个**未决的取舍** —— 见那里。）

### TSan 那一批告警: **已定论 —— 误报, 且有运行时证据**

`./build.sh --tsan` 会在 `Ack::queues` 上报**大量** data race (单用例就有几十条)。
**结论是误报**, 依据不是"看代码觉得像", 而是**运行时证据**:

`Ack::MuLock` (`include/litearm/ack.hpp`) 在 **Debug 构建**下会登记 `mu` 的持有者,
于是 `queues` / `state` / `recv_` 的每一处访问都能断言"我确实在锁内"。开着这套断言跑
**全部 308 条用例**: 断言**一次都没触发**, 而 TSan 照样报那几十条 ⇒ 那些访问在运行时
**被证实在锁内**。

反过来说, 这套断言现在是**常设的**: `Ack::queues` 的"必须在 `mu` 之下"这条不变量以前
只靠**人眼审**那 6 处访问点, 一旦被破坏就是**静默的数据竞争** (最难查的一类)。
现在它被破坏会在**下一次调试构建里当场 abort 并指名站点**。Release 下退化成
`std::unique_lock`, 零额外开销。

**怎么用 TSan**: `./build.sh --tsan` (本机需要 `setarch -R` 关 ASLR, 脚本已自动加)。
脚本会设 `TSAN_OPTIONS=halt_on_error=0:exitcode=0` —— 少了这两个开关,
**打印着"全部通过"的用例会被 ctest 判成 Failed**: 前者是"检测到就当场 exit(66)"让断言
跑不到, 后者是"跑完了也把退出码设成 66"。这条踩过, 别删。

⚠⚠ **但 `exitcode=0` 有一个反噬 (2026-09-28 实测, 比上面那条更阴)**: TSan 的**死锁检测器**
只有一个 **64 格的固定数组** (`sanitizer_deadlock_detector.h:67`), 被它同时追踪的 mutex
超过 64 个就 `FATAL: ThreadSanitizer CHECK failed (0x40, 0x40)` **当场自杀** —— 而自杀走的
同样是 `internal__exit(exitcode)` ⇒ **退出码 0** ⇒ **ctest 报 Passed**。
⇒ 任何"在一次运行里创建几十个 `Arm`"的用例都会撞上它, 于是**跑到一半就没了, 报告却是绿的**。
⇒ **判据不是 ctest 那一行, 而是框架自己那行汇总** (`全部通过: N 通过 / M 失败`) 有没有出现,
以及 stderr 里有没有 `FATAL: ThreadSanitizer CHECK failed`。
本仓现有 21 个套件都**没有**撞上 (逐个直跑核对过); 临时绕过用
`TSAN_OPTIONS=...:detect_deadlocks=0`。
⚠ 同一次分诊里还发现: 那条 `data race ... tests/test_zero_g.cpp:18 in count_zero_g_on`
**不是误报** —— 它是测试辅助函数在**不持 `tx_lock`** 的情况下遍历 `tx_log`, 而保活线程正在
并发 `emplace_back` (扩容会把正在遍历的缓冲区释放掉)。10 个测试文件里 36 处都是这么写的,
现已统一改走持锁的 `tx_snapshot()` / `tx_count()`。

## 平台

| 平台 | 串口后端 | 状态 |
| --- | --- | --- |
| Linux | POSIX termios + `flock` + `poll` | 已实测 (含真 pty 测试) |
| macOS | POSIX termios + `flock` + `poll` | 同一份代码路径; 但**端口自动发现不可用** (无 sysfs), 请显式传端口 |
| Windows | Win32 `CreateFile`/`SetCommState`/`ReadFile`/`WriteFile` | 已实现但**未在 Windows 上实机验证**; 端口自动发现同样不可用 |
| Linux / aarch64 | 同 Linux (交叉编译) | **只写了工具链文件, 一次都没编过** —— 本机没装 aarch64 工具链。见「安装与打包 / aarch64 交叉编译」 |

⚠ **真机验收的范围**（2026-09-28）：**全部可逆功能已跑通** —— 14 组 / 68 项断言 / 218 帧，
**每组前后都回零**，含只读访问器全家、关节与笛卡尔运动、连续伺服、零重力、前馈与关节参数
（读原值写回）、采集、自检、使能失能往返、急停与恢复；逐帧审计确认**没有发出任何不可逆命令**。

**仍未在真机上跑**：四项不可逆（`save_params` / `model().commit()` / `reset_factory` /
`enter_dfu` —— 用户裁决排除）＋ `activate()`（本板已激活）。详见 `UNVERIFIED_REGISTER.md`
第 14 条。别把"大部分验过"读成"全验过"。

Windows 上用 MSVC + CMake 构建即可 (`cmake -S . -B build -G "Visual Studio 17 2022"`)。
`env.ps1` / `run_example.ps1` 见仓库根目录。

## 许可

本移植沿用上游随仓的 `LICENSE` 文件原文 (**Apache License 2.0**)。
⚠ 上游 `README.md` 写的是 "MIT", 而它自己的 `LICENSE` 文件是 Apache-2.0 —— 那份不一致是
上游就有的; 本仓以 LICENSE 文件为准。
