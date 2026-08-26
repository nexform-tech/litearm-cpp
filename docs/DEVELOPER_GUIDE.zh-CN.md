# litearm-cpp 开发指南与接口说明

`litearm-cpp` 是 LiteArm 机械臂的 C++ 客户端 SDK（C++17），
通过网络连接机械臂控制服务，适合低延迟嵌入式场景。**纯客户端，无运动学/动力学依赖**。

```text
你的程序 ──→ 机械臂控制服务 ──→ 机械臂 / CAN
```

---

## 目录

- [1. 环境要求与构建](#1-环境要求与构建)
  - [1.1 安装 protobuf](#11-安装-protobuf)
  - [1.2 构建](#12-构建)
- [2. 快速开始](#2-快速开始)
- [3. 连接管理](#3-连接管理)
- [4. 值类型 LiteArmValue](#4-值类型-litearmvalue)
- [5. 接口说明](#5-接口说明)
  - [5.1 计算（不驱动电机）](#51-计算不驱动电机)
  - [5.2 运动控制](#52-运动控制)
  - [5.3 状态读取](#53-状态读取)
  - [5.4 急停 / 使能](#54-急停--使能)
  - [5.5 DIRECT 模式 —— 逐帧 MIT 直接控制](#55-direct-模式--逐帧-mit-直接控制)
  - [5.6 参数调节](#56-参数调节)
  - [5.7 外设设备](#57-外设设备)
  - [5.8 系统 / 设置 / 轨迹 / 设备管理 / 遥操](#58-系统--设置--轨迹--设备管理--遥操)
- [6. 异常处理](#6-异常处理)
- [7. 安全提示](#7-安全提示)
- [8. 与 litearm-python 对比](#8-与-litearm-python-对比)

## 1. 环境要求与构建

| 项目 | 要求 |
|---|---|
| 编译器 | C++17（GCC 8+ / Clang 7+） |
| CMake | 3.16+ |
| Protobuf | 3.19+（headers、库、`protoc`） |
| Google Test | 仅测试需要（自动下载） |

### 1.1 安装 protobuf

先检查版本：

```bash
protoc --version    # 需 "libprotoc 3.19" 或更新版本
```

若未安装或版本过低，按需选择一种方式：

```bash
# Debian / Ubuntu（24.04+ 自带 protobuf 3.21）
sudo apt update && sudo apt install -y protobuf-compiler libprotobuf-dev

# 或 conda（任意系统）
conda install -c conda-forge protobuf=3.19.6

# 或源码编译（任意系统）
cd /tmp
curl -LO https://github.com/protocolbuffers/protobuf/releases/download/v3.19.6/protobuf-cpp-3.19.6.tar.gz
tar xzf protobuf-cpp-3.19.6.tar.gz && cd protobuf-3.19.6
./configure && make -j$(nproc) && sudo make install
sudo ldconfig
```

### 1.2 构建

```bash
mkdir build && cd build
cmake ..                      # 系统安装的 protobuf 会被自动找到
cmake --build . -j$(nproc)
ctest --output-on-failure     # 运行测试
```

protobuf 在自定义前缀（例如已激活的 conda 环境）时：

```bash
cmake .. -DPROTOBUF_ROOT=$CONDA_PREFIX
```

## 2. 快速开始

```cpp
#include <litearm/arm.hpp>

int main() {
    auto arm = litearm::Arm("tcp/192.168.1.100:7447", "armA");

    auto state = arm.get_state();                 // std::optional<RobotState>
    if (state) {
        auto& q = state->q;                        // 关节位置
    }

    arm.movej({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*speed=*/0.5);
    auto [pos, rot] = arm.fk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    arm.request_stop();                            // 高优先级急停
    arm.close();
    return 0;
}
```

## 3. 连接管理

```cpp
litearm::Arm arm("tcp/127.0.0.1:7447", "armA",
                 /*transport=*/nullptr);   // 可选预配置连接（高级用途）
arm.close();                               // 关闭连接
```

- `Arm` 不可拷贝、可移动；`arm_id()` 返回机械臂标识。
- `get_state(refresh=false)` 同步读取状态缓存，未收到返回 `std::nullopt`。

## 4. 值类型 LiteArmValue

接口参数/返回值统一用动态值类型 `LiteArmValue`（tagged union），构造与访问：

```cpp
using litearm::LiteArmValue;

LiteArmValue v = LiteArmValue::from_vec({0.1, 0.2});            // vector<double> → List
LiteArmValue m = LiteArmValue::from_mat({{1,0,0},{0,1,0},{0,0,1}});  // 嵌套 → List<List>

double x  = v.as_list()[0].as_double();        // 访问
auto vec  = v.to_vec();                        // 提取 vector<double>
auto mat  = m.to_mat();                        // 提取 vector<vector<double>>
auto cfg  = LiteArmValue::make_map({{"type", LiteArmValue("gripper")}});
```

`Kind` 枚举：`Null / Bool / Int / Double / String / Bytes / List / Map`。便捷别名 `Vec7`、`Mat3x3`。

## 5. 接口说明

> 运动类方法返回 `bool`；纯计算返回数据；其他接口返回 `LiteArmValue`。

### 5.1 计算（不驱动电机）

| 方法 | 说明 |
|---|---|
| `fk(q)` | 正运动学 → `(位置, 旋转矩阵)` |
| `ik(pos_d, R_d, q_seed=nullopt)` | 逆运动学 → `(q, 是否成功)` |
| `plan_movel(q_start, pose_goal)` | 直线笛卡尔路径规划 → 关节路径 |
| `plan_movec(q_start, pose_via, pose_goal)` | 圆弧路径规划（过中间点） |
| `plan_movep(q_start, poses_goal)` | 多航点路径规划 |

### 5.2 运动控制

| 方法 | 说明 |
|---|---|
| `movej(q_target, speed=1.0, settle_s=1.0, max_cycles=nullopt, allow_start_collision_recovery=false)` | 关节空间点到点 |
| `home(speed=0.3, settle_s=0.5, max_cycles=nullopt)` | 回零：所有关节归零，绕开限位和自碰路径检查 |
| `recover_joint_limits(speed=0.05, settle_s=0.5, max_cycles=nullopt, inset_rad=0.0)` | 越限关节缓慢回安全边界（需 server `allow_limit_recovery=True`） |
| `movel(pose_goal, speed=1.0, settle_s=0.8, max_cycles=nullopt)` | 笛卡尔直线 |
| `movec(pose_via, pose_goal, speed=1.0, settle_s=0.8, max_cycles=nullopt)` | 笛卡尔圆弧 |
| `movep(poses_goal, speed=1.0, settle_s=0.8, max_cycles=nullopt)` | 多航点带拐角平滑 |
| `replay_joint_path(q_path, speed=1.0, settle_s=0.5, goto_start=true, goto_speed=0.3, max_cycles=nullopt)` | 回放关节序列 |
| `replay_trajectory(traj, speed=1.0, goto_start=true, goto_speed=0.3, max_cycles=nullopt, check_singularity=true)` | 回放 `JointTrajectory`（另有 `LiteArmValue` 重载） |
| `replay_timed_trajectory(traj_q, traj_t, speed=1.0, goto_start=true, goto_speed=0.3, simplify_tolerance_rad=0.01, max_cycles=nullopt)` | 按原始时间轴回放（自动拉伸保安全） |
| `play_trajectory(trajectory, speed=1.0, goto_start=true, goto_speed=0.3, verify_robot=true, simplify_tolerance_rad=0.01, max_cycles=nullopt)` | 回放已保存轨迹（`JointTrajectory` 或 server 侧路径字符串，双载） |
| `record_trajectory(output="trajectories", duration_s=nullopt, sample_rate_hz=100.0, filter_alpha=0.15, name=nullopt)` | 拖动录轨迹 → `JointTrajectory` |
| `hold(kp_scale=3.0, max_cycles=nullopt)` | 提高刚度持位 |
| `zero_gravity(max_cycles=nullopt, duration_s=nullopt, measured_overspeed_factor=nullopt, vel_max=nullopt)` | 零重力（自由拖动）模式 |
| `joint_impedance(q_des, K, B, tau_max=nullopt, engage_sec=0.3, max_cycles=nullopt)` | 关节空间阻抗控制 |
| `cartesian_impedance(q_des, K_cart, B_cart, v_des=nullopt, tau_max=nullopt, engage_sec=0.3, max_cycles=nullopt, sigma_min_thresh=nullopt, max_ori_err=nullopt, measured_overspeed_factor=nullopt, vel_max=nullopt)` | 笛卡尔空间阻抗控制 |
| `joint_follow(K=nullopt, B=nullopt, speed_limit=nullopt, accel_limit=nullopt, engage_sec=0.3, max_cycles=nullopt, duration_s=nullopt)` | 跟随外部目标 |

### 5.3 状态读取

| 方法 | 说明 |
|---|---|
| `get_state(refresh=false)` | 状态缓存最近状态（`std::optional<RobotState>`，同步） |
| `get_tcp_pose()` | 当前 TCP 位姿 → `(位置, 旋转矩阵)` |

`RobotState` 字段：`q / dq / tau / faults / errs / temps / state / feedback / watchdog / robot_serial / config_checksum_sha256`。

### 5.4 急停 / 使能

| 方法 | 说明 |
|---|---|
| `request_stop()` | 高优先级急停（独立急停通道） |
| `clear_stop()` | 清除停止状态回到就绪 |
| `enable()` | 使能全部电机并锁住当前姿态 |
| `disable()` | ⚠️ 失能全部电机（机械臂会掉臂！），CAN 保持连接 |

### 5.5 DIRECT 模式 —— 逐帧 MIT 直接控制

> DIRECT 模式是 LiteArm 的逐帧 MIT 直接控制通道。通过 `send_mit` 以 250Hz 典型频率
> 发送五参数 (kp/kd/q_ref/dq_ref/tau_ff) 实时控制关节电机，内置 4 条永不关闭的核心安全护栏。

**与普通运动控制的区别：**

| 特性 | 普通运动控制 (`movej` 等) | DIRECT 模式 (`send_mit`) |
| --- | --- | --- |
| 控制方式 | 目标位置 + 速度，自动规划 | 逐帧五参数 MIT 命令 |
| 帧率 | 一次调用，自动执行 | 用户循环控制（典型 250Hz） |
| 阻塞 | 阻塞，等运动完成 | 非阻塞，立即返回 |

**进入与退出：**

- **进入**：首次调用 `send_mit` 时自动进入 DIRECT 模式
- **退出**：`request_stop()` 主动退出 / 看门狗超时自动回 hold / 电机故障自动退出

#### send_mit —— 发送 MIT 控制帧

**Description:** 异步 pub 五参数 MIT 控制帧到机械臂命令通道。非阻塞，立即返回。首次调用自动进入 DIRECT 模式。

**Function Definition:**

```cpp
void send_mit(
    const std::vector<double>& kp,      // 长度 7，位置刚度
    const std::vector<double>& kd,      // 长度 7，速度阻尼
    const std::vector<double>& q_ref,   // 长度 7，目标关节角度 (rad)
    const std::vector<double>& dq_ref,  // 长度 7，目标角速度 (rad/s)
    const std::vector<double>& tau_ff   // 长度 7，前馈力矩 (N·m)
);
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `kp` | `const std::vector<double>&` | 位置刚度，长度 7，范围 `[0, 500]`。典型值 15–200 |
| `kd` | `const std::vector<double>&` | 速度阻尼，长度 7，范围 `[0, 5]`。典型值 0.5–3.0 |
| `q_ref` | `const std::vector<double>&` | 目标关节角度（rad），长度 7。相邻帧跳变受斜率限制 |
| `dq_ref` | `const std::vector<double>&` | 目标角速度（rad/s），长度 7。被 clamp 到 `±DQ_MAX` |
| `tau_ff` | `const std::vector<double>&` | 前馈力矩（N·m），长度 7。被 clamp 到 `±min(guards_tau_max, TAU_MAX)` |

**Return Value:** `void` — 异步发送，不等待回执。

**Usage Example:**

```cpp
// 发送单帧（自动进入 DIRECT 模式）
arm.send_mit(
    {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0},
    {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
);
```

#### set_guards —— 配置全局护栏

**Description:** 全局一次性配置护栏参数（RPC）。所有参数为 `std::optional`，`std::nullopt` 表示不改变。**全局持久**：退出 DIRECT 后不重置。

**Function Definition:**

```cpp
LiteArmValue set_guards(
    std::optional<double> slew_limit = std::nullopt,
    std::optional<double> tau_max = std::nullopt,
    std::optional<double> watchdog_timeout = std::nullopt,
    std::optional<bool> position_bounds = std::nullopt,
    std::optional<bool> velocity_bounds = std::nullopt,
    std::optional<bool> jerk_limit = std::nullopt
);
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `slew_limit` | `std::optional<double>` | 全局斜率限制（rad/s）。`std::nullopt` = 不改变 |
| `tau_max` | `std::optional<double>` | 全局力矩上限（N·m）。`std::nullopt` = 不改变 |
| `watchdog_timeout` | `std::optional<double>` | 看门狗超时（秒），范围 `[0.05, 2.0]` |
| `position_bounds` | `std::optional<bool>` | 是否开启位置软限位。默认 `false` |
| `velocity_bounds` | `std::optional<bool>` | 是否开启速度软限位。默认 `false` |
| `jerk_limit` | `std::optional<bool>` | 是否开启加加速度限制。默认 `false` |

**Return Value:** `LiteArmValue` — RPC 回复值。

**Usage Example:**

```cpp
// 组合配置
arm.set_guards(1.0, 10.0, 0.10, true);
// slew_limit=1.0, tau_max=10.0, watchdog_timeout=0.10, position_bounds=true

// 仅修改单个参数
arm.set_guards(std::nullopt, std::nullopt, 0.05);  // 仅收紧看门狗到 50ms
```

#### get_guards —— 读取当前护栏配置

**Description:** 读取当前生效的护栏配置（RPC 同步）。

**Function Definition:**

```cpp
LiteArmValue get_guards();
```

**Return Value:** `LiteArmValue` — 包含 6 个字段的 map。

**Usage Example:**

```cpp
auto guards = arm.get_guards();
auto kw = guards.as_map();
std::cout << "slew_limit = " << kw["slew_limit"].as_double() << " rad/s\n";
```

#### 完整控制循环示例

```cpp
/** DIRECT 模式 250Hz 控制循环 —— 正弦波扫关节1。 */
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <signal.h>
#include "litearm/arm.hpp"
#include "litearm/transport.hpp"

static volatile sig_atomic_t running = 1;
void handle_signal(int) { running = 0; }

int main() {
    signal(SIGINT, handle_signal);

    constexpr double DT = 0.004;   // 4ms → 250Hz
    constexpr double FREQ = 0.5;
    constexpr double AMP = 0.5;
    constexpr int N = 7;

    auto tp = std::make_shared<litearm::ZenohTransport>("tcp/192.168.1.100:7447");
    litearm::Arm arm("", "armA", tp);

    // 配置护栏（一次性）
    arm.set_guards(2.0, 20.0, 0.10, true);

    double t = 0.0;
    while (running) {
        auto loop_start = std::chrono::steady_clock::now();

        std::vector<double> q_ref(N, 0.0);
        q_ref[0] = AMP * std::sin(2.0 * M_PI * FREQ * t);

        arm.send_mit(
            std::vector<double>(N, 50.0),
            std::vector<double>(N, 1.5),
            q_ref,
            std::vector<double>(N, 0.0),
            std::vector<double>(N, 0.0)
        );

        t += DT;
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        auto sleep_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(DT) - elapsed);
        if (sleep_ns.count() > 0) std::this_thread::sleep_for(sleep_ns);
    }

    arm.request_stop();
    std::cout << "已退出 DIRECT 模式" << std::endl;
    return 0;
}
```

#### 安全护栏说明

| 护栏 | 说明 | 可关闭？ |
| --- | --- | --- |
| **护栏1：协议参数 clamp** | kp≤500、kd≤5、dq_ref≤DQ_MAX、tau_ff≤TAU_MAX | 不可关闭 |
| **护栏2：命令通道斜率限制** | 相邻帧 q_ref 跳变 ≤ slew_limit × dt，dt 有上界 0.10s | 不可关闭，`slew_limit` 可收紧 |
| **护栏3：看门狗 fail-soft** | 命令中断超时自动回 hold | 不可关闭，`watchdog_timeout` 可调 |
| **护栏4：单一所有权** | DIRECT 激活时拒绝运动命令和遥操 | 不可关闭 |
| **位置限制（附加）** | q_ref 逐帧 clamp 到关节软限位 | 默认关，`position_bounds: true` 开启 |
| **速度限制（附加）** | dq_ref 逐帧 clamp 到 `±DQ_MAX` | 默认关，`velocity_bounds: true` 开启 |
| **加加速度限制（附加）** | dq_ref 变化率受限 | 默认关，`jerk_limit: true` 开启 |

> **安全底线：机械臂永远不允许乱飞。** 命令通道斜率限制 + 单一收口 + 看门狗 fail-soft + 固件兜底。

### 5.6 参数调节

| 方法 | 说明 |
|---|---|
| `set_gains(kp=nullopt, kd=nullopt)` / `get_gains()` | PD 增益设置/读取 |
| `clear_faults()` | 清除电机故障 |
| `set_payload(mass, com={0,0,0})` / `get_payload()` | 末端负载（质量 + 质心） |
| `set_installation(base_rpy=nullopt, gravity=nullopt)` / `get_installation()` | 安装姿态（基座 RPY 或重力向量） |

### 5.7 外设设备

```cpp
auto hand = arm.device("hand_0");          // RemoteDevice
hand.open(); hand.close();                 // 开/合
hand.set_force(0.5);                       // 抓取力
hand.set_gesture("pinch"); hand.list_gestures();
hand.finger_move({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
hand.set_speed({...}); hand.set_torque({...});
hand.get_state();

auto gripper = arm.device("gripper_0");
gripper.set_width(0.5); double w = gripper.get_width();

auto teach = arm.device("teach_0");
teach.get_joints(); teach.get_buttons();

// 通用：get_status / get_info / connect / disconnect / clear_faults
// 设备管理器：arm.devices()["hand_0"] 等价于 arm.device("hand_0")
```

`RemoteDevice::call(method, kwargs)` 可调用任意设备方法，自动加 `device.{id}.` 前缀。

### 5.8 系统 / 设置 / 轨迹 / 设备管理 / 遥操

均返回 `LiteArmValue`：

| 分组 | 方法 |
|---|---|
| 系统 | `get_system_stats()`、`get_logs(page=1, size=50, search="")`、`restart_service()`、`reconnect()` |
| 设置 | `get_joint_limits/set_joint_limits(limits)`、`get_zero_offsets/set_zero_offsets(offsets)`、`get_end_effector/set_end_effector(config)`、`get_cartesian_limits/set_cartesian_limits(limits)`、`get_collision_config/set_collision_config(config)` |
| 轨迹 | `start_recording/stop_recording/discard_recording/get_recording_state/get_playback_state/list_trajectories/save_trajectory(id,name,points,duration=nullopt)/delete_trajectory(id)` |
| 设备 | `list_device_types()`、`connect_device(category, subtype, device_id="end_0", can_iface="", config={})`、`disconnect_device(device_id="end_0")`、`get_active_device(device_id="end_0")` |
| 遥操 | `enter_teleop(mode, params={})`、`exit_teleop()`、`get_teleop_status()` |

> 遥操态下服务端拒绝一切手动控制指令，只放行只读 / 急停 / `exit_teleop`。

## 6. 异常处理

所有异常继承 `LiteArmError`（`std::runtime_error` 子类），服务端异常原样抛出。

```cpp
#include <litearm/exceptions.hpp>

try {
    arm.movej(...);
} catch (const litearm::SafetyViolationError& e) {   // 超时/跟随/故障/看门狗
    // e.details 为 unordered_map<string,string>
} catch (const litearm::LiteArmError& e) {           // 兜底
    std::cerr << e.what();
}
```

常用类型：`NotConnectedError`、`ConfigurationError`、`InvalidCommandError`、`CartesianPlanError`、
`MotionTimeoutError`、`MotorFaultError`、`ArmFault`、`WatchdogError`、`MotionCancelled`。

## 7. 安全提示

- ⚠️ `disable()` 会使机械臂在重力作用下坠落，务必确认安全。
- `request_stop()` 为高优先级急停，应绑定到独立物理急停通道。
- 遥操态下不会执行手动控制指令。
- `recover_joint_limits` 仅在 server 以 `allow_limit_recovery=True` 启动时可用。

## 8. 与 litearm-python 对比

| 方面 | litearm-python | litearm-cpp |
|---|---|---|
| 值类型 | 原生 `dict`/`list` | `LiteArmValue`（tagged union） |
| 依赖 | 自动安装 | protobuf（gtest 仅测试） |
| 适用 | 快速原型 / 脚本 | 低延迟嵌入式 / 实时系统 |

三个版本接口方法一一对应，代码可跨语言迁移。

## License

Proprietary
