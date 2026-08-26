# litearm-cpp

LiteArm 机械臂的 C++17 客户端 SDK。通过网络连接机械臂控制服务，即可从任意机器控制机械臂——适合低延迟、嵌入式场景。

API 与 [litearm-python](../litearm-python) 对齐，便于跨语言迁移。

> 📖 完整开发指南与接口说明见 [docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)。

## 环境要求

- C++17 编译器（GCC 8+，Clang 7+）
- CMake 3.16+
- Protobuf 3.19+（headers、库、`protoc` 编译器）
- Google Test（仅测试需要；首次配置时自动下载）

## 安装 protobuf

`litearm-cpp` 需要 protobuf ≥ 3.19。先检查本机是否已装：

```bash
protoc --version    # 应显示 "libprotoc 3.19" 或更新版本
```

若未安装或版本过低，任选一种方式安装：

**Debian / Ubuntu**（24.04+ 自带 protobuf 3.21）：

```bash
sudo apt update
sudo apt install -y protobuf-compiler libprotobuf-dev
```

**Conda**（任意系统）：

```bash
conda install -c conda-forge protobuf=3.19.6
```

**源码编译**（任意系统，无需包管理器）：

```bash
cd /tmp
curl -LO https://github.com/protocolbuffers/protobuf/releases/download/v3.19.6/protobuf-cpp-3.19.6.tar.gz
tar xzf protobuf-cpp-3.19.6.tar.gz && cd protobuf-3.19.6
./configure && make -j$(nproc) && sudo make install
sudo ldconfig
```

安装后验证：

```bash
protoc --version
```

## 构建

```bash
mkdir build && cd build
cmake ..                      # 系统安装的 protobuf 会被自动找到
cmake --build . -j$(nproc)
```

若 protobuf 安装在自定义前缀（例如已激活的 conda 环境），告诉 CMake 位置：

```bash
cmake .. -DPROTOBUF_ROOT=$CONDA_PREFIX
```

## 测试

```bash
cd build
ctest --output-on-failure
```

## 项目结构

```text
litearm-cpp/
├── CMakeLists.txt
├── proto/
│   └── litearm.proto          # 接口定义
├── generated/
│   ├── litearm.pb.h           # 生成的头文件
│   └── litearm.pb.cc          # 生成的源文件
├── include/litearm/
│   ├── arm.hpp                # Arm 类（主 API）
│   ├── codec.hpp              # 序列化
│   ├── device.hpp             # RemoteDevice / DeviceManager（外设）
│   ├── exceptions.hpp         # 异常体系
│   ├── protocol.hpp           # 命名约定
│   ├── transport.hpp          # 传输抽象
│   └── types.hpp              # 核心类型（Value、RobotState 等）
├── src/
│   ├── arm.cpp
│   ├── codec.cpp
│   ├── device.cpp
│   ├── exceptions.cpp
│   ├── transport.cpp
│   └── types.cpp
└── tests/
    ├── test_arm.cpp
    ├── test_codec.cpp
    ├── test_device.cpp
    ├── test_protocol.cpp
    └── test_transport.cpp
```

## 使用

```cpp
#include <litearm/arm.hpp>

int main() {
    // 连接机械臂控制服务
    auto arm = litearm::Arm("tcp/192.168.1.100:7447", "armA");

    // 读取当前状态
    auto state = arm.get_state();
    if (state) {
        // state->q 为关节位置
    }

    // 关节空间运动
    arm.movej({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*speed=*/0.5);

    // 回零：所有关节归零，绕开限位和自碰检查
    arm.home(/*speed=*/0.3, /*settle_s=*/0.5);

    // 正运动学
    auto [pos, rot] = arm.fk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // 急停
    arm.request_stop();

    arm.close();
    return 0;
}
```

### 外设设备使用

```cpp
#include <litearm/arm.hpp>

int main() {
    auto arm = litearm::Arm("tcp/192.168.1.100:7447", "armA");

    // 灵巧手
    auto hand = arm.device("hand_0");        // 或 arm.devices()["hand_0"]
    hand.open();
    hand.set_gesture("pinch");
    hand.finger_move({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});

    // 夹爪
    auto gripper = arm.device("gripper_0");
    gripper.set_width(0.5);
    double w = gripper.get_width();

    // 示教板
    auto teach = arm.device("teach_0");
    auto joints = teach.get_joints();

    arm.close();
    return 0;
}
```

设备方法路由到 `device.{device_id}.{method}` 接口，因此机械臂控制服务背后的任意末端外设都可以用同一个 `device(id)` 句柄访问。

## API 参考

`litearm::Arm` 类提供以下方法分组：

### 计算（无硬件）

- `fk(q)` — 正运动学
- `ik(pos_d, R_d, q_seed)` — 逆运动学
- `plan_movel(q_start, pose_goal)` — 笛卡尔直线规划
- `plan_movec(q_start, pose_via, pose_goal)` — 圆弧规划
- `plan_movep(q_start, poses_goal)` — 多航点规划

### 运动

- `movej(q_target, speed, settle_s, max_cycles, allow_start_collision_recovery)` — 关节空间运动
- `home(speed, settle_s, max_cycles)` — 回零：所有关节归零，绕开限位和自碰路径检查
- `recover_joint_limits(speed, settle_s, max_cycles, inset_rad)` — 越限关节回安全边界
- `movel(pose_goal, speed, settle_s, max_cycles)` — 笛卡尔直线
- `movec(pose_via, pose_goal, ...)` — 圆弧
- `movep(poses_goal, ...)` — 多航点带拐角平滑
- `replay_joint_path(q_path, ...)` — 回放关节路径
- `replay_trajectory(traj, ..., check_singularity)` — 回放已录轨迹
- `replay_timed_trajectory(traj_q, traj_t, ...)` — 按原始时间轴回放（安全拉伸）
- `play_trajectory(trajectory, ...)` — 回放已保存轨迹
- `record_trajectory(...)` — 拖动录轨迹
- `hold(kp_scale, max_cycles)` — 持位
- `zero_gravity(max_cycles, duration_s, measured_overspeed_factor, vel_max)` — 零重力（自由拖动）
- `joint_impedance(q_des, K, B, ...)` — 关节空间阻抗控制
- `cartesian_impedance(q_des, K_cart, B_cart, ...)` — 笛卡尔阻抗控制
- `joint_follow(K, B, ...)` — 跟随外部目标

### 状态

- `get_state(refresh)` — 读取状态缓存
- `get_tcp_pose()` — 当前 TCP 位姿

### 控制

- `request_stop()` — 急停（独立通道）
- `clear_stop()` — 清除停止状态
- `enable()` — 使能全部电机并保持当前姿态
- `disable()` — ⚠️ 失能全部电机（机械臂会掉臂！），CAN 保持连接

### 参数

- `set_gains(kp, kd)` / `get_gains()`
- `clear_faults()`
- `set_payload(mass, com)` / `get_payload()`
- `set_installation(base_rpy, gravity)` / `get_installation()`

### 外设设备

```cpp
auto hand = arm.device("hand_0");      // 或 arm.devices()["hand_0"]
hand.open();
hand.set_gesture("pinch");
hand.finger_move({0.1, 0.2, ...});

auto gripper = arm.device("gripper_0");
gripper.set_width(0.5);
double w = gripper.get_width();

auto teach = arm.device("teach_0");
teach.get_joints();
```

`RemoteDevice` 方法路由到 `device.{device_id}.{method}`。通用方法：
`get_status/get_info/connect/disconnect/clear_faults/open/close/set_force`。
灵巧手：`get_state/set_gesture/list_gestures/finger_move/set_speed/set_torque`。
夹爪：`set_width/get_width`。示教板：`get_joints/get_buttons`。

### 扩展接口

- `get_system_stats()` / `get_logs(page, size, search)` / `restart_service()`
- 设置：`get_joint_limits` / `set_joint_limits(limits)`、
  `get_zero_offsets` / `set_zero_offsets(offsets)`、
  `get_end_effector` / `set_end_effector(config)`、
  `get_cartesian_limits` / `set_cartesian_limits(limits)`、
  `get_collision_config` / `set_collision_config(config)`
- 轨迹管理：`start_recording` / `stop_recording` / `discard_recording`、
  `get_recording_state` / `get_playback_state` / `list_trajectories`、
  `save_trajectory(id, name, points, duration)` / `delete_trajectory(id)`
- 设备管理：`list_device_types()`、
  `connect_device(category, subtype, device_id, can_iface, config)`、
  `disconnect_device(device_id)` / `get_active_device(device_id)`
- 遥操：`enter_teleop(mode, params)` / `exit_teleop()` / `get_teleop_status()`

## 与 litearm-python 对比

| 方面 | litearm-python | litearm-cpp |
|---|---|---|
| 语言 | Python 3.10+ | C++17 |
| 值类型 | 原生 `dict`/`list` | `LiteArmValue`（tagged union） |
| 依赖 | 自动安装 | protobuf（gtest 仅测试） |
| 适用 | 快速原型 / 脚本 | 低延迟嵌入式 / 实时系统 |

三个 SDK 接口方法一一对应，代码可跨语言迁移。

主要区别：

- **客户端无运动学/动力学** — 所有计算都在服务端进行。
- **传输层可注入**，便于无外部依赖的单元测试。
- **值类型**：Python 用原生 `dict`/`list`；C++ 用 `LiteArmValue`（tagged union）。
- **异常体系一致** — 同名同继承关系。

## License

与 litearm-python 相同。
