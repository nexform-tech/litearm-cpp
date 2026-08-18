# litearm-cpp

C++17 client SDK for the LiteArm robotic arm. Connect to the arm control service
over the network and control the arm from any machine — ideal for low-latency,
embedded use.

Mirrors the API of [litearm-python](../litearm-python) for seamless migration
between languages.

> 📖 Full developer guide & API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md).

## Requirements

- C++17 compiler (GCC 8+, Clang 7+)
- CMake 3.16+
- Protobuf 3.19+ (headers, library, and `protoc` compiler)
- Google Test (for tests)

## Build

```bash
mkdir build && cd build
cmake .. -DPROTOBUF_ROOT=/path/to/protobuf
cmake --build . -j$(nproc)
```

### Finding Protobuf

If protobuf is not auto-detected, point CMake to it explicitly:

```bash
cmake .. -DPROTOBUF_ROOT=/usr/local
```

## Test

```bash
cd build
ctest --output-on-failure
```

## Project Structure

```text
litearm-cpp/
├── CMakeLists.txt
├── proto/
│   └── litearm.proto          # Interface definition
├── generated/
│   ├── litearm.pb.h           # Generated headers
│   └── litearm.pb.cc          # Generated source
├── include/litearm/
│   ├── arm.hpp                # Arm class (main API)
│   ├── codec.hpp              # Serialization
│   ├── device.hpp             # RemoteDevice / DeviceManager (peripherals)
│   ├── exceptions.hpp         # Exception hierarchy
│   ├── protocol.hpp           # Naming conventions
│   ├── transport.hpp          # Transport abstraction
│   └── types.hpp              # Core types (Value, RobotState, etc.)
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

## Usage

```cpp
#include <litearm/arm.hpp>

int main() {
    // Connect to the arm control service
    auto arm = litearm::Arm("tcp/192.168.1.100:7447", "armA");

    // Get current state
    auto state = arm.get_state();
    if (state) {
        // state->q contains joint positions
    }

    // Move to joint target
    arm.movej({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*speed=*/0.5);

    // Forward kinematics
    auto [pos, rot] = arm.fk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Emergency stop
    arm.request_stop();

    arm.close();
    return 0;
}
```

### Peripheral device usage

```cpp
#include <litearm/arm.hpp>

int main() {
    auto arm = litearm::Arm("tcp/192.168.1.100:7447", "armA");

    // Dexterous hand
    auto hand = arm.device("hand_0");        // or arm.devices()["hand_0"]
    hand.open();
    hand.set_gesture("pinch");
    hand.finger_move({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});

    // Gripper
    auto gripper = arm.device("gripper_0");
    gripper.set_width(0.5);
    double w = gripper.get_width();

    // Teach pendant
    auto teach = arm.device("teach_0");
    auto joints = teach.get_joints();

    arm.close();
    return 0;
}
```

Device methods route to `device.{device_id}.{method}` on the service, so any
end-effector or peripheral behind the arm control service is reachable with the
same `device(id)` handle.

## API Reference

The `litearm::Arm` class provides these method groups:

### Computation (no hardware)

- `fk(q)` — Forward kinematics
- `ik(pos_d, R_d, q_seed)` — Inverse kinematics
- `plan_movel(q_start, pose_goal)` — Plan Cartesian line
- `plan_movec(q_start, pose_via, pose_goal)` — Plan circular arc
- `plan_movep(q_start, poses_goal)` — Plan multi-waypoint path

### Motion

- `movej(q_target, speed, settle_s, max_cycles, allow_start_collision_recovery)` — Joint-space move
- `recover_joint_limits(speed, settle_s, max_cycles, inset_rad)` — Return out-of-limit joints to safe boundary
- `movel(pose_goal, speed, settle_s, max_cycles)` — Cartesian line
- `movec(pose_via, pose_goal, ...)` — Circular arc
- `movep(poses_goal, ...)` — Multi-waypoint with corner blending
- `replay_joint_path(q_path, ...)` — Replay joint path
- `replay_trajectory(traj, ..., check_singularity)` — Replay recorded trajectory
- `replay_timed_trajectory(traj_q, traj_t, ...)` — Replay on recorded time axis (safety-enforced)
- `play_trajectory(trajectory, ...)` — Load & replay a saved JointTrajectory
- `record_trajectory(...)` — Record by drag
- `hold(kp_scale, max_cycles)` — Hold position
- `zero_gravity(max_cycles, duration_s, measured_overspeed_factor, vel_max)` — Free-drag mode
- `joint_impedance(q_des, K, B, ...)` — Joint impedance control
- `cartesian_impedance(q_des, K_cart, B_cart, ...)` — Cartesian impedance
- `joint_follow(K, B, ...)` — Follow external target

### State

- `get_state(refresh)` — Read broadcast state cache
- `get_tcp_pose()` — Current TCP pose

### Control

- `request_stop()` — Emergency stop (independent channel)
- `clear_stop()` — Clear stop condition
- `enable()` — Enable all motors and hold current pose (re-enable after disable)
- `disable()` — Disable all motors (arm drops under gravity!); CAN stays connected

### Parameters

- `set_gains(kp, kd)` / `get_gains()`
- `clear_faults()`
- `set_payload(mass, com)` / `get_payload()`
- `set_installation(base_rpy, gravity)` / `get_installation()`

### Peripheral devices

```cpp
auto hand = arm.device("hand_0");      // or arm.devices()["hand_0"]
hand.open();
hand.set_gesture("pinch");
hand.finger_move({0.1, 0.2, ...});

auto gripper = arm.device("gripper_0");
gripper.set_width(0.5);
double w = gripper.get_width();

auto teach = arm.device("teach_0");
teach.get_joints();
```

`RemoteDevice` methods route to `device.{device_id}.{method}`. Common methods:
`get_status/get_info/connect/disconnect/clear_faults/open/close/set_force`.
Hand: `get_state/set_gesture/list_gestures/finger_move/set_speed/set_torque`.
Gripper: `set_width/get_width`. Teach: `get_joints/get_buttons`.

### Extended interfaces

- `get_system_stats()` / `get_logs(page, size, search)` / `restart_service()`
- Settings: `get_joint_limits` / `set_joint_limits(limits)`,
  `get_zero_offsets` / `set_zero_offsets(offsets)`,
  `get_end_effector` / `set_end_effector(config)`,
  `get_cartesian_limits` / `set_cartesian_limits(limits)`,
  `get_collision_config` / `set_collision_config(config)`
- Trajectory management: `start_recording` / `stop_recording` / `discard_recording`,
  `get_recording_state` / `get_playback_state` / `list_trajectories`,
  `save_trajectory(id, name, points, duration)` / `delete_trajectory(id)`
- Device management: `list_device_types()`,
  `connect_device(category, subtype, device_id, can_iface, config)`,
  `disconnect_device(device_id)` / `get_active_device(device_id)`
- Teleop: `enter_teleop(mode, params)` / `exit_teleop()` / `get_teleop_status()`

## Comparison with litearm-python

| Aspect        | litearm-python                | litearm-cpp                              |
| ------------- | ----------------------------- | ---------------------------------------- |
| Language      | Python 3.10+                  | C++17                                    |
| Value type    | Native `dict`/`list`          | `LiteArmValue` (tagged union)            |
| Dependencies  | Auto-installed                | protobuf, gtest (test only)              |
| Use case      | Rapid prototyping, scripting  | Low-latency embedded, real-time systems  |

Both SDKs expose the same interface and connect to the same arm control service.
The C++ client's `Arm` class mirrors the Python `Arm` class method-for-method, so
code can be migrated between languages with minimal changes.

Key differences:

- **No kinematics/dynamics on the client** — all computation runs server-side.
- **Transport is injectable**, which enables zero-dependency unit testing.
- **Value type**: Python uses native `dict`/`list`; C++ uses `LiteArmValue` (a tagged union).
- **Exception hierarchy** is identical — same type names, same inheritance.

## License

Same as litearm-python.
