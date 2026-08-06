# litearm-cpp

C++ client SDK for the LiteArm robotic arm — remotely calls a `litearm-server` over Eclipse Zenoh via protobuf RPC.

Mirrors the API of `litearm-python` for seamless migration between languages.

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

By default, CMake looks for protobuf in `$CONDA_PREFIX` or `/home/llx/miniconda3/envs/unitree_sim_env`.
Override with:

```bash
cmake .. -DPROTOBUF_ROOT=/usr/local
```

## Test

```bash
cd build
ctest --output-on-failure
```

## Project Structure

```
litearm-cpp/
├── CMakeLists.txt
├── proto/
│   └── litearm.proto          # Wire protocol definition
├── generated/
│   ├── litearm.pb.h           # Generated protobuf headers
│   └── litearm.pb.cc          # Generated protobuf source
├── include/litearm/
│   ├── arm.hpp                # Arm class (main API)
│   ├── codec.hpp              # Protobuf serialization
│   ├── exceptions.hpp         # Exception hierarchy
│   ├── protocol.hpp           # Topic naming conventions
│   ├── transport.hpp          # Transport abstraction
│   └── types.hpp              # Core types (Value, RobotState, etc.)
├── src/
│   ├── arm.cpp
│   ├── codec.cpp
│   ├── exceptions.cpp
│   ├── transport.cpp
│   └── types.cpp
└── tests/
    ├── test_arm.cpp
    ├── test_codec.cpp
    ├── test_protocol.cpp
    └── test_transport.cpp
```

## Usage

```cpp
#include <litearm/arm.hpp>

int main() {
    // Connect to litearm-server
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

## API Reference

The `litearm::Arm` class provides these method groups:

### Computation (no hardware)
- `fk(q)` — Forward kinematics
- `ik(pos_d, R_d, q_seed)` — Inverse kinematics
- `plan_movel(q_start, pose_goal)` — Plan Cartesian line
- `plan_movec(q_start, pose_via, pose_goal)` — Plan circular arc
- `plan_movep(q_start, poses_goal)` — Plan multi-waypoint path

### Motion
- `movej(q_target, speed, settle_s, max_cycles)` — Joint-space move
- `movel(pose_goal, speed, settle_s, max_cycles)` — Cartesian line
- `movec(pose_via, pose_goal, ...)` — Circular arc
- `movep(poses_goal, ...)` — Multi-waypoint with corner blending
- `replay_joint_path(q_path, ...)` — Replay joint path
- `replay_trajectory(traj, ...)` — Replay recorded trajectory
- `record_trajectory(...)` — Record by dragging
- `hold(kp_scale, max_cycles)` — Hold position
- `zero_gravity(max_cycles, duration_s)` — Free-drag mode
- `joint_impedance(q_des, K, B, ...)` — Joint impedance control
- `cartesian_impedance(q_des, K_cart, B_cart, ...)` — Cartesian impedance
- `joint_follow(K, B, ...)` — Follow external target

### State
- `get_state(refresh)` — Read broadcast state cache
- `get_tcp_pose()` — Current TCP pose via RPC

### Control
- `request_stop()` — Emergency stop (publish, no RPC)
- `clear_stop()` — Clear stop condition

### Parameters
- `set_gains(kp, kd)` / `get_gains()`
- `clear_faults()`
- `set_payload(mass, com)` / `get_payload()`
- `set_installation(base_rpy, gravity)` / `get_installation()`

## License

Same as litearm-python.
