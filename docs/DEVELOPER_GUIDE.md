# litearm-cpp Developer Guide & API Reference

`litearm-cpp` is the C++ client SDK for the LiteArm robotic arm (C++17).
Connect to the arm control service over the network — ideal for low-latency,
embedded scenarios. **A pure client with no kinematics/dynamics dependencies.**

```text
Your program ──→ Arm control service ──→ Arm / CAN
```

---

## Table of Contents

- [1. Requirements & Build](#1-requirements--build)
  - [1.1 Install protobuf](#11-install-protobuf)
  - [1.2 Build](#12-build)
- [2. Quick Start](#2-quick-start)
- [3. Connection Management](#3-connection-management)
- [4. Value Type: LiteArmValue](#4-value-type-litearmvalue)
- [5. API Reference](#5-api-reference)
  - [5.1 Computation (no motors driven)](#51-computation-no-motors-driven)
  - [5.2 Motion Control](#52-motion-control)
  - [5.3 State Reading](#53-state-reading)
  - [5.4 Emergency Stop / Enable](#54-emergency-stop--enable)
  - [5.5 DIRECT Mode — Per-frame MIT Direct Control](#55-direct-mode--per-frame-mit-direct-control)
  - [5.6 Parameters](#56-parameters)
  - [5.7 Peripheral Devices](#57-peripheral-devices)
  - [5.8 System / Settings / Trajectories / Devices / Teleop](#58-system--settings--trajectories--devices--teleop-extended-interfaces)
- [6. Exceptions](#6-exceptions)
- [7. Safety Notes](#7-safety-notes)
- [8. Comparison with litearm-python](#8-comparison-with-litearm-python)

## 1. Requirements & Build

| Item | Requirement |
|---|---|
| Compiler | C++17 (GCC 8+ / Clang 7+) |
| CMake | 3.16+ |
| Protobuf | 3.19+ (headers, library, `protoc`) |
| Google Test | Tests only (downloaded automatically) |

### 1.1 Install protobuf

Check your version first:

```bash
protoc --version    # needs "libprotoc 3.19" or newer
```

If it is missing or too old, install it:

```bash
# Debian / Ubuntu (24.04+ ships protobuf 3.21)
sudo apt update && sudo apt install -y protobuf-compiler libprotobuf-dev

# or conda (any OS)
conda install -c conda-forge protobuf=3.19.6

# or build from source (any OS)
cd /tmp
curl -LO https://github.com/protocolbuffers/protobuf/releases/download/v3.19.6/protobuf-cpp-3.19.6.tar.gz
tar xzf protobuf-cpp-3.19.6.tar.gz && cd protobuf-3.19.6
./configure && make -j$(nproc) && sudo make install
sudo ldconfig
```

### 1.2 Build

```bash
mkdir build && cd build
cmake ..                      # system-installed protobuf is found automatically
cmake --build . -j$(nproc)
ctest --output-on-failure     # run tests
```

Protobuf in a custom prefix (e.g. an activated conda env):

```bash
cmake .. -DPROTOBUF_ROOT=$CONDA_PREFIX
```

## 2. Quick Start

```cpp
#include <litearm/arm.hpp>

int main() {
    auto arm = litearm::Arm("tcp/192.168.1.100:7447", "armA");

    auto state = arm.get_state();                 // std::optional<RobotState>
    if (state) {
        auto& q = state->q;                        // joint positions
    }

    arm.movej({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*speed=*/0.5);
    auto [pos, rot] = arm.fk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    arm.request_stop();                            // high-priority emergency stop
    arm.close();
    return 0;
}
```

## 3. Connection Management

```cpp
litearm::Arm arm("tcp/127.0.0.1:7447", "armA",
                 /*transport=*/nullptr);   // optional pre-configured connection (advanced)
arm.close();                               // close the connection
```

- `Arm` is non-copyable but movable; `arm_id()` returns the arm identifier.
- `get_state(refresh=false)` synchronously reads the state cache; returns
  `std::nullopt` before the first update.

## 4. Value Type: LiteArmValue

Interface parameters and return values use the dynamic value type `LiteArmValue`
(a tagged union). Construction and access:

```cpp
using litearm::LiteArmValue;

LiteArmValue v = LiteArmValue::from_vec({0.1, 0.2});            // vector<double> → List
LiteArmValue m = LiteArmValue::from_mat({{1,0,0},{0,1,0},{0,0,1}});  // nested → List<List>

double x  = v.as_list()[0].as_double();        // access
auto vec  = v.to_vec();                        // extract vector<double>
auto mat  = m.to_mat();                        // extract vector<vector<double>>
auto cfg  = LiteArmValue::make_map({{"type", LiteArmValue("gripper")}});
```

`Kind` enum: `Null / Bool / Int / Double / String / Bytes / List / Map`. Convenience
aliases `Vec7`, `Mat3x3`.

## 5. API Reference

> Motion methods return `bool`; pure-computation methods return data; other
> interfaces return `LiteArmValue`.

### 5.1 Computation (no motors driven)

| Method | Description |
|---|---|
| `fk(q)` | Forward kinematics → `(position, rotation matrix)` |
| `ik(pos_d, R_d, q_seed=nullopt)` | Inverse kinematics → `(q, success)` |
| `plan_movel(q_start, pose_goal)` | Cartesian line path planning → joint path |
| `plan_movec(q_start, pose_via, pose_goal)` | Circular-arc path planning (via a waypoint) |
| `plan_movep(q_start, poses_goal)` | Multi-waypoint path planning |

### 5.2 Motion Control

| Method | Description |
|---|---|
| `movej(q_target, speed=1.0, settle_s=1.0, max_cycles=nullopt, allow_start_collision_recovery=false)` | Joint-space point-to-point |
| `home(speed=0.3, settle_s=0.5, max_cycles=nullopt)` | Home all joints to zero — bypasses joint-limit and self-collision path checks |
| `recover_joint_limits(speed=0.05, settle_s=0.5, max_cycles=nullopt, inset_rad=0.0)` | Slowly return out-of-limit joints to the safe boundary (requires server `allow_limit_recovery=True`) |
| `movel(pose_goal, speed=1.0, settle_s=0.8, max_cycles=nullopt)` | Cartesian line move |
| `movec(pose_via, pose_goal, speed=1.0, settle_s=0.8, max_cycles=nullopt)` | Circular arc move |
| `movep(poses_goal, speed=1.0, settle_s=0.8, max_cycles=nullopt)` | Multi-waypoint move with corner blending |
| `replay_joint_path(q_path, speed=1.0, settle_s=0.5, goto_start=true, goto_speed=0.3, max_cycles=nullopt)` | Replay a joint sequence |
| `replay_trajectory(traj, speed=1.0, goto_start=true, goto_speed=0.3, max_cycles=nullopt, check_singularity=true)` | Replay a `JointTrajectory` (also a `LiteArmValue` overload) |
| `replay_timed_trajectory(traj_q, traj_t, speed=1.0, goto_start=true, goto_speed=0.3, simplify_tolerance_rad=0.01, max_cycles=nullopt)` | Replay on the original time axis (auto-stretch for safety) |
| `play_trajectory(trajectory, speed=1.0, goto_start=true, goto_speed=0.3, verify_robot=true, simplify_tolerance_rad=0.01, max_cycles=nullopt)` | Replay a saved trajectory (`JointTrajectory` or server-side path string, dual overload) |
| `record_trajectory(output="trajectories", duration_s=nullopt, sample_rate_hz=100.0, filter_alpha=0.15, name=nullopt)` | Record by drag → `JointTrajectory` |
| `hold(kp_scale=3.0, max_cycles=nullopt)` | Hold with higher stiffness |
| `zero_gravity(max_cycles=nullopt, duration_s=nullopt, measured_overspeed_factor=nullopt, vel_max=nullopt)` | Zero-gravity (free-drag) mode |
| `joint_impedance(q_des, K, B, tau_max=nullopt, engage_sec=0.3, max_cycles=nullopt)` | Joint-space impedance control |
| `cartesian_impedance(q_des, K_cart, B_cart, v_des=nullopt, tau_max=nullopt, engage_sec=0.3, max_cycles=nullopt, sigma_min_thresh=nullopt, max_ori_err=nullopt, measured_overspeed_factor=nullopt, vel_max=nullopt)` | Cartesian impedance control |
| `joint_follow(K=nullopt, B=nullopt, speed_limit=nullopt, accel_limit=nullopt, engage_sec=0.3, max_cycles=nullopt, duration_s=nullopt)` | Follow an external target |

### 5.3 State Reading

| Method | Description |
|---|---|
| `get_state(refresh=false)` | Latest cached state (`std::optional<RobotState>`, sync) |
| `get_tcp_pose()` | Current TCP pose → `(position, rotation matrix)` |

`RobotState` fields: `q / dq / tau / faults / errs / temps / state / feedback / watchdog / robot_serial / config_checksum_sha256`.

### 5.4 Emergency Stop / Enable

| Method | Description |
|---|---|
| `request_stop()` | High-priority emergency stop (independent channel) |
| `clear_stop()` | Clear the stop condition and return to ready |
| `enable()` | Enable all motors and lock the current pose |
| `disable()` | ⚠️ Disables all motors (the arm drops under gravity!), CAN stays connected |

### 5.5 DIRECT Mode — Per-frame MIT Direct Control

> DIRECT mode is LiteArm's per-frame MIT direct control channel. Send five-parameter
> (kp/kd/q_ref/dq_ref/tau_ff) commands at a typical 250Hz to control joint motors in real-time,
> with 4 never-disableable core safety guardrails built in.

**Comparison with ordinary motion control:**

| Feature | Ordinary motion control (`movej` etc.) | DIRECT mode (`send_mit`) |
| --- | --- | --- |
| Control method | Target position + velocity, auto-planned | Per-frame 5-parameter MIT command |
| Frame rate | One call, auto-execution | User loop control (typical 250Hz) |
| Blocking | Blocking, waits for completion | Non-blocking, returns immediately |

**Entry and exit:**

- **Entry**: Automatically enters DIRECT mode on the first `send_mit` call
- **Exit**: `request_stop()` / watchdog timeout / motor fault

#### send_mit — Send MIT Control Frame

**Description:** Async publish a five-parameter MIT control frame to the arm command
channel. Non-blocking, returns immediately. First call auto-enters DIRECT mode.

**Function Definition:**

```cpp
void send_mit(
    const std::vector<double>& kp,      // length 7, position stiffness
    const std::vector<double>& kd,      // length 7, velocity damping
    const std::vector<double>& q_ref,   // length 7, target joint angles (rad)
    const std::vector<double>& dq_ref,  // length 7, target angular velocity (rad/s)
    const std::vector<double>& tau_ff   // length 7, feedforward torque (N·m)
);
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `kp` | `const std::vector<double>&` | Position stiffness, length 7, range `[0, 500]`. Typical: 15–200 |
| `kd` | `const std::vector<double>&` | Velocity damping, length 7, range `[0, 5]`. Typical: 0.5–3.0 |
| `q_ref` | `const std::vector<double>&` | Target joint angles (rad), length 7. Inter-frame jumps are slew-limited |
| `dq_ref` | `const std::vector<double>&` | Target angular velocity (rad/s), length 7. Clamped to `±DQ_MAX` |
| `tau_ff` | `const std::vector<double>&` | Feedforward torque (N·m), length 7. Clamped to `±min(guards_tau_max, TAU_MAX)` |

**Return Value:** `void` — async send, no acknowledgment.

**Usage Example:**

```cpp
// Single frame (auto-enters DIRECT mode)
arm.send_mit(
    {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0},
    {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
);
```

#### set_guards — Configure Global Guardrails

**Description:** One-time global guardrail configuration (RPC). All params are
`std::optional`, `std::nullopt` = no change. **Globally persistent**: not reset on DIRECT exit.

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
| `slew_limit` | `std::optional<double>` | Global slew rate limit (rad/s). `std::nullopt` = no change |
| `tau_max` | `std::optional<double>` | Global torque limit (N·m). `std::nullopt` = no change |
| `watchdog_timeout` | `std::optional<double>` | Watchdog timeout (s), range `[0.05, 2.0]` |
| `position_bounds` | `std::optional<bool>` | Enable position soft-limits. Default `false` |
| `velocity_bounds` | `std::optional<bool>` | Enable velocity soft-limits. Default `false` |
| `jerk_limit` | `std::optional<bool>` | Enable jerk limiting. Default `false` |

**Return Value:** `LiteArmValue` — RPC reply.

**Usage Example:**

```cpp
// Combined configuration
arm.set_guards(1.0, 10.0, 0.10, true);
// slew_limit=1.0, tau_max=10.0, watchdog_timeout=0.10, position_bounds=true

// Single parameter change
arm.set_guards(std::nullopt, std::nullopt, 0.05);  // tighten watchdog to 50ms
```

#### get_guards — Read Current Guardrail Configuration

**Description:** Read the currently active guardrail configuration (RPC, synchronous).

**Function Definition:**

```cpp
LiteArmValue get_guards();
```

**Return Value:** `LiteArmValue` — map with 6 fields.

**Usage Example:**

```cpp
auto guards = arm.get_guards();
auto kw = guards.as_map();
std::cout << "slew_limit = " << kw["slew_limit"].as_double() << " rad/s\n";
```

#### Full Control Loop Example

```cpp
/** DIRECT mode 250Hz control loop — sine wave on joint 1. */
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
    std::cout << "DIRECT mode exited" << std::endl;
    return 0;
}
```

#### Safety Guardrails

| Guardrail | Description | Disableable? |
| --- | --- | --- |
| **Guard 1: Protocol param clamp** | kp≤500, kd≤5, dq_ref≤DQ_MAX, tau_ff≤TAU_MAX | Never |
| **Guard 2: Command slew limit** | Inter-frame q_ref jump ≤ slew_limit × dt, dt capped at 0.10s | Never; `slew_limit` can be tightened |
| **Guard 3: Watchdog fail-soft** | Command interruption → auto-hold (low-stiffness PD) | Never; `watchdog_timeout` adjustable |
| **Guard 4: Single ownership** | Rejects motion commands and teleop while DIRECT active | Never |
| **Position bounds (optional)** | q_ref clamped to joint soft-limits per frame | Off by default; `position_bounds: true` |
| **Velocity bounds (optional)** | dq_ref clamped to `±DQ_MAX` per frame | Off by default; `velocity_bounds: true` |
| **Jerk limit (optional)** | dq_ref change rate limited per frame | Off by default; `jerk_limit: true` |

> **Safety bottom line: The arm must never fly away.**
> Command slew limit + single ownership + watchdog fail-soft + firmware fallback.

### 5.6 Parameters

| Method | Description |
|---|---|
| `set_gains(kp=nullopt, kd=nullopt)` / `get_gains()` | Get/set PD gains |
| `clear_faults()` | Clear motor faults |
| `set_payload(mass, com={0,0,0})` / `get_payload()` | End-effector payload (mass + center of mass) |
| `set_installation(base_rpy=nullopt, gravity=nullopt)` / `get_installation()` | Mounting orientation (base RPY or gravity vector) |

### 5.7 Peripheral Devices

```cpp
auto hand = arm.device("hand_0");          // RemoteDevice
hand.open(); hand.close();                 // open / close
hand.set_force(0.5);                       // grip force
hand.set_gesture("pinch"); hand.list_gestures();
hand.finger_move({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
hand.set_speed({...}); hand.set_torque({...});
hand.get_state();

auto gripper = arm.device("gripper_0");
gripper.set_width(0.5); double w = gripper.get_width();

auto teach = arm.device("teach_0");
teach.get_joints(); teach.get_buttons();

// Common: get_status / get_info / connect / disconnect / clear_faults
// Device manager: arm.devices()["hand_0"] equals arm.device("hand_0")
```

`RemoteDevice::call(method, kwargs)` invokes any device method, automatically
adding the `device.{id}.` prefix.

### 5.8 System / Settings / Trajectories / Devices / Teleop (extended interfaces)

All return `LiteArmValue`:

| Group | Methods |
|---|---|
| System | `get_system_stats()`、`get_logs(page=1, size=50, search="")`、`restart_service()`、`reconnect()` |
| Settings | `get_joint_limits/set_joint_limits(limits)`、`get_zero_offsets/set_zero_offsets(offsets)`、`get_end_effector/set_end_effector(config)`、`get_cartesian_limits/set_cartesian_limits(limits)`、`get_collision_config/set_collision_config(config)` |
| Trajectories | `start_recording/stop_recording/discard_recording/get_recording_state/get_playback_state/list_trajectories/save_trajectory(id,name,points,duration=nullopt)/delete_trajectory(id)` |
| Devices | `list_device_types()`、`connect_device(category, subtype, device_id="end_0", can_iface="", config={})`、`disconnect_device(device_id="end_0")`、`get_active_device(device_id="end_0")` |
| Teleop | `enter_teleop(mode, params={})`、`exit_teleop()`、`get_teleop_status()` |

> In teleop mode the service rejects all manual-control commands; only read-only,
> emergency-stop, and `exit_teleop` calls are allowed.

## 6. Exceptions

All exceptions inherit from `LiteArmError` (a `std::runtime_error` subclass).
Server exceptions are re-thrown with the same type.

```cpp
#include <litearm/exceptions.hpp>

try {
    arm.movej(...);
} catch (const litearm::SafetyViolationError& e) {   // timeout/follow/fault/watchdog
    // e.details is an unordered_map<string,string>
} catch (const litearm::LiteArmError& e) {           // fallback
    std::cerr << e.what();
}
```

Common types: `NotConnectedError`, `ConfigurationError`, `InvalidCommandError`,
`CartesianPlanError`, `MotionTimeoutError`, `MotorFaultError`, `ArmFault`,
`WatchdogError`, `MotionCancelled`.

## 7. Safety Notes

- ⚠️ `disable()` drops the arm under gravity — make sure the area is clear.
- `request_stop()` is a high-priority emergency stop; bind it to an independent
  physical e-stop channel.
- Manual-control commands are rejected during teleop.
- `recover_joint_limits` is only available when the server runs with
  `allow_limit_recovery=True`.

## 8. Comparison with litearm-python

| Aspect | litearm-python | litearm-cpp |
|---|---|---|
| Value type | Native `dict`/`list` | `LiteArmValue` (tagged union) |
| Dependencies | Auto-installed | protobuf (gtest tests only) |
| Use case | Rapid prototyping / scripting | Low-latency embedded / real-time systems |

The three SDKs expose the same interface, method-for-method — code can be
migrated across languages.

## License

Proprietary
