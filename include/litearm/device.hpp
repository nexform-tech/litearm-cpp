#pragma once
/**
 * @file device.hpp
 * @brief RemoteDevice — generic peripheral remote interface.
 *
 * Mirrors litearm-python's RemoteDevice/DeviceManager. Methods are proxied
 * through "device.{device_id}.{method}" RPC calls to litearm-server.
 *
 * Usage:
 * @code
 *   auto arm = Arm::create("tcp/192.168.1.100:7447");
 *   auto hand = arm.device("hand_0");
 *   hand.open();
 *   hand.set_gesture("pinch");
 *
 *   auto gripper = arm.device("gripper_0");
 *   gripper.set_width(0.5);
 * @endcode
 */

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "litearm/types.hpp"

namespace litearm {

/**
 * Generic remote peripheral interface.
 *
 * Holds a device_id and an RPC function (injected by the Arm) that prefixes
 * calls with "device.{device_id}.". Mirrors pylitearm RemoteDevice.
 */
class RemoteDevice {
public:
    /// RPC function: method name → kwargs → result.
    using RpcFn = std::function<LiteArmValue(
        const std::string&, std::map<std::string, LiteArmValue>)>;

    RemoteDevice(std::string device_id, RpcFn rpc);

    /// Call an arbitrary device method (prefixes "device.{id}.").
    LiteArmValue call(const std::string& method,
                      std::map<std::string, LiteArmValue> kwargs = {});

    /// Device identifier (e.g. "hand_0", "gripper_0", "teach_0").
    const std::string& device_id() const { return device_id_; }

    // ── Common methods ──────────────────────────────────────────────────────

    /// Get device status.
    LiteArmValue get_status();
    /// Get device info.
    LiteArmValue get_info();
    /// Connect the device.
    bool connect();
    /// Disconnect the device.
    void disconnect();
    /// Clear device faults.
    bool clear_faults();

    // ── End-effector methods (gripper / dexterous hand) ─────────────────────

    /// Open the end-effector.
    bool open();
    /// Close the end-effector.
    bool close();
    /// Set grip force.
    bool set_force(double force);

    // ── Dexterous hand methods ──────────────────────────────────────────────

    /// Get device state (device.state_method, aligned with handGetState).
    LiteArmValue get_state();
    /// Set a gesture (hand only).
    bool set_gesture(const std::string& gesture);
    /// List supported gestures (hand only).
    LiteArmValue list_gestures();
    /// Per-finger motion (pose = per-finger angles).
    bool finger_move(const std::vector<double>& pose);
    /// Set per-finger speeds.
    bool set_speed(const std::vector<double>& speed);
    /// Set per-finger torques.
    bool set_torque(const std::vector<double>& torque);

    // ── Gripper methods ─────────────────────────────────────────────────────

    /// Set gripper width (gripper only).
    bool set_width(double width);
    /// Get gripper width (gripper only).
    double get_width();

    // ── Teach pendant methods ───────────────────────────────────────────────

    /// Read joint angles (teach only).
    LiteArmValue get_joints();
    /// Read button states (teach only).
    LiteArmValue get_buttons();

private:
    std::string device_id_;
    RpcFn rpc_;
};

/**
 * Device manager — lazily creates RemoteDevice handles.
 */
class DeviceManager {
public:
    using RpcFn = RemoteDevice::RpcFn;

    explicit DeviceManager(RpcFn rpc);

    /// Get (and lazily create) a device interface.
    RemoteDevice get(const std::string& device_id);

    /// Already-created device handle.
    RemoteDevice operator[](const std::string& device_id);

    /// Whether a device handle has been created.
    bool contains(const std::string& device_id) const;

    /// ids of all created handles.
    std::vector<std::string> device_ids() const;

private:
    RpcFn rpc_;
    std::map<std::string, RemoteDevice> devices_;
};

} // namespace litearm
