#include "litearm/device.hpp"

namespace litearm {

// ── RemoteDevice ─────────────────────────────────────────────────────────────

RemoteDevice::RemoteDevice(std::string device_id, RpcFn rpc)
    : device_id_(std::move(device_id))
    , rpc_(std::move(rpc))
{
}

LiteArmValue RemoteDevice::call(const std::string& method,
                                std::map<std::string, LiteArmValue> kwargs)
{
    return rpc_("device." + device_id_ + "." + method, std::move(kwargs));
}

// ── Common methods ───────────────────────────────────────────────────────────

LiteArmValue RemoteDevice::get_status() {
    return call("get_status");
}

LiteArmValue RemoteDevice::get_info() {
    return call("get_info");
}

bool RemoteDevice::connect() {
    return call("connect").as_bool();
}

void RemoteDevice::disconnect() {
    call("disconnect");
}

bool RemoteDevice::clear_faults() {
    return call("clear_faults").as_bool();
}

// ── End-effector methods ─────────────────────────────────────────────────────

bool RemoteDevice::open() {
    return call("open").as_bool();
}

bool RemoteDevice::close() {
    return call("close").as_bool();
}

bool RemoteDevice::set_force(double force) {
    return call("set_force", {{"force", LiteArmValue(force)}}).as_bool();
}

// ── Dexterous hand methods ───────────────────────────────────────────────────

LiteArmValue RemoteDevice::get_state() {
    return call("get_state");
}

bool RemoteDevice::set_gesture(const std::string& gesture) {
    return call("set_gesture", {{"gesture", LiteArmValue(gesture)}}).as_bool();
}

LiteArmValue RemoteDevice::list_gestures() {
    return call("list_gestures");
}

bool RemoteDevice::finger_move(const std::vector<double>& pose) {
    return call("finger_move", {{"pose", LiteArmValue::from_vec(pose)}}).as_bool();
}

bool RemoteDevice::set_speed(const std::vector<double>& speed) {
    return call("set_speed", {{"speed", LiteArmValue::from_vec(speed)}}).as_bool();
}

bool RemoteDevice::set_torque(const std::vector<double>& torque) {
    return call("set_torque", {{"torque", LiteArmValue::from_vec(torque)}}).as_bool();
}

// ── Gripper methods ──────────────────────────────────────────────────────────

bool RemoteDevice::set_width(double width) {
    return call("set_width", {{"width", LiteArmValue(width)}}).as_bool();
}

double RemoteDevice::get_width() {
    return call("get_width").as_double();
}

// ── Teach pendant methods ────────────────────────────────────────────────────

LiteArmValue RemoteDevice::get_joints() {
    return call("get_joints");
}

LiteArmValue RemoteDevice::get_buttons() {
    return call("get_buttons");
}

// ── DeviceManager ────────────────────────────────────────────────────────────

DeviceManager::DeviceManager(RpcFn rpc)
    : rpc_(std::move(rpc))
{
}

RemoteDevice DeviceManager::get(const std::string& device_id) {
    auto it = devices_.find(device_id);
    if (it == devices_.end()) {
        it = devices_.emplace(device_id, RemoteDevice(device_id, rpc_)).first;
    }
    return it->second;
}

RemoteDevice DeviceManager::operator[](const std::string& device_id) {
    return get(device_id);
}

bool DeviceManager::contains(const std::string& device_id) const {
    return devices_.count(device_id) > 0;
}

std::vector<std::string> DeviceManager::device_ids() const {
    std::vector<std::string> ids;
    ids.reserve(devices_.size());
    for (const auto& [id, dev] : devices_) {
        ids.push_back(id);
    }
    return ids;
}

} // namespace litearm
