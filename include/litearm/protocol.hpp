#pragma once
/**
 * @file protocol.hpp
 * @brief Zenoh topic naming conventions for litearm v4 protocol.
 */

#include <string>

namespace litearm {

inline constexpr int PROTOCOL_VERSION = 1;

/// RPC request/reply topic for the given arm.
inline std::string rpc_topic(const std::string& arm_id) {
    return "litearm/v4/" + arm_id + "/rpc";
}

/// State broadcast topic for the given arm.
inline std::string state_topic(const std::string& arm_id) {
    return "litearm/v4/" + arm_id + "/state";
}

/// Command channel topic for the given arm (e.g. servo targets).
inline std::string command_topic(const std::string& arm_id) {
    return "litearm/v4/" + arm_id + "/command";
}

/// High-priority emergency stop topic for the given arm.
inline std::string estop_topic(const std::string& arm_id) {
    return "litearm/v4/" + arm_id + "/estop";
}

} // namespace litearm
