#pragma once
/**
 * @file codec.hpp
 * @brief Protobuf serialization for litearm v4 wire protocol.
 *
 * All messages use protobuf for type-safe, multi-language serialization.
 * Exception types are registered so that remote errors can be reconstructed
 * on the client side.
 */

#include <string>
#include <utility>

#include "litearm/types.hpp"

// Forward-declare protobuf generated types
namespace litearm_proto {
class Value;
class RpcRequest;
class RpcReply;
class RobotState;
}

namespace litearm {

// ── Value conversion ────────────────────────────────────────────────────────

/// Convert a LiteArmValue to protobuf Value.
litearm_proto::Value to_proto(const LiteArmValue& val);

/// Convert a protobuf Value to LiteArmValue.
LiteArmValue from_proto(const litearm_proto::Value& proto);

// ── Request encoding/decoding ───────────────────────────────────────────────

/// Encode an RPC request to protobuf bytes.
std::string encode_request(const std::string& method,
                           const std::map<std::string, LiteArmValue>& kwargs);

/// Decode an RPC request from protobuf bytes. Returns (method, kwargs).
std::pair<std::string, std::map<std::string, LiteArmValue>>
decode_request(const std::string& payload);

// ── Reply encoding/decoding ─────────────────────────────────────────────────

/// Encode a successful RPC reply to protobuf bytes.
std::string encode_reply_ok(const LiteArmValue& result);

/// Encode an error RPC reply to protobuf bytes.
std::string encode_reply_error(const std::string& error_type,
                               const std::string& error_msg,
                               const std::map<std::string, std::string>& details = {});

/// Decode an RPC reply from protobuf bytes.
/// Returns the result on success, throws the appropriate exception on error.
LiteArmValue decode_reply(const std::string& payload);

// ── State encoding/decoding ─────────────────────────────────────────────────

/// Encode a RobotState to protobuf bytes.
std::string encode_state(const RobotState& state);

/// Decode a RobotState from protobuf bytes.
RobotState decode_state(const std::string& payload);

// ── Estop encoding/decoding ─────────────────────────────────────────────────

/// Encode an emergency stop signal to protobuf bytes.
std::string encode_estop(bool trigger = true);

/// Decode an emergency stop signal from protobuf bytes.
bool decode_estop(const std::string& payload);

} // namespace litearm
