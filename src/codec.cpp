#include "litearm/codec.hpp"
#include "litearm/exceptions.hpp"
#include "litearm.pb.h"

#include <stdexcept>

namespace litearm {

// ── Value conversion: LiteArmValue → protobuf ───────────────────────────────

litearm_proto::Value to_proto(const LiteArmValue& val) {
    litearm_proto::Value pv;

    switch (val.kind()) {
    case LiteArmValue::Kind::Null:
        pv.mutable_none_val();
        break;

    case LiteArmValue::Kind::Bool:
        pv.set_bool_val(val.as_bool());
        break;

    case LiteArmValue::Kind::Int:
        pv.set_int_val(val.as_int());
        break;

    case LiteArmValue::Kind::Double:
        pv.set_double_val(val.as_double());
        break;

    case LiteArmValue::Kind::String:
        pv.set_string_val(val.as_string());
        break;

    case LiteArmValue::Kind::Bytes: {
        const auto& b = val.as_bytes();
        pv.set_bytes_val(b.data(), b.size());
        break;
    }

    case LiteArmValue::Kind::List: {
        auto* lv = pv.mutable_list_val();
        for (const auto& item : val.as_list()) {
            *lv->add_values() = to_proto(item);
        }
        break;
    }

    case LiteArmValue::Kind::Map: {
        auto* mv = pv.mutable_map_val();
        for (const auto& [k, v] : val.as_map()) {
            (*mv->mutable_values())[k] = to_proto(v);
        }
        break;
    }
    }

    return pv;
}

// ── Value conversion: protobuf → LiteArmValue ──────────────────────────────

LiteArmValue from_proto(const litearm_proto::Value& pv) {
    if (pv.has_none_val()) return LiteArmValue(nullptr);
    if (pv.has_bool_val()) return LiteArmValue(pv.bool_val());
    if (pv.has_int_val()) return LiteArmValue(pv.int_val());
    if (pv.has_double_val()) return LiteArmValue(pv.double_val());
    if (pv.has_string_val()) return LiteArmValue(pv.string_val());

    if (pv.has_bytes_val()) {
        const auto& s = pv.bytes_val();
        LiteArmValue::BytesType bytes(s.begin(), s.end());
        return LiteArmValue(std::move(bytes));
    }

    if (pv.has_list_val()) {
        LiteArmValue::ListType list;
        for (const auto& item : pv.list_val().values()) {
            list.push_back(from_proto(item));
        }
        return LiteArmValue(std::move(list));
    }

    if (pv.has_map_val()) {
        LiteArmValue::MapType map;
        for (const auto& [k, v] : pv.map_val().values()) {
            map[k] = from_proto(v);
        }
        return LiteArmValue(std::move(map));
    }

    return LiteArmValue(nullptr);
}

// ── Request encoding/decoding ───────────────────────────────────────────────

std::string encode_request(const std::string& method,
                           const std::map<std::string, LiteArmValue>& kwargs)
{
    litearm_proto::RpcRequest req;
    req.set_method(method);
    for (const auto& [k, v] : kwargs) {
        (*req.mutable_kwargs())[k] = to_proto(v);
    }
    return req.SerializeAsString();
}

std::pair<std::string, std::map<std::string, LiteArmValue>>
decode_request(const std::string& payload)
{
    litearm_proto::RpcRequest req;
    if (!req.ParseFromString(payload)) {
        throw std::runtime_error("Failed to parse RpcRequest");
    }

    std::map<std::string, LiteArmValue> kwargs;
    for (const auto& [k, v] : req.kwargs()) {
        kwargs[k] = from_proto(v);
    }

    return {req.method(), std::move(kwargs)};
}

// ── Reply encoding/decoding ─────────────────────────────────────────────────

std::string encode_reply_ok(const LiteArmValue& result) {
    litearm_proto::RpcReply reply;
    reply.set_ok(true);
    *reply.mutable_result() = to_proto(result);
    return reply.SerializeAsString();
}

std::string encode_reply_error(const std::string& error_type,
                               const std::string& error_msg,
                               const std::map<std::string, std::string>& details)
{
    litearm_proto::RpcReply reply;
    reply.set_ok(false);

    auto* err = reply.mutable_error();
    err->set_type(error_type);
    err->set_message(error_msg);

    for (const auto& [k, v] : details) {
        litearm_proto::Value dv;
        dv.set_string_val(v);
        (*err->mutable_details())[k] = dv;
    }

    return reply.SerializeAsString();
}

LiteArmValue decode_reply(const std::string& payload) {
    litearm_proto::RpcReply reply;
    if (!reply.ParseFromString(payload)) {
        throw LiteArmError("Failed to parse RpcReply");
    }

    if (reply.ok()) {
        return from_proto(reply.result());
    }

    // Reconstruct exception from error info
    const auto& err = reply.error();
    std::unordered_map<std::string, std::string> details;
    for (const auto& [k, v] : err.details()) {
        if (v.has_string_val()) {
            details[k] = v.string_val();
        } else {
            // Convert non-string details values to string representation
            details[k] = "[non-string]";
        }
    }

    throw_exception(err.type(), err.message(), std::move(details));
}

// ── State encoding/decoding ─────────────────────────────────────────────────

std::string encode_state(const RobotState& state) {
    litearm_proto::RobotState proto;

    for (double v : state.q) proto.add_q(v);
    for (double v : state.dq) proto.add_dq(v);
    for (double v : state.tau) proto.add_tau(v);
    for (int v : state.errs) proto.add_errs(v);

    for (const auto& f : state.faults) {
        auto* pf = proto.add_fault();
        pf->set_joint(f.joint);
        pf->set_err_code(f.err_code);
    }

    for (const auto& t : state.temps) {
        auto* pt = proto.add_temps();
        pt->set_mos_temp(t.mos_temp);
        pt->set_coil_temp(t.coil_temp);
    }

    proto.set_state(state.state);

    // Feedback
    auto* fb = proto.mutable_feedback();
    fb->set_max_age_s(state.feedback.max_age_s);
    for (int j : state.feedback.stale_joints) {
        fb->add_stale_joints(j);
    }
    for (const auto& jfb : state.feedback.joints) {
        auto* pj = fb->add_joints();
        pj->set_joint(jfb.joint);
        pj->set_received(jfb.received);
        pj->set_age_s(jfb.age_s);
        pj->set_fresh(jfb.fresh);
    }

    // Watchdog
    auto* wd = proto.mutable_watchdog();
    wd->set_enabled(state.watchdog.enabled);
    wd->set_timeout_s(state.watchdog.timeout_s);
    wd->set_mode(state.watchdog.mode);
    wd->set_tripped(state.watchdog.tripped);
    wd->set_last_kick_age_s(state.watchdog.last_kick_age_s);

    proto.set_robot_serial(state.robot_serial);
    proto.set_config_checksum_sha256(state.config_checksum_sha256);

    return proto.SerializeAsString();
}

RobotState decode_state(const std::string& payload) {
    litearm_proto::RobotState proto;
    if (!proto.ParseFromString(payload)) {
        throw std::runtime_error("Failed to parse RobotState");
    }

    RobotState state;

    state.q.assign(proto.q().begin(), proto.q().end());
    state.dq.assign(proto.dq().begin(), proto.dq().end());
    state.tau.assign(proto.tau().begin(), proto.tau().end());
    state.errs.assign(proto.errs().begin(), proto.errs().end());

    for (const auto& pf : proto.fault()) {
        state.faults.push_back({pf.joint(), pf.err_code()});
    }

    for (const auto& pt : proto.temps()) {
        state.temps.push_back({pt.mos_temp(), pt.coil_temp()});
    }

    state.state = proto.state();

    // Feedback
    if (proto.has_feedback()) {
        state.feedback.max_age_s = proto.feedback().max_age_s();
        state.feedback.stale_joints.assign(
            proto.feedback().stale_joints().begin(),
            proto.feedback().stale_joints().end());
        for (const auto& pj : proto.feedback().joints()) {
            state.feedback.joints.push_back({
                pj.joint(), pj.received(), pj.age_s(), pj.fresh()
            });
        }
    }

    // Watchdog
    if (proto.has_watchdog()) {
        state.watchdog.enabled = proto.watchdog().enabled();
        state.watchdog.timeout_s = proto.watchdog().timeout_s();
        state.watchdog.mode = proto.watchdog().mode();
        state.watchdog.tripped = proto.watchdog().tripped();
        state.watchdog.last_kick_age_s = proto.watchdog().last_kick_age_s();
    }

    state.robot_serial = proto.robot_serial();
    state.config_checksum_sha256 = proto.config_checksum_sha256();

    return state;
}

// ── Estop encoding/decoding ─────────────────────────────────────────────────

std::string encode_estop(bool trigger) {
    litearm_proto::Estop estop;
    estop.set_trigger(trigger);
    return estop.SerializeAsString();
}

bool decode_estop(const std::string& payload) {
    litearm_proto::Estop estop;
    if (!estop.ParseFromString(payload)) {
        return false;
    }
    return estop.trigger();
}

} // namespace litearm
