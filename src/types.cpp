#include "litearm/types.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace litearm {

// ── LiteArmValue ────────────────────────────────────────────────────────────

LiteArmValue::LiteArmValue(ListType v)
    : kind_(Kind::List), list_val_(std::make_shared<ListType>(std::move(v))) {}

LiteArmValue::LiteArmValue(MapType v)
    : kind_(Kind::Map), map_val_(std::make_shared<MapType>(std::move(v))) {}

bool LiteArmValue::as_bool() const {
    if (kind_ != Kind::Bool) throw std::bad_cast();
    return bool_val_;
}

int64_t LiteArmValue::as_int() const {
    if (kind_ != Kind::Int) throw std::bad_cast();
    return int_val_;
}

double LiteArmValue::as_double() const {
    if (kind_ != Kind::Double) throw std::bad_cast();
    return double_val_;
}

const std::string& LiteArmValue::as_string() const {
    if (kind_ != Kind::String) throw std::bad_cast();
    return str_val_;
}

const LiteArmValue::BytesType& LiteArmValue::as_bytes() const {
    if (kind_ != Kind::Bytes) throw std::bad_cast();
    return bytes_val_;
}

const LiteArmValue::ListType& LiteArmValue::as_list() const {
    if (kind_ != Kind::List) throw std::bad_cast();
    return *list_val_;
}

const LiteArmValue::MapType& LiteArmValue::as_map() const {
    if (kind_ != Kind::Map) throw std::bad_cast();
    return *map_val_;
}

LiteArmValue::ListType& LiteArmValue::as_list() {
    if (kind_ != Kind::List) throw std::bad_cast();
    return *list_val_;
}

LiteArmValue::MapType& LiteArmValue::as_map() {
    if (kind_ != Kind::Map) throw std::bad_cast();
    return *map_val_;
}

LiteArmValue LiteArmValue::from_vec(const std::vector<double>& v) {
    ListType list;
    list.reserve(v.size());
    for (double d : v) {
        list.emplace_back(d);
    }
    return LiteArmValue(std::move(list));
}

LiteArmValue LiteArmValue::from_mat(const std::vector<std::vector<double>>& m) {
    ListType outer;
    outer.reserve(m.size());
    for (const auto& row : m) {
        outer.push_back(from_vec(row));
    }
    return LiteArmValue(std::move(outer));
}

std::vector<double> LiteArmValue::to_vec() const {
    std::vector<double> result;
    if (kind_ == Kind::List) {
        for (const auto& item : *list_val_) {
            if (item.is_double()) {
                result.push_back(item.as_double());
            } else if (item.is_int()) {
                result.push_back(static_cast<double>(item.as_int()));
            } else {
                throw std::bad_cast();
            }
        }
    } else {
        throw std::bad_cast();
    }
    return result;
}

std::vector<std::vector<double>> LiteArmValue::to_mat() const {
    std::vector<std::vector<double>> result;
    if (kind_ == Kind::List) {
        for (const auto& item : *list_val_) {
            result.push_back(item.to_vec());
        }
    } else {
        throw std::bad_cast();
    }
    return result;
}

LiteArmValue LiteArmValue::make_map(
    std::initializer_list<std::pair<std::string, LiteArmValue>> items)
{
    MapType m;
    for (auto& [k, v] : items) {
        m[k] = v;
    }
    return LiteArmValue(std::move(m));
}

// ── ArmState ────────────────────────────────────────────────────────────────

const char* to_string(ArmState state) {
    switch (state) {
        case ArmState::DISCONNECTED: return "disconnected";
        case ArmState::CONNECTING:   return "connecting";
        case ArmState::READY:        return "ready";
        case ArmState::MOVING:       return "moving";
        case ArmState::HOLDING:      return "holding";
        case ArmState::ZERO_GRAVITY: return "zero_gravity";
        case ArmState::IMPEDANCE:    return "impedance";
        case ArmState::FOLLOWING:    return "following";
        case ArmState::STOPPING:     return "stopping";
        case ArmState::FAULT:        return "fault";
        default:                     return "unknown";
    }
}

ArmState arm_state_from_string(const std::string& s) {
    if (s == "disconnected") return ArmState::DISCONNECTED;
    if (s == "connecting")   return ArmState::CONNECTING;
    if (s == "ready")        return ArmState::READY;
    if (s == "moving")       return ArmState::MOVING;
    if (s == "holding")      return ArmState::HOLDING;
    if (s == "zero_gravity") return ArmState::ZERO_GRAVITY;
    if (s == "impedance")    return ArmState::IMPEDANCE;
    if (s == "following")    return ArmState::FOLLOWING;
    if (s == "stopping")     return ArmState::STOPPING;
    if (s == "fault")        return ArmState::FAULT;
    return ArmState::UNKNOWN;
}

// ── JointTrajectory ─────────────────────────────────────────────────────────

double JointTrajectory::duration_s() const {
    if (frames.empty()) return 0.0;
    return frames.back().t;
}

std::vector<std::vector<double>> JointTrajectory::q() const {
    std::vector<std::vector<double>> result;
    result.reserve(frames.size());
    for (const auto& f : frames) {
        result.push_back(f.q);
    }
    return result;
}

std::vector<double> JointTrajectory::t() const {
    std::vector<double> result;
    result.reserve(frames.size());
    for (const auto& f : frames) {
        result.push_back(f.t);
    }
    return result;
}

LiteArmValue JointTrajectory::to_value() const {
    LiteArmValue::MapType m;
    m["schema"] = LiteArmValue(SCHEMA);
    m["name"] = LiteArmValue(name);
    m["created_at"] = LiteArmValue(created_at);
    m["duration_s"] = LiteArmValue(duration_s());

    if (sample_rate_hz) m["sample_rate_hz"] = LiteArmValue(*sample_rate_hz);
    else m["sample_rate_hz"] = LiteArmValue(nullptr);

    if (filter_alpha) m["filter_alpha"] = LiteArmValue(*filter_alpha);
    else m["filter_alpha"] = LiteArmValue(nullptr);

    if (robot_serial) m["robot_serial"] = LiteArmValue(*robot_serial);
    else m["robot_serial"] = LiteArmValue(nullptr);

    if (config_checksum_sha256) m["config_checksum_sha256"] = LiteArmValue(*config_checksum_sha256);
    else m["config_checksum_sha256"] = LiteArmValue(nullptr);

    LiteArmValue::ListType frame_list;
    for (const auto& f : frames) {
        LiteArmValue::MapType fm;
        fm["t"] = LiteArmValue(f.t);
        fm["q"] = LiteArmValue::from_vec(f.q);
        if (f.dq) fm["dq"] = LiteArmValue::from_vec(*f.dq);
        if (f.tau) fm["tau"] = LiteArmValue::from_vec(*f.tau);
        frame_list.push_back(LiteArmValue(std::move(fm)));
    }
    m["frames"] = LiteArmValue(std::move(frame_list));

    return LiteArmValue(std::move(m));
}

static std::vector<double> extract_vec(const LiteArmValue& v) {
    if (v.is_list()) return v.to_vec();
    throw std::runtime_error("Expected list for vector extraction");
}

JointTrajectory JointTrajectory::from_value(const LiteArmValue& v) {
    if (!v.is_map()) throw std::runtime_error("JointTrajectory: expected map");
    const auto& m = v.as_map();

    JointTrajectory traj;

    // Schema validation
    auto schema_it = m.find("schema");
    if (schema_it != m.end() && !schema_it->second.is_null()) {
        if (schema_it->second.as_string() != SCHEMA) {
            throw std::runtime_error("Unsupported trajectory schema: " +
                                     schema_it->second.as_string());
        }
    }

    // Name
    auto name_it = m.find("name");
    if (name_it != m.end() && !name_it->second.is_null()) {
        traj.name = name_it->second.as_string();
    }

    // created_at
    auto ca_it = m.find("created_at");
    if (ca_it != m.end() && !ca_it->second.is_null()) {
        traj.created_at = ca_it->second.as_string();
    }

    // sample_rate_hz
    auto sr_it = m.find("sample_rate_hz");
    if (sr_it == m.end()) sr_it = m.find("sample_rate");
    if (sr_it != m.end() && !sr_it->second.is_null()) {
        traj.sample_rate_hz = sr_it->second.as_double();
    }

    // filter_alpha
    auto fa_it = m.find("filter_alpha");
    if (fa_it != m.end() && !fa_it->second.is_null()) {
        traj.filter_alpha = fa_it->second.as_double();
    }

    // robot_serial
    auto rs_it = m.find("robot_serial");
    if (rs_it != m.end() && !rs_it->second.is_null()) {
        traj.robot_serial = rs_it->second.as_string();
    }

    // config_checksum_sha256
    auto cs_it = m.find("config_checksum_sha256");
    if (cs_it != m.end() && !cs_it->second.is_null()) {
        traj.config_checksum_sha256 = cs_it->second.as_string();
    }

    // Frames (support both "frames" and legacy "trajectory" key)
    auto frames_it = m.find("frames");
    if (frames_it == m.end()) frames_it = m.find("trajectory");
    if (frames_it == m.end() || !frames_it->second.is_list()) {
        throw std::runtime_error("Trajectory missing frames/trajectory array");
    }

    for (const auto& item : frames_it->second.as_list()) {
        if (!item.is_map()) {
            throw std::runtime_error("Trajectory frame must be a map");
        }
        const auto& fm = item.as_map();

        TrajectoryFrame frame;

        auto t_it = fm.find("t");
        if (t_it != fm.end()) frame.t = t_it->second.as_double();

        auto q_it = fm.find("q");
        if (q_it == fm.end()) q_it = fm.find("joints");
        if (q_it != fm.end()) frame.q = extract_vec(q_it->second);

        auto dq_it = fm.find("dq");
        if (dq_it != fm.end() && !dq_it->second.is_null()) {
            frame.dq = extract_vec(dq_it->second);
        }

        auto tau_it = fm.find("tau");
        if (tau_it != fm.end() && !tau_it->second.is_null()) {
            frame.tau = extract_vec(tau_it->second);
        }

        traj.frames.push_back(std::move(frame));
    }

    // Normalize timestamps: shift so first frame starts at t=0
    if (!traj.frames.empty() && traj.frames[0].t != 0.0) {
        double origin = traj.frames[0].t;
        for (auto& f : traj.frames) {
            f.t -= origin;
        }
    }

    return traj;
}

} // namespace litearm
