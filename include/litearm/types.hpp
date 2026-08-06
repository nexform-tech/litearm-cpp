#pragma once
/**
 * @file types.hpp
 * @brief Core types for the litearm C++ SDK.
 *
 * Provides LiteArmValue (dynamic value type), ArmState enum,
 * RobotState, JointTrajectory, and related types.
 */

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace litearm {

// ── Convenience aliases ─────────────────────────────────────────────────────

using Vec7 = std::vector<double>;
using Mat3x3 = std::vector<std::vector<double>>;

// ── Dynamic value type ──────────────────────────────────────────────────────

/// Recursive dynamic value type (like Python's Any or JSON value).
/// Used for RPC arguments/results that vary by method.
class LiteArmValue {
public:
    enum class Kind {
        Null, Bool, Int, Double, String, Bytes, List, Map
    };

    using ListType = std::vector<LiteArmValue>;
    using MapType = std::map<std::string, LiteArmValue>;
    using BytesType = std::vector<uint8_t>;

    // Constructors
    LiteArmValue() : kind_(Kind::Null) {}
    LiteArmValue(std::nullptr_t) : kind_(Kind::Null) {}
    LiteArmValue(bool v) : kind_(Kind::Bool), bool_val_(v) {}
    LiteArmValue(int v) : kind_(Kind::Int), int_val_(v) {}
    LiteArmValue(int64_t v) : kind_(Kind::Int), int_val_(v) {}
    LiteArmValue(double v) : kind_(Kind::Double), double_val_(v) {}
    LiteArmValue(const char* v) : kind_(Kind::String), str_val_(v) {}
    LiteArmValue(std::string v) : kind_(Kind::String), str_val_(std::move(v)) {}
    LiteArmValue(BytesType v) : kind_(Kind::Bytes), bytes_val_(std::move(v)) {}
    LiteArmValue(ListType v);
    LiteArmValue(MapType v);

    // Kind queries
    Kind kind() const { return kind_; }
    bool is_null() const { return kind_ == Kind::Null; }
    bool is_bool() const { return kind_ == Kind::Bool; }
    bool is_int() const { return kind_ == Kind::Int; }
    bool is_double() const { return kind_ == Kind::Double; }
    bool is_string() const { return kind_ == Kind::String; }
    bool is_bytes() const { return kind_ == Kind::Bytes; }
    bool is_list() const { return kind_ == Kind::List; }
    bool is_map() const { return kind_ == Kind::Map; }

    // Value accessors (throw std::bad_cast on type mismatch)
    bool as_bool() const;
    int64_t as_int() const;
    double as_double() const;
    const std::string& as_string() const;
    const BytesType& as_bytes() const;
    const ListType& as_list() const;
    const MapType& as_map() const;

    // Mutable accessors
    ListType& as_list();
    MapType& as_map();

    // Convenience conversions
    /// Convert a vector<double> to a List of Double values.
    static LiteArmValue from_vec(const std::vector<double>& v);
    /// Convert a nested vector to a List of Lists.
    static LiteArmValue from_mat(const std::vector<std::vector<double>>& m);
    /// Extract as vector<double> (each element must be numeric).
    std::vector<double> to_vec() const;
    /// Extract as nested vector<vector<double>>.
    std::vector<std::vector<double>> to_mat() const;

    // Map helpers
    /// Shorthand: build a map from initializer list.
    static LiteArmValue make_map(std::initializer_list<std::pair<std::string, LiteArmValue>> items);

private:
    Kind kind_;
    bool bool_val_ = false;
    int64_t int_val_ = 0;
    double double_val_ = 0.0;
    std::string str_val_;
    BytesType bytes_val_;
    std::shared_ptr<ListType> list_val_;
    std::shared_ptr<MapType> map_val_;
};

// ── Arm state enum ──────────────────────────────────────────────────────────

enum class ArmState {
    DISCONNECTED,
    CONNECTING,
    READY,
    MOVING,
    HOLDING,
    ZERO_GRAVITY,
    IMPEDANCE,
    FOLLOWING,
    STOPPING,
    FAULT,
    UNKNOWN,
};

/// Convert ArmState to its string representation.
const char* to_string(ArmState state);

/// Parse ArmState from string. Returns UNKNOWN for unrecognized values.
ArmState arm_state_from_string(const std::string& s);

// ── Robot state types ───────────────────────────────────────────────────────

struct JointFeedbackState {
    int joint = 0;
    int received = 0;
    double age_s = 0.0;
    bool fresh = false;
};

struct FeedbackState {
    double max_age_s = 0.0;
    std::vector<JointFeedbackState> joints;
    std::vector<int> stale_joints;
};

struct WatchdogState {
    bool enabled = false;
    double timeout_s = 0.0;
    std::string mode;
    bool tripped = false;
    double last_kick_age_s = 0.0;
};

struct FaultInfo {
    int joint = 0;
    int err_code = 0;
};

struct TemperatureInfo {
    int mos_temp = 0;
    int coil_temp = 0;
};

struct RobotState {
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<FaultInfo> faults;
    std::vector<int> errs;
    std::vector<TemperatureInfo> temps;
    std::string state;
    FeedbackState feedback;
    WatchdogState watchdog;
    std::string robot_serial;
    std::string config_checksum_sha256;
};

// ── Trajectory types ────────────────────────────────────────────────────────

struct TrajectoryFrame {
    double t = 0.0;
    std::vector<double> q;
    std::optional<std::vector<double>> dq;
    std::optional<std::vector<double>> tau;
};

/// Validated, portable recording of a seven-axis joint trajectory.
struct JointTrajectory {
    static constexpr const char* SCHEMA = "pylitearm.joint_trajectory.v1";

    std::vector<TrajectoryFrame> frames;
    std::string name = "trajectory";
    std::string created_at;
    std::optional<double> sample_rate_hz;
    std::optional<double> filter_alpha;
    std::optional<std::string> robot_serial;
    std::optional<std::string> config_checksum_sha256;
    std::optional<std::string> source_path;

    /// Total duration in seconds (last frame timestamp).
    double duration_s() const;

    /// Extract all position vectors.
    std::vector<std::vector<double>> q() const;

    /// Extract all timestamps.
    std::vector<double> t() const;

    /// Serialize to LiteArmValue (map).
    LiteArmValue to_value() const;

    /// Deserialize from LiteArmValue (map).
    static JointTrajectory from_value(const LiteArmValue& v);
};

} // namespace litearm
