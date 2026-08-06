#include "litearm/arm.hpp"
#include "litearm/codec.hpp"
#include "litearm/exceptions.hpp"
#include "litearm/protocol.hpp"

namespace litearm {

// ── Constructor / Destructor ────────────────────────────────────────────────

Arm::Arm(const std::string& /*endpoint*/,
         const std::string& arm_id,
         std::shared_ptr<Transport> transport)
    : arm_id_(arm_id)
    , rpc_topic_(rpc_topic(arm_id))
    , estop_topic_(estop_topic(arm_id))
{
    if (transport) {
        tp_ = std::move(transport);
    } else {
        // Default: create an InProcTransport (zenoh not linked by default)
        // In production, users should pass a ZenohTransport
        tp_ = std::make_shared<InProcTransport>();
    }
    state_sub_ = tp_->sub(state_topic(arm_id));
}

Arm::~Arm() {
    try { close(); } catch (...) {}
}

// ── Internal RPC ────────────────────────────────────────────────────────────

LiteArmValue Arm::rpc(const std::string& method,
                      std::map<std::string, LiteArmValue> kwargs)
{
    std::string payload = encode_request(method, kwargs);
    std::string reply = tp_->query(rpc_topic_, payload);
    return decode_reply(reply);
}

// ── Pure computation API ────────────────────────────────────────────────────

std::pair<std::vector<double>, std::vector<std::vector<double>>>
Arm::fk(const std::vector<double>& q) {
    auto result = rpc("fk", {{"q", LiteArmValue::from_vec(q)}});
    // Result is [position, rotation_matrix]
    const auto& list = result.as_list();
    return {list[0].to_vec(), list[1].to_mat()};
}

std::pair<std::vector<double>, bool>
Arm::ik(const std::vector<double>& pos_d,
        const std::vector<std::vector<double>>& R_d,
        const std::optional<std::vector<double>>& q_seed)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["pos_d"] = LiteArmValue::from_vec(pos_d);
    kwargs["R_d"] = LiteArmValue::from_mat(R_d);
    if (q_seed) {
        kwargs["q_seed"] = LiteArmValue::from_vec(*q_seed);
    } else {
        kwargs["q_seed"] = LiteArmValue(nullptr);
    }

    auto result = rpc("ik", std::move(kwargs));
    const auto& list = result.as_list();
    return {list[0].to_vec(), list[1].as_bool()};
}

std::vector<std::vector<double>>
Arm::plan_movel(const std::vector<double>& q_start,
                const LiteArmValue& pose_goal)
{
    auto result = rpc("plan_movel", {
        {"q_start", LiteArmValue::from_vec(q_start)},
        {"pose_goal", pose_goal},
    });
    return result.to_mat();
}

std::vector<std::vector<double>>
Arm::plan_movec(const std::vector<double>& q_start,
                const LiteArmValue& pose_via,
                const LiteArmValue& pose_goal)
{
    auto result = rpc("plan_movec", {
        {"q_start", LiteArmValue::from_vec(q_start)},
        {"pose_via", pose_via},
        {"pose_goal", pose_goal},
    });
    return result.to_mat();
}

std::vector<std::vector<double>>
Arm::plan_movep(const std::vector<double>& q_start,
                const LiteArmValue& poses_goal)
{
    auto result = rpc("plan_movep", {
        {"q_start", LiteArmValue::from_vec(q_start)},
        {"poses_goal", poses_goal},
    });
    return result.to_mat();
}

// ── Motion execution ────────────────────────────────────────────────────────

bool Arm::movej(const std::vector<double>& q_target,
                double speed, double settle_s,
                std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["q_target"] = LiteArmValue::from_vec(q_target);
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["settle_s"] = LiteArmValue(settle_s);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("movej", std::move(kwargs)).as_bool();
}

bool Arm::movel(const LiteArmValue& pose_goal,
                double speed, double settle_s,
                std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["pose_goal"] = pose_goal;
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["settle_s"] = LiteArmValue(settle_s);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("movel", std::move(kwargs)).as_bool();
}

bool Arm::movec(const LiteArmValue& pose_via,
                const LiteArmValue& pose_goal,
                double speed, double settle_s,
                std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["pose_via"] = pose_via;
    kwargs["pose_goal"] = pose_goal;
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["settle_s"] = LiteArmValue(settle_s);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("movec", std::move(kwargs)).as_bool();
}

bool Arm::movep(const LiteArmValue& poses_goal,
                double speed, double settle_s,
                std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["poses_goal"] = poses_goal;
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["settle_s"] = LiteArmValue(settle_s);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("movep", std::move(kwargs)).as_bool();
}

bool Arm::replay_joint_path(const std::vector<std::vector<double>>& q_path,
                            double speed, double settle_s,
                            bool goto_start, double goto_speed,
                            std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["q_path"] = LiteArmValue::from_mat(q_path);
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["settle_s"] = LiteArmValue(settle_s);
    kwargs["goto_start"] = LiteArmValue(goto_start);
    kwargs["goto_speed"] = LiteArmValue(goto_speed);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("replay_joint_path", std::move(kwargs)).as_bool();
}

bool Arm::replay_trajectory(const JointTrajectory& traj,
                            double speed, bool goto_start,
                            double goto_speed,
                            std::optional<int> max_cycles)
{
    return replay_trajectory(traj.to_value(), speed, goto_start,
                             goto_speed, max_cycles);
}

bool Arm::replay_trajectory(const LiteArmValue& traj_q,
                            double speed, bool goto_start,
                            double goto_speed,
                            std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["traj_q"] = traj_q;
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["goto_start"] = LiteArmValue(goto_start);
    kwargs["goto_speed"] = LiteArmValue(goto_speed);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("replay_trajectory", std::move(kwargs)).as_bool();
}

JointTrajectory Arm::record_trajectory(
    const std::string& output,
    std::optional<double> duration_s,
    double sample_rate_hz,
    double filter_alpha,
    std::optional<std::string> name)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["output"] = LiteArmValue(output);
    kwargs["duration_s"] = duration_s
        ? LiteArmValue(*duration_s)
        : LiteArmValue(nullptr);
    kwargs["sample_rate_hz"] = LiteArmValue(sample_rate_hz);
    kwargs["filter_alpha"] = LiteArmValue(filter_alpha);
    kwargs["name"] = name
        ? LiteArmValue(*name)
        : LiteArmValue(nullptr);

    auto result = rpc("record_trajectory", std::move(kwargs));
    return JointTrajectory::from_value(result);
}

bool Arm::hold(double kp_scale, std::optional<int> max_cycles) {
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["kp_scale"] = LiteArmValue(kp_scale);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("hold", std::move(kwargs)).as_bool();
}

bool Arm::zero_gravity(std::optional<int> max_cycles,
                       std::optional<double> duration_s)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);
    kwargs["duration_s"] = duration_s
        ? LiteArmValue(*duration_s)
        : LiteArmValue(nullptr);

    return rpc("zero_gravity", std::move(kwargs)).as_bool();
}

bool Arm::joint_impedance(const std::vector<double>& q_des,
                          const LiteArmValue& K,
                          const LiteArmValue& B,
                          std::optional<LiteArmValue> tau_max,
                          double engage_sec,
                          std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["q_des"] = LiteArmValue::from_vec(q_des);
    kwargs["K"] = K;
    kwargs["B"] = B;
    kwargs["tau_max"] = tau_max ? *tau_max : LiteArmValue(nullptr);
    kwargs["engage_sec"] = LiteArmValue(engage_sec);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("joint_impedance", std::move(kwargs)).as_bool();
}

bool Arm::cartesian_impedance(const std::vector<double>& q_des,
                              const LiteArmValue& K_cart,
                              const LiteArmValue& B_cart,
                              std::optional<LiteArmValue> v_des,
                              std::optional<LiteArmValue> tau_max,
                              double engage_sec,
                              std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["q_des"] = LiteArmValue::from_vec(q_des);
    kwargs["K_cart"] = K_cart;
    kwargs["B_cart"] = B_cart;
    kwargs["v_des"] = v_des ? *v_des : LiteArmValue(nullptr);
    kwargs["tau_max"] = tau_max ? *tau_max : LiteArmValue(nullptr);
    kwargs["engage_sec"] = LiteArmValue(engage_sec);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("cartesian_impedance", std::move(kwargs)).as_bool();
}

bool Arm::joint_follow(std::optional<LiteArmValue> K,
                       std::optional<LiteArmValue> B,
                       std::optional<LiteArmValue> speed_limit,
                       std::optional<LiteArmValue> accel_limit,
                       double engage_sec,
                       std::optional<int> max_cycles,
                       std::optional<double> duration_s)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["K"] = K ? *K : LiteArmValue(nullptr);
    kwargs["B"] = B ? *B : LiteArmValue(nullptr);
    kwargs["speed_limit"] = speed_limit ? *speed_limit : LiteArmValue(nullptr);
    kwargs["accel_limit"] = accel_limit ? *accel_limit : LiteArmValue(nullptr);
    kwargs["engage_sec"] = LiteArmValue(engage_sec);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);
    kwargs["duration_s"] = duration_s
        ? LiteArmValue(*duration_s)
        : LiteArmValue(nullptr);

    return rpc("joint_follow", std::move(kwargs)).as_bool();
}

// ── State reading ───────────────────────────────────────────────────────────

std::optional<RobotState> Arm::get_state(bool /*refresh*/) {
    auto raw = state_sub_->drain_latest();
    if (raw.has_value()) {
        last_state_ = decode_state(*raw);
    }
    return last_state_;
}

std::pair<std::vector<double>, std::vector<std::vector<double>>>
Arm::get_tcp_pose() {
    auto result = rpc("get_tcp_pose");
    const auto& list = result.as_list();
    return {list[0].to_vec(), list[1].to_mat()};
}

// ── Emergency stop ──────────────────────────────────────────────────────────

void Arm::request_stop() {
    tp_->pub(estop_topic_, encode_estop());
}

void Arm::clear_stop() {
    rpc("clear_stop");
}

// ── Parameter tuning ────────────────────────────────────────────────────────

LiteArmValue Arm::set_gains(std::optional<LiteArmValue> kp,
                            std::optional<LiteArmValue> kd)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["kp"] = kp ? *kp : LiteArmValue(nullptr);
    kwargs["kd"] = kd ? *kd : LiteArmValue(nullptr);
    return rpc("set_gains", std::move(kwargs));
}

LiteArmValue Arm::get_gains() {
    return rpc("get_gains");
}

LiteArmValue Arm::clear_faults() {
    return rpc("clear_faults");
}

LiteArmValue Arm::set_payload(double mass, const std::vector<double>& com) {
    return rpc("set_payload", {
        {"mass", LiteArmValue(mass)},
        {"com", LiteArmValue::from_vec(com)},
    });
}

LiteArmValue Arm::get_payload() {
    return rpc("get_payload");
}

LiteArmValue Arm::set_installation(
    std::optional<std::vector<double>> base_rpy,
    std::optional<std::vector<double>> gravity)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["base_rpy"] = base_rpy
        ? LiteArmValue::from_vec(*base_rpy)
        : LiteArmValue(nullptr);
    kwargs["gravity"] = gravity
        ? LiteArmValue::from_vec(*gravity)
        : LiteArmValue(nullptr);
    return rpc("set_installation", std::move(kwargs));
}

LiteArmValue Arm::get_installation() {
    return rpc("get_installation");
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

void Arm::close() {
    if (tp_) {
        tp_->close();
    }
}

} // namespace litearm
