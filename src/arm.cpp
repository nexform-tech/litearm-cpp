#include "litearm/arm.hpp"
#include "litearm/codec.hpp"
#include "litearm/exceptions.hpp"
#include "litearm/protocol.hpp"

#include <chrono>

namespace litearm {

// ── Constructor / Destructor ────────────────────────────────────────────────

Arm::Arm(const std::string& /*endpoint*/,
         const std::string& arm_id,
         std::shared_ptr<Transport> transport)
    : arm_id_(arm_id)
    , rpc_topic_(rpc_topic(arm_id))
    , estop_topic_(estop_topic(arm_id))
    , command_topic_(command_topic(arm_id))
    , client_id_("sdk-cpp-" + std::to_string(std::hash<std::string>{}(arm_id) & 0xFFFF))
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
                std::optional<int> max_cycles,
                bool allow_start_collision_recovery)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["q_target"] = LiteArmValue::from_vec(q_target);
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["settle_s"] = LiteArmValue(settle_s);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);
    kwargs["allow_start_collision_recovery"] = LiteArmValue(allow_start_collision_recovery);

    return rpc("movej", std::move(kwargs)).as_bool();
}

bool Arm::recover_joint_limits(double speed, double settle_s,
                               std::optional<int> max_cycles,
                               double inset_rad)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["settle_s"] = LiteArmValue(settle_s);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);
    kwargs["inset_rad"] = LiteArmValue(inset_rad);

    return rpc("recover_joint_limits", std::move(kwargs)).as_bool();
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
                            std::optional<int> max_cycles,
                            bool check_singularity)
{
    return replay_trajectory(traj.to_value(), speed, goto_start,
                             goto_speed, max_cycles, check_singularity);
}

bool Arm::replay_trajectory(const LiteArmValue& traj_q,
                            double speed, bool goto_start,
                            double goto_speed,
                            std::optional<int> max_cycles,
                            bool check_singularity)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["traj_q"] = traj_q;
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["goto_start"] = LiteArmValue(goto_start);
    kwargs["goto_speed"] = LiteArmValue(goto_speed);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);
    kwargs["check_singularity"] = LiteArmValue(check_singularity);

    return rpc("replay_trajectory", std::move(kwargs)).as_bool();
}

bool Arm::replay_timed_trajectory(
    const std::vector<std::vector<double>>& traj_q,
    const std::vector<double>& traj_t,
    double speed, bool goto_start, double goto_speed,
    double simplify_tolerance_rad, std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["traj_q"] = LiteArmValue::from_mat(traj_q);
    kwargs["traj_t"] = LiteArmValue::from_vec(traj_t);
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["goto_start"] = LiteArmValue(goto_start);
    kwargs["goto_speed"] = LiteArmValue(goto_speed);
    kwargs["simplify_tolerance_rad"] = LiteArmValue(simplify_tolerance_rad);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("replay_timed_trajectory", std::move(kwargs)).as_bool();
}

bool Arm::play_trajectory(const JointTrajectory& trajectory,
                          double speed, bool goto_start,
                          double goto_speed, bool verify_robot,
                          double simplify_tolerance_rad,
                          std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["trajectory"] = trajectory.to_value();
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["goto_start"] = LiteArmValue(goto_start);
    kwargs["goto_speed"] = LiteArmValue(goto_speed);
    kwargs["verify_robot"] = LiteArmValue(verify_robot);
    kwargs["simplify_tolerance_rad"] = LiteArmValue(simplify_tolerance_rad);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("play_trajectory", std::move(kwargs)).as_bool();
}

bool Arm::play_trajectory(const std::string& trajectory,
                          double speed, bool goto_start,
                          double goto_speed, bool verify_robot,
                          double simplify_tolerance_rad,
                          std::optional<int> max_cycles)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["trajectory"] = LiteArmValue(trajectory);
    kwargs["speed"] = LiteArmValue(speed);
    kwargs["goto_start"] = LiteArmValue(goto_start);
    kwargs["goto_speed"] = LiteArmValue(goto_speed);
    kwargs["verify_robot"] = LiteArmValue(verify_robot);
    kwargs["simplify_tolerance_rad"] = LiteArmValue(simplify_tolerance_rad);
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);

    return rpc("play_trajectory", std::move(kwargs)).as_bool();
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
                       std::optional<double> duration_s,
                       std::optional<double> measured_overspeed_factor,
                       std::optional<std::vector<double>> vel_max)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["max_cycles"] = max_cycles
        ? LiteArmValue(static_cast<int64_t>(*max_cycles))
        : LiteArmValue(nullptr);
    kwargs["duration_s"] = duration_s
        ? LiteArmValue(*duration_s)
        : LiteArmValue(nullptr);
    kwargs["measured_overspeed_factor"] = measured_overspeed_factor
        ? LiteArmValue(*measured_overspeed_factor)
        : LiteArmValue(nullptr);
    kwargs["vel_max"] = vel_max
        ? LiteArmValue::from_vec(*vel_max)
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
                              std::optional<int> max_cycles,
                              std::optional<double> sigma_min_thresh,
                              std::optional<double> max_ori_err,
                              std::optional<double> measured_overspeed_factor,
                              std::optional<std::vector<double>> vel_max)
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
    kwargs["sigma_min_thresh"] = sigma_min_thresh
        ? LiteArmValue(*sigma_min_thresh)
        : LiteArmValue(nullptr);
    kwargs["max_ori_err"] = max_ori_err
        ? LiteArmValue(*max_ori_err)
        : LiteArmValue(nullptr);
    kwargs["measured_overspeed_factor"] = measured_overspeed_factor
        ? LiteArmValue(*measured_overspeed_factor)
        : LiteArmValue(nullptr);
    kwargs["vel_max"] = vel_max
        ? LiteArmValue::from_vec(*vel_max)
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

namespace {
std::string json_list(const std::vector<double>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += ",";
        s += std::to_string(v[i]);
    }
    s += "]";
    return s;
}
}  // namespace

void Arm::send_mit(const std::vector<double>& kp, const std::vector<double>& kd,
                   const std::vector<double>& q_ref, const std::vector<double>& dq_ref,
                   const std::vector<double>& tau_ff) {
    std::string frame = R"({"type":"mit","client_id":")" + client_id_ +
                        R"(","seq":)" + std::to_string(seq_++) +
                        R"(,"ts":)" + std::to_string(
                            std::chrono::duration<double>(
                                std::chrono::system_clock::now().time_since_epoch()).count()) +
                        R"(,"kp":)" + json_list(kp) +
                        R"(,"kd":)" + json_list(kd) +
                        R"(,"q_ref":)" + json_list(q_ref) +
                        R"(,"dq_ref":)" + json_list(dq_ref) +
                        R"(,"tau_ff":)" + json_list(tau_ff) + "}";
    tp_->pub(command_topic_, frame);
}

LiteArmValue Arm::set_guards(std::optional<double> slew_limit,
                             std::optional<double> tau_max,
                             std::optional<double> watchdog_timeout,
                             std::optional<bool> position_bounds,
                             std::optional<bool> velocity_bounds,
                             std::optional<bool> jerk_limit) {
    std::map<std::string, LiteArmValue> kw;
    if (slew_limit) kw["slew_limit"] = *slew_limit;
    if (tau_max) kw["tau_max"] = *tau_max;
    if (watchdog_timeout) kw["watchdog_timeout"] = *watchdog_timeout;
    if (position_bounds) kw["position_bounds"] = *position_bounds;
    if (velocity_bounds) kw["velocity_bounds"] = *velocity_bounds;
    if (jerk_limit) kw["jerk_limit"] = *jerk_limit;
    return rpc("set_guards", kw);
}

LiteArmValue Arm::get_guards() {
    return rpc("get_guards", {});
}

void Arm::enable() {
    rpc("enable");
}

void Arm::disable() {
    rpc("disable");
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

// ── Peripheral devices ───────────────────────────────────────────────────────

RemoteDevice Arm::device(const std::string& device_id) {
    if (!devices_) {
        devices_ = std::make_shared<DeviceManager>(
            [this](const std::string& method,
                   std::map<std::string, LiteArmValue> kwargs) {
                return rpc(method, std::move(kwargs));
            });
    }
    return devices_->get(device_id);
}

DeviceManager& Arm::devices() {
    if (!devices_) {
        devices_ = std::make_shared<DeviceManager>(
            [this](const std::string& method,
                   std::map<std::string, LiteArmValue> kwargs) {
                return rpc(method, std::move(kwargs));
            });
    }
    return *devices_;
}

// ── Server extension RPCs ────────────────────────────────────────────────────

LiteArmValue Arm::get_system_stats() {
    return rpc("get_system_stats");
}

LiteArmValue Arm::get_logs(int page, int size, const std::string& search) {
    return rpc("get_logs", {
        {"page", LiteArmValue(static_cast<int64_t>(page))},
        {"size", LiteArmValue(static_cast<int64_t>(size))},
        {"search", LiteArmValue(search)},
    });
}

LiteArmValue Arm::restart_service() {
    return rpc("restart_service");
}

LiteArmValue Arm::get_joint_limits() {
    return rpc("get_joint_limits");
}

LiteArmValue Arm::set_joint_limits(const LiteArmValue& limits) {
    return rpc("set_joint_limits", {{"limits", limits}});
}

LiteArmValue Arm::get_zero_offsets() {
    return rpc("get_zero_offsets");
}

LiteArmValue Arm::set_zero_offsets(const LiteArmValue& offsets) {
    return rpc("set_zero_offsets", {{"offsets", offsets}});
}

LiteArmValue Arm::get_end_effector() {
    return rpc("get_end_effector");
}

LiteArmValue Arm::set_end_effector(const LiteArmValue& config) {
    return rpc("set_end_effector", {{"config", config}});
}

LiteArmValue Arm::get_cartesian_limits() {
    return rpc("get_cartesian_limits");
}

LiteArmValue Arm::set_cartesian_limits(const LiteArmValue& limits) {
    return rpc("set_cartesian_limits", {{"limits", limits}});
}

LiteArmValue Arm::get_collision_config() {
    return rpc("get_collision_config");
}

LiteArmValue Arm::set_collision_config(const LiteArmValue& config) {
    return rpc("set_collision_config", {{"config", config}});
}

LiteArmValue Arm::start_recording() {
    return rpc("start_recording");
}

LiteArmValue Arm::stop_recording() {
    return rpc("stop_recording");
}

LiteArmValue Arm::discard_recording() {
    return rpc("discard_recording");
}

LiteArmValue Arm::get_recording_state() {
    return rpc("get_recording_state");
}

LiteArmValue Arm::get_playback_state() {
    return rpc("get_playback_state");
}

LiteArmValue Arm::list_trajectories() {
    return rpc("list_trajectories");
}

LiteArmValue Arm::save_trajectory(const std::string& id,
                                  const std::string& name,
                                  const std::vector<std::vector<double>>& points,
                                  std::optional<double> duration)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["id"] = LiteArmValue(id);
    kwargs["name"] = LiteArmValue(name);
    kwargs["points"] = LiteArmValue::from_mat(points);
    kwargs["duration"] = duration ? LiteArmValue(*duration) : LiteArmValue(nullptr);
    return rpc("save_trajectory", std::move(kwargs));
}

LiteArmValue Arm::delete_trajectory(const std::string& id) {
    return rpc("delete_trajectory", {{"id", LiteArmValue(id)}});
}

LiteArmValue Arm::list_device_types() {
    return rpc("list_device_types");
}

LiteArmValue Arm::connect_device(const std::string& category,
                                 const std::string& subtype,
                                 const std::string& device_id,
                                 const std::string& can_iface,
                                 const LiteArmValue& config)
{
    return rpc("connect_device", {
        {"category", LiteArmValue(category)},
        {"subtype", LiteArmValue(subtype)},
        {"device_id", LiteArmValue(device_id)},
        {"can_iface", LiteArmValue(can_iface)},
        {"config", config},
    });
}

LiteArmValue Arm::disconnect_device(const std::string& device_id) {
    return rpc("disconnect_device", {{"device_id", LiteArmValue(device_id)}});
}

LiteArmValue Arm::get_active_device(const std::string& device_id) {
    return rpc("get_active_device", {{"device_id", LiteArmValue(device_id)}});
}

LiteArmValue Arm::enter_teleop(
    const std::string& mode,
    const std::map<std::string, LiteArmValue>& params)
{
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["mode"] = LiteArmValue(mode);
    for (const auto& [key, value] : params) {
        kwargs[key] = value;
    }
    return rpc("enter_teleop", std::move(kwargs));
}

LiteArmValue Arm::exit_teleop() {
    return rpc("exit_teleop");
}

LiteArmValue Arm::get_teleop_status() {
    return rpc("get_teleop_status");
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

void Arm::close() {
    if (tp_) {
        tp_->close();
    }
}

} // namespace litearm
