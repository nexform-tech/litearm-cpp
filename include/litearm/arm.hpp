#pragma once
/**
 * @file arm.hpp
 * @brief Remote Arm client — API-compatible with pylitearm.Arm.
 *
 * Connects to litearm-server via zenoh and forwards all calls as RPC.
 * No pylitearm dependency, no Pinocchio, no Eigen.
 */

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "litearm/device.hpp"
#include "litearm/transport.hpp"
#include "litearm/types.hpp"

namespace litearm {

/**
 * LiteArm remote client. API mirrors pylitearm.Arm for seamless migration.
 *
 * Usage:
 * @code
 *   auto arm = Arm::create("tcp/192.168.1.100:7447");
 *   auto state = arm->get_state();
 *   arm->movej({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.5);
 *   arm->close();
 * @endcode
 */
class Arm {
public:
    /// Connect to a litearm-server daemon.
    /// @param endpoint  Zenoh endpoint (e.g. "tcp/127.0.0.1:7447").
    ///                  Defaults to ``LITEARM_ENDPOINT`` env var, then ``tcp/127.0.0.1:7447``.
    /// @param arm_id    Arm identifier (default "armA")
    /// @param transport Optional pre-configured Transport (for testing)
    explicit Arm(const std::string& endpoint = "",
                 const std::string& arm_id = "armA",
                 std::shared_ptr<Transport> transport = nullptr);

    ~Arm();

    // Non-copyable, movable
    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;
    Arm(Arm&&) = default;
    Arm& operator=(Arm&&) = default;

    // ── Pure computation API (no hardware needed on server) ─────────────────

    /// Forward kinematics: joint angles → (position, rotation_matrix).
    std::pair<std::vector<double>, std::vector<std::vector<double>>>
    fk(const std::vector<double>& q);

    /// Inverse kinematics: (position, rotation) → (q, success).
    std::pair<std::vector<double>, bool>
    ik(const std::vector<double>& pos_d,
       const std::vector<std::vector<double>>& R_d,
       const std::optional<std::vector<double>>& q_seed = std::nullopt);

    /// Plan a straight-line Cartesian path. Returns list of joint configs.
    std::vector<std::vector<double>>
    plan_movel(const std::vector<double>& q_start,
               const LiteArmValue& pose_goal);

    /// Plan a circular-arc Cartesian path through via-point.
    std::vector<std::vector<double>>
    plan_movec(const std::vector<double>& q_start,
               const LiteArmValue& pose_via,
               const LiteArmValue& pose_goal);

    /// Plan a multi-waypoint Cartesian path.
    std::vector<std::vector<double>>
    plan_movep(const std::vector<double>& q_start,
               const LiteArmValue& poses_goal);

    // ── Motion execution ────────────────────────────────────────────────────

    /// Move to joint target.
    bool movej(const std::vector<double>& q_target,
               double speed = 1.0,
               double settle_s = 1.0,
               std::optional<int> max_cycles = std::nullopt,
               bool allow_start_collision_recovery = false);

    /// Slowly return every out-of-limit joint to the nearest safe boundary.
    /// Requires server connected with allow_limit_recovery=True.
    bool recover_joint_limits(double speed = 0.05,
                              double settle_s = 0.5,
                              std::optional<int> max_cycles = std::nullopt,
                              double inset_rad = 0.0);

    /// Home all joints to zero position ([0, 0, 0, 0, 0, 0, 0]).
    /// Unlike movej, home() bypasses joint-limit and self-collision path
    /// safety checks. Hardware-level fault / temperature / overspeed /
    /// following-error protection remains active.
    /// Default speed is 0.3 (slow) — homing is a recovery operation.
    bool home(double speed = 0.3,
              double settle_s = 0.5,
              std::optional<int> max_cycles = std::nullopt);

    /// Move in a straight Cartesian line.
    bool movel(const LiteArmValue& pose_goal,
               double speed = 1.0,
               double settle_s = 0.8,
               std::optional<int> max_cycles = std::nullopt);

    /// Move in a circular arc through via-point.
    bool movec(const LiteArmValue& pose_via,
               const LiteArmValue& pose_goal,
               double speed = 1.0,
               double settle_s = 0.8,
               std::optional<int> max_cycles = std::nullopt);

    /// Move through a sequence of Cartesian waypoints with corner blending.
    bool movep(const LiteArmValue& poses_goal,
               double speed = 1.0,
               double settle_s = 0.8,
               std::optional<int> max_cycles = std::nullopt);

    /// Replay a sequence of joint configurations.
    bool replay_joint_path(const std::vector<std::vector<double>>& q_path,
                           double speed = 1.0,
                           double settle_s = 0.5,
                           bool goto_start = true,
                           double goto_speed = 0.3,
                           std::optional<int> max_cycles = std::nullopt);

    /// Replay a JointTrajectory or path.
    /// check_singularity: treat singular Jacobian as a hard error (server-side default).
    bool replay_trajectory(const JointTrajectory& traj,
                           double speed = 1.0,
                           bool goto_start = true,
                           double goto_speed = 0.3,
                           std::optional<int> max_cycles = std::nullopt,
                           bool check_singularity = true);

    /// Replay from raw value (dict-like).
    bool replay_trajectory(const LiteArmValue& traj_q,
                           double speed = 1.0,
                           bool goto_start = true,
                           double goto_speed = 0.3,
                           std::optional<int> max_cycles = std::nullopt,
                           bool check_singularity = true);

    /// Replay a measured trajectory on its recorded time axis.
    /// Safety-enforced: automatically stretches time to respect vel/acc/jerk limits.
    bool replay_timed_trajectory(
        const std::vector<std::vector<double>>& traj_q,
        const std::vector<double>& traj_t,
        double speed = 1.0,
        bool goto_start = true,
        double goto_speed = 0.3,
        double simplify_tolerance_rad = 0.01,
        std::optional<int> max_cycles = std::nullopt);

    /// Load and replay a saved JointTrajectory (serialized for transport).
    bool play_trajectory(const JointTrajectory& trajectory,
                         double speed = 1.0,
                         bool goto_start = true,
                         double goto_speed = 0.3,
                         bool verify_robot = true,
                         double simplify_tolerance_rad = 0.01,
                         std::optional<int> max_cycles = std::nullopt);

    /// Load and replay a saved trajectory by server-side path (e.g. "trajectories/traj_001.json").
    bool play_trajectory(const std::string& trajectory,
                         double speed = 1.0,
                         bool goto_start = true,
                         double goto_speed = 0.3,
                         bool verify_robot = true,
                         double simplify_tolerance_rad = 0.01,
                         std::optional<int> max_cycles = std::nullopt);

    /// Record a trajectory by dragging the arm (zero_gravity mode).
    JointTrajectory record_trajectory(
        const std::string& output = "trajectories",
        std::optional<double> duration_s = std::nullopt,
        double sample_rate_hz = 100.0,
        double filter_alpha = 0.15,
        std::optional<std::string> name = std::nullopt);

    /// Hold current position with increased stiffness.
    bool hold(double kp_scale = 3.0,
              std::optional<int> max_cycles = std::nullopt);

    /// Enable zero-gravity (free-drag) mode.
    /// measured_overspeed_factor / vel_max kept for server compatibility
    /// (deprecated, no effect in pylitearm).
    bool zero_gravity(std::optional<int> max_cycles = std::nullopt,
                      std::optional<double> duration_s = std::nullopt,
                      std::optional<double> measured_overspeed_factor = std::nullopt,
                      std::optional<std::vector<double>> vel_max = std::nullopt);

    /// Joint-space impedance control.
    bool joint_impedance(const std::vector<double>& q_des,
                         const LiteArmValue& K,
                         const LiteArmValue& B,
                         std::optional<LiteArmValue> tau_max = std::nullopt,
                         double engage_sec = 0.3,
                         std::optional<int> max_cycles = std::nullopt);

    /// Cartesian-space impedance control.
    bool cartesian_impedance(const std::vector<double>& q_des,
                             const LiteArmValue& K_cart,
                             const LiteArmValue& B_cart,
                             std::optional<LiteArmValue> v_des = std::nullopt,
                             std::optional<LiteArmValue> tau_max = std::nullopt,
                             double engage_sec = 0.3,
                             std::optional<int> max_cycles = std::nullopt,
                             std::optional<double> sigma_min_thresh = std::nullopt,
                             std::optional<double> max_ori_err = std::nullopt,
                             std::optional<double> measured_overspeed_factor = std::nullopt,
                             std::optional<std::vector<double>> vel_max = std::nullopt);

    /// Follow an external target provider.
    bool joint_follow(std::optional<LiteArmValue> K = std::nullopt,
                      std::optional<LiteArmValue> B = std::nullopt,
                      std::optional<LiteArmValue> speed_limit = std::nullopt,
                      std::optional<LiteArmValue> accel_limit = std::nullopt,
                      double engage_sec = 0.3,
                      std::optional<int> max_cycles = std::nullopt,
                      std::optional<double> duration_s = std::nullopt);

    // ── State reading ───────────────────────────────────────────────────────

    /// Get latest robot state from broadcast cache (no RPC call).
    /// Returns nullptr if no state received yet.
    std::optional<RobotState> get_state(bool refresh = false);

    /// Get current TCP pose as (position, rotation_matrix).
    std::pair<std::vector<double>, std::vector<std::vector<double>>>
    get_tcp_pose();

    // ── Emergency stop ──────────────────────────────────────────────────────

    /// Send high-priority emergency stop signal.
    void request_stop();

    /// Clear the stop condition and return to ready state.
    void clear_stop();

    // ── Parameter tuning ────────────────────────────────────────────────────

    /// Set PD controller gains.
    LiteArmValue set_gains(std::optional<LiteArmValue> kp = std::nullopt,
                           std::optional<LiteArmValue> kd = std::nullopt);

    /// Get current PD controller gains.
    LiteArmValue get_gains();

    /// Clear motor faults. Returns list of cleared (motor_id, fault_code).
    LiteArmValue clear_faults();

    /// Direct joint motor control via pure MIT five-parameter per-frame publish.
    /// Async pub to the command topic — no RPC round-trip.
    void send_mit(const std::vector<double>& kp, const std::vector<double>& kd,
                  const std::vector<double>& q_ref, const std::vector<double>& dq_ref,
                  const std::vector<double>& tau_ff);

    /// Configure global safety guards (RPC). nullopt leaves the guard unchanged.
    LiteArmValue set_guards(
        std::optional<double> slew_limit = std::nullopt,
        std::optional<double> tau_max = std::nullopt,
        std::optional<double> watchdog_timeout = std::nullopt,
        std::optional<bool> position_bounds = std::nullopt,
        std::optional<bool> velocity_bounds = std::nullopt,
        std::optional<bool> jerk_limit = std::nullopt);

    /// Read current guard configuration (RPC).
    LiteArmValue get_guards();

    /// Enable all motors and hold current pose (re-enable after disable()).
    void enable();

    /// Disable all motors (arm will drop under gravity!). CAN stays connected.
    void disable();

    /// Set end-effector payload (mass + center of mass).
    LiteArmValue set_payload(double mass,
                             const std::vector<double>& com = {0.0, 0.0, 0.0});

    /// Get current payload configuration.
    LiteArmValue get_payload();

    /// Set installation orientation (base RPY or gravity vector).
    LiteArmValue set_installation(
        std::optional<std::vector<double>> base_rpy = std::nullopt,
        std::optional<std::vector<double>> gravity = std::nullopt);

    /// Get current installation configuration.
    LiteArmValue get_installation();

    /// Set per-joint gravity calibration scale (7 non-negative values,
    /// multiplicative on the CAD gravity torque). Default transition_s=2.0s gives
    /// a smooth linear ramp so the compensation torque never jumps; transition_s<=0
    /// applies instantly (offline/static only).
    LiteArmValue set_gravity_scale(std::vector<double> scale,
                                   double transition_s = 2.0);

    /// Get current per-joint gravity calibration scale. During a transition the
    /// returned "scale" is the mid-transition value and "target" is set.
    LiteArmValue get_gravity_scale();

    /// Persist current gravity scale to the server yaml (effective after restart).
    LiteArmValue save_gravity_scale();

    /// Persist current payload (mass + com) to the server yaml (effective after restart).
    LiteArmValue save_payload();

    /// Persist current installation orientation (base_rpy) to the server yaml.
    LiteArmValue save_installation();

    // ── Peripheral devices ───────────────────────────────────────────────────

    /// Get a remote device interface (hand/gripper/teach), e.g. "hand_0".
    /// Methods are proxied through "device.{device_id}.{method}" RPCs.
    RemoteDevice device(const std::string& device_id);

    /// Access the lazy device manager (devices()["hand_0"] syntax).
    DeviceManager& devices();

    // ── Server extension RPCs (system / settings / trajectory / device / teleop) ──

    /// Get system stats (CPU, memory, board temperature, uptime).
    LiteArmValue get_system_stats();

    /// Get server logs (paginated).
    LiteArmValue get_logs(int page = 1, int size = 50,
                          const std::string& search = "");

    /// Request restart of the arm service.
    LiteArmValue restart_service();

    /// Reconnect hardware from any state — re-initialize motors after arm hot-restart.
    /// Returns { "state": str, "success": bool, "error": str (optional) }.
    LiteArmValue reconnect();

    LiteArmValue get_joint_limits();
    LiteArmValue set_joint_limits(const LiteArmValue& limits);

    LiteArmValue get_zero_offsets();
    LiteArmValue set_zero_offsets(const LiteArmValue& offsets);

    LiteArmValue get_end_effector();
    LiteArmValue set_end_effector(const LiteArmValue& config);

    LiteArmValue get_cartesian_limits();
    LiteArmValue set_cartesian_limits(const LiteArmValue& limits);

    LiteArmValue get_collision_config();
    LiteArmValue set_collision_config(const LiteArmValue& config);

    /// Server-side recording / trajectory CRUD.
    LiteArmValue start_recording();
    LiteArmValue stop_recording();
    LiteArmValue discard_recording();
    LiteArmValue get_recording_state();
    LiteArmValue get_playback_state();
    LiteArmValue list_trajectories();
    LiteArmValue save_trajectory(const std::string& id,
                                 const std::string& name,
                                 const std::vector<std::vector<double>>& points,
                                 std::optional<double> duration = std::nullopt);
    LiteArmValue delete_trajectory(const std::string& id);

    /// On-demand device daemon management (server forks device_daemon).
    LiteArmValue list_device_types();
    LiteArmValue connect_device(const std::string& category,
                                const std::string& subtype,
                                const std::string& device_id = "end_0",
                                const std::string& can_iface = "",
                                const LiteArmValue& config = LiteArmValue(nullptr));
    LiteArmValue disconnect_device(const std::string& device_id = "end_0");
    LiteArmValue get_active_device(const std::string& device_id = "end_0");

    /// Master/slave teleoperation (shares state with CLI --teleop-mode).
    LiteArmValue enter_teleop(
        const std::string& mode,
        const std::map<std::string, LiteArmValue>& params = {});
    LiteArmValue exit_teleop();
    LiteArmValue get_teleop_status();

    // ── Lifecycle ───────────────────────────────────────────────────────────

    /// Close the transport connection.
    void close();

    /// Arm identifier.
    const std::string& arm_id() const { return arm_id_; }

private:
    /// Send an RPC call and return the result (or throw on error).
    LiteArmValue rpc(const std::string& method,
                     std::map<std::string, LiteArmValue> kwargs = {});

    std::shared_ptr<Transport> tp_;
    std::string endpoint_;
    std::string arm_id_;
    std::string rpc_topic_;
    std::string estop_topic_;
    std::string command_topic_;
    std::string client_id_;
    int seq_ = 0;
    std::shared_ptr<Sub> state_sub_;
    std::optional<RobotState> last_state_;
    std::shared_ptr<DeviceManager> devices_;  // lazy (created on first use)
};

} // namespace litearm
