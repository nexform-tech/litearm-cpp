#pragma once
/**
 * @file arm.hpp
 * @brief Remote Arm client — API-compatible with pylitearm.Arm.
 *
 * Connects to litearm-server via zenoh and forwards all calls as RPC.
 * No pylitearm dependency, no Pinocchio, no Eigen.
 */

#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

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
    /// @param endpoint  Zenoh endpoint (e.g. "tcp/127.0.0.1:7447")
    /// @param arm_id    Arm identifier (default "armA")
    /// @param transport Optional pre-configured Transport (for testing)
    explicit Arm(const std::string& endpoint = "tcp/127.0.0.1:7447",
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
    bool replay_trajectory(const JointTrajectory& traj,
                           double speed = 1.0,
                           bool goto_start = true,
                           double goto_speed = 0.3,
                           std::optional<int> max_cycles = std::nullopt);

    /// Replay from raw value (dict-like).
    bool replay_trajectory(const LiteArmValue& traj_q,
                           double speed = 1.0,
                           bool goto_start = true,
                           double goto_speed = 0.3,
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
    bool zero_gravity(std::optional<int> max_cycles = std::nullopt,
                      std::optional<double> duration_s = std::nullopt);

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
                             std::optional<int> max_cycles = std::nullopt);

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
    std::string arm_id_;
    std::string rpc_topic_;
    std::string estop_topic_;
    std::shared_ptr<Sub> state_sub_;
    std::optional<RobotState> last_state_;
};

} // namespace litearm
