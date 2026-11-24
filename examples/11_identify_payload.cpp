// Example 11 - end-effector payload (point mass + COM) identification.
//
// The firmware models a payload as a point mass with a COM in the ee flange
// frame: set_payload(mass, com).  This example identifies those four numbers
// from static gravity poses.
//
// The empty-arm model is the firmware's own dynamics bank by default: live
// runs read body1..7 with arm.model().get_body() and convert them to the
// DH/Craig-MDH phi used by RNEA.  --load-params FILE overrides that with an
// example-10 result; offline replay has no arm, so it still requires it.
//
// The firmware payload feedforward is zeroed before a loaded capture (it
// otherwise contaminates the measured torques); write the result back after.
//
// The pose family is wrist-only: J1 and J3 stay at zero, J2 is fixed at 0.9,
// J4 at -0.8, and only J5..J7 vary over 20 representative wrist poses, so
// the fit uses J5..J7 only (rank 4, condition 4.33).  This avoids the front
// axes, whose model/friction error is larger.
//
// Recommended (differential) workflow:
//   1. remove the payload, capture an empty baseline:
//        ./11_identify_payload --go --capture-baseline payload_baseline.csv
//   2. install the payload, identify against that baseline:
//        ./11_identify_payload --go --baseline payload_baseline.csv
//
// Without a baseline (absolute mode, more sensitive to model/friction error):
//   ./11_identify_payload --go --absolute
//
// Offline replay of captured CSVs (no arm):
//   ./11_identify_payload --loaded-csv payload_loaded_<stamp>.csv \
//       --baseline-csv payload_baseline.csv \
//       --load-params ident_result/SN_<stamp>.yaml
//
// Read-only plan (connects, prints preflight, never moves):
//   ./11_identify_payload --load-params ident_result/SN_<stamp>.yaml
//   ./11_identify_payload                       # firmware bank as the model
//
// --apply writes the result to the firmware's RAM payload parameters; it never
// calls save_params(), so a reboot restores the previous payload.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "litearm/errors.hpp"
#include "litearm/ident/export.hpp"
#include "litearm/ident/model.hpp"
#include "litearm/ident/payload.hpp"
#include "litearm/rot.hpp"

#include "_common.hpp"

using namespace litearm;
namespace ids = litearm::ident;

namespace {

using Vec3 = ids::Vec3;
using Vec7 = ids::Vec7;

constexpr int kN = ids::kJoints;

// Wrist-only family: J1 and J3 stay at zero, J2 is fixed at 0.9, J4 at -0.8,
// and only J5..J7 vary over 20 representative wrist configurations (corners,
// edges and centre): rank 4 with J5..J7 (condition 4.33), EE z 0.25..0.40 m.
// The front axes (model error is larger) are kept out of both the motion and
// the regression.
const std::array<Vec7, 20> kWristPoses{{
    {0.0, 0.9, 0.0, -0.8, 1.2, 0.1, -0.8},
    {0.0, 0.9, 0.0, -0.8, 1.2, 0.1, 0.8},
    {0.0, 0.9, 0.0, -0.8, 1.2, 1.2, -0.8},
    {0.0, 0.9, 0.0, -0.8, 1.2, 1.2, 0.8},
    {0.0, 0.9, 0.0, -0.8, -1.2, 0.1, -0.8},
    {0.0, 0.9, 0.0, -0.8, -1.2, 0.1, 0.8},
    {0.0, 0.9, 0.0, -0.8, -1.2, 1.2, -0.8},
    {0.0, 0.9, 0.0, -0.8, -1.2, 1.2, 0.8},
    {0.0, 0.9, 0.0, -0.8, 0.0, 0.1, -0.8},
    {0.0, 0.9, 0.0, -0.8, 0.0, 0.1, 0.8},
    {0.0, 0.9, 0.0, -0.8, 0.0, 1.2, -0.8},
    {0.0, 0.9, 0.0, -0.8, 0.0, 1.2, 0.8},
    {0.0, 0.9, 0.0, -0.8, 0.8, 0.6, 0.0},
    {0.0, 0.9, 0.0, -0.8, -0.8, 0.6, 0.0},
    {0.0, 0.9, 0.0, -0.8, 0.4, 0.9, 0.4},
    {0.0, 0.9, 0.0, -0.8, -0.4, 0.9, -0.4},
    {0.0, 0.9, 0.0, -0.8, 0.4, 0.3, -0.4},
    {0.0, 0.9, 0.0, -0.8, -0.4, 0.3, 0.4},
    {0.0, 0.9, 0.0, -0.8, 0.6, 0.5, -0.6},
    {0.0, 0.9, 0.0, -0.8, -0.6, 0.5, 0.6},
}};

/// Fixed approach pose, used before every capture series so the baseline and
/// loaded sessions approach each target from the same direction.
const Vec7 kLeadIn{0.0, 0.9, 0.0, -0.8, 0.0, 0.5, 0.0};

struct ArmLimits {
    Vec7 q_min{};
    Vec7 q_max{};
    Vec7 tau_max{};
};

struct CaptureConfig {
    int samples = 80;
    double sample_interval_s = 0.012;
    double speed = 0.10;
    double max_pose_error_rad = 0.15;
};

struct CaptureMeta {
    std::string kind;
    std::string serial = "UNASSIGNED";
    std::string firmware;
    Vec3 gravity{0.0, 0.0, -9.81};
    double bootstrap_mass = 0.0;
    Vec3 bootstrap_com{};
    int samples_per_pose = 0;
    double sample_interval_s = 0.0;
    double speed = 0.0;
};

struct PoseCapture {
    Vec7 target{};
    Vec7 q{};
    Vec7 tau{};
    Vec7 tau_std{};
    double pose_error_rad = 0.0;
    double max_abs_dq = 0.0;
};

std::string timestamp_suffix() {
    const std::time_t now = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &now);
#else
    localtime_r(&now, &tm);
#endif
    char buf[32] = {};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

std::string read_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw InvalidCommandError("cannot read " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string num(double v) {
    if (std::isnan(v)) return ".nan";
    if (std::isinf(v)) return v > 0 ? ".inf" : "-.inf";
    std::ostringstream os;
    os << std::setprecision(17) << v;
    return os.str();
}

Vec7 parse_vec7(const std::string& text, const Vec7& fallback) {
    Vec7 out = fallback;
    std::istringstream ss(text);
    std::string part;
    int i = 0;
    while (std::getline(ss, part, ',') && i < kN) {
        out[size_t(i++)] = std::atof(part.c_str());
    }
    if (i != kN) throw InvalidCommandError("expected 7 comma-separated values: " + text);
    return out;
}

Vec3 parse_vec3(const std::string& text, const Vec3& fallback) {
    Vec3 out = fallback;
    std::istringstream ss(text);
    std::string part;
    int i = 0;
    while (std::getline(ss, part, ',') && i < 3) {
        out[size_t(i++)] = std::atof(part.c_str());
    }
    if (i != 3) throw InvalidCommandError("expected 3 comma-separated values: " + text);
    return out;
}

std::vector<double> to_std(const Vec7& v) {
    return std::vector<double>(v.begin(), v.end());
}

Vec7 to_vec7(const std::vector<double>& v) {
    Vec7 out{};
    for (int j = 0; j < kN && j < int(v.size()); ++j) out[size_t(j)] = v[size_t(j)];
    return out;
}

bool finite3(const Vec3& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    if (n % 2 != 0) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

ArmLimits read_limits(Arm& arm) {
    ArmLimits lim;
    const auto jps = arm.params().all_joint_params();
    if (int(jps.size()) != kN) {
        throw InvalidCommandError("could not read all 7 joint parameter blocks");
    }
    for (int j = 0; j < kN; ++j) {
        lim.q_min[size_t(j)] = jps[size_t(j)].q_min;
        lim.q_max[size_t(j)] = jps[size_t(j)].q_max;
        lim.tau_max[size_t(j)] = jps[size_t(j)].tau_max;
    }
    return lim;
}

void check_pose_limits(const ArmLimits& lim, const std::vector<Vec7>& poses,
                       const char* what) {
    for (const auto& q : poses) {
        for (int j = 0; j < kN; ++j) {
            if (q[size_t(j)] < lim.q_min[size_t(j)] - 1e-6 ||
                q[size_t(j)] > lim.q_max[size_t(j)] + 1e-6) {
                throw InvalidCommandError(
                    std::string(what) + ": joint " + std::to_string(j + 1) +
                    " target outside soft limits");
            }
        }
    }
}

void print_payload_state(Arm& arm) {
    try {
        const double mass = arm.get_ff_scalar(4).value;
        const Vec3 com{arm.get_ff_scalar(5, 0).value,
                       arm.get_ff_scalar(5, 1).value,
                       arm.get_ff_scalar(5, 2).value};
        std::printf("firmware payload: mass=%.4f kg com=[%.4f, %.4f, %.4f]\n",
                    mass, com[0], com[1], com[2]);
    } catch (const std::exception& e) {
        std::printf("firmware payload readback failed: %s\n", e.what());
    }
}

Vec3 read_live_gravity(Arm& arm, const example::Args& args) {
    Vec3 g{0.0, 0.0, -9.81};
    bool ok = true;
    try {
        for (int k = 0; k < 3; ++k) g[size_t(k)] = arm.get_ff_scalar(6, k).value;
        ok = finite3(g) && std::abs(g[0]) + std::abs(g[1]) + std::abs(g[2]) > 1e-6;
    } catch (const std::exception&) {
        ok = false;
    }
    if (!ok) {
        if (args.has("gravity")) g = parse_vec3(args.str("gravity", ""), g);
        std::printf("WARNING: could not read firmware gravity; using [%.3f, %.3f, %.3f]\n",
                    g[0], g[1], g[2]);
    }
    return g;
}

/// Read the firmware's authoritative dynamics bank as the empty-arm model.
/// get_body() reads the bank without payload composition, in body10 form
/// (m, COM, I_com); convert it back to the joint-origin phi used by RNEA.
ids::Phi70 read_firmware_phi(Arm& arm) {
    ids::Phi70 phi{};
    for (int i = 0; i < kN; ++i) {
        const auto body = arm.model().get_body(i + 1).value;
        if (body.size() != 10) {
            throw InvalidCommandError(
                "firmware model read: body " + std::to_string(i + 1) + " has " +
                std::to_string(body.size()) + " values");
        }
        std::array<double, 10> b{};
        for (int k = 0; k < 10; ++k) b[size_t(k)] = body[size_t(k)];
        const auto p = ids::body10_to_phi(b);
        for (int k = 0; k < 10; ++k) phi[size_t(i * 10 + k)] = p[size_t(k)];
    }
    return phi;
}

/// Empty-arm model for identification: live runs default to the firmware bank;
/// --load-params overrides it, and offline replay requires it.
ids::Phi70 load_phi(const example::Args& args, Arm* arm, std::string* source) {
    const std::string path = args.str("load-params", "");
    if (path.empty()) {
        if (arm == nullptr) {
            throw InvalidCommandError(
                "--load-params FILE is required for offline replay (no arm to "
                "read the firmware model)");
        }
        *source = "firmware model bank (get_body 1..7)";
        return read_firmware_phi(*arm);
    }
    const std::string text = read_file(path);
    // An SN config describes the controller's Craig/MDH link frames by
    // construction.  Legacy identified_params files must declare that frame.
    const bool sn = text.find("robot_group:") != std::string::npos;
    if (!sn && text.find("phi_frame: craig_mdh") == std::string::npos) {
        throw InvalidCommandError(
            "--load-params must be a DH-frame identification result "
            "(SN config or phi_frame: craig_mdh_link_frames); " + path +
            " does not declare it");
    }
    *source = path;
    return ids::load_result_yaml(text).phi;
}

ids::PayloadOptions make_options(const example::Args& args, const Vec3& gravity) {
    ids::PayloadOptions o;
    o.joints = {4, 5, 6};  // zero-based J5..J7 (wrist-only family)
    o.max_payload_kg = args.num("max-payload-kg", o.max_payload_kg);
    o.max_com_radius_m = args.num("max-com-radius", o.max_com_radius_m);
    o.max_condition = args.num("condition-max", o.max_condition);
    o.huber_iterations = int(args.num("huber-iterations", o.huber_iterations));
    o.gravity = gravity;
    if (o.max_payload_kg <= 0.0 || o.max_payload_kg > 20.0) {
        throw InvalidCommandError("--max-payload-kg must be in (0, 20]");
    }
    return o;
}

CaptureConfig make_capture_config(const example::Args& args) {
    CaptureConfig c;
    c.speed = args.speed_explicit ? args.speed : 0.10;
    if (!(c.speed > 0.0) || c.speed > 0.20) {
        throw InvalidCommandError("--speed must be in (0, 0.2] for payload capture");
    }
    c.samples = int(args.num("samples", 80));
    if (c.samples < 20) {
        throw InvalidCommandError("--samples must be >= 20");
    }
    c.sample_interval_s = args.num("sample-interval", 0.012);
    if (c.sample_interval_s < 0.005) {
        throw InvalidCommandError("--sample-interval must be >= 0.005 s");
    }
    c.max_pose_error_rad = args.num("max-pose-error", 0.15);
    if (c.max_pose_error_rad < 0.0 || c.max_pose_error_rad > 0.20) {
        throw InvalidCommandError("--max-pose-error must be in [0, 0.20] rad");
    }
    return c;
}

int pose_count(const example::Args& args) {
    const double fallback = args.num("pose-count", double(kWristPoses.size()));
    const int n = int(args.num("poses", fallback));
    if (n < 3 || n > int(kWristPoses.size())) {
        throw InvalidCommandError("--pose-count must be in [3, " +
                                  std::to_string(kWristPoses.size()) + "]");
    }
    return n;
}

/// Pose selection: --pose-set "1,10,20" (1-based, any order) or the first
/// --pose-count poses from the wrist-only family.
std::vector<Vec7> select_poses(const example::Args& args) {
    if (!args.has("pose-set")) {
        const int n = pose_count(args);
        return std::vector<Vec7>(kWristPoses.begin(), kWristPoses.begin() + n);
    }
    std::vector<Vec7> out;
    std::istringstream ss(args.str("pose-set", ""));
    std::string part;
    while (std::getline(ss, part, ',')) {
        if (part.empty()) continue;
        const int idx = std::atoi(part.c_str());
        if (idx < 1 || idx > int(kWristPoses.size())) {
            throw InvalidCommandError("--pose-set indices must be in [1, " +
                                      std::to_string(kWristPoses.size()) + "]");
        }
        out.push_back(kWristPoses[size_t(idx - 1)]);
    }
    if (out.size() < 3 || out.size() > kWristPoses.size()) {
        throw InvalidCommandError("--pose-set needs 3.." +
                                  std::to_string(kWristPoses.size()) + " poses");
    }
    return out;
}

void check_tcp(Arm& arm, const RobotState& st) {
    const auto tcp = arm.get_tcp().value;
    if (!tcp) return;
    const ids::FkResult fk = ids::fk_ee(to_vec7(st.q()));
    const double dp = std::sqrt(
        (fk.p_ee[0] - (*tcp)[0]) * (fk.p_ee[0] - (*tcp)[0]) +
        (fk.p_ee[1] - (*tcp)[1]) * (fk.p_ee[1] - (*tcp)[1]) +
        (fk.p_ee[2] - (*tcp)[2]) * (fk.p_ee[2] - (*tcp)[2]));
    rot::Mat3 Rhost{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Rhost[size_t(i)][size_t(j)] = fk.R_ee[size_t(3 * i + j)];
    const rot::Mat3 Rfw =
        rot::rpy_to_mat({(*tcp)[3], (*tcp)[4], (*tcp)[5]});
    const rot::Mat3 Rerr = rot::mat_mul(rot::mat_T(Rhost), Rfw);
    const double trace = Rerr[0][0] + Rerr[1][1] + Rerr[2][2];
    const double angle = std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0));
    if (dp > 0.005 || angle > 0.05) {
        std::ostringstream os;
        os << "host FK does not match firmware TCP: position error " << dp
           << " m, orientation error " << angle << " rad";
        throw InvalidCommandError(os.str());
    }
}

void print_torque_margins(const ArmLimits& lim, const std::vector<Vec7>& poses,
                          const ids::Phi70& phi, const Vec3& gravity) {
    const Vec7 zero{};
    Vec7 worst{};
    for (const auto& q : poses) {
        const Vec7 tau = ids::rnea(q, zero, zero, phi, gravity);
        for (int j = 0; j < kN; ++j) {
            worst[size_t(j)] = std::max(worst[size_t(j)], std::abs(tau[size_t(j)]));
        }
    }
    std::printf("empty-model gravity torque (max over poses, payload not included):\n");
    bool over = false;
    for (int j = 0; j < kN; ++j) {
        const double ratio = worst[size_t(j)] / lim.tau_max[size_t(j)];
        std::printf("  J%d %7.3f / %6.1f Nm  (%.0f%%)%s\n", j + 1,
                    worst[size_t(j)], lim.tau_max[size_t(j)], 100.0 * ratio,
                    ratio > 1.0 ? "  OVER LIMIT" : (ratio > 0.8 ? "  high" : ""));
        over = over || ratio > 1.0;
    }
    if (over) {
        throw InvalidCommandError("predicted gravity torque exceeds a joint limit");
    }
}

void preflight(Arm& arm, const std::vector<Vec7>& poses,
               const ids::Phi70& margin_phi, const Vec3& gravity) {
    if (arm.n() != kN) {
        throw InvalidCommandError("requires a 7-joint arm");
    }
    const auto st = arm.get_state(true, 0.5).value;
    if (!st) throw InvalidCommandError("no state frame");
    if (st->faulted()) {
        throw InvalidCommandError("arm is faulted: " + st->fault_detail());
    }
    if (st->cart_busy()) {
        throw InvalidCommandError("arm is busy with a Cartesian motion");
    }
    const ArmLimits lim = read_limits(arm);
    for (int j = 0; j < kN && j < int(st->q().size()); ++j) {
        const double q = st->q()[size_t(j)];
        if (q < lim.q_min[size_t(j)] - 0.01 || q > lim.q_max[size_t(j)] + 0.01) {
            throw InvalidCommandError(
                "current pose is outside the soft limits; home or move it first");
        }
    }
    check_pose_limits(lim, poses, "target pose");
    check_tcp(arm, *st);
    std::printf("gravity: [%.4f, %.4f, %.4f] m/s^2\n", gravity[0], gravity[1],
                gravity[2]);
    print_torque_margins(lim, poses, margin_phi, gravity);
    print_payload_state(arm);
}

PoseCapture capture_pose(Arm& arm, const Vec7& target, const CaptureConfig& cfg) {
    bool timed_out = false;
    try {
        arm.movej(to_std(target), cfg.speed);
    } catch (const MotionTimeoutError&) {
        timed_out = true;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(
                              std::max(3000, int(cfg.samples * cfg.sample_interval_s * 4000.0)));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<Vec7> qs;
    std::vector<Vec7> taus;
    double max_dq = 0.0;
    qs.reserve(size_t(cfg.samples));
    taus.reserve(size_t(cfg.samples));
    while (int(qs.size()) < cfg.samples &&
           std::chrono::steady_clock::now() < deadline) {
        try {
            const auto m = arm.get_state(true, 0.2);
            if (m.value) {
                const RobotState& st = *m.value;
                if (st.faulted()) {
                    throw MotorFaultError("fault while sampling: " + st.fault_detail());
                }
                double dq_max = 0.0;
                for (double v : st.dq()) dq_max = std::max(dq_max, std::abs(v));
                if (dq_max <= 0.03) {
                    qs.push_back(to_vec7(st.q()));
                    taus.push_back(to_vec7(st.tau()));
                    max_dq = std::max(max_dq, dq_max);
                }
            }
        } catch (const MotorFaultError&) {
            throw;
        } catch (const std::exception&) {
            // Transient status-read failure: keep sampling until the deadline.
        }
        std::this_thread::sleep_for(
            std::chrono::duration<double>(cfg.sample_interval_s));
    }
    if (int(qs.size()) < cfg.samples) {
        throw InvalidCommandError("not enough static samples at a pose");
    }

    PoseCapture out;
    out.target = target;
    std::vector<double> col(size_t(qs.size()));
    for (int j = 0; j < kN; ++j) {
        for (size_t i = 0; i < qs.size(); ++i) col[i] = qs[i][size_t(j)];
        out.q[size_t(j)] = median(col);
        for (size_t i = 0; i < qs.size(); ++i) col[i] = taus[i][size_t(j)];
        out.tau[size_t(j)] = median(col);
        double sum = 0.0;
        for (double v : col) sum += v;
        const double mean = sum / double(col.size());
        double ss = 0.0;
        for (double v : col) ss += (v - mean) * (v - mean);
        out.tau_std[size_t(j)] = std::sqrt(ss / double(col.size()));
    }
    double pose_error = 0.0;
    for (int j = 0; j < kN; ++j) {
        pose_error = std::max(pose_error, std::abs(out.q[size_t(j)] - target[size_t(j)]));
    }
    out.pose_error_rad = pose_error;
    out.max_abs_dq = max_dq;
    if (timed_out) {
        std::printf("  WARNING: movej timed out; measured pose error %.4f rad\n",
                    pose_error);
    }
    if (pose_error > cfg.max_pose_error_rad) {
        throw InvalidCommandError("pose error " + std::to_string(pose_error) +
                                  " rad exceeds --max-pose-error");
    }
    return out;
}

ids::PayloadCapture capture_series(Arm& arm, const std::vector<Vec7>& targets,
                                   const CaptureConfig& cfg,
                                   const CaptureMeta& meta,
                                   const std::string& path) {
    ids::PayloadCapture cap;
    cap.kind = meta.kind;
    cap.robot_serial = meta.serial;
    cap.firmware = meta.firmware;
    cap.gravity = meta.gravity;
    cap.bootstrap_mass = meta.bootstrap_mass;
    cap.bootstrap_com = meta.bootstrap_com;
    cap.samples_per_pose = meta.samples_per_pose;
    cap.sample_interval_s = meta.sample_interval_s;
    cap.speed = meta.speed;

    // Loaded PD droop can exceed the SDK's default 0.03 rad arrival tolerance,
    // making every movej burn the full 15 s timeout.  The measurement uses the
    // measured q and enforces cfg.max_pose_error_rad, so a looser arrival window
    // and a shorter timeout are safe here and keep the capture fast.
    if (arm.q_tol < 0.12) arm.q_tol = 0.12;
    if (arm.move_timeout > 6.0) arm.move_timeout = 6.0;

    try {
        arm.movej(to_std(kLeadIn), cfg.speed);
    } catch (const MotionTimeoutError&) {
        // Loaded PD droop can exceed the SDK's q_tol (0.03 rad) at this approach
        // pose.  The lead-in is only a repeatable approach direction, not a
        // measurement, so continue from the measured pose like capture_pose does.
        std::printf("WARNING: lead-in move timed out (loaded droop); "
                    "continuing from the measured pose\n");
    }
    for (size_t i = 0; i < targets.size(); ++i) {
        std::printf("pose %zu/%zu ...\n", i + 1, targets.size());
        const PoseCapture pc = capture_pose(arm, targets[i], cfg);
        std::printf("  q=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f] pose_err=%.4f "
                    "max|dq|=%.4f\n",
                    pc.q[0], pc.q[1], pc.q[2], pc.q[3], pc.q[4], pc.q[5],
                    pc.q[6], pc.pose_error_rad, pc.max_abs_dq);
        cap.target_q.push_back(pc.target);
        cap.samples.push_back({pc.q, pc.tau});
        cap.tau_std.push_back(pc.tau_std);
        cap.pose_error_rad.push_back(pc.pose_error_rad);
        cap.max_abs_dq.push_back(pc.max_abs_dq);
        cap.complete = false;
        ids::write_payload_capture(path, cap);
    }
    cap.complete = true;
    ids::write_payload_capture(path, cap);
    std::printf("wrote %s\n", path.c_str());
    return cap;
}

void print_result(const ids::PayloadResult& r) {
    std::printf("method=%s poses=%d equations=%d rank=%d condition=%.3g\n",
                r.method.c_str(), r.poses, r.equations, r.rank, r.condition);
    std::printf("mass = %.6f kg\n", r.mass);
    std::printf("com  = [%.6f, %.6f, %.6f] m (ee flange)\n", r.com[0], r.com[1],
                r.com[2]);
    std::printf("first moment = [%.6f, %.6f, %.6f] kg*m\n", r.first_moment[0],
                r.first_moment[1], r.first_moment[2]);
    std::printf("residual rms=%.6f Nm max=%.6f Nm R^2=%.6f\n",
                r.residual_rms_nm, r.residual_max_nm, r.r_squared);
}

void emit_vec(std::ostream& os, const Vec7& v) {
    os << "[" << num(v[0]);
    for (int j = 1; j < kN; ++j) os << ", " << num(v[size_t(j)]);
    os << "]";
}

void write_result_yaml(const ids::PayloadResult& r,
                       const ids::PayloadCapture& capture,
                       const std::string& out_dir, const std::string& stamp,
                       const std::string& model_source,
                       const std::string& baseline_file,
                       const std::string& loaded_file) {
    std::filesystem::create_directories(out_dir);
    const std::string path = out_dir + "/payload_result_" + stamp + ".yaml";
    std::ofstream os(path);
    if (!os) throw InvalidCommandError("cannot write " + path);
    os << "schema: litearm.payload_identification.v1\n";
    os << "kind: result\n";
    os << "description: end-effector payload identification (point mass)\n";
    os << "method: " << r.method << "\n";
    os << "payload_model: point_mass\n";
    os << "com_frame: ee_flange (DH/litearm_dh.urdf)\n";
    os << "mass: " << num(r.mass) << "\n";
    os << "com: [" << num(r.com[0]) << ", " << num(r.com[1]) << ", "
       << num(r.com[2]) << "]\n";
    os << "first_moment_kg_m: [" << num(r.first_moment[0]) << ", "
       << num(r.first_moment[1]) << ", " << num(r.first_moment[2]) << "]\n";
    os << "residual_rms_nm: " << num(r.residual_rms_nm) << "\n";
    os << "residual_max_nm: " << num(r.residual_max_nm) << "\n";
    os << "r_squared: " << num(r.r_squared) << "\n";
    os << "rank: " << r.rank << "\n";
    os << "condition: " << num(r.condition) << "\n";
    os << "equations: " << r.equations << "\n";
    os << "poses: " << r.poses << "\n";
    os << "joints: [";
    for (size_t i = 0; i < r.joints.size(); ++i) {
        os << (i ? ", " : "") << (r.joints[i] + 1);
    }
    os << "]\n";
    os << "gravity_m_s2: [" << num(capture.gravity[0]) << ", "
       << num(capture.gravity[1]) << ", " << num(capture.gravity[2]) << "]\n";
    os << "model_source: " << model_source << "\n";
    os << "model_phi_frame: craig_mdh_link_frames\n";
    os << "baseline_file: " << (baseline_file.empty() ? "none" : baseline_file) << "\n";
    os << "loaded_file: " << loaded_file << "\n";
    os << "bootstrap_payload:\n";
    os << "  mass: " << num(capture.bootstrap_mass) << "\n";
    os << "  com: [" << num(capture.bootstrap_com[0]) << ", "
       << num(capture.bootstrap_com[1]) << ", "
       << num(capture.bootstrap_com[2]) << "]\n";
    os << "capture:\n";
    os << "  samples_per_pose: " << capture.samples_per_pose << "\n";
    os << "  sample_interval_s: " << num(capture.sample_interval_s) << "\n";
    os << "  speed: " << num(capture.speed) << "\n";
    os << "per_pose:\n";
    for (size_t i = 0; i < capture.samples.size(); ++i) {
        os << "  - index: " << i << "\n";
        os << "    target_q: ";
        emit_vec(os, capture.target_q[i]);
        os << "\n    q: ";
        emit_vec(os, capture.samples[i].q);
        os << "\n    tau: ";
        emit_vec(os, capture.samples[i].tau);
        os << "\n    tau_std: ";
        emit_vec(os, capture.tau_std[i]);
        os << "\n    pose_error_rad: " << num(capture.pose_error_rad[i]) << "\n";
        os << "    max_abs_dq: " << num(capture.max_abs_dq[i]) << "\n";
    }
    std::printf("wrote %s\n", path.c_str());
}

void apply_payload(Arm& arm, const ids::PayloadResult& r) {
    std::printf("WARNING: set_payload writes RAM only; save_params() is not called\n");
    arm.set_payload(r.mass, r.com);
    const double mass = arm.get_ff_scalar(4).value;
    const Vec3 com{arm.get_ff_scalar(5, 0).value, arm.get_ff_scalar(5, 1).value,
                   arm.get_ff_scalar(5, 2).value};
    std::printf("payload readback: mass=%.6f kg com=[%.6f, %.6f, %.6f]\n", mass,
                com[0], com[1], com[2]);
    if (std::abs(mass - r.mass) > 1e-4 || std::abs(com[0] - r.com[0]) > 1e-4 ||
        std::abs(com[1] - r.com[1]) > 1e-4 || std::abs(com[2] - r.com[2]) > 1e-4) {
        std::printf("WARNING: firmware clamped or altered the payload values\n");
    }
}

int run_offline(const example::Args& args) {
    if (args.has("apply")) {
        throw InvalidCommandError("--apply requires a live arm");
    }
    if (args.go) {
        throw InvalidCommandError("--go is not used in offline mode");
    }
    if (args.has("capture-baseline") || args.has("baseline") ||
        args.has("capture-loaded") || args.has("bootstrap-mass")) {
        throw InvalidCommandError(
            "live-only flags cannot be used with --loaded-csv");
    }
    const std::string loaded_path = args.str("loaded-csv", "");
    if (loaded_path.empty()) {
        throw InvalidCommandError("offline mode requires --loaded-csv");
    }
    const bool has_baseline = args.has("baseline-csv");
    const bool absolute = args.has("absolute");
    if (has_baseline == absolute) {
        throw InvalidCommandError(
            "offline mode needs exactly one of --baseline-csv or --absolute");
    }
    ids::PayloadCapture loaded = ids::load_payload_capture(loaded_path, "loaded");
    ids::PayloadCapture baseline;
    std::vector<ids::PayloadSample>* baseline_samples = nullptr;
    std::string baseline_path;
    if (has_baseline) {
        baseline_path = args.str("baseline-csv", "");
        baseline = ids::load_payload_capture(baseline_path, "baseline");
        if (baseline.samples.size() != loaded.samples.size()) {
            throw InvalidCommandError(
                "baseline and loaded captures have different pose counts");
        }
        for (int k = 0; k < 3; ++k) {
            if (std::abs(baseline.gravity[size_t(k)] - loaded.gravity[size_t(k)]) >
                0.02) {
                throw InvalidCommandError(
                    "baseline and loaded captures used different gravity");
            }
        }
        for (size_t i = 0; i < loaded.target_q.size() && i < baseline.target_q.size(); ++i) {
            for (int j = 0; j < kN; ++j) {
                if (std::abs(baseline.target_q[i][size_t(j)] -
                             loaded.target_q[i][size_t(j)]) > 0.05) {
                    throw InvalidCommandError(
                        "baseline and loaded captures use different pose sets at "
                        "pose " + std::to_string(i + 1));
                }
            }
        }
        baseline_samples = &baseline.samples;
    }

    std::string model_source;
    const ids::Phi70 phi = load_phi(args, nullptr, &model_source);
    const ids::PayloadOptions opt = make_options(args, loaded.gravity);
    const ids::PayloadResult r =
        ids::solve_payload(loaded.samples, baseline_samples, &phi, opt);
    print_result(r);
    write_result_yaml(r, loaded, args.str("out", "ident_result"),
                      timestamp_suffix(), model_source, baseline_path, loaded_path);
    return 0;
}

int run_plan(Arm& arm, const std::vector<Vec7>& poses, const example::Args& args,
             const ids::Phi70& margin_phi, const std::string& model_source) {
    std::printf("model frames: Craig/MDH link frames (DH/litearm_dh.urdf)\n");
    if (arm.n() != kN) throw InvalidCommandError("requires a 7-joint arm");
    const ArmLimits lim = read_limits(arm);
    check_pose_limits(lim, poses, "target pose");
    const Vec3 gravity = read_live_gravity(arm, args);
    const auto st = arm.get_state(true, 0.5).value;
    if (!st) throw InvalidCommandError("no state frame");
    if (st->faulted()) throw InvalidCommandError("arm is faulted: " + st->fault_detail());
    if (st->cart_busy()) throw InvalidCommandError("arm is busy with a Cartesian motion");
    check_tcp(arm, *st);
    std::printf("read-only plan: %d poses, J5..J7 (wrist-only)\n",
                int(poses.size()));
    std::printf("torque margin model: %s\n", model_source.c_str());
    print_torque_margins(lim, poses, margin_phi, gravity);
    print_payload_state(arm);
    const ids::PayloadOptions opt = make_options(args, gravity);
    const auto diag = ids::payload_pose_diagnostics(poses, opt);
    std::printf("payload pose set: rank=%d equations=%d condition=%.3g\n",
                diag.rank, diag.equations, diag.condition);
    if (diag.rank < ids::kPayloadParams) {
        std::printf("WARNING: pose set rank is below 4; mass and COM are not "
                    "fully observable\n");
    }
    std::printf("baseline capture:\n  ./11_identify_payload --go "
                "--capture-baseline payload_baseline.csv\n");
    std::printf("loaded identification:\n  ./11_identify_payload --go "
                "--baseline payload_baseline.csv   # firmware bank is the model\n");
    std::printf("read-only plan; add --go to capture\n");
    return 0;
}

int run_capture_baseline(Arm& arm, const std::vector<Vec7>& poses,
                         const example::Args& args,
                         const ids::Phi70& margin_phi) {
    if (args.has("load-params")) {
        throw InvalidCommandError("--load-params is not used for baseline capture");
    }
    if (args.num("bootstrap-mass", 0.0) != 0.0) {
        throw InvalidCommandError("--bootstrap-mass must be 0 for baseline capture");
    }
    const std::string path = args.str("capture-baseline", "");
    if (path.empty()) throw InvalidCommandError("--capture-baseline needs a file path");
    const Vec3 gravity = read_live_gravity(arm, args);
    preflight(arm, poses, margin_phi, gravity);
    std::printf("WARNING: remove the payload from the arm before baseline capture\n");
    arm.enable();
    arm.set_payload(0.0, {0.0, 0.0, 0.0});
    const double mass = arm.get_ff_scalar(4).value;
    if (std::abs(mass) > 1e-6) {
        std::printf("WARNING: firmware payload mass is %.4f, expected 0\n", mass);
    }
    const CaptureConfig cfg = make_capture_config(args);
    CaptureMeta meta;
    meta.kind = "baseline";
    meta.serial = args.str("serial", "UNASSIGNED");
    meta.firmware = arm.firmware();
    meta.gravity = gravity;
    meta.samples_per_pose = cfg.samples;
    meta.sample_interval_s = cfg.sample_interval_s;
    meta.speed = cfg.speed;
    capture_series(arm, poses, cfg, meta, path);
    std::printf("baseline capture complete; arm remains enabled and holding\n");
    return 0;
}

int run_identify_live(Arm& arm, const std::vector<Vec7>& poses,
                      const example::Args& args, bool use_baseline) {
    std::string model_source;
    const ids::Phi70 phi = load_phi(args, &arm, &model_source);
    const Vec3 gravity = read_live_gravity(arm, args);
    preflight(arm, poses, phi, gravity);

    ids::PayloadCapture baseline;
    std::vector<ids::PayloadSample> baseline_used;
    std::vector<ids::PayloadSample>* baseline_samples = nullptr;
    std::string baseline_path;
    if (use_baseline) {
        baseline_path = args.str("baseline", "");
        if (baseline_path.empty()) throw InvalidCommandError("--baseline needs a file");
        baseline = ids::load_payload_capture(baseline_path, "baseline");
        if (baseline.samples.size() < poses.size()) {
            throw InvalidCommandError(
                "baseline has fewer poses than the requested pose count");
        }
        for (int k = 0; k < 3; ++k) {
            if (std::abs(baseline.gravity[size_t(k)] - gravity[size_t(k)]) > 0.02) {
                throw InvalidCommandError(
                    "baseline and current firmware use different gravity");
            }
        }
        if (baseline.created_unix > 0) {
            const std::int64_t now = std::int64_t(std::time(nullptr));
            if (now - baseline.created_unix > 86400) {
                std::printf("WARNING: baseline is more than 24 h old; "
                            "re-capture it near the loaded run\n");
            }
        }
        for (size_t i = 0; i < poses.size() && i < baseline.target_q.size(); ++i) {
            for (int j = 0; j < kN; ++j) {
                if (std::abs(baseline.target_q[i][size_t(j)] - poses[i][size_t(j)]) >
                    0.05) {
                    throw InvalidCommandError(
                        "baseline pose " + std::to_string(i + 1) +
                        " does not match the current pose set; use the same "
                        "--pose-set for the baseline and the loaded capture");
                }
            }
        }
        baseline_used.assign(baseline.samples.begin(),
                             baseline.samples.begin() +
                                 std::ptrdiff_t(poses.size()));
        baseline_samples = &baseline_used;
    } else {
        std::printf("WARNING: absolute mode uses the identified empty-arm model; "
                    "a differential baseline is more accurate\n");
    }

    const double bootstrap_mass = args.num("bootstrap-mass", 0.0);
    const Vec3 bootstrap_com = parse_vec3(args.str("bootstrap-com", "0,0,0"),
                                          Vec3{0.0, 0.0, 0.0});
    if (bootstrap_mass < 0.0) {
        throw InvalidCommandError("--bootstrap-mass must be >= 0");
    }
    if (bootstrap_mass > 0.0) {
        arm.set_payload(bootstrap_mass, bootstrap_com);
        std::printf("bootstrap payload: mass=%.4f com=[%.4f, %.4f, %.4f]\n",
                    bootstrap_mass, bootstrap_com[0], bootstrap_com[1],
                    bootstrap_com[2]);
        std::printf("WARNING: the bootstrap payload is active while sampling; it "
                    "biases the identified payload (clear it for a clean fit)\n");
    } else {
        // Any payload already in the firmware feedforward contaminates the
        // measured torques.  Zero it for the identification; write the result
        // back afterwards.
        const double fw_mass = arm.get_ff_scalar(4).value;
        if (std::abs(fw_mass) > 1e-6) {
            std::printf("WARNING: firmware payload mass %.4f cleared for "
                        "identification (write the result back afterwards)\n",
                        fw_mass);
        }
        arm.set_payload(0.0, {0.0, 0.0, 0.0});
    }

    const CaptureConfig cfg = make_capture_config(args);
    arm.enable();
    CaptureMeta meta;
    meta.kind = "loaded";
    meta.serial = args.str("serial", "UNASSIGNED");
    meta.firmware = arm.firmware();
    meta.gravity = gravity;
    meta.bootstrap_mass = bootstrap_mass;
    meta.bootstrap_com = bootstrap_com;
    meta.samples_per_pose = cfg.samples;
    meta.sample_interval_s = cfg.sample_interval_s;
    meta.speed = cfg.speed;
    std::string loaded_path = args.str("capture-loaded", "");
    if (loaded_path.empty()) {
        loaded_path = args.str("out", "ident_result") + "/payload_loaded_" +
                      timestamp_suffix() + ".csv";
    }
    const ids::PayloadCapture loaded =
        capture_series(arm, poses, cfg, meta, loaded_path);

    if (use_baseline) {
        double max_delta = 0.0;
        for (size_t i = 0; i < poses.size() && i < baseline.samples.size(); ++i) {
            for (int j = 0; j < kN; ++j) {
                max_delta = std::max(
                    max_delta, std::abs(loaded.samples[i].q[size_t(j)] -
                                        baseline.samples[i].q[size_t(j)]));
            }
        }
        std::printf("baseline/loaded max pose delta: %.4f rad\n", max_delta);
    }

    const ids::PayloadOptions opt = make_options(args, gravity);
    const ids::PayloadResult r =
        ids::solve_payload(loaded.samples, baseline_samples, &phi, opt);
    print_result(r);
    write_result_yaml(r, loaded, args.str("out", "ident_result"),
                      timestamp_suffix(), model_source, baseline_path, loaded_path);
    if (args.has("apply")) apply_payload(arm, r);
    std::printf("arm remains enabled and holding\n");
    return 0;
}

int run_real(Arm& arm, const example::Args& args) {
    if (arm.n() != kN) {
        std::printf("requires a 7-joint arm (n=%d)\n", arm.n());
        return 1;
    }
    const std::vector<Vec7> poses = select_poses(args);

    const bool capture_baseline = args.has("capture-baseline");
    const bool use_baseline = args.has("baseline");
    const bool absolute = args.has("absolute");
    const int modes = int(capture_baseline) + int(use_baseline) + int(absolute);
    if (modes > 1) {
        throw InvalidCommandError(
            "choose exactly one of --capture-baseline, --baseline, --absolute");
    }
    if (args.has("baseline-csv")) {
        throw InvalidCommandError("--baseline-csv is offline only; use --baseline");
    }
    if (args.has("loaded-csv")) {
        throw InvalidCommandError("--loaded-csv is offline mode");
    }
    if (args.has("apply") && !(use_baseline || absolute)) {
        throw InvalidCommandError("--apply requires --baseline or --absolute");
    }
    if (args.has("capture-loaded") && !(use_baseline || absolute)) {
        throw InvalidCommandError("--capture-loaded requires a solve mode");
    }

    if (!args.go) {
        std::string model_source;
        const ids::Phi70 phi = load_phi(args, &arm, &model_source);
        return run_plan(arm, poses, args, phi, model_source);
    }
    if (capture_baseline) {
        const ids::Phi70 fw = read_firmware_phi(arm);
        return run_capture_baseline(arm, poses, args, fw);
    }
    if (use_baseline || absolute) {
        return run_identify_live(arm, poses, args, use_baseline);
    }
    throw InvalidCommandError(
        "--go requires --capture-baseline, --baseline, or --absolute");
}

}  // namespace

int main(int argc, char** argv) {
    const std::string desc =
        "11_identify_payload - end-effector mass/COM identification\n"
        "  (wrist-only: J1=J3=0, J2=0.9, J4=-0.8 fixed; fit with J5..J7)\n"
        "  --capture-baseline FILE  capture an empty-arm baseline (requires --go)\n"
        "  --baseline FILE          differential identification (requires --go)\n"
        "  --absolute               absolute identification (requires --go)\n"
        "  --load-params FILE       empty-arm model override (default: read the\n"
        "                           firmware bank; required for offline replay)\n"
        "  --loaded-csv FILE        offline loaded capture (no arm)\n"
        "  --baseline-csv FILE      offline baseline capture\n"
        "  --capture-loaded FILE    write the live loaded capture here\n"
        "  --out DIR                output directory (default ident_result)\n"
        "  --apply                  write the result to the firmware (RAM only)\n"
        "  --pose-count N           first N poses to use, 3..20 (default all 20)\n"
        "  --pose-set LIST          explicit 1-based pose list, e.g. 1,10,20\n"
        "  --samples N              static samples per pose (default 80)\n"
        "  --sample-interval S      sampling interval (default 0.012 s)\n"
        "  --speed S                move speed scale, (0, 0.2] (default 0.10)\n"
        "  --max-pose-error RAD     allowed pose error (default 0.15)\n"
        "  --max-payload-kg KG      safety limit (default 5)\n"
        "  --max-com-radius M       safety limit (default 0.5)\n"
        "  --huber-iterations N     robust iterations (default 8)\n"
        "  --condition-max C        pose-set condition limit (default 1e4)\n"
        "  --bootstrap-mass KG      provisional feedforward payload while moving\n"
        "  --bootstrap-com X,Y,Z    provisional COM while moving\n"
        "  --gravity X,Y,Z          fallback gravity if the firmware read fails\n"
        "  --serial TEXT            metadata robot serial\n"
        "  --go                     actually move/capture\n";

    example::Args parsed;
    if (!example::parse_args(argc, argv, desc, &parsed)) return 0;
    if (parsed.has("loaded-csv")) {
        try {
            return run_offline(parsed);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error: %s\n", e.what());
            return 1;
        }
    }
    return example::run_example(argc, argv, desc, run_real);
}
