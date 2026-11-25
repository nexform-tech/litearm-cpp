// End-effector payload identification from static joint-torque samples.
//
// The firmware payload model is a point mass: mass m and COM c in the ee
// flange frame.  Its gravity torque is linear in the first moments
// theta = [m, m*cx, m*cy, m*cz], so a small set of static poses gives a
// linear least-squares problem.  The regressor is built in the Craig/MDH link
// frames of DH/litearm_dh.urdf (see model.hpp), matching the identified
// empty-arm model from example 10.
//
// Two targets are supported:
//   - differential: loaded tau - empty-baseline tau (recommended; cancels
//     empty-model error and same-session static friction),
//   - absolute: loaded tau - rnea(empty_phi) (needs a calibrated model).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "litearm/ident/model.hpp"

namespace litearm {
namespace ident {

/// One static pose: measured joint angles and measured holding torques.
struct PayloadSample {
    Vec7 q{};
    Vec7 tau{};
};

struct PayloadOptions {
    /// Zero-based joint indices.  J1 (index 0) is excluded by default because
    /// its vertical yaw axis sees no gravity moment from a point mass.
    std::vector<int> joints{1, 2, 3, 4, 5, 6};
    double max_payload_kg = 5.0;
    double min_mass_kg = 0.02;
    double max_com_radius_m = 0.50;
    double max_condition = 1.0e4;
    double min_row_norm = 1.0e-7;
    int huber_iterations = 8;
    int min_poses = 2;
    Vec3 gravity{0.0, 0.0, -9.81};
};

struct PayloadResult {
    double mass = 0.0;
    Vec3 com{};             ///< ee flange frame, m
    Vec3 first_moment{};    ///< m*com, kg*m, ee flange frame
    double residual_rms_nm = 0.0;
    double residual_max_nm = 0.0;
    double r_squared = 0.0;
    int rank = 0;
    double condition = 0.0;
    int equations = 0;
    int poses = 0;
    std::vector<int> joints;    ///< zero-based
    std::string method;         ///< differential_static | absolute_static
};

struct PayloadPoseDiagnostics {
    int equations = 0;
    int rank = 0;
    double condition = 0.0;
    double min_column_norm = 0.0;
};

/// Solve payload mass/COM.  `empty_phi` is required: it is the empty-arm model
/// used for absolute mode and for transporting a baseline between poses.
/// With `baseline`, the target is the loaded-minus-empty measured torque at
/// matching poses; without it, the empty model gravity is subtracted.
PayloadResult solve_payload(const std::vector<PayloadSample>& loaded,
                            const std::vector<PayloadSample>* baseline,
                            const Phi70* empty_phi,
                            const PayloadOptions& options = {});

/// Observability check for a pose set.  Never throws for rank/condition; the
/// caller decides whether the reported numbers are acceptable.
PayloadPoseDiagnostics payload_pose_diagnostics(
    const std::vector<Vec7>& q, const PayloadOptions& options = {});

/// Full static capture, including per-pose diagnostics.  Written as a
/// versioned CSV so a live run can be replayed offline.
struct PayloadCapture {
    std::string schema = "litearm.payload_capture.v1";
    std::string kind;           ///< "baseline" | "loaded"
    std::string created_utc;
    std::int64_t created_unix = 0;
    std::string robot_serial = "UNASSIGNED";
    std::string firmware;
    Vec3 gravity{0.0, 0.0, -9.81};
    double bootstrap_mass = 0.0;
    Vec3 bootstrap_com{};
    int samples_per_pose = 0;
    double sample_interval_s = 0.0;
    double speed = 0.0;
    bool complete = false;
    std::vector<Vec7> target_q;
    std::vector<PayloadSample> samples;
    std::vector<Vec7> tau_std;
    std::vector<double> pose_error_rad;
    std::vector<double> max_abs_dq;
};

/// Write/read the capture CSV.  `expected_kind` empty accepts any kind.
/// The reader rejects incomplete captures, schema/kind mismatches, malformed
/// rows, and non-finite values.
void write_payload_capture(const std::string& path,
                           const PayloadCapture& capture);
PayloadCapture load_payload_capture(const std::string& path,
                                    const std::string& expected_kind = "");

}  // namespace ident
}  // namespace litearm
