// Real-arm Fourier capture: continuous pure-PD streaming + 100 Hz status
// sampling (q/dq/tau).  No firmware-log windows, so long periods never pause.
#pragma once

#include <array>
#include <vector>

#include "litearm/ident/model.hpp"
#include "litearm/ident/trajectory.hpp"

namespace litearm {

class Arm;

namespace ident {

struct CaptureOptions {
    int cycles = 4;
    int send_hz = 300;
    bool use_measured_q = true;
    bool use_measured_dq = true;
    bool analytic_dq = false;
    Vec7 kp{};
    Vec7 kd{};
    bool override_gains = false;
    double max_tracking_error_rad = 0.35;
    double max_send_lateness_s = 0.02;
    int min_valid_samples_per_cycle = 1800;
    bool restore_ff_mask = true;
    double hold_speed = 0.2;
    /// Start phase within the period, in seconds.  A negative value (default)
    /// auto-selects the phase with the smallest max |dq| so the movej -> PD
    /// handoff and the final PD -> movej stop do not step the velocity.
    double start_phase_s = -1.0;
    /// Settle time after movej reaches the start pose, before streaming.
    double settle_s = 0.15;
};

struct CaptureStats {
    int cycles = 0;
    int commands_sent = 0;
    int status_samples = 0;
    int valid_samples = 0;
    int dropped_samples = 0;
    double max_tracking_error_rad = 0.0;
    double max_send_lateness_s = 0.0;
    double start_phase_s = 0.0;
    int ff_mask_before = 0;
};

struct CaptureResult {
    std::vector<SamplePoint> samples;
    CaptureStats stats;
};

/// Capture `cycles` Fourier periods continuously: pure PD tracking, no stops.
/// The caller must keep the arm supported and the emergency stop within reach.
CaptureResult capture_center(Arm& arm, const FourierTrajectory& trajectory,
                             const CaptureOptions& options = {});

}  // namespace ident
}  // namespace litearm
