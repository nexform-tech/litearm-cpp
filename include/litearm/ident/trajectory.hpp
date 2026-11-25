// Fourier trajectory evaluation and joint-limit fitting for the LiteArm 7-DOF arm.
#pragma once

#include <array>
#include <vector>

#include "litearm/ident/model.hpp"

namespace litearm {
namespace ident {

struct Limits {
    Vec7 q_min{};
    Vec7 q_max{};
    Vec7 vel_max{};
    Vec7 acc_max{};
    Vec7 tau_max{};
    Vec7 jerk_max{};

    /// Python SDK defaults (q limits are fallback only; the caller should
    /// overwrite them with the firmware's JointParam values).
    static Limits defaults();
};

struct FourierCoefficients {
    Vec7 q0{};
    std::vector<std::array<double, kJoints>> a;  // [harmonic][joint]
    std::vector<std::array<double, kJoints>> b;
    double period = 24.0;
};

class FourierTrajectory {
public:
    struct State {
        Vec7 q{};
        Vec7 dq{};
        Vec7 ddq{};
    };

    FourierTrajectory() = default;
    explicit FourierTrajectory(FourierCoefficients coefficients);

    State eval(double t) const;
    const Vec7& q0() const noexcept { return c_.q0; }
    double period() const noexcept { return c_.period; }
    int harmonics() const noexcept { return int(c_.a.size()); }
    Vec7 peak_excursion() const;
    const FourierCoefficients& coefficients() const noexcept { return c_; }

private:
    FourierCoefficients c_;
};

/// Result of fitting an existing trajectory into the joint limits.
struct LimitFit {
    FourierTrajectory trajectory;
    Vec7 center_shift{};  ///< applied a0 shift per joint (rad)
    Vec7 scale{};         ///< applied harmonic scale per joint (1 = unchanged)
};

/// Fit an existing trajectory into the limits without changing its shape:
/// per joint, move the center only as far as needed and scale every harmonic
/// by one common factor if the excursion is still wider than the allowed
/// range.  The excursion is evaluated densely (`check_samples` per period),
/// and `position_margin` is kept from both limits.  Throws std::runtime_error
/// when a joint limit is narrower than twice the margin.
LimitFit fit_fourier_to_limits(const FourierTrajectory& trajectory,
                               const Limits& limits,
                               double position_margin = 0.05,
                               int check_samples = 2000);

/// Minimum ee z over one period.
double ee_z_min(const FourierTrajectory& trajectory, int samples = 400);
/// Minimum 6x7 Jacobian singular value over one period.
double sigma_min_of_trajectory(const FourierTrajectory& trajectory, int samples = 400);

}  // namespace ident
}  // namespace litearm
