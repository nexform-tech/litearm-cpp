// Fourier trajectory evaluation and joint-limit fitting.
#include "litearm/ident/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace litearm {
namespace ident {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

}  // namespace

Limits Limits::defaults() {
    Limits l;
    l.q_min = {-2.809547, -1.727547, -2.809547, -3.071547, -2.809547, -1.553547, -1.553547};
    l.q_max = {2.809547, 1.727547, 2.809547, 0.017547, 2.809547, 1.553547, 1.553547};
    l.vel_max = {2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0};
    l.acc_max = {8.0, 8.0, 7.0, 7.0, 9.0, 9.0, 9.0};
    l.tau_max = {78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0};
    l.jerk_max = {300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0};
    return l;
}

FourierTrajectory::FourierTrajectory(FourierCoefficients coefficients)
    : c_(std::move(coefficients)) {
    if (c_.a.size() != c_.b.size() || c_.a.empty() || c_.period <= 0.0) {
        throw std::invalid_argument("FourierCoefficients: a/b 长度或 period 非法");
    }
}

FourierTrajectory::State FourierTrajectory::eval(double t) const {
    const int K = int(c_.a.size());
    const double omega = kTwoPi / c_.period;
    State out;
    out.q = c_.q0;
    for (int k = 0; k < K; ++k) {
        const double kw = double(k + 1) * omega;
        const double s = std::sin(kw * t);
        const double c = std::cos(kw * t);
        for (int j = 0; j < kJoints; ++j) {
            out.q[size_t(j)] += c_.a[size_t(k)][size_t(j)] * s +
                                c_.b[size_t(k)][size_t(j)] * c;
            out.dq[size_t(j)] += c_.a[size_t(k)][size_t(j)] * (kw * c) -
                                 c_.b[size_t(k)][size_t(j)] * (kw * s);
            out.ddq[size_t(j)] -=
                c_.a[size_t(k)][size_t(j)] * (kw * kw * s) +
                c_.b[size_t(k)][size_t(j)] * (kw * kw * c);
        }
    }
    return out;
}

Vec7 FourierTrajectory::peak_excursion() const {
    Vec7 out{};
    for (size_t k = 0; k < c_.a.size(); ++k) {
        for (int j = 0; j < kJoints; ++j) {
            out[size_t(j)] += std::hypot(c_.a[k][size_t(j)], c_.b[k][size_t(j)]);
        }
    }
    return out;
}

LimitFit fit_fourier_to_limits(const FourierTrajectory& trajectory,
                               const Limits& limits, double position_margin,
                               int check_samples) {
    if (check_samples < 2) check_samples = 2;
    FourierCoefficients c = trajectory.coefficients();
    const int K = int(c.a.size());
    const double period = c.period;
    Vec7 shift{};
    Vec7 scale{};
    shift.fill(0.0);
    scale.fill(1.0);
    for (int j = 0; j < kJoints; ++j) {
        double mn = 1e300;
        double mx = -1e300;
        for (int s = 0; s < check_samples; ++s) {
            const double q = trajectory
                                 .eval(period * double(s) / double(check_samples))
                                 .q[size_t(j)];
            mn = std::min(mn, q);
            mx = std::max(mx, q);
        }
        const double lo = limits.q_min[size_t(j)] + position_margin;
        const double hi = limits.q_max[size_t(j)] - position_margin;
        if (hi < lo) {
            throw std::runtime_error(
                "fit_fourier_to_limits: joint limit is narrower than twice the "
                "position margin");
        }
        const double q0 = c.q0[size_t(j)];
        const double d_hi = std::max(mx - q0, 0.0);
        const double d_lo = std::max(q0 - mn, 0.0);
        double alpha = 1.0;
        if (d_hi + d_lo > 0.0) {
            alpha = std::min(1.0, (hi - lo) / (d_hi + d_lo));
        }
        const double a_lo = lo + alpha * d_lo;
        const double a_hi = hi - alpha * d_hi;
        const double a0 =
            a_lo <= a_hi ? std::min(std::max(q0, a_lo), a_hi) : 0.5 * (a_lo + a_hi);
        for (int k = 0; k < K; ++k) {
            c.a[size_t(k)][size_t(j)] *= alpha;
            c.b[size_t(k)][size_t(j)] *= alpha;
        }
        shift[size_t(j)] = a0 - q0;
        scale[size_t(j)] = alpha;
        c.q0[size_t(j)] = a0;
    }
    return {FourierTrajectory(std::move(c)), shift, scale};
}

double ee_z_min(const FourierTrajectory& trajectory, int samples) {
    double out = 1e300;
    const double period = trajectory.period();
    for (int s = 0; s < samples; ++s) {
        const double t = period * double(s) / double(samples);
        out = std::min(out, fk_ee(trajectory.eval(t).q).p_ee[2]);
    }
    return out;
}

double sigma_min_of_trajectory(const FourierTrajectory& trajectory, int samples) {
    double out = 1e300;
    const double period = trajectory.period();
    for (int s = 0; s < samples; ++s) {
        const double t = period * double(s) / double(samples);
        out = std::min(out, sigma_min(trajectory.eval(t).q));
    }
    return out;
}

}  // namespace ident
}  // namespace litearm
