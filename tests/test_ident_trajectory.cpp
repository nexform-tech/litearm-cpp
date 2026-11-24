// Fourier trajectory evaluation and limit-fitting tests.
#include "test_framework.hpp"

#include <cmath>

#include "litearm/ident/trajectory.hpp"

using namespace litearm::ident;

TEST(ident_fourier_eval_matches_derivatives) {
    FourierCoefficients c;
    c.q0 = {0.1, -0.2, 0.3, -0.4, 0.1, -0.2, 0.3};
    c.period = 2.0;
    c.a.assign(2, {});
    c.b.assign(2, {});
    for (int k = 0; k < 2; ++k) {
        for (int j = 0; j < kJoints; ++j) {
            c.a[size_t(k)][size_t(j)] = 0.01 * double(j + 1) / double(k + 1);
            c.b[size_t(k)][size_t(j)] = 0.005 * double(k + 1) / double(j + 1);
        }
    }
    FourierTrajectory traj(c);
    const double t = 0.37;
    const auto s = traj.eval(t);
    const double h = 1e-6;
    const auto sp = traj.eval(t + h);
    const auto sm = traj.eval(t - h);
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR((sp.q[size_t(j)] - sm.q[size_t(j)]) / (2 * h),
                   s.dq[size_t(j)], 1e-6);
        CHECK_NEAR((sp.dq[size_t(j)] - sm.dq[size_t(j)]) / (2 * h),
                   s.ddq[size_t(j)], 1e-5);
    }
    CHECK_NEAR(traj.period(), 2.0, 1e-15);
}

TEST(ident_fit_fourier_to_limits_respects_envelope) {
    FourierCoefficients c;
    c.q0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    c.period = 8.0;
    c.a.assign(1, {});
    c.b.assign(1, {});
    for (int j = 0; j < kJoints; ++j) c.a[0][size_t(j)] = 2.0;
    FourierTrajectory traj(c);
    const Limits lim = Limits::defaults();
    const LimitFit fit = fit_fourier_to_limits(traj, lim, 0.05, 4000);
    for (int s = 0; s < 4000; ++s) {
        const Vec7 q = fit.trajectory.eval(8.0 * double(s) / 4000.0).q;
        for (int j = 0; j < kJoints; ++j) {
            CHECK(q[size_t(j)] >= lim.q_min[size_t(j)] + 0.05 - 1e-9);
            CHECK(q[size_t(j)] <= lim.q_max[size_t(j)] - 0.05 + 1e-9);
        }
    }
    // J1 fits unchanged; J2/J6 are scaled and the asymmetric J4 is both
    // scaled and moved to the center of its range.
    CHECK_NEAR(fit.center_shift[0], 0.0, 1e-12);
    CHECK_NEAR(fit.scale[0], 1.0, 1e-12);
    CHECK(fit.scale[1] < 1.0);
    CHECK(fit.scale[3] < 1.0);
    CHECK_NEAR(fit.center_shift[3], 0.5 * (lim.q_min[3] + lim.q_max[3]), 1e-9);
}

TEST(ident_fit_fourier_to_limits_shifts_without_scaling) {
    // A small trajectory outside on one side only is moved, not scaled.
    FourierCoefficients c;
    c.q0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    c.period = 8.0;
    c.a.assign(1, {});
    c.b.assign(1, {});
    for (int j = 0; j < kJoints; ++j) c.a[0][size_t(j)] = 0.3;
    FourierTrajectory traj(c);
    const Limits lim = Limits::defaults();
    const LimitFit fit = fit_fourier_to_limits(traj, lim, 0.05, 1000);
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(fit.scale[size_t(j)], 1.0, 1e-12);
    }
    // J4's asymmetric range pulls its center down to the top of the feasible
    // interval; the symmetric joints need no shift.
    CHECK_NEAR(fit.center_shift[3], lim.q_max[3] - 0.05 - 0.3, 1e-9);
    CHECK_NEAR(fit.center_shift[0], 0.0, 1e-12);
}
