// Host dynamics model / regressor tests (offline, no arm).
#include "test_framework.hpp"

#include <cmath>

#include "litearm/ident/model.hpp"

using namespace litearm::ident;

namespace {
constexpr Vec7 kQ{0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7};
constexpr Vec7 kDq{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7};
constexpr Vec7 kDdq{0.5, -0.4, 0.3, -0.2, 0.1, 0.0, -0.1};
}  // namespace

TEST(ident_cad_phi_matches_dh_urdf) {
    const Phi70 phi = cad_phi();
    CHECK_NEAR(phi[0], 0.636061100335262, 1e-12);
    CHECK_NEAR(phi[10], 0.296281105636482, 1e-12);
    // Link 7 includes the fixed ee_link (0.152315 + 0.0284157).
    CHECK_NEAR(phi[60], 0.180730892021824, 1e-12);
    // Link 1 in the Craig/MDH link frame of DH/litearm_dh.urdf: m*cy and Ixx
    // about the joint origin.  The vendor URDF frame would give
    // phi[2] = -9.1665780685427515e-05 and phi[4] = 0.001653364700813207.
    CHECK_NEAR(phi[2], 0.00012292162233737059, 1e-15);
    CHECK_NEAR(phi[4], 0.00066316600787191577, 1e-15);
    CHECK_NEAR(phi[9], 0.00040945097224849201, 1e-15);
}

TEST(ident_regressor_matches_direct_rnea) {
    const Phi70 phi = cad_phi();
    const auto Y = inertia_regressor(kQ, kDq, kDdq);
    const Vec7 tau = rnea(kQ, kDq, kDdq, phi);
    double max_err = 0.0;
    for (int j = 0; j < kJoints; ++j) {
        double pred = 0.0;
        for (int c = 0; c < kInertiaParams; ++c) {
            pred += Y[size_t(j * kInertiaParams + c)] * phi[size_t(c)];
        }
        max_err = std::max(max_err, std::abs(pred - tau[size_t(j)]));
    }
    CHECK(max_err < 1e-10);
    // Golden RNEA on the Craig/MDH chain from DH/litearm_dh.urdf.
    CHECK_NEAR(tau[0], 0.03839066389385002, 1e-10);
    CHECK_NEAR(tau[1], -2.1405551247437202, 1e-10);
    CHECK_NEAR(tau[6], -0.010998098315063836, 1e-10);
    // Same physical sample as the Python/Pinocchio vendor-URDF model.  Torques
    // are frame invariant; the ~1e-5 Nm residual is the vendor URDF's rounded
    // rpy values, not a frame-convention error.
    CHECK_NEAR(tau[0], 0.038390904919973864, 2e-5);
    CHECK_NEAR(tau[1], -2.1405692543652588, 2e-5);
    CHECK_NEAR(tau[6], -0.010997266204398859, 2e-5);
}

TEST(ident_active_regressor_layout) {
    const auto Y = active_regressor(kQ, kDq, kDdq, 300.0);
    const auto Yi = inertia_regressor(kQ, kDq, kDdq);
    for (int j = 0; j < kJoints; ++j) {
        for (int c = 0; c < kInertiaParams; ++c) {
            CHECK_NEAR(Y[size_t(j * kActiveParams + c)],
                       Yi[size_t(j * kInertiaParams + c)], 1e-15);
        }
        const double sc = smooth_coulomb(kDq[size_t(j)], 300.0);
        CHECK_NEAR(Y[size_t(j * kActiveParams + 70 + j)], sc, 1e-15);
        CHECK_NEAR(Y[size_t(j * kActiveParams + 77 + j)], kDq[size_t(j)], 1e-15);
        CHECK_NEAR(Y[size_t(j * kActiveParams + 84 + j)], std::abs(sc), 1e-15);
        if (j >= 2) {
            CHECK_NEAR(Y[size_t(j * kActiveParams + 91 + (j - 2))],
                       kDdq[size_t(j)], 1e-15);
        }
    }
    // jm1/jm2 columns are absent; columns 70/77/84 for row 0 are the only
    // actuator entries for joint 1.
    CHECK_NEAR(Y[size_t(70 + 1)], 0.0, 1e-15);
    CHECK_NEAR(Y[size_t(77 + 1)], 0.0, 1e-15);
}

TEST(ident_fk_and_sigma_match_dh_chain) {
    const FkResult fk = fk_ee(kQ);
    // Golden FK and Jacobian singular value on the Craig/MDH chain.
    CHECK_NEAR(fk.p_ee[0], -0.034683178600637896, 1e-10);
    CHECK_NEAR(fk.p_ee[1], 0.17896252228462173, 1e-10);
    CHECK_NEAR(fk.p_ee[2], 0.7447914672874163, 1e-10);
    CHECK_NEAR(sigma_min(kQ), 0.012029986195160188, 1e-10);
    // The same physical endpoint as the vendor URDF; the ~2e-6 m residual is
    // the rounded rpy values in the URDF text.
    CHECK_NEAR(fk.p_ee[0], -0.034681126345566249, 1e-5);
    CHECK_NEAR(fk.p_ee[1], 0.1789628522679225, 1e-5);
    CHECK_NEAR(fk.p_ee[2], 0.74479108742168143, 1e-5);
    CHECK_NEAR(sigma_min(kQ), 0.012029683311135474, 1e-5);
}
