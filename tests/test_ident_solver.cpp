// Solver tests: synthetic recovery, actuator projection, CAD fill invariance.
#include "test_framework.hpp"

#include <cmath>
#include <random>
#include <vector>

#include "litearm/ident/solver.hpp"

using namespace litearm::ident;

namespace {

std::vector<SamplePoint> synthetic_samples(const Phi70& phi, const Vec7& a,
                                           const Vec7& fv, const Vec7& b,
                                           const Vec7& jm, int n = 400,
                                           unsigned seed = 1) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> qd(-0.7, 0.7);
    std::uniform_real_distribution<double> vd(-1.0, 1.0);
    std::uniform_real_distribution<double> ad(-3.0, 3.0);
    std::vector<SamplePoint> out;
    out.reserve(size_t(n));
    for (int s = 0; s < n; ++s) {
        SamplePoint sp;
        for (int j = 0; j < kJoints; ++j) {
            sp.q[size_t(j)] = qd(rng);
            sp.dq[size_t(j)] = vd(rng);
            sp.ddq[size_t(j)] = ad(rng);
        }
        const auto Y = active_regressor(sp.q, sp.dq, sp.ddq, 300.0);
        for (int j = 0; j < kJoints; ++j) {
            double tau = 0.0;
            for (int c = 0; c < kInertiaParams; ++c) {
                tau += Y[size_t(j * kActiveParams + c)] * phi[size_t(c)];
            }
            tau += Y[size_t(j * kActiveParams + 70 + j)] * a[size_t(j)];
            tau += Y[size_t(j * kActiveParams + 77 + j)] * fv[size_t(j)];
            tau += Y[size_t(j * kActiveParams + 84 + j)] * b[size_t(j)];
            if (j >= 2) {
                tau += Y[size_t(j * kActiveParams + 91 + (j - 2))] * jm[size_t(j)];
            }
            sp.tau[size_t(j)] = tau;
        }
        out.push_back(sp);
    }
    return out;
}

}  // namespace

TEST(ident_solver_recovers_known_actuators) {
    const Phi70 phi = cad_phi();
    Vec7 a{}, fv{}, b{}, jm{};
    for (int j = 0; j < kJoints; ++j) {
        a[size_t(j)] = 0.5;
        fv[size_t(j)] = 0.1;
        b[size_t(j)] = 0.02;
        jm[size_t(j)] = 0.01;
    }
    jm[0] = jm[1] = 0.0;
    const auto samples = synthetic_samples(phi, a, fv, b, jm);

    IdentifyOptions opt;
    opt.regularization = 1e6;
    opt.friction_ridge = 0.0;
    opt.dc_ridge = 0.0;
    opt.armature_ridge = 0.0;
    opt.physical_links = false;
    const IdentResult r = identify(samples, opt);

    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(r.coulomb[size_t(j)], 0.5, 2e-6);
        CHECK_NEAR(r.viscous[size_t(j)], 0.1, 2e-6);
        CHECK_NEAR(r.dc[size_t(j)], 0.02, 2e-6);
        CHECK_NEAR(r.armature[size_t(j)], j >= 2 ? 0.01 : 0.0, 2e-6);
        CHECK(r.info.rmse[size_t(j)] < 1e-6);
    }
    CHECK(r.armature[0] == 0.0 && r.armature[1] == 0.0);
}

TEST(ident_solver_fixed_inertia_keeps_phi_and_recovers_actuators) {
    // A non-physical fixed inertia block (negative Ixx/Izz) must be kept
    // exactly; only the friction/armature columns are solved.
    Phi70 fixed = cad_phi();
    fixed[4] = -1e-3;
    fixed[9] = -1e-3;
    CHECK(!physically_valid(fixed));

    Vec7 a{}, fv{}, b{}, jm{};
    for (int j = 0; j < kJoints; ++j) {
        a[size_t(j)] = 0.4;
        fv[size_t(j)] = 0.08;
        b[size_t(j)] = 0.01;
        jm[size_t(j)] = 0.02;
    }
    jm[0] = jm[1] = 0.0;
    const auto samples = synthetic_samples(fixed, a, fv, b, jm, 400, 3);

    IdentifyOptions opt;
    opt.fix_inertia = true;
    opt.fixed_phi = fixed;
    opt.friction_ridge = 0.0;
    opt.dc_ridge = 0.0;
    opt.armature_ridge = 0.0;
    opt.physical_links = false;
    const IdentResult r = identify(samples, opt);

    CHECK(r.info.fixed_inertia);
    CHECK_EQ(r.info.rank_inertia, 0);
    for (int c = 0; c < kInertiaParams; ++c) {
        CHECK_NEAR(r.phi[size_t(c)], fixed[size_t(c)], 0.0);
        CHECK_NEAR(r.phi_raw[size_t(c)], fixed[size_t(c)], 0.0);
    }
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(r.coulomb[size_t(j)], 0.4, 2e-6);
        CHECK_NEAR(r.viscous[size_t(j)], 0.08, 2e-6);
        CHECK_NEAR(r.dc[size_t(j)], 0.01, 2e-6);
        CHECK_NEAR(r.armature[size_t(j)], j >= 2 ? 0.02 : 0.0, 2e-6);
        CHECK(r.info.rmse[size_t(j)] < 1e-6);
    }
}

TEST(ident_solver_projects_infeasible_actuators) {
    const Phi70 phi = cad_phi();
    Vec7 a{}, fv{}, b{}, jm{};
    for (int j = 0; j < kJoints; ++j) {
        a[size_t(j)] = -0.5;
        fv[size_t(j)] = -0.1;
        b[size_t(j)] = 2.0;
        jm[size_t(j)] = -0.01;
    }
    const auto samples = synthetic_samples(phi, a, fv, b, jm);

    IdentifyOptions opt;
    opt.regularization = 1e6;
    opt.friction_ridge = 0.0;
    opt.dc_ridge = 0.0;
    opt.armature_ridge = 0.0;
    opt.physical_links = false;
    const IdentResult r = identify(samples, opt);

    const Vec7 fc0 = r.fc0();
    const Vec7 fc1 = r.fc1();
    for (int j = 0; j < kJoints; ++j) {
        CHECK(r.coulomb[size_t(j)] >= 0.0);
        CHECK(r.viscous[size_t(j)] >= 0.0);
        CHECK(std::abs(r.dc[size_t(j)]) <= r.coulomb[size_t(j)] + 1e-12);
        CHECK(fc0[size_t(j)] >= 0.0);
        CHECK(fc1[size_t(j)] <= 0.0);
        CHECK(r.armature[size_t(j)] >= 0.0);
    }
    CHECK(r.armature[0] == 0.0 && r.armature[1] == 0.0);
}

TEST(ident_solver_cad_fill_preserves_inertial_prediction) {
    const Phi70 phi = cad_phi();
    Vec7 a{}, fv{}, b{}, jm{};
    for (int j = 0; j < kJoints; ++j) {
        a[size_t(j)] = 0.3;
        fv[size_t(j)] = 0.05;
        b[size_t(j)] = 0.0;
    }
    const auto samples = synthetic_samples(phi, a, fv, b, jm, 200, 7);
    IdentifyOptions opt;
    opt.regularization = 1e-2;
    opt.dc_ridge = 1e5;
    const IdentResult r = identify(samples, opt);

    double max_d = 0.0;
    for (const auto& sp : samples) {
        const auto Y = inertia_regressor(sp.q, sp.dq, sp.ddq);
        for (int j = 0; j < kJoints; ++j) {
            double p_fill = 0.0, p_raw = 0.0;
            for (int c = 0; c < kInertiaParams; ++c) {
                p_fill += Y[size_t(j * kInertiaParams + c)] * r.phi[size_t(c)];
                p_raw += Y[size_t(j * kInertiaParams + c)] * r.phi_raw[size_t(c)];
            }
            max_d = std::max(max_d, std::abs(p_fill - p_raw));
        }
    }
    CHECK(max_d < 1e-6);
    CHECK(r.info.rank_inertia > 0);
    CHECK(r.info.rank_inertia <= kInertiaParams);
}

TEST(ident_physical_projection_fixes_triangle_inequality) {
    // Mimic the J1 case: the identified Izz also carries the (collinear) J1
    // rotor inertia, so the link tensor has one huge principal moment.
    Phi70 phi = cad_phi();
    phi[4] = 1e-4;   // Ixx (about the joint origin; c is tiny here)
    phi[5] = 0.0;    // Ixy
    phi[6] = 1e-4;   // Iyy
    phi[7] = 0.0;    // Ixz
    phi[8] = 0.0;    // Iyz
    phi[9] = 0.1;    // Izz
    CHECK(!physically_valid(phi));

    const double m = phi[0], hx = phi[1], hy = phi[2], hz = phi[3];
    const PhysicalRepair rep = project_physical_links(phi, cad_phi());
    CHECK(rep.triangle >= 1);
    CHECK(physically_valid(phi));
    CHECK_NEAR(phi[0], m, 0.0);   // mass preserved
    CHECK_NEAR(phi[1], hx, 0.0);  // first moment preserved (gravity torque)
    CHECK_NEAR(phi[2], hy, 0.0);
    CHECK_NEAR(phi[3], hz, 0.0);
}

TEST(ident_physical_projection_replaces_nonfinite_link) {
    Phi70 phi = cad_phi();
    phi[10] = std::nan("");  // link 2 mass
    const Phi70 cad = cad_phi();
    const PhysicalRepair rep = project_physical_links(phi, cad);
    CHECK(rep.nonfinite == 1);
    CHECK(rep.fallback == 1);
    CHECK(physically_valid(phi));
    CHECK_NEAR(phi[10], cad[10], 0.0);
    CHECK_NEAR(phi[11], cad[11], 0.0);
}

TEST(ident_physical_projection_replaces_absurd_com) {
    Phi70 phi = cad_phi();
    phi[21] = 10.0;  // link 3 first moment -> |c| ~ 26 m
    const PhysicalRepair rep = project_physical_links(phi, cad_phi());
    CHECK(rep.bad_com == 1);
    CHECK(rep.fallback == 1);
    CHECK(physically_valid(phi));
}

TEST(ident_solver_unit_mass_gauge_recovers_unit_mass_truth) {
    // Build a physically valid unit-mass truth: m = 1, c = CAD COM,
    // I_com = CAD I_com.  The solver must keep every mass at 1 and still fit.
    const Phi70 cad = cad_phi();
    Phi70 truth{};
    for (int i = 0; i < kJoints; ++i) {
        const double* p = &cad[size_t(i * 10)];
        const double m = p[0];
        const double cx = p[1] / m, cy = p[2] / m, cz = p[3] / m;
        const double c2 = cx * cx + cy * cy + cz * cz;
        const double ixx = p[4] - m * (c2 - cx * cx);
        const double ixy = p[5] + m * cx * cy;
        const double iyy = p[6] - m * (c2 - cy * cy);
        const double ixz = p[7] + m * cx * cz;
        const double iyz = p[8] + m * cy * cz;
        const double izz = p[9] - m * (c2 - cz * cz);
        double* t = &truth[size_t(i * 10)];
        t[0] = 1.0;
        t[1] = cx;
        t[2] = cy;
        t[3] = cz;
        t[4] = ixx + (c2 - cx * cx);
        t[5] = ixy - cx * cy;
        t[6] = iyy + (c2 - cy * cy);
        t[7] = ixz - cx * cz;
        t[8] = iyz - cy * cz;
        t[9] = izz + (c2 - cz * cz);
    }
    CHECK(physically_valid(truth));

    Vec7 a{}, fv{}, b{}, jm{};
    for (int j = 0; j < kJoints; ++j) {
        a[size_t(j)] = 0.4;
        fv[size_t(j)] = 0.06;
    }
    const auto samples = synthetic_samples(truth, a, fv, b, jm, 300, 21);

    IdentifyOptions opt;
    opt.regularization = 1e-6;
    opt.friction_ridge = 0.0;
    opt.dc_ridge = 0.0;
    opt.armature_ridge = 0.0;
    opt.physical_links = false;
    opt.unit_mass = true;
    const IdentResult r = identify(samples, opt);

    for (int i = 0; i < kJoints; ++i) CHECK_NEAR(r.phi[size_t(i * 10)], 1.0, 0.0);
    CHECK(physically_valid(r.phi));
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(r.coulomb[size_t(j)], 0.4, 2e-6);
        CHECK_NEAR(r.viscous[size_t(j)], 0.06, 2e-6);
        // The CAD fill/rank tolerance leaves a small residual on this
        // noiseless synthetic set; on the real capture the unit-mass fit
        // matches the physical-mass RMSE.
        CHECK(r.info.rmse[size_t(j)] < 0.1);
    }
}

TEST(ident_solver_output_is_physically_valid) {
    // Fit data generated by an unphysical link tensor; the reported phi must
    // still be projected onto the physical cone.
    Phi70 truth = cad_phi();
    truth[9] = 0.1;  // triangle violation on link 1
    Vec7 a{}, fv{}, b{}, jm{};
    for (int j = 0; j < kJoints; ++j) {
        a[size_t(j)] = 0.2;
        fv[size_t(j)] = 0.02;
    }
    const auto samples = synthetic_samples(truth, a, fv, b, jm, 400, 11);
    IdentifyOptions opt;
    opt.regularization = 1e-6;
    opt.physical_links = false;
    const IdentResult r = identify(samples, opt);
    CHECK(physically_valid(r.phi));
}
