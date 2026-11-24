// Payload identification tests (offline, synthetic data, no arm).
#include "test_framework.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "litearm/ident/model.hpp"
#include "litearm/ident/payload.hpp"
#include "litearm/rot.hpp"

#include "../src/dh_chain.hpp"

using namespace litearm::ident;

namespace {

constexpr Vec3 kG{0.0, 0.0, -9.81};
constexpr Vec7 kQ{0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7};

const std::array<Vec7, 10> kPoses{{
    {0.0, 0.6, 0.0, -1.0, 0.0, 0.5, 0.0},
    {0.0, 1.0, 0.0, -1.2, 0.0, 0.6, 0.0},
    {0.0, 1.3, 0.0, -0.8, 0.0, 0.4, 0.0},
    {0.0, 0.8, 0.5, -1.4, 0.3, 0.5, 0.2},
    {0.0, 0.8, -0.5, -1.4, -0.3, 0.5, -0.2},
    {0.5, 1.0, 0.3, -1.0, 0.5, 0.7, 0.3},
    {-0.5, 1.0, -0.3, -1.0, -0.5, 0.7, -0.3},
    {0.0, 0.5, 0.8, -1.6, 0.6, 0.8, 0.4},
    {0.0, 1.2, 0.0, -0.5, 1.0, 0.3, -0.5},
    {0.0, 0.9, 0.4, -1.1, -0.8, 1.0, 0.6},
}};

Vec7 apply_regressor(const Vec7& q, const Vec7& theta, const Vec3& g) {
    const auto Y = payload_gravity_regressor(q, g);
    Vec7 out{};
    for (int j = 0; j < kJoints; ++j) {
        for (int k = 0; k < kPayloadParams; ++k) {
            out[size_t(j)] += Y[size_t(j * kPayloadParams + k)] * theta[size_t(k)];
        }
    }
    return out;
}

Vec7 theta_of(double m, const Vec3& c) {
    return {m, m * c[0], m * c[1], m * c[2]};
}

Vec7 add(const Vec7& a, const Vec7& b) {
    Vec7 out{};
    for (int j = 0; j < kJoints; ++j) out[size_t(j)] = a[size_t(j)] + b[size_t(j)];
    return out;
}

Vec7 subtract(const Vec7& a, const Vec7& b) {
    Vec7 out{};
    for (int j = 0; j < kJoints; ++j) out[size_t(j)] = a[size_t(j)] - b[size_t(j)];
    return out;
}

Vec7 constant_vec7(double v) {
    Vec7 out{};
    out.fill(v);
    return out;
}

std::vector<PayloadSample> make_loaded(const Vec7& theta, const Vec7& bias,
                                       const std::vector<Vec7>& poses) {
    const Phi70 phi = cad_phi();
    const Vec7 zero{};
    std::vector<PayloadSample> out;
    for (const auto& q : poses) {
        PayloadSample s;
        s.q = q;
        s.tau = add(add(rnea(q, zero, zero, phi, kG), bias),
                    apply_regressor(q, theta, kG));
        out.push_back(s);
    }
    return out;
}

std::vector<Vec7> default_pose_list() {
    std::vector<Vec7> q(kPoses.begin(), kPoses.end());
    return q;
}

struct TempFile {
    std::filesystem::path path;
    explicit TempFile(const char* name)
        : path(std::filesystem::temp_directory_path() / name) {}
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

PayloadCapture sample_capture(const char* kind) {
    PayloadCapture c;
    c.kind = kind;
    c.robot_serial = "TEST";
    c.firmware = "Litearm1.5.0-7J";
    c.samples_per_pose = 2;
    c.sample_interval_s = 0.01;
    c.speed = 0.1;
    for (int i = 0; i < 3; ++i) {
        PayloadSample s;
        for (int j = 0; j < kJoints; ++j) {
            s.q[size_t(j)] = 0.1 * double(i + 1) + 0.01 * double(j);
            s.tau[size_t(j)] = 0.5 * double(i + 1) - 0.02 * double(j);
        }
        Vec7 target = s.q;
        for (int j = 0; j < kJoints; ++j) target[size_t(j)] += 0.001;
        Vec7 std{};
        for (int j = 0; j < kJoints; ++j) std[size_t(j)] = 0.001 * double(j);
        c.samples.push_back(s);
        c.target_q.push_back(target);
        c.tau_std.push_back(std);
        c.pose_error_rad.push_back(0.002);
        c.max_abs_dq.push_back(0.01);
    }
    c.complete = true;
    return c;
}

}  // namespace

TEST(ident_payload_regressor_matches_point_jacobian) {
    const FkResult fk = fk_ee(kQ);
    const Vec3 c{0.03, -0.02, 0.09};
    const double m = 1.7;
    // Payload point in the base frame.
    const double rc[3] = {
        fk.R_ee[0] * c[0] + fk.R_ee[1] * c[1] + fk.R_ee[2] * c[2],
        fk.R_ee[3] * c[0] + fk.R_ee[4] * c[1] + fk.R_ee[5] * c[2],
        fk.R_ee[6] * c[0] + fk.R_ee[7] * c[1] + fk.R_ee[8] * c[2],
    };
    Vec7 expected{};
    for (int j = 0; j < kJoints; ++j) {
        const double jv[3] = {fk.J[size_t(0 * kJoints + j)],
                              fk.J[size_t(1 * kJoints + j)],
                              fk.J[size_t(2 * kJoints + j)]};
        const double jw[3] = {fk.J[size_t(3 * kJoints + j)],
                              fk.J[size_t(4 * kJoints + j)],
                              fk.J[size_t(5 * kJoints + j)]};
        const double jp[3] = {jv[0] + jw[1] * rc[2] - jw[2] * rc[1],
                              jv[1] + jw[2] * rc[0] - jw[0] * rc[2],
                              jv[2] + jw[0] * rc[1] - jw[1] * rc[0]};
        expected[size_t(j)] = -m * (jp[0] * kG[0] + jp[1] * kG[1] + jp[2] * kG[2]);
    }
    const Vec7 got = apply_regressor(kQ, theta_of(m, c), kG);
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(got[size_t(j)], expected[size_t(j)], 1e-12);
    }
}

TEST(ident_payload_regressor_matches_rnea_delta) {
    // Independent check of sign and EE-frame convention: add a point mass as
    // an extra body on link 7 and compare RNEA(with payload) - RNEA(empty).
    const Vec3 c{0.03, -0.02, 0.09};
    const double m = 1.7;
    const auto& ee = litearm::ident_detail::kDhEe;
    const litearm::rot::Mat3 Re = litearm::rot::rpy_to_mat(ee.rpy);
    const litearm::rot::Vec3 cvec{c[0], c[1], c[2]};
    const litearm::rot::Vec3 tc = litearm::rot::mat_vec(Re, cvec);
    const Vec3 t{ee.xyz[0] + tc[0], ee.xyz[1] + tc[1], ee.xyz[2] + tc[2]};
    const double t2 = t[0] * t[0] + t[1] * t[1] + t[2] * t[2];

    Phi70 phi = cad_phi();
    phi[60] += m;
    phi[61] += m * t[0];
    phi[62] += m * t[1];
    phi[63] += m * t[2];
    phi[64] += m * (t2 - t[0] * t[0]);
    phi[65] += -m * t[0] * t[1];
    phi[66] += m * (t2 - t[1] * t[1]);
    phi[67] += -m * t[0] * t[2];
    phi[68] += -m * t[1] * t[2];
    phi[69] += m * (t2 - t[2] * t[2]);

    const Phi70 empty = cad_phi();
    const Vec7 zero{};
    const Vec7 theta = theta_of(m, c);
    for (const auto& q : kPoses) {
        const Vec7 with = rnea(q, zero, zero, phi, kG);
        const Vec7 without = rnea(q, zero, zero, empty, kG);
        const Vec7 got = apply_regressor(q, theta, kG);
        for (int j = 0; j < kJoints; ++j) {
            CHECK_NEAR(with[size_t(j)] - without[size_t(j)], got[size_t(j)], 1e-10);
        }
    }
}

TEST(ident_payload_regressor_j1_zero_for_vertical_gravity) {
    for (const auto& q : kPoses) {
        const auto Y = payload_gravity_regressor(q, kG);
        for (int k = 0; k < kPayloadParams; ++k) {
            CHECK_NEAR(Y[size_t(k)], 0.0, 1e-12);
        }
    }
}

TEST(ident_payload_regressor_tilted_gravity) {
    const Vec3 g{0.4, -0.3, -9.79};
    const FkResult fk = fk_ee(kQ);
    const Vec3 c{0.05, 0.02, 0.07};
    const double m = 2.1;
    const double rc[3] = {
        fk.R_ee[0] * c[0] + fk.R_ee[1] * c[1] + fk.R_ee[2] * c[2],
        fk.R_ee[3] * c[0] + fk.R_ee[4] * c[1] + fk.R_ee[5] * c[2],
        fk.R_ee[6] * c[0] + fk.R_ee[7] * c[1] + fk.R_ee[8] * c[2],
    };
    Vec7 expected{};
    for (int j = 0; j < kJoints; ++j) {
        const double jv[3] = {fk.J[size_t(0 * kJoints + j)],
                              fk.J[size_t(1 * kJoints + j)],
                              fk.J[size_t(2 * kJoints + j)]};
        const double jw[3] = {fk.J[size_t(3 * kJoints + j)],
                              fk.J[size_t(4 * kJoints + j)],
                              fk.J[size_t(5 * kJoints + j)]};
        const double jp[3] = {jv[0] + jw[1] * rc[2] - jw[2] * rc[1],
                              jv[1] + jw[2] * rc[0] - jw[0] * rc[2],
                              jv[2] + jw[0] * rc[1] - jw[1] * rc[0]};
        expected[size_t(j)] = -m * (jp[0] * g[0] + jp[1] * g[1] + jp[2] * g[2]);
    }
    const Vec7 got = apply_regressor(kQ, theta_of(m, c), g);
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(got[size_t(j)], expected[size_t(j)], 1e-12);
    }
}

TEST(ident_payload_absolute_recovers_exact) {
    const Vec3 c{0.03, -0.02, 0.09};
    const Vec7 theta = theta_of(1.75, c);
    const auto loaded = make_loaded(theta, constant_vec7(0.0), default_pose_list());
    const Phi70 phi = cad_phi();
    const PayloadResult r = solve_payload(loaded, nullptr, &phi);
    CHECK_NEAR(r.mass, 1.75, 1e-9);
    CHECK_NEAR(r.com[0], c[0], 1e-9);
    CHECK_NEAR(r.com[1], c[1], 1e-9);
    CHECK_NEAR(r.com[2], c[2], 1e-9);
    CHECK_EQ(r.rank, 4);
    CHECK_EQ(r.equations, 60);
    CHECK_EQ(r.poses, 10);
    CHECK(r.method == "absolute_static");
    CHECK(r.residual_rms_nm < 1e-9);
    CHECK_NEAR(r.r_squared, 1.0, 1e-9);
}

TEST(ident_payload_differential_cancels_constant_bias) {
    const Vec3 c{-0.04, 0.06, 0.12};
    const Vec7 theta = theta_of(2.3, c);
    const Vec7 bias{1.2, -0.8, 0.4, 0.9, -0.3, 0.2, -0.5};
    const auto poses = default_pose_list();
    const Phi70 phi = cad_phi();
    const Vec7 zero{};

    std::vector<PayloadSample> baseline;
    for (const auto& q : poses) {
        baseline.push_back({q, add(rnea(q, zero, zero, phi, kG), bias)});
    }
    std::vector<PayloadSample> loaded;
    for (size_t i = 0; i < poses.size(); ++i) {
        loaded.push_back({poses[i],
                          add(baseline[i].tau, apply_regressor(poses[i], theta, kG))});
    }
    const PayloadResult r = solve_payload(loaded, &baseline, &phi);
    CHECK_NEAR(r.mass, 2.3, 1e-9);
    CHECK_NEAR(r.com[0], c[0], 1e-9);
    CHECK_NEAR(r.com[1], c[1], 1e-9);
    CHECK_NEAR(r.com[2], c[2], 1e-9);
    CHECK(r.method == "differential_static");
}

TEST(ident_payload_transports_baseline_for_pose_mismatch) {
    const Vec3 c{0.02, -0.05, 0.08};
    const Vec7 theta = theta_of(1.4, c);
    const Vec7 bias{0.3, -0.2, 0.1, -0.1, 0.05, 0.02, -0.01};
    const Phi70 phi = cad_phi();
    const Vec7 zero{};

    std::vector<PayloadSample> baseline;
    std::vector<PayloadSample> loaded;
    const auto poses = default_pose_list();
    for (size_t i = 0; i < poses.size(); ++i) {
        const Vec7 qe = poses[i];
        Vec7 ql = qe;
        for (int j = 0; j < kJoints; ++j) {
            ql[size_t(j)] += 0.01 * double((j + 1) % 3 - 1) + 0.005 * double(i % 2);
        }
        baseline.push_back({qe, add(rnea(qe, zero, zero, phi, kG), bias)});
        loaded.push_back(
            {ql, add(add(rnea(ql, zero, zero, phi, kG), bias),
                     apply_regressor(ql, theta, kG))});
    }
    const PayloadResult good = solve_payload(loaded, &baseline, &phi);
    CHECK_NEAR(good.mass, 1.4, 1e-9);
    CHECK_NEAR(good.com[0], c[0], 1e-9);
    CHECK_NEAR(good.com[1], c[1], 1e-9);
    CHECK_NEAR(good.com[2], c[2], 1e-9);

    // Pretending the baseline was taken at the loaded q removes the transport
    // term and leaves the empty-model pose difference as a payload signal.
    std::vector<PayloadSample> wrong = baseline;
    for (size_t i = 0; i < wrong.size(); ++i) wrong[i].q = loaded[i].q;
    const PayloadResult bad = solve_payload(loaded, &wrong, &phi);
    CHECK(std::abs(bad.mass - 1.4) > 1e-3);
}

TEST(ident_payload_rejects_rank_deficient_poses) {
    const Vec3 c{0.03, -0.02, 0.09};
    const Vec7 theta = theta_of(1.5, c);
    std::vector<PayloadSample> loaded;
    const Phi70 phi = cad_phi();
    const Vec7 zero{};
    for (int i = 0; i < 10; ++i) {
        const Vec7& q = kPoses[0];
        loaded.push_back({q, add(rnea(q, zero, zero, phi, kG),
                                 apply_regressor(q, theta, kG))});
    }
    CHECK_THROWS_AS(solve_payload(loaded, nullptr, &phi), std::runtime_error);
    std::vector<Vec7> poses(10, kPoses[0]);
    const PayloadPoseDiagnostics d = payload_pose_diagnostics(poses);
    CHECK_EQ(d.rank, 3);
}

TEST(ident_payload_rejects_condition_limit) {
    const std::vector<Vec7> poses = default_pose_list();
    const PayloadPoseDiagnostics d = payload_pose_diagnostics(poses);
    CHECK_EQ(d.rank, 4);
    CHECK(d.condition > 2.0);
    PayloadOptions opt;
    opt.max_condition = 2.0;
    const Vec7 theta = theta_of(1.2, {0.01, 0.02, 0.03});
    const auto loaded = make_loaded(theta, constant_vec7(0.0), poses);
    const Phi70 phi = cad_phi();
    CHECK_THROWS_AS(solve_payload(loaded, nullptr, &phi, opt), std::runtime_error);
}

TEST(ident_payload_rejects_bad_mass_and_com) {
    const Phi70 phi = cad_phi();
    const auto poses = default_pose_list();
    {
        const auto loaded =
            make_loaded(theta_of(0.005, {0.0, 0.0, 0.1}), constant_vec7(0.0), poses);
        CHECK_THROWS_AS(solve_payload(loaded, nullptr, &phi), std::runtime_error);
    }
    {
        const auto loaded =
            make_loaded(theta_of(6.0, {0.0, 0.0, 0.1}), constant_vec7(0.0), poses);
        CHECK_THROWS_AS(solve_payload(loaded, nullptr, &phi), std::runtime_error);
    }
    {
        const auto loaded =
            make_loaded(theta_of(1.0, {0.6, 0.0, 0.0}), constant_vec7(0.0), poses);
        CHECK_THROWS_AS(solve_payload(loaded, nullptr, &phi), std::runtime_error);
    }
}

TEST(ident_payload_rejects_bad_inputs) {
    const Phi70 phi = cad_phi();
    const auto loaded = make_loaded(theta_of(1.0, {0.0, 0.0, 0.1}),
                                    constant_vec7(0.0), default_pose_list());
    CHECK_THROWS_AS(solve_payload({}, nullptr, &phi), std::invalid_argument);
    CHECK_THROWS_AS(solve_payload(loaded, nullptr, nullptr),
                    std::invalid_argument);
    {
        std::vector<PayloadSample> short_baseline(loaded.size() - 1);
        for (size_t i = 0; i < short_baseline.size(); ++i) short_baseline[i] = loaded[i];
        CHECK_THROWS_AS(solve_payload(loaded, &short_baseline, &phi),
                        std::invalid_argument);
    }
    {
        PayloadOptions opt;
        opt.joints = {1, 1};
        CHECK_THROWS_AS(solve_payload(loaded, nullptr, &phi, opt),
                        std::invalid_argument);
    }
    {
        PayloadOptions opt;
        opt.joints = {0, 7};
        CHECK_THROWS_AS(solve_payload(loaded, nullptr, &phi, opt),
                        std::invalid_argument);
    }
    {
        auto bad = loaded;
        bad[0].q[0] = std::nan("");
        CHECK_THROWS_AS(solve_payload(bad, nullptr, &phi), std::invalid_argument);
    }
    {
        auto bad = loaded;
        bad[0].tau[0] = std::nan("");
        CHECK_THROWS_AS(solve_payload(bad, nullptr, &phi), std::invalid_argument);
    }
}

TEST(ident_payload_huber_robust_to_outlier) {
    const Vec3 c{0.02, -0.03, 0.06};
    const Vec7 theta = theta_of(1.8, c);
    const auto poses = default_pose_list();
    const Phi70 phi = cad_phi();
    const Vec7 zero{};

    std::vector<PayloadSample> baseline;
    std::vector<PayloadSample> loaded;
    for (size_t i = 0; i < poses.size(); ++i) {
        const Vec7& q = poses[i];
        baseline.push_back({q, rnea(q, zero, zero, phi, kG)});
        Vec7 noise{};
        for (int j = 0; j < kJoints; ++j) {
            noise[size_t(j)] = 0.002 * std::sin(0.7 * double(i + 1) + double(j));
        }
        Vec7 tau = add(add(rnea(q, zero, zero, phi, kG),
                           apply_regressor(q, theta, kG)),
                       noise);
        if (i == 4) tau[2] += 0.35;   // single outlier
        loaded.push_back({q, tau});
    }
    const PayloadResult r = solve_payload(loaded, &baseline, &phi);
    CHECK_NEAR(r.mass, 1.8, 0.02);
    CHECK_NEAR(r.com[0], c[0], 0.003);
    CHECK_NEAR(r.com[1], c[1], 0.003);
    CHECK_NEAR(r.com[2], c[2], 0.003);
}

TEST(ident_payload_capture_csv_roundtrip) {
    TempFile file("litearm_payload_capture_roundtrip.csv");
    const PayloadCapture c = sample_capture("baseline");
    write_payload_capture(file.path.string(), c);
    const PayloadCapture got = load_payload_capture(file.path.string(), "baseline");
    CHECK(got.schema == c.schema);
    CHECK(got.kind == c.kind);
    CHECK(got.complete);
    CHECK_EQ(int(got.samples.size()), int(c.samples.size()));
    CHECK_EQ(got.samples_per_pose, c.samples_per_pose);
    CHECK_NEAR(got.gravity[2], -9.81, 1e-12);
    for (size_t i = 0; i < c.samples.size(); ++i) {
        for (int j = 0; j < kJoints; ++j) {
            CHECK_NEAR(got.samples[i].q[size_t(j)], c.samples[i].q[size_t(j)], 1e-15);
            CHECK_NEAR(got.samples[i].tau[size_t(j)], c.samples[i].tau[size_t(j)], 1e-15);
            CHECK_NEAR(got.target_q[i][size_t(j)], c.target_q[i][size_t(j)], 1e-15);
            CHECK_NEAR(got.tau_std[i][size_t(j)], c.tau_std[i][size_t(j)], 1e-15);
        }
        CHECK_NEAR(got.pose_error_rad[i], c.pose_error_rad[i], 1e-15);
        CHECK_NEAR(got.max_abs_dq[i], c.max_abs_dq[i], 1e-15);
    }
    CHECK_THROWS_AS(load_payload_capture(file.path.string(), "loaded"),
                    std::invalid_argument);

    TempFile partial("litearm_payload_capture_partial.csv");
    PayloadCapture p = c;
    p.complete = false;
    write_payload_capture(partial.path.string(), p);
    CHECK_THROWS_AS(load_payload_capture(partial.path.string(), "baseline"),
                    std::invalid_argument);
}

TEST(ident_payload_capture_csv_rejects_wrong_schema) {
    TempFile file("litearm_payload_capture_schema.csv");
    {
        std::ofstream f(file.path.string());
        f << "# schema: something.else\n# kind: baseline\n# complete: 1\n";
        f << "pose,q0,q1,q2,q3,q4,q5,q6,target_q0,target_q1,target_q2,"
             "target_q3,target_q4,target_q5,target_q6,tau0,tau1,tau2,tau3,tau4,"
             "tau5,tau6,tau_std0,tau_std1,tau_std2,tau_std3,tau_std4,tau_std5,"
             "tau_std6,pose_error_rad,max_abs_dq\n";
    }
    CHECK_THROWS_AS(load_payload_capture(file.path.string(), "baseline"),
                    std::invalid_argument);
}
