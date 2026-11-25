// YAML/SN export tests (no arm).
#include "test_framework.hpp"

#include <cmath>
#include <string>

#include "litearm/errors.hpp"
#include "litearm/ident/export.hpp"
#include "litearm/ident/trajectory.hpp"

using namespace litearm;
using namespace litearm::ident;

namespace {

IdentResult sample_result() {
    IdentResult r;
    r.phi = cad_phi();
    for (int j = 0; j < kJoints; ++j) {
        r.coulomb[size_t(j)] = 0.5;
        r.viscous[size_t(j)] = 0.1;
        r.dc[size_t(j)] = 0.02;
        r.armature[size_t(j)] = j >= 2 ? 0.01 : 0.0;
    }
    r.info.rank = 69;
    r.info.rank_fit = 69;
    r.info.rank_inertia = 43;
    r.info.n_samples = 100;
    r.info.n_rows = 700;
    return r;
}

std::string minimal_sn_template() {
    std::string s;
    s += "robot_group:\n  robot:\n  - joint:\n";
    for (int i = 0; i < kJoints; ++i) {
        s += "    - kinematics_parameters:\n        d: 0.1\n";
        s += "      dynamics_parameters:\n";
        const char* keys[16] = {"m", "cmx", "cmy", "cmz", "ixx", "ixy", "ixz",
                                "iyy", "iyz", "izz", "fv", "fv2", "fv3", "fc0", "fc1", "jm"};
        for (const char* k : keys) s += "        " + std::string(k) + ": 0.0\n";
    }
    return s;
}

}  // namespace

TEST(ident_export_yaml_has_70_phi_and_actuators) {
    const IdentResult r = sample_result();
    ExportInfo info;
    const std::string y = identified_yaml(r, info);
    CHECK(y.find("n_params: 70") != std::string::npos);
    CHECK(y.find("phi_flat:") != std::string::npos);
    CHECK(y.find("controller_dynamics_parameters:") != std::string::npos);
    CHECK(y.find("fc0: 0.52000000000000002") != std::string::npos);
    CHECK(y.find("fc1: -0.47999999999999998") != std::string::npos);
    CHECK(y.find("joint_3: 0.01") != std::string::npos);
}

TEST(ident_export_sn_template_patches_all_blocks) {
    const IdentResult r = sample_result();
    const std::string out = sn_template_yaml(r, minimal_sn_template());
    CHECK(out.find("m: 6.3606110033526198e-01") != std::string::npos);
    CHECK(out.find("fc0: 5.2000000000000002e-01") != std::string::npos);
    CHECK(out.find("fc1: -4.7999999999999998e-01") != std::string::npos);
    CHECK(out.find("fv2: 0.0000000000000000e+00") != std::string::npos);
}

TEST(ident_export_load_sn_roundtrip) {
    const IdentResult r = sample_result();
    const std::string sn = sn_template_yaml(r, minimal_sn_template());
    const IdentResult got = load_result_yaml(sn);
    for (int i = 0; i < kInertiaParams; ++i) {
        CHECK_NEAR(got.phi[size_t(i)], r.phi[size_t(i)], 1e-12);
    }
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(got.viscous[size_t(j)], r.viscous[size_t(j)], 1e-12);
        // fc0/fc1 recover the symmetric Coulomb term and the DC offset.
        CHECK_NEAR(got.coulomb[size_t(j)], r.coulomb[size_t(j)], 1e-12);
        CHECK_NEAR(got.dc[size_t(j)], r.dc[size_t(j)], 1e-12);
        CHECK_NEAR(got.armature[size_t(j)], r.armature[size_t(j)], 1e-12);
    }
}

TEST(ident_export_load_sn_rejects_partial_config) {
    CHECK_THROWS_AS(load_sn_yaml("robot_group:\n  robot:\n  - joint:\n"),
                    InvalidCommandError);
    CHECK_THROWS_AS(load_result_yaml("robot_group:\n  robot:\n  - joint:\n"),
                    InvalidCommandError);
}

TEST(ident_export_phi_to_body10_about_com_in_dh_frame) {
    const Phi70 phi = cad_phi();
    std::array<double, 10> link1{};
    for (int k = 0; k < 10; ++k) link1[size_t(k)] = phi[size_t(k)];
    const auto p = phi_to_body10(link1);
    // Firmware body10 in the Craig/MDH link frame, about the COM: this must
    // reproduce the DH/litearm_dh.urdf <inertial> block of link joint1.
    CHECK_NEAR(p[0], 0.636061100335262, 1e-15);
    CHECK_NEAR(p[1], -0.0001447613913, 1e-15);
    CHECK_NEAR(p[2], 0.0001932544252, 1e-15);
    CHECK_NEAR(p[3], -0.02194391221, 1e-15);
    CHECK_NEAR(p[4], 0.00035685639071821067, 1e-15);
    CHECK_NEAR(p[5], 4.44212600824964e-08, 1e-15);
    CHECK_NEAR(p[6], -1.9576777316502063e-06, 1e-15);
    CHECK_NEAR(p[7], 0.0005549359023792373, 1e-15);
    CHECK_NEAR(p[8], 1.0199656140599972e-06, 1e-15);
    CHECK_NEAR(p[9], 0.000409413887893391, 1e-15);
}

TEST(ident_export_body10_phi_roundtrip) {
    const Phi70 phi = cad_phi();
    for (int i = 0; i < kJoints; ++i) {
        std::array<double, 10> link{};
        for (int k = 0; k < 10; ++k) link[size_t(k)] = phi[size_t(i * 10 + k)];
        const auto back = body10_to_phi(phi_to_body10(link));
        for (int k = 0; k < 10; ++k) {
            CHECK_NEAR(back[size_t(k)], link[size_t(k)], 1e-15);
        }
    }
}

TEST(ident_export_load_yaml_roundtrip) {
    const IdentResult r = sample_result();
    ExportInfo info;
    const IdentResult got = load_identified_yaml(identified_yaml(r, info));
    for (int i = 0; i < kInertiaParams; ++i) {
        CHECK_NEAR(got.phi[size_t(i)], r.phi[size_t(i)], 1e-15);
    }
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(got.coulomb[size_t(j)], r.coulomb[size_t(j)], 1e-15);
        CHECK_NEAR(got.viscous[size_t(j)], r.viscous[size_t(j)], 1e-15);
        CHECK_NEAR(got.dc[size_t(j)], r.dc[size_t(j)], 1e-15);
        CHECK_NEAR(got.armature[size_t(j)], r.armature[size_t(j)], 1e-15);
    }
}

TEST(ident_export_load_yaml_rejects_short_phi) {
    CHECK_THROWS_AS(load_identified_yaml("phi_flat:\n- 1.0\n- 2.0\n"),
                    InvalidCommandError);
}

TEST(ident_export_rejects_bad_coulomb_signs) {
    IdentResult r = sample_result();
    r.dc[0] = -10.0;   // fc0 = 0.5 - 10 < 0
    CHECK_THROWS_AS(sn_template_yaml(r, minimal_sn_template()),
                    InvalidCommandError);
}

TEST(ident_export_rejects_unphysical_inertia) {
    IdentResult r = sample_result();
    r.phi[9] = 0.1;  // link 1: Izz >> Ixx+Iyy, triangle inequality violated
    CHECK(!physically_valid(r.phi));
    CHECK_THROWS_AS(identified_yaml(r, ExportInfo{}), InvalidCommandError);
    CHECK_THROWS_AS(sn_template_yaml(r, minimal_sn_template()), InvalidCommandError);
}

TEST(ident_export_rejects_nonfinite_inertia) {
    IdentResult r = sample_result();
    r.phi[0] = std::nan("");
    CHECK_THROWS_AS(identified_yaml(r, ExportInfo{}), InvalidCommandError);
    CHECK_THROWS_AS(sn_template_yaml(r, minimal_sn_template()), InvalidCommandError);
}

TEST(ident_export_load_yaml_rejects_nan) {
    CHECK_THROWS_AS(load_identified_yaml("phi_flat:\n- .nan\n"),
                    InvalidCommandError);
}
