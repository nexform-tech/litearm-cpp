// Firmware model write-back tests using the offline fake transport.
#include "test_support.hpp"

#include "litearm/ident/export.hpp"

using namespace litearm;
using namespace litearm::ident;

namespace {
IdentResult cad_result() {
    IdentResult r;
    r.phi = cad_phi();
    for (int j = 0; j < kJoints; ++j) {
        r.coulomb[size_t(j)] = 0.2;
        r.viscous[size_t(j)] = 0.01;
        r.dc[size_t(j)] = 0.0;
        r.armature[size_t(j)] = j >= 2 ? 0.005 : 0.0;
    }
    return r;
}
}  // namespace

TEST(ident_write_model_bodies_and_readback) {
    lt::Offline off;
    const IdentResult r = cad_result();
    WriteModelOptions opt;
    opt.write_jm = false;
    opt.check_gravity = false;
    const WriteModelReport rep = write_model(*off, r, opt);
    CHECK(rep.committed);
    CHECK_EQ(int(rep.staged_mask), 0x0FE);
    CHECK(rep.max_body_readback_error < 1e-4);
    for (int i = 0; i < kJoints; ++i) {
        std::array<double, 10> p{};
        for (int k = 0; k < 10; ++k) p[size_t(k)] = r.phi[size_t(i * 10 + k)];
        const auto want = phi_to_body10(p);
        const auto got = off->model().get_body(i + 1).value;
        for (int k = 0; k < 10; ++k) CHECK_NEAR(got[size_t(k)], want[size_t(k)], 1e-4);
    }
}

TEST(ident_write_model_optional_jm) {
    lt::Offline off;
    const IdentResult r = cad_result();
    WriteModelOptions opt;
    opt.write_jm = true;
    opt.check_gravity = false;
    const WriteModelReport rep = write_model(*off, r, opt);
    CHECK(rep.committed);
    CHECK_EQ(int(rep.staged_mask), 0x2FE);
    const auto jm = off->model().get_jm().value;
    CHECK_NEAR(jm[0], 0.0, 1e-12);
    CHECK_NEAR(jm[2], 0.005, 1e-6);
    CHECK_NEAR(jm[6], 0.005, 1e-6);
}

TEST(ident_write_model_requires_seven_joints) {
    // 1J 台架自上游 #11 起在 connect 阶段就被拒 (FirmwareMismatchError) ——
    // 变体门放在 Arm::connect(), write_model 的 7 关节检查轮不到它。
    CHECK_THROWS_AS(lt::Offline("Litearm1.7.0-1J", 1), FirmwareMismatchError);
}

TEST(ident_write_model_allows_nonphysical_when_requested) {
    lt::Offline off;
    IdentResult r = cad_result();
    // Link 1's I_com is no longer positive definite.
    r.phi[4] = -10.0;
    WriteModelOptions opt;
    opt.check_gravity = false;
    CHECK_THROWS_AS(write_model(*off, r, opt), InvalidCommandError);
    opt.allow_nonphysical = true;
    const WriteModelReport rep = write_model(*off, r, opt);
    CHECK(rep.committed);
    CHECK(rep.max_body_readback_error < 1e-4);
}

TEST(ident_write_model_friction_and_readback) {
    lt::Offline off;
    const IdentResult r = cad_result();
    WriteModelOptions opt;
    opt.check_gravity = false;
    opt.write_friction = true;
    const WriteModelReport rep = write_model(*off, r, opt);
    CHECK(rep.committed);
    CHECK(rep.friction_written);
    CHECK(rep.max_friction_readback_error < 1e-6);
    CHECK_EQ(int(rep.old_friction.size()), 3);
    CHECK_EQ(int(rep.new_friction.size()), 3);
    const auto fv = off->get_ff_vec(9).value;
    const auto fc0 = off->get_ff_vec(10).value;
    const auto fc1 = off->get_ff_vec(11).value;
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(fv[size_t(j)], r.viscous[size_t(j)], 1e-6);
        CHECK_NEAR(fc0[size_t(j)], r.fc0()[size_t(j)], 1e-6);
        CHECK_NEAR(fc1[size_t(j)], r.fc1()[size_t(j)], 1e-6);
    }
}

TEST(ident_write_model_rejects_bad_friction_sign) {
    lt::Offline off;
    IdentResult r = cad_result();
    r.dc[0] = 1.0;  // fc1[0] = -coulomb + dc = 0.8 > 0
    WriteModelOptions opt;
    opt.check_gravity = false;
    opt.write_friction = true;
    // Refused before any write: the firmware contract is fv>=0, fc0>=0, fc1<=0.
    CHECK_THROWS_AS(write_model(*off, r, opt), InvalidCommandError);
    const auto fc0 = off->get_ff_vec(10).value;
    for (double v : fc0) CHECK_NEAR(v, 0.0, 1e-12);
}

TEST(ident_model_commit_jm_seeds_staged_bodies) {
    lt::Offline off;
    std::vector<std::vector<double>> before;
    for (int i = 1; i <= MODEL_JM_N; ++i) {
        before.push_back(off->model().get_body(i).value);
    }
    const std::vector<double> jm{0.0, 0.0, 0.01, 0.02, 0.03, 0.04, 0.05};
    off->model().commit_jm(jm);
    const auto got_jm = off->model().get_jm().value;
    for (int j = 0; j < MODEL_JM_N; ++j) {
        CHECK_NEAR(got_jm[size_t(j)], jm[size_t(j)], 1e-6);
    }
    for (int i = 0; i < MODEL_JM_N; ++i) {
        const auto got = off->model().get_body(i + 1).value;
        for (int k = 0; k < MODEL_BODY_PARAMS; ++k) {
            CHECK_NEAR(got[size_t(k)], before[size_t(i)][size_t(k)], 1e-6);
        }
    }
    CHECK_EQ(int(off->model().status().value.staged_mask), 0);
}

TEST(ident_model_jm_only_commit_rejected) {
    lt::Offline off;
    off->model().set_jm(std::vector<double>(MODEL_JM_N, 0.001));
    // The firmware zero-model sentinel rejects a staging that has jm but no
    // body group; commit_jm() is the supported path.
    CHECK_THROWS_AS(off->model().commit(MODEL_BIT_JM), CommandRejectedError);
}
