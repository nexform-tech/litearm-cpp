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
