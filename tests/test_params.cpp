// 关节级参数 —— 0x22 / 0x23 / 0x24 / 0x36。
#include "test_support.hpp"

using namespace litearm;

TEST(params_get_returns_the_decoded_record) {
    lt::Offline off;
    const auto m = off->params().get_joint_param(0);
    CHECK_EQ(m.value.idx, 0);
    CHECK_NEAR(m.value.kp, 50.0, 1e-6);
    CHECK_NEAR(m.value.kd, 2.0, 1e-6);
    CHECK_NEAR(m.value.tau_max, 10.0, 1e-6);
    CHECK_NEAR(m.value.q_min, -3.0, 1e-6);
    CHECK_NEAR(m.value.q_max, 3.0, 1e-6);
}

TEST(params_set_then_read_back_closes_the_loop) {
    lt::Offline off;
    off->params().set_joint_param(2, 30.0, 1.5, 20.0);
    const auto m = off->params().get_joint_param(2);
    CHECK_NEAR(m.value.kp, 30.0, 1e-6);
    CHECK_NEAR(m.value.kd, 1.5, 1e-6);
    CHECK_NEAR(m.value.tau_max, 20.0, 1e-6);
    // 别的关节没被动
    CHECK_NEAR(off->params().get_joint_param(3).value.kp, 50.0, 1e-6);
}

TEST(params_set_limits_then_read_back) {
    lt::Offline off;
    off->params().set_joint_limits(0, -1.0, 1.0);
    const auto m = off->params().get_joint_param(0);
    CHECK_NEAR(m.value.q_min, -1.0, 1e-6);
    CHECK_NEAR(m.value.q_max, 1.0, 1e-6);
}

TEST(params_limits_require_qmin_lt_qmax_locally) {
    // 固件要求 q_min < q_max; 本地先说清楚, 免得白跑一趟。
    lt::Offline off;
    CHECK_THROWS_AS(off->params().set_joint_limits(0, 1.0, 1.0), InvalidCommandError);
    CHECK_THROWS_AS(off->params().set_joint_limits(0, 2.0, 1.0), InvalidCommandError);
}

TEST(params_index_is_bounds_checked_against_the_joint_count) {
    lt::Offline off;
    CHECK_THROWS_AS(off->params().get_joint_param(-1), InvalidCommandError);
    CHECK_THROWS_AS(off->params().get_joint_param(7), InvalidCommandError);
    CHECK_THROWS_AS(off->params().set_joint_param(7, 1, 1, 1), InvalidCommandError);
    CHECK_THROWS_AS(off->params().set_joint_limits(7, 0, 1), InvalidCommandError);
    CHECK_NOTHROW(off->params().get_joint_param(6));
}

TEST(params_all_joint_params_aggregates_n_roundtrips) {
    // 返回 vector<JointParam> —— 不是 vector<Msg>: N 帧的信封一个 hz 描述不了。
    lt::Offline off;
    const auto all = off->params().all_joint_params();
    CHECK_EQ(int(all.size()), 7);
    for (int i = 0; i < 7; ++i) CHECK_EQ(all[size_t(i)].idx, i);
}

TEST(params_all_joint_params_on_the_single_axis_bench) {
    lt::Offline off("Litearm1.7.0-1J", 1);
    CHECK_EQ(int(off->params().all_joint_params().size()), 1);
}

TEST(params_reset_factory_requires_a_disarmed_arm) {
    lt::Offline off;
    off->params().set_joint_param(0, 30.0, 1.0, 20.0);
    off->enable();
    try {
        off->params().reset_factory();
        FAIL("武装态应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x36);
        CHECK_EQ(int(e.code), 0x04);
    }
    off->disable();
    CHECK_NOTHROW(off->params().reset_factory());
    // 回出厂默认
    CHECK_NEAR(off->params().get_joint_param(0).value.kp, 50.0, 1e-6);
}

TEST(params_firmware_rejections_are_surfaced_with_their_codes) {
    lt::Offline off;
    off.t().err_override[proto::CMD_SET_JOINT_LIMITS] = 0x03;
    try {
        off->params().set_joint_limits(0, -1.0, 1.0);
        FAIL("应当抛异常");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.code), 0x03);
        // 0x23 的 0x03 有它自己的语义 (新限位关不住在途目标)
        CHECK(std::string(e.what()).find("在途目标") != std::string::npos);
    }
}

TEST(params_unsupported_read_is_reported_as_such) {
    lt::Offline off;
    off.t().unknown_cmds.insert(proto::CMD_GET_JOINT_PARAM);
    CHECK_THROWS_AS(off->params().get_joint_param(0), UnsupportedByFirmwareError);
}

TEST(params_short_frame_is_rejected_by_the_decoder) {
    CHECK_THROWS_AS(JointParam::decode(std::vector<uint8_t>(10, 0)), TransportError);
}

TEST(params_armed_gate_on_limits_returns_state_code) {
    // 武装中且该轴的当前参考落在新区间之外 -> ERR{0x23,0x03} (固件 ctrl_axis_outside 第①/⑤条)
    lt::Offline off;
    off->enable();
    off.t().q[0] = 0.0;
    try {
        off->params().set_joint_limits(0, 1.0, 2.0);   // 当前 q=0 在新区间外
        FAIL("应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.code), 0x03);
    }
    CHECK_NOTHROW(off->params().set_joint_limits(0, -1.0, 1.0));   // 容得下当前值
}
