// RobotState —— 状态帧的面向对象视图 (enabled / cart_busy / faulted / 断轴 / 推断值)。
#include "test_support.hpp"

using namespace litearm;

namespace {
RobotState st_with(uint16_t flags, int mode, uint16_t joint_fault, int n = 7) {
    std::vector<double> q(size_t(n), 0.1);
    const auto payload = lt::make_status_payload(flags, 3, q, joint_fault);
    RobotState st = decode_state(payload);
    st.mode = mode;
    st.mode_name = proto::mode_name(mode);
    return st;
}
}  // namespace

TEST(state_enabled_bit_is_bit_9) {
    CHECK(st_with(uint16_t(1u << 9), 1, 0).enabled());
    CHECK(!st_with(0x0000, 1, 0).enabled());
    // 低位不是 enabled
    CHECK(!st_with(0x01, 1, 0).enabled());
}

TEST(state_cart_busy_bit_is_bit_10_and_is_not_a_safety_flag) {
    const RobotState st = st_with(uint16_t(1u << 10), 1, 0);
    CHECK(st.cart_busy());
    // bit6..10 一起变成"故障位"是本属性要防的方向: flag_names 只看 bit0..bit5。
    CHECK(st.flag_names.empty());
    CHECK(!st.faulted());
}

TEST(state_faulted_covers_fault_bit_emergency_and_joint_fault) {
    CHECK(st_with(0x0001, 1, 0).faulted());            // FAULT 位
    CHECK(st_with(0x0000, 6, 0).faulted());            // EMERGENCY
    CHECK(st_with(0x0000, 1, 0x0002).faulted());       // 单轴断轴 (G7)
    CHECK(!st_with(0x0000, 1, 0).faulted());
    // 关键: 单轴锁存不一定置全局 FAULT 位, 若不看 joint_fault 就会等到超时才报"未到位"。
    const RobotState single = st_with(0x0000, 1, 0x0004);
    CHECK(!(single.flags & 1));
    CHECK(single.faulted());
}

TEST(state_flag_names_list_the_set_safety_bits) {
    const RobotState st = st_with(uint16_t(0x01 | 0x04), 1, 0);
    CHECK_EQ(int(st.flag_names.size()), 2);
    CHECK_EQ(st.flag_names[0], std::string("FAULT"));
    CHECK_EQ(st.flag_names[1], std::string("FB_STALE"));
}

TEST(state_fault_axes_are_zero_based) {
    const RobotState st = st_with(0, 1, uint16_t((1u << 1) | (1u << 3)));
    const auto ax = st.fault_axes();
    CHECK_EQ(int(ax.size()), 2);
    CHECK_EQ(ax[0], 1);   // J2
    CHECK_EQ(ax[1], 3);   // J4
}

TEST(state_fault_axes_cover_all_16_bits) {
    const RobotState st = st_with(0, 1, 0xFFFF);
    CHECK_EQ(int(st.fault_axes().size()), proto::MAX_JOINTS);
}

TEST(state_fault_detail_is_readable) {
    CHECK_EQ(st_with(0, 1, 0).fault_detail(), std::string("无故障位"));
    const RobotState st = st_with(0x01, 1, uint16_t(1u << 2));
    const std::string d = st.fault_detail();
    CHECK(d.find("flags=FAULT") != std::string::npos);
    CHECK(d.find("断轴=J3") != std::string::npos);
}

TEST(state_drop_hold_inferred_is_exactly_joint_fault_nonzero) {
    // 名字带 _inferred 是刻意的: 固件不上报这个量。判据是那条单向蕴含的逆否
    // (joint_fault == 0 蕴含 drop_hold == false), 反向不成立。
    CHECK(!st_with(0, 1, 0).drop_hold_inferred());
    CHECK(st_with(0, 1, 0x0001).drop_hold_inferred());
    // 等价写法就是 bool(joint_fault)
    const RobotState st = st_with(0, 1, 0x0008);
    CHECK(st.drop_hold_inferred() == (st.joint_fault != 0));
}

TEST(state_q_dq_tau_projections) {
    // 状态帧的关节数只有 1 与 7 两种合法布局 (见 decode_status)。
    std::vector<double> q{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    std::vector<uint8_t> body;
    const auto h = proto::pack_u16le(0);
    body.insert(body.end(), h.begin(), h.end());
    const auto s = proto::pack_u16le(1);
    body.insert(body.end(), s.begin(), s.end());
    for (double v : q) {
        const auto f = proto::pack_f32s(std::vector<double>{v, v * 2, v * 3, 0, 0});
        body.insert(body.end(), f.begin(), f.end());
        body.push_back(0);
    }
    const auto jf = proto::pack_u16le(0);
    body.insert(body.end(), jf.begin(), jf.end());
    const RobotState st = decode_state(body);
    CHECK_EQ(int(st.n()), 7);
    CHECK_EQ(int(st.q().size()), 7);
    CHECK_NEAR(st.q()[1], 2.0, 1e-6);
    CHECK_NEAR(st.dq()[2], 6.0, 1e-6);
    CHECK_NEAR(st.tau()[0], 3.0, 1e-6);
}

TEST(state_decode_state_rejects_illegal_frame) {
    CHECK_THROWS_AS(decode_state(std::vector<uint8_t>{1, 2, 3}), TransportError);
    CHECK_THROWS_AS(decode_state(std::vector<uint8_t>(4 + 21 * 3, 0)), TransportError);
}

TEST(state_decode_state_maps_mode_name) {
    const std::vector<double> q(7, 0.0);
    for (int mode = 0; mode < 8; ++mode) {
        const auto payload = lt::make_status_payload(uint16_t(mode << 6), 0, q);
        const RobotState st = decode_state(payload);
        CHECK_EQ(st.mode, mode);
        CHECK_EQ(st.mode_name, std::string(proto::mode_name(mode)));
    }
}
