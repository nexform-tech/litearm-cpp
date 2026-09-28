// 前馈/动力学调参 (0x26/0x27/0x28/0x31 + 读回 0x2B/0x2C)。
#include "test_support.hpp"

using namespace litearm;

TEST(ff_item_tables_cover_the_firmware_ranges) {
    // item 1..15 都能写/能读回 (含 12~15 的零重力与 kd_extra)
    CHECK_EQ(int(Arm::ff_vec_items().size()), 15);
    CHECK_EQ(int(Arm::ff_scalar_items().size()), 17);   // 1..8 + 10..18
    CHECK_EQ(int(Arm::ff_scalar_ro_items().size()), 1);
    CHECK_EQ(Arm::ff_scalar_ro_items()[0].first, 9);
    // item 7/8 是具名入口的载体
    CHECK_EQ(Arm::ff_vec_items()[6].first, 7);
    CHECK_EQ(Arm::ff_vec_items()[6].second, std::string("gravity_scale"));
    CHECK_EQ(Arm::ff_vec_items()[14].first, 15);
    CHECK_EQ(Arm::ff_vec_items()[14].second, std::string("kd_extra"));
}

TEST(ff_vec_write_then_read_back) {
    lt::Offline off;
    const std::vector<double> gs{1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6};
    off->set_ff_vec(7, gs);
    const auto got = off->get_ff_vec(7);
    CHECK_EQ(int(got.value.size()), 7);
    for (size_t i = 0; i < gs.size(); ++i) CHECK_NEAR(got.value[i], gs[i], 1e-6);
}

TEST(ff_vec_validates_item_and_arity) {
    lt::Offline off;
    CHECK_THROWS_AS(off->set_ff_vec(0, std::vector<double>(7, 0.0)), InvalidCommandError);
    CHECK_THROWS_AS(off->set_ff_vec(16, std::vector<double>(7, 0.0)), InvalidCommandError);
    CHECK_THROWS_AS(off->set_ff_vec(7, std::vector<double>(6, 0.0)), InvalidCommandError);
    CHECK_THROWS_AS(off->get_ff_vec(0), InvalidCommandError);
    CHECK_THROWS_AS(off->get_ff_vec(16), InvalidCommandError);
    // item 1..15 全都能读写
    for (int item = 1; item <= 15; ++item) {
        CHECK_NOTHROW(off->set_ff_vec(item, std::vector<double>(7, 0.5)));
        CHECK_NOTHROW(off->get_ff_vec(item));
    }
}

TEST(ff_named_gravity_and_inertia_scales) {
    lt::Offline off;
    const std::vector<double> v{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    CHECK_NOTHROW(off->set_gravity_scale(v));
    CHECK_NOTHROW(off->set_inertia_scale(v));
    CHECK_THROWS_AS(off->set_gravity_scale(std::vector<double>(4, 1.0)),
                    InvalidCommandError);
    CHECK_THROWS_AS(off->set_inertia_scale(std::vector<double>(4, 1.0)),
                    InvalidCommandError);
}

TEST(ff_scalar_write_then_read_back) {
    lt::Offline off;
    off->set_ff_scalar(4, 0, 1.25);          // payload_mass
    CHECK_NEAR(off->get_ff_scalar(4, 0).value, 1.25, 1e-6);
    off->set_ff_scalar(5, 2, -0.5);          // payload_com z
    CHECK_NEAR(off->get_ff_scalar(5, 2).value, -0.5, 1e-6);
    // item 10..18 (零重力族与 hold_kp_gain) 也能写
    for (int item : {10, 11, 12, 13, 14, 15, 16, 17, 18}) {
        CHECK_NOTHROW(off->set_ff_scalar(item, 0, 0.5));
        CHECK_NOTHROW(off->get_ff_scalar(item, 0));
    }
}

TEST(ff_scalar_validates_item_and_sub) {
    lt::Offline off;
    CHECK_THROWS_AS(off->set_ff_scalar(0, 0, 1.0), InvalidCommandError);
    CHECK_THROWS_AS(off->set_ff_scalar(9, 0, 1.0), InvalidCommandError);   // item 9 保留
    CHECK_THROWS_AS(off->set_ff_scalar(19, 0, 1.0), InvalidCommandError);
    CHECK_THROWS_AS(off->set_ff_scalar(4, 3, 1.0), InvalidCommandError);   // sub 0..2
    CHECK_THROWS_AS(off->set_ff_scalar(4, -1, 1.0), InvalidCommandError);
    // 读口收 item 9 (只读扩展) 与 1..18
    CHECK_NOTHROW(off->get_ff_scalar(9, 0));
    CHECK_THROWS_AS(off->get_ff_scalar(19), InvalidCommandError);
}

TEST(ff_mask_refuses_bits_outside_ff_all) {
    // 固件侧会做 mask & FF_ALL_MASK, 于是 0x1000 这类误用被静默折成 0 = 前馈全关
    // (重力补偿被关掉, 臂会垂下来)。宁可在这里报错, 也不让一个危险值悄悄变成另一个"合法"值。
    lt::Offline off;
    CHECK_NOTHROW(off->set_ff_mask(0));
    CHECK_NOTHROW(off->set_ff_mask(proto::FF_ALL));
    CHECK_NOTHROW(off->set_ff_mask(proto::FF_G | proto::FF_INERTIA));
    CHECK_THROWS_AS(off->set_ff_mask(0x1000), InvalidCommandError);
    CHECK_THROWS_AS(off->set_ff_mask(0xFFFFFFFFu), InvalidCommandError);
}

TEST(ff_mask_reads_back_through_the_read_only_item) {
    lt::Offline off;
    off->set_ff_mask(proto::FF_G | proto::FF_FRICTION);
    // get_ff_mask 是 get_ff_scalar(9,0) 的标量投影 (走 0x2C item 9, 只读)
    const int mask = off->get_ff_mask();
    CHECK_EQ(mask, int(proto::FF_G | proto::FF_FRICTION));
}

TEST(ff_preset_validates_the_range) {
    lt::Offline off;
    CHECK_NOTHROW(off->ff_preset(0));
    CHECK_NOTHROW(off->ff_preset(1));
    CHECK_NOTHROW(off->ff_preset(2));
    CHECK_THROWS_AS(off->ff_preset(3), InvalidCommandError);
    CHECK_THROWS_AS(off->ff_preset(-1), InvalidCommandError);
}

TEST(ff_set_payload_writes_mass_and_each_component) {
    lt::Offline off;
    const size_t before = off.t().tx_count();
    off->set_payload(1.5, {0.01, -0.02, 0.03});
    // 1 次 item4 + 3 次 item5
    // ⚠ 取**一次**快照再数, 不要边遍历边问桩要长度: 保活线程会并发 append,
    //   裸读 `tx_log` 是 use-after-free 级的竞争 (见 `tx_snapshot` 的说明)。
    const auto log = off.t().tx_snapshot();
    int n = 0;
    for (size_t i = before; i < log.size(); ++i) {
        if (log[i].first == proto::CMD_SET_FF_SCALAR) ++n;
    }
    CHECK_EQ(n, 4);
    CHECK_NEAR(off->get_ff_scalar(4, 0).value, 1.5, 1e-6);
    CHECK_NEAR(off->get_ff_scalar(5, 0).value, 0.01, 1e-6);
    CHECK_NEAR(off->get_ff_scalar(5, 2).value, 0.03, 1e-6);
}

TEST(ff_set_gravity_vector_writes_three_components) {
    lt::Offline off;
    off->set_gravity_vector({0.0, 0.0, -9.81});
    CHECK_NEAR(off->get_ff_scalar(6, 2).value, -9.81, 1e-4);
}

TEST(ff_save_params_requires_a_disarmed_arm) {
    // 写 flash 期间电机不得在无监督下保持使能 —— 固件回 ERR{0x25,0x04}。
    lt::Offline off;
    off->enable();
    try {
        off->save_params();
        FAIL("武装态应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x25);
        CHECK_EQ(int(e.code), 0x04);
    }
    off->disable();
    CHECK_NOTHROW(off->save_params());
}

TEST(ff_errors_propagate_the_firmware_code) {
    lt::Offline off;
    off.t().err_override[proto::CMD_SET_FF_VEC] = 0x02;
    try {
        off->set_ff_vec(1, std::vector<double>(7, 1.0));
        FAIL("应当抛异常");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.code), 0x02);
        CHECK(std::string(e.what()).find("符号契约") != std::string::npos);
    }
}
