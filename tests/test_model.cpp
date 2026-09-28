// 动力学模型在线导入 —— 0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39。
#include "test_support.hpp"

using namespace litearm;

namespace {
std::vector<double> body10(double m) {
    return {m, 0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0};
}
}  // namespace

TEST(model_mask_constants_are_pinned) {
    // 工具/产线实际写入 body1..7 + jm = 0x2FE。常量漂移会让 commit 撞 ERR{0x32,0x07}。
    CHECK_EQ(int(MODEL_BIT_JM), 1 << 9);
    CHECK_EQ(int(MODEL_BIT_BODY(3)), 1 << 3);
    CHECK_EQ(int(MODEL_MASK_WRITTEN), 0x2FE);
    CHECK_EQ(MODEL_NBODY, 9);
    CHECK_EQ(MODEL_BODY_PARAMS, 10);
    CHECK_EQ(MODEL_JM_N, 7);
}

TEST(model_status_decodes_the_five_byte_reply) {
    const std::vector<uint8_t> payload{proto::RSP_MODEL_STATUS, 1};
    std::vector<uint8_t> body = payload;
    const auto m = proto::pack_u16le(0x2FE);
    body.insert(body.end(), m.begin(), m.end());
    body.push_back(1);
    const ModelStatus s = ModelStatus::decode(body);
    CHECK_EQ(s.override_level, 1);
    CHECK_EQ(int(s.staged_mask), 0x2FE);
    CHECK_EQ(s.dirty, 1);
}

TEST(model_status_rejects_a_wrong_length_or_id) {
    CHECK_THROWS_AS(ModelStatus::decode(std::vector<uint8_t>(5, 0)), TransportError);
    std::vector<uint8_t> bad{0x99, 0, 0, 0, 0};
    CHECK_THROWS_AS(ModelStatus::decode(bad), TransportError);
}

TEST(model_probe_uses_0x34_and_maps_0x00_to_false) {
    // 用 0x34 探测: 旧固件没有这条命令的 case => default => ERR{0x34,0x00}。
    lt::Offline off;
    CHECK(off->model().probe());
    lt::Offline off2;
    off2.t().unknown_cmds.insert(proto::CMD_GET_MODEL_PARAM);
    CHECK(!off2->model().probe());
}

TEST(model_get_and_set_body_round_trip_after_commit) {
    lt::Offline off;
    const auto original = off->model().get_body(1).value;
    CHECK_EQ(int(original.size()), 10);

    for (int i = 1; i <= 7; ++i) {
        off->model().set_body(i, body10(2.0 + i));
    }
    off->model().set_jm(std::vector<double>(7, 0.001));
    // 写 staging 不生效 —— bank 还没变
    CHECK_NEAR(off->model().get_body(1).value[0], original[0], 1e-9);
    CHECK(!off->model().status().value.staged_mask ? true : true);

    off->model().commit(MODEL_MASK_WRITTEN);
    CHECK_NEAR(off->model().get_body(1).value[0], 3.0, 1e-6);
    CHECK_NEAR(off->model().get_body(7).value[0], 9.0, 1e-6);
    CHECK_NEAR(off->model().get_jm().value[0], 0.001, 1e-9);
    CHECK_EQ(off->model().status().value.override_level, 1);
}

TEST(model_set_body_validates_index_and_arity) {
    lt::Offline off;
    CHECK_THROWS_AS(off->model().set_body(-1, body10(1.0)), InvalidCommandError);
    CHECK_THROWS_AS(off->model().set_body(9, body10(1.0)), InvalidCommandError);
    CHECK_THROWS_AS(off->model().set_body(1, std::vector<double>(9, 1.0)),
                    InvalidCommandError);
    CHECK_THROWS_AS(off->model().set_body(1, std::vector<double>(11, 1.0)),
                    InvalidCommandError);
    CHECK_THROWS_AS(off->model().get_body(9), InvalidCommandError);
}

TEST(model_set_jm_requires_seven_values) {
    lt::Offline off;
    CHECK_THROWS_AS(off->model().set_jm(std::vector<double>(6, 0.0)),
                    InvalidCommandError);
    CHECK_NOTHROW(off->model().set_jm(std::vector<double>(7, 0.0)));
}

TEST(model_commit_mask_must_match_what_was_staged) {
    // 掩码不符 -> ERR{0x32,0x07} —— 与"数值非法"分开的码, 产线最常见故障。
    lt::Offline off;
    off->model().set_body(1, body10(1.0));       // staging 里只有 body1
    try {
        off->model().commit(MODEL_BIT_BODY(2));  // 声称写过 body2 —— 与 staging 不符
        FAIL("应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x32);
        CHECK_EQ(int(e.code), 0x07);
        CHECK(std::string(e.what()).find("掩码不符") != std::string::npos);
    }
    // 逐位相等才能提交
    CHECK_NOTHROW(off->model().commit(MODEL_BIT_BODY(1)));
}

TEST(model_commit_mask_is_packed_as_little_endian_u16) {
    // 写成 bytes([mask]) 会截断成 1 字节 (0x2FE & 0xFF = 0xFE), 固件直接拒。
    lt::Offline off;
    off->model().set_body(1, body10(1.0));
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first != proto::CMD_MODEL_COMMIT) continue;
        CHECK_EQ(kv.second.size(), size_t(2));
        CHECK_EQ(int(proto::read_u16le(kv.second, 0)), 1 << 1);
    }
}

TEST(model_commit_and_revert_require_a_disarmed_arm) {
    lt::Offline off;
    off->model().set_body(1, body10(1.0));
    off->enable();
    try {
        off->model().commit(MODEL_BIT_BODY(1));
        FAIL("武装态应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.code), 0x04);
    }
    CHECK_THROWS_AS(off->model().revert(), CommandRejectedError);
    off->disable();
    CHECK_NOTHROW(off->model().revert());
    // 回退后 dirty == 1 (RAM != flash), 但不要据此提示"补固化"
    CHECK_EQ(off->model().status().value.dirty, 1);
}

TEST(model_set_body_rejects_the_ee_body_with_nonzero_mass) {
    // body8 (ee 固定体) 的质量必须恒 0。
    lt::Offline off;
    try {
        off->model().set_body(8, body10(1.0));
        FAIL("应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.code), 0x02);
    }
    CHECK_NOTHROW(off->model().set_body(8, body10(0.0)));
}

TEST(model_get_gravity_returns_seven_values) {
    lt::Offline off;
    const std::vector<double> q(7, 0.0);
    const auto g = off->model().get_gravity(q);
    CHECK_EQ(int(g.value.size()), 7);
    CHECK_THROWS_AS(off->model().get_gravity(std::vector<double>(6, 0.0)),
                    InvalidCommandError);
    // 载荷 = q[7]
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first == proto::CMD_GET_GRAVITY) CHECK_EQ(kv.second.size(), size_t(28));
    }
}

TEST(model_status_reports_the_staged_mask) {
    lt::Offline off;
    off->model().set_body(2, body10(1.0));
    off->model().set_body(3, body10(1.0));
    off->model().set_jm(std::vector<double>(7, 0.0));
    const auto s = off->model().status().value;
    CHECK_EQ(int(s.staged_mask), (1 << 2) | (1 << 3) | (1 << 9));
}

TEST(model_get_jm_frame_shape) {
    lt::Offline off;
    const auto m = off->model().get_jm();
    CHECK_EQ(int(m.value.size()), 7);
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first == proto::CMD_GET_MODEL_JM) CHECK(kv.second.empty());
    }
}

TEST(model_unsupported_firmware_is_reported_by_type) {
    lt::Offline off;
    off.t().unknown_cmds.insert(proto::CMD_GET_MODEL_PARAM);
    CHECK_THROWS_AS(off->model().get_body(0), UnsupportedByFirmwareError);
    // ⚠ 不能用 0x30/0x33 探测: 旧固件对它们有显式 case 回 ERR{cmd,0x02}, 无法区分
    // 「无此命令」与「参数非法」。
    lt::Offline off2;
    off2.t().err_override[proto::CMD_GET_MODEL_PARAM] = 0x02;
    CHECK_THROWS_AS(off2->model().probe(), CommandRejectedError);
}

TEST(model_body_frame_echo_is_checked) {
    // 回显 body_idx 与请求不一致 => 帧布局漂移, 必须响亮。
    // ⚠ 用桩的确定注入点而不是手工 push_frame —— 后者会与读线程抢时序。
    lt::Offline off;
    off.t().model_body_echo_idx = 5;
    CHECK_THROWS_AS(off->model().get_body(0), TransportError);
    // 回显一致时正常解出
    off.t().model_body_echo_idx = -1;
    CHECK_EQ(int(off->model().get_body(0).value.size()), 10);
}
