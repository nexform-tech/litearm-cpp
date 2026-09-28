// 授权/激活 —— 0x2F 查询 / 0x3F 提交凭据 / 未授权时的 ENABLE 门禁。
#include "test_support.hpp"

using namespace litearm;

TEST(license_decode_reads_the_twenty_six_byte_record) {
    std::vector<uint8_t> body{1, 1};                       // state=1, ver=1
    const std::vector<uint8_t> uid{0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                                   0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B};
    body.insert(body.end(), uid.begin(), uid.end());
    const auto a = proto::pack_u32le(42);
    const auto b = proto::pack_u32le(20260924);
    const auto c = proto::pack_u32le(1);
    body.insert(body.end(), a.begin(), a.end());
    body.insert(body.end(), b.begin(), b.end());
    body.insert(body.end(), c.begin(), c.end());
    CHECK_EQ(body.size(), size_t(26));
    const LicenseInfo info = LicenseInfo::decode(body);
    CHECK_EQ(info.state, 1);
    CHECK_EQ(info.ver, 1);
    CHECK(info.uid[0] == 0x10);
    CHECK_EQ(int(info.cust_id), 42);
    CHECK_EQ(int(info.issued), 20260924);
    CHECK(info.activated());
    CHECK(info.factory_mode());
}

TEST(license_decode_rejects_a_wrong_length) {
    CHECK_THROWS_AS(LicenseInfo::decode(std::vector<uint8_t>(25, 0)), TransportError);
    CHECK_THROWS_AS(LicenseInfo::decode(std::vector<uint8_t>(27, 0)), TransportError);
}

TEST(license_state_names_cover_the_three_known_codes_and_unknowns) {
    LicenseInfo a;
    a.state = 0;
    CHECK_EQ(a.state_name(), std::string("not_activated"));
    a.state = 1;
    CHECK_EQ(a.state_name(), std::string("activated"));
    a.state = 2;
    CHECK_EQ(a.state_name(), std::string("activated_factory"));
    // 认不出的码带上原值回 (不静默)
    a.state = 7;
    CHECK_EQ(a.state_name(), std::string("unknown_state_7"));
    CHECK_EQ(std::string(license_state_name(0)), std::string("not_activated"));
}

TEST(license_activated_is_state_nonzero_and_factory_mode_is_a_separate_bit) {
    LicenseInfo a;
    a.state = 0;
    a.flags = 0x1;
    // 产线码不表示"激活与否", 别拿它替代 activated()
    CHECK(!a.activated());
    CHECK(a.factory_mode());
    a.state = 2;
    CHECK(a.activated());
}

TEST(license_uid_hex_is_the_twenty_four_char_form_the_signer_wants) {
    lt::Offline off;
    const LicenseInfo info = off->license();
    CHECK_EQ(int(info.uid_hex().size()), 24);
    CHECK_EQ(info.uid_hex(), std::string("10111213141516171819 1a1b").substr(0, 0) +
                                 std::string("10111213141516171819") + "1a1b");
    CHECK(info.activated());
}

TEST(license_reads_the_uid_even_when_unactivated) {
    // 未激活也回 UID, 而 cust_id/issued/flags 全 0 —— 签发器必须从本记录取那 12 字节,
    // 不得改用 USB 序列号字符串。
    lt::Offline off;
    off.t().activated = false;
    off.t().license_cust_id = 1234;   // 桩里设了也不会回 (固件约定)
    off.t().license_issued = 5678;
    const LicenseInfo info = off->license();
    CHECK(!info.activated());
    CHECK_EQ(int(info.cust_id), 0);
    CHECK_EQ(int(info.issued), 0);
    CHECK_EQ(int(info.flags), 0);
    CHECK_EQ(info.uid_hex(), std::string("10111213141516171819") + "1a1b");
}

TEST(license_unactivated_firmware_refuses_enable_with_its_own_code) {
    // 固件 ctrl_enable() 的**第一条**判据就是 !license_is_activated(), 优先于其余所有码。
    // 重发无用、无旁路。
    lt::Offline off;
    off.t().activated = false;
    try {
        off->enable(3);
        FAIL("未激活不该使能成功");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x10);
        CHECK_EQ(int(e.code), 0x08);
        CHECK(std::string(e.what()).find("未激活") != std::string::npos);
    }
}

TEST(license_activate_validates_the_mac_length_locally) {
    lt::Offline off;
    const uint8_t mac[16] = {0};
    CHECK_THROWS_AS(off->activate(1, 20260924, 0, mac, 15), InvalidCommandError);
    CHECK_THROWS_AS(off->activate(1, 20260924, 0, mac, 17), InvalidCommandError);
}

TEST(license_activate_validates_the_reserved_flag_bits_locally) {
    // 保留位必须为 0 (固件会拒, 但它把失败折进 0x02 的聚合档 => 本地先说清楚)。
    lt::Offline off;
    const uint8_t mac[16] = {0};
    CHECK_THROWS_AS(off->activate(1, 20260924, 0x2, mac, 16), InvalidCommandError);
    CHECK_THROWS_AS(off->activate(1, 20260924, 0xFF, mac, 16), InvalidCommandError);
}

TEST(license_activate_succeeds_and_the_payload_is_twenty_eight_bytes) {
    lt::Offline off;
    off.t().activated = false;
    const uint8_t mac[16] = {0xAA, 0xBB, 0xCC, 0xDD, 1, 2, 3, 4,
                             5, 6, 7, 8, 9, 10, 11, 12};
    CHECK_NOTHROW(off->activate(7, 20260924, 0, mac, 16));
    CHECK(off.t().activated);
    CHECK_EQ(int(off.t().license_cust_id), 7);
    CHECK_EQ(int(off.t().license_issued), 20260924);
    bool checked = false;
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first != proto::CMD_ACTIVATE) continue;
        CHECK_EQ(kv.second.size(), size_t(28));
        // cust_id/issued/flags 都是 LE u32, 之后跟 16 字节 MAC
        CHECK_EQ(int(proto::read_u32le(kv.second, 0)), 7);
        CHECK_EQ(int(proto::read_u32le(kv.second, 4)), 20260924);
        CHECK_EQ(int(proto::read_u32le(kv.second, 8)), 0);
        CHECK_EQ(int(kv.second[12]), 0xAA);
        checked = true;
    }
    CHECK(checked);
}

TEST(license_activate_complete_flag_reports_factory_mode) {
    lt::Offline off;
    off.t().activated = false;
    const uint8_t mac[16] = {0};
    CHECK_NOTHROW(off->activate(1, 20260924, 0x1, mac, 16));
    CHECK_EQ(off->license().state, 2);
    CHECK(off->license().factory_mode());
}

TEST(license_activate_zero_two_is_an_aggregate_that_must_be_disambiguated) {
    // 固件把「flags 保留位非 0 / 已存在 / MAC 不符 / 密钥非法 / 写或读回失败」**全折成
    // 0x02** —— 于是上一条 ACK 被丢掉后重发就会拿到 0x02, 而机器**其实已经解锁**。
    // 故在这一档必须回读一次 license(): 只有设备确实 state == 0 才抛。
    lt::Offline off;
    // 情形 A: 其实已经激活过 => 0x02 不该被当成失败
    off.t().activated = true;
    const uint8_t mac[16] = {0};
    CHECK_NOTHROW(off->activate(1, 20260924, 0, mac, 16));

    // 情形 B: 真的没解锁 => 照实抛
    lt::Offline off2;
    off2.t().activated = false;
    off2.t().license_fail_next = true;
    try {
        off2->activate(1, 20260924, 0, mac, 16);
        FAIL("真的失败时应当抛");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.code), 0x02);
        CHECK(std::string(e.what()).find("聚合档") != std::string::npos);
    }
}

TEST(license_activate_other_codes_are_raised_directly) {
    lt::Offline off;
    // 直接置桩的武装位 (而不是 enable()): 未激活时 enable 会被 0x08 挡下, 那是另一条路径。
    off.t().enabled = true;   // 武装态 => ERR{0x3F,0x04}
    const uint8_t mac[16] = {0};
    try {
        off->activate(1, 20260924, 0, mac, 16);
        FAIL("武装态应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x3F);
        CHECK_EQ(int(e.code), 0x04);
    }
}

TEST(license_read_does_not_return_a_msg_envelope) {
    // RSP_LICENSE 是请求/应答式的**设备身份记录**, 没有固件发起的流量 —— Msg 的 per-type
    // hz/timestamp 只会变成"调用方自己轮询的频率", 对设备一无所指。
    lt::Offline off;
    const LicenseInfo info = off->license();   // 返回裸 LicenseInfo, 不是 Msg<LicenseInfo>
    CHECK_EQ(int(info.state), 1);
}

TEST(license_on_firmware_without_0x2f_times_out_rather_than_reporting_unsupported) {
    // ⚠ **这是从 Python 版原样继承的行为, 不是移植引入的**: license() 调 expect 时**不传**
    //   echo_cmd, 而 `wait_keys(RSP_LICENSE, 无回显)` 只等 `(RSP_LICENSE, None)` 这一条队
    //   列 —— 固件那条 `ERR{0x2F,0x00}` 落在 `(RSP_ERR, 0x2F)` 里, 于是本入口看不到它,
    //   只能等满窗口报超时 (那条 ERR 随后由下一次 `drain_for(0x2F)` 清掉, 不会串台)。
    //
    //   ⇒ 在**低于 1.8.0** 的固件 (SDK 支持 1.5.0+) 上, `license()` 报的是"无应答"而不是
    //     "固件没有这条命令"。要修只需给那次 expect 传上 `echo_cmd=CMD_GET_LICENSE`,
    //     但那是**行为变更**, 移植时刻意保留原语义 —— 见 README 的"与原版的已知差异"。
    lt::Offline off;
    off.t().unknown_cmds.insert(proto::CMD_GET_LICENSE);
    CHECK_THROWS_AS(off->license(0.15), MotionTimeoutError);
    // 别的入口逐个都传了 echo_cmd, 故它们能把 0x00 判成"固件没这条命令"
    off.t().unknown_cmds.insert(proto::CMD_GET_JOINT_PARAM);
    CHECK_THROWS_AS(off->params().get_joint_param(0), UnsupportedByFirmwareError);
}
