// 帧协议层 —— CRC / 组帧 / 解帧 / 版本解析 / 开机签名 / 状态帧布局 / 覆盖契约。
#include "test_support.hpp"

using namespace litearm;

// ---------------------------------------------------------------- CRC

TEST(protocol_crc_matches_the_standard_check_value) {
    // CRC-16/CCITT-FALSE 的目录标准检查值: "123456789" -> 0x29B1。
    // ⚠ 判据是**目录里的标准值**, 不是"与本函数自己对拍"。
    const std::string s = "123456789";
    CHECK_EQ(proto::crc16_ccitt_false(reinterpret_cast<const uint8_t*>(s.data()),
                                      s.size()),
             uint16_t(0x29B1));
}

TEST(protocol_crc_matches_an_independent_reference_over_many_lengths) {
    // 一份独立的纯位循环参考实现当判据 —— 覆盖长度 0..259 与随机载荷, 逐一比对。
    std::vector<uint8_t> buf;
    uint32_t seed = 0x12345678u;
    for (int len = 0; len <= 259; ++len) {
        buf.clear();
        for (int i = 0; i < len; ++i) {
            seed = seed * 1664525u + 1013904223u;
            buf.push_back(uint8_t(seed >> 24));
        }
        CHECK_EQ(proto::crc16_ccitt_false(buf.data(), buf.size()),
                 lt::crc16_reference(buf.data(), buf.size()));
    }
}

TEST(protocol_crc_empty_input_is_the_init_value) {
    CHECK_EQ(proto::crc16_ccitt_false(nullptr, 0), uint16_t(0xFFFF));
}

// ---------------------------------------------------------------- 组帧/解帧

TEST(protocol_pack_frame_layout_is_byte_exact) {
    const std::vector<uint8_t> payload{0x01, 0x02, 0x03};
    const auto f = proto::pack_frame(0x10, payload);
    CHECK_EQ(f.size(), size_t(8));         // SOF + CMD + LEN + 3 + CRC2
    CHECK_EQ(int(f[0]), int(proto::SOF));
    CHECK_EQ(int(f[1]), 0x10);
    CHECK_EQ(int(f[2]), 3);
    CHECK_EQ(int(f[3]), 0x01);
    const uint16_t crc = uint16_t(f[6] | (f[7] << 8));   // 低字节在前
    CHECK_EQ(crc, lt::crc16_reference(f.data(), 6));
}

TEST(protocol_pack_frame_rejects_over_255_byte_payload) {
    std::vector<uint8_t> big(256, 0);
    CHECK_THROWS_AS(proto::pack_frame(0x10, big), std::invalid_argument);
    std::vector<uint8_t> ok(255, 0);
    CHECK_NOTHROW(proto::pack_frame(0x10, ok));
}

TEST(protocol_unpack_frame_roundtrips) {
    for (int len = 0; len <= 40; ++len) {
        std::vector<uint8_t> payload;
        for (int i = 0; i < len; ++i) payload.push_back(uint8_t(i * 7 + 1));
        const auto f = proto::pack_frame(0x43, payload);
        const auto got = proto::unpack_frame(f);
        CHECK(got.has_value());
        if (got) {
            CHECK_EQ(int(got->cmd), 0x43);
            CHECK(got->payload == payload);
        }
    }
}

TEST(protocol_unpack_frame_rejects_bad_input) {
    const auto good = proto::pack_frame(0x10, std::vector<uint8_t>{1, 2, 3});
    // 短于最小帧长
    CHECK(!proto::unpack_frame(std::vector<uint8_t>{0xA5, 0x10, 0x00}).has_value());
    // SOF 不对
    auto bad_sof = good;
    bad_sof[0] = 0x00;
    CHECK(!proto::unpack_frame(bad_sof).has_value());
    // 长度与实际不符
    auto bad_len = good;
    bad_len[2] = 0x09;
    CHECK(!proto::unpack_frame(bad_len).has_value());
    // CRC 坏
    auto bad_crc = good;
    bad_crc.back() ^= 0xFF;
    CHECK(!proto::unpack_frame(bad_crc).has_value());
    // 尾部多一个字节 (长度就不匹配了)
    auto extra = good;
    extra.push_back(0x00);
    CHECK(!proto::unpack_frame(extra).has_value());
}

// ---------------------------------------------------------------- f32 载荷

TEST(protocol_f32_pack_unpack_roundtrip) {
    const std::vector<double> vals{0.0, 1.5, -2.25, 3.14159265, 1e6, -1e-6, 123.456};
    const auto b = proto::pack_f32s(vals);
    CHECK_EQ(b.size(), vals.size() * 4);
    const auto back = proto::unpack_f32s(b.data(), 0, int(vals.size()));
    for (size_t i = 0; i < vals.size(); ++i) {
        CHECK_NEAR(back[i], vals[i], 1e-4);
    }
}

TEST(protocol_u16_u32_are_little_endian) {
    const auto u16 = proto::pack_u16le(0x2FE);
    CHECK_EQ(int(u16[0]), 0xFE);
    CHECK_EQ(int(u16[1]), 0x02);
    CHECK_EQ(int(proto::read_u16le(u16, 0)), 0x2FE);
    const auto u32 = proto::pack_u32le(0xDEADBEEF);
    CHECK_EQ(int(u32[0]), 0xEF);
    CHECK_EQ(int(u32[3]), 0xDE);
    CHECK_EQ(proto::read_u32le(u32, 0), uint32_t(0xDEADBEEF));
}

// ---------------------------------------------------------------- 版本解析

TEST(protocol_parse_firmware_version_accepts_the_agreed_shape) {
    const auto v = proto::parse_firmware_version("Litearm1.5.2-7J");
    CHECK(v.has_value());
    CHECK_EQ(v->major, 1);
    CHECK_EQ(v->minor, 5);
    CHECK_EQ(v->patch, 2);
    CHECK_EQ(v->variant, std::string("7J"));

    const auto bench = proto::parse_firmware_version("Litearm1.7.0-1J");
    CHECK(bench.has_value());
    CHECK_EQ(bench->variant, std::string("1J"));

    // 带空白也要能解析 (固件回显可能带尾随字符)
    CHECK(proto::parse_firmware_version("  Litearm1.8.0-7J  ").has_value());
    // 无 -variant 也算合法 (variant 空)
    const auto no_variant = proto::parse_firmware_version("Litearm1.5.0");
    CHECK(no_variant.has_value());
    CHECK_EQ(no_variant->variant, std::string(""));
}

TEST(protocol_parse_firmware_version_rejects_legacy_and_malformed) {
    CHECK(!proto::parse_firmware_version("A1.2.3-USB").has_value());   // 旧形态
    CHECK(!proto::parse_firmware_version("Litearm1.5").has_value());   // 两段
    CHECK(!proto::parse_firmware_version("Litearm1.5.2.3-7J").has_value());
    CHECK(!proto::parse_firmware_version("Litearm1.x.2-7J").has_value());
    CHECK(!proto::parse_firmware_version("").has_value());
    CHECK(!proto::parse_firmware_version("garbage").has_value());
}

TEST(protocol_version_ordering_ignores_variant) {
    const auto a = proto::parse_firmware_version("Litearm1.5.0-7J");
    const auto b = proto::parse_firmware_version("Litearm1.5.1-7J");
    const auto c = proto::parse_firmware_version("Litearm1.6.0-1J");
    CHECK(*a < *b);
    CHECK(*b < *c);
    CHECK(*a <= *a);
    CHECK(*a == *a);
    CHECK(!(*c < *a));
}

// ---------------------------------------------------------------- 开机签名

TEST(protocol_boot_banner_normal) {
    const auto r = proto::parse_boot_banner("noise [litearm-usbcdc] ready (sig_ok=1) more");
    CHECK(r.has_value());
    CHECK_EQ(*r, std::string("normal"));
}

TEST(protocol_boot_banner_iwdg_reset) {
    const auto r =
        proto::parse_boot_banner("x [litearm-usbcdc] ready (sig_ok=1, iwdg-rst) y");
    CHECK(r.has_value());
    CHECK_EQ(*r, std::string("iwdg-rst"));
}

TEST(protocol_boot_banner_takes_the_last_one) {
    // 重连/多次枚举会重复出现, 取最后一次。
    const auto r = proto::parse_boot_banner(
        "[litearm-usbcdc] ready (a) ... [litearm-usbcdc] ready (b, iwdg-rst)");
    CHECK(r.has_value());
    CHECK_EQ(*r, std::string("iwdg-rst"));
}

TEST(protocol_boot_banner_absent) {
    CHECK(!proto::parse_boot_banner("").has_value());
    CHECK(!proto::parse_boot_banner("just some noise").has_value());
}

// ---------------------------------------------------------------- 状态帧布局

TEST(protocol_decode_status_new_layout_with_joint_fault) {
    // 6+21N: 固件 >= 1.5.0。7J => 153B。
    const std::vector<double> q{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    const auto payload = lt::make_status_payload(0x0000, 42, q, 0x0008);
    CHECK_EQ(payload.size(), size_t(6 + 21 * 7));
    const auto d = proto::decode_status(payload);
    CHECK(d.has_value());
    CHECK_EQ(int(d->seq), 42);
    CHECK_EQ(int(d->joints.size()), 7);
    CHECK_EQ(int(d->joint_fault), 0x0008);
    CHECK_NEAR(d->joints[3].q, 0.4, 1e-6);
}

TEST(protocol_decode_status_legacy_layout_without_joint_fault) {
    // 4+21N: 固件 <= 1.4.x。尾部没有 joint_fault, 视为 0。
    const std::vector<double> q{1.0};
    std::vector<uint8_t> body;
    const auto h = proto::pack_u16le(0);
    body.insert(body.end(), h.begin(), h.end());
    const auto s = proto::pack_u16le(7);
    body.insert(body.end(), s.begin(), s.end());
    const auto v = proto::pack_f32s(std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0});
    body.insert(body.end(), v.begin(), v.end());
    body.push_back(0);
    CHECK_EQ(body.size(), size_t(4 + 21));
    const auto d = proto::decode_status(body);
    CHECK(d.has_value());
    CHECK_EQ(int(d->joints.size()), 1);
    CHECK_EQ(int(d->seq), 7);
    CHECK_EQ(int(d->joint_fault), 0);
}

TEST(protocol_decode_status_rejects_illegal_shapes) {
    CHECK(!proto::decode_status(std::vector<uint8_t>{1, 2}).has_value());   // < 4B
    // 关节数不是 1 也不是 7
    std::vector<uint8_t> three(4 + 21 * 3, 0);
    CHECK(!proto::decode_status(three).has_value());
    // 尾部多出 1 个字节 (既不是 0 也不是 2)
    std::vector<uint8_t> bad_tail(4 + 21 * 7 + 1, 0);
    CHECK(!proto::decode_status(bad_tail).has_value());
}

TEST(protocol_decode_status_splits_mode_and_flags) {
    // flags 低 6 位是安全 flag, bit6..8 是 mode, bit9 enabled, bit10 cart_busy。
    const uint16_t flags = uint16_t(0x0001 | (3 << 6) | (1 << 9) | (1 << 10));
    const std::vector<double> q(7, 0.0);
    const auto d = proto::decode_status(lt::make_status_payload(flags, 0, q));
    CHECK(d.has_value());
    CHECK_EQ(d->mode, 3);
    CHECK_EQ(int(d->flag_names.size()), 1);   // 只有 FAULT, mode/enabled/cart_busy 不算
    CHECK_EQ(std::string(d->flag_names[0]), std::string("FAULT"));
    CHECK_EQ(std::string(proto::mode_name(3)), std::string("MOVE_JS"));
}

TEST(protocol_mode_name_covers_all_and_unknown) {
    CHECK_EQ(std::string(proto::mode_name(0)), std::string("INIT"));
    CHECK_EQ(std::string(proto::mode_name(7)), std::string("ZERO_G"));
    CHECK_EQ(std::string(proto::mode_name(99)), std::string("UNKNOWN"));
}

TEST(protocol_flag_name_only_covers_the_six_safety_bits) {
    CHECK_EQ(std::string(proto::flag_name(0)), std::string("FAULT"));
    CHECK_EQ(std::string(proto::flag_name(5)), std::string("OVERSPEED"));
    CHECK_EQ(std::string(proto::flag_name(6)), std::string("UNKNOWN"));   // bit6 起是 mode
    CHECK_EQ(std::string(proto::flag_name(9)), std::string("UNKNOWN"));   // 是 enabled
}

// ---------------------------------------------------------------- 归属判据

TEST(protocol_only_ack_and_err_are_keyed_by_echo) {
    CHECK(proto::is_echoed(proto::RSP_ACK));
    CHECK(proto::is_echoed(proto::RSP_ERR));
    // 别的帧的 payload[0] 是**数据**, 拿它当归属键是错的
    CHECK(!proto::is_echoed(proto::RSP_STATUS));
    CHECK(!proto::is_echoed(proto::RSP_JOINT_PARAM));
    CHECK(!proto::is_echoed(proto::RSP_LICENSE));
}

// ---------------------------------------------------------------- 覆盖契约

TEST(protocol_command_coverage_matches_the_python_table_exactly) {
    // 判据是**逐条对表**, 不是"数量差不多": 这份契约是"固件每条已实现命令都有 SDK 入口"
    // 的断言载体, 少一条就是少一个入口。
    // ⚠ 数量是 49 —— 上游注释里那句"47/47"是更早一轮的记数, 表本身一直是 49 条。
    const std::set<uint8_t> expected = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x10, 0x11, 0x12,
        0x13, 0x14, 0x15, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
        0x27, 0x28, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B,
        0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x49,
    };
    std::set<uint8_t> seen;
    for (const auto& c : proto::command_coverage()) {
        CHECK(seen.insert(c.cmd).second);          // 无重复命令码
        CHECK(c.entry != nullptr && c.entry[0] != '\0');
    }
    CHECK_EQ(int(seen.size()), int(expected.size()));
    CHECK(seen == expected);
    // 0x16 在固件里不存在 (0x15 与 0x20 之间没有命令), 不该被凭空补进来。
    CHECK(seen.count(0x16) == 0);
}

TEST(protocol_sync_sentinel_tables_are_all_empty) {
    // 四张表的语义**各不相同** (别混), 而它们**当前都是空的** —— 留空是有意的:
    // 它们是"当前没有未实现命令 / 没有固件单边命令 / 没有已知缺口"这句话的断言载体。
    // 往任何一张里加条目都等于宣布一件事, 加之前先想清楚 (见 protocol.hpp 的说明)。
    CHECK(proto::unimplemented_cmds().empty());
    CHECK(proto::firmware_only_cmds().empty());
    CHECK(proto::firmware_only_rsps().empty());
    CHECK(proto::preexisting_gaps().empty());

    // ⚠ **本仓跑不了固件侧的比对**: 上游 `test_protocol_sync.py` 要解析固件仓的
    //   `hal/usb_cmd.h` 双向对表, 而这里没有固件树。所以**别把这条读成"已经跟固件对过账"**
    //   —— 它只保证"我们这边没有单方面豁免任何东西"。
    //   要补上固件侧的那一半: 把固件仓放到旁边, 解析 usb_cmd.h 的 `#define CMD_*` /
    //   `#define RSP_*` 名字与值, 与本文件的常量表双向比对。
}

TEST(protocol_rsp_of_cmd_table_is_consistent) {
    for (const auto& kv : proto::rsp_of_cmd()) {
        const auto got = proto::rsp_of(kv.first);
        CHECK(got.has_value());
        if (got) CHECK_EQ(int(*got), int(kv.second));
    }
    // ACK/ERR 两条由命令码自己就能推出来, 故不在表里
    CHECK(!proto::rsp_of(proto::CMD_MOVE_J).has_value());
    // GET_STATUS 的应答走单槽不是队列, 故也不在
    CHECK(!proto::rsp_of(proto::CMD_GET_STATUS).has_value());
    CHECK_EQ(int(*proto::rsp_of(proto::CMD_GET_JOINT_PARAM)), int(proto::RSP_JOINT_PARAM));
    CHECK_EQ(int(*proto::rsp_of(proto::CMD_GET_JOINT_PARAM)), int(proto::RSP_JOINT_PARAM));
}

TEST(protocol_dual_id_constants_are_intentional) {
    // 0x49: 下行 CMD_KIN_BENCH 与上行 RSP_JOINT_PARAM 同值 —— 靠收发方向区分。
    CHECK_EQ(int(proto::CMD_KIN_BENCH), int(proto::RSP_JOINT_PARAM));
    // 0x41: 下行 CMD_GET_FIRMWARE 与上行 RSP_DETAIL 同值。
    CHECK_EQ(int(proto::CMD_GET_FIRMWARE), int(proto::RSP_DETAIL));
}
