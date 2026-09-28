// 固件自检 —— KIN_BENCH 文本解析与两帧收齐。
#include "test_support.hpp"

using namespace litearm;

TEST(diag_parse_timings_handles_names_glued_to_digits) {
    // 固件用 txt(名字) + u32_cat(n) 拼接, 而 u32_cat 只在数字之后补空格 —— 所以名字与
    // 第一个数字是相连的 (`FK200 108 240 `), 不能用「名字 + 空白 + 数字」式正则去切。
    const auto r = parse_kin_bench(testing::KIN_BENCH_TEXT);
    CHECK_EQ(int(r.timings.size()), 9);
    const auto& fk = r.timings.at("FK");
    CHECK_EQ(int(fk.size()), 3);
    CHECK_EQ(int(fk[0]), 200);
    CHECK_EQ(int(fk[1]), 108);
    CHECK_EQ(int(fk[2]), 240);
    // LOOP1 是 (cycles,) —— `LOOP14000` 里名字与数字相连
    CHECK_EQ(int(r.timings.at("LOOP1").size()), 1);
    CHECK_EQ(int(r.timings.at("LOOP1")[0]), 4000);
    const auto& ik = r.timings.at("IK");
    CHECK_EQ(int(ik[0]), 100);
    CHECK_EQ(int(ik[1]), 2600);
    CHECK_EQ(int(ik[2]), 5200);
}

TEST(diag_parse_link_reads_every_counter) {
    const auto r = parse_kin_bench(testing::KIN_BENCH_TEXT);
    CHECK_EQ(int(r.link.at("crc")), 3);
    CHECK_EQ(int(r.link.at("ovf")), 11);
    CHECK_EQ(int(r.link.at("rfd")), 1);
    CHECK_EQ(int(r.link.at("txf")), 7);
    CHECK_EQ(int(r.link.at("loop_k")), 2);
    CHECK_EQ(int(r.link.at("ovr")), 4);
    // flt=<锁存拍> /<原因码> —— 数字与 '/' 之间有一个尾随空格 (u32_cat 语义所致)
    CHECK_EQ(int(r.link.at("fault_tick")), 12345);
    CHECK_EQ(int(r.link.at("fault_cause")), 6);
    // st 是语义化的布尔: 1 = 从 flash 装载
    CHECK_EQ(int(r.link.at("store_from_flash")), 1);
}

TEST(diag_parse_link_reads_the_digit_bearing_keys) {
    // ⚠ rxl0 / rxl1 **名字里带数字** —— 从前写的是 [a-z_]+, 匹配不到它们, 于是那两个
    // "静默丢反馈"的计数器整个丢失。这是同一个文件里第二次栽在"名字里带数字"上。
    const auto r = parse_kin_bench(testing::KIN_BENCH_TEXT);
    CHECK_EQ(int(r.rx_fifo_lost_motor()), 13);     // rxl0
    CHECK_EQ(int(r.rx_fifo_lost_bridge()), 17);    // rxl1
    CHECK_EQ(int(r.gsusb_ring_drops()), 19);       // rbd
    CHECK(r.link.count("rxl0") != 0);
    CHECK(r.link.count("rxl1") != 0);
}

TEST(diag_convenience_getters_read_zero_only_when_the_key_is_absent) {
    // 拿不到就是静默的 0, 与"没有出错"分不开 —— 这正是"只收一帧必然丢掉全部链路计数"
    // 那条纪律的由来。
    KinBenchResult empty;
    CHECK_EQ(int(empty.crc_errors()), 0);
    CHECK_EQ(int(empty.reply_dropped()), 0);
    CHECK_EQ(int(empty.can_tx_fail()), 0);
    CHECK_EQ(int(empty.loop_max_kcycle()), 0);
    CHECK_EQ(int(empty.loop_overruns()), 0);
    const auto r = parse_kin_bench(testing::KIN_BENCH_TEXT);
    CHECK_EQ(int(r.crc_errors()), 3);
    CHECK_EQ(int(r.reply_dropped()), 1);
    CHECK_EQ(int(r.can_tx_fail()), 7);
    CHECK_EQ(int(r.loop_max_kcycle()), 2);
    CHECK_EQ(int(r.loop_overruns()), 4);
}

TEST(diag_empty_reply_is_an_error_not_all_zeros) {
    CHECK_THROWS_AS(parse_kin_bench(""), TransportError);
    CHECK_THROWS_AS(parse_kin_bench("   \n  \n"), TransportError);
}

TEST(diag_result_keeps_the_raw_text) {
    const auto r = parse_kin_bench(testing::KIN_BENCH_TEXT);
    CHECK(r.raw == std::string(testing::KIN_BENCH_TEXT));
    CHECK(r.str() == r.raw);
}

TEST(diag_kin_bench_collects_both_frames) {
    // ⚠ 只收第 1 帧会**静默**丢掉全部链路计数 (那几个 link.get(k,0) 出口会全报 0,
    // 与"没有出错"分不开)。
    lt::Offline off;
    const auto m = off->diag().kin_bench();
    CHECK_EQ(int(m.value.timings.size()), 9);
    CHECK(m.value.link.count("crc") != 0);          // 第 2 帧确实收到了
    CHECK_EQ(int(m.value.crc_errors()), 3);
    CHECK_EQ(int(m.value.rx_fifo_lost_motor()), 13);   // 带数字的键也在
}

TEST(diag_kin_bench_is_compatible_with_old_firmware_that_sends_one_frame) {
    // 旧固件只发一帧, 靠短窗口超时退出 —— 那是兼容情形, 不是错误。
    lt::Offline off;
    off.t().kin_bench_one_frame = true;
    const auto m = off->diag().kin_bench();
    CHECK_EQ(int(m.value.timings.size()), 9);
    CHECK(m.value.link.empty());     // 链路计数确实拿不到 (旧固件根本不发)
}

TEST(diag_kin_bench_empty_reply_is_rejected) {
    lt::Offline off;
    off.t().kin_bench_text = "";
    CHECK_THROWS_AS(off->diag().kin_bench(), TransportError);
}

TEST(diag_kin_bench_timeout_is_configurable) {
    lt::Offline off;
    CHECK_NOTHROW(off->diag().kin_bench(2.0));
}

TEST(diag_split_kin_bench_mirrors_the_firmware_framing) {
    // 拆点取 LINK 行首 —— 固件是"重置缓冲后从 `LINK ` 重新写", 故第 2 帧以 LINK 开头。
    const auto parts = testing::split_kin_bench(testing::KIN_BENCH_TEXT);
    CHECK_EQ(int(parts.size()), 2);
    CHECK(parts[1].rfind("LINK", 0) == 0);
    CHECK(parts[0].find("LINK") == std::string::npos);
    // 不含 LINK 的文本仍回一帧
    CHECK_EQ(int(testing::split_kin_bench("FK200 ").size()), 1);
    // one_frame 只给第 1 帧
    CHECK_EQ(int(testing::split_kin_bench(testing::KIN_BENCH_TEXT, true).size()), 1);
}
