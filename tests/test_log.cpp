// 300Hz 控制拍采集 —— 0x2D / 0x2E。
#include "test_support.hpp"

using namespace litearm;

TEST(log_sample_size_is_four_plus_twelve_n) {
    CHECK_EQ(sample_size(7), 4 + 12 * 7);
    CHECK_EQ(sample_size(1), 4 + 12);
    CHECK_EQ(sample_size(0), 4);
}

TEST(log_parse_samples_decodes_the_firmware_layout) {
    const auto blob = testing::make_log(5, 7);
    CHECK_EQ(blob.size(), size_t(5 * sample_size(7)));
    const auto samples = parse_samples(blob, 7);
    CHECK_EQ(int(samples.size()), 5);
    for (int i = 0; i < 5; ++i) {
        const auto& s = samples[size_t(i)];
        CHECK_EQ(int(s.tick), i);
        CHECK_EQ(int(s.q_ref.size()), 7);
        CHECK_NEAR(s.q_ref[0], 1.0, 1e-6);
        CHECK_NEAR(s.q_ref[6], 61.0, 1e-6);
        CHECK_NEAR(s.dq[0], 0.5, 1e-6);
        CHECK_NEAR(s.tau[0], -0.25, 1e-6);
    }
}

TEST(log_parse_samples_refuses_a_truncated_blob) {
    // 数据截断/错位时拒绝解析出半截样本 —— 不静默。
    auto blob = testing::make_log(3, 7);
    blob.resize(blob.size() - 1);
    CHECK_THROWS_AS(parse_samples(blob, 7), TransportError);
    CHECK_THROWS_AS(parse_samples(blob, 0), TransportError);
}

TEST(log_start_stop_and_read_back_immediately) {
    lt::Offline off;
    CHECK_NOTHROW(off->log().start(10));
    CHECK_EQ(off.t().log_target, 10);
    auto reader = off->log().reader();
    CHECK_EQ(int(reader.total()), 10 * sample_size(7));
    const auto samples = reader.samples();
    CHECK_EQ(int(samples.size()), 10);
    const auto blob = reader.read_all();
    CHECK_EQ(blob.size(), size_t(10 * sample_size(7)));
    CHECK_NOTHROW(off->log().stop());
    CHECK_EQ(off.t().log_target, 0);
}

TEST(log_start_rejects_negative_and_over_capacity) {
    lt::Offline off;
    CHECK_THROWS_AS(off->log().start(-1), InvalidCommandError);
    try {
        off->log().start(LOG_MAX_SAMPLES + 1);
        FAIL("应当被拒");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x2D);
        CHECK_EQ(int(e.code), 0x02);
    }
}

TEST(log_start_zero_is_equivalent_to_stop) {
    lt::Offline off;
    off->log().start(5);
    off->log().start(0);
    CHECK_EQ(off.t().log_target, 0);
    CHECK(!off.t().log_active);
}

TEST(log_reader_retries_on_a_dropped_reply_at_the_same_cursor) {
    // 固件是"绕开连续 USB 流、按游标续读"的设计, 所以重试/续读语义是必须的, 不是可选优化。
    lt::Offline off;
    off->log().start(6);
    off.t().log_read_fail_once = true;
    auto reader = off->log().reader(0.3, 3);
    const auto samples = reader.samples();
    CHECK_EQ(int(samples.size()), 6);   // 掉一帧不该丢一段数据
}

TEST(log_reader_refuses_to_loop_forever_on_a_stuck_cursor) {
    // 固件游标若因异常/干扰回了一个不大于当前 offset 的非零值, 天真的实现会在同一位置无限
    // 次重读 —— 必须自己判定"无进展"并报错。
    lt::Offline off;
    off->log().start(100);
    off.t().log_cursor_stuck = true;
    auto reader = off->log().reader(0.2, 0);
    CHECK_THROWS_AS(reader.read_all(), TransportError);
}

TEST(log_wait_for_polls_until_the_firmware_has_recorded_enough) {
    // 固件是 300Hz **逐拍**记录的: start() 返回时缓冲里还没有数据 —— 等待必须靠轮询,
    // 不能靠 sleep 猜时间。
    lt::Offline off;
    off.t().log_hz = 300.0;     // 打开逐拍模型
    off->log().start(30);       // 30 拍约 0.1s
    auto reader = off->log().reader(1.0, 3);
    const uint32_t total = reader.wait_for(30, 2.0, 0.01);
    CHECK(total >= uint32_t(30 * sample_size(7)));
    const auto samples = reader.samples();
    CHECK(int(samples.size()) >= 30);
}

TEST(log_wait_for_raises_instead_of_returning_half_data) {
    lt::Offline off;
    off.t().log_hz = 30.0;      // 很慢
    off->log().start(2400);     // 要 80 秒才录得满
    auto reader = off->log().reader(0.5, 1);
    const double t0 = now_s();
    CHECK_THROWS_AS(reader.wait_for(2400, 0.2, 0.01), MotionTimeoutError);
    CHECK(now_s() - t0 >= 0.15);
}

TEST(log_capture_waits_then_returns_samples) {
    lt::Offline off;
    off.t().log_hz = 300.0;
    const auto samples = off->log().capture(20, 1.0, 3, 2.0);
    CHECK(int(samples.size()) >= 20);
}

TEST(log_dump_writes_the_raw_byte_stream) {
    lt::Offline off;
    off->log().start(8);
    // ⚠ 用**相对路径**: 本仓要能开源、在任何人的机器上跑, 不许把 /tmp 或开发者家目录写死。
    //    ctest 的工作目录就是构建目录, 直接跑二进制时就是当前目录; `.gitignore` 已覆盖 *.bin。
    const std::string path = "litearm_test_dump.bin";
    const size_t n = off->log().dump(path, /*wait=*/false, 0.5, 3, -1.0);
    CHECK_EQ(int(n), 8 * sample_size(7));
    FILE* f = std::fopen(path.c_str(), "rb");
    CHECK(f != nullptr);
    if (f) {
        std::vector<uint8_t> got(n);
        const size_t rd = std::fread(got.data(), 1, n, f);
        std::fclose(f);
        CHECK_EQ(int(rd), int(n));
        // 落盘的字节就是固件给的原样字节流
        CHECK_EQ(int(parse_samples(got, 7).size()), 8);
    }
    std::remove(path.c_str());
}

TEST(log_dump_wait_uses_the_last_start_target) {
    lt::Offline off;
    off.t().log_hz = 300.0;
    off->log().start(10);
    CHECK_EQ(off->log().last_target(), 10);
    const std::string path = "litearm_test_dump2.bin";
    const size_t n = off->log().dump(path, /*wait=*/true, 1.0, 3, 2.0);
    CHECK(int(n) >= 10 * sample_size(7));
    std::remove(path.c_str());
}

TEST(log_read_reply_short_is_rejected) {
    // ⚠ 用桩的确定注入点而不是手工 push_frame —— 后者造出来的帧会与读线程抢时序
    //   (它可能先被读进队列, 再由 drain_for 清掉), 用例会偶发红。
    lt::Offline off;
    off->log().start(3);
    off.t().log_reply_short = true;
    auto reader = off->log().reader(0.2, 0);
    CHECK_THROWS_AS(reader.read_all(), TransportError);
    // 关掉注入之后照样读得回来
    off.t().log_reply_short = false;
    CHECK_EQ(int(reader.samples().size()), 3);
}

TEST(log_read_unsupported_command_is_reported) {
    lt::Offline off;
    off.t().unknown_cmds.insert(proto::CMD_LOG_READ);
    auto reader = off->log().reader(0.2, 0);
    CHECK_THROWS_AS(reader.total(), UnsupportedByFirmwareError);
}

TEST(log_iter_chunks_preserves_chunk_boundaries) {
    // 上游 `LogReader.iter_chunks()` 的 C++ 等价物 (那边是生成器, 这边是块向量)。
    // 判据与 `read_all` 的一致性 + **块边界确实是固件的分块** (245B 一块)。
    lt::Offline off;
    off->log().start(50);
    auto reader = off->log().reader();
    const auto chunks = reader.iter_chunks();
    CHECK(int(chunks.size()) >= 2);              // 50 拍 x88B = 4400B => 至少要两块
    size_t total = 0;
    for (size_t i = 0; i + 1 < chunks.size(); ++i) {
        CHECK_EQ(int(chunks[i].size()), LOG_READ_CHUNK);   // 非末块满 245B
        total += chunks[i].size();
    }
    total += chunks.back().size();
    CHECK_EQ(int(total), 50 * sample_size(7));
    // 与 read_all 拼接结果逐字节相同
    auto reader2 = off->log().reader();
    const auto flat = reader2.read_all();
    std::vector<uint8_t> joined;
    for (const auto& c : chunks) joined.insert(joined.end(), c.begin(), c.end());
    CHECK(joined == flat);
}

TEST(log_iter_chunks_refuses_to_loop_forever_on_a_stuck_cursor) {
    lt::Offline off;
    off->log().start(100);
    off.t().log_cursor_stuck = true;
    auto reader = off->log().reader(0.2, 0);
    CHECK_THROWS_AS(reader.iter_chunks(), TransportError);
}

TEST(log_bulk_capture_round_trips_every_sample) {
    // 245B 一块 => 7J 的一拍 88B 会跨块 —— 跨块拼接必须字节一致。
    lt::Offline off;
    off->log().start(50);
    auto reader = off->log().reader();
    const auto samples = reader.samples();
    CHECK_EQ(int(samples.size()), 50);
    for (int i = 0; i < 50; ++i) CHECK_EQ(int(samples[size_t(i)].tick), i);
    CHECK_EQ(int(reader.total_bytes()), 50 * sample_size(7));
}
