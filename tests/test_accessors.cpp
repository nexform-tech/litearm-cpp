// 只读访问器 —— ⚠ 这七个**全是与 Python 参照分叉**的加法 (参照侧一个都没有)。
//
// 它们的共同契约 (也是本文件要钉的东西):
//   * **纯本地快照**: 不发帧、不改状态;
//   * **终态下也不抛** —— 现场最需要它们的时候, 正是链路已经废掉的时候。
#include "test_support.hpp"

using namespace litearm;

// ---------------------------------------------------------------- is_connected

TEST(accessor_is_connected_reflects_the_link_state) {
    lt::Offline off;
    CHECK(off->is_connected());
    off->close();
    CHECK(!off->is_connected());
}

TEST(accessor_is_connected_turns_false_when_the_reader_dies) {
    // ⚠ 两个条件里的**第一条** (读线程死了)。这一支**不用等**超龄, 立刻翻转。
    lt::Offline off;
    CHECK(off->is_connected());
    off.t().dfu_gone = true;   // 桩的读路径开始抛 ⇒ reader_loop 走 die()
    const double end = now_s() + 1.0;
    while (off->is_connected() && now_s() < end) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(!off->is_connected());
}

// ⚠ 「状态帧超龄 ⇒ 判失联」那条分支**搬到 `tests/test_clock.cpp` 了**:
//   `clock_fake_clock_makes_the_staleness_judgement_instant_and_deterministic`。
//   原来它在**这里**, 靠 `sleep_for(2.1s)` 真睡过 `STATUS_STALE_MAX_S` —— 那条写法有两个
//   毛病: ① 一条用例 2 秒; ② 断言里混进了"调度器会不会按时醒来"这个无关变量。
//   搬到假钟下面之后: 时间由用例推, 瞬间到位, 因果干净。
//   ⚠ 这不是"把一个判据删了": 断言的性质**一模一样**(越过阈值 ⇒ 翻 false), 只是换了驱动。
//     而"默认时间源确实是真钟"由 `test_clock.cpp` 的
//     `clock_default_is_steady_so_timestamps_are_real` 单独守着。

// ---------------------------------------------------------------- status_seq / msg_hz

TEST(accessor_status_seq_and_msg_hz_track_the_stream) {
    lt::Offline off;
    const uint64_t s1 = off->status_seq();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const uint64_t s2 = off->status_seq();
    CHECK(s2 > s1);
    // 桩的被动流是 1ms 一拍 ⇒ 频率远高于 100Hz。⚠ 只判"确实在报频率", 不钉具体值
    //   (钉具体值就是把这条用例变成对调度器速度的断言)。
    CHECK(off->msg_hz(proto::RSP_STATUS) > 100.0);
    // 从没出现过的上行 id ⇒ 0
    CHECK_NEAR(off->msg_hz(0x7F), 0.0, 1e-12);
}

// ---------------------------------------------------------------- banner_version

TEST(accessor_banner_version_requires_a_real_version_string) {
    lt::Offline off;
    CHECK(!off->banner_version().has_value());   // 桩默认没有噪声文本

    off.t().set_text_log("noise [litearm-usbcdc] ready (sig_ok=1) Litearm1.8.0-7J");
    const auto v = off->banner_version();
    CHECK(v.has_value());
    if (v) CHECK_EQ(*v, std::string("Litearm1.8.0-7J"));

    // ⚠ 小写的 `[litearm-usbcdc]` **不许**被当成版本锚 (锚是大小写敏感的 "Litearm")
    off.t().set_text_log("only [litearm-usbcdc] ready, no version here");
    CHECK(!off->banner_version().has_value());

    // ⚠ 只找到前缀、后面没有合法版本号 ⇒ 也**不许**报出去。
    //   本函数的全部用处就是"告诉现场板子上跑的是哪一版", 报半截比不报更坏。
    off.t().set_text_log("Litearm");
    CHECK(!off->banner_version().has_value());
    off.t().set_text_log("Litearm1.5");
    CHECK(!off->banner_version().has_value());
}

// ---------------------------------------------------------------- host_stats

TEST(accessor_host_stats_starts_clean) {
    lt::Offline off;
    const auto s = off->host_stats();
    CHECK_EQ(s.dropped, uint64_t(0));
    CHECK_EQ(s.bad_status_frames, uint64_t(0));
    CHECK_EQ(s.flush_failures, uint64_t(0));
    CHECK_EQ(s.cart_evicted_unclaimed, uint64_t(0));
    CHECK_EQ(s.cart_extra_replies, uint64_t(0));
    CHECK(!s.cart_probe_silent);
}

TEST(accessor_host_stats_forwards_the_transport_counters) {
    // ⚠ 这条钉的是**转发链**: 假件注入的值必须能穿过测试夹具那层包装 (`SharedFake`)
    //   到达 `host_stats()`。它曾经**传不上来** —— `flush_failures()` 当时不是虚函数,
    //   包装层读的是自己那个恒 0 的计数器。包装层漏转发口 = 假绿, 见 test_support.hpp。
    lt::Offline off;
    off.t().set_flush_failures(42);
    CHECK_EQ(off->host_stats().flush_failures, uint64_t(42));
}

TEST(accessor_host_stats_counts_status_frames_that_pass_crc_but_fail_to_decode) {
    // ⚠ 这个计数存在的**唯一**理由: "每帧都解不出来" 与 "根本没有帧" 在等待者眼里
    //   是同一种静默, 而两者要查的方向相反 (前者查版本/格式错配, 后者查链路)。
    //   没有它, 现场没法把二者分开。
    lt::Offline off;
    // 3 字节的状态载荷: 帧层合法 (CRC 对), 但短于最小状态帧 ⇒ 解码必抛
    off.t().push_frame(proto::RSP_STATUS, {0x01, 0x02, 0x03});
    const double end = now_s() + 1.0;
    while (off->host_stats().bad_status_frames == 0 && now_s() < end) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK_EQ(off->host_stats().bad_status_frames, uint64_t(1));
}

// ---------------------------------------------------------------- options

TEST(accessor_options_reports_current_values_not_a_stale_snapshot) {
    // ⚠⚠ 这条钉的是**设计决定**: `options()` 返回**现值**, 不是构造期那份死快照。
    //   本仓的旋钮是公开成员、**运行期可改** (与 A 侧"只在构造期定"不同) ⇒ 返回死快照
    //   会让调用方读到过期值 —— 那正是本仓刚修掉的 `port_string()` 那类坑
    //   (注释承诺一套、代码做另一套)。见 arm.hpp 里 `options()` 的说明。
    lt::Offline off;
    off->move_timeout = 7.5;
    off->q_tol = 0.123;
    const auto o = off->options();
    CHECK_NEAR(o.move_timeout, 7.5, 1e-12);
    CHECK_NEAR(o.q_tol, 0.123, 1e-12);
    CHECK(o.port.has_value());
    if (o.port) CHECK_EQ(*o.port, std::string("fake"));
}

// ---------------------------------------------------------------- is_in_dfu

TEST(accessor_is_in_dfu_flips_on_enter_dfu) {
    lt::Offline off;
    CHECK(!off->is_in_dfu());
    off->enter_dfu();
    CHECK(off->is_in_dfu());
}

// ---------------------------------------------------------------- 纯本地: 终态下也不抛

TEST(accessor_local_snapshots_never_throw_even_in_the_terminal_state) {
    // ⚠ 这是这批访问器的**共同契约**: 终态下链路已经废了, 而那正是现场最想看一眼
    //   "到底发生了什么"的时候 —— 此时抛异常等于把诊断口自己关掉。
    lt::Offline off;
    off->enter_dfu();
    CHECK(off->is_in_dfu());
    CHECK_NOTHROW(off->is_connected());
    CHECK_NOTHROW(off->status_seq());
    CHECK_NOTHROW(off->msg_hz(proto::RSP_STATUS));
    CHECK_NOTHROW(off->banner_version());
    CHECK_NOTHROW(off->host_stats());
    CHECK_NOTHROW(off->options());
    CHECK_NOTHROW(off->port_string());
}
