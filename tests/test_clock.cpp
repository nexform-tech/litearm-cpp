// 可注入时钟 (`ArmOptions::clock`)。
//
// ⚠⚠ **本仓的时钟只管"现在几点", 不管"怎么等"** —— 先读 `include/litearm/clock.hpp`
//   开头那段。要旨: 等待走条件变量的**真实**时间, 只有 deadline/时间戳走注入的钟
//   ⇒ 一个**不会自己前进**的假钟会让"等超时真的发生"的判据**挂死**。
//   本文件里所有用假钟的用例都只碰**纯比较**类判据, 一条等待路径都不走。
#include "test_support.hpp"

using namespace litearm;

// ---------------------------------------------------------------- 默认 = 真钟

TEST(clock_default_is_steady_so_timestamps_are_real) {
    // 不注入 ⇒ 进程级 `SteadyClock`。
    //
    // ⚠⚠ 判据**不能**写成"时间戳 > 某个常数" —— `steady_clock` 的 epoch 是**开机时刻**,
    //   那个常数就变成了"机器开机时长必须超过多少", 于是刚开机的机器上**必红**
    //   (我第一版写的 `> 1e4` 就是这种: 开机不足 2.8 小时的机器上直接失败)。
    //   正解: 与**同一个时钟的当前值**比 —— 这才是"时间戳来自这个钟"的直接表述,
    //   与开机多久无关。
    lt::Offline off;
    const auto m = off->get_ff_scalar(1, 0);
    CHECK_NEAR(m.timestamp, steady_clock_instance().now_s(), 30.0);
    // ⚠ 顺带守住另一个真陷阱: 若默认钟被误换成"从 0 起"的实现, 上面那条立刻红
    //   (0 与 steady 的当前值差着整个开机时长)。
}

TEST(clock_injection_is_visible_through_options) {
    auto clk = testing::FakeClock::make();
    lt::Offline off("Litearm1.7.0-7J", 7, clk);
    CHECK(off->options().clock == clk);
}

TEST(clock_connect_works_under_an_injected_clock) {
    // 注入假钟**不许**破坏连接流程: 握手/建软限缓存都要照常走完。
    // (它们之所以能走完, 是因为**应答真的会到** —— 假件的应答是立刻的。
    //  若应答不到, `expect` 会等满 timeout; 而 timeout 在假钟下永远到不了 ⇒ 挂死。
    //  这正是 clock.hpp 里那条警告的具体形状。)
    auto clk = testing::FakeClock::make();
    lt::Offline off("Litearm1.7.0-7J", 7, clk);
    CHECK_EQ(off->n(), 7);
    CHECK_EQ(off->firmware(), std::string("Litearm1.7.0-7J"));
    CHECK(off->is_connected());
}

// ---------------------------------------------------------------- 假钟的最大价值

TEST(clock_fake_clock_makes_the_staleness_judgement_instant_and_deterministic) {
    // ★★ 这条是假钟**最大的价值所在**。
    //
    // `Arm::is_connected()` 的第二条判据是"状态帧超龄"(`STATUS_STALE_MAX_S` = 2.0s)。
    // 用真钟验它要**真睡满 2 秒**才能翻到 false —— 一条用例 2 秒, 而且是对调度器的断言。
    // 用假钟: 时间由用例**推**, 瞬间到位, 且断言的因果是干净的 (推 3 秒 ⇒ 越阈值)。
    auto clk = testing::FakeClock::make();
    lt::Offline off("Litearm1.7.0-7J", 7, clk);
    // ⚠ **必须先把被动流停掉**: 假件每 1ms 还在投递状态帧, 而投递会**刷新时间戳**
    //   (用当前假钟的值) ⇒ 不停流的话推完时间立刻又被刷新, 判据永远翻不到 false。
    lt::quiesce(off.t());

    CHECK(off->is_connected());
    clk->advance(1.0);
    CHECK(off->is_connected());     // 1.0s < 2.0s: 还在阈值内
    clk->advance(2.0);              // 累计 3.0s
    CHECK(!off->is_connected());    // 越过阈值 ⇒ 判失联
    // ⚠ 判据是"**纯粹由时钟推进触发**"的: 全程没有真睡, 也没有任何一帧新数据。
}

TEST(clock_status_seq_is_not_affected_by_the_clock) {
    // 状态帧计数是**到达**的度量, 与时间源无关 —— 别让注入时钟顺带改了它。
    auto clk = testing::FakeClock::make();
    lt::Offline off("Litearm1.7.0-7J", 7, clk);
    const uint64_t a = off->status_seq();
    clk->advance(100.0);            // 推 100 秒: 一帧都不会因此多出来
    const uint64_t b = off->status_seq();
    CHECK(b >= a);
    CHECK(b - a < 100);             // 推时间不产帧
}

// ---------------------------------------------------------------- 假钟自身的契约

TEST(clock_fake_clock_starts_non_zero_and_advances_monotonically) {
    // ⚠ 起始值**刻意非 0**: 本仓多处用 `timestamp <= 0` 表示"从没收到过"
    //   (如 `Arm::is_connected` 里那句 `recv_stats(...).second <= 0.0`)。
    //   假钟若从 0 起, 那类判据会在"明明收到过"时判假 ⇒ 用例测的是一个现实中不存在的状态。
    auto clk = testing::FakeClock::make();
    CHECK(clk->now_s() > 0.0);
    const double a = clk->now_s();
    clk->advance(2.5);
    CHECK_NEAR(clk->now_s(), a + 2.5, 1e-12);
    clk->advance(0.0);              // 推进 0 是合法的 no-op
    CHECK_NEAR(clk->now_s(), a + 2.5, 1e-12);
}
