// 零重力 (拖动示教) —— 保活线程 / 双向守卫 / 幂等启停。
#include "test_support.hpp"

using namespace litearm;

namespace {
int count_cmd(const testing::FakeTransport& t, uint8_t cmd) {
    int n = 0;
    for (const auto& kv : t.tx_snapshot()) {
        if (kv.first == cmd) ++n;
    }
    return n;
}
int count_zero_g_on(const testing::FakeTransport& t) {
    int n = 0;
    for (const auto& kv : t.tx_snapshot()) {
        if (kv.first == proto::CMD_ZERO_G && kv.second.size() == 1 &&
            kv.second[0] == 0x01) {
            ++n;
        }
    }
    return n;
}

/// 等到 `count_zero_g_on(t) >= want` 或超时。返回**实际**看到的条数。
///
/// ⚠ 为什么不用"睡固定时长再看一眼": 那是在断言**调度器的快慢**, 不是在断言保活。
///   实测 (2026-09-28, TSan 下): `sleep(80ms)` + 断言 `>= 3` 会以约 1/6 的概率翻红 ——
///   TSan 让每条锁操作慢一个数量级, 保活线程被饿到只跑了 2 拍。而"保活会不会周期重发"
///   这件事本身**没问题**; 翻红的是判据, 不是被测行为。
///   改成"等到出现为止(有上界)"之后: 保活真坏了就一定超时失败, 机器慢只是多等一会儿。
int wait_for_zero_g_on(const testing::FakeTransport& t, int want, double timeout_s) {
    const double end = now_s() + timeout_s;
    while (true) {
        const int n = count_zero_g_on(t);
        if (n >= want) return n;
        if (now_s() >= end) return n;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

/// 等到 `pred()` 为真, 或超时。返回最终是否成立。
///
/// ⚠ 本文件里凡是"后台保活线程该做某事了"的判据都用它, **不要**写
///   `sleep_for(80ms); CHECK(...)`: 那断言的是**调度器有多快**, 而 TSan/ASan 下保活线程
///   会被饿到在 80ms 里跑不了几拍 (实测 2026-09-28: `test_zero_g` 与 `test_threading`
///   各有一条用例因此翻红)。上界给足之后, 保活真坏了照样失败, 机器慢只是多等一会儿。
template <typename Pred>
bool wait_until(Pred pred, double timeout_s) {
    const double end = now_s() + timeout_s;
    while (true) {
        if (pred()) return true;
        if (now_s() >= end) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
}  // namespace

TEST(zero_g_start_sends_on_and_starts_the_keepalive) {
    lt::Offline off;
    off->zero_g_start(0.005);
    CHECK(off->zero_g_active());
    // 固件 0x06 自带 watchdog_kick (「重发即保活」), 而固件里没有别的命令能维持一个非
    // MOVE_J 模式不被 0.10s 看门狗掐死 => 保活必须周期重发。
    // ⚠ 判据是"**会重发**" (至少 3 拍), 不是"80ms 内正好几拍" —— 周期 5ms 只是下限约束,
    //   上界由固件看门狗 (0.10s) 兜着; 断言具体拍数就等于断言机器有多快 (见上面那条)。
    CHECK(wait_for_zero_g_on(off.t(), 3, 2.0) >= 3);
    off->zero_g_stop();
    CHECK(!off->zero_g_active());
    // 退出会发 on=0
    int off_frames = 0;
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first == proto::CMD_ZERO_G && kv.second.size() == 1 &&
            kv.second[0] == 0x00) {
            ++off_frames;
        }
    }
    CHECK_EQ(off_frames, 1);
}

TEST(zero_g_start_is_idempotent) {
    lt::Offline off;
    off->zero_g_start(0.01);
    const int first = count_zero_g_on(off.t());
    off->zero_g_start(0.01);
    // 已激活 = 幂等 no-op: 根本不写 0x06, 无 coast 风险
    CHECK_EQ(count_zero_g_on(off.t()), first);
    off->zero_g_stop();
}

TEST(zero_g_stop_is_idempotent_and_does_not_invent_a_session) {
    lt::Offline off;
    // 从未进入过就不发任何 0x06 —— 幂等退出不等于无中生有
    CHECK_NOTHROW(off->zero_g_stop());
    CHECK_NOTHROW(off->zero_g_stop());
    CHECK_EQ(count_cmd(off.t(), proto::CMD_ZERO_G), 0);
}

TEST(zero_g_guard_rejects_action_commands) {
    lt::Offline off;
    off->zero_g_start(0.01);
    CHECK_THROWS_AS(off->movej(std::vector<double>(7, 0.0)), InvalidCommandError);
    CHECK_THROWS_AS(off->movej_sync(std::vector<double>(7, 0.0)), InvalidCommandError);
    CHECK_THROWS_AS(off->move_p({0.3, 0.0, 0.4, 0.0, 0.0, 0.0}), InvalidCommandError);
    CHECK_THROWS_AS(off->home(), InvalidCommandError);
    CHECK_THROWS_AS(off->enable(), InvalidCommandError);
    CHECK_THROWS_AS(off->save_params(), InvalidCommandError);
    CHECK_THROWS_AS(off->move_js(std::vector<double>(7, 0.0)), InvalidCommandError);
    // 归因入口: 文案要说清"为什么拒"与"怎么退出"
    try {
        off->movej(std::vector<double>(7, 0.0));
        FAIL("应当被拒");
    } catch (const InvalidCommandError& e) {
        CHECK(std::string(e.what()).find("zero_g_stop()") != std::string::npos);
    }
    off->zero_g_stop();
}

TEST(zero_g_guard_lets_queries_and_energy_shedding_through) {
    // 拖动示教期间仍应能读状态; 降能量方向的动作必须永远可达。
    lt::Offline off;
    off->zero_g_start(0.01);
    CHECK_NOTHROW(off->get_state(true));
    CHECK_NOTHROW(off->get_tcp());
    CHECK_NOTHROW(off->get_status_now());
    CHECK_NOTHROW(off->emergency_stop());
    CHECK_NOTHROW(off->disable());
    CHECK_NOTHROW(off->params().get_joint_param(0));
    CHECK_NOTHROW(off->diag().kin_bench());
    off->zero_g_stop();
}

TEST(zero_g_period_is_validated_against_the_watchdog) {
    // 固件看门狗超时是 0.10s —— 周期必须留在它以内且不至于太快。
    lt::Offline off;
    CHECK_THROWS_AS(off->zero_g_start(0.1), InvalidCommandError);
    CHECK_THROWS_AS(off->zero_g_start(0.001), InvalidCommandError);
    CHECK_THROWS_AS(off->zero_g_start(-1.0), InvalidCommandError);
    CHECK_NOTHROW(off->zero_g_start(0.04));
    off->zero_g_stop();
}

TEST(zero_g_raii_session_stops_on_scope_exit) {
    lt::Offline off;
    {
        auto session = off->zero_g(0.01);
        CHECK(session.active());
        CHECK(session.arm() == off.arm.get());
    }
    CHECK(!off->zero_g_active());
    int off_frames = 0;
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first == proto::CMD_ZERO_G && kv.second.size() == 1 &&
            kv.second[0] == 0x00) {
            ++off_frames;
        }
    }
    CHECK_EQ(off_frames, 1);
}

TEST(zero_g_session_stop_is_the_throwing_variant) {
    lt::Offline off;
    auto session = off->zero_g(0.01);
    CHECK_NOTHROW(session.stop());
    CHECK(!off->zero_g_active());
    // 已经停过之后再 stop 是空操作
    CHECK_NOTHROW(session.stop());
}

TEST(zero_g_session_can_be_moved) {
    lt::Offline off;
    auto a = off->zero_g(0.01);
    auto b = std::move(a);
    // 被移动走的那一个不再是持有者
    CHECK(!a.active());
    CHECK(b.active());
    b.reset();
    CHECK(!off->zero_g_active());
}

TEST(zero_g_keepalive_failure_is_visible_and_not_silent) {
    // 写失败 = 保活已断, 必须让调用方知道 —— 臂已脱离零重力, 不能静默。
    lt::Offline off;
    off.t().zg_fail_after = 1;      // 第 2 次 0x06 写 (首次保活) 抛
    off->zero_g_start(0.005);
    // 等保活线程真正跑到那一拍 (有上界) —— 见 `wait_until` 的说明。
    CHECK(wait_until([&] { return !off->zero_g_active(); }, 2.0));
    CHECK(off->zero_g_error() != nullptr);
    CHECK(!off->zero_g_error_text().empty());
    // raise_on_lost=true 时 stop 会把原异常抛出来
    CHECK_THROWS_AS(off->zero_g_stop(true), TransportError);
    // stop 消费并清空它
    CHECK(off->zero_g_error() == nullptr);
}

TEST(zero_g_stop_without_raise_swallows_the_keepalive_failure) {
    lt::Offline off;
    off.t().zg_fail_after = 1;
    off->zero_g_start(0.005);
    // 先确保保活**已经断掉**再 stop —— 否则测到的是"正常退出", 不是"退出时吞掉保活异常"。
    wait_until([&] { return !off->zero_g_active(); }, 2.0);
    CHECK_NOTHROW(off->zero_g_stop(false));
}

TEST(zero_g_close_stops_the_keepalive_thread) {
    // 第一步收保活线程必须是第一步 (它持有一个写者; 残留会让进程退出时打哑 CDC)。
    lt::Offline off;
    off->zero_g_start(0.005);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    off->close();
    // ⚠ 基线必须取在 **close 之后**: 取在它之前的话, 采样与 close 之间保活线程还能**合法地**
    //   再发一帧 (Debug/TSan 下变慢就会撞上, 实测 2 != 1) —— 那是采样窗口的竞态, 不是缺陷。
    const int after_close = count_zero_g_on(off.t());
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    // close 之后不该再有任何保活帧出去
    CHECK_EQ(count_zero_g_on(off.t()), after_close);
}

// ---------------------------------------------------------------- 反向守卫

TEST(zero_g_is_refused_while_a_cartesian_motion_is_in_flight) {
    // 中途进场会丢掉位置环、只剩重力前馈 => 臂会靠摩擦滑停 (coast), 比受控接管差得多。
    lt::Offline off;
    // ⚠ `quiesce` 在这里的作用与别处**不同**: 不是躲 get_status_now 的窗口, 而是让
    //   "bit10 还亮着"这件事确定。桩的 CART_BUSY 只在"交付后的第 2..7 帧"置位 (见
    //   `set_cart_busy`), 被动流一直在跑 ⇒ TSan 下守卫去读的时候窗口可能**已经过期**,
    //   于是它读到 bit10=0 而放行 —— 翻红的是布景, 不是守卫。
    //   关掉被动流之后, 从 `set_cart_busy(5)` 到守卫读帧之间只会多出**它自己**那条 0x40
    //   的应答帧, 计数恰好 +1, 必落在窗口内。
    lt::quiesce(off.t());
    // 制造"在途": 让某条笛卡尔入口持着串行锁 (用一条永不返回的响应拖住它不容易),
    // 故改用第二条判据 —— 现取一帧状态的 bit10 为 1。
    off.t().set_cart_busy(3);   // 接下来几帧状态带 CART_BUSY
    off.t().set_pose({0.3, 0.0, 0.4, 0.0, 0.0, 0.0});
    // 先跑一条 wait=false 的 move_l 让它把 cart_busy 拉起来
    off->move_l({0.32, 0.0, 0.42, 0.0, 0.0, 0.0}, 0.5, false);
    off.t().set_cart_busy(5);   // 保持"臂还在跑"的形状
    try {
        off->zero_g_start(0.01);
        // 若放行了, 说明两条判据都没抓住 —— 但也可能只是桩的帧序巧合, 故只报失败不崩。
        off->zero_g_stop();
        CHECK(false);
    } catch (const InvalidCommandError& e) {
        CHECK(std::string(e.what()).find("笛卡尔运动在途") != std::string::npos);
    }
}

TEST(zero_g_is_refused_conservatively_when_the_status_is_unavailable) {
    // 未确认就不拦, 漏过去的正是"轨迹中途进场靠摩擦滑停"那一侧 —— 故保守拒绝,
    // 但必须说清是哪一种拒绝。
    //
    // ⚠⚠ `quiesce` 是本用例的**前提**, 不是讲究 —— 缺了它本用例在 TSan 下约 1/10 概率翻红
    //   (实测 2026-09-28: 30 次里 3 次 "应当保守拒绝")。
    //
    //   原因与 `cmd_unsupported_during_status_read_...` 是**同一个**: `get_status_now`
    //   的结论判据后半条是"有任何新状态帧到", 而被动流与我们的 `0x40` 是两条独立的线。
    //   帧先落进窗口 ⇒ 它以为命令成功了 ⇒ 这里拿到一帧**陈旧**的状态 ⇒ `cart_busy()`
    //   为 0 ⇒ 守卫**放行**。
    //   ⚠ 这是本缺陷里后果最重的一处: 它落在**安全守卫**上, 而且是**fail-open** ——
    //   "没能确认是否在途"本该保守拒绝, 却因为"误以为确认过了"而放行。见 README。
    lt::Offline off;
    lt::quiesce(off.t());
    off.t().err_override[proto::CMD_GET_STATUS] = 0x03;
    try {
        off->zero_g_start(0.01);
        off->zero_g_stop();
        FAIL("应当保守拒绝");
    } catch (const InvalidCommandError& e) {
        CHECK(std::string(e.what()).find("保守拒绝") != std::string::npos);
    }
}

TEST(zero_g_already_active_skips_the_reverse_guard) {
    // 已激活 = 幂等 no-op: 根本不写 0x06, 无 coast 风险, 故不该被反向守卫拦住。
    lt::Offline off;
    off->zero_g_start(0.01);
    off.t().set_cart_busy(3);
    CHECK_NOTHROW(off->zero_g_start(0.01));
    off.t().clear_cart_busy();
    off->zero_g_stop();
}

TEST(zero_g_last_reset_reason_and_error_are_local_accessors) {
    lt::Offline off;
    CHECK(off->zero_g_error() == nullptr);
    CHECK(off->zero_g_error_text().empty());
    CHECK_EQ(off->zero_g_active(), false);
}
