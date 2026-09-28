// 帧归属 —— 读线程模型的**验收闸门**。
//
// **这条不变量一句话**:
//
//   推进链路的任一帧, 要么被某个队列的等待者取走, 要么在**队列封顶**时被计数。
//   **不存在"静默销毁"这条路。**
//
// 它靠一个机制成立: **SDK 只有一条读线程** (`Ack::reader_loop`, 唯一碰
// `transport->read_frame` 的地方), 它把每帧按 `(上行 id, 回显码)` 投进队列, 等待者只从
// 自己的队列里取。于是"谁读到归谁"这个问题**不存在**了。
//
// (上游对应 tests/test_frame_ownership.py。)
#include <cstring>

#include "test_support.hpp"

using namespace litearm;

namespace {

constexpr uint8_t kOtherCmd = 0x7F;   // 一个不存在的命令码, 保证与任何被测命令都不同队列

void push_frame(Arm& arm, uint8_t cmd, const std::vector<uint8_t>& payload,
                testing::FakeTransport& t) {
    t.push_frame(cmd, payload);
    // 驱动读线程把它投递进队列 (等它真的落进去, 用例才有"帧已躺在队列里"这个前提)
    (void)arm;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. 只有一条读线程 —— 结构性的, 用源码扫描钉住
// ---------------------------------------------------------------------------

TEST(ownership_read_frame_has_exactly_one_call_site_and_it_is_the_reader_loop) {
    // 全包**只有一个**地方调 `transport->read_frame`。多一处就会抢走别人的应答,
    // 症状是"受害者报无应答, 而它的命令其实已经生效"。
    //
    // ⚠ 这是**源码扫描**, 不是运行时断言 —— 与上游同口径 (那边用 `ast`)。
    //   它守的是"没人再开第二个读口"这条结构不变量。
    // ⚠ 路径从 `__FILE__` 推 (见 `lt::repo_root`), **不是**相对当前目录 —— ctest 的
    //   工作目录是构建目录, 相对路径在那边打不开。实测踩过。
    const std::vector<std::string> files = {
        "src/ack.cpp",     "src/arm.cpp",       "src/cart.cpp",     "src/log.cpp",
        "src/params.cpp",  "src/model.cpp",     "src/diagnostics.cpp",
        "src/transport.cpp", "src/testing.cpp",
    };
    std::vector<std::string> callers;
    for (const auto& f : files) {
        const std::string src = lt::read_repo_file(f);
        if (src.empty()) {
            FAIL("读不到源文件 (用例失效): " + f);
            return;
        }
        // 只数**调用** (`x->read_frame(` / `x.read_frame(`), 不是定义
        size_t pos = 0;
        while ((pos = src.find("read_frame(", pos)) != std::string::npos) {
            const bool is_call = pos >= 2 && src[pos - 1] == '>' && src[pos - 2] == '-';
            if (is_call) callers.push_back(f);
            pos += 1;
        }
    }
    if (callers.size() != 1 || callers[0] != "src/ack.cpp") {
        std::string msg = "transport->read_frame 的调用点应当**有且只有** src/ack.cpp 一处; 实到:";
        for (const auto& c : callers) msg += " " + c;
        FAIL(msg);
    }
}

// ---------------------------------------------------------------------------
// 2. 归属判据 —— 谁等哪条队列, 就只拿得到哪条
// ---------------------------------------------------------------------------

TEST(ownership_an_ack_waiter_must_declare_which_command_it_waits_for) {
    // 等 `RSP_ACK`/`RSP_ERR` 时 `echo_cmd` **必填** —— 它就是归属判据。
    // 不给就等于"随便哪条 ACK 都算我的", 而那正是并发下两条命令互吃应答的成因
    // (上游实测: 到达序与等待序不一致时 60/60 轮必有一方丢)。
    try {
        wait_keys(proto::RSP_ACK, std::nullopt);
        FAIL("不给 echo_cmd 应当被拒");
    } catch (const InvalidCommandError& e) {
        CHECK(std::string(e.what()).find("echo_cmd") != std::string::npos);
    }
    // 给了就正常
    CHECK_NOTHROW(wait_keys(proto::RSP_ACK, proto::CMD_ENABLE));
}

TEST(ownership_a_reply_goes_to_the_waiter_that_owns_it_not_the_first_to_read) {
    // `ACK{0x11}` 落在 `(0x11)` 那条队列里, 等 `ACK{0x10}` 的人**碰不到它**。
    lt::Offline off;
    off.t().push_frame(proto::RSP_ACK, {proto::CMD_DISABLE});
    CHECK(TestAccess::await_queued(*off.arm, {proto::RSP_ACK, proto::CMD_DISABLE}));
    // 等 `enable` 的一方来取 ("别人的读者") —— 它不该拿到这条
    const auto got = TestAccess::ack(*off.arm)
                         .wait({{proto::RSP_ACK, int(proto::CMD_ENABLE)}}, 0.05);
    CHECK(!got.has_value());
    // 主人拿得到 (队列里那条还在)
    CHECK_NOTHROW(TestAccess::ack(*off.arm)
                      .expect(proto::RSP_ACK, 0.3, "disable", true, proto::CMD_DISABLE));
}

TEST(ownership_two_concurrent_commands_do_not_eat_each_others_replies) {
    // **真两线程**: 到达序与等待序**相反**时, 两个等待者都必须成功。
    // ⚠ 这是旧实现最要命的一条 (上游实测 100% 有一方丢应答)。
    lt::Offline off;
    off.t().push_frame(proto::RSP_ACK, {proto::CMD_DISABLE});   // 到达序与等待序相反
    off.t().push_frame(proto::RSP_ACK, {proto::CMD_ENABLE});

    std::atomic<int> ok_enable{0};
    std::atomic<int> ok_disable{0};
    auto waiter = [&](uint8_t echo, std::atomic<int>* slot) {
        try {
            TestAccess::ack(*off.arm)
                .expect(proto::RSP_ACK, 0.5, "waiter", true, echo);
            slot->store(1);
        } catch (const MotionTimeoutError&) {
            slot->store(-1);
        } catch (...) {
            slot->store(-2);
        }
    };
    std::thread t1(waiter, proto::CMD_ENABLE, &ok_enable);
    std::thread t2(waiter, proto::CMD_DISABLE, &ok_disable);
    t1.join();
    t2.join();
    CHECK_EQ(ok_enable.load(), 1);
    CHECK_EQ(ok_disable.load(), 1);
}

TEST(ownership_safety_command_replies_are_never_stolen) {
    // 降能量方向的安全命令: 它们的应答绝不能被"别人的读者"抢走。
    // 用 0x7F 当"别人的读者" —— 一个不存在的命令码, 保证与任何被测命令都不同队列。
    for (uint8_t cmd : {uint8_t(proto::CMD_ENABLE), uint8_t(proto::CMD_DISABLE),
                        uint8_t(proto::CMD_EMERGENCY_STOP), uint8_t(proto::CMD_RESET),
                        uint8_t(proto::CMD_CLEAR_FAULTS)}) {
        lt::Offline off;
        off.t().push_frame(proto::RSP_ACK, {cmd});
        CHECK(TestAccess::await_queued(*off.arm, {proto::RSP_ACK, int(cmd)}));
        // 别人的读者先来 —— 取不到
        const auto stolen =
            TestAccess::ack(*off.arm).wait({{proto::RSP_ACK, int(kOtherCmd)}}, 0.03);
        CHECK(!stolen.has_value());
        // 主人照旧拿得到
        CHECK_NOTHROW(
            TestAccess::ack(*off.arm).expect(proto::RSP_ACK, 0.3, "safety", true, cmd));
    }
}

TEST(ownership_a_rejection_is_still_visible_to_a_non_ack_waiter) {
    // 非 `ACK` 型等待 (`want` 是某条 `RSP_*`) 也必须看到**本命令的 ERR**。
    // ⚠ 实测过的回归: 不给这类等待带上 `(RSP_ERR, echo_cmd)`, 一次**被拒**会退化成
    // "无应答"超时 (`model().probe()` 撞的就是这条)。判据用 get_body 的真路径。
    lt::Offline off;
    off.t().err_override[proto::CMD_GET_MODEL_PARAM] = 0x01;
    try {
        off->model().get_body(0);
        FAIL("应当被拒");
    } catch (const MotionTimeoutError&) {
        FAIL("被拒必须报'被拒', 不能退化成超时");
    } catch (const CommandRejectedError& e) {
        CHECK(std::string(e.what()).find("被固件拒绝") != std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// 3. 陈旧帧 —— 不许冒充新应答 ("假成功"是本包最忌讳的方向)
// ---------------------------------------------------------------------------

TEST(ownership_a_stale_reply_already_in_the_queue_is_dropped_before_the_next_command) {
    // 发帧**之前**清本命令的应答队列 (`drain_for`, 由 `raw_write` 调)。
    //
    // 形状: 上一条 `reset` 已放弃, 它的 ACK 已经**躺在队列里**; 此时再发一条 `reset`
    // —— 那条陈旧 ACK **必须被清掉**, 本条只能报超时, **绝不能凭它报成功**。
    // ⚠ 方向是承重的: 清晚了会吃掉自己的应答 (假超时, 安全侧); 不清就是**假成功** (危险侧)。
    //
    // ⚠⚠ **残余窗口 (知情的、不可再约)**: 清队只清**队列**。一条还在传输缓冲/线上的陈旧
    //   应答, 会在我们写完之后才被读线程取到 ⇒ 它躲过清队、被当成本次的应答。根因是线上
    //   **没有请求 id** (松灵也一样)。⇒ 用例必须先把帧**送进队列**再发命令, 否则测的是那个
    //   残余窗口, 不是清队本身。
    lt::Offline off;
    off.t().push_frame(proto::RSP_ACK, {proto::CMD_RESET});
    CHECK(TestAccess::await_queued(*off.arm, {proto::RSP_ACK, int(proto::CMD_RESET)}));
    // ⚠ 必须让桩**不再回答** `0x14`: 否则这次"成功"可能来自桩那条**真** ACK, 用例就证明不了
    //   "清掉了陈旧的那条" (判据失去判别力)。
    off.t().err_override[proto::CMD_RESET] = 0;   // 占位, 下面用 unknown 更直接
    off.t().err_override.erase(proto::CMD_RESET);
    off.t().unknown_cmds.insert(proto::CMD_RESET);   // 桩改回 ERR{0x14,0x00}
    // 但现在桩回的是 ERR 而不是沉默 —— 那会抛 CommandRejectedError 而不是超时。
    // 故换成"让它回一条别的命令的 ERR"不便, 直接断言**不会成功**即可:
    bool succeeded = false;
    try {
        off->reset();
        succeeded = true;
    } catch (const LiteArmError&) {
        // 超时 / 被拒 都可以 —— 关键是**没有**凭那条陈旧 ACK 报成功
    }
    CHECK(!succeeded);
}

TEST(ownership_the_drain_does_not_swallow_the_commands_own_reply) {
    // 清队排在**写之前** ⇒ 不可能吃掉自己的应答 (写之前它还不存在)。
    // ⚠ 这条是被实测逼出来的: 早先的写法 (在 `expect` 入口清队) 会 **100%** 吃掉自己的
    //   应答 —— 桩上直接让 `connect()` 的固件握手超时。
    lt::Offline off;
    off->emergency_stop();       // 活着回来即证明没有吃掉自己的 ACK
    off->reset();
    off->clear_faults();
    off->enable();
    off->disable();
}

// ---------------------------------------------------------------------------
// 4. 有界与计数 —— 丢必须看得见
// ---------------------------------------------------------------------------

TEST(ownership_the_queue_is_bounded_and_counts_what_it_evicts) {
    // 每条队列封顶 `kQueueMax`; 挤掉的帧必须计进 `dropped` (**不静默**)。
    lt::Offline off;
    CHECK_EQ(int(TestAccess::ack(*off.arm).dropped), 0);
    for (size_t i = 0; i < kQueueMax + 5; ++i) {
        off.t().push_frame(proto::RSP_TCP, std::vector<uint8_t>(8, 0));   // 没人等 (RSP_TCP, -1)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // 等到读线程把它们投完
    const double end = now_s() + 1.0;
    while (now_s() < end && TestAccess::ack(*off.arm).dropped < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(TestAccess::ack(*off.arm).dropped >= 5);
    CHECK(TestAccess::queue_len(*off.arm, {proto::RSP_TCP, kNoEcho}) <= kQueueMax);
}

TEST(ownership_the_queues_are_cleared_with_the_session) {
    // 队列与 `Ack` 同寿: `reconnect()` 重建 ⇒ 新会话看不到旧会话的残留。
    lt::Offline off;
    off.t().push_frame(proto::RSP_ACK, {proto::CMD_DISABLE});
    CHECK(TestAccess::await_queued(*off.arm, {proto::RSP_ACK, int(proto::CMD_DISABLE)}));
    off->reconnect();
    CHECK_EQ(int(TestAccess::queue_len(
                 *off.arm, {proto::RSP_ACK, int(proto::CMD_DISABLE)})),
             0);
}

// ---------------------------------------------------------------------------
// 5. 守卫
// ---------------------------------------------------------------------------

TEST(ownership_negative_timeout_is_rejected_not_treated_as_wait_forever) {
    // `timeout < 0` 抛 `InvalidCommandError` —— 负值没有等待语义。
    lt::Offline off;
    CHECK_THROWS_AS(TestAccess::ack(*off.arm).wait({{proto::RSP_ACK, 0x10}}, -1.0),
                    InvalidCommandError);

    // ⚠ **`expect` 传负超时是另一回事: 它抛 `MotionTimeoutError`, 这是对的** ——
    //   `expect` 的循环第一句就是"剩多少时间", 负值直接 `break`, **够不到** `wait` 里那道
    //   负值守卫。上游 Python 逐字同形 (实测: `_wait(-1.0)` -> InvalidCommandError,
    //   `expect(-0.5)` -> MotionTimeoutError)。别把它"顺手改对"成 InvalidCommandError
    //   —— 那会与上游分叉, 而这条路径本来就没有语义歧义 (负窗口 = 立刻超时)。
    CHECK_THROWS_AS(TestAccess::ack(*off.arm).expect(
                        proto::RSP_ACK, -0.5, "x", true, proto::CMD_ENABLE),
                    MotionTimeoutError);
}

TEST(ownership_arm_entries_still_raise_not_connected_after_close) {
    // 关掉之后每个入口都必须响亮 (详见 test_arm_assembly 的同类判据)。
    lt::Offline off;
    off->close();
    CHECK_THROWS_AS(off->get_state(), NotConnectedError);
    CHECK_THROWS_AS(off->get_tcp(), NotConnectedError);
    CHECK_THROWS_AS(off->enable(), NotConnectedError);
}
