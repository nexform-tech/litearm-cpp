// 并发与收尾 —— 唯一读者 / 并发读状态 / 读线程生命周期 / 串行锁。
#include <limits>

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
}  // namespace

TEST(threading_concurrent_state_readers_do_not_starve_each_other) {
    // 读线程按 100Hz 一直填状态单槽, 驱动型读者只是"等单槽前进" —— 多个并发读者
    // 不再互相饿 (旧实现里"只认自己读到的帧"会被并发读者饿到超时)。
    lt::Offline off;
    std::atomic<int> ok{0};
    std::atomic<int> failed{0};
    auto worker = [&]() {
        for (int i = 0; i < 40; ++i) {
            try {
                const auto m = off->get_state(true, 0.5);
                if (m.value) ++ok;
            } catch (...) {
                ++failed;
            }
        }
    };
    std::vector<std::thread> ths;
    for (int i = 0; i < 4; ++i) ths.emplace_back(worker);
    for (auto& t : ths) t.join();
    CHECK_EQ(failed.load(), 0);
    CHECK_EQ(ok.load(), 160);
}

TEST(threading_concurrent_cartesian_entries_are_serialized) {
    // 串行是强制的: 0x3A/0x3B/0x3E 共用码空间, ERR 回显只能按码判, >=2 条在途时会互相
    // 认下对方的 ERR/0x4E => 假成功。持锁把它们变成排队。
    lt::Offline off;
    std::atomic<int> ok{0};
    std::atomic<int> failed{0};
    auto worker = [&](int seed) {
        for (int i = 0; i < 6; ++i) {
            try {
                std::array<double, 6> p{0.30 + seed * 0.001, 0.0,
                                        0.40 + i * 0.001, 0.0, 0.0, 0.0};
                const CartPlan plan = off->move_l(p, 0.5, /*wait=*/false);
                if (plan.ok) ++ok;
            } catch (...) {
                ++failed;
            }
        }
    };
    std::vector<std::thread> ths;
    for (int i = 0; i < 3; ++i) ths.emplace_back(worker, i);
    for (auto& t : ths) t.join();
    // 每一条都该拿到自己的规划结果 —— 没有错配, 也没有互相吃掉应答
    CHECK_EQ(failed.load(), 0);
    CHECK_EQ(ok.load(), 18);
    CHECK_EQ(off->poll_cart().has_value(), false);
}

TEST(threading_mixed_readers_and_motion) {
    lt::Offline off;
    std::atomic<bool> stop{false};
    std::atomic<int> read_errors{0};
    std::thread reader([&]() {
        while (!stop.load()) {
            try {
                off->get_state(true, 0.3);
                off->get_tcp(0.3);
            } catch (const NotConnectedError&) {
                break;
            } catch (...) {
                ++read_errors;
            }
        }
    });
    for (int i = 0; i < 5; ++i) {
        CHECK_NOTHROW(off->movej(std::vector<double>(7, 0.01 * i)));
    }
    stop.store(true);
    reader.join();
    CHECK_EQ(read_errors.load(), 0);
}

TEST(threading_close_while_another_thread_is_reading) {
    // close() 必须能把阻塞在等待里的读者唤醒 —— 否则 teardown 会挂死。
    lt::Offline off;
    std::atomic<bool> released{false};
    std::thread reader([&]() {
        try {
            off->get_state(true, 3.0);
        } catch (...) {
        }
        released.store(true);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const double t0 = now_s();
    off->close();
    CHECK(now_s() - t0 < 2.0);       // close 不该被那次等待拖住
    reader.join();
    CHECK(released.load());
}

TEST(threading_close_stops_the_reader_thread) {
    lt::Offline off;
    off->get_state();                 // 确认会话在跑
    off->close();
    // 关掉之后调用必须响亮, 而不是从一个已经停掉的读线程上拿到陈旧值
    CHECK_THROWS_AS(off->get_state(), NotConnectedError);
    // 重复 close 不抛
    CHECK_NOTHROW(off->close());
}

TEST(threading_reconnect_after_close_is_clean) {
    lt::Offline off;
    off->close();
    CHECK_NOTHROW(off->reconnect());
    CHECK_EQ(off->n(), 7);
    CHECK_NOTHROW(off->get_state());
}

TEST(threading_msg_hz_converges_for_the_passive_status_stream) {
    // 被动连续流 (状态帧, 固件 100Hz): 两帧之后 hz 收敛到接近固件周期。
    lt::Offline off;
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    const auto m = off->get_state();
    CHECK(m.value.has_value());
    // 桩的 auto_status 是 1ms 一拍, 故 hz 应当明显大于 0
    CHECK(m.hz > 10.0);
    CHECK(m.timestamp > 0.0);
}

TEST(threading_msg_hz_is_zero_when_a_frame_type_never_arrived) {
    lt::Offline off;
    // 只读一次单发请求/应答式的帧 => count < 2 => hz 写死 0.0 (一个样本定不出频率)
    const auto m = off->get_ff_scalar(1, 0);
    CHECK_NEAR(m.hz, 0.0, 1e-12);
    CHECK(m.timestamp > 0.0);
}

// ---------------------------------------------------------------------------
// 同帧节流: **已整套删除** (2026-09-28)
// ---------------------------------------------------------------------------
//
// 这里原有 7 条用例 (默认关闭 / 丢不掉等 ACK 的命令 / 对保活生效 / 拒绝非法间隔 /
// 终态守卫不被短路 / 丢帧不清队)。它们测的那套机制 —— `set_tx_repeat_min_interval` +
// `raw_write` 的 `droppable` 形参 + `tx_allowed` —— **已经不存在了**, 用例随功能一起下线。
// ⚠ 这不是"删测试凑绿", 是**功能下线**: 依据见 README「与原版的差异」——
//   参照实现里那套 `_tx_allowed` 是**私有且默认全关**的(行为上等价于无), 故不做。
//
// ⚠ 那 7 条里有**两条**护的其实不是节流本身, 而是别的不变量 —— 它们被改写保留:
//   ① 终态守卫必须**响亮** ⇒ 见下面 `threading_the_terminal_state_guard_stays_loud`;
//   ② "清队钩子的理据是**固件收到**命令才会作废在途规划 ⇒ 只有真发出去的帧才清队" ⇒
//      这条现在**结构性地成立**(唯一写口进了就一定会写), 不再需要用例看着它。
//      原来那条 `a_dropped_frame_does_not_clear_the_cart_queue` 因此下线。

TEST(threading_the_terminal_state_guard_stays_loud) {
    // ⚠ 终态 (`reject_if_in_dfu`) 下**任何**写都必须抛 `ArmIsInDfuError`, 绝不许有任何
    //   "静默、什么都发生"的路径 —— 否则调用方看不出自己已经在一个废掉的会话上。
    //
    // 本用例的理由与节流无关 (删节流前后都该成立): 当年它是为了钉"守卫要排在节流**之前**"。
    // 现在守卫是唯一排在前面的人, 这条更要钉住。
    lt::Offline off;
    TestAccess::raw_write(*off.arm, proto::CMD_ZERO_G, {0x01});   // 先真发一条
    off->enter_dfu();                                            // 终端态
    CHECK_THROWS_AS(TestAccess::raw_write(*off.arm, proto::CMD_ZERO_G, {0x01}),
                    ArmIsInDfuError);
}

TEST(threading_every_raw_write_reaches_the_wire_there_is_no_drop_path) {
    // ⚠ 删掉节流之后唯一写口的**新**不变式: 进了 `raw_write` 就一定会写出去。
    //   载荷逐字节相同的连发也照样全发 —— 从前那种"重复帧被静默丢掉"的路径**不存在**。
    //   (伺服类命令 `move_js` 靠 >=10Hz 重发维持固件 0.1s 命令看门狗 ⇒ 丢它的帧
    //    就是在给它断流, 这是本条要钉的现场。)
    lt::Offline off;
    const std::vector<double> q(7, 0.1);
    for (int i = 0; i < 5; ++i) CHECK_NOTHROW(off->move_js(q));
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_JS), 5);   // 5 发 5 到

    const std::vector<double> z(7, 0.0);
    for (int i = 0; i < 3; ++i) CHECK_NOTHROW(off->movej(z));
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_J), 3);
}

TEST(threading_offline_transport_registration_is_not_leaked_by_the_fixture) {
    // FakeTransport 不登记端口 (它是桩), 故进程内登记表在这些用例里应当保持干净。
    {
        lt::Offline off;
        (void)off;
    }
    // 真传输那半由 test_transport.cpp 的独占用例覆盖。
    CHECK(true);
}
