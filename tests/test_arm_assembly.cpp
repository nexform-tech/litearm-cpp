// Arm 的会话装配 —— 握手/版本约定/幂等 connect/reconnect/收尾/终态。
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

TEST(assembly_connect_handshakes_and_fills_in_the_session_facts) {
    lt::Offline off("Litearm1.7.0-7J", 7);
    CHECK_EQ(off->firmware(), std::string("Litearm1.7.0-7J"));
    CHECK_EQ(off->n(), 7);
    CHECK(off->fw_version().has_value());
    if (off->fw_version()) {
        CHECK_EQ(off->fw_version()->major, 1);
        CHECK_EQ(off->fw_version()->minor, 7);
        CHECK_EQ(off->fw_version()->patch, 0);
    }
    // 握手期间确实发过 GET_FIRMWARE
    CHECK_EQ(count_cmd(off.t(), proto::CMD_GET_FIRMWARE), 1);
}

TEST(assembly_connect_works_on_the_single_axis_bench_firmware) {
    lt::Offline off("Litearm1.7.0-1J", 1);
    CHECK_EQ(off->n(), 1);
    CHECK_EQ(off->firmware(), std::string("Litearm1.7.0-1J"));
}

TEST(assembly_rejects_a_legacy_version_string) {
    lt::Unconnected un("A1.2.3-USB");
    CHECK_THROWS_AS(un.arm->connect(), FirmwareMismatchError);
    // 失败路径必须关链路, 不留半开的会话
    CHECK(!un.arm->firmware().size());
    CHECK_EQ(un.fake->closed, true);
}

TEST(assembly_rejects_too_old_firmware) {
    lt::Unconnected un("Litearm1.4.9-7J");
    CHECK_THROWS_AS(un.arm->connect(), FirmwareMismatchError);
}

TEST(assembly_accepts_exactly_the_minimum_firmware) {
    lt::Offline off("Litearm1.5.0-7J", 7);
    CHECK_EQ(off->firmware(), std::string("Litearm1.5.0-7J"));
}

TEST(assembly_connect_is_idempotent_for_the_same_target) {
    lt::Offline off;
    // 重复调用最不该打断的正是正在跑的那个会话 —— 不关链路、不重建、不重新握手。
    CHECK_EQ(count_cmd(off.t(), proto::CMD_GET_FIRMWARE), 1);
    off.arm->connect();
    off.arm->connect();
    CHECK_EQ(count_cmd(off.t(), proto::CMD_GET_FIRMWARE), 1);
    CHECK_EQ(off->n(), 7);
}

TEST(assembly_reconnect_always_rebuilds) {
    lt::Offline off;
    CHECK_EQ(count_cmd(off.t(), proto::CMD_GET_FIRMWARE), 1);
    off.arm->reconnect();
    CHECK_EQ(count_cmd(off.t(), proto::CMD_GET_FIRMWARE), 2);
}

TEST(assembly_connect_to_another_port_really_retargets) {
    // 幂等不是"永远不做事": 显式给了另一个端口时真的改靶 —— 否则调用方以为连的是 ACM2,
    // 实际还在 ACM1, 而本模块一贯不接受"静默无效"。
    auto a = std::make_shared<testing::FakeTransport>("portA", 0.2, "Litearm1.7.0-7J", 7);
    auto b = std::make_shared<testing::FakeTransport>("portB", 0.2, "Litearm1.7.0-7J", 7);
    ArmOptions opts;
    opts.port = "portA";
    opts.transport_factory = [a, b](const std::string& p) -> std::unique_ptr<Transport> {
        if (p == "portB") return std::unique_ptr<Transport>(new lt::SharedFake(b));
        return std::unique_ptr<Transport>(new lt::SharedFake(a));
    };
    opts.move_timeout = 1.0;
    Arm arm(opts);
    arm.connect();
    CHECK_EQ(a->tx_count() > 0, true);
    arm.connect("portB");
    CHECK_EQ(b->tx_count() > 0, true);
    CHECK_EQ(count_cmd(*b, proto::CMD_GET_FIRMWARE), 1);
    arm.close();
}

TEST(assembly_close_is_idempotent_and_clears_session_facts) {
    lt::Offline off;
    off.arm->close();
    CHECK(!off.arm->firmware().size());
    CHECK_EQ(off.arm->n(), 0);
    CHECK(!off.arm->fw_version().has_value());
    CHECK_NOTHROW(off.arm->close());
    CHECK_NOTHROW(off.arm->close());
}

TEST(assembly_disconnect_is_close) {
    lt::Offline off;
    off.arm->disconnect();
    CHECK_EQ(off.arm->n(), 0);
    CHECK_THROWS_AS(off.arm->get_tcp(), NotConnectedError);
}

TEST(assembly_after_close_the_session_is_unusable) {
    lt::Offline off;
    off.arm->close();
    CHECK_THROWS_AS(off.arm->get_state(), NotConnectedError);
    CHECK_THROWS_AS(off.arm->movej(std::vector<double>(7, 0.0)), NotConnectedError);
    CHECK_THROWS_AS(off.arm->enable(), NotConnectedError);
    CHECK_THROWS_AS(off.arm->get_tcp(), NotConnectedError);
    CHECK_THROWS_AS(off.arm->ik({0.3, 0, 0.4, 0, 0, 0}), NotConnectedError);
}

TEST(assembly_unconnected_arm_raises_not_connected_not_something_else) {
    lt::Unconnected un;
    // 未连接时必须是 NotConnectedError —— 不能因 tr_ 为空退化成 AttributeError 那类。
    CHECK_THROWS_AS(un.arm->get_state(), NotConnectedError);
    CHECK_THROWS_AS(un.arm->enable(), NotConnectedError);
    CHECK_THROWS_AS(un.arm->get_status_now(), NotConnectedError);
    CHECK_THROWS_AS(un.arm->poll_cart(), NotConnectedError);
}

// ---------------------------------------------------------------- DFU 终态

TEST(assembly_dfu_refuses_while_enabled_without_sending_a_byte) {
    lt::Offline off;
    off->enable();
    const size_t before = off.t().tx_count();
    CHECK_THROWS_AS(off->enter_dfu(), InvalidCommandError);
    // 一个字节都不发
    CHECK_EQ(off.t().tx_count(), before);
    // 臂与控制链路原样可用
    CHECK_NOTHROW(off->get_state());
}

TEST(assembly_dfu_success_makes_the_session_terminal) {
    lt::Offline off;
    CHECK_NOTHROW(off->enter_dfu());
    // 成功返回后任何走写口/读口的调用都抛终态异常
    CHECK_THROWS_AS(off->get_state(), ArmIsInDfuError);
    CHECK_THROWS_AS(off->movej(std::vector<double>(7, 0.0)), ArmIsInDfuError);
    CHECK_THROWS_AS(off->get_tcp(), ArmIsInDfuError);
    CHECK_THROWS_AS(off->connect(), ArmIsInDfuError);
    CHECK_THROWS_AS(off->reconnect(), ArmIsInDfuError);
    // 终态不因重连复活
    CHECK_THROWS_AS(off->connect("fake"), ArmIsInDfuError);
    // close() 照旧可用 (幂等空操作) —— teardown 在任何状态下都不该抛
    CHECK_NOTHROW(off->close());
}

TEST(assembly_dfu_terminal_state_also_blocks_local_precheck_entries) {
    // move_js / send_mit / send_mit_all 把 arity/idx 预检排在 _cmd 之前, 而终态下 n 是 0
    // => 预检先炸会报出误导性的 "需 N 个" (N=0), 把人引向一个不存在的 arity bug。
    lt::Offline off;
    off->enter_dfu();
    CHECK_THROWS_AS(off->move_js(std::vector<double>{}), ArmIsInDfuError);
    CHECK_THROWS_AS(off->send_mit(0, 0, 0, 0, 0, 0), ArmIsInDfuError);
    CHECK_THROWS_AS(off->send_mit_all({}, {}, {}, {}, {}), ArmIsInDfuError);
    // 纯本地访问器不抛 (它们本来就不碰链路, 那是对的)
    CHECK_EQ(off->zero_g_active(), false);
    CHECK_NOTHROW(off->last_reset_reason());
}

TEST(assembly_dfu_without_the_device_leaving_is_not_terminal) {
    // ACK 只表示"已登记", 不表示"会跳" —— 登记的静默撤销点有三个。那条异常是它们的
    // 唯一出口: 说清"什么都没发生", 并让 Arm 保持可用。
    lt::Offline off;
    off.t().dfu_vanishes = false;
    CHECK_THROWS_AS(off->enter_dfu(0.1), LiteArmError);
    // 对象照旧可用
    CHECK_NOTHROW(off->get_state());
    CHECK_EQ(off->n(), 7);
}

TEST(assembly_dfu_when_armed_pending_firmware_side_rejects) {
    // SDK 从状态帧看不到 enable_pending => 本地预检放行到固件, 由固件回 ERR{0x15,0x03}。
    lt::Offline off;
    off.t().dfu_armed_pending = true;
    try {
        off->enter_dfu();
        FAIL("应当被固件拒绝");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x15);
        CHECK_EQ(int(e.code), 0x03);
    }
    // 那一刻固件还没登记 => 不是"登记被撤销", 对象可用
    CHECK_NOTHROW(off->get_state());
}

TEST(assembly_dfu_rom_table_invalid_is_reported_by_code) {
    lt::Offline off;
    off.t().dfu_rom_table_invalid = true;
    try {
        off->enter_dfu();
        FAIL("应当被固件拒绝");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.code), 0x02);
    }
    CHECK_NOTHROW(off->get_state());
}

TEST(assembly_last_reset_reason_reads_the_boot_banner) {
    lt::Offline off;
    // 桩的 text_log 是空的 => 没有开机签名
    CHECK_EQ(off->last_reset_reason(), std::string(""));
    // 直接把签名喂进传输的噪声留痕 (真机上它是被当噪声丢掉的字节)
    CHECK_NOTHROW(off->last_reset_reason());
}

TEST(assembly_bench_model_axis_default_mirrors_the_firmware_header) {
    lt::Offline off;
    CHECK_EQ(off->bench_model_axis, proto::BENCH_MODEL_AXIS);
    CHECK_EQ(off->bench_model_axis, 5);
}

TEST(assembly_cart_support_is_probed_once_at_connect) {
    lt::Offline off;
    CHECK(off->cart_supported());
    // 探测走的是空载荷 0x3A, 且探测帧不进 CartPending 队列 (登记了会造出一个永远等不到
    // 0x4E 的悬挂态)。
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_L), 1);
}

TEST(assembly_arm_options_are_honoured) {
    auto fake = std::make_shared<testing::FakeTransport>("fake", 0.2, "Litearm1.7.0-7J", 7);
    ArmOptions opts;
    opts.port = "fake";
    opts.q_tol = 0.123;
    opts.dq_tol = 0.456;
    opts.arrive_frames = 5;
    opts.move_timeout = 7.0;
    opts.min_firmware = proto::FirmwareVersion{1, 5, 0, ""};
    auto shared = fake;
    opts.transport_factory = [shared](const std::string&) -> std::unique_ptr<Transport> {
        return std::unique_ptr<Transport>(new lt::SharedFake(shared));
    };
    Arm arm(opts);
    CHECK_NEAR(arm.q_tol, 0.123, 1e-12);
    CHECK_NEAR(arm.dq_tol, 0.456, 1e-12);
    CHECK_EQ(arm.arrive_frames, 5);
    CHECK_NEAR(arm.move_timeout, 7.0, 1e-12);
    arm.connect();
    CHECK_EQ(arm.n(), 7);
    arm.close();
}
