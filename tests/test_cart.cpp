// 笛卡尔固件规划 —— FIFO 配对 / 吸收额度 / 三条入口 / 规划失败映射 / 能力探测。
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
const std::array<double, 6> kP1{0.30, 0.0, 0.40, 3.1416, 0.0, 0.0};
const std::array<double, 6> kP2{0.32, 0.0, 0.42, 3.1416, 0.0, 0.0};
const std::array<double, 6> kP3{0.34, 0.0, 0.44, 3.1416, 0.0, 0.0};

std::vector<uint8_t> plan_reply(bool ok, int err, int n_wp, uint32_t us) {
    std::vector<uint8_t> body{uint8_t(ok ? 1 : 0), uint8_t(err)};
    const auto a = proto::pack_u16le(uint16_t(n_wp));
    const auto b = proto::pack_u32le(us);
    body.insert(body.end(), a.begin(), a.end());
    body.insert(body.end(), b.begin(), b.end());
    return body;
}
}  // namespace

// ---------------------------------------------------------------- CartPlan

TEST(cart_plan_from_reply_parses_the_eight_byte_payload) {
    const CartPlan p = CartPlan::from_reply(plan_reply(true, 0, 12, 4700));
    CHECK(p.ok);
    CHECK_EQ(p.err, 0);
    CHECK_EQ(p.n_wp, 12);
    CHECK_EQ(int(p.plan_us), 4700);
}

TEST(cart_plan_from_reply_rejects_a_short_frame) {
    CHECK_THROWS_AS(CartPlan::from_reply(std::vector<uint8_t>{1, 0, 0}), TransportError);
}

TEST(cart_raise_for_plan_maps_every_err_to_its_own_exception) {
    CartPlan ok;
    ok.ok = true;
    CHECK_NOTHROW(raise_for_plan(ok));       // 成功时什么都不做

    auto mk = [](int err) {
        CartPlan p;
        p.ok = false;
        p.err = err;
        return p;
    };
    CHECK_THROWS_AS(raise_for_plan(mk(CART_ERR_IK)), IKError);
    CHECK_THROWS_AS(raise_for_plan(mk(CART_ERR_COLLINEAR)), CartesianPlanError);
    CHECK_THROWS_AS(raise_for_plan(mk(CART_ERR_TOO_LONG)), CartesianPlanError);
    CHECK_THROWS_AS(raise_for_plan(mk(CART_ERR_LIMIT)), CartesianPlanError);
    CHECK_THROWS_AS(raise_for_plan(mk(CART_ERR_BADARG)), InvalidCommandError);
    // 接管是**预期内的**, 必须与"规划失败"分开: 混进通用异常会让调用方走故障恢复。
    CHECK_THROWS_AS(raise_for_plan(mk(CART_ERR_CANCELED)), MotionSupersededError);
    // 未登记的 err 也归到 CartesianPlanError, 但消息里带上原始码
    try {
        raise_for_plan(mk(99));
        FAIL("应当抛异常");
    } catch (const CartesianPlanError& e) {
        CHECK(std::string(e.what()).find("99") != std::string::npos);
    }
}

TEST(cart_start_tolerances_track_the_move_p_defaults) {
    // move_c 的起点校验与 move_p 的到位判据必须是**同一把尺子**, 否则 move_p 收工的位置
    // 会被 move_c 判成"起点不一致"。
    CHECK_NEAR(CART_START_POS_TOL, 0.006, 1e-12);
    CHECK_NEAR(CART_START_RPY_TOL, 0.03, 1e-12);
    lt::Offline off;
    (void)off;
}

// ---------------------------------------------------------------- 三条入口

TEST(cart_move_l_happy_path_plans_and_settles) {
    lt::Offline off;
    const CartPlan plan = off->move_l(kP2, 0.5);
    CHECK(plan.ok);
    CHECK_EQ(plan.n_wp, 12);
    CHECK_EQ(int(plan.plan_us), 4700);
    // wait=true => 四个收尾字段填真值
    CHECK(plan.started_busy);          // 桩在 0x4E 之后的第 2..7 帧置 bit10
    CHECK(plan.settled);               // 停稳 + 回读 TCP 与目标对得上
    CHECK_EQ(int(plan.q_final.size()), 7);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_L), 2);   // 探测 1 次 + 本次 1 次
}

TEST(cart_move_l_payload_is_pose_then_speed) {
    lt::Offline off;
    off->move_l(kP2, 0.25);
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first != proto::CMD_MOVE_L || kv.second.empty()) continue;
        if (kv.second.size() != 28) continue;   // 跳过探测那条空载荷
        const auto v = proto::unpack_f32s(kv.second.data(), 0, 6);
        CHECK_NEAR(v[0], 0.32, 1e-6);
        const auto sp = proto::unpack_f32s(kv.second.data(), 24, 1);
        CHECK_NEAR(sp[0], 0.25, 1e-6);
    }
}

TEST(cart_move_l_wait_false_only_waits_for_the_plan) {
    lt::Offline off;
    const CartPlan plan = off->move_l(kP2, 0.5, /*wait=*/false);
    CHECK(plan.ok);
    // 这四个字段不是"没到位", 而是"没等"
    CHECK(!plan.started_busy);
    CHECK(!plan.settled);
    CHECK(plan.q_final.empty());
    CHECK_NEAR(plan.settle_err_rad, 0.0, 1e-12);
}

TEST(cart_move_l_validates_speed_and_pose) {
    lt::Offline off;
    CHECK_THROWS_AS(off->move_l(kP2, 1.5), InvalidCommandError);
    CHECK_THROWS_AS(off->move_l(kP2, -0.1), InvalidCommandError);
    CHECK_NOTHROW(off->move_l(kP2, 0.0));
    CHECK_NOTHROW(off->move_l(kP2, 1.0));
    // 位姿形态非法 -> InvalidCommandError (文案里带实际收到的形状)
    rot::Mat3 bad{};
    CHECK_THROWS_AS(off->move_l(rot::PoseInput::from_pos_rot({0, 0, 0}, bad)),
                    InvalidCommandError);
}

TEST(cart_move_l_accepts_all_three_pose_forms) {
    lt::Offline off;
    CHECK_NOTHROW(off->move_l(kP2));
    const auto R = rot::rpy_to_mat({3.1416, 0.0, 0.0});
    CHECK_NOTHROW(off->move_l(rot::PoseInput::from_pos_rot(
        rot::Vec3{0.32, 0.0, 0.42}, R)));
    rot::Mat4 M{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) M[size_t(i)][size_t(j)] = R[size_t(i)][size_t(j)];
    }
    M[0][3] = 0.32;
    M[1][3] = 0.0;
    M[2][3] = 0.42;
    M[3][3] = 1.0;
    CHECK_NOTHROW(off->move_l(M));
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_L), 4);   // 探测 + 三次
}

TEST(cart_move_c_checks_the_start_against_the_measured_tcp) {
    lt::Offline off;
    off.t().set_pose({0.30, 0.0, 0.40, 3.1416, 0.0, 0.0});
    // 起点与实测 TCP 一致 -> 放行
    CHECK_NOTHROW(off->move_c(kP1, kP2, kP3));
    // 起点对不上 -> 响亮拒绝 (固件圆弧的起点恒为实测 TCP)
    const std::array<double, 6> far{9.0, 9.0, 9.0, 0.0, 0.0, 0.0};
    try {
        off->move_c(far, kP2, kP3);
        FAIL("应当拒绝");
    } catch (const InvalidCommandError& e) {
        CHECK(std::string(e.what()).find("固件圆弧的起点恒为实测 TCP") != std::string::npos);
    }
}

TEST(cart_move_c_accepts_a_rotation_equivalent_start) {
    // 万向锁附近同一个旋转可以给出差很远的 rpy 分量 —— 起点校验必须补旋转等价判定,
    // 只抄逐分量会在那时永远判失败。
    lt::Offline off;
    const rot::Mat3 R = rot::rpy_to_mat({0.0, 3.14159265 / 2, 0.0});
    // 实测 TCP 报万向锁规范化后的 rpy
    const auto canonical = rot::mat_to_rpy(R);
    off.t().pose = std::vector<double>{0.3, 0.0, 0.4, canonical[0], canonical[1],
                                       canonical[2]};
    // 调用方给的是同一旋转的另一组 rpy (yaw 加了 2pi)
    const std::array<double, 6> start{0.3, 0.0, 0.4, canonical[0], canonical[1],
                                      canonical[2] + 2 * 3.14159265};
    CHECK_NOTHROW(off->move_c(start, kP2, kP3));
}

TEST(cart_move_path_sends_begin_add_run_and_targets_the_last_waypoint) {
    lt::Offline off;
    const CartPlan plan = off->move_path({kP1, kP2, kP3}, 0.5);
    CHECK(plan.ok);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_CART_BEGIN), 1);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_CART_ADD), 3);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_CART_RUN), 1);
    // BEGIN 的载荷 = n u8 + sp f32
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first != proto::CMD_CART_BEGIN) continue;
        CHECK_EQ(kv.second.size(), size_t(5));
        CHECK_EQ(int(kv.second[0]), 3);
    }
    // ADD 的载荷 = idx u8 + pose[6]
    int idx_seen = 0;
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first != proto::CMD_CART_ADD) continue;
        CHECK_EQ(kv.second.size(), size_t(25));
        CHECK_EQ(int(kv.second[0]), idx_seen);
        ++idx_seen;
    }
    // RUN 是空载荷
    for (const auto& kv : off.t().tx_snapshot()) {
        if (kv.first == proto::CMD_CART_RUN) CHECK(kv.second.empty());
    }
}

TEST(cart_move_path_rejects_empty_and_over_capacity_paths) {
    lt::Offline off;
    CHECK_THROWS_AS(off->move_path({}), InvalidCommandError);
    // 33 个路点超固件 CART_MAX_GOAL = 32
    const std::vector<rot::PoseInput> many(33, rot::PoseInput(kP1));
    CHECK_THROWS_AS(off->move_path(many), InvalidCommandError);
    const std::vector<rot::PoseInput> exactly_32(32, rot::PoseInput(kP1));
    CHECK_NOTHROW(off->move_path(exactly_32));
}

TEST(cart_plan_failures_are_mapped_from_the_0x4e_result) {
    struct Case {
        int err;
        int expect;   // 0 = IKError, 1 = CartesianPlanError, 2 = MotionSuperseded,
                      // 3 = InvalidCommandError
    };
    const std::vector<Case> cases = {{CART_ERR_IK, 0},       {CART_ERR_COLLINEAR, 1},
                                     {CART_ERR_TOO_LONG, 1}, {CART_ERR_LIMIT, 1},
                                     {CART_ERR_CANCELED, 2}, {CART_ERR_BADARG, 3}};
    for (const auto& c : cases) {
        lt::Offline off;
        off.t().cart_err_override = c.err;
        bool matched = false;
        try {
            off->move_l(kP2, 0.5);
        } catch (const IKError&) {
            matched = (c.expect == 0);
        } catch (const MotionSupersededError&) {
            matched = (c.expect == 2);
        } catch (const InvalidCommandError&) {
            matched = (c.expect == 3);
        } catch (const CartesianPlanError&) {
            matched = (c.expect == 1);
        }
        CHECK(matched);
        // 规划失败时入口**不返回** CartPlan
    }
}

TEST(cart_unsupported_firmware_fails_closed_without_sending_a_command) {
    lt::Offline off;
    // 关掉编译开关: 5 条整段不在固件里, 空载荷探测会拿到 ERR{0x3A,0x00}
    off.t().cart_supported = false;
    lt::Unconnected un;
    // 重新连一次让探测看到"不支持"
    lt::Offline off2;
    (void)off2;
    (void)un;
    // 更直接的做法: 用一个已经探过且为 false 的会话
    auto fake = std::make_shared<testing::FakeTransport>("fake", 0.2, "Litearm1.7.0-7J", 7);
    fake->cart_supported = false;
    ArmOptions opts;
    opts.port = "fake";
    auto shared = fake;
    opts.transport_factory = [shared](const std::string&) -> std::unique_ptr<Transport> {
        return std::unique_ptr<Transport>(new lt::SharedFake(shared));
    };
    opts.move_timeout = 1.0;
    Arm arm(opts);
    arm.connect();
    CHECK(!arm.cart_supported());
    const size_t before = fake->tx_count();
    CHECK_THROWS_AS(arm.move_l(kP2, 0.5), UnsupportedByFirmwareError);
    // 一个字节都不发 (未确认支持就不发能起规划的命令)
    CHECK_EQ(fake->tx_count(), before);
    arm.close();
}

TEST(cart_probe_silence_is_reported_differently_from_a_confirmed_absence) {
    // 探测那一步把两种"不支持"分开了: 一支该换固件, 另一支该查链路。混成一句话会让现场
    // 无法归因。
    auto fake = std::make_shared<testing::FakeTransport>("fake", 0.2, "Litearm1.7.0-7J", 7);
    fake->unknown_cmds.insert(proto::CMD_MOVE_L);   // 每次探测都回 ERR{0x3A,0x00}
    ArmOptions opts;
    opts.port = "fake";
    auto shared = fake;
    opts.transport_factory = [shared](const std::string&) -> std::unique_ptr<Transport> {
        return std::unique_ptr<Transport>(new lt::SharedFake(shared));
    };
    opts.move_timeout = 1.0;
    Arm arm(opts);
    arm.connect();
    try {
        arm.move_l(kP2, 0.5);
        FAIL("应当抛异常");
    } catch (const UnsupportedByFirmwareError& e) {
        CHECK(std::string(e.what()).find("固件确报不支持") != std::string::npos);
    }
    arm.close();
}

TEST(cart_zero_g_state_rejects_all_three_entry_points_before_registering) {
    lt::Offline off;
    off->zero_g_start();
    CHECK_THROWS_AS(off->move_l(kP2), InvalidCommandError);
    CHECK_THROWS_AS(off->move_c(kP1, kP2, kP3), InvalidCommandError);
    CHECK_THROWS_AS(off->move_path({kP1, kP2}), InvalidCommandError);
    off->zero_g_stop();
}

TEST(cart_poll_cart_returns_nothing_when_no_result_is_waiting) {
    lt::Offline off;
    CHECK(!off->poll_cart().has_value());
}

TEST(cart_poll_cart_claims_a_result_that_nobody_waited_for) {
    lt::Offline off;
    // wait=false 的入口返回后结果已被 wait 取走 (unclaim) —— 这里手工造一条"没人认领"的:
    // 直接往收集器里喂一条 0x4E。
    off.t().push_frame(proto::RSP_CART_PLAN, plan_reply(true, 0, 5, 1234));
    // 读线程会把它交给收集器; 此时队列为空 => 那是"多了一条" => 从读路径炸出来。
    bool caught = false;
    try {
        for (int i = 0; i < 20 && !caught; ++i) {
            off->get_state(true, 0.05);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const LiteArmError& e) {
        caught = true;
        CHECK(std::string(e.what()).find("队列为空") != std::string::npos);
    }
    CHECK(caught);
}

// ---------------------------------------------------------------- 清队与作废

TEST(cart_a_clearing_opcode_invalidates_in_flight_plans) {
    // movej/home/estop/disable 这些 opcode 会让固件静默作废在途规划 —— 一条 0x4E 都不发。
    // SDK 侧必须在**发出前**清掉待配队列, 否则下一个 0x4E 一到就错配。
    CartPending cp(cart_absorb_ttl(1.0));
    const auto tok = cp.register_token();
    CHECK_EQ(cp.pending(), 1);
    const int n = cp.clear_pending("测试作废");
    CHECK_EQ(n, 1);
    CHECK_EQ(cp.pending(), 0);
    // 清队唤醒等待者 (标成"结局未知"), 不让它白等到超时
    CHECK_THROWS_AS(cp.wait(tok, 0.01), CartReplyLostError);
}

TEST(cart_clear_pending_with_nothing_in_flight_does_not_refresh_the_credit) {
    // 零重力保活线程每 40ms 发一条清队 opcode (而它几乎永远是空清队)。若空清队也刷新截止,
    // 额度就永远不过期 —— 此后一条**真**脱同步的应答会被永久静默吞掉。
    CartPending cp(0.05);   // 很短的额度存活时间
    cp.clear_pending();
    cp.clear_pending();
    cp.clear_pending();
    // 等过 TTL, 再喂一条不可归属的 0x4E —— 应该报错而不是被静默吸收
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    CHECK_THROWS_AS(cp.on_reply(plan_reply(true, 0, 1, 1)), LiteArmError);
    CHECK_EQ(int(cp.extra_replies), 1);
}

TEST(cart_the_queue_pairs_replies_in_arrival_order) {
    CartPending cp(1.0);
    const auto t1 = cp.register_token();
    const auto t2 = cp.register_token();
    cp.on_reply(plan_reply(true, 0, 1, 1));      // 第一条配给 t1
    cp.on_reply(plan_reply(true, 0, 2, 2));      // 第二条配给 t2
    CHECK(!cp.pending());
    // 等待者按 token 拿到各自那条 (通过 wait 取回原始载荷)
    // 注意: wait 会去 unclaim, 但这两条已经被 resolve 且进了待认领列。
    CHECK_EQ(int(CartPlan::from_reply(cp.wait(t1, 0.1)).n_wp), 1);
    CHECK_EQ(int(CartPlan::from_reply(cp.wait(t2, 0.1)).n_wp), 2);
}

TEST(cart_an_unmatched_reply_is_loud_and_counted) {
    // 应答比请求多说明固件与主机**已经**错位, 吞掉它就把错位藏了起来。
    CartPending cp(1.0);
    try {
        cp.on_reply(plan_reply(true, 0, 1, 1));
        FAIL("应当抛异常");
    } catch (const LiteArmError& e) {
        CHECK(std::string(e.what()).find("错配") != std::string::npos);
    }
    CHECK_EQ(int(cp.extra_replies), 1);
    // 抛基类而不给它一个专门的异常类型: 没有任何调用方能据它做出正确决定, 也不该被专门 catch。
    CHECK_EQ(int(cp.absorbed_replies), 0);
}

TEST(cart_clearing_grants_absorb_credits_that_swallow_late_replies) {
    // 清队那一刻固件那条规划可能已经跑完、应答早躺在主机 RX 缓冲里 —— 记账被销毁了,
    // 但被清掉的条数留下一份额度。
    CartPending cp(1.0);
    cp.register_token();
    cp.register_token();
    CHECK_EQ(cp.clear_pending(), 2);
    CHECK_NOTHROW(cp.on_reply(plan_reply(true, 0, 1, 1)));
    CHECK_NOTHROW(cp.on_reply(plan_reply(true, 0, 1, 1)));
    CHECK_EQ(int(cp.absorbed_replies), 2);
    // 额度用完了 => 第三条响亮报错
    CHECK_THROWS_AS(cp.on_reply(plan_reply(true, 0, 1, 1)), LiteArmError);
}

TEST(cart_an_inherited_credit_is_consumed_and_does_not_perpetuate_itself) {
    // connect() 重建会话时把旧会话的在途条数当额度继承过来 —— 它至多影响新会话的一条命令,
    // 且**不被超时路径续期** (否则此后每一条命令都会报"结局未知")。
    CartPending cp(1.0, /*absorb=*/1);
    CHECK_NOTHROW(cp.on_reply(plan_reply(true, 0, 1, 1)));   // 吃掉那一格
    CHECK_EQ(int(cp.absorbed_replies), 1);
    CHECK_THROWS_AS(cp.on_reply(plan_reply(true, 0, 1, 1)), LiteArmError);
}

TEST(cart_absorb_credit_expires_lazily) {
    CartPending cp(0.05);
    cp.register_token();
    cp.clear_pending();       // 一格额度, 存活 0.05s
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    CHECK_THROWS_AS(cp.on_reply(plan_reply(true, 0, 1, 1)), LiteArmError);
}

TEST(cart_drop_and_absorb_leaves_a_credit_for_a_late_reply) {
    CartPending cp(1.0);
    const auto tok = cp.register_token();
    cp.drop_and_absorb(tok, /*timed_out=*/false);
    CHECK_EQ(cp.pending(), 0);
    // 固件其实受理了那条命令 => 它的应答还在路上 => 不该从无关读路径炸出 LiteArmError
    CHECK_NOTHROW(cp.on_reply(plan_reply(true, 0, 1, 1)));
}

TEST(cart_timeout_path_does_not_extend_the_credit_when_one_was_already_consumed) {
    // 超时支的唯一例外: 窗口里已经吸收过一条 => 那条就是本条请求的应答, 固件不再欠,
    // 故不补额度 (补了就是**自持**: 实测曾让此后每一条命令都报"结局未知")。
    CartPending cp(1.0);
    const auto tok = cp.register_token();          // 水位线: absorbed_at = 0
    const auto other = cp.register_token();
    cp.drop_and_absorb(other, /*timed_out=*/false);   // 造出一格额度
    // 这一条不可归属的应答被那格额度吸收 (落在 tok 的等待窗口里)
    cp.on_reply(plan_reply(true, 0, 1, 1));
    CHECK_EQ(int(cp.absorbed_replies), 1);
    // tok 的窗口里确实消费过额度 => 那条就是它自己的应答 => 不补
    cp.drop_and_absorb(tok, /*timed_out=*/true);
    // 不补额度 => 下一条不可归属的应答应当响亮报错, 而不是被静默吞掉
    CHECK_THROWS_AS(cp.on_reply(plan_reply(true, 0, 1, 1)), LiteArmError);
}

TEST(cart_read_link_failure_path_still_grants_a_credit) {
    // pump 支 (读链路炸) 根本没等过窗口 => "被吸收的是自己那条"没有依据 => 照旧补一格。
    CartPending cp(1.0);
    const auto tok = cp.register_token();
    cp.on_reply(plan_reply(true, 0, 1, 1));   // 消费一格 (给 tok)
    cp.drop_and_absorb(tok, /*timed_out=*/false);
    // 补了一格 => 下一条应答被吸收而不是报错
    CHECK_NOTHROW(cp.on_reply(plan_reply(true, 0, 1, 1)));
}

TEST(cart_token_timeout_raises_reply_lost_not_a_plain_timeout) {
    // 这里要表达的是"未知结局", 混进通用超时会让调用方按"没生效"去重发。
    CartPending cp(1.0);
    const auto tok = cp.register_token();
    try {
        cp.wait(tok, 0.05);
        FAIL("应当抛异常");
    } catch (const CartReplyLostError& e) {
        CHECK(std::string(e.what()).find("结局未知") != std::string::npos);
    } catch (const MotionTimeoutError&) {
        FAIL("必须是 CartReplyLostError, 不是通用超时");
    }
}

TEST(cart_the_unclaimed_column_is_bounded) {
    // 无界的唯一现实来源是"调用方放弃 token" (ACK 超时路径), 每放弃一条就永久留下一条。
    CartPending cp(1.0);
    for (size_t i = 0; i < kUnclaimedMax + 3; ++i) {
        cp.register_token();
        cp.on_reply(plan_reply(true, 0, int(i), uint32_t(i)));
    }
    CHECK_EQ(int(cp.evicted_unclaimed), 3);
    // claim_resolved 只交付队首, 且一次一条
    int claimed = 0;
    while (cp.claim_resolved()) ++claimed;
    CHECK_EQ(claimed, int(kUnclaimedMax));
}

TEST(cart_claim_resolved_does_not_repeat_delivery) {
    CartPending cp(1.0);
    cp.register_token();
    cp.on_reply(plan_reply(true, 0, 7, 1));
    auto t1 = cp.claim_resolved();
    CHECK(t1 != nullptr);
    CHECK(cp.claim_resolved() == nullptr);   // 同一条结果不会被交付两次
}

TEST(cart_wait_removes_the_result_from_the_unclaimed_column) {
    CartPending cp(1.0);
    const auto tok = cp.register_token();
    cp.on_reply(plan_reply(true, 0, 9, 1));
    CHECK_EQ(int(CartPlan::from_reply(cp.wait(tok, 0.1)).n_wp), 9);
    // 已被 wait 取走 => poll_cart 不该再交付一次
    CHECK(cp.claim_resolved() == nullptr);
}

TEST(cart_drop_removes_the_token_from_both_columns) {
    CartPending cp(1.0);
    const auto tok = cp.register_token();
    cp.drop(tok);
    CHECK_EQ(cp.pending(), 0);
    CHECK(cp.claim_resolved() == nullptr);
}

TEST(cart_request_drops_its_own_token_when_the_write_fails) {
    // 契约: write 抛 TransportError 蕴含整帧未送达 => 摘下自己那个 token 才是干净的。
    CartPending cp(1.0);
    CHECK_THROWS_AS(cp.request([]() { throw TransportError("未送达"); }), TransportError);
    CHECK_EQ(cp.pending(), 0);
}

TEST(cart_request_keeps_the_token_when_the_failure_cannot_prove_non_delivery) {
    // 其余异常证明不了帧没送达 (最现实的是中断落在 flush 之后) => 留在队里:
    // 最坏是后续请求报"结局未知" (安全方向), 而摘掉它会让那条应答配给后面的活 token
    // = **假成功** (危险方向)。
    CartPending cp(1.0);
    CHECK_THROWS_AS(cp.request([]() { throw std::runtime_error("别的东西"); }),
                    std::runtime_error);
    CHECK_EQ(cp.pending(), 1);
}

// ---------------------------------------------------------------- 常量契约

TEST(cart_clears_upon_set_is_exactly_the_twelve_opcodes) {
    const std::set<uint8_t> expect = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                      0x11, 0x12, 0x13, 0x14, 0x2A};
    CHECK(cart_clears_upon() == expect);
    // 0x3A/0x3B/0x3E **不在**此列 —— 它们欠应答, 而且欠两条。
    CHECK(cart_clears_upon().count(0x3A) == 0);
    CHECK(cart_clears_upon().count(0x3B) == 0);
    CHECK(cart_clears_upon().count(0x3E) == 0);
}

TEST(cart_absorb_ttl_has_a_floor_and_only_grows) {
    // 额度是"清队/放弃那一刻那条请求可能还欠一条应答"的通行证, 下限是**正确性条件**。
    CHECK_NEAR(cart_absorb_ttl(1.0), kAbsorbTtlFloor, 1e-12);
    CHECK_NEAR(cart_absorb_ttl(0.0), kAbsorbTtlFloor, 1e-12);
    // 更大的 move_timeout 不该被一条新加的兜底砍短
    CHECK_NEAR(cart_absorb_ttl(30.0), 30.0, 1e-12);
}

TEST(cart_error_code_constants_match_the_firmware_enum) {
    CHECK_EQ(CART_ERR_OK, 0);
    CHECK_EQ(CART_ERR_IK, 1);
    CHECK_EQ(CART_ERR_COLLINEAR, 2);
    CHECK_EQ(CART_ERR_TOO_LONG, 3);
    CHECK_EQ(CART_ERR_LIMIT, 4);
    CHECK_EQ(CART_ERR_CANCELED, 5);
    CHECK_EQ(CART_ERR_BADARG, 6);
    CHECK_EQ(CartPlan::ERR_REPLY_LOST, -1);
}
