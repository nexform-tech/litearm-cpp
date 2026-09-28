// 客户端预检 —— **发帧之前**的本地拒发 (⚠ 与 Python 参照分叉, 见 README)。
//
// 本文件钉四件事:
//   ① 非有限目标 (NaN / ±inf) 必须拦 —— 且**与软限缓存空不空无关**;
//   ② 软限越限必须拦 —— 但**只在缓存可用时**; 缓存不可用一律 fail-open (放行);
//   ③ 缓存本身的两条纪律: 读不到 ⇒ 弃用; **倒置/非有限 ⇒ 整份弃用**(不是跳那一轴);
//   ④ `home()` **刻意豁免** —— 臂漂出软限位后, 恢复路径不该被自己的预检堵死。
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
std::vector<double> zeros7() { return std::vector<double>(7, 0.0); }
}  // namespace

// ---------------------------------------------------------------- 软限缓存

TEST(precheck_connect_builds_the_soft_limit_cache) {
    // 桩的关节参数默认是 q∈[-3, 3] (见 `testing.hpp` 的 jp_q_min/jp_q_max)。
    lt::Offline off;
    const int before = count_cmd(off.t(), proto::CMD_GET_JOINT_PARAM);
    CHECK_EQ(before, 7);   // 连接期读回 7 条 ⇒ 缓存已建

    // 界内 ⇒ 放行 (照常下发)
    CHECK_NOTHROW(off->movej(std::vector<double>(7, 2.5)));
    // 界外 ⇒ 本地拒发, **一帧都不发**
    const int sent = count_cmd(off.t(), proto::CMD_MOVE_J);
    CHECK_THROWS_AS(off->movej(std::vector<double>(7, 3.5)), InvalidCommandError);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_J), sent);   // 没有多出下行帧
}

TEST(precheck_limit_violation_names_the_axis_and_both_bounds) {
    // 文案是这条例外的**全部现场价值**: 得说清哪个轴、什么值、界限多少。
    lt::Offline off;
    try {
        off->movej(std::vector<double>{0.0, 0.0, -4.25, 0.0, 0.0, 0.0, 0.0});
        FAIL("越限的目标必须被本地拒发");
    } catch (const InvalidCommandError& e) {
        const std::string m = e.what();
        CHECK(m.find("第 3 轴") != std::string::npos);   // 轴号
        CHECK(m.find("-4.25") != std::string::npos);     // 目标值
        CHECK(m.find("-3") != std::string::npos);        // 下限
    }
}

TEST(precheck_fails_open_when_the_limits_are_unavailable) {
    // 读不到限位 ⇒ **不预检、放行** (fail-open)。fail-closed 会让所有运动命令全废。
    lt::Unconnected uc;
    uc.fake->err_override[proto::CMD_GET_JOINT_PARAM] = 0x03;   // 固件拒了 0x24
    CHECK_NOTHROW(uc.arm->connect());   // ⚠ 读不到限位**绝不能让连接失败**
    // 一个怎么看都越限的目标 —— 缓存空 ⇒ 放行到固件
    CHECK_NOTHROW(uc.arm->movej(std::vector<double>(7, 99.0)));
}

TEST(precheck_discards_the_whole_cache_when_a_limit_is_inverted) {
    // 倒置的软限 (`q_min > q_max`) 会让 `std::clamp` 撞 UB ⇒ **整份弃用**,
    // 而不是"跳过那一轴" —— 后者会打断 "非空 ⇒ 每条都有限且不倒置" 这条不变式。
    lt::Unconnected uc;
    uc.fake->set_joint_limits(5.0, -5.0);   // 倒置
    CHECK_NOTHROW(uc.arm->connect());
    CHECK_NOTHROW(uc.arm->movej(std::vector<double>(7, 99.0)));   // 缓存已弃用 ⇒ 放行
}

TEST(precheck_discards_the_whole_cache_when_a_limit_is_not_finite) {
    lt::Unconnected uc;
    uc.fake->set_joint_limits(-3.0, std::numeric_limits<double>::infinity());
    CHECK_NOTHROW(uc.arm->connect());
    CHECK_NOTHROW(uc.arm->movej(std::vector<double>(7, 99.0)));
}

// ---------------------------------------------------------------- 非有限值

TEST(precheck_rejects_non_finite_targets_even_with_an_empty_cache) {
    // ⚠⚠ 这是**最要紧的一条**: NaN 与任何数的比较都为假 ⇒ 若让它走到越限比较,
    //   会**静默放行**; 而它进到固件的 `clampf` 同样是 UB。
    //   ⇒ 它必须排在"缓存空 ⇒ 放行"**之前**, 即缓存空也要拦。
    lt::Unconnected uc;
    uc.fake->jp_q_min = 5.0;
    uc.fake->jp_q_max = -5.0;   // 让缓存**必然为空**
    CHECK_NOTHROW(uc.arm->connect());

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double bad : {nan, inf, -inf}) {
        std::vector<double> q = zeros7();
        q[4] = bad;
        CHECK_THROWS_AS(uc.arm->movej(q), InvalidCommandError);
        CHECK_THROWS_AS(uc.arm->movej_sync(q), InvalidCommandError);
    }
    // 且文案点名是哪个轴
    std::vector<double> q = zeros7();
    q[4] = nan;
    try {
        uc.arm->movej(q);
        FAIL("NaN 目标必须被拒");
    } catch (const InvalidCommandError& e) {
        CHECK(std::string(e.what()).find("第 5 轴") != std::string::npos);
    }
}

TEST(precheck_movej_sync_shares_the_limit_check) {
    lt::Offline off;
    const int sent = count_cmd(off.t(), proto::CMD_MOVE_J_SYNC);
    CHECK_THROWS_AS(off->movej_sync(std::vector<double>(7, 3.5)), InvalidCommandError);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_J_SYNC), sent);
}

// ---------------------------------------------------------------- 速度

TEST(precheck_q_is_usable_directly_as_a_dry_run) {
    // ⚠⚠ 它是**公开**入口, 用途写在 `arm.hpp` 里: 调用方可以**预演** ——
    //   "问一句这组目标发出去会不会被本地拒", 而**不必真发一条命令**。
    //
    // 这条用例有两重身份:
    //   ① 那句公开承诺的判据 (它真的能当预演用, 且**一帧都不发**);
    //   ② 补上"公开成员哨兵"抓到的一个**真缺口** —— 有一条用例曾经把 `precheck_q_` 登记
    //      进「已练习」名单, 而它在**全部测试源里一次都没被直接调用过**。
    //      (它此前只被 `movej` / `move_p` **间接**带过, 那不叫"这个入口被练习过"。)
    lt::Offline off;
    const int before = count_cmd(off.t(), proto::CMD_MOVE_J);
    CHECK_NOTHROW(off->precheck_q_(std::vector<double>(7, 0.0), "dry-run"));
    CHECK_THROWS_AS(off->precheck_q_(std::vector<double>(7, 99.0), "dry-run"),
                    InvalidCommandError);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_J), before);   // 预演**一帧都不发**
}

TEST(precheck_speed_is_nan_safe_and_bounds_checked) {
    // ⚠ 判据形态 `!(speed >= 0 && speed <= 1)` 是承重的: 写成 `speed < 0 || speed > 1`
    //   会让 **NaN 静默放行**。
    lt::Offline off;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double bad : {nan, -0.1, 1.1, 1e9}) {
        CHECK_THROWS_AS(off->precheck_speed_(bad, "t"), InvalidCommandError);
    }
    // 边界值必须**放行** (闭区间)
    CHECK_NOTHROW(off->precheck_speed_(0.0, "t"));
    CHECK_NOTHROW(off->precheck_speed_(1.0, "t"));
    CHECK_NOTHROW(off->precheck_speed_(0.5, "t"));
}

TEST(precheck_move_p_rejects_speed_locally_before_sending_anything) {
    // ⚠ 这只速度预检是**六个运动入口里唯一 Python 没有的** —— Python 直接把 speed
    //   打包发走、由固件 `clampf` 静默钳。本仓改成发帧前拒发 ⇒ 与参照分叉。
    lt::Offline off;
    const int sent = count_cmd(off.t(), proto::CMD_MOVE_P);
    CHECK_THROWS_AS(off->move_p(std::array<double, 6>{0.3, 0.0, 0.35, 0.0, 0.0, 0.0}, 2.0),
                    InvalidCommandError);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_P), sent);
}

// ---------------------------------------------------------------- move_p 的容差闸

TEST(precheck_move_p_rejects_bad_tolerances_before_sending_anything) {
    // ⚠⚠ 为什么必须拦: `pose_near` 的第一句是 `if (dp >= pos_tol) return false;`,
    //   而 **`dp >= NaN` 为假** ⇒ **位置那一半被静默跳过** ⇒ 到位只由朝向决定 ⇒
    //   **十米外也判"到位"**(move_p 会当场成功返回, 而臂没到)。
    lt::Offline off;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const std::array<double, 6> goal{0.3, 0.0, 0.35, 0.0, 0.0, 0.0};
    const int sent = count_cmd(off.t(), proto::CMD_MOVE_P);

    CHECK_THROWS_AS(off->move_p(goal, 0.5, nan, 0.03), InvalidCommandError);
    CHECK_THROWS_AS(off->move_p(goal, 0.5, 0.006, nan), InvalidCommandError);
    CHECK_THROWS_AS(off->move_p(goal, 0.5, 0.0, 0.03), InvalidCommandError);    // ≤0
    CHECK_THROWS_AS(off->move_p(goal, 0.5, 0.006, -1.0), InvalidCommandError);
    CHECK_EQ(count_cmd(off.t(), proto::CMD_MOVE_P), sent);   // 一条都没发出去
}

TEST(pose_near_with_nan_tolerance_skips_the_position_half_which_is_why_the_gate_exists) {
    // 这条钉的是**闸门存在的理由本身**(不是闸门): 直接调 `pose_near` 传 NaN 容差,
    // 十米外的目标也会被判成"到位"。⇒ 闸门必须在**进入它之前**拦住。
    // ⚠ 若哪天 `pose_near` 自己加了 fail-closed 兜底, 本条会红 —— 那时应当**同时**
    //   保留两侧(纵深), 而不是删掉闸门; 见 A 侧 `src/pose.cpp` 的两层做法。
    lt::Offline off;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const std::array<double, 6> ten_meters_away{10.0, 0.0, 0.35, 0.0, 0.0, 0.0};
    const std::array<double, 6> goal{0.3, 0.0, 0.35, 0.0, 0.0, 0.0};
    CHECK(TestAccess::pose_near(*off.arm, ten_meters_away, goal, nan, 0.03));
}

// ---------------------------------------------------------------- home 的豁免

TEST(precheck_home_is_exempt_so_the_recovery_path_stays_open) {
    // ⚠⚠ 这是**刻意**的豁免 (A 侧 I-27): `home()` 绝不调 `precheck_q_`。
    //   理由: 目标恒为"全零", 而臂若因为标定/装配漂到了软限位之外, 那正是**最需要
    //   home 的时候** —— 自己的预检把恢复路径堵死是荒谬的。固件侧对 home 有自己的
    //   速度与力矩约束。
    lt::Unconnected uc;
    uc.fake->set_joint_limits(1.0, 2.0);   // 0 **不在**区间内 ⇒ 若走预检必被拒
    CHECK_NOTHROW(uc.arm->connect());
    // 同一个缓存下, movej(0) 被拒…
    CHECK_THROWS_AS(uc.arm->movej(zeros7()), InvalidCommandError);
    // …而 home() 照样发得出去
    CHECK_NOTHROW(uc.arm->home(0.01));
    CHECK_EQ(count_cmd(*uc.fake, proto::CMD_HOME), 1);
}

// ---------------------------------------------------------------- 预检不误伤正常路径

TEST(precheck_does_not_reject_a_normal_motion_sequence) {
    lt::Offline off;
    CHECK_NOTHROW(off->movej(std::vector<double>(7, 0.01), 0.5));
    CHECK_NOTHROW(off->movej_sync(std::vector<double>(7, 0.01), 0.5));
    CHECK_NOTHROW(off->move_p(std::array<double, 6>{0.3, 0.0, 0.35, 0.0, 0.0, 0.0}, 0.5));
    CHECK_NOTHROW(off->home(0.01));
}
