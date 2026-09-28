// 「覆盖所有功能」的**可执行断言** —— 固件每条命令都要被某个 SDK 入口**真的发出去过**。
//
// `test_protocol.cpp` 只保证「入口能在表里解析到」, 那是**静态**的。
// 这里做**动态**验证: 在桩固件上把每个入口真跑一遍, 断言固件会收到对应的命令 id。
// 两者缺一不可 —— 属性存在但一调用就炸 (或发错 id), 静态检查看不出来。
//
// (上游对应 tests/test_full_coverage.py。)
#include <typeinfo>

#include "test_support.hpp"

using namespace litearm;

namespace {

std::set<uint8_t> sent_ids(const testing::FakeTransport& t) {
    std::set<uint8_t> out;
    for (const auto& kv : t.tx_snapshot()) out.insert(kv.first);
    return out;
}

std::vector<double> zeros7() { return std::vector<double>(7, 0.0); }

/// `text` 里有没有出现"把 `name` 当函数/方法调用"的形状 (即 `name(` 且前一个字符
/// **不是**标识符字符)。
///
/// ⚠ 那个"前一个字符"的判据是承重的: 没有它, `fnum(` 会被当成 `n(`, 于是 `n` / `log`
///   这类短名字会被**别的东西**顶替成"已练习", 判据静默失效。
bool calls_name(const std::string& text, const std::string& name) {
    const std::string needle = name + "(";
    for (size_t p = text.find(needle); p != std::string::npos; p = text.find(needle, p + 1)) {
        if (p == 0) return true;
        const char prev = text[p - 1];
        if (!(std::isalnum(static_cast<unsigned char>(prev)) || prev == '_')) return true;
    }
    return false;
}

/// 从 `header_rel` 里抽出 `class <cls>` 的 **public 段的方法名** (去掉构造函数/析构)。
///
/// ⚠⚠ **不能只看"以 `;` 或 `{` 结尾的行"** —— 那条规则会漏掉**一行写完的方法体**
///   (`int f() const { return x_; }` 那一行结尾是 `}`)。实测(2026-09-28):
///   漏掉之后判据对它们**静默失效**, 而失效方向正是假绿 —— 往 `ModelParams` 里注入
///   一个**没有任何测试调用**的公开方法, 判据照样绿。同一条缺陷也在 `Arm` 那条上
///   (`is_in_dfu` 这种内联的就是这么漏掉的)。
///
///   ⇒ 改判"**结构**": 类声明的 public 段里**只有声明、没有语句** ⇒ 每个 `标识符(`
///     就是一个方法名。关键字与内嵌类型名 (`std::function<void(int)>` 里的 `void(`)
///     用一个关键字表挡掉。
///
/// ⚠ 与 `Arm` 那条不同, 这里**没有手写名单** —— 名字是**从头文件机械推导**的。
///   那意味着"新加一个公开方法而没写测试"会**立刻红**, 而不是等谁想起来去登记。
std::vector<std::string> public_method_names(const std::string& header_rel,
                                             const std::string& cls) {
    const std::string src = lt::strip_cxx_comments(lt::read_repo_file(header_rel));
    const std::string key = "class " + cls + " {";
    const size_t at = src.find(key);
    if (at == std::string::npos) return {};
    const size_t end = src.find("\n};", at);
    const size_t priv = src.find("private:", at);
    const size_t stop = std::min(end == std::string::npos ? src.size() : end,
                                 priv == std::string::npos ? src.size() : priv);
    const std::string pub = src.substr(at + key.size(), stop - (at + key.size()));

    static const std::set<std::string> kNotMethods = {
        "if", "for", "while", "switch", "return", "sizeof", "decltype", "noexcept",
        // 类型名 —— 它们只会出现在 `std::function<void(int)>` 这类**参数类型**里
        "void", "int", "bool", "char", "float", "double", "long", "short",
        "unsigned", "signed", "const", "static", "explicit",
    };

    std::vector<std::string> out;
    for (size_t i = 0; i < pub.size(); ++i) {
        if (pub[i] != '(') continue;
        size_t e = i;
        while (e > 0 && (std::isalnum(static_cast<unsigned char>(pub[e - 1])) ||
                         pub[e - 1] == '_'))
            --e;
        if (e == i) continue;                        // `(` 前面不是标识符
        if (e > 0 && pub[e - 1] == '~') continue;    // 析构函数
        // ⚠ **构造函数的成员初始化列表**也是 `名字(参数)` 的形状
        //   (`LogReader(...) : arm_(arm), timeout_(timeout) {}`) ⇒ 会把 `arm_` /
        //   `timeout_` 这些**数据成员**误当成方法。实测踩过。
        //   按**结构**挡: 初始化列表里每一条前面是 `:` 或 `,` (中间可有空格),
        //   而方法名前面是类型/空格/`*`/`&`。
        {
            size_t k = e;
            while (k > 0 && (pub[k - 1] == ' ' || pub[k - 1] == '\t' || pub[k - 1] == '\n')) --k;
            if (k > 0 && (pub[k - 1] == ',' || pub[k - 1] == ':')) continue;
        }
        const std::string name = pub.substr(e, i - e);
        if (name == cls) continue;                   // 构造函数不算入口
        if (kNotMethods.count(name) != 0) continue;
        if (std::find(out.begin(), out.end(), name) == out.end()) out.push_back(name);
    }
    return out;
}

}  // namespace

TEST(coverage_every_implemented_command_is_exercised_through_its_entry) {
    lt::Offline off;
    Arm& arm = *off.arm;

    arm.connect();                       // 0x41 固件版本 (幂等; 夹具已握过手)
    arm.get_status_now();                // 0x40 主动取状态

    arm.enable();                                             // 0x10
    arm.movej(std::vector<double>(7, 0.05), 0.3);             // 0x01
    arm.movej_sync(std::vector<double>(7, 0.05), 0.3);        // 0x07 [SYNC] 同步 PTP
    arm.move_p({0.30, 0.0, 0.35, 0.0, 0.0, 0.0}, 0.3);        // 0x02
    arm.move_js(zeros7(), zeros7());                          // 0x03
    arm.move_js(zeros7(), zeros7(), zeros7());                // 0x03 (带 tau_ff)
    arm.send_mit(0, 0.0, 0.0, 50.0, 2.0, 0.0);                // 0x04
    arm.send_mit_all(zeros7(), zeros7(), std::vector<double>(7, 50.0),
                     std::vector<double>(7, 2.0), zeros7());  // 0x05
    arm.set_motion_mode(0);                                   // 0x20
    arm.set_speed(50);                                        // 0x21
    arm.ik({0.30, 0.0, 0.35, 0.0, 0.0, 0.0});                // 0x42
    arm.get_tcp();                                            // 0x43
    arm.set_ff_vec(1, zeros7());                              // 0x26
    arm.set_ff_scalar(1, 0, 0.0);                             // 0x28
    arm.set_ff_mask(proto::FF_ALL);                           // 0x27
    arm.ff_preset(1);                                         // 0x31
    arm.get_ff_vec(1);                                        // 0x2B
    arm.get_ff_scalar(1, 0);                                  // 0x2C
    arm.home();                                               // 0x2A
    arm.params().set_joint_param(0, 50.0, 2.0, 10.0);         // 0x22
    arm.params().set_joint_limits(0, -3.0, 3.0);              // 0x23
    arm.params().get_joint_param(0);                          // 0x24
    arm.log().start(4);                                       // 0x2D
    arm.log().reader().read_all();                            // 0x2E
    arm.log().stop();                                         // 0x2D (n=0)
    arm.diag().kin_bench();                                   // 0x49
    {
        auto zg = arm.zero_g();                               // 0x06
    }

    // ---- [笛卡尔] 固件原生规划 (0x3A/0x3B/0x3C/0x3D/0x3E) ----
    // ⚠ **必须留在这段使能窗口内**: 下面 `disable()` 之后固件侧三道门禁会回
    // `ERR{cmd,0x03}` (未使能) —— 那是**固件**的行为, 不是桩的。真机上放到下面就是一次
    // `CommandRejectedError`。
    // `move_c` 的 pose_start 必须与**实测 TCP** 一致 (上面的 move_p 已把桩的 pose 设成它)。
    arm.move_l({0.30, 0.0, 0.35, 0.0, 0.0, 0.0}, 0.3);                      // 0x3A
    arm.move_c({0.30, 0.0, 0.35, 0.0, 0.0, 0.0},                            // 0x3B
              {0.30, 0.0, 0.40, 0.0, 0.0, 0.0},
              {0.32, 0.0, 0.40, 0.0, 0.0, 0.0}, 0.3);
    arm.move_path({{0.30, 0.0, 0.35, 0.0, 0.0, 0.0},                        // 0x3C/0x3D/0x3E
                   {0.30, 0.0, 0.40, 0.0, 0.0, 0.0},
                   {0.32, 0.0, 0.40, 0.0, 0.0, 0.0}}, 0.3);

    arm.clear_faults();                                       // 0x13
    arm.reset();                                              // 0x14
    arm.enable();
    arm.disable();                                            // 0x11
    // ⚠ `0x25` **必须留在这段失能窗口内** (固件 `ctrl_is_armed()` => `ERR{0x25,0x04}`)。
    // 它从前排在使能窗口里 —— 桩那时不建模该门禁, 离线全绿, 真机上却是一次货真价实的
    // `CommandRejectedError`。这正是本文件要消灭的那类"会说谎的测试"。
    arm.save_params();                                        // 0x25 (须失能)
    arm.emergency_stop();                                     // 0x12
    arm.params().reset_factory();                             // 0x36 (须失能)

    // 其余公开入口也真跑一遍 (只登记不调用 = 死接口)
    arm.get_state(true);
    arm.get_ff_mask();                                        // 0x2C item 9 只读
    arm.set_gravity_scale(std::vector<double>(7, 1.0));       // 0x26 item 7
    arm.set_inertia_scale(std::vector<double>(7, 1.0));       // 0x26 item 8
    arm.set_payload(0.5, {0.0, 0.0, 0.0});                    // 0x28 item 4/5
    arm.set_gravity_vector({0.0, 0.0, -9.81});                // 0x28 item 6
    arm.park();                                               // 0x20 mode 0
    arm.zero_g_start();                                       // 0x06 显式配对
    arm.zero_g_stop();

    // ---- 动力学模型在线导入 (0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39) ----
    // commit / revert 要求失能 (固件判据 ctrl_is_armed = enabled || enable_pending)
    arm.disable();
    arm.model().probe();                                      // 0x34 (能力探测)
    arm.model().get_body(0);                                  // 0x34
    for (int i = 1; i <= 7; ++i) {                            // 0x30 x7 (body1..7)
        std::vector<double> v{1.0, 0.0, 0.0, 0.0};
        v.resize(MODEL_BODY_PARAMS, 0.0);
        arm.model().set_body(i, v);
    }
    arm.model().set_jm(zeros7());                             // 0x33
    arm.model().get_jm();                                     // 0x35
    arm.model().commit(MODEL_MASK_WRITTEN);                   // 0x32 (掩码须逐位相符)
    arm.model().status();                                     // 0x38
    arm.model().get_gravity(zeros7());                        // 0x39
    arm.model().revert();                                     // 0x37

    // ---- 授权 0x2F 查询 / 0x3F 提交 ----
    // ⚠ `0x3F` 同样**必须留在失能窗口内** —— 固件对已武装的臂回 `ERR{0x3F,0x04}`。
    // 桩默认 activated=true => 这一发得到聚合档 0x02(已存在), 再由 SDK 回读 0x2F 定性后
    // **正常返回** —— 那正是本包要跑的路径。
    arm.license();                                            // 0x2F
    {
        const uint8_t mac[16] = {0};
        arm.activate(1u, 20260101u, 0u, mac, 16);             // 0x3F
    }

    // ---- [DFU] 0x15 —— **必须是最后一条** ----
    // 固件侧它是两段式: ACK 只表示"已登记", 还要等设备真的消失才置终态。桩默认模拟"真跳转"
    // (ACK 之后读路径抛 TransportError), 于是本行正常返回、而 arm 从此进**终端态**。
    // 放在使能中会得到 `ERR{0x15,0x03}`, 故这里必须是失能态。
    arm.enter_dfu();                                          // 0x15 (终端态)

    const auto sent = sent_ids(off.t());                      // 终态前取快照
    off.arm->close();                                         // 终态下仍必须可用 (幂等空操作, 不抛)
    // 终态**不因 reconnect 复活** —— 要接着用臂请新建一个 Arm。
    // (这条钉在这里而不是只写在 test_arm_assembly: 它是"终端态"最容易被顺手放宽的一处)
    CHECK_THROWS_AS(off.arm->reconnect(), ArmIsInDfuError);

    // ---- 判据: 覆盖表里的每一条都真的出去过 ----
    std::vector<std::string> missing;
    for (const auto& c : proto::command_coverage()) {
        if (sent.count(c.cmd) == 0) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "0x%02X(%s)", c.cmd, c.entry);
            missing.push_back(buf);
        }
    }
    if (!missing.empty()) {
        std::string msg = "以下固件命令从未被任何 SDK 入口真正发出:";
        for (const auto& m : missing) msg += " " + m;
        FAIL(msg);
    }
}

TEST(coverage_no_extra_command_ids_are_sent) {
    // 反向: 发出去的 id 必须都在覆盖表里 (防止发了未登记/越界的命令)。
    lt::Offline off;
    Arm& arm = *off.arm;
    arm.connect();
    arm.enable();
    arm.movej(zeros7());
    arm.get_tcp();
    arm.disable();

    std::set<uint8_t> known;
    for (const auto& c : proto::command_coverage()) known.insert(c.cmd);

    std::vector<std::string> unknown;
    for (uint8_t id : sent_ids(off.t())) {
        if (known.count(id) == 0) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "0x%02X", id);
            unknown.push_back(buf);
        }
    }
    if (!unknown.empty()) {
        std::string msg = "发出了未登记的命令 id:";
        for (const auto& u : unknown) msg += " " + u;
        FAIL(msg);
    }
}

TEST(surface_every_api_raises_not_connected_when_offline) {
    // 未连接时**所有**对外 API 都必须抛 `NotConnectedError`。
    // 漏掉这一点, "先写后检"的路径会静默退化成别的异常, 而调用方按错误类型做分支
    // (重连/重试) 的代码会全部失效。
    lt::Unconnected un;
    Arm& arm = *un.arm;
    std::vector<std::string> wrong;
    auto check = [&](const char* name, const std::function<void()>& fn) {
        try {
            fn();
            wrong.push_back(std::string(name) + ": 未报错");
        } catch (const NotConnectedError&) {
            // 期望
        } catch (const std::exception& e) {
            wrong.push_back(std::string(name) + ": " + typeid(e).name() +
                            " (应为 NotConnectedError)");
        }
    };
    // 这一组只要求"响亮地报错", 类型不限 (上游这三条同样抛 InvalidCommandError, 见下)
    auto check_loud = [&](const char* name, const std::function<void()>& fn) {
        try {
            fn();
            wrong.push_back(std::string(name) + ": 未报错");
        } catch (const LiteArmError&) {
            // 期望 (必须落在本包的错误体系内)
        } catch (const std::exception& e) {
            wrong.push_back(std::string(name) + ": " + typeid(e).name() +
                            " (应为 LiteArmError 的派生类)");
        }
    };
    check("get_state", [&] { arm.get_state(); });
    check("get_status_now", [&] { arm.get_status_now(); });
    check("get_tcp", [&] { arm.get_tcp(); });
    check("ik", [&] { arm.ik({0.3, 0, 0.35, 0, 0, 0}); });
    check("get_ff_vec", [&] { arm.get_ff_vec(1); });
    check("get_ff_scalar", [&] { arm.get_ff_scalar(1); });
    check("set_ff_mask", [&] { arm.set_ff_mask(proto::FF_ALL); });
    check("ff_preset", [&] { arm.ff_preset(1); });
    check("save_params", [&] { arm.save_params(); });
    check("zero_g", [&] { auto s = arm.zero_g(); });
    check("enable", [&] { arm.enable(); });
    check("disable", [&] { arm.disable(); });
    check("emergency_stop", [&] { arm.emergency_stop(); });
    check("reset", [&] { arm.reset(); });
    check("clear_faults", [&] { arm.clear_faults(); });
    check("set_speed", [&] { arm.set_speed(50); });
    check("set_motion_mode", [&] { arm.set_motion_mode(0); });
    check("park", [&] { arm.park(); });
    check("movej", [&] { arm.movej(zeros7()); });
    check("movej_sync", [&] { arm.movej_sync(zeros7()); });
    check("home", [&] { arm.home(); });
    // ⚠ **这三条是刻意的例外, 且与上游逐字一致** (实测过 Python: 同样抛
    //   InvalidCommandError)。原因: 它们的**本地 arity 预检排在 `_require()` 之前**,
    //   而未连接时 `n_ == 0` ⇒ `q.size()(7) != 0` ⇒ 先炸 arity。
    //   `movej` 刻意不这样 (它先 `_require()`, 见那里的注释: "拿 n=0 做 arity 校验
    //   只会给出误导性的'需要 0 个关节角'"), 但 `move_js`/`send_mit`/`send_mit_all`
    //   保留了"终态守卫排在最前"的形状 —— 两条都是上游的原样行为, 别顺手统一。
    //   判据只要求"响亮地报错" (本包错误体系内), 调用方不会静默拿到陈旧值。
    check_loud("move_js", [&] { arm.move_js(zeros7()); });
    check_loud("send_mit", [&] { arm.send_mit(0, 0, 0, 0, 0, 0); });
    check_loud("send_mit_all", [&] { arm.send_mit_all(zeros7(), zeros7(), zeros7(),
                                                      zeros7(), zeros7()); });
    // DFU 也是"先判连接": 未连接时第一因是断开, 不是"本对象已进终态"
    check("enter_dfu", [&] { arm.enter_dfu(); });
    check("license", [&] { arm.license(); });
    check("activate", [&] {
        const uint8_t mac[16] = {0};
        arm.activate(1u, 20260101u, 0u, mac, 16);
    });
    check("params.get_joint_param", [&] { arm.params().get_joint_param(0); });
    check("params.set_joint_param", [&] { arm.params().set_joint_param(0, 1, 1, 1); });
    check("params.set_joint_limits", [&] { arm.params().set_joint_limits(0, -1, 1); });
    check("params.reset_factory", [&] { arm.params().reset_factory(); });
    // ⚠ `all_joint_params` **不在这里**: 未连接时 `n_ == 0` ⇒ 循环零次 ⇒ 返回空表,
    //   一次链路都不碰 (上游同款: 实测 Python 也返回 `[]`)。这是**对的** —— 没有关节
    //   就没有参数可读, 不是错误。放在下面单独断言。
    check("model.get_body", [&] { arm.model().get_body(0); });
    check("model.set_body", [&] { arm.model().set_body(1, std::vector<double>(10, 0.0)); });
    check("model.set_jm", [&] { arm.model().set_jm(zeros7()); });
    check("model.get_jm", [&] { arm.model().get_jm(); });
    check("model.commit", [&] { arm.model().commit(0); });
    check("model.revert", [&] { arm.model().revert(); });
    check("model.status", [&] { arm.model().status(); });
    check("model.get_gravity", [&] { arm.model().get_gravity(zeros7()); });
    check("log.start", [&] { arm.log().start(10); });
    check("log.stop", [&] { arm.log().stop(); });
    check("log.reader().read_all", [&] { arm.log().reader().read_all(); });
    check("diag.kin_bench", [&] { arm.diag().kin_bench(); });
    // 笛卡尔三条 + poll_cart 同样先判连接
    check("move_l", [&] { arm.move_l({0.32, 0.0, 0.35, 0.0, 0.0, 0.0}); });
    check("move_c", [&] {
        arm.move_c({0.32, 0.0, 0.35, 0.0, 0.0, 0.0}, {0.30, 0.02, 0.35, 0.0, 0.0, 0.0},
                   {0.31, 0.01, 0.35, 0.0, 0.0, 0.0});
    });
    check("move_path", [&] { arm.move_path({{0.31, 0.0, 0.35, 0.0, 0.0, 0.0}}); });
    check("move_p", [&] { arm.move_p({0.31, 0.0, 0.35, 0.0, 0.0, 0.0}); });
    check("poll_cart", [&] { arm.poll_cart(); });

    // 未连接时"没有关节" ⇒ 空表, 不报错 (与上游一致)
    CHECK(arm.params().all_joint_params().empty());

    if (!wrong.empty()) {
        std::string msg = "未连接时的错误类型不一致:";
        for (const auto& w : wrong) msg += " [" + w + "]";
        FAIL(msg);
    }
}

// `Arm` 的公开方法/属性 —— **名单守卫**用的名单 (两条用例共用, 见下面两处说明)。
static const std::set<std::string> kExercised = {
    "activate", "cart_supported", "clear_faults", "close", "connect", "diag",
    "banner_version", "disable", "disconnect", "emergency_stop", "enable", "enter_dfu",
    "ff_preset",
    "ff_scalar_items", "ff_scalar_ro_items", "ff_vec_items", "firmware", "fw_version",
    "get_ff_mask", "get_ff_scalar", "get_ff_vec", "get_state", "get_status_now",
    "get_tcp", "home", "host_stats", "ik", "is_connected", "is_in_dfu",
    "last_reset_reason", "license",
    "log", "model", "msg_hz", "options",
    "move_c", "move_js", "move_l", "move_p", "move_path", "movej", "movej_sync",
    "n", "params", "park", "poll_cart", "precheck_q_", "precheck_speed_",
    "reconnect", "reset", "save_params",
    "send_mit", "send_mit_all", "set_ff_mask", "set_ff_scalar", "set_ff_vec",
    "set_gravity_scale", "set_gravity_vector", "set_inertia_scale",
    "set_motion_mode", "set_payload", "set_speed", "status_seq",
    "zero_g", "zero_g_active", "zero_g_error",
    "zero_g_error_text", "zero_g_start", "zero_g_stop",
};
// 允许的少量内部/兼容名 + 公开的可调旋钮 (数据成员, 不是入口)
static const std::set<std::string> kAllowedExtra = {
    "port_string",   // 端口字符串 (只读属性; 上游是 _port)
    "q_tol", "dq_tol", "arrive_frames", "move_timeout",   // 可调旋钮
    "bench_model_axis",
};

TEST(coverage_every_public_arm_entry_is_exercised_somewhere) {
    // **名单守卫**: 不许出现没登记进 `kExercised` 的公开成员。
    //
    // ⚠ **措辞要说准**: 本条只做**集合比较** (`public - kExercised - kAllowedExtra`)。
    //   它问的是"这个成员**登记**了没有", **不是**"它真的被调用过没有" —— 后者由
    //   下面那条 `coverage_every_registered_entry_is_actually_called_by_a_test` 负责。
    //   两条**分工不同, 缺一条就是一个假绿通路**: 只有本条 ⇒ 往名单里塞个新名字就能过。
    const std::set<std::string>& exercised = kExercised;
    const std::set<std::string>& allowed_extra = kAllowedExtra;
    // 从 header 机械提取公开成员名, 与上表比对。
    // ⚠ 这里是**源码扫描**, 不是 C++ 反射 (语言没有) —— 与上游同口径。
    // ⚠ 必须**先剥掉注释**: 不剥的话注释里的词 (`sleep` / `ff_clamp` / `activated` …)
    //   会被当成成员名, 这条哨兵就变成了噪声源 (实测踩过)。
    // 公开成员名**从头文件机械推导** (提取器的实现与那条"别再漏掉内联方法体"的说明
    // 见 `public_method_names`)。
    const auto names = public_method_names("include/litearm/arm.hpp", "Arm");
    CHECK(!names.empty());   // 空集上"每个名字都登记过"恒真 —— 空炮形状
    if (names.empty()) return;

    std::vector<std::string> unregistered;
    for (const auto& name : names) {
        if (exercised.count(name) || allowed_extra.count(name)) continue;
        if (std::find(unregistered.begin(), unregistered.end(), name) ==
            unregistered.end()) {
            unregistered.push_back(name);
        }
    }
    if (!unregistered.empty()) {
        std::string msg = "Arm 上出现未登记进 `exercised` 的公开成员:";
        for (const auto& u : unregistered) msg += " " + u;
        msg += " —— 新增公开入口请把它加进本用例的名单 (否则没人会注意到它没被练习过)";
        FAIL(msg);
    }
}

TEST(coverage_every_subobject_public_method_is_called_by_a_test) {
    // ★★ 与 `Arm` 那条**同一目的**, 但**不要手写名单**: 名字从头文件推导 ⇒
    //   "新增公开方法却不写测试"当场红。子对象是四条入口 (`params` / `model` / `log` /
    //   `diag`) 的**实际工作面**, 它们漏测比起 `Arm` 少一层可见性。
    //
    // ⚠ 覆盖的类清单是**手写的** (扫不了"全部类" —— 语言没有反射), 所以要跟头文件走:
    //   新增一个带公开方法的子对象类时, 记得往 `kClasses` 里加一行。
    static const std::vector<std::pair<std::string, std::string>> kClasses = {
        {"JointParams", "include/litearm/params.hpp"},
        {"ModelParams", "include/litearm/model.hpp"},
        {"ArmLog", "include/litearm/log.hpp"},
        {"LogReader", "include/litearm/log.hpp"},
        {"Diagnostics", "include/litearm/diagnostics.hpp"},
    };

    const auto files = lt::list_repo_files("tests", "test_", ".cpp");
    CHECK(!files.empty());
    if (files.empty()) return;
    std::string all;
    for (const auto& f : files) all += lt::strip_cxx_comments(lt::read_repo_file(f)) + "\n";

    std::vector<std::string> problems;
    for (const auto& kv : kClasses) {
        const auto names = public_method_names(kv.second, kv.first);
        if (names.empty()) {
            problems.push_back(kv.first + " (在 " + kv.second + " 里**一个方法名都没解析出来**"
                               " —— 类改名了? 头文件格式变了? 这条判据对它已静默失效)");
            continue;
        }
        for (const auto& n : names) {
            if (!calls_name(all, n)) problems.push_back(kv.first + "::" + n);
        }
    }
    if (!problems.empty()) {
        std::string msg = "以下公开方法在全部测试源里**找不到任何一次调用**:";
        for (const auto& p : problems) msg += " " + p;
        msg += "\n    → 给它补一条测试。⚠ 解析为空也报在这里: 那说明这条判据已经"
               "对它静默失效了 (空集上「每个方法都被调用过」恒真)。";
        FAIL(msg);
    }
}

TEST(coverage_every_registered_entry_is_actually_called_by_a_test) {
    // ★★ 本条补上「名单守卫」缺的那一半。
    //
    // 上面那条只做**集合比较**: 它保证"新公开成员必须登记", 但登记之后**没人检查那个
    // 名字真的被调用过** —— 往名单里添个名字、一条测试都不写, 它照样绿。
    // 那是一条**假绿通路**, 而本仓对假绿的口径是"比没测更糟": 它让"名单里有"读起来
    // 像"已经验过了"。
    //
    // 判据: 逐个在**全部测试源**里找"把这个名字当函数/方法调用"的形状。
    // ⚠ 扫的是**去注释后**的文本 —— 否则注释里提一句就算"练过"。
    // ⚠ 文件清单**跟着目录走** (`list_repo_files`), 不是手写的 —— 手写清单漏一个文件,
    //   那条判据就对那个文件静默失效, 而且失效方向正是假绿。
    const auto files = lt::list_repo_files("tests", "test_", ".cpp");
    CHECK(!files.empty());   // 空集上"每个名字都被调用过"恒真 —— 那是空炮形状
    if (files.empty()) return;

    std::string all;
    for (const auto& f : files) all += lt::strip_cxx_comments(lt::read_repo_file(f)) + "\n";

    std::vector<std::string> not_called;
    for (const auto& name : kExercised) {
        if (!calls_name(all, name)) not_called.push_back(name);
    }
    if (!not_called.empty()) {
        std::string msg =
            "以下成员**登记为已练习、但在全部测试源里找不到任何一次调用**:";
        for (const auto& m : not_called) msg += " " + m;
        msg += "\n    → 要么给它补一条测试, 要么把它从 `kExercised` 里摘掉。"
               "\n    ⚠ 别只是把名字留在名单里: 那正是本条要堵的假绿通路"
               " (名单说「练过」, 而它从没被调用过)。\n    扫过的文件: ";
        for (const auto& f : files) msg += f + " ";
        FAIL(msg);
    }
}
