// fork 守卫 —— 子进程里的会话 fail-closed (一个字节都不下发)。
//
// ⚠ 这是本包最"载重"的安全判据之一, 且**只能靠真 fork 测**: 线程不被 fork 复制是操作系统的
// 事实, 任何桩都模拟不出来。
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>

#include "test_support.hpp"

using namespace litearm;

#ifndef _WIN32

namespace {

/// 子进程里跑一段代码, 返回它的退出码 (0 = 子进程判定通过)。
int run_in_child(const std::function<void()>& body) {
    ::fflush(nullptr);
    const pid_t pid = ::fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        // 子进程。⚠ 别用 std::exit (它会跑静态析构) —— 用 _exit。
        int rc = 0;
        try {
            body();
        } catch (...) {
            rc = 1;
        }
        ::_exit(rc);
    }
    int status = 0;
    ::waitpid(pid, &status, 0);
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -1;
}

}  // namespace

TEST(fork_guard_child_process_refuses_every_command) {
    // 子进程**继承 fd** (写照样能到 CDC 上) 但**不继承线程** —— 读线程在子进程里不存在。
    // 于是命令会真的发出去、而应答永远没人读 ⇒ 调用方只看到"无应答"超时 ⇒ 重试 =
    // **重复下发**。所以口径是: 一个字节都不下发, 立刻抛。
    lt::Offline off;
    const size_t before = off.t().tx_count();
    const int rc = run_in_child([&off]() {
        int refusals = 0;
        const auto expect_refusal = [&](const std::function<void()>& f) {
            try {
                f();
            } catch (const ForkedSessionError&) {
                ++refusals;
            } catch (const NotConnectedError&) {
                // 也接受 —— ForkedSessionError 继承自它, 但这条分支不该被走到
                ++refusals;
            }
        };
        // 读路径
        expect_refusal([&]() { off.arm->get_state(); });
        expect_refusal([&]() { off.arm->get_state(true); });
        expect_refusal([&]() { off.arm->get_tcp(); });
        expect_refusal([&]() { off.arm->get_status_now(); });
        // 写路径
        expect_refusal([&]() { off.arm->enable(); });
        expect_refusal([&]() { off.arm->disable(); });
        expect_refusal([&]() { off.arm->emergency_stop(); });
        expect_refusal([&]() { off.arm->movej(std::vector<double>(7, 0.0)); });
        expect_refusal([&]() { off.arm->params().get_joint_param(0); });
        expect_refusal([&]() { off.arm->diag().kin_bench(); });
        expect_refusal([&]() { off.arm->poll_cart(); });
        // 一条都没漏才算通过
        if (refusals != 11) ::_exit(3);
    });
    CHECK_EQ(rc, 0);
    // **零下发** —— 这才是这条守卫要买的东西
    CHECK_EQ(off.t().tx_count(), before);
}

TEST(fork_guard_child_refuses_with_the_specific_exception_type) {
    lt::Offline off;
    const int rc = run_in_child([&off]() {
        try {
            off.arm->get_state();
            ::_exit(1);   // 没抛 = 失败
        } catch (const ForkedSessionError&) {
            ::_exit(0);
        } catch (...) {
            ::_exit(2);   // 抛了别的类型
        }
    });
    CHECK_EQ(rc, 0);
}

TEST(fork_guard_close_in_the_child_is_still_callable) {
    // close() 在子进程里**仍可调** (收尾不能被拦死), 但它**只做会话状态清理**,
    // **不碰传输层** —— 那会取一把从父进程继承来、永远不会释放的锁 ⇒ **永久挂死**。
    // (离线 FakeTransport 没有那把锁, 所以这条判据测的是"不走传输层"而非"不会挂" ——
    //  真机上的那一半由"不碰传输层"这个结构性事实保证。)
    lt::Offline off;
    const int rc = run_in_child([&off]() {
        try {
            off.arm->close();
        } catch (...) {
            ::_exit(1);
        }
        if (off.arm->n() != 0) ::_exit(2);          // 会话状态确实清了
        if (off.arm->firmware().size() != 0) ::_exit(3);
        ::_exit(0);
    });
    CHECK_EQ(rc, 0);
    // 父进程的顺序不因 fork 而改变 —— 它照样要能收尾
    CHECK_NOTHROW(off.arm->get_state());
    off.arm->close();
}

TEST(fork_guard_destroying_the_whole_arm_in_the_child_does_not_hang) {
    // `close()` 走的是 `Arm` 的 fork 分支 (release 掉两个句柄)。而**销毁整个对象**是另一条
    // 路径: `~Arm` -> `close()` -> 之后**成员析构**还要跑 (`cart_` / 各子模块 / 串行锁 …)。
    // 那条路径也必须不挂 —— 它靠的是"只有 condvar 的析构会死锁, 而本对象剩下的成员只涉及
    // malloc/free"。这条用例把那个判断钉住。
    lt::Offline off;
    const int rc = run_in_child([&off]() {
        // 子进程里销毁整个 Arm (析构会兜底 close)
        off.arm.reset();
        if (off.arm != nullptr) ::_exit(1);
        ::_exit(0);
    });
    CHECK_EQ(rc, 0);
}

TEST(fork_guard_a_child_arm_going_out_of_scope_does_not_hang) {
    // 同一个形状, 但走"作用域结束自动析构"—— 用户最可能写成的那一种。
    lt::Offline off;
    const int rc = run_in_child([&off]() {
        // 子进程里要新建一个**自己的**会话 —— 真机上还要求父进程先 close 释放端口,
        // 这里用桩故不涉及。`Arm()` 不带工厂会去开真串口, 故照 `a_fresh_arm_...` 那条
        // 用例的样子注入一份子进程本地的桩。
        testing::FakeTransport fake("fake", 0.2, "Litearm1.7.0-7J", 7);
        auto holder = std::shared_ptr<testing::FakeTransport>(&fake, [](void*) {});
        {
            ArmOptions opts;
            opts.port = "fake";
            opts.transport_factory = [holder](const std::string&)
                -> std::unique_ptr<Transport> {
                return std::unique_ptr<Transport>(new lt::SharedFake(holder));
            };
            opts.move_timeout = 1.0;
            Arm scoped(opts);
            scoped.connect();
            if (scoped.n() != 7) ::_exit(2);
        }                                // 作用域退出 -> ~Arm -> close()
        ::_exit(0);
    });
    CHECK_EQ(rc, 0);
    off.arm->close();
}

TEST(fork_guard_parent_still_works_after_the_child_exits) {
    lt::Offline off;
    const size_t before = off.t().tx_count();
    CHECK_EQ(run_in_child([&off]() { ::_exit(off.arm->n() == 7 ? 0 : 1); }), 0);
    // 父进程的会话一个字节都没被扰动
    CHECK_EQ(off.t().tx_count(), before);
    CHECK_NOTHROW(off.arm->movej(std::vector<double>(7, 0.0)));
}

TEST(fork_guard_a_fresh_arm_in_the_child_is_the_supported_path) {
    // 子进程要用臂 ⇒ 在子进程里**新建一个 Arm** (父进程先 close 释放端口)。
    // 这里验的是: 新对象没有继承来的 pid_ 落章, 于是守卫放行 —— 它有自己的会话。
    lt::Offline off;
    const int rc = run_in_child([&off]() {
        // 在子进程里新建一个 Arm 并连上**同一台桩** (真机上要求父进程先 close)。
        testing::FakeTransport fake("fake", 0.2, "Litearm1.7.0-7J", 7);
        ArmOptions opts;
        opts.port = "fake";
        opts.transport_factory = [&fake](const std::string&) -> std::unique_ptr<Transport> {
            // 子进程里不能共享父进程的对象图, 故这一份是子进程自己构造的转发壳
            return std::unique_ptr<Transport>(new lt::SharedFake(
                std::shared_ptr<testing::FakeTransport>(&fake, [](void*) {})));
        };
        opts.move_timeout = 1.0;
        Arm child(opts);
        child.connect();
        if (child.n() != 7) ::_exit(1);
        child.movej(std::vector<double>(7, 0.0));
        child.close();
        ::_exit(0);
    });
    CHECK_EQ(rc, 0);
    off.arm->close();
}

#else  // _WIN32

TEST(fork_guard_is_a_posix_only_concern) {
    // Windows 没有 fork() —— 这道守卫在那边是纯空操作 (进程模型不同)。
    CHECK(true);
}

#endif  // _WIN32
