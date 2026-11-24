// 样例 08 · 零重力拖动示教 (freedrive) —— 带末端负载(夹爪)。
//
// 演示:
//   arm.set_payload(mass, com)   把末端负载写进固件的动力学模型
//   arm.zero_g()                 进入零重力 + 启动保活, 返回 RAII 会话
//   arm.get_state()              拖动期间照常读 (查询类不被零重力门禁拦)
//
// 为什么负载要紧 (TROUBLESHOOTING §8): 固件的动力学补偿**用**这个质量。末端挂着夹爪
// 却按 0 算, 重力补偿就少一块 —— 拖起来末端朝重力方向沉, 越伸出去越明显。
// 相反方向也会错: 没挂件却填了质量, 臂会被"顶"着往上飘。
// ⚠ 只写 RAM。要持久化得 arm.save_params()，而那个写 Flash、不可逆 —— 本样例不做。
//
// ⚠ 退出前必须把关节推回软限位内 (arm.hpp:359 原文):
//   "零重力模式会跳过位置越限锁存 (否则拖动到限位附近会永久锁死), 但一旦退出,
//    越限锁存立刻生效且重启会重锁。"
//   故本样例在收到结束请求时先查限位: 还有轴越限就**不退出**, 列出越限轴让你推回去,
//   要再按一次回车才强制退出。
//
// ⚠ 进入零重力后臂是**失重**的 —— 它随重力走, 不是定在原地。手扶好再按 --go。
// ⚠ 拖动期间固件拒绝动作命令, 但放行查询; 急停/失能也永远放行 (TROUBLESHOOTING §12)。
//
// 结束方式: 按回车, 或 ^C。
//
// 运行:
//   ./08_zero_g_drag                       # 只读: 报状态/限位/将设的负载, 不进零重力
//   ./08_zero_g_drag --go                  # 真正进入零重力拖动
//   ./08_zero_g_drag --go --seconds 30     # 加个时间上限 (到点自动收尾)
//   ./08_zero_g_drag --go --payload 0.7 --comz 0.03
//   ./08_zero_g_drag --go --payload 1.104 --com 0.035,-0.005,0.054   # 全三维质心
//
// 安全: 需 --go 才设负载并进入零重力。
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "_common.hpp"

using namespace litearm;

namespace {

/// 末端负载默认值: 本机装的 LiteGrip 夹爪 (质量 kg, 质心在 ee 法兰系)。
constexpr double kDefaultPayloadKg = 0.7;
constexpr double kDefaultComZ = 0.03;

std::atomic<int> g_stop_requests{0};

extern "C" void on_sigint(int) { g_stop_requests.fetch_add(1); }

/// Parse "x,y,z" (ee flange frame).  Missing components keep the fallback.
std::array<double, 3> parse_com(const std::string& text,
                                const std::array<double, 3>& fallback) {
    std::array<double, 3> out = fallback;
    std::istringstream ss(text);
    std::string part;
    int i = 0;
    while (i < 3 && std::getline(ss, part, ',')) {
        out[size_t(i++)] = std::atof(part.c_str());
    }
    return out;
}

struct Limits {
    std::vector<double> lo, hi;
};

Limits fetch_limits(Arm& arm) {
    Limits lim;
    lim.lo.resize(size_t(arm.n()));
    lim.hi.resize(size_t(arm.n()));
    for (int i = 0; i < arm.n(); ++i) {
        const JointParam p = arm.params().get_joint_param(i).value;
        lim.lo[size_t(i)] = p.q_min;
        lim.hi[size_t(i)] = p.q_max;
    }
    return lim;
}

/// 打印越限轴, 返回越限轴数。
int report_violations(const RobotState& st, const Limits& lim) {
    int n = 0;
    for (int i = 0; i < st.n(); ++i) {
        const double q = st.joints[size_t(i)].q;
        const double d_lo = q - lim.lo[size_t(i)];
        const double d_hi = lim.hi[size_t(i)] - q;
        if (d_lo >= 0.0 && d_hi >= 0.0) {
            continue;
        }
        ++n;
        std::printf("    J%-2d q=%+.5f  限位[%+.5f, %+.5f]  在外侧 %.5f rad\n", i + 1, q,
                    lim.lo[size_t(i)], lim.hi[size_t(i)], d_lo < 0.0 ? -d_lo : -d_hi);
    }
    return n;
}

}  // namespace

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv,
        "样例 08 · 零重力拖动示教 (带末端负载)\n"
        "  --go              真正设置负载并进入零重力 (默认只读)\n"
        "  --seconds N       拖动时间上限, 秒 (默认不限)\n"
        "  --payload KG      末端负载质量 (默认 0.7, 即本机夹爪)\n"
        "  --com X,Y,Z       负载质心 x,y,z, ee 法兰系 (推荐, 覆盖 --comz)\n"
        "  --comz M          只给 z 时的简写 (默认 0.03)\n"
        "  回车 / ^C         结束拖动 (会先检查软限位)",
        [](Arm& arm, const example::Args& args) -> int {
            const double mass = args.num("payload", kDefaultPayloadKg);
            std::array<double, 3> com{0.0, 0.0, args.num("comz", kDefaultComZ)};
            if (args.has("com")) com = parse_com(args.str("com", ""), com);
            // 非 const: 时限到点但仍有轴越限时, 这里会清 0 把控制权交回手动。
            double max_seconds = args.num("seconds", 0.0);

            const auto st0 = arm.get_state().value;
            if (!st0) {
                std::printf("取不到状态帧 —— 链路无上行\n");
                return 1;
            }
            const Limits lim = fetch_limits(arm);

            std::printf("当前 q = %s\n", example::fmt(st0->q(), 4).c_str());
            std::printf("flags=");
            if (st0->flag_names.empty()) {
                std::printf("-");
            } else {
                for (size_t i = 0; i < st0->flag_names.size(); ++i) {
                    std::printf("%s%s", i ? "," : "", st0->flag_names[i].c_str());
                }
            }
            std::printf("  enabled=%d faulted=%d\n", st0->enabled() ? 1 : 0,
                        st0->faulted() ? 1 : 0);

            const int viol0 = report_violations(*st0, lim);
            if (viol0 > 0) {
                std::printf("★ 进入前已有 %d 个轴越限。零重力会跳过越限锁存, 所以可以趁\n"
                            "  拖动把它们推回限位内 —— 但退出前必须推回去。\n",
                            viol0);
            }

            if (!args.go) {
                std::printf("\n将设置: payload_mass=%.3f kg, com={%.4f, %.4f, %.4f}\n",
                            mass, com[0], com[1], com[2]);
                std::printf("只读: 加 --go 才真正下发并进入零重力。\n");
                return 0;
            }

            // ── 负载 ──────────────────────────────────────────────────────
            std::printf("\nset_payload(%.3f, {%.4f, %.4f, %.4f})\n", mass, com[0],
                        com[1], com[2]);
            arm.set_payload(mass, com);
            std::printf("  读回 payload_mass = %.4f kg\n", arm.get_ff_scalar(4).value);
            const int mask = arm.get_ff_mask();
            std::printf("  ff_mask = 0x%03X  重力补偿(G,bit1) %s\n", mask,
                        (mask & 0x2) ? "开着" : "★关着 —— 臂不会失重");

            // ── 使能与进入零重力 ──────────────────────────────────────────
            arm.enable();
            std::printf("\nzero_g(): 进入零重力拖动。手扶好 —— 臂现在是失重的。\n");
            auto zg = arm.zero_g();

            std::signal(SIGINT, on_sigint);
            std::thread([] {
                std::string line;
                while (std::getline(std::cin, line)) {
                    g_stop_requests.fetch_add(1);
                }
            }).detach();

            std::printf("\n按回车或 ^C 结束 (会先检查软限位)。★ = 越限轴。\n\n");

            const auto t0 = std::chrono::steady_clock::now();
            int handled = 0;
            for (;;) {
                const double t =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                        .count();

                if (const auto stm = arm.get_state().value) {
                    std::string marks;
                    for (int i = 0; i < stm->n(); ++i) {
                        const double q = stm->joints[size_t(i)].q;
                        if (q < lim.lo[size_t(i)] || q > lim.hi[size_t(i)]) {
                            marks += " J" + std::to_string(i + 1) + "★";
                        }
                    }
                    std::printf("t=%5.1fs  q=%s  %s\r", t, example::fmt(stm->q(), 2).c_str(),
                                marks.empty() ? "全部在限位内  " : ("越限:" + marks).c_str());
                    std::fflush(stdout);
                }

                // 保活写失败 = 会话其实已经掉出零重力。RAII 析构会把这种异常吞掉,
                // 所以必须自己查 —— 否则会以为还在拖动。
                if (!arm.zero_g_active()) {
                    std::printf("\n\n★ 保活已中断, 臂可能已掉出零重力。原因: %s\n",
                                arm.zero_g_error_text().c_str());
                    break;
                }

                const int r = g_stop_requests.load();
                if (r > handled) {
                    std::printf("\n收到结束请求 (t=%.1fs)\n", t);
                    const auto st = arm.get_state().value;
                    const int nviol = st ? report_violations(*st, lim) : 0;
                    if (nviol == 0) {
                        std::printf("限位检查通过, 退出。\n");
                        break;
                    }
                    if (r > handled + 1) {
                        std::printf("★ 仍有 %d 轴越限, 按你的确认强制退出。\n"
                                    "  退出后越限锁存立刻生效, 重启会重锁。\n",
                                    nviol);
                        break;
                    }
                    std::printf("★ 还有 %d 个轴在限位外 —— 先别退出。\n"
                                "  趁还在零重力把它们推回限位内, 然后重新按回车。\n"
                                "  (要无视风险强制退出: 再按一次回车 / ^C)\n",
                                nviol);
                    handled = r;
                }

                if (max_seconds > 0.0 && t >= max_seconds) {
                    std::printf("\n到 --seconds %.1f 上限。\n", max_seconds);
                    const auto st = arm.get_state().value;
                    if (!st || report_violations(*st, lim) == 0) {
                        break;
                    }
                    std::printf("★ 时限到了但仍有轴越限, 继续拖动 —— 推回限位内后按回车。\n");
                    max_seconds = 0.0;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            std::printf("\n退出零重力\n");
            zg.stop();  // 抛出版本: 保活曾中断的话在这里抛出, 不静默吞掉
            const std::string err = arm.zero_g_error_text();
            std::printf("  保活中断记录: %s\n", err.empty() ? "(无)" : err.c_str());
            std::printf("  ⚠ 退出是异步的 —— 稍等片刻再发运动命令 (TROUBLESHOOTING §12)\n");
            return 0;
        },
        []() { std::printf("断开 (保持使能持位)\n"); });
}
