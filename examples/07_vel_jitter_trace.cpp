// 样例 07 · 一次运动的逐拍采集 (速度环抖动分析用, 真机, 需 --go)。
//
// 做什么: 从当前位形出发, 让末端**水平**平移一段距离, 并同时录**两路**互补的数据
// (速度环的输入/输出 vs 反馈):
//
//   路① `<tag>_status.csv` —— **100Hz 实测流**, 从状态帧取。给速度环补上**反馈**那一路
//        (实测位置/速度只在这里有)。
//        列: t, seq, mode, flags, joint_fault, q0..6, dq0..6, tau0..6
//        靠 seq 逐帧对账 (丢帧如实报数, 不掩盖)。走的是有间歇掉帧的 CDC 上行。
//
//   路② `<tag>.csv` —— **300Hz 固件日志** (CMD_LOG_CTRL 0x2D / CMD_LOG_READ 0x2E),
//        固件侧逐拍记录, 绕开 CDC 掉帧:
//        列: tick, t, q_ref0..6, dq0..6, tau0..6
//        tick = 控制拍号 (300Hz), q_ref = **参考**位置, dq = **指令**速度,
//        tau = **实测**力矩。
//        ⚠ 没有实测位置/速度 —— 那两路只在路①里。
//        ⚠ 出厂 ff_mask 一般含 FF_VELREF(0x100), 固件把 kd·dq 移进 tau 并令 dq_s=0,
//          **且这发生在打点之前** => 出厂配置下日志的 dq 列恒为 0 (tau 列仍是真话)。
//          要看真实 dq_s 就用 `--ff-mask 0x0BF` 把该位清掉。
//
// ⚠ 缓冲区体检: 固件 LOG_MAX_SAMPLES=2400 (约 8s@300Hz)。装不下时固件写满自停、运动
//   照走, 结果是"数据被截断但不报错" —— 所以本样例显式核对录到的拍数并报出来。
//
// ⚠ 与前身 (Python 版 07) 的差别: 那一版还带**方向 IK 预检 + 终点软限位余量门槛**与
//   四段 A/B 序列 (`--legs`)。本样例只保留"一次平移 + 双路采集"这条主干, 把方向选择
//   交给调用方 —— 预检要用到 PC 侧模型, 而本包的定位是"不做 PC 侧运动学"。
//
// 安全: 默认只读, 加 --go 才动。跑完**刻意不 disable**: 该位形失能会因自重坠回,
//       保持使能持位才是安全终态。急停在手边。
//
// 运行:
//   ./07_vel_jitter_trace                          # 只读: 打印计划
//   ./07_vel_jitter_trace --go                     # 真跑 (默认 40mm)
//   ./07_vel_jitter_trace --go --dist 0.1 --dur 2.5
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "_common.hpp"

using namespace litearm;

namespace {

// 固件 LOG_MAX_SAMPLES / LITEARM_CTRL_HZ —— 直接用库里的常量, 免得两处各写一份。
// (别名带 k 前缀: `using namespace litearm` 之下叫 CTRL_HZ 会与库里的那个撞成二义。)
constexpr int LOG_TICKS = litearm::LOG_MAX_SAMPLES;
constexpr double kCtrlHz = double(litearm::CTRL_HZ);

/// 方向: 0=+X 1=+Y 2=-X 3=-Y (世界系水平四向)。
const char* axis_name(int dir) {
    switch (dir) {
        case 0: return "+X";
        case 1: return "+Y";
        case 2: return "-X";
        default: return "-Y";
    }
}

void apply_dir(int dir, std::array<double, 6>* p, double d) {
    switch (dir) {
        case 0: (*p)[0] += d; break;
        case 1: (*p)[1] += d; break;
        case 2: (*p)[0] -= d; break;
        default: (*p)[1] -= d; break;
    }
}

/// 把 100Hz 状态流的逐帧差异写成 CSV; 返回 (写出的帧数, 丢帧数)。
std::pair<int, int> dump_status_csv(const std::string& path,
                                    const std::vector<std::pair<double, RobotState>>& rows) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) return {0, 0};
    std::fprintf(f, "t,seq,mode,flags,joint_fault,q0,q1,q2,q3,q4,q5,q6,"
                    "dq0,dq1,dq2,dq3,dq4,dq5,dq6,tau0,tau1,tau2,tau3,tau4,tau5,tau6\n");
    int written = 0;
    int gaps = 0;
    long prev_seq = -1;
    for (const auto& row : rows) {
        const RobotState& st = row.second;
        if (prev_seq >= 0) {
            const long delta = (long(st.seq) - prev_seq) & 0xFFFF;
            if (delta != 1) ++gaps;      // 丢帧如实报数, 不掩盖
        }
        prev_seq = long(st.seq);
        std::fprintf(f, "%.4f,%u,%d,%u,%u", row.first, st.seq, st.mode, st.flags,
                     st.joint_fault);
        for (double v : st.q()) std::fprintf(f, ",%.6f", v);
        for (double v : st.dq()) std::fprintf(f, ",%.6f", v);
        for (double v : st.tau()) std::fprintf(f, ",%.6f", v);
        std::fprintf(f, "\n");
        ++written;
    }
    std::fclose(f);
    return {written, gaps};
}

}  // namespace

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 07 · 一次运动的双路逐拍采集",
        [](Arm& arm, const example::Args& args) -> int {
            // 本样例私有开关: --dist / --dur / --dir / --tag / --ff-mask
            const double dist = args.num("dist", 0.040);
            const double dur = args.num("dur", 1.5);
            const int dir = int(args.num("dir", 0.0));
            const std::string tag = args.str("tag", "trace");
            const bool set_ff_mask = args.has("ff-mask");
            const uint32_t ff_mask = uint32_t(args.num("ff-mask", 0x0BF));

            const auto tcp = arm.get_tcp().value;
            const auto st = arm.get_state().value;
            if (!tcp || !st) {
                std::printf("取不到状态或 TCP\n");
                return 1;
            }
            std::array<double, 6> goal = *tcp;
            apply_dir(dir, &goal, dist);

            // 时间预算: 固件是 300Hz 逐拍记录的, 运动时长 * 300 必须落在缓冲里。
            const int expect_ticks = static_cast<int>(dur * kCtrlHz * 1.3) + 60;
            std::printf("计划: 沿 %s 平移 %.0fmm (speed=%.2f, 约 %.1fs)\n", axis_name(dir),
                        dist * 1000.0, args.speed, dur);
            std::printf("      当前 TCP %s\n", example::fmt(*tcp, 4).c_str());
            std::printf("      目标 TCP %s\n", example::fmt(goal, 4).c_str());
            std::printf("      预计录到 %d 拍 (固件上限 %d 拍 ≈ %.1fs)\n", expect_ticks,
                        LOG_TICKS, LOG_TICKS / kCtrlHz);
            if (expect_ticks > LOG_TICKS) {
                std::printf("⚠️  预计超出固件缓冲 —— 数据会被**静默截断**。请减小 --dur。\n");
            }
            if (!args.go) {
                std::printf("\n只读: 加 --go 才真跑 (不改电机状态)\n");
                return 0;
            }
            if (!arm.cart_supported()) {
                std::printf("本机固件没编进 LITEARM_CART_PLAN —— 没有可用的直线入口, 退出\n");
                return 1;
            }
            if (set_ff_mask) {
                std::printf("\n先写 ff_mask=0x%03X (该分支无武装门控, 使能态可写)\n",
                            ff_mask);
                arm.set_ff_mask(ff_mask);
            }

            arm.enable();

            // ---- 路①: 起一个采集线程, 按 100Hz 抓状态帧 ----
            std::vector<std::pair<double, RobotState>> rows;
            std::atomic<bool> stop{false};
            std::thread grabber([&]() {
                const double t0 = now_s();
                while (!stop.load()) {
                    try {
                        const auto m = arm.get_state(true, 0.05);
                        if (m.value) rows.emplace_back(now_s() - t0, *m.value);
                    } catch (...) {
                        break;
                    }
                }
            });

            // ---- 路②: 开 300Hz 固件日志, 然后发运动 ----
            // ⚠ 线程必须在**所有路径**上 stop+join: move_l 抛异常 (固件拒绝规划、超时
            //   都是正常工况) 时若不收线程, `std::thread` 析构会对 joinable 线程直接
            //   `std::terminate` —— 实测就是 core dump (exit 134), 且 stdout 还在块缓冲里,
            //   错误信息也一起丢。
            CartPlan plan{};
            double move_s = 0.0;
            try {
                arm.log().start(LOG_TICKS);
                const double t_move0 = now_s();
                plan = arm.move_l(goal, args.speed);
                move_s = now_s() - t_move0;
            } catch (...) {
                stop.store(true);
                grabber.join();
                throw;
            }
            stop.store(true);
            grabber.join();

            // 等固件把该录的录完 (逐拍记录, 运动结束时可能还差最后一拍)
            int recorded_ticks = 0;
            try {
                LogReader reader = arm.log().reader(1.0, 3);
                reader.wait_for(expect_ticks, 3.0, 0.02);
                recorded_ticks = int(reader.total_bytes()) / sample_size(arm.n());
            } catch (const LiteArmError& e) {
                std::printf("⚠️  等待录满失败 (%s) —— 按此刻已录到的读回\n", e.what());
            }

            // ---- 落盘 ----
            const std::string status_csv = tag + "_status.csv";
            const std::string log_csv = tag + ".csv";
            const auto sc = dump_status_csv(status_csv, rows);

            FILE* lf = std::fopen(log_csv.c_str(), "w");
            int log_rows = 0;
            if (lf != nullptr) {
                std::fprintf(lf, "tick");
                for (int j = 0; j < arm.n(); ++j) std::fprintf(lf, ",q_ref%d", j);
                for (int j = 0; j < arm.n(); ++j) std::fprintf(lf, ",dq%d", j);
                for (int j = 0; j < arm.n(); ++j) std::fprintf(lf, ",tau%d", j);
                std::fprintf(lf, "\n");
                try {
                    LogReader reader = arm.log().reader(1.0, 3);
                    for (const auto& s : reader.samples()) {
                        std::fprintf(lf, "%u", s.tick);
                        for (double v : s.q_ref) std::fprintf(lf, ",%.6f", v);
                        for (double v : s.dq) std::fprintf(lf, ",%.6f", v);
                        for (double v : s.tau) std::fprintf(lf, ",%.6f", v);
                        std::fprintf(lf, "\n");
                        ++log_rows;
                    }
                } catch (const LiteArmError& e) {
                    std::printf("⚠️  日志读回失败: %s\n", e.what());
                }
                std::fclose(lf);
            }

            std::printf("\n=== 结果 ===\n");
            std::printf("规划: ok=%d n_wp=%d plan=%.1fms settled=%d\n", int(plan.ok),
                        plan.n_wp, double(plan.plan_us) / 1000.0, int(plan.settled));
            std::printf("运动耗时 %.2fs (预计 %.2fs)\n", move_s, dur);
            std::printf("路① %s: %d 帧, %d 处丢帧\n", status_csv.c_str(), sc.first,
                        sc.second);
            std::printf("路② %s: %d 拍 (固件录到 %d 拍)\n", log_csv.c_str(), log_rows,
                        recorded_ticks);
            if (recorded_ticks >= LOG_TICKS - 2) {
                std::printf("⚠️  固件缓冲已写满 —— 数据可能被**静默截断** (减小 --dur)\n");
            }
            const auto final_tcp = arm.get_tcp().value;
            if (final_tcp) {
                std::printf("收尾 TCP: %s\n", example::fmt(*final_tcp, 4).c_str());
            }
            if (set_ff_mask) {
                std::printf("提示: ff_mask 已改成 0x%03X, 需要恢复请写回原值或 reset\n",
                            ff_mask);
            }
            return 0;
        },
        // 刻意不 disable: 该位形失能会因自重坠回。
        []() { std::printf("断开 (保持使能持位是安全终态)\n"); });
}
