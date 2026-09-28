// 样例 06 · 笛卡尔路径 (固件规划): move_l / move_c / move_path。
//
// 与 03 的区别: 03 的 `move_p(单 pose)` 是固件做**关节空间**插值 (不是直线);
// 本样例这三条把**末端路径**交给固件规划 —— 末端真的沿直线/圆弧/折线走。
//
//   arm.move_l(goal, speed)                末端走**直线** (位置线性 + 姿态 slerp)
//   arm.move_c(start, via, goal, speed)    末端走**圆弧** (三点定圆; via 的姿态被忽略)
//   arm.move_path({p1, p2, ...}, speed)    依次经过多个位姿 (**尖角**)
//
// 位姿三种写法都收: 固件的 [x,y,z,roll,pitch,yaw] / (pos[3], R[3x3]) / 4x4 齐次矩阵。
// 起点不是参数 —— move_l/move_path 由固件取**当前实测 TCP**, move_c 的 start 必须与实测
// TCP 一致 (容差 6mm / 0.03rad, 超差抛 InvalidCommandError 并给出实际值)。
//
// 返回值是 CartPlan。ok/err/n_wp/plan_us 是固件给的规划结果;
// started_busy/settled/q_final/settle_err_rad 只有 wait=true (默认) 才填真值,
// wait=false 时是"**没等**", 不是"没到位"。
// ⚠ ok=true 只说明**固件受理并规划出来了**, 不说明臂停在了目标上 (settled=false 就是
// "轨迹被别的运动作废了") —— 现场要回读 arm.get_tcp() 看真实落点。
//
// ⚠ 三条已知降级 (相对被它取代的 PC 侧规划):
//   * **无拐角倒角**: 协议里没有 blend 字段, 所以多路点过拐角是**尖的**;
//   * **无下发前预览**: 固件没有 dry-run, 0x4E 要发出去才回 —— 圆弧实际扫多大只能
//     **事后**从 n_wp 判读;
//   * **速度预检改由固件做**: 路径的可接受性由固件在规划阶段判, 入口按 plan.err 抛
//     CartesianPlanError。
//
// 安全: 需 --go; 默认在当前 TCP 上沿 +X 平移 1cm 再回程 (小步可回退)。
// ⚠ 跑完刻意不 disable (同 02)。
//
// 运行:
//   ./06_cartesian --go
//   ./06_cartesian --go --speed 0.3     # 更慢
#include <cstdio>

#include "_common.hpp"

using namespace litearm;

namespace {
void show(const char* label, const CartPlan& plan) {
    std::printf("  %s: ok=%d n_wp=%d plan=%.1fms busy=%d settled=%d settle_err=%.4frad\n",
                label, int(plan.ok), plan.n_wp, double(plan.plan_us) / 1000.0,
                int(plan.started_busy), int(plan.settled), plan.settle_err_rad);
}
}  // namespace

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 06 · 笛卡尔路径 (固件规划)",
        [](Arm& arm, const example::Args& args) -> int {
            const auto tcp0 = arm.get_tcp().value;
            if (!tcp0) {
                std::printf("取不到当前 TCP\n");
                return 1;
            }
            std::printf("当前 TCP: %s\n", example::fmt(*tcp0, 4).c_str());
            const std::array<double, 3> pos{tcp0->at(0), tcp0->at(1), tcp0->at(2)};
            const std::array<double, 3> rpy{tcp0->at(3), tcp0->at(4), tcp0->at(5)};

            if (!arm.cart_supported()) {
                std::printf("本机固件没编进 LITEARM_CART_PLAN —— 笛卡尔入口不可用\n");
                return 1;
            }
            if (!args.go) {
                std::printf("只读连接: 加 --go 才会执行 (跳过)\n");
                return 0;
            }

            arm.enable();

            // ---- move_l: 直线 1cm ----
            const std::array<double, 6> goal{pos[0] + 0.010, pos[1], pos[2],
                                             rpy[0], rpy[1], rpy[2]};
            std::printf("\n--- move_l 直线 +1cm ---\n");
            show("move_l", arm.move_l(goal, args.speed));

            // ---- move_c: 小圆弧。⚠ start 必须与**此刻实测** TCP 一致 ----
            std::printf("\n--- move_c 圆弧 ---\n");
            const auto start_opt = arm.get_tcp().value;
            if (start_opt) {
                const std::array<double, 6> start = *start_opt;
                const std::array<double, 6> via{start[0] + 0.008, start[1] + 0.008,
                                                start[2], rpy[0], rpy[1], rpy[2]};
                const std::array<double, 6> end{start[0], start[1] + 0.016, start[2],
                                                rpy[0], rpy[1], rpy[2]};
                const CartPlan plan = arm.move_c(start, via, end, args.speed);
                show("move_c", plan);
                std::printf("    (没有下发前预览: 扫了多大弧只能从 n_wp=%d 事后判读)\n",
                            plan.n_wp);

                // ---- move_path: 两个路点, 尖角 (协议没有倒角字段) ----
                const std::array<double, 6> w1{start[0] + 0.010, start[1] + 0.005,
                                               start[2], rpy[0], rpy[1], rpy[2]};
                const std::array<double, 6> w2{start[0] + 0.010, start[1] + 0.015,
                                               start[2], rpy[0], rpy[1], rpy[2]};
                std::printf("\n--- move_path 2 路点 (尖角, 无倒角) ---\n");
                show("move_path", arm.move_path({w1, w2}, args.speed));
            }

            // ---- 回起点 ----
            std::printf("\n回起点...\n");
            show("move_l", arm.move_l(*tcp0, args.speed));
            const auto final_tcp = arm.get_tcp().value;
            if (final_tcp) {
                std::printf("收尾 TCP: %s\n", example::fmt(*final_tcp, 4).c_str());
            }
            return 0;
        },
        []() { std::printf("断开 (保持使能持位)\n"); });
}
