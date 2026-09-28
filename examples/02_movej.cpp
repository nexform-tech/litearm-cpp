// 样例 02 · movej 关节轨迹 (单发, 固件 S 曲线自完成 + 静止保持)。
//
// 演示:
//   arm.enable()
//   arm.movej(q_target, speed)   一次下发; 固件规划 S 曲线到位并静止保持,
//                                无需 PC 逐帧保活。
//   到位判定: 后端采样状态, 各轴 |q-target|<容差 且 dq≈0 连续 N 帧。
//
// ⚠ 跑完**刻意不 disable**: 该位形失能会因自重坠回, 保持使能持位才是安全终态。
//
// 安全: 需 --go 才 enable+运动。默认目标 = 当前位形小步试探。
//
// 运行:
//   ./02_movej --go                          # 当前位置 + 小步
//   ./02_movej --go 0.1 0 -0.1 0 0 0 0      # 显式目标 (7 角)
#include <cstdio>

#include "_common.hpp"

using namespace litearm;

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 02 · movej 关节轨迹",
        [](Arm& arm, const example::Args& args) -> int {
            const auto st0 = arm.get_state().value;
            if (!args.go) {
                std::printf("只读连接: 加 --go 才会 enable 并运动 (跳过)\n");
                if (st0) std::printf("当前 q = %s\n", example::fmt(st0->q()).c_str());
                return 0;
            }
            arm.enable();
            std::vector<double> target;
            if (!args.rest.empty()) {
                if (int(args.rest.size()) != arm.n()) {
                    std::printf("需 %d 个关节角, 给了 %zu 个\n", arm.n(), args.rest.size());
                    return 1;
                }
                target = args.rest;
            } else {
                target = st0 ? st0->q() : std::vector<double>(size_t(arm.n()), 0.0);
                if (target.size() > 2) target[2] += 0.1;   // 默认小步: J3 +0.1
                std::printf("默认目标 = 当前位置 + 小步: %s\n",
                            example::fmt(target).c_str());
            }
            arm.movej(target, args.speed);
            const auto st = arm.get_state().value;
            if (st) {
                std::printf("movej 到位  q = %s\n", example::fmt(st->q()).c_str());
                std::printf("          tau = %s\n", example::fmt(st->tau(), 2).c_str());
            }
            return 0;
        },
        []() { std::printf("断开 (保持使能持位)\n"); });
}
