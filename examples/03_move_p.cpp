// 样例 03 · move_p 位姿运动 (单 pose, 固件内置 IK + S 曲线)。
//
// 演示:
//   arm.get_tcp()               读当前末端 pos[3]+rpy[3]
//   arm.move_p(pose, speed)     一次下发 pose; 固件后台 DLS-IK 求 q -> S 曲线到位
//                               后端轮询 get_tcp 至 TCP≈目标 (pos 容差/rpy 容差) 判定到位。
//
// 注意: move_p 是【点到点】**关节空间**插值 (非笛卡尔直线), 且**只收单个位姿** ——
// 要末端走直线/圆弧/多路点见 06_cartesian.cpp。
//
// 安全: 需 --go; 默认在当前 TCP 上沿 +X 平移 1cm (小步可回退)。
// ⚠ 跑完刻意不 disable (同 02)。
//
// 运行: ./03_move_p --go
#include <cstdio>

#include "_common.hpp"

using namespace litearm;

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 03 · move_p 位姿运动",
        [](Arm& arm, const example::Args& args) -> int {
            const auto tcp = arm.get_tcp().value;
            if (!tcp) {
                std::printf("取不到当前 TCP\n");
                return 1;
            }
            std::printf("当前 TCP: %s\n", example::fmt(*tcp, 4).c_str());
            if (!args.go) {
                std::printf("只读连接: 加 --go 才会执行 move_p (跳过)\n");
                return 0;
            }
            arm.enable();
            std::array<double, 6> target = *tcp;
            target[0] += 0.01;   // 世界 +X 1cm 小步
            std::printf("move_p 目标: %s\n", example::fmt(target, 4).c_str());
            arm.move_p(target, args.speed);
            const auto tcp2 = arm.get_tcp().value;
            if (tcp2) std::printf("move_p 到位  TCP: %s\n", example::fmt(*tcp2, 4).c_str());
            return 0;
        },
        []() { std::printf("断开 (保持使能持位)\n"); });
}
