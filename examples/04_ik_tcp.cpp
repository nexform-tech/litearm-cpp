// 样例 04 · IK / TCP 查询 (只读, 不需 --go)。
//
// 演示:
//   arm.get_tcp()         当前末端位姿 (固件当前反馈 FK)
//   arm.ik(pose)          pose[6] -> q[7] (固件后台 DLS IK, seed=当前 q)
//   常见组合: 读当前 pose -> 验证 ik(pose) 解回 q 应与当前关节角相近 (自洽)。
//
// 运行: ./04_ik_tcp
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "_common.hpp"

using namespace litearm;

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 04 · IK / TCP 查询",
        [](Arm& arm, const example::Args&) -> int {
            const auto st = arm.get_state().value;
            const auto tcp = arm.get_tcp().value;
            if (!st || !tcp) {
                std::printf("取不到状态或 TCP\n");
                return 1;
            }
            std::printf("当前 q   = %s\n", example::fmt(st->q()).c_str());
            std::printf("当前 TCP = %s\n", example::fmt(*tcp, 4).c_str());

            const std::vector<double> q_ik = arm.ik(*tcp);   // 用当前 pose 反解
            std::printf("\nik(当前pose) = %s\n", example::fmt(q_ik).c_str());
            double dev = 0.0;
            const auto cur = st->q();
            for (size_t i = 0; i < q_ik.size() && i < cur.size(); ++i) {
                dev = std::max(dev, std::fabs(q_ik[i] - cur[i]));
            }
            std::printf("与当前关节角最大偏差 = %.3f rad (自洽应 <~0.05)\n", dev);
            std::printf("\n已断开 (只读)\n");
            return 0;
        });
}
