// 样例 01 · 连接握手 + 只读状态/末端位姿 (安全, 不需要 --go)。
//
// 演示:
//   Arm().connect()      自动找 CDC 并校验固件版本约定 (Litearm>=1.5.0)
//   firmware()/n()       固件版本串 + 关节数
//   get_state()          状态帧: q/dq/tau/温度/err/flags/mode (返回 Msg 信封)
//   get_tcp()            当前末端 pos[3]+rpy[3] (固件 FK, 无运动学模型)
//
// 运行: ./01_hello
#include <cstdio>

#include "_common.hpp"

using namespace litearm;

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 01 · 连接握手 + 只读状态",
        [](Arm& arm, const example::Args&) -> int {
            std::printf("固件: %s  (n=%d 关节)\n", arm.firmware().c_str(), arm.n());

            const auto st_msg = arm.get_state();
            if (!st_msg.value) {
                std::printf("\n[状态] 取不到状态帧 (链路无上行)\n");
                return 1;
            }
            const RobotState& st = *st_msg.value;
            std::printf("\n[状态] mode=%s hz=%.1f flags=", st.mode_name.c_str(), st_msg.hz);
            if (st.flag_names.empty()) {
                std::printf("-");
            } else {
                for (size_t i = 0; i < st.flag_names.size(); ++i) {
                    std::printf("%s%s", i ? "," : "", st.flag_names[i].c_str());
                }
            }
            std::printf("\n");
            std::printf("  q   = %s\n", example::fmt(st.q()).c_str());
            std::printf("  dq  = %s\n", example::fmt(st.dq()).c_str());
            std::printf("  tau = %s\n", example::fmt(st.tau()).c_str());
            if (st.faulted()) {
                std::printf("  ⚠️  检测到 FAULT/EMERGENCY (%s) —— 先排查再运动!\n",
                            st.fault_detail().c_str());
            }

            const auto tcp = arm.get_tcp().value;
            if (tcp) {
                std::printf(
                    "\n[末端] pos = %s  rpy = %s\n",
                    example::fmt(std::vector<double>(tcp->begin(), tcp->begin() + 3), 4)
                        .c_str(),
                    example::fmt(std::vector<double>(tcp->begin() + 3, tcp->end()), 3)
                        .c_str());
            } else {
                std::printf("\n[末端] 取不到 TCP\n");
            }
            std::printf("\n已断开 (只读, 未改电机状态)\n");
            return 0;
        });
}
