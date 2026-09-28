// 样例 05 · 内置动力学/控制律调参 (RAM 生效, 需 save 才持久)。
//
// 演示:
//   arm.ff_preset(1)             出厂组合: master+G+惯量+科氏+摩擦+积分 (开箱)
//   arm.set_gravity_scale(gs)    重力逐关节缩放 (承重 J2/J3 需 2~4.5x)
//   arm.set_inertia_scale(is)    惯量逐关节缩放
//   arm.set_gravity_vector(g)    换安装/倒装改重力方向
//   arm.set_payload(mass, com)   末端负载 (点质量, com 在 ee 法兰系)
//   arm.save_params()            持久化到 Flash (0x25) —— ⚠ 写 flash, 不可逆
//
// 安全: 需 --go 才真正下发; 调参后建议用 02 的小步 movej 验证手感想硬还是软。
//
// 运行:
//   ./05_ff_tune            # 只打印将发的内容
//   ./05_ff_tune --go       # 下发出厂 preset + 显示性 set
#include <cstdio>

#include "_common.hpp"

using namespace litearm;

namespace {
const std::vector<double> GS{2.7, 2.1, 1.75, 1.85, 2.9, 3.1, 1.2};   // 出厂 yaml
const std::vector<double> IS{2.84, 8.62, 9.5, 2.56, 5.27, 4.02, 3.56};
}  // namespace

int main(int argc, char** argv) {
    return example::run_example(
        argc, argv, "样例 05 · 动力学/控制律调参",
        [](Arm& arm, const example::Args& args) -> int {
            std::printf("将演示的固件 FF 命令 (0x26-0x28/0x31):\n");
            std::printf("  ff_preset(1)                -> master+G+惯量+科氏+摩擦+积分\n");
            std::printf("  set_gravity_scale(%s)\n", example::fmt(GS).c_str());
            std::printf("  set_inertia_scale(%s)\n", example::fmt(IS).c_str());
            std::printf("  set_gravity_vector(0,0,-9.81) / set_payload(...) / save_params()\n");
            if (!args.go) {
                std::printf("只读: 加 --go 才下发 (不改固件参数)\n");
                return 0;
            }
            arm.ff_preset(1);
            arm.set_gravity_scale(GS);
            arm.set_inertia_scale(IS);
            arm.set_gravity_vector({0.0, 0.0, -9.81});
            // 负载演示: 先清空(默认出厂无负载), 注释掉即不清
            // arm.set_payload(0.0, {0.0, 0.0, 0.0});
            arm.save_params();
            std::printf("已下发出厂 preset + gs/is/gravity 并保存到 Flash\n");
            std::printf("读回校验: gravity_scale[0] = %.3f, inertia_scale[0] = %.3f\n",
                        arm.get_ff_vec(7).value[0], arm.get_ff_vec(8).value[0]);
            return 0;
        },
        []() { std::printf("已断开\n"); });
}
