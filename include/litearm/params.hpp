// 关节级运行时参数 —— 固件 CMD_SET_JOINT_PARAM(0x22) / SET_JOINT_LIMITS(0x23) /
// GET_JOINT_PARAM(0x24) / PARAM_RESET(0x36) 的面向对象封装。
//
// 与 FF/动力学调参族 (留在 Arm 上) 的分工: 那一族是前馈/控制律参数
// (0x26/0x27/0x28/0x31 + 读回 0x2B/0x2C); 本模块是关节级 MIT 刚度/阻尼/力矩钳幅与
// 软限位。两者都是 RAM 生效, 持久化统一走 Arm::save_params() (0x25)。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "litearm/msg.hpp"
#include "litearm/transport.hpp"

namespace litearm {

class Arm;

/// 固件 RSP_JOINT_PARAM(0x49) 的 21B 应答: idx + kp,kd,tau_max,q_min,q_max。
struct JointParam {
    int idx = 0;
    double kp = 0.0;
    double kd = 0.0;
    double tau_max = 0.0;
    double q_min = 0.0;
    double q_max = 0.0;

    static JointParam decode(const std::vector<uint8_t>& payload);
};

/// `arm.params()` —— 关节级参数的读写入口 (RAM 生效)。
class JointParams {
public:
    explicit JointParams(Arm* arm) : arm_(arm) {}

    /// 改写单关节 MIT 刚度/阻尼/力矩钳幅 (RAM; 须 save_params() 才持久化)。
    /// 固件侧还有一道值合法性校验, 非法时回 ERR{0x22,0x02}。
    void set_joint_param(int idx, double kp, double kd, double tau_max);

    /// 改写单关节软限位 (RAM)。固件要求 q_min < q_max。
    void set_joint_limits(int idx, double q_min, double q_max);

    /// 读回单关节参数 (与上面两个构成写->读回闭环)。
    /// 返回 Msg 信封 (帧 id RSP_JOINT_PARAM)。单发请求/应答式 => 第一次调用 hz == 0.0。
    Msg<JointParam> get_joint_param(int idx, double timeout = 1.0);

    /// 逐关节读回全部参数 (N 次往返)。
    ///
    /// 返回 vector<JointParam> —— 不是 vector<Msg>: 它是 N 次往返的聚合 (N 帧、N 个
    /// 到达时刻), 一个 hz/timestamp 描述不了它; 要单帧的信封请调 get_joint_param(i)。
    std::vector<JointParam> all_joint_params();

    /// 恢复出厂默认 + 失效 flash (固件 0x36)。
    ///
    /// 固件要求失能态 (擦写窗口 CPU 停顿, 电机不能在无监督下保持使能), 已武装时回
    /// ERR{0x36,0x04}; 本方法不代劳 disable(), 以免替调用方做安全决策。
    void reset_factory();

private:
    int check_idx(int idx) const;
    Arm* arm_;
};

}  // namespace litearm
