// 运行状态 —— 状态帧 (G8) 的面向对象视图。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "litearm/protocol.hpp"

namespace litearm {

/// 一个关节的一拍。
struct JointState {
    double q = 0.0;
    double dq = 0.0;
    double tau = 0.0;
    double t_mos = 0.0;
    double t_coil = 0.0;
    uint8_t err = 0;
};

/// 一帧完整的状态。
struct RobotState {
    int mode = 0;
    std::string mode_name = "INIT";
    uint16_t flags = 0;
    std::vector<std::string> flag_names;
    uint16_t seq = 0;
    std::vector<JointState> joints;
    uint16_t joint_fault = 0;   // 固件 G7 断轴位图 (1.5.0 起上报; 旧布局恒 0)

    int n() const { return int(joints.size()); }

    /// flags bit9 (固件 1.5.0 起上报; 旧固件恒 false, 不可据此判失能)。
    bool enabled() const { return (flags & (1u << proto::FLAG_ENABLED_BIT)) != 0; }

    /// 固件 flags bit10 = 笛卡尔规划/播放进行中。
    ///
    /// 刻意不放进 flag_names —— 真正的护栏是 decode_status 里那个隐式的 range(6):
    /// flag_names 只会去看 bit0..bit5, 于是"往 FLAG_NAMES 加 CART_BUSY 什么都不会发生"。
    /// 但"现在无事发生" = 一条防线写在别人的隐式常量里 —— 一旦有人把那个 6 改大,
    /// bit6..bit10 (mode 三位 + enabled + cart_busy) 会一起变成"故障位"。
    bool cart_busy() const { return (flags & (1u << proto::FLAG_CART_BUSY_BIT)) != 0; }

    /// 固件已断轴(失能)的关节号, 0 基 (如 {1, 3} = J2/J4)。
    std::vector<int> fault_axes() const;

    std::vector<double> q() const;
    std::vector<double> dq() const;
    std::vector<double> tau() const;

    /// 推断值, 不是读到的位 —— 掉线刚性持位锁存 (固件 g_arm.drop_hold) 是否可能在生效。
    ///
    /// 为什么只能是推断: 固件不上报这个量 —— 状态帧里没有它的位, 它也没有自己的
    /// ARM_FLAG_*, 固件里 drop_hold 只出现在门禁处, 没有任何上报路径。SDK 侧唯一能拿到的
    /// 是固件自己写下的一条单向蕴含:
    ///     "ENABLE 门禁无需另设 —— drop_hold 为真时 joint_fault 必非 0"
    /// => 逆否: joint_fault == 0 蕴含 drop_hold == false。反向不成立。所以:
    ///   * 本属性为 false => 确定没有该锁存;
    ///   * 本属性为 true  => 可能在锁存, 也可能只是一次"只降级单轴"的故障 ——
    ///     温度 / 跟随误差 / 限位 / 超速那几类按用户裁决只锁该轴、不置 drop_hold。
    ///
    /// 要确证 drop_hold, 用 0x06 错误码 —— 会带 drop_hold 门禁的那些命令
    /// (0x01/0x03/0x04/0x05/0x07/0x2A 与笛卡尔 5 条 0x3A~0x3E) 抛出的
    /// CommandRejectedError::code == 0x06 就是当拍 drop_hold 为真。
    ///
    /// 名字带 _inferred 是刻意的: 一个叫 drop_hold 的布尔会让人以为它来自状态帧,
    /// 从而写出"它一变就能自恢复"的代码 —— 而固件那边的真实语义恰恰是"必须锁存,
    /// 不能随标志位自恢复"。清它的唯一途径是 reset() / clear_faults()。
    bool drop_hold_inferred() const { return joint_fault != 0; }

    /// FAULT 位 / EMERGENCY / 任一轴被固件断轴 (G7)。
    ///
    /// 固件单轴锁存 (断轴/越限/超速/温度) 不一定置全局 FAULT 位, 只置对应的 flags 位并
    /// 把该轴记进 joint_fault —— 若不看 joint_fault, movej 不会早失败, 只会耗满超时后
    /// 报"未到位"。
    bool faulted() const {
        return (flags & 1u) != 0 || mode == 6 || joint_fault != 0;
    }

    /// 人类可读的故障描述 (安全 flags + 断轴号)。
    std::string fault_detail() const;
};

/// 状态帧 -> RobotState。非法帧抛 TransportError。
RobotState decode_state(const std::vector<uint8_t>& payload);

}  // namespace litearm
