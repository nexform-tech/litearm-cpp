// Msg —— 一帧的值 + 它到达的统计 (11 个"读一帧"型 getter 的返回信封)。
#pragma once

#include <string>

namespace litearm {

/// **一帧的值 + 它到达的统计** —— 读一帧型 getter 的返回信封。
///
/// 形态借自松灵的同名结构 ({msg, hz, timestamp}), 语义由本包定:
///   * value     —— 原返回值。取不到帧的入口 (get_state/get_tcp) 用 std::nullopt 表达,
///                  即"没取到"这件事从"返回空"变成"Msg.value 为空";
///   * hz        —— 该类帧在本会话里的平均到达频率, 见 Ack::recv_stats;
///   * timestamp —— 该类帧最近一帧的本地单调时刻 (本会话从没收到过该类帧时为 0.0)。
///
/// 被包的入口: get_state / get_status_now / get_tcp / get_ff_vec / get_ff_scalar /
/// params.get_joint_param / model.get_body / model.get_jm / model.status /
/// model.get_gravity / diag.kin_bench。
///
/// 刻意不包的: move_* 与 home (它们返回动作结果, 不是"读一帧"); 纯本地量
/// (n / firmware / last_reset_reason); license (RSP_LICENSE 没有固件发起的流量);
/// 以及两个派生 getter (get_ff_mask / all_joint_params)。
template <class T>
struct Msg {
    T value{};
    double hz = 0.0;
    double timestamp = 0.0;

    const T& operator*() const { return value; }
    const T* operator->() const { return &value; }
};

}  // namespace litearm
