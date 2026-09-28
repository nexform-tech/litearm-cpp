#include "litearm/params.hpp"

#include "litearm/arm.hpp"

namespace litearm {

JointParam JointParam::decode(const std::vector<uint8_t>& payload) {
    if (payload.size() < 21) {
        throw TransportError("RSP_JOINT_PARAM 帧短 (" + std::to_string(payload.size()) +
                             "B, 期望 21B)");
    }
    JointParam p;
    p.idx = payload[0];
    const auto v = proto::unpack_f32s(payload.data(), 1, 5);
    p.kp = v[0];
    p.kd = v[1];
    p.tau_max = v[2];
    p.q_min = v[3];
    p.q_max = v[4];
    return p;
}

int JointParams::check_idx(int idx) const {
    const int n = arm_->n();
    const int upper = n > 0 ? n : 1;
    if (idx < 0 || idx >= upper) {
        throw InvalidCommandError("关节索引越界: " + std::to_string(idx) + " (有效 0.." +
                                  std::to_string(n - 1) + ")");
    }
    return idx;
}

void JointParams::set_joint_param(int idx, double kp, double kd, double tau_max) {
    idx = check_idx(idx);
    std::vector<uint8_t> payload{uint8_t(idx)};
    const auto v = proto::pack_f32s(std::vector<double>{kp, kd, tau_max});
    payload.insert(payload.end(), v.begin(), v.end());
    arm_->cmd_expect_ack(proto::CMD_SET_JOINT_PARAM, payload,
                         "set_joint_param(J" + std::to_string(idx + 1) + ")");
}

void JointParams::set_joint_limits(int idx, double q_min, double q_max) {
    idx = check_idx(idx);
    if (!(q_min < q_max)) {
        throw InvalidCommandError("软限位需 q_min < q_max (给的是 " +
                                  std::to_string(q_min) + ", " + std::to_string(q_max) +
                                  ")");
    }
    std::vector<uint8_t> payload{uint8_t(idx)};
    const auto v = proto::pack_f32s(std::vector<double>{q_min, q_max});
    payload.insert(payload.end(), v.begin(), v.end());
    arm_->cmd_expect_ack(proto::CMD_SET_JOINT_LIMITS, payload,
                         "set_joint_limits(J" + std::to_string(idx + 1) + ")");
}

Msg<JointParam> JointParams::get_joint_param(int idx, double timeout) {
    idx = check_idx(idx);
    const uint8_t p = uint8_t(idx);
    arm_->write_query(proto::CMD_GET_JOINT_PARAM, &p, 1);   // 查询类: 不受守卫限制
    const auto r = arm_->require().expect(
        proto::RSP_JOINT_PARAM, timeout, "get_joint_param(J" + std::to_string(idx + 1) + ")",
        true, proto::CMD_GET_JOINT_PARAM);
    return arm_->wrap(JointParam::decode(r.payload), proto::RSP_JOINT_PARAM);
}

std::vector<JointParam> JointParams::all_joint_params() {
    std::vector<JointParam> out;
    const int n = arm_->n();
    out.reserve(size_t(n));
    for (int i = 0; i < n; ++i) out.push_back(get_joint_param(i).value);
    return out;
}

void JointParams::reset_factory() {
    // 固件要求失能态 (擦写窗口 CPU 停顿, 电机不能在无监督下保持使能), 已武装时回
    // ERR{0x36,0x04}; 本方法不代劳 disable(), 以免替调用方做安全决策。
    arm_->cmd_expect_ack(proto::CMD_PARAM_RESET, nullptr, 0, "reset_factory", 2.5);
}

}  // namespace litearm
