#include "litearm/model.hpp"

#include "litearm/arm.hpp"

namespace litearm {

ModelStatus ModelStatus::decode(const std::vector<uint8_t>& payload) {
    // payload 含首字节 RSP id (与 RSP_FF_VEC/RSP_FF_SCALAR 同口径)。
    if (payload.size() != 5) {
        throw TransportError("RSP_MODEL_STATUS 帧长 " + std::to_string(payload.size()) +
                             "B, 期望 5B");
    }
    if (payload[0] != proto::RSP_MODEL_STATUS) {
        throw TransportError("RSP_MODEL_STATUS 帧 id 与期望不符");
    }
    ModelStatus s;
    s.override_level = int(payload[1]);
    s.staged_mask = proto::read_u16le(payload, 2);   // u16 LE
    s.dirty = int(payload[4]);
    return s;
}

int ModelParams::check_body_idx(int idx) const {
    if (idx < 0 || idx >= MODEL_NBODY) {
        throw InvalidCommandError("刚体索引越界: " + std::to_string(idx) + " (有效 0.." +
                                  std::to_string(MODEL_NBODY - 1) + ")");
    }
    return idx;
}

bool ModelParams::probe() {
    // 用 0x34 探测: 旧固件没有这条命令的 case => 落 default => ERR{0x34,0x00}
    // => 捕获 UnsupportedByFirmwareError 并回 false。
    // 不能用 0x30/0x33 探测 —— 旧固件对它们有显式 case 回 ERR{cmd,0x02} (与"数值非法"
    // 同码), 无法区分「无此命令」与「参数非法」。
    try {
        get_body(0);
        return true;
    } catch (const UnsupportedByFirmwareError&) {
        return false;
    }
}

Msg<std::vector<double>> ModelParams::get_body(int idx, double timeout) {
    idx = check_body_idx(idx);
    const uint8_t p = uint8_t(idx);
    arm_->write_query(proto::CMD_GET_MODEL_PARAM, &p, 1);
    const auto r = arm_->require().expect(
        proto::RSP_MODEL_PARAM, timeout, "model.get_body(" + std::to_string(idx) + ")",
        true, proto::CMD_GET_MODEL_PARAM);
    const size_t want = 2 + 4 * size_t(MODEL_BODY_PARAMS);
    if (r.payload.size() != want) {
        throw TransportError("RSP_MODEL_PARAM 帧长 " + std::to_string(r.payload.size()) +
                             "B, 期望 " + std::to_string(want) + "B");
    }
    if (r.payload[0] != proto::RSP_MODEL_PARAM) {
        throw TransportError("RSP_MODEL_PARAM 帧 id 与期望不符");
    }
    if (r.payload[1] != uint8_t(idx)) {
        throw TransportError("RSP_MODEL_PARAM 回显 body_idx=" +
                             std::to_string(r.payload[1]) + ", 期望 " +
                             std::to_string(idx));
    }
    return arm_->wrap(proto::unpack_f32s(r.payload.data(), 2, MODEL_BODY_PARAMS),
                      proto::RSP_MODEL_PARAM);
}

void ModelParams::set_body(int idx, const std::vector<double>& vals) {
    idx = check_body_idx(idx);
    if (int(vals.size()) != MODEL_BODY_PARAMS) {
        throw InvalidCommandError(
            "body 参数须为 " + std::to_string(MODEL_BODY_PARAMS) +
            " 个 (m, cmx, cmy, cmz, ixx, ixy, ixz, iyy, iyz, izz), 给的是 " +
            std::to_string(vals.size()) + " 个");
    }
    std::vector<uint8_t> payload{uint8_t(idx)};
    const auto v = proto::pack_f32s(vals);
    payload.insert(payload.end(), v.begin(), v.end());
    arm_->cmd_expect_ack(proto::CMD_SET_MODEL_PARAM, payload,
                         "model.set_body(" + std::to_string(idx) + ")");
}

Msg<std::vector<double>> ModelParams::get_jm(double timeout) {
    arm_->write_query(proto::CMD_GET_MODEL_JM);
    const auto r = arm_->require().expect(proto::RSP_MODEL_JM, timeout, "model.get_jm",
                                          true, proto::CMD_GET_MODEL_JM);
    const size_t want = 1 + 4 * size_t(MODEL_JM_N);
    if (r.payload.size() != want) {
        throw TransportError("RSP_MODEL_JM 帧长 " + std::to_string(r.payload.size()) +
                             "B, 期望 " + std::to_string(want) + "B");
    }
    if (r.payload[0] != proto::RSP_MODEL_JM) {
        throw TransportError("RSP_MODEL_JM 帧 id 与期望不符");
    }
    return arm_->wrap(proto::unpack_f32s(r.payload.data(), 1, MODEL_JM_N),
                      proto::RSP_MODEL_JM);
}

void ModelParams::set_jm(const std::vector<double>& vals) {
    if (int(vals.size()) != MODEL_JM_N) {
        throw InvalidCommandError("jm 须为 " + std::to_string(MODEL_JM_N) + " 个, 给的是 " +
                                  std::to_string(vals.size()) + " 个");
    }
    arm_->cmd_expect_ack(proto::CMD_SET_MODEL_JM, proto::pack_f32s(vals), "model.set_jm");
}

void ModelParams::commit(uint16_t expected_mask) {
    // ⚠ u16 LE —— 没有 u16 pack helper 时写成单字节会截断 (0x2FE & 0xFF = 0xFE) 而固件
    // len < 2 直接拒。
    arm_->cmd_expect_ack(proto::CMD_MODEL_COMMIT, proto::pack_u16le(expected_mask),
                         "model.commit", 2.5);
}

void ModelParams::revert() {
    arm_->cmd_expect_ack(proto::CMD_REVERT_MODEL, nullptr, 0, "model.revert", 2.5);
}

Msg<ModelStatus> ModelParams::status(double timeout) {
    arm_->write_query(proto::CMD_GET_MODEL_STATUS);
    const auto r = arm_->require().expect(proto::RSP_MODEL_STATUS, timeout, "model.status",
                                          true, proto::CMD_GET_MODEL_STATUS);
    return arm_->wrap(ModelStatus::decode(r.payload), proto::RSP_MODEL_STATUS);
}

Msg<std::vector<double>> ModelParams::get_gravity(const std::vector<double>& q,
                                                  double timeout) {
    // 给定关节角算重力项 G(q) (纯读, 无门控, 不改任何状态)。
    // 产线的「静态重力核查」用它: 到位静止后比对实测 tau 与 G(q) —— 这是唯一能抓住
    // 「错台 yaml / 重力符号错 / 模型没生效」的判据 (读回比对只能证明字节落位)。
    if (int(q.size()) != MODEL_JM_N) {
        throw InvalidCommandError("q 须为 " + std::to_string(MODEL_JM_N) + " 个, 给的是 " +
                                  std::to_string(q.size()) + " 个");
    }
    arm_->write_query(proto::CMD_GET_GRAVITY, proto::pack_f32s(q));
    const auto r = arm_->require().expect(proto::RSP_GRAVITY, timeout,
                                          "model.get_gravity", true,
                                          proto::CMD_GET_GRAVITY);
    const size_t want = 1 + 4 * size_t(MODEL_JM_N);
    if (r.payload.size() != want) {
        throw TransportError("RSP_GRAVITY 帧长 " + std::to_string(r.payload.size()) +
                             "B, 期望 " + std::to_string(want) + "B");
    }
    if (r.payload[0] != proto::RSP_GRAVITY) {
        throw TransportError("RSP_GRAVITY 帧 id 与期望不符");
    }
    return arm_->wrap(proto::unpack_f32s(r.payload.data(), 1, MODEL_JM_N),
                      proto::RSP_GRAVITY);
}

}  // namespace litearm
