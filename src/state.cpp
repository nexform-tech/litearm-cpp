#include "litearm/state.hpp"

#include <sstream>

#include "litearm/errors.hpp"

namespace litearm {

std::vector<int> RobotState::fault_axes() const {
    std::vector<int> out;
    for (int i = 0; i < proto::MAX_JOINTS; ++i) {
        if (joint_fault & (1u << i)) out.push_back(i);
    }
    return out;
}

std::vector<double> RobotState::q() const {
    std::vector<double> out;
    out.reserve(joints.size());
    for (const auto& j : joints) out.push_back(j.q);
    return out;
}

std::vector<double> RobotState::dq() const {
    std::vector<double> out;
    out.reserve(joints.size());
    for (const auto& j : joints) out.push_back(j.dq);
    return out;
}

std::vector<double> RobotState::tau() const {
    std::vector<double> out;
    out.reserve(joints.size());
    for (const auto& j : joints) out.push_back(j.tau);
    return out;
}

std::string RobotState::fault_detail() const {
    std::vector<std::string> parts;
    if (!flag_names.empty()) {
        std::string s = "flags=";
        for (size_t i = 0; i < flag_names.size(); ++i) {
            if (i) s += ",";
            s += flag_names[i];
        }
        parts.push_back(s);
    }
    if (joint_fault) {
        std::string s = "断轴=";
        bool first = true;
        for (int a : fault_axes()) {
            if (!first) s += ",";
            first = false;
            s += "J" + std::to_string(a + 1);
        }
        parts.push_back(s);
    }
    if (parts.empty()) return "无故障位";
    std::string out;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i) out += " ";
        out += parts[i];
    }
    return out;
}

RobotState decode_state(const std::vector<uint8_t>& payload) {
    const auto dec = proto::decode_status(payload);
    if (!dec) throw TransportError("非法状态帧");
    RobotState st;
    st.mode = dec->mode;
    st.mode_name = proto::mode_name(dec->mode);
    st.flags = dec->flags;
    for (const char* nm : dec->flag_names) st.flag_names.emplace_back(nm);
    st.seq = dec->seq;
    st.joint_fault = dec->joint_fault;
    st.joints.reserve(dec->joints.size());
    for (const auto& j : dec->joints) {
        JointState js;
        js.q = j.q;
        js.dq = j.dq;
        js.tau = j.tau;
        js.t_mos = j.t_mos;
        js.t_coil = j.t_coil;
        js.err = j.err;
        st.joints.push_back(js);
    }
    return st;
}

}  // namespace litearm
