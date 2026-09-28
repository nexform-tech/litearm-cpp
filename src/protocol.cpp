#include "litearm/protocol.hpp"

#include <cstring>
#include <map>
#include <regex>

namespace litearm {
namespace proto {

namespace {

const char* const kModeNames[] = {"INIT", "MOVE_J", "MOVE_P", "MOVE_JS",
                                  "MOVE_MIT", "MIT_ALL", "EMERGENCY", "ZERO_G"};
// flags bit0..bit5 才是安全 flag (6..8 是 mode, 9 是 enabled, 10 是 cart_busy)。
const char* const kFlagNames[] = {"FAULT", "WD_TRIPPED", "FB_STALE",
                                  "TEMP_WARN", "POS_VIOL", "OVERSPEED"};

// 命令覆盖契约 —— 与 Python 侧 _protocol.COMMAND_COVERAGE 逐条一致。
const std::vector<CommandCoverage> kCoverage = {
    {0x01, "Arm.movej"},
    {0x07, "Arm.movej_sync"},
    {0x02, "Arm.move_p"},
    {0x03, "Arm.move_js"},
    {0x04, "Arm.send_mit"},
    {0x05, "Arm.send_mit_all"},
    {0x06, "Arm.zero_g"},
    {0x10, "Arm.enable"},
    {0x11, "Arm.disable"},
    {0x12, "Arm.emergency_stop"},
    {0x13, "Arm.clear_faults"},
    {0x14, "Arm.reset"},
    {0x15, "Arm.enter_dfu"},
    {0x20, "Arm.set_motion_mode"},
    {0x21, "Arm.set_speed"},
    {0x22, "Arm.params.set_joint_param"},
    {0x23, "Arm.params.set_joint_limits"},
    {0x24, "Arm.params.get_joint_param"},
    {0x25, "Arm.save_params"},
    {0x26, "Arm.set_ff_vec"},
    {0x27, "Arm.set_ff_mask"},
    {0x28, "Arm.set_ff_scalar"},
    {0x2A, "Arm.home"},
    {0x2B, "Arm.get_ff_vec"},
    {0x2F, "Arm.license"},
    {0x3F, "Arm.activate"},
    {0x2C, "Arm.get_ff_scalar"},
    {0x2D, "Arm.log.start"},
    {0x2E, "Arm.log.reader"},
    {0x30, "Arm.model.set_body"},
    {0x31, "Arm.ff_preset"},
    {0x32, "Arm.model.commit"},
    {0x33, "Arm.model.set_jm"},
    {0x34, "Arm.model.get_body"},
    {0x35, "Arm.model.get_jm"},
    {0x37, "Arm.model.revert"},
    {0x38, "Arm.model.status"},
    {0x39, "Arm.model.get_gravity"},
    {0x36, "Arm.params.reset_factory"},
    {0x40, "Arm.get_status_now"},
    {0x41, "Arm.connect"},
    {0x42, "Arm.ik"},
    {0x43, "Arm.get_tcp"},
    {0x49, "Arm.diag.kin_bench"},
    // [笛卡尔] 0x3C/0x3D/0x3E 是同一条入口的三个阶段 (BEGIN -> ADD x n -> RUN),
    // 三者的帧都由 Arm::move_path 发出, 故三条都指向它。
    {0x3A, "Arm.move_l"},
    {0x3B, "Arm.move_c"},
    {0x3C, "Arm.move_path"},
    {0x3D, "Arm.move_path"},
    {0x3E, "Arm.move_path"},
};

const std::vector<std::pair<uint8_t, uint8_t>> kRspOfCmd = {
    {0x24, 0x49},  // GET_JOINT_PARAM  -> RSP_JOINT_PARAM
    {0x2B, 0x4B},  // GET_FF_VEC       -> RSP_FF_VEC
    {0x2C, 0x4C},  // GET_FF_SCALAR    -> RSP_FF_SCALAR
    {0x2E, 0x4D},  // LOG_READ         -> RSP_LOG_DATA
    {0x34, 0x54},  // GET_MODEL_PARAM  -> RSP_MODEL_PARAM
    {0x35, 0x55},  // GET_MODEL_JM     -> RSP_MODEL_JM
    {0x38, 0x56},  // GET_MODEL_STATUS -> RSP_MODEL_STATUS
    {0x39, 0x57},  // GET_GRAVITY      -> RSP_GRAVITY
    {0x41, 0x44},  // GET_FIRMWARE     -> RSP_FIRMWARE
    {0x42, 0x47},  // GET_IK           -> RSP_IK
    {0x43, 0x48},  // GET_TCP          -> RSP_TCP
    {0x49, 0x4A},  // KIN_BENCH        -> RSP_KIN_BENCH
};

float f32_at(const uint8_t* b, size_t off) {
    float v;
    std::memcpy(&v, b + off, 4);
    return v;
}

}  // namespace

const char* mode_name(int mode) {
    if (mode >= 0 && mode < 8) return kModeNames[mode];
    return "UNKNOWN";
}

const char* flag_name(int bit) {
    if (bit >= 0 && bit < 6) return kFlagNames[bit];
    return "UNKNOWN";
}

// 纯位循环实现 —— 与 Python 侧测试用的独立参考实现同式, 便于交叉验证。
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= uint16_t(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) {
                crc = uint16_t((crc << 1) ^ 0x1021);
            } else {
                crc = uint16_t(crc << 1);
            }
        }
    }
    return crc;
}

std::vector<uint8_t> pack_frame(uint8_t cmd, const uint8_t* payload, size_t len) {
    if (len > 255) {
        throw std::invalid_argument("payload >255B");
    }
    std::vector<uint8_t> body;
    body.reserve(3 + len);
    body.push_back(SOF);
    body.push_back(cmd);
    body.push_back(uint8_t(len));
    if (payload != nullptr && len > 0) {
        body.insert(body.end(), payload, payload + len);
    }
    uint16_t crc = crc16_ccitt_false(body.data(), body.size());
    body.push_back(uint8_t(crc & 0xFF));
    body.push_back(uint8_t(crc >> 8));
    return body;
}

std::optional<Frame> unpack_frame(const uint8_t* frame, size_t len) {
    if (len < 5 || frame[0] != SOF) return std::nullopt;
    const size_t ln = frame[2];
    if (len != ln + 5) return std::nullopt;
    const uint16_t crc = uint16_t(frame[3 + ln] | (frame[4 + ln] << 8));
    if (crc16_ccitt_false(frame, 3 + ln) != crc) return std::nullopt;
    Frame f;
    f.cmd = frame[1];
    f.payload.assign(frame + 3, frame + 3 + ln);
    return f;
}

std::vector<uint8_t> pack_f32s(const double* values, size_t count) {
    std::vector<uint8_t> out;
    out.reserve(count * 4);
    for (size_t i = 0; i < count; ++i) {
        const float f = float(values[i]);
        uint8_t b[4];
        std::memcpy(b, &f, 4);
        out.insert(out.end(), b, b + 4);
    }
    return out;
}

std::vector<double> unpack_f32s(const uint8_t* b, size_t off, int count) {
    std::vector<double> out;
    out.reserve(size_t(count));
    for (int i = 0; i < count; ++i) {
        out.push_back(double(f32_at(b, off + size_t(i) * 4)));
    }
    return out;
}

uint32_t read_u32le(const uint8_t* b) {
    return uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) |
           (uint32_t(b[3]) << 24);
}
uint16_t read_u16le(const uint8_t* b) { return uint16_t(b[0] | (b[1] << 8)); }
uint32_t read_u32le(const std::vector<uint8_t>& b, size_t off) {
    return read_u32le(b.data() + off);
}
uint16_t read_u16le(const std::vector<uint8_t>& b, size_t off) {
    return read_u16le(b.data() + off);
}

std::optional<FirmwareVersion> parse_firmware_version(const std::string& ver) {
    // 'Litearm1.5.2-7J' -> (1,5,2,'7J'); 旧 'A1.x-...-USB' 不符合新约定。
    size_t i = 0;
    while (i < ver.size() && std::isspace(static_cast<unsigned char>(ver[i]))) ++i;
    std::string s = ver.substr(i);
    const std::string prefix = "Litearm";
    if (s.rfind(prefix, 0) == 0) {
        s = s.substr(prefix.size());
    } else {
        return std::nullopt;   // 含旧 'A' 开头形态
    }
    std::string ver_part = s;
    std::string variant;
    const size_t dash = s.find('-');
    if (dash != std::string::npos) {
        ver_part = s.substr(0, dash);
        variant = s.substr(dash + 1);
    }
    // 三段必须全是数字
    std::vector<std::string> parts;
    size_t start = 0;
    while (true) {
        const size_t p = ver_part.find('.', start);
        if (p == std::string::npos) {
            parts.push_back(ver_part.substr(start));
            break;
        }
        parts.push_back(ver_part.substr(start, p - start));
        start = p + 1;
    }
    if (parts.size() != 3) return std::nullopt;
    for (const auto& p : parts) {
        if (p.empty()) return std::nullopt;
        for (char c : p) {
            if (!std::isdigit(static_cast<unsigned char>(c))) return std::nullopt;
        }
    }
    FirmwareVersion v;
    v.major = std::stoi(parts[0]);
    v.minor = std::stoi(parts[1]);
    v.patch = std::stoi(parts[2]);
    v.variant = variant;
    return v;
}

std::optional<std::string> parse_boot_banner(const std::string& text) {
    // 固件唯一能区分「独立看门狗复位过」的地方 (banner 带 ", iwdg-rst")。
    // 重连/多次枚举会重复出现, 取最后一次。
    static const std::regex re(R"(\[litearm-usbcdc\]\s+ready\s*\(([^)]*)\))");
    std::optional<std::string> last;
    for (auto it = std::sregex_iterator(text.begin(), text.end(), re);
         it != std::sregex_iterator(); ++it) {
        last = (*it)[1].str();
    }
    if (!last) return std::nullopt;
    return (last->find("iwdg-rst") != std::string::npos) ? std::string("iwdg-rst")
                                                        : std::string("normal");
}

std::optional<StatusDecoded> decode_status(const uint8_t* payload, size_t len) {
    if (len < 4) return std::nullopt;
    StatusDecoded out;
    out.flags = read_u16le(payload);
    out.seq = read_u16le(payload + 2);
    const size_t n = (len - 4) / 21;
    if (n != 1 && n != 7) return std::nullopt;
    const size_t tail = len - 4 - n * 21;
    if (tail != 0 && tail != 2) return std::nullopt;   // 0 = 旧布局, 2 = joint_fault u16
    out.joint_fault = (tail == 2) ? read_u16le(payload + 4 + n * 21) : 0;
    out.mode = (out.flags >> 6) & 0x7;
    for (int k = 0; k < 6; ++k) {
        if (out.flags & (1u << k)) out.flag_names.push_back(kFlagNames[k]);
    }
    out.joints.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t off = 4 + i * 21;
        JointRaw j;
        j.q = double(f32_at(payload, off));
        j.dq = double(f32_at(payload, off + 4));
        j.tau = double(f32_at(payload, off + 8));
        j.t_mos = double(f32_at(payload, off + 12));
        j.t_coil = double(f32_at(payload, off + 16));
        j.err = payload[off + 20];
        out.joints.push_back(j);
    }
    return out;
}

const std::map<uint8_t, std::string>& unimplemented_cmds() {
    static const std::map<uint8_t, std::string> kEmpty;
    return kEmpty;
}
const std::map<uint8_t, std::string>& firmware_only_cmds() {
    static const std::map<uint8_t, std::string> kEmpty;
    return kEmpty;
}
const std::map<uint8_t, std::string>& firmware_only_rsps() {
    static const std::map<uint8_t, std::string> kEmpty;
    return kEmpty;
}
const std::map<uint8_t, std::string>& preexisting_gaps() {
    static const std::map<uint8_t, std::string> kEmpty;
    return kEmpty;
}

const std::vector<CommandCoverage>& command_coverage() { return kCoverage; }
const std::vector<std::pair<uint8_t, uint8_t>>& rsp_of_cmd() { return kRspOfCmd; }

std::optional<uint8_t> rsp_of(uint8_t cmd) {
    for (const auto& kv : kRspOfCmd) {
        if (kv.first == cmd) return kv.second;
    }
    return std::nullopt;
}

}  // namespace proto
}  // namespace litearm
