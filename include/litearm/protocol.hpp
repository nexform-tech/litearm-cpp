// 帧协议层 —— 与 litearm-stm32 固件 / tools/arm_console.py 逐字节一致。
//
// 帧: SOF(0xA5) CMD(1B) LEN(1B) PAYLOAD(0..255) CRC16_LO CRC16_HI
// CRC16-CCITT-FALSE, 覆盖 [SOF..PAYLOAD]。
// 上行 100Hz 状态帧: 6+21N (固件 >=1.5.0, 尾部 joint_fault u16) 或 4+21N
// (<=1.4.x, 仅兼容解析); 应答/异步返回穿插。
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace litearm {
namespace proto {

// ---------------------------------------------------------------- 帧
inline constexpr uint8_t SOF = 0xA5;

// ---- 下行 CMD (固件 hal/usb_cmd.h) ----
inline constexpr uint8_t CMD_MOVE_J = 0x01;
inline constexpr uint8_t CMD_MOVE_P = 0x02;
inline constexpr uint8_t CMD_MOVE_JS = 0x03;
inline constexpr uint8_t CMD_MOVE_MIT = 0x04;
inline constexpr uint8_t CMD_MOVE_MIT_ALL = 0x05;
// 零重力(拖动示教) 进入/退出: payload [u8] 1=进 0=退 -> ACK
// 固件侧自带 watchdog_kick, 重发即保活 —— 上位机需周期重发(建议 <=50ms),
// 否则 0.1s 后看门狗把模式掐回 fail-soft 持位。
inline constexpr uint8_t CMD_ZERO_G = 0x06;
// [SYNC] 同步 PTP movej: 全关节同时到达, 末端沿关节空间直线 q(s) = q0 + s*dq。
// 载荷与 CMD_MOVE_J 逐字节相同; 语义差异只在轨迹形状。
inline constexpr uint8_t CMD_MOVE_J_SYNC = 0x07;
inline constexpr uint8_t CMD_ENABLE = 0x10;
inline constexpr uint8_t CMD_DISABLE = 0x11;
inline constexpr uint8_t CMD_EMERGENCY_STOP = 0x12;
inline constexpr uint8_t CMD_CLEAR_FAULTS = 0x13;
inline constexpr uint8_t CMD_RESET = 0x14;
// [DFU] 免探针进 ROM 系统 bootloader (终端态操作): 空载荷 -> ACK 表示"已登记"。
inline constexpr uint8_t CMD_ENTER_DFU = 0x15;
inline constexpr uint8_t CMD_SET_MOTION_MODE = 0x20;
inline constexpr uint8_t CMD_SET_SPEED_PERCENT = 0x21;
inline constexpr uint8_t CMD_SET_JOINT_PARAM = 0x22;
inline constexpr uint8_t CMD_SET_JOINT_LIMITS = 0x23;
inline constexpr uint8_t CMD_GET_JOINT_PARAM = 0x24;
inline constexpr uint8_t CMD_PARAM_SAVE = 0x25;
// [P1-C] 恢复出厂默认 + 失效 flash (空载荷; 须失能, 否则 ERR{0x36,0x04})
inline constexpr uint8_t CMD_PARAM_RESET = 0x36;
// [HOME] 回零位舒展姿 (空载荷): 目标=URDF 零位 0, 固件侧速度写死 0.10。
inline constexpr uint8_t CMD_HOME = 0x2A;
// item(1B)+f32[7]: 1 friction/2 ki/3 i_max/4-6 wall/7 gs/8 is
//   9-11 摩擦 v2 (fv/fc0/fc1) / 12-14 零重力 (zg_kp/zg_kd/zg_damping) / 15 kd_extra
// 12 zg_kp 松手保持的虚拟弹簧刚度 Nm/rad 钳 [0,500]
// 13 zg_kd MIT 速度阻尼 (固件会抬到下限以上) 钳 [0,5]
// 14 zg_damping 附加速度阻尼 Nm.s/rad, 出厂 0 钳 [0,50]
// 15 kd_extra tau 域软件微分阻尼, 出厂 [6,6,6,6,0,0,0], 腕部 J5-J7 必须留 0。
// 任一分量 NaN 整组拒绝 (ERR{0x26,0x02}), 幅值超限静默钳制。
inline constexpr uint8_t CMD_SET_FF_VEC = 0x26;
inline constexpr uint8_t CMD_SET_FF_FLAGS = 0x27;      // u32 ff_mask LE
// item+sub+f32: 1 fric_db/2 margin/3 slew/4 payload_mass/5 payload_com/6 gravity
//   7 friction_model (0/1/2) / 8 fric_v2_eps
//   10-17 零重力 (drag_gain/drag_db/drag_kd_margin/vel_thr/
//            engage_sec/engage_kp/engage_kd/wall_fw_kd)
//   18 hold_kp_gain. item 9 保留给 0x2C 的 ff_mask 读回
inline constexpr uint8_t CMD_SET_FF_SCALAR = 0x28;
inline constexpr uint8_t CMD_FF_PRESET = 0x31;         // u8: 0 全关 / 1 出厂 / 2 全开
// [readback 0x2B/0x2C] 参数读回 (镜像写口, 纯读)
inline constexpr uint8_t CMD_GET_FF_VEC = 0x2B;        // item(1B) -> RSP_FF_VEC
inline constexpr uint8_t CMD_GET_FF_SCALAR = 0x2C;     // item+sub -> RSP_FF_SCALAR
// [P1 傅里叶辨识采集] 300Hz 控制拍记录/读回 (固件 hal/log_capture.*)
inline constexpr uint8_t CMD_LOG_CTRL = 0x2D;          // u32 n_ticks LE (0=停/清)
inline constexpr uint8_t CMD_SET_MODEL_PARAM = 0x30;   // body_idx(1B)+f32[10] -> staging
inline constexpr uint8_t CMD_MODEL_COMMIT = 0x32;      // u16 expected_mask LE
inline constexpr uint8_t CMD_SET_MODEL_JM = 0x33;      // f32[7] -> staging 的 jm
inline constexpr uint8_t CMD_GET_MODEL_PARAM = 0x34;   // body_idx(1B) -> RSP_MODEL_PARAM
inline constexpr uint8_t CMD_GET_MODEL_JM = 0x35;      // (空) -> RSP_MODEL_JM
inline constexpr uint8_t CMD_REVERT_MODEL = 0x37;      // (空) -> ACK
inline constexpr uint8_t CMD_GET_MODEL_STATUS = 0x38;  // (空) -> RSP_MODEL_STATUS
inline constexpr uint8_t CMD_GET_GRAVITY = 0x39;       // q[7]f32 -> RSP_GRAVITY
inline constexpr uint8_t CMD_LOG_READ = 0x2E;          // u32 offset_byte LE -> RSP_LOG_DATA
// 双 ID: 下行 CMD_KIN_BENCH 与上行 RSP_JOINT_PARAM 同为 0x49 (固件 usb_cmd.h)。
// 收发方向不同, 故靠方向区分, 不可用单张 ID 表查。
inline constexpr uint8_t CMD_KIN_BENCH = 0x49;
inline constexpr uint8_t CMD_GET_STATUS = 0x40;
inline constexpr uint8_t CMD_GET_FIRMWARE = 0x41;
inline constexpr uint8_t CMD_GET_IK = 0x42;
inline constexpr uint8_t CMD_GET_TCP = 0x43;

// ---- [授权/激活] 固件 license.c + usb_cmd.c 的 license 分支 (固件 1.8.0+) ----
// 本层只做"查询 + 提交凭据", 不含任何算 MAC 的代码 —— 这是规格的硬要求。
inline constexpr uint8_t CMD_GET_LICENSE = 0x2F;       // 无载荷 -> RSP_LICENSE(0x4F) 26B
inline constexpr uint8_t CMD_ACTIVATE = 0x3F;          // 28B -> ACK{0x3F} / ERR{0x3F,code}
// 28B = cust_id u32 LE + issued u32 LE + flags u32 LE + mac[16]
// 长度判据是 len < 28 => 更长的载荷会被接受, 尾部多余字节不进 MAC。

// ---- [笛卡尔运控] 固件原生规划 (受固件 #if LITEARM_CART_PLAN 编译开关约束) ----
// 关掉开关时这 5 条整段不在固件里, 会落到 usb_cmd.c 的 default 分支 ->
// ERR{cmd,0x00}。故 SDK 必须探测而不是假定 (见 cart::probe)。
// 0x3A/0x3B/0x3C/0x3D 的长度校验排在门禁与一切副作用之前, 所以"发空载荷"不会让臂动
// —— 这四条可以当能力探针。0x3E (CART_RUN) 是例外: 空载荷就是它的合法载荷, 且
// cart_gate_ok 的门禁有实打实的副作用, 别拿它当探针。
inline constexpr uint8_t CMD_MOVE_L = 0x3A;            // pose[6]f32 + sp f32 (28B)
inline constexpr uint8_t CMD_MOVE_C = 0x3B;            // via[6] + end[6] + sp (52B)
inline constexpr uint8_t CMD_CART_BEGIN = 0x3C;        // n u8 + sp f32 (5B, n in [1,32])
inline constexpr uint8_t CMD_CART_ADD = 0x3D;          // idx u8 + pose[6] (25B)
inline constexpr uint8_t CMD_CART_RUN = 0x3E;          // 空载荷: 提交并开始规划

// ---- 上行 ----
inline constexpr uint8_t RSP_STATUS = 0x40;
// 死登记 (固件仍定义 RSP_DETAIL 0x41 但发送方已删; 且与下行 CMD_GET_FIRMWARE 撞号)。
// 保留定义是为了让协议同步测试的按名比对自然通过 —— 不要给它造生产者。
inline constexpr uint8_t RSP_DETAIL = 0x41;
inline constexpr uint8_t RSP_FIRMWARE = 0x44;
inline constexpr uint8_t RSP_ACK = 0x45;
inline constexpr uint8_t RSP_ERR = 0x46;
inline constexpr uint8_t RSP_IK = 0x47;
inline constexpr uint8_t RSP_TCP = 0x48;
inline constexpr uint8_t RSP_JOINT_PARAM = 0x49;
inline constexpr uint8_t RSP_KIN_BENCH = 0x4A;         // CMD_KIN_BENCH 应答: 自检文本
inline constexpr uint8_t RSP_FF_VEC = 0x4B;            // 载荷首字节冗余回填本 id
inline constexpr uint8_t RSP_FF_SCALAR = 0x4C;         // 同上
inline constexpr uint8_t RSP_LOG_DATA = 0x4D;          // [u32 total][u32 next][u8 n][n bytes]
// [笛卡尔] 规划结果: ok u8 + err u8 + n_wp u16 LE + plan_us u32 LE = 8B。
// 载荷里没有命令 id —— 只能按受理顺序 FIFO 配对。规划失败也必须发这一条。
inline constexpr uint8_t RSP_CART_PLAN = 0x4E;
// [授权] CMD_GET_LICENSE(0x2F) 的应答 —— 26B:
//   [state u8][ver u8][uid 12B][cust_id u32 LE][issued u32 LE][flags u32 LE]
// 载荷首字节是 state(0/1/2), 不是命令号 => 它不在"回显归属"那一类。
// 未激活时 cust_id/issued/flags 全 0, 但 UID 照回。
inline constexpr uint8_t RSP_LICENSE = 0x4F;
// [授权] 本版无生产者 —— 激活成功回的是 ACK{0x3F}, 故固件里没有任何代码会发出 0x50。
inline constexpr uint8_t RSP_ACTIVATE = 0x50;
inline constexpr uint8_t RSP_MODEL_PARAM = 0x54;       // [0x54, body_idx, f32[10]] = 42B
inline constexpr uint8_t RSP_MODEL_JM = 0x55;          // [0x55, f32[7]] = 29B
inline constexpr uint8_t RSP_MODEL_STATUS = 0x56;      // [0x56, override, mask u16 LE, dirty]
inline constexpr uint8_t RSP_GRAVITY = 0x57;           // [0x57, G(q)[7]] = 29B

// ---- ff_mask 位 (固件 litearm.h) ----
inline constexpr uint32_t FF_MASTER = 0x0001;
inline constexpr uint32_t FF_G = 0x0002;
inline constexpr uint32_t FF_INERTIA = 0x0004;
inline constexpr uint32_t FF_CORIOLIS = 0x0008;
inline constexpr uint32_t FF_FRICTION = 0x0010;
inline constexpr uint32_t FF_INTEGRAL = 0x0020;
inline constexpr uint32_t FF_WALL = 0x0040;
inline constexpr uint32_t FF_QUANT = 0x0080;
inline constexpr uint32_t FF_VELREF = 0x0100;
inline constexpr uint32_t FF_ALL = 0x01FF;

// 固件 IK 的 seed 恒为模型 7 轴 (KIN_N), 与 LITEARM_NUM_JOINTS 解耦。
inline constexpr int KIN_N = 7;
// 台架电机对应的整臂模型轴 (joint_cfg.h 的 LITEARM_BENCH_MODEL_AXIS, 台架=5)。
inline constexpr int BENCH_MODEL_AXIS = 5;

inline constexpr int MODE_ZERO_G = 7;
// flags bit9 = 使能位 (固件 1.5.0 起; 低 6 位才是安全 flag, 6..8 是 mode)
inline constexpr int FLAG_ENABLED_BIT = 9;
// flags bit10 = 笛卡尔规划/播放进行中 (刻意不放进 FLAG_NAMES, 见 state.hpp)
inline constexpr int FLAG_CART_BUSY_BIT = 10;
// joint_fault 位图 (u16) 能表达的关节数上限
inline constexpr int MAX_JOINTS = 16;

const char* mode_name(int mode);
const char* flag_name(int bit);

// ---------------------------------------------------------------- 帧编解码

/// **CRC-16/CCITT-FALSE** (poly 0x1021 / init 0xFFFF / 无末异或)。
/// 覆盖 [SOF..PAYLOAD], 两字节低字节在前 (见 pack_frame)。
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);
inline uint16_t crc16_ccitt_false(const std::vector<uint8_t>& v) {
    return crc16_ccitt_false(v.data(), v.size());
}

/// 组一帧: SOF CMD LEN PAYLOAD CRC_LO CRC_HI。
std::vector<uint8_t> pack_frame(uint8_t cmd, const uint8_t* payload, size_t len);
inline std::vector<uint8_t> pack_frame(uint8_t cmd) { return pack_frame(cmd, nullptr, 0); }
inline std::vector<uint8_t> pack_frame(uint8_t cmd, const std::vector<uint8_t>& p) {
    return pack_frame(cmd, p.data(), p.size());
}

/// 一条解出来的帧。
struct Frame {
    uint8_t cmd = 0;
    std::vector<uint8_t> payload;
};

/// 校验完整帧; 合法返回帧, 否则 nullopt。
std::optional<Frame> unpack_frame(const uint8_t* frame, size_t len);
inline std::optional<Frame> unpack_frame(const std::vector<uint8_t>& f) {
    return unpack_frame(f.data(), f.size());
}

// ---------------------------------------------------------------- f32 载荷

std::vector<uint8_t> pack_f32s(const double* values, size_t count);
inline std::vector<uint8_t> pack_f32s(const std::vector<double>& v) {
    return pack_f32s(v.data(), v.size());
}
inline std::vector<uint8_t> pack_f32s(const std::array<double, 6>& v) {
    return pack_f32s(v.data(), v.size());
}
inline std::vector<uint8_t> pack_f32s(const std::array<double, 7>& v) {
    return pack_f32s(v.data(), v.size());
}

std::vector<double> unpack_f32s(const uint8_t* b, size_t off, int count);

inline std::vector<uint8_t> pack_u32le(uint32_t v) {
    return {uint8_t(v & 0xFF), uint8_t((v >> 8) & 0xFF),
            uint8_t((v >> 16) & 0xFF), uint8_t((v >> 24) & 0xFF)};
}
inline std::vector<uint8_t> pack_u16le(uint16_t v) {
    return {uint8_t(v & 0xFF), uint8_t((v >> 8) & 0xFF)};
}
inline std::vector<uint8_t> pack_f32le(double v) {
    return pack_f32s(&v, 1);
}

uint32_t read_u32le(const uint8_t* b);
uint16_t read_u16le(const uint8_t* b);
uint32_t read_u32le(const std::vector<uint8_t>& b, size_t off);
uint16_t read_u16le(const std::vector<uint8_t>& b, size_t off);

// ---------------------------------------------------------------- 固件版本

struct FirmwareVersion {
    int major = 0;
    int minor = 0;
    int patch = 0;
    std::string variant;   // "7J" / "1J" / ...
    bool operator<(const FirmwareVersion& o) const {
        if (major != o.major) return major < o.major;
        if (minor != o.minor) return minor < o.minor;
        return patch < o.patch;
    }
    bool operator<=(const FirmwareVersion& o) const { return !(o < *this); }
    bool operator==(const FirmwareVersion& o) const {
        return major == o.major && minor == o.minor && patch == o.patch;
    }
};

/// 'Litearm1.5.2-7J' -> {1,5,2,"7J"}; 旧 'A1.x-...-USB' 返回 nullopt(不符合新约定)。
std::optional<FirmwareVersion> parse_firmware_version(const std::string& ver);

/// 从串口噪声文本里解析开机签名 -> "normal" | "iwdg-rst"; 没有则 nullopt。
/// 固件唯一能区分「独立看门狗复位过」的地方。重连/多次枚举会重复出现, 取最后一次。
std::optional<std::string> parse_boot_banner(const std::string& text);

// ---------------------------------------------------------------- 状态帧

/// 一个关节的一拍。err 是 payload 尾部那个字节。
struct JointRaw {
    double q = 0.0, dq = 0.0, tau = 0.0, t_mos = 0.0, t_coil = 0.0;
    uint8_t err = 0;
};

/// 状态帧解析结果。
struct StatusDecoded {
    uint16_t flags = 0;
    uint16_t seq = 0;
    int mode = 0;
    std::vector<const char*> flag_names;
    std::vector<JointRaw> joints;
    uint16_t joint_fault = 0;
};

/// 状态帧 -> StatusDecoded; 非法返回 nullopt。
/// 兼容两种布局 (尾部 u16 joint_fault 为固件 1.5.0 起新增):
///   4+21N  固件 <=1.4.x (无 joint_fault, 视为 0)
///   6+21N  固件 >=1.5.0 (7J=153B / 1J=27B)
std::optional<StatusDecoded> decode_status(const uint8_t* payload, size_t len);
inline std::optional<StatusDecoded> decode_status(const std::vector<uint8_t>& p) {
    return decode_status(p.data(), p.size());
}

// ---------------------------------------------------------------- 命令覆盖契约
//
// 固件每条已实现的下行命令 -> SDK 上的一等方法入口。由测试强制 (见
// tests/test_protocol_sync.cpp 对 _protocol.py 的 COMMAND_COVERAGE 逐条断言)。
struct CommandCoverage {
    uint8_t cmd;
    const char* entry;
};
const std::vector<CommandCoverage>& command_coverage();

/// 请求 -> 它的那条 RSP_* 应答 (只列"应答不是 ACK/ERR"的命令)。
/// 唯一用途是 Arm::raw_write 在发出前清掉本命令的应答队列。
/// 0x40 (GET_STATUS) 不在: 它的应答走单槽不是队列。
const std::vector<std::pair<uint8_t, uint8_t>>& rsp_of_cmd();
std::optional<uint8_t> rsp_of(uint8_t cmd);

/// 回显命令码当归属判据的上行 id —— 只有这两类帧的 payload[0] 是"原命令码"。
inline bool is_echoed(uint8_t c) { return c == RSP_ACK || c == RSP_ERR; }

// ---------------------------------------------------------------- 同步哨兵表
//
// 上游 (`_protocol.py`) 用四张表把"SDK 与固件的覆盖关系"变成可断言的东西 ——
// `tests/test_protocol_sync.py` 直接解析**固件头文件** (hal/usb_cmd.h), 双向比对名字与
// ID, 并逐条核对这四张表**仍然成立**。它们的语义各不相同, 别混:
//
//   * `unimplemented_cmds()`  —— 固件注释即「未实现」, SDK 有意不提供入口。
//     不是永久豁免: 一旦固件实现了它们, 那条测试立刻红, 提醒补入口。
//   * `firmware_only_cmds()` / `firmware_only_rsps()` —— 用户裁决 SDK **有意不暴露**
//     ("不做", 不是"还没做")。往这里加条目等于宣布"SDK 有意不做这条命令"。
//   * `preexisting_gaps()` —— **已知缺口** (本应同步而未同步), 与"有意不做"不同。
//
// 四张表**当前都是空的**。留空是**有意的**: 它们现在是"当前没有未实现命令 / 没有固件单边
// 命令 / 没有已知缺口"这句话的**断言载体**, 而不是可以顺手塞东西的地方。
//
// ⚠ **本仓跑不了那个比对**: `test_protocol_sync.py` 要读固件仓 (`LITEARM_FW_DIR`) 的
//   `hal/usb_cmd.h`, 而 C++ 仓里没有固件树。故这里保留的是**表本身与它们的语义**
//   (由 `tests/test_protocol.cpp` 断言为空), 固件侧那一半需要把固件仓放到旁边才能跑。
//   别把"表是空的"读成"已经跟固件对过账"。
const std::map<uint8_t, std::string>& unimplemented_cmds();
const std::map<uint8_t, std::string>& firmware_only_cmds();
const std::map<uint8_t, std::string>& firmware_only_rsps();
const std::map<uint8_t, std::string>& preexisting_gaps();

}  // namespace proto
}  // namespace litearm
