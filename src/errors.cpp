#include "litearm/errors.hpp"

#include <cstdio>

namespace litearm {

namespace {

// 具体档: (cmd, code) -> 语义。逐条取自固件原注释, 不自行起名。
// code == 0x00 一律不在此表 (见头文件说明)。
const std::map<std::pair<int, int>, std::string>& build_err_text() {
    static const std::map<std::pair<int, int>, std::string> kTable = {
        // ---- 0x01 MOVE_J (每轴一条独立 S 曲线; 0x07 是同步版, 同码同语义) ----
        {{0x01, 0x01}, "载荷长度不足 (需 4xN+4 字节)"},
        {{0x01, 0x02}, "含非有限值, 或 sp 越界 —— sp 是 0..1 的小数倍率 (传 30 想表达 "
                       "30% 属单位陷阱, 固件显式拒而不静默 clamp 成 100%)"},
        {{0x01, 0x03}, "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_j)"},
        {{0x01, 0x04}, "零重力(拖动示教)进行中 —— 须显式 zero_g off 退出, 固件不隐式退出"},
        {{0x01, 0x06}, "掉线刚性持位锁存 (drop_hold) 中 —— 须先 reset/clear_faults"},

        // ---- 0x07 MOVE_J_SYNC (载荷与 0x01 逐字节相同, 只换轨迹形状) ----
        {{0x07, 0x01}, "载荷长度不足 (需 4xN+4 字节)"},
        {{0x07, 0x02}, "含非有限值, 或 sp 越界 (同 0x01 的 0..1 小数倍率约定)"},
        {{0x07, 0x03}, "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_j_sync)"},
        {{0x07, 0x04}, "零重力(拖动示教)进行中"},
        {{0x07, 0x06}, "掉线刚性持位锁存 (drop_hold) 中 —— 须先 reset/clear_faults"},

        // ---- 0x02 MOVE_P (固件后台 IK + 走 S 曲线) ----
        {{0x02, 0x01}, "载荷长度不足 (需 28 字节: pose[6] + sp)"},
        {{0x02, 0x02}, "位姿含非有限值, 或 sp 越界 (0..1 小数倍率)"},
        {{0x02, 0x03}, "**二义码, 两处发射点语义不同**: 1) dispatch 的「未使能 / "
                       "EMERGENCY 锁存」 2) 后台 IK 失败 —— 原注释「不可达 / 解非法」。"
                       "上位机无法从本码区分两者, 只能按「这条 move_p 没成立」处置 "
                       "(不要据此判失能)"},
        {{0x02, 0x04}, "零重力(拖动示教)进行中"},
        {{0x02, 0x05}, "IK 后台通道忙 (已有一次后台求解在途)"},

        // ---- 0x3A / 0x3B MOVE_L / MOVE_C (固件原生笛卡尔) ----
        // 门禁三档由 cart_gate_ok() 统一发: !enabled(0x03) -> drop_hold(0x06) ->
        // 零重力(0x04); 收口码由 cart_reply() 透传 cart_req_* 的 rc。
        {{0x3A, 0x01}, "载荷长度不足 (需 28 字节)"},
        {{0x3A, 0x02}, "位姿含非有限值, 或 sp 越界 (0..1 小数倍率)"},
        {{0x3A, 0x03}, "未使能 (笛卡尔门禁 cart_gate_ok)"},
        {{0x3A, 0x04}, "零重力中 (门禁) 或 状态机不允许 —— RECV 会话未收尾时不许插队"},
        {{0x3A, 0x06}, "掉线刚性持位锁存 (drop_hold) 中 —— 须先 reset/clear_faults"},
        {{0x3B, 0x01}, "载荷长度不足 (需 52 字节: 途经点 + 终点 + sp)"},
        {{0x3B, 0x02}, "位姿含非有限值, 或 sp 越界 (0..1 小数倍率)"},
        {{0x3B, 0x03}, "未使能 (笛卡尔门禁)"},
        {{0x3B, 0x04}, "零重力中 (门禁) 或 状态机不允许 —— RECV 会话未收尾时不许插队"},
        {{0x3B, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},

        // ---- 0x3C / 0x3D / 0x3E CART_BEGIN / ADD / RUN (多路点三段式) ----
        // BEGIN/ADD 刻意不走 cart_gate_ok (RECV 态不占用臂), 但三档判据与档序逐档一致。
        {{0x3C, 0x01}, "载荷长度不足 (需 5 字节: 点数 n + sp)"},
        {{0x3C, 0x02}, "sp 非有限/越界, 或点数 n 越界 (须 1..CART_MAX_GOAL=32)"},
        {{0x3C, 0x03}, "未使能"},
        {{0x3C, 0x04}, "零重力中 或 状态机不允许 —— 只有 IDLE 能开新会话 (含 RECV 重入)"},
        {{0x3C, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},
        {{0x3D, 0x01}, "载荷长度不足 (需 25 字节: idx + pose[6])"},
        {{0x3D, 0x02}, "位姿含非有限值 (dispatch), 或 idx 越界 (cart_req_add)"},
        {{0x3D, 0x03}, "未使能"},
        {{0x3D, 0x04}, "零重力中, 或 非 RECV 态, 或该 idx 已 ADD 过 (去重)"},
        {{0x3D, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},
        {{0x3E, 0x03}, "未使能 (笛卡尔门禁)"},
        {{0x3E, 0x04}, "零重力中 (门禁) 或 状态机不允许 —— 非 RECV 态, 或点名不满"},
        {{0x3E, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},

        // ---- 0x03 MOVE_JS (流式关节透传) ----
        {{0x03, 0x01}, "载荷长度不足 (需 8xN 字节; 带 tau_ff 时 12xN)"},
        {{0x03, 0x02}, "q/dq(/tau_ff) 含非有限值"},
        {{0x03, 0x03}, "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_js)"},
        {{0x03, 0x04}, "零重力(拖动示教)进行中"},
        {{0x03, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},

        // ---- 0x04 MOVE_MIT (单关节透传) ----
        {{0x04, 0x01}, "载荷长度不足 (需 21 字节)"},
        {{0x04, 0x02}, "idx 越界, 或 q/dq/kp/kd/tau 含非有限值"},
        {{0x04, 0x03}, "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_mit)"},
        {{0x04, 0x04}, "零重力(拖动示教)进行中"},
        {{0x04, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},

        // ---- 0x05 MOVE_MIT_ALL (全臂单帧透传) ----
        {{0x05, 0x01}, "载荷长度不足 (需 20xN 字节)"},
        {{0x05, 0x02}, "q/dq/kp/kd/tau 含非有限值"},
        {{0x05, 0x03}, "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_mit_all)"},
        {{0x05, 0x04}, "零重力(拖动示教)进行中"},
        {{0x05, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},

        // ---- 0x06 ZERO_G (进入/退出拖动示教; 重发即保活) ----
        {{0x06, 0x01}, "载荷长度不足 (需 1 字节: on)"},
        {{0x06, 0x02}, "拒绝: EMERGENCY 锁存 (进入与退出都拒), 或进入时未使能 "
                       "(退出 on=0 幂等, 未使能也收)"},

        // ---- 0x2A HOME (回零位舒展姿; 内部复用 ctrl_accept_move_j) ----
        {{0x2A, 0x03}, "未使能 (dispatch 前置) 或 EMERGENCY 锁存 (ctrl_accept_move_j)"},
        {{0x2A, 0x04}, "零重力(拖动示教)进行中"},
        {{0x2A, 0x06}, "掉线刚性持位锁存 (drop_hold) 中"},

        // ---- 0x10 ENABLE (使能全关节; 码位决定是否值得重试) ----
        {{0x10, 0x08}, "**未激活(未授权)—— 重发无用, 无旁路**。固件 ctrl_enable() 的"
                       "第一条判据就是 !license_is_activated(), 故本码说的是"
                       "「这台臂有没有被授权」, 而不是「现在能不能使能」—— 后者优先于"
                       "其余所有码。补救: 用 Arm.license() 查 state, 再拿厂商签发的凭据"
                       "走 Arm.activate()"},
        {{0x10, 0x03}, "可重试 —— 电机反馈未齐 / CMODE 首写 (含 CMODE 回读不良已补写, "
                       "重发即可)"},
        {{0x10, 0x06}, "**锁存, 须先 RESET** —— EMERGENCY 或 joint_fault 非 0 "
                       "(重发无用)。码值虽与运动命令的 0x06 相同, 但判据不同 —— "
                       "本档只看 EMERGENCY / joint_fault, 不判掉线刚性持位锁存"},
        {{0x10, 0x07}, "部分轴 CMODE 补写预算耗尽仍未进 MIT —— **重发无用**, 须现场排查"},

        // ---- 0x15 ENTER_DFU (SDK 里唯一的终端态操作: 进 ROM bootloader) ----
        {{0x15, 0x01}, "载荷必须为空 (本命令不吃参数)"},
        {{0x15, 0x02}, "ROM 系统 bootloader 向量表无效 —— 跳过去也起不来, 故当场拒"},
        {{0x15, 0x03}, "使能中, 或使能在途 (enable_pending) —— 跳转会停 TIM3, "
                       "电机 100ms 后松开, 有重力负载则下垂"},

        // ---- 0x3F ACTIVATE (提交授权凭据; 固件 1.8.0+) ----
        // 线上码除长度沿用 0x01 外, 其余一律折成 0x02, 故 0x02 不等于"你的码不对"。
        {{0x3F, 0x01}, "载荷长度不足 (需 >=28 字节) —— 注意判据是 < 28, 更长的载荷会被"
                       "接受并静默忽略尾部"},
        {{0x3F, 0x02}, "**聚合档, 必须回查 0x2F 才能定性** —— flags 保留位非 0 / 已经"
                       "激活过 / MAC 不符 / 固件编译进来的密钥非法 / 写或读回失败, 全折叠"
                       "成本码。其中「已经激活过」意味着设备可能其实已经解锁 (例如上一条 "
                       "ACK 被丢掉后重发) —— 把它当失败会让产线重复返工, 故不要据本码判"
                       "机器没激活"},
        {{0x3F, 0x04}, "已武装 (使能中或使能在途) 须先失能 —— 与 0x25 同语义: 写 flash "
                       "期间电机不得在无监督下保持使能"},
        {{0x3F, 0x05}, "flash 忙 (main 正在整扇区保存参数) —— 稍后重试即可"},

        // ---- 0x20 / 0x21 运行期旋钮 (都只判长度) ----
        {{0x20, 0x01}, "载荷长度不足 (需 1 字节: mode)"},
        {{0x21, 0x01}, "载荷长度不足 (需 1 字节: 百分比)"},

        // ---- 0x42 GET_IK / 0x49 KIN_BENCH (后台求解通道) ----
        {{0x42, 0x01}, "载荷长度不足 (需 52 字节: pose[6] + seed[7])"},
        {{0x42, 0x02}, "pose 或 seed 含非有限值"},
        {{0x42, 0x05}, "忙 —— 已有一次后台 IK 在途 (成功登记时不即时应答, 等 RSP_IK)"},
        {{0x49, 0x05}, "忙 —— 已有一次 kin_bench 在途"},

        // ---- 0x22 / 0x23 / 0x24 关节参数 ----
        {{0x22, 0x01}, "载荷长度不足 (需 13 字节: idx + kp + kd + tau_max)"},
        {{0x22, 0x02}, "idx 越界, 或 kp/kd/tau_max 非法 (NaN 或 kp<0 / kd<0 / tau_max<=0)"},
        {{0x23, 0x01}, "载荷长度不足 (需 9 字节: idx + q_min + q_max)"},
        {{0x23, 0x02}, "数值非法 (NaN / q_min>=q_max), 或 idx 越界, 或放宽请求 "
                       "(固件只许收窄, 照编译期默认值取交集)"},
        {{0x23, 0x03}, "新限位关不住在途目标 —— 武装中且该轴的当前参考/在途轨迹目标落在"
                       "新区间之外; 笛卡尔在途 (state != IDLE) 或同步轨迹在途时一律回本码"},
        {{0x24, 0x01}, "载荷长度不足 (需 1 字节: idx)"},
        {{0x24, 0x02}, "idx 越界"},

        // ---- 0x25 / 0x36 参数固化与恢复出厂 ----
        {{0x25, 0x04}, "武装中 (enabled 或 enable_pending 在途) —— 须先失能。"
                       "两处都会发本码: 受理时, 以及异步擦写执行前对武装态的复查"},
        {{0x25, 0x03}, "**异步保存失败** —— flash_store_save() 返回 false。"
                       "与 0x04 不同, 它在受理之后才到达 (main 循环里执行)"},
        {{0x36, 0x04}, "武装中 (enabled 或 enable_pending 在途) —— 须先失能"},
        {{0x36, 0x05}, "flash 参数保存进行中 —— 忙, 稍后重试 (交错会毁参数)"},
        {{0x36, 0x03}, "flash 失效化失败 —— 恢复出厂未生效 (RAM 与 flash 都保持旧值)"},

        // ---- 0x26 / 0x27 / 0x28 / 0x2B / 0x2C / 0x31 前馈与动力学旋钮 ----
        {{0x26, 0x01}, "载荷长度不足 (需 1 + 4xDOF 字节)"},
        {{0x26, 0x02}, "item 越界 (1..15), 或值非法 (NaN / item>=9 的符号契约不符: "
                       "fv>=0, fc0>=0, fc1<=0)"},
        {{0x26, 0x04}, "[仅 feat/hyy-model-import 分支] item 7 (gravity_scale) 武装态拒 "
                       "—— 须先失能"},
        {{0x27, 0x01}, "载荷长度不足 (需 4 字节: ff_mask)"},
        {{0x28, 0x01}, "载荷长度不足 (需 6 字节: item + sub + f32)"},
        {{0x28, 0x02}, "item 越界 (合法区间是 1..8 与 10..18; item 9 已保留给 0x2C 读 "
                       "ff_mask, 写它走 default 同样回本码), 或值非法 —— 只有三类: "
                       "NaN、item 5/6 的 sub>2、item 7 的 v 不属于 {0,1,2}。"
                       "幅值一律静默钳制只对有 clamp 的那些 item 成立: 它们过 "
                       "ff_clamp() 后照样 return true 回 ACK。例: payload_mass 钳 [0,20] "
                       "=> set_payload(-5) 回 ACK 并把质量静默钳成 0, 不是被拒"},
        {{0x28, 0x04}, "[仅 feat/hyy-model-import 分支] item 6 (gravity 向量) 武装态拒"},
        {{0x2B, 0x01}, "载荷长度不足 (需 1 字节: item)"},
        {{0x2B, 0x02}, "item 越界 (须 1..15)"},
        {{0x2C, 0x01}, "载荷长度不足 (需 2 字节: item + sub)"},
        {{0x2C, 0x02}, "item 越界 (须 1..18)"},
        {{0x31, 0x02}, "preset 非法 (须 0..2), 或载荷长度不足 (两者共用本码)"},

        // ---- 0x30 / 0x32 / 0x33 / 0x34 / 0x37 / 0x39 动力学模型在线导入 ----
        {{0x30, 0x01}, "载荷长度不足 (需 1 + 4xBODY_PARAMS 字节)"},
        {{0x30, 0x02}, "body_idx 越界, 或 staging 失败 (数值非法)"},
        {{0x32, 0x01}, "载荷长度不足 (需 2 字节: expected_mask)"},
        {{0x32, 0x04}, "武装中 —— 须先失能 (门控排在掩码校验之前)"},
        {{0x32, 0x07}, "**模型掩码不符** —— 产线最常见故障, 与「数值非法」完全不同"},
        {{0x32, 0x02}, "dyn_model_commit 失败"},
        {{0x33, 0x01}, "载荷长度不足 (需 4xJM_N 字节)"},
        {{0x33, 0x02}, "staging 失败 (数值非法)"},
        {{0x34, 0x01}, "载荷长度不足 (需 1 字节: body_idx)"},
        {{0x34, 0x02}, "body_idx 越界"},
        {{0x37, 0x04}, "武装中 —— 须先失能"},
        {{0x39, 0x01}, "载荷长度不足 (需 4xDOF 字节: q[7])"},

        // ---- 0x2D / 0x2E 300Hz 控制拍采集 ----
        {{0x2D, 0x01}, "载荷长度不足 (需 4 字节: n_ticks)"},
        {{0x2D, 0x02}, "超容量 —— 请求的拍数超过 LOG_MAX_SAMPLES"},
        {{0x2E, 0x01}, "载荷长度不足 (需 4 字节: offset)"},
    };
    return kTable;
}

// 通用档: code -> 语义 (跨命令)。具体档未命中时用。
// 通用档不是"可以随便猜"的意思 —— 每个 code 在固件里都有稳定的骨架含义。
const std::map<int, std::string>& build_err_code_text() {
    static const std::map<int, std::string> kTable = {
        {0x00, "固件没有实现这条命令 (default 分支, 无任何副作用)"},
        {0x01, "载荷长度不足"},
        {0x02, "字段非法 (非有限值 / 越界 / idx 或 item 非法)"},
        {0x03, "被拒 —— 逐命令而异 (常见: 未使能 / EMERGENCY 锁存 / 新限位关不住在途目标)"},
        {0x04, "被拒 —— 逐命令而异 (常见: 须先失能 / 零重力中 / 状态机不允许)"},
        {0x05, "忙 —— 通道被占或参数保存中, 稍后重试"},
        {0x06, "锁存 —— 须先 reset / clear_faults"},
        {0x07, "掩码不符, 或 CMODE 补写预算耗尽 (重发无用)"},
    };
    return kTable;
}

std::string hex2(int v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%02X", v & 0xFF);
    return buf;
}

}  // namespace

const std::map<std::pair<int, int>, std::string>& err_text() { return build_err_text(); }
const std::map<int, std::string>& err_code_text() { return build_err_code_text(); }

std::string err_reason(uint8_t cmd, uint8_t code) {
    // 三级查找 (照认不出也要把原始码带出来的口径, 不许吞):
    //   1. 具体档命中 -> 用它;
    //   2. 否则通用档命中 -> 用它, 并附上原始码;
    //   3. 都没有 -> 明说"未登记", 带上原始 cmd/code。
    // 第 2/3 档之所以必须带原始码: 固件新增一档错误码时, 这条路径就是唯一会让上位机
    // 看见"这是我没见过的码"的地方 —— 折叠成一句通用文本等于把它藏起来。
    const auto& table = err_text();
    const auto it = table.find({int(cmd), int(code)});
    if (it != table.end()) return it->second;
    const auto& gen = err_code_text();
    const auto git = gen.find(int(code));
    if (git != gen.end()) {
        return git->second + " [本命令未登记此码: cmd=0x" + hex2(cmd) +
               ", code=0x" + hex2(code) + "]";
    }
    return "**未登记的固件错误码** cmd=0x" + hex2(cmd) + ", code=0x" + hex2(code) +
           " —— 子版本可能比本 SDK 新, 请对照固件 usb_cmd.c 的 "
           "usb_cmd_reply(RSP_ERR...)";
}

const std::set<uint8_t>& enable_retryable_codes() {
    static const std::set<uint8_t> kCodes = {0x03};
    return kCodes;
}

void raise_firmware_error(const std::vector<uint8_t>& payload, const std::string& prefix) {
    const uint8_t cmd = payload.size() > 0 ? payload[0] : 0;
    const uint8_t code = payload.size() > 1 ? payload[1] : 0;
    const std::string base = prefix + "ERR [" + hex2(cmd) + "," + hex2(code) + "]";
    if (code == 0x00) {
        throw UnsupportedByFirmwareError(
            base + " —— 固件没有实现这条命令 (需烧入引入该命令的固件版本)", cmd, code);
    }
    throw CommandRejectedError(base + " —— " + err_reason(cmd, code), cmd, code);
}

}  // namespace litearm
