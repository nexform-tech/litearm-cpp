// 错误层级 —— 镜像 litearm-python 的 errors.py (本包独立定义, 不依赖任何上游源码)。
#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "litearm/protocol.hpp"

namespace litearm {

/// 本包所有错误基类。
class LiteArmError : public std::runtime_error {
public:
    explicit LiteArmError(const std::string& what) : std::runtime_error(what) {}
};

/// 未连接/串口未开时调用。
class NotConnectedError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 在 fork 出的子进程里使用父进程建立的会话 —— 本包 fail-closed 拒绝。
///
/// 为什么必须拒绝 (不是保守, 是唯一正确的选择):
///   * 子进程继承 fd (os.write 照样把字节写到 CDC 上) 但不继承线程 —— 线程不被 fork
///     复制 => 读线程在子进程里不存在;
///   * 于是命令会真的发出去, 而应答永远没人读 => 子进程等满超时报"无应答",
///     用户以为命令失败、重试 => 重复下发;
///   * get_state() 那种读路径更隐蔽: 它不报错, 只是静默返回继承来的陈旧 state。
///
/// 所以口径是: 子进程里一个字节都不下发、也不认任何缓存, 立刻抛本异常。
/// 子进程要用臂 => 新建一个 Arm。
///
/// 继承自 NotConnectedError: catch(NotConnectedError) 与 catch(LiteArmError)
/// 都抓得到它 (这个会话在本进程里确实不可用)。
class ForkedSessionError : public NotConnectedError {
public:
    using NotConnectedError::NotConnectedError;
};

/// 串口读写/帧 CRC 校验失败。
class TransportError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 固件版本不符合约定 (应 Litearm<主.次.修>-{7J|1J}) 或不满足最低版本。
class FirmwareMismatchError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 参数/命令非法 (长度、取值越界等)。
class InvalidCommandError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 状态帧 flags FAULT 或进入 EMERGENCY (含 G7 单关节故障降级)。
class MotorFaultError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// move 超时未到位。
class MotionTimeoutError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// IK 求解失败 / 目标不可达。
class IKError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 固件显式 ERR (payload 含 [cmd, code])。
///
/// 两个字段都是可编程判定用的, 消息里另有可读文本 (由 err_reason 拼出):
///   * cmd  —— 固件回显的命令码。以它为准, 不要拿"我刚发了哪条"去推断:
///     异步路径 (0x02 MOVE_P 的 IK 收尾 / 0x25 参数保存) 的 ERR 与它对应的那条命令
///     不在同一次收发里。
///   * code —— 错误码。语义逐命令定义 (同一个 0x03 在 0x01 上是"未使能"、在 0x10 上
///     是"可重试"、在 0x23 上是"新限位关不住在途目标"); 0x00 恒为"固件没有这条命令"。
///
/// 带 drop_hold 门禁的那些运动命令 (0x01/0x03/0x04/0x05/0x07/0x2A 与笛卡尔 0x3A~0x3E)
/// 的 0x06 恒等于「掉线刚性持位锁存 (drop_hold)」—— 那是 SDK 唯一能确证该锁存存在的
/// 信号; 状态帧里读不到它 (见 RobotState::drop_hold_inferred)。
/// 两个例外: 0x02 MOVE_P 没有该门禁 (永不回 0x06); 0x10 ENABLE 的 0x06 判的是
/// EMERGENCY / joint_fault, 不含 drop_hold。
class CommandRejectedError : public LiteArmError {
public:
    explicit CommandRejectedError(const std::string& what, uint8_t cmd = 0,
                                  uint8_t code = 0)
        : LiteArmError(what), cmd(cmd), code(code) {}
    uint8_t cmd = 0;
    uint8_t code = 0;
};

/// 固件没有实现这条命令 (ERR 码 == 0x00)。
///
/// 固件 usb_cmd.c 的 default 分支对未实现命令只回 ERR{cmd,0x00} 且无任何副作用;
/// 已实现命令的错误码一律落在 0x01..0x07。故 0x00 是「固件无此命令」的唯一稳定哨兵
/// —— 比版本号可靠 (固件版本号不随命令增删而变)。
class UnsupportedByFirmwareError : public CommandRejectedError {
public:
    using CommandRejectedError::CommandRejectedError;
};

/// 固件规划被拒 (IK 无解 / 三点共线 / 超容量 / 越限位)。
///
/// 对应固件 RSP_CART_PLAN 的 err 字段 (cart_err_t) 里 2=COLLINEAR / 3=TOO_LONG /
/// 4=LIMIT 三种 —— 都是这条轨迹本身不成立, 臂一步没动。
/// 刻意不继承 CommandRejectedError: 固件这里回的是规划结果, 不是 ERR{cmd,code} 形态的
/// 「拒绝执行这条命令」, 硬套会凭空多出语义错误的 cmd/code 字段。
class CartesianPlanError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 这条笛卡尔请求被新请求取代 —— 预期内的接管, 不是失败。
///
/// 对应 err=5=CANCELED。抢占是正常用法 (新目标来了就接管旧的), 所以调用方必须能把它与
/// 「规划失败」分开: 混进通用异常会让正常抢占走成故障分支。不得继承 CartesianPlanError。
class MotionSupersededError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 0x4E 应答丢失 (固件单槽 pending 在突发下会吞应答) —— 未知结局。
///
/// 固件侧那份应答只有一个槽位: 同一 main 排空窗口内登记 >=3 条笛卡尔请求时会吞掉中段
/// 应答。这是本 SDK 自造的语义, 不是固件回码 —— 臂可能已经动了, 也可能没有, 只能靠
/// 回读实际状态判定, 不可当失败重发。
class CartReplyLostError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 本 Arm 已把设备交棒进 ROM bootloader —— 会话的终态, 不可再用。
///
/// 由 Arm::enter_dfu 置位, 且只在确认设备真的从 CDC 上消失之后 (判据 = 读路径抛
/// TransportError): 那一刻本对象关掉 transport, 此后任何走 require() / 读口 / 写口的
/// 调用 —— 含 connect()/reconnect() —— 都抛本异常。烧完固件要接着用臂请新建一个 Arm
/// (本对象刻意不提供"复活"入口)。
///
/// 刻意不继承 NotConnectedError: 那个的语义是"还没连 / 链路断了, 连上即可", 而这里不是
/// 一次可恢复的掉线 —— 是本会话按调用方要求主动把设备交了出去。
class ArmIsInDfuError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

// ---- 跨线错误 (原住在 litearm-server) ----

/// 该入口不可跨线: 未注册 / 返回活对象 / 返回不可序列化类型。
/// 与"固件拒绝"刻意分开: 这不是设备出错, 不要重试。
class NotRemoteable : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 该能力在当前后端上没有实现 (不是因为调用方式不对)。
class NotSupportedOnThisBackend : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 遥操态下拒绝手动控制。
class TeleopLockedError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

/// 遥操正在切换中 (enter/exit 未完成)。
class TeleopBusyError : public LiteArmError {
public:
    using LiteArmError::LiteArmError;
};

// ===========================================================================
// (cmd, code) 语义表 —— 固件 ERR 应答的唯一解读处。
//
// 真源 = 固件仓 User/litearm/hal/usb_cmd.c 的 usb_cmd_dispatch() 与
// User/litearm/kinematics/kin_runner.c (后台 IK 的收尾)。逐条取自固件原注释。
// code == 0x00 一律不在此表 —— 它恒等于"固件没有这条命令"(唯一真源是 usb_cmd.c 的
// default 分支), 由通用档承担。
// ===========================================================================

const std::map<std::pair<int, int>, std::string>& err_text();
const std::map<int, std::string>& err_code_text();

/// (cmd, code) -> 可读语义。未登记的码不静默 (三级查找, 认不出也把原始码带出来)。
std::string err_reason(uint8_t cmd, uint8_t code);

/// CMD_ENABLE (0x10) 的返回码里值得重试的那些 —— Arm::enable 的重试白名单。
///
/// 判据是白名单 ("只有它说可重试才重试") 而不是"黑名单": 固件对这几个码的注释是并列的:
///   0x03 "可重试 (反馈未齐 / CMODE 首写…重发即可)"      -> 重试
///   0x06 "锁存须先 reset (EMERGENCY/joint_fault)"        -> 否
///   0x07 "CMODE 补写预算耗尽, 重发无用, 须现场排查"      -> 否
///   0x00 固件没有这条命令 (default 分支)                  -> 否
/// 黑名单写法 ("除 0x06 外都重试") 会让 0x07 与 0x00 各自白耗 3.3 秒。
const std::set<uint8_t>& enable_retryable_codes();

/// 把固件 ERR 应答 ([cmd, code]) 映射成异常并抛出。
///
/// `code == 0x00` <=> 固件 usb_cmd.c 的 default 分支 —— 即未实现该命令, 且固件侧无任何
/// 副作用 (唯一真源)。已实现命令的错误码一律落在 0x01..0x07。
/// 抛出的具体类型按 code 定 (0x00 -> UnsupportedByFirmwareError), 故返回 void 而不是
/// 异常对象: 返回值会按基类切片, catch (UnsupportedByFirmwareError&) 就抓不到了。
///
/// 消息里的语义文本不在这里写死: 走 err_reason, 它的第 2/3 档会带上原始码 —— 固件新增一档
/// 时这里是唯一会如实说出"我没见过这个码"的地方。
/// .cmd / .code 两个字段照旧供调用方编程判定 (它们是本异常存在的理由)。
[[noreturn]] void raise_firmware_error(const std::vector<uint8_t>& payload,
                                       const std::string& prefix);

}  // namespace litearm
