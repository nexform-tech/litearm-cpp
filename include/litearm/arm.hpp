// Arm —— 高层子集: 直连 STM32 固件, 轨迹/IK/动力学/控制律全由固件内置承担。
//
// 设计定位:
//   - 单发语义: movej/move_p 一次下发, 固件 S 曲线自完成 + 到位受控静止保持;
//   - 不做 PC 侧轨迹/运动学; fk 仅当前位姿 (get_tcp), ik 走固件 get_ik;
//   - 笛卡尔运动只有一条路: 固件规划 move_l/move_c/move_path;
//   - 连续伺服 (move_js/send_mit) 需调用方按 >=10Hz 重发, 否则 0.1s 看门狗 fail-soft。
//
// 固件版本约定: get_firmware 应回 'Litearm<主.次.修>-{7J|1J}';
// <1.5.0 拒绝 (依赖 6+21N 状态帧 / joint_fault / enabled 位 + 静止保持语义)。
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "litearm/ack.hpp"
#include "litearm/cart.hpp"
#include "litearm/clock.hpp"
#include "litearm/diagnostics.hpp"
#include "litearm/errors.hpp"
#include "litearm/log.hpp"
#include "litearm/model.hpp"
#include "litearm/msg.hpp"
#include "litearm/params.hpp"
#include "litearm/protocol.hpp"
#include "litearm/rot.hpp"
#include "litearm/state.hpp"
#include "litearm/transport.hpp"

namespace litearm {

/// 后端要求的最低固件 (move_j 受控静止保持语义 + 6+21N 状态帧/joint_fault/enabled 位)。
inline const proto::FirmwareVersion MIN_FW{1, 5, 0, ""};
inline constexpr const char* FIRMWARE_PREFIX = "Litearm";

/// 零重力保活周期。固件 watchdog_timeout_s = 0.10s, 且 0x06 自带 watchdog_kick
/// (「重发即保活」)—— 取 0.04s 留 2.5 倍余量。
inline constexpr double ZG_KEEPALIVE_S = 0.04;
/// 等保活线程停下来的上限 (超过说明串口写阻塞)。
inline constexpr double ZG_JOIN_S = 1.0;

/// 状态帧**超龄**阈值 —— 超过这么久没收到新状态帧, `is_connected()` 就判"链路不健康"。
///
/// ⚠ 取值要比固件的广播周期 (100Hz ⇒ 10ms) **大两个数量级**: 这个判据要能容忍
///   一次 USB 抖动/丢帧, 只在"对方**持续**不发"时才翻转。取 2.0s 与 A 侧一致。
/// ⚠ 它与"读线程死了"是**两条独立的信号**: 后者立刻判死, 前者管"线程活着但没人喂它"。
inline constexpr double STATUS_STALE_MAX_S = 2.0;

/// `Arm::enter_dfu` 等 RSP_ACK{0x15} 的窗口。
inline constexpr double DFU_ACK_TIMEOUT_S = 1.2;
/// `Arm::enter_dfu` 观察"设备真的消失"的窗口 —— 取 0.3s 量级。
/// 固件侧从登记到交棒的上界是 100ms (三道门控共用同一个 age), 本窗口留的是 USB 重枚举
/// 的余量, 不是"等固件那个上界"。
inline constexpr double DFU_VANISH_TIMEOUT_S = 0.3;

/// "零重力保活期间拒绝其它下行命令"这条守卫的唯一文案 —— 两个抛出点共用。
///
/// 为什么必须是同一个常量: 两处的判据必须一致 —— 从前是各抄一份逐字节相同的字符串,
/// 改一侧就静默漂移 (调用方按文案归因, 两句不一样会被读成两种不同的拒绝)。
/// 文案本身仍是用户可见的归因入口: 它要说清"为什么拒"与"怎么退出"。
extern const char* const ZERO_G_GUARD_MESSAGE;

/// "笛卡尔在途时拒绝进入零重力"这条反向守卫的文案。
///
/// 只拒绝而不给出路会把操作员卡住 —— 文案必须把更好的停机动作写在里面:
/// movej() 是受控接管 (新 S 曲线从当前状态收口), 比 0x06 的"丢位置环、靠摩擦滑停"好得多。
extern const char* const CART_IN_FLIGHT_GUARD_MESSAGE;
/// 上面那条的"没能确认是否在途"变体 —— 保守拒绝时必须说清是哪一种拒绝。
extern const char* const CART_IN_FLIGHT_UNCONFIRMED_MESSAGE;
/// enter_dfu 的本地使能态预检文案。
extern const char* const DFU_ENABLED_MESSAGE;
/// enter_dfu 的"登记被撤销/未执行"文案 —— 收到 ACK{0x15} 之后设备在等待窗口内没有从
/// CDC 上消失。这条异常是那条静默撤销路径的唯一出口: 说清"什么都没发生", 并让 Arm 保持
/// 可用。
extern const char* const DFU_REVOKED_MESSAGE;

/// 设备授权记录 —— CMD_GET_LICENSE(0x2F) 的应答 (RSP_LICENSE 0x4F, 26B)。
///
/// 未激活时也回 UID, 而 cust_id/issued/flags 全 0。签发工具必须从本记录取 uid
/// (uid_hex 就是它要的形态) —— 不得改用 USB 序列号字符串, 两者不是一个东西。
///
/// 本包不解释 uid 那 12 字节的语义 (固件是 HAL_GetUIDw0/1/2 各 4B 直接 memcpy, 原始
/// 寄存器内存序), 只把设备给的原样转交 —— 签发器读的也是同一份字节, 故二者天然自洽。
struct LicenseInfo {
    int state = 0;    //: 0=未激活 / 1=已激活 / 2=已激活且产线模式
    int ver = 0;      //: 记录版本 (LICENSE_REC_VER, 当前 1)
    std::array<uint8_t, 12> uid{};  //: 12B, 本机 UID (原始寄存器内存序)
    uint32_t cust_id = 0;           //: 客户号 (未激活恒 0)
    uint32_t issued = 0;            //: 签发日 YYYYMMDD (未激活恒 0)
    uint32_t flags = 0;             //: bit0 = 产线码 (未激活恒 0)

    /// 是否已激活 —— 判据只是 state != 0 (未激活时其余字段无意义)。
    bool activated() const { return state != 0; }
    /// 产线码 (flags bit0) —— 不表示"激活与否", 故别拿它替代 activated()。
    bool factory_mode() const { return (flags & 0x1u) != 0; }
    /// state 的可读名; 认不出的码带上原值回 (不静默)。
    std::string state_name() const;
    /// UID 的 24 位小写 hex —— 签发器要的就是这个形态。
    std::string uid_hex() const;

    static LicenseInfo decode(const std::vector<uint8_t>& payload);
};

/// RSP_LICENSE 的 state 字节 -> 可读名。
const char* license_state_name(int state);

/// 构造参数 —— 对应 Python 侧 Arm(...) 的关键字参数。
struct ArmOptions {
    /// 串口路径。不设则自动发现 (VID:PID 1d50:606f)。
    std::optional<std::string> port;
    /// 传输工厂 (缺省真串口)。给它是为了接假传输做离线运行
    /// (litearm::testing::FakeTransport) —— 不需要真硬件就能起一个完整会话。
    /// 注入了工厂就必须同时给一个占位 port (如 "fake"), 否则无硬件时会在走到本工厂之前
    /// 就抛 "未找到 STM32 CDC"。
    ///
    /// ⚠ **所有权**: 工厂返回的 `unique_ptr` 由 `Arm` 接管 ⇒ 它**活不过 `close()`**
    ///   (`close()` 会把传输 reset 掉)。调用方若在 `close()` 之后还握着指向那个传输的
    ///   裸指针/引用, 之后再解引用就是 UB (实测踩过: 包装传输的审计表在 close 后读,
    ///   打印出一条 `len=1.5e19` 的垃圾随即段错误)。要在 close 之后还用, 自己拿
    ///   `shared_ptr` 兜住。
    std::function<std::unique_ptr<Transport>(const std::string& port)> transport_factory;
    /// 时间源。空 = `SteadyClock` (生产默认)。
    ///
    /// ⚠⚠ 注入假钟之前**务必先读 `clock.hpp` 开头那段**: 本仓的等待走条件变量的**真实**
    ///   时间, 只有 deadline/时间戳走注入的钟 ⇒ 一个不会自己前进的假钟会让"等超时真的
    ///   发生"的判据**挂死**。假钟只给"**纯比较**"类判据用。
    std::shared_ptr<const Clock> clock;
    proto::FirmwareVersion min_firmware = MIN_FW;
    double q_tol = 0.03;
    double dq_tol = 0.10;
    int arrive_frames = 3;
    double move_timeout = 15.0;
};

/// 宿主机侧计数快照 (`Arm::host_stats()`) —— 纯本地, 不发帧、终态下也返回。
///
/// ⚠ 逐字段的**判读方向**都写在各自的注释里, 别只看数字。这几个计数合起来的用处只有
///   一个: 把**几种在调用方看来一模一样**的"静默"分开 ——
///   "对方没发" / "发了但解不出" / "解出了但没人认领" / "认领了但队列爆了"。
struct HostStats {
    /// 队列封顶被挤掉的帧 (丢最旧) —— 「帧到了, 但没有等待者, 且积压超过上限」。
    uint64_t dropped = 0;
    /// **CRC 对但解码失败**的状态帧 —— 「链路活着, 但双方对帧格式的理解已经不一致」。
    /// ⚠ 它与 `dropped` 的区别是承重的: 前者是"读得懂但没人要", 后者是"根本读不懂"。
    uint64_t bad_status_frames = 0;
    /// 传输层 flush 失败累计 —— 「写下去了, 但没保证出得去」(只计数、不抛)。
    uint64_t flush_failures = 0;
    /// 笛卡尔在途配对表封顶挤出的条数。
    uint64_t cart_evicted_unclaimed = 0;
    /// 笛卡尔回帧比请求多的条数 (配对已错位的信号)。
    uint64_t cart_extra_replies = 0;
    /// 笛卡尔能力探测"3 次全安静 ⇒ fail-closed 判不支持"**是否发生过**。
    ///
    /// ⚠ 本仓把它建成**布尔**而不是计数 (A 侧是计数): 探测每个会话只跑一次
    ///   (`connect()` 里), 计数的第二个 1 永远不会有。语义见 `cart_supported()`。
    bool cart_probe_silent = false;
};

class Arm;
class _AckFriend;
//: 测试专用的访问口 —— 定义在 tests/test_support.hpp (必须是 litearm 命名空间里的名字,
//: `friend` 才认得它)。见 `Arm` 私有段里那条 friend 的说明。
struct TestAccess;

/// 零重力会话 —— `Arm::zero_g()` 的返回值: 既已启动保活, 又可用作 RAII 守卫。
///
///     {
///         auto zg = arm.zero_g();     // 进入 + 保活
///         ...                          // 可 get_state() 读状态, 不可发其它动作命令
///     }                                // 退出块时自动收尾 (不抛)
///
/// 或显式配对:
///     arm.zero_g_start();
///     ...
///     arm.zero_g_stop();               // 抛出版本
///
/// ⚠ C++ 的析构函数不能抛, 故析构走的是 zero_g_stop(false) 那一档 (吞掉保活中断与退出帧
/// 失败)。要拿到那些异常请显式调 stop()。
class ZeroGSession {
public:
    explicit ZeroGSession(Arm* arm = nullptr) : arm_(arm) {}
    ZeroGSession(const ZeroGSession&) = delete;
    ZeroGSession& operator=(const ZeroGSession&) = delete;
    ZeroGSession(ZeroGSession&& o) noexcept : arm_(o.arm_) { o.arm_ = nullptr; }
    ZeroGSession& operator=(ZeroGSession&& o) noexcept {
        if (this != &o) {
            reset();
            arm_ = o.arm_;
            o.arm_ = nullptr;
        }
        return *this;
    }
    ~ZeroGSession() { reset(); }

    /// 显式停止 (抛出版本, 语义同 Arm::zero_g_stop(true))。
    void stop();
    /// 提前收尾, 等价于析构 (不抛)。
    void reset();
    bool active() const;
    Arm* arm() const { return arm_; }

private:
    Arm* arm_ = nullptr;
};

class Arm {
public:
    Arm();
    explicit Arm(const std::string& port);
    explicit Arm(const ArmOptions& opts);
    ~Arm();

    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;

    // ---------- 连接 ----------

    /// 连上 CDC 并握手 (幂等)。
    ///
    /// 已经连着同一个目标时是 no-op (返回 *this, 不关链路、不重建会话) —— 重复调用最不该
    /// 打断的正是正在跑的那个会话。幂等不是"永远不做事": 显式给了另一个端口时真的改靶。
    /// 想强制重建同一个目标用 reconnect()。
    Arm& connect();
    Arm& connect(const std::string& port);
    /// 强制重建会话 —— connect() 已连即 no-op, 本方法无条件重建。
    Arm& reconnect();
    Arm& reconnect(const std::string& port);

    /// 收尾 (幂等) —— 收保活线程 -> 停读线程 -> 关链路 -> 清会话状态。
    /// 这是全部收尾路径的唯一实现 (disconnect 与析构都委托到这里)。
    void close();
    /// close() 的别名 —— 同一个操作的两个名字, 判据也完全一样。
    void disconnect() { close(); }

    // ---------- 状态读取 ----------

    /// 当前状态 (固件 100Hz 被动状态流的最近一帧)。
    /// refresh=false 且本会话已有缓存时不取帧, 直接回缓存。取不到帧时 value 为空。
    Msg<std::optional<RobotState>> get_state(bool refresh = false, double timeout = 0.5);

    /// 主动发 GET_STATUS 0x40 取一帧状态。
    /// 与 get_state() 的区别: 后者只消费被动流, 链路静默时只能靠超时发现。
    /// timeout <= 0 时语义是"立刻返回当前缓存"。
    Msg<RobotState> get_status_now(double timeout = 0.5);

    /// 当前末端位姿 pos[3]+rpy[3] (固件当前反馈 FK)。取不到时 value 为空。
    Msg<std::optional<std::array<double, 6>>> get_tcp(double timeout = 0.6);

    // ---------- 关节运动 ----------

    /// 关节运动 (CMD_MOVE_J 0x01): 一次下发, 固件 S 曲线自完成, 本方法等到位。
    RobotState movej(const std::vector<double>& q, double speed = 1.0);

    /// 同步 PTP movej (0x07): 全关节同时到达, 末端沿关节空间直线 q(s) = q0 + s*dq。
    ///
    /// 与 movej 的唯一差别是轨迹形状: movej 每轴一条独立 S 曲线, 各轴先到先停, 末端中间
    /// 轨迹不可预测; movej_sync 用一条路径标量 s: 0->1 的 S 曲线同时驱动全轴, 故所有关节
    /// 同一拍到达, 且末端落在关节空间线段上 (可预测、可校验)。
    /// 代价: 同步被最慢的轴拖住, 整体比 movej 慢 —— 这是可选模式而非替换。
    /// 固件状态帧仍报 mode=MOVE_J, 无法从状态区分同步/异步。
    RobotState movej_sync(const std::vector<double>& q, double speed = 1.0);

    /// 点到点运动 (CMD_MOVE_P 0x02): 固件后台自己 IK + 走 S 曲线, 本方法轮询 get_tcp
    /// 判到位, 返回 RobotState。
    ///
    /// **这是关节空间插值, 不是直线** —— 实测 30 mm 的目标, 末端横向摆出 17.6 mm。
    /// 要末端走直线/圆弧/多路点见 move_l / move_c / move_path。
    /// 本方法只收单个位姿; 多个位姿请用 move_path (语义不同)。
    RobotState move_p(const rot::PoseInput& pose, double speed = 1.0,
                      double pos_tol = 0.006, double rpy_tol = 0.03);

    /// 固件 CMD_HOME 0x2A: 各轴回 URDF 零位舒展姿。
    ///
    /// 固件侧速度写死 0.10 (低安全速度), 故本方法不接受 speed。与 movej({0,...}) 的差异:
    /// 固件注释明确该命令「允许从当前越软限/贴端发起」。仍受 tau 上限 / 看门狗 / 可急停
    /// 约束, 须先 enable()。
    RobotState home(double timeout = -1.0);

    /// 单发 move_js。连续伺服需调用方按 >=10Hz 重发, 否则 0.1s 看门狗 fail-soft。
    void move_js(const std::vector<double>& q, const std::vector<double>& dq = {},
                 const std::vector<double>& tau_ff = {});
    /// 单关节 MIT 透传 (0x04)。⚠ 绕过规划; 须调用方维持 >=10Hz。
    void send_mit(int idx, double q, double dq, double kp, double kd, double tau);
    /// 全臂单帧透传 (0x05)。⚠ 同上。
    void send_mit_all(const std::vector<double>& q, const std::vector<double>& dq,
                      const std::vector<double>& kp, const std::vector<double>& kd,
                      const std::vector<double>& tau);

    // ---------- 客户端预检 (⚠ 与 Python 参照**分叉**, 按移植纪律登记) ----------
    //
    // 下面两条是**发帧之前**的本地拒发。它们**不是**参照实现的行为: Python 直接把参数打包
    // 发走、由固件的 `clampf` 静默钳制。C++ 侧改成在本地拦下的理由**只有一条**:
    // 参数错到"比较全为假"时, 静默放行的后果是**臂真的动了**(或判据判错), 而调用方
    // 完全无从察觉。见 `src/arm.cpp` 里各自的实现注释。
    //
    // ⚠ 与 Python 分叉 ⇒ **不进**跨语言对拍(见 README「与原版的已知差异」)。
    //
    // ⚠ 公开是**刻意**的(与 A 侧一致): 它们是纯函数、无副作用, 调用方可以拿它做**预演**
    //   ("这一组目标发出去会不会被本地拒"), 而不必真的发一条命令。

    /// 关节目标预检: 非有限值 / 越软限 ⇒ `InvalidCommandError`。**发帧前**调用。
    ///
    /// ⚠ 未连接或软限缓存为空时**不拦**(fail-open) —— 只有"非有限值"那一条无条件生效。
    void precheck_q_(const std::vector<double>& q, const char* who) const;
    /// 速度预检: `speed` 必须 ∈ [0, 1]。**发帧前**调用。
    void precheck_speed_(double speed, const char* who) const;

    // ---------- 生活/安全 ----------

    /// 使能全关节 (CMD_ENABLE 0x10), 必要时重试。
    ///
    /// **只有固件明说"可重试"的码才重试** —— 判据是白名单 (= {0x03}), 不是"除锁存外都
    /// 重试"。每次重试之间 sleep(0.3), 12 次尝试只有 11 次 sleep 约 3.3 秒。
    /// 对锁存故障 (EMERGENCY / 单关节故障) 那 3.3 秒是纯浪费 —— 固件在第一次答复里就已经
    /// 说清了"须先 RESET", 而重试不可能让它变绿。
    void enable(int attempts = 12);
    /// 失能 —— 绕过零重力守卫 (降能量方向)。
    /// ⚠ 一旦失能, 臂不再被位置环抱住, 有负载会下坠。
    void disable();
    /// 急停 —— 绕过零重力守卫 (降能量方向的安全动作必须永远可达)。
    void emergency_stop();
    /// 清故障 + 重新锚定 (不是 MCU 重启)。
    void reset();
    /// 只清 RAM 里的故障位。
    void clear_faults();

    /// 发 SET_MOTION_MODE 0x20。
    ///
    /// **本固件只识别 mode=0**: 固件实现就是一行 park_requested = (mode == 0) —— mode
    /// 不写进 g_arm.mode, 其它值只是"清除 park 声明", 回 ACK 而模式不变。
    /// 非 0 值响亮失败 (从前只发一条告警就照发命令, 调用方拿到的是"成功"而臂还在旧模式)。
    void set_motion_mode(int mode);
    /// SET_MOTION_MODE J=0: 声明 park, 静止保持用全刚度。
    void park();

    /// 设置全局调速器百分比 0..100 (固件 0x21)。
    ///
    /// 这是整数百分比, 且固件侧是全局且持续的 (gov_ratio = percent/100), 与
    /// movej(speed=0..1) 的单条轨迹倍率不是一回事: set_speed(1) 是 1% 速度 (全局生效),
    /// 按 0..1 思维调用会得到"臂爬行"。
    void set_speed(int percent);

    // ---------- 零重力 (拖动示教) ----------

    /// 进入零重力拖动示教, 并启动保活。返回 RAII 会话对象。
    ///
    /// 固件 CMD_ZERO_G 自带 watchdog_kick (「重发即保活」), 且 SET_MOTION_MODE /
    /// SET_SPEED_PERCENT 都刻意不 kick —— 固件里没有别的命令能维持一个非 MOVE_J 模式不被
    /// 0.10s 看门狗掐死。故保活必须由后台线程按 period (默认 0.04s) 周期重发; 只发一次会在
    /// 0.1s 后掉回 fail-soft 持位, 而调用方只看到 ACK (静默失败)。
    ///
    /// ⚠ 退出前先把关节推回软限位内: 零重力模式会跳过位置越限锁存 (否则拖动到限位附近会
    /// 永久锁死), 但一旦退出, 越限锁存立刻生效且重启会重锁。
    /// ⚠ 笛卡尔在途时会被拒绝 (InvalidCommandError) —— 中途进场会丢掉位置环、靠摩擦滑停。
    ZeroGSession zero_g(double period = ZG_KEEPALIVE_S);

    /// 进入零重力并启动保活线程 (幂等: 已激活时直接返回)。
    void zero_g_start(double period = ZG_KEEPALIVE_S);
    /// 停保活并退出零重力 (幂等)。退出的 on=0 固件侧也幂等。
    /// raise_on_lost=true 时, 若保活曾因写失败中断, 在此抛出原异常 —— 臂已脱离零重力, 不能静默。
    void zero_g_stop(bool raise_on_lost = false);
    /// 保活是否仍在维持 (保活期写失败会立刻变 false)。
    bool zero_g_active() const;
    /// 保活中断的原因 (无则空)。zero_g_stop() 会消费并清空它。
    std::exception_ptr zero_g_error();
    /// 保活中断原因的文本 (无则空串)。
    std::string zero_g_error_text();

    // ---------- FF / 动力学调参 ----------

    /// 写 ff_mask (固件 0x27)。
    ///
    /// 只接受 FF_ALL 范围内的位。固件侧会做 mask & FF_ALL_MASK, 于是 0x1000 这类误用被
    /// 静默折成 0 = 前馈全关 (重力补偿被关掉, 臂会垂下来), -1 折成全开。宁可在这里报错,
    /// 也不让一个危险值悄悄变成另一个"合法"值。
    void set_ff_mask(uint32_t mask);
    /// 前馈预设: 0 全关 / 1 出厂 / 2 全开。
    void ff_preset(int preset);

    /// set_ff_vec 的 item -> 含义 (与固件 params_ff_vec 的守卫/case 一一对应)。
    static const std::vector<std::pair<int, std::string>>& ff_vec_items();
    /// set_ff_scalar 的 item -> 含义 (item 9 保留: 写口拒绝, 只给 0x2C 读 ff_mask)。
    static const std::vector<std::pair<int, std::string>>& ff_scalar_items();
    /// 0x2C 只读 item (写口不接受的)。
    static const std::vector<std::pair<int, std::string>>& ff_scalar_ro_items();

    /// 写 0x26 向量。item 见 ff_vec_items() (1..15), values 恒 7 个。
    void set_ff_vec(int item, const std::vector<double>& values);
    /// 写 0x28 标量。item 见 ff_scalar_items(); sub 仅 item 5/6 有意义 (0..2)。
    void set_ff_scalar(int item, int sub, double value);

    /// 读回 0x26 向量 (7 值)。与 set_ff_vec 同 item 编号。
    Msg<std::vector<double>> get_ff_vec(int item, double timeout = 1.0);
    /// 读回 0x28 标量。item 9 为只读扩展 (ff_mask)。
    Msg<double> get_ff_scalar(int item, int sub = 0, double timeout = 1.0);
    /// 读回 ff_mask (走 0x2C item 9; 值 <=0x1FF, f32 精确)。
    /// 刻意保持裸整数 (不返回 Msg): 它是 get_ff_scalar(9, 0) 的标量投影, round(Msg)
    /// 没有意义, 它只能消费裸值。
    int get_ff_mask(double timeout = 1.0);

    /// 重力缩放向量 (item 7, 7 值)。
    void set_gravity_scale(const std::vector<double>& gs);
    /// 惯量缩放向量 (item 8, 7 值)。
    void set_inertia_scale(const std::vector<double>& isc);

    /// 设末端载荷质量 (item 4) 与质心 (item 5, sub 0..2)。
    ///
    /// ⚠ mass 为负 (或 >20) 不会被拒 —— 固件走 ff_clamp(v, 0, 20) 静默钳制后回 ACK。
    /// 所以 set_payload(-5) 会成功, 而质量被钳成 0 => 重力前馈随之改变。
    /// 别靠"传负值探边界"来判断参数有没有写进去: 写进去的是钳后的值, 要确认请用
    /// get_ff_scalar 读回 (读回的是固件里钳后的真值)。
    void set_payload(double mass, const std::array<double, 3>& com = {0.0, 0.0, 0.0});
    /// 重力向量 (item 6, 3 值)。
    void set_gravity_vector(const std::array<double, 3>& g);

    /// 持久化当前运行时参数到 Flash (固件异步, 约 0.5-1s)。
    /// ⚠ 写 flash, 不可逆; 须先 disable()。
    void save_params();

    // ---------- 笛卡尔运动 (固件原生规划) ----------
    //
    // 规划 (采样/逐点 IK/Hermite 播放) 全在固件里, PC 侧只发点、收 0x4E 结果帧。
    // 能力 (受固件 #if LITEARM_CART_PLAN 约束) 在 connect() 时探一次。
    //
    // ⚠ 三条命令串行使用 (一条受理前不要发下一条): 固件的规划结果只有一个槽位, 连续流水线
    // 发多条会吞掉中段应答 (那时报 CartReplyLostError = 结局未知)。
    // ⚠ 被非笛卡尔命令 (movej/estop/disable/home/… 共 12 条 opcode) 作废时, 固件一条
    // 0x4E 都不发 —— 那些命令在发出前就会清掉本 SDK 的待配队列。

    /// 笛卡尔直线 (CMD_MOVE_L 0x3A): 末端沿起点->目标直线走, 姿态 slerp。
    /// 起点由固件取当前实测 TCP (不是参数)。
    CartPlan move_l(const rot::PoseInput& pose, double speed = 1.0, bool wait = true);
    /// 笛卡尔圆弧 (CMD_MOVE_C 0x3B): 起点 + pose_via + pose_goal 三点定圆。
    /// pose_via 的姿态被忽略 (固件只取它的位置三分量)。
    /// ⚠ pose_start 必须与调用时的实测 TCP 一致 (容差 6mm / 0.03rad, 含旋转等价判定)。
    CartPlan move_c(const rot::PoseInput& pose_start, const rot::PoseInput& pose_via,
                    const rot::PoseInput& pose_goal, double speed = 1.0,
                    bool wait = true);
    /// 多路点 (0x3C BEGIN -> 0x3D ADD x n -> 0x3E RUN): 依次经过 poses, 尖角。
    /// poses 最多 32 个。协议里没有倒角字段, 所以拐角是尖的。
    /// ⚠ BEGIN/ADD 中途失败时不补发任何"清状态"命令: 固件侧 RECV 态自带 2s 倒计时。
    CartPlan move_path(const std::vector<rot::PoseInput>& poses, double speed = 1.0,
                       bool wait = true);

    /// 认领一条还没有被取走的笛卡尔规划结果。本方法不碰链路。
    std::optional<CartPlan> poll_cart();

    /// 固件是否支持笛卡尔 (connect() 时探得)。
    bool cart_supported() const { return cart_supported_; }

    // ---------- 逆运动学 ----------

    /// 固件后台 IK: pose[6] + seed[7] -> q[7]。失败抛 IKError。
    ///
    /// seed 恒为模型 7 轴 (KIN_N), 与关节数解耦。台架 1J 上当前反馈只有 1 个值, 故未显式给
    /// q_seed 时按 BENCH_MODEL_AXIS(=5) 把台架电机的实测值填进 7 轴种子的对应位, 其余为 0。
    std::vector<double> ik(const std::array<double, 6>& pose,
                           const std::vector<double>& q_seed = {}, double timeout = 3.0);

    // ---------- 授权/激活 (固件 1.8.0+) ----------

    /// 读设备授权记录 (CMD_GET_LICENSE 0x2F -> RSP_LICENSE 0x4F, 26B)。
    ///
    /// 未激活的臂除 ENABLE 外一切照常 (售后/产线要能诊断) => 本方法在未激活时正常返回
    /// (activated() 为 false), 不抛异常 —— "未激活"是一种状态, 不是错误。
    /// 刻意不返回 Msg: RSP_LICENSE 没有固件发起的流量 —— 它是请求/应答式的设备身份记录。
    LicenseInfo license(double timeout = 1.0);

    /// 提交厂商签发的授权凭据 (CMD_ACTIVATE 0x3F)。**须先 disable()**。
    ///
    /// mac = 16 字节 (两个 SipHash-2-4 标签), 由厂商侧签发工具产出。
    /// 本包不产生也不需要密钥 —— 规格的硬要求: 客户侧只要有一份能算 MAC 的代码, 这套机制
    /// 就归零。
    ///
    /// ERR{0x3F,0x02} 是聚合档, 光看码会把"已经激活过"误判成失败 => 本方法在这一档回读一次
    /// license(): 只有设备确实 state == 0 才抛。
    void activate(uint32_t cust_id, uint32_t issued, uint32_t flags, const uint8_t* mac,
                  size_t mac_len, double timeout = 2.0);

    // ---------- DFU (SDK 里唯一的终端态操作) ----------

    /// 进 ROM 系统 bootloader (CMD_ENTER_DFU 0x15) —— 两段式, 会话终态。
    ///
    /// 设备随后重新枚举成 0483:DF11 (与 CDC 端口无关), 本对象不再可用: 成功返回后任何走
    /// 写口/读口的调用 (含 connect/reconnect) 都抛 ArmIsInDfuError; 烧完固件请新建一个 Arm。
    /// close() 照旧可用 (幂等空操作) —— teardown 在任何状态下都不该抛。
    ///
    /// 两段式的理由 (固件契约): 使能中不跳 (本地预检 st.enabled); 发空载荷等 ACK{0x15};
    /// ACK 只表示"已登记", 不表示"会跳" —— 登记的静默撤销点有三个 (都不回报而 ACK 是成功
    /// 的) => 还要等设备真的消失。消失则置终态; 窗口内没消失则抛"登记被撤销/未执行"且 Arm
    /// 保持可用 (那是静默撤销路径的唯一出口)。
    ///
    /// ⚠ 本方法不是线程安全的串行化点 (与 disable/emergency_stop 一样不取 _cart_serial)。
    void enter_dfu(double timeout = DFU_VANISH_TIMEOUT_S);

    // ---------- 子模块 ----------

    /// 关节级参数读写入口。
    JointParams& params();
    /// 动力学模型读写入口。
    ModelParams& model();
    /// 300Hz 控制拍采集入口。
    ArmLog& log();
    /// 固件自检入口。
    Diagnostics& diag();

    // ---------- 只读属性 ----------

    /// 关节数 (connect 后由第一帧状态定出; 未连接时为 0)。
    int n() const { return n_; }
    /// 固件版本字符串, 如 "Litearm1.8.0-7J"。
    const std::string& firmware() const { return firmware_; }
    /// 固件版本三元组 (未连接时为空)。
    const std::optional<proto::FirmwareVersion>& fw_version() const { return fw_version_; }
    /// 上次启动原因: "normal" | "iwdg-rst" | "" (未收到开机签名)。
    /// iwdg-rst 表示固件侧独立看门狗复位过 —— 这是固件唯一会外显该信息的地方。
    std::string last_reset_reason();
    /// 台架电机对应的模型轴 (仅供非整臂 IK 种子映射; 整臂不受影响)。
    int bench_model_axis = proto::BENCH_MODEL_AXIS;

    // ---- 下列 7 个只读访问器是**与 Python 参照分叉**的加法 (参照侧一个都没有) ----
    //
    // ⚠ 它们都是**纯本地快照**: 不发帧、不改状态、**终态 (DFU) 下也照常返回**。
    //   这是刻意的 —— 现场排查最需要它们的时候, 正是链路已经废掉的时候。

    /// 链路还在不在 (读线程活着 **且** 状态帧不过期)。
    ///
    /// ⚠ **两个条件都要**: 只看"对象已连接"会把"读线程死了"读成健康; 只看"读过帧"
    ///   会把"连着但对方不再发字节"读成健康。判据与 A 侧一致。
    bool is_connected() const;
    /// 是不是已经进了 DFU 终态 (终态下**任何**写都会抛 `ArmIsInDfuError`)。
    bool is_in_dfu() const { return dfu_entered_; }
    /// 状态帧累计计数 (成功**解码**的那些; 回绕 u16)。
    uint64_t status_seq() const;
    /// 某类上行帧本会话的平均到达频率 (Hz); 样本不足 2 条时返回 0。
    /// ⚠ 分母是**实际到达间隔** (不是"自 connect 起"), 故不会随链路空闲一起衰减。
    double msg_hz(uint8_t id) const;
    /// 从开机横幅里解析出的固件版本串 (`"Litearm1.8.0-7J"`); 没抓到横幅返回 `nullopt`。
    ///
    /// ⚠ 与 `firmware()` **不是**一回事: 后者来自握手 `0x41` 的应答, **权威**; 前者来自
    ///   开机噪声, 只在开机那一刻发一次 ⇒ 连上时板子早已启动的话多半是 `nullopt`。
    ///   它服务的是"现场想知道板子上跑的是哪版"这种诊断场景。
    std::optional<std::string> banner_version() const;
    /// 宿主机侧计数快照 (纯本地, 终态下也不抛)。
    HostStats host_stats() const;
    /// 当前生效的选项 (构造期入参 + 运行期可调旋钮的**现值**)。
    ///
    /// ⚠⚠ 与 A 侧**签名不同** (A 返回构造期那份 `const ArmOptions&`): 本仓的旋钮
    ///   (`q_tol` / `dq_tol` / `arrive_frames` / `move_timeout`) 是**公开成员、运行期可改**的
    ///   (见本节末), 返回一份构造期的死快照会让调用方读到**过期值** —— 那正是本仓刚修掉的
    ///   `port_string()` 那类"注释与实现脱节"的坑。故这里逐次组装、返回现值。
    ArmOptions options() const;

    // ---------- 用户可调旋钮 (与 Python 侧的公开属性对应) ----------

    /// 到位判据: 关节角容差 (rad)。
    double q_tol = 0.03;
    /// 到位判据: 关节速度容差 (rad/s)。
    double dq_tol = 0.10;
    /// 到位判据: 连续静止拍数 (防到位瞬间误判/抖动)。
    int arrive_frames = 3;
    /// 动作等待窗口 (move_p 到位 / 笛卡尔等 0x4E 都用它)。
    ///
    /// ⚠ CartPending 的吸收额度只在本属性被读到的那一刻取一次快照 (两个构造点:
    /// 构造函数与 connect 里那处重建), 且有下限 —— 传下去的是 cart_absorb_ttl(move_timeout),
    /// 不是裸的本属性。于是 connect 之后再改本属性会让"等待窗口"与"额度有效期"分叉, 且
    /// 两个方向都不安全。要改就 reconnect()。
    double move_timeout = 15.0;

    /// ⚠ 当前链路实际用的端口 (未连接时回退到构造入参, 都没有则为空)。
    ///
    /// ⚠⚠ **2026-09-28 改**: 原来是 `return port_`, 而 `port_` 只是**构造入参** ——
    ///   走自动发现 (`connect()` 不带端口) 时它恒为空, 于是"连上了却报空端口"。
    ///   头里的注释当年写的是「构造时给的, **或自动发现到的**」—— 那句承诺代码没做到。
    ///   真机实测复现 (`~/Desktop/litearm-cpp-对比测试报告.md` §5.1)。
    ///   ⇒ 现在按**注释原本的承诺**返回实际链路端口。
    const std::string& port_string() const;

private:
    friend class Ack;
    friend class CartPending;
    friend struct CartImpl;
    //: 测试专用的访问口 (定义在 tests/test_support.hpp)。
    //:
    //: 它存在的理由是"帧归属"那组判据**必须**能摸到会话内部的应答队列 —— 上游 Python
    //: 直接写 `arm._a._queues` (那边没有访问控制), C++ 里就得显式开一个口子。
    //: ⚠ 宁可开这一个**有文档、只有一处**的口子, 也不要把 `require()` / `raw_write()`
    //:   这类收口公开出去 —— 那会让"唯一写口"这条安全设计在公开面上破功。
    friend struct TestAccess;
    // 子模块 (params/model/log/diagnostics) 也是 Arm 的一部分 —— 它们只经这三处收口
    // (require / write_query / cmd_expect_ack) 碰链路, 故显式列进来, 而不是给一个更宽的
    // "谁都行"的口子。
    friend class JointParams;
    friend class ModelParams;
    friend class ArmLog;
    friend class LogReader;
    friend class Diagnostics;
    friend bool probe(Arm*);
    friend CartPlan move_l(Arm*, const rot::PoseInput&, double, bool);
    friend CartPlan move_c(Arm*, const rot::PoseInput&, const rot::PoseInput&,
                           const rot::PoseInput&, double, bool);
    friend CartPlan move_path(Arm*, const std::vector<rot::PoseInput>&, double, bool);

    /// 链路是否已死 (读线程退出) —— 给 CartPending::wait 判"该不该当场收场"。
    bool link_dead(std::string* why) const;

    Arm& connect_impl(const std::optional<std::string>& port);

    // ---- 收口: 所有路径都经过这三个 ----
    Ack& require();                       // 状态读取与查询类下行的收口
    /// 唯一写口 —— **发出去就一定会到 USB**(没有"静默丢弃"这条路径)。
    ///
    /// ⚠⚠ **2026-09-28 变更**: 原来这里有一个 `droppable` 形参, 让调用方声明"这一帧丢了
    ///   也无所谓", 同帧节流只对那种写口生效。**同帧节流整套已删除**(见 README
    ///   「与原版的差异」: 参照实现里那套 `_tx_allowed` / `set_tx_repeat_min_interval`
    ///   是私有且默认全关的, 行为上等价于无 ⇒ 本仓**不做**)。形参随之消失。
    ///   ⇒ 现在"一条等 ACK 的命令被静默丢掉"这条路径**不存在** —— 不是靠射程收窄挡住的,
    ///     而是**根本没有那把刀**。
    void raw_write(uint8_t cmd, const uint8_t* payload, size_t len);
    void raw_write(uint8_t cmd) { raw_write(cmd, nullptr, 0); }
    void raw_write(uint8_t cmd, const std::vector<uint8_t>& p) {
        raw_write(cmd, p.data(), p.size());
    }
    void write_query(uint8_t cmd, const uint8_t* payload, size_t len);  // 查询类出口
    void write_query(uint8_t cmd) { write_query(cmd, nullptr, 0); }
    void write_query(uint8_t cmd, const std::vector<uint8_t>& p) {
        write_query(cmd, p.data(), p.size());
    }
    void write_cmd(uint8_t cmd, const uint8_t* payload, size_t len, bool guarded = true);
    void write_cmd(uint8_t cmd, bool guarded = true) {
        write_cmd(cmd, nullptr, 0, guarded);
    }
    void write_cmd(uint8_t cmd, const std::vector<uint8_t>& p, bool guarded = true) {
        write_cmd(cmd, p.data(), p.size(), guarded);
    }
    void cmd_expect_ack(uint8_t cmd, const uint8_t* payload, size_t len,
                        const std::string& label, double timeout = 1.2);
    void cmd_expect_ack(uint8_t cmd, const std::vector<uint8_t>& p,
                        const std::string& label, double timeout = 1.2) {
        cmd_expect_ack(cmd, p.data(), p.size(), label, timeout);
    }

    void reject_if_in_dfu() const;
    void reject_if_wrong_process() const;
    /// Ack::wait 的守卫收口 (终态 -> fork -> 未连接), 顺序是承重的。
    void guard_for_ack_wait();
    /// 读线程拿到 0x4E 时的收集器入口 (Ack 调)。
    void on_cart_plan_reply(const std::vector<uint8_t>& payload) const;

    void clear_session_state();
    std::optional<RobotState> read_status(double timeout);
    /// 驱动型读者: 等到 done(a) 为真、或链路没了、或窗口用尽。一条帧都不认领。
    bool pump_until(double timeout, const std::function<bool(Ack&)>& done,
                    const std::string& label);
    RobotState arrive(const std::vector<double>& target, double timeout = -1.0);
    bool pose_near(const std::array<double, 6>& tcp, const std::array<double, 6>& goal,
                   double pos_tol, double rpy_tol) const;
    Msg<std::optional<RobotState>> msg_state(const std::optional<RobotState>& v);
    template <class T>
    Msg<T> wrap(T value, uint8_t ch) {
        Msg<T> m;
        m.value = std::move(value);
        if (a_ != nullptr) {
            const auto st = a_->recv_stats(ch);
            m.hz = st.first;
            m.timestamp = st.second;
        }
        return m;
    }
    /// 当前时间源 —— **空安全**: 没注入就回落到进程级的 `SteadyClock`。
    ///
    /// ⚠ 空安全不是防御性编程: `Arm::Arm() = default` 与 `Arm(port)` 两条构造路径**都不设**
    ///   `clock_` (只有收 `ArmOptions` 那条设), 直接解引用就是空指针。
    /// ⚠ 给 `Ack` / `CartPending` / `LogReader` 这些"只拿得到 `Arm*`"的友元用。
    ///   它们**不许**各自缓存一份引用 —— `Arm` 一旦被移动, 缓存的引用就悬空。
    const Clock& clock() const {
        return clock_ != nullptr ? *clock_ : steady_clock_instance();
    }

    void reject_if_cart_in_flight();
    void zg_keepalive(double period);
    bool wait_until_link_lost(double timeout);
    /// 建软限缓存 (`limits_`)。**只在 `connect_impl` 末尾调一次**。
    ///
    /// ⚠ 全程 **fail-open**: 读不到就清空缓存(不预检、放行), 绝不让连接失败 ——
    ///   fail-closed 会让**所有**运动命令全废, 那是更坏的错误。
    void refresh_limits();

    /// 注入的时间源。**空 = 用进程级的 `SteadyClock`** (见 `clock()`)。
    ///
    /// ⚠ 用 `shared_ptr` 而不是裸指针/引用: 传输 (`SerialTransport`) 也要拿它, 而传输的
    ///   寿命**短于** `Arm` (`close()` 会把它 reset 掉) ⇒ 裸指针会在那条路径上悬空。
    std::shared_ptr<const Clock> clock_;

    std::string port_;        ///< 构造入参 (可能为空 —— 走自动发现时)
    std::string live_port_;   ///< 当前链路实际用的端口; 未连接时为空
    std::function<std::unique_ptr<Transport>(const std::string&)> transport_factory_;
    proto::FirmwareVersion min_firmware_ = MIN_FW;
    std::unique_ptr<Transport> tr_;
    std::unique_ptr<Ack> a_;
    std::string firmware_;
    std::optional<proto::FirmwareVersion> fw_version_;
    int n_ = 0;

    /// 软限缓存 —— 连接时由 `refresh_limits()` 填一次; 空 ⇒ 不做软限预检 (fail-open)。
    ///
    /// ⚠ 不变式: **非空 ⇒ 每一条都有限、且 `q_min <= q_max`**。`refresh_limits()` 的合理性
    ///   校验就是为此: `std::clamp(v, lo, hi)` 要求 `lo <= hi`, 违反是 **UB**(实测
    ///   libstdc++ 静默返回 `lo`), 而 `home` 的目标要靠它钳。坏值只能**整份弃用**,
    ///   不能"跳过那一轴" —— 否则上面那条不变式就断了。
    std::vector<JointParam> limits_;

    // 零重力保活会话。
    // zg_lock_ 只串行化启停; 保活线程写错误时取的是 zg_err_mu_ —— 若它也去取 zg_lock_,
    // 就会与"持着 zg_lock_ 等 join"的 zero_g_stop 死锁。
    std::atomic<bool> zg_active_{false};
    std::thread zg_thread_;
    std::atomic<bool> zg_stop_{false};
    std::exception_ptr zg_error_;
    std::string zg_error_text_;
    std::mutex zg_err_mu_;
    mutable std::recursive_mutex zg_lock_;
    double zg_join_s_ = ZG_JOIN_S;

    std::unique_ptr<JointParams> params_;
    std::unique_ptr<ModelParams> model_;
    std::unique_ptr<ArmLog> log_;
    std::unique_ptr<Diagnostics> diag_;

    std::unique_ptr<CartPending> cart_;
    /// 笛卡尔入口的串行锁 —— 从登记 token 一直持到 0x4E 配对完成; wait=true 时再持到停稳。
    /// (理由: 0x3A/0x3B/0x3E 共用码空间, ERR 回显只能按码判, >=2 条在途时会互相认下对方的
    /// ERR/0x4E => 假成功。)
    ///
    /// 可重入 (recursive_mutex), 不是普通 mutex —— 嵌套点只有一个: move_path 把
    /// BEGIN + ADD x n + RUN 整段圈进本锁, 而 RUN 那一段本身也走 request_and_wait, 它也要
    /// 拿这把锁。换成普通 mutex 会在那个嵌套点把调用者自己锁死。
    ///
    /// ⚠ emergency_stop/disable/zero_g*/get_tcp 自己不许获取本锁 —— 降能量方向的动作与只读
    /// 查询必须永远可达 (持锁者可能阻塞到 move_timeout)。
    mutable std::recursive_mutex cart_serial_;

    bool cart_supported_ = false;
    bool cart_probe_silent_ = false;
    bool dfu_entered_ = false;
    int pid_ = -1;

};

// ---------------------------------------------------------------------------
// 简易 CLI (巡检/冒烟) —— 对应 python -m litearm / litearm-python
// ---------------------------------------------------------------------------
int main_impl(const std::vector<std::string>& argv);

}  // namespace litearm
