#include "litearm/arm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>

#include "litearm/cart.hpp"

#ifdef _WIN32
#  include <process.h>
#  define LITEARM_GETPID() ::_getpid()
#else
#  include <unistd.h>
#  define LITEARM_GETPID() ::getpid()
#endif

namespace litearm {

const char* const ZERO_G_GUARD_MESSAGE =
    "零重力保活正在进行, 拒绝其它下行命令 (会改写模式/看门狗, 与保活互相打架); "
    "先 zero_g_stop() 退出";

const char* const CART_IN_FLIGHT_GUARD_MESSAGE =
    "笛卡尔运动在途, 拒绝进入零重力 (中途进场会丢掉位置环、只剩重力前馈, "
    "臂会靠摩擦滑停): 先 movej() 受控接管收口, 或 emergency_stop() 急停, "
    "或等它结束再进入";

const char* const CART_IN_FLIGHT_UNCONFIRMED_MESSAGE =
    "取不到状态帧, 无法确认笛卡尔是否在途 —— 保守拒绝进入零重力 (未确认就放行, "
    "漏的正是'轨迹中途进场靠摩擦滑停'那一侧): 先 get_state(refresh=true) "
    "确认臂已停稳, 再重试";

const char* const DFU_ENABLED_MESSAGE =
    "使能中拒绝进入 DFU —— 跳转停 TIM3 后电机 100ms 就松开, 有重力负载会下垂; "
    "先 disable() 再调 enter_dfu()";

const char* const DFU_REVOKED_MESSAGE =
    "DFU 登记被撤销/未执行 —— 固件回了 ACK 但设备在等待窗口内没有离开 CDC "
    "(并发使能或交棒前向量表复读失败都会静默清掉登记, 都不回报); "
    "臂与控制链路原样可用, 确认已失能后重试即可";

namespace {

int current_pid() { return LITEARM_GETPID(); }

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::string fnum(double v, int prec = 1) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return buf;
}

/// `%.6g` 的数字格式化 —— **预检文案专用**。
///
/// ⚠ 别拿 `fnum` 顶替: 它是 `%.*f` 且默认 1 位小数, 报"第 1 轴目标 0.0123 越软限"会印成
///   `0.0` —— 调用方拿着这条文案去排查, 看到的数与实际发的数对不上。
///   而预检的**全部价值**就在"说清哪一个轴、什么值、什么界限"。
/// ⚠ 对 NaN / ±inf 有定义输出 (`nan` / `inf`), 这正是"非有限值"那条要印的东西。
std::string gnum(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

}  // namespace

// ===========================================================================
// LicenseInfo
// ===========================================================================

const char* license_state_name(int state) {
    switch (state) {
        case 0: return "not_activated";
        case 1: return "activated";
        case 2: return "activated_factory";
        default: return "unknown";
    }
}

std::string LicenseInfo::state_name() const {
    if (state == 0 || state == 1 || state == 2) return license_state_name(state);
    return "unknown_state_" + std::to_string(state);
}

std::string LicenseInfo::uid_hex() const {
    static const char* kHex = "0123456789abcdef";
    std::string out;
    out.reserve(24);
    for (uint8_t b : uid) {
        out.push_back(kHex[b >> 4]);
        out.push_back(kHex[b & 0xF]);
    }
    return out;
}

LicenseInfo LicenseInfo::decode(const std::vector<uint8_t>& payload) {
    // 本帧载荷的首字节是 state, 不是冗余帧 id —— 与 RSP_MODEL_STATUS / RSP_FF_VEC /
    // RSP_FF_SCALAR 那一类 (固件把本 id 冗余回填进 payload[0]) 不同。
    // 别照抄它们的 decode 去比 payload[0] == RSP_LICENSE。
    if (payload.size() != 26) {
        throw TransportError("RSP_LICENSE 帧长 " + std::to_string(payload.size()) +
                             "B, 期望 26B");
    }
    LicenseInfo info;
    info.state = int(payload[0]);
    info.ver = int(payload[1]);
    std::copy(payload.begin() + 2, payload.begin() + 14, info.uid.begin());
    info.cust_id = proto::read_u32le(payload, 14);
    info.issued = proto::read_u32le(payload, 18);
    info.flags = proto::read_u32le(payload, 22);
    return info;
}

// ===========================================================================
// ZeroGSession
// ===========================================================================

void ZeroGSession::stop() {
    if (arm_ != nullptr) {
        Arm* a = arm_;
        arm_ = nullptr;
        a->zero_g_stop(true);
    }
}

void ZeroGSession::reset() {
    if (arm_ != nullptr) {
        Arm* a = arm_;
        arm_ = nullptr;
        // 析构路径不能抛 —— C++ 里从析构函数抛异常会直接 terminate。故这一支吞掉全部异常;
        // 要拿到"保活曾中断"那条, 请显式调 stop() (它走 raise_on_lost=true)。
        try {
            a->zero_g_stop(false);
        } catch (...) {
        }
    }
}

bool ZeroGSession::active() const { return arm_ != nullptr && arm_->zero_g_active(); }

// ===========================================================================
// Arm —— 构造 / 连接 / 收尾
// ===========================================================================

Arm::Arm() = default;

Arm::Arm(const std::string& port) : port_(port) {}

Arm::Arm(const ArmOptions& opts)
    : port_(opts.port.value_or("")),
      transport_factory_(opts.transport_factory),
      min_firmware_(opts.min_firmware) {
    clock_ = opts.clock;   // 空则 `clock()` 回落进程级 SteadyClock
    q_tol = opts.q_tol;
    dq_tol = opts.dq_tol;
    arrive_frames = opts.arrive_frames;
    move_timeout = opts.move_timeout;
    // 构造点 ①: 与 write 口同寿 —— raw_write 里挂着它的清队钩子, 故必须早于任何写存在。
    cart_ = std::make_unique<CartPending>(cart_absorb_ttl(move_timeout), 0, this);
}

Arm::~Arm() {
    // 兜底收尾 —— 只委托给 close(), 自己一行业务都不写 (第二套清理逻辑必然漂)。
    try {
        close();
    } catch (...) {
        // 析构/ GC 时刻抛出去没人接; 解释器退出时更可能挂死。
    }
}

Arm& Arm::connect() { return connect_impl(std::nullopt); }
Arm& Arm::connect(const std::string& port) { return connect_impl(port); }

Arm& Arm::connect_impl(const std::optional<std::string>& port) {
    // 终态不因 connect() 复活 (唯一的出路是新建一个 Arm)。放在最前: 终态下连收尾动作
    // 都不做 (fail-fast、无副作用)。
    reject_if_in_dfu();
    // fork 守卫也放在最前: 否则下面那句"幂等早退"会在子进程里成功返回 —— 用户以为连上了,
    // 拿到的是一个没有读线程的会话。
    reject_if_wrong_process();
    // 幂等早退。比较的是当前链路的 port (不是 port_ 成员): 后者只是构造函数的入参。
    if (tr_ && (!port.has_value() || *port == tr_->port_name())) return *this;
    if (tr_) {
        // 只有"改靶"才走到这里 —— 先收干净旧会话再开新的。
        close();
    }
    std::string p;
    if (port.has_value()) {
        p = *port;
    } else if (!port_.empty()) {
        p = port_;
    } else {
        const auto found = find_cdc_port();
        if (found) p = *found;
    }
    if (p.empty()) {
        throw TransportError("未找到 STM32 CDC (VID:PID 1d50:606f), 请显式指定 port");
    }
    // 唯一的传输构造点 (reconnect = close + connect, 故也走这里)。
    // 注入了工厂就用工厂 —— 离线与真机走同一条会话装配路径。
    live_port_ = p;   // `port_string()` 报的就是它 (构造入参可能压根是空的)
    tr_ = transport_factory_ ? transport_factory_(p)
                             : std::unique_ptr<Transport>(new SerialTransport(p));
    a_ = std::make_unique<Ack>(this);
    // fork 守卫的落章点 —— 本会话属于这个进程。位置: 会话对象一建好就落, 早于握手写帧,
    // 也早于读线程起。
    pid_ = current_pid();
    // 读线程的起点只有这一处合法位置 —— Ack 已就绪、而下面那句握手的 expect 必须已经有人读。
    a_->start_reader(tr_.get());
    // 新会话 = 新配对状态。重建要做两半, 缺一半都会出问题:
    //   1. 旧对象走一次 clear_pending —— 把在途 token 标成"结局未知"并唤醒仍在等它的线程;
    //   2. 把旧的**在途条数**作为吸收额度继承给新对象 —— 旧会话那几条仍可能欠着应答,
    //      新对象一格额度都没有的话, 那条迟到应答一到就撞上"队列为空"判据, LiteArmError
    //      会从毫不相干的读路径里炸出来, 归因指向"固件与主机已错配", 真相是"我们自己重建
    //      了会话"。
    // 继承那一格的射程只有一条命令: 被吃掉的那条请求随后超时放弃, 而超时路径不再给它补
    // 额度 => 第二条起恢复正常。
    const int inflight = cart_ ? cart_->pending() : 0;
    if (cart_) {
        cart_->clear_pending("会话重建 (connect) —— 固件侧不会再发 0x4E, 结局未知");
    }
    cart_ = std::make_unique<CartPending>(cart_absorb_ttl(move_timeout), inflight, this);

    // 握手: 固件版本
    raw_write(proto::CMD_GET_FIRMWARE);
    try {
        const auto r = a_->expect(proto::RSP_FIRMWARE, 1.5, "get_firmware", true,
                                  proto::CMD_GET_FIRMWARE);
        firmware_ = trim(std::string(r.payload.begin(), r.payload.end()));
    } catch (...) {
        close();
        throw;
    }
    const auto ver = proto::parse_firmware_version(firmware_);
    if (!ver || firmware_.rfind(FIRMWARE_PREFIX, 0) != 0) {
        close();
        throw FirmwareMismatchError(
            "固件版本不符合约定: '" + firmware_ + "' —— 应为 " + FIRMWARE_PREFIX +
            "<主.次.修>-{7J|1J}; 请烧录 >=" + FIRMWARE_PREFIX +
            std::to_string(min_firmware_.major) + "." +
            std::to_string(min_firmware_.minor) + "." +
            std::to_string(min_firmware_.patch));
    }
    if (*ver < min_firmware_) {
        close();
        throw FirmwareMismatchError(
            "固件 " + firmware_ + " 过旧: 需 >=" + FIRMWARE_PREFIX +
            std::to_string(min_firmware_.major) + "." +
            std::to_string(min_firmware_.minor) + "." +
            std::to_string(min_firmware_.patch) + " (move_j 受控静止保持语义)");
    }
    fw_version_ = *ver;
    // 等待一帧状态定关节数
    const auto st = read_status(1.0);
    if (!st) {
        close();
        throw TransportError("连上但收不到状态帧");
    }
    n_ = st->n();
    // 笛卡尔能力探测 —— 连接时一次, 不是首次调用时才探。
    // 探测本身不抛"固件不支持" (它只是返回 false); 能从上面逃出来的是写失败 (链路问题)
    // —— 与版本握手失败同级: 关链路再抛, 不留半开的会话。
    try {
        cart_supported_ = probe(this);
    } catch (...) {
        close();
        throw;
    }
    // ★★ 最后一步: 建软限缓存 (供 `precheck_q_` / `home` 用)。
    //   ⚠ 位置在**版本门禁与能力探测之后**、且**在 try 之外** —— 它自己全程吞异常做
    //     fail-open, 把它圈进上面的 try 只会让"限位读不到"变成"连接失败"。
    //   ⚠⚠ 这一步会给连接期**多出 n 条 `0x24`**。参照实现(Python)的 connect() **不读软限**
    //     ⇒ 这是**与 Python 分叉**的一处, 已登记。跨语言对拍时连接期的下行两侧本来就不同名,
    //     故对拍一律**从 connect 之后开始录**(见 /tmp/gen_conformance.py)。
    refresh_limits();
    return *this;
}

Arm& Arm::reconnect() {
    close();
    return connect();
}

Arm& Arm::reconnect(const std::string& port) {
    close();
    return connect(port);
}

void Arm::close() {
    // 第一步收保活线程必须是第一步 (它持有一个写者; 残留会让进程退出时打哑 CDC)。
    // 收不干净 (串口写卡住) 时 zero_g_stop 会抛错 —— teardown 不能因此挂死或半途而废,
    // 这里吞掉异常后照常关链路。
    if (pid_ != -1 && current_pid() != pid_) {
        // fork 出来的子进程里, 本方法**不做传输层收尾** —— 那会永久挂死。三条**独立**的
        // 死锁, 各自实测过, 缺一条都不足以解释:
        //   * `SerialTransport::close` 要取 rlock, 而父进程的读线程几乎一直持着它 =>
        //     子进程继承到一把**已加锁**的互斥量, 而能解锁的那个线程在子进程里不存在;
        //   * `Ack` 的析构会 join 一条在子进程里**不存在**的线程 (join 永久阻塞);
        //   * ⚠⚠ **最隐蔽也最要紧的一条**: `Ack` 析构时必然拆掉它的两个
        //     `std::condition_variable` 成员, 而父进程的读线程正停在 `stop_cv_` 上 ——
        //     glibc 的 `pthread_cond_destroy` 会去取 condvar 的内部锁, 那把锁与已不存在的
        //     等待者绑在一起 ⇒ **永久阻塞**。最小复现与逐项剥离见 `Ack::~Ack` 的注释。
        //     ⇒ 这一条**躲不开**: 它在成员析构里, 只要 `~Ack` 跑起来就会发生。
        //     故这里必须 `release()` (不析构), 而**不是**靠 `Ack` 里的属主守卫。
        // 故这里**故意泄漏**两个句柄 (release 而不是 reset):
        //   * 传输出去的那个继承来的 fd 留给进程退出时由内核关闭 —— 它既不读也不写
        //     (守卫把命令全拒了), 父进程也另有自己的 fd, 故留着是无害的;
        //   * ⚠ **`a_.release()` 这一句是载重的, 不是"顺手再加一层保险"**: 改成
        //     `a_.reset()` 会立刻把上面第三条死锁引回来 (而 `Ack` 里的属主守卫拦不住它)。
        //     删它之前请先读 `Ack::~Ack` 的注释。
        // ⚠ 会话状态照清, 且**不取任何锁** (`clear_session_state` 会取 tx 表那把锁, 而
        //   零重力保活线程可能正持着它): 子进程里那几张表留着也无害。
        a_.release();
        tr_.release();
        firmware_.clear();
        fw_version_.reset();
        n_ = 0;
        cart_supported_ = false;
        cart_probe_silent_ = false;
        pid_ = -1;
        return;
    }
    if (zg_thread_.joinable() || zg_active_) {
        try {
            zero_g_stop(false);
        } catch (...) {
            // 关链路时保活故障不该阻断关闭
        }
    }
    // 停读线程必须排在 tr_->close() 之前: 否则它会从已经关掉的传输上抛 TransportError
    // => reader_error 非空 => 一次正常关闭被记成"链路丢失"。
    if (a_) a_->stop_reader();
    if (tr_) {
        tr_->close();
        tr_.reset();
        a_.reset();
    }
    clear_session_state();
}

void Arm::clear_session_state() {
    // 清会话派生的状态 —— 关掉之后不许再报上一个会话的事实。
    firmware_.clear();
    fw_version_.reset();
    n_ = 0;
    cart_supported_ = false;
    cart_probe_silent_ = false;
    // fork 守卫的落章与会话同寿 —— 清掉之后守卫放行, 但 a_/tr_ 也在同一次 close 里清成
    // 空 => require() 抛 NotConnectedError, 两条路等价。
    pid_ = -1;
    live_port_.clear();
    // ⚠ 这里原来还有一段"清同帧节流的时间戳表"—— 节流整套已删 (2026-09-28, 见 README),
    //   那段随之消失。它当年存在的理由是"新会话第一帧不该撞上上个会话留下的时刻戳而
    //   被静默丢掉"; 现在**没有能丢掉帧的机制**, 那条理由本身不存在了。
}

// ===========================================================================
// 守卫
// ===========================================================================

void Arm::reject_if_in_dfu() const {
    if (!dfu_entered_) return;
    throw ArmIsInDfuError(
        "本 Arm 已交棒进 DFU (会话终态): 设备当时从 CDC 上消失, 此后所有会走到取帧/写帧"
        "收口的入口都抛本异常 (含 move_js/send_mit/send_mit_all 这类带本地 arity/idx "
        "预检的)。参数本身已非法时, 入口自己的本地预检会先抛 InvalidCommandError —— "
        "那是另一件事 (与终态无关), 不是说终态还能接着用。"
        "烧完固件请新建一个 Arm 连新固件 (本对象不会复活)");
}

void Arm::reject_if_wrong_process() const {
    const int pid = pid_;
    if (pid != -1 && current_pid() != pid) {
        throw ForkedSessionError(
            "本会话是在 PID " + std::to_string(pid) + " 里建立的, 当前进程是 " +
            std::to_string(current_pid()) +
            " —— fork 出的子进程不继承线程 (读线程不存在), 命令会真的写出去却永远等不到"
            "应答, 调用方只会看到'无应答'超时 (读状态更隐蔽: 静默回继承来的陈旧值)。"
            "故本包在这里 fail-closed: 一个字节都不下发。"
            "子进程要用臂请新建一个 Arm; close() 仍可调, 收尾不受影响");
    }
}

void Arm::guard_for_ack_wait() {
    // ① 终态必须排在"已关"之前
    reject_if_in_dfu();
    // ② fork 守卫: 唯一取帧口
    reject_if_wrong_process();
    if (a_ == nullptr) {
        throw NotConnectedError("链路已关闭 (Arm::close() 之后) —— 请先 connect()");
    }
}

void Arm::reject_if_cart_in_flight() {
    // 反向守卫 —— 笛卡尔在途时拒绝进入零重力。
    //
    // 为什么拦: 固件 0x06 丢掉位置环、只剩重力前馈 => 轨迹中途进场会让臂靠摩擦滑停
    // (coast), 比 movej 的受控接管 / emergency_stop 差 => 拦住它并把替代动作写进消息。
    // 只拦这一侧: movej/movej_sync 是受控接管; zero_g_stop/emergency_stop/disable 是
    // 降能量方向, 永远可达, 本方法也不碰它们。
    //
    // 两条信号缺一不可:
    //   ① 串行锁被持有 —— 三条笛卡尔入口全程持它, 所以"被持有"= 有一次调用正在途;
    //   ② 现取一帧状态的 bit10 (cart_busy) 为 1 —— 臂仍在跑。
    // 只用 ① 会漏掉 wait=false (那条调用已经返回、锁已释放, 而臂仍在跑);
    // 只用 ② 会漏掉"锁已被持有、规划还没起"的那一小段。
    // ② 必须现取, 不能用缓存: 那可能是很久以前的帧 —— 臂已停稳却仍报忙 = 假拒绝。
    if (!cart_serial_.try_lock()) {
        throw InvalidCommandError(CART_IN_FLIGHT_GUARD_MESSAGE);
    }
    cart_serial_.unlock();
    try {
        if (get_status_now().value.cart_busy()) {
            throw InvalidCommandError(CART_IN_FLIGHT_GUARD_MESSAGE);
        }
    // ⚠⚠ 下面这三条 catch 是**保守拒绝**那一侧, 而它的成立依赖 `get_status_now()` 在
    //   "0x40 被拒"时**确实抛**。已知缺陷 (`get_status_now` 的 done 谓词, 见那里的注释):
    //   被动流的一帧若落进判据窗口, 它会**误以为命令成功**, 拿回一帧**陈旧**的状态 ——
    //   于是这里 `cart_busy()` 读到 0, **守卫 fail-open 放行**。
    //   实测 TSan 下 `zero_g_is_refused_conservatively_when_the_status_is_unavailable`
    //   30 次里翻红 3 次 (2026-09-28)。
    //   ⇒ 语义与上游一致, 不在此处单独加宽限 (那关不严, 只会造出"已修好"的错觉);
    //     缺口登记在 README「已知继承的差异/缺口」。
    } catch (const MotionTimeoutError&) {
        throw InvalidCommandError(CART_IN_FLIGHT_UNCONFIRMED_MESSAGE);
    } catch (const TransportError&) {
        throw InvalidCommandError(CART_IN_FLIGHT_UNCONFIRMED_MESSAGE);
    } catch (const CommandRejectedError&) {
        throw InvalidCommandError(CART_IN_FLIGHT_UNCONFIRMED_MESSAGE);
    }
}

// ===========================================================================
// 收口: require / 报文出口
// ===========================================================================

Ack& Arm::require() {
    reject_if_in_dfu();
    reject_if_wrong_process();
    if (a_ == nullptr) throw NotConnectedError("未 connect()");
    return *a_;
}

bool Arm::pump_until(double timeout, const std::function<bool(Ack&)>& done,
                     const std::string& label) {
    // 驱动型读者的收口: 等到 done(a) 为真、或链路没了、或窗口用尽。
    // 它一条帧都不认领 —— 状态帧由读线程填进单槽, 本方法只是等单槽前进到满足条件。
    (void)label;
    Ack& a = require();
    const double end = clock().now_s() + timeout;
    Ack::MuLock lk(a.mu, a.dbg_mu_owner);
    a.dbg_check("pump_until");
    while (true) {
        if (a.reader_error) {
            throw TransportError("读线程已退出: " + a.reader_error_text);
        }
        if (!a.errors.empty()) {
            // 协议层异常 (典型: 笛卡尔收集器的"多了一条") —— 读线程把它存起来交给等待者。
            // 驱动型读者也要看它: 否则一次"配对已错位"只对队列等待者可见。
            std::exception_ptr e = a.errors.front();
            a.errors.pop_front();
            std::rethrow_exception(e);
        }
        if (done(a)) return true;
        const double left = end - clock().now_s();
        if (left <= 0.0) return false;
        a.cv.wait_for(lk.ul(), std::chrono::duration<double>(left));
    }
}

void Arm::raw_write(uint8_t cmd, const uint8_t* payload, size_t len) {
    // 最底层写口 —— 所有下行帧的唯一出口 (含零重力保活线程)。
    //
    // 保活线程刻意绕过 write_cmd (避开零重力守卫), 但不该绕过清队: 它发的 0x06 正是会
    // 静默作废在途笛卡尔规划的那一类。所以清队钩子挂在这一层。
    //
    // 发出前清, 不看应答: 固件的 cart_invalidate_before_motion 是 ctrl_accept_move_j
    // 的第一条语句, 位于全部门禁之前 —— 所以被 ERR 0x03/0x04/0x06 拒掉的命令同样已经作废
    // 了在途规划。写成"收到 ACK 才清队"就会在这条被拒路径上留下陈旧 token。
    //
    // ⚠⚠ **2026-09-28: 这里原来还有一道"同帧节流"闸** (排在清队之前, 只对 `droppable=true`
    //   的写口生效)。整套节流已删除 (参照实现里它是私有且默认全关的 ⇒ 行为上等价于无,
    //   故按 A 侧的做法**不做**; 见 README)。删掉之后本函数的不变式变强了:
    //   **进了这个函数就一定会写出去** —— "清队了但帧没发"这种半状态**不可构造**。
    reject_if_in_dfu();
    reject_if_wrong_process();   // fork 守卫: 唯一写口, 结构性地保证"子进程零下发"
    if (cart_clears_upon().count(cmd) != 0) {
        char hexbuf[8];
        std::snprintf(hexbuf, sizeof(hexbuf), "%02X", cmd);
        cart_->clear_pending(std::string("被 0x") + hexbuf +
                             " 作废 (固件侧 cart_invalidate_before_motion)");
    }
    // 清本命令的应答队列必须排在写之前: 我们的应答只可能在写之后到达, 故此刻清队不可能
    // 吃掉自己的应答。反过来会 100% 吃掉。
    if (a_) a_->drain_for(cmd);
    tr_->write_frame(cmd, payload, len);
}

void Arm::write_query(uint8_t cmd, const uint8_t* payload, size_t len) {
    // 查询类下行命令的唯一出口 (get_tcp/get_ik/参数读回/采集/自检)。
    // 与 write_cmd 的差别: 不做零重力守卫 —— 拖动示教期间仍应能读状态。
    // 两者都先 require() 再写: 未连接时必须是 NotConnectedError。
    require();
    raw_write(cmd, payload, len);
}

void Arm::write_cmd(uint8_t cmd, const uint8_t* payload, size_t len, bool guarded) {
    // 动作类下行命令的唯一出口 —— 零重力保活期在这里统一拒绝。
    // guarded=false 只给「降能量」方向的安全动作 (急停/失能) —— 它们必须永远可达。
    if (guarded && zg_active_) {
        throw InvalidCommandError(ZERO_G_GUARD_MESSAGE);
    }
    write_query(cmd, payload, len);
}

void Arm::cmd_expect_ack(uint8_t cmd, const uint8_t* payload, size_t len,
                         const std::string& label, double timeout) {
    Ack& a = require();
    write_cmd(cmd, payload, len);
    a.expect(proto::RSP_ACK, timeout, label, true, cmd);
}

bool Arm::link_dead(std::string* why) const {
    if (a_ == nullptr) {
        if (why) *why = "会话已关闭";
        return true;
    }
    Ack::MuLock lk(a_->mu, a_->dbg_mu_owner);
    a_->dbg_check("link_dead");
    if (a_->reader_error) {
        if (why) *why = a_->reader_error_text;
        return true;
    }
    return false;
}

void Arm::on_cart_plan_reply(const std::vector<uint8_t>& payload) const {
    if (cart_) cart_->on_reply(payload);
}

// ===========================================================================
// 状态读取
// ===========================================================================

std::optional<RobotState> Arm::read_status(double timeout) {
    // 等到一条新的状态帧 (status_seq 前进), 返回它解码出的 state。
    Ack& a = require();
    const uint64_t seq0 = a.status_seq_now();
    pump_until(timeout,
               [seq0](Ack& a) { return a.status_seq_now() != seq0; }, "状态帧");
    return a.state_copy();
}

Msg<std::optional<RobotState>> Arm::get_state(bool refresh, double timeout) {
    Ack& a = require();
    std::optional<RobotState> st;
    if (!a.state_copy() || refresh) {
        st = read_status(timeout);
    } else {
        st = a.state_copy();
    }
    return wrap(st, proto::RSP_STATUS);
}

Msg<RobotState> Arm::get_status_now(double timeout) {
    Ack& a = require();
    write_query(proto::CMD_GET_STATUS);
    if (timeout <= 0.0) {
        // 不等待: 读线程一直在填, "现在这一刻的状态"就是缓存里那个。
        auto st = a.state_copy();
        if (!st) throw MotionTimeoutError("get_status_now: 还没有任何状态帧 (链路刚起?)");
        return wrap(*st, proto::RSP_STATUS);
    }
    const uint64_t seq0 = a.status_seq_now();
    const QueueKey err_key{proto::RSP_ERR, int(proto::CMD_GET_STATUS)};
    auto done = [&](Ack& a) {
        // 本命令被拒也算"有结论": 0x40 被拒时固件回 ERR{0x40,code}。
        // 只看回显本命令码的那条: 别人的 ERR 落在别的队列里, 不是我们的结论。
        //
        // ⚠⚠ **已知缺陷 (上游同款, 移植刻意保留原语义)**: 下面这后半条判据认的不是
        //   "我们那条命令的应答", 而是**任何**一帧新状态帧 —— 而固件的 100Hz 被动流
        //   与我们的 `0x40` 是两条独立的线, 谁先被读线程投递没有保证。
        //   若在 `seq0` 采样与首次判据求值之间落进一帧状态帧, 判据**当场成立**, 而那条
        //   `ERR{0x40,code}` 可能还排在 `resp_` 里没被投递 ⇒ 下面查一眼 ERR 队列 (空)
        //   便一路落到 `state_copy()` 成功 ⇒ **固件拒了, 函数报成功**。
        //   实测: 调用期间一帧状态帧都没交付时 150/150 正确抛出; 交付了恰好 1 帧时,
        //   150 次里有 26~35 次报成功。
        //
        //   为什么**不在这修**: 状态帧里**没有命令回显** (那是 ACK/ERR 才有的), 所以
        //   "这一帧是不是我们那条命令的应答"在协议上**不可判定**。能给"失败"分支加宽限
        //   窗口, 但关不严 —— 只是把窗口压小, 却会造出"已经修好了"的错觉, 那比明写着的
        //   已知限制更危险。另外这条路径在受支持的固件上**不可达**: `connect()` 的版本门
        //   已经挡掉了没有 `0x40` 的旧固件 (0x40 的失败只可能来自固件 default 分支)。
        //   ⇒ 记在这里 + README「已知继承的差异/缺口」, 语义与上游保持一致。
        // ⚠ 共用例 (tests/test_commands.cpp) 必须先把链路静下来 (`lt::quiesce`) 再断言,
        //   否则它测的是上面这个窗口, 不是 ERR 映射。
        const auto it = a.queues.find(err_key);
        return (it != a.queues.end() && !it->second.empty()) ||
               a.status_seq_now() != seq0;
    };
    pump_until(timeout, done, "get_status_now");
    {
        Ack::MuLock lk(a.mu, a.dbg_mu_owner);
        a.dbg_check("get_status_now");
        const auto it = a.queues.find(err_key);
        if (it != a.queues.end() && !it->second.empty()) {
            const auto p = it->second.front().payload;
            it->second.pop_front();
            raise_firmware_error(p, "get_status_now: ");
        }
    }
    auto st = a.state_copy();
    if (!st) {
        throw MotionTimeoutError("get_status_now 无状态帧应答 (超时 " +
                                 fnum(timeout) + "s)");
    }
    return wrap(*st, proto::RSP_STATUS);
}

Msg<std::optional<std::array<double, 6>>> Arm::get_tcp(double timeout) {
    Ack& a = require();
    write_query(proto::CMD_GET_TCP);
    const auto r = a.expect(proto::RSP_TCP, timeout, "get_tcp", true, proto::CMD_GET_TCP);
    if (r.payload.size() >= 24) {
        const auto v = proto::unpack_f32s(r.payload.data(), 0, 6);
        return wrap(std::optional<std::array<double, 6>>(
                        std::array<double, 6>{v[0], v[1], v[2], v[3], v[4], v[5]}),
                    proto::RSP_TCP);
    }
    return wrap(std::optional<std::array<double, 6>>(std::nullopt), proto::RSP_TCP);
}

std::string Arm::last_reset_reason() {
    if (!tr_) return "";
    const auto r = proto::parse_boot_banner(tr_->text_log());
    return r.value_or("");
}

const std::string& Arm::port_string() const {
    // 链路在 ⇒ 报**实际**端口; 否则回退到构造入参。
    // ⚠ 两者确实可能不同: `Arm()` 不带端口时 `port_` 恒空, 真正连上的是
    //   `find_cdc_port()` 找到的那个。见 arm.hpp 里这处的说明。
    return live_port_.empty() ? port_ : live_port_;
}

// ---------------------------------------------------------------- 只读访问器
// (⚠ 与 Python 参照分叉的加法 —— 参照侧一个都没有; 见 arm.hpp)

bool Arm::is_connected() const {
    if (tr_ == nullptr || a_ == nullptr) return false;
    if (!tr_->is_open()) return false;
    if (a_->reader_error) return false;   // 读线程死了
    // 线程活着, 但对方可能**早就不发字节了** —— 用"最后一帧状态帧有多旧"判。
    // ⚠ 两个条件缺一不可: 只看指针会把"读线程死了"读成健康; 只看"读到过帧"会把
    //   "连着但不再发"读成健康。
    const auto st = a_->recv_stats(proto::RSP_STATUS);
    if (st.second <= 0.0) return false;   // 这一会话从没成功收到过状态帧
    return (clock().now_s() - st.second) < STATUS_STALE_MAX_S;
}

uint64_t Arm::status_seq() const { return a_ == nullptr ? 0 : a_->status_seq_now(); }

double Arm::msg_hz(uint8_t id) const {
    return a_ == nullptr ? 0.0 : a_->recv_stats(id).first;
}

std::optional<std::string> Arm::banner_version() const {
    if (tr_ == nullptr) return std::nullopt;
    const std::string log = tr_->text_log();
    // ⚠ 锚是**大小写敏感**的 "Litearm" —— 开机横幅里另有一处小写的
    //   `[litearm-usbcdc]`, 不能被它带跑。
    const size_t p = log.find(FIRMWARE_PREFIX);
    if (p == std::string::npos) return std::nullopt;
    size_t e = p;
    while (e < log.size()) {
        const char c = log[e];
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-')) break;
        ++e;
    }
    const std::string tok = log.substr(p, e - p);
    // ★ 必须**真的能解析成版本串**才认。只找到前缀不算 —— 那会把 "Litearm" 这种半截
    //   东西当版本报出去, 而本函数的全部用处就是"告诉现场板子上跑的是哪一版"。
    if (!proto::parse_firmware_version(tok).has_value()) return std::nullopt;
    return tok;
}

HostStats Arm::host_stats() const {
    HostStats s;
    if (a_ != nullptr) {
        Ack::MuLock lk(a_->mu, a_->dbg_mu_owner);
        a_->dbg_check("host_stats");
        s.dropped = a_->dropped;
        s.bad_status_frames = a_->bad_status_frames;
    }
    if (tr_ != nullptr) s.flush_failures = tr_->flush_failures();
    if (cart_ != nullptr) {
        s.cart_evicted_unclaimed = cart_->evicted_unclaimed;
        s.cart_extra_replies = cart_->extra_replies;
    }
    s.cart_probe_silent = cart_probe_silent_;
    return s;
}

ArmOptions Arm::options() const {
    // ⚠ 逐次组装成**现值**, 不是构造期那份死快照 —— 理由见 arm.hpp 里这条的说明。
    ArmOptions o;
    o.port = port_.empty() ? std::nullopt : std::optional<std::string>(port_);
    o.transport_factory = transport_factory_;
    // ⚠ 报的是**注入的那一份** (`clock_`), 没注入时是空 —— 不回报 `clock()` 那个
    //   "回落到 SteadyClock" 的结果: 那会让人以为有人显式注入过。
    o.clock = clock_;
    o.min_firmware = min_firmware_;
    o.q_tol = q_tol;
    o.dq_tol = dq_tol;
    o.arrive_frames = arrive_frames;
    o.move_timeout = move_timeout;
    return o;
}

// ===========================================================================
// 使能 / 安全
// ===========================================================================

void Arm::enable(int attempts) {
    // 只有固件明说"可重试"的码才重试 —— 判据是白名单 (= {0x03}), 不是"除锁存外都重试"。
    // 每次重试之间 sleep(0.3), 12 次尝试只有 11 次 sleep 约 3.3 秒。
    Ack& a = require();
    for (int i = 0; i < attempts; ++i) {
        write_cmd(proto::CMD_ENABLE);
        try {
            a.expect(proto::RSP_ACK, 1.0, "enable(" + std::to_string(i + 1) + ")", true,
                     proto::CMD_ENABLE);
            return;
        } catch (const CommandRejectedError& e) {
            // echo_cmd 保证到达这里的 ERR 回显的就是 0x10, 故 e.code 就是判据那一列。
            if (i >= attempts - 1 ||
                enable_retryable_codes().count(e.code) == 0) {
                throw;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }
    throw LiteArmError("enable 超时");
}

void Arm::disable() {
    // 同样绕过零重力守卫 (降能量方向)。
    write_cmd(proto::CMD_DISABLE, nullptr, 0, false);
    require().expect(proto::RSP_ACK, 1.2, "disable", true, proto::CMD_DISABLE);
}

void Arm::emergency_stop() {
    // 绕过零重力守卫 (降能量方向的安全动作必须永远可达)。
    write_cmd(proto::CMD_EMERGENCY_STOP, nullptr, 0, false);
    require().expect(proto::RSP_ACK, 1.2, "emergency_stop", true,
                     proto::CMD_EMERGENCY_STOP);
}

void Arm::reset() { cmd_expect_ack(proto::CMD_RESET, nullptr, 0, "reset"); }

void Arm::clear_faults() {
    cmd_expect_ack(proto::CMD_CLEAR_FAULTS, nullptr, 0, "clear_faults");
}

void Arm::set_motion_mode(int mode) {
    if (mode < 0 || mode > 255) {
        throw InvalidCommandError("mode 需 0..255 (给的是 " + std::to_string(mode) + ")");
    }
    if (mode != 0) {
        // 本固件只识别 0 —— 其它值固件回 ACK 但模式不变。响亮失败优于静默无效。
        throw InvalidCommandError(
            "set_motion_mode(" + std::to_string(mode) +
            "): 本固件只识别 0 (park 声明) —— 其它值固件回 ACK 但模式不变 "
            "(ctrl_set_motion_mode 只写 park_requested, 不写 g_arm.mode); "
            "要声明 park 请用 arm.park()");
    }
    const uint8_t p = uint8_t(mode);
    cmd_expect_ack(proto::CMD_SET_MOTION_MODE, &p, 1, "set_motion_mode");
}

void Arm::park() { set_motion_mode(0); }

void Arm::set_speed(int percent) {
    if (percent < 0 || percent > 100) {
        throw InvalidCommandError(
            "percent 需整数百分比 0..100; 若手里是 0..1 的倍率请乘 100");
    }
    const uint8_t p = uint8_t(percent);
    cmd_expect_ack(proto::CMD_SET_SPEED_PERCENT, &p, 1, "set_speed");
}

// ===========================================================================
// 零重力
// ===========================================================================

ZeroGSession Arm::zero_g(double period) {
    zero_g_start(period);
    return ZeroGSession(this);
}

void Arm::zero_g_start(double period) {
    if (!(period >= 0.005 && period < 0.10)) {
        throw InvalidCommandError(
            "保活周期需 ∈[0.005, 0.10) —— 固件看门狗超时是 0.10s");
    }
    // 反向守卫排在 zg_lock 之外: 它要读一帧状态 (上限 get_status_now 的 timeout),
    // 圈进锁里就会让 zero_g_stop / close 陪着等 —— 降能量方向的退出动作不该被一次查询拖住。
    // 位置仍在写 0x06 之前、置 zg_active 之前, 两条都满足。
    if (!zg_active_) {
        reject_if_cart_in_flight();
    }
    std::lock_guard<std::recursive_mutex> lk(zg_lock_);
    if (zg_active_) return;   // 已激活 = 幂等 no-op: 根本不写 0x06, 无 coast 风险
    const uint8_t on = 0x01;
    cmd_expect_ack(proto::CMD_ZERO_G, &on, 1, "zero_g(on)");   // 先确认进入再开线程
    zg_error_ = nullptr;
    zg_error_text_.clear();
    zg_stop_.store(false);
    // 顺序要紧: 先置 active (守卫立刻生效) -> 起线程 -> 最后才发布句柄。
    // 反过来时, 插进来的 zero_g_stop 会 join 一个尚未启动的 thread, 且异常在置
    // active=false 之前抛出 => active 永久卡 true: 保活一帧不发、动作命令被守卫永久拒绝。
    zg_active_ = true;
    zg_thread_ = std::thread(&Arm::zg_keepalive, this, period);
}

void Arm::zg_keepalive(double period) {
    // 保活线程: 只写不读。
    //
    // 不读是刻意的 —— 主线程正在等某条命令的 ACK, 保活线程若去读串口就会把那条 ACK 抢走,
    // 制造难查的偶发超时。
    //
    // 写走 raw_write (而不是直写 tr_): 它绕开的是零重力守卫, 不该顺带绕开清队 ——
    // 保活每 40ms 一条 0x06 正是会静默作废在途笛卡尔规划的那一类。
    while (!zg_stop_.load()) {
        std::this_thread::sleep_for(std::chrono::duration<double>(period));
        if (zg_stop_.load()) return;
        try {
            const uint8_t on = 0x01;
            // ⚠ 这里原来是全包唯一一处 `droppable=true` (声明"丢了也无所谓、可被节流")。
            //   同帧节流整套已删 (2026-09-28) ⇒ 本帧**一定会写出去**; 保活的节奏由上面那句
            //   `sleep_for(period)` 单独决定, 没有第二个人在丢它的帧。
            raw_write(proto::CMD_ZERO_G, &on, 1);
        } catch (...) {
            // 写失败 = 保活已断, 必须让调用方知道。
            // ⚠ 这里只取 zg_err_mu_, **不能**取 zg_lock_ —— zero_g_stop 持着 zg_lock_
            // 等 join, 本线程再取它就是死锁。
            {
                std::lock_guard<std::mutex> lk(zg_err_mu_);
                zg_error_ = std::current_exception();
                try {
                    std::rethrow_exception(zg_error_);
                } catch (const std::exception& ex) {
                    zg_error_text_ = ex.what();
                } catch (...) {
                    zg_error_text_ = "未知保活错误";
                }
            }
            zg_active_.store(false);
            zg_stop_.store(true);
            return;
        }
    }
}

void Arm::zero_g_stop(bool raise_on_lost) {
    std::lock_guard<std::recursive_mutex> lk(zg_lock_);
    const bool was_session = zg_thread_.joinable() || zg_active_.load();
    // 先置 stop 再 join: 线程在 period 到点后会看到它并退出。
    zg_stop_.store(true);
    zg_active_.store(false);
    if (zg_thread_.joinable()) {
        // std::thread 没有超时 join, 故这里是无界 join —— 但它是有界的: 保活线程要么在
        // sleep(period) (period < 0.10s), 要么在一次写里 (写有 0.5s 硬上界)。
        zg_thread_.join();
    }
    std::exception_ptr err;
    {
        std::lock_guard<std::mutex> lk(zg_err_mu_);
        err = zg_error_;
        zg_error_ = nullptr;
        zg_error_text_.clear();
    }
    std::exception_ptr exit_err;
    // 从未进入过就不发任何 0x06 —— 幂等退出不等于无中生有。
    if (was_session && tr_) {
        try {
            const uint8_t off = 0x00;
            write_cmd(proto::CMD_ZERO_G, &off, 1);   // 此刻 inactive, 守卫放行
        } catch (...) {
            exit_err = std::current_exception();
        }
    }
    if (raise_on_lost) {
        // 保活中断优先于退出帧失败上报 (前者才是臂为什么脱离零重力的原因)
        if (err) std::rethrow_exception(err);
        if (exit_err) std::rethrow_exception(exit_err);
    }
}

bool Arm::zero_g_active() const { return zg_active_.load(); }

std::exception_ptr Arm::zero_g_error() {
    std::lock_guard<std::mutex> lk(zg_err_mu_);
    return zg_error_;
}

std::string Arm::zero_g_error_text() {
    std::lock_guard<std::mutex> lk(zg_err_mu_);
    return zg_error_text_;
}

// ===========================================================================
// 到位等待 / 关节运动
// ===========================================================================

RobotState Arm::arrive(const std::vector<double>& target, double timeout) {
    Ack& a = require();
    const std::vector<double> tgt = target;
    const double budget = (timeout < 0.0) ? move_timeout : timeout;
    auto done = [&](const RobotState& st) {
        if (int(st.joints.size()) != int(tgt.size())) return false;
        const auto qs = st.q();
        for (size_t i = 0; i < tgt.size(); ++i) {
            if (std::fabs(qs[i] - tgt[i]) >= q_tol) return false;
        }
        for (double d : st.dq()) {
            if (std::fabs(d) >= dq_tol) return false;
        }
        return true;
    };
    int n_ok = 0;   // 连续到位拍数 (防到位瞬间误判/抖动)
    uint64_t seq0 = a.status_seq_now();
    const double end = clock().now_s() + budget;
    Ack::MuLock lk(a.mu, a.dbg_mu_owner);
    a.dbg_check("arrive");
    while (true) {
        if (a.reader_error) {
            throw TransportError("读线程已退出: " + a.reader_error_text);
        }
        if (a.state && a.status_seq_now() != seq0) {
            seq0 = a.status_seq_now();
            if (a.state->faulted()) {
                throw MotorFaultError("未到位即故障: FAULT " + a.state->fault_detail());
            }
            n_ok = done(*a.state) ? n_ok + 1 : 0;
            if (n_ok >= arrive_frames) return *a.state;
        }
        const double left = end - clock().now_s();
        if (left <= 0.0) break;
        a.cv.wait_for(lk.ul(), std::chrono::duration<double>(left));
    }
    throw MotionTimeoutError("未到位, 超时 " + fnum(budget) + "s");
}

// ===========================================================================
// 客户端预检 (⚠ 与 Python 参照分叉 —— 见 arm.hpp 与 README)
// ===========================================================================

void Arm::precheck_q_(const std::vector<double>& q, const char* who) const {
    // ★★ ① **非有限值 ⇒ 抛**。⚠⚠ 它**必须排在"缓存空 ⇒ 放行"之前**:
    //   NaN 与**任何**数的比较都为假 ⇒ 若让它走到下面那两条越限比较, 会**静默放行**;
    //   而 NaN 进到固件的 `clampf` 同样是 UB。这一条**与软限位无关** —— 缓存空也要拦。
    for (size_t i = 0; i < q.size(); ++i) {
        if (!std::isfinite(q[i])) {
            throw InvalidCommandError(
                std::string(who) + ": 第 " + std::to_string(i + 1) + " 轴的目标不是有限数 (" +
                gnum(q[i]) +
                ") —— 非有限值会让所有越限比较为假 (静默放行), 拒绝");
        }
    }
    // ★ ② **缓存空 ⇒ 放行** (fail-open)。一条限位都没拿到 ⇒ 不预检, 由固件钳制 + 到位判据兜。
    //   fail-closed 会让**所有**运动命令全废 —— 那是更坏的错误。
    if (limits_.empty()) return;
    // ★ ③ **长度不匹配 ⇒ 放行**: 拿到的限位不是这台臂的 ⇒ 判不了, 保守取"不拦"。
    //   ⚠ 按索引硬比会把目标与**别的轴**的限位配在一起 —— 那比不判更坏。
    //   ⚠ move_p 走的就是这一支 (位姿恒 6 个数, 而 limits_ 是 n 条) ⇒ 对 it 而言
    //     本函数**只兜非有限值**那一条。别把这段读成"笛卡尔目标也越限检查过了"。
    if (limits_.size() != q.size()) return;
    // ★ ④ 逐轴比; 越限 ⇒ 抛, 文案**点名轴号 / 目标值 / 限位** (现场定位用)。
    for (size_t i = 0; i < q.size(); ++i) {
        if (q[i] < limits_[i].q_min || q[i] > limits_[i].q_max) {
            throw InvalidCommandError(std::string(who) + ": 第 " + std::to_string(i + 1) +
                                      " 轴目标 " + gnum(q[i]) + " 越软限 [" +
                                      gnum(limits_[i].q_min) + ", " + gnum(limits_[i].q_max) +
                                      "]");
        }
    }
}

void Arm::precheck_speed_(double speed, const char* who) const {
    // ⚠⚠ 判据形态必须是 `!(speed >= 0.0 && speed <= 1.0)`: 写成 `speed < 0 || speed > 1`
    //   会让 **NaN 静默放行** (两个比较都为假) —— 与 `precheck_q_` 第 ① 条同一条理由。
    //   ⚠ 这和 `movej` 里原有的那句是**同一个判据**, 只是文案更完整 —— 保留两处是刻意的:
    //     那两处是 Python 逐字移植的原文案, 动它会破坏与参照的对齐。
    if (!(speed >= 0.0 && speed <= 1.0)) {
        throw InvalidCommandError(std::string(who) + ": speed 必须在 [0, 1] 的倍率内, 收到 " +
                                  gnum(speed) +
                                  " (⚠ 0..1 是**单条轨迹的倍率**; 全局百分比请走 set_speed(0..100))");
    }
}

void Arm::refresh_limits() {
    // 建软限缓存。**只在 `connect_impl` 末尾调一次**, 全程 fail-open。
    //
    // ⚠⚠ **本实现与 A 侧有一处结构性差异, 是有意的**: A 的 `refresh_limits_` 前面有一大段
    //   "等第一帧状态帧"的分片等待 (50 轮 × 推钟 1ms), 因为 A 的 `connect()` 不保证
    //   "已经解码过一帧状态帧"。**本仓的 connect() 已经保证了** —— 它有一句
    //   `read_status(); if (!st) throw "连上但收不到状态帧"`, 走到这里 `n_` 必然已知。
    //   ⇒ 那段等待在本仓是**死代码**, 直接省掉 (省掉还顺带避免了那段代码在
    //   "假传输 + 静默链路"下要退化成 skip 的麻烦)。
    //   ⚠ 若将来 connect 的时序改了 (不再保证状态帧), 这一段必须补回来。
    limits_.clear();
    try {
        limits_ = params().all_joint_params();
    } catch (const LiteArmError& e) {
        // ⚠ **fail-open**: 读不到限位 ⇒ 不预检、放行, 由固件钳制 + 到位判据兜。
        //   在这里抛出去只会让**连接失败** —— 那是更坏的错误。
        (void)e;
        return;
    }
    // ⚠⚠ **合理性校验** (规格没写, 是本仓自己加的; 不加会把未定义行为带进判据):
    //   任一条限位**非有限**、或 **`q_min > q_max` (倒置)** ⇒ **整份缓存弃用**。
    //   为什么必须**整份**弃用、而不是"跳过那一轴": `std::clamp(v, lo, hi)` 的**标准要求是
    //   `lo <= hi`** (违反即 UB), 而 `home` 的目标要靠它钳。留一条坏的 = 上面那条
    //   "非空 ⇒ 每条都有限且不倒置"的不变式就断了, 而那种断法在运行期**观测不到**。
    for (const auto& jp : limits_) {
        if (!std::isfinite(jp.q_min) || !std::isfinite(jp.q_max) || jp.q_min > jp.q_max) {
            limits_.clear();
            return;
        }
    }
}

RobotState Arm::movej(const std::vector<double>& q, double speed) {
    // 先判连接: 未连接时 n_ 还是 0, 拿它做 arity 校验只会给出误导性的"需要 0 个关节角"。
    Ack& a = require();
    if (!(speed >= 0.0 && speed <= 1.0)) throw InvalidCommandError("speed 需 0..1");
    if (int(q.size()) != n_) {
        throw InvalidCommandError("movej 需要 " + std::to_string(n_) +
                                  " 个关节角 (N=" + std::to_string(n_) + ")");
    }
    // ⚠ 软限预检排在 arity 之后 (arity 已保证 `q.size() == n_` ⇒ `precheck_q_` 的
    //   "长度不匹配 ⇒ 放行"那一支在这里**不可达**, 真正生效的是越限与非有限值两条)。
    // ⚠ 而上面的 speed 检查**保持 Python 原文案不动** —— 只有 `move_p` 那只速度预检是新增的。
    precheck_q_(q, "movej");
    std::vector<uint8_t> payload = proto::pack_f32s(q);
    const auto sp = proto::pack_f32le(speed);
    payload.insert(payload.end(), sp.begin(), sp.end());
    write_cmd(proto::CMD_MOVE_J, payload);
    a.expect(proto::RSP_ACK, 1.0, "movej", true, proto::CMD_MOVE_J);
    return arrive(q);
}

RobotState Arm::movej_sync(const std::vector<double>& q, double speed) {
    Ack& a = require();
    if (!(speed >= 0.0 && speed <= 1.0)) throw InvalidCommandError("speed 需 0..1");
    if (int(q.size()) != n_) {
        throw InvalidCommandError("movej_sync 需要 " + std::to_string(n_) +
                                  " 个关节角 (N=" + std::to_string(n_) + ")");
    }
    // 软限预检 —— 与 `movej` 同一条理由与同一个位置 (见那里)。
    precheck_q_(q, "movej_sync");
    std::vector<uint8_t> payload = proto::pack_f32s(q);
    const auto sp = proto::pack_f32le(speed);
    payload.insert(payload.end(), sp.begin(), sp.end());
    write_cmd(proto::CMD_MOVE_J_SYNC, payload);
    a.expect(proto::RSP_ACK, 1.0, "movej_sync", true, proto::CMD_MOVE_J_SYNC);
    return arrive(q);
}

bool Arm::pose_near(const std::array<double, 6>& tcp, const std::array<double, 6>& goal,
                    double pos_tol, double rpy_tol) const {
    // 位置逐分量 + 朝向的逐分量或旋转等价判定。
    //
    // 朝向必须补一条旋转等价判定: 固件 kin_rot_to_rpy 在 |pitch| ~= pi/2 (万向锁) 时强制
    // yaw=0, 于是同一个旋转的 rpy 分量可以差很远, 逐分量比较会永远判不到位 —— 表现为每次
    // move_p 都耗满 move_timeout。旋转判据是逐分量判据的超集 (先试逐分量, 不满足再看夹角),
    // 不会放松原语义。
    double dp = 0.0;
    for (int i = 0; i < 3; ++i) dp = std::max(dp, std::fabs(tcp[i] - goal[i]));
    if (dp >= pos_tol) return false;
    double dr = 0.0;
    for (int i = 3; i < 6; ++i) dr = std::max(dr, std::fabs(tcp[i] - goal[i]));
    if (dr < rpy_tol) return true;
    return rot::orient_angle({tcp[3], tcp[4], tcp[5]},
                             {goal[3], goal[4], goal[5]}) < rpy_tol;
}

RobotState Arm::move_p(const rot::PoseInput& pose, double speed, double pos_tol,
                       double rpy_tol) {
    require();   // 先判连接: 未连接时的形状报错会掩盖第一因
    // ⚠⚠ **容差闸 (纵深第一层, 排在 speed 预检之前、写帧之前)** —— 两个容差必须是**正有限值**。
    //   为什么必须拦: `pose_near` 的第一句是 `if (dp >= pos_tol) return false;`, 而
    //   **`dp >= NaN` 为假** ⇒ **位置那一半被静默跳过** ⇒ 到位只由朝向决定 ⇒
    //   **十米外也判"到位"**(本命令会当场成功返回, 而臂没到)。这与 `precheck_q_` ① /
    //   `precheck_speed_` 守的是同一族"NaN 让比较全为假 ⇒ 静默放行", 只是长在容差上。
    //   ≤ 0 的容差是"永远到不了位"(安全的另一侧), 但同样是调用方 bug ⇒ 一并拒绝 ——
    //   **不许**静默把整条 `move_timeout` 预算烧光。
    // ⚠ 判据形态必须是 `!(x > 0.0)`: 它同时覆盖 NaN 与 ≤0; 写成 `x <= 0.0` 对 NaN 为**假**
    //   ⇒ **拦不住**(与 `precheck_speed_` 同一条理由)。
    if (!(pos_tol > 0.0) || !(rpy_tol > 0.0)) {
        throw InvalidCommandError(
            std::string("move_p: pos_tol / rpy_tol 必须是**正有限值** (收到 pos_tol=") +
            gnum(pos_tol) + ", rpy_tol=" + gnum(rpy_tol) +
            ") —— pos_tol 为 NaN 会让**位置判定被静默跳过**(`dp >= NaN` 为假)"
            "⇒ 十米外也判到位; ≤ 0 则永远到不了位。本命令在**发帧之前**被拒。");
    }
    const std::array<double, 6> vec = rot::as_pose6(pose);
    // ⚠ 下面两条都是**与 Python 参照分叉**的本地拒发 (Python 直接发走、由固件钳):
    //   ② speed: `move_p` 的这只是六个运动入口里**唯一**参照没预检的。
    precheck_speed_(speed, "move_p");
    //   ③ 软限预检这一行**实际只兜"非有限位姿"**: 位姿恒 6 个数而 `limits_` 是 n 条
    //      ⇒ `precheck_q_` 的"长度不匹配 ⇒ 放行"**恒命中**, 越限那一支对 move_p 不可达。
    //      ⚠ 别把它读成"笛卡尔目标也做过越限检查了" —— 那需要 IK, 而 PC 侧没有本地 IK
    //        (发一条 0x43 问固件会凭空多一帧、还多一次超时窗口) ⇒ 不做。
    precheck_q_(std::vector<double>(vec.begin(), vec.end()), "move_p");
    std::vector<uint8_t> payload = proto::pack_f32s(vec);
    const auto sp = proto::pack_f32le(speed);
    payload.insert(payload.end(), sp.begin(), sp.end());
    write_cmd(proto::CMD_MOVE_P, payload);
    a_->expect(proto::RSP_ACK, 1.0, "move_p", true, proto::CMD_MOVE_P);
    const double end = clock().now_s() + move_timeout;
    while (clock().now_s() < end) {
        // 先看状态: 故障即抛
        const auto st = get_state(true, 0.1).value;
        if (st && st->faulted()) {
            throw MotorFaultError("move_p: FAULT " + st->fault_detail());
        }
        const auto tcp = get_tcp().value;
        if (st && tcp && pose_near(*tcp, vec, pos_tol, rpy_tol)) return *st;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw MotionTimeoutError("move_p 未到目标位姿, 超时 " + fnum(move_timeout) + "s");
}

RobotState Arm::home(double timeout) {
    // timeout 语义与 arrive 一致: <0 表示用 move_timeout。
    write_cmd(proto::CMD_HOME);
    Ack& a = require();
    a.expect(proto::RSP_ACK, 1.5, "home", true, proto::CMD_HOME);
    return arrive(std::vector<double>(size_t(n_), 0.0), timeout);
}

void Arm::move_js(const std::vector<double>& q, const std::vector<double>& dq,
                  const std::vector<double>& tau_ff) {
    // 终态守卫排在本 arity 预检之前: 终态下 n_ 已是 0, 让预检先跑只会报 "q 需 N 个" (N=0)
    // 这种误导性消息。
    reject_if_in_dfu();
    if (int(q.size()) != n_) throw InvalidCommandError("move_js q 需 N 个");
    const std::vector<double> dqv =
        dq.empty() ? std::vector<double>(size_t(n_), 0.0) : dq;
    if (int(dqv.size()) != n_) throw InvalidCommandError("move_js dq 需 N 个");
    std::vector<double> all = q;
    all.insert(all.end(), dqv.begin(), dqv.end());
    std::vector<uint8_t> payload = proto::pack_f32s(all);
    if (!tau_ff.empty()) {
        if (int(tau_ff.size()) != n_) throw InvalidCommandError("move_js tau_ff 需 N 个");
        const auto extra = proto::pack_f32s(tau_ff);
        payload.insert(payload.end(), extra.begin(), extra.end());
    }
    cmd_expect_ack(proto::CMD_MOVE_JS, payload, "move_js");
}

void Arm::send_mit(int idx, double q, double dq, double kp, double kd, double tau) {
    reject_if_in_dfu();   // 排在 idx 预检之前 (终态下 n_=0 => 否则报"idx 越界")
    if (idx < 0 || idx >= n_) throw InvalidCommandError("idx 越界");
    std::vector<uint8_t> payload{uint8_t(idx)};
    const auto vals = proto::pack_f32s(std::vector<double>{q, dq, kp, kd, tau});
    payload.insert(payload.end(), vals.begin(), vals.end());
    cmd_expect_ack(proto::CMD_MOVE_MIT, payload, "send_mit");
}

void Arm::send_mit_all(const std::vector<double>& q, const std::vector<double>& dq,
                       const std::vector<double>& kp, const std::vector<double>& kd,
                       const std::vector<double>& tau) {
    reject_if_in_dfu();   // 排在 arity 预检之前 (终态下 n_=0)
    const std::vector<std::pair<const std::vector<double>*, const char*>> arrs = {
        {&q, "q"}, {&dq, "dq"}, {&kp, "kp"}, {&kd, "kd"}, {&tau, "tau"}};
    for (const auto& kv : arrs) {
        if (int(kv.first->size()) != n_) {
            throw InvalidCommandError(std::string("send_mit_all ") + kv.second +
                                      " 需 N 个");
        }
    }
    std::vector<double> all;
    for (const auto& kv : arrs) all.insert(all.end(), kv.first->begin(), kv.first->end());
    cmd_expect_ack(proto::CMD_MOVE_MIT_ALL, proto::pack_f32s(all), "send_mit_all");
}

// ===========================================================================
// FF / 动力学调参
// ===========================================================================

void Arm::set_ff_mask(uint32_t mask) {
    if ((mask & ~uint32_t(proto::FF_ALL)) != 0) {
        throw InvalidCommandError(
            "ff_mask 超出范围 —— 有效位只有 FF_ALL=0x1FF (固件侧会静默掩码, "
            "误用高位会被折成 0=前馈全关)");
    }
    cmd_expect_ack(proto::CMD_SET_FF_FLAGS, proto::pack_u32le(mask), "set_ff_mask");
}

void Arm::ff_preset(int preset) {
    if (preset != 0 && preset != 1 && preset != 2) {
        throw InvalidCommandError("preset 需 0(全关)/1(出厂)/2(全开)");
    }
    const uint8_t p = uint8_t(preset);
    cmd_expect_ack(proto::CMD_FF_PRESET, &p, 1, "ff_preset");
}

const std::vector<std::pair<int, std::string>>& Arm::ff_vec_items() {
    static const std::vector<std::pair<int, std::string>> kItems = {
        {1, "friction"},   {2, "ki"},          {3, "i_max"},
        {4, "wall_stiff"}, {5, "wall_damp"},   {6, "wall_tau_max"},
        {7, "gravity_scale"}, {8, "inertia_scale"},
        {9, "friction_v"},  {10, "friction_fc0"}, {11, "friction_fc1"},
        {12, "zg_kp"},      {13, "zg_kd"},     {14, "zg_damping"},
        {15, "kd_extra"}};
    return kItems;
}

const std::vector<std::pair<int, std::string>>& Arm::ff_scalar_items() {
    static const std::vector<std::pair<int, std::string>> kItems = {
        {1, "fric_db"},    {2, "wall_margin"},   {3, "friction_slew"},
        {4, "payload_mass"}, {5, "payload_com"}, {6, "gravity"},
        {7, "friction_model"}, {8, "fric_v2_eps"},
        {10, "drag_gain"}, {11, "drag_db"},      {12, "drag_kd_margin"},
        {13, "zg_vel_thr"}, {14, "zg_engage_sec"}, {15, "zg_engage_kp"},
        {16, "zg_engage_kd"}, {17, "wall_fw_kd"}, {18, "hold_kp_gain"}};
    return kItems;
}

const std::vector<std::pair<int, std::string>>& Arm::ff_scalar_ro_items() {
    static const std::vector<std::pair<int, std::string>> kItems = {{9, "ff_mask"}};
    return kItems;
}

namespace {
bool in_items(const std::vector<std::pair<int, std::string>>& items, int item) {
    for (const auto& kv : items) {
        if (kv.first == item) return true;
    }
    return false;
}
}  // namespace

void Arm::set_ff_vec(int item, const std::vector<double>& values) {
    if (!in_items(ff_vec_items(), item) || values.size() != 7) {
        throw InvalidCommandError("set_ff_vec item∈1..15 且需 7 值");
    }
    std::vector<uint8_t> payload{uint8_t(item)};
    const auto vals = proto::pack_f32s(values);
    payload.insert(payload.end(), vals.begin(), vals.end());
    cmd_expect_ack(proto::CMD_SET_FF_VEC, payload, "set_ff_vec item" + std::to_string(item));
}

void Arm::set_ff_scalar(int item, int sub, double value) {
    if (!in_items(ff_scalar_items(), item) || sub < 0 || sub > 2) {
        throw InvalidCommandError("set_ff_scalar item∈{1..8,10..18}, sub∈0..2");
    }
    std::vector<uint8_t> payload{uint8_t(item), uint8_t(sub)};
    const auto v = proto::pack_f32le(value);
    payload.insert(payload.end(), v.begin(), v.end());
    cmd_expect_ack(proto::CMD_SET_FF_SCALAR, payload,
                   "set_ff_scalar" + std::to_string(item));
}

Msg<std::vector<double>> Arm::get_ff_vec(int item, double timeout) {
    if (!in_items(ff_vec_items(), item)) {
        throw InvalidCommandError("get_ff_vec item∈1..15");
    }
    Ack& a = require();
    const uint8_t p = uint8_t(item);
    write_query(proto::CMD_GET_FF_VEC, &p, 1);
    const auto r = a.expect(proto::RSP_FF_VEC, timeout,
                            "get_ff_vec item" + std::to_string(item), true,
                            proto::CMD_GET_FF_VEC);
    if (r.payload.size() < 2 + 7 * 4) {
        throw TransportError("RSP_FF_VEC 帧短");
    }
    // [0]=RSP id, [1]=item, 之后 7xf32
    return wrap(proto::unpack_f32s(r.payload.data(), 2, 7), proto::RSP_FF_VEC);
}

Msg<double> Arm::get_ff_scalar(int item, int sub, double timeout) {
    if (!in_items(ff_scalar_items(), item) && !in_items(ff_scalar_ro_items(), item)) {
        throw InvalidCommandError("get_ff_scalar item∈{1..18}");
    }
    if (sub < 0 || sub > 2) throw InvalidCommandError("get_ff_scalar sub∈0..2");
    Ack& a = require();
    const uint8_t p[2] = {uint8_t(item), uint8_t(sub)};
    write_query(proto::CMD_GET_FF_SCALAR, p, 2);
    const auto r = a.expect(proto::RSP_FF_SCALAR, timeout,
                            "get_ff_scalar item" + std::to_string(item), true,
                            proto::CMD_GET_FF_SCALAR);
    if (r.payload.size() < 3 + 4) throw TransportError("RSP_FF_SCALAR 帧短");
    // [0]=RSP id, [1]=item, [2]=sub, 之后 f32
    const auto v = proto::unpack_f32s(r.payload.data(), 3, 1);
    return wrap(v[0], proto::RSP_FF_SCALAR);
}

int Arm::get_ff_mask(double timeout) {
    return int(std::llround(get_ff_scalar(9, 0, timeout).value));
}

void Arm::set_gravity_scale(const std::vector<double>& gs) {
    if (gs.size() != 7) throw InvalidCommandError("gravity_scale 需 7 值");
    set_ff_vec(7, gs);
}

void Arm::set_inertia_scale(const std::vector<double>& isc) {
    if (isc.size() != 7) throw InvalidCommandError("inertia_scale 需 7 值");
    set_ff_vec(8, isc);
}

void Arm::set_payload(double mass, const std::array<double, 3>& com) {
    set_ff_scalar(4, 0, mass);
    for (int k = 0; k < 3; ++k) set_ff_scalar(5, k, com[size_t(k)]);
}

void Arm::set_gravity_vector(const std::array<double, 3>& g) {
    for (int k = 0; k < 3; ++k) set_ff_scalar(6, k, g[size_t(k)]);
}

void Arm::save_params() {
    cmd_expect_ack(proto::CMD_PARAM_SAVE, nullptr, 0, "save_params", 2.5);
}

// ===========================================================================
// IK / 授权
// ===========================================================================

std::vector<double> Arm::ik(const std::array<double, 6>& pose,
                            const std::vector<double>& q_seed, double timeout) {
    Ack& a = require();
    std::vector<double> seed;
    if (q_seed.empty()) {
        const auto st = get_state().value;
        if (!st) throw TransportError("ik 需要 q_seed, 但当前取不到状态帧");
        if (st->n() == proto::KIN_N) {
            seed = st->q();
        } else {
            // 台架/非整臂: 关节数 != 模型轴数, 按模型轴映射构造 7 轴种子
            seed.assign(proto::KIN_N, 0.0);
            const int ax = bench_model_axis;
            if (ax >= 0 && ax < proto::KIN_N && st->n() > 0) {
                seed[size_t(ax)] = st->joints[0].q;
            }
        }
    } else {
        seed = q_seed;
    }
    if (int(seed.size()) != proto::KIN_N) {
        throw InvalidCommandError("q_seed 需 7 个 (固件 IK seed 恒为模型 7 轴)");
    }
    std::vector<double> all(pose.begin(), pose.end());
    all.insert(all.end(), seed.begin(), seed.end());
    write_query(proto::CMD_GET_IK, proto::pack_f32s(all));
    const auto r = a.expect(proto::RSP_IK, timeout, "ik", true, proto::CMD_GET_IK);
    if (r.payload.size() < 7 * 4 + 1) throw TransportError("ik 应答帧短");
    const auto qs = proto::unpack_f32s(r.payload.data(), 0, 7);
    if (r.payload[7 * 4] == 0) throw IKError("目标不可达/IK 失败");
    return qs;
}

LicenseInfo Arm::license(double timeout) {
    Ack& a = require();
    write_query(proto::CMD_GET_LICENSE);
    const auto r = a.expect(proto::RSP_LICENSE, timeout, "get_license");
    return LicenseInfo::decode(r.payload);
}

void Arm::activate(uint32_t cust_id, uint32_t issued, uint32_t flags, const uint8_t* mac,
                   size_t mac_len, double timeout) {
    if (mac_len != 16) {
        throw InvalidCommandError("mac 需 16 字节 (两个 SipHash-2-4 标签), 给的是 " +
                                  std::to_string(mac_len));
    }
    if ((flags & ~0x1u) != 0) {
        // 保留位必须为 0 (固件会拒, 但那边折进 0x02 的聚合档 => 本地先说清楚)
        throw InvalidCommandError("flags 只允许 bit0 (产线码)");
    }
    std::vector<uint8_t> payload = proto::pack_u32le(cust_id);
    const auto a1 = proto::pack_u32le(issued);
    const auto a2 = proto::pack_u32le(flags);
    payload.insert(payload.end(), a1.begin(), a1.end());
    payload.insert(payload.end(), a2.begin(), a2.end());
    payload.insert(payload.end(), mac, mac + mac_len);
    try {
        cmd_expect_ack(proto::CMD_ACTIVATE, payload, "activate", timeout);
    } catch (const CommandRejectedError& e) {
        if (e.code != 0x02) throw;
        // 0x02 聚合档 => 只有回读能定性 (见头文件): 设备可能其实已经解锁。
        if (license().activated()) return;
        throw;
    }
}

// ===========================================================================
// 笛卡尔 (转发给 cart.cpp 的实现)
// ===========================================================================

CartPlan Arm::move_l(const rot::PoseInput& pose, double speed, bool wait) {
    return litearm::move_l(this, pose, speed, wait);
}

CartPlan Arm::move_c(const rot::PoseInput& pose_start, const rot::PoseInput& pose_via,
                     const rot::PoseInput& pose_goal, double speed, bool wait) {
    return litearm::move_c(this, pose_start, pose_via, pose_goal, speed, wait);
}

CartPlan Arm::move_path(const std::vector<rot::PoseInput>& poses, double speed,
                        bool wait) {
    return litearm::move_path(this, poses, speed, wait);
}

std::optional<CartPlan> Arm::poll_cart() {
    // 本方法不碰链路 —— 0x4E 由读线程直接交给收集器, 所以"认领"就只是看一眼待认领队列。
    require();
    const auto tok = cart_->claim_resolved();
    if (!tok || !tok->reply) return std::nullopt;
    CartPlan plan = CartPlan::from_reply(*tok->reply);
    raise_for_plan(plan);
    return plan;
}

// ===========================================================================
// DFU
// ===========================================================================

bool Arm::wait_until_link_lost(double timeout) {
    // 判据只有一条: reader_error 被置 —— 读线程从传输层收到异常就把它记在那里。
    // 设备被摘掉 / 跳进 ROM bootloader 时 read 会抛, 被包成 TransportError。
    // 正常关闭绝不能写成这一格 (close 必须先停读线程再关传输)。
    Ack& a = require();
    const double end = clock().now_s() + timeout;
    Ack::MuLock lk(a.mu, a.dbg_mu_owner);
    while (!a.reader_error) {
        const double left = end - clock().now_s();
        if (left <= 0.0) return false;
        a.cv.wait_for(lk.ul(), std::chrono::duration<double>(left));
    }
    return true;
}

void Arm::enter_dfu(double timeout) {
    Ack& a = require();   // 未连接 => NotConnectedError
    // 预检读一帧现取的状态 (refresh=true), 不用缓存: 拿一个十几秒前的 enabled 去判"能不能
    // 跳"两个方向都错。读不到状态帧时不拦 —— 门禁的权威在固件, 本地预检只为可读性。
    const auto st = get_state(true).value;
    if (st && st->enabled()) {
        throw InvalidCommandError(DFU_ENABLED_MESSAGE);
    }
    write_cmd(proto::CMD_ENTER_DFU);   // 空载荷; guarded 保持默认 (零重力期同样拒)
    a.expect(proto::RSP_ACK, DFU_ACK_TIMEOUT_S, "enter_dfu", true, proto::CMD_ENTER_DFU);
    if (!wait_until_link_lost(timeout)) {
        throw LiteArmError(DFU_REVOKED_MESSAGE);
    }
    try {
        close();   // 与 Arm::close 同一套收尾 (含收保活线程)
    } catch (...) {
        dfu_entered_ = true;
        throw;
    }
    // 置位放 finally-位置: 设备确实没了, 无论收尾是否出岔子, 本对象都不该再被当成一个活着
    // 的会话 (否则后续调用会从读路径抛 TransportError —— 归因指向"链路故障", 而真相是
    // "我们把它交出去了")。
    dfu_entered_ = true;
}

// ===========================================================================
// 子模块
// ===========================================================================

JointParams& Arm::params() {
    if (!params_) params_ = std::make_unique<JointParams>(this);
    return *params_;
}

ModelParams& Arm::model() {
    if (!model_) model_ = std::make_unique<ModelParams>(this);
    return *model_;
}

ArmLog& Arm::log() {
    if (!log_) log_ = std::make_unique<ArmLog>(this);
    return *log_;
}

Diagnostics& Arm::diag() {
    if (!diag_) diag_ = std::make_unique<Diagnostics>(this);
    return *diag_;
}

// ===========================================================================
// CLI
// ===========================================================================

namespace {

void print_state(const Msg<std::optional<RobotState>>& m) {
    if (!m.value) {
        std::cout << "(取不到状态帧)\n";
        return;
    }
    const RobotState& st = *m.value;
    std::cout << "mode=" << st.mode_name << " seq=" << st.seq << " flags=";
    if (st.flag_names.empty()) {
        std::cout << "-";
    } else {
        for (size_t i = 0; i < st.flag_names.size(); ++i) {
            if (i) std::cout << ",";
            std::cout << st.flag_names[i];
        }
    }
    std::cout << " hz=" << fnum(m.hz) << " age=" << fnum(now_s() - m.timestamp, 2)
              << "s\n";
    for (size_t i = 0; i < st.joints.size(); ++i) {
        const auto& j = st.joints[i];
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "  J%zu: q=% .3f dq=% .2f tau=% .2f T=% .0f/% .0fC err=%02X\n",
                      i + 1, j.q, j.dq, j.tau, j.t_mos, j.t_coil, j.err);
        std::cout << buf;
    }
}

}  // namespace

int main_impl(const std::vector<std::string>& argv) {
    std::string action = "status";
    std::string port;
    double speed = 0.3;
    std::vector<double> targets;
    for (size_t i = 0; i < argv.size(); ++i) {
        const std::string& a = argv[i];
        if (a == "--port" && i + 1 < argv.size()) {
            port = argv[++i];
        } else if (a == "--speed" && i + 1 < argv.size()) {
            speed = std::stod(argv[++i]);
        } else if (a == "-h" || a == "--help") {
            std::cout << "用法: litearm-cpp [--port PORT] [--speed S] "
                         "[status|fw|enable|disable|reset|emergency|movej|home|tcp] "
                         "[targets...]\n";
            return 0;
        } else if (action == "status" && targets.empty() &&
                   (a == "status" || a == "fw" || a == "enable" || a == "disable" ||
                    a == "reset" || a == "emergency" || a == "movej" || a == "home" ||
                    a == "tcp")) {
            action = a;
        } else {
            targets.push_back(std::stod(a));
        }
    }

    ArmOptions opts;
    if (!port.empty()) opts.port = port;
    Arm arm(opts);
    arm.connect();
    try {
        if (action == "fw") {
            std::cout << "firmware: " << arm.firmware() << " | n = " << arm.n() << "\n";
        } else if (action == "status") {
            const auto msg = arm.get_state();
            if (!msg.value) {
                std::cerr << "取不到状态帧 (链路无上行)\n";
                arm.close();
                return 1;
            }
            print_state(msg);
        } else if (action == "enable") {
            arm.enable();
            std::cout << "enabled\n";
        } else if (action == "disable") {
            arm.disable();
            std::cout << "disabled\n";
        } else if (action == "reset") {
            arm.reset();
            std::cout << "reset\n";
        } else if (action == "emergency") {
            arm.emergency_stop();
            std::cout << "emergency\n";
        } else if (action == "home") {
            arm.home();
            std::cout << "home done\n";
        } else if (action == "movej") {
            if (int(targets.size()) != arm.n()) {
                std::cerr << "movej 需 " << arm.n() << " 个关节角\n";
                arm.close();
                return 1;
            }
            arm.movej(targets, speed);
            std::cout << "movej done\n";
        } else if (action == "tcp") {
            const auto msg = arm.get_tcp();
            std::cout << "tcp: ";
            if (msg.value) {
                for (size_t i = 0; i < 6; ++i) {
                    if (i) std::cout << ", ";
                    std::cout << fnum((*msg.value)[i], 4);
                }
            } else {
                std::cout << "(取不到)";
            }
            std::cout << " (hz=" << fnum(msg.hz) << ")\n";
        }
    } catch (...) {
        arm.close();
        throw;
    }
    arm.close();
    return 0;
}

}  // namespace litearm
