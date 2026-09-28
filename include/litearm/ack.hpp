// 应答等待器 —— 唯一读者 + 按 (上行 id, 回显码) 分队列的投递。
//
// 本设计的全部秘密: 帧的归属 = 它落在哪条队列。ACK{0x10} 与 ACK{0x11} 进两条队列,
// 因此永远配不错。读线程只投递、不判定; 判定由队列决定。
//
// 三条纪律:
//   1. **只投递, 不判定** —— "这帧是谁的"由队列决定, 不由线程决定;
//   2. **死了要响亮** —— 死因进 reader_error 并唤醒所有等待者, 绝不静默退出;
//   3. **正常关闭绝不许写那一格** —— 否则"我们主动收尾"会被读成"设备没了"。
#pragma once

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "litearm/clock.hpp"
#include "litearm/protocol.hpp"
#include "litearm/state.hpp"
#include "litearm/transport.hpp"

namespace litearm {

class Arm;

/// 一条帧该进哪条队列 —— (上行 id, 回显码或 kNoEcho)。
///
/// 只有 RSP_ACK / RSP_ERR 的 payload[0] 是"原命令码", 因此只有它们需要第二维来区分
/// 同 id 的不同命令。别的帧的 payload[0] 是数据 (关节号 / item 索引 / 文本首字节),
/// 拿它当归属键是错的。
using QueueKey = std::pair<uint8_t, int>;

inline constexpr int kNoEcho = -1;

QueueKey read_key(uint8_t c, const std::vector<uint8_t>& payload);

/// 等一条 want 应答时该看哪几条队列。
///
/// `want` 是 RSP_ACK/RSP_ERR 时 `echo_cmd` 必填 (归属判据就是那个回显码, 不给就等于
/// "随便哪条 ACK 都算我的" —— 那正是并发下两条命令互吃应答的成因)。这里直接拒绝而不是
/// 退化成通配。
std::vector<QueueKey> wait_keys(uint8_t want, std::optional<uint8_t> echo_cmd);

/// expect() 的返回: 收到的是 ACK / ERR / 目标应答。
struct ExpectResult {
    enum class Kind { Ack, Err, Rsp };
    Kind kind = Kind::Ack;
    std::vector<uint8_t> payload;

    bool is_ack() const { return kind == Kind::Ack; }
    bool is_err() const { return kind == Kind::Err; }
    bool is_rsp() const { return kind == Kind::Rsp; }
};

/// 一条队列项 —— 载荷 + 它的**到达序**(单调递增, 全队列统一)。
/// 到达序是多条 key 上的队列同时有货时挑"最早到达那条"的依据。
struct QueueEntry {
    uint8_t id = 0;
    std::vector<uint8_t> payload;
    uint64_t seq = 0;
};

using Queue = std::deque<QueueEntry>;

/// 每条应答队列的深度上限。真堆到这个数说明主人再也不来取了 (孤儿), 丢最旧的并计
/// dropped —— 与全包对"丢"的一贯口径一致 (不静默)。
inline constexpr size_t kQueueMax = 64;

/// 读线程读到"没有帧"时的退避 (秒)。
/// 不能依赖"传输会阻塞": 那是 SerialTransport 的实现性质, 不是传输接口的契约 ——
/// 测试桩就完全忽略 timeout。不退避的空循环会跑出接近 100% 的 CPU。
inline constexpr double kIdleSleepS = 0.001;

/// 读线程每一拍允许阻塞的时长 (秒)。
inline constexpr double kReadSliceS = 0.1;

class Ack {
public:
    explicit Ack(Arm* arm);
    ~Ack();

    Ack(const Ack&) = delete;
    Ack& operator=(const Ack&) = delete;

    // ---- 读线程侧 ----

    /// 起读线程。唯一调用点是 Arm::connect (时序见那里: Ack 已就绪、而下面那句握手的
    /// expect 必须已经有人读)。
    void start_reader(Transport* tr);

    /// 停读线程。**必须排在 transport->close() 之前**。
    ///
    /// 否则读线程会从已经关掉的传输上抛 TransportError => reader_error 非空 =>
    /// 一次正常关闭被记成"链路丢失"，而 Arm::wait_until_link_lost 正是拿那一格当
    /// "设备没了"的判据。也必须 join: 线程的 target 是成员函数 => 线程引用本对象。
    void stop_reader();

    // ---- 等待者侧 ----

    /// 清掉 cmd 的应答队列 —— 由 Arm::raw_write 在发帧之前调。
    ///
    /// 这是"陈旧帧冒充新应答 => 假成功"的唯一一道闸, 位置是载重的:
    ///   * 必须在写之前 —— 我们的应答只可能在写之后到达, 故清队不可能吃掉自己的应答;
    ///     反过来 (先写后清) 会 100% 吃掉;
    ///   * 必须与写同一个线程、紧挨着 —— 清队与写之间只隔几微秒, 那几微秒里到达的陈旧帧
    ///     是残余风险 (知情的、不可再约: 线上没有请求 id)。
    void drain_for(uint8_t cmd);

    /// 在 keys 这几条队列上等最早到达的那条; 超时返回 nullopt。
    ///
    /// 队列在发帧之前刚被 drain_for 清过 => 这里不需要水位: 队列里出现的东西一定是写
    /// 之后到达的。多条 key 都有货时取 seq 最小的那条 (ACK 与 ERR 可能同时在队, 而
    /// err_waits_for_ack 要求按到达序先看到 ERR)。
    ///
    /// 守卫排在取队列之前 (一个都不能少 —— 漏哪条都是从"响亮地报错"静默退化成
    /// "等满超时"): 终态 -> fork -> 未连接 -> 负 timeout。
    std::optional<QueueEntry> wait(const std::vector<QueueKey>& keys, double timeout);

    /// 等 want 那条应答 —— 从属于本命令的那条队列里等。
    ///
    /// `err_waits_for_ack=true` 时，一条匹配 echo_cmd 的 ERR 不是终局: 记下它, 继续等
    /// ACK{echo_cmd} —— 找到 => 那条 ERR 是别人的, 本命令其实已被受理 (按正常应答返回,
    /// 不抛); 窗口耗尽仍只有 ERR => 那条 ERR 就是本命令的, 抛它。
    ///
    /// 判据是固件对"受理/拒绝"的应答形状不同: 受理必生成一条 RSP_ACK{cmd}, 拒绝只生成
    /// RSP_ERR{cmd,code} (绝不补 ACK)。这条形状只在"生成侧"成立, 到达侧不是: 受理那条
    /// ACK 自己也会丢 (固件应答 FIFO 满时丢最新)。那时窗口里只剩一条 ERR => 本方法判
    /// "被拒"而本命令其实已被受理。
    ///
    /// 代价: 真被拒时要等满整个窗口才返回。所以只有笛卡尔三条入口开这个开关。
    ExpectResult expect(uint8_t want, double timeout, const std::string& cmdlabel,
                        bool raise_on_err = true,
                        std::optional<uint8_t> echo_cmd = std::nullopt,
                        bool err_waits_for_ack = false);

    /// 本会话里 id c 那类帧的 (hz, 最近一帧的本地时间)。
    ///
    /// hz = 自本会话首次收到该类帧起的平均到达频率 (count-1)/(last-first)。分母是实际
    /// 到达的间隔, 故它不随链路空闲而衰减。样本不足 2 条时 hz 写死 0.0。
    /// 本会话从没收到过该类帧时返回 (0.0, 0.0) —— 0.0 是哨兵。
    std::pair<double, double> recv_stats(uint8_t c);

    // ---- 共享状态 (读线程写, 等待者读) ----
    //
    // ⚠ 这些是设计的一部分而不是"暴露内部": Arm 的驱动型读者 (_pump_until / _arrive
    // / poll 系列) 直接看状态单槽与条件变量, 那正是"广播帧不进队列"这条设计的必然结果
    // (100Hz 连续流进队列会在 0.6s 内撑满 kQueueMax 并制造恒定的淘汰噪声)。
    mutable std::mutex mu;
    std::condition_variable cv;

    /// 持 `mu` 的 RAII 锁。
    ///
    /// **Debug 构建下它多带一条"锁纪律"检查**: 登记当前持锁线程, 于是任何碰受保护状态的
    /// 地方都能断言"我确实在锁内" (`dbg_check`)。Release 构建退化成 `std::unique_lock`,
    /// 零额外开销。
    ///
    /// 为什么值得常年留着: `Ack::queues` / `state` / `recv_` 的"必须在 `mu` 之下"这条
    /// 不变量, 全靠**人眼审**那 6 处访问点; 一旦被破坏, 后果是**静默的数据竞争** ——
    /// 最难查的一类缺陷 (TSan 之外的工具有时候也说不清, 见下)。有了它, 破坏这条不变量
    /// 会在**下一次调试构建里当场 abort**, 并指名是哪个站点。
    ///
    /// ⚠ 它不是 TSan 的替代品, 而是"当 TSan 报了这一带、而人眼又看不出问题时, 用来
    /// **一锤定音**的那件工具"。用例见 README 的"TSan 现状"。
    struct MuLock {
#ifndef NDEBUG
        std::unique_lock<std::mutex> lk;
        std::atomic<std::thread::id>* owner;

        MuLock(std::mutex& m, std::atomic<std::thread::id>& o) : lk(m), owner(&o) {
            owner->store(std::this_thread::get_id());
        }
        ~MuLock() { owner->store(std::thread::id{}); }
#else
        std::unique_lock<std::mutex> lk;

        MuLock(std::mutex& m, std::atomic<std::thread::id>&) : lk(m) {}
#endif
        MuLock(const MuLock&) = delete;
        MuLock& operator=(const MuLock&) = delete;
        /// 给 `condition_variable::wait_for` 用 (它要的是 `unique_lock&`)。
        std::unique_lock<std::mutex>& ul() { return lk; }
    };

    /// 持 `mu` 的线程 (仅 Debug 构建维护; Release 下 `dbg_check` 是空操作)。
    std::atomic<std::thread::id> dbg_mu_owner{};

    /// 断言"当前线程持有 `mu`"。仅 Debug 构建生效 —— 见 `MuLock` 的注释。
    void dbg_check(const char* where) const {
#ifndef NDEBUG
        if (dbg_mu_owner.load() != std::this_thread::get_id()) {
            std::fprintf(stderr,
                         "\n[litearm] **锁纪律被破坏**: %s 在未持有 Ack::mu 的情况下访问了"
                         "受保护状态 —— 这是一处静默的数据竞争。\n",
                         where);
            std::fflush(stderr);
            std::abort();
        }
#else
        (void)where;
#endif
    }

    /// 状态单槽 + 它的到达计数。
    /// status_seq 是原子的: 等待者要在**不持锁**的判据里读它 ("这一拍和上一拍是不是
    /// 同一帧"), 而写方在锁内递增。
    std::optional<RobotState> state;
    std::atomic<uint64_t> status_seq{0};

    /// 取状态单槽的副本 (持锁)。state 由读线程在锁内写, 直接读它是数据竞争。
    std::optional<RobotState> state_copy();
    uint64_t status_seq_now() const { return status_seq.load(); }

    /// 读线程的死因 (传输层异常)。非空 = 链路没了。
    /// 正常关闭绝不许写这一格 (顺序见 stop_reader)。
    std::exception_ptr reader_error;
    /// 上面那一格的文本形式 (构造异常消息用, 免去在持锁时 rethrow)。
    std::string reader_error_text;

    /// 投递时协议层抛出的异常 (典型: 笛卡尔收集器的"多了一条")。
    /// 读线程不当场抛、也不死, 存这里由下一个等待者抛出, 抛一次就出队
    /// => 不会 50Hz 刷屏。
    std::deque<std::exception_ptr> errors;

    /// 因队列封顶被挤掉的帧累计条数 —— 只做可观测性, 不改变任何行为。
    /// 它合并了从前的 unexpected_frames 与 foreign_frames —— 两者含义本就相同:
    /// 「没等到主人的帧」。
    uint64_t dropped = 0;

    /// **CRC 对但解码失败**的状态帧累计条数 (只做可观测性)。
    ///
    /// ⚠ 没有它, "链路一直在收帧、但每一帧都解不出来"与"链路根本没数据"在**外面看
    ///   一模一样**(都是状态单槽不更新), 而这两种处境要查的方向完全相反。
    ///   有了它, `host_stats()` 就能把二者分开。
    uint64_t bad_status_frames = 0;

    /// 队列表 —— (上行 id, 回显码) -> 按到达序的队列。
    std::map<QueueKey, Queue> queues;

private:
    void reader_loop(Transport* tr);
    void die(std::exception_ptr e);
    void deliver(uint8_t c, std::vector<uint8_t> payload, double now);
    void note_recv(uint8_t c, double now);
    void on_status(const std::vector<uint8_t>& payload);

    struct RecvEnt {
        uint64_t count = 0;
        double first = 0.0;
        double last = 0.0;
    };

    Arm* arm_ = nullptr;

    /// 时间源 —— 从会话 (`Arm`) 要, **不自己缓存一份**。
    ///
    /// ⚠ 三条理由, 每条都踩过:
    ///   ① `Arm` 是可移动的 ⇒ 缓存引用会在移动后悬空;
    ///   ② `arm_` **可能为空** (可以直接 `Ack a(nullptr)`), 故要空安全;
    ///   ③ 时钟是**每会话**注入的 (`ArmOptions::clock`) ⇒ 它必须跟着会话走,
    ///      不能是 `Ack` 自己的成员 (那样注入的钟根本到不了这里)。
    const Clock& clk() const;
    Transport* tr_ = nullptr;
    /// 建立这条读线程的进程。非属主进程 (fork 出来的子进程) 里绝不可 join ——
    /// 那条线程在子进程里**不存在**, 而 `std::thread::join()` 会永久阻塞、
    /// `~std::thread` 对未 join 的线程会 `std::terminate`。
    int owner_pid_ = -1;
    std::thread reader_;
    std::atomic<bool> stop_{false};
    /// 退避用的独立条件变量 —— 不能被 mu 串住: 退避时若持 mu 会挡住所有等待者。
    std::mutex stop_mu_;
    std::condition_variable stop_cv_;
    uint64_t next_seq_ = 0;
    std::map<uint8_t, RecvEnt> recv_;
};

}  // namespace litearm
