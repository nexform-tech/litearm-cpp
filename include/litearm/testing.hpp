// 脚本化应答的假传输 —— 让 Arm 全流程可离线测试 (镜像 testing.py 的桩硬件风格)。
//
// 按收到的下行命令推送预设帧 (ACK/状态/RSP_TCP/RSP_IK), 实现 Transport 接口, 供测试注入
// ArmOptions::transport_factory 用。
//
// ⚠ 桩的每个字段都是"为了复现固件的某个具体形状"而存在的, 别按"看起来用不到"删掉 ——
// 每一条都至少钉着一个判据 (见 src/testing.cpp 里逐条的注释)。
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "litearm/clock.hpp"
#include "litearm/protocol.hpp"
#include "litearm/transport.hpp"

namespace litearm {
namespace testing {

/// 可**手工推进**的假钟。
///
/// ⚠⚠ 用法有一条硬前提, 先读 `clock.hpp` 开头那段: 本仓的时钟只管"现在几点",
///   **不管怎么等** ⇒ 假钟**不会自己前进**, 于是凡"等超时真的发生"的判据会**挂死**
///   (不是失败)。它只该用在**纯比较**类判据上: 到期比较、时间戳、TTL 水位。
///
///   能用的典型场合 —— 而且这是它最大的价值: **"多久没收到帧算失联"** 这类判据
///   本来要真睡满 `STATUS_STALE_MAX_S` (2.0s) 才验得到, 用本类 `advance()` 是**瞬间**的。
class FakeClock : public Clock {
public:
    /// 起始值**刻意非 0**。
    ///
    /// ⚠ 本仓多处用 `timestamp <= 0.0` 表示"**从没收到过**"(如 `Arm::is_connected` 里
    ///   `recv_stats(...).second <= 0.0` 那一句)。假钟若从 0 起, 那类判据会在
    ///   "明明收到过"时判假 —— 于是用例测的是一个**现实中不存在**的状态。
    static constexpr double kStartS = 1000.0;

    static std::shared_ptr<FakeClock> make();

    double now_s() const override;
    /// 手工推进 `s` 秒。⚠ 契约只要求"单调不减", **推进是用例的责任** ——
    ///   本类不会因为有人等它就自己走。
    void advance(double s);

private:
    mutable std::mutex mu_;
    double now_ = kStartS;
};

}  // namespace testing
}  // namespace litearm

namespace litearm {
namespace testing {

/// 固件 usb_cmd.c 对 5 条笛卡尔命令的最小载荷长度校验 (0x3A 28 / 0x3B 52 / 0x3C 5 /
/// 0x3D 25 / 0x3E 0)。固件把长度校验排在门禁与一切副作用之前, 桩照同一顺序 ——
/// 于是空载荷只会得到 ERR{cmd,0x01} 而不会让臂动, SDK 的能力探测正是靠这一点。
extern const std::map<uint8_t, size_t> CART_MIN_LEN;

/// KIN_BENCH 回文本 —— 逐字节镜像固件 kin_runner.c 的组装结果, 不是"看着像"。
/// 关键在 u32_cat() 的语义: 数字先写 digits 再补一个尾随空格, 而 txt() 原样拼接 (不加
/// 分隔符)。于是:
///   FK   + 200 + 108 + 240  ->  "FK200 108 240 "
///   LOOP1 + 4000            ->  "LOOP14000 "     <- 名字与数字相连!
/// 最后一行 LINK 是拿到 CRC 坏帧 / 应答 FIFO 丢弃 / CAN TX 失败分类桶等诊断计数的唯一通道。
extern const char* const KIN_BENCH_TEXT;

/// 把 KIN_BENCH 全文按真机那样拆帧 (第 1 帧耗时 / 第 2 帧 LINK 行)。
/// 不含 LINK 的文本仍回一帧 —— 空文本要走 parse_kin_bench 的"空回执 = 异常"判据。
std::vector<std::string> split_kin_bench(const std::string& text, bool one_frame = false);

/// 造 n_ticks 拍固件 log_sample_t 的字节流: u32 tick + q_ref[N] + dq[N] + tau[N]。
std::vector<uint8_t> make_log(int n_ticks, int n);

/// 造一帧**固件 1.5.x 真实布局** (6+21N) 的状态帧 —— 尾部带 joint_fault u16。
std::vector<uint8_t> make_status(const std::vector<double>& q,
                                 const std::vector<double>& dq, int mode, uint16_t flags,
                                 uint16_t seq, uint16_t joint_fault, int n);

class FakeTransport : public Transport {
public:
    explicit FakeTransport(const std::string& port = "fake", double timeout = 0.2,
                           const std::string& fw = "Litearm1.7.0-7J", int n = 7);

    // ---- Transport 接口 ----
    using Transport::write_frame;   // 别让下面那条 override 把基类的便捷重载藏掉
    void write_frame(uint8_t cmd, const uint8_t* payload, size_t len) override;
    std::optional<proto::Frame> read_frame(double timeout) override;
    void close() override;
    bool is_open() const override;
    std::string port_name() const override { return port; }
    std::string text_log() const override;

    /// 注入"噪声文本" —— 模拟真传输把**非帧字节**留下的痕迹 (固件开机横幅就在里面)。
    /// ⚠ 走本口注入 (持 `state_lock`), 别直接写 `noise_text`: 它被读线程经 `text_log()` 读。
    void set_text_log(std::string s);
    /// 注入 flush (tcdrain) 失败计数 —— 模拟"写出去了但没保证出得去"。
    void set_flush_failures(uint64_t n);

    /// **注帧口**: 往响应队列里塞一帧 (不需要先有一条下行命令)。
    void push_frame(uint8_t cmd, const std::vector<uint8_t>& payload = {});

    /// 某命令各次下行的时刻 (与 tx_log 同下标)。
    std::vector<double> stamps_of(uint8_t cmd) const;

    /// 下行流水账的**一致快照** (持 `tx_lock`) —— 读 `tx_log` 一律走这里。
    ///
    /// ⚠⚠ 别裸遍历 `tx_log`: 零重力保活线程会**并发** `emplace_back`, 而 `std::vector`
    ///   扩容会把正在被遍历的缓冲区**释放掉** —— 那是 use-after-free, 不是"读到旧值"
    ///   那种无害竞争。实测: TSan 报 `data race ... tests/test_zero_g.cpp:18 in
    ///   count_zero_g_on` (2026-09-28), 而这类读者曾有 **10 个测试文件 / 36 处**。
    ///   `stamps_of()` 一直是持锁的, 只有裸读 `tx_log` 的那些漏了 —— 照它写。
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> tx_snapshot() const;

    /// 当前下行条数 (持 `tx_lock`)。只关心"有没有/多了几条"时用它, 比快照便宜。
    size_t tx_count() const;

    /// 固件 ctrl_is_armed() 的桩侧等价物 (= enabled || enable_pending)。
    /// 五处门禁 (0x25/0x36/0x32/0x37/0x23) 与 0x15 共用这一个判据 —— 判据只有这一份,
    /// 别在任何一处退回 self.enabled。
    bool armed() const { return enabled || dfu_armed_pending; }

    // ---- 固件状态 (可注入) ----
    std::string port;
    double timeout;
    std::string fw;
    int n;
    std::map<int, std::vector<double>> ff_vec;              // item -> 7 值 (供 0x2B 读回)
    std::map<std::pair<int, int>, double> ff_scalar;        // (item, sub) -> 值
    uint32_t ff_mask = 0;                                   // item 9 只读回填
    /// 当前(假)关节位形 / TCP。
    /// ⚠ 这两个**只由写线程改、由读线程读** (读线程要用它们合成状态帧与 `0x48` 应答),
    /// 裸写就是数据竞争 ⇒ 用 `set_q()` / `set_pose()` (持 `state_lock`)。
    std::vector<double> q;
    std::vector<double> pose;
    void set_q(std::vector<double> v);
    void set_pose(std::vector<double> v);
    /// 把**全部轴**的软限位设成 `[q_min, q_max]` (持 `state_lock`)。
    ///
    /// ⚠ 必须走本口, 别直接改 `joint_params`: 那个表是**构造期**由 `jp_*` 建好的,
    ///   构造之后再改 `jp_q_min` / `jp_q_max` **没有任何效果**(实测: 预检用例里
    ///   改完仍报 "越软限 [-3, 3]"); 而直接改 `joint_params` 又会与读线程的
    ///   `write_frame` 抢 (它整段持 `state_lock` 读这张表)。
    /// ⚠ 也就**只能**在 `connect()`(会读一次 `0x24`) **之前**调 —— 软限缓存是连接期
    ///   建一次的, 连上之后再改桩, 宿主机那份缓存不会跟着变。
    void set_joint_limits(double q_min, double q_max);
    /// 缓冲空时是否补一拍 idle 状态帧。
    ///
    /// ⚠ **原子**, 理由与 `status_seq` 同款: 读线程在 `read_frame` 里读它, 而用例要从
    /// **主线程**关掉它 (用例想先把链路静下来时) —— 裸 `bool` 就是数据竞争。
    /// ⚠ 用例**别直接**用它, 用 `lt::quiesce()` (tests/test_support.hpp): "关掉被动流"
    /// 只是静链路的**一半**, 另一半是"等在途帧落定", 只做一半的用例仍然是偶发红。
    std::atomic<bool> auto_status{false};
    /// 合成状态帧的最小间隔 (秒)。
    /// ⚠ 没有它, read_frame 会无限快地造状态帧 —— SDK 有读线程, 它会以桩能供上的最高速率
    /// 空转, 整套用例被拖垮。取 1ms (桩可以比真机快, 但不能无界)。
    /// ⚠ 同 `auto_status`: 读线程读、用例可能写 ⇒ 要改它之前先把它也改成原子。
    double auto_status_period = 0.001;
    bool closed = false;
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> tx_log;
    std::vector<double> tx_stamps;

    // 注入: 第 N 次 CMD_ZERO_G 之后抛 TransportError (模拟 CDC 掉线)
    int zg_fail_after = -1;
    /// ⚠ 原子: 保活线程自己加, 用例从主线程读。
    std::atomic<int> zg_writes{0};
    /// >0 时每次保活写 (0x06 on=1) 卡这么久 —— 模拟 CDC 写阻塞
    double zg_slow_write_s = 0.0;
    /// 固件"没有实现"的命令 —— 按固件 default 分支回 ERR{cmd,0x00}
    std::set<uint8_t> unknown_cmds;
    /// 强制某命令回指定错误码 (模拟 ERR{cmd,code})
    std::map<uint8_t, uint8_t> err_override;

    // [2026-09-14] 动力学模型在线导入。
    // 模型恒 9 刚体 —— 与 self.n (关节数) 无关, 台架 1J 下也是 9。
    int model_nbody = 9;
    std::vector<std::vector<double>> model_body;
    std::vector<double> model_jm;
    std::map<int, std::vector<double>> model_stage_body;
    std::optional<std::vector<double>> model_stage_jm;
    bool model_override = false;
    bool model_dirty = false;

    // 关节级参数表 (0x22/0x23 写, 0x24 读, 0x36 回出厂默认)。
    // ⚠ 原子: 读线程在 `stamp_status` 里把它盖成状态帧的 bit9, 而用例从主线程改它
    //   (`off.t().enabled = true` 这种写法保持可用)。
    std::atomic<bool> enabled{false};

    // [授权] 是否已激活。默认 true —— 否则每一条既有用例的 enable() 都会撞
    // ERR{0x10,0x08} (固件 ctrl_enable 的第一条判据), 而那些用例与授权无关。
    bool activated = true;
    int license_ver = 1;
    std::vector<uint8_t> license_uid;
    uint32_t license_cust_id = 0;
    uint32_t license_issued = 0;
    uint32_t license_flags = 0;
    /// 注入: 下一次 0x3F 回 ERR{0x3F,0x02} (模拟 MAC 不符 / 密钥非法 / 写失败 ——
    /// 它们全折成这一档)。桩不可能真的验 MAC —— 它没有也不需要密钥 (规格硬要求)。
    bool license_fail_next = false;

    double jp_kp = 50.0, jp_kd = 2.0, jp_tau_max = 10.0, jp_q_min = -3.0, jp_q_max = 3.0;
    std::vector<std::map<std::string, double>> joint_params;

    // 300Hz 采集
    int log_max = 2400;
    std::vector<uint8_t> log_bytes;
    bool log_active = false;
    /// 模拟 USB 掉帧: 下一次 CMD_LOG_READ 不回 (读者须按游标续读/重试)
    bool log_read_fail_once = false;
    /// CMD_LOG_READ 回一段**过短的载荷** —— 用于造"应答帧短"那条分支。
    /// ⚠ 确定注入点, 别在用例里手工 push_frame (会与读线程抢时序)。
    bool log_reply_short = false;
    /// >0 时模拟固件的逐拍记录 (300Hz 语义): 启动后按"每拍 1/log_hz 秒"推进,
    /// 与读回次数无关 —— 这是必须的真实模型: start() 之后立刻读回只能拿到一个前缀,
    /// 等待必须靠轮询已记录量。
    double log_hz = 0.0;
    double log_t0 = -1.0;
    int log_target = 0;
    int log_recorded_ticks = -1;
    /// 模拟固件回一个不前进的游标 (next_byte == offset) —— 读者必须自己判定"无进展"并报错。
    bool log_cursor_stuck = false;
    int log_read_calls = 0;

    // KIN_BENCH
    std::string kin_bench_text;
    /// 模拟旧固件: 只发第 1 帧 (耗时帧), 不发 LINK 诊断帧。
    bool kin_bench_one_frame = false;

    // 覆盖 GET_TCP 回读的位置/朝向 (模拟固件的 rpy 规范化 —— 含万向锁强制 yaw=0)
    std::optional<std::vector<double>> pos_override;
    std::optional<std::vector<double>> rpy_override;
    /// 非空时 `0x43` 原样回这段载荷 (而不是打包 6 个 f32) —— 用于造"帧长不足"那条分支。
    /// ⚠ 用**桩的确定注入点**, 不要在用例里手工 push_frame: 那样造出来的帧会与读线程抢
    /// 时序 (它可能先被读进队列, 再由 `drain_for` 清掉), 用例变成偶发红。
    std::optional<std::vector<uint8_t>> tcp_payload_override;
    /// `0x42` 应答里那个 ok 字节 (0 = 目标不可达/IK 失败)。
    int ik_ok = 1;
    /// >= 0 时 `0x34` 回显这个 body_idx 而不是被请求的那个 —— 用于造"帧布局漂移"。
    int model_body_echo_idx = -1;

    // ---- [笛卡尔] 0x3A~0x3E + RSP_CART_PLAN ----
    /// 固件的 #if LITEARM_CART_PLAN 编译开关。false = 这 5 条整段不在固件里。
    bool cart_supported = true;
    int cart_n_wp = 12;
    uint32_t cart_plan_us = 4700;
    /// 非空时 0x4E 回 ok=0 + err=该值 —— 造规划失败 (cart_err_t 1..6)。
    std::optional<int> cart_err_override;
    /// 交付给 SDK 的状态帧计数 (自最后一条 0x4E 起) —— 驱动 CART_BUSY (bit10)。
    /// ⚠ 读线程在 `stamp_status` 里读它 ⇒ 同样只能经下面这两个口改。
    std::optional<int> cart_busy_seq;
    void set_cart_busy(int seq);
    void clear_cart_busy();
    /// **交付给 SDK 的状态帧序号** —— 每交付一帧 +1 (u16 回绕)。
    /// 到位判据的新鲜度闸靠它判"这一帧是不是命令之后生成的" => 桩必须逐帧递增。
    /// 原子: 读线程在 `stamp_status` 里递增它, 而用例可能从主线程读它对账。
    std::atomic<uint16_t> status_seq{0};

    // [DFU] 0x15: 只登记, 不在命令处理里跳
    bool dfu_armed_pending = false;
    bool dfu_rom_table_invalid = false;
    /// 登记成功后设备是否真的离开 CDC。true = 真跳转 (ACK 之后读路径抛 TransportError);
    /// false = 两条静默撤销路径的形状 (设备还在 CDC 上, 而 ACK 是成功的)。
    bool dfu_vanishes = true;
    bool dfu_gone = false;
    /// 登记 ACK 之后、设备离开之前还会继续投递的帧。
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> dfu_post_ack_frames;

    /// 当前(假)路径的最后一个路点 —— 0x3E (RUN) 的目标就是它。
    std::vector<double> path_last_pose;

    /// 下行流水账的锁 (`tx_log` / `tx_stamps`)。
    mutable std::mutex tx_lock;

    /// **桩的状态锁。**
    ///
    /// ⚠ 这不是可选的: SDK 的读线程在**并发**调 `read_frame`, 而测试线程在调 `write_frame`
    /// —— 两者都要动 `resp_` (读线程 `pop_front`、写线程 `push_back`) 与 `q`/`pose` 等字段。
    /// Python 版的桩不需要这把锁 (GIL 让单个容器操作原子), C++ 里没有它就是数据竞争:
    /// 实测 (ASan) 会以 **SEGV on `resp_.pop_front()`** 的形态偶发崩在
    /// `FakeTransport::read_frame` 里。
    /// 锁的取用点: `write_frame` / `read_frame` / `push_frame`, 三者互不嵌套。
    mutable std::mutex state_lock;

private:
    int log_recorded() const;
    bool axis_outside(int idx, double qmin, double qmax) const;
    void cart_cmd(uint8_t cmd, const std::vector<uint8_t>& payload);
    std::vector<double> cart_goal(uint8_t cmd, const std::vector<uint8_t>& payload) const;
    std::vector<uint8_t> cart_plan_frame() const;
    std::optional<proto::Frame> stamp_status(std::optional<proto::Frame> fr);
    void push(const std::vector<uint8_t>& frame);

    std::deque<std::vector<uint8_t>> resp_;
    double auto_status_last_ = -1.0;
    /// 噪声文本 (见 `set_text_log`)。读线程经 `text_log()` 读, 故写入持 `state_lock`。
    std::string noise_text;
};

}  // namespace testing
}  // namespace litearm
