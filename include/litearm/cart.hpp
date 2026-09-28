// 笛卡尔固件规划的应答配对 —— 0x4E 与在途请求的 FIFO 配对。
//
// 固件原生笛卡尔 (0x3A/0x3B/0x3E) 的应答载荷里没有命令 id, 只能按受理顺序 FIFO 配对;
// 而固件那份应答只有一个槽位 (plan_pending 单 bool + 单份载荷): 同一个 main 排空窗口内
// 登记 >=3 条时, 中段请求零应答。于是两条守卫缺一不可:
//
// * 多了一条 分两支。清队销毁在途记账, 于是后来那条 0x4E 归属不明 —— 固件的应答是在
//   规划完成时就推上 USB 的, 而主机清队发生在我们写那条清队 opcode 的那一刻; 两者之间
//   隔着 USB 延迟。所以"清队那一刻固件那条规划已经跑完、应答早躺在主机 RX 缓冲里"是
//   可达的一支, 它属于刚被我们清掉的那条请求。而 0x4E 载荷里没有命令 id, 任何 FIFO
//   方案必有一支判错, 只能选判错得安全的那一支: 宁可报"结局未知", 绝不报"成功":
//     - 清队后的不可归属应答 (吸收额度 > 0) -> 计数 + 吸收, 不报错;
//     - 真·脱同步 (额度耗尽且队列空) -> 计数 + 报错, 不静默丢弃。
// * 少了一条 (token 超时没收尾) -> 摘除该 token 并抛 CartReplyLostError —— 缺了这条,
//   后续的 0x4E 会被错配给一条早已被吞掉的请求。
//
// 串行是强制的 (不是建议), 由 Arm 里的 _cart_serial 锁实现 —— 上面两条守卫都只在串行下
// 成立: _request_and_wait 是先登记、后写, 而配对按受理顺序 FIFO => 没有锁时"两条线程的
// 登记顺序"与"固件的受理顺序"可以倒置 => 两条应答各自配给对方那条 token (0x4E 载荷里
// 没有命令 id, 原理上不可区分) => 假成功。
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "litearm/clock.hpp"
#include "litearm/rot.hpp"

namespace litearm {

class Arm;
class Ack;

/// 会静默作废在途笛卡尔规划的 opcode (固件侧走 cart_invalidate_before_motion ->
/// cart_abort, 一条 0x4E 都不发)。
///
/// 逐条依据 (固件运行路径的调用点): ctrl_accept_move_j (0x01, 且 0x2A home 复用同一处)
/// / kin_runner_request_move_p (0x02) / ctrl_accept_move_js (0x03) /
/// ctrl_accept_move_mit (0x04) / ctrl_accept_move_mit_all (0x05) /
/// ctrl_accept_zero_g (0x06) / ctrl_accept_move_j_sync (0x07) / ctrl_disable (0x11) /
/// ctrl_emergency_stop (0x12) / ctrl_clear_faults (0x13) / ctrl_reset (0x14)。
///
/// 0x3A/0x3B/0x3E 不在此列 —— 它们欠应答, 而且欠两条: 被取代的那条在 PLANNING 期由
/// cart_begin_request 补一条 CANCELED, 新受理那条随后还有自己的结果。清队会让报告与
/// 物理事实相反。
///
/// 这张表只解释"少发"的一半: 突发下 N>=3 的吞应答不经过
/// cart_invalidate_before_motion (是 cart_begin_request 的 unreported 分支吞的),
/// 那半边由 CartPending::wait 的"少了一条"守卫兜住 —— 别以为"没在这张表里 => 必有应答"。
const std::set<uint8_t>& cart_clears_upon();

/// "已收到应答但没人认领"那一列的上界 —— 超出即丢最旧的。
///
/// 为什么必须有界: 调用方放弃 token 的那条路径 (ACK 超时) 每放弃一条就在那一列永久留下
/// 一条 —— 固件其实已受理, 迟到的那条 0x4E 会把它配进去, 而它的调用方再也不会来 wait。
/// 为什么这个界是安全的: 那一列里的结果只能由 Arm::poll_cart 认领, 而它一次只认领一条;
/// 被丢的只是那些请求的规划结果, 它们的调用方早已拿到异常放弃了; 丢弃不碰 wait 的路径
/// => 不制造假成功。
inline constexpr size_t kUnclaimedMax = 8;

/// 吸收额度的存活下限 (秒) —— Arm 传进来的 absorb_ttl 取 max(move_timeout, 本值)。
///
/// 为什么必须有下限: 额度是"清队/放弃那一刻, 那条请求可能还欠一条应答"的通行证。
/// 额度过期而那条应答还在路上时, 它一到就撞上 on_reply: 队列若非空 (清队后又登记了
/// 新请求) 就被配给那条新 token => 调用方拿到别人那条的规划结果并报成功 (假成功);
/// 队列若空则报 LiteArmError "固件与主机已错配" (归因误导)。
///
/// 下限 = 12.0s 约 3 + 8 + 1, 三段各自有固件依据:
///   1. 生成 (<= 写帧 + 3s): RSP_CART_PLAN 由 main 循环每圈调一次,
///      规划期的硬上界是 CART_PLAN_MAX_TICKS = 3s;
///   2. 发射时延 (<= 约 8s): 结果生成 != 已上 USB, 同一圈里排在它前面的
///      usb_cmd_report() 可能正卡在参数保存的阻塞擦写上; 唯一的上界来自
///      hw_watchdog.h —— 擦写前把 IWDG 重装成约 8s, 即"允许多长的停顿才不复位";
///   3. 链路 + 主机侧取帧余量: 约 1s 量级。
///
/// 两个方向都会出错: 取大 => 真·脱同步的应答被静默吸收 (硬错误降级成静默);
/// 取小/过期 => 假成功。两害相权必须偏向取大。
inline constexpr double kAbsorbTtlFloor = 12.0;

/// Arm 要传给 CartPending 的 absorb_ttl —— 两个构造点共用这一处, 免得两处各写一遍 max。
double cart_absorb_ttl(double move_timeout);

/// 固件 cart_err_t (cart_plan.h) 的取值 —— 0x4E 载荷第 2 字节。
///
/// 这是规划层的码, 与 RSP_ERR 第二字节的门禁原因码 (未使能 0x03 / 零重力 0x04 /
/// 掉线锁存 0x06) 共用 1~6 的数值区间却语义完全不同, 别互推。
inline constexpr int CART_ERR_OK = 0;
inline constexpr int CART_ERR_IK = 1;          //: 某路点 IK 无解, 或相邻解跳变超阈值
inline constexpr int CART_ERR_COLLINEAR = 2;   //: move_c 三点共线, 定不出圆
inline constexpr int CART_ERR_TOO_LONG = 3;    //: 路点超容量 —— 整条拒绝, 臂一步没动
inline constexpr int CART_ERR_LIMIT = 4;       //: 路点本身或段中插值越关节限位
inline constexpr int CART_ERR_CANCELED = 5;    //: 被新请求取代 (预期内的接管, 不是故障)
inline constexpr int CART_ERR_BADARG = 6;      //: 入参非法

/// move_c 的起点校验容差 —— 与 Arm::move_p 的默认值同源 (6mm / 0.03rad), 不是抄来的巧合:
/// move_c 校验的就是"实际 TCP 是否在 pose_start 附近", 判据必须与 move_p 的到位判据一致
/// (同一把尺子), 否则 move_p 收工的位置会被 move_c 判成"起点不一致"。
/// 同源关系由 test_cart_protocol.cpp 钉住。
inline constexpr double CART_START_POS_TOL = 0.006;
inline constexpr double CART_START_RPY_TOL = 0.03;

/// 一条笛卡尔命令的规划结果 (固件 RSP_CART_PLAN 0x4E 的载荷)。
///
/// 三个入口 (Arm::move_l / move_c / move_path) 都返回它。ok=false 时入口不返回而是抛
/// 对应异常 (见 raise_for_plan)。
///
/// **started_busy / settled / q_final / settle_err_rad 四个字段由 wait 决定**:
///   wait=true  -> 填真值 (等到状态帧的 CART_BUSY(bit10) 落 0 且 q 静止, 再回读一次 TCP
///                 与目标比对才返回)
///   wait=false -> 一律"未等待"默认值, 只等到规划结果
/// wait=false 时这四个字段不是"没到位", 而是"没等" —— 想知道臂现在什么样, 该
/// get_state() 回读, 而不是拿这份陈旧的规划去推断。
///
/// settled=true 要两条一起成立 (缺一不可): 判到位收的尾, 且到位判据满足之后回读的实际
/// TCP 与本次请求的目标位姿对得上。
///
/// **ok=true 而 settled=false 是一个必须存在的结局** —— 它的含义是"这条笛卡尔轨迹被别的
/// 运动作废了" (锁外/进程外的 movej/home/zero_g: 固件一收到就把这条轨迹作废, 而收尾照样
/// 是 bit10=0 + q 静止), 或者 get_tcp() 取不到。ok 只说明"固件受理并规划出来了", 不说明
/// 臂停在目标上。现场必须回读 get_tcp() 看真实落点。
///
/// settle_err_rad 的名字沿用旧 PC 侧规划器的字段, 但语义已经变了: 从前是"收尾后与终点指令
/// 的最大关节偏差", 而在固件原生路径下该量物理不可得。现在它是"结束时各轴 q 与 q_final
/// 的差", 即判到位那 arrive_frames 帧的抖动幅度 (与终点指令无关)。
struct CartPlan {
    bool ok = false;
    int err = 0;
    int n_wp = 0;
    uint32_t plan_us = 0;
    bool started_busy = false;
    bool settled = false;
    std::vector<double> q_final;
    double settle_err_rad = 0.0;

    /// SDK 自造的 err 档 (不是固件 cart_err_t 的取值): 这条请求的结局未知。
    /// 与 CartReplyLostError 说的是同一件事; 当前没有产出者 (那情形直接抛异常)。
    static constexpr int ERR_REPLY_LOST = -1;

    /// 0x4E 载荷 -> CartPlan。载荷 = ok u8 + err u8 + n_wp u16 LE + plan_us u32 LE (8B)。
    static CartPlan from_reply(const std::vector<uint8_t>& payload);
};

/// ok=false 时按 err 抛对应异常 (ok=true 时什么都不做)。
///
///   err=1 IK                                     -> IKError
///   err=2 COLLINEAR / 3 TOO_LONG / 4 LIMIT       -> CartesianPlanError
///   err=5 CANCELED                               -> MotionSupersededError (不是失败)
///   err=6 BADARG                                 -> InvalidCommandError
///
/// CANCELED 单独成型是刻意的: 接管是正常用法, 混进"规划失败"会让调用方走故障恢复。
/// 反过来 CartesianPlanError 的三档都是"这条轨迹本身不成立, 臂一步没动"。
/// 未登记的 err 值也归到 CartesianPlanError (ok=0 就是"规划没成"), 但消息里带上原始码。
void raise_for_plan(const CartPlan& plan);

/// 一条在途笛卡尔请求的票据 (SDK 自造, 固件不知道它的存在)。
struct CartToken {
    std::mutex mu;
    std::condition_variable cv;
    bool done = false;
    std::optional<std::vector<uint8_t>> reply;   //: 原始 0x4E 载荷
    std::optional<std::string> lost;             //: 非空 = 结局未知, 值即给调用方看的原因
    /// 登记那一刻的 absorbed_replies 水位线 —— 用来判"本条请求的等待窗口里有没有吸收额度
    /// 被消费掉" (见 drop_and_absorb: 消费过就等于固件已经把它那条应答交出来了, 于是不许
    /// 再补额度, 否则额度永 >=1 = 自持)。
    uint64_t absorbed_at = 0;

    void resolve(const std::vector<uint8_t>& payload);
    void fail(const std::string& reason);
};

using CartTokenPtr = std::shared_ptr<CartToken>;

/// 在途笛卡尔请求的 FIFO 队列 + 0x4E 收集器。
///
/// 三种角色落在同一个对象上 (它们必须共享同一份队列状态):
///   * register_/request/drop —— 记账 (发出命令的入口用);
///   * on_reply —— 收集器: 读线程认领到 0x4E 就交给它;
///   * clear_pending —— 清队: 被 cart_clears_upon() 里的命令作废时调用, 并把被清掉的
///     条数记成吸收额度。
class CartPending {
public:
    /// absorb_ttl 是吸收额度的存活时间, 由 Arm 传 max(move_timeout, kAbsorbTtlFloor)。
    /// absorb 是建对象时就带进来的额度 —— 只给 Arm::connect 那一处用。
    explicit CartPending(double absorb_ttl, int absorb = 0, Arm* arm = nullptr);
    ~CartPending();

    /// 登记一条在途请求。**必须在写之前调** —— 否则窗口内到来的 0x4E 会落空。
    CartTokenPtr register_token();

    /// 登记 token 并立刻发帧 —— 登记与写在同一个 try 内。
    ///
    /// **write 只许包含"写"** —— 校验必须由调用方放在 request() 之外: 校验失败根本不该
    /// 登记 token (登记了又要摘, 就多出一段"这个异常到底发生在写之前还是写之后"的推断)。
    ///
    /// 写失败 (TransportError) 才摘除自己那个 token 并原样抛出。两个方向别记反:
    ///   * 滞留一条未送达的 token => 在途比应答多一条 => 应答顺次前移一格 => 安全;
    ///   * 丢弃一条其实已送达的 token => 在途比应答少一条 => 假成功, 危险。
    /// **其余任何异常一律 re-raise 且不动队列** —— 它们证明不了帧没送达
    /// (最现实的一支是 KeyboardInterrupt 落在 flush 里: 那时整帧已在驱动里会被送达)。
    CartTokenPtr request(const std::function<void()>& write);

    /// 摘除指定的那条 token (按对象身份, 不是队尾 —— 并发下队尾是别人的)。
    void drop(const CartTokenPtr& token);

    /// 摘除 token, 并留一格吸收额度 —— 摘 token 与给额度必须在同一把锁内。
    ///
    /// timed_out 说明为什么要放弃这条请求: 超时支传 true、读链路炸支传 false。
    /// 它只决定下面那个"例外段"生不生效, 不影响"摘 token"这一半。
    ///
    /// 例外 (唯一一条, 只对超时支生效): timed_out 且本条请求的窗口里已经消费过额度 =>
    /// 不补。吸收先于配对, 所以一条在途 token 的应答到了、而额度还有存量时, 被吃掉的
    /// 就是它自己那条 —— 那条应答已经交出来了, 固件不再欠它什么。此时再补一格, 补出来的
    /// 那格会去吃下一条命令的应答, 那条又超时、又补一格 ... 自持。
    ///
    /// pump 支根本没等过窗口 (读链路当场就炸了), 这条前提在它身上没有依据 => 它照旧补一格。
    void drop_and_absorb(const CartTokenPtr& token, bool timed_out);

    /// 非阻塞认领: 队首 token 若已收尾就摘掉并返回它, 否则 nullopt。
    ///
    /// FIFO 配对下队首就是"最早那条" —— 它被解答之前, 后面的 token 不可能被解答,
    /// 所以只看队首是对的。用途只有 Arm::poll_cart()。
    /// 判据必须含 done, 不能只看"那一列非空": on_reply 是先登记进那一列、后 resolve,
    /// 两者之间有一条缝 —— 并发读者落进缝里就会拿到一个还没收尾的 token。
    CartTokenPtr claim_resolved();

    /// 收集器 —— 读线程见到 0x4E 时调它。
    ///
    /// **先消费吸收额度, 再配对** —— 顺序是决定性的: 清队后立刻登记的新 token 会让队列
    /// 非空, 而那条清队前就已到达的迟到应答若被拿去配对, 就配错了 (报成功而实际没跑完,
    /// 即假成功)。故额度那一步必须排在"队列空/非空"判断之前。
    ///
    /// 队列空且额度为 0 才是真 "多了一条": 计数并报错 (抛基类 LiteArmError —— 那是 SDK
    /// 的配对模型与固件脱同步, 内部不变量被破坏, 不该被专门 catch), 不静默丢弃。
    void on_reply(const std::vector<uint8_t>& payload);

    /// 清队 —— 由 Arm::raw_write 在发出清队 opcode 之前调用。返回摘除条数。
    ///
    /// 这几条请求在固件侧已经静默作废, 永远等不到 0x4E 了: 它们被标成"结局未知"并唤醒
    /// 等待者。**记账被销毁了, 但被清掉的条数留下一份额度**: 清队那一刻固件那条规划可能
    /// 已经跑完、应答早躺在主机 RX 缓冲里。
    /// n == 0 时两个字段都不动 —— 这不是优化, 是正确性: 零重力保活线程每 40ms 发一条清队
    /// opcode (0x06, 而它几乎永远是空清队), 若空清队也刷新截止, 额度就永远不过期, 此后
    /// 一条真脱同步的应答会被永久静默吞掉。额度跨多次清队累加。
    int clear_pending(const std::string& reason = "");

    /// 等这条 token 收尾; 超时按"少了一条"处理。返回原始 0x4E 载荷。
    ///
    /// 收尾不成功一律抛 CartReplyLostError (不是 MotionTimeoutError: 这里要表达的是
    /// "未知结局", 混进通用超时会让调用方按"没生效"去重发)。
    /// 两条放弃路径 (超时 / 读线程死) 摘 token 时都留一格吸收额度。
    /// 读线程死了要当场响亮: 等满 timeout 才报"结局未知"会把"链路断了"说成"固件没回"。
    std::vector<uint8_t> wait(const CartTokenPtr& token, double timeout);

    /// 在途条数。
    int pending();

    /// 因越界被丢弃的"待认领"条数 (累计) —— 丢弃不报错, 故至少要看得见。
    uint64_t evicted_unclaimed = 0;
    /// 队列空却收到 0x4E 的累计条数 —— "多了一条"的可观测面。
    uint64_t extra_replies = 0;
    /// 被吸收掉的不可归属应答的累计条数 —— 与 extra_replies 并列的另一个可观测面。
    uint64_t absorbed_replies = 0;

private:
    /// 时间源 —— 从会话 (`Arm`) 要, **不自己缓存一份** (理由与 `Ack::clk()` 逐条相同:
    /// `Arm` 可移动、`arm_` 可为空、时钟是每会话注入的)。
    const Clock& clk() const;

    void unclaim_locked(const CartTokenPtr& token);
    int absorbance_locked(double now);
    void unclaim(const CartTokenPtr& token);
    Arm* arm_of() const { return arm_; }

    std::mutex lock_;
    Arm* arm_ = nullptr;
    std::deque<CartTokenPtr> q_;
    std::deque<CartTokenPtr> unclaimed_;
    double absorb_ttl_;
    int absorb_ = 0;
    double absorb_deadline_ = 0.0;
};

// ---------------------------------------------------------------------------
// 三条入口的实现 (由 Arm 的同名方法转发)
// ---------------------------------------------------------------------------

/// 能力探测: 固件是否支持笛卡尔 (受 #if LITEARM_CART_PLAN 编译开关约束)。
///
/// **只发空载荷的 0x3A**: 0x3A~0x3E 里没有只读命令 —— 发一条合法载荷就是"connect 之后臂
/// 自己动一下"。空载荷撞的是固件的长度校验, 它排在 cart_gate_ok 与一切副作用之前, 于是:
///   有笛卡尔 (开关 ON)  -> ERR{0x3A,0x01} (长度不足) => 能力在
///   无笛卡尔 (开关 OFF) -> ERR{0x3A,0x00} (default 分支) => 能力不在
///
/// **"一片安静" != "不支持" —— 故本探测有界重试**: 固件的 default 分支保证任何固件都会
/// 对 0x3A 回一条 ERR, 所以读超时只可能是这一帧丢了 (本硬件的 USB CDC 上行有已知丢帧),
/// 而不是"固件没有这条命令"; 而本函数的结果在 connect() 里缓存整个会话。
/// 两种回应的处置不同: ERR{0x3A,0x00} 是确定结论, 立刻返回 false 不重试; 读超时则重试,
/// 仍安静才返回 false (fail-closed), 并把这一情形记进 Arm 的 _cart_probe_silent。
bool probe(Arm* arm);

CartPlan move_l(Arm* arm, const rot::PoseInput& pose, double speed = 1.0,
                bool wait = true);
CartPlan move_c(Arm* arm, const rot::PoseInput& pose_start,
                const rot::PoseInput& pose_via, const rot::PoseInput& pose_goal,
                double speed = 1.0, bool wait = true);
CartPlan move_path(Arm* arm, const std::vector<rot::PoseInput>& poses,
                   double speed = 1.0, bool wait = true);

}  // namespace litearm
