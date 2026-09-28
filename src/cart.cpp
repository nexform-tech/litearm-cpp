#include "litearm/cart.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "litearm/ack.hpp"
#include "litearm/arm.hpp"

namespace litearm {

const std::set<uint8_t>& cart_clears_upon() {
    static const std::set<uint8_t> kSet = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,   // move_j/P/js/mit/mit_all/zero_g/j_sync
        0x11, 0x12, 0x13, 0x14, 0x2A,               // disable/estop/clear_faults/reset/home
    };
    return kSet;
}

double cart_absorb_ttl(double move_timeout) {
    return std::max(move_timeout, kAbsorbTtlFloor);
}

// ===========================================================================
// CartPlan / raise_for_plan
// ===========================================================================

CartPlan CartPlan::from_reply(const std::vector<uint8_t>& payload) {
    if (payload.size() < 8) {
        throw TransportError("RSP_CART_PLAN 帧短: " + std::to_string(payload.size()) +
                             "B (应 8B) —— 帧布局漂移?");
    }
    CartPlan p;
    p.ok = payload[0] != 0;
    p.err = payload[1];
    p.n_wp = proto::read_u16le(payload, 2);
    p.plan_us = proto::read_u32le(payload, 4);
    return p;
}

void raise_for_plan(const CartPlan& plan) {
    if (plan.ok) return;
    const int err = plan.err;
    if (err == CART_ERR_IK) {
        throw IKError("固件笛卡尔规划失败: 路点 IK 无解或解跳变 (err=" +
                      std::to_string(err) + ")");
    }
    if (err == CART_ERR_CANCELED) {
        throw MotionSupersededError(
            "这条笛卡尔请求被固件的另一条请求取代 (err=5 CANCELED) —— "
            "预期内的接管, 不是失败");
    }
    if (err == CART_ERR_BADARG) {
        throw InvalidCommandError("固件拒绝这条笛卡尔命令: 入参非法 (err=" +
                                  std::to_string(err) + ")");
    }
    std::string detail;
    switch (err) {
        case CART_ERR_COLLINEAR: detail = "三点共线, 定不出圆 (move_c)"; break;
        case CART_ERR_TOO_LONG: detail = "路点超容量 —— 整条拒绝, 臂一步没动"; break;
        case CART_ERR_LIMIT: detail = "路点或段中插值越关节限位"; break;
        default: detail = "未登记的 err=" + std::to_string(err); break;
    }
    throw CartesianPlanError("固件笛卡尔规划失败: " + detail + " (err=" +
                             std::to_string(err) + ")");
}

// ===========================================================================
// CartToken
// ===========================================================================

void CartToken::resolve(const std::vector<uint8_t>& payload) {
    {
        std::lock_guard<std::mutex> lk(mu);
        reply = payload;
        done = true;
    }
    cv.notify_all();
}

void CartToken::fail(const std::string& reason) {
    {
        std::lock_guard<std::mutex> lk(mu);
        lost = reason;
        done = true;
    }
    cv.notify_all();
}

// ===========================================================================
// CartPending
// ===========================================================================

const Clock& CartPending::clk() const {
    return arm_ != nullptr ? arm_->clock() : steady_clock_instance();
}

CartPending::CartPending(double absorb_ttl, int absorb, Arm* arm)
    : arm_(arm), absorb_ttl_(absorb_ttl), absorb_(absorb) {
    // 带进来的额度从建对象这一刻起算 absorb_ttl (与 clear_pending / drop_and_absorb 给
    // 额度时的口径一致: 额度总是"从现在起再活 absorb_ttl")。
    absorb_deadline_ = absorb ? (clk().now_s() + absorb_ttl) : 0.0;
}

CartPending::~CartPending() = default;

CartTokenPtr CartPending::register_token() {
    // 必须在写之前调 —— 否则窗口内到来的 0x4E 会落空。
    auto tok = std::make_shared<CartToken>();
    std::lock_guard<std::mutex> lk(lock_);
    tok->absorbed_at = absorbed_replies;   // 水位线: 见 drop_and_absorb
    q_.push_back(tok);
    return tok;
}

CartTokenPtr CartPending::request(const std::function<void()>& write) {
    auto tok = register_token();
    try {
        write();
    } catch (const TransportError&) {
        // 契约: write_frame 抛 TransportError => 整帧未送达 (残缺帧被固件按 CRC 丢)
        // => 摘下自己那个 token 才是干净的 (在途条数 <-> 应答条数重新相等)。
        drop(tok);
        throw;
    } catch (...) {
        // 其余异常证明不了帧没送达 (最现实的是中断落在 flush 之后, 那时整帧已在驱动里)
        // => 留在队里: 最坏是后续请求报"结局未知" (安全方向), 而摘掉它会让那条应答配给
        // 后面的活 token = 假成功 (危险方向)。
        throw;
    }
    return tok;
}

void CartPending::drop(const CartTokenPtr& token) {
    // 摘除指定的那条 token (按对象身份, 不是队尾 —— 并发下队尾是别人的)。
    // 两个地方都要摘: 在途队列与"已收到应答但没人认领"那一列 —— 一条被判死的请求不该
    // 还能被 poll_cart 认领出结果。
    std::lock_guard<std::mutex> lk(lock_);
    unclaim_locked(token);
}

void CartPending::unclaim_locked(const CartTokenPtr& token) {
    for (auto it = q_.begin(); it != q_.end(); ++it) {
        if (*it == token) {
            q_.erase(it);
            break;
        }
    }
    for (auto it = unclaimed_.begin(); it != unclaimed_.end(); ++it) {
        if (*it == token) {
            unclaimed_.erase(it);
            break;
        }
    }
}

void CartPending::drop_and_absorb(const CartTokenPtr& token, bool timed_out) {
    // 与 clear_pending 同构: 摘掉 token 的那一刻, 固件那条应答可能已经在路上 (它受理了
    // 这条命令, 只是我们等不到/读不到)。不留额度的话, 它一到就撞上"队列空"那条判据
    // => LiteArmError 从毫不相干的读路径炸出来。
    //
    // 例外 (唯一一条, 只对超时支生效): 窗口里已经消费过额度 => 不补。见头文件。
    std::lock_guard<std::mutex> lk(lock_);
    unclaim_locked(token);
    if (timed_out && absorbed_replies != token->absorbed_at) {
        // 超时支 + 窗口里已经吸收过一条 —— 那条就是本条请求的应答: 固件不再欠, 故不补额度
        // (补了就是自持)。两个字段都不动。
        // pump 支不走这里: 它没等过窗口, "被吃掉的是自己那条"没有依据。
        return;
    }
    const double now = clk().now_s();
    absorb_ = absorbance_locked(now) + 1;
    absorb_deadline_ = now + absorb_ttl_;
}

CartTokenPtr CartPending::claim_resolved() {
    std::lock_guard<std::mutex> lk(lock_);
    if (!unclaimed_.empty() && unclaimed_.front()->done) {
        auto tok = unclaimed_.front();
        unclaimed_.pop_front();
        return tok;
    }
    return nullptr;
}

void CartPending::on_reply(const std::vector<uint8_t>& payload) {
    CartTokenPtr tok;
    {
        std::lock_guard<std::mutex> lk(lock_);
        if (absorbance_locked(clk().now_s()) > 0) {
            // 清队后的不可归属应答 —— 属于刚被清掉的那条请求, 与"真·脱同步"是两回事:
            // 消费一格额度, 计数, 直接返回 (不配对、不报错、队列不动)。
            absorb_ -= 1;
            absorbed_replies += 1;
            return;
        }
        if (q_.empty()) {
            extra_replies += 1;
            // "多了一条"抛基类 LiteArmError 而不给它一个专门的异常类型: 那是 SDK 的配对
            // 模型与固件脱同步 (内部不变量被破坏), 没有任何调用方能据它做出正确决定, 也
            // 不该被专门 catch —— 给它一个专用类型反而在邀请后续代码去 except 它。
            throw LiteArmError(
                "收到 0x4E (笛卡尔规划应答) 但队列为空 —— 应答比请求多, "
                "固件与主机已错配 (已计数 extra_replies=" +
                std::to_string(extra_replies) + "); 这不是可以忽略的噪声");
        }
        tok = q_.front();
        q_.pop_front();
        // 先登记"待认领"再唤醒等待者: wait 醒来后会把它从这一列移走, 顺序反过来的话那次
        // 移除会扑空, 于是同一条结果被 poll_cart 再交付一次。
        unclaimed_.push_back(tok);
        // 给那一列封顶: 无界的唯一现实来源是"调用方放弃 token" (ACK 超时路径)。
        while (unclaimed_.size() > kUnclaimedMax) {
            unclaimed_.pop_front();
            evicted_unclaimed += 1;
        }
    }
    tok->resolve(payload);
}

int CartPending::clear_pending(const std::string& reason) {
    std::deque<CartTokenPtr> tokens;
    int n = 0;
    {
        std::lock_guard<std::mutex> lk(lock_);
        tokens.swap(q_);
        n = int(tokens.size());
        if (n) {
            const double now = clk().now_s();
            absorb_ = absorbance_locked(now) + n;
            absorb_deadline_ = now + absorb_ttl_;
        }
        // n == 0 时两个字段都不动 —— 这不是优化, 是正确性: 零重力保活线程每 40ms 发一条
        // 清队 opcode, 若空清队也刷新截止, 额度就永远不过期。
    }
    const std::string why =
        (reason.empty() ? std::string("在途规划被其它命令作废") : reason) +
        " —— 固件侧不会再发 0x4E, 结局未知 (只能回读状态判定)";
    for (const auto& t : tokens) t->fail(why);
    return n;
}

int CartPending::absorbance_locked(double now) {
    // 当前有效吸收额度 —— 惰性过期: now > 截止 即视为 0。
    // 判据是严格大于: now == 截止 算未过期。
    if (absorb_ && now > absorb_deadline_) absorb_ = 0;
    return absorb_;
}

int CartPending::pending() {
    std::lock_guard<std::mutex> lk(lock_);
    return int(q_.size());
}

void CartPending::unclaim(const CartTokenPtr& token) {
    std::lock_guard<std::mutex> lk(lock_);
    for (auto it = unclaimed_.begin(); it != unclaimed_.end(); ++it) {
        if (*it == token) {
            unclaimed_.erase(it);
            break;
        }
    }
}

std::vector<uint8_t> CartPending::wait(const CartTokenPtr& token, double timeout) {
    const double end = clk().now_s() + timeout;
    bool link_down = false;
    std::string link_why;
    while (true) {
        {
            std::unique_lock<std::mutex> lk(token->mu);
            if (token->reply || token->lost) break;
            const double left = end - clk().now_s();
            if (left <= 0.0) break;
            token->cv.wait_for(lk, std::chrono::duration<double>(left));
        }
        // 读线程死了要当场响亮: 等满 timeout 才报"结局未知"会把"链路断了"说成
        // "固件没回"。所以每一拍醒来看一眼读者死因。
        if (arm_ != nullptr) {
            std::string why;
            if (arm_->link_dead(&why)) {
                link_down = true;
                link_why = why;
                break;
            }
        }
    }
    if (token->lost) {
        throw CartReplyLostError(*token->lost);
    }
    if (link_down) {
        // 把病因原样抛出去: 报 CartReplyLostError ("结局未知") 会把"链路断了"说成
        // "固件没回", 排查方向直接反。
        // timed_out=false: 这一支没等过窗口 => 照旧补额度。走成 true 会让固件真欠的那条
        // 应答被"例外段"吞掉 => 下一条命令假成功。
        drop_and_absorb(token, false);
        token->fail("读线程已退出 (链路没了) —— 结局未知");
        throw TransportError("等待 0x4E 期间读线程已退出: " + link_why);
    }
    if (!token->reply) {
        // 超时 —— "少了一条"
        drop_and_absorb(token, true);
        token->fail("超时 " + std::to_string(timeout) +
                    "s 没等到 0x4E 应答 (固件单槽 pending 在突发下会吞掉中段应答) —— "
                    "结局未知 (只能回读状态判定)");
        throw CartReplyLostError(*token->lost);
    }
    unclaim(token);   // 结果已交付, 不许再被 poll_cart 认领
    return *token->reply;
}

// ===========================================================================
// 三条入口的内部实现
// ===========================================================================

namespace {

/// 位姿入参 -> [x,y,z,r,p,y] (固件 0x4E 家族的载荷形态)。
///
/// 不合法时 as_pose 抛的 InvalidCommandError 文案里带实际收到的形状 —— 这里只把 label
/// (哪条入口的哪个位姿) 前缀上去, 原文原样保留, 不重写、不换成笼统的"pose 非法"。
std::array<double, 6> as_pose6_labeled(const rot::PoseInput& pose, const std::string& label) {
    try {
        return rot::as_pose6(pose);
    } catch (const InvalidCommandError& e) {
        throw InvalidCommandError(label + ": " + e.what());
    } catch (const std::exception& e) {
        throw InvalidCommandError(label + ": " + e.what());
    }
}

double as_speed(double speed, const std::string& label) {
    if (!(speed >= 0.0 && speed <= 1.0)) {
        throw InvalidCommandError(label + ": speed 需 0..1 (给的是 " +
                                  std::to_string(speed) + ")");
    }
    return speed;
}

}  // namespace

struct CartImpl {
    /// ② 能力检查 —— 排在 require 之后、零重力守卫之前。
    ///
    /// 报错分两支措辞 —— 探测那一步把两种"不支持"分开了 (_cart_probe_silent), 在这里混成
    /// 一句话会让现场无法归因: 一支该换固件, 另一支该查链路。两支的处置方向相同 (都不发帧,
    /// fail-closed), 分开的只是说法。
    static void require_cart_support(Arm* arm) {
        if (arm->cart_supported_) return;
        std::string why;
        if (arm->cart_probe_silent_) {
            why = "探测未获确认: connect() 时的空载荷 0x3A 探测 3 次都没等到应答 —— "
                  "固件的 default 分支保证任何固件都会回一条 ERR, 所以这只能是上行丢帧 "
                  "(本硬件 USB CDC 有已知丢帧), 不是'固件没有这条命令'";
        } else {
            why = "固件确报不支持: connect() 时的空载荷 0x3A 探测回了 ERR{0x3A,0x00} "
                  "(default 分支 => 固件没编进 LITEARM_CART_PLAN)";
        }
        throw UnsupportedByFirmwareError(
            why + " —— 故不发帧 (未确认支持就不发能起规划的命令)。当前固件: " +
                      (arm->firmware_.empty() ? std::string("?") : arm->firmware_),
            proto::CMD_MOVE_L, 0x00);
    }

    /// ③ 零重力守卫 —— **必须在 request() 之前**跑完。
    ///
    /// write_cmd 里有一道判据完全相同的守卫, 它在这里重复出现, 是为了让"零重力保活期间
    /// 不许发动作命令"这条判定发生在登记 token 之前 —— 否则 InvalidCommandError 会在写失败
    /// 的位置抛出, 而 CartPending::request 只对 TransportError 摘 token, 于是 token 留在
    /// 队里。
    static void reject_in_zero_g(Arm* arm) {
        if (arm->zero_g_active()) throw InvalidCommandError(ZERO_G_GUARD_MESSAGE);
    }

    /// 此刻已知的最高状态帧序号 —— 到位判据的新鲜度闸水位线。
    /// 还没见过任何状态帧时返回 nullopt (那次等待会先拿第一帧当起算点)。
    static std::optional<uint16_t> seq_now(Arm* arm) {
        const auto st = arm->require().state_copy();
        if (!st) return std::nullopt;
        return st->seq;
    }

    /// 到位判据满足之后, 回读一次实际 TCP 与本次请求的目标位姿比对。
    ///
    /// 这是"假成功"方向唯一的一道防线: bit10 1->0 + q 静止 这个观察, 同样由"这条轨迹被别的
    /// 运动作废之后的收尾"产生 —— 固件 ctrl_accept_move_j 的第一条语句就是作废在途笛卡尔
    /// 规划, 于是下一帧 bit10=0, 臂转去执行那条命令; 它跑完也静止 => 只看 bit10 会把"中途
    /// 被扯断、TCP 根本不在目标上"报成到位。跨进程同样成立, SDK 侧拦不住。
    /// 取不到就报"没到位" (超时/断连/被别的 ERR 串台): 保守方向, 与"宁可报未知"同向。
    static bool tcp_reached(Arm* arm, const std::array<double, 6>& goal) {
        std::optional<std::array<double, 6>> tcp;
        try {
            tcp = arm->get_tcp().value;
        } catch (const LiteArmError&) {
            return false;
        }
        if (!tcp) return false;
        return arm->pose_near(*tcp, goal, CART_START_POS_TOL, CART_START_RPY_TOL);
    }

    /// 等"臂真的停下来" —— 返回 (started_busy, q_final, settle_err_rad)。
    ///
    /// 判据不能复用 Arm::arrive: 那个的 done 判据要目标关节向量, 而笛卡尔路径的目标 q 在
    /// PC 侧不存在 (规划在固件里) => 这里的 q_tol 语义随之从"与目标的差"改成"与上一帧的差"
    /// (数值沿用 0.03)。上限沿用 move_timeout (不新造旋钮)。
    ///
    /// 双判据缺一不可: 相邻两帧的 q 逐轴差 < q_tol 连续 arrive_frames 帧 且 dq 的 max-norm
    /// < dq_tol。只抄 q_tol 一半会宽松得多 —— 静态保持下 dq 的抖动足以把"还在爬行"判成
    /// "停稳"。
    ///
    /// 新鲜度闸 (seq0 + (st.seq - seq0) & 0xFFFF 属于 (0, 32768), u16 回绕安全): 只信
    /// "命令发出之后生成"的状态帧。陈旧帧直接丢弃: 既不判到位, 也不判故障 —— 故障是锁存的,
    /// 真故障会在可信帧上重现, 用陈旧帧判故障只会造成假中止。
    ///
    /// 必须能退出: 一旦臂在安全包络触发下锁存 (mode=EMERGENCY + enabled=false, 不动 cart
    /// FSM), CART_BUSY 会常亮 —— 只等 bit10 落 0 会一直等到超时, 这就是上限存在的理由
    /// (另外 st.faulted 那条会立刻抛: EMERGENCY 置 mode=6)。
    ///
    /// 断连时本循环在下一次取帧就拿到 NotConnectedError 退出, 不会挂到超时 —— 归因必须是
    /// "链路断了"而不是"没到位"。故取帧必须经 arm->require(): 直接摸 arm->a_ 在 close 之后
    /// 是空, 抛出来的会是别的异常。
    struct SettleInfo {
        bool started_busy = false;
        std::vector<double> q_final;
        double settle_err_rad = 0.0;
    };

    static SettleInfo wait_settled(Arm* arm, const std::string& label,
                                   std::optional<uint16_t> seq0) {
        const double budget = arm->move_timeout;
        const double end = arm->clock().now_s() + budget;
        SettleInfo out;
        std::optional<std::vector<double>> prev_q;
        // 判到位的那几帧的 q (arrive_frames 帧为窗, 静止判据一破就清空重来) ——
        // 收尾时用它算 settle_err_rad (最后几帧的抖动幅度)。
        std::vector<std::vector<double>> window;
        // 上一帧的 seq —— 去重: 采样是靠"等 status_seq 前进"驱动的, 但等超时时 state 可能
        // 还是同一帧; 数两次就等于窗里少判了一帧。
        std::optional<uint16_t> last_seq;
        while (arm->clock().now_s() < end) {
            Ack& a = arm->require();
            const uint64_t cur = a.status_seq_now();
            const double left = end - arm->clock().now_s();
            arm->pump_until(std::min(0.05, std::max(left, 0.0)),
                            [cur](Ack& a) { return a.status_seq_now() != cur; }, label);
            const auto st_opt = arm->require().state_copy();
            if (!st_opt) continue;
            const RobotState& st = *st_opt;
            if (last_seq && st.seq == *last_seq) continue;   // 同一帧: 不重复计数
            last_seq = st.seq;
            if (!seq0) {
                seq0 = st.seq;   // 还没有水位线: 第一帧只当起算点
                continue;
            }
            const uint16_t delta = uint16_t((st.seq - *seq0) & 0xFFFF);
            if (!(delta > 0 && delta < 32768)) continue;   // 陈旧帧: 不判到位, 也不判故障
            if (st.faulted()) {
                throw MotorFaultError(label + ": 未到位即故障 FAULT " + st.fault_detail());
            }
            if (st.cart_busy()) out.started_busy = true;
            const std::vector<double> q = st.q();
            // 无前一帧时不能短路掉 dq 那半边 (起点那帧一样要过 dq_tol): 让一个 dq 超容差的
            // 帧当上静止窗的起点, 就等于窗里少判了一帧。
            bool still = true;
            for (double d : st.dq()) {
                if (!(std::fabs(d) < arm->dq_tol)) {
                    still = false;
                    break;
                }
            }
            if (still && prev_q) {
                if (prev_q->size() != q.size()) {
                    still = false;
                } else {
                    for (size_t i = 0; i < q.size(); ++i) {
                        if (!(std::fabs(q[i] - (*prev_q)[i]) < arm->q_tol)) {
                            still = false;
                            break;
                        }
                    }
                }
            }
            // 窗 = 本段连续静止的帧 (起点那帧没有"上一帧"可差, 但它不破坏连续性: 第一帧就算
            // 数); 一动就清空重来。
            if (still) {
                window.push_back(q);
                while (int(window.size()) > arm->arrive_frames) window.erase(window.begin());
            } else {
                window.clear();
            }
            prev_q = q;
            // 主判据: 见过 bit10=1 => 要的正是 1->0 那一刻; 降级分支: 始终没见过 1 也收。
            if (int(window.size()) >= arm->arrive_frames && !st.cart_busy()) {
                out.q_final = window.back();
                double err = 0.0;
                for (const auto& w : window) {
                    for (size_t i = 0; i < w.size(); ++i) {
                        err = std::max(err, std::fabs(w[i] - out.q_final[i]));
                    }
                }
                out.settle_err_rad = err;
                return out;
            }
        }
        throw MotionTimeoutError(label + " 未到位, 超时 " + std::to_string(budget) + "s");
    }

    /// ⑤ 登记 token + 写帧, 然后等 0x4E 并映射结果。**全程持 cart_serial_。**
    ///
    /// request 的 callable 只包含那一次帧写 —— 用 write_cmd (而不是 raw_write): 它带零重力
    /// 守卫, 是既有约定; 绕开它会让"零重力中发笛卡尔"变成只能在固件侧被拒。
    ///
    /// 串行是强制的 (见 cart.hpp 顶部): 持锁范围是登记 -> 等 ACK/ERR -> 等 0x4E —— 少一段
    /// 都不行, 因为假成功正是"两条在途交错"造成的。
    ///
    /// 等 ACK/ERR 不是可选的: 固件受理后立刻回一条 RSP_ACK{cmd}, 被门禁或长度校验拒掉时回的是
    /// RSP_ERR{cmd,code} 且不会有 0x4E。不读这一条的话, 一次"臂未使能被拒"会表现成耗满
    /// move_timeout 之后的 CartReplyLostError (报"结局未知"), 把用户推向"臂可能正在动"的
    /// 错误结论。
    ///
    /// 这一等必须按"受理必回 ACK、拒绝绝不回 ACK"的形状读 (err_waits_for_ack=true): 一条
    /// 匹配命令码的 ERR 证明不了"我这条被拒了" —— 0x3A/0x3B/0x3E 共用码空间, 而流里可以有
    /// 别人的 ERR (最现实的一条: 探测的迟到应答)。
    ///
    /// 固件显式拒绝时摘掉自己那条 token: 受理前的每一道校验/门禁都在 cart_req_* 之前 break,
    /// 所以"本命令的 ERR => 永远不会有 0x4E"。留着它会占住 FIFO 队首, 让下一条合法笛卡尔命令
    /// 的应答配给这条死 token。
    /// ⚠ 这里只 drop, 不给吸收额度 (clear_pending/超时那两处才给): 本命令确实没被受理 =>
    /// 固件一格应答都不欠 => 留额度只会去吞下一条命令自己的 0x4E。判"被拒"与"给额度"这两件
    /// 事不能同时做。
    ///
    /// wait=true 时到位等待也在这把锁之内 —— 一条笛卡尔命令的"整条运动"才是串行的单位。
    /// goal 由调用方给 (它知道本次请求的终点): move_l/move_c 用终点, move_path 用最后一个
    /// 路点。目标位姿是必填的: 没有它就只剩"看 bit10"这一条判据 —— 那是本模块最防的假成功
    /// 方向。
    static CartPlan request_and_wait(Arm* arm, uint8_t cmd,
                                     const std::vector<uint8_t>& payload,
                                     const std::string& label,
                                     const std::array<double, 6>& goal,
                                     std::optional<uint16_t> seq0, bool wait) {
        std::lock_guard<std::recursive_mutex> lk(arm->cart_serial_);
        const auto tok = arm->cart_->request([&]() { arm->write_cmd(cmd, payload); });
        try {
            arm->require().expect(proto::RSP_ACK, 1.0, label, true, cmd, true);
        } catch (const CommandRejectedError&) {
            arm->cart_->drop(tok);
            throw;
        }
        const auto reply = arm->cart_->wait(tok, arm->move_timeout);
        CartPlan plan = CartPlan::from_reply(reply);
        raise_for_plan(plan);
        if (wait) {
            const SettleInfo info = wait_settled(arm, label, seq0);
            plan.started_busy = info.started_busy;
            plan.q_final = info.q_final;
            plan.settle_err_rad = info.settle_err_rad;
            // 到位判据满足之后才回读 TCP: "停稳"不等于"停在了目标上"。
            plan.settled = tcp_reached(arm, goal);
        }
        return plan;
    }
};

// ===========================================================================
// 能力探测
// ===========================================================================

namespace {
/// 探测的总尝试次数 (首次 + 重试)。见 probe 的说明: 读超时只可能是丢帧。
constexpr int kCartProbeAttempts = 3;
}  // namespace

bool probe(Arm* arm) {
    arm->cart_probe_silent_ = false;
    for (int i = 0; i < kCartProbeAttempts; ++i) {
        arm->write_query(proto::CMD_MOVE_L);
        try {
            const auto r = arm->require().expect(
                proto::RSP_ERR, 1.0, "笛卡尔能力探测 (0x3A 空载荷)", false,
                proto::CMD_MOVE_L);
            // 判据就是能力判定那一条: 0x00 恒等于固件的 default 分支 (未实现该命令),
            // 其余任何非 0 的错误码都说明这条命令在固件里存在。
            const uint8_t code = r.payload.size() > 1 ? r.payload[1] : 0;
            return code != 0x00;
        } catch (const MotionTimeoutError&) {
            continue;   // 安静 = 这一帧丢了, 再试一次
        }
    }
    arm->cart_probe_silent_ = true;   // 3 次均无应答 —— 探测未获确认 (不是"确报不支持")
    return false;
}

// ===========================================================================
// 三条入口
// ===========================================================================

CartPlan move_l(Arm* arm, const rot::PoseInput& pose, double speed, bool wait) {
    arm->require();                        // ① 未连接 -> NotConnectedError (早于登记)
    CartImpl::require_cart_support(arm);   // ②
    CartImpl::reject_in_zero_g(arm);       // ③
    const auto vec = as_pose6_labeled(pose, "move_l");   // ④ (无副作用的参数校验)
    const double sp = as_speed(speed, "move_l");
    std::vector<uint8_t> payload = proto::pack_f32s(vec);
    const auto spb = proto::pack_f32le(sp);
    payload.insert(payload.end(), spb.begin(), spb.end());
    // 水位线取在那一次 0x3A 帧写之前 —— 按"只信命令发出之后生成的状态帧"取。
    const auto seq0 = CartImpl::seq_now(arm);
    return CartImpl::request_and_wait(arm, proto::CMD_MOVE_L, payload, "move_l", vec,
                                      seq0, wait);
}

CartPlan move_c(Arm* arm, const rot::PoseInput& pose_start, const rot::PoseInput& pose_via,
                const rot::PoseInput& pose_goal, double speed, bool wait) {
    arm->require();                        // ①
    CartImpl::require_cart_support(arm);   // ②
    CartImpl::reject_in_zero_g(arm);       // ③
    const auto start = as_pose6_labeled(pose_start, "move_c pose_start");   // ④
    const auto via = as_pose6_labeled(pose_via, "move_c pose_via");
    const auto goal = as_pose6_labeled(pose_goal, "move_c pose_goal");
    const double sp = as_speed(speed, "move_c");
    // ⑤ 起点校验 + 发帧 + 等应答 = 一个临界区。
    // 参数校验 (①②③④) 留在锁外: 它们不读帧、不写帧, 进锁只会平白拉长持锁时间。
    //
    // 起点校验那次 get_tcp 在 cart_serial_ 之内 (不是"取锁之前"): 它读的就是"此刻 TCP",
    // 与正在跑的那条运动本来就是同一个临界区。放在锁外实测会抢走在跑那条的 ACK。
    // 水位线 seq0 取在 get_tcp 之后: 那次读已经把 TCP 校验要用的状态帧吃进来了, 取在它
    // 之前只会把水位线白白放低 (闸更松), 不会有别的效果。
    std::lock_guard<std::recursive_mutex> lk(arm->cart_serial_);
    const auto tcp = arm->get_tcp().value;
    if (!tcp) {
        throw TransportError("move_c 需要校验起点, 但取不到当前 TCP (get_tcp 无应答)");
    }
    if (!arm->pose_near(*tcp, start, CART_START_POS_TOL, CART_START_RPY_TOL)) {
        std::ostringstream os;
        os << "move_c: pose_start 与当前 TCP 不一致 —— 固件圆弧的起点恒为实测 TCP, 给的 "
              "start=[";
        for (int i = 0; i < 6; ++i) os << (i ? ", " : "") << start[size_t(i)];
        os << "], 实际 tcp=[";
        for (int i = 0; i < 6; ++i) os << (i ? ", " : "") << (*tcp)[size_t(i)];
        os << "]";
        throw InvalidCommandError(os.str());
    }
    std::vector<uint8_t> payload = proto::pack_f32s(via);
    const auto g = proto::pack_f32s(goal);
    const auto s = proto::pack_f32le(sp);
    payload.insert(payload.end(), g.begin(), g.end());
    payload.insert(payload.end(), s.begin(), s.end());
    const auto seq0 = CartImpl::seq_now(arm);
    return CartImpl::request_and_wait(arm, proto::CMD_MOVE_C, payload, "move_c", goal,
                                      seq0, wait);
}

CartPlan move_path(Arm* arm, const std::vector<rot::PoseInput>& poses, double speed,
                   bool wait) {
    arm->require();                        // ①
    CartImpl::require_cart_support(arm);   // ②
    CartImpl::reject_in_zero_g(arm);       // ③
    if (poses.empty()) throw InvalidCommandError("move_path: 路径为空");
    if (poses.size() > 32) {   // 固件 CART_MAX_GOAL = 32
        throw InvalidCommandError("move_path: 路点数 " + std::to_string(poses.size()) +
                                  " 超固件上限 32 (CART_MAX_GOAL)");
    }
    std::vector<std::array<double, 6>> pts;
    pts.reserve(poses.size());
    for (const auto& p : poses) pts.push_back(as_pose6_labeled(p, "move_path"));   // ④
    const double sp = as_speed(speed, "move_path");
    // 水位线取在 BEGIN 之前 —— 理由是窗口完整性: "命令"是这一整段 (BEGIN+ADD x n+RUN),
    // 而闸只信 seq > seq0 的帧 => 水位线取得越晚、被排除的帧越多。
    const auto seq0 = CartImpl::seq_now(arm);
    // token 只登记在 0x3E (RUN) 上: 只有 RUN 会起规划, 也只有它会产出 0x4E。
    // 0x3C/0x3D 既不在清队集合里, 也不会产生 0x4E。
    //
    // BEGIN + ADD x n + RUN 整段持 cart_serial_: 只圈 RUN 是不够的 —— 固件的 RECV 收集态
    // 只从 IDLE 开, 两条并发 move_path 里后到的那条会在 BEGIN 就被拒。持锁的作用是把它变成
    // 排队 (两条都跑完), 并把 BEGIN->RUN 整段与其它笛卡尔入口互斥。
    //
    // BEGIN/ADD 中途失败时不做任何"清状态"的收尾: 抛错即可 —— 固件侧 RECV 态带 2s 倒计时,
    // 会自行收敛。但别拿"不占用臂"当论据 —— RECV 占用臂 (BEGIN/ADD 会当场作废在途的逐关节
    // 轨迹并把参考锚住), 不占用的是 CART_BUSY (bit10) 那一位。
    std::lock_guard<std::recursive_mutex> lk(arm->cart_serial_);
    {
        std::vector<uint8_t> begin_payload{uint8_t(pts.size())};
        const auto s = proto::pack_f32le(sp);
        begin_payload.insert(begin_payload.end(), s.begin(), s.end());
        arm->cmd_expect_ack(proto::CMD_CART_BEGIN, begin_payload, "move_path BEGIN");
    }
    for (size_t i = 0; i < pts.size(); ++i) {
        std::vector<uint8_t> add_payload{uint8_t(i)};
        const auto p = proto::pack_f32s(pts[i]);
        add_payload.insert(add_payload.end(), p.begin(), p.end());
        arm->cmd_expect_ack(proto::CMD_CART_ADD, add_payload,
                            "move_path ADD " + std::to_string(i));
    }
    return CartImpl::request_and_wait(arm, proto::CMD_CART_RUN, {}, "move_path RUN",
                                      pts.back(), seq0, wait);
}

}  // namespace litearm
