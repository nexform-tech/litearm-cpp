#include "litearm/ack.hpp"

#include <chrono>
#include <cstdio>

#include "litearm/arm.hpp"

#ifdef _WIN32
#  include <process.h>
#  define LITEARM_ACK_GETPID() ::_getpid()
#else
#  include <unistd.h>
#  define LITEARM_ACK_GETPID() ::getpid()
#endif

namespace litearm {

QueueKey read_key(uint8_t c, const std::vector<uint8_t>& payload) {
    // 只有 RSP_ACK / RSP_ERR 的 payload[0] 是"原命令码"。
    if (proto::is_echoed(c) && !payload.empty()) return {c, int(payload[0])};
    return {c, kNoEcho};
}

std::vector<QueueKey> wait_keys(uint8_t want, std::optional<uint8_t> echo_cmd) {
    if (!proto::is_echoed(want)) {
        // 给了 echo_cmd 就还要等"回显了本命令码的 ERR" —— 那是本命令被拒的唯一载体
        // (expect 的 raise_on_err 靠它)。少了这一条, 一次被拒会退化成"无应答"超时。
        if (!echo_cmd) return {{want, kNoEcho}};
        return {{want, kNoEcho}, {proto::RSP_ERR, int(*echo_cmd)}};
    }
    if (!echo_cmd) {
        throw InvalidCommandError(
            "等 RSP_ACK/RSP_ERR 必须给 echo_cmd —— 它的归属判据是回显的命令码; "
            "不给就分不清'我的 ACK'与'别人的 ACK' (并发下会互吃)");
    }
    if (want == proto::RSP_ACK) {
        // 受理回 ACK{cmd}、拒绝回 ERR{cmd,code} (绝不补 ACK) => 两条队列都得等
        // (err_waits_for_ack 与 raise_on_err 都靠这一条)。
        return {{proto::RSP_ACK, int(*echo_cmd)}, {proto::RSP_ERR, int(*echo_cmd)}};
    }
    return {{want, int(*echo_cmd)}};
}

Ack::Ack(Arm* arm) : arm_(arm) {}

const Clock& Ack::clk() const {
    // ⚠ 空安全: `arm_` 允许为空 (可以直接 `Ack a(nullptr)`)。
    return arm_ != nullptr ? arm_->clock() : steady_clock_instance();
}

Ack::~Ack() {
    if (owner_pid_ != -1 && LITEARM_ACK_GETPID() != owner_pid_) {
        // **fork 出来的子进程**: 这条读线程在子进程里不存在。
        //   * `stop_reader()` 会 `join()` —— 对一个不存在的线程 join 会**永久阻塞**;
        //   * 直接放着不管则 `~std::thread` 会因"joinable 却没 join"而 `std::terminate`。
        // 故把句柄交出去**故意泄漏**: 子进程要么马上退出 (内核回收), 要么会新建自己的
        // `Arm`。比"为了体面地回收一个对象而挂死/中止"划算得多。
        //
        // ⚠⚠ **这个守卫是必要的, 但远不是充分的。** 实测 (2026-09-24, glibc 2.35) 定位到
        //   真正会挂死的那一步**不是 join, 而是本类成员的析构**:
        //
        //       ~condition_variable(stop_cv_)  ← 子进程里**永久阻塞**
        //
        //   最小复现: 一个线程 `cv.wait_for(...)` 停在一个 condvar 上, `fork()` 之后在
        //   子进程里析构那个 condvar —— glibc 的 `pthread_cond_destroy` 会去取 condvar
        //   自己的**内部锁** (组切换 `__condvar_quiesce_and_switch_g1`), 而那把锁与已经
        //   不存在的等待者绑在一起 ⇒ 死锁。(同一个实验里: 析构"没人在等"的 condvar 与
        //   mutex **都没事**; detach 继承来的线程句柄也不抛; malloc/free 也安全。
        //   问题**只**出在"析构一个有人停在上面的 condvar"。)
        //
        //   而 `stop_cv_` / `cv` 是**成员** ⇒ 只要 `~Ack` 跑起来, 这一步就躲不掉 ——
        //   本守卫拦不住它。**所以调用方必须根本不析构本对象**: 真正救命的是
        //   `Arm::close()` fork 分支里的 `a_.release()` (见那里的注释), 本守卫只是
        //   在"有人绕过那条路径直接 delete 一个继承来的 Ack"时把伤害从
        //   `std::terminate` 降级为泄漏 —— 那一步之后仍会在成员析构上挂住。
        //   ⚠ 别把上面这段读成"本守卫能兜住": 它兜不住。
        // ⚠ 走到这里说明**有人直接析构了一个继承来的 `Ack`** —— 支持的路径
        //   (`Arm::close()` / `~Arm`) 不会这么做 (它们 `release()` 这个对象)。
        //   按本仓"丢帧不静默"的一贯口径, 这里也要出声: 这条路**下一句就会挂死**
        //   (成员析构里那个 condvar), 静默地挂比报一句错难查得多。
        std::fprintf(stderr,
                     "[litearm] 警告: 在 fork 出来的子进程 (pid=%d) 里析构了一个属于 "
                     "pid=%d 的会话对象。这不是支持的用法 —— 紧接着的成员析构会**永久"
                     "阻塞** (详见 ack.cpp 里 ~Ack 的注释)。请改用 arm.close(), 或在子进程"
                     "里新建一个 Arm。\n",
                     int(LITEARM_ACK_GETPID()), int(owner_pid_));
        std::fflush(stderr);
        try {
            new std::thread(std::move(reader_));
        } catch (...) {
            // 分配失败时无路可走 —— 但这已经不可能是"子进程收尾"这条路径上的常态。
        }
        return;
    }
    stop_reader();
}

void Ack::start_reader(Transport* tr) {
    tr_ = tr;
    owner_pid_ = LITEARM_ACK_GETPID();
    stop_.store(false);
    reader_ = std::thread(&Ack::reader_loop, this, tr);
}

void Ack::stop_reader() {
    if (owner_pid_ != -1 && LITEARM_ACK_GETPID() != owner_pid_) {
        // 非属主进程 (fork 的子进程): 那条线程不在这里, join 会永久挂死。见析构函数。
        stop_.store(true);
        if (reader_.joinable()) {
            try {
                new std::thread(std::move(reader_));
            } catch (...) {
            }
        }
        return;
    }
    // 必须排在 transport->close() 之前 (否则一次正常关闭会被记成"链路丢失")。
    // 也必须 join: 线程的 target 是成员函数 => 线程引用本对象。
    stop_.store(true);
    {
        std::lock_guard<std::mutex> lk(stop_mu_);
        // 取一下锁再 notify: 保证退避中的读线程已经进了 wait (否则这次 notify 会丢,
        // 那一拍要多等 kIdleSleepS —— 无害但没必要)。
    }
    stop_cv_.notify_all();
    if (reader_.joinable()) reader_.join();
}

void Ack::reader_loop(Transport* tr) {
    // 唯一的读者 —— 全包只有这一个地方碰 transport->read_frame。
    //
    // 本设计的全部代价就是下面两条纪律:
    //   1. 只投递, 不判定 —— "这帧是谁的"由队列决定, 不由线程决定;
    //   2. 死了要响亮 —— 死因进 reader_error 并唤醒所有等待者, 绝不静默退出。
    while (!stop_.load()) {
        std::optional<proto::Frame> fr;
        try {
            fr = tr->read_frame(kReadSliceS);
        } catch (...) {
            die(std::current_exception());
            return;
        }
        if (!fr) {
            // 退避: SerialTransport 空闲时会阻塞满 kReadSliceS, 但传输接口没有这个契约
            // —— 测试桩就完全忽略 timeout。不退避的空循环会跑出接近 100% 的 CPU。
            // 用带超时的 wait 而不是纯 sleep: 让 stop_reader 不必等满这一拍。
            std::unique_lock<std::mutex> lk(stop_mu_);
            stop_cv_.wait_for(lk, std::chrono::duration<double>(kIdleSleepS));
            continue;
        }
        try {
            deliver(fr->cmd, std::move(fr->payload), clk().now_s());
        } catch (...) {
            // 协议层异常 (典型: 笛卡尔收集器的"多了一条"): 不当场抛、也不死, 存起来交给
            // 下一个等待者。
            MuLock lk(mu, dbg_mu_owner);
            errors.push_back(std::current_exception());
            cv.notify_all();
        }
    }
}

void Ack::die(std::exception_ptr e) {
    // 链路没了 —— 记死因并唤醒所有等待者 (绝不静默退出)。
    std::string text = "未知传输错误";
    try {
        std::rethrow_exception(e);
    } catch (const std::exception& ex) {
        text = ex.what();
    } catch (...) {
    }
    {
        std::lock_guard<std::mutex> lk(mu);
        reader_error = e;
        reader_error_text = text;
        cv.notify_all();
    }
}

void Ack::deliver(uint8_t c, std::vector<uint8_t> payload, double now) {
    // 把一帧投递到它该去的地方 —— 本包唯一的分发点。
    Arm* owner = nullptr;
    bool is_cart = false;
    {
        MuLock lk(mu, dbg_mu_owner);
        dbg_check("deliver");
        note_recv(c, now);
        if (c == proto::RSP_STATUS) {
            on_status(payload);
            status_seq += 1;
        } else if (c == proto::RSP_CART_PLAN) {
            owner = arm_;      // 收集器在锁外调 (见下)
            is_cart = true;
        } else {
            const QueueKey k = read_key(c, payload);
            Queue& q = queues[k];
            if (q.size() >= kQueueMax) {
                q.pop_front();   // 孤儿: 丢最旧, 不静默
                dropped += 1;
            }
            QueueEntry e;
            e.id = c;
            e.payload = std::move(payload);
            e.seq = next_seq_++;
            q.push_back(std::move(e));
        }
        cv.notify_all();
    }
    // on_reply 必须在 mu 之外: 它要去取 CartPending 的锁, 在全局锁里调另一个子系统的锁,
    // 那边一慢就会把所有等待者一起卡住。
    if (is_cart && owner != nullptr) {
        owner->on_cart_plan_reply(payload);
    }
}

void Ack::on_status(const std::vector<uint8_t>& payload) {
    try {
        state = decode_state(payload);
    } catch (const std::exception&) {
        // 非法状态帧: 忽略 (不改变已有缓存), 但**要计数**。
        // ⚠ 计数不是装饰: "每帧都解不出来"与"根本没有帧"在等待者眼里是同一种静默,
        //   而两者要查的方向相反。见 `bad_status_frames` 的说明。
        bad_status_frames += 1;
    }
}

void Ack::note_recv(uint8_t c, double now) {
    auto it = recv_.find(c);
    if (it == recv_.end()) {
        RecvEnt e;
        e.count = 1;
        e.first = now;
        e.last = now;
        recv_[c] = e;
    } else {
        it->second.count += 1;
        it->second.last = now;
    }
}

std::pair<double, double> Ack::recv_stats(uint8_t c) {
    MuLock lk(mu, dbg_mu_owner);
    dbg_check("recv_stats");
    const auto it = recv_.find(c);
    if (it == recv_.end()) return {0.0, 0.0};
    const RecvEnt& e = it->second;
    const double span = e.last - e.first;
    if (e.count < 2 || span <= 0.0) {
        // span <= 0 是防御性的: 两帧撞进同一个读数到不了 —— 留着是为了让本函数对任何
        // 输入都有定义。
        return {0.0, e.last};
    }
    return {double(e.count - 1) / span, e.last};
}

std::optional<RobotState> Ack::state_copy() {
    MuLock lk(mu, dbg_mu_owner);
    dbg_check("state_copy");
    return state;
}

void Ack::drain_for(uint8_t cmd) {
    // 由 Arm::raw_write 在发帧之前调 —— 位置是载重的 (见头文件)。
    std::vector<QueueKey> keys = {{proto::RSP_ACK, int(cmd)}, {proto::RSP_ERR, int(cmd)}};
    const auto rsp = proto::rsp_of(cmd);
    if (rsp) keys.emplace_back(*rsp, kNoEcho);
    MuLock lk(mu, dbg_mu_owner);
    dbg_check("drain_for");
    for (const auto& k : keys) {
        const auto it = queues.find(k);
        if (it != queues.end()) {
            dropped += it->second.size();
            queues.erase(it);
        }
    }
}

std::optional<QueueEntry> Ack::wait(const std::vector<QueueKey>& keys, double timeout) {
    if (arm_ == nullptr) {
        throw NotConnectedError("链路已关闭 (Arm::close() 之后) —— 请先 connect()");
    }
    // 三个守卫排最前, 一个都不能少 —— 漏哪条都是从"响亮地报错"静默退化成"等满超时"。
    arm_->guard_for_ack_wait();
    if (timeout < 0.0) {
        throw InvalidCommandError(
            "timeout 不能为负 —— 负值没有等待语义; 要'不等、只探一次'请传 0.0");
    }
    const double end = clk().now_s() + timeout;
    MuLock lk(mu, dbg_mu_owner);
    dbg_check("wait");
    while (true) {
        QueueEntry best;
        bool have_best = false;
        QueueKey best_key{};
        for (const auto& k : keys) {
            const auto it = queues.find(k);
            if (it == queues.end() || it->second.empty()) continue;
            if (!have_best || it->second.front().seq < best.seq) {
                best = it->second.front();
                best_key = k;
                have_best = true;
            }
        }
        if (have_best) {
            queues[best_key].pop_front();
            return best;
        }
        if (!errors.empty()) {
            std::exception_ptr e = errors.front();
            errors.pop_front();
            std::rethrow_exception(e);
        }
        if (reader_error) {
            throw TransportError("读线程已退出: " + reader_error_text);
        }
        const double left = end - clk().now_s();
        if (left <= 0.0) return std::nullopt;
        cv.wait_for(lk.ul(), std::chrono::duration<double>(left));
    }
}

ExpectResult Ack::expect(uint8_t want, double timeout, const std::string& cmdlabel,
                         bool raise_on_err, std::optional<uint8_t> echo_cmd,
                         bool err_waits_for_ack) {
    const std::vector<QueueKey> keys = wait_keys(want, echo_cmd);
    std::optional<std::vector<uint8_t>> err_seen;
    // 这里不清队: 清队由 Arm::raw_write 在发帧之前做 (drain_for) —— 放在这里会 100%
    // 吃掉自己的应答。
    const double end = clk().now_s() + timeout;
    while (true) {
        const double left = end - clk().now_s();
        if (left <= 0.0) break;
        const auto fr = wait(keys, left);
        if (!fr) break;
        const uint8_t c = fr->id;
        const std::vector<uint8_t>& pl = fr->payload;
        if (c == proto::RSP_ERR) {
            if (err_waits_for_ack && !err_seen) {
                // 先记下, 等窗口给结论。第二条起的 ERR 不再记 —— 一条命令至多被拒一次。
                err_seen = pl;
                continue;
            }
            if (raise_on_err) raise_firmware_error(pl, cmdlabel + " 被固件拒绝: ");
            ExpectResult r;
            r.kind = ExpectResult::Kind::Err;
            r.payload = pl;
            return r;
        }
        ExpectResult r;
        r.kind = (want == proto::RSP_ACK) ? ExpectResult::Kind::Ack
                                          : ExpectResult::Kind::Rsp;
        r.payload = pl;
        return r;
    }
    if (err_seen) {
        // 窗口里只见过 ERR、没见到 ACK{echo_cmd} => 它就是本命令的拒绝。
        if (raise_on_err) raise_firmware_error(*err_seen, cmdlabel + " 被固件拒绝: ");
        // 本行今天没有调用方走到 (err_waits_for_ack 且 raise_on_err=false 才可达)。
        // 保留它是刻意的: 它两个兄弟分支各覆盖一种开关组合, 三者共同定义 expect 的返回
        // 契约 —— 删掉它, 那条"我不想抛、但请把拒绝告诉我的"组合会掉到末尾的超时异常,
        // 而那时明明收到了应答 (一条 ERR), 归因直接反了。
        ExpectResult r;
        r.kind = ExpectResult::Kind::Err;
        r.payload = *err_seen;
        return r;
    }
    throw MotionTimeoutError(cmdlabel + " 无应答(超时 " +
                             std::to_string(timeout) + "s)");
}

}  // namespace litearm
