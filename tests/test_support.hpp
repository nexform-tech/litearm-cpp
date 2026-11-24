// 测试夹具 —— 离线 Arm (注入 FakeTransport) 与若干共用断言。
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "litearm/litearm.hpp"
#include "litearm/testing.hpp"
#include "test_framework.hpp"

namespace litearm {
/// 测试专用访问口 —— `Arm` 里 `friend struct TestAccess;` 一行换来的。
///
/// ⚠ 只有**一个**方法, 而且只给出 `Ack&`。别往这里加 `raw_write` / `raw_write` 之类:
/// 那会把"唯一写口/唯一读口"这两条安全设计在测试里绕过去, 判据就失去判别力了。
struct TestAccess {
    /// 会话内部的应答队列 + 读线程状态 (上游 Python 直接摸 `arm._a`)。
    static Ack& ack(Arm& arm) { return arm.require(); }
    /// 该队列当前的长度 (没有该队列时为 0)。
    static size_t queue_len(Arm& arm, QueueKey key) {
        Ack& a = arm.require();
        std::lock_guard<std::mutex> lk(a.mu);
        const auto it = a.queues.find(key);
        return it == a.queues.end() ? 0 : it->second.size();
    }
    /// 直接下发一帧 (上游用例直接调 `arm._raw_write`; C++ 有访问控制, 走这里)。
    ///
    /// ⚠ 只给"**测顺序/测守卫**"这类用例用 —— 它绕开了 `write_cmd` 的零重力守卫,
    /// 也绕开了 `_cmd` 的 ACK 等待。**不要**拿它去替代正常入口 (那会让用例测的不是
    /// 用户真正走的那条路)。
    static void raw_write(Arm& arm, uint8_t cmd, const std::vector<uint8_t>& payload) {
        arm.raw_write(cmd, payload.data(), payload.size());
    }

    /// 往"在途笛卡尔配对队列"里登记一条 token (上游: `arm._cart.register()`)。
    /// 返回的票据可以用来查它有没有被标成"结局未知" (`->lost`)。
    static CartTokenPtr cart_register(Arm& arm) { return arm.cart_->register_token(); }
    /// 在途条数 (上游: `arm._cart.pending`)。
    static int cart_pending(Arm& arm) { return arm.cart_->pending(); }

    /// 到位判据的纯谓词 (`Arm::pose_near`) —— 无副作用, 故开给用例。
    /// 上游 Python 直接调 `Arm._pose_near`; C++ 有访问控制, 走这里。
    static bool pose_near(Arm& arm, const std::array<double, 6>& tcp,
                          const std::array<double, 6>& goal, double pos_tol,
                          double rpy_tol) {
        return arm.pose_near(tcp, goal, pos_tol, rpy_tol);
    }

    /// 等某条队列被读线程填上 (用例要控制"帧已经躺在队列里"这个前提)。
    static bool await_queued(Arm& arm, QueueKey key, double timeout = 1.0) {
        const double end = now_s() + timeout;
        while (now_s() < end) {
            if (queue_len(arm, key) > 0) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return queue_len(arm, key) > 0;
    }
};

}  // namespace litearm

namespace lt {

/// 仓库根目录 —— 从 `__FILE__` 推 (它在**任何人的机器**上都成立)。
///
/// ⚠ 别用"相对当前目录"的路径 (`"include/litearm/arm.hpp"` 这种): 手跑二进制时 cwd 是
/// 仓根, 而 ctest 的 cwd 是**构建目录** —— 同一个路径两种结果。实测踩过。
std::string repo_root();

/// 读一个源文件 (相对仓库根)。读不到返回空串。
std::string read_repo_file(const std::string& rel_path);

/// 列出 `repo_root()/<dir>` 下文件名以 `prefix` 开头、以 `suffix` 结尾的文件
/// (返回相对仓库根的路径, 按名排序)。
///
/// ⚠ 用它是为了让"扫全部测试源"这类判据**跟着目录走**, 而不是维护一张手写清单 ——
///   手写清单漏掉一个文件时, 那条判据就对那个文件**静默失效**, 而失效的方向恰好是
///   **假绿**(新加的文件不在清单里 ⇒ 它里面缺什么都不会被报出来)。
std::vector<std::string> list_repo_files(const std::string& dir, const std::string& prefix,
                                         const std::string& suffix = "");

/// 剥掉 `/* */` 与 `//` 注释 —— **一切源码扫描类判据的必备前提**。
///
/// ⚠ 不剥的话注释里的词会被当成代码: 实测踩过 (一条哨兵把 `sleep` / `ff_clamp` /
///   `activated` 这些**注释里的词**当成了公开成员名, 于是那条判据变成噪声源)。
inline std::string strip_cxx_comments(const std::string& raw) {
    std::string src;
    src.reserve(raw.size());
    for (size_t i = 0; i < raw.size();) {
        if (raw[i] == '/' && i + 1 < raw.size() && raw[i + 1] == '*') {
            const size_t e = raw.find("*/", i + 2);
            i = (e == std::string::npos) ? raw.size() : e + 2;
            src.push_back(' ');
        } else if (raw[i] == '/' && i + 1 < raw.size() && raw[i + 1] == '/') {
            const size_t e = raw.find('\n', i);
            i = (e == std::string::npos) ? raw.size() : e;
            src.push_back(' ');
        } else {
            src.push_back(raw[i++]);
        }
    }
    return src;
}


using namespace litearm;

/// 把共享的 FakeTransport 包成一个 Transport —— 这样 Arm 析构时只销毁这层薄壳,
/// 测试持有的 fake 实例仍可用 (跨 reconnect 也看得见同一份状态)。
class SharedFake : public Transport {
public:
    explicit SharedFake(std::shared_ptr<testing::FakeTransport> t) : t_(std::move(t)) {}
    void write_frame(uint8_t cmd, const uint8_t* payload, size_t len) override {
        t_->write_frame(cmd, payload, len);
    }
    std::optional<proto::Frame> read_frame(double timeout) override {
        return t_->read_frame(timeout);
    }
    void close() override { t_->close(); }
    bool is_open() const override { return t_->is_open(); }
    // ⚠⚠ 这两条**必须转发给假件**, 不能返回空/默认值 —— 它们曾是两个**哑掉的口**:
    //   * `text_log()` 原来恒返回 `{}` ⇒ `last_reset_reason()` / `banner_version()`
    //     经由 `lt::Offline` **从来就测不到**(读了永远说"没有横幅")。实测发现。
    //   * `flush_failures()` 原来落到基类**自己**那个恒 0 的计数器上
    //     (它当时不是虚函数) ⇒ 假件注入的值永远传不上来。已把基类那条改成虚函数。
    //   ⇒ 教训与本仓别处同款: **包装层漏转发口 = 假绿**, 而假绿比没测更糟。
    std::string text_log() const override { return t_->text_log(); }
    uint64_t flush_failures() const override { return t_->flush_failures(); }
    std::string port_name() const override { return t_->port; }

private:
    std::shared_ptr<testing::FakeTransport> t_;
};

/// 一个连着桩固件的离线 Arm 会话。析构时自动 close() —— 读线程的生命周期与其绑死。
struct Offline {
    std::shared_ptr<testing::FakeTransport> fake;
    std::unique_ptr<Arm> arm;

    explicit Offline(const std::string& fw = "Litearm1.7.0-7J", int n = 7,
                     std::shared_ptr<const Clock> clock = nullptr) {
        fake = std::make_shared<testing::FakeTransport>("fake", 0.2, fw, n);
        auto shared = fake;
        ArmOptions opts;
        opts.port = "fake";
        // ⚠ 注入假钟之前**先读 `clock.hpp` 开头**: 假钟不会自己前进, 凡"等超时真的
        //   发生"的判据在它下面会**挂死**。它只给"纯比较"类判据用。
        opts.clock = std::move(clock);
        opts.transport_factory = [shared](const std::string&) -> std::unique_ptr<Transport> {
            return std::unique_ptr<Transport>(new SharedFake(shared));
        };
        // 离线用例不该在超时上耗时间: 把窗口压到用例能接受的量级。
        opts.move_timeout = 1.0;
        arm = std::make_unique<Arm>(opts);
        arm->connect();
    }

    ~Offline() {
        if (arm) {
            try {
                arm->close();
            } catch (...) {
            }
        }
    }

    Offline(const Offline&) = delete;
    Offline& operator=(const Offline&) = delete;

    Arm& operator*() { return *arm; }
    Arm* operator->() { return arm.get(); }
    testing::FakeTransport& t() { return *fake; }
};

/// 一个**还没连**的 Arm (共享同一份 FakeTransport 注入) —— 给"装配路径本身"的用例用。
struct Unconnected {
    std::shared_ptr<testing::FakeTransport> fake;
    std::unique_ptr<Arm> arm;

    explicit Unconnected(const std::string& fw = "Litearm1.7.0-7J", int n = 7,
                         double move_timeout = 1.0) {
        fake = std::make_shared<testing::FakeTransport>("fake", 0.2, fw, n);
        auto shared = fake;
        ArmOptions opts;
        opts.port = "fake";
        opts.transport_factory = [shared](const std::string&) -> std::unique_ptr<Transport> {
            return std::unique_ptr<Transport>(new SharedFake(shared));
        };
        opts.move_timeout = move_timeout;
        arm = std::make_unique<Arm>(opts);
    }

    ~Unconnected() {
        if (arm) {
            try {
                arm->close();
            } catch (...) {
            }
        }
    }

    Unconnected(const Unconnected&) = delete;
    Unconnected& operator=(const Unconnected&) = delete;
};


// ---------------------------------------------------------------- 共用小工具

/// 把桩的链路**静下来**: 关掉被动状态流, 再等在途的帧全部落定。
///
/// ⚠ 为什么这是个必需品而不是讲究: 桩的被动流 (100Hz) 与 `connect()` 握手时压进 `resp_`
///   的那几帧, 同用例刚下发的命令是**两条独立的线**, 谁先被读线程投递没有保证。
///   凡是判据里带 "`status_seq` 前进" 的入口 (`get_status_now` / `read_status` / `arrive`),
///   只要在 `seq0` 采样与首次判据之间有帧被投递, 判据就**当场成立** —— 哪怕那一帧跟本命令
///   毫无关系。要精确控制"命令之后才该有的那一帧", 就必须先把这些在途帧排干。
///
/// ⚠ 两个动作缺一不可: 只关流不停等 ⇒ `connect()` 的残留帧照样会落进窗口 (实测仍有约
///   4% 的翻红率); 只停等不关流 ⇒ 1ms 一拍的被动流会在停等之后**继续**落进窗口。
///   两个都做 ⇒ 实测 150/150 稳定。
inline void quiesce(testing::FakeTransport& t, double settle_s = 0.05) {
    t.auto_status.store(false);
    // ⚠⚠ 判据是"**连续 `settle_s` 没有新帧交付**", **不是**"睡满 `settle_s`"。
    //
    //   后者在机器忙的时候给不出任何保证: 读线程被饿住 ⇒ 关流**之前**生成的残留帧会在
    //   睡完之后才被交付 ⇒ 依赖"链路已经静下来"的用例偶发红。
    //   实测踩过: 一批 `--clean` 全量重建之后的运行里, 用 `quiesce` 的那批用例红了 1 次,
    //   而之后 ~370 次执行一次都复现不出来 —— 低频 + 与时序相关, 正是这个形状。
    //
    //   `status_seq` 是**交付**计数 (桩在 `stamp_status` 里递增) ⇒ "它不动"就等于
    //   "没有新帧被交付", 与"读线程跑得多快"无关。
    uint16_t last = t.status_seq.load();
    double stable_since = now_s();
    const double hard_end = now_s() + 2.0;   // 上界: 读线程真死了也不许把用例挂死
    while (now_s() < hard_end) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        const uint16_t cur = t.status_seq.load();
        if (cur != last) {
            last = cur;
            stable_since = now_s();
            continue;
        }
        if (now_s() - stable_since >= settle_s) return;
    }
}

/// 一条"骨架合法"的状态帧载荷 (7 关节) —— 供协议层用例做逐字段断言。
inline std::vector<uint8_t> make_status_payload(uint16_t flags, uint16_t seq,
                                                const std::vector<double>& q,
                                                uint16_t joint_fault = 0) {
    std::vector<uint8_t> body;
    const auto h = proto::pack_u16le(flags);
    body.insert(body.end(), h.begin(), h.end());
    const auto s = proto::pack_u16le(seq);
    body.insert(body.end(), s.begin(), s.end());
    for (size_t i = 0; i < q.size(); ++i) {
        const auto v = proto::pack_f32s(std::vector<double>{q[i], 0.0, 0.0, 0.0, 0.0});
        body.insert(body.end(), v.begin(), v.end());
        body.push_back(0);
    }
    const auto jf = proto::pack_u16le(joint_fault);
    body.insert(body.end(), jf.begin(), jf.end());
    return body;
}

/// 独立于被测实现的 CRC 参考实现 (纯位循环) —— **判据不能是"拿本函数跟自己对拍"**。
inline uint16_t crc16_reference(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= uint16_t(d[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
        }
    }
    return crc;
}

}  // namespace lt
