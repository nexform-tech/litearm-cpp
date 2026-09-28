// 串口传输 —— USB CDC 帧读写 (含自动发现/自愈)。
//
// 端点独占是两层的, 缺一层都不够 —— Linux 下 CDC 口不独占, 第二个进程 open 成功且分吃
// 同一字节流 (双方拿残帧、静默失败); Windows 反而由 OS 强制独占 => 独占性因平台而异,
// 不能当常量依赖。固件那侧无从提供, 只能主机侧收口:
//
//   * 跨进程: flock(LOCK_EX|LOCK_NB) (POSIX) / 文件句柄本身 (Windows), 失败抛
//     TransportError;
//   * 进程内: claim_port() 的 port -> 持有者 登记表。它与 flock 不是重复的: 它排在
//     open 之前 (无副作用、fail-fast), 且那一刻分得清是"自己人重复开"还是"别的进程
//     占着" —— 等 flock 失败时只剩一句 "Resource temporarily unavailable", 两者无从分辨。
//
// 两层挡的都是端口级的互斥, 与上层开了几个 Arm 无关: 两个 Arm 各指一个口是合法的,
// 要挡的是同一个口被两条链路同时开 —— 不论那两条来自两个 Arm、同一个 Arm 的两次构造,
// 还是绕过 Arm 直接 new 出来的传输。故这道门挂在传输这一层。
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "litearm/framing.hpp"
#include "litearm/protocol.hpp"

namespace litearm {

/// 传输接口 —— 真串口与测试桩都实现它。
///
/// 它不是"内部细节": Arm 只通过这个接口碰链路, 注入点见 ArmOptions::transport_factory。
class Transport {
public:
    virtual ~Transport() = default;

    /// 写一帧。失败抛 TransportError。
    ///
    /// **"抛 TransportError 蕴含整帧未送达"是一条载重不变量**: CartPending::request 靠
    /// "写失败就摘掉自己那个 token"成立, 而那条只在"帧没出去"时才安全 —— 帧其实送到了却
    /// 摘掉 token, 固件随后那条 0x4E 会配给队列里后面那条活 token, 于是报"成功"而实际
    /// 没跑 (假成功)。
    ///
    /// 而两半的物理事实不同: write 抛 => 帧没送达 => 抛 TransportError;
    /// drain (tcdrain/flush) 抛 => 整帧已经交进驱动、会被送达, 失败收不回来
    /// => 不算发送失败, 只计进 flush_failures (静默是另一条底线)。
    virtual void write_frame(uint8_t cmd, const uint8_t* payload, size_t len) = 0;

    /// 便捷重载 —— 载荷直接给 vector 或省略 (空载荷)。它们转发到上面那条虚函数,
    /// 故子类只需实现一个。
    void write_frame(uint8_t cmd) { write_frame(cmd, nullptr, 0); }
    void write_frame(uint8_t cmd, const std::vector<uint8_t>& p) {
        write_frame(cmd, p.data(), p.size());
    }

    /// 读一帧; 超时返回 nullopt。跳过噪声字节, 丢弃 CRC 坏帧。
    ///
    /// 缓冲自持完整流 (含候选帧头 SOF): 读超时时已到达的半帧留在缓冲里, 下次调用续读即可
    /// —— 旧实现超时时已把 SOF 丢掉, 余下字节会被当新流扫描, 导致整帧丢失。
    virtual std::optional<proto::Frame> read_frame(double timeout) = 0;

    virtual void close() = 0;
    virtual bool is_open() const = 0;

    /// 最近被跳过的非帧字节 (解码后), 用于解析固件开机签名。默认空。
    virtual std::string text_log() const { return {}; }

    /// 端口名 (用于错误消息与进程内登记)。默认空。
    virtual std::string port_name() const { return {}; }

    /// flush (tcdrain) 失败的累计次数 —— 只做可观测性, 不改变任何行为。
    ///
    /// ⚠ **虚函数**, 不是可有可无: 测试夹具 (`tests/test_support.hpp` 的 `SharedFake`)
    ///   把真传输**包一层**再交给 `Arm`, 非虚的话 `host_stats().flush_failures` 读到的
    ///   是**包装层自己**那个恒 0 的计数器, 假件注入的值永远传不上来 —— 而"假件与真件
    ///   行为不一致"正是本仓最忌讳的假绿(见 `stamps_of` / `tx_snapshot` 的同款说明)。
    virtual uint64_t flush_failures() const { return flush_failures_.load(); }

protected:
    std::atomic<uint64_t> flush_failures_{0};
};

using Frame = proto::Frame;

/// 按 VID:PID 1d50:606f 自动发现 CDC 端口; 找不到返回 nullopt。
///
/// 不读任何环境变量 —— 脚本层用 LITEARM_PORT 覆盖, 那是脚本的事。
std::optional<std::string> find_cdc_port();

// ---------------------------------------------------------------- 真串口

class SerialTransport : public Transport {
public:
    /// 打开并占用 port。失败 (含被占用) 抛 TransportError。
    explicit SerialTransport(const std::string& port, double timeout = 0.2);
    ~SerialTransport() override;

    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    using Transport::write_frame;   // 别让下面那条 override 把基类的便捷重载藏掉
    void write_frame(uint8_t cmd, const uint8_t* payload, size_t len) override;
    std::optional<proto::Frame> read_frame(double timeout) override;
    void close() override;
    bool is_open() const override;
    std::string text_log() const override;
    std::string port_name() const override { return port_; }

    /// 噪声留痕上限。固件开机签名不是帧, 会被当噪声丢弃; 留痕是为了解析出
    /// 「本次是否 IWDG 复位」。噪声可能无限多, 故必须封顶。
    /// ⚠ 上限的**实现**在 `FrameReader` (`kNoiseMax`, 同值) —— 本常量保留是为了
    ///   兼容"从外面读它"的调用方; 改一处要改两处, 故这里注明。
    static constexpr size_t kTextLogMax = 2048;

private:
    std::optional<proto::Frame> read_frame_locked(double end_steady);
    std::optional<std::vector<uint8_t>> read_chunk(double end_steady);
    size_t want_bytes() const;

    std::string port_;
    double timeout_;
    int fd_ = -1;
    std::mutex rlock_;            // 读者互斥
    std::mutex wlock_;            // 写者互斥
    /// **分帧状态机** —— 与平台无关, 抽在 `litearm/framing.hpp` 里。
    ///
    /// ⚠⚠ 从前 POSIX 与 Win32 两条后端**各写了一份**同样的状态机 (SOF 扫描 / 半帧超时 /
    ///   噪声捕获), 代码里自称"与 POSIX 侧同一段逻辑"。那是**会漂的拷贝**, 而 Windows
    ///   那条分支在 Linux 上编不了 ⇒ 漂了也不会当场发现。抽出之后两条后端只剩 I/O。
    FrameReader reader_;
    bool closed_ = false;
    // 唤醒读线程用 (close 时置位, 让阻塞在 poll 上的读者及时退出)
    std::atomic<bool> stop_flag_{false};
};

/// 单调时钟秒 (对应 Python 的 time.monotonic)。
double now_s();

/// 把 `port` 记到当前传输名下; 已被另一个活着的持有者占着时响亮失败。
/// 排在 open 之前 (那一刻还没有任何副作用, 且错里能直接写清"是进程内自己人占用")。
void claim_port(const std::string& port, const Transport* owner);
void release_port(const std::string& port, const Transport* owner);
/// 进程内登记表快照 (端口名, 持有者) —— 供测试断言不泄漏。
std::vector<std::pair<std::string, const Transport*>> port_owners_snapshot();

}  // namespace litearm
