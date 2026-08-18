#pragma once
/**
 * @file transport.hpp
 * @brief Pluggable transport layer for litearm v4.
 *
 * Provides pub/sub and query/reply abstractions with two backends:
 * - InProcTransport: in-process queues (zero-dependency, for testing)
 * - ZenohTransport: zenoh-c networking (production use, optional)
 */

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace litearm {

// state 广播 50Hz × 最长 query timeout 300s = 15000 条消息。设 20000 留余量。
// 不足时旧消息被环形丢弃。InProcTransport 与未来 ZenohTransport 共用此常量。
inline constexpr size_t kDefaultFifoDepth = 20000;

// RPC 默认超时（秒）= 5 分钟。zenoh 默认仅约 10s，运动方法（movej/movel/...）
// 可能跑几十秒甚至更久。InProcTransport 的 query() 是同步直接调用（不需 timeout），
// 此常量预留给未来的 ZenohTransport。
inline constexpr double kDefaultQueryTimeoutS = 300.0;

// ── Abstract base classes ───────────────────────────────────────────────────

/// Subscription handle: FIFO, try_recv() non-blocking, drain_latest().
class Sub {
public:
    virtual ~Sub() = default;

    /// Non-blocking receive. Returns bytes or empty optional if queue is empty.
    virtual std::optional<std::string> try_recv() = 0;

    /// Drain to the last message, discarding older ones.
    /// Returns the latest message or empty optional if queue was empty.
    std::optional<std::string> drain_latest();
};

/// Abstract transport: pub/sub + query/reply.
class Transport {
public:
    virtual ~Transport() = default;

    /// Publish payload to topic.
    virtual void pub(const std::string& topic, const std::string& payload) = 0;

    /// Subscribe to topic. Returns a Sub handle.
    virtual std::shared_ptr<Sub> sub(const std::string& topic) = 0;

    /// Send a query and wait for a single reply. Returns reply bytes.
    virtual std::string query(const std::string& topic, const std::string& payload) = 0;

    /// Register a queryable on topic. handler(payload) -> reply_bytes.
    using Handler = std::function<std::string(const std::string&)>;
    virtual void declare_queryable(const std::string& topic, Handler handler) = 0;

    /// Close the transport and release resources.
    virtual void close() {}
};

// ── InProc backend (testing / single-process mode) ──────────────────────────

/// In-process pub/sub + query/reply. Zero-dependency, for testing.
class InProcTransport : public Transport {
public:
    explicit InProcTransport(size_t fifo_depth = kDefaultFifoDepth);
    ~InProcTransport() override;

    void pub(const std::string& topic, const std::string& payload) override;
    std::shared_ptr<Sub> sub(const std::string& topic) override;
    std::string query(const std::string& topic, const std::string& payload) override;
    void declare_queryable(const std::string& topic, Handler handler) override;
    void close() override;

private:
    class InProcSub;

    size_t depth_;
    std::mutex lock_;
    std::unordered_map<std::string, std::vector<std::shared_ptr<InProcSub>>> subs_;
    std::unordered_map<std::string, Handler> queryables_;
    bool closed_ = false;
};

// ── Factory ─────────────────────────────────────────────────────────────────

/// Create a transport by kind: "inproc" (testing) or "zenoh" (production).
/// Zenoh transport requires zenoh-c to be linked.
std::unique_ptr<Transport> make_transport(const std::string& kind = "inproc");

} // namespace litearm
