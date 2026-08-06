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
    explicit InProcTransport(size_t fifo_depth = 16);
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
