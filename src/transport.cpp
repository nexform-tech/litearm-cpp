#include "litearm/transport.hpp"
#include <stdexcept>

namespace litearm {

// ── Sub base class ──────────────────────────────────────────────────────────

std::optional<std::string> Sub::drain_latest() {
    std::optional<std::string> latest;
    while (true) {
        auto msg = try_recv();
        if (!msg.has_value()) break;
        latest = std::move(msg);
    }
    return latest;
}

// ── InProcSub ───────────────────────────────────────────────────────────────

class InProcTransport::InProcSub : public Sub {
public:
    explicit InProcSub(size_t max_depth) : max_depth_(max_depth) {}

    std::optional<std::string> try_recv() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return std::nullopt;
        std::string msg = std::move(queue_.front());
        queue_.pop_front();
        return msg;
    }

    void push(const std::string& payload) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(payload);
        while (queue_.size() > max_depth_) {
            queue_.pop_front();
        }
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

private:
    size_t max_depth_;
    std::mutex mutex_;
    std::deque<std::string> queue_;
};

// ── InProcTransport ─────────────────────────────────────────────────────────

InProcTransport::InProcTransport(size_t fifo_depth)
    : depth_(fifo_depth) {}

InProcTransport::~InProcTransport() {
    close();
}

void InProcTransport::pub(const std::string& topic, const std::string& payload) {
    std::lock_guard<std::mutex> lock(lock_);
    if (closed_) return;
    auto it = subs_.find(topic);
    if (it == subs_.end()) return;
    for (auto& sub : it->second) {
        sub->push(payload);
    }
}

std::shared_ptr<Sub> InProcTransport::sub(const std::string& topic) {
    std::lock_guard<std::mutex> lock(lock_);
    auto sub = std::make_shared<InProcSub>(depth_);
    subs_[topic].push_back(sub);
    return sub;
}

std::string InProcTransport::query(const std::string& topic, const std::string& payload) {
    std::lock_guard<std::mutex> lock(lock_);
    auto it = queryables_.find(topic);
    if (it == queryables_.end()) {
        throw std::runtime_error("No queryable registered for topic: " + topic);
    }
    // Call handler while holding the lock (matches Python behavior for tests)
    return it->second(payload);
}

void InProcTransport::declare_queryable(const std::string& topic, Handler handler) {
    std::lock_guard<std::mutex> lock(lock_);
    if (queryables_.count(topic)) {
        throw std::runtime_error("Queryable already registered for topic: " + topic);
    }
    queryables_[topic] = std::move(handler);
}

void InProcTransport::close() {
    std::lock_guard<std::mutex> lock(lock_);
    if (closed_) return;
    closed_ = true;
    for (auto& [topic, subs] : subs_) {
        for (auto& sub : subs) {
            sub->clear();
        }
    }
    subs_.clear();
    queryables_.clear();
}

// ── Factory ─────────────────────────────────────────────────────────────────

std::unique_ptr<Transport> make_transport(const std::string& kind) {
    if (kind == "inproc") {
        return std::make_unique<InProcTransport>();
    }
    throw std::runtime_error(
        "Unknown transport kind: " + kind +
        ". Available: 'inproc'. Zenoh transport requires zenoh-c (not linked).");
}

} // namespace litearm
