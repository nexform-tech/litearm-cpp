#include "litearm/log.hpp"

#include <cstdio>
#include <thread>

#include "litearm/arm.hpp"

namespace litearm {

namespace {
/// 单块 RSP_LOG_DATA 的头长度: u32 total + u32 next + u8 n
constexpr size_t kLogHdr = 9;
/// 轮询已记录量的间隔 (秒)
constexpr double kPollS = 0.05;
}  // namespace

int sample_size(int n_joints) { return 4 + 12 * n_joints; }

std::vector<LogSample> parse_samples(const std::vector<uint8_t>& blob, int n_joints) {
    const int sz = sample_size(n_joints);
    if (sz <= 0) {
        throw InvalidCommandError("关节数非法: " + std::to_string(n_joints));
    }
    if (blob.size() % size_t(sz)) {
        throw TransportError("采集字节数 " + std::to_string(blob.size()) +
                             " 不是单拍 " + std::to_string(sz) +
                             "B 的整数倍 —— 数据截断/错位, 拒绝解析出半截样本");
    }
    std::vector<LogSample> out;
    out.reserve(blob.size() / size_t(sz));
    for (size_t off = 0; off + size_t(sz) <= blob.size(); off += size_t(sz)) {
        LogSample s;
        s.tick = proto::read_u32le(blob, off);
        s.q_ref = proto::unpack_f32s(blob.data(), off + 4, n_joints);
        s.dq = proto::unpack_f32s(blob.data(), off + 4 + 4 * size_t(n_joints), n_joints);
        s.tau = proto::unpack_f32s(blob.data(), off + 4 + 8 * size_t(n_joints), n_joints);
        out.push_back(std::move(s));
    }
    return out;
}

const Clock& LogReader::clk() const {
    return arm_ != nullptr ? arm_->clock() : steady_clock_instance();
}

std::vector<uint8_t> LogReader::read_chunk(uint32_t offset) {
    std::exception_ptr last;
    for (int i = 0; i <= retries_; ++i) {
        try {
            arm_->write_query(proto::CMD_LOG_READ, proto::pack_u32le(offset));
            const auto r = arm_->require().expect(
                proto::RSP_LOG_DATA, timeout_,
                "log_read@" + std::to_string(offset), true, proto::CMD_LOG_READ);
            const auto& p = r.payload;
            if (p.size() < kLogHdr) {
                throw TransportError("RSP_LOG_DATA 帧短 (" + std::to_string(p.size()) + "B)");
            }
            const uint32_t total = proto::read_u32le(p, 0);
            const uint32_t nxt = proto::read_u32le(p, 4);
            const uint8_t n = p[8];
            if (n > LOG_READ_CHUNK || p.size() < kLogHdr + n) {
                throw TransportError("RSP_LOG_DATA 载荷异常: n=" + std::to_string(n) +
                                     ", 实到 " + std::to_string(p.size() - kLogHdr) + "B");
            }
            total_bytes_ = total;
            next_ = nxt;
            return std::vector<uint8_t>(p.begin() + long(kLogHdr),
                                        p.begin() + long(kLogHdr) + n);
        } catch (const MotionTimeoutError& e) {
            // 掉帧: 同一游标重试
            last = std::current_exception();
            continue;
        }
    }
    if (last) std::rethrow_exception(last);
    throw TransportError("log_read 失败");
}

uint32_t LogReader::total() {
    read_chunk(0);
    return total_bytes_;
}

uint32_t LogReader::wait_for(int n_ticks, double timeout, double poll) {
    const int sz = sample_size(arm_->n());
    const uint32_t want = uint32_t(n_ticks) * uint32_t(sz);
    if (want == 0) return 0;
    if (timeout < 0.0) {
        // 按 n_ticks / CTRL_HZ * 1.5 + 3s 推算 (300Hz 逐拍记录)
        timeout = double(n_ticks) / double(CTRL_HZ) * 1.5 + 3.0;
    }
    const double end = clk().now_s() + timeout;
    while (true) {
        const uint32_t tot = total();
        if (tot >= want) return tot;
        if (clk().now_s() >= end) {
            throw MotionTimeoutError("采集未在 " + std::to_string(timeout) +
                                     "s 内录满 " + std::to_string(n_ticks) + " 拍 (只录到 " +
                                     std::to_string(tot / uint32_t(sz)) + " 拍 / " +
                                     std::to_string(tot) + "B) —— 可调大 record_timeout");
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(poll));
    }
}

std::vector<std::vector<uint8_t>> LogReader::iter_chunks() {
    // 必须自检游标是否前进: 固件游标若因异常/干扰回了一个不大于当前 offset 的非零值,
    // 天真的实现会在同一位置无限次重读 (死循环)。这里直接报错。
    std::vector<std::vector<uint8_t>> chunks;
    uint32_t off = 0;
    while (true) {
        auto chunk = read_chunk(off);
        if (!chunk.empty()) chunks.push_back(std::move(chunk));
        if (next_ == 0) return chunks;
        if (next_ <= off) {
            throw TransportError("LOG_READ 游标未前进 (offset=" + std::to_string(off) +
                                 " -> next_byte=" + std::to_string(next_) +
                                 "), 拒绝在同一游标上重复读回 (死循环保护)");
        }
        off = next_;
    }
}

std::vector<uint8_t> LogReader::read_all() {
    std::vector<uint8_t> out;
    for (const auto& chunk : iter_chunks()) {
        out.insert(out.end(), chunk.begin(), chunk.end());
    }
    return out;
}

std::vector<LogSample> LogReader::samples() {
    return parse_samples(read_all(), arm_->n());
}

void ArmLog::start(int n_ticks) {
    if (n_ticks < 0) {
        throw InvalidCommandError("n_ticks 需 >=0 (给的是 " + std::to_string(n_ticks) + ")");
    }
    arm_->cmd_expect_ack(proto::CMD_LOG_CTRL, proto::pack_u32le(uint32_t(n_ticks)),
                         "log_start");
    last_target_ = n_ticks;
}

void ArmLog::stop() {
    arm_->cmd_expect_ack(proto::CMD_LOG_CTRL, proto::pack_u32le(0), "log_stop");
    last_target_ = 0;
}

LogReader ArmLog::reader(double timeout, int retries) {
    return LogReader(arm_, timeout, retries);
}

std::vector<LogSample> ArmLog::capture(int n_ticks, double timeout, int retries,
                                       double record_timeout) {
    start(n_ticks);
    LogReader r = reader(timeout, retries);
    r.wait_for(n_ticks, record_timeout);
    return r.samples();
}

size_t ArmLog::dump(const std::string& path, bool wait, double timeout, int retries,
                    double record_timeout) {
    LogReader r = reader(timeout, retries);
    if (wait && last_target_ > 0) {
        r.wait_for(last_target_, record_timeout);
    }
    const auto blob = r.read_all();
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) throw TransportError("无法写入 " + path);
    const size_t n = std::fwrite(blob.data(), 1, blob.size(), f);
    std::fclose(f);
    return n;
}

}  // namespace litearm
