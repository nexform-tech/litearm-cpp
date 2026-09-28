// 300Hz 控制拍采集 —— 固件 CMD_LOG_CTRL(0x2D) / CMD_LOG_READ(0x2E) 的封装。
//
// 固件侧 (hal/log_capture.h): 跑激励轨迹时把每控制拍 (tick / 执行参考 q_ref /
// 执行速度 dq / 电机实测力矩 tau) 追加进 RAM 缓冲, LOG_MAX_SAMPLES=2400 (约 8s@300Hz)
// 记满自停; 上位机用 LOG_READ 按字节游标分块读回 —— 这条"绕开连续 USB 流、按游标续读"
// 的设计正是为规避 CDC 间歇掉帧而做的, 所以本模块的重试/续读语义是必须的, 不是可选优化。
//
// 样本布局 (固件 log_sample_t): u32 tick + q_ref[N] + dq[N] + tau[N], 即 4 + 12N 字节。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "litearm/clock.hpp"
#include "litearm/transport.hpp"

namespace litearm {

class Arm;

/// 固件 LOG_MAX_SAMPLES (hal/log_capture.h) —— 超过会回 ERR{0x2D,0x02}
inline constexpr int LOG_MAX_SAMPLES = 2400;
/// 固件单块读回上限 LOG_READ_CHUNK (usb_cmd.c) —— 9B 头 + n <= 245
inline constexpr int LOG_READ_CHUNK = 245;
/// 固件控制循环频率 (litearm.h 的 LITEARM_CTRL_HZ) —— 采集是逐拍记录的,
/// 故 n 拍需要 n/HZ 秒才录满。用于推算默认等待预算。
inline constexpr int CTRL_HZ = 300;

/// 单拍字节数 = sizeof(u32) + 3 * N * sizeof(f32)。
int sample_size(int n_joints);

/// 一拍采样 (固件 log_sample_t)。
struct LogSample {
    uint32_t tick = 0;
    std::vector<double> q_ref;
    std::vector<double> dq;
    std::vector<double> tau;
};

/// 把原始字节流解析成样本列表; 不是样本整数倍则抛 TransportError。
std::vector<LogSample> parse_samples(const std::vector<uint8_t>& blob, int n_joints);

/// 按固件游标 (next_byte) 分块读回采集缓冲。
///
/// next_byte == 0 表示读完; 掉帧 (应答整块丢失) 时按当前游标重试, 不会丢一段数据。
class LogReader {
public:
    LogReader(Arm* arm, double timeout = 1.0, int retries = 3)
        : arm_(arm), timeout_(timeout), retries_(retries) {}

    /// 之前一次读回的固件总字节数 (读完后可查)。
    uint32_t total_bytes() const { return total_bytes_; }

    /// 查询固件当前已记录的字节数 (一次 LOG_READ 往返, 不保存数据)。
    ///
    /// 采集是 300Hz 逐拍进行的, start(n) 返回时缓冲里还没有数据 —— 等待录满必须靠轮询
    /// 本方法, 不能靠 sleep 猜时间。
    uint32_t total();

    /// 等固件记满 n_ticks 拍; 返回实际总字节数。
    ///
    /// timeout<0 时按 n_ticks/CTRL_HZ*1.5 + 3s 推算。超时抛 MotionTimeoutError 并报出
    /// 实际录到多少拍 —— 绝不返回半截数据冒充成功。
    uint32_t wait_for(int n_ticks, double timeout = -1.0, double poll = 0.05);

    /// 逐块产出原始字节 (读到 `next_byte == 0` 结束), **保留块边界**。
    ///
    /// 必须自检游标是否前进: 固件游标若因异常/干扰回了一个不大于当前 offset 的非零值,
    /// 天真的实现会在同一位置无限次重读 (死循环)。这里直接报错。
    ///
    /// C++ 没有生成器, 故用"块向量"表达上游那个 `Iterator[bytes]` —— 语义相同
    /// (逐块、保边界), 只是**一次性取完**而不是惰性产出。要拼接成一整块用 `read_all()`。
    /// ⚠ 满量程 2400 拍 x 88B ≈ 211 kB, 一次取完完全放得下; 真要流式处理请自己循环
    /// `read_chunk` 那一层 (它是私有的, 故本方法就是公开面里的流式等价物)。
    std::vector<std::vector<uint8_t>> iter_chunks();

    /// 读回整块原始字节 (读到 `next_byte == 0` 结束)。
    std::vector<uint8_t> read_all();

    /// 读回并解析为样本列表。
    std::vector<LogSample> samples();

private:
    std::vector<uint8_t> read_chunk(uint32_t offset);

    /// 时间源 —— 从会话 (`Arm`) 要, 空安全 (理由同 `Ack::clk()`)。
    /// ⚠ 定义在 `log.cpp`, **不能内联在这里**: 本头里 `Arm` 只是前向声明,
    ///   `arm_->clock()` 需要完整类型 (实测: 内联会 `invalid use of incomplete type`)。
    const Clock& clk() const;

    Arm* arm_;
    double timeout_;
    int retries_;
    uint32_t total_bytes_ = 0;
    uint32_t next_ = 0;
};

/// `arm.log()` —— 采集启停 + 读回入口。
class ArmLog {
public:
    explicit ArmLog(Arm* arm) : arm_(arm) {}

    /// 开始记录 n_ticks 拍后自停 (n=0 等效 stop())。
    ///
    /// 固件是逐拍记录的: 本调用返回时缓冲里还没有数据 (300Hz, n 拍要 n/300 秒)。
    /// 要拿完整数据须用 capture() (自动等) 或 dump() (默认等)。
    /// 固件侧 n > LOG_MAX_SAMPLES(2400) 回 ERR{0x2D,0x02}。
    void start(int n_ticks);

    /// 停止/清空记录 (停止后仍可用 reader() 读回已记录部分)。
    void stop();

    LogReader reader(double timeout = 1.0, int retries = 3);

    /// 便捷: 采 n_ticks 拍 -> 等录满 -> 读回 -> 解析成样本列表。
    ///
    /// 固件是 300Hz 逐拍记录的 —— start(n) 返回时缓冲里还没有数据, n_ticks=600 要约 2 秒
    /// 才录满, 故本方法会轮询等待。等不满则抛错, 不返回半截数据。
    std::vector<LogSample> capture(int n_ticks, double timeout = 1.0, int retries = 3,
                                   double record_timeout = -1.0);

    /// 读回原始字节流落盘 (解析交给调用方 / parse_samples)。返回字节数。
    ///
    /// wait=true (默认) 会先等最近一次 start(n) 的 n 拍录满 —— 否则 start() 之后立刻
    /// dump 会落盘一个空/半截文件。要读"此刻已录到多少"就传 wait=false。
    size_t dump(const std::string& path, bool wait = true, double timeout = 1.0,
                int retries = 3, double record_timeout = -1.0);

    /// 最近一次 start(n) 的 n。
    int last_target() const { return last_target_; }

private:
    Arm* arm_;
    int last_target_ = 0;
};

}  // namespace litearm
