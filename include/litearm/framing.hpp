#pragma once
// 分帧器 (L0) —— 把字节流切成帧。**与平台无关**。
//
// ⚠⚠ **它存在的理由只有一个: 消灭重复。** 从前本仓的 POSIX 与 Win32 两条传输后端
//   **各写过一份**分帧状态机 (SOF 扫描 / 半帧超时 / 噪声捕获), 代码里自己写着
//   "与 POSIX 侧同一段逻辑" —— 那是一份**会漂的拷贝**: 改了一侧忘另一侧, 而 Windows
//   那条分支在 Linux 上**连编都编不了**, 漂了也不会有人当场发现。
//
//   ⇒ 抽成这一个类之后: 新增/修改的都是**被编译、被测试**的代码, 两条后端只剩 I/O。
//
// ⚠ 本文件**不许**出现任何平台相关的头 (`windows.h` / `termios.h` / `<fcntl.h>`) ——
//   一旦出现, 这个类就再也不能在"另一侧"编译, 抽出来的意义当场归零。
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "litearm/protocol.hpp"

namespace litearm {

class FrameReader {
public:
    /// 喂入字节, 返回缓冲里的**第一条**完整帧 (没有则 `nullopt`)。
    ///
    /// ⚠⚠ **返回值必须接住 —— 它不只是"试试看", 它把那条帧取走了。**
    ///   丢掉返回值 = **丢掉那一帧**, 而且症状很隐蔽: 数据明明在流, 却一帧都读不出来。
    ///   实测**在同一个下午踩了两次** (一次在 `SerialTransport` 的读循环里, 一次在用例里)
    ///   ⇒ 这是本 API 最容易踩的坑, 故标准用法是**取空**:
    ///
    /// ```cpp
    /// if (auto f = r.feed(chunk.data(), chunk.size())) return *f;   // 这一批里可能就有整帧
    /// while (auto f = r.feed(nullptr, 0)) { /* 处理 f */ }          // 再取空缓冲
    /// ```
    ///
    /// ⚠ 一次 `read()` 常常带回来 20+ 条 153B 的状态帧 (100Hz 流 + 一读 4096B) ——
    ///   剩下的**留在缓冲里**, 靠后续 `feed(nullptr, 0)` 取出。**是留着不是丢掉**。
    ///
    /// ⚠⚠ **没有时间参数, 而且不该有**: 装配是纯粹的字节操作, 不需要知道"现在几点"。
    ///   半帧窗口那个**唯一**需要时间的判据在 `tick(now)` 里。签名里留一个用不到的
    ///   `now` 会在 `--werror` 下变成编译失败, 而更坏的是它让人以为"喂字节时也在计时"。
    std::optional<proto::Frame> feed(const uint8_t* data, size_t len);

    /// 没有新字节时也要过一遍**半帧窗口**。
    ///
    /// ⚠ 一个"看起来像帧头、却永远收不齐"的残片必须在超时后被丢掉, 否则它会**永远堵在
    ///   缓冲头部**, 后面所有真帧一条都出不来 —— 症状是"链路活着但一帧都收不到"。
    /// ⚠ 读循环**每一拍**都要调它, 包括 `read_frame` 返回空的那几拍。
    ///
    /// 返回**缓冲有没有被改动** (即: 丢掉了假帧头)。改动过 ⇒ 调用方应当**立刻重试装配**
    /// —— 刚被丢掉的那个假帧头后面很可能就跟着一条真帧。
    /// ⚠ 这个返回值不是可选装饰: 少了它, 调用方要么漏掉那一帧 (要等到下一次读字节),
    ///   要么在"tick 什么都没干"时空转。两种都不对。
    bool tick(double now);

    /// 被当作非帧字节跳过的**噪声文本** (固件开机横幅就在里面)。
    ///
    /// ⚠ 与另一套实现的 `take_noise()` 不同, 本函数**不清空**: 本仓的 `text_log()` 是
    ///   只读且被**多次**调用 (`last_reset_reason()` / `banner_version()` 都要读它)。
    std::string noise() const { return std::string(noise_.begin(), noise_.end()); }

    void reset();

    /// 缓冲里还有多少未消化的字节 (诊断用: "帧体未齐"时它就是那个半帧的长度)。
    size_t buffered() const { return buf_.size(); }

private:
    /// 装配循环: 尽量从缓冲头部取出一条完整帧; 取不出返回 `nullopt`。
    std::optional<proto::Frame> try_assemble();
    void note(uint8_t b);

    std::vector<uint8_t> buf_;
    std::deque<char> noise_;
    /// "在途残帧"的起始时刻; `< 0` 表示没有。
    double partial_since_ = -1.0;

    /// 一个收不齐的假帧头最多留多久 (秒)。
    /// ⚠ 它**不是**"帧间隔上限" —— 它只管**假帧头**的清理, 与固件多久发一帧无关。
    static constexpr double kPartialMaxS = 0.25;
    /// 噪声留痕上限 —— 与 `SerialTransport::kTextLogMax` 同口径 (2048)。
    static constexpr size_t kNoiseMax = 2048;
};

}  // namespace litearm
