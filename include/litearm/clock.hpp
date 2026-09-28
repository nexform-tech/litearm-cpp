#pragma once
// 可注入时钟 —— 让"超时 / 到期 / 时间戳"这类判据**可被测试控制**。
//
// ⚠⚠ **本仓的时钟只管"现在几点", 不管"怎么等"** —— 这是与另一套实现 (Gitee
//   `yudao_hz_1/litearm-cpp`) 的一处**刻意不同**, 理由写在这里, 因为它是承重的:
//
//   那套的 `Clock` 还有一个 `sleep_until(t)` (假钟 = 把虚拟时间**跳到** t 并立刻返回)。
//   本仓**不提供**它, 因为本仓的等待**清一色**是 `condition_variable::wait_for` ——
//   那些等待靠的是**有帧到达时被 notify 立刻唤醒**, 而不是"睡够时间"。若把睡眠也搬进
//   时钟, 就会丢掉"立刻唤醒"这条 (每次等待都要等满一整片), 得不偿失。
//
//   ⇒ 本仓的语义是:
//     * **deadline / 到期比较 / 时间戳** 走注入的钟 (`Clock::now_s()`);
//     * **等待** 走条件变量的**真实**时间 (带一个固定的真实分片)。
//
//   ⚠⚠ **由此带来一条必须知道的后果**: 注入一个**不会自己前进**的假钟时,
//     `now + timeout` 这类 deadline **永远到不了** ⇒ 凡"等超时真的发生"的判据会**挂死**
//     (不是失败, 是挂死)。所以:
//       * 假钟只给"**纯比较**"类判据用 —— 到期比较、时间戳、TTL 水位, 那些**不等待**;
//       * 凡"超时真的发生"的判据 (ACK 超时 / 写超时 / 到位超时) **必须用真钟**
//         (`SteadyClock` 或默认构造), 或由用例主动 `FakeClock::advance()` 推时间。
//     这条与另一套实现的结论**一致** (它的注释也写"凡'超时真的发生'的判据要用 SteadyClock"),
//     只是本仓因为不提供 `sleep_until`, 后果更硬: 那边是"退化成 skip", 这边是"挂死"。
#include <chrono>
#include <memory>

namespace litearm {

/// 时间源。单位是**秒 (double)** —— 与 `now_s()` 同口径, 于是仓里所有
/// `end = now + timeout` 形式的表达式**一个都不用改**。
class Clock {
public:
    virtual ~Clock() = default;
    /// 单调递增的"现在", 单位秒。⚠ 契约: **单调不减**。
    virtual double now_s() const = 0;
};

/// 生产用时钟 (steady_clock, 与从前的 `now_s()` 逐位一致)。
class SteadyClock : public Clock {
public:
    double now_s() const override;
};

/// 进程级默认时钟的**唯一**实例 —— 给那些拿不到会话对象的调用点兜底
/// (例如不注入时钟时直接 `new SerialTransport(...)`)。
///
/// ⚠ 它是**共享且只读**的, 没有 setter: 要换时间源就**显式注入** (`ArmOptions::clock`),
///   别做进程级可变全局 —— 一个进程里两个 `Arm` 想要两个不同时钟时, 全局会静默串味。
const Clock& steady_clock_instance();

}  // namespace litearm
