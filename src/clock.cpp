#include "litearm/clock.hpp"

namespace litearm {

double SteadyClock::now_s() const {
    // ⚠ 与 `litearm::now_s()` (transport.cpp) **逐位一致** —— 后者保留是因为它是公开 API,
    //   而本类是它的"可注入"版本。两处实现必须同口径 (同一个 steady_clock 的
    //   `time_since_epoch`), 否则注入与不注入会给出不同的数值。
    using clk = std::chrono::steady_clock;
    return std::chrono::duration<double>(clk::now().time_since_epoch()).count();
}

const Clock& steady_clock_instance() {
    static const SteadyClock kInstance;
    return kInstance;
}

}  // namespace litearm
