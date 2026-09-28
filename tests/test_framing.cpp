// 分帧器 (`FrameReader`) —— 直接单测。
//
// ⚠ 这个文件是**抽取分帧层带来的额外好处**: 从前这段逻辑长在 `SerialTransport` 里、
//   只能经 pty **间接**测; 现在它是独立类, 可以逐条精确地喂字节。
//   (抽取的**主要**理由是消灭 POSIX/Win32 两份重复 —— 见 framing.hpp 开头。)
#include "test_support.hpp"

using namespace litearm;

namespace {
std::vector<uint8_t> one_frame(uint8_t cmd, const std::vector<uint8_t>& payload) {
    return proto::pack_frame(cmd, payload);
}
/// 喂一批字节并**取回那一条帧**。
///
/// ⚠⚠ 注意 `feed` 的返回是**取走**不是"试试看" —— 直接 `r.feed(...)` 丢掉返回值就会
///   把帧弄丢 (实测踩过两次: 一次在 transport 的读循环里, 一次就在这个文件的初版里)。
///   `framing.hpp` 的 `feed` 注释里有完整的"取空"写法。
std::optional<proto::Frame> feed_one(FrameReader& r, const std::vector<uint8_t>& bytes) {
    if (auto f = r.feed(bytes.data(), bytes.size())) return f;
    return r.feed(nullptr, 0);   // 这一批没凑出整帧 —— 也许上一批留了半帧, 这里补上了
}
}  // namespace

TEST(framing_reads_a_single_frame) {
    FrameReader r;
    const auto wire = one_frame(0x48, {0x01, 0x02, 0x03});
    const auto f = feed_one(r, wire);
    CHECK(f.has_value());
    if (f) {
        CHECK_EQ(int(f->cmd), 0x48);
        CHECK(f->payload == std::vector<uint8_t>({0x01, 0x02, 0x03}));
    }
    CHECK_EQ(r.buffered(), size_t(0));   // 恰好一条, 缓冲空了
}

TEST(framing_drains_several_frames_from_one_feed) {
    // ⚠ 这是**必须支持**的形状: 一次 `read()` 常带回 20+ 条 153B 状态帧。
    //   剩下的**留在缓冲里**, 靠后续 `feed(nullptr, 0)` 取出 —— 不是丢掉。
    FrameReader r;
    std::vector<uint8_t> wire;
    for (int i = 0; i < 5; ++i) {
        const auto one = one_frame(0x40, std::vector<uint8_t>(3, uint8_t(i)));
        wire.insert(wire.end(), one.begin(), one.end());
    }
    // 一次喂 5 条 —— `feed` 只**返回第一条**, 其余 4 条留在缓冲里
    if (auto f = r.feed(wire.data(), wire.size())) CHECK_EQ(int(f->payload[0]), 0);
    for (int i = 1; i < 5; ++i) {                 // 再取空剩下的 4 条
        const auto f = r.feed(nullptr, 0);
        CHECK(f.has_value());
        if (f) CHECK_EQ(int(f->payload[0]), i);
    }
    CHECK(!r.feed(nullptr, 0).has_value());   // 取空之后没有了
}

TEST(framing_keeps_a_partial_frame_across_feeds) {
    FrameReader r;
    const auto wire = one_frame(0x48, {0x0A, 0x0B, 0x0C});
    r.feed(wire.data(), 4);                       // 只喂一半
    CHECK(!r.feed(nullptr, 0).has_value());
    CHECK_EQ(r.buffered(), size_t(4));            // 半个帧**留着** (含 SOF)
    CHECK(feed_one(r, std::vector<uint8_t>(wire.begin() + 4, wire.end())).has_value());  // 补齐
}

TEST(framing_skips_noise_and_remembers_it) {
    // 开机横幅就是走这条路的 —— 而它是"板子上跑的是哪一版"的唯一来源, 所以**必须留痕**。
    FrameReader r;
    std::vector<uint8_t> wire{'h', 'e', 'l', 'l', 'o'};
    const auto f = one_frame(0x40, {0x01});
    wire.insert(wire.end(), f.begin(), f.end());
    CHECK(feed_one(r, wire).has_value());
    CHECK_EQ(r.noise(), std::string("hello"));
}

TEST(framing_drops_a_bogus_header_only_after_the_partial_window) {
    // ⚠ 一个"看起来像帧头、却永远收不齐"的残片必须被丢掉, 否则它会**永远堵在缓冲头部**
    //   —— 症状是"链路活着但一帧都收不到"。而在**超时之前**不能丢: 真帧可能正在路上。
    FrameReader r;
    // 声称有 255 字节载荷, 但只给 0 个 —— 永远收不齐
    CHECK(!feed_one(r, {proto::SOF, 0x40, 0xFF}).has_value());

    CHECK(!r.tick(0.0));      // 第一次 tick: 起算, 什么都没丢
    CHECK(!r.tick(0.1));      // 还没到窗口
    CHECK(!r.tick(0.2));      // 还差一点
    CHECK(r.tick(0.3));       // 越过 0.25s 窗口 ⇒ 丢掉假帧头, **返回值说"缓冲变了"**
    CHECK_EQ(r.buffered(), size_t(2));   // SOF 被丢掉, 剩 CMD/LEN 当噪声

    // 丢掉之后**后面的真帧立刻能出来** —— 这正是那个窗口存在的意义
    const auto got = feed_one(r, one_frame(0x41, {0x07}));
    CHECK(got.has_value());
    if (got) CHECK_EQ(int(got->cmd), 0x41);
}

TEST(framing_tick_is_a_no_op_when_there_is_no_partial_frame) {
    FrameReader r;
    CHECK(!r.tick(100.0));
    // ⚠ 要让"一条**完整**帧留在缓冲里"这个形状成立, 必须喂**两条**再只取一条 ——
    //   因为 `feed` 是**取走**语义: 喂一条取一条的话缓冲永远是空的 (实测踩过)。
    std::vector<uint8_t> two;
    for (int i = 0; i < 2; ++i) {
        const auto f = one_frame(0x40, {uint8_t(i)});
        two.insert(two.end(), f.begin(), f.end());
    }
    if (auto f = r.feed(two.data(), two.size())) CHECK_EQ(int(f->payload[0]), 0);
    const size_t left = r.buffered();
    CHECK(left > 0);          // 第二条还躺在缓冲里, 且是**完整**的
    CHECK(!r.tick(100.0));    // ⇒ 不是"在途残帧", tick 不该丢它
    CHECK_EQ(r.buffered(), left);
}

TEST(framing_resyncs_after_a_crc_corrupt_frame) {
    FrameReader r;
    auto bad = one_frame(0x48, {0x01, 0x02});
    bad.back() ^= 0xFF;                       // 打坏 CRC
    const auto good = one_frame(0x43, {0x09});
    std::vector<uint8_t> wire = bad;
    wire.insert(wire.end(), good.begin(), good.end());
    const auto f = feed_one(r, wire);
    CHECK(f.has_value());
    if (f) CHECK_EQ(int(f->cmd), 0x43);       // 坏帧被丢, 真帧出来了
}

TEST(framing_reset_clears_everything) {
    FrameReader r;
    r.feed(reinterpret_cast<const uint8_t*>("xy"), 2);
    CHECK(!feed_one(r, {proto::SOF, 0x40, 0xFF}).has_value());
    CHECK(r.buffered() > 0);
    r.reset();
    CHECK_EQ(r.buffered(), size_t(0));
    CHECK_EQ(r.noise(), std::string(""));
    CHECK(!r.tick(0.0));                      // 计时也清了: 不会立刻丢掉下一段残片
}

TEST(framing_noise_is_capped) {
    // 噪声可能无限多 (坏链路) ⇒ 必须封顶, 否则内存无界增长。
    FrameReader r;
    const std::vector<uint8_t> junk(5000, 0x00);   // 全是不是 SOF 的字节
    CHECK(!feed_one(r, junk).has_value());
    CHECK(r.noise().size() <= 2048);
    CHECK_EQ(r.buffered(), size_t(0));             // 垃圾全被消化掉
}
