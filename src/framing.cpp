#include "litearm/framing.hpp"

namespace litearm {

std::optional<proto::Frame> FrameReader::try_assemble() {
    while (!buf_.empty()) {
        if (buf_[0] != proto::SOF) {
            // 不是帧头 ⇒ 当噪声记一笔再丢掉。**必须记**: 固件开机横幅就是走这条路的,
            // 而它是"板子上跑的是哪一版 / 那次是不是看门狗复位"的唯一来源。
            note(buf_[0]);
            buf_.erase(buf_.begin());
            continue;
        }
        if (buf_.size() < 3) return std::nullopt;   // CMD/LEN 还没到齐
        const size_t ln = buf_[2];
        const size_t total = 3 + ln + 2;            // SOF + CMD + LEN + 载荷 + CRC2
        if (buf_.size() < total) return std::nullopt;   // 帧体没齐 —— **保留**(含 SOF)
        auto got = proto::unpack_frame(buf_.data(), total);
        if (got) {
            buf_.erase(buf_.begin(), buf_.begin() + long(total));
            partial_since_ = -1.0;
            return got;
        }
        // CRC 或长度坏 —— 丢掉**这个** SOF 重找, 而不是丢掉整段缓冲:
        // 后面很可能紧跟一条真帧, 而它的 SOF 就在不远处。
        buf_.erase(buf_.begin());
    }
    return std::nullopt;
}

std::optional<proto::Frame> FrameReader::feed(const uint8_t* data, size_t len) {
    if (data != nullptr && len > 0) buf_.insert(buf_.end(), data, data + len);
    return try_assemble();
}

bool FrameReader::tick(double now) {
    bool changed = false;
    // ⚠ 循环: 一次可能连续丢掉多个假帧头 (缓冲里可能积了好几段收不齐的残片)。
    while (!buf_.empty() && buf_[0] == proto::SOF) {
        if (partial_since_ < 0.0) {
            partial_since_ = now;   // 这一拍起算
            break;
        }
        if (now - partial_since_ <= kPartialMaxS) break;   // 还没超时, 再等等
        buf_.erase(buf_.begin());   // 收不齐 ⇒ 假帧头, 丢掉重扫
        partial_since_ = -1.0;
        changed = true;
    }
    // 头部不再是 SOF (或缓冲空了) ⇒ 没有"在途的残帧"这回事
    if (buf_.empty() || buf_[0] != proto::SOF) partial_since_ = -1.0;
    return changed;
}

void FrameReader::reset() {
    buf_.clear();
    noise_.clear();
    partial_since_ = -1.0;
}

void FrameReader::note(uint8_t b) {
    noise_.push_back(static_cast<char>(b));
    while (noise_.size() > kNoiseMax) noise_.pop_front();
}

}  // namespace litearm
