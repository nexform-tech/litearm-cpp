// 串口传输 —— 用**真 pty** (不是桩) 验帧读写/噪声跳过/残缺帧/非阻塞语义/端点独占。
//
// 用真 pty 而不是 FakeTransport: 桩会把 timeout 整个忽略掉, 于是"read_frame(0) 其实是
// 一个字节都不碰"这类缺陷在桩上全绿 (上游就是这么漏掉过一次真机故障的)。
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cstdlib>
#include <thread>

#include "test_support.hpp"

using namespace litearm;

namespace {

/// 一对 pty: 测试持有 master, SerialTransport 打开 slave。
struct Pty {
    int master = -1;
    std::string slave;

    Pty() {
        master = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master < 0) return;
        if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
            ::close(master);
            master = -1;
            return;
        }
        const char* nm = ::ptsname(master);
        if (nm != nullptr) slave = nm;
        // master 侧非阻塞: 测试自己用 poll 控节奏
        const int fl = ::fcntl(master, F_GETFL);
        ::fcntl(master, F_SETFL, fl | O_NONBLOCK);
    }
    ~Pty() {
        if (master >= 0) ::close(master);
    }
    bool ok() const { return master >= 0 && !slave.empty(); }

    void send(const std::vector<uint8_t>& bytes) const {
        size_t off = 0;
        while (off < bytes.size()) {
            const ssize_t n = ::write(master, bytes.data() + off, bytes.size() - off);
            if (n > 0) {
                off += size_t(n);
                continue;
            }
            pollfd p{};
            p.fd = master;
            p.events = POLLOUT;
            ::poll(&p, 1, 100);
        }
    }

    std::vector<uint8_t> drain(int timeout_ms = 200) const {
        std::vector<uint8_t> out;
        const double end = now_s() + double(timeout_ms) / 1000.0;
        while (now_s() < end) {
            uint8_t buf[512];
            const ssize_t n = ::read(master, buf, sizeof(buf));
            if (n > 0) {
                out.insert(out.end(), buf, buf + n);
                continue;
            }
            pollfd p{};
            p.fd = master;
            p.events = POLLIN;
            if (::poll(&p, 1, 20) <= 0) break;
        }
        return out;
    }
};

/// 每个用例用**独享的端口名** —— 登记表按端口字符串比, 共享会互相挡。
struct TransportFixture {
    Pty pty;
    std::unique_ptr<SerialTransport> tr;

    explicit TransportFixture(const std::string& tag) {
        (void)tag;
        if (!pty.ok()) return;
        tr.reset(new SerialTransport(pty.slave, 0.05));
    }
    ~TransportFixture() {
        if (tr) tr->close();
    }
    bool ok() const { return pty.ok() && tr != nullptr; }
};

/// 没有真 pty 可用时跳过 (受限容器/沙箱)。
bool have_pty() {
    Pty p;
    return p.ok();
}

}  // namespace

TEST(transport_pty_is_available_in_this_environment) {
    // 这条不是"测环境", 是给下面用例的守卫: 没有 pty 时它们会静默通过。
    CHECK(have_pty());
}

TEST(transport_write_frame_puts_the_exact_bytes_on_the_wire) {
    TransportFixture fx("write");
    if (!fx.ok()) return;
    const std::vector<uint8_t> payload{0x01, 0x02, 0x03};
    fx.tr->write_frame(0x10, payload);
    const auto got = fx.pty.drain();
    const auto expect = proto::pack_frame(0x10, payload);
    CHECK(got == expect);
}

TEST(transport_read_frame_returns_a_well_formed_frame) {
    TransportFixture fx("read");
    if (!fx.ok()) return;
    const auto wire = proto::pack_frame(0x48, proto::pack_f32s(std::vector<double>{1, 2, 3, 4, 5, 6}));
    fx.pty.send(wire);
    const auto fr = fx.tr->read_frame(0.5);
    CHECK(fr.has_value());
    if (fr) {
        CHECK_EQ(int(fr->cmd), 0x48);
        CHECK_EQ(fr->payload.size(), size_t(24));
    }
}

TEST(transport_skips_noise_and_remembers_it) {
    TransportFixture fx("noise");
    if (!fx.ok()) return;
    // 固件开机签名不是帧, 会被当噪声丢弃 —— 必须留痕 (last_reset_reason 靠它)。
    const std::string banner = "[litearm-usbcdc] ready (sig_ok=1, iwdg-rst)\n";
    std::vector<uint8_t> wire(banner.begin(), banner.end());
    const auto frame = proto::pack_frame(0x45, std::vector<uint8_t>{0x10});
    wire.insert(wire.end(), frame.begin(), frame.end());
    fx.pty.send(wire);
    const auto fr = fx.tr->read_frame(0.5);
    CHECK(fr.has_value());
    if (fr) CHECK_EQ(int(fr->cmd), 0x45);
    const std::string log = fx.tr->text_log();
    CHECK(log.find("iwdg-rst") != std::string::npos);
}

TEST(transport_drops_crc_corrupt_frames_and_resyncs) {
    TransportFixture fx("crcbad");
    if (!fx.ok()) return;
    auto bad = proto::pack_frame(0x43, std::vector<uint8_t>{1, 2, 3});
    bad.back() ^= 0xFF;   // 破坏 CRC
    const auto good = proto::pack_frame(0x47, std::vector<uint8_t>{9});
    std::vector<uint8_t> wire = bad;
    wire.insert(wire.end(), good.begin(), good.end());
    fx.pty.send(wire);
    const auto fr = fx.tr->read_frame(0.5);
    CHECK(fr.has_value());
    if (fr) {
        CHECK_EQ(int(fr->cmd), 0x47);   // 坏帧被丢, 好帧被扫出来
        CHECK_EQ(int(fr->payload[0]), 9);
    }
    // 坏帧的字节 (含它那个被丢掉的 SOF) 也应留痕
    CHECK(!fx.tr->text_log().empty());
}

TEST(transport_keeps_a_partial_frame_across_calls) {
    // 半帧超时**不该丢帧**: 旧实现在超时时已把 SOF 丢掉, 余下字节会被当新流扫描 ⇒ 整帧丢失。
    TransportFixture fx("partial");
    if (!fx.ok()) return;
    const auto wire = proto::pack_frame(0x44, std::vector<uint8_t>{0x41, 0x42});
    fx.pty.send(std::vector<uint8_t>(wire.begin(), wire.begin() + 3));   // 只发头三个字节
    const auto first = fx.tr->read_frame(0.05);
    CHECK(!first.has_value());   // 还没齐
    fx.pty.send(std::vector<uint8_t>(wire.begin() + 3, wire.end()));
    const auto second = fx.tr->read_frame(0.5);
    CHECK(second.has_value());
    if (second) {
        CHECK_EQ(int(second->cmd), 0x44);
        CHECK_EQ(second->payload.size(), size_t(2));
    }
}

TEST(transport_timeout_zero_still_picks_up_bytes_already_waiting) {
    // timeout=0 是"有就给我", 不是"一个字节都不准碰" —— 后者会让"数据早已躺在驱动缓冲里"
    // 这一支永远取不到 (真机上 poll_cart 会恒返回 None)。
    TransportFixture fx("nbzero");
    if (!fx.ok()) return;
    const auto wire = proto::pack_frame(0x45, std::vector<uint8_t>{0x10});
    fx.pty.send(wire);                       // 确保数据已到达
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const auto fr = fx.tr->read_frame(0.0);
    CHECK(fr.has_value());
    if (fr) CHECK_EQ(int(fr->cmd), 0x45);
}

TEST(transport_read_returns_nothing_when_idle) {
    TransportFixture fx("idle");
    if (!fx.ok()) return;
    const double t0 = now_s();
    const auto fr = fx.tr->read_frame(0.05);
    const double dt = now_s() - t0;
    CHECK(!fr.has_value());
    CHECK(dt >= 0.04);   // 真的等过窗口, 不是立刻返回
    CHECK(dt < 0.5);
}

TEST(transport_discards_a_bogus_frame_header_after_the_partial_window) {
    // 噪声里凑出的假帧头 (0xA5 + 声称长度大于实际数据) 永远收不齐 —— 超过窗口即丢弃重扫,
    // 否则它会把它后面的真帧永久挡住。
    TransportFixture fx("bogus");
    if (!fx.ok()) return;
    const std::vector<uint8_t> bogus{proto::SOF, 0x10, 0xFF};   // 声称 255B 载荷
    fx.pty.send(bogus);
    // 窗口 (0.25s) 内读不到东西
    CHECK(!fx.tr->read_frame(0.05).has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // 窗口过后它被丢掉, 于是后面这条真帧能被扫出来
    const auto good = proto::pack_frame(0x43, std::vector<uint8_t>{7});
    fx.pty.send(good);
    const auto fr = fx.tr->read_frame(0.5);
    CHECK(fr.has_value());
    if (fr) CHECK_EQ(int(fr->cmd), 0x43);
}

TEST(transport_close_is_idempotent) {
    TransportFixture fx("idem");
    if (!fx.ok()) return;
    fx.tr->close();
    CHECK(!fx.tr->is_open());
    CHECK_NOTHROW(fx.tr->close());
    CHECK_NOTHROW(fx.tr->close());
    CHECK(!fx.tr->is_open());
}

TEST(transport_operations_after_close_are_loud) {
    TransportFixture fx("afterclose");
    if (!fx.ok()) return;
    fx.tr->close();
    CHECK_THROWS_AS(fx.tr->write_frame(0x10, nullptr, 0), TransportError);
    CHECK_THROWS_AS(fx.tr->read_frame(0.01), TransportError);
}

TEST(transport_claims_the_port_in_process) {
    // 进程内那道门排在 open 之前 —— 那一刻还没有任何副作用, 且错里能写清"是进程内自己人占用"。
    Pty pty;
    if (!pty.ok()) return;
    {
        SerialTransport a(pty.slave, 0.05);
        bool caught = false;
        std::string what;
        try {
            SerialTransport b(pty.slave, 0.05);
        } catch (const TransportError& e) {
            caught = true;
            what = e.what();
        }
        CHECK(caught);
        // 归因必须指向"进程内占用", 而不是 pyserial 那句 "Resource temporarily unavailable"
        CHECK(what.find("本进程内另一个传输占用") != std::string::npos);
    }
    // 关掉之后端口该被释放 —— 同一个口能被再次打开
    CHECK_NOTHROW(SerialTransport c(pty.slave, 0.05));
}

TEST(transport_release_only_drops_its_own_registration) {
    Pty pty;
    if (!pty.ok()) return;
    SerialTransport a(pty.slave, 0.05);
    const auto snap = port_owners_snapshot();
    bool found = false;
    for (const auto& kv : snap) {
        if (kv.first == pty.slave && kv.second == &a) found = true;
    }
    CHECK(found);
    a.close();
    // close 之后本传输那条登记该消失
    for (const auto& kv : port_owners_snapshot()) {
        CHECK(!(kv.first == pty.slave && kv.second == &a));
    }
}

TEST(transport_failed_open_releases_the_registration) {
    // 打不开的口不该把登记留下 —— 否则这次失败会把端口占死。
    bool caught = false;
    try {
        SerialTransport bad("/dev/litearm-does-not-exist-xyz", 0.05);
    } catch (const TransportError&) {
        caught = true;
    }
    CHECK(caught);
    for (const auto& kv : port_owners_snapshot()) {
        CHECK(kv.first != std::string("/dev/litearm-does-not-exist-xyz"));
    }
}

TEST(transport_find_cdc_port_is_safe_without_hardware) {
    // 本机没有接臂时应当返回空, 而不是抛/崩。接臂时返回 /dev/ttyACM*。
    const auto p = find_cdc_port();
    if (p) {
        CHECK(p->rfind("/dev/", 0) == 0);
    }
}

TEST(transport_serial_is_open_reflects_state) {
    TransportFixture fx("isopen");
    if (!fx.ok()) return;
    CHECK(fx.tr->is_open());
    fx.tr->close();
    CHECK(!fx.tr->is_open());
}

TEST(transport_port_name_is_reported) {
    TransportFixture fx("portname");
    if (!fx.ok()) return;
    CHECK_EQ(fx.tr->port_name(), fx.pty.slave);
}

TEST(transport_flush_failures_start_at_zero) {
    TransportFixture fx("flush");
    if (!fx.ok()) return;
    CHECK_EQ(int(fx.tr->flush_failures()), 0);
    fx.tr->write_frame(0x10, nullptr, 0);
    // 正常路径不该记失败
    CHECK_EQ(int(fx.tr->flush_failures()), 0);
}
