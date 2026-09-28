#include "litearm/transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>

#include "litearm/errors.hpp"

#ifdef _WIN32
#  include <windows.h>
#else
#  include <dirent.h>
#  include <errno.h>
#  include <fcntl.h>
#  include <poll.h>
#  include <sys/file.h>
#  include <sys/ioctl.h>
#  include <termios.h>
#  include <unistd.h>
#endif

namespace litearm {

namespace {

constexpr uint32_t kCdcVid = 0x1D50;
constexpr uint32_t kCdcPid = 0x606F;

// (半帧窗口的常量 `kPartialMaxS` 已随分帧状态机搬进 `FrameReader` —— 见
//  `include/litearm/framing.hpp`。那里的注释保留了同一条"为什么必须有这个窗口"的理由。)

/// 读超时之后那段"非阻塞补读"的时间上限 (秒)。
/// 超时已到仍要把已经到达的字节读进来 (否则 timeout=0 一个字节都不碰), 但数据持续到达时
/// 不能因此变成没有界的循环 —— 取 20ms。
constexpr double kNonblockExtraS = 0.02;

/// 写超时 —— 必须有界。一次卡住的保活写会长时间占着写锁, 把 emergency_stop()/close()
/// 一起拖住; 而"急停必须永远可达"是本包的安全底线。921600 baud 下 260B 帧约 2.8ms。
constexpr double kWriteTimeoutS = 0.5;

/// 一次最多向串口索取的字节数。
///
/// 由来: 真机空闲时状态流就有 ~14 kB/s (100Hz x 79B), 逐字节 read 是每字节一次
/// syscall。必须按"已经到达的字节数"要 (in_waiting), 第一轮就满足 size => 立即返回、
/// 不等待。
constexpr size_t kReadChunkMax = 4096;

/// 读线程每一拍允许阻塞的时长 —— 真正的读者 (Ack) 用同一档。
constexpr double kReadSliceS = 0.1;

std::mutex g_owners_mu;
std::map<std::string, const Transport*>& owners() {
    static std::map<std::string, const Transport*> m;
    return m;
}

}  // namespace

double now_s() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

void claim_port(const std::string& port, const Transport* owner) {
    std::lock_guard<std::mutex> lk(g_owners_mu);
    auto& m = owners();
    const auto it = m.find(port);
    if (it != m.end() && it->second != nullptr && it->second != owner) {
        throw TransportError(
            "端口 " + port +
            " 已被本进程内另一个传输占用 —— 同一端口同时只能有一条链路 "
            "(跨进程那半由打开时的 flock 保证)。先把它关掉 "
            "(持有者是 Arm 的话是 arm.close() / arm.disconnect()), 或改用另一个端口。");
    }
    m[port] = owner;
}

void release_port(const std::string& port, const Transport* owner) {
    // 只撤掉属于 owner 的那条登记 (不是无条件 erase): 本函数的契约应当是自明的
    // ("只撤自己的")。
    std::lock_guard<std::mutex> lk(g_owners_mu);
    auto& m = owners();
    const auto it = m.find(port);
    if (it != m.end() && it->second == owner) m.erase(it);
}

std::vector<std::pair<std::string, const Transport*>> port_owners_snapshot() {
    std::lock_guard<std::mutex> lk(g_owners_mu);
    std::vector<std::pair<std::string, const Transport*>> out;
    for (const auto& kv : owners()) out.emplace_back(kv.first, kv.second);
    return out;
}

// ===========================================================================
// 端口自动发现
// ===========================================================================
#ifndef _WIN32

namespace {

std::optional<std::string> read_file_trim(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (f == nullptr) return std::nullopt;
    char buf[256] = {0};
    const size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
    std::fclose(f);
    if (n == 0) return std::nullopt;
    std::string s(buf, n);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) {
        s.pop_back();
    }
    return s;
}

/// 从 /sys/class/tty/<name>/device 出发向上找 idVendor/idProduct。
std::optional<std::pair<uint32_t, uint32_t>> usb_ids_of(const std::string& tty_name) {
    const std::string dev = "/sys/class/tty/" + tty_name + "/device";
    char resolved[4096] = {0};
    if (::realpath(dev.c_str(), resolved) == nullptr) return std::nullopt;
    std::string dir(resolved);
    for (int up = 0; up < 6; ++up) {
        const auto vid = read_file_trim(dir + "/idVendor");
        const auto pid = read_file_trim(dir + "/idProduct");
        if (vid && pid) {
            try {
                return std::make_pair(uint32_t(std::stoul(*vid, nullptr, 16)),
                                      uint32_t(std::stoul(*pid, nullptr, 16)));
            } catch (...) {
                return std::nullopt;
            }
        }
        const size_t slash = dir.find_last_of('/');
        if (slash == std::string::npos || slash == 0) break;
        dir = dir.substr(0, slash);
    }
    return std::nullopt;
}

}  // namespace

std::optional<std::string> find_cdc_port() {
#  ifdef __linux__
    DIR* d = ::opendir("/sys/class/tty");
    if (d == nullptr) return std::nullopt;
    std::vector<std::string> names;
    while (dirent* e = ::readdir(d)) {
        const std::string nm = e->d_name;
        if (nm == "." || nm == "..") continue;
        names.push_back(nm);
    }
    ::closedir(d);
    std::sort(names.begin(), names.end());
    for (const auto& nm : names) {
        const auto ids = usb_ids_of(nm);
        if (ids && ids->first == kCdcVid && ids->second == kCdcPid) {
            const std::string path = "/dev/" + nm;
            if (::access(path.c_str(), F_OK) == 0) return path;
        }
    }
    return std::nullopt;
#  else
    // macOS: 没有 sysfs, 逐个探 /dev/tty.usbmodem* 会在没有权限时给出假阳性, 故不猜 ——
    // 让调用方显式给 port。这是刻意的取舍, 不是遗漏。
    return std::nullopt;
#  endif
}

// ===========================================================================
// POSIX 串口
// ===========================================================================
namespace {

int open_posix(const std::string& port) {
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw TransportError("打开串口 " + port + " 失败: " + std::strerror(errno));
    }
    return fd;
}

void apply_termios(int fd, const std::string& port) {
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
        const int e = errno;
        ::close(fd);
        throw TransportError("串口 " + port + " tcgetattr 失败: " + std::strerror(e));
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    // 921600 —— 与 Python 侧同档。
    const speed_t baud = B921600;
    ::cfsetispeed(&tio, baud);
    ::cfsetospeed(&tio, baud);
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        const int e = errno;
        ::close(fd);
        throw TransportError("串口 " + port + " tcsetattr 失败: " + std::strerror(e));
    }
}

}  // namespace

SerialTransport::SerialTransport(const std::string& port, double timeout)
    : port_(port), timeout_(timeout) {
    // 进程内那道门排在 open 之前 (见模块顶部): 那一刻还没有任何副作用。
    claim_port(port_, this);
    try {
        fd_ = open_posix(port_);
        // 跨进程独占: 内核的 flock。它比的是 open file description, 故同进程内两次
        // open 也会互斥。
        if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            const int e = errno;
            ::close(fd_);
            fd_ = -1;
            throw TransportError("串口 " + port + " 已被占用 (flock 失败): " +
                                 std::strerror(e));
        }
        apply_termios(fd_, port_);
    } catch (...) {
        // 那一格进程内登记也要撤 —— 否则这次失败会把端口占死。
        release_port(port_, this);
        throw;
    }
}

SerialTransport::~SerialTransport() { close(); }

void SerialTransport::close() {
    // 幂等: 重复调用是 no-op, 不抛。与在途读/写互斥 —— 否则 os.close(fd) 之后 fd 号被
    // 复用, 落在这个窗口的读者会对已被释放的 fd 号做 termios 配置。
    //
    // 幂等的判据是 closed_ 那一格, 不是"关两次恰好也没事": 上层有多条收尾路径
    // (Arm::close / Arm::disconnect / Arm 析构), 它们全落在这里。
    {
        std::lock_guard<std::mutex> lr(rlock_);
        std::lock_guard<std::mutex> lw(wlock_);
        if (closed_) return;
        closed_ = true;
    }
    stop_flag_.store(true);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    // 放在实例锁外: 它取的是模块级那把锁, 而全仓没有第二处会持着模块锁再去取实例锁
    // => 两把锁不嵌套, 就不必论证"没有谁按相反顺序取它们"。
    release_port(port_, this);
}

bool SerialTransport::is_open() const {
    if (closed_) return false;
    return fd_ >= 0;
}

std::string SerialTransport::text_log() const {
    std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(rlock_));
    // 噪声留痕现在由 `FrameReader` 持有 (两条后端共用同一份) —— 见 framing.hpp 里
    // "存在的理由只有一个: 消灭重复"那段。
    return reader_.noise();
}

void SerialTransport::write_frame(uint8_t cmd, const uint8_t* payload, size_t len) {
    const std::vector<uint8_t> frame = proto::pack_frame(cmd, payload, len);
    std::lock_guard<std::mutex> lk(wlock_);
    if (closed_ || fd_ < 0) throw TransportError("写失败: 链路已关闭");
    const double deadline = now_s() + kWriteTimeoutS;
    size_t off = 0;
    while (off < frame.size()) {
        const double left = deadline - now_s();
        if (left <= 0.0) {
            // write() 抛 => 帧没送达 (残缺帧被固件按 CRC 丢) => 抛 TransportError。
            throw TransportError("写失败: 写超时 (整帧未送达)");
        }
        pollfd p{};
        p.fd = fd_;
        p.events = POLLOUT;
        const int ms = int(std::max(0.0, std::min(left, 0.05)) * 1000.0);
        const int pr = ::poll(&p, 1, ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            throw TransportError(std::string("写失败: ") + std::strerror(errno));
        }
        if (pr == 0) continue;   // 还没可写, 回到上面判截止
        const ssize_t n = ::write(fd_, frame.data() + off, frame.size() - off);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            throw TransportError(std::string("写失败: ") + std::strerror(errno));
        }
        off += size_t(n);
    }
    // flush (tcdrain) 抛 => 整帧已经交进驱动、会被送达, 失败收不回来 => 不算发送失败,
    // 只计进 flush_failures (静默是另一条底线)。
    if (::tcdrain(fd_) != 0) {
        flush_failures_.fetch_add(1);
    }
}

std::optional<std::vector<uint8_t>> SerialTransport::read_chunk(double end_steady) {
    const double remain = std::max(0.0, end_steady - now_s());
    pollfd p{};
    p.fd = fd_;
    p.events = POLLIN;
    const int ms = int(std::min(remain, 0.05) * 1000.0);
    const int pr = ::poll(&p, 1, ms);
    if (pr < 0) {
        if (errno == EINTR) return std::nullopt;
        throw TransportError(std::string("读失败: ") + std::strerror(errno));
    }
    if (pr == 0) return std::nullopt;
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        // 设备被摘掉 / 跳进 ROM bootloader 时 Linux 上的典型形状。
        throw TransportError("读失败: 设备已从总线上消失 (POLLERR/POLLHUP)");
    }
    const size_t want = want_bytes();
    std::vector<uint8_t> chunk(want);
    const ssize_t n = ::read(fd_, chunk.data(), want);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return std::nullopt;
        throw TransportError(std::string("读失败: ") + std::strerror(errno));
    }
    if (n == 0) return std::nullopt;   // select 报就绪而 read 返回空 = 设备断了
    chunk.resize(size_t(n));
    return chunk;
}

size_t SerialTransport::want_bytes() const {
    // 这次 read 该要几个字节 = 已经到达的字节数 (封顶 kReadChunkMax)。
    //   in_waiting > 1 => 按它要 —— 一次把这批取走, 这是本改动的全部收益所在;
    //   否则 (0/1/拿不到) => 要 1: 阻塞等第一个字节。
    // 拿不到就必须退化成 1, 不能抛 —— 退化成 1 只慢不错。
    int n = 0;
    if (::ioctl(fd_, TIOCINQ, &n) != 0 || n <= 1) return 1;
    return std::min<size_t>(size_t(n), kReadChunkMax);
}

std::optional<proto::Frame> SerialTransport::read_frame(double timeout) {
    // timeout=0 是"有就给我", 不是"一个字节都不准碰": 超时已到时仍要把已经到达的字节
    // 非阻塞地读进来, 否则"数据早已躺在驱动缓冲里"这一支永远取不到。
    const double end = now_s() + (timeout >= 0.0 ? timeout : timeout_);
    std::lock_guard<std::mutex> lk(rlock_);
    if (closed_ || fd_ < 0) throw TransportError("读失败: 链路已关闭");
    return read_frame_locked(end);
}

std::optional<proto::Frame> SerialTransport::read_frame_locked(double end) {
    double nb_end = -1.0;   // 超时之后那段非阻塞取字节的截止
    while (true) {
        // ① 装配 —— 分帧状态机在 `FrameReader` 里 (两条后端共用同一份)。
        //    这里传 `nullptr, 0`: 只把**缓冲里已经攒下的**帧取空。一次 `read()` 常带回
        //    20+ 条 153B 状态帧, 所以这一步必须循环取到空为止。
        if (auto f = reader_.feed(nullptr, 0)) return f;
        const double now = now_s();
        // ② 半帧窗口 —— 一个收不齐的假帧头必须在超时后被丢掉, 否则它会永远堵在缓冲头部。
        //    ⚠ 它返回"缓冲有没有被改动": 改动过就要**立刻重试装配**, 因为刚丢掉的假帧头
        //      后面很可能就跟着一条真帧。
        if (reader_.tick(now)) continue;
        if (now >= end) {
            // 窗口已经用尽 —— 但仍要把已经到达的字节读进来。这一步不会等新数据:
            // 传进去的 end 就是此刻 => poll 的超时被夹到 0 => 没数据立刻没结果。
            // 必须有时间上限 (kNonblockExtraS): 数据持续到达时, 光靠"读到凑出一帧"是
            // 没有界的。
            if (nb_end < 0.0) {
                nb_end = now + kNonblockExtraS;
            } else if (now > nb_end) {
                return std::nullopt;
            }
            auto chunk = read_chunk(now);
            if (chunk && !chunk->empty()) {
                // ⚠⚠ **必须接住返回值**: 这一次 `feed` 很可能**当场就装配出一条帧**
                //   (字节是整帧进来的) —— 丢掉它, 下一轮的 `feed(nullptr,0)` 就再也拿不到
                //   (帧已经被取走了), 症状是"链路有数据但一帧都读不出来"。实测踩过。
                if (auto f = reader_.feed(chunk->data(), chunk->size())) return f;
                continue;   // 没凑出整帧 -> 回上面重试装配
            }
            return std::nullopt;
        }
        auto chunk = read_chunk(end);
        if (chunk && !chunk->empty()) {
            // ⚠ 同上: 接住返回值 (这一次 feed 很可能就装配出了帧)
            if (auto f = reader_.feed(chunk->data(), chunk->size())) return f;
        }
    }
}

// ===========================================================================
// Windows 串口 (与本机 Linux 上的 POSIX 后端同契约; 未在 Windows 上实机验证)
// ===========================================================================
#else  // _WIN32

namespace {

std::string win_path(const std::string& port) {
    if (port.rfind("\\\\.\\", 0) == 0) return port;
    return "\\\\.\\" + port;
}

}  // namespace

SerialTransport::SerialTransport(const std::string& port, double timeout)
    : port_(port), timeout_(timeout) {
    claim_port(port_, this);
    const std::string path = win_path(port);
    HANDLE h = ::CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                             OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        release_port(port_, this);
        throw TransportError("打开串口 " + port + " 失败: CreateFile err=" +
                             std::to_string(::GetLastError()));
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(h, &dcb)) {
        ::CloseHandle(h);
        release_port(port_, this);
        throw TransportError("串口 " + port + " GetCommState 失败");
    }
    dcb.BaudRate = 921600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!::SetCommState(h, &dcb)) {
        ::CloseHandle(h);
        release_port(port_, this);
        throw TransportError("串口 " + port + " SetCommState 失败");
    }
    COMMTIMEOUTS tmo{};
    tmo.ReadIntervalTimeout = MAXDWORD;
    tmo.ReadTotalTimeoutMultiplier = 0;
    tmo.ReadTotalTimeoutConstant = 0;
    tmo.WriteTotalTimeoutConstant = DWORD(kWriteTimeoutS * 1000);
    ::SetCommTimeouts(h, &tmo);
    fd_ = int(reinterpret_cast<intptr_t>(h));
}

SerialTransport::~SerialTransport() { close(); }

void SerialTransport::close() {
    {
        std::lock_guard<std::mutex> lr(rlock_);
        std::lock_guard<std::mutex> lw(wlock_);
        if (closed_) return;
        closed_ = true;
    }
    stop_flag_.store(true);
    if (fd_ >= 0) {
        ::CloseHandle(reinterpret_cast<HANDLE>(intptr_t(fd_)));
        fd_ = -1;
    }
    release_port(port_, this);
}

bool SerialTransport::is_open() const { return !closed_ && fd_ >= 0; }

std::string SerialTransport::text_log() const {
    std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(rlock_));
    // 噪声留痕现在由 `FrameReader` 持有 (两条后端共用同一份) —— 见 framing.hpp 里
    // "存在的理由只有一个: 消灭重复"那段。
    return reader_.noise();
}

void SerialTransport::write_frame(uint8_t cmd, const uint8_t* payload, size_t len) {
    const std::vector<uint8_t> frame = proto::pack_frame(cmd, payload, len);
    std::lock_guard<std::mutex> lk(wlock_);
    if (closed_ || fd_ < 0) throw TransportError("写失败: 链路已关闭");
    HANDLE h = reinterpret_cast<HANDLE>(intptr_t(fd_));
    DWORD off = 0;
    while (off < frame.size()) {
        DWORD wrote = 0;
        if (!::WriteFile(h, frame.data() + off, DWORD(frame.size() - off), &wrote,
                         nullptr)) {
            throw TransportError("写失败: WriteFile err=" +
                                 std::to_string(::GetLastError()));
        }
        if (wrote == 0) throw TransportError("写失败: 写超时 (整帧未送达)");
        off += wrote;
    }
    if (!::FlushFileBuffers(h)) flush_failures_.fetch_add(1);
}

std::optional<std::vector<uint8_t>> SerialTransport::read_chunk(double end_steady) {
    const double remain = std::max(0.0, end_steady - now_s());
    if (remain <= 0.0) return std::nullopt;   // 非阻塞档: 交给下面 want_bytes
    std::this_thread::sleep_for(std::chrono::milliseconds(
        long(std::min(remain, 0.05) * 1000.0)));
    return std::nullopt;
}

size_t SerialTransport::want_bytes() const { return 1; }

std::optional<proto::Frame> SerialTransport::read_frame(double timeout) {
    const double end = now_s() + (timeout >= 0.0 ? timeout : timeout_);
    std::lock_guard<std::mutex> lk(rlock_);
    if (closed_ || fd_ < 0) throw TransportError("读失败: 链路已关闭");
    HANDLE h = reinterpret_cast<HANDLE>(intptr_t(fd_));
    for (;;) {
        DWORD errors = 0;
        COMSTAT st{};
        ::ClearCommError(h, &errors, &st);
        if (errors & (CE_FRAME | CE_RXOVER | CE_OVERRUN)) {
            // 线路噪声/溢出: 与 POSIX 侧"丢弃坏帧重扫"同口径, 不抛。
        }
        if (st.cbInQue > 0) {
            const DWORD want = DWORD(std::min<size_t>(want_bytes() == 1 ? st.cbInQue
                                                                       : want_bytes(),
                                                      kReadChunkMax));
            std::vector<uint8_t> chunk(want);
            DWORD got = 0;
            if (!::ReadFile(h, chunk.data(), want, &got, nullptr)) {
                throw TransportError("读失败: ReadFile err=" +
                                     std::to_string(::GetLastError()));
            }
            if (got > 0) {
                chunk.resize(got);
                // ⚠ 同上: 必须接住返回值, 否则整帧会被就地丢掉
                if (auto f = reader_.feed(chunk.data(), chunk.size())) return f;
            }
        }
        // ⚠⚠ 这一段从前是**与 POSIX 侧逐字重复**的一份分帧状态机 (代码里自称"同一段
        //   逻辑")。那是会漂的拷贝, 而本分支在 Linux 上**编不了** ⇒ 漂了也不会当场发现。
        //   现在两边都调 `FrameReader` —— 同样的代码在 POSIX 侧被编译、被测试。
        const double now = now_s();
        if (auto f = reader_.feed(nullptr, 0)) return f;
        if (reader_.tick(now)) continue;   // 丢掉了假帧头 ⇒ 立刻重试装配
        if (now >= end) return std::nullopt;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

std::optional<std::string> find_cdc_port() {
    // 逐个打开 COM1..COM32 读描述符无法可靠拿到 VID:PID (需要 SetupAPI), 故不猜 ——
    // 让调用方显式给 port。与 macOS 分支同一个取舍。
    return std::nullopt;
}

#endif  // _WIN32

}  // namespace litearm
