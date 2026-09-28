#include "litearm/diagnostics.hpp"

#include <cctype>
#include <sstream>

#include "litearm/arm.hpp"

namespace litearm {

namespace {

/// 固件 kin_bench_run 产出的计时段名 (长名在前, 避免前缀误配)。
///
/// ⚠ 固件用 txt(名字) + u32_cat(n) 拼接, 而 u32_cat 只在数字之后补空格 —— 所以名字与第一个
/// 数字是相连的 (`FK200 108 240 ` / `LOOP14000 `), 不能用「名字 + 空白 + 数字」式正则去切。
const char* const kBlockNames[] = {"LOOP1", "RNEA", "JAC", "IK", "FK", "SC", "G", "M", "LAW"};

bool ends_with_digits_only(const std::string& s) {
    if (s.empty()) return false;
    for (char c : s) {
        if (!std::isdigit(static_cast<unsigned char>(c)) && c != ' ') return false;
    }
    return true;
}

/// 尝试把 `line` 解析成 `<块名><数字序列>`; 成功返回 true 并填 name/values。
bool parse_timing(const std::string& line, std::string* name,
                  std::vector<long long>* values) {
    for (const char* blk : kBlockNames) {
        const std::string b(blk);
        if (line.rfind(b, 0) != 0) continue;
        const std::string rest = line.substr(b.size());
        if (!ends_with_digits_only(rest)) continue;
        std::vector<long long> vals;
        std::istringstream is(rest);
        long long v = 0;
        while (is >> v) vals.push_back(v);
        if (vals.empty()) continue;
        *name = b;
        *values = vals;
        return true;
    }
    return false;
}

/// key=value 对。
/// ⚠ 键名可以带数字 —— rxl0 / rxl1 就是 (它们正是"RX FIFO 溢出 = 静默丢反馈 = 可能整臂
/// EMERGENCY"的那两个计数器)。从前写的是 [a-z_]+, 匹配不到带数字的键, 于是那两个静默丢失。
void parse_kv(const std::string& body, std::map<std::string, long long>* out) {
    size_t i = 0;
    while (i < body.size()) {
        // 键: [a-z_][a-z0-9_]*
        const size_t start = i;
        if (!(std::islower(static_cast<unsigned char>(body[i])) || body[i] == '_')) {
            ++i;
            continue;
        }
        while (i < body.size() &&
               (std::islower(static_cast<unsigned char>(body[i])) ||
                std::isdigit(static_cast<unsigned char>(body[i])) || body[i] == '_')) {
            ++i;
        }
        const std::string key = body.substr(start, i - start);
        if (i >= body.size() || body[i] != '=') continue;
        ++i;   // skip '='
        const size_t dstart = i;
        while (i < body.size() && std::isdigit(static_cast<unsigned char>(body[i]))) ++i;
        if (i == dstart) continue;
        try {
            (*out)[key] = std::stoll(body.substr(dstart, i - dstart));
        } catch (...) {
        }
    }
}

/// `flt=<锁存拍> /<原因码>` —— 数字与 '/' 之间有一个尾随空格 (u32_cat 语义所致)
void parse_flt(const std::string& body, std::map<std::string, long long>* out,
               std::string* stripped) {
    const size_t pos = body.find("flt=");
    if (pos == std::string::npos) {
        *stripped = body;
        return;
    }
    size_t i = pos + 4;
    const size_t d0 = i;
    while (i < body.size() && std::isdigit(static_cast<unsigned char>(body[i]))) ++i;
    if (i == d0) {
        *stripped = body;
        return;
    }
    const long long tick = std::stoll(body.substr(d0, i - d0));
    while (i < body.size() && (body[i] == ' ' || body[i] == '/')) ++i;
    const size_t d1 = i;
    while (i < body.size() && std::isdigit(static_cast<unsigned char>(body[i]))) ++i;
    if (i == d1) {
        *stripped = body;
        return;
    }
    const long long cause = std::stoll(body.substr(d1, i - d1));
    (*out)["fault_tick"] = tick;
    (*out)["fault_cause"] = cause;
    std::string s = body;
    s.erase(pos, i - pos);
    *stripped = s;
}

std::vector<std::string> split_lines(const std::string& text) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : text) {
        if (c == '\n') {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r') {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

long long get(const std::map<std::string, long long>& m, const char* k) {
    const auto it = m.find(k);
    return it == m.end() ? 0 : it->second;
}

}  // namespace

long long KinBenchResult::crc_errors() const { return get(link, "crc"); }
long long KinBenchResult::reply_dropped() const { return get(link, "rfd"); }
long long KinBenchResult::can_tx_fail() const { return get(link, "txf"); }
long long KinBenchResult::loop_max_kcycle() const { return get(link, "loop_k"); }
long long KinBenchResult::loop_overruns() const { return get(link, "ovr"); }
long long KinBenchResult::rx_fifo_lost_motor() const { return get(link, "rxl0"); }
long long KinBenchResult::rx_fifo_lost_bridge() const { return get(link, "rxl1"); }
long long KinBenchResult::gsusb_ring_drops() const { return get(link, "rbd"); }

KinBenchResult parse_kin_bench(const std::string& text) {
    if (trim(text).empty()) {
        // 空文本 = 异常回执, 拒绝当成全 0 返回。
        throw TransportError("KIN_BENCH 回执为空 —— 非正常响应, 拒绝当成全 0 返回");
    }
    KinBenchResult out;
    out.raw = text;
    for (const std::string& raw_line : split_lines(text)) {
        const std::string line = trim(raw_line);
        if (line.empty()) continue;
        if (line.rfind("LINK", 0) == 0) {
            std::string body = line.substr(4);
            std::string stripped;
            parse_flt(body, &out.link, &stripped);
            std::map<std::string, long long> kv;
            parse_kv(stripped, &kv);
            for (const auto& item : kv) {
                if (item.first == "st") {
                    // 语义化的布尔: 1 = 从 flash 装载
                    out.link["store_from_flash"] = item.second ? 1 : 0;
                } else {
                    out.link[item.first] = item.second;
                }
            }
            continue;
        }
        std::string name;
        std::vector<long long> vals;
        if (parse_timing(line, &name, &vals)) {
            out.timings[name] = vals;
        }
    }
    return out;
}

Msg<KinBenchResult> Diagnostics::kin_bench(double timeout) {
    Arm* arm = arm_;
    arm->write_query(proto::CMD_KIN_BENCH);   // 查询/自检类: 不受零重力守卫
    // 第 1 帧 (各项耗时) —— 等满 timeout
    const auto r1 = arm->require().expect(proto::RSP_KIN_BENCH, timeout, "kin_bench", true,
                                          proto::CMD_KIN_BENCH);
    std::string all(r1.payload.begin(), r1.payload.end());
    // 第 2 帧 (LINK 诊断行) —— 只等一个短窗口: 新固件它紧跟第 1 帧到, 旧固件不发。
    // 超时不是错误 (旧固件只有一帧, 那是兼容情形); 但也不能因为"拿不到就算了"而把窗口设
    // 成 0 —— 那等于没改, 又回到静默 0。
    for (int i = 1; i < KIN_BENCH_FRAMES; ++i) {
        try {
            const auto r2 = arm->require().expect(proto::RSP_KIN_BENCH, KIN_BENCH_MORE_S,
                                                  "kin_bench(LINK 帧)", false,
                                                  proto::CMD_KIN_BENCH);
            all.append(r2.payload.begin(), r2.payload.end());
        } catch (const MotionTimeoutError&) {
            break;
        } catch (const TransportError&) {
            break;
        }
    }
    return arm->wrap(parse_kin_bench(all), proto::RSP_KIN_BENCH);
}

}  // namespace litearm
