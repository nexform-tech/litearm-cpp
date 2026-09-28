// 协议漂移防护 —— SDK 与固件头文件的**双向强制比对**。
//
// 为什么必须有这个文件: 上一次栽的跟头就是「命令/帧布局改了, SDK 没跟上」——
// 固件 1.5.x 把状态帧从 `4+21N` 改成 `6+21N` 时漏同步 SDK, 而桩测试整替换 transport、
// 自造旧布局, 结果**离线全绿、真机必挂**。人工核对挡不住这种事, 只有把固件头文件当输入
// 喂给测试才挡得住。
//
// 做法: 直接解析**固件仓库的源码** (不依赖固件编译), 断言固件与 SDK 的命令/应答常量
// 双向一致、入口无死角、状态帧布局未变、能力判定假设仍成立、版本门满足。
//
// 固件仓库位置: 环境变量 `LITEARM_FW_DIR`, 默认 `~/litearm-stm32`。
// **找不到时 SKIP, 不是 pass** —— 假绿比没测更糟 (上游同款口径)。
//
// ⚠ **SDK 侧用"解析 protocol.hpp"代替 Python 的 `dir(P)`**: C++ 没有反射, 而
//   `inline constexpr uint8_t CMD_X = 0xNN;` 与固件的 `#define CMD_X 0xNN` 是**同一种
//   东西** —— 两边都按**源码**对表, 反而是最对称的口径。副作用: 与 `test_full_coverage.cpp`
//   的"公开面哨兵"同一种技术 (那儿扫的是 `arm.hpp`)。
//
// ⚠ **"入口可调用"那半条移到了动态检查**: 上游用 `getattr(arm, "Arm.params.set_joint_param")`
//   验证入口摸得到。C++ 没有这个名字解析, 而 `test_full_coverage.cpp` 的
//   `coverage_every_implemented_command_is_exercised_through_its_entry` 做的是**更强**的
//   事 —— 它把每个入口**真调一遍**并断言命令真的发出去了。故这里只做"命令集合双向一致"。
#include <cstdlib>
#include <map>
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "test_support.hpp"

using namespace litearm;

namespace {

// ---------------------------------------------------------------- 固件源码

std::string fw_dir() {
    const char* env = std::getenv("LITEARM_FW_DIR");
    if (env != nullptr && *env != '\0') return env;
    const char* home = std::getenv("HOME");
    if (home != nullptr && *home != '\0') return std::string(home) + "/litearm-stm32";
    return "litearm-stm32";
}

/// 读固件树里的一个文件; 不在则 SKIP (不是失败, 但**必须看得见**)。
std::string fw_read(const std::string& rel) {
    const std::string full = fw_dir() + "/" + rel;
    FILE* f = std::fopen(full.c_str(), "rb");
    if (f == nullptr) {
        SKIP("固件源码不在 " + full +
             " —— 设 LITEARM_FW_DIR 指向 litearm-stm32 仓库 (找不到时 SKIP, 不是 pass: "
             "假绿比没测更糟)");
    }
    std::string out;
    char buf[8192];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

const char* kUsbCmdH = "User/litearm/hal/usb_cmd.h";
const char* kUsbCmdC = "User/litearm/hal/usb_cmd.c";
const char* kLitearmH = "User/litearm/litearm.h";
const char* kJointCfgH = "User/litearm/params/joint_cfg.h";

// ---------------------------------------------------------------- 解析工具

std::vector<std::string> split_lines(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == '\n') {
            out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

/// `#define <prefix>NAME 0xNN <行尾>` -> {名字: (值, 行尾注释/内容)}。
///
/// ⚠ **逐行匹配, 不跨行** —— 上游那条注释记着这个坑: 用 `\s` 会跨越换行, 于是 `(.*)$`
/// 会把**下一行**整行吞掉, 使相邻的 `#define` 交替漏匹配 (当年 34 条只命中 26 条)。
/// 逐行做天然不会有这个问题。
std::map<std::string, std::pair<int, std::string>> fw_defines(const std::string& src,
                                                              const std::string& prefix,
                                                              size_t min_expected) {
    std::map<std::string, std::pair<int, std::string>> out;
    const std::regex re("^#[ \\t]*define[ \\t]+(" + prefix +
                        "[A-Za-z0-9_]+)[ \\t]+(0x[0-9A-Fa-f]+)[ \\t]*(.*)$");
    for (const auto& line : split_lines(src)) {
        std::smatch m;
        if (!std::regex_match(line, m, re)) continue;
        out[m[1].str()] = {int(std::stol(m[2].str(), nullptr, 16)), m[3].str()};
    }
    // ⚠ **防空转**: 一条都没解析出来时, 所有"遍历固件常量"的判据都会**静默通过** ——
    //   那正是本文件要消灭的"假绿" (上游的 Python 版有同一个洞)。
    //   下界取保守值: 固件**增加**命令只会让条数变多, 所以下界不会误伤; 而解析器一旦被
    //   改坏 (或固件头的写法变了), 条数会塌到 0, 这里当场红。
    if (out.size() < min_expected) {
        FAIL("固件头文件的 `" + prefix + "` 常量只解析出 " + std::to_string(out.size()) +
             " 条 (下界 " + std::to_string(min_expected) +
             ") —— 固件头格式可能变了, 或者解析器坏了; 后面那些判据会**静默失效**");
    }
    return out;
}

/// 剥掉 `/* */` 与 `//` 注释 (SDK 头文件里注释密度很高, 不剥会把注释里的名字当常量)。
std::string strip_comments(const std::string& raw) {
    std::string src;
    src.reserve(raw.size());
    for (size_t i = 0; i < raw.size();) {
        if (raw[i] == '/' && i + 1 < raw.size() && raw[i + 1] == '*') {
            const size_t e = raw.find("*/", i + 2);
            i = (e == std::string::npos) ? raw.size() : e + 2;
            src.push_back(' ');
        } else if (raw[i] == '/' && i + 1 < raw.size() && raw[i + 1] == '/') {
            const size_t e = raw.find('\n', i);
            i = (e == std::string::npos) ? raw.size() : e;
            src.push_back(' ');
        } else {
            src.push_back(raw[i++]);
        }
    }
    return src;
}

/// SDK 侧: `inline constexpr uint8_t CMD_X = 0xNN;` -> {名字: 值}。
/// 这就是 Python 那边 `dir(_protocol)` 的 C++ 等价物 (见文件顶部说明)。
std::map<std::string, int> sdk_defines(const std::string& prefix) {
    const std::string src = strip_comments(lt::read_repo_file("include/litearm/protocol.hpp"));
    std::map<std::string, int> out;
    const std::regex re("\\b(" + prefix + "[A-Za-z0-9_]+)\\s*=\\s*(0x[0-9A-Fa-f]+)");
    for (auto it = std::sregex_iterator(src.begin(), src.end(), re);
         it != std::sregex_iterator(); ++it) {
        out[(*it)[1].str()] = int(std::stol((*it)[2].str(), nullptr, 16));
    }
    return out;
}

/// 取出 `static void <name>(void) { ... }` 的函数体 (把解析限定在单个函数内)。
/// C 源码里用花括号配对来收尾, 比正则的非贪婪更稳。
std::string fn_body(const std::string& src, const std::string& name) {
    const std::string key = "static void " + name + "(void)";
    size_t p = src.find(key);
    if (p == std::string::npos) {
        // 容忍 `static void\n<name>(void)`
        const std::regex re("static[ \\t]+void[ \\t]+" + name + "[ \\t]*\\([ \\t]*void[ \\t]*\\)");
        std::smatch m;
        if (!std::regex_search(src, m, re)) return {};
        p = size_t(m.position());
    }
    const size_t brace = src.find('{', p);
    if (brace == std::string::npos) return {};
    int depth = 1;
    size_t i = brace + 1;
    while (i < src.size() && depth > 0) {
        if (src[i] == '{') ++depth;
        else if (src[i] == '}') --depth;
        ++i;
    }
    return src.substr(brace + 1, i > brace + 1 ? i - brace - 2 : 0);
}

std::string collapse_spaces(const std::string& s) {
    std::string out;
    bool prev_space = false;
    for (char c : s) {
        const bool is_sp = (c == ' ' || c == '\t' || c == '\n' || c == '\r');
        if (is_sp) {
            if (!prev_space && !out.empty()) out.push_back(' ');
        } else {
            out.push_back(c);
        }
        prev_space = is_sp;
    }
    while (!out.empty() && out.back() == ' ') out.pop_back();
    return out;
}

std::map<std::string, std::pair<int, std::string>> unimplemented_map() {
    std::map<std::string, std::pair<int, std::string>> out;
    for (const auto& kv : proto::unimplemented_cmds()) {
        out[kv.second] = {int(kv.first), ""};
    }
    return out;
}

std::set<std::string> values_of(const std::map<uint8_t, std::string>& m) {
    std::set<std::string> out;
    for (const auto& kv : m) out.insert(kv.second);
    return out;
}

}  // namespace

// ---------------------------------------------------------------- 1. 常量双向

TEST(sync_every_firmware_cmd_has_sdk_constant) {
    // 固件每条已实现命令都要在 SDK 里有**同名同值**常量。
    const auto fw = fw_defines(fw_read(kUsbCmdH), "CMD_", 40);
    const auto sdk = sdk_defines("CMD_");
    std::set<std::string> exempt = values_of(proto::unimplemented_cmds());
    for (const auto& n : values_of(proto::firmware_only_cmds())) exempt.insert(n);
    for (const auto& n : values_of(proto::preexisting_gaps())) exempt.insert(n);

    std::vector<std::string> missing;
    for (const auto& kv : fw) {
        if (exempt.count(kv.first)) continue;
        const auto it = sdk.find(kv.first);
        char buf[160];
        if (it == sdk.end()) {
            std::snprintf(buf, sizeof(buf), "%s=0x%02X (SDK: 没有)", kv.first.c_str(),
                          kv.second.first);
        } else if (it->second != kv.second.first) {
            std::snprintf(buf, sizeof(buf), "%s=0x%02X (SDK: 0x%02X)", kv.first.c_str(),
                          kv.second.first, it->second);
        } else {
            continue;
        }
        missing.push_back(buf);
    }
    if (!missing.empty()) {
        std::string msg = "SDK protocol.hpp 缺少/值不符的固件命令:";
        for (const auto& m : missing) msg += " " + m;
        FAIL(msg);
    }
}

TEST(sync_sdk_has_no_command_the_firmware_lacks) {
    // 反向: SDK 不许声明固件没有的命令 (漂移)。
    const auto fw = fw_defines(fw_read(kUsbCmdH), "CMD_", 40);
    const auto sdk = sdk_defines("CMD_");
    std::string extra;
    for (const auto& kv : sdk) {
        if (fw.count(kv.first) == 0) extra += " " + kv.first;
    }
    if (!extra.empty()) FAIL("SDK 声明了固件没有的命令 (漂移):" + extra);
}

TEST(sync_every_firmware_rsp_has_sdk_constant) {
    const auto fw = fw_defines(fw_read(kUsbCmdH), "RSP_", 15);
    const auto sdk = sdk_defines("RSP_");
    const auto exempt = values_of(proto::firmware_only_rsps());
    std::vector<std::string> missing;
    for (const auto& kv : fw) {
        if (exempt.count(kv.first)) continue;
        const auto it = sdk.find(kv.first);
        char buf[160];
        if (it == sdk.end()) {
            std::snprintf(buf, sizeof(buf), "%s=0x%02X (SDK: 没有)", kv.first.c_str(),
                          kv.second.first);
        } else if (it->second != kv.second.first) {
            std::snprintf(buf, sizeof(buf), "%s=0x%02X (SDK: 0x%02X)", kv.first.c_str(),
                          kv.second.first, it->second);
        } else {
            continue;
        }
        missing.push_back(buf);
    }
    if (!missing.empty()) {
        std::string msg = "SDK protocol.hpp 缺少/值不符的固件应答:";
        for (const auto& m : missing) msg += " " + m;
        FAIL(msg);
    }
}

// ---------------------------------------------------------------- 2. 入口无死角

TEST(sync_every_implemented_cmd_has_a_coverage_entry) {
    // 「无死角」: 固件每条已实现命令都要在 `COMMAND_COVERAGE` 里登记。
    // (上游这里还 `getattr` 验一次入口摸得到 —— C++ 没有名字解析, 那半条由
    //  `test_full_coverage.cpp` 的**动态**检查覆盖: 它把每个入口真调一遍。)
    const auto fw = fw_defines(fw_read(kUsbCmdH), "CMD_", 40);
    std::map<int, std::string> coverage;
    for (const auto& c : proto::command_coverage()) coverage[int(c.cmd)] = c.entry;

    std::vector<std::string> problems;
    for (const auto& kv : fw) {
        const int val = kv.second.first;
        if (proto::unimplemented_cmds().count(uint8_t(val)) ||
            proto::firmware_only_cmds().count(uint8_t(val)) ||
            proto::preexisting_gaps().count(uint8_t(val))) {
            continue;
        }
        if (coverage.count(val) == 0) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "0x%02X %s: 未登记 SDK 入口", val,
                          kv.first.c_str());
            problems.push_back(buf);
        }
    }
    if (!problems.empty()) {
        std::string msg = "覆盖缺口:";
        for (const auto& p : problems) msg += " " + p;
        FAIL(msg);
    }
}

TEST(sync_coverage_map_has_no_stale_entries) {
    // 反向: 覆盖表里不许有固件已不存在的命令 id (否则是过期登记)。
    const auto fw = fw_defines(fw_read(kUsbCmdH), "CMD_", 40);
    std::set<int> fw_ids;
    for (const auto& kv : fw) fw_ids.insert(kv.second.first);
    std::string stale;
    for (const auto& c : proto::command_coverage()) {
        if (fw_ids.count(int(c.cmd)) == 0) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), " 0x%02X", c.cmd);
            stale += buf;
        }
    }
    if (!stale.empty()) FAIL("COMMAND_COVERAGE 有过期登记:" + stale);
}

TEST(sync_exemption_tables_are_still_valid) {
    // 三张豁免表都**不是永久白名单**: 固件侧若删掉/改名/换号, 这条必须红。
    //
    // ⚠ 三张表的**语义不同**, 别以为一条判据就是一个意思:
    //   · `FIRMWARE_ONLY_CMDS`/`FIRMWARE_ONLY_RSPS` —— 用户裁决 SDK **有意不暴露**
    //     ("不做", 不是"还没做"); 两张**现在都是空的**;
    //   · `PREEXISTING_GAPS` —— **既有的固件/SDK 不同步**, 本应同步而未同步。
    //   判据相同 (固件里必须仍有这几条) 不等于理由相同。
    //
    // ⚠ 它同时守反向: 一旦 SDK 补上了同名常量 (= 真的实现了入口), 豁免就该撤掉,
    //   否则豁免表会变成"谁都看不见的过期登记"。
    const auto fw = fw_defines(fw_read(kUsbCmdH), "CMD_", 40);
    const auto fwr = fw_defines(fw_read(kUsbCmdH), "RSP_", 15);
    const auto sdk_c = sdk_defines("CMD_");
    const auto sdk_r = sdk_defines("RSP_");

    std::vector<std::string> problems;
    auto check_cmd = [&](const std::map<uint8_t, std::string>& tbl, const char* tblname) {
        for (const auto& kv : tbl) {
            const auto it = fw.find(kv.second);
            if (it == fw.end()) {
                problems.push_back(std::string(tblname) + " 里的 " + kv.second +
                                   " 在固件头文件里已不存在 —— 请从该表移除");
                continue;
            }
            if (it->second.first != int(kv.first)) {
                char buf[200];
                std::snprintf(buf, sizeof(buf),
                              "%s 在固件里是 0x%02X, %s 写的是 0x%02X —— 请同步",
                              kv.second.c_str(), it->second.first, tblname, int(kv.first));
                problems.push_back(buf);
                continue;
            }
            if (sdk_c.count(kv.second)) {
                problems.push_back(kv.second + " 已被 SDK 实现, 请从 " + tblname + " 移除");
            }
        }
    };
    check_cmd(proto::firmware_only_cmds(), "FIRMWARE_ONLY_CMDS");
    check_cmd(proto::preexisting_gaps(), "PREEXISTING_GAPS");
    for (const auto& kv : proto::firmware_only_rsps()) {
        const auto it = fwr.find(kv.second);
        if (it == fwr.end()) {
            problems.push_back("FIRMWARE_ONLY_RSPS 里的 " + kv.second +
                               " 在固件头文件里已不存在 —— 请从该表移除");
            continue;
        }
        if (it->second.first != int(kv.first)) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                          "%s 在固件里是 0x%02X, FIRMWARE_ONLY_RSPS 写的是 0x%02X —— 请同步",
                          kv.second.c_str(), it->second.first, int(kv.first));
            problems.push_back(buf);
        } else if (sdk_r.count(kv.second)) {
            problems.push_back(kv.second + " 已被 SDK 实现, 请从 FIRMWARE_ONLY_RSPS 移除");
        }
    }
    if (!problems.empty()) {
        std::string msg = "豁免表已过期:";
        for (const auto& p : problems) msg += " [" + p + "]";
        FAIL(msg);
    }
}

// ---------------------------------------------------------------- 3. 状态帧布局

TEST(sync_status_frame_layout_unchanged) {
    // 状态帧载荷长度表达式必须仍是 `6 + N*21` (SDK 解析器按此硬编码)。
    const std::string body = fn_body(fw_read(kUsbCmdC), "usb_cmd_report_status");
    CHECK(!body.empty());   // 找不到函数体 => 固件源码结构变了
    if (body.empty()) return;
    const std::regex re("uint8_t[ \\t]+payload[ \\t]*\\[[ \\t]*([^\\]]+)\\]");
    std::smatch m;
    if (!std::regex_search(body, m, re)) {
        FAIL("找不到状态帧 payload 声明 —— 固件源码结构可能变了");
        return;
    }
    const std::string expr = collapse_spaces(m[1].str());
    CHECK_EQ(expr, std::string("6 + LITEARM_NUM_JOINTS * 21"));
    if (expr != "6 + LITEARM_NUM_JOINTS * 21") {
        // 这条一旦红, 说明固件侧改了布局而 SDK 的 decode_status 还按 6+21N 解 ——
        // 历史上正是这类改动漏同步导致"离线全绿、真机必挂"。
        FAIL("固件状态帧布局变了: `" + expr + "` —— SDK decode_status 的 6+21N 假设需同步");
    }
}

TEST(sync_status_frame_joint_stride_matches_sdk_constant) {
    // 每关节字节数 (5 个 f32 + 1 个 err = 21) 必须与 SDK 的步长一致。
    const std::string body = fn_body(fw_read(kUsbCmdC), "usb_cmd_report_status");
    CHECK(!body.empty());
    if (body.empty()) return;
    const std::regex re(
        // ⚠ 与上游**逐字对齐**: 上游是 `\{(.*?)\n    \}` (非贪婪到"换行 + 4 空格 + 右花括号"),
        //   而**不是**"到第一个 `}`" —— 循环体里只要有内层花括号 (例如一个 if), 后者就会
        //   截断, 于是步长算少、判据失去意义。这类"看着等价其实不等价"的正则正是本文件
        //   要防的东西。
        "for[ \\t]*\\(int i = 0; i < LITEARM_NUM_JOINTS; i\\+\\+\\)[ \\t]*"
        "\\{([\\s\\S]*?)\\n    \\}");
    std::smatch m;
    if (!std::regex_search(body, m, re)) {
        FAIL("找不到状态帧的逐关节打包循环 —— 固件源码结构可能变了");
        return;
    }
    const std::string seg = m[1].str();
    int n_floats = 0, n_u8 = 0;
    const std::regex re_f32("f32_to_le[ \\t]*\\(");
    const std::regex re_u8("\\*pp\\+\\+[ \\t]*=");
    for (auto it = std::sregex_iterator(seg.begin(), seg.end(), re_f32);
         it != std::sregex_iterator(); ++it) {
        ++n_floats;
    }
    for (auto it = std::sregex_iterator(seg.begin(), seg.end(), re_u8);
         it != std::sregex_iterator(); ++it) {
        ++n_u8;
    }
    CHECK_EQ(n_floats * 4 + n_u8, 21);
}

// ---------------------------------------------------------------- 4. 台架常量

TEST(sync_bench_model_axis_matches_sdk_constant) {
    const std::string src = fw_read(kJointCfgH);
    const std::regex re("#if\\s+LITEARM_BENCH_1J([\\s\\S]*?)#else");
    std::smatch m;
    if (!std::regex_search(src, m, re)) {
        FAIL("joint_cfg.h 结构变了 (找不到 #if LITEARM_BENCH_1J ... #else)");
        return;
    }
    const std::regex ax("#define[ \\t]+LITEARM_BENCH_MODEL_AXIS[ \\t]+(\\d+)");
    std::smatch am;
    const std::string bench_branch = m[1].str();   // 命名成 lvalue: 临时对象绑不上
    if (!std::regex_search(bench_branch, am, ax)) {
        FAIL("找不到台架分支的 LITEARM_BENCH_MODEL_AXIS");
        return;
    }
    const int fw_axis = std::stoi(am[1].str());
    CHECK_EQ(fw_axis, proto::BENCH_MODEL_AXIS);
    if (fw_axis != proto::BENCH_MODEL_AXIS) {
        // 不一致 => 台架上的 IK 种子会填错轴。
        FAIL("固件台架模型轴 = " + std::to_string(fw_axis) + ", SDK BENCH_MODEL_AXIS = " +
             std::to_string(proto::BENCH_MODEL_AXIS) + " —— IK 种子会填错轴");
    }
}

// ---------------------------------------------------------------- 5. 能力判定假设

TEST(sync_err_code_zero_only_comes_from_the_default_branch) {
    // `ERR{cmd,0x00}` 必须**只**由未实现命令的 `default` 分支产生 ——
    // SDK 的 `UnsupportedByFirmwareError` 判定完全建立在这个假设上。
    const std::string src = fw_read(kUsbCmdC);
    const std::regex re("RSP_ERR,\\s*\\(const uint8_t\\[\\]\\)\\{([^}]*)\\}");
    std::vector<std::string> zeros;
    for (auto it = std::sregex_iterator(src.begin(), src.end(), re);
         it != std::sregex_iterator(); ++it) {
        const std::string inner = collapse_spaces((*it)[1].str());
        if (inner.size() >= 4 && inner.compare(inner.size() - 4, 4, "0x00") == 0) {
            zeros.push_back(inner);
        }
    }
    CHECK_EQ(int(zeros.size()), 1);
    if (zeros.size() != 1) {
        FAIL("固件里有 " + std::to_string(zeros.size()) +
             " 处回 ERR 码 0x00 (期望恰好 1 处, 即 default 分支) —— SDK 会把它误判成"
             "「固件不支持该命令」");
        return;
    }
    // 那唯一一处必须在 default 分支里。
    const std::regex dflt(
        "default:\\s*\\n\\s*usb_cmd_reply\\(RSP_ERR,\\s*\\(const uint8_t\\[\\]\\)\\{"
        "cmd, 0x00\\}");
    CHECK(std::regex_search(src, dflt));
}

// ---------------------------------------------------------------- 6. 版本门

TEST(sync_firmware_version_not_below_sdk_minimum) {
    const std::string src = fw_read(kLitearmH);
    const std::regex re("#define[ \\t]+LITEARM_FW_VERSION[ \\t]+\"([^\"]+)\"");
    std::vector<std::string> vers;
    for (auto it = std::sregex_iterator(src.begin(), src.end(), re);
         it != std::sregex_iterator(); ++it) {
        vers.push_back((*it)[1].str());
    }
    CHECK(!vers.empty());
    if (vers.empty()) {
        FAIL("找不到 LITEARM_FW_VERSION");
        return;
    }
    std::string bad;
    for (const auto& v : vers) {
        const auto p = proto::parse_firmware_version(v);
        if (!p) {
            bad += " " + v + "(不符合约定)";
            continue;
        }
        if (*p < MIN_FW) {
            bad += " " + v + "(低于 SDK 要求的 " + std::to_string(MIN_FW.major) + "." +
                   std::to_string(MIN_FW.minor) + "." + std::to_string(MIN_FW.patch) + ")";
        }
    }
    if (!bad.empty()) FAIL("固件自报版本不满足 SDK 最低要求:" + bad);
}
