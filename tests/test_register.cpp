// 《未验证登记册》的**格式守门**。
//
// ⚠⚠ 这条测试存在的理由，与登记册本身存在的理由是同一条：**「存在差异」这种没有出处的
//   条目等于没登记** —— 现场复核不了，就得从头再查一遍。而"每条都带出处"如果只靠自觉，
//   它会在第一次赶时间的时候失效。所以把它变成**机器判据**。
//
// ⚠ 判据只查**形状**（有没有写出处、几列、id 连不连号），**查不了出处是真的**。
//   别把"格式绿"读成"登记册都核过了" —— 那是两件事。这条限制本身也登记在册（第 18 条）。
#include "test_support.hpp"

#include <algorithm>
#include <regex>
#include <set>

using namespace litearm;

namespace {

constexpr const char* kRegistryPath = "UNVERIFIED_REGISTER.md";

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

struct EntryRow {
    int line_no = 0;
    int id = 0;
    size_t cols = 0;
    std::string text;
};

std::vector<std::string> slurp_lines(const std::string& text) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : text) {
        if (c == '\n') {
            if (!cur.empty() && cur.back() == '\r') cur.pop_back();   // CRLF 容错
            out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

/// 解析"条目行"：`| <整数> | … | … | … | … |`。
/// ⚠ **按结构判**（首列是纯十进制整数），不是按"含 `|`" —— 后者会把表头、分隔线、
///   图例的小表一起卷进来，判据就退化成"这文件非空且有冒号"。
std::vector<EntryRow> entry_rows(const std::vector<std::string>& lines) {
    std::vector<EntryRow> out;
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string s = trim(lines[i]);
        if (s.empty() || s[0] != '|') continue;
        std::vector<std::string> cells;
        std::string cur;
        for (char c : s) {
            if (c == '|') {
                cells.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(c);
            }
        }
        cells.push_back(cur);
        if (cells.size() < 3u) continue;
        const std::string first = trim(cells[1]);
        if (first.empty() || first.find_first_not_of("0123456789") != std::string::npos) continue;
        EntryRow r;
        r.line_no = int(i) + 1;
        r.id = std::atoi(first.c_str());
        r.cols = cells.size() - 2u;   // 去掉首尾两个空串 ⇒ 真实列数
        r.text = s;
        out.push_back(std::move(r));
    }
    return out;
}

/// 出处的**形状**：`文件.扩展名:数字` 或 7~40 位十六进制（提交哈希）。
/// ⚠ 两半都要有 —— 只认前者会让"见提交 `ca391b1`"这类条目过不去；只认后者则行号出处过不去。
bool has_reference(const std::string& line) {
    static const std::regex kFileLine(R"(\w+\.\w+:\d+)");
    static const std::regex kCommitHash(R"(\b[0-9a-f]{7,40}\b)");
    return std::regex_search(line, kFileLine) || std::regex_search(line, kCommitHash);
}

struct Registry {
    std::vector<std::string> lines;
    std::vector<EntryRow> rows;
    bool loaded = false;
};

const Registry& registry() {
    static Registry r = [] {
        Registry out;
        const std::string raw = lt::read_repo_file(kRegistryPath);
        if (raw.empty()) return out;
        out.lines = slurp_lines(raw);
        out.rows = entry_rows(out.lines);
        out.loaded = true;
        return out;
    }();
    return r;
}

}  // namespace

TEST(register_file_is_present_and_not_empty) {
    const auto& r = registry();
    CHECK(r.loaded);
    if (!r.loaded) return;
    // ⚠ 空册只有一种解释：这一步没做。本仓确实有要登记的东西（分叉、继承缺陷、未验证项），
    //   所以"空"不是一个合法状态。
    CHECK(!r.lines.empty());
    CHECK(!r.rows.empty());
}

TEST(register_every_entry_carries_a_file_or_commit_reference) {
    const auto& r = registry();
    // ⚠ 先钉"条目行 ≥ 1"：**空集上「每行都有出处」恒真** —— 那正是本仓明令禁止的空炮形状。
    CHECK(!r.rows.empty());
    if (r.rows.empty()) return;
    std::vector<std::string> bad;
    for (const auto& e : r.rows) {
        if (!has_reference(e.text)) {
            bad.push_back("第 " + std::to_string(e.line_no) + " 行 (条目 " +
                          std::to_string(e.id) + "): " + e.text);
        }
    }
    if (!bad.empty()) {
        std::string msg =
            "以下登记条目**没有出处** —— 必须带 `文件:行号` 或 7~40 位提交哈希:\n";
        for (const auto& b : bad) msg += "      " + b + "\n";
        msg += "    ⚠ 「存在差异」这种没有出处的条目**等于没登记**（现场无法复核）。";
        FAIL(msg);
    }
}

TEST(register_every_entry_row_has_exactly_five_cells) {
    const auto& r = registry();
    CHECK(!r.rows.empty());
    if (r.rows.empty()) return;
    // 5 = 登记册开头「格式约定」钉死的那张表的列数 (# / 项 / 性质 / 状态 / 出处)。
    // ⚠ 改列数 = 改**全册**体例 ⇒ 那件事要连本行一起改（故意的）。
    constexpr size_t kCols = 5u;
    std::vector<std::string> bad;
    for (const auto& e : r.rows) {
        if (e.cols != kCols) {
            bad.push_back("第 " + std::to_string(e.line_no) + " 行 (条目 " +
                          std::to_string(e.id) + ") 有 " + std::to_string(e.cols) +
                          " 列, 不是 " + std::to_string(kCols) + " 列");
        }
    }
    if (!bad.empty()) {
        std::string msg =
            "登记册表格结构坏了（多半是少写或多写了一个 `|`）:\n";
        for (const auto& b : bad) msg += "      " + b + "\n";
        msg += "    ⚠ 这一条**故意不看内容**: 出处那条判据只查『这行有没有出处』，\n"
               "      少一个 `|` 时出处会**被动地**并进别的格里 ⇒ 那条判据照样绿。";
        FAIL(msg);
    }
}

TEST(register_has_no_table_row_that_lost_its_leading_pipe) {
    // ⚠⚠ 补一条**直指病因**的判据。
    //
    // 丢掉行首那个 `|` 的行 (`3 | **…` 而不是 `| 3 | **…`) **不会被 `entry_rows` 解析**
    // ⇒ 它对上面三条判据**全部隐形**。实测: 那样改一行之后, "列数"那条一声不响。
    // 它**间接**被 id 连续性兜住 (少一条 ⇒ 号段出现空档), 但那条报错指向的是**症状**
    // ("id 3 出现 0 次"), 而真正的病因是"某一行开头少了个竖线" —— 差着一层归因。
    //
    // 判据: 表区里不该有"**数字开头、后面跟着竖线**"的行 —— 正常的条目行以 `|` 开头,
    // 正常的散文以文字/`-`/`*`/`>` 开头。
    const auto& r = registry();
    CHECK(!r.lines.empty());
    if (r.lines.empty()) return;
    static const std::regex kBrokenRow(R"(^\d+\s*\|)");
    std::vector<std::string> bad;
    for (size_t i = 0; i < r.lines.size(); ++i) {
        const std::string s = trim(r.lines[i]);
        if (std::regex_search(s, kBrokenRow)) {
            bad.push_back("第 " + std::to_string(i + 1) + " 行: " + s);
        }
    }
    if (!bad.empty()) {
        std::string msg =
            "登记册里有**丢了行首 `|` 的表格行** —— 那一行不会被解析, 对全部判据隐形:\n";
        for (const auto& b : bad) msg += "      " + b + "\n";
        msg += "    ⚠ 补上开头那个 `|` 即可。这类坏行的危险在于它**读起来还像一条登记**,\n"
               "      而机器完全看不见它。";
        FAIL(msg);
    }
}

TEST(register_ids_are_exactly_one_to_n) {
    const auto& r = registry();
    CHECK(!r.rows.empty());
    if (r.rows.empty()) return;

    std::map<int, int> seen;   // id ⇒ 出现次数
    for (const auto& e : r.rows) seen[e.id] += 1;

    std::vector<std::string> bad;
    // ① 每个 id 必须**恰好一次**：0 次 = 悬空（删条目没重编号），>1 次 = 重号
    //    （会让某条**静默消失**在肉眼扫表里）。
    for (int n = 1; n <= int(r.rows.size()); ++n) {
        const auto it = seen.find(n);
        const int hits = (it == seen.end()) ? 0 : it->second;
        if (hits != 1) {
            bad.push_back("id " + std::to_string(n) + " 出现 " + std::to_string(hits) +
                          " 次（必须恰好一次）");
        }
    }
    // ② 不许有超出 1..N 的 id（N = 条目总数）
    for (const auto& kv : seen) {
        if (kv.first < 1 || kv.first > int(r.rows.size())) {
            bad.push_back("id " + std::to_string(kv.first) + " 越界（应在 1.." +
                          std::to_string(r.rows.size()) + " 之内）");
        }
    }
    if (!bad.empty()) {
        std::string msg =
            "登记册 id 不是恰好 1..N:\n";
        for (const auto& b : bad) msg += "      " + b + "\n";
        msg += "    ⚠ 删条目要**重编号** —— 那是响的改动; 留空号看起来更省事,\n"
               "      但它会让「一共有多少条」这个数变得不可信。";
        FAIL(msg);
    }
}
