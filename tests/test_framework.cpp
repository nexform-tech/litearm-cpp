#include "test_framework.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>

namespace lt {

std::string repo_root() {
    // __FILE__ 形如 <whatever>/tests/test_framework.cpp —— 往上两级就是仓根。
    std::string f = __FILE__;
    for (int i = 0; i < 2; ++i) {
        const size_t p = f.find_last_of('/');
        if (p == std::string::npos) return ".";
        f = f.substr(0, p);
    }
    return f;
}

std::string read_repo_file(const std::string& rel_path) {
    const std::string full = repo_root() + "/" + rel_path;
    FILE* fp = std::fopen(full.c_str(), "rb");
    if (fp == nullptr) return {};
    std::string out;
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), fp)) > 0) out.append(buf, n);
    std::fclose(fp);
    return out;
}

std::vector<std::string> list_repo_files(const std::string& dir, const std::string& prefix,
                                         const std::string& suffix) {
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    std::error_code ec;
    const fs::path root = repo_root() + "/" + dir;
    for (const auto& e : fs::directory_iterator(root, ec)) {
        if (ec) break;
        const std::string name = e.path().filename().string();
        if (name.rfind(prefix, 0) != 0) continue;                       // 前缀不符
        if (!suffix.empty() &&                                             // 后缀不符
            (name.size() < suffix.size() ||
             name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0))
            continue;
        out.push_back(dir + "/" + name);
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

CurrentTest& current() {
    static CurrentTest c;
    return c;
}

namespace {

/// 把用例名里的下划线排版成更好读的形态 —— 只影响输出, 不影响标题。
bool has_flag(int argc, char** argv, const char* flag) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], flag) == 0) return true;
    }
    return false;
}

std::string filter_of(int argc, char** argv) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--filter") == 0) return argv[i + 1];
    }
    return "";
}

}  // namespace

int run_all(int argc, char** argv) {
    auto& tests = registry();
    std::sort(tests.begin(), tests.end(),
              [](const TestCase& a, const TestCase& b) {
                  return std::strcmp(a.name, b.name) < 0;
              });
    const std::string filter = filter_of(argc, argv);
    const bool verbose = has_flag(argc, argv, "-v") || has_flag(argc, argv, "--verbose");

    int passed = 0;
    int failed = 0;
    int skipped = 0;
    std::vector<std::string> failed_names;
    for (const auto& t : tests) {
        if (!filter.empty() && std::string(t.name).find(filter) == std::string::npos) {
            continue;
        }
        current() = CurrentTest{};
        bool crashed = false;
        std::string crash_what;
        bool was_skipped = false;
        std::string skip_reason;
        try {
            t.fn();
        } catch (const SkipTest& s) {
            was_skipped = true;
            skip_reason = s.reason;
        } catch (const std::exception& e) {
            crashed = true;
            crash_what = e.what();
        } catch (...) {
            crashed = true;
            crash_what = "未知异常";
        }
        if (was_skipped) {
            // ⚠ **单独播报**, 绝不混进"通过" —— 假绿比没测更糟。
            skipped += 1;
            std::cout << "  SKIP " << t.name << "\n        " << skip_reason << "\n";
            continue;
        }
        if (crashed) {
            current().failures += 1;
            current().messages.push_back("用例体抛出未捕获异常: " + crash_what);
        }
        if (current().failures == 0) {
            passed += 1;
            if (verbose) {
                std::cout << "  ok   " << t.name << " (" << current().checks
                          << " 项断言)\n";
            }
        } else {
            failed += 1;
            failed_names.push_back(t.name);
            std::cout << "  FAIL " << t.name << "\n";
            for (const auto& m : current().messages) {
                std::cout << "       " << m << "\n";
            }
        }
    }
    std::cout << (failed == 0 ? "全部通过" : "有失败") << ": " << passed << " 通过 / "
              << failed << " 失败";
    if (skipped > 0) {
        // 跳过**必须摆在明面上**: 只报"通过"会把"没测"读成"测过了"。
        std::cout << " / **" << skipped << " 跳过** (环境前提不满足, 见上面的 SKIP 行)";
    }
    std::cout << "\n";

    if (failed != 0) return 1;
    // ⚠ **有跳过时返回 77, 不是 0** —— 否则 ctest 只显示 "Passed", 而"跳过"这件事被
    //   完全盖住 (实测: 打印着"11 跳过"的用例在 ctest 里就是一行 "Passed")。
    //   CMakeLists 里对每个测试设了 `SKIP_RETURN_CODE 77`, ctest 于是把它报成
    //   `***Skipped` —— 粗粒度 (一个二进制里只要**有**跳过就整体标 Skipped), 但
    //   **响亮**; 而响亮正是这里唯一要买的东西。
    return skipped > 0 ? 77 : 0;
}

}  // namespace lt

int main(int argc, char** argv) { return lt::run_all(argc, argv); }
