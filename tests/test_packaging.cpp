// 打包一致性 —— 装出来的东西**自洽**, 且版本只有**一个**说法。
//
// ⚠ 这一类的失败有个共同的恶劣形状: **报错指向的地方离病因很远**。
//   实测过的两例 (一例在另一套实现上):
//     * 装出来的 prefix 里少一个头 ⇒ 消费者报 "litearm/version.hpp: 没有那个文件",
//       而病因是 install 规则漏了一行;
//     * CMake 包里漏了 `find_dependency(Threads)` ⇒ 消费者报
//       "The link interface ... contains Threads::Threads but the target was not found",
//       而库本身编得好好的。
//   ⇒ 所以在**卖出去之前**在这里守一道, 而不是等消费者来报。
#include "test_support.hpp"

using namespace litearm;

namespace {

/// 从 `text` 里取 `key` 之后紧邻的那串数字版本 (形如 `1.2.3`)。
std::string version_after(const std::string& text, const std::string& key) {
    const size_t at = text.find(key);
    if (at == std::string::npos) return {};
    size_t i = at + key.size();
    while (i < text.size() && !std::isdigit(static_cast<unsigned char>(text[i]))) ++i;
    size_t j = i;
    while (j < text.size() &&
           (std::isdigit(static_cast<unsigned char>(text[j])) || text[j] == '.'))
        ++j;
    return text.substr(i, j - i);
}

}  // namespace

TEST(packaging_version_has_exactly_one_source_of_truth) {
    // CMakeLists 的 `project(... VERSION x.y.z)` 与 `litearm.hpp` 里那三个宏 + 版本串
    // 必须一致。⚠ 两处都手写 ⇒ 只改一处是**静默**的, 而症状是"客户拿到的版本号是旧的"
    // 这种极难察觉的错。故钉住。
    const std::string cmake = lt::read_repo_file("CMakeLists.txt");
    const std::string hpp = lt::read_repo_file("include/litearm/litearm.hpp");
    CHECK(!cmake.empty());
    CHECK(!hpp.empty());
    if (cmake.empty() || hpp.empty()) return;

    const std::string v = version_after(cmake, "project(litearm-cpp VERSION");
    CHECK(!v.empty());
    const std::string major = v.substr(0, v.find('.'));
    const std::string rest = v.substr(v.find('.') + 1);
    const std::string minor = rest.substr(0, rest.find('.'));
    const std::string patch = rest.substr(rest.find('.') + 1);

    CHECK_EQ(version_after(hpp, "LITEARM_VERSION_STRING"), v);
    CHECK_EQ(version_after(hpp, "LITEARM_VERSION_MAJOR"), major);
    CHECK_EQ(version_after(hpp, "LITEARM_VERSION_MINOR"), minor);
    CHECK_EQ(version_after(hpp, "LITEARM_VERSION_PATCH"), patch);
    // 运行期函数也必须跟同一份走
    CHECK_EQ(std::string(version()), v);
}

TEST(packaging_every_public_header_is_self_contained_in_the_shipped_dir) {
    // 装出去的是**整个 `include/litearm/` 目录** ⇒ 只要公开头里引用的兄弟头**都在这个目录里**,
    // 装出来的就是自洽的。这条守的是"引了一个不在本目录的头" (改名 / 打错 / 放在别处)。
    const auto headers = lt::list_repo_files("include/litearm", "", ".hpp");
    CHECK(!headers.empty());
    if (headers.empty()) return;

    std::vector<std::string> missing;
    for (const auto& h : headers) {
        const std::string src = lt::strip_cxx_comments(lt::read_repo_file(h));
        const std::string marker = "#include \"litearm/";
        for (size_t p = src.find(marker); p != std::string::npos;
             p = src.find(marker, p + 1)) {
            const size_t b = p + marker.size();
            const size_t e = src.find('"', b);
            if (e == std::string::npos) continue;
            const std::string name = src.substr(b, e - b);
            if (lt::read_repo_file("include/litearm/" + name).empty()) {
                missing.push_back(h + " -> litearm/" + name);
            }
        }
    }
    if (!missing.empty()) {
        std::string msg = "公开头引用了**不在 include/litearm/ 里**的兄弟头 ⇒ 装出去就编不过:";
        for (const auto& m : missing) msg += "\n      " + m;
        FAIL(msg);
    }
}

TEST(packaging_cmake_and_pkgconfig_templates_are_present) {
    // 这两个模板是 install 规则的输入。⚠ 它们**不在** include/ 下, 所以不会被
    // `install(DIRECTORY include/litearm ...)` 顺带装上 —— 只会在 cmake 配置期被读。
    // 删了/改名了 ⇒ 配置期报错 (还算响); 但被 `if(EXISTS)` 之类的写法包住时就是静默的。
    CHECK(!lt::read_repo_file("cmake/litearmConfig.cmake.in").empty());
    CHECK(!lt::read_repo_file("cmake/litearm.pc.in").empty());
    CHECK(!lt::read_repo_file("cmake/toolchain-aarch64-linux-gnu.cmake").empty());
}

TEST(packaging_pkgconfig_and_config_carry_the_same_version) {
    // `.pc` 与 CMake 包配置都从 `@PROJECT_VERSION@` 取值 ⇒ 二者必须同源。
    // 这条钉的是"将来有人把 .pc 里的 Version 写死成字面量"。
    const std::string pc = lt::read_repo_file("cmake/litearm.pc.in");
    CHECK(!pc.empty());
    if (pc.empty()) return;
    CHECK(pc.find("Version: @PROJECT_VERSION@") != std::string::npos);
}
