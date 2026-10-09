// 样例参数解析 (examples/_common.cpp) —— 负数位置参数与私有开关的负值。
//
// 起因: 解析器原先把所有 `-` 开头的 token 都当未知开关 (exit 2), 于是 02_movej
// 头注释里推荐的 `./02_movej --go 0.1 0 -0.1 0 0 0 0` 根本跑不起来;
// `--dist -0.04` (向 -X 平移) 同理。修复口径: 能解析成**完整一个数**的 token
// (位置参数槽与私有开关的值槽) 按参数走; 非数字的 `-x` 仍是错误路径。
#include "test_support.hpp"

#include "../examples/_common.hpp"

#ifndef _WIN32
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

/// 把**参数词表**喂给 example::parse_args (它从 argv[1] 开始读)。
/// ⚠ 别把"程序名" (`02_movej`) 放进词表: 真实进程里程序名在 argv[0], 而 parse_args
///   从 argv[1] 起解析 —— 放进词表会被 `atof` 吃成一个数字位置参数 (实测踩过)。
bool parse(const std::vector<std::string>& words, example::Args* out) {
    std::vector<std::string> store = words;
    std::vector<char*> argv;
    argv.reserve(store.size() + 1);
    argv.push_back(const_cast<char*>("example"));
    for (auto& w : store) argv.push_back(w.data());
    return example::parse_args(int(argv.size()), argv.data(), "test", out);
}

}  // namespace

TEST(example_args_negative_positional_is_a_number_not_a_switch) {
    // 02_movej 文档用法: 负数关节角必须能作为位置参数传进来。
    example::Args a;
    CHECK(parse({"--go", "0.1", "0", "-0.1", "0", "0", "0", "0"}, &a));
    CHECK_TRUE(a.go);
    CHECK_EQ(a.rest.size(), size_t(7));
    CHECK_NEAR(a.rest[0], 0.1, 1e-12);
    CHECK_NEAR(a.rest[2], -0.1, 1e-12);
}

TEST(example_args_single_dash_and_scientific_forms) {
    example::Args a;
    CHECK(parse({"-5", "-.5", "-1e-3"}, &a));
    CHECK_EQ(a.rest.size(), size_t(3));
    CHECK_NEAR(a.rest[0], -5.0, 1e-12);
    CHECK_NEAR(a.rest[1], -0.5, 1e-12);
    CHECK_NEAR(a.rest[2], -1e-3, 1e-15);
}

TEST(example_args_private_switch_accepts_negative_value) {
    // `--dist -0.04` (向 -X 平移) 要把 -0.04 吃成值, 而不是当成未知开关。
    example::Args a;
    CHECK(parse({"--dist", "-0.04"}, &a));
    CHECK_EQ(a.extra.count("dist"), size_t(1));
    CHECK_NEAR(std::atof(a.extra.at("dist").c_str()), -0.04, 1e-12);
    CHECK_TRUE(a.rest.empty());
}

TEST(example_args_positive_forms_unchanged) {
    example::Args a;
    CHECK(parse({"--dist", "0.04", "--verbose", "--tag", "trace_x"}, &a));
    CHECK_NEAR(std::atof(a.extra.at("dist").c_str()), 0.04, 1e-12);
    CHECK_EQ(a.extra.at("verbose"), "1");
    CHECK_EQ(a.extra.at("tag"), "trace_x");
}

TEST(example_args_help_returns_false) {
    example::Args a;
    CHECK_FALSE(parse({"--help"}, &a));
    CHECK_FALSE(parse({"-h"}, &a));
}

#ifndef _WIN32
TEST(example_args_unknown_switch_still_exits_2) {
    // 非数字的 `-x` 仍是错误路径 (exit 2) —— 修复只放行"能解析成数"的 token。
    ::fflush(nullptr);
    const pid_t pid = ::fork();
    CHECK(pid >= 0);
    if (pid == 0) {
        example::Args a;
        parse({"-x"}, &a);  // 期望 exit(2)
        ::_exit(0);         // 没退: 退出码 0, 父进程判失败
    }
    int status = 0;
    ::waitpid(pid, &status, 0);
    CHECK(WIFEXITED(status));
    CHECK_EQ(WEXITSTATUS(status), 2);
}
#endif
