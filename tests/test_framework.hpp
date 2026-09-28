// 极小测试框架 —— 零依赖 (不引 gtest/catch), 因为本仓的定位是"离线即可编译运行"。
//
// 用法:
//     #include "test_framework.hpp"
//     TEST(protocol_pack_frame_roundtrip) {
//         auto f = litearm::proto::pack_frame(0x10);
//         CHECK_EQ(f[0], litearm::proto::SOF);
//     }
//
// 每条 TEST 注册到全局表; main 由 test_framework.cpp 提供 (每个 test_*.cpp 编成独立的
// 可执行文件, ctest 逐个跑)。
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace lt {

struct TestCase {
    const char* name;
    const char* file;
    void (*fn)();
};

/// 全局用例表 (定义在 test_framework.cpp)。
std::vector<TestCase>& registry();

/// 用例**跳过** —— 环境前提不满足时用 (例如固件源码不在本机)。
///
/// ⚠ 与"通过"**不是一回事**, 且必须**看得见**: 报告里单独计一列, 播报时打 `SKIP` 行。
/// 上游同款 (`pytest.skip`), 理由一句话: **假绿比没测更糟**。
///
/// 判据是"这个前提**环境上**拿不到" (固件树不在), 不是"这条太难测先跳过" ——
/// 后者是删测试, 本仓不许。
struct SkipTest {
    std::string reason;
};

/// 当前用例的失败计数与日志 —— 用例结束时由 run_all 检查。
struct CurrentTest {
    int checks = 0;
    int failures = 0;
    std::vector<std::string> messages;
};
CurrentTest& current();

int run_all(int argc, char** argv);

/// 注册器 —— 用静态对象在 main 之前把用例塞进表里。
struct Registrar {
    Registrar(const char* name, const char* file, void (*fn)()) {
        registry().push_back(TestCase{name, file, fn});
    }
};

}  // namespace lt

#define LT_CONCAT_(a, b) a##b
#define LT_CONCAT(a, b) LT_CONCAT_(a, b)

#define TEST(name)                                                             \
    static void LT_CONCAT(lt_test_fn_, __LINE__)();                            \
    static ::lt::Registrar LT_CONCAT(lt_test_reg_, __LINE__)(                  \
        #name, __FILE__, &LT_CONCAT(lt_test_fn_, __LINE__));                   \
    static void LT_CONCAT(lt_test_fn_, __LINE__)()

#define LT_FAIL_(msg)                                                          \
    do {                                                                       \
        ::lt::current().failures += 1;                                         \
        std::ostringstream lt_os_;                                             \
        lt_os_ << __FILE__ << ":" << __LINE__ << ": " << (msg);                \
        ::lt::current().messages.push_back(lt_os_.str());                      \
    } while (0)

#define CHECK(cond)                                                            \
    do {                                                                       \
        ::lt::current().checks += 1;                                           \
        if (!(cond)) LT_FAIL_(std::string("CHECK(" #cond ") 失败"));           \
    } while (0)

#define CHECK_TRUE(cond) CHECK(cond)
#define CHECK_FALSE(cond) CHECK(!(cond))

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        ::lt::current().checks += 1;                                           \
        const auto lt_a_ = (a);                                                \
        const auto lt_b_ = (b);                                                \
        if (!(lt_a_ == lt_b_)) {                                               \
            std::ostringstream lt_os2_;                                        \
            lt_os2_ << "CHECK_EQ(" #a ", " #b ") 失败: " << lt_a_              \
                    << " != " << lt_b_;                                        \
            LT_FAIL_(lt_os2_.str());                                           \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        ::lt::current().checks += 1;                                           \
        const double lt_a2_ = double(a);                                       \
        const double lt_b2_ = double(b);                                       \
        if (!(std::fabs(lt_a2_ - lt_b2_) <= double(tol))) {                    \
            std::ostringstream lt_os3_;                                        \
            lt_os3_ << "CHECK_NEAR(" #a ", " #b ") 失败: " << lt_a2_           \
                    << " vs " << lt_b2_ << " (tol " << double(tol) << ")";     \
            LT_FAIL_(lt_os3_.str());                                           \
        }                                                                      \
    } while (0)

/// 断言 `expr` 抛出 `ExType` (必须精确匹配到该类型或它的派生类)。
#define CHECK_THROWS_AS(expr, ExType)                                          \
    do {                                                                       \
        ::lt::current().checks += 1;                                           \
        bool lt_thrown_ = false;                                               \
        try {                                                                  \
            expr;                                                              \
        } catch (const ExType&) {                                              \
            lt_thrown_ = true;                                                 \
        } catch (const std::exception& lt_e_) {                                \
            std::ostringstream lt_os4_;                                        \
            lt_os4_ << "CHECK_THROWS_AS(" #expr ", " #ExType                   \
                    << ") 抛了别的类型: " << lt_e_.what();                     \
            LT_FAIL_(lt_os4_.str());                                           \
            lt_thrown_ = true;                                                 \
        }                                                                      \
        if (!lt_thrown_) LT_FAIL_("CHECK_THROWS_AS(" #expr ", " #ExType        \
                                  ") 没抛 (期望抛出)");                        \
    } while (0)

/// 断言 `expr` 不抛任何异常。
#define CHECK_NOTHROW(expr)                                                    \
    do {                                                                       \
        ::lt::current().checks += 1;                                           \
        try {                                                                  \
            expr;                                                              \
        } catch (const std::exception& lt_e_) {                                \
            std::ostringstream lt_os5_;                                        \
            lt_os5_ << "CHECK_NOTHROW(" #expr ") 抛了: " << lt_e_.what();      \
            LT_FAIL_(lt_os5_.str());                                           \
        }                                                                      \
    } while (0)

/// 显式失败 (用于"运行到了不该到的分支")。
#define FAIL(msg) LT_FAIL_(msg)

/// 跳过本用例 (环境前提不满足)。**会在报告里单列一行**, 不算通过。
/// 用法: `SKIP("固件源码不在 ... —— 设 LITEARM_FW_DIR");`
#define SKIP(reason) throw ::lt::SkipTest{std::string(reason)}
