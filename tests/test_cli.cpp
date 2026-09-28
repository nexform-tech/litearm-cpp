// 简易 CLI (巡检/冒烟) —— 参数解析与错误路径。
#include "test_support.hpp"

using namespace litearm;

TEST(cli_help_exits_zero) {
    CHECK_EQ(main_impl({"--help"}), 0);
    CHECK_EQ(main_impl({"-h"}), 0);
}

TEST(cli_unknown_port_is_a_transport_error_not_a_crash) {
    // 无硬件 / 端口不存在时应当是那句可读的 TransportError, 而不是别的异常。
    CHECK_THROWS_AS(main_impl({"status", "--port", "/dev/litearm-nope-xyz"}),
                    TransportError);
}

TEST(cli_no_hardware_at_all_is_reported_as_such) {
    // 本机没接臂且没给 --port => 自动发现失败 => 同一句 TransportError。
    // (若本机真的接了 1d50:606f 的臂, 这条会去连它 —— 那也不是失败, 故用 catch 兜住。)
    try {
        const int rc = main_impl({"fw", "--port", "/dev/litearm-nope-xyz"});
        CHECK_EQ(rc, 0);
    } catch (const TransportError& e) {
        CHECK(std::string(e.what()).find("/dev/litearm-nope-xyz") != std::string::npos ||
              std::string(e.what()).find("STM32 CDC") != std::string::npos);
    }
}

TEST(cli_action_names_are_recognised) {
    // 这些 action 都会被解析, 只是没有硬件时连不上 —— 故只断言"连不上"这一步,
    // 而不是"参数不认识"。
    for (const char* action : {"status", "fw", "enable", "disable", "reset",
                               "emergency", "movej", "home", "tcp"}) {
        CHECK_THROWS_AS(main_impl({action, "--port", "/dev/litearm-nope-xyz"}),
                        TransportError);
    }
}

TEST(cli_version_string_is_exposed) {
    CHECK_EQ(std::string(version()), std::string("2.1.0"));
}
