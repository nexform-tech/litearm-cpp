// 错误层级 + (cmd, code) 语义表 + ERR 载荷 -> 异常的映射。
#include "test_support.hpp"

using namespace litearm;

TEST(errors_hierarchy_everything_derives_from_lite_arm_error) {
    // 全部 catch 点都挂在 LiteArmError 上, 所以每一条都必须是它的派生类。
    CHECK((std::is_base_of<LiteArmError, NotConnectedError>::value));
    CHECK((std::is_base_of<LiteArmError, ForkedSessionError>::value));
    CHECK((std::is_base_of<LiteArmError, TransportError>::value));
    CHECK((std::is_base_of<LiteArmError, FirmwareMismatchError>::value));
    CHECK((std::is_base_of<LiteArmError, InvalidCommandError>::value));
    CHECK((std::is_base_of<LiteArmError, MotorFaultError>::value));
    CHECK((std::is_base_of<LiteArmError, MotionTimeoutError>::value));
    CHECK((std::is_base_of<LiteArmError, IKError>::value));
    CHECK((std::is_base_of<LiteArmError, CommandRejectedError>::value));
    CHECK((std::is_base_of<LiteArmError, CartesianPlanError>::value));
    CHECK((std::is_base_of<LiteArmError, MotionSupersededError>::value));
    CHECK((std::is_base_of<LiteArmError, CartReplyLostError>::value));
    CHECK((std::is_base_of<LiteArmError, ArmIsInDfuError>::value));
    CHECK((std::is_base_of<LiteArmError, NotRemoteable>::value));
    CHECK((std::is_base_of<LiteArmError, NotSupportedOnThisBackend>::value));
    CHECK((std::is_base_of<LiteArmError, TeleopLockedError>::value));
    CHECK((std::is_base_of<LiteArmError, TeleopBusyError>::value));
}

TEST(errors_forked_session_is_a_not_connected_error) {
    // 继承自 NotConnectedError: except NotConnectedError 与 except LiteArmError 都抓得到 ——
    // 这个会话在本进程里确实不可用。
    CHECK((std::is_base_of<NotConnectedError, ForkedSessionError>::value));
    try {
        throw ForkedSessionError("x");
    } catch (const NotConnectedError&) {
        // 期望走到这里
    } catch (...) {
        FAIL("ForkedSessionError 应当能被 NotConnectedError 抓到");
    }
}

TEST(errors_unsupported_by_firmware_is_a_command_rejected_error) {
    // 既有捕获 CommandRejectedError 的调用方不受影响。
    CHECK((std::is_base_of<CommandRejectedError, UnsupportedByFirmwareError>::value));
}

TEST(errors_motion_superseded_is_not_a_plan_error) {
    // 接管是正常用法, 不得继承 CartesianPlanError —— 混进"规划失败"会让调用方走故障恢复。
    CHECK(!(std::is_base_of<CartesianPlanError, MotionSupersededError>::value));
    CHECK(!(std::is_base_of<CommandRejectedError, CartesianPlanError>::value));
    // ArmIsInDfuError 刻意不继承 NotConnectedError: 它不是一次可恢复的掉线。
    CHECK(!(std::is_base_of<NotConnectedError, ArmIsInDfuError>::value));
}

TEST(errors_raise_firmware_error_code_zero_is_unsupported) {
    // code == 0x00 恒等于固件 usb_cmd.c 的 default 分支 = 未实现该命令。
    const std::vector<uint8_t> payload{0x34, 0x00};
    try {
        raise_firmware_error(payload, "model.get_body: ");
        FAIL("应当抛异常");
    } catch (const UnsupportedByFirmwareError& e) {
        CHECK_EQ(int(e.cmd), 0x34);
        CHECK_EQ(int(e.code), 0x00);
        CHECK(std::string(e.what()).find("固件没有实现这条命令") != std::string::npos);
    }
}

TEST(errors_raise_firmware_error_other_codes_are_command_rejected) {
    const std::vector<uint8_t> payload{0x01, 0x03};
    try {
        raise_firmware_error(payload, "movej 被固件拒绝: ");
        FAIL("应当抛异常");
    } catch (const UnsupportedByFirmwareError&) {
        FAIL("非 0 码不该被判成 UnsupportedByFirmwareError");
    } catch (const CommandRejectedError& e) {
        CHECK_EQ(int(e.cmd), 0x01);
        CHECK_EQ(int(e.code), 0x03);
        // 文案里要带出具体语义 (查 ERR_TEXT)
        CHECK(std::string(e.what()).find("ERR [01,03]") != std::string::npos);
        CHECK(std::string(e.what()).find("未使能") != std::string::npos);
    }
}

TEST(errors_raise_firmware_error_handles_short_payload) {
    // 载荷缺失时按 0 处理, 不越界读。
    try {
        raise_firmware_error({}, "x: ");
        FAIL("应当抛异常");
    } catch (const UnsupportedByFirmwareError& e) {
        CHECK_EQ(int(e.cmd), 0);
        CHECK_EQ(int(e.code), 0);
    }
}

TEST(errors_err_reason_first_tier_uses_the_specific_table) {
    // 具体档命中 -> 用它。同一个 code 在不同命令上语义不同, 这正是具体档存在的理由。
    const std::string a = err_reason(0x01, 0x03);
    const std::string b = err_reason(0x10, 0x03);
    CHECK(a != b);
    CHECK(a.find("EMERGENCY") != std::string::npos);
    CHECK(b.find("可重试") != std::string::npos);
    // 0x23 的 0x03 又是第三层语义
    CHECK(err_reason(0x23, 0x03).find("新限位关不住在途目标") != std::string::npos);
}

TEST(errors_err_reason_second_tier_appends_the_raw_code) {
    // 具体档未命中、通用档命中 -> 用通用档并附上原始码 (固件新增一档时这里是唯一会让上位机
    // 看见"这是我没见过的码"的地方)。
    const std::string s = err_reason(0x01, 0x05);   // 0x01 未登记 0x05
    CHECK(s.find("本命令未登记此码") != std::string::npos);
    CHECK(s.find("cmd=0x01") != std::string::npos);
    CHECK(s.find("code=0x05") != std::string::npos);
}

TEST(errors_err_reason_third_tier_says_unregistered) {
    const std::string s = err_reason(0x01, 0x7F);   // 两级都没有
    CHECK(s.find("未登记的固件错误码") != std::string::npos);
    CHECK(s.find("cmd=0x01") != std::string::npos);
    CHECK(s.find("code=0x7F") != std::string::npos);
}

TEST(errors_zero_code_is_never_in_the_specific_table) {
    // code == 0x00 一律不在此表 —— 它恒等于"固件没有这条命令", 由通用档承担。
    for (const auto& kv : err_text()) {
        CHECK(kv.first.second != 0x00);
    }
    CHECK(err_code_text().count(0x00) != 0);
}

TEST(errors_error_codes_are_unique_per_command_and_well_formed) {
    for (const auto& kv : err_text()) {
        CHECK(kv.first.first > 0);
        // 0x08 是唯一超过 0x07 的一档 (0x10 ENABLE 的"未激活")
        CHECK(kv.first.second >= 0x01 && kv.first.second <= 0x08);
        CHECK(!kv.second.empty());
    }
}

TEST(errors_specific_table_pins_the_load_bearing_entries) {
    // 这几条是现场判定用的判据, 逐条钉住 —— 改错了会让人判反方向。
    CHECK(err_reason(0x01, 0x06).find("drop_hold") != std::string::npos);
    CHECK(err_reason(0x2A, 0x06).find("drop_hold") != std::string::npos);
    CHECK(err_reason(0x3A, 0x06).find("drop_hold") != std::string::npos);
    CHECK(err_reason(0x10, 0x08).find("未激活") != std::string::npos);
    CHECK(err_reason(0x10, 0x06).find("EMERGENCY") != std::string::npos);
    CHECK(err_reason(0x10, 0x07).find("重发无用") != std::string::npos);
    CHECK(err_reason(0x02, 0x03).find("二义码") != std::string::npos);
    CHECK(err_reason(0x32, 0x07).find("掩码不符") != std::string::npos);
    CHECK(err_reason(0x3F, 0x02).find("聚合档") != std::string::npos);
    CHECK(err_reason(0x23, 0x02).find("放宽") != std::string::npos);
    CHECK(err_reason(0x28, 0x04).find("feat/hyy-model-import") != std::string::npos);
}

TEST(errors_0x28_02_text_scopes_the_clamp_claim) {
    // 幅值静默钳制只对**有 clamp 的** item 成立 —— 文案不许说成全称。
    const std::string s = err_reason(0x28, 0x02);
    CHECK(s.find("静默钳制") != std::string::npos);
    CHECK(s.find("item 9") != std::string::npos);
    CHECK(s.find("payload_mass") != std::string::npos);
}

TEST(errors_enable_retry_whitelist_is_only_0x03) {
    // 白名单 ("只有它说可重试才重试") 而不是黑名单: 0x06/0x07/0x00 都明确是"重发无用"。
    const auto& codes = enable_retryable_codes();
    CHECK_EQ(int(codes.size()), 1);
    CHECK(codes.count(0x03) != 0);
    CHECK(codes.count(0x06) == 0);
    CHECK(codes.count(0x07) == 0);
    CHECK(codes.count(0x00) == 0);
}

TEST(errors_generic_table_covers_the_skeleton_codes) {
    for (int c = 0; c <= 7; ++c) {
        CHECK(err_code_text().count(c) != 0);
    }
    CHECK(err_code_text().at(0x01).find("长度") != std::string::npos);
    CHECK(err_code_text().at(0x06).find("reset") != std::string::npos);
}
