// Minimal offline smoke tests for the public API.
//
// These tests cover the parts that do not need a serial port or an arm:
// the version string, the frame codec, rotation math, and pose normalization.
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "litearm/litearm.hpp"
#include "litearm/testing.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::cerr << "FAIL: " << what << "\n";
        ++failures;
    }
}

bool near(double got, double want, double eps = 1e-9) {
    return std::fabs(got - want) <= eps;
}

}  // namespace

int main() {
    check(std::string(litearm::version()) == "2.1.0", "version");

    // CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1.
    const std::string text = "123456789";
    const auto crc = litearm::proto::crc16_ccitt_false(
        reinterpret_cast<const uint8_t*>(text.data()), text.size());
    check(crc == 0x29B1, "crc16 known vector");

    const std::vector<uint8_t> payload{0x01, 0x02, 0x03};
    const auto frame = litearm::proto::pack_frame(litearm::proto::CMD_ENABLE, payload);
    const auto parsed = litearm::proto::unpack_frame(frame);
    check(parsed.has_value(), "frame parses");
    if (parsed) {
        check(parsed->cmd == litearm::proto::CMD_ENABLE, "frame command");
        check(parsed->payload == payload, "frame payload");
    }

    auto corrupt = frame;
    corrupt.back() ^= 0xFF;
    check(!litearm::proto::unpack_frame(corrupt).has_value(), "bad CRC rejected");

    const litearm::rot::Vec3 rpy{0.2, -0.3, 1.1};
    const auto rotation = litearm::rot::rpy_to_mat(rpy);
    const auto roundtrip = litearm::rot::mat_to_rpy(rotation);
    check(near(roundtrip[0], rpy[0]) && near(roundtrip[1], rpy[1]) &&
              near(roundtrip[2], rpy[2]),
          "rpy round trip");
    check(litearm::rot::is_rotation(rotation), "rotation matrix is valid");

    const auto pose = litearm::rot::as_pose(
        std::array<double, 6>{0.3, 0.0, 0.4, 0.0, 0.0, 0.0});
    check(near(pose.first[0], 0.3) && near(pose.first[2], 0.4), "as_pose position");

    // 固件变体门: 整臂 7J 与 7 关节测试构建 test-7J 都收, 台架单电机 (1J) 仍拒。
    const auto test_ver = litearm::proto::parse_firmware_version("Litearm1.13.0-test-7J");
    check(test_ver.has_value() && test_ver->major == 1 && test_ver->minor == 13 &&
              test_ver->patch == 0 && test_ver->variant == "test-7J",
          "test-7J version parses");

    enum class ConnectOutcome { kConnected, kFirmwareMismatch, kOtherError };

    auto try_connect = [](const std::string& fw, std::string* err,
                          std::string* seen_fw) {
        litearm::ArmOptions opts;
        opts.port = "fake";   // 注入了工厂就必须给占位 port
        opts.transport_factory = [fw](const std::string& port) {
            return std::make_unique<litearm::testing::FakeTransport>(port, 0.2, fw, 7);
        };
        try {
            litearm::Arm arm(opts);
            arm.connect();
            if (seen_fw != nullptr) *seen_fw = arm.firmware();
            return ConnectOutcome::kConnected;
        } catch (const litearm::FirmwareMismatchError& e) {
            // 单独接住: 拒绝若退化成 TransportError 之类的别的失败模式, 用例必须失败,
            // 而不是"连不上"就算过。
            if (err != nullptr) *err = e.what();
            return ConnectOutcome::kFirmwareMismatch;
        } catch (const std::exception& e) {
            if (err != nullptr) *err = e.what();
            return ConnectOutcome::kOtherError;
        }
    };

    auto expect_firmware_mismatch = [&try_connect](const std::string& fw,
                                                   const char* rejected,
                                                   const char* named) {
        std::string err;
        check(try_connect(fw, &err, nullptr) == ConnectOutcome::kFirmwareMismatch, rejected);
        check(err.find(fw) != std::string::npos, named);
    };

    {
        std::string err;
        std::string seen_fw;
        check(try_connect("Litearm1.13.0-test-7J", &err, &seen_fw) ==
                  ConnectOutcome::kConnected,
              "connect accepts test-7J");
        check(seen_fw == "Litearm1.13.0-test-7J", "firmware() reports the test build");
    }
    // 回归: 门禁是"先 close() 再抛", 而 close() 清 firmware_ —— 报错文案必须用快照,
    // 否则现场只能看到"固件  的变体是 '1J'" (版本串空), 无从判断板上烧的是哪一版。
    // 三条门禁各钉一条拒绝用例: 格式门 / 过旧门 / 变体门。
    expect_firmware_mismatch("A1.0.0-USB",
                             "malformed firmware is rejected with FirmwareMismatchError",
                             "malformed rejection names the actual firmware string");
    expect_firmware_mismatch("Litearm1.0.0-7J",
                             "too-old firmware is rejected with FirmwareMismatchError",
                             "too-old rejection names the actual firmware string");
    expect_firmware_mismatch("Litearm1.13.0-1J",
                             "1J is rejected with FirmwareMismatchError",
                             "1J rejection names the actual firmware string");
    expect_firmware_mismatch("Litearm1.13.0-test-1J",
                             "test-1J is rejected with FirmwareMismatchError",
                             "test-1J rejection names the actual firmware string");

    if (failures != 0) {
        std::cerr << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "ok\n";
    return 0;
}
