// Minimal offline smoke tests for the public API.
//
// These tests cover the parts that do not need a serial port or an arm:
// the version string, the frame codec, rotation math, and pose normalization.
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "litearm/litearm.hpp"

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

    if (failures != 0) {
        std::cerr << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "ok\n";
    return 0;
}
