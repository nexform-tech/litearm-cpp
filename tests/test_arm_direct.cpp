#include <gtest/gtest.h>
#include <string>
#include "litearm/arm.hpp"
#include "litearm/codec.hpp"
#include "litearm/protocol.hpp"
#include "litearm/transport.hpp"

namespace litearm {
namespace {

TEST(ArmDirect, SendMitPublishesMitFrame) {
    auto tp = std::make_shared<InProcTransport>();
    auto sub = tp->sub(command_topic("armA"));
    Arm arm("", "armA", tp);
    arm.send_mit({15.0,15.0,15.0,15.0,15.0,15.0,15.0},
                 {2.0,2.0,2.0,2.0,2.0,2.0,2.0},
                 {0.1,0.1,0.1,0.1,0.1,0.1,0.1},
                 {0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                 {0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    auto payload = sub->try_recv();
    ASSERT_TRUE(payload.has_value());
    EXPECT_NE(payload->find("\"mit\""), std::string::npos);
    EXPECT_NE(payload->find("\"q_ref\""), std::string::npos);
}

TEST(ArmDirect, SetGuardsSendsRpc) {
    auto tp = std::make_shared<InProcTransport>();
    tp->declare_queryable(rpc_topic("armA"), [](const std::string& payload) -> std::string {
        auto [method, kwargs] = decode_request(payload);
        EXPECT_EQ(method, "set_guards");
        EXPECT_EQ(kwargs.at("slew_limit").as_double(), 0.5);
        return encode_reply_ok(LiteArmValue());
    });
    Arm arm("", "armA", tp);
    arm.set_guards(0.5);
}

}  // namespace
}  // namespace litearm