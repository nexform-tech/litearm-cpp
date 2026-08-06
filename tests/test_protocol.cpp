#include <gtest/gtest.h>
#include "litearm/protocol.hpp"

namespace litearm {
namespace {

TEST(Protocol, RpcTopic) {
    EXPECT_EQ(rpc_topic("armA"), "litearm/v4/armA/rpc");
    EXPECT_EQ(rpc_topic("armB"), "litearm/v4/armB/rpc");
    EXPECT_EQ(rpc_topic("arm_123"), "litearm/v4/arm_123/rpc");
}

TEST(Protocol, StateTopic) {
    EXPECT_EQ(state_topic("armA"), "litearm/v4/armA/state");
    EXPECT_EQ(state_topic("armB"), "litearm/v4/armB/state");
}

TEST(Protocol, CommandTopic) {
    EXPECT_EQ(command_topic("armA"), "litearm/v4/armA/command");
}

TEST(Protocol, EstopTopic) {
    EXPECT_EQ(estop_topic("armA"), "litearm/v4/armA/estop");
}

TEST(Protocol, ProtocolVersion) {
    EXPECT_EQ(PROTOCOL_VERSION, 1);
}

} // namespace
} // namespace litearm
