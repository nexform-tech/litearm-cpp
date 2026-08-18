#include <gtest/gtest.h>
#include "litearm/arm.hpp"
#include "litearm/codec.hpp"
#include "litearm/protocol.hpp"
#include "litearm/transport.hpp"

namespace litearm {
namespace {

using KwargsMap = std::map<std::string, LiteArmValue>;

/// Capture device.* RPC calls and reply true by default.
struct MockServer {
    std::shared_ptr<InProcTransport> tp;
    std::vector<std::string> methods;               // full "device.X.method"
    std::vector<KwargsMap> kwargs_list;

    explicit MockServer(const std::string& arm_id = "armA") {
        tp = std::make_shared<InProcTransport>();
        tp->declare_queryable(rpc_topic(arm_id),
            [this](const std::string& payload) -> std::string {
                auto [method, kwargs] = decode_request(payload);
                methods.push_back(method);
                kwargs_list.push_back(kwargs);

                LiteArmValue result(true);
                if (method.find(".get_width") != std::string::npos) {
                    result = LiteArmValue(0.42);
                } else if (method.find(".list_gestures") != std::string::npos ||
                           method.find(".get_joints") != std::string::npos) {
                    result = LiteArmValue(LiteArmValue::ListType{});
                } else if (method.find(".get_state") != std::string::npos ||
                           method.find(".get_status") != std::string::npos ||
                           method.find(".get_info") != std::string::npos ||
                           method.find(".get_buttons") != std::string::npos) {
                    result = LiteArmValue(LiteArmValue::MapType{});
                }
                return encode_reply_ok(result);
            });
    }
};

// ── RemoteDevice routing ─────────────────────────────────────────────────────

TEST(RemoteDevice, RoutesThroughArmRpc) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    auto hand = arm.device("hand_0");
    EXPECT_TRUE(hand.open());
    EXPECT_EQ(srv.methods.back(), "device.hand_0.open");

    EXPECT_TRUE(hand.close());
    EXPECT_EQ(srv.methods.back(), "device.hand_0.close");

    EXPECT_TRUE(hand.set_gesture("pinch"));
    EXPECT_EQ(srv.methods.back(), "device.hand_0.set_gesture");
    EXPECT_EQ(srv.kwargs_list.back().at("gesture").as_string(), "pinch");

    EXPECT_TRUE(hand.finger_move({0.1, 0.2, 0.3, 0.4, 0.5, 0.6}));
    EXPECT_EQ(srv.methods.back(), "device.hand_0.finger_move");
    EXPECT_EQ(srv.kwargs_list.back().at("pose").to_vec().size(), 6u);

    EXPECT_TRUE(hand.set_speed({1.0, 1.0}));
    EXPECT_EQ(srv.methods.back(), "device.hand_0.set_speed");

    EXPECT_TRUE(hand.set_torque({1.0, 1.0}));
    EXPECT_EQ(srv.methods.back(), "device.hand_0.set_torque");
}

TEST(RemoteDevice, StateAndGestureListing) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    auto hand = arm.device("hand_0");
    // get_state / list_gestures / get_status / get_info return whatever server sends
    hand.get_state();
    EXPECT_EQ(srv.methods.back(), "device.hand_0.get_state");
    hand.list_gestures();
    EXPECT_EQ(srv.methods.back(), "device.hand_0.list_gestures");
    hand.get_status();
    EXPECT_EQ(srv.methods.back(), "device.hand_0.get_status");
    hand.get_info();
    EXPECT_EQ(srv.methods.back(), "device.hand_0.get_info");

    // bool-returning common methods
    EXPECT_TRUE(hand.connect());
    hand.disconnect();
    EXPECT_TRUE(hand.clear_faults());
    EXPECT_TRUE(hand.set_force(0.8));
    EXPECT_EQ(srv.kwargs_list.back().at("force").as_double(), 0.8);
}

TEST(RemoteDevice, GripperWidth) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    auto gripper = arm.device("gripper_0");
    EXPECT_TRUE(gripper.set_width(0.5));
    EXPECT_EQ(srv.methods.back(), "device.gripper_0.set_width");
    EXPECT_DOUBLE_EQ(srv.kwargs_list.back().at("width").as_double(), 0.5);

    double width = gripper.get_width();
    EXPECT_EQ(srv.methods.back(), "device.gripper_0.get_width");
    EXPECT_DOUBLE_EQ(width, 0.42);
}

TEST(RemoteDevice, TeachPendantMethods) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    auto teach = arm.device("teach_0");
    teach.get_joints();
    EXPECT_EQ(srv.methods.back(), "device.teach_0.get_joints");
    teach.get_buttons();
    EXPECT_EQ(srv.methods.back(), "device.teach_0.get_buttons");
}

TEST(RemoteDevice, ArbitraryCall) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    auto dev = arm.device("custom_0");
    dev.call("some_method", {{"arg", LiteArmValue(int64_t(1))}});
    EXPECT_EQ(srv.methods.back(), "device.custom_0.some_method");
    EXPECT_EQ(srv.kwargs_list.back().at("arg").as_int(), 1);
}

TEST(RemoteDevice, ArmPrefixDoesNotApplyToPlainRpc) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    // device() must NOT leak the device prefix into ordinary arm RPCs
    arm.device("hand_0");  // create the lazy DeviceManager
    EXPECT_TRUE(arm.movej({0,0,0,0,0,0,0}));
    EXPECT_EQ(srv.methods.back(), "movej");
}

// ── DeviceManager ────────────────────────────────────────────────────────────

TEST(DeviceManager, LazyCreationAndReuse) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    auto& mgr = arm.devices();
    EXPECT_FALSE(mgr.contains("hand_0"));

    auto h1 = mgr.get("hand_0");
    EXPECT_TRUE(mgr.contains("hand_0"));
    auto h2 = mgr["hand_0"];
    EXPECT_EQ(h1.device_id(), h2.device_id());

    auto ids = mgr.device_ids();
    ASSERT_EQ(ids.size(), 1u);
    EXPECT_EQ(ids[0], "hand_0");
}

TEST(DeviceManager, DistinctDevicesRouteDistinctly) {
    MockServer srv;
    Arm arm("unused", "armA", srv.tp);

    auto& mgr = arm.devices();
    mgr["hand_0"].open();
    mgr["gripper_0"].open();
    EXPECT_EQ(srv.methods[0], "device.hand_0.open");
    EXPECT_EQ(srv.methods[1], "device.gripper_0.open");
}

} // namespace
} // namespace litearm
