#include <gtest/gtest.h>
#include "litearm/codec.hpp"
#include "litearm/exceptions.hpp"
#include "litearm/types.hpp"
#include "litearm.pb.h"

namespace litearm {
namespace {

// ── Value conversion roundtrip ──────────────────────────────────────────────

TEST(CodecValue, NullRoundtrip) {
    LiteArmValue v(nullptr);
    auto pv = to_proto(v);
    auto back = from_proto(pv);
    EXPECT_TRUE(back.is_null());
}

TEST(CodecValue, BoolRoundtrip) {
    for (bool b : {true, false}) {
        LiteArmValue v(b);
        auto pv = to_proto(v);
        auto back = from_proto(pv);
        EXPECT_TRUE(back.is_bool());
        EXPECT_EQ(back.as_bool(), b);
    }
}

TEST(CodecValue, IntRoundtrip) {
    for (int64_t i : {int64_t(0), int64_t(42), int64_t(-1), int64_t(1LL << 40)}) {
        LiteArmValue v(i);
        auto pv = to_proto(v);
        auto back = from_proto(pv);
        EXPECT_TRUE(back.is_int());
        EXPECT_EQ(back.as_int(), i);
    }
}

TEST(CodecValue, DoubleRoundtrip) {
    for (double d : {0.0, 3.14, -2.718, 1e-10}) {
        LiteArmValue v(d);
        auto pv = to_proto(v);
        auto back = from_proto(pv);
        EXPECT_TRUE(back.is_double());
        EXPECT_DOUBLE_EQ(back.as_double(), d);
    }
}

TEST(CodecValue, StringRoundtrip) {
    for (const auto& s : {"", "hello", "litearm/v4/armA/rpc"}) {
        std::string str(s);
        LiteArmValue v(str);
        auto pv = to_proto(v);
        auto back = from_proto(pv);
        EXPECT_TRUE(back.is_string());
        EXPECT_EQ(back.as_string(), str);
    }
}

TEST(CodecValue, ListRoundtrip) {
    LiteArmValue::ListType list;
    list.push_back(LiteArmValue(1.0));
    list.push_back(LiteArmValue(2.0));
    list.push_back(LiteArmValue(3.0));
    LiteArmValue v(std::move(list));

    auto pv = to_proto(v);
    auto back = from_proto(pv);
    EXPECT_TRUE(back.is_list());
    EXPECT_EQ(back.as_list().size(), 3u);
    EXPECT_DOUBLE_EQ(back.as_list()[0].as_double(), 1.0);
    EXPECT_DOUBLE_EQ(back.as_list()[2].as_double(), 3.0);
}

TEST(CodecValue, MapRoundtrip) {
    LiteArmValue::MapType m;
    m["key"] = LiteArmValue(std::string("value"));
    m["num"] = LiteArmValue(42.0);
    LiteArmValue v(std::move(m));

    auto pv = to_proto(v);
    auto back = from_proto(pv);
    EXPECT_TRUE(back.is_map());
    EXPECT_EQ(back.as_map().at("key").as_string(), "value");
    EXPECT_DOUBLE_EQ(back.as_map().at("num").as_double(), 42.0);
}

TEST(CodecValue, NestedRoundtrip) {
    // Nested: list containing maps
    LiteArmValue::MapType inner;
    inner["name"] = LiteArmValue(std::string("joint_0"));
    inner["value"] = LiteArmValue(1.5);

    LiteArmValue::ListType list;
    list.push_back(LiteArmValue(std::move(inner)));
    LiteArmValue v(std::move(list));

    auto pv = to_proto(v);
    auto back = from_proto(pv);
    EXPECT_TRUE(back.is_list());
    EXPECT_EQ(back.as_list()[0].as_map().at("name").as_string(), "joint_0");
}

TEST(CodecValue, VecConversion) {
    std::vector<double> vec = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    auto v = LiteArmValue::from_vec(vec);
    auto back = v.to_vec();
    EXPECT_EQ(back, vec);
}

TEST(CodecValue, MatConversion) {
    std::vector<std::vector<double>> mat = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
    };
    auto v = LiteArmValue::from_mat(mat);
    auto back = v.to_mat();
    EXPECT_EQ(back, mat);
}

TEST(CodecValue, BadCastThrows) {
    LiteArmValue v(42.0);
    EXPECT_THROW(v.as_bool(), std::bad_cast);
    EXPECT_THROW(v.as_string(), std::bad_cast);
    EXPECT_THROW(v.as_list(), std::bad_cast);
}

// ── Request encoding/decoding ───────────────────────────────────────────────

TEST(CodecRequest, EmptyKwargs) {
    std::map<std::string, LiteArmValue> kwargs;
    auto payload = encode_request("get_gains", kwargs);
    auto [method, decoded] = decode_request(payload);
    EXPECT_EQ(method, "get_gains");
    EXPECT_TRUE(decoded.empty());
}

TEST(CodecRequest, SimpleKwargs) {
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["speed"] = LiteArmValue(0.5);
    kwargs["name"] = LiteArmValue(std::string("test"));

    auto payload = encode_request("movej", kwargs);
    auto [method, decoded] = decode_request(payload);
    EXPECT_EQ(method, "movej");
    EXPECT_DOUBLE_EQ(decoded.at("speed").as_double(), 0.5);
    EXPECT_EQ(decoded.at("name").as_string(), "test");
}

TEST(CodecRequest, NestedKwargs) {
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["q_target"] = LiteArmValue::from_vec({0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
    kwargs["max_cycles"] = LiteArmValue(nullptr);

    auto payload = encode_request("movej", kwargs);
    auto [method, decoded] = decode_request(payload);
    EXPECT_EQ(method, "movej");
    auto q = decoded.at("q_target").to_vec();
    EXPECT_EQ(q.size(), 7u);
    EXPECT_DOUBLE_EQ(q[0], 0.1);
    EXPECT_TRUE(decoded.at("max_cycles").is_null());
}

// ── Reply encoding/decoding ─────────────────────────────────────────────────

TEST(CodecReply, OkNull) {
    auto payload = encode_reply_ok(LiteArmValue(nullptr));
    auto result = decode_reply(payload);
    EXPECT_TRUE(result.is_null());
}

TEST(CodecReply, OkBool) {
    auto payload = encode_reply_ok(LiteArmValue(true));
    auto result = decode_reply(payload);
    EXPECT_TRUE(result.as_bool());
}

TEST(CodecReply, OkList) {
    LiteArmValue::ListType list;
    list.push_back(LiteArmValue(1.0));
    list.push_back(LiteArmValue(2.0));
    auto payload = encode_reply_ok(LiteArmValue(std::move(list)));
    auto result = decode_reply(payload);
    EXPECT_EQ(result.as_list().size(), 2u);
}

TEST(CodecReply, OkMap) {
    LiteArmValue::MapType m;
    m["kp"] = LiteArmValue(100.0);
    m["kd"] = LiteArmValue(10.0);
    auto payload = encode_reply_ok(LiteArmValue(std::move(m)));
    auto result = decode_reply(payload);
    EXPECT_DOUBLE_EQ(result.as_map().at("kp").as_double(), 100.0);
}

TEST(CodecReply, ErrorLiteArmError) {
    auto payload = encode_reply_error("LiteArmError", "something went wrong");
    EXPECT_THROW(decode_reply(payload), LiteArmError);
    try {
        decode_reply(payload);
    } catch (const LiteArmError& e) {
        EXPECT_STREQ(e.what(), "something went wrong");
    }
}

TEST(CodecReply, ErrorSafetyViolation) {
    std::map<std::string, std::string> details;
    details["joint"] = "3";
    details["reason"] = "overtemp";
    auto payload = encode_reply_error("SafetyViolationError", "safety!", details);

    try {
        decode_reply(payload);
        FAIL() << "Expected SafetyViolationError";
    } catch (const SafetyViolationError& e) {
        EXPECT_STREQ(e.what(), "safety!");
        EXPECT_EQ(e.details.at("joint"), "3");
        EXPECT_EQ(e.details.at("reason"), "overtemp");
    }
}

TEST(CodecReply, ErrorFeedbackTimeout) {
    auto payload = encode_reply_error("FeedbackTimeoutError", "stale feedback");
    try {
        decode_reply(payload);
        FAIL() << "Expected FeedbackTimeoutError";
    } catch (const FeedbackTimeoutError& e) {
        EXPECT_STREQ(e.what(), "stale feedback");
    }
}

TEST(CodecReply, ErrorMotionCancelled) {
    auto payload = encode_reply_error("MotionCancelled", "user stop");
    try {
        decode_reply(payload);
        FAIL() << "Expected MotionCancelled";
    } catch (const MotionCancelled& e) {
        EXPECT_STREQ(e.what(), "user stop");
    }
}

TEST(CodecReply, ErrorConfigurationError) {
    auto payload = encode_reply_error("ConfigurationError", "bad config");
    try {
        decode_reply(payload);
        FAIL() << "Expected ConfigurationError";
    } catch (const ConfigurationError& e) {
        EXPECT_STREQ(e.what(), "bad config");
    }
}

TEST(CodecReply, ErrorTransportError) {
    auto payload = encode_reply_error("TransportError", "can write timeout");
    try {
        decode_reply(payload);
        FAIL() << "Expected TransportError";
    } catch (const TransportError& e) {
        EXPECT_STREQ(e.what(), "can write timeout");
    }
}

TEST(CodecReply, ErrorUnknownTypeFallsBackToBase) {
    auto payload = encode_reply_error("SomeNewError", "unknown error type");
    try {
        decode_reply(payload);
        FAIL() << "Expected LiteArmError";
    } catch (const LiteArmError& e) {
        EXPECT_STREQ(e.what(), "unknown error type");
    }
}

TEST(CodecReply, ErrorCartesianPlan) {
    auto payload = encode_reply_error("CartesianPlanError", "IK failed");
    try {
        decode_reply(payload);
        FAIL() << "Expected CartesianPlanError";
    } catch (const CartesianPlanError& e) {
        EXPECT_STREQ(e.what(), "IK failed");
        EXPECT_EQ(e.index, -1);
    }
}

// ── State encoding/decoding ─────────────────────────────────────────────────

TEST(CodecState, FullRoundtrip) {
    RobotState state;
    state.q = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    state.dq = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    state.tau = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    state.errs = {0, 0, 0, 0, 0, 0, 0};
    state.faults = {{3, 1}};
    state.temps = {{40, 45}, {38, 42}, {41, 44}, {39, 43}, {40, 46}, {37, 41}, {42, 47}};
    state.state = "ready";

    state.feedback.max_age_s = 0.5;
    state.feedback.stale_joints = {2};
    state.feedback.joints.push_back({0, 100, 0.01, true});

    state.watchdog.enabled = true;
    state.watchdog.timeout_s = 1.0;
    state.watchdog.mode = "soft";
    state.watchdog.tripped = false;
    state.watchdog.last_kick_age_s = 0.1;

    state.robot_serial = "SN12345";
    state.config_checksum_sha256 = "abc123def456";

    auto payload = encode_state(state);
    auto decoded = decode_state(payload);

    EXPECT_EQ(decoded.q, state.q);
    EXPECT_EQ(decoded.dq, state.dq);
    EXPECT_EQ(decoded.tau, state.tau);
    EXPECT_EQ(decoded.errs, state.errs);
    EXPECT_EQ(decoded.faults.size(), 1u);
    EXPECT_EQ(decoded.faults[0].joint, 3);
    EXPECT_EQ(decoded.faults[0].err_code, 1);
    EXPECT_EQ(decoded.temps.size(), 7u);
    EXPECT_EQ(decoded.temps[0].mos_temp, 40);
    EXPECT_EQ(decoded.state, "ready");
    EXPECT_DOUBLE_EQ(decoded.feedback.max_age_s, 0.5);
    EXPECT_EQ(decoded.feedback.stale_joints.size(), 1u);
    EXPECT_EQ(decoded.feedback.joints.size(), 1u);
    EXPECT_TRUE(decoded.watchdog.enabled);
    EXPECT_DOUBLE_EQ(decoded.watchdog.timeout_s, 1.0);
    EXPECT_EQ(decoded.watchdog.mode, "soft");
    EXPECT_EQ(decoded.robot_serial, "SN12345");
    EXPECT_EQ(decoded.config_checksum_sha256, "abc123def456");
}

TEST(CodecState, EmptyState) {
    RobotState state;
    state.state = "disconnected";

    auto payload = encode_state(state);
    auto decoded = decode_state(payload);
    EXPECT_EQ(decoded.state, "disconnected");
    EXPECT_TRUE(decoded.q.empty());
}

// ── Estop encoding/decoding ─────────────────────────────────────────────────

TEST(CodecEstop, TriggerTrue) {
    auto payload = encode_estop(true);
    EXPECT_TRUE(decode_estop(payload));
}

TEST(CodecEstop, TriggerFalse) {
    auto payload = encode_estop(false);
    EXPECT_FALSE(decode_estop(payload));
}

TEST(CodecEstop, InvalidPayloadReturnsFalse) {
    EXPECT_FALSE(decode_estop("invalid data"));
}

} // namespace
} // namespace litearm
