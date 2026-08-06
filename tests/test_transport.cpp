#include <gtest/gtest.h>
#include "litearm/transport.hpp"
#include "litearm/codec.hpp"

#include <thread>

namespace litearm {
namespace {

// ── Pub/Sub ─────────────────────────────────────────────────────────────────

TEST(InProcPubSub, BasicPubSub) {
    InProcTransport tp;
    auto sub = tp.sub("topic/a");
    tp.pub("topic/a", "hello");
    auto msg = sub->try_recv();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(*msg, "hello");
    // Queue should be empty now
    EXPECT_FALSE(sub->try_recv().has_value());
}

TEST(InProcPubSub, MultipleSubscribers) {
    InProcTransport tp;
    auto sub1 = tp.sub("topic/a");
    auto sub2 = tp.sub("topic/a");

    tp.pub("topic/a", "msg1");

    EXPECT_EQ(*sub1->try_recv(), "msg1");
    EXPECT_EQ(*sub2->try_recv(), "msg1");
}

TEST(InProcPubSub, TopicIsolation) {
    InProcTransport tp;
    auto sub_a = tp.sub("topic/a");
    auto sub_b = tp.sub("topic/b");

    tp.pub("topic/a", "for_a");

    EXPECT_EQ(*sub_a->try_recv(), "for_a");
    EXPECT_FALSE(sub_b->try_recv().has_value());
}

TEST(InProcPubSub, FifoDepthOverflow) {
    InProcTransport tp(3);  // depth=3
    auto sub = tp.sub("topic/a");

    for (int i = 0; i < 10; ++i) {
        tp.pub("topic/a", std::to_string(i));
    }

    // Only last 3 should survive
    auto m1 = sub->try_recv();
    auto m2 = sub->try_recv();
    auto m3 = sub->try_recv();
    auto m4 = sub->try_recv();

    ASSERT_TRUE(m1.has_value());
    ASSERT_TRUE(m2.has_value());
    ASSERT_TRUE(m3.has_value());
    EXPECT_FALSE(m4.has_value());
    EXPECT_EQ(*m1, "7");
    EXPECT_EQ(*m2, "8");
    EXPECT_EQ(*m3, "9");
}

// ── Drain latest ────────────────────────────────────────────────────────────

TEST(InProcDrain, EmptyQueue) {
    InProcTransport tp;
    auto sub = tp.sub("topic/a");
    EXPECT_FALSE(sub->drain_latest().has_value());
}

TEST(InProcDrain, SingleMessage) {
    InProcTransport tp;
    auto sub = tp.sub("topic/a");
    tp.pub("topic/a", "only_one");
    auto latest = sub->drain_latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(*latest, "only_one");
}

TEST(InProcDrain, MultipleMessages) {
    InProcTransport tp;
    auto sub = tp.sub("topic/a");
    tp.pub("topic/a", "first");
    tp.pub("topic/a", "second");
    tp.pub("topic/a", "third");

    auto latest = sub->drain_latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(*latest, "third");
    // Queue should be drained
    EXPECT_FALSE(sub->try_recv().has_value());
}

// ── Query/Reply ─────────────────────────────────────────────────────────────

TEST(InProcQuery, BasicQueryReply) {
    InProcTransport tp;
    tp.declare_queryable("rpc", [](const std::string& payload) {
        return "echo:" + payload;
    });

    auto reply = tp.query("rpc", "hello");
    EXPECT_EQ(reply, "echo:hello");
}

TEST(InProcQuery, MissingQueryableThrows) {
    InProcTransport tp;
    EXPECT_THROW(tp.query("missing", "data"), std::runtime_error);
}

TEST(InProcQuery, DoubleRegistrationThrows) {
    InProcTransport tp;
    tp.declare_queryable("rpc", [](const std::string& p) { return p; });
    EXPECT_THROW(
        tp.declare_queryable("rpc", [](const std::string& p) { return p; }),
        std::runtime_error
    );
}

TEST(InProcQuery, HandlerExceptionPropagates) {
    InProcTransport tp;
    tp.declare_queryable("rpc", [](const std::string&) -> std::string {
        throw std::runtime_error("handler error");
    });

    EXPECT_THROW(tp.query("rpc", "data"), std::runtime_error);
}

TEST(InProcQuery, ComplexPayloadRoundtrip) {
    InProcTransport tp;

    // Register a mock handler that decodes requests and returns replies
    tp.declare_queryable("rpc", [](const std::string& payload) -> std::string {
        auto [method, kwargs] = decode_request(payload);
        if (method == "echo") {
            return encode_reply_ok(kwargs.at("data"));
        }
        return encode_reply_error("LiteArmError", "unknown method");
    });

    // Send a request
    std::map<std::string, LiteArmValue> kwargs;
    kwargs["data"] = LiteArmValue(std::string("test_value"));
    auto req = encode_request("echo", kwargs);

    auto reply = tp.query("rpc", req);
    auto result = decode_reply(reply);
    EXPECT_EQ(result.as_string(), "test_value");
}

// ── Close ───────────────────────────────────────────────────────────────────

TEST(InProcClose, ClearStateOnClose) {
    InProcTransport tp;
    auto sub = tp.sub("topic/a");
    tp.declare_queryable("rpc", [](const std::string& p) { return p; });

    tp.pub("topic/a", "msg");
    tp.close();

    // Publishing after close should not crash (silently ignored)
    tp.pub("topic/a", "after_close");
    // Querying after close should fail
    EXPECT_THROW(tp.query("rpc", "data"), std::runtime_error);
}

// ── Factory ─────────────────────────────────────────────────────────────────

TEST(TransportFactory, InProcKind) {
    auto tp = make_transport("inproc");
    ASSERT_NE(tp, nullptr);
}

TEST(TransportFactory, UnknownKindThrows) {
    EXPECT_THROW(make_transport("unknown"), std::runtime_error);
}

} // namespace
} // namespace litearm
