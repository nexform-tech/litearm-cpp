#include <gtest/gtest.h>
#include "litearm/arm.hpp"
#include "litearm/codec.hpp"
#include "litearm/exceptions.hpp"
#include "litearm/protocol.hpp"
#include "litearm/transport.hpp"

namespace litearm {
namespace {

// ── Mock server helper ──────────────────────────────────────────────────────

/// Create an Arm + InProcTransport pair with a mock RPC handler.
struct MockArm {
    Arm arm;
    std::shared_ptr<InProcTransport> tp;
    std::shared_ptr<Sub> estop_sub;  // for verifying estop publishes
};

using KwargsMap = std::map<std::string, LiteArmValue>;

template<typename Handler>
MockArm make_mock_arm(const std::string& arm_id, Handler handler) {
    auto tp = std::make_shared<InProcTransport>();

    // Register the mock RPC handler
    tp->declare_queryable(rpc_topic(arm_id),
        [handler](const std::string& payload) -> std::string {
            auto [method, kwargs] = decode_request(payload);
            try {
                LiteArmValue result = handler(method, kwargs);
                return encode_reply_ok(result);
            } catch (const LiteArmError& e) {
                return encode_reply_error("LiteArmError", e.what());
            } catch (const std::exception& e) {
                return encode_reply_error("LiteArmError", e.what());
            }
        });

    auto estop_sub = tp->sub(estop_topic(arm_id));

    Arm arm("unused", arm_id, tp);
    return MockArm{std::move(arm), tp, estop_sub};
}

// ── FK / IK ─────────────────────────────────────────────────────────────────

TEST(ArmFkIk, ForwardKinematics) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "fk");
        EXPECT_EQ(kwargs.at("q").as_list().size(), 7u);
        // Return [position, rotation_matrix]
        LiteArmValue::ListType result;
        result.push_back(LiteArmValue::from_vec({1.0, 2.0, 3.0}));
        result.push_back(LiteArmValue::from_mat({
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
        }));
        return LiteArmValue(std::move(result));
    });

    auto [pos, rot] = mock.arm.fk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    EXPECT_EQ(pos.size(), 3u);
    EXPECT_DOUBLE_EQ(pos[0], 1.0);
    EXPECT_EQ(rot.size(), 3u);
    EXPECT_DOUBLE_EQ(rot[0][0], 1.0);
}

TEST(ArmFkIk, InverseKinematics) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "ik");
        EXPECT_EQ(kwargs.at("pos_d").as_list().size(), 3u);
        EXPECT_EQ(kwargs.at("R_d").as_list().size(), 3u);
        LiteArmValue::ListType result;
        result.push_back(LiteArmValue::from_vec({0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7}));
        result.push_back(LiteArmValue(true));
        return LiteArmValue(std::move(result));
    });

    auto [q, success] = mock.arm.ik(
        {1.0, 2.0, 3.0},
        {{1,0,0},{0,1,0},{0,0,1}}
    );
    EXPECT_TRUE(success);
    EXPECT_EQ(q.size(), 7u);
}

TEST(ArmFkIk, PlanMovel) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "plan_movel");
        LiteArmValue::ListType path;
        path.push_back(LiteArmValue::from_vec({0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7}));
        path.push_back(LiteArmValue::from_vec({0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}));
        return LiteArmValue(std::move(path));
    });

    auto path = mock.arm.plan_movel(
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        LiteArmValue(std::string("goal"))
    );
    EXPECT_EQ(path.size(), 2u);
    EXPECT_EQ(path[0].size(), 7u);
}

// ── Motion execution ────────────────────────────────────────────────────────

TEST(ArmMotion, Movej) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "movej");
        EXPECT_DOUBLE_EQ(kwargs.at("speed").as_double(), 0.5);
        EXPECT_DOUBLE_EQ(kwargs.at("settle_s").as_double(), 1.0);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.movej({0,0,0,0,0,0,0}, 0.5));
}

TEST(ArmMotion, MovejWithMaxCycles) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "movej");
        EXPECT_EQ(kwargs.at("max_cycles").as_int(), 100);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.movej({0,0,0,0,0,0,0}, 1.0, 1.0, 100));
}

TEST(ArmMotion, Movel) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "movel");
        EXPECT_DOUBLE_EQ(kwargs.at("speed").as_double(), 0.5);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.movel(LiteArmValue(std::string("goal")), 0.5));
}

TEST(ArmMotion, Movec) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "movec");
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.movec(
        LiteArmValue(std::string("via")),
        LiteArmValue(std::string("goal")),
        0.5));
}

TEST(ArmMotion, Movep) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "movep");
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.movep(LiteArmValue(std::string("waypoints")), 0.5));
}

TEST(ArmMotion, Hold) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "hold");
        EXPECT_DOUBLE_EQ(kwargs.at("kp_scale").as_double(), 3.0);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.hold(3.0));
}

TEST(ArmMotion, ZeroGravity) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "zero_gravity");
        EXPECT_TRUE(kwargs.at("max_cycles").is_null());
        EXPECT_TRUE(kwargs.at("duration_s").is_null());
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.zero_gravity());
}

TEST(ArmMotion, JointImpedance) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "joint_impedance");
        EXPECT_DOUBLE_EQ(kwargs.at("engage_sec").as_double(), 0.3);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.joint_impedance(
        {0,0,0,0,0,0,0},
        LiteArmValue::from_vec({100,100,100,100,100,100,100}),
        LiteArmValue::from_vec({10,10,10,10,10,10,10})));
}

TEST(ArmMotion, CartesianImpedance) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "cartesian_impedance");
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.cartesian_impedance(
        {0,0,0,0,0,0,0},
        LiteArmValue::from_vec({100,100,100,100,100,100}),
        LiteArmValue::from_vec({10,10,10,10,10,10})));
}

TEST(ArmMotion, JointFollow) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "joint_follow");
        EXPECT_DOUBLE_EQ(kwargs.at("engage_sec").as_double(), 0.3);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.joint_follow());
}

TEST(ArmMotion, ReplayJointPath) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "replay_joint_path");
        EXPECT_TRUE(kwargs.at("goto_start").as_bool());
        EXPECT_DOUBLE_EQ(kwargs.at("goto_speed").as_double(), 0.3);
        return LiteArmValue(true);
    });

    std::vector<std::vector<double>> path = {
        {0,0,0,0,0,0,0},
        {0.1,0.1,0.1,0.1,0.1,0.1,0.1},
    };
    EXPECT_TRUE(mock.arm.replay_joint_path(path));
}

TEST(ArmMotion, ReplayTrajectory) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "replay_trajectory");
        return LiteArmValue(true);
    });

    // Use raw LiteArmValue
    LiteArmValue::MapType traj;
    traj["name"] = LiteArmValue(std::string("test_traj"));
    traj["schema"] = LiteArmValue(std::string("pylitearm.joint_trajectory.v1"));

    LiteArmValue::ListType frames;
    for (int i = 0; i < 2; ++i) {
        LiteArmValue::MapType f;
        f["t"] = LiteArmValue(static_cast<double>(i));
        f["q"] = LiteArmValue::from_vec({0,0,0,0,0,0,0});
        frames.push_back(LiteArmValue(std::move(f)));
    }
    traj["frames"] = LiteArmValue(std::move(frames));

    EXPECT_TRUE(mock.arm.replay_trajectory(LiteArmValue(std::move(traj))));
}

// ── Record trajectory ───────────────────────────────────────────────────────

TEST(ArmRecord, RecordTrajectory) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "record_trajectory");
        EXPECT_EQ(kwargs.at("output").as_string(), "trajectories");
        EXPECT_DOUBLE_EQ(kwargs.at("sample_rate_hz").as_double(), 100.0);

        LiteArmValue::MapType result;
        result["schema"] = LiteArmValue(std::string("pylitearm.joint_trajectory.v1"));
        result["name"] = LiteArmValue(std::string("my_recording"));
        result["created_at"] = LiteArmValue(std::string("2024-01-01T00:00:00Z"));
        result["sample_rate_hz"] = LiteArmValue(100.0);
        result["filter_alpha"] = LiteArmValue(nullptr);
        result["robot_serial"] = LiteArmValue(nullptr);
        result["config_checksum_sha256"] = LiteArmValue(nullptr);

        LiteArmValue::ListType frames;
        for (int i = 0; i < 3; ++i) {
            LiteArmValue::MapType f;
            f["t"] = LiteArmValue(static_cast<double>(i) * 0.01);
            f["q"] = LiteArmValue::from_vec({0,0,0,0,0,0,0});
            frames.push_back(LiteArmValue(std::move(f)));
        }
        result["frames"] = LiteArmValue(std::move(frames));

        return LiteArmValue(std::move(result));
    });

    auto traj = mock.arm.record_trajectory();
    EXPECT_EQ(traj.name, "my_recording");
    EXPECT_EQ(traj.frames.size(), 3u);
    EXPECT_DOUBLE_EQ(traj.sample_rate_hz.value(), 100.0);
}

// ── Request stop ────────────────────────────────────────────────────────────

TEST(ArmStop, RequestStopPublishesEstop) {
    auto mock = make_mock_arm("armA", [](const std::string&, const std::map<std::string, LiteArmValue>&) {
        return LiteArmValue(nullptr);
    });

    mock.arm.request_stop();

    auto msg = mock.estop_sub->try_recv();
    ASSERT_TRUE(msg.has_value());
    EXPECT_TRUE(decode_estop(*msg));
}

TEST(ArmStop, ClearStop) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "clear_stop");
        return LiteArmValue(nullptr);
    });

    mock.arm.clear_stop();
}

TEST(ArmStop, Enable) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "enable");
        return LiteArmValue(nullptr);
    });

    mock.arm.enable();
}

TEST(ArmStop, Disable) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "disable");
        return LiteArmValue(nullptr);
    });

    mock.arm.disable();
}

// ── Get state ───────────────────────────────────────────────────────────────

TEST(ArmState, NoBroadcastReturnsNullopt) {
    auto mock = make_mock_arm("armA", [](const std::string&, const std::map<std::string, LiteArmValue>&) {
        return LiteArmValue(nullptr);
    });

    EXPECT_FALSE(mock.arm.get_state().has_value());
}

TEST(ArmState, SingleBroadcast) {
    auto mock = make_mock_arm("armA", [](const std::string&, const std::map<std::string, LiteArmValue>&) {
        return LiteArmValue(nullptr);
    });

    // Publish a state broadcast
    RobotState state;
    state.q = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    state.dq = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    state.tau = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    state.state = "ready";
    state.robot_serial = "SN001";
    mock.tp->pub(state_topic("armA"), encode_state(state));

    auto result = mock.arm.get_state();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->state, "ready");
    EXPECT_EQ(result->q.size(), 7u);
    EXPECT_DOUBLE_EQ(result->q[0], 0.1);
    EXPECT_EQ(result->robot_serial, "SN001");
}

TEST(ArmState, MultipleBroadcastsReturnsLatest) {
    auto mock = make_mock_arm("armA", [](const std::string&, const std::map<std::string, LiteArmValue>&) {
        return LiteArmValue(nullptr);
    });

    for (const char* state_name : {"ready", "moving", "holding"}) {
        RobotState state;
        state.state = state_name;
        mock.tp->pub(state_topic("armA"), encode_state(state));
    }

    auto result = mock.arm.get_state();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->state, "holding");
}

// ── Parameter tuning ────────────────────────────────────────────────────────

TEST(ArmParams, SetGains) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "set_gains");
        EXPECT_DOUBLE_EQ(kwargs.at("kp").as_double(), 100.0);
        LiteArmValue::MapType result;
        result["kp"] = LiteArmValue(100.0);
        result["kd"] = LiteArmValue(10.0);
        return LiteArmValue(std::move(result));
    });

    auto result = mock.arm.set_gains(LiteArmValue(100.0));
    EXPECT_DOUBLE_EQ(result.as_map().at("kp").as_double(), 100.0);
}

TEST(ArmParams, GetGains) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "get_gains");
        LiteArmValue::MapType result;
        result["kp"] = LiteArmValue(200.0);
        result["kd"] = LiteArmValue(20.0);
        return LiteArmValue(std::move(result));
    });

    auto result = mock.arm.get_gains();
    EXPECT_DOUBLE_EQ(result.as_map().at("kp").as_double(), 200.0);
}

TEST(ArmParams, ClearFaults) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "clear_faults");
        LiteArmValue::ListType faults;
        LiteArmValue::ListType pair;
        pair.push_back(LiteArmValue(int64_t(3)));
        pair.push_back(LiteArmValue(int64_t(1)));
        faults.push_back(LiteArmValue(std::move(pair)));
        return LiteArmValue(std::move(faults));
    });

    auto result = mock.arm.clear_faults();
    EXPECT_EQ(result.as_list().size(), 1u);
}

TEST(ArmParams, SetPayload) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "set_payload");
        EXPECT_DOUBLE_EQ(kwargs.at("mass").as_double(), 1.5);
        auto com = kwargs.at("com").to_vec();
        EXPECT_EQ(com.size(), 3u);
        EXPECT_DOUBLE_EQ(com[0], 0.01);
        LiteArmValue::MapType result;
        result["mass"] = LiteArmValue(1.5);
        return LiteArmValue(std::move(result));
    });

    auto result = mock.arm.set_payload(1.5, {0.01, 0.02, 0.03});
    EXPECT_DOUBLE_EQ(result.as_map().at("mass").as_double(), 1.5);
}

TEST(ArmParams, GetPayload) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "get_payload");
        LiteArmValue::MapType result;
        result["mass"] = LiteArmValue(0.0);
        return LiteArmValue(std::move(result));
    });

    auto result = mock.arm.get_payload();
    EXPECT_DOUBLE_EQ(result.as_map().at("mass").as_double(), 0.0);
}

TEST(ArmParams, SetInstallation) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "set_installation");
        EXPECT_TRUE(kwargs.at("gravity").is_null());
        LiteArmValue::MapType result;
        result["base_rpy"] = LiteArmValue::from_vec({0.0, 0.0, 0.0});
        return LiteArmValue(std::move(result));
    });

    std::vector<double> rpy = {0.0, 0.0, 0.0};
    auto result = mock.arm.set_installation(rpy);
    EXPECT_TRUE(result.is_map());
}

TEST(ArmParams, GetInstallation) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "get_installation");
        LiteArmValue::MapType result;
        result["base_rpy"] = LiteArmValue::from_vec({0.0, 0.0, 0.0});
        return LiteArmValue(std::move(result));
    });

    auto result = mock.arm.get_installation();
    EXPECT_TRUE(result.is_map());
}

TEST(ArmParams, GetTcpPose) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "get_tcp_pose");
        LiteArmValue::ListType result;
        result.push_back(LiteArmValue::from_vec({0.5, 0.0, 0.3}));
        result.push_back(LiteArmValue::from_mat({
            {1,0,0},{0,1,0},{0,0,1}
        }));
        return LiteArmValue(std::move(result));
    });

    auto [pos, rot] = mock.arm.get_tcp_pose();
    EXPECT_EQ(pos.size(), 3u);
    EXPECT_DOUBLE_EQ(pos[0], 0.5);
    EXPECT_EQ(rot.size(), 3u);
}

// ── Error handling ──────────────────────────────────────────────────────────

TEST(ArmError, SafetyViolationPropagates) {
    auto tp = std::make_shared<InProcTransport>();

    // Register a handler that always returns a safety error
    tp->declare_queryable(rpc_topic("armA"),
        [](const std::string&) -> std::string {
            std::map<std::string, std::string> details;
            details["motor"] = "J3";
            details["temp"] = "85";
            return encode_reply_error("SafetyViolationError", "overtemp", details);
        });

    Arm arm("unused", "armA", tp);

    try {
        arm.movej({0,0,0,0,0,0,0});
        FAIL() << "Expected SafetyViolationError";
    } catch (const SafetyViolationError& e) {
        EXPECT_STREQ(e.what(), "overtemp");
        EXPECT_EQ(e.details.at("motor"), "J3");
    }
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

TEST(ArmLifecycle, ArmIdAccessor) {
    auto mock = make_mock_arm("armB", [](const std::string&, const std::map<std::string, LiteArmValue>&) {
        return LiteArmValue(nullptr);
    });
    EXPECT_EQ(mock.arm.arm_id(), "armB");
}

TEST(ArmLifecycle, CloseIsIdempotent) {
    auto mock = make_mock_arm("armA", [](const std::string&, const std::map<std::string, LiteArmValue>&) {
        return LiteArmValue(nullptr);
    });
    mock.arm.close();
    mock.arm.close();  // Should not crash
}

// ── Unified params ──────────────────────────────────────────────────────────

TEST(ArmParams, MovejCollisionRecovery) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "movej");
        EXPECT_TRUE(kwargs.at("allow_start_collision_recovery").as_bool());
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.movej({0,0,0,0,0,0,0}, 1.0, 1.0, 100, true));
}

TEST(ArmParams, ZeroGravityNewParams) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "zero_gravity");
        EXPECT_DOUBLE_EQ(kwargs.at("measured_overspeed_factor").as_double(), 1.5);
        auto vel_max = kwargs.at("vel_max").to_vec();
        EXPECT_EQ(vel_max.size(), 7u);
        if (vel_max.size() == 7u) {
            EXPECT_DOUBLE_EQ(vel_max[0], 0.5);
        }
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.zero_gravity(std::nullopt, std::nullopt, 1.5,
                                      std::vector<double>{0.5,0.5,0.5,0.5,0.5,0.5,0.5}));
}

TEST(ArmParams, CartesianImpedanceNewParams) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "cartesian_impedance");
        EXPECT_DOUBLE_EQ(kwargs.at("sigma_min_thresh").as_double(), 0.01);
        EXPECT_DOUBLE_EQ(kwargs.at("max_ori_err").as_double(), 0.2);
        EXPECT_DOUBLE_EQ(kwargs.at("measured_overspeed_factor").as_double(), 1.5);
        auto vel_max = kwargs.at("vel_max").to_vec();
        EXPECT_EQ(vel_max.size(), 6u);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.cartesian_impedance(
        {0,0,0,0,0,0,0},
        LiteArmValue::from_vec({100,100,100,100,100,100}),
        LiteArmValue::from_vec({10,10,10,10,10,10}),
        std::nullopt, std::nullopt, 0.3, std::nullopt,
        0.01, 0.2, 1.5,
        std::vector<double>{1,1,1,1,1,1}));
}

TEST(ArmParams, RecoverJointLimits) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "recover_joint_limits");
        EXPECT_DOUBLE_EQ(kwargs.at("speed").as_double(), 0.05);
        EXPECT_DOUBLE_EQ(kwargs.at("settle_s").as_double(), 0.5);
        EXPECT_DOUBLE_EQ(kwargs.at("inset_rad").as_double(), 0.1);
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.recover_joint_limits(0.05, 0.5, std::nullopt, 0.1));
}

TEST(ArmMotion, ReplayTimedTrajectory) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "replay_timed_trajectory");
        auto traj_q = kwargs.at("traj_q").to_mat();
        EXPECT_EQ(traj_q.size(), 2u);
        if (traj_q.size() == 2u) {
            EXPECT_EQ(traj_q[0].size(), 7u);
        }
        auto traj_t = kwargs.at("traj_t").to_vec();
        EXPECT_EQ(traj_t.size(), 2u);
        EXPECT_DOUBLE_EQ(kwargs.at("simplify_tolerance_rad").as_double(), 0.01);
        return LiteArmValue(true);
    });

    std::vector<std::vector<double>> traj_q = {
        {0,0,0,0,0,0,0},
        {0.1,0.1,0.1,0.1,0.1,0.1,0.1},
    };
    EXPECT_TRUE(mock.arm.replay_timed_trajectory(traj_q, {0.0, 1.0}));
}

TEST(ArmMotion, PlayTrajectoryString) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "play_trajectory");
        EXPECT_EQ(kwargs.at("trajectory").as_string(), "trajectories/traj_001.json");
        EXPECT_TRUE(kwargs.at("verify_robot").as_bool());
        return LiteArmValue(true);
    });

    EXPECT_TRUE(mock.arm.play_trajectory("trajectories/traj_001.json"));
}

// ── Server extension RPCs ────────────────────────────────────────────────────

namespace {

LiteArmValue ok_map() {
    LiteArmValue::MapType m;
    m["ok"] = LiteArmValue(true);
    return LiteArmValue(std::move(m));
}

} // namespace

TEST(ArmExtension, GetSystemStats) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "get_system_stats");
        LiteArmValue::MapType m;
        m["cpu_percent"] = LiteArmValue(12.3);
        m["uptime_seconds"] = LiteArmValue(int64_t(42));
        return LiteArmValue(std::move(m));
    });

    auto result = mock.arm.get_system_stats();
    EXPECT_DOUBLE_EQ(result.as_map().at("cpu_percent").as_double(), 12.3);
}

TEST(ArmExtension, GetLogs) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        EXPECT_EQ(method, "get_logs");
        EXPECT_EQ(kwargs.at("page").as_int(), 2);
        EXPECT_EQ(kwargs.at("size").as_int(), 10);
        EXPECT_EQ(kwargs.at("search").as_string(), "movej");
        return ok_map();
    });

    auto result = mock.arm.get_logs(2, 10, "movej");
    EXPECT_TRUE(result.as_map().at("ok").as_bool());
}

TEST(ArmExtension, RestartService) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>&) {
        EXPECT_EQ(method, "restart_service");
        return ok_map();
    });

    EXPECT_TRUE(mock.arm.restart_service().as_map().at("ok").as_bool());
}

TEST(ArmExtension, SettingsRoundtrip) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        if (method == "get_joint_limits" || method == "get_zero_offsets" ||
            method == "get_end_effector" || method == "get_cartesian_limits" ||
            method == "get_collision_config") {
            return LiteArmValue(LiteArmValue::MapType{});
        }
        // set_* methods echo the payload back
        if (kwargs.count("limits")) return kwargs.at("limits");
        if (kwargs.count("offsets")) return kwargs.at("offsets");
        if (kwargs.count("config")) return kwargs.at("config");
        return ok_map();
    });

    EXPECT_TRUE(mock.arm.get_joint_limits().is_map());
    LiteArmValue::MapType limits;
    limits["q_max"] = LiteArmValue::from_vec({3.0,3.0,3.0,3.0,3.0,3.0,3.0});
    auto echoed = mock.arm.set_joint_limits(LiteArmValue(limits));
    EXPECT_DOUBLE_EQ(echoed.as_map().at("q_max").to_vec()[0], 3.0);

    EXPECT_TRUE(mock.arm.set_zero_offsets(ok_map()).is_map());
    EXPECT_TRUE(mock.arm.set_end_effector(ok_map()).is_map());
    EXPECT_TRUE(mock.arm.set_cartesian_limits(ok_map()).is_map());
    EXPECT_TRUE(mock.arm.set_collision_config(ok_map()).is_map());
}

TEST(ArmExtension, TrajectoryCrud) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        if (method == "list_trajectories") {
            LiteArmValue::MapType m;
            m["total"] = LiteArmValue(int64_t(1));
            return LiteArmValue(std::move(m));
        }
        if (method == "save_trajectory") {
            EXPECT_EQ(kwargs.at("id").as_string(), "t1");
            EXPECT_EQ(kwargs.at("name").as_string(), "demo");
            EXPECT_EQ(kwargs.at("points").to_mat().size(), 2u);
            return ok_map();
        }
        if (method == "delete_trajectory") {
            EXPECT_EQ(kwargs.at("id").as_string(), "t1");
            return ok_map();
        }
        return ok_map();
    });

    EXPECT_TRUE(mock.arm.start_recording().is_map());
    EXPECT_TRUE(mock.arm.stop_recording().is_map());
    EXPECT_TRUE(mock.arm.discard_recording().is_map());
    EXPECT_TRUE(mock.arm.get_recording_state().is_map());
    EXPECT_TRUE(mock.arm.get_playback_state().is_map());
    EXPECT_EQ(mock.arm.list_trajectories().as_map().at("total").as_int(), 1);

    std::vector<std::vector<double>> points = {
        {0,0,0,0,0,0,0},
        {0.1,0.1,0.1,0.1,0.1,0.1,0.1},
    };
    EXPECT_TRUE(mock.arm.save_trajectory("t1", "demo", points).as_map().at("ok").as_bool());
    EXPECT_TRUE(mock.arm.delete_trajectory("t1").as_map().at("ok").as_bool());
}

TEST(ArmExtension, DeviceControl) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        if (method == "list_device_types") {
            return LiteArmValue(LiteArmValue::ListType{});
        }
        if (method == "connect_device") {
            EXPECT_EQ(kwargs.at("category").as_string(), "hand");
            EXPECT_EQ(kwargs.at("subtype").as_string(), "lite6_hand");
            EXPECT_EQ(kwargs.at("device_id").as_string(), "end_0");
            return ok_map();
        }
        if (method == "disconnect_device" || method == "get_active_device") {
            EXPECT_EQ(kwargs.at("device_id").as_string(), "end_0");
            return ok_map();
        }
        return ok_map();
    });

    EXPECT_TRUE(mock.arm.list_device_types().is_list());
    EXPECT_TRUE(mock.arm.connect_device("hand", "lite6_hand").as_map().at("ok").as_bool());
    EXPECT_TRUE(mock.arm.disconnect_device().as_map().at("ok").as_bool());
    EXPECT_TRUE(mock.arm.get_active_device().as_map().at("ok").as_bool());
}

TEST(ArmExtension, Teleop) {
    auto mock = make_mock_arm("armA", [](const std::string& method, const std::map<std::string, LiteArmValue>& kwargs) {
        if (method == "enter_teleop") {
            EXPECT_EQ(kwargs.at("mode").as_string(), "slave");
            EXPECT_EQ(kwargs.at("peer").as_string(), "tcp/10.0.0.1:7447");
            return ok_map();
        }
        return ok_map();
    });

    std::map<std::string, LiteArmValue> params;
    params["peer"] = LiteArmValue(std::string("tcp/10.0.0.1:7447"));
    EXPECT_TRUE(mock.arm.enter_teleop("slave", params).as_map().at("ok").as_bool());
    EXPECT_TRUE(mock.arm.exit_teleop().is_map());
    EXPECT_TRUE(mock.arm.get_teleop_status().is_map());
}

} // namespace
} // namespace litearm
