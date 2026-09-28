// 旋转/位姿工具 —— rpy<->矩阵、SO(3) 指数/对数、slerp、位姿误差、形态归一化。
#include "test_support.hpp"

using namespace litearm;
using namespace litearm::rot;

namespace {
constexpr double kPi = 3.14159265358979323846;

Mat3 m(std::initializer_list<double> v) {
    Mat3 out{};
    int i = 0;
    for (double x : v) out[size_t(i / 3)][size_t(i % 3)] = x, ++i;
    return out;
}
}  // namespace

TEST(rot_eye_and_basic_ops) {
    const Mat3 I = eye3();
    CHECK_NEAR(I[0][0], 1.0, 1e-15);
    CHECK_NEAR(I[0][1], 0.0, 1e-15);
    const Mat3 A = m({1, 2, 3, 4, 5, 6, 7, 8, 9});
    const Mat3 At = mat_T(A);
    CHECK_NEAR(At[0][1], 4.0, 1e-15);
    CHECK_NEAR(At[2][0], 3.0, 1e-15);
    const Vec3 v{1.0, 1.0, 1.0};
    const auto Av = mat_vec(A, v);
    CHECK_NEAR(Av[0], 6.0, 1e-12);
    CHECK_NEAR(Av[2], 24.0, 1e-12);
    CHECK_NEAR(dot({1, 2, 3}, {4, 5, 6}), 32.0, 1e-12);
    CHECK_NEAR(norm({3, 4, 0}), 5.0, 1e-12);
}

TEST(rot_cross_product_handedness) {
    const Vec3 x = cross({1, 0, 0}, {0, 1, 0});
    CHECK_NEAR(x[0], 0.0, 1e-15);
    CHECK_NEAR(x[1], 0.0, 1e-15);
    CHECK_NEAR(x[2], 1.0, 1e-15);
}

TEST(rot_unit_rejects_zero_vector)
{
    CHECK_THROWS_AS(unit({0.0, 0.0, 0.0}), InvalidCommandError);
    const auto u = unit({0.0, 3.0, 4.0});
    CHECK_NEAR(u[1], 0.6, 1e-12);
}

TEST(rot_rpy_to_mat_is_rz_ry_rx) {
    // R = Rz(yaw) * Ry(pitch) * Rx(roll), 与固件 kin_rpy_to_rot 同式。
    const double r = 0.3, p = -0.4, y = 0.5;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    const Mat3 expect = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                          {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                          {-sp, cp * sr, cp * cr}}};
    const Mat3 got = rpy_to_mat({r, p, y});
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) CHECK_NEAR(got[i][j], expect[i][j], 1e-15);
    }
}

TEST(rot_rpy_mat_roundtrip) {
    const std::vector<Vec3> cases = {
        {0.0, 0.0, 0.0},   {0.3, -0.4, 0.5},  {-1.0, 0.2, 2.0},
        {kPi / 2 - 0.2, 0.1, -1.2},  {0.0, 0.9, 0.0},
    };
    for (const auto& rpy : cases) {
        const Mat3 R = rpy_to_mat(rpy);
        CHECK(is_rotation(R));
        const Vec3 back = mat_to_rpy(R);
        // rpy 分量可以不同 (周期性), 但**同一个旋转**必须复现。
        const Mat3 R2 = rpy_to_mat(back);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) CHECK_NEAR(R2[i][j], R[i][j], 1e-9);
        }
    }
}

TEST(rot_mat_to_rpy_forces_yaw_zero_at_gimbal_lock) {
    // |pitch| ~= pi/2 (万向锁) 时强制 yaw=0 —— 与固件 kin_rot_to_rpy 的锁分支同约定。
    const Mat3 R = rpy_to_mat({0.7, kPi / 2, 0.9});
    const Vec3 rpy = mat_to_rpy(R);
    CHECK_NEAR(rpy[2], 0.0, 1e-12);
    CHECK_NEAR(std::fabs(rpy[1]), kPi / 2, 1e-5);
}

TEST(rot_log_exp_roundtrip) {
    const std::vector<Vec3> ws = {
        {1e-13, 0.0, 0.0},  {0.5, 0.0, 0.0},  {0.1, -0.2, 0.3},  {2.0, 1.0, -0.5},
    };
    for (const auto& w : ws) {
        const Mat3 R = rot_exp(w);
        CHECK(is_rotation(R));
        const Vec3 back = rot_log(R);
        CHECK_NEAR(norm(sub(back, w)), 0.0, 1e-9);
    }
}

TEST(rot_log_of_identity_is_zero) {
    const Vec3 w = rot_log(eye3());
    CHECK_NEAR(norm(w), 0.0, 1e-15);
}

TEST(rot_log_clamps_acos_argument) {
    // 数值噪声会让 trace 越界到 +-(1+eps) 然后抛 math domain error —— 必须夹住。
    // 这里给一个"几乎但不完全正交"的矩阵: 归一化后 trace 会略超 3。
    Mat3 R = eye3();
    R[0][0] = 1.0 + 1e-12;
    R[1][1] = 1.0 + 1e-12;
    R[2][2] = 1.0 + 1e-12;
    CHECK_NOTHROW(rot_log(R));
}

TEST(rot_slerp_hits_both_ends_and_the_midpoint) {
    const Mat3 R0 = rpy_to_mat({0.0, 0.0, 0.0});
    const Mat3 R1 = rpy_to_mat({0.0, 0.0, 1.0});
    const Mat3 a = rot_slerp(R0, R1, 0.0);
    const Mat3 b = rot_slerp(R0, R1, 1.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK_NEAR(a[i][j], R0[i][j], 1e-12);
            CHECK_NEAR(b[i][j], R1[i][j], 1e-12);
        }
    }
    const Mat3 mid = rot_slerp(R0, R1, 0.5);
    // 半程的转角应当是全程的一半
    CHECK_NEAR(rot_angle(mid), rot_angle(mat_mul(mat_T(R0), R1)) / 2.0, 1e-9);
}

TEST(rot_slerp_of_identical_rotations_is_that_rotation) {
    const Mat3 R = rpy_to_mat({0.2, 0.3, 0.4});
    const Mat3 got = rot_slerp(R, R, 0.37);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) CHECK_NEAR(got[i][j], R[i][j], 1e-12);
    }
}

TEST(rot_pose_err_orientation_difference_has_the_right_direction) {
    // ⚠ 姿态差必须是 rot_log(R_d * R^T), 不能写成 rot_log(R * R_d^T) —— 后者是复合不是差,
    // 符号相反; 曾因此把 0.59mm 位置误差的位姿报成 153 度姿态误差 (恰好是该位姿转角的 2 倍)。
    const Mat3 R = rpy_to_mat({0.0, 0.0, 0.4});
    const Mat3 R_d = rpy_to_mat({0.0, 0.0, 0.6});
    const auto e = pose_err({0.0, 0.0, 0.0}, R, {0.00059, 0.0, 0.0}, R_d);
    CHECK_NEAR(e.first, 0.00059, 1e-9);
    // 真值是 0.2 rad 的差, 不是 2 倍的复合角
    CHECK_NEAR(e.second, 0.2, 1e-9);
}

TEST(rot_is_rotation_rejects_non_so3) {
    CHECK(is_rotation(eye3()));
    Mat3 scaled = eye3();
    scaled[0][0] = 2.0;
    CHECK(!is_rotation(scaled));
    // det = -1 的反射不是 SO(3)
    Mat3 refl = eye3();
    refl[2][2] = -1.0;
    CHECK(!is_rotation(refl));
    Mat3 zero{};
    CHECK(!is_rotation(zero));
}

TEST(rot_orient_angle_is_zero_for_equivalent_rpy) {
    // 同一个旋转的两组不同 rpy (绕 z 加 2*pi) 之间的夹角是 0。
    CHECK_NEAR(orient_angle({0.0, 0.0, 0.3}, {0.0, 0.0, 0.3 + 2 * kPi}), 0.0, 1e-9);
    CHECK_NEAR(orient_angle({0.1, 0.2, 0.3}, {0.1, 0.2, 0.3}), 0.0, 1e-12);
    CHECK(orient_angle({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}) > 0.9);
}

// ---------------------------------------------------------------- PoseInput

TEST(rot_as_pose_accepts_a_six_vector) {
    const std::array<double, 6> v{0.3, 0.0, 0.4, 0.0, 0.0, 0.0};
    const Pose p = as_pose(v);
    CHECK_NEAR(p.first[0], 0.3, 1e-12);
    CHECK_NEAR(p.first[2], 0.4, 1e-12);
    CHECK_NEAR(p.second[0][0], 1.0, 1e-12);
}

TEST(rot_as_pose_accepts_pos_plus_rotation_matrix) {
    const Vec3 pos{1.0, 2.0, 3.0};
    const Mat3 R = rpy_to_mat({0.1, 0.2, 0.3});
    const Pose p = as_pose(pos, R);
    CHECK_NEAR(p.first[1], 2.0, 1e-12);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) CHECK_NEAR(p.second[i][j], R[i][j], 1e-12);
    }
}

TEST(rot_as_pose_accepts_a_homogeneous_4x4) {
    const Mat3 R = rpy_to_mat({0.0, 0.0, 0.5});
    Mat4 M{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) M[size_t(i)][size_t(j)] = R[i][j];
    }
    M[0][3] = 0.11;
    M[1][3] = 0.22;
    M[2][3] = 0.33;
    M[3][3] = 1.0;
    const Pose p = as_pose(M);
    CHECK_NEAR(p.first[0], 0.11, 1e-12);
    CHECK_NEAR(p.first[2], 0.33, 1e-12);
    CHECK_NEAR(p.second[0][1], R[0][1], 1e-12);
}

TEST(rot_as_pose_rejects_invalid_shapes_with_a_readable_message) {
    // 形态不合法必须抛 InvalidCommandError, 且文案里要带**实际收到的形状** ——
    // 只说"pose 非法"会让调用方在 6 向量和位姿对之间反复猜。
    try {
        as_pose(std::vector<double>{1.0, 2.0, 3.0});   // 3 个分量, 不是 6
        FAIL("应当抛异常");
    } catch (const InvalidCommandError& e) {
        CHECK(std::string(e.what()).find("3") != std::string::npos);
    }
    // 旋转矩阵不合法 (不是 SO(3))
    Mat3 bad{};
    bad[0][0] = 5.0;
    CHECK_THROWS_AS(as_pose(PoseInput::from_pos_rot({0, 0, 0}, bad)), InvalidCommandError);
    // 4x4 的旋转块不合法
    Mat4 zeroM{};
    CHECK_THROWS_AS(PoseInput::from_homogeneous(zeroM), InvalidCommandError);
}

TEST(rot_as_pose6_normalizes_all_three_forms_to_the_same_six_vector) {
    const Vec3 pos{0.3, 0.0, 0.4};
    const Vec3 rpy{0.1, 0.2, 0.3};
    const auto a = as_pose6(pos, rpy);
    const auto b = as_pose6(pos, rpy_to_mat(rpy));
    for (int i = 0; i < 6; ++i) CHECK_NEAR(a[size_t(i)], b[size_t(i)], 1e-9);
}

TEST(rot_rpy_to_rowmajor_matches_the_matrix) {
    const Vec3 rpy{0.1, 0.2, 0.3};
    const auto flat = rpy_to_rowmajor(rpy);
    const Mat3 R = rpy_to_mat(rpy);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK_NEAR(flat[size_t(i * 3 + j)], R[size_t(i)][size_t(j)], 1e-15);
        }
    }
}
