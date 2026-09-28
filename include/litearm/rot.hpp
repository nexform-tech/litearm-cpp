// 旋转/位姿小工具 —— 纯数学, 无外部依赖。
//
// 约定与固件 kin.c 完全一致 (ZYX 内旋)。三条互不相同的表示在本包来回转:
//
//   6 向量        (x, y, z, roll, pitch, yaw)     固件 / 本包 get_tcp / move_p
//   位姿对        (position[3], rotation[3x3])    pylitearm 的 movel/movec/movep
//   旋转矩阵      R[3][3] 行主序                   内部统一用这个算
//
// as_pose() 是唯一的入口, 上面前两种都吃。
#pragma once

#include <array>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

namespace litearm {
namespace rot {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;
using Mat4 = std::array<std::array<double, 4>, 4>;
/// 归一化后的位姿: (位置[3], 旋转矩阵[3x3])
using Pose = std::pair<Vec3, Mat3>;

inline constexpr double kEps = 1e-12;

Mat3 eye3();
Mat3 mat_mul(const Mat3& A, const Mat3& B);
Mat3 mat_T(const Mat3& A);
Vec3 mat_vec(const Mat3& A, const Vec3& v);
double dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
double norm(const Vec3& v);
Vec3 sub(const Vec3& a, const Vec3& b);
/// 零向量抛 InvalidCommandError。
Vec3 unit(const Vec3& v);
double det3(const Mat3& R);

// RPY <-> 旋转矩阵 (ZYX 内旋, 与固件 kin_rpy_to_rot / kin_rot_to_rpy 同约定)。
//
// rpy_to_mat: (roll, pitch, yaw) -> R, 即 R = Rz(yaw) * Ry(pitch) * Rx(roll)。
// 与固件逐元素同式 —— 别再自己推一遍, 推错方向不会报错, 只会让所有姿态静默反着转。
Mat3 rpy_to_mat(const Vec3& rpy);
/// R -> (roll, pitch, yaw)。万向锁 (|pitch| ~= pi/2) 时强制 yaw=0 —— 与固件
/// kin_rot_to_rpy 的锁分支同约定。只保证"锁上强制 yaw=0"这条约定一致 —— 两侧的锁带
/// 宽度不同: 本函数用 |pitch| 距 pi/2 小于 1e-6, 固件用 |cos(pitch)| <= 1e-4。
Vec3 mat_to_rpy(const Mat3& R);

// SO(3) 上的指数/对数 —— slerp 与姿态误差的基础。
/// R -> 旋转向量 (轴 x 角)。acos 的入参必须夹住: 数值噪声会让它越界到 +-(1+eps)。
Vec3 rot_log(const Mat3& R);
/// 旋转向量 -> R (Rodrigues)。
Mat3 rot_exp(const Vec3& w);
/// 姿态球面插值 R0 * exp(t * log(R0^T R1)) —— 走最短弧。
Mat3 rot_slerp(const Mat3& R0, const Mat3& R1, double t);
/// 旋转矩阵对应的转角 (rad)。
double rot_angle(const Mat3& R);

/// (位置误差 m, 姿态误差 rad)。
///
/// 姿态差必须是 rot_log(R_d * R^T), 不能写成 rot_log(R * R_d^T) —— 后者是复合不是差,
/// 符号相反; 曾因此把 0.59mm 位置误差的位姿报成 153 度姿态误差。
std::pair<double, double> pose_err(const Vec3& pos, const Mat3& R, const Vec3& pos_d,
                                   const Mat3& R_d);

/// 是不是合法的 SO(3) 成员 (正交 + det ~= +1)。
bool is_rotation(const Mat3& R, double atol = 1e-5);

// ---------------------------------------------------------------- 位姿归一化

/// 位姿入参的三种可接受写法 —— 这是 C++ 侧对 Python 动态形态判定的等价表达。
struct PoseInput {
    enum class Kind { Pose6, PosRot, Homogeneous };
    Kind kind = Kind::Pose6;
    Vec3 pos{};
    Mat3 R{};          // PosRot
    Vec3 rpy{};        // Pose6

    PoseInput() = default;

    //: 隐式转换 —— 让 `arm.move_p({0.3, 0, 0.4, 3.14, 0, 0})` 这类最常见写法直接可用。
    //: (pos[3], R[3x3]) 与 4x4 两种写法用下面那两个具名工厂 —— 它们**不**隐式转换是
    //: 刻意的: 一个 3 元素 vector 到底是位置还是别的, 靠隐式转换猜不出来。
    PoseInput(const std::array<double, 6>& v) { *this = from_vec6(v); }   // NOLINT
    PoseInput(const std::vector<double>& v) { *this = from_vec6(v); }     // NOLINT
    /// 花括号字面量 —— 让 `arm.move_p({0.3, 0, 0.4, 3.14, 0, 0})` 直接可用
    /// (有它时, 花括号列表会优先选这一条, 于是不会与上面两条撞成二义)。
    PoseInput(std::initializer_list<double> v) {                          // NOLINT
        *this = from_vec6(std::vector<double>(v));
    }
    PoseInput(const Mat4& M) { *this = from_homogeneous(M); }             // NOLINT
    PoseInput(const Vec3& pos_in, const Mat3& R_in) {                     // NOLINT
        *this = from_pos_rot(pos_in, R_in);
    }

    // 形式 1: [x, y, z, roll, pitch, yaw]
    static PoseInput from_vec6(const Vec3& xyz, const Vec3& rpy);
    static PoseInput from_vec6(const std::array<double, 6>& v);
    static PoseInput from_vec6(const std::vector<double>& v);
    // 形式 2: (position[3], R[3x3])
    static PoseInput from_pos_rot(const Vec3& pos, const Mat3& R);
    // 形式 3: 4x4 齐次矩阵
    static PoseInput from_homogeneous(const Mat4& M);
};

/// 把调用方给的东西归一化成 (position[3], R[3x3])。
/// 不合法的形态抛 InvalidCommandError, 文案里给出实际收到的形状。
Pose as_pose(const PoseInput& in);

/// 便捷: 任何写法 -> 归一化位姿。
inline Pose as_pose(const std::array<double, 6>& v) {
    return as_pose(PoseInput::from_vec6(v));
}
/// std::vector 写法 —— 长度必须恰好 6, 否则抛 InvalidCommandError。
/// (C++ 里 6 向量最常见的载体就是 vector, 收它免得调用方自己转 array。)
inline Pose as_pose(const std::vector<double>& v) {
    return as_pose(PoseInput::from_vec6(v));
}
inline Pose as_pose(const Vec3& xyz, const Vec3& rpy) {
    return as_pose(PoseInput::from_vec6(xyz, rpy));
}
inline Pose as_pose(const Vec3& pos, const Mat3& R) {
    return as_pose(PoseInput::from_pos_rot(pos, R));
}
inline Pose as_pose(const Mat4& M) { return as_pose(PoseInput::from_homogeneous(M)); }

/// 任意写法 -> 固件要的 [x,y,z,roll,pitch,yaw]。
std::array<double, 6> as_pose6(const PoseInput& in);
inline std::array<double, 6> as_pose6(const std::array<double, 6>& v) {
    return as_pose6(PoseInput::from_vec6(v));
}
inline std::array<double, 6> as_pose6(const std::vector<double>& v) {
    return as_pose6(PoseInput::from_vec6(v));
}
inline std::array<double, 6> as_pose6(const Vec3& pos, const Mat3& R) {
    return as_pose6(PoseInput::from_pos_rot(pos, R));
}
inline std::array<double, 6> as_pose6(const Vec3& xyz, const Vec3& rpy) {
    return as_pose6(PoseInput::from_vec6(xyz, rpy));
}
inline std::array<double, 6> as_pose6(const Mat4& M) {
    return as_pose6(PoseInput::from_homogeneous(M));
}

/// 便捷: 6 向量 -> 9 元组行主序旋转矩阵 (与固件同式)。
inline std::array<double, 9> rpy_to_rowmajor(const Vec3& rpy) {
    const Mat3 m = rpy_to_mat(rpy);
    return {m[0][0], m[0][1], m[0][2], m[1][0], m[1][1],
            m[1][2], m[2][0], m[2][1], m[2][2]};
}

/// 两组 rpy 所代表旋转之间的测地夹角 (rad)。trace(Ra^T * Rb) 求夹角。
double orient_angle(const Vec3& a, const Vec3& b);

}  // namespace rot
}  // namespace litearm
