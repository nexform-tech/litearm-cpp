#include "litearm/rot.hpp"

#include <cmath>
#include <sstream>

#include "litearm/errors.hpp"

namespace litearm {
namespace rot {

Mat3 eye3() {
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 mat_mul(const Mat3& A, const Mat3& B) {
    Mat3 out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
        }
    }
    return out;
}

Mat3 mat_T(const Mat3& A) {
    Mat3 out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) out[i][j] = A[j][i];
    }
    return out;
}

Vec3 mat_vec(const Mat3& A, const Vec3& v) {
    return {A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
            A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
            A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2]};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

double norm(const Vec3& v) { return std::sqrt(dot(v, v)); }

Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

Vec3 unit(const Vec3& v) {
    const double n = norm(v);
    if (n < kEps) throw InvalidCommandError("零向量无法归一化");
    return {v[0] / n, v[1] / n, v[2] / n};
}

double det3(const Mat3& R) {
    return R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
           R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
           R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
}

Mat3 rpy_to_mat(const Vec3& rpy) {
    // 与固件 kin_rpy_to_rot 逐元素同式。
    const double r = rpy[0], p = rpy[1], y = rpy[2];
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
             {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
             {-sp, cp * sr, cp * cr}}};
}

Vec3 mat_to_rpy(const Mat3& R) {
    const double p = std::atan2(-R[2][0], std::hypot(R[0][0], R[1][0]));
    if (std::fabs(std::fabs(p) - M_PI / 2.0) < 1e-6) {
        return {std::atan2(-R[1][2], R[1][1]), p, 0.0};
    }
    return {std::atan2(R[2][1], R[2][2]), p, std::atan2(R[1][0], R[0][0])};
}

Vec3 rot_log(const Mat3& R) {
    const double tr = R[0][0] + R[1][1] + R[2][2];
    const double c = std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0));
    const double ang = std::acos(c);
    if (ang < 1e-9) return {0.0, 0.0, 0.0};
    const Vec3 v{R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]};
    const double k = ang / (2.0 * std::sin(ang));
    return {v[0] * k, v[1] * k, v[2] * k};
}

Mat3 rot_exp(const Vec3& w) {
    const double ang = norm(w);
    if (ang < 1e-12) return eye3();
    const Vec3 k{w[0] / ang, w[1] / ang, w[2] / ang};
    const Mat3 K{{{0.0, -k[2], k[1]}, {k[2], 0.0, -k[0]}, {-k[1], k[0], 0.0}}};
    const double s = std::sin(ang), c = 1.0 - std::cos(ang);
    const Mat3 KK = mat_mul(K, K);
    Mat3 out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[i][j] = (i == j ? 1.0 : 0.0) + s * K[i][j] + c * KK[i][j];
        }
    }
    return out;
}

Mat3 rot_slerp(const Mat3& R0, const Mat3& R1, double t) {
    const Vec3 w = rot_log(mat_mul(mat_T(R0), R1));
    if (norm(w) < 1e-12) return R0;
    return mat_mul(R0, rot_exp({w[0] * t, w[1] * t, w[2] * t}));
}

double rot_angle(const Mat3& R) { return norm(rot_log(R)); }

std::pair<double, double> pose_err(const Vec3& pos, const Mat3& R, const Vec3& pos_d,
                                   const Mat3& R_d) {
    return {norm(sub(pos_d, pos)), rot_angle(mat_mul(R_d, mat_T(R)))};
}

bool is_rotation(const Mat3& R, double atol) {
    const Mat3 Rt = mat_T(R);
    const Mat3 I3 = eye3();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += Rt[i][k] * R[k][j];
            if (std::fabs(s - I3[i][j]) > atol) return false;
        }
    }
    return det3(R) > 0.999;
}

double orient_angle(const Vec3& a, const Vec3& b) {
    const Mat3 Ra = rpy_to_mat(a), Rb = rpy_to_mat(b);
    double c = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) c += Ra[j][i] * Rb[j][i];   // trace(Ra^T * Rb)
    }
    return std::acos(std::max(-1.0, std::min(1.0, (c - 1.0) / 2.0)));
}

// ---------------------------------------------------------------- PoseInput

PoseInput PoseInput::from_vec6(const Vec3& xyz, const Vec3& rpy) {
    PoseInput p;
    p.kind = Kind::Pose6;
    p.pos = xyz;
    p.rpy = rpy;
    return p;
}

PoseInput PoseInput::from_vec6(const std::array<double, 6>& v) {
    return from_vec6(Vec3{v[0], v[1], v[2]}, Vec3{v[3], v[4], v[5]});
}

PoseInput PoseInput::from_vec6(const std::vector<double>& v) {
    if (v.size() != 6) {
        std::ostringstream os;
        os << "pose 6 向量需 6 个分量, 收到 " << v.size();
        throw InvalidCommandError(os.str());
    }
    return from_vec6(std::array<double, 6>{v[0], v[1], v[2], v[3], v[4], v[5]});
}

PoseInput PoseInput::from_pos_rot(const Vec3& pos, const Mat3& R) {
    if (!is_rotation(R)) {
        throw InvalidCommandError("pose 旋转矩阵不是有效 SO(3) (正交且 det~=+1)");
    }
    PoseInput p;
    p.kind = Kind::PosRot;
    p.pos = pos;
    p.R = R;
    return p;
}

PoseInput PoseInput::from_homogeneous(const Mat4& M) {
    Mat3 R{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) R[i][j] = M[i][j];
    }
    if (!is_rotation(R)) {
        throw InvalidCommandError("pose 4x4 的旋转块不是有效 SO(3)");
    }
    PoseInput p;
    p.kind = Kind::Homogeneous;
    p.pos = {M[0][3], M[1][3], M[2][3]};
    p.R = R;
    return p;
}

Pose as_pose(const PoseInput& in) {
    switch (in.kind) {
        case PoseInput::Kind::Pose6:
            return {in.pos, rpy_to_mat(in.rpy)};
        case PoseInput::Kind::PosRot:
        case PoseInput::Kind::Homogeneous:
        default:
            return {in.pos, in.R};
    }
}

std::array<double, 6> as_pose6(const PoseInput& in) {
    const Pose p = as_pose(in);
    const Vec3 rpy = mat_to_rpy(p.second);
    return {p.first[0], p.first[1], p.first[2], rpy[0], rpy[1], rpy[2]};
}

}  // namespace rot
}  // namespace litearm
