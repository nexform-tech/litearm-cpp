// Host-side 7-DOF dynamics model used by the optional identification module.
//
// Frames follow the `DH/` folder, not the vendor URDF: every link frame is the
// controller's Craig/MDH frame rebuilt in DH/litearm_dh.urdf (seven revolute
// joints, joints 4/6/7 on -Z, fixed ee_link merged into link 7).  Inertia
// parameters therefore live in those DH link frames and can be compared
// directly with DH/dh_params.yaml.  They use the Pinocchio convention:
//   phi_link = [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
// with the inertia tensor expressed about the joint origin.
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace litearm {
namespace ident {

inline constexpr int kJoints = 7;
inline constexpr int kInertiaParams = 10 * kJoints;   // 70
inline constexpr int kActiveParams = 96;              // 70 + 7 + 7 + 7 + 5
inline constexpr int kPayloadParams = 4;              // m, m*cx, m*cy, m*cz

using Vec3 = std::array<double, 3>;
using Vec7 = std::array<double, 7>;
using Phi70 = std::array<double, kInertiaParams>;
using PayloadRegressor = std::array<double, kJoints * kPayloadParams>;

/// One identification sample, ordered joint 1..7.
struct SamplePoint {
    double t = 0.0;
    Vec7 q{};
    Vec7 dq{};
    Vec7 ddq{};
    Vec7 tau{};
};

/// CAD phi in the DH link frames, about the joint origin.  Link 7 includes
/// ee_link.  The values are the vendor CAD inertials re-expressed in the
/// Craig/MDH frames of DH/litearm_dh.urdf.
Phi70 cad_phi();

/// What `project_physical_links` had to repair.
struct PhysicalRepair {
    int nonfinite = 0;       ///< link had a NaN/Inf parameter -> CAD fallback
    int bad_mass = 0;        ///< m <= min_mass -> CAD fallback
    int bad_com = 0;         ///< non-finite or beyond max_com_radius -> CAD fallback
    int bad_inertia = 0;     ///< eigendecomposition failed -> CAD fallback
    int clamped = 0;         ///< I_com eigenvalue clamped to the positive cone
    int triangle = 0;        ///< principal moments violated the triangle inequality
    int fallback = 0;        ///< links replaced wholesale by the fallback model

    bool changed() const {
        return nonfinite || bad_mass || bad_com || bad_inertia || clamped ||
               triangle || fallback;
    }
};

/// Per-link projection onto physically realizable rigid-body parameters:
/// finite values, m > min_mass, COM radius <= max_com_radius, I_com positive
/// definite, and principal moments satisfying the triangle inequality.
///
/// A link that cannot be repaired keeps its mass and first moment and only has
/// I_com adjusted (gravity terms are therefore unchanged).  A link whose mass
/// or COM is unusable is replaced wholesale by `fallback` (normally
/// `cad_phi()`, itself repaired the same way).  The result is guaranteed to
/// pass `physically_valid`.
PhysicalRepair project_physical_links(Phi70& phi, const Phi70& fallback,
                                      double min_mass = 1e-4,
                                      double max_com_radius = 1.0);

/// True iff every link is physically realizable.  `why` receives the first
/// violation, e.g. "link 1: principal moments violate the triangle inequality".
bool physically_valid(const Phi70& phi, std::string* why = nullptr,
                      double min_mass = 1e-4, double max_com_radius = 1.0);

/// Direct recursive Newton-Euler torque for the given phi.
Vec7 rnea(const Vec7& q, const Vec7& dq, const Vec7& ddq, const Phi70& phi,
          const Vec3& gravity = Vec3{0.0, 0.0, -9.81});

/// 7 x 70 joint-torque regressor, row-major (row j = joint j).
std::array<double, kJoints * kInertiaParams> inertia_regressor(
    const Vec7& q, const Vec7& dq, const Vec7& ddq,
    const Vec3& gravity = Vec3{0.0, 0.0, -9.81});

/// 7 x 96 regressor: inertia | smooth Coulomb | viscous | DC | jm3..jm7.
std::array<double, kJoints * kActiveParams> active_regressor(
    const Vec7& q, const Vec7& dq, const Vec7& ddq, double K_coulomb = 300.0,
    const Vec3& gravity = Vec3{0.0, 0.0, -9.81});

/// Smooth Coulomb column (2/pi)*atan(K*dq).
double smooth_coulomb(double dq, double K = 300.0);

/// Forward kinematics and the 6x7 geometric Jacobian at the ee frame.
struct FkResult {
    Vec3 p_ee{};
    std::array<double, 9> R_ee{};   // row-major 3x3, ee flange orientation
    std::array<double, 6 * kJoints> J{};   // row-major 6x7, base frame
};

FkResult fk_ee(const Vec7& q);

/// 7 x 4 static payload gravity regressor, row-major (row j = joint j).
/// Columns are [m, m*cx, m*cy, m*cz]; c is the COM in the ee flange frame
/// (the fixed kDhEe transform from link 7).  The gravity vector is the
/// base-frame acceleration; the result is the extra motor torque needed to
/// hold the point mass, i.e. the same sign convention as rnea().
PayloadRegressor payload_gravity_regressor(
    const Vec7& q, const Vec3& gravity = Vec3{0.0, 0.0, -9.81});

/// Minimum singular value of the 6x7 ee Jacobian.
double sigma_min(const Vec7& q);

}  // namespace ident
}  // namespace litearm
