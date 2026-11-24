// Host-side 7-DOF dynamics model and 10-parameter joint-torque regressor.
//
// Frame conventions are the Craig/MDH link frames of the `DH/` folder, not the
// vendor URDF link frames (see DH/litearm_dh.urdf):
//   - joint origins and rpy are the MDH values,
//   - joints 4, 6 and 7 use axis -Z (signs +,+,-,+,-,-,+ ... in order),
//   - the fixed ee_link is merged into link 7 in the CAD inertials.
//
// The resulting phi is expressed directly in the controller's link frames, so
// it is directly comparable with DH/dh_params.yaml and `phi_to_body10` /
// `--write-model` need no frame conversion.
#include "litearm/ident/model.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "../dh_chain.hpp"

namespace litearm {
namespace ident {
namespace {

using Mat3 = Eigen::Matrix3d;
using Vec3d = Eigen::Vector3d;

inline Vec3d to_eigen(const rot::Vec3& v) { return {v[0], v[1], v[2]}; }
inline Mat3 to_eigen(const rot::Mat3& m) {
    Mat3 out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out(i, j) = m[size_t(i)][size_t(j)];
    return out;
}
inline Mat3 rpy_eigen(const rot::Vec3& rpy) { return to_eigen(rot::rpy_to_mat(rpy)); }

struct Kinematics {
    std::array<Mat3, 8> R{};      // R[0] = base, R[i] = link i
    std::array<Vec3d, 8> p{};
    std::array<Vec3d, 8> axis{};  // joint i axis in base frame
    std::array<Vec3d, 8> omega{};
    std::array<Vec3d, 8> alpha{};
    std::array<Vec3d, 8> a{};
};

Kinematics kinematics(const Vec7& q, const Vec7& dq, const Vec7& ddq,
                      const Vec3d& gravity) {
    Kinematics k;
    k.R[0] = Mat3::Identity();
    k.p[0] = Vec3d::Zero();
    k.omega[0] = Vec3d::Zero();
    k.alpha[0] = Vec3d::Zero();
    k.a[0] = -gravity;
    // joint axes are defined after their origin; fill in the loop.
    for (int i = 0; i < kJoints; ++i) {
        const auto& o = ident_detail::kDhJoints[size_t(i)];
        const Mat3 R_o = rpy_eigen(o.rpy);
        const Vec3d p_o = to_eigen(o.xyz);
        const double sign = o.sign;
        const double ang = sign * q[size_t(i)];
        const double c = std::cos(ang), s = std::sin(ang);
        Mat3 Rz;
        Rz << c, -s, 0.0,
              s, c, 0.0,
              0.0, 0.0, 1.0;
        k.R[size_t(i + 1)] = k.R[size_t(i)] * R_o * Rz;
        k.p[size_t(i + 1)] = k.p[size_t(i)] + k.R[size_t(i)] * p_o;
        k.axis[size_t(i + 1)] = sign * (k.R[size_t(i + 1)] * Vec3d::UnitZ());
    }
    for (int i = 0; i < kJoints; ++i) {
        const Vec3d r = k.p[size_t(i + 1)] - k.p[size_t(i)];
        const Vec3d axis = k.axis[size_t(i + 1)];
        const Vec3d w_prev = k.omega[size_t(i)];
        k.omega[size_t(i + 1)] = w_prev + axis * dq[size_t(i)];
        k.alpha[size_t(i + 1)] = k.alpha[size_t(i)] + axis * ddq[size_t(i)] +
                                 w_prev.cross(axis * dq[size_t(i)]);
        k.a[size_t(i + 1)] = k.a[size_t(i)] + k.alpha[size_t(i)].cross(r) +
                             w_prev.cross(w_prev.cross(r));
    }
    return k;
}

struct LinkParam {
    double m = 0.0;
    Vec3d h{};       // m*c in link frame
    Mat3 I{};        // about the link origin, link frame
};

LinkParam link_param(const Phi70& phi, int i) {
    LinkParam p;
    const double* v = &phi[size_t(i * kInertiaParams / kJoints)];
    p.m = v[0];
    p.h = Vec3d(v[1], v[2], v[3]);
    p.I << v[4], v[5], v[7],
           v[5], v[6], v[8],
           v[7], v[8], v[9];
    return p;
}

/// Add the wrench of one link (expressed in base frame about p_i) to the
/// ancestor joint torques.
void propagate(const Kinematics& k, int i, const Vec3d& f, const Vec3d& n,
               std::array<double, kJoints>& tau) {
    Vec3d mom = n;
    Vec3d force = f;
    for (int j = i; j >= 1; --j) {
        tau[size_t(j - 1)] += k.axis[size_t(j)].dot(mom);
        if (j > 1) {
            mom += (k.p[size_t(j)] - k.p[size_t(j - 1)]).cross(force);
        }
    }
}

}  // namespace

Phi70 cad_phi() {
    Phi70 out{};
    for (int i = 0; i < kJoints; ++i) {
        const auto& b = ident_detail::kDhCadBodies[size_t(i)];
        const Vec3d c = to_eigen(b.c);
        const Mat3 Ic = to_eigen(b.I_com);
        const Mat3 Io = Ic + b.m * ((c.dot(c)) * Mat3::Identity() - c * c.transpose());
        const Vec3d h = b.m * c;
        double* v = &out[size_t(i * 10)];
        v[0] = b.m;
        v[1] = h[0];
        v[2] = h[1];
        v[3] = h[2];
        v[4] = Io(0, 0);
        v[5] = Io(0, 1);
        v[6] = Io(1, 1);
        v[7] = Io(0, 2);
        v[8] = Io(1, 2);
        v[9] = Io(2, 2);
    }
    return out;
}

namespace {

enum class LinkFix { ok, nonfinite, bad_mass, bad_com, bad_inertia };

/// Repair one link in place.  Mass and first moment are never touched; only
/// I_com is clamped/projected.  Returns why the link is unusable when the
/// caller should fall back to the CAD link instead.
LinkFix repair_link(Phi70& phi, int i, double min_mass, double max_com_radius,
                    int* clamped, int* triangle) {
    double* p = &phi[size_t(i) * 10];
    for (int k = 0; k < 10; ++k) {
        if (!std::isfinite(p[k])) return LinkFix::nonfinite;
    }
    const double m = p[0];
    if (!(m > min_mass)) return LinkFix::bad_mass;
    const Vec3d c(p[1] / m, p[2] / m, p[3] / m);
    if (!c.allFinite() || c.norm() > max_com_radius) return LinkFix::bad_com;

    Mat3 Io;
    Io << p[4], p[5], p[7],
          p[5], p[6], p[8],
          p[7], p[8], p[9];
    Mat3 I = Io - m * (c.squaredNorm() * Mat3::Identity() - c * c.transpose());
    I = 0.5 * (I + I.transpose());
    Eigen::SelfAdjointEigenSolver<Mat3> eig(I);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
        return LinkFix::bad_inertia;
    }

    const Vec3d ev = eig.eigenvalues();          // ascending
    constexpr double kMinEig = 1e-6;
    Vec3d fixed = ev.cwiseMax(kMinEig);
    if ((fixed.array() != ev.array()).any() && clamped) ++*clamped;

    // Principal moments of any real body satisfy lambda_max <= lambda_mid +
    // lambda_min.  Project onto that half-space with a tiny relative margin so
    // the exported numbers stay strictly inside it.
    const double scale = std::max(1.0, fixed.cwiseAbs().maxCoeff());
    const double margin = 1e-9 * scale;
    if (fixed[2] > fixed[0] + fixed[1] - margin) {
        const double s = (fixed[2] - fixed[0] - fixed[1] + margin) / 3.0;
        fixed[0] += s;
        fixed[1] += s;
        fixed[2] -= s;
        fixed = fixed.cwiseMax(kMinEig);
        if (triangle) ++*triangle;
    }

    if ((fixed.array() != ev.array()).any()) {
        const Mat3 If =
            eig.eigenvectors() * fixed.asDiagonal() * eig.eigenvectors().transpose();
        const Mat3 Ir = 0.5 * (If + If.transpose());
        const Mat3 Io_new =
            Ir + m * (c.squaredNorm() * Mat3::Identity() - c * c.transpose());
        p[4] = Io_new(0, 0);
        p[5] = Io_new(0, 1);
        p[6] = Io_new(1, 1);
        p[7] = Io_new(0, 2);
        p[8] = Io_new(1, 2);
        p[9] = Io_new(2, 2);
    }
    return LinkFix::ok;
}

void set_minimal_link(Phi70& phi, int i) {
    double* p = &phi[size_t(i) * 10];
    for (int k = 0; k < 10; ++k) p[k] = 0.0;
    p[0] = 1.0;
    p[4] = p[6] = p[9] = 1e-6;
}

void copy_link(Phi70& dst, const Phi70& src, int i) {
    for (int k = 0; k < 10; ++k) {
        dst[size_t(i) * 10 + size_t(k)] = src[size_t(i) * 10 + size_t(k)];
    }
}

}  // namespace

PhysicalRepair project_physical_links(Phi70& phi, const Phi70& fallback,
                                      double min_mass, double max_com_radius) {
    PhysicalRepair rep;
    Phi70 fb = fallback;
    for (int i = 0; i < kJoints; ++i) {
        if (repair_link(fb, i, min_mass, max_com_radius, nullptr, nullptr) !=
            LinkFix::ok) {
            set_minimal_link(fb, i);
        }
    }
    for (int i = 0; i < kJoints; ++i) {
        const LinkFix r =
            repair_link(phi, i, min_mass, max_com_radius, &rep.clamped, &rep.triangle);
        if (r == LinkFix::ok) continue;
        switch (r) {
            case LinkFix::nonfinite: ++rep.nonfinite; break;
            case LinkFix::bad_mass: ++rep.bad_mass; break;
            case LinkFix::bad_com: ++rep.bad_com; break;
            case LinkFix::bad_inertia: ++rep.bad_inertia; break;
            case LinkFix::ok: break;
        }
        copy_link(phi, fb, i);
        ++rep.fallback;
    }
    return rep;
}

bool physically_valid(const Phi70& phi, std::string* why, double min_mass,
                      double max_com_radius) {
    for (int i = 0; i < kJoints; ++i) {
        const double* p = &phi[size_t(i) * 10];
        auto fail = [&](const char* msg) {
            if (why) *why = "link " + std::to_string(i + 1) + ": " + msg;
            return false;
        };
        for (int k = 0; k < 10; ++k) {
            if (!std::isfinite(p[k])) return fail("non-finite parameter");
        }
        const double m = p[0];
        if (!(m > min_mass)) return fail("mass must be positive");
        const Vec3d c(p[1] / m, p[2] / m, p[3] / m);
        if (!c.allFinite() || c.norm() > max_com_radius) {
            return fail("COM radius is out of range");
        }
        Mat3 Io;
        Io << p[4], p[5], p[7],
              p[5], p[6], p[8],
              p[7], p[8], p[9];
        const Mat3 I = Io - m * (c.squaredNorm() * Mat3::Identity() - c * c.transpose());
        Eigen::SelfAdjointEigenSolver<Mat3> eig(0.5 * (I + I.transpose()));
        if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
            return fail("eigen decomposition failed");
        }
        const Vec3d ev = eig.eigenvalues();
        if (!(ev[0] > 0.0)) return fail("inertia is not positive definite");
        const double tol = 1e-9 * std::max(1.0, ev[2]);
        if (ev[2] > ev[0] + ev[1] + tol) {
            return fail("principal moments violate the triangle inequality");
        }
    }
    return true;
}

Vec7 rnea(const Vec7& q, const Vec7& dq, const Vec7& ddq, const Phi70& phi,
          const Vec3& gravity_in) {
    const Kinematics k = kinematics(q, dq, ddq, to_eigen(gravity_in));
    std::array<double, kJoints> tau{};
    for (int i = 1; i <= kJoints; ++i) {
        const int idx = i - 1;
        const LinkParam lp = link_param(phi, idx);
        const Mat3 R = k.R[size_t(i)];
        const Vec3d h = R * lp.h;
        const Mat3 I = R * lp.I * R.transpose();
        const Vec3d w = k.omega[size_t(i)];
        const Vec3d al = k.alpha[size_t(i)];
        const Vec3d a = k.a[size_t(i)];
        const Vec3d f = lp.m * a + al.cross(h) + w.cross(w.cross(h));
        const Vec3d n = I * al + w.cross(I * w) + h.cross(a);
        propagate(k, i, f, n, tau);
    }
    Vec7 out{};
    for (int i = 0; i < kJoints; ++i) out[size_t(i)] = tau[size_t(i)];
    return out;
}

std::array<double, kJoints * kInertiaParams> inertia_regressor(
    const Vec7& q, const Vec7& dq, const Vec7& ddq, const Vec3& gravity_in) {
    const Kinematics k = kinematics(q, dq, ddq, to_eigen(gravity_in));
    std::array<double, kJoints * kInertiaParams> Y{};
    Y.fill(0.0);

    for (int i = 1; i <= kJoints; ++i) {
        const Mat3 R = k.R[size_t(i)];
        const Vec3d w = k.omega[size_t(i)];
        const Vec3d al = k.alpha[size_t(i)];
        const Vec3d a = k.a[size_t(i)];

        // 10 basis parameters: m, hx, hy, hz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz.
        for (int s = 0; s < 10; ++s) {
            Vec3d f = Vec3d::Zero();
            Vec3d n = Vec3d::Zero();
            if (s == 0) {
                f = a;
            } else if (s >= 1 && s <= 3) {
                Vec3d h_local = Vec3d::Zero();
                h_local[size_t(s - 1)] = 1.0;
                const Vec3d h = R * h_local;   // basis is in the link frame
                f = al.cross(h) + w.cross(w.cross(h));
                n = h.cross(a);
            } else {
                Mat3 I = Mat3::Zero();
                switch (s) {
                    case 4: I(0, 0) = 1.0; break;                 // Ixx
                    case 5: I(0, 1) = I(1, 0) = 1.0; break;       // Ixy
                    case 6: I(1, 1) = 1.0; break;                 // Iyy
                    case 7: I(0, 2) = I(2, 0) = 1.0; break;       // Ixz
                    case 8: I(1, 2) = I(2, 1) = 1.0; break;       // Iyz
                    default: I(2, 2) = 1.0; break;                // Izz
                }
                const Mat3 Ib = R * I * R.transpose();
                n = Ib * al + w.cross(Ib * w);
            }
            std::array<double, kJoints> tau{};
            propagate(k, i, f, n, tau);
            const int col = (i - 1) * 10 + s;
            for (int j = 0; j < kJoints; ++j) {
                Y[size_t(j * kInertiaParams + col)] = tau[size_t(j)];
            }
        }
    }
    return Y;
}

double smooth_coulomb(double dq, double K) {
    return (2.0 / M_PI) * std::atan(K * dq);
}

std::array<double, kJoints * kActiveParams> active_regressor(
    const Vec7& q, const Vec7& dq, const Vec7& ddq, double K_coulomb,
    const Vec3& gravity) {
    const auto Yi = inertia_regressor(q, dq, ddq, gravity);
    std::array<double, kJoints * kActiveParams> Y{};
    Y.fill(0.0);
    for (int j = 0; j < kJoints; ++j) {
        for (int c = 0; c < kInertiaParams; ++c) {
            Y[size_t(j * kActiveParams + c)] = Yi[size_t(j * kInertiaParams + c)];
        }
        const double sc = smooth_coulomb(dq[size_t(j)], K_coulomb);
        Y[size_t(j * kActiveParams + 70 + j)] = sc;
        Y[size_t(j * kActiveParams + 77 + j)] = dq[size_t(j)];
        Y[size_t(j * kActiveParams + 84 + j)] = std::abs(sc);
        if (j >= 2) Y[size_t(j * kActiveParams + 91 + (j - 2))] = ddq[size_t(j)];
    }
    return Y;
}

FkResult fk_ee(const Vec7& q) {
    const Vec7 zero{};
    const Kinematics k = kinematics(q, zero, zero, Vec3d::Zero());
    const auto& ee = ident_detail::kDhEe;
    const Mat3 R_ee = k.R[7] * rpy_eigen(ee.rpy);
    const Vec3d p_ee = k.p[7] + k.R[7] * to_eigen(ee.xyz);

    FkResult out;
    out.p_ee = {p_ee[0], p_ee[1], p_ee[2]};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out.R_ee[size_t(3 * i + j)] = R_ee(i, j);
    for (int j = 1; j <= kJoints; ++j) {
        const Vec3d axis = k.axis[size_t(j)];
        const Vec3d r = p_ee - k.p[size_t(j)];
        const Vec3d lin = axis.cross(r);
        const int col = j - 1;
        out.J[size_t(0 * kJoints + col)] = lin[0];
        out.J[size_t(1 * kJoints + col)] = lin[1];
        out.J[size_t(2 * kJoints + col)] = lin[2];
        out.J[size_t(3 * kJoints + col)] = axis[0];
        out.J[size_t(4 * kJoints + col)] = axis[1];
        out.J[size_t(5 * kJoints + col)] = axis[2];
    }
    return out;
}

PayloadRegressor payload_gravity_regressor(const Vec7& q, const Vec3& gravity_in) {
    const Vec7 zero{};
    const Kinematics k = kinematics(q, zero, zero, Vec3d::Zero());
    const auto& ee = ident_detail::kDhEe;
    const Mat3 R_ee = k.R[7] * rpy_eigen(ee.rpy);
    const Vec3d p_ee = k.p[7] + k.R[7] * to_eigen(ee.xyz);
    const Vec3d g = to_eigen(gravity_in);

    PayloadRegressor Y{};
    Y.fill(0.0);
    for (int j = 1; j <= kJoints; ++j) {
        const Vec3d axis = k.axis[size_t(j)];
        const Vec3d r0 = p_ee - k.p[size_t(j)];
        const size_t row = size_t(j - 1) * kPayloadParams;
        // Holding torque: rnea() integrates with base acceleration -g, so the
        // payload contribution is -J^T (m g), not +J^T (m g).
        Y[row + 0] = -(axis.cross(r0)).dot(g);
        for (int c = 0; c < 3; ++c) {
            Y[row + size_t(1 + c)] = -(axis.cross(R_ee.col(c))).dot(g);
        }
    }
    return Y;
}

double sigma_min(const Vec7& q) {
    const FkResult fk = fk_ee(q);
    Eigen::Matrix<double, 6, 7> J;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 7; ++j) J(i, j) = fk.J[size_t(i * 7 + j)];
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(J);
    return svd.singularValues()(svd.singularValues().size() - 1);
}

}  // namespace ident
}  // namespace litearm
