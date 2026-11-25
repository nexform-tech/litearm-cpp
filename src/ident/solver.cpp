// 96-parameter MAP identification solver.
//
// Layout: [0,70) inertia | [70,77) symmetric Coulomb a | [77,84) viscous fv |
// [84,91) asymmetry b | [91,96) jm3..jm7.
//   fc0 = a + b, fc1 = -a + b.
#include "litearm/ident/solver.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace litearm {
namespace ident {
namespace {

using Mat3 = Eigen::Matrix3d;
using Vec3d = Eigen::Vector3d;

constexpr int kOffC = 70;
constexpr int kOffFv = 77;
constexpr int kOffB = 84;
constexpr int kOffJm = 91;
constexpr int kNumJm = 5;

struct LinkCom {
    double m = 0.0;
    Vec3d c{};
    Mat3 I{};
};

LinkCom phi_link_to_com(const Phi70& phi, int i) {
    LinkCom out;
    const double* p = &phi[size_t(i * 10)];
    out.m = p[0];
    if (!(out.m > 1e-9) || !std::isfinite(out.m)) {
        out.m = 1e-6;
    }
    out.c = Vec3d(p[1], p[2], p[3]) / out.m;
    Mat3 Io;
    Io << p[4], p[5], p[7],
          p[5], p[6], p[8],
          p[7], p[8], p[9];
    out.I = Io - out.m * ((out.c.dot(out.c)) * Mat3::Identity() - out.c * out.c.transpose());
    return out;
}

void com_to_phi_link(const LinkCom& lc, Phi70& phi, int i) {
    const Mat3 Io = lc.I + lc.m * ((lc.c.dot(lc.c)) * Mat3::Identity() -
                                   lc.c * lc.c.transpose());
    double* p = &phi[size_t(i * 10)];
    p[0] = lc.m;
    p[1] = lc.m * lc.c[0];
    p[2] = lc.m * lc.c[1];
    p[3] = lc.m * lc.c[2];
    p[4] = Io(0, 0);
    p[5] = Io(0, 1);
    p[6] = Io(1, 1);
    p[7] = Io(0, 2);
    p[8] = Io(1, 2);
    p[9] = Io(2, 2);
}

struct ProjectionCounts {
    int a = 0;
    int fv = 0;
    int b = 0;
    int jm = 0;
};

ProjectionCounts project_actuators(Vec7& a, Vec7& fv, Vec7& b, Vec7& jm) {
    ProjectionCounts c;
    for (int i = 0; i < kJoints; ++i) {
        if (a[size_t(i)] < 0.0) ++c.a;
        a[size_t(i)] = std::max(a[size_t(i)], 0.0);
        if (fv[size_t(i)] < 0.0) ++c.fv;
        fv[size_t(i)] = std::max(fv[size_t(i)], 0.0);
        if (std::abs(b[size_t(i)]) > a[size_t(i)]) ++c.b;
        b[size_t(i)] = std::clamp(b[size_t(i)], -a[size_t(i)], a[size_t(i)]);
        if (i < 2) {
            jm[size_t(i)] = 0.0;
        } else {
            if (jm[size_t(i)] < 0.0) ++c.jm;
            jm[size_t(i)] = std::max(jm[size_t(i)], 0.0);
        }
    }
    return c;
}

std::vector<double> blend_physical(Phi70& phi, const Phi70& cad, double min_eig) {
    std::vector<double> alphas(size_t(kJoints), 0.0);
    for (int i = 0; i < kJoints; ++i) {
        LinkCom id = phi_link_to_com(phi, i);
        const LinkCom cd = phi_link_to_com(cad, i);
        Eigen::SelfAdjointEigenSolver<Mat3> eig(id.I);
        double alpha = 0.0;
        if (eig.eigenvalues().minCoeff() <= min_eig) {
            alpha = 1.0;
            for (int step = 0; step <= 100; ++step) {
                const double a = double(step) / 100.0;
                const Mat3 I = (1.0 - a) * id.I + a * cd.I;
                Eigen::SelfAdjointEigenSolver<Mat3> e(I);
                if (e.eigenvalues().minCoeff() > min_eig) {
                    alpha = a;
                    break;
                }
            }
            id.I = (1.0 - alpha) * id.I + alpha * cd.I;
            com_to_phi_link(id, phi, i);
        }
        alphas[size_t(i)] = alpha;
    }
    return alphas;
}

Eigen::MatrixXd qr_r(const Eigen::MatrixXd& Y, int ncols) {
    if (Y.rows() < ncols) {
        return Y;
    }
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(Y);
    return qr.matrixQR().topLeftCorner(ncols, ncols)
        .template triangularView<Eigen::Upper>().toDenseMatrix();
}

}  // namespace

Vec7 IdentResult::fc0() const {
    Vec7 out{};
    for (int i = 0; i < kJoints; ++i) out[size_t(i)] = coulomb[size_t(i)] + dc[size_t(i)];
    return out;
}

Vec7 IdentResult::fc1() const {
    Vec7 out{};
    for (int i = 0; i < kJoints; ++i) out[size_t(i)] = -coulomb[size_t(i)] + dc[size_t(i)];
    return out;
}

IdentResult identify(const std::vector<SamplePoint>& samples,
                     const IdentifyOptions& options) {
    if (samples.empty()) throw std::invalid_argument("identify: no samples");

    const std::size_t total_rows = samples.size() * kJoints;
    const std::size_t diag_max = std::min<std::size_t>(
        total_rows, std::max<std::size_t>(1, options.diagnostics_max_rows));
    const std::size_t step = std::max<std::size_t>(
        1, (total_rows + diag_max - 1) / diag_max);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(kActiveParams, kActiveParams);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(kActiveParams);
    Eigen::MatrixXd Ydiag = Eigen::MatrixXd::Zero(Eigen::Index(diag_max), kActiveParams);
    std::size_t diag_row = 0;

    for (std::size_t s = 0; s < samples.size(); ++s) {
        const SamplePoint& sp = samples[s];
        const auto Y = active_regressor(sp.q, sp.dq, sp.ddq, options.K_coulomb,
                                        options.gravity);
        Eigen::Matrix<double, kJoints, kActiveParams> Yb;
        Eigen::Matrix<double, kJoints, 1> tb;
        for (int j = 0; j < kJoints; ++j) {
            tb[j] = sp.tau[size_t(j)];
            for (int c = 0; c < kActiveParams; ++c) {
                Yb(j, c) = Y[size_t(j * kActiveParams + c)];
            }
        }
        H.noalias() += Yb.transpose() * Yb;
        g.noalias() += Yb.transpose() * tb;

        for (int j = 0; j < kJoints; ++j) {
            const std::size_t global = s * kJoints + std::size_t(j);
            if (global % step == 0 && diag_row < diag_max) {
                for (int c = 0; c < kActiveParams; ++c) {
                    Ydiag(Eigen::Index(diag_row), c) = Yb(j, c);
                }
                ++diag_row;
            }
        }
    }
    if (diag_row == 0) {
        Ydiag.resize(0, kActiveParams);
    } else {
        Ydiag.conservativeResize(Eigen::Index(diag_row), kActiveParams);
    }

    Eigen::VectorXd prior = Eigen::VectorXd::Zero(kActiveParams);
    const Phi70 cad = cad_phi();
    for (int c = 0; c < kInertiaParams; ++c) {
        prior[c] = options.fix_inertia ? options.fixed_phi[size_t(c)] : cad[size_t(c)];
    }

    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(kActiveParams);
    for (int c = 0; c < 70; ++c) lambda[c] = options.regularization;
    for (int c = kOffC; c < kOffFv; ++c) lambda[c] = options.friction_ridge;
    for (int c = kOffFv; c < kOffB; ++c) lambda[c] = options.friction_ridge;
    for (int c = kOffB; c < kOffJm; ++c) lambda[c] = options.dc_ridge;
    for (int c = kOffJm; c < kActiveParams; ++c) lambda[c] = options.armature_ridge;

    std::vector<int> active;
    active.reserve(kActiveParams);
    for (int c = 0; c < kActiveParams; ++c) {
        if (options.fix_dc && c >= kOffB && c < kOffJm) continue;
        // Fixed-inertia mode: all 70 inertial columns are constants.
        if (options.fix_inertia && c < kInertiaParams) continue;
        // Unit-mass gauge: the mass columns are fixed at 1, not solved for.
        if (!options.fix_inertia && options.unit_mass && c < kInertiaParams &&
            c % 10 == 0) {
            continue;
        }
        active.push_back(c);
    }
    const int na = int(active.size());
    Eigen::MatrixXd Ha(na, na);
    Eigen::VectorXd ga(na), la(na), pa(na);
    for (int i = 0; i < na; ++i) {
        const int ci = active[size_t(i)];
        ga[i] = g[ci];
        la[i] = lambda[ci];
        pa[i] = prior[ci];
        for (int j = 0; j < na; ++j) Ha(i, j) = H(ci, active[size_t(j)]);
    }
    if (options.fix_inertia) {
        // Move the fixed 70-column inertia contribution to the right-hand side.
        for (int i = 0; i < na; ++i) {
            const int ci = active[size_t(i)];
            for (int c = 0; c < kInertiaParams; ++c) {
                ga[i] -= H(ci, c) * options.fixed_phi[size_t(c)];
            }
        }
    } else if (options.unit_mass) {
        // Move the fixed m = 1 contribution to the right-hand side.
        for (int i = 0; i < na; ++i) {
            const int ci = active[size_t(i)];
            for (int mi = 0; mi < kJoints; ++mi) ga[i] -= H(ci, mi * 10);
        }
    }
    for (int i = 0; i < na; ++i) Ha(i, i) += la[i];
    ga += la.cwiseProduct(pa);
    Eigen::VectorXd theta_a;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(Ha);
    if (ldlt.info() == Eigen::Success) {
        theta_a = ldlt.solve(ga);
    } else {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(Ha);
        theta_a = cod.solve(ga);
    }
    if (theta_a.size() != na || !theta_a.allFinite()) {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(Ha);
        theta_a = cod.solve(ga);
    }
    Eigen::VectorXd theta = Eigen::VectorXd::Zero(kActiveParams);
    for (int i = 0; i < na; ++i) theta[active[size_t(i)]] = theta_a[i];
    if (options.fix_inertia) {
        for (int c = 0; c < kInertiaParams; ++c) theta[c] = options.fixed_phi[size_t(c)];
    } else if (options.unit_mass) {
        for (int i = 0; i < kJoints; ++i) theta[i * 10] = 1.0;
    }

    IdentResult out;
    for (int c = 0; c < 70; ++c) out.phi_raw[size_t(c)] = theta[c];
    for (int j = 0; j < kJoints; ++j) {
        out.coulomb[size_t(j)] = theta[kOffC + j];
        out.viscous[size_t(j)] = theta[kOffFv + j];
        out.dc[size_t(j)] = options.fix_dc ? 0.0 : theta[kOffB + j];
    }
    for (int j = 0; j < kJoints; ++j) {
        out.armature[size_t(j)] = (j < 2) ? 0.0 : theta[kOffJm + (j - 2)];
    }
    const ProjectionCounts pc = project_actuators(out.coulomb, out.viscous, out.dc,
                                                   out.armature);
    out.info.projection = {{pc.a, pc.fv, pc.b, pc.jm}};

    // CAD fill on the inertia block.
    int rank_inertia = 0;
    Phi70 phi_filled = out.phi_raw;
    if (options.fix_inertia) {
        // Fixed inertia: keep the supplied values exactly, including models the
        // firmware accepts but that are not physically realizable (e.g. m=1
        // mass-only banks with negative COM inertias).
        for (int c = 0; c < kInertiaParams; ++c) {
            phi_filled[size_t(c)] = options.fixed_phi[size_t(c)];
        }
    } else if (options.unit_mass) {
        // Only the 63 non-mass inertial parameters are free; fill their
        // unobservable directions from CAD and keep every mass at exactly 1.
        std::vector<int> free_cols;
        for (int c = 0; c < kInertiaParams; ++c)
            if (c % 10 != 0) free_cols.push_back(c);
        const int nf = int(free_cols.size());
        if (Ydiag.rows() > 0) {
            Eigen::MatrixXd Yf(Ydiag.rows(), nf);
            for (int k = 0; k < nf; ++k) Yf.col(k) = Ydiag.col(free_cols[size_t(k)]);
            Eigen::MatrixXd R = qr_r(Yf, nf);
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeFullV);
            const auto sv = svd.singularValues();
            const double tol = (sv.size() ? 1e-6 * sv[0] : 0.0);
            for (int i = 0; i < sv.size(); ++i) if (sv[i] > tol) ++rank_inertia;
            const Eigen::MatrixXd V = svd.matrixV();
            Eigen::VectorXd pf = Eigen::VectorXd::Zero(nf);
            Eigen::VectorXd pc = Eigen::VectorXd::Zero(nf);
            for (int k = 0; k < nf; ++k) {
                pf[k] = out.phi_raw[size_t(free_cols[size_t(k)])];
                pc[k] = cad[size_t(free_cols[size_t(k)])];
            }
            if (rank_inertia > 0) {
                const Eigen::MatrixXd Vi = V.leftCols(rank_inertia);
                pf = Vi * (Vi.transpose() * pf);
            }
            if (rank_inertia < nf) {
                const Eigen::MatrixXd Vu = V.rightCols(nf - rank_inertia);
                pf += Vu * (Vu.transpose() * pc);
            }
            for (int k = 0; k < nf; ++k) phi_filled[size_t(free_cols[size_t(k)])] = pf[k];
        }
    } else if (Ydiag.rows() > 0) {
        const Eigen::MatrixXd Yi = Ydiag.leftCols(70);
        Eigen::MatrixXd R = qr_r(Yi, 70);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeFullV);
        const auto sv = svd.singularValues();
        const double tol = (sv.size() ? 1e-6 * sv[0] : 0.0);
        for (int i = 0; i < sv.size(); ++i) if (sv[i] > tol) ++rank_inertia;
        const Eigen::MatrixXd V = svd.matrixV();
        Eigen::VectorXd phi_raw = Eigen::VectorXd::Zero(70);
        Eigen::VectorXd phi_cad = Eigen::VectorXd::Zero(70);
        for (int c = 0; c < 70; ++c) {
            phi_raw[c] = out.phi_raw[size_t(c)];
            phi_cad[c] = cad[size_t(c)];
        }
        if (rank_inertia > 0) {
            const Eigen::MatrixXd Vi = V.leftCols(rank_inertia);
            phi_raw = Vi * (Vi.transpose() * phi_raw);
        }
        if (rank_inertia < 70) {
            const Eigen::MatrixXd Vu = V.rightCols(70 - rank_inertia);
            phi_raw += Vu * (Vu.transpose() * phi_cad);
        }
        for (int c = 0; c < 70; ++c) phi_filled[size_t(c)] = phi_raw[c];
    }
    if (!options.fix_inertia) {
        if (options.physical_links) {
            out.info.cad_blend_alpha = blend_physical(phi_filled, cad, options.cad_min_eig);
        }
        // Final guarantee: every exported link is a physically realizable rigid
        // body (positive mass, positive-definite I_com, principal moments obeying
        // the triangle inequality).  Unusable links fall back to CAD.
        project_physical_links(phi_filled, cad);
        if (!physically_valid(phi_filled)) {
            Phi70 safe = cad;
            project_physical_links(safe, cad);
            phi_filled = safe;
        }
        if (options.unit_mass) {
            // The CAD fallback replaces a link wholesale (including its mass), which
            // would silently break the unit-mass gauge.
            for (int i = 0; i < kJoints; ++i) {
                if (std::abs(phi_filled[size_t(i * 10)] - 1.0) > 1e-9) {
                    throw std::runtime_error(
                        "identify: unit-mass gauge lost on link " + std::to_string(i + 1) +
                        " (its COM/inertia could not be repaired without changing m)");
                }
            }
        }
    }
    out.phi = phi_filled;

    // Diagnostics: rank and singular values of the sampled 96-column matrix.
    out.info.rank_inertia = rank_inertia;
    out.info.rank_is_sampled = Ydiag.rows() < Eigen::Index(total_rows);
    if (Ydiag.rows() > 0) {
        Eigen::MatrixXd R = qr_r(Ydiag, kActiveParams);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(R);
        const auto sv = svd.singularValues();
        out.info.sv_max = sv.size() ? sv[0] : 0.0;
        out.info.sv_min = sv.size() ? sv[sv.size() - 1] : 0.0;
        out.info.cond = (out.info.sv_min > 0.0)
                            ? out.info.sv_max / out.info.sv_min
                            : std::numeric_limits<double>::infinity();
        const double tol = out.info.sv_max * 1e-12;
        out.info.rank = 0;
        for (int i = 0; i < sv.size(); ++i) if (sv[i] > tol) ++out.info.rank;

        Eigen::MatrixXd Yfit(Ydiag.rows(), na);
        for (int i = 0; i < na; ++i) Yfit.col(i) = Ydiag.col(active[size_t(i)]);
        Eigen::MatrixXd Rf = qr_r(Yfit, na);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_fit(Rf);
        const auto svf = svd_fit.singularValues();
        const double tol_fit = (svf.size() ? svf[0] * 1e-12 : 0.0);
        out.info.rank_fit = 0;
        for (int i = 0; i < svf.size(); ++i) if (svf[i] > tol_fit) ++out.info.rank_fit;
    }
    out.info.n_samples = int(samples.size());
    out.info.n_rows = int(total_rows);
    out.info.n_params = kActiveParams;
    out.info.physical_links = options.physical_links && !options.fix_inertia;
    out.info.fixed_inertia = options.fix_inertia;
    if (out.info.cad_blend_alpha.empty()) {
        out.info.cad_blend_alpha.assign(std::size_t(kJoints), 0.0);
    }

    // RMSE and ablations.
    std::array<double, kJoints> sse{}, sse_no_dc{}, sse_no_jm{}, sse_no_act{};
    for (const auto& sp : samples) {
        const auto Y = active_regressor(sp.q, sp.dq, sp.ddq, options.K_coulomb,
                                        options.gravity);
        for (int j = 0; j < kJoints; ++j) {
            double pred = 0.0;
            double pred_no_dc = 0.0;
            double pred_no_jm = 0.0;
            double pred_no_act = 0.0;
            for (int c = 0; c < 70; ++c) {
                pred += Y[size_t(j * kActiveParams + c)] * out.phi[size_t(c)];
                pred_no_dc += Y[size_t(j * kActiveParams + c)] * out.phi[size_t(c)];
                pred_no_jm += Y[size_t(j * kActiveParams + c)] * out.phi[size_t(c)];
                pred_no_act += Y[size_t(j * kActiveParams + c)] * out.phi[size_t(c)];
            }
            pred += Y[size_t(j * kActiveParams + kOffC + j)] * out.coulomb[size_t(j)];
            pred += Y[size_t(j * kActiveParams + kOffFv + j)] * out.viscous[size_t(j)];
            pred += Y[size_t(j * kActiveParams + kOffB + j)] * out.dc[size_t(j)];
            if (j >= 2) {
                pred += Y[size_t(j * kActiveParams + kOffJm + (j - 2))] *
                        out.armature[size_t(j)];
            }
            pred_no_dc = pred - Y[size_t(j * kActiveParams + kOffB + j)] * out.dc[size_t(j)];
            pred_no_jm = pred;
            if (j >= 2) {
                pred_no_jm = pred - Y[size_t(j * kActiveParams + kOffJm + (j - 2))] *
                                         out.armature[size_t(j)];
            }
            pred_no_act = pred_no_dc;
            if (j >= 2) {
                pred_no_act = pred_no_dc - Y[size_t(j * kActiveParams + kOffJm + (j - 2))] *
                                               out.armature[size_t(j)];
            }
            const double e = sp.tau[size_t(j)] - pred;
            const double e_dc = sp.tau[size_t(j)] - pred_no_dc;
            const double e_jm = sp.tau[size_t(j)] - pred_no_jm;
            const double e_act = sp.tau[size_t(j)] - pred_no_act;
            sse[size_t(j)] += e * e;
            sse_no_dc[size_t(j)] += e_dc * e_dc;
            sse_no_jm[size_t(j)] += e_jm * e_jm;
            sse_no_act[size_t(j)] += e_act * e_act;
        }
    }
    const double n = double(samples.size());
    for (int j = 0; j < kJoints; ++j) {
        out.info.rmse[size_t(j)] = std::sqrt(sse[size_t(j)] / n);
        out.info.rmse_no_dc[size_t(j)] = std::sqrt(sse_no_dc[size_t(j)] / n);
        out.info.rmse_no_jm[size_t(j)] = std::sqrt(sse_no_jm[size_t(j)] / n);
        out.info.rmse_no_actuators[size_t(j)] = std::sqrt(sse_no_act[size_t(j)] / n);
    }
    return out;
}

}  // namespace ident
}  // namespace litearm
