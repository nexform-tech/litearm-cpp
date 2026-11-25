// 96-parameter MAP identification solver: 70 inertia + 7 symmetric Coulomb
// + 7 viscous + 7 asymmetry + 5 armature (jm3..jm7).
#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "litearm/ident/model.hpp"

namespace litearm {
namespace ident {

struct IdentifyOptions {
    double regularization = 1e-2;   // inertia CAD prior
    double friction_ridge = 1e-2;   // a and fv zero-mean ridge
    double dc_ridge = 1e5;          // asymmetry b zero-mean ridge
    bool fix_dc = false;            // drop the DC columns, b = 0
    double armature_ridge = 1e-2;   // jm3..jm7 zero-mean ridge
    double K_coulomb = 300.0;
    bool physical_links = true;     // blend non-PD I_com toward CAD
    double cad_min_eig = 1e-5;
    /// Fix all 70 inertia parameters to `fixed_phi` and solve only the
    /// friction/armature columns.  The fixed values are kept exactly, even if
    /// they are not physically valid (e.g. a controller SN bank with negative
    /// inertias for a mass-only model).
    bool fix_inertia = false;
    Phi70 fixed_phi{};
    /// Fix every link mass to 1.0 and re-fit the remaining inertial parameters
    /// (the controller's factory model uses the unit-mass gauge).  The mass
    /// columns become a constant contribution; h = m*c and I_origin are still
    /// free, so the torque fit stays close to the physical-mass solution.
    bool unit_mass = false;
    std::size_t diagnostics_max_rows = 100000;
    Vec3 gravity{0.0, 0.0, -9.81};
};

struct IdentificationInfo {
    int rank = 0;
    int rank_fit = 0;
    int rank_inertia = 0;
    bool rank_is_sampled = false;
    double sv_max = 0.0;
    double sv_min = 0.0;
    double cond = 0.0;
    int n_samples = 0;
    int n_rows = 0;
    int n_params = kActiveParams;
    std::array<int, 4> projection{{0, 0, 0, 0}};  // a, fv, |b|>a, jm
    std::vector<double> cad_blend_alpha;
    bool physical_links = true;
    bool fixed_inertia = false;
    std::array<double, kJoints> rmse{};
    std::array<double, kJoints> rmse_no_dc{};
    std::array<double, kJoints> rmse_no_jm{};
    std::array<double, kJoints> rmse_no_actuators{};
};

struct IdentResult {
    Phi70 phi{};      // CAD-filled / physically blended
    Phi70 phi_raw{};  // raw data solution
    Vec7 coulomb{};   // a
    Vec7 viscous{};   // fv
    Vec7 dc{};        // b
    Vec7 armature{};  // jm, jm1/jm2 = 0
    IdentificationInfo info{};

    Vec7 fc0() const;
    Vec7 fc1() const;
};

IdentResult identify(const std::vector<SamplePoint>& samples,
                     const IdentifyOptions& options = {});

}  // namespace ident
}  // namespace litearm
