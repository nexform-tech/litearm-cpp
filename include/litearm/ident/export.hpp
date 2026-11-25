// YAML/SN export and firmware model write-back for identified parameters.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "litearm/ident/model.hpp"
#include "litearm/ident/solver.hpp"

namespace litearm {

class Arm;

namespace ident {

struct ExportInfo {
    std::string robot_serial = "UNASSIGNED";
    std::string urdf_path;
    std::string urdf_sha256;
    int n_cycles = 0;
    int n_harmonics = 5;
    double period = 24.0;
    double ee_z_min = 0.15;
    std::vector<Vec7> centers;
    std::vector<Vec7> executed_centers;
};

/// Python-SDK-compatible identified_params.yaml (70 phi + actuator fields).
std::string identified_yaml(const IdentResult& result, const ExportInfo& info);

/// Parse the inertia/friction/armature blocks of identified_yaml() output.
/// Used to write a saved result back to the firmware without re-capturing.
IdentResult load_identified_yaml(const std::string& text);

/// Parse the seven dynamics_parameters blocks of an exported SN/HYY config.
/// body10 (about the COM, controller DH link frame) is converted back to the
/// joint-origin phi used by write_model(); fc0/fc1 recover coulomb and dc.
IdentResult load_sn_yaml(const std::string& text, bool check_physical = true);

/// Load a saved identification result; accepts either an exported SN config
/// or the legacy identified_params schema.  Pass check_physical=false to
/// load a controller bank whose inertia is not physically realizable.
IdentResult load_result_yaml(const std::string& text,
                             bool check_physical = true);

/// Patch the 16 dynamics_parameters values of a known SN/HYY template.
/// Non-physical inertia can only be exported with allow_nonphysical=true.
std::string sn_template_yaml(const IdentResult& result,
                             const std::string& template_text,
                             bool allow_nonphysical = false);

/// phi link (about joint origin) -> firmware body10 (about COM).
std::array<double, 10> phi_to_body10(const std::array<double, 10>& phi_link);

/// Firmware body10 (about COM) -> phi about the joint origin.  No physical
/// validity check: the firmware bank can hold a non-physical identified model
/// (e.g. the vendor's unit-mass model) and its RNEA is still well defined.
std::array<double, 10> body10_to_phi(const std::array<double, 10>& body);

struct WriteModelOptions {
    bool write_jm = false;
    /// Also write the identified friction to the firmware's 0x26 vectors:
    /// item 9 = fv, item 10 = fc0, item 11 = fc1.
    bool write_friction = false;
    bool check_gravity = true;
    double gravity_tolerance_nm = 0.25;
    bool rollback_on_gravity_mismatch = false;
    /// Allow writing an inertia block that fails physically_valid().  Needed
    /// when the inertia is fixed from a controller bank with negative inertias.
    bool allow_nonphysical = false;
};

struct WriteModelReport {
    std::vector<std::vector<double>> old_bodies;
    std::vector<std::vector<double>> new_bodies;
    std::vector<double> old_jm;
    std::vector<double> new_jm;
    /// fv/fc0/fc1 in that order; populated only when write_friction is set.
    std::vector<std::vector<double>> old_friction;
    std::vector<std::vector<double>> new_friction;
    std::uint16_t staged_mask = 0;
    double max_body_readback_error = 0.0;
    double max_friction_readback_error = 0.0;
    double max_gravity_delta_nm = 0.0;
    bool committed = false;
    bool friction_written = false;
    bool rolled_back = false;
};

/// Write bodies 1..7 (and optionally jm and friction) through arm.model() /
/// set_ff_vec(); never calls save_params().  The arm must be disarmed/supported
/// by the caller.  allow_nonphysical skips the host physical-validity check so
/// a fixed-inertia controller bank can be written back verbatim.
WriteModelReport write_model(Arm& arm, const IdentResult& result,
                             const WriteModelOptions& options = {});

}  // namespace ident
}  // namespace litearm
