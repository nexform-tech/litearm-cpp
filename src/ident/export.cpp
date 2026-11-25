// YAML/SN export and guarded firmware model write-back.
#include "litearm/ident/export.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "litearm/arm.hpp"
#include "litearm/errors.hpp"
#include "litearm/ident/trajectory.hpp"

namespace litearm {
namespace ident {
namespace {

using Mat3 = Eigen::Matrix3d;
using Vec3d = Eigen::Vector3d;

/// Poses used to compare the firmware's gravity model with the host RNEA after
/// a write.  These are validation poses, not an excitation design.
std::vector<Vec7> gravity_check_poses() {
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0},
        {0.4, 0.2, -0.7, -1.6, 0.4, 0.6, 0.2},
        {-0.4, 0.3, -0.3, -1.4, -0.4, 0.6, -0.2},
        {-0.8, 0.4, 0.4, -1.9, -0.5, 0.5, -0.4},
        {0.8, 0.8, -0.4, -1.0, 0.6, -0.6, 0.4},
    };
}

std::string num(double v) {
    if (std::isnan(v)) return ".nan";
    if (std::isinf(v)) return v > 0 ? ".inf" : "-.inf";
    std::ostringstream os;
    os << std::setprecision(17) << v;
    return os.str();
}

/// Every exported/written inertia block must be a physically realizable rigid
/// body.  The solver already projects its output; this catches hand-edited or
/// legacy YAML files loaded through --load-result.
void require_physical(const Phi70& phi, const char* what) {
    std::string why;
    if (!physically_valid(phi, &why)) {
        throw InvalidCommandError(std::string(what) +
                                  ": inertia parameters are not physically valid (" +
                                  why + ")");
    }
}

std::string sci(double v) {
    if (std::isnan(v)) return ".nan";
    if (std::isinf(v)) return v > 0 ? ".inf" : "-.inf";
    std::ostringstream os;
    os << std::scientific << std::setprecision(16) << v;
    return os.str();
}

std::string trim(const std::string& s) {
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const auto b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

std::string indent_of(const std::string& s) {
    return std::string(s.begin(), std::find_if(s.begin(), s.end(),
                                               [](char c) { return c != ' ' && c != '\t'; }));
}

struct BodyCom {
    double m = 0.0;
    Vec3d c{};
    Mat3 I{};
};

BodyCom phi_link_com(const std::array<double, 10>& p) {
    BodyCom b;
    b.m = p[0];
    b.c = Vec3d(p[1], p[2], p[3]) / b.m;
    Mat3 Io;
    Io << p[4], p[5], p[7],
          p[5], p[6], p[8],
          p[7], p[8], p[9];
    b.I = Io - b.m * ((b.c.dot(b.c)) * Mat3::Identity() - b.c * b.c.transpose());
    return b;
}

double parse_number(const std::string& s, const char* what) {
    try {
        std::size_t used = 0;
        const double v = std::stod(s, &used);
        if (used != s.size()) throw std::invalid_argument("trailing");
        if (!std::isfinite(v)) throw std::invalid_argument("non-finite");
        return v;
    } catch (const std::exception&) {
        throw InvalidCommandError(std::string(what) + ": bad number '" + s + "'");
    }
}

}  // namespace

std::array<double, 10> phi_to_body10(const std::array<double, 10>& phi_link) {
    const BodyCom b = phi_link_com(phi_link);
    return {b.m, b.c[0], b.c[1], b.c[2],
            b.I(0, 0), b.I(0, 1), b.I(0, 2), b.I(1, 1), b.I(1, 2), b.I(2, 2)};
}

std::array<double, 10> body10_to_phi(const std::array<double, 10>& body) {
    const double m = body[0];
    const double cx = body[1], cy = body[2], cz = body[3];
    const double ixx = body[4], ixy = body[5], ixz = body[6];
    const double iyy = body[7], iyz = body[8], izz = body[9];
    return {m, m * cx, m * cy, m * cz,
            ixx + m * (cy * cy + cz * cz),
            ixy - m * cx * cy,
            iyy + m * (cx * cx + cz * cz),
            ixz - m * cx * cz,
            iyz - m * cy * cz,
            izz + m * (cx * cx + cy * cy)};
}

std::string identified_yaml(const IdentResult& result, const ExportInfo& info) {
    require_physical(result.phi, "identified_yaml export");
    std::ostringstream os;
    os << "schema: pylitearm.identified_params.v1\n";
    os << "phi_semantics: pinocchio.Inertia.toDynamicParameters@joint-origin\n";
    os << "phi_frame: craig_mdh_link_frames (DH/litearm_dh.urdf)\n";
    os << "description: LiteArm identified parameters (10 per link, 7 links)\n";
    os << "n_params: 70\n";
    os << "n_links: 7\n";
    os << "joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]\n";
    if (!info.urdf_path.empty()) os << "urdf_path: " << info.urdf_path << "\n";
    if (!info.urdf_sha256.empty()) os << "urdf_sha256: " << info.urdf_sha256 << "\n";
    os << "robot_serial: " << info.robot_serial << "\n";
    os << "n_segments: " << info.centers.size() << "\n";
    os << "n_harmonics: " << info.n_harmonics << "\n";
    os << "period: " << num(info.period) << "\n";
    os << "ee_z_min: " << num(info.ee_z_min) << "\n";

    auto emit_centers = [&](const char* key, const std::vector<Vec7>& centers) {
        if (centers.empty()) {
            os << key << ": []\n";
            return;
        }
        os << key << ":\n";
        for (const auto& c : centers) {
            os << "  - [" << num(c[0]);
            for (int j = 1; j < kJoints; ++j) os << ", " << num(c[size_t(j)]);
            os << "]\n";
        }
    };
    emit_centers("centers", info.centers);
    emit_centers("executed_centers", info.executed_centers);

    os << "identification_info:\n";
    os << "  rank: " << result.info.rank << "\n";
    os << "  rank_fit: " << result.info.rank_fit << "\n";
    os << "  rank_inertia: " << result.info.rank_inertia << "\n";
    os << "  rank_is_sampled: " << (result.info.rank_is_sampled ? "true" : "false") << "\n";
    os << "  sv_max: " << num(result.info.sv_max) << "\n";
    os << "  sv_min: " << num(result.info.sv_min) << "\n";
    os << "  cond: " << num(result.info.cond) << "\n";
    os << "  n_samples: " << result.info.n_samples << "\n";
    os << "  n_rows: " << result.info.n_rows << "\n";
    os << "  n_params: " << result.info.n_params << "\n";
    os << "  n_inertia_params: 70\n";
    os << "  n_coulomb_params: 7\n";
    os << "  n_viscous_params: 7\n";
    os << "  n_dc_params: 7\n";
    os << "  n_armature_params: 5\n";
    os << "  n_friction_params: 21\n";
    os << "  friction_model: coulomb_pos_neg_viscous+armature\n";
    os << "  parameter_layout: {inertia: [0, 70], coulomb: [70, 77], "
          "viscous: [77, 84], coulomb_dc: [84, 91], armature: [91, 96]}\n";
    os << "  dropped_columns: [91, 92]\n";
    os << "  dropped_reason: jm1/jm2 structurally collinear with link1/link2 Izz\n";
    os << "  armature_fixed_zero: [1, 2]\n";
    os << "  projection: {coulomb: " << result.info.projection[0]
       << ", viscous: " << result.info.projection[1]
       << ", dc: " << result.info.projection[2]
       << ", armature: " << result.info.projection[3] << "}\n";
    os << "  cad_fill_method: base_params.fill_unidentifiable_from_cad\n";
    os << "  physical_links: " << (result.info.physical_links ? "true" : "false") << "\n";
    os << "  rmse: [";
    for (int j = 0; j < kJoints; ++j) {
        if (j) os << ", ";
        os << num(result.info.rmse[size_t(j)]);
    }
    os << "]\n";

    os << "phi_flat:\n";
    for (double v : result.phi) os << "- " << num(v) << "\n";

    os << "per_link:\n";
    const char* slots[10] = {"m", "mcx", "mcy", "mcz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz"};
    for (int i = 0; i < kJoints; ++i) {
        os << "  '" << (i + 1) << "':\n";
        for (int k = 0; k < 10; ++k) {
            os << "    " << slots[k] << ": " << num(result.phi[size_t(i * 10 + k)]) << "\n";
        }
    }

    os << "friction:\n";
    for (int j = 0; j < kJoints; ++j) {
        os << "  joint_" << (j + 1) << ":\n";
        os << "    coulomb: " << num(result.coulomb[size_t(j)]) << "\n";
        os << "    viscous: " << num(result.viscous[size_t(j)]) << "\n";
        os << "    fv: " << num(result.viscous[size_t(j)]) << "\n";
        os << "    fc0: " << num(result.fc0()[size_t(j)]) << "\n";
        os << "    fc1: " << num(result.fc1()[size_t(j)]) << "\n";
        os << "    coulomb_dc: " << num(result.dc[size_t(j)]) << "\n";
    }
    os << "armature:\n";
    for (int j = 0; j < kJoints; ++j) {
        os << "  joint_" << (j + 1) << ": " << num(result.armature[size_t(j)]) << "\n";
    }
    os << "armature_fixed_zero: [1, 2]\n";
    os << "identification_type: inertia+friction(coulomb_pos_neg_viscous)+armature\n";
    os << "cad_fusion: true\n";
    os << "cad_fill_method: base_params.fill_unidentifiable_from_cad\n";

    os << "controller_dynamics_parameters:\n";
    const char* sn[16] = {"m", "cmx", "cmy", "cmz", "ixx", "ixy", "ixz",
                          "iyy", "iyz", "izz", "fv", "fv2", "fv3", "fc0", "fc1", "jm"};
    for (int i = 0; i < kJoints; ++i) {
        std::array<double, 10> p{};
        for (int k = 0; k < 10; ++k) p[size_t(k)] = result.phi[size_t(i * 10 + k)];
        const auto b = phi_to_body10(p);
        const double fv = result.viscous[size_t(i)];
        const double fc0 = result.fc0()[size_t(i)];
        const double fc1 = result.fc1()[size_t(i)];
        const double vals[16] = {b[0], b[1], b[2], b[3], b[4], b[5], b[6],
                                 b[7], b[8], b[9], fv, 0.0, 0.0, fc0, fc1,
                                 result.armature[size_t(i)]};
        os << "  joint_" << (i + 1) << ":\n";
        for (int k = 0; k < 16; ++k) os << "    " << sn[k] << ": " << num(vals[k]) << "\n";
    }
    return os.str();
}

IdentResult load_identified_yaml(const std::string& text) {
    auto value_of = [](const std::string& s) {
        return parse_number(s, "identified_params.yaml");
    };

    enum class Section { none, phi, friction, armature };
    IdentResult r;
    Section section = Section::none;
    int phi_count = 0;
    int joint = -1;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        const std::string t = trim(line);
        if (t.empty() || t[0] == '#') continue;

        // phi_flat items are top-level "- value" lines.
        if (section == Section::phi && t[0] == '-') {
            if (phi_count >= kInertiaParams) {
                throw InvalidCommandError(
                    "identified_params.yaml: more than 70 phi values");
            }
            r.phi[size_t(phi_count++)] = value_of(trim(t.substr(1)));
            continue;
        }

        if (line == t) {  // top-level key
            joint = -1;
            if (t == "phi_flat:") section = Section::phi;
            else if (t == "friction:") section = Section::friction;
            else if (t == "armature:") section = Section::armature;
            else section = Section::none;
            continue;
        }

        if (section == Section::friction) {
            if (t.rfind("joint_", 0) == 0 && t.back() == ':') {
                const int id = std::stoi(t.substr(6, t.size() - 7));
                if (id < 1 || id > kJoints) {
                    throw InvalidCommandError(
                        "identified_params.yaml: bad friction joint " + t);
                }
                joint = id - 1;
                continue;
            }
            if (joint < 0) continue;
            const auto colon = t.find(':');
            if (colon == std::string::npos) continue;
            const std::string key = trim(t.substr(0, colon));
            const double v = value_of(trim(t.substr(colon + 1)));
            if (key == "coulomb") r.coulomb[size_t(joint)] = v;
            else if (key == "viscous" || key == "fv") r.viscous[size_t(joint)] = v;
            else if (key == "coulomb_dc") r.dc[size_t(joint)] = v;
        } else if (section == Section::armature) {
            if (t.rfind("joint_", 0) != 0) continue;
            const auto colon = t.find(':');
            if (colon == std::string::npos) continue;
            const int id = std::stoi(trim(t.substr(6, colon - 6)));
            if (id < 1 || id > kJoints) {
                throw InvalidCommandError(
                    "identified_params.yaml: bad armature joint " + t);
            }
            r.armature[size_t(id - 1)] = value_of(trim(t.substr(colon + 1)));
        }
    }

    if (phi_count != kInertiaParams) {
        throw InvalidCommandError(
            "identified_params.yaml: expected 70 phi values, got " +
            std::to_string(phi_count));
    }
    r.armature[0] = 0.0;
    r.armature[1] = 0.0;
    return r;
}

IdentResult load_sn_yaml(const std::string& text, bool check_physical) {
    const char* keys[16] = {"m", "cmx", "cmy", "cmz", "ixx", "ixy", "ixz",
                            "iyy", "iyz", "izz", "fv", "fv2", "fv3", "fc0", "fc1", "jm"};
    std::vector<std::string> lines;
    {
        std::istringstream in(text);
        std::string line;
        while (std::getline(in, line)) lines.push_back(line);
    }

    std::vector<std::array<double, 16>> blocks;
    for (size_t li = 0; li < lines.size(); ++li) {
        if (trim(lines[li]) != "dynamics_parameters:") continue;
        if (blocks.size() >= size_t(kJoints)) {
            throw InvalidCommandError("SN load: more than 7 dynamics_parameters blocks");
        }
        const size_t base_indent = indent_of(lines[li]).size();
        std::array<double, 16> block{};
        bool seen[16] = {false};
        for (size_t k = li + 1; k < lines.size(); ++k) {
            const std::string t = trim(lines[k]);
            if (t.empty()) continue;
            if (indent_of(lines[k]).size() <= base_indent) break;
            const auto colon = t.find(':');
            if (colon == std::string::npos) continue;
            const std::string key = trim(t.substr(0, colon));
            int ki = -1;
            for (int x = 0; x < 16; ++x) if (key == keys[x]) { ki = x; break; }
            if (ki < 0) continue;
            block[size_t(ki)] =
                parse_number(trim(t.substr(colon + 1)), "SN dynamics_parameters");
            seen[ki] = true;
        }
        for (int x = 0; x < 16; ++x) {
            if (!seen[x]) {
                throw InvalidCommandError(
                    "SN load: dynamics_parameters block " +
                    std::to_string(blocks.size() + 1) + " is missing key " + keys[x]);
            }
        }
        blocks.push_back(block);
    }
    if (blocks.size() != size_t(kJoints)) {
        throw InvalidCommandError("SN load: expected 7 dynamics_parameters blocks, got " +
                                  std::to_string(blocks.size()));
    }

    IdentResult r;
    for (int i = 0; i < kJoints; ++i) {
        const auto& b = blocks[size_t(i)];
        const double m = b[0];
        if (!(m > 1e-4)) {
            throw InvalidCommandError("SN load: link " + std::to_string(i + 1) +
                                      " has non-positive mass");
        }
        // body10 about the COM -> phi about the joint origin (parallel axis).
        std::array<double, 10> body{};
        for (int k = 0; k < 10; ++k) body[size_t(k)] = b[size_t(k)];
        const auto p = body10_to_phi(body);
        for (int k = 0; k < 10; ++k) r.phi[size_t(i * 10 + k)] = p[size_t(k)];
        r.viscous[size_t(i)] = b[10];
        r.coulomb[size_t(i)] = 0.5 * (b[13] - b[14]);
        r.dc[size_t(i)] = 0.5 * (b[13] + b[14]);
        r.armature[size_t(i)] = b[15];
    }
    if (check_physical) require_physical(r.phi, "SN load");
    return r;
}

IdentResult load_result_yaml(const std::string& text, bool check_physical) {
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (trim(line) == "robot_group:") return load_sn_yaml(text, check_physical);
    }
    return load_identified_yaml(text);
}

std::string sn_template_yaml(const IdentResult& result,
                             const std::string& template_text,
                             bool allow_nonphysical) {
    if (!allow_nonphysical) require_physical(result.phi, "SN export");
    std::vector<std::array<double, 10>> bodies;
    for (int i = 0; i < kJoints; ++i) {
        std::array<double, 10> p{};
        for (int k = 0; k < 10; ++k) p[size_t(k)] = result.phi[size_t(i * 10 + k)];
        bodies.push_back(phi_to_body10(p));
    }
    const char* keys[16] = {"m", "cmx", "cmy", "cmz", "ixx", "ixy", "ixz",
                            "iyy", "iyz", "izz", "fv", "fv2", "fv3", "fc0", "fc1", "jm"};
    std::vector<std::array<double, 16>> vals;
    for (int i = 0; i < kJoints; ++i) {
        const auto& b = bodies[size_t(i)];
        const double fc0 = result.fc0()[size_t(i)];
        const double fc1 = result.fc1()[size_t(i)];
        if (fc0 < -1e-12) throw InvalidCommandError("SN export: fc0 must be >= 0");
        if (fc1 > 1e-12) throw InvalidCommandError("SN export: fc1 must be <= 0");
        vals.push_back({b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9],
                        result.viscous[size_t(i)], 0.0, 0.0, fc0, fc1,
                        result.armature[size_t(i)]});
    }

    std::istringstream in(template_text);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);

    int block = 0;
    int replaced = 0;
    for (size_t li = 0; li < lines.size(); ++li) {
        if (trim(lines[li]).find("dynamics_parameters:") == std::string::npos) continue;
        if (block >= kJoints) {
            throw InvalidCommandError("SN export: template has more than 7 dynamics blocks");
        }
        const std::string base_indent = indent_of(lines[li]);
        bool seen[16] = {false};
        size_t k = li + 1;
        for (; k < lines.size(); ++k) {
            const std::string t = trim(lines[k]);
            if (t.empty()) continue;
            if (indent_of(lines[k]).size() < base_indent.size()) break;
            const auto colon = t.find(':');
            if (colon == std::string::npos) continue;
            const std::string key = trim(t.substr(0, colon));
            int ki = -1;
            for (int x = 0; x < 16; ++x) if (key == keys[x]) { ki = x; break; }
            if (ki < 0) continue;
            const std::string ind = indent_of(lines[k]);
            lines[k] = ind + key + ": " + sci(vals[size_t(block)][size_t(ki)]);
            seen[ki] = true;
            ++replaced;
        }
        for (int x = 0; x < 16; ++x) {
            if (!seen[x]) {
                throw InvalidCommandError("SN export: dynamics_parameters block is missing key " +
                                          std::string(keys[x]));
            }
        }
        ++block;
    }
    if (block != kJoints) {
        throw InvalidCommandError("SN export: template must contain exactly 7 dynamics blocks");
    }
    std::ostringstream out;
    for (size_t i = 0; i < lines.size(); ++i) {
        out << lines[i];
        if (i + 1 < lines.size()) out << '\n';
    }
    return out.str();
}

WriteModelReport write_model(Arm& arm, const IdentResult& result,
                             const WriteModelOptions& options) {
    // Refuse physically impossible parameters before touching the arm (and
    // before disable(): a bad file must not drop a holding arm).  With
    // allow_nonphysical the fixed-inertia bank is written back verbatim.
    if (!options.allow_nonphysical) require_physical(result.phi, "write_model");

    // The firmware's friction contract is absolute (0x26 items 9/10/11:
    // fv >= 0, fc0 >= 0, fc1 <= 0).  Check it before any write: a rejection
    // after the model commit would leave the arm half-updated.
    std::vector<std::vector<double>> new_friction;
    if (options.write_friction) {
        const Vec7 fc0 = result.fc0();
        const Vec7 fc1 = result.fc1();
        new_friction.assign(3, std::vector<double>(size_t(kJoints), 0.0));
        for (int j = 0; j < kJoints; ++j) {
            const double fv = result.viscous[size_t(j)];
            const double c0 = fc0[size_t(j)];
            const double c1 = fc1[size_t(j)];
            if (!std::isfinite(fv) || !std::isfinite(c0) || !std::isfinite(c1) ||
                fv < -1e-12 || c0 < -1e-12 || c1 > 1e-12) {
                throw InvalidCommandError(
                    "write_model: joint " + std::to_string(j + 1) +
                    " violates the friction contract (fv>=0, fc0>=0, fc1<=0)");
            }
            // Clamp tiny negatives so the firmware sees a valid sign.
            new_friction[0][size_t(j)] = std::max(0.0, fv);
            new_friction[1][size_t(j)] = std::max(0.0, c0);
            new_friction[2][size_t(j)] = std::min(0.0, c1);
        }
    }

    if (arm.n() != kJoints) {
        throw InvalidCommandError("write_model: requires a 7-joint arm");
    }
    if (!arm.model().probe()) {
        throw UnsupportedByFirmwareError("write_model: firmware lacks model import");
    }
    try {
        arm.disable();
    } catch (const LiteArmError&) {
        // Already disabled is fine; the model commit only requires a disarmed arm.
    }
    const auto status = arm.model().status().value;
    if (status.staged_mask != 0) {
        throw InvalidCommandError("write_model: firmware has staged model changes");
    }

    WriteModelReport rep;
    for (int i = 1; i <= kJoints; ++i) {
        rep.old_bodies.push_back(arm.model().get_body(i).value);
    }
    rep.old_jm = arm.model().get_jm().value;
    if (options.write_friction) {
        rep.old_friction.resize(3);
        for (int k = 0; k < 3; ++k) {
            rep.old_friction[size_t(k)] = arm.get_ff_vec(9 + k).value;
        }
    }

    for (int i = 0; i < kJoints; ++i) {
        std::array<double, 10> p{};
        for (int k = 0; k < 10; ++k) p[size_t(k)] = result.phi[size_t(i * 10 + k)];
        const auto b = phi_to_body10(p);
        rep.new_bodies.push_back({b[0], b[1], b[2], b[3], b[4], b[5],
                                  b[6], b[7], b[8], b[9]});
    }
    rep.new_jm.assign(size_t(kJoints), 0.0);
    for (int j = 0; j < kJoints; ++j) rep.new_jm[size_t(j)] = result.armature[size_t(j)];

    for (int i = 0; i < kJoints; ++i) arm.model().set_body(i + 1, rep.new_bodies[size_t(i)]);
    const uint16_t mask = options.write_jm ? MODEL_MASK_WRITTEN : uint16_t(0x0FE);
    if (options.write_jm) arm.model().set_jm(rep.new_jm);
    arm.model().commit(mask);
    rep.staged_mask = mask;
    rep.committed = true;

    for (int i = 0; i < kJoints; ++i) {
        const auto got = arm.model().get_body(i + 1).value;
        for (int k = 0; k < 10; ++k) {
            const double d = std::abs(got[size_t(k)] - rep.new_bodies[size_t(i)][size_t(k)]);
            rep.max_body_readback_error = std::max(rep.max_body_readback_error, d);
        }
    }
    if (options.write_jm) {
        const auto got = arm.model().get_jm().value;
        for (int j = 0; j < kJoints; ++j) {
            rep.max_body_readback_error = std::max(
                rep.max_body_readback_error,
                std::abs(got[size_t(j)] - rep.new_jm[size_t(j)]));
        }
    }

    if (options.write_friction) {
        rep.new_friction = new_friction;
        arm.set_ff_vec(9, rep.new_friction[0]);
        arm.set_ff_vec(10, rep.new_friction[1]);
        arm.set_ff_vec(11, rep.new_friction[2]);
        rep.friction_written = true;
        for (int k = 0; k < 3; ++k) {
            const auto got = arm.get_ff_vec(9 + k).value;
            for (int j = 0; j < kJoints; ++j) {
                rep.max_friction_readback_error = std::max(
                    rep.max_friction_readback_error,
                    std::abs(got[size_t(j)] - rep.new_friction[size_t(k)][size_t(j)]));
            }
        }
    }

    if (options.check_gravity) {
        for (const auto& q : gravity_check_poses()) {
            const Vec7 g_host = rnea(q, Vec7{}, Vec7{}, result.phi);
            const auto g_fw = arm.model().get_gravity(std::vector<double>(q.begin(), q.end())).value;
            for (int j = 0; j < kJoints; ++j) {
                rep.max_gravity_delta_nm = std::max(
                    rep.max_gravity_delta_nm, std::abs(g_fw[size_t(j)] - g_host[size_t(j)]));
            }
        }
        if (rep.max_gravity_delta_nm > options.gravity_tolerance_nm &&
            options.rollback_on_gravity_mismatch) {
            for (int i = 0; i < kJoints; ++i)
                arm.model().set_body(i + 1, rep.old_bodies[size_t(i)]);
            uint16_t rollback_mask = 0x0FE;
            if (options.write_jm) {
                arm.model().set_jm(rep.old_jm);
                rollback_mask = MODEL_MASK_WRITTEN;
            }
            arm.model().commit(rollback_mask);
            if (rep.friction_written) {
                arm.set_ff_vec(9, rep.old_friction[0]);
                arm.set_ff_vec(10, rep.old_friction[1]);
                arm.set_ff_vec(11, rep.old_friction[2]);
            }
            rep.rolled_back = true;
        }
    }
    return rep;
}

}  // namespace ident
}  // namespace litearm
