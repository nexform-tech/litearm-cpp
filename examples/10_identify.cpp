// Example 10 - 96-parameter Fourier identification.
//
// Default scheme: the 70 inertia parameters are fixed from
// examples/SN.yaml; only the friction/armature block
// (fv, fc0, fc1, jm) is identified.  Pass --full-inertia to solve all 96
// parameters, or --fixed-model FILE to use another fixed inertia bank.
//
// The only result file written is the controller's SN dynamics_parameters
// config (`robot_group -> joint -> dynamics_parameters`): the 16 firmware
// values per joint are replaced, while kinematics/axis/tool/base stay as in
// the template supplied with --sn-template (default: the fixed model file).
//
// Excitation comes from a coefficient file: the shipped two-segment default
// examples/fourier_coeffs_default.txt (24 s period), or another file supplied
// with --coeffs.  Capture is continuous pure-PD tracking sampled at 100 Hz
// (q/dq/tau from the status stream), so the trajectory never pauses.
//
// Every link mass is written as 1 by default, matching the controller's
// factory bank.  This is the unit-mass gauge: the identification fixes m = 1
// and re-fits the remaining inertial parameters (h = m*c and I_origin stay
// free), rather than rescaling a physical-mass solution.  Use --physical-mass
// for the kg-mass model.
//
// Link frames follow the `DH/` folder, not the vendor URDF: the host model is
// the controller's Craig/MDH chain from DH/litearm_dh.urdf (joints 4/6/7 on
// -Z, ee_link merged into link 7).  The exported dynamics are therefore
// expressed in those DH link frames and are directly comparable with
// DH/dh_params.yaml; no per-link frame conversion is needed.  --urdf and
// --urdf-sha256 are recorded in the SN file header.
//
// Offline (no hardware):
//   ./10_identify --input-csv fourier_log.csv --output ident_result \
//       --sn-template /path/to/SN<serial>.yaml
//
// Real arm (read-only plan unless --go):
//   ./10_identify --sn-template /path/to/SN<serial>.yaml  # plan only
//   ./10_identify --go --cycles 1 --sn-template ...       # one cycle per segment
//   ./10_identify --go --cycles 4 --sn-template ... --write-model
//
// Explicit excitation (77 coefficients per segment; fitted to the joint limits
// per joint by a center shift and, if needed, a uniform harmonic scale; the file
// is not modified; no ee-height lift / singularity check):
//   ./10_identify --go --coeffs examples/fourier_coeffs_default.txt \
//       --sn-template /path/to/SN<serial>.yaml
//
// Write a saved result back without capturing (arm must be supported):
//   ./10_identify --load-result ident_result/SN_*.yaml --write-model
//
// This path is deliberately explicit: --go is required for motion and
// --write-model is required to touch the firmware dynamics model.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "litearm/ident/capture.hpp"
#include "litearm/ident/coefficients.hpp"
#include "litearm/ident/csv.hpp"
#include "litearm/ident/export.hpp"
#include "litearm/ident/solver.hpp"
#include "litearm/ident/trajectory.hpp"

#include "_common.hpp"

using namespace litearm;
using namespace litearm::ident;

namespace {

constexpr double kDefaultPeriod = 24.0;
constexpr const char* kDefaultCoeffsName = "fourier_coeffs_default.txt";
constexpr const char* kDefaultFixedModelName = "SN.yaml";

Vec7 parse_vec7(const std::string& text, const Vec7& fallback) {
    Vec7 out = fallback;
    std::istringstream ss(text);
    std::string part;
    int i = 0;
    while (std::getline(ss, part, ',') && i < kJoints) {
        out[size_t(i++)] = std::atof(part.c_str());
    }
    return out;
}

std::string read_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw InvalidCommandError("cannot read " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// A shipped example input, resolved from the source tree so the binary also
/// works when run from the build directory.
std::string default_example_path(const char* name, const char* option) {
    namespace fs = std::filesystem;
    std::vector<fs::path> candidates;
#ifdef LITEARM_EXAMPLES_DIR
    candidates.emplace_back(fs::path(LITEARM_EXAMPLES_DIR) / name);
#endif
    candidates.emplace_back(fs::path("examples") / name);
    candidates.emplace_back(fs::path("..") / "examples" / name);
    for (const auto& path : candidates) {
        std::error_code ec;
        if (fs::exists(path, ec)) return path.string();
    }
    throw InvalidCommandError(std::string("default file not found: ") + name +
                              "; pass " + option);
}

std::string default_coeffs_path() {
    return default_example_path(kDefaultCoeffsName, "--coeffs FILE");
}

std::string default_fixed_model_path() {
    return default_example_path(kDefaultFixedModelName, "--fixed-model FILE");
}

/// Local YYYYMMDD_HHMMSS stamp shared by every file of one identification run.
/// Timestamped names keep successive runs as a time series instead of
/// overwriting the previous result.
std::string timestamp_suffix() {
    const std::time_t now = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &now);
#else
    localtime_r(&now, &tm);
#endif
    char buf[32] = {};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

void print_result(const IdentResult& r) {
    if (r.info.fixed_inertia) {
        std::printf("inertia: fixed (not re-identified); only friction/armature solved\n");
    }
    std::printf("rank=%d rank_fit=%d rank_inertia=%d/%d sampled=%d\n",
                r.info.rank, r.info.rank_fit, r.info.rank_inertia, kInertiaParams,
                int(r.info.rank_is_sampled));
    std::printf("RMSE (Nm):");
    for (double v : r.info.rmse) std::printf(" %.4f", v);
    std::printf("\n");
    std::printf("a (symmetric Coulomb):");
    for (double v : r.coulomb) std::printf(" %+.4f", v);
    std::printf("\nfc0:");
    for (double v : r.fc0()) std::printf(" %+.4f", v);
    std::printf("\nfc1:");
    for (double v : r.fc1()) std::printf(" %+.4f", v);
    std::printf("\nfv:");
    for (double v : r.viscous) std::printf(" %+.4f", v);
    std::printf("\njm:");
    for (double v : r.armature) std::printf(" %+.5f", v);
    std::printf("\n");
}

ExportInfo make_export_info(const example::Args& args, int cycles, int harmonics,
                            double period, double ee_z_min,
                            const std::vector<Vec7>& centers) {
    ExportInfo info;
    info.robot_serial = args.str("serial", "UNASSIGNED");
    info.urdf_path = args.str("urdf", "");
    info.urdf_sha256 = args.str("urdf-sha256", "");
    info.n_cycles = cycles;
    info.n_harmonics = harmonics;
    info.period = period;
    info.ee_z_min = ee_z_min;
    info.centers = centers;
    info.executed_centers = centers;
    return info;
}

/// The controller's factory bank uses m = 1 for every arm link, so the SN
/// export defaults to the unit-mass gauge (re-identified, not rescaled).
bool use_unit_mass(const example::Args& args) {
    return !args.has("physical-mass");
}

/// External coefficients replace the internal excitation design; the loaded
/// trajectory is fitted to the joint limits (no ee lift or sigma check).
bool external_coefficients_requested(const example::Args& args) {
    return args.has("coeffs") || args.has("coeffs-inline");
}

std::string sn_template_path(const example::Args& args) {
    return args.has("sn-template") ? args.str("sn-template", "")
                                   : default_fixed_model_path();
}

std::string fixed_model_path(const example::Args& args) {
    return args.has("fixed-model") ? args.str("fixed-model", "")
                                   : default_fixed_model_path();
}

void require_sn_template(const example::Args& args) {
    (void)read_file(sn_template_path(args));  // fail before motion
}

void write_outputs(const IdentResult& result, const ExportInfo& info,
                   const example::Args& args, bool external_trajectory) {
    require_sn_template(args);
    const std::string out_dir = args.str("output", "ident_result");
    std::filesystem::create_directories(out_dir);
    const std::string stamp = timestamp_suffix();
    const std::string tpl = sn_template_path(args);
    // The template stem carries the controller serial (SN2026101900005);
    // the serial is redundant in the output name, so keep only SN_<stamp>.
    const std::string out = out_dir + "/SN_" + stamp + ".yaml";
    std::ofstream f(out);
    if (!f) throw InvalidCommandError("cannot write " + out);
    f << "# Generated by litearm-cpp 10_identify at " << stamp << "\n";
    f << "# template: " << tpl << "\n";
    f << "# phi_frame: craig_mdh_link_frames (DH/litearm_dh.urdf)\n";
    f << "# inertia is about the COM in the controller DH link frame; "
         "kinematics/axis are the template's\n";
    const bool fixed_mode = !args.has("full-inertia");
    if (fixed_mode) {
        f << "# inertia: fixed from " << fixed_model_path(args)
          << " (not re-identified); only friction/armature identified\n";
    } else if (use_unit_mass(args)) {
        f << "# mass: unit-mass gauge (m=1), re-identified with the mass fixed; "
             "not a physical-mass rescale\n";
    }
    f << "# serial: " << info.robot_serial;
    if (!info.urdf_path.empty()) f << "  urdf: " << info.urdf_path;
    if (!info.urdf_sha256.empty()) f << "  urdf_sha256: " << info.urdf_sha256;
    f << "\n";
    if (info.n_cycles > 0 || !info.centers.empty()) {
        f << "# capture: n_segments=" << info.centers.size()
          << " cycles=" << info.n_cycles
          << " harmonics=" << info.n_harmonics
          << " period=" << info.period;
        if (!external_trajectory) f << " ee_z_min=" << info.ee_z_min;
        f << "\n";
    }
    if (external_trajectory) {
        f << "# trajectory: external Fourier coefficients, fitted to the joint "
             "limits (center shift + uniform harmonic scale); ee-height/singularity "
             "not enforced\n";
    }
    const bool allow_nonphysical =
        fixed_mode || args.has("allow-nonphysical") || result.info.fixed_inertia;
    f << sn_template_yaml(result, read_file(tpl), allow_nonphysical);
    std::printf("wrote %s\n", out.c_str());
}

/// Write the Python 17 44-column CSV so the captured data can be replayed by
/// the Python SDK.  qd and qm are both the measured q; tau and tau_m are both
/// the firmware's measured torque.
void write_fourier_csv(const std::string& path,
                       const std::vector<std::vector<SamplePoint>>& per_center,
                       int cycles, double period) {
    std::ofstream out(path);
    if (!out) throw InvalidCommandError("cannot write " + path);
    out << "seg,t";
    for (const char* p : {"qd", "qm", "dqd", "ddqd", "tau", "tau_m"}) {
        for (int j = 0; j < kJoints; ++j) out << ',' << p << j;
    }
    out << '\n' << std::setprecision(6);
    for (size_t ci = 0; ci < per_center.size(); ++ci) {
        for (const auto& sp : per_center[ci]) {
            const int cycle = int(sp.t / period);
            const int seg = int(ci) * cycles + cycle;
            const double t_global = double(ci) * double(cycles) * period + sp.t;
            out << seg << ',' << std::fixed << std::setprecision(4) << t_global;
            out << std::setprecision(6);
            auto emit = [&](const Vec7& v) {
                for (double x : v) out << ',' << x;
            };
            emit(sp.q);   // qd
            emit(sp.q);   // qm
            emit(sp.dq);  // dqd
            emit(sp.ddq); // ddqd
            out << std::setprecision(4);
            emit(sp.tau); // tau
            emit(sp.tau); // tau_m
            out << '\n';
            out << std::setprecision(6) << std::defaultfloat;
        }
    }
    std::printf("wrote %s\n", path.c_str());
}

IdentifyOptions make_solve_options(const example::Args& args) {
    IdentifyOptions o;
    o.regularization = args.num("regularization", o.regularization);
    o.friction_ridge = args.num("friction-ridge", o.friction_ridge);
    o.dc_ridge = args.num("dc-ridge", o.dc_ridge);
    o.fix_dc = args.has("fix-dc");
    o.armature_ridge = args.num("armature-ridge", o.armature_ridge);
    o.K_coulomb = args.num("coulomb-k", o.K_coulomb);
    o.physical_links = !args.has("no-physical-links");
    o.diagnostics_max_rows = size_t(args.num("diagnostics-max-rows", 100000));

    const bool full_inertia = args.has("full-inertia");
    if (full_inertia && args.has("fixed-model")) {
        throw InvalidCommandError(
            "--full-inertia and --fixed-model are mutually exclusive");
    }
    o.unit_mass = use_unit_mass(args);
    if (!full_inertia) {
        // Default scheme: keep the inertia bank fixed and identify only the
        // friction/armature columns.
        const bool explicit_model = args.has("fixed-model");
        const std::string path = explicit_model ? args.str("fixed-model", "")
                                                : default_fixed_model_path();
        const IdentResult fixed = load_result_yaml(read_file(path), false);
        o.fix_inertia = true;
        o.fixed_phi = fixed.phi;
        o.unit_mass = false;  // all 70 inertia parameters are fixed
        std::string why;
        if (!physically_valid(fixed.phi, &why)) {
            std::printf("WARNING: fixed model %s is not physically valid (%s); "
                        "inertia will be kept exactly as supplied\n",
                        path.c_str(), why.c_str());
        }
        std::printf("fixed inertia%s: %s (70 parameters)\n",
                    explicit_model ? "" : " (default)", path.c_str());
    }
    return o;
}

int run_offline(const example::Args& args) {
    if (!args.has("input-csv")) {
        throw InvalidCommandError("offline mode requires --input-csv");
    }
    if (args.has("write-model")) {
        throw InvalidCommandError("--write-model requires --load-result or --go");
    }
    require_sn_template(args);  // fail before solving: the result is SN-only
    const auto samples = load_fourier_csv(args.str("input-csv", ""));
    std::printf("loaded %zu samples\n", samples.size());
    const IdentResult result = identify(samples, make_solve_options(args));
    print_result(result);
    const ExportInfo info = make_export_info(
        args, 0, kExternalHarmonics, args.num("period", kDefaultPeriod), 0.15, {});
    write_outputs(result, info, args, false);
    return 0;
}

/// Write a saved SN_*.yaml (or legacy identified_params*.yaml) back to the
/// firmware. Without --write-model this only parses and prints a summary.
int run_load_result(const example::Args& args) {
    const std::string path = args.str("load-result", "");
    const std::string text = read_file(path);
    const bool sn = text.find("robot_group:") != std::string::npos;
    const bool fixed_output = text.find("# inertia: fixed from") != std::string::npos;
    const IdentResult result = load_result_yaml(
        text, !(args.has("allow-nonphysical") || fixed_output));
    if (fixed_output) {
        std::printf("fixed-inertia result detected; loading unchecked inertia\n");
    }
    std::printf("loaded %s (%s)\n", path.c_str(), sn ? "SN config" : "identified_params");
    if (!sn && text.find("phi_frame: craig_mdh") == std::string::npos) {
        // Legacy results (including Python 17 output against the vendor URDF)
        // carry phi in that URDF's link frames.  --write-model sends phi as-is,
        // so loading one here would silently install the wrong frame.
        std::printf("WARNING: %s does not declare phi_frame: craig_mdh_link_frames; "
                    "it may have been identified in vendor-URDF link frames. "
                    "Re-identify against DH/litearm_dh.urdf before --write-model.\n",
                    path.c_str());
    }
    std::printf("mass (kg):");
    for (int j = 0; j < kJoints; ++j) {
        std::printf(" %.4f", result.phi[size_t(j * 10)]);
    }
    std::printf("\nfc0:");
    for (double v : result.fc0()) std::printf(" %+.4f", v);
    std::printf("\nfc1:");
    for (double v : result.fc1()) std::printf(" %+.4f", v);
    std::printf("\njm:");
    for (double v : result.armature) std::printf(" %+.5f", v);
    std::printf("\n");

    if (!args.has("write-model")) {
        std::printf("read-only; add --write-model to write bodies 1..7\n");
        return 0;
    }

    auto arm = example::make_arm(args);
    std::printf("WARNING: writing the firmware model requires a disarmed, supported arm\n");
    WriteModelOptions wopt;
    wopt.write_jm = args.has("write-jm");
    wopt.check_gravity = !args.has("no-gravity-check");
    wopt.rollback_on_gravity_mismatch = args.has("rollback-gravity");
    wopt.allow_nonphysical =
        args.has("allow-nonphysical") || fixed_output || result.info.fixed_inertia;
    const WriteModelReport rep = write_model(*arm, result, wopt);
    std::printf("committed mask=0x%03X readback_err=%.3g gravity_delta=%.3f "
                "rolled_back=%d\n",
                rep.staged_mask, rep.max_body_readback_error,
                rep.max_gravity_delta_nm, int(rep.rolled_back));
    return rep.rolled_back ? 1 : 0;
}

Limits read_limits(Arm& arm) {
    Limits lim = Limits::defaults();
    const auto jps = arm.params().all_joint_params();
    for (int j = 0; j < kJoints && j < int(jps.size()); ++j) {
        lim.q_min[size_t(j)] = jps[size_t(j)].q_min;
        lim.q_max[size_t(j)] = jps[size_t(j)].q_max;
    }
    return lim;
}

/// One captured segment: the fitted trajectory plus the diagnostics printed in
/// the plan.  These are informational only; the fit does not enforce ee height
/// or singularity limits.
struct SegmentPlan {
    FourierTrajectory trajectory;
    Vec7 q0_before{};
    Vec7 q0_after{};
    Vec7 scale{};
    double z_min_before = 0.0;
    double z_min_after = 0.0;
    double sigma_min = 0.0;
};

int run_real(Arm& arm, const example::Args& args) {
    if (arm.n() != kJoints) {
        std::printf("requires a 7-joint arm (n=%d)\n", arm.n());
        return 1;
    }
    std::printf("model frames: Craig/MDH link frames (DH/litearm_dh.urdf)\n");

    const Limits limits = read_limits(arm);
    const double period = args.num("period", kDefaultPeriod);
    if (!(std::isfinite(period) && period > 0.0)) {
        throw InvalidCommandError("--period must be finite and positive");
    }

    // Excitation is always a coefficient file: the shipped default, or an
    // explicit --coeffs/--coeffs-inline override.
    const bool explicit_coeffs_file = args.has("coeffs");
    const bool has_coeffs_inline = args.has("coeffs-inline");
    if (explicit_coeffs_file && has_coeffs_inline) {
        throw InvalidCommandError("--coeffs and --coeffs-inline are mutually exclusive");
    }
    const std::string coeffs_path =
        has_coeffs_inline
            ? std::string()
            : (explicit_coeffs_file ? args.str("coeffs", "") : default_coeffs_path());
    const std::vector<FourierCoefficients> coeffs =
        has_coeffs_inline
            ? parse_fourier_coefficients_inline(args.str("coeffs-inline", ""), period)
            : load_fourier_coefficients(coeffs_path, period);
    if (coeffs.size() > 2) {
        throw InvalidCommandError(
            "external coefficients: at most 2 segments are supported; got " +
            std::to_string(coeffs.size()));
    }

    if (!explicit_coeffs_file && !has_coeffs_inline) {
        std::printf("default coefficients: %s\n", coeffs_path.c_str());
    }
    std::printf("external Fourier coefficients: %zu segment(s), period=%.3f s, "
                "harmonics=%d\n",
                coeffs.size(), period, kExternalHarmonics);
    std::printf("WARNING: external coefficients are fitted to the joint limits "
                "(per-joint center shift + uniform harmonic scale); the file is "
                "not modified, and no ee-height lift or singularity check is "
                "performed\n");

    std::vector<SegmentPlan> plans;
    plans.reserve(coeffs.size());
    for (size_t i = 0; i < coeffs.size(); ++i) {
        const FourierTrajectory raw(coeffs[i]);
        const LimitFit fit = fit_fourier_to_limits(raw, limits);
        std::printf("segment %zu limit fit (margin 0.05 rad):", i + 1);
        bool changed = false;
        for (int j = 0; j < kJoints; ++j) {
            const double sh = fit.center_shift[size_t(j)];
            const double sc = fit.scale[size_t(j)];
            if (std::abs(sh) <= 1e-9 && std::abs(sc - 1.0) <= 1e-9) continue;
            std::printf(" J%d a0%+.3f", j + 1, sh);
            if (std::abs(sc - 1.0) > 1e-9) std::printf(" x%.3f", sc);
            changed = true;
        }
        std::printf("%s\n", changed ? "" : " none");

        SegmentPlan plan;
        plan.trajectory = fit.trajectory;
        plan.q0_before = raw.q0();
        plan.q0_after = fit.trajectory.q0();
        plan.scale = fit.scale;
        plan.z_min_before = ee_z_min(raw);
        plan.z_min_after = ee_z_min(fit.trajectory);
        plan.sigma_min = sigma_min_of_trajectory(fit.trajectory);

        const Vec7 peak = fit.trajectory.peak_excursion();
        std::printf("segment %zu: q0=[", i + 1);
        for (int j = 0; j < kJoints; ++j) {
            std::printf("%s%.3f", j ? ", " : "", plan.q0_after[size_t(j)]);
        }
        std::printf("] peak=[");
        for (int j = 0; j < kJoints; ++j) {
            std::printf("%s%.3f", j ? ", " : "", peak[size_t(j)]);
        }
        std::printf("] z_min=%.3f sigma=%.4f (info only, not enforced)\n",
                    plan.z_min_after, plan.sigma_min);
        plans.push_back(std::move(plan));
    }
    if (!args.go) {
        std::printf("read-only plan; add --go to capture\n");
        return 0;
    }
    require_sn_template(args);  // fail before motion: the result is SN-only

    if (args.has("home-first")) {
        std::printf("home: enable and return to the zero pose (0.10 rad/s)\n");
        arm.enable();
        // The arm may start folded against hard stops; 15 s is not enough for
        // >1.5 rad of travel at the firmware's fixed 0.10 rad/s.  Use 60 s so
        // the arrival check does not time out after the arm has reached zero.
        const RobotState st = arm.home(60.0);
        std::printf("home done: q=[");
        const auto q = st.q();
        for (int j = 0; j < kJoints && j < int(q.size()); ++j) {
            std::printf("%s%.3f", j ? ", " : "", q[size_t(j)]);
        }
        std::printf("]\n");
    }

    CaptureOptions copt;
    copt.cycles = int(args.num("cycles", copt.cycles));
    copt.send_hz = int(args.num("send-hz", copt.send_hz));
    copt.analytic_dq = args.has("analytic-dq");
    copt.use_measured_q = !args.has("reference-q");
    copt.use_measured_dq = !args.has("analytic-dq") && !args.has("reference-dq");
    if (args.has("kp") || args.has("kd")) {
        copt.override_gains = true;
        copt.kp = parse_vec7(args.str("kp", "0,0,0,0,0,0,0"), copt.kp);
        copt.kd = parse_vec7(args.str("kd", "0,0,0,0,0,0,0"), copt.kd);
    }

    std::vector<std::vector<SamplePoint>> per_segment;
    std::vector<SamplePoint> samples;
    for (size_t i = 0; i < plans.size(); ++i) {
        std::printf("capturing segment %zu/%zu ...\n", i + 1, plans.size());
        CaptureResult cr = capture_center(arm, plans[i].trajectory, copt);
        std::printf("  cycles=%d commands=%d status=%d valid=%d dropped=%d "
                    "start_phase=%.2fs max_track=%.3f max_late=%.4f\n",
                    cr.stats.cycles, cr.stats.commands_sent, cr.stats.status_samples,
                    cr.stats.valid_samples, cr.stats.dropped_samples,
                    cr.stats.start_phase_s,
                    cr.stats.max_tracking_error_rad, cr.stats.max_send_lateness_s);
        samples.insert(samples.end(), cr.samples.begin(), cr.samples.end());
        per_segment.push_back(std::move(cr.samples));
    }
    if (samples.empty()) {
        std::printf("no valid samples\n");
        return 1;
    }

    const std::string out_dir = args.str("output", "ident_result");
    std::filesystem::create_directories(out_dir);
    write_fourier_csv(out_dir + "/fourier_log.csv", per_segment, copt.cycles,
                      period);

    const IdentResult result = identify(samples, make_solve_options(args));
    print_result(result);
    std::vector<Vec7> executed;
    for (const auto& p : plans) executed.push_back(p.q0_after);
    const ExportInfo info = make_export_info(
        args, copt.cycles, kExternalHarmonics, period, 0.15, executed);
    write_outputs(result, info, args, true);

    if (args.has("write-model")) {
        std::printf("WARNING: writing the firmware model requires a disarmed, supported arm\n");
        WriteModelOptions wopt;
        wopt.write_jm = args.has("write-jm");
        wopt.check_gravity = !args.has("no-gravity-check");
        wopt.rollback_on_gravity_mismatch = args.has("rollback-gravity");
        wopt.allow_nonphysical = !args.has("full-inertia") ||
                                 args.has("allow-nonphysical") ||
                                 result.info.fixed_inertia;
        const WriteModelReport rep = write_model(arm, result, wopt);
        std::printf("committed mask=0x%03X readback_err=%.3g gravity_delta=%.3f "
                    "rolled_back=%d\n",
                    rep.staged_mask, rep.max_body_readback_error,
                    rep.max_gravity_delta_nm, int(rep.rolled_back));
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string desc =
        "10_identify - 96-parameter dynamics identification\n"
        "  --input-csv FILE        offline solve from a Python 17 CSV (no arm)\n"
        "  --load-result FILE      load SN_*.yaml or identified_params*.yaml (add\n"
        "                          --write-model to write it back without capture)\n"
        "  --output DIR            output directory (default ident_result)\n"
        "  --cycles N              cycles per segment (default 4)\n"
        "  --period S              Fourier period (default 24)\n"
        "  --coeffs FILE           Fourier coefficients instead of the default\n"
        "                          file examples/fourier_coeffs_default.txt: 1-2\n"
        "                          segments of 77 numbers, joint-major\n"
        "                          (a0,a1,b1,...,a5,b5 per joint);\n"
        "                          q=a0+sum a_n*cos(nwt)+b_n*sin(nwt).  Fitted to\n"
        "                          the joint limits per joint (center shift +\n"
        "                          uniform harmonic scale); the file is not\n"
        "                          modified; no ee-height lift or singularity check\n"
        "  --coeffs-inline LIST    same values inline (154 numbers = 2 segments);\n"
        "                          use --coeffs-inline=LIST if the first value is\n"
        "                          negative\n"
        "  --fixed-model FILE      keep all 70 inertia parameters from this file\n"
        "                          (SN config or identified_params) and solve only\n"
        "                          Coulomb/viscous/armature (section 4).  Default:\n"
        "                          examples/SN.yaml\n"
        "  --full-inertia          solve all 96 parameters instead of the default\n"
        "                          fixed-inertia friction-only scheme\n"
        "  --allow-nonphysical     allow loading/writing an inertia model that is\n"
        "                          not physically realizable\n"
        "  --send-hz H             streaming rate (default 300)\n"
        "  --kp LIST / --kd LIST   MIT gains (comma separated, 7 values)\n"
        "  --regularization L      inertia CAD ridge (default 1e-2)\n"
        "  --friction-ridge L      a/fv ridge (default 1e-2)\n"
        "  --dc-ridge L            asymmetry b ridge (default 1e5)\n"
        "  --fix-dc                force b=0\n"
        "  --armature-ridge L      jm ridge (default 1e-2)\n"
        "  --no-physical-links     disable I_com CAD blending\n"
        "  --physical-mass         keep physical link masses (default: unit-mass\n"
        "                          gauge m=1, re-identified like the factory bank)\n"
        "  --sn-template FILE      SN config template; default: the fixed model\n"
        "                          file (examples/SN.yaml)\n"
        "  --urdf FILE             record the model URDF path in the SN header\n"
        "                          (use DH/litearm_dh.urdf for this frame convention)\n"
        "  --urdf-sha256 HEX       record the matching SHA-256 in the SN header\n"
        "  --write-model           write bodies 1..7 to the firmware\n"
        "  --write-jm              also write jm\n"
        "  --rollback-gravity      rollback on gravity mismatch\n"
        "  --home-first            enable + home to the zero pose before capture\n"
        "  --go                    actually move/capture\n";

    example::Args parsed;
    if (!example::parse_args(argc, argv, desc, &parsed)) return 0;
    if (external_coefficients_requested(parsed) &&
        (parsed.has("input-csv") || parsed.has("load-result"))) {
        std::fprintf(stderr,
                     "error: --coeffs/--coeffs-inline apply only to real-arm capture\n");
        return 1;
    }
    if (parsed.has("load-result")) {
        try {
            return run_load_result(parsed);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error: %s\n", e.what());
            return 1;
        }
    }
    if (parsed.has("input-csv")) {
        try {
            return run_offline(parsed);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error: %s\n", e.what());
            return 1;
        }
    }
    return example::run_example(argc, argv, desc, run_real);
}
