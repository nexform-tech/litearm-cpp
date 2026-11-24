// Static payload (point mass + COM) identification and capture CSV I/O.
#include "litearm/ident/payload.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace litearm {
namespace ident {
namespace {

using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

bool finite_vec3(const Vec3& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool finite_vec7(const Vec7& v) {
    for (double x : v)
        if (!std::isfinite(x)) return false;
    return true;
}

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    if (n % 2 != 0) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

void validate_options(const PayloadOptions& o) {
    if (o.joints.empty()) {
        throw std::invalid_argument("payload: joints must not be empty");
    }
    std::vector<int> seen;
    seen.reserve(o.joints.size());
    for (int j : o.joints) {
        if (j < 0 || j >= kJoints) {
            throw std::invalid_argument("payload: joint index out of range");
        }
        if (std::find(seen.begin(), seen.end(), j) != seen.end()) {
            throw std::invalid_argument("payload: duplicate joint index");
        }
        seen.push_back(j);
    }
    if (!(o.max_payload_kg > 0.0)) {
        throw std::invalid_argument("payload: max_payload_kg must be positive");
    }
    if (!(o.min_mass_kg > 0.0)) {
        throw std::invalid_argument("payload: min_mass_kg must be positive");
    }
    if (!(o.max_com_radius_m > 0.0)) {
        throw std::invalid_argument("payload: max_com_radius_m must be positive");
    }
    if (!(o.max_condition > 0.0)) {
        throw std::invalid_argument("payload: max_condition must be positive");
    }
    if (!(o.min_row_norm >= 0.0)) {
        throw std::invalid_argument("payload: min_row_norm must be >= 0");
    }
    if (o.huber_iterations < 1) {
        throw std::invalid_argument("payload: huber_iterations must be >= 1");
    }
    if (o.min_poses < 2) {
        throw std::invalid_argument("payload: min_poses must be >= 2");
    }
    if (!finite_vec3(o.gravity) ||
        (o.gravity[0] == 0.0 && o.gravity[1] == 0.0 && o.gravity[2] == 0.0)) {
        throw std::invalid_argument("payload: gravity must be finite and non-zero");
    }
}

void validate_samples(const std::vector<PayloadSample>& loaded,
                      const std::vector<PayloadSample>* baseline,
                      const PayloadOptions& o) {
    if (int(loaded.size()) < o.min_poses) {
        throw std::invalid_argument("payload: not enough loaded poses");
    }
    for (const auto& s : loaded) {
        if (!finite_vec7(s.q) || !finite_vec7(s.tau)) {
            throw std::invalid_argument("payload: loaded sample is not finite");
        }
    }
    if (baseline != nullptr) {
        if (baseline->size() != loaded.size()) {
            throw std::invalid_argument(
                "payload: baseline and loaded pose counts differ");
        }
        for (const auto& s : *baseline) {
            if (!finite_vec7(s.q) || !finite_vec7(s.tau)) {
                throw std::invalid_argument("payload: baseline sample is not finite");
            }
        }
    }
}

struct System {
    MatXd A;
    VecXd b;
    bool differential = false;
};

System build_system(const std::vector<PayloadSample>& loaded,
                    const std::vector<PayloadSample>* baseline,
                    const Phi70& empty_phi, const PayloadOptions& o) {
    const Vec7 zero{};
    const size_t max_rows = loaded.size() * o.joints.size();
    std::vector<double> rows;
    std::vector<double> rhs;
    rows.reserve(max_rows * kPayloadParams);
    rhs.reserve(max_rows);

    for (size_t i = 0; i < loaded.size(); ++i) {
        const Vec7& q_loaded = loaded[i].q;
        const auto Y = payload_gravity_regressor(q_loaded, o.gravity);
        Vec7 target = loaded[i].tau;
        if (baseline != nullptr) {
            const Vec7 g_loaded = rnea(q_loaded, zero, zero, empty_phi, o.gravity);
            const Vec7 g_baseline =
                rnea((*baseline)[i].q, zero, zero, empty_phi, o.gravity);
            for (int j = 0; j < kJoints; ++j) {
                target[size_t(j)] -= (*baseline)[i].tau[size_t(j)] +
                                     g_loaded[size_t(j)] - g_baseline[size_t(j)];
            }
        } else {
            const Vec7 g_loaded = rnea(q_loaded, zero, zero, empty_phi, o.gravity);
            for (int j = 0; j < kJoints; ++j) {
                target[size_t(j)] -= g_loaded[size_t(j)];
            }
        }
        for (int j : o.joints) {
            const size_t off = size_t(j) * kPayloadParams;
            double norm2 = 0.0;
            for (int k = 0; k < kPayloadParams; ++k) {
                const double v = Y[off + size_t(k)];
                norm2 += v * v;
            }
            if (std::sqrt(norm2) <= o.min_row_norm) continue;
            for (int k = 0; k < kPayloadParams; ++k) {
                rows.push_back(Y[off + size_t(k)]);
            }
            rhs.push_back(target[size_t(j)]);
        }
    }

    System sys;
    sys.differential = baseline != nullptr;
    const Eigen::Index m = Eigen::Index(rhs.size());
    if (m < kPayloadParams) {
        throw std::runtime_error("payload: not enough observable torque rows");
    }
    sys.A.resize(m, kPayloadParams);
    sys.b.resize(m);
    for (Eigen::Index r = 0; r < m; ++r) {
        for (int k = 0; k < kPayloadParams; ++k) {
            sys.A(r, k) = rows[size_t(r) * kPayloadParams + size_t(k)];
        }
        sys.b[r] = rhs[size_t(r)];
    }
    return sys;
}

struct RankInfo {
    int rank = 0;
    double condition = std::numeric_limits<double>::infinity();
    double min_column_norm = 0.0;
};

RankInfo rank_of(MatXd A) {
    RankInfo info;
    const Eigen::Index ncols = A.cols();
    if (A.rows() < ncols) return info;
    info.min_column_norm = std::numeric_limits<double>::infinity();
    for (Eigen::Index c = 0; c < ncols; ++c) {
        const double n = A.col(c).norm();
        info.min_column_norm = std::min(info.min_column_norm, n);
        A.col(c) /= std::max(n, 1e-12);
    }
    Eigen::JacobiSVD<MatXd> svd(A);
    const auto sv = svd.singularValues();
    int rank = 0;
    for (Eigen::Index i = 0; i < sv.size(); ++i) {
        if (sv[i] > 1e-9) ++rank;
    }
    info.rank = rank;
    if (rank == ncols && sv.size() == ncols && sv[ncols - 1] > 0.0) {
        info.condition = sv[0] / sv[ncols - 1];
    }
    return info;
}

std::string trim(const std::string& s) {
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const auto b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::istringstream ss(line);
    std::string part;
    while (std::getline(ss, part, ',')) out.push_back(trim(part));
    return out;
}

double to_double(const std::string& s, const char* what) {
    try {
        std::size_t used = 0;
        const double v = std::stod(s, &used);
        if (used != s.size() || !std::isfinite(v)) throw std::invalid_argument("bad");
        return v;
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string("payload capture: bad ") + what);
    }
}

}  // namespace

PayloadPoseDiagnostics payload_pose_diagnostics(
    const std::vector<Vec7>& q, const PayloadOptions& options) {
    validate_options(options);
    PayloadPoseDiagnostics out;
    if (q.empty()) return out;

    std::vector<double> rows;
    for (const auto& pose : q) {
        if (!finite_vec7(pose)) {
            throw std::invalid_argument("payload: pose is not finite");
        }
        const auto Y = payload_gravity_regressor(pose, options.gravity);
        for (int j : options.joints) {
            const size_t off = size_t(j) * kPayloadParams;
            double norm2 = 0.0;
            for (int k = 0; k < kPayloadParams; ++k) norm2 += Y[off + size_t(k)] *
                                                              Y[off + size_t(k)];
            if (std::sqrt(norm2) <= options.min_row_norm) continue;
            for (int k = 0; k < kPayloadParams; ++k) {
                rows.push_back(Y[off + size_t(k)]);
            }
        }
    }
    const Eigen::Index m = Eigen::Index(rows.size() / kPayloadParams);
    if (m == 0) return out;
    MatXd A(m, kPayloadParams);
    for (Eigen::Index r = 0; r < m; ++r) {
        for (int k = 0; k < kPayloadParams; ++k) {
            A(r, k) = rows[size_t(r) * kPayloadParams + size_t(k)];
        }
    }
    const RankInfo info = rank_of(std::move(A));
    out.equations = int(m);
    out.rank = info.rank;
    out.condition = info.condition;
    out.min_column_norm = info.min_column_norm;
    return out;
}

PayloadResult solve_payload(const std::vector<PayloadSample>& loaded,
                            const std::vector<PayloadSample>* baseline,
                            const Phi70* empty_phi,
                            const PayloadOptions& options) {
    if (empty_phi == nullptr) {
        throw std::invalid_argument(
            "payload: empty_phi is required (identified empty-arm model)");
    }
    validate_options(options);
    validate_samples(loaded, baseline, options);

    System sys = build_system(loaded, baseline, *empty_phi, options);
    const Eigen::Index m = sys.A.rows();
    if (m < kPayloadParams) {
        throw std::runtime_error("payload: not enough observable torque rows");
    }

    std::vector<double> scale(kPayloadParams, 0.0);
    for (int c = 0; c < kPayloadParams; ++c) {
        scale[size_t(c)] = sys.A.col(c).norm();
        if (!(scale[size_t(c)] > 1e-9)) {
            throw std::runtime_error(
                "payload: pose set cannot observe all mass/COM parameters");
        }
    }
    MatXd As = sys.A;
    for (int c = 0; c < kPayloadParams; ++c) As.col(c) /= scale[size_t(c)];

    const RankInfo info = rank_of(As);
    if (info.rank < kPayloadParams) {
        throw std::runtime_error(
            "payload: pose set rank " + std::to_string(info.rank) +
            " < 4; add poses with different wrist orientations");
    }
    if (!(info.condition <= options.max_condition)) {
        throw std::runtime_error(
            "payload: pose set is ill-conditioned (condition " +
            std::to_string(info.condition) + " > " +
            std::to_string(options.max_condition) + ")");
    }

    VecXd theta_s = VecXd::Zero(kPayloadParams);
    VecXd weights = VecXd::Ones(m);
    for (int it = 0; it < options.huber_iterations; ++it) {
        const VecXd sqrt_w = weights.array().sqrt();
        MatXd Aw = As;
        Aw.array().colwise() *= sqrt_w.array();
        VecXd bw = sys.b.array() * sqrt_w.array();
        Eigen::JacobiSVD<MatXd> svd(Aw, Eigen::ComputeThinU | Eigen::ComputeThinV);
        theta_s = svd.solve(bw);
        if (!theta_s.allFinite()) {
            throw std::runtime_error("payload: solver produced non-finite parameters");
        }
        const VecXd residual = sys.b - As * theta_s;
        std::vector<double> r(residual.data(), residual.data() + residual.size());
        const double center = median(r);
        for (double& v : r) v = std::abs(v - center);
        const double sigma = 1.4826 * median(r) + 1e-9;
        const double cutoff = 1.5 * sigma;
        for (Eigen::Index i = 0; i < m; ++i) {
            const double a = std::abs(residual[i] - center);
            weights[i] = (a <= cutoff) ? 1.0 : cutoff / a;
        }
    }

    VecXd theta = VecXd::Zero(kPayloadParams);
    for (int c = 0; c < kPayloadParams; ++c) theta[c] = theta_s[c] / scale[size_t(c)];

    PayloadResult result;
    result.mass = theta[0];
    if (!std::isfinite(result.mass) || result.mass <= options.min_mass_kg) {
        throw std::runtime_error(
            "payload: identified mass invalid or too small (mass " +
            std::to_string(result.mass) + " kg)");
    }
    if (result.mass > options.max_payload_kg) {
        throw std::runtime_error(
            "payload: identified mass " + std::to_string(result.mass) +
            " kg exceeds the safety limit " +
            std::to_string(options.max_payload_kg) + " kg");
    }
    result.first_moment = {theta[1], theta[2], theta[3]};
    result.com = {theta[1] / result.mass, theta[2] / result.mass,
                  theta[3] / result.mass};
    if (!finite_vec3(result.com)) {
        throw std::runtime_error("payload: identified COM is not finite");
    }
    const double radius = std::sqrt(result.com[0] * result.com[0] +
                                    result.com[1] * result.com[1] +
                                    result.com[2] * result.com[2]);
    if (radius > options.max_com_radius_m) {
        throw std::runtime_error(
            "payload: identified COM radius " + std::to_string(radius) +
            " m exceeds the safety limit " +
            std::to_string(options.max_com_radius_m) + " m");
    }

    const VecXd residual = sys.b - sys.A * theta;
    double ss_res = 0.0;
    double max_abs = 0.0;
    for (Eigen::Index i = 0; i < m; ++i) {
        ss_res += residual[i] * residual[i];
        max_abs = std::max(max_abs, std::abs(residual[i]));
    }
    const double mean_b = sys.b.mean();
    double ss_tot = 0.0;
    for (Eigen::Index i = 0; i < m; ++i) {
        const double d = sys.b[i] - mean_b;
        ss_tot += d * d;
    }
    result.residual_rms_nm = std::sqrt(ss_res / double(m));
    result.residual_max_nm = max_abs;
    result.r_squared = (ss_tot > 1e-12) ? (1.0 - ss_res / ss_tot) : 0.0;
    result.rank = info.rank;
    result.condition = info.condition;
    result.equations = int(m);
    result.poses = int(loaded.size());
    result.joints = options.joints;
    result.method = sys.differential ? "differential_static" : "absolute_static";
    return result;
}

void write_payload_capture(const std::string& path,
                           const PayloadCapture& capture) {
    if (capture.schema.empty() || capture.kind.empty()) {
        throw std::invalid_argument("payload capture: schema/kind must be set");
    }
    const size_t n = capture.samples.size();
    if (capture.target_q.size() != n || capture.tau_std.size() != n ||
        capture.pose_error_rad.size() != n || capture.max_abs_dq.size() != n) {
        throw std::invalid_argument("payload capture: per-pose vectors differ in size");
    }
    const std::filesystem::path out(path);
    if (out.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(out.parent_path(), ec);
    }
    std::ofstream f(path, std::ios::trunc);
    if (!f) throw std::runtime_error("payload capture: cannot write " + path);

    std::int64_t unix_stamp = capture.created_unix;
    std::string utc = capture.created_utc;
    if (unix_stamp == 0 || utc.empty()) {
        const std::time_t now = std::time(nullptr);
        unix_stamp = std::int64_t(now);
        std::tm tm{};
#ifdef _WIN32
        gmtime_s(&tm, &now);
#else
        gmtime_r(&now, &tm);
#endif
        char buf[32] = {};
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
        utc = buf;
    }

    f << std::setprecision(17);
    f << "# schema: " << capture.schema << "\n";
    f << "# kind: " << capture.kind << "\n";
    f << "# complete: " << (capture.complete ? 1 : 0) << "\n";
    f << "# created_unix: " << unix_stamp << "\n";
    f << "# created_utc: " << utc << "\n";
    f << "# robot_serial: " << capture.robot_serial << "\n";
    f << "# firmware: " << capture.firmware << "\n";
    f << "# gravity: " << capture.gravity[0] << "," << capture.gravity[1] << ","
      << capture.gravity[2] << "\n";
    f << "# bootstrap_mass: " << capture.bootstrap_mass << "\n";
    f << "# bootstrap_com: " << capture.bootstrap_com[0] << ","
      << capture.bootstrap_com[1] << "," << capture.bootstrap_com[2] << "\n";
    f << "# samples_per_pose: " << capture.samples_per_pose << "\n";
    f << "# sample_interval_s: " << capture.sample_interval_s << "\n";
    f << "# speed: " << capture.speed << "\n";

    f << "pose";
    for (int j = 0; j < kJoints; ++j) f << ",q" << j;
    for (int j = 0; j < kJoints; ++j) f << ",target_q" << j;
    for (int j = 0; j < kJoints; ++j) f << ",tau" << j;
    for (int j = 0; j < kJoints; ++j) f << ",tau_std" << j;
    f << ",pose_error_rad,max_abs_dq\n";
    for (size_t i = 0; i < n; ++i) {
        f << i;
        for (double v : capture.samples[i].q) f << "," << v;
        for (double v : capture.target_q[i]) f << "," << v;
        for (double v : capture.samples[i].tau) f << "," << v;
        for (double v : capture.tau_std[i]) f << "," << v;
        f << "," << capture.pose_error_rad[i] << "," << capture.max_abs_dq[i] << "\n";
    }
}

PayloadCapture load_payload_capture(const std::string& path,
                                    const std::string& expected_kind) {
    std::ifstream f(path);
    if (!f) throw std::invalid_argument("payload capture: cannot read " + path);

    PayloadCapture out;
    std::string line;
    bool header_seen = false;
    bool schema_seen = false;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string t = trim(line);
        if (t.empty()) continue;
        if (t[0] == '#') {
            const std::string body = trim(t.substr(1));
            const auto colon = body.find(':');
            if (colon == std::string::npos) continue;
            const std::string key = trim(body.substr(0, colon));
            const std::string value = trim(body.substr(colon + 1));
            if (key == "schema") {
                schema_seen = true;
                out.schema = value;
            } else if (key == "kind") {
                out.kind = value;
            } else if (key == "complete") {
                out.complete = (value == "1" || value == "true");
            } else if (key == "created_unix") {
                try {
                    out.created_unix = std::stoll(value);
                } catch (const std::exception&) {
                    throw std::invalid_argument("payload capture: bad created_unix");
                }
            } else if (key == "created_utc") {
                out.created_utc = value;
            } else if (key == "robot_serial") {
                out.robot_serial = value;
            } else if (key == "firmware") {
                out.firmware = value;
            } else if (key == "gravity") {
                const auto parts = split_csv(value);
                if (parts.size() != 3) {
                    throw std::invalid_argument("payload capture: bad gravity");
                }
                out.gravity = {to_double(parts[0], "gravity"),
                               to_double(parts[1], "gravity"),
                               to_double(parts[2], "gravity")};
            } else if (key == "bootstrap_mass") {
                out.bootstrap_mass = to_double(value, "bootstrap_mass");
            } else if (key == "bootstrap_com") {
                const auto parts = split_csv(value);
                if (parts.size() != 3) {
                    throw std::invalid_argument("payload capture: bad bootstrap_com");
                }
                out.bootstrap_com = {to_double(parts[0], "bootstrap_com"),
                                     to_double(parts[1], "bootstrap_com"),
                                     to_double(parts[2], "bootstrap_com")};
            } else if (key == "samples_per_pose") {
                try {
                    out.samples_per_pose = std::stoi(value);
                } catch (const std::exception&) {
                    throw std::invalid_argument(
                        "payload capture: bad samples_per_pose");
                }
            } else if (key == "sample_interval_s") {
                out.sample_interval_s = to_double(value, "sample_interval_s");
            } else if (key == "speed") {
                out.speed = to_double(value, "speed");
            }
            continue;
        }
        if (t.rfind("pose,", 0) == 0) {
            header_seen = true;
            continue;
        }
        if (!header_seen) {
            throw std::invalid_argument("payload capture: data before CSV header");
        }
        const auto parts = split_csv(t);
        constexpr size_t kExpected = 1 + 7 * 4 + 2;   // pose + q/target/tau/std + 2
        if (parts.size() != kExpected) {
            throw std::invalid_argument("payload capture: wrong column count");
        }
        size_t col = 0;
        const int index = std::stoi(parts[col++]);
        if (index != int(out.samples.size())) {
            throw std::invalid_argument(
                "payload capture: pose indices are not sequential");
        }
        PayloadSample s;
        for (int j = 0; j < kJoints; ++j) s.q[size_t(j)] = to_double(parts[col++], "q");
        Vec7 target{};
        for (int j = 0; j < kJoints; ++j)
            target[size_t(j)] = to_double(parts[col++], "target_q");
        for (int j = 0; j < kJoints; ++j)
            s.tau[size_t(j)] = to_double(parts[col++], "tau");
        Vec7 std{};
        for (int j = 0; j < kJoints; ++j) {
            std[size_t(j)] = to_double(parts[col++], "tau_std");
            if (std[size_t(j)] < 0.0) {
                throw std::invalid_argument("payload capture: negative tau_std");
            }
        }
        const double pose_error = to_double(parts[col++], "pose_error_rad");
        const double max_dq = to_double(parts[col++], "max_abs_dq");
        if (pose_error < 0.0 || max_dq < 0.0) {
            throw std::invalid_argument(
                "payload capture: negative pose_error_rad/max_abs_dq");
        }
        out.samples.push_back(s);
        out.target_q.push_back(target);
        out.tau_std.push_back(std);
        out.pose_error_rad.push_back(pose_error);
        out.max_abs_dq.push_back(max_dq);
    }

    if (!schema_seen) {
        throw std::invalid_argument("payload capture: missing schema");
    }
    if (out.schema != "litearm.payload_capture.v1") {
        throw std::invalid_argument("payload capture: unsupported schema " +
                                    out.schema);
    }
    if (out.kind.empty()) {
        throw std::invalid_argument("payload capture: missing kind");
    }
    if (!expected_kind.empty() && out.kind != expected_kind) {
        throw std::invalid_argument("payload capture: expected kind '" +
                                    expected_kind + "', got '" + out.kind + "'");
    }
    if (!out.complete) {
        throw std::invalid_argument(
            "payload capture: file is incomplete (complete: 0)");
    }
    if (out.samples.size() < 2) {
        throw std::invalid_argument("payload capture: fewer than two poses");
    }
    return out;
}

}  // namespace ident
}  // namespace litearm
