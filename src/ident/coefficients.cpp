// External Fourier coefficient parsing for 10_identify.  See
// include/litearm/ident/coefficients.hpp for the text format.
#include "litearm/ident/coefficients.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace litearm {
namespace ident {
namespace {

void require_period(double period) {
    if (!std::isfinite(period) || period <= 0.0) {
        throw std::invalid_argument(
            "external Fourier coefficients: period must be finite and positive");
    }
}

bool is_separator(char c) {
    return std::isspace(static_cast<unsigned char>(c)) != 0 || c == ',';
}

std::string trim(const std::string& s) {
    size_t begin = 0;
    size_t end = s.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(s[begin])) != 0) ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1])) != 0) --end;
    return s.substr(begin, end - begin);
}

double parse_number(const std::string& token, const std::string& where) {
    char* end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') {
        throw std::invalid_argument(where + ": expected a number, got \"" + token + "\"");
    }
    if (!std::isfinite(value)) {
        throw std::invalid_argument(where + ": \"" + token + "\" is not finite");
    }
    return value;
}

/// A `# period: <seconds>` comment declares the coefficient design.  Returns
/// the declared value, or nullopt when the text carries no declaration.
std::optional<double> declared_period(const std::string& text) {
    std::optional<double> declared;
    std::istringstream in(text);
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        if (line_no == 1 && line.size() >= 3 &&
            static_cast<unsigned char>(line[0]) == 0xEF &&
            static_cast<unsigned char>(line[1]) == 0xBB &&
            static_cast<unsigned char>(line[2]) == 0xBF) {
            line.erase(0, 3);  // UTF-8 BOM
        }
        const size_t hash = line.find('#');
        if (hash == std::string::npos) continue;
        std::string body = trim(line.substr(hash + 1));
        if (body.rfind("period", 0) != 0) continue;
        std::string rest = trim(body.substr(6));
        if (rest.empty() || (rest[0] != ':' && rest[0] != '=')) continue;  // prose
        rest = trim(rest.substr(1));
        const std::string where = "external Fourier coefficients: line " +
                                  std::to_string(line_no) + ": '# period:'";
        if (rest.empty()) throw std::invalid_argument(where + " needs a value");
        char* end = nullptr;
        const double value = std::strtod(rest.c_str(), &end);
        if (end == rest.c_str() || !trim(std::string(end)).empty() ||
            !std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(where + " bad value \"" + rest + "\"");
        }
        if (declared) {
            const double tol =
                1e-9 * std::max(1.0, std::max(std::abs(*declared), std::abs(value)));
            if (std::abs(*declared - value) > tol) {
                throw std::invalid_argument(where +
                                            " conflicts with an earlier declaration");
            }
        }
        declared = value;
    }
    return declared;
}

/// values holds exactly kCoefficientsPerSegment (77) numbers, joint-major:
/// [a0, a1(cos), b1(sin), a2, b2, ..., a5, b5] per joint.
FourierCoefficients make_segment(const std::vector<double>& values, double period) {
    if (static_cast<int>(values.size()) != kCoefficientsPerSegment) {
        throw std::invalid_argument("external Fourier coefficients: internal size mismatch");
    }
    FourierCoefficients c;
    c.period = period;
    c.a.assign(static_cast<size_t>(kExternalHarmonics), {});
    c.b.assign(static_cast<size_t>(kExternalHarmonics), {});
    for (size_t i = 0; i < values.size(); ++i) {
        const int joint = static_cast<int>(i) / kCoefficientsPerJoint;
        const int slot = static_cast<int>(i) % kCoefficientsPerJoint;
        if (slot == 0) {
            c.q0[static_cast<size_t>(joint)] = values[i];  // external a0
            continue;
        }
        const int harmonic = (slot - 1) / 2;  // 0..4 -> n = 1..5
        if ((slot - 1) % 2 == 0) {
            // external a_n multiplies cos(n*w*t); FourierCoefficients::b is COS
            c.b[static_cast<size_t>(harmonic)][static_cast<size_t>(joint)] = values[i];
        } else {
            // external b_n multiplies sin(n*w*t); FourierCoefficients::a is SIN
            c.a[static_cast<size_t>(harmonic)][static_cast<size_t>(joint)] = values[i];
        }
    }
    return c;
}

void append_values(const std::string& text, const std::string& where,
                   std::vector<double>* values) {
    std::string token;
    for (char c : text) {
        if (is_separator(c)) {
            if (!token.empty()) {
                values->push_back(parse_number(token, where));
                token.clear();
            }
        } else {
            token += c;
        }
    }
    if (!token.empty()) values->push_back(parse_number(token, where));
}

}  // namespace

std::vector<FourierCoefficients> parse_fourier_coefficients_text(
    const std::string& text, double period) {
    require_period(period);

    // Refuse a period that disagrees with the file's declared design period:
    // the same coefficients at a different period are a different trajectory.
    if (const auto declared = declared_period(text)) {
        const double tol =
            1e-9 * std::max(1.0, std::max(std::abs(*declared), std::abs(period)));
        if (std::abs(*declared - period) > tol) {
            throw std::invalid_argument(
                "external Fourier coefficients: file declares period=" +
                std::to_string(*declared) + " s but --period is " +
                std::to_string(period) +
                " s; velocities scale as 1/T and accelerations as 1/T^2, so "
                "the trajectory would not match the coefficient design");
        }
    }

    std::vector<FourierCoefficients> out;
    std::vector<double> values;
    int start_line = 0;
    int line_no = 0;

    auto finalize = [&]() {
        if (values.empty()) return;
        if (static_cast<int>(values.size()) != kCoefficientsPerSegment) {
            throw std::invalid_argument(
                "external Fourier coefficients: segment " + std::to_string(out.size() + 1) +
                " (starting line " + std::to_string(start_line) + ") has " +
                std::to_string(values.size()) + " values; expected " +
                std::to_string(kCoefficientsPerSegment) +
                " (7 joints x 11: a0,a1,b1,...,a5,b5)");
        }
        out.push_back(make_segment(values, period));
        values.clear();
        start_line = 0;
    };

    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        ++line_no;
        if (line_no == 1 && line.size() >= 3 &&
            static_cast<unsigned char>(line[0]) == 0xEF &&
            static_cast<unsigned char>(line[1]) == 0xBB &&
            static_cast<unsigned char>(line[2]) == 0xBF) {
            line.erase(0, 3);  // UTF-8 BOM
        }
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        line = trim(line);

        if (line.empty()) {
            // A blank line ends a segment only once it is complete; blank
            // lines inside a 7-row block are harmless.
            if (static_cast<int>(values.size()) == kCoefficientsPerSegment) finalize();
            continue;
        }
        if (line == "---") {
            // A duplicate separator after a completed segment is ignored.
            finalize();
            continue;
        }

        std::string token;
        for (char c : line) {
            if (is_separator(c)) {
                if (!token.empty()) {
                    if (static_cast<int>(values.size()) == kCoefficientsPerSegment) {
                        throw std::invalid_argument(
                            "external Fourier coefficients: line " +
                            std::to_string(line_no) + ": segment " +
                            std::to_string(out.size() + 1) +
                            " already has 77 values; separate segments with a blank "
                            "line or ---");
                    }
                    if (values.empty()) start_line = line_no;
                    values.push_back(parse_number(
                        token, "external Fourier coefficients: line " +
                                   std::to_string(line_no)));
                    token.clear();
                }
            } else {
                token += c;
            }
        }
        if (!token.empty()) {
            if (static_cast<int>(values.size()) == kCoefficientsPerSegment) {
                throw std::invalid_argument(
                    "external Fourier coefficients: line " + std::to_string(line_no) +
                    ": segment " + std::to_string(out.size() + 1) +
                    " already has 77 values; separate segments with a blank line or ---");
            }
            if (values.empty()) start_line = line_no;
            values.push_back(parse_number(
                token,
                "external Fourier coefficients: line " + std::to_string(line_no)));
        }
    }
    finalize();  // EOF: keeps a complete last segment, rejects a partial one
    if (out.empty()) {
        throw std::invalid_argument("external Fourier coefficients: no values found");
    }
    return out;
}

std::vector<FourierCoefficients> parse_fourier_coefficients_inline(
    const std::string& text, double period) {
    require_period(period);

    std::vector<double> values;
    append_values(text, "external Fourier coefficients (inline)", &values);
    if (values.empty()) {
        throw std::invalid_argument("external Fourier coefficients (inline): no values found");
    }
    if (values.size() % static_cast<size_t>(kCoefficientsPerSegment) != 0) {
        throw std::invalid_argument(
            "external Fourier coefficients (inline): expected a positive multiple of " +
            std::to_string(kCoefficientsPerSegment) + " values, got " +
            std::to_string(values.size()));
    }

    std::vector<FourierCoefficients> out;
    const size_t n_segments = values.size() / static_cast<size_t>(kCoefficientsPerSegment);
    out.reserve(n_segments);
    for (size_t s = 0; s < n_segments; ++s) {
        const auto begin = values.begin() + static_cast<std::ptrdiff_t>(
                                                s * kCoefficientsPerSegment);
        out.push_back(make_segment(
            std::vector<double>(begin, begin + kCoefficientsPerSegment), period));
    }
    return out;
}

std::vector<FourierCoefficients> load_fourier_coefficients(
    const std::string& path, double period) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot read " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse_fourier_coefficients_text(ss.str(), period);
}

}  // namespace ident
}  // namespace litearm
