// Offline CSV reader for the Python 17 log format.
#include "litearm/ident/csv.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace litearm {
namespace ident {
namespace {
std::vector<double> split(const std::string& line) {
    std::vector<double> out;
    std::string field;
    std::istringstream ss(line);
    while (std::getline(ss, field, ',')) out.push_back(std::stod(field));
    return out;
}
}  // namespace

std::vector<SamplePoint> load_fourier_csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open CSV: " + path);
    std::string header;
    if (!std::getline(in, header)) throw std::runtime_error("empty CSV: " + path);
    const int cols = int(std::count(header.begin(), header.end(), ',')) + 1;
    const bool has_tau_m = cols >= 2 + 6 * kJoints;
    if (cols < 2 + 5 * kJoints) {
        throw std::runtime_error("CSV has too few columns: " + std::to_string(cols));
    }
    std::vector<SamplePoint> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto v = split(line);
        if (int(v.size()) < cols) continue;
        SamplePoint s;
        s.t = v[1];
        const int qm_off = 2 + kJoints;
        const int dq_off = 2 + 2 * kJoints;
        const int ddq_off = 2 + 3 * kJoints;
        const int tau_off = 2 + 4 * kJoints;
        const int taum_off = 2 + 5 * kJoints;
        const bool measured = has_tau_m && int(v.size()) > taum_off + kJoints - 1;
        for (int j = 0; j < kJoints; ++j) {
            s.q[size_t(j)] = v[size_t((measured ? qm_off : 2) + j)];
            s.dq[size_t(j)] = v[size_t(dq_off + j)];
            s.ddq[size_t(j)] = v[size_t(ddq_off + j)];
            s.tau[size_t(j)] = v[size_t((measured ? taum_off : tau_off) + j)];
        }
        out.push_back(s);
    }
    return out;
}

}  // namespace ident
}  // namespace litearm
