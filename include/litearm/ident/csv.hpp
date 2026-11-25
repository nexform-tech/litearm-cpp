// Offline CSV input for cross-checking against the Python 17 log format.
#pragma once

#include <string>
#include <vector>

#include "litearm/ident/model.hpp"

namespace litearm {
namespace ident {

/// Load a Python 17 fourier_log.csv (44 columns; the legacy 37-column form is
/// also accepted, but tau_m is then unavailable).  Returns samples using the
/// measured torque (tau_m) and measured position (qm) when available.
std::vector<SamplePoint> load_fourier_csv(const std::string& path);

}  // namespace ident
}  // namespace litearm
