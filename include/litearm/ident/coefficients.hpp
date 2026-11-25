// External Fourier excitation coefficients for 10_identify.
//
// The input convention is the one used in the identification notes: per joint
// (joint-major), 11 values
//
//   a0  a1  b1  a2  b2  a3  b3  a4  b4  a5  b5
//
// with
//
//   q_j(t) = a0 + sum_{n=1..5} [ a_n*cos(n*w*t) + b_n*sin(n*w*t) ],
//   w = 2*pi/period.
//
// An optional `# period: <seconds>` comment pins the design period.  Parsing
// with a different `period` argument is rejected: velocities scale as 1/T and
// accelerations as 1/T^2, so the recorded trajectory would not match the
// coefficient design.  Files without the comment keep the caller's period
// (backward compatible).
//
// FourierCoefficients stores the sin coefficient in `a` and the cos
// coefficient in `b` (see FourierTrajectory::eval), so a_n maps to
// b[n-1][j] and b_n maps to a[n-1][j].  kCoefficientsPerSegment = 77.
#pragma once

#include <string>
#include <vector>

#include "litearm/ident/trajectory.hpp"

namespace litearm {
namespace ident {

inline constexpr int kExternalHarmonics = 5;
inline constexpr int kCoefficientsPerJoint = 1 + 2 * kExternalHarmonics;  // 11
inline constexpr int kCoefficientsPerSegment = kJoints * kCoefficientsPerJoint;  // 77

/// Parse file-style external Fourier coefficients.
///
/// One segment is exactly 77 numbers in joint-major order.  Numbers may be
/// separated by whitespace and/or commas; `#` starts a comment.  A segment is
/// terminated by a line containing only `---`, by a blank line once the
/// segment already holds 77 values, or by EOF.  Blank lines inside a partially
/// filled segment are ignored.  Accepts any positive number of segments.
///
/// Throws std::invalid_argument on malformed, non-finite, or wrongly sized
/// input, or when `period` is not finite and positive.
std::vector<FourierCoefficients> parse_fourier_coefficients_text(
    const std::string& text, double period);

/// Parse an inline list.  Values are separated by whitespace and/or commas;
/// every consecutive 77 values form one segment.  The total count must be a
/// positive multiple of 77.  Throws std::invalid_argument otherwise.
std::vector<FourierCoefficients> parse_fourier_coefficients_inline(
    const std::string& text, double period);

/// Read `path` and parse it with parse_fourier_coefficients_text().
/// Throws std::runtime_error when the file cannot be opened.
std::vector<FourierCoefficients> load_fourier_coefficients(
    const std::string& path, double period);

}  // namespace ident
}  // namespace litearm
