// External Fourier coefficient parsing tests (10_identify --coeffs).
#include "test_framework.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "litearm/ident/coefficients.hpp"
#include "test_support.hpp"

using namespace litearm::ident;

namespace {

std::string numbers_csv(int count, double base = 1.0) {
    std::ostringstream ss;
    ss.precision(17);
    for (int i = 0; i < count; ++i) {
        if (i) ss << ", ";
        ss << (base + double(i));
    }
    return ss.str();
}

/// Seven rows of eleven comma-separated values, taken from `values`.
std::string rows_7x11(const std::vector<double>& values) {
    std::ostringstream ss;
    ss.precision(17);
    for (int j = 0; j < 7; ++j) {
        for (int k = 0; k < 11; ++k) {
            if (k) ss << ", ";
            ss << values[size_t(j * 11 + k)];
        }
        ss << '\n';
    }
    return ss.str();
}

std::vector<double> sequential_values(double base) {
    std::vector<double> v(77);
    for (int i = 0; i < 77; ++i) v[size_t(i)] = base + double(i);
    return v;
}

}  // namespace

TEST(ident_coefficients_inline_maps_joint_major) {
    const auto segments = parse_fourier_coefficients_inline(numbers_csv(77), 4.0);
    CHECK_EQ(int(segments.size()), 1);
    const FourierCoefficients& c = segments[0];
    CHECK_NEAR(c.period, 4.0, 1e-15);
    CHECK_EQ(int(c.a.size()), kExternalHarmonics);
    CHECK_EQ(int(c.b.size()), kExternalHarmonics);
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(c.q0[size_t(j)], 1.0 + double(11 * j), 1e-12);  // a0
        for (int n = 1; n <= 5; ++n) {
            // external a_n (cos) -> b[n-1]; external b_n (sin) -> a[n-1]
            CHECK_NEAR(c.b[size_t(n - 1)][size_t(j)],
                       1.0 + double(11 * j + 2 * n - 1), 1e-12);
            CHECK_NEAR(c.a[size_t(n - 1)][size_t(j)],
                       1.0 + double(11 * j + 2 * n), 1e-12);
        }
    }
}

TEST(ident_coefficients_inline_two_segments) {
    const auto segments =
        parse_fourier_coefficients_inline(numbers_csv(154), 8.0);
    CHECK_EQ(int(segments.size()), 2);
    CHECK_NEAR(segments[0].q0[0], 1.0, 1e-12);
    CHECK_NEAR(segments[1].q0[0], 78.0, 1e-12);
    CHECK_EQ(int(segments[1].a.size()), kExternalHarmonics);
}

TEST(ident_coefficients_eval_matches_documented_formula) {
    const auto values = sequential_values(0.25);
    const auto segments =
        parse_fourier_coefficients_inline(numbers_csv(77, 0.25), 4.0);
    CHECK_EQ(int(segments.size()), 1);
    const FourierTrajectory traj(segments[0]);
    const double t = 0.37;
    const double w = 2.0 * 3.14159265358979323846 / 4.0;
    const auto s = traj.eval(t);
    for (int j = 0; j < kJoints; ++j) {
        double expect = values[size_t(11 * j)];
        for (int n = 1; n <= 5; ++n) {
            expect += values[size_t(11 * j + 2 * n - 1)] * std::cos(double(n) * w * t);
            expect += values[size_t(11 * j + 2 * n)] * std::sin(double(n) * w * t);
        }
        CHECK_NEAR(s.q[size_t(j)], expect, 1e-12);
    }
}

TEST(ident_coefficients_text_accepts_comments_and_separators) {
    const std::string text = "# header\n" + numbers_csv(77) + "\n---\n" +
                             numbers_csv(77, 100.0) + "\n";
    const auto segments = parse_fourier_coefficients_text(text, 8.0);
    CHECK_EQ(int(segments.size()), 2);
    CHECK_NEAR(segments[0].q0[0], 1.0, 1e-12);
    CHECK_NEAR(segments[1].q0[0], 100.0, 1e-12);
}

TEST(ident_coefficients_text_accepts_blank_line_separator) {
    const std::string text = rows_7x11(sequential_values(1.0)) + "\n" +
                             rows_7x11(sequential_values(100.0));
    const auto segments = parse_fourier_coefficients_text(text, 8.0);
    CHECK_EQ(int(segments.size()), 2);
    CHECK_NEAR(segments[0].q0[2], 23.0, 1e-12);
    CHECK_NEAR(segments[1].q0[2], 122.0, 1e-12);
}

TEST(ident_coefficients_text_7x11_rows_map_rows_to_joints) {
    const std::string text = rows_7x11(sequential_values(1.0)) + "---\n" +
                             rows_7x11(sequential_values(1000.0));
    const auto segments = parse_fourier_coefficients_text(text, 8.0);
    CHECK_EQ(int(segments.size()), 2);
    // row 3, column 1 is J3's a0
    CHECK_NEAR(segments[0].q0[2], 23.0, 1e-12);
    CHECK_NEAR(segments[1].q0[2], 1022.0, 1e-12);
    // row 1, column 2 is J1's cos coefficient a1 -> FourierCoefficients::b
    CHECK_NEAR(segments[0].b[0][0], 2.0, 1e-12);
}

TEST(ident_coefficients_text_blank_lines_inside_block_are_ignored) {
    const std::string rows = rows_7x11(sequential_values(1.0));
    // Split after the third row and insert a blank line; still one segment.
    size_t pos = 0;
    for (int i = 0; i < 3; ++i) pos = rows.find('\n', pos) + 1;
    const std::string text = rows.substr(0, pos) + "\n" + rows.substr(pos);
    const auto segments = parse_fourier_coefficients_text(text, 8.0);
    CHECK_EQ(int(segments.size()), 1);
    CHECK_NEAR(segments[0].q0[6], 67.0, 1e-12);
}

TEST(ident_coefficients_rejects_bad_counts) {
    CHECK_THROWS_AS(parse_fourier_coefficients_inline(numbers_csv(76), 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline(numbers_csv(78), 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline(numbers_csv(153), 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline(numbers_csv(160), 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline("", 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(
        parse_fourier_coefficients_text(numbers_csv(76) + "\n---\n", 8.0),
        std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_text(numbers_csv(78), 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_text("# nothing\n", 8.0),
                    std::invalid_argument);
}

TEST(ident_coefficients_rejects_non_finite_and_garbage) {
    CHECK_THROWS_AS(parse_fourier_coefficients_inline("nan", 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline("inf", 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline("-inf", 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline("1e999", 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline("abc", 8.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline("1.2.3", 8.0),
                    std::invalid_argument);
}

TEST(ident_coefficients_rejects_bad_period) {
    CHECK_THROWS_AS(parse_fourier_coefficients_inline(numbers_csv(77), 0.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_inline(numbers_csv(77), -1.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(
        parse_fourier_coefficients_inline(
            numbers_csv(77), std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}

TEST(ident_coefficients_loads_file) {
    const auto path = std::filesystem::temp_directory_path() /
                      "litearm_ident_coefficients_test.txt";
    {
        std::ofstream out(path);
        out << rows_7x11(sequential_values(1.0)) << "---\n"
            << rows_7x11(sequential_values(100.0));
    }
    const auto segments = load_fourier_coefficients(path.string(), 8.0);
    std::filesystem::remove(path);
    CHECK_EQ(int(segments.size()), 2);
    CHECK_NEAR(segments[0].q0[0], 1.0, 1e-12);
    CHECK_NEAR(segments[1].q0[0], 100.0, 1e-12);
}

TEST(ident_coefficients_missing_file) {
    CHECK_THROWS_AS(
        load_fourier_coefficients("/nonexistent/litearm_coeffs.txt", 8.0),
        std::runtime_error);
}

TEST(ident_coefficients_default_file_parses) {
    const std::string text = lt::read_repo_file("examples/fourier_coeffs_default.txt");
    CHECK(!text.empty());
    const auto segments = parse_fourier_coefficients_text(text, 24.0);
    CHECK_EQ(int(segments.size()), 2);
    CHECK_NEAR(segments[0].q0[0], 0.065969371871808, 1e-15);
    CHECK_NEAR(segments[1].q0[0], 0.001636544390708, 1e-15);
    CHECK_NEAR(segments[0].period, 24.0, 1e-15);
    CHECK_NEAR(segments[1].period, 24.0, 1e-15);
    CHECK_EQ(int(FourierTrajectory(segments[0]).harmonics()), 5);
    CHECK_EQ(int(FourierTrajectory(segments[1]).harmonics()), 5);
}

TEST(ident_coefficients_declared_period_pins_design) {
    const std::string text = "# period: 24\n" + rows_7x11(sequential_values(1.0));
    CHECK_EQ(int(parse_fourier_coefficients_text(text, 24.0).size()), 1);
    CHECK_THROWS_AS(parse_fourier_coefficients_text(text, 48.0),
                    std::invalid_argument);
}

TEST(ident_coefficients_declared_period_rejects_bad_value) {
    const std::string rows = rows_7x11(sequential_values(1.0));
    CHECK_THROWS_AS(parse_fourier_coefficients_text("# period: nope\n" + rows, 24.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(parse_fourier_coefficients_text("# period: 0\n" + rows, 24.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(
        parse_fourier_coefficients_text("# period: 24\n# period: 8\n" + rows, 24.0),
        std::invalid_argument);
}

TEST(ident_coefficients_default_file_period_is_pinned) {
    const std::string text = lt::read_repo_file("examples/fourier_coeffs_default.txt");
    CHECK(!text.empty());
    CHECK_THROWS_AS(parse_fourier_coefficients_text(text, 48.0),
                    std::invalid_argument);
}
