// Optional regression against the Python 17 fourier_log.csv.
//
// Set LITEARM_IDENT_LOG to the CSV path. The test SKIPs when the variable is
// absent so a clean checkout without the Python tree still passes.
//
// The Python 17 reference model uses the vendor URDF link frames while this
// module uses the Craig/MDH frames of DH/litearm_dh.urdf.  Torques, friction
// and armature are frame invariant, so the same log must recover the same
// values up to the vendor URDF's rounded rpy values.
#include "test_framework.hpp"

#include <cstdlib>
#include <string>

#include "litearm/ident/csv.hpp"
#include "litearm/ident/solver.hpp"

using namespace litearm::ident;

TEST(ident_python_log_regression) {
    const char* path = std::getenv("LITEARM_IDENT_LOG");
    if (path == nullptr || *path == '\0') {
        SKIP("set LITEARM_IDENT_LOG to the Python fourier_log.csv");
    }
    const auto samples = load_fourier_csv(path);
    CHECK_EQ(int(samples.size()), 40000);
    IdentifyOptions opt;
    opt.diagnostics_max_rows = 100000;
    const IdentResult r = identify(samples, opt);

    CHECK_EQ(r.info.rank, 69);
    CHECK_EQ(r.info.rank_inertia, 43);
    const double want_rmse[7] = {0.5264, 0.7338, 0.3649, 0.2424, 0.1271, 0.0683, 0.0799};
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(r.info.rmse[size_t(j)], want_rmse[j], 0.02);
    }
    const double want_a[7] = {1.2938, 1.2000, 0.4351, 0.6141, 0.1670, 0.1086, 0.1035};
    for (int j = 0; j < kJoints; ++j) {
        CHECK_NEAR(r.coulomb[size_t(j)], want_a[j], 0.02);
    }
    CHECK_NEAR(r.armature[2], 0.07195, 0.01);
    CHECK_NEAR(r.armature[4], 0.05970, 0.01);
    CHECK(r.armature[0] == 0.0 && r.armature[1] == 0.0);
}
