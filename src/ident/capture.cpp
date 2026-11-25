// Real-arm Fourier capture: continuous pure-PD streaming with 100 Hz status
// sampling (q/dq/tau).  The firmware log is not used, so a 24 s period runs
// without the 8 s hold/read pauses that the 2400-tick log would require.
#include "litearm/ident/capture.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "litearm/arm.hpp"
#include "litearm/errors.hpp"
#include "litearm/state.hpp"

namespace litearm {
namespace ident {
namespace {

using Clock = std::chrono::steady_clock;

double now_s() {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

Vec7 to_vec7(const std::vector<double>& v) {
    Vec7 out{};
    for (int j = 0; j < kJoints && j < int(v.size()); ++j) out[size_t(j)] = v[size_t(j)];
    return out;
}

struct CmdRecord {
    double host_t = 0.0;
    double global_t = 0.0;  // time since the start of this capture
    double traj_t = 0.0;    // phase within one Fourier period
    Vec7 q{};
    Vec7 dq{};
};

struct StatusRecord {
    double host_t = 0.0;
    Vec7 q{};
    Vec7 dq{};
    Vec7 tau{};
};

}  // namespace

CaptureResult capture_center(Arm& arm, const FourierTrajectory& trajectory,
                             const CaptureOptions& options) {
    if (arm.n() != kJoints) {
        throw InvalidCommandError("capture_center: requires a 7-joint arm");
    }
    if (options.cycles <= 0 || options.send_hz <= 0) {
        throw InvalidCommandError("capture_center: cycles/send_hz must be positive");
    }

    const auto jps = arm.params().all_joint_params();
    if (int(jps.size()) != kJoints) {
        throw InvalidCommandError("capture_center: could not read all joint params");
    }
    const auto st0 = arm.get_state().value;
    if (!st0 || st0->faulted()) {
        throw InvalidCommandError("capture_center: arm is missing a healthy state");
    }
    const Vec7 q_now = to_vec7(st0->q());
    for (int j = 0; j < kJoints; ++j) {
        if (q_now[size_t(j)] < jps[size_t(j)].q_min - 0.01 ||
            q_now[size_t(j)] > jps[size_t(j)].q_max + 0.01) {
            throw InvalidCommandError(
                "capture_center: current pose is outside the soft limits; "
                "use --home-first or move the arm to a safe pose");
        }
    }
    const double payload = arm.get_ff_scalar(4).value;
    if (std::abs(payload) > 1e-3) {
        throw InvalidCommandError(
            "capture_center: payload mass is non-zero; identification assumes an empty arm");
    }

    // Host model vs firmware TCP self-check.
    if (const auto tcp = arm.get_tcp().value) {
        const FkResult fk = fk_ee(to_vec7(st0->q()));
        const rot::Vec3 fw_p{tcp->at(0), tcp->at(1), tcp->at(2)};
        const rot::Vec3 h_p{fk.p_ee[0], fk.p_ee[1], fk.p_ee[2]};
        const double dp = rot::norm(rot::sub(h_p, fw_p));
        if (dp > 0.005) {
            throw InvalidCommandError(
                "capture_center: host FK does not match firmware TCP (position error " +
                std::to_string(dp) + " m); aborting before motion");
        }
    }

    Vec7 kp = options.kp;
    Vec7 kd = options.kd;
    if (!options.override_gains) {
        for (int j = 0; j < kJoints; ++j) {
            kp[size_t(j)] = jps[size_t(j)].kp;
            kd[size_t(j)] = jps[size_t(j)].kd;
        }
    }

    CaptureResult result;
    CaptureStats& stats = result.stats;
    stats.ff_mask_before = arm.get_ff_mask();

    // Pick a start phase with low joint velocity.  Starting at phase 0 can
    // step the velocity when movej hands over to PD (and again when the last
    // command hands back to movej); the Fourier trajectory is periodic, so a
    // phase shift only changes where the period is cut, not the data set.
    const double period = trajectory.period();
    double phase0 = options.start_phase_s;
    if (phase0 < 0.0) {
        phase0 = 0.0;
        double best = std::numeric_limits<double>::infinity();
        constexpr int kPhaseSamples = 400;
        for (int k = 0; k < kPhaseSamples; ++k) {
            const double t = period * double(k) / double(kPhaseSamples);
            const auto s = trajectory.eval(t);
            double peak = 0.0;
            for (double v : s.dq) peak = std::max(peak, std::abs(v));
            if (peak < best) {
                best = peak;
                phase0 = t;
            }
        }
    }
    phase0 = std::fmod(phase0, period);
    if (phase0 < 0.0) phase0 += period;
    stats.start_phase_s = phase0;

    std::atomic<bool> stop{false};
    std::atomic<bool> fault{false};
    std::mutex status_mu;
    std::vector<StatusRecord> status;
    status.reserve(20000);

    // Poll the 100 Hz status stream.  Unlike the 300 Hz firmware log this has
    // no 8 s capacity limit, so the trajectory can run continuously.
    std::thread sampler([&]() {
        while (!stop.load()) {
            try {
                const auto m = arm.get_state(true, 0.05);
                if (m.value) {
                    StatusRecord r;
                    r.host_t = now_s();
                    r.q = to_vec7(m.value->q());
                    r.dq = to_vec7(m.value->dq());
                    r.tau = to_vec7(m.value->tau());
                    {
                        std::lock_guard<std::mutex> lock(status_mu);
                        status.push_back(r);
                    }
                    if (m.value->faulted()) fault.store(true);
                }
            } catch (...) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    auto cleanup = [&]() {
        stop.store(true);
        if (sampler.joinable()) sampler.join();
        try {
            if (options.restore_ff_mask) arm.set_ff_mask(uint32_t(stats.ff_mask_before));
        } catch (...) {
        }
    };

    try {
        arm.enable();

        const Vec7 start = trajectory.eval(phase0).q;
        arm.movej(std::vector<double>(start.begin(), start.end()), 0.3);
        const auto st = arm.get_state().value;
        if (!st) throw InvalidCommandError("capture_center: no state after movej");
        const Vec7 q_after_move = to_vec7(st->q());
        double max_err = 0.0;
        for (int j = 0; j < kJoints; ++j) {
            max_err = std::max(max_err, std::abs(q_after_move[size_t(j)] - start[size_t(j)]));
        }
        if (max_err > 0.05) {
            throw InvalidCommandError("capture_center: did not reach the trajectory start");
        }
        if (options.settle_s > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(options.settle_s));
        }

        const double total_duration = double(options.cycles) * trajectory.period();
        const int total_cmds =
            std::max(1, int(std::lround(total_duration * double(options.send_hz))));
        const double dt = total_duration / double(total_cmds);
        std::vector<CmdRecord> cmds;
        cmds.reserve(size_t(total_cmds));

        arm.set_ff_mask(0);
        const double t0 = now_s();
        for (int k = 0; k < total_cmds; ++k) {
            if (fault.load()) {
                throw InvalidCommandError("capture_center: joint fault during streaming");
            }
            const double global_t = double(k) * dt;
            const double traj_t = std::fmod(phase0 + global_t, trajectory.period());
            const auto state = trajectory.eval(traj_t);
            for (int j = 0; j < kJoints; ++j) {
                if (!std::isfinite(state.q[size_t(j)]) ||
                    !std::isfinite(state.dq[size_t(j)]) ||
                    !std::isfinite(state.ddq[size_t(j)])) {
                    throw InvalidCommandError(
                        "capture_center: non-finite trajectory point");
                }
                if (state.q[size_t(j)] < jps[size_t(j)].q_min ||
                    state.q[size_t(j)] > jps[size_t(j)].q_max) {
                    throw InvalidCommandError(
                        "capture_center: trajectory exceeds joint limits");
                }
            }
            const double deadline = t0 + global_t;
            arm.send_mit_all(std::vector<double>(state.q.begin(), state.q.end()),
                             std::vector<double>(state.dq.begin(), state.dq.end()),
                             std::vector<double>(kp.begin(), kp.end()),
                             std::vector<double>(kd.begin(), kd.end()),
                             std::vector<double>(kJoints, 0.0));
            const double sent = now_s();
            const double late = sent - deadline;
            stats.max_send_lateness_s = std::max(stats.max_send_lateness_s, late);
            if (late > options.max_send_lateness_s) {
                throw InvalidCommandError(
                    "capture_center: send loop is too late (" +
                    std::to_string(late) + " s)");
            }
            CmdRecord c;
            c.host_t = sent;
            c.global_t = global_t;
            c.traj_t = traj_t;
            c.q = state.q;
            c.dq = state.dq;
            cmds.push_back(c);
            ++stats.commands_sent;
            std::this_thread::sleep_until(
                Clock::time_point(std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(deadline + dt))));
        }

        if (options.restore_ff_mask) {
            arm.set_ff_mask(uint32_t(stats.ff_mask_before));
        }
        const double end_phase =
            std::fmod(phase0 + total_duration, trajectory.period());
        const Vec7 end_q = trajectory.eval(end_phase).q;
        arm.movej(std::vector<double>(end_q.begin(), end_q.end()), options.hold_speed);

        stop.store(true);
        if (sampler.joinable()) sampler.join();

        std::vector<StatusRecord> snapshot;
        {
            std::lock_guard<std::mutex> lock(status_mu);
            snapshot = status;
        }
        stats.status_samples = int(snapshot.size());

        std::vector<SamplePoint> samples;
        samples.reserve(snapshot.size());
        int dropped = 0;
        for (const auto& s : snapshot) {
            if (s.host_t < t0 - 0.02 || s.host_t > t0 + total_duration + 0.05) {
                ++dropped;
                continue;
            }
            auto it = std::lower_bound(
                cmds.begin(), cmds.end(), s.host_t,
                [](const CmdRecord& c, double t) { return c.host_t < t; });
            const CmdRecord* best = nullptr;
            if (it == cmds.begin()) {
                best = &*it;
            } else if (it == cmds.end()) {
                best = &*(it - 1);
            } else {
                const auto& a = *(it - 1);
                const auto& b = *it;
                best = (s.host_t - a.host_t <= b.host_t - s.host_t) ? &a : &b;
            }
            if (best == nullptr) {
                ++dropped;
                continue;
            }
            const auto analytic = trajectory.eval(best->traj_t);
            SamplePoint sp;
            sp.t = best->global_t;
            sp.q = options.use_measured_q ? s.q : best->q;
            sp.dq = options.analytic_dq
                        ? analytic.dq
                        : (options.use_measured_dq ? s.dq : best->dq);
            sp.ddq = analytic.ddq;
            sp.tau = s.tau;

            double te = 0.0;
            for (int j = 0; j < kJoints; ++j) {
                te = std::max(te, std::abs(s.q[size_t(j)] - best->q[size_t(j)]));
            }
            stats.max_tracking_error_rad = std::max(stats.max_tracking_error_rad, te);
            if (te > options.max_tracking_error_rad) fault.store(true);

            samples.push_back(sp);
        }
        stats.valid_samples = int(samples.size());
        stats.dropped_samples = dropped;
        stats.cycles = options.cycles;

        if (fault.load()) {
            throw InvalidCommandError("capture_center: tracking error exceeded limit");
        }
        const int min_valid = options.min_valid_samples_per_cycle * options.cycles;
        if (int(samples.size()) < min_valid) {
            throw InvalidCommandError(
                "capture_center: too few valid status samples (" +
                std::to_string(samples.size()) + " < " + std::to_string(min_valid) + ")");
        }

        cleanup();
        result.samples = std::move(samples);
        return result;
    } catch (...) {
        cleanup();
        throw;
    }
}

}  // namespace ident
}  // namespace litearm
