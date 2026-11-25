English | [简体中文](README.zh-CN.md)

# Examples

Eleven runnable programs that exercise the SDK against a real arm; read this before running any of them.

**Read-only by default.** Anything that moves requires an explicit `--go`.

```bash
cd ..
./run_example.sh 01_hello
./run_example.sh 02_movej --go
```

| Example | What it does | Needs `--go` |
| --- | --- | --- |
| `01_hello` | Handshake, firmware version, state read (q/dq/tau/flags/mode), current TCP | No (read-only) |
| `02_movej` | Joint motion: `enable` → `movej` → wait for arrival → read back | **Yes** |
| `03_move_p` | Cartesian point-to-point: read TCP → `move_p` → read TCP back | **Yes** |
| `04_ik_tcp` | `get_tcp` + `ik(pose)` self-consistency check | No (read-only) |
| `05_ff_tune` | Dynamics and control-law tuning: `ff_preset` / gs / is / gravity / payload | **Yes** |
| `06_cartesian` | Cartesian line / arc / multi-waypoint (firmware planning) | **Yes** |
| `07_vel_jitter_trace` | Dual-channel per-tick capture: 100 Hz state stream + 300 Hz firmware log | **Yes** |
| `08_zero_g_drag` | Freedrive: set the end payload (the gripper) → `zero_g` → drag by hand → soft-limit check before exit | **Yes** |
| `09_ik_accuracy` | Firmware IK accuracy sweep: FK model self-check vs `get_tcp`, Halton joint-space sampling, PC-side FK re-evaluation, unreachable-pose rejection, `--dump` CSV | No (read-only) |
| `10_identify` | Dynamics identification in the controller's Craig/MDH link frames (`DH/litearm_dh.urdf`): default fixes the 70 inertia parameters from `examples/SN.yaml` and identifies only friction/armature; `--full-inertia` for all 96 parameters. Offline CSV solve (`--input-csv`), default two-segment excitation from `examples/fourier_coeffs_default.txt` over 24 s or another coefficient file (`--coeffs`/`--coeffs-inline`, 77 per segment, fitted to the joint limits with the file left unmodified, no ee/singularity handling), SN export, optional `--write-model` / `--write-jm` / `--write-friction` write-back (requires `LITEARM_BUILD_IDENT=ON`) | **Yes** for capture/write-back |
| `11_identify_payload` | End-effector point-mass payload identification (mass + COM) from static gravity poses, using the example-10 DH-frame model: empty baseline (`--capture-baseline`), differential solve (`--baseline`), absolute solve (`--absolute`), offline CSV replay, optional RAM-only `--apply` (requires `LITEARM_BUILD_IDENT=ON`) | **Yes** for capture/apply |

## Example 11 quick start

`11_identify_payload` moves through static wrist poses. Build with `./build.sh --werror`
(it enables the identification module), then run these commands from the repository root:

```bash
# Read-only plan: connect, print the pose set, torque margins and rank. No motion.
./build/11_identify_payload --port /dev/ttyACM0

# Quick absolute run: the first 3 of the 20 wrist poses, payload installed.
./build/11_identify_payload --port /dev/ttyACM0 --go --absolute \
  --pose-count 3 --out /tmp/payload_test

# Differential run (recommended; remove the payload for the baseline).
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --capture-baseline /tmp/payload_baseline.csv --pose-count 20
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --baseline /tmp/payload_baseline.csv --pose-count 20 --out /tmp/payload_test

# Write the result to the firmware (RAM only; never calls save_params()).
./build/11_identify_payload --port /dev/ttyACM0 --go \
  --baseline /tmp/payload_baseline.csv --pose-count 20 --apply
```

Results: `payload_loaded_<stamp>.csv` and `payload_result_<stamp>.yaml` under `--out`
(default `ident_result`), plus the baseline CSV at the path passed to `--capture-baseline`.

- `--go` is required for motion; without it the example only connects and prints the plan.
- The loaded run zeroes the firmware payload feedforward. If you do not pass `--apply`, the
  pre-run payload is restored when the run finishes.
- Use the same `--pose-count` / `--pose-set` for the baseline and the loaded run.
- The example leaves the arm enabled and holding at the last pose; home or disable it yourself.
- The default speed is 0.10; pass `--speed 0.05` for a slower first pass.
- If the current pose is outside the soft limits, run
  `./build/litearm-cpp --port /dev/ttyACM0 home` first.

## Common flags

All examples accept these three:

```
--port PORT    Serial port (defaults to autodetecting 1d50:606f, or the LITEARM_PORT environment variable)
--go           Actually enable / move / write parameters. Without it the connection is read-only and no torque is applied.
--speed S      Move speed scale 0~1 (default 0.3)
```

Per-example flags (run with `--help`):

- `02_movej` positional arguments: seven target joint angles. Omitted, it defaults to "current position plus a small step on J3".
- `07_vel_jitter_trace`: `--dist` `--dur` `--dir` `--tag` `--ff-mask`
- `08_zero_g_drag`: `--seconds` (time limit, 0 = none) `--payload` (kg) `--com X,Y,Z` (full ee-flange COM, recommended) or `--comz` (z only). Ends on Enter or ^C.
- `09_ik_accuracy`: `--n` (samples) `--seed zero|target|prev` `--margin` (fraction of joint range sampled) `--dump FILE` (per-sample CSV) `--unreach K` (unreachable-pose rejection probes) `--timeout` `--frame ee|j7` `--j6axis 1|-1` (force a model variant instead of the self-check's pick).
- `10_identify`: `--input-csv FILE` (offline solve, no arm), `--cycles N`, `--period S` (default 24; must match the coefficient file's `# period:` declaration), `--send-hz H`, `--kp`/`--kd` lists, `--coeffs FILE` / `--coeffs-inline LIST` (77-number segments, joint-major `a0,a1,b1,...,a5,b5`; fitted to the joint limits per joint by a center shift + uniform harmonic scale, with the input file left unmodified; no ee/singularity handling; default file `examples/fourier_coeffs_default.txt`), `--fixed-model FILE` (default `examples/SN.yaml`; keep its 70 inertia parameters and solve only friction/armature), `--full-inertia` (solve all 96 parameters instead), `--allow-nonphysical` (accept/export/write a non-physical inertia block), `--sn-template FILE` (default: the fixed model file; result written as an SN `dynamics_parameters` config), `--physical-mass` (kg masses instead of the unit-mass gauge m=1 in `--full-inertia` mode), `--urdf`/`--urdf-sha256` (model provenance in the SN header; use `DH/litearm_dh.urdf`), `--write-model`, `--write-jm`, `--write-friction` (0x26 items 9/10/11), `--rollback-gravity`, plus the ridge/regularization flags from `--help`. Requires `-DLITEARM_BUILD_IDENT=ON`.
- `11_identify_payload`: `--capture-baseline FILE` (payload physically removed), `--baseline FILE` (differential; recommended), `--absolute` (no baseline; uses the firmware bank), `--loaded-csv FILE` / `--baseline-csv FILE` (offline replay), `--load-params FILE` (empty-arm model override; default is the firmware model bank, required for offline replay), `--capture-loaded FILE`, `--apply` (RAM only; without it the pre-run payload is restored), `--pose-count N` (3..20, default 20), `--pose-set LIST` (explicit 1-based poses, e.g. `1,10,20`), `--samples N`, `--sample-interval S`, `--speed S`, `--max-pose-error RAD`, `--max-payload-kg KG`, `--max-com-radius M`, `--bootstrap-mass`/`--bootstrap-com`, `--condition-max`, `--huber-iterations`. Requires `-DLITEARM_BUILD_IDENT=ON`.

## Safety conventions

- **Read-only by default.** Without `--go` an example only connects and queries; it sends no motion command at all.
- **Small steps you can back out of.** Examples that move default to a small step such as 1 cm / 0.1 rad.
- **They deliberately do not `disable()` when finished** (02/03/06/07/08). Disabling at those poses lets the arm fall back under its own weight, so staying enabled and holding position is the safe end state. Keep the emergency stop within reach.
- **Freedrive is limp, not parked.** `08_zero_g_drag` puts the arm in zero gravity: it follows gravity instead of holding its pose, so support it before running with `--go`. It also refuses to exit while any joint is outside its soft limit — zero gravity skips the over-limit latch, but exiting immediately re-arms it (see `arm.hpp`:359), and a restart re-locks. Press Enter again (or ^C twice) to override.
- **RAM only.** `05_ff_tune` writes FF parameters to RAM and never calls `save_params()`; power-cycling restores the flash values. Persisting them to flash is a separate, irreversible action you can run manually.

## Migrating from the Python examples

Correspondence:

| Python | C++ |
| --- | --- |
| `arm.get_state().value.q` | `arm.get_state().value->q()` |
| `arm.movej([...], speed=0.3)` | `arm.movej({...}, 0.3)` |
| `arm.move_l(pose, speed=0.5)` | `arm.move_l(pose, 0.5)` |
| `arm.model.get_body(1)` | `arm.model().get_body(1)` |
| `with arm.zero_g():` | `{ auto zg = arm.zero_g(); ... }` |
| `for jp in arm.params.all_joint_params()` | `for (const auto& jp : arm.params().all_joint_params())` |

`07_vel_jitter_trace` is the only example with a **deliberate omission**. The Python version also carries a "directional IK precheck plus an end-point soft-limit margin gate" and a four-leg A/B sequence (`--legs`). Those prechecks need a PC-side model, and this package's stated position is that it **does no PC-side kinematics** — the heavy computation all lives in the firmware — so the C++ version keeps only the "one translation plus dual-channel capture" trunk and leaves the choice of direction to the caller.
