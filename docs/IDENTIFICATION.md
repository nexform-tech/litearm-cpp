# Dynamics identification (optional)

This module ports the Python SDK's 96-parameter Fourier identification into
`litearm-cpp`. It is **off by default**; the core SDK remains dependency-free.

## Build

```bash
cmake -S . -B build-ident -DCMAKE_BUILD_TYPE=Release \
  -DLITEARM_BUILD_IDENT=ON -DLITEARM_BUILD_TESTS=ON -DLITEARM_BUILD_EXAMPLES=ON
cmake --build build-ident -j"$(nproc)"
ctest --test-dir build-ident -R 'test_ident_' --output-on-failure
```

The optional target needs Eigen3 (header-only). The public headers under
`include/litearm/ident/` only expose `std` types; Eigen is confined to
`src/ident/*.cpp`.

## Model

The identified parameter vector has 96 entries:

| Slice | Meaning |
| --- | --- |
| `[0,70)` | 10 inertial parameters per link: `m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz` (inertia about the joint origin) |
| `[70,77)` | symmetric Coulomb `a` |
| `[77,84)` | viscous `fv` |
| `[84,91)` | asymmetry `b` |
| `[91,96)` | armature `jm3..jm7` |

`fc0 = a + b` and `fc1 = -a + b`. `jm1/jm2` are structurally collinear with
link1/link2 `Izz` and are fixed to 0. `fv2/fv3` are HYY reserved fields and stay
0. The inertia block uses the Pinocchio `toDynamicParameters` convention; the
firmware model's `set_body` fields are about the COM, so `phi_to_body10` performs
the parallel-axis conversion.

**Default fixed-inertia scheme.** `10_identify` normally fixes all 70 inertia
parameters from `examples/SN.yaml` and identifies only the friction/armature
block; see "Fixed inertia" below. Use `--full-inertia` to solve the full 96
parameters instead.

**Unit-mass gauge (`--full-inertia`).** The controller's factory bank writes
every arm-link mass as 1.0 (the vendor SN template does the same). `10_identify`
therefore identifies in that gauge: each mass column is fixed at 1 and the
remaining inertial parameters (`m*c` and the inertia about the joint origin)
are re-fitted. This is a constrained re-identification, **not** a rescale of a
physical-mass solution — rescaling alone changes the acceleration regressor's
mass term by `1/m` and destroys the torque fit (J2 RMSE grows from ~0.6 Nm to
~9 Nm on the reference capture). The unit-mass fit keeps the same RMSE as the
physical-mass model on the real data. Pass `--physical-mass` to identify and
export the kg-mass model instead.

The host model uses the **controller's Craig/MDH link frames** rebuilt in
`DH/litearm_dh.urdf` — not the link frames of the vendor
`assets/litearm/urdf/litearm.urdf`. Joints 4/6/7 rotate about -Z
(`axis = s_i * z`), and the fixed `ee_link` is merged into link 7. Consequently
`phi_flat` is expressed directly in the DH link frames and can be compared
with `DH/dh_params.yaml` without any per-link conversion; the same `phi` is
what `phi_to_body10` / `--write-model` send to the firmware. `cad_phi()` is the
vendor CAD inertia re-expressed in those DH frames, and is used only as the
solver prior. The 70-column regressor is built analytically from the 10 basis
parameters; `rnea()` is an independent recursive Newton-Euler implementation
used for validation and gravity checks.

To have the Python SDK consume a C++ result, point its
`model.urdf_path` at `DH/litearm_dh.urdf` as described in `DH/README.md`
section 0, then record that file with `--urdf` / `--urdf-sha256` so the YAML
binds to the same model.

## Physical validity

In `--full-inertia` mode, `identify()` projects every link onto physically
realizable rigid-body parameters before returning:

- all ten values finite, `m > 1e-4 kg`, `|c| <= 1 m`;
- `I_com` positive definite;
- principal moments satisfy the triangle inequality
  (`lambda_max <= lambda_mid + lambda_min`).

The default fixed-inertia mode skips this projection and keeps the supplied
bank unchanged, including non-physical values.

A link whose mass or COM is unusable (NaN/Inf, too small, or outside the COM
radius) is replaced by its CAD link. For the remaining links only `I_com` is
adjusted — mass and first moment (`m`, `m*c`) are preserved, so gravity
torques do not change. This is required because `jm1/jm2` are structurally
collinear with link1/link2 `Izz` and are folded into it; the identified
tensor can therefore look like a thin rod and violate the triangle inequality.
`identified_yaml`, `sn_template_yaml`, and `write_model` re-check the result
and refuse non-finite or non-physical parameters. The fixed-inertia path is the
explicit exception: it keeps the supplied bank, marks the output header with
`# inertia: fixed from ...`, and permits export/write-back of that bank.
`write_model` performs the check before `disable()` in full-inertia mode, so a
bad file never drops a holding arm.

Physical validity is a host-side condition. The firmware's `model.commit`
may impose additional range checks; see `UNVERIFIED_REGISTER.md`.

## Offline use (no arm)

```bash
./build-ident/10_identify \
  --input-csv /path/to/fourier_log.csv \
  --output ident_result
```

This accepts the Python 17 44-column CSV (legacy 37-column files are also
accepted). The result is written only as `SN_<YYYYMMDD_HHMMSS>.yaml`, the
controller's SN config format: the 16 `dynamics_parameters` values of every
joint are replaced and the template's kinematics/axis/limits are kept. The
template defaults to `examples/SN.yaml` (the fixed model); its serial suffix is
not repeated in the output name. Each run appends a fresh timestamped file, so
successive runs form a time series instead of overwriting the previous result.
This path uses measured position (`qm`) and measured torque (`tau_m`) when
present. In `--full-inertia` mode the masses are written as 1.0 (unit-mass
gauge); add `--physical-mass` for kg masses.

The file starts with a comment header recording the template, the DH frame
convention, the optional `--urdf`/`--urdf-sha256` provenance, and the capture
parameters. The older `identified_params_*.yaml` schema is no longer written
(examples 10 and 11 can still read it with `--load-result`/`--load-params`).

On the Python reference log the C++ solver reproduces the Python result:
`rank=69/96`, `rank_inertia=43/70`, per-joint RMSE
`0.5264 / 0.7338 / 0.3649 / 0.2424 / 0.1271 / 0.0683 / 0.0799 Nm`.

## Real-arm capture

Excitation is always a coefficient file. By default it is the shipped
two-segment `examples/fourier_coeffs_default.txt` (77 coefficients per segment,
5 harmonics), captured over a **24 s** period. The path is resolved from the
source tree so the binary also works when run from the build directory;
`--coeffs FILE` overrides the file.

Capture is pure PD and continuous: the FF mask is cleared, `send_mit_all`
streams the Fourier `q/dq` at 300 Hz, and the 100 Hz status stream supplies
measured `q/dq/tau` for the whole trajectory. `ddq` is analytic from the
trajectory. The firmware log is not used, so a 24 s period runs without the
8 s capacity holds; each segment yields roughly 2400 samples per cycle.
The capture starts at the phase with the smallest max `|dq|` (not necessarily
phase 0) and waits 0.15 s after `movej` reaches that pose, so the handoff from
`movej` to PD and the final handoff back to `movej` do not step the velocity.
Capture aborts if the send deadline slips, a joint faults, tracking error
exceeds the limit, or fewer than `1800 × cycles` valid status samples are
recovered.

```bash
# Read-only plan (connects, reads limits, does not move)
./build-ident/10_identify

# Real capture of the default coefficient file; keep the stop within reach
./build-ident/10_identify --go --cycles 1
./build-ident/10_identify --go --cycles 4 --output ident_result
```

The command loop aborts on a send-deadline slip, joint fault, or excessive
tracking error. The arm is left enabled and holding at the end.

`--home-first` uses a 60 s home window. The default `Arm::home()` timeout (15 s)
is too short when the arm starts folded against the hard stops: the firmware
homes at a fixed 0.10 rad/s, so more than 1.5 rad of travel can exceed 15 s even
though the arm reaches zero. If a home call reports a timeout, read the state
before retrying; the arm may already be at zero.

## Fourier coefficient files

The default excitation is `examples/fourier_coeffs_default.txt`; `--coeffs
FILE` (or `--coeffs-inline LIST`) supplies a different file directly. Each
segment has **77 numbers = 7 joints × 11 coefficients**, joint-major:

```
J1: a0 a1 b1 a2 b2 a3 b3 a4 b4 a5 b5
...
J7: a0 a1 b1 a2 b2 a3 b3 a4 b4 a5 b5
```

with

```
q_j(t) = a0 + sum_{n=1..5} [ a_n*cos(n*w*t) + b_n*sin(n*w*t) ]
w = 2*pi / period
```

`FourierCoefficients` stores the sin coefficient in `a` and the cos
coefficient in `b`, so external `a_n` maps to `b[n-1][j]` and external `b_n`
to `a[n-1][j]`.

File rules:

- numbers separated by whitespace and/or commas; `#` starts a comment; a
  UTF-8 BOM is ignored;
- exactly 77 values per segment; segments are separated by a line containing
  only `---`, or by a blank line once the current segment already holds 77
  values; blank lines inside a partially filled segment are ignored;
- rows are joints and columns are coefficients. An 11×7 layout also contains
  77 numbers but means something else;
- one or two segments are supported. They are captured sequentially
  (`capture_center` moves to each segment's `t=0` start) and the samples are
  merged into a single `identify()` solve; `--cycles` applies to each segment.

**Limit fitting.** The loaded trajectory is fitted to the joint position limits
without editing the coefficient file: per joint, the center (`a0`) is moved
only as far as needed, and when the excursion is still wider than the allowed
range, all harmonics of that joint are scaled by one common factor. The fit
uses the firmware's `q_min`/`q_max` and keeps a 0.05 rad margin, so it is also
inside the slightly wider URDF limits; the applied shift/scale is printed per
segment. There is no ee-height lift and no `sigma_min` rejection; the plan
prints `z_min` and `sigma` for information only. The SN header records the
fitted trajectory. `--period` must match the period used to design the numbers
(velocity and acceleration scale as `1/T` and `1/T²`; the shipped file is
designed for the 24 s default). `capture_center` aborts on non-finite
commands, commands outside the firmware joint limits, or excessive tracking
error — that is the runtime hardware guard, not trajectory design.

```bash
# Read-only plan for the default two-segment file
./build-ident/10_identify

# Capture both default segments
./build-ident/10_identify --go --cycles 1 \
  --sn-template /path/to/SN<serial>.yaml

# A different coefficient file
./build-ident/10_identify --go --cycles 1 \
  --coeffs /path/to/other_coeffs.txt \
  --sn-template /path/to/SN<serial>.yaml

# Inline (154 numbers = 2 segments); use =LIST if the first value is negative
./build-ident/10_identify --go --cycles 1 \
  --coeffs-inline='0.1, 0.2, ...' \
  --sn-template /path/to/SN<serial>.yaml
```

`--coeffs` and `--coeffs-inline` are mutually exclusive and apply only to
real-arm capture; combining them with `--input-csv` or `--load-result` is an
error.

## Fixed inertia: friction/armature-only re-identification

`--fixed-model FILE` loads an SN config or `identified_params` file, fixes all
**70 inertia parameters** to the values in that file, and solves only the
friction/armature block:

- symmetric Coulomb `a` (7)
- viscous `fv` (7)
- asymmetry `b` (7)
- armature `jm3..jm7` (5)

This is the default scheme: `--fixed-model` defaults to
`examples/SN.yaml`, and `--sn-template` defaults to the same file. Pass
`--full-inertia` to solve all 96 parameters instead, or `--fixed-model FILE`
to use another inertia bank.

The fixed inertia is kept exactly as supplied, even if it is not physically
valid (e.g. a controller bank with `m=1` and negative COM inertias); the run
prints a warning. `rank_fit` is then the rank of the 26 active columns, while
`rank` still describes the full 96-column regressor. `fv2/fv3` remain 0, and
the usual non-negativity/`|b|<=a` projections still apply to the identified
friction; in particular a solve that wants negative `jm` is clamped to 0.

```bash
# Default scheme: fixed examples/SN.yaml, friction/armature only
./build-ident/10_identify --input-csv ident_result/fourier_log.csv

# Another fixed bank
./build-ident/10_identify --input-csv ident_result/fourier_log.csv \
  --fixed-model path/to/SN.yaml --sn-template path/to/SN.yaml
```

Non-physical inertia is normally rejected by `load_result_yaml` and
`sn_template_yaml`; the default/fixed-model path opts into the unchecked path
for the run, and the exported SN header records `# inertia: fixed from ...`.
When re-loading such an output with `--load-result`, that header is detected
automatically; `--allow-nonphysical` is still available for other
non-physical files.

## Model write-back

`--write-model` is explicit and requires a supported arm (disarmed during the
commit). It stages bodies 1..7 (`set_body`), optionally `jm` (`--write-jm`),
commits `0x0FE` or `0x2FE`, reads the model back, and optionally compares
`get_gravity(q)` against the host RNEA. `save_params()` is **never** called, so
a reboot restores the flash model. Support the arm before committing.

`--load-result` writes `phi` to the firmware exactly as stored. Only load files
identified against the DH link frames (an SN config exported by this tool, or a
legacy `identified_params` file declaring `phi_frame: craig_mdh_link_frames`,
or a Python result produced with `model.urdf_path` set to
`DH/litearm_dh.urdf`). Results produced against the vendor URDF describe the
same bodies in different per-link frames; loading them would silently install
the wrong dynamics. The example prints a warning when a legacy file does not
declare the DH frame. Fixed-inertia outputs carry the
`# inertia: fixed from ...` header and are loaded/written without the physical
check; other non-physical files require `--allow-nonphysical`.

```bash
./build-ident/10_identify --go --cycles 4 --write-model
./build-ident/10_identify --go --cycles 4 --write-model --write-jm
```

To write an existing result without capturing again, add `--load-result`.
Without `--write-model` it only parses the file and prints the mass/friction
summary:

```bash
./build-ident/10_identify --load-result ident_result/SN_20261117_143620.yaml
./build-ident/10_identify --load-result ident_result/SN_20261117_143620.yaml --write-model
```

For an SN file the loader reads the seven `dynamics_parameters` blocks and
converts body10 (about the COM) back to the joint-origin `phi` used by
`write_model`; `fc0`/`fc1` recover the symmetric Coulomb term and DC offset.
Legacy `identified_params` files are still accepted and read their
`phi_flat`, `friction`, and `armature` blocks. Write-back is not saved to
flash: a reboot restores the previous model.

## Payload identification (example 11)

`11_identify_payload` identifies an end-effector point mass and COM in the
EE flange frame. The firmware's payload model is exactly that: `set_payload`
takes a mass and a three-vector COM (`ff_scalar` items 4/5); it has no payload
inertia. **Live runs use the firmware's own dynamics bank as the empty-arm
model**: `arm.model().get_body(1..7)` is read (bank only, no payload
composition, in the Craig/MDH link frames) and converted to the `phi` used by
RNEA. This is the controller's authoritative model and needs no example-10
file. `--load-params FILE` overrides it with an example-10 result (SN config,
or a legacy `identified_params` file declaring `phi_frame: craig_mdh_link_frames`);
offline CSV replay has no arm, so it still requires `--load-params`. The result
records `model_source` (firmware bank or the file path) and
`com_frame: ee_flange (DH/litearm_dh.urdf)` together with the mass and COM.

The payload gravity torque is linear in `theta = [m, m*cx, m*cy, m*cz]`:

```
Y_j,0    = -(a_j x (p_ee - p_j)) . g
Y_j,1+k  = -(a_j x (R_ee e_k)) . g       k = 0,1,2
tau      = Y(q) theta
```

The negative sign matches `rnea()`'s holding-torque convention (base
acceleration `-g`). `p_ee`/`R_ee` are the `ee_link` frame (`kDhEe`), not link 7.
With vertical gravity, J1 (a vertical yaw axis) has exactly zero payload
sensitivity, so J2..J7 are used by default; `--joints` overrides this for a
tilted mount. A single pose has rank 3 (the gravity-direction component of the
first moment is unobservable); the 10-pose family gives rank 4 and a scaled
condition around 3.4.

Recommended differential workflow (same-session empty baseline cancels the
empty-model error and static-friction bias):

```bash
# 1. payload removed
./build-ident/11_identify_payload --go --capture-baseline payload_baseline.csv
# 2. payload installed (empty-arm model = the firmware bank)
./build-ident/11_identify_payload --go --baseline payload_baseline.csv
```

The baseline is transported to the loaded poses with
`rnea(q_loaded) - rnea(q_baseline)` when the settled poses differ slightly;
the RNEA model is the firmware bank read at start-up (or `--load-params`).
Re-capture the baseline close to the loaded run; a baseline older than 24 h
prints a warning. Absolute mode uses the empty-arm gravity (firmware bank by
default) and is more sensitive to model error and stiction:

```bash
./build-ident/11_identify_payload --go --absolute
```

Capture uses 5 static wrist poses: J1 and J3 stay at zero, J2 is fixed at 0.9,
J4 at -0.8, and only J5..J7 vary (20 representative wrist corners/edges/centre;
rank 4, condition 4.33, EE z 0.25–0.40 m). The fit uses J5..J7 only, avoiding
the front axes whose model error is larger. Each pose is sampled at 80 samples
at 12 ms, `|dq| <= 0.03 rad/s`, with a fixed lead-in pose and a speed scale of
0.10 (hard limit 0.20). Before a loaded capture the example zeros the firmware
payload feedforward (`set_payload(0,0,0)`), because an active payload biases the
measured torques; write the identified value back afterwards.
`--pose-count 3` or `--pose-set ...` still gives rank 4 for a quick check, but
fewer poses leave fewer equations for the same residual — use all 20 for the
final identification. Every pose is checkpointed
to CSV with `complete: 0`; the final row set is `complete: 1`, so an interrupted
capture is rejected instead of silently solving a partial series. The loaded
capture is always written to `<out>/payload_loaded_<stamp>.csv` (or
`--capture-loaded FILE`), which makes every live solve replayable without the
arm:

```bash
./build-ident/11_identify_payload \
  --loaded-csv payload_loaded_<stamp>.csv \
  --baseline-csv payload_baseline.csv \
  --load-params ident_result/SN_<stamp>.yaml
```

`--apply` calls `set_payload(mass, com)` and reads the values back; the
firmware silently clamps out-of-range values. It writes RAM only and never
calls `save_params()`, so a reboot restores the previous payload.

## Limitations

- The SN/HYY template uses a line-oriented patcher; it supports block-style
  templates with exactly seven `dynamics_parameters` blocks and all 16 keys.
- Asymmetric Coulomb (`b`) is weakly identifiable on the current Fourier data;
  the solver shrinks it with `--dc-ridge` (default `1e5`). `--fix-dc` forces
  `b=0`. A real asymmetry estimate needs a low-speed/zero-crossing excitation.
- Static gravity poses for **payload** identification are implemented by
  example 11; a full 96-parameter static-gravity identification is still not
  implemented.
- Payload identification is limited to the firmware's point-mass model: mass
  and COM, no payload inertia and no payload friction. A full inertial payload
  would need a model write, not `set_payload`.
- Hardware capture, baseline capture, and `--apply` are **not** verified by
  the offline suite. See `UNVERIFIED_REGISTER.md` for the current status.
