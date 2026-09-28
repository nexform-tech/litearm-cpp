# Examples

Seven runnable programs that exercise the SDK against a real arm; read this before running any of them.

**Read-only by default.** Anything that moves requires an explicit `--go`.

```bash
cd ..
source env.sh
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

## Safety conventions

- **Read-only by default.** Without `--go` an example only connects and queries; it sends no motion command at all.
- **Small steps you can back out of.** Examples that move default to a small step such as 1 cm / 0.1 rad.
- **They deliberately do not `disable()` when finished** (02/03/06/07). Disabling at those poses lets the arm fall back under its own weight, so staying enabled and holding position is the safe end state. Keep the emergency stop within reach.
- **Irreversible command.** `05_ff_tune` calls `save_params()`, which **writes flash**. Do not run it casually on a calibrated arm.

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
