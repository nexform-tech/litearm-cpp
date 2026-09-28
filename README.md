# litearm-cpp

A **C++ SDK** for the LiteArm robotic arm — it talks **directly to the firmware over a USB
serial link**. No server, no middleware. Trajectory planning, kinematics and dynamics all live
in the firmware; the host only sends waypoints and reads results back.

This repository is the C++ port of
[litearm-python](https://github.com/nexform-tech/litearm-python) 2.1.0: the same wire
protocol, the same semantics, the same safety criteria, ported line by line. The test suite
is likewise fully offline.

> Full API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md);
> field troubleshooting (symptom → cause → fix): [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## Highlights

- **Zero external dependencies**: only the C++17 standard library and the platform's own
  serial API (POSIX termios / Win32). No libserialport, no numpy — the heavy computation
  already lives in the firmware.
- **One location**: headers in `include/litearm/`, a single static library `liblitearm.a`.
- **Direct USB**: one cable to the firmware; VID:PID `1d50:606f` is discovered automatically.
- **The firmware does the heavy lifting**: planning, kinematics and dynamics all live there;
  the host just sends waypoints and judges arrival.
- **Complete motion API**: joint motion, Cartesian lines/arcs/multi-waypoint paths, freedrive.
- **Safety built in**: forked children fail closed, an independent emergency-stop path, and
  every irreversible command is called out.

## Building

| Item | Requirement |
| --- | --- |
| Compiler | C++17 (g++ 7+ / clang 6+ / MSVC 2019+) |
| Build | CMake ≥ 3.14 |
| Dependencies | none (termios/poll/flock on POSIX, the Win32 serial API on Windows) |
| Firmware | `Litearm1.5.0` or newer |
| Connection | USB CDC serial, VID:PID `1d50:606f` |

```bash
./build.sh                 # configure + build + run tests (fully offline)
source env.sh              # put build/ on PATH (builds on first use)
./run_example.sh 01_hello
```

Manual build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CMake options: `-DLITEARM_BUILD_EXAMPLES=OFF` / `-DLITEARM_BUILD_TESTS=OFF` /
`-DLITEARM_WERROR=ON`.

On Linux you need serial port permission:

```bash
sudo usermod -aG dialout $USER      # takes effect after you log in again
```

To consume it as a subdirectory:

```cmake
add_subdirectory(third_party/litearm-cpp)
target_link_libraries(your_app PRIVATE litearm::litearm)
```

### Installing and packaging

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j && cmake --install build
```

What gets installed (all three consumption styles have been verified — see "How this was
verified" below):

```
include/litearm/*.hpp              # every public header (including testing.hpp / framing.hpp)
lib/liblitearm.a                   # or .so (add -DBUILD_SHARED_LIBS=ON)
lib/cmake/litearm/litearmConfig.cmake
lib/cmake/litearm/litearmConfigVersion.cmake
lib/cmake/litearm/litearmTargets.cmake
lib/pkgconfig/litearm.pc
```

**Consumption style 1 — CMake**:

```cmake
find_package(litearm CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE litearm::litearm)
```

**Consumption style 2 — pkg-config**:

```bash
g++ my_app.cpp $(pkg-config --cflags --libs litearm)
# static linking needs --static (it pulls in -lpthread from Libs.private)
```

**Consumption style 3 — subdirectory** (`add_subdirectory`, see above).

⚠ The umbrella header `litearm.hpp` **deliberately omits** `testing.hpp` and the transport
implementation details — to use the fakes, include `<litearm/testing.hpp>` yourself.

#### How this was verified (don't stop at "it installed")

Install it for real into a temporary prefix, then write a **real consumer** and build and run
it with `find_package` and with `pkg-config`:

```bash
cmake -S . -B build-inst -DCMAKE_INSTALL_PREFIX=/tmp/litearm-prefix \
      -DLITEARM_BUILD_TESTS=OFF -DLITEARM_BUILD_EXAMPLES=OFF
cmake --build build-inst -j && cmake --install build-inst
# then: cmake consumer with find_package(litearm CONFIG REQUIRED)  -> builds -> runs
#       pkg-config consumer with g++ main.cpp $(pkg-config --cflags --libs litearm) -> builds -> runs
```

`tests/test_packaging.cpp` additionally guards the classes of problem you can catch **before
shipping**: that the version has exactly one source of truth (CMakeLists ↔ the macros in the
header ↔ `version()`), that every sibling header referenced by a public header actually lives
in `include/litearm/`, that both packaging templates are present, and that the `.pc` version is
not hard-coded.

#### Cross-compiling for aarch64

```bash
cmake -S . -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64-linux-gnu.cmake
cmake --build build-arm64 -j
```

⚠⚠ **This repository has never been run on aarch64** — only the "does it compile" layer has
been attempted, and **even that layer was not verified on this machine** (no
`aarch64-linux-gnu-g++` installed): the toolchain file was parsed correctly and failed cleanly
at "compiler not found". Alignment, endianness and serial behaviour on arm64 are **unguarded**.
Do not read this section as "arm64 is supported".

⚠ That toolchain file **forces** `LITEARM_BUILD_TESTS=OFF`: this repository's tests do not
depend on gtest so they *would* compile, but ctest would then execute aarch64 binaries on the
build machine ⇒ a wall of "Exec format error". To actually **run** tests under
cross-compilation, configure `CROSSCOMPILING_EMULATOR` (qemu-aarch64).

## Quick start

```cpp
#include <litearm/litearm.hpp>
using namespace litearm;

int main() {
    Arm arm;                   // no port given -> auto-discover (VID:PID 1d50:606f)
    arm.connect();             // checks the firmware version convention

    std::printf("firmware %s, %d joints\n", arm.firmware().c_str(), arm.n());

    arm.enable();              // you must enable before any motion

    // Joint motion: the firmware plans an S-curve and finishes on its own
    arm.movej({0.1, 0, 0, 0, 0, 0, 0}, 0.3);

    // Cartesian line: pose = 3 positions (m) + 3 orientations (rad)
    arm.move_l({0.30, 0.0, 0.40, 3.1416, 0.0, 0.0}, 0.5);

    // Read state -- note the .value: these return a Msg envelope, see below
    auto st = arm.get_state().value;
    if (st) std::printf("q = %s\n", fmt(st->q()).c_str());

    auto tcp = arm.get_tcp().value;
    if (tcp) std::printf("TCP = %s\n", fmt(*tcp).c_str());

    // Inverse kinematics: pose -> joint angles
    auto q = arm.ik({0.30, 0.0, 0.35, 3.1416, 0.0, 0.0});

    arm.home();                // return to the zero pose
    arm.close();               // disconnect (the destructor also does this as a backstop)
}
```

`Arm::connect()` is the only entry point. Every session carries a **reader thread**, so you
**must `close()`** — the C++ destructor is a backstop, but closing explicitly is clearer (the
destructor only runs when the object is actually destroyed). With no port given it
auto-discovers; the SDK **reads no environment variables**.

## API reference

### Connecting

```cpp
Arm arm;                              // auto-discover
Arm arm2("/dev/ttyACM0");             // explicit port
arm.connect();                        // idempotent: a no-op if already connected to the same target
arm.reconnect();                      // force a rebuild (needed after changing move_timeout)
arm.close();                          // disconnect (idempotent)
printf("%s / %d\n", arm.firmware().c_str(), arm.n());
```

### Joint motion

All three require `enable()` first: `movej` is a single shot, `movej_sync` is synchronised
PTP, `home` returns to the zero pose.

```cpp
arm.enable();
arm.movej({0.1,0,0,0,0,0,0}, 0.3);            // independent S-curve per axis (first to arrive stops first)
arm.movej_sync({0.1,0,0,0,0,0,0});            // one path scalar drives every axis; they arrive together (slower but predictable)
arm.home();                                    // firmware speed is hard-coded to 0.10
```

### Cartesian motion

```cpp
std::array<double,6> P1{0.30, 0.0, 0.40, 3.1416, 0.0, 0.0};
std::array<double,6> P2{0.32, 0.0, 0.42, 3.1416, 0.0, 0.0};
std::array<double,6> P3{0.34, 0.0, 0.44, 3.1416, 0.0, 0.0};

CartPlan a = arm.move_l(P2, 0.5);              // straight line
CartPlan b = arm.move_p(P2);                   // joint-space point to point -- NOT a straight line!
CartPlan c = arm.move_path({P1, P2, P3}, 0.5); // passes through in order, sharp corners
CartPlan d = arm.move_c(arm.get_tcp().value.value(), P2, P3);   // arc (start must be the measured TCP)

CartPlan e = arm.move_l(P2, 0.5, /*wait=*/false);   // does not block; poll yourself afterwards
auto claimed = arm.poll_cart();                      // claim a result nobody took
arm.set_speed(50);                                   // global speed governor: integer percent 0..100
```

Poses also accept `(pos[3], R[3x3])` and a 4x4 homogeneous matrix:

```cpp
rot::Vec3 pos{0.32, 0.0, 0.42};
rot::Mat3 R = rot::rpy_to_mat({3.1416, 0.0, 0.0});
arm.move_l(rot::PoseInput::from_pos_rot(pos, R), 0.5);
```

### State / kinematics

```cpp
auto st = arm.get_state().value;               // std::optional<RobotState>
if (st) {
    fmt(st->q()); fmt(st->dq()); fmt(st->tau());
    bool en = st->enabled(), busy = st->cart_busy(), bad = st->faulted();
    std::string detail = st->fault_detail();
    std::vector<int> axes = st->fault_axes();   // 0-based
}
auto tcp = arm.get_tcp().value;                 // std::optional<std::array<double,6>>
arm.get_status_now();                           // ask for a frame on demand
arm.ik({0.30, 0.0, 0.35, 3.1416, 0.0, 0.0});    // pose -> q[7]
```

### Housekeeping / safety

```cpp
arm.emergency_stop();       // emergency stop: no preconditions at all
arm.reset();                // clear faults + re-anchor (not an MCU reset)
arm.clear_faults();         // clears only the RAM fault bits
arm.disable();              // ⚠ once disabled, the position loop no longer holds the arm
```

### Feedforward / dynamics

```cpp
arm.set_payload(1.0);                                   // **always** call this after changing the payload
arm.set_gravity_scale({1,1,1,1,1,1,1});
arm.ff_preset(1);                                       // 0 all off / 1 factory / 2 all on
double m = arm.get_ff_scalar(4).value;                  // read back payload_mass
arm.get_ff_mask();                                      // ff_mask (0x2C item 9, read-only)
```

### Freedrive (zero gravity)

```cpp
{
    auto zg = arm.zero_g();        // enter + start the keepalive
    std::this_thread::sleep_for(std::chrono::seconds(10));
}                                  // leaving the scope finishes it (does not throw)
// or pair explicitly: arm.zero_g_start(); ...; arm.zero_g_stop();
```

### Joint-level parameters (`arm.params()`)

```cpp
JointParam p = arm.params().get_joint_param(0).value;   // J1's kp/kd/tau_max/soft limits
arm.params().set_joint_param(0, 30.0, 1.0, 20.0);       // idx, kp, kd, tau_max
arm.params().set_joint_limits(0, -1.0, 1.0);            // can only narrow
for (const auto& jp : arm.params().all_joint_params()) {
    printf("J%d kp=%.1f q=[%.3f, %.3f]\n", jp.idx + 1, jp.kp, jp.q_min, jp.q_max);
}
```

### Dynamics model (`arm.model()`)

```cpp
if (arm.model().probe()) {                              // does the firmware support online import?
    for (int i = 1; i <= 7; ++i) arm.model().set_body(i, body10);
    arm.model().set_jm({...});
    arm.model().commit(MODEL_MASK_WRITTEN);             // 0x2FE; requires the arm to be disabled
}
auto s = arm.model().status().value;                    // override / staged_mask / dirty
auto g = arm.model().get_gravity({0,0,0,0,0,0,0}).value;  // G(q)
```

### Data capture (`arm.log()`)

```cpp
arm.log().start(300);                                   // record 300 ticks then stop by itself
LogReader r = arm.log().reader();
uint32_t total = r.total();                             // bytes recorded so far
auto samples = r.samples();                             // parsed tick by tick
// convenience: capture + wait until full + read back
auto s2 = arm.log().capture(600);
arm.log().dump("trace.bin", /*wait=*/true);             // raw bytes to disk
```

### Firmware self-test (`arm.diag()`)

```cpp
KinBenchResult kb = arm.diag().kin_bench().value;
printf("crc bad frames %lld, replies dropped %lld, CAN TX failures %lld\n",
       kb.crc_errors(), kb.reply_dropped(), kb.can_tx_fail());
```

### Licensing / activation

```cpp
LicenseInfo lic = arm.license();                        // returns normally even when not activated; does not throw
printf("%s uid=%s\n", lic.state_name().c_str(), lic.uid_hex().c_str());
arm.disable();
arm.activate(cust_id, issued, flags, mac16, 16);        // requires disabled; the MAC is issued by the vendor
```

### Persistence

```cpp
arm.disable();
arm.save_params();          // ⚠ writes flash, irreversible
```

### Read-only properties

```cpp
arm.n();                    // number of joints
arm.firmware();             // version string, e.g. "Litearm1.8.0-7J" (from the 0x41 handshake; authoritative)
arm.fw_version();           // std::optional<FirmwareVersion>
arm.cart_supported();       // whether the firmware was built with LITEARM_CART_PLAN
arm.last_reset_reason();    // "normal" / "iwdg-rst" / "" (no boot signature seen)
arm.zero_g_active();        // whether the keepalive is still running
arm.port_string();          // the port **actually** in use (falls back to the ctor argument when disconnected)
```

Diagnostic additions (**all of them diverge from the Python reference**, which has none of
them; all are pure local snapshots that **do not throw even in the terminal state** — the
moment you need them most is exactly when the link is already dead):

```cpp
arm.is_connected();         // reader thread alive **and** the status frame is not stale (both conditions required)
arm.is_in_dfu();            // whether the DFU terminal state was entered
arm.status_seq();           // running count of successfully decoded status frames
arm.msg_hz(0x40);           // mean arrival rate of that uplink frame type (Hz; the denominator is the actual arrival interval)
arm.banner_version();       // std::optional -- version string from the boot banner (the banner is sent once at boot)
arm.host_stats();           // host-side counter snapshot, see `struct HostStats`
arm.options();              // the options in effect (**current values**, not a construction-time snapshot)
```

### Tunable knobs

```cpp
arm.q_tol = 0.03;           // arrival criterion: joint angle tolerance (rad)
arm.dq_tol = 0.10;          // arrival criterion: joint velocity tolerance (rad/s)
arm.arrive_frames = 3;      // consecutive at-rest frames
arm.move_timeout = 15.0;    // motion wait window (seconds) -- ⚠ changing it requires reconnect()
```

## The `Msg<T>` return envelope

Eleven "read one frame" getters return `Msg<T> { value, hz, timestamp }`:

| Field | Meaning |
| --- | --- |
| `value` | the underlying return value. Entries that can fail to get a frame (`get_state` / `get_tcp`) express that as `std::nullopt` |
| `hz` | **mean arrival rate** of that frame type in this session (measured from the first arrival onward; does not decay while the link is idle) |
| `timestamp` | local monotonic time of the **most recent** such frame; `0.0` if none has ever arrived |

The eleven that are wrapped: `get_state` / `get_status_now` / `get_tcp` / `get_ff_vec` /
`get_ff_scalar` / `params().get_joint_param` / `model().get_body` / `model().get_jm` /
`model().status` / `model().get_gravity` / `diag().kin_bench`.

**Deliberately not wrapped**: `move_*` / `home` (they are **action results**, not "read one
frame"); purely local quantities (`n()` / `firmware()` / `last_reset_reason()`); `license()`
(there is no firmware-initiated traffic — `hz` there would only ever be the caller's own
polling rate); and the two derived getters (`get_ff_mask` / `all_joint_params`, which are
aggregates or scalar projections).

Two cases where `hz == 0.0`: (1) that frame type has **never arrived** (then `timestamp` is
also 0); (2) exactly one frame has arrived — a single sample cannot define a rate. For
single-request/response entries (`get_joint_param` / `get_body` / `get_jm` / `get_gravity`)
the **first call necessarily reports `hz == 0.0`**, and from the second call onward it equals
**the caller's own polling rate**, not any firmware period.

## Command line

```bash
./01_hello                      # equivalent to the CLI's status
./04_ik_tcp
./02_movej --go 0.1 0 -0.1 0 0 0 0
```

The SDK also ships a small CLI (`litearm::main_impl`) for walk-up checks:

```bash
litearm-cpp status              # read-only
litearm-cpp fw                  # version string + joint count
litearm-cpp tcp                 # current pose + frame rate
litearm-cpp movej -0.1 0 0 0 0 0 0 --speed 0.3
litearm-cpp home
```

## Caveats

### Multi-process: a forked child must not reuse the parent's `Arm`

The commands **really do go out on the wire**, but the parent's reader thread will eat the
replies — all you see is a "no reply" timeout, and retrying means **sending the command a
second time**. Reading state is sneakier: it does not error, it just **forever returns the
stale value it inherited**.

So this library is **fail-closed**: inside a child process, any command immediately raises
`ForkedSessionError` and not a single byte goes out.

For a child to use the arm, the **parent must `close()` first** to release the port, then
fork, then construct a new `Arm` in the child.

⚠ `close()` is **still callable** in the child (it only clears session state and does not
touch the transport, so it will not hang), but do not expect it to release the serial port.

The C++ version of this needs more explanation than the Python one — in a child process
`close()` **deliberately leaks two handles**:

- The `Ack` object must not be destroyed: its destructor would `join` a thread that **does not
  exist in the child**, and `std::thread::join()` blocks **forever** on a nonexistent thread
  while `~std::thread` calls `std::terminate` on a thread that was never joined. So it is
  `release()`d and left to the kernel.
- The transport is the same story: the read lock that `SerialTransport::close()` takes is
  **very likely held by the parent's reader thread** at the instant of `fork()`, and the
  thread that could release it is not in the child — taking it is a deadlock.

A child either exits quickly or constructs its own `Arm`, so the leak is bounded.

**⚠ The step that genuinely hangs is destroying a `std::condition_variable` that somebody is
parked on** (located empirically, glibc 2.35): the parent's reader thread is parked on `Ack`'s
`stop_cv_`, and destroying that condvar in the forked child makes glibc take the condvar's own
internal lock (a group switch) — a lock bound to a waiter that **no longer exists** ⇒
permanent block. Minimal reproduction: one thread parked in `cv.wait_for(...)`, `fork()`, then
destroy that condvar. (In the same experiment: destroying a condvar/mutex **nobody is waiting
on** is fine; `detach()`ing an inherited thread handle does not throw; `malloc`/`free` are
safe — the problem is **only** a condvar someone is parked on.)

So the rule here is: **never destroy an inherited session object in a child process**.
Following the supported paths (`close()` / leaving the scope) satisfies this automatically;
if someone bypasses it and destroys the object directly, the library **warns loudly** rather
than hanging silently — the same "dropped frames are never silent" stance as the rest of the
repository.

(Tests: `tests/test_fork_guard.cpp` really calls `fork()` and checks 7 cases — file-descriptor
inheritance, "threads are not copied", and "destroying the condvar hangs" are all operating
system facts that no fake can simulate.)

### Safety rules

1. **After `disable()` the position loop no longer holds the arm** — it will sag under load.
2. **`movej` does not check joint limits** — an out-of-range target is clamped by the firmware
   and the arm **still travels the whole way**.
3. **`move_js` / `send_mit` need ≥10 Hz keepalive** — otherwise the 0.1 s watchdog drops
   stiffness and the arm slowly goes soft.
4. **`enter_dfu()` is a terminal operation** — every entry point stops working afterwards, and
   you must construct a new `Arm` after reflashing.

### Irreversible commands: do not run these on a calibrated arm

The following overwrite or erase the **per-arm calibrated** dynamics model, with **no undo**:

| Entry | Effect |
| --- | --- |
| `save_params()` | writes current RAM into flash |
| `arm.model().commit()` | makes staged dynamics-model changes take effect |
| `arm.model().revert()` | rolls the dynamics model back (does not touch flash, so a power cycle brings it back) |
| `arm.params().reset_factory()` | restores factory settings |

Only do these on a board whose calibration is worth nothing.
**`arm.model().set_jm()` should never be called** — a wrong joint mapping makes the arm flail,
and there is no reliable way back.

When stress-testing the CAN link, run only `candump` (read-only); **never `cangen`**: `can0`
**is** the motor bus.

## Examples

See [examples/README.md](examples/README.md):

- `01_hello` — handshake + firmware version + read state
- `02_movej` — joint motion
- `03_move_p` — Cartesian point to point
- `04_ik_tcp` — inverse kinematics and the current pose
- `05_ff_tune` — dynamics / control-law tuning
- `06_cartesian` — Cartesian line / arc / multi-waypoint
- `07_vel_jitter_trace` — tick-by-tick capture of the 100 Hz status stream and the 300 Hz firmware log together

Examples are **read-only by default**; anything that moves requires `--go`:

```bash
source env.sh
./run_example.sh 01_hello
./run_example.sh 02_movej --go
```

## Tests

```bash
./build.sh                  # configure + build + ctest (fully offline, never touches hardware)
ctest --test-dir build -R test_cart -V
```

27 test binaries covering protocol/CRC/rotation/error codes/status frames/serial (a real
pty)/session assembly/the command surface/**frame ownership**/**command coverage**/**firmware
protocol sync**/feedforward/joint parameters/model/capture/self-test/Cartesian/zero-gravity/
licensing/concurrency/**fork guard**/CLI.

**No hardware is required** — the tests use a scripted fake transport
(`litearm::testing::FakeTransport`), and the serial layer itself is tested through a **real
pty** (not a fake: a fake would ignore `timeout` entirely, so defects like "`read_frame(0)`
actually touches zero bytes" go green on a fake).

### The unverified register

`UNVERIFIED_REGISTER.md` at the repository root lists, item by item, everything that is
**unverified / known-imperfect / deliberately diverging from the reference implementation** —
and **every entry must carry a source reference** (`file:line`), otherwise it does not count
as registered (nobody can re-check it in the field). Skim it before saying "this was tested".

⚠ That discipline **has machine-enforced criteria** (`tests/test_register.cpp`): the column
count, the shape of the reference, contiguous ids, and "**an invisible row that lost its
leading `|`**" (such a row still reads like a registration while the machine cannot see it at
all). The criteria check **shape only** — they cannot tell whether a reference still points at
the sentence it meant to. That limitation is itself registered (entry 18).

### Firmware protocol sync (`test_protocol_sync`)

This one **needs the firmware repository** to run, and **SKIPs** by default (not "passes" —
a false green is worse than no test):

```bash
LITEARM_FW_DIR=/path/to/litearm-stm32 ctest --test-dir build -R test_protocol_sync -V
```

It parses the firmware sources directly (**no firmware build required**) and enforces a
bidirectional comparison:

- every firmware `CMD_*`/`RSP_*` has the **same name and value** in the SDK's `protocol.hpp`
  (and the reverse — the SDK may not declare a command the firmware lacks);
- every implemented firmware command has an entry in `COMMAND_COVERAGE`, with **no stale
  entries**;
- the three exemption tables (`FIRMWARE_ONLY_CMDS`/`FIRMWARE_ONLY_RSPS`/`PREEXISTING_GAPS`)
  still hold;
- the status frame layout is still `6 + N*21` and the per-joint stride is still 21 B;
- `LITEARM_BENCH_MODEL_AXIS` in `joint_cfg.h` matches the SDK constant;
- `ERR{cmd,0x00}` is still produced **only** by the firmware's `default` branch (the SDK's
  capability detection rests entirely on this);
- the firmware's self-reported version is not below `MIN_FW`.

**Why it has to exist**: when firmware 1.5.x changed the status frame from `4+21N` to `6+21N`,
the SDK was not updated, while the fake-based tests replaced the whole transport and
manufactured the old layout — **fully green offline, guaranteed to fail on real hardware**.
Manual review does not stop this class of problem.

⚠ **SKIP is listed separately in the report**: if you see `<N> skipped` after a run, the sync
half **was not tested**. Do not read it as tested.

## Relationship to the original (litearm-python 2.1.0)

**Ported item by item**, with an identical wire protocol and semantics; differences are purely
at the language level:

| Topic | Python | C++ |
| --- | --- | --- |
| Pose argument | shape decided at runtime (`as_pose` accepts list/tuple) | `rot::PoseInput`, with a named factory for each of three spellings; a 6-vector converts implicitly |
| Joint count | `arm.n` attribute | `arm.n()` method |
| Tunable knobs | `arm.move_timeout = ...` | same-named public members; just assign |
| Unavailable values | `None` | `std::nullopt` |
| Zero-gravity context | `with arm.zero_g():` | `{ auto zg = arm.zero_g(); ... }` (RAII) |
| Teardown | `__del__` backstop | destructor backstop (likewise delegates to `close()` only) |
| Test framework | pytest | a minimal in-house framework (zero dependencies, one executable per `test_*.cpp`) |

### Four deliberate divergences from the original (since 2026-09-28)

> All four were aligned to a second, independent C++ implementation (the Gitee repository
> `yudao_hz_1/litearm-cpp`, branch `feat/p0-protocol-layer`). **Within the range both cover, it
> is byte-for-byte identical to the Python reference** (its golden samples drive this
> repository to 19/19); the only disagreements are the trade-offs below.

#### 1. Same-frame throttling: **removed entirely** (the original has it; this repository does not)

The original's `_tx_allowed` / `set_tx_repeat_min_interval` allows throttling **any** frame.
The consequence: once a command that **waits for an ACK** is dropped, the frame never reaches
the USB link ⇒ the firmware never replies ⇒ the caller can only **wait out the whole window**
and then report "no reply" — the attribution is **backwards**.

This repository **previously** narrowed the scope to write paths declared "droppable" (only
the zero-gravity keepalive is written that way). **Now it removes the mechanism entirely**,
because in the reference implementation it is **private and off by default** (behaviourally
equivalent to absent) — not worth keeping a path that can be misused:

- `Arm::set_tx_repeat_min_interval()` and `Arm::tx_throttled_frames()` are **gone**;
- the `droppable` parameter of `raw_write` is **gone**;
- ⇒ `raw_write`'s invariant is now stronger: **reaching this function means the frame will be
  written**, so the half-state "queue cleared but nothing sent" cannot exist. The criterion is
  `threading_every_raw_write_reaches_the_wire_there_is_no_drop_path` in
  `tests/test_threading.cpp`.

#### 2. Client-side prechecks: **four added** (the original has none; the firmware clamps)

All of them refuse **locally, before a frame is sent**. The single shared reason: when a
parameter is wrong enough that "every comparison is false", silently letting it through means
**the arm really moves** and the caller has no way to notice.

| Precheck | Criterion | Where |
|---|---|---|
| Non-finite targets | target contains NaN / ±inf ⇒ throw | `precheck_q_`, **ordered before "cache empty ⇒ allow"** |
| Soft-limit violation | outside `[q_min, q_max]` ⇒ throw | `precheck_q_`; the cache comes from the `0x24` reads at the end of `connect()` |
| Speed | `!(speed >= 0 && speed <= 1)` | `precheck_speed_`; **the `move_p` one is the only one of the six motion entries the original lacks** |
| Tolerances | `!(pos_tol > 0) \|\| !(rpy_tol > 0)` | inline in `move_p`; `dp >= NaN` being false would **silently skip the position half** of the check |

⚠ Two **deliberate non-checks**: `home()` is **never** prechecked (once the arm has drifted
outside its soft limits, the recovery path must not be blocked by its own precheck); and
`move_js` / `send_mit` / `send_mit_all` are pass-through commands and are not prechecked.

⚠ The cost must be stated: `connect()` therefore issues **n extra `0x24` frames** (the
reference's `connect()` does not read soft limits), and when they cannot be read it
**fails open** (no precheck, allow — never let the connection fail). Criteria in
`tests/test_precheck.cpp` (13 of them).

#### 3. Diagnostic accessors: **seven added** (the original has none)

`is_connected()` / `is_in_dfu()` / `status_seq()` / `msg_hz()` / `banner_version()` /
`host_stats()` / `options()`. All are **pure local snapshots that do not throw even in the
terminal state** — the moment you need them most is exactly when the link is dead. Criteria in
`tests/test_accessors.cpp` (11 of them).

⚠ `options()` returns **current values** rather than a construction-time snapshot (consistent
with this repository's design, where the knobs are public members mutable at runtime). This
repository's `port_string()` was also fixed along the way — it used to return the **constructor
argument**, which is permanently empty under auto-discovery, **while the header comment
promised "or the auto-discovered one"** (comment and implementation had drifted apart;
reproduced on real hardware).

#### 4. Time source: **injectable** (the original has none)

```cpp
auto clk = litearm::testing::FakeClock::make();
litearm::ArmOptions opts;
opts.port = "fake";
opts.clock = clk;                  // empty = SteadyClock (the production default)
```

⚠⚠ **This repository's clock only answers "what time is it", not "how do I wait"** — a
**deliberate difference** from the other implementation (the Gitee one): its `Clock` also has
`sleep_until(t)` (a fake clock jumps virtual time to `t` and returns immediately), which this
repository **does not provide**. The reason: every wait here is a
`condition_variable::wait_for`, relying on **being notified the instant a frame arrives**;
moving sleeping into the clock would throw that away, forcing every wait to run out a full
slice.

⇒ The semantics here are: **deadlines / expiry comparisons / timestamps** use the injected
clock; **waiting** uses the condition variable's **real** time.

⚠⚠ **One consequence you must know**: injecting a fake clock that **does not advance on its
own** means a `now + timeout` deadline **never arrives** ⇒ any criterion of the form "the
timeout really happens" will **hang** (not fail — hang). So a fake clock is only for
**pure-comparison** criteria (expiry comparisons, timestamps, TTL levels — the ones that do
not wait). The other implementation reached the same conclusion (its comments likewise say
"criteria where the timeout really happens must use SteadyClock"); the difference is that
there it degrades to a skip, while here it hangs.

**Its payoff is one concrete test**: "how long without a frame counts as disconnected"
(`STATUS_STALE_MAX_S` = 2.0 s) used to require **really sleeping a full 2 seconds**; with a
fake clock, `advance(3.0)` is **instantaneous**, and the assertion no longer mixes in "will the
scheduler wake up on time". Criteria in `tests/test_clock.cpp`.

⚠ **What does not follow the injection**: the transport layer (`transport.cpp`) **keeps the
real clock** — its `poll` / `tcdrain` timeouts are **hardware** timeouts, where injecting a
fake clock would be **wrong**, not merely useless.

### Known inherited gaps (all of them exist in the original; none were introduced by the port)

- `license()` on firmware **below 1.8.0** reports a "**no reply timeout**" rather than "the
  firmware lacks this command": it calls `expect` without an `echo_cmd`, so the
  `ERR{0x2F,0x00}` lands in a different queue and this entry cannot see it. The evidence and
  the fix are in a test in `tests/test_license.cpp`. The port **deliberately preserves the
  original semantics**.
- `get_status_now()` can **report success** when the firmware **rejects** `0x40` (returning a
  status frame instead of raising `UnsupportedByFirmwareError`). The reason is that the second
  half of its conclusion predicate is "**any** new status frame arrived" — and the firmware's
  100 Hz passive stream is an independent line from our `0x40`: if a status frame lands
  between the `seq0` sample and the first predicate evaluation, the predicate is satisfied on
  the spot while the `ERR{0x40,code}` may not have been delivered yet, so the function glances
  at the ERR queue (empty) and falls through to "return success".
  **Quantified** (2026-09-28, experiment below): with **zero status frames delivered** during
  the call ⇒ 150/150 raise correctly; with exactly one delivered ⇒ 26–35 of 150 report
  success. Under TSan, `cmd_unsupported_during_status_read_...` in `tests/test_commands.cpp`
  therefore goes red about 1 time in 15.

  ⚠⚠ **The same root cause has a second manifestation with far more serious consequences — it
  fails open on a safety guard**: `Arm::reject_if_cart_in_flight()` (the reverse guard on the
  zero-gravity entry) reads one fresh frame via `get_status_now()` to judge `cart_busy`, and
  **conservatively refuses** when it cannot (`CART_IN_FLIGHT_UNCONFIRMED_MESSAGE`). The window
  above makes it "think it succeeded" — what it gets is a **stale status frame**, `cart_busy()`
  is 0 ⇒ **the guard lets it through**. That is precisely the side the guard exists to prevent:
  "entering mid-trajectory ⇒ the arm coasts to a stop on friction".
  Measured under TSan: `zero_g_is_refused_conservatively_when_the_status_is_unavailable` goes
  red 3 times in 30.

  Both tests now use `lt::quiesce()` (turn off the passive stream + wait for in-flight frames
  to settle) to remove the variable, so each pins only the thing it means to pin (the
  ERR→exception mapping / the conservative refusal). **Do not change them back to "assert
  directly on a live link"** — that turns them back into timing tests.

  **Why it is not fixed**: status frames carry **no command echo** (only ACK/ERR do), so "is
  this frame the reply to our command" is **undecidable at the protocol level**; adding a grace
  window to the failure branch only shrinks the window, it cannot close it — and a half-fix
  that *looks* fixed is more dangerous than a known limitation written down. On top of that,
  this path is **unreachable** on supported firmware: `connect()`'s version gate already
  rejects old firmware that lacks `0x40`.
  ⇒ Semantics preserved, recorded here and next to the `done` predicate in `src/arm.cpp`.
- **No CDC auto-discovery outside `linux`**: macOS has no sysfs, and Windows needs SetupAPI to
  read VID:PID reliably; guessing would produce false positives. Pass the port explicitly.
  This is a deliberate trade-off, not an omission.
- **The first `connect()` after USB re-enumeration times out on the handshake**
  (`get_firmware` no reply); waiting a few seconds and retrying works. Measured on real
  hardware (2026-09-28): **4 times** that day, **every first connection** after replugging or
  re-attaching the USB passthrough failed, and every retry succeeded; never happened on a warm
  link.
  ⚠ The reference implementation **does not retry either** (its `connect()` likewise sends
  `0x41` once, waits 1.5 s, and closes the link and throws on failure) ⇒ this is **inherited
  behaviour**, not introduced by the port, and the original semantics are deliberately kept.
  In the field: **retry once after a second or two**; do not immediately suspect the firmware
  or the cable. (Register entry 21; whether to add a bounded retry is an **open trade-off** —
  see there.)

### The batch of TSan warnings: **settled — false positives, with runtime evidence**

`./build.sh --tsan` reports **large numbers** of data races on `Ack::queues` (dozens from a
single test case). **The conclusion is false positive**, and the basis is not "the code looks
fine to me" but **runtime evidence**:

`Ack::MuLock` (`include/litearm/ack.hpp`) registers the owner of `mu` in **debug builds**, so
every access to `queues` / `state` / `recv_` can assert "I really am inside the lock". With
that assertion active, the **entire suite of 379 cases** ran: the assertion **never fired
once**, while TSan still reported those dozens ⇒ those accesses were **proven at runtime** to
be inside the lock.

Conversely, that assertion is now **permanent**: the invariant "`Ack::queues` must be under
`mu`" used to rest on **eyeballing** the six access sites, and breaking it would be a **silent
data race** (the hardest kind to find). Now breaking it **aborts on the spot in the next debug
build and names the site**. In release builds it degrades to `std::unique_lock`, at zero cost.

**How to use TSan**: `./build.sh --tsan` (on this machine it needs `setarch -R` to disable
ASLR; the script adds it automatically). The script sets
`TSAN_OPTIONS=halt_on_error=0:exitcode=0` — without those two switches, **a test that prints
"all passed" gets judged Failed by ctest**: the first is "exit(66) on the spot when something
is detected", which stops the assertions from ever running; the second is "set the exit code to
66 even after finishing". Been there; do not delete them.

⚠⚠ **But `exitcode=0` has a backfire (measured 2026-09-28, sneakier than the above)**: TSan's
**deadlock detector** has only a **fixed 64-slot array**
(`sanitizer_deadlock_detector.h:67`), and once more than 64 mutexes are tracked at once it
does `FATAL: ThreadSanitizer CHECK failed (0x40, 0x40)` and **kills itself on the spot** — and
that death likewise goes through `internal__exit(exitcode)` ⇒ **exit code 0** ⇒ **ctest
reports Passed**.
⇒ Any test that "creates dozens of `Arm` objects in one run" hits it, so it **dies halfway
through while the report is green**.
⇒ **The criterion is not the ctest line, it is whether the framework's own summary line**
(`all passed: N passed / M failed`) **appeared**, plus whether stderr contains
`FATAL: ThreadSanitizer CHECK failed`.
None of the current 27 suites hits it (checked by running each binary directly); to bypass it
temporarily, use `TSAN_OPTIONS=...:detect_deadlocks=0`.
⚠ The same triage also found that `data race ... tests/test_zero_g.cpp:18 in count_zero_g_on`
is **not** a false positive — it is a test helper iterating `tx_log` **without holding
`tx_lock`** while the keepalive thread is concurrently `emplace_back`ing (reallocation frees
the buffer being iterated). 36 sites across 10 test files were written that way; they now all
go through the locked `tx_snapshot()` / `tx_count()`.

## Platforms

| Platform | Serial backend | Status |
| --- | --- | --- |
| Linux | POSIX termios + `flock` + `poll` | Tested on hardware (including real pty tests) |
| macOS | POSIX termios + `flock` + `poll` | Same code path; but **port auto-discovery is unavailable** (no sysfs) — pass the port explicitly |
| Windows | Win32 `CreateFile`/`SetCommState`/`ReadFile`/`WriteFile` | Implemented but **never verified on Windows**; port auto-discovery is likewise unavailable |
| Linux / aarch64 | same as Linux (cross-compiled) | **Toolchain file only, never compiled once** — no aarch64 toolchain on this machine. See "Installing and packaging / Cross-compiling for aarch64" |

⚠ **Scope of hardware acceptance** (2026-09-28): **every reversible feature has been exercised**
— 14 groups / 68 assertions / 218 frames, **homing before and after every group**, covering the
full set of read-only accessors, joint and Cartesian motion, continuous servo, zero-gravity,
feedforward and joint parameters (read the original value, write it back), capture, self-test,
enable/disable round trips, and emergency stop with recovery; the frame-by-frame audit confirms
**no irreversible command was ever sent**.

**Still never run on hardware**: the four irreversible ones (`save_params` / `model().commit()`
/ `reset_factory` / `enter_dfu` — excluded by user decision) plus `activate()` (this board is
already activated). See `UNVERIFIED_REGISTER.md` entry 14. Do not read "most things verified"
as "everything verified".

On Windows, build with MSVC + CMake (`cmake -S . -B build -G "Visual Studio 17 2022"`).
`env.ps1` / `run_example.ps1` are in the repository root.

## License

This port reuses the upstream `LICENSE` file verbatim (**Apache License 2.0**).
⚠ The upstream `README.md` says "MIT" while its own `LICENSE` file is Apache-2.0 — that
inconsistency exists upstream; this repository treats the LICENSE file as authoritative.
