This document describes how the litearm C++ SDK is built — the wire protocol, the read and write
paths, and the safety invariants that hold them together — and it is written for anyone who is
going to modify `src/`, `include/litearm/`, or the tests, or who needs to know why part of the API
behaves the way it does.

# litearm-cpp Developer Guide

The C++ SDK for the LiteArm robotic arm — it talks straight to the firmware over a USB serial
link.

Planning, kinematics and dynamics all live in the firmware; the host side does exactly three
things: **encode and decode frames**, **send commands**, and **judge arrival**.

## Contents

1. [Module map](#1-module-map)
2. [Wire protocol](#2-wire-protocol)
3. [Read path: one reader + split queues](#3-read-path-one-reader--split-queues)
4. [Write path: one write point + two hooks](#4-write-path-one-write-point--two-hooks)
5. [Cartesian: FIFO pairing and the absorb budget](#5-cartesian-fifo-pairing-and-the-absorb-budget)
6. [Lock order](#6-lock-order)
7. [Safety invariants](#7-safety-invariants)
8. [Error surface](#8-error-surface)
9. [Tests](#9-tests)
10. [Comparison with the Python version](#10-comparison-with-the-python-version)

---

## 1. Module map

| Header | Python counterpart | Responsibility |
| --- | --- | --- |
| `protocol.hpp` | `_protocol.py` | frame encode/decode / CRC16 / constants / status-frame layout / version parsing / the coverage contract |
| `errors.hpp` | `errors.py` | exception hierarchy + the `(cmd, code)` semantics table + the `ERR` -> exception mapping |
| `rot.hpp` | `_rot.py` | pure rotation/pose math (ZYX intrinsic, the same convention as the firmware's `kin.c`) + pose-shape normalization |
| `state.hpp` | `state.py` | `RobotState` / `JointState` — the object-oriented view of a status frame |
| `transport.hpp` | `transport.py` | transport interface + the real serial port (POSIX/Win32) + port discovery + the in-process port registry |
| `ack.hpp` | `_Ack` in `arm.py` | the **single reader** + queues split by (uplink id, echoed code) + `expect` |
| `cart.hpp` | `cart.py` | Cartesian FIFO pairing / the absorb budget / `CartPlan` / the three entry points |
| `arm.hpp` | `arm.py` | the `Arm` main class / `Msg` / `LicenseInfo` / zero-gravity sessions / CLI |
| `params.hpp` | `params.py` | joint-level parameters (0x22/0x23/0x24/0x36) |
| `model.hpp` | `model.py` | online import of the dynamics model (0x30..0x39) |
| `log.hpp` | `log.py` | 300 Hz capture (0x2D/0x2E) |
| `diagnostics.hpp` | `diagnostics.py` | firmware self-test (0x49) + text parsing |
| `testing.hpp` | `testing.py` | a fake transport with scripted replies — it makes the whole test suite run offline |
| `msg.hpp` | `Msg` in `arm.py` | the return envelope `{value, hz, timestamp}` |

**Design boundaries** (deliberately not done): an arbitrary `fk(q)` — the firmware's `CMD_GET_TCP`
can only compute the pose of the **currently fed-back q**, and there is no downlink command that
takes a q and returns a pose; impedance control — advanced cases go through pylitearm + server.
The host side does no planning and no kinematics; the heavy computation already lives in the
firmware.

---

## 2. Wire protocol

```text
Frame:  SOF(0xA5)  CMD(1B)  LEN(1B)  PAYLOAD(0..255)  CRC16_LO  CRC16_HI
CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no final XOR), covering [SOF..PAYLOAD]
```

Uplink status frames at 100 Hz (two layouts; the trailing `joint_fault u16` is new as of firmware
1.5.0):

| Layout | Firmware | Length (7J / 1J) |
| --- | --- | --- |
| `4 + 21N` | ≤ 1.4.x | 151 / 25 |
| `6 + 21N` | ≥ 1.5.0 | 153 / 27 |

One joint is 21 B = `q f32 + dq f32 + tau f32 + t_mos f32 + t_coil f32 + err u8`.
Bit allocation in `flags`: **bit0..5 = safety flags**, **bit6..8 = mode**, **bit9 = enabled**,
**bit10 = cart_busy** — so `flag_names` looks only at bit0..bit5 (see the
`for (k = 0; k < 6; ++k)` in `protocol.cpp`; raising that bound would turn mode/enabled/cart_busy
into "fault bits" as well).

**Caution:** two IDs are dual-purpose: `0x49` (downlink `CMD_KIN_BENCH` / uplink
`RSP_JOINT_PARAM`) and `0x41` (downlink `CMD_GET_FIRMWARE` / uplink `RSP_DETAIL`, the latter being
a **dead registration**). You tell them apart by **direction**; a single ID table cannot look them
up.

The coverage contract `command_coverage()` (49 entries) is the carrier for the assertion that
"every command the firmware implements has an SDK entry point"; `tests/test_protocol.cpp` pins it
against the table entry by entry.

### 2.1 The framer `FrameReader` (`include/litearm/framing.hpp`)

The state machine that cuts the byte stream into frames (SOF scanning / the half-frame window /
the noise trace) is **a class of its own**, and the two `SerialTransport` backends (POSIX and
Win32) **share the same one**.

**Warning:** **its only reason to exist is to kill duplication.** The two backends **each used to
have their own copy** of the same state machine, with a comment in the code claiming it was "the
same logic as the POSIX side" — that was **a copy that drifts**, and the Windows branch **does not
even compile** on Linux, so a drift would not have been caught on the spot by anyone. After the
extraction the two backends hold nothing but I/O, and this logic is **compiled and unit-tested**
(9 cases in `tests/test_framing.cpp`) instead of being verifiable only indirectly, through a pty.

**Caution:** **`feed()` has take-it semantics, not try-it semantics** — dropping its return value
drops that frame, and the symptom is extremely hard to see (data is flowing, yet you cannot read a
single frame). This was hit **twice in the same afternoon** in practice. The standard usage:

```cpp
if (auto f = r.feed(chunk.data(), chunk.size())) return *f;   // this batch may already hold a whole frame
while (auto f = r.feed(nullptr, 0)) { /* handle f */ }        // then drain the empty buffer
```

**Caution:** `tick(now)` returns **whether the buffer was modified**: if it was, you must retry
assembly immediately (a real frame very likely follows right behind the fake header you just
dropped). Without this return value, the caller either misses frames or spins.

---

## 3. Read path: one reader + split queues

**Exactly one place in the whole package touches `transport->read_frame`**: `Ack::reader_loop`. It
obeys only two rules:

1. **Deliver only, never decide** — "whose frame is this" is decided by the **queue**, not by the thread;
2. **Die loudly** — the cause of death goes into `reader_error` and wakes every waiter; never exit silently.

A frame's **ownership = which queue it lands in**, keyed by `(uplink id, echoed code or kNoEcho)`:

- only `RSP_ACK` / `RSP_ERR` have "the original command code" in `payload[0]`, so only they need the second dimension; for every other frame `payload[0]` is **data** (joint index / item index / status byte).
- So `ACK{0x10}` and `ACK{0x11}` naturally land in **two** different queues — two concurrent commands eating each other's replies is **structurally impossible**.

Three special deliveries:

| Frame | Destination | Reason |
| --- | --- | --- |
| `RSP_STATUS` | the **single slot** `state` + `status_seq++` | a 100 Hz **broadcast** frame. In a queue it would fill `kQueueMax` within 0.6 s; and take-it-and-it-is-gone queue semantics never fit a broadcast |
| `RSP_CART_PLAN` | handed to `CartPending::on_reply` **outside** the lock | avoids calling another subsystem's lock while holding the global lock |
| everything else | the `queues[(id, echo)]` queue | each queue caps at `kQueueMax`; going over drops the **oldest** and bumps `dropped` |

`expect(want, timeout, label, raise_on_err, echo_cmd, err_waits_for_ack)`: when `want` is
`ACK`/`ERR`, **`echo_cmd` is mandatory** (without it you cannot tell "my ACK" from "someone else's
ACK");

- `err_waits_for_ack=true`: an `ERR` matching `echo_cmd` is **not final** — keep waiting for `ACK{echo_cmd}`; if you find it ⇒ that ERR was **someone else's** (this command was in fact accepted, do not throw); if the window runs out with only the ERR ⇒ that ERR belongs to this command, throw it. The cost: **a command that really is rejected waits out the whole window**. That is why only the three Cartesian entry points turn this switch on.

**Caution:** **this shape — "an accepted command always gets an ACK, a rejected one never gets a
compensating ACK" — holds only on the generating side, not on the arriving side**: the acceptance
`ACK{cmd}` **can itself be dropped** (the firmware's reply FIFO drops the newest when full).

### Three kinds of reader

| Kind | Where frames are taken | Examples |
| --- | --- | --- |
| queue waiter | `Ack::wait` (waiting on a named queue) | `expect` / `enable` / parameter read-back |
| driver-style reader | `Arm::pump_until` (**claims not a single frame**) | `read_status` / `get_status_now` / `wait_settled` |
| event waiter | the condition variable on `token` | `CartPending::wait` |

The driver-style reader is the necessary consequence of "broadcast frames never enter a queue": it
merely **waits for the single slot to advance**, so several concurrent `get_state()` calls no
longer starve one another.

---

## 4. Write path: one write point + two hooks

`Arm::raw_write` is the single exit for **every** downlink frame (including the zero-gravity
keepalive thread, which deliberately bypasses `write_cmd`'s zero-gravity guard but **must not**
bypass the queue clear). The order of the hooks is load-bearing:

```text
① fork guard / terminal-state guard   —— structurally guarantees "a child process sends nothing" and "a terminal session sends no more frames"
② clear the queue (cart_->clear_pending)  —— must come **before** drain_for
③ drain_for(cmd)         —— must come **before** the write
④ tr_->write_frame(...)
```

- **Why ③ must come before the write**: our reply can only arrive **after** the write, so clearing the queue at this point cannot eat our own reply. The other order (write first, clear after) eats it 100% of the time.

**Warning:** **2026-09-28: this used to hold a "same-frame throttle" as well (ordered before ②),
and it has been removed as a whole.** Back then, placing it ahead of the queue clear had a bearing
(the queue clear is justified by "the firmware only invalidates an in-flight plan once it
**receives** this command", and a frame that gets dropped leaves the firmware state untouched).
Removing it makes **that bearing disappear on its own** — the single write point's new invariant
is "**reaching this function means the frame will be written out**", and the half-state "queue
cleared but the frame never sent" is **impossible to construct**. See README, "Four deliberate
divergences from the original", item 1.

Two exits at the layer above:

| Exit | Zero-gravity guard | Use |
| --- | --- | --- |
| `write_query` | none | **Queries** (get_tcp/get_ik/parameter read-back/capture/self-test) — you must still be able to read state during freedrive |
| `write_cmd` | **yes** (can be turned off with `guarded=false`) | **Actions**. `guarded=false` is only for actions in the energy-reducing direction (emergency stop / disable) — those must always be reachable |

---

## 5. Cartesian: FIFO pairing and the absorb budget

The firmware's native Cartesian replies (`0x3A/0x3B/0x3E`) carry **no command id** in the payload,
so they can only be paired FIFO by **acceptance order**; and the firmware holds only **one slot**
for those replies ⇒ when 3 or more are registered within the same drain window, the middle request
gets zero replies.

The three entry points (`move_l` / `move_c` / `move_path`) **hold the serialization lock the whole
way**: from registering the token until the `0x4E` pairing completes; with `wait=true` they **hold
it until the arm settles** as well.

### The two guards

**"One too many"** (the queue is empty yet a `0x4E` arrives): count it and throw the base
`LiteArmError`. Giving it no dedicated exception type is deliberate — it means the **pairing model
has lost sync with the firmware** and an internal invariant is broken; no caller can make a correct
decision from it, and it **should not be caught specifically**.

**"One too few"** (the token times out without being closed out): remove that token and throw
`CartReplyLostError` (**not** `MotionTimeoutError` — what this has to express is "unknown outcome";
lumping it in with a generic timeout would make the caller resend on the assumption that nothing
took effect).

### The absorb budget

When you clear the queue or abandon a token, that firmware reply **may already be on the wire**.
Without a budget, it arrives and hits the "queue is empty" criterion ⇒ a `LiteArmError` blows up
out of a **completely unrelated read path**.

The budget = the number of entries removed, and it lives for `absorb_ttl = max(move_timeout, 12.0)`
seconds.

**Caution:** **both directions go wrong**:

- **too large** ⇒ a genuinely out-of-sync reply is absorbed silently (a hard error degrades into **silence**);
- **too small / expired** ⇒ a reply still on the wire hits "the queue is not empty" and gets paired with a **newly registered** token ⇒ the caller receives **someone else's** result and **reports success** = a **false success**.

Faced with two evils you must **lean large**. The firmware basis for the 12.0 s floor is a sum of
three terms (planning 3 s + erase/write pause 8 s + link 1 s); the source for each term is in the
comment on `kAbsorbTtlFloor` in `cart.hpp`.

**Two correctness conditions for the queue clear**:

- when `n == 0` **do not refresh the deadline** — the zero-gravity keepalive sends a queue-clear opcode every 40 ms (almost always an empty clear), and if an empty clear refreshed the deadline too, the budget would **never expire**;
- the budget **accumulates across queue clears** — if it were written as "reset to this call's count", the late reply left over from the previous clear would degrade into a hard error.

**The single exception in `drop_and_absorb`**: `timed_out == true` and the budget **has already
been consumed** within this request's window ⇒ **do not top it up** (absorption happens **before**
pairing, so what got eaten is this request's own reply; topping up again would make it
**self-sustaining** — before the fix, this was measured to make every subsequent Cartesian command
in a new session report "unknown outcome").

**Caution:** this exception **holds only for the timeout branch**: the pump branch (where the read
link blows up on the spot) never waited out a window at all, so that premise has no basis for it.

---

## 6. Lock order

```text
cart_serial_  →  CartPending::lock_  →  transport::wlock_
zg_lock_      →  CartPending::lock_  →  transport::wlock_
```

**There is no cycle in the reverse direction**, and the criterion is "no lower layer knows about
an upper layer": neither `CartPending` nor `SerialTransport` references `cart_serial_` / `zg_lock_`.

- `cart_serial_` is a **`std::recursive_mutex`** — there is exactly one nesting point: `move_path` wraps the whole `BEGIN + ADD×n + RUN` sequence in this lock, and the RUN part itself also goes through `request_and_wait`.
- `zg_lock_` serializes start/stop only; **the keepalive thread does not take it** (it takes `zg_err_mu_`) — otherwise it would deadlock with `zero_g_stop`, which holds `zg_lock_` while waiting to join.
- **`emergency_stop`/`disable`/`zero_g*`/`get_tcp` must not acquire `cart_serial_` themselves** — energy-reducing actions and read-only queries must always be reachable (the lock holder may block for up to `move_timeout`). **Caution:** but **an entry point calling `get_tcp()` inside its own critical section is a different matter**: that is exactly what `move_c`'s start-point validation does.
- `Ack::mu` is **a leaf lock, reinforced**: it is held only inside `deliver`/`wait`, and `deliver` calls **no subsystem at all** (`on_reply` is deliberately kept outside the lock).

---

## 7. Safety invariants

Each one is pinned by a test:

1. **a forked child sends nothing** — `ForkedSessionError` hangs off three structural choke points (`require` / `Ack::wait` / `raw_write`) rather than being enumerated entry by entry. A `close()` in the child **deliberately leaks** the `Ack` and transport handles. There are three **independent** deadlocks here, each measured: ① the read lock that `SerialTransport::close()` wants to take is very likely held by the parent's reader thread; ② `join()` on a thread that **does not exist** in the child blocks forever; ③ **Caution:** the most insidious one — destructing a `std::condition_variable` (`Ack::stop_cv_`) that **still has someone parked on it** blocks forever (glibc's group switch takes the condvar's internal lock, and that lock is tied to waiters that no longer exist). ③ happens inside **member destruction**, so it cannot be dodged ⇒ the caller must **not destruct that object at all**. See the fork branch of `close()` in `src/arm.cpp` and `Ack::~Ack()` (the latter describes a minimal reproduction).

   **Corollary (for the next person to add a thread)**: any object that "may be destructed in a child process and holds a condvar" has this shape. Before adding such a member, work out whether anything will destruct it after a fork.
2. **a terminal session sends nothing** — the flag is set once `enter_dfu` confirms the device is gone, and from then on every entry point throws `ArmIsInDfuError`.
3. **a write failure ⟹ the whole frame was not delivered** — `write_frame` reports a throwing `write()` and a throwing `flush()` **separately**: the former throws `TransportError`, the latter **does not throw** (the whole frame is already in the driver; a failure cannot be taken back) and only bumps `flush_failures`. This invariant is the precondition that lets `CartPending::request` remove a token safely.
4. **`RSP_CART_PLAN` replies carry no command id** ⟹ any FIFO scheme must misjudge in one branch ⟹ you can only pick **the branch that misjudges safely**: report "unknown outcome", **never** report "success".
5. **arrival ≠ stopped** — `CartPlan::settled` requires two things together: the tail of the arrival criterion **and** the **actual TCP**, read back **after** the arrival criterion is met, matching the target. Looking at `bit10` alone reports "torn off halfway, TCP nowhere near the target" as arrived.
6. **during zero gravity, action commands are refused while queries and energy-reducing actions pass** — the two throw sites share **the same constant** (`ZERO_G_GUARD_MESSAGE`), so changing one will not drift silently.
7. **`0x00` always means "the firmware does not have this command"** — it is the only stable sentinel, more reliable than a version number.
8. **irreversible commands are not done for you** — `save_params` / `reset_factory` / `model.commit` all require the caller to call `disable()` first; the library does not make safety decisions on the caller's behalf.

---

## 8. Error surface

```text
LiteArmError                     every error in this package
├─ NotConnectedError             not connected / link already closed
│  └─ ForkedSessionError         reusing the parent's session in a child process (fail-closed)
├─ TransportError                serial read/write / frame CRC / link loss
├─ FirmwareMismatchError         version does not match the convention, or is too old
├─ InvalidCommandError           illegal argument/command
├─ MotorFaultError               status frame FAULT / EMERGENCY / single-joint dropout
├─ MotionTimeoutError            move timed out without arriving
├─ IKError                       IK failed
├─ CommandRejectedError          the firmware explicitly returned ERR{cmd, code}
│  └─ UnsupportedByFirmwareError ERR code == 0x00 ("the firmware lacks this command")
├─ CartesianPlanError            plan rejected (IK/collinear/over capacity/out of limits) — the arm did not move a step
├─ MotionSupersededError         superseded by a newer request (an expected takeover, **not a failure**)
├─ CartReplyLostError            the 0x4E was lost — outcome unknown
├─ ArmIsInDfuError               terminal session state
└─ NotRemoteable / NotSupportedOnThisBackend / TeleopLockedError / TeleopBusyError
```

Three relationships that **deliberately do not inherit** (each pinned by a test):

- `MotionSupersededError` does **not** inherit `CartesianPlanError` — lumping it into "planning failed" would route a normal preemption down the fault branch;
- `CartesianPlanError` does **not** inherit `CommandRejectedError` — what the firmware returns here is a **planning result**, not the `ERR{cmd, code}` shape, and forcing it in would conjure up `cmd`/`code` fields with the wrong meaning;
- `ArmIsInDfuError` does **not** inherit `NotConnectedError` — the latter means "just connect and you are fine", while the terminal state is not a recoverable disconnect (mixing them would make "reconnect on disconnect" logic treat DFU as an ordinary dropout).

`err_reason(cmd, code)` is the **only** place that interprets `(cmd, code)`, with a three-level
lookup:

1. the specific tier `err_text()` hits ⇒ use it (the same `0x03` means different things on `0x01`/`0x10`/`0x23`);
2. the generic tier hits ⇒ use it **and append the raw code**;
3. neither ⇒ say plainly "not registered", **with the raw cmd/code**.

Why tiers 2 and 3 must carry the raw code: when the firmware adds a new error code, this path is
the **only** place where the host will see "this is a code I have not seen before".

---

## 9. Tests

```bash
./build.sh                       # build + ctest
./build.sh --asan                # AddressSanitizer + UBSan
ctest --test-dir build -R test_cart -V
./build/tests/... --filter movej # filter by name within one file (each test_*.cpp is a standalone executable)
```

| Test | Coverage |
| --- | --- |
| `test_protocol` | CRC (standard check value + an independent reference implementation) / frame assembly and parsing / version parsing / boot signature / both status-frame layouts / the coverage contract checked against the table entry by entry |
| `test_rot` | rpy<->matrix / gimbal lock / SO(3) log and exp / slerp / the direction of the pose error / the three ways of writing a pose |
| `test_errors` | the hierarchy / the three-level lookup / the whitelist / the `ERR` -> exception type mapping |
| `test_state` | enabled/cart_busy/faulted/joint dropout/`drop_hold_inferred` |
| `test_transport` | **a real pty**: byte-by-byte round trip / noise skipping and the noise trace / resync after a bad frame / a half frame surviving across calls / `timeout=0` meaning "give me whatever there is" / a fake header dropped on timeout / endpoint exclusivity / registration released on the failure path |
| `test_framing` | the framer **unit-tested directly** (no longer only indirectly, through a pty): feed several frames at once and drain them / a half frame across calls / the noise trace and its cap / fake headers timing out / resync after a bad frame / `reset` |
| `test_register` | format gatekeeping for the unverified register: five columns / every entry has a source / ids are exactly 1..N / no invisible rows missing the leading `\|` |
| `test_packaging` | the categories you can still guard before shipping: a single source for the version / every sibling header of a public header is in the installed directory / the packaging template is present |
| `test_arm_assembly` | handshake / version rejection / idempotent connect / retargeting / teardown / the four DFU terminal-state paths |
| `test_commands` | the enable retry whitelist / safety commands / joint motion and arrival / servo pass-through arity / state reads / IK |
| `test_ff` | 0x26/0x27/0x28/0x31 + 0x2B/0x2C read-back / the item table |
| `test_params` | 0x22/0x23/0x24/0x36 + the armed gate |
| `test_model` | probing / the three levels staging-bank-commit / the mask bit by bit / requires disable |
| `test_log` | sample layout / cursor continuation / drop retry / the infinite-loop guard when the cursor does not advance / the per-tick recording model |
| `test_diagnostics` | timing lines (a name joined to a number) / LINK lines (**keys whose names contain digits**) / both frames received / old-firmware compatibility |
| `test_cart` | `CartPlan` / exception mapping / the three entry points / **FIFO pairing** / **the absorb budget in both directions** / the queue clear / capability probing |
| `test_zero_g` | the keepalive thread / the guards in both directions / RAII / an interrupted keepalive is visible / close stops the thread |
| `test_license` | the 26 B record / not activated / read-back qualification of the `0x02` aggregate tier / the enable gate when unlicensed |
| `test_threading` | concurrent state readers / concurrent Cartesian entry points serialized / close waking the waiters / `Msg.hz` semantics / no drop path on the single write point |
| `test_precheck` | client-side prechecks (non-finite values / soft limits / speed / tolerances) and the two rules for the soft-limit cache |
| `test_accessors` | the seven read-only diagnostic accessors (including not throwing in the terminal state, and the forwarding chain through the wrapper layer) |
| `test_clock` | the injectable time source: the real clock by default / injection is visible / "too old ⇒ link lost" is deterministically verifiable under a fake clock |
| `test_frame_ownership` | **frame ownership**: the single read point (source scan) / the ownership criterion / two threads not eating each other's replies / stale frames clearing the queue / the queue cap / the guards |
| `test_full_coverage` | **the dynamic command-coverage sentinel** (every implemented command really was sent out) / no extra ids / the public-member list guard / a full sweep of error types while not connected |
| `test_protocol_sync` | **a two-way comparison against the firmware headers** — **needs `LITEARM_FW_DIR`, otherwise SKIP** (see the end of §9) |
| `test_fork_guard` | a real `fork()`: the child sends nothing / teardown does not hang |

**All of it is offline** — using `testing::FakeTransport` (a fake transport whose replies are
scripted per command). The serial port itself is tested with **a real pty**: a stub ignores
`timeout` entirely, so defects like "`read_frame(0)` actually touches not a single byte" come out
all green on a stub (that is exactly how upstream once missed a real-hardware failure).

**The only exception** is `test_protocol_sync`: it reads source from the **firmware repository**
and **SKIPs** when it cannot find it (`LITEARM_FW_DIR`, default `~/litearm-stm32`). The
frame-ownership suite holds two more **source-scanning** criteria (the single read point, the
public member list) — they derive a path from `__FILE__` through `lt::repo_root()`, independent of
the current working directory (ctest's cwd is the build directory, and a relative path will not
open there — hit in practice).

**Caution:** **a SKIP is not a pass**: these are counted separately in the report and printed one
by one. Seeing "N skipped" means that part **was not tested**.

---

## 10. Comparison with the Python version

The port is a **line-by-line comparison**: the same constant values, the same error strings, the
same order of criteria. The only differences are at the language level:

| Topic | Python | C++ |
| --- | --- | --- |
| pose argument | shape decided at runtime | `rot::PoseInput` (a named factory for each of the three spellings; a 6-vector converts implicitly) |
| unavailable values | `None` | `std::nullopt` |
| attributes | `arm.n` | `arm.n()` (read-only) / public members (tunable knobs) |
| context manager | `with arm.zero_g():` | RAII `ZeroGSession`, the destructor does not throw |
| overloads | same name, polymorphic | convenience overloads such as `Arm::write_frame(cmd, {payload})` |
| weak references | `weakref` (the reader thread does not pin `Arm`) | not applicable: the reader thread is joined in `close()`, and the lifetime is guaranteed by the join order |
| `__del__` | GC as a teardown backstop | the destructor as a teardown backstop (likewise delegating only to `close()`) |

**Caution:** **two known inherited gaps** (they exist in the original and were deliberately
preserved by the port):

1. `license()` does not pass `echo_cmd`, so on firmware **below 1.8.0** it reports "no reply timeout" instead of "the firmware lacks this command" (evidence in `tests/test_license.cpp`).
2. outside `linux` there is no automatic CDC discovery (macOS has no sysfs; Windows needs SetupAPI to read the VID:PID reliably, and guessing one would produce false positives).
