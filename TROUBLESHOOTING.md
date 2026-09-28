This is the troubleshooting guide for the `litearm` C++ SDK, which drives the
7-axis arm over the serial line protocol — read it when `connect()`,
`enable()`, or a motion command fails, misbehaves, or returns something you
cannot explain, and follow it from symptom to cause to remedy.

# litearm-cpp field troubleshooting manual

Every entry follows the same three beats: **symptom → cause → what to do**.
Start with the quick-reference table below.

Recommended order of investigation: look at the **error code** first (§4 warns
about two error-code spaces that are easy to confuse), then the **link
diagnostic counters** (§11), and only then suspect the cable.

> This file is the C++ version of the upstream
> `litearm-python/TROUBLESHOOTING.md`. The symptoms and causes are **firmware
> behavior** and are language-independent; the entry-point names here have
> been rewritten for the C++ API (things like `arm.model().commit()`).
> **Warning:** the "not yet verified" items in the upstream document's last
> section are **equally unverified in this port** — do not assume they have
> been verified just because the language changed. See §16.

## Quick-reference table

| What you see | Which section |
| --- | --- |
| `connect()` cannot open the port / cannot find the device | §1 |
| `connect()` reports a firmware version mismatch | §1 |
| `enable()` is rejected | §2 |
| Commands in a forked child time out; a retry "sometimes works" | §3 |
| Readings in a forked child **never change**, yet no error is raised | §3 |
| You get `ERR{0x02,0x03}` and cannot tell which meaning applies | §4 |
| The number "3" means two different things | §4 |
| Back-to-back Cartesian commands lose replies | §5 |
| `move_c()` arc fails | §6 |
| Cartesian accuracy is far from what you expect | §7 |
| An unexplained constant Cartesian offset | §8 |
| The state still trails a little when `movej` returns | §9 |
| `last_reset_reason()` returns an empty string | §10 |
| `kin_bench` counters are all 0 | §11 |
| A motion command sent right after `zero_g` is rejected | §12 |
| The arm slowly goes soft after `move_js` / `send_mit` | §13 |
| Flashing fails after `enter_dfu()` | §14 |
| `set_speed` / `set_joint_limits` behave counter-intuitively | §15 |
| You want to know what else is unverified | §16 |
| **After a power cycle, `enable()` returns `ERR{0x10,0x06}`** | §17 |

---

## 1. `connect()` will not connect

**Symptom**: `TransportError` (cannot open the port / cannot find the device)
or `FirmwareMismatchError` (version mismatch).

| Cause | How to confirm |
| --- | --- |
| Device not found — not plugged in, no driver, not `1d50:606f` | `lsusb`; call `litearm::find_cdc_port()` on its own and see what it returns |
| Port already in use — another process or session still holds it open | On Linux, `fuser /dev/ttyACM0`; on Windows exclusivity is enforced by the OS, so if you cannot get it, you cannot open it |
| Firmware too old (< 1.5.0), or the version string does not follow the convention | Read the `FirmwareMismatchError` message — it quotes the version string it **actually saw** |

**What to do**: for `FirmwareMismatchError`, flash `Litearm1.5.0` or later; if
the port is in use, close whatever is holding it.

**Warning:** a device **re-enumeration** (unplug/replug, after `enter_dfu()`,
a real power cycle) changes the `/dev/ttyACM*` **number**. A script with
`LITEARM_PORT` hard-coded now points at a port that does not exist — this is
the most common cause of "it worked a moment ago".

**Warning:** this library **reads no environment variables**: `LITEARM_PORT`
is only something the `env.sh` / sample layer uses.

---

## 2. `enable()` is rejected

**Symptom**: `enable(attempts)` throws `CommandRejectedError`.

**Cause and remedy**: `attempts` retries only the codes on a **whitelist** —
only the one marked "retryable" below is resent. For every other code,
resending **does nothing at all** and only wastes time.

| Code | Meaning | What to do |
| --- | --- | --- |
| `ERR{0x10,0x03}` | Transient failure (the only retryable one on the firmware's whitelist) | Leave it to `attempts`, or resend later |
| `ERR{0x10,0x06}` | **Latched** fault — resending is useless | Call `reset()` first, then find out which axis it is (`st->joint_fault` / `st->fault_axes()`) |
| `ERR{0x10,0x07}` | Resending is useless | Consult the firmware code table |
| `ERR{0x10,0x00}` | **The firmware does not have this command** | Firmware too old, upgrade it |

**Warning:** a close relative that often gets mixed in here: **when the arm is
not enabled, `movej` is rejected with `ERR[01,3]`**, and its message points at
**two** possibilities at once — "not enabled **or** EMERGENCY latched". Do not
read only the first half of it.

---

## 3. A forked child process behaves strangely

**Cause**: `fork` **does not copy threads**, but it **does copy file
descriptors**. So in the child, commands **really do go out on the wire**, and
the parent's read thread eats the replies. See the multi-process section of
the [README](README.md).

**How to recognize it**:

| Symptom | Explanation |
| --- | --- |
| A command "times out", but a retry "sometimes works" | The command **did go out**; the parent read the reply ⇒ that retry was a **duplicate send** |
| `get_state()` raises no error, but the values **never change** | It silently returns the inherited **stale** values — the most insidious case |
| `connect()` throws in the child | The parent still holds the port ⇒ the parent must `close()` first |
| Any command immediately throws `ForkedSessionError` | The guard **is working correctly**; this is not a fault |

**What to do**: have the parent call `close()` to release the port, then
`fork`, and then **construct a new** `Arm` in the child.

**Warning:** in C++ there is one more layer: in the child, `close()`
**deliberately leaks** two handles (`Ack` and the transport layer) —
destroying them would hang forever (see that README section). The leak is
bounded and intentional.

---

## 4. Two error codes that are easy to confuse

### `ERR{0x02,0x03}` is ambiguous

**The same `(command, code)` pair has two completely different origins in the
firmware**: it means both "not enabled / EMERGENCY latched" and "IK
unreachable / invalid solution".

**What to do**: receiving it **does not mean** the arm has been disabled. Look
at the **current state** (`st->enabled()` / `st->mode`) instead of at the code
— if the arm is enabled, you are on the IK branch.

### Two code spaces that both run 1–6

| Origin | Meaning |
| --- | --- |
| The `err` field in a `0x4E` reply (`CartPlan::err`) | **The planning result itself**: unreachable / collinear / over capacity / out of limits |
| The second byte of `RSP_ERR` | **Gating reason code**: not enabled `0x03` / drag teaching in progress `0x04` / `drop_hold` `0x06` |

**Both take values from 1–6, and their meanings have nothing to do with each
other.** When you get a "3", first ask which path it came from.

---

## 5. Back-to-back Cartesian commands lose replies

**Symptom**: after sending several `move_l` / `move_path` commands in a row,
one of them throws `CartReplyLostError` ("outcome unknown"), or later replies
get **misaligned** (one command's answer is collected by the next one).
Reproduced 3 out of 3 times on real hardware.

**Cause**: the firmware holds only **one** pending-planning slot at a time —
it is not a queue. The cancellation replies of the superseded command
overwrite each other, so a single missing reply shifts the whole pairing
sequence. **The root cause is in the firmware, not in this library.**

**What to do**: send Cartesian commands **serially** — wait for one to finish
before sending the next. The library already serializes calls **within a
single process**, so normal use never hits this; only cross-process or
multi-client concurrency does.

---

## 6. `move_c()` arc fails

**Symptom**: `CartesianPlanError` is thrown, and the arm **does not move at
all**.

| `err` | Cause |
| --- | --- |
| `2` | **The three points are collinear** (or nearly collinear) — the circle center runs off to infinity |
| `1` | No IK solution. Starting from the **fully extended `home` pose** (a singular configuration) **always** produces this; the firmware is doing the right thing, this is not a defect |
| `3` | Over capacity / unreachable |

**What to do**: pick three non-collinear points; when starting from a singular
configuration, move out of the singular configuration first.

**Warning:** the **`start` in `move_c(start, via, goal)` must match the
measured TCP at the time of the call** (tolerance 6 mm / 0.03 rad). It is not
a free "start from here" parameter — it **is reconciled against reality**, so
using a TCP from mid-motion as the start point is rejected. **Warning:**
**`via`'s orientation is ignored**; only its position takes part in defining
the circle.

---

## 7. Cartesian accuracy is not a single number

**The end-effector residual depends on [distance x speed x payload pose]**. It
is not some fixed device specification, and it varies noticeably with the
payload.

**What to do**: any accuracy comparison must use **the same pose, the same
conventions, and the same payload** — otherwise you are measuring a pose
difference or a payload difference, not an accuracy difference.

---

## 8. A constant Cartesian offset — check `payload_mass` first

**Symptom**: the Cartesian pose carries a constant offset of about 8 mm,
unrelated to the protocol and the planner.

**Cause**: the device still holds a stale `payload_mass = 1.0` (the factory
default should be `0.0`). The firmware's dynamics compensation uses that mass,
so the tool frame is systematically pushed off.

**What to do**: after changing the payload, **always** call `set_payload()`.
When you see an unexplained constant offset, read `payload_mass` back with
`get_ff_scalar(4)` first, and suspect other things after that.

---

## 9. `movej` returns before the arm has fully settled

**Symptom**: read the state the moment `movej` returns and every axis still
has a small residual.

**Cause**: `movej` returns when the **arrival criterion** is met — every axis
is within `q_tol`, and the velocity has been quiet for `arrive_frames`
consecutive frames. At that moment the arm has not come to a complete stop
yet.

Measured (driving J6 to 0.7): at the moment of return it reads **0.6884**;
within 5 seconds it converges on its own to **0.6991** and holds there for 20
seconds. That is about 0.012 rad (0.7 degrees), inside `q_tol = 0.03` — **this
is by design, not drift**.

**What to do**: if you need an exact value, wait a few seconds before reading,
or tighten `q_tol` yourself.

---

## 10. `last_reset_reason()` returns an empty string

**This is correct behavior, not a parse failure.** The boot signature is sent
**once**, only **after a real MCU reset**, and `reset()` does not make it
repeat. Getting an empty string in everyday use is expected.

It only has a value when you **connect soon after a real reset** (`"normal"` /
`"iwdg-rst"`).

**Warning**, a related clarification: **`reset()` is a software state reset,
not an MCU reboot.** It does **not** trigger USB re-enumeration, **the same
`Arm` object stays usable afterwards**, and the signature is not resent.

---

## 11. `kin_bench` counters are all 0

**Symptom**: the link diagnostic counters from `arm.diag().kin_bench()`
(`crc_errors()` / `reply_dropped()` / `can_tx_fail()` …) all read 0.

**Cause**: this **does not necessarily** mean the link is clean. All zeros can
also mean "nothing was ever read" — and that kind of failure is **silent**: no
error, just a 0 that looks perfectly healthy.

**What to do**: before treating this as a link health check, confirm that
frames are actually arriving — look at `Msg.hz` / `Msg.timestamp`. If both are
`0.0`, that class of frame **has never arrived**.

**Warning:** among these counters, `crc` is live and accurate; `can_tx_fail`
can be surprisingly large (a cumulative value reported by the firmware, whose
exact definition has not been verified).

---

## 12. `zero_g` keep-alive period and asynchronous exit

| Symptom | Cause | What to do |
| --- | --- | --- |
| Motion commands are rejected during drag teaching | Firmware watchdog semantics: only queries get through | Queries are unaffected; **emergency stop / disable are the exceptions** and are always let through |
| `period=0.5` is rejected locally | The keep-alive must be **under 0.10 s**; if the firmware does not receive a resend within 0.10 s it falls into fail-soft | Use a `period` in `[0.005, 0.10)`; the default `0.04` is fine |
| A motion command sent right after `zero_g_stop()` is rejected | **The exit is asynchronous** — the firmware needs a little time to wind down after the call returns | Wait a moment before sending motion commands |

A background thread resends the keep-alive automatically. If it stops because
of a **write failure**, exiting **throws**; it does not fail silently. You can
read the state from the read-only `zero_g_active()` / `zero_g_error()`.

---

## 13. The arm slowly goes soft after `move_js` / `send_mit`

**Cause**: these are **continuous-servo / passthrough** entry points that
**bypass motion planning**, and **the keep-alive is the caller's job**:
**resend at >=10 Hz**. Once the 0.1 s watchdog expires the firmware enters
fail-soft (reduced stiffness + τ=0) and the arm slowly sags under gravity —
the measured sag matches the estimate for "0.6x stiffness + τ=0".

**What to do**: as long as the motion is still needed, keep resending at
>=10 Hz.

**Warning:** the array length must be **`n`** (checked locally), and the
values must be **finite** — `NaN` / `Inf` make the firmware reject the whole
frame (`ERR{cmd,0x02}`). `send_mit` / `send_mit_all` / `move_js` all have this
check. **Warning:** `dq` in `move_js` is a **velocity reference**, not a
clamp.

---

## 14. After `enter_dfu()`

- **You cannot flash immediately**: `ACK{0x15}` only means "registered"; the device must first **re-enumerate** as `0483:DF11`. Running the flash right away fails; wait ten-odd seconds and it works.
- To tell whether the device is really gone, look for a **read or write that throws** — **not** `is_open()`, and **not** "read 0 bytes".
- Once it returns successfully, **this `Arm` is dead**: every entry point throws `ArmIsInDfuError` (except `close()`). After flashing the firmware, **construct a new `Arm`**.
- Calling it while enabled is **rejected locally** (the jump stops TIM3 ⇒ the motors release within 100 ms, and a load will sag).

---

## 15. Two counter-intuitive parameter behaviors

**`set_speed(percent)` takes an integer percentage, not a multiplier.**
`set_speed(1)` means **1% speed** — think in 0..1 terms and you get a crawling
arm. It is also **global and persistent** (it stays in effect until some call
with reset semantics), which is not the same thing as the **per-trajectory
multiplier** of `movej(speed=0..1)`. The argument must be an **`int` in
0..100**; out-of-range values are rejected locally (the C++ parameter is an
`int`, so problems like "passing a bool" cannot arise here).

**`set_joint_limits()` is not idempotent.** The firmware **only allows
narrowing**, so writing the **current** values back unchanged is judged a
"widening request" and rejected (`ERR[23,2]`). ⇒ Do not use it for a
read-modify-write round-trip check.

---

## 16. Not yet verified

None of the following counts as verified:

- `enter_dfu()` — the only terminal operation; recovering from it requires reflashing the firmware;
- The persistence of `save_params()` (it writes flash);
- `reset_factory()` (it wipes the tuned parameters);
- The **success** path of `move_c()` — no reliably succeeding arc was ever constructed;
- `move_js` / `send_mit` / `send_mit_all` — never run on real hardware;
- The **actual narrowing effect** of `set_joint_limits()` (only "writing the old values back is rejected" was verified);
- **Whether `capture()` always records 0 frames while disabled** — this appears only in this repository's field notes; there is no corresponding check or test in the code; not re-checked;
- **Windows** — unverified (the serial backend is implemented, but it has never been run on a real Windows machine);
- **The C++ port itself**: all tests are **offline** (fake transport + real pty), and **none has been run on a real arm** — this is a different matter from the items above, but do not assume it is verified either.

### Irreversible commands: do not run them on a calibrated arm

All four of the following **overwrite or erase this arm's individually
identified dynamics model**, and there is **no undo**:

| Command | Entry point |
| --- | --- |
| `0x25` | `arm.save_params()` |
| `0x32` | `arm.model().commit()` |
| `0x36` | `arm.params().reset_factory()` |
| `0x37` | `arm.model().revert()` |

**The only safe approach: do it on a board whose calibration has no value.**

**Warning**, also: `0x33` `arm.model().set_jm()` **should never be called** —
it rewrites the joint mapping (including signs), a single mistake makes the
arm **flail**, and the only two local recovery paths (`revert` /
`save_params`) are both in the table above ⇒ **there is no reliable way
back**.

**Warning:** when stress-testing the CAN link, run only **`candump`
(read-only) — never `cangen`**: `can0` **is** the motor bus.

---

## 17. `enable()` returns `ERR{0x10,0x06}` after a power cycle (latched)

**Symptom** (measured on real hardware on 2026-09-28, twice): the board loses
power ⇒ the arm loses force and collapses under gravity ⇒ after power is
restored, `enable()` returns `ERR{0x10,0x06}` (latched; you must call
`reset()` first, **resending is useless**).

**Warning:** **work out which kind it is first** — the two measurements have
**different shapes, and different remedies**:

### Case A: all-axis latch (most common)

```text
joint_fault=0x007F  enabled=0  err=0 (7/7 axes)  max torque amplitude ≈ 0.2 N·m (that is just noise)  FB_STALE
```

**This is the normal state of "the motors are not enabled yet"**, not broken
hardware. With this sequence it recovered in the measurements:

```text
clear_faults()  ->  joint_fault cleared (0x007F -> 0x0000)
                   Warning: faulted() is still 1 here — because mode=EMERGENCY also counts as faulted
reset()         -> everything else cleared (flags zeroed, mode=INIT)
enable()        -> succeeds; and **the motors start reporting immediately** (axes with err=0: 7/7 -> 0/7)
```

⇒ The criterion is **whether `err` becomes 1 after `enable()`**. If it
changed, the arm is genuinely good.

### Case B: single-axis latch

```text
joint_fault=0x0008 (axis 4)  FAULT + WD_TRIPPED + FB_STALE
enable() returns ERR{0x10,0x06}; movej() returns ERR{0x01,0x06} "rigid hold on lost link (drop_hold)"
```

Measured: `clear_faults()` **changes nothing, bit by bit**; `reset()` clears
it **but it latches again within one check period**.

**Warning:** the **difference between cases A and B has not been explained**
(the suspicion is that `drop_hold` and an ordinary `joint_fault` are not the
same bit, or that it is related to "that `enable()` never succeeded and the
motor never came online"). **This manual does not invent a unified theory for
them** — both cases are measurements, and they are written as measured.

### General handles

| Handle | How to use it |
| --- | --- |
| The `joint_fault` bit map | `0x0008` = bit3 = **axis 4** (1-based). You do not need to tear down the whole set — go and check that one motor |
| `fault_axes()` | Same, returns the list of axis numbers |
| The **`err` byte** | Looks like "whether that axis's motor **is reporting**": healthy = 1, bus down / not enabled = 0. Warning: the SDK only **passes it through verbatim** and does not interpret it, so use it as a **clue**, not as a conclusion |
| Whether `FB_STALE` is "current" or "historical" | Call `reset()` once and read again immediately: **it comes back** ⇒ the hardware still has a problem; **it does not come back** ⇒ it was only a historical latch |

### Two things that are easy to misjudge

- **`enabled=1` does not equal "the enable succeeded".** When the firmware loses contact with one of the motors it **powers the axis itself to hold it** ("rigid hold on lost link"). The upside is that the arm does not fall; the downside is that it **masks the fact that "the enable was actually rejected"**.
- **`reset()` drops the enable** (measured `enabled: 1→0`). It is a **software state reset, not an MCU reboot** (see §10): it does not trigger USB re-enumeration, and the same `Arm` object stays usable afterwards.
