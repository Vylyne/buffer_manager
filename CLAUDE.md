# CLAUDE.md

Ground rules for working in this repo. Read `NOTES.md` first, every session —
it is Vi's inbox to Claude.

## Project stance

This is a **downstream project**, not an upstream contribution. It is a fork of
`Nitrooxyde/Mellow-LLL-Plus-Canonical`. Byte-identical parity with upstream is
**explicitly abandoned** — restructuring is allowed. French comments in the
firmware and host code get translated to English as they are touched.

`upstream/main` is a remote-tracking branch, not something to merge from. Use it
for `git diff upstream/main -- <path>` and for deliberate, one-off cherry-picks.

`c:/git/github/buffer_manager_old` is a **read-only reference repo** — the
retired firmware fork. Never edit it; only read it for comparison.

## Hardware and filament path

```
Filamentalist V3 rewinder → buffer → MDM (on buffer arm) → long reverse bowden
    → Roadrunner → short bowden → Extruder → toolhead switch → hotend
```

Six buffers, six toolheads, **1:1, no splitter/merger**. Passive rewinders are
present. There is no toolhead cutter.

## Pin map

| Pin | Old fork | Canonical (current) | Target |
|---|---|---|---|
| PB2/PB3/PB4 | HALL1/2/3 | same | unchanged |
| PB7 | filament entry switch | same | unchanged |
| PB12/PB13 | KEY2/KEY1 internal feed/retract | same | unchanged |
| PA2/PA3 | external retract/feed buttons | `inform` **outputs** | external buttons, in firmware |
| PA4/PA5 | MDM presence / motion | IOEXT ch0/ch1 | MDM module |
| PB5/PB6 | MBSIG | same | unchanged |
| PB10/PB11 | left free for Klipper | IOEXT ch2/ch3 | leave unconfigured, Klipper owns |
| PC13/PC14 | EXTENSION_PIN1/2 | undefined | leave undefined |
| PA6 / PB1 | TMC enable / UART | same | unchanged |

Old-fork pin defines live at `buffer_manager_old/klippy_src/buffer.c:37-56`.

## Standing constraints

- **Nothing here has been validated on hardware.** Every distance and duration
  in code is a **placeholder** and must be commented as such.
- **Never run gcode from a Klipper timer callback.** Hand off via
  `reactor.register_callback()`, which runs in a greenlet that may block.
- **Bound every assisted motion with a deadline.** The shipped command set only
  offers bounded moves on purpose.
- Do not commit unless explicitly asked to. Do not push.

## Gate commands

Run before reporting any task done:

```bash
cd c:/git/github/buffer_manager
python -m py_compile klippy/extras/*.py
git status --short          # nothing unexpected
git diff --stat             # scope matches the task
```

### Firmware builds cannot happen on this machine

`src/buffer.c` is not built in this repo. It is *copied into a Klipper tree* on the
Klipper host (a Raspberry Pi) and built there:

```bash
cp ~/lll-firmware/src/buffer.c ~/klipper/src/buffer.c
# add to ~/klipper/src/Makefile  (reverted by `git pull` on the Klipper tree):
#   src-$(CONFIG_MACH_STM32F072) += buffer.c
cp ~/lll-firmware/build/klipper.config ~/klipper/.config
cd ~/klipper && make olddefconfig && make clean && make
```

Target: STM32F072, 8 KiB bootloader offset (application address `0x8002000`, must
match Katapult), 8 MHz crystal, USB on PA11/PA12. Reference binary and the exact
config that produced it are in `build/`.

**Consequence:** an agent working on this Windows checkout **cannot compile the
firmware and must never claim it compiles.** Verify C changes by review only, and say
plainly that the build was not run. The same applies to hardware behaviour: nothing in
this repo has been hardware-validated.

## Klipper API gotchas

**Registering for MCU responses.** The retired fork crashed at startup on this:

```python
self.mcu.register_response(self._handle_result, "buffer_result")
# AttributeError: 'MCU' object has no attribute 'register_response'.
#                 Did you mean: 'register_serial_response'?
```

Follow the pattern in `klippy/extras/lll_buffer.py` instead — it is the code that
actually runs on this hardware:

```python
self.mcu.register_serial_response(self._handle_event, "lll_event ev=%c a=%u")
```

Two separate traps live here. The method is **`register_serial_response`**, and what
you pass as the message argument matters — `lll_buffer.py` passes the *full format
string*. The old fork additionally carried a comment recording that it had already
been bitten once by passing the wrong form of that argument. Copy the working call
shape rather than reasoning it out from the name.

**Prefer not to touch the MCU at all.** `klippy/extras/buffer_manager.py` makes zero
direct MCU calls by design: it drives the lane through the backend seam and hears
about firmware events through `lll_buffer`'s `event_gcode` hook. That is why this
whole class of bug cannot reach it. Keep it that way — new firmware-facing plumbing
belongs in `lll_buffer.py` or the backend, not in the pipeline.

