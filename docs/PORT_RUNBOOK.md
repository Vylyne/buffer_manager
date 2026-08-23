# buffer_manager port — execution runbook

Work packages for delegated agents. Each task is self-contained: an agent that never
saw the planning conversation should be able to execute it from the task text plus
**§0 Shared facts**. Every agent reads §0 first.

Repo: `c:/git/github/buffer_manager` (fork of `Nitrooxyde/Mellow-LLL-Plus-Canonical`).
Read-only reference: `c:/git/github/buffer_manager_old` — the retired firmware fork.
**Never edit the reference repo.**

---

## §0 Shared facts

Read this before any task. These were established by investigation; do not re-derive
them, and do not assume anything here is wrong without evidence.

### Project stance

This is a **downstream project**, not an upstream contribution. Byte-identical parity
with upstream is **explicitly abandoned**. French comments get translated to English.
Restructuring is allowed. `upstream/main` remains a remote-tracking branch for
`git diff upstream/main -- <path>` and deliberate cherry-picks.

### Hardware and filament path

```
Filamentalist V3 rewinder → buffer → MDM (on buffer arm) → long reverse bowden
    → Roadrunner → short bowden → Extruder → toolhead switch → hotend
```

Six buffers, six toolheads, **1:1, no splitter/merger**. Passive rewinders present.
No toolhead cutter.

### Pin map

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

### Firmware facts (`src/buffer.c`, ~1581 lines, French-commented)

- **Protocol block at lines ~37-99** documents every command, event and param. It is
  the firmware↔host contract and must stay byte-consistent with
  `klippy/extras/lll_buffer.py`'s format strings.
- **Events:** `lll_event ev=%c a=%u`. `0=ecode_set 1=error_cleared 2=present_edge
  3=diag_edge 4=button 5=arm_change 6=role`.
- **`arm` is a raw bitfield** `H3H2H1` — bit0=hall1, bit1=hall2, bit2=hall3, built by
  `(gpio_in_read(hall3)<<2)|(gpio_in_read(hall2)<<1)|gpio_in_read(hall1)`. Obstructed
  = 1. Identical layout to the old fork's hall array.
- **Events are mask-gated:** `ev_send()` drops anything not in `telem_mask`, default
  `0x0F`. **EV_ARM is bit 5 and therefore OFF by default** — arrival detection needs
  the mask raised to `0x2F`. Param is TELEM func 9, key 0.
- **`inform`** = PA2/PA3 outputs pulsed 3 s after a *short click* on KEY1/KEY2, to tell
  a non-Klipper mainboard what the user pressed. Reverse-engineered parity with
  Mellow's stock firmware (`buffer.c:517-520`, logic at `718-746`). Redundant under
  Klipper, which already receives `EV_BUTTON`.
- **IOEXT is poll-only** — there is no IOEXT event in the enum. Reading a channel means
  `lll_io_read`, bounded by the host poll interval (default 1.5 s). Unusable for
  anything in a control loop.
- IOEXT table at `src/buffer.c:292-305`. ch4/ch5 (PA2/PA3) are flagged `unverified`
  and need `lll_unlock` (key `0xDEB1`).
- `FEN_DEFAULT` enables FEED, BUTTONS, ENTRANCE, TMC, MOVE, TELEM.

### Host facts (`klippy/extras/lll_buffer.py`, ~732 lines, French-commented)

- Section `[lll_buffer <name>]`; registers `LLL_*` mux commands keyed on `LANE`.
- `event_gcode` hook runs a template on **every** firmware event, with
  `event.name/code/value/lane` in context. Guarded against re-entrance by
  `_in_event_gcode`.
- `probe_arm()` sends a fresh `lll_query` and returns `(arm, role)`.
- `_persist()` writes tweaks to `[save_variables]` when present.

### Roadrunner facts

Vendor repo: `C:/Syncthing/Maker Stuff/Printers/Git/roadrunner-filament-sensor`.
Klippy module `klippy/extras/high_resolution_filament_sensor.py`, section
**`[high_resolution_filament_sensor <name>]`**.

`get_status()` returns: `filament_detected` (bool), `position` (**float mm**, magnetic
rotary encoder, computed independently of the extruder — valid even when the extruder
is idle), `motion.{detected,direction,commanded_distance,expected_distance,
measured_distance,measured_speed}`, `underextrusion_rate`, `underextrusion_detected`,
`runout`, `sensor_connected`, `magnet_state`.

Internal state refreshes every **100 ms** (`CHECK_RUNOUT_TIMEOUT = .100`) — that is the
resolution ceiling; sampling faster gains nothing. `motion.*` and
`underextrusion_detected` are referenced against *commanded extruder moves*, so they
are meaningful only while the extruder is pulling. `position` is valid in all phases.

### MDM encoder scale — derived, not guessed

The Mellow MDM ships three printable encoder disks. Their published "minimum
detection distance" is the filament travel per **full segment cycle** (one segment
plus one gap):

| Disk | Min detection distance | segments x distance |
|---|---|---|
| 3-segment | 5.8 mm | 17.4 |
| 5-segment (**installed**) | 3.5 mm | 17.5 |
| 10-segment | 1.73 mm | 17.3 |

That product is constant at ~17.4 mm, which is the filament travel per disk
revolution — the cross-check that confirms the reading.

**Vi runs the 5-segment disk (3.5 mm).** Our firmware counts **both** edges
(`track_edges()` in `src/buffer.c` increments on any level change), so one count is
half a cycle:

> **`mdm_mm_per_count` = 3.5 / 2 = 1.75 mm**

Two independent routes to the same figure, which is the reason to trust it:

| Route | Arithmetic | Result |
|---|---|---|
| Per revolution | 5 segments break the beam 5x/rev = **10 edges/rev**; 17.4 mm / 10 | 1.74 mm |
| Per cycle | 3.5 mm per cycle / 2 edges | 1.75 mm |

The difference is rounding on whether the circumference reads 17.4 or 17.5 mm.

**The trap:** 3.5 mm is the number on the vendor's spec sheet, so plugging it straight
in is the natural mistake. That would make every MDM distance read **2x too long** —
filament would appear to advance twice as fast as it really is, and in T10's
differential check real buckling would look like healthy movement.

**If the disk is ever swapped, this number changes** — it is a config option, never a
constant:

| Disk | Per cycle | Edges/rev | `mdm_mm_per_count` |
|---|---|---|---|
| 3-segment | 5.8 mm | 6 | 2.90 mm |
| 5-segment (installed) | 3.5 mm | 10 | 1.75 mm |
| 10-segment | 1.73 mm | 20 | 0.865 mm |

**Quantization floor — this constrains T10's design.** One count is the finest movement
MDM can resolve: 1.75 mm on the installed disk, 2.90 mm on the 3-segment. So a
MDM-vs-Roadrunner comparison **cannot** work per-pulse; it needs a rolling window long
enough to accumulate several counts. At roughly 10 counts for +/-10% confidence that is
**~17.5 mm of travel** on the installed disk (~29 mm on the 3-segment). Note how far
this is from T8's `stall_mm = 0.5 mm`, which is fine only because the Roadrunner is
high-resolution — the MDM side needs thresholds an order of magnitude coarser, and a
window measured in tens of millimetres rather than in pulses.

Use 1.75 as the default rather than a placeholder. It is derived from the vendor
figure, not benched, so it still wants confirmation against the Roadrunner's `position`
over a long pull — but it is a real number with a known provenance.

**The vendor's "limited by the sampling rate of the printer firmware's filament
detection" warning does not apply to us.** That constrains host-side Klipper sensors
polling over USB. `track_edges()` is called from `buffer_task` (a `DECL_TASK`) with no
rate gate, so it samples at MCU main-loop rate — kHz+. At 1.75 mm/count even a wild
200 mm/s feed is only ~114 Hz, roughly three orders of magnitude of headroom. Counting
in firmware rather than pushing per-edge events is what buys this.

### IOEXT channel indices are a published contract — never renumber

T2 and T3 both remove rows from `ioext_def[]` in `src/buffer.c`. **Do not renumber the
remaining channels.** Channel indices are host-facing: they appear in
`LLL_IO_CFG CH=n` / `LLL_IO_READ CH=n` in users' configs and in the command manuals.
Renumbering would silently repoint an existing config at a different physical pin.

Leave a **documented hole** instead — keep the array slot with a comment saying the pin
was reclaimed and by which task, so ch6..ch11 keep their current indices. If the array
is index-addressed, the hole must still be a valid entry that refuses every operation
(treat it as permanently unavailable), not a gap that shifts later rows.

### Current state of the port

`klippy/extras/lll_load.py` exists (~380 lines, English). It implements arm-settle
arrival detection over the firmware's continuous auto-feed, plus a `present`-edge
unload. It compiles. **It is a first draft that this runbook substantially replaces** —
the arm-settle path survives only as a no-sensor fallback.

### Standing constraints

- Nothing here has been validated on hardware. Every distance and duration in code is
  a **placeholder** and must be commented as such.
- Never run gcode from a Klipper timer callback — hand off via
  `reactor.register_callback()`, which runs in a greenlet that may block.
- Bound every assisted motion with a deadline. The shipped command set only offers
  bounded moves on purpose.
- Do not commit unless the task says to. Do not push.

---

## Task graph

```
T1 inform removal ─┬─> T2 external buttons ─┐
                   └─> T3 MDM module ───────┼─> T4 protocol block + version
T5 translate firmware comments (independent) ┘
T6 rename host module ─> T7 backend seam ─> T8 load ─> T9 unload ─> T10 differential
T11 README ─ T12 repo standard files (independent, any time)
```

Firmware tasks T1–T5 and host tasks T6–T10 can proceed in parallel until T4/T8 meet.

---

## T1 — Delete `inform`

**Files:** `src/buffer.c`

**Why:** `inform` drives PA2/PA3 as outputs to signal a non-Klipper mainboard. Those
pins are needed as button inputs, and Klipper already gets `EV_BUTTON`. Relocating it
to PC13/PC14 was considered and rejected: those pins are not exposed, so the dumb
mainboard it exists to serve could never see them.

**Steps**
1. Remove `inform_en`, `inform`, `inform_ms` state and the `ext1`/`ext2` `gpio_out`
   handles.
2. Remove the `gpio_out_setup(EXT1_PIN, 0)` / `gpio_out_setup(EXT2_PIN, 1)` calls in
   `buffer_init` (~line 517-520) and the `EXT1_PIN`/`EXT2_PIN` defines.
3. Remove the `inform_ok` gating and all `gpio_out_write(ext1/ext2, …)` in the button
   handler (~718-746). **Keep the surrounding click/hold detection and the
   `ev_send(EV_BUTTON, …)` calls intact** — only the inform side effects go.
4. Remove the `BUTTONS` func key 1 (`inform`) param, its `PARAMS` entry in
   `klippy/extras/lll_buffer.py` (`'inform': (1, 1)`), and its row in the protocol
   block and docs.
5. Remove IOEXT ch4/ch5 special-casing (~line 541-546) that hands pins back to inform.
   Leave the channels themselves for now — T2 removes them.
6. Remove the Hi-Z restore cycle (~line 870-875) that re-establishes the inform
   outputs if an IOEXT cycle left them released.
7. Remove the `lll_param` BUTTONS key-1 handler (~line 964-969) that re-enables inform
   and re-runs `gpio_out_setup` on PA2/PA3.
8. Remove the `io_cfg` refusal (~line 1140-1141) that rejects claiming PA2/PA3 while
   inform is active — with inform gone there is nothing left to conflict with.

**The step list above is a guide, not an inventory.** `inform` is woven through more of
this file than a first read suggests. Drive the work from
`grep -n "inform\|ext1\|ext2\|EXT1_PIN\|EXT2_PIN" src/buffer.c` and do not stop
until it comes back empty. Deleting the declarations before the uses leaves the file
uncompilable, so work uses-first, declarations-last.

**Acceptance:** firmware compiles; no reference to `inform` or `EXT1_PIN`/`EXT2_PIN`
remains (`grep -rn "inform\|EXT1_PIN\|EXT2_PIN" src/ klippy/`); `LLL_SET
PARAM=inform` is gone from the host param table.

**Report:** anything that referenced inform beyond the list above.

---

## T2 — External Feed/Retract buttons on PA2/PA3

**Depends on:** T1

**Files:** `src/buffer.c`

**Why:** Vi has external feed/retract buttons on the exposed 3-pin header. They must
behave **identically to the internal KEY1/KEY2 buttons on PB12/PB13, including with no
host attached** — that is the buffer's standalone autonomy. Binding them host-side as
`[gcode_button]` would make them die whenever Klipper is down, unlike the internal
pair, so this belongs in firmware.

**Reference:** old fork defines, `buffer_manager_old/klippy_src/buffer.c:43-44`:
`KEY_EXTERNAL_RETRACT = PA2`, `KEY_EXTERNAL_FEED = PA3`.

**Steps**
1. Define `KEY_EXTERNAL_RETRACT` (PA2) and `KEY_EXTERNAL_FEED` (PA3), set up as inputs
   matching the internal keys' pull configuration.
2. Feed them into the **same** debounce/click/hold state machine as PB12/PB13 — OR the
   external read with the internal one at the point where `k1`/`k2` are sampled. Do
   **not** build a second parallel state machine: a click must mean the same thing
   whichever button produced it, including the double-click and hold behaviours.
3. Remove IOEXT ch4/ch5 (PA2/PA3) from `ioext_def[]`, and renumber or leave a
   documented hole — whichever keeps the channel-index contract with the host stable.
   State which you chose.
4. `EV_BUTTON` continues to report presses. Default to **not** distinguishing internal
   from external in the event payload, which keeps the existing wire format; note it
   if you find a reason to.

**Acceptance:** compiles; external and internal buttons produce identical firmware
behaviour and identical `EV_BUTTON` payloads; IOEXT channel count and indices are
consistent between `src/buffer.c` and `klippy/extras/lll_buffer.py`.

**Report:** the IOEXT renumbering decision, and whether the pull config on PA2/PA3
differs from PB12/PB13 in a way that needs bench confirmation.

---

## T3 — MDM module on PA4/PA5

**Depends on:** T1 (ordering only)

**Files:** `src/buffer.c`, `klippy/extras/lll_buffer.py`

**Why:** MDM sits on the buffer arm, at the *near* end of the long reverse bowden; the
Roadrunner sits at the far end. Together they bracket most of the filament path, which
is what makes buckling *inside* the bowden detectable. Canonical demoted these pins to
IOEXT, which is poll-only and useless in a control loop.

**Reference:** old fork used PA4 presence, PA5 motion, and did stall detection in
firmware — "no MDM encoder pulse for `JOB_STALL_MS` means filament isn't actually
moving, regardless of what the buffer motor is doing"
(`buffer_manager_old/klippy_src/buffer.c:614-616`).

**Steps**
1. Remove PA4/PA5 (ch0/ch1) from `ioext_def[]`; same renumbering note as T2.
2. Add an MDM module: PA4 presence input, PA5 motion input.
3. **Presence** → push an event on edge. Add `EV_MDM` (next free ev id) carrying the
   new level, and give it a `telem_mask` bit. Document the new id in the protocol
   block.
4. **Motion** → a **free-running `uint32` pulse counter**, incremented on the counted
   edge, exposed as a new field in the `lll_state` response. **Do not emit an event per
   edge** — an encoder pulse train would flood the serial link and starve other
   telemetry. The host diffs the counter between samples, exactly as it diffs the
   Roadrunner's `position`.
5. Counter wraps at `uint32`; the host must diff with wrap-safe unsigned arithmetic.
   Say so in a comment.
6. Do **not** port the old firmware-side stall latch. Stall policy lives host-side
   where it is tunable — see T10.
7. Update the `lll_state` format string in **both** `src/buffer.c` and
   `lll_buffer.py::_build_config` so they stay byte-identical to each other, and
   extend `_update_state` / `state` dict to carry the new fields.

**Acceptance:** compiles; `LLL_QUERY` reports MDM presence and a counter; the counter
advances when filament is pulled through by hand; format strings match on both sides.

**Report:** the chosen `EV_MDM` id and mask bit, the new `lll_state` format string, and
which edge (rising/falling/both) the counter counts.

---

## T4 — Protocol block and version bump

**Depends on:** T1, T2, T3

**Files:** `src/buffer.c` (protocol block ~37-99), `klippy/extras/lll_buffer.py`,
`docs/LLL_COMMANDS_MANUAL.en.md`, `docs/COMMANDES_LLL_MANUEL.fr.md`

**Steps**
1. Update the protocol block for: inform removal, the new `EV_MDM` id, the extended
   `lll_state` fields, and the revised IOEXT table.
2. Bump the protocol version so a host/firmware mismatch fails loudly rather than
   silently misparsing. If no version field exists, add one to the state response and
   have the host refuse to start on mismatch.
3. Update both command manuals to match. English is authoritative; French parity is
   nice-to-have, not blocking — if you cannot write it confidently, leave the French
   section stale and say so rather than guessing.
4. **Known-stale doc sites carried over from T1-T3, confirm each:**
   - Both manuals still reference the deleted `inform` param (flagged by T1).
   - `docs/LLL_COMMANDS_MANUAL.en.md` ~line 242 still says the button/arm/role events
     need `event_mask VALUE=127`, and its event-name list omits `mdm`. With `EV_MDM=7`
     the "everything on" value is now **255**, not 127. (The params table row at ~125
     and the firmware clamp at `command_lll_param` were already corrected — the clamp
     had been `val & 0x7F`, which silently made bit 7 unreachable and `EV_MDM` a dead
     event. If you add any further `EV_` id, widen that clamp again.)

**Acceptance:** every command/event/param in the protocol block matches the code;
host refuses a mismatched firmware with a clear message.

**Report:** the version scheme chosen, and any doc section left stale.

---

## T5 — Translate firmware comments to English

**Independent** — but coordinate, it touches the same files as T1–T4. Prefer running it
**after** T4 to avoid conflicts.

**Files:** `src/buffer.c`, `klippy/extras/lll_buffer.py`

**Steps**
1. Translate French comments to English, preserving meaning precisely. These comments
   encode hard-won bench findings ("confirmé RE binaire", "fix #4", "bench A9") —
   keep those markers and their references intact.
2. Do **not** rename identifiers, change logic, or reflow code. Comments only.
3. Where a comment documents a safety invariant (grace period, bounded moves, shutdown
   handling), keep it verbatim in meaning — these are the reasons the firmware is safe.

**Acceptance:** `git diff` shows comment-only changes; firmware compiles and the binary
is functionally unchanged.

**Report:** any comment whose meaning you could not translate confidently — leave the
French in place and flag it rather than guessing.

---

## T6 — Rename the host module

**Files:** `klippy/extras/lll_load.py` → `klippy/extras/buffer_manager.py`

**Why:** the board may change (the LLL stepper is small and its TMC UART is bitbanged;
a BTT EBB42 or similar is a possibility). Config should not be tied to the LLL board.

**Steps**
1. `git mv klippy/extras/lll_load.py klippy/extras/buffer_manager.py`.
2. Section becomes `[buffer_manager <name>]`; class `BufferManager`;
   `load_config_prefix` unchanged in shape.
3. Commands `LLL_LOAD_*` → `BUFFER_LOAD_*`, `LLL_UNLOAD*` → `BUFFER_UNLOAD*`. Keep the
   mux key as `LANE`.
4. The config option pointing at the buffer lane stays `lane:` and still resolves to an
   `lll_buffer` object for now.

**Acceptance:** `python -m py_compile klippy/extras/buffer_manager.py`; no `lll_load`
references remain.

---

## T7 — Backend seam

**Depends on:** T6

**Files:** `klippy/extras/buffer_manager.py`

**Why:** so a future EBB42-class board is a new backend rather than a rewrite. Build
the seam now; implement **only** the LLL backend.

**Steps**
1. Define a narrow lane-driver interface: `arm()` → bitfield, `jog(dir, ms)`,
   `run(dir, ms)`, `stop()`, `state()` → dict, `mdm()` → `(present, counter)`.
2. Implement `LllBufferBackend` wrapping `lll_buffer`, driving it through its **public**
   surface (its `LLL_*` gcode commands and `get_status`) wherever practical.
3. The pipeline code must touch only the interface, never `lll_buffer` internals.
4. Do not add a second backend. Do not add config for backend selection beyond what a
   single implementation needs.

**Acceptance:** compiles; pipeline code has no direct `lll_buffer` attribute access.

**Report:** any place the public surface was insufficient and an internal had to be
touched — that is a candidate for a firmware/host API addition.

---

## T8 — Three-phase load

**Depends on:** T7, and T3/T4 for MDM fields

**Files:** `klippy/extras/buffer_manager.py`

**Why:** the current draft rides the firmware's **continuous** auto-feed, which is the
approach upstream specifically found fragile: load must be pulsed, "because a
continuous push overshoots: buckling at any unguided gap in the path is the failure
mode of *pushing*" (`docs/MULTICOLOR_STATUS.en.md:47-51`).

**Preconditions to enforce:** the lane's tool must be picked up and active — refuse
otherwise, no motion on a docked tool. Phase A is cold-safe. Phase B requires the
extruder above `min_extrude_temp` (Klipper blocks E-moves cold); fail at the A→B
boundary with a clear message, not mid-move.

**Phases**
- **A — PUSH** (buffer alone, cold-safe). Pulsed `jog(fwd, pulse_ms)` on a 100 ms tick
  — matching the Roadrunner's own refresh rate, so one pulse per sample. Ends on
  Roadrunner `filament_detected`. Each tick diffs Roadrunner `position`; a pulse that
  commands movement but produces no encoder delta is buckling or slip — fail fast
  rather than burning the deadline pushing into a jam. Accumulate mm-per-ms so the
  module **learns its own feed rate** instead of needing it benched.
- **B — ASSIST** (needs heat). `run(fwd, …)` concurrent with a bounded
  `G1 E<assist_mm> F<assist_speed>` — the extruder must not fight the whole path alone.
  Ends when the **toolhead switch** confirms the gear actually grabbed.
  `underextrusion_detected` and measured-vs-expected distance are the slip
  discriminators here.
- **C — HANDOFF.** `stop()` the lane so firmware autonomy resumes per role, then run
  `loaded_gcode` via `reactor.register_callback`.

**Config:** `roadrunner_sensor`, `post_extruder_pin`, `extruder`, `pulse_ms` (100),
`assist_mm`, `assist_speed`, `stall_mm` (minimum encoder delta per pulse), timeouts,
and the gcode templates. **Every distance/duration default is a placeholder — comment
each as such.**

**Keep from the draft:** phase scaffolding, the `register_callback` handoff,
`_exec_template`, the `event_mask` raise at ready (EV_ARM is bit 5, excluded from the
`0x0F` default), and the `event_gcode` bridge command. **Keep arm-settle as a
no-sensor fallback.**

**Post-extruder switch:** raw pin via `buttons.register_buttons`, reusing the pattern
at `buffer_manager_old/klippy_extras/buffer_manager.py:156-172`.

**Acceptance:** compiles; with no sensors configured the arm-settle fallback still
works; docked tool refuses; cold extruder fails cleanly at the A→B boundary.

**Report:** the learned-feed-rate approach, and every value you had to invent.

---

## T9 — Unload

**Depends on:** T8

**Files:** `klippy/extras/buffer_manager.py`

**Why:** upstream's unload needed a post-splitter detector and a calibrated tail
(measured detector→splitter distance + margin) because filament had to park clear of a
shared junction. **There is no splitter here** — 1:1 lanes — so both of those phases
disappear and the calibration with them.

**Sequence**
1. Optional `tip_forming_gcode` — there is no cutter, so the tip is shaped, not cut.
2. Synchronized retract: `G1 E-<n>` while `run(back, …)`. Upstream's finding: an
   extruder pulling alone against the whole path slips; with the buffer actively
   following, the tip clears reliably.
3. Continue until the **toolhead switch** clears, then until Roadrunner
   `filament_detected` goes false.
4. Retract past the extruder entry by an **encoder-measured** distance. The old fork's
   `gear_to_mdm_mm: 60` guess
   (`buffer_manager_old/klippy_macros/buffer_manager.cfg:66`, itself marked
   `TODO MEASURE`) is retired — measure it, do not port it.
5. `stop()`, then `unloaded_gcode`.

**Acceptance:** compiles; failure at any phase runs `unload_failed_gcode` with a reason
and leaves the lane stopped.

---

## T10 — MDM/Roadrunner differential monitoring

**Depends on:** T8, T3

**Files:** `klippy/extras/buffer_manager.py`

**Why:** MDM and Roadrunner bracket the long reverse bowden. MDM counter advancing
while Roadrunner `position` does not means filament is moving at the buffer but not at
the toolhead — i.e. **buckling or compressing inside the bowden**, which neither sensor
detects alone. This is the capability the two-encoder layout exists to provide.

**Steps**
1. During assisted motion, sample both and compare deltas over a rolling window.
2. Divergence beyond a configurable tolerance → fail the phase with a distinct reason
   naming buckling, separately from a plain stall.
3. Wrap-safe unsigned arithmetic on the MDM counter.
4. MDM counts encoder pulses, the Roadrunner reports mm. **The scale factor is now
   derived — see "MDM encoder scale" in §0.** Default `mdm_mm_per_count` to **1.75**
   (5-segment disk, 3.5 mm per cycle, both edges counted), documented as vendor-derived
   and pending bench confirmation rather than as an invented placeholder. Keep it a
   config option so a different disk can be swapped in, and still have the check
   disable itself with a logged warning if it is explicitly zeroed out.

**Acceptance:** compiles; the check is inert unless explicitly configured.

**Report:** the calibration procedure a human should follow to find the scale factor.

---

## T11 — README

**Files:** `README.md`

**Steps:** rewrite for this fork — the hardware stack with links (Filamentalist V3:
`https://github.com/Carrot-collective/Filamentalist_Rewinder/tree/main/Filamentalist_FV3_Rewinder`;
Roadrunner: `https://github.com/EiNSTeiN-/roadrunner-filament-sensor/`; Mellow LLL Fly
Plus buffer + Mellow MDM), the filament path diagram from §0, the pin map table, why
the upstream blockers do not apply to a 1:1 toolchanger with passive rewinders, and
clear attribution to Nitrooxyde's Canonical as the base with a link.

Per Vi's repo standard, include a `## Features` checkbox list and a `## TODO` section
immediately after `## Contents`.

**Report:** anything asserted about hardware you could not verify from the repos.

---

## T12 — Repo standard files

**Files:** `NOTES.md`, `CLAUDE.md` (new)

The fork has neither. `CLAUDE.md` carries project ground rules, gate commands
(`python -m py_compile`, the firmware build), and the conventions in §0. `NOTES.md` is
Vi's inbox: dated entries, newest first, struck through when acted on.

---

## Validation gates

Every task, before reporting done:

```bash
cd c:/git/github/buffer_manager
python -m py_compile klippy/extras/*.py
git status --short          # nothing unexpected
git diff --stat             # scope matches the task
```

Firmware tasks must additionally build. **No task can be hardware-validated** — say so
explicitly rather than implying a change is proven.

## Reporting back

Each agent returns: what changed (files + summary), anything in §0 found to be wrong,
every invented value, and anything left undone. Vi validates before the next dependent
task starts.
