# Multicolor / automatic filament change — status and path for equipped setups

> French version: [MULTICOLOR_STATUS.fr.md](MULTICOLOR_STATUS.fr.md)

## Where this project stands today

On the reference setup, color changes are done **manually** (park the lane,
rewind with the physical buttons, activate the next lane — see the commands
manual). The reason is hardware, not firmware: our setup currently has
**no automatic filament cutter** and **no passive respooler** (e.g.
Filamentalist-style rewinders). Without those two pieces, a fully automatic
change has to deal with a molten filament tip (mushroom shaping, hot pulls)
and with loose retracted filament — and that made our unload procedures far
more complex and fragile than they should be.

**If your setup has a toolhead cutter and passive rewinders, the automatic
path is realistic with what this repo ships** — we prototyped it end to end
and it worked; here is what we learned.

## The architecture we prototyped (and why it is not shipped as official)

Two sensors and **one measured number** are enough:

- a **presence sensor at the extruder entry** (many extruders have one
  built in);
- a **filament detector after the splitter/merger** (ours was a BTT SFS 2.0
  on a Pico RP2040 flashed as a USB Klipper MCU — presence + motion);
- the **measured path length between that detector and the splitter exit**.
  That single distance is the whole calibration.

### Unload (3 sequenced phases)

1. **Synchronized retract** — the extruder retracts while the buffer lane
   runs `back` at its *real measured* speed (measure it: ours ran ~1.9× the
   spec figure). This matters: an extruder pulling alone against the whole
   path slips; with the buffer actively following, the tip cleared the
   extruder-entry sensor reliably.
2. **Continue until the post-splitter detector goes clear** — the tip has
   left the common section.
3. **Calibrated tail** — retract a fixed extra length = *measured
   detector→splitter-exit distance + margin* (we measured 200 mm and used
   220 mm), parking the filament tip just outside the splitter. The lane is
   now free for the next color.

### Load

Pulsed feeding (short bounded jogs, ~100 ms, host-side tick) until the
extruder-entry sensor sees filament, then hand over to the extruder. Pulsed,
because a continuous push overshoots: buckling at any unguided gap in the
path is the failure mode of *pushing* (different from the slip you fight
when *pulling*).

### Lessons that survived the teardown

- The **motion output** of the detector is readable even when the sensor's
  runout logic is disabled — it is your slip-vs-advance discriminator during
  any assisted move.
- Every assisted motion must be **bounded** (deadline) and observable; the
  shipped command set only offers bounded moves (`LLL_JOG`, `LLL_RUN`) on
  purpose.
- A **parked** lane never auto-feeds — park-first is the safe manipulation
  workflow, automatic or manual.

### Why we withdrew it

Our installation accumulated **slip/friction problems** in the filament path
(a badly angled sensor mount adds real drag; repeated slip episodes chew the
filament, which then drags and jams everywhere — a vicious circle). And
**without a cutter**, tip shaping turned every unload into a long, fragile,
hot procedure. Rather than publish something that only worked on a good day,
we rolled the printer back to the simple, validated manual workflow and kept
the automatic sequence out of the official feature set.

With a **cutter** (clean square tip, no shaping) and **passive respoolers**
(the retracted filament winds itself back), phases 1–3 above lose their
failure modes almost entirely. The shipped primitives are the building
blocks: per-lane roles, bounded `LLL_RUN`/`LLL_JOG`, telemetry events with
`event_gcode`, `LLL_FILAMENT` identity, `LLL_DETECT_LOADED` probing.

If you build this on your equipped setup, measure your detector→splitter
distance, keep every move bounded, and watch the motion signal — that is the
entire method.
