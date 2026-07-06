# The project story — multicolor with just buffers

> French version: [THE_PROJECT_STORY.fr.md](THE_PROJECT_STORY.fr.md)

## The objective

Print **multicolor / multi-material on a single direct-drive toolhead,
without a selector MMU** — no ERCF, no BoxTurtle, no AMS, no toolchanger.
Just **one cheap Mellow LLL Plus buffer per filament lane**, merged by a
passive splitter into the printer's single filament path, with Klipper
orchestrating which lane is active.

Why that shape:

- **Modular and plug-and-play**: every lane is one board on one USB cable
  (+ 24 V). Adding a lane = one `[mcu]` + one `[lll_buffer]` section. No
  selector mechanics, no gate calibration, no cart.
- **The buffers were already needed anyway**: on the reference setup the
  printer is fed **externally**, with technical filaments living in a
  **dryer** — long feed paths need active buffering regardless of
  multicolor.
- **A lane that is `parked` never pushes** — the firmware guarantees it.
  Swapping colors is: park the current lane, rewind it clear of the
  splitter, activate the next one.

![Reference rig](img/architecture_tested.svg)

## What was proven

- **Two lanes ran in production** on the reference setup: real prints with
  color changes, the manual/semi-manual swap workflow, per-lane roles,
  runout → pause, the dashboard and the LCD menu. That is what this
  repository ships.
- **The fully automatic unload/load pipeline was prototyped end-to-end and
  worked**: synchronized retract (extruder + buffer running back together —
  an extruder pulling alone against the whole path slips), phases sequenced
  by sensors (extruder-entry clear → post-splitter detector clear), then a
  **calibrated tail** = the measured detector→splitter distance + margin,
  parking the filament tip just outside the merge. Loading was pulse-fed
  until the extruder-entry sensor confirmed arrival. See
  [MULTICOLOR_STATUS.en.md](MULTICOLOR_STATUS.en.md) for the method.

The single most important lesson: **nothing about loading/unloading has to
be guesswork**. A sensor at a *determined* position plus one *measured*
distance turns "hope the tip is out" into a deterministic, repeatable
sequence. Our SFS placed at a known point after the splitter, with the
SFS→splitter distance measured once, made unload lengths exact. The
extruder-entry sensor (Smart Orbiter) made load completion a confirmed
event instead of a timed push.

## What held us back — equipment, not concept

We withdrew the automatic pipeline from the official feature set, and the
honest reason is **missing hardware**, not the architecture:

| Missing piece | Consequence in our tests |
|---|---|
| **No filament cutter at the toolhead** (XOL toolhead, cutter mod not yet mounted) | Every unload had to shape and drag a **molten tip** (mushroom) back through the whole path — long, hot, fragile procedures with many failure modes |
| **No passive respoolers** (Filamentalist-style rewinders) | Retracted filament had to be managed by the lane buffers alone; unload/store of the freed lane stayed an open problem |
| **No motion-measuring wheel on the path** (encoder-class feedback) until late | Early load/unload ran open-loop; only when the SFS arrived did we get a slip-vs-advance signal — the single most useful diagnostic we found |
| **Accumulated path friction** (a badly-angled sensor mount, coupler drag, long PTFE runs from the external dryer) | Slip episodes chewed the filament; a chewed filament drags and jams everywhere, which poisoned every subsequent test |

Under those conditions the automation stack grew complicated — compensating
mechanics with software until the machine could not even print single-color
reliably. We rolled everything back to the clean, validated core (this
repo), and kept the pipeline as documentation.

## The honest conclusion — it is absolutely feasible

**With proper equipment, LLL-only multicolor is a sound, reachable
target.** What it takes is a *clean installation* and *sensors that remove
chance* from loading and unloading:

![Target rig](img/architecture_target.svg)

- **A cutter at the toolhead** → clean square tip, no molten-tip shaping:
  the unload phases lose almost all their failure modes.
- **Passive respoolers per spool** → the retracted filament winds itself
  back; the freed lane is genuinely stored.
- **A filament sensor at a determined position after the splitter**, with
  the splitter distance **measured once** — proven on our rig: it makes the
  tail length exact.
- **Reinforce it with a second sensor before the splitter**: two sensors
  *bracketing* the merge confirm the tip hand-off in **both** directions —
  load and unload become sensor-confirmed, never distance-guessed.
- **An extruder-entry presence sensor** (Smart Orbiter class) validates
  that the filament actually reached the extruder.
- Per-lane entrance sensors + the buffer arm telemetry cover the rest.

All the firmware primitives those sequences need are already in this
repository: per-lane roles, bounded moves (`LLL_JOG`/`LLL_RUN`), telemetry
events with `event_gcode`, filament identity (`LLL_FILAMENT`), physical
lane probing (`LLL_DETECT_LOADED`).

It is a project of **small steps and details** — measured distances,
determined sensor positions, bounded moves, one change at a time. Every one
of those details is cheap; skipping them is what gets expensive.
