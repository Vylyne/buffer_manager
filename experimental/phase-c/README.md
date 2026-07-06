# Phase C (v1.1) — experimental, informational only

> **Status: NOT official. Unfinished project.** This folder exists for
> transparency and feedback — it is *not* part of the supported firmware.
> The official, field-validated firmware of this repository is the canonical
> v1 in `src/` + `klippy/extras/`. Use this one at your own risk.
>
> **Update:** the grace-period half of Phase C has since been re-derived,
> hardened (tick-level done-latch, so the grace can never re-arm) and
> **integrated into the official firmware as safety fix #9** — see the main
> README. What remains unique to this folder is the `rewind_pause` option
> and its post-mortem.

## What it is

"Phase C" is a 46-line evolution of the canonical firmware (diff verified
line by line against the canonical source). Exactly three changes:

1. **Boot grace period** — a `host_ever` flag plus a 20 s cap: after a hard
   MCU reset (M112, power-cycle), the board starts **no autonomous feed**
   until Klipper has spoken to it once, or 20 s have elapsed. Buttons, jog,
   run and roles stay fully active during the grace. Rationale: `role:
   parked` in printer.cfg covers Klipper restarts but *not* a hard MCU
   reset, where the board runs a few seconds on its compiled defaults
   (active/auto) before the config arrives.
2. **`rewind_pause` opt-in parameter** (BUTTONS, default **0** = strictly
   canonical behaviour) — after a manual rewind (button hold), the lane
   latches a pause (`ecode 6`) instead of auto-feeding forward again; a
   short button click or `LLL_CLEAR_ERROR` re-arms. This reintroduces the
   stock Mellow "pause after rewind", as an option.
3. Removal of one dead condition (cleanup, no behaviour change).

The host extra in this folder (`lll_buffer.py`) is the matching version
(adds `ecode 6` decoding and the `rewind_pause` parameter). **Always deploy
the pair together.**

## Why it was withdrawn

- The code was adversarially reviewed before flashing (0 critical, 0 major)
  and ran on the reference setup. But **`rewind_pause=1` on an *active* lane
  proved to be a footgun in real use**: the pause is implemented as a
  latched error, and a latched error freezes the lane's entire auto-feed —
  during a print this leaves the buffer arm fully tense while the extruder
  pulls alone, i.e. grinding. This failure mode was hit and traced in the
  field. Left at its default (0) the option is inert, but the safe redesign
  (inhibit only the auto re-forward, without latching an error) was never
  built.
- The reference setup was rolled back to the canonical build, which remains
  the only version we stand behind.

Known minor issues from the pre-flash review, for anyone experimenting:
the grace period also gates the parked back-follow during those seconds
(harmless: stop = freewheel); a 49.7-day timer wrap re-arms one grace on a
board that has *never* been host-driven; a `rewind_pause` latch triggered by
a mainboard-signal hold can only be cleared by button/host; the latch also
arms in `parked` role (over-broad but clearable).

## Files

| File | What | Fingerprint |
|---|---|---|
| `buffer.c` | v1.1 MCU source (exact snapshot that was built & flashed on the reference setup) | md5 `b962f9f5c9802fc8047797792648d49e` |
| `lll_buffer.py` | matching host extra | md5 `54e1482d4aa1129ca68e1c072a3030a5` |
| `klipper_phaseC_v11.bin` | prebuilt binary, 53 936 bytes, build `20260704_211838` | sha256 `0adb0db396ca8293cbb5dbd0be3bf5101c669c2edfa54ccfdb9a5901bca77018` |

## Trying it / reverting

Install exactly as in the main README, substituting this folder's
`buffer.c` and `lll_buffer.py`. To revert: flash
`build/klipper_canonical_v1_fix9.bin` (or rebuild from `src/buffer.c`) and
restore the main `klippy/extras/lll_buffer.py`, then `FIRMWARE_RESTART`.

Feedback (issues, test reports on other setups) is welcome — that is the
whole reason this folder exists.
