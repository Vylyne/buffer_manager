# Changelog

## v1.0.0 — "Canonical" (runtime-toggleable) — 2026-07-02

The autonomous smart-buffer core is unchanged (standalone-parity: same hall
conventions, same TMC2208 `VACTUAL` drive, same feed behaviour). Everything
below is added **on top** of it.

### Added
- **Full Klipper host control** via a new extra (`klippy/extras/lll_buffer.py`):
  21 G-code commands, multiplexed per lane (`LANE=`), N-scalable — one
  `[mcu]` + one `[lll_buffer]` section per board.
- **Roles per lane**: `active` / `parked` / `standby` — a parked lane never
  auto-feeds forward (multi-lane safety), `LLL_SET_IDLE` kept as a
  backwards-compatible alias.
- **4 feed automation levels**: `auto` / `semi_auto` (one-shot `LLL_ARM`) /
  `semi_manual` / `manual`.
- **11 runtime-toggleable functions** (`LLL_ENABLE`): FEED, BUTTONS, MBSIG,
  ENTRANCE, TMC, MOVE, DIAG, LED, ADC, TELEM, IOEXT — one flash, everything
  selectable at runtime.
- **Bounded motion commands**: `LLL_JOG` (≤10 s) and `LLL_RUN` (deadline,
  refreshable) — no unbounded motion order exists in the command set.
- **Runtime TMC tuning** (`LLL_SET`): current, microsteps, stealthChop,
  timeouts… persisted and replayed at every boot.
- **Named profiles**: `LLL_PROFILE_SAVE/LOAD/LIST/DELETE`.
- **Telemetry events** (`lll_event`) with configurable `event_gcode` hook
  (react to jams/runout/diag edges from printer.cfg in ~20–50 ms).
- **IOEXT**: 12-channel index-only GPIO/PWM/ADC register for the spare
  connectors — critical pins are *physically unreachable* (not in the table);
  unverified pins gated behind `LLL_UNLOCK`.
- **Filament identity & probing**: `LLL_FILAMENT` (type/color/temp/spool)
  and `LLL_DETECT_LOADED` (detects which lane is physically engaged in the
  extruder using only the buffer arm sensors).
- **Web dashboard** (`dashboard/`): self-contained HTML page, Moonraker
  WebSocket, lane auto-discovery, VCR-style transport with burst-bounded
  continuous mode — shown inside Mainsail as an iframe webcam card.
- **Spoolman integration** (optional): active lane drives Moonraker's active
  spool.
- **LCD menu example** (`config/lcd_menu_lll.cfg.example`): per-lane state +
  Activate / Park / Clear error.
- **Prebuilt binary** `build/klipper_canonical_v1_fix9.bin` (53 824 bytes,
  build `20260705_234145`, sha256
  `9239bdc367096438e72528d361f75f1ebb811784698cb0872ca79b2410837944`) — the
  exact binary running in production on the reference setup.

### Safety hardening (8 fixes, all in the shipped code)
1. `sched_is_shutdown()` guard as the literal first line of the buffer task,
   plus `DECL_SHUTDOWN` driving the motor enable high — **bench-proven**:
   `M112` during a commanded run cuts the motor dead.
2. Single-writer: command handlers set flags only; only the task and the
   shutdown handler touch EN/UART.
3. Forward run requires filament present && no latched error (checked every
   tick); backward run allowed on runout (legitimate unload).
4. `back_timeout` (default 60 s) on parked follow and commanded back runs.
5. USB/SWD/UART/hall/button pins and the 5 V headers are outside the IOEXT
   table — index-only addressing makes them unreachable by construction.
6. Every stop writes `VACTUAL=0` **before** disabling; every motion rewrites
   GCONF+VACTUAL.
7. Roles/modes/enables are replayed in Klipper's config sequence (before
   `ready`), closing the boot window on Klipper restarts (`role: parked`).
8. No UART bit-bang inside command handlers; TMC writes are deferred and
   spaced in the task.
9. Boot grace period: after a hard MCU reset (`FIRMWARE_RESTART`, `M112`,
   power-cycle), no autonomous feed starts until the host has spoken once
   (20 s cap; done-latch evaluated every tick so it can never re-arm) —
   closes the phantom-push window of the compiled defaults. Field-proven:
   `FIRMWARE_RESTART` with a fully-tensioned arm → zero motion. Deliberate
   standalone trade-off: a never-host-driven board starts auto-feeding 20 s
   after power-on (stock Mellow starts ~1 s after boot).

### Changed vs the predecessor project
- `src/buffer.c` grows from the single-purpose autonomous port (~490 lines)
  to the canonical runtime-toggleable implementation (~1550 lines), protocol
  shared verbatim with the host extra.

### Compatibility
- Same board (Mellow FLY LLL Buffer Plus, STM32F072CB), same Katapult
  bootloader layout (8 KiB offset), same flash procedure.
- Defaults preserve standalone behaviour: a board that is never spoken to
  feeds exactly like the v1.x firmware (buttons included).

## Lineage

This project supersedes
[Mellow-LLL-Plus-Klipper-Firmware](https://github.com/Nitrooxyde/Mellow-LLL-Plus-Klipper-Firmware)
(2026-05-24), whose autonomous smart-buffer core — Mellow's standalone
auto-feed ported into a Klipper MCU task, single USB cable — is embedded
here **unchanged**. That repo remains valid for a minimal, autonomous-only
build. Bootloader install without opening the case:
[LLL-Plus-Katapult-NoDFU](https://github.com/Nitrooxyde/LLL-Plus-Katapult-NoDFU).
