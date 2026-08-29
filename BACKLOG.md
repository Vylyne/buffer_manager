<!--
DO NOT READ THIS FILE unless Vi explicitly asks for it by name or asks what's
in the backlog. It is not part of the standard "read NOTES.md/CLAUDE.md every
session" set, and it is not something to act on unprompted.
-->

# BACKLOG.md

Ideas parked for later — not committed to, not scheduled. Newest first.

## 2026-08-26 — Safe redesign of rewind-then-pause

`experimental/phase-c/` prototyped a `rewind_pause` option (pause auto-feed
after a manual rewind, re-armed by a button click or `LLL_CLEAR_ERROR`) and
withdrew it: the implementation latched a full `is_error`, which freezes the
lane's *entire* auto-feed — on an actively-printing lane that means the
extruder keeps pulling while the buffer stops following, i.e. grinding. The
folder's own README already names the fix that was never built: inhibit
only the automatic re-forward after a rewind, without latching an error that
also blocks the print-feed path. That's a real firmware design task (a new,
narrower state distinct from `is_error`), not a mechanical port — parking it
here rather than adopting phase-c's code as-is. See
`docs/REVIEW_2026-08-26.md`'s phase-c finding for the full trace (it also
predates this fork's T1–T5 rework entirely, so nothing else in that folder
carries over cleanly regardless).

## 2026-08-22 — Dashboard: MDM + load/unload

The dashboard (`dashboard/index.html`) still works as upstream shipped it —
`LLL_CLEAR_ERROR`, `LLL_DETECT_LOADED`, `LLL_FILAMENT`, `LLL_JOG`, `LLL_ROLE`,
`SET_ACTIVE_LANE` are all untouched by the fork's changes. But it has two gaps
relative to what this fork now has:

- No MDM readout. `lll_state` carries `mdm_present`/`mdm_count` (T3), and the
  differential buckling check (T10) is built entirely around them — none of
  that is visible in the UI, only via `LLL_QUERY` / logs.
- No load/unload controls. `buffer_manager.py`'s `BUFFER_LOAD_BEGIN` /
  `BUFFER_UNLOAD` / abort commands aren't wired into the page at all — the
  dashboard only knows about the lane, not the pipeline sitting on top of it.

Neither is broken, both are additions. Worth doing if the pipeline proves out
on the bench and the dashboard becomes the place to watch it work.
