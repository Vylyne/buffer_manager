<!--
DO NOT READ THIS FILE unless Vi explicitly asks for it by name or asks what's
in the backlog. It is not part of the standard "read NOTES.md/CLAUDE.md every
session" set, and it is not something to act on unprompted.
-->

# BACKLOG.md

Ideas parked for later — not committed to, not scheduled. Newest first.

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
