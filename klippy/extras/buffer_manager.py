# Mellow LLL Plus buffer — host-side load/unload pipeline
#
# Adds automatic "filament reached the extruder" detection and a bounded
# unload on top of the canonical lll_buffer lane. This module is strictly
# additive: it drives lll_buffer through its public surface only (via the
# LaneBackend seam below), so both src/buffer.c and
# klippy/extras/lll_buffer.py stay byte-identical to upstream and no
# firmware change is required by this file.
#
# Load pipeline (T8) — three phases, pulsed, not continuous
# -----------------------------------------------------------------------
# Upstream's own retired draft found that riding the firmware's continuous
# auto-feed is fragile: a continuous push overshoots and buckles at any
# unguided gap in the path (docs/MULTICOLOR_STATUS.en.md:47-51). This
# module instead pulses the lane forward in short bounded jog()s on a
# ~100 ms tick — matching the Roadrunner filament sensor's own internal
# refresh rate (CHECK_RUNOUT_TIMEOUT = .100s), so one pulse maps to one
# fresh sample, not faster or slower than the sensor can actually report.
#
#   Phase A — PUSH     buffer alone, cold-safe. Pulsed jog() toward the
#                       Roadrunner. Ends on Roadrunner filament_detected.
#                       Learns its own mm-per-ms feed rate as it goes.
#   Phase B — ASSIST    needs the extruder above min_extrude_temp. Bounded
#                       run() concurrent with a bounded G1 E<assist_mm>.
#                       Ends when the post-extruder (toolhead) switch
#                       confirms the gear actually grabbed.
#   Phase C — HANDOFF   stop() the lane (firmware autonomy resumes per
#                       role/mode) then run loaded_gcode via
#                       reactor.register_callback.
#
# If roadrunner_sensor / post_extruder_pin are not configured, this module
# falls back to the original arm-settle heuristic below, watching the
# firmware's own continuous auto-feed via the dancer-arm hall sensors.
# That path is unchanged by T8 — it exists for lanes with no extra sensors
# wired up yet.
#
# Unload pipeline (T9) — mirrors the load pipeline's structure
# -----------------------------------------------------------------------
# Upstream's retired unload needed a post-splitter detector and a
# calibrated tail (measured detector->splitter distance + margin) because
# filament had to park clear of a shared junction. This layout has no
# splitter - six lanes, six toolheads, 1:1 - so both the extra detector
# and the calibration disappear with it.
#
#   1. tip_forming_gcode (optional)   no cutter exists, so the tip is
#                                      shaped rather than cut.
#   2/3. RETRACT   pulsed G1 E-<unload_pulse_mm> concurrent with
#                  backend.run("back", pulse_ms) - upstream's finding
#                  (same as phase B) is that the extruder pulling ALONE
#                  against the whole path slips; the buffer actively
#                  following makes the tip clear reliably. Ends once the
#                  post-extruder (toolhead) switch clears AND the
#                  Roadrunner's filament_detected goes false. Each pulse
#                  diffs Roadrunner position, same stall discriminator as
#                  phase A.
#   4. CLEAR       a small bounded number of further pulses past the
#                  extruder entry. By this point the Roadrunner has
#                  nothing left threading its encoder wheel, so a lack of
#                  further movement here is the *expected* steady state,
#                  not a fault - see _run_unload_clear(). This replaces
#                  the old fork's blind, unmeasured gear_to_mdm_mm=60
#                  guess with a small bounded fallback instead of another
#                  fixed calibrated distance.
#   5. stop() then unloaded_gcode.
#
# Lanes without roadrunner_sensor/post_extruder_pin configured fall back to
# the original pre-T8 behaviour: LLL_RUN back, waiting for the firmware's
# own entrance-switch `present` edge to go low. That path is unchanged by
# T9 - see cmd_UNLOAD.
#
# DIFFERENTIAL MONITORING (T10). The MDM sits on the buffer arm, the
# Roadrunner near the toolhead; they bracket the long reverse bowden, so
# comparing them reveals filament advancing at one end but not the other -
# which neither sensor can detect alone. _diff_check() runs in phase B and
# in the unload synchronized retract - the two windows where filament
# actually threads BOTH sensors. It deliberately does NOT run in phase A:
# that loop ends when filament first reaches the Roadrunner, so throughout
# it the Roadrunner reads a constant zero and there is nothing to compare
# against. Phase A measures with the MDM alone.
#
# It is windowed, NOT per-pulse: one MDM count is mdm_mm_per_count (1.75mm
# by default), the finest movement it can resolve, so within a single
# ~100ms pulse the counter usually has not ticked at all and a per-pulse
# comparison would false-positive constantly.
#
# It reports a DISAGREEMENT, never "buckling" specifically. Both encoders
# are friction-coupled and either can slip; if they slip together this
# sees nothing. Defaults to warn rather than error for that reason.
#
# NOTHING IN THIS FILE HAS RUN ON HARDWARE. Every distance, duration and
# threshold introduced for the sensor pipeline is an explicitly-commented
# placeholder — see the config option comments in BufferManager.__init__.
#
# How it hears about the dancer arm
# ---------------------------------
# buffer.c pushes an EV_ARM event on every arm transition carrying the raw
# hall bitfield. lll_buffer dispatches firmware events to its event_gcode
# hook, so the wiring is a one-line bridge in printer.cfg:
#
#   [lll_buffer lll]
#   mcu: lll
#   event_gcode:
#       BUFFER_LOAD_EVENT LANE=lll NAME={event.name} VALUE={event.value}
#
#   [buffer_manager lll]
#   #lane: lll                 # lll_buffer section to watch (default: name)
#   #roadrunner_sensor:        # [high_resolution_filament_sensor <name>],
#                               # unset together with post_extruder_pin to
#                               # use the no-sensor arm-settle fallback
#   #post_extruder_pin:        # raw switch pin, after the extruder gear
#   #extruder: extruder        # extruder object this lane's E-moves target
#   #pulse_ms: 100
#   #assist_mm: 30
#   #assist_speed: 300
#   #stall_mm: 0.5
#   #load_settle_time: 1.5     # no-sensor fallback only
#   #load_timeout: 180
#   loaded_gcode:
#       M117 filament at extruder
#   #load_failed_gcode:
#   #tip_forming_gcode:        # optional; no cutter, so the tip is shaped
#   #unload_pulse_mm: 2.0      # sensor pipeline only
#   #unload_retract_speed: 900 # sensor pipeline only
#   #unload_clear_mm: 15       # sensor pipeline only
#   #unload_clear_pulses: 10   # sensor pipeline only
#   #unload_timeout: 90
#   #unload_run_ms: 0          # no-sensor fallback only
#   #unloaded_gcode:
#   #unload_failed_gcode:
#
# EV_ARM is gated by the firmware's telemetry mask, which defaults to 0x0F
# (bits 0-3) and therefore does NOT include arm (bit 5). This module raises
# the mask to 0x2F at ready, every boot, so the wiring cannot silently be
# half-configured.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

logger = logging.getLogger(__name__)

# Load state machine.
#
# The dancer arm tells us where the filament is (buffer.c convention,
# obstructed = 1):
#   HALL3 -> arm at pos1, buffer actively feeding    (filament in transit)
#   HALL2 -> arm at pos2, balanced                   (pushed until it couldn't)
#   HALL1 -> arm at pos3, excess slack               (buffer retracting)
#
# Sensor pipeline (T8, used when roadrunner_sensor + post_extruder_pin are
# both configured):
PHASE_IDLE = "idle"
PHASE_A_PUSH = "a_push"  # pulsed buffer-only feed, cold-safe
PHASE_B_ASSIST = "b_assist"  # buffer + extruder assist, needs heat
PHASE_ARRIVED = "arrived"  # phase C handoff has run
PHASE_FAILED = "failed"

# No-sensor fallback phases (arm-settle heuristic over the firmware's own
# continuous auto-feed). "Arrived" means we saw it feed and then hold
# steady at HALL2: the filament is up against the extruder gear and the
# buffer has settled. Requiring the feed phase first matters - an arm
# already resting at HALL2 would otherwise look like an instant arrival
# with filament nowhere near the extruder.
PHASE_WAIT_FEED = "wait_feed"
PHASE_FEEDING = "feeding"

# Phases in which a firmware error should abort the in-flight load. Split in
# two because _on_error handles them differently, not for cosmetic reasons:
# the fallback phases run off a reactor timer and can be failed directly,
# while the sensor phases run in their own greenlet loop and must be asked to
# stop via _phase_abort_reason instead (see _check_phase_abort).
LOAD_FALLBACK_PHASES = (PHASE_WAIT_FEED, PHASE_FEEDING)
LOAD_SENSOR_PHASES = (PHASE_A_PUSH, PHASE_B_ASSIST)

UNLOAD_IDLE = "idle"
# No-sensor fallback only (pre-T8 behaviour, unchanged): LLL_RUN back,
# waiting for the firmware's own entrance-switch `present` edge.
UNLOAD_RUNNING = "running"
# Sensor pipeline (T9), used when roadrunner_sensor + post_extruder_pin are
# both configured - mirrors PHASE_A_PUSH/PHASE_B_ASSIST/PHASE_ARRIVED above.
UNLOAD_TIP_FORM = "tip_form"  # optional tip_forming_gcode
UNLOAD_RETRACT = "retract"  # synchronized retract: steps 2-3
UNLOAD_CLEAR = "clear"  # bounded extra retract past extruder entry: step 4
UNLOAD_DONE = "done"
UNLOAD_FAILED = "failed"

# Phases in which a firmware error should abort the in-flight unload via
# the same poll-based abort idiom as LOAD_SENSOR_PHASES/_check_phase_abort.
UNLOAD_ACTIVE_PHASES = (UNLOAD_TIP_FORM, UNLOAD_RETRACT, UNLOAD_CLEAR)

# buffer.c: arm bitfield is H3H2H1 (bit2=hall3, bit1=hall2, bit0=hall1).
ARM_HALL1 = 0x01
ARM_HALL2 = 0x02
ARM_HALL3 = 0x04

# Firmware telemetry mask: default 0x0F plus EV_ARM (bit 5).
EV_ARM_BIT = 0x20
DEFAULT_EVENT_MASK = 0x0F | EV_ARM_BIT

# lll_param addressing for TELEM(9) key 0 = event_mask.
FUNC_TELEM = 9
KEY_EVENT_MASK = 0


# ----------------------------------------------------------------------
# Lane-driver backend seam
# ----------------------------------------------------------------------
#
# Pipeline code (load/unload/differential-monitoring, T8-T10) must talk
# only to this interface, never to a concrete backend's wrapped object.
# The reason to keep this narrow: the LLL board (bitbanged TMC UART, a
# small stepper) may eventually be swapped for something like a BTT
# EBB42. That should be a new backend class, not a rewrite of the
# pipeline. Exactly one backend exists today - LllBufferBackend - and
# there is deliberately no config option to pick between backends: there
# is nothing yet to pick between.
class LaneBackend(object):
    """Narrow interface a buffer_manager lane pipeline drives.

    A future backend (e.g. a non-Mellow board) implements this same
    surface; the pipeline code does not need to know which one it has.
    """

    def arm(self):
        """Return the current dancer-arm bitfield (H3H2H1, obstructed=1)."""
        raise NotImplementedError

    def jog(self, direction, ms):
        """Pulse the lane fwd/back for ms milliseconds."""
        raise NotImplementedError

    def run(self, direction, ms):
        """Bounded run fwd/back. ms=0 lets the backend pick its own bound
        (the LLL backend falls back to firmware run_max_ms)."""
        raise NotImplementedError

    def stop(self):
        """Stop host-driven motion; backend autonomy resumes per its own
        role/mode semantics."""
        raise NotImplementedError

    def state(self):
        """Return a dict snapshot of lane state (role, error, present,
        arm, ...). Contents are backend-specific."""
        raise NotImplementedError

    def mdm(self):
        """Return (present, counter): MDM presence (bool) and a
        free-running motion pulse counter (int, wraps at uint32), or
        (None, None) if this backend/firmware has no MDM module."""
        raise NotImplementedError


class LllBufferBackend(LaneBackend):
    """Wraps an lll_buffer object, driving it through its public surface
    (LLL_* gcode commands and get_status/probe_arm) wherever practical.

    One exception is noted at set_event_mask() below: lll_buffer's public
    surface has no non-persisting parameter setter, so this backend has
    to reach past LLL_SET into lane.param_cmd directly to avoid polluting
    the user's saved tweaks. That gap is a candidate for a future
    lll_buffer API addition (e.g. an LLL_SET NOPERSIST=1 flag, or a
    dedicated set_transient_param() method) - see the T7 report.
    """

    def __init__(self, lane, gcode, lane_name):
        self.lane = lane
        self.gcode = gcode
        self.lane_name = lane_name

    def arm(self):
        # probe_arm() is one of lll_buffer's documented probe helpers
        # ("used by LLL_DETECT_LOADED, all lanes"), not a private detail.
        arm, _role = self.lane.probe_arm()
        return arm

    def jog(self, direction, ms):
        self.gcode.run_script_from_command(
            f"LLL_JOG LANE={self.lane_name} DIR={direction} MS={ms}"
        )

    def run(self, direction, ms):
        self.gcode.run_script_from_command(
            f"LLL_RUN LANE={self.lane_name} DIR={direction} MS={ms}"
        )

    def stop(self):
        # Public surface only: LLL_STOP clears the host run flag and
        # hands autonomy back to the firmware per role/mode.
        self.gcode.run_script(f"LLL_STOP LANE={self.lane_name}")

    def state(self):
        # get_status()'s eventtime arg is unused by lll_buffer (kept only
        # for parity with Klipper's general status-object convention), so
        # a real timestamp is not needed here.
        return self.lane.get_status(0.0)

    def mdm(self):
        # T3 added mdm_present / mdm_count to lll_state and to
        # lll_buffer's status dict (both confirmed present as of this
        # task). mdm_count is a free-running uint32 pulse counter that
        # WRAPS - callers must diff it with wrap-safe unsigned arithmetic,
        # e.g. (new - old) & 0xFFFFFFFF. This backend just reports the
        # raw counter; the differencing is T10's job (MDM/Roadrunner
        # buckling detection), not this one's.
        s = self.state()
        return bool(s.get("mdm_present")), s.get("mdm_count")

    def has_event_gcode(self):
        # Wiring sanity check specific to this backend's event-bridge
        # design (see module docstring "How it hears about the dancer
        # arm"). Not part of the portable LaneBackend surface - a future
        # backend may not use an event_gcode bridge at all.
        return getattr(self.lane, "has_event_gcode", False)

    def set_event_mask(self, mask):
        # lll_buffer's only public parameter setter is the LLL_SET gcode
        # command (cmd_SET), which always persists via self._persist() -
        # see lll_buffer.py. event_mask must NOT become part of the
        # user's persisted tweak set (raised fresh every boot instead;
        # see the module docstring above), and there is no non-persisting
        # public equivalent. Reaching into lane.param_cmd is the
        # narrowest available way to hit TELEM(9) key 0 without that side
        # effect. See the class docstring - this is the one place this
        # backend touches an lll_buffer internal.
        return self.lane.param_cmd.send([FUNC_TELEM, KEY_EVENT_MASK, mask])


class BufferManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.name = config.get_name().split()[-1]
        self.lane_name = config.get("lane", self.name)
        # Set by _connect(): a LaneBackend, never an lll_buffer directly.
        self.backend = None

        # ------------------------------------------------------------
        # Sensor pipeline config (T8). roadrunner_sensor and
        # post_extruder_pin gate the whole three-phase pipeline: both
        # set -> phase A/B/C; both unset -> no-sensor arm-settle
        # fallback. One without the other is a config error, since a
        # half-configured pipeline would silently behave like the
        # fallback while the user believes they wired up sensors.
        # ------------------------------------------------------------
        self.roadrunner_name = config.get("roadrunner_sensor", "")
        post_extruder_pin = config.get("post_extruder_pin", None)
        have_rr = bool(self.roadrunner_name)
        have_pe = post_extruder_pin is not None
        if have_rr != have_pe:
            raise config.error(
                f"buffer_manager {self.name}: roadrunner_sensor and post_extruder_pin"
                " must both be set together (the three-phase load pipeline"
                " needs both to detect the phase A/B boundaries), or both"
                " left unset to use the no-sensor arm-settle fallback"
            )
        self.use_sensor_pipeline = have_rr and have_pe
        self.extruder_name = config.get("extruder", "extruder")
        self.roadrunner = None  # resolved in _connect()
        self.extruder = None  # resolved in _connect()

        # --- EVERY value below this point is an untested placeholder. ---
        # Nothing in this module has run on hardware; see
        # docs/PORT_RUNBOOK.md "Standing constraints".

        # Phase A: pulse width and tick period. 100ms is not arbitrary -
        # it matches the Roadrunner's own CHECK_RUNOUT_TIMEOUT refresh
        # rate (docs/PORT_RUNBOOK.md §0), so sampling faster gains
        # nothing. The value itself has not been run on this hardware.
        self.pulse_ms = config.getint("pulse_ms", 100, minval=20, maxval=2000)
        # PLACEHOLDER: minimum Roadrunner position delta (mm) a single
        # pulse must produce to count as progress. A pulse below this is
        # a "stall" sample (buckle or slip). Untested; likely needs
        # tightening once a real feed rate is benched.
        self.stall_mm = config.getfloat("stall_mm", 0.5, above=0.0)
        # PLACEHOLDER: consecutive stalled pulses (phase A) or
        # consecutive underextrusion_detected assist pulses (phase B)
        # before giving up. 1 was considered too hair-trigger for a
        # single 100ms sample window (Roadrunner position resolution /
        # sensor noise could produce an isolated false stall); this
        # tolerates a couple before calling it a real jam. Chosen for
        # "fail fast" per the runbook, not benched.
        self.stall_count = config.getint("stall_count", 3, minval=1)

        # Phase B: extruder-assist bounded move. PLACEHOLDERS - assist_mm
        # is meant to roughly cover the short bowden from Roadrunner to
        # the extruder gear plus margin; assist_speed borrows
        # lll_buffer.py's cmd_DETECT_LOADED F300 as a starting point for
        # this different path, not because it was benched here.
        self.assist_mm = config.getfloat("assist_mm", 30.0, above=0.0)
        self.assist_speed = config.getfloat("assist_speed", 300.0, above=0.0)
        # PLACEHOLDER: how many assist G1 E<assist_mm> pulses phase B
        # will attempt before failing if the toolhead switch never
        # confirms a grab.
        self.assist_max_pulses = config.getint("assist_max_pulses", 5, minval=1)
        # PLACEHOLDER: fallback bound (ms) for backend.run() during phase
        # B when phase A produced no usable learned feed rate (e.g. the
        # lane started already touching the Roadrunner). When a learned
        # rate IS available, phase B derives its own run bound from it
        # instead - see _run_phase_b().
        self.phase_b_run_ms = config.getint(
            "phase_b_run_ms", 4000, minval=100, maxval=60000
        )
        # PLACEHOLDER: sub-budget (s) for the whole of phase B, separate
        # from (and bounded by) the overall load_timeout below.
        self.phase_b_timeout = config.getfloat("phase_b_timeout", 30.0, above=0.0)

        # No-sensor fallback only: how long HALL2 must stay obstructed
        # before we call it arrived, and how long we wait for that to
        # happen before giving up.
        self.load_settle_time = config.getfloat("load_settle_time", 1.5, above=0.0)
        # Overall deadline for the whole load, either pipeline.
        self.load_timeout = config.getfloat("load_timeout", 180.0, above=0.0)
        # Overall deadline for the whole unload, either pipeline (steps
        # 1-5 for the sensor pipeline, or the single bounded run() below
        # for the no-sensor fallback).
        self.unload_timeout = config.getfloat("unload_timeout", 90.0, above=0.0)
        # No-sensor fallback only: bounded reverse run passed straight to
        # LLL_RUN. The firmware clamps its own back_timeout_ms on top of
        # this, so the host deadline only has to be the looser of the two
        # for the failure message to come from the right place.
        self.unload_run_ms = config.getint("unload_run_ms", 0, minval=0, maxval=60000)

        # ------------------------------------------------------------
        # Unload sensor pipeline (T9). Reuses stall_mm/stall_count from
        # the load config above - same idea (min encoder delta per pulse
        # before calling it a stall), no reason to duplicate the knob.
        # ------------------------------------------------------------
        # tip_forming_gcode template is loaded below, alongside the other
        # gcode_macro templates (gcode_macro is not resolved yet here).
        #
        # PLACEHOLDER: per-pulse retract distance (mm), issued as
        # G1 E-<n> on the same pulse_ms tick as the concurrent
        # backend.run("back", pulse_ms) during the synchronized retract
        # (steps 2-3). Untested; wants tuning against unload_retract_speed
        # once benched.
        self.unload_pulse_mm = config.getfloat("unload_pulse_mm", 2.0, above=0.0)
        # PLACEHOLDER: G1 feedrate (mm/min) for the retract pulses above.
        # Borrowed as a starting point from the old fork's
        # variable_unload_speed (900,
        # buffer_manager_old/klippy_macros/buffer_manager.cfg:71) - not
        # benched on this hardware or against this pulse width.
        self.unload_retract_speed = config.getfloat(
            "unload_retract_speed", 900.0, above=0.0
        )
        # PLACEHOLDER: target encoder-MEASURED mm of retract, accumulated
        # from real Roadrunner position deltas between the toolhead
        # switch clearing and filament_detected going false. Stands in
        # for "roughly how far the extruder-entry-to-Roadrunner span is",
        # but unlike the old fork's gear_to_mdm_mm this is never driven
        # blind - falling short of it only logs a warning; see
        # _run_unload_clear(). Invented, not measured.
        self.unload_clear_mm = config.getfloat("unload_clear_mm", 15.0, above=0.0)
        # PLACEHOLDER: bounded count of extra pulse_ms pulses run AFTER
        # filament_detected goes false, as margin past the extruder
        # entry. The Roadrunner cannot see this motion (nothing left
        # threading its encoder wheel - see _run_unload_clear()), so this
        # is deliberately small and count-bounded rather than a blind
        # timed/distance move - it replaces the old fork's unmeasured
        # gear_to_mdm_mm=60 guess, not a re-implementation of it.
        # MDM encoder scale. The MDM sits on the buffer arm, so during the
        # final unload retract - after the tip has passed the Roadrunner -
        # it is the ONLY sensor filament still threads, and therefore the
        # only one that can still measure. Vendor-derived, not benched:
        # the 5-segment disk's published 3.5 mm is per full segment cycle,
        # and the firmware counts BOTH edges, so one count is half of that.
        # Swapping disks changes it: 3-segment = 2.90, 10-segment = 0.865.
        # Set to 0 to disable MDM measurement and fall back to bounded
        # blind pulses.
        self.mdm_mm_per_count = config.getfloat("mdm_mm_per_count", 1.75, minval=0.0)
        # --- T10: MDM/Roadrunner differential monitoring ---------------
        # The MDM sits on the buffer arm, the Roadrunner near the toolhead;
        # they bracket the long reverse bowden. Filament advancing at one
        # end but not the other means it is going somewhere it should not
        # - buckling or compressing INSIDE the bowden, which neither
        # sensor can see alone.
        #
        # This CANNOT be a per-pulse check. One MDM count is
        # mdm_mm_per_count (1.75 mm by default) - the finest movement it
        # can resolve - so on a single ~100 ms pulse the counter usually
        # has not ticked at all. Compare over a rolling window long enough
        # to accumulate several counts instead; ~17.5 mm is about 10
        # counts, i.e. roughly +/-10% quantization.
        # Consecutive zero-MDM-count pulses before phase A calls it a
        # stall. Much coarser than the Roadrunner's stall_count: one MDM
        # count is mdm_mm_per_count (1.75mm), so a single 100ms pulse at a
        # modest feed rate may legitimately produce none. PLACEHOLDER.
        self.phase_a_stall_pulses = config.getint("phase_a_stall_pulses", 10, minval=1)
        self.differential_window_mm = config.getfloat(
            "differential_window_mm", 17.5, above=0.0
        )
        # Fraction of the window the two ends may disagree by before it is
        # reported. Deliberately generous: PLACEHOLDER, unvalidated, and
        # real buckling shows up as near-total divergence (MDM moving,
        # Roadrunner flat) rather than a subtle percentage.
        self.differential_tolerance = config.getfloat(
            "differential_tolerance", 0.5, above=0.0
        )
        # warn (default) or error. Defaults to warn because this heuristic
        # has never run on hardware and BOTH encoders are friction-coupled
        # - either can slip, and if they slip together this sees nothing.
        # A disagreement means "something is wrong somewhere", not proof
        # of which end failed. Switch to error once benched.
        self.differential_action = config.getchoice(
            "differential_action", {"warn": "warn", "error": "error"}, "warn"
        )
        self.unload_clear_pulses = config.getint("unload_clear_pulses", 10, minval=0)

        self.event_mask = config.getint(
            "event_mask", DEFAULT_EVENT_MASK, minval=0, maxval=0x7F
        )
        if not self.event_mask & EV_ARM_BIT:
            raise config.error(
                f"buffer_manager {self.name}: event_mask"
                f" 0x{self.event_mask:02X} does not include the arm bit"
                f" (0x{EV_ARM_BIT:02X}) - arrival detection would never fire"
            )

        gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.loaded_template = gcode_macro.load_template(config, "loaded_gcode", "")
        self.load_failed_template = gcode_macro.load_template(
            config, "load_failed_gcode", ""
        )
        self.unloaded_template = gcode_macro.load_template(config, "unloaded_gcode", "")
        self.unload_failed_template = gcode_macro.load_template(
            config, "unload_failed_gcode", ""
        )
        # Optional: there is no toolhead cutter on this hardware, so the
        # tip is shaped rather than cut. Empty by default (no-op).
        self.tip_forming_template = gcode_macro.load_template(
            config, "tip_forming_gcode", ""
        )

        self.arm = 0
        self.load_phase = PHASE_IDLE
        self.load_fail_reason = None
        self.settle_deadline = None
        self.load_deadline = None
        # Set by an event/error callback to interrupt an in-flight phase
        # A/B loop (which runs in its own reactor callback greenlet and
        # polls this rather than being pushed to directly - see
        # _check_phase_abort()). "aborted" is the quiet-cancel sentinel
        # used by cmd_LOAD_ABORT; any other string is a failure reason.
        self._phase_abort_reason = None
        # Bumped on every fresh LOAD_BEGIN and on LOAD_ABORT. A phase A/B
        # greenlet captures its own value at the point it is SCHEDULED (not
        # inside its own body - by the time the body runs, a newer call may
        # have already bumped this again) and compares it at every loop
        # check. This is a stronger guard than _phase_abort_reason alone:
        # LOAD_BEGIN resets that sentinel to None at its own top, so an
        # ABORT immediately followed by a fresh BEGIN can otherwise let the
        # old greenlet wake up, find the sentinel already cleared and
        # load_phase already PHASE_A_PUSH again (set by the new call), and
        # keep running concurrently with the new one. The generation check
        # catches that: a superseded greenlet sees its own value no longer
        # matches and returns quietly (not a failure - being superseded is
        # the intended outcome of a fresh LOAD_BEGIN).
        self._load_generation = 0
        # Phase A learned feed rate accumulators (mm advanced / ms of
        # pulse that produced that advance), reset each load attempt.
        self._learned_feed_mm = 0.0
        self._learned_feed_ms = 0
        self._phaseA_stall_streak = 0
        self._phaseB_slip_streak = 0
        # Updated by the post-extruder switch button callback, sensor
        # pipeline only.
        self._toolhead_switch_state = False

        self.unload_phase = UNLOAD_IDLE
        self.unload_fail_reason = None
        self.unload_deadline = None
        # Set by an event/error callback to interrupt an in-flight unload
        # TIP_FORM/RETRACT/CLEAR loop (own reactor callback greenlet,
        # polled rather than pushed to - same idiom as
        # _phase_abort_reason/_check_phase_abort above). "aborted" is the
        # quiet-cancel sentinel used by cmd_UNLOAD_ABORT.
        self._unload_abort_reason = None
        # Same idiom as _load_generation above, for the TIP_FORM/RETRACT/
        # CLEAR greenlet chain.
        self._unload_generation = 0
        self._unload_stall_streak = 0
        # Encoder-measured (Roadrunner position delta) mm retracted since
        # the toolhead switch cleared, accumulated by _run_unload_retract
        # and consumed by _run_unload_clear - see the T9 module docstring.
        self._unload_since_switch_mm = 0.0
        self._unload_clear_accum_mm = 0.0
        self._diff_rr_mm = 0.0  # Roadrunner mm this window
        self._diff_mdm_base = None  # MDM counter at window start
        self._diff_disabled = False  # latched off after a failure
        self._mdm_last_counter = None  # phase A per-pulse baseline
        self._diff_last_rr = 0.0  # phase B Roadrunner baseline
        self._phaseA_nomdm_warned = False

        self.load_timer = self.reactor.register_timer(self._load_timer)
        self.unload_timer = self.reactor.register_timer(self._unload_timer)

        if have_pe:
            # Raw pin via buttons.register_buttons, reusing the pattern
            # at buffer_manager_old/klippy_extras/buffer_manager.py:
            # 156-172: a single registration delivers one fresh, debounced
            # snapshot per callback rather than racing independent
            # registrations. That file combines three hall pins into one
            # 3-bit snapshot for that reason; here there is only one pin,
            # so there is nothing to combine, but the same registration
            # idiom is kept for consistency (and in case a second switch
            # is ever added on this lane).
            buttons = self.printer.load_object(config, "buttons")
            buttons.register_buttons([post_extruder_pin], self._toolhead_switch_event)

        self.printer.register_event_handler("klippy:connect", self._connect)
        self.printer.register_event_handler("klippy:ready", self._ready)

        self.gcode.register_mux_command(
            "BUFFER_LOAD_BEGIN",
            "LANE",
            self.lane_name,
            self.cmd_LOAD_BEGIN,
            desc="Arm the arrival watcher; runs loaded_gcode once filament "
            "reaches the extruder",
        )
        self.gcode.register_mux_command(
            "BUFFER_LOAD_ABORT",
            "LANE",
            self.lane_name,
            self.cmd_LOAD_ABORT,
            desc="Cancel an armed arrival watcher without running any macro",
        )
        self.gcode.register_mux_command(
            "BUFFER_UNLOAD",
            "LANE",
            self.lane_name,
            self.cmd_UNLOAD,
            desc="Run the lane back until the entrance sensor clears",
        )
        self.gcode.register_mux_command(
            "BUFFER_UNLOAD_ABORT",
            "LANE",
            self.lane_name,
            self.cmd_UNLOAD_ABORT,
            desc="Stop an in-flight unload without running any macro",
        )
        # Bridge target for lll_buffer's event_gcode hook.
        self.gcode.register_mux_command(
            "BUFFER_LOAD_EVENT",
            "LANE",
            self.lane_name,
            self.cmd_LOAD_EVENT,
            desc="Internal: feed an lll_buffer firmware event to buffer_manager",
        )

    def _connect(self):
        # The one place this module looks up an lll_buffer object by name.
        # It is wrapped in a backend immediately and never touched again
        # from outside LllBufferBackend.
        lane_obj = self.printer.lookup_object("lll_buffer " + self.lane_name, None)
        if lane_obj is None:
            raise self.printer.config_error(
                f"buffer_manager {self.name}: no [lll_buffer {self.lane_name}]"
                " section found"
            )
        self.backend = LllBufferBackend(lane_obj, self.gcode, self.lane_name)
        if not self.backend.has_event_gcode():
            raise self.printer.config_error(
                f"buffer_manager {self.name}: [lll_buffer {self.lane_name}]"
                " has no event_gcode - add"
                "\n    event_gcode:\n"
                f"        BUFFER_LOAD_EVENT LANE={self.lane_name} NAME={{event.name}}"
                " VALUE={event.value}"
            )
        if self.use_sensor_pipeline:
            self.roadrunner = self.printer.lookup_object(
                "high_resolution_filament_sensor " + self.roadrunner_name, None
            )
            if self.roadrunner is None:
                raise self.printer.config_error(
                    f"buffer_manager {self.name}: no [high_resolution_filament_sensor"
                    f" {self.roadrunner_name}] section found"
                )
            self.extruder = self.printer.lookup_object(self.extruder_name, None)
            if self.extruder is None:
                raise self.printer.config_error(
                    f"buffer_manager {self.name}: extruder object"
                    f" '{self.extruder_name}' not found"
                )

    def _ready(self):
        # Raise the firmware telemetry mask so EV_ARM is actually emitted.
        # Done every boot with a direct MCU command rather than LLL_SET: it
        # must not depend on [save_variables] being present, and it must not
        # quietly become part of the user's persisted tweak set.
        try:
            assert self.backend is not None
            p = self.backend.set_event_mask(self.event_mask)
            if not p.get("ok", 1):
                logger.warning(
                    "buffer_manager %s: firmware refused event_mask 0x%02X",
                    self.name,
                    self.event_mask,
                )
            elif p.get("val") != self.event_mask:
                logger.warning(
                    "buffer_manager %s: event_mask applied as 0x%02X, asked 0x%02X",
                    self.name,
                    p.get("val"),
                    self.event_mask,
                )
        except Exception:
            logger.exception("buffer_manager %s: could not set event_mask", self.name)

    # ------------------------------------------------------------------
    # Event bridge
    # ------------------------------------------------------------------
    def cmd_LOAD_EVENT(self, gcmd):
        name = gcmd.get("NAME", "").strip().lower()
        value = gcmd.get_int("VALUE", 0)
        if name == "arm":
            self._on_arm(value)
        elif name == "present":
            self._on_present(value)
        elif name == "ecode_set" and value:
            self._on_error(value)

    def _on_error(self, ecode):
        # The firmware latched an error, so the motor is stopped. Fail now
        # instead of waiting out the deadline for an arrival that cannot
        # come.
        reason = f"buffer error latched (ecode={ecode})"
        if self.load_phase in LOAD_FALLBACK_PHASES:
            self._fail_load(reason)
        elif self.load_phase in LOAD_SENSOR_PHASES:
            # The phase A/B loop runs in its own reactor callback greenlet
            # (never a timer - see the module docstring), and polls this
            # flag itself between pulses rather than being interrupted
            # directly here. This handler may run from MCU response
            # handling, which is not a safe place to block on gcode.
            self._phase_abort_reason = reason
        if self.unload_phase == UNLOAD_RUNNING:
            # No-sensor fallback: no in-flight greenlet loop to poll - fail
            # directly, same as the load fallback above.
            self._fail_unload(reason)
        elif self.unload_phase in UNLOAD_ACTIVE_PHASES:
            # Sensor pipeline: TIP_FORM/RETRACT/CLEAR run in their own
            # reactor callback greenlet and poll this rather than being
            # pushed to directly - see _check_unload_abort().
            self._unload_abort_reason = reason

    # ------------------------------------------------------------------
    # Load arrival detection - no-sensor fallback (arm-settle heuristic)
    # ------------------------------------------------------------------
    def _on_arm(self, arm):
        self.arm = arm
        if self.load_phase not in (PHASE_WAIT_FEED, PHASE_FEEDING):
            return
        eventtime = self.reactor.monotonic()
        h2 = bool(arm & ARM_HALL2)
        h3 = bool(arm & ARM_HALL3)
        if self.load_phase == PHASE_WAIT_FEED:
            if h3:
                self.load_phase = PHASE_FEEDING
            return
        # PHASE_FEEDING: start (or restart) the settle window whenever the
        # arm sits at HALL2 alone. Any other position clears it, so a brief
        # pass through pos2 mid-transit does not count as arrival.
        if h2 and not h3:
            if self.settle_deadline is None:
                self.settle_deadline = eventtime + self.load_settle_time
                self._rearm_load_timer()
        elif self.settle_deadline is not None:
            self.settle_deadline = None
            self._rearm_load_timer()

    def _rearm_load_timer(self):
        waketime = self._next_load_waketime()
        self.reactor.update_timer(self.load_timer, waketime)

    def _next_load_waketime(self):
        candidates = [
            d for d in (self.settle_deadline, self.load_deadline) if d is not None
        ]
        return min(candidates) if candidates else self.reactor.NEVER

    def _load_timer(self, eventtime):
        # Fallback-only timer: phase A/B (sensor pipeline) run entirely in
        # a register_callback greenlet, never off this timer.
        if self.load_phase not in (PHASE_WAIT_FEED, PHASE_FEEDING):
            return self.reactor.NEVER
        if self.settle_deadline is not None:
            if not self.arm & ARM_HALL2:
                # Defensive: the arm left HALL2 without an event clearing the
                # window. Drop it here rather than reporting an arrival the
                # sensors no longer support.
                self.settle_deadline = None
            elif eventtime >= self.settle_deadline:
                self.load_phase = PHASE_ARRIVED
                self.settle_deadline = None
                self.load_deadline = None
                # Never run gcode from a timer callback - it executes in the
                # main reactor greenlet, and loaded_gcode blocks on M109.
                # Hand off to a callback greenlet, which can block safely.
                self.reactor.register_callback(self._run_loaded_gcode)
                return self.reactor.NEVER
        if self.load_deadline is not None and eventtime >= self.load_deadline:
            self._fail_load("timed out waiting for filament to reach the extruder")
            return self.reactor.NEVER
        return self._next_load_waketime()

    def _fail_load(self, reason):
        self.load_phase = PHASE_FAILED
        self.load_fail_reason = reason
        self.settle_deadline = None
        self.load_deadline = None
        self._phase_abort_reason = None
        self.reactor.update_timer(self.load_timer, self.reactor.NEVER)
        if self.use_sensor_pipeline:
            # The fallback path never commands motion (it only watches the
            # firmware's own auto-feed), so there is nothing to stop there.
            # Phase A/B did command jog()/run() and must be stopped.
            self._stop_lane()
        self.reactor.register_callback(self._run_load_failed_gcode)

    def _run_loaded_gcode(self, eventtime):
        self._exec_template(self.loaded_template, "loaded_gcode")

    def _run_load_failed_gcode(self, eventtime):
        self._exec_template(self.load_failed_template, "load_failed_gcode")

    # ------------------------------------------------------------------
    # Load arrival detection - sensor pipeline (phase A / B / C)
    # ------------------------------------------------------------------
    def _check_phase_abort(self, generation):
        """Consume a pending abort/failure request, if any, and check that
        the calling greenlet is still the current one. Returns True if the
        calling phase loop must stop now. Called between every pulse so a
        firmware error event, BUFFER_LOAD_ABORT, or a newer LOAD_BEGIN
        interrupts promptly without needing to touch the loop from outside
        it.

        The generation check catches a case _phase_abort_reason alone
        cannot: cmd_LOAD_BEGIN resets that sentinel to None at its own top,
        so an ABORT immediately followed by a fresh BEGIN (before this
        greenlet's next wakeup) can leave the sentinel already cleared by
        the time this runs. Comparing generation stops the (now superseded)
        old greenlet regardless - see the _load_generation comment in
        __init__."""
        if generation != self._load_generation:
            return True
        reason = self._phase_abort_reason
        if reason is None:
            return False
        self._phase_abort_reason = None
        if reason != "aborted":
            self._fail_load(reason)
        return True

    def _roadrunner_status(self):
        if self.roadrunner is None:
            return {}
        try:
            return self.roadrunner.get_status(self.reactor.monotonic())
        except Exception:
            logger.exception(
                "buffer_manager %s: roadrunner status read failed", self.name
            )
            return {}

    def _extruder_can_extrude(self):
        # Klipper blocks E-moves below min_extrude_temp; ask the extruder
        # object rather than duplicating that threshold here (same idiom
        # lll_buffer.py's cmd_DETECT_LOADED uses).
        if self.extruder is None:
            return False
        try:
            return bool(
                self.extruder.get_status(self.reactor.monotonic()).get(
                    "can_extrude", False
                )
            )
        except Exception:
            logger.exception(
                "buffer_manager %s: extruder status read failed", self.name
            )
            return False

    def _toolhead_switch_event(self, eventtime, state):
        self._toolhead_switch_state = bool(state & 1)

    def _run_phase_a(self, eventtime, generation):
        # Phase A - PUSH: buffer alone, cold-safe. Pulsed jog() on a
        # pulse_ms tick until the Roadrunner reports filament_detected.
        # Runs entirely in this callback greenlet (never a timer), so
        # reactor.pause() between pulses is safe - see module docstring.
        #
        # generation was captured by the caller at the moment this greenlet
        # was scheduled (see cmd_LOAD_BEGIN) - see _load_generation in
        # __init__ for why that matters.
        if generation != self._load_generation or self.load_phase != PHASE_A_PUSH:
            return  # superseded, or aborted/failed before this greenlet
            # even started
        while True:
            if self._check_phase_abort(generation):
                return
            if self.reactor.monotonic() >= self.load_deadline:
                self._fail_load(
                    "phase A (push) timed out before filament reached the"
                    " Roadrunner sensor"
                )
                return
            status = self._roadrunner_status()
            if status.get("filament_detected"):
                break
            try:
                assert self.backend is not None
                self.backend.jog("fwd", self.pulse_ms)
            except Exception:
                logger.exception("buffer_manager %s: phase A jog failed", self.name)
                self._fail_load("jog command failed during phase A")
                return
            self.reactor.pause(self.reactor.monotonic() + self.pulse_ms / 1000.0)
            # PHASE A MEASURES WITH THE MDM, NOT THE ROADRUNNER.
            # This loop runs *until* filament reaches the Roadrunner, so
            # for its entire duration nothing threads that encoder wheel
            # and its `position` cannot advance. Watching it here would
            # read every healthy pulse as a stall and fail every load.
            # The tip is upstream of the Roadrunner but downstream of the
            # MDM, so the MDM is the only sensor that can see this phase -
            # the mirror of _run_unload_clear(), where the tip has passed
            # the Roadrunner and the MDM is again the only one left.
            # (The differential check is meaningless here for the same
            # reason - one of its two inputs is blind. It runs in phase B.)
            counts = self._mdm_delta_counts()
            if counts is None:
                # No MDM: nothing can measure this phase. The deadline is
                # the only backstop. Said once, not once per pulse.
                if not self._phaseA_nomdm_warned:
                    self._phaseA_nomdm_warned = True
                    logger.warning(
                        "buffer_manager %s: no MDM counter - phase A has"
                        " no way to detect a stall and will rely on"
                        " load_timeout alone",
                        self.name,
                    )
            elif counts > 0:
                # Learned feed rate also comes from the MDM here, for the
                # same reason. counts * mm_per_count, not Roadrunner mm.
                self._learned_feed_mm += counts * self.mdm_mm_per_count
                self._learned_feed_ms += self.pulse_ms
                self._phaseA_stall_streak = 0
            else:
                # Zero counts on one pulse is NOT evidence of a stall: one
                # count is 1.75mm by default, and a 100ms pulse at a
                # modest feed rate may legitimately not tick. Only a long
                # run of zero-count pulses means the filament is not
                # moving at the buffer end. phase_a_stall_pulses is
                # therefore much coarser than the Roadrunner's stall_count.
                self._phaseA_stall_streak += 1
                if self._phaseA_stall_streak >= self.phase_a_stall_pulses:
                    expected_mm = (self._phaseA_stall_streak
                                  * self.mdm_mm_per_count)
                    self._fail_load(
                        f"phase A stalled: {self._phaseA_stall_streak}"
                        " consecutive pulses produced no MDM movement at all"
                        f" (>= {expected_mm:.1f}mm of travel expected) -"
                        " filament is not advancing at the buffer end."
                        " Buckling, slip, or an empty lane"
                    )
                    return
        # Phase A complete - filament seen at the Roadrunner. Make the
        # A/B boundary explicit even though pulses are self-terminating.
        self._stop_lane()
        self.reactor.register_callback(
            lambda et, g=generation: self._run_phase_b(et, g)
        )

    def _run_phase_b(self, eventtime, generation):
        # Phase B - ASSIST: needs the extruder above min_extrude_temp.
        # Fail cleanly right at the A->B boundary if it is cold, rather
        # than mid-move - Klipper would refuse the E-move anyway, but
        # only after the buffer side has already started running.
        if generation != self._load_generation or self.load_phase != PHASE_A_PUSH:
            return  # superseded, or aborted/failed already
        if not self._extruder_can_extrude():
            self._fail_load(
                "phase A complete but the extruder is below"
                " min_extrude_temp - heat it (e.g. M109) before phase B"
                " (assist) can run"
            )
            return
        self.load_phase = PHASE_B_ASSIST
        self._toolhead_switch_state = False
        self._phaseB_slip_streak = 0
        # Keep the phase-B budget bounded by the overall load deadline when
        # one is available.  The explicit None check also makes the
        # optional deadline safe for static type checking.
        deadline_b = self.reactor.monotonic() + self.phase_b_timeout
        if self.load_deadline is not None:
            deadline_b = min(self.load_deadline, deadline_b)

        # Bound backend.run() using the feed rate phase A just learned,
        # rather than a blind placeholder, when a sample is available.
        # x2 is an invented safety margin (assist load is not push-alone,
        # so the true rate may differ) - not benched.
        if self._learned_feed_ms > 0:
            rate_mm_per_ms = self._learned_feed_mm / float(self._learned_feed_ms)
            run_ms = (
                int(self.assist_mm / rate_mm_per_ms * 2)
                if rate_mm_per_ms > 0
                else self.phase_b_run_ms
            )
            run_ms = max(self.pulse_ms, min(run_ms, 60000))
        else:
            run_ms = self.phase_b_run_ms

        gcode_move = self.printer.lookup_object("gcode_move")

        attempts = 0
        # Baseline for the differential window. Filament has just reached
        # the Roadrunner, so this is the first position reading that means
        # anything.
        self._diff_last_rr = self._roadrunner_status().get("position", 0.0)
        self._diff_reset()
        while True:
            if self._check_phase_abort(generation):
                return
            if self.reactor.monotonic() >= deadline_b:
                self._fail_load(
                    "phase B (assist) timed out waiting for the toolhead"
                    " switch to confirm pickup"
                )
                return
            attempts += 1
            try:
                assert self.backend is not None
                self.backend.run("fwd", run_ms)
            except Exception:
                logger.exception("buffer_manager %s: phase B run failed", self.name)
                self._fail_load("run command failed during phase B")
                return
            # The extruder must not fight the whole path alone - the
            # buffer run() above is already dispatched and bounded
            # (autonomous on the firmware side) while this blocks on the
            # extrude move, which is the "concurrent" half of phase B.
            try:
                self._extrude(gcode_move, self.assist_mm, self.assist_speed)
            except Exception:
                logger.exception(
                    "buffer_manager %s: phase B assist move failed", self.name
                )
                self._fail_load("assist extrude move failed during phase B")
                return
            if self._check_phase_abort(generation):
                return
            status = self._roadrunner_status()
            # T10 differential lives HERE, not in phase A. By this point
            # filament threads BOTH the MDM (buffer end) and the Roadrunner
            # (toolhead end), so the two are finally measuring the same
            # strand and a disagreement between them means something. In
            # phase A the Roadrunner has no filament in it at all, so the
            # comparison there would be against a constant zero.
            rr_delta = abs(
                status.get("position", self._diff_last_rr) - self._diff_last_rr
            )
            self._diff_last_rr = status.get("position", self._diff_last_rr)
            if self._diff_check(rr_delta, self._fail_load):
                return
            if status.get("underextrusion_detected"):
                self._phaseB_slip_streak += 1
                if self._phaseB_slip_streak >= self.stall_count:
                    self._fail_load(
                        "phase B underextrusion persisted for"
                        f" {self._phaseB_slip_streak} assist pulses"
                        " (slip suspected)"
                    )
                    return
            else:
                self._phaseB_slip_streak = 0
            if self._toolhead_switch_state:
                break
            if attempts >= self.assist_max_pulses:
                self._fail_load(
                    "phase B: toolhead switch did not confirm pickup after"
                    f" {attempts} assist pulses"
                )
                return
        # Phase C - HANDOFF.
        self._stop_lane()
        self.load_phase = PHASE_ARRIVED
        self.load_deadline = None
        self.reactor.register_callback(self._run_loaded_gcode)

    # ------------------------------------------------------------------
    # Unload - no-sensor fallback (pre-T8 behaviour, unchanged by T9)
    # ------------------------------------------------------------------
    # LLL_RUN back, waiting for the firmware's own entrance-switch
    # `present` edge to go low. Used when roadrunner_sensor/
    # post_extruder_pin are not configured - mirrors how the load path
    # falls back to the arm-settle heuristic (see self.use_sensor_pipeline).
    def _on_present(self, present):
        if self.unload_phase != UNLOAD_RUNNING or present:
            return
        # Filament cleared the entrance sensor: the lane is empty. This is a
        # better completion signal than a fixed reverse distance, which is
        # why the unload does not guess at a length.
        self.unload_phase = UNLOAD_DONE
        self.unload_deadline = None
        self.reactor.update_timer(self.unload_timer, self.reactor.NEVER)
        self.reactor.register_callback(self._stop_then_unloaded)

    def _unload_timer(self, eventtime):
        # Fallback-only timer: the sensor pipeline's TIP_FORM/RETRACT/CLEAR
        # phases run entirely in a register_callback greenlet, never off
        # this timer - same split as _load_timer vs phase A/B above.
        if self.unload_phase != UNLOAD_RUNNING:
            return self.reactor.NEVER
        if self.unload_deadline is not None and eventtime >= self.unload_deadline:
            self._fail_unload("timed out waiting for the entrance sensor to clear")
            return self.reactor.NEVER
        return self.unload_deadline or self.reactor.NEVER

    def _fail_unload(self, reason):
        self.unload_phase = UNLOAD_FAILED
        self.unload_fail_reason = reason
        self.unload_deadline = None
        self._unload_abort_reason = None
        self.reactor.update_timer(self.unload_timer, self.reactor.NEVER)
        self.reactor.register_callback(self._stop_then_unload_failed)

    def _check_unload_abort(self, generation):
        """Consume a pending abort/failure request for the sensor-pipeline
        unload, if any, and check that the calling greenlet is still the
        current one. Returns True if the calling phase loop must stop now.
        Mirrors _check_phase_abort() above, including the generation check -
        see _unload_generation/_load_generation in __init__."""
        if generation != self._unload_generation:
            return True
        reason = self._unload_abort_reason
        if reason is None:
            return False
        self._unload_abort_reason = None
        if reason != "aborted":
            self._fail_unload(reason)
        return True

    # ------------------------------------------------------------------
    # Unload - sensor pipeline (T9): TIP_FORM -> RETRACT -> CLEAR -> DONE
    # ------------------------------------------------------------------
    def _run_unload_tip_form(self, eventtime, generation):
        # Step 1 (optional): no toolhead cutter exists on this hardware,
        # so the tip is shaped rather than cut. Empty template by default
        # - a no-op in that case. Best-effort like the other gcode hooks
        # in this file (_exec_template logs and swallows a failing
        # template rather than aborting the unload over cosmetic tip
        # shaping); a real failure further down (stall, timeout, firmware
        # error) still aborts normally.
        #
        # generation was captured by the caller at the moment this greenlet
        # was scheduled (see cmd_UNLOAD) - see _unload_generation in
        # __init__ for why that matters.
        if (
            generation != self._unload_generation
            or self.unload_phase != UNLOAD_TIP_FORM
        ):
            return
        self._exec_template(self.tip_forming_template, "tip_forming_gcode")
        if self._check_unload_abort(generation):
            return
        self.unload_phase = UNLOAD_RETRACT
        self.reactor.register_callback(
            lambda et, g=generation: self._run_unload_retract(et, g)
        )

    def _run_unload_retract(self, eventtime, generation):
        # Steps 2-3: synchronized retract. G1 E-<unload_pulse_mm> issued
        # concurrent with backend.run("back", pulse_ms) on every tick -
        # upstream's finding (docs/PORT_RUNBOOK.md T9, same root cause as
        # phase B on load) is that the extruder pulling ALONE against the
        # whole path slips; with the buffer actively following, the tip
        # clears reliably. Runs entirely in this callback greenlet (never
        # a timer), so reactor.pause() between pulses is safe - same
        # idiom as _run_phase_a/_run_phase_b.
        #
        # Ends once BOTH are true:
        #   - the post-extruder (toolhead) switch has cleared (filament
        #     retracted back past it)
        #   - the Roadrunner's filament_detected has gone false (filament
        #     retracted back past the Roadrunner too)
        # Order matters physically (toolhead switch is downstream/closer
        # to the hotend than the Roadrunner - see §0's filament path), but
        # this loop tolerates either already being true at entry (e.g. a
        # retried unload) rather than assuming a fresh transition.
        if generation != self._unload_generation or self.unload_phase != UNLOAD_RETRACT:
            return  # superseded, or aborted/failed before this greenlet
            # even started
        # Klipper blocks E-moves (both directions) below min_extrude_temp,
        # same as phase B on load - fail cleanly here rather than mid-move.
        if not self._extruder_can_extrude():
            self._fail_unload(
                "cannot retract - the extruder is below min_extrude_temp,"
                " heat it (e.g. M109) before BUFFER_UNLOAD can run the"
                " synchronized retract"
            )
            return
        gcode_move = self.printer.lookup_object("gcode_move")
        switch_cleared = not self._toolhead_switch_state
        sensor_cleared = False
        self._unload_since_switch_mm = 0.0
        # T10: differential monitoring runs inside the loop below via
        # _diff_check(); start a fresh comparison window here.
        self._diff_reset()
        while True:
            if self._check_unload_abort(generation):
                return
            if self.reactor.monotonic() >= self.unload_deadline:
                self._fail_unload(
                    f"synchronized retract timed out (switch_cleared={switch_cleared}"
                    f" sensor_cleared={sensor_cleared})"
                )
                return
            pos_before = self._roadrunner_status().get("position", 0.0)
            try:
                assert self.backend is not None
                self.backend.run("back", self.pulse_ms)
            except Exception:
                logger.exception(
                    "buffer_manager %s: unload retract run() failed", self.name
                )
                self._fail_unload("backend run() failed during synchronized retract")
                return
            # The buffer run() above is already dispatched and bounded
            # (autonomous on the firmware side) while this blocks on the
            # retract move - the "concurrent" half of the synchronized
            # retract, same shape as phase B's assist move.
            try:
                self._extrude(
                    gcode_move, -self.unload_pulse_mm, self.unload_retract_speed
                )
            except Exception:
                logger.exception(
                    "buffer_manager %s: unload retract extrude move failed", self.name
                )
                self._fail_unload(
                    "retract extrude move failed during synchronized retract"
                )
                return
            if self._check_unload_abort(generation):
                return
            status = self._roadrunner_status()
            # abs(): the Roadrunner's position sign convention depends on
            # its own invert_direction config, which this module does not
            # know. Using the magnitude avoids risking a false stall from
            # having guessed the sign wrong - untested either way.
            moved = abs(status.get("position", pos_before) - pos_before)
            # T10: same rolling comparison during the synchronized
            # retract. Both ends move backward here; magnitudes only.
            if self._diff_check(moved, self._fail_unload):
                return
            if moved >= self.stall_mm:
                self._unload_stall_streak = 0
                if switch_cleared:
                    self._unload_since_switch_mm += moved
            elif not sensor_cleared:
                # Only meaningful while the strand is still expected to be
                # threading the Roadrunner - once sensor_cleared is True
                # this loop is about to break anyway, and a lack of
                # movement AFTER that point is the expected case handled
                # in _run_unload_clear(), not a stall here.
                self._unload_stall_streak += 1
                if self._unload_stall_streak >= self.stall_count:
                    self._fail_unload(
                        f"synchronized retract stalled: {self._unload_stall_streak}"
                        " consecutive pulses moved <"
                        f"{self.stall_mm:.2f}mm (upstream's finding is that"
                        " the extruder pulling alone against the whole"
                        " path slips - the buffer run() above should be"
                        " preventing that)"
                    )
                    return
            if not switch_cleared and not self._toolhead_switch_state:
                switch_cleared = True
            if switch_cleared and not status.get("filament_detected", True):
                sensor_cleared = True
            if switch_cleared and sensor_cleared:
                break
        self._unload_clear_accum_mm = self._unload_since_switch_mm
        self.unload_phase = UNLOAD_CLEAR
        self.reactor.register_callback(
            lambda et, g=generation: self._run_unload_clear(et, g)
        )

    def _run_unload_clear(self, eventtime, generation):
        # Step 4: retract further past the extruder entry. By this point
        # the Roadrunner has already reported filament_detected=False -
        # meaning nothing is left threading its encoder wheel. The
        # Roadrunner's `position` is driven purely by that wheel's own
        # rotation (see high_resolution_filament_sensor.py's
        # _update_state_from_sensor(): computed from the raw magnetic
        # rotation reading, not gated on any commanded move), so once
        # nothing touches it, position CANNOT advance regardless of what
        # the buffer/extruder do further upstream. A silent Roadrunner
        # here is therefore the EXPECTED steady state, not evidence of a
        # jam - unlike _run_unload_retract()'s stall check just above,
        # a lack of movement in THIS phase does not fail the unload.
        #
        # What the encoder *could* still do: _run_unload_retract()
        # accumulated real measured travel (_unload_since_switch_mm)
        # between the toolhead switch clearing and the Roadrunner going
        # quiet. If that already covers unload_clear_mm - a placeholder
        # standing in for "roughly how far the extruder-entry-to-
        # Roadrunner span is" - the extruder gear has already been
        # confirmed clear by real motion, and this phase is just a short,
        # small, COUNT-bounded number of extra pulses for margin, never
        # the old fork's blind, unmeasured gear_to_mdm_mm=60 guess
        # (buffer_manager_old/klippy_macros/buffer_manager.cfg:66-67). If
        # it falls short, that is only logged as a warning: the two
        # discrete signals from the retract phase (switch clear + sensor
        # clear) remain the authoritative completion signal - this phase
        # is best-effort confirmation on top of them, not a hard gate.
        if generation != self._unload_generation or self.unload_phase != UNLOAD_CLEAR:
            return
        if self._unload_clear_accum_mm < self.unload_clear_mm:
            logger.warning(
                "buffer_manager %s: unload - only %.2fmm of"
                " encoder-measured retract between the toolhead switch"
                " clearing and the Roadrunner losing filament, short of"
                " the %.2fmm unload_clear_mm margin. Continuing with the"
                " bounded extra-pulse margin anyway - see the"
                " unload_clear_mm config comment.",
                self.name,
                self._unload_clear_accum_mm,
                self.unload_clear_mm,
            )
        gcode_move = self.printer.lookup_object("gcode_move")
        # The Roadrunner is blind from here on - the tip is upstream of it.
        # But the tip sits between the Roadrunner and the BUFFER, so the MDM
        # still has filament through it and can still measure this retract.
        # That is what the two-sensor bracket buys: whichever end the tip has
        # not yet passed is the end that can still see it. Fall back to
        # bounded blind pulses only if the MDM is unavailable or disabled.
        mdm_target_mm = max(0.0, self.unload_clear_mm - self._unload_clear_accum_mm)
        mdm_base = None
        if self.mdm_mm_per_count > 0.0:
            try:
                assert self.backend is not None
                _present, counter = self.backend.mdm()
                if counter is not None:
                    mdm_base = counter
            except Exception:
                logger.exception(
                    "buffer_manager %s: MDM read failed,"
                    " falling back to bounded pulses",
                    self.name,
                )
        if mdm_base is None:
            logger.info(
                "buffer_manager %s: unload - no MDM measurement"
                " available, using %d bounded blind pulses",
                self.name,
                self.unload_clear_pulses,
            )
        mdm_moved_mm = 0.0
        pulses = 0
        while pulses < self.unload_clear_pulses:
            if self._check_unload_abort(generation):
                return
            if self.reactor.monotonic() >= self.unload_deadline:
                self._fail_unload(
                    "timed out during the final post-sensor retract margin (step 4)"
                )
                return
            pulses += 1
            try:
                assert self.backend is not None
                self.backend.run("back", self.pulse_ms)
            except Exception:
                logger.exception(
                    "buffer_manager %s: unload clear run() failed", self.name
                )
                self._fail_unload(
                    "backend run() failed during the final retract margin"
                )
                return
            try:
                self._extrude(
                    gcode_move, -self.unload_pulse_mm, self.unload_retract_speed
                )
            except Exception:
                logger.exception(
                    "buffer_manager %s: unload clear extrude move failed", self.name
                )
                self._fail_unload(
                    "retract extrude move failed during the final retract margin"
                )
                return
            if self._check_unload_abort(generation):
                return
            if mdm_base is not None:
                try:
                    _present, counter = self.backend.mdm()
                except Exception:  # noqa: BLE001
                    counter = None
                if counter is not None:
                    # uint32 counter: wrap-safe unsigned delta.
                    delta = (counter - mdm_base) & 0xFFFFFFFF
                    mdm_moved_mm = delta * self.mdm_mm_per_count
                    if mdm_moved_mm >= mdm_target_mm:
                        logger.info(
                            "buffer_manager %s: unload - MDM confirmed"
                            " %.2fmm of final retract in %d pulses",
                            self.name,
                            mdm_moved_mm,
                            pulses,
                        )
                        break
        else:
            if mdm_base is not None and mdm_moved_mm < mdm_target_mm:
                # Bound reached without the MDM confirming the margin. Not
                # fatal: the switch-clear and sensor-clear signals already
                # said the strand is out. Both encoders are friction
                # coupled and can slip, so this is a heuristic, not proof.
                logger.warning(
                    "buffer_manager %s: unload - MDM measured only %.2fmm"
                    " of the %.2fmm final retract margin within %d pulses;"
                    " the strand is out per the switch and sensor, but the"
                    " tip may be parked shorter than intended",
                    self.name,
                    mdm_moved_mm,
                    mdm_target_mm,
                    self.unload_clear_pulses,
                )
        # Step 5 - stop() then unloaded_gcode.
        self.unload_phase = UNLOAD_DONE
        self.unload_deadline = None
        self.reactor.register_callback(self._stop_then_unloaded)

    def _stop_then_unloaded(self, eventtime):
        self._stop_lane()
        self._exec_template(self.unloaded_template, "unloaded_gcode")

    def _stop_then_unload_failed(self, eventtime):
        self._stop_lane()
        self._exec_template(self.unload_failed_template, "unload_failed_gcode")

    # ------------------------------------------------------------------
    # T10 - MDM/Roadrunner differential monitoring
    # ------------------------------------------------------------------
    def _mdm_delta_counts(self):
        """Counts the MDM has advanced since the last call, or None if the
        MDM is unavailable. Wrap-safe. Used by phase A, where the
        Roadrunner is blind because filament has not reached it yet."""
        if self.mdm_mm_per_count <= 0.0:
            return None
        try:
            assert self.backend is not None
            _present, counter = self.backend.mdm()
        except Exception:
            logger.exception("buffer_manager %s: MDM read failed", self.name)
            return None
        if counter is None:
            return None
        if self._mdm_last_counter is None:
            self._mdm_last_counter = counter
            return 0
        delta = (counter - self._mdm_last_counter) & 0xFFFFFFFF
        self._mdm_last_counter = counter
        return delta

    def _diff_reset(self):
        """Start a fresh comparison window."""
        self._diff_rr_mm = 0.0
        self._diff_mdm_base = None

    def _diff_available(self):
        if self._diff_disabled or not self.use_sensor_pipeline:
            return False
        return self.mdm_mm_per_count > 0.0

    def _diff_sample(self, rr_delta_mm):
        """Feed one pulse's Roadrunner travel into the rolling window.

        Returns a reason string when the two ends disagree over a full
        window, else None. Never raises: a monitoring check must not be
        what breaks an otherwise-good load.
        """
        if not self._diff_available():
            return None
        try:
            assert self.backend is not None
            _present, counter = self.backend.mdm()
        except Exception:
            logger.exception(
                "buffer_manager %s: MDM read failed -"
                " differential monitoring disabled for this"
                " run",
                self.name,
            )
            self._diff_disabled = True
            return None
        if counter is None:
            # Backend has no MDM (e.g. a future non-Mellow board). Say so
            # once, then stay quiet rather than warning every pulse.
            logger.info(
                "buffer_manager %s: no MDM counter available -"
                " differential monitoring inactive",
                self.name,
            )
            self._diff_disabled = True
            return None
        if self._diff_mdm_base is None:
            self._diff_mdm_base = counter
            self._diff_rr_mm = 0.0
            return None
        # Magnitudes only. The Roadrunner's sign depends on its own
        # invert_direction config, which this module does not know, and
        # the MDM counter only ever counts up (it counts edges, not
        # direction). Both ends move the same way during any one phase,
        # so comparing magnitudes is sound and avoids guessing a sign.
        self._diff_rr_mm += abs(rr_delta_mm)
        if self._diff_rr_mm < self.differential_window_mm:
            return None
        # uint32 counter: wrap-safe unsigned delta, same idiom as
        # _run_unload_clear().
        counts = (counter - self._diff_mdm_base) & 0xFFFFFFFF
        mdm_mm = counts * self.mdm_mm_per_count
        rr_mm = self._diff_rr_mm
        # Allowance = a fraction of the window PLUS one full MDM count,
        # because one count is the quantization floor and a +/-1 count
        # reading is not evidence of anything.
        allow = (
            self.differential_window_mm * self.differential_tolerance
            + self.mdm_mm_per_count
        )
        self._diff_reset()
        if abs(mdm_mm - rr_mm) <= allow:
            return None
        # Deliberately does NOT claim "buckling". Both encoders are
        # friction-coupled and either can slip; all that is actually known
        # is that the two ends of the bowden disagree.
        which = (
            "buffer end ahead of toolhead end"
            if mdm_mm > rr_mm
            else "toolhead end ahead of buffer end"
        )
        return (
            f"filament path disagreement: over {rr_mm:.1f}mm at the"
            f" Roadrunner the MDM measured {mdm_mm:.1f}mm ({counts} counts"
            f" x {self.mdm_mm_per_count:.3f}mm), {which}."
            " Either the strand is buckling/compressing inside the"
            " bowden or one of the two encoders is slipping - this"
            " check cannot tell which"
        )

    def _diff_check(self, rr_delta_mm, fail_fn):
        """Sample, and act per differential_action. Returns True if the
        caller should stop (only ever in 'error' mode)."""
        reason = self._diff_sample(rr_delta_mm)
        if reason is None:
            return False
        if self.differential_action == "error":
            fail_fn(reason)
            return True
        logger.warning("buffer_manager %s: %s", self.name, reason)
        self.gcode.respond_info(f"buffer_manager {self.name}: {reason}")
        return False

    def _stop_lane(self):
        try:
            assert self.backend is not None
            self.backend.stop()
        except Exception:
            logger.exception("buffer_manager %s: backend stop failed", self.name)

    def _extrude(self, gcode_move, mm, speed_mm_min):
        """Issue a bounded G1 E<mm> at speed_mm_min, restoring the caller's
        absolute/relative extrude mode afterwards. mm may be negative
        (retract). Shared by phase B's assist pull, the synchronized unload
        retract, and the post-sensor unload clear pulses - the three sites
        that concurrently drive an extruder move alongside a backend
        jog()/run(). Exceptions propagate to the caller, which already wraps
        each call site with its own phase-specific failure message."""
        was_abs = gcode_move.get_status(self.reactor.monotonic())["absolute_extrude"]
        script = f"M83\nG1 E{mm:.2f} F{round(speed_mm_min)}\nM400"
        if was_abs:
            script += "\nM82"
        self.gcode.run_script(script)

    def _exec_template(self, template, label):
        script = template.render().strip()
        if not script:
            return
        try:
            self.gcode.run_script(script)
        except Exception:
            logger.exception("buffer_manager %s: error running %s", self.name, label)

    # ------------------------------------------------------------------
    # G-code commands
    # ------------------------------------------------------------------
    def _sync_arm(self):
        # EV_ARM only fires on a transition, and buffer.c deliberately skips
        # the very first reading after boot (it just seeds prev_arm), so the
        # cached value can be stale or unset when a load is armed. Take a
        # fresh reading rather than trusting it.
        try:
            assert self.backend is not None
            self.arm = self.backend.arm()
        except Exception:
            logger.exception("buffer_manager %s: arm probe failed", self.name)

    def _require_active_role(self, gcmd, action="load"):
        # The lane's tool must be picked up and active - refuse otherwise,
        # never command motion on a docked tool. lll_buffer's 'role'
        # (active|parked|standby, set via LLL_ROLE / SET_ACTIVE_LANE) is
        # this codebase's existing representation of "this lane's tool is
        # the one currently engaged" - see config/printer.cfg.example,
        # where every non-selected lane's role defaults to 'parked'.
        # Fails closed: an unreadable role (backend error) refuses too.
        #
        # Called from both cmd_LOAD_BEGIN and cmd_UNLOAD (T9) - unload
        # commands E-moves too (the sensor pipeline's synchronized
        # retract), so it needs the same precondition: driving an E-move
        # while a different tool is mounted would push/pull filament in
        # the wrong toolhead. `action` only changes the wording below.
        try:
            assert self.backend is not None
            role = self.backend.state().get("role")
        except Exception:
            logger.exception("buffer_manager %s: role check failed", self.name)
            role = None
        if role != "active":
            raise gcmd.error(
                f"buffer_manager {self.lane_name}: refusing to {action} -"
                f" lane role is '{role}', not 'active'. Pick up this lane's"
                f" tool (LLL_ROLE LANE={self.lane_name} ROLE=active) before"
                " commanding motion on it."
            )
        # The role check above is a CONVENTION - it says what the buffer
        # believes, not what the toolchanger actually did. The thing that
        # decides where a 'G1 E' lands is Klipper's currently-active
        # extruder, so check that too: with the wrong tool mounted, a lane
        # marked active would otherwise drive filament into a different
        # toolhead. Fails closed if the toolhead cannot be read.
        try:
            active_ex = self.printer.lookup_object("toolhead").get_extruder()
            active_name = active_ex.get_name()
        except Exception:
            logger.exception(
                "buffer_manager %s: active-extruder check failed", self.name
            )
            active_name = None
        if active_name != self.extruder_name:
            raise gcmd.error(
                f"buffer_manager {self.lane_name}: refusing to {action} -"
                f" this lane feeds '{self.extruder_name}' but the active"
                f" extruder is '{active_name}'. Select this lane's tool"
                " first; E-moves would otherwise go to the wrong toolhead."
            )

    def cmd_LOAD_BEGIN(self, gcmd):
        self._require_active_role(gcmd)
        timeout = gcmd.get_float("TIMEOUT", self.load_timeout, above=0.0)
        self.load_fail_reason = None
        self._phase_abort_reason = None
        self.load_deadline = self.reactor.monotonic() + timeout

        # T10: fresh comparison window per run, and clear the latch so a
        # transient MDM read failure last time does not disable the check
        # forever.
        self._diff_disabled = False
        self._diff_reset()
        self._mdm_last_counter = None
        self._phaseA_nomdm_warned = False
        if self.use_sensor_pipeline:
            self._learned_feed_mm = 0.0
            self._learned_feed_ms = 0
            self._phaseA_stall_streak = 0
            self._phaseB_slip_streak = 0
            self.load_phase = PHASE_A_PUSH
            # Captured here, at the scheduling call site, not inside
            # _run_phase_a's own body - see _load_generation in __init__.
            self._load_generation += 1
            gen = self._load_generation
            self.reactor.register_callback(lambda et, g=gen: self._run_phase_a(et, g))
            gcmd.respond_info(
                f"LLL {self.lane_name}: phase A (push) - pulsing toward the Roadrunner"
                " sensor"
            )
            return

        # No-sensor fallback: arm-settle heuristic over the firmware's own
        # continuous auto-feed (unchanged by T8 - see module docstring).
        self._sync_arm()
        self.load_phase = PHASE_WAIT_FEED
        self.settle_deadline = None
        # Already feeding when armed - skip straight to the feeding phase so
        # an insert that beat us to it is not stuck waiting for a fresh edge.
        if self.arm & ARM_HALL3:
            self.load_phase = PHASE_FEEDING
        self._rearm_load_timer()
        gcmd.respond_info(
            f"LLL {self.lane_name}: waiting for filament to reach the"
            " extruder (no-sensor fallback)"
        )

    def cmd_LOAD_ABORT(self, gcmd):
        self.load_phase = PHASE_IDLE
        self.load_fail_reason = None
        self.settle_deadline = None
        self.load_deadline = None
        # Quiet-cancel sentinel: an in-flight phase A/B loop notices this
        # on its next poll and returns without running load_failed_gcode
        # (load_phase is already IDLE here, not FAILED).
        self._phase_abort_reason = "aborted"
        # Also invalidates any in-flight phase A/B greenlet immediately,
        # independent of whether it has consumed _phase_abort_reason yet -
        # see _load_generation in __init__.
        self._load_generation += 1
        self.reactor.update_timer(self.load_timer, self.reactor.NEVER)
        if self.use_sensor_pipeline:
            # _stop_lane() -> backend.stop() -> gcode.run_script(), which
            # takes gcode.mutex. This handler is itself running under that
            # same mutex (Klipper's mux command dispatch), and the mutex is
            # not reentrant - calling it inline here would deadlock. Defer
            # to a callback greenlet instead, same idiom this module already
            # uses everywhere else a lane-stop needs real gcode dispatch
            # (see _fail_load, _stop_then_unloaded).
            self.reactor.register_callback(lambda et: self._stop_lane())
        gcmd.respond_info(f"LLL {self.lane_name}: load watcher cancelled")

    def cmd_UNLOAD(self, gcmd):
        # Same precondition as load - see _require_active_role()'s
        # docstring comment. Applies to both branches below: the sensor
        # pipeline commands E-moves directly, and even the no-sensor
        # fallback's buffer-only pull has no business running against a
        # docked tool's lane.
        self._require_active_role(gcmd, action="unload")
        timeout = gcmd.get_float("TIMEOUT", self.unload_timeout, above=0.0)
        self.unload_fail_reason = None
        self._unload_abort_reason = None
        self.unload_deadline = self.reactor.monotonic() + timeout
        # T10: see the matching note in cmd_LOAD_BEGIN.
        self._diff_disabled = False
        self._diff_reset()

        if self.use_sensor_pipeline:
            self._unload_stall_streak = 0
            self._unload_since_switch_mm = 0.0
            self._unload_clear_accum_mm = 0.0
            self.unload_phase = UNLOAD_TIP_FORM
            # Captured here, at the scheduling call site, not inside
            # _run_unload_tip_form's own body - see _unload_generation in
            # __init__.
            self._unload_generation += 1
            gen = self._unload_generation
            self.reactor.register_callback(
                lambda et, g=gen: self._run_unload_tip_form(et, g)
            )
            gcmd.respond_info(f"LLL {self.lane_name}: unloading - forming tip")
            return

        # No-sensor fallback: unchanged pre-T8 behaviour - LLL_RUN back,
        # wait for the firmware's own entrance-switch `present` edge.
        ms = gcmd.get_int("MS", self.unload_run_ms, minval=0, maxval=60000)
        self.unload_phase = UNLOAD_RUNNING
        self.reactor.update_timer(self.unload_timer, self.unload_deadline)
        # MS=0 lets the backend bound the run with its own default (the
        # LLL backend falls back to firmware run_max_ms).
        assert self.backend is not None
        self.backend.run("back", ms)
        gcmd.respond_info(f"LLL {self.lane_name}: unloading (no-sensor fallback)")

    def cmd_UNLOAD_ABORT(self, gcmd):
        self.unload_phase = UNLOAD_IDLE
        self.unload_fail_reason = None
        self.unload_deadline = None
        # Quiet-cancel sentinel: an in-flight TIP_FORM/RETRACT/CLEAR loop
        # notices this on its next poll and returns without running
        # unload_failed_gcode (unload_phase is already IDLE here, not
        # FAILED) - same idiom as cmd_LOAD_ABORT.
        self._unload_abort_reason = "aborted"
        # Also invalidates any in-flight TIP_FORM/RETRACT/CLEAR greenlet
        # immediately, independent of whether it has consumed
        # _unload_abort_reason yet - see _unload_generation in __init__.
        self._unload_generation += 1
        self.reactor.update_timer(self.unload_timer, self.reactor.NEVER)
        # Deferred for the same reason as cmd_LOAD_ABORT above: this handler
        # runs under gcode.mutex, and _stop_lane() -> backend.stop() ->
        # gcode.run_script() would try to take it again.
        self.reactor.register_callback(lambda et: self._stop_lane())
        gcmd.respond_info(f"LLL {self.lane_name}: unload cancelled")

    def get_status(self, eventtime):
        learned_rate = None
        if self._learned_feed_ms > 0:
            learned_rate = self._learned_feed_mm / float(self._learned_feed_ms)
        return {
            "lane": self.lane_name,
            "sensor_pipeline": self.use_sensor_pipeline,
            "load_phase": self.load_phase,
            "load_fail_reason": self.load_fail_reason,
            "learned_feed_mm_per_ms": learned_rate,
            "toolhead_switch": self._toolhead_switch_state,
            "unload_phase": self.unload_phase,
            "unload_fail_reason": self.unload_fail_reason,
            "arm": self.arm,
            "hall1": bool(self.arm & ARM_HALL1),
            "hall2": bool(self.arm & ARM_HALL2),
            "hall3": bool(self.arm & ARM_HALL3),
        }


def load_config_prefix(config):
    return BufferManager(config)
