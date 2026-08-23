# Mellow LLL Plus buffer -- CANONICAL v1 Klippy extra (runtime-toggleable firmware)
#
# Pairs with the canonical v1 src/buffer.c. The format strings below must
# stay BYTE-IDENTICAL to buffer.c's PROTOCOL block.
#
# Back-compat preserved with the deployed macros:
#   - method set_idle(v)  (called by SET_ACTIVE_LANE)
#   - gcode LLL_SET_IDLE / LLL_QUERY / LLL_CLEAR_ERROR / LLL_JOG (same params)
#   - get_status keeps the 'idle', 'error', 'feeding' keys
#
# Replay (safety fix #7):
#   - printer.cfg options (static role/feed_mode) -> mcu.add_config_cmd
#     (replayed in the config sequence, BEFORE ready)
#   - runtime tweaks (LLL_SET/LLL_ENABLE/...) -> [save_variables] if present,
#     re-applied at klippy:connect
#
# Config:
#   [lll_buffer lll]
#   mcu: lll                  # name of the board's [mcu]
#   #role: active             # active|parked|standby (compiled default: active)
#   #feed_mode: auto          # auto|semi_auto|semi_manual|manual
import logging
import threading
import json as _json
from urllib import request as _urlreq

# Wire protocol version (T4). Must equal src/buffer.c's LLL_PROTO_VERSION;
# checked against the firmware's `proto` field (in lll_state) at
# klippy:connect. Bump BOTH together on any change to the meaning of a
# command/event/param wire field -- including one that leaves a format
# string's TEXT unchanged (reassigned ev/func/key id, reordered enum, a
# changed bit layout). Klipper's MCU dictionary lookup already rejects a
# format-string text mismatch at identify time on its own; it cannot catch a
# same-shape field whose meaning silently changed, which is what this is for.
PROTOCOL_VERSION = 1

FUNC_IDS = {'feed': 0, 'buttons': 1, 'mbsig': 2, 'entrance': 3, 'tmc': 4,
            'move': 5, 'diag': 6, 'led': 7, 'adc': 8, 'telem': 9, 'ioext': 10}
FUNC_NAMES = {v: k for k, v in FUNC_IDS.items()}
ROLES = {'active': 0, 'parked': 1, 'standby': 2}
ROLE_NAMES = {v: k for k, v in ROLES.items()}
MODES = {'auto': 0, 'semi_auto': 1, 'semi_manual': 2, 'manual': 3}
MODE_NAMES = {v: k for k, v in MODES.items()}
ECODES = {0: 'none', 1: 'feed_timeout', 2: 'back_timeout', 3: 'diag_fault',
          4: 'host_alive', 5: 'clear_pending'}
EVENTS = {0: 'ecode_set', 1: 'error_cleared', 2: 'present', 3: 'diag',
          4: 'button', 5: 'arm', 6: 'role', 7: 'mdm'}
# "flat" param name -> (func, key)
PARAMS = {
    'feed_timeout':  (0, 0), 'back_timeout': (0, 1),
    'gate_by_role':  (1, 0),
    'irun':          (4, 0), 'ihold':        (4, 1), 'iholddelay': (4, 2),
    'mres':          (4, 3), 'stealth':      (4, 4), 'standstill': (4, 5),
    'tpowerdown':    (4, 6), 'speed':        (4, 7),
    'run_max':       (5, 0), 'host_alive':   (5, 1),
    'diag_action':   (6, 0), 'diag_polarity': (6, 1),
    'event_mask':    (9, 0),
}
IO_FUNCS = {'release': 0, 'din': 1, 'dout': 2, 'pwm': 3, 'adc': 4}
DIR_MAP = {'fwd': 1, 'forward': 1, 'back': 2, 'rewind': 2, 'stop': 0}
UNLOCK_KEY = 0xDEB1


class LLLBuffer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu = self.printer.lookup_object('mcu ' + config.get('mcu'))
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        # static options (replayed via add_config_cmd, config sequence)
        self.cfg_role = config.getchoice('role', {k: k for k in ROLES},
                                         'active')
        self.cfg_mode = config.getchoice('feed_mode', {k: k for k in MODES},
                                         'auto')
        # enable: feed,led / disable: buttons -- funcs (de)activated right
        # from the config sequence (full fix #7 for static config)
        self.cfg_enable = [f.strip().lower()
                           for f in config.get('enable', '').split(',')
                           if f.strip()]
        self.cfg_disable = [f.strip().lower()
                            for f in config.get('disable', '').split(',')
                            if f.strip()]
        for f in self.cfg_enable + self.cfg_disable:
            if f not in FUNC_IDS:
                raise config.error("lll_buffer %s: unknown func '%s'"
                                   % (self.name, f))
        # state poll cadence (keepalive) -- must stay < host_alive if enabled
        self.poll_interval = config.getfloat('poll_interval', 1.5,
                                             minval=0.3, maxval=30.)
        # event hook: gcode triggered on EVERY firmware event push
        # (~20-50 ms reaction instead of waiting for the poll). Jinja context:
        # event.name / event.code / event.value / event.lane
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.has_event_gcode = config.get('event_gcode', None) is not None
        self.event_template = gcode_macro.load_template(config, 'event_gcode',
                                                        '')
        self._in_event_gcode = False
        # state seen by get_status
        self.state = {'role': 'active', 'mode': 'auto', 'error': 0,
                      'ecode': 'none', 'feeding': 0, 'running': 0, 'arm': 0,
                      'present': 0, 'diag': 0, 'cfgerr': 0, 'idle': 0,
                      'mdm_present': 0, 'mdm_count': 0, 'proto': 0,
                      'last_event': '', 'filament': {}}
        self._prev_error = 0
        self._query_busy = False
        self.mcu.register_config_callback(self._build_config)
        self.poll_timer = self.reactor.register_timer(self._poll)
        self.printer.add_object('lll_buffer ' + self.name, self)
        self.printer.register_event_handler('klippy:connect', self._connect)
        for cmd, fn, d in [
                ('LLL_ROLE', self.cmd_ROLE, 'Set lane role active|parked|standby'),
                ('LLL_SET_IDLE', self.cmd_SET_IDLE, 'Freeze(1)/feed(0) a lane (compat)'),
                ('LLL_ENABLE', self.cmd_ENABLE, 'Enable/disable a firmware function'),
                ('LLL_MODE', self.cmd_MODE, 'Set FEED automation level'),
                ('LLL_SET', self.cmd_SET, 'Set a runtime parameter'),
                ('LLL_ARM', self.cmd_ARM, 'Arm one feed cycle (semi_auto)'),
                ('LLL_JOG', self.cmd_JOG, 'Jog a lane fwd/back/stop for MS ms'),
                ('LLL_RUN', self.cmd_RUN, 'Bounded run fwd/back (MS=0 -> run_max)'),
                ('LLL_STOP', self.cmd_STOP, 'Stop host motion (autonomy resumes)'),
                ('LLL_CLEAR_ERROR', self.cmd_CLEAR, 'Clear a lane jam/error'),
                ('LLL_QUERY', self.cmd_QUERY, 'Report lane state'),
                ('LLL_UNLOCK', self.cmd_UNLOCK, 'Unlock unverified IOEXT pins'),
                ('LLL_PROFILE_SAVE', self.cmd_PROFILE_SAVE,
                 'Save current tweaks as a named profile'),
                ('LLL_PROFILE_LOAD', self.cmd_PROFILE_LOAD,
                 'Apply a named profile (persists as current)'),
                ('LLL_PROFILE_LIST', self.cmd_PROFILE_LIST,
                 'List saved profiles'),
                ('LLL_PROFILE_DELETE', self.cmd_PROFILE_DELETE,
                 'Delete a named profile'),
                ('LLL_FILAMENT', self.cmd_FILAMENT,
                 'Register filament type/color loaded on this lane'),
                ('LLL_IO_CFG', self.cmd_IO_CFG, 'Configure an IOEXT channel'),
                ('LLL_IO_WRITE', self.cmd_IO_WRITE, 'Write an IOEXT dout/pwm'),
                ('LLL_IO_READ', self.cmd_IO_READ, 'Read an IOEXT channel')]:
            self.gcode.register_mux_command(cmd, 'LANE', self.name, fn, desc=d)
        # GLOBAL command (not mux): detects the lane physically loaded by
        # probing the arm (only one instance registers it ; the
        # handler enumerates all lanes at runtime)
        try:
            self.gcode.register_command(
                'LLL_DETECT_LOADED', self.cmd_DETECT_LOADED,
                desc='Probe which lane is physically loaded (arm sensors)')
        except Exception:
            pass                         # already registered by another lane

    # ------------------------------------------------------------------
    def _build_config(self):
        cq = self.mcu.alloc_command_queue()
        mcu = self.mcu
        self.role_cmd = mcu.lookup_command("lll_role role=%c", cq=cq)
        self.set_idle_cmd = mcu.lookup_command("lll_set_idle value=%c", cq=cq)
        # lll_arm always ACKs -> query wrapper MANDATORY: an orphan ack
        # would be stolen by the next command waiting on 'lll_ack'
        self.arm_cmd = mcu.lookup_query_command(
            "lll_arm", "lll_ack func=%c key=%c val=%u ok=%c", cq=cq)
        self.jog_cmd = mcu.lookup_command("lll_jog dir=%c ms=%u", cq=cq)
        self.run_cmd = mcu.lookup_command("lll_run dir=%c ms=%u", cq=cq)
        self.stop_cmd = mcu.lookup_command("lll_stop", cq=cq)
        self.clear_cmd = mcu.lookup_command("lll_clear_error", cq=cq)
        self.io_write_cmd = mcu.lookup_command("lll_io_write ch=%c val=%u",
                                               cq=cq)
        self.query_cmd = mcu.lookup_query_command(
            "lll_query",
            "lll_state role=%c mode=%c error=%c ecode=%c feeding=%c"
            " running=%c arm=%c present=%c diag=%c cfgerr=%c"
            " mdm_present=%c mdm_count=%u proto=%c", cq=cq)
        self.enable_cmd = mcu.lookup_query_command(
            "lll_enable func=%c en=%c",
            "lll_ack func=%c key=%c val=%u ok=%c", cq=cq)
        self.mode_cmd = mcu.lookup_query_command(
            "lll_mode func=%c level=%c",
            "lll_ack func=%c key=%c val=%u ok=%c", cq=cq)
        self.param_cmd = mcu.lookup_query_command(
            "lll_param func=%c key=%c val=%u",
            "lll_ack func=%c key=%c val=%u ok=%c", cq=cq)
        self.unlock_cmd = mcu.lookup_query_command(
            "lll_unlock key=%u",
            "lll_ack func=%c key=%c val=%u ok=%c", cq=cq)
        self.io_cfg_cmd = mcu.lookup_query_command(
            "lll_io_cfg ch=%c func=%c arg=%u",
            "lll_io_ack ch=%c func=%c ok=%c", cq=cq)
        self.io_read_cmd = mcu.lookup_query_command(
            "lll_io_read ch=%c",
            "lll_io ch=%c val=%u ok=%c", cq=cq)
        self.mcu.register_serial_response(self._handle_event,
                                          "lll_event ev=%c a=%u")
        # replay the static options in the config sequence (fix #7)
        if self.cfg_role != 'active':
            self.mcu.add_config_cmd("lll_role role=%d" % ROLES[self.cfg_role])
        if self.cfg_mode != 'auto':
            self.mcu.add_config_cmd("lll_mode func=0 level=%d"
                                    % MODES[self.cfg_mode])
        # NB: each lll_enable/lll_mode config command triggers a firmware
        # lll_ack with no waiter -> 1 benign 'got {...}' warning in klippy.log
        # per command at config time (dispatched by name, confirmed harmless)
        for f in self.cfg_enable:
            self.mcu.add_config_cmd("lll_enable func=%d en=1" % FUNC_IDS[f])
        for f in self.cfg_disable:
            self.mcu.add_config_cmd("lll_enable func=%d en=0" % FUNC_IDS[f])

    # ------------------------------------------------------------------
    def _connect(self):
        self._check_protocol_version()
        self._replay_saved()
        sv = self.printer.lookup_object('save_variables', None)
        if sv is not None:
            fil = sv.allVariables.get(self._filament_var(), {})
            if isinstance(fil, dict):
                self.state['filament'] = fil
        self.reactor.update_timer(self.poll_timer, self.reactor.NOW)

    # T4: first exchange with the board -> verifies the firmware speaks the
    # same protocol as this module BEFORE any replay/poll. A mismatch on a
    # field that does not change the format string's shape (a reassigned
    # event/func/key id, a reordered enum) would slip through Klipper's MCU
    # dictionary lookup without raising an error -> this counter is the only
    # net for that case ; refuse to start rather than silently misparsing.
    def _check_protocol_version(self):
        try:
            self._query_busy = True
            p = self.query_cmd.send([])
        finally:
            self._query_busy = False
        fw_proto = p['proto']
        if fw_proto != PROTOCOL_VERSION:
            raise self.printer.config_error(
                "lll_buffer %s: protocol version mismatch - firmware speaks"
                " proto=%d, this klippy/extras/lll_buffer.py speaks"
                " proto=%d. Reflash the firmware from this repo's src/"
                " buffer.c or update lll_buffer.py to match - refusing to"
                " start rather than silently misparsing lll_state."
                % (self.name, fw_proto, PROTOCOL_VERSION))
        self._update_state(p)

    def _filament_var(self):
        return ('lll_%s_filament' % self.name).lower()

    def _saved_var(self):
        # save_variables refuses uppercase in names
        return ('lll_%s_tweaks' % self.name).lower()

    def _profiles_var(self):
        return ('lll_%s_profiles' % self.name).lower()

    def _sv_write(self, varname, value):
        sv = self.printer.lookup_object('save_variables', None)
        if sv is None:
            return False
        cmd = self.gcode.create_gcode_command(
            'SAVE_VARIABLE', 'SAVE_VARIABLE',
            {'VARIABLE': varname, 'VALUE': repr(value)})
        sv.cmd_SAVE_VARIABLE(cmd)
        return True

    # applies a tweak (key of the persisted dict) ; returns the firmware
    # response dict (or None for commands with no ack)
    def _apply_tweak(self, k, v):
        if k.startswith('param_'):
            func, key = PARAMS[k[6:]]
            return self.param_cmd.send([func, key, int(v)])
        if k.startswith('enable_'):
            return self.enable_cmd.send([FUNC_IDS[k[7:]], int(v)])
        if k == 'role':
            self.role_cmd.send([ROLES[v]])
            return None
        if k == 'mode':
            return self.mode_cmd.send([0, MODES[v]])
        raise KeyError(k)

    def _replay_saved(self):
        sv = self.printer.lookup_object('save_variables', None)
        if sv is None:
            return
        tweaks = sv.allVariables.get(self._saved_var(), {})
        if not isinstance(tweaks, dict):
            return
        # try PER ITEM: a stale key must not abort the rest
        # (notably role, replayed last by alphabetical order)
        for k, v in sorted(tweaks.items()):
            try:
                p = self._apply_tweak(k, v)
                if p is not None and not p.get('ok', 1):
                    logging.warning("lll_buffer %s: replay tweak %s=%s"
                                    " REFUSED by firmware", self.name, k, v)
            except Exception:
                logging.exception("lll_buffer %s: replay tweak %s failed",
                                  self.name, k)

    def _persist(self, key, val):
        sv = self.printer.lookup_object('save_variables', None)
        if sv is None:
            return
        tweaks = dict(sv.allVariables.get(self._saved_var(), {}))
        tweaks[key] = val
        try:
            self._sv_write(self._saved_var(), tweaks)
        except Exception:
            logging.exception("lll_buffer %s: persist failed", self.name)

    # ------------------------------------------------------------------
    # NAMED PROFILES (BEACON_MODEL_SELECT style): a profile = a bundle of
    # tweaks (mode/params/enables -- NOT role: that belongs to the swap
    # system). LOAD applies ITS settings and persists them as the current
    # state ; whatever a profile does not define stays as-is.
    # ------------------------------------------------------------------
    def _get_profiles(self):
        sv = self.printer.lookup_object('save_variables', None)
        if sv is None:
            return None
        profs = sv.allVariables.get(self._profiles_var(), {})
        return dict(profs) if isinstance(profs, dict) else {}

    def cmd_PROFILE_SAVE(self, gcmd):
        profs = self._get_profiles()
        if profs is None:
            raise gcmd.error("[save_variables] required in printer.cfg")
        name = gcmd.get('NAME').lower()
        sv = self.printer.lookup_object('save_variables')
        tweaks = dict(sv.allVariables.get(self._saved_var(), {}))
        tweaks.pop('role', None)         # role does not belong in a profile
        if not tweaks:
            raise gcmd.error("LLL %s: nothing to save (no setting applied yet"
                             " - use LLL_SET/LLL_MODE/LLL_ENABLE first)"
                             % self.name)
        profs[name] = tweaks
        self._sv_write(self._profiles_var(), profs)
        gcmd.respond_info("LLL %s: profile '%s' saved (%s)"
                          % (self.name, name,
                             ', '.join('%s=%s' % kv
                                       for kv in sorted(tweaks.items()))))

    def cmd_PROFILE_LOAD(self, gcmd):
        profs = self._get_profiles()
        if profs is None:
            raise gcmd.error("[save_variables] required in printer.cfg")
        name = gcmd.get('NAME').lower()
        if name not in profs:
            raise gcmd.error("LLL %s: unknown profile '%s' (available: %s)"
                             % (self.name, name,
                                ', '.join(sorted(profs)) or 'none'))
        refused = []
        for k, v in sorted(profs[name].items()):
            try:
                p = self._apply_tweak(k, v)
                if p is not None and not p.get('ok', 1):
                    refused.append(k)
                else:
                    self._persist(k, v)  # becomes the current state (reboot-safe)
            except Exception:
                logging.exception("lll_buffer %s: profile %s item %s failed",
                                  self.name, name, k)
                refused.append(k)
        if refused:
            raise gcmd.error("LLL %s: profile '%s' applied PARTIALLY,"
                             " refused: %s" % (self.name, name,
                                              ', '.join(refused)))
        gcmd.respond_info("LLL %s: profile '%s' applied (%d settings)"
                          % (self.name, name, len(profs[name])))

    def cmd_PROFILE_LIST(self, gcmd):
        profs = self._get_profiles()
        if not profs:
            gcmd.respond_info("LLL %s: no profiles saved" % self.name)
            return
        lines = ["LLL %s: %d profile(s)" % (self.name, len(profs))]
        for name in sorted(profs):
            lines.append("  %s: %s" % (name,
                         ', '.join('%s=%s' % kv
                                   for kv in sorted(profs[name].items()))))
        gcmd.respond_info('\n'.join(lines))

    def cmd_PROFILE_DELETE(self, gcmd):
        profs = self._get_profiles()
        if profs is None:
            raise gcmd.error("[save_variables] required in printer.cfg")
        name = gcmd.get('NAME').lower()
        if name not in profs:
            raise gcmd.error("LLL %s: unknown profile '%s'" % (self.name, name))
        del profs[name]
        self._sv_write(self._profiles_var(), profs)
        gcmd.respond_info("LLL %s: profile '%s' deleted" % (self.name, name))

    # ------------------------------------------------------------------
    def _handle_event(self, params):
        # WARNING: runs in the background serial thread -> trampoline
        # to the reactor before any respond_info / state mutation
        ev, a = params['ev'], params['a']
        self.reactor.register_async_callback(
            lambda e, ev=ev, a=a: self._process_event(ev, a))

    def _process_event(self, ev, a):
        name = EVENTS.get(ev, str(ev))
        self.state['last_event'] = '%s:%d' % (name, a)
        if ev == 0:
            self.gcode.respond_info(
                "LLL %s: ecode=%s%s" % (self.name, ECODES.get(a, a),
                                        " - resolve then 'LLL_CLEAR_ERROR"
                                        " LANE=%s'" % self.name if a else ""))
        elif ev == 1:
            self.gcode.respond_info("LLL %s: error cleared" % self.name)
        elif ev == 3:
            self.gcode.respond_info("LLL %s: DIAG %s"
                                    % (self.name, 'ASSERT' if a else 'clear'))
        # event_gcode hook (fast strategic reaction, ~20-50 ms)
        if not self.has_event_gcode or self._in_event_gcode:
            return                       # absent or re-entrance (event triggered
        try:                             # by the hook itself) -> skip
            self._in_event_gcode = True
            ctx = self.event_template.create_template_context()
            ctx['event'] = {'name': name, 'code': ev, 'value': a,
                            'lane': self.name}
            self.gcode.run_script(self.event_template.render(ctx))
        except Exception:
            logging.exception("lll_buffer %s: event_gcode failed", self.name)
        finally:
            self._in_event_gcode = False

    def _update_state(self, p):
        s = self.state
        s['role'] = ROLE_NAMES.get(p['role'], p['role'])
        s['mode'] = MODE_NAMES.get(p['mode'], p['mode'])
        s['error'] = p['error']
        s['ecode'] = ECODES.get(p['ecode'], p['ecode'])
        s['feeding'] = p['feeding']
        s['running'] = p['running']
        s['arm'] = p['arm']
        s['present'] = p['present']
        s['diag'] = p['diag']
        s['cfgerr'] = p['cfgerr']
        s['mdm_present'] = p['mdm_present']
        s['mdm_count'] = p['mdm_count']                 # uint32_t, wraps;
                                                        # diff wrap-safe (T3)
        s['proto'] = p['proto']
        s['idle'] = 1 if p['role'] else 0              # compat macros
        if p['error'] and not self._prev_error:        # rising edge = jam
            self.gcode.respond_info(
                "LLL %s: JAM (%s) - resolve then 'LLL_CLEAR_ERROR LANE=%s'"
                " or a short button click"
                % (self.name, s['ecode'], self.name))
        self._prev_error = p['error']

    def _poll(self, eventtime):
        # poll = host-alive keepalive ; get_status NEVER sends
        if self._query_busy:
            return eventtime + self.poll_interval  # a gcode query is in
        try:                             # flight: not 2 'lll_state' waiters
            self._query_busy = True
            self._update_state(self.query_cmd.send([]))
        except Exception:
            logging.exception("lll_buffer %s: poll failed", self.name)
        finally:
            self._query_busy = False
        return eventtime + self.poll_interval

    # ------------------------------------------------------------------
    # called by SET_ACTIVE_LANE (active -> 0, inactive -> 1) -- compat
    def set_idle(self, v):
        self.set_idle_cmd.send([1 if v else 0])
        self.state['role'] = 'parked' if v else 'active'
        self.state['idle'] = 1 if v else 0
        if not v:
            self._push_active_spool()

    def cmd_SET_IDLE(self, gcmd):
        v = gcmd.get_int('VALUE', minval=0, maxval=1)
        self.set_idle_cmd.send([v])
        self.state['role'] = 'parked' if v else 'active'
        self.state['idle'] = v
        if v == 0:
            self._push_active_spool()    # this lane becomes active

    def cmd_ROLE(self, gcmd):
        r = gcmd.get('ROLE').lower()
        if r not in ROLES:
            raise gcmd.error("ROLE must be active|parked|standby")
        self.role_cmd.send([ROLES[r]])
        self.state['role'] = r
        self.state['idle'] = 0 if r == 'active' else 1
        self._persist('role', r)
        if r == 'active':
            self._push_active_spool()
        gcmd.respond_info("LLL %s: role=%s" % (self.name, r))

    def cmd_ENABLE(self, gcmd):
        f = gcmd.get('FUNC').lower()
        if f not in FUNC_IDS:
            raise gcmd.error("FUNC must be one of %s" % '|'.join(FUNC_IDS))
        en = gcmd.get_int('EN', minval=0, maxval=1)
        p = self.enable_cmd.send([FUNC_IDS[f], en])
        if not p['ok']:
            raise gcmd.error("LLL %s: enable %s=%d REFUSED (pin conflict?)"
                             % (self.name, f, en))
        self._persist('enable_' + f, en)
        gcmd.respond_info("LLL %s: %s %s" % (self.name, f,
                                             'enabled' if en else 'disabled'))

    def cmd_MODE(self, gcmd):
        m = gcmd.get('LEVEL').lower()
        if m not in MODES:
            raise gcmd.error("LEVEL must be auto|semi_auto|semi_manual|manual")
        p = self.mode_cmd.send([0, MODES[m]])
        if not p['ok']:
            raise gcmd.error("LLL %s: mode refused" % self.name)
        self._persist('mode', m)
        gcmd.respond_info("LLL %s: feed mode=%s" % (self.name, m))

    def cmd_SET(self, gcmd):
        name = gcmd.get('PARAM').lower()
        if name not in PARAMS:
            raise gcmd.error("PARAM must be one of %s" % '|'.join(sorted(PARAMS)))
        val = gcmd.get_int('VALUE', minval=0)
        func, key = PARAMS[name]
        p = self.param_cmd.send([func, key, val])
        if not p['ok']:
            raise gcmd.error("LLL %s: %s=%d REFUSED" % (self.name, name, val))
        self._persist('param_' + name, p['val'])
        gcmd.respond_info("LLL %s: %s=%d (applied=%d)"
                          % (self.name, name, val, p['val']))

    def cmd_ARM(self, gcmd):
        p = self.arm_cmd.send([])
        if not p['ok']:
            raise gcmd.error("LLL %s: arm refused (feed mode must be"
                             " semi_auto)" % self.name)
        gcmd.respond_info("LLL %s: armed (one feed cycle)" % self.name)

    def cmd_JOG(self, gcmd):
        d = gcmd.get('DIR', 'stop').lower()
        if d not in DIR_MAP:
            raise gcmd.error("DIR must be fwd|back|stop")
        ms = gcmd.get_int('MS', 500, minval=0, maxval=10000)
        self.jog_cmd.send([DIR_MAP[d], ms])
        gcmd.respond_info("LLL %s: jog %s %dms" % (self.name, d, ms))

    def cmd_RUN(self, gcmd):
        d = gcmd.get('DIR', 'stop').lower()
        if d not in DIR_MAP:
            raise gcmd.error("DIR must be fwd|back|stop")
        ms = gcmd.get_int('MS', 0, minval=0, maxval=60000)
        self.run_cmd.send([DIR_MAP[d], ms])
        gcmd.respond_info("LLL %s: run %s (ms=%d, 0=run_max)"
                          % (self.name, d, ms))

    def cmd_STOP(self, gcmd):
        self.stop_cmd.send([])
        gcmd.respond_info("LLL %s: stopped (autonomy resumes per role/mode)"
                          % self.name)

    def cmd_CLEAR(self, gcmd):
        self.clear_cmd.send([])
        gcmd.respond_info("LLL %s: clear requested" % self.name)

    def cmd_QUERY(self, gcmd):
        if self._query_busy:             # poll in flight: avoids 2 waiters
            s = self.state               # on 'lll_state' -> answer from cache
            gcmd.respond_info(
                "LLL %s (cache <1.5s): role=%s mode=%s error=%s ecode=%s"
                " feeding=%s running=%s arm=%s present=%s diag=%s cfgerr=%s"
                % (self.name, s['role'], s['mode'], s['error'], s['ecode'],
                   s['feeding'], s['running'], s['arm'], s['present'],
                   s['diag'], s['cfgerr']))
            return
        try:
            self._query_busy = True
            p = self.query_cmd.send([])
        finally:
            self._query_busy = False
        self._update_state(p)            # freshness on demand: a macro can
                                         # do LLL_QUERY then read
                                         # get_status (no longer bounded by the poll)
        gcmd.respond_info(
            "LLL %s: role=%s mode=%s error=%d ecode=%s feeding=%d running=%d"
            " arm=%d present=%d diag=%d cfgerr=%d"
            % (self.name, ROLE_NAMES.get(p['role'], p['role']),
               MODE_NAMES.get(p['mode'], p['mode']), p['error'],
               ECODES.get(p['ecode'], p['ecode']), p['feeding'], p['running'],
               p['arm'], p['present'], p['diag'], p['cfgerr']))

    def cmd_UNLOCK(self, gcmd):
        try:
            key = int(gcmd.get('KEY', '0'), 0)
        except ValueError:
            raise gcmd.error("KEY must be numeric (hex ok, ex: KEY=0xDEB1)")
        if key != UNLOCK_KEY:
            raise gcmd.error("LLL %s: bad key (unverified pins stay locked)"
                             % self.name)
        p = self.unlock_cmd.send([key])
        gcmd.respond_info("LLL %s: unverified IOEXT pins %s"
                          % (self.name, 'UNLOCKED' if p['ok'] else 'locked'))

    def cmd_IO_CFG(self, gcmd):
        ch = gcmd.get_int('CH', minval=0, maxval=11)
        f = gcmd.get('FUNC').lower()
        if f not in IO_FUNCS:
            raise gcmd.error("FUNC must be release|din|dout|pwm|adc")
        if f == 'din':
            pull = {'none': 0, 'up': 1, 'down': 2}.get(
                gcmd.get('PULL', 'none').lower())
            if pull is None:
                raise gcmd.error("PULL must be none|up|down")
            arg = pull
        elif f == 'dout':
            arg = (gcmd.get_int('INIT', 0, minval=0, maxval=1)
                   | (gcmd.get_int('SHUTDOWN', 0, minval=0, maxval=1) << 1)
                   | (gcmd.get_int('MAXON', 0, minval=0, maxval=65535) << 8))
        elif f == 'pwm':
            arg = gcmd.get_int('PERIOD', 100, minval=10, maxval=10000)
        else:
            arg = 0
        p = self.io_cfg_cmd.send([ch, IO_FUNCS[f], arg])
        if not p['ok']:
            raise gcmd.error(
                "LLL %s: io_cfg ch%d %s REFUSED (unlock? module conflict?"
                " non-ADC channel?)" % (self.name, ch, f))
        gcmd.respond_info("LLL %s: ch%d = %s" % (self.name, ch, f))

    def cmd_IO_WRITE(self, gcmd):
        ch = gcmd.get_int('CH', minval=0, maxval=11)
        val = gcmd.get_int('VALUE', minval=0, maxval=255)
        self.io_write_cmd.send([ch, val])

    def cmd_IO_READ(self, gcmd):
        ch = gcmd.get_int('CH', minval=0, maxval=11)
        p = self.io_read_cmd.send([ch])
        if not p['ok']:
            raise gcmd.error("LLL %s: ch%d not configured/readable"
                             % (self.name, ch))
        gcmd.respond_info("LLL %s: ch%d = %d" % (self.name, ch, p['val']))

    def cmd_FILAMENT(self, gcmd):
        info = {'type': gcmd.get('TYPE', ''),
                'color': gcmd.get('COLOR', ''),
                'temp': gcmd.get_int('TEMP', 0, minval=0, maxval=350),
                'spool_id': gcmd.get_int('SPOOL_ID', 0, minval=0)}
        self.state['filament'] = info
        self._sv_write(self._filament_var(), info)
        gcmd.respond_info("LLL %s: filament = %s %s%s%s"
                          % (self.name, info['type'], info['color'],
                             ' (%dC)' % info['temp'] if info['temp'] else '',
                             ' [spool #%d]' % info['spool_id']
                             if info['spool_id'] else ''))

    # push THIS lane's Spoolman spool as Moonraker's active spool
    # (consumption tracking then counts against the right spool, per lane).
    # HTTP in a daemon thread: never blocking for the klippy reactor.
    def _push_active_spool(self):
        sid = (self.state.get('filament') or {}).get('spool_id', 0)
        if not sid:
            return
        def worker(spool_id):
            try:
                req = _urlreq.Request(
                    'http://localhost:7125/server/spoolman/spool_id',
                    data=_json.dumps({'spool_id': spool_id}).encode(),
                    headers={'Content-Type': 'application/json'},
                    method='POST')
                _urlreq.urlopen(req, timeout=3).read()
            except Exception:
                logging.exception("lll_buffer %s: spoolman push failed",
                                  self.name)
        threading.Thread(target=worker, args=(sid,), daemon=True).start()

    # probe helpers (used by LLL_DETECT_LOADED, all lanes)
    def probe_arm(self):
        if self._query_busy:
            self.reactor.pause(self.reactor.monotonic() + 0.2)
        try:
            self._query_busy = True
            p = self.query_cmd.send([])
            self._update_state(p)
            return p['arm'], p['role']
        finally:
            self._query_busy = False

    def _fil_desc(self):
        f = self.state.get('filament') or {}
        parts = [f.get('type', ''), f.get('color', '')]
        s = ' '.join(x for x in parts if x)
        if f.get('temp'):
            s += ' (%dC)' % f['temp']
        return (' — ' + s) if s else ''

    # ------------------------------------------------------------------
    # LLL_DETECT_LOADED [MM=10] [EXPECTED=<lane>] [ON_MISMATCH=warn|pause|error]
    # Probes which lane is physically loaded: all lanes to standby, a small
    # extruder retract -> ONLY the engaged lane's arm moves -> verdict.
    # Requires an extruder >= min_extrude_temp (Klipper blocks a cold E-move,
    # and the filament would be frozen in the meltzone = guaranteed slip).
    # ------------------------------------------------------------------
    def cmd_DETECT_LOADED(self, gcmd):
        mm = gcmd.get_float('MM', 10., minval=2., maxval=50.)
        expected = gcmd.get('EXPECTED', '').lower()
        on_mismatch = gcmd.get('ON_MISMATCH', 'warn').lower()
        eventtime = self.reactor.monotonic()
        extruder = self.printer.lookup_object('extruder')
        if not extruder.get_status(eventtime).get('can_extrude', False):
            raise gcmd.error(
                "LLL probe: extruder cold - heat above min_extrude_temp"
                " first (e.g. M109 S175) then retry")
        lanes = [obj for _, obj in self.printer.lookup_objects('lll_buffer')]
        if expected and expected not in [o.name for o in lanes]:
            raise gcmd.error("LLL probe: unknown lane EXPECTED '%s'"
                             % expected)
        arms0, saved_roles = {}, {}
        gm_abs = self.printer.lookup_object('gcode_move').get_status(
            eventtime)['absolute_extrude']
        changed = []
        try:
            for o in lanes:
                arm, role = o.probe_arm()
                arms0[o.name] = arm
                saved_roles[o.name] = role
                o.role_cmd.send([2])     # standby: nobody else compensates
            self.gcode.run_script_from_command(
                "M83\nG1 E-%.1f F300\nM400" % (mm,))
            self.reactor.pause(self.reactor.monotonic() + 0.6)
            for o in lanes:
                arm, _ = o.probe_arm()
                if arm != arms0[o.name]:
                    changed.append(o.name)
            self.gcode.run_script_from_command(
                "G1 E%.1f F300\nM400%s" % (mm, "\nM82" if gm_abs else ""))
        finally:
            for o in lanes:              # restore the roles no matter what happens
                if o.name in saved_roles:
                    try:
                        o.role_cmd.send([saved_roles[o.name]])
                    except Exception:
                        logging.exception("lll probe: role restore %s failed",
                                          o.name)
        if len(changed) == 1:
            loaded = changed[0]
            lane_obj = [o for o in lanes if o.name == loaded][0]
            self._sv_write('lll_loaded_lane', loaded)
            if expected and loaded != expected:
                self._probe_fail(gcmd, on_mismatch,
                                 "expected '%s', DETECTED '%s'%s"
                                 % (expected, loaded, lane_obj._fil_desc()))
                return
            gcmd.respond_info("LLL probe: lane '%s' loaded%s%s"
                              % (loaded, lane_obj._fil_desc(),
                                 ' - match OK' if expected else ''))
        elif not changed:
            self._probe_fail(gcmd, on_mismatch if expected else 'warn',
                             "no arm moved (increase MM=, check the"
                             " temperature and the loading)")
        else:
            self._probe_fail(gcmd, on_mismatch if expected else 'warn',
                             "several arms moved (%s) - retry"
                             % ', '.join(changed))

    def _probe_fail(self, gcmd, action, detail):
        if action == 'error':
            raise gcmd.error('FILAMENT CHECK: ' + detail)
        if action == 'pause':
            self.gcode.run_script_from_command("PAUSE")
            gcmd.respond_info(
                "!! FILAMENT CHECK: %s. Load the correct lane, verify with"
                " LLL_DETECT_LOADED, then RESUME." % detail)
        else:
            gcmd.respond_info("LLL probe: %s" % detail)

    # ------------------------------------------------------------------
    def get_status(self, eventtime):
        return dict(self.state)


def load_config_prefix(config):
    return LLLBuffer(config)
