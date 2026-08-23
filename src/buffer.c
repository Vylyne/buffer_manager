// LLL Buffer Plus -- CANONICAL v1 firmware "runtime-toggleable" (one-flash 128 KB)
//
// Implements the CANONICAL v1 SPEC (2026-07-02, user-validated): the autonomous
// smart-buffer core of the Mellow standalone (ported word-for-word, hall/
// VACTUAL/IHOLD_IRUN conventions identical to the deployed variant) + a full
// Klipper host control layer: 11 runtime-toggleable functions, 4 automation
// levels for FEED, index-only IOEXT registry, telemetry events.
//
// ADDED file (does not modify any existing Klipper file) -> survives git pull.
// Required Makefile hook: src-$(CONFIG_MACH_STM32F072) += buffer.c
//
// ---------------------------------------------------------------------------
// SAFETY (the 8 fixes from spec §1, all present):
//  1. sched_is_shutdown() guard = LITERALLY the 1st line of buffer_task
//  2. SINGLE-WRITER: command handlers NEVER touch EN/UART --
//     flags only; only buffer_task/apply_state + buffer_shutdown write
//  3. run FWD requires present && !is_error (checked EVERY tick); BACK allowed
//     on runout (legitimate unload)
//  4. back_timeout_ms (def. 60 s) on the parked freeze-follow AND on run back
//  5. PB8/PB9 (5 V headers) + PB14/PB15 (MDM host) OUT of the IOEXT table
//     (index-only => physically unreachable)
//  6. every STOP writes VACTUAL=0 BEFORE EN=1; every move rewrites
//     GCONF+VACTUAL (apply_state); after a shutdown, tmc_ready=0 => resync
//  7. compiled defaults = role active / mode auto (north-star "keep
//     buffering"); the host replays role/mode/enables via add_config_cmd
//  8. NO UART bit-bang in handlers; runtime TMC writes are deferred
//     and spaced (>=2 ms) in the task
//  9. BOOT GRACE PERIOD (fix for the "compiled defaults" window, finding
//     from bench A9): no autonomous motion from step 9 while the host has
//     NEVER spoken AND < 20 s since boot -- closes off the phantom push from
//     a clean MCU reset (FIRMWARE_RESTART / M112 / power-on). Accepted
//     standalone trade-off: a board that is never driven waits 20 s before
//     auto-feed (the original Mellow firmware starts ~1 s after boot).
//     grace_done latches on each tick => never re-armed (g_ms wrap included)
//
// ---------------------------------------------------------------------------
// PROTOCOL (block shared between firmware <-> klippy/extras/lll_buffer.py --
//           the formats MUST stay BYTE-IDENTICAL on both sides):
//
//  lll_role     role=%c            0=active 1=parked 2=standby
//  lll_set_idle value=%c           back-compat ALIAS (1->parked, 0->active)
//  lll_enable   func=%c en=%c      -> lll_ack func key=0xFE val=en ok
//  lll_mode     func=%c level=%c   (func 0 only) 0=auto 1=semi-auto
//                                  2=semi-manual 3=manual -> ack key=0xFD
//  lll_param    func=%c key=%c val=%u -> "lll_ack func=%c key=%c val=%u ok=%c"
//  lll_arm                         semi-auto: arms ONE cycle -> ack key=0xFB
//  lll_jog      dir=%c ms=%u       0=stop 1=fwd 2=back ; ms clamped 1..10000
//  lll_run      dir=%c ms=%u       bounded ; ms=0 -> run_max_ms ; refreshable
//  lll_stop                        stop motor + clear jog/run/arm (flags only)
//  lll_clear_error                 refused (ecode=5) if DIAG still asserted
//  lll_query                       -> "lll_state role=%c mode=%c error=%c
//                                      ecode=%c feeding=%c running=%c arm=%c
//                                      present=%c diag=%c cfgerr=%c
//                                      mdm_present=%c mdm_count=%u proto=%c"
//    mdm_present (T3, PA4): live MDM level, ASSUMPTION -- polarity not
//      bench-confirmed (see comment at prev_mdm_present)
//    mdm_count (T3, PA5): free-running uint32_t pulse counter, wrap
//      included -- the host MUST diff with wrap-safe unsigned
//      arithmetic, exactly as for the Roadrunner's `position`
//    proto (T4): LLL_PROTO_VERSION (see define). lll_buffer.py compares its
//      own constant against this value at klippy:connect and REFUSES to
//      start on a mismatch (message naming both versions), rather than
//      continuing to parse an lll_state that changed under its feet.
//  lll_io_cfg   ch=%c func=%c arg=%u -> "lll_io_ack ch=%c func=%c ok=%c"
//  lll_io_write ch=%c val=%u
//  lll_io_read  ch=%c              -> "lll_io ch=%c val=%u ok=%c"
//  lll_unlock   key=%u             0xDEB1: ungates the unverified pins (session)
//  Asynchronous PUSH: "lll_event ev=%c a=%u"
//    ev: 0=ecode_set(a=ecode) 1=error_cleared 2=present_edge(a=0/1)
//        3=diag_edge(a=0/1) 4=button(a: 1=click-retract 2=click-feed
//        3=press-retract 4=press-feed 5=hold-end) 5=arm_change(a=bitfield)
//        6=role(a=role) 7=mdm_present_edge(a=0/1, T3, PA4) -- bit 7 (0x80) of
//        telem_mask, OUTSIDE the 0x0F default (like EV_ARM/bit5) ; NO event
//        for PA5 motion (see mdm_count above) -- an encoder pulse train
//        would flood telemetry
//
//  ENUMS: role   0=active 1=parked 2=standby
//         mode   0=auto 1=semi-auto 2=semi-manual 3=manual
//         ecode  0=none 1=feed_timeout 2=back_timeout 3=diag_fault
//                4=host_alive 5=jam_cleared-pending
//         arm    raw bitfield H3H2H1 (bit2=hall3 bit1=hall2 bit0=hall1)
//         func   0=FEED 1=BUTTONS 2=MBSIG 3=ENTRANCE 4=TMC 5=MOVE 6=DIAG
//                7=LED 8=ADC 9=TELEM 10=IOEXT
//
//  PARAMS (lll_param func key val):
//   FEED(0):  0=feed_timeout_ms [1000..60000] def 60000
//             1=back_timeout_ms [1000..60000] def 60000
//   BUTTONS(1): 0=gate_by_role 0/1 def 0
//   TMC(4):   0=IRUN [0..15]  (clamp = known-good deployed value ; do NOT
//                raise without the motor datasheet -- zero guesswork)
//             1=IHOLD [0..15] ; 2=IHOLDDELAY [0..15] ; 3=MRES [0..8]
//             (0=256µ..8=full ; NB: VACTUAL_MAG does NOT auto-rescale)
//             4=stealth 0/1 (GCONF en_spreadcycle inverted)
//             5=standstill [0..3] (PWMCONF freewheel ; 1=freewheel requires IHOLD=0)
//             6=TPOWERDOWN [0..255] ; 7=vactual_mag [0..8388607] def 77575
//   MOVE(5):  0=run_max_ms [1000..60000] def 30000
//             1=host_alive_ms 0=off else [3000..60000] def 0 (opt-in)
//   DIAG(6):  0=action 0=report-only(def) 1=stop-latch
//             1=polarity 0=active-high(def) 1=active-low   (bench it!)
//   TELEM(9): 0=event_mask (bit per ev) def 0x0F (ecode/error_cleared/
//             present/diag) -- EV_ARM (bit5=0x20) and EV_MDM (bit7=0x80,
//             T3) are OUTSIDE the default, enable explicitly if needed
//             (e.g. 0x8F for MDM+default, 0x2F for ARM+default)
//
//  IOEXT (index-only, ch -> compiled pin ; OUT OF TABLE = UNREACHABLE):
//   ch0=HOLE(T3, ex-PA4/IO1) ch1=HOLE(T3, ex-PA5/IO2) -- PA4/PA5 are
//     now the MDM presence/motion module (see MDM_*_PIN) ; lll_io_cfg
//     unconditionally refuses ch0/ch1, indices kept so as not to
//     shift ch2..ch11 (host contract, see runbook §0)
//   ch2=PB10(IO4) ch3=PB11(IO3)
//   ch4=HOLE(T2, ex-PA2/EXT1) ch5=HOLE(T2, ex-PA3/EXT2) -- PA2/PA3 are
//     now the external retract/feed buttons (see KEY_EXT_*_PIN) ;
//     lll_io_cfg unconditionally refuses ch4/ch5, indices kept
//     so as not to shift ch6..ch11 (host contract, see runbook §0)
//   ch6=PB5(*MBSIG off) ch7=PB6(*MBSIG off) ch8=PA8(LED1,unv) ch9=PA15(LED2,unv)
//   ch10=PA0(ADC-VIN,adc,unv) ch11=PA1(ADC-NTC,adc,unv)
//   io_cfg funcs: 0=release 1=din(arg=pull:0/1/2) 2=dout(arg: bit0=init,
//     bit1=shutdown_val, bits8..23=max_on s, 0=no watchdog) 3=pwm-soft
//     (arg=period ms [10..10000], duty via io_write 0..255) 4=adc
//   unv = unverified -> requires lll_unlock ; conflicts: ch6/7 MBSIG off ;
//   ch8/9 LED off ; ch10/11 ADC off
//
// Standalone conventions carried over WORD FOR WORD (buffer.cpp / buffer.h):
//   halls: obstructed = 1 ; PB7: filament present = 0 ; buttons: pressed = 0
//   HALL3 (PB4) obstructed -> Forward ; HALL2 (PB3) -> Stop ; HALL1 (PB2) -> Back
//   FORWARD = GCONF shaft=1 ; VACTUAL always POSITIVE ; stop before reversing
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"      // CONFIG_CLOCK_FREQ
#include "board/gpio.h"    // gpio_in_setup / gpio_out_setup / gpio_adc_*
#include "board/misc.h"    // timer_read_time
#include "sched.h"         // DECL_TASK / DECL_INIT / DECL_SHUTDOWN / sched_is_shutdown
#include "command.h"       // DECL_COMMAND / sendf

// ------------------------------------------------------------------
// Pins (Klipper encoding: GPIO(PORT,NUM) = (PORT-'A')*16 + NUM)  [buffer.h]
// ------------------------------------------------------------------
#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))

#define HALL1_PIN     GPIO('B', 2)   // 光感3 -> pos3 -> Back
#define HALL2_PIN     GPIO('B', 3)   // 光感2 -> pos2 -> Stop
#define HALL3_PIN     GPIO('B', 4)   // 光感1 -> pos1 -> Forward
#define FIL_PIN       GPIO('B', 7)   // ENDSTOP_3 / 耗材开关 (filament switch)
#define KEY1_PIN      GPIO('B', 13)  // KEY1 后退 (retract)
#define KEY2_PIN      GPIO('B', 12)  // KEY2 前进 (feed)
#define KEY_EXT_RETRACT_PIN GPIO('A', 2)  // exposed 3-pin header, external
                                          // retract button (T2; ex-EXT1/inform)
#define KEY_EXT_FEED_PIN    GPIO('A', 3)  // exposed 3-pin header, external
                                          // feed button (T2; ex-EXT2/inform)
#define MDM_PRESENCE_PIN    GPIO('A', 4)  // MDM filament presence (T3; ex-IOEXT ch0)
#define MDM_MOTION_PIN      GPIO('A', 5)  // MDM encoder motion (T3; ex-IOEXT ch1)
#define FRONT_SIG_PIN GPIO('B', 5)   // MBSIG "forward" (active low) -- OFF by default
#define BACK_SIG_PIN  GPIO('B', 6)   // MBSIG "backward" (active low) -- OFF by default
#define EN_PIN        GPIO('A', 6)   // TMC enable (active low)
#define UART_PIN      GPIO('B', 1)   // TMC2208/2225 single-wire UART (PDN_UART)
#define DIAG_PIN      GPIO('B', 0)   // DIAG TMC (input) -- polarity = bench-determine
#define LED1_PIN      GPIO('A', 8)   // STATE_LED -- bench
#define LED2_PIN      GPIO('A', 15)  // ERR_LED -- bench

// ------------------------------------------------------------------
// Standalone parameters (NO modification of the deployed values)
// ------------------------------------------------------------------
// VACTUAL = SPEED * microsteps * 200 / 60 / 0.715   (buffer.cpp)
#define SPEED_RPM        260
#define MICROSTEPS_DIV   64
#define VACTUAL_DEFAULT  ((int32_t)((int64_t)SPEED_RPM * MICROSTEPS_DIV * 200 / 60 * 1000 / 715))
#define VACTUAL_MAX      8388607   // HARDWARE bound: VACTUAL = 24 signed bits

#define TICKS_PER_MS     (CONFIG_CLOCK_FREQ / 1000)

// Bit-bang UART: 9600 baud (standalone beginSerial(9600))
#define UART_BAUD        9600
#define BIT_TICKS        (CONFIG_CLOCK_FREQ / UART_BAUD)

// ------------------------------------------------------------------
// TMC2208/2225 registers + base values (identical to deployed)
// ------------------------------------------------------------------
#define TMC_GCONF        0x00
#define TMC_IHOLD_IRUN   0x10
#define TMC_TPOWERDOWN   0x11
#define TMC_VACTUAL      0x22
#define TMC_CHOPCONF     0x6C
#define TMC_PWMCONF      0x70
#define TMC_WRITE_FLAG   0x80
#define TMC_SLAVE_ADDR   0x00
#define TMC_SYNC         0x05

#define GCONF_BASE       0x000001C4UL   // spreadcycle,pdn_disable,mstep_reg,multistep_filt
#define GCONF_SHAFT      0x00000008UL   // bit3 shaft (FORWARD=1)
#define GCONF_SPREAD     0x00000004UL   // bit2 en_spreadcycle
#define CHOPCONF_BASE    0x12020055UL   // toff=5,vsense=1,mres=2(64µ),intpol=1
#define PWMCONF_BASE     0xC10D0024UL   // TMC2208 default (pwm_autoscale)
#define IRUN_DEFAULT     15             // rms_current(500 mA)@0.11 -- known-good
#define IHOLD_DEFAULT    7
#define IHOLDDELAY_DEF   1
#define IRUN_MAX         15             // clamp = deployed value; no higher without
                                        // the motor datasheet (zero guesswork)
#define TPOWERDOWN_DEF   20             // TMC2208 reset default (datasheet)

// ------------------------------------------------------------------
// Enums protocole
// ------------------------------------------------------------------
enum { ST_STOP = 0, ST_FORWARD, ST_BACK };
enum { ROLE_ACTIVE = 0, ROLE_PARKED, ROLE_STANDBY };
enum { FM_AUTO = 0, FM_SEMI_AUTO, FM_SEMI_MAN, FM_MANUAL };
enum { EC_NONE = 0, EC_FEED_TIMEOUT, EC_BACK_TIMEOUT, EC_DIAG_FAULT,
       EC_HOST_ALIVE, EC_CLEAR_PENDING };
enum { EV_ECODE = 0, EV_ERR_CLEAR, EV_PRESENT, EV_DIAG, EV_BUTTON,
       EV_ARM, EV_ROLE, EV_MDM };
enum { FUNC_FEED = 0, FUNC_BUTTONS, FUNC_MBSIG, FUNC_ENTRANCE, FUNC_TMC,
       FUNC_MOVE, FUNC_DIAG, FUNC_LED, FUNC_ADC, FUNC_TELEM, FUNC_IOEXT,
       FUNC_COUNT };
#define FEN(f)           (1u << (f))
#define FEN_DEFAULT      (FEN(FUNC_FEED) | FEN(FUNC_BUTTONS) | FEN(FUNC_ENTRANCE) \
                          | FEN(FUNC_TMC) | FEN(FUNC_MOVE) | FEN(FUNC_TELEM))
#define UNLOCK_KEY       0xDEB1

// Wire protocol version (T4). Exposed as `proto` in lll_state, checked by
// lll_buffer.py at klippy:connect. Bump BOTH this and lll_buffer.py's
// PROTOCOL_VERSION together on ANY change to the meaning of a
// command/event/param wire field -- including one that does NOT change a
// format string's text (e.g. reassigning an ev/func/key id, reordering an
// enum, changing arm's bit layout). Klipper's own MCU dictionary lookup
// already rejects a format-STRING mismatch at identify time; it cannot
// catch a same-shape field whose meaning silently changed underneath it,
// which is exactly what this counter is for.
#define LLL_PROTO_VERSION 1

// ------------------------------------------------------------------
// Global state
// ------------------------------------------------------------------
static struct gpio_in hall1, hall2, hall3, fil, key1, key2, front_sig, back_sig,
                      diag_in, key_ext_retract, key_ext_feed, mdm_presence,
                      mdm_motion;
static struct gpio_out en_out, uart_out, led1, led2;

static uint16_t func_en = FEN_DEFAULT;
static uint8_t role = ROLE_ACTIVE;
static uint8_t feed_mode = FM_AUTO;
static uint8_t armed;                  // semi-auto: one cycle armed
static uint8_t unlocked;               // lll_unlock session
static uint8_t cfgerr;                 // sticky: a config was refused

static uint8_t last_state = ST_STOP;   // APPLIED motor state (single source)
static uint8_t tmc_ready;
static uint8_t is_feeding;
static uint8_t is_error;
static uint8_t ecode = EC_NONE;

// host intents (set by the handlers, applied by the task -- single-writer)
static uint8_t host_jog;               // 0/1/2
static uint32_t jog_start_ms, jog_ms;
static uint8_t host_run;               // 0/1/2
static uint32_t run_start_ms, run_ms;
static uint8_t stop_req, clear_req;

// runtime params
static uint32_t feed_timeout_ms = 60000;
static uint32_t back_timeout_ms = 60000;
static uint32_t run_max_ms = 30000;
static uint32_t host_alive_ms;         // 0 = off (opt-in)
static uint8_t btn_gate_role;          // 0 = buttons active even when parked (parity)
static uint8_t diag_action;            // 0=report-only 1=stop-latch
static uint8_t diag_pol;               // 0=active-high 1=active-low
static uint8_t telem_mask = 0x0F;

// TMC shadows + deferred writes (bit per register)
static uint32_t gconf_shadow = GCONF_BASE;
static uint32_t chopconf_shadow = CHOPCONF_BASE;
static uint32_t pwmconf_shadow = PWMCONF_BASE;
static uint8_t tmc_irun = IRUN_DEFAULT, tmc_ihold = IHOLD_DEFAULT,
               tmc_iholddelay = IHOLDDELAY_DEF, tmc_tpowerdown = TPOWERDOWN_DEF;
static int32_t vactual_mag = VACTUAL_DEFAULT;
enum { TP_GCONF = 1, TP_CHOPCONF = 2, TP_IHOLD = 4, TP_PWMCONF = 8,
       TP_TPD = 16, TP_VACT = 32 };
static uint8_t tmc_pending;
static uint32_t tmc_last_ms;

// ms clock (49 d wrap; the 32-bit tick @48 MHz wraps at 89 s)
static uint32_t g_ms, ms_last_ticks, ms_acc;
#define ELAPSED_MS(since) ((uint32_t)(g_ms - (since)))

// host-alive
static uint32_t last_host_ms;
static uint8_t alive_tripped;

// boot grace period (fix #9): no AUTO feed before 20 s OR the 1st host cmd
static uint8_t host_ever;              // the host has spoken at least once
static uint8_t grace_done;             // latch: grace over, never re-armed
#define BOOT_GRACE_MS 20000

// buttons (non-blocking polling, port of the standalone's attachInterrupt)
static uint8_t key1_prev = 1, key2_prev = 1;
static uint32_t key1_press_ms, key2_press_ms, key1_rel_ms, key2_rel_ms;
static uint8_t key1_held, key2_held, key1_rel, key2_rel, key1_cnt, key2_cnt;
static uint8_t manual;                 // 0=none 1=fwd 2=back

// parked freeze-follow: bounded by back_timeout
static uint8_t parked_back_on;
static uint32_t parked_back_ms;

// telemetry edges
static uint8_t prev_present = 0xFF, prev_arm = 0xFF;

// MDM (T3, PA4/PA5, ex-IOEXT ch0/ch1). Presence pushes EV_MDM on edge, same
// sentinel idiom as prev_present/prev_arm above (0xFF = "no sample yet",
// suppresses a spurious edge event on the very first tick after boot).
// Motion is a free-running pulse counter, NOT an event -- see track_edges().
// No firmware-side stall latch here on purpose: stall policy is host-side
// and tunable (see port runbook T10), unlike the old fork which latched a
// stall in firmware.
static uint8_t prev_mdm_present = 0xFF;
static uint8_t mdm_motion_prev;
static uint32_t mdm_motion_count;   // wraps at UINT32_MAX; host MUST diff
                                    // successive samples with wrap-safe
                                    // unsigned arithmetic, e.g.
                                    // (uint32_t)(new_count - old_count)

// DIAG: TIME-based debounce (level stable >= 8 ms) -- robust regardless of
// the task cadence (100 ms floor at idle)
static uint8_t diag_assert, diag_deb_lvl;
static uint32_t diag_stable_ms;

// LED : blink 2 Hz
#define LED_BLINK_MS 250

// ------------------------------------------------------------------
// IOEXT -- index-only registry. Pins OUT OF TABLE = UNREACHABLE.
// HARD_RESERVED (never here): PA11/12 USB, PA13/14 SWD, PB1 UART, PA6 EN,
// PA7 DIR, PC13 STEP, PB2/3/4 halls, PB7, PB12/13 buttons, PB14/PB15 (MDM
// host), PB8/PB9 (5 V headers), PF0/PF1 HSE.
// ------------------------------------------------------------------
#define IOEXT_NCH 12
enum { IOF_NONE = 0, IOF_DIN, IOF_DOUT, IOF_PWM, IOF_ADC };

struct ioext_def {
    uint8_t pin;
    uint8_t adc_ok;      // STM32F072: ADC IN0-IN7 = PA0-PA7 only here
    uint8_t unverified;  // requires lll_unlock
};
static const struct ioext_def ioext_def[IOEXT_NCH] = {
    // ch0/ch1 HOLE (T3): PA4/PA5 reclaimed as the MDM presence/motion
    // inputs (MDM_PRESENCE_PIN/MDM_MOTION_PIN, buffer_init) and are no
    // longer reachable through IOEXT. The slots are kept -- not removed and
    // not renumbered -- so ch2..ch11 keep their host-facing indices
    // (LLL_IO_CFG CH=n contract, see §0 of the port runbook).
    // command_lll_io_cfg refuses ch==0/1 unconditionally (even with
    // lll_unlock), so ioch[0]/ioch[1].func can never leave IOF_NONE and
    // lll_io_read/lll_io_write fall through to their existing "unclaimed
    // channel" no-op/ok=0 paths. The pin/adc_ok/unverified fields below are
    // now dead data, kept only so this stays a structurally valid row.
    { GPIO('A', 4),  1, 0 },   // ch0  was IO1 -- see HOLE comment above
    { GPIO('A', 5),  1, 0 },   // ch1  was IO2 -- see HOLE comment above
    { GPIO('B', 10), 0, 0 },   // ch2  IO4
    { GPIO('B', 11), 0, 0 },   // ch3  IO3
    // ch4/ch5 HOLE (T2): PA2/PA3 reclaimed as the external retract/feed
    // button inputs (KEY_EXT_RETRACT_PIN/KEY_EXT_FEED_PIN, buffer_init) and
    // are no longer reachable through IOEXT. The slots are kept -- not
    // removed and not renumbered -- so ch6..ch11 keep their host-facing
    // indices (LLL_IO_CFG CH=n contract, see §0 of the port runbook).
    // command_lll_io_cfg refuses ch==4/5 unconditionally (even with
    // lll_unlock), so ioch[4]/ioch[5].func can never leave IOF_NONE and
    // lll_io_read/lll_io_write fall through to their existing "unclaimed
    // channel" no-op/ok=0 paths. The pin/adc_ok/unverified fields below are
    // now dead data, kept only so this stays a structurally valid row.
    { GPIO('A', 2),  1, 1 },   // ch4  was EXT1 -- see HOLE comment above
    { GPIO('A', 3),  1, 1 },   // ch5  was EXT2 -- see HOLE comment above
    { GPIO('B', 5),  0, 0 },   // ch6  MBSIG feed  (claim iff MBSIG off)
    { GPIO('B', 6),  0, 0 },   // ch7  MBSIG retract
    { GPIO('A', 8),  0, 1 },   // ch8  LED1 (claim iff LED off)
    { GPIO('A', 15), 0, 1 },   // ch9  LED2
    { GPIO('A', 0),  1, 1 },   // ch10 ADC VIN (claim iff ADC off)
    { GPIO('A', 1),  1, 1 },   // ch11 ADC NTC
};

struct ioext_ch {
    uint8_t func;              // IOF_*
    uint8_t module_owned;      // internal claim by the ADC module (func 8)
    struct gpio_in in;
    struct gpio_out out;
    struct gpio_adc adc;
    uint8_t out_level, shutdown_level;
    uint32_t max_on_ms;        // dout watchdog (0 = off)
    uint32_t on_since_ms;
    uint32_t pwm_period_ms, pwm_phase_ms;
    uint8_t pwm_duty;          // 0..255
    uint16_t adc_val;          // last raw 12-bit sample
};
static struct ioext_ch ioch[IOEXT_NCH];
static uint8_t adc_rr;         // round-robin sampling
static uint8_t adc_inflight = 0xFF;

// ------------------------------------------------------------------
// UART TMC bit-bang TX (identical to deployed -- TX-only, documented mode)
// ------------------------------------------------------------------
static void
bit_delay(void)
{
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < BIT_TICKS)
        ;
}

static void
uart_send_byte(uint8_t b)
{
    gpio_out_write(uart_out, 0);
    bit_delay();
    for (uint8_t i = 0; i < 8; i++) {
        gpio_out_write(uart_out, (b >> i) & 0x01);
        bit_delay();
    }
    gpio_out_write(uart_out, 1);
    bit_delay();
}

static uint8_t
tmc_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t cur = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (cur & 0x01))
                crc = (crc << 1) ^ 0x07;
            else
                crc = crc << 1;
            cur >>= 1;
        }
    }
    return crc;
}

static void
tmc_write(uint8_t reg, uint32_t val)
{
    uint8_t dg[8];
    dg[0] = TMC_SYNC;
    dg[1] = TMC_SLAVE_ADDR;
    dg[2] = reg | TMC_WRITE_FLAG;
    dg[3] = (val >> 24) & 0xFF;
    dg[4] = (val >> 16) & 0xFF;
    dg[5] = (val >> 8) & 0xFF;
    dg[6] = val & 0xFF;
    dg[7] = tmc_crc(dg, 7);
    for (uint8_t i = 0; i < 8; i++)
        uart_send_byte(dg[i]);
    tmc_last_ms = g_ms;
}

static void
tmc_delay_ms(uint32_t ms)
{
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < ms * TICKS_PER_MS)
        ;
}

// values composed from the shadows
static uint32_t
tmc_ihold_irun_val(void)
{
    return (uint32_t)tmc_ihold | ((uint32_t)tmc_irun << 8)
           | ((uint32_t)tmc_iholddelay << 16);
}

static uint32_t
tmc_gconf_val(uint8_t state)
{
    return gconf_shadow | (state == ST_FORWARD ? GCONF_SHAFT : 0);
}

static void
tmc_configure(void)
{
    tmc_delay_ms(10);
    tmc_write(TMC_GCONF, gconf_shadow);          tmc_delay_ms(2);
    tmc_write(TMC_CHOPCONF, chopconf_shadow);    tmc_delay_ms(2);
    tmc_write(TMC_IHOLD_IRUN, tmc_ihold_irun_val()); tmc_delay_ms(2);
    tmc_write(TMC_PWMCONF, pwmconf_shadow);      tmc_delay_ms(2);
    tmc_write(TMC_TPOWERDOWN, tmc_tpowerdown);   tmc_delay_ms(2);
    tmc_write(TMC_VACTUAL, 0);                   tmc_delay_ms(2);
    last_state = ST_STOP;
    tmc_pending = 0;
}

// applies ONE deferred TMC write per tick, spaced >= 2 ms (fix #8)
static void
tmc_apply_pending(void)
{
    if (!tmc_pending || ELAPSED_MS(tmc_last_ms) < 2)
        return;
    if (tmc_pending & TP_GCONF) {
        tmc_write(TMC_GCONF, tmc_gconf_val(last_state));
        tmc_pending &= ~TP_GCONF;
    } else if (tmc_pending & TP_CHOPCONF) {
        tmc_write(TMC_CHOPCONF, chopconf_shadow);
        tmc_pending &= ~TP_CHOPCONF;
    } else if (tmc_pending & TP_IHOLD) {
        tmc_write(TMC_IHOLD_IRUN, tmc_ihold_irun_val());
        tmc_pending &= ~TP_IHOLD;
    } else if (tmc_pending & TP_PWMCONF) {
        tmc_write(TMC_PWMCONF, pwmconf_shadow);
        tmc_pending &= ~TP_PWMCONF;
    } else if (tmc_pending & TP_TPD) {
        tmc_write(TMC_TPOWERDOWN, tmc_tpowerdown);
        tmc_pending &= ~TP_TPD;
    } else if (tmc_pending & TP_VACT) {
        if (last_state != ST_STOP)      // refresh speed while moving
            tmc_write(TMC_VACTUAL, (uint32_t)vactual_mag);
        tmc_pending &= ~TP_VACT;
    }
}

// ------------------------------------------------------------------
// apply_state -- the ONLY writer of the motor, along with tmc_apply_pending/
// configure (all called exclusively from buffer_task). Standalone
// convention: stop before reversing, GCONF+VACTUAL rewritten on every move,
// VACTUAL=0 BEFORE EN=1 on STOP (fix #6).
// ------------------------------------------------------------------
static void
apply_state(uint8_t state)
{
    if (state == last_state)
        return;
    switch (state) {
    case ST_FORWARD:
        gpio_out_write(en_out, 0);
        if (last_state == ST_BACK) { tmc_write(TMC_VACTUAL, 0); tmc_delay_ms(2); }
        tmc_write(TMC_GCONF, tmc_gconf_val(ST_FORWARD));
        tmc_delay_ms(2);
        tmc_write(TMC_VACTUAL, (uint32_t)vactual_mag);
        break;
    case ST_BACK:
        gpio_out_write(en_out, 0);
        if (last_state == ST_FORWARD) { tmc_write(TMC_VACTUAL, 0); tmc_delay_ms(2); }
        tmc_write(TMC_GCONF, tmc_gconf_val(ST_BACK));
        tmc_delay_ms(2);
        tmc_write(TMC_VACTUAL, (uint32_t)vactual_mag);
        break;
    case ST_STOP:
    default:
        tmc_write(TMC_VACTUAL, 0);
        gpio_out_write(en_out, 1);
        break;
    }
    last_state = state;
}

// ------------------------------------------------------------------
// Events (TELEM)
// ------------------------------------------------------------------
static void
ev_send(uint8_t ev, uint32_t a)
{
    if (!(func_en & FEN(FUNC_TELEM)) || !(telem_mask & (1u << ev)))
        return;
    sendf("lll_event ev=%c a=%u", ev, a);
}

static void
set_ecode(uint8_t code, uint8_t latch_error)
{
    ecode = code;
    if (latch_error)
        is_error = 1;
    ev_send(EV_ECODE, code);
}

// ------------------------------------------------------------------
// Init (at boot) -- core only. The pins of the OFF-by-default modules
// (MBSIG PB5/PB6, DIAG PB0, LED PA8/PA15, ADC PA0/PA1) are NOT claimed
// here: lazy claim at enable (full-USB, unverified pins left untouched).
// ------------------------------------------------------------------
void
buffer_init(void)
{
    hall1 = gpio_in_setup(HALL1_PIN, 0);
    hall2 = gpio_in_setup(HALL2_PIN, 0);
    hall3 = gpio_in_setup(HALL3_PIN, 0);
    fil   = gpio_in_setup(FIL_PIN, 0);
    key1  = gpio_in_setup(KEY1_PIN, 0);
    key2  = gpio_in_setup(KEY2_PIN, 0);
    // Pull differs from key1/key2 on purpose: KEY1/KEY2 sit on dedicated
    // button pins that the board pulls up in hardware (bench-confirmed by
    // the deployed convention above: pull=0 there still reads 1=released).
    // PA2/PA3 are the bare exposed header with no such onboard pull, so the
    // MCU's internal pull-up is enabled here (pull=1) to get the same
    // idle-high/pressed-low behaviour with a plain switch-to-ground button.
    // Matches the old fork (buffer_manager_old/klippy_src/buffer.c:399-400,
    // key_external_retract/feed also set up with pull=1). Needs bench
    // confirmation once external buttons are wired — see report.
    key_ext_retract = gpio_in_setup(KEY_EXT_RETRACT_PIN, 1);
    key_ext_feed    = gpio_in_setup(KEY_EXT_FEED_PIN, 1);
    // MDM presence/motion (T3): pull=0 (no internal pull), matching the old
    // fork (buffer_manager_old/klippy_src/buffer.c:401-402) and NOT the
    // pull=1 chosen above for PA2/PA3. Reasoning: PA2/PA3 are a bare header
    // with a plain switch-to-ground and no onboard pull, but the Mellow MDM
    // is a powered sensor module with its own active output driving the
    // line, so an internal pull would fight (or be redundant with) that
    // drive. Unverified -- needs bench confirmation once MDM is wired, same
    // as the old fork flagged its own presence-polarity assumption unverified.
    mdm_presence = gpio_in_setup(MDM_PRESENCE_PIN, 0);
    mdm_motion   = gpio_in_setup(MDM_MOTION_PIN, 0);
    mdm_motion_prev = gpio_in_read(mdm_motion);  // avoid a false edge on the
                                                 // first tick (old fork parity)
    en_out   = gpio_out_setup(EN_PIN, 1);        // motor disabled (active low)
    uart_out = gpio_out_setup(UART_PIN, 1);      // UART idle = high
}
DECL_INIT(buffer_init);

// ------------------------------------------------------------------
// IOEXT helpers
// ------------------------------------------------------------------
static void
ioext_release(uint8_t ch)
{
    if (ioch[ch].func == IOF_NONE)
        return;
    if (ioch[ch].func == IOF_ADC) {
        // conversion in flight: consume/cancel it, otherwise the never-read
        // EOC freezes ALL other ADC channels (bug caught in review)
        gpio_adc_cancel_sample(ioch[ch].adc);
        if (adc_inflight == ch)
            adc_inflight = 0xFF;
    }
    ioch[ch].func = IOF_NONE;
    ioch[ch].module_owned = 0;
    ioch[ch].in = gpio_in_setup(ioext_def[ch].pin, 0);   // Hi-Z
}

static void
ioext_release_all(void)
{
    for (uint8_t i = 0; i < IOEXT_NCH; i++)
        if (!ioch[i].module_owned)       // the ADC module's channels stay
            ioext_release(i);            // with it (FUNC_ADC always enabled)
}

// IOEXT soft engine: soft PWM + max_on watchdog on douts.
// NB: soft-PWM granularity = task cadence (100 ms floor at
// idle) -- suited to LED/relay/slow-fan, NOT precise servo/signal.
static void
ioext_engine(void)
{
    for (uint8_t i = 0; i < IOEXT_NCH; i++) {
        struct ioext_ch *c = &ioch[i];
        if (c->func == IOF_PWM) {
            uint32_t pos = ELAPSED_MS(c->pwm_phase_ms);
            if (pos >= c->pwm_period_ms) {
                c->pwm_phase_ms = g_ms;
                pos = 0;
            }
            uint32_t on_ms = c->pwm_period_ms * c->pwm_duty / 256;
            gpio_out_write(c->out, pos < on_ms);
        } else if (c->func == IOF_DOUT && c->max_on_ms
                   && c->out_level != c->shutdown_level
                   && ELAPSED_MS(c->on_since_ms) > c->max_on_ms) {
            c->out_level = c->shutdown_level;    // watchdog : retombe
            gpio_out_write(c->out, c->out_level);
        }
    }
}

// ADC round-robin sampling, non-blocking (one channel in flight max)
static void
adc_engine(void)
{
    if (adc_inflight != 0xFF) {
        struct ioext_ch *c = &ioch[adc_inflight];
        if (c->func != IOF_ADC) {                // released in the meantime
            adc_inflight = 0xFF;
            return;
        }
        if (!gpio_adc_sample(c->adc)) {
            c->adc_val = gpio_adc_read(c->adc);
            adc_inflight = 0xFF;
        }
        return;
    }
    for (uint8_t n = 0; n < IOEXT_NCH; n++) {
        uint8_t i = (uint8_t)((adc_rr + 1 + n) % IOEXT_NCH);
        if (ioch[i].func == IOF_ADC) {
            adc_rr = i;
            adc_inflight = i;
            gpio_adc_sample(ioch[i].adc);        // starts the conversion
            return;
        }
    }
}

// internal claim by the ADC module (func 8) on ch10/ch11
static void
adc_module_claim(void)
{
    for (uint8_t i = 10; i <= 11; i++) {
        ioch[i].adc = gpio_adc_setup(ioext_def[i].pin);
        ioch[i].func = IOF_ADC;
        ioch[i].module_owned = 1;
        ioch[i].adc_val = 0;
    }
}

static void
adc_module_release(void)
{
    for (uint8_t i = 10; i <= 11; i++)
        if (ioch[i].module_owned)
            ioext_release(i);
}

// ------------------------------------------------------------------
// DIAG (PB0): time-based debounce (stable >= 8 ms), configurable polarity
// ------------------------------------------------------------------
static void
diag_engine(void)
{
    if (!(func_en & FEN(FUNC_DIAG)))
        return;
    uint8_t lvl = gpio_in_read(diag_in);
    uint8_t asserted = diag_pol ? !lvl : lvl;
    if (asserted != diag_deb_lvl) {
        diag_deb_lvl = asserted;         // changed: restart the window
        diag_stable_ms = g_ms;
        return;
    }
    if (ELAPSED_MS(diag_stable_ms) >= 8 && diag_assert != diag_deb_lvl) {
        diag_assert = diag_deb_lvl;
        ev_send(EV_DIAG, diag_assert);
        if (diag_assert && diag_action == 1)
            set_ecode(EC_DIAG_FAULT, 1);
        // completion of a pending clear (ecode 5) happens in the task
    }
}

// ------------------------------------------------------------------
// LED: PA8 state (present=steady, moving=blink, empty=off) ; PA15 error
// (steady ; blink if clear-pending). Lazy claim at enable.
// ------------------------------------------------------------------
static void
led_engine(void)
{
    if (!(func_en & FEN(FUNC_LED)))
        return;
    uint8_t blink = (g_ms / LED_BLINK_MS) & 1;
    uint8_t present = !gpio_in_read(fil);
    if (last_state != ST_STOP)
        gpio_out_write(led1, blink);
    else
        gpio_out_write(led1, present);
    if (is_error)
        gpio_out_write(led2, ecode == EC_CLEAR_PENDING ? blink : 1);
    else
        gpio_out_write(led2, 0);
}

// ------------------------------------------------------------------
// Buttons + MBSIG -- non-blocking port of motor_control (standalone parity).
// Returns the intent: 0=none, 1=fwd held, 2=back held, 3=hold-end.
// NEVER writes the motor (single-writer: the task applies it).
// Separate gating: keypad clicks/holds = FUNC_BUTTONS ; PB5/PB6 = FUNC_MBSIG.
// NB cadence: Klipper tasks run on wakeup (100 ms floor at
// idle, much faster while comms are active) -- same conditions as
// deployed, clicks/holds >= 100 ms reliable.
// ------------------------------------------------------------------
static uint8_t mb_only_hold;    // current hold driven by MBSIG alone (bounded)
static uint8_t mb_tripped;      // mb-only bound tripped: MBSIG signals
                                // ignored until BOTH deassert
static uint32_t hold_start_ms;

static uint8_t
process_buttons(void)
{
    uint8_t buttons_on = !!(func_en & FEN(FUNC_BUTTONS));
    // External PA2/PA3 header buttons feed the SAME debounce/click/hold state
    // machine as the internal PB12/PB13 keys (T2) -- combined right here, at
    // the sample point, so every downstream consumer (click, double-click,
    // hold, EV_BUTTON) is untouched and cannot tell which pin produced the
    // press. Both signals are active-low (1=released, 0=pressed), so "either
    // button pressed" is the AND of the raw levels, not their OR:
    //   pressed_combined = pressed_int || pressed_ext
    //                     = !raw_int || !raw_ext
    //                     = !(raw_int && raw_ext)   (De Morgan)
    //   => raw_combined  = raw_int & raw_ext
    // A naive OR of the raw active-low bits would instead compute
    // pressed_int && pressed_ext (both buttons must be down at once), which
    // is wrong.
    uint8_t k1 = gpio_in_read(key1) & gpio_in_read(key_ext_retract);
    uint8_t k2 = gpio_in_read(key2) & gpio_in_read(key_ext_feed);

    if (buttons_on) {
        if (key1_prev && !k1) { key1_press_ms = g_ms; key1_cnt++; key1_held = 1;
                                ev_send(EV_BUTTON, 3); }
        else if (!key1_prev && k1) {
            if (ELAPSED_MS(key1_press_ms) <= 500) { key1_rel = 1;
                                                    key1_rel_ms = g_ms; }
            else key1_cnt = 0;
            key1_held = 0;
        }
        key1_prev = k1;
        if (key2_prev && !k2) { key2_press_ms = g_ms; key2_cnt++; key2_held = 1;
                                ev_send(EV_BUTTON, 4); }
        else if (!key2_prev && k2) {
            if (ELAPSED_MS(key2_press_ms) <= 500) { key2_rel = 1;
                                                    key2_rel_ms = g_ms; }
            else key2_cnt = 0;
            key2_held = 0;
        }
        key2_prev = k2;

        // short click: clear request (via the task's DIAG gate -- a click
        // no longer bypasses a still-asserted DIAG stop-latch)
        if (key1_rel && ELAPSED_MS(key1_rel_ms) > 500) {
            key1_rel = 0;
            if (key1_cnt == 1) {
                if (!clear_req) clear_req = 2;   // 2 = button source
                ev_send(EV_BUTTON, 1);
            }
            key1_cnt = 0;
        }
        if (key2_rel && ELAPSED_MS(key2_rel_ms) > 500) {
            key2_rel = 0;
            if (key2_cnt == 1) {
                if (!clear_req) clear_req = 2;   // 2 = button source
                ev_send(EV_BUTTON, 2);
            }
            key2_cnt = 0;
        }
    } else {
        key1_prev = k1; key2_prev = k2;          // resync the edges
        key1_held = key2_held = key1_rel = key2_rel = 0;
        key1_cnt = key2_cnt = 0;
    }

    // long hold (>=500 ms) OR mainboard signal low (if MBSIG enabled)
    uint8_t mb_back = 0, mb_fwd = 0;
    if (func_en & FEN(FUNC_MBSIG)) {
        mb_back = !gpio_in_read(back_sig);
        mb_fwd  = !gpio_in_read(front_sig);
        if (mb_tripped) {
            // bound tripped: MBSIG is ignored until BOTH signals go back
            // inactive (otherwise a stuck signal would restart a fresh
            // window on every trip)
            if (!mb_back && !mb_fwd)
                mb_tripped = 0;
            else
                mb_back = mb_fwd = 0;
        }
    }
    uint8_t key_back = buttons_on && key1_held
                       && ELAPSED_MS(key1_press_ms) >= 500;
    uint8_t key_fwd  = buttons_on && key2_held
                       && ELAPSED_MS(key2_press_ms) >= 500;

    if (key_back || mb_back) {                   // BACK priority (standalone)
        uint8_t mo = !key_back;
        if (manual != 2) { manual = 2; hold_start_ms = g_ms; }
        else if (mo && !mb_only_hold)            // key->mb: fresh window
            hold_start_ms = g_ms;
        mb_only_hold = mo;
        return 2;
    }
    if (key_fwd || mb_fwd) {
        uint8_t mo = !key_fwd;
        if (manual != 1) { manual = 1; hold_start_ms = g_ms; }
        else if (mo && !mb_only_hold)            // key->mb: fresh window
            hold_start_ms = g_ms;
        mb_only_hold = mo;
        return 1;
    }
    if (manual)   { manual = 0; ev_send(EV_BUTTON, 5); return 3; }
    return 0;
}

// ------------------------------------------------------------------
// Command handlers -- FLAGS ONLY (fix #2). Every command refreshes
// the host-alive keepalive.
// ------------------------------------------------------------------
static void
ack(uint8_t func, uint8_t key, uint32_t val, uint8_t ok)
{
    if (!ok)
        cfgerr = 1;
    sendf("lll_ack func=%c key=%c val=%u ok=%c", func, key, val, ok);
}

static void
host_seen(void)
{
    last_host_ms = g_ms;
    alive_tripped = 0;
    host_ever = 1;                     // ends the boot grace period (fix #9)
}

void
command_lll_role(uint32_t *args)
{
    host_seen();
    uint8_t r = args[0];
    if (r > ROLE_STANDBY)
        return;
    if (r != role) {
        role = r;
        parked_back_on = 0;
        ev_send(EV_ROLE, role);
    }
}
DECL_COMMAND(command_lll_role, "lll_role role=%c");

// back-compat ALIAS: lll_set_idle 1 -> parked, 0 -> active
void
command_lll_set_idle(uint32_t *args)
{
    host_seen();
    uint8_t r = args[0] ? ROLE_PARKED : ROLE_ACTIVE;
    if (r != role) {
        role = r;
        parked_back_on = 0;
        ev_send(EV_ROLE, role);
    }
}
DECL_COMMAND(command_lll_set_idle, "lll_set_idle value=%c");

void
command_lll_enable(uint32_t *args)
{
    host_seen();
    uint8_t func = args[0], en = !!args[1];
    if (func >= FUNC_COUNT) {
        ack(func, 0xFE, en, 0);
        return;
    }
    // IOEXT conflicts: refuse the enable if the module's pins are claimed
    if (en) {
        if ((func == FUNC_MBSIG && (ioch[6].func || ioch[7].func))
            || (func == FUNC_LED && (ioch[8].func || ioch[9].func))
            || (func == FUNC_ADC && (ioch[10].func || ioch[11].func))) {
            ack(func, 0xFE, en, 0);
            return;
        }
    }
    uint8_t was = !!(func_en & FEN(func));
    if (was == en) {
        ack(func, 0xFE, en, 1);
        return;
    }
    if (en) {
        // lazy claim of the module's pins (GPIO config -- not motor)
        if (func == FUNC_MBSIG) {
            front_sig = gpio_in_setup(FRONT_SIG_PIN, 1);   // pullup, actif bas
            back_sig  = gpio_in_setup(BACK_SIG_PIN, 1);
            mb_tripped = 0;
        } else if (func == FUNC_DIAG) {
            diag_in = gpio_in_setup(DIAG_PIN, 0);
            diag_assert = 0; diag_deb_lvl = 0; diag_stable_ms = g_ms;
        } else if (func == FUNC_LED) {
            led1 = gpio_out_setup(LED1_PIN, 0);
            led2 = gpio_out_setup(LED2_PIN, 0);
        } else if (func == FUNC_ADC) {
            adc_module_claim();
        }
        func_en |= FEN(func);
    } else {
        func_en &= ~FEN(func);
        if (func == FUNC_MBSIG) {
            gpio_in_setup(FRONT_SIG_PIN, 0);               // release into Hi-Z
            gpio_in_setup(BACK_SIG_PIN, 0);
            if (manual) {                // MBSIG hold in flight -> explicit STOP
                manual = 0;              // (otherwise FSM state=last_state
                stop_req = 1;            //  inheritance = unbounded runaway BACK)
            }
        } else if (func == FUNC_DIAG) {
            diag_assert = 0;
            gpio_in_setup(DIAG_PIN, 0);
        } else if (func == FUNC_LED) {
            gpio_out_write(led1, 0); gpio_out_write(led2, 0);
            gpio_in_setup(LED1_PIN, 0); gpio_in_setup(LED2_PIN, 0);
        } else if (func == FUNC_ADC) {
            adc_module_release();
        } else if (func == FUNC_IOEXT) {
            ioext_release_all();                           // safety
        } else if (func == FUNC_BUTTONS || func == FUNC_MBSIG) {
            if (manual) {                // a hold in flight -> explicit STOP
                manual = 0;
                stop_req = 1;            // otherwise the hold FSM inherits the motion
            }
        } else if (func == FUNC_MOVE) {
            if (host_jog || host_run) {  // kill the jog/run in flight
                host_jog = 0;
                host_run = 0;
                stop_req = 1;
            }
        } else if (func == FUNC_FEED) {
            is_feeding = 0;
        }
        // motor: the task applies the resulting state on the next tick
    }
    ack(func, 0xFE, en, 1);
}
DECL_COMMAND(command_lll_enable, "lll_enable func=%c en=%c");

void
command_lll_mode(uint32_t *args)
{
    host_seen();
    uint8_t func = args[0], level = args[1];
    if (func != FUNC_FEED || level > FM_MANUAL) {
        ack(func, 0xFD, level, 0);
        return;
    }
    feed_mode = level;
    armed = 0;
    ack(func, 0xFD, level, 1);
}
DECL_COMMAND(command_lll_mode, "lll_mode func=%c level=%c");

static uint32_t
clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void
command_lll_param(uint32_t *args)
{
    host_seen();
    uint8_t func = args[0], key = args[1];
    uint32_t val = args[2];
    uint32_t applied = val;              // the ACK returns the APPLIED value
    uint8_t ok = 1;                      // (clamped), not the requested value
    switch (func) {
    case FUNC_FEED:
        if (key == 0)      applied = feed_timeout_ms = clamp_u32(val, 1000, 60000);
        else if (key == 1) applied = back_timeout_ms = clamp_u32(val, 1000, 60000);
        else ok = 0;
        break;
    case FUNC_BUTTONS:
        if (key == 0)      applied = btn_gate_role = !!val;
        else ok = 0;
        break;
    case FUNC_TMC:
        if (!(func_en & FEN(FUNC_TMC))) { ok = 0; break; }
        switch (key) {
        case 0: applied = tmc_irun = clamp_u32(val, 0, IRUN_MAX);
                tmc_pending |= TP_IHOLD; break;
        case 1: applied = tmc_ihold = clamp_u32(val, 0, IRUN_MAX);
                tmc_pending |= TP_IHOLD; break;
        case 2: applied = tmc_iholddelay = clamp_u32(val, 0, 15);
                tmc_pending |= TP_IHOLD; break;
        case 3: if (val > 8) { ok = 0; break; }
                chopconf_shadow = (chopconf_shadow & ~(0xFUL << 24))
                                  | ((uint32_t)val << 24);
                tmc_pending |= TP_CHOPCONF; break;
        case 4: applied = !!val;
                gconf_shadow = val ? (gconf_shadow & ~GCONF_SPREAD)
                                   : (gconf_shadow | GCONF_SPREAD);
                tmc_pending |= TP_GCONF; break;
        case 5: if (val > 3) { ok = 0; break; }
                pwmconf_shadow = (pwmconf_shadow & ~(0x3UL << 20))
                                 | ((uint32_t)val << 20);
                tmc_pending |= TP_PWMCONF; break;
        case 6: applied = tmc_tpowerdown = clamp_u32(val, 0, 255);
                tmc_pending |= TP_TPD; break;
        case 7: applied = vactual_mag = clamp_u32(val, 0, VACTUAL_MAX);
                tmc_pending |= TP_VACT; break;
        default: ok = 0;
        }
        break;
    case FUNC_MOVE:
        if (key == 0)      applied = run_max_ms = clamp_u32(val, 1000, 60000);
        else if (key == 1) applied = host_alive_ms
                               = val ? clamp_u32(val, 3000, 60000) : 0;
        else ok = 0;
        break;
    case FUNC_DIAG:
        if (key == 0)      applied = diag_action = !!val;
        else if (key == 1) applied = diag_pol = !!val;
        else ok = 0;
        break;
    case FUNC_TELEM:
        // 0xFF, not 0x7F: the clamp encodes the number of existing events.
        // EV_MDM=7 (bit 0x80) was added -> a 0x7F clamp would make it
        // impossible to activate (dead event, indistinguishable from a
        // wiring fault on the bench). Widen again if an EV_ id >7 appears.
        if (key == 0)      applied = telem_mask = val & 0xFF;
        else ok = 0;
        break;
    default:
        ok = 0;
    }
    ack(func, key, ok ? applied : val, ok);
}
DECL_COMMAND(command_lll_param, "lll_param func=%c key=%c val=%u");

void
command_lll_arm(uint32_t *args)
{
    (void)args;
    host_seen();
    if (feed_mode == FM_SEMI_AUTO) {
        armed = 1;
        ack(FUNC_FEED, 0xFB, 1, 1);
    } else {
        ack(FUNC_FEED, 0xFB, 0, 0);
    }
}
DECL_COMMAND(command_lll_arm, "lll_arm");

void
command_lll_jog(uint32_t *args)
{
    host_seen();
    uint8_t dir = args[0];
    if (!dir || dir > 2) {
        // stop ALWAYS accepted (even with MOVE off) ; stop_req forces the
        // STOP: without it, the hold FSM's "no hall = keep the state" would
        // inherit the host's motion (bug caught in review -- regression vs deployed)
        host_jog = 0;
        stop_req = 1;
        return;
    }
    if (!(func_en & FEN(FUNC_MOVE))) {
        cfgerr = 1;
        return;
    }
    host_jog = dir;
    jog_ms = clamp_u32(args[1], 1, 10000);
    jog_start_ms = g_ms;
}
DECL_COMMAND(command_lll_jog, "lll_jog dir=%c ms=%u");

void
command_lll_run(uint32_t *args)
{
    host_seen();
    uint8_t dir = args[0];
    if (!dir || dir > 2) {
        host_run = 0;
        host_jog = 0;                    // deployed parity: run 0 cuts the jog
        is_feeding = 0;
        stop_req = 1;                    // same as jog: explicit STOP via the task
        return;
    }
    if (!(func_en & FEN(FUNC_MOVE))) {
        cfgerr = 1;
        return;
    }
    host_run = dir;
    run_ms = args[1] ? clamp_u32(args[1], 1, run_max_ms) : run_max_ms;
    run_start_ms = g_ms;                 // refreshable: re-send = re-window
}
DECL_COMMAND(command_lll_run, "lll_run dir=%c ms=%u");

void
command_lll_stop(uint32_t *args)
{
    (void)args;
    host_seen();
    stop_req = 1;                        // the task applies it (single-writer)
    host_jog = 0;
    host_run = 0;
    armed = 0;
}
DECL_COMMAND(command_lll_stop, "lll_stop");

void
command_lll_clear_error(uint32_t *args)
{
    (void)args;
    host_seen();
    clear_req = 1;
}
DECL_COMMAND(command_lll_clear_error, "lll_clear_error");

void
command_lll_query(uint32_t *args)
{
    (void)args;
    host_seen();
    uint8_t arm = (gpio_in_read(hall3) << 2) | (gpio_in_read(hall2) << 1)
                  | gpio_in_read(hall1);
    sendf("lll_state role=%c mode=%c error=%c ecode=%c feeding=%c running=%c"
          " arm=%c present=%c diag=%c cfgerr=%c mdm_present=%c mdm_count=%u"
          " proto=%c",
          role, feed_mode, is_error, ecode, is_feeding,
          last_state != ST_STOP, arm, !gpio_in_read(fil),
          diag_assert, cfgerr, !gpio_in_read(mdm_presence), mdm_motion_count,
          LLL_PROTO_VERSION);
}
DECL_COMMAND(command_lll_query, "lll_query");

void
command_lll_io_cfg(uint32_t *args)
{
    host_seen();
    uint8_t ch = args[0], func = args[1];
    uint32_t arg = args[2];
    if (ch >= IOEXT_NCH || func > IOF_ADC || !(func_en & FEN(FUNC_IOEXT))) {
        cfgerr = 1;
        sendf("lll_io_ack ch=%c func=%c ok=%c", ch, func, 0);
        return;
    }
    struct ioext_ch *c = &ioch[ch];
    uint8_t ok = 1;
    if (c->module_owned) {
        ok = 0;             // channel owned by the ADC module: go through
                            // lll_enable func=8 en=0, not io_cfg
    } else if (ch == 0 || ch == 1 || ch == 4 || ch == 5) {
        ok = 0;             // HOLE (T2/T3): ch0/1 = PA4/PA5 (MDM presence/
                            // motion), ch4/5 = PA2/PA3 (external buttons),
                            // both hardcoded -- unconditional refusal even
                            // with lll_unlock, and even for func=0 (release)
                            // since they are never claimed by IOEXT
    } else if (func == IOF_NONE) {
        ioext_release(ch);
    } else if (ioext_def[ch].unverified && !unlocked) {
        ok = 0;                          // unverified pin: requires lll_unlock
    } else if ((ch == 6 || ch == 7) && (func_en & FEN(FUNC_MBSIG))) {
        ok = 0;
    } else if ((ch == 8 || ch == 9) && (func_en & FEN(FUNC_LED))) {
        ok = 0;
    } else if ((ch == 10 || ch == 11) && (func_en & FEN(FUNC_ADC))) {
        ok = 0;
    } else if (func == IOF_ADC && !ioext_def[ch].adc_ok) {
        ok = 0;                          // R_CAP_UNSUPPORTED (6/12 ADC channels)
    } else {
        ioext_release(ch);               // release the old config
        switch (func) {
        case IOF_DIN:
            // pull: 0=none, 1=up, 2=down (stm32: pull_up 1 / -1)
            c->in = gpio_in_setup(ioext_def[ch].pin,
                                  arg == 1 ? 1 : (arg == 2 ? -1 : 0));
            break;
        case IOF_DOUT:
            c->out_level = arg & 1;
            c->shutdown_level = (arg >> 1) & 1;
            c->max_on_ms = ((arg >> 8) & 0xFFFF) * 1000;
            c->on_since_ms = g_ms;
            c->out = gpio_out_setup(ioext_def[ch].pin, c->out_level);
            break;
        case IOF_PWM:
            c->pwm_period_ms = clamp_u32(arg, 10, 10000);
            c->pwm_duty = 0;
            c->pwm_phase_ms = g_ms;
            c->shutdown_level = 0;
            c->out = gpio_out_setup(ioext_def[ch].pin, 0);
            break;
        case IOF_ADC:
            c->adc = gpio_adc_setup(ioext_def[ch].pin);
            c->adc_val = 0;
            break;
        }
        c->func = func;
        c->module_owned = 0;
    }
    if (!ok)
        cfgerr = 1;
    sendf("lll_io_ack ch=%c func=%c ok=%c", ch, func, ok);
}
DECL_COMMAND(command_lll_io_cfg, "lll_io_cfg ch=%c func=%c arg=%u");

void
command_lll_io_write(uint32_t *args)
{
    host_seen();
    uint8_t ch = args[0];
    uint32_t val = args[1];
    if (ch >= IOEXT_NCH)
        return;
    struct ioext_ch *c = &ioch[ch];
    if (c->func == IOF_DOUT) {
        c->out_level = !!val;
        c->on_since_ms = g_ms;
        gpio_out_write(c->out, c->out_level);
    } else if (c->func == IOF_PWM) {
        c->pwm_duty = val > 255 ? 255 : val;
    }
}
DECL_COMMAND(command_lll_io_write, "lll_io_write ch=%c val=%u");

void
command_lll_io_read(uint32_t *args)
{
    host_seen();
    uint8_t ch = args[0];
    uint32_t val = 0;
    uint8_t ok = 1;
    if (ch >= IOEXT_NCH) {
        ok = 0;
    } else {
        struct ioext_ch *c = &ioch[ch];
        switch (c->func) {
        case IOF_DIN:  val = gpio_in_read(c->in); break;
        case IOF_ADC:  val = c->adc_val; break;
        case IOF_DOUT: val = c->out_level; break;
        case IOF_PWM:  val = c->pwm_duty; break;
        default:       ok = 0;
        }
    }
    sendf("lll_io ch=%c val=%u ok=%c", ch, val, ok);
}
DECL_COMMAND(command_lll_io_read, "lll_io_read ch=%c");

void
command_lll_unlock(uint32_t *args)
{
    host_seen();
    unlocked = (args[0] == UNLOCK_KEY);
    ack(FUNC_IOEXT, 0xFC, unlocked, unlocked);
}
DECL_COMMAND(command_lll_unlock, "lll_unlock key=%u");

// ------------------------------------------------------------------
// Fail-safe shutdown: IRQ OFF -> ONLY gpio_out_write calls (no UART).
// EN high = driver disabled ; IOEXT douts fall back to their shutdown_val.
// ------------------------------------------------------------------
void
buffer_shutdown(void)
{
    gpio_out_write(en_out, 1);
    for (uint8_t i = 0; i < IOEXT_NCH; i++) {
        struct ioext_ch *c = &ioch[i];
        if (c->func == IOF_DOUT || c->func == IOF_PWM)
            gpio_out_write(c->out, c->shutdown_level);
    }
}
DECL_SHUTDOWN(buffer_shutdown);

// ------------------------------------------------------------------
// ms clock (wrap-safe: the 32-bit tick wraps at ~89 s, g_ms at ~49 d)
// ------------------------------------------------------------------
static void
millis_update(void)
{
    uint32_t now = timer_read_time();
    ms_acc += now - ms_last_ticks;
    ms_last_ticks = now;
    if (ms_acc >= TICKS_PER_MS) {
        g_ms += ms_acc / TICKS_PER_MS;
        ms_acc %= TICKS_PER_MS;
    }
}

// tracking of present/arm edges (telemetry)
static void
track_edges(void)
{
    uint8_t present = !gpio_in_read(fil);
    if (present != prev_present) {
        if (prev_present != 0xFF)
            ev_send(EV_PRESENT, present);
        prev_present = present;
    }
    uint8_t arm = (gpio_in_read(hall3) << 2) | (gpio_in_read(hall2) << 1)
                  | gpio_in_read(hall1);
    if (arm != prev_arm) {
        if (prev_arm != 0xFF)
            ev_send(EV_ARM, arm);
        prev_arm = arm;
    }

    // MDM presence (PA4, T3): push EV_MDM on edge. ASSUMPTION, unverified on
    // real hardware (same caveat the old fork flagged for this pin): active
    // low, i.e. raw 0 = filament present, mirroring the FIL_PIN/"present"
    // convention above. Confirm on the bench once MDM is wired.
    uint8_t mdm_present = !gpio_in_read(mdm_presence);
    if (mdm_present != prev_mdm_present) {
        if (prev_mdm_present != 0xFF)
            ev_send(EV_MDM, mdm_present);
        prev_mdm_present = mdm_present;
    }

    // MDM motion (PA5, T3): free-running pulse counter, deliberately NOT an
    // event -- an encoder pulse train would flood the serial link and starve
    // other telemetry. Exposed as a field in lll_state instead; the host
    // diffs successive samples, exactly as it already diffs the Roadrunner's
    // `position`. Counts BOTH edges (old fork parity,
    // buffer_manager_old/klippy_src/buffer.c:616-620) to double resolution
    // and stay robust to an unknown/asymmetric pulse duty cycle. No
    // firmware-side stall latch (see comment at prev_mdm_present decl) --
    // stall policy is host-side and tunable.
    uint8_t mdm_mot = gpio_in_read(mdm_motion);
    if (mdm_mot != mdm_motion_prev) {
        mdm_motion_count++;
        mdm_motion_prev = mdm_mot;
    }
}

// 60 s feed watchdog -- ALWAYS active on every forward (auto AND commanded)
static uint32_t feed_start_ms_v;

static uint8_t
feed_watchdog(void)
{
    if (!is_feeding) {
        is_feeding = 1;
        feed_start_ms_v = g_ms;
        return 0;
    }
    if (ELAPSED_MS(feed_start_ms_v) > feed_timeout_ms) {
        set_ecode(EC_FEED_TIMEOUT, 1);
        is_feeding = 0;
        return 1;
    }
    return 0;
}

// ------------------------------------------------------------------
// TASK -- the single priority ladder from spec §4
// ------------------------------------------------------------------
void
buffer_task(void)
{
    // 0. shutdown guard -- LITERALLY the first line (fix #1)
    if (sched_is_shutdown()) {
        gpio_out_write(en_out, 1);
        buffer_shutdown();               // IOEXT douts -> shutdown_val (idempotent)
        host_jog = 0; host_run = 0; is_feeding = 0; manual = 0; armed = 0;
        last_state = ST_STOP;
        tmc_ready = 0;                   // full TMC resync on resume (fix #6)
        return;
    }
    millis_update();

    // boot grace period (fix #9): latch evaluated on EVERY tick, even when
    // the flow does not reach down to step 9 (no filament, error, parked)
    // -> g_ms wrap (49.7 d) can NEVER re-arm a grace period.
    if (!grace_done && (host_ever || g_ms >= BOOT_GRACE_MS))
        grace_done = 1;

    // 1. lazy TMC init (timer ready on the 1st tick, not in DECL_INIT)
    if (!tmc_ready) {
        tmc_configure();
        tmc_ready = 1;
        return;
    }

    track_edges();
    tmc_apply_pending();
    diag_engine();
    led_engine();
    adc_engine();
    ioext_engine();

    // requested error clear (1=host, 2=button click): does something only
    // if there is an error/ecode to clear (a swap click with no error no
    // longer spams EV_ERR_CLEAR nor creates a phantom ecode=5); refused if
    // DIAG is still asserted (-> ecode 5); cfgerr is only purged by the HOST
    if (clear_req) {
        uint8_t from_host = (clear_req == 1);
        clear_req = 0;
        if (is_error || ecode != EC_NONE) {
            if (diag_assert && (func_en & FEN(FUNC_DIAG))) {
                ecode = EC_CLEAR_PENDING;    // pending: DIAG must drop
            } else {
                is_error = 0;
                ecode = EC_NONE;
                parked_back_on = 0;          // fresh follow window
                if (from_host)
                    cfgerr = 0;
                ev_send(EV_ERR_CLEAR, 0);
            }
        } else if (from_host) {
            cfgerr = 0;                      // host clear with no error: purge
        }
    }
    // completion of a pending clear once DIAG drops
    if (ecode == EC_CLEAR_PENDING && !diag_assert) {
        is_error = 0;
        ecode = EC_NONE;
        parked_back_on = 0;
        ev_send(EV_ERR_CLEAR, 0);
    }

    // explicit stop (lll_stop): immediate stop, autonomy resumes on the next
    // tick per role/mode (north-star "keep buffering")
    if (stop_req) {
        stop_req = 0;
        is_feeding = 0;
        apply_state(ST_STOP);
        return;
    }

    // 2. host-alive (opt-in): expired -> kills HOST motion only,
    // autonomy continues (north-star)
    if (host_alive_ms && ELAPSED_MS(last_host_ms) > host_alive_ms
        && !alive_tripped) {
        alive_tripped = 1;
        if (host_jog || host_run) {
            host_jog = 0;
            host_run = 0;
            is_feeding = 0;
            set_ecode(EC_HOST_ALIVE, 0);
            apply_state(ST_STOP);
            return;
        }
        set_ecode(EC_HOST_ALIVE, 0);
    }

    // 3. BUTTONS/MBSIG (standalone parity: work even parked, except for the gate)
    if ((func_en & (FEN(FUNC_BUTTONS) | FEN(FUNC_MBSIG)))
        && (!btn_gate_role || role == ROLE_ACTIVE)) {
        uint8_t bi = process_buttons();
        if (bi == 1 || bi == 2) {
            // safety floor: a hold driven by MBSIG ALONE (no human on the
            // button) is bounded by feed/back_timeout (stuck signal != user)
            if (mb_only_hold) {
                uint32_t lim = (bi == 1) ? feed_timeout_ms : back_timeout_ms;
                if (ELAPSED_MS(hold_start_ms) > lim) {
                    set_ecode(bi == 1 ? EC_FEED_TIMEOUT : EC_BACK_TIMEOUT, 1);
                    manual = 0;
                    mb_tripped = 1;      // re-arms only on deassertion
                    apply_state(ST_STOP);
                    return;
                }
            }
            is_feeding = 0;              // freeze the feed clock during a hold
            apply_state(bi == 1 ? ST_FORWARD : ST_BACK);
            return;
        }
        if (bi == 3) { apply_state(ST_STOP); return; }   // hold end
    }

    // 4. JOG (explicit override, deadline <= 10 s)
    if (host_jog) {
        if (ELAPSED_MS(jog_start_ms) < jog_ms) {
            is_feeding = 0;              // freeze the feed clock during a jog
            apply_state(host_jog == 1 ? ST_FORWARD : ST_BACK);
            return;
        }
        host_jog = 0;
        apply_state(ST_STOP);
        return;
    }

    // 5. RUN (bounded ; FWD requires present && !error -- checked EVERY tick, fix #3)
    if (host_run) {
        if (ELAPSED_MS(run_start_ms) >= run_ms) {
            host_run = 0;
            is_feeding = 0;
            apply_state(ST_STOP);
            return;
        }
        if (host_run == 1) {
            if (gpio_in_read(fil) || is_error) {         // runout or error
                host_run = 0;
                is_feeding = 0;
                apply_state(ST_STOP);
                return;
            }
            // semi-manual: the commanded feed auto-stops at MID/TAUT
            if (feed_mode == FM_SEMI_MAN
                && (gpio_in_read(hall2) || gpio_in_read(hall1))) {
                host_run = 0;
                is_feeding = 0;
                apply_state(ST_STOP);
                return;
            }
            if (feed_watchdog()) {                       // 60 s ALWAYS (fix)
                host_run = 0;
                apply_state(ST_STOP);
                return;
            }
            apply_state(ST_FORWARD);
            return;
        }
        // BACK: allowed on runout (unload) but bounded by back_timeout (fix #4)
        if (ELAPSED_MS(run_start_ms) > back_timeout_ms) {
            host_run = 0;
            set_ecode(EC_BACK_TIMEOUT, 1);
            apply_state(ST_STOP);
            return;
        }
        is_feeding = 0;
        apply_state(ST_BACK);
        return;
    }

    // 6. ENTRANCE: no more filament -> STOP the auto paths
    if ((func_en & FEN(FUNC_ENTRANCE)) && gpio_in_read(fil)) {
        apply_state(ST_STOP);
        is_feeding = 0;
        // standalone parity: removing the filament clears a jam (feed OR
        // back timeout -- both physically resolve by pulling the
        // filament) ; NOT a diag_fault (hardware) nor a clear-pending
        if (is_error && (ecode == EC_FEED_TIMEOUT || ecode == EC_BACK_TIMEOUT)) {
            is_error = 0;
            ecode = EC_NONE;
            ev_send(EV_ERR_CLEAR, 0);
        }
        return;
    }

    // 7. latched error -> STOP
    if (is_error) {
        apply_state(ST_STOP);
        is_feeding = 0;
        return;
    }

    // 8. ROLE
    if (role == ROLE_STANDBY) {
        apply_state(ST_STOP);
        is_feeding = 0;
        return;
    }
    if (role == ROLE_PARKED) {
        // directional freeze: follows a rewind (hall1) so as not to block
        // the unload, NEVER forward ; bounded by back_timeout (fix #4)
        is_feeding = 0;
        if (gpio_in_read(hall1)) {
            if (!parked_back_on) {
                parked_back_on = 1;
                parked_back_ms = g_ms;
            }
            if (ELAPSED_MS(parked_back_ms) > back_timeout_ms) {
                set_ecode(EC_BACK_TIMEOUT, 1);
                parked_back_on = 0;      // fresh window after clear
                apply_state(ST_STOP);
                return;
            }
            apply_state(ST_BACK);
        } else {
            parked_back_on = 0;
            apply_state(ST_STOP);
        }
        return;
    }

    // 9. FEED per mode (role == ACTIVE) -- standalone hall state machine
    if (!(func_en & FEN(FUNC_FEED)) || feed_mode == FM_MANUAL) {
        apply_state(ST_STOP);
        is_feeding = 0;
        return;
    }
    // boot grace period (fix #9): after a clean MCU reset, the task runs on
    // compiled defaults (active/auto) before the config arrives ->
    // NO autonomous motion from step 9 (forward feed AND active-role
    // back-follow) while the host has NEVER spoken and < 20 s. Under
    // Klipper, the 1st command (config replay/poll) arrives within a few
    // seconds and ends the grace period. Buttons, jog, run, roles, and
    // parked follow (steps 3-8) = unaffected. ASSUMED standalone trade-off
    // (not parity): a board that is never driven waits 20 s at boot before
    // auto-feed, where the original Mellow firmware starts ~1 s after
    // boot -- price accepted to close off the post-reset phantom push.
    if (!grace_done) {
        apply_state(ST_STOP);
        is_feeding = 0;
        return;
    }
    uint8_t h1 = gpio_in_read(hall1), h2 = gpio_in_read(hall2),
            h3 = gpio_in_read(hall3);
    uint8_t state;
    if (feed_mode == FM_SEMI_MAN) {
        // no auto-start forward ; tension-release follow is kept
        state = h1 ? ST_BACK : ST_STOP;
    } else {
        // AUTO / SEMI-AUTO: standalone FSM (no hall = keep the state)
        if (h3)      state = ST_FORWARD;
        else if (h2) state = ST_STOP;
        else if (h1) state = ST_BACK;
        else         state = last_state;
        if (feed_mode == FM_SEMI_AUTO) {
            // cycle consumed at MID only if a feed actually ran
            if (h2 && last_state == ST_FORWARD)
                armed = 0;
            if (state == ST_FORWARD && !armed)
                state = ST_STOP;                 // not armed: no feed
        }
    }
    if (state == ST_FORWARD) {
        if (feed_watchdog()) {
            apply_state(ST_STOP);
            return;
        }
    } else {
        is_feeding = 0;
    }
    apply_state(state);
}
DECL_TASK(buffer_task);
