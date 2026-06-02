#!/usr/bin/env python3
"""
Curses TUI for MDX desktop mills.

  Top half  — live machine state: XYZA coordinates, flags, status
  Bottom half — scrolling log output (Python logging)

Keybindings:
  ←/→        X axis  −/+
  ↑/↓        Y axis  −/+
  a / z      Z axis  +/−  (a raises, z lowers)
  [/]        A axis  −/+
  Esc        cancel in-flight jog
  f          cycle jog speed: vslow / slow / medium / fast (default slow)
  1/2/3/4/5  XYZ step: 0.01/0.1/1.0/10.0/50.0 mm  A step: 0.01/0.1/1.0/10.0/90.0°
  s          spindle on / off (toggle)
  d          A-axis rotary drilling on / off (toggle; Drill Workpiece dialog)
  < / >      spindle target RPM  −500 / +500
  - / +      spindle & feed override %  −10 / +10
  c          open coordinate systems dialog (activate / move-to / overwrite)
  m          open Move-To picker (presets + User Specify numeric entry)
  p          extras — log axis snapshot (RE: unknown_config_reads 0x5f0+0x3804)
  q          quit

Cut panel (visible when --file is given):
  r          run — stream file continuously in bulk mode
  x          step mode — pause before each block
  n / Space  send next block (step mode)
  p          pause bulk run (enter step mode)

Run: python3 -m mdx40a.ui.tui [-v|-vv] [--file <file.nc>]
"""

import argparse
import collections
import curses
import logging
import time
from typing import Optional, Tuple

from .. import machine as _machine
from .. import trace as _trace
from ..machine import (FLAG_VIEW_LED, FLAG_DOOR, FLAG_SPINDLE, FLAG_CMD_MOVE,
                       FLAG_TOOLBTN, FLAG_NC_READY, FLAG_MOVING, FLAG_BUSY,
                       FLAG_ERROR, FLAG_STATE, FLAG_STATE_SHIFT, JOG_SPEED_MAX,
                       STATE_MAP)
from ..trace import Tracer
from ..cutjob import CutJob

# ── Jog parameters ────────────────────────────────────────────────────────────

STEPS_LINEAR = [0.01, 0.1, 1.0, 10.0, 50.0]   # XYZ jog distances (mm)
STEPS_ROTARY = [0.01, 0.1, 1.0, 10.0, 90.0]   # A jog distances (degrees)

# Jog speed presets cycled by the `f` key (mm/min).
# slow = VPanel continuous-jog ramp start (240); fast = firmware max (0xFFFF).
SPEED_PRESETS = [
    ('vslow',  120),
    ('slow',   240),
    ('medium', 480),
    ('fast',   JOG_SPEED_MAX),
]

# ── Colour pair IDs ───────────────────────────────────────────────────────────

_CP_HEADER   = 1
_CP_LABEL    = 2
_CP_VALUE    = 3
_CP_MOVING   = 4
_CP_STATUS   = 5
_CP_KEYS     = 6
_CP_SEP      = 7
_CP_LOG_DBG  = 8
_CP_LOG_INFO = 9
_CP_LOG_WARN = 10
_CP_LOG_ERR  = 11
_CP_ACTIVE   = 12   # active / selected row in WCS dialog
_CP_DIM      = 13   # dimmed / unavailable


# ── Curses log handler ────────────────────────────────────────────────────────

class _LogBuffer(logging.Handler):
    """Captures log records into a fixed-size deque for the TUI log pane."""

    _FMT = logging.Formatter(
        '%(asctime)s.%(msecs)03d  %(levelname)-7s  %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )

    def __init__(self, maxlines: int = 500):
        super().__init__()
        self._lines: collections.deque = collections.deque(maxlen=maxlines)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            text = self._FMT.format(record)
        except Exception:
            text = record.getMessage()
        self._lines.append((record.levelno, text))

    def tail(self, n: int) -> list[Tuple[int, str]]:
        return list(self._lines)[-n:]


# ── TUI ───────────────────────────────────────────────────────────────────────

class TUI:
    def __init__(self, machine: _machine.MDX40A, log_buf: _LogBuffer,
                 tracer: Optional[Tracer] = None):
        self._m            = machine
        self._log          = log_buf
        self._tracer       = tracer
        self._step_i       = 2           # index into STEPS_LINEAR / STEPS_ROTARY (default 1.0mm / 1.0°)
        self._speed_i      = 1           # index into SPEED_PRESETS; `f` cycles (default slow)
        self._moving       : Optional[str] = None   # axis currently jogging (None → idle)
        self._jog_armed    = False                   # True between send_jog() and post-motion settle
        self._jog_sent_at  = 0.0                     # monotonic timestamp of last send_jog()
        self._quit         = False
        self._last_poll    = 0.0                     # monotonic timestamp of last machine.poll()
        # WCS overlay dialog state
        self._wcs_open           = False
        self._wcs_sel            = 0        # selected row: 0=MCS, 1-10=WCS1-10
        self._wcs_data           : Optional[list] = None   # list[11] of (x,y,z,a)|None
        self._wcs_loading        = False
        self._coord_entry_pending = False   # set by Move picker's User Specify entry; consumed in run loop
        # Move-To picker state ('m' key)
        self._move_open          = False
        self._move_sel           = 0
        self._drill_active        = False   # A-axis rotary drill mode (SET 0x3809)
        # NC cut panel
        self._cut_job: Optional[CutJob] = None
        # Tool diameter offsets dialog
        self._tool_open     = False
        self._tool_sel      = 0
        self._tool_data     : Optional[list] = None   # list[8] float|None
        self._tool_editing  = False
        self._tool_edit_buf = ''

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self, stdscr: curses.window) -> None:
        self._init_colors()
        curses.curs_set(0)
        stdscr.nodelay(True)
        # getch() returns every 100 ms — our cooperative tick
        stdscr.timeout(100)
        # ncurses defaults ESCDELAY to ~1000 ms to see if Esc is the start of an
        # arrow/function-key sequence. Drop it to 25 ms so Esc is delivered on
        # the next tick.
        curses.set_escdelay(25)

        while not self._quit:
            try:
                key = stdscr.getch()
            except curses.error:
                key = -1

            if key == curses.KEY_RESIZE:
                stdscr.clear()
            elif key != -1:
                self._handle_key(key)

            if self._coord_entry_pending:
                self._coord_entry_pending = False
                self._coord_entry_dialog(stdscr)
                stdscr.clear()

            # Ping handoff — drive machine polling from the main loop.
            now = time.monotonic()
            if now - self._last_poll >= _machine.POLL_INTERVAL:
                self._m.poll()
                self._last_poll = now
                self._update_jog_indicator()

            if self._cut_job:
                self._cut_job.service()

            self._draw(stdscr)

    # ── NC file ───────────────────────────────────────────────────────────────

    def load_nc_file(self, path: str) -> None:
        """Load an NC/RML file and arm the cut panel in step mode."""
        self._cut_job = CutJob.from_file(self._m, path)
        self._cut_job.start_step()

    # ── Colour setup ──────────────────────────────────────────────────────────

    def _init_colors(self) -> None:
        curses.start_color()
        curses.use_default_colors()

        def P(idx, fg, bg=-1):
            curses.init_pair(idx, fg, bg)

        P(_CP_HEADER,   curses.COLOR_BLACK,  curses.COLOR_CYAN)
        P(_CP_LABEL,    curses.COLOR_CYAN,   -1)
        P(_CP_VALUE,    curses.COLOR_WHITE,  -1)
        P(_CP_MOVING,   curses.COLOR_YELLOW, -1)
        P(_CP_STATUS,   curses.COLOR_GREEN,  -1)
        P(_CP_KEYS,     curses.COLOR_CYAN,   -1)
        P(_CP_SEP,      curses.COLOR_WHITE,  -1)
        P(_CP_LOG_DBG,  curses.COLOR_WHITE,  -1)
        P(_CP_LOG_INFO, curses.COLOR_WHITE,  -1)
        P(_CP_LOG_WARN, curses.COLOR_YELLOW, -1)
        P(_CP_LOG_ERR,  curses.COLOR_RED,    -1)
        P(_CP_ACTIVE,   curses.COLOR_BLACK,  curses.COLOR_WHITE)
        P(_CP_DIM,      curses.COLOR_WHITE,  -1)

    # ── Input ─────────────────────────────────────────────────────────────────

    def _handle_key(self, key: int) -> None:
        # Modal dialogs swallow all keys while open
        if self._wcs_open:
            self._wcs_handle_key(key)
            return
        if self._tool_open:
            self._tool_handle_key(key)
            return
        if self._move_open:
            self._move_handle_key(key)
            return

        if key == 27:                                  # Esc — cancel an in-flight jog
            if self._jog_armed:
                self._m.stop_motion()
                self._jog_armed = False
                self._moving    = None
            return

        if key in (ord('q'), ord('Q')):
            if self._moving:
                self._m.stop_motion()
            if self._cut_job:
                self._cut_job.abort()
            self._quit = True
            return

        # NC cut panel keys (when a file is loaded)
        if self._cut_job:
            job = self._cut_job
            if key in (ord('r'), ord('R')):
                self._annotate_nc_key('r', 'RUN', job)
                job.start_run()
                return
            if key in (ord('x'), ord('X')):
                self._annotate_nc_key('x', 'STEP', job)
                job.start_step()
                return
            if key in (ord('n'), ord('N'), ord(' '), 10, 13):
                self._annotate_nc_key(chr(key) if 32 <= key < 127 else 'ENTER',
                                      'NEXT_BLOCK', job)
                job.next_block()
                return
            if key in (ord('p'), ord('P')):
                self._annotate_nc_key('p', 'PAUSE', job)
                job.pause()
                return

        JOG_KEYS = {
            curses.KEY_RIGHT: ('X', +1),
            curses.KEY_LEFT:  ('X', -1),
            curses.KEY_UP:    ('Y', +1),
            curses.KEY_DOWN:  ('Y', -1),
            ord('a'):         ('Z', +1),   # a → raise Z
            ord('A'):         ('Z', +1),
            ord('z'):         ('Z', -1),   # z → lower Z
            ord('Z'):         ('Z', -1),
            ord(']'):         ('A', +1),
            ord('['):         ('A', -1),
        }
        if key in JOG_KEYS:
            axis, sign = JOG_KEYS[key]
            self._start_jog(axis, sign)
            return

        if key in (ord('f'), ord('F')):
            self._speed_i = (self._speed_i + 1) % len(SPEED_PRESETS)
            self.annotate(f"KEY f  speed={SPEED_PRESETS[self._speed_i][0]}")
        elif key in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5')):
            self._step_i = key - ord('1')
            lin = STEPS_LINEAR[self._step_i]
            rot = STEPS_ROTARY[self._step_i]
            self.annotate(f"KEY {chr(key)}  step={lin}mm/{rot}°")
        elif key in (ord('s'), ord('S')):
            self._toggle_spindle()
        elif key == ord('<'):
            self._adjust_spindle_rpm(-500)
        elif key == ord('>'):
            self._adjust_spindle_rpm(+500)
        elif key in (ord('-'), ord('_')):
            self._adjust_overrides(-10)
        elif key in (ord('+'), ord('=')):
            self._adjust_overrides(+10)
        elif key in (ord('d'), ord('D')):
            self._toggle_drill_mode()
        elif key in (ord('c'), ord('C')):
            self._wcs_open_dialog()
        elif key in (ord('m'), ord('M')):
            self._move_open_dialog()
        elif key in (ord('t'), ord('T')):
            self._tool_open_dialog()
        elif key in (ord('p'), ord('P')):
            self._m.fetch_axis_snapshot()

    def annotate(self, msg: str) -> None:
        """Write a `# <msg>` marker to the trace, if a tracer was attached.

        Centralises the previous `t = _trace.get_active(); if t: t.annotate(...)`
        pattern so callers can just write `self.annotate("…")` and stay
        decoupled from the global-state plumbing.
        """
        if self._tracer is not None:
            self._tracer.annotate(msg)

    def _annotate_nc_key(self, key_label: str, action: str, job: CutJob) -> None:
        """Write a `# KEY <k> <action> ...` marker for an NC-panel key press."""
        self.annotate(
            f"KEY {key_label}  {action}  "
            f"state={job.state}  block={job.block_idx}/{job.total}  "
            f"file={job.filename!r}"
        )

    def _start_jog(self, axis: str, sign: int) -> None:
        if self._jog_armed:
            return  # previous jog still settling — ignore (no overlap)
        step  = STEPS_ROTARY[self._step_i] if axis == 'A' else STEPS_LINEAR[self._step_i]
        dist  = sign * step
        speed = SPEED_PRESETS[self._speed_i][1]

        unit = '°' if axis == 'A' else 'mm'
        self.annotate(
            f"JOG {axis} {dist:+.3f}{unit}  speed={speed}  cmd=0x4f5/displacement"
        )

        try:
            self._m.send_jog(axis, dist, speed=speed)
        except Exception as exc:
            logging.getLogger('tui').error("Jog %s %+.3f mm failed: %s", axis, dist, exc)
            return
        self._moving     = axis
        self._jog_armed  = True
        self._jog_sent_at = time.monotonic()

    def _update_jog_indicator(self) -> None:
        """Clear the moving marker once the firmware reports the axes idle.

        Gates on state-block FLAG_MOVING | FLAG_CMD_MOVE. These are the bits
        that clear empirically when a fixed-step jog (SET 0x04f5) finishes.

        Note on the other candidates:
          - State-block FLAG_BUSY (bit 13) is asserted in normal operation —
            unsuitable for motion gating.
          - Ping-word bits 2/21 (machine.is_busy) are what VPanel's
            jog_wait_motion_complete @ 0x00417b00 polls, but only for the
            absolute-move path (send_abs_move → SET 0x04f7). The fixed-step
            jog path (dev_send_trigger_data → wait_move_bit_clear) waits on
            ping bit 22 instead, and the firmware doesn't drive bits 2/21
            during a jog — leaving is_busy stuck after every jog.

        Two conditions, to avoid clearing before motion has started:
          - at least 200 ms elapsed since send_jog (firmware needs a tick to
            raise the bits);
          - neither MOVING nor CMD_MOVE set on the cached state.
        """
        if not self._jog_armed:
            return
        if time.monotonic() - self._jog_sent_at < 0.2:
            return
        s = self._m.state
        if s.flags & (FLAG_MOVING | FLAG_CMD_MOVE):
            return
        self._jog_armed = False
        self._moving    = None

    def _toggle_spindle(self) -> None:
        s = self._m.state
        if s.flags & FLAG_SPINDLE:
            self._m.spindle_off()
        else:
            self._m.spindle_on_rpm(self._m.spindle_target_rpm)

    def _toggle_drill_mode(self) -> None:
        """Toggle A-axis rotary drilling mode."""
        self._drill_active = not self._drill_active
        self._m.rotary_drill_mode(self._drill_active)

    def _adjust_spindle_rpm(self, delta: int) -> None:
        new_rpm = self._m.spindle_target_rpm + delta
        self.annotate(f"KEY <>  spindle_target_rpm={new_rpm}")
        # While the spindle is off, only adjust the cached target — `s` will
        # push it to the device when the user actually starts the spindle.
        # While running, push live so the new RPM takes effect immediately.
        if self._m.state.flags & FLAG_SPINDLE:
            self._m.spindle_on_rpm(new_rpm)
        else:
            self._m.set_target_rpm(new_rpm)

    def _adjust_overrides(self, delta: int) -> None:
        """Adjust spindle speed % and cutting feed % together (+/- keys)."""
        pct = max(10, min(200, self._m.spindle_speed_pct + delta))
        s = self._m.state
        if s.flags & FLAG_SPINDLE:
            self._m.set_spindle_speed(pct)
        else:
            self._m.set_spindle_speed_cached(pct)
        self._m.set_cutting_feed(pct)
        self.annotate(f"KEY +-  override_pct={pct}")

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _draw(self, stdscr: curses.window) -> None:
        rows, cols = stdscr.getmaxyx()
        stdscr.erase()

        state_h = rows // 2
        sep1    = state_h
        self._draw_state(stdscr, state_h, cols)
        self._draw_separator(stdscr, sep1, cols)

        if self._cut_job:
            nc_row = sep1 + 1
            nc_h   = max(4, min(9, rows - nc_row - 3))
            sep2   = nc_row + nc_h
            log_row = sep2 + 1
            log_h   = rows - log_row
            self._draw_nc_panel(stdscr, nc_row, nc_h, cols)
            self._draw_separator(stdscr, sep2, cols)
        else:
            log_row = sep1 + 1
            log_h   = rows - log_row

        self._draw_log(stdscr, log_row, log_h, cols)

        try:
            stdscr.refresh()
        except curses.error:
            pass

        if self._wcs_open:
            self._draw_wcs_dialog(stdscr, rows, cols)

        if self._tool_open:
            self._draw_tool_dialog(stdscr, rows, cols)

        if self._move_open:
            self._draw_move_dialog(stdscr, rows, cols)

    def _draw_state(self, win: curses.window, height: int, cols: int) -> None:
        s     = self._m.state
        CP    = curses.color_pair
        BOLD  = curses.A_BOLD

        # Row 0 — title bar
        speed_lbl = SPEED_PRESETS[self._speed_i][0]
        lin_lbl   = STEPS_LINEAR[self._step_i]
        rot_lbl   = STEPS_ROTARY[self._step_i]
        wcs_lbl   = 'MCS' if self._m.active_wcs == 0 else f'WCS{self._m.active_wcs}'
        rotary_lbl = {0: "No Extension", 1: "Rotary Axis", 2: "Rotary Vice"}.get(
            self._m.rotary_extension_byte, "?")
        title     = f" Roland MDX-40A  │  {speed_lbl}  │  XYZ {lin_lbl}mm  A {rot_lbl}°  │  {wcs_lbl}  │  {rotary_lbl} "
        self._put(win, 0, 0, title.ljust(cols), CP(_CP_HEADER) | BOLD)

        if height < 3:
            return

        # Rows 2-5 — coordinate pairs, two per row (WCS-relative when WCS active)
        row = 2
        dx, dy, dz, da = self._display_xyza(s)
        axes = [
            ('X', dx, 'mm'),
            ('Y', dy, 'mm'),
            ('Z', dz, 'mm'),
            ('A', da, '° '),
        ]
        for i in range(0, 4, 2):
            if row >= height:
                break
            for col_off, (axis, val, unit) in zip((0, cols // 2), axes[i:i+2]):
                moving  = (self._moving == axis)
                v_attr  = CP(_CP_MOVING) | BOLD if moving else CP(_CP_VALUE) | BOLD
                l_attr  = CP(_CP_LABEL)
                marker  = ' ◀▶' if moving else '   '
                self._put(win, row, col_off + 2, axis + ' :', l_attr)
                self._put(win, row, col_off + 6, f'{val:+11.3f} {unit}{marker}', v_attr)
            row += 1

        # Row: raw flags hex
        row += 1
        if row < height:
            self._put(win, row, 2, f'flags  0x{s.flags:08X}  ping  0x{self._m._last_ping_word:04X}', CP(_CP_LABEL))

        # Row: decoded flags
        row += 1
        if row < height:
            self._draw_flag_bits(win, row, s.flags, cols)

        # Row: status / state enum
        row += 1
        if row < height:
            state_num = (s.flags & _machine.FLAG_STATE) >> _machine.FLAG_STATE_SHIFT
            state_str = STATE_MAP.get(state_num, f'#{state_num}')
            self._put(win, row, 2, f'state  {state_str:<12} ', CP(_CP_LABEL))

        # Row: spindle speed + on/off
        # Layout: spindle  OFF  live 9000  tgt  9000  ×100% =  9000 RPM  feed 100%  runtime Xh XXm
        #         RE: actual_rpm = MulDiv(target_rpm, pct, 100) @ update_state_and_coords
        #         <> keys set target RPM (SET 0x3901); +- keys set both override %s together
        row += 1
        if row < height:
            spindle_on  = bool(s.flags & FLAG_SPINDLE)
            spd_pct     = self._m.spindle_speed_pct
            feed_pct    = self._m.cutting_feed_pct
            tgt_rpm     = self._m.spindle_target_rpm
            actual_rpm  = tgt_rpm * spd_pct // 100
            live        = s.spindle_rpm
            live_str    = f'{live:5d}' if live is not None else '    ?'
            state_str   = 'ON ' if spindle_on else 'off'
            state_attr  = (CP(_CP_STATUS) | BOLD) if spindle_on else CP(_CP_LABEL)
            self._put(win, row,  2, 'spindle', CP(_CP_LABEL))
            self._put(win, row, 10, state_str, state_attr)
            self._put(win, row, 14, f'live {live_str}', CP(_CP_VALUE))
            self._put(win, row, 25, f'tgt {tgt_rpm:5d}', CP(_CP_VALUE))
            self._put(win, row, 35, f'×{spd_pct:3d}% = {actual_rpm:5d} RPM',
                      CP(_CP_VALUE) | BOLD)
            self._put(win, row, 54, f'feed {feed_pct:3d}%', CP(_CP_LABEL))
            if self._drill_active:
                self._put(win, row, 66, 'DRILL', CP(_CP_LOG_WARN) | BOLD)
            secs = self._m.spindle_secs
            if secs is not None:
                h, m = secs // 3600, (secs % 3600) // 60
                self._put(win, row, 73, f'runtime {h}h {m:02d}m', CP(_CP_LABEL))

        # Row: rotary axis centreline origin (only shown when rotary is installed)
        rotary_byte = self._m.rotary_extension_byte
        if rotary_byte is not None and rotary_byte >= 1:
            row += 1
            if row < height:
                cl = self._m.rotary_centerline
                if cl is not None:
                    cx, cy, cz = cl
                    text = f'Rotary origin  X: {cx:.3f}   Y: {cy:.3f}   Z: {cz:.3f}'
                else:
                    text = 'Rotary origin  (not yet read)'
                self._put(win, row, 2, text, CP(_CP_LABEL))

        # Rows: key reference (near bottom of state pane)
        ref_row = height - 2
        if ref_row > row + 1:
            key_lines = [
                '  jog: ←→ X   ↑↓ Y   a/z Z   [] A    f speed (vslow/slow/medium/fast)   1-5 step',
                '  s spindle   <> RPM   d A-drill   -/+ override%   c coords   m move-to   t tools   q quit',
            ]
            for i, line in enumerate(key_lines):
                r = ref_row + i
                if r < height:
                    self._put(win, r, 0, line, CP(_CP_KEYS))

    def _draw_flag_bits(self, win: curses.window, row: int, flags: int, cols: int) -> None:
        """Render a compact decoded flag line, e.g.: [SPINDLE] [MTR_PWR] [MOVING]"""
        CP   = curses.color_pair
        BOLD = curses.A_BOLD

        # Each entry: (mask, label, active_cp, inactive_cp_or_None)
        # None for inactive_cp means don't show when inactive
        BITS = [
            (FLAG_VIEW_LED, 'VIEW_LED', _CP_LOG_ERR, _CP_LOG_INFO),
            (FLAG_DOOR,     'DOOR',     _CP_LOG_ERR, _CP_LOG_INFO),
            (FLAG_SPINDLE,  'SPINDLE',  _CP_STATUS,  _CP_LOG_INFO),
            (FLAG_TOOLBTN,  'TOOLBTN',  _CP_MOVING,  _CP_LOG_INFO),
            (FLAG_NC_READY, 'NC_READY', _CP_STATUS,  _CP_LOG_INFO),
            (FLAG_CMD_MOVE, 'CMD_MOVE', _CP_MOVING,  _CP_LOG_INFO),
            (FLAG_MOVING,   'MOVING',   _CP_MOVING,  _CP_LOG_INFO),
            (FLAG_BUSY,     'BUSY',     _CP_MOVING,  _CP_LOG_INFO),
            (FLAG_ERROR,    'ERROR',    _CP_LOG_ERR, _CP_LOG_INFO),
        ]

        x = 2
        self._put(win, row, x, 'bits   ', CP(_CP_LABEL))
        x += 7
        for mask, label, act_cp, inact_cp in BITS:
            active = bool(flags & mask)
            if active:
                self._put(win, row, x, f'[{label}]', CP(act_cp) | BOLD)
                x += len(label) + 3
            elif inact_cp is not None:
                self._put(win, row, x, f' {label} ', CP(inact_cp))
                x += len(label) + 3
            if x >= cols - 2:
                break

    # ── WCS helpers ───────────────────────────────────────────────────────────

    def _display_xyza(self, s: _machine.MachineState) -> tuple:
        """Subtract active WCS origin from machine coords for display."""
        ox, oy, oz, oa = self._m.wcs_offset
        return (s.x_mm - ox, s.y_mm - oy, s.z_mm - oz, (s.a_deg - oa) % 360.0)

    def _wcs_open_dialog(self) -> None:
        self._wcs_open    = True
        self._wcs_sel     = self._m.active_wcs   # start cursor on active slot
        self._wcs_data    = None
        self._wcs_loading = True
        self._wcs_load()

    def _wcs_load(self) -> None:
        data = [(0.0, 0.0, 0.0, 0.0)]   # index 0 = MCS always zero
        for slot in range(1, 11):
            data.append(self._m.get_wcs_origin(slot))
        self._wcs_data    = data
        self._wcs_loading = False

    def _wcs_handle_key(self, key: int) -> None:
        if key in (27, ord('q'), ord('Q')):       # Esc / q — close
            self._wcs_open = False
            return
        if key == curses.KEY_UP:
            self._wcs_sel = max(0, self._wcs_sel - 1)
        elif key == curses.KEY_DOWN:
            self._wcs_sel = min(10, self._wcs_sel + 1)
        elif key in (10, 13):                      # Enter — activate
            self._m.set_active_wcs(self._wcs_sel)
        elif key in (ord('m'), ord('M')):          # Move to stored origin
            if self._wcs_sel == 0:
                return   # MCS origin is always (0,0,0,0) — no-op / already there
            if self._wcs_data and self._wcs_data[self._wcs_sel]:
                ox, oy, oz, oa = self._wcs_data[self._wcs_sel]
                self._m.move_to_machine_pos(ox, oy, oz, oa)
        elif key in (ord('x'), ord('X'),
                     ord('y'), ord('Y'),
                     ord('z'), ord('Z'),
                     ord('a'), ord('A')):          # Overwrite one axis with current position
            if self._wcs_sel == 0:
                return   # cannot overwrite MCS
            slot = self._wcs_sel
            axis = chr(key).upper()
            if self._m.partial_update_wcs_origin(slot, axis) and self._wcs_data:
                self._wcs_data[slot] = self._m.get_wcs_origin(slot)
        elif key in (ord('r'), ord('R')):         # Reload all origins from device
            self._wcs_data    = None
            self._wcs_loading = True
            self._wcs_load()

    def _draw_wcs_dialog(self, stdscr: curses.window, rows: int, cols: int) -> None:
        CP   = curses.color_pair
        BOLD = curses.A_BOLD

        dh = min(18, rows - 2)
        dw = min(76, cols - 2)
        dy = (rows - dh) // 2
        dx = (cols - dw) // 2

        try:
            win = curses.newwin(dh, dw, dy, dx)
        except curses.error:
            return

        win.erase()
        win.box()

        # Title row
        loading = '  loading…' if self._wcs_loading else ''
        title = f' Coordinate Systems{loading}'
        win.addstr(0, 2, title[:dw - 4], CP(_CP_HEADER) | BOLD)

        if dh < 5:
            win.refresh()
            return

        # Column headers
        hdr = f"{'':4s}  {'X (mm)':>12s}  {'Y (mm)':>12s}  {'Z (mm)':>12s}  {'A (°)':>10s}"
        win.addstr(1, 1, hdr[:dw - 2], CP(_CP_LABEL))
        win.addstr(2, 1, '─' * (dw - 2), CP(_CP_LABEL))

        # Data rows: 0=MCS, 1-10=WCS1-10
        for i in range(min(11, dh - 5)):
            row = 3 + i
            if row >= dh - 3:
                break
            label   = 'MCS ' if i == 0 else f'WC{i:<2d}'
            is_active   = (i == self._m.active_wcs)
            is_selected = (i == self._wcs_sel)

            if self._wcs_data and self._wcs_data[i] is not None:
                x, y, z, a = self._wcs_data[i]
                vals = f'{x:>+12.3f}  {y:>+12.3f}  {z:>+12.3f}  {a:>+10.3f}'
            elif self._wcs_loading:
                vals = f'{"…":>12s}  {"…":>12s}  {"…":>12s}  {"…":>10s}'
            else:
                vals = f'{"?":>12s}  {"?":>12s}  {"?":>12s}  {"?":>10s}'

            suffix = ' ACT' if is_active else '    '
            line   = f' {label} {vals} {suffix}'

            if is_selected:
                attr = CP(_CP_ACTIVE) | BOLD
            elif is_active:
                attr = CP(_CP_STATUS) | BOLD
            else:
                attr = CP(_CP_VALUE)

            try:
                win.addstr(row, 1, line[:dw - 2], attr)
            except curses.error:
                pass

        # Key reference — separator at dh-3, keys at dh-2, border at dh-1
        ref_row = dh - 3
        keys = ' ↑↓ navigate   ↵ activate   M move to   XYZA overwrite axis   Esc close'
        win.addstr(ref_row,     1, '─' * (dw - 2), CP(_CP_LABEL))
        win.addstr(ref_row + 1, 1, keys[:dw - 2],  CP(_CP_KEYS))

        win.refresh()

    # ── Tool diameter offsets dialog ──────────────────────────────────────────

    def _tool_open_dialog(self) -> None:
        self._tool_open     = True
        self._tool_sel      = 0
        self._tool_editing  = False
        self._tool_edit_buf = ''
        self._tool_data     = self._m.get_tool_offsets()

    def _tool_handle_key(self, key: int) -> None:
        if self._tool_editing:
            self._tool_edit_key(key)
            return
        if key in (27, ord('q'), ord('Q')):
            self._tool_open = False
        elif key == curses.KEY_UP:
            self._tool_sel = max(0, self._tool_sel - 1)
        elif key == curses.KEY_DOWN:
            self._tool_sel = min(7, self._tool_sel + 1)
        elif key in (ord('e'), ord('E'), 10, 13):
            if self._tool_data is not None:
                val = self._tool_data[self._tool_sel]
                self._tool_edit_buf = f'{val:.3f}' if val is not None else ''
                self._tool_editing  = True
        elif key in (ord('r'), ord('R')):
            self._tool_editing = False
            self._tool_data    = self._m.get_tool_offsets()

    def _tool_edit_key(self, key: int) -> None:
        if key == 27:   # Esc — cancel
            self._tool_editing  = False
            self._tool_edit_buf = ''
        elif key in (10, 13):   # Enter — commit
            try:
                val  = float(self._tool_edit_buf)
                slot = self._tool_sel + 1
                if self._tool_data is not None:
                    self._tool_data[self._tool_sel] = val
                self._m.set_tool_offset(slot, val)
            except ValueError:
                pass
            self._tool_editing  = False
            self._tool_edit_buf = ''
        elif key in (127, curses.KEY_BACKSPACE, 8):   # Backspace
            self._tool_edit_buf = self._tool_edit_buf[:-1]
        elif 32 <= key < 128:
            ch = chr(key)
            if ch in '0123456789.' or (ch == '-' and not self._tool_edit_buf):
                if len(self._tool_edit_buf) < 10:
                    self._tool_edit_buf += ch

    def _draw_tool_dialog(self, stdscr: curses.window, rows: int, cols: int) -> None:
        CP   = curses.color_pair
        BOLD = curses.A_BOLD

        dh = min(15, rows - 2)
        dw = min(56, cols - 2)
        dy = (rows - dh) // 2
        dx = (cols - dw) // 2

        try:
            win = curses.newwin(dh, dw, dy, dx)
        except curses.error:
            return

        win.erase()
        win.box()

        win.addstr(0, 2, ' Tool Diameter Offsets'[:dw - 4], CP(_CP_HEADER) | BOLD)

        if dh < 5:
            win.refresh()
            return

        hdr = f"  {'Slot':4s}  {'Value (mm)':>12s}"
        win.addstr(1, 1, hdr[:dw - 2], CP(_CP_LABEL))
        win.addstr(2, 1, '─' * (dw - 2), CP(_CP_LABEL))

        for i in range(8):
            row = 3 + i
            if row >= dh - 3:
                break
            is_sel = (i == self._tool_sel)

            if self._tool_data and self._tool_data[i] is not None:
                val_str = f'{self._tool_data[i]:>12.3f}'
            else:
                val_str = f'{"?":>12s}'

            if is_sel and self._tool_editing:
                line = f'  T{i + 1:<3d}  {val_str}  → {self._tool_edit_buf}_'
                attr = CP(_CP_ACTIVE) | BOLD
            elif is_sel:
                line = f'  T{i + 1:<3d}  {val_str}'
                attr = CP(_CP_ACTIVE) | BOLD
            else:
                line = f'  T{i + 1:<3d}  {val_str}'
                attr = CP(_CP_VALUE)

            try:
                win.addstr(row, 1, line[:dw - 2], attr)
            except curses.error:
                pass

        ref_row = dh - 3
        keys = ' ↑↓ navigate   Enter/E edit   R reload   Esc close'
        win.addstr(ref_row,     1, '─' * (dw - 2), CP(_CP_LABEL))
        win.addstr(ref_row + 1, 1, keys[:dw - 2],  CP(_CP_KEYS))

        win.refresh()

    # ── Move-To picker ('m' key) ──────────────────────────────────────────────

    def _move_targets(self) -> list:
        """Return [(label, action_callable_or_None), ...] for the picker.

        Each callable is invoked on Enter and takes no args. The User Specified
        entry uses None as a sentinel — the handler closes the picker and
        queues the existing numeric-entry modal via `_coord_entry_pending`.
        Rotary-only entries are gated on the live `rotary_extension_byte`.
        """
        m = self._m
        targets = [
            ('View Position',     lambda: m.move_to_view_position()),
            ('X Origin',          lambda: m.move_to_origin(0x1)),
            ('Y Origin',          lambda: m.move_to_origin(0x2)),
            ('Z Origin',          lambda: m.move_to_origin(0x4)),
            ('XY Origin',         lambda: m.move_to_origin(0x3)),
        ]
        rotary = m.rotary_extension_byte
        if rotary is not None and rotary >= 1:
            targets.append(('A Origin',          lambda: m.move_to_origin(0x8)))
        if rotary is not None and rotary >= 2:
            targets.append(('Rotation center Y', lambda: m.move_to_rotation_center_y()))
        targets.append(('User Specified…', None))   # None → open numeric entry dialog
        return targets

    def _move_open_dialog(self) -> None:
        self._move_open = True
        self._move_sel  = 0

    def _move_handle_key(self, key: int) -> None:
        targets = self._move_targets()
        if key in (27, ord('q'), ord('Q')):
            self._move_open = False
            return
        if key == curses.KEY_UP:
            self._move_sel = max(0, self._move_sel - 1)
        elif key == curses.KEY_DOWN:
            self._move_sel = min(len(targets) - 1, self._move_sel + 1)
        elif key in (10, 13):
            label, action = targets[self._move_sel]
            self.annotate(f"KEY m  move target={label!r}")
            if action is None:
                # User Specify — close picker, queue numeric entry modal
                self._move_open = False
                self._coord_entry_pending = True
            else:
                try:
                    action()
                except Exception as exc:
                    logging.getLogger('tui').error("Move target %r failed: %s", label, exc)

    def _draw_move_dialog(self, stdscr: curses.window, rows: int, cols: int) -> None:
        CP   = curses.color_pair
        BOLD = curses.A_BOLD

        targets = self._move_targets()

        # Clamp selection in case rotary status changed since open.
        if self._move_sel >= len(targets):
            self._move_sel = len(targets) - 1

        dh = min(6 + len(targets), rows - 2)
        dw = min(48, cols - 2)
        dy = (rows - dh) // 2
        dx = (cols - dw) // 2

        try:
            win = curses.newwin(dh, dw, dy, dx)
        except curses.error:
            return

        win.erase()
        win.box()

        wcs_lbl = 'MCS' if self._m.active_wcs == 0 else f'WCS{self._m.active_wcs}'
        title = f' Move To  (using {wcs_lbl})'
        win.addstr(0, 2, title[:dw - 4], CP(_CP_HEADER) | BOLD)
        win.addstr(1, 1, '─' * (dw - 2), CP(_CP_LABEL))

        for i, (label, _action) in enumerate(targets):
            row = 2 + i
            if row >= dh - 3:
                break
            is_sel = (i == self._move_sel)
            attr   = CP(_CP_ACTIVE) | BOLD if is_sel else CP(_CP_VALUE)
            try:
                win.addstr(row, 1, f'  {label} '.ljust(dw - 2)[:dw - 2], attr)
            except curses.error:
                pass

        ref_row = dh - 3
        keys = ' ↑↓ navigate   ↵ execute   Esc close'
        win.addstr(ref_row,     1, '─' * (dw - 2), CP(_CP_LABEL))
        win.addstr(ref_row + 1, 1, keys[:dw - 2],  CP(_CP_KEYS))

        win.refresh()

    # ── Coordinate entry dialog (blocking, 'c' key) ───────────────────────────

    def _coord_entry_dialog(self, stdscr: curses.window) -> None:
        """Modal dialog: user types XYZA target in active CS, machine moves there."""
        s  = self._m.state
        dx, dy, dz, da = self._display_xyza(s)
        rows, cols = stdscr.getmaxyx()

        dh, dw = 13, 52
        wy = max(0, (rows - dh) // 2)
        wx = max(0, (cols - dw) // 2)

        try:
            win = curses.newwin(dh, dw, wy, wx)
        except curses.error:
            return

        CP   = curses.color_pair
        BOLD = curses.A_BOLD
        wcs_lbl = 'MCS' if self._m.active_wcs == 0 else f'WCS{self._m.active_wcs}'

        win.erase()
        win.box()
        win.addstr(0, 2, f' Move to position ({wcs_lbl}) '[:dw - 4],
                   CP(_CP_HEADER) | BOLD)
        win.addstr(1, 2, 'Leave blank to keep current value.', CP(_CP_LABEL))
        win.addstr(2, 2, '─' * (dw - 4), CP(_CP_LABEL))
        win.addstr(9, 2, '─' * (dw - 4), CP(_CP_LABEL))
        win.addstr(10, 2, '[Enter] move   [↑↓] field   [Esc] cancel', CP(_CP_KEYS))

        # Blocking, no-echo input — we draw chars manually so Esc/Backspace work.
        curses.noecho()
        curses.curs_set(1)
        win.nodelay(False)
        win.keypad(True)

        axes_info = [
            ('X', dx, 'mm'),
            ('Y', dy, 'mm'),
            ('Z', dz, 'mm'),
            ('A', da, '° '),
        ]
        bufs      = ['', '', '', '']   # per-axis edit buffer
        sel       = 0                   # currently-selected axis
        cancelled = False
        MAX_LEN   = 8

        # Pre-compute the input column once — all 4 prompts have the same width.
        sample_prompt = f'  X (mm)  current {0.0:>+10.3f}  → '
        inp_x = min(1 + len(sample_prompt), dw - 10)

        def redraw():
            for i, (axis, cur, unit) in enumerate(axes_info):
                row = 3 + i
                prompt = f'  {axis} ({unit})  current {cur:>+10.3f}  → '
                attr   = (CP(_CP_ACTIVE) | BOLD) if i == sel else CP(_CP_LABEL)
                win.addstr(row, 1, prompt[:dw - 2], attr)
                # Render this axis's buffer, padded with spaces so backspace shows through.
                win.addstr(row, inp_x, bufs[i].ljust(MAX_LEN), CP(_CP_VALUE) | BOLD)
            win.move(3 + sel, inp_x + len(bufs[sel]))
            win.refresh()

        try:
            while True:
                redraw()
                try:
                    ch = win.getch()
                except curses.error:
                    cancelled = True
                    break
                if ch == 27:                                       # Esc — cancel
                    cancelled = True
                    break
                if ch in (10, 13):                                 # Enter — commit
                    break
                if ch == curses.KEY_UP:
                    sel = max(0, sel - 1)
                elif ch == curses.KEY_DOWN:
                    sel = min(len(axes_info) - 1, sel + 1)
                elif ch in (curses.KEY_BTAB,):                     # Shift-Tab
                    sel = max(0, sel - 1)
                elif ch == 9:                                       # Tab
                    sel = min(len(axes_info) - 1, sel + 1)
                elif ch in (127, curses.KEY_BACKSPACE, 8):
                    if bufs[sel]:
                        bufs[sel] = bufs[sel][:-1]
                elif 32 <= ch < 128 and len(bufs[sel]) < MAX_LEN:
                    c = chr(ch)
                    if (c.isdigit()
                            or (c == '.' and '.' not in bufs[sel])
                            or (c == '-' and not bufs[sel])):
                        bufs[sel] += c
        finally:
            curses.curs_set(0)

        if cancelled:
            return

        # Parse buffers; empty → keep current display value.
        defaults = (dx, dy, dz, da)
        final_display = []
        for b, default in zip(bufs, defaults):
            b = b.strip()
            if not b:
                final_display.append(default)
                continue
            try:
                final_display.append(float(b))
            except ValueError:
                final_display.append(default)

        # Convert WCS-relative display coords back to machine coords
        ox, oy, oz, oa = self._m.wcs_offset
        mx = final_display[0] + ox
        my = final_display[1] + oy
        mz = final_display[2] + oz
        ma = final_display[3] + oa

        self._m.move_to_machine_pos(mx, my, mz, ma)

    def _draw_nc_panel(self, win: curses.window, start: int, height: int, cols: int) -> None:
        if not self._cut_job or height < 2:
            return
        job  = self._cut_job
        CP   = curses.color_pair
        BOLD = curses.A_BOLD

        idx   = job.block_idx
        total = job.total
        st    = job.state
        pct   = int(100 * idx / total) if total else 0

        mode_lbl = {
            CutJob.IDLE:     'IDLE',
            CutJob.STEPPING: 'STEP',
            CutJob.WAITING:  'WAIT',
            CutJob.RUNNING:  'RUN ',
            CutJob.DONE:     'DONE',
            CutJob.ERROR:    'ERR ',
        }.get(st, st[:4].upper())

        bar_w  = max(4, min(20, cols // 5))
        filled = int(bar_w * idx / total) if total else 0
        bar    = '█' * filled + '░' * (bar_w - filled)

        err_suffix = f'  {job.error}' if st == CutJob.ERROR and job.error else ''
        hdr = (f'  {mode_lbl}  {job.filename}  │  '
               f'Block {idx:4d}/{total:<4d}  [{bar}] {pct:3d}%'
               f'{err_suffix}')
        hdr_attr = CP(_CP_LOG_ERR) | BOLD if st == CutJob.ERROR else CP(_CP_LABEL) | BOLD
        self._put(win, start, 0, hdr, hdr_attr)

        # Key hint on the right of the header row
        hint = 'r run  x step  n/Spc next  p pause'
        hint_col = max(0, cols - len(hint) - 1)
        self._put(win, start, hint_col, hint, CP(_CP_KEYS))

        # Context lines: 2 before cursor, cursor highlighted, rest after
        view_start = max(0, idx - 2)
        for off, bi in enumerate(range(view_start, min(total, view_start + height - 1))):
            r = start + 1 + off
            if r >= start + height:
                break
            txt = job.line_at(bi)
            num = f'{bi + 1:4d}'
            if bi == idx:
                self._put(win, r, 0, f' ▶ {num}  {txt}', CP(_CP_ACTIVE) | BOLD)
            elif bi < idx:
                self._put(win, r, 0, f'   {num}  {txt}', CP(_CP_DIM))
            else:
                self._put(win, r, 0, f'   {num}  {txt}', CP(_CP_VALUE))

    def _draw_separator(self, win: curses.window, row: int, cols: int) -> None:
        rows, _ = win.getmaxyx()
        if row >= rows:
            return
        self._put(win, row, 0, '─' * (cols - 1), curses.color_pair(_CP_SEP))

    def _draw_log(self, win: curses.window, start: int, height: int, cols: int) -> None:
        if height <= 0:
            return
        rows, _ = win.getmaxyx()
        lines   = self._log.tail(height)
        # Pin most-recent line to bottom
        screen_row = start + max(0, height - len(lines))
        for level, text in lines:
            if screen_row >= rows:
                break
            if level >= logging.ERROR:
                attr = curses.color_pair(_CP_LOG_ERR)
            elif level >= logging.WARNING:
                attr = curses.color_pair(_CP_LOG_WARN)
            else:
                attr = curses.color_pair(_CP_LOG_INFO)
            self._put(win, screen_row, 0, text, attr)
            screen_row += 1

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _put(win: curses.window, y: int, x: int, text: str, attr: int = 0) -> None:
        rows, cols = win.getmaxyx()
        if y < 0 or y >= rows or x >= cols:
            return
        avail = cols - x
        if avail <= 0:
            return
        try:
            # Never write into the very last cell (bottom-right corner triggers error)
            safe = text.replace('\x00', '·')
            win.addstr(y, x, safe[:avail - (1 if y == rows - 1 else 0)], attr)
        except curses.error:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='MDX-40A interactive TUI')
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='-v INFO  -vv DEBUG')
    parser.add_argument('--file', '-f', metavar='FILE',
                        help='NC/RML file to load in the cut panel')
    parser.add_argument('--mock', action='store_true',
                        help='Mock USB layer — run without a physical device')
    args = parser.parse_args(argv)

    level = {0: logging.WARNING, 1: logging.INFO}.get(args.verbose, logging.DEBUG)

    # Capture log into our buffer *before* connecting so handshake msgs appear
    log_buf = _LogBuffer()
    log_buf.setLevel(logging.DEBUG)
    root = logging.getLogger()
    root.addHandler(log_buf)
    root.setLevel(level)
    logging.getLogger('usb').setLevel(logging.WARNING)
    # Our own modules emit INFO during discover/handshake/jog — make sure those
    # land in `log_buf` even at -v=0 (which sets root to WARNING). Setting the
    # `mdx40a` logger to an explicit floor bypasses root's level filter for any
    # `mdx40a.*` sublogger (since effective-level walks stop at the first
    # non-NOTSET ancestor). -vv still enables DEBUG for our code.
    logging.getLogger('mdx40a').setLevel(min(level, logging.INFO))

    from ..usb import MdxUSB, MdxMockUSB

    raw_link = MdxMockUSB() if args.mock else MdxUSB.discover()
    with _trace.open_trace() as t, t.wrap_link(raw_link) as link:
        import sys
        t.annotate(f"argv: {' '.join(sys.argv)}")
        m = _machine.MDX40A(link)
        tui = TUI(m, log_buf, t)
        if args.file:
            tui.load_nc_file(args.file)
        curses.wrapper(tui.run)


if __name__ == '__main__':
    main()
