#!/usr/bin/env python3
"""
Curses TUI for Roland MDX-40A.

  Top half  — live machine state: XYZA coordinates, flags, status
  Bottom half — scrolling log output (Python logging)

Keybindings:
  ←/→        X axis  −/+
  ↑/↓        Y axis  −/+
  a / z      Z axis  +/−  (a raises, z lowers)
  [/]        A axis  −/+
  f          toggle fast / slow jog speed
  1/2/3/4/5  XYZ step: 0.01/0.1/1.0/10.0/50.0 mm  A step: 0.01/0.1/1.0/10.0/90.0°
  s          spindle on / off (toggle)
  < / >      spindle target RPM  −500 / +500
  - / +      spindle & feed override %  −10 / +10
  q / ESC    quit

Run: sudo python3 -m mdx40a.ui.tui [-v|-vv]
"""

import argparse
import collections
import curses
import logging
import threading
import time
from typing import List, Optional, Tuple

from .. import machine as _machine
from .. import trace as _trace
from ..machine import (FLAG_DOOR, FLAG_SPINDLE, FLAG_CMD_MOVE, FLAG_TOOLBTN,
                       FLAG_MOVING, FLAG_BUSY, FLAG_ERROR,
                       FLAG_STATE, FLAG_STATE_SHIFT, STATE_MAP)
from . import log as _log

# ── Jog parameters ────────────────────────────────────────────────────────────

STEPS_LINEAR = [0.01, 0.1, 1.0, 10.0, 50.0]   # XYZ jog distances (mm)
STEPS_ROTARY = [0.01, 0.1, 1.0, 10.0, 90.0]   # A jog distances (degrees)

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


# ── Curses log handler ────────────────────────────────────────────────────────

class _LogBuffer(logging.Handler):
    """Captures log records into a fixed-size deque for the TUI log pane."""

    _FMT = logging.Formatter(
        '%(asctime)s  %(levelname)-7s  %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )

    def __init__(self, maxlines: int = 500):
        super().__init__()
        self._lines: collections.deque = collections.deque(maxlen=maxlines)
        self._lock  = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            text = self._FMT.format(record)
        except Exception:
            text = record.getMessage()
        with self._lock:
            self._lines.append((record.levelno, text))

    def tail(self, n: int) -> List[Tuple[int, str]]:
        with self._lock:
            return list(self._lines)[-n:]


# ── TUI ───────────────────────────────────────────────────────────────────────

class TUI:
    def __init__(self, machine: _machine.MDX40A, log_buf: _LogBuffer):
        self._m            = machine
        self._log          = log_buf
        self._step_i       = 2           # index into STEPS_LINEAR / STEPS_ROTARY (default 1.0mm / 1.0°)
        self._fast         = False
        self._moving       : Optional[str]             = None   # axis currently jogging
        self._jog_thr      : Optional[threading.Thread] = None
        self._quit         = threading.Event()

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self, stdscr: curses.window) -> None:
        self._init_colors()
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(100)     # getch() returns every 100 ms so the screen refreshes

        while not self._quit.is_set():
            try:
                key = stdscr.getch()
            except curses.error:
                key = -1

            if key == curses.KEY_RESIZE:
                stdscr.clear()
            elif key != -1:
                self._handle_key(key)

            self._draw(stdscr)

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

    # ── Input ─────────────────────────────────────────────────────────────────

    def _handle_key(self, key: int) -> None:
        if key in (ord('q'), ord('Q'), 27):
            if self._moving:
                self._m.stop_motion()
            self._quit.set()
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
            self._fast = not self._fast
            t = _trace.get_active()
            if t:
                t.annotate(f"KEY f  speed={'FAST' if self._fast else 'slow'}")
        elif key in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5')):
            self._step_i = key - ord('1')
            lin = STEPS_LINEAR[self._step_i]
            rot = STEPS_ROTARY[self._step_i]
            t = _trace.get_active()
            if t:
                t.annotate(f"KEY {chr(key)}  step={lin}mm/{rot}°")
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

    def _start_jog(self, axis: str, sign: int) -> None:
        if self._jog_thr and self._jog_thr.is_alive():
            return  # previous jog still settling — ignore
        step  = STEPS_ROTARY[self._step_i] if axis == 'A' else STEPS_LINEAR[self._step_i]
        dist  = sign * step
        speed = _machine.JOG_SPEED_FAST if self._fast else _machine.JOG_SPEED_SLOW

        t = _trace.get_active()
        if t:
            unit = '°' if axis == 'A' else 'mm'
            t.annotate(f"JOG {axis} {dist:+.3f}{unit}  speed={speed}  cmd=0x4f5/displacement")

        log = logging.getLogger('tui')

        def _run():
            self._moving = axis
            try:
                self._m.jog(axis, dist, speed=speed)
            except Exception as exc:
                log.error("Jog %s %+.3f mm failed: %s", axis, dist, exc)
            finally:
                self._moving = None

        self._jog_thr = threading.Thread(target=_run, daemon=True, name='jog')
        self._jog_thr.start()

    def _toggle_spindle(self) -> None:
        s = self._m.state
        if s.flags & FLAG_SPINDLE:
            self._m.spindle_off()
        else:
            self._m.spindle_on(self._m.spindle_speed_pct)

    def _adjust_spindle_rpm(self, delta: int) -> None:
        """Adjust configured spindle target RPM via SET 0x3901 (<> keys)."""
        new_rpm = self._m.spindle_target_rpm + delta
        t = _trace.get_active()
        if t:
            t.annotate(f"KEY <>  spindle_target_rpm={new_rpm}")
        threading.Thread(
            target=self._m.set_spindle_rpm, args=(new_rpm,),
            daemon=True, name='rpm-set',
        ).start()

    def _adjust_overrides(self, delta: int) -> None:
        """Adjust spindle speed % and cutting feed % together (+/- keys)."""
        pct = max(10, min(200, self._m.spindle_speed_pct + delta))
        s = self._m.state
        if s.flags & FLAG_SPINDLE:
            self._m.set_spindle_speed(pct)
        else:
            self._m.set_spindle_speed_cached(pct)
        self._m.set_cutting_feed(pct)
        t = _trace.get_active()
        if t:
            t.annotate(f"KEY +-  override_pct={pct}")

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _draw(self, stdscr: curses.window) -> None:
        rows, cols = stdscr.getmaxyx()
        stdscr.erase()

        # Divide screen: top half state, separator, bottom half log
        state_h = rows // 2
        sep_row = state_h
        log_row = sep_row + 1
        log_h   = rows - log_row

        self._draw_state(stdscr, state_h, cols)
        self._draw_separator(stdscr, sep_row, cols)
        self._draw_log(stdscr, log_row, log_h, cols)

        try:
            stdscr.refresh()
        except curses.error:
            pass

    def _draw_state(self, win: curses.window, height: int, cols: int) -> None:
        s     = self._m.state
        CP    = curses.color_pair
        BOLD  = curses.A_BOLD

        # Row 0 — title bar
        speed_lbl = "FAST" if self._fast else "slow"
        lin_lbl   = STEPS_LINEAR[self._step_i]
        rot_lbl   = STEPS_ROTARY[self._step_i]
        title     = f" Roland MDX-40A  │  {speed_lbl}  │  XYZ {lin_lbl}mm  A {rot_lbl}° "
        self._put(win, 0, 0, title.ljust(cols), CP(_CP_HEADER) | BOLD)

        if height < 3:
            return

        # Rows 2-5 — coordinate pairs, two per row
        row = 2
        axes = [
            ('X', s.x_mm,  'mm'),
            ('Y', s.y_mm,  'mm'),
            ('Z', s.z_mm,  'mm'),
            ('A', s.a_deg, '° '),
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
            self._put(win, row, 2, f'flags  0x{s.flags:08X}', CP(_CP_LABEL))

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
        # Layout: spindle  OFF  tgt  9000  ×100% =  9000 RPM  feed 100%  runtime Xh XXm
        #         RE: actual_rpm = MulDiv(target_rpm, pct, 100) @ update_state_and_coords
        #         <> keys set target RPM (SET 0x3901); +- keys set both override %s together
        row += 1
        if row < height:
            spindle_on  = bool(s.flags & FLAG_SPINDLE)
            spd_pct     = self._m.spindle_speed_pct
            feed_pct    = self._m.cutting_feed_pct
            tgt_rpm     = self._m.spindle_target_rpm
            actual_rpm  = tgt_rpm * spd_pct // 100
            state_str   = 'ON ' if spindle_on else 'off'
            state_attr  = (CP(_CP_STATUS) | BOLD) if spindle_on else CP(_CP_LABEL)
            self._put(win, row,  2, 'spindle', CP(_CP_LABEL))
            self._put(win, row, 10, state_str, state_attr)
            self._put(win, row, 14, f'tgt {tgt_rpm:5d}', CP(_CP_VALUE))
            self._put(win, row, 24, f'×{spd_pct:3d}% = {actual_rpm:5d} RPM',
                      CP(_CP_VALUE) | BOLD)
            self._put(win, row, 43, f'feed {feed_pct:3d}%', CP(_CP_LABEL))
            secs = self._m.spindle_secs
            if secs is not None:
                h, m = secs // 3600, (secs % 3600) // 60
                self._put(win, row, 53, f'runtime {h}h {m:02d}m', CP(_CP_LABEL))

        # Rows: key reference (near bottom of state pane)
        ref_row = height - 2
        if ref_row > row + 1:
            key_lines = [
                '  ←→ X   ↑↓ Y   a/z Z   [] A',
                '  f fast/slow   1-5 step   s spindle   <> target RPM   -/+ overrides%   q quit',
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
            (FLAG_DOOR,     'DOOR',     _CP_LOG_ERR, _CP_LOG_INFO),
            (FLAG_SPINDLE,  'SPINDLE',  _CP_STATUS,  _CP_LOG_INFO),
            (FLAG_TOOLBTN,  'TOOLBTN',  _CP_MOVING,  _CP_LOG_INFO),
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
            if screen_row >= rows - 1:
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
    args = parser.parse_args(argv)

    level = {0: logging.WARNING, 1: logging.INFO}.get(args.verbose, logging.DEBUG)

    # Capture log into our buffer *before* connecting so handshake msgs appear
    log_buf = _LogBuffer()
    log_buf.setLevel(logging.DEBUG)
    root = logging.getLogger()
    root.addHandler(log_buf)
    root.setLevel(level)
    logging.getLogger('usb').setLevel(logging.WARNING)

    with _trace.open_trace() as t:
        _trace.set_active(t)
        import sys
        t.annotate(f"argv: {' '.join(sys.argv)}")
        with _machine.MDX40A() as m:
            tui = TUI(m, log_buf)
            curses.wrapper(tui.run)
        _trace.set_active(None)


if __name__ == '__main__':
    main()
