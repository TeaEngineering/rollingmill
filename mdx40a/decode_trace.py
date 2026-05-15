#!/usr/bin/env python3
"""
Offline decoder for Roland MDX-40A USB trace files.

Usage:
    python -m mdx40a.decode_trace ~/roland/20260513-151859.txt
    python -m mdx40a.decode_trace --jog ~/roland/*.txt     # jog commands + coord trail
    python -m mdx40a.decode_trace --state ~/roland/*.txt   # 0x0100 state blocks only
    python -m mdx40a.decode_trace --all ~/roland/foo.txt      # every decoded line
    python -m mdx40a.decode_trace --no-strict ~/roland/foo.txt  # warn but don't abort

Exit codes:
    0  clean (all wValues recognised)
    1  aborted on first unknown wValue (default behaviour)
    2  unknown wValues seen (--no-strict run)
"""

import argparse
import re
import struct
import sys
from pathlib import Path
from typing import Optional

from .machine import FLAG_STATE_SHIFT, STATE_MAP

# ── Flag bit table (mirrors machine.py FLAG_* constants) ─────────────────────
_FLAG_BITS = {
    28: 'DOOR',
    27: 'SPINDLE',
    26: 'CMD_MOVE',
    25: 'TOOLBTN',
    22: 'MOVING',
    13: 'BUSY',
    12: 'ERROR',
}

# ── Trace-line regex ──────────────────────────────────────────────────────────
_LINE_RE = re.compile(
    r'^(\d{2}:\d{2}:\d{2}\.\d+)\s+'
    r'([<>!#])\s+'
    r'(?:(SET|GET)\s+wv=(0x[0-9a-fA-F]+)\s+\d+(?::\s*([0-9a-fA-F]*))?'
    r'|(.+))'
)

# ── Complete wValue table ─────────────────────────────────────────────────────
# Any wValue NOT in this dict produces an "! UNKNOWN" error line.
# Add entries here as new commands are discovered and RE'd.
KNOWN_WVALUES: dict = {
    # ── Handshake / ping ─────────────────────────────────────────────────────
    0x0001: 'ping',
    0x0002: 'machine_type',
    0x0003: 'trigger_response',
    0x0101: 'model_string',
    # ── State blocks ─────────────────────────────────────────────────────────
    0x0100: 'state_block',            # GET only; 32 bytes XYZA + flags + RPM
    0x0200: 'coord_pair',             # GET only; 4-byte position used for step completion
    # ── Motion ───────────────────────────────────────────────────────────────
    0x04f5: 'jog',                    # SET; relative displacement '>HH4i' (spd,0,X,Y,Z,A)
    0x04f6: 'waypoint',               # SET; relative milling waypoint (speed=0xFFFF)
    0x04f7: 'abs_move',               # SET; absolute position '>HH4i' (spd,0xFFFF,X,Y,Z,A)
    # ── Spindle / motor ──────────────────────────────────────────────────────
    0x03f0: 'spindle_on',             # SET bare trigger
    0x03f1: 'spindle_off',            # SET bare trigger
    0x03f2: 'origin_capture',         # SET bare trigger (also used as "resume")
    0x03f3: 'stop',                   # SET bare trigger; immediate motion stop
    0x03f5: 'keepalive',              # SET 1-byte payload; every 200ms
    0x0307: 'cutting_feed_pct',       # SET 1 byte (10-200)
    0x3008: 'spindle_pct',            # SET 1 byte (10-200)
    0x3009: 'spindle_stop',           # SET bare trigger (used in drilling sequences)
    0x3808: 'nc_spindle_speed',       # SET 2×uint16 [RPM, mode=2]; NC job only
    0x3809: 'rotatary_drill',         # SET 2×uint16 [1,0xFFFF]=start / [0,0]=stop (A-axis, Drill Workpiece dialog)
    0x3900: 'spindle_rpm_read',       # Pattern B; GET 0x0003 → 1×uint32 RPM
    0x3901: 'spindle_rpm_write',      # SET 1×uint32 RPM; waits ping bit21
    # ── Operation bracket ────────────────────────────────────────────────────
    0x1109: 'op_bracket',             # SET 1 byte: 0x00=begin, 0xff=end
    # ── Coordinate system selection ──────────────────────────────────────────
    0x3006: 'coord_sys',              # SET 1×uint32 slot (0=MCS, 1-10=WCS1-10)
    # ── WCS origin reads (Pattern B; SET trigger → GET 0x0003 → 4×uint32 XYZA) ─
    0x030b: 'wcs_read_1',
    **{0x3202 + i: f'wcs_read_{i + 2}' for i in range(9)},   # WCS2-10: 0x3202..0x320a
    # ── WCS origin writes (SET wValue + 4×uint32 XYZA) ──────────────────────
    0x030c: 'wcs_write_1',
    **{0x3335 + i: f'wcs_write_{i + 2}' for i in range(9)},  # WCS2-10: 0x3335..0x333d
    # ── 200ms poll heartbeat ─────────────────────────────────────────────────
    0x3005: 'poll_speed_range',       # Pattern B; 2×uint32 [min, max] speed
    0x3003: 'poll_speed',             # Pattern B; 1×uint32 current speed
    0x3800: 'poll_status_byte',       # Pattern B; 1 byte machine status
    0x3b01: 'poll_3b01',              # Pattern B; 4 bytes (timer sync, meaning TBD)
    # ── Device / machine status ──────────────────────────────────────────────
    0x3804: 'device_status',          # Pattern B; 6×uint32; word[0] bit2=busy
    # ── Spindle rotation time ────────────────────────────────────────────────
    0x2405: 'spindle_time_read',      # Pattern B; 16 bytes; uint32[0]=total seconds
    0x2425: 'spindle_time_reset',     # SET bare trigger; waits ping bit21
    # ── Axis / machine config ────────────────────────────────────────────────
    0x2012: 'motion_limits',          # SET 2×uint32; waits ping bit21
    0x3107: 'axis_config',            # SET 6 bytes; waits ping bit21
    **{0x347b + i: f'axis_param_{i + 1}' for i in range(8)},  # 0x347b..0x3482
}

# ── Pattern B trigger set ─────────────────────────────────────────────────────
# SET commands that prime the device; the next GET 0x0003 is their response.
# Tracked by parse_lines so decode_line can dispatch 0x0003 correctly.
_PATTERN_B: frozenset = frozenset({
    0x3804, 0x2405,
    0x3005, 0x3003, 0x3800, 0x3b01,
    0x3900,
    0x0101,
    0x030b,
    *range(0x3202, 0x320b),   # WCS reads 2-10
})

# ── Sentinel objects ──────────────────────────────────────────────────────────
_SUPPRESS = object()   # known line, not interesting enough to print by default


# ── Individual decoders ───────────────────────────────────────────────────────

def _flag_str(flags: int) -> str:
    active = [name for bit, name in sorted(_FLAG_BITS.items(), reverse=True)
              if flags & (1 << bit)]
    state_val = (flags >> FLAG_STATE_SHIFT) & 0x7
    active.insert(0, f"ST={STATE_MAP.get(state_val, str(state_val))}")
    return ' '.join(active)


def decode_state(ts: str, data: bytes) -> str:
    if len(data) < 20:
        return f"{ts}  GET 0x0100  [too short: {data.hex()}]"
    flags = struct.unpack_from('>I', data, 0)[0]
    x, y, z, a = struct.unpack_from('>4i', data, 4)
    rpm = struct.unpack_from('>I', data, 20)[0] if len(data) >= 24 else 0
    return (f"{ts}  STATE  X={x/1000:+9.3f}  Y={y/1000:+9.3f}  "
            f"Z={z/1000:+9.3f}  A={a/1000:+8.3f}  "
            f"flags={flags:08x} [{_flag_str(flags)}]  rpm={rpm}")


def decode_jog(ts: str, data: bytes) -> str:
    if len(data) < 20:
        return f"{ts}  JOG  [too short: {data.hex()}]"
    speed, opts, x, y, z, a = struct.unpack_from('>HH4i', data)
    spd = 'MAX' if speed == 0xFFFF else str(speed)
    return (f"{ts}  JOG  speed={spd}  "
            f"dX={x/1000:+7.3f}  dY={y/1000:+7.3f}  "
            f"dZ={z/1000:+7.3f}  dA={a/1000:+7.3f}")


def decode_abs_move(ts: str, data: bytes) -> str:
    if len(data) < 20:
        return f"{ts}  ABS_MOVE  [too short: {data.hex()}]"
    speed, flags, x, y, z, a = struct.unpack_from('>HH4i', data)
    return (f"{ts}  ABS_MOVE  speed={speed} flags={flags:04x}  "
            f"X={x/1000:+9.3f}  Y={y/1000:+9.3f}  "
            f"Z={z/1000:+9.3f}  A={a/1000:+8.3f}")


def _decode_xyza(ts: str, label: str, data: bytes) -> str:
    if len(data) < 16:
        return f"{ts}  {label}  [too short: {data.hex()}]"
    x, y, z, a = struct.unpack_from('>4i', data)
    return (f"{ts}  {label}  "
            f"X={x/1000:+9.3f}  Y={y/1000:+9.3f}  "
            f"Z={z/1000:+9.3f}  A={a/1000:+8.3f}")


def _decode_device_status(ts: str, data: bytes) -> str:
    if len(data) < 24:
        return f"{ts}  STATUS_0x3804  [too short: {len(data)}b  {data.hex()}]"
    words = struct.unpack_from('>6I', data)
    busy  = bool(words[0] & 0x04)
    flags = ' '.join(n for bit, n in [(2, 'BUSY'), (1, 'b1'), (0, 'b0'), (3, 'b3')]
                     if words[0] & (1 << bit)) or 'idle'
    return (f"{ts}  STATUS_0x3804  busy={'YES' if busy else 'no '}  "
            f"w0=0x{words[0]:08x}[{flags}]  "
            f"[{' '.join(f'{w:08x}' for w in words)}]")


# ── Line parser ───────────────────────────────────────────────────────────────

def parse_lines(path: Path):
    """Yield (ts, direction, wv|None, data, context).

    Annotation ('#') lines: direction='#', wv=None, context=comment text.
    Data lines: context = most recent Pattern-B trigger wValue (for 0x0003 dispatch).
    """
    last_trigger: Optional[int] = None
    with open(path) as f:
        for line in f:
            line = line.rstrip()
            if not line:
                continue
            m = _LINE_RE.match(line)
            if not m:
                continue
            ts, direction, op, wv_str, hex_data, comment = m.groups()
            if op:
                wv   = int(wv_str, 16)
                data = bytes.fromhex(hex_data) if hex_data else b''
                if direction == '>' and wv in _PATTERN_B:
                    last_trigger = wv
                yield ts, direction, wv, data, last_trigger
            else:
                yield ts, '#', None, b'', (comment or '')


# ── Main decode dispatch ──────────────────────────────────────────────────────

def decode_line(
    ts: str,
    direction: str,
    wv: int,
    data: bytes,
    verbose: bool = False,
    last_trigger: Optional[int] = None,
):
    """Return a display string, _SUPPRESS (silent), or _UNKNOWN (unrecognised)."""

    if wv not in KNOWN_WVALUES:
        raise ValueError(f"Unknown wv value {wv}")

    # ── Always-decoded transfers ───────────────────────────────────────────────
    if wv == 0x0100:
        return decode_state(ts, data)

    if wv == 0x04f5:
        return decode_jog(ts, data)

    if wv == 0x04f7:
        return decode_abs_move(ts, data)

    # GET 0x0003 — Pattern B response; dispatch by what triggered it
    if wv == 0x0003 and direction == '<':
        if last_trigger == 0x3804:
            return _decode_device_status(ts, data)
        if last_trigger == 0x2405 and len(data) >= 4:
            secs = struct.unpack_from('>I', data)[0]
            h, m = secs // 3600, (secs // 60) % 60
            return f"{ts}  SPINDLE_TIME  {h}h {m:02d}m  ({secs} s)"
        if last_trigger == 0x3900 and len(data) >= 4:
            rpm = struct.unpack_from('>I', data)[0]
            return f"{ts}  SPINDLE_RPM_CFG  {rpm} RPM"
        if last_trigger is not None and (
            last_trigger == 0x030b or 0x3202 <= last_trigger <= 0x320a
        ):
            slot = 1 if last_trigger == 0x030b else (last_trigger - 0x3202 + 2)
            return _decode_xyza(ts, f"WCS_ORIGIN[{slot}]", data)
        # Unknown trigger response — always show, never suppress
        trig = f'0x{last_trigger:04x}' if last_trigger else '?'
        return f"{ts}  TRIGGER_RESP(from {trig})  {len(data)}b  {data.hex()}"

    # ── Short decoders ────────────────────────────────────────────────────────
    if wv == 0x0001 and direction == '<':
        if not verbose:
            return _SUPPRESS
        if len(data) >= 4:
            resp_len, err = data[3], bool(data[2] & 0x10)
            return (f"{ts}  PING  raw={data.hex()}  resp_len={resp_len}"
                    + ('  ERR' if err else ''))
        return f"{ts}  PING  {data.hex()}"

    if wv == 0x03f5:
        return _SUPPRESS if not verbose else f"{ts}  KEEPALIVE"

    if wv == 0x03f0 and direction == '>':
        return f"{ts}  SPINDLE_ON"

    if wv == 0x03f1 and direction == '>':
        return f"{ts}  SPINDLE_OFF"

    if wv == 0x03f2 and direction == '>':
        return f"{ts}  ORIGIN_CAPTURE / RESUME"

    if wv == 0x03f3 and direction == '>':
        return f"{ts}  STOP (immediate)"

    if wv == 0x1109 and direction == '>' and data:
        action = ('begin' if data[0] == 0x00 else
                  'end'   if data[0] == 0xff else f'0x{data[0]:02x}')
        return f"{ts}  OP_BRACKET  {action}"

    if wv == 0x3006 and direction == '>' and len(data) >= 4:
        slot = struct.unpack_from('>I', data)[0]
        label = 'MCS' if slot == 0 else f'WCS{slot}'
        return f"{ts}  COORD_SYS  → {label}"

    if wv == 0x0307 and direction == '>' and data:
        return f"{ts}  CUTTING_FEED  {data[0]}%"

    if wv == 0x3008 and direction == '>' and data:
        return f"{ts}  SPINDLE_PCT  {data[0]}%"

    if wv == 0x3009 and direction == '>':
        return f"{ts}  SPINDLE_STOP (bare)"

    if wv == 0x3901 and direction == '>' and len(data) >= 4:
        rpm = struct.unpack_from('>I', data)[0]
        return f"{ts}  SPINDLE_RPM_WRITE  {rpm} RPM"

    if wv == 0x3808 and direction == '>' and len(data) >= 4:
        rpm, mode = struct.unpack_from('>HH', data)
        return f"{ts}  NC_SPINDLE_SPEED  {rpm} RPM  mode={mode}"

    if wv == 0x3809 and direction == '>' and len(data) >= 4:
        val, flags = struct.unpack_from('>HH', data)
        return f"{ts}  ROTARY_DRILLING val={val} flags={flags:04x}"

    if wv == 0x2425 and direction == '>':
        return f"{ts}  SPINDLE_TIME_RESET"

    if wv == 0x04f6 and direction == '>':
        return decode_abs_move(ts, data).replace('ABS_MOVE', 'WAYPOINT')

    # WCS origin write (SET with 4×uint32 XYZA)
    if direction == '>' and (wv == 0x030c or 0x3335 <= wv <= 0x333d):
        slot = 1 if wv == 0x030c else (wv - 0x3335 + 2)
        return _decode_xyza(ts, f"WCS_WRITE[{slot}]", data)

    # WCS origin read trigger (SET side of Pattern B)
    if direction == '>' and (wv == 0x030b or 0x3202 <= wv <= 0x320a):
        slot = 1 if wv == 0x030b else (wv - 0x3202 + 2)
        return f"{ts}  > WCS_READ_TRIG[{slot}]" if verbose else _SUPPRESS

    # Device status trigger
    if wv == 0x3804 and direction == '>':
        return f"{ts}  > DEVICE_STATUS_TRIG" if verbose else _SUPPRESS

    # Poll triggers — suppress unless verbose
    if wv in (0x3005, 0x3003, 0x3800, 0x3b01, 0x3900) and direction == '>':
        return _SUPPRESS if not verbose else f"{ts}  > POLL_TRIG {KNOWN_WVALUES[wv]}"

    if wv == 0x3804 and direction == '<':
        return _SUPPRESS   # handled via 0x0003 path above

    # Axis config / params — show as raw in verbose, suppress otherwise
    if wv in (0x3107, 0x2012) or 0x347b <= wv <= 0x3482:
        if not verbose:
            return _SUPPRESS
        return (f"{ts}  {'SET' if direction=='>' else 'GET'} "
                f"{KNOWN_WVALUES[wv]}  {len(data)}b  {data.hex()}")

    if not verbose:
        return _SUPPRESS

    op = 'SET' if direction == '>' else 'GET'
    return f"{ts}  {op} {KNOWN_WVALUES.get(wv, f'0x{wv:04x}')}  {len(data)}b  {data.hex()}"


# ── Display modes ─────────────────────────────────────────────────────────────

def show_jog_trails(path: Path, window_secs: float = 2.0) -> None:
    events = list(parse_lines(path))

    def ts_secs(ts: str) -> float:
        h, m, s = ts.split(':')
        return int(h) * 3600 + int(m) * 60 + float(s)

    for i, (ts, direction, wv, data, _) in enumerate(events):
        if direction == '>' and wv == 0x04f5:
            print()
            print(decode_jog(ts, data))
            t0 = ts_secs(ts)
            for ts2, _d2, wv2, data2, _ in events[i + 1:]:
                if ts_secs(ts2) - t0 > window_secs:
                    break
                if wv2 == 0x0100:
                    print(' ', decode_state(ts2, data2))


def show_all_states(path: Path) -> None:
    for ts, direction, wv, data, _ in parse_lines(path):
        if wv == 0x0100:
            print(decode_state(ts, data))


def show_all(path: Path, verbose: bool = False, strict: bool = False) -> bool:
    """Decode and print all lines. Returns True if no unknown wValues were seen."""
    clean = True
    for ts, direction, wv, data, last_trig in parse_lines(path):
        if direction == '#':
            print(f"# {last_trig}")   # last_trig carries the annotation text here
            continue
        try:
            result = decode_line(ts, direction, wv, data,
                                 verbose=verbose, last_trigger=last_trig)
            if result is _SUPPRESS:
                continue
        except ValueError as v:
            print(f"Unable to decode {ts=} {wv:04x=} {direction=} data={data.hex()}", file=sys.stderr)
            clean = False
            sys.exit(1)
        else:
            print(result)
    return clean


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description='Decode Roland MDX-40A USB trace files')
    ap.add_argument('files', nargs='+', metavar='FILE')
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--jog',   action='store_true',
                      help='show jog commands with coordinate trail')
    mode.add_argument('--state', action='store_true',
                      help='show only 0x0100 state blocks')
    ap.add_argument('-v', '--verbose', action='store_true',
                    help='show ping, keepalive, poll triggers, and raw fallbacks')
    ap.add_argument('--no-strict', dest='strict', action='store_false',
                    help='continue past unknown wValues instead of aborting (exit 2 at end)')
    ap.set_defaults(strict=True)
    ap.add_argument('--window', type=float, default=2.0, metavar='SECS',
                    help='state trail window after each jog in --jog mode (default 2.0)')
    args = ap.parse_args()

    all_clean = True
    for path in (Path(f) for f in args.files):
        if not path.exists():
            print(f"ERROR: {path} not found", file=sys.stderr)
            sys.exit(1)
        if len(args.files) > 1:
            print(f"\n=== {path.name} ===")
        if args.jog:
            show_jog_trails(path, args.window)
        elif args.state:
            show_all_states(path)
        else:
            if not show_all(path, verbose=args.verbose, strict=args.strict):
                all_clean = False

    if not all_clean:
        sys.exit(2)


if __name__ == '__main__':
    main()
