"""Milling-time estimate for G-code blocks.

A small modal interpreter over the blocks from ``parse_nc_blocks``. It is
pure (no machine access) and only used by the lint CLI, never to gate a job.

Model:
  * the tool starts at the workspace origin (0, 0, 0) — positions are in the
    program's own work coordinates, so G54..G59 offsets are ignored;
  * G01/G02/G03 take path length / (F x feed override); a helical arc counts
    its Z travel in the path length;
  * G00 axes move independently: max(XY distance / rapid_xy, Z / rapid_z);
  * G04 dwells for its X or P value in seconds;
  * G92 redefines the work origin: no motion, it just costs ``min_op_s`` and
    sets the current position to its axis values;
  * every motion or dwell block costs at least ``min_op_s``, as a floor for
    per-block communication/bookkeeping latency;
  * no acceleration or corner-slowdown model, so files made of many tiny
    segments will really run slower than this says.

The A axis, G53/G28, feed modes other than units/min and canned cycles are
not modelled.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Optional, Sequence

_MM_PER_INCH = 25.4

# Assumed MDX-40A figures, not measured: override on the command line.
DEFAULT_RAPID_XY = 3000.0   # mm/min
DEFAULT_RAPID_Z = 3000.0    # mm/min
DEFAULT_MIN_OP_S = 0.25

_WORD = re.compile(r'([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))')
_PAREN_COMMENT = re.compile(r'\([^)]*\)')

# Arc plane -> (first axis, second axis, first-axis offset word, second-axis
# offset word, third/helical axis). Ordered so that G02/G03 sense is
# clockwise/counter-clockwise when viewed from the positive third axis.
_PLANES = {
    17: ('X', 'Y', 'I', 'J', 'Z'),
    18: ('Z', 'X', 'K', 'I', 'Y'),
    19: ('Y', 'Z', 'J', 'K', 'X'),
}


@dataclass(frozen=True)
class EstimateOptions:
    rapid_xy: float = DEFAULT_RAPID_XY          # mm/min
    rapid_z: float = DEFAULT_RAPID_Z            # mm/min
    feed_override_pct: float = 100.0            # scales programmed F
    min_op_s: float = DEFAULT_MIN_OP_S          # per motion/dwell block floor


@dataclass
class TimeEstimate:
    total_s: float = 0.0
    cutting_s: float = 0.0
    rapid_s: float = 0.0
    dwell_s: float = 0.0
    command_s: float = 0.0                       # non-motion commands (G92)
    path_mm: float = 0.0
    moves: int = 0
    warnings: list[str] = field(default_factory=list)


def format_duration(seconds: float) -> str:
    """``3725.4`` -> ``'1h 02m 05s'``; under an hour drops the hours."""
    s = int(round(seconds))
    h, rem = divmod(s, 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h}h {m:02d}m {s:02d}s"
    if m:
        return f"{m}m {s:02d}s"
    return f"{s}s"


def _arc_length(
    plane: int, start: dict, end: dict, words: dict, mm: float, ccw: bool,
) -> Optional[float]:
    """Length of an arc in *plane*, including helical travel. None if the
    block has neither a usable centre offset nor radius."""
    a, b, oa, ob, h = _PLANES[plane]
    sa, sb, ea, eb = start[a], start[b], end[a], end[b]

    if oa in words or ob in words:
        ca = sa + words.get(oa, 0.0) * mm
        cb = sb + words.get(ob, 0.0) * mm
    elif 'R' in words:
        r = words['R'] * mm
        da, db = ea - sa, eb - sb
        d = math.hypot(da, db)
        if d == 0:
            return None                          # R form can't do full circles
        half = max(r * r - (d / 2) ** 2, 0.0)
        off = math.sqrt(half) / d
        # Minor arc (r > 0): centre is left of the chord for CCW, right for CW.
        sign = (1 if ccw else -1) * (1 if r > 0 else -1)
        ca = (sa + ea) / 2 - sign * db * off
        cb = (sb + eb) / 2 + sign * da * off
    else:
        return None

    radius = math.hypot(sa - ca, sb - cb)
    a0 = math.atan2(sb - cb, sa - ca)
    a1 = math.atan2(eb - cb, ea - ca)
    sweep = (a1 - a0) if ccw else (a0 - a1)
    sweep %= 2 * math.pi
    if sweep < 1e-9:                             # end == start: full circle
        sweep = 2 * math.pi
    return math.hypot(radius * sweep, end[h] - start[h])


def estimate_time(
    blocks: Sequence[bytes], opts: Optional[EstimateOptions] = None,
) -> TimeEstimate:
    """Estimate run time of *blocks* (already parsed by ``parse_nc_blocks``)."""
    opts = opts or EstimateOptions()
    est = TimeEstimate()
    pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
    mm = 1.0                                     # coordinate units -> mm
    absolute = True
    plane = 17
    mode: Optional[int] = None                   # modal motion G: 0..3
    feed = 0.0                                   # programmed F, units/min
    warned: set[str] = set()
    source = ''                                  # current block, for warnings

    def warn(line_no: int, key: str, msg: str) -> None:
        if key not in warned:                    # once per kind, first line
            warned.add(key)
            est.warnings.append(f"line {line_no}: {msg}: {source}")

    def add(kind: str, seconds: float) -> None:
        seconds = max(seconds, opts.min_op_s)
        est.total_s += seconds
        setattr(est, kind, getattr(est, kind) + seconds)
        est.moves += kind in ('cutting_s', 'rapid_s')

    for line_no, block in enumerate(blocks, start=1):
        source = block.decode('ascii', errors='replace').strip()
        text = _PAREN_COMMENT.sub('', source).split(';', 1)[0].strip()
        if not text or text.startswith(('%', '/')):
            continue
        words: dict[str, float] = {}
        gcodes: list[float] = []
        for letter, value in _WORD.findall(text):
            letter = letter.upper()
            if letter == 'G':
                gcodes.append(float(value))
            else:
                words[letter] = float(value)

        dwell = set_origin = False
        for g in gcodes:
            gi = int(g)
            if g != gi:
                continue                         # G38.2 etc: not modelled
            if gi == 20:
                mm = _MM_PER_INCH
            elif gi == 21:
                mm = 1.0
            elif gi == 90:
                absolute = True
            elif gi == 91:
                absolute = False
            elif gi in _PLANES:
                plane = gi
            elif gi in (0, 1, 2, 3):
                mode = gi
            elif gi == 4:
                dwell = True
            elif gi == 92:
                set_origin = True
            elif gi in (28, 53):
                warn(line_no, f'G{gi}', f"G{gi} is not modelled")

        if 'F' in words:
            feed = words['F']
        if 'A' in words:
            warn(line_no, 'A', "A axis moves are ignored")

        if dwell:
            add('dwell_s', words.get('X', words.get('P', 0.0)))
            continue

        if set_origin:
            for ax in pos:
                if ax in words:
                    pos[ax] = words[ax] * mm
            add('command_s', 0.0)
            continue

        if not any(ax in words for ax in pos):
            continue
        if mode is None:
            warn(line_no, 'nomode', "axis move before any G00/G01/G02/G03")
            continue

        end = {}
        for ax, cur in pos.items():
            if ax not in words:
                end[ax] = cur
            else:
                v = words[ax] * mm
                end[ax] = v if absolute else cur + v

        dx, dy, dz = (end[a] - pos[a] for a in 'XYZ')
        if mode == 0:
            xy = math.hypot(dx, dy)
            est.path_mm += math.sqrt(xy * xy + dz * dz)
            secs = max(xy / opts.rapid_xy, abs(dz) / opts.rapid_z) * 60.0
            add('rapid_s', secs)
        else:
            if mode == 1:
                length = math.sqrt(dx * dx + dy * dy + dz * dz)
            else:
                length = _arc_length(plane, pos, end, words, mm, ccw=(mode == 3))
                if length is None:
                    warn(line_no, 'arc', "arc without I/J/K or R; treated as straight line")
                    length = math.sqrt(dx * dx + dy * dy + dz * dz)
            est.path_mm += length
            eff_feed = feed * mm * opts.feed_override_pct / 100.0   # mm/min
            if eff_feed <= 0:
                warn(line_no, 'nofeed', "cutting move with no feed rate set")
                secs = 0.0
            else:
                secs = length / eff_feed * 60.0
            add('cutting_s', secs)
        pos = end

    return est
