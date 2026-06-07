#!/usr/bin/env python3
"""Render a text string to single-line g-code using a QCAD CXF stroke font.

Reads a CXF font file, lays out the input text, chains connected primitives
into continuous strokes, and emits an RS-274 g-code file with G2/G3 arcs.
The output is intended to be streamed to the MDX-40A via the existing
`rollingmill --file <out.gcode>` TUI flow.

CXF dialects supported:
  - QCAD form:   [0041] A    (hex code point, optional trailing char)
  - Hershey form: [A] 20     (literal char, trailing primitive count)
Both forms use `L x1,y1,x2,y2` for lines. QCAD form additionally uses
`A cx,cy,r,a0,a1` (CCW arc) and `AR cx,cy,r,a0,a1` (CW arc), with angles
in degrees.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


# ---------- primitives ----------

@dataclass
class Line:
    x1: float
    y1: float
    x2: float
    y2: float

    def start(self) -> tuple[float, float]:
        return (self.x1, self.y1)

    def end(self) -> tuple[float, float]:
        return (self.x2, self.y2)

    def reversed(self) -> "Line":
        return Line(self.x2, self.y2, self.x1, self.y1)


@dataclass
class Arc:
    cx: float
    cy: float
    r: float
    a0_deg: float  # start angle (degrees)
    a1_deg: float  # end angle (degrees)
    ccw: bool      # True for CCW (CXF "A"), False for CW (CXF "AR")

    def _pt(self, deg: float) -> tuple[float, float]:
        rad = math.radians(deg)
        return (self.cx + self.r * math.cos(rad),
                self.cy + self.r * math.sin(rad))

    def start(self) -> tuple[float, float]:
        return self._pt(self.a0_deg)

    def end(self) -> tuple[float, float]:
        return self._pt(self.a1_deg)

    def reversed(self) -> "Arc":
        return Arc(self.cx, self.cy, self.r, self.a1_deg, self.a0_deg, not self.ccw)


Primitive = Line | Arc


# ---------- font model ----------

@dataclass
class Glyph:
    char: str
    primitives: list[Primitive] = field(default_factory=list)

    @property
    def width(self) -> float:
        if not self.primitives:
            return 0.0
        max_x = float("-inf")
        for p in self.primitives:
            if isinstance(p, Line):
                max_x = max(max_x, p.x1, p.x2)
            else:
                # Arc bounding box max-x is conservatively cx + r
                max_x = max(max_x, p.cx + p.r)
        return max(0.0, max_x)


@dataclass
class Font:
    glyphs: dict[str, Glyph] = field(default_factory=dict)
    letter_spacing: float = 3.0
    word_spacing: float = 6.75
    name: str = ""

    @property
    def cap_height(self) -> float:
        """Measured height of capital 'H' (or fallback), used for scaling."""
        for ch in ("H", "M", "I", "A"):
            g = self.glyphs.get(ch)
            if g and g.primitives:
                ys: list[float] = []
                for p in g.primitives:
                    if isinstance(p, Line):
                        ys.extend([p.y1, p.y2])
                    else:
                        ys.extend([p.cy - p.r, p.cy + p.r])
                return max(ys) - min(ys)
        return 9.0  # QCAD default


# ---------- CXF parser ----------

_HEADER_RE = re.compile(r"^\[([^\]]+)\]\s*(.*)$")


def parse_cxf(path: Path) -> Font:
    """Parse a CXF font file into a Font. Tolerant of both QCAD and Hershey
    bracket forms, and skips unknown metadata."""
    font = Font(name=path.stem)
    glyph: Glyph | None = None

    with path.open("r", encoding="latin-1") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                glyph = None
                continue
            if line.startswith("#"):
                # Metadata: "# Key: Value"
                m = re.match(r"#\s*([^:]+):\s*(.+)", line)
                if m:
                    key = m.group(1).strip()
                    val = m.group(2).strip()
                    if key == "LetterSpacing":
                        font.letter_spacing = float(val)
                    elif key == "WordSpacing":
                        font.word_spacing = float(val)
                    elif key == "Name" and not font.name:
                        font.name = val
                continue
            m = _HEADER_RE.match(line)
            if m:
                inside = m.group(1)
                trailing = m.group(2).strip()
                ch = _decode_char(inside, trailing)
                if ch is not None:
                    glyph = Glyph(char=ch)
                    font.glyphs[ch] = glyph
                else:
                    glyph = None
                continue
            if glyph is None:
                continue
            prim = _parse_primitive(line)
            if prim is not None:
                glyph.primitives.append(prim)
    return font


def _decode_char(inside: str, trailing: str) -> str | None:
    """Decode the character from a [...] header. QCAD form has a hex code
    inside the brackets; Hershey form has the literal character."""
    inside = inside.strip()
    if not inside:
        return None
    # QCAD form: 1-6 hex digits → code point
    if re.fullmatch(r"[0-9A-Fa-f]{1,6}", inside) and len(inside) >= 2:
        try:
            cp = int(inside, 16)
            return chr(cp)
        except ValueError:
            pass
    # Hershey form: single literal character
    if len(inside) == 1:
        return inside
    # Fallback: if trailing field has a single char, use that
    if len(trailing) == 1:
        return trailing
    return None


def _parse_primitive(line: str) -> Primitive | None:
    """Parse a single CXF primitive line."""
    parts = line.split(None, 1)
    if len(parts) != 2:
        return None
    op, rest = parts[0], parts[1]
    nums = [n.strip() for n in rest.split(",")]
    try:
        vals = [float(n) for n in nums]
    except ValueError:
        return None
    if op == "L" and len(vals) >= 4:
        return Line(vals[0], vals[1], vals[2], vals[3])
    if op == "A" and len(vals) >= 5:
        return Arc(vals[0], vals[1], vals[2], vals[3], vals[4], ccw=True)
    if op == "AR" and len(vals) >= 5:
        return Arc(vals[0], vals[1], vals[2], vals[3], vals[4], ccw=False)
    return None


# ---------- stroke chaining ----------

def chain_primitives(prims: list[Primitive], tol: float = 1e-4
                     ) -> list[list[Primitive]]:
    """Group primitives into continuous polyline paths by endpoint matching.

    Each output path is a list of primitives where the end of one matches
    the start of the next. Primitives may be reversed in place to chain.
    """
    remaining = list(prims)
    paths: list[list[Primitive]] = []

    def near(a: tuple[float, float], b: tuple[float, float]) -> bool:
        return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol

    while remaining:
        # Seed path with arbitrary first primitive
        current = [remaining.pop(0)]
        extended = True
        while extended:
            extended = False
            head_start = current[0].start()
            tail_end = current[-1].end()
            for i, p in enumerate(remaining):
                ps, pe = p.start(), p.end()
                if near(pe, head_start):
                    current.insert(0, p)
                    remaining.pop(i)
                    extended = True
                    break
                if near(ps, head_start):
                    current.insert(0, p.reversed())
                    remaining.pop(i)
                    extended = True
                    break
                if near(ps, tail_end):
                    current.append(p)
                    remaining.pop(i)
                    extended = True
                    break
                if near(pe, tail_end):
                    current.append(p.reversed())
                    remaining.pop(i)
                    extended = True
                    break
        paths.append(current)
    return paths


# ---------- g-code emission ----------

@dataclass
class GCodeOpts:
    height_mm: float = 10.0
    x0: float = 0.0
    y0: float = 0.0
    feed: float = 200.0
    plunge_feed: float = 50.0
    z_safe: float = 2.0
    z_cut: float = -0.1
    return_home: bool = False
    workspace: str | None = None  # e.g. "G54".."G59"; emitted in header if set
    spindle_rpm: int = 10000      # S word value, emitted with M03


def _fmt(v: float) -> str:
    """Format a number with up to 3 decimal places, trimmed."""
    s = f"{v:.3f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def _fmtp(v: float) -> str:
    """Format a position value: up to 3 decimal places, always at least 1."""
    s = f"{v:.3f}"
    # Trim trailing zeros but leave at least one decimal digit.
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    if s == "-0.0":
        s = "0.0"
    return s


def emit_gcode(text: str, font: Font, opts: GCodeOpts) -> str:
    """Render `text` as g-code using `font`. Returns the file contents."""
    cap = font.cap_height
    scale = opts.height_mm / cap if cap > 0 else 1.0
    out: list[str] = []
    w = out.append

    w("(Generated by utils/text_to_gcode.py)")
    w(f"(Text: {text!r})")
    w(f"(Font: {font.name}, cap height: {opts.height_mm} mm, scale: {_fmt(scale)})")
    w("%")
    w("O00000001")
    w("G21")  # mm
    w("G90")  # absolute
    # w("G94")  # feed per minute
    w("G17")  # XY plane
    if opts.workspace:
        w(opts.workspace)  # workspace offset (G54..G59)
    w(f"F{_fmtp(opts.feed)}")
    w(f"S{opts.spindle_rpm}")
    w("M03")  # spindle on (CW)
    w(f"G00 Z{_fmtp(opts.z_safe)}")

    cursor_x = opts.x0
    base_y = opts.y0

    for ch in text:
        if ch == " ":
            cursor_x += font.word_spacing * scale
            continue
        if ch == "\n":
            # Simple line break: drop one cap-height + line spacing back to x0
            base_y -= opts.height_mm * 1.5
            cursor_x = opts.x0
            continue
        glyph = font.glyphs.get(ch)
        if glyph is None or not glyph.primitives:
            # Unknown char: advance like a space
            cursor_x += font.word_spacing * scale
            continue

        paths = chain_primitives(list(glyph.primitives))
        for path in paths:
            _emit_path(w, path, scale, cursor_x, base_y, opts)

        cursor_x += (glyph.width + font.letter_spacing) * scale

    w(f"G00 Z{_fmtp(opts.z_safe)}")
    if opts.return_home:
        w("G00 X0.0 Y0.0")
    w("M05")
    w("M30")
    return "\n".join(out) + "\n"


def _emit_path(write: callable, path: list[Primitive], scale: float,
               ox: float, oy: float, opts: GCodeOpts) -> None:
    """Emit g-code for one continuous path."""
    if not path:
        return
    sx, sy = path[0].start()
    write(f"G00 X{_fmtp(ox + sx * scale)} Y{_fmtp(oy + sy * scale)}")
    write(f"G01 Z{_fmtp(opts.z_cut)} F{_fmtp(opts.plunge_feed)}")
    write(f"F{_fmtp(opts.feed)}")

    for prim in path:
        if isinstance(prim, Line):
            ex, ey = prim.end()
            write(f"G01 X{_fmtp(ox + ex * scale)} Y{_fmtp(oy + ey * scale)}")
        else:
            # Arc: I and J are offsets from arc START to centre, in user units.
            start_x, start_y = prim.start()
            end_x, end_y = prim.end()
            i = (prim.cx - start_x) * scale
            j = (prim.cy - start_y) * scale
            code = "G03" if prim.ccw else "G02"
            write(
                f"{code} X{_fmtp(ox + end_x * scale)} Y{_fmtp(oy + end_y * scale)}"
                f" I{_fmtp(i)} J{_fmtp(j)}"
            )
    write(f"G00 Z{_fmtp(opts.z_safe)}")


# ---------- CLI ----------

DEFAULT_FONT = Path(__file__).resolve().parent.parent / "fonts" / "romans.cxf"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--text", required=True, help="String to render")
    p.add_argument("--out", required=True, type=Path,
                   help="Output .gcode file path")
    p.add_argument("--font", type=Path, default=DEFAULT_FONT,
                   help=f"CXF font path (default: {DEFAULT_FONT})")
    p.add_argument("--height", type=float, default=10.0,
                   help="Cap height in mm (default 10.0)")
    p.add_argument("--x0", type=float, default=0.0,
                   help="Start X in mm (default 0.0)")
    p.add_argument("--y0", type=float, default=0.0,
                   help="Start Y in mm (default 0.0)")
    p.add_argument("--feed", type=float, default=200.0,
                   help="Cutting feed mm/min (default 200)")
    p.add_argument("--plunge-feed", type=float, default=50.0,
                   help="Z plunge feed mm/min (default 50)")
    p.add_argument("--z-safe", type=float, default=2.0,
                   help="Pen-up Z height mm (default 2.0)")
    p.add_argument("--z-cut", type=float, default=-0.1,
                   help="Pen-down Z depth mm (default -0.1)")
    p.add_argument("--return-home", action="store_true",
                   help="Append G0 X0 Y0 at end of program")
    p.add_argument("--workspace", default=None,
                   choices=["G54", "G55", "G56", "G57", "G58", "G59"],
                   help="Workspace offset to select in header (e.g. G54)")
    p.add_argument("--spindle-rpm", type=int, default=10000,
                   help="Spindle speed in RPM, emitted with M03 (default 10000)")
    args = p.parse_args(argv)

    if not args.font.exists():
        print(f"error: font not found: {args.font}", file=sys.stderr)
        return 2

    font = parse_cxf(args.font)
    if not font.glyphs:
        print(f"error: no glyphs parsed from {args.font}", file=sys.stderr)
        return 2

    opts = GCodeOpts(
        height_mm=args.height,
        x0=args.x0, y0=args.y0,
        feed=args.feed, plunge_feed=args.plunge_feed,
        z_safe=args.z_safe, z_cut=args.z_cut,
        return_home=args.return_home,
        workspace=args.workspace,
        spindle_rpm=args.spindle_rpm,
    )
    gcode = emit_gcode(args.text, font, opts)
    args.out.write_text(gcode)
    print(f"wrote {args.out} ({len(gcode.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
