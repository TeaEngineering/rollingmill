#!/usr/bin/env python3
"""Rough/flatten a rectangular pocket, spiralling out from the centre in arcs.

Generates MDX-40A g-code that clears a ``--width`` x ``--height`` rectangle
down to Z=-``--depth`` with a flat end mill, in steps of ``--z-step`` mm.
The workspace origin (X0 Y0 Z0, top of stock) is the lower-left corner of the
rectangle by default, or its centre with ``--center-origin``.

Each depth level is entered by a helical ramp round the centre (no plunge) and
then spirals outwards. Every turn of
the spiral is a rounded rectangle (straight runs joined by G02/G03 quarter
arcs) that is ``--stepover`` (fraction of tool diameter) further out than the
last, growing from the rectangle's medial axis, so the cut stays an even
width along the straights. A final sharp-cornered perimeter pass squares up
the corners. The pocket is the finished size: the cutter centre stays
``tool_dia/2`` inside the rectangle.

Cuts are climb-milled (clockwise with a clockwise spindle); pass
``--conventional`` to reverse. Output suits `rollingmill --file <out.gcode>`.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from rollingmill import tools as _tools
from rollingmill.tools import resolve as _resolve

_EPS = 1e-6


@dataclass
class PocketOpts:
    width_mm: float  # pocket size along X
    height_mm: float  # pocket size along Y
    depth_mm: float  # total depth below Z0
    tool_dia_mm: float = 3.0
    stepover: float = 0.45  # radial step as a fraction of tool diameter
    z_step_mm: float = 0.5  # depth of cut per level
    center_origin: bool = False  # origin at pocket centre instead of lower-left
    x0: float = 0.0  # extra offset applied to the origin
    y0: float = 0.0
    conventional: bool = False  # conventional instead of climb milling
    feed: float = 300.0
    plunge_feed: float = 50.0
    z_safe: float = 2.0
    return_home: bool = False
    workspace: str | None = None  # e.g. "G54".."G59"; emitted in header if set
    spindle_rpm: int = 10000
    tool_label: str | None = None  # library tool id/name, noted in the header


def _fmt(v: float) -> str:
    """Format a number with up to 3 decimal places, trimmed."""
    s = f"{v:.3f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def _fmtp(v: float) -> str:
    """Format a position value: up to 3 decimal places, always at least 1."""
    s = f"{v:.3f}"
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    if s == "-0.0":
        s = "0.0"
    return s


def _radii(half_short: float, step: float) -> list[float]:
    """Offsets from the medial axis: step/2, 3*step/2, ... ending exactly at half_short.

    Starting at half a step puts the two innermost rows (either side of the
    axis) one step apart like every other pair of neighbouring rows. A turn
    that would sit within a quarter step of the last one is dropped.
    """
    out: list[float] = []
    r = step / 2.0
    while r < half_short - 0.25 * step:
        out.append(r)
        r += step
    out.append(half_short)
    return out


def emit_gcode(opts: PocketOpts) -> str:
    """Render the spiral pocketing program as MDX-40A g-code."""
    if opts.tool_dia_mm <= 0:
        raise ValueError(f"tool_dia_mm={opts.tool_dia_mm} must be positive")
    if opts.depth_mm <= 0:
        raise ValueError(f"depth_mm={opts.depth_mm} must be positive")
    if opts.z_step_mm <= 0:
        raise ValueError(f"z_step_mm={opts.z_step_mm} must be positive")
    if not (0.0 < opts.stepover <= 1.0):
        raise ValueError(f"stepover={opts.stepover} must be in (0, 1]")
    if opts.width_mm <= opts.tool_dia_mm or opts.height_mm <= opts.tool_dia_mm:
        raise ValueError(
            f"pocket {_fmt(opts.width_mm)} x {_fmt(opts.height_mm)} mm must exceed "
            f"tool_dia_mm={_fmt(opts.tool_dia_mm)} in both directions"
        )

    pitch = opts.stepover * opts.tool_dia_mm
    cx = opts.x0 + (0.0 if opts.center_origin else opts.width_mm / 2.0)
    cy = opts.y0 + (0.0 if opts.center_origin else opts.height_mm / 2.0)

    # Work in local (u, v) coords with u along the long axis, centred on the
    # pocket. Cutter-centre rectangle has half-sizes A >= B.
    swap = opts.height_mm > opts.width_mm
    A = (max(opts.width_mm, opts.height_mm) - opts.tool_dia_mm) / 2.0
    B = (min(opts.width_mm, opts.height_mm) - opts.tool_dia_mm) / 2.0
    # Climb milling in a pocket is clockwise; the local path is built CCW, so
    # mirror v. Swapping u/v is also a reflection, so track the parity.
    mirror = not opts.conventional
    ccw = (mirror + swap) % 2 == 0
    corner_cap = opts.tool_dia_mm  # keeps the gap to the sharp corner cleared

    def world(u: float, v: float) -> tuple[float, float]:
        if mirror:
            v = -v
        return (cx + v, cy + u) if swap else (cx + u, cy + v)

    arc_cmd = "G03" if ccw else "G02"
    total_levels = max(1, math.ceil(opts.depth_mm / opts.z_step_mm - _EPS))
    rhos = _radii(B, pitch)

    out: list[str] = []
    w = out.append

    def line(u: float, v: float) -> None:
        x, y = world(u, v)
        w(f"G01 X{_fmtp(x)} Y{_fmtp(y)}")

    def arc(
        u0: float, v0: float, u1: float, v1: float, uc: float, vc: float, z: float | None = None
    ) -> None:
        x0, y0 = world(u0, v0)
        x1, y1 = world(u1, v1)
        xc, yc = world(uc, vc)
        zw = "" if z is None else f" Z{_fmtp(z)}"
        w(f"{arc_cmd} X{_fmtp(x1)} Y{_fmtp(y1)}{zw} I{_fmtp(xc - x0)} J{_fmtp(yc - y0)}")

    def rounded_loop(rho: float) -> None:
        """One turn round the rounded rectangle, ending at (0, -rho).

        Arriving from the previous turn at (0, -b_prev), the first move is a
        slanted line to the start of this turn's first corner, so the tool
        keeps moving along the long axis while stepping out rather than
        stopping for a sideways jog (it degenerates to a radial step only
        where the straight is zero-length, i.e. circular turns).
        """
        a, b = (A - B) + rho, rho
        c = min(rho, corner_cap)
        s, t = a - c, b - c  # straight-run ends / arc-centre offsets
        # (start corner point, end corner point, arc centre) CCW from the bottom edge
        corners = [
            ((s, -b), (a, -t), (s, -t)),
            ((a, t), (s, b), (s, t)),
            ((-s, b), (-a, t), (-s, t)),
            ((-a, -t), (-s, -b), (-s, -t)),
        ]
        for (u0, v0), (u1, v1), (uc, vc) in corners:
            if math.hypot(u0 - cur[0], v0 - cur[1]) > _EPS:
                line(u0, v0)
            if c > _EPS:
                arc(u0, v0, u1, v1, uc, vc)
            else:
                line(u1, v1)
            cur[0], cur[1] = u1, v1
        if abs(cur[0]) > _EPS or abs(cur[1] + b) > _EPS:
            line(0.0, -b)
        cur[0], cur[1] = 0.0, -b

    cur = [0.0, 0.0]

    w("(Generated by rollingmill.pocket -- spiral-out roughing pocket)")
    tool_src = f"tool {opts.tool_label}" if opts.tool_label else "tool (no library entry)"
    w(
        f"({tool_src}: dia {_fmt(opts.tool_dia_mm)} mm, radius {_fmt(opts.tool_dia_mm / 2)} mm, "
        f"feed {_fmt(opts.feed)}, plunge {_fmt(opts.plunge_feed)} mm/min, S{opts.spindle_rpm})"
    )
    w(
        f"(Pocket: {_fmt(opts.width_mm)} x {_fmt(opts.height_mm)} mm, "
        f"depth: {_fmt(opts.depth_mm)} mm)"
    )
    w(
        f"(Centre: X{_fmt(cx)} Y{_fmt(cy)}, stepover: {_fmt(pitch)} mm "
        f"= {_fmt(opts.stepover * 100)}% of tool, {len(rhos)} turns per level)"
    )
    w(
        f"(Levels: {total_levels} x {_fmt(opts.z_step_mm)} mm to Z{_fmt(-opts.depth_mm)}, "
        f"{'conventional' if opts.conventional else 'climb'}, "
        f"helical entry r={_fmt(rhos[0])} mm)"
    )
    w("%")
    w("O00000001")
    w("G21")  # mm
    w("G90")  # absolute
    w("G17")  # XY plane
    if opts.workspace:
        w(opts.workspace)
    w(f"F{_fmtp(opts.feed)}")
    w(f"S{opts.spindle_rpm}")
    w("M03")  # spindle on (CW)
    w(f"G00 Z{_fmtp(opts.z_safe)}")

    # Each level is entered by one helical revolution (two half-arcs, pitch
    # = the level's depth increment) round the pocket centre at the first
    # spiral radius, so the spiral then starts at full depth with no plunge.
    r_h = rhos[0]
    z_prev = 0.0
    for i in range(1, total_levels + 1):
        z = -min(i * opts.z_step_mm, opts.depth_mm)
        w(f"(level {i}/{total_levels}, Z{_fmt(z)})")
        x, y = world(0.0, -r_h)
        w(f"G00 X{_fmtp(x)} Y{_fmtp(y)}")
        w(f"G01 Z{_fmtp(z_prev)} F{_fmtp(opts.plunge_feed)}")  # to previous floor
        w(f"F{_fmtp(opts.feed)}")
        arc(0.0, -r_h, 0.0, r_h, 0.0, 0.0, (z_prev + z) / 2.0)
        arc(0.0, r_h, 0.0, -r_h, 0.0, 0.0, z)
        z_prev = z
        cur[0], cur[1] = 0.0, -r_h
        for rho in rhos:
            rounded_loop(rho)
        # Sharp-cornered perimeter pass to square up the corners.
        for u, v in [(A, -B), (A, B), (-A, B), (-A, -B), (0.0, -B)]:
            line(u, v)
        w(f"G00 Z{_fmtp(opts.z_safe)}")

    if opts.return_home:
        w("G00 X0.0 Y0.0")
    w("M05")
    w("M30")
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--width", type=float, required=True, help="Pocket size along X in mm")
    p.add_argument("--height", type=float, required=True, help="Pocket size along Y in mm")
    p.add_argument(
        "--depth", type=float, required=True, help="Total depth below Z0 in mm (positive)"
    )
    p.add_argument("--out", type=Path, required=True, help="Output .gcode file path")
    p.add_argument(
        "--tool-dia",
        type=float,
        default=None,
        help="End-mill diameter mm (default 3.0, or from --tool)",
    )
    p.add_argument(
        "--stepover",
        type=float,
        default=0.45,
        help="Radial step between spiral turns as a fraction of tool diameter (default 0.45)",
    )
    p.add_argument(
        "--z-step", type=float, default=0.5, help="Depth of cut per level mm (default 0.5)"
    )
    p.add_argument(
        "--center-origin",
        action="store_true",
        help="Workspace origin is the pocket centre (default: its lower-left corner)",
    )
    p.add_argument("--x0", type=float, default=0.0, help="Origin offset in X mm (default 0.0)")
    p.add_argument("--y0", type=float, default=0.0, help="Origin offset in Y mm (default 0.0)")
    p.add_argument(
        "--conventional",
        action="store_true",
        help="Conventional instead of climb milling (reverses spiral direction)",
    )
    p.add_argument(
        "--feed", type=float, default=None, help="Cutting feed mm/min (default 300, or from --tool)"
    )
    p.add_argument(
        "--plunge-feed",
        type=float,
        default=None,
        help="Z plunge feed mm/min (default 50, or from --tool)",
    )
    p.add_argument("--z-safe", type=float, default=2.0, help="Retract Z height mm (default 2.0)")
    p.add_argument("--return-home", action="store_true", help="Append G00 X0 Y0 at end of program")
    p.add_argument(
        "--workspace",
        default=None,
        choices=["G54", "G55", "G56", "G57", "G58", "G59"],
        help="Workspace offset to select in header (e.g. G54)",
    )
    p.add_argument(
        "--spindle-rpm",
        type=int,
        default=None,
        help="Spindle speed in RPM, emitted with M03 (default 10000, or from --tool)",
    )
    p.add_argument(
        "--tool",
        default=None,
        help="Tool id or name from ~/.rollingmill/tools.json; "
        "supplies defaults for --tool-dia, --feed, "
        "--plunge-feed and --spindle-rpm if those are not set",
    )
    args = p.parse_args(argv)

    tool = None
    if args.tool is not None:
        try:
            tool = _tools.find(_tools.load(), args.tool)
        except (KeyError, ValueError) as e:
            p.error(str(e))

    opts = PocketOpts(
        width_mm=args.width,
        height_mm=args.height,
        depth_mm=args.depth,
        tool_dia_mm=_resolve(args.tool_dia, tool, "diameter_mm", 3.0),
        stepover=args.stepover,
        z_step_mm=args.z_step,
        center_origin=args.center_origin,
        x0=args.x0,
        y0=args.y0,
        conventional=args.conventional,
        feed=_resolve(args.feed, tool, "feed_mm_min", 300.0),
        plunge_feed=_resolve(args.plunge_feed, tool, "plunge_feed_mm_min", 50.0),
        z_safe=args.z_safe,
        return_home=args.return_home,
        workspace=args.workspace,
        spindle_rpm=_resolve(args.spindle_rpm, tool, "spindle_rpm", 10000),
        tool_label=None if tool is None else f"{tool.id} - {tool.name}",
    )
    try:
        gcode = emit_gcode(opts)
    except ValueError as e:
        p.error(str(e))
    args.out.write_text(gcode)
    print(f"wrote {args.out} ({len(gcode.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
