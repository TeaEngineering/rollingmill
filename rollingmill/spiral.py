#!/usr/bin/env python3
"""Render a "neat spiral" (mathematically a hypotrochoid) to single-line g-code.

Given two gear tooth counts and a pen-arm length, this traces the curve drawn
when a small toothed wheel rolls inside a fixed toothed ring -- the classic
"spirograph" family of curves. ("Spirograph" is a registered trademark; this
module uses the descriptive name "neat spiral" throughout.)

Parametric form (units: tooth counts; one tooth == one unit):

    x(t) = (R - r) cos(t) + d cos(((R - r) / r) t)
    y(t) = (R - r) sin(t) - d sin(((R - r) / r) t)

where R is the ring tooth count, r is the rotor tooth count, and d is the
pen-arm length measured in the same tooth-count units as r.

The curve closes after t in [0, 2 pi * r / gcd(R, r)]. We scale everything so
the maximum radial extent equals the user's requested radius in mm, then emit
a single pen-down / pen-up cycle centred on (x0, y0), suitable for streaming
to the MDX-40A via `rollingmill --file <out.gcode>`.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from math import gcd
from pathlib import Path
from typing import Iterator


@dataclass
class SpiralOpts:
    radius_mm: float
    x0: float = 0.0
    y0: float = 0.0
    chord_tol_mm: float = 0.05
    feed: float = 200.0
    plunge_feed: float = 50.0
    z_safe: float = 2.0
    z_cut: float = -0.1
    return_home: bool = False
    workspace: str | None = None  # e.g. "G54".."G59"; emitted in header if set
    spindle_rpm: int = 10000


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


def _hypo_point(R: int, r: int, d: float, t: float) -> tuple[float, float]:
    """Hypotrochoid position at parameter t, in tooth-count units."""
    Rmr = R - r
    ratio = Rmr / r
    return (Rmr * math.cos(t) + d * math.cos(ratio * t),
            Rmr * math.sin(t) - d * math.sin(ratio * t))


def _hypo_tangent_unit(R: int, r: int, d: float, t: float) -> tuple[float, float]:
    """Unit tangent to the hypotrochoid at parameter t. Falls back to a
    forward-difference at cusps (only possible when d == r)."""
    Rmr = R - r
    ratio = Rmr / r
    dx = -Rmr * math.sin(t) - d * ratio * math.sin(ratio * t)
    dy = Rmr * math.cos(t) - d * ratio * math.cos(ratio * t)
    mag = math.hypot(dx, dy)
    if mag < 1e-9:
        eps = 1e-6
        p0x, p0y = _hypo_point(R, r, d, t)
        p1x, p1y = _hypo_point(R, r, d, t + eps)
        dx, dy = p1x - p0x, p1y - p0y
        mag = math.hypot(dx, dy)
        if mag < 1e-15:
            return (1.0, 0.0)
    return (dx / mag, dy / mag)


def _fit_tangent_arc(
    p0: tuple[float, float],
    t0: tuple[float, float],
    p1: tuple[float, float],
) -> tuple[float, float, float, bool] | None:
    """Fit a circular arc tangent to t0 at p0 and passing through p1.

    Returns (centre_x, centre_y, radius, cw) where cw=True means clockwise
    (G02) and cw=False means counter-clockwise (G03). Returns None for the
    collinear case (caller should emit a straight G01 line)."""
    chord_x = p1[0] - p0[0]
    chord_y = p1[1] - p0[1]
    chord_sq = chord_x * chord_x + chord_y * chord_y
    if chord_sq < 1e-24:
        return None
    # Left-hand normal of t0 (t0 rotated +90°):
    nx, ny = -t0[1], t0[0]
    # Signed perpendicular distance from the tangent line at p0 to p1.
    proj = nx * chord_x + ny * chord_y
    if abs(proj) < 1e-9 * math.sqrt(chord_sq):
        return None  # collinear
    R_signed = chord_sq / (2.0 * proj)
    cx = p0[0] + R_signed * nx
    cy = p0[1] + R_signed * ny
    # R_signed > 0  -> centre on the left of the tangent -> CCW (G03)
    # R_signed < 0  -> centre on the right               -> CW  (G02)
    return (cx, cy, abs(R_signed), R_signed < 0)


def _arc_max_deviation(
    R: int, r: int, d: float, scale: float,
    t_start: float, t_end: float,
    centre: tuple[float, float], radius: float,
    n_check: int = 5,
) -> float:
    """Max ||P(t) - centre| - radius| over t in (t_start, t_end), in mm."""
    cx, cy = centre
    max_dev = 0.0
    for i in range(1, n_check + 1):
        u = i / (n_check + 1)
        t = t_start + (t_end - t_start) * u
        px, py = _hypo_point(R, r, d, t)
        dev = abs(math.hypot(px * scale - cx, py * scale - cy) - radius)
        if dev > max_dev:
            max_dev = dev
    return max_dev


def hypotrochoid_arcs(
    R: int, r: int, d: float, scale: float, chord_tol_mm: float
) -> Iterator[tuple]:
    """Tile one closure of the hypotrochoid with circular arcs, in mm.

    First yield: ``("start", x, y)`` — initial pen-down point.
    Subsequent yields are either:

      * ``("arc", end_x, end_y, i_rel, j_rel, cw)`` — a G02/G03 arc whose
        centre lies at ``(prev_end + (i_rel, j_rel))`` and which ends at
        ``(end_x, end_y)``. ``cw=True`` selects G02, ``cw=False`` selects G03.
      * ``("line", end_x, end_y)`` — a straight G01 segment, emitted only
        for the rare collinear case where arc fitting is degenerate.

    Each arc's deviation from the true curve is bounded above by
    ``chord_tol_mm``."""
    if R <= r:
        raise ValueError(f"ring tooth count R={R} must exceed rotor r={r}")
    if r <= 0:
        raise ValueError(f"rotor tooth count r={r} must be positive")
    if d < 0:
        raise ValueError(f"pen arm d={d} must be non-negative")

    period = 2.0 * math.pi * r / gcd(R, r)

    sx0, sy0 = _hypo_point(R, r, d, 0.0)
    sx, sy = sx0 * scale, sy0 * scale
    yield ("start", sx, sy)

    t = 0.0
    cur_x, cur_y = sx, sy
    step = period / 200.0
    min_step = period / 200000.0
    max_step = period / 4.0

    while t < period - 1e-12:
        attempt = min(step, period - t)

        for _ in range(40):
            next_t = t + attempt
            # Snap to the exact period on the closing arc so floating-point
            # drift in `t` doesn't shift the endpoint off the start point.
            if abs(next_t - period) < period * 1e-12:
                next_t = period
                attempt = next_t - t
            tangent = _hypo_tangent_unit(R, r, d, t)
            p1_raw = _hypo_point(R, r, d, next_t)
            p1 = (p1_raw[0] * scale, p1_raw[1] * scale)

            arc = _fit_tangent_arc((cur_x, cur_y), tangent, p1)
            if arc is None:
                yield ("line", p1[0], p1[1])
                cur_x, cur_y = p1
                t = next_t
                step = min(attempt * 1.3, max_step)
                break

            cx, cy, radius, cw = arc
            dev = _arc_max_deviation(R, r, d, scale, t, next_t, (cx, cy), radius)

            if dev <= chord_tol_mm:
                yield ("arc", p1[0], p1[1], cx - cur_x, cy - cur_y, cw)
                cur_x, cur_y = p1
                t = next_t
                if dev < chord_tol_mm * 0.25:
                    step = min(attempt * 1.5, max_step)
                else:
                    step = attempt
                break

            if attempt <= min_step:
                # Can't shrink further; emit and advance.
                yield ("arc", p1[0], p1[1], cx - cur_x, cy - cur_y, cw)
                cur_x, cur_y = p1
                t = next_t
                step = attempt
                break

            attempt *= 0.5
        else:
            raise RuntimeError(f"arc fitting failed to converge at t={t}")


def hypotrochoid_points(
    R: int, r: int, d: float, scale: float, chord_tol_mm: float
) -> Iterator[tuple[float, float]]:
    """Yield (x, y) points along one closure of the hypotrochoid, in mm,
    centred on (0, 0). The first and last points coincide (closed loop).

    Retained for callers that want raw points (e.g. for plotting); the
    g-code emitter uses :func:`hypotrochoid_arcs` instead."""
    if R <= r:
        raise ValueError(f"ring tooth count R={R} must exceed rotor r={r}")
    if r <= 0:
        raise ValueError(f"rotor tooth count r={r} must be positive")
    if d < 0:
        raise ValueError(f"pen arm d={d} must be non-negative")

    Rmr = R - r
    ratio = Rmr / r
    period = 2.0 * math.pi * r / gcd(R, r)

    # Max tangential speed of the pen tip (tooth-units per radian of t):
    #   |v|_max <= |R - r| + d * |R - r| / r = (R - r) * (1 + d / r)
    v_max = Rmr * (1.0 + d / r)
    # Convert chord tolerance into a dt cap.
    dt_for_tol = chord_tol_mm / (scale * v_max) if (scale * v_max) > 0 else period
    # Ensure at least ~720 samples per closure so very small d still looks smooth.
    dt_floor = period / 720.0
    dt = min(dt_for_tol, dt_floor)
    n = max(720, int(math.ceil(period / dt)))

    for i in range(n + 1):  # inclusive end -> repeats first point to close
        t = period * (i / n)
        x = Rmr * math.cos(t) + d * math.cos(ratio * t)
        y = Rmr * math.sin(t) - d * math.sin(ratio * t)
        yield scale * x, scale * y


def emit_gcode(R: int, r: int, d: float, opts: SpiralOpts) -> str:
    """Render a single neat spiral as MDX-40A g-code."""
    if R <= r:
        raise ValueError(f"ring tooth count R={R} must exceed rotor r={r}")
    if r <= 0:
        raise ValueError(f"rotor tooth count r={r} must be positive")
    if d < 0:
        raise ValueError(f"pen arm d={d} must be non-negative")
    if opts.radius_mm <= 0:
        raise ValueError(f"radius_mm={opts.radius_mm} must be positive")

    R_max = (R - r) + d
    if R_max <= 0:
        raise ValueError("degenerate geometry: (R - r) + d must be > 0")
    scale = opts.radius_mm / R_max

    out: list[str] = []
    w = out.append

    w("(Generated by rollingmill.spiral -- neat spiral)")
    w(f"(Ring: {R}, Rotor: {r}, Pen: {_fmt(d)}, Radius: {_fmt(opts.radius_mm)} mm)")
    w(f"(Centre: X{_fmt(opts.x0)} Y{_fmt(opts.y0)}, scale: {_fmt(scale)} mm/tooth)")
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

    segs = list(hypotrochoid_arcs(R, r, d, scale, opts.chord_tol_mm))
    _, sx, sy = segs[0]  # ("start", x, y)
    w(f"G00 X{_fmtp(opts.x0 + sx)} Y{_fmtp(opts.y0 + sy)}")
    w(f"G01 Z{_fmtp(opts.z_cut)} F{_fmtp(opts.plunge_feed)}")
    w(f"F{_fmtp(opts.feed)}")
    for seg in segs[1:]:
        if seg[0] == "arc":
            _, ex, ey, i_rel, j_rel, cw = seg
            op = "G02" if cw else "G03"
            w(f"{op} X{_fmtp(opts.x0 + ex)} Y{_fmtp(opts.y0 + ey)} "
              f"I{_fmtp(i_rel)} J{_fmtp(j_rel)}")
        else:  # ("line", ex, ey)
            _, ex, ey = seg
            w(f"G01 X{_fmtp(opts.x0 + ex)} Y{_fmtp(opts.y0 + ey)}")
    w(f"G00 Z{_fmtp(opts.z_safe)}")

    if opts.return_home:
        w("G00 X0.0 Y0.0")
    w("M05")
    w("M30")
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--ring", type=int, required=True,
                   help="Ring (outer fixed gear) tooth count R")
    p.add_argument("--rotor", type=int, required=True,
                   help="Rotor (inner rolling gear) tooth count r")
    p.add_argument("--pen", type=float, required=True,
                   help="Pen arm length d, in the same tooth units as --rotor")
    p.add_argument("--radius", type=float, required=True,
                   help="Desired max radial extent of the pattern in mm")
    p.add_argument("--out", type=Path, required=True,
                   help="Output .gcode file path")
    p.add_argument("--x0", type=float, default=0.0,
                   help="Centre X in mm (default 0.0)")
    p.add_argument("--y0", type=float, default=0.0,
                   help="Centre Y in mm (default 0.0)")
    p.add_argument("--chord-tol", type=float, default=0.05,
                   help="Chord tolerance for sampling, mm (default 0.05)")
    p.add_argument("--feed", type=float, default=200.0,
                   help="Cutting feed mm/min (default 200)")
    p.add_argument("--plunge-feed", type=float, default=50.0,
                   help="Z plunge feed mm/min (default 50)")
    p.add_argument("--z-safe", type=float, default=2.0,
                   help="Pen-up Z height mm (default 2.0)")
    p.add_argument("--z-cut", type=float, default=-0.1,
                   help="Pen-down Z depth mm (default -0.1)")
    p.add_argument("--return-home", action="store_true",
                   help="Append G00 X0 Y0 at end of program")
    p.add_argument("--workspace", default=None,
                   choices=["G54", "G55", "G56", "G57", "G58", "G59"],
                   help="Workspace offset to select in header (e.g. G54)")
    p.add_argument("--spindle-rpm", type=int, default=10000,
                   help="Spindle speed in RPM, emitted with M03 (default 10000)")
    args = p.parse_args(argv)

    if args.ring <= args.rotor:
        p.error(f"--ring ({args.ring}) must be strictly greater than --rotor ({args.rotor})")
    if args.rotor <= 0:
        p.error(f"--rotor ({args.rotor}) must be positive")
    if args.pen < 0:
        p.error(f"--pen ({args.pen}) must be non-negative")
    if args.radius <= 0:
        p.error(f"--radius ({args.radius}) must be positive")

    opts = SpiralOpts(
        radius_mm=args.radius,
        x0=args.x0, y0=args.y0,
        chord_tol_mm=args.chord_tol,
        feed=args.feed, plunge_feed=args.plunge_feed,
        z_safe=args.z_safe, z_cut=args.z_cut,
        return_home=args.return_home,
        workspace=args.workspace,
        spindle_rpm=args.spindle_rpm,
    )
    gcode = emit_gcode(args.ring, args.rotor, args.pen, opts)
    args.out.write_text(gcode)
    print(f"wrote {args.out} ({len(gcode.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
