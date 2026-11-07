#!/usr/bin/env python3
"""Render a stack of turned-spindle shapes to MDX-40A rotary-axis g-code.

Designed for an MDX-40A fitted with a rotary attachment such that the
workpiece extends along the **X** axis and the **A** axis rotates around X
(the standard Roland rotary mount). Each section of the totem is a
cross-section shape stacked along X; the sum of section lengths is the
total spindle length.

Coordinate system
-----------------
- X is the longitudinal totem axis. Sections stack from ``opts.x0`` in the
  +X direction.
- Z is measured from the workpiece rotation axis: ``Z=0`` is on the axis,
  ``Z>0`` is above. The original cylindrical stock has its surface at
  ``Z = stock_radius`` and cuts descend toward the axis.
- ``Z = safe_radius`` is the absolute clearance height above the workpiece
  at which the tool can rotate the workpiece safely (i.e. ``safe_radius``
  must be ``>= stock_radius`` with a margin).
- A is in degrees. Cuts at one face / one angular slice keep the workpiece
  stationary; the workpiece is rotated only when the tool is at the safe Z.
- Y stays at 0 (on-axis) for round shapes; polygon facet clearing rasters
  Y across the face width (which is symmetric around Y=0 since the face
  is currently up).

Cut algorithm per section
-------------------------
**Round shapes** (``cylinder``, ``ellipse``, ``donut``): step A in fine
increments. At each angle theta, descend Z in passes from the stock
surface down to ``r(theta)`` (the radius at that angle for this shape),
sweeping X at each Z. Single tool path at Y=0.

**Polygon shapes** (``triangle``, ``square``, ``pentagon``, ``hex``,
``octagon``): step A in ``n`` discrete positions (one per face). At each
face position, raster the face rectangle (length = section length in X,
width = polygon side length in Y) at descending Z until Z reaches the
polygon's apothem (face center distance from axis). Z stepover uses
``depth_step``; Y stepover uses ``tool_diameter * step_over_ratio``.

The "size" parameter for each shape is the **circumscribed / maximum
radius**: the distance from the rotation axis to the polygon's corner,
or to the cylinder/ellipse outer radius. So a square with ``size=12`` has
corners at radius 12, faces at radius ``12*cos(pi/4) = 8.49``, and side
length ``12*sqrt(2) = 16.97`` mm.

Safety: every A-move is preceded by a Z retract to ``safe_radius`` and a
return to Y=0, ensuring the tool clears the (potentially partly-cut)
workpiece during rotation.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from rollingmill import tools as _tools
from rollingmill.tools import resolve as _resolve


POLYGON_SIDES: dict[str, int] = {
    "triangle": 3,
    "square": 4,
    "pentagon": 5,
    "hex": 6,
    "octagon": 8,
}
ROUND_SHAPES: set[str] = {"cylinder", "ellipse", "donut"}
EMPTY_SHAPES: set[str] = {"empty"}
SHAPE_ALIASES: dict[str, str] = {
    "cube": "square",  # user wrote "cube" in example
    "torus": "donut",
    "hexagon": "hex",
}
ALL_SHAPES: set[str] = set(POLYGON_SIDES) | ROUND_SHAPES | EMPTY_SHAPES


@dataclass
class Section:
    shape: str
    size: float  # max / circumscribed radius in mm
    length: float  # extent along X (totem axis) in mm
    x_start: float = 0.0  # set by emit_gcode based on cumulative section sums

    @property
    def x_end(self) -> float:
        return self.x_start + self.length


@dataclass
class TotemOpts:
    out: Path | None = None
    stock_radius: float = 15.0
    safe_radius: float = 20.0  # absolute Z height clearance for A-moves
    depth_step: float = 0.5  # max Z descent per pass
    angle_step_deg: float = 5.0  # A step for round shapes
    tool_diameter: float = 3.0  # for polygon raster step-over
    step_over_ratio: float = 0.5
    ellipse_aspect: float = 0.6  # semi-minor / semi-major for "ellipse"
    x0: float = 0.0
    feed: float = 200.0
    plunge_feed: float = 50.0
    a_feed: float = 1800.0  # deg/min for A-axis motion
    spindle_rpm: int = 10000
    workspace: str | None = None  # G54..G59
    return_home: bool = False


def _fmt(v: float) -> str:
    s = f"{v:.3f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def _fmtp(v: float) -> str:
    s = f"{v:.3f}"
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    if s == "-0.0":
        s = "0.0"
    return s


# ---------- shape radii ----------


def _r_cylinder(size: float, theta: float) -> float:
    return size


def _r_ellipse(size: float, theta: float, aspect: float) -> float:
    a = size
    b = size * aspect
    return (a * b) / math.hypot(b * math.cos(theta), a * math.sin(theta))


def _r_donut(size: float, theta: float, x_rel: float, length: float) -> float:
    """Bead/torus profile: r varies along X as a half-sine bulge from
    ``size/2`` at the ends to ``size`` at the centre. Theta-independent."""
    base = size * 0.5
    bulge = size - base
    return base + bulge * math.sin(math.pi * x_rel / length)


def _r_polygon(n: int, size: float, theta: float) -> float:
    """Radius from polygon centre to its boundary at angle theta, for a
    regular n-gon with circumscribed radius ``size``."""
    face_angle = 2.0 * math.pi / n
    local = ((theta + face_angle / 2.0) % face_angle) - face_angle / 2.0
    apothem = size * math.cos(math.pi / n)
    return apothem / math.cos(local)


# ---------- g-code emission helpers ----------


def _z_passes(opts: TotemOpts, z_final: float) -> list[float]:
    """List of Z depths from just below the stock surface down to z_final."""
    if z_final >= opts.stock_radius:
        return []  # no material to remove
    n_passes = max(1, math.ceil((opts.stock_radius - z_final) / opts.depth_step))
    return [
        opts.stock_radius - (i + 1) * (opts.stock_radius - z_final) / n_passes
        for i in range(n_passes)
    ]


def _retract_and_rotate(opts: TotemOpts, a_deg: float) -> list[str]:
    """Emit the safe retract + A-axis rotation sequence.

    With A rotating around X, the tool must be centred on the workpiece
    axis in the Y direction (Y=0) and lifted in Z above the workpiece
    before any A move. X is along the totem and need not change.
    """
    return [
        f"G00 Z{_fmtp(opts.safe_radius)}",
        f"G00 Y0.0",
        f"G01 A{_fmtp(a_deg)} F{_fmtp(opts.a_feed)}",
    ]


# ---------- per-shape emit ----------


def _emit_round_section(opts: TotemOpts, section: Section) -> list[str]:
    """Round shape: step A finely, sweep X back-and-forth across Z passes.

    The tool stays at Y=0 (on the rotation axis when projected to the YZ
    plane through the workpiece) and sweeps X between x_start and x_end
    along the totem length. Within one A angle, consecutive Z passes
    alternate direction and plunge in place at the previous sweep's end
    — the retract to safe_radius only happens once the innermost depth
    is reached, before the next A rotation."""
    out: list[str] = []

    n_steps = max(1, int(round(360.0 / opts.angle_step_deg)))
    is_donut = section.shape == "donut"
    x_start_abs = opts.x0 + section.x_start
    x_end_abs = opts.x0 + section.x_end

    if is_donut:
        # Switch to the XZ arc plane for the donut bulge arcs (G18). The
        # rest of the program assumes G17, so we restore it after the
        # section is emitted.
        out.append("G18")

    for i in range(n_steps):
        theta_deg = i * (360.0 / n_steps)
        theta = math.radians(theta_deg)

        out.extend(_retract_and_rotate(opts, theta_deg))

        # For donut, r varies along X. Each pass at depth z_pass is cut as
        # G02/G03 arcs in the XZ plane: an optional G01 flat lead at z_pass
        # up to where the half-sine bulge crosses z_pass (x1), a rising arc
        # to the peak (length/2, size), a falling arc to the other crossing
        # (x2, z_pass), and an optional flat lead-out. Many small polyline
        # segments overrun the Roland's block-processing rate and cause
        # jerky motion; one rising + one falling arc per pass keeps the
        # firmware fed.
        if is_donut:
            base = section.size * 0.5
            bulge = section.size - base
            z_passes = _z_passes(opts, base)
            if not z_passes:
                continue
            peak = section.size
            x_mid_abs = x_start_abs + section.length / 2
            out.append(f"G00 X{_fmtp(x_start_abs)} Y0.0")
            out.append(f"G01 Z{_fmtp(opts.stock_radius)} F{_fmtp(opts.plunge_feed)}")
            sweep_dir = 1  # +1 = left-to-right, -1 = right-to-left
            for z_pass in z_passes:
                out.append(f"G01 Z{_fmtp(z_pass)} F{_fmtp(opts.plunge_feed)}")
                out.append(f"F{_fmtp(opts.feed)}")
                if z_pass >= peak - 1e-9:
                    # Bulge peak is at or below this pass: cut a flat all
                    # the way across.
                    target_x = x_end_abs if sweep_dir > 0 else x_start_abs
                    out.append(f"G01 X{_fmtp(target_x)}")
                else:
                    # Bulge crosses z_pass at x1 (rising) and x2 (falling).
                    ratio = max(0.0, min(1.0, (z_pass - base) / bulge))
                    x1 = (section.length / math.pi) * math.asin(ratio)
                    x2 = section.length - x1
                    dx = section.length / 2 - x1
                    # The arc through (x1, z_pass), (length/2, peak),
                    # (x2, z_pass) has its centre at (length/2, zc) by
                    # symmetry.
                    zc = ((peak * peak - z_pass * z_pass) - dx * dx) / (2.0 * (peak - z_pass))
                    if sweep_dir > 0:
                        if x1 > 1e-6:
                            out.append(f"G01 X{_fmtp(x_start_abs + x1)}")
                        out.append(
                            f"G03 X{_fmtp(x_mid_abs)} Z{_fmtp(peak)} "
                            f"I{_fmtp(dx)} K{_fmtp(zc - z_pass)}"
                        )
                        out.append(
                            f"G03 X{_fmtp(x_start_abs + x2)} "
                            f"Z{_fmtp(z_pass)} "
                            f"I0 K{_fmtp(zc - peak)}"
                        )
                        if x2 < section.length - 1e-6:
                            out.append(f"G01 X{_fmtp(x_end_abs)}")
                    else:
                        if x2 < section.length - 1e-6:
                            out.append(f"G01 X{_fmtp(x_start_abs + x2)}")
                        out.append(
                            f"G02 X{_fmtp(x_mid_abs)} Z{_fmtp(peak)} "
                            f"I{_fmtp(-dx)} K{_fmtp(zc - z_pass)}"
                        )
                        out.append(
                            f"G02 X{_fmtp(x_start_abs + x1)} "
                            f"Z{_fmtp(z_pass)} "
                            f"I0 K{_fmtp(zc - peak)}"
                        )
                        if x1 > 1e-6:
                            out.append(f"G01 X{_fmtp(x_start_abs)}")
                sweep_dir *= -1
            continue

        if section.shape == "cylinder":
            r_target = _r_cylinder(section.size, theta)
        elif section.shape == "ellipse":
            r_target = _r_ellipse(section.size, theta, opts.ellipse_aspect)
        else:
            raise ValueError(f"unknown round shape: {section.shape!r}")

        z_passes = _z_passes(opts, r_target)
        if not z_passes:
            continue
        out.append(f"G00 X{_fmtp(x_start_abs)} Y0.0")
        sweep_dir = 1
        for z_pass in z_passes:
            out.append(f"G01 Z{_fmtp(z_pass)} F{_fmtp(opts.plunge_feed)}")
            out.append(f"F{_fmtp(opts.feed)}")
            target_x = x_end_abs if sweep_dir > 0 else x_start_abs
            out.append(f"G01 X{_fmtp(target_x)}")
            sweep_dir *= -1

    if is_donut:
        out.append("G17")  # restore XY arc plane

    return out


def _emit_empty_section(opts: TotemOpts, section: Section) -> list[str]:
    """No-cut spacer: leaves the stock untouched for this section's length.

    The X stacking cursor is advanced by ``section.length`` in
    :func:`emit_gcode` regardless of what this returns, so the next
    section's cuts begin further down the workpiece."""
    return []


def _emit_polygon_section(opts: TotemOpts, section: Section) -> list[str]:
    """Polygon: discrete facets; raster X/Y plane at descending Z per face.

    With the totem along X, each face rectangle when the face is "up" spans
    X in [x_start, x_end] (along the workpiece length) and Y in
    [-half_side, +half_side] (the face width perpendicular to the rotation
    axis). The raster does long sweeps along X with Y stepping between
    sweeps. Every tool-tip position is inset from the face boundary by the
    tool radius so the body just touches the boundary without overcutting.
    After each Z pass's serpentine raster, the tool traces the inset
    rectangle perimeter as a cleanup pass: a cylindrical end-mill leaves
    small arrow-shaped uncut cusps at every raster direction change, and a
    closed perimeter pass at the same Z removes them before descending."""
    out: list[str] = []
    n = POLYGON_SIDES[section.shape]
    apothem = section.size * math.cos(math.pi / n)
    side = 2.0 * section.size * math.sin(math.pi / n)
    half_side = side / 2.0
    tool_r = opts.tool_diameter / 2.0

    # Inset rectangle for the tool TIP, in the rotated face frame:
    #   X (totem length) ranges over the section's extent.
    #   Y (face width)   is symmetric around the rotation axis.
    x_min = section.x_start + tool_r
    x_max = section.x_end - tool_r
    y_min = -half_side + tool_r
    y_max = +half_side - tool_r
    if x_max - x_min < 1e-6 or y_max - y_min < 1e-6:
        raise ValueError(
            f"polygon {section.shape!r} face too small for the tool "
            f"(side={side:.3f} mm, length={section.length:.3f} mm, "
            f"tool diameter={opts.tool_diameter:.3f} mm)"
        )

    step_over = max(0.1, opts.tool_diameter * opts.step_over_ratio)

    for face_idx in range(n):
        face_angle_deg = face_idx * (360.0 / n)
        out.extend(_retract_and_rotate(opts, face_angle_deg))

        # Corner-clearing extensions in the face-width (Y) direction.
        # Adjacent polygon faces share a corner along Y=±half_side. To
        # avoid leaving an unsupported wedge of stock at that corner when
        # the second of the pair is cut, each face overhangs the inset
        # rectangle toward the next-to-be-cut face. With right-hand-rule
        # positive A around +X (CW from the operator's view at the
        # tailstock side), the next-to-be-cut face sits at +Y in the tool
        # frame at the current A position:
        #   * Face 0: extend BOTH sides (+Y pre-clears C_{0,1}; -Y
        #     pre-clears C_{N-1,0}).
        #   * Faces 1..N-2: extend +Y only (toward face_{i+1}; the -Y
        #     side, C_{i-1,i}, was already pre-cleared by face_{i-1}'s
        #     +Y extension).
        #   * Face N-1: extend BOTH sides too. Geometrically face 0's
        #     -Y body and face N-2's +Y body each land *tangent* to the
        #     original cylinder boundary at the corner C_{N-1,0} /
        #     C_{N-2,N-1} (chord_half - tool_r places the tool body
        #     exactly on the cylinder). A tangent pass has no chip load
        #     — it brushes the surface without biting in, leaving a
        #     thin residual ridge. Face N-1 with no extension also
        #     terminates its body tangent at the same line, from the
        #     same side, so no cutter ever passes the corner with
        #     engagement. Extending face N-1 makes it approach each
        #     corner from the opposite frame, severing the burr.
        # The extension distance is computed per Z pass below: at height
        # z, the cylindrical stock's half-chord in Y is sqrt(R^2 - z^2),
        # so the tool tip only needs to reach (chord_half - tool_r) for
        # the body to sweep past the stock edge at that Z. Shallower Z
        # passes (near the stock surface) need little or no extension;
        # the deepest pass at Z=apothem needs the most.
        extend_pos = True
        extend_neg = face_idx == 0 or face_idx == n - 1

        for z_pass in _z_passes(opts, apothem):
            chord_half = math.sqrt(max(0.0, opts.stock_radius**2 - z_pass**2))
            y_clear = chord_half - tool_r
            y_max_face = max(y_max, y_clear) if extend_pos else y_max
            y_min_face = min(y_min, -y_clear) if extend_neg else y_min

            n_y_steps = max(1, math.ceil((y_max_face - y_min_face) / step_over))
            y_positions = [
                y_min_face + (i / n_y_steps) * (y_max_face - y_min_face)
                for i in range(n_y_steps + 1)
            ]

            # Start of pass: rapid to first Y, plunge to z, serpentine raster
            # along X (long axis).
            out.append(f"G00 X{_fmtp(opts.x0 + x_min)} Y{_fmtp(y_positions[0])}")
            out.append(f"G01 Z{_fmtp(z_pass)} F{_fmtp(opts.plunge_feed)}")
            out.append(f"F{_fmtp(opts.feed)}")
            sweep_dir = 1  # +1 = toward x_max, -1 = toward x_min
            for yi, y in enumerate(y_positions):
                if yi > 0:
                    out.append(f"G01 Y{_fmtp(y)}")
                target_x = x_max if sweep_dir > 0 else x_min
                out.append(f"G01 X{_fmtp(opts.x0 + target_x)}")
                sweep_dir *= -1

            # Cleanup perimeter trace at the same Z, removing the arrow-
            # shaped cusps that a cylindrical tool leaves at each raster
            # reversal. Walk from the raster's end to the (x_min, y_min)
            # corner through the already-cleared region, then close the
            # rectangle.
            out.append(f"G01 X{_fmtp(opts.x0 + x_min)} Y{_fmtp(y_min)}")
            out.append(f"G01 X{_fmtp(opts.x0 + x_max)}")
            out.append(f"G01 Y{_fmtp(y_max)}")
            out.append(f"G01 X{_fmtp(opts.x0 + x_min)}")
            out.append(f"G01 Y{_fmtp(y_min)}")
            # No safe-Z retract here: the next Z pass at this same A
            # angle plunges straight from this Z to a deeper one, and
            # the next face's _retract_and_rotate retracts before the
            # A move. End-of-program emits a final safe-Z retract too.
    return out


# ---------- public API ----------


def _validate_sections(opts: TotemOpts, sections: list[Section]) -> None:
    if not sections:
        raise ValueError("at least one --shape section is required")
    if opts.stock_radius <= 0:
        raise ValueError("stock_radius must be positive")
    if opts.safe_radius < opts.stock_radius:
        raise ValueError(
            f"safe_radius ({opts.safe_radius}) must be >= stock_radius ({opts.stock_radius})"
        )
    for s in sections:
        canonical = SHAPE_ALIASES.get(s.shape, s.shape)
        if canonical not in ALL_SHAPES:
            valid = ", ".join(sorted(ALL_SHAPES | set(SHAPE_ALIASES)))
            raise ValueError(f"unknown shape {s.shape!r}; valid: {valid}")
        s.shape = canonical
        if canonical not in EMPTY_SHAPES:
            if s.size <= 0:
                raise ValueError(f"shape size must be positive, got {s.size}")
            if s.size > opts.stock_radius:
                raise ValueError(
                    f"shape {s.shape!r} size {s.size} exceeds stock_radius {opts.stock_radius}"
                )
        if s.length <= 0:
            raise ValueError(f"shape length must be positive, got {s.length}")


def emit_gcode(sections: list[Section], opts: TotemOpts) -> str:
    """Render a stacked-shape totem as MDX-40A rotary-axis g-code."""
    _validate_sections(opts, sections)

    # Compute cumulative x_start for each section.
    cursor = 0.0
    for s in sections:
        s.x_start = cursor
        cursor += s.length
    total_length = cursor

    out: list[str] = []
    w = out.append

    w("(Generated by rollingmill.totem -- rotary-axis spindle)")
    w(f"(Stock radius: {_fmt(opts.stock_radius)} mm, total length: {_fmt(total_length)} mm)")
    for s in sections:
        w(
            f"(Section: {s.shape} size={_fmt(s.size)} length={_fmt(s.length)} "
            f"@ X={_fmt(opts.x0 + s.x_start)})"
        )
    w("%")
    w("O00000001")
    w("G21")  # mm
    w("G90")  # absolute
    w("G17")  # XY plane (arc plane; irrelevant here but conventional)
    if opts.workspace:
        w(opts.workspace)
    w(f"F{_fmtp(opts.feed)}")
    w(f"S{opts.spindle_rpm}")
    w("M03")  # spindle on (CW)
    w(f"G00 Z{_fmtp(opts.safe_radius)}")
    w(f"G01 A0.0 F{_fmtp(opts.a_feed)}")  # zero A datum

    for section in sections:
        w(
            f"(--- section: {section.shape} x=[{_fmt(opts.x0 + section.x_start)}, "
            f"{_fmt(opts.x0 + section.x_end)}] ---)"
        )
        if section.shape in POLYGON_SIDES:
            out.extend(_emit_polygon_section(opts, section))
        elif section.shape in ROUND_SHAPES:
            out.extend(_emit_round_section(opts, section))
        elif section.shape in EMPTY_SHAPES:
            out.extend(_emit_empty_section(opts, section))
        else:
            raise AssertionError(f"unreachable shape: {section.shape}")

    w(f"G00 Z{_fmtp(opts.safe_radius)}")
    w("G00 Y0.0")
    if opts.return_home:
        w("G00 X0.0 Y0.0")
        w("G01 A0.0 F" + _fmtp(opts.a_feed))
    w("M05")
    w("M30")
    return "\n".join(out) + "\n"


# ---------- CLI ----------


class _ShapeAction(argparse.Action):
    """Append a Section parsed from NAME:SIZE:LENGTH (or ; or , separators)."""

    def __call__(self, parser, namespace, values, option_string=None):
        sections = getattr(namespace, self.dest, None) or []
        spec = str(values).replace(";", ":").replace(",", ":")
        parts = [p for p in spec.split(":") if p]
        name = parts[0].lower() if parts else ""
        # "empty" is a no-cut spacer with only a length, no size.
        if name in EMPTY_SHAPES:
            if len(parts) != 2:
                raise argparse.ArgumentError(
                    self,
                    f"expected {name}:LENGTH (got {values!r}); e.g. {name}:40",
                )
            try:
                length = float(parts[1])
            except ValueError as e:
                raise argparse.ArgumentError(self, f"length must be numeric: {e}")
            sections.append(Section(shape=name, size=0.0, length=length))
            setattr(namespace, self.dest, sections)
            return
        if len(parts) != 3:
            raise argparse.ArgumentError(
                self,
                f"expected NAME:SIZE:LENGTH (got {values!r}); e.g. cylinder:10:10",
            )
        try:
            size = float(parts[1])
            length = float(parts[2])
        except ValueError as e:
            raise argparse.ArgumentError(self, f"size/length must be numeric: {e}")
        sections.append(Section(shape=name, size=size, length=length))
        setattr(namespace, self.dest, sections)


def main(argv: list[str] | None = None) -> int:
    valid = sorted(ALL_SHAPES) + sorted(SHAPE_ALIASES)
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--shape",
        action=_ShapeAction,
        dest="sections",
        default=[],
        metavar="NAME:SIZE:LENGTH",
        help=(
            "Stack a section on the totem; repeatable. NAME one of: "
            + ", ".join(valid)
            + ". SIZE = max/circumscribed radius (mm). "
            "LENGTH = extent along X (mm). Order on the command line is "
            "the stacking order, starting at X=x0."
        ),
    )
    p.add_argument("--out", type=Path, required=True, help="Output .gcode file path")
    p.add_argument(
        "--workspace",
        default=None,
        choices=["G54", "G55", "G56", "G57", "G58", "G59"],
        help="Workspace offset to select (origin of the stock)",
    )
    p.add_argument(
        "--stock-radius",
        type=float,
        default=15.0,
        help="Initial cylindrical stock radius mm (default 15)",
    )
    p.add_argument(
        "--safe-radius",
        type=float,
        default=20.0,
        help="Absolute Z clearance for A-moves mm (default 20). Must be >= stock_radius.",
    )
    p.add_argument(
        "--depth-step", type=float, default=0.5, help="Maximum Z descent per pass mm (default 0.5)"
    )
    p.add_argument(
        "--angle-step",
        type=float,
        default=5.0,
        help="A-axis step for round shapes, deg (default 5)",
    )
    p.add_argument(
        "--tool",
        default=None,
        help="Tool id or name from ~/.rollingmill/tools.json; "
        "supplies defaults for --tool-diameter, --feed, "
        "--plunge-feed and --spindle-rpm if those are not set",
    )
    p.add_argument(
        "--tool-diameter",
        type=float,
        default=None,
        help="Tool diameter mm, for polygon raster step-over (default 3.0, or from --tool)",
    )
    p.add_argument(
        "--step-over",
        type=float,
        default=0.5,
        help="Y step-over as fraction of tool diameter (default 0.5 = 50%% overlap)",
    )
    p.add_argument(
        "--ellipse-aspect",
        type=float,
        default=0.6,
        help="Semi-minor / semi-major for ellipse (default 0.6)",
    )
    p.add_argument(
        "--x0", type=float, default=0.0, help="X offset for the base of the totem mm (default 0)"
    )
    p.add_argument(
        "--feed", type=float, default=None, help="Cutting feed mm/min (default 200, or from --tool)"
    )
    p.add_argument(
        "--plunge-feed",
        type=float,
        default=None,
        help="Z plunge feed mm/min (default 50, or from --tool)",
    )
    p.add_argument(
        "--a-feed", type=float, default=1800.0, help="A axis feed deg/min (default 1800)"
    )
    p.add_argument(
        "--spindle-rpm",
        type=int,
        default=None,
        help="Spindle speed RPM (default 10000, or from --tool)",
    )
    p.add_argument(
        "--return-home",
        action="store_true",
        help="Append rapid to X0 Y0 A0 at the end of the program",
    )
    args = p.parse_args(argv)

    if not args.sections:
        p.error("at least one --shape is required (e.g. --shape cylinder:10:20)")

    tool = None
    if args.tool is not None:
        try:
            tool = _tools.find(_tools.load(), args.tool)
        except (KeyError, ValueError) as e:
            p.error(str(e))

    opts = TotemOpts(
        out=args.out,
        stock_radius=args.stock_radius,
        safe_radius=args.safe_radius,
        depth_step=args.depth_step,
        angle_step_deg=args.angle_step,
        tool_diameter=_resolve(args.tool_diameter, tool, "diameter_mm", 3.0),
        step_over_ratio=args.step_over,
        ellipse_aspect=args.ellipse_aspect,
        x0=args.x0,
        feed=_resolve(args.feed, tool, "feed_mm_min", 200.0),
        plunge_feed=_resolve(args.plunge_feed, tool, "plunge_feed_mm_min", 50.0),
        a_feed=args.a_feed,
        spindle_rpm=_resolve(args.spindle_rpm, tool, "spindle_rpm", 10000),
        workspace=args.workspace,
        return_home=args.return_home,
    )
    _print_summary(opts, args.sections)
    try:
        gcode = emit_gcode(args.sections, opts)
    except ValueError as e:
        p.error(str(e))
    args.out.write_text(gcode)
    print(f"wrote {args.out} ({len(gcode.splitlines())} lines)")
    return 0


def _print_summary(opts: TotemOpts, sections: list[Section]) -> None:
    """Print the resolved TotemOpts and the stacked sections table."""
    print("TotemOpts:")
    rows = [
        ("stock_radius", f"{opts.stock_radius:.3f} mm"),
        ("safe_radius", f"{opts.safe_radius:.3f} mm"),
        ("depth_step", f"{opts.depth_step:.3f} mm"),
        ("angle_step", f"{opts.angle_step_deg:.3f} deg"),
        ("tool_diameter", f"{opts.tool_diameter:.3f} mm"),
        ("step_over", f"{opts.step_over_ratio:.3f} (frac of tool dia)"),
        ("ellipse_aspect", f"{opts.ellipse_aspect:.3f}"),
        ("x0", f"{opts.x0:.3f} mm"),
        ("feed", f"{opts.feed:.1f} mm/min"),
        ("plunge_feed", f"{opts.plunge_feed:.1f} mm/min"),
        ("a_feed", f"{opts.a_feed:.1f} deg/min"),
        ("spindle_rpm", f"{opts.spindle_rpm}"),
        ("workspace", opts.workspace or "(none)"),
        ("return_home", "yes" if opts.return_home else "no"),
    ]
    key_w = max(len(k) for k, _ in rows)
    for k, v in rows:
        print(f"  {k:<{key_w}}  {v}")

    print()
    print("Sections:")
    header = ("#", "shape", "size", "length", "x_start", "x_end")
    cursor = opts.x0
    rows2: list[tuple[str, ...]] = []
    for i, s in enumerate(sections, start=1):
        canonical = SHAPE_ALIASES.get(s.shape, s.shape)
        x_start = cursor
        x_end = cursor + s.length
        cursor = x_end
        rows2.append(
            (
                str(i),
                canonical + (f" (={s.shape})" if canonical != s.shape else ""),
                f"{s.size:.3f}",
                f"{s.length:.3f}",
                f"{x_start:.3f}",
                f"{x_end:.3f}",
            )
        )
    widths = [max(len(r[c]) for r in (header, *rows2)) for c in range(len(header))]
    fmt = "  " + "  ".join(f"{{:<{w}}}" for w in widths)
    print(fmt.format(*header))
    print(fmt.format(*("-" * w for w in widths)))
    for row in rows2:
        print(fmt.format(*row))
    print(f"  total length: {cursor - opts.x0:.3f} mm")
    print()


if __name__ == "__main__":
    sys.exit(main())
