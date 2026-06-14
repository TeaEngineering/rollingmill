#!/usr/bin/env python3
"""Render a randomly generated rectangular maze to single-line g-code.

Uses the `erbsland-maze` library to carve a maze on a rectangular grid, then
extracts wall *centerlines* (one pen stroke per wall, not the outline of the
filled wall material that erbsland-maze's SVG renderer produces) and emits
g-code suitable for streaming to the MDX-40A via the existing
`rollingmill --file <out.gcode>` TUI flow.

The emitted program uses bottom-left origin (Y+ up) like the other g-code
emitters in this package, with a single pen-down / pen-up cycle per merged
wall polyline.
"""
from __future__ import annotations

import argparse
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from erbsland_maze import (
    Direction,
    Generator,
    GeneratorSetup,
    Room,
    SvgLayout,
    SvgSetup,
    Wall,
)
from erbsland_maze.line import Line
from erbsland_maze.point import Point
from erbsland_maze.poly_line import PolyLine


ENDPOINT_MODES = ("none", "outline", "filled")


@dataclass
class MazeOpts:
    width: float = 100.0
    height: float = 100.0
    cell: float = 4.0
    wall: float = 1.7
    seed: int | None = None
    endpoints: str = "outline"          # one of ENDPOINT_MODES
    endpoint_size: float = 1.5          # mm, side length of marker square
    endpoint_fill_pitch: float = 0.4    # mm, zig-zag pitch for "filled"
    x0: float = 0.0
    y0: float = 0.0
    feed: float = 200.0
    plunge_feed: float = 50.0
    z_safe: float = 2.0
    z_cut: float = -0.1
    return_home: bool = False
    workspace: str | None = None        # e.g. "G54".."G59"; emitted in header
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


def _wall_endpoints(layout: SvgLayout, wall: Wall) -> tuple[Point, Point]:
    """Return the two corner points spanning the centerline of a wall."""
    rect = layout.get_location_rectangle(wall.location)
    match wall.direction:
        case Direction.NORTH:
            return rect.top_left, rect.top_right
        case Direction.EAST:
            return rect.top_right, rect.bottom_right
        case Direction.SOUTH:
            return rect.bottom_left, rect.bottom_right
        case Direction.WEST:
            return rect.top_left, rect.bottom_left


def _extract_centerlines(
    rooms: Iterable[Room], layout: SvgLayout
) -> list[PolyLine]:
    """Collect deduped wall centerlines and merge them into polylines.

    Rooms are sorted by location because `RoomGrid.get_all_rooms()` returns a
    `set` keyed on `id(room)` — iterating it directly would emit walls in an
    address-dependent order and the merged-polyline output would differ
    between runs even for the same seed."""
    seen: set[frozenset[Point]] = set()
    lines: list[Line] = []
    sorted_rooms = sorted(rooms, key=lambda r: (r.location.y, r.location.x))
    for room in sorted_rooms:
        for wall in room.get_walls():
            if room.is_open_connection(wall):
                continue
            p1, p2 = _wall_endpoints(layout, wall)
            key = frozenset({p1, p2})
            if key in seen:
                continue
            seen.add(key)
            lines.append(Line(p1, p2))
    polylines = PolyLine.from_merged_lines(lines)
    for polyline in polylines:
        polyline.optimize()
    return polylines


def _generate(opts: MazeOpts) -> tuple[SvgLayout, Generator]:
    """Build and run the maze generator, returning the layout + generator."""
    random.seed(opts.seed)
    svg_setup = SvgSetup(
        width=float(opts.width),
        height=float(opts.height),
        side_length=float(opts.cell),
        wall_thickness=float(opts.wall),
    )
    gen_setup = GeneratorSetup(verbose=False)
    layout = SvgLayout(svg_setup)
    gen = Generator(layout, gen_setup)
    gen.prepare_rooms()
    # Mirror the retry loop in Generator.generate_and_save without writing SVG.
    from erbsland_maze.generator_error import NoValidSolutionError
    for attempt in range(gen.setup.maximum_attempts):
        try:
            gen.generate_maze()
            if gen.setup.allow_islands:
                gen.fill_islands()
            gen.verify_maze()
            gen.connect_longest_path()
            break
        except NoValidSolutionError:
            if attempt == gen.setup.maximum_attempts - 1:
                raise
            gen.room_grid.reset_rooms_and_connections()
    return layout, gen


def _flip_y(opts: MazeOpts, y_svg: float) -> float:
    """Convert SVG (Y+ down) to mill (Y+ up) with bottom-left origin."""
    return opts.y0 + (opts.height - y_svg)


def _emit_polyline(opts: MazeOpts, pts: list[Point]) -> list[str]:
    """Emit a pen-down / draw / pen-up cycle for one polyline."""
    out: list[str] = []
    if len(pts) < 2:
        return out
    sx = opts.x0 + pts[0].x
    sy = _flip_y(opts, pts[0].y)
    out.append(f"G00 X{_fmtp(sx)} Y{_fmtp(sy)}")
    out.append(f"G01 Z{_fmtp(opts.z_cut)} F{_fmtp(opts.plunge_feed)}")
    out.append(f"F{_fmtp(opts.feed)}")
    for p in pts[1:]:
        x = opts.x0 + p.x
        y = _flip_y(opts, p.y)
        out.append(f"G01 X{_fmtp(x)} Y{_fmtp(y)}")
    out.append(f"G00 Z{_fmtp(opts.z_safe)}")
    return out


def _room_center_svg(layout: SvgLayout, room: Room) -> tuple[float, float]:
    """Return (x, y) of a room's centre, in SVG mm (Y+ down)."""
    rect = layout.get_room_rectangle(room)
    return rect.pos.x + rect.size.width / 2.0, rect.pos.y + rect.size.height / 2.0


def _emit_endpoint(opts: MazeOpts, cx_svg: float, cy_svg: float) -> list[str]:
    """Emit a marker shape centred at (cx_svg, cy_svg) in SVG mm."""
    if opts.endpoints == "none":
        return []
    half = opts.endpoint_size / 2.0
    # Marker corners in SVG mm:
    x_left = cx_svg - half
    x_right = cx_svg + half
    y_top = cy_svg - half
    y_bottom = cy_svg + half
    if opts.endpoints == "outline":
        pts = [
            Point(x_left, y_top),
            Point(x_right, y_top),
            Point(x_right, y_bottom),
            Point(x_left, y_bottom),
            Point(x_left, y_top),
        ]
        return _emit_polyline(opts, pts)
    if opts.endpoints == "filled":
        out: list[str] = []
        pitch = max(opts.endpoint_fill_pitch, 0.05)
        rows: list[list[Point]] = []
        n_rows = max(2, int(round(opts.endpoint_size / pitch)) + 1)
        for i in range(n_rows):
            t = i / (n_rows - 1)
            y = y_top + t * (y_bottom - y_top)
            if i % 2 == 0:
                rows.append([Point(x_left, y), Point(x_right, y)])
            else:
                rows.append([Point(x_right, y), Point(x_left, y)])
        # Chain into one continuous polyline by joining each row to the next.
        chained: list[Point] = []
        for i, row in enumerate(rows):
            if i == 0:
                chained.extend(row)
            else:
                # Vertical step then horizontal sweep.
                chained.append(row[0])
                chained.append(row[1])
        out.extend(_emit_polyline(opts, chained))
        return out
    raise ValueError(f"unknown endpoints mode: {opts.endpoints!r}")


def emit_gcode(opts: MazeOpts) -> str:
    """Render a maze as MDX-40A g-code."""
    if opts.width <= 0 or opts.height <= 0:
        raise ValueError("width and height must be positive")
    if opts.cell <= 0 or opts.wall <= 0:
        raise ValueError("cell and wall must be positive")
    if opts.endpoints not in ENDPOINT_MODES:
        raise ValueError(
            f"endpoints={opts.endpoints!r} must be one of {ENDPOINT_MODES}"
        )

    seed = opts.seed if opts.seed is not None else random.randrange(2**32)
    opts_resolved = MazeOpts(**{**opts.__dict__, "seed": seed})

    layout, gen = _generate(opts_resolved)
    rooms = list(gen.room_grid.get_all_rooms())
    polylines = _extract_centerlines(rooms, layout)

    out: list[str] = []
    w = out.append

    w("(Generated by rollingmill.maze)")
    w(f"(Size: {_fmt(opts.width)} x {_fmt(opts.height)} mm, "
      f"cell: {_fmt(opts.cell)} mm, wall: {_fmt(opts.wall)} mm)")
    w(f"(Seed: {seed}, endpoints: {opts.endpoints})")
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

    for polyline in polylines:
        pts = list(polyline.points)
        if polyline.is_closed and pts:
            pts = pts + [pts[0]]
        out.extend(_emit_polyline(opts_resolved, pts))

    for room in gen.path_end_rooms:
        cx, cy = _room_center_svg(layout, room)
        out.extend(_emit_endpoint(opts_resolved, cx, cy))

    w(f"G00 Z{_fmtp(opts.z_safe)}")
    if opts.return_home:
        w("G00 X0.0 Y0.0")
    w("M05")
    w("M30")
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, required=True,
                   help="Output .gcode file path")
    p.add_argument("--width", type=float, default=100.0,
                   help="Canvas width in mm (default 100)")
    p.add_argument("--height", type=float, default=100.0,
                   help="Canvas height in mm (default 100)")
    p.add_argument("--cell", type=float, default=4.0,
                   help="Cell side length in mm, including wall (default 4.0)")
    p.add_argument("--wall", type=float, default=1.7,
                   help="Wall thickness in mm (default 1.7)")
    p.add_argument("--seed", type=int, default=None,
                   help="Random seed (default: random; echoed in header)")
    p.add_argument("--endpoints", choices=ENDPOINT_MODES, default="outline",
                   help="Endpoint marker style (default outline)")
    p.add_argument("--endpoint-size", type=float, default=1.5,
                   help="Endpoint marker side length in mm (default 1.5)")
    p.add_argument("--x0", type=float, default=0.0,
                   help="X offset of bottom-left corner in mm (default 0.0)")
    p.add_argument("--y0", type=float, default=0.0,
                   help="Y offset of bottom-left corner in mm (default 0.0)")
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

    opts = MazeOpts(
        width=args.width, height=args.height,
        cell=args.cell, wall=args.wall,
        seed=args.seed,
        endpoints=args.endpoints,
        endpoint_size=args.endpoint_size,
        x0=args.x0, y0=args.y0,
        feed=args.feed, plunge_feed=args.plunge_feed,
        z_safe=args.z_safe, z_cut=args.z_cut,
        return_home=args.return_home,
        workspace=args.workspace,
        spindle_rpm=args.spindle_rpm,
    )
    gcode = emit_gcode(opts)
    args.out.write_text(gcode)
    print(f"wrote {args.out} ({len(gcode.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
