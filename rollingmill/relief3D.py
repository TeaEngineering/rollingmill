#!/usr/bin/env python3
"""Render a greyscale image as a 3D relief carving in single-pass g-code.

The image's brightness is treated as height: white = top of stock, black =
deepest cut. The chosen cutter's tip shape eats into a per-pixel "reachable
surface" via a morphological erosion (so a wide ball nose cannot dive into
fine detail it physically cannot reach), and the result is rastered with a
boustrophedon pattern.

This is v1: finishing pass only, single tool, no roughing and no tool
change. Use a ball-nose cutter unless you understand the cusp behaviour of
end mills on slopes.

Streamed via the existing ``rollingmill --file out.gcode`` TUI flow.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from rollingmill import tools as _tools


_RELIEF_EXTRA_HINT = (
    "rollingmill.relief3D needs numpy, pillow and scipy. Install them with:\n"
    "    pip install rollingmill[relief]\n"
    "or manually:\n"
    "    pip install numpy pillow scipy"
)


def _require_deps():
    try:
        import numpy as np
        from PIL import Image
        from scipy.ndimage import gaussian_filter, grey_dilation
    except ImportError as e:
        raise ImportError(_RELIEF_EXTRA_HINT) from e
    return np, Image, gaussian_filter, grey_dilation


@dataclass
class ReliefOpts:
    image: Path
    out: Path
    tool: str
    stock_width_mm: float
    stock_height_mm: float
    z_max: float = 0.0
    z_min: float = -3.0
    gamma: float = 1.0
    pixel_pitch_mm: float | None = None  # default: tool radius / 4
    stepover_mm: float | None = None  # default: tool diameter * 0.3
    blur_px: float = 0.5  # gaussian σ in resampled pixels
    x0: float = 0.0
    y0: float = 0.0
    z_safe: float = 2.0
    feed: float | None = None  # default: from tool, else 400
    plunge_feed: float | None = None  # default: from tool, else 80
    spindle_rpm: int | None = None  # default: from tool, else 14000
    workspace: str | None = None
    return_home: bool = False
    raster_axis: Literal["x", "y"] = "x"
    direction: Literal["zigzag", "oneway"] = "zigzag"
    invert: bool = False


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


def _resolved_pitch(opts: ReliefOpts, tool: _tools.Tool) -> float:
    if opts.pixel_pitch_mm is not None:
        return opts.pixel_pitch_mm
    return max(tool.diameter_mm / 2.0 / 4.0, 0.05)


def _resolved_stepover(opts: ReliefOpts, tool: _tools.Tool) -> float:
    if opts.stepover_mm is not None:
        return opts.stepover_mm
    return max(tool.diameter_mm * 0.3, 0.05)


def load_heightmap(opts: ReliefOpts, pitch_mm: float):
    """Image -> 2D numpy array of mill-space Z values.

    Returns an array indexed ``H[y_idx, x_idx]`` where ``y_idx = 0`` is the
    *bottom* row in mill coords (Y+ up). Z values lie in
    ``[z_min, z_max]``.
    """
    np, Image, gaussian_filter, _ = _require_deps()
    if opts.z_max <= opts.z_min:
        raise ValueError("z_max must be > z_min")
    if opts.stock_width_mm <= 0 or opts.stock_height_mm <= 0:
        raise ValueError("stock_width_mm and stock_height_mm must be > 0")
    if opts.gamma <= 0:
        raise ValueError("gamma must be > 0")

    img = Image.open(opts.image).convert("L")
    nx = max(2, int(round(opts.stock_width_mm / pitch_mm)))
    ny = max(2, int(round(opts.stock_height_mm / pitch_mm)))
    img = img.resize((nx, ny), Image.LANCZOS)

    a = np.asarray(img, dtype=float) / 255.0  # (ny, nx), row 0 = top
    if opts.blur_px > 0:
        a = gaussian_filter(a, sigma=opts.blur_px)
    a = np.clip(a, 0.0, 1.0) ** opts.gamma

    if opts.invert:
        intensity = 1.0 - a
    else:
        intensity = a

    z = opts.z_min + intensity * (opts.z_max - opts.z_min)
    # PIL row 0 is the image top; mill Y+ is up, so flip so array row 0 is
    # the bottom of the stock.
    z = z[::-1, :]
    return z


def erode_by_tool(heightmap, tool: _tools.Tool, pitch_mm: float):
    """Apply tool-tip morphology so reach(x,y) is the deepest tip Z that does
    not gouge anywhere in the tool's footprint."""
    np, _, _, grey_dilation = _require_deps()
    fp = tool.footprint(pitch_mm)
    mask = ~np.isinf(fp)
    if not mask.any():
        raise ValueError("tool footprint is empty at this pitch")
    structure = np.where(mask, -fp, 0.0)
    # reach(x,y) = max over (dx,dy) in mask of [H(x+dx, y+dy) - fp(dx,dy)]
    return grey_dilation(heightmap, footprint=mask, structure=structure)


def raster_path(reach_map, opts: ReliefOpts, pitch_mm: float, stepover_mm: float):
    """Build a single boustrophedon polyline across the reach map.

    Returns a list of ``(x_mm, y_mm, z_mm)`` tuples in mill coords (bottom-
    left origin offset by ``opts.x0, opts.y0``).
    """
    np, _, _, _ = _require_deps()
    ny, nx = reach_map.shape

    # Sample stride in pixels for the stepover; clamp to at least 1.
    row_stride_px = max(1, int(round(stepover_mm / pitch_mm)))
    row_indices = list(range(0, ny, row_stride_px))
    if row_indices[-1] != ny - 1:
        row_indices.append(ny - 1)

    xs_mm = np.arange(nx) * pitch_mm + opts.x0
    poly: list[tuple[float, float, float]] = []
    forward = True
    for r_idx, r in enumerate(row_indices):
        y_mm = opts.y0 + r * pitch_mm
        zs = reach_map[r, :]
        col_iter = range(nx) if forward else range(nx - 1, -1, -1)
        for c in col_iter:
            poly.append((float(xs_mm[c]), float(y_mm), float(zs[c])))
        if opts.direction == "zigzag":
            forward = not forward
    return poly


def emit_gcode(opts: ReliefOpts) -> str:
    """Run the full pipeline and return RS-274 g-code as a string."""
    if not opts.image.exists():
        raise FileNotFoundError(f"image not found: {opts.image}")

    tool = _tools.find(_tools.load(), opts.tool)
    pitch = _resolved_pitch(opts, tool)
    stepover = _resolved_stepover(opts, tool)

    feed = _tools.resolve(opts.feed, tool, "feed_mm_min", 400.0)
    plunge_feed = _tools.resolve(opts.plunge_feed, tool, "plunge_feed_mm_min", 80.0)
    spindle_rpm = _tools.resolve(opts.spindle_rpm, tool, "spindle_rpm", 14000)

    heightmap = load_heightmap(opts, pitch)
    reach_map = erode_by_tool(heightmap, tool, pitch)
    poly = raster_path(reach_map, opts, pitch, stepover)

    out: list[str] = []
    w = out.append

    w("(Generated by rollingmill.relief3D)")
    w(
        f"(Image: {opts.image.name}, stock: "
        f"{_fmt(opts.stock_width_mm)} x {_fmt(opts.stock_height_mm)} mm)"
    )
    w(f"(Tool: {tool.id} ({tool.type}, dia {_fmt(tool.diameter_mm)} mm))")
    w(f"(Pitch: {_fmt(pitch)} mm, stepover: {_fmt(stepover)} mm)")
    w(f"(Z range: {_fmt(opts.z_min)} .. {_fmt(opts.z_max)} mm)")
    w("%")
    w("O00000001")
    w("G21")
    w("G90")
    w("G17")
    if opts.workspace:
        w(opts.workspace)
    w(f"F{_fmtp(feed)}")
    w(f"S{spindle_rpm}")
    w("M03")
    w(f"G00 Z{_fmtp(opts.z_safe)}")

    if poly:
        x0, y0, z0 = poly[0]
        w(f"G00 X{_fmtp(x0)} Y{_fmtp(y0)}")
        w(f"G01 Z{_fmtp(z0)} F{_fmtp(plunge_feed)}")
        w(f"F{_fmtp(feed)}")
        prev = poly[0]
        for x, y, z in poly[1:]:
            parts = ["G01"]
            if x != prev[0]:
                parts.append(f"X{_fmtp(x)}")
            if y != prev[1]:
                parts.append(f"Y{_fmtp(y)}")
            if z != prev[2]:
                parts.append(f"Z{_fmtp(z)}")
            if len(parts) > 1:
                w(" ".join(parts))
            prev = (x, y, z)

    w(f"G00 Z{_fmtp(opts.z_safe)}")
    if opts.return_home:
        w("G00 X0.0 Y0.0")
    w("M05")
    w("M30")
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--image", type=Path, required=True, help="Input greyscale image (PNG/JPG/...)")
    p.add_argument("--out", type=Path, required=True, help="Output .gcode file path")
    p.add_argument("--tool", required=True, help="Tool id or name from ~/.rollingmill/tools.json")
    p.add_argument("--stock-width", type=float, required=True, help="Stock X extent in mm")
    p.add_argument("--stock-height", type=float, required=True, help="Stock Y extent in mm")
    p.add_argument(
        "--z-max", type=float, default=0.0, help="Top-of-stock Z mm (white pixel; default 0.0)"
    )
    p.add_argument(
        "--z-min", type=float, default=-3.0, help="Deepest cut Z mm (black pixel; default -3.0)"
    )
    p.add_argument(
        "--gamma", type=float, default=1.0, help="Tone curve on the heightmap (default 1.0)"
    )
    p.add_argument(
        "--pitch",
        type=float,
        default=None,
        help="Image resample pitch mm (default: tool radius / 4)",
    )
    p.add_argument(
        "--stepover",
        type=float,
        default=None,
        help="Raster row spacing mm (default: tool dia * 0.3)",
    )
    p.add_argument(
        "--blur",
        type=float,
        default=0.5,
        help="Gaussian blur sigma in pixels (default 0.5; 0 disables)",
    )
    p.add_argument("--invert", action="store_true", help="Black=top instead of black=deepest")
    p.add_argument(
        "--x0", type=float, default=0.0, help="X offset of bottom-left of stock mm (default 0)"
    )
    p.add_argument(
        "--y0", type=float, default=0.0, help="Y offset of bottom-left of stock mm (default 0)"
    )
    p.add_argument("--z-safe", type=float, default=2.0, help="Rapid Z height mm (default 2.0)")
    p.add_argument(
        "--feed",
        type=float,
        default=None,
        help="Cutting feed mm/min (default from --tool, else 400)",
    )
    p.add_argument(
        "--plunge-feed",
        type=float,
        default=None,
        help="Z plunge feed mm/min (default from --tool, else 80)",
    )
    p.add_argument(
        "--spindle-rpm",
        type=int,
        default=None,
        help="Spindle RPM (default from --tool, else 14000)",
    )
    p.add_argument(
        "--workspace",
        default=None,
        choices=["G54", "G55", "G56", "G57", "G58", "G59"],
        help="Workspace offset to select in header",
    )
    p.add_argument("--return-home", action="store_true", help="Append G00 X0 Y0 before M05")
    p.add_argument(
        "--raster-axis", choices=["x", "y"], default="x", help="Raster direction (default x)"
    )
    p.add_argument(
        "--direction",
        choices=["zigzag", "oneway"],
        default="zigzag",
        help="Boustrophedon zigzag or one-way (default zigzag)",
    )
    args = p.parse_args(argv)

    opts = ReliefOpts(
        image=args.image,
        out=args.out,
        tool=args.tool,
        stock_width_mm=args.stock_width,
        stock_height_mm=args.stock_height,
        z_max=args.z_max,
        z_min=args.z_min,
        gamma=args.gamma,
        pixel_pitch_mm=args.pitch,
        stepover_mm=args.stepover,
        blur_px=args.blur,
        invert=args.invert,
        x0=args.x0,
        y0=args.y0,
        z_safe=args.z_safe,
        feed=args.feed,
        plunge_feed=args.plunge_feed,
        spindle_rpm=args.spindle_rpm,
        workspace=args.workspace,
        return_home=args.return_home,
        raster_axis=args.raster_axis,
        direction=args.direction,
    )

    try:
        gcode = emit_gcode(opts)
    except (FileNotFoundError, KeyError, ValueError) as e:
        p.error(str(e))
    except ImportError as e:
        print(str(e), file=sys.stderr)
        return 2

    args.out.write_text(gcode)
    print(f"wrote {args.out} ({len(gcode.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
