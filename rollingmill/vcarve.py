#!/usr/bin/env python3
"""Convert an SVG drawing into V-carving g-code for the MDX-40A.

This is a *general-purpose* V-carve engine that treats an SVG the way you'd
want a sign carved: it classifies every shape as either a **filled region** or
a **stroke-only path** and handles each correctly.

  * **Filled regions** (e.g. bold letterforms) are rendered to a bitmap via
    Inkscape and carved along their **medial axis**, plunging deeper where the
    shape is wider so the V-bit's rim just touches the edges -- tapering to
    sharp points at terminals.
  * **Stroke-only paths** (a logo outline, an arrow) are carved along their
    **vector centreline** at a *constant* depth fixed by the declared stroke
    width. Reading the width directly avoids the two failure modes of
    rasterising a stroke: anti-aliased edges crossing the ink threshold (which
    makes the groove too deep), and self-intersections/arrowheads where the
    filled union is wider than the line (which makes the medial axis dive).

Both share one depth rule set by the bit's half-angle ``t``. At depth ``d`` the
cone has radius ``r = d * tan(t)``, so to make the rim touch edges a clearance
``r`` away we plunge to ``d = r / tan(t)``. For a filled region ``r`` is the
local medial clearance; for a stroke of width ``w`` it is ``w / 2`` (constant).

The heavy dependencies (numpy, scipy, scikit-image, pillow, svgelements) are
optional and
imported lazily, matching :mod:`rollingmill.relief3D`. Rendering needs the
``inkscape`` CLI on ``PATH`` (or pass a pre-rendered PNG mask via ``--png``,
which is also how the tests exercise the geometry without Inkscape).

Output is single-tool g-code streamable via ``rollingmill --file out.gcode``.
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import tempfile
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path

from rollingmill import tools as _tools
from rollingmill.tools import resolve as _resolve

_VCARVE_EXTRA_HINT = (
    "rollingmill.vcarve needs numpy, scipy, scikit-image, pillow and svgelements.\n"
    "Install with:\n"
    "    pip install rollingmill[vcarve]\n"
    "or manually:\n"
    "    pip install numpy scipy scikit-image pillow svgelements"
)


def _progress(opts: VCarveOpts, msg: str) -> None:
    """Print a progress line to stdout when running as a CLI (opts.progress)."""
    if opts.progress:
        print(f"[vcarve] {msg}", flush=True)


def _require_deps():
    try:
        import numpy as np
        from PIL import Image
        from skimage.morphology import medial_axis
    except ImportError as e:
        raise SystemExit(f"{e}\n\n{_VCARVE_EXTRA_HINT}") from e
    return np, Image, medial_axis


def _require_svg():
    try:
        import svgelements
    except ImportError as e:
        raise SystemExit(f"{e}\n\n{_VCARVE_EXTRA_HINT}") from e
    return svgelements


# --- number formatting (kept consistent with rollingmill.spiral) -------------


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


@dataclass
class VCarveOpts:
    page_w_mm: float = 300.0
    page_h_mm: float = 300.0
    pitch_mm: float = 0.05  # raster resolution (mm/pixel) for the filled-region pass
    half_angle_deg: float = 30.0  # V-bit half-angle from axis (60deg incl -> 30)
    threshold: int = 200  # luminance < threshold counts as ink
    max_depth_mm: float = 3.0  # clamp the deepest cut (material/bit limit)
    min_depth_mm: float = 0.0  # raise faint grooves to at least this depth
    stepdown_mm: float = 0.6  # max depth removed per pass (roughing)
    prune_mm: float = 0.4  # drop skeleton spur branches shorter than this
    chord_tol_mm: float = 0.05  # toolpath decimation tolerance
    x0: float = 0.0
    y0: float = 0.0
    feed: float = 400.0
    plunge_feed: float = 80.0
    z_safe: float = 2.0
    spindle_rpm: int = 14000
    return_home: bool = False
    workspace: str | None = None
    inkscape_bin: str = "inkscape"
    stroke_flatten_mm: float = 0.3  # curve sampling step for stroke centrelines
    markers: bool = True  # expand marker-start/-end (arrowheads) into grooves
    arcs: bool = True  # fit circular stroke runs to G02/G03 arcs
    arc_tol_mm: float = 0.02  # max deviation when fitting an arc to a stroke run
    fill_arcs: bool = False  # opt-in: fit circular fill medial-axis runs to arcs (helix in Z)
    fill_arc_tol_mm: float = 0.06  # arc-fit tolerance for the (coarser) fill medial axis
    fill_pitch_mm: float = 0.05  # per-element raster pitch for fills (matches the old whole-page)
    preview_path: Path | None = None  # if set, also write a toolpath preview PNG
    preview_pitch_mm: float = 0.25  # preview PNG resolution (mm/px); smaller = larger image
    bit_diameter_mm: float | None = None  # caps the preview's cone footprint radius
    progress: bool = False  # print progress to stdout (set by the CLI; silent as a library)


# --- rasterisation -----------------------------------------------------------


def load_png_to_mask(png_path: Path, threshold: int):
    """Load a PNG and threshold it to a boolean ink mask (True = carve)."""
    np, Image, _ = _require_deps()
    img = Image.open(png_path).convert("L")
    arr = np.asarray(img)
    return arr < threshold


# --- skeleton graph tracing --------------------------------------------------

_NBR8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _trace_skeleton(skel) -> list[list[tuple[int, int]]]:
    """Trace a 1-pixel skeleton into ordered polylines of (row, col) pixels.

    Edges run between *nodes* (skeleton pixels whose 8-neighbour degree != 2:
    endpoints and junctions). Pure loops (rings of degree-2 pixels, e.g. the
    bowl of an 'O') are emitted as closed polylines.
    """
    import numpy as np

    H, W = skel.shape
    rows, cols = np.nonzero(skel)
    pts = set(zip(rows.tolist(), cols.tolist(), strict=True))

    nbrs: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for r, c in pts:
        ns = []
        for dr, dc in _NBR8:
            p = (r + dr, c + dc)
            if p in pts:
                ns.append(p)
        nbrs[(r, c)] = ns

    def deg(p: tuple[int, int]) -> int:
        return len(nbrs[p])

    visited: set[tuple[tuple[int, int], tuple[int, int]]] = set()

    def key(a, b):
        return (a, b) if a <= b else (b, a)

    def walk(start, first) -> list[tuple[int, int]]:
        path = [start, first]
        visited.add(key(start, first))
        prev, cur = start, first
        while deg(cur) == 2:
            nxt = [p for p in nbrs[cur] if p != prev]
            if not nxt:
                break
            nn = nxt[0]
            k = key(cur, nn)
            if k in visited:
                break
            visited.add(k)
            prev, cur = cur, nn
            path.append(cur)
        return path

    polylines: list[list[tuple[int, int]]] = []

    # Branches anchored on nodes (endpoints/junctions).
    nodes = [p for p in pts if deg(p) != 2]
    for n in nodes:
        for nb in list(nbrs[n]):
            if key(n, nb) in visited:
                continue
            polylines.append(walk(n, nb))

    # Remaining pure loops (all degree-2, untouched by the node pass).
    for p in pts:
        if deg(p) != 2:
            continue
        for nb in nbrs[p]:
            if key(p, nb) in visited:
                continue
            path = walk(p, nb)
            if path[-1] != p:  # close the ring
                path.append(p)
            polylines.append(path)
            break

    return polylines


def _prune_spurs(
    polylines: list[list[tuple[int, int]]],
    deg: dict[tuple[int, int], int],
    min_len_px: float,
) -> list[list[tuple[int, int]]]:
    """Drop short whisker branches (one free end) that only add noise."""
    out = []
    for pl in polylines:
        a, b = pl[0], pl[-1]
        is_spur = (deg.get(a, 0) == 1) != (deg.get(b, 0) == 1)  # exactly one endpoint
        if is_spur:
            length = sum(
                math.hypot(pl[i + 1][0] - pl[i][0], pl[i + 1][1] - pl[i][1])
                for i in range(len(pl) - 1)
            )
            if length < min_len_px:
                continue
        out.append(pl)
    return out


# --- geometry ----------------------------------------------------------------


def _depth_from_clearance(r_mm: float, opts: VCarveOpts) -> float:
    """Depth (mm, positive) to make a V-bit rim touch edges ``r_mm`` away."""
    tan_t = math.tan(math.radians(opts.half_angle_deg))
    d = r_mm / tan_t
    if d > opts.max_depth_mm:
        d = opts.max_depth_mm
    return d


def _rdp(points: list[tuple[float, float, float]], tol: float) -> list[tuple[float, float, float]]:
    """Ramer-Douglas-Peucker on (x, y, z) polylines (Euclidean in 3D)."""
    if len(points) < 3:
        return points
    a = points[0]
    b = points[-1]
    ax, ay, az = a
    bx, by, bz = b
    abx, aby, abz = bx - ax, by - ay, bz - az
    ab2 = abx * abx + aby * aby + abz * abz
    dmax = 0.0
    idx = 0
    for i in range(1, len(points) - 1):
        px, py, pz = points[i]
        if ab2 == 0.0:
            d = math.dist((px, py, pz), (ax, ay, az))
        else:
            t = ((px - ax) * abx + (py - ay) * aby + (pz - az) * abz) / ab2
            t = max(0.0, min(1.0, t))
            proj = (ax + t * abx, ay + t * aby, az + t * abz)
            d = math.dist((px, py, pz), proj)
        if d > dmax:
            dmax = d
            idx = i
    if dmax <= tol:
        return [a, b]
    left = _rdp(points[: idx + 1], tol)
    right = _rdp(points[idx:], tol)
    return left[:-1] + right


@dataclass
class ToolPath:
    """One connected carve: (X, Y, target_depth) vertices, optionally with arcs.

    ``arcs`` maps a segment index ``i`` to ``(cx, cy, cw)``, meaning the move
    from ``pts[i]`` to ``pts[i+1]`` is a circular arc about centre ``(cx, cy)``
    (``cw=True`` -> G02, else G03) rather than a straight line. Segments absent
    from the map are straight. Fill/spur paths leave ``arcs`` empty.
    """

    pts: list[tuple[float, float, float]]  # X_mm, Y_mm, depth_mm (>=0)
    arcs: dict[int, tuple[float, float, bool]] = field(default_factory=dict)

    @property
    def max_depth(self) -> float:
        return max((p[2] for p in self.pts), default=0.0)

    def reversed(self) -> ToolPath:
        """Reverse traversal, remapping arc segment indices and flipping cw."""
        n = len(self.pts)
        rpts = list(reversed(self.pts))
        rarcs = {n - 2 - i: (cx, cy, not cw) for i, (cx, cy, cw) in self.arcs.items()}
        return ToolPath(rpts, rarcs)


def _polylines_to_toolpaths(polylines, dist, ox_mm, oy_mm, pitch, opts: VCarveOpts):
    """Convert medial (row, col) polylines to machine-space ToolPaths.

    Pixel (r, c) maps to SVG-mm ``(ox + (c+.5)*pitch, oy + (r+.5)*pitch)``; Y is
    then flipped to the bed. Depth = clearance/tan(t). The centreline is
    RDP-decimated at ``chord_tol``. When ``--fill-arcs`` is enabled, circular
    runs are additionally fitted to arcs (helical where depth ramps)."""
    out: list[ToolPath] = []
    for pl in polylines:
        raw: list[tuple[float, float, float]] = []
        for r, c in pl:
            x_svg = ox_mm + (c + 0.5) * pitch
            y_svg = oy_mm + (r + 0.5) * pitch
            depth = _depth_from_clearance(float(dist[r, c]) * pitch, opts)
            X = opts.x0 + x_svg
            Y = opts.y0 + (opts.page_h_mm - y_svg)  # flip SVG-down to bed-up
            raw.append((X, Y, depth))
        if opts.min_depth_mm > 0.0 and raw and max(p[2] for p in raw) < opts.min_depth_mm:
            raw = [(x, y, opts.min_depth_mm) for (x, y, _z) in raw]

        verts = _rdp(raw, opts.chord_tol_mm)
        arcs: dict[int, tuple[float, float, bool]] = {}
        if opts.fill_arcs and len(verts) >= 3:  # opt-in circular/helical arc fitting
            xy = [(x, y) for (x, y, _z) in verts]
            vidx, arcs = _fit_arcs(xy, opts.fill_arc_tol_mm, opts.chord_tol_mm)
            verts = [verts[i] for i in vidx]
        if len(verts) >= 2 or (len(verts) == 1 and verts[0][2] > 0):
            out.append(ToolPath(verts, arcs))
    return out


def _mask_medial_polylines(mask, opts: VCarveOpts, pitch: float):
    """Medial axis of ``mask`` -> spur-pruned (row, col) polylines + dist array."""
    np, _, medial_axis = _require_deps()
    skel, dist = medial_axis(mask, return_distance=True)
    polylines = _trace_skeleton(skel)
    rows, cols = np.nonzero(skel)
    pts = set(zip(rows.tolist(), cols.tolist(), strict=True))
    degmap = {
        (r, c): sum(1 for dr, dc in _NBR8 if (r + dr, c + dc) in pts) for r, c in pts
    }
    polylines = _prune_spurs(polylines, degmap, opts.prune_mm / pitch)
    return polylines, dist


def build_toolpaths(mask, opts: VCarveOpts) -> list[ToolPath]:
    """Whole-mask medial-axis carve (used for the ``--png`` path)."""
    polylines, dist = _mask_medial_polylines(mask, opts, opts.pitch_mm)
    return _polylines_to_toolpaths(polylines, dist, 0.0, 0.0, opts.pitch_mm, opts)


def _render_element_mask(svg_path: Path, elem_id: str, opts: VCarveOpts):
    """Render one filled SVG element to an ink mask at ``fill_pitch_mm``.

    Uses ``--export-id --export-id-only`` so only that object's own bbox is
    rasterised, keeping high DPI cheap. Returns the boolean mask."""
    np, Image, _ = _require_deps()
    exe = shutil.which(opts.inkscape_bin)
    if exe is None:
        raise SystemExit(
            f"could not find the {opts.inkscape_bin!r} executable on PATH.\n"
            "Install Inkscape, or pre-render the page and pass --png <mask.png>."
        )
    dpi = 25.4 / opts.fill_pitch_mm
    with tempfile.TemporaryDirectory() as td:
        png = Path(td) / "e.png"
        cmd = [
            exe,
            "--export-type=png",
            f"--export-id={elem_id}",
            "--export-id-only",
            "--export-background=white",
            "--export-background-opacity=1",
            f"--export-dpi={dpi}",
            f"--export-filename={png}",
            str(svg_path),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not png.exists():
            raise SystemExit(
                f"inkscape failed to render element {elem_id!r}:\n"
                + (proc.stderr or proc.stdout or "(no output)")
            )
        return np.asarray(Image.open(png).convert("L")) < opts.threshold


def build_fill_toolpaths(svg_path: Path, fill_elems, opts: VCarveOpts) -> list[ToolPath]:
    """Per-element high-DPI fill carve: render each filled element on its own
    bbox, medial-axis it, and map back to page millimetres.

    The element bbox is tight, so ink often touches the raster border (e.g. a
    text line's stems span its full height). ``medial_axis``/the distance
    transform have no background beyond the edge there, which inflates the
    clearance -> depth along border-touching strokes. Padding the mask with a
    background margin gives every edge a true exterior, fixing the medial axis."""
    np, _, _ = _require_deps()
    out: list[ToolPath] = []
    n = len(fill_elems)
    for k, (elem_id, bbox) in enumerate(fill_elems, 1):
        if not elem_id:
            continue
        x0, y0, x1, y1 = bbox
        t0 = time.time()
        _progress(opts, f"fill {k}/{n} {elem_id!r} ({x1 - x0:.0f}x{y1 - y0:.0f}mm @{opts.fill_pitch_mm}mm): rendering + medial axis...")
        mask = _render_element_mask(svg_path, elem_id, opts)
        h, w = mask.shape
        pitch = ((x1 - x0) / w + (y1 - y0) / h) / 2.0  # actual mm/px from render (pre-pad)
        pad = max(2, math.ceil(0.5 / pitch))  # ~0.5 mm background border
        mask = np.pad(mask, pad, mode="constant", constant_values=False)
        polylines, dist = _mask_medial_polylines(mask, opts, pitch)
        # padded pixel (0,0) sits `pad` px left/above the element's bbox origin
        tps = _polylines_to_toolpaths(polylines, dist, x0 - pad * pitch, y0 - pad * pitch, pitch, opts)
        out += tps
        _progress(opts, f"fill {k}/{n} {elem_id!r}: {len(tps)} toolpaths ({time.time() - t0:.1f}s)")
    return out


def _order_paths(paths: list[ToolPath]) -> list[ToolPath]:
    """Greedy nearest-neighbour ordering to shorten rapid travel."""
    if not paths:
        return []
    remaining = list(paths)
    ordered = [remaining.pop(0)]
    while remaining:
        cx, cy, _ = ordered[-1].pts[-1]
        best_i = 0
        best_d = float("inf")
        best_rev = False
        for i, p in enumerate(remaining):
            sx, sy, _ = p.pts[0]
            ex, ey, _ = p.pts[-1]
            ds = (sx - cx) ** 2 + (sy - cy) ** 2
            de = (ex - cx) ** 2 + (ey - cy) ** 2
            if ds < best_d:
                best_d, best_i, best_rev = ds, i, False
            if de < best_d:
                best_d, best_i, best_rev = de, i, True
        nxt = remaining.pop(best_i)
        if best_rev:
            nxt = nxt.reversed()
        ordered.append(nxt)
    return ordered


def _passes_for(
    path: ToolPath, opts: VCarveOpts
) -> Iterator[tuple[list[tuple[float, float, float]], dict[int, tuple[float, float, bool]]]]:
    """Yield serpentine depth passes as ``(points, arcs)``.

    Deep grooves are reached in ``stepdown`` increments; alternate passes run
    in opposite directions so the tool steps deeper at the shared end rather
    than lifting and repositioning. ``arcs`` maps segment index -> (cx, cy, cw)
    in the current traversal direction.
    """
    dmax = path.max_depth
    n = max(1, math.ceil(dmax / opts.stepdown_mm - 1e-9)) if dmax > 0 else 1
    m = len(path.pts)
    for k in range(n):
        floor = (k + 1) * opts.stepdown_mm
        if k % 2 == 0:
            seq = [(x, y, -min(depth, floor)) for (x, y, depth) in path.pts]
            arcs = path.arcs
        else:
            seq = [(x, y, -min(depth, floor)) for (x, y, depth) in reversed(path.pts)]
            arcs = {m - 2 - i: (cx, cy, not cw) for i, (cx, cy, cw) in path.arcs.items()}
        yield seq, arcs


def emit_gcode(paths: list[ToolPath], opts: VCarveOpts) -> str:
    ordered = _order_paths(paths)
    out: list[str] = []
    w = out.append

    total_depth = max((p.max_depth for p in ordered), default=0.0)
    w("(Generated by rollingmill.vcarve -- medial-axis V-carve)")
    w(
        f"(V-bit half-angle: {_fmt(opts.half_angle_deg)} deg, pitch: {_fmt(opts.pitch_mm)} mm, "
        f"max depth used: {_fmt(total_depth)} mm)"
    )
    w(f"(Paths: {len(ordered)}, stepdown: {_fmt(opts.stepdown_mm)} mm)")
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

    cur_feed = "cut"  # track so we only emit F when the regime changes
    for path in ordered:
        first = True
        for seg, arcs in _passes_for(path, opts):
            sx, sy, sz = seg[0]
            if first:
                w(f"G00 X{_fmtp(sx)} Y{_fmtp(sy)}")
                w(f"G01 Z{_fmtp(sz)} F{_fmtp(opts.plunge_feed)}")
                w(f"F{_fmtp(opts.feed)}")
                cur_feed = "cut"
                first = False
                prev = (sx, sy, sz)
            else:
                # Continuing at the shared end: step Z deeper in place.
                if abs(sz - prev[2]) > 1e-6:
                    w(f"G01 Z{_fmtp(sz)} F{_fmtp(opts.plunge_feed)}")
                    cur_feed = "plunge"
                prev = (sx, sy, sz)
            for i in range(len(seg) - 1):
                x, y, z = seg[i + 1]
                arc = arcs.get(i)
                if arc is not None:  # circular arc; helical (with Z) if depth ramps
                    cx, cy, cw = arc
                    op = "G02" if cw else "G03"
                    irel, jrel = cx - seg[i][0], cy - seg[i][1]
                    zword = f" Z{_fmtp(z)}" if abs(z - seg[i][2]) > 1e-6 else ""
                    feed = f" F{_fmtp(opts.feed)}" if cur_feed != "cut" else ""
                    w(f"{op} X{_fmtp(x)} Y{_fmtp(y)}{zword} I{_fmtp(irel)} J{_fmtp(jrel)}{feed}")
                    cur_feed = "cut"
                elif cur_feed != "cut":
                    w(f"G01 X{_fmtp(x)} Y{_fmtp(y)} Z{_fmtp(z)} F{_fmtp(opts.feed)}")
                    cur_feed = "cut"
                else:
                    w(f"G01 X{_fmtp(x)} Y{_fmtp(y)} Z{_fmtp(z)}")
                prev = (x, y, z)
        w(f"G00 Z{_fmtp(opts.z_safe)}")

    if opts.return_home:
        w("G00 X0.0 Y0.0")
    w("M05")
    w("M30")
    return "\n".join(out) + "\n"


# --- SVG classification: filled regions vs stroke-only paths ----------------

# svgelements is parsed at ppi=25.4 so that a "300mm" page with a 0..300
# viewBox yields coordinates and stroke widths directly in millimetres.
_PPI_MM = 25.4

# SVG namespaces we round-trip through ElementTree without prefix mangling.
_SVG_NS = {
    "": "http://www.w3.org/2000/svg",
    "svg": "http://www.w3.org/2000/svg",
    "inkscape": "http://www.inkscape.org/namespaces/inkscape",
    "sodipodi": "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd",
    "xlink": "http://www.w3.org/1999/xlink",
}


@dataclass
class StrokeSpec:
    """A stroke-only feature: mm subpaths (SVG y-down) and its paint style."""

    subpaths: list[list[tuple[float, float]]]
    width_mm: float
    linejoin: str = "miter"  # miter | bevel | round
    linecap: str = "butt"  # butt | square | round
    miterlimit: float = 4.0
    marker_start: str | None = None
    marker_end: str | None = None
    is_marker: bool = False  # geometry expanded from a marker (no arc-fitting)
    # Per-subpath, per-point flag: did this point come from a curved (bezier/arc)
    # segment? Only True runs are arc-fit candidates. Empty = treat all as curve.
    subpath_curves: list[list[bool]] = field(default_factory=list)


def _svg_shapes(svg_path: Path):
    """Yield rendered (non-defs) shape elements from ``svg_path``, in mm."""
    svgelements = _require_svg()
    svg = svgelements.SVG.parse(str(svg_path), reify=True, ppi=_PPI_MM)
    for e in svg.elements():
        if isinstance(e, svgelements.Shape) and not isinstance(
            e, (svgelements.SVG, svgelements.Group)
        ):
            yield svgelements, e


def _classify(e) -> str | None:
    """'fill' if the shape has a fill, else 'stroke' if it has a visible
    stroke, else None (invisible)."""
    fill = getattr(e, "fill", None)
    has_fill = fill is not None and fill.value is not None
    stroke = getattr(e, "stroke", None)
    try:
        width = float(e.implicit_stroke_width or 0.0)
    except (TypeError, ValueError):
        width = 0.0
    has_stroke = stroke is not None and stroke.value is not None and width > 0.0
    if has_fill:
        return "fill"
    if has_stroke:
        return "stroke"
    return None


_CURVE_SEGS = ("CubicBezier", "QuadraticBezier", "Arc")


def _flatten(shape, step_mm: float) -> list[tuple[list[tuple[float, float]], list[bool]]]:
    """Flatten a path/shape into subpaths, sampling curves by length.

    Returns ``(points, curve_flags)`` per subpath; ``curve_flags[i]`` is True
    when point ``i`` came from a curved segment (bezier/arc). Straight (line)
    points are False, so only bezier/arc-origin runs are arc-fit candidates."""
    subpaths: list[tuple[list[tuple[float, float]], list[bool]]] = []
    cur: list[tuple[float, float]] = []
    curf: list[bool] = []

    def close():
        if len(cur) >= 2:
            subpaths.append((cur[:], curf[:]))

    for seg in shape.segments():
        name = type(seg).__name__
        if name == "Move":
            close()
            p = seg.end
            cur, curf = [(float(p.x), float(p.y))], [False]
        elif name == "Close":
            if cur:
                cur.append(cur[0])
                curf.append(False)
                close()
            cur, curf = [], []
        else:  # Line / CubicBezier / QuadraticBezier / Arc
            is_curve = name in _CURVE_SEGS
            try:
                length = float(seg.length(error=1e-4))
            except TypeError:
                length = float(seg.length())
            n = max(1, math.ceil(length / step_mm))
            for i in range(1, n + 1):
                p = seg.point(i / n)
                cur.append((float(p.x), float(p.y)))
                curf.append(is_curve)
    close()
    return subpaths


# A plain V-bit centreline groove rounds off convex corners (by w/2) and line
# ends (to a semicircle). To recover the sharp features the stroke's join/cap
# style implies, we add short *spurs* that ramp the depth to zero at the corner
# -- the tool rides up out of the material exactly at the point. This is the
# analytic medial axis of the stroke outline at each join/cap.

_CORNER_ANGLE_RAD = math.radians(18.0)  # min turn to count a flattened vertex as a corner


def _unit(ax: float, ay: float) -> tuple[float, float]:
    m = math.hypot(ax, ay)
    return (0.0, 0.0) if m < 1e-12 else (ax / m, ay / m)


def _is_closed(sp: list[tuple[float, float]]) -> bool:
    return len(sp) >= 3 and math.dist(sp[0], sp[-1]) < 1e-6


def _turn(a, p, b) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Return (dir_in, dir_out, turn angle) at vertex p between a->p->b."""
    din = _unit(p[0] - a[0], p[1] - a[1])
    dout = _unit(b[0] - p[0], b[1] - p[1])
    if din == (0.0, 0.0) or dout == (0.0, 0.0):
        return din, dout, 0.0
    dot = max(-1.0, min(1.0, din[0] * dout[0] + din[1] * dout[1]))
    return din, dout, math.acos(dot)


def _corner_indices(sp, closed: bool) -> list[int]:
    n = len(sp)
    idx = []
    if closed:
        m = n - 1  # last vertex duplicates the first
        for i in range(m):
            _, _, turn = _turn(sp[(i - 1) % m], sp[i], sp[(i + 1) % m])
            if turn > _CORNER_ANGLE_RAD:
                idx.append(i)
    else:
        for i in range(1, n - 1):
            _, _, turn = _turn(sp[i - 1], sp[i], sp[i + 1])
            if turn > _CORNER_ANGLE_RAD:
                idx.append(i)
    return idx


def _join_spurs(sp, i, closed, r, depth, spec: StrokeSpec):
    """Tapering spur(s) (in SVG-mm (x, y, depth)) that sharpen a join at sp[i]."""
    if spec.linejoin == "round":
        return []
    m = len(sp) - 1 if closed else len(sp)
    p = sp[i]
    a = sp[(i - 1) % m] if closed else sp[i - 1]
    b = sp[(i + 1) % m] if closed else sp[i + 1]
    din, dout, turn = _turn(a, p, b)
    if turn <= _CORNER_ANGLE_RAD:
        return []
    mdir = _unit(din[0] - dout[0], din[1] - dout[1])  # outer bisector (toward miter tip)
    if mdir == (0.0, 0.0):
        return []
    sin_half = math.cos(turn / 2.0)  # sin(interior/2), interior = pi - turn
    if sin_half < 1e-6:
        return []
    bevel = spec.linejoin == "bevel" or (1.0 / sin_half) > spec.miterlimit
    if not bevel:
        M = r / sin_half
        tip = (p[0] + mdir[0] * M, p[1] + mdir[1] * M)
        return [[(p[0], p[1], depth), (tip[0], tip[1], 0.0)]]

    def outer_normal(d):
        c1, c2 = (-d[1], d[0]), (d[1], -d[0])
        return c1 if (c1[0] * mdir[0] + c1[1] * mdir[1]) >= 0 else c2

    spurs = []
    for d in (din, dout):
        nrm = outer_normal(d)
        c = (p[0] + nrm[0] * r, p[1] + nrm[1] * r)
        spurs.append([(p[0], p[1], depth), (c[0], c[1], 0.0)])
    return spurs


def _cap_spurs(sp, r, depth, linecap, at_start: bool):
    """Spurs that square a butt/square line end (round caps need none)."""
    if linecap == "round":
        return []
    if at_start:
        e, nxt = sp[0], sp[1]
        into = _unit(nxt[0] - e[0], nxt[1] - e[1])
    else:
        e, prv = sp[-1], sp[-2]
        into = _unit(prv[0] - e[0], prv[1] - e[1])
    if into == (0.0, 0.0):
        return []
    out_dir = (-into[0], -into[1])
    perp = (-into[1], into[0])
    if linecap == "square":
        base = e
        off = (out_dir[0] * r, out_dir[1] * r)
    else:  # butt: base pulled r into the path, corners at the endpoint
        base = (e[0] + into[0] * r, e[1] + into[1] * r)
        off = (0.0, 0.0)
    c1 = (e[0] + off[0] + perp[0] * r, e[1] + off[1] + perp[1] * r)
    c2 = (e[0] + off[0] - perp[0] * r, e[1] + off[1] - perp[1] * r)
    return [
        [(base[0], base[1], depth), (c1[0], c1[1], 0.0)],
        [(base[0], base[1], depth), (c2[0], c2[1], 0.0)],
    ]


def _trim_polyline(sp, t_start: float, t_end: float, flags=None):
    """Shorten a polyline by arc length ``t_start``/``t_end`` from each end.

    If ``flags`` (per-point bool) is given, trims it in parallel and returns
    ``(pts, flags)``; interpolated end points are marked non-curve (False)."""
    def trim_front(pts, fl, t):
        if t <= 0.0:
            return pts, fl
        acc = 0.0
        for i in range(len(pts) - 1):
            seg = math.dist(pts[i], pts[i + 1])
            if acc + seg >= t:
                f = (t - acc) / seg if seg > 0 else 0.0
                nx = pts[i][0] + (pts[i + 1][0] - pts[i][0]) * f
                ny = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * f
                return [(nx, ny)] + pts[i + 1 :], ([False] + fl[i + 1 :] if fl is not None else None)
            acc += seg
        return pts[-1:], (fl[-1:] if fl is not None else None)

    pts, fl = trim_front(list(sp), list(flags) if flags is not None else None, t_start)
    rpts, rfl = trim_front(list(reversed(pts)), list(reversed(fl)) if fl is not None else None, t_end)
    pts, fl = list(reversed(rpts)), (list(reversed(rfl)) if rfl is not None else None)
    return (pts, fl) if flags is not None else pts


# --- circular-arc fitting for stroke centrelines ----------------------------
# Straight moves stream as G01; a run of points that lies on a circle becomes a
# single G02/G03, which the mill cuts faster and more accurately than a chain
# of chords. Fitting is done in machine coords (after the Y-flip) so the arc
# direction (cw) is already correct for output.


def _solve3(A, b):
    """Solve a 3x3 linear system by Cramer's rule; None if near-singular."""
    def det3(m):
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    d = det3(A)
    if abs(d) < 1e-12:
        return None
    out = []
    for c in range(3):
        M = [row[:] for row in A]
        for ri in range(3):
            M[ri][c] = b[ri]
        out.append(det3(M) / d)
    return out[0], out[1], out[2]


def _circle_fit(pts):
    """Algebraic (Kasa) circle fit -> (cx, cy, r), or None if degenerate."""
    if len(pts) < 3:
        return None
    Sx = Sy = Sxx = Syy = Sxy = Sxxx = Syyy = Sxyy = Sxxy = 0.0
    for x, y in pts:
        Sx += x
        Sy += y
        Sxx += x * x
        Syy += y * y
        Sxy += x * y
        Sxxx += x * x * x
        Syyy += y * y * y
        Sxyy += x * y * y
        Sxxy += x * x * y
    n = len(pts)
    A = [[Sxx, Sxy, Sx], [Sxy, Syy, Sy], [Sx, Sy, float(n)]]
    b = [-(Sxxx + Sxyy), -(Sxxy + Syyy), -(Sxx + Syy)]
    sol = _solve3(A, b)
    if sol is None:
        return None
    D, E, F = sol
    cx, cy = -D / 2.0, -E / 2.0
    r2 = cx * cx + cy * cy - F
    if r2 <= 0.0:
        return None
    return cx, cy, math.sqrt(r2)


def _arc_sweep(pts, cx, cy) -> float:
    """Signed total angle (radians) swept about (cx, cy) along ``pts``."""
    ang = [math.atan2(py - cy, px - cx) for px, py in pts]
    total = 0.0
    for a, b in zip(ang, ang[1:], strict=False):
        d = b - a
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        total += d
    return total


def _rdp2(pts, tol):
    """Ramer-Douglas-Peucker on 2D (x, y) points."""
    if len(pts) < 3:
        return list(pts)
    a, b = pts[0], pts[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    dmax, idx = 0.0, 0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        if L2 == 0.0:
            d = math.hypot(px - a[0], py - a[1])
        else:
            t = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
            d = math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))
        if d > dmax:
            dmax, idx = d, i
    if dmax <= tol:
        return [a, b]
    return _rdp2(pts[: idx + 1], tol)[:-1] + _rdp2(pts[idx:], tol)


def _grow_arc(poly, i, tol, min_pts=6, min_sweep_deg=12.0, max_sweep_deg=180.0, max_radius=5000.0, eligible=None):
    """Longest circular run starting at ``poly[i]``: (end_index, cx, cy, cw) or None.

    ``eligible`` (per-point bool) restricts the run to a contiguous eligible span,
    so arcs are never fitted across straight (line-origin) points."""
    n = len(poly)
    end = n - 1
    if eligible is not None:  # cap the run at the contiguous eligible span from i
        end = i
        while end + 1 < n and eligible[end + 1]:
            end += 1
    best = None
    j = i + min_pts - 1
    while j <= end:
        fit = _circle_fit(poly[i : j + 1])
        if fit is None:
            break
        cx, cy, rad = fit
        if rad > max_radius:
            break
        if max(abs(math.hypot(px - cx, py - cy) - rad) for px, py in poly[i : j + 1]) > tol:
            break
        sweep = _arc_sweep(poly[i : j + 1], cx, cy)
        if abs(sweep) > math.radians(max_sweep_deg):
            break
        best = (j, cx, cy, sweep)
        j += 1
    if best is None:
        return None
    j, cx, cy, sweep = best
    if abs(sweep) < math.radians(min_sweep_deg):
        return None
    return j, cx, cy, (sweep < 0.0)  # machine coords y-up: decreasing angle = cw (G02)


def _rdp2_idx(poly, lo, hi, tol):
    """RDP over poly[lo..hi] (XY), returning the kept *indices* (incl. lo, hi)."""
    if hi - lo < 2:
        return list(range(lo, hi + 1))
    a, b = poly[lo], poly[hi]
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    dmax, idx = 0.0, lo
    for i in range(lo + 1, hi):
        px, py = poly[i]
        if L2 == 0.0:
            d = math.hypot(px - a[0], py - a[1])
        else:
            t = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
            d = math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))
        if d > dmax:
            dmax, idx = d, i
    if dmax <= tol:
        return [lo, hi]
    return _rdp2_idx(poly, lo, idx, tol)[:-1] + _rdp2_idx(poly, idx, hi, tol)


def _fit_arcs(poly, arc_tol, line_tol, eligible=None):
    """Compress a polyline into (vertex indices, arcs). Arc runs collapse to one
    segment each; straight runs are RDP'd to lines. ``arcs`` maps segment index
    (position in the returned vertex list) -> (cx, cy, cw). Returning indices
    lets callers carry a per-vertex payload (e.g. depth) through the fit.

    ``eligible`` (per-point bool) marks which points may belong to an arc; a
    point that is not eligible (e.g. from a straight line segment) is never the
    start of, or spanned by, an arc -- so sharp line corners stay corners."""
    n = len(poly)
    if n < 2:
        return list(range(n)), {}
    vidx = [0]
    arcs: dict[int, tuple[float, float, bool]] = {}

    def flush(hi):  # emit line vertices from the last committed vertex up to hi
        if hi <= vidx[-1]:
            return
        for k in _rdp2_idx(poly, vidx[-1], hi, line_tol)[1:]:
            vidx.append(k)

    i = 0
    while i < n - 1:
        grown = None
        if eligible is None or eligible[i]:
            grown = _grow_arc(poly, i, arc_tol, eligible=eligible)
        if grown is not None:
            j, cx, cy, cw = grown
            flush(i)  # commit the straight run up to poly[i]
            vidx.append(j)
            arcs[len(vidx) - 2] = (cx, cy, cw)
            i = j
        else:
            i += 1
    flush(n - 1)
    return vidx, arcs


def _arc_points(p0, p1, cx, cy, cw, chord):
    """Sample an arc from p0 to p1 about (cx, cy) at ~``chord`` spacing (excl. p0)."""
    rad = math.hypot(p0[0] - cx, p0[1] - cy)
    a0 = math.atan2(p0[1] - cy, p0[0] - cx)
    a1 = math.atan2(p1[1] - cy, p1[0] - cx)
    if cw:
        while a1 > a0:
            a1 -= 2 * math.pi
    else:
        while a1 < a0:
            a1 += 2 * math.pi
    sweep = a1 - a0
    m = max(1, math.ceil(abs(sweep) * rad / max(chord, 1e-6)))
    return [
        (cx + rad * math.cos(a0 + sweep * k / m), cy + rad * math.sin(a0 + sweep * k / m))
        for k in range(1, m + 1)
    ]


def _densify(path: ToolPath, chord: float) -> list[tuple[float, float, float]]:
    """Expand a ToolPath to a dense (X, Y, depth) polyline, sampling any arcs."""
    if not path.pts:
        return []
    out = [path.pts[0]]
    for i in range(len(path.pts) - 1):
        p0, p1 = path.pts[i], path.pts[i + 1]
        arc = path.arcs.get(i)
        if arc is None:
            out.append(p1)
        else:
            cx, cy, cw = arc
            out.extend((x, y, p1[2]) for (x, y) in _arc_points(p0, p1, cx, cy, cw, chord))
    return out


def _stroke_toolpaths(spec: StrokeSpec, opts: VCarveOpts) -> list[ToolPath]:
    """Centreline groove (with arc fitting) + join/cap sharpening spurs."""
    r = spec.width_mm / 2.0
    depth = _depth_from_clearance(r, opts)

    def flip(x, y):
        return (opts.x0 + x, opts.y0 + (opts.page_h_mm - y))

    def line_tp(pts):  # SVG-mm (x, y, depth) -> Y-flipped straight ToolPath
        flipped = [(*flip(x, y), d) for (x, y, d) in pts]
        flipped = _rdp(flipped, opts.chord_tol_mm)
        return ToolPath(flipped) if len(flipped) >= 2 else None

    def main_tp(main_xy, curve_flags):  # arc-fitted centreline groove
        poly = [flip(x, y) for (x, y) in main_xy]
        # Only bezier/arc-origin geometry is an arc candidate; straight line
        # segments (triangle corners) and marker geometry are never fitted.
        if opts.arcs and not spec.is_marker and len(poly) >= 3:
            vidx, arcs = _fit_arcs(poly, opts.arc_tol_mm, opts.chord_tol_mm, eligible=curve_flags)
            verts = [poly[i] for i in vidx]
        else:
            verts, arcs = _rdp2(poly, opts.chord_tol_mm), {}
        if len(verts) < 2:
            return None
        return ToolPath([(x, y, depth) for (x, y) in verts], arcs)

    out: list[ToolPath] = []
    for si, sp in enumerate(spec.subpaths):
        if len(sp) < 2:
            continue
        flags = spec.subpath_curves[si] if si < len(spec.subpath_curves) else [True] * len(sp)
        closed = _is_closed(sp)
        spurs: list[list[tuple[float, float, float]]] = []

        # Join sharpening at every sharp vertex.
        for i in _corner_indices(sp, closed):
            spurs += _join_spurs(sp, i, closed, r, depth, spec)

        # Cap sharpening + groove trimming at open ends. An end covered by a
        # marker (an arrowhead) gets no cap *spurs* -- the arrowhead sits on it --
        # but the groove is still trimmed back so its rounded end tucks under the
        # marker instead of poking out at the vertex. The marker's own arm tips
        # do get capped (so they finish square).
        main, main_flags = sp, flags
        if not closed:
            cap_start = not spec.marker_start
            cap_end = not spec.marker_end
            if cap_start:
                spurs += _cap_spurs(sp, r, depth, spec.linecap, at_start=True)
            if cap_end:
                spurs += _cap_spurs(sp, r, depth, spec.linecap, at_start=False)
            trim_start = (cap_start and spec.linecap == "butt") or bool(spec.marker_start)
            trim_end = (cap_end and spec.linecap == "butt") or bool(spec.marker_end)
            if trim_start or trim_end:
                main, main_flags = _trim_polyline(
                    sp, r if trim_start else 0.0, r if trim_end else 0.0, flags=flags
                )

        tp = main_tp(main, main_flags)
        if tp is not None:
            out.append(tp)
        for spur in spurs:
            tp = line_tp(spur)
            if tp is not None:
                out.append(tp)
    return out


def parse_svg_features(
    svg_path: Path, opts: VCarveOpts
) -> tuple[set[str], list[StrokeSpec], list[tuple[str, tuple[float, float, float, float]]]]:
    """Classify shapes; return (stroke ids, stroke specs, filled elements).

    Each filled element is ``(id, (x0, y0, x1, y1))`` in millimetres, so it can
    be rendered on its own bbox. The list is falsy when there are no fills."""
    stroke_ids: set[str] = set()
    strokes: list[StrokeSpec] = []
    fill_elems: list[tuple[str, tuple[float, float, float, float]]] = []
    for _svgelements, e in _svg_shapes(svg_path):
        kind = _classify(e)
        if kind == "fill":
            eid = e.values.get("id")
            bb = e.bbox()
            if eid and bb is not None:
                fill_elems.append((eid, (float(bb[0]), float(bb[1]), float(bb[2]), float(bb[3]))))
        elif kind == "stroke":
            eid = e.values.get("id")
            if eid:
                stroke_ids.add(eid)
            flat = _flatten(e, opts.stroke_flatten_mm)
            if flat:
                try:
                    mlim = float(e.values.get("stroke-miterlimit") or 4.0)
                except (TypeError, ValueError):
                    mlim = 4.0
                strokes.append(
                    StrokeSpec(
                        subpaths=[pts for pts, _cf in flat],
                        subpath_curves=[cf for _pts, cf in flat],
                        width_mm=float(e.implicit_stroke_width),
                        linejoin=e.values.get("stroke-linejoin") or "miter",
                        linecap=e.values.get("stroke-linecap") or "butt",
                        miterlimit=mlim,
                        marker_start=e.values.get("marker-start"),
                        marker_end=e.values.get("marker-end"),
                    )
                )
    return stroke_ids, strokes, fill_elems


_SHAPE_TAGS = ("path", "rect", "circle", "ellipse", "line", "polyline", "polygon")


def _ensure_ids(svg_path: Path, dst: Path) -> None:
    """Copy ``svg_path`` to ``dst``, assigning an id to any shape lacking one, so
    every filled element can be targeted individually by ``--export-id``."""
    import xml.etree.ElementTree as ET

    for prefix, uri in _SVG_NS.items():
        ET.register_namespace(prefix, uri)
    tree = ET.parse(svg_path)
    seen = {n.get("id") for n in tree.getroot().iter() if n.get("id")}
    k = 0
    for n in tree.getroot().iter():
        if n.tag.split("}")[-1] in _SHAPE_TAGS and not n.get("id"):
            while f"vc{k}" in seen:
                k += 1
            n.set("id", f"vc{k}")
            seen.add(f"vc{k}")
            k += 1
    dst.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dst, xml_declaration=True, encoding="utf-8")


# --- marker (arrowhead) expansion -------------------------------------------


def _parse_marker_defs(svg_path: Path) -> dict:
    """Read <marker> definitions keyed by id (raw attributes + child paths)."""
    import xml.etree.ElementTree as ET

    def local(tag: str) -> str:
        return tag.split("}")[-1]

    markers: dict = {}
    root = ET.parse(svg_path).getroot()
    for m in root.iter():
        if local(m.tag) != "marker":
            continue
        children = [
            (c.get("d"), c.get("transform"), c.get("style", "") + ";" + (c.get("stroke-width") and f"stroke-width:{c.get('stroke-width')}" or ""))
            for c in m.iter()
            if local(c.tag) == "path" and c.get("d")
        ]
        markers[m.get("id")] = {
            "refX": float(m.get("refX", 0) or 0),
            "refY": float(m.get("refY", 0) or 0),
            "orient": m.get("orient", "0") or "0",
            "markerWidth": float(m.get("markerWidth", 3) or 3),
            "markerHeight": float(m.get("markerHeight", 3) or 3),
            "markerUnits": m.get("markerUnits") or "strokeWidth",
            "viewBox": m.get("viewBox"),
            "children": children,
        }
    return markers


def _style_num(style: str, key: str, default: float) -> float:
    for part in (style or "").split(";"):
        part = part.strip()
        if part.startswith(key + ":"):
            try:
                return float(part.split(":", 1)[1])
            except ValueError:
                return default
    return default


def _marker_specs(svg_path: Path, strokes: list[StrokeSpec], opts: VCarveOpts) -> list[StrokeSpec]:
    """Expand marker-start/-end refs on stroke paths into carveable specs."""
    svgelements = _require_svg()
    defs = _parse_marker_defs(svg_path)
    if not defs:
        return []
    out: list[StrokeSpec] = []
    for spec in strokes:
        for ref, at_start in ((spec.marker_start, True), (spec.marker_end, False)):
            if not ref:
                continue
            mid = ref.strip()[5:-1].lstrip("#") if ref.startswith("url(") else ref.lstrip("#")
            mdef = defs.get(mid)
            if mdef is None or not spec.subpaths:
                continue
            sp = spec.subpaths[0] if at_start else spec.subpaths[-1]
            if at_start:
                vertex = sp[0]
                tan = (sp[1][0] - sp[0][0], sp[1][1] - sp[0][1])
            else:
                vertex = sp[-1]
                tan = (sp[-1][0] - sp[-2][0], sp[-1][1] - sp[-2][1])
            mag = math.hypot(*tan) or 1.0
            ux, uy = tan[0] / mag, tan[1] / mag
            angle = math.atan2(uy, ux)
            orient = mdef["orient"]
            if orient == "auto-start-reverse" and at_start:
                angle += math.pi
            elif orient not in ("auto", "auto-start-reverse"):
                try:
                    angle = math.radians(float(orient))
                except ValueError:
                    angle = 0.0
            # content -> world scale
            vb = mdef["viewBox"]
            vbw = float(vb.split()[2]) if vb else mdef["markerWidth"]
            s = mdef["markerWidth"] / (vbw or 1.0)
            if mdef["markerUnits"] != "userSpaceOnUse":
                s *= spec.width_mm
            ca, sa = math.cos(angle), math.sin(angle)
            refx, refy = mdef["refX"], mdef["refY"]
            for cd, ctransform, cstyle in mdef["children"]:
                cpath = svgelements.Path(cd)
                if ctransform:
                    cpath *= svgelements.Matrix(ctransform)
                content = _flatten(cpath, max(opts.stroke_flatten_mm / max(s, 1e-6), 1e-3))
                world = []
                for subp, _cf in content:
                    pts = []
                    for (px, py) in subp:
                        lx, ly = (px - refx) * s, (py - refy) * s
                        wx = vertex[0] + (ca * lx - sa * ly)
                        wy = vertex[1] + (sa * lx + ca * ly)
                        pts.append((wx, wy))
                    world.append(pts)
                cw = _style_num(cstyle, "stroke-width", 1.0) * s
                out.append(StrokeSpec(subpaths=world, width_mm=cw, linecap="square", is_marker=True))
    return out


# The preview PNG is built at PREVIEW_SS x the requested output resolution
# (``--preview-pitch`` mm/px) and box-downsampled by PREVIEW_SS for antialiasing.
_PREVIEW_SS = 2


def _preview_dims(opts: VCarveOpts) -> tuple[float, int, int, int, int]:
    """Return (internal px/mm, internal w, internal h, output w, output h)."""
    ppmm = _PREVIEW_SS / max(opts.preview_pitch_mm, 1e-6)
    w = max(1, min(8000, round(opts.page_w_mm * ppmm)))
    h = max(1, min(8000, round(opts.page_h_mm * ppmm)))
    return ppmm, w, h, max(1, round(w / _PREVIEW_SS)), max(1, round(h / _PREVIEW_SS))


def render_preview(paths: list[ToolPath], opts: VCarveOpts, out_png: Path) -> None:
    """Write a carved-surface depth-map preview PNG in the drawing's orientation.

    Rather than drawing thin lines, this models what the V-bit actually removes:
    at plunge depth ``D`` the cone carves a swath of radius ``D * tan(t)`` around
    the toolpath, tapering from ``D`` at the centre to the surface at the rim.
    Each pixel is coloured by the **deepest** cut reaching it (deep = blue,
    shallow = pale), so overlaps and the bit's finite width show correctly. Built
    supersampled and downsampled for antialiasing; Y is flipped back to SVG
    orientation so text reads upright.
    """
    np, Image, _ = _require_deps()
    from scipy.ndimage import grey_dilation

    tan_t = math.tan(math.radians(opts.half_angle_deg))
    d_max = max((p.max_depth for p in paths), default=0.0)

    # Internal (supersampled) resolution: SS x the requested output px/mm.
    ppmm, w, h, _out_w, _out_h = _preview_dims(opts)

    # Stamp toolpath centreline depths into L (max where paths overlap), sampling
    # each segment at ~pixel spacing with interpolated depth.
    L = np.zeros((h, w), dtype=float)
    xs: list[float] = []
    ys: list[float] = []
    ds: list[float] = []

    def screen(X: float, Y: float) -> tuple[float, float]:
        y_svg = opts.page_h_mm - (Y - opts.y0)  # undo the bed Y-flip
        return ((X - opts.x0) * ppmm, y_svg * ppmm)

    for path in paths:
        dense = _densify(path, 1.0 / ppmm)  # sample arcs to ~1 px before stamping
        for (x0, y0, d0), (x1, y1, d1) in zip(dense[:-1], dense[1:], strict=False):
            sx0, sy0 = screen(x0, y0)
            sx1, sy1 = screen(x1, y1)
            n = max(1, math.ceil(math.hypot(sx1 - sx0, sy1 - sy0)))
            for i in range(n + 1):
                f = i / n
                xs.append(sx0 + (sx1 - sx0) * f)
                ys.append(sy0 + (sy1 - sy0) * f)
                ds.append(d0 + (d1 - d0) * f)

    if xs:
        xi = np.clip(np.round(np.array(xs)).astype(int), 0, w - 1)
        yi = np.clip(np.round(np.array(ys)).astype(int), 0, h - 1)
        np.maximum.at(L, (yi, xi), np.array(ds))

    if d_max > 0.0 and xs:
        # Spread each centreline depth into its V-groove: depth = max over nearby
        # toolpath of (D - dist/tan_t). A single big conical kernel is exact but
        # slow; because a cone dilated by a cone is a wider cone of the same
        # slope, we iterate a small conical dilation instead -- a few passes
        # rebuild the same envelope for a fraction of the cost.
        r_mm = d_max * tan_t
        if opts.bit_diameter_mm is not None:
            r_mm = min(r_mm, opts.bit_diameter_mm / 2.0)
        rk = 3  # small-kernel radius in px
        yy, xx = np.mgrid[-rk : rk + 1, -rk : rk + 1]
        dloc = np.hypot(xx, yy) / ppmm
        footprint = dloc <= rk / ppmm + 1e-9
        structure = np.where(footprint, -dloc / tan_t, 0.0)
        depth = L
        for _ in range(max(1, math.ceil(r_mm * ppmm / rk))):
            depth = grey_dilation(depth, footprint=footprint, structure=structure)
        np.maximum(depth, 0.0, out=depth)
    else:
        depth = L

    dnorm = d_max or opts.max_depth_mm or 1.0
    t = np.clip(depth / dnorm, 0.0, 1.0)
    carved = depth > 1e-6
    r_ch = np.where(carved, 235 - 205 * t, 255)
    g_ch = np.where(carved, 235 - 150 * t, 255)
    b_ch = np.where(carved, 60 + 150 * t, 255)
    rgb = np.stack([r_ch, g_ch, b_ch], axis=-1).clip(0, 255).astype("uint8")

    img = Image.fromarray(rgb, "RGB")
    img = img.resize((_out_w, _out_h), Image.LANCZOS)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_png)


def render_preview_svg(paths: list[ToolPath], opts: VCarveOpts, out_svg: Path) -> None:
    """Write a vector SVG of the recovered toolpath centrelines, depth-coloured.

    Complements the depth-map PNG: the PNG shows the *removed area*, this shows
    the *tool placement* -- each centreline (arcs sampled to a smooth polyline)
    drawn as a thin line whose colour tracks cut depth (deep = blue, shallow =
    pale). In the drawing's orientation so it overlays the source art."""
    d_max = max((p.max_depth for p in paths), default=0.0) or opts.max_depth_mm or 1.0
    band_w = d_max / 12.0

    def colour(depth: float) -> str:
        t = min(max(depth / d_max, 0.0), 1.0)
        return f"rgb({int(235 - 205 * t)},{int(235 - 150 * t)},{int(60 + 150 * t)})"

    def to_svg(X: float, Y: float) -> tuple[float, float]:
        return (X - opts.x0, opts.page_h_mm - (Y - opts.y0))

    elems: list[str] = []

    def flush(run, dsum, n):
        if len(run) >= 2:
            pts = " ".join(f"{x:.2f},{y:.2f}" for x, y in run)
            elems.append(
                f'<polyline points="{pts}" fill="none" stroke="{colour(dsum / n)}" '
                'stroke-width="0.35" stroke-linecap="round"/>'
            )

    for path in paths:
        dense = _densify(path, 0.2)
        if len(dense) < 2:
            continue
        run = [to_svg(dense[0][0], dense[0][1])]
        run_band = int(dense[0][2] / band_w)
        dsum, n = dense[0][2], 1
        for X, Y, z in dense[1:]:
            p = to_svg(X, Y)
            run.append(p)
            dsum += z
            n += 1
            b = int(z / band_w)
            if b != run_band:  # colour band changed -> start a new polyline
                flush(run, dsum, n)
                run, run_band, dsum, n = [p], b, z, 1
        flush(run, dsum, n)

    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{_fmt(opts.page_w_mm)}mm" '
        f'height="{_fmt(opts.page_h_mm)}mm" '
        f'viewBox="0 0 {_fmt(opts.page_w_mm)} {_fmt(opts.page_h_mm)}">\n'
        f'<rect width="{_fmt(opts.page_w_mm)}" height="{_fmt(opts.page_h_mm)}" fill="white"/>\n'
        + "\n".join(elems)
        + "\n</svg>\n"
    )
    out_svg.parent.mkdir(parents=True, exist_ok=True)
    out_svg.write_text(svg)


def run(svg_or_png: Path, opts: VCarveOpts, is_png: bool) -> str:
    t_start = time.time()
    paths: list[ToolPath] = []
    if is_png:
        _progress(opts, f"loading mask {svg_or_png.name}...")
        paths = build_toolpaths(load_png_to_mask(svg_or_png, opts.threshold), opts)
        _progress(opts, f"{len(paths)} toolpaths from mask")
    else:
        with tempfile.TemporaryDirectory() as td:
            work = Path(td) / "work.svg"
            _progress(opts, f"parsing {svg_or_png.name}...")
            _ensure_ids(svg_or_png, work)  # give every shape an id for per-element render
            _stroke_ids, strokes, fill_elems = parse_svg_features(work, opts)
            _progress(opts, f"{len(fill_elems)} filled element(s), {len(strokes)} stroke(s)")
            if fill_elems:
                paths += build_fill_toolpaths(work, fill_elems, opts)
            n0 = len(paths)
            for spec in strokes:
                paths += _stroke_toolpaths(spec, opts)
            _progress(opts, f"strokes: {len(paths) - n0} toolpaths")
            if opts.markers:
                n1 = len(paths)
                try:
                    for mspec in _marker_specs(work, strokes, opts):
                        paths += _stroke_toolpaths(mspec, opts)
                    if len(paths) > n1:
                        _progress(opts, f"markers: {len(paths) - n1} toolpaths")
                except Exception as e:  # markers are best-effort; never fail the carve
                    print(f"warning: could not expand markers ({e}); arrowheads skipped")
    if opts.preview_path is not None:
        out_svg = opts.preview_path.with_suffix(".svg")
        _, _, _, ow, oh = _preview_dims(opts)
        _progress(opts, f"writing PNG preview {opts.preview_path.name} ({_fmt(opts.preview_pitch_mm)}mm/px, {ow}x{oh}px)...")
        t0 = time.time()
        render_preview(paths, opts, opts.preview_path)
        _progress(opts, f"  wrote {opts.preview_path.name} ({time.time() - t0:.1f}s)")
        _progress(opts, f"writing SVG preview {out_svg.name}...")
        t1 = time.time()
        render_preview_svg(paths, opts, out_svg)
        _progress(opts, f"  wrote {out_svg.name} ({time.time() - t1:.1f}s)")
    _progress(opts, f"emitting g-code for {len(paths)} toolpaths...")
    gcode = emit_gcode(paths, opts)
    _progress(opts, f"done in {time.time() - t_start:.1f}s")
    return gcode


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--svg", type=Path, help="Input SVG (rendered via Inkscape)")
    src.add_argument("--png", type=Path, help="Pre-rendered black-on-white mask PNG")
    p.add_argument("--out", type=Path, required=True, help="Output .gcode path")
    p.add_argument("--page-w", type=float, default=300.0, help="Page width mm (default 300)")
    p.add_argument("--page-h", type=float, default=300.0, help="Page height mm (default 300)")
    p.add_argument("--pitch", type=float, default=0.05,
                   help="Raster pitch mm/px for the fills medial-axis pass (default 0.05; "
                        "finer keeps straight strokes straight)")
    p.add_argument("--threshold", type=int, default=200, help="Ink luminance cutoff 0-255 (default 200)")
    p.add_argument("--max-depth", type=float, default=3.0, help="Deepest cut mm (default 3.0)")
    p.add_argument("--min-depth", type=float, default=0.0, help="Floor for faint thin lines mm (default 0)")
    p.add_argument("--stepdown", type=float, default=0.6, help="Max depth per pass mm (default 0.6)")
    p.add_argument("--prune", type=float, default=0.4, help="Drop skeleton spurs shorter than mm (default 0.4)")
    p.add_argument("--chord-tol", type=float, default=0.05, help="Toolpath decimation tol mm (default 0.05)")
    p.add_argument("--fill-pitch", type=float, default=0.05, help="Per-element raster pitch for fills mm (default 0.05)")
    p.add_argument("--fill-arcs", action="store_true", help="Fit fill medial-axis runs to circular/helical arcs (default off)")
    p.add_argument("--fill-arc-tol", type=float, default=0.06, help="Arc-fit tolerance for fills mm (default 0.06)")
    p.add_argument("--x0", type=float, default=0.0, help="Page origin X on bed mm (default 0)")
    p.add_argument("--y0", type=float, default=0.0, help="Page origin Y on bed mm (default 0)")
    p.add_argument("--half-angle", type=float, default=None, help="V-bit half-angle deg (default 30, or from --tool)")
    p.add_argument("--feed", type=float, default=None, help="Cutting feed mm/min (default 400, or from --tool)")
    p.add_argument("--plunge-feed", type=float, default=None, help="Plunge feed mm/min (default 80, or from --tool)")
    p.add_argument("--z-safe", type=float, default=2.0, help="Rapid/clearance Z mm (default 2.0)")
    p.add_argument("--spindle-rpm", type=int, default=None, help="Spindle RPM (default 14000, or from --tool)")
    p.add_argument("--return-home", action="store_true", help="Append G00 X0 Y0 at end")
    p.add_argument("--workspace", default=None, choices=["G54", "G55", "G56", "G57", "G58", "G59"], help="Workspace offset to select")
    p.add_argument("--inkscape", default="inkscape", help="Inkscape executable (default 'inkscape')")
    p.add_argument("--stroke-flatten", type=float, default=0.3, help="Curve sampling step for stroke centrelines mm (default 0.3)")
    p.add_argument("--no-arcs", action="store_true", help="Do not fit circular stroke runs to G02/G03 arcs")
    p.add_argument("--arc-tol", type=float, default=0.02, help="Max deviation when fitting a stroke arc mm (default 0.02)")
    p.add_argument("--no-markers", action="store_true", help="Do not carve marker-start/-end arrowheads")
    p.add_argument("--preview", nargs="?", const="__AUTO__", default=None,
                   help="Also write a depth-coloured toolpath preview PNG (defaults to <out>.png)")
    p.add_argument("--preview-pitch", type=float, default=0.25,
                   help="Preview PNG resolution mm/px (default 0.25; smaller = larger image)")
    p.add_argument("--quiet", action="store_true", help="Suppress progress output")
    p.add_argument("--tool", default=None, help="Tool id/name; supplies half-angle, feeds, RPM defaults")
    args = p.parse_args(argv)

    tool = None
    if args.tool is not None:
        try:
            tool = _tools.find(_tools.load(), args.tool)
        except (KeyError, ValueError) as e:
            p.error(str(e))
    if args.half_angle is None and tool is not None and tool.half_angle_deg is None:
        p.error(f"tool {tool.id!r} has no half_angle_deg; pass --half-angle")

    opts = VCarveOpts(
        page_w_mm=args.page_w,
        page_h_mm=args.page_h,
        pitch_mm=args.pitch,
        half_angle_deg=_resolve(args.half_angle, tool, "half_angle_deg", 30.0),
        threshold=args.threshold,
        max_depth_mm=args.max_depth,
        min_depth_mm=args.min_depth,
        stepdown_mm=args.stepdown,
        prune_mm=args.prune,
        chord_tol_mm=args.chord_tol,
        x0=args.x0,
        y0=args.y0,
        feed=_resolve(args.feed, tool, "feed_mm_min", 400.0),
        plunge_feed=_resolve(args.plunge_feed, tool, "plunge_feed_mm_min", 80.0),
        z_safe=args.z_safe,
        spindle_rpm=_resolve(args.spindle_rpm, tool, "spindle_rpm", 14000),
        return_home=args.return_home,
        workspace=args.workspace,
        inkscape_bin=args.inkscape,
        stroke_flatten_mm=args.stroke_flatten,
        markers=not args.no_markers,
        arcs=not args.no_arcs,
        arc_tol_mm=args.arc_tol,
        fill_arcs=args.fill_arcs,
        fill_arc_tol_mm=args.fill_arc_tol,
        fill_pitch_mm=args.fill_pitch,
        preview_path=(
            None
            if args.preview is None
            else (args.out.with_suffix(".png") if args.preview == "__AUTO__" else Path(args.preview))
        ),
        preview_pitch_mm=args.preview_pitch,
        bit_diameter_mm=(tool.diameter_mm if tool is not None else None),
        progress=not args.quiet,
    )
    if args.pitch <= 0:
        p.error("--pitch must be > 0")
    if args.stepdown <= 0:
        p.error("--stepdown must be > 0")

    gcode = run(args.png or args.svg, opts, is_png=args.png is not None)
    args.out.write_text(gcode)
    print(f"wrote {args.out} ({len(gcode.splitlines())} lines)")
    if opts.preview_path is not None:
        print(f"wrote preview {opts.preview_path} and {opts.preview_path.with_suffix('.svg')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
