"""Tests for rollingmill/spiral.py -- neat-spiral (hypotrochoid) g-code emit."""
from __future__ import annotations

import math
from pathlib import Path

import pytest

from rollingmill import spiral


# ---------- helpers ----------

def _xs(s: str) -> list[float]:
    xs: list[float] = []
    for line in s.splitlines():
        for tok in line.split():
            if tok.startswith("X") and len(tok) > 1:
                try:
                    xs.append(float(tok[1:]))
                except ValueError:
                    pass
    return xs


def _ys(s: str) -> list[float]:
    ys: list[float] = []
    for line in s.splitlines():
        for tok in line.split():
            if tok.startswith("Y") and len(tok) > 1:
                try:
                    ys.append(float(tok[1:]))
                except ValueError:
                    pass
    return ys


# ---------- header / footer order ----------

def test_emit_header_and_footer_order() -> None:
    out = spiral.emit_gcode(96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0, spindle_rpm=8000))
    lines = out.splitlines()
    assert "G21" in lines
    assert "G90" in lines
    assert "G17" in lines
    assert "S8000" in lines
    assert "M03" in lines
    assert "M05" in lines
    assert "M30" in lines
    assert lines.index("S8000") < lines.index("M03")
    assert lines.index("M03") < lines.index("G00 Z2.0")
    assert lines.index("M03") < lines.index("M05") < lines.index("M30")


def test_emit_workspace_in_header() -> None:
    out = spiral.emit_gcode(
        96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0, workspace="G54")
    )
    lines = out.splitlines()
    assert "G54" in lines
    assert lines.index("G54") < lines.index("G00 Z2.0")


def test_emit_no_workspace_by_default() -> None:
    out = spiral.emit_gcode(96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0))
    wcs = {"G54", "G55", "G56", "G57", "G58", "G59"}
    assert not any(l.strip() in wcs for l in out.splitlines())


def test_emit_return_home_appended() -> None:
    out = spiral.emit_gcode(
        96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0, return_home=True)
    )
    lines = out.splitlines()
    assert "G00 X0.0 Y0.0" in lines
    assert lines.index("G00 X0.0 Y0.0") > lines.index("M03")
    assert lines.index("G00 X0.0 Y0.0") < lines.index("M30")


# ---------- single pen cycle ----------

def test_single_pen_down_up_cycle() -> None:
    out = spiral.emit_gcode(96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0))
    lines = out.splitlines()
    # Exactly one pen-down plunge.
    plunges = [l for l in lines if l.startswith("G01 Z")]
    assert len(plunges) == 1
    # At least two pen-up lifts (one in header, one after drawing).
    lifts = [l for l in lines if l.startswith("G00 Z")]
    assert len(lifts) >= 2
    # Drawing strokes are arcs (G02/G03), with rare G01 fallback for
    # collinear segments.
    arcs = [l for l in lines if l.startswith("G02 ") or l.startswith("G03 ")]
    assert len(arcs) >= 20


def test_arc_output_is_compact() -> None:
    """Arc-based emit should be dramatically smaller than a chord-only
    approximation at the same tolerance."""
    out = spiral.emit_gcode(96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0))
    # The equivalent G01-chord output at chord_tol_mm=0.05 is ~60k lines.
    # Arc fitting should be at least 10x smaller.
    assert len(out.splitlines()) < 6000


def test_arc_ij_are_incremental() -> None:
    """Each arc's centre, computed from start point + (I, J), must lie at
    `radius` away from both the start and end points (within tolerance)."""
    out = spiral.emit_gcode(96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0))
    lines = out.splitlines()
    # Track current pen XY through the program.
    cur_x: float | None = None
    cur_y: float | None = None
    checked = 0
    for line in lines:
        toks = line.split()
        if not toks:
            continue
        op = toks[0]
        coords: dict[str, float] = {}
        for tok in toks[1:]:
            if len(tok) > 1 and tok[0] in "XYIJZ":
                try:
                    coords[tok[0]] = float(tok[1:])
                except ValueError:
                    pass
        if op in ("G00", "G01") and "X" in coords and "Y" in coords:
            cur_x, cur_y = coords["X"], coords["Y"]
        elif op in ("G02", "G03") and cur_x is not None:
            ex, ey = coords["X"], coords["Y"]
            i_rel, j_rel = coords["I"], coords["J"]
            cx, cy = cur_x + i_rel, cur_y + j_rel
            r_start = math.hypot(cur_x - cx, cur_y - cy)
            r_end = math.hypot(ex - cx, ey - cy)
            assert abs(r_start - r_end) < 0.01, (
                f"arc centre inconsistent: r_start={r_start}, r_end={r_end}, "
                f"line={line!r}"
            )
            cur_x, cur_y = ex, ey
            checked += 1
    assert checked > 20


# ---------- geometry: centre + scale ----------

def test_pattern_centred_and_bounded() -> None:
    radius = 10.0
    cx, cy = 50.0, 25.0
    out = spiral.emit_gcode(
        96, 52, 30.0, spiral.SpiralOpts(radius_mm=radius, x0=cx, y0=cy)
    )
    xs = _xs(out)
    ys = _ys(out)
    # Skip the trailing return-home zeros (not enabled here, but be defensive).
    drawing_xs = [x for x in xs if x != 0.0]
    drawing_ys = [y for y in ys if y != 0.0]
    eps = 0.05
    assert all(cx - radius - eps <= x <= cx + radius + eps for x in drawing_xs)
    assert all(cy - radius - eps <= y <= cy + radius + eps for y in drawing_ys)
    # And the pattern actually reaches close to the radius envelope.
    rmax = max(
        math.hypot(x - cx, y - cy)
        for x, y in zip(drawing_xs, drawing_ys)
    )
    assert abs(rmax - radius) < 0.5  # within sampling tolerance


# ---------- closure ----------

def test_curve_closes_to_start() -> None:
    out = spiral.emit_gcode(96, 52, 30.0, spiral.SpiralOpts(radius_mm=40.0))
    rapids = [l for l in out.splitlines() if l.startswith("G00 X")]
    drawn = [
        l for l in out.splitlines()
        if l.startswith(("G01 X", "G02 ", "G03 "))
    ]
    # First XY is the rapid to start; last drawn move closes the curve.
    first = rapids[0]
    last = drawn[-1]

    def xy(s: str) -> tuple[float, float]:
        x = y = 0.0
        for tok in s.split():
            if tok.startswith("X"):
                x = float(tok[1:])
            elif tok.startswith("Y"):
                y = float(tok[1:])
        return x, y

    fx, fy = xy(first)
    lx, ly = xy(last)
    assert abs(fx - lx) < 0.1
    assert abs(fy - ly) < 0.1


# ---------- geometry sanity: hypocycloid limit ----------

def test_hypocycloid_max_extent_equals_radius() -> None:
    """When d == r the curve is a hypocycloid whose cusps touch a circle of
    radius (R - r) + d = R - r + r = R (in tooth units). After scaling, max
    radial extent should equal radius_mm."""
    out = spiral.emit_gcode(5, 3, 3.0, spiral.SpiralOpts(radius_mm=20.0, chord_tol_mm=0.01))
    xs = _xs(out)
    ys = _ys(out)
    rmax = max(math.hypot(x, y) for x, y in zip(xs, ys))
    assert abs(rmax - 20.0) < 0.1


# ---------- CLI ----------

def test_cli_smoke(tmp_path: Path) -> None:
    out_path = tmp_path / "s.gcode"
    rc = spiral.main([
        "--ring", "96", "--rotor", "52", "--pen", "30",
        "--radius", "40", "--out", str(out_path),
    ])
    assert rc == 0
    text = out_path.read_text()
    assert "M30" in text
    assert "G21" in text


def test_cli_workspace_and_centre(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out_path = tmp_path / "s.gcode"
    rc = spiral.main([
        "--ring", "96", "--rotor", "52", "--pen", "30",
        "--radius", "15", "--x0", "100", "--y0", "100",
        "--workspace", "G54",
        "--out", str(out_path),
    ])
    assert rc == 0
    text = out_path.read_text()
    assert "G54" in text.splitlines()
    captured = capsys.readouterr().out
    assert str(out_path) in captured


# ---------- validation ----------

def test_emit_rejects_rotor_ge_ring() -> None:
    with pytest.raises(ValueError):
        spiral.emit_gcode(30, 30, 10.0, spiral.SpiralOpts(radius_mm=20.0))
    with pytest.raises(ValueError):
        spiral.emit_gcode(20, 30, 10.0, spiral.SpiralOpts(radius_mm=20.0))


def test_emit_rejects_non_positive_radius() -> None:
    with pytest.raises(ValueError):
        spiral.emit_gcode(96, 52, 30.0, spiral.SpiralOpts(radius_mm=0.0))


def test_cli_rejects_rotor_ge_ring(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as ei:
        spiral.main([
            "--ring", "30", "--rotor", "30", "--pen", "10",
            "--radius", "20", "--out", str(tmp_path / "x.gcode"),
        ])
    assert ei.value.code != 0
