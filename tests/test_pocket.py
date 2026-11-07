"""Tests for rollingmill/pocket.py -- spiral-out roughing pocket g-code emit."""

from __future__ import annotations

import math
import re

import pytest

from rollingmill import pocket


def _words(line: str) -> dict[str, float]:
    return {m.group(1): float(m.group(2)) for m in re.finditer(r"([A-Z])(-?\d+\.?\d*)", line)}


def _trace(gcode: str, step: float = 0.1):
    """Walk the cutting moves; yield (z, x, y) samples along every G01/G02/G03."""
    x = y = z = 0.0
    pts = []
    for line in gcode.splitlines():
        if line.startswith("("):
            continue
        wd = _words(line)
        g = wd.get("G")
        if g is None and not any(k in wd for k in "XYZ"):
            continue
        nx, ny, nz = wd.get("X", x), wd.get("Y", y), wd.get("Z", z)
        if line.startswith(("G02", "G03")):
            xc, yc = x + wd["I"], y + wd["J"]
            r0 = math.hypot(x - xc, y - yc)
            r1 = math.hypot(nx - xc, ny - yc)
            assert r0 == pytest.approx(r1, abs=2e-3), line
            a0, a1 = math.atan2(y - yc, x - xc), math.atan2(ny - yc, nx - xc)
            d = a1 - a0
            if line.startswith("G03"):
                d %= 2 * math.pi
            else:
                d = -((-d) % (2 * math.pi))
            n = max(1, int(abs(d) * r0 / step))
            for k in range(1, n + 1):
                a = a0 + d * k / n
                pts.append((nz, xc + r0 * math.cos(a), yc + r0 * math.sin(a)))
        elif line.startswith("G01"):
            n = max(1, int(math.hypot(nx - x, ny - y) / step))
            for k in range(1, n + 1):
                pts.append((nz, x + (nx - x) * k / n, y + (ny - y) * k / n))
        x, y, z = nx, ny, nz
    return pts


def _opts(**kw) -> pocket.PocketOpts:
    base = dict(width_mm=30.0, height_mm=20.0, depth_mm=1.0, tool_dia_mm=3.0, stepover=0.4)
    base.update(kw)
    return pocket.PocketOpts(**base)


def test_levels_and_final_depth():
    g = pocket.emit_gcode(_opts(depth_mm=1.2, z_step_mm=0.5))
    # Entry is helical: G01 Z only drops to the previous floor (in air) ...
    assert [float(m) for m in re.findall(r"G01 Z(-?[\d.]+)", g)] == [0.0, -0.5, -1.0]
    # ... and each level's helix arcs carry Z down to the new floor.
    arc_zs = [float(m) for m in re.findall(r"G0[23] X\S+ Y\S+ Z(-?[\d.]+)", g)]
    assert arc_zs == [-0.25, -0.5, -0.75, -1.0, -1.1, -1.2]
    assert g.rstrip().endswith("M30")


@pytest.mark.parametrize(
    "w,h,center,conv",
    [(30, 20, False, False), (20, 30, False, False), (25, 25, True, True), (12, 40, True, False)],
)
def test_stays_in_bounds_and_clears_everything(w, h, center, conv):
    tool = 3.0
    g = pocket.emit_gcode(
        _opts(width_mm=w, height_mm=h, tool_dia_mm=tool, center_origin=center, conventional=conv)
    )
    pts = [(x, y) for z, x, y in _trace(g) if z == -1.0]
    lo_x, lo_y = (-w / 2, -h / 2) if center else (0.0, 0.0)
    r = tool / 2
    for x, y in pts:
        assert lo_x + r - 1e-3 <= x <= lo_x + w - r + 1e-3
        assert lo_y + r - 1e-3 <= y <= lo_y + h - r + 1e-3
    # Every point of the pocket (bar the tool-radius corner fillets) is swept.
    n = 40
    for i in range(n + 1):
        for j in range(n + 1):
            px, py = lo_x + w * i / n, lo_y + h * j / n
            ex = max(lo_x + r - px, 0, px - (lo_x + w - r))
            ey = max(lo_y + r - py, 0, py - (lo_y + h - r))
            if math.hypot(ex, ey) > r:
                continue  # in the unreachable corner
            assert min(math.hypot(px - x, py - y) for x, y in pts) <= r + 1e-3, (px, py)


def test_climb_is_clockwise_and_conventional_reverses():
    assert "G03" not in pocket.emit_gcode(_opts())
    assert "G02" in pocket.emit_gcode(_opts())
    conv = pocket.emit_gcode(_opts(conventional=True))
    assert "G02" not in conv and "G03" in conv


def test_helical_entry_is_centred_on_pocket():
    # Helix of r=0.6 (half a 1.2 mm step) round (15, 10).
    g = pocket.emit_gcode(_opts())
    assert "G00 X15.0 Y10.6\nG01 Z0.0 F50.0\nF300.0\nG02 X15.0 Y9.4 Z-0.25 I0.0 J-0.6" in g
    g = pocket.emit_gcode(_opts(center_origin=True, x0=5.0))
    assert "G00 X5.0 Y0.6\n" in g


@pytest.mark.parametrize(
    "kw",
    [
        dict(width_mm=3.0),
        dict(height_mm=2.0),
        dict(depth_mm=0),
        dict(z_step_mm=0),
        dict(stepover=0),
        dict(stepover=1.5),
        dict(tool_dia_mm=0),
    ],
)
def test_rejects_bad_options(kw):
    with pytest.raises(ValueError):
        pocket.emit_gcode(_opts(**kw))


def test_step_out_keeps_moving_along_long_axis():
    # Closing turn 1 returns to (30, 10.6); the step out to turn 2 (Y11.8) must
    # be a slanted line that also advances in X, not a pure sideways jog.
    g = pocket.emit_gcode(_opts(width_mm=60.0, height_mm=20.0, depth_mm=0.5)).splitlines()
    i = g.index("G01 X30.0 Y10.6")
    step = _words(g[i + 1])
    assert g[i + 1].startswith("G01") and step["Y"] == pytest.approx(11.8)
    assert step["X"] > 30.0


def test_rows_evenly_spaced_from_the_centre():
    # Rows either side of the axis are one step apart, like every other pair --
    # not two steps, which left a gap down the middle when the cutter was small.
    assert pocket._radii(37.5, 3.0)[:3] == [1.5, 4.5, 7.5]
    assert pocket._radii(37.5, 3.0)[-1] == 37.5
    # A turn within a quarter step of the last is merged into it.
    assert pocket._radii(8.0, 3.0) == [1.5, 4.5, 8.0]
    assert pocket._radii(1.0, 3.0) == [1.0]


def test_header_records_tool_values():
    g = pocket.emit_gcode(_opts(tool_label="end_5mm - 5mm flat", feed=400.0, spindle_rpm=12000))
    assert (
        "(tool end_5mm - 5mm flat: dia 3 mm, radius 1.5 mm, feed 400, plunge 50 mm/min, S12000)"
        in g
    )
    assert "stepover: 1.2 mm = 40% of tool" in g
    assert "(tool (no library entry): dia 3 mm" in pocket.emit_gcode(_opts())
