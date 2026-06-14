"""Tests for rollingmill.maze — erbsland-maze-backed g-code emit."""
from __future__ import annotations

from pathlib import Path

import pytest

# erbsland_maze drags in pycairo; if either is missing the whole module is
# untestable. Skip cleanly rather than failing collection.
pytest.importorskip("erbsland_maze")
pytest.importorskip("cairo")

from rollingmill import maze  # noqa: E402


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


def _opts(**overrides) -> maze.MazeOpts:
    base = dict(width=40.0, height=40.0, cell=4.0, wall=1.7, seed=42)
    base.update(overrides)
    return maze.MazeOpts(**base)


# ---------- header / footer order ----------

def test_emit_header_and_footer_order() -> None:
    out = maze.emit_gcode(_opts(spindle_rpm=8000))
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
    out = maze.emit_gcode(_opts(workspace="G54"))
    lines = out.splitlines()
    assert "G54" in lines
    assert lines.index("G54") < lines.index("G00 Z2.0")


def test_emit_no_workspace_by_default() -> None:
    out = maze.emit_gcode(_opts())
    wcs = {"G54", "G55", "G56", "G57", "G58", "G59"}
    assert not any(l.strip() in wcs for l in out.splitlines())


def test_emit_return_home_appended() -> None:
    out = maze.emit_gcode(_opts(return_home=True))
    lines = out.splitlines()
    assert "G00 X0.0 Y0.0" in lines
    assert lines.index("G00 X0.0 Y0.0") > lines.index("M03")
    assert lines.index("G00 X0.0 Y0.0") < lines.index("M30")


# ---------- determinism ----------

def test_same_seed_same_output() -> None:
    a = maze.emit_gcode(_opts(seed=123))
    b = maze.emit_gcode(_opts(seed=123))
    assert a == b


def test_different_seed_different_output() -> None:
    a = maze.emit_gcode(_opts(seed=1))
    b = maze.emit_gcode(_opts(seed=2))
    assert a != b


# ---------- geometry: bounds + Y-flip ----------

def test_xy_within_canvas_bounds() -> None:
    opts = _opts(width=50.0, height=30.0, x0=10.0, y0=5.0)
    out = maze.emit_gcode(opts)
    # Skip the X0 Y0 the test fixtures might add; emit no return_home.
    xs = _xs(out)
    ys = _ys(out)
    eps = 0.01
    assert all(opts.x0 - eps <= x <= opts.x0 + opts.width + eps for x in xs)
    assert all(opts.y0 - eps <= y <= opts.y0 + opts.height + eps for y in ys)


def test_y_axis_is_flipped() -> None:
    """With Y-flip, drawn Y values must span both ends of the canvas (close to
    0 and close to height) and the first drawn rapid lands at the TOP of the
    bed (Y near height), because walls are extracted top-to-bottom in SVG
    space — the canvas-top wall maps to a mill-top Y after flipping."""
    height = 40.0
    out = maze.emit_gcode(_opts(width=40.0, height=height))
    ys = _ys(out)
    assert max(ys) > height - 0.5
    assert min(ys) < 0.5
    first_rapid = next(l for l in out.splitlines()
                       if l.startswith("G00 X") and "Y" in l)
    y = float(next(t for t in first_rapid.split() if t.startswith("Y"))[1:])
    assert y > height * 0.7  # ~top of the bed after flip


# ---------- single pen cycle structure ----------

def test_each_polyline_has_pen_down_up() -> None:
    out = maze.emit_gcode(_opts())
    lines = out.splitlines()
    plunges = [l for l in lines if l.startswith("G01 Z")]
    lifts = [l for l in lines if l.startswith("G00 Z")]
    rapids_xy = [l for l in lines if l.startswith("G00 X") and "Y" in l
                 and l != "G00 X0.0 Y0.0"]
    # Every drawn polyline has one G00 X Y rapid + one Z plunge.
    assert len(plunges) == len(rapids_xy)
    # Lifts include the initial header lift, one per polyline, and the
    # trailing footer lift.
    assert len(lifts) >= len(plunges) + 2


def test_nontrivial_stroke_count() -> None:
    out = maze.emit_gcode(_opts(width=100.0, height=100.0))
    g01_xy = [l for l in out.splitlines()
              if l.startswith("G01 X") and "Y" in l]
    # A 100x100 / 4 mm cell maze has hundreds of wall segments.
    assert len(g01_xy) > 100


# ---------- endpoint marker modes ----------

def test_endpoints_none_skips_markers() -> None:
    none_out = maze.emit_gcode(_opts(endpoints="none"))
    outline_out = maze.emit_gcode(_opts(endpoints="outline"))
    none_lines = len(none_out.splitlines())
    out_lines = len(outline_out.splitlines())
    assert out_lines > none_lines


def test_endpoints_filled_more_than_outline() -> None:
    outline_out = maze.emit_gcode(_opts(endpoints="outline"))
    filled_out = maze.emit_gcode(_opts(endpoints="filled"))
    assert len(filled_out.splitlines()) > len(outline_out.splitlines())


def test_emit_rejects_unknown_endpoint_mode() -> None:
    with pytest.raises(ValueError):
        maze.emit_gcode(_opts(endpoints="bogus"))


# ---------- validation ----------

def test_emit_rejects_non_positive_dims() -> None:
    with pytest.raises(ValueError):
        maze.emit_gcode(_opts(width=0.0))
    with pytest.raises(ValueError):
        maze.emit_gcode(_opts(height=-1.0))


def test_emit_rejects_non_positive_cell_wall() -> None:
    with pytest.raises(ValueError):
        maze.emit_gcode(_opts(cell=0.0))
    with pytest.raises(ValueError):
        maze.emit_gcode(_opts(wall=0.0))


# ---------- CLI ----------

def test_cli_smoke(tmp_path: Path) -> None:
    out_path = tmp_path / "m.gcode"
    rc = maze.main([
        "--out", str(out_path), "--width", "40", "--height", "40",
        "--seed", "42",
    ])
    assert rc == 0
    text = out_path.read_text()
    assert "M30" in text
    assert "G21" in text
    assert "(Seed: 42" in text


def test_cli_seed_in_header(tmp_path: Path) -> None:
    out_path = tmp_path / "m.gcode"
    rc = maze.main([
        "--out", str(out_path), "--width", "30", "--height", "30",
        "--seed", "7",
    ])
    assert rc == 0
    assert "(Seed: 7" in out_path.read_text()


def test_cli_workspace_round_trip(tmp_path: Path,
                                  capsys: pytest.CaptureFixture[str]) -> None:
    out_path = tmp_path / "m.gcode"
    rc = maze.main([
        "--out", str(out_path), "--width", "30", "--height", "30",
        "--seed", "5", "--workspace", "G55",
    ])
    assert rc == 0
    assert "G55" in out_path.read_text().splitlines()
    captured = capsys.readouterr().out
    assert str(out_path) in captured
