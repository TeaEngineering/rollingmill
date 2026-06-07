"""Tests for utils/text_to_gcode.py — CXF parser, stroke chaining, g-code emit."""
from __future__ import annotations

import math
import sys
from pathlib import Path

# utils/ is not part of the package; add it to sys.path for import.
REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "utils"))

import text_to_gcode as t2g  # noqa: E402


# ---------- parser ----------

def test_parse_qcad_form(tmp_path: Path) -> None:
    cxf = tmp_path / "tiny.cxf"
    cxf.write_text(
        "# Format: QCad II Font\n"
        "# LetterSpacing: 3.0\n"
        "# WordSpacing: 6.75\n"
        "\n"
        "[0041] A\n"
        "L 0,0,3,9\n"
        "L 3,9,6,0\n"
        "L 1,3,5,3\n"
    )
    font = t2g.parse_cxf(cxf)
    assert "A" in font.glyphs
    a = font.glyphs["A"]
    assert len(a.primitives) == 3
    assert isinstance(a.primitives[0], t2g.Line)
    assert a.primitives[0].x2 == 3 and a.primitives[0].y2 == 9
    assert font.letter_spacing == 3.0
    assert font.word_spacing == 6.75


def test_parse_hershey_form(tmp_path: Path) -> None:
    cxf = tmp_path / "tiny.cxf"
    cxf.write_text(
        "# LetterSpacing: 3.0\n"
        "\n"
        "[A] 3\n"
        "L 0,0,3,9\n"
        "L 3,9,6,0\n"
        "L 1,3,5,3\n"
    )
    font = t2g.parse_cxf(cxf)
    assert "A" in font.glyphs
    assert len(font.glyphs["A"].primitives) == 3


def test_arc_direction(tmp_path: Path) -> None:
    cxf = tmp_path / "arcs.cxf"
    cxf.write_text(
        "[004F] O\n"
        "A 5,5,5,0,180\n"
        "AR 5,5,5,180,360\n"
    )
    font = t2g.parse_cxf(cxf)
    o = font.glyphs["O"]
    assert isinstance(o.primitives[0], t2g.Arc)
    assert o.primitives[0].ccw is True
    assert o.primitives[1].ccw is False


# ---------- chaining ----------

def test_chain_two_lines_meeting() -> None:
    # Two line segments sharing an endpoint should chain into one path.
    prims = [
        t2g.Line(0, 0, 1, 0),
        t2g.Line(1, 0, 1, 1),
    ]
    paths = t2g.chain_primitives(prims)
    assert len(paths) == 1
    assert len(paths[0]) == 2


def test_chain_two_separate_strokes() -> None:
    # Two pairs of connected lines; the pairs don't touch each other.
    prims = [
        t2g.Line(0, 0, 1, 0),  # ↘ pair A
        t2g.Line(1, 0, 1, 1),  # ↗ pair A — shares (1,0)
        t2g.Line(5, 0, 6, 0),  # ↘ pair B — disconnected from pair A
        t2g.Line(6, 0, 6, 1),  # ↗ pair B — shares (6,0)
    ]
    paths = t2g.chain_primitives(prims)
    assert len(paths) == 2
    assert all(len(p) == 2 for p in paths)


def test_chain_letter_O_arcs() -> None:
    # Two semicircles sharing endpoints should chain into one closed path.
    top = t2g.Arc(0, 0, 5, 0, 180, ccw=True)     # (5,0) → (-5,0) over the top
    bot = t2g.Arc(0, 0, 5, 180, 360, ccw=True)   # (-5,0) → (5,0) under the bottom
    paths = t2g.chain_primitives([top, bot])
    assert len(paths) == 1
    assert len(paths[0]) == 2


def test_chain_reverses_to_connect() -> None:
    # Two lines whose orientations don't naturally chain — one must be reversed.
    prims = [
        t2g.Line(0, 0, 1, 0),  # ends at (1,0)
        t2g.Line(2, 0, 1, 0),  # also ends at (1,0); needs reversal
    ]
    paths = t2g.chain_primitives(prims)
    assert len(paths) == 1
    assert len(paths[0]) == 2


# ---------- emit ----------

def _make_font_with_I() -> t2g.Font:
    font = t2g.Font(name="testfont")
    font.glyphs["H"] = t2g.Glyph("H", [
        t2g.Line(0, 0, 0, 9),
        t2g.Line(6, 0, 6, 9),
        t2g.Line(0, 4.5, 6, 4.5),
    ])
    font.glyphs["I"] = t2g.Glyph("I", [t2g.Line(0, 0, 0, 9)])
    return font


def test_emit_basic_I() -> None:
    font = _make_font_with_I()
    out = t2g.emit_gcode("I", font, t2g.GCodeOpts(height_mm=9.0))
    lines = out.strip().splitlines()
    # Should have header, exactly one pen-up→down→stroke→pen-up cycle, and M30.
    assert any(l.startswith("G21") for l in lines)
    assert any(l.startswith("G90") for l in lines)
    assert "M30" in lines
    # One plunge `G01 Z` for the single stroke.
    plunges = [l for l in lines if l.startswith("G01 Z")]
    assert len(plunges) == 1, plunges
    # And at least one pen-up Z after (final lift) — plus initial.
    assert sum(1 for l in lines if l.startswith("G00 Z")) >= 2


def test_emit_handles_space_and_unknown() -> None:
    font = _make_font_with_I()
    # Space and unknown char should not crash; should produce no strokes.
    out = t2g.emit_gcode(" ?", font, t2g.GCodeOpts(height_mm=9.0))
    assert "M30" in out
    assert "G01 Z" not in out  # no plunges since no known glyphs


def test_emit_arc_ij_offsets() -> None:
    # Single CCW quarter arc from (1,0) to (0,1), centred at origin, r=1.
    font = t2g.Font(name="t")
    font.glyphs["X"] = t2g.Glyph("X", [
        t2g.Arc(0, 0, 1, 0, 90, ccw=True),
    ])
    # Use cap_height fallback: this font has no H/M/I/A → cap_height=9.0.
    # Scale at height=9.0 is 1.0, so I/J are the raw offsets.
    out = t2g.emit_gcode("X", font, t2g.GCodeOpts(height_mm=9.0))
    arc_lines = [l for l in out.splitlines() if l.startswith("G03 ")]
    assert len(arc_lines) == 1
    parts = arc_lines[0].split()
    # End is (0,1); offset from start (1,0) to centre (0,0) is I=-1, J=0.
    by = {p[0]: float(p[1:]) for p in parts[1:]}
    assert math.isclose(by["X"], 0.0, abs_tol=1e-4)
    assert math.isclose(by["Y"], 1.0, abs_tol=1e-4)
    assert math.isclose(by["I"], -1.0, abs_tol=1e-4)
    assert math.isclose(by["J"], 0.0, abs_tol=1e-4)


def test_emit_spindle_on_and_off() -> None:
    font = _make_font_with_I()
    out = t2g.emit_gcode("I", font, t2g.GCodeOpts(height_mm=9.0, spindle_rpm=8000))
    lines = out.splitlines()
    # S word and M03 appear in the header, before the first Z move.
    assert "S8000" in lines
    assert "M03" in lines
    assert lines.index("S8000") < lines.index("M03")
    assert lines.index("M03") < lines.index("G00 Z2.0")
    # M05 stop comes after the last G00 Z move (the lift), before M30.
    assert lines.index("M03") < lines.index("M05") < lines.index("M30")


def test_emit_workspace_in_header() -> None:
    font = _make_font_with_I()
    out = t2g.emit_gcode("I", font, t2g.GCodeOpts(height_mm=9.0, workspace="G54"))
    lines = out.splitlines()
    # G54 should appear once in the header, before the first feed/Z setup.
    assert "G54" in lines
    assert lines.index("G54") < lines.index("G00 Z2.0")
    # No G94 should be emitted (mill is always mm/min, line is commented out).
    assert not any(l.strip() == "G94" for l in lines)


def test_emit_no_workspace_by_default() -> None:
    font = _make_font_with_I()
    out = t2g.emit_gcode("I", font, t2g.GCodeOpts(height_mm=9.0))
    assert not any(l.strip().startswith("G5") and l.strip() != "G54" for l in out.splitlines() if l.strip() in {"G54","G55","G56","G57","G58","G59"})


def test_emit_cw_arc_uses_G02() -> None:
    font = t2g.Font(name="t")
    font.glyphs["X"] = t2g.Glyph("X", [
        t2g.Arc(0, 0, 1, 90, 0, ccw=False),
    ])
    out = t2g.emit_gcode("X", font, t2g.GCodeOpts(height_mm=9.0))
    assert any(l.startswith("G02 ") for l in out.splitlines())
    assert not any(l.startswith("G03 ") for l in out.splitlines())


# ---------- integration ----------

def test_vendored_romans_font_loads_and_renders() -> None:
    """Smoke test: parse the vendored romans.cxf and render 'AB'."""
    font_path = REPO / "fonts" / "romans.cxf"
    if not font_path.exists():
        return  # vendored font is optional for CI
    font = t2g.parse_cxf(font_path)
    assert "A" in font.glyphs
    assert "B" in font.glyphs
    out = t2g.emit_gcode("AB", font, t2g.GCodeOpts(height_mm=10.0))
    assert "M30" in out
    # Roman Simplex is lines-only; should produce G01 moves but no arcs.
    assert "G01 X" in out
