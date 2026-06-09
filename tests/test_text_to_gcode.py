"""Tests for rollingmill.text_to_gcode — pyhershey-backed g-code emit."""
from __future__ import annotations

from rollingmill import text_to_gcode as t2g


# ---------- font discovery ----------

def test_available_fonts_includes_roman_simplex() -> None:
    fonts = t2g.available_fonts()
    assert "roman_simplex" in fonts
    assert fonts == sorted(fonts)


def test_default_font_is_in_available_list() -> None:
    assert t2g.DEFAULT_FONT in t2g.available_fonts()


# ---------- emit ----------

def test_emit_basic_renders_strokes() -> None:
    out = t2g.emit_gcode("I", "roman_simplex", t2g.GCodeOpts(height_mm=9.0))
    lines = out.strip().splitlines()
    assert any(l.startswith("G21") for l in lines)
    assert any(l.startswith("G90") for l in lines)
    assert "M30" in lines
    # At least one plunge for the single stroke; at least one final lift.
    assert any(l.startswith("G01 Z") for l in lines)
    assert sum(1 for l in lines if l.startswith("G00 Z")) >= 2


def test_emit_handles_space_and_unsupported() -> None:
    # Space advances the cursor without strokes; non-printable should not crash.
    out = t2g.emit_gcode(" \x01A", "roman_simplex", t2g.GCodeOpts(height_mm=9.0))
    assert "M30" in out
    # 'A' is supported and should produce a plunge.
    assert any(l.startswith("G01 Z") for l in out.splitlines())


def test_emit_newline_advances_y_and_resets_x() -> None:
    out = t2g.emit_gcode("A\nB", "roman_simplex", t2g.GCodeOpts(height_mm=10.0, x0=0.0, y0=0.0))
    # After newline, base_y drops by 1.5 × height; some Y values should be negative.
    ys: list[float] = []
    for line in out.splitlines():
        for tok in line.split():
            if tok.startswith("Y") and len(tok) > 1:
                try:
                    ys.append(float(tok[1:]))
                except ValueError:
                    pass
    assert any(y < -1.0 for y in ys), "expected negative Y for second line"


def test_emit_spindle_on_and_off_order() -> None:
    out = t2g.emit_gcode("I", "roman_simplex", t2g.GCodeOpts(height_mm=9.0, spindle_rpm=8000))
    lines = out.splitlines()
    assert "S8000" in lines
    assert "M03" in lines
    assert lines.index("S8000") < lines.index("M03")
    assert lines.index("M03") < lines.index("G00 Z2.0")
    assert lines.index("M03") < lines.index("M05") < lines.index("M30")


def test_emit_workspace_in_header() -> None:
    out = t2g.emit_gcode("I", "roman_simplex", t2g.GCodeOpts(height_mm=9.0, workspace="G54"))
    lines = out.splitlines()
    assert "G54" in lines
    assert lines.index("G54") < lines.index("G00 Z2.0")
    assert not any(l.strip() == "G94" for l in lines)


def test_emit_no_workspace_by_default() -> None:
    out = t2g.emit_gcode("I", "roman_simplex", t2g.GCodeOpts(height_mm=9.0))
    wcs = {"G54", "G55", "G56", "G57", "G58", "G59"}
    assert not any(l.strip() in wcs for l in out.splitlines())


def test_emit_return_home_appended() -> None:
    out = t2g.emit_gcode("I", "roman_simplex",
                         t2g.GCodeOpts(height_mm=9.0, return_home=True))
    lines = out.splitlines()
    assert "G00 X0.0 Y0.0" in lines
    assert lines.index("G00 X0.0 Y0.0") > lines.index("M03")
    assert lines.index("G00 X0.0 Y0.0") < lines.index("M30")


def test_emit_uses_per_glyph_advance_width() -> None:
    """Consecutive letters should be offset by `advance_width * scale`, not by
    a fixed global LetterSpacing. Verify by checking that the start of the
    second glyph's first stroke is at the expected cursor position."""
    from pyhershey import glyph_factory as gf
    g_P = gf.from_ascii("P", "roman_simplex")
    g_a = gf.from_ascii("a", "roman_simplex")
    cap = t2g._cap_height("roman_simplex")
    scale = 25.0 / cap
    expected_a_start_x = g_P.advance_width * scale + g_a.segments[0][0][0] * scale

    out = t2g.emit_gcode("Pa", "roman_simplex", t2g.GCodeOpts(height_mm=25.0))
    g00_lines = [l for l in out.splitlines() if l.startswith("G00 X")]
    # Take the first G00 X after the second-glyph boundary (heuristic: 'a's
    # first stroke starts further right than any P stroke).
    xs = []
    for line in g00_lines:
        for tok in line.split():
            if tok.startswith("X"):
                xs.append(float(tok[1:]))
    # All X positions in the 'a' glyph are >= P's advance — find the min that is.
    a_xs = [x for x in xs if x >= g_P.advance_width * scale - 0.01]
    assert a_xs, "expected at least one G00 in the 'a' glyph region"
    assert abs(min(a_xs) - expected_a_start_x) < 0.05


def test_letter_spacing_adds_gap() -> None:
    """`letter_spacing_mm` should add a fixed mm gap after each glyph."""
    out_tight = t2g.emit_gcode("II", "roman_simplex",
                               t2g.GCodeOpts(height_mm=10.0, letter_spacing_mm=0.0))
    out_loose = t2g.emit_gcode("II", "roman_simplex",
                               t2g.GCodeOpts(height_mm=10.0, letter_spacing_mm=5.0))

    def max_x(s: str) -> float:
        xs = []
        for line in s.splitlines():
            for tok in line.split():
                if tok.startswith("X") and len(tok) > 1:
                    try:
                        xs.append(float(tok[1:]))
                    except ValueError:
                        pass
        return max(xs)

    assert max_x(out_loose) > max_x(out_tight) + 4.0


def test_list_fonts_cli(capsys) -> None:
    rc = t2g.main(["--list-fonts"])
    assert rc == 0
    captured = capsys.readouterr().out.strip().splitlines()
    assert "roman_simplex" in captured
