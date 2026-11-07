"""Tests for rollingmill.totem -- stacked rotary-axis spindle g-code emit."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from rollingmill import totem


def _opts(**overrides) -> totem.TotemOpts:
    base = dict(stock_radius=15.0, safe_radius=22.0, depth_step=1.0, angle_step_deg=30.0)
    base.update(overrides)
    return totem.TotemOpts(**base)


def _sec(shape: str, size: float, length: float) -> totem.Section:
    return totem.Section(shape=shape, size=size, length=length)


# ---------- header / footer order ----------


def test_emit_header_and_footer_order() -> None:
    out = totem.emit_gcode([_sec("cylinder", 10.0, 10.0)], _opts(spindle_rpm=8000))
    lines = out.splitlines()
    for token in ("G21", "G90", "G17", "S8000", "M03", "M05", "M30"):
        assert token in lines, f"missing {token}"
    assert lines.index("S8000") < lines.index("M03")
    assert lines.index("M03") < lines.index("M05") < lines.index("M30")


def test_workspace_emitted_in_header() -> None:
    out = totem.emit_gcode([_sec("cylinder", 10.0, 10.0)], _opts(workspace="G55"))
    assert "G55" in out.splitlines()


def test_return_home_emits_xy_zero_and_a_zero() -> None:
    out = totem.emit_gcode([_sec("cylinder", 8.0, 5.0)], _opts(return_home=True))
    lines = out.splitlines()
    assert "G00 X0.0 Y0.0" in lines
    assert any(l.startswith("G01 A0.0") for l in lines)


# ---------- safety: A moves preceded by safe-Z retract ----------


def test_every_a_move_is_preceded_by_z_safe_retract() -> None:
    """Every A rotation must be preceded by a Z retract to safe_radius,
    so the tool clears the workpiece while it spins. The initial A0 datum
    only needs the Z retract; subsequent in-loop A moves should also have
    a Y0 reset between the retract and the A move (Y is the off-axis
    coordinate now that A rotates around X)."""
    safe_r = 22.0
    safe_line = f"G00 Z{safe_r}"
    out = totem.emit_gcode([_sec("hex", 12.0, 10.0)], _opts(safe_radius=safe_r))
    lines = out.splitlines()
    a_moves = [i for i, l in enumerate(lines) if l.startswith("G01 A")]
    assert a_moves, "expected at least one A move"
    for i in a_moves:
        window = lines[max(0, i - 3) : i]
        assert safe_line in window, (
            f"A move at line {i} not preceded by {safe_line!r} within 3 lines (window={window!r})"
        )

    in_loop = [i for i in a_moves if lines[i - 1].startswith("G00 Y")]
    assert in_loop, "expected in-loop A moves to be preceded by Y0 reset"
    for i in in_loop:
        assert lines[i - 1] == "G00 Y0.0", (
            f"A move at line {i} not preceded by Y0 reset: {lines[i - 1]!r}"
        )
        assert lines[i - 2] == safe_line, (
            f"A move at line {i} not preceded by Z retract: {lines[i - 2]!r}"
        )


# ---------- polygon facet count ----------


def test_polygon_emits_one_face_block_per_side() -> None:
    """Hex (6 sides) at angle_step doesn't apply -- polygons step A by n
    discrete face positions."""
    out = totem.emit_gcode([_sec("hex", 10.0, 10.0)], _opts())
    a_angles = [float(l.split()[1][1:]) for l in out.splitlines() if l.startswith("G01 A")]
    # First A0 is the initial datum; then 6 A-moves for hex faces.
    # Last A is the final/return position if return_home; here disabled.
    distinct = sorted(set(a_angles))
    # Expect 0, 60, 120, 180, 240, 300 covered.
    expected = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
    for e in expected:
        assert any(abs(a - e) < 0.01 for a in distinct), f"missing A={e} in {distinct}"


def test_square_emits_four_face_angles() -> None:
    out = totem.emit_gcode([_sec("square", 8.0, 6.0)], _opts())
    a_angles = set()
    for l in out.splitlines():
        if l.startswith("G01 A"):
            a_angles.add(round(float(l.split()[1][1:]), 1))
    for e in (0.0, 90.0, 180.0, 270.0):
        assert e in a_angles


# ---------- round shape angular step ----------


def test_cylinder_uses_angle_step() -> None:
    out = totem.emit_gcode([_sec("cylinder", 10.0, 10.0)], _opts(angle_step_deg=45.0))
    a_angles = set()
    for l in out.splitlines():
        if l.startswith("G01 A"):
            a_angles.add(round(float(l.split()[1][1:]), 1))
    # 360/45 = 8 steps: 0, 45, 90, 135, 180, 225, 270, 315.
    for e in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
        assert e in a_angles


# ---------- depth passes ----------


def test_polygon_raster_is_inset_by_tool_radius_in_x() -> None:
    """Along the totem length (X), the tool tip must stop one tool radius
    short of the section ends so the cutter body just touches the
    boundary without overcutting into the neighbouring section. The Y
    (face-width) direction deliberately overshoots for corner clearing
    and is covered by the per-face extension tests below."""
    size = 10.0
    tool_d = 3.0
    section_length = 12.0
    out = totem.emit_gcode(
        [_sec("square", size, section_length)], _opts(tool_diameter=tool_d, depth_step=2.0)
    )
    tool_r = tool_d / 2.0
    x_max_expected = section_length - tool_r

    xs: list[float] = []
    for l in out.splitlines():
        if not (l.startswith("G01") or l.startswith("G00 X") or l.startswith("G00 Y")):
            continue
        for tok in l.split():
            if tok.startswith("X") and len(tok) > 1:
                try:
                    xs.append(float(tok[1:]))
                except ValueError:
                    pass
    cut_xs = [x for x in xs if abs(x) > 1e-9]
    assert max(cut_xs) == pytest.approx(x_max_expected, abs=0.01)
    assert min(cut_xs) == pytest.approx(tool_r, abs=0.01)


def test_polygon_cleanup_perimeter_pass_per_z() -> None:
    """Each polygon Z pass must end with a closed perimeter trace at the
    same Z (4 edges forming the inset rectangle) before the next motion
    (either the rapid to the next pass's start Y, or the next face's
    safe-Z retract — both are G00s)."""
    out = totem.emit_gcode([_sec("hex", 12.0, 10.0)], _opts(depth_step=2.0, tool_diameter=3.0))
    lines = out.splitlines()
    plunges = [i for i, l in enumerate(lines) if l.startswith("G01 Z")]
    assert len(plunges) > 0
    # For each plunge, verify the segment ending at the next G00 contains
    # the 4 perimeter edges as its last lines, in order: X to x_max along
    # y_min, Y to y_max along x_max, X to x_min along y_max, Y to y_min
    # closing the loop.
    side = 2.0 * 12.0 * math.sin(math.pi / 6)
    half_side = side / 2.0
    tool_r = 1.5
    # Inset rectangle: X along totem length [0+tool_r, 10-tool_r],
    # Y along face width [-half_side+tool_r, +half_side-tool_r].
    x_min = tool_r
    x_max = 10.0 - tool_r
    y_max = half_side - tool_r
    x_min_str = f"G01 X{_fmtp_for_test(x_min)}"
    x_max_str = f"G01 X{_fmtp_for_test(x_max)}"
    y_min_str = f"G01 Y{_fmtp_for_test(-y_max)}"
    y_max_str = f"G01 Y{_fmtp_for_test(y_max)}"
    for plunge_idx in plunges:
        next_g00 = next(i for i in range(plunge_idx + 1, len(lines)) if lines[i].startswith("G00"))
        segment = lines[plunge_idx:next_g00]
        assert segment[-4:] == [x_max_str, y_max_str, x_min_str, y_min_str], (
            f"unexpected perimeter at plunge {plunge_idx}: {segment[-5:]!r}"
        )


def _fmtp_for_test(v: float) -> str:
    """Mirror the formatting in totem._fmtp so test expectations match."""
    return totem._fmtp(v)


def _per_face_raster_y_bounds(out: str, n_faces: int) -> list[tuple[float, float]]:
    """Slice the g-code into one (min_y, max_y) tuple per face.

    Collects every Y value in the face's body. The corner-clearing
    extension is along Y (the face width direction, since the totem is
    along X and adjacent polygon faces meet at the Y=±half_side
    boundaries). The cleanup trace uses the inset (y_min, y_max) so it
    never extends past the raster's range; the observed min/max equals
    the raster's actual reach including any extension.
    """
    lines = out.splitlines()
    a_lines = [i for i, l in enumerate(lines) if l.startswith("G01 A")]
    # First "G01 A0.0" is the initial datum; the next n are per-face cuts.
    assert len(a_lines) >= n_faces + 1, a_lines
    face_starts = a_lines[1 : 1 + n_faces]
    face_ends = face_starts[1:] + [len(lines)]
    bounds: list[tuple[float, float]] = []
    for start, end in zip(face_starts, face_ends):
        ys: list[float] = []
        for line in lines[start:end]:
            if not (
                line.startswith("G00 X") or line.startswith("G00 Y") or line.startswith("G01 ")
            ):
                continue
            for tok in line.split():
                if tok.startswith("Y") and len(tok) > 1:
                    try:
                        v = float(tok[1:])
                    except ValueError:
                        continue
                    # Skip the safe-Y-reset (Y0.0) that precedes A moves.
                    if abs(v) < 1e-9:
                        continue
                    ys.append(v)
        bounds.append((min(ys), max(ys)))
    return bounds


def test_face_zero_extends_both_y_directions() -> None:
    """Face 0 must overshoot both y_min and y_max by the corner-clearing
    extension so the two corners shared with face 1 and face N-1 are
    pre-cleared. The extension is Z-aware: at the deepest pass (Z =
    apothem) the raster reaches ±(chord_half - tool_r) where chord_half
    is the cylindrical stock's half-chord at that Z; shallower passes
    extend less."""
    size = 12.0
    tool_d = 3.0
    opts = _opts(tool_diameter=tool_d, depth_step=2.0)
    out = totem.emit_gcode([_sec("hex", size, 10.0)], opts)
    tool_r = tool_d / 2.0
    apothem = size * math.cos(math.pi / 6)
    chord_half = math.sqrt(opts.stock_radius**2 - apothem**2)
    y_clear = chord_half - tool_r
    bounds = _per_face_raster_y_bounds(out, n_faces=6)
    y_min_face0, y_max_face0 = bounds[0]
    assert y_max_face0 == pytest.approx(y_clear, abs=0.01)
    assert y_min_face0 == pytest.approx(-y_clear, abs=0.01)


def test_middle_faces_extend_toward_next_face_only() -> None:
    """Faces 1..N-2 extend on +Y (toward the next-to-be-cut face under the
    standard RH-rule positive-A rotation, which appears CW from the
    operator's tailstock-side view) and stop at the inset y_min on the
    -Y side (where the previously-cut face already cleared the corner).
    The +Y extension is Z-aware: at the deepest pass it reaches
    chord_half - tool_r, where chord_half is the cylindrical stock's
    half-chord at Z=apothem."""
    size = 12.0
    tool_d = 3.0
    opts = _opts(tool_diameter=tool_d, depth_step=2.0)
    out = totem.emit_gcode([_sec("hex", size, 10.0)], opts)
    half_side = size * math.sin(math.pi / 6)
    tool_r = tool_d / 2.0
    y_max_inset = half_side - tool_r
    apothem = size * math.cos(math.pi / 6)
    chord_half = math.sqrt(opts.stock_radius**2 - apothem**2)
    y_clear = chord_half - tool_r
    bounds = _per_face_raster_y_bounds(out, n_faces=6)
    for i in (1, 2, 3, 4):  # middle faces (n=6, so 1..N-2 = 1..4)
        y_min_face, y_max_face = bounds[i]
        assert y_max_face == pytest.approx(y_clear, abs=0.01), (
            f"face {i}: y_max should reach chord_half-tool_r, got {y_max_face}"
        )
        assert y_min_face == pytest.approx(-y_max_inset, abs=0.01), (
            f"face {i}: y_min should be inset, got {y_min_face}"
        )


def test_last_face_extends_both_y_directions() -> None:
    """Face N-1 must overshoot both y_min and y_max like face 0. Face 0's
    -Y body and face N-2's +Y body each terminate *tangent* to the
    original cylinder boundary at the corners they share with face N-1
    — no chip engagement, leaving a residual ridge. Face N-1 needs its
    own chord_half extension on both sides to approach each corner from
    the opposite frame and sever the burr."""
    size = 12.0
    tool_d = 3.0
    opts = _opts(tool_diameter=tool_d, depth_step=2.0)
    out = totem.emit_gcode([_sec("hex", size, 10.0)], opts)
    tool_r = tool_d / 2.0
    apothem = size * math.cos(math.pi / 6)
    chord_half = math.sqrt(opts.stock_radius**2 - apothem**2)
    y_clear = chord_half - tool_r
    bounds = _per_face_raster_y_bounds(out, n_faces=6)
    y_min_face_last, y_max_face_last = bounds[5]
    assert y_max_face_last == pytest.approx(y_clear, abs=0.01)
    assert y_min_face_last == pytest.approx(-y_clear, abs=0.01)


def test_polygon_z_passes_terminate_at_apothem() -> None:
    """For a hex with size=12, the deepest Z must equal the apothem
    (= 12 * cos(pi/6) ~= 10.392)."""
    out = totem.emit_gcode([_sec("hex", 12.0, 10.0)], _opts(depth_step=0.5))
    z_cuts = [float(l.split()[1][1:]) for l in out.splitlines() if l.startswith("G01 Z")]
    expected_apothem = 12.0 * math.cos(math.pi / 6)
    assert min(z_cuts) == pytest.approx(expected_apothem, abs=0.001)


def test_cylinder_z_passes_terminate_at_radius() -> None:
    out = totem.emit_gcode([_sec("cylinder", 8.0, 10.0)], _opts(depth_step=0.5))
    z_cuts = [float(l.split()[1][1:]) for l in out.splitlines() if l.startswith("G01 Z")]
    assert min(z_cuts) == pytest.approx(8.0, abs=0.001)


def test_cylinder_one_safe_retract_per_angle() -> None:
    """Within one A angle the tool should sweep back-and-forth across Z
    passes and only retract to safe_radius once, before the next A
    rotation. So between any two consecutive A moves there must be
    exactly one G00 Z<safe> line, not one per Z pass."""
    safe_r = 22.0
    safe_line = f"G00 Z{_fmtp_for_test(safe_r)}"
    # depth_step small enough that multiple Z passes are needed per angle.
    out = totem.emit_gcode(
        [_sec("cylinder", 8.0, 10.0)],
        _opts(safe_radius=safe_r, depth_step=0.5, angle_step_deg=90.0),
    )
    lines = out.splitlines()
    a_indices = [i for i, l in enumerate(lines) if l.startswith("G01 A")]
    assert len(a_indices) >= 3, a_indices
    # Look at retract counts between successive in-section A moves
    # (skip the initial A0 datum window — it precedes the first cut).
    for prev, curr in zip(a_indices[1:-1], a_indices[2:]):
        segment = lines[prev:curr]
        retract_count = sum(1 for l in segment if l == safe_line)
        assert retract_count == 1, (
            f"expected 1 safe-Z retract between A moves at lines "
            f"{prev}..{curr}, got {retract_count}: {segment!r}"
        )


def test_cylinder_sweep_direction_alternates() -> None:
    """Across Z passes at one angle the X target should alternate
    between x_end and x_start (back-and-forth), rather than always
    cutting toward x_end with a rapid return between passes."""
    out = totem.emit_gcode(
        [_sec("cylinder", 8.0, 10.0)], _opts(depth_step=0.5, angle_step_deg=180.0)
    )
    lines = out.splitlines()
    # Find the first A move after the initial A0 datum and take the
    # block up to the next A move (or end). Within it, the G01 X… lines
    # are the per-pass cut targets.
    a_indices = [i for i, l in enumerate(lines) if l.startswith("G01 A")]
    # a_indices[0] is the datum; a_indices[1] is the first per-angle cut.
    start = a_indices[1]
    end = a_indices[2] if len(a_indices) > 2 else len(lines)
    cut_xs: list[float] = []
    for l in lines[start:end]:
        if l.startswith("G01 X"):
            cut_xs.append(float(l.split()[1][1:]))
    assert len(cut_xs) >= 2, f"expected multiple Z passes: {cut_xs!r}"
    # Pass 1 cuts toward x_end (=10), pass 2 toward x_start (=0), …
    expected = [10.0 if i % 2 == 0 else 0.0 for i in range(len(cut_xs))]
    for got, exp in zip(cut_xs, expected):
        assert got == pytest.approx(exp, abs=0.001), (
            f"sweep target sequence {cut_xs!r} does not alternate end/start"
        )


# ---------- section stacking along X ----------


def test_sections_stack_in_x() -> None:
    sections = [
        _sec("cylinder", 10.0, 10.0),
        _sec("cylinder", 8.0, 15.0),
    ]
    out = totem.emit_gcode(sections, _opts(x0=5.0))
    xs: set[float] = set()
    for l in out.splitlines():
        for tok in l.split():
            if tok.startswith("X") and len(tok) > 1:
                try:
                    xs.add(round(float(tok[1:]), 3))
                except ValueError:
                    pass
    # First section spans x=[5, 15], second [15, 30]; both endpoints emitted.
    assert 5.0 in xs
    assert 15.0 in xs
    assert 30.0 in xs


# ---------- shape aliases ----------


def test_cube_aliases_to_square() -> None:
    out = totem.emit_gcode([_sec("cube", 8.0, 6.0)], _opts())
    a_angles = {
        round(float(l.split()[1][1:]), 1) for l in out.splitlines() if l.startswith("G01 A")
    }
    for e in (0.0, 90.0, 180.0, 270.0):
        assert e in a_angles


def test_torus_aliases_to_donut() -> None:
    # Should not raise; just verify the section runs and emits.
    out = totem.emit_gcode([_sec("torus", 10.0, 20.0)], _opts(angle_step_deg=90.0))
    assert "M30" in out.splitlines()


# ---------- donut: arcs instead of polyline ----------


def test_donut_uses_g18_arcs_not_polyline() -> None:
    """Donut sections must emit G02/G03 arcs in the G18 (XZ) plane, not
    many tiny G01 X..Z.. polyline segments — the Roland MDX-40A's
    block-processing rate can't keep motion fluid through a 40-segment
    sine approximation."""
    out = totem.emit_gcode([_sec("donut", 10.0, 20.0)], _opts(angle_step_deg=90.0, depth_step=2.0))
    lines = out.splitlines()
    arc_lines = [l for l in lines if l.startswith("G02") or l.startswith("G03")]
    polyline_xz = [l for l in lines if l.startswith("G01 X") and " Z" in l]
    assert arc_lines, "expected donut to use G02/G03 arcs"
    assert not polyline_xz, (
        f"expected no G01 X..Z.. polyline waypoints, found {len(polyline_xz)}: {polyline_xz[:3]!r}"
    )
    assert "G18" in lines, "donut must switch to XZ arc plane"
    # G17 appears in the header; after the donut section it must be
    # restored so subsequent sections (or end-of-program) see XY mode.
    last_g18 = max(i for i, l in enumerate(lines) if l == "G18")
    next_g17 = next(
        (i for i, l in enumerate(lines[last_g18:], start=last_g18) if l == "G17"),
        None,
    )
    assert next_g17 is not None, "G18 must be balanced by a later G17"


def test_donut_arc_geometry_matches_sine_at_three_points() -> None:
    """The arc through each pass interpolates the half-sine bulge at three
    points: the two crossings (x1, z_pass) and (x2, z_pass) where the
    sine equals z_pass, plus the peak (length/2, size). Verify the arc
    endpoints and centre offsets land where the math expects."""
    size = 10.0
    length = 20.0
    out = totem.emit_gcode(
        [_sec("donut", size, length)], _opts(angle_step_deg=90.0, depth_step=2.0)
    )
    lines = out.splitlines()
    # Find a representative rising-arc/falling-arc pair: a G02 ending at
    # peak (Z=size) followed immediately by a G02 ending at z_pass.
    pairs = []
    for i, l in enumerate(lines[:-1]):
        if not l.startswith("G02 "):
            continue
        nxt = lines[i + 1]
        if not nxt.startswith("G02 "):
            continue
        if f"Z{_fmtp_for_test(size)}" not in l:
            continue
        pairs.append((l, nxt))
    assert pairs, "expected at least one rising+falling G02 arc pair"
    rise, fall = pairs[0]
    # Falling arc's K offset should be (zc - peak) and its endpoint Z is
    # z_pass. Rising arc's endpoint is the peak; its I offset is dx.
    # Extract Z from falling arc:
    z_pass = float([tok[1:] for tok in fall.split() if tok.startswith("Z")][0])
    base = size / 2
    bulge = size - base
    x1 = (length / math.pi) * math.asin((z_pass - base) / bulge)
    dx = length / 2 - x1
    zc = ((size * size - z_pass * z_pass) - dx * dx) / (2 * (size - z_pass))
    # Rising-arc tokens: G02 X{mid} Z{peak} I{dx} K{zc - z_pass}
    rise_toks = {tok[0]: float(tok[1:]) for tok in rise.split()[1:]}
    # G02 sweeps -X from x2 to the midpoint, so the centre is at -dx.
    assert rise_toks["I"] == pytest.approx(-dx, abs=0.01)
    assert rise_toks["K"] == pytest.approx(zc - z_pass, abs=0.01)
    # Falling-arc tokens: G02 X{x_start+x2} Z{z_pass} I0 K{zc - peak}
    fall_toks = {tok[0]: float(tok[1:]) for tok in fall.split()[1:]}
    assert fall_toks["I"] == pytest.approx(0.0, abs=0.01)
    assert fall_toks["K"] == pytest.approx(zc - size, abs=0.01)


def test_donut_alternates_g02_g03_with_sweep_direction() -> None:
    """Left-to-right sweeps go CW in the G18 frame (G02); right-to-left
    sweeps go CCW (G03). Within one theta block, successive arc-using
    passes alternate sweep direction and so alternate G02/G03.
    (sweep_dir resets to +1 at each new theta, so the alternation only
    holds inside a single theta's pass list.)"""
    out = totem.emit_gcode([_sec("donut", 10.0, 20.0)], _opts(angle_step_deg=360.0, depth_step=2.0))
    # Single theta -> all rising-to-peak arcs are in one block.
    rises = [
        l
        for l in out.splitlines()
        if (l.startswith("G02") or l.startswith("G03")) and " Z10.0 " in (l + " ")
    ]
    assert len(rises) >= 2, f"expected multiple arc-using passes: {rises!r}"
    codes = [l.split()[0] for l in rises]
    for a, b in zip(codes, codes[1:]):
        assert a != b, f"expected alternating G02/G03, got {codes!r}"


# ---------- validation ----------


def test_rejects_empty_sections() -> None:
    with pytest.raises(ValueError):
        totem.emit_gcode([], _opts())


def test_rejects_size_exceeding_stock_radius() -> None:
    with pytest.raises(ValueError):
        totem.emit_gcode([_sec("cylinder", 20.0, 10.0)], _opts(stock_radius=15.0))


def test_rejects_unknown_shape() -> None:
    with pytest.raises(ValueError):
        totem.emit_gcode([_sec("dodecahedron", 10.0, 10.0)], _opts())


def test_rejects_safe_radius_below_stock() -> None:
    with pytest.raises(ValueError):
        totem.emit_gcode([_sec("cylinder", 10.0, 10.0)], _opts(stock_radius=15.0, safe_radius=10.0))


def test_rejects_non_positive_dims() -> None:
    with pytest.raises(ValueError):
        totem.emit_gcode([_sec("cylinder", 0.0, 10.0)], _opts())
    with pytest.raises(ValueError):
        totem.emit_gcode([_sec("cylinder", 10.0, 0.0)], _opts())


# ---------- shape radius math ----------


def test_polygon_apothem_at_face_center() -> None:
    """For a regular n-gon with circumscribed radius R, r(theta=0) equals the
    apothem R*cos(pi/n) (because the face is centred on theta=0)."""
    for shape, n in totem.POLYGON_SIDES.items():
        r0 = totem._r_polygon(n, 10.0, 0.0)
        assert r0 == pytest.approx(10.0 * math.cos(math.pi / n))


def test_polygon_corner_radius_equals_size() -> None:
    """At theta = pi/n the polygon's corner sits at the full circumscribed
    radius."""
    for shape, n in totem.POLYGON_SIDES.items():
        r_corner = totem._r_polygon(n, 10.0, math.pi / n)
        assert r_corner == pytest.approx(10.0, abs=1e-9)


def test_ellipse_axes() -> None:
    r0 = totem._r_ellipse(10.0, 0.0, 0.6)
    r_quarter = totem._r_ellipse(10.0, math.pi / 2, 0.6)
    assert r0 == pytest.approx(10.0)
    assert r_quarter == pytest.approx(6.0)


# ---------- CLI ----------


def test_cli_smoke(tmp_path: Path) -> None:
    out_path = tmp_path / "t.gcode"
    rc = totem.main(
        [
            "--out",
            str(out_path),
            "--stock-radius",
            "15",
            "--safe-radius",
            "22",
            "--shape",
            "cylinder:10:10",
            "--shape",
            "hex:12:15",
            "--angle-step",
            "30",
        ]
    )
    assert rc == 0
    text = out_path.read_text()
    assert "M30" in text
    assert "(Section: cylinder" in text
    assert "(Section: hex" in text


def test_cli_requires_at_least_one_shape(tmp_path: Path) -> None:
    out_path = tmp_path / "t.gcode"
    with pytest.raises(SystemExit):
        totem.main(["--out", str(out_path)])


def test_cli_rejects_malformed_shape(tmp_path: Path) -> None:
    out_path = tmp_path / "t.gcode"
    with pytest.raises(SystemExit):
        totem.main(
            [
                "--out",
                str(out_path),
                "--shape",
                "cylinder:10",  # missing length
            ]
        )


def test_cli_accepts_semicolon_and_comma_separators(tmp_path: Path) -> None:
    out_path = tmp_path / "t.gcode"
    rc = totem.main(
        [
            "--out",
            str(out_path),
            "--shape",
            "cylinder;10;10",
            "--shape",
            "hex,12,10",
            "--angle-step",
            "60",
        ]
    )
    assert rc == 0


def test_cli_tool_supplies_defaults_and_explicit_flag_overrides(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """--tool fills in spindle from the library; --feed still overrides."""
    monkeypatch.setenv("HOME", str(tmp_path))
    out_path = tmp_path / "t.gcode"
    rc = totem.main(
        [
            "--out",
            str(out_path),
            "--shape",
            "cylinder:10:10",
            "--tool",
            "end_6mm",  # seeded by tools.load() on first call
            "--feed",
            "999",  # explicit override wins over tool default
        ]
    )
    assert rc == 0
    text = out_path.read_text()
    # end_6mm seeds spindle_rpm=12000; not overridden -> from tool.
    assert "S12000" in text
    # Explicit --feed 999 wins over tool's feed_mm_min=600.
    assert "F999" in text
    assert "F600" not in text
