import math

import pytest

from rollingmill.cutjob import parse_nc_blocks
from rollingmill.estimate import EstimateOptions, estimate_time, format_duration


def est(text, **kw):
    return estimate_time(parse_nc_blocks(text.encode()), EstimateOptions(**kw))


def test_linear_cut_at_feed():
    e = est("G21\nG90\nG01 X100 F100\n")
    assert e.cutting_s == pytest.approx(60.0)
    assert e.path_mm == pytest.approx(100)


def test_feed_override_and_inches():
    assert est("G01 X100 F100\n", feed_override_pct=50).cutting_s == pytest.approx(120.0)
    assert est("G20\nG01 X1 F1\n").cutting_s == pytest.approx(60.0)


def test_rapid_from_origin_uses_axis_speeds():
    e = est("G00 X30 Y40 Z10\n", rapid_xy=3000, rapid_z=600)
    assert e.rapid_s == pytest.approx(max(50 / 3000, 10 / 600) * 60)


def test_incremental_and_modal_motion():
    e = est("G91\nG01 X10 F600\nX10\n")
    assert e.path_mm == pytest.approx(20)
    assert e.cutting_s == pytest.approx(2.0)


def test_full_circle_and_half_arc():
    full = est("G01 X10 F600\nG03 X10 Y0 I-10 J0\n")
    assert full.path_mm == pytest.approx(10 + 2 * math.pi * 10)
    half = est("G01 X10 F600\nG02 X-10 Y0 I-10 J0\n")
    assert half.path_mm == pytest.approx(10 + math.pi * 10)


def test_radius_arc_matches_center_arc():
    a = est("G01 X10 F600\nG03 X0 Y10 I-10 J0\n")
    b = est("G01 X10 F600\nG03 X0 Y10 R10\n")
    assert b.path_mm == pytest.approx(a.path_mm)


def test_min_op_floor_and_dwell():
    e = est("G01 X0.001 F600\nG04 P1\n")
    assert e.cutting_s == pytest.approx(0.25)
    assert e.dwell_s == pytest.approx(1.0)


def test_comments_and_warnings():
    e = est("(G01 X999)\nG01 X10 (inline) F100\nG01 X20 F0\nA5\nG01 A1\n")
    assert e.path_mm == pytest.approx(20)
    assert any("no feed" in w for w in est("G01 X5\n").warnings)
    assert any("before any G0" in w for w in est("X5\n").warnings)


def test_format_duration():
    assert format_duration(5) == "5s"
    assert format_duration(125) == "2m 05s"
    assert format_duration(3725) == "1h 02m 05s"


def test_g92_is_a_command_not_a_move():
    e = est("G91\nG92X0Y0Z0\nG01 X10 F600\n")
    assert e.warnings == []
    assert e.command_s == pytest.approx(0.25)
    assert e.moves == 1
    assert e.total_s == pytest.approx(0.25 + 1.0)
    # G92 re-labels the current position without moving
    assert est("G92 X50\nG01 X60 F600\n").path_mm == pytest.approx(10)
