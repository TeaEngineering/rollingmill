"""Tests for rollingmill.tools — host-side cutting-tool library."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from rollingmill import tools


def _ball(**overrides) -> tools.Tool:
    base = dict(
        id="b",
        name="ball",
        type="ball_nose",
        diameter_mm=3.0,
        length_mm=38.0,
        flute_length_mm=12.0,
        shank_diameter_mm=3.0,
        flutes=2,
        feed_mm_min=400.0,
        plunge_feed_mm_min=80.0,
        spindle_rpm=14000,
    )
    base.update(overrides)
    return tools.Tool(**base)


# ---------- default_path / HOME ----------


def test_default_path_honours_home(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    assert tools.default_path() == tmp_path / ".rollingmill" / "tools.json"


# ---------- load / save round-trip ----------


def test_save_then_load_round_trip(tmp_path: Path) -> None:
    p = tmp_path / "tools.json"
    seed = [
        _ball(),
        tools.Tool(
            id="vb",
            name="vbit",
            type="v_bit",
            diameter_mm=6.0,
            length_mm=38.0,
            flute_length_mm=10.0,
            shank_diameter_mm=6.0,
            flutes=2,
            half_angle_deg=30.0,
            feed_mm_min=500,
            plunge_feed_mm_min=80,
            spindle_rpm=14000,
        ),
    ]
    tools.save(seed, p)
    loaded = tools.load(p)
    assert [t.id for t in loaded] == ["b", "vb"]
    assert loaded[1].half_angle_deg == 30.0


def test_load_missing_seeds_file(tmp_path: Path) -> None:
    p = tmp_path / "fresh" / "tools.json"
    assert not p.exists()
    out = tools.load(p)
    assert p.exists()
    ids = {t.id for t in out}
    assert ids == {t.id for t in tools.SEED_TOOLS}
    # The written file should be valid JSON.
    raw = json.loads(p.read_text())
    assert isinstance(raw, list) and len(raw) == len(tools.SEED_TOOLS)


def test_load_rejects_non_list(tmp_path: Path) -> None:
    p = tmp_path / "tools.json"
    p.write_text('{"id": "x"}')
    with pytest.raises(ValueError, match="expected a JSON list"):
        tools.load(p)


def test_to_dict_omits_none_fields() -> None:
    t = _ball()
    d = t.to_dict()
    assert "half_angle_deg" not in d
    assert "corner_radius_mm" not in d
    assert "firmware_slot" not in d


# ---------- find ----------


def test_find_by_id() -> None:
    seed = tools.SEED_TOOLS
    assert tools.find(seed, "ball_3mm").id == "ball_3mm"


def test_find_by_name_fallback() -> None:
    seed = tools.SEED_TOOLS
    assert tools.find(seed, "3mm 2-flute ball nose").id == "ball_3mm"


def test_find_missing_raises_with_known_list() -> None:
    seed = tools.SEED_TOOLS
    with pytest.raises(KeyError, match="known:"):
        tools.find(seed, "no_such_tool")


# ---------- add / remove ----------


def test_add_rejects_duplicate_id() -> None:
    seed = list(tools.SEED_TOOLS)
    with pytest.raises(ValueError, match="already exists"):
        tools.add(seed, _ball(id="ball_3mm"))


def test_remove_returns_smaller_list() -> None:
    seed = list(tools.SEED_TOOLS)
    out = tools.remove(seed, "ball_3mm")
    assert len(out) == len(seed) - 1
    assert all(t.id != "ball_3mm" for t in out)


def test_remove_missing_raises() -> None:
    with pytest.raises(KeyError):
        tools.remove(list(tools.SEED_TOOLS), "no_such")


# ---------- validation ----------


def test_vbit_requires_half_angle() -> None:
    with pytest.raises(ValueError, match="v_bit requires half_angle_deg"):
        tools.Tool(
            id="v",
            name="v",
            type="v_bit",
            diameter_mm=6.0,
            length_mm=38,
            flute_length_mm=10,
            shank_diameter_mm=6.0,
            flutes=2,
            feed_mm_min=500,
            plunge_feed_mm_min=80,
            spindle_rpm=14000,
        )


def test_bullnose_requires_corner_radius() -> None:
    with pytest.raises(ValueError, match="bull_nose requires corner_radius_mm"):
        tools.Tool(
            id="bn",
            name="bn",
            type="bull_nose",
            diameter_mm=6.0,
            length_mm=38,
            flute_length_mm=10,
            shank_diameter_mm=6.0,
            flutes=2,
            feed_mm_min=500,
            plunge_feed_mm_min=80,
            spindle_rpm=14000,
        )


def test_unknown_type_raises() -> None:
    with pytest.raises(ValueError, match="type"):
        _ball(type="laser")  # type: ignore[arg-type]


def test_zero_diameter_raises() -> None:
    with pytest.raises(ValueError, match="diameter_mm"):
        _ball(diameter_mm=0.0)


def test_firmware_slot_range() -> None:
    with pytest.raises(ValueError, match="firmware_slot"):
        _ball(firmware_slot=9)


def test_half_angle_range() -> None:
    with pytest.raises(ValueError, match="half_angle_deg"):
        tools.Tool(
            id="v",
            name="v",
            type="v_bit",
            diameter_mm=6.0,
            length_mm=38,
            flute_length_mm=10,
            shank_diameter_mm=6.0,
            flutes=2,
            half_angle_deg=0.0,
            feed_mm_min=500,
            plunge_feed_mm_min=80,
            spindle_rpm=14000,
        )


# ---------- footprint shapes (depend on numpy) ----------


def test_endmill_footprint_is_flat_disc() -> None:
    np = pytest.importorskip("numpy")
    t = tools.Tool(
        id="e",
        name="e",
        type="end_mill",
        diameter_mm=4.0,
        length_mm=30,
        flute_length_mm=15,
        shank_diameter_mm=4.0,
        flutes=2,
        feed_mm_min=600,
        plunge_feed_mm_min=150,
        spindle_rpm=12000,
    )
    fp = t.footprint(pitch_mm=0.5)
    # Centre: 0; corner: inf
    cy = cx = fp.shape[0] // 2
    assert fp[cy, cx] == 0.0
    assert math.isinf(fp[0, 0])
    # Inside-the-disc cells are all zero.
    inside = fp != np.inf
    assert np.all(fp[inside] == 0.0)


def test_ballnose_footprint_dome() -> None:
    pytest.importorskip("numpy")
    t = _ball()  # 3mm dia → r=1.5
    fp = t.footprint(pitch_mm=0.1)
    cy = cx = fp.shape[0] // 2
    assert fp[cy, cx] == pytest.approx(0.0)
    # At an off-centre pixel inside the disc, offset should equal
    # r - sqrt(r^2 - d^2)
    r = 1.5
    d = 0.5
    px = int(round(d / 0.1))
    expected = r - math.sqrt(r * r - d * d)
    assert fp[cy, cx + px] == pytest.approx(expected, abs=0.02)
    # Outside the disc is inf.
    assert math.isinf(fp[0, 0])


def test_vbit_footprint_cone() -> None:
    pytest.importorskip("numpy")
    t = tools.Tool(
        id="v",
        name="v",
        type="v_bit",
        diameter_mm=6.0,
        length_mm=38,
        flute_length_mm=10,
        shank_diameter_mm=6.0,
        flutes=2,
        half_angle_deg=45.0,
        feed_mm_min=500,
        plunge_feed_mm_min=80,
        spindle_rpm=14000,
    )
    fp = t.footprint(pitch_mm=0.5)
    cy = cx = fp.shape[0] // 2
    # 45° half-angle → offset == radial distance.
    d = 1.0
    px = int(round(d / 0.5))
    assert fp[cy, cx + px] == pytest.approx(d, abs=0.01)
    assert fp[cy, cx] == pytest.approx(0.0)


def test_bullnose_footprint_flat_centre_round_shoulder() -> None:
    pytest.importorskip("numpy")
    t = tools.Tool(
        id="bn",
        name="bn",
        type="bull_nose",
        diameter_mm=6.0,
        length_mm=38,
        flute_length_mm=15,
        shank_diameter_mm=6.0,
        flutes=2,
        corner_radius_mm=1.0,
        feed_mm_min=500,
        plunge_feed_mm_min=80,
        spindle_rpm=14000,
    )
    fp = t.footprint(pitch_mm=0.1)
    cy = cx = fp.shape[0] // 2
    # Centre is flat (within flat radius = 3 - 1 = 2 mm).
    assert fp[cy, cx] == 0.0
    # At d ≈ 1.5 mm we're inside the flat region (flat_r=2.0): still 0.
    px = int(round(1.5 / 0.1))
    assert fp[cy, cx + px] == pytest.approx(0.0, abs=1e-9)
    # At d ≈ 2.5 mm we're in the torus (flat_r=2, c=1).
    px = int(round(2.5 / 0.1))
    d_torus = 0.5  # 2.5 - 2.0
    expected = 1.0 - math.sqrt(1.0 - d_torus * d_torus)
    assert fp[cy, cx + px] == pytest.approx(expected, abs=0.02)
