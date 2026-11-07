"""Tests for rollingmill.relief3D — greyscale-image → 3D relief g-code."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

# All relief3D tests need the optional 'relief' extra.
np = pytest.importorskip("numpy")
PIL = pytest.importorskip("PIL")
pytest.importorskip("scipy")

from PIL import Image  # noqa: E402

from rollingmill import relief3D, tools  # noqa: E402


@pytest.fixture(autouse=True)
def _isolated_tools_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Point ~/.rollingmill/tools.json at a per-test tmp dir."""
    monkeypatch.setenv("HOME", str(tmp_path))
    # Seed the library; relief3D.emit_gcode() will tools.load() it.
    tools.save(tools.SEED_TOOLS)
    return tmp_path


def _write_image(path: Path, arr_uint8) -> Path:
    Image.fromarray(arr_uint8, mode="L").save(path)
    return path


def _flat_image(path: Path, value: int, size: int = 16) -> Path:
    return _write_image(path, np.full((size, size), value, dtype=np.uint8))


def _opts(tmp_path: Path, image: Path, **overrides) -> relief3D.ReliefOpts:
    base = dict(
        image=image,
        out=tmp_path / "out.gcode",
        tool="ball_3mm",
        stock_width_mm=10.0,
        stock_height_mm=10.0,
        z_max=0.0,
        z_min=-2.0,
        pixel_pitch_mm=0.5,
        stepover_mm=1.0,
        blur_px=0.0,  # off for deterministic depths
    )
    base.update(overrides)
    return relief3D.ReliefOpts(**base)


def _extract_z(gcode: str) -> list[float]:
    out: list[float] = []
    for tok in re.findall(r"Z(-?\d+(?:\.\d+)?)", gcode):
        out.append(float(tok))
    return out


# ---------- heightmap basics ----------


def test_load_heightmap_white_maps_to_zmax(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "white.png", 255)
    opts = _opts(tmp_path, img)
    h = relief3D.load_heightmap(opts, pitch_mm=0.5)
    assert h.shape == (20, 20)
    assert np.allclose(h, opts.z_max)


def test_load_heightmap_black_maps_to_zmin(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "black.png", 0)
    opts = _opts(tmp_path, img)
    h = relief3D.load_heightmap(opts, pitch_mm=0.5)
    assert np.allclose(h, opts.z_min)


def test_invert_flips_mapping(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "white.png", 255)
    opts = _opts(tmp_path, img, invert=True)
    h = relief3D.load_heightmap(opts, pitch_mm=0.5)
    assert np.allclose(h, opts.z_min)


# ---------- erosion ----------


def test_erosion_on_flat_white_is_flat(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "white.png", 255)
    opts = _opts(tmp_path, img)
    tool = tools.find(tools.SEED_TOOLS, "ball_3mm")
    h = relief3D.load_heightmap(opts, pitch_mm=0.5)
    r = relief3D.erode_by_tool(h, tool, pitch_mm=0.5)
    assert np.allclose(r, opts.z_max)


def test_erosion_widens_dark_region_by_tool(tmp_path: Path) -> None:
    """A wide-enough black square lets the ball nose reach z_min at its
    centre; far outside the dark region the reach stays at z_max."""
    img_arr = np.full((20, 20), 255, dtype=np.uint8)
    # 5x5 mm dark square at centre (pitch=0.5 → 10x10 cells); a 3mm
    # ball nose (radius 1.5mm) easily fits with room to spare.
    img_arr[5:15, 5:15] = 0
    img = _write_image(tmp_path / "square.png", img_arr)
    opts = _opts(tmp_path, img)
    tool = tools.find(tools.SEED_TOOLS, "ball_3mm")
    h = relief3D.load_heightmap(opts, pitch_mm=0.5)
    r = relief3D.erode_by_tool(h, tool, pitch_mm=0.5)
    # Deepest reach == z_min (somewhere in the dark core).
    assert pytest.approx(r.min(), abs=1e-6) == opts.z_min
    # Far away from the dark region the surface is still uncut.
    assert pytest.approx(r[0, 0], abs=1e-6) == opts.z_max


def test_erosion_in_narrow_pocket_is_limited_by_tool_radius(tmp_path: Path) -> None:
    """A pocket too narrow for the tool clamps the reach above z_min."""
    img_arr = np.full((20, 20), 255, dtype=np.uint8)
    img_arr[8:12, 8:12] = 0  # 2x2 mm dark square — narrower than 3mm bit
    img = _write_image(tmp_path / "narrow.png", img_arr)
    opts = _opts(tmp_path, img)
    tool = tools.find(tools.SEED_TOOLS, "ball_3mm")
    h = relief3D.load_heightmap(opts, pitch_mm=0.5)
    r = relief3D.erode_by_tool(h, tool, pitch_mm=0.5)
    # Cannot reach z_min: dome of the ball nose hits the white shoulders.
    assert r.min() > opts.z_min


# ---------- raster path ----------


def test_raster_zigzag_alternates_direction(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 128)
    opts = _opts(
        tmp_path, img, stepover_mm=2.0, stock_width_mm=5.0, stock_height_mm=5.0, pixel_pitch_mm=1.0
    )
    h = relief3D.load_heightmap(opts, pitch_mm=1.0)
    poly = relief3D.raster_path(h, opts, pitch_mm=1.0, stepover_mm=2.0)
    # First row goes 0 → x_max; second row x_max → 0.
    xs = [p[0] for p in poly]
    nx = h.shape[1]
    assert xs[0] == pytest.approx(opts.x0)
    assert xs[nx - 1] == pytest.approx(opts.x0 + (nx - 1) * 1.0)
    assert xs[nx] == pytest.approx(xs[nx - 1])  # Y step keeps X
    assert xs[2 * nx - 1] == pytest.approx(opts.x0)


def test_raster_oneway_always_increases_x(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 128)
    opts = _opts(
        tmp_path,
        img,
        stepover_mm=2.0,
        stock_width_mm=5.0,
        stock_height_mm=5.0,
        pixel_pitch_mm=1.0,
        direction="oneway",
    )
    h = relief3D.load_heightmap(opts, pitch_mm=1.0)
    poly = relief3D.raster_path(h, opts, pitch_mm=1.0, stepover_mm=2.0)
    nx = h.shape[1]
    # Each row begins at x0.
    for row in range(0, len(poly), nx):
        assert poly[row][0] == pytest.approx(opts.x0)


# ---------- header / footer / flags in g-code ----------


def test_emit_header_and_footer(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 200)
    out = relief3D.emit_gcode(_opts(tmp_path, img))
    lines = out.splitlines()
    for token in ("G21", "G90", "G17", "M03", "M05", "M30"):
        assert token in lines, f"missing {token} in:\n{out}"
    assert lines[-2] == "M05"
    assert lines[-1] == "M30"


def test_workspace_present_in_header(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 200)
    out = relief3D.emit_gcode(_opts(tmp_path, img, workspace="G55"))
    assert "G55" in out.splitlines()


def test_return_home_appends_before_m05(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 200)
    out = relief3D.emit_gcode(_opts(tmp_path, img, return_home=True))
    lines = out.splitlines()
    idx_m05 = lines.index("M05")
    assert lines[idx_m05 - 1] == "G00 X0.0 Y0.0"


def test_white_image_emits_zsafe_only(tmp_path: Path) -> None:
    """A pure-white image carves nothing below z_max=0; all Zs should be
    either the rapid z_safe or the surface 0.0."""
    img = _flat_image(tmp_path / "white.png", 255)
    out = relief3D.emit_gcode(_opts(tmp_path, img))
    zs = _extract_z(out)
    assert zs, "expected at least one Z word"
    # Every Z is 0 (surface) or z_safe (rapid).
    for z in zs:
        assert z in (0.0, 2.0), f"unexpected Z={z}"


def test_dark_image_descends_to_zmin(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "black.png", 0)
    out = relief3D.emit_gcode(_opts(tmp_path, img, z_min=-1.5))
    zs = _extract_z(out)
    assert min(zs) == pytest.approx(-1.5)


# ---------- error paths ----------


def test_unknown_tool_raises_friendly_error(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 128)
    opts = _opts(tmp_path, img, tool="no_such_tool")
    with pytest.raises(KeyError, match="known:"):
        relief3D.emit_gcode(opts)


def test_missing_image_raises(tmp_path: Path) -> None:
    opts = _opts(tmp_path, tmp_path / "nope.png")
    with pytest.raises(FileNotFoundError):
        relief3D.emit_gcode(opts)


def test_invalid_z_range(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 128)
    opts = _opts(tmp_path, img, z_max=-1.0, z_min=0.0)
    with pytest.raises(ValueError, match="z_max must be > z_min"):
        relief3D.emit_gcode(opts)


# ---------- CLI smoke ----------


def test_cli_writes_file(tmp_path: Path) -> None:
    img = _flat_image(tmp_path / "g.png", 200)
    out = tmp_path / "out.gcode"
    rc = relief3D.main(
        [
            "--image",
            str(img),
            "--out",
            str(out),
            "--tool",
            "ball_3mm",
            "--stock-width",
            "10",
            "--stock-height",
            "10",
            "--z-min",
            "-1.0",
            "--workspace",
            "G54",
        ]
    )
    assert rc == 0
    text = out.read_text()
    assert "G54" in text.splitlines()
    assert "M30" in text.splitlines()
