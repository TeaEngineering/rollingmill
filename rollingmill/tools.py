#!/usr/bin/env python3
"""Host-side cutting-tool library, persisted as JSON at ~/.rollingmill/tools.json.

Each tool carries the geometry needed by path planners (notably
``rollingmill.relief3D``) plus default feeds/speeds that emitters can use as
sensible defaults when the user does not pass explicit flags.

The library is intentionally dependency-free — ``numpy`` is only imported
lazily by :meth:`Tool.footprint`, which path planners use to build a
morphological structuring element.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Literal

ToolType = Literal["end_mill", "ball_nose", "v_bit", "engraver", "bull_nose"]

VALID_TYPES: tuple[ToolType, ...] = (
    "end_mill", "ball_nose", "v_bit", "engraver", "bull_nose",
)


@dataclass
class Tool:
    id: str
    name: str
    type: ToolType
    diameter_mm: float
    length_mm: float
    flute_length_mm: float
    shank_diameter_mm: float
    flutes: int
    feed_mm_min: float
    plunge_feed_mm_min: float
    spindle_rpm: int
    half_angle_deg: float | None = None
    corner_radius_mm: float | None = None
    firmware_slot: int | None = None
    max_doc_mm: float | None = None

    def __post_init__(self) -> None:
        if self.type not in VALID_TYPES:
            raise ValueError(
                f"tool {self.id!r}: type {self.type!r} must be one of {VALID_TYPES}"
            )
        if self.diameter_mm <= 0:
            raise ValueError(f"tool {self.id!r}: diameter_mm must be > 0")
        if self.length_mm <= 0:
            raise ValueError(f"tool {self.id!r}: length_mm must be > 0")
        if self.flute_length_mm <= 0:
            raise ValueError(f"tool {self.id!r}: flute_length_mm must be > 0")
        if self.shank_diameter_mm <= 0:
            raise ValueError(f"tool {self.id!r}: shank_diameter_mm must be > 0")
        if self.flutes <= 0:
            raise ValueError(f"tool {self.id!r}: flutes must be > 0")
        if self.feed_mm_min <= 0:
            raise ValueError(f"tool {self.id!r}: feed_mm_min must be > 0")
        if self.plunge_feed_mm_min <= 0:
            raise ValueError(f"tool {self.id!r}: plunge_feed_mm_min must be > 0")
        if self.spindle_rpm <= 0:
            raise ValueError(f"tool {self.id!r}: spindle_rpm must be > 0")
        if self.type == "v_bit" and self.half_angle_deg is None:
            raise ValueError(
                f"tool {self.id!r}: v_bit requires half_angle_deg"
            )
        if self.type == "bull_nose" and self.corner_radius_mm is None:
            raise ValueError(
                f"tool {self.id!r}: bull_nose requires corner_radius_mm"
            )
        if self.half_angle_deg is not None and not (0 < self.half_angle_deg < 90):
            raise ValueError(
                f"tool {self.id!r}: half_angle_deg must be in (0, 90)"
            )
        if self.corner_radius_mm is not None and self.corner_radius_mm <= 0:
            raise ValueError(
                f"tool {self.id!r}: corner_radius_mm must be > 0"
            )
        if self.firmware_slot is not None and not (1 <= self.firmware_slot <= 8):
            raise ValueError(
                f"tool {self.id!r}: firmware_slot must be in 1..8"
            )

    def to_dict(self) -> dict[str, Any]:
        d = dataclasses.asdict(self)
        return {k: v for k, v in d.items() if v is not None}

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Tool":
        known = {f.name for f in dataclasses.fields(cls)}
        unknown = set(d) - known
        if unknown:
            raise ValueError(f"unknown tool fields: {sorted(unknown)}")
        return cls(**d)

    def footprint(self, pitch_mm: float) -> Any:
        """Return the tool tip's height-offset map as a 2D numpy array.

        Cells inside the cutter's reach hold the height the tip would touch
        if the cutter axis passed through that offset; cells outside hold
        ``+inf`` so that ``scipy.ndimage.grey_erosion`` treats them as
        unreachable.
        """
        import numpy as np  # lazy; tools.py stays dependency-free

        if pitch_mm <= 0:
            raise ValueError("pitch_mm must be > 0")
        r = self.diameter_mm / 2.0
        radius_px = max(1, int(math.ceil(r / pitch_mm)))
        n = 2 * radius_px + 1
        cy = cx = radius_px
        yy, xx = np.mgrid[0:n, 0:n]
        dx = (xx - cx) * pitch_mm
        dy = (yy - cy) * pitch_mm
        d = np.hypot(dx, dy)
        inside = d <= r + 1e-9
        out = np.full((n, n), np.inf, dtype=float)
        if self.type == "end_mill":
            out[inside] = 0.0
        elif self.type == "ball_nose":
            inner = inside
            d_clipped = np.clip(d[inner], 0.0, r)
            out[inner] = r - np.sqrt(np.maximum(r * r - d_clipped * d_clipped, 0.0))
        elif self.type == "v_bit" or self.type == "engraver":
            assert self.half_angle_deg is not None
            tan_t = math.tan(math.radians(self.half_angle_deg))
            out[inside] = d[inside] / tan_t
        elif self.type == "bull_nose":
            assert self.corner_radius_mm is not None
            c = self.corner_radius_mm
            flat_r = max(r - c, 0.0)
            flat = d <= flat_r + 1e-9
            torus = inside & ~flat
            out[flat] = 0.0
            d_in_torus = d[torus] - flat_r
            d_in_torus = np.clip(d_in_torus, 0.0, c)
            out[torus] = c - np.sqrt(np.maximum(c * c - d_in_torus * d_in_torus, 0.0))
        else:
            raise ValueError(f"unhandled tool type {self.type!r}")
        return out


def default_path() -> Path:
    """Return the canonical tools.json path (honours $HOME)."""
    return Path(os.path.expanduser("~")) / ".rollingmill" / "tools.json"


SEED_TOOLS: list[Tool] = [
    Tool(
        id="end_6mm", name="6mm 2-flute end mill", type="end_mill",
        diameter_mm=6.0, length_mm=50.0, flute_length_mm=22.0,
        shank_diameter_mm=6.0, flutes=2, firmware_slot=1,
        feed_mm_min=600.0, plunge_feed_mm_min=150.0, spindle_rpm=12000,
        max_doc_mm=3.0,
    ),
    Tool(
        id="ball_3mm", name="3mm 2-flute ball nose", type="ball_nose",
        diameter_mm=3.0, length_mm=38.0, flute_length_mm=12.0,
        shank_diameter_mm=3.175, flutes=2, firmware_slot=2,
        feed_mm_min=400.0, plunge_feed_mm_min=80.0, spindle_rpm=14000,
        max_doc_mm=1.0,
    ),
    Tool(
        id="vbit_60", name="60° V-bit", type="v_bit",
        diameter_mm=6.35, length_mm=38.0, flute_length_mm=10.0,
        shank_diameter_mm=6.35, flutes=2, half_angle_deg=30.0,
        firmware_slot=3,
        feed_mm_min=500.0, plunge_feed_mm_min=80.0, spindle_rpm=14000,
        max_doc_mm=1.0,
    ),
]


def load(path: Path | None = None) -> list[Tool]:
    """Load tools from ``path`` (defaulting to ``default_path()``).

    If the file does not exist, seed it with ``SEED_TOOLS`` and return that.
    """
    p = path or default_path()
    if not p.exists():
        p.parent.mkdir(parents=True, exist_ok=True)
        save(SEED_TOOLS, p)
        return list(SEED_TOOLS)
    raw = json.loads(p.read_text())
    if not isinstance(raw, list):
        raise ValueError(f"{p}: expected a JSON list of tools, got {type(raw).__name__}")
    return [Tool.from_dict(d) for d in raw]


def save(tools: Iterable[Tool], path: Path | None = None) -> None:
    p = path or default_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    payload = [t.to_dict() for t in tools]
    p.write_text(json.dumps(payload, indent=2) + "\n")


def resolve(explicit: Any, tool: "Tool | None", attr: str, fallback: Any) -> Any:
    """Pick a value with priority: explicit CLI > tool default > fallback.

    Used by emitters wiring a ``--tool`` flag alongside legacy per-value flags:
    if the user passed the legacy flag (``explicit is not None``) it wins;
    otherwise the tool's attribute is used; otherwise the hard-coded fallback.
    """
    if explicit is not None:
        return explicit
    if tool is not None:
        v = getattr(tool, attr, None)
        if v is not None:
            return v
    return fallback


def find(tools: list[Tool], ref: str) -> Tool:
    """Find a tool by id, falling back to exact name match."""
    for t in tools:
        if t.id == ref:
            return t
    for t in tools:
        if t.name == ref:
            return t
    known = ", ".join(t.id for t in tools) or "(none)"
    raise KeyError(f"no tool with id or name {ref!r}; known: {known}")


def add(tools: list[Tool], tool: Tool) -> list[Tool]:
    if any(t.id == tool.id for t in tools):
        raise ValueError(f"tool id {tool.id!r} already exists")
    return tools + [tool]


def remove(tools: list[Tool], id: str) -> list[Tool]:
    out = [t for t in tools if t.id != id]
    if len(out) == len(tools):
        raise KeyError(f"no tool with id {id!r}")
    return out


def _print_table(tools: list[Tool]) -> None:
    if not tools:
        print("(no tools)")
        return
    header = f"{'id':<14} {'type':<10} {'dia':>6} {'len':>6} {'slot':>4}  name"
    print(header)
    print("-" * len(header))
    for t in tools:
        slot = "-" if t.firmware_slot is None else str(t.firmware_slot)
        print(
            f"{t.id:<14} {t.type:<10} {t.diameter_mm:>6.2f} "
            f"{t.length_mm:>6.1f} {slot:>4}  {t.name}"
        )


def _print_detail(t: Tool) -> None:
    for k, v in t.to_dict().items():
        print(f"{k}: {v}")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--path", type=Path, default=None,
                   help="Override library path (default: ~/.rollingmill/tools.json)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("init", help="Create the tools file with seed tools")
    sub.add_parser("list", help="List all tools")

    sp_show = sub.add_parser("show", help="Show one tool by id or name")
    sp_show.add_argument("ref")

    sp_rm = sub.add_parser("remove", help="Remove a tool by id")
    sp_rm.add_argument("id")

    args = p.parse_args(argv)
    path = args.path

    if args.cmd == "init":
        target = path or default_path()
        if target.exists():
            print(f"{target} already exists; not overwriting")
            return 1
        save(SEED_TOOLS, target)
        print(f"wrote {target} with {len(SEED_TOOLS)} seed tools")
        return 0

    if args.cmd == "list":
        _print_table(load(path))
        return 0

    if args.cmd == "show":
        _print_detail(find(load(path), args.ref))
        return 0

    if args.cmd == "remove":
        tools = remove(load(path), args.id)
        save(tools, path)
        print(f"removed {args.id}; {len(tools)} tools remain")
        return 0

    return 2


if __name__ == "__main__":
    sys.exit(main())
