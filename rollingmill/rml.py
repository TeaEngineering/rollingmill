"""
RML-1 (Roland Machine Language) command builder.

Pure string/bytes construction — no USB, no machine state.
Commands are sent via MDX40A.bulk_write() or mdx40a.usb.bulk_write().

RML-1 reference:
  ^Z          — initialize / soft reset
  !MC0;       — spindle off
  !MC1;       — spindle on
  !PZ<z>,<f>; — set Z speed (z in units, f = feed rate)
  V<f>;       — set feed rate (mm/min)
  Z<x>,<y>,<z>; — move to absolute position
  H;          — go to home position
"""


def initialize() -> bytes:
    """Soft reset / initialize."""
    return b'\x1a'


def spindle(on: bool) -> bytes:
    """Turn spindle on (True) or off (False)."""
    return b'!MC1;' if on else b'!MC0;'


def feed_rate(mm_per_min: float) -> bytes:
    """Set feed rate in mm/min."""
    return f'V{mm_per_min:.1f};'.encode()


def move_abs(x_mm: float, y_mm: float, z_mm: float) -> bytes:
    """Move to absolute position (mm)."""
    # RML-1 uses 1/100 mm units
    x = round(x_mm * 100)
    y = round(y_mm * 100)
    z = round(z_mm * 100)
    return f'Z{x},{y},{z};'.encode()


def home() -> bytes:
    """Return to home position."""
    return b'H;'
