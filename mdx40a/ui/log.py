"""
Logging configuration for mdx40a tools.

Protocol/debug messages go to stderr via Python logging.
Coordinate display goes to stdout directly (monitor.py).
"""

import logging
import sys


def setup(level: int = logging.WARNING) -> None:
    """Configure root logger to write to stderr."""
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(logging.Formatter(
        '%(asctime)s  %(levelname)-7s  %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    ))
    logging.getLogger().addHandler(handler)
    logging.getLogger().setLevel(level)
    # Quiet noisy USB library
    logging.getLogger('usb').setLevel(logging.WARNING)
