"""Lint CLI: ``python -m rollingmill.check <filename>``.

Lives in its own module (rather than under ``cutjob``) because the
package's ``__init__.py`` re-exports ``CutJob``. Running
``python -m rollingmill.cutjob`` would trigger a runpy ``RuntimeWarning``
about the module being imported twice; this module is not imported by
``__init__.py``, so ``python -m rollingmill.check`` is warning-free.

Output is gcc-style: a header line ``file:line:col: [rule N] message``,
the offending source line, and a caret line under the column. ANSI
colour is applied when stdout is a TTY and ``NO_COLOR`` is unset.
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

from .cutjob import NCLintError, NCWarning, load_nc_blocks
from .estimate import (
    DEFAULT_MIN_OP_S, DEFAULT_RAPID_XY, DEFAULT_RAPID_Z,
    EstimateOptions, estimate_time, format_duration,
)


# Minimal ANSI palette. Empty strings when colour is disabled, so the
# format strings work identically in both modes.
class _Style:
    def __init__(self, enabled: bool) -> None:
        self.bold   = "\x1b[1m"  if enabled else ""
        self.dim    = "\x1b[2m"  if enabled else ""
        self.red    = "\x1b[31m" if enabled else ""
        self.green  = "\x1b[32m" if enabled else ""
        self.yellow = "\x1b[33m" if enabled else ""
        self.cyan   = "\x1b[36m" if enabled else ""
        self.reset  = "\x1b[0m"  if enabled else ""


def _use_color(stream) -> bool:
    """True iff *stream* is a TTY and the user hasn't opted out via NO_COLOR."""
    if os.environ.get("NO_COLOR"):
        return False
    isatty = getattr(stream, "isatty", None)
    return bool(isatty and isatty())


def _format_warning(
    path: str, blocks, w: NCWarning, style: _Style,
) -> str:
    """Render a single warning as three lines: header, source, caret."""
    raw = blocks[w.line - 1].decode("ascii", errors="replace")
    source = raw.rstrip("\r\n")
    caret = " " * max(0, w.column - 1) + "^"
    header = (
        f"{style.bold}{path}:{w.line}:{w.column}:{style.reset} "
        f"{style.red}[rule {w.rule}]{style.reset} "
        f"{style.yellow}{w.message}{style.reset}"
    )
    return (
        f"{header}\n"
        f"{style.dim}{source}{style.reset}\n"
        f"{style.green}{caret}{style.reset}"
    )


def lint_file(path: str, estimate: Optional[EstimateOptions] = None) -> int:
    """Read *path*, run ``load_nc_blocks``, print results in three-line ANSI format.

    Returns the same exit-code contract as :func:`main`: 0 clean, 1 warnings,
    2 I/O error. Used both by the standalone CLI and by ``tui.main`` to
    short-circuit a bad ``--file`` before any USB discovery. When *estimate*
    is given and the file is clean, a milling-time estimate is also printed.
    """
    try:
        with open(path, 'rb') as fh:
            data = fh.read()
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    style = _Style(_use_color(sys.stdout))
    try:
        blocks = load_nc_blocks(data)
    except NCLintError as exc:
        print(f"{style.bold}{style.red}Input file failed validation:{style.reset}")
        for w in exc.warnings:
            print(_format_warning(path, exc.blocks, w, style))
        summary_style = _Style(_use_color(sys.stderr))
        print(
            f"{summary_style.bold}{path}: {len(exc.warnings)} warning(s)"
            f"{summary_style.reset}",
            file=sys.stderr,
        )
        return 1
    except ValueError as exc:
        print(f"error: {path}: {exc}", file=sys.stderr)
        return 2
    print(f"{style.green}{path}: OK ({len(blocks)} blocks){style.reset}")
    if estimate is not None:
        _print_estimate(blocks, estimate, style)
    return 0


def _print_estimate(blocks, opts: EstimateOptions, style: _Style) -> None:
    est = estimate_time(blocks, opts)
    print(
        f"{style.bold}Estimated time: {format_duration(est.total_s)}{style.reset}"
        f" (cutting {format_duration(est.cutting_s)},"
        f" rapid {format_duration(est.rapid_s)},"
        f" dwell {format_duration(est.dwell_s)},"
        f" commands {format_duration(est.command_s)};"
        f" {est.moves} moves, {est.path_mm:.0f} mm path)"
    )
    for w in est.warnings:
        print(f"{style.yellow}  estimate: {w}{style.reset}")


def main(argv: Optional[list[str]] = None) -> int:
    """CLI entry: ``python -m rollingmill.check <filename>``."""
    parser = argparse.ArgumentParser(
        prog="python -m rollingmill.check",
        description="Lint an NC file and estimate its milling time.",
    )
    parser.add_argument("filename")
    parser.add_argument("--rapid-xy", type=float, default=DEFAULT_RAPID_XY,
                        metavar="MM_MIN", help="XY rapid speed (default %(default)s mm/min)")
    parser.add_argument("--rapid-z", type=float, default=DEFAULT_RAPID_Z,
                        metavar="MM_MIN", help="Z rapid speed (default %(default)s mm/min)")
    parser.add_argument("--feed-override", type=float, default=100.0,
                        metavar="PCT", help="cutting feed override %% (default %(default)s)")
    parser.add_argument("--min-op", type=float, default=DEFAULT_MIN_OP_S,
                        metavar="SEC", help="minimum time per move/dwell (default %(default)s s)")
    parser.add_argument("--no-estimate", action="store_true",
                        help="lint only, skip the time estimate")
    try:
        args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    except SystemExit as exc:
        return int(exc.code or 0) if isinstance(exc.code, int) else 2
    opts = None if args.no_estimate else EstimateOptions(
        rapid_xy=args.rapid_xy, rapid_z=args.rapid_z,
        feed_override_pct=args.feed_override, min_op_s=args.min_op,
    )
    return lint_file(args.filename, opts)


if __name__ == "__main__":
    sys.exit(main())
