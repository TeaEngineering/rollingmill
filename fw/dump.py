#!/usr/bin/env python3
"""Extract firmware regions from a Roland PUF1-wrapped .rfw file.

The PUF1 block can either be the whole file (.rfw) or embedded inside a
larger binary (e.g. the DropOut .exe firmware installer); in the latter
case the script scans the file for the PUF1 magic.

Container layout (derived by inspection of TS30M190.rfw):

  - 48-byte ASCII header:
        0..3   "PUF1"           magic
        4..7   small field      (format/version, e.g. "8518")
        8..31  description      (e.g. "  TS30 MAIN V190        ")
        32..39 hex string       body checksum/length marker
        40..47 hex string       byte length of record stream
  - Motorola S-record stream with the leading 'S' (0x53) rewritten to
    'B' (0x42).  Records seen: B0 (file name, 16-bit address),
    B2 (data, 24-bit address), B8 (end record, 24-bit address).
    Each record is: 'B' type len addr[2|3] data[...] cksum
    where (len + sum(addr+data) + cksum) & 0xFF == 0.
  - 8-byte trailer: 4 hex chars + "PUFE".

This script parses the records, groups B2 payloads into contiguous
address ranges and writes one .bin per region.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

MAGIC_HEAD = b"PUF1"
MAGIC_TAIL = b"PUFE"
ADDR_SIZE = {0x30: 2, 0x32: 3, 0x38: 3}  # B0, B2, B8


def find_puf1(data: bytes) -> int:
    """Return offset of the PUF1 block.  Tries offset 0, otherwise scans
    forward (the .exe installers embed the firmware blob inside the binary).
    Raises ValueError if no plausible header is found.
    """
    if data[:4] == MAGIC_HEAD:
        return 0
    search_from = 0
    while True:
        i = data.find(MAGIC_HEAD, search_from)
        if i < 0:
            raise ValueError("No PUF1 magic found in file")
        # Validate: bytes 32..40 and 40..48 must be ASCII hex digits.
        try:
            int(data[i + 32 : i + 40], 16)
            int(data[i + 40 : i + 48], 16)
            return i
        except ValueError:
            search_from = i + 1


def parse_rfw(data: bytes, start: int | None = None):
    if start is None:
        start = find_puf1(data)
    if data[start : start + 4] != MAGIC_HEAD:
        raise ValueError(f"Not a PUF1 block at offset 0x{start:x} (magic={data[start:start+4]!r})")

    header = {
        "magic": data[start + 0 : start + 4].decode("ascii"),
        "field1": data[start + 4 : start + 8].decode("ascii"),
        "description": data[start + 8 : start + 32].decode("ascii").strip(),
        "hex1": int(data[start + 32 : start + 40], 16),
        "hex2": int(data[start + 40 : start + 48], 16),
        "start_offset": start,
    }

    # hex2 is the byte length of the record stream that follows the 48-byte
    # header.  An 8-byte trailer "<4 hex chars>PUFE" sits immediately after.
    record_stream_len = header["hex2"]
    trailer_off = start + 48 + record_stream_len
    if trailer_off + 8 > len(data):
        raise ValueError(
            f"Truncated file: header at 0x{start:x} claims record stream "
            f"length 0x{record_stream_len:x} but file ends at 0x{len(data):x}"
        )
    trailer = data[trailer_off : trailer_off + 8]
    if trailer[4:] != MAGIC_TAIL:
        raise ValueError(f"Missing PUFE trailer at 0x{trailer_off:x} (got {trailer!r})")
    header["trailer_checksum"] = trailer[:4].decode("ascii")

    name = None
    entry = None
    b2_records: list[tuple[int, bytes]] = []

    pos = start + 48
    end = trailer_off  # records run up to (not into) the trailer
    while pos < end:
        if data[pos] != ord("B"):
            raise ValueError(f"Expected 'B' record marker at 0x{pos:x}, got 0x{data[pos]:02x}")
        rtype = data[pos + 1]
        if rtype not in ADDR_SIZE:
            raise ValueError(f"Unknown record type B{chr(rtype)} at 0x{pos:x}")
        length = data[pos + 2]
        asz = ADDR_SIZE[rtype]
        rec_end = pos + 3 + length
        addr = int.from_bytes(data[pos + 3 : pos + 3 + asz], "big")
        payload = data[pos + 3 + asz : rec_end - 1]
        cksum = data[rec_end - 1]
        # Verify checksum: (len + addr_bytes + data_bytes + cksum) low byte == 0
        if (length + sum(data[pos + 3 : rec_end - 1]) + cksum) & 0xFF != 0xFF & 0xFF:
            # The S-record rule is: cksum = ~(len+addr+data) & 0xFF
            calc = (~(length + sum(data[pos + 3 : rec_end - 1]))) & 0xFF
            if calc != cksum:
                raise ValueError(
                    f"Bad checksum in B{chr(rtype)} at 0x{pos:x}: "
                    f"got 0x{cksum:02x}, expected 0x{calc:02x}"
                )
        if rtype == 0x30:
            name = payload.decode("ascii", errors="replace")
        elif rtype == 0x32:
            b2_records.append((addr, payload))
        elif rtype == 0x38:
            entry = addr
        pos = rec_end

    return header, name, entry, b2_records


def regions_from_records(b2: list[tuple[int, bytes]]) -> list[tuple[int, bytes]]:
    """Group B2 payloads into contiguous (start_addr, bytes) regions."""
    if not b2:
        return []
    b2_sorted = sorted(b2, key=lambda r: r[0])
    regions: list[tuple[int, bytearray]] = []
    cur_addr, cur_buf = b2_sorted[0][0], bytearray(b2_sorted[0][1])
    for addr, payload in b2_sorted[1:]:
        end = cur_addr + len(cur_buf)
        if addr == end:
            cur_buf.extend(payload)
        elif addr < end:
            raise ValueError(f"Overlapping records near 0x{addr:x}")
        else:
            regions.append((cur_addr, bytes(cur_buf)))
            cur_addr, cur_buf = addr, bytearray(payload)
    regions.append((cur_addr, bytes(cur_buf)))
    return regions


def main() -> int:
    ap = argparse.ArgumentParser(description="Dump Roland .rfw firmware regions to .bin files")
    ap.add_argument("rfw", type=Path, help="input .rfw firmware file")
    ap.add_argument(
        "-o",
        "--outdir",
        type=Path,
        default=Path("."),
        help="output directory (default: cwd)",
    )
    args = ap.parse_args()

    data = args.rfw.read_bytes()
    header, name, entry, b2 = parse_rfw(data)

    print(f"PUF1 block at file offset 0x{header['start_offset']:X}")
    print(f"Header:      magic={header['magic']} field1={header['field1']!r}")
    print(f"Description: {header['description']!r}")
    print(f"hex1=0x{header['hex1']:X}  hex2=0x{header['hex2']:X}  file_size={len(data)} (0x{len(data):X})")
    print(f"Trailer checksum: {header['trailer_checksum']!r}")
    print(f"B0 file name: {name!r}")
    print(f"B8 end-record address field: 0x{entry:08X}" if entry is not None else "B8 end record: (none)")
    print(f"B2 records: {len(b2)}")

    regions = regions_from_records(b2)
    args.outdir.mkdir(parents=True, exist_ok=True)
    stem = args.rfw.stem
    print(f"\nRegions ({len(regions)}):")
    for i, (addr, buf) in enumerate(regions):
        out = args.outdir / f"{stem}_region{i}_{addr:08x}.bin"
        out.write_bytes(buf)
        print(f"  0x{addr:08X} .. 0x{addr + len(buf):08X}  ({len(buf)} bytes)  -> {out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
