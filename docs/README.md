# Reverse Engineering Notes

Roland MDX-40A USB protocol — findings from Ghidra analysis of `VP_MDX40A.exe` and driver DLLs.

| Binary | `VP_MDX40A.exe` — PE32, x86, MFC, built 2012-11-07 |
|--------|-----------------------------------------------------|
| Tools  | Ghidra 12.0.4, pefile, PyGhidra 3.0.2              |

---

## Protocol Reference

| Document | Summary |
|----------|---------|
| [USB Protocol Reference](usb-protocol.md) | Vendor control command table (all wValues), Pattern A/B transfer sequences, payload formats, byte-swap conventions |
| [Machine State & Motion](machine-state.md) | State flags (GET 0x0100), ping register (GET 0x0001), motion-complete detection algorithm, observed live values |
| [Jog Speeds](jog-speeds.md) | Step mode table, continuous ramp algorithm, A-axis differences |
| [Sending NC/RML Code](sending-nc-code.md) | Bulk output flow, operation bracket (SET 0x1109), stepped test-cut mode, Windows driver stack |

## Machine Operations

| Document | Summary |
|----------|---------|
| [Coordinate Systems, Move To, and Origins](coordinate-systems.md) | WCS slots, display math, Move-To modes, Set Origin commands, VIEW position vs WCS origin |
| [Jig Detection Algorithm](vpanel-jig-detect.md) | A-axis calibration rod routine — triple-contact probing, pin bisection, origin write |
| [Tool Diameter Offsets](tool-diameter-offsets.md) | Storage and USB commands for 8-slot tool diameter offset table |
| [Tool Sensor Calibration](tool-sensor-calibration.md) | Z-sensor location calibration routine (stub) |

---

## Key Findings at a Glance

**Transport:** All machine control uses USB vendor control transfers (bRequest=0x01). NC/RML file
data uses the bulk-OUT endpoint. There is no framing on the bulk channel — raw bytes only.

**Two transfer patterns:**
- Pattern A — direct `GET wValue` (e.g. coordinates via `0x0100`)
- Pattern B — `SET wValue` trigger then poll ping, then `GET 0x0003` for response data

**Critical:** The firmware ignores bulk NC data unless wrapped in an operation bracket:
`SET 0x1109 = 0x00` before data, `SET 0x1109 = 0xFF` after.

**Byte order:** All multi-byte device values are big-endian.
