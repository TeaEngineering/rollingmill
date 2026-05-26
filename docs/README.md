# Reverse Engineering Notes

The MDX-40A is installed as a Windows printer (Class=Printer), not a raw USB HID or WinUSB device. VPanel uses USB vendor control transfers (EP0) for machine state, jog, and configuration; NC/RML file data flows through the Windows printer stack to USB bulk-OUT.

The key components are:

| File          | Role                                             |
|---------------|--------------------------------------------------|
| VP_MDX40A.exe | VPanel application                               |
| RD25DGR64.DLL | Graphics DLL — the print driver proper           |
| RD25DUI64.DLL | UI/config DLL — printer properties pages         |
| rdlm64.dll    | Language Monitor                                 |
| rd25dlf64.dll | Language Filter (chained with LM)                |
| MDX40Ax64.RPD | Roland Printer Descriptor — machine capabilities |
| MDX40AMAT.DAT | Material/tool data                               |


Windows:

    VPanel → WritePrinter("Roland MDX-40A") → Spooler → rdlm64.dll → USBPRINT.SYS → USB bulk-out

RollingMill:

    Our app → libusb bulk_write() → USB bulk-out

We skip the entire printer stack.



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
| [Rotary Jig Alignment](rotary-jig-alignment.md) | A-axis calibration rod routine, "Current Jig" indicator, centreline equality check |
| [Tool Diameter Offsets](tool-diameter-offsets.md) | Storage and USB commands for 8-slot tool diameter offset table |
| [Tool Sensor Calibration](tool-sensor-calibration.md) | Refines the XY centre of the Z-height sensor pad |

---

## Key Findings at a Glance

**Transport:** All machine control uses USB vendor control transfers (bRequest=0x01). NC/RML file
data uses the bulk-OUT endpoint. There is no framing on the bulk channel — raw bytes only.

**Two transfer patterns:**
- Pattern A — direct `GET wValue` (e.g. coordinates via `0x0100`)
- Pattern B — `SET wValue` trigger then poll ping, then `GET 0x0003` for response data

**Byte order:** All multi-byte device values are big-endian.
