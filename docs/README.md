# Reverse Engineering Notes

The MDX-40A is installed as a Windows printer (Class=Printer), not a raw USB HID or WinUSB device. VPanel uses USB vendor control transfers (EP0) for machine state, jog, and configuration; NC/RML file data flows through the Windows printer stack to USB bulk-OUT.

The key components are:

| File          | Role                                             | sha256 |
|---------------|--------------------------------------------------|-------------------
| VP_MDX40A.exe | VPanel application                               | e360efc53452a71db41ddbe803c161c8a1e90987def6c2ebf44cf4cb6ee6c736 |
| RD25DGR64.DLL | Graphics DLL — the print driver proper           | 4315f516e3f69329827baca1819a70bb4a589296e5bbd365d3474e6676f17a61 |
| RD25DUI64.DLL | UI/config DLL — printer properties pages         | cbc2417d3470b3394d5029144b6f50e6d30fbebc3386aaa4eb3170ff1d6f372c |
| rdlm64.dll    | Language Monitor                                 | 87700f4e62eab3e26cdf480d04924e7c1a869e6c555343c3af416e8ef4a58f00 |
| rd25dlf64.dll | Language Filter (chained with LM - not stripped, only JP) | 00ffe3295d0fac73d0ded9c9bdded3d14548f4110d0223564d6e1b8fea0e8811 |
| MDX40Ax64.RPD | Roland Printer Descriptor — machine capabilities | - |
| MDX40AMAT.DAT | Material/tool data                               | - |

I used Ghidra 12.0.4 for the dissassembly work.

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
