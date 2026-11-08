# Z0 sensor probing

Select **User (RML-1)** with `c`, close the coordinate dialog, then press **b**.
Place the clean connected sensor on the workpiece, position the tool directly
above it, turn off the spindle, and close the cover. The **Z0 Sensor Probe**
dialog shows the sensor thickness stored in the machine: **Enter** starts the
descent/retract cycle and **Esc** dismisses the dialog, or requests an
immediate stop during motion. The TUI keeps polling and displaying Z/feed. Other machine commands are blocked until a fresh idle reply arrives.

The firmware updates the User Z origin; the application then reads slot 1 again
and refreshes its coordinate display. A cancellation, device error, lost status,
unobserved motion or timeout is reported as an unverified result, never success.
Quit during an active cycle requests stop; quit again after idle to exit.

Only mode 1 (User/RML-1) has live-capture evidence. Other WCS selections are
rejected. This feature uses the firmware's existing sensor thickness/settings;
it does not write a guessed thickness. The cycle uses the machine's stored sensor
thickness (GET 0x3900 / SET 0x3901, the Setup dialog's "Thickness of Z0 Sensor",
1/1000 mm), which is read at connect, shown in the dialog and in the TUI state pane to
the right of the spindle runtime. Measure your sensor and check it matches: a wrong
value shifts the Z origin the cycle sets by the same amount. To change it, type the
new value in the dialog (14.000–16.000 mm, as in VPanel's Setup dialog) and press
**H**; the value is written, read back and verified, and **Enter** is refused while an
edited value has not been set. There is no NC streaming during probing;
launch without `--file`. The fresh state must be idle, with cover closed and no
error or spindle-on flag.

## Setting probe thickness

The cycle subtracts the thickness of the conductive pad, which is stored in the machine:
`SET 0x3902` carries no thickness. The two commands below are the Setup dialog's
"Thickness of Z0 Sensor" box (see [usb-protocol.md](usb-protocol.md)).

| wValue | Payload | Notes |
|---|---|---|
| `0x3900` | reply `>I`, 1/1000 mm | Read (Pattern B): `SET 0x3900`, poll ping, `GET 0x0003`. `15000` = 15 mm |
| `0x3901` | `>I`, 1/1000 mm | Write, inside the `SET 0x1109` `00` … `FF` bracket; then poll ping bit 21 (`0x00200000`) until it clears (3 s timeout) |

To set 15.030 mm: `SET 0x1109 00`, `SET 0x3901 00 00 3A B6`, wait for the ack, `SET 0x1109 FF`, then read `0x3900`
back to check it (`00 00 3A B6`). VPanel limits the value to 14.000–16.000 mm; the firmware's own limits are not
known. Writing a changed value and reading it back was confirmed on a real MDX-40A.

## Protocol evidence

A passive VPanel/USB-driver capture from a physical MDX-40A showed:

1. SET 0x03F5 payload 02, operation begin SET 0x1109 payload 00.
2. Stop spindle: SET 0x3006 uint32 big-endian zero.
3. Twice: Pattern B query 0x2001, two-byte reply 00 14. Bit 2 of byte 1
   satisfies the connected check; bit 3 is the contact condition.
4. SET **0x3902**, payload **00 01 FF FF FF FF** (`>HHH`, mode 1, -1, -1).
5. Direct GET 0x0100 32-byte status polls during descent/retraction. During
   this phase the TUI does not send its normal idle heartbeat/secondary reads.
6. Motion flags clear, then re-read the User origin with Pattern B 0x030B.

Operation cleanup restores 0x03F5/0x1109 to FF, matching VPanel's operation exit.

The captured cycle descended at 60 mm/min, retracted at 1800 mm/min and ended
in about 16.9 seconds. Those speeds are firmware-controlled, not hard-coded
motion commands. A lowest sampled Z of -79.880 mm preceded reversal; exact
contact instant was between samples. No text success response is sent.
