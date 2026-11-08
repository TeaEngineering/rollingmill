# Z0 sensor probing

Select the coordinate system whose Z origin you want to set with `c` (**User (RML-1)**,
**EXOFS** or **G54–G59**), activate it, then close the coordinate dialog.
Then press **b** to open the Z-origin dialog.
Place the clean connected sensor on the workpiece, position the tool directly
above it, turn off the spindle, and close the cover. The **Z0 Sensor Probe**
dialog shows the target coordinate system and the sensor thickness stored in the machine: **Enter** starts the
descent/retract cycle and **Esc** dismisses the dialog, or requests an
immediate stop during motion. The TUI keeps polling and displaying Z/feed. Other machine commands are blocked until a fresh idle reply arrives.

The firmware updates the target's Z origin; the application then reads that slot again
and refreshes its coordinate display. A cancellation, device error, lost status,
unobserved motion or timeout is reported as an unverified result, never success.
Quit during an active cycle requests stop; quit again after idle to exit.

You cannot start the Z-origin probing for machine coordinates (MCS - slot 0) since that
offset is fixed at zero. VPanel only offers EXOFS and G54–G59 when the command set is NC Code or
automatic, so the dialog warns when the command set is RML-1; whether the firmware
accepts those modes then is not known. The cycle uses the machine's stored sensor
thickness (GET 0x3900 / SET 0x3901, the Setup dialog's "Thickness of Z0 Sensor",
1/1000 mm), which is read at connect, shown in the dialog and in the TUI state pane.

Measure your sensor and check it matches: a wrong value shifts the Z origin the cycle
sets by the same amount. To change it, type the new value in the dialog
(14.000–16.000 mm, as in VPanel's Setup dialog) and press
**H**; the value is written, read back and verified, and **Enter** is refused while an
edited value has not been set.

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

## Target coordinate system

The first word of `SET 0x3902` is the WCS slot index (the same numbering as
`WCS_SLOT_NAMES` and the `wcs_code` of `SET 0x3501`); the other two words are `FFFF`.
From VPanel's `MDx3902_GiantDispatch` (`0x403b10`, operation 6, called from
`set_origin_using_sensor_button_gd6`), whose `cs` is the "Set Origin Point" dropdown's
item ID (`dialog_poulate_coordinatesys_dropdown`, `0x413e80`):

| Slot (`cs`) | Coordinate system | First word | VPanel encoder |
|---|---|---|---|
| 0 | MCS | not allowed | — |
| 1 | User (RML-1) | `0001` | `0x41b270` |
| 2 | EXOFS | `0002` | `0x41b2b0` |
| 3–8 | G54–G59 | `0003`–`0008` | `0x41b2f0(cs + 0x33)`: `(G − 54) + 3` |
| 10–310 | G54.1 P1–P300 | `cs − 1` | `0x41b340(cs − 9)`: `(n − 1) + 9`; not implemented here |

## Protocol evidence

A passive VPanel/USB-driver capture from a physical MDX-40A showed:

1. SET 0x03F5 payload 02, operation begin SET 0x1109 payload 00.
2. Stop spindle: SET 0x3006 uint32 big-endian zero.
3. Twice: Pattern B query 0x2001, two-byte reply 00 14. Bit 2 of byte 1
   satisfies the connected check; bit 3 is the contact condition.
4. SET **0x3902**, payload **00 01 FF FF FF FF** (`>HHH`, slot, -1, -1).
5. Direct GET 0x0100 32-byte status polls during descent/retraction. During
   this phase the TUI does not send its normal idle heartbeat/secondary reads.
6. Motion flags clear, then re-read the User origin with Pattern B 0x030B.

Operation cleanup restores 0x03F5/0x1109 to FF, matching VPanel's operation exit.

The captured cycle descended at 60 mm/min, retracted at 1800 mm/min and ended
in about 16.9 seconds. Those speeds are firmware-controlled, not hard-coded
motion commands. A lowest sampled Z of -79.880 mm preceded reversal; exact
contact instant was between samples. No text success response is sent.
