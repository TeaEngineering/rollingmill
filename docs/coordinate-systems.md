# Coordinate Systems and Origins

Reverse engineered from `VP_MDX40A.exe` (PE32 MFC app, built 2012-11-07) via
Ghidra. All commands here are vendor control transfers (`bRequest=0x01`, EP0).
See [USB Protocol Reference](usb-protocol.md) for transfer-pattern semantics.

---

## Coordinate Systems

The MDX-40A firmware maintains multiple **Work Coordinate System (WCS) slots**:

- **WCS0** — machine coordinates (absolute, origin at home position)
- **WCS1–8** — named user coordinate systems
- **WCS9–309** — extended indexed coordinate systems

Origins persist in firmware non-volatile memory across USB disconnect and power
cycle. VPanel keeps a RAM cache of the active WCS origin (`this+0x64..0x70`,
4×uint32 XYZA) for display.

### Display math

VPanel's displayed coordinates subtract the active WCS origin from machine position:
 `compute_display_coords @ 0x00403810`:

```
displayed_X = machine_X - origin_X
displayed_Y = machine_Y - origin_Y
displayed_Z = machine_Z - origin_Z
displayed_A = (machine_A - origin_A) % 360000      // mod 360.000°
```

All values in 1/1000 mm (or 1/1000° for A) and are absolute Machine Coordinates.

Origin is zero when MCS (slot 0) is active.

RE: `FUN_00403810` (compute displayed coords) in VP_MDX40A.exe.


### Reading origins back

On startup and on every WCS change VPanel calls `refresh_work_origin →
query_coord_system_by_index` (0x00403910) — Pattern B reads:

| WCS slot   | Pattern B wValue       | Response              |
|------------|------------------------|-----------------------|
| WCS1       | `GET 0x030b`           | 4×uint32 XYZA offsets |
| WCS2       | `GET 0x3202`           | 4×uint32              |
| WCS3–8     | `GET 0x3203..0x3208`   | 4×uint32 each         |
| WCS9–309   | `GET 0x3209..0x3334`   | 4×uint32 each         |


### Origin write

Used internally (e.g. after Detect Jig, axis-config sequences) — write a
specific value into a WCS origin slot:

| WCS slot   | SET wValue           | Payload         |
|------------|----------------------|-----------------|
| WCS1       | `0x030c`             | 4×uint32 XYZA   |
| WCS2       | `0x3335`             | 4×uint32 XYZA   |
| WCS3       | `0x3336`             | 4×uint32 XYZA   |
| WCS4       | `0x3337`             | 4×uint32 XYZA   |
| WCS5       | `0x3338`             | 4×uint32 XYZA   |
| WCS6       | `0x3339`             | 4×uint32 XYZA   |
| WCS7       | `0x333A`             | 4×uint32 XYZA   |
| WCS8       | `0x333B`             | 4×uint32 XYZA   |
| WCS9–309   | `0x333C + (slot-9)`  | 4×uint32 XYZA   |

Big-endian uint32, units 1/1000 mm, reverse of above.

---

### Set origin ... here feature

This updates one or more axis of the chosen origin to the current tool position.
It is a read-modify-write operation: reads the current origin, reads the tool position with 0x0301, patches the selected axis values and then writes back the origin.

#### Choose coordinate system

| Dropdown value                         | ID |
|----------------------------------------|----|
| User Coordinate System (RML mode only) | 1  |
| G54                                    | 3  |
| G55                                    | 4  |
| G56                                    | 5  |
| G57                                    | 6  |
| G58                                    | 7  |
| G59                                    | 8  |
| EXOFS                                  | 2  |

See `dialog_poulate_coordinatesys_dropdown` at `0x00413380`.

#### Set from current position

| Dropdown box | `axis_choice` |
|--------------|---------------|
| X Origin     | 0             |
| Y Origin     | 1             |
| Z Origin     | 2             |
| XY Origin    | 3             |
| XYZ Origin   | 4             |
| A Origin     | 5             |

See `partial_update_coord_sys(coord_sys, axis_choice, curr_pos)` at `` for the axis update logic.

#### Set Z origin using sensor (down from this XY position)

Descends until the touch sensor is detected, then applies the Z0 sensor
adjustment to get the final Z origin.

#### Set "Y origin" / "Z origin" at centre of rotation

Not yet understood

#### Set YZ origin at centre of rotation

Not yet understood


## Moves

For moving to the orign of a selected coordinate system, see [move-commands.md](move-commands.md)

