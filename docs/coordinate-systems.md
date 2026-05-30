# Coordinate Systems, Move To, and Origins

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

---

## Move To

Three modes, selected by the main UI panel's radio buttons.

### Mode 1 — Move to VIEW Position

The firmware has a per-WCS **VIEW position** register (separate from the WCS
origin). One trigger command tells the machine to drive itself there:

```
SET 0x1109, [0x00]                                      // begin operation bracket
SET 0x3902, struct.pack('>3H', cs, 0xFFFF, 0xFFFF)      // wcs_index + 2 sentinels
SET 0x3f2,  []                                          // resume
SET 0x1109, [0xFF]                                      // end operation bracket
```

`cs` encoding for the `0x3902` payload:

| WCS    | `cs` value     |
|--------|----------------|
| WCS1   | `0x0001`       |
| WCS2   | `0x0002`       |
| WCS3–8 | `cs + 0x33`    |
| WCS10+ | `cs + 8`       |

No coordinate math in VPanel — firmware handles the entire motion.

### Mode 2 — Move to User Specified Location

Handled by `on_cmd_move_to_user_position @ 0x00415e50`. The user types target
coordinates into a dialog; VPanel does **no** direct absolute-move USB write.
Instead it dispatches through `MDx3902_GiantDispatch` with mode `6`, and the
firmware performs the move using its own internal logic:

```
SET 0x1109, [0x00]                                            // begin operation bracket
do_preferences_dialog(this)                                    // user enters target coords
MDx3902_GiantDispatch(this->coordinat_sys_for_testcut, 6)      // SET 0x3902 dispatch, mode=6
FUN_00414170(this)                                             // post-dispatch hook (TODO: trace)
SET 0x3f2, []                                                  // resume
SET 0x1109, [0xFF]                                             // end operation bracket
```

- Active WCS comes from `this->coordinat_sys_for_testcut` — Mode 2 honours the
  currently-selected WCS, it does **not** force machine coordinates over USB.
- The absolute-move `>HH4i` payload (speed, flags, X, Y, Z, A) is the
  firmware's internal handling of `mode=6`, not what VPanel sends on the wire.

### Mode 3 — Move to Named/Stored Position

A combo box selects from firmware-stored named positions. Bare triggers —
the firmware uses its internally stored coordinates for that slot:

```
SET 0x3806    // move to EXOFS stored position
SET 0x3807    // move to alternate stored position
```

---

### Explicit origin write

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

Big-endian uint32, units 1/1000 mm.

---

## VIEW Position vs WCS Origin

Two different per-WCS registers with overlapping but distinct purposes:

|                | WCS Origin                                       | VIEW Position                                    |
|----------------|--------------------------------------------------|--------------------------------------------------|
| Purpose        | User coordinate system zero point                | Named "go-here" target for Move To               |
| Written by     | `SET 0x030c..0x333B+`                            | `SET 0x3803` (Detect Jig) / `SET 0x3805`         |
| Read by        | `GET 0x030b/0x3202+` (Pattern B)                 | (not observed — VIEW is consumed internally by `SET 0x3902`/`0x3806`/`0x3807`) |
| Used by        | Display (`displayed = machine - origin`)         | `SET 0x3902` Move To VIEW                        |
| Per WCS?       | Yes                                              | Yes                                              |
| Persistent?    | Yes (firmware non-volatile)                      | Yes (firmware non-volatile)                      |


---

## Summary

| Feature                       | USB Command(s)                                | Result lives in            | VPanel math? |
|-------------------------------|-----------------------------------------------|----------------------------|--------------|
| Move → VIEW position          | `SET 0x3902`                                  | Firmware VIEW reg          | No           |
| Move → User specify           | `SET 0x04f7` (absolute)                       | Machine motion             | Display only |
| Move → Named position         | `SET 0x3806` / `0x3807`                       | Firmware named regs        | No           |
| Set Origin (explicit write)   | `SET 0x030c` / `0x3335..0x333B+` + 4×uint32   | Firmware WCS slot          | Caller       |
| Detect Jig result             | `SET 0x3803` + 6×uint32                       | Firmware VIEW reg          | Yes (probing)|
| Read origin back              | `GET 0x030b` etc. (Pattern B)                 | VPanel RAM cache           | Display only |
