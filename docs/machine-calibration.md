# Machine Calibration

The VPanel *Setup → Correction* tab exposes two independent calibration sets:

- **Distance Adjustment** — per-axis scaling factors for X, Y, Z (and A, hidden from the UI).
- **Axis Angle Correction for the Center of Rotation** — two-point linear correction for the rotary A-axis: an X-position pair `(Point1, Point2)` with Y/Z offsets at each, used to deskew the rotary axis when its rotation axis isn't perfectly parallel to the X-axis of the bed.

The dialog is launched from the main panel's *Setup* button (`launch_Setup_dialog_button_2329h @ 0x00416360`). On open it calls `read_axis_correction_factors @ 0x004017d0` to populate both sets from the firmware; on *OK* it calls `settings_write_axis_scaling_and_rotary_offsets @ 0x004018a0` to push them back. Each of those wrappers fans out to one read and one write per calibration set — four leaf functions in total.

Both write paths poll `GET 0x0001` bit 21 clear after the SET — see [usb-protocol.md](usb-protocol.md#ping-bit-21--command-acknowledgement).

---

## Axis Scaling

Three percentage values shown in the dialog (X, Y, Z), plus a fourth (A) read and written but not exposed in the UI. Stored on the firmware as 4 × `(numerator, denominator)` `uint32` pairs; the effective scale per axis is `numerator / denominator`. VPanel always writes the denominator as `1,000,000`, so the wire numerator is `round(percent × 10000)` (default `100.000 %` → `1,000,000 / 1,000,000` = 1.0).

| Direction | Function | wValue | Payload | Notes |
|-----------|----------|--------|---------|-------|
| Read  | `get_XYZ_axis_scaling_values_0x5f0 @ 0x00419f60` | `GET 0x05f0` | 32 bytes BE — `4 × uint32` numerators then `4 × uint32` denominators, axis order X, Y, Z, A | Pattern B. Values scaled by `DOUBLE_0043e9e8` (= 100.0) for display so they read as percentages. |
| Write | `set_XYZ_axis_scaling_values_0x5f1 @ 0x0041a110` (axis-scaling write) | `SET 0x05f1` | 8 × `uint32` BE — `[numX, numY, numZ, numA, 1e6, 1e6, 1e6, 1e6]` | Each percentage × `DOUBLE_0043e9f0` (= 10000.0) → `uint32` via `__ftol`; denominators always 1,000,000. Poll ping bit 21 clear after. |

---

## Rotary Axis Calibration

Two correction points along the rotary A-axis, each carrying a Y and Z offset that the firmware uses to lift/translate the toolpath as a function of X position. Stored as 6 × signed `int32` in 1/1000 mm, big-endian.

The dialog lays the six values out as:

```
Point1 - X(A) : refX1     mm
  offset Y(B) : ofsY1     mm
  offset Z(C) : ofsZ1     mm
Point2 - X(D) : refX2     mm
  offset Y(E) : ofsY2     mm
  offset Z(F) : ofsZ2     mm
```

Wire order matches the visual order: `[refX1, ofsY1, ofsZ1, refX2, ofsY2, ofsZ2]`.

`settings_write_axis_scaling_and_rotary_offsets` swaps the two triples before sending if `refX1 > refX2`, so the firmware always receives the lower-X point first.

| Direction | Function | wValue | Payload | Notes |
|-----------|----------|--------|---------|-------|
| Read  | `get_rotary_axis_angle_correction_0x3804 @ 0x0041ad10` | `GET 0x3804` | 24 bytes BE — 6 × `int32` in 1/1000 mm, order `[refX1, ofsY1, ofsZ1, refX2, ofsY2, ofsZ2]` | Pattern B. Same wValue also used in [usb-protocol.md](usb-protocol.md#axis-configuration--tool-offsets) for the pre-cut diagnostic snapshot — the layout there is the same six values. |
| Write | `set_rotary_axis_angle_correction_0x3805 @ 0x0041ad70` | `SET 0x3805` | 24 bytes BE — 6 × `int32`, same layout, lower-X point first | Poll ping bit 21 clear after. |

Note this is _a very similar_ but different call to that used at the end of the automatic [rotary jig alignment function](rotary-jig-alignment.md), which writes 6x int32 to 0x3803.

---

## Dialog control flow (for reference)

```
launch_Setup_dialog_button_2329h @ 0x00416360
    SET 0x3006 = 0                              (force spindle off)
    read_axis_correction_factors @ 0x004017d0
        get_XYZ_axis_scaling_values_0x5f0       (GET 0x5f0)
        get_rotary_axis_angle_correction_0x3804 (GET 0x3804)
    SET 0x03f5  / SET 0x1109 = 0x00             (keepalive + bracket begin)
    inner_show_settings_dialog_modal            (modal dialog loop)
    if user pressed OK:
        settings_write_axis_scaling_and_rotary_offsets @ 0x004018a0
            set_XYZ_axis_scaling_values_0x5f1       (SET 0x5f1, poll bit21)   ← if scaling dirty
            set_rotary_axis_angle_correction_0x3805 (SET 0x3805, poll bit21)  ← if rotary dirty
    SET 0x03f2 / SET 0x03f5 / SET 0x1109 = 0xff (bracket end)
```

The writer guards each leaf on a dirty-flag at `+0xb0` (scaling) and `+0x126` (rotary), so unchanged tabs don't re-issue the SET.
