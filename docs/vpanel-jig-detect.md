# Jig Detection Algorithm

The A-axis calibration rod routine finds and stores the position of the A-axis rotation
centre in XYZ machine space, using a detection bar and detection pin fitted to the rotary table.

Reference video: https://youtu.be/UKR7mRjQrUs?t=934

---

## Call Chain

```
execute_cut_job (0x00416360)
  └─► FUN_00436e60
        └─► (*vtable[0xBC])()  →  start_jig_detect_dialog (0x0040f220)
              └─► FUN_00410050   (full jig detection sequence)
```

`start_jig_detect_dialog` is a virtual method of `AutoCalibrateWindowClass`
(vtable slot `0xBC` / index 47). `execute_cut_job` sets the operation bracket
(`SET 0x1109 = 0x00`) before calling into this chain.

---

## Functions

### FUN_0040ff60 — direction-consistent contact extremum

Picks the first contact position consistent with the probe approach direction.
Used to reject bounce/overtravel artefacts from three repeated touches.

```c
int select_contact(int pos, int touch1, int touch2, int direction) {
    if (direction < 0) return max(pos, touch1, touch2);  // approaching from +: take highest
    if (direction > 0) return min(pos, touch1, touch2);  // approaching from -: take lowest
    return pos;
}
```

### FUN_0040fbf0 — triple-contact surface probe

Probes one surface three times and returns the measured position:

1. Moves to the approach position
2. Calls `mdx_move_dest_query_position_slow_78_neg1` three times at ±1 mm offsets,
   recording each contact position
3. Per axis, calls `FUN_0040ff60` to select the direction-consistent extremum
4. Writes the result back into the output `DestXYZA`

### FUN_00410050 — full jig detection sequence

Top-level caller. Receives two nominal pin positions plus probe geometry, then:

1. `spindle_stop_and_wait` — spindle must be off before probing
2. Probe both sides of two registration pins using `FUN_0040fbf0`:
   - Pin 1: touch +Y side → `local_b0`, then −Y side → `local_70`
   - Pin 2: touch +Y side → `local_c0`, then −Y side → `local_80`
3. Bisect to find each pin's Y centre:
   ```
   pin1_centre_Y = (local_b0.nY + local_80.nY) / 2
   pin2_centre_Y = (local_70.nY + local_c0.nY) / 2
   ```
4. Probe the top face (Z height) of each pin with a Z-direction vector
5. Subtract probe tip radius (`iVar3 = param_4 / 2`) to get actual pin centre Z
6. Call `MdxAction_x3803` with the two measured positions → writes detected jig origins to device

---

## Summary Table

| Function | Role |
|----------|------|
| `FUN_0040ff60` | Pick direction-consistent extremum of 3 touch readings |
| `FUN_0040fbf0` | Triple-contact probe of one surface; returns measured position |
| `FUN_00410050` | Full jig detection: probe 2 registration pins (Y-bisect + Z-height), write origins |
