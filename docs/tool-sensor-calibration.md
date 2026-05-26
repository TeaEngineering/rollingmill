# VPanel — Tool Sensor Location Adjustment

Function: `FUN_004182c0` (`update_toolsensor_position`).
Reference video: https://youtu.be/UKR7mRjQrUs?si=-HaAmA-wAzin0NZD&t=413
Example run: `(290.2, 266.5) → (290.5, 266.5)`.

The automatic tool-height-sensor centre detection routine. It probes both
sides of the trapezoidal Z-height sensor pad in X and Y (±18 mm = 36 mm jig
diameter), averages the two touch points on each axis to find the true
centre, and writes the result back to the machine. Intended to be run with
the supplied detection pin in the 6 mm spindle collet, but any flat-ended
rod will work.

`send_waypoint_allowing_cancel` is the actual probing primitive — the move
stalls when the tool touches the jig edge, returning the position at contact.

---

## Preconditions

1. `FUN_00430acf()` and `FUN_00417e20()` — guard checks.
2. Save current XY from `param_1->default_toolsensor_X/Y`, then call
   `MdxAction_x3a03_p1` to seed the work coordinate origin.
3. `MdxAction_x3a00_p1(0xffffffff)` — enable rapid jog.
4. Wait for jog busy bits to clear, then read current position into
   `local_8c` via `query_0x30e_pos`.

`iVar4 = current_Z − 35000` — safe clearance height used throughout.

---

## Movement sequence

### Z descent

| Step | Move                              | Notes                            |
|------|-----------------------------------|----------------------------------|
| 1    | Waypoint `Z → iVar4`              | Rapid lift to clearance          |
| 2    | abs_move `Z → iVar4 − 10000`      | Slow descent, speed `0x3c` (60)  |
| 3    | Waypoint `Z → iVar4 − 10500`      | Creep to probe height, speed 300 |

### X-axis probing

| Step | Move                              | Notes                              |
|------|-----------------------------------|------------------------------------|
| 4    | Waypoint `X + 18000`, cancel-ok   | Probe +X edge; `iVar1` = touched X |
| 5    | Waypoint `Z → iVar4`              | Lift                               |
| 6    | Waypoint `X → origin`             | Return to centre, safe height      |
| 7    | Waypoint `Z → probe_height`       | Descend again                      |
| 8    | Waypoint `X − 18000`, cancel-ok   | Probe −X edge                      |
| 9    | Waypoint `Z → iVar4`              | Lift                               |

After step 9: `iVar1 = (+X_touch) + (−X_touch)` — accumulated sum of both edges.

### Y-axis probing

| Step | Move                                  | Notes                              |
|------|---------------------------------------|------------------------------------|
| 10   | Waypoint `X → origin`, `Z → iVar4`    | Return home, safe                  |
| 11   | Waypoint `Z → probe_height`           | Descend                            |
| 12   | Waypoint `Y − 18000`, cancel-ok       | Probe −Y edge; `iVar2` = touched Y |
| 13   | Waypoint `Z → iVar4`                  | Lift                               |
| 14   | Waypoint `Y → origin`                 | Return, safe                       |
| 15   | Waypoint `Z → probe_height`           | Descend                            |
| 16   | Waypoint `Y + 18000`, cancel-ok       | Probe +Y edge                      |
| 17   | Waypoint `Z → iVar4`                  | Lift                               |

---

## Compute centre

```
centre_X = iVar1 / 2                    // (+X_touch + −X_touch) / 2
centre_Y = (iVar2 + local_a0.nY) / 2    // (−Y_touch + +Y_touch) / 2
```

The tool is moved to the computed centre. The values are stored in
`param_1->toolsensor_X` / `param_1->toolsensor_Y`, and
`MdxAction_x3a03_save_toolsensor_location` is called again to write the
centre as the new work coordinate origin.
