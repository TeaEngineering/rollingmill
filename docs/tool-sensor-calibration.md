# VPanel — Tool Sensor Location Adjustment

Function: FUN_004182c0
As shown at https://youtu.be/UKR7mRjQrUs?si=-HaAmA-wAzin0NZD&t=413
Example run: (290.2, 266.5) → (290.5, 266.5)

This is supposed to be done using the supplied detection pin in the 6mm spindle collet, but any flat ended rod would do.

The function:
  1. Guards on FUN_00430acf() and FUN_00417e20() (precondition checks)
  2. Saves current XY from param_1->field_0x68/0x6c, calls MdxAction_x3a03_p1 (sets work coordinate origin)
  3. Calls MdxAction_x3a00_p1(0xffffffff) (rapid jog enable)
  4. Waits for jog busy bits, then reads current position into local_8c via query_0x30e_pos

  Then the movement sequence:

  iVar4 = current_Z - 35000          ← safe clearance height

  1. Waypoint  Z→iVar4                (rapid lift to clearance)
  2. abs_move  Z→iVar4-10000          (slow descent, speed 0x3c=60)
  3. Waypoint  Z→iVar4-10500          (creep to probe height, speed 300)

  ── X-axis probing ──
  4. Waypoint  X+18000, cancel-ok     (probe +X edge)   iVar1 = touched X
  5. Waypoint  Z→iVar4                (lift)
  6. Waypoint  X→origin               (return to centre, safe height)
  7. Waypoint  Z→probe_height         (descend again)
  8. Waypoint  X-18000, cancel-ok     (probe -X edge)
  9. Waypoint  Z→iVar4                (lift)
     iVar1 = (+X_touch) + (-X_touch)  ← sum of both edges

  ── Y-axis probing ──
  10. Waypoint  X→origin, Z→iVar4     (return home, safe)
  11. Waypoint  Z→probe_height        (descend)
  12. Waypoint  Y-18000, cancel-ok    (probe -Y edge)   iVar2 = touched Y
  13. Waypoint  Z→iVar4               (lift)
  14. Waypoint  Y→origin              (return, safe)
  15. Waypoint  Z→probe_height        (descend)
  16. Waypoint  Y+18000, cancel-ok    (probe +Y edge)
  17. Waypoint  Z→iVar4               (lift)

  ── Compute centre ──
  centre_X = iVar1 / 2
  centre_Y = (iVar2 + local_a0.nY) / 2

  Then it moves the tool to the computed centre, stores it in param_1->toolsensor_X and param_1->toolsensor_Y, and calls MdxAction_x3a03_p1 again to set that as the new work coordinate origin.

  This is the automatic tool height sensor centre detection routine — it probes both sides of the trapezoidal sensor in X and Y (±18 mm = 36 mm jig diameter), averages the two touch points on each axis to find the true centre, and saves that to the machine. The send_waypoint_allowing_cancel calls are the actual probing moves where the machine will stall/touch the jig edge.

