# Move Commands

Detailed payload layouts and dispatch behaviour for the Motion SET commands
summarised in [usb-protocol.md](usb-protocol.md#motion).

---

## Jog / move payload — `SET 0x04f5`, `0x04f6`, `0x04f7`

All three commands share the same 20-byte `'>HH4i'` payload:

```
bytes  0–1   uint16  speed  (mm/min; 0xFFFF = firmware max)
bytes  2–3   uint16  flags  (0x0000 = relative delta, 0xFFFF = absolute target)
bytes  4–7   int32   X      (1/1000 mm, signed)
bytes  8–11  int32   Y
bytes 12–15  int32   Z
bytes 16–19  int32   A      (1/1000 degree, signed)
```

| wValue | Use |
|--------|-----|
| `0x04f5` | Relative jog — interactive jogging. |
| `0x04f6` | Relative waypoint — max-speed step inside multi-step milling sequences. |
| `0x04f7` | Absolute move — XYZA in machine coordinates. |

---

## Move-to-origin — `SET 0x3501` (6 bytes, `>HHH`)

```
bytes 0–1   uint16  wcs_code     (which work coordinate system's origin to use)
bytes 2–3   uint16  axis_mask    (bitfield: which axes participate in the move)
bytes 4–5   uint16  speed        (mm/min; 0xFFFF = firmware max, what VPanel always sends)
```

### `wcs_code` — mirrors the Coordinate-System dropdown's item-data


| Dropdown value                | ID | Notes                  |
|-------------------------------|----|------------------------|
| Machine Coordinate System     | 0  | Fixed origin (0,0,0,0) |
| User Coordinate System        | 1  | RML mode only          |
| G54                           | 3  | +EXOFS, NC mode only   |
| G55                           | 4  | +EXOFS, NC mode only   |
| G56                           | 5  | +EXOFS, NC mode only   |
| G57                           | 6  | +EXOFS, NC mode only   |
| G58                           | 7  | +EXOFS, NC mode only   |
| G59                           | 8  | +EXOFS, NC mode only   |
| EXOFS                         | 2  | NC mode only           |

### `axis_mask` — bitfield, multiple bits = simultaneous move

| Bit | Mask | Axis |
|-----|------|------|
| 0 | `0x01` | X |
| 1 | `0x02` | Y |
| 2 | `0x04` | Z |
| 3 | `0x08` | A |

### Observed values from the Move dropdown

| `axis_mask` | Effect |
|-------------|--------|
| `1` | Move X to origin |
| `2` | Move Y to origin |
| `3` | Move XY to origin (simultaneous) |
| `4` | Move Z to origin |
| `8` | Move A to origin (rotary-only entry) |

### Dispatch (VPanel internals)

`move_to_origin_dispatch_x3501 @ 0x00403eb0` dispatches one of ~45 leaf
functions selected by `(wcs_code, axis_mask)`; each leaf builds the 3-uint16
payload above and calls `send_trigger_u16_array(0x3501, ...)`. Invoked from
the Move button (`on_cmd_move_dispatch @ 0x00415ae0`, control ID `0x1fe1`) on
the main panel.

---

## Other motion commands

| wValue | Notes |
|--------|-------|
| `0x03f3` | Motion stop (`send_motion_stop`) — immediately halts in-flight motion. |
| `0x1109` | Operation bracket — payload `0x00` = begin, `0xFF` = end; must wrap all jog, move-to, and NC job sequences. |
| `0x3808` | Move Y to centre of rotary A-axis. Payload `>HH` — mode (`2`), speed (`0xFFFF` = firmware max). Rotary-only entry of the Move dropdown. |
| `0x0500` | Move to View Position. Payload `>H` — speed (`0xFFFF` = firmware max). Parks the machine in the front-of-bed view pose for workpiece load/unload. |

## Rollingmill selected-coordinate origin moves

The TUI's origin presets now resolve the selected slot's stored origin, read
the current position, and send `0x04f7` in machine coordinates. Unselected axes
retain their current position. This matches the selected coordinate display
and avoids `0x3501`'s command-set-mode restrictions. MCS uses zero origin; a
failed origin/position read cancels the move. View Position and rotation-center
presets remain machine-defined locations. Numeric entry already adds the
selected display offset to the entered coordinates.

## Dialog and coordinate conversion update

The Move-To preview/view preset is explicitly labeled MCS. Origin presets and
User Specified numeric targets share `move_to_position`, which reads the
selected coordinate origin and adds it to entered values. Blank numeric
fields preserve the current machine-axis position. If a coordinate origin
cannot be read, selection/movement fails visibly instead of substituting zero.
The machine-defined rotation-center shortcut was removed from this picker;
its machine API remains available separately.

Cooperative overlays now use `noutrefresh` and one final `doupdate` per frame,
so the background and dialog are presented together. Blocking numeric entry
retains its independent refresh. PTY checks found no repeated dialog repaint
output for Move-To, Coordinate Systems and Tool Diameter Offsets while idle.

## Absolute move lifecycle correction

The live session trace `20261004-211007.558.txt` recorded origin targets being
sent successfully, immediately followed by `0x03f2` and `0x1109=FF`; sampled
positions then remained unchanged. Static inspection of VPanel confirms that
it begins interactive operations with `0x03f5=02` and `0x1109=00`, waits for
motion completion after `0x04f7`, then unwinds with `0x03f2`, `0x03f5=FF` and
`0x1109=FF`. Its absolute-move callers use numeric speeds 120/3000.

Rollingmill now keeps an absolute move pending across poll ticks, uses direct
status reads instead of idle heartbeat writes, and defers cleanup until the
move ends. Completion requires the sampled target position (within 0.002 mm
per axis); ended motion at a different position is unverified. Unacknowledged
commands, cancellation and missing status produce a visible failure/status
message. The preset 0xFFFF speed sentinel maps to 3000 mm/min for absolute
moves. Competing TUI operations are blocked while the move is pending; Esc
requests stop and polling continues until idle.

This correction has mocked lifecycle and packet tests; it has not yet been
confirmed by a new physical movement. The trace and static code establish the
protocol mismatch, but hardware validation is still needed to confirm the
cause of the observed no-motion symptom.
