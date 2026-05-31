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

| Value | WCS slot |
|-------|----------|
| `0` | MCS (machine coordinates — origin = `(0,0,0,0)`) |
| `1` | WCS1 |
| `2` | EXOFS (the secondary G54-style offset shown as "EXOFS" in the dropdown) |
| `3`..`9` | WCS3..WCS9 |
| `10`..`309` | Extended WCS slots (not exposed in the standard dropdown) |

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
