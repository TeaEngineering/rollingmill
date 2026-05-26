# USB Protocol Reference

All machine control uses USB vendor control transfers with `bRequest=0x01`.
NC/RML file data goes over the bulk-OUT endpoint — no framing, raw bytes only.

**Byte order:** All multi-byte values from the device are big-endian. VPanel applies
byte-swap helpers (`FUN_0041b6a0` uint32, `FUN_0041b680` uint16, `FUN_0041b730` uint32 array) if the machine identification value does not read back as 0x1234.

---

## Transfer Patterns

### Pattern A — Direct GET (no prior trigger)

```
GET wValue → response bytes
```

Used for registers that are always ready (ping, coordinates, machine type).

### Pattern B — Trigger SET → poll → GET 0x0003

```
SET wValue          (primes the device)
poll GET 0x0001     until ping[3] (C LE high byte) is non-zero = response length
GET 0x0003          (retrieve response of that many bytes)
```

All status/config queries use Pattern B. The polling step is mandatory — firing
`GET 0x0003` immediately after the SET returns zero bytes.

---

## Pattern A Commands

| wValue | Name | Response | Notes |
|--------|------|----------|-------|
| `0x0001` | `probe_device_ping` | 4 bytes LE uint32 | Status bits — see [machine-state.md](machine-state.md) |
| `0x0002` | `detect_machine_type` | 4 bytes | High word `0x1234` → MDX-40A confirmed |
| `0x0003` | `dev_read_response` | N bytes | Fetch asynchronous data response |
| `0x0100` | `get_status_0x100` | 32 bytes BE | Machine state flags + XYZA position + spindle RPM — see [machine-state.md](machine-state.md) |
| `0x0200` | `get_nc_bytes_processed` | 4 bytes BE uint32 | Bytes of NC data consumed by firmware; used for stepping through NC code — see [Sending NC/RML Code](sending-nc-code.md) |

---

## Pattern B Commands

### Keepalive / poll

| wValue | Name | Response | Notes |
|--------|------|----------|-------|
| `0x03f5` | `poll_keepalive` | 1 byte | Sent every 200 ms; payload is 1 byte (value uninitialised in VPanel) |
| `0x0101` | `get_ascii_str_0x101` | ≤256 bytes | ASCII model/firmware string |

### Machine status

| wValue | Name | Response | Notes |
|--------|------|----------|-------|
| `0x2001` | `trigger_read_0x2001` | variable | Status flags byte; bit 2 and bit 3 have distinct meanings |
| `0x2010` | `get_uint32_0x2010` | 4 bytes | Single uint32 status |
| `0x2100` | `get_uint16arr_0x2100` | variable | Array of uint16 values (byte-swapped) |
| `0x3800` | `get_status_byte_0x3800` | 1 byte | Machine status byte — polled every 200 ms |
| `0x3b01` | `query_0x3b01_var` | 4 bytes | Variable-length read; used for poll timer sync |

### Speed / feed

| wValue | Name | Response | Notes |
|--------|------|----------|-------|
| `0x3003` | `get_uint32_0x3003` | 4 bytes | Current spindle/feed speed; clamped to `[0x3005.min, 0x3005.max]` |
| `0x3005` | `get_uint32_pair_0x3005` | 8 bytes | Speed range `[min_speed, max_speed]`; stored at `obj[0x21]`/`obj[0x22]` |

### Spindle

| wValue | Name | Response | Notes |
|--------|------|----------|-------|
| `0x2405` | `get_status_struct_0x2405` | 16 bytes | `uint32[0]` = total spindle rotation time in seconds |
| `0x3900` | `get_uint32_0x3900` | 4 bytes BE | Configured spindle target RPM; range 4500–15000 |

### Coordinate systems / WCS

| wValue | Name | Response | Notes |
|--------|------|----------|-------|
| `0x030a` | `query_0x30a` | 16 bytes | Motion step target position `[XYZA]`; SET before move, GET to confirm |
| `0x030b` | `query_0x30b` | 16 bytes | WCS1 work origin offsets `[XYZA]` |
| `0x3202` | `query_0x3202` | 16 bytes | WCS2 work origin offsets `[XYZA]` |
| `0x3203`–`0x3208` | `query_axis_params_range` | 16 bytes each | WCS3–8 origins (4×uint32) |
| `0x3209`–`0x3334` | `query_param_table_range` | 16 bytes each | WCS9–309 entries; see [Coordinate Systems](coordinate-systems.md) |
| `0x030d` | `query_0x30d` | 16 bytes | Move waypoint lower bound |
| `0x030e` | `query_0x30e` | 16 bytes | Move waypoint upper bound; `element[3]` (A axis) patched before motion planning |

### Axis configuration / tool offsets

| wValue | Name | Response | Notes |
|--------|------|----------|-------|
| `0x3106` | `query_0x3106_6bytes` | 6 bytes | `byte[1]` ∈ {1,2} checked for firmware mode |
| `0x346b`–`0x3472` | `query_indexed_0x346a` (index 1–8) | 4 bytes each | Read tool diameter offsets (8 slots) |
| `0x3701` | `get_uint32_0x3700` | 4 bytes | Single uint32 |
| `0x3801` | `get_rotary_axis_centreline_0x3801` | 12 bytes | 3×uint32 = stored rotary A-axis centreline `[X, Y, Z]` in 1/1000 mm. Only `Y, Z` define the line (X is "along" the rotation axis, so its value is informational only). Written by the jig-detect routine via `SET 0x3803`. Read by the "Current Jig" indicator in the main panel — see [vpanel-jig-detect.md](vpanel-jig-detect.md#current-jig-indicator). |
| `0x3804` | `get_6uint32_0x3804` | 24 bytes | 6×uint32 — busy/status block; `word[0] bit 2 (0x4)` gates jog commands |
| `0x3a02` | `get_2uint32_0x3a02` | 8 bytes | 2×uint32 |
| `0x3a05` | `get_uint32_0x3a05` | 4 bytes | Used in spindle-stop to override Z for positioning |
| `0x05f0` | `get_axis_angles_0x5f0` | 32 bytes | 8×uint32 = 4 `(value, scale)` pairs → rotation angles XYZA |

---

## SET Commands (host → device)

All SET commands are vendor control OUT transfers with `bRequest=0x01`.
Payloads are given in Python `struct.pack` notation.

### Motion

| wValue | Payload | Notes |
|--------|---------|-------|
| `0x04f5` | `>HH4i` — speed, `0x0000`, dx, dy, dz, da | **Relative jog.** Deltas in 1/1000 mm (A: 1/1000°). Interactive jogging. |
| `0x04f6` | `>HH4i` — `0xFFFF`, `0x0000`, dx, dy, dz, da | **Relative waypoint.** Max-speed step; used in multi-step milling sequences. |
| `0x04f7` | `>HH4i` — speed, `0xFFFF`, absX, absY, absZ, absA | **Absolute move.** Absolute machine coords in 1/1000 mm. |
| `0x1109` | 1 byte: `0x00` = begin, `0xFF` = end | **Operation bracket.** Must wrap all jog, move-to, and NC job sequences. |

All three jog/move commands share the same 20-byte `'>HH4i'` payload layout:

```
bytes  0–1   uint16  speed  (mm/min; 0xFFFF = firmware max)
bytes  2–3   uint16  flags  (0x0000 = relative delta, 0xFFFF = absolute target)
bytes  4–7   int32   X      (1/1000 mm, signed)
bytes  8–11  int32   Y
bytes 12–15  int32   Z
bytes 16–19  int32   A      (1/1000 degree, signed)
```

### Spindle

| wValue | Payload | Notes |
|--------|---------|-------|
| `0x03f0` | (bare) | Spindle ON |
| `0x03f1` | (bare) | Spindle OFF |
| `0x3006` | `>I` — RPM uint32 | Set spindle speed (0 = off). |
| `0x3008` | 1 byte (10–200) | Spindle speed override % |
| `0x3009` | (bare) | Spindle stop; followed by `wait_busy_bits_clear` in drilling sequences |
| `0x3808` | `<HH` — RPM uint16, mode=2 | NC S-word spindle speed; sent during NC job only |
| `0x3809` | `<HH` — [1, 0xFFFF] start / [0, 0] stop | Rotary A-axis drilling mode (continuous slow rotation) |
| `0x3901` | `>I` — RPM uint32 | Write spindle target RPM (4500–15000); poll ping bit 21 clear after |

### Feed rate

| wValue | Payload | Notes |
|--------|---------|-------|
| `0x0307` | 1 byte (10–200) | Cutting feed rate override %; range 10–200 |

### Coordinate systems / WCS

| wValue | Payload | Notes |
|--------|---------|-------|
| `0x03f2` | (bare) | Latch current position as WCS origin (also "resume") |
| `0x03f3` | (bare) | Unknown? |
| `0x030c` | `>4i` — X, Y, Z, A (1/1000 mm) | Write WCS1 origin |
| `0x3335`–`0x333d` | `>4i` — X, Y, Z, A | Write WCS2–10 origins |

### Axis configuration / tool offsets

| wValue | Payload | Notes |
|--------|---------|-------|
| `0x3107` | 6 bytes | Set axis configuration; poll ping bit 21 clear after |
| `0x347b`–`0x3482` | `>I` — uint32 (index 1–8) | Write tool diameter offsets; poll ping bit 21 clear after each |
| `0x2012` | 2×uint32 (8 bytes) | Write motion limits pair; poll ping bit 21 clear after |
| `0x3468` | 1 byte (bool) | Toggle "Optional Block Skip" on/off |

### Housekeeping

| wValue | Payload | Notes |
|--------|---------|-------|
| `0x03f5` | 1 byte | Keepalive / poll trigger; send every 200 ms |
| `0x2425` | (bare) | Reset spindle rotation time counter to zero; poll ping bit 21 clear after |
| `0x0200` | (trigger) | Prime NC bytes-processed counter read (Pattern B) |

---

## 200 ms Poll Sequence

VPanel's timer fires every 200 ms and sends (all Pattern B unless noted):

```
SET 0x03f5  (keepalive — 1 byte payload)
SET 0x3005 → GET 0x0003  (speed range min/max)
SET 0x3800 → GET 0x0003  (machine status byte)
SET 0x3003 → GET 0x0003  (current speed value)
SET 0x3b01 → GET 0x0003  (timer sync)
GET 0x0100              (Pattern A — XYZA coordinates + state flags)
```

Note: XYZA coordinates (`GET 0x0100`) are Pattern A, not triggered by the keepalive.

---

## Startup Handshake

Performed once at connection:

```
USB class GET_DEVICE_ID   (bmRequestType=0xA1, bRequest=0x00, wValue=0, wIndex=intf)
USB class SOFT_RESET      (bmRequestType=0x21, bRequest=0x02)
GET 0x0001                (ping — confirm device responding)
GET 0x0002                (machine type — expect high word 0x1234)
SET 0x03f5  b'\x00'       (keepalive)
SET 0x3804 → GET 0x0003   (device status block — 6×uint32)
SET 0x3900 → GET 0x0003   (read configured spindle RPM)
```

---

## Ping bit 21 — command acknowledgement

Several SET commands require waiting for the firmware to acknowledge before
sending the next command. Poll `GET 0x0001` until bit 21 (`0x00200000`) clears,
3-second timeout (`wait_ping_bit21_clear` @ `FUN_0041b930`):

Commands that require this wait: `SET 0x3901`, `SET 0x2425`, `SET 0x3107`,
`SET 0x347b`–`SET 0x3482`, `SET 0x2012`.
