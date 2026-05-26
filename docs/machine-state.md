# Machine State & Motion Detection

---

## State Block — GET 0x0100 (32 bytes, big-endian)

Read directly via Pattern A (no trigger needed). Returns 32 bytes.

```
Offset  Type      Field
 0      uint32 BE  flags        (see bit table below)
 4      int32  BE  X_pos        (1/1000 mm, signed)
 8      int32  BE  Y_pos        (1/1000 mm, signed)
12      int32  BE  Z_pos        (1/1000 mm, signed)
16      int32  BE  A_pos        (1/1000 degree, signed)
20      uint32 BE  spindle_rpm  (current target RPM)
24–31   (other status, partially decoded)
```

Parse with `struct.unpack_from('>I4iI', data)`. All values in 1/1000 mm (or 1/1000° for A) and are absolute Machine Coordinates. See details to [translate to other coordinate systems](coordinate-systems.md).

### State flags (bytes 0–3, big-endian uint32)

| Bit | Mask | Name | Meaning |
|-----|------|------|---------|
| 28 | `0x10000000` | DOOR | Enclosure door open — jog inhibited while set |
| 27 | `0x08000000` | SPINDLE | Spindle motor on; fires RPM/speed update |
| 26 | `0x04000000` | CMD_MOVE | Motion command currently executing |
| 25 | `0x02000000` | MTR_PWR | Motor powered / front-panel parking brake indicator |
| 22 | `0x00400000` | MOVING | Axis velocity > 0 (actively translating) |
| 18–16 | `0x00070000` | STATE | Machine state enum (see below) |
| 13 | `0x00002000` | BUSY | Motion in progress (motion-complete loop exit gate) |
| 12 | `0x00001000` | ERROR | Error condition |

**STATE enum values** (bits 18–16, shift right 16):

| Value | Meaning |
|-------|---------|
| 0 | Init / unknown |
| 1 | Homing / seeking origin |
| 2 | Idle / normal |
| 3 | Motion commanded → transitions back to 2 on completion |
| 4 | Error |

**Constant bits** (hardware-config, always set at power-on — ignore for status):
bits 23, 17, 11, 4, 3, 2 (`0x00820E1C`)

### Live observations

| Condition | Observed value | Notes |
|-----------|----------------|-------|
| Startup / homing | `0x010A2804` | STATE=1, CMD_MOVE=1, MOVING=1, MTR_PWR=0 |
| Idle (post-home) | `0x0202081C` | STATE=2 |

**Important:** MTR_PWR (bit 25) is NOT set during homing — motor power is NOT required for
motion commands to execute. It reflects the front-panel button state only.


---

## Ping Register — GET 0x0001 (4 bytes, little-endian uint32)

Separate from the state block. Used for motion-complete detection and command acknowledgement.

```
struct.unpack_from('<I', data)[0]
```

### Ping bits

| Bit | Mask | Name | Meaning |
|-----|------|------|---------|
| 22 | `0x00400000` | MOVE_BIT | Axis velocity > 0; asserts when motion starts, clears when axis decelerates to rest |
| 21+2 | `0x00200004` | BUSY_MASK | Firmware motion-complete gate; two consecutive reads clear → done |
| 20 | `0x00100000` | ERROR_MASK | Device error condition |
| 13 | `0x00002000` | ACK_JOG | Transient acknowledgement for `SET 0x4f5` (relative jog) |
| 12 | `0x00001000` | ACK_ABS | Transient acknowledgement for `SET 0x4f7` (absolute move) |
| 21 | `0x00200000` | CMD_ACK | Clears after `SET 0x3901` / `SET 0x2425` / `SET 0x3107` / axis param writes |

Bit 21 appears in both `BUSY_MASK` (combined with bit 2) and `CMD_ACK` (alone); the two names reflect different uses of the same physical bit.

### Steady-state values (live traces, 2026-05-12)

| Condition | Ping value | Notes |
|-----------|-----------|-------|
| Normal idle | `0x00820800` | Bits 23, 17, 11 — hardware config constants |
| After `SET 0x04f5` | `0x00822800` | Bit 13 briefly set, returns to idle |
| After `SET 0x04f7` | `0x00821800` | Bit 12 briefly set, returns to idle |

---

## Motion-Complete Wait

RE: `jog_wait_busy_bits_clear` @ `0x00417b00`, `wait_move_bit_clear` @ `0x0041b8d0`.

Two-phase poll of `GET 0x0001`:

**Phase 1** — wait for bit 22 (MOVE_BIT) to assert then clear (axis started and settled).
Not always needed for short moves that complete before the first poll.

**Phase 2** — wait for two consecutive reads with bits 21+2 (`0x00200004`) both clear.
100 ms between polls, 30-second timeout.

```python
last_clear = False
while monotonic() < deadline:
    sleep(0.100)
    ping = GET_0x0001()
    if ping & 0x00100000:   # ERROR_MASK
        return FAILED
    now_clear = (ping & 0x00200004) == 0
    if now_clear and last_clear:
        return DONE
    last_clear = now_clear
```

---

## NC Bytes-Processed Counter — GET 0x0200

Pattern B trigger read: `SET 0x0200` → poll → `GET 0x0003`, 4 bytes big-endian uint32.

RE: `get_nc_bytes_processed_0x200` @ `0x0041c290`.

The firmware increments this counter as it consumes NC data from its internal buffer.
Used for step-completion detection in stepped/test-cut mode:

```python
before   = GET_0x0200()
bulk_write(block)
expected = (before + len(block)) & 0xFFFFFFFF
poll until GET_0x0200() == expected   # 10-second timeout, 20 ms interval
```

See [Sending NC code](sending-nc-code.md) for usage.

---

## GET 0x3804 — Device Status Block

Pattern B, 24 bytes big-endian (6×uint32).

RE: `get_6uint32_0x3804` @ `0x0041ad10`.

`word[0] bit 2 (0x04)`:
- VPanel `FUN_00403e90`: if set, jog commands are suppressed
- VPanel `FUN_004042e0`: used as motion-complete exit condition

Read at startup and before issuing jog commands to check firmware readiness.
