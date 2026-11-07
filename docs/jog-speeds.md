# Jog Speeds

Reverse engineered from `jog_key_timer_handler` @ 0x00417770 in VP_MDX40A.exe.
Data table at DAT_00440a68, 5 rows × 16 bytes (stride = `step_mode × 16`).
Each row: `[+0] int32 XYZ_delta  [+4] int32 A_delta  [+8] int32 XYZ_speed  [+12] int32 A_speed`.
Units: 1/1000 mm (or 1/1000 ° for A). Speed: mm/min, or `0xFFFF` = firmware maximum.

| step_mode | VPanel UI label | XYZ delta   | A delta     | XYZ speed       | A speed         |
|-----------|-----------------|-------------|-------------|-----------------|-----------------|
| 0         | "1 Step"        | 10 (0.010 mm) | 10 (0.010°) | 0xFFFF (max)  | 0xFFFF (max)    |
| 1         | "10 Step"       | 100 (0.100 mm) | 100 (0.100°) | 0xFFFF       | 0xFFFF          |
| 2         | "100 Step"      | 1000 (1.000 mm) | 1000 (1.000°) | 0xFFFF     | 0xFFFF          |
| 3         | "Low Speed"     | 500 (0.500 mm) | 500 (0.500°) | 240 mm/min   | 120 mm/min      |
| 4         | "High Speed"    | 5000 (5.000 mm) | 8000 (8.000°) | 1800 mm/min | 3000 mm/min   |

**Step modes (0–2):** all use firmware-maximum speed (`0xFFFF`). The fast/slow UI toggle has no effect; the only variable is displacement magnitude.

**Continuous modes (3–4):** the direction key is held and VPanel sends repeated jog commands on a 100 ms timer. A speed ramp is applied using `jog_fire_counter` (incremented each tick):

* Tick 1: debounce — sends one 0.010 mm step at max speed
* Ticks 2–10: ramp-start (Low Speed row): XYZ = 240 mm/min, A = 120 mm/min
* Ticks 11–40: linear interpolation from Low Speed values toward the target row
* Ticks 41+: constant at target row values

Low Speed is its own target, so it runs at constant 240/120 mm/min with no ramp.
High Speed ramps: XYZ 240 → 1800 mm/min, A 120 → 3000 mm/min over 3 seconds (30 ticks × 100 ms).

**A axis:** step modes are identical to XYZ (same deltas, same 0xFFFF speed). In continuous mode the ramp-start speed is halved (120 vs 240 mm/min) and the High Speed target delta is larger (8.000° vs 5.000 mm per tick). There is no VPanel-level gating on A-axis jog commands; the firmware simply ignores `nA` when no rotary table is installed.

## Rollingmill hold-to-jog

Hold-to-jog is the default. A tap in an ordinary Linux terminal sends a single
selected pulse at its paired constant feed. Holding the direction
key starts continuous jogging after keyboard auto-repeat begins. Press `j`
to switch to fixed-step mode, which uses the same selected pulse/feed pair.

Ordinary terminals use a **200 ms repeat timeout**. The initial keyboard-repeat
delay is accommodated separately (up to one second between the first press
and first repeat). Subsequent repeats renew the timeout without restarting
motion. Once repeats stop, motion stop is requested. A repeat rate
slower than five presses per second can interrupt motion; use the normal
keyboard-repeat setting. Two quick taps are indistinguishable from a hold
and can briefly start continuous motion, bounded by the same timeout.

Actual key-release events are used automatically when a terminal supports
the Kitty keyboard protocol; this is optional. No Windows APIs, privileged
Linux input access or special terminal installation is required for the
repeat fallback. Esc always requests stop. Remaining repeat events after
Esc cannot restart motion until there has been a 200 ms quiet interval.
Queued input events carry receipt timestamps, so old buffered repeats cannot
renew a motion timeout when subsequently handled.

Jog commands refresh every 100 ms without waiting for motion to end. The application applies no speed ramp. Door/error
flags, stale or failed status reads also request stop. The cooperative loop
checks input every 25 ms between USB operations. Release detection therefore
takes up to 200 ms plus loop/USB latency in an ordinary terminal. Physical
smoothness has not yet been validated on the mill.

## Current pulse selector

`f` cycles a single pulse/feed selection, defaulting to 5 mm / 3000 mm/min:

| Pulse (mm) | Feed (mm/min) |
|------------|---------------|
| 0.01 | 6 |
| 0.1 | 60 |
| 1 | 600 |
| 5 | 3000 |

Each pair represents 100 ms of travel. A single press sends the whole selected
pulse; a hold repeatedly sends that same pulse at that same feed, with no
application acceleration ramp. The previous independent speed and numeric
step selectors were removed. Rotary A uses the same numeric pairs in degrees
and degrees/minute. Firmware motor acceleration remains firmware-controlled.
The reverse-engineered VPanel table above is historical protocol evidence;
these application presets are the requested proportional pulse/feed mapping.
