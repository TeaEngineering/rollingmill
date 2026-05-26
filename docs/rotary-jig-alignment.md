# Rotary Jig Detection Algorithm

The A-axis calibration rod routine finds and stores the position of the A-axis rotation
centre in XYZ machine space, using a detection bar and detection pin fitted to the rotary table.

Reference video: https://youtu.be/UKR7mRjQrUs?t=934

The stored value is queried to show if the current workplace origin is on-centerline or not.

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
   - Pin 1: touch −Y side → `sense_x1`, then +Y side → `local_80`
   - Pin 2: touch −Y side → `sense_x2`, then +Y side → `local_c0`
3. Bisect to find each pin's Y centre:
   ```
   pin1_centre_Y = (sense_x1.nY + local_80.nY) / 2
   pin2_centre_Y = (local_c0.nY + sense_x2.nY) / 2
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

---

## Current Jig Indicator

The main panel and two of the move-to sub-dialogs (`FUN_0040eea0`, `FUN_0040e760`)
embed a custom-painted picture control bound to bitmap resource **`0x1ee0`** — a
5-frame sprite strip showing the progressive installation state of the rotary
A-axis attachment. User-facing copy:

> **(10) Current Jig** — This displays the detected jig. When the origin point
> has been set at the specified point for the jig, the location of the origin is
> indicated by a red arrow. When you have changed the jig, be sure to click
> **[Detect Jig]**. (This is necessary even when you have removed and reattached
> the same jig.) Clicking the **[Detect Jig]** button displays the **[Jig
> Detection]** dialog box.

In the main panel the control lives at `AutoClass33::field_0xb90`, bound in
OnInitDialog (`FUN_004149d0` @ `0x414e94`):

```c
AutoClass33::FUN_00431673(&this->field_0xb90, 0);             // initially disabled
AutoClass33::FUN_0040b0f0(&this->field_0xb90, 0x1ee0, 5);     // bitmap 0x1ee0, 5 frames
```

The current frame index (0–4) is stored at `control + 0x4c` by the status setter
`update_dialog_display` (`FUN_0040b4b0`), which then `InvalidateRect`s the
control to force a repaint.

### Frame meanings

| Frame | Depicted state |
|-------|----------------|
| **0** | No rotary axis installed |
| **1** | Unknown fixture installed (detected something, not the rotary signature) |
| **2** | Rotary axis installed, but no vice fitted on it |
| **3** | Rotary + vice installed (origin not yet verified) |
| **4** | Rotary + vice installed AND workpiece origin positioned on the centreline (red arrow) |

### Frame selection — `FUN_00414170`

After each "move-to" workflow (and on dialog init), the indicator is refreshed
by `FUN_00414170`. **No motion happens inside this function** — it is a pure
configuration test that walks a chain of preconditions and finally compares two
stored values:

1. `field_0x80 < 1` → **frame 0** (no rotary attachment sensed).
   `field_0x80` is `AutoClass33.is_rotary_axis_installed`, populated every 200 ms by
   `AutoClass33::poll_timer_200ms` from `GET 0x3800` (the extension-port status
   byte) — see [machine-state.md](machine-state.md#get-0x3800--extension-port--rotary-status).
2. `field_0xc30 < 1` → **frame 2** (rotary present, but no vice on it).
3. `coordsys_idx == 2` (G55, the non-rotary WCS) → **frame 3** (rotary + vice
   installed; skip the centreline test because we're not in a rotary WCS).
4. Otherwise: compare the firmware's stored rotary centreline against the
   active WCS origin:
   - `get_rotary_axis_centreline_0x3801(comms, &centreline)` →
     `(X, Y, Z)` calibrated centreline (see [usb-protocol.md](usb-protocol.md)).
   - `query_coord_system_by_index(coordsys_idx, &origin)` →
     current WCS work origin.
   - Quantise both to a 1/100 mm grid and test Y, Z equality. If matched →
     **frame 4** (on centreline); else → **frame 3**.

Frame **1** (unknown fixture) is not produced by `FUN_00414170` itself — it
comes from the **Detect Jig** workflow (`FUN_00410050` and surrounding code)
when probing finds pins in positions that don't match the expected rotary
signature.

### The `MulDiv(x, 1, 10) * 10` tolerance test

Coordinates are in 1/1000 mm. The function quantises both values to a 1/100 mm
grid before equality testing:

```c
iVar3 = MulDiv(centreline.Y, 1, 10);   // round-to-nearest at 10-unit grid
centreline.Y = iVar3 * 10;
iVar3 = MulDiv(centreline.Z, 1, 10);
centreline.Z = iVar3 * 10;
origin.Y = MulDiv(origin.Y, 1, 10) * 10;
origin.Z = MulDiv(origin.Z, 1, 10) * 10;
```

Win32 `MulDiv` rounds to the *nearest* integer (not toward zero as C `/`
would), so the grid is symmetric about zero — the comparison behaves
identically for negative coordinates. The 1/100 mm tolerance absorbs any
sub-tolerance drift between the user's touched-off origin and the
factory/calibrated centreline.

### Why X is excluded

The rotary A-axis rotates *around* the X axis — every X position lies on the
centreline by definition. So only Y and Z carry "is the origin on the line?"
information; the X component of the stored centreline is informational (the
nominal X of the calibration touch-off) and is not part of the equality test.
