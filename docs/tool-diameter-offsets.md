# Tool Diameter Offsets

The MDX-40A stores 8 tool diameter offset slots in firmware. VPanel reads and writes them
as part of the cut job setup flow (`execute_cut_job` → `apply_axis_config_to_device`).

If a tool-diameter offset is not set by G10 in the NC code, in advance of activating Tool Diameter Offset (G41 left, or G42 right) then these stored values are used.

A similar system exists for Tool Length Offset (G43 enables, G49 cancels).

---

## USB Commands

| Operation | wValue range | Count | Width | Index |
|-----------|-------------|-------|-------|-------|
| Read  | `0x346b` – `0x3472` | 8 | uint32 BE | 1–8 |
| Write | `0x347b` – `0x3482` | 8 | uint32 BE | 1–8 |

Read and write use different wValue ranges. Both are Pattern B (trigger SET → GET 0x0003).
Write commands require polling ping bit 21 clear after each one.

---

## RE Details

**Read** — `FUN_00402990` (populates dialog fields from device):

```c
uint32 *p = &AutoDialogClass::field_0x94;
for (int i = 1; i <= 8; i++, p++) {
    query_indexed_0x346a(this, i, p);   // wValue = 0x346a + i
}
```

The 8 values are stored at `AutoDialogClass::field_0x94` through `+0xb0` (32 bytes).

**Write** — `apply_axis_config_to_device`:

```c
if (param_1->field_0xc5 != 0) {   // dirty flag — only write if changed
    uint32 *p = &AutoDialogClass::field_0x94;
    for (int i = 1; i <= 8; i++, p++) {
        MdxAction_x347a__(this, i, *p);   // wValue = 0x347a + i
        wait_ping_bit21_clear();
    }
}
```

`field_0xc5` is the dirty flag — set only when the user modifies a value in the dialog.
The write fires in `apply_axis_config_to_device`, called from `execute_cut_job`'s setup path.
