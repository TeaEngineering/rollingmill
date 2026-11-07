# Z0 sensor probing

Select **User (RML-1)** with `c`, close the coordinate dialog, then press **b**.
Place the clean connected sensor on the workpiece, position the tool directly
above it, turn off the spindle, and close the cover. **Y** explicitly confirms
and starts the descent/retract cycle; **Esc** dismisses the confirmation or
requests an immediate stop during motion. The TUI keeps polling and displaying
Z/feed. Other machine commands are blocked until a fresh idle reply arrives.

The firmware updates the User Z origin; the application then reads slot 1 again
and refreshes its coordinate display. A cancellation, device error, lost status,
unobserved motion or timeout is reported as an unverified result, never success.
Quit during an active cycle requests stop; quit again after idle to exit.

Only mode 1 (User/RML-1) has live-capture evidence. Other WCS selections are
rejected. This feature uses the firmware's existing sensor thickness/settings;
it does not write a guessed thickness. There is no NC streaming during probing;
launch without `--file`. The fresh state must be idle, with cover closed and no
error or spindle-on flag.

## Protocol evidence

A passive VPanel/USB-driver capture from a physical MDX-40A showed:

1. SET 0x03F5 payload 02, operation begin SET 0x1109 payload 00.
2. Stop spindle: SET 0x3006 uint32 big-endian zero.
3. Twice: Pattern B query 0x2001, two-byte reply 00 14. Bit 2 of byte 1
   satisfies the connected check; bit 3 is the contact condition.
4. SET **0x3902**, payload **00 01 FF FF FF FF** (`>HHH`, mode 1, -1, -1).
5. Direct GET 0x0100 32-byte status polls during descent/retraction. During
   this phase the TUI does not send its normal idle heartbeat/secondary reads.
6. Motion flags clear, then re-read the User origin with Pattern B 0x030B.

`MdxUSB.vend_set` already prepends the vendor header. Its initiation data stage
is **01 39 02 00 00 01 FF FF FF FF**; do not add a second header in the feature.
Operation cleanup restores 0x03F5/0x1109 to FF, matching VPanel's operation exit.

The captured cycle descended at 60 mm/min, retracted at 1800 mm/min and ended
in about 16.9 seconds. Those speeds are firmware-controlled, not hard-coded
motion commands. A lowest sampled Z of -79.880 mm preceded reversal; exact
contact instant was between samples. No text success response is sent.

Tests replay the real status samples in `tests/fixtures/mdx40a_zprobe_status.json`,
verify the exact USB initiation bytes, and exercise sensor, state, timeout,
disconnect, cancellation and TUI interlocks. The new application feature has
not itself been motion-tested on hardware yet.
