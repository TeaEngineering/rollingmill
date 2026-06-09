RollingMill
======

<img src="logo.png" width="35%" align="left" alt="RollingMill logo">

Open-source USB control stack for Roland DG MDX-series desktop mills.

RollingMill communicates directly with the machine over libusb, replacing both the proprietary Windows kernel driver and the VPanel control application.

The project currently provides:

* Live machine state monitoring
* Axis position readback
* Variable speed Jogging
* Spindle control and Rotary Axis Drilling mode
* Interactive terminal UI (TUI)
* Cross-platform support (Linux, macOS, Windows)

Reverse Engineering
------
The MDX USB protocol was reverse engineered from the official Windows driver stack using Ghidra analysis and protocol inspection. See [the protocol documentation](docs/). It should be obvious that the Roland DG corporation neither authorised or approved of this work.

The official Windows stack consists of:

* RD25D — kernel-mode USB and printer filter driver
* VPanel — user-space control application

RollingMill implements both layers in Python, and works great on macOS.

Getting started
-----

This library is available from PyPi as `rollingmill`.

    $ pip install rollingmill
    $ rollingmill                                            # launch TUI on real hardware
    $ rollingmill --file sample-nc/example-doc-part.gcode    # stream an NC file



Gotchas/Notes using the mill
-----

* If your NC-code 'crashes' the Z position above zero (i.e. at the top of the machine), then the machine's interpreter starts running rapidly flattened against Z=0 (all feeds become rapid moves, descending is inhibited) until the end of the program. This is quite terrifying should it happen, but is a machine feature.
* If you send NC-code (i.e. G-code) in RML-1 mode, it will hang. Recovery for now is a power-cycle.
* Single-stepping nc-code _partially works_, but the MDX complains about being starved of NC code and lights the View-LED when you reach a cutting operation.
* The progress display during a cutting job is that of filling the machine buffer, rather than the actual block being executed, which usually runs a few blocks behind. It's a shame there are not two counters so that this could be made 100% accurate.
* There might be an issue with storing A-axis values in workspace offsets and recalling them, as sometimes I get an extra +360° rotation on recall.
* I need a Z-origin sensor to test the sensor-based Z-origin setting, and decode the thickness settings — NYI.
* The geometry of the mill means that without tooling, it is fairly difficult (but not impossible) to crash. The rotary axis makes this much easier as the firmware allow the collet to hit the vice area.
* Be warned that bad, terrible things could happen driving your hardware with this experimental software, due to known or unknown bugs or defects, for which you entirely agree to take all risks in operating and hold the contributors blameless.


MDX GCode Intepreter Notes
----
* G-codes must be two digits (`G01`, `G00` etc)
* Dont forget to turn the spindle on (Sxxxx M03) otherwise the machine will fault on the first `G01` operation
* Ensure all `XYZ` values have a decimal point, and no more than 3 digits of precision
* Feed rates need a decimal point too
* Machine coordinate moves `G53` must be always given in `G90` absolute programming. Specifying G53 in G91 incremental results in an error.


ZCL-40A 4th Axis
----
When this unit is installed, the X-, Y-, and Z-axis travel of the MDX-40A is reduced:

| Axis | MDX-40A     | MDX-40A + ZCL-40A      |
|------|-------------|------------------------|
| X    | 0 → 305 mm  | 34 → 305 mm            |
| Y    | 0 → 305 mm  | 0 → 305 mm             |
| Z    | 0 → −105 mm | 0 → −68 mm             |
| A    | —           | 0 → 360° (continuous)  |

These limits are enforced in firmware, including from the physical jog controls on the machine itself. The X-restriction clears the rotary axis body, but not the rotary vice or tailstock, so care is needed.


Engraving text
-----

`rollingmill.text_to_gcode` renders a text string to a single-line g-code file using a Hershey stroke font from the `pyhershey` library. Glyphs carry per-character advance-width metrics, so spacing is naturally kerned. The output is plain RS-274 (lines only — no arcs) intended to be streamed via the TUI's `--file` flow.

```
python -m rollingmill.text_to_gcode \
    --text "PART-001" \
    --out /tmp/label.gcode \
    --height 8 \
    --z-cut -0.1 --z-safe 2 \
    --feed 200 \
    --workspace G54
```

`--workspace G54..G59` is optional; when set, the matching WCS-select word is emitted in the header so the cut runs against that work-offset.

Then load on the machine: `rollingmill --file /tmp/label.gcode`.

The default font is `roman_simplex`. List all available fonts with `--list-fonts`; pass any name (e.g. `--font gothic_german_triplex`) to switch. Use `--letter-spacing 1.5` to add (or, with a negative value, remove) extra mm of gap after each glyph's natural advance.
