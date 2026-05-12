RollingMill
======

Open-source USB control stack for Roland MDX-series desktop mills.

RollingMill communicates directly with the machine over libusb, replacing both the proprietary Windows kernel driver and the VPanel control application.

The project currently provides:

* Direct USB communication with the machine
* Live machine state monitoring
* Axis position readback
* Status and state flag decoding
* Interactive terminal UI (TUI)
* Cross-platform support (Linux, macOS, Windows)

Reverse Engineering
------
The Roland USB protocol was reverse engineered from the official Windows driver stack using Ghidra analysis and protocol inspection.

The official Windows stack consists of:

* RD25D — kernel-mode USB driver
* VPanel — user-space control application

RollingMill implements both layers in pure Python.


ZCL-40A 4th Axis
----
When this unit is installed, the X-, Y-, and Z-axis travel path of the MDX-40A are reduced:

                       X     Y     Z
    MDX-40A           305 x 305 x 105 (mm)
    MDX-40A+ZCL-40A   271 x 305 x 68  (mm)

These limits are enforced in firmware, including from the physical jog controls on the machine itself.