# Sending NC/RML Code to the MDX-40A

Reverse engineered from VP_MDX40A.exe (Cut dialog and transport layer).

---

## Cut Dialog (dialog ID 0x1004)

Managed by class with:
- Listbox at control ID `0x300` (ordered NC file paths)
- NC preview control at object offset `+0x274`
- Machine pointer at `dialog+0x5c`

Registry persistence: `OutputFiles` key with `FileCount` and `File_0`, `File_1`, … subkeys.

### Button handlers

| Action      | Function       | Notes |
|-------------|----------------|-------|
| Init dialog | `FUN_004074e0` | Loads file list from registry; enables/disables buttons |
| Add         | `FUN_004077f0` | Opens file dialog, filter `*.rml; *.prn; *.nc; *.ncd; *.nct; *.txt` |
| Delete      | `LB_DELETESTRING` | — |
| Delete All  | `FUN_00407930` | `LB_RESETCONTENT` |
| Move Up     | `FUN_00407960` | Delete + insert at index−1 |
| Move Down   | `FUN_00407a20` | Delete + insert at index+1 |
| Save List   | `FUN_00407d80` | Saves `.ofl` file via MFC `CArchive`; mutex `VPANEL_OUTPUT_FILE_LIST` |
| View List   | `FUN_00407e70` | Loads `.ofl` into listbox |
| Output      | `FUN_00407fc0` | Main send loop (see below) |

---

## Bulk Output Flow (FUN_00407fc0)

1. Save file list to registry (`FUN_00407280`)
2. Get `RolandDeviceSession = *(*(dialog+0x5c) + 0x5c)`
3. Read ping flags → if `flags_byte[0] & 0x10` → show error `0x80F`, abort
4. For each file in listbox (index 0..N−1):
   a. If "Paused at Each File" checkbox set: show pause dialog `0x809`; cancel → stop
   b. Build docname: `sprintf("VPanel %d/%d", i+1, N)`  (string at `0x0044c2f8`)
   c. `FUN_0041bfa0(RolandDeviceSession, &filename, docname)`
5. Hide dialog (`FUN_0042e56d`)

### Per-file send (FUN_0041bfa0 @ 0x0041bfa0)

1. `CFile::Open(filename, 0x20)` — binary read, `SHARE_DENY_NONE`
2. Call `transport->vtable[4](file, docname)` = `FUN_0041c9c0`:
   - Get serialiser from `file->vtable[6]`
   - Write docname to IPC shared memory (`FUN_0041de70`):
     - `strlen(docname)` → copy to `shared_mem + 0x24`
     - `SetEvent(transport+0x1c)` — signal helper thread
     - `WaitForSingleObject(transport+0x20, 500 ms)`

### send_to_printer (FUN_0041d3c0 — transport vtable[23] / offset +0x5c)

Inner loop that transfers file bytes over USBPRINT:

```
printer_name = object+8  (or "RemoteController" if null)

OpenPrinterA(printer_name, &hPrinter, NULL)
StartDocPrinterA(hPrinter, level=1,
    DOC_INFO_1 { pDocName="VPanel N/M", pOutputFile=NULL, pDatatype="RAW" })
StartPagePrinter(hPrinter)

while True:
    cbBuf = CFile::Read(buffer, 0x8000)   // 32 KB chunks
    if cbBuf == 0: break
    ptr = buffer
    while cbBuf > 0:
        WritePrinter(hPrinter, ptr, cbBuf, &written)
        ptr   += written
        cbBuf -= written

EndPagePrinter(hPrinter)
EndDocPrinter(hPrinter)
ClosePrinter(hPrinter)
```

---

## Transport Architecture

Two parallel transports in VPanel:

1. **USBPRINT** (`send_to_printer`) — `OpenPrinterA` → `WritePrinter` → `USBPRINT.sys` → USB bulk-OUT.
   Printer name is the device's USBPRINT port name (stored at `RolandDeviceSession+8`),
   or `"RemoteController"` as fallback.

2. **USB Simulator IPC** — `OpenFileMappingA("Global\RemoteController <serial>")` +
   9 named events (`USB Simulator BulkOut Request/Acknowledge`, etc.) for in-process simulation.

### Language monitor — rdlm64.dll

Sits between the spooler and `USBPRINT.SYS`. Its port functions wrap the USB writes:

- `pfnStartDocPort` — sends the **Head blob** (hex-encoded bytes from registry key `Head`) to
  USB bulk-OUT before any file data.
- `pfnWritePort` (`DoDIfferentKindsOfWrite` @ `0x180003B40`) — passes data to the
  <!-- TODO: confirm symbol name in rdlm64.dll once loaded into Ghidra -->
  `rd25dlf64.dll` filter (if loaded), then to `USBPRINT pfnWrite` → USB bulk-OUT.
- `pfnEndDocPort` — sends the **Tail blob** (registry key `Tail`) after all data.

### Filter — rd25dlf64.dll

Transparent to all normal bytes:
- Buffers each chunk in `GlobalAlloc`.
- Scans byte-by-byte; copies each byte to output unmodified.
- Only intercepts `\x03<CommandName>,...;` (ETX-prefixed sequences) for `ToolInfo` and
  `NextPage` tool-change dialogs; those are re-injected verbatim after signalling the dialog.
- A `"PUF"` prefix triggers full passthrough with zero parsing.
- **Adds no framing to the bulk channel.** RML-1/G-code bytes arrive at USB unmodified.

The filter is completely transparent to firmware status. LogsRead and LogsWrite do exactly two things: pass bytes straight through, and intercept `\x03<Command>,...;` escape sequences. There is no USB status read, no ping, no error check anywhere in the filter. The only "error" behaviour is that dialog_canceled causes LogsRead to fast-forward read_pos = data_len (discards the rest of the current write buffer). The filter cannot detect firmware rejection of illegal coordinates.

#### Tool change — ToolInfo escape sequence
Tool-change interception is data-embedded, not a separate USB command. The NC file must contain:

    \x03ToolInfo,<toolNo>,<XYspeed>,<Zspeed>,<FillPitch>,<ZDown>,<ZUp>,<ZEngPitch>,<ColorBits>,<SpindleRPM>;\n

rd25dlf64.dll `LogsRead` accumulates bytes after `\x03` until `;`, then calls `LogsDialogToolInfo` which parses the comma-separated fields and shows the "Change Tool" dialog. Dialog result codes: 7 = OK/continue, 2 = cancel job.

#### "Prompt to continue" — NextPage escape sequence

Same mechanism:

    \x03NextPage,<pageNumber>;\n

Shows: "Outputs N Page. Change work, please." Dialog: 7 = continue, 2 = cancel.



---

## Command-set selection (RML-1 vs NC)

The MDX-40A firmware parses incoming bulk-OUT data in one of **three** modes,
matching the three radio buttons on the Setup → Modeling Machine tab. The
selection is stored in `byte[1]` of the 6-byte machine-config struct accessible
via GET 0x3106 / SET 0x3107. Mapping derived from the dialog's MFC `DDX_Radio`
binding (control-ID range handler at `0xFD3..0xFD5` in message map `0x0043ecb0`,
plus the enable-check `0 < value < 3` at `FUN_00402af0`):

| `byte[1]` | Setup radio button | Behaviour |
|---|---|---|
| `0` | **RML-1** (IDC `0xFD3`) | RML-1 only. Sending G-code stalls the NC bytes-processed counter and the firmware raises an error bit. |
| `1` | **NC Code** (IDC `0xFD4`) | NC Code only. Sending RML-1 stalls the counter and the firmware raises an error bit. |
| `2` | **Selected automatically (RML-1/NC Code)** (IDC `0xFD5`) | Auto-detect: the firmware picks the parser based on the program's first byte (`%` or `(` → NC, otherwise RML-1). Safest default. |

Called from `write_ncode_settings` @ `0x00402a20` when the Setup-tab NC/RML selection changes.

Mirrored in `mdx40a/machine.py` as the constants `MDX40A.CMDSET_RML1`,
`CMDSET_NC`, `CMDSET_AUTO`.

### Symptom of the wrong mode

Sending `%\r\nO00000001\r\n...` while in mode 0 (RML-1) advances the NC
bytes-processed counter to 3 (the `%\r\n` is accepted as ASCII data) and then
stops. Subsequent bulk writes return success at the USB layer but the firmware
sets an error bit and stops advancing the counter, so `CutJob._service_ack`
times out with `have 3, expected 14` (or similar). The same happens if you send
RML-1 in mode 1 (NC Code only). Mode 2 accepts either format.

### Read-modify-write switch — `write_ncode_settings @ 0x00402a20`

VPanel applies the mode change via the Setup dialog. The disassembly is a plain
read-modify-write of the 6-byte struct:

```c
cfg = query_0x3106_6bytes(machine);             // GET 0x3106, 6 bytes
if (cfg.nc_rml_flags != desired) {
    cfg.nc_rml_flags = desired;                 // byte[1] = 0 or 1
    send_axis_config_0x3107(machine, &cfg);     // SET 0x3107, 6 bytes
    wait_ping_bit21_clear(machine);
}
```

Mirrored in `mdx40a/machine.py` as `MDX40A.set_command_set(mode)`. The setting
is persisted by the firmware across power cycles.

---

## Operation Bracket — SET 0x1109 (critical)

`execute_cut_job` @ `0x00416360` calls `send_operation_bracket_0x1109` with `0x00` **before**
any NC data is sent, and `0xFF` **after** the job completes.

```
SET wValue=0x1109, data=[0x00]   // begin — firmware enables NC data acceptance
... bulk-OUT NC/RML data ...
SET wValue=0x1109, data=[0xFF]   // end
```

Without the begin bracket the firmware silently ignores all bulk NC data.

The same bracket wraps other interactive operations (jog, move-to, detect jig) in VPanel.

---

## Stepped / Test Cut Mode

Fundamentally different from bulk output: treats the file as discrete blocks, sends one at a
time, and synchronises with the machine between each block.

### File parsing — FUN_0040bc92 @ 0x0040bc92

- Loads entire file into RAM.
- Scans for `\r` or `\n` → records offset as block boundary.
- NC code detected if first non-empty block starts with `%` or `(`.
- Test Cut is NC-only; if "Command Set = RML-1" is selected, the Test button is disabled.

### Per-block send — `nc_send_one_slice` @ 0x00405b40

```
before   = GET wValue=0x0200, 4 bytes big-endian   // bytes-processed counter
expected = before + block_byte_length              // stored at AutoClass33+0x74 (active_coordsys)

// Send via IPC shared memory (FUN_0041dd50):
copy block_text → shared_memory[write_offset + 6]
write length    → shared_memory[write_offset + 4]
SetEvent(transport+0x0c)                    // signal transport thread
WaitForSingleObject(transport+0x10, 500 ms) // wait for ACK

// Completion poll (FUN_00405bc0 / FUN_00405bf0):
poll: GET wValue=0x0200 == expected?
```

The firmware counter at `wValue=0x0200` (4-byte big-endian uint32) tracks bytes consumed from
its internal NC buffer. Step completion is detected when the counter reaches `before + len(block)`.

### Bytes-processed counter — `get_nc_bytes_processed_0x200` @ 0x0041c290

**Direct vendor-IN GET** at `wValue=0x0200, len=4`, 4 bytes big-endian → uint32. Issued through
the device session's `read_status_op` vtable slot (`amc_lock_vtable +0x18`) with arguments
`(wValue, buf, len)`. Disassembly confirms a single transfer — no preceding SET, no ping
polling, **not Pattern B**. The firmware treats `returned_len == 4` as success.

Three sites consume it:

| Address | Role | Predicate |
|---|---|---|
| `0x00405b40` | `nc_send_one_slice` | reads counter → `AutoClass33.active_coordsys = counter + slice_len` → `send_data_slice_USB(slice_base, slice_len)` |
| `0x00405bc0` | step-mode is-done | `byte[+0x71] != 0xff && counter == active_coordsys` |
| `0x00405bf0` | run-mode is-done | extra status-byte gate (see below) |

The `active_coordsys` field name in Ghidra is misleading — at this point in the cut-job state
machine it holds the **expected post-send counter value**, not a coordinate system.

### Run-mode status byte — `AutoClass33+0x71`

`FUN_00405bf0` (run-mode is-done predicate) reads the byte at `AutoClass33+0x71` — the second
byte of the `user_cancelled_op` uint32 at offset 112 — and gates advancement on it:

```c
byte s = AutoClass33+0x71;
if (s == 0xff)             return 0;      // aborted/cancelled — never advance
if (s & 0x40)              return 0;      // pause-pending  (operator action required)
if (counter != expected)   return 0;      // firmware not yet done with last block
return (s >> 7);                          // bit 7 = "operator confirmed, advance OK"
```

Summary of bits:

| Byte value / mask | Meaning |
|---|---|
| `0xff` (whole byte) | Aborted — stops the stream permanently |
| bit 6 (`0x40`) | "Pause pending" — operator action required, do not advance |
| bit 7 (`0x80`) | "Continue" — operator confirmed; combined with `counter == expected`, advance |

The handshake matches `rdlm64.dll`'s `\x03ToolInfo,...;` and `\x03NextPage,...;` escape
sequences: when those appear in the data stream, the filter raises a dialog (codes 7=continue,
2=cancel), and the operator click toggles the bits here. Step mode skips bit 6/7 entirely —
each `next_block()` press from the operator IS the "continue" signal — so its predicate
(`FUN_00405bc0`) only checks the cancel sentinel and counter equality.

### Cut-job dialog state table — `0x0043f370`

The cut/test-cut dialog runs a four-state machine; each row is a `{on_enter, tick, on_exit,
finalize}` vtable. The tick at `FUN_00405680` (row 3) demonstrates the step-vs-run branch via
`AutoClass33.run_mode_byte`:

```c
tick(p):
  if (p->run_mode_byte == 0) {            // STEP
      if (!is_slice_done_step(p))   return p;     // counter wait
      update_progress_ui(p);
      if (!send_one_slice(p))       return p;     // dispatch next block
  } else {                                 // RUN
      if (!is_slice_done_run(p))    return p;     // counter + status-byte wait
  }
  finalize(p);
```

The streaming senders in rows 0 and 2 (`FUN_00405540`) use the run-mode predicate and call
`nc_send_one_slice` after each ack — i.e. **one NC block per tick, gated on
`counter == prev + len(block)`** even in run mode. There is no multi-block bulk burst here;
VPanel's "32 KB" appears only at the upper `WritePrinter` boundary (see Bulk Output Flow),
and is split block-by-block before reaching the USB.

---

## Python Implementation Notes

`mdx40a/usb.py::bulk_write()` is the direct equivalent of `WritePrinter` on macOS/Linux.

- NC/RML data is sent **raw** (verbatim binary) in chunks — no framing, no protocol headers.
- The docname `"VPanel N/M"` is Windows spooler metadata and not needed for direct USB.
- The Head/Tail blobs from `rdlm64.dll` are not yet known; their content may include RML-1
  initialisation sequences (e.g. `!MC0;`). Investigate registry on a Windows install.

### Required call sequence

```python
machine.set_command_set(1)      # SET 0x3107 byte[1]=1  ← put firmware in NC mode
machine.begin_nc_job()          # SET 0x1109 = 0x00     ← mandatory bracket open
for block in blocks:            # one CR/LF-terminated NC line per bulk-OUT
    before = machine.get_nc_bytes_processed()
    machine.bulk_write(block)
    expected = (before + len(block)) & 0xFFFFFFFF
    while machine.get_nc_bytes_processed() != expected:
        time.sleep(0.005)
machine.end_nc_job()            # SET 0x1109 = 0xFF
```

Pre-send check (mirrors `FUN_00407fc0` step 3):

```python
ping = machine._ping_status()
if ping != -1 and (ping & 0x00100010):   # bit 4 = device busy/error
    raise RuntimeError("Machine not ready for NC output")
```

`CutJob` in `mdx40a/cutjob.py` implements this loop — see `_send_block` and `_service_ack`.
It handles the bracket automatically and sends one block per `service()` tick, gated on the
counter. The mode switch is the caller's responsibility (do it once at connect).

### Direct-USB hosts must throttle on the counter

On Windows VPanel relies on USBPRINT.sys + rdlm64.dll for flow control: `WritePrinter` accepts
32 KB at a time, the language monitor and printer-class driver buffer and split it into
firmware-sized chunks invisibly. A direct-USB client (us, on macOS/Linux) has none of that
machinery, so a bare 32 KB bulk-OUT write hangs the endpoint after the firmware's first-block
buffer fills.

`CutJob` mirrors VPanel's `nc_send_one_slice` loop: one CR/LF-terminated block per tick, gated
on `counter == prev + len(block)`. The 32 KB chunking path was removed in favour of this; see
the `Required call sequence` snippet above for the equivalent stand-alone loop.



## NC Code on the machine

* Uses workspace coordinate origins (read from machine)
* uses tool offsets 1-9
* uses current spindle speed (unless overritten by M...)
