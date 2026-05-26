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

The filter is completely transparent to firmware status. LogsRead and LogsWrite do exactly two things: pass bytes straight through, and intercept `\x03<Command>,...;` escape sequences. There is no USB status read, no ping, no error check anywhere in the filter. The only "error" behaviour is that dialog_canceled causes LogsRead to fast-forward read_pos = data_len (discards the rest of the current write buffer). The filter cannot detect firmware rejection of illegal
  coordinates.

#### Tool change — ToolInfo escape sequence
Tool-change interception is data-embedded, not a separate USB command. The NC file must contain:

    \x03ToolInfo,<toolNo>,<XYspeed>,<Zspeed>,<FillPitch>,<ZDown>,<ZUp>,<ZEngPitch>,<ColorBits>,<SpindleRPM>;\n

rd25dlf64.dll `LogsRead` accumulates bytes after `\x03` until `;`, then calls `LogsDialogToolInfo` which parses the comma-separated fields and shows the "Change Tool" dialog. Dialog result codes: 7 = OK/continue, 2 = cancel job.

#### "Prompt to continue" — NextPage escape sequence

Same mechanism:

    \x03NextPage,<pageNumber>;\n

Shows: "Outputs N Page. Change work, please." Dialog: 7 = continue, 2 = cancel.



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

### Per-block send — FUN_00405b40 @ 0x00405b40

```
before   = GET wValue=0x0200, 4 bytes big-endian   // bytes-processed counter
expected = before + block_byte_length

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

---

## Python Implementation Notes

`mdx40a/usb.py::bulk_write()` is the direct equivalent of `WritePrinter` on macOS/Linux.

- NC/RML data is sent **raw** (verbatim binary) in chunks — no framing, no protocol headers.
- The docname `"VPanel N/M"` is Windows spooler metadata and not needed for direct USB.
- The Head/Tail blobs from `rdlm64.dll` are not yet known; their content may include RML-1
  initialisation sequences (e.g. `!MC0;`). Investigate registry on a Windows install.

### Required call sequence

```python
machine.begin_nc_job()          # SET 0x1109 = 0x00  ← mandatory
machine.bulk_write(nc_data)     # raw RML-1 / G-code bytes, any chunk size
machine.end_nc_job()            # SET 0x1109 = 0xFF
```

Pre-send check (mirrors `FUN_00407fc0` step 3):

```python
ping = machine._ping_status()
if ping != -1 and (ping & 0x00100010):   # bit 4 = device busy/error
    raise RuntimeError("Machine not ready for NC output")
```

`CutJob` in `mdx40a/cutjob.py` handles the bracket automatically via `begin_nc_job()` /
`end_nc_job()` calls around the first bulk write and the DONE/ERROR exit paths.
