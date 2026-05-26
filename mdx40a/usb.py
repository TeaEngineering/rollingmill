"""
Raw USB transport layer for Roland MDX-40A.

No Roland semantics here — just the four primitives the protocol needs.
All functions raise usb.core.USBError on failure.

Mock mode
---------
Call set_mock() before find_device() to run without a physical device.
Every read returns a zero-filled buffer of the requested length; writes
are discarded.  find_device() returns a _MockDevice sentinel that satisfies
all attribute accesses made by machine.py.
"""

import time
from typing import Optional

import usb.core
import usb.util

from . import trace as _trace

VID = 0x0B75
PID = 0x03DB

# bmRequestType values for vendor control transfers on EP0
_T_SET = 0x40   # host→device, vendor, device recipient
_T_GET = 0xC0   # device→host, vendor, device recipient
_BREQUEST = 0x01
_WINDEX   = 0x0000

# ── Mock mode ─────────────────────────────────────────────────────────────────

_mock = False


class _MockDevice:
    """Returned by find_device() in mock mode.  Satisfies all accesses in machine.py."""

    manufacturer  = 'Roland DG'
    product       = 'MDX-40A (mock)'
    serial_number = 'MOCK0000'
    bus     = 0
    address = 0

    def ctrl_transfer(self, bmRequestType, bRequest, *args, **kwargs):
        # IN transfers (bit 7 set): return zeroed buffer of requested length.
        if bmRequestType & 0x80:
            length = args[2] if len(args) > 2 else 0
            return bytearray(int(length))
        return 0


_MOCK_DEV = _MockDevice()


def set_mock(enabled: bool = True) -> None:
    """Enable or disable mock (no-USB) mode.  Call before find_device()."""
    global _mock
    _mock = enabled


# ── Device lifecycle ──────────────────────────────────────────────────────────

def find_device():
    """Return the first MDX-40A USB device found, or None."""
    if _mock:
        return _MOCK_DEV
    return usb.core.find(idVendor=VID, idProduct=PID)


def claim(dev):
    """Detach kernel driver if needed and claim interface 0."""
    if _mock:
        return 0
    intf_num = dev[0][(0, 0)].bInterfaceNumber
    try:
        if dev.is_kernel_driver_active(intf_num):
            dev.detach_kernel_driver(intf_num)
    except usb.core.USBError:
        pass
    usb.util.claim_interface(dev, intf_num)
    return intf_num


def release(dev, intf_num):
    """Release a previously claimed interface."""
    if _mock:
        return
    try:
        usb.util.release_interface(dev, intf_num)
    except usb.core.USBError:
        pass


# ── Transfer primitives ───────────────────────────────────────────────────────

def vend_set(dev, wValue: int, data: bytes = b'', timeout=2000):
    """Vendor control OUT (VEND_SET_CMD). data=b'' for a bare trigger.

    RE: deviceioctl_write_short/long — RD25D driver always prepends a 4-byte
    header to the USB data stage: [bRequest=0x01, wValue_hi, wValue_lo, 0x00].
    For the long path (>= 3 bytes) the driver builds this from the 3-byte IOCTL
    InBuffer + a padding byte; for the short path it is embedded in InBuffer
    directly (nInBufferSize = nBytes + 4).  Either way the device sees:
        [0x01, wValue_hi, wValue_lo, 0x00] + payload
    Trace logging records the logical payload (without header).
    """
    if _mock:
        return 0
    t = _trace.get_active()
    header = bytes([0x01, (wValue >> 8) & 0xFF, wValue & 0xFF, 0x00])
    wire_data = header + data
    try:
        result = dev.ctrl_transfer(_T_SET, _BREQUEST, wValue, _WINDEX, wire_data, timeout=timeout)
        if t:
            t.log_set(wValue, data)
        return result
    except Exception as exc:
        if t:
            t.log_error('SET', wValue, exc)
        raise


def vend_get(dev, wValue: int, length: int, timeout=2000):
    """Vendor control IN (VEND_GET_CMD). Returns array of `length` bytes."""
    if _mock:
        return bytearray(length)
    t = _trace.get_active()
    try:
        result = dev.ctrl_transfer(_T_GET, _BREQUEST, wValue, _WINDEX, length, timeout=timeout)
        if t:
            t.log_get(wValue, bytes(result))
        return result
    except Exception as exc:
        if t:
            t.log_error('GET', wValue, exc)
        raise


def trigger_read(dev, trig_wvalue: int, length: int, timeout=2000) -> bytes:
    """Pattern B: SET trigger wValue to prime device, then GET wValue=0x0003.

    Returns the GET response as `bytes` (pyusb's raw `array('B', ...)` wrapped
    so logs and downstream consumers see a clean b'\\x00...' representation).
    """
    if _mock:
        return b'\x00' * length
    vend_set(dev, trig_wvalue, timeout=timeout)
    return bytes(vend_get(dev, 0x0003, length, timeout=timeout))


def trigger_read_b(dev, wValue, max_length, poll_timeout=0.200, timeout=2000) -> Optional[bytes]:
    """Pattern B with ping polling (RE: dev_trigger_read @ 0x0041bb90).

    SET wValue → poll GET 0x0001 until ping[3] (C LE *uint32 >> 24) is non-zero
    (= firmware-reported response length) → GET 0x0003 of that many bytes.
    Returns bytes on success, None on timeout or device error.
    """
    if _mock:
        return bytearray(max_length)
    vend_set(dev, wValue, b'', timeout=timeout)
    deadline = time.monotonic() + poll_timeout
    while time.monotonic() < deadline:
        ping = vend_get(dev, 0x0001, 4, timeout=timeout)
        if len(ping) >= 4:
            if ping[2] & 0x10:      # bit 20 = device error
                return None
            length = ping[3]        # C LE high byte = response length
            if length:
                return vend_get(dev, 0x0003, min(int(length), max_length), timeout=timeout)
        time.sleep(0.005)
    return None


def bulk_write(dev, data, timeout=2000):
    """Write raw bytes to the bulk-OUT endpoint."""
    if _mock:
        return len(data)
    ep_out = None
    for cfg in dev:
        for intf in cfg:
            for ep in intf:
                if (usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_OUT
                        and usb.util.endpoint_type(ep.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK):
                    ep_out = ep
                    break
    if ep_out is None:
        raise usb.core.USBError("No bulk-OUT endpoint found")
    return ep_out.write(data, timeout=timeout)
