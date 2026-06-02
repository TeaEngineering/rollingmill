"""
Raw USB transport layer for Roland MDX-40A.

`MdxUSB` is the real-device link. Construct via `MdxUSB.discover()`, which:
  1. Finds the MDX-40A USB device (VID=0x0B75, PID=0x03DB),
  2. Detaches any kernel driver and claims interface 0,
  3. Caches the bulk-OUT endpoint,
  4. Issues the two USB printer-class steps (GET_DEVICE_ID, SOFT_RESET) —
     these are the only handshake steps that need the interface number.

`MdxMockUSB` is the in-process double. It implements the same `MdxLink`
interface as `MdxUSB` but never touches real USB — methods short-circuit
and (optionally) emit trace lines. Use it in unit tests and `--mock` UIs
by constructing it directly: `MDX40A(MdxMockUSB())`.

The remaining handshake (ping, endian check, keepalive, firmware-id read,
calibration reads, spindle RPM read) is `MDX40A.__init__`'s responsibility.
"""

import logging
from typing import Any, Optional, Protocol

import usb.core
import usb.util

log = logging.getLogger(__name__)


# ── Link Protocol ────────────────────────────────────────────────────────────

class MdxLink(Protocol):
    """The interface that any USB transport must implement to be usable by
    `MDX40A`. Implemented by `MdxUSB` (real device) and `MdxMockUSB` (test
    double). Duck-typed; no runtime enforcement, but new methods on `MdxUSB`
    must be mirrored on `MdxMockUSB` or unit tests will break.
    """

    def vend_set(self, wValue: int, data: bytes = b"") -> None: ...
    def vend_get(self, wValue: int, length: int) -> bytes: ...
    def bulk_write(self, data: bytes) -> int: ...

    def release(self) -> None: ...
    def __enter__(self) -> "MdxLink": ...
    def __exit__(self, *exc: Any) -> None: ...


# ── Real-device link ─────────────────────────────────────────────────────────

class MdxUSB:
    """Real-device USB link. Owns the libusb device handle, the claimed
    interface number, and the cached bulk-OUT endpoint."""

    VID = 0x0B75
    PID = 0x03DB

    # bmRequestType values for vendor control transfers on EP0
    _T_SET = 0x40   # host→device, vendor, device recipient
    _T_GET = 0xC0   # device→host, vendor, device recipient
    _BREQUEST = 0x01
    _WINDEX = 0x0000

    def __init__(self, dev: Any, intf_num: int, ep_out: Any, timeout:int):
        self._dev = dev
        self._intf_num = intf_num
        self._ep_out = ep_out
        self._timeout = timeout

    # ── Factory ──────────────────────────────────────────────────────────────

    @classmethod
    def discover(cls, timeout:int=2000) -> "MdxUSB":
        """Find, open and initialise the MDX-40A USB link.

        find_device → claim interface 0 → cache bulk-OUT EP → GET_DEVICE_ID →
        SOFT_RESET. Raises RuntimeError if the device is not found or has no
        bulk-OUT endpoint.
        """
        dev = usb.core.find(idVendor=cls.VID, idProduct=cls.PID)
        if dev is None:
            raise RuntimeError(
                f"MDX-40A not found (VID=0x{cls.VID:04X} PID=0x{cls.PID:04X}). "
                "Is it powered on?"
            )

        intf_num = dev[0][(0, 0)].bInterfaceNumber
        try:
            if dev.is_kernel_driver_active(intf_num):
                dev.detach_kernel_driver(intf_num)
        except usb.core.USBError:
            pass
        usb.util.claim_interface(dev, intf_num)
        log.info(
            "MDX-40A link bus=%d addr=%d serial=%r interface=%d",
            dev.bus, dev.address, dev.serial_number, intf_num,
        )

        ep_out = _find_bulk_out(dev)
        if ep_out is None:
            usb.util.release_interface(dev, intf_num)
            raise RuntimeError("No bulk-OUT endpoint found on MDX-40A")

        link = cls(dev, intf_num, ep_out, timeout)
        link._printer_class_handshake()
        return link

    def _printer_class_handshake(self) -> None:
        """The two USB printer-class steps that use the interface number.

        On Windows USBPRINT.SYS issues these during device enumeration;
        on macOS/Linux with libusb they must be sent explicitly. Failure
        is non-fatal — some firmware revisions don't implement them.
        """
        # GET_DEVICE_ID (bmRequestType=0xA1, bRequest=0x00)
        try:
            data = self._dev.ctrl_transfer(
                0xA1, 0x00, 0x0000, self._intf_num, 1024, timeout=2000,
            )
            device_id = bytes(data).decode('ascii', errors='replace')
            log.info("GET_DEVICE_ID: %s", device_id)
        except usb.core.USBError:
            pass

        # SOFT_RESET (bmRequestType=0x21, bRequest=0x02)
        try:
            self._dev.ctrl_transfer(
                0x21, 0x02, 0x0000, self._intf_num, 0, timeout=2000,
            )
        except usb.core.USBError:
            pass

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def release(self) -> None:
        """Release the claimed interface. Safe to call multiple times."""
        if self._dev is None:
            return
        try:
            usb.util.release_interface(self._dev, self._intf_num)
        except usb.core.USBError:
            pass
        self._dev = None

    def __enter__(self) -> "MdxUSB":
        return self

    def __exit__(self, *_: Any) -> None:
        self.release()

    # ── Transfer primitives ──────────────────────────────────────────────────

    def vend_set(self, wValue: int, data: bytes = b"") -> None:
        """Vendor control OUT (VEND_SET_CMD). data=b'' for a bare trigger.

        RE: deviceioctl_write_short/long — RD25D driver always prepends a 4-byte
        header to the USB data stage: [bRequest=0x01, wValue_hi, wValue_lo, 0x00].
        For the long path (>= 3 bytes) the driver builds this from the 3-byte IOCTL
        InBuffer + a padding byte; for the short path it is embedded in InBuffer
        directly (nInBufferSize = nBytes + 4).  Either way the device sees:
            [0x01, wValue_hi, wValue_lo, 0x00] + payload
        """
        header = bytes([0x01, (wValue >> 8) & 0xFF, wValue & 0xFF, 0x00])
        wire_data = header + data
        self._dev.ctrl_transfer(
            self._T_SET, self._BREQUEST, wValue, self._WINDEX, wire_data,
            timeout=self._timeout,
        )

    def vend_get(self, wValue: int, length: int) -> bytes:
        """Vendor control IN (VEND_GET_CMD). Returns array of `length` bytes."""
        result = self._dev.ctrl_transfer(
            self._T_GET, self._BREQUEST, wValue, self._WINDEX, length,
            timeout=self._timeout,
        )
        return bytes(result)

    def bulk_write(self, data: bytes) -> int:
        """Write raw bytes to the bulk-OUT endpoint."""
        return int(self._ep_out.write(data, timeout=self._timeout))


# ── Mock link (test/UI-no-hardware double) ───────────────────────────────────

class MdxMockUSB:
    """In-process mock that implements the `MdxLink` interface without
    touching real USB. Transfers short-circuit:

      - vend_set: no-op.
      - vend_get: zero-filled buffer. Special cases:
          wValue=0x0002 → b'\\x00\\x00\\x12\\x34' (endian sig — MDX40A handshake check).
          wValue=0x0001 → b'\\x00\\x00\\x00\\xff' (ping: byte[3]=0xff means
              "response of length 255 is ready"). Allows `MDX40A.pattern_b_read`
              to terminate immediately when running against the mock —
              otherwise the polling loop would spin until poll_timeout.
      - bulk_write: returns len(data).
    """

    def __init__(self) -> None:
        log.info("MdxMockUSB created")

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def release(self) -> None:
        pass

    def __enter__(self) -> "MdxMockUSB":
        return self

    def __exit__(self, *_: Any) -> None:
        pass

    # ── Transfer primitives ──────────────────────────────────────────────────

    def vend_set(self, wValue: int, data: bytes = b"") -> None:
        pass

    def vend_get(self, wValue: int, length: int) -> bytes:
        if wValue == 0x0002:
            return (b"\x00\x00\x12\x34" + b"\x00" * length)[:length]
        if wValue == 0x0001 and length >= 4:
            # Ping: byte[3]=0xff = "max response ready" so MDX40A.pattern_b_read exits the loop.
            return b"\x00\x00\x00\xff"
        return bytes(length)

    def bulk_write(self, data: bytes) -> int:
        return len(data)


# ── Internal helpers ─────────────────────────────────────────────────────────

def _find_bulk_out(dev: Any) -> Optional[Any]:
    """Return the first bulk-OUT endpoint exposed by the device, or None."""
    for cfg in dev:
        for intf in cfg:
            for ep in intf:
                if (
                    usb.util.endpoint_direction(ep.bEndpointAddress)
                    == usb.util.ENDPOINT_OUT
                    and usb.util.endpoint_type(ep.bmAttributes)
                    == usb.util.ENDPOINT_TYPE_BULK
                ):
                    return ep
    return None


# ── CLI smoke test (real device only) ────────────────────────────────────────

def main() -> None:
    """Open the real USB layer: discover the device, then release.
    No machine-level handshake.

    """
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")
    with MdxUSB.discover():
        pass
    print("released ok")


if __name__ == "__main__":
    main()
