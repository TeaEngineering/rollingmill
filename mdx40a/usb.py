"""
Raw USB transport layer for Roland MDX-40A.

No Roland semantics here — just the four primitives the protocol needs.
All functions raise usb.core.USBError on failure.
"""

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


def find_device():
    """Return the first MDX-40A USB device found, or None."""
    return usb.core.find(idVendor=VID, idProduct=PID)


def claim(dev):
    """Detach kernel driver if needed and claim interface 0."""
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
    try:
        usb.util.release_interface(dev, intf_num)
    except usb.core.USBError:
        pass


def vend_set(dev, wValue, data=b'', timeout=2000):
    """Vendor control OUT (VEND_SET_CMD). data=b'' for a bare trigger."""
    t = _trace.get_active()
    try:
        result = dev.ctrl_transfer(_T_SET, _BREQUEST, wValue, _WINDEX, data, timeout=timeout)
        if t:
            t.log_set(wValue, bytes(data))
        return result
    except Exception as exc:
        if t:
            t.log_error('SET', wValue, exc)
        raise


def vend_get(dev, wValue, length, timeout=2000):
    """Vendor control IN (VEND_GET_CMD). Returns array of `length` bytes."""
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


def trigger_read(dev, trig_wvalue, length, timeout=2000):
    """Pattern B: SET trigger wValue to prime device, then GET wValue=0x0003."""
    vend_set(dev, trig_wvalue, timeout=timeout)
    return vend_get(dev, 0x0003, length, timeout=timeout)


def bulk_write(dev, data, timeout=2000):
    """Write raw bytes to the bulk-OUT endpoint."""
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
