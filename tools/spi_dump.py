#!/usr/bin/env python3
"""Zero-dependency Switch 2 SPI-flash reader over the USB vendor interface (iface 1).

Uses usbdevfs ioctls directly (no pyusb, no root needed once the udev rule is in
place). Frame format (from nsogcd spi_tool.py / BlueRetro sw2.c):

    02 91 00 04 00 08 00 00 <len> 7e 00 00 <addr LE32>

Usage:
    ./spi_dump.py [--pid 0x2069] <offset_hex> <length>
    ./spi_dump.py --pid 0x2069 13000 64
"""
import ctypes, fcntl, glob, os, struct, sys, time

VID = 0x057E
EP_OUT, EP_IN = 0x02, 0x82
VENDOR_IF = 1
CLAIM   = 0x8004550F
RELEASE = 0x80045510
BULK    = 0xC0185502

INIT_DEFAULT = bytes([0x03,0x91,0x00,0x0d,0x00,0x08,0x00,0x00,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF])
INIT_SET_LED = bytes([0x09,0x91,0x00,0x07,0x00,0x08,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00])


class Bulk(ctypes.Structure):
    _fields_ = [("ep", ctypes.c_uint), ("len", ctypes.c_uint),
                ("timeout", ctypes.c_uint), ("data", ctypes.c_void_p)]


def devnode(pid):
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            if int(open(d+"/idVendor").read(),16) != VID: continue
            if int(open(d+"/idProduct").read(),16) != pid: continue
            b = int(open(d+"/busnum").read()); n = int(open(d+"/devnum").read())
            return f"/dev/bus/usb/{b:03d}/{n:03d}"
        except (OSError, ValueError):
            pass
    return None


def bulk(fd, ep, data, length, to):
    buf = ctypes.create_string_buffer(bytes(data) if data else b"", length)
    x = Bulk(ep=ep, len=length, timeout=to, data=ctypes.cast(buf, ctypes.c_void_p))
    try:
        n = fcntl.ioctl(fd, BULK, x); return buf.raw[:n]
    except OSError:
        return b""


def drain(fd):
    for _ in range(8):
        if not bulk(fd, EP_IN, None, 64, 30): break


def spi_read(fd, offset, length):
    cmd = bytearray([0x02,0x91,0x00,0x04, 0x00,0x08,0x00,0x00, length & 0xFF, 0x7e,0x00,0x00])
    cmd += struct.pack('<I', offset)
    while len(cmd) < 16: cmd.append(0)
    drain(fd)
    bulk(fd, EP_OUT, cmd, len(cmd), 1000)
    for _ in range(6):
        r = bulk(fd, EP_IN, None, 64, 1500)
        if r and len(r) >= 4 and r[0] == 0x02 and r[1] == 0x01:
            return r
    return None


def hexdump(base, data):
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hx = " ".join(f"{b:02x}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"  {base+i:08x}  {hx:<47}  {asc}")


def main():
    pid = 0x2069
    args = sys.argv[1:]
    if args and args[0] == "--pid":
        pid = int(args[1], 0); args = args[2:]
    if len(args) < 2:
        print("usage: spi_dump.py [--pid 0xXXXX] <offset_hex> <length>"); sys.exit(1)
    offset = int(args[0], 16); total = int(args[1])

    dn = devnode(pid)
    if not dn: sys.exit(f"controller pid {pid:#06x} not found")
    fd = os.open(dn, os.O_RDWR)
    try:
        fcntl.ioctl(fd, CLAIM, struct.pack("I", VENDOR_IF))
        bulk(fd, EP_OUT, INIT_DEFAULT, len(INIT_DEFAULT), 1000); time.sleep(0.05)
        bulk(fd, EP_OUT, INIT_SET_LED, len(INIT_SET_LED), 1000); time.sleep(0.05)

        print(f"pid={pid:#06x}  SPI read 0x{offset:08x}..0x{offset+total:08x}\n")
        got = bytearray(); first_raw = None
        addr = offset
        while len(got) < total:
            n = min(0x30, total - len(got))
            r = spi_read(fd, addr, n)
            if r is None:
                print(f"  0x{addr:08x}: <no response>"); break
            if first_raw is None:
                first_raw = r
                print(f"raw first response ({len(r)}B): {r.hex(' ')}\n")
            # data payload sits after the 16-byte echo header
            payload = r[16:16+n]
            got += payload
            addr += n
        print("decoded payload:")
        hexdump(offset, bytes(got))
    finally:
        try: fcntl.ioctl(fd, RELEASE, struct.pack("I", VENDOR_IF))
        except OSError: pass
        os.close(fd)


if __name__ == "__main__":
    main()
