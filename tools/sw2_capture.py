#!/usr/bin/env python3
"""
Nintendo Switch 2 controller protocol capture helper.

Zero external dependencies (pure stdlib: ctypes + usbdevfs ioctls + hidraw
reads). Intended to reverse-engineer the button bit -> name mapping and the
report layout for the in-kernel hid-nintendo2 driver.

What it does:
  1. Finds the USB device for the given PID and sends the proprietary
     initialization handshake to the vendor interface (interface 1) bulk OUT
     endpoint, which makes the controller start emitting usable HID input
     reports on its HID interface (interface 0).
  2. Opens the matching /dev/hidrawN node and streams input reports, decoding
     the button bytes (in binary) and the analog sticks. It highlights which
     button BITS change so you can press one button at a time and read off its
     bit number.
  3. Logs every raw report (hex) to a timestamped file for offline analysis.

Run as root (needs usbdevfs + hidraw access):
    sudo ./sw2_capture.py --pid 0x2069        # Pro Controller
    sudo ./sw2_capture.py --pid 0x2073        # NSO GameCube
    sudo ./sw2_capture.py --pid 0x2067        # Joy-Con 2 (L)
    sudo ./sw2_capture.py --pid 0x2066        # Joy-Con 2 (R)

Press buttons one at a time; note the reported bit numbers. Ctrl-C to stop.
"""
import argparse
import ctypes
import fcntl
import glob
import os
import struct
import sys
import time

VID = 0x057E

# Vendor interface (carries the init handshake) and its bulk endpoints.
VENDOR_INTERFACE = 1
EP_OUT = 0x02
EP_IN = 0x82

# usbdevfs ioctl numbers (see <linux/usbdevice_fs.h>).
USBDEVFS_CLAIMINTERFACE = 0x8004550F
USBDEVFS_RELEASEINTERFACE = 0x80045510
USBDEVFS_BULK = 0xC0185502

# Report byte offsets (data[0] is the report ID), shared across these models.
OFF_BUTTONS = 3
OFF_STICK_L = 6
OFF_STICK_R = 9

# Known button bit -> name, per PID. Pro Controller (0x2069) is confirmed from
# capture; GameCube (0x2073) bits 0..15 match Pro, upper bits tentative;
# Joy-Con 2 (0x2067/0x2066) not yet captured (unknown bits print as '?').
_PRO_NAMES = {
    0: "B", 1: "A", 2: "Y", 3: "X", 4: "R", 5: "ZR", 6: "+",
    7: "RStick", 8: "Down", 9: "Right", 10: "Left", 11: "Up",
    12: "L", 13: "ZL", 14: "-", 15: "LStick", 16: "Home",
    17: "Capture", 18: "SR", 19: "SL", 20: "C",
}
# GameCube: bits 0..4,6,8..13 match the Pro; but bit 5 is the classic Z button,
# and the GameCube has no stick clicks (7/15), Minus (14) or SR/SL (18/19).
# Upper bits: 16=Home, 17=Capture, 20=C (GameChat).
_GC_NAMES = {k: _PRO_NAMES[k] for k in
             (0, 1, 2, 3, 4, 6, 8, 9, 10, 11, 12, 13)}
_GC_NAMES.update({5: "Z", 16: "Home", 17: "Capture", 20: "C"})
# Joy-Con 2 (L), 0x2067: bits 0..8 confirmed; SL/SR tentative. Bit 16 is a
# constant status flag, not a button.
_JCL_NAMES = {
    0: "Down", 1: "Right", 2: "Left", 3: "Up", 4: "L", 5: "ZL",
    6: "-", 7: "LStick", 8: "Capture", 9: "SL?", 10: "SR?",
    16: "(flag)",
}
# Joy-Con 2 (R), 0x2066: bits 0..8 and 12 confirmed; SL/SR tentative; bit 16
# is the constant status flag. C (GameChat) is at bit 12 here.
_JCR_NAMES = {
    0: "B", 1: "A", 2: "Y", 3: "X", 4: "R", 5: "ZR", 6: "+",
    7: "RStick", 8: "Home", 9: "SL?", 10: "SR?", 12: "C", 16: "(flag)",
}
BIT_NAMES = {
    0x2069: _PRO_NAMES,
    0x2073: _GC_NAMES,
    0x2067: _JCL_NAMES,
    0x2066: _JCR_NAMES,
}

# Proprietary init sequence (sent to vendor bulk OUT). Mirrors the driver's
# nx2_init_seq; several commands have unknown semantics.
INIT_SEQ = [
    bytes([0x03, 0x91, 0x00, 0x0D, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00,
           0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
    bytes([0x07, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]),
    bytes([0x16, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]),
    bytes([0x15, 0x91, 0x00, 0x01, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x02,
           0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
           0xFF, 0xFF]),
    bytes([0x15, 0x91, 0x00, 0x02, 0x00, 0x11, 0x00, 0x00, 0x00, 0xFF,
           0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
           0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
    bytes([0x15, 0x91, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00]),
    bytes([0x09, 0x91, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    bytes([0x0C, 0x91, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x27, 0x00,
           0x00, 0x00]),
    bytes([0x11, 0x91, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00]),
    bytes([0x0A, 0x91, 0x00, 0x08, 0x00, 0x14, 0x00, 0x00, 0x01, 0xFF,
           0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x35, 0x00, 0x46,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    bytes([0x0C, 0x91, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x27, 0x00,
           0x00, 0x00]),
    bytes([0x03, 0x91, 0x00, 0x0A, 0x00, 0x04, 0x00, 0x00, 0x09, 0x00,
           0x00, 0x00]),
    bytes([0x10, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]),
    bytes([0x01, 0x91, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00]),
    bytes([0x03, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00]),
    bytes([0x0A, 0x91, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x03, 0x00,
           0x00]),
    bytes([0x09, 0x91, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
]


class BulkTransfer(ctypes.Structure):
    _fields_ = [
        ("ep", ctypes.c_uint),
        ("len", ctypes.c_uint),
        ("timeout", ctypes.c_uint),
        ("data", ctypes.c_void_p),
    ]


def find_usb_devnode(pid):
    """Return (devnode_path, sysfs_path) for the VID:pid USB device."""
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(d, "idVendor")) as f:
                if int(f.read().strip(), 16) != VID:
                    continue
            with open(os.path.join(d, "idProduct")) as f:
                if int(f.read().strip(), 16) != pid:
                    continue
            with open(os.path.join(d, "busnum")) as f:
                bus = int(f.read().strip())
            with open(os.path.join(d, "devnum")) as f:
                dev = int(f.read().strip())
            return f"/dev/bus/usb/{bus:03d}/{dev:03d}", d
        except (OSError, ValueError):
            continue
    return None, None


def find_hidraw(pid):
    """Return /dev/hidrawN for the controller's HID interface."""
    for hid in glob.glob(f"/sys/bus/hid/devices/0003:{VID:04X}:{pid:04X}.*"):
        for hr in glob.glob(os.path.join(hid, "hidraw", "hidraw*")):
            return "/dev/" + os.path.basename(hr)
    return None


def usb_bulk(fd, ep, data, length, timeout_ms):
    buf = ctypes.create_string_buffer(data if data else b"", length)
    xfer = BulkTransfer(ep=ep, len=length, timeout=timeout_ms,
                        data=ctypes.cast(buf, ctypes.c_void_p))
    n = fcntl.ioctl(fd, USBDEVFS_BULK, xfer)
    return buf.raw[:n]


def do_init(devnode):
    fd = os.open(devnode, os.O_RDWR)
    ifnum = struct.pack("I", VENDOR_INTERFACE)
    fcntl.ioctl(fd, USBDEVFS_CLAIMINTERFACE, ifnum)
    print(f"Claimed interface {VENDOR_INTERFACE}, sending init sequence...")
    for i, cmd in enumerate(INIT_SEQ):
        try:
            usb_bulk(fd, EP_OUT, cmd, len(cmd), 1000)
            usb_bulk(fd, EP_IN, None, 64, 100)   # best-effort drain
        except OSError as e:
            print(f"  cmd {i}: {e}")
        time.sleep(0.05)
    print("Init sequence complete. (Keeping interface claimed for keepalive.)")
    # Intentionally keep fd open so the controller stays initialized.
    return fd


def decode_stick(d):
    x = d[0] | ((d[1] & 0x0F) << 8)
    y = (d[1] >> 4) | (d[2] << 4)
    return x, y


# The Switch player-number LED patterns (players 1..8), as LED bitmasks
# (bit 0 = LED 1). Matches the in-tree hid-nintendo driver.
PLAYER_LED_PATTERNS = [0x01, 0x03, 0x07, 0x0f, 0x09, 0x05, 0x0d, 0x06]


def vib_frame(lf_amp, hf_amp, lf_freq=0x0e1, hf_freq=0x1e1):
    """Build a 5-byte haptic frame (low/high freq amplitude)."""
    v = (lf_freq & 0x1ff)
    v |= (lf_amp & 0x3ff) << 10
    v |= (hf_freq & 0x1ff) << 20
    v |= (hf_amp & 0x3ff) << 30
    return v.to_bytes(5, "little")


# Per-controller HID output report ID (from the HID descriptors).
HID_OUT_REPORT_ID = {0x2069: 0x02, 0x2067: 0x01, 0x2066: 0x01, 0x2073: 0x03}


def motor_packet(pid, lf_amp, hf_amp, pkt_id):
    """[0x50|id] + 3 haptic frames; doubled for the Pro (two motors)."""
    pkt = bytes([0x50 | (pkt_id & 0x0f)]) + vib_frame(lf_amp, hf_amp) * 3
    return pkt * 2 if pid == 0x2069 else pkt


def _drive(send, n=48):
    try:
        for i in range(n):
            send(i)
            time.sleep(0.025)
    except (KeyboardInterrupt, OSError) as e:
        if isinstance(e, OSError):
            print(f"  send failed: {e} (this variant not supported)")
            return
    try:
        for i in range(3):           # stop: zero amplitude
            send(i, stop=True)
            time.sleep(0.01)
    except OSError:
        pass


def rumble_hidraw(pid, lead):
    """Send rumble as a HID output report over interface 0 (via hidraw)."""
    path = find_hidraw(pid)
    if not path:
        print("  no hidraw node; skipping")
        return
    rid = HID_OUT_REPORT_ID.get(pid, 0x01)
    hf = os.open(path, os.O_RDWR)
    try:
        def send(i, stop=False):
            amp = 0 if stop else 700
            body = bytes([rid]) + lead + motor_packet(pid, amp, amp, i)
            os.write(hf, body.ljust(64, b"\x00"))
        _drive(send)
    finally:
        os.close(hf)


def rumble_vendor_cmd(fd, pid):
    """Send rumble command-framed (cmd 0x0a) over the vendor bulk OUT."""
    def send(i, stop=False):
        amp = 0 if stop else 700
        mp = motor_packet(pid, amp, amp, i)
        frame = bytes([0x0a, 0x91, 0x00, 0x02, 0x00, len(mp), 0x00, 0x00]) + mp
        usb_bulk(fd, EP_OUT, frame, len(frame), 1000)
    _drive(send)


def led_test(fd):
    """Cycle the player 1..8 LED patterns (vendor cmd 0x09 sub 0x07)."""
    print("Cycling player 1..8 LED patterns -- watch the controller. "
          "Ctrl-C to stop.\n")
    try:
        for n, mask in enumerate(PLAYER_LED_PATTERNS, 1):
            cmd = bytes([0x09, 0x91, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00,
                         mask, 0, 0, 0, 0, 0, 0, 0])
            usb_bulk(fd, EP_OUT, cmd, len(cmd), 1000)
            print(f"  player {n}: mask 0x{mask:02x}  "
                  f"LEDs[{''.join('*' if mask & (1 << i) else '.' for i in range(4))}]")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped.")


def s16le(d, off):
    """Signed 16-bit little-endian at byte offset off."""
    return int.from_bytes(d[off:off + 2], "little", signed=True)


def monitor_buttons(hf, log, pid):
    """Stream input reports and print which button bits change."""
    print("Press buttons one at a time. Ctrl-C to stop.\n")
    names = BIT_NAMES.get(pid, {})
    prev_btns = 0
    while True:
        data = os.read(hf, 64)
        if not data:
            break
        log.write(data.hex() + "\n")
        if len(data) < OFF_STICK_R + 3:
            continue
        btns = (data[OFF_BUTTONS] | (data[OFF_BUTTONS + 1] << 8) |
                (data[OFF_BUTTONS + 2] << 16))
        if btns != prev_btns:
            newly = btns & ~prev_btns
            bits = [i for i in range(24) if newly & (1 << i)]
            lx, ly = decode_stick(data[OFF_STICK_L:OFF_STICK_L + 3])
            rx, ry = decode_stick(data[OFF_STICK_R:OFF_STICK_R + 3])
            msg = (f"id=0x{data[0]:02x} btns={btns:024b} "
                   f"L=({lx:4d},{ly:4d}) R=({rx:4d},{ry:4d})")
            if bits:
                labelled = ", ".join(f"{i}={names.get(i, '?')}" for i in bits)
                msg += f"  >>> NEW BIT(S): [{labelled}]"
            print(msg)
            log.write(f"# {msg}\n")
            prev_btns = btns


# Byte offset where the trailing vendor/IMU region begins (after buttons,
# sticks, and the GameCube trigger bytes). The IMU data is expected somewhere
# in here.
IMU_SCAN_START = 15


def monitor_imu(hf, log, pid):
    """
    Help locate the IMU (gyro/accel) fields. Hold the controller still, then
    rotate it slowly about each axis in turn. Live view shows the trailing
    region interpreted as signed 16-bit LE values; on exit a per-byte activity
    summary highlights which offsets actually move (those are the IMU).
    """
    print("IMU mode: keep the controller STILL for ~2s, then rotate slowly\n"
          "about each axis (pitch, roll, yaw). Ctrl-C to stop.\n")
    bmin = [255] * 64
    bmax = [0] * 64
    last_print = 0.0
    n = 0
    try:
        while True:
            data = os.read(hf, 64)
            if not data:
                break
            log.write(data.hex() + "\n")
            n += 1
            for i in range(min(len(data), 64)):
                bmin[i] = min(bmin[i], data[i])
                bmax[i] = max(bmax[i], data[i])
            now = time.monotonic()
            if now - last_print >= 0.15:
                last_print = now
                vals = " ".join(
                    f"{off}:{s16le(data, off):>6d}"
                    for off in range(IMU_SCAN_START, len(data) - 1, 2))
                print(f"id=0x{data[0]:02x} s16le[{IMU_SCAN_START}..]: {vals}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        _imu_summary(bmin, bmax, n)


def _imu_summary(bmin, bmax, n):
    print(f"\n--- byte activity over {n} reports (range = max-min) ---")
    print("offset  range   min   max   (* = likely IMU/active)")
    for i in range(64):
        if bmax[i] < bmin[i]:
            continue
        rng = bmax[i] - bmin[i]
        mark = " *" if rng >= 16 else ""
        if rng > 0:
            print(f"  {i:2d}    {rng:4d}   {bmin[i]:3d}   {bmax[i]:3d}{mark}")
    active = [i for i in range(64) if bmax[i] >= bmin[i]
              and bmax[i] - bmin[i] >= 16]
    if active:
        print(f"\nMost active byte offsets (candidate IMU region): {active}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", required=True,
                    help="product id, e.g. 0x2069")
    ap.add_argument("--no-init", action="store_true",
                    help="skip handshake (controller already initialized)")
    ap.add_argument("--imu", action="store_true",
                    help="IMU/gyro discovery mode (rotate the controller)")
    ap.add_argument("--leds", action="store_true",
                    help="cycle the player 1..8 LED patterns and exit")
    ap.add_argument("--rumble", action="store_true",
                    help="test rumble framing variants and exit")
    args = ap.parse_args()
    pid = int(args.pid, 0)

    if os.geteuid() != 0:
        print("Run as root (sudo).", file=sys.stderr)
        return 1

    devnode, _ = find_usb_devnode(pid)
    if not devnode:
        print(f"USB device {VID:04x}:{pid:04x} not found. Plugged in?",
              file=sys.stderr)
        return 1
    print(f"USB device: {devnode}")

    fd = None
    if not args.no_init:
        fd = do_init(devnode)
        time.sleep(0.5)

    if args.leds or args.rumble:
        if fd is None:
            fd = os.open(devnode, os.O_RDWR)
            fcntl.ioctl(fd, USBDEVFS_CLAIMINTERFACE,
                        struct.pack("I", VENDOR_INTERFACE))
        if args.leds:
            led_test(fd)
        if args.rumble:
            variants = [
                ("C: HID output report, id + packet", lambda: rumble_hidraw(pid, b"")),
                ("D: HID output report, id + 0x00 + packet",
                 lambda: rumble_hidraw(pid, b"\x00")),
                ("E: vendor bulk, command-framed (cmd 0x0a)",
                 lambda: rumble_vendor_cmd(fd, pid)),
            ]
            for label, fn in variants:
                print(f"\n=== Variant {label} ===  (feel for ~1.2s)")
                fn()
                time.sleep(1.2)
            print("\nTell me which variant (if any) buzzed.")
        return 0

    hidraw = find_hidraw(pid)
    if not hidraw:
        print("Could not find hidraw node for the controller.",
              file=sys.stderr)
        return 1
    print(f"Reading from {hidraw}")

    mode = "imu" if args.imu else "buttons"
    logname = f"capture-{pid:04x}-{mode}-{int(time.time())}.log"
    log = open(logname, "w")
    print(f"Logging raw reports to {logname}\n")

    hf = os.open(hidraw, os.O_RDONLY)
    try:
        if args.imu:
            monitor_imu(hf, log, pid)
        else:
            try:
                monitor_buttons(hf, log, pid)
            except KeyboardInterrupt:
                print("\nStopped.")
    finally:
        os.close(hf)
        log.close()
        print(f"Saved {logname}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
