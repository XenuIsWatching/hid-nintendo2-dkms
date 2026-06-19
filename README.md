# hid-nintendo2

Linux HID driver for **Nintendo Switch 2** controllers over **USB**, packaged as
an out-of-tree / DKMS module so it can be used now, before (or instead of)
waiting for mainline. It is the Switch 2 counterpart to the in-tree
`hid-nintendo` driver — in the same spirit as `hid-nx-dkms` did for
`hid-nintendo` before its mainline merge.

## Supported controllers (USB)

| Controller | USB ID |
| --- | --- |
| Switch 2 Pro Controller | `057e:2069` |
| Joy-Con 2 (L) | `057e:2067` |
| Joy-Con 2 (R) | `057e:2066` |
| NSO GameCube Controller | `057e:2073` |

## Feature support

| Feature | Pro | Joy-Con 2 L/R | GameCube |
| --- | :---: | :---: | :---: |
| Buttons + D-pad | ✅ | ✅ | ✅ |
| Analog stick(s) | ✅ (2) | ✅ (1) | ✅ (2) |
| Analog triggers | — | — | ✅ |
| IMU (gyro + accel) | ✅ | ✅ | ✅ |
| Player LEDs | ✅ | ✅ | ✅ |
| Rumble (`FF_RUMBLE`) | ✅ | ✅ | ✅ (on/off) |

Buttons and sticks are exposed on a standard input device; the IMU is a separate
input device (accel on `ABS_X/Y/Z`, gyro on `ABS_RX/RY/RZ`,
`INPUT_PROP_ACCELEROMETER`, `MSC_TIMESTAMP`). Player LEDs appear under
`/sys/class/leds` and a unique player number is assigned per controller.

## Not supported

- **Bluetooth** — over Bluetooth these controllers use a proprietary BLE
  protocol (not HID-over-GATT), so they never reach the kernel HID layer. That
  needs BlueZ/userspace work and is out of scope for this driver.
- **Joy-Con mouse and SL/SR buttons** — only usable with a detached Joy-Con,
  i.e. over Bluetooth.
- **Charging-grip GL/GR** — the grip (`057e:2068`) is a plain USB hub; its GL/GR
  buttons do not appear anywhere in the Joy-Con's USB reports.
- **Battery** — not present in the USB input reports (and a USB-connected
  controller is always charging).

The remaining items would need the Bluetooth transport. See
[`docs/PROTOCOL.md`](docs/PROTOCOL.md) for the reverse-engineered protocol and
the open questions.

## Requirements

- Kernel headers for your running kernel (`linux-headers-$(uname -r)` or your
  distro's equivalent).
- `make`, a C compiler matching your kernel's toolchain, and `dkms` for the DKMS
  install.

## Install (DKMS, recommended)

```sh
sudo cp -r . /usr/src/hid-nintendo2-0.1.0
sudo dkms add     -m hid-nintendo2 -v 0.1.0
sudo dkms build   -m hid-nintendo2 -v 0.1.0
sudo dkms install -m hid-nintendo2 -v 0.1.0
```

DKMS rebuilds the module automatically on kernel upgrades. To remove:

```sh
sudo dkms remove -m hid-nintendo2 -v 0.1.0 --all
sudo rm -rf /usr/src/hid-nintendo2-0.1.0
```

## Install (manual, quick test)

```sh
make                 # builds hid-nintendo2.ko against the running kernel
sudo insmod hid-nintendo2.ko
# or install into the modules tree and load by name:
sudo make install && sudo modprobe hid-nintendo2
```

Unload with `sudo rmmod hid-nintendo2`. If a controller was already bound to
`hid-generic`, replug it after loading so `hid-nintendo2` claims it.

## Verifying

```sh
# Confirm the controllers bound to nintendo2 (not hid-generic):
for d in /sys/bus/hid/devices/0003:057E:20*; do
    printf '%s -> %s\n' "$(basename "$d")" \
        "$(basename "$(readlink "$d/driver")")"
done

sudo evtest       # watch buttons/sticks/IMU on the chosen device
sudo fftest        # test rumble
```

## Tools

[`tools/`](tools/) contains `sw2_capture.py`, a zero-dependency capture/decoder
used to reverse-engineer the protocol. It can decode button bits, locate the IMU
fields, cycle the LEDs, and test rumble. See [`tools/README.md`](tools/README.md).

## Credits / references

Protocol reverse-engineering references (used for byte sequences / report
layouts only; the code here is original):

- https://github.com/ikz87/NSW2-controller-enabler
- https://github.com/loserkidsblink/nsogcd
- https://github.com/Nadeflore/switch2-controllers
- https://github.com/darthcloud/BlueRetro
- libsdl-org/SDL `SDL_hidapi_switch2.c`

## License

GPL-2.0-or-later. See the SPDX identifier in `hid-nintendo2.c`.
