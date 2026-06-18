# hid-nintendo2

Linux HID driver for **Nintendo Switch 2** controllers over **USB**, packaged as
an out-of-tree / DKMS module (so it can be used before — or instead of — waiting
for mainline). This is the Switch 2 counterpart to the in-tree `hid-nintendo`
driver (which only covers the original Switch controllers), in the same spirit
as `hid-nx-dkms` did for `hid-nintendo` prior to its mainline merge.

## Supported controllers (USB)

| Controller | USB ID |
| --- | --- |
| Switch 2 Pro Controller | `057e:2069` |
| Joy-Con 2 (L) | `057e:2067` |
| Joy-Con 2 (R) | `057e:2066` |
| NSO GameCube Controller | `057e:2073` |

The Joy-Con 2 Charging Grip (`057e:2068`) is a USB hub, not a controller; the
Joy-Cons attached to it enumerate as the IDs above. Its GL/GR buttons are not
exposed over the Joy-Con HID interface and are therefore not seen by this
driver.

> **USB only.** Over Bluetooth these controllers speak a proprietary BLE
> protocol (not HID-over-GATT), so they never reach the kernel HID layer. That
> path needs BlueZ/userspace work and is out of scope here.

## Status

- ✅ Initialization handshake (sent over the vendor interface), digital buttons,
  analog stick(s).
- ✅ Button maps confirmed for all four controllers (Joy-Con SL/SR rail buttons
  still tentative).
- ✅ GameCube analog triggers (`ABS_Z` / `ABS_RZ`).
- ✅ IMU (gyro + accel) on a separate input device — Pro Controller only so far
  (offsets decoded from capture; per-axis signs/gyro scale may need refinement).
- ⬜ IMU for Joy-Con/GameCube, rumble, player LEDs, battery, Joy-Con mouse.

## Requirements

- Kernel headers for your running kernel (`linux-headers-$(uname -r)` /
  `linux-headers` depending on distro).
- `make`, a C compiler, and for the DKMS route, `dkms`.

## Install (DKMS, recommended)

```sh
sudo cp -r . /usr/src/hid-nintendo2-0.1.0
sudo dkms add -m hid-nintendo2 -v 0.1.0
sudo dkms build -m hid-nintendo2 -v 0.1.0
sudo dkms install -m hid-nintendo2 -v 0.1.0
```

DKMS will rebuild the module automatically on kernel upgrades.

To remove:

```sh
sudo dkms remove -m hid-nintendo2 -v 0.1.0 --all
sudo rm -rf /usr/src/hid-nintendo2-0.1.0
```

## Install (manual, quick test)

```sh
make                       # builds hid-nintendo2.ko against the running kernel
sudo insmod hid-nintendo2.ko
# or, to install into the modules tree:
sudo make install && sudo modprobe hid-nintendo2
```

Unload with `sudo rmmod hid-nintendo2`.

If a controller is already bound to `hid-generic`, replug it (or rebind) after
loading the module so `hid-nintendo2` claims it.

## Verifying

```sh
# Confirm the controller bound to nintendo2 (not hid-generic):
for d in /sys/bus/hid/devices/0003:057E:20*; do
    printf '%s -> %s\n' "$(basename "$d")" \
        "$(basename "$(readlink "$d/driver")")"
done

# Watch input events:
sudo evtest      # pick the controller device
```

## Credits / references

Original `hid-nintendo` authors (Daniel J. Ogorchock, Nadia Holmquist Pedersen,
Emily Strickland, Ryan McClelland). Protocol reverse-engineering references
(used for byte sequences / report layouts only; all code here is original):

- https://github.com/ikz87/NSW2-controller-enabler
- https://github.com/loserkidsblink/nsogcd
- https://github.com/Nadeflore/switch2-controllers
- libsdl-org/SDL `SDL_hidapi_switch2.c`

## License

GPL-2.0-or-later. See the SPDX identifier in `hid-nintendo2.c`.
