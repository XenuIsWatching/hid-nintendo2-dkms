# Switch 2 controller capture tooling

Purpose: reverse-engineer the **button bit → name mapping** and report layout
for the in-kernel `hid-nintendo2` driver. Zero external dependencies (pure
Python stdlib + usbdevfs ioctls + hidraw). Everything needs **root**.

The controllers do not emit usable HID reports until a proprietary
initialization handshake is sent to their vendor USB interface (interface 1).
These scripts send that handshake, then decode the resulting reports.

## Quick start (do this tomorrow)

1. Plug in the controller over USB-C.
2. Run the decoder, which inits the controller and then prints button bits:

   ```bash
   cd ~/switch2-refs/sw2-capture
   sudo ./sw2_capture.py --pid 0x2069     # Pro Controller
   # other PIDs: 0x2073 GameCube, 0x2067 Joy-Con2 L, 0x2066 Joy-Con2 R
   ```

3. Press **one button at a time**. Each press prints a line like:

   ```
   id=0x09 btns=000000000000000000000010 L=(2048,2050) R=(2049,2047)  >>> NEW BIT(S): [1]
   ```

   Write down which physical button sets which bit. Go through every button
   (A/B/X/Y, dpad, L/R/ZL/ZR, +/-, Home, Capture, stick-clicks, and SL/SR on
   Joy-Cons). The full raw stream is also saved to `capture-<pid>-<ts>.log`.

4. Hand the bit list back to update the button maps in `hid-nintendo2.c`
   (`sw2_procon_btns`, `sw2_joyconl_btns`, etc.).

## IMU discovery mode

Finds the gyro/accelerometer fields in the report:

```bash
sudo ./sw2_capture.py --pid 0x2069 --imu
```

Hold the controller **still** for a couple of seconds, then **rotate it slowly
about each axis in turn** (pitch, then roll, then yaw). The live view prints the
trailing report region interpreted as signed 16-bit little-endian values; on
exit it prints a per-byte activity summary (range = max-min) and lists the most
active byte offsets — those are the IMU. (An uncontrolled capture of the Pro
Controller already points at roughly bytes 21..44.)

## LED test mode

Verify the player-LED command (vendor `cmd=0x09 sub=0x07`) without loading the
kernel module — it cycles the player 1..8 patterns on the controller:

```bash
sudo ./sw2_capture.py --pid 0x2069 --leds
```

The same patterns are used by the kernel driver. Once the driver is loaded, the
LEDs also appear under `/sys/class/leds/*player-*` and can be toggled with
`echo 1 | sudo tee /sys/class/leds/<name>/brightness`.

## Rumble test mode

The haptic payload format is known (5-byte frames, packet = `0x50|id` + 3
frames); only how it is framed over the USB vendor endpoint is unconfirmed.
Raw vibration packets on the vendor bulk OUT do nothing (that channel is for
commands; over BLE vibration uses a separate characteristic). This mode tries
the more likely USB paths so you can confirm by feel:

```bash
sudo ./sw2_capture.py --pid 0x2069 --rumble
```

It runs, with pauses between each:
- **Variant C** — HID output report (`id + packet`) over interface 0 via hidraw
- **Variant D** — HID output report (`id + 0x00 + packet`)
- **Variant E** — command-framed (`cmd 0x0a`) over the vendor bulk OUT

Report which variant (if any) buzzes.

## Full USB capture (optional, for IMU/rumble/init RE)

Captures all USB traffic on the controller's bus via usbmon while initializing:

```bash
sudo ./run_usbmon.sh 0x2069 30      # pid, seconds
```

Outputs `usbmon-busN-<pid>-<ts>.txt` (raw URBs) plus the decoder log.

## Notes / gotchas

- Interface 1 (vendor 0xff) is unbound by default, so the scripts can claim it
  without detaching any kernel driver.
- The Pro Controller also exposes USB audio interfaces (headphone jack) bound
  to `snd-usb-audio`; the scripts deliberately do **not** call
  `set_configuration`, so audio is left undisturbed.
- Init is **not persistent** — if reports stop, re-run the script. It keeps the
  vendor interface claimed while running as a crude keepalive.
- Confirmed so far: GameCube (0x2073) main report id is **0x0A**; Pro (0x2069)
  is **0x09**; Joy-Con 2 L/R are **0x07 / 0x08**. Stick data is two 12-bit
  little-endian values per 3 bytes, center 2048. GameCube analog triggers are
  in the vendor bytes at report offsets 13/14.
