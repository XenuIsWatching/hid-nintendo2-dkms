# Nintendo Switch 2 controller protocol notes (USB)

Reverse-engineering notes for the `hid-nintendo2` driver. Scope is the **USB**
transport for the Switch 2 Pro Controller (`0x2069`), Joy-Con 2 L (`0x2067`) /
R (`0x2066`), and NSO GameCube controller (`0x2073`). Bluetooth uses a separate
proprietary BLE protocol and is out of scope here.

Most of this was derived from our own `usbmon`/hidraw captures (see
`tools/sw2_capture.py`), cross-checked against the community references credited
in the driver header (NSW2-controller-enabler, nsogcd, Nadeflore/switch2-
controllers, SDL `SDL_hidapi_switch2.c`).

> Note: the **USB** report layout differs from the **BLE** layout that the
> Nadeflore/switch2-controllers project documents (e.g. over BLE the IMU axes
> are stored in separate accel/gyro blocks; over USB they are interleaved). Do
> not assume BLE offsets apply to USB.

## 1. USB topology

Each controller exposes two relevant interfaces:

| Interface | Class | Endpoints | Use |
|---|---|---|---|
| 0 | HID (3) | int IN `0x81`, int OUT `0x01` | input reports (after init) |
| 1 | Vendor (0xff) | bulk OUT `0x02`, bulk IN `0x82` | init handshake + commands |

The Pro Controller additionally exposes USB-audio interfaces (headphone jack);
ignore them. The driver binds interface 0 and reaches interface 1 (which is
otherwise unbound) for the handshake and for output commands (LEDs).

## 2. Initialization handshake

The controller does **not** emit usable HID input reports on interface 0 until a
proprietary sequence is sent to interface 1's bulk OUT. The sequence is not
persistent and must be re-sent on every connect. See `sw2_init_seq` in the
driver for the exact byte sequences (originally from NSW2-controller-enabler).

### Command frame

Commands on the vendor channel use a 16-byte frame:

```
off size field
 0   1   cmd
 1   1   type (0x91 request)
 2   1   interface (0x00 USB)
 3   1   subcmd
 4   4   header (length/flags)
 8   N   payload
```

### Feature command `cmd=0x0c`

Optional sensors are gated behind a feature bitmask, enabled with two commands
(`sub=0x02` init, then `sub=0x04` enable), payload = flag byte:

| Flag | Feature |
|---|---|
| `0x04` | motion (IMU) |
| `0x10` | mouse (Joy-Con optical sensor) |
| `0x80` | magnetometer |

Our init enables flag `0x27` (`0x20|0x04|0x02|0x01`) — i.e. **motion on**, mouse
and magnetometer **off**. That is why the IMU is populated but the mouse and
magnetometer slots in the report are inactive.

### LED command `cmd=0x09 sub=0x07`

Sets the four player-indicator LEDs from a bitmask (bit 0 = LED 1) at payload
byte 0:

```
09 91 00 07 00 08 00 00 <mask> 00 00 00 00 00 00 00
```

The driver assigns a unique player number (IDA) and maps it to the standard
Switch player patterns: P1..P8 = `0x01 0x03 0x07 0x0f 0x09 0x05 0x0d 0x06`.

### SPI flash read `cmd=0x02 sub=0x04`

Per-unit calibration lives in the controller's SPI flash and is read over the
vendor channel during init. The frame carries a length and a little-endian
address; the reply echoes a 16-byte header followed by the data payload:

```
02 91 00 04 00 08 00 00 <len> 7e 00 00 <addr LE32>
```

Reads of up to ~0x30 bytes per request work. The device-info block at `0x13000`
holds the ASCII serial, USB VID/PID and colour codes. See "Calibration" below for
the stick/trigger calibration this is used for.

### Rumble (HID output report, interface 0)

Rumble does **not** go over the vendor channel — it is a **HID output report**
on interface 0 (confirmed by feel for all four controllers). `out_report_id` is
from the descriptor: Pro `0x02`, Joy-Con `0x01`, GameCube `0x03`.

**Pro / Joy-Con (HD rumble):**

```
<id> <motor packet> [<motor packet> ...]
```
- motor packet = `0x50|rolling_id` + three 5-byte haptic frames. The Pro takes
  **two** motor packets (left, right); Joy-Cons take **one**.
- 5-byte frame (40 bits LE): `lf_freq(9) | lf_amp(10)<<10 | hf_freq(9)<<20 |
  hf_amp(10)<<30`. Defaults `lf_freq=0x0e1`, `hf_freq=0x1e1`; amplitude
  `0..~800` (`≈ 800 * magnitude / 0xffff`).

**GameCube (simple on/off, combined with the LED command — BlueRetro format):**

```
03 <0x50|id> <01=on|00=off> 00 00  09 91 00 07 00 08 00 00 <led_mask> ...
```
The rumble is a single on/off byte at offset 2; bytes 5.. carry an embedded
SET_LED command, so the driver folds in the current player-LED mask.

The driver exposes all of this as a memless `FF_RUMBLE` device (strong→lf_amp,
weak→hf_amp; GameCube: any nonzero → on) and resends every ~30 ms while active.

## 3. Input report layout (interface 0)

`data[0]` is the report ID (Pro `0x09`, GameCube `0x0a`, Joy-Con L `0x07`,
Joy-Con R `0x08`). All multi-byte fields are little-endian. Offsets below are for
the Pro Controller; the **right Joy-Con shifts the sensor section (offset 15
onward) by +1 byte** due to an extra reserved byte in the pre-sensor padding.

> A controller that is only *partially* initialized (e.g. sent the wake command
> but not the full init sequence) streams a different, basic report — observed on
> the GameCube as report `0x05`, with a different layout (byte 3 constant `0x04`,
> sticks around bytes 11–16). It is a compatibility/fallback report, not the
> native one; the full init promotes the controller to the native ID above. The
> driver matches only the native per-type ID, so a fallback-mode controller
> simply produces no input events rather than misparsed ones.

```
off  size  field
 0    1    report ID
 1    1    packet counter (+1 per report)
 2    1    constant 0x20
 3    3    buttons (bit-packed; see maps below)
 6    3    left stick   (two 12-bit axes)
 9    3    right stick  (two 12-bit axes; C-stick on GameCube)
 13   1    left analog trigger  (GameCube only, ~0x24..0xf0)
 14   1    right analog trigger (GameCube only)
 15   1    sensor-section marker 0x1e
 16   2    hardware timestamp (increments ~+3/report; unit unconfirmed)
 18   2    constant marker 0x0c00
 ~20  ..   frame counter + feature-gated mouse/magnetometer slots
           (inactive unless enabled via the feature command above)
 32   12   IMU sample (see below)
```

### Buttons

3 bytes = up to 24 bits. Bit numbering is from the LSB of `data[3]`.

**Pro Controller (confirmed):**

| bit | btn | bit | btn | bit | btn |
|---|---|---|---|---|---|
| 0 | B | 8 | Dpad-Down | 16 | Home |
| 1 | A | 9 | Dpad-Right | 17 | Capture |
| 2 | Y | 10 | Dpad-Left | 18 | SR / right rear |
| 3 | X | 11 | Dpad-Up | 19 | SL / left rear |
| 4 | R | 12 | L | 20 | C (GameChat) |
| 5 | ZR | 13 | ZL | | |
| 6 | + | 14 | − | | |
| 7 | R-stick | 15 | L-stick | | |

**GameCube (confirmed):** bits 0–4,6,8–13 as Pro, plus bit 5 = **Z**, 16 = Home,
17 = Capture, 20 = C. No stick clicks, no Minus, no SR/SL. Z uses `BTN_Z`,
Capture uses `BTN_SELECT` (no Minus to collide with).

**Joy-Con 2 L (confirmed bits 0–8):** 0 Down, 1 Right, 2 Left, 3 Up, 4 L, 5 ZL,
6 −, 7 stick-click, 8 Capture. SL/SR (bits 9/10) tentative.

**Joy-Con 2 R (confirmed bits 0–8,12):** 0 B, 1 A, 2 Y, 3 X, 4 R, 5 ZR, 6 +,
7 stick-click, 8 Home, 12 C. SL/SR (bits 9/10) tentative.

> Bit 16 on the Joy-Cons is a constant status flag, not a button. The grip's
> GL/GR buttons are not reported over the Joy-Con HID interface.

### Sticks

Two 12-bit values packed into 3 bytes:
`x = b0 | ((b1 & 0x0f) << 8)`, `y = (b1 >> 4) | (b2 << 4)`. Raw center ≈ 2048.
The driver applies per-unit calibration (below) and reports signed axes
(−32768..32767).

### Calibration (SPI flash)

Read with the SPI command above during init.

**Sticks** — 9-byte blob: three packed (x,y) pairs in the same 12-bit format as
the stick reports, giving per-axis `neutral` / `positive` span / `negative` span:

| Address | Use |
|---|---|
| `0x130a8` | factory primary calibration (left / main stick) |
| `0x130e8` | factory secondary calibration (right / C stick) |
| `0x1fc040` | user primary (11 bytes: `a1 b2` magic + 9-byte blob) |
| `0x1fc080` | user secondary |

User calibration overrides factory when the `0xa1b2` magic is present (axes are
all `0xff` when unset). Apply: `out = (raw − neutral) * 32768 / span`, choosing
the positive or negative span by sign; the Y axis is negated. Single-stick
controllers (Joy-Cons) use the primary slot only.

**GameCube triggers** — 2 bytes at `0x13140` = `[right_zero, left_zero]`, the
per-unit released rest points (`0xff` = unset). Note the order is right-then-left
(verified on hardware: the left trigger, report byte 13, rests at the *second*
byte). No full-pull value is stored anywhere in the calibration region — only the
rest points — and the full-pull (~0xee–0xf0) varies by unit, so the driver tracks
each trigger's observed maximum at runtime (grow-only, seeded below the typical
full-pull) and scales `out = 4096 * (raw − zero) / (max − zero)` to 0..4095.

### IMU (accelerometer; gyro NOT decoded)

One sample per report. The accelerometer is three s16 LE values at the IMU
offset + 2/6/10 (X/Y/Z):

| Controller | report | IMU offset (accel at +2/+6/+10) |
|---|---|---|
| Pro / GameCube / Joy-Con L | 0x09 / 0x0a / 0x07 | 32 |
| Joy-Con R | 0x08 | 33 |

Accelerometer is **confirmed**: rests at gravity (|a| ≈ 4096 LSB/g, ~4096 on the
down axis, ~0 on the others). The driver reports raw s16 with resolution 4096
LSB/g and `INPUT_PROP_ACCELEROMETER`.

**Gyro is not decoded.** The three s16 slots interleaved with the accel (IMU
offset + 0/4/8) look like a gyro by position but are noise over USB at our
config: held perfectly still they swing ±18000 and flip sign every sample
(lag-1 autocorrelation ≈ 0, versus ≈ 1.0 for the real accel), and they don't
correlate with the accel-derived rotation rate. The only other smooth fields in
the report (s16 at 22/26/30) are non-zero at rest, respond to *all* rotations,
and have non-constant magnitude — a fused/derived field, not a clean gyro or
magnetometer.

For reference, the BLE projects (`switch2-controllers`) store accel then gyro as
**three consecutive** s16 each (BLE offsets 48–53, 54–59) after enabling
`FEATURE_MOTION (0x04)` — the same flag we set. But our USB layout differs
(accel is stride-4, and BLE offsets 48–59 are zero in our USB report), so those
offsets don't map. Recovering a USB gyro likely needs further RE (e.g. SPI
calibration load, a different or additional enable, or a different decode) plus a
clean slow non-saturating capture. Until then the driver ships accel-only.

## 4. Battery

The **USB input report carries no battery field** (scanning every stable field,
only the `0x0c00` marker falls in the voltage range, and it is constant). The
BLE report does carry battery voltage/current/temperature, but that is a
different format. Since a USB-connected controller is always charging, battery
is not implemented over USB.

## 5. Known unknowns / TODO

- Mouse (Joy-Con): enable feature `0x10`, then decode the mouse coords/roughness/
  distance fields (BLE places them right after the sticks) → `REL_X`/`REL_Y`.
- Magnetometer: enable feature `0x80`; no standard Linux joystick axis for it.
- The pre-IMU bytes ~20..31 (frame counter + inactive feature slots) are only
  partially identified.
- Joy-Con SL/SR button bits (9/10) are unverified guesses.
- Charging-grip GL/GR buttons: **appear absent from the USB data path.** The
  grip (`0x2068`) is a pure USB hub; with a Joy-Con attached, pressing GL/GR
  produced no change anywhere in the Joy-Con's report (button field constant, no
  alternate report ID, only motion noise on the IMU bytes). BlueRetro has no
  GL/GR handling either. Likely Bluetooth-only or requires a feature flag we
  don't set (init enables `0x27`; untried bits `0x08`/`0x40`).
