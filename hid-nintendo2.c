// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for Nintendo Switch 2 controllers (USB)
 *
 * Copyright (c) 2026 Ryan McClelland <rymcclel@gmail.com>
 *
 * This driver supports the Nintendo Switch 2 family of controllers when
 * connected over USB:
 *   - Switch 2 Pro Controller	(057e:2069)
 *   - Joy-Con 2 (L)		(057e:2067)
 *   - Joy-Con 2 (R)		(057e:2066)
 *   - NSO GameCube Controller	(057e:2073)
 *
 * Unlike the original Switch controllers (see hid-nintendo.c), the Switch 2
 * controllers use a completely different, proprietary protocol. They expose
 * two relevant USB interfaces:
 *   - interface 0: a HID interface (this driver binds here) which carries the
 *     standard input reports, but only *after* the controller has been
 *     initialized;
 *   - interface 1: a vendor-specific (class 0xff) interface with bulk
 *     endpoints, over which a proprietary initialization handshake must be
 *     sent to switch the controller out of its default mode and make it emit
 *     usable HID input reports.
 *
 * Bluetooth is intentionally not supported: over Bluetooth these controllers
 * speak a proprietary BLE protocol (not HID-over-GATT) and therefore never
 * reach the HID layer.
 *
 * The following resources were referenced while writing this driver (used for
 * protocol/byte-sequence reference only; all code here is original):
 *   https://github.com/ikz87/NSW2-controller-enabler
 *   https://github.com/loserkidsblink/nsogcd
 *   https://github.com/Nadeflore/switch2-controllers
 *   https://github.com/darthcloud/BlueRetro
 *   libsdl-org/SDL SDL_hidapi_switch2.c
 */

#include <linux/hid.h>
#include <linux/idr.h>
#include <linux/input.h>
#include <linux/ktime.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

/*
 * Device IDs. Mirrors the relevant entries from the in-tree
 * drivers/hid/hid-ids.h, defined locally so this builds as an out-of-tree
 * module.
 */
#define USB_VENDOR_ID_NINTENDO			0x057e
#define USB_DEVICE_ID_NINTENDO_SW2_JOYCONR	0x2066
#define USB_DEVICE_ID_NINTENDO_SW2_JOYCONL	0x2067
#define USB_DEVICE_ID_NINTENDO_SW2_PROCON	0x2069
#define USB_DEVICE_ID_NINTENDO_SW2_NSOGC	0x2073

/* The proprietary init/handshake commands are sent to this USB interface. */
#define SW2_VENDOR_INTERFACE	1

/* Steady-state input report IDs (interface 0), per controller type. */
#define SW2_INPUT_REPORT_PROCON		0x09
#define SW2_INPUT_REPORT_JOYCONL	0x07
#define SW2_INPUT_REPORT_JOYCONR	0x08
#define SW2_INPUT_REPORT_NSOGC		0x0a

/*
 * Input report layout (USB, interface 0), as reverse-engineered. Offsets are
 * for the Pro Controller (report 0x09); the right Joy-Con (0x08) shifts the
 * sensor section onward by one byte (see SW2_OFF_IMU_JOYCONR). All multi-byte
 * fields are little-endian. See docs/PROTOCOL.md for the full write-up.
 *
 *   off  size  field
 *   0    1     report ID
 *   1    1     packet counter (increments per report)
 *   2    1     constant 0x20
 *   3    3     buttons (bit-packed, see the per-type maps below)
 *   6    3     left stick   (two 12-bit axes)
 *   9    3     right stick  (two 12-bit axes; C-stick on GameCube)
 *   13   1     left analog trigger  (GameCube only)
 *   14   1     right analog trigger (GameCube only)
 *   15   1     sensor-section marker 0x1e
 *   16   2     hardware timestamp (unit unconfirmed)
 *   18   2     constant marker 0x0c00
 *   ~20  ..    frame counter + feature-gated mouse/magnetometer slots,
 *              inactive unless enabled via the feature command (we enable
 *              motion only; see the init sequence's 0x0c commands)
 *   32   12    IMU sample: gyro/accel interleaved per axis (see SW2_OFF_IMU)
 */

/* Offsets within an input report (data[0] is the report ID). */
#define SW2_OFF_BUTTONS		3	/* 3 bytes of bit-packed buttons */
#define SW2_OFF_STICK_L		6	/* 3 bytes -> two 12-bit axes */
#define SW2_OFF_STICK_R		9	/* 3 bytes -> two 12-bit axes */
#define SW2_OFF_TRIGGER_L	13	/* analog left trigger (GameCube) */
#define SW2_OFF_TRIGGER_R	14	/* analog right trigger (GameCube) */

/*
 * IMU block: one sample per report. The accelerometer is three s16 LE values at
 * imu_offset + 2/6/10 (X/Y/Z), interleaved with three s16 slots at
 * imu_offset + 0/4/8. Those interleaved slots LOOK like a gyro by position but
 * carry only noise over USB at our config (uncorrelated sample-to-sample and
 * non-zero at rest), so no usable gyro is decoded - see docs/PROTOCOL.md. The
 * Pro (0x09), GameCube (0x0a) and left Joy-Con (0x07) place the block at offset
 * 32; the right Joy-Con (0x08) has one extra leading byte (offset 33).
 */
#define SW2_OFF_IMU		32
#define SW2_OFF_IMU_JOYCONR	33
#define SW2_IMU_LEN		12

/* Accelerometer resolution: ~4096 LSB per g (rest |a| measured ~4096). */
#define SW2_IMU_ACCEL_RES_PER_G	4096

/* Command-frame fields shared by the LED command and the GameCube rumble report. */
#define SW2_CMD_SET_LED		0x09
#define SW2_REQ_TYPE_REQ	0x91
#define SW2_RSP_TYPE_RSP	0x01
#define SW2_IFACE_USB		0x00
#define SW2_SUBCMD_SET_LED	0x07

/*
 * SPI flash read (vendor command 0x02, subcommand 0x04). Per-unit calibration
 * for the analog sticks and (GameCube) the analog triggers lives in flash and
 * is read during probe. Frame: 02 91 00 04 00 08 00 00 <len> 7e 00 00 <addr LE>.
 */
#define SW2_CMD_READ_SPI	0x02
#define SW2_SUBCMD_READ_SPI	0x04
#define SW2_FLASH_READ_MAX	0x30	/* bytes per read request */

/* Flash addresses (see docs/PROTOCOL.md). */
#define SW2_FLASH_FACTORY_STICK1	0x130a8	/* left / main stick */
#define SW2_FLASH_FACTORY_STICK2	0x130e8	/* right / C stick */
#define SW2_FLASH_FACTORY_TRIGGER	0x13140	/* GameCube L/R zero points */
#define SW2_FLASH_USER_STICK1		0x1fc040
#define SW2_FLASH_USER_STICK2		0x1fc080
#define SW2_STICK_CALIB_LEN		9	/* packed neutral/pos/neg per axis */
#define SW2_USER_CALIB_LEN		11	/* 2-byte magic + 9-byte calib */
#define SW2_TRIGGER_CALIB_LEN		2	/* [left_zero, right_zero] */
#define SW2_USER_CALIB_MAGIC		0xa1b2

/* Player LEDs: vendor command 0x09, subcommand 0x07, bitmask at payload byte. */
#define SW2_NUM_PLAYER_LEDS	4
#define SW2_NUM_LED_PATTERNS	8

/*
 * Rumble is delivered as a HID output report on interface 0 (NOT the vendor
 * channel). Payload after the report ID is one or more motor packets, each
 * [0x50 | rolling_id] followed by three 5-byte haptic frames. The Pro sends two
 * motor packets (left, right); Joy-Cons send one. Each 5-byte frame packs
 * lf_freq(9) | lf_amp(10) | hf_freq(9) | hf_amp(10) little-endian. Confirmed
 * for the Pro Controller and both Joy-Con 2; the GameCube path is unconfirmed.
 */
#define SW2_VIB_LF_FREQ		0x0e1
#define SW2_VIB_HF_FREQ		0x1e1
#define SW2_VIB_AMP_MAX		800
#define SW2_VIB_FRAME_LEN	5
#define SW2_RUMBLE_PERIOD_MS	30	/* resend interval while active */

/*
 * Sticks are reported as calibrated signed values (evdev convention). The raw
 * report carries 12-bit axes; per-unit factory calibration (neutral/positive/
 * negative span) from flash maps them onto this range. SW2_STICK_CENTER is the
 * fallback midpoint used only when calibration is absent.
 */
#define SW2_STICK_CENTER	2048
#define SW2_STICK_AXIS_MIN	(-32768)
#define SW2_STICK_AXIS_MAX	32767

/*
 * GameCube analog triggers (report bytes 13/14, L/R), scaled to
 * 0..SW2_TRIGGER_RANGE. The per-unit released "zero" point comes from flash
 * (SW2_FLASH_FACTORY_TRIGGER); the fully-pulled value is NOT stored and varies
 * by unit, so the full-scale point is auto-calibrated at runtime (the highest
 * raw value seen, grow-only) starting from SW2_TRIGGER_RAW_FULL as an initial
 * estimate. The seed is deliberately below the observed full-pull (~0xe0) so
 * the grow-only tracker can adapt up to each trigger's true maximum; a seed at
 * or above a unit's real full-pull would cap that trigger below 100%. Until the
 * first full pull a trigger may reach 100% slightly early; it self-corrects once
 * pulled fully. SW2_TRIGGER_RAW_MIN is the fallback zero if flash lacks trigger
 * calibration.
 */
#define SW2_TRIGGER_RANGE	4095
#define SW2_TRIGGER_RAW_FULL	0xc0	/* initial full-scale seed (auto-grows) */
#define SW2_TRIGGER_RAW_MIN	0x24

enum sw2_ctlr_type {
	SW2_CTLR_PROCON,
	SW2_CTLR_JOYCONL,
	SW2_CTLR_JOYCONR,
	SW2_CTLR_NSOGC,
};

/*
 * Per-axis stick calibration: the resting "neutral" reading plus the spans from
 * neutral to the positive and negative extremes. All zero means uncalibrated.
 */
struct sw2_axis_calib {
	u16 neutral;
	u16 positive;
	u16 negative;
};

struct sw2_stick_calib {
	struct sw2_axis_calib x;
	struct sw2_axis_calib y;
};

struct sw2_ctlr {
	struct hid_device *hdev;
	struct input_dev *input;
	struct input_dev *imu_input;
	struct usb_device *udev;
	enum sw2_ctlr_type type;
	const struct sw2_ctlr_info *info;	/* per-type parameters */
	unsigned int ep_out;	/* bulk OUT pipe on the vendor interface */
	unsigned int ep_in;	/* bulk IN pipe on the vendor interface */
	struct mutex out_mutex;	/* serializes vendor OUT transfers */
	struct sw2_stick_calib stick_calib[2];	/* [0]=left/main, [1]=right/C */
	u8 trigger_zero[2];	/* GameCube L/R rest points (0 = uncalibrated) */
	u8 trigger_max[2];	/* GameCube L/R full-scale, auto-calibrated */
	struct led_classdev player_leds[SW2_NUM_PLAYER_LEDS];
	u8 led_mask;		/* current player LED bitmask */
	int player_id;		/* assigned player number, -1 if none */
	ktime_t imu_last;	/* time of previous IMU sample */
	u32 imu_timestamp_us;	/* accumulated IMU timestamp (MSC_TIMESTAMP) */
	struct delayed_work rumble_work;
	spinlock_t rumble_lock;	/* protects the fields below */
	u16 rumble_lf;		/* low-freq amplitude */
	u16 rumble_hf;		/* high-freq amplitude */
	bool rumble_active;
	u8 rumble_pkt_id;	/* rolling 4-bit packet counter */
};

/*
 * Player-number LED patterns (players 1..8), as LED bitmasks (bit 0 = LED 1).
 * Matches the in-tree hid-nintendo driver's patterns.
 */
static const u8 sw2_player_led_patterns[SW2_NUM_LED_PATTERNS] = {
	0x01, 0x03, 0x07, 0x0f, 0x09, 0x05, 0x0d, 0x06,
};

static DEFINE_IDA(sw2_player_ida);

/*
 * One entry maps a button bit (bit_nr counts from the LSB of the first button
 * byte, i.e. data[SW2_OFF_BUTTONS]) to a Linux input event code.
 */
struct sw2_button_map {
	u8 bit_nr;
	u16 code;
};

/*
 * Pro Controller button map. CONFIRMED via a usbmon/hidraw capture of an
 * initialized controller (report 0x09, button field in data[3..5]).
 */
static const struct sw2_button_map sw2_procon_btns[] = {
	{ 0,  BTN_SOUTH },		/* B */
	{ 1,  BTN_EAST },		/* A */
	{ 2,  BTN_WEST },		/* Y */
	{ 3,  BTN_NORTH },		/* X */
	{ 4,  BTN_TR },			/* R */
	{ 5,  BTN_TR2 },		/* ZR */
	{ 6,  BTN_START },		/* + */
	{ 7,  BTN_THUMBR },		/* right stick click */
	{ 8,  BTN_DPAD_DOWN },
	{ 9,  BTN_DPAD_RIGHT },
	{ 10, BTN_DPAD_LEFT },
	{ 11, BTN_DPAD_UP },
	{ 12, BTN_TL },			/* L */
	{ 13, BTN_TL2 },		/* ZL */
	{ 14, BTN_SELECT },		/* - */
	{ 15, BTN_THUMBL },		/* left stick click */
	{ 16, BTN_MODE },		/* Home */
	{ 17, BTN_Z },			/* Capture */
	{ 18, BTN_TRIGGER_HAPPY1 },	/* SR / right rear */
	{ 19, BTN_TRIGGER_HAPPY2 },	/* SL / left rear */
	{ 20, BTN_C },			/* C (GameChat) */
	{ }
};

/*
 * GameCube controller button map. CONFIRMED via capture (report 0x0a). The
 * GameCube has fewer buttons than the Pro: no stick clicks (bits 7/15), no
 * Minus (bit 14) and no SR/SL (bits 18/19). Its classic "Z" button reports in
 * the bit-5 slot (the Pro's ZR), and it adds a small "ZL" (bit 13). Capture
 * uses BTN_SELECT since there is no Minus button to conflict with.
 */
static const struct sw2_button_map sw2_nsogc_btns[] = {
	{ 0,  BTN_SOUTH },	/* B */
	{ 1,  BTN_EAST },	/* A */
	{ 2,  BTN_WEST },	/* Y */
	{ 3,  BTN_NORTH },	/* X */
	{ 4,  BTN_TR },		/* R */
	{ 5,  BTN_Z },		/* Z (classic) */
	{ 6,  BTN_START },	/* Start */
	{ 8,  BTN_DPAD_DOWN },
	{ 9,  BTN_DPAD_RIGHT },
	{ 10, BTN_DPAD_LEFT },
	{ 11, BTN_DPAD_UP },
	{ 12, BTN_TL },		/* L */
	{ 13, BTN_TL2 },	/* ZL (small) */
	{ 16, BTN_MODE },	/* Home */
	{ 17, BTN_SELECT },	/* Capture */
	{ 20, BTN_C },		/* C (GameChat) */
	{ }
};

/*
 * Joy-Con 2 button maps (single side, 16-bit button field in data[3..4]; the
 * data[5] byte holds status flags, e.g. bit 16 is set constantly, so bits >=16
 * are deliberately not mapped). The grip's GL/GR buttons are not reported here.
 *
 * Left Joy-Con: bits 0..8 CONFIRMED via capture. SL/SR (bits 9/10) are still
 * tentative, pending a capture that exercises the side-rail buttons.
 */
static const struct sw2_button_map sw2_joyconl_btns[] = {
	{ 0,  BTN_DPAD_DOWN },
	{ 1,  BTN_DPAD_RIGHT },
	{ 2,  BTN_DPAD_LEFT },
	{ 3,  BTN_DPAD_UP },
	{ 4,  BTN_TL },			/* L */
	{ 5,  BTN_TL2 },		/* ZL */
	{ 6,  BTN_SELECT },		/* - */
	{ 7,  BTN_THUMBL },		/* stick click */
	{ 8,  BTN_Z },			/* Capture */
	{ 9,  BTN_TRIGGER_HAPPY1 },	/* SL (tentative) */
	{ 10, BTN_TRIGGER_HAPPY2 },	/* SR (tentative) */
	{ }
};

/*
 * Right Joy-Con: bits 0..8 and 12 CONFIRMED via capture. SL/SR (bits 9/10)
 * remain tentative. Note the C (GameChat) button is at bit 12 here, unlike the
 * Pro Controller where it is bit 20.
 */
static const struct sw2_button_map sw2_joyconr_btns[] = {
	{ 0,  BTN_SOUTH },		/* B */
	{ 1,  BTN_EAST },		/* A */
	{ 2,  BTN_WEST },		/* Y */
	{ 3,  BTN_NORTH },		/* X */
	{ 4,  BTN_TR },			/* R */
	{ 5,  BTN_TR2 },		/* ZR */
	{ 6,  BTN_START },		/* + */
	{ 7,  BTN_THUMBR },		/* stick click */
	{ 8,  BTN_MODE },		/* Home */
	{ 9,  BTN_TRIGGER_HAPPY1 },	/* SL (tentative) */
	{ 10, BTN_TRIGGER_HAPPY2 },	/* SR (tentative) */
	{ 12, BTN_C },			/* C (GameChat) */
	{ }
};

/* Per-controller-type parameters, indexed by enum sw2_ctlr_type. */
struct sw2_ctlr_info {
	u8 input_report_id;
	u8 out_report_id;		/* HID output report ID for rumble */
	u8 imu_offset;			/* report offset of the IMU sample */
	bool has_right_stick;
	bool has_triggers;
	const struct sw2_button_map *buttons;
};

static const struct sw2_ctlr_info sw2_ctlr_infos[] = {
	[SW2_CTLR_PROCON] = {
		.input_report_id = SW2_INPUT_REPORT_PROCON,
		.out_report_id	 = 0x02,
		.imu_offset	 = SW2_OFF_IMU,
		.has_right_stick = true,
		.buttons	 = sw2_procon_btns,
	},
	[SW2_CTLR_JOYCONL] = {
		.input_report_id = SW2_INPUT_REPORT_JOYCONL,
		.out_report_id	 = 0x01,
		.imu_offset	 = SW2_OFF_IMU,
		.buttons	 = sw2_joyconl_btns,
	},
	[SW2_CTLR_JOYCONR] = {
		.input_report_id = SW2_INPUT_REPORT_JOYCONR,
		.out_report_id	 = 0x01,
		.imu_offset	 = SW2_OFF_IMU_JOYCONR,
		.buttons	 = sw2_joyconr_btns,
	},
	[SW2_CTLR_NSOGC] = {
		.input_report_id = SW2_INPUT_REPORT_NSOGC,
		.out_report_id	 = 0x03,
		.imu_offset	 = SW2_OFF_IMU,
		.has_right_stick = true,
		.has_triggers	 = true,
		.buttons	 = sw2_nsogc_btns,
	},
};

/*
 * Proprietary USB initialization sequence, sent to the vendor interface's bulk
 * OUT endpoint. Derived from the NSW2-controller-enabler project; several
 * commands still have unknown semantics. This must be re-sent on every connect
 * (the controller does not persist the configuration). The same sequence is
 * reported to work for the Pro Controller and GameCube controller.
 */
struct sw2_init_cmd {
	const u8 *data;
	size_t len;
};

#define SW2_CMD(...)							\
	{ (const u8[]){ __VA_ARGS__ }, sizeof((u8[]){ __VA_ARGS__ }) }

static const struct sw2_init_cmd sw2_init_seq[] = {
	SW2_CMD(0x03, 0x91, 0x00, 0x0d, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff),
	SW2_CMD(0x07, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00),
	SW2_CMD(0x16, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00),
	SW2_CMD(0x15, 0x91, 0x00, 0x01, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x02,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff),
	SW2_CMD(0x15, 0x91, 0x00, 0x02, 0x00, 0x11, 0x00, 0x00, 0x00, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff),
	SW2_CMD(0x15, 0x91, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00),
	SW2_CMD(0x09, 0x91, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	SW2_CMD(0x0c, 0x91, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x27, 0x00,
		0x00, 0x00),
	SW2_CMD(0x11, 0x91, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00),
	SW2_CMD(0x0a, 0x91, 0x00, 0x08, 0x00, 0x14, 0x00, 0x00, 0x01, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x35, 0x00, 0x46,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	SW2_CMD(0x0c, 0x91, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x27, 0x00,
		0x00, 0x00),
	SW2_CMD(0x03, 0x91, 0x00, 0x0a, 0x00, 0x04, 0x00, 0x00, 0x09, 0x00,
		0x00, 0x00),
	SW2_CMD(0x10, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00),
	SW2_CMD(0x01, 0x91, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00),
	SW2_CMD(0x03, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00),
	SW2_CMD(0x0a, 0x91, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x03, 0x00,
		0x00),
	SW2_CMD(0x09, 0x91, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
};

/* Unpack two 12-bit little-endian values from three bytes. */
static void sw2_unpack_stick(const u8 *d, u16 *x, u16 *y)
{
	*x = get_unaligned_le16(d) & 0x0fff;
	*y = get_unaligned_le16(d + 1) >> 4;
}

/*
 * Map one raw 12-bit stick axis to the calibrated signed output range using the
 * per-unit neutral/positive/negative spans. Falls back to a fixed centre/scale
 * when the controller reports no calibration.
 */
static int sw2_calib_axis(const struct sw2_axis_calib *c, u16 raw, bool negate)
{
	int v = raw;

	if (c->neutral && c->positive && c->negative) {
		v = (v - c->neutral) * (SW2_STICK_AXIS_MAX + 1);
		v /= (v < 0) ? c->negative : c->positive;
	} else {
		v = (v - SW2_STICK_CENTER) * 16;
	}
	if (negate)
		v = -v;
	return clamp(v, SW2_STICK_AXIS_MIN, SW2_STICK_AXIS_MAX);
}

/*
 * Map a raw GameCube trigger byte to 0..SW2_TRIGGER_RANGE. The rest point is the
 * per-unit zero from flash (or a fallback); the full-scale point is tracked at
 * runtime as the highest raw seen, so it adapts to each unit without assuming a
 * fixed maximum.
 */
static int sw2_remap_trigger(struct sw2_ctlr *ctlr, int idx, u8 raw)
{
	int zero = ctlr->trigger_zero[idx] ? : SW2_TRIGGER_RAW_MIN;
	int v;

	if (raw > ctlr->trigger_max[idx])
		ctlr->trigger_max[idx] = raw;
	if (ctlr->trigger_max[idx] <= zero)
		return 0;

	v = (SW2_TRIGGER_RANGE + 1) * (raw - zero) /
	    (ctlr->trigger_max[idx] - zero);
	return clamp(v, 0, SW2_TRIGGER_RANGE);
}

/*
 * Locate the bulk endpoints on the vendor interface (interface 1) of the same
 * USB device the HID interface lives on.
 */
static int sw2_find_vendor_endpoints(struct sw2_ctlr *ctlr)
{
	struct usb_endpoint_descriptor *ep_in = NULL, *ep_out = NULL;
	struct usb_interface *vintf;
	int ret;

	vintf = usb_ifnum_to_if(ctlr->udev, SW2_VENDOR_INTERFACE);
	if (!vintf)
		return -ENODEV;

	ret = usb_find_common_endpoints(vintf->cur_altsetting, &ep_in, &ep_out,
					NULL, NULL);
	if (ret)
		return ret;

	ctlr->ep_in = usb_rcvbulkpipe(ctlr->udev, ep_in->bEndpointAddress);
	ctlr->ep_out = usb_sndbulkpipe(ctlr->udev, ep_out->bEndpointAddress);
	return 0;
}

static int sw2_send_init(struct sw2_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	u8 *rxbuf;
	u8 *txbuf;
	int i, ret, actual;

	txbuf = kmalloc(64, GFP_KERNEL);
	rxbuf = kmalloc(64, GFP_KERNEL);
	if (!txbuf || !rxbuf) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(sw2_init_seq); i++) {
		memcpy(txbuf, sw2_init_seq[i].data, sw2_init_seq[i].len);
		ret = usb_bulk_msg(ctlr->udev, ctlr->ep_out, txbuf,
				   sw2_init_seq[i].len, &actual, 1000);
		if (ret) {
			hid_err(hdev, "init cmd %d failed: %d\n", i, ret);
			goto out;
		}

		/* Best-effort drain of the command's reply. */
		usb_bulk_msg(ctlr->udev, ctlr->ep_in, rxbuf, 64, &actual, 100);
		msleep(50);
	}
	ret = 0;
out:
	kfree(txbuf);
	kfree(rxbuf);
	return ret;
}

/*
 * Read @len bytes (<= SW2_FLASH_READ_MAX) from SPI flash at @addr over the
 * vendor interface into @out. The reply echoes a 16-byte command header
 * followed by the data payload.
 */
static int sw2_read_flash(struct sw2_ctlr *ctlr, u32 addr, u8 len, u8 *out)
{
	u8 *txbuf, *rxbuf;
	int ret, actual, i;

	if (len > SW2_FLASH_READ_MAX)
		return -EINVAL;

	txbuf = kzalloc(64, GFP_KERNEL);
	rxbuf = kzalloc(64, GFP_KERNEL);
	if (!txbuf || !rxbuf) {
		ret = -ENOMEM;
		goto out;
	}

	txbuf[0] = SW2_CMD_READ_SPI;
	txbuf[1] = SW2_REQ_TYPE_REQ;
	txbuf[2] = SW2_IFACE_USB;
	txbuf[3] = SW2_SUBCMD_READ_SPI;
	txbuf[5] = 0x08;
	txbuf[8] = len;
	txbuf[9] = 0x7e;
	put_unaligned_le32(addr, &txbuf[12]);

	mutex_lock(&ctlr->out_mutex);
	ret = usb_bulk_msg(ctlr->udev, ctlr->ep_out, txbuf, 16, &actual, 1000);
	if (ret)
		goto unlock;

	/* The reply may trail stray frames from init; match cmd/type. */
	for (i = 0; i < 8; i++) {
		ret = usb_bulk_msg(ctlr->udev, ctlr->ep_in, rxbuf, 64,
				   &actual, 1000);
		if (ret)
			goto unlock;
		if (actual >= 16 + len && rxbuf[0] == SW2_CMD_READ_SPI &&
		    rxbuf[1] == SW2_RSP_TYPE_RSP) {
			memcpy(out, rxbuf + 16, len);
			ret = 0;
			goto unlock;
		}
	}
	ret = -EIO;
unlock:
	mutex_unlock(&ctlr->out_mutex);
out:
	kfree(txbuf);
	kfree(rxbuf);
	return ret;
}

/* Unpack a 9-byte packed stick-calibration blob (neutral/positive/negative). */
static void sw2_unpack_stick_calib(struct sw2_stick_calib *c, const u8 *d)
{
	c->x.neutral  = d[0] | ((d[1] & 0x0f) << 8);
	c->y.neutral  = (d[1] >> 4) | (d[2] << 4);
	c->x.positive = d[3] | ((d[4] & 0x0f) << 8);
	c->y.positive = (d[4] >> 4) | (d[5] << 4);
	c->x.negative = d[6] | ((d[7] & 0x0f) << 8);
	c->y.negative = (d[7] >> 4) | (d[8] << 4);
}

/*
 * Load per-unit calibration from flash: factory stick calibration (overridden
 * by user calibration when the magic marker is present) and, for the GameCube,
 * the analog-trigger zero points. Best-effort: on any read failure the relevant
 * calibration simply stays at its uncalibrated default.
 */
static void sw2_read_calibration(struct sw2_ctlr *ctlr)
{
	u8 buf[SW2_USER_CALIB_LEN];

	if (!sw2_read_flash(ctlr, SW2_FLASH_FACTORY_STICK1,
			    SW2_STICK_CALIB_LEN, buf))
		sw2_unpack_stick_calib(&ctlr->stick_calib[0], buf);

	if (!sw2_read_flash(ctlr, SW2_FLASH_USER_STICK1,
			    SW2_USER_CALIB_LEN, buf) &&
	    get_unaligned_le16(buf) == SW2_USER_CALIB_MAGIC)
		sw2_unpack_stick_calib(&ctlr->stick_calib[0], buf + 2);

	if (ctlr->info->has_right_stick) {
		if (!sw2_read_flash(ctlr, SW2_FLASH_FACTORY_STICK2,
				    SW2_STICK_CALIB_LEN, buf))
			sw2_unpack_stick_calib(&ctlr->stick_calib[1], buf);

		if (!sw2_read_flash(ctlr, SW2_FLASH_USER_STICK2,
				    SW2_USER_CALIB_LEN, buf) &&
		    get_unaligned_le16(buf) == SW2_USER_CALIB_MAGIC)
			sw2_unpack_stick_calib(&ctlr->stick_calib[1], buf + 2);
	}

	if (ctlr->info->has_triggers &&
	    !sw2_read_flash(ctlr, SW2_FLASH_FACTORY_TRIGGER,
			    SW2_TRIGGER_CALIB_LEN, buf) &&
	    buf[0] != 0xff && buf[1] != 0xff) {
		ctlr->trigger_zero[0] = buf[0];
		ctlr->trigger_zero[1] = buf[1];
	}
}

/* Set the four player-indicator LEDs from a bitmask (bit 0 = LED 1). */
static int sw2_set_player_leds(struct sw2_ctlr *ctlr, u8 mask)
{
	u8 cmd[16] = { SW2_CMD_SET_LED, SW2_REQ_TYPE_REQ, SW2_IFACE_USB,
		       SW2_SUBCMD_SET_LED, 0x00, 0x08, 0x00, 0x00, mask };
	u8 *buf;
	int ret, actual;

	ctlr->led_mask = mask;

	buf = kmemdup(cmd, sizeof(cmd), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&ctlr->out_mutex);
	ret = usb_bulk_msg(ctlr->udev, ctlr->ep_out, buf, sizeof(cmd),
			   &actual, 1000);
	mutex_unlock(&ctlr->out_mutex);

	kfree(buf);
	return ret;
}

/*
 * The vendor command sets all four LEDs at once, so gather the state of every
 * LED into a mask and send them together (the brightness argument is ignored).
 */
static int sw2_player_led_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	struct hid_device *hdev = to_hid_device(led->dev->parent);
	struct sw2_ctlr *ctlr = hid_get_drvdata(hdev);
	u8 mask = 0;
	int i;

	if (!ctlr)
		return -ENODEV;

	for (i = 0; i < SW2_NUM_PLAYER_LEDS; i++)
		if (ctlr->player_leds[i].brightness)
			mask |= BIT(i);

	return sw2_set_player_leds(ctlr, mask);
}

static int sw2_leds_create(struct sw2_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	struct device *dev = &hdev->dev;
	int i, ret, pattern;
	u8 mask;

	/* Assign a unique player number and map it to an LED pattern. */
	ret = ida_alloc(&sw2_player_ida, GFP_KERNEL);
	if (ret < 0)
		return ret;
	ctlr->player_id = ret;
	pattern = ret % SW2_NUM_LED_PATTERNS;
	mask = sw2_player_led_patterns[pattern];
	hid_info(hdev, "assigned player %d\n", pattern + 1);

	for (i = 0; i < SW2_NUM_PLAYER_LEDS; i++) {
		struct led_classdev *led = &ctlr->player_leds[i];

		led->name = devm_kasprintf(dev, GFP_KERNEL, "%s:green:player-%d",
					   dev_name(dev), i + 1);
		if (!led->name)
			return -ENOMEM;
		led->brightness = !!(mask & BIT(i));
		led->max_brightness = 1;
		led->brightness_set_blocking = sw2_player_led_set;
		led->flags = LED_CORE_SUSPENDRESUME | LED_HW_PLUGGABLE;

		ret = devm_led_classdev_register(dev, led);
		if (ret)
			return ret;
	}

	/* Reflect the assigned player pattern on the hardware. */
	return sw2_set_player_leds(ctlr, mask);
}

/* Encode one 5-byte haptic frame at p. */
static void sw2_vib_frame(u8 *p, u16 lf_amp, u16 hf_amp)
{
	u64 v = SW2_VIB_LF_FREQ & 0x1ff;

	v |= (u64)(lf_amp & 0x3ff) << 10;
	v |= (u64)(SW2_VIB_HF_FREQ & 0x1ff) << 20;
	v |= (u64)(hf_amp & 0x3ff) << 30;
	put_unaligned_le32((u32)v, p);	/* low 32 bits */
	p[4] = v >> 32;			/* high 8 bits */
}

/* Build and send the current rumble state as a HID output report. */
static void sw2_send_rumble(struct sw2_ctlr *ctlr)
{
	u8 buf[64] = { 0 };
	int pos = 0, motors, m, f;
	unsigned long flags;
	u16 lf, hf;

	spin_lock_irqsave(&ctlr->rumble_lock, flags);
	lf = ctlr->rumble_lf;
	hf = ctlr->rumble_hf;
	spin_unlock_irqrestore(&ctlr->rumble_lock, flags);

	buf[pos++] = ctlr->info->out_report_id;

	if (ctlr->type == SW2_CTLR_NSOGC) {
		/*
		 * GameCube: simple on/off rumble (byte 2) in a report that also
		 * carries the SET_LED command. Include the current LED mask so a
		 * rumble does not disturb the player LEDs.
		 */
		buf[1] = 0x50 | (ctlr->rumble_pkt_id & 0x0f);
		buf[2] = (lf || hf) ? 0x01 : 0x00;
		buf[5] = SW2_CMD_SET_LED;
		buf[6] = SW2_REQ_TYPE_REQ;
		buf[7] = SW2_IFACE_USB;
		buf[8] = SW2_SUBCMD_SET_LED;
		buf[10] = 0x08;
		buf[13] = ctlr->led_mask;
	} else {
		motors = (ctlr->type == SW2_CTLR_PROCON) ? 2 : 1;
		for (m = 0; m < motors; m++) {
			buf[pos++] = 0x50 | (ctlr->rumble_pkt_id & 0x0f);
			for (f = 0; f < 3; f++) {
				sw2_vib_frame(&buf[pos], lf, hf);
				pos += SW2_VIB_FRAME_LEN;
			}
		}
	}
	ctlr->rumble_pkt_id++;

	hid_hw_output_report(ctlr->hdev, buf, sizeof(buf));
}

static void sw2_rumble_worker(struct work_struct *work)
{
	struct sw2_ctlr *ctlr =
		container_of(to_delayed_work(work), struct sw2_ctlr, rumble_work);
	unsigned long flags;
	bool active;

	sw2_send_rumble(ctlr);

	spin_lock_irqsave(&ctlr->rumble_lock, flags);
	active = ctlr->rumble_active;
	spin_unlock_irqrestore(&ctlr->rumble_lock, flags);

	/* Resend periodically to sustain the effect; one final zero packet stops. */
	if (active)
		schedule_delayed_work(&ctlr->rumble_work,
				      msecs_to_jiffies(SW2_RUMBLE_PERIOD_MS));
}

/* FF_RUMBLE callback (memless). Runs in atomic context: only stash + schedule. */
static int sw2_play_effect(struct input_dev *dev, void *data,
			   struct ff_effect *effect)
{
	struct sw2_ctlr *ctlr = input_get_drvdata(dev);
	unsigned long flags;
	u16 lf, hf;

	if (effect->type != FF_RUMBLE)
		return 0;

	lf = (u32)effect->u.rumble.strong_magnitude * SW2_VIB_AMP_MAX / 0xffff;
	hf = (u32)effect->u.rumble.weak_magnitude * SW2_VIB_AMP_MAX / 0xffff;

	spin_lock_irqsave(&ctlr->rumble_lock, flags);
	ctlr->rumble_lf = lf;
	ctlr->rumble_hf = hf;
	ctlr->rumble_active = lf || hf;
	spin_unlock_irqrestore(&ctlr->rumble_lock, flags);

	mod_delayed_work(system_wq, &ctlr->rumble_work, 0);
	return 0;
}

static void sw2_report_buttons(struct sw2_ctlr *ctlr, const u8 *data)
{
	const struct sw2_button_map *map = ctlr->info->buttons;
	u32 buttons = get_unaligned_le24(&data[SW2_OFF_BUTTONS]);

	for (; map->code; map++)
		input_report_key(ctlr->input, map->code,
				 !!(buttons & BIT(map->bit_nr)));
}

static void sw2_report_sticks(struct sw2_ctlr *ctlr, const u8 *data)
{
	const struct sw2_stick_calib *c = ctlr->stick_calib;
	u16 x, y;

	sw2_unpack_stick(&data[SW2_OFF_STICK_L], &x, &y);
	input_report_abs(ctlr->input, ABS_X, sw2_calib_axis(&c[0].x, x, false));
	input_report_abs(ctlr->input, ABS_Y, sw2_calib_axis(&c[0].y, y, true));

	if (ctlr->info->has_right_stick) {
		sw2_unpack_stick(&data[SW2_OFF_STICK_R], &x, &y);
		input_report_abs(ctlr->input, ABS_RX, sw2_calib_axis(&c[1].x, x, false));
		input_report_abs(ctlr->input, ABS_RY, sw2_calib_axis(&c[1].y, y, true));
	}
}

/*
 * Report the IMU sample. Only the accelerometer is decoded: the three accel
 * s16 values sit at imu_offset + 2/6/10 (X/Y/Z), interleaved with three other
 * s16 slots (imu_offset + 0/4/8) that, despite their position, carry only noise
 * over USB at our config (uncorrelated and non-zero at rest), so they are NOT a
 * usable gyro - see docs/PROTOCOL.md. Accel rests at ~4096 LSB/g.
 */
static void sw2_report_imu(struct sw2_ctlr *ctlr, const u8 *data)
{
	const u8 *p = data + ctlr->info->imu_offset;
	ktime_t now = ktime_get();

	input_report_abs(ctlr->imu_input, ABS_X, (s16)get_unaligned_le16(p + 2));
	input_report_abs(ctlr->imu_input, ABS_Y, (s16)get_unaligned_le16(p + 6));
	input_report_abs(ctlr->imu_input, ABS_Z, (s16)get_unaligned_le16(p + 10));

	/*
	 * Report a monotonic microsecond timestamp accumulated from the
	 * measured inter-sample interval. The report does carry a 16-bit
	 * hardware timestamp, but its unit is unconfirmed, so we use the
	 * measured arrival time instead.
	 */
	if (ctlr->imu_last)
		ctlr->imu_timestamp_us +=
			ktime_to_us(ktime_sub(now, ctlr->imu_last));
	ctlr->imu_last = now;
	input_event(ctlr->imu_input, EV_MSC, MSC_TIMESTAMP,
		    ctlr->imu_timestamp_us);

	input_sync(ctlr->imu_input);
}

static int sw2_hid_event(struct hid_device *hdev, struct hid_report *report,
			 u8 *data, int size)
{
	struct sw2_ctlr *ctlr = hid_get_drvdata(hdev);

	if (!ctlr || !ctlr->input)
		return 0;
	if (size < SW2_OFF_STICK_R + 3 || data[0] != ctlr->info->input_report_id)
		return 0;

	sw2_report_buttons(ctlr, data);
	sw2_report_sticks(ctlr, data);

	if (ctlr->info->has_triggers) {
		input_report_abs(ctlr->input, ABS_Z,
				 sw2_remap_trigger(ctlr, 0, data[SW2_OFF_TRIGGER_L]));
		input_report_abs(ctlr->input, ABS_RZ,
				 sw2_remap_trigger(ctlr, 1, data[SW2_OFF_TRIGGER_R]));
	}

	input_sync(ctlr->input);

	if (ctlr->imu_input &&
	    size >= ctlr->info->imu_offset + SW2_IMU_LEN)
		sw2_report_imu(ctlr, data);

	return 0;
}

static int sw2_input_create(struct sw2_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	const struct sw2_button_map *map = ctlr->info->buttons;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(&hdev->dev);
	if (!input)
		return -ENOMEM;

	input->id.bustype = BUS_USB;
	input->id.vendor = hdev->vendor;
	input->id.product = hdev->product;
	input->id.version = hdev->version;
	input->name = hdev->name;
	input_set_drvdata(input, ctlr);

	for (; map->code; map++)
		input_set_capability(input, EV_KEY, map->code);

	input_set_abs_params(input, ABS_X, SW2_STICK_AXIS_MIN, SW2_STICK_AXIS_MAX, 32, 128);
	input_set_abs_params(input, ABS_Y, SW2_STICK_AXIS_MIN, SW2_STICK_AXIS_MAX, 32, 128);
	if (ctlr->info->has_right_stick) {
		input_set_abs_params(input, ABS_RX, SW2_STICK_AXIS_MIN, SW2_STICK_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_RY, SW2_STICK_AXIS_MIN, SW2_STICK_AXIS_MAX, 32, 128);
	}
	if (ctlr->info->has_triggers) {
		input_set_abs_params(input, ABS_Z, 0, SW2_TRIGGER_RANGE, 32, 128);
		input_set_abs_params(input, ABS_RZ, 0, SW2_TRIGGER_RANGE, 32, 128);
	}

	input_set_capability(input, EV_FF, FF_RUMBLE);
	ret = input_ff_create_memless(input, NULL, sw2_play_effect);
	if (ret)
		return ret;

	ctlr->input = input;
	return input_register_device(input);
}

static int sw2_imu_input_create(struct sw2_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	struct input_dev *imu;

	imu = devm_input_allocate_device(&hdev->dev);
	if (!imu)
		return -ENOMEM;

	imu->id.bustype = BUS_USB;
	imu->id.vendor = hdev->vendor;
	imu->id.product = hdev->product;
	imu->id.version = hdev->version;
	imu->name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s IMU", hdev->name);
	if (!imu->name)
		return -ENOMEM;
	input_set_drvdata(imu, ctlr);

	/* accelerometer only (the gyro slots carry noise over USB; see PROTOCOL.md) */
	input_set_abs_params(imu, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(imu, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(imu, ABS_Z, -32768, 32767, 0, 0);
	input_abs_set_res(imu, ABS_X, SW2_IMU_ACCEL_RES_PER_G);
	input_abs_set_res(imu, ABS_Y, SW2_IMU_ACCEL_RES_PER_G);
	input_abs_set_res(imu, ABS_Z, SW2_IMU_ACCEL_RES_PER_G);
	input_set_capability(imu, EV_MSC, MSC_TIMESTAMP);
	__set_bit(INPUT_PROP_ACCELEROMETER, imu->propbit);

	ctlr->imu_input = imu;
	return input_register_device(imu);
}

static enum sw2_ctlr_type sw2_type_from_product(__u16 product)
{
	switch (product) {
	case USB_DEVICE_ID_NINTENDO_SW2_JOYCONL:
		return SW2_CTLR_JOYCONL;
	case USB_DEVICE_ID_NINTENDO_SW2_JOYCONR:
		return SW2_CTLR_JOYCONR;
	case USB_DEVICE_ID_NINTENDO_SW2_NSOGC:
		return SW2_CTLR_NSOGC;
	case USB_DEVICE_ID_NINTENDO_SW2_PROCON:
	default:
		return SW2_CTLR_PROCON;
	}
}

static int sw2_hid_probe(struct hid_device *hdev,
			 const struct hid_device_id *id)
{
	struct sw2_ctlr *ctlr;
	int ret;

	ctlr = devm_kzalloc(&hdev->dev, sizeof(*ctlr), GFP_KERNEL);
	if (!ctlr)
		return -ENOMEM;

	ctlr->hdev = hdev;
	ctlr->udev = interface_to_usbdev(to_usb_interface(hdev->dev.parent));
	ctlr->player_id = -1;
	ctlr->trigger_max[0] = SW2_TRIGGER_RAW_FULL;
	ctlr->trigger_max[1] = SW2_TRIGGER_RAW_FULL;
	mutex_init(&ctlr->out_mutex);
	ctlr->type = sw2_type_from_product(id->product);
	ctlr->info = &sw2_ctlr_infos[ctlr->type];
	spin_lock_init(&ctlr->rumble_lock);
	INIT_DELAYED_WORK(&ctlr->rumble_work, sw2_rumble_worker);
	hid_set_drvdata(hdev, ctlr);

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = sw2_find_vendor_endpoints(ctlr);
	if (ret) {
		hid_err(hdev, "no vendor bulk endpoints: %d\n", ret);
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		return ret;

	ret = hid_hw_open(hdev);
	if (ret)
		goto err_stop;

	ret = sw2_send_init(ctlr);
	if (ret)
		goto err_close;

	sw2_read_calibration(ctlr);

	ret = sw2_input_create(ctlr);
	if (ret)
		goto err_close;

	if (ctlr->info->imu_offset) {
		ret = sw2_imu_input_create(ctlr);
		if (ret)
			goto err_close;
	}

	ret = sw2_leds_create(ctlr);
	if (ret)
		hid_warn(hdev, "failed to register player LEDs: %d\n", ret);

	return 0;

err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void sw2_hid_remove(struct hid_device *hdev)
{
	struct sw2_ctlr *ctlr = hid_get_drvdata(hdev);

	if (ctlr) {
		cancel_delayed_work_sync(&ctlr->rumble_work);
		if (ctlr->player_id >= 0)
			ida_free(&sw2_player_ida, ctlr->player_id);
	}
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id sw2_hid_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_SW2_PROCON) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_SW2_JOYCONL) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_SW2_JOYCONR) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_SW2_NSOGC) },
	{ }
};
MODULE_DEVICE_TABLE(hid, sw2_hid_devices);

static struct hid_driver sw2_hid_driver = {
	.name		= "nintendo2",
	.id_table	= sw2_hid_devices,
	.probe		= sw2_hid_probe,
	.remove		= sw2_hid_remove,
	.raw_event	= sw2_hid_event,
};

static int __init sw2_init(void)
{
	return hid_register_driver(&sw2_hid_driver);
}

static void __exit sw2_exit(void)
{
	hid_unregister_driver(&sw2_hid_driver);
	ida_destroy(&sw2_player_ida);
}

module_init(sw2_init);
module_exit(sw2_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan McClelland <rymcclel@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo Switch 2 controllers (USB)");
