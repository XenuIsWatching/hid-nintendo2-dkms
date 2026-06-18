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
 *   libsdl-org/SDL SDL_hidapi_switch2.c
 */

#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>

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

/* Offsets within an input report (data[0] is the report ID). */
#define SW2_OFF_BUTTONS		3	/* 3 bytes of bit-packed buttons */
#define SW2_OFF_STICK_L		6	/* 3 bytes -> two 12-bit axes */
#define SW2_OFF_STICK_R		9	/* 3 bytes -> two 12-bit axes */
#define SW2_OFF_TRIGGER_L	13	/* analog left trigger (GameCube) */
#define SW2_OFF_TRIGGER_R	14	/* analog right trigger (GameCube) */

/*
 * IMU: one sample per report, six s16 LE values interleaved per axis
 * (gyro then accel: gX,aX,gY,aY,gZ,aZ at +0,+2,...,+10). Confirmed at the same
 * offset for the Pro Controller (report 0x09) and the GameCube controller
 * (report 0x0a) via rotation captures. The Joy-Con 2 offset is not yet known.
 */
#define SW2_OFF_IMU		32
#define SW2_IMU_LEN		12

/* Accelerometer resolution: ~4096 LSB per g (rest |a| measured ~4096). */
#define SW2_IMU_ACCEL_RES_PER_G	4096
/*
 * Gyro resolution in LSB per degree/s. Not yet measured for the Switch 2;
 * reuse the original Switch value as a placeholder until calibrated.
 */
#define SW2_IMU_GYRO_RES_PER_DPS	14247

#define SW2_STICK_CENTER	2048	/* 12-bit stick midpoint */
#define SW2_STICK_MAX		4095
/* GameCube analog triggers report roughly 0x24..0xf0; remap to 0..255. */
#define SW2_TRIGGER_RAW_MIN	0x24
#define SW2_TRIGGER_RAW_MAX	0xf0

enum sw2_ctlr_type {
	SW2_CTLR_PROCON,
	SW2_CTLR_JOYCONL,
	SW2_CTLR_JOYCONR,
	SW2_CTLR_NSOGC,
};

struct sw2_ctlr {
	struct hid_device *hdev;
	struct input_dev *input;
	struct input_dev *imu_input;
	struct usb_device *udev;
	enum sw2_ctlr_type type;
	u8 input_report_id;
	u8 imu_offset;		/* report offset of IMU sample, 0 if none */
	unsigned int ep_out;	/* bulk OUT pipe on the vendor interface */
	unsigned int ep_in;	/* bulk IN pipe on the vendor interface */
};

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

static const struct sw2_button_map *sw2_button_map_for(enum sw2_ctlr_type type)
{
	switch (type) {
	case SW2_CTLR_NSOGC:
		return sw2_nsogc_btns;
	case SW2_CTLR_JOYCONL:
		return sw2_joyconl_btns;
	case SW2_CTLR_JOYCONR:
		return sw2_joyconr_btns;
	case SW2_CTLR_PROCON:
	default:
		return sw2_procon_btns;
	}
}

static bool sw2_has_right_stick(enum sw2_ctlr_type type)
{
	return type == SW2_CTLR_PROCON || type == SW2_CTLR_NSOGC;
}

static bool sw2_has_triggers(enum sw2_ctlr_type type)
{
	return type == SW2_CTLR_NSOGC;
}

/* Unpack two 12-bit little-endian values from three bytes. */
static void sw2_unpack_stick(const u8 *d, u16 *x, u16 *y)
{
	*x = d[0] | ((d[1] & 0x0f) << 8);
	*y = (d[1] >> 4) | (d[2] << 4);
}

static int sw2_remap_trigger(u8 raw)
{
	int v = clamp_t(int, raw, SW2_TRIGGER_RAW_MIN, SW2_TRIGGER_RAW_MAX);

	return (v - SW2_TRIGGER_RAW_MIN) * 255 /
	       (SW2_TRIGGER_RAW_MAX - SW2_TRIGGER_RAW_MIN);
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

static void sw2_report_buttons(struct sw2_ctlr *ctlr, const u8 *data)
{
	const struct sw2_button_map *map = sw2_button_map_for(ctlr->type);
	u32 buttons = data[SW2_OFF_BUTTONS] |
		      (data[SW2_OFF_BUTTONS + 1] << 8) |
		      (data[SW2_OFF_BUTTONS + 2] << 16);

	for (; map->code; map++)
		input_report_key(ctlr->input, map->code,
				 !!(buttons & BIT(map->bit_nr)));
}

static void sw2_report_sticks(struct sw2_ctlr *ctlr, const u8 *data)
{
	u16 x, y;

	sw2_unpack_stick(&data[SW2_OFF_STICK_L], &x, &y);
	input_report_abs(ctlr->input, ABS_X, x);
	input_report_abs(ctlr->input, ABS_Y, SW2_STICK_MAX - y);

	if (sw2_has_right_stick(ctlr->type)) {
		sw2_unpack_stick(&data[SW2_OFF_STICK_R], &x, &y);
		input_report_abs(ctlr->input, ABS_RX, x);
		input_report_abs(ctlr->input, ABS_RY, SW2_STICK_MAX - y);
	}
}

static s16 sw2_s16le(const u8 *p)
{
	return (s16)(p[0] | (p[1] << 8));
}

/*
 * Report the IMU sample. The six s16 values are interleaved per axis as
 * gyro,accel for X, then Y, then Z, starting at ctlr->imu_offset. Gyro is
 * reported on ABS_R[XYZ], accel on ABS_[XYZ], on a separate input device.
 */
static void sw2_report_imu(struct sw2_ctlr *ctlr, const u8 *data)
{
	const u8 *p = data + ctlr->imu_offset;

	input_report_abs(ctlr->imu_input, ABS_RX, sw2_s16le(p + 0));  /* gyro X */
	input_report_abs(ctlr->imu_input, ABS_X,  sw2_s16le(p + 2));  /* acc X */
	input_report_abs(ctlr->imu_input, ABS_RY, sw2_s16le(p + 4));  /* gyro Y */
	input_report_abs(ctlr->imu_input, ABS_Y,  sw2_s16le(p + 6));  /* acc Y */
	input_report_abs(ctlr->imu_input, ABS_RZ, sw2_s16le(p + 8));  /* gyro Z */
	input_report_abs(ctlr->imu_input, ABS_Z,  sw2_s16le(p + 10)); /* acc Z */
	input_sync(ctlr->imu_input);
}

static int sw2_hid_event(struct hid_device *hdev, struct hid_report *report,
			 u8 *data, int size)
{
	struct sw2_ctlr *ctlr = hid_get_drvdata(hdev);

	if (!ctlr || !ctlr->input)
		return 0;
	if (size < SW2_OFF_STICK_R + 3 || data[0] != ctlr->input_report_id)
		return 0;

	sw2_report_buttons(ctlr, data);
	sw2_report_sticks(ctlr, data);

	if (sw2_has_triggers(ctlr->type)) {
		input_report_abs(ctlr->input, ABS_Z,
				 sw2_remap_trigger(data[SW2_OFF_TRIGGER_L]));
		input_report_abs(ctlr->input, ABS_RZ,
				 sw2_remap_trigger(data[SW2_OFF_TRIGGER_R]));
	}

	input_sync(ctlr->input);

	if (ctlr->imu_input && ctlr->imu_offset &&
	    size >= ctlr->imu_offset + SW2_IMU_LEN)
		sw2_report_imu(ctlr, data);

	return 0;
}

static int sw2_input_create(struct sw2_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	const struct sw2_button_map *map = sw2_button_map_for(ctlr->type);
	struct input_dev *input;

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

	input_set_abs_params(input, ABS_X, 0, SW2_STICK_MAX, 16, 128);
	input_set_abs_params(input, ABS_Y, 0, SW2_STICK_MAX, 16, 128);
	if (sw2_has_right_stick(ctlr->type)) {
		input_set_abs_params(input, ABS_RX, 0, SW2_STICK_MAX, 16, 128);
		input_set_abs_params(input, ABS_RY, 0, SW2_STICK_MAX, 16, 128);
	}
	if (sw2_has_triggers(ctlr->type)) {
		input_set_abs_params(input, ABS_Z, 0, 255, 0, 0);
		input_set_abs_params(input, ABS_RZ, 0, 255, 0, 0);
	}

	ctlr->input = input;
	return input_register_device(input);
}

static int sw2_imu_input_create(struct sw2_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	struct input_dev *imu;
	int g = SW2_IMU_GYRO_RES_PER_DPS;
	int a = SW2_IMU_ACCEL_RES_PER_G;

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

	/* accelerometer (g units) */
	input_set_abs_params(imu, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(imu, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(imu, ABS_Z, -32768, 32767, 0, 0);
	input_abs_set_res(imu, ABS_X, a);
	input_abs_set_res(imu, ABS_Y, a);
	input_abs_set_res(imu, ABS_Z, a);
	/* gyroscope (deg/s) */
	input_set_abs_params(imu, ABS_RX, -32768, 32767, 0, 0);
	input_set_abs_params(imu, ABS_RY, -32768, 32767, 0, 0);
	input_set_abs_params(imu, ABS_RZ, -32768, 32767, 0, 0);
	input_abs_set_res(imu, ABS_RX, g);
	input_abs_set_res(imu, ABS_RY, g);
	input_abs_set_res(imu, ABS_RZ, g);
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

static const u8 sw2_input_report_ids[] = {
	[SW2_CTLR_PROCON]  = SW2_INPUT_REPORT_PROCON,
	[SW2_CTLR_JOYCONL] = SW2_INPUT_REPORT_JOYCONL,
	[SW2_CTLR_JOYCONR] = SW2_INPUT_REPORT_JOYCONR,
	[SW2_CTLR_NSOGC]   = SW2_INPUT_REPORT_NSOGC,
};

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
	ctlr->type = sw2_type_from_product(id->product);
	ctlr->input_report_id = sw2_input_report_ids[ctlr->type];
	/* IMU offset confirmed for the Pro and GameCube controllers. */
	if (ctlr->type == SW2_CTLR_PROCON || ctlr->type == SW2_CTLR_NSOGC)
		ctlr->imu_offset = SW2_OFF_IMU;
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

	ret = sw2_input_create(ctlr);
	if (ret)
		goto err_close;

	if (ctlr->imu_offset) {
		ret = sw2_imu_input_create(ctlr);
		if (ret)
			goto err_close;
	}

	return 0;

err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void sw2_hid_remove(struct hid_device *hdev)
{
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
module_hid_driver(sw2_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan McClelland <rymcclel@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo Switch 2 controllers (USB)");
