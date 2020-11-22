/*
 * Open VFD Driver
 *
 * Copyright (C) 2018 Arthur Liberman (arthur_liberman (at) hotmail.com)
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 */

#include <dm.h>
#include <linux/compat.h>
#include <asm-generic/gpio.h>
#include <asm/arch/gpio.h>
#include "openvfd_drv.h"
#include "controllers/controller_list.h"

unsigned char vfd_display_auto_power = 0;

static struct vfd_platform_data *pdata = NULL;
static struct controller_interface *controller = NULL;
static struct mutex mutex;

static struct vfd_display_data disp;

/****************************************************************
 *	Function Name:		FD628_GetKey
 *	Description:		Read key code value
 *	Parameters:		void
 *	Return value:		INT32U returns the key value
 **************************************************************************************************************************************
Key value encoding
	| 0	| 0	| 0	| 0	| 0	| 0	| KS10	| KS9	| KS8	| KS7	| KS6	| KS5	| KS4	| KS3	| KS2	| KS1	|
KEYI1 	| bit15	| bit14	| bit13	| bit12	| bit11	| bit10	| bit9	| bit8	| bit7	| bit6	| bit5	| bit4	| bit3	| bit2	| bit1	| bit0	|
KEYI2 	| bit31	| bit30	| bit29	| bit28	| bit27	| bit26	| bit25	| bit24	| bit23	| bit22	| bit21	| bit20	| bit19	| bit18	| bit17	| bit16	|
***************************************************************************************************************************************/
static u_int32 FD628_GetKey(struct vfd_dev *dev)
{
	u_int8 i, keyDataBytes[5];
	u_int32 FD628_KeyData = 0;
	mutex_lock(&mutex);
	controller->read_data(keyDataBytes, sizeof(keyDataBytes));
	mutex_unlock(&mutex);
	for (i = 0; i != 5; i++) {			/* Pack 5 bytes of key code values into 2 words */
		if (keyDataBytes[i] & 0x01)
			FD628_KeyData |= (0x00000001 << i * 2);
		if (keyDataBytes[i] & 0x02)
			FD628_KeyData |= (0x00010000 << i * 2);
		if (keyDataBytes[i] & 0x08)
			FD628_KeyData |= (0x00000002 << i * 2);
		if (keyDataBytes[i] & 0x10)
			FD628_KeyData |= (0x00020000 << i * 2);
	}

	return (FD628_KeyData);
}

static void unlocked_set_power(unsigned char state)
{
	if (vfd_display_auto_power && controller) {
		controller->set_power(state);
		if (state && pdata)
			controller->set_brightness_level(pdata->dev->brightness);
	}
}

static void set_power(unsigned char state)
{
	mutex_lock(&mutex);
	unlocked_set_power(state);
	mutex_unlock(&mutex);
}

static void init_controller(struct vfd_dev *dev)
{
	struct controller_interface *temp_ctlr;

	switch (dev->dtb_active.display.controller) {
	case CONTROLLER_FD628:
	case CONTROLLER_FD620:
	case CONTROLLER_TM1618:
		pr_dbg2("Select FD628 controller\n");
		temp_ctlr = init_fd628(dev);
		break;
	case CONTROLLER_HBS658:
		pr_dbg2("Select HBS658 controller\n");
		temp_ctlr = init_fd628(dev);
		break;
	case CONTROLLER_FD650:
		pr_dbg2("Select FD650 controller\n");
		temp_ctlr = init_fd650(dev);
		break;
	case CONTROLLER_FD655:
		pr_dbg2("Select FD655 controller\n");
		temp_ctlr = init_fd650(dev);
		break;
	case CONTROLLER_FD6551:
		pr_dbg2("Select FD6551 controller\n");
		temp_ctlr = init_fd650(dev);
		break;
	default:
		pr_dbg2("Select Dummy controller\n");
		temp_ctlr = init_dummy(dev);
		break;
	}

	if (controller != temp_ctlr) {
		unlocked_set_power(0);
		controller = temp_ctlr;
		if (!controller->init()) {
			pr_dbg2("Failed to initialize the controller, reverting to Dummy controller\n");
			controller = init_dummy(dev);
			dev->dtb_active.display.controller = CONTROLLER_7S_MAX;
		}
	}
}

ssize_t openvfd_dev_read(char __user * buf)
{
	__u32 disk = 0;
	struct vfd_dev *dev = pdata->dev;
	__u32 diskvalue = 0;
	int rbuf[2] = { 0 };

	if (controller == NULL) {
		pr_error("openvfd controller uninitialized\n");
		return -1;
	}

	//pr_dbg("start read keyboard value...............\r\n");
	if (dev->Keyboard_diskstatus == 1) {
		diskvalue = FD628_GetKey(dev);
		if (diskvalue == 0)
			return 0;
	}
	dev->key_respond_status = 0;
	rbuf[1] = dev->key_fg;
	if (dev->key_fg)
		rbuf[0] = disk;
	else
		rbuf[0] = diskvalue;
	//pr_dbg("Keyboard value:%d\n, status : %d\n",rbuf[0],rbuf[1]);
	memcpy(buf, rbuf, sizeof(rbuf));
	return sizeof(rbuf);
}

/**
 * @param buf: Incoming LED codes.
 * 		  [0]	Display indicators mask (wifi, eth, usb, etc.)
 * 		  [1-4]	7 segment characters, to be displayed left to right.
 * @return
 */
static ssize_t openvfd_dev_write(const char __user * buf, size_t count)
{
	ssize_t status = 0;
	unsigned long missing;
	static struct vfd_display_data data;

	if (controller == NULL) {
		pr_error("openvfd controller uninitialized\n");
		return -1;
	}

	if (count == sizeof(data)) {
		missing = copy_from_user(&data, buf, count);
		if (missing == 0 && count > 0) {
			mutex_lock(&mutex);
			if (controller->write_display_data(&data))
				pr_dbg("openvfd_dev_write count : %ld\n", count);
			else {
				status = -1;
				pr_error("openvfd_dev_write failed to write %ld bytes (display_data)\n", count);
			}
			mutex_unlock(&mutex);
		}
	} else if (count > 0) {
		unsigned char *raw_data;
		raw_data = kzalloc(count, GFP_KERNEL);
		if (raw_data) {
			missing = copy_from_user(raw_data, buf, count);
			mutex_lock(&mutex);
			if (controller->write_data((unsigned char*)raw_data, count))
				pr_dbg("openvfd_dev_write count : %ld\n", count);
			else {
				status = -1;
				pr_error("openvfd_dev_write failed to write %ld bytes (raw_data)\n", count);
			}
			mutex_unlock(&mutex);
			kfree(raw_data);
		}
		else {
			status = -1;
			pr_error("openvfd_dev_write failed to allocate %ld bytes (raw_data)\n", count);
		}
	}

	return status;
}

static void led_on_store(const char *buf)
{
	mutex_lock(&mutex);
	controller->set_icon(buf, 1);
	mutex_unlock(&mutex);
}

static void led_off_store(const char *buf)
{
	mutex_lock(&mutex);
	controller->set_icon(buf, 0);
	mutex_unlock(&mutex);
}

unsigned char vfd_gpio_protocol[2] = { 0x00, 0x00 };
unsigned char vfd_chars[7] = { 0, 1, 2, 3, 4, 5, 6 };
unsigned char vfd_dot_bits[8] = { 0, 1, 2, 3, 4, 5, 6, 0 };
unsigned char vfd_display_type[4] = { 0x00, 0x00, 0x00, 0x00 };
unsigned char vfd_brightness = 0xFF;
int vfd_gpio_protocol_argc = 2;
int vfd_chars_argc = 0;
int vfd_dot_bits_argc = 0;
int vfd_display_type_argc = 0;

static int get_param_array_from_dt(int node, const char *name, unsigned char *array)
{
	int size, i;
	unsigned int dt_array[16];

	size = fdtdec_get_int_array_count(gd->fdt_blob, node, name, dt_array, ARRAY_SIZE(dt_array));
	if (size < 0) {
		pr_dbg2("cannot find %s\n", name);
		return size;
	}

	for (i = 0; i < size; i++) {
		array[i] = dt_array[i];
	}

	return size;
}

static void print_param_debug(const char *label, int argc, unsigned char param[])
{
	int i, len = 0;
	char buffer[1024];
	len = scnprintf(buffer, sizeof(buffer), "%s", label);
	if (argc)
		for (i = 0; i < argc; i++)
			len += scnprintf(buffer + len, sizeof(buffer), "#%d = 0x%02X; ", i, param[i]);
	else
		len += scnprintf(buffer + len, sizeof(buffer), "Empty.");
	pr_dbg("%s\n", buffer);
}

static int verify_module_params(struct vfd_dev *dev)
{
	int ret = (vfd_chars_argc >= 5 && vfd_dot_bits_argc >= 7 && vfd_display_type_argc == 4) ? 1 : -1;

	print_param_debug("vfd_gpio_protocol:\t", vfd_gpio_protocol_argc, vfd_gpio_protocol);
	print_param_debug("vfd_chars:\t\t", vfd_chars_argc, vfd_chars);
	print_param_debug("vfd_dot_bits:\t\t", vfd_dot_bits_argc, vfd_dot_bits);
	print_param_debug("vfd_display_type:\t", vfd_display_type_argc, vfd_display_type);

	dev->hw_protocol.protocol = vfd_gpio_protocol[0];
	dev->hw_protocol.device_id = vfd_gpio_protocol[1];

	if (ret >= 0) {
		int i;
		for (i = 0; i < 7; i++)
			dev->dtb_active.dat_index[i] = vfd_chars[i];
		for (i = 0; i < 8; i++)
			dev->dtb_active.led_dots[i] = ledDots[vfd_dot_bits[i]];
		dev->dtb_active.display.type = vfd_display_type[0];
		dev->dtb_active.display.reserved = vfd_display_type[1];
		dev->dtb_active.display.flags = vfd_display_type[2];
		dev->dtb_active.display.controller = vfd_display_type[3];
	}

	return ret >= 0;
}

static int request_pin(int node, const char *name, struct vfd_pin *pin, unsigned char enable_skip)
{
	int ret = 0;
	struct gpio_desc gpio;

	pin->flags.bits.is_requested = 0;
	if (!enable_skip) {
		ret = gpio_request_by_name_nodev(offset_to_ofnode(node), name, 0, &gpio, GPIOD_IS_OUT);
		if (!ret) {
			pin->flags.bits.is_requested = 1;
			pin->pin = gpio_get_number(&gpio);
		} else {
			pr_error("can't request gpio of %s", name);
		}
	}
	return ret;
}

inline void reset_display_data(void)
{
	memset(&disp, 0, sizeof(disp));
	disp.mode = DISPLAY_MODE_TITLE;
	disp.string_main[0] = ' ';
	disp.string_main[1] = ' ';
	disp.string_main[2] = ' ';
	disp.string_main[3] = ' ';
	pdata->dev->status_led_mask = 0;
}

int openvfd_driver_probe(void)
{
	int node, state = -EINVAL;
	u_int8 allow_skip_request = vfd_gpio_protocol[0] > 0;

	pr_dbg("%s get in\n", __func__);

	node = fdt_node_offset_by_compatible(gd->fdt_blob, -1, DEV_NAME);
	if (node < 0) {
		pr_dbg2("cannot find openvfd node\n");
		state = -EINVAL;
		goto get_openvfd_node_fail;
	}

	vfd_chars_argc = get_param_array_from_dt(node, "vfd_chars", vfd_chars);
	vfd_dot_bits_argc = get_param_array_from_dt(node, "vfd_dot_bits", vfd_dot_bits);
	vfd_display_type_argc = get_param_array_from_dt(node, "vfd_display_type", vfd_display_type);
	get_param_array_from_dt(node, "vfd_brightness", &vfd_brightness);

	pdata = kzalloc(sizeof(struct vfd_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_error("platform data is required!\n");
		state = -EINVAL;
		goto get_openvfd_mem_fail;
	}

	pdata->dev = kzalloc(sizeof(*(pdata->dev)), GFP_KERNEL);
	if (!(pdata->dev)) {
		pr_error("platform dev is required!\n");
		goto get_param_mem_fail;
	}

	pdata->dev->mutex = &mutex;
	pr_dbg2("Version: %s\n", OPENVFD_DRIVER_VERSION);
	if (!verify_module_params(pdata->dev)) {
		pr_error("Failed to verify VFD configuration.\n");
		goto get_param_mem_fail;
	}

	if (request_pin(node, "gpio-clk", &pdata->dev->clk_pin, allow_skip_request))
		goto get_gpio_req_fail;
	if (request_pin(node, "gpio-dat", &pdata->dev->dat_pin, allow_skip_request))
		goto get_gpio_req_fail;
	if (request_pin(node, "gpio-stb", &pdata->dev->stb_pin, allow_skip_request))
		goto get_gpio_req_fail;

	pdata->dev->dtb_default = pdata->dev->dtb_active;
	pdata->dev->brightness = 0xFF;

	mutex_lock(&mutex);
	init_controller(pdata->dev);
	controller->set_brightness_level(vfd_brightness);
	reset_display_data();
	mutex_unlock(&mutex);

	return 0;

get_gpio_req_fail:
	if (pdata->dev->stb_pin.flags.bits.is_requested)
		gpio_free(pdata->dev->stb_pin.pin);
	if (pdata->dev->dat_pin.flags.bits.is_requested)
		gpio_free(pdata->dev->dat_pin.pin);
	if (pdata->dev->clk_pin.flags.bits.is_requested)
		gpio_free(pdata->dev->clk_pin.pin);
get_param_mem_fail:
	kfree(pdata->dev);
get_openvfd_mem_fail:
	kfree(pdata);
get_openvfd_node_fail:
	if (pdata && pdata->dev)
		mutex_unlock(&mutex);
	return state;
}

int openvfd_driver_remove(void)
{
	set_power(0);
#ifdef CONFIG_OF
	if (pdata->dev->stb_pin.flags.bits.is_requested)
		gpio_free(pdata->dev->stb_pin.pin);
	if (pdata->dev->dat_pin.flags.bits.is_requested)
		gpio_free(pdata->dev->dat_pin.pin);
	if (pdata->dev->clk_pin.flags.bits.is_requested)
		gpio_free(pdata->dev->clk_pin.pin);
	kfree(pdata->dev);
	kfree(pdata);
	pdata = NULL;
#endif
	return 0;
}

void openvfd_set_title(const char *seg_str)
{
	mutex_lock(&mutex);
	disp.string_main[0] = seg_str[0];
	disp.string_main[1] = seg_str[1];
	disp.string_main[2] = seg_str[2];
	disp.string_main[3] = seg_str[3];
	openvfd_dev_write((char *)&disp, sizeof(disp));
	mutex_unlock(&mutex);
}

void openvfd_set_icon(const char *name, bool on)
{
	mutex_lock(&mutex);
	if (on)
		led_on_store(name);
	else
		led_off_store(name);
	openvfd_dev_write((char *)&disp, sizeof(disp));
	mutex_unlock(&mutex);
}

void openvfd_clear(void)
{
	mutex_lock(&mutex);
	reset_display_data();
	openvfd_dev_write((char *)&disp, sizeof(disp));
	mutex_unlock(&mutex);
}
