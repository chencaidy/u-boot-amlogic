// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Caidy
 * Author: Chen Caidy <chen@caidy.cc>
 */

#include <command.h>
#include <common.h>
#include <dm.h>
#include <env.h>
#include <init.h>
#include <net.h>
#include <asm/io.h>
#include <asm/arch/boot.h>
#include <asm/arch/sm.h>
#include <asm/arch/eth.h>

#define EFUSE_MAC_OFFSET	20
#define EFUSE_MAC_SIZE		12
#define MAC_ADDR_LEN		6

extern int openvfd_driver_probe(void);
extern int openvfd_driver_remove(void);

int misc_init_r(void)
{
	u8 mac_addr[MAC_ADDR_LEN];
	char efuse_mac_addr[EFUSE_MAC_SIZE], tmp[3];
	ssize_t len;

	openvfd_driver_probe();

	switch (meson_get_boot_device())
	{
	case BOOT_DEVICE_SD:
		run_command("vfd icon sd on", 0);
		break;
	case BOOT_DEVICE_USB:
		run_command("vfd icon usb on", 0);
		break;
	default:
		break;
	}

	meson_eth_init(PHY_INTERFACE_MODE_RGMII, 0);

	if (!eth_env_get_enetaddr("ethaddr", mac_addr)) {
		len = meson_sm_read_efuse(EFUSE_MAC_OFFSET,
					  efuse_mac_addr, EFUSE_MAC_SIZE);
		if (len != EFUSE_MAC_SIZE)
			return 0;

		/* MAC is stored in ASCII format, 1bytes = 2characters */
		for (int i = 0; i < 6; i++) {
			tmp[0] = efuse_mac_addr[i * 2];
			tmp[1] = efuse_mac_addr[i * 2 + 1];
			tmp[2] = '\0';
			mac_addr[i] = simple_strtoul(tmp, NULL, 16);
		}

		if (is_valid_ethaddr(mac_addr))
			eth_env_set_enetaddr("ethaddr", mac_addr);
		else
			meson_generate_serial_ethaddr();
	}

	return 0;
}
