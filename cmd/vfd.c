// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2002
 * Detlev Zundel, DENX Software Engineering, dzu@denx.de.
 */

/*
 * BMP handling routines
 */

#include <common.h>
#include <bmp_layout.h>
#include <command.h>
#include <dm.h>
#include <gzip.h>
#include <image.h>
#include <lcd.h>
#include <log.h>
#include <malloc.h>
#include <mapmem.h>
#include <splash.h>
#include <video.h>
#include <asm/byteorder.h>

extern int openvfd_set_title(const char *seg_str);
extern int openvfd_set_icon(const char *name, bool on);
extern void openvfd_clear(void);

static int do_vfd_clear(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	switch (argc) {
	case 1: /* use argument */
		openvfd_clear();
		break;
	default:
		return CMD_RET_USAGE;
	}

	return 0;
}

static int do_vfd_title(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	switch (argc) {
	case 2: /* use argument */
		openvfd_set_title(argv[1]);
		break;
	default:
		return CMD_RET_USAGE;
	}

	return 0;
}

static int do_vfd_icon(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	bool state;

	switch (argc) {
	case 3: /* use argument */
		if (!strncmp(argv[2], "on", 2))
			state = true;
		else
			state = false;
		openvfd_set_icon(argv[1], state);
		break;
	default:
		return CMD_RET_USAGE;
	}

	return 0;
}

static struct cmd_tbl cmd_bmp_sub[] = {
	U_BOOT_CMD_MKENT(clear, 1, 0, do_vfd_clear, "", ""),
	U_BOOT_CMD_MKENT(title, 2, 0, do_vfd_title, "", ""),
	U_BOOT_CMD_MKENT(icon, 3, 0, do_vfd_icon, "", ""),
};

/*
 * Subroutine:  do_vfd
 *
 * Description: Handler for 'vfd' command..
 *
 * Inputs:	argv[1] contains the subcommand
 *
 * Return:      None
 *
 */
static int do_vfd(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct cmd_tbl *c;

	/* Strip off leading 'bmp' command argument */
	argc--;
	argv++;

	c = find_cmd_tbl(argv[0], &cmd_bmp_sub[0], ARRAY_SIZE(cmd_bmp_sub));

	if (c)
		return  c->cmd(cmdtp, flag, argc, argv);
	else
		return CMD_RET_USAGE;
}

U_BOOT_CMD(
	vfd,	5,	1,	do_vfd,
	"set VFD display data",
	"clear\t\t\t- clear vfd display\n"
	"vfd title <string>\t\t- display string title\n"
	"vfd icon <label> on|off\t\t- display icon (b-t eth wifi spdif hdmi cvbs usb sd power apps setup alarm play pause)"
);
