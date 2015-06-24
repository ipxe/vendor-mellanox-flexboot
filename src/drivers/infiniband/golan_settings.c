/*
 * Copyright (C) 2014-2015 Mellanox Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <ipxe/driver_settings.h>
#include <ipxe/in.h>
#include "golan.h"
#include "golan_settings.h"


static int golan_wol_applies ( struct settings *settings __unused ) {
	return DOES_NOT_APPLY;
}

static int golan_blink_leds_applies ( struct settings *settings __unused ) {
	return DOES_NOT_APPLY;
}

static int golan_vm_applies ( struct settings *settings __unused ) {
	return DOES_NOT_APPLY;
}

static struct driver_setting_operation golan_setting_ops[] = {
	{ &virt_mode_setting,	&golan_vm_applies, NULL, NULL },
	{ &blink_leds_setting,	&golan_blink_leds_applies, NULL, NULL },
	{ &wol_setting,			&golan_wol_applies, NULL, NULL },
};

void golan_update_setting_ops () {
	struct driver_setting_operation *golan_ops;
	struct driver_setting_operation *driver_ops;
	unsigned int i;

	for ( i = 0 ; i < ( sizeof ( golan_setting_ops ) /
		sizeof ( golan_setting_ops[0] ) ) ; i++ ) {
		golan_ops = &golan_setting_ops[i];
		driver_ops = find_setting_ops ( golan_ops->setting );
		if ( driver_ops ) {
			if ( golan_ops->applies )
				driver_ops->applies = golan_ops->applies;
			if ( golan_ops->store )
				driver_ops->store = golan_ops->store;
			if ( golan_ops->nv_store )
				driver_ops->nv_store = golan_ops->nv_store;
		}
	}
}

/* this part should be general for a pci device and not specific pci device  */
static struct driver_device_name golan_device_names [] = {
	{ 0x1011, "ConnectIB HCA" },
	{ 0x1013, "ConnectX4 HCA" },
};

char * golan_get_device_name ( struct golan *golan ) {
	unsigned int i;
	for ( i = 0 ; i < ( sizeof ( golan_device_names ) /
				sizeof ( golan_device_names[0] ) ) ; i++ ) {
		if ( golan->pci->id->device == golan_device_names[i].device_id )
			return golan_device_names[i].name;
	}
	return NULL;
}
