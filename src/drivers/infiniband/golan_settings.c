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
#include "mlx_utils/mlx_lib/mlx_blink_leds/mlx_blink_leds.h"
#include "golan.h"
#include "golan_settings.h"
#include "flexboot_nodnic.h"

static int shomron_wol_applies ( struct settings *settings ) {
	struct driver_settings *drv_settings =
			get_driver_settings_from_settings ( settings );
	struct flexboot_nodnic *flexboot_nodnic =
				( struct flexboot_nodnic * ) drv_settings->drv_priv;
	return ( flexboot_nodnic->wol_en ) ? APPLIES : DOES_NOT_APPLY;
}

static int shomron_blink_leds_restore ( struct settings *settings, const void *data,
                    size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct flexboot_nodnic *flexboot_nodnic = ( struct flexboot_nodnic * ) driver_settings->drv_priv;
	struct nv_port_conf *port_nv_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	int32 to_val = 0;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		/* Stored to generics to avoid saving it to flash */
		generic_settings_store ( settings, setting,
				&port_nv_conf->blink_leds, sizeof ( port_nv_conf->blink_leds ) );
		return STORED;
	}
	to_val = be32_to_cpu ( *( ( int32* ) data) );
	if (  to_val > MAX_BLINK_LEDS || to_val < MIN_BLINK_LEDS )
			return INVALID_INPUT;

	/* run command of blinking leds */
	mlx_blink_leds ( flexboot_nodnic->device_priv.utils, to_val );
	return SUCCESS;
}

static int shomron_virt_read ( struct driver_settings *settings ) {
	struct flexboot_nodnic *flexboot_nodnic = ( struct flexboot_nodnic * ) settings->drv_priv;
	struct nv_conf_defaults *defaults = ( struct nv_conf_defaults * ) settings->defaults;
	union mlx_nvconfig_virt_conf nv_virt_conf;
	union mlx_nvconfig_virt_caps nv_virt_caps;
	struct nv_conf *conf = ( struct nv_conf * ) settings->priv_data;
	struct driver_tlv_header tlv_hdr;
	int rc;

	memset ( &nv_virt_caps, 0, sizeof ( nv_virt_caps ) );
	memset ( &tlv_hdr, 0, sizeof ( tlv_hdr ) );
	tlv_hdr.type = GLOPAL_PCI_CAPS_TYPE;
	tlv_hdr.length = sizeof ( nv_virt_caps );
	tlv_hdr.data = &nv_virt_caps;
	if ( ( rc = settings->callbacks.tlv_read ( flexboot_nodnic, &tlv_hdr ) ) ) {
		DBGC ( flexboot_nodnic, "Failed to read global PCI capabilities\n" );
	}
	conf->virt_conf.sriov_support = nv_virt_caps.sriov_support;
	conf->max_num_of_vfs_supported = nv_virt_caps.max_vfs_per_pf;

	memset ( &nv_virt_conf, 0, sizeof ( nv_virt_conf ) );
	memset ( &tlv_hdr, 0, sizeof ( tlv_hdr ) );
	tlv_hdr.type = GLOPAL_PCI_SETTINGS_TYPE;
	tlv_hdr.length = sizeof ( nv_virt_conf );
	tlv_hdr.data = &nv_virt_conf;
	if ( ( rc = settings->callbacks.tlv_read ( flexboot_nodnic,
			& tlv_hdr ) != 0 ) || ! nv_virt_conf.sriov_valid ) {
		nv_virt_conf.num_of_vfs = defaults->total_vfs;
		nv_virt_conf.virt_mode = defaults->sriov_en;
	}
	conf->virt_conf.virt_mode = nv_virt_conf.virt_mode;
	conf->virt_conf.num_of_vfs = nv_virt_conf.num_of_vfs;

	return rc;
}

static int shomron_virt_nv_store ( struct driver_settings *driver_settings ) {
	struct flexboot_nodnic *flexboot_nodnic = ( struct flexboot_nodnic * ) driver_settings->drv_priv;
	union mlx_nvconfig_virt_conf nv_virt_conf;
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	char buf[255] = {0};
	uint32_t swabbed[3];
	int rc;

	memset ( & nv_virt_conf, 0, sizeof ( nv_virt_conf ) );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, &virt_mode_setting );
	conf->virt_conf.virt_mode = ( buf[0] == 'S' );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, &virt_num_setting );
	conf->virt_conf.num_of_vfs = strtoul ( buf, NULL, 10 );
	nv_virt_conf.sriov_valid = 1;
	nv_virt_conf.virt_mode = conf->virt_conf.virt_mode;
	nv_virt_conf.num_of_vfs = conf->virt_conf.num_of_vfs;
	swabbed[0] = cpu_to_be32 ( nv_virt_conf.dword[0] );
	swabbed[1] = cpu_to_be32 ( nv_virt_conf.dword[1] );
	swabbed[2] = cpu_to_be32 ( nv_virt_conf.dword[2] );

	if ( ( rc = driver_settings->callbacks.tlv_write ( flexboot_nodnic,
				swabbed, 0, GLOPAL_PCI_SETTINGS_TYPE, sizeof ( swabbed ) ) ) ) {
		return rc;
	}

	return rc;
}

static struct driver_setting_operation golan_setting_ops[] = {
	{ &virt_mode_setting,	NULL, NULL, &shomron_virt_nv_store, &shomron_virt_read },
	{ &virt_num_setting, 	NULL, NULL, &shomron_virt_nv_store, &shomron_virt_read },
	{ &blink_leds_setting,	NULL, &shomron_blink_leds_restore, NULL, NULL },
	{ &wol_setting,			&shomron_wol_applies, NULL, NULL, NULL },
};

void golan_update_setting_ops () {
	driver_setting_update_setting_ops ( golan_setting_ops,
		( sizeof ( golan_setting_ops ) / sizeof ( golan_setting_ops[0] ) ) );
}

/* this part should be general for a pci device and not specific pci device  */
static struct driver_device_name golan_device_names [] = {
	{ 0x1011, "ConnectIB HCA" },
	{ 0x1013, "ConnectX-4 HCA" },
	{ 0x1015, "ConnectX-4Lx HCA" },
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
