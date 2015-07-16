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

#include <ipxe/settings.h>
#include <ipxe/tables.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <ipxe/timer.h>
#include <ipxe/netdevice.h>
#include <ipxe/driver_settings.h>
#include <ipxe/in.h>
#include <ipxe/infiniband.h>
#include <ipxe/vlan.h>
#include <ipxe/iscsi.h>
#include "hermon.h"
#include "hermon_settings.h"

static int hermon_wol_applies ( struct settings *settings ) {
	struct driver_settings* driver_settings = get_driver_settings_from_settings ( settings );
	struct hermon *hermon = ( struct hermon * ) driver_settings->drv_priv;

	if ( ( driver_settings->index == 1 ) && hermon->cap.nv_config_flags.nv_config_wol_port1 )
		return APPLIES;

	if ( ( driver_settings->index == 2 ) && hermon->cap.nv_config_flags.nv_config_wol_port2 )
		return APPLIES;

	return DOES_NOT_APPLY;
}

static int hermon_blink_leds_applies ( struct settings *settings ) {
	struct driver_settings* driver_settings = get_driver_settings_from_settings ( settings );
	struct hermon *hermon = ( struct hermon * ) driver_settings->drv_priv;

	return hermon->cap.port_beacon;
}

static int hermon_vm_applies ( struct settings *settings ) {
	struct driver_settings* driver_settings = get_driver_settings_from_settings ( settings );
	struct hermon *hermon = ( struct hermon * ) driver_settings->drv_priv;

	return hermon->cap.nv_config_flags.nv_config_sriov_en;
}

static int hermon_virt_nv_store ( struct driver_settings *driver_settings ) {
	struct hermon *hermon = ( struct hermon * ) driver_settings->drv_priv;
	union hermon_nv_virt_conf nv_virt_conf;
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	char buf[255] = {0};
	uint32_t dword;
	int rc;

	memset ( &nv_virt_conf, 0, sizeof ( nv_virt_conf ) );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, & virt_mode_setting );
	conf->virt_conf.virt_mode = ( buf[0] == 'S' );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, & virt_num_setting );
	conf->virt_conf.num_of_vfs = strtoul ( buf, NULL, 10 );
	nv_virt_conf.virt_mode = conf->virt_conf.virt_mode;
	nv_virt_conf.num_of_vfs = conf->virt_conf.num_of_vfs;
	dword = cpu_to_be32 ( nv_virt_conf.dword );

	if ( ( rc = driver_settings->callbacks.tlv_write ( hermon,
				& dword, 0, VIRTUALIZATION_TYPE, sizeof ( dword ) ) ) ) {
		return -EACCES;
	}

	return rc;
}

static int hermon_virt_read ( struct driver_settings *settings ) {
	struct hermon *hermon = ( struct hermon * ) settings->drv_priv;
	struct nv_conf_defaults *defaults = ( struct nv_conf_defaults * ) settings->defaults;
	union hermon_nv_virt_conf nv_virt_conf;
	struct nv_conf *conf = ( struct nv_conf * ) settings->priv_data;
	struct driver_tlv_header tlv_hdr;
	int rc;

	memset ( &nv_virt_conf, 0, sizeof ( nv_virt_conf ) );
	memset ( &tlv_hdr, 0, sizeof ( tlv_hdr ) );
	tlv_hdr.type = VIRTUALIZATION_TYPE;
	tlv_hdr.length = sizeof ( nv_virt_conf );
	tlv_hdr.data = &nv_virt_conf;

	if ( ( rc = settings->callbacks.tlv_read ( hermon,
			& tlv_hdr ) != 0 ) ) {
		nv_virt_conf.num_of_vfs = defaults->total_vfs;
		nv_virt_conf.virt_mode = defaults->sriov_en;
	}

	conf->virt_conf.sriov_valid = 1;
	conf->virt_conf.virt_mode = nv_virt_conf.virt_mode;
	conf->virt_conf.num_of_vfs = nv_virt_conf.num_of_vfs;

	return rc;
}

static int hermon_nv_write_blink_leds ( struct hermon *hermon,
			struct hermon_port *port, int duration ) {
	struct nv_port_conf *conf = & ( port->port_nv_conf );
	union hermonprm_set_port set_port;
	int rc = 0;

	conf->blink_leds = duration;
	/* Set blink LEDs if supported */
	if ( hermon->cap.port_beacon ) {
		memset ( &set_port, 0, sizeof ( set_port ) );
		MLX_FILL_1 ( &set_port.beacon, 0, duration, conf->blink_leds );
		MLX_FILL_1 ( &set_port.beacon, 0, version, 0 );
		if ( ( rc = hermon_cmd_set_port ( hermon, port->ibdev->port,
			&set_port, 4 ) ) != 0 ) {
			DBGC ( hermon, "ConnectX3 %p port %d could not set port: %s\n",
				hermon, port->ibdev->port, strerror ( rc ) );
		}
	}
	return rc;
}

static int blink_leds_store ( struct settings *settings, const void *data,
			  size_t len, const  struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct hermon *hermon = ( struct hermon * ) driver_settings->drv_priv;
	struct hermon_port *port;
	int32_t input;
	int rc, port_num;

	if ( !data  || ( ntohl ( * ( int32_t * ) data ) == 0 ) )
		return SUCCESS;

	input = ntohl ( * ( int32_t * ) data );

	if ( ( input > MAX_BLINK_LEDS ) || ( input < 0 ) )
		return INVALID_INPUT;

	port_num = driver_settings->index - 1;
	if ( ( port_num < 0 ) || ( port_num > ( int ) ( hermon->num_ports ) ) )
		return INVALID_INPUT;

	port = & ( hermon->port[ port_num ] );
	generic_settings_store ( settings, setting, data, len );

	if ( ( rc = hermon_open ( hermon ) ) != 0 ) {
		printf ( " Couldn't open device (rc = %d) \n", rc );
		return INVALID_INPUT;
	}

	if ( ( rc =  hermon_nv_write_blink_leds ( hermon, port, input ) ) != 0 ) {
		DBGC ( hermon, "Failed to store blink leds (rc = %d)\n", rc );
	}

	hermon_close ( hermon );

	return STORED;
}

static struct driver_setting_operation hermon_setting_ops[] = {
	{ &virt_mode_setting, &hermon_vm_applies, NULL, &hermon_virt_nv_store, &hermon_virt_read },
	{ &virt_num_setting, NULL, NULL, &hermon_virt_nv_store, &hermon_virt_read },
	{ &blink_leds_setting,	&hermon_blink_leds_applies, &blink_leds_store, NULL, NULL },
	{ &wol_setting,			&hermon_wol_applies, NULL, NULL, NULL },
};

void hermon_update_setting_ops () {
	driver_setting_update_setting_ops ( hermon_setting_ops,
		( sizeof ( hermon_setting_ops ) / sizeof ( hermon_setting_ops[0] ) ) );
}

static int hermon_open_wrapper ( void *priv ) {
	return hermon_open ( priv );
}

static void hermon_close_wrapper ( void *priv ) {
	return hermon_close ( priv );
}

int hermon_init_port_settings ( struct hermon *hermon, struct hermon_port *port ) {
	struct driver_settings *driver_settings = & ( port->driver_settings );

	/* TODO: Need to update also after NODNIC IS STARTED */
	driver_settings->callbacks.open_dev			= hermon_open_wrapper;
	driver_settings->callbacks.close_dev		= hermon_close_wrapper;
	driver_settings->callbacks.set_default		= NULL;
	if ( hermon->cap.nv_mem_access_supported )
		driver_settings->callbacks.tlv_write	= hermon_flash_write_tlv;
	driver_settings->callbacks.tlv_read			= hermon_flash_read_tlv_wrapper;
	driver_settings->callbacks.tlv_invalidate	= hermon_flash_invalidate_tlv;
	driver_settings->netdev						= port->netdev;
	driver_settings->index						= port->ibdev->port;
	driver_settings->drv_priv					= hermon;
	driver_settings->priv_data					= & ( port->port_nv_conf );
	driver_settings->defaults					= & ( port->defaults );

	return driver_settings_init_port ( driver_settings );
}

int hermon_init_settings ( struct hermon *hermon ) {
	struct driver_settings *driver_settings = & ( hermon->hermon_settings );
	struct generic_settings *gen_settings = & ( driver_settings->generic_settings );

	gen_settings->settings.op = driver_settings_get_operations ();
	gen_settings->settings.default_scope = &main_scope;
	nv_settings_root = gen_settings;

	driver_settings->callbacks.open_dev			= hermon_open_wrapper;
	driver_settings->callbacks.close_dev		= hermon_close_wrapper;
	driver_settings->callbacks.set_default		= NULL;
	if ( hermon->cap.nv_mem_access_supported )
		driver_settings->callbacks.tlv_write	= hermon_flash_write_tlv;
	driver_settings->callbacks.tlv_read			= hermon_flash_read_tlv_wrapper;
	driver_settings->callbacks.tlv_invalidate	= hermon_flash_invalidate_tlv;
	driver_settings->callbacks.set_ro_device_settings = hermon_get_ro_pci_settings;
	driver_settings->netdev						= NULL;
	driver_settings->index						= 0;
	driver_settings->drv_priv					= hermon;
	driver_settings->priv_data					= & ( hermon->hermon_nv_conf );
	driver_settings->defaults					= & ( hermon->defaults );

	hermon_update_setting_ops ();

	return driver_settings_init ( driver_settings );
}

struct driver_device_name hermon_device_names [] = {
	{ 0x1003, "ConnectX-3 HCA" },
	{ 0x1007, "ConnectX-3Pro HCA" },
};

char * get_device_name ( struct hermon *hermon ) {
	unsigned int i;
	for ( i = 0 ; i < ( sizeof ( hermon_device_names ) /
				sizeof ( hermon_device_names[0] ) ) ; i++ ) {
		if ( hermon->pci->id->device == hermon_device_names[i].device_id )
			return hermon_device_names[i].name;
	}
	return NULL;
}
