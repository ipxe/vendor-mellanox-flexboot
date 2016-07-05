/*
 * Copyright (C) 2016 Mellanox Technologies Ltd.
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

#include <ipxe/driver_settings.h>
#include <stdlib.h>
#include <stdio.h>
#include <ipxe/in.h>
#include <errno.h>
#include <mlx_nvconfig_prm.h>
#include "flex_debug_log.h"

const struct settings_scope debug_scope;

struct setting debug_en_setting __setting ( SETTING_FLEXBOOT, debug_en ) = {
	.name = "debug_en",
	.description = "Debug enable",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_debug_en __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &debug_en_setting,
	.type = OPTION,
};

struct setting stp_dbg_setting __setting ( SETTING_FLEXBOOT, stp_dbg ) = {
	.name = "stp_dbg",
	.description = "STP debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_stp_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &stp_dbg_setting,
	.type = OPTION,
};

struct setting romprefix_dbg_setting __setting ( SETTING_FLEXBOOT, romprefix_dbg ) = {
	.name = "romprefix_dbg",
	.description = "Romprefix debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 1,
};

struct extended_setting ext_romprefix_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &romprefix_dbg_setting,
	.type = OPTION,
};

struct setting dhcp_dbg_setting __setting ( SETTING_FLEXBOOT, dhcp_dbg ) = {
	.name = "dhcp_dbg",
	.description = "Dhcp debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_dhcp_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &dhcp_dbg_setting,
	.type = OPTION,
};

struct setting dhcpv6_dbg_setting __setting ( SETTING_FLEXBOOT, dhcpv6_dbg ) = {
	.name = "dhcpv6_dbg",
	.description = "Dhcpv6 debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_dhcpv6_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &dhcpv6_dbg_setting,
	.type = OPTION,
};

struct setting arp_dbg_setting __setting ( SETTING_FLEXBOOT, arp_dbg ) = {
	.name = "arp_dbg",
	.description = "ARP debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_arp_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &arp_dbg_setting,
	.type = OPTION,
};

struct setting neighbor_dbg_setting __setting ( SETTING_FLEXBOOT, neighbor_dbg ) = {
	.name = "neighbor_dbg",
	.description = "Neighbor debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_neighbor_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &neighbor_dbg_setting,
	.type = OPTION,
};

struct setting ndp_dbg_setting __setting ( SETTING_FLEXBOOT, ndp_dbg ) = {
	.name = "ndp_dbg",
	.description = "NDP debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_ndp_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &ndp_dbg_setting,
	.type = OPTION,
};

struct setting uri_dbg_setting __setting ( SETTING_FLEXBOOT, uri_dbg ) = {
	.name = "uri_dbg",
	.description = "URI debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_uri_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &uri_dbg_setting,
	.type = OPTION,
};

struct setting driver_dbg_setting __setting ( SETTING_FLEXBOOT, driver_dbg ) = {
	.name = "driver_dbg",
	.description = "Driver debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_driver_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &driver_dbg_setting,
	.type = OPTION,
};

struct setting nodnic_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_dbg ) = {
	.name = "nodnic_dbg",
	.description = "Nodnic debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_nodnic_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &nodnic_dbg_setting,
	.type = OPTION,
};

struct setting nodnic_cmd_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_cmd_dbg ) = {
	.name = "nodnic_cmd_dbg",
	.description = "Nodnic cmd debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 1,
};

struct extended_setting ext_nodnic_cmd_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &nodnic_cmd_dbg_setting,
	.type = OPTION,
};

struct setting nodnic_device_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_device_dbg ) = {
	.name = "nodnic_device_dbg",
	.description = "Nodnic device debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 1,
};

struct extended_setting ext_nodnic_device_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &nodnic_device_dbg_setting,
	.type = OPTION,
};

struct setting nodnic_port_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_port_dbg ) = {
	.name = "nodnic_port_dbg",
	.description = "Nodnic port debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 1,
};

struct extended_setting ext_nodnic_port_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &nodnic_port_dbg_setting,
	.type = OPTION,
};

struct setting netdevice_dbg_setting __setting ( SETTING_FLEXBOOT, netdevice_dbg ) = {
	.name = "netdevice_dbg",
	.description = "Netdevice debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_netdevice_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &netdevice_dbg_setting,
	.type = OPTION,
};

struct setting tftp_dbg_setting __setting ( SETTING_FLEXBOOT, tftp_dbg ) = {
	.name = "tftp_dbg",
	.description = "TFTP debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_tftp_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &tftp_dbg_setting,
	.type = OPTION,
};

struct setting udp_dbg_setting __setting ( SETTING_FLEXBOOT, udp_dbg ) = {
	.name = "udp_dbg",
	.description = "UDP debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_udp_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &udp_dbg_setting,
	.type = OPTION,
};

struct setting tcp_dbg_setting __setting ( SETTING_FLEXBOOT, tcp_dbg ) = {
	.name = "tcp_dbg",
	.description = "TCP debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_tcp_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &tcp_dbg_setting,
	.type = OPTION,
};

struct setting tcpip_dbg_setting __setting ( SETTING_FLEXBOOT, tcpip_dbg ) = {
	.name = "tcpip_dbg",
	.description = "TCP/IP debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_tcpip_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &tcpip_dbg_setting,
	.type = OPTION,
};

struct setting ipv4_dbg_setting __setting ( SETTING_FLEXBOOT, ipv4_dbg ) = {
	.name = "ipv4_dbg",
	.description = "IPv4 debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_ipv4_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &ipv4_dbg_setting,
	.type = OPTION,
};

struct setting ipv6_dbg_setting __setting ( SETTING_FLEXBOOT, ipv6_dbg ) = {
	.name = "ipv6_dbg",
	.description = "IPv6 debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_ipv6_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &ipv6_dbg_setting,
	.type = OPTION,
};

struct setting drv_set_dbg_setting __setting ( SETTING_FLEXBOOT, drv_set_dbg ) = {
	.name = "drv_set_dbg",
	.description = "Driver settings debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 0,
};

struct extended_setting ext_drv_set_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &drv_set_dbg_setting,
	.type = OPTION,
};

struct setting stat_update_dbg_setting __setting ( SETTING_FLEXBOOT, stat_update_dbg ) = {
	.name = "stat_update_dbg",
	.description = "Status update debug level",
	.type = &setting_type_string,
	.scope = &debug_scope,
	.hidden = 1,
};

struct extended_setting ext_stat_update_dbg __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &stat_update_dbg_setting,
	.type = OPTION,
};

struct debug_file_data_entry{
	debug_log_file	dbg_file;
	mlx_uint32	offset;
	mlx_uint8	align;
	mlx_uint32	mask;
};

#define DebugDataEntry( _dbg_file, _offset, _align, _mask) { \
	.dbg_file = _dbg_file,\
	.offset = _offset,\
	.align = _align,\
	.mask = _mask,\
}

#define BIT_MASK_A_TO_B( a, b ) ( ( ( unsigned int ) - 1 >> ( 31 -  ( b ) ) ) & ~ ( ( 1U << ( a ) ) - 1 ) )

static struct debug_file_data_entry debug_file_data_table[] = {
	DebugDataEntry( stp_debug_log, 0x1, 0, BIT_MASK_A_TO_B ( 0, 1 ) ),
	DebugDataEntry( romprefix_debug_log, 0x1, 2, BIT_MASK_A_TO_B ( 2, 3 ) ),
	DebugDataEntry( dhcp_debug_log, 0x1, 4, BIT_MASK_A_TO_B ( 4, 5 ) ),
	DebugDataEntry( dhcpv6_debug_log, 0x1, 6, BIT_MASK_A_TO_B(6, 7 ) ),
	DebugDataEntry( arp_debug_log, 0x1, 8, BIT_MASK_A_TO_B ( 8, 9 ) ),
	DebugDataEntry( neighbor_debug_log, 0x1, 10, BIT_MASK_A_TO_B ( 10, 11 ) ),
	DebugDataEntry( ndp_debug_log, 0x1, 12, BIT_MASK_A_TO_B ( 12, 13 ) ),
	DebugDataEntry( uri_debug_log, 0x1, 14, BIT_MASK_A_TO_B ( 14, 15 ) ),
	DebugDataEntry( driver_debug_log, 0x1, 16, BIT_MASK_A_TO_B ( 16, 17 ) ),
	DebugDataEntry( nodnic_debug_log, 0x1, 18, BIT_MASK_A_TO_B ( 18, 19 ) ),
	DebugDataEntry( nodnic_cmd_debug_log, 0x1, 20, BIT_MASK_A_TO_B ( 20 , 21 ) ),
	DebugDataEntry( nodnic_device_debug_log, 0x1, 22, BIT_MASK_A_TO_B (22, 23 ) ),
	DebugDataEntry( nodnic_port_debug_log, 0x1, 24, BIT_MASK_A_TO_B ( 24, 25 ) ),
	DebugDataEntry( netdevice_debug_log, 0x1, 26, BIT_MASK_A_TO_B ( 26, 27 ) ),
	DebugDataEntry( tftp_debug_log, 0x1, 28, BIT_MASK_A_TO_B ( 28, 29 ) ),
	DebugDataEntry( udp_debug_log, 0x1, 30, BIT_MASK_A_TO_B ( 30, 31 ) ),
	DebugDataEntry( tcp_debug_log, 0x2, 0, BIT_MASK_A_TO_B ( 0, 1 ) ),
	DebugDataEntry( tcpip_debug_log, 0x2, 2, BIT_MASK_A_TO_B ( 2, 3 ) ),
	DebugDataEntry( ipv4_debug_log, 0x2, 4, BIT_MASK_A_TO_B ( 4, 5 ) ),
	DebugDataEntry( ipv6_debug_log, 0x2, 6, BIT_MASK_A_TO_B ( 6, 7 ) ),
	DebugDataEntry( drv_set_debug_log, 0x2, 8, BIT_MASK_A_TO_B ( 8, 9 ) ),
	DebugDataEntry( stat_update_debug_log, 0x2, 10, BIT_MASK_A_TO_B ( 10, 11 ) ),
};

static union mlx_nvconfig_debug_conf priv_debug_conf = {.dword[0] = 0,.dword[1] = 0,.dword[2] = 0 };

static void update_nv_debug_field ( union mlx_nvconfig_debug_conf *dbg_conf,
			debug_log_file dbg_file_id,int new_val ) {
	struct debug_file_data_entry *dbg_entry = & debug_file_data_table [ dbg_file_id ];
	mlx_uint32 * dword = & dbg_conf->dword [ dbg_entry->offset ];
	*dword = ( *dword & ( ~ ( dbg_entry->mask ) ) ) | ( ( ( new_val & 0x3 ) << dbg_entry->align ) );
}

#ifndef DEVICE_CX3
int init_debug_settings ( struct settings_operations *operations ) {
	struct generic_settings *root_settings;
	struct generic_settings *debug_settings;

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return -EINVAL;
	}

	root_settings = nv_settings_root;
	debug_settings = zalloc ( sizeof ( struct generic_settings ) );
	if ( debug_settings == NULL ) {
		printf ( "%s: Failed to allocate memory for debug settings\n", __FUNCTION__ );
		return -ENOMEM;
	}

	generic_settings_init ( debug_settings, NULL);
	register_settings ( &debug_settings->settings, &root_settings->settings, "Diagnostics" );
	debug_settings->settings.op = operations;
	debug_settings->settings.default_scope = &debug_scope;

	return 0;
}
#else
int init_debug_settings ( struct settings_operations *operations __unused ) {
	return 0;
}
#endif

static int debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	union mlx_nvconfig_debug_conf *dbg_conf = & ( conf->debug_conf );
	union mlx_nvconfig_debug_conf swapped;
	int rc;

	swapped.dword[0] = cpu_to_be32 ( dbg_conf->dword[0] );
	swapped.dword[1] = cpu_to_be32 ( dbg_conf->dword[1] );
	swapped.dword[2] = cpu_to_be32 ( dbg_conf->dword[2] );
	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
			&swapped, driver_settings->index,
			NV_ROM_DEBUG_LEVEL, sizeof ( swapped ) ) ) ) {
			return -EACCES;
	}

	return SUCCESS;
}

int debug_en_setting_nv_store ( struct driver_settings *driver_settings ) {
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	union mlx_nvconfig_debug_conf *dbg_conf = & ( conf->debug_conf );
	char buf[255] = {0};
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &debug_en_setting );
	dbg_conf->dbg_log_en = ( buf[0] == 'E' );

	return debug_setting_nv_store ( driver_settings );
}

static int fetch_dbg_setting_value ( struct driver_settings *driver_settings,
								struct setting *dbg_setting ) {
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	char buf[255] = {0};
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, dbg_setting );
	switch ( buf[0] ) {
		case 'D' : return DEBUG_LEVEL_DISABLE; break;
		case 'B' : return DEBUG_LEVEL_BASIC; break;
		case 'A' :
			if ( buf[1] == 'd') {
			return DEBUG_LEVEL_ADVANCED;
			} else {
			return DEBUG_LEVEL_ALL;
			}
			break;
	}

	return DEBUG_LEVEL_DISABLE;
}

static int file_debug_setting_nv_store ( struct driver_settings *driver_settings,
							struct setting *setting, debug_log_file file_id ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	union mlx_nvconfig_debug_conf *dbg_conf = & ( conf->debug_conf );
	update_nv_debug_field ( dbg_conf, file_id,
			fetch_dbg_setting_value ( driver_settings, setting ) );
	return debug_setting_nv_store ( driver_settings );
}

int stp_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &stp_dbg_setting,
			stp_debug_log ) ;
}

int romprefix_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &romprefix_dbg_setting,
			romprefix_debug_log ) ;
}

int dhcp_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &dhcp_dbg_setting,
			dhcp_debug_log ) ;
}

int dhcpv6_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &dhcpv6_dbg_setting,
			dhcpv6_debug_log ) ;
}

int arp_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &arp_dbg_setting,
			arp_debug_log ) ;
}

int neighbor_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &neighbor_dbg_setting,
			neighbor_debug_log ) ;
}

int ndp_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &ndp_dbg_setting,
			ndp_debug_log ) ;
}

int uri_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &uri_dbg_setting,
			uri_debug_log ) ;
}

int driver_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &driver_dbg_setting,
			driver_debug_log ) ;
}

int nodnic_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &nodnic_dbg_setting,
			nodnic_debug_log ) ;
}

int nodnic_cmd_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &nodnic_cmd_dbg_setting,
			nodnic_cmd_debug_log ) ;
}

int nodnic_device_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &nodnic_device_dbg_setting,
			nodnic_device_debug_log ) ;
}

int nodnic_port_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &nodnic_port_dbg_setting,
			nodnic_port_debug_log ) ;
}

int netdevice_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &netdevice_dbg_setting,
			netdevice_debug_log ) ;
}

int tftp_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &tftp_dbg_setting,
			tftp_debug_log ) ;
}

int udp_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &udp_dbg_setting,
			udp_debug_log ) ;
}

int tcp_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &tcp_dbg_setting,
			tcp_debug_log ) ;
}

int tcpip_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &tcpip_dbg_setting,
			tcpip_debug_log ) ;
}

int ipv4_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &ipv4_dbg_setting,
			ipv4_debug_log ) ;
}

int ipv6_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &ipv6_dbg_setting,
			ipv6_debug_log ) ;
}

int drv_set_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &drv_set_dbg_setting,
			drv_set_debug_log ) ;
}

int stat_update_debug_setting_nv_store ( struct driver_settings *driver_settings ) {
	return file_debug_setting_nv_store ( driver_settings, &stat_update_dbg_setting,
			stat_update_debug_log ) ;
}

int debug_setting_restore ( struct settings *settings, const void *data,
				size_t len, const struct setting *setting ) {
	/* If setting is deleted, set default */
	if ( !data || !len ) {
			generic_settings_store ( settings, setting,
					STR_DISABLE, strlen ( STR_DISABLE ) + 1 );
		return STORED;
	}

	return SUCCESS;
}

static int get_ext_dbg_type ( struct settings *settings ) {
	char buf[MAX_SETTING_STR] = { 0 };
	int rc;

	rc = fetchf_setting ( settings, &debug_en_setting, NULL, NULL, buf, MAX_SETTING_STR );
	if ( ( rc > 0 ) && ( buf[0] == 'D' ) )
		return LABEL;

	return OPTION;
}

void set_debug_settings_ext_type ( void ) {
	ext_stp_dbg.get_extended_type = &get_ext_dbg_type;
	ext_romprefix_dbg.get_extended_type = &get_ext_dbg_type;
	ext_dhcp_dbg.get_extended_type = &get_ext_dbg_type;
	ext_dhcpv6_dbg.get_extended_type = &get_ext_dbg_type;
	ext_arp_dbg.get_extended_type = &get_ext_dbg_type;
	ext_neighbor_dbg.get_extended_type = &get_ext_dbg_type;
	ext_ndp_dbg.get_extended_type = &get_ext_dbg_type;
	ext_uri_dbg.get_extended_type = &get_ext_dbg_type;
	ext_driver_dbg.get_extended_type = &get_ext_dbg_type;
	ext_nodnic_dbg.get_extended_type = &get_ext_dbg_type;
	ext_nodnic_cmd_dbg.get_extended_type = &get_ext_dbg_type;
	ext_nodnic_device_dbg.get_extended_type = &get_ext_dbg_type;
	ext_nodnic_port_dbg.get_extended_type = &get_ext_dbg_type;
	ext_netdevice_dbg.get_extended_type = &get_ext_dbg_type;
	ext_tftp_dbg.get_extended_type = &get_ext_dbg_type;
	ext_udp_dbg.get_extended_type = &get_ext_dbg_type;
	ext_tcp_dbg.get_extended_type = &get_ext_dbg_type;
	ext_tcpip_dbg.get_extended_type = &get_ext_dbg_type;
	ext_ipv4_dbg.get_extended_type = &get_ext_dbg_type;
	ext_ipv6_dbg.get_extended_type = &get_ext_dbg_type;
	ext_drv_set_dbg.get_extended_type = &get_ext_dbg_type;
	ext_stat_update_dbg.get_extended_type = &get_ext_dbg_type;
}

static int driver_set_debug_en_setting ( struct driver_settings *driver_settings ){
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	union mlx_nvconfig_debug_conf *dbg_conf = & ( conf->debug_conf );
	struct settings *main_settings;
	struct settings *settings;
	char buf[256];
	int rc = 0;

	main_settings = & ( driver_settings->generic_settings.settings );
	rc = fetch_setting_origin ( main_settings, &debug_en_setting, &settings );
	if ( rc || ( !rc && !settings ) ) {
			DBGC ( driver_settings, "Failed to find the debug settings block\n" );
			return rc;
	}

	memset ( buf, 0, sizeof ( buf ) );
	DRIVER_STORE_STR_SETTING( driver_settings, settings, debug_en_setting,
						( dbg_conf->dbg_log_en ? STR_ENABLE : STR_DISABLE ) );

	return SUCCESS;
}

static int driver_set_debug_settings ( struct driver_settings *driver_settings,
			struct setting *setting, uint32_t dbg_lvl ) {
	struct settings *main_settings;
	struct settings *settings;
	char buf[256];
	int rc = 0;

	main_settings = & ( driver_settings->generic_settings.settings );
	rc = fetch_setting_origin ( main_settings, &debug_en_setting, &settings );
	if ( rc || ( !rc && !settings ) ) {
	DBGC ( driver_settings, "Failed to find the debug settings block\n" );
	return rc;
	}

	memset ( buf, 0, sizeof ( buf ) );
	switch ( dbg_lvl ) {
		case DEBUG_LEVEL_DISABLE: snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_DISABLE );break;
		case DEBUG_LEVEL_BASIC: snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_BASIC );break;
		case DEBUG_LEVEL_ADVANCED: snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_ADVANCED );break;
		case DEBUG_LEVEL_ALL: snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_ALL );break;
		default: snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_DISABLE );break;
	}

	DRIVER_STORE_SETTING( driver_settings, settings,
			(*setting), buf );

	return SUCCESS;
}

static int driver_set_debug_lvl_settings ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	union mlx_nvconfig_debug_conf *dbg_conf = & ( conf->debug_conf );
	int rc;

	if ( ( rc = driver_set_debug_settings ( driver_settings, &stp_dbg_setting,
			dbg_conf->stp_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register stp debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &romprefix_dbg_setting,
			dbg_conf->romprefix_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register romprefix debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &dhcp_dbg_setting,
			dbg_conf->dhcp_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register dhcp debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &dhcpv6_dbg_setting,
			dbg_conf->dhcpv6_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register dhcpv6 debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &arp_dbg_setting,
			dbg_conf->arp_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register arp debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &neighbor_dbg_setting,
			dbg_conf->neighbor_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register neighbor debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &ndp_dbg_setting,
			dbg_conf->ndp_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register ndp debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &uri_dbg_setting,
			dbg_conf->uri_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register uri debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &driver_dbg_setting,
			dbg_conf->driver_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register driver debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &nodnic_dbg_setting,
			dbg_conf->nodnic_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register nodnic debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &nodnic_cmd_dbg_setting,
			dbg_conf->nodnic_cmd_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register nodnic cmd debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &nodnic_device_dbg_setting,
			dbg_conf->nodnic_device_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register nodnic device debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &nodnic_port_dbg_setting,
			dbg_conf->nodnic_port_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register nodnic port debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &netdevice_dbg_setting,
			dbg_conf->netdevice_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register netdevice debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &tftp_dbg_setting,
			dbg_conf->tftp_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register tftp debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &udp_dbg_setting,
			dbg_conf->udp_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register udp debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &tcp_dbg_setting,
			dbg_conf->tcp_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register tcp debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &tcpip_dbg_setting,
			dbg_conf->tcpip_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register tcpip debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &ipv4_dbg_setting,
			dbg_conf->ipv4_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register ipv4 debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &ipv6_dbg_setting,
			dbg_conf->ipv6_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register ipv6 debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &drv_set_dbg_setting,
			dbg_conf->drv_set_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register driver settings debug settings\n" );
	}
	if ( ( rc = driver_set_debug_settings ( driver_settings, &stat_update_dbg_setting,
			dbg_conf->stat_update_dbg_lvl ) ) ) {
		DBGC ( driver_settings, "Failed to register status update debug settings\n" );
	}

	return 0;
}

int driver_register_debug_nv_settings ( struct driver_settings *driver_settings ) {
	int rc = 0;

	/* debug settings */
	if ( ( rc = driver_set_debug_en_setting ( driver_settings ) ) )
			DBGC ( driver_settings, "Failed to set debug enable setting\n" );
	rc = driver_set_debug_lvl_settings ( driver_settings );

	return rc;
}

int driver_flash_read_debug_settings ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	union mlx_nvconfig_debug_conf *dbg_conf = & ( conf->debug_conf );
	int rc = 0;

	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			NV_ROM_DEBUG_LEVEL, 0, sizeof ( *dbg_conf ), 0, dbg_conf) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read the debug configuration from the flash (rc = %d)\n", rc );
		/* default set all to zero (disbaled) */
		memset( & conf->debug_conf,0,sizeof ( *dbg_conf ) );
	}
	memcpy ( &priv_debug_conf, dbg_conf, sizeof ( union mlx_nvconfig_debug_conf ) );

	return rc;
}

int get_debug_en ( uint32_t dbg_file_id, uint32_t dbg_level ) {
	struct debug_file_data_entry *dbg_entry = &debug_file_data_table[ dbg_file_id ];
	uint32_t cur_dbg_lvl;

	if ( ! priv_debug_conf.dbg_log_en )
		return 0;

	cur_dbg_lvl = ( ( priv_debug_conf.dword[dbg_entry->offset] & dbg_entry->mask ) >>  dbg_entry->align ) ;
	if ( ! cur_dbg_lvl || cur_dbg_lvl < dbg_level ) {
		return 0;
	}

	return 1;
}
