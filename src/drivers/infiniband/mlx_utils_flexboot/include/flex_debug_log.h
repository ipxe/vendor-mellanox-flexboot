#ifndef FLEX_DEBUG_LOG_H_
#define FLEX_DEBUG_LOG_H_
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
#include <mlx_nvconfig_prm.h>
#include <ipxe/driver_settings.h>

enum {
	DEBUG_LEVEL_DISABLE	= 0x0,
	DEBUG_LEVEL_BASIC	= 0x1,
	DEBUG_LEVEL_ADVANCED	= 0x2,
	DEBUG_LEVEL_ALL	= 0x3,
};

typedef enum {
	stp_debug_log =0,
	romprefix_debug_log,
	dhcp_debug_log,
	dhcpv6_debug_log,
	arp_debug_log,
	neighbor_debug_log,
	ndp_debug_log,
	uri_debug_log,
	driver_debug_log,
	nodnic_debug_log,
	nodnic_cmd_debug_log,
	nodnic_device_debug_log,
	nodnic_port_debug_log,
	netdevice_debug_log,
	tftp_debug_log,
	udp_debug_log,
	tcp_debug_log,
	tcpip_debug_log,
	ipv4_debug_log,
	ipv6_debug_log,
	drv_set_debug_log,
	stat_update_debug_log,
} debug_log_file;

/**
 * Print debugging message if we are at a certain debug level
 *
 * @v level			Debug level
 * @v debug_file	Debug log file index
 * @v ...			printf() argument list
 */
#define DBG_IF_PRIV( level, debug_file,... ) do {	\
	if ( get_debug_en ( debug_file, level ) ) {		\
		dbg_printf ( __VA_ARGS__ );					\
	}												\
} while ( 0 )

/**
 * Print a hex dump if we are at a certain debug level
 *
 * @v level			Debug level
 * @v debug_file	Debug log file index
 * @v dispaddr		Display address
 * @v data			Data to print
 * @v len			Length of data
 */
#define DBG_HDA_IF_PRIV( level, debug_file,dispaddr, data, len )  do {	\
	if ( get_debug_en ( debug_file, level ) ) {		\
		union {										\
			unsigned long ul;						\
			typeof ( dispaddr ) raw;				\
		} da;										\
		da.ul = 0;									\
		da.raw = dispaddr;							\
		dbg_hex_dump_da ( da.ul, data, len );		\
	}												\
} while ( 0 )

/**
 * Select colour for debug messages if we are at a certain debug level
 *
 * @v level			Debug level
 * @v debug_file	Debug log file index
 * @v id 			Message stream ID
 */
#define DBG_AC_IF_PRIV( level, debug_file, id ) do {	\
	if ( get_debug_en ( debug_file, level ) ) {			\
		union {											\
			unsigned long ul;							\
			typeof ( id ) raw;							\
		} dbg_stream;									\
		dbg_stream.ul = 0;								\
		dbg_stream.raw = id;							\
		dbg_autocolourise ( dbg_stream.ul );			\
	}													\
} while ( 0 )

/**
 * Revert colour for debug messages if we are at a certain debug level
 *
 * @v level			Debug level
 * @v debug_file	Debug log file index
 */
#define DBG_DC_IF_PRIV( level, debug_file ) do {	\
	if ( get_debug_en ( debug_file, level ) ) {		\
		dbg_decolourise();							\
	}												\
} while ( 0 )

/* Autocolourising versions of the DBGxxx_IF() macros */
#define DBGC_IF_PRIV( level, debug_file, id, ... ) do {			\
	DBG_AC_IF_PRIV ( level, debug_file, id );					\
	DBG_IF_PRIV ( level, debug_file, __VA_ARGS__ );				\
	DBG_DC_IF_PRIV ( level, debug_file );						\
} while ( 0 )

#define DBGC_HDA_IF_PRIV( level, debug_file, id, ... ) do {	\
	DBG_AC_IF_PRIV ( level, debug_file, id );				\
	DBG_HDA_IF_PRIV ( level, debug_file, __VA_ARGS__ );		\
	DBG_DC_IF_PRIV ( level, debug_file );					\
} while ( 0 )

/**
 * Print a hex dump if we are at a certain debug level
 *
 * @v level		Debug level
 * @v data		Data to print
 * @v len		Length of data
 */
#define DBGC_HD_IF_PRIV( level, debug_file, id, data, len ) do {			\
		const void *_data = data;			\
		DBGC_HDA_IF_PRIV ( level, debug_file, id, _data, _data, len );	\
	} while ( 0 )

#ifndef DEVICE_CX3
	#define DBGC_PRIV_IF( priv_level, level, debug_file, id, ...) do {	\
		if ( DBG_ ## level ) {							\
			DBGC ( id, __VA_ARGS__ );									\
		} else {														\
			DBGC_IF_PRIV ( priv_level, debug_file, id, __VA_ARGS__ );	\
		}																\
	}while(0)

	#define DBGCP_PRIV_IF( priv_level, level, debug_file, id, ...) do {	\
		if ( DBG_ ## level ) {							\
			DBGCP ( id, __VA_ARGS__ );									\
		} else {														\
			DBGC_IF_PRIV ( priv_level, debug_file, id, __VA_ARGS__ );	\
		}																\
	}while(0)

	#define DBGC2_PRIV_IF( priv_level, level, debug_file, id, ...) do {	\
		if ( DBG_ ## level ) {							\
			DBGC2 ( id, __VA_ARGS__ );									\
		} else {														\
			DBGC_IF_PRIV ( priv_level, debug_file, id, __VA_ARGS__ );	\
		}																\
	}while(0)

	#define DBG_PRIV_IF( priv_level,level, debug_file, ...) do {	\
		if ( DBG_ ## level ) {						\
			DBG ( __VA_ARGS__ );									\
		} else {													\
			DBG_IF_PRIV ( priv_level, debug_file, __VA_ARGS__ );	\
		}															\
	}while(0)

	#define DBG2_PRIV_IF( priv_level,level, debug_file, ...) do {	\
		if ( DBG_ ## level ) {						\
			DBG2 ( __VA_ARGS__ );									\
		} else {													\
			DBG_IF_PRIV ( priv_level, debug_file, __VA_ARGS__ );	\
		}															\
	}while(0)

	#define DBGP_PRIV_IF( priv_level,level, debug_file, ...) do {	\
		if ( DBG_ ## level ) {						\
			DBGP ( __VA_ARGS__ );									\
		} else {													\
			DBG_IF_PRIV ( priv_level, debug_file, __VA_ARGS__ );	\
		}															\
	}while(0)

	#define DBGC_PRIV_HDA_IF( priv_level,level, debug_file, id, ...) do {	\
		if ( DBG_ ## level ) {								\
			DBGC_HDA ( id, __VA_ARGS__ );									\
		} else {															\
			DBGC_HDA_IF_PRIV ( priv_level, debug_file, id, __VA_ARGS__ );	\
		}																	\
	}while(0)

	#define DBGC_PRIV_HD_IF( priv_level,level, debug_file, id, ...) do {	\
		if ( DBG_ ## level ) {								\
			DBGC_HD ( id, __VA_ARGS__ );									\
		} else {															\
			DBGC_HD_IF_PRIV ( priv_level, debug_file, id, __VA_ARGS__ );	\
		}																	\
	}while(0)

	#define DBGCP_PRIV_HDA_IF( priv_level,level, debug_file, id, ...) do {	\
		if ( DBG_ ## level ) {								\
			DBGCP_HDA ( id,__VA_ARGS__ );									\
		} else {															\
			DBGC_HDA_IF_PRIV ( priv_level, debug_file, id, __VA_ARGS__ );	\
		}																	\
	}while(0)

	#define DBGC2_PRIV_HDA_IF( priv_level,level, debug_file, id, ...) do {	\
		if ( DBG_ ## level ) {								\
			DBGC2_HDA ( id, __VA_ARGS__ );									\
		} else {															\
			DBGC_HDA_IF_PRIV ( priv_level, debug_file, id, __VA_ARGS__ );	\
		}																	\
	}while(0)

	#define DBGP_PRIV_HDA_IF( priv_level,level, debug_file, ...) do {	\
		if ( DBG_ ## level ) {							\
			DBGP_HDA ( __VA_ARGS__ );							\
		} else {														\
			DBG_HDA_IF_PRIV ( priv_level, debug_file, __VA_ARGS__ );	\
		}																\
	}while(0)
#else
	#define DBGC_PRIV_IF( priv_level, level, debug_file, id, ...) DBGC ( id, __VA_ARGS__ )
	#define DBGCP_PRIV_IF( priv_level, level, debug_file, id, ...) DBGCP ( id, __VA_ARGS__ )
	#define DBGC2_PRIV_IF( priv_level, level, debug_file, id, ...) DBGC2 ( id, __VA_ARGS__ )
	#define DBG_PRIV_IF( priv_level,level, debug_file, ...) DBG ( __VA_ARGS__ )
	#define DBG2_PRIV_IF( priv_level,level, debug_file, ...) DBG2 ( __VA_ARGS__ )
	#define DBGP_PRIV_IF( priv_level,level, debug_file, ...) DBGP ( __VA_ARGS__ )
	#define DBGC_PRIV_HDA_IF( priv_level,level, debug_file, id, ...) DBGC_HDA ( id, __VA_ARGS__ )
	#define DBGC_PRIV_HD_IF( priv_level,level, debug_file, id, ...) DBGC_HD ( id, __VA_ARGS__ )
	#define DBGCP_PRIV_HDA_IF( priv_level,level, debug_file, id, ...) DBGCP_HDA ( id,__VA_ARGS__ )
	#define DBGC2_PRIV_HDA_IF( priv_level,level, debug_file, id, ...) DBGC2_HDA ( id, __VA_ARGS__ )
	#define DBGP_PRIV_HDA_IF( priv_level,level, debug_file, ...) DBGP_HDA ( __VA_ARGS__ )
#endif

#define DBG_PRIV( debug_file, ... )	DBG_PRIV_IF ( DEBUG_LEVEL_BASIC, LOG, debug_file, ##__VA_ARGS__ )
#define DBGC_PRIV( debug_file, id, ... )	DBGC_PRIV_IF ( DEBUG_LEVEL_BASIC, LOG, debug_file, id, ##__VA_ARGS__ )
#define DBGC_HDA_PRIV( debug_file, id, ... )	DBGC_PRIV_HDA_IF ( DEBUG_LEVEL_BASIC, LOG, debug_file, id, ##__VA_ARGS__ )
#define DBGC_HD_PRIV( debug_file, id,... )		DBGC_PRIV_HD_IF	( DEBUG_LEVEL_BASIC,LOG, debug_file, id, ##__VA_ARGS__ )

#define DBG2_PRIV( debug_file, ... )	DBG2_PRIV_IF ( DEBUG_LEVEL_ADVANCED, EXTRA, debug_file, ##__VA_ARGS__ )
#define DBGC2_PRIV( debug_file, id, ... )	DBGC2_PRIV_IF ( DEBUG_LEVEL_ADVANCED, EXTRA,debug_file, id, ##__VA_ARGS__ )
#define DBGC2_HDA_PRIV( debug_file, id, ... )	DBGC2_PRIV_HDA_IF ( DEBUG_LEVEL_ADVANCED, EXTRA,debug_file, id, ##__VA_ARGS__ )

#define DBGP_PRIV( debug_file, id, ... )	DBGP_PRIV_IF ( DEBUG_LEVEL_ALL, PROFILE, debug_file, ##__VA_ARGS__ )
#define DBGP_HDA_PRIV( debug_file,... )		DBGP_PRIV_HDA_IF	( DEBUG_LEVEL_ALL, PROFILE, debug_file, ##__VA_ARGS__ )
#define DBGCP_PRIV( debug_file, id, ... )	DBGCP_PRIV_IF ( DEBUG_LEVEL_ALL, PROFILE, debug_file, id, ##__VA_ARGS__ )
#define DBGCP_HDA_PRIV( debug_file, id, ... )		DBGCP_PRIV_HDA_IF	( DEBUG_LEVEL_ALL, PROFILE, debug_file, id, ##__VA_ARGS__ )

/******* Private files Macros *******/
#define DBGC_STP(...) DBGC_PRIV ( stp_debug_log, __VA_ARGS__)
#define DBGC2_STP(...) DBGC2_PRIV ( stp_debug_log, __VA_ARGS__)
#define DBGC_HDA_STP(...) DBGC_HDA_PRIV ( stp_debug_log, __VA_ARGS__)

#define DBGC_DHCP(...) DBGC_PRIV ( dhcp_debug_log, __VA_ARGS__)
#define DBG_DHCP(...) DBG_PRIV ( dhcp_debug_log, __VA_ARGS__)

#define DBGC_DHCPV6(...) DBGC_PRIV ( dhcpv6_debug_log, __VA_ARGS__)

#define DBGC_ARP(...) DBGC_PRIV ( arp_debug_log, __VA_ARGS__)
#define DBGC2_ARP(...) DBGC2_PRIV ( arp_debug_log, __VA_ARGS__)

#define DBGC_NEIGHBOUR(...) DBGC_PRIV ( neighbor_debug_log, __VA_ARGS__)
#define DBGC2_NEIGHBOUR(...) DBGC2_PRIV ( neighbor_debug_log, __VA_ARGS__)

#define DBGC_NDP(...) DBGC_PRIV ( ndp_debug_log, __VA_ARGS__)

#define DBGC_URI(...) DBGC_PRIV ( uri_debug_log, __VA_ARGS__)

#define DBGC_HERMON(...) DBGC_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBG_HERMON(...) DBG_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBGC2_HERMON(...) DBGC2_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBGC2_HDA_HERMON(...) DBGC2_HDA_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBGC_HDA_HERMON(...) DBGC_HDA_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBGCP_HERMON(...) DBGCP_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBGCP_HDA_HERMON( ... ) DBGCP_HDA_PRIV ( driver_debug_log, __VA_ARGS__)

#define DBGC_GOLAN(...) DBGC_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBG_GOLAN(...) DBG_PRIV ( driver_debug_log, __VA_ARGS__)
#define DBGC2_GOLAN(...) DBGC2_PRIV ( driver_debug_log, __VA_ARGS__)

#define DBGC_NODNIC(...) DBGC_PRIV ( nodnic_debug_log, __VA_ARGS__)
#define DBGCP_NODNIC(...) DBGCP_PRIV ( nodnic_debug_log, __VA_ARGS__)
#define DBGC_HDA_NODNIC(...) DBGC_HDA_PRIV ( nodnic_debug_log, __VA_ARGS__)
#define DBG_NODNIC(...) DBG_PRIV ( nodnic_debug_log, __VA_ARGS__)
#define DBG2_NODNIC(...) DBG2_PRIV ( nodnic_debug_log, __VA_ARGS__)

#define DBGC_NET_DEV(...) DBGC_PRIV ( netdevice_debug_log, __VA_ARGS__)
#define DBGC2_NET_DEV(...) DBGC2_PRIV ( netdevice_debug_log, __VA_ARGS__)

#define DBGC_TFTP(...) DBGC_PRIV ( tftp_debug_log, __VA_ARGS__)
#define DBGC2_TFTP(...) DBGC2_PRIV ( tftp_debug_log, __VA_ARGS__)
#define DBGC_HD_TFTP(...) DBGC_HD_PRIV ( tftp_debug_log, __VA_ARGS__)

#define DBGC_UDP(...) DBGC_PRIV ( udp_debug_log, __VA_ARGS__)
#define DBG_UDP(...) DBG_PRIV ( udp_debug_log, __VA_ARGS__)
#define DBGC2_UDP(...) DBGC2_PRIV ( udp_debug_log, __VA_ARGS__)

#define DBGC_TCP(...) DBGC_PRIV ( tcp_debug_log, __VA_ARGS__)
#define DBG_TCP(...) DBG_PRIV ( tcp_debug_log, __VA_ARGS__)
#define DBGC2_TCP(...) DBGC2_PRIV ( tcp_debug_log, __VA_ARGS__)

#define DBG_TCPIP(...) DBG_PRIV ( tcpip_debug_log, __VA_ARGS__)

#define DBGC_IPV4(...) DBGC_PRIV ( ipv4_debug_log, __VA_ARGS__)
#define DBGC2_IPV4(...) DBGC2_PRIV ( ipv4_debug_log, __VA_ARGS__)

#define DBGC_IPV6(...) DBGC_PRIV ( ipv6_debug_log, __VA_ARGS__)
#define DBGC_HDA_IPV6(...) DBGC_HDA_PRIV ( ipv6_debug_log, __VA_ARGS__)
#define DBGC2_IPV6(...) DBGC2_PRIV ( ipv6_debug_log, __VA_ARGS__)
#define DBG_IPV6(...) DBG_PRIV ( ipv6_debug_log, __VA_ARGS__)

#define DBGC_DRV_SET(...) DBGC_PRIV ( drv_set_debug_log, __VA_ARGS__)
#define DBG_DRV_SET(...) DBG_PRIV ( drv_set_debug_log, __VA_ARGS__)

extern struct setting debug_en_setting __setting ( SETTING_FLEXBOOT, debug_en );
extern struct setting stp_dbg_setting __setting ( SETTING_FLEXBOOT, stp_dbg );
extern struct setting romprefix_dbg_setting __setting ( SETTING_FLEXBOOT, romprefix_dbg );
extern struct setting dhcp_dbg_setting __setting ( SETTING_FLEXBOOT, dhcp_dbg );
extern struct setting dhcpv6_dbg_setting __setting ( SETTING_FLEXBOOT, dhcpv6_dbg );
extern struct setting arp_dbg_setting __setting ( SETTING_FLEXBOOT, arp_dbg );
extern struct setting neighbor_dbg_setting __setting ( SETTING_FLEXBOOT, neighbor_dbg );
extern struct setting ndp_dbg_setting __setting ( SETTING_FLEXBOOT, ndp_dbg );
extern struct setting uri_dbg_setting __setting ( SETTING_FLEXBOOT, uri_dbg );
extern struct setting driver_dbg_setting __setting ( SETTING_FLEXBOOT, driver_dbg );
extern struct setting nodnic_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_dbg );
extern struct setting nodnic_cmd_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_cmd_dbg );
extern struct setting nodnic_device_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_device_dbg );
extern struct setting nodnic_port_dbg_setting __setting ( SETTING_FLEXBOOT, nodnic_port_dbg );
extern struct setting netdevice_dbg_setting __setting ( SETTING_FLEXBOOT, netdevice_dbg );
extern struct setting tftp_dbg_setting __setting ( SETTING_FLEXBOOT, tftp_dbg );
extern struct setting udp_dbg_setting __setting ( SETTING_FLEXBOOT, udp_dbg );
extern struct setting tcp_dbg_setting __setting ( SETTING_FLEXBOOT, tcp_dbg );
extern struct setting tcpip_dbg_setting __setting ( SETTING_FLEXBOOT, tcpip_dbg );
extern struct setting ipv4_dbg_setting __setting ( SETTING_FLEXBOOT, ipv4_dbg );
extern struct setting ipv6_dbg_setting __setting ( SETTING_FLEXBOOT, ipv6_dbg );
extern struct setting drv_set_dbg_setting __setting ( SETTING_FLEXBOOT, drv_set_dbg );
extern struct setting stat_update_dbg_setting __setting ( SETTING_FLEXBOOT, stat_update_dbg );

extern struct extended_setting ext_debug_en __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_stp_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_romprefix_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_dhcp_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_dhcpv6_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_arp_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_neighbor_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_ndp_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_uri_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_driver_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_nodnic_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_nodnic_cmd_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_nodnic_device_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_nodnic_port_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_netdevice_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_tftp_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_udp_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_tcp_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_tcpip_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_ipv4_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_ipv6_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_drv_set_dbg __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_stat_update_dbg __table_entry ( NV_CONFIG, 01 );

extern int init_debug_settings ( struct settings_operations *operations );
extern int debug_en_setting_nv_store ( struct driver_settings *driver_settings );
extern int stp_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int romprefix_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int dhcp_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int dhcpv6_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int arp_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int neighbor_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int ndp_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int uri_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int driver_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int nodnic_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int nodnic_cmd_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int nodnic_device_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int nodnic_port_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int netdevice_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int tftp_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int udp_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int tcp_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int tcpip_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int ipv4_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int ipv6_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int drv_set_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int stat_update_debug_setting_nv_store ( struct driver_settings *driver_settings );
extern int debug_setting_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting );
extern void set_debug_settings_ext_type ( void );
extern int driver_register_debug_nv_settings ( struct driver_settings *driver_settings );
extern int driver_flash_read_debug_settings ( struct driver_settings *driver_settings );
extern int get_debug_en ( uint32_t dbg_file_id, uint32_t dbg_level );
#endif
