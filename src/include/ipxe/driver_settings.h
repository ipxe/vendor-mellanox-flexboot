#ifndef DRIVER_SETTINGS_H_
#define DRIVER_SETTINGS_H_

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
#include <ipxe/netdevice.h>
#include <ipxe/if_ether.h>

#define NV_CONFIG __table( struct extended_setting, "nv_config")

#define STR_ENABLE		"Enabled"
#define STR_DISABLE		"Disabled"
#define STR_ISCSI		"iSCSI"
#define STR_PXE			"PXE"
#define STR_NONE		"None"
#define STR_SRIOV		"SR-IOV"
#define STR_IPV4		"IPv4"
#define STR_ONE_TIME_DISABLED "One time disabled"

#define APPLIES 1
#define DOES_NOT_APPLY 0

#define SUCCESS		0
#define INVALID_INPUT	-1
#define STORED		1

#define MAX_ISCSI_NAME 223
#define MIN_ISCSI_NAME 0
#define MAX_CHAP_ID 128
#define MIN_CHAP_ID 0
#define MAX_CHAP_SECRET 16
#define MIN_CHAP_SECRET 12
#define MAX_TCP_PORT 65535
#define MIN_TCP_PORT 1
#define MIN_BOOT_LUN 0
#define MAX_BOOT_LUN 255
#define MAX_BLINK_LEDS 15
#define MAX_VIRTUAL_ID 4094
#define MIN_VIRTUAL_ID 1
#define MAX_ROOT_PATH_LEN 0xfd
#define MAX_STR_SETTING_BUF_SIZE 256
#define MAX_BANNER_TO 14
#define MIN_BANNER_TO 1
#define MIN_PKEY_VAL 0
#define MAX_PKEY_VAL 65535

/*
 * iPXE-specific definitions
 *
 */

#define IP_STRING_LEN	16

/*
 * Maximum length for domain name (DHCP option).
 * Its limited due to the iPXE code that checks if the option length is
 * not larger than 0xff (including the option header - 2 bytes)
 *
 */
#define DOMAIN_MAX_LEN 0xfd

enum {
	WAKE_ON_LAN_TYPE				= 0x10,
	VIRTUALIZATION_TYPE				= 0x11,
	VPI_LINK_TYPE					= 0x12,
	BANNER_TO_TYPE 					= 0x2010,
	FLOW_CONTROL_TYPE				= 0x2020,
	BOOT_SETTINGS_TYPE				= 0x2021,
	ISCSI_GENERAL_SETTINGS_TYPE		= 0x2100,
	IB_BOOT_SETTING_TYPE			= 0x2022,

	// Types for iSCSI strings
	DHCP_VEND_ID					= 0x2101,
	ISCSI_INITIATOR_IPV4_ADDR		= 0x2102,
	ISCSI_INITIATOR_SUBNET			= 0x2103,
	ISCSI_INITIATOR_IPV4_GATEWAY	= 0x2104,
	ISCSI_INITIATOR_IPV4_PRIM_DNS	= 0x2105,
	ISCSI_INITIATOR_IPV4_SECDNS		= 0x2106,
	ISCSI_INITIATOR_NAME			= 0x2107,
	ISCSI_INITIATOR_CHAP_ID			= 0x2108,
	ISCSI_INITIATOR_CHAP_PWD		= 0x2109,
	ISCSI_INITIATOR_DHCP_CONF_TYPE	= 0x210a,

	CONNECT_FIRST_TGT				= 0x2200,
	FIRST_TGT_IP_ADDRESS			= 0x2201,
	FIRST_TGT_TCP_PORT				= 0x2202,
	FIRST_TGT_BOOT_LUN				= 0x2203,
	FIRST_TGT_ISCSI_NAME			= 0x2204,
	FIRST_TGT_CHAP_ID				= 0x2205,
	FIRST_TGT_CHAP_PWD				= 0x2207,

	CONNECT_SECOND_TGT				= 0x2300,
	SECOND_TGT_IP_ADDRESS			= 0x2301,
	SECOND_TGT_TCP_PORT				= 0x2302,
	SECOND_TGT_BOOT_LUN				= 0x2303,
	SECOND_TGT_ISCSI_NAME			= 0x2304,
	SECOND_TGT_CHAP_ID				= 0x2305,
	SECOND_TGT_CHAP_PWD				= 0x2307
};

#define DEF_LINK_TYPE_IB	1
#define DEF_LINK_TYPE_ETH	2
#define DEF_LINK_TYPE_VPI	3

struct nv_port_conf_defaults {
	uint8_t pptx;
	uint8_t pprx;
	uint8_t boot_option_rom_en;
	uint8_t boot_vlan_en;
	uint8_t boot_retry_count;
	uint8_t boot_protocol;
	uint16_t boot_vlan;
	uint16_t boot_pkey;
	uint8_t en_wol_magic;
	uint8_t network_link_type;
	uint8_t iscsi_boot_to_target;
	uint8_t iscsi_vlan_en;
	uint8_t iscsi_tcp_timestamps_en;
	uint8_t iscsi_chap_mutual_auth_en;
	uint8_t iscsi_chap_auth_en;
	uint8_t iscsi_dhcp_params_en;
	uint8_t iscsi_ipv4_dhcp_en;
	uint8_t iscsi_lun_busy_retry_count;
	uint8_t iscsi_link_up_delay_time;
};

struct nv_conf_defaults {
	uint8_t max_vfs;
	uint8_t total_vfs;
	uint8_t sriov_en;
	uint8_t maximum_uar_bar_size;
	uint8_t uar_bar_size;
	uint8_t flexboot_menu_to;
};

struct nv_conf_ini {
	char dhcp_user_class[8];
	uint8_t uri_boot_retry_delay;
	uint8_t uri_boot_retry;
	uint8_t option_rom_debug;
	uint8_t promiscuous_vlan;
};

#define ISCSI_INITIATOR_IP_ADDR_MASK	0b00000001
#define ISCSI_INITIATOR_SUBNET_MASK		0b00000010
#define ISCSI_INITIATOR_GATEWAY_MASK	0b00000100
#define ISCSI_INITIATOR_PRIME_DNS_MASK	0b00001000
#define ISCSI_INITIATOR_NAME_MASK		0b00010000
#define ISCSI_INITIATOR_CHAP_ID_MASK	0b00100000
#define ISCSI_INITIATOR_CHAP_PASS_MASK	0b01000000
#define ISCSI_TARGET_CONNECT_STR_LEN	9
#define ISCSI_IPV4_ADDR_STR_LEN			16
#define ISCSI_INITIATOR_NAME_STR_LEN	224
#define ISCSI_CHAP_ID_STR_LEN			129
#define ISCSI_CHAP_PASS_STR_LEN			17

#define DRIVER_STORE_SETTING( id, settings, setting, data )				\
	do {																\
		if ( ( rc = storef_setting ( settings, &setting, data ) ) ) 	\
			DBGC ( id, "Failed to store %s\n", setting.name );			\
	} while ( 0 )

#define DRIVER_ISCSI_STORE_IP_TO_SETTING( id, setting, is_valid, mask, data )	\
	do {																		\
		if ( is_valid & mask ) {												\
			memset ( &ip_addr, 0, sizeof ( ip_addr ) );							\
			/* Don't store IP setting if the string is zeros */					\
			inet_aton ( data, &ip_addr );										\
			if ( ip_addr.s_addr != 0 ) {										\
				if ( ( rc = store_setting ( settings, &setting, 				\
						&ip_addr, sizeof ( ip_addr ) ) ) ) {					\
					DBGC ( id, "Failed to store %s "							\
					"to netdevice settings\n", setting.name );					\
				}																\
			}																	\
		}																		\
	} while (0)

#define DRIVER_MAX_STR_LEN_SETTING	255

#define DRIVER_STORE_STR_SETTING( id, settings, setting, data )			\
	do {																\
		memset ( buf, 0, sizeof ( buf ) );								\
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", data );		\
		DRIVER_STORE_SETTING( id, settings, setting, buf );				\
	} while ( 0 )

#define DRIVER_STORE_INT_SETTING( id, settings, setting, data )			\
	do {																\
		memset ( buf, 0, sizeof ( buf ) );								\
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%d", data );		\
		DRIVER_STORE_SETTING( id, settings, setting, buf );				\
	} while ( 0 )


#define DRIVER_ISCSI_STORE_STR_TO_SETTING( id, setting, is_valid, mask, data )	\
	do {																		\
		if ( is_valid & mask ) {												\
			DRIVER_STORE_SETTING( id, settings, setting, data );				\
		}																		\
	} while (0)

#define DRIVER_STORE_INT_SETTING_EN( id, is_enabled, settings, setting )	\
	do {																	\
		if ( ( rc = storen_setting ( settings, setting, is_enabled ) ) ) {	\
			DBGC ( id, "Failed to set %s with value %d\n",					\
				( setting )->name, is_enabled );							\
		} else {															\
			DBGC ( id, "Successfully stored %s with value %d\n",			\
				( setting )->name, is_enabled );							\
		}																	\
	} while ( 0 )

struct nv_iscsi_initiator_params {
	char ipv4_addr[ISCSI_IPV4_ADDR_STR_LEN];
	char subnet[ISCSI_IPV4_ADDR_STR_LEN];
	char gateway[ISCSI_IPV4_ADDR_STR_LEN];
	char primary_dns[ISCSI_IPV4_ADDR_STR_LEN];
	char name[ISCSI_INITIATOR_NAME_STR_LEN];
	char chap_id[ISCSI_CHAP_ID_STR_LEN];
	char chap_pass[ISCSI_CHAP_PASS_STR_LEN];
	uint8_t valid_mask;
};

#define ISCSI_TARGET_CONNECT_MASK		0b00000001
#define ISCSI_TARGET_IP_ADDR_MASK		0b00000010
#define ISCSI_TARGET_TCP_PORT_MASK		0b00000100
#define ISCSI_TARGET_LUN_MASK			0b00001000
#define ISCSI_TARGET_NAME_MASK			0b00010000
#define ISCSI_TARGET_CHAP_ID_MASK		0b00100000
#define ISCSI_TARGET_CHAP_PASS_MASK		0b01000000
#define ISCSI_TARGET_TCP_PORT_STR_LEN	6
#define ISCSI_TARGET_LUN_STR_LEN		4
#define ISCSI_TARGET_NAME_STR_LEN		224
#define ISCSI_TARGET_CHAP_ID_STR_LEN	129
#define ISCSI_TARGET_CHAP_PASS_STR_LEN	17

struct nv_iscsi_target_params {
	char connect[ISCSI_TARGET_CONNECT_STR_LEN];
	char ip_addr[ISCSI_IPV4_ADDR_STR_LEN];
	char tcp_port[ISCSI_TARGET_TCP_PORT_STR_LEN];
	char lun[ISCSI_TARGET_LUN_STR_LEN];
	char name[ISCSI_TARGET_NAME_STR_LEN];
	char chap_id[ISCSI_TARGET_CHAP_ID_STR_LEN];
	char chap_pass[ISCSI_TARGET_CHAP_PASS_STR_LEN];
	uint8_t ip_ver;
	uint8_t valid_mask;
};

union nv_iscsi_general {
	struct {
		uint32_t	reserved0			:22;
		uint32_t	boot_to_target		:2;
		uint32_t	reserved1			:2;
		uint32_t	vlan_en				:1;
		uint32_t	tcp_timestamps_en	:1;
		uint32_t	chap_mutual_auth_en	:1;
		uint32_t	chap_auth_en		:1;
		uint32_t	reserved2			:2;
		/*-------------------*/
		uint32_t	vlan				:12;
		uint32_t	reserved3			:20;
		/*-------------------*/
		uint32_t	lun_busy_retry_count:8;
		uint32_t	link_up_delay_time	:8;
		uint32_t	reserved4			:16;
	};
	uint32_t dword[3];
};

union nv_iscsi_init_dhcp_conf {
	struct {
		uint32_t reserved0		:30;
		uint32_t dhcp_iscsi_en	:1;
		uint32_t ipv4_dhcp_en	:1;

	};
	uint32_t dword;
};

union nv_nic_boot_conf {
	struct {
		uint32_t	vlan_id				: 12;
		uint32_t	link_speed			: 4;
		uint32_t	legacy_boot_prot	: 8;
		uint32_t	boot_retry_count	: 3;
		uint32_t	boot_strap_type		: 3;
		uint32_t	en_vlan				: 1;
		uint32_t	en_option_rom		: 1;
	};
	uint32_t dword;
};

union nv_nic_ib_boot_conf {
	struct {
		uint32_t	boot_pkey			: 16;
		uint32_t	reserved0			: 16;
	};
	uint32_t dword;
};

union nv_wol_conf {
	struct {
		uint32_t	reserved0		:9;
		uint32_t	en_wol_passwd	:1;
		uint32_t	en_wol_magic	:1;
		uint32_t	reserved1		:21;
		uint32_t	reserved2		:32;
	};
	uint32_t dword[2];
};

struct nv_iscsi_conf {
	union nv_iscsi_init_dhcp_conf		init_dhcp_conf;
	union nv_iscsi_general				gen_conf;
	struct nv_iscsi_initiator_params	initiator_params;
	struct nv_iscsi_target_params		first_tgt_params;
	/** Port default configurations */
	struct nv_port_conf_defaults defaults;
};

struct nv_nic_conf {
	union nv_nic_boot_conf		boot_conf;
	union nv_wol_conf			wol_conf;
	union nv_nic_ib_boot_conf	ib_boot_conf;
};

union nv_rom_banner_timeout_conf {
	struct {
		uint32_t	rom_banner_to	: 4;
		uint32_t	reserved		: 28;
	};
	uint32_t dword;
};

union nv_virt_conf {
	struct {
		uint32_t num_of_vfs	: 16;
		uint32_t rsrvd		: 15;
		uint32_t virt_mode	: 1;
	};
	uint32_t dword;
};

#define FW_VER_STR_LEN			12
#define FLEXBOOT_VER_STR_LEN	12
struct firmware_image_props {
	char family_fw_version[FW_VER_STR_LEN];
	char flexboot_version[FLEXBOOT_VER_STR_LEN];
};

#define DEVICE_NAME_STR_LEN		129
#define MAC_ADDRESS_STR_LEN		18
struct nv_conf {
	struct firmware_image_props fw_image_props;
	union nv_virt_conf virt_conf;
	union nv_rom_banner_timeout_conf rom_banner_to;
	char device_name[DEVICE_NAME_STR_LEN];
	char virtual_mac[MAC_ADDRESS_STR_LEN];
	char bdf_name[32]; /* busdevfn string name */
	unsigned int desc_dev_id;
	const char *driver_name;
	uint8_t max_num_of_vfs_supported;
};

struct nv_port_conf {
	struct nv_nic_conf		nic;
	struct nv_iscsi_conf	iscsi;
	uint8_t					blink_leds;
	/** Factory MAC (physical MAC) */
	uint8_t phys_mac[ETH_ALEN];
};


enum {
	BOOT_PROTOCOL_NONE	= 0,
	BOOT_PROTOCOL_PXE	= 1,
	BOOT_PROTOCOL_ISCSI	= 2,
	BOOT_PROTOCOL_FCOE	= 3,
};

enum {
	ISCSI_BOOT_TO_TARGET_ENABLE	= 0,
	ISCSI_BOOT_TO_TARGET_DISABLE	= 1,
	ISCSI_BOOT_TO_TARGET_ONE_TIME_DISABLE	= 2,
};

struct options_list {
	char **options_array;
	unsigned int options_num;
	unsigned int current_index;
};

enum menu_setting_type { LABEL, INPUT, OPTION };

struct driver_settings;

struct driver_tlv_header {
	uint32_t	type;
	uint32_t	type_mod;
	uint32_t	length;
	uint32_t	version;
	void 		*data;
};

typedef int ( * open_device_fn ) ( void *drv_priv );
typedef void ( * close_device_fn ) ( void *drv_priv );
typedef void ( * set_default_fn ) ( struct driver_settings *driver_settings );
typedef int ( * tlv_write_fn ) ( void *drv_priv, void *data,
		uint32_t tlv_mod, uint32_t tlv_type, uint32_t data_len );
typedef int ( * tlv_read_fn ) ( void *drv_priv,
		struct driver_tlv_header *tlv_hdr );

typedef int ( * tlv_invaidate_fn ) ( void *drv_priv, uint32_t tlv_mod,
		uint32_t tlv_type );

typedef void ( * set_ro_device_fn ) ( void *drv_priv );

struct driver_settings_callbacks {
	open_device_fn		open_dev;
	close_device_fn		close_dev;
	set_default_fn		set_default;
	tlv_write_fn		tlv_write;
	tlv_read_fn			tlv_read;
	tlv_invaidate_fn	tlv_invalidate;
	set_ro_device_fn 	set_ro_device_settings;
};

struct driver_settings {
	struct generic_settings generic_settings;
	struct driver_settings_callbacks callbacks;
	struct net_device *netdev;
	void *drv_priv;
	void *priv_data;
	void *defaults;
	struct list_head modified_list;
	uint8_t store_menu;
	struct list_head list;
	int index;
};

struct driver_setting_operation {
	const struct setting *setting;
	int ( *applies ) ( struct settings *settings );
	/** Store setting (or NULL if not supported)
	 *
	 * @v netdev		Network device
	 * @v data		Setting data, or NULL to clear setting
	 * @v len		Length of setting data
	 * @ret rc		Return status code
	 */
	int ( * store ) ( struct settings *settings, const void *data,
			  size_t len, const struct setting *setting );
	/* Store setting to flash ( or NULL if not supported ) */
	int ( * nv_store ) ( struct driver_settings *settings );
};

struct extended_setting {
	struct setting *setting;
	enum menu_setting_type type;
	void *data;
	char *instructions;
	struct list_head modified;

	int ( * get_extended_type ) ( struct settings *settings );
	int ( *nv_store ) ( struct driver_settings *settings );
};

struct driver_device_name {
	/** Device ID */
	uint16_t device_id;
	/** Device name */
	char* name;
};

extern struct generic_settings *nv_settings_root;
extern struct list_head driver_settings_list;

extern const struct settings_scope main_scope;
extern const struct settings_scope port_scope;
extern const struct settings_scope fw_scope;
extern const struct settings_scope nic_scope;
extern const struct settings_scope iscsi_scope;
extern const struct settings_scope iscsi_general_scope;
extern const struct settings_scope iscsi_init_scope;
extern const struct settings_scope iscsi_target_scope;

extern struct setting virt_mode_setting __setting( SETTING_HERMON, virt_mode );
extern struct setting virt_num_setting __setting( SETTING_HERMON, virt_num );
extern struct setting virt_num_max_setting __setting( SETTING_HERMON, virt_num_max );
extern struct setting blink_leds_setting __setting ( SETTING_HERMON, blink_leds );
extern struct setting device_name_setting __setting ( SETTING_HERMON, device_name );
extern struct setting chip_type_setting __setting ( SETTING_HERMON, chip_type );
extern struct setting pci_id_setting __setting ( SETTING_HERMON, pci_id );
extern struct setting bus_dev_fun_setting __setting ( SETTING_HERMON, bus_dev_fun );
extern struct setting mac_add_setting __setting ( SETTING_HERMON, mac_add );
extern struct setting phy_mac_setting __setting ( SETTING_HERMON, virt_mac );
extern struct setting flex_version_setting __setting( SETTING_HERMON, flex_version );
extern struct setting fw_version_setting __setting( SETTING_HERMON, fw_version );
extern struct setting boot_protocol_setting __setting( SETTING_HERMON, boot_protocol );
extern struct setting virt_lan_setting __setting( SETTING_HERMON, virt_lan );
extern struct setting virt_id_setting __setting( SETTING_HERMON, virt_id );
extern struct setting link_speed_setting __setting( SETTING_HERMON, link_speed );
extern struct setting opt_rom_setting __setting( SETTING_HERMON, opt_rom );
extern struct setting boot_retries_setting __setting( SETTING_HERMON, boot_retries );
extern struct setting boot_strap_setting __setting( SETTING_HERMON, boot_strap );
extern struct setting wol_setting __setting( SETTING_HERMON, wol );
extern struct setting dhcp_ip_setting __setting( SETTING_HERMON, dhcp_ip );
extern struct setting dhcp_iscsi_setting __setting( SETTING_HERMON, dhcp_iscsi );
extern struct setting iscsi_chap_setting __setting( SETTING_HERMON, iscsi_chap );
extern struct setting iscsi_mutual_chap_setting __setting( SETTING_HERMON, iscsi_mutual_chap );
extern struct setting ip_ver_setting __setting( SETTING_HERMON, ip_ver );
extern struct setting ipv4_add_setting __setting( SETTING_HERMON, ipv4_add );
extern struct setting subnet_mask_setting __setting( SETTING_HERMON, subnet_mask );
extern struct setting ipv4_gateway_setting __setting( SETTING_HERMON, ipv4_gateway );
extern struct setting ipv4_dns_setting __setting( SETTING_HERMON, ipv4_dns );
extern struct setting iscsi_init_name_setting __setting( SETTING_HERMON, iscsi_name );
extern struct setting init_chapid_setting __setting( SETTING_HERMON, init_chapid );
extern struct setting init_chapsec_setting __setting( SETTING_HERMON, init_chapsec );
extern struct setting connect_setting __setting( SETTING_HERMON, connect );
extern struct setting target_ip_setting __setting( SETTING_HERMON, target_ip );
extern struct setting tcp_port_setting __setting( SETTING_HERMON, tcp_port );
extern struct setting boot_lun_setting __setting( SETTING_HERMON, boot_lun );
extern struct setting iscsi_target_name_setting __setting( SETTING_HERMON, iscsi_target_name );
extern struct setting target_chapid_setting __setting( SETTING_HERMON, target_chapid );
extern struct setting target_chapsec_setting __setting( SETTING_HERMON, target_chapsec );
extern struct setting flexboot_menu_to_setting __setting ( SETTING_HERMON, flexboot_menu_to );

extern struct extended_setting ext_virt_mode __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_virt_num __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_virt_num_max __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_blink_leds __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_device_name __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_chip_type __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_pci_id __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_bus_dev_fun __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_mac __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_virt_mac __table_entry (NV_CONFIG, 01) ;
extern struct extended_setting ext_flex_version __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_fw_version __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_boot_protocol __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_virt_lan __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_virt_id __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_virt_id __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_opt_rom __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_boot_retries __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_boot_strap __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_wol __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_dhcp_ip __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_dhcp_iscsi __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_iscsi_chap __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_iscsi_mutual_chap __table_entry ( NV_CONFIG, 01 );
extern struct extended_setting ext_ip_ver __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_ipv4_add __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_subnet_mask __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_ipv4_gateway __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_ipv4_dns __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_iscsi_init_name __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_init_chapid __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_init_chapsec __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_connect __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_target_ip __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_tcp_port __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_boot_lun __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_iscsi_target_name __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_target_chapid __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_target_chapsec __table_entry (NV_CONFIG, 01);
extern struct extended_setting ext_flexboot_menu_to_setting __table_entry ( NV_CONFIG, 01 );

struct settings * driver_settings_from_netdev ( struct net_device *netdev );
int driver_settings_get_boot_prot_val ( struct settings *settings );
int driver_settings_get_boot_ret_val ( struct settings *settings );
const char * driver_settings_get_boot_ret_str ( unsigned int value );
struct extended_setting* find_extended ( const struct setting *setting );
int driver_settings_init ( struct driver_settings *driver_settings );
int driver_settings_init_port ( struct driver_settings *driver_settings );
int driver_settings_nv_store ();
int root_path_store ( struct settings *netdev_settings, struct settings *driver_settings );
void destroy_driver_settings ();
int driver_set_device_settings ( struct driver_settings *driver_settings );
struct driver_settings* get_driver_settings_from_settings ( struct settings *settings );
int init_firmware_settings ( struct settings_operations *operations );
struct driver_setting_operation * find_setting_ops ( const struct setting *setting );
struct settings_operations * driver_settings_get_operations ();
int driver_register_port_nv_settings( struct driver_settings *driver_settings );
int driver_register_nv_settings ( struct driver_settings *driver_settings );
int driver_settings_get_port_nvdata ( struct driver_settings *driver_settings,
		unsigned int port_num );
int driver_settings_get_nvdata ( struct driver_settings *driver_settings );
void copy_netdev_settings_to_netdev ( struct net_device *src, struct net_device *dest );
int driver_settings_get_nv_boot_en ( void *priv_data, unsigned int port,
		u8* boot_enable, tlv_read_fn read_tlv_fn,
		struct nv_port_conf_defaults  *defaults );
int driver_settings_get_tlv_version ( uint32_t tlv_type );
int driver_settings_get_iscsi_boot_to_target_val ( struct settings *settings );
int driver_settings_get_ib_pkey_val ( struct settings *settings );
#endif /*DRIVER_SETTINGS_H_*/
