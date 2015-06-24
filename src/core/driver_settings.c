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

#include <ipxe/driver_settings.h>
#include <ipxe/boot_menu_ui.h>
#include <ipxe/iscsi.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <byteswap.h>
#include <ipxe/in.h>
#include <errno.h>
#include <config/general.h>

struct settings_page {
	struct generic_settings **new_settings;
	struct generic_settings **parent;
	char *name;
	const struct settings_scope *scope;
};

struct extended_options {
	struct extended_setting *ext;
	int size;
	char *array [8];
};

struct extended_description {
	struct extended_setting *ext;
	char description[258];
};


/*************************** NV_ROOT - root for all driver settings ***************/

struct generic_settings* nv_settings_root = NULL;
struct list_head driver_settings_list = LIST_HEAD_INIT ( driver_settings_list );

/* All menu's scopes, used for defining which settings appear in each page */
const struct settings_scope main_scope;
const struct settings_scope port_scope;
const struct settings_scope fw_scope;
const struct settings_scope nic_scope;
const struct settings_scope iscsi_scope;
const struct settings_scope iscsi_general_scope;
const struct settings_scope iscsi_init_scope;
const struct settings_scope iscsi_target_scope;

/********************* All settings for menu ***********************/

struct setting virt_mode_setting __setting( SETTING_FLEXBOOT, virt_mode ) = {
	.name = "virt_mode",
	.description = "Virtualization mode",
	.type = &setting_type_string,
	.scope = &main_scope,
};

struct extended_setting ext_virt_mode __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &virt_mode_setting,
	.type = OPTION,
};

struct setting virt_num_setting __setting( SETTING_FLEXBOOT, virt_num ) = {
	.name = "virt_num",
	.description = "Number of virtual functions",
	.type = &setting_type_int32,
	.scope = &main_scope,
};

struct extended_setting ext_virt_num __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &virt_num_setting,
	.type = INPUT,
};

struct setting virt_num_max_setting __setting( SETTING_FLEXBOOT, virt_num_max ) = {
	.name = "virt_num_max",
	.description = "Max number of VFs supported",
	.type = &setting_type_int32,
	.scope = &main_scope,
	.hidden = 1,
};

struct extended_setting ext_virt_num_max __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &virt_num_max_setting,
	.type = LABEL,
};

struct setting blink_leds_setting __setting ( SETTING_FLEXBOOT, blink_leds ) = {
	.name = "blink_leds",
	.description = "Blink leds",
	.type = &setting_type_int32,
	.scope = &port_scope,
};

struct extended_setting ext_blink_leds __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &blink_leds_setting,
	.type = INPUT,
};

struct setting device_name_setting __setting ( SETTING_FLEXBOOT, device_name ) = {
	.name = "device_name",
	.description = "Device name",
	.type = &setting_type_string,
	.scope = &main_scope,
};

struct extended_setting ext_device_name __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &device_name_setting,
	.type = LABEL,
};

struct setting chip_type_setting __setting ( SETTING_FLEXBOOT, chip_type ) = {
	.name = "chip_type",
	.description = "Chip type",
	.type = &setting_type_string,
	.scope = &main_scope,
};

struct extended_setting ext_chip_type __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &chip_type_setting,
	.type = LABEL,
};


struct setting flexboot_menu_to_setting __setting ( SETTING_FLEXBOOT, chip_type ) = {
	.name = "flexboot_menu_to",
	.description = "Banner menu timeout",
	.type = &setting_type_int8,
	.scope = &main_scope,
};

struct extended_setting ext_flexboot_menu_to_setting __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &flexboot_menu_to_setting,
	.type = INPUT,
};

struct setting pci_id_setting __setting ( SETTING_FLEXBOOT, pci_id ) = {
	.name = "pci_id",
	.description = "PCI device ID",
	.type = &setting_type_int32,
	.scope = &main_scope,
};

struct extended_setting ext_pci_id __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &pci_id_setting,
	.type = LABEL,
};

struct setting bus_dev_fun_setting __setting ( SETTING_FLEXBOOT, bus_dev_fun ) = {
	.name = "bus_dev_fun",
	.description = "Bus:Device:Function",
	.type = &setting_type_string,
	.scope = &main_scope,
};

struct extended_setting ext_bus_dev_fun __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &bus_dev_fun_setting,
	.type = LABEL,
};

struct setting mac_add_setting __setting ( SETTING_FLEXBOOT, mac_add ) = {
	.name = "mac_add",
	.description = "Virtual MAC Address",
	.type = &setting_type_string,
	.scope = &port_scope,
};

struct extended_setting ext_mac_add __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &mac_add_setting,
	.type = LABEL,
};

struct setting phy_mac_setting __setting ( SETTING_FLEXBOOT, phy_mac ) = {
	.name = "phy_mac",
	.description = "Physical MAC Address",
	.type = &setting_type_string,
	.scope = &port_scope,
	.hidden = 0,
};

struct extended_setting ext_physical_mac __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &phy_mac_setting,
	.type = LABEL,
};

struct setting flex_version_setting __setting( SETTING_FLEXBOOT, flex_version ) = {
	.name = "flex_version",
	.description = "Flexboot version",
	.type = &setting_type_string,
	.scope = &fw_scope,
};

struct extended_setting ext_flex_version __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &flex_version_setting,
	.type = LABEL,
};

struct setting fw_version_setting __setting( SETTING_FLEXBOOT, fw_version ) = {
	.name = "fw_version",
	.description = "Family firmware version",
	.type = &setting_type_string,
	.scope = &fw_scope,
};

struct extended_setting ext_fw_version __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &fw_version_setting,
	.type = LABEL,
};

struct setting boot_protocol_setting __setting( SETTING_FLEXBOOT, boot_protocol ) = {
	.name = "boot_protocol",
	.description = "Legacy boot protocol",
	.type = &setting_type_string,
	.scope = &nic_scope,
};

struct extended_setting ext_boot_protocol __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &boot_protocol_setting,
	.type = OPTION,
};

struct setting virt_lan_setting __setting( SETTING_FLEXBOOT, virt_lan ) = {
	.name = "virt_lan",
	.description = "Virtual LAN mode",
	.type = &setting_type_string,
	.scope = &nic_scope,
};

struct extended_setting ext_virt_lan __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &virt_lan_setting,
	.type = OPTION,
};

struct setting virt_id_setting __setting( SETTING_FLEXBOOT, virt_id ) = {
	.name = "virt_id",
	.description = "Virtual LAN ID",
	.type = &setting_type_int32,
	.scope = &nic_scope,
};

struct extended_setting ext_virt_id __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &virt_id_setting,
	.type = INPUT,
};

struct setting boot_pkey_setting __setting( SETTING_FLEXBOOT, boot_pkey ) = {
	.name = "boot_pkey",
	.description = "PKey Value",
	.type = &setting_type_int32,
	.scope = &nic_scope,
};

struct extended_setting ext_boot_pkey __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &boot_pkey_setting,
	.type = INPUT,
};

struct setting opt_rom_setting __setting( SETTING_FLEXBOOT, opt_rom ) = {
	.name = "opt_rom",
	.description = "Option ROM",
	.type = &setting_type_string,
	.scope = &nic_scope,
};

struct extended_setting ext_opt_rom __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &opt_rom_setting,
	.type = OPTION,
};

struct setting boot_retries_setting __setting( SETTING_FLEXBOOT, boot_retries ) = {
	.name = "boot_retries",
	.description = "Boot retry count",
	.type = &setting_type_string,
	.scope = &nic_scope,
};

struct extended_setting ext_boot_retries __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &boot_retries_setting,
	.type = OPTION,
};

struct setting wol_setting __setting( SETTING_FLEXBOOT, wol ) = {
	.name = "wol",
	.description = "Wake on LAN",
	.type = &setting_type_string,
	.scope = &nic_scope,
};

struct extended_setting ext_wol __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &wol_setting,
	.type = OPTION,
};

struct setting ip_ver_setting __setting( SETTING_FLEXBOOT, ip_ver ) = {
	.name = "ip_ver",
	.description = "IP Version",
	.type = &setting_type_string,
	.scope = &iscsi_general_scope,
};

struct extended_setting ext_ip_ver __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &ip_ver_setting,
	.type = LABEL,
};

struct setting dhcp_ip_setting __setting( SETTING_FLEXBOOT, dhcp_ip ) = {
	.name = "dhcp_ip",
	.description = "DHCP IP",
	.type = &setting_type_string,
	.scope = &iscsi_general_scope,
};

struct extended_setting ext_dhcp_ip __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &dhcp_ip_setting,
	.type = OPTION,
};

struct setting dhcp_iscsi_setting __setting( SETTING_FLEXBOOT, dhcp_iscsi ) = {
	.name = "dhcp_iscsi",
	.description = "DHCP Parameters",
	.type = &setting_type_string,
	.scope = &iscsi_general_scope,
};

struct extended_setting ext_dhcp_iscsi __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &dhcp_iscsi_setting,
	.type = OPTION,
};

struct setting iscsi_chap_setting __setting( SETTING_FLEXBOOT, iscsi_chap ) = {
	.name = "iscsi_chap_auth",
	.description = "CHAP Authentication",
	.type = &setting_type_string,
	.scope = &iscsi_general_scope,
};

struct extended_setting ext_iscsi_chap __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &iscsi_chap_setting,
	.type = OPTION,
};

struct setting iscsi_mutual_chap_setting __setting( SETTING_FLEXBOOT, iscsi_mutual_chap ) = {
	.name = "iscsi_mutual_chap_auth",
	.description = "CHAP Mutual Authentication",
	.type = &setting_type_string,
	.scope = &iscsi_general_scope,
};

struct extended_setting ext_iscsi_mutual_chap __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &iscsi_mutual_chap_setting,
	.type = OPTION,
};

struct setting iscsi_boot_to_target_setting __setting( SETTING_FLEXBOOT, iscsi_boot_to_target ) = {
	.name = "iscsi_boot_to_target",
	.description = "Boot to target",
	.type = &setting_type_string,
	.scope = &iscsi_general_scope,
};

struct extended_setting ext_iscsi_boot_to_target __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &iscsi_boot_to_target_setting,
	.type = OPTION,
};

struct setting ipv4_add_setting __setting( SETTING_FLEXBOOT, ipv4_add ) = {
	.name = "ipv4_add",
	.description = "IPv4 Address",
	.type = &setting_type_ipv4,
	.scope = &iscsi_init_scope,
};

struct extended_setting ext_ipv4_add __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &ipv4_add_setting,
	.type = INPUT,
};

struct setting subnet_mask_setting __setting( SETTING_FLEXBOOT, subnet_mask ) = {
	.name = "subnet_mask",
	.description = "Subnet mask",
	.type = &setting_type_ipv4,
	.scope = &iscsi_init_scope,
};

struct extended_setting ext_subnet_mask __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &subnet_mask_setting,
	.type = INPUT,
};

struct setting ipv4_gateway_setting __setting( SETTING_FLEXBOOT, ipv4_gateway ) = {
	.name = "ipv4_gateway",
	.description = "IPv4 Default Gateway",
	.type = &setting_type_ipv4,
	.scope = &iscsi_init_scope,
};

struct extended_setting ext_ipv4_gateway __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &ipv4_gateway_setting,
	.type = INPUT,
};

struct setting ipv4_dns_setting __setting( SETTING_FLEXBOOT, ipv4_dns ) = {
	.name = "ipv4_dns",
	.description = "IPv4 Primary DNS",
	.type = &setting_type_ipv4,
	.scope = &iscsi_init_scope,
};

struct extended_setting ext_ipv4_dns __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &ipv4_dns_setting,
	.type = INPUT,
};

struct setting iscsi_init_name_setting __setting( SETTING_FLEXBOOT, iscsi_init_name ) = {
	.name = "iscsi_init_name",
	.description = "iSCSI Name",
	.type = &setting_type_string,
	.scope = &iscsi_init_scope,
};

struct extended_setting ext_iscsi_init_name __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &iscsi_init_name_setting,
	.type = INPUT,
};

struct setting init_chapid_setting __setting( SETTING_FLEXBOOT, init_chapid ) = {
	.name = "init_chapid",
	.description = "CHAP ID",
	.type = &setting_type_string,
	.scope = &iscsi_init_scope,
};

struct extended_setting ext_init_chapid __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &init_chapid_setting,
	.type = INPUT,
};

struct setting init_chapsec_setting __setting( SETTING_FLEXBOOT, init_chapsec ) = {
	.name = "init_chapsec",
	.description = "CHAP Secret",
	.type = &setting_type_string,
	.scope = &iscsi_init_scope,
};

struct extended_setting ext_init_chapsec __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &init_chapsec_setting,
	.type = INPUT,
};

struct setting connect_setting __setting( SETTING_FLEXBOOT, connect ) = {
	.name = "connect",
	.description = "Connect",
	.type = &setting_type_string,
	.scope = &iscsi_target_scope,
};

struct extended_setting ext_connect __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &connect_setting,
	.type = OPTION,
};

struct setting target_ip_setting __setting( SETTING_FLEXBOOT, target_ip ) = {
	.name = "target_ip",
	.description = "IP Address",
	.type = &setting_type_ipv4,
	.scope = &iscsi_target_scope,
};

struct extended_setting ext_target_ip __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &target_ip_setting,
	.type = INPUT,
};

struct setting tcp_port_setting __setting( SETTING_FLEXBOOT, tcp_port ) = {
	.name = "tcp_port",
	.description = "TCP Port",
	.type = &setting_type_int32,
	.scope = &iscsi_target_scope,
};

struct extended_setting ext_tcp_port __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &tcp_port_setting,
	.type = INPUT,
};

struct setting boot_lun_setting __setting( SETTING_FLEXBOOT, boot_lun ) = {
	.name = "boot_lun",
	.description = "Boot LUN",
	.type = &setting_type_int32,
	.scope = &iscsi_target_scope,
};

struct extended_setting ext_boot_lun __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &boot_lun_setting,
	.type = INPUT,
};

struct setting iscsi_target_name_setting __setting( SETTING_FLEXBOOT, iscsi_target_name ) = {
	.name = "iscsi_target_name",
	.description = "iSCSI Name",
	.type = &setting_type_string,
	.scope = &iscsi_target_scope,
};

struct extended_setting ext_iscsi_target_name __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &iscsi_target_name_setting,
	.type = INPUT,
};

struct setting target_chapid_setting __setting( SETTING_FLEXBOOT, target_chapid ) = {
	.name = "target_chapid",
	.description = "CHAP ID",
	.type = &setting_type_string,
	.scope = &iscsi_target_scope,
};

struct extended_setting ext_target_chapid __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &target_chapid_setting,
	.type = INPUT,
};

struct setting target_chapsec_setting __setting( SETTING_FLEXBOOT, target_chapsec ) = {
	.name = "target_chapsec",
	.description = "CHAP Secret",
	.type = &setting_type_string,
	.scope = &iscsi_target_scope,
};

struct extended_setting ext_target_chapsec __table_entry ( NV_CONFIG, 01 ) = {
	.setting = &target_chapsec_setting,
	.type = INPUT,
};

#define DRIVER_SETTINGS_FETCH_SETTING( settings, setting )					\
	do {																	\
		memset ( buf, 0, sizeof ( buf ) );									\
		if ( ( rc = fetchf_setting ( settings, setting , NULL, NULL, buf,	\
							sizeof ( buf ) ) ) <= 0 ) {						\
				DBGC ( settings, "Failed to fetch %s setting (rc = %d)\n",	\
						( setting )->name, rc );							\
		}																	\
	} while ( 0 )

/*************************** Extended Settings functions ********************************/

struct extended_setting *find_extended ( const struct setting *setting ) {
	struct extended_setting *ext;

	for_each_table_entry ( ext, NV_CONFIG ) {
		if ( strcmp (ext->setting->name, setting->name) == 0 ) {
			return ext;
		}
	}
	return NULL;
}

//**************************************************************************************//

static int create_options_list ( struct extended_setting *ext, int size, char **array ) {
	struct options_list *options = malloc ( sizeof (*options) );
	int i;

	options->options_array = malloc ( size * sizeof (char*));
	for ( i = 0; i < size; i++ ) {
		( (char**)options->options_array )[i] = malloc ( strlen( array[i] ) + 1 );
		strcpy ( ((char**)options->options_array)[i], array[i] );
	}
	options->options_num = size;
	options->current_index = 0;
	ext->data = options;
	return 0;
}

#define STR_NO_RETRIES	"No retries"
#define STR_1_RETRY	"1 Retry"
#define STR_2_RETRIES	"2 Retries"
#define STR_3_RETRIES	"3 Retries"
#define STR_4_RETRIES	"4 Retries"
#define STR_5_RETRIES	"5 Retries"
#define STR_6_RETRIES	"6 Retries"
#define STR_INDEF_RET	"Indefinite Retries"

struct driver_settings* get_driver_settings_from_settings ( struct settings *settings ) {
	struct settings *main_settings = settings;
	struct generic_settings *generic;
	struct driver_settings *driver_settings;

	/* If these are not device settings, needs to get to main settings */
	if ( ( settings->default_scope != &main_scope ) && ( settings->default_scope != &fw_scope ) ) {
		while ( main_settings->default_scope != &port_scope )
			main_settings = main_settings->parent;
	} else {
		if ( settings->default_scope == &fw_scope )
			main_settings = main_settings->parent;
	}

	generic = container_of ( main_settings, struct generic_settings, settings);
	driver_settings = container_of ( generic, struct driver_settings, generic_settings);

	return driver_settings;
}

struct settings * driver_settings_from_netdev ( struct net_device *netdev ) {
	struct settings *settings_root;
	struct settings *settings;

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return NULL;
	}

	settings_root = &nv_settings_root->settings;

	list_for_each_entry ( settings, &settings_root->children, siblings ) {
		/* The driver settings starts with the name of the net device,
		 * for example, net0: ..., so its enough to compare the the 4th
		 * char which is the number of the net device.
		 */
		if ( settings->name[3] == netdev->name[3] )
			return settings;
	}

	return NULL;
}

int driver_settings_get_boot_prot_val ( struct settings *settings ) {
	char buf[20] = { 0 };

	if ( fetchf_setting ( settings, &boot_protocol_setting, NULL, NULL, buf, sizeof ( buf ) ) <= 0 )
		return BOOT_PROTOCOL_NONE;
	if ( buf[0] == 'P' )
		return BOOT_PROTOCOL_PXE;
	if ( buf[0] == 'i' )
		return BOOT_PROTOCOL_ISCSI;
	return BOOT_PROTOCOL_NONE;
}

int driver_settings_get_ib_pkey_val ( struct settings *settings ) {
	char buf[255];
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &boot_pkey_setting );
	return strtoul ( buf, NULL, 10 );
}

int driver_settings_get_iscsi_boot_to_target_val ( struct settings *settings ) {
	char buf[ sizeof ( STR_ONE_TIME_DISABLED ) + 1 ] = { 0 };
	int rc;

	if ( fetchf_setting ( settings, &iscsi_boot_to_target_setting, NULL, NULL, buf, sizeof ( buf ) ) <= 0 )
		return ISCSI_BOOT_TO_TARGET_ENABLE;

	switch ( buf[0] ) {
	case 'E':
		rc = ISCSI_BOOT_TO_TARGET_ENABLE;
		break;
	case 'D':
	case 'O':
		rc = ISCSI_BOOT_TO_TARGET_DISABLE;
		break;
	default:
		rc = ISCSI_BOOT_TO_TARGET_ENABLE;
		break;
	};

	return rc;
}

int driver_settings_get_boot_ret_val ( struct settings *settings ) {
       char buf[20] = { 0 };

       if ( fetchf_setting ( settings, &boot_retries_setting, NULL, NULL, buf, sizeof ( buf ) ) <= 0 )
               return 0;
       if ( buf[0] == 'I' )
               return 7;
       if ( buf[0] == 'N' )
               return 0;
       return ( buf[0] - '0' );
}

const char * driver_settings_get_boot_ret_str ( unsigned int value ) {
	switch ( value ) {
		case 1: return STR_1_RETRY;
		case 2:	return STR_2_RETRIES;
		case 3: return STR_3_RETRIES;
		case 4: return STR_4_RETRIES;
		case 5: return STR_5_RETRIES;
		case 6: return STR_6_RETRIES;
		case 7: return STR_INDEF_RET;
		case 0:
			;/* Fall through */
		default:
			;/* Fall through */
	}

	return STR_NO_RETRIES;
}

static struct extended_options extended_options_list[] = {
	{ &ext_virt_mode,			2, { STR_NONE, STR_SRIOV } },
	{ &ext_boot_protocol,		3, { STR_NONE, STR_PXE, STR_ISCSI } },
	{ &ext_virt_lan,			2, { STR_DISABLE, STR_ENABLE } },
	{ &ext_opt_rom,				2, { STR_DISABLE, STR_ENABLE } },
	{ &ext_boot_retries,		8, { STR_NO_RETRIES, STR_1_RETRY, STR_2_RETRIES,
									 STR_3_RETRIES, STR_4_RETRIES, STR_5_RETRIES,
									 STR_6_RETRIES, STR_INDEF_RET } },
	{ &ext_iscsi_boot_to_target, 3, { STR_DISABLE, STR_ONE_TIME_DISABLED,
													STR_ENABLE } },
	{ &ext_wol,					2, { STR_DISABLE, STR_ENABLE } },
	{ &ext_connect,				2, { STR_DISABLE, STR_ENABLE } },
	{ &ext_dhcp_ip,				2, { STR_DISABLE, STR_ENABLE } },
	{ &ext_dhcp_iscsi,			2, { STR_DISABLE, STR_ENABLE } },
	{ &ext_iscsi_chap,			2, { STR_DISABLE, STR_ENABLE } },
	{ &ext_iscsi_mutual_chap,	2, { STR_DISABLE, STR_ENABLE } },
};

static struct extended_description extended_description_list[] = {
	{ &ext_virt_num, "Enter a non-negative number smaller than maximum supported" },
	{ &ext_blink_leds, "Enter positive number in range 0-15" },
	{ &ext_virt_id, "Enter positive number in range 1-4094" },
	{ &ext_ipv4_add, "Specify the IPv4 address of the iSCSI initiator" },
	{ &ext_subnet_mask, "Specify the IPv4 subnet mask of the iSCSI initiator" },
	{ &ext_ipv4_gateway, "Specify the IPv4 default gateway of the iSCSI initiator" },
	{ &ext_ipv4_dns, "Specify the IPv4 primary DNS IP of the iSCSI initiator" },
	{ &ext_iscsi_init_name, "Specify the initiator iSCSI Qualified Name. Maximum length is 223" },
	{ &ext_init_chapid, "iSCSI initiator reverse CHAP ID. Maximum length is 128" },
	{ &ext_init_chapsec, "iSCSI initiator reverse CHAP secret. Length should be 0 or 12 to 16" },
	{ &ext_target_ip,"Specify the IP address of the first iSCSI target" },
	{ &ext_tcp_port, "TCP port number of the first iSCSI target. Enter number in range 1-65535" },
	{ &ext_boot_lun, "Boot LUN on the first iSCSI storage target. Enter number in range 0-255" },
	{ &ext_iscsi_target_name, "iSCSI Qualified Name of the first iSCSI storage target. Maximum length is 223" },
	{ &ext_target_chapid, "First iSCSI storage target CHAP ID. Maximum length is 128" },
	{ &ext_target_chapsec, "First iSCSI storage target CHAP secret. Length should be 0 or 12 to 16" },
	{ &ext_boot_protocol, "Select boot protocol priority. If chosen, protocol will be tried first" },
	{ &ext_flexboot_menu_to_setting, "Enter a timeout in seconds range 1-14" },
	{ &ext_boot_pkey, "PKey to be used by PXE boot. Enter a number in range 0-65535" },
};

void fill_extended_settings (void){

	unsigned int i;
	char *instructions;

	for ( i = 0 ; i < ( sizeof ( extended_options_list ) / sizeof ( extended_options_list[0] ) ) ; i++ ) {
		create_options_list ( extended_options_list[i].ext, extended_options_list[i].size,
					extended_options_list[i].array);
	}

	for ( i = 0 ; i < ( sizeof ( extended_description_list ) / sizeof ( extended_description_list[0] ) ) ; i++ ) {
		instructions = malloc ( strlen ( extended_description_list[i].description ) + 1 );
		strcpy (instructions, extended_description_list[i].description);
		extended_description_list[i].ext->instructions = instructions;
	}

}

int init_firmware_settings ( struct settings_operations *operations ) {
	struct generic_settings *root_settings;
	struct generic_settings *firmware_settings;

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return -EINVAL;
	}

	root_settings = nv_settings_root;
	firmware_settings = zalloc ( sizeof ( struct generic_settings ) );
	if ( firmware_settings == NULL ) {
		printf ( "%s: Failed to allocate memory for firmware settings\n", __FUNCTION__ );
		return -ENOMEM;
	}

	generic_settings_init ( firmware_settings, NULL);
	register_settings ( &firmware_settings->settings, &root_settings->settings, "Firmware Image Properties" );
	firmware_settings->settings.op = operations;
	firmware_settings->settings.default_scope = &fw_scope;

	return 0;
}

static void destroy_settings_children ( struct settings *settings ) {
	struct settings *iterator;
	struct settings *temp_settings;
	struct generic_settings *generic;

	if ( list_empty ( &settings->children ) )
		return;

	list_for_each_entry_safe ( iterator, temp_settings, &settings->children, siblings ) {
		destroy_settings_children ( iterator );
		list_del ( &iterator->siblings );
		generic = container_of ( iterator, struct generic_settings, settings);
		free ( generic );
	}

	return;
}

static void destroy_modified_list ( struct driver_settings *driver_settings ) {
	struct extended_setting *iterator;
	struct extended_setting *temp_ext;

	if ( list_empty ( &driver_settings->modified_list ) )
		return;

	list_for_each_entry_safe ( iterator, temp_ext, &driver_settings->modified_list, modified ) {
		list_del ( &iterator->modified );
		free ( iterator );
	}

	return;
}

void destroy_driver_settings () {
	struct settings *settings_root;
	struct settings *settings;
	struct settings *temp_settings;
	struct generic_settings *generic;
	struct extended_setting *ext;
	struct options_list *options;
	struct driver_settings *driver_settings;
	unsigned int i;

	if ( ! nv_settings_root ) {
		return;
	}

	settings_root = &nv_settings_root->settings;

	/* Free all memory allocated for modified lists */
	list_for_each_entry ( driver_settings, &driver_settings_list, list )
		destroy_modified_list ( driver_settings );

	/* Free all memory allocated for settings */
	list_for_each_entry_safe ( settings, temp_settings, &settings_root->children, siblings ) {
		if ( settings ) {
			destroy_settings_children ( settings );
			list_del ( &settings->siblings );
			if ( settings->default_scope != &port_scope ) {
				generic = container_of ( settings, struct generic_settings, settings);
				free ( generic );
			} else {
				unregister_settings ( settings );
			}
		}
	}
	unregister_settings ( settings_root );

	/* Free all memory allocated for extended setting */
	for_each_table_entry ( ext, NV_CONFIG ) {
		if ( ext->instructions )
			free ( ext->instructions );
		if ( ext->type == OPTION ) {
			options = (struct options_list*)ext->data;
			if ( options ) {
				for ( i = 0; i < options->options_num; i ++ ) {
					if ( options->options_array[i] )
						free ( options->options_array[i] );
				}
				free (options);
			}
		}
	}

	/* Set root to NULL so function is executed once */
	nv_settings_root = NULL;

	return;
}

static int is_protocol_iscsi ( struct settings *settings ) {
	struct settings *main_settings;
	struct settings *nic_settings;
	char protocol[8] = { 0 };

	main_settings = settings;
        while ( main_settings->default_scope != &port_scope )
                main_settings = main_settings->parent;

	list_for_each_entry ( nic_settings, &main_settings->children, siblings ) {
		if ( nic_settings->default_scope == &nic_scope )
			break;
	}

	generic_settings_fetch ( nic_settings, &boot_protocol_setting, protocol, sizeof ( protocol ) );
        if ( protocol[0] == 'i' )
                return 1;

	return 0;
}

int root_path_store ( struct settings *netdev_settings, struct settings *driver_settings ) {
	struct settings *tmp_settings;
	char ip_address[256] = { 0 };
	char port[256] = { 0 };
	char boot_lun[256] = { 0 };
	char iscsi_name[256] = { 0 };
	char root_path[MAX_ROOT_PATH_LEN] = { 0 };
	char connect[10] = { 0 };
	char dhcp_ip[10] = { 0 };
	char dhcp_iscsi[10] = { 0 };
	int rc;

	tmp_settings = driver_settings;
	while ( tmp_settings->default_scope != &port_scope )
		tmp_settings = tmp_settings->parent;

	/* First check if connect is enabled and iscsi is the boot protocol,
	 * if not then don't save in system settings */
	fetchf_setting ( tmp_settings, &connect_setting, NULL, NULL, connect, sizeof ( connect ) );
	fetchf_setting ( tmp_settings, &dhcp_iscsi_setting, NULL, NULL, dhcp_iscsi, sizeof ( dhcp_iscsi ) );
	fetchf_setting ( tmp_settings, &dhcp_ip_setting, NULL, NULL, dhcp_ip, sizeof ( dhcp_ip ) );
	if ( ( ( dhcp_ip[0] == 'E' ) && ( dhcp_iscsi[0] == 'E' ) ) || ( connect[0] != 'E' ) ) {
		storef_setting ( netdev_settings, &root_path_setting, NULL );
		/* Clear the root_path setting from DHCP/pxebs/proxyDHCP if it exists */
		list_for_each_entry ( tmp_settings, &netdev_settings->children, siblings ) {
			storef_setting ( tmp_settings, &root_path_setting, NULL );
		}
		return SUCCESS;
	}

	fetchf_setting ( driver_settings, &target_ip_setting, NULL, NULL, ip_address, sizeof ( ip_address ) );
	fetchf_setting ( driver_settings, &tcp_port_setting, NULL, NULL, port, sizeof ( port ) );
	fetchf_setting ( driver_settings, &boot_lun_setting, NULL, NULL, boot_lun, sizeof ( boot_lun ) );
	fetchf_setting ( driver_settings, &iscsi_target_name_setting, NULL, NULL, iscsi_name, sizeof ( iscsi_name ) );

	snprintf ( root_path, MAX_ROOT_PATH_LEN, "iscsi:%s:%s:%s:%s:%s",
						ip_address[0] ? ip_address : "\0",
						"\0", /*protocol */
						port[0]	? port	: "\0",
						boot_lun[0] ? boot_lun : "\0",
						iscsi_name[0] ? iscsi_name : "\0");

	rc = store_setting ( netdev_settings, &root_path_setting, root_path, sizeof ( root_path ) );
	if ( rc != 0 )
		return INVALID_INPUT;

	return SUCCESS;
}

static struct net_device* get_netdev_from_settings ( struct settings *settings ) {
	struct settings *main_settings = settings;
	struct generic_settings *generic;
	struct driver_settings *driver_set;

	while ( main_settings->default_scope != &port_scope )
		main_settings = main_settings->parent;

	generic = container_of ( main_settings, struct generic_settings, settings);
	driver_set = container_of ( generic, struct driver_settings, generic_settings);

	return driver_set->netdev;
//	for_each_netdev ( netdev ) {
//		if ( driver_set->index == ( netdev->index + 1 ) )
//			return netdev;
//	}
//	printf ( "%s: Fatal error!\n", __FUNCTION__ );
//	return NULL;
}

static int netdevice_settings_store ( struct settings *settings,
		const struct setting *setting, const void *data, size_t len ) {
	struct settings *port_settings = settings;
	struct net_device *netdev = get_netdev_from_settings ( settings );
	char dhcp_ip_en[10] = { 0 };
	int rc;

	/* Check if boot protocol is iSCSI, if not then don't store system settings */
	if ( ! is_protocol_iscsi ( settings ) )
		return SUCCESS;

	while ( port_settings->default_scope != &port_scope )
		port_settings = port_settings->parent;

	fetchf_setting ( port_settings, &dhcp_ip_setting, NULL, NULL, dhcp_ip_en, sizeof ( dhcp_ip_en ) );
	if ( dhcp_ip_en[0] == 'E' )
		return SUCCESS;

	if ( ( rc = store_setting ( netdev_settings ( netdev ), setting, data, len ) ) == 0 )
		return SUCCESS;

	return INVALID_INPUT;
}

static int ipv4_address_store ( struct settings *settings , const void *data,
		    size_t len, const struct setting *setting __unused)  {
	return netdevice_settings_store ( settings, &ip_setting, data, len );
}

static int subnet_mask_store ( struct settings *settings , const void *data,
                    size_t len, const struct setting *setting __unused)  {
	return netdevice_settings_store( settings, &netmask_setting, data, len );
}

static int ipv4_gateway_store ( struct settings *settings , const void *data,
                    size_t len, const struct setting *setting __unused)  {
	return netdevice_settings_store( settings, &gateway_setting, data, len );
}

static int ipv4_dns_store ( struct settings *settings , const void *data,
                    size_t len, const struct setting *setting __unused)  {
	return netdevice_settings_store( settings, &dns_setting, data, len );
}

static int iscsi_settings_check_and_store ( struct settings *settings, const struct setting *setting,
		const void *data, size_t len, unsigned int max_len, unsigned int min_len ) {
	struct net_device *netdev = get_netdev_from_settings ( settings );

	if ( len != 0 && ( len > max_len || len < min_len ) )
		return INVALID_INPUT;

	if ( store_setting ( netdev_settings ( netdev ), setting, data, len ) )
		return INVALID_INPUT;

	return SUCCESS;
}

static int iscsi_initiator_name_store ( struct settings *settings , const void *data,
                    size_t len , const struct setting *setting __unused ) {
	return iscsi_settings_check_and_store ( settings, &initiator_iqn_setting,
			data, len, MAX_ISCSI_NAME, MIN_ISCSI_NAME );
}

static int iscsi_initiator_chapid_store ( struct settings *settings , const void *data,
                    size_t len , const struct setting *setting __unused ) {
	struct settings *port_settings = settings;
	char chap_en[10] = { 0 };
	char mut_chap_en[10] = { 0 };
	char connect[10] = { 0 };

	while ( port_settings->default_scope != &port_scope )
		port_settings = port_settings->parent;

	fetchf_setting ( port_settings, &connect_setting, NULL, NULL, connect, sizeof ( connect ) );
	fetchf_setting ( port_settings, &iscsi_chap_setting, NULL, NULL, chap_en, sizeof ( chap_en ) );
	fetchf_setting ( port_settings, &iscsi_mutual_chap_setting, NULL, NULL, mut_chap_en, sizeof ( mut_chap_en ) );
	if ( ( chap_en[0] == 'D' ) || ( mut_chap_en[0] == 'D' ) || ( connect[0] != 'E' ) ) {
		return SUCCESS;
	}

	return iscsi_settings_check_and_store ( settings, &reverse_username_setting,
			data, len, MAX_CHAP_ID , MIN_CHAP_ID );
}

static int iscsi_initiator_chapsec_store ( struct settings *settings , const void *data,
                    size_t len , const struct setting *setting __unused ) {
	struct settings *port_settings = settings;
	char chap_en[10] = { 0 };
	char mut_chap_en[10] = { 0 };
	char connect[10] = { 0 };

	while ( port_settings->default_scope != &port_scope )
		port_settings = port_settings->parent;

	fetchf_setting ( port_settings, &connect_setting, NULL, NULL, connect, sizeof ( connect ) );
	fetchf_setting ( port_settings, &iscsi_chap_setting, NULL, NULL, chap_en, sizeof ( chap_en ) );
	fetchf_setting ( port_settings, &iscsi_mutual_chap_setting, NULL, NULL, mut_chap_en, sizeof ( mut_chap_en ) );
	if ( ( chap_en[0] == 'D' ) || ( mut_chap_en[0] == 'D' ) || ( connect[0] != 'E' ) ) {
		return SUCCESS;
	}

	return iscsi_settings_check_and_store ( settings, &reverse_password_setting,
			data, len, MAX_CHAP_SECRET , MIN_CHAP_SECRET );
}

static int iscsi_target_chapid_store ( struct settings *settings , const void *data,
                    size_t len , const struct setting *setting __unused ) {
	struct settings *port_settings = settings;
	char chap_en[10] = { 0 };
	char connect[10] = { 0 };

	while ( port_settings->default_scope != &port_scope )
		port_settings = port_settings->parent;

	fetchf_setting ( port_settings, &connect_setting, NULL, NULL, connect, sizeof ( connect ) );
	fetchf_setting ( port_settings, &iscsi_chap_setting, NULL, NULL, chap_en, sizeof ( chap_en ) );
	if ( ( chap_en[0] == 'D' ) || ( connect[0] != 'E' ) ) {
		return SUCCESS;
	}

	return iscsi_settings_check_and_store ( settings, &username_setting,
			data, len, MAX_CHAP_ID , MIN_CHAP_ID );
}

static int iscsi_target_chapsec_store ( struct settings *settings , const void *data,
                    size_t len , const struct setting *setting __unused ) {
	struct settings *port_settings = settings;
	char chap_en[10] = { 0 };
	char connect[10] = { 0 };

	while ( port_settings->default_scope != &port_scope )
		port_settings = port_settings->parent;

	fetchf_setting ( port_settings, &connect_setting, NULL, NULL, connect, sizeof ( connect ) );
	fetchf_setting ( port_settings, &iscsi_chap_setting, NULL, NULL, chap_en, sizeof ( chap_en ) );
	if ( ( chap_en[0] == 'D' ) || ( connect[0] != 'E' ) ) {
		return SUCCESS;
	}

	return iscsi_settings_check_and_store ( settings, &password_setting,
		data, len, MAX_CHAP_SECRET , MIN_CHAP_SECRET );
}

static int iscsi_tgt_store_root_path ( struct settings *settings, const struct setting *setting,
			const void *data, size_t len ) {
	struct net_device *netdev;
	char old[256];
	int fetch_rc, rc;

	if ( ! settings )
		return INVALID_INPUT;

	/* Save previous value in case of error */
	fetch_rc = fetchf_setting ( settings, setting, NULL, NULL, old, sizeof ( old ) );
	if ( ( rc = generic_settings_store ( settings, setting, data, len ) ) )
		return rc;

	netdev = get_netdev_from_settings ( settings );
	if ( ( rc = root_path_store ( netdev_settings ( netdev ), settings ) ) ) {
		if ( fetch_rc > 0 )
			generic_settings_store ( settings, setting, old, sizeof ( old ) );
		return rc;
	}

	return 0;
}

static int target_ip_store ( struct settings *settings, const void *data,
                    size_t len, const struct setting *setting ) {
	return iscsi_tgt_store_root_path ( settings, setting, data, len );
}

static int target_iscsi_name_store ( struct settings *settings , const void *data,
                    size_t len , const struct setting *setting ) {
	if ( data && len > MAX_ISCSI_NAME ) {
		return INVALID_INPUT;
	}

	return iscsi_tgt_store_root_path ( settings, setting, data, len );
}

static int target_tcp_port_store ( struct settings *settings , const void *data,
                    size_t len, const struct setting *setting ) {
	int32_t input = 0;

	if ( data ) {
		input = ntohl ( *( ( int32_t * ) data ) );

		if ( input > MAX_TCP_PORT || input < MIN_TCP_PORT )
			return INVALID_INPUT;
	}

	return iscsi_tgt_store_root_path ( settings, setting, data, len );
}

static int target_boot_lun_store ( struct settings *settings , const void *data,
                    size_t len, const struct setting *setting ) {
	int32_t input = 0;

	if ( data ) {
		input = ntohl ( *( ( int32_t * ) data ) );

		if ( input > MAX_BOOT_LUN || input < MIN_BOOT_LUN )
			return INVALID_INPUT;
	}

	return iscsi_tgt_store_root_path ( settings, setting, data, len );
}

static int is_modified ( struct list_head *list, const char *name ) {
	struct extended_setting *ext;

	list_for_each_entry ( ext, list, modified ) {
		if ( strcmp (ext->setting->name, name ) == 0 )
			return 1;
	}
	return 0;
}

/******************************************************************************/
/************************* Driver setting operations **************************/
/******************************************************************************/
static int nic_boot_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	struct nv_nic_conf *nic_conf = & ( conf->nic );
	union nv_nic_boot_conf *boot_conf = & ( nic_conf->boot_conf );
	union nv_nic_boot_conf reversed_boot_conf;
	char buf[255] = {0};
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &boot_protocol_setting );
	switch ( buf[0] ) {
		case 'N' : boot_conf->legacy_boot_prot = BOOT_PROTOCOL_NONE;
			break;
		case 'P' : boot_conf->legacy_boot_prot = BOOT_PROTOCOL_PXE;
			break;
		case 'i' : boot_conf->legacy_boot_prot = BOOT_PROTOCOL_ISCSI;
			break;
	}
	DRIVER_SETTINGS_FETCH_SETTING ( settings, &virt_lan_setting );
	boot_conf->en_vlan = ( buf[0] == 'E' );

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &virt_id_setting );
	boot_conf->vlan_id = strtoul ( buf, NULL, 10 );

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &opt_rom_setting );
	boot_conf->en_option_rom = ( buf[0] == 'E' );

	boot_conf->boot_retry_count = driver_settings_get_boot_ret_val ( settings );
	reversed_boot_conf.dword = cpu_to_be32 ( boot_conf->dword );

	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
				&reversed_boot_conf, driver_settings->index,
				BOOT_SETTINGS_TYPE, sizeof ( reversed_boot_conf ) ) ) )
		return -EACCES;
	return 0;
}

static int wol_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	union nv_wol_conf *wol_conf = & ( conf->nic.wol_conf );
	char buf[255] = {0};
	uint32_t swapped[2];
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &wol_setting );
	wol_conf->en_wol_magic = ( buf[0] == 'E' );
	swapped[0] = cpu_to_be32(wol_conf->dword[0]);
	swapped[1] = cpu_to_be32(wol_conf->dword[1]);

	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
				swapped, driver_settings->index, WAKE_ON_LAN_TYPE,
				sizeof ( *wol_conf ) ) ) )
		return -EACCES;
	return 0;
}

static int nic_ib_boot_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	union nv_nic_ib_boot_conf *ib_boot_conf = & ( conf->nic.ib_boot_conf );
	union nv_nic_ib_boot_conf reversed_ib_boot_conf;
	char buf[255];
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &boot_pkey_setting );
	ib_boot_conf->boot_pkey =  strtoul ( buf, NULL, 10 );
	reversed_ib_boot_conf.dword = cpu_to_be32 ( ib_boot_conf->dword );

	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
				&reversed_ib_boot_conf, driver_settings->index,
				IB_BOOT_SETTING_TYPE, sizeof ( reversed_ib_boot_conf ) ) ) )
		return -EACCES;
	return 0;
}

static int dhcp_flags_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	union nv_iscsi_init_dhcp_conf  *init_conf = & (conf->iscsi.init_dhcp_conf );
	union nv_iscsi_init_dhcp_conf swapped;
	char buf[255] = {0};
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &dhcp_ip_setting );
	init_conf->ipv4_dhcp_en = ( buf[0] == 'E' );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, &dhcp_iscsi_setting );
	init_conf->dhcp_iscsi_en = ( buf[0] == 'E' );

	swapped.dword = cpu_to_be32 ( init_conf->dword );
	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
				&swapped, driver_settings->index,
			ISCSI_INITIATOR_DHCP_CONF_TYPE,	sizeof ( swapped ) ) ) ) {
		return -EACCES;
	}
	return 0;
}

static int iscsi_gen_flags_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	union nv_iscsi_general *gen_conf = & ( conf->iscsi.gen_conf );
	union nv_iscsi_general swapped;
	char buf[255] = {0};
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &iscsi_chap_setting );
	gen_conf->chap_auth_en = ( buf[0] == 'E' );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, &iscsi_mutual_chap_setting );
	gen_conf->chap_mutual_auth_en = ( buf[0] == 'E' );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, &iscsi_boot_to_target_setting );
	switch ( buf[0] ) {
	case 'E' : gen_conf->boot_to_target = ISCSI_BOOT_TO_TARGET_ENABLE;
		break;
	case 'D' : gen_conf->boot_to_target = ISCSI_BOOT_TO_TARGET_DISABLE;
		break;
	case 'O' : gen_conf->boot_to_target = ISCSI_BOOT_TO_TARGET_ONE_TIME_DISABLE;
		break;
	}

	swapped.dword[0] = cpu_to_be32 ( gen_conf->dword[0] );
	swapped.dword[1] = cpu_to_be32 ( gen_conf->dword[1] );
	swapped.dword[2] = cpu_to_be32 ( gen_conf->dword[2] );

	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
				&swapped, driver_settings->index,
				ISCSI_GENERAL_SETTINGS_TYPE, sizeof ( swapped ) ) ) ) {
		return -EACCES;
	}

	return 0;
}

static int iscsi_parameters_nv_store ( struct driver_settings *driver_settings,
				struct setting *setting, void *source,
				uint32_t size, uint32_t tlv_type ) {
	int index = driver_settings->index;
	char buf[255] = {0};
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( & ( driver_settings->generic_settings.settings ), setting );
	if ( rc > 0 ) {
		memcpy ( source, buf, size );
		rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
				source, index, tlv_type, size );
	} else {
		rc = 0;
		/* Connect will always be fetched since it has a default value, this check will always return true */
		if ( strncmp ( setting->name, "Connect", strlen ( "Connect" ) ) )
			driver_settings->callbacks.tlv_invalidate ( driver_settings->drv_priv,
					index, tlv_type );
	}

	if ( rc )
		return -EACCES;

	return 0;
}

static int ipv4_address_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &ipv4_add_setting,
			&conf->iscsi.initiator_params.ipv4_addr,
			sizeof ( conf->iscsi.initiator_params.ipv4_addr ),
			ISCSI_INITIATOR_IPV4_ADDR );
}

static int subnet_mask_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &subnet_mask_setting,
			&conf->iscsi.initiator_params.subnet,
			sizeof ( conf->iscsi.initiator_params.subnet ),
			ISCSI_INITIATOR_SUBNET );
}

static int ipv4_gateway_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &ipv4_gateway_setting,
			&conf->iscsi.initiator_params.gateway,
			sizeof ( conf->iscsi.initiator_params.gateway ),
			ISCSI_INITIATOR_IPV4_GATEWAY );
}

static int ipv4_dns_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &ipv4_dns_setting,
			&conf->iscsi.initiator_params.primary_dns,
			sizeof ( conf->iscsi.initiator_params.primary_dns ),
			ISCSI_INITIATOR_IPV4_PRIM_DNS );
}

static int iscsi_initiator_name_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings,  &iscsi_init_name_setting,
		&conf->iscsi.initiator_params.name,
		sizeof ( conf->iscsi.initiator_params.name ),
		ISCSI_INITIATOR_NAME );
}

static int iscsi_initiator_chapid_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &init_chapid_setting,
		&conf->iscsi.initiator_params.chap_id,
		sizeof ( conf->iscsi.initiator_params.chap_id ),
		ISCSI_INITIATOR_CHAP_ID );
}

static int iscsi_initiator_chapsec_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &init_chapsec_setting,
		&conf->iscsi.initiator_params.chap_pass,
		sizeof ( conf->iscsi.initiator_params.chap_pass ),
		ISCSI_INITIATOR_CHAP_PWD );
}


static int connect_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &connect_setting,
		&conf->iscsi.first_tgt_params.connect,
		sizeof ( conf->iscsi.first_tgt_params.connect ),
		CONNECT_FIRST_TGT );
}

static int target_ip_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &target_ip_setting,
		&conf->iscsi.first_tgt_params.ip_addr,
		sizeof ( conf->iscsi.first_tgt_params.ip_addr ),
		FIRST_TGT_IP_ADDRESS );
}

static int target_tcp_port_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &tcp_port_setting,
		&conf->iscsi.first_tgt_params.tcp_port,
		sizeof ( conf->iscsi.first_tgt_params.tcp_port ),
		FIRST_TGT_TCP_PORT );
}

static int target_boot_lun_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &boot_lun_setting,
		&conf->iscsi.first_tgt_params.lun,
		sizeof ( conf->iscsi.first_tgt_params.lun ),
		FIRST_TGT_BOOT_LUN );
}

static int target_iscsi_name_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &iscsi_target_name_setting,
		&conf->iscsi.first_tgt_params.name,
		sizeof ( conf->iscsi.first_tgt_params.name ), FIRST_TGT_ISCSI_NAME );
}


static int iscsi_target_chapid_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &target_chapid_setting,
		&conf->iscsi.first_tgt_params.chap_id,
		sizeof ( conf->iscsi.first_tgt_params.chap_id ), FIRST_TGT_CHAP_ID );
}

static int iscsi_target_chapsec_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	return iscsi_parameters_nv_store ( driver_settings, &target_chapsec_setting,
		&conf->iscsi.first_tgt_params.chap_pass,
		sizeof ( conf->iscsi.first_tgt_params.chap_pass ), FIRST_TGT_CHAP_PWD );
}

static int virt_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	char buf[255] = {0};
	uint32_t dword;
	int rc;

	DRIVER_SETTINGS_FETCH_SETTING ( settings, &virt_mode_setting );
	conf->virt_conf.virt_mode = ( buf[0] == 'S' );
	DRIVER_SETTINGS_FETCH_SETTING ( settings, &virt_num_setting );
	conf->virt_conf.num_of_vfs = strtoul ( buf, NULL, 10 );
	dword = cpu_to_be32 ( conf->virt_conf.dword );

	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
				&dword, 0, VIRTUALIZATION_TYPE, sizeof (dword) ) ) ) {
		return -EACCES;
	}
	return 0;
}

static int flexboot_menu_to_nv_store ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	uint8_t buf = 0;
	uint32_t tmp = 0;
	int rc = 0;

	generic_settings_fetch ( & ( driver_settings->generic_settings.settings ),
			&flexboot_menu_to_setting, &buf, sizeof ( buf ) );
	conf->rom_banner_to.rom_banner_to = buf;
	tmp = cpu_to_be32 ( conf->rom_banner_to.dword );

	if ( ( rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
			&tmp, 0, BANNER_TO_TYPE, sizeof (tmp) ) ) ) {
		return -EACCES;
	}

	return 0;
}

#define MAX_SETTING_STR 128
int virt_mode_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_conf_defaults *defaults = ( struct nv_conf_defaults * ) driver_settings->defaults;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		if ( defaults->sriov_en )
			generic_settings_store ( settings, setting, STR_SRIOV, strlen ( STR_SRIOV ) + 1 );
		else
			generic_settings_store ( settings, setting, STR_NONE, strlen ( STR_NONE ) + 1 );
		return STORED;
	}

	return SUCCESS;
}

#define RESTORE_ENABLED_DISABLED( defaultdata )								\
	do {																	\
		struct driver_settings *driver_settings =							\
		get_driver_settings_from_settings ( settings );						\
		struct nv_port_conf_defaults *defaults = 						\
		( struct nv_port_conf_defaults * ) driver_settings->defaults;	\
		if ( !data || !len ) {												\
			if ( defaults->defaultdata )									\
				generic_settings_store ( settings, setting,					\
						STR_ENABLE, strlen ( STR_ENABLE ) + 1 );			\
			else															\
				generic_settings_store ( settings, setting,					\
						STR_DISABLE, strlen ( STR_DISABLE ) + 1 );			\
			return STORED;													\
		}																	\
		return SUCCESS;														\
	} while ( 0 )


int virt_num_check_or_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_conf_defaults *defaults = ( struct nv_conf_defaults * ) driver_settings->defaults;
	int32_t maximum, number;
	int rc;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		number = ntohl ( defaults->total_vfs );
		generic_settings_store ( settings, setting, &number, sizeof ( number ) );
		return STORED;
	}

	number = ntohl ( * ( int32_t * ) data );
	if ( number < 0 )
		return INVALID_INPUT;

	if ( ( rc = generic_settings_fetch ( settings, &virt_num_max_setting,
						&maximum, sizeof ( maximum ) ) ) <= 0 )
		return rc;

	maximum = htonl ( maximum );
	if ( number >= maximum )
		return INVALID_INPUT;

	return SUCCESS;
}

int boot_protocol_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		/** Assuming boot protocol is 0-none, 1-pxe, 2-iscsi */
		switch ( defaults->boot_protocol ) {
		case 2:
			generic_settings_store ( settings, setting, STR_ISCSI, strlen ( STR_ISCSI ) + 1 );
			break;
		case 1:
			generic_settings_store ( settings, setting, STR_PXE, strlen ( STR_PXE ) + 1 );
			break;
		default:
			generic_settings_store ( settings, setting, STR_NONE, strlen ( STR_NONE ) + 1 );
			break;
		}
		return STORED;
	}

	return SUCCESS;
}

int virt_lan_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	RESTORE_ENABLED_DISABLED ( boot_vlan_en );
}

int virt_id_check_or_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;
	int32_t number;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		number = ntohl ( defaults->boot_vlan  );
		generic_settings_store ( settings, setting, &number, sizeof ( number ) );
		return STORED;
	}

	number = ntohl ( * ( int32_t * ) data );

	if ( number > MAX_VIRTUAL_ID || number < MIN_VIRTUAL_ID )
		return INVALID_INPUT;

	return SUCCESS;
}

int boot_pkey_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;
	int32_t number;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		number = defaults->boot_pkey;
		generic_settings_store ( settings, setting, &number, sizeof ( number ) );
		return STORED;
	}

	number = * ( int32_t * ) data;
	number = be32_to_cpu ( number );

	if ( ( number > MAX_PKEY_VAL ) || ( number < MIN_PKEY_VAL ) ) {
		return INVALID_INPUT;
	}

	return SUCCESS;
}

int opt_rom_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	RESTORE_ENABLED_DISABLED ( boot_option_rom_en );
}

int boot_retries_restore ( struct settings *settings, const void *data,
                    size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;
	const char *retries;
	/* If setting is deleted, set default */
	if ( !data || !len ) {
		retries = driver_settings_get_boot_ret_str ( defaults->boot_retry_count );
		generic_settings_store ( settings, setting, retries, 10 );
		return STORED;
	}

	return SUCCESS;
}

int wol_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
        RESTORE_ENABLED_DISABLED ( en_wol_magic );
}

int dhcp_ip_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	RESTORE_ENABLED_DISABLED( iscsi_ipv4_dhcp_en );
}

int dhcp_iscsi_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	RESTORE_ENABLED_DISABLED( iscsi_dhcp_params_en );
}

int iscsi_chap_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	RESTORE_ENABLED_DISABLED( iscsi_chap_auth_en );
}

int iscsi_mutual_chap_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	RESTORE_ENABLED_DISABLED( iscsi_chap_mutual_auth_en );
}

int iscsit_boot_to_target_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		switch ( defaults->iscsi_boot_to_target ) {
		case ISCSI_BOOT_TO_TARGET_ONE_TIME_DISABLE:
			generic_settings_store ( settings, setting, STR_ONE_TIME_DISABLED,
					strlen ( STR_ONE_TIME_DISABLED ) + 1 );
			break;
		case ISCSI_BOOT_TO_TARGET_DISABLE:
			generic_settings_store ( settings, setting, STR_DISABLE,
					strlen ( STR_DISABLE ) + 1 );
			break;
		default:
			/* Enabled in default */
			generic_settings_store ( settings, setting, STR_ENABLE,
					strlen ( STR_ENABLE ) + 1 );
			break;
		}
		return STORED;
	}

	return SUCCESS;
}

int connect_restore ( struct settings *settings, const void *data,
		size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;
	struct nv_port_conf *conf = ( struct nv_port_conf * ) driver_settings->priv_data;

	if ( !data || !len ) {
		if ( ( conf->nic.boot_conf.legacy_boot_prot == BOOT_PROTOCOL_ISCSI ) &&
		     ( defaults->iscsi_dhcp_params_en == 0 ) )
			generic_settings_store ( settings, setting, STR_ENABLE, strlen ( STR_ENABLE ) + 1 );
		else
			generic_settings_store ( settings, setting, STR_DISABLE, strlen ( STR_DISABLE ) + 1 );
		return STORED;
	}
	return SUCCESS;
}

int flexboot_menu_to_restore ( struct settings *settings, const void *data,
                    size_t len, const struct setting *setting ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	struct nv_conf_defaults *def_conf = ( struct nv_conf_defaults * ) driver_settings->defaults;
	uint8 to_val = 0;

	/* If setting is deleted, set default */
	if ( !data || !len ) {
		generic_settings_store ( settings, setting,
				& ( def_conf->flexboot_menu_to ),  sizeof( uint8_t ) );
		return STORED;
	}

	to_val = *( ( uint8* ) data);
	if (  to_val > MAX_BANNER_TO || to_val < MIN_BANNER_TO )
			return INVALID_INPUT;
	return SUCCESS;
}

static struct driver_setting_operation driver_setting_operations[] = {
	{ &virt_mode_setting,			NULL, &virt_mode_restore, &virt_nv_store },
	{ &virt_num_setting,			NULL, &virt_num_check_or_restore, &virt_nv_store },
	{ &virt_num_max_setting,		NULL, NULL, NULL },
	{ &blink_leds_setting,			NULL, NULL, NULL },
	{ &device_name_setting, 		NULL, NULL, NULL },
	{ &chip_type_setting,			NULL, NULL, NULL },
	{ &flexboot_menu_to_setting,	NULL, &flexboot_menu_to_restore, &flexboot_menu_to_nv_store },
	{ &pci_id_setting,				NULL, NULL, NULL },
	{ &bus_dev_fun_setting,			NULL, NULL, NULL },
	{ &mac_add_setting,				NULL, NULL, NULL },
	{ &phy_mac_setting,				NULL, NULL, NULL },
	{ &flex_version_setting,		NULL, NULL, NULL },
	{ &fw_version_setting,			NULL, NULL, NULL },
	{ &boot_protocol_setting, 		NULL, &boot_protocol_restore, &nic_boot_nv_store },
	{ &virt_lan_setting,			NULL, &virt_lan_restore, &nic_boot_nv_store },
	{ &virt_id_setting,				NULL, &virt_id_check_or_restore, &nic_boot_nv_store },
	{ &opt_rom_setting,				NULL, &opt_rom_restore, &nic_boot_nv_store },
	{ &boot_retries_setting,		NULL, &boot_retries_restore, &nic_boot_nv_store },
	{ &wol_setting,					NULL, NULL, &wol_nv_store },
	{ &boot_pkey_setting,			NULL, &boot_pkey_restore, &nic_ib_boot_nv_store },
	{ &dhcp_ip_setting,				NULL, &dhcp_ip_restore, &dhcp_flags_nv_store },
	{ &dhcp_iscsi_setting,			NULL, &dhcp_iscsi_restore, &dhcp_flags_nv_store },
	{ &iscsi_chap_setting,			NULL, &iscsi_chap_restore, &iscsi_gen_flags_nv_store },
	{ &iscsi_mutual_chap_setting,	NULL, &iscsi_mutual_chap_restore, &iscsi_gen_flags_nv_store },
	{ &iscsi_boot_to_target_setting, NULL, &iscsit_boot_to_target_restore, &iscsi_gen_flags_nv_store },
	{ &ip_ver_setting, 				NULL, NULL, NULL },
	{ &ipv4_add_setting,			NULL, &ipv4_address_store, &ipv4_address_nv_store },
	{ &subnet_mask_setting, 		NULL, &subnet_mask_store, &subnet_mask_nv_store },
	{ &ipv4_gateway_setting,		NULL, &ipv4_gateway_store, &ipv4_gateway_nv_store },
	{ &ipv4_dns_setting,			NULL, &ipv4_dns_store, &ipv4_dns_nv_store },
	{ &iscsi_init_name_setting,		NULL, &iscsi_initiator_name_store, &iscsi_initiator_name_nv_store },
	{ &init_chapid_setting,			NULL, &iscsi_initiator_chapid_store, &iscsi_initiator_chapid_nv_store },
	{ &init_chapsec_setting,		NULL, &iscsi_initiator_chapsec_store, &iscsi_initiator_chapsec_nv_store },
	{ &connect_setting,				NULL, &connect_restore, &connect_nv_store },
	{ &target_ip_setting,			NULL, &target_ip_store, &target_ip_nv_store },
	{ &tcp_port_setting,			NULL, &target_tcp_port_store, &target_tcp_port_nv_store },
	{ &boot_lun_setting,			NULL, &target_boot_lun_store, &target_boot_lun_nv_store },
	{ &iscsi_target_name_setting,	NULL, &target_iscsi_name_store, &target_iscsi_name_nv_store },
	{ &target_chapid_setting,		NULL, &iscsi_target_chapid_store, &iscsi_target_chapid_nv_store, },
	{ &target_chapsec_setting,		NULL, &iscsi_target_chapsec_store, &iscsi_target_chapsec_nv_store },
};

const struct setting * ipxe_iscsi_settings[] = {
		&dns_setting,
		&gateway_setting,
		&netmask_setting,
		&ip_setting,
		&initiator_iqn_setting,
		&reverse_username_setting,
		&reverse_password_setting,
		&username_setting,
		&password_setting,
		&root_path_setting,
		&uriboot_retry_delay_setting,
		&uriboot_retry_setting,
		NULL
};

struct driver_setting_operation * find_setting_ops ( const struct setting *setting ) {
	struct driver_setting_operation *ops;
	unsigned int i;

	for ( i = 0 ; i < ( sizeof ( driver_setting_operations ) /
		sizeof ( driver_setting_operations[0] ) ) ; i++ ) {
		ops = &driver_setting_operations[i];
		if ( ops->setting == setting )
			return ops;
	}

	return NULL;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

void copy_netdev_settings_to_netdev ( struct net_device *src, struct net_device *dest ) {
	char buf[MAX_STR_SETTING_BUF_SIZE];
	int i = 0;

	for ( i = 0; ipxe_iscsi_settings[i] != NULL; i++ ) {
		memset ( buf, 0, sizeof ( buf ) );
		if ( fetchf_setting ( netdev_settings ( src ), ipxe_iscsi_settings[i], NULL, NULL, buf, sizeof ( buf ) ) > 0 )
			storef_setting( netdev_settings ( dest ), ipxe_iscsi_settings[i], buf );
	}
}

static int driver_settings_store ( struct settings *settings,
		   const struct setting *setting, const void *data, size_t len ) {
	struct driver_setting_operation *op;
	struct extended_setting *ext;
	struct extended_setting *new_ext;
	struct driver_settings *driver_settings;
	unsigned int i;
	int rc = 0;

	for ( i = 0 ; i < ( sizeof ( driver_setting_operations ) /
			    sizeof ( driver_setting_operations[0] ) ) ; i++ ) {
		op = &driver_setting_operations[i];
		if ( setting_cmp ( setting, op->setting ) == 0 ) {
			if ( op->store ) {
				if ( ( rc = op->store ( settings, data, len, setting ) ) == INVALID_INPUT )
					return rc;
			}
			break;
		}
	}

	if ( rc != STORED )
		rc = generic_settings_store ( settings, setting, data, len );
	else
		rc = SUCCESS;

	/* Save setting in modified list - only for driver settings */
	driver_settings = get_driver_settings_from_settings ( settings );
	if ( rc == SUCCESS && driver_settings->store_menu ) {
		if ( ! is_modified ( &driver_settings->modified_list, setting->name ) ) {
			new_ext = malloc ( sizeof ( struct extended_setting ) );
			ext = find_extended ( setting );
			memcpy ( new_ext, ext, sizeof ( struct extended_setting ) );
			list_add ( &new_ext->modified, &driver_settings->modified_list );
		}
	}
	return rc;
}

static int driver_settings_applies ( struct settings *settings, const struct setting *setting ) {
	struct driver_setting_operation *op;
	unsigned int i, rc;

	for ( i = 0 ; i < ( sizeof ( driver_setting_operations ) /
			    sizeof ( driver_setting_operations[0] ) ) ; i++ ) {
		op = &driver_setting_operations[i];
		if ( setting_cmp ( setting, op->setting ) == 0 ) {
			if ( setting->scope == settings->default_scope) {
				if ( op->applies ) {
					if ( ( rc = op->applies ( settings ) ) )
						return APPLIES;
					else
						return DOES_NOT_APPLY;
				} else {
					return APPLIES;
				}
				break;
			}
		}
	}
	return DOES_NOT_APPLY;
}

static struct settings_operations driver_settings_operations = {
	.store = &driver_settings_store,
	.fetch = &generic_settings_fetch,
	.applies = &driver_settings_applies,
};

struct settings_operations * driver_settings_get_operations () {
	return &driver_settings_operations;
}

static int modified_has_duplicates ( struct list_head *modified_list,
                                struct extended_setting *ext ) {
	struct extended_setting *cur_extended = ext;
	list_for_each_entry_continue ( cur_extended, modified_list, modified ) {
		if ( ext->nv_store == cur_extended->nv_store )
			return 1;
	}
	return 0;
}

int driver_settings_nv_store () {
	struct driver_settings *driver_settings;
	struct driver_settings *root_settings;
	struct extended_setting *ext;
	struct extended_setting *temp;
	int rc, flag = 0;

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return -22;
	}

	/* Make sure flash access is supported */
	if ( boot_menu_get_mode () == 0 )
		return 0;

	/* Check that something was modified */
	list_for_each_entry ( driver_settings, &driver_settings_list, list ) {
		if ( ! list_empty ( & ( driver_settings->modified_list ) ) ) {
			flag = 1;
			break;
		}
	}
	if ( ! flag )
		return 0;

	root_settings = get_driver_settings_from_settings( &nv_settings_root->settings );
	if ( root_settings->callbacks.open_dev ) {
		 if ( ( rc = root_settings->callbacks.open_dev ( root_settings->drv_priv ) ) != 0 ) {
			 return rc;
		 }
	}

	list_for_each_entry ( driver_settings, &driver_settings_list, list ) {
		if ( ! list_empty ( &driver_settings->modified_list ) ) {
			list_for_each_entry_safe ( ext, temp, &driver_settings->modified_list, modified ) {

				if ( ext->nv_store && ! modified_has_duplicates ( &driver_settings->modified_list, ext ) ) {

					if ( ( rc = ext->nv_store ( driver_settings ) ) ) {
						printf ("Setting %s couldn't be saved - %s \n",
							ext->setting->name, strerror( rc ) );
					}
				}
			}
		}
	}

	if ( root_settings->callbacks.close_dev )
		root_settings->callbacks.close_dev ( root_settings->drv_priv );

	return 0;
}

static void driver_setting_port_defaults ( struct driver_settings *driver_settings ) {
	struct driver_setting_operation *op;
	struct settings *settings;
	struct extended_setting *ext;
	unsigned int i;
	int rc;

	for ( i = 0 ; i < ( sizeof ( driver_setting_operations ) /
                            sizeof ( driver_setting_operations[0] ) ) ; i++ ) {
		op = &driver_setting_operations[i];
		if ( fetch_setting_origin ( &driver_settings->generic_settings.settings, op->setting, &settings ) )
			continue;
		ext = find_extended ( op->setting );
		if ( settings && ext->type != LABEL ) {
			if ( ( rc = driver_settings_store ( settings, op->setting, NULL, 0 ) ) )
				DBG ( "Failed to store the default value of %s (rc = %d)\n",
						op->setting->name, rc );
		}
	}
}

static void driver_settings_defaults () {
	struct driver_setting_operation *op;
	struct settings *settings;
	struct extended_setting *ext;
	unsigned int i;
	int rc;

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return;
	}

	settings = &nv_settings_root->settings;

	for ( i = 0 ; i < ( sizeof ( driver_setting_operations ) /
				sizeof ( driver_setting_operations[0] ) ) ; i++ ) {
		op = &driver_setting_operations[i];
		if ( !setting_applies ( settings, op->setting ) )
			continue;
		ext = find_extended ( op->setting );
		if ( ext->type != LABEL ) {
			if ( ( rc = driver_settings_store ( settings, op->setting, NULL, 0 ) ) )
				DBG ( "Failed to store the default value of %s (rc = %d)\n",
						op->setting->name, rc );
		}
	}
}

static int get_virt_id_type ( struct settings *settings ) {
	char buf[MAX_SETTING_STR] = { 0 };
	int rc;

	rc = fetchf_setting ( settings, &virt_lan_setting, NULL, NULL, buf, MAX_SETTING_STR );
	if ( ( rc > 0 ) && ( buf[0] == 'D' ) )
		return LABEL;

	return INPUT;
}

static int get_virt_num_type ( struct settings *settings ) {
	char buffer[MAX_SETTING_STR] = { 0 };
	int rc;

	if ( setting_applies ( settings, &virt_mode_setting ) ) {
		rc = fetchf_setting ( settings, &virt_mode_setting, NULL, NULL, buffer, MAX_SETTING_STR );
		if ( ( rc > 0 ) && ( buffer[0] == 'S' ) )
			return INPUT;
	}

	return LABEL;
}

static void disable_port_settings_nv_store_aux ( const struct settings_scope * hidlist[],
							unsigned int hidlistlen, struct settings* root ) {
	unsigned int i = 0;
	struct settings *child;

	list_for_each_entry ( child, &root->children, siblings ) {
		disable_port_settings_nv_store_aux ( hidlist, hidlistlen, child );
	}

	for ( i = 0 ; i < hidlistlen ; i++ ) {
		if ( root->default_scope == hidlist[i] ) {
			root->hidden = 1;
			break;
		}
	}
}

static void disable_port_settings_nv_store ( struct driver_settings *driver_settings  ) {
	const struct settings_scope * hidlist[] = { &nic_scope, &iscsi_scope,
												&iscsi_general_scope,
												&iscsi_init_scope,
												&iscsi_target_scope };
	if ( boot_menu_get_mode () )
		return;

	disable_port_settings_nv_store_aux ( hidlist,
			( sizeof( hidlist ) / sizeof( hidlist[0] ) ),
			& ( driver_settings->generic_settings.settings ) );
}

static int driver_settings_init_port_aux ( struct driver_settings *driver_settings,
		struct settings_operations *operations, char *name ) {
	struct generic_settings *root_settings;
	struct generic_settings *main_settings = &driver_settings->generic_settings;
	struct generic_settings *nic_settings = NULL;
	struct generic_settings *iscsi_settings = NULL;
	struct generic_settings *iscsi_general_settings = NULL;
	struct generic_settings *iscsi_initiator_settings = NULL;
	struct generic_settings *iscsi_target_settings = NULL;
	unsigned int i;

	root_settings = nv_settings_root;

	struct settings_page settings_pages[] = {
		{ &main_settings, &root_settings, name, &port_scope },
		{ &nic_settings, &main_settings,"NIC Configuration", &nic_scope },
		{ &iscsi_settings, &main_settings, "iSCSI Configuration", &iscsi_scope },
		{ &iscsi_general_settings, &iscsi_settings, "iSCSI General Parameters", &iscsi_general_scope },
		{ &iscsi_initiator_settings, &iscsi_settings, "iSCSI Initiator Parameters", &iscsi_init_scope },
		{ &iscsi_target_settings, &iscsi_settings, "iSCSI First target Parameters", &iscsi_target_scope },
	};

	for ( i = 0 ; i < ( sizeof ( settings_pages ) / sizeof ( settings_pages[0] ) ) ; i++ ) {
		if ( i != 0 ) {  /* The first settings were already allocated in driver_settings */
			*settings_pages[i].new_settings = malloc (sizeof (struct generic_settings));
		}
		generic_settings_init ( *settings_pages[i].new_settings, NULL);
		register_settings (&(*settings_pages[i].new_settings)->settings, &(*settings_pages[i].parent)->settings, settings_pages[i].name );
		(*settings_pages[i].new_settings)->settings.op = operations;
		(*settings_pages[i].new_settings)->settings.default_scope = settings_pages[i].scope;
	}

	/* Add to global list of driver settings */
	list_add ( &driver_settings->list, &driver_settings_list );
	/* When finished with settings blocks, start adding info for extended settings */
	fill_extended_settings ();

	return 0;
}


#define PORT_LABEL_LEN 100
int driver_settings_init_port ( struct driver_settings *driver_settings ) {
	struct net_device *netdev = driver_settings->netdev;
	char *menu_name;
	char tmp[PORT_LABEL_LEN] = { 0 };

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return -EINVAL;
	}

	snprintf ( tmp, PORT_LABEL_LEN, "%s : Port %d - %s ", netdev->name,
			driver_settings->index, netdev_addr( netdev ) );
	menu_name = zalloc ( ( strlen ( tmp ) + 1 ) );
	if ( menu_name == NULL ) {
		printf ( "%s: Failed to allocate memory for port name\n", __FUNCTION__ );
		return -ENOMEM;
	}

	strcpy ( menu_name, tmp );

	driver_settings->callbacks.set_default = driver_setting_port_defaults;
	INIT_LIST_HEAD ( &driver_settings->modified_list );
	driver_settings->store_menu = 0;
	driver_settings_init_port_aux ( driver_settings,
			&driver_settings_operations, menu_name );

	if ( ! boot_menu_get_mode () )
		disable_port_settings_nv_store( driver_settings );

	/* Set the function pointers of the extended settings */
	ext_virt_id.get_extended_type = &get_virt_id_type;
	ext_virt_num.get_extended_type = &get_virt_num_type;

	return 0;
}

static void init_extended_nv_store () {
 	struct driver_setting_operation *op;
 	struct extended_setting *ext;
 	unsigned int i;
	unsigned int hid = ( boot_menu_get_mode () == 0 );

 	for ( i = 0 ; i < ( sizeof ( driver_setting_operations ) /
		sizeof ( driver_setting_operations[0] ) ) ; i++ ) {
		op = &driver_setting_operations[i];
		ext = find_extended ( op->setting );
		ext->nv_store = op->nv_store;
		if ( hid )
			ext->setting->hidden = ( ext->nv_store != NULL );
 	}
}

#define ROOT_SETTINGS_NAME	"System setup"
int driver_settings_init ( struct driver_settings *driver_settings ) {
	struct generic_settings *gen_settings = & ( driver_settings->generic_settings );
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct driver_settings_callbacks *callbacks = & ( driver_settings->callbacks );
	int rc;

	if ( ! nv_settings_root || ! callbacks ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return -EINVAL;
	}

	/* check if can show rw settings */
	if ( callbacks->tlv_write )
		boot_menu_set_mode ( 1 );

	if ( callbacks->set_ro_device_settings )
		callbacks->set_ro_device_settings( driver_settings->drv_priv );

	/* Get Flexboot version */
#ifdef __BASE_BUILD_VERSION__
	/* If not compiled with version number - put the default verison */
	strcpy ( conf->fw_image_props.flexboot_version, __BASE_BUILD_VERSION__ );
#else
	strcpy ( conf->fw_image_props.flexboot_version, __BUILD_VERSION__ );
#endif
	generic_settings_init ( gen_settings, NULL );

	if ( ( rc = register_settings ( & ( gen_settings->settings ),
			NULL, ROOT_SETTINGS_NAME ) ) != 0 )
		return rc;

	gen_settings->settings.op = &driver_settings_operations;
	gen_settings->settings.default_scope = &main_scope;
	callbacks->set_default = driver_settings_defaults;

	INIT_LIST_HEAD ( &driver_settings->modified_list );
	driver_settings->store_menu = 0;

	/* Add to driver settings list  */
	list_add ( &driver_settings->list, &driver_settings_list );

	init_firmware_settings ( &driver_settings_operations );
	init_extended_nv_store ();

	return 0;
}

int driver_set_device_settings ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct settings *settings = & ( driver_settings->generic_settings.settings );
	char buf[DRIVER_MAX_STR_LEN_SETTING];
	int rc = -1;

	DRIVER_STORE_STR_SETTING( driver_settings,  settings, virt_mode_setting,
                        ( conf->virt_conf.virt_mode ? STR_SRIOV : STR_NONE ) );
	DRIVER_STORE_INT_SETTING( driver_settings, settings, virt_num_setting, conf->virt_conf.num_of_vfs );
	DRIVER_STORE_INT_SETTING( driver_settings, settings, virt_num_max_setting, conf->max_num_of_vfs_supported );
	DRIVER_STORE_STR_SETTING( driver_settings, settings, device_name_setting, conf->device_name );
	DRIVER_STORE_STR_SETTING( driver_settings, settings, chip_type_setting, conf->driver_name );
	DRIVER_STORE_INT_SETTING( driver_settings, settings, pci_id_setting, conf->desc_dev_id );
	DRIVER_STORE_STR_SETTING( driver_settings,settings, bus_dev_fun_setting, conf->bdf_name );
	DRIVER_STORE_INT_SETTING( driver_settings, settings, flexboot_menu_to_setting, conf->rom_banner_to.rom_banner_to );

	return 0;
}

static int driver_settings_read_nv_settings ( struct driver_settings *driver_settings,
		uint32_t type, uint32_t type_mod, uint32_t length,
		uint32_t *version, void *data ) {
	struct driver_tlv_header tlv_hdr = { type, type_mod, length, ( version == NULL ) ? 0 : *version, data };
	int rc = 0;

	rc = driver_settings->callbacks.tlv_read( driver_settings->drv_priv, &tlv_hdr );
	if ( version )
		*version = tlv_hdr.version;

	return rc;
}


static int driver_flash_read_nic_boot_config ( struct driver_settings *driver_settings,
		unsigned int port_num ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct nv_nic_conf *nic_conf = & ( port_conf->nic );
	union nv_nic_boot_conf *nic_boot_conf = & ( nic_conf->boot_conf );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;
	int rc;

	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			BOOT_SETTINGS_TYPE, port_num, sizeof ( *nic_boot_conf ),
			NULL , nic_boot_conf ) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read the boot configurations from the flash (rc = %d)\n", rc );

		nic_boot_conf->en_option_rom	= defaults->boot_option_rom_en;
		nic_boot_conf->en_vlan			= defaults->boot_vlan_en;
		nic_boot_conf->boot_retry_count	= defaults->boot_retry_count;
		nic_boot_conf->legacy_boot_prot	= defaults->boot_protocol;
		nic_boot_conf->vlan_id			= defaults->boot_vlan;
	}

	return rc;
}

static int driver_flash_read_nic_wol_config ( struct driver_settings *driver_settings,
		unsigned int port_num ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct nv_nic_conf *nic_conf = & ( port_conf->nic );
	union nv_wol_conf *conf = & ( nic_conf->wol_conf );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;
	int rc;

	/* Read WOL configuration */
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			WAKE_ON_LAN_TYPE, port_num, sizeof ( *conf ), NULL , conf ) ) != 0 ) {
		conf->en_wol_magic = defaults->en_wol_magic;
		DBGC ( driver_settings, "Failed to read the WOL configuration from the flash (rc = %d)\n", rc );
	}
	return rc;
}

static int driver_flash_read_nic_ib_boot_config ( struct driver_settings *driver_settings,
		unsigned int port_num ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	union nv_nic_ib_boot_conf *nic_ib_boot_conf = & ( port_conf->nic.ib_boot_conf );
	struct nv_port_conf_defaults *defaults = ( struct nv_port_conf_defaults * ) driver_settings->defaults;
	int rc;

	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			IB_BOOT_SETTING_TYPE, port_num, sizeof ( *nic_ib_boot_conf ),
			NULL , nic_ib_boot_conf ) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read the InfiniBand boot configurations from the flash (rc = %d)\n", rc );
		nic_ib_boot_conf->boot_pkey	= defaults->boot_pkey;
	}

	return rc;
}

static int driver_flash_read_iscsi_initiator_ipv4_addr ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	int rc = 0;
	uint32_t length = sizeof ( initiator_params->ipv4_addr );
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_IPV4_ADDR, port_num, length, 0 , initiator_params->ipv4_addr ) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read iSCSI Initiator 'IPv4' setting (rc = %d)\n", rc );
	} else {
		initiator_params->ipv4_addr[ length - 1] = '\0';
		initiator_params->valid_mask |= ISCSI_INITIATOR_IP_ADDR_MASK;
	}
	return rc;
}

static int driver_flash_read_iscsi_initiator_subnet ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	int rc = 0;

	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_SUBNET, port_num, sizeof ( initiator_params->subnet ), NULL,
			initiator_params->subnet ) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read iSCSI Initiator 'Subnet' setting (rc = %d)\n", rc );
	} else {
		initiator_params->valid_mask |= ISCSI_INITIATOR_SUBNET_MASK;
	}

	return rc;
}

static int driver_flash_read_iscsi_initiator_ipv4_gateway ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_IPV4_GATEWAY, port_num, sizeof ( initiator_params->gateway ), NULL , initiator_params->gateway ) ) != 0 )
			DBGC ( driver_settings, "Failed to read iSCSI Initiator 'Gateway' setting (rc = %d)\n", rc );
		else
			initiator_params->valid_mask |= ISCSI_INITIATOR_GATEWAY_MASK;
	return rc;
}

static int driver_flash_read_iscsi_initiator_ipv4_prim_dns ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_IPV4_PRIM_DNS, port_num, sizeof ( initiator_params->primary_dns ), NULL , initiator_params->primary_dns ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI Initiator 'Primary DNS' setting (rc = %d)\n", rc );
	else
		initiator_params->valid_mask |= ISCSI_INITIATOR_PRIME_DNS_MASK;
	return rc;
}

static int driver_flash_read_iscsi_initiator_name ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_NAME, port_num, sizeof ( initiator_params->name ), NULL , initiator_params->name ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI Initiator 'Name' setting (rc = %d)\n", rc );
	else
		initiator_params->valid_mask |= ISCSI_INITIATOR_NAME_MASK;
	return rc;
}

static int driver_flash_read_iscsi_initiator_chap_id ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_CHAP_ID, port_num, sizeof ( initiator_params->chap_id ), NULL , initiator_params->chap_id ) ) != 0 )
			DBGC ( driver_settings, "Failed to read iSCSI Initiator 'CHAP ID' setting (rc = %d)\n", rc );
	else
			initiator_params->valid_mask |= ISCSI_INITIATOR_CHAP_ID_MASK;
	return rc;
}


static int driver_flash_read_iscsi_initiator_chap_pwd ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_CHAP_PWD, port_num, sizeof ( initiator_params->chap_pass ), NULL , initiator_params->chap_pass ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI Initiator 'CHAP Password' setting (rc = %d)\n", rc );
	else
		initiator_params->valid_mask |= ISCSI_INITIATOR_CHAP_PASS_MASK;
	return rc;
}

static int driver_flash_read_iscsi_initiator ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_initiator_params *initiator_params ) {
	driver_flash_read_iscsi_initiator_ipv4_addr ( driver_settings, port_num, initiator_params );
	driver_flash_read_iscsi_initiator_subnet ( driver_settings, port_num, initiator_params );
	driver_flash_read_iscsi_initiator_ipv4_gateway ( driver_settings, port_num, initiator_params );
	driver_flash_read_iscsi_initiator_ipv4_prim_dns ( driver_settings, port_num, initiator_params );
	driver_flash_read_iscsi_initiator_name ( driver_settings, port_num, initiator_params );
	driver_flash_read_iscsi_initiator_chap_id ( driver_settings, port_num, initiator_params );
	driver_flash_read_iscsi_initiator_chap_pwd ( driver_settings, port_num, initiator_params );
	return 0;
}

static int driver_flash_read_iscsi_first_target_connect ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			CONNECT_FIRST_TGT, port_num, sizeof ( first_tgt_params->connect ), NULL , first_tgt_params->connect ) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read iSCSI first target 'Connect' setting (rc = %d)\n", rc );
		strcpy ( first_tgt_params->connect, STR_DISABLE );
	}
	first_tgt_params->valid_mask |= ISCSI_TARGET_CONNECT_MASK;
	return rc;
}

static int driver_flash_read_iscsi_first_target_ip_addr ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	int rc = 0;
	uint32_t length = sizeof ( first_tgt_params->ip_addr );
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			FIRST_TGT_IP_ADDRESS, port_num, length, NULL , first_tgt_params->ip_addr ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI first target 'IP' setting (rc = %d)\n", rc );
			else {
				first_tgt_params->ip_addr[length - 1] = '\0';
				first_tgt_params->valid_mask |= ISCSI_TARGET_IP_ADDR_MASK;
			}
	return rc;
}

static int driver_flash_read_iscsi_first_target_tcp_port ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			FIRST_TGT_TCP_PORT, port_num, sizeof ( first_tgt_params->tcp_port ), NULL , first_tgt_params->tcp_port ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI first target 'TCP port' setting (rc = %d)\n", rc );
			else
				first_tgt_params->valid_mask |= ISCSI_TARGET_TCP_PORT_MASK;
	return rc;
}

static int driver_flash_read_iscsi_first_target_boot_lun ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			FIRST_TGT_BOOT_LUN, port_num, sizeof ( first_tgt_params->lun ), 0 , first_tgt_params->lun ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI first target 'LUN' setting (rc = %d)\n", rc );
			else
				first_tgt_params->valid_mask |= ISCSI_TARGET_LUN_MASK;
	return rc;
}

static int driver_flash_read_iscsi_first_target_iscsi_name ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			FIRST_TGT_ISCSI_NAME, port_num, sizeof ( first_tgt_params->name ), NULL , first_tgt_params->name ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI first target 'Name' setting (rc = %d)\n", rc );
			else
				first_tgt_params->valid_mask |= ISCSI_TARGET_NAME_MASK;
	return rc;
}

static int driver_flash_read_iscsi_first_target_chap_id ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			FIRST_TGT_CHAP_ID, port_num, sizeof ( first_tgt_params->chap_id ), NULL , first_tgt_params->chap_id ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI first target 'CHAP ID' setting (rc = %d)\n", rc );
	else
		first_tgt_params->valid_mask |= ISCSI_TARGET_CHAP_ID_MASK;
	return rc;
}

static int driver_flash_read_iscsi_first_target_chap_pwd ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			FIRST_TGT_CHAP_PWD, port_num, sizeof ( first_tgt_params->chap_pass ), NULL , first_tgt_params->chap_pass ) ) != 0 )
		DBGC ( driver_settings, "Failed to read iSCSI first target 'CHAP Password' setting (rc = %d)\n", rc );
	else
		first_tgt_params->valid_mask |= ISCSI_TARGET_CHAP_PASS_MASK;
	return rc;
}

static int driver_flash_read_iscsi_first_tgt ( struct driver_settings *driver_settings,
		unsigned int port_num, struct nv_iscsi_target_params *first_tgt_params ) {
	driver_flash_read_iscsi_first_target_connect ( driver_settings, port_num, first_tgt_params );
	driver_flash_read_iscsi_first_target_ip_addr ( driver_settings, port_num, first_tgt_params );
	driver_flash_read_iscsi_first_target_tcp_port ( driver_settings, port_num, first_tgt_params );
	driver_flash_read_iscsi_first_target_boot_lun ( driver_settings, port_num, first_tgt_params );
	driver_flash_read_iscsi_first_target_iscsi_name ( driver_settings, port_num, first_tgt_params );
	driver_flash_read_iscsi_first_target_chap_id ( driver_settings, port_num, first_tgt_params );
	driver_flash_read_iscsi_first_target_chap_pwd ( driver_settings, port_num, first_tgt_params );
	return 0;
}


static void driver_iscsi_init_dhcp_no_tlv ( struct driver_settings *driver_settings,
		union nv_iscsi_init_dhcp_conf *init_dhcp_conf ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct nv_iscsi_conf *iscsi_conf = & ( port_conf->iscsi );
	struct nv_iscsi_initiator_params *init_params =
			& ( iscsi_conf->initiator_params );
	struct nv_iscsi_target_params *tgt_params =
			& ( iscsi_conf->first_tgt_params );

	init_dhcp_conf->ipv4_dhcp_en = 0;
	init_dhcp_conf->dhcp_iscsi_en = 0;
	if ( ( tgt_params->connect[0] == 'D' ) ||
	     ( ( tgt_params->valid_mask & ISCSI_TARGET_CONNECT_MASK ) == 0 ) ) {
	        /* 'Connect' is disabled. Take from IP and parameters from DHCP */
			init_dhcp_conf->ipv4_dhcp_en = 1;
			init_dhcp_conf->dhcp_iscsi_en = 1;
	} else {
	        if ( ( init_params->ipv4_addr[0] == 0 ) ||
	             ( ( init_params->valid_mask & ISCSI_INITIATOR_IP_ADDR_MASK ) == 0 ) ) {
	                /* No Initiator's IP found. Go to DHCP */
	        		init_dhcp_conf->ipv4_dhcp_en = 1;
	        }
	}

	return;
}

static void driver_flash_read_iscsi_init_dhcp( struct driver_settings *driver_settings,
		unsigned int port_num, union nv_iscsi_init_dhcp_conf *init_dhcp_conf ) {
	int rc = 0;
	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_INITIATOR_DHCP_CONF_TYPE, port_num, sizeof ( *init_dhcp_conf ), NULL , init_dhcp_conf ) ) != 0 ) {
			DBGC ( driver_settings, "Failed to read iSCSI initiator DHCP settings (rc = %d)."
					" Using default behavior.\n", rc );
			driver_iscsi_init_dhcp_no_tlv ( driver_settings, init_dhcp_conf );
		}
	return;
}

static void driver_flash_read_iscsi_general ( struct driver_settings *driver_settings,
		unsigned int port_num,union nv_iscsi_general *gen_conf ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct nv_iscsi_conf *iscsi_conf = & ( port_conf->iscsi );
	struct nv_iscsi_target_params *target =
			& ( iscsi_conf->first_tgt_params );
	struct nv_iscsi_initiator_params *initiator =
			& ( iscsi_conf->initiator_params );
	struct nv_port_conf_defaults *defaults =
			( struct nv_port_conf_defaults * ) & ( driver_settings->defaults );
	int rc = 0;
	uint32_t version = 0;
	union nv_iscsi_general swapped;

	rc = driver_settings_read_nv_settings ( driver_settings,
			ISCSI_GENERAL_SETTINGS_TYPE, port_num, sizeof ( *gen_conf ), &version , gen_conf );
	if ( rc || ( !rc && ( version == 0 ) ) ) {
		if ( rc ) {
			DBGC ( driver_settings, "Failed to read iSCSI general settings (rc = %d)\n", rc );
			gen_conf->chap_mutual_auth_en	= defaults->iscsi_chap_mutual_auth_en;
			gen_conf->chap_auth_en			= defaults->iscsi_chap_auth_en;
		} else {
			DBGC ( driver_settings, "Found iSCSI general settings TLV version 0 \n");
			/* TLV version is 0 - Use default behavior */
			gen_conf->chap_auth_en = ( target->chap_id[0] || target->chap_pass[0] );
			/* TODO: Verify that mutual chap depends on the regular chap */
			gen_conf->chap_mutual_auth_en = ( initiator->chap_id[0] || initiator->chap_pass[0] );
		}

		gen_conf->boot_to_target 		= defaults->iscsi_boot_to_target;
		gen_conf->vlan_en				= defaults->iscsi_vlan_en;
		gen_conf->tcp_timestamps_en		= defaults->iscsi_tcp_timestamps_en;
		gen_conf->lun_busy_retry_count	= defaults->iscsi_lun_busy_retry_count;
		gen_conf->link_up_delay_time	= defaults->iscsi_link_up_delay_time;
		gen_conf->boot_to_target		= defaults->iscsi_boot_to_target;
	}

	if ( ! boot_post_shell &&
		 ( gen_conf->boot_to_target == ISCSI_BOOT_TO_TARGET_ONE_TIME_DISABLE ) ) {
		gen_conf->boot_to_target = ISCSI_BOOT_TO_TARGET_ENABLE;
		swapped.dword[0] = cpu_to_be32 ( gen_conf->dword[0] );
		swapped.dword[1] = cpu_to_be32 ( gen_conf->dword[1] );
		swapped.dword[2] = cpu_to_be32 ( gen_conf->dword[2] );

		rc = driver_settings->callbacks.tlv_write ( driver_settings->drv_priv,
							&swapped, driver_settings->index,
							ISCSI_GENERAL_SETTINGS_TYPE, sizeof ( swapped ) );
		gen_conf->boot_to_target = ISCSI_BOOT_TO_TARGET_ONE_TIME_DISABLE;
		if ( rc ) {
			DBGC ( driver_settings, "Failed to update boot_to_target (rc = %d)\n", rc );
			return;
		}
	}
}

static int driver_flash_read_iscsi_config( struct driver_settings *driver_settings,
		unsigned int port_num ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct nv_iscsi_conf *iscsi_conf = & ( port_conf->iscsi );
	/* Read Initiator settings */
	driver_flash_read_iscsi_initiator ( driver_settings, port_num, & ( iscsi_conf->initiator_params ) );
	/* Read first target settings */
	driver_flash_read_iscsi_first_tgt ( driver_settings, port_num, & ( iscsi_conf->first_tgt_params ) );
	/* Read General settings - Needs to be after the above two functions
	 * 'Connect' parameter from target
	 * 'IP' parameter from initiator
	 */
	driver_flash_read_iscsi_init_dhcp ( driver_settings, port_num, & ( iscsi_conf->init_dhcp_conf ) );
	driver_flash_read_iscsi_general (  driver_settings, port_num, & ( iscsi_conf->gen_conf ) );
	return 0;
}

static int driver_flash_read_general_settings ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *conf =  ( struct nv_port_conf * ) driver_settings->priv_data;

	/* Default blink LEDs */
	conf->blink_leds = 0;

	return 0;
}

static int driver_set_nic_settings ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct nv_nic_conf *nic_conf = & ( port_conf->nic );
	struct settings *menu_settings = & ( driver_settings->generic_settings.settings );
	union nv_nic_boot_conf *conf = & ( nic_conf->boot_conf );
	union nv_nic_ib_boot_conf *ib_conf = & ( nic_conf->ib_boot_conf );
	struct settings *origin;
	char buf[DRIVER_MAX_STR_LEN_SETTING];
	int rc;
	rc = fetch_setting_origin ( menu_settings, &boot_protocol_setting, &origin );
	if ( rc || ( !rc && !origin ) ) {
		DBGC ( driver_settings, "Failed to find NIC configuration settings block\n" );
		return rc;
	}

	memset ( buf, 0, sizeof ( buf ) );
	/* Store legacy boot protocol - NONE/PXE/iSCSI */
	if ( conf->legacy_boot_prot == BOOT_PROTOCOL_PXE )
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_PXE );
	else if (conf->legacy_boot_prot == BOOT_PROTOCOL_ISCSI )
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_ISCSI );
	else
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_NONE );

	DRIVER_STORE_SETTING( driver_settings, origin, boot_protocol_setting, buf );
	DRIVER_STORE_STR_SETTING( driver_settings, origin, virt_lan_setting, ( conf->en_vlan ? STR_ENABLE : STR_DISABLE ) );
	DRIVER_STORE_INT_SETTING( driver_settings, origin, virt_id_setting, conf->vlan_id );
	DRIVER_STORE_STR_SETTING( driver_settings, origin, opt_rom_setting, ( conf->en_option_rom ? STR_ENABLE : STR_DISABLE ) );
	DRIVER_STORE_INT_SETTING( driver_settings, origin, boot_pkey_setting, ib_conf->boot_pkey );

	/* Find boot retry string to save according to boot retry number */
	snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s",
			driver_settings_get_boot_ret_str ( conf->boot_retry_count ) );
	DRIVER_STORE_SETTING( driver_settings, origin, boot_retries_setting, buf );
	DRIVER_STORE_STR_SETTING( driver_settings, origin, wol_setting, ( nic_conf->wol_conf.en_wol_magic ? STR_ENABLE : STR_DISABLE ) );
	return 0;
}

static int driver_set_iscsi_settings ( struct driver_settings *driver_settings ) {
	struct nv_port_conf *port_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct nv_iscsi_conf *iscsi_conf = & ( port_conf->iscsi );
	struct settings *settings;
	struct settings *main_settings;
	struct nv_iscsi_initiator_params *initiator = &( iscsi_conf->initiator_params );
	struct nv_iscsi_target_params *first_tgt = &( iscsi_conf->first_tgt_params );
	union nv_iscsi_init_dhcp_conf *init_dhcp_conf = &( iscsi_conf->init_dhcp_conf );
	union nv_iscsi_general *gen_conf = &( iscsi_conf->gen_conf );
	char buf[DRIVER_MAX_STR_LEN_SETTING];
	struct in_addr ip_addr;
	int rc;

	main_settings = & ( driver_settings->generic_settings.settings );
	/* Store general settings */
	rc = fetch_setting_origin ( main_settings, &ip_ver_setting, &settings );
        if ( rc || ( !rc && !settings ) ) {
                DBGC ( driver_settings, "Failed to find the iSCSI general settings block\n" );
                return rc;
        }

	/* Currently only IPv4 is supported */
	DRIVER_STORE_SETTING( driver_settings, settings, ip_ver_setting, STR_IPV4 );
	DRIVER_STORE_SETTING( driver_settings, settings, dhcp_ip_setting,
			( init_dhcp_conf->ipv4_dhcp_en ? STR_ENABLE : STR_DISABLE ) );
	DRIVER_STORE_SETTING( driver_settings, settings, dhcp_iscsi_setting,
			( init_dhcp_conf->dhcp_iscsi_en ? STR_ENABLE : STR_DISABLE ) );
	DRIVER_STORE_SETTING( driver_settings, settings, iscsi_chap_setting,
			( gen_conf->chap_auth_en ? STR_ENABLE : STR_DISABLE ) );
	DRIVER_STORE_SETTING( driver_settings, settings, iscsi_mutual_chap_setting,
			( gen_conf->chap_mutual_auth_en ? STR_ENABLE : STR_DISABLE ) );

	memset ( buf, 0, sizeof ( buf ) );
	if ( gen_conf->boot_to_target == ISCSI_BOOT_TO_TARGET_ENABLE )
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_ENABLE );
	else if ( gen_conf->boot_to_target == ISCSI_BOOT_TO_TARGET_DISABLE )
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_DISABLE );
	else
		snprintf ( buf, DRIVER_MAX_STR_LEN_SETTING, "%s", STR_ONE_TIME_DISABLED );

	DRIVER_STORE_SETTING( driver_settings, settings,
			iscsi_boot_to_target_setting, buf );

	/* Store initiator's settings */
	rc = fetch_setting_origin ( main_settings, &ipv4_add_setting, &settings );
	if ( rc || ( !rc && !settings ) ) {
		DBGC ( driver_settings, "Failed to find the iSCSI initiator's settings block\n" );
		return rc;
	}

	DRIVER_ISCSI_STORE_IP_TO_SETTING( driver_settings, ipv4_add_setting, initiator->valid_mask,
		ISCSI_INITIATOR_IP_ADDR_MASK, initiator->ipv4_addr );
	DRIVER_ISCSI_STORE_IP_TO_SETTING( driver_settings, subnet_mask_setting, initiator->valid_mask,
		ISCSI_INITIATOR_SUBNET_MASK, initiator->subnet );
	DRIVER_ISCSI_STORE_IP_TO_SETTING( driver_settings, ipv4_gateway_setting, initiator->valid_mask,
		ISCSI_INITIATOR_GATEWAY_MASK, initiator->gateway );
	DRIVER_ISCSI_STORE_IP_TO_SETTING( driver_settings, ipv4_dns_setting, initiator->valid_mask,
		ISCSI_INITIATOR_PRIME_DNS_MASK, initiator->primary_dns );
	DRIVER_ISCSI_STORE_STR_TO_SETTING ( driver_settings, iscsi_init_name_setting, initiator->valid_mask,
		ISCSI_INITIATOR_NAME_MASK, initiator->name );
	DRIVER_ISCSI_STORE_STR_TO_SETTING ( driver_settings, init_chapid_setting, initiator->valid_mask,
		ISCSI_INITIATOR_CHAP_ID_MASK, initiator->chap_id );
	DRIVER_ISCSI_STORE_STR_TO_SETTING ( driver_settings, init_chapsec_setting, initiator->valid_mask,
		ISCSI_INITIATOR_CHAP_PASS_MASK, initiator->chap_pass );

	/* Set target's settings */
	rc = fetch_setting_origin ( main_settings, &connect_setting, &settings );
	if ( rc || ( !rc && !settings ) ) {
		DBGC ( driver_settings, "Failed to find the iSCSI initiator's settings block\n" );
		return rc;
	}

	DRIVER_ISCSI_STORE_STR_TO_SETTING( driver_settings, connect_setting, first_tgt->valid_mask,
		ISCSI_TARGET_CONNECT_MASK, first_tgt->connect );
	DRIVER_ISCSI_STORE_IP_TO_SETTING( driver_settings, target_ip_setting, first_tgt->valid_mask,
		ISCSI_TARGET_IP_ADDR_MASK, first_tgt->ip_addr );
	DRIVER_ISCSI_STORE_STR_TO_SETTING( driver_settings, tcp_port_setting, first_tgt->valid_mask,
		ISCSI_TARGET_TCP_PORT_MASK, first_tgt->tcp_port );
	DRIVER_ISCSI_STORE_STR_TO_SETTING( driver_settings, boot_lun_setting, first_tgt->valid_mask,
		ISCSI_TARGET_LUN_MASK, first_tgt->lun );
	DRIVER_ISCSI_STORE_STR_TO_SETTING( driver_settings, iscsi_target_name_setting, first_tgt->valid_mask,
		ISCSI_TARGET_NAME_MASK, first_tgt->name );
	DRIVER_ISCSI_STORE_STR_TO_SETTING( driver_settings, target_chapid_setting, first_tgt->valid_mask,
		ISCSI_TARGET_CHAP_ID_MASK, first_tgt->chap_id );
	DRIVER_ISCSI_STORE_STR_TO_SETTING( driver_settings, target_chapsec_setting, first_tgt->valid_mask,
		ISCSI_TARGET_CHAP_PASS_MASK, first_tgt->chap_pass );

	return 0;
}
static int driver_set_general_settings ( struct driver_settings *driver_settings ) {
	struct settings *menu_settings = & ( driver_settings->generic_settings.settings );
	struct nv_port_conf *port_nv_conf = ( struct nv_port_conf * ) driver_settings->priv_data;
	struct net_device *netdev =  driver_settings->netdev;
	char buf[DRIVER_MAX_STR_LEN_SETTING];
	int rc = 0;

	/* Stored to generics to avoid saving it to flash */
	generic_settings_store ( menu_settings, &blink_leds_setting,
			&port_nv_conf->blink_leds, sizeof ( port_nv_conf->blink_leds ) );
	DRIVER_STORE_STR_SETTING( driver_settings, menu_settings, mac_add_setting,
			netdev_addr ( netdev ) );
	DRIVER_STORE_STR_SETTING( driver_settings, menu_settings, phy_mac_setting,
			netdev->ll_protocol->ntoa( port_nv_conf->phys_mac ) );

	return rc;
}

int driver_settings_get_port_nvdata ( struct driver_settings *driver_settings,
		unsigned int port_num ) {
	int rc;
	/* Read NIC configuration from NV memory */
	if ( ( rc = driver_flash_read_nic_boot_config ( driver_settings , port_num ) ) ) {
		DBGC ( driver_settings, "Failed to read NIC configuration from NV memory (rc = %d)\n", rc );
	}

	/* Read WOL configuration from NV memory */
	if ( ( rc = driver_flash_read_nic_wol_config ( driver_settings, port_num ) ) ) {
		DBGC ( driver_settings, "Failed to read WOL configuration from NV memory (rc = %d)\n", rc );
	}

	/* Read Infiniband boot configuration from NV memory */
	if ( ( rc = driver_flash_read_nic_ib_boot_config ( driver_settings, port_num ) ) ) {
		DBGC ( driver_settings, "Failed to read NIC ib configuration from NV memory (rc = %d)\n", rc );
	}

	/* Read iSCSI configuration from NV memory */
	if ( ( rc = driver_flash_read_iscsi_config ( driver_settings, port_num) ) ) {
		DBGC ( driver_settings, "Failed to read iSCSI configuration from NV memory (rc = %d)\n", rc );
	}

	/* Read general settings */
	if ( ( rc = driver_flash_read_general_settings ( driver_settings ) ) ) {
		DBGC ( driver_settings, "Failed to read general settings from NV memory (rc = %d)\n", rc );
	}

	return 0;
}

int driver_register_port_nv_settings ( struct driver_settings *driver_settings ) {
	int rc;

	/* Register NIC configuration */
	if ( ( rc = driver_set_nic_settings ( driver_settings ) ) ) {
		DBGC ( driver_settings, "Failed to register NIC settings (rc = %d)\n", rc );
	}

	/* Register iSCSI configuration */
	if ( ( rc = driver_set_iscsi_settings ( driver_settings ) ) ) {
		DBGC ( driver_settings, "Failed to register iSCSI settings (rc = %d)\n", rc );
	}

	/* Register general configuration */
	if ( ( rc = driver_set_general_settings ( driver_settings ) ) ) {
		DBGC ( driver_settings, "Failed to register general settings (rc = %d)\n", rc );
	}

	return 0;
}

static int driver_flash_read_virtualization_settings ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct nv_conf_defaults *defaults = ( struct nv_conf_defaults * ) driver_settings->defaults;
	int rc = 0;

	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			VIRTUALIZATION_TYPE, 0, sizeof ( conf->virt_conf.dword ), 0,
			& conf->virt_conf.dword ) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read the virtualization configuration from the flash (rc = %d)\n", rc );
		conf->virt_conf.num_of_vfs      = defaults->total_vfs;
		conf->virt_conf.virt_mode       = defaults->sriov_en;
	}
	return rc;
}

static int driver_flash_read_banner_to_settings ( struct driver_settings *driver_settings ) {
	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct nv_conf_defaults *defaults = ( struct nv_conf_defaults * ) driver_settings->defaults;
	int rc = 0;

	if ( ( rc = driver_settings_read_nv_settings ( driver_settings,
			BANNER_TO_TYPE, 0, sizeof ( conf->rom_banner_to.dword ), 0, & conf->rom_banner_to.dword ) ) != 0 ) {
		DBGC ( driver_settings, "Failed to read the rom banner timeout configuration from the flash (rc = %d)\n", rc );
		conf->rom_banner_to.rom_banner_to = defaults->flexboot_menu_to;
	}

	return rc;
}

static int driver_set_fw_settings (  struct driver_settings *driver_settings ) {

	struct nv_conf *conf = ( struct nv_conf * ) driver_settings->priv_data;
	struct firmware_image_props *props = & ( conf->fw_image_props );
	struct settings *menu_settings = & ( driver_settings->generic_settings.settings );
	struct settings *origin;
	char buf[DRIVER_MAX_STR_LEN_SETTING];
	int rc;

	rc = fetch_setting_origin ( menu_settings, &fw_version_setting, &origin );
	if ( rc || ( !rc && !origin ) ) {
		DBGC ( driver_settings, "Failed to find FW image properties settings block\n" );
		return rc;
	}

	DRIVER_STORE_STR_SETTING( driver_settings, origin, fw_version_setting, props->family_fw_version );
	DRIVER_STORE_STR_SETTING( driver_settings, origin, flex_version_setting, props->flexboot_version );

	return 0;
}

int driver_settings_get_nvdata ( struct driver_settings *driver_settings ) {
	/* Read virtualization configuration */
	driver_flash_read_virtualization_settings ( driver_settings );
	/* Read Banner timeout settings */
	driver_flash_read_banner_to_settings ( driver_settings );
	return 0;
}

int driver_register_nv_settings ( struct driver_settings *driver_settings ) {
	int rc = 0;

	/* Set general settings */
	driver_set_device_settings ( driver_settings );
	/* Firmware settings were already been saved */
	if ( ( rc = driver_set_fw_settings ( driver_settings ) ) )
		DBGC ( driver_settings, "Failed to register FW settings\n" );
	return rc;
}

int driver_settings_get_nv_boot_en ( void *priv_data, unsigned int port,
		u8* boot_enable, tlv_read_fn read_tlv_fn,
		struct nv_port_conf_defaults  *defaults ) {
	struct driver_tlv_header tlv;
	union nv_nic_boot_conf conf;
	int rc;
	u8  option_rom_en;
	u8  legacy_boot_protocol;

	memset ( &conf, 0, sizeof ( conf ) );
	memset ( &tlv, 0, sizeof ( tlv ) );
	tlv.type		= BOOT_SETTINGS_TYPE;
	tlv.type_mod	= port;
	tlv.length		= sizeof ( conf );
	tlv.data		= ( void* ) &conf;

	if ( ( rc = read_tlv_fn ( priv_data, &tlv ) ) ) {
		DBGC ( priv_data, "Failed to read the boot TLV from the flash (rc = %d)\n", rc );
		/* This failure could be due to missing TLV - return the default value */
		option_rom_en		= defaults->boot_option_rom_en;
		legacy_boot_protocol	= defaults->boot_protocol;
	} else {
		option_rom_en		= conf.en_option_rom;
		legacy_boot_protocol	= conf.legacy_boot_prot;
	}

	if ( ( legacy_boot_protocol != 0 ) && ( option_rom_en != 0 ) )
		*boot_enable = 1;
	else
		*boot_enable = 0;

	return 0;
}

int driver_settings_get_tlv_version ( uint32_t tlv_type ) {
	int version = 0;

	switch ( tlv_type ) {
	case ISCSI_GENERAL_SETTINGS_TYPE:
		version = 1;
		break;
	default:
		version = 0;
		break;
	}
	return version;
}
