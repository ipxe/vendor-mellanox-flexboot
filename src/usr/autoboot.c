/*
 * Copyright (C) 2006 Michael Brown <mbrown@fensystems.co.uk>.
 * Copyright (C) 2008-2015 Mellanox Technologies Ltd.
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
 *
 * You can also choose to distribute this program under the terms of
 * the Unmodified Binary Distribution Licence (as given in the file
 * COPYING.UBDL), provided that you have satisfied its requirements.
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <ipxe/netdevice.h>
#include <ipxe/dhcp.h>
#include <ipxe/settings.h>
#include <ipxe/image.h>
#include <ipxe/sanboot.h>
#include <ipxe/uri.h>
#include <ipxe/open.h>
#include <ipxe/init.h>
#include <ipxe/keys.h>
#include <ipxe/version.h>
#include <ipxe/shell.h>
#include <ipxe/features.h>
#include <ipxe/image.h>
#include <ipxe/timer.h>
#include <ipxe/vlan.h>
#include <usr/ifmgmt.h>
#include <usr/route.h>
#include <usr/imgmgmt.h>
#include <usr/prompt.h>
#include <ipxe/console.h>
#include <usr/autoboot.h>
#include <config/general.h>
#include <ipxe/boot_menu_ui.h>
#include <ipxe/driver_settings.h>
#include <ipxe/iscsi.h>
#include <ipxe/status_updater.h>
#include <config/branding.h>

/** @file
 *
 * Automatic booting
 *
 */

/** Flag to keep net device open in case we want to preserve the iscsi hook */
static uint8_t keep_netdev_open = 0;

/** Link-layer address of preferred autoboot device, if known */
static uint8_t autoboot_ll_addr[MAX_LL_ADDR_LEN];

/** Device location of preferred autoboot device, if known */
static struct device_description autoboot_desc;

/** Autoboot device tester */
static int ( * is_autoboot_device ) ( struct net_device *netdev );

/* Disambiguate the various error causes */
#define ENOENT_BOOT __einfo_error ( EINFO_ENOENT_BOOT )
#define EINFO_ENOENT_BOOT \
	__einfo_uniqify ( EINFO_ENOENT, 0x01, "Nothing to boot" )

#define NORMAL	"\033[0m"
#define BOLD	"\033[1m"
#define CYAN	"\033[36m"

/** The "scriptlet" setting */
const struct setting scriptlet_setting __setting ( SETTING_MISC, scriptlet ) = {
	.name = "scriptlet",
	.description = "Boot scriptlet",
	.tag = DHCP_EB_SCRIPTLET,
	.type = &setting_type_string,
};

/**
 * Perform PXE menu boot when PXE stack is not available
 */
__weak int pxe_menu_boot ( struct net_device *netdev __unused ) {
	return -ENOTSUP;
}

/** The "keep-san" setting */
const struct setting keep_san_setting __setting ( SETTING_SANBOOT_EXTRA,
						  keep-san ) = {
	.name = "keep-san",
	.description = "Preserve SAN connection",
	.tag = DHCP_EB_KEEP_SAN,
	.type = &setting_type_int8,
};

/** The "skip-san-boot" setting */
const struct setting skip_san_boot_setting __setting ( SETTING_SANBOOT_EXTRA,
						       skip-san-boot ) = {
	.name = "skip-san-boot",
	.description = "Do not boot from SAN device",
	.tag = DHCP_EB_SKIP_SAN_BOOT,
	.type = &setting_type_int8,
};

/**
 * Boot from filename and root-path URIs
 *
 * @v filename		Filename
 * @v root_path		Root path
 * @v drive		SAN drive (if applicable)
 * @v flags		Boot action flags
 * @ret rc		Return status code
 *
 * The somewhat tortuous flow of control in this function exists in
 * order to ensure that the "sanboot" command remains identical in
 * function to a SAN boot via a DHCP-specified root path, and to
 * provide backwards compatibility for the "keep-san" and
 * "skip-san-boot" options.
 */
int uriboot ( struct uri *filename, struct uri *root_path, int drive,
	      unsigned int flags ) {
	struct image *image;
	int iscsi_hook = 0, iscsi_describe = 0;
	int rc;

	/* Hook SAN device, if applicable */
	if ( root_path ) {
		if ( ( rc = san_hook ( root_path, drive ) ) != 0 ) {
			printf ( "Could not open SAN device: %s\n",
				 strerror ( rc ) );
			if ( filename )
				goto pxe_label;
			goto err_san_hook;
		}
		iscsi_hook = 1;
		printf ( "Registered SAN device %#02x\n", drive );
	}

	/* Describe SAN device, if applicable */
	if ( ( drive >= 0 ) && ! ( flags & URIBOOT_NO_SAN_DESCRIBE ) ) {
		if ( ( rc = san_describe ( drive ) ) != 0 ) {
			printf ( "Could not describe SAN device %#02x: %s\n",
				 drive, strerror ( rc ) );
			if ( filename )
				goto pxe_label;
			goto err_san_describe;
		}
		iscsi_describe = 1;
	}

	/* Allow a root-path-only boot with skip-san enabled to succeed */
	rc = 0;

	/* Check boot priority ISCSI vs PXE */
	if( ( ! ( flags & URIBOOT_NO_SAN_BOOT ) ) && ( flags & URIBOOT_SAN_BOOT_PRIO ) )
		goto iscsi_label;

 pxe_label:
	/* Attempt filename boot if applicable */
	if ( filename ) {
		status_update ( STATUS_UPDATE_PXE_BOOT_START );
		if ( ( rc = imgdownload ( filename, 0, &image ) ) == 0 ) {
			imgstat ( image );
			image->flags |= IMAGE_AUTO_UNREGISTER;
			if ( ( rc = image_exec ( image ) ) != 0 ) {
				printf ( "Could not boot image: %s\n",
					 strerror ( rc ) );
				/* Fall through to (possibly) attempt a SAN boot
				 * as a fallback.  If no SAN boot is attempted,
				 * our status will become the return status.
				 */
			} else {
				/* Always print an extra newline, because we
				 * don't know where the NBP may have left the
				 * cursor.
				 */
				printf ( "\n" );
			}
			if ( ! ipxe_is_started () ) {
				status_update ( STATUS_UPDATE_PXE_BOOT_END );
				goto uriboot_done;
			}
		}
		status_update ( STATUS_UPDATE_PXE_BOOT_END );
	}

	if ( flags & URIBOOT_SAN_BOOT_PRIO )
		goto err_download;

 iscsi_label:
	/* Attempt SAN boot if applicable */
	if ( iscsi_describe && ( drive >= 0 ) && ! ( flags & URIBOOT_NO_SAN_BOOT ) ) {
		if ( fetch_intz_setting ( NULL, &skip_san_boot_setting) == 0 ) {
			printf ( "Booting from SAN device %#02x\n", drive );
			rc = san_boot ( drive );
			printf ( "Boot from SAN device %#02x failed: %s\n",
				 drive, strerror ( rc ) );
		} else {
			printf ( "Skipping boot from SAN device %#02x\n",
				 drive );
			/* Avoid overwriting a possible failure status
			 * from a filename boot.
			 */
		}
	}

	if ( flags & URIBOOT_SAN_BOOT_PRIO )
		goto pxe_label;

 err_download:
 err_san_describe:
	/* Unhook SAN device, if applicable */
	if ( iscsi_hook && ( drive >= 0 ) ) {
		if ( ( fetch_intz_setting ( NULL, &keep_san_setting ) == 0 ) &&
		     ! ( flags & URIBOOT_NO_SAN_UNHOOK ) ) {
			san_unhook ( drive );
			printf ( "Unregistered SAN device %#02x\n", drive );
		} else {
			printf ( "Preserving SAN device %#02x\n", drive );
			keep_netdev_open = 1;
		}
	}
 err_san_hook:
 uriboot_done:
	return rc;
}

/**
 * Fetch next-server and filename settings into a URI
 *
 * @v settings		Settings block
 * @ret uri		URI, or NULL on failure
 */
struct uri * fetch_next_server_and_filename ( struct settings *settings ) {
	union {
		struct sockaddr sa;
		struct sockaddr_in sin;
	} next_server;
	char *raw_filename = NULL;
	struct uri *uri = NULL;
	char *filename;

	/* Initialise server address */
	memset ( &next_server, 0, sizeof ( next_server ) );

	/* If we have a filename, fetch it along with the next-server
	 * setting from the same settings block.
	 */
	if ( fetch_setting ( settings, &filename_setting, &settings,
			     NULL, NULL, 0 ) >= 0 ) {
		fetch_string_setting_copy ( settings, &filename_setting,
					    &raw_filename );
		fetch_ipv4_setting ( settings, &next_server_setting,
				     &next_server.sin.sin_addr );
	}
	if ( ! raw_filename )
		goto err_fetch;

	/* Populate server address */
	if ( next_server.sin.sin_addr.s_addr ) {
		next_server.sin.sin_family = AF_INET;
		printf ( "Next server: %s\n",
			 inet_ntoa ( next_server.sin.sin_addr ) );
	}

	/* Expand filename setting */
	filename = expand_settings ( raw_filename );
	if ( ! filename )
		goto err_expand;
	if ( filename[0] )
		printf ( "Filename: %s\n", filename );

	/* Construct URI */
	uri = pxe_uri ( &next_server.sa, filename );
	if ( ! uri )
		goto err_parse;

 err_parse:
	free ( filename );
 err_expand:
	free ( raw_filename );
 err_fetch:
	return uri;
}

/**
 * Fetch root-path setting into a URI
 *
 * @v settings		Settings block
 * @ret uri		URI, or NULL on failure
 */
static struct uri * fetch_root_path ( struct settings *settings ) {
	struct uri *uri = NULL;
	char *raw_root_path;
	char *root_path;

	/* Fetch root-path setting */
	fetch_string_setting_copy ( settings, &root_path_setting,
				    &raw_root_path );
	if ( ! raw_root_path )
		goto err_fetch;

	/* Expand filename setting */
	root_path = expand_settings ( raw_root_path );
	if ( ! root_path )
		goto err_expand;

	/* Parse root path */
	if ( root_path[0] )
		printf ( "Root path: %s\n", root_path );
	uri = parse_uri ( root_path );
	if ( ! uri )
		goto err_parse;

 err_parse:
	free ( root_path );
 err_expand:
	free ( raw_root_path );
 err_fetch:
	return uri;
}

/**
 * Check whether or not we have a usable PXE menu
 *
 * @ret have_menu	A usable PXE menu is present
 */
static int have_pxe_menu ( void ) {
	struct setting vendor_class_id_setting
		= { .tag = DHCP_VENDOR_CLASS_ID };
	struct setting pxe_discovery_control_setting
		= { .tag = DHCP_PXE_DISCOVERY_CONTROL };
	struct setting pxe_boot_menu_setting
		= { .tag = DHCP_PXE_BOOT_MENU };
	char buf[ 10 /* "PXEClient" + NUL */ ];
	unsigned int pxe_discovery_control;

	fetch_string_setting ( NULL, &vendor_class_id_setting,
			       buf, sizeof ( buf ) );
	pxe_discovery_control =
		fetch_uintz_setting ( NULL, &pxe_discovery_control_setting );

	return ( ( strcmp ( buf, "PXEClient" ) == 0 ) &&
		 setting_exists ( NULL, &pxe_boot_menu_setting ) &&
		 ( ! ( ( pxe_discovery_control & PXEBS_SKIP ) &&
		       setting_exists ( NULL, &filename_setting ) ) ) );
}

struct netdev_to_driver_iscsi_setting {
	const struct setting *netdev_setting;
	const struct setting *driver_setting;
};

void iscsi_settings_from_driver_settings ( struct settings *netdev_settings,
		struct settings *driver_settings ) {
	struct settings *sub_settings;
	char buf[255] = {0};
	int i, clear = 0;
	struct netdev_to_driver_iscsi_setting iscsi_table[] = {
		{ &initiator_iqn_setting, &iscsi_init_name_setting },
		{ NULL, NULL }
	};

	if ( ( ! netdev_settings ) || ( ! driver_settings ) )
		return;

	fetchf_setting ( driver_settings, &connect_setting, NULL, NULL, buf, sizeof ( buf ) );
        if ( buf[0] != 'E' )
		clear = 1;

	for ( i = 0 ; iscsi_table[i].driver_setting ; i++ ) {
		storef_setting ( netdev_settings, iscsi_table[i].netdev_setting, NULL );
		/* Clear the setting from DHCP/pxebs/proxyDHCP if it exists */
		list_for_each_entry ( sub_settings, &netdev_settings->children, siblings ) {
			storef_setting ( sub_settings, iscsi_table[i].netdev_setting, NULL );
		}

		if ( clear == 0 && fetchf_setting ( driver_settings, iscsi_table[i].driver_setting,
						NULL, NULL, buf, sizeof ( buf ) ) > 0 )
			storef_setting ( netdev_settings, iscsi_table[i].netdev_setting, buf );
	}

	root_path_store ( netdev_settings, driver_settings );
}

#define TOTAL_BOOT_RETRIES	7

/**
 * According to errno.h we need the POSIX value.
 **/
#define POSIX_ERROR_OFFSET	24
inline int errno_cmp ( int errno1, int errno2 ) {
	return ((errno1 >> POSIX_ERROR_OFFSET) == (errno2 >> POSIX_ERROR_OFFSET));
}

/**
 * Boot from a network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
int netboot ( struct net_device *netdev ) {
	struct uri *filename;
	struct uri *root_path;
	struct settings *netdevice_settings = netdev_settings ( netdev );
	struct settings *driver_settings = NULL;
	char buf[20] = { 0 };
	unsigned int flags;
	int retries = 0, boot_prot = 0, uriboot_retries = 0, canceled = 0;
	int iscsi_dhcp_ip = 0, iscsi_dhcp_params = 0, uriboot_retrie_delay = 0;
	int iscsi_boot_to_target = 0;
	int rc;

	keep_netdev_open = 0;

	/* Fetch stored settings */
	if ( nv_settings_root ) {
		driver_settings = driver_settings_from_netdev ( netdev );
		boot_prot = driver_settings_get_boot_prot_val ( driver_settings );
		retries = driver_settings_get_boot_ret_val ( driver_settings );
		rc = fetchf_setting ( driver_settings, &dhcp_ip_setting, NULL, NULL, buf, sizeof ( buf ) );
		iscsi_dhcp_ip = ( ( rc > 0 ) && ( buf[0] == 'E' ) );
		memset ( buf, 0, sizeof ( buf ) );
		rc = fetchf_setting ( driver_settings, &dhcp_iscsi_setting, NULL, NULL, buf, sizeof ( buf ) );
		iscsi_dhcp_params = ( ( rc > 0 ) && ( buf[0] == 'E' ) );
		iscsi_boot_to_target =
				driver_settings_get_iscsi_boot_to_target_val ( driver_settings );
	}
	uriboot_retrie_delay = fetch_intz_setting ( netdevice_settings, &uriboot_retry_delay_setting );
	uriboot_retries = fetch_intz_setting ( netdevice_settings, &uriboot_retry_setting );

 retry:
	/* Open device and display device status */
	if ( ( rc = ifopen ( netdev ) ) != 0 )
		goto err_ifopen;
	ifstat ( netdev );

	status_update ( STATUS_UPDATE_WAIT_ON_LINKUP );

	/* Check if we want to take the configurations from the DHCP server or not */
	if ( ( boot_prot == BOOT_PROTOCOL_ISCSI ) && ( ! iscsi_dhcp_ip ) ) {
		if ( ( rc = iflinkwait ( netdev, LINK_WAIT_TIMEOUT ) ) == 0 ) {
			/* Give some time to the network to settle.
			 * Switches may be still building their routing tables */
			rc = ifnetwork_wait ( netdev, LINK_WAIT_TIMEOUT, NETWORK_WAIT_TIMEOUT );
			if ( errno_cmp ( rc, -ECANCELED ) )
				printf ( "Operation canceled!\n" );
		}
	} else {
		rc = ifconf ( netdev, NULL );
	}

	if ( rc != 0 ) {
		if ( errno_cmp ( rc, -ECANCELED ) )
			canceled = 1;
		goto err_link_up_and_config;
	}

	/* Update iSCSI settings from flash if needed */
	if ( nv_settings_root && iscsi_dhcp_ip && ( ! iscsi_dhcp_params ) )
		iscsi_settings_from_driver_settings ( netdevice_settings, driver_settings );

	route();

	/* Try PXE menu boot, if applicable */
	if ( have_pxe_menu() && ( boot_prot == BOOT_PROTOCOL_PXE ) ) {
		printf ( "Booting from PXE menu...\n" );
		rc = pxe_menu_boot ( netdev );
		goto err_pxe_menu_boot;
	}

	/* Fetch next server and filename (if any) */
	filename = fetch_next_server_and_filename ( netdevice_settings );


	/* Fetch root path (if any) */
	root_path = fetch_root_path ( netdevice_settings );

	/* If we have both a filename and a root path, ignore an
	 * unsupported or missing URI scheme in the root path, since
	 * it may represent an NFS root.
	 */
	if ( filename && root_path &&
	     ( ( ! uri_is_absolute ( root_path ) ) ||
	       ( xfer_uri_opener ( root_path->scheme ) == NULL ) ) ) {
		printf ( "Ignoring unsupported root path\n" );
		uri_put ( root_path );
		root_path = NULL;
	}

	/* Check that we have something to boot */
	if ( ! ( filename || root_path ) ) {
		rc = -ENOENT_BOOT;
		printf ( "Nothing to boot: %s\n", strerror ( rc ) );
		goto err_no_boot;
	}

	status_update ( STATUS_UPDATE_URI_BOOT );

 uriboot_retry:
	/* Boot using next server, filename and root path */
	flags = ( root_path ? URIBOOT_NO_SAN_UNHOOK : URIBOOT_NO_SAN );
	flags |= ( ( iscsi_boot_to_target == ISCSI_BOOT_TO_TARGET_ENABLE ) ? 0 : URIBOOT_NO_SAN_BOOT );
	flags |= ( ( boot_prot == BOOT_PROTOCOL_ISCSI ) ? URIBOOT_SAN_BOOT_PRIO : 0 );
	if ( ( rc = uriboot ( filename, root_path, san_default_drive(), flags ) ) != 0 ) {
		if ( ! ipxe_is_started () || errno_cmp ( rc, -ECANCELED ) ) {
			canceled = 1;
			goto err_uriboot;
		}
		if ( uriboot_retries > 0 ) {
			uriboot_retries--;
			printf( "Boot retry in %d seconds.\n", uriboot_retrie_delay );
			sleep( uriboot_retrie_delay );
			goto uriboot_retry;
		}
		goto err_uriboot;
	}

 err_uriboot:
 err_no_boot:
	uri_put ( root_path );
	uri_put ( filename );
 err_pxe_menu_boot:
 err_link_up_and_config:
 err_ifopen:
	/* Close net device if we didn't hook to an iscsi target */
	if ( ipxe_is_started () && ! keep_netdev_open ) {
		if ( ( ! canceled ) && rc && retries ) {
			if ( retries < TOTAL_BOOT_RETRIES )
				--retries;
			rc = 0;
			goto retry;
		}
		ifclose ( netdev );
	}

	return rc;
}

/**
 * Test if network device matches the autoboot device bus type and location
 *
 * @v netdev		Network device
 * @ret is_autoboot	Network device matches the autoboot device
 */
static int is_autoboot_busloc ( struct net_device *netdev ) {
	struct device *dev;

	for ( dev = netdev->dev ; dev ; dev = dev->parent ) {
		if ( ( dev->desc.bus_type == autoboot_desc.bus_type ) &&
		     ( dev->desc.location == autoboot_desc.location ) )
			return 1;
	}
	return 0;
}

/**
 * Identify autoboot device by bus type and location
 *
 * @v bus_type		Bus type
 * @v location		Location
 */
void set_autoboot_busloc ( unsigned int bus_type, unsigned int location ) {

	/* Record autoboot device description */
	autoboot_desc.bus_type = bus_type;
	autoboot_desc.location = location;

	/* Mark autoboot device as present */
	is_autoboot_device = is_autoboot_busloc;
}

/**
 * Test if network device matches the autoboot device link-layer address
 *
 * @v netdev		Network device
 * @ret is_autoboot	Network device matches the autoboot device
 */
static int is_autoboot_ll_addr ( struct net_device *netdev ) {

	return ( memcmp ( netdev->ll_addr, autoboot_ll_addr,
			  netdev->ll_protocol->ll_addr_len ) == 0 );
}

/**
 * Identify autoboot device by link-layer address
 *
 * @v ll_addr		Link-layer address
 * @v len		Length of link-layer address
 */
void set_autoboot_ll_addr ( const void *ll_addr, size_t len ) {

	/* Record autoboot link-layer address (truncated if necessary) */
	if ( len > sizeof ( autoboot_ll_addr ) )
		len = sizeof ( autoboot_ll_addr );
	memcpy ( autoboot_ll_addr, ll_addr, len );

	/* Mark autoboot device as present */
	is_autoboot_device = is_autoboot_ll_addr;
}

/**
 * Boot the system
 */
static int autoboot ( void ) {
	struct net_device *netdev;
	struct net_device *vlan;
	int rc = -ENODEV;

	/* Try booting from each network device.  If we have a
	 * specified autoboot device location, then use only devices
	 * matching that location.
	 */
	for_each_netdev ( netdev ) {
		/* Skip any non-matching devices, if applicable */
		if ( is_autoboot_device && ( ! is_autoboot_device ( netdev ) ) )
			continue;

		/* Skip trunk device and boot from the VLAN device */
		if ( ( vlan = vlan_present ( netdev ) ) ) {
			printf ( "VLAN is present on %s. Skipping trunk net device...\n", netdev->name );
			rc = netboot ( vlan );
			continue;
		} else if ( vlan_tag ( netdev ) ) {
			/* Skip the VLAN device - we already tried to boot from it before */
			continue;
		}

		/* Attempt booting from this device */
		rc = netboot ( netdev );

		if ( ! ipxe_is_started () )
			return rc;
	}

	printf ( "No more ports. Exiting FlexBoot...\n" );
	return rc;
}

/**
 * Prompt for shell entry
 *
 * @ret	enter_shell	User wants to enter shell
 */
static int show_banner_and_get_key ( void ) {
	if ( boot_post_shell ) {
		/* Remove the key press from the buffer */
		if ( iskey() )
			getchar();
		return CTRL_B;
	}
	/* Skip prompt if timeout is zero */
	if ( BANNER_TIMEOUT <= 0 )
		return 0;

	/* Prompt user */
	printf ( "\n" );
	return ( prompt_any ( "Press Ctrl-B for FlexBoot setup, or ESC to skip boot...",
			  ( ( BANNER_TIMEOUT * TICKS_PER_SEC ) / 10 ) ) );
}


/**
 * Main iPXE flow of execution
 *
 * @v netdev		Network device, or NULL
 * @ret rc			Return status code
 */
int ipxe ( struct net_device *netdev ) {
	struct feature *feature;
	struct image *image;
	char *scriptlet;
	int key_pressed;
	int rc;

	/*
	 * Print welcome banner
	 *
	 *
	 * If you wish to brand this build of iPXE, please do so by
	 * defining the string PRODUCT_NAME in config/branding.h.
	 *
	 * While nothing in the GPL prevents you from removing all
	 * references to iPXE or http://ipxe.org, we prefer you not to
	 * do so.
	 *
	 */
	printf ( NORMAL "\n\n" PRODUCT_NAME "\nFeatures:" );
	for_each_table_entry ( feature, FEATURES )
		printf ( " %s", feature->name );
	printf ( "\n" );

	/* Boot system */
	if ( ( image = first_image() ) != NULL ) {
		/* We have an embedded image; execute it */
		rc = image_exec ( image );
	} else if ( ( key_pressed = show_banner_and_get_key () ) == CTRL_B ) {
		/* User wants shell; just give them a shell */
#ifdef FLASH_CONFIGURATION
		rc = boot_menu_ui ();
#else
		rc = shell ();
#endif
	} else if ( key_pressed != ESC ) {
		fetch_string_setting_copy ( NULL, &scriptlet_setting,
					    &scriptlet );
		if ( scriptlet ) {
			/* User has defined a scriptlet; execute it */
			rc = system ( scriptlet );
			free ( scriptlet );
			return rc;
		} else {
			/* Try booting.  If booting fails, offer the
			 * user another chance to enter the shell.
			 */
			if ( netdev ) {
				rc = netboot ( netdev );
				DBG ( "iPXE netboot status: %d\n", rc );
			} else {
				rc = autoboot();
				DBG ( "iPXE autoboot status: %d\n", rc );
			}
			if ( show_banner_and_get_key () == CTRL_B )
				rc = shell();
		}
	}

	return rc;
}
