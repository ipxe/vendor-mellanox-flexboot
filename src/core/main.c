/**************************************************************************
iPXE -  Network Bootstrap Program

Literature dealing with the network protocols:
	ARP - RFC826
	RARP - RFC903
	UDP - RFC768
	BOOTP - RFC951, RFC2132 (vendor extensions)
	DHCP - RFC2131, RFC2132 (options)
	TFTP - RFC1350, RFC2347 (options), RFC2348 (blocksize), RFC2349 (tsize)
	RPC - RFC1831, RFC1832 (XDR), RFC1833 (rpcbind/portmapper)

**************************************************************************/

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <stddef.h>
#include <stdio.h>
#include <ipxe/init.h>
#include <ipxe/version.h>
#include <usr/autoboot.h>
#include "mlx_bullseye.h"

/**
 * Main entry point
 *
 * @ret rc		Return status code
 */
__asmcall int main ( void ) {
	DBG ( "FlexBoot main() started...\n" );
	/*
	 * The below is effective only
	 * if MLX_BULLSEYE is defined
	 */
	MlxBullseyeInit();

	/* Perform one-time-only initialisation (e.g. heap) */
	initialise();

	/* Some devices take an unreasonably long time to initialise */
	printf ( "%s initialising devices...\n", product_short_name );
	startup();
	printf ( "Initialising completed.\n" );

	ipxe ( NULL );

	shutdown_exit();

	DBG ( "FlexBoot shutdown completed... returning to real mode\n" );

	return 0;
}
