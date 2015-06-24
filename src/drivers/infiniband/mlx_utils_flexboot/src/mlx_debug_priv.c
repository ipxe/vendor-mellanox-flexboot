/*
 * Copyright (C) 2007 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * Based in part upon the original driver by Mellanox Technologies
 * Ltd.  Portions may be Copyright (c) Mellanox Technologies Ltd.
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

/*
 * MlxDebug.c
 *
 *  Created on: Jan 21, 2015
 *      Author: moshikos
 */

#pragma GCC push_options
#pragma GCC optimize 0

/*
 * *********************************
 * Includes
 * *********************************
 */

#ifdef MLX_DEBUG

#include <stdio.h>
#include <ipxe/gdbserial.h>
#include <ipxe/gdbstub.h>
#include "mlx_debug.h"

#else

#endif

/*
 * *********************************
 * Private Forward Decelerations
 * *********************************
 */

#ifdef MLX_DEBUG

#else

#endif

/*
 * *********************************
 * Private Data
 * *********************************
 */

#ifdef MLX_DEBUG

static int mlx_debug_initialized = 0;

#else

#endif

/*
 * *********************************
 * Private Implementation
 * *********************************
 */

#ifdef MLX_DEBUG

#else

#endif

/*
 * *********************************
 * Public Implementation
 * *********************************
 */


#ifdef MLX_DEBUG


void MlxDebugBreakSerial()
{
	if ( mlx_debug_initialized ) {
		printf ( "\n\n"
				"############################################################\n"
				"Software breakpoint\n"
				"############################################################\n"
				);
		gdbmach_breakpoint();
	} else
	{
		MlxDebugInitializeSerial();
		mlx_debug_initialized = 1;
	}
}

void MlxDebugInitializeSerial()
{
	printf ( "\n\n"
				"############################################################\n"
				"Init debugger: Waiting to sync with debugger on serial port\n"
				"############################################################\n"
				);
	gdbstub_start ( gdbserial_configure() );
	printf ( "System synched with debugger\n"
			 "############################################################\n");
}

void _MlxDebugWaitInfinite(void)
{
	volatile int i = 0;

	printf ( "\n\n"
				"############################################################\n"
				"Init debugger: Waiting to infinite loop while i=0\n"
				"############################################################\n"
				);
	while ( i == 0 )
		;
}

#else

#endif

#pragma GCC pop_options
