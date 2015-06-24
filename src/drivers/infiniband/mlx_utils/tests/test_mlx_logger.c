/*
 * Copyright (C) 2015 Mellanox Technologies Ltd.
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

/*
 * *********************************
 * Includes
 * *********************************
 */

#include "../../mlx_utils/unity/unity.h"
#include "mlx_logger.h"
/*
 * *********************************
 * Private Forward Decelerations
 * *********************************
 */




/*
 * *********************************
 * Private Data
 * *********************************
 */


/*
 * *********************************
 * Private Implementation
 * *********************************
 */


/*
 * *********************************
 * Public Implementation
 * *********************************
 */

void setUp(void)
{
  //This is run before EACH TEST
}

void tearDown(void)
{
}

void test_mlx_logger()
{
	nmlx5_LogMessage();
	nmlx5_LogMessage("BLABLA");
	nmlx5_TraceMessage();
	nmlx5_TraceMessage("BLABLA");
}
