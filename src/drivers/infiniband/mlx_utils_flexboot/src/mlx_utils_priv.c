/*
 * MlxUtilsPriv.c
 *
 *  Created on: Jan 25, 2015
 *      Author: maord
 */

#include <unistd.h>

#include "mlx_utils_priv.h"
mlx_status
mlx_utils_delay_in_ms_priv(
			IN mlx_uint32 msecs
		)
{
	mdelay(msecs);
	return MLX_SUCCESS;
}

mlx_status
mlx_utils_delay_in_us_priv(
			IN mlx_uint32 usecs
		)
{
	udelay(usecs);
	return MLX_SUCCESS;
}

mlx_status
mlx_utils_ilog2_priv(
			IN mlx_uint32 i,
			OUT mlx_uint32 *log
		)
{
	uint32_t reg = 0xffff;
	asm volatile ("bsr %1, %0" : "=r" (reg) : "m" (i));
	*log = reg;
	return MLX_SUCCESS;
}
