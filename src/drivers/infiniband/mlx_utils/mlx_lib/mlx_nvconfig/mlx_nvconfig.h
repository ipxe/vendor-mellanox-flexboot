#ifndef MLX_NVCONFIG_H_
#define MLX_NVCONFIG_H_

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

#include "mlx_reg_access.h"
#include "mlx_utils.h"

struct nvconfig_tlv_type {
	 mlx_uint32 param_idx	:16;
	 mlx_uint32 port		:8;
	 mlx_uint32 param_class	:8;
};


struct nvconfig_nvqc {
	struct nvconfig_tlv_type tlv_type;
/* -------------- */
	 mlx_uint32 support_rd	:1; /*the configuration item is supported and can be read */
	 mlx_uint32 support_wr	:1; /*the configuration item is supported and can be updated */
	 mlx_uint32 reserved1	:2;
	 mlx_uint32 version		:4; /*The maximum version of the configuration item currently supported by the firmware. */
	 mlx_uint32 reserved2	:24;
};


struct nvconfig_header {
	 mlx_uint32 length		:8; /*Size of configuration item data in bytes between 0..256 */
	 mlx_uint32 reserved0	:3;
	 mlx_uint32 shadow		:1; /* 0 - FW writes configuration item directly, 1 - FW writes configuration to a temporary aread untill NVCT operation is called */
	 mlx_uint32 version		:4; /* Configuration item version */
	 mlx_uint32 reserved1	:8;

	 mlx_uint32 rd_en		:1; /*enables reading the TLV by lower priorities
									0 - TLV can be read by the subsequent lifecycle priorities.
									1 - TLV cannot be read by the subsequent lifecycle priorities. */
	 mlx_uint32 over_en		:1; /*enables overwriting the TLV by lower priorities
									0 - Can only be overwritten by the current lifecycle priority
									1 - Allowed to be overwritten by subsequent lifecycle priorities */
	 mlx_uint32 reserved2	:2;
	 mlx_uint32 default_value		:1; /*when set in query , will display the default configuration */
	 mlx_uint32 reserved3	:3;
/* -------------- */
	 struct nvconfig_tlv_type tlv_type;;
/* -------------- */
	mlx_uint32 crc			:16;
	mlx_uint32 reserved		:16;
};

#define NVCONFIG_MAX_TLV_SIZE 256

struct nvconfig_nvda {
	struct nvconfig_header nv_header;
	mlx_uint8 data[NVCONFIG_MAX_TLV_SIZE];
};


mlx_status
nvconfig_query_capability(
		IN mlx_utils *utils,
		IN mlx_uint8 port,
		IN mlx_uint16 tlv_type,
		OUT mlx_boolean *read_supported,
		OUT mlx_boolean *write_supported
		);


mlx_status
nvconfig_nvdata_invalidate(
		IN mlx_utils *utils,
		IN mlx_uint8 port,
		IN mlx_uint16 tlv_type
		);

mlx_status
nvconfig_nvdata_access(
		IN mlx_utils *utils,
		IN mlx_uint8 port,
		IN mlx_uint16 tlv_type,
		IN REG_ACCESS_OPT opt,
		IN mlx_size data_size,
		IN OUT mlx_uint8 *version,
		IN OUT mlx_void *data
		);

#endif /* MLX_NVCONFIG_H_ */
