#ifndef MLX_LINK_SPEED_H_
#define MLX_LINK_SPEED_H_

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

#define LINK_SPEED_1GB_MASK 0x2
#define LINK_SPEED_10GB_MASK 0x701c
#define LINK_SPEED_40GB_MASK 0x180C0
#define LINK_SPEED_100GB_MASK 0x700000

#define LINK_SPEED_SDR_MASK 0x1
#define LINK_SPEED_DDR_MASK 0x2
#define LINK_SPEED_QDR_MASK 0xC
#define LINK_SPEED_FDR_MASK 0x10
#define LINK_SPEED_EDR20_MASK 0x200
#define LINK_SPEED_EDR_MASK 0x20

#define LINK_SPEED_WITDH_1_MASK 0x1
#define LINK_SPEED_WITDH_2_MASK 0x2
#define LINK_SPEED_WITDH_4_MASK 0x4
#define LINK_SPEED_WITDH_8_MASK 0x8
#define LINK_SPEED_WITDH_12_MASK 0x10

#define GIGA_TO_BIT 0x40000000

typedef enum {
	LINK_SPEED_IB = 0,
	LINK_SPEED_FC,
	LINK_SPEED_ETH,
} LINK_SPEED_TYPE;

typedef enum {
	LINK_SPEED_1GB = 0,
	LINK_SPEED_10GB,
	LINK_SPEED_40GB,
	LINK_SPEED_100GB,
	LINK_SPEED_SDR,
	LINK_SPEED_DEFAULT,
} LINK_SPEED;

struct mlx_link_speed {
	mlx_uint32 proto_mask	:3;
	mlx_uint32 reserved1	:13;
	mlx_uint32 loacl_port	:8;
	mlx_uint32 reserved2	:8;
	/* -------------- */
	mlx_uint32 reserved3	:32;
	/* -------------- */
	mlx_uint32 reserved4	:32;
	/* -------------- */
	mlx_uint32 eth_proto_capability	:32;
	/* -------------- */
	mlx_uint32 ib_proto_capability	:16;
	mlx_uint32 ib_link_width_capability	:16;
	/* -------------- */
	mlx_uint32 reserved5	:32;
	/* -------------- */
	mlx_uint32 eth_proto_admin	:32;
	/* -------------- */
	mlx_uint32 ib_proto_admin	:16;
	mlx_uint32 ib_link_width_admin	:16;
	/* -------------- */
	mlx_uint32 reserved6	:32;
	/* -------------- */
	mlx_uint32 eth_proto_oper	:32;
	/* -------------- */
	mlx_uint32 ib_proto_oper	:16;
	mlx_uint32 ib_link_width_oper	:16;
};

mlx_status
mlx_set_link_speed(
		IN mlx_utils *utils,
		IN mlx_uint8 port_num,
		IN LINK_SPEED_TYPE type,
		IN LINK_SPEED speed
		);

mlx_status
mlx_get_max_speed(
		IN mlx_utils *utils,
		IN mlx_uint8 port_num,
		IN LINK_SPEED_TYPE type,
		OUT mlx_uint64 *speed
		);

#endif /* MLX_LINK_SPEED_H_ */
