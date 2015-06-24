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

#include "mlx_nvconfig.h"
#include "mlx_memory.h"
#include "mlx_bail.h"

mlx_status
nvconfig_query_capability(
		IN mlx_utils *utils,
		IN mlx_uint8 port,
		IN mlx_uint16 tlv_type,
		OUT mlx_boolean *read_supported,
		OUT mlx_boolean *write_supported
		)
{
	mlx_status status = MLX_SUCCESS;
	struct nvconfig_nvqc nvqc;
	mlx_uint32 reg_status;

	if (utils == NULL || read_supported == NULL || write_supported == NULL) {
		status = MLX_INVALID_PARAMETER;
		goto bad_param;
	}
	mlx_memory_set(utils, &nvqc, 0, sizeof(nvqc));
	nvqc.tlv_type.param_class = 0x1;
	nvqc.tlv_type.param_idx = tlv_type;
	nvqc.tlv_type.port = port;

	status = mlx_reg_access(utils, REG_ID_NVQC, REG_ACCESS_READ, &nvqc, sizeof(nvqc),
			&reg_status);
	MLX_CHECK_STATUS(utils, status, reg_err, "mlx_reg_access failed ");
	if (reg_status != 0) {
		MLX_DEBUG_ERROR(utils,"mlx_reg_access failed with status = %d\n", reg_status);
		status = MLX_FAILED;
		goto reg_err;
	}
	*read_supported = nvqc.support_rd;
	*write_supported = nvqc.support_wr;
reg_err:
bad_param:
	return status;
}

mlx_status
nvconfig_nvdata_invalidate(
		IN mlx_utils *utils,
		IN mlx_uint8 port,
		IN mlx_uint16 tlv_type
		)
{
	mlx_status status = MLX_SUCCESS;
	struct nvconfig_header nv_header;
	mlx_uint32 reg_status;

	if (utils == NULL) {
		status = MLX_INVALID_PARAMETER;
		goto bad_param;
	}
	mlx_memory_set(utils, &nv_header, 0, sizeof(nv_header));

	nv_header.tlv_type.param_class = 0x1;
	nv_header.tlv_type.param_idx = tlv_type;
	nv_header.tlv_type.port = port;

	status = mlx_reg_access(utils, REG_ID_NVDI, REG_ACCESS_WRITE, &nv_header, sizeof(nv_header),
			&reg_status);
	MLX_CHECK_STATUS(utils, status, reg_err, "mlx_reg_access failed ");
	if (reg_status != 0) {
		MLX_DEBUG_ERROR(utils,"mlx_reg_access failed with status = %d\n", reg_status);
		status = MLX_FAILED;
		goto reg_err;
	}
reg_err:
bad_param:
	return status;
}

mlx_status
nvconfig_nvdata_access(
		IN mlx_utils *utils,
		IN mlx_uint8 port,
		IN mlx_uint16 tlv_type,
		IN REG_ACCESS_OPT opt,
		IN mlx_size data_size,
		IN OUT mlx_uint8 *version,
		IN OUT mlx_void *data
		)
{
	mlx_status status = MLX_SUCCESS;
	struct nvconfig_nvda nvda;
	mlx_uint32 reg_status;
	mlx_uint32 real_size_to_read;

	if (utils == NULL || data == NULL || data_size > NVCONFIG_MAX_TLV_SIZE) {
		status = MLX_INVALID_PARAMETER;
		goto bad_param;
	}

	mlx_memory_set(utils, &nvda, 0, sizeof(nvda));
	nvda.nv_header.length = ((data_size + 3) / sizeof(mlx_uint32)) * sizeof(mlx_uint32);
	nvda.nv_header.rd_en = 0;
	nvda.nv_header.over_en = 1;
	nvda.nv_header.version = *version;
	nvda.nv_header.tlv_type.param_class = 0x1;
	nvda.nv_header.tlv_type.param_idx = tlv_type;
	nvda.nv_header.tlv_type.port = port;
	mlx_memory_cpy(utils, nvda.data, data, data_size);

	status = mlx_reg_access(utils, REG_ID_NVDA, opt, &nvda,
			data_size + sizeof(nvda.nv_header), &reg_status);
	MLX_CHECK_STATUS(utils, status, reg_err, "mlx_reg_access failed ");
	if (reg_status != 0) {
		MLX_DEBUG_ERROR(utils,"mlx_reg_access failed with status = %d\n", reg_status);
		status = MLX_FAILED;
		goto reg_err;
	}
	if (opt == REG_ACCESS_READ) {
		real_size_to_read = (nvda.nv_header.length > data_size) ? data_size :
				nvda.nv_header.length;
		mlx_memory_cpy(utils, data, nvda.data, real_size_to_read);
		*version = nvda.nv_header.version;
	}
reg_err:
bad_param:
	return status;
}


