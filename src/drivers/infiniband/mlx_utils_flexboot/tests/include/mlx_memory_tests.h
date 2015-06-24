/*
 * MlxMemoryTest.h
 *
 *  Created on: Jan 22, 2015
 *      Author: maord
 */

#ifndef STUB_MLXUTILS_TESTS_INCLUDE_MLXMEMORYTEST_H_
#define STUB_MLXUTILS_TESTS_INCLUDE_MLXMEMORYTEST_H_

#include "mlx_memory.h"

mlx_status
mlx_memory_alloc_test(
				IN mlx_utils *utils
				);
mlx_status
mlx_memory_zalloc_test(
		IN mlx_utils *utils
		);

mlx_status
mlx_memory_alloc_dma_test(
		IN mlx_utils *utils
		);

mlx_status
mlx_memory_map_dma_test(
		IN mlx_utils *utils
		);

mlx_status
mlx_memory_cmp_test(
		IN mlx_utils *utils
		);

#endif /* STUB_MLXUTILS_TESTS_INCLUDE_MLXMEMORYTEST_H_ */
