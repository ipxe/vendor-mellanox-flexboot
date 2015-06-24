/*
 * MlxMemoryTests.c
 *
 *  Created on: Jan 22, 2015
 *      Author: maord
 */

#include "mlx_memory_tests.h"

#include <stddef.h>
#include <stdio.h>

mlx_status
mlx_memory_alloc_test(
				IN mlx_utils *utils
				)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint32 *i = NULL;
	status = mlx_memory_alloc(utils, sizeof(mlx_uint32), (mlx_void**)&i);
	if(status!= MLX_SUCCESS){
		printf("mlx_memory_alloc_ failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_free(utils, &i);
	if(status!= MLX_SUCCESS ){
		printf("mlx_memory_free failed - status =%d\n",status);
	}
bail:
	return status;
}

mlx_status
mlx_memory_zalloc_test(
		IN mlx_utils *utils
		)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint32 *i = NULL;
	status = mlx_memory_zalloc(utils, sizeof(mlx_uint32), (mlx_void**)&i);
	if(status!= MLX_SUCCESS || *i != 0){
		printf("mlx_memory_zalloc failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_free(utils, &i);
	if(status!= MLX_SUCCESS ){
		printf("mlx_memory_free failed - status =%d\n",status);
	}
bail:
	return status;
}

mlx_status
mlx_memory_alloc_dma_test(
		IN mlx_utils *utils
		)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint32 *i = NULL;
	status = mlx_memory_alloc_dma(utils, sizeof(mlx_uint32),sizeof(mlx_uint32) ,(mlx_void**)&i);
	if(status!= MLX_SUCCESS){
		printf("mlx_memory_alloc_dma failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_free_dma(utils, sizeof(mlx_uint32) ,&i);
	if(status!= MLX_SUCCESS ){
		printf("mlx_memory_free_dma failed - status =%d\n",status);
	}
bail:
	return status;
}

mlx_status
mlx_memory_map_dma_test(
		IN mlx_utils *utils
		)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint32 *i = NULL;
	mlx_physical_address paddr = 0;
	mlx_void *mapping = NULL;
	status = mlx_memory_alloc_dma(utils, sizeof(mlx_uint32),sizeof(mlx_uint32) ,(mlx_void**)&i);
	if(status!= MLX_SUCCESS){
		printf("mlx_memory_alloc_dma failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_map_dma(utils, i, sizeof(mlx_uint32), &paddr ,&mapping);
	if(status!= MLX_SUCCESS || paddr == 0){
		printf("mlx_memory_map_dma failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_ummap_dma(utils, mapping);
	if(status!= MLX_SUCCESS ){
		printf("mlx_memory_ummap_dma failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_free_dma(utils, sizeof(mlx_uint32) ,&i);
	if(status!= MLX_SUCCESS ){
		printf("mlx_memory_free_dma failed - status =%d\n",status);
	}
bail:
	return status;
}

mlx_status
mlx_memory_cmp_test(
		IN mlx_utils *utils
		)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint32 out = 0;
	mlx_uint32 a = 5;
	mlx_uint32 b = 5;
	mlx_uint32 c = 3;
	mlx_mac_address mac1 = {{1,2,3,4,5,6}};
	mlx_mac_address mac2 = {{1,2,3,4,5,6}};
	mlx_mac_address mac3 = {{6,6,6,6,6,6}};

	status = mlx_memory_cmp(utils, &a, &b, sizeof(a), &out);
	if(status!= MLX_SUCCESS || out){
		printf("mlx_memory_cmp 1 failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_cmp(utils, &a, &c, sizeof(a), &out);
	if(status!= MLX_SUCCESS || !out){
		printf("mlx_memory_cmp 2 failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_cmp(utils, &mac1, &mac2, sizeof(mac1), &out);
	if(status!= MLX_SUCCESS || out){
		printf("mlx_memory_cmp 3 failed - status =%d\n",status);
		goto bail;
	}
	status = mlx_memory_cmp(utils, &mac1, &mac3, sizeof(mac1), &out);
	if(status!= MLX_SUCCESS || !out){
		printf("mlx_memory_cmp 4 failed - status =%d\n",status);
		goto bail;
	}
bail:
	return status;
}
