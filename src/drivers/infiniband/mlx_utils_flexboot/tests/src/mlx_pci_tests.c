/*
 * MlxPciTest.c
 *
 *  Created on: Jan 22, 2015
 *      Author: maord
 */

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
 * test_mlx_logger.c
 *
 *   Created on: Jan 20, 2015
 *       Author: moshikos
 *  Description: [ To be filled ]
 */



/*
 * *********************************
 * Includes
 * *********************************
 */

#include <stdio.h>
#include "unity.h"
#include "mlx_utils.h"
#include "mlx_pci_tests.h"

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

#define MLX_PCI_VENDOR_ID_OFFSET 0
#define MLX_PCI_DEVICE_ID_OFFSET 2

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

mlx_status
mlx_pci_read_vendor_id_test1(
			IN mlx_utils *utils
			)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint16 vendor_id = 0;
	status = mlx_pci_read(utils, MlxPciWidthUint16, MLX_PCI_VENDOR_ID_OFFSET, 1, &vendor_id);
	if(status!= MLX_SUCCESS){
		printf("mlx_pci_read_vendor_id_test1: mlx_pci_read failed - status =%d\n",status);
		goto bail;
	}
	if(utils->pci->vendor != vendor_id){
		printf("mlx_pci_read_vendor_id_test1: utils->pci->vendor != vendor_id (%x != %x)\n",
				utils->pci->vendor,
				vendor_id);
			goto bail;
	}
bail:
	return status;
}

mlx_status
mlx_pci_read_vendor_id_test2(
			IN mlx_utils *utils
			)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint16 vendor_id = 0;
	status = mlx_pci_read(utils, MlxPciWidthUint8, MLX_PCI_VENDOR_ID_OFFSET, 2, &vendor_id);
	if(status!= MLX_SUCCESS){
		printf("mlx_pci_read_vendor_id_test2: mlx_pci_read failed - status =%d\n",status);
		goto bail;
	}
	if(utils->pci->vendor != vendor_id){
		printf("mlx_pci_read_vendor_id_test2: utils->pci->vendor != vendor_id (%x != %x)\n",
				utils->pci->vendor, vendor_id);
			goto bail;
	}
bail:
	return status;
}

mlx_status
mlx_pci_read_device_id_test1(
			IN mlx_utils *utils
			)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint16 device_id = 0;
	status = mlx_pci_read(utils, MlxPciWidthUint16, MLX_PCI_DEVICE_ID_OFFSET, 1, &device_id);
	if(status!= MLX_SUCCESS){
		printf("mlx_pci_read_device_id_test1: mlx_pci_read failed - status =%d\n",status);
		goto bail;
	}
	if(utils->pci->device != device_id){
		printf("mlx_pci_read_device_id_test1: utils->pci->devie != device_id (%x != %x)\n",
				utils->pci->device, device_id);
			goto bail;
	}
bail:
	return status;
}

mlx_status
mlx_pci_read_device_id_test2(
			IN mlx_utils *utils
			)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint16 device_id = 0;
	status = mlx_pci_read(utils, MlxPciWidthUint8, MLX_PCI_DEVICE_ID_OFFSET, 2, &device_id);
	if(status!= MLX_SUCCESS){
		printf("mlx_pci_read_device_id_test2: mlx_pci_read failed - status =%d\n",status);
		goto bail;
	}
	if(utils->pci->device != device_id){
		printf("mlx_pci_read_device_id_test2: utils->pci->devie != device_id (%x != %x)\n",
				utils->pci->device, device_id);
			goto bail;
	}
bail:
	return status;
}

mlx_status
mlx_pci_read_vendor_and_device_id_test1(
			IN mlx_utils *utils
			)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint16 vendor_id = 0;
	mlx_uint16 device_id = 0;
	mlx_uint32 tmp = 0;
	status = mlx_pci_read(utils, MlxPciWidthUint32, MLX_PCI_VENDOR_ID_OFFSET, 1, &tmp);
	if(status!= MLX_SUCCESS){
		printf("mlx_pci_read_vendor_and_device_id_test1: mlx_pci_read failed - status =%d\n",status);
		goto bail;
	}
	vendor_id = (mlx_uint16)(tmp & 0xFFFF);
	device_id = (mlx_uint16)((tmp >> 16) & 0xFFFF);
	if(utils->pci->device != device_id){
		printf("mlx_pci_read_vendor_and_device_id_test1: utils->pci->device != device_id (%x != %x)\n",
				utils->pci->device, device_id);
			goto bail;
	}
	if(utils->pci->vendor != vendor_id){
		printf("mlx_pci_read_vendor_and_device_id_test1: utils->pci->vendor != vendor_id (%x != %x)\n",
			utils->pci->vendor, vendor_id);
		goto bail;
	}
bail:
	return status;
}

mlx_status
mlx_pci_read_vendor_and_device_id_test2(
			IN mlx_utils *utils
			)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint16 vendor_id = 0;
	mlx_uint16 device_id = 0;
	mlx_uint32 tmp = 0;
	status = mlx_pci_read(utils, MlxPciWidthUint16, MLX_PCI_VENDOR_ID_OFFSET, 2, &tmp);
	if(status!= MLX_SUCCESS){
		printf("mlx_pci_read_vendor_and_device_id_test2: mlx_pci_read failed - status =%d\n",status);
		goto bail;
	}
	vendor_id = (mlx_uint16)(tmp & 0xFFFF);
	device_id = (mlx_uint16)((tmp >> 16) & 0xFFFF);
	if(utils->pci->device != device_id){
		printf("mlx_pci_read_vendor_and_device_id_test2: utils->pci->device != device_id (%x != %x)\n",
				utils->pci->device, device_id);
			goto bail;
	}
	if(utils->pci->vendor != vendor_id){
		printf("mlx_pci_read_vendor_and_device_id_test2: utils->pci->vendor != vendor_id (%x != %x)\n",
			utils->pci->vendor, vendor_id);
		goto bail;
	}
bail:
	return status;
}

mlx_status
mlx_pci_read_vendor_and_device_id_test3(
			IN mlx_utils *utils
			)
{
	mlx_status status = MLX_SUCCESS;
	mlx_uint16 vendor_id = 0;
	mlx_uint16 device_id = 0;
	mlx_uint32 tmp = 0;
	status = mlx_pci_read(utils, MlxPciWidthUint8, MLX_PCI_VENDOR_ID_OFFSET, 4, &tmp);
	if(status!= MLX_SUCCESS){
		printf("mlx_pci_read_vendor_and_device_id_test3: mlx_pci_read failed - status =%d\n",status);
		goto bail;
	}
	vendor_id = (mlx_uint16)(tmp & 0xFFFF);
	device_id = (mlx_uint16)((tmp >> 16) & 0xFFFF);
	if(utils->pci->device != device_id){
		printf("mlx_pci_read_vendor_and_device_id_test3: utils->pci->device != device_id (%x != %x)\n",
				utils->pci->device, device_id);
			goto bail;
	}
	if(utils->pci->vendor != vendor_id){
		printf("mlx_pci_read_vendor_and_device_id_test3: utils->pci->vendor != vendor_id (%x != %x)\n",
			utils->pci->vendor, vendor_id);
		goto bail;
	}
bail:
	return status;
}




void runTest(UnityTestFunction test)
{
  {
      setUp();
      test();
  }
  if (TEST_PROTECT() && !TEST_IS_IGNORED)
  {
    tearDown();
  }
}
void resetTest(void);
void resetTest(void)
{
  tearDown();
  setUp();
}


int mlx_pci_tests(mlx_utils *utils)
{
	UnityBegin("test_mlx_logger.c");

	// RUN_TEST calls runTest
	//RUN_TEST(test_mlx_logger);

	mlx_pci_read_vendor_id_test1(utils);
	mlx_pci_read_device_id_test1(
			utils
			);
	mlx_pci_read_device_id_test2(
			utils
			);
	mlx_pci_read_vendor_and_device_id_test1(
			utils
			);
	mlx_pci_read_vendor_and_device_id_test2(
			utils
			);
	mlx_pci_read_vendor_and_device_id_test3(
			utils
			);

	UnityEnd();
	return 0;
}
