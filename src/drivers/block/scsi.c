/*
 * Copyright (C) 2006 Michael Brown <mbrown@fensystems.co.uk>.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <stddef.h>
#include <string.h>
#include <byteswap.h>
#include <errno.h>
#include <gpxe/blockdev.h>
#include <gpxe/scsi.h>

/** @file
 *
 * SCSI block device
 *
 */

static inline __attribute__ (( always_inline )) struct scsi_device *
block_to_scsi ( struct block_device *blockdev ) {
	return container_of ( blockdev, struct scsi_device, blockdev );
}

/**
 * Issue SCSI command
 *
 * @v scsi		SCSI device
 * @v command		SCSI command
 * @ret rc		Return status code
 */
static int scsi_command ( struct scsi_device *scsi,
			  struct scsi_command *command ) {
	int rc;

	/* Clear sense response code before issuing command */
	command->sense_response = 0;

	/* Issue SCSI command */
	if ( ( rc = scsi->command ( scsi, command ) ) != 0 ) {
		/* Something went wrong with the issuing mechanism,
		 * (rather than with the command itself)
		 */
		DBG ( "SCSI %p " SCSI_CDB_FORMAT " err %d\n",
		      scsi, SCSI_CDB_DATA ( command->cdb ), rc );
		return rc;
	}

	/* Check for SCSI errors */
	if ( command->status != 0 ) {
		DBG ( "SCSI %p " SCSI_CDB_FORMAT " status %02x sense %02x\n",
		      scsi, SCSI_CDB_DATA ( command->cdb ),
		      command->status, command->sense_response );
		return -EIO;
	}

	return 0;
}

/**
 * Read block from SCSI device
 *
 * @v blockdev		Block device
 * @v block		LBA block number
 * @v count		Block count
 * @v buffer		Data buffer
 * @ret rc		Return status code
 */
static int scsi_read ( struct block_device *blockdev, uint64_t block,
		       unsigned long count, userptr_t buffer ) {
	struct scsi_device *scsi = block_to_scsi ( blockdev );
	struct scsi_command command;
	struct scsi_cdb_read_16 *cdb = &command.cdb.read16;

	/* Issue READ (16) */
	memset ( &command, 0, sizeof ( command ) );
	cdb->opcode = SCSI_OPCODE_READ_16;
	cdb->lba = cpu_to_be64 ( block );
	cdb->len = cpu_to_be32 ( count );
	command.data_in = buffer;
	command.data_in_len = ( count * blockdev->blksize );
	return scsi_command ( scsi, &command );
}

/**
 * Write block to SCSI device
 *
 * @v blockdev		Block device
 * @v block		LBA block number
 * @v count		Block count
 * @v buffer		Data buffer
 * @ret rc		Return status code
 */
static int scsi_write ( struct block_device *blockdev, uint64_t block,
		        unsigned long count, userptr_t buffer ) {
	struct scsi_device *scsi = block_to_scsi ( blockdev );
	struct scsi_command command;
	struct scsi_cdb_write_16 *cdb = &command.cdb.write16;

	/* Issue WRITE (16) */
	memset ( &command, 0, sizeof ( command ) );
	cdb->opcode = SCSI_OPCODE_WRITE_16;
	cdb->lba = cpu_to_be64 ( block );
	cdb->len = cpu_to_be32 ( count );
	command.data_out = buffer;
	command.data_out_len = ( count * blockdev->blksize );
	return scsi_command ( scsi, &command );
}

/**
 * Read capacity of SCSI device via READ CAPACITY (10)
 *
 * @v blockdev		Block device
 * @ret rc		Return status code
 */
static int scsi_read_capacity_10 ( struct block_device *blockdev ) {
	struct scsi_device *scsi = block_to_scsi ( blockdev );
	struct scsi_command command;
	struct scsi_cdb_read_capacity_10 *cdb = &command.cdb.readcap10;
	struct scsi_capacity_10 capacity;
	int rc;

	/* Issue READ CAPACITY (10) */
	memset ( &command, 0, sizeof ( command ) );
	cdb->opcode = SCSI_OPCODE_READ_CAPACITY_10;
	command.data_in = virt_to_user ( &capacity );
	command.data_in_len = sizeof ( capacity );

	if ( ( rc = scsi_command ( scsi, &command ) ) != 0 )
		return rc;

	/* Fill in block device fields */
	blockdev->blksize = be32_to_cpu ( capacity.blksize );
	blockdev->blocks = ( be32_to_cpu ( capacity.lba ) + 1 );

	return 0;
}

/**
 * Read capacity of SCSI device via READ CAPACITY (16)
 *
 * @v blockdev		Block device
 * @ret rc		Return status code
 */
static int scsi_read_capacity_16 ( struct block_device *blockdev ) {
	struct scsi_device *scsi = block_to_scsi ( blockdev );
	struct scsi_command command;
	struct scsi_cdb_read_capacity_16 *cdb = &command.cdb.readcap16;
	struct scsi_capacity_16 capacity;
	int rc;

	/* Issue READ CAPACITY (16) */
	memset ( &command, 0, sizeof ( command ) );
	cdb->opcode = SCSI_OPCODE_SERVICE_ACTION_IN;
	cdb->service_action = SCSI_SERVICE_ACTION_READ_CAPACITY_16;
	cdb->len = cpu_to_be32 ( sizeof ( capacity ) );
	command.data_in = virt_to_user ( &capacity );
	command.data_in_len = sizeof ( capacity );

	if ( ( rc = scsi_command ( scsi, &command ) ) != 0 )
		return rc;

	/* Fill in block device fields */
	blockdev->blksize = be32_to_cpu ( capacity.blksize );
	blockdev->blocks = ( be64_to_cpu ( capacity.lba ) + 1 );
	return 0;
}

/**
 * Read capacity of SCSI device
 *
 * @v blockdev		Block device
 * @ret rc		Return status code
 */
static int scsi_read_capacity ( struct block_device *blockdev ) {
	int rc;

	/* Issue a theoretically extraneous READ CAPACITY (10)
	 * command, solely in order to draw out the "CHECK CONDITION
	 * (power-on occurred)" that some dumb targets insist on
	 * sending as an error at start of day.
	 */
	scsi_read_capacity_10 ( blockdev );

	/* Try READ CAPACITY (10), which is a mandatory command, first. */
	if ( ( rc = scsi_read_capacity_10 ( blockdev ) ) != 0 )
		return rc;

	/* If capacity range was exceeded (i.e. capacity.lba was
	 * 0xffffffff, meaning that blockdev->blocks is now zero), use
	 * READ CAPACITY (16) instead.  READ CAPACITY (16) is not
	 * mandatory, so we can't just use it straight off.
	 */
	if ( blockdev->blocks == 0 ) {
		if ( ( rc = scsi_read_capacity_16 ( blockdev ) ) != 0 )
			return rc;
	}

	return 0;
}

/**
 * Initialise SCSI device
 *
 * @v scsi		SCSI device
 * @ret rc		Return status code
 *
 * Initialises a SCSI device.  The scsi_device::command and
 * scsi_device::lun fields must already be filled in.  This function
 * will configure scsi_device::blockdev, including issuing a READ
 * CAPACITY call to determine the block size and total device size.
 */
int init_scsidev ( struct scsi_device *scsi ) {
	/** Fill in read and write methods, and get device capacity */
	scsi->blockdev.read = scsi_read;
	scsi->blockdev.write = scsi_write;
	return scsi_read_capacity ( &scsi->blockdev );
}
