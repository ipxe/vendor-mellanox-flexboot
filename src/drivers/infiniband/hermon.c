/*
 * Copyright (C) 2006 Michael Brown <mbrown@fensystems.co.uk>.
 * Copyright (C) 2008-2015 Mellanox Technologies Ltd.
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <errno.h>
#include <byteswap.h>
#include <ipxe/io.h>
#include <ipxe/pci.h>
#include <ipxe/pcibackup.h>
#include <ipxe/malloc.h>
#include <ipxe/umalloc.h>
#include <ipxe/iobuf.h>
#include <ipxe/in.h>
#include <ipxe/netdevice.h>
#include <ipxe/process.h>
#include <ipxe/infiniband.h>
#include <ipxe/ib_smc.h>
#include <ipxe/if_ether.h>
#include <ipxe/ethernet.h>
#include <ipxe/settings.h>
#include <ipxe/fcoe.h>
#include <ipxe/vlan.h>
#include <ipxe/bofm.h>
#include <ipxe/iscsi.h>
#include <ipxe/settings.h>
#include <ipxe/status_updater.h>
#include <usr/ifmgmt.h>
#include <config/general.h>
#include <ipxe/boot_menu_ui.h>
#include "hermon.h"
#include "hermon_settings.h"
#include "flexboot_nodnic.h"
#include "mlx_port.h"

/*
 * If defined, use interrupts in normal driver
 */
#undef HERMON_IRQ_ENABLED

extern void ( * ipoib_hw_guid2mac ) ( const void *hw_addr, void *ll_addr );
static uint8_t g_mac_admin_bit = 0x00;
static uint8_t g_fact_mac_addr[ETH_ALEN];

static struct hermon_port* hermon_get_port_from_ibdev(struct ib_device *ibdev)
{
	struct hermon *hermon = (struct hermon *)(ib_get_drvdata(ibdev));
	int i ;

	for (i = 0; i < HERMON_MAX_PORTS; ++i) {
		if (hermon->port[i].ibdev == ibdev) {
			return &(hermon->port[i]);
		}
	}
	/* Should not be here */
	printf("hermon_get_port_from_ibdev: Bad pointer\n");
	assert(0);
	return NULL;
}


/************************************************/
/* ScratchPad debug								*/
/************************************************/
#define SCRATCHPAD_WORDS_START	0x5c238
#define SCRATCHPAD_SIZE		43

uint32_t	scratchpad_idx	= 0;
uint32_t	scratchpad_flag	= 1;
void*		scratchpad_addr	= NULL;

inline void hermon_write_scratchpad ( uint32_t val ){
	uint32_t	swapped;
	void		*addr;

	if (scratchpad_addr == NULL) {
		return;
	}

	addr	= (scratchpad_addr + (4 * scratchpad_idx));

	if (scratchpad_idx >= SCRATCHPAD_SIZE) {
		swapped	= cpu_to_be32(scratchpad_flag);
		barrier();
		writel(swapped, addr);
		scratchpad_idx = 0;
		scratchpad_flag++;
	}

	swapped	= cpu_to_be32(val);
	barrier();
	writel(swapped, addr);
	scratchpad_idx++;
}

/*******************************************/

/**
 * @file
 *
 * Mellanox ConnectX3 Infiniband HCA
 *
 */

/***************************************************************************
 *
 * Queue number allocation
 *
 ***************************************************************************
 */

/**
 * Allocate offsets within usage bitmask
 *
 * @v bits		Usage bitmask
 * @v bits_len		Length of usage bitmask
 * @v num_bits		Number of contiguous bits to allocate within bitmask
 * @ret bit		First free bit within bitmask, or negative error
 */
static int hermon_bitmask_alloc ( hermon_bitmask_t *bits,
				  unsigned int bits_len,
				  unsigned int num_bits ) {
	unsigned int bit = 0;
	hermon_bitmask_t mask = 1;
	unsigned int found = 0;

	/* Search bits for num_bits contiguous free bits */
	while ( bit < bits_len ) {
		if ( ( mask & *bits ) == 0 ) {
			if ( ++found == num_bits )
				goto found;
		} else {
			found = 0;
		}
		bit++;
		mask = ( mask << 1 ) | ( mask >> ( 8 * sizeof ( mask ) - 1 ) );
		if ( mask == 1 )
			bits++;
	}
	return -ENFILE;

 found:
	/* Mark bits as in-use */
	do {
		*bits |= mask;
		if ( mask == 1 )
			bits--;
		mask = ( mask >> 1 ) | ( mask << ( 8 * sizeof ( mask ) - 1 ) );
	} while ( --found );

	return ( bit - num_bits + 1 );
}

/**
 * Free offsets within usage bitmask
 *
 * @v bits		Usage bitmask
 * @v bit		Starting bit within bitmask
 * @v num_bits		Number of contiguous bits to free within bitmask
 */
static void hermon_bitmask_free ( hermon_bitmask_t *bits,
				  int bit, unsigned int num_bits ) {
	hermon_bitmask_t mask;

	for ( ; num_bits ; bit++, num_bits-- ) {
		mask = ( 1 << ( bit % ( 8 * sizeof ( mask ) ) ) );
		bits[ ( bit / ( 8 * sizeof ( mask ) ) ) ] &= ~mask;
	}
}

/***************************************************************************
 *
 * HCA commands
 *
 ***************************************************************************
 */

/**
 * Wait for ConnectX3 command completion
 *
 * @v hermon		ConnectX3 device
 * @v hcr		HCA command registers
 * @ret rc		Return status code
 */
static int hermon_cmd_wait ( struct hermon *hermon,
			     struct hermonprm_hca_command_register *hcr ) {
	unsigned int wait;

	for ( wait = ( HERMON_HCR_MAX_WAIT_MS * 100 ) ; wait ; wait-- ) {
		hcr->u.dwords[6] =
			readl ( hermon->config + HERMON_HCR_REG ( 6 ) );
		if ( ( MLX_GET ( hcr, go ) == 0 ) &&
		     ( MLX_GET ( hcr, t ) == hermon->toggle ) )
			return 0;
		udelay ( 10 );
	}
	return -EBUSY;
}

/**
 * Return physical function for specific commands
 *
 * @v hermon		ConnectX3 device
 * @v opcode		HCA command opcode
 */
static u8 hermon_cmd_get_function ( struct hermon *hermon,
				    int opcode ) {
	switch ( opcode ) {
	case HERMON_HCR_SW2HW_EQ:
	case HERMON_HCR_SW2HW_CQ:
	case HERMON_HCR_RST2INIT_QP:
	case HERMON_HCR_INIT2RTR_QP:
	case HERMON_HCR_RTR2RTS_QP:
	case HERMON_HCR_SW2HW_MPT:
	case HERMON_HCR_2RST_QP:
	case HERMON_HCR_HW2SW_CQ:
	case HERMON_HCR_HW2SW_EQ:
		return hermon->physical_function;
		break;
	default:
		return 0;
	}

	return 0;
}

/**
 * Issue HCA command
 *
 * @v hermon		ConnectX3 device
 * @v command		Command opcode, flags and input/output lengths
 * @v op_mod		Opcode modifier (0 if no modifier applicable)
 * @v in		Input parameters
 * @v in_mod		Input modifier (0 if no modifier applicable)
 * @v out		Output parameters
 * @event		if true, command will complete with event
 * @ret rc		Return status code
 */
static int hermon_cmd ( struct hermon *hermon, unsigned long command,
			unsigned int op_mod, const void *in,
			unsigned int in_mod, void *out, int event) {
	struct hermonprm_hca_command_register hcr;
	unsigned int opcode = HERMON_HCR_OPCODE ( command );
	size_t in_len = HERMON_HCR_IN_LEN ( command );
	size_t out_len = HERMON_HCR_OUT_LEN ( command );
	u8 physical_function;
	void *in_buffer;
	void *out_buffer;
	unsigned int status;
	unsigned int i;
	int rc;
	u32 evw;

	assert ( in_len <= HERMON_MBOX_SIZE );
	assert ( out_len <= HERMON_MBOX_SIZE );

	DBGC2 ( hermon, "ConnectX3 %p command %02x in %zx%s out %zx%s\n",
		hermon, opcode, in_len,
		( ( command & HERMON_HCR_IN_MBOX ) ? "(mbox)" : "" ), out_len,
		( ( command & HERMON_HCR_OUT_MBOX ) ? "(mbox)" : "" ) );

	/* Check that HCR is free */
	if ( ( rc = hermon_cmd_wait ( hermon, &hcr ) ) != 0 ) {
		DBGC ( hermon, "ConnectX3 %p command interface locked\n",
		       hermon );
		return rc;
	}

	/* Flip HCR toggle */
	hermon->toggle = ( 1 - hermon->toggle );

	/* Get physical function */
	physical_function = hermon_cmd_get_function ( hermon, opcode );

	/* Prepare HCR */
	memset ( &hcr, 0, sizeof ( hcr ) );
	in_buffer = &hcr.u.dwords[0];
	if ( in_len && ( command & HERMON_HCR_IN_MBOX ) ) {
		memset ( hermon->mailbox_in, 0, HERMON_MBOX_SIZE );
		in_buffer = hermon->mailbox_in;
		MLX_FILL_H ( &hcr, 0, in_param_h, virt_to_bus ( in_buffer ) );
		MLX_FILL_1 ( &hcr, 1, in_param_l, virt_to_bus ( in_buffer ) |
			     physical_function );
	} else
		MLX_FILL_1 ( &hcr, 1, in_param_l, physical_function );
	memcpy ( in_buffer, in, in_len );
	MLX_FILL_1 ( &hcr, 2, input_modifier, in_mod );
	out_buffer = &hcr.u.dwords[3];
	if ( out_len && ( command & HERMON_HCR_OUT_MBOX ) ) {
		out_buffer = hermon->mailbox_out;
		MLX_FILL_H ( &hcr, 3, out_param_h,
			     virt_to_bus ( out_buffer ) );
		MLX_FILL_1 ( &hcr, 4, out_param_l,
			     virt_to_bus ( out_buffer ) );
	}
	MLX_FILL_4 ( &hcr, 6,
		     opcode, opcode,
		     opcode_modifier, op_mod,
		     go, 1,
		     t, hermon->toggle );

	DBGC2 ( hermon, "ConnectX3 %p issuing command %04x\n",
	       hermon, opcode );
	DBGC2_HDA ( hermon, virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
		    &hcr, sizeof ( hcr ) );
	if ( in_len && ( command & HERMON_HCR_IN_MBOX ) ) {
		DBGC2 ( hermon, "Input mailbox:\n" );
		DBGC2_HDA ( hermon, virt_to_phys ( in_buffer ), in_buffer,
			    ( ( in_len < 512 ) ? in_len : 512 ) );
	}

	if ( event ) {
		evw = be32_to_cpu ( hcr.u.dwords[6] );
		evw |= 0x400000;
		hcr.u.dwords[6] = cpu_to_be32 ( evw );
	}

	/* Issue command */
	for ( i = 0 ; i < ( sizeof ( hcr ) / sizeof ( hcr.u.dwords[0] ) ) ;
	      i++ ) {
		writel ( hcr.u.dwords[i],
			 hermon->config + HERMON_HCR_REG ( i ) );
		barrier();
	}

	if ( event )
		return 0;

	/* Wait for command completion */
	if ( ( rc = hermon_cmd_wait ( hermon, &hcr ) ) != 0 ) {
		DBGC ( hermon, "ConnectX3 %p timed out waiting for command:\n",
		       hermon );
		DBGC_HDA ( hermon,
			   virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
			   &hcr, sizeof ( hcr ) );
		return rc;
	}

	/* fixes FW race following unmap FW area - ask Achiad */
	readl ( hermon->config + HERMON_HCR_REG ( 6 ) );

	/* Check command status */
	status = MLX_GET ( &hcr, status );
	if ( status != 0 ) {
		if ( ( ( opcode == HERMON_HCR_MOD_STAT_CFG ) && ( op_mod == 0xe ) ) ||
		     ( opcode == HERMON_HCR_INIT_DIAG_BUFFER ) ) {
			/* Could be as a result of missing TLV - print it as debug */
			DBGC ( hermon, "ConnectX3 %p command 0x%x failed with status %02x:\n",
				hermon, opcode, status );
		} else if ( opcode == HERMON_HCR_SENSE_PORT ) {
			DBGC ( hermon, "ConnectX3 %p command 0x%x failed with status %02x:\n",
				hermon, opcode, status );
			return status;
		} else {
			printf ( "ConnectX3 %p command 0x%x failed with status %02x:\n",
				hermon, opcode, status );
		}
		DBGC_HDA ( hermon,
			   virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
			   &hcr, sizeof ( hcr ) );
		return -EIO;
	}

	/* Read output parameters, if any */
	hcr.u.dwords[3] = readl ( hermon->config + HERMON_HCR_REG ( 3 ) );
	hcr.u.dwords[4] = readl ( hermon->config + HERMON_HCR_REG ( 4 ) );
	memcpy ( out, out_buffer, out_len );
	if ( out_len ) {
		DBGC2 ( hermon, "Output%s:\n",
			( command & HERMON_HCR_OUT_MBOX ) ? " mailbox" : "" );
		DBGC2_HDA ( hermon, virt_to_phys ( out_buffer ), out_buffer,
			    ( ( out_len < 512 ) ? out_len : 512 ) );
	}

	return 0;
}

static inline int
hermon_cmd_query_dev_cap ( struct hermon *hermon,
			   struct hermonprm_query_dev_cap *dev_cap ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_DEV_CAP,
						 1, sizeof ( *dev_cap ) ),
			    0, NULL, 0, dev_cap, 0 );
}

static inline int
hermon_cmd_query_adapter ( struct hermon *hermon, struct hermonprm_query_adapter *query_adapter ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_ADAPTER,
						 1, sizeof ( *query_adapter) ),
			    0, NULL, 0, query_adapter, 0 );
}

static inline int
hermon_cmd_query_fw ( struct hermon *hermon, struct hermonprm_query_fw *fw ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_FW,
						 1, sizeof ( *fw ) ),
			    0, NULL, 0, fw, 0 );
}

static inline int
hermon_cmd_init_hca ( struct hermon *hermon,
		      const struct hermonprm_init_hca *init_hca ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_INIT_HCA,
						1, sizeof ( *init_hca ) ),
			    0, init_hca, 0, NULL, 0 );
}

static inline int
hermon_cmd_close_hca ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_CLOSE_HCA ),
			    0, NULL, 0, NULL, 0 );
}

static inline int
hermon_cmd_post_doorbell ( struct hermon *hermon, unsigned int in_mod,
			const struct hermonprm_scalar_parameter *in) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_POST_DOORBELL,
			    0, 3 /* Take only the lower 3 bytes */),
			    0, in, in_mod, NULL, 0 );
}

static inline int
hermon_cmd_post_sqp_doorbell ( struct hermon *hermon, unsigned int qpn) {
	return hermon_cmd_post_doorbell ( hermon, ( qpn & 0x00ffffff ), NULL );
}

static inline int
hermon_cmd_post_eq_doorbell ( struct hermon *hermon, unsigned int eqn,
				unsigned int eq_ci) {
	struct hermonprm_scalar_parameter tmp;
	MLX_FILL_1 ( &tmp, 1, value, eq_ci );
#define POST_DB_RESRC_TYPE_EQ	0x03
#define POST_DB_RESRC_TYPE_BIT	24
	return hermon_cmd_post_doorbell ( hermon,
			( ( POST_DB_RESRC_TYPE_EQ << POST_DB_RESRC_TYPE_BIT ) |
			  ( eqn & 0x00ffffff ) ), &tmp);
}

static inline int
hermon_cmd_init_port ( struct hermon *hermon, unsigned int port ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_INIT_PORT ),
			    0, NULL, port, NULL, 0 );
}

static inline int
hermon_cmd_close_port ( struct hermon *hermon, unsigned int port ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_CLOSE_PORT ),
			    0, NULL, port, NULL, 0 );
}

int hermon_cmd_set_port ( struct hermon *hermon, uint16_t in_mod,
		const union hermonprm_set_port *set_port,
		unsigned int op_mod ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_SET_PORT,
						1, sizeof ( *set_port ) ),
			    op_mod, set_port, in_mod, NULL, 0 );
}

static inline int
hermon_cmd_sw2hw_mpt ( struct hermon *hermon, unsigned int index,
		       const struct hermonprm_mpt *mpt ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_SW2HW_MPT,
						1, sizeof ( *mpt ) ),
			    0, mpt, index, NULL, 0 );
}

static inline int
hermon_cmd_hw2sw_mpt ( struct hermon *hermon, unsigned int index ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_HW2SW_MPT),
			    0, NULL, index, NULL, 0 );
}

static inline int
hermon_cmd_write_mtt ( struct hermon *hermon,
		       const struct hermonprm_write_mtt *write_mtt ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_WRITE_MTT,
						1, sizeof ( *write_mtt ) ),
			    0, write_mtt, 1, NULL, 0 );
}

static inline int
hermon_cmd_map_eq ( struct hermon *hermon, unsigned long index_map,
		    const struct hermonprm_event_mask *mask ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_MAP_EQ,
						0, sizeof ( *mask ) ),
			    0, mask, index_map, NULL, 0 );
}

static inline int
hermon_cmd_sw2hw_eq ( struct hermon *hermon, unsigned int index,
		      const struct hermonprm_eqc *eqctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_SW2HW_EQ,
						1, sizeof ( *eqctx ) ),
			    0, eqctx, index, NULL, 0 );
}

static inline int
hermon_cmd_hw2sw_eq ( struct hermon *hermon, unsigned int index,
		      struct hermonprm_eqc *eqctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_HW2SW_EQ,
						 1, sizeof ( *eqctx ) ),
			    1, NULL, index, eqctx, 0 );
}

static inline int
hermon_cmd_sw2hw_cq ( struct hermon *hermon, unsigned long cqn,
		      const struct hermonprm_completion_queue_context *cqctx ){
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_SW2HW_CQ,
						1, sizeof ( *cqctx ) ),
			    0, cqctx, cqn, NULL, 0 );
}

static inline int
hermon_cmd_hw2sw_cq ( struct hermon *hermon, unsigned long cqn,
		      struct hermonprm_completion_queue_context *cqctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_HW2SW_CQ,
						 1, sizeof ( *cqctx ) ),
			    0, NULL, cqn, cqctx, 0 );
}

static inline int
hermon_cmd_rst2init_qp ( struct hermon *hermon, unsigned long qpn,
			 const struct hermonprm_qp_ee_state_transitions *ctx ){
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_RST2INIT_QP,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL, 0 );
}

static inline int
hermon_cmd_init2rtr_qp ( struct hermon *hermon, unsigned long qpn,
			 const struct hermonprm_qp_ee_state_transitions *ctx ){
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_INIT2RTR_QP,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL, 0 );
}

static inline int
hermon_cmd_rtr2rts_qp ( struct hermon *hermon, unsigned long qpn,
			const struct hermonprm_qp_ee_state_transitions *ctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_RTR2RTS_QP,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL, 0 );
}

static inline int
hermon_cmd_rts2rts_qp ( struct hermon *hermon, unsigned long qpn,
			const struct hermonprm_qp_ee_state_transitions *ctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_RTS2RTS_QP,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL, 0 );
}

static inline int
hermon_cmd_2rst_qp ( struct hermon *hermon, unsigned long qpn ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_2RST_QP ),
			    0x03, NULL, qpn, NULL, 0 );
}

static inline int
hermon_cmd_query_qp ( struct hermon *hermon, unsigned long qpn,
		      struct hermonprm_qp_ee_state_transitions *ctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_QP,
						 1, sizeof ( *ctx ) ),
			    0, NULL, qpn, ctx, 0 );
}

static inline int
hermon_cmd_conf_special_qp ( struct hermon *hermon, unsigned int internal_qps __unused,
			     unsigned long base_qpn ) {
	return hermon_cmd ( hermon, 0x0023, 0x01, NULL, base_qpn, NULL, 0 );
}


static inline int
hermon_cmd_mad_ifc ( struct hermon *hermon, unsigned int port,
		     union hermonprm_mad *mad ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_INOUT_CMD ( HERMON_HCR_MAD_IFC,
						   1, sizeof ( *mad ),
						   1, sizeof ( *mad ) ),
			    0x03, mad, port, mad, 0 );
}

static inline int
hermon_cmd_read_mcg ( struct hermon *hermon, uint8_t op_mod, unsigned int in_mod,
		      struct hermonprm_mcg_entry *mcg ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_READ_MCG,
						 1, sizeof ( *mcg ) ),
			    op_mod, NULL, in_mod, mcg, 0 );
}

static inline int
hermon_cmd_write_mcg ( struct hermon *hermon, uint8_t op_mod, unsigned int in_mod,
		       const struct hermonprm_mcg_entry *mcg ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_WRITE_MCG,
						1, sizeof ( *mcg ) ),
			    op_mod, mcg, in_mod, NULL, 0 );
}

static inline int
hermon_cmd_mgid_hash ( struct hermon *hermon, const union ib_gid *gid,
		       struct hermonprm_mgm_hash *hash, u8 protocol ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_INOUT_CMD ( HERMON_HCR_MGID_HASH,
						   1, sizeof ( *gid ),
						   0, sizeof ( *hash ) ),
			    protocol, gid, 0, hash, 0 );
}

static inline int
hermon_cmd_mod_stat_cfg ( struct hermon *hermon, unsigned int mode,
			  unsigned int input_mod,
			  struct hermonprm_scalar_parameter *portion ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_INOUT_CMD ( HERMON_HCR_MOD_STAT_CFG,
						   0, sizeof ( *portion ),
						   0, sizeof ( *portion ) ),
			    mode, portion, input_mod, portion, 0 );
}

static inline int
hermon_cmd_query_port ( struct hermon *hermon, unsigned int port,
			struct hermonprm_query_port_cap *query_port ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_PORT,
						 1, sizeof ( *query_port ) ),
			    0, NULL, port, query_port, 0 );
}

static inline int
hermon_cmd_sense_port ( struct hermon *hermon, unsigned int port,
			struct hermonprm_sense_port *port_type ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_SENSE_PORT,
						 0, sizeof ( *port_type ) ),
			    0, NULL, port, port_type, 0 );
}

static inline int
hermon_cmd_run_fw ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_RUN_FW ),
			    0, NULL, 0, NULL, 0 );
}

static inline int
hermon_cmd_unmap_icm ( struct hermon *hermon, unsigned int page_count,
		       const struct hermonprm_scalar_parameter *offset ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_UNMAP_ICM,
						0, sizeof ( *offset ) ),
			    0, offset, page_count, NULL, 0 );
}

static inline int
hermon_cmd_map_icm ( struct hermon *hermon,
		     const struct hermonprm_virtual_physical_mapping *map ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_MAP_ICM,
						1, sizeof ( *map ) ),
			    0, map, 1, NULL, 0 );
}

static inline int
hermon_cmd_unmap_icm_aux ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_UNMAP_ICM_AUX ),
			    0, NULL, 0, NULL, 0 );
}

static inline int
hermon_cmd_map_icm_aux ( struct hermon *hermon,
		       const struct hermonprm_virtual_physical_mapping *map ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_MAP_ICM_AUX,
						1, sizeof ( *map ) ),
			    0, map, 1, NULL, 0 );
}

static inline int
hermon_cmd_set_icm_size ( struct hermon *hermon,
			  const struct hermonprm_scalar_parameter *icm_size,
			  struct hermonprm_scalar_parameter *icm_aux_size ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_INOUT_CMD ( HERMON_HCR_SET_ICM_SIZE,
						   0, sizeof ( *icm_size ),
						   0, sizeof (*icm_aux_size) ),
			    0, icm_size, 0, icm_aux_size, 0 );
}

static inline int
hermon_cmd_unmap_fa ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_UNMAP_FA ),
			    0, NULL, 0, NULL, 0 );
}

static inline int
hermon_cmd_map_fa ( struct hermon *hermon,
		    const struct hermonprm_virtual_physical_mapping *map ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_MAP_FA,
						1, sizeof ( *map ) ),
			    0, map, 1, NULL, 0 );
}

static inline int
hermon_cmd_get_op_req ( struct hermon *hermon, unsigned int request_type,
			unsigned int report_status,
			struct hermonprm_get_op_req *get_op_req ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_GET_OP_REQ,
						1, sizeof ( *get_op_req ) ),
			    request_type, NULL, report_status, get_op_req, 0 );
}

static inline int
hermon_cmd_register_access ( struct hermon *hermon,
			struct hermonprm_register_access *register_access ) {
	return hermon_cmd ( hermon,
				    HERMON_HCR_INOUT_CMD ( HERMON_HCR_REGISTER_ACCESS,
							   1, sizeof ( *register_access ),
							   1, sizeof ( *register_access ) ),
				    0, register_access, 0, register_access, 0 );
}

static inline int
hermon_cmd_query_romini ( struct hermon *hermon,
			struct hermonprm_query_romini *ini ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_ROMINI,
						 1, sizeof ( *ini ) ),
			    0, NULL, 0, ini, 0 );
}

static inline int
hermon_cmd_query_defparams ( struct hermon *hermon,
			union hermonprm_query_defparams *def, unsigned int op_mod ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_DEFPARAMS,
						 1, sizeof ( *def ) ),
			    op_mod, NULL, 0, def, 0 );
}

#define HERMON_OPMOD_OCBB_EVENT_VAL 0x4
static inline int
hermon_cmd_init_diagnostic_buffer ( struct hermon *hermon, uint32_t in_mod, uint64_t *in ) {
	return hermon_cmd ( hermon, HERMON_HCR_IN_CMD ( HERMON_HCR_INIT_DIAG_BUFFER,
			0, sizeof ( *in ) ), HERMON_OPMOD_OCBB_EVENT_VAL,
			in, in_mod, NULL, 0 );
}

/***************************************************************************
 *
 * Memory translation table operations
 *
 ***************************************************************************
 */

/**
 * Allocate MTT entries
 *
 * @v hermon		ConnectX3 device
 * @v memory		Memory to map into MTT
 * @v len		Length of memory to map
 * @v mtt		MTT descriptor to fill in
 * @ret rc		Return status code
 */
static int hermon_alloc_mtt ( struct hermon *hermon,
			      const void *memory, size_t len,
			      struct hermon_mtt *mtt ) {
	struct hermonprm_write_mtt write_mtt;
	physaddr_t start;
	physaddr_t addr;
	unsigned int page_offset;
	unsigned int num_pages;
	int mtt_offset;
	unsigned int mtt_base_addr;
	unsigned int i;
	int rc;

	/* Find available MTT entries */
	start = virt_to_phys ( memory );
	page_offset = ( start & ( HERMON_PAGE_SIZE - 1 ) );
	start -= page_offset;
	len += page_offset;
	num_pages = ( ( len + HERMON_PAGE_SIZE - 1 ) / HERMON_PAGE_SIZE );
	mtt_offset = hermon_bitmask_alloc ( hermon->mtt_inuse, HERMON_MAX_MTTS,
					    num_pages );
	if ( mtt_offset < 0 ) {
		printf ( "ConnectX3 %p could not allocate %d MTT entries\n",
		       hermon, num_pages );
		rc = mtt_offset;
		goto err_mtt_offset;
	}
	mtt_base_addr = ( ( hermon->cap.reserved_mtts + mtt_offset ) *
			  hermon->cap.mtt_entry_size );
	addr = start;

	/* Fill in MTT structure */
	mtt->mtt_offset = mtt_offset;
	mtt->num_pages = num_pages;
	mtt->mtt_base_addr = mtt_base_addr;
	mtt->page_offset = page_offset;

	/* Construct and issue WRITE_MTT commands */
	for ( i = 0 ; i < num_pages ; i++ ) {
		memset ( &write_mtt, 0, sizeof ( write_mtt ) );
		MLX_FILL_1 ( &write_mtt.mtt_base_addr, 1,
			     value, mtt_base_addr );
		MLX_FILL_H ( &write_mtt.mtt, 0, ptag_h, addr );
		MLX_FILL_2 ( &write_mtt.mtt, 1,
			     p, 1,
			     ptag_l, ( addr >> 3 ) );
		if ( ( rc = hermon_cmd_write_mtt ( hermon,
						   &write_mtt ) ) != 0 ) {
			printf ( "ConnectX3 %p could not write MTT at %x\n",
			       hermon, mtt_base_addr );
			goto err_write_mtt;
		}
		addr += HERMON_PAGE_SIZE;
		mtt_base_addr += hermon->cap.mtt_entry_size;
	}

	DBGC ( hermon, "ConnectX3 %p MTT entries [%#x,%#x] for "
	       "[%08lx,%08lx,%08lx,%08lx)\n", hermon, mtt->mtt_offset,
	       ( mtt->mtt_offset + mtt->num_pages - 1 ), start,
	       ( start + page_offset ), ( start + len ), addr );

	return 0;

 err_write_mtt:
	hermon_bitmask_free ( hermon->mtt_inuse, mtt_offset, num_pages );
 err_mtt_offset:
	return rc;
}

/**
 * Free MTT entries
 *
 * @v hermon		ConnectX3 device
 * @v mtt		MTT descriptor
 */
static void hermon_free_mtt ( struct hermon *hermon,
			      struct hermon_mtt *mtt ) {

	DBGC ( hermon, "ConnectX3 %p MTT entries [%#x,%#x] freed\n",
	       hermon, mtt->mtt_offset,
	       ( mtt->mtt_offset + mtt->num_pages - 1 ) );
	hermon_bitmask_free ( hermon->mtt_inuse, mtt->mtt_offset,
			      mtt->num_pages );
}

/***************************************************************************
 *
 * Static configuration operations
 *
 ***************************************************************************
 */

/**
 * Calculate offset within static configuration
 *
 * @v field		Field
 * @ret offset		Offset
 */
#define HERMON_MOD_STAT_CFG_OFFSET( field )				     \
	( ( MLX_BIT_OFFSET ( struct hermonprm_mod_stat_cfg_st, field ) / 8 ) \
	  & ~( sizeof ( struct hermonprm_scalar_parameter ) - 1 ) )

/**
 * Query or modify static configuration
 *
 * @v hermon		ConnectX3 device
 * @v port		Port
 * @v mode		Command mode
 * @v offset		Offset within static configuration
 * @v stat_cfg		Static configuration
 * @ret rc		Return status code
 */
static int hermon_mod_stat_cfg ( struct hermon *hermon, unsigned int mport,
				 unsigned int mode, unsigned int offset,
				 unsigned int physical_function, unsigned int setup_mode,
				 struct hermonprm_mod_stat_cfg *stat_cfg ) {
	struct hermonprm_scalar_parameter *portion =
		( ( void * ) &stat_cfg->u.bytes[offset] );
	struct hermonprm_mod_stat_cfg_input_mod mod;
	int rc;

	/* Construct input modifier */
	memset ( &mod, 0, sizeof ( mod ) );
	MLX_FILL_4 ( &mod, 0,
		     portnum, mport,
		     offset, offset,
		     physical_function, physical_function,
		     setup_mode, setup_mode );

	/* Sanity check */
	assert ( ( offset % sizeof ( *portion ) ) == 0 );

	/* Issue command */
	if ( ( rc = hermon_cmd_mod_stat_cfg ( hermon, mode,
					      be32_to_cpu ( mod.u.dwords[0] ),
					      portion ) ) != 0 )
		return rc;

	return 0;
}

/***************************************************************************
 *
 * MAD operations
 *
 ***************************************************************************
 */

/**
 * Issue management datagram
 *
 * @v ibdev		Infiniband device
 * @v mad		Management datagram
 * @ret rc		Return status code
 */
static int hermon_mad ( struct ib_device *ibdev, union ib_mad *mad ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	union hermonprm_mad mad_ifc;
	int rc;

	linker_assert ( sizeof ( *mad ) == sizeof ( mad_ifc.mad ),
			mad_size_mismatch );

	/* Copy in request packet */
	memcpy ( &mad_ifc.mad, mad, sizeof ( mad_ifc.mad ) );

	/* Issue MAD */
	if ( ( rc = hermon_cmd_mad_ifc ( hermon, ibdev->port,
					 &mad_ifc ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not issue MAD IFC: "
		       "%s\n", hermon, ibdev->port, strerror ( rc ) );
		return rc;
	}

	/* Copy out reply packet */
	memcpy ( mad, &mad_ifc.mad, sizeof ( *mad ) );

	if ( mad->hdr.status != 0 ) {
		DBGC ( hermon, "ConnectX3 %p port %d MAD IFC status %04x\n",
		       hermon, ibdev->port, ntohs ( mad->hdr.status ) );
		return -EIO;
	}
	return 0;
}

/***************************************************************************
 *
 * Completion queue operations
 *
 ***************************************************************************
 */

/**
 * Create completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @ret rc		Return status code
 */
static int hermon_create_cq ( struct ib_device *ibdev,
			      struct ib_completion_queue *cq ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_completion_queue *hermon_cq;
	struct hermonprm_completion_queue_context cqctx;
	int cqn_offset;
	unsigned int i;
	int rc;

	/* Find a free completion queue number */
	cqn_offset = hermon_bitmask_alloc ( hermon->cq_inuse,
					    HERMON_MAX_CQS, 1 );
	if ( cqn_offset < 0 ) {
		DBGC ( hermon, "ConnectX3 %p out of completion queues\n",
		       hermon );
		rc = cqn_offset;
		goto err_cqn_offset;
	}
	cq->cqn = ( hermon->cap.reserved_cqs + cqn_offset );

	/* Allocate control structures */
	hermon_cq = zalloc ( sizeof ( *hermon_cq ) );
	if ( ! hermon_cq ) {
		rc = -ENOMEM;
		goto err_hermon_cq;
	}

	/* Allocate doorbell */
	hermon_cq->doorbell = malloc_dma( 2 * sizeof( *hermon_cq->set_ci_db ),
					  2 * sizeof( *hermon_cq->set_ci_db ) );
	if ( ! hermon_cq->doorbell ) {
		rc = -ENOMEM;
		goto err_doorbell;
	}

	hermon_cq->set_ci_db = (u32 *)hermon_cq->doorbell;
	hermon_cq->arm_db = &hermon_cq->set_ci_db[1];
	*hermon_cq->set_ci_db = 0;
	*hermon_cq->arm_db = 0;
	hermon_cq->arm_sn = 1;

	/* Allocate completion queue itself */
	hermon_cq->cqe_size = ( cq->num_cqes * sizeof ( hermon_cq->cqe[0] ) );
	hermon_cq->cqe = malloc_dma ( hermon_cq->cqe_size,
				      sizeof ( hermon_cq->cqe[0] ) );
	if ( ! hermon_cq->cqe ) {
		rc = -ENOMEM;
		goto err_cqe;
	}
	memset ( hermon_cq->cqe, 0, hermon_cq->cqe_size );
	for ( i = 0 ; i < cq->num_cqes ; i++ ) {
		MLX_FILL_1 ( &hermon_cq->cqe[i].normal, 7, owner, 1 );
	}
	barrier();

	/* Allocate MTT entries */
	if ( ( rc = hermon_alloc_mtt ( hermon, hermon_cq->cqe,
				       hermon_cq->cqe_size,
				       &hermon_cq->mtt ) ) != 0 )
		goto err_alloc_mtt;

	/* Hand queue over to hardware */
	memset ( &cqctx, 0, sizeof ( cqctx ) );
	MLX_FILL_1 ( &cqctx, 0, st, 0xa /* "Event fired" */ );
	MLX_FILL_1 ( &cqctx, 2,
		     page_offset, ( hermon_cq->mtt.page_offset >> 5 ) );
	MLX_FILL_2 ( &cqctx, 3,
		     usr_page, HERMON_UAR_NON_EQ_PAGE,
		     log_cq_size, fls ( cq->num_cqes - 1 ) );
	MLX_FILL_1 ( &cqctx, 5, c_eqn, hermon->eq.eqn );
	MLX_FILL_H ( &cqctx, 6, mtt_base_addr_h,
		     hermon_cq->mtt.mtt_base_addr );
	MLX_FILL_1 ( &cqctx, 7, mtt_base_addr_l,
		     ( hermon_cq->mtt.mtt_base_addr >> 3 ) );
	MLX_FILL_H ( &cqctx, 14, db_record_addr_h,
		     virt_to_phys ( hermon_cq->doorbell ) );
	MLX_FILL_1 ( &cqctx, 15, db_record_addr_l,
		     ( virt_to_phys ( hermon_cq->doorbell ) >> 3 ) );
	if ( ( rc = hermon_cmd_sw2hw_cq ( hermon, cq->cqn, &cqctx ) ) != 0 ) {
		printf ( "ConnectX3 %p CQN %#lx SW2HW_CQ failed: %s\n",
		       hermon, cq->cqn, strerror ( rc ) );
		goto err_sw2hw_cq;
	}

	DBGC ( hermon, "ConnectX3 %p CQN %#lx ring [%08lx,%08lx), doorbell "
	       "%08lx\n", hermon, cq->cqn, virt_to_phys ( hermon_cq->cqe ),
	       ( virt_to_phys ( hermon_cq->cqe ) + hermon_cq->cqe_size ),
	       virt_to_phys ( hermon_cq->doorbell ) );
	ib_cq_set_drvdata ( cq, hermon_cq );
	return 0;

 err_sw2hw_cq:
	hermon_free_mtt ( hermon, &hermon_cq->mtt );
 err_alloc_mtt:
	free_dma ( hermon_cq->cqe, hermon_cq->cqe_size );
 err_cqe:
	free_dma ( hermon_cq->doorbell, 2 * sizeof( *hermon_cq->set_ci_db ) );
 err_doorbell:
	free ( hermon_cq );
 err_hermon_cq:
	hermon_bitmask_free ( hermon->cq_inuse, cqn_offset, 1 );
 err_cqn_offset:
	return rc;
}

/**
 * Destroy completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void hermon_destroy_cq ( struct ib_device *ibdev,
				struct ib_completion_queue *cq ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_completion_queue *hermon_cq = ib_cq_get_drvdata ( cq );
	struct hermonprm_completion_queue_context cqctx;
	int cqn_offset;
	int rc;

	/* Take ownership back from hardware */
	if ( ( rc = hermon_cmd_hw2sw_cq ( hermon, cq->cqn, &cqctx ) ) != 0 ) {
		printf ( "ConnectX3 %p CQN %#lx FATAL HW2SW_CQ failed: "
		       "%s\n", hermon, cq->cqn, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}

	/* Free MTT entries */
	hermon_free_mtt ( hermon, &hermon_cq->mtt );

	/* Free memory */
	free_dma ( hermon_cq->cqe, hermon_cq->cqe_size );
	free_dma ( hermon_cq->doorbell, 2 * sizeof ( *hermon_cq->set_ci_db ) );
	free ( hermon_cq );

	/* Mark queue number as free */
	cqn_offset = ( cq->cqn - hermon->cap.reserved_cqs );
	hermon_bitmask_free ( hermon->cq_inuse, cqn_offset, 1 );

	ib_cq_set_drvdata ( cq, NULL );
}

/***************************************************************************
 *
 * Queue pair operations
 *
 ***************************************************************************
 */

/**
 * Assign queue pair number
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int hermon_alloc_qpn ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	unsigned int port_offset;
	int qpn_offset;

	/* Calculate queue pair number */
	port_offset = ( ibdev->port - HERMON_PORT_BASE );

	switch ( qp->type ) {
	case IB_QPT_SMI:
		qp->qpn = ( hermon->special_qpn_base + port_offset );
		return 0;
	case IB_QPT_GSI:
		qp->qpn = ( hermon->special_qpn_base + 2 + port_offset );
		return 0;
	case IB_QPT_UD:
	case IB_QPT_RC:
	case IB_QPT_ETH:
		/* Find a free queue pair number */
		qpn_offset = hermon_bitmask_alloc ( hermon->qp_inuse,
						    HERMON_MAX_QPS, 1 );
		if ( qpn_offset < 0 ) {
			DBGC ( hermon, "ConnectX3 %p out of queue pairs\n",
			       hermon );
			return qpn_offset;
		}
		qp->qpn = ( ( random() & HERMON_QPN_RANDOM_MASK ) |
			    ( hermon->qpn_base + qpn_offset ) );
		return 0;
	default:
		printf ( "ConnectX3 %p unsupported QP type %d\n",
		       hermon, qp->type );
		return -ENOTSUP;
	}
}

/**
 * Free queue pair number
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void hermon_free_qpn ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	int qpn_offset;

	qpn_offset = ( ( qp->qpn & ~HERMON_QPN_RANDOM_MASK )
		       - hermon->qpn_base );
	if ( qpn_offset >= 0 )
		hermon_bitmask_free ( hermon->qp_inuse, qpn_offset, 1 );
}

/**
 * Calculate transmission rate
 *
 * @v av		Address vector
 * @ret hermon_rate	ConnectX3 rate
 */
static unsigned int hermon_rate ( struct ib_address_vector *av ) {
	return ( ( ( av->rate >= IB_RATE_2_5 ) && ( av->rate <= IB_RATE_120 ) )
		 ? ( av->rate + 5 ) : 0 );
}

/**
 * Calculate schedule queue
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret sched_queue	Schedule queue
 */
static unsigned int hermon_sched_queue ( struct ib_device *ibdev,
					 struct ib_queue_pair *qp ) {
	return ( ( ( qp->type == IB_QPT_SMI ) ?
		   HERMON_SCHED_QP0 : HERMON_SCHED_DEFAULT ) |
		 ( ( ibdev->port - 1 ) << 6 ) );
}

/** Queue pair transport service type map */
static uint8_t hermon_qp_st[] = {
	[IB_QPT_SMI] = HERMON_ST_MLX,
	[IB_QPT_GSI] = HERMON_ST_MLX,
	[IB_QPT_UD] = HERMON_ST_UD,
	[IB_QPT_RC] = HERMON_ST_RC,
	[IB_QPT_ETH] = HERMON_ST_MLX,
};

/**
 * Create queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int hermon_create_qp ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_queue_pair *hermon_qp;
	struct hermonprm_qp_ee_state_transitions qpctx;
	int rc;

	/* Calculate queue pair number */
	if ( ( rc = hermon_alloc_qpn ( ibdev, qp ) ) != 0 )
		goto err_alloc_qpn;

	/* Allocate control structures */
	hermon_qp = zalloc ( sizeof ( *hermon_qp ) );
	if ( ! hermon_qp ) {
		rc = -ENOMEM;
		goto err_hermon_qp;
	}

	/* Allocate doorbells */
	hermon_qp->recv.doorbell =
		malloc_dma ( sizeof ( hermon_qp->recv.doorbell[0] ),
			     sizeof ( hermon_qp->recv.doorbell[0] ) );
	if ( ! hermon_qp->recv.doorbell ) {
		rc = -ENOMEM;
		goto err_recv_doorbell;
	}
	memset ( hermon_qp->recv.doorbell, 0,
		 sizeof ( hermon_qp->recv.doorbell[0] ) );
	hermon_qp->send.doorbell =
		( hermon->uar + HERMON_UAR_NON_EQ_PAGE * HERMON_PAGE_SIZE +
		  HERMON_DB_POST_SND_OFFSET );

	/* Allocate work queue buffer */
	hermon_qp->send.num_wqes = ( qp->send.num_wqes /* headroom */ + 1 +
				( 2048 / sizeof ( hermon_qp->send.wqe[0] ) ) );
	hermon_qp->send.num_wqes =
		( 1 << fls ( hermon_qp->send.num_wqes - 1 ) ); /* round up */
	hermon_qp->send.wqe_size = ( hermon_qp->send.num_wqes *
				     sizeof ( hermon_qp->send.wqe[0] ) );
	hermon_qp->recv.wqe_size = ( qp->recv.num_wqes *
				     sizeof ( hermon_qp->recv.wqe[0] ) );
	hermon_qp->wqe_size = ( hermon_qp->send.wqe_size +
				hermon_qp->recv.wqe_size );
	hermon_qp->wqe = malloc_dma ( hermon_qp->wqe_size,
				      sizeof ( hermon_qp->send.wqe[0] ) );
	if ( ! hermon_qp->wqe ) {
		rc = -ENOMEM;
		goto err_alloc_wqe;
	}
	hermon_qp->send.wqe = hermon_qp->wqe;
	memset ( hermon_qp->send.wqe, 0xff, hermon_qp->send.wqe_size );
	hermon_qp->recv.wqe = ( hermon_qp->wqe + hermon_qp->send.wqe_size );
	memset ( hermon_qp->recv.wqe, 0, hermon_qp->recv.wqe_size );

	/* Allocate MTT entries */
	if ( ( rc = hermon_alloc_mtt ( hermon, hermon_qp->wqe,
				       hermon_qp->wqe_size,
				       &hermon_qp->mtt ) ) != 0 ) {
		goto err_alloc_mtt;
	}

	/* Transition queue to INIT state */
	memset ( &qpctx, 0, sizeof ( qpctx ) );
	MLX_FILL_2 ( &qpctx, 2,
		     qpc_eec_data.pm_state, HERMON_PM_STATE_MIGRATED,
		     qpc_eec_data.st, hermon_qp_st[qp->type] );
	MLX_FILL_1 ( &qpctx, 3, qpc_eec_data.pd, HERMON_GLOBAL_PD );
	MLX_FILL_4 ( &qpctx, 4,
		     qpc_eec_data.log_rq_size, fls ( qp->recv.num_wqes - 1 ),
		     qpc_eec_data.log_rq_stride,
		     ( fls ( sizeof ( hermon_qp->recv.wqe[0] ) - 1 ) - 4 ),
		     qpc_eec_data.log_sq_size,
		     fls ( hermon_qp->send.num_wqes - 1 ),
		     qpc_eec_data.log_sq_stride,
		     ( fls ( sizeof ( hermon_qp->send.wqe[0] ) - 1 ) - 4 ) );
	MLX_FILL_1 ( &qpctx, 5,
		     qpc_eec_data.usr_page, HERMON_UAR_NON_EQ_PAGE );
	MLX_FILL_1 ( &qpctx, 33, qpc_eec_data.cqn_snd, qp->send.cq->cqn );
	MLX_FILL_4 ( &qpctx, 38,
		     qpc_eec_data.rre, 1,
		     qpc_eec_data.rwe, 1,
		     qpc_eec_data.rae, 1,
		     qpc_eec_data.page_offset,
		     ( hermon_qp->mtt.page_offset >> 6 ) );
	MLX_FILL_1 ( &qpctx, 41, qpc_eec_data.cqn_rcv, qp->recv.cq->cqn );
	MLX_FILL_H ( &qpctx, 42, qpc_eec_data.db_record_addr_h,
		     virt_to_phys ( hermon_qp->recv.doorbell ) );
	MLX_FILL_1 ( &qpctx, 43, qpc_eec_data.db_record_addr_l,
		     ( virt_to_phys ( hermon_qp->recv.doorbell ) >> 2 ) );
	MLX_FILL_H ( &qpctx, 52, qpc_eec_data.mtt_base_addr_h,
		     hermon_qp->mtt.mtt_base_addr );
	MLX_FILL_1 ( &qpctx, 53, qpc_eec_data.mtt_base_addr_l,
		     ( hermon_qp->mtt.mtt_base_addr >> 3 ) );

	DBGC ( hermon, "ConnectX3 %p QPN %#lx send ring [%08lx,%08lx), doorbell "
			"%08lx\n", hermon, qp->qpn,	virt_to_phys ( hermon_qp->send.wqe ),
			( virt_to_phys ( hermon_qp->send.wqe ) + hermon_qp->send.wqe_size ),
			virt_to_phys ( hermon_qp->send.doorbell ) );

	if ( ( rc = hermon_cmd_rst2init_qp ( hermon, qp->qpn,
					     &qpctx ) ) != 0 ) {
		printf ( "ConnectX3 %p QPN %#lx RST2INIT_QP failed: %s\n",
		       hermon, qp->qpn, strerror ( rc ) );
		goto err_rst2init_qp;
	}
	hermon_qp->state = HERMON_QP_ST_INIT;

	DBGC ( hermon, "ConnectX3 %p QPN %#lx send ring [%08lx,%08lx), doorbell "
	       "%08lx\n", hermon, qp->qpn,
	       virt_to_phys ( hermon_qp->send.wqe ),
	       ( virt_to_phys ( hermon_qp->send.wqe ) +
		 hermon_qp->send.wqe_size ),
	       virt_to_phys ( hermon_qp->send.doorbell ) );
	DBGC ( hermon, "ConnectX3 %p QPN %#lx receive ring [%08lx,%08lx), "
	       "doorbell %08lx\n", hermon, qp->qpn,
	       virt_to_phys ( hermon_qp->recv.wqe ),
	       ( virt_to_phys ( hermon_qp->recv.wqe ) +
		 hermon_qp->recv.wqe_size ),
	       virt_to_phys ( hermon_qp->recv.doorbell ) );
	DBGC ( hermon, "ConnectX3 %p QPN %#lx send CQN %#lx receive CQN %#lx\n",
	       hermon, qp->qpn, qp->send.cq->cqn, qp->recv.cq->cqn );
	ib_qp_set_drvdata ( qp, hermon_qp );
	return 0;

	hermon_cmd_2rst_qp ( hermon, qp->qpn );
 err_rst2init_qp:
	hermon_free_mtt ( hermon, &hermon_qp->mtt );
 err_alloc_mtt:
	free_dma ( hermon_qp->wqe, hermon_qp->wqe_size );
 err_alloc_wqe:
	free_dma ( hermon_qp->recv.doorbell,
		   sizeof ( hermon_qp->recv.doorbell[0] ) );
 err_recv_doorbell:
	free ( hermon_qp );
 err_hermon_qp:
	hermon_free_qpn ( ibdev, qp );
 err_alloc_qpn:
	return rc;
}

/**
 * Modify queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int hermon_modify_qp ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_queue_pair *hermon_qp = ib_qp_get_drvdata ( qp );
	struct hermonprm_qp_ee_state_transitions qpctx;
	int rc;

	/* Transition queue to RTR state, if applicable */
	if ( hermon_qp->state < HERMON_QP_ST_RTR ) {
		memset ( &qpctx, 0, sizeof ( qpctx ) );
		MLX_FILL_1 ( &qpctx, 0,
				 opt_param_mask, 0x10 );
		MLX_FILL_2 ( &qpctx, 4,
			     qpc_eec_data.mtu,
			     ( ( qp->type == IB_QPT_ETH ) ?
			       HERMON_MTU_ETH : HERMON_MTU_2048 ),
			     qpc_eec_data.msg_max, 31 );
		MLX_FILL_1 ( &qpctx, 7,
			     qpc_eec_data.remote_qpn_een, qp->av.qpn );
		MLX_FILL_1 ( &qpctx, 8,
				 qpc_eec_data.primary_address_path.pkey_index,
				 ibdev->pkey_index );
		MLX_FILL_1 ( &qpctx, 9,
			     qpc_eec_data.primary_address_path.rlid,
			     qp->av.lid );
		MLX_FILL_1 ( &qpctx, 10,
			     qpc_eec_data.primary_address_path.max_stat_rate,
			     hermon_rate ( &qp->av ) );
		memcpy ( &qpctx.u.dwords[12], &qp->av.gid,
			 sizeof ( qp->av.gid ) );
		MLX_FILL_1 ( &qpctx, 16,
			     qpc_eec_data.primary_address_path.sched_queue,
			     hermon_sched_queue ( ibdev, qp ) );
		MLX_FILL_1 ( &qpctx, 38,
			     qpc_eec_data.physical_function,
			     hermon->physical_function );
		MLX_FILL_1 ( &qpctx, 39,
			     qpc_eec_data.next_rcv_psn, qp->recv.psn );
		if ( ( rc = hermon_cmd_init2rtr_qp ( hermon, qp->qpn,
						     &qpctx ) ) != 0 ) {
			printf ( "ConnectX3 %p QPN %#lx INIT2RTR_QP failed:"
			       " %s\n", hermon, qp->qpn, strerror ( rc ) );
			return rc;
		}
		hermon_qp->state = HERMON_QP_ST_RTR;
	}

	/* Transition queue to RTS state */
	if ( hermon_qp->state < HERMON_QP_ST_RTS ) {
		memset ( &qpctx, 0, sizeof ( qpctx ) );
		MLX_FILL_1 ( &qpctx, 10,
			     qpc_eec_data.primary_address_path.ack_timeout,
			     14 /* 4.096us * 2^(14) = 67ms */ );
		MLX_FILL_2 ( &qpctx, 30,
			     qpc_eec_data.retry_count, HERMON_RETRY_MAX,
			     qpc_eec_data.rnr_retry, HERMON_RETRY_MAX );
		MLX_FILL_1 ( &qpctx, 32,
			     qpc_eec_data.next_send_psn, qp->send.psn );
		MLX_FILL_1 ( &qpctx, 38,
			     qpc_eec_data.physical_function,
			     hermon->physical_function );
		if ( ( rc = hermon_cmd_rtr2rts_qp ( hermon, qp->qpn,
						    &qpctx ) ) != 0 ) {
			printf ( "ConnectX3 %p QPN %#lx RTR2RTS_QP failed: "
			       "%s\n", hermon, qp->qpn, strerror ( rc ) );
			return rc;
		}
		hermon_qp->state = HERMON_QP_ST_RTS;
	}

	/* Update parameters in RTS state */
	memset ( &qpctx, 0, sizeof ( qpctx ) );
	MLX_FILL_1 ( &qpctx, 0, opt_param_mask, HERMON_QP_OPT_PARAM_QKEY );
	MLX_FILL_1 ( &qpctx, 38,
			     qpc_eec_data.physical_function,
			     hermon->physical_function );
	MLX_FILL_1 ( &qpctx, 44, qpc_eec_data.q_key, qp->qkey );
	if ( ( rc = hermon_cmd_rts2rts_qp ( hermon, qp->qpn, &qpctx ) ) != 0 ){
		printf ( "ConnectX3 %p QPN %#lx RTS2RTS_QP failed: %s\n",
		       hermon, qp->qpn, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Destroy queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void hermon_destroy_qp ( struct ib_device *ibdev,
				struct ib_queue_pair *qp ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_queue_pair *hermon_qp = ib_qp_get_drvdata ( qp );
	int rc;

	/* Take ownership back from hardware */
	if ( ( rc = hermon_cmd_2rst_qp ( hermon, qp->qpn ) ) != 0 ) {
		printf ( "ConnectX3 %p QPN %#lx FATAL 2RST_QP failed: %s\n",
		       hermon, qp->qpn, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}

	/* Free MTT entries */
	hermon_free_mtt ( hermon, &hermon_qp->mtt );

	/* Free memory */
	free_dma ( hermon_qp->wqe, hermon_qp->wqe_size );
	free_dma ( hermon_qp->recv.doorbell,
		   sizeof ( hermon_qp->recv.doorbell[0] ) );
	free ( hermon_qp );

	/* Mark queue number as free */
	hermon_free_qpn ( ibdev, qp );

	ib_qp_set_drvdata ( qp, NULL );
}

/***************************************************************************
 *
 * Work request operations
 *
 ***************************************************************************
 */

/**
 * Construct UD send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @v wqe		Send work queue entry
 * @ret opcode		Control opcode
 */
static unsigned int
hermon_fill_ud_send_wqe ( struct ib_device *ibdev,
			  struct ib_queue_pair *qp __unused,
			  struct ib_address_vector *av,
			  struct io_buffer *iobuf,
			  union hermon_send_wqe *wqe ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );

	MLX_FILL_1 ( &wqe->ud.ctrl, 1, ds,
		     ( ( offsetof ( typeof ( wqe->ud ), data[1] ) / 16 ) ) );
	MLX_FILL_1 ( &wqe->ud.ctrl, 2, c, 0x03 /* generate completion */ );
	MLX_FILL_2 ( &wqe->ud.ud, 0,
		     ud_address_vector.pd, HERMON_GLOBAL_PD,
		     ud_address_vector.port_number, ibdev->port );
	MLX_FILL_2 ( &wqe->ud.ud, 1,
		     ud_address_vector.rlid, av->lid,
		     ud_address_vector.g, av->gid_present );
	MLX_FILL_1 ( &wqe->ud.ud, 2,
		     ud_address_vector.max_stat_rate, hermon_rate ( av ) );
	MLX_FILL_1 ( &wqe->ud.ud, 3, ud_address_vector.sl, av->sl );
	memcpy ( &wqe->ud.ud.u.dwords[4], &av->gid, sizeof ( av->gid ) );
	MLX_FILL_1 ( &wqe->ud.ud, 8, destination_qp, av->qpn );
	MLX_FILL_1 ( &wqe->ud.ud, 9, q_key, av->qkey );
	MLX_FILL_1 ( &wqe->ud.data[0], 0, byte_count, iob_len ( iobuf ) );
	MLX_FILL_1 ( &wqe->ud.data[0], 1, l_key, hermon->lkey );
	MLX_FILL_H ( &wqe->ud.data[0], 2,
		     local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &wqe->ud.data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );
	return HERMON_OPCODE_SEND;
}

/**
 * Construct MLX send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @v wqe		Send work queue entry
 * @ret opcode		Control opcode
 */
static unsigned int
hermon_fill_mlx_send_wqe ( struct ib_device *ibdev,
			   struct ib_queue_pair *qp,
			   struct ib_address_vector *av,
			   struct io_buffer *iobuf,
			   union hermon_send_wqe *wqe ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct io_buffer headers;

	/* Construct IB headers */
	iob_populate ( &headers, &wqe->mlx.headers, 0,
		       sizeof ( wqe->mlx.headers ) );
	iob_reserve ( &headers, sizeof ( wqe->mlx.headers ) );
	ib_push ( ibdev, &headers, qp, iob_len ( iobuf ), av );

	/* Fill work queue entry */
	MLX_FILL_1 ( &wqe->mlx.ctrl, 1, ds,
		     ( ( offsetof ( typeof ( wqe->mlx ), data[2] ) / 16 ) ) );
	MLX_FILL_5 ( &wqe->mlx.ctrl, 2,
		     c, 0x03 /* generate completion */,
		     icrc, 0 /* generate ICRC */,
		     max_statrate, hermon_rate ( av ),
		     slr, 0,
		     v15, ( ( qp->ext_qpn == IB_QPN_SMI ) ? 1 : 0 ) );
	MLX_FILL_1 ( &wqe->mlx.ctrl, 3, rlid, av->lid );
	MLX_FILL_1 ( &wqe->mlx.data[0], 0,
		     byte_count, iob_len ( &headers ) );
	MLX_FILL_1 ( &wqe->mlx.data[0], 1, l_key, hermon->lkey );
	MLX_FILL_H ( &wqe->mlx.data[0], 2,
		     local_address_h, virt_to_bus ( headers.data ) );
	MLX_FILL_1 ( &wqe->mlx.data[0], 3,
		     local_address_l, virt_to_bus ( headers.data ) );
	MLX_FILL_1 ( &wqe->mlx.data[1], 0,
		     byte_count, ( iob_len ( iobuf ) + 4 /* ICRC */ ) );
	MLX_FILL_1 ( &wqe->mlx.data[1], 1, l_key, hermon->lkey );
	MLX_FILL_H ( &wqe->mlx.data[1], 2,
		     local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &wqe->mlx.data[1], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );
	return HERMON_OPCODE_SEND;
}

/**
 * Construct RC send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @v wqe		Send work queue entry
 * @ret opcode		Control opcode
 */
static unsigned int
hermon_fill_rc_send_wqe ( struct ib_device *ibdev,
			  struct ib_queue_pair *qp __unused,
			  struct ib_address_vector *av __unused,
			  struct io_buffer *iobuf,
			  union hermon_send_wqe *wqe ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );

	MLX_FILL_1 ( &wqe->rc.ctrl, 1, ds,
		     ( ( offsetof ( typeof ( wqe->rc ), data[1] ) / 16 ) ) );
	MLX_FILL_1 ( &wqe->rc.ctrl, 2, c, 0x03 /* generate completion */ );
	MLX_FILL_1 ( &wqe->rc.data[0], 0, byte_count, iob_len ( iobuf ) );
	MLX_FILL_1 ( &wqe->rc.data[0], 1, l_key, hermon->lkey );
	MLX_FILL_H ( &wqe->rc.data[0], 2,
		     local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &wqe->rc.data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );
	return HERMON_OPCODE_SEND;
}

/**
 * Construct Ethernet send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @v wqe		Send work queue entry
 * @ret opcode		Control opcode
 */
static unsigned int
hermon_fill_eth_send_wqe ( struct ib_device *ibdev,
			   struct ib_queue_pair *qp __unused,
			   struct ib_address_vector *av __unused,
			   struct io_buffer *iobuf,
			   union hermon_send_wqe *wqe ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );

	/* Fill work queue entry */
	MLX_FILL_1 ( &wqe->eth.ctrl, 1, ds,
		     ( ( offsetof ( typeof ( wqe->mlx ), data[1] ) / 16 ) ) );
	MLX_FILL_2 ( &wqe->eth.ctrl, 2,
		     c, 0x03 /* generate completion */,
		     s, 1 /* inhibit ICRC */ );
	MLX_FILL_1 ( &wqe->eth.data[0], 0,
		     byte_count, iob_len ( iobuf ) );
	MLX_FILL_1 ( &wqe->eth.data[0], 1, l_key, hermon->lkey );
	MLX_FILL_H ( &wqe->eth.data[0], 2,
		     local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &wqe->eth.data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );
	return HERMON_OPCODE_SEND;
}

/** Work queue entry constructors */
static unsigned int
( * hermon_fill_send_wqe[] ) ( struct ib_device *ibdev,
			       struct ib_queue_pair *qp,
			       struct ib_address_vector *av,
			       struct io_buffer *iobuf,
			       union hermon_send_wqe *wqe ) = {
	[IB_QPT_SMI] = hermon_fill_mlx_send_wqe,
	[IB_QPT_GSI] = hermon_fill_mlx_send_wqe,
	[IB_QPT_UD] = hermon_fill_ud_send_wqe,
	[IB_QPT_RC] = hermon_fill_rc_send_wqe,
	[IB_QPT_ETH] = hermon_fill_eth_send_wqe,
};

/**
 * Post send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int hermon_post_send ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp,
			      struct ib_address_vector *av,
			      struct io_buffer *iobuf ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_queue_pair *hermon_qp = ib_qp_get_drvdata ( qp );
	struct ib_work_queue *wq = &qp->send;
	struct hermon_send_work_queue *hermon_send_wq = &hermon_qp->send;
	union hermon_send_wqe *wqe;
	union hermonprm_doorbell_register db_reg;
	unsigned long wqe_idx_mask;
	unsigned long wqe_idx;
	unsigned int owner;
	unsigned int opcode;
	int rc;

	/* Allocate work queue entry */
	wqe_idx = ( wq->next_idx & ( hermon_send_wq->num_wqes - 1 ) );
	owner = ( ( wq->next_idx & hermon_send_wq->num_wqes ) ? 1 : 0 );
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[ wqe_idx & wqe_idx_mask ] ) {
		DBGC ( hermon, "ConnectX3 %p QPN %#lx send queue full",
		       hermon, qp->qpn );
		return -ENOBUFS;
	}
	wq->iobufs[ wqe_idx & wqe_idx_mask ] = iobuf;
	wqe = &hermon_send_wq->wqe[wqe_idx];

	/* Construct work queue entry */
	memset ( ( ( ( void * ) wqe ) + 4 /* avoid ctrl.owner */ ), 0,
		   ( sizeof ( *wqe ) - 4 ) );
	assert ( qp->type < ( sizeof ( hermon_fill_send_wqe ) /
			      sizeof ( hermon_fill_send_wqe[0] ) ) );
	assert ( hermon_fill_send_wqe[qp->type] != NULL );
	opcode = hermon_fill_send_wqe[qp->type] ( ibdev, qp, av, iobuf, wqe );
	barrier();
	MLX_FILL_2 ( &wqe->ctrl, 0,
		     opcode, opcode,
		     owner, owner );
	DBGCP ( hermon, "ConnectX3 %p QPN %#lx posting send WQE %#lx:\n",
		hermon, qp->qpn, wqe_idx );
	DBGCP_HDA ( hermon, virt_to_phys ( wqe ), wqe, sizeof ( *wqe ) );

	/* Ring doorbell register */
	MLX_FILL_1 ( &db_reg.send, 0, qn, qp->qpn );
	barrier();
	if ( hermon->cap.cmdif_post_doorbell ) {
		if ( (rc = hermon_cmd_post_sqp_doorbell ( hermon, qp->qpn ) ) != 0 ) {
			printf ( "Failed to post doorbell command on ConnectX3 %p QPN %#lx ",
				hermon, qp->qpn );
			return rc;
		}
	} else {
		writel ( db_reg.dword[0], hermon_send_wq->doorbell );
	}

	/* Update work queue's index */
	wq->next_idx++;

	return 0;
}

/**
 * Post receive work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int hermon_post_recv ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp,
			      struct io_buffer *iobuf ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_queue_pair *hermon_qp = ib_qp_get_drvdata ( qp );
	struct ib_work_queue *wq = &qp->recv;
	struct hermon_recv_work_queue *hermon_recv_wq = &hermon_qp->recv;
	struct hermonprm_recv_wqe *wqe;
	unsigned int wqe_idx_mask;

	/* Allocate work queue entry */
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[wq->next_idx & wqe_idx_mask] ) {
		DBGC ( hermon, "ConnectX3 %p QPN %#lx receive queue full",
		       hermon, qp->qpn );
		return -ENOBUFS;
	}
	wq->iobufs[wq->next_idx & wqe_idx_mask] = iobuf;
	wqe = &hermon_recv_wq->wqe[wq->next_idx & wqe_idx_mask].recv;

	/* Construct work queue entry */
	MLX_FILL_1 ( &wqe->data[0], 0, byte_count, iob_tailroom ( iobuf ) );
	MLX_FILL_1 ( &wqe->data[0], 1, l_key, hermon->lkey );
	MLX_FILL_H ( &wqe->data[0], 2,
		     local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &wqe->data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );

	/* Update work queue's index */
	wq->next_idx++;

	/* Update doorbell record */
	barrier();
	MLX_FILL_1 ( hermon_recv_wq->doorbell, 0, receive_wqe_counter,
		     ( wq->next_idx & 0xffff ) );

	return 0;
}

/**
 * Handle completion
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @v cqe		Hardware completion queue entry
 * @ret rc		Return status code
 */
static int hermon_complete ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq,
			     union hermonprm_completion_entry *cqe ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct ib_work_queue *wq;
	struct ib_queue_pair *qp;
	struct io_buffer *iobuf;
	struct ib_address_vector recv_dest;
	struct ib_address_vector recv_source;
	struct ib_global_route_header *grh;
	struct ib_address_vector *source;
	struct hermonprm_qp_ee_state_transitions qp_ctx;
	struct hermon_queue_pair *hermon_qp;
	unsigned int opcode;
	unsigned long qpn;
	int is_send;
	unsigned long wqe_idx;
	unsigned long wqe_idx_mask;
	size_t len;
	int rc = 0;

	/* Parse completion */
	qpn = MLX_GET ( &cqe->normal, qpn );
	is_send = MLX_GET ( &cqe->normal, s_r );
	opcode = MLX_GET ( &cqe->normal, opcode );
	if ( opcode == HERMON_OPCODE_CQE_ERROR ) {
		DBGC ( hermon, "ConnectX3 %p CQN %#lx syndrome %x vendor %x\n",
		       hermon, cq->cqn, MLX_GET ( &cqe->error, syndrome ),
		       MLX_GET ( &cqe->error, vendor_error_syndrome ) );
		rc = -EIO;
		/* Don't return immediately; propagate error to completer */
	}

	/* Identify work queue */
	wq = ib_find_wq ( cq, qpn, is_send );
	if ( ! wq ) {
		DBGC ( hermon, "ConnectX3 %p CQN %#lx unknown %s QPN %#lx\n",
		       hermon, cq->cqn, ( is_send ? "send" : "recv" ), qpn );
		return -EIO;
	}
	qp = wq->qp;

	/* Identify work queue entry */
	wqe_idx = MLX_GET ( &cqe->normal, wqe_counter );
	wqe_idx_mask = ( wq->num_wqes - 1 );
	DBGCP ( hermon, "ConnectX3 %p CQN %#lx QPN %#lx %s WQE %#lx completed:\n",
		hermon, cq->cqn, qp->qpn, ( is_send ? "send" : "recv" ),
		wqe_idx );
	DBGCP_HDA ( hermon, virt_to_phys ( cqe ), cqe, sizeof ( *cqe ) );

	/* Identify I/O buffer */
	iobuf = wq->iobufs[ wqe_idx & wqe_idx_mask ];
	if ( ! iobuf ) {
		DBGC ( hermon, "ConnectX3 %p CQN %#lx QPN %#lx empty %s WQE "
		       "%#lx\n", hermon, cq->cqn, qp->qpn,
		       ( is_send ? "send" : "recv" ), wqe_idx );
		return -EIO;
	}
	wq->iobufs[ wqe_idx & wqe_idx_mask ] = NULL;

	if ( is_send ) {
		/* Hand off to completion handler */
		ib_complete_send ( ibdev, qp, iobuf, rc );
	} else if ( rc ) {
		/* Query the QP state */
		hermon_qp = ib_qp_get_drvdata ( qp );
		memset ( &qp_ctx, 0, sizeof ( qp_ctx ) );
		if ( 0 == hermon_cmd_query_qp ( hermon, qp->qpn, &qp_ctx ) ) {
			hermon_qp->state = MLX_GET( &qp_ctx, qpc_eec_data.state );
			DBGC ( hermon, "ConnectX3 %p QPN %#lx state is %d\n",
					hermon, qp->qpn, hermon_qp->state );
		}
		/* Propogate error to receive completion handler */
		ib_complete_recv ( ibdev, qp, NULL, NULL, iobuf, rc );
	} else {
		/* Set received length */
		len = MLX_GET ( &cqe->normal, byte_cnt );
		assert ( len <= iob_tailroom ( iobuf ) );
		iob_put ( iobuf, len );
		memset ( &recv_dest, 0, sizeof ( recv_dest ) );
		recv_dest.qpn = qpn;
		memset ( &recv_source, 0, sizeof ( recv_source ) );
		switch ( qp->type ) {
		case IB_QPT_SMI:
		case IB_QPT_GSI:
		case IB_QPT_UD:
			assert ( iob_len ( iobuf ) >= sizeof ( *grh ) );
			grh = iobuf->data;
			iob_pull ( iobuf, sizeof ( *grh ) );
			/* Construct address vector */
			source = &recv_source;
			source->qpn = MLX_GET ( &cqe->normal, srq_rqpn );
			source->lid = MLX_GET ( &cqe->normal, slid_smac47_32 );
			source->sl = MLX_GET ( &cqe->normal, sl );
			recv_dest.gid_present = source->gid_present =
				MLX_GET ( &cqe->normal, g );
			memcpy ( &recv_dest.gid, &grh->dgid,
				 sizeof ( recv_dest.gid ) );
			memcpy ( &source->gid, &grh->sgid,
				 sizeof ( source->gid ) );
			break;
		case IB_QPT_RC:
			source = &qp->av;
			break;
		case IB_QPT_ETH:
			/* Construct address vector */
			source = &recv_source;
			source->vlan_present = MLX_GET ( &cqe->normal, vlan );
			source->vlan = MLX_GET ( &cqe->normal, vid );
			break;
		default:
			assert ( 0 );
			return -EINVAL;
		}
		/* Hand off to completion handler */
		ib_complete_recv ( ibdev, qp, &recv_dest, source, iobuf, rc );
	}

	return rc;
}

static void arm_cq ( struct hermon *hermon, struct ib_completion_queue *cq ) {
	struct hermon_completion_queue *hcq = ib_cq_get_drvdata ( cq );
	u32 doorbell[2];
	u32 sn;
	u32 ci;
	void *addr = hermon->uar + HERMON_UAR_NON_EQ_PAGE * HERMON_PAGE_SIZE + MLX4_CQ_DOORBELL;

	sn = hcq->arm_sn & 3;
	ci = cq->next_idx & 0xffffff;

	*hcq->arm_db = cpu_to_be32 ( sn << 28 | MLX4_CQ_DB_REQ_NOT | ci );

	/*
	 * Make sure that the doorbell record in host memory is
	 * written before ringing the doorbell via PCI MMIO.
	 */
	wmb();

	doorbell[0] = cpu_to_be32 ( sn << 28 | MLX4_CQ_DB_REQ_NOT | cq->cqn );
	doorbell[1] = cpu_to_be32 ( ci );

	writel ( doorbell[0], addr );
	writel ( doorbell[1], addr + 4 );
}

/**
 * Poll completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void hermon_poll_cq ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_completion_queue *hermon_cq = ib_cq_get_drvdata ( cq );
	union hermonprm_completion_entry *cqe;
	unsigned int cqe_idx_mask;
	int rc;

	while ( 1 ) {
		/* Look for completion entry */
		cqe_idx_mask = ( cq->num_cqes - 1 );
		cqe = &hermon_cq->cqe[cq->next_idx & cqe_idx_mask];
		if ( MLX_GET ( &cqe->normal, owner ) ^
		     ( ( cq->next_idx & cq->num_cqes ) ? 1 : 0 ) ) {
			/* Entry still owned by hardware; end of poll */
			break;
		}

		/* Handle completion */
		if ( ( rc = hermon_complete ( ibdev, cq, cqe ) ) != 0 ) {
			printf ( "ConnectX3 %p CQN %#lx failed to complete:"
			       " %s\n", hermon, cq->cqn, strerror ( rc ) );
			DBGC_HDA ( hermon, virt_to_phys ( cqe ),
				   cqe, sizeof ( *cqe ) );
		}

		/* Update completion queue's index */
		cq->next_idx++;
	}
	/* Update doorbell record */
	MLX_FILL_1 ( hermon_cq->doorbell, 0, update_ci,
			( cq->next_idx & 0x00ffffffUL ) );
	arm_cq ( hermon, cq );
}


/***************************************************************************
 *
 * Multicast group operations
 *
 ***************************************************************************
 */


/**
 * Attach to multicast group entry
 *
 * @v ibdev		Infiniband device
 * @v gid		Multicast GID
 * @v qp		MCG entry index
 * @v prev_index	MCG entry previous entry
 * @v mcg		MCG entry
 * @ret rc		Return status code
 */
static int hermon_find_entry ( struct ib_device *ibdev,
			       union ib_gid *gid,
			       int *index,
			       int *prev_index,
			       struct hermonprm_mcg_entry *mcg ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermonprm_mgm_hash hash;
	u8 *mcg_gid_p;
	int rc = 0;

	/* Generate hash table index */
	if ( ( rc = hermon_cmd_mgid_hash
			    ( hermon, gid, &hash, ibdev->protocol ) ) != 0 ) {
		printf ( "ConnectX3 %p could not hash GID: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	*index = MLX_GET ( &hash, hash );
	*prev_index = -1;

	/* Check for existing hash table entry */
	do {
		if ( ( rc = hermon_cmd_read_mcg	( hermon, 0, *index, mcg ) ) != 0 ) {
			printf ( "ConnectX3 %p could not read MCG %#x: "
			       "%s\n", hermon, *index, strerror ( rc ) );
			return rc;
		}

		if ( MLX_GET ( mcg, hdr.members_count ) == 0 ) {
			if ( *index != (int) MLX_GET ( &hash, hash ) ) {
				DBGC ( hermon, "ConnectX3 %p Found zero MGID"
					       " in AMGM\n", hermon );
				return -EBUSY;
			}
			/* Add new entry */
			return 0;
		}

		mcg_gid_p = (u8 *)mcg;
		mcg_gid_p += 16;
		if ( !memcmp ( mcg_gid_p, gid, sizeof ( *gid ) ) &&
		     ( MLX_GET ( mcg, hdr.protocol ) == ibdev->protocol ) ) {
			DBGC ( hermon, "ConnectX3 %p Found MCG entry with the"
			       " same GID\n", hermon );
			return 0;
		}

		*prev_index = *index;
		*index = MLX_GET ( mcg, hdr.next_mcg );
	} while ( *index );

	/* Entry was not found */
	*index = -1;

	return 0;
}

/**
 * Attach to multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 * @ret rc		Return status code
 */
static int hermon_mcast_attach ( struct ib_device *ibdev,
				 struct ib_queue_pair *qp,
				 union ib_gid *gid ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermonprm_mcg_entry mcg;
	int index = 0;
	int prev_index = 0;
	unsigned int members_count;
	u8 link = 0;
	unsigned int i;
	int rc;

	if ( ( rc = hermon_find_entry ( ibdev, gid, &index, &prev_index, &mcg ) ) != 0 ) {
		printf ( "ConnectX3 %p failed to search MCG entry %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	if ( index == -1 ) {
		link = 1;
		index = hermon->mcg_aux_index ++;
		DBGC ( hermon, "ConnectX3 %p add new entry with index 0x%x\n",
		       hermon, index );
		memset ( &mcg, 0, sizeof ( mcg ) );
		memcpy ( &mcg.u.dwords[4], gid, sizeof ( *gid ) );
	}

	members_count = MLX_GET ( &mcg, hdr.members_count );

	if ( members_count >= HERMON_MAX_MGM_MEMBERS ) {
		DBGC ( hermon, "ConnectX3 %p MGM is full\n",
		       hermon );
		return -ENOMEM;
	}

	for ( i = 0; i < members_count; ++i ) {
		if ( MLX_GET ( &mcg, qp[i] ) == qp->qpn ) {
			DBGC ( hermon, "ConnectX3 %p QP 0x%lx already member"
			       " in MGM\n", hermon, qp->qpn );
			return 0;
		}
	}

	/* Update hash table entry */
	MLX_FILL_2 ( &mcg, 1, hdr.members_count, ++members_count,
		     hdr.protocol, ibdev->protocol );
	MLX_FILL_1 ( &mcg, ( 8 + (int)members_count - 1 ),
		     qp[members_count - 1].qpn, qp->qpn );
	memcpy ( &mcg.u.dwords[4], gid, sizeof ( *gid ) );
	if ( ( rc = hermon_cmd_write_mcg ( hermon, 0, index, &mcg ) ) != 0 ) {
		printf ( "ConnectX3 %p could not write MCG %#x: %s\n",
		       hermon, index, strerror ( rc ) );
		return rc;
	}

	if ( !link )
		return 0;

	/* Repair link list */
	if ( ( rc = hermon_cmd_read_mcg ( hermon, 0, prev_index, &mcg ) ) != 0 ) {
		printf ( "ConnectX3 %p could not read MCG %#x: %s\n",
		       hermon, prev_index, strerror ( rc ) );
		return rc;
	}

	MLX_FILL_1 ( &mcg, 0, hdr.next_mcg, index );

	if ( ( rc = hermon_cmd_write_mcg ( hermon, 0, prev_index, &mcg ) ) != 0 ) {
		printf ( "ConnectX3 %p could not write MCG %#x: %s\n",
		       hermon, prev_index, strerror ( rc ) );
		return rc;
	}

	return 0;
}

static int hermon_default_mcast_attach ( struct ib_device *ibdev,
				 struct ib_queue_pair *qp,
				 uint32_t in_mod ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermonprm_mcg_entry mcg;
	int rc;

	memset ( &mcg, 0, sizeof ( mcg ) );
	MLX_FILL_1 ( &mcg, 1, hdr.members_count, 1 );
	MLX_FILL_1 ( &mcg, 8, qp[0].qpn, qp->qpn );
	if ( ( rc = hermon_cmd_write_mcg ( hermon, 1, in_mod, &mcg ) ) != 0 ) {
		printf ( "ConnectX3 %p could not write default MCG: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Detach from multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 * @ret rc		Return status code
 */
static void hermon_mcast_detach ( struct ib_device *ibdev,
				  struct ib_queue_pair *qp,
				  union ib_gid *gid ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermonprm_mcg_entry mcg;
	int index = 0;
	int prev_index = 0;
	int next_index = 0;
	unsigned int members_count;
	int i;
	int rc;
	int qp_index;

	if ( ( rc = hermon_find_entry ( ibdev, gid, &index, &prev_index, &mcg ) ) != 0 ) {
		printf ( "ConnectX3 %p failed to search MCG entry %s\n",
		       hermon, strerror ( rc ) );
		return;
	}

	if (index == -1) {
		printf ( "ConnectX3 %p GID not found in MCG table\n",
		       hermon );
		return;
	}

	members_count = MLX_GET ( &mcg, hdr.members_count );
	for ( qp_index = -1, i = 0; i < (int)members_count; ++i )
		if ( MLX_GET ( &mcg, qp[i] ) == qp->qpn )
			qp_index = i;

	if ( qp_index == -1 ) {
		printf ( "ConnectX3 %p QP not found in MCG entry\n",
		       hermon );
		return;
	}

	MLX_FILL_2 ( &mcg, 1, hdr.members_count, --members_count,
		     hdr.protocol, ibdev->protocol );
	MLX_FILL_1 ( &mcg, ( 8 + qp_index ), qp[qp_index].qpn,
		     MLX_GET ( &mcg, qp[i -1].qpn ) );
	MLX_FILL_1 ( &mcg, ( 8 + i - 1 ), qp[i - 1].qpn, 0 );
	if ( members_count ) {
		if ( ( rc = hermon_cmd_write_mcg ( hermon, 0, index, &mcg ) ) != 0 ) {
			printf ( "ConnectX3 %p could not write MCG %#x: %s\n",
			       hermon, index, strerror ( rc ) );
			return;
		}
		return;
	}

	if ( prev_index == -1 ) {
		next_index = MLX_GET ( &mcg, hdr.next_mcg );
		if ( next_index ) {
			if ( ( rc = hermon_cmd_read_mcg ( hermon, 0, next_index, &mcg ) ) != 0 ) {
				printf ( "ConnectX3 %p could not read MCG "
				       "%#x: %s\n", hermon, next_index,
				       strerror ( rc ) );
				return;
			}
		} else
			memset ( &mcg.u.dwords[4], 0, sizeof ( union ib_gid ) );

		if ( ( rc = hermon_cmd_write_mcg ( hermon, 0, index, &mcg ) ) != 0 ) {
			printf ( "ConnectX3 %p could not write MCG %#x:"
			       " %s\n", hermon, index, strerror ( rc ) );
			return;
		}
	} else {
		/* Remove entry from AMGM */
		next_index = MLX_GET ( &mcg, hdr.next_mcg );

		if ( ( rc = hermon_cmd_read_mcg ( hermon, 0, prev_index, &mcg ) ) != 0 ) {
			printf ( "ConnectX3 %p could not read MCG %#x:"
			       " %s\n", hermon, prev_index, strerror ( rc ) );
			return;
		}

		MLX_FILL_1 ( &mcg, 0, hdr.next_mcg, next_index );
		if ( ( rc = hermon_cmd_write_mcg ( hermon, 0, index, &mcg ) ) != 0 ) {
			printf ( "ConnectX3 %p could not write MCG %#x: %s\n",
			       hermon, index, strerror ( rc ) );
			return;
		}
	}

	return;
}

static void hermon_default_mcast_detach ( struct ib_device *ibdev,
				  struct ib_queue_pair *qp __unused,
				  uint32_t in_mod ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermonprm_mcg_entry mcg;
	int rc;

	memset ( &mcg, 0, sizeof ( mcg ) );
	if ( ( rc = hermon_cmd_write_mcg ( hermon, 1, in_mod, &mcg ) ) != 0 )
		printf ( "ConnectX3 %p could not write default MCG: %s\n",
		       hermon, strerror ( rc ) );
}

/***************************************************************************
 *
 * Event queues
 *
 ***************************************************************************
 */

/**
 * Create event queue
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
static int hermon_create_eq ( struct hermon *hermon ) {
	struct hermon_event_queue *hermon_eq = &hermon->eq;
	struct hermonprm_eqc eqctx;
	struct hermonprm_event_mask mask;
	unsigned int i;
	int rc;

	/* Select event queue number */
	hermon_eq->eqn = ( 4 * hermon->cap.reserved_uars );
	if ( hermon_eq->eqn < hermon->cap.reserved_eqs )
		hermon_eq->eqn = hermon->cap.reserved_eqs;

	/* Calculate doorbell address */
	hermon_eq->doorbell =
		( hermon->uar + HERMON_DB_EQ_OFFSET ( hermon_eq->eqn ) );

	/* Allocate event queue itself */
	hermon_eq->eqe_size =
		( HERMON_NUM_EQES * sizeof ( hermon_eq->eqe[0] ) );
	hermon_eq->eqe = malloc_dma ( hermon_eq->eqe_size,
				      sizeof ( hermon_eq->eqe[0] ) );
	if ( ! hermon_eq->eqe ) {
		rc = -ENOMEM;
		printf ( "ConnectX3 %p could not allocate EQEs %s\n",
			hermon, strerror ( rc ) );
		goto err_eqe;
	}
	memset ( hermon_eq->eqe, 0, hermon_eq->eqe_size );
	for ( i = 0 ; i < HERMON_NUM_EQES ; i++ ) {
		MLX_FILL_1 ( &hermon_eq->eqe[i].generic, 7, owner, 1 );
	}
	barrier();

	/* Allocate MTT entries */
	if ( ( rc = hermon_alloc_mtt ( hermon, hermon_eq->eqe,
				       hermon_eq->eqe_size,
				       &hermon_eq->mtt ) ) != 0 ) {
		printf ( "ConnectX3 %p could not allocate MTTs for EQ%s\n",
			hermon, strerror ( rc ) );
		goto err_alloc_mtt;
	}

	/* Hand queue over to hardware */
	memset ( &eqctx, 0, sizeof ( eqctx ) );
	MLX_FILL_2 ( &eqctx, 0,
		     st, 0xa /* "Fired" */,
		     oi, 1 );
	MLX_FILL_1 ( &eqctx, 2,
		     page_offset, ( hermon_eq->mtt.page_offset >> 5 ) );
	MLX_FILL_1 ( &eqctx, 3, log_eq_size, fls ( HERMON_NUM_EQES - 1 ) );
	MLX_FILL_H ( &eqctx, 6, mtt_base_addr_h,
		     hermon_eq->mtt.mtt_base_addr );
	MLX_FILL_1 ( &eqctx, 7, mtt_base_addr_l,
		     ( hermon_eq->mtt.mtt_base_addr >> 3 ) );
	if ( ( rc = hermon_cmd_sw2hw_eq ( hermon, hermon_eq->eqn,
					  &eqctx ) ) != 0 ) {
		printf ( "ConnectX3 %p EQN %#lx SW2HW_EQ failed: %s\n",
		       hermon, hermon_eq->eqn, strerror ( rc ) );
		goto err_sw2hw_eq;
	}

	/* Map all events to this event queue */
	memset ( &mask, 0xff, sizeof ( mask ) );
	if ( ( rc = hermon_cmd_map_eq ( hermon,
					( HERMON_MAP_EQ | hermon_eq->eqn ),
					&mask ) ) != 0 ) {
		printf ( "ConnectX3 %p EQN %#lx MAP_EQ failed: %s\n",
		       hermon, hermon_eq->eqn, strerror ( rc )  );
		goto err_map_eq;
	}

	DBGC ( hermon, "ConnectX3 %p EQN %#lx ring [%08lx,%08lx), doorbell "
	       "%08lx\n", hermon, hermon_eq->eqn,
	       virt_to_phys ( hermon_eq->eqe ),
	       ( virt_to_phys ( hermon_eq->eqe ) + hermon_eq->eqe_size ),
	       virt_to_phys ( hermon_eq->doorbell ) );
	return 0;

 err_map_eq:
	hermon_cmd_hw2sw_eq ( hermon, hermon_eq->eqn, &eqctx );
 err_sw2hw_eq:
	hermon_free_mtt ( hermon, &hermon_eq->mtt );
 err_alloc_mtt:
	free_dma ( hermon_eq->eqe, hermon_eq->eqe_size );
 err_eqe:
	memset ( hermon_eq, 0, sizeof ( *hermon_eq ) );
	return rc;
}

/**
 * Destroy event queue
 *
 * @v hermon		ConnectX3 device
 */
static void hermon_destroy_eq ( struct hermon *hermon ) {
	struct hermon_event_queue *hermon_eq = &hermon->eq;
	struct hermonprm_eqc eqctx;
	struct hermonprm_event_mask mask;
	int rc;

	/* Unmap events from event queue */
	memset ( &mask, 0xff, sizeof ( mask ) );
	if ( ( rc = hermon_cmd_map_eq ( hermon,
					( HERMON_UNMAP_EQ | hermon_eq->eqn ),
					&mask ) ) != 0 ) {
		printf ( "ConnectX3 %p EQN %#lx FATAL MAP_EQ failed to "
		       "unmap: %s\n", hermon, hermon_eq->eqn, strerror ( rc ) );
		/* Continue; HCA may die but system should survive */
	}

	/* Take ownership back from hardware */
	if ( ( rc = hermon_cmd_hw2sw_eq ( hermon, hermon_eq->eqn,
					  &eqctx ) ) != 0 ) {
		printf ( "ConnectX3 %p EQN %#lx FATAL HW2SW_EQ failed: %s\n",
		       hermon, hermon_eq->eqn, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}

	/* Free MTT entries */
	hermon_free_mtt ( hermon, &hermon_eq->mtt );

	/* Free memory */
	free_dma ( hermon_eq->eqe, hermon_eq->eqe_size );
	memset ( hermon_eq, 0, sizeof ( *hermon_eq ) );
}

static void hermon_event_completion ( struct hermon *hermon,
					union hermonprm_event_entry *eqe ) {
	struct ib_completion_queue *cq = NULL;
	struct hermon_completion_queue *hcq = NULL;
	struct hermon_port *port;
	unsigned int cqn, i;

	cqn = MLX_GET ( &eqe->completion, data.cqn );
	for ( i = 0 ; i < hermon->num_ports ; i++ ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		port = &hermon->port[i];
		cq = port->eth_cq;
		if ( ( port->ibdev->protocol == HERMON_PROT_ETH ) &&
		     ( cq != NULL ) && ( cq->cqn == cqn ) ) {
			if ( ( hcq = ib_cq_get_drvdata ( cq ) ) ) {
				++hcq->arm_sn;
				break;
			}
		}
	}
}

/**
 * Handle port state event
 *
 * @v hermon		ConnectX3 device
 * @v eqe		Port state change event queue entry
 */
static void hermon_event_port_state_change ( struct hermon *hermon,
					     union hermonprm_event_entry *eqe){
	unsigned int port;
	int link_up;

	/* Get port and link status */
	port = ( MLX_GET ( &eqe->port_state_change, data.p ) - 1 );
	link_up = ( MLX_GET ( &eqe->generic, event_sub_type ) & 0x04 );
	DBGC ( hermon, "ConnectX3 %p port %d link %s\n", hermon, ( port + 1 ),
	       ( link_up ? "up" : "down" ) );

	/* Sanity check */
	if ( port >= hermon->num_ports ) {
		printf ( "ConnectX3 %p port %d does not exist!\n",
		       hermon, ( port + 1 ) );
		return;
	}

	/* If the port is not disabled, notify device of port state change */
	if ( hermon->port_mask & ( port + 1 ) )
		hermon->port[port].type->state_change ( hermon,
						&hermon->port[port], link_up );
}

static void hermon_event_port_mgmt_change ( struct hermon *hermon,
					union hermonprm_event_entry *eqe ) {
	uint8_t port = ( MLX_GET ( &eqe->port_mgmt_change, port ) - 1 );

	/* Sanity check */
	if ( port >= hermon->num_ports ) {
		printf ( "ConnectX3 %p port %d does not exist!\n",
		       hermon, ( port + 1 ) );
		return;
	}

	/* If the port is not disabled, notify device of port management change */
	if ( hermon->port_mask & ( port + HERMON_PORT_BASE ) )
		ib_smc_update ( hermon->port[port].ibdev, hermon_mad );
}

/**
 * Operation required action
 *
 * @v ibdev		Infiniband device
 * @ret rc		Return status code
 */
static int hermon_opreq_action ( struct hermon *hermon )
{
	struct ib_device *ibdev;
	unsigned int port;
	struct hermonprm_mcg_entry mcg;
	struct hermonprm_eth_mgi mgi;
	struct hermonprm_get_op_req get_op_req;
	struct ib_multicast_gid *mgid;
	struct ib_queue_pair qp;
	int rc;

	if ( ( rc = hermon_cmd_get_op_req ( hermon, 0, 0,
					    &get_op_req ) ) != 0 ) {
		printf ( "ConnectX3 %p could not get op request %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	memcpy ( &mcg, &get_op_req.u.dwords[8],
		 sizeof ( struct hermonprm_mcg_entry ) );
	qp.qpn = MLX_GET ( &mcg, qp[0].qpn );
	memcpy ( &mgi, &get_op_req.u.dwords[12],
		 sizeof ( struct hermonprm_eth_mgi ) );
	port = MLX_GET ( &mgi, portnum );

	ibdev = hermon->port[port - 1].ibdev;
	if ( ibdev->protocol == HERMON_PROT_ETH ) {
		MLX_SET ( &mgi, vep_num, hermon->port[port - 1].vep_number );
	}

	switch ( MLX_GET ( &get_op_req , request_type ) ) {
	case HERMON_OPREQ_ADD_TO_MCG:
		if ( MLX_GET ( &get_op_req, mcg.hdr.member_remove ) ) {
			hermon_mcast_detach
				( ibdev, &qp, ( union ib_gid * ) &mgi );
		} else {
			if ( ( rc = hermon_mcast_attach
			     ( ibdev, &qp, ( union ib_gid * ) &mgi ) ) != 0 ) {
				printf ( "ConnectX3 %p QPN %lx could not "
				       " attach: %s\n", hermon, qp.qpn,
				       strerror ( rc ) );
				return rc;
			}

			/* Add to software multicast GID list */
			mgid = zalloc ( sizeof ( *mgid ) );
			if ( ! mgid ) {
				return -ENOMEM;
			}
			memcpy ( &mgid->gid, &mgi, sizeof ( mgid->gid ) );
			mgid->protocol = ibdev->protocol;
			mgid->qpn = qp.qpn;
			list_add ( &mgid->list, &hermon->ncsi_mgids );
		}
		break;
	default:
		printf ( "ConnectX3 %p Unrecognized NCSI event\n", hermon );
		break;
	}

	if ( ( rc = hermon_cmd_get_op_req
				       ( hermon, 1, 0, &get_op_req ) ) != 0 ) {
		printf ( "ConnectX3 %p could not get op request %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	return	0;
}

#ifdef HERMON_IRQ_ENABLED
static void hermon_arm_eq ( struct hermon *hermon ) {
	struct hermon_event_queue *heq = &hermon->eq;

	writel ( cpu_to_be32 ( 0x80000000 | ( heq->next_idx & 0xffffff ) ), heq->doorbell );
	wmb();
}

static void hermon_clr_int ( struct hermon *hermon ) {
	u64  intapin_val;
	u32 *addr = ( u32 * )( hermon->clr_int + hermon->clr_int_bar_offset );
	u32 vh, vl;

	/* clear the interrupt */
	intapin_val = ( u64 )1 << hermon->intapin;
	vh = intapin_val >> 32;
	vl = intapin_val & 0xffffffff;

	writel ( cpu_to_be32 ( vh ), addr );
	writel ( cpu_to_be32 ( vl ), &addr[1] );
	barrier();
}
#endif

/**
 * Poll event queue
 *
 * @v ibdev		Infiniband device
 */
static void hermon_poll_eq ( struct ib_device *ibdev ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_event_queue *hermon_eq = &hermon->eq;
	struct net_device *netdev = ib_get_ownerdata ( ibdev );
	struct hermonprm_query_port_cap query_port;
	union hermonprm_event_entry *eqe;
	union hermonprm_doorbell_register db_reg;
	unsigned int eqe_idx_mask;
	unsigned int event_type;
	int rc = 0;

#ifdef HERMON_IRQ_ENABLED
	hermon_clr_int ( hermon );
#endif

	/* No event is generated upon reaching INIT, so we must poll
	 * separately for link state changes while we remain DOWN.
	 */
	if ( ibdev->protocol != HERMON_PROT_ETH && ib_is_open ( ibdev ) &&
	     ( ibdev->port_state == IB_PORT_STATE_DOWN ) ) {
		ib_smc_update ( ibdev, hermon_mad );
	}

	/* Poll event queue */
	while ( 1 ) {
		/* Look for event entry */
		eqe_idx_mask = ( HERMON_NUM_EQES - 1 );
		eqe = &hermon_eq->eqe[hermon_eq->next_idx & eqe_idx_mask];
		if ( MLX_GET ( &eqe->generic, owner ) ^
		     ( ( hermon_eq->next_idx & HERMON_NUM_EQES ) ? 1 : 0 ) ) {
			/* Entry still owned by hardware; end of poll */
			break;
		}
		DBGCP ( hermon, "ConnectX3 %p EQN %#lx event:\n",
			hermon, hermon_eq->eqn );
		DBGCP_HDA ( hermon, virt_to_phys ( eqe ),
			    eqe, sizeof ( *eqe ) );

		/* Handle event */
		event_type = MLX_GET ( &eqe->generic, event_type );
		switch ( event_type ) {
		case HERMON_EV_CQ_COMPLETION:
			hermon_event_completion ( hermon, eqe );
			break;
		case HERMON_EV_CMD_COMPLETION:
			/* currently only NOP uses event and it does not need further processing.
			 * If the command needs further processing it needs to be done here
			 */
			break;
		case HERMON_EV_PORT_STATE_CHANGE:
			hermon_event_port_state_change ( hermon, eqe );
			break;
		case HERMON_EV_PORT_MGMNT_CHANGE:
			hermon_event_port_mgmt_change ( hermon, eqe );
			break;
		case HERMON_EV_TYPE_OP_REQUIRED:
			hermon_opreq_action ( hermon );
			break;
		default:
			DBGC ( hermon, "ConnectX3 %p EQN %#lx unrecognised event "
			       "type %#x:\n",
			       hermon, hermon_eq->eqn, event_type );
			DBGC_HDA ( hermon, virt_to_phys ( eqe ),
				   eqe, sizeof ( *eqe ) );
			break;
		}

		/* Update event queue's index */
		hermon_eq->next_idx++;

		/* Ring doorbell */
		if ( hermon->cap.cmdif_post_doorbell ) {
			if ( (rc = hermon_cmd_post_eq_doorbell ( hermon, hermon_eq->eqn,
					hermon_eq->next_idx ) ) != 0 ) {
				printf ( "Failed to post doorbell command on ConnectX3 %p EQN %#lx ",
					hermon, hermon_eq->eqn );
				return;
			}
		} else {
			MLX_FILL_1 ( &db_reg.event, 0,
				     ci, ( hermon_eq->next_idx & 0x00ffffffUL ) );
			writel ( db_reg.dword[0], hermon_eq->doorbell );
		}
	}

	if ( ibdev->protocol == HERMON_PROT_ETH && netdev->link_rc ) {
		if ( ( rc = hermon_cmd_query_port ( hermon,
					    ibdev->port,
					    &query_port ) ) != 0 ) {
			printf ( "ConnectX3 %p port %d could not "
			       "query port: %s\n", hermon, ibdev->port,
			       strerror ( rc ) );
		} else {
			if ( MLX_GET ( &query_port, link_state ) )
				netdev->link_rc = 0;
		}
	}
}

/***************************************************************************
 *
 * Firmware control
 *
 ***************************************************************************
 */

/**
 * Map virtual to physical address for firmware usage
 *
 * @v hermon		ConnectX3 device
 * @v map		Mapping function
 * @v va		Virtual address
 * @v pa		Physical address
 * @v len		Length of region
 * @ret rc		Return status code
 */
static int hermon_map_vpm ( struct hermon *hermon,
			    int ( *map ) ( struct hermon *hermon,
			    const struct hermonprm_virtual_physical_mapping* ),
			    uint64_t va, physaddr_t pa, size_t len ) {
	struct hermonprm_virtual_physical_mapping mapping;
	physaddr_t start;
	physaddr_t low;
	physaddr_t high;
	physaddr_t end;
	size_t size;
	int rc;

	/* Sanity checks */
	assert ( ( va & ( HERMON_PAGE_SIZE - 1 ) ) == 0 );
	assert ( ( pa & ( HERMON_PAGE_SIZE - 1 ) ) == 0 );
	assert ( ( len & ( HERMON_PAGE_SIZE - 1 ) ) == 0 );

	/* Calculate starting points */
	start = pa;
	end = ( start + len );
	size = ( 1UL << ( fls ( start ^ end ) - 1 ) );
	low = high = ( end & ~( size - 1 ) );
	assert ( start < low );
	assert ( high <= end );

	/* These mappings tend to generate huge volumes of
	 * uninteresting debug data, which basically makes it
	 * impossible to use debugging otherwise.
	 */
	DBG_DISABLE ( DBGLVL_LOG | DBGLVL_EXTRA );

	/* Map blocks in descending order of size */
	while ( size >= HERMON_PAGE_SIZE ) {

		/* Find the next candidate block */
		if ( ( low - size ) >= start ) {
			low -= size;
			pa = low;
		} else if ( ( high + size ) <= end ) {
			pa = high;
			high += size;
		} else {
			size >>= 1;
			continue;
		}
		assert ( ( va & ( size - 1 ) ) == 0 );
		assert ( ( pa & ( size - 1 ) ) == 0 );

		/* Map this block */
		memset ( &mapping, 0, sizeof ( mapping ) );
		MLX_FILL_1 ( &mapping, 0, va_h, ( va >> 32 ) );
		MLX_FILL_1 ( &mapping, 1, va_l, ( va >> 12 ) );
		MLX_FILL_H ( &mapping, 2, pa_h, pa );
		MLX_FILL_2 ( &mapping, 3,
			     log2size, ( ( fls ( size ) - 1 ) - 12 ),
			     pa_l, ( pa >> 12 ) );
		if ( ( rc = map ( hermon, &mapping ) ) != 0 ) {
			DBG_ENABLE ( DBGLVL_LOG | DBGLVL_EXTRA );
			printf ( "ConnectX3 %p could not map %08llx+%zx to "
			       "%08lx: %s\n",
			       hermon, va, size, pa, strerror ( rc ) );
			return rc;
		}
		va += size;
	}
	assert ( low == start );
	assert ( high == end );

	DBG_ENABLE ( DBGLVL_LOG | DBGLVL_EXTRA );
	return 0;
}

/**
 * Start firmware running
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
static int hermon_start_firmware ( struct hermon *hermon ) {
	struct hermonprm_query_fw fw;
	unsigned int fw_pages;
	size_t fw_len;
	physaddr_t fw_base;
	int rc;

	/* Get firmware parameters */
	if ( ( rc = hermon_cmd_query_fw ( hermon, &fw ) ) != 0 ) {
		printf ( "ConnectX3 %p could not query firmware: %s\n",
		       hermon, strerror ( rc ) );
		goto err_query_fw;
	}
	DBGC ( hermon, "ConnectX3 %p firmware version %d.%d.%d\n", hermon,
	       MLX_GET ( &fw, fw_rev_major ), MLX_GET ( &fw, fw_rev_minor ),
	       MLX_GET ( &fw, fw_rev_subminor ) );
	fw_pages = MLX_GET ( &fw, fw_pages );
	DBGC ( hermon, "ConnectX3 %p requires %d pages (%d kB) for firmware\n",
	       hermon, fw_pages, ( fw_pages * 4 ) );

	hermon->clr_int_bar = MLX_GET ( &fw, clr_int_bar );
	hermon->clr_int_bar_offset_l = MLX_GET ( &fw, clr_int_base_offset_l );
	hermon->clr_int_bar_offset_h = MLX_GET ( &fw, clr_int_base_offset_h );
	hermon->clr_int_bar_offset = ( ( hermon->clr_int_bar_offset_l) | ( ( (u64) ( hermon->clr_int_bar_offset_h ) ) << 32 ) );
	if ( hermon->clr_int_bar == 0 ) {
		/* BAR 0 */
		hermon->clr_int = hermon->config;
	} else if ( hermon->clr_int_bar == 2 ) {
		/* BAR 2 */
		hermon->clr_int = hermon->uar;
	} else {
		/* Invalid value */
		printf ( "Unexpected bar value: %d 0x%llx\n", hermon->clr_int_bar, hermon->clr_int_bar_offset );
		goto err_query_fw;
	}

	/* Allocate firmware pages and map firmware area */
	fw_len = ( fw_pages * HERMON_PAGE_SIZE );
	if ( ! hermon->firmware_area ) {
		hermon->firmware_len = fw_len;
		hermon->firmware_area = umalloc ( hermon->firmware_len );
		if ( ! hermon->firmware_area ) {
			rc = -ENOMEM;
			printf ( "ConnectX3 %p could not allocate firmware area %s\n",
				hermon, strerror ( rc ) );
			goto err_alloc_fa;
		}
	} else {
		assert ( hermon->firmware_len == fw_len );
	}
	fw_base = user_to_phys ( hermon->firmware_area, 0 );
	DBGC ( hermon, "ConnectX3 %p firmware area at physical [%08lx,%08lx)\n",
	       hermon, fw_base, ( fw_base + fw_len ) );
	if ( ( rc = hermon_map_vpm ( hermon, hermon_cmd_map_fa,
				     0, fw_base, fw_len ) ) != 0 ) {
		printf ( "ConnectX3 %p could not map firmware: %s\n",
		       hermon, strerror ( rc ) );
		goto err_map_fa;
	}

	/* Start firmware */
	if ( ( rc = hermon_cmd_run_fw ( hermon ) ) != 0 ) {
		printf ( "ConnectX3 %p could not run firmware: %s\n",
		       hermon, strerror ( rc ) );
		goto err_run_fw;
	}

	DBGC ( hermon, "ConnectX3 %p firmware started\n", hermon );
	return 0;

 err_run_fw:
 err_map_fa:
	hermon_cmd_unmap_fa ( hermon );
 err_alloc_fa:
 err_query_fw:
	return rc;
}

/**
 * Stop firmware running
 *
 * @v hermon		ConnectX3 device
 */
static void hermon_stop_firmware ( struct hermon *hermon ) {
	int rc;

	if ( ( rc = hermon_cmd_unmap_fa ( hermon ) ) != 0 ) {
		printf ( "ConnectX3 %p FATAL could not stop firmware: %s\n",
		       hermon, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		hermon->firmware_area = UNULL;
		return;
	}
	sleep(2);
}

/***************************************************************************
 *
 * Infinihost Context Memory management
 *
 ***************************************************************************
 */

/**
 * Get device limits
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
static int hermon_get_cap ( struct hermon *hermon ) {
	struct hermonprm_query_dev_cap dev_cap;
	uint32_t dword;
	int rc;

	if ( ( rc = hermon_cmd_query_dev_cap ( hermon, &dev_cap ) ) != 0 ) {
		DBGC ( hermon, "ConnectX3 %p could not get device limits: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	hermon->cap.cmpt_entry_size = MLX_GET ( &dev_cap, c_mpt_entry_sz );
	hermon->cap.reserved_qps =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_qps ) );
	hermon->cap.qpc_entry_size = MLX_GET ( &dev_cap, qpc_entry_sz );
	hermon->cap.altc_entry_size = MLX_GET ( &dev_cap, altc_entry_sz );
	hermon->cap.auxc_entry_size = MLX_GET ( &dev_cap, aux_entry_sz );
	hermon->cap.reserved_srqs =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_srqs ) );
	hermon->cap.srqc_entry_size = MLX_GET ( &dev_cap, srq_entry_sz );
	hermon->cap.reserved_cqs =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_cqs ) );
	hermon->cap.cqc_entry_size = MLX_GET ( &dev_cap, cqc_entry_sz );
	hermon->cap.reserved_eqs = MLX_GET ( &dev_cap, num_rsvd_eqs );
	if ( hermon->cap.reserved_eqs == 0 ) {
		/* Backward compatibility */
		hermon->cap.reserved_eqs =
			( 1 << MLX_GET ( &dev_cap, log2_rsvd_eqs ) );
	}
	hermon->cap.eqc_entry_size = MLX_GET ( &dev_cap, eqc_entry_sz );
	hermon->cap.reserved_mtts =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_mtts ) );
	hermon->cap.mtt_entry_size = MLX_GET ( &dev_cap, mtt_entry_sz );
	hermon->cap.reserved_mrws =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_mrws ) );
	hermon->cap.dmpt_entry_size = MLX_GET ( &dev_cap, d_mpt_entry_sz );
	hermon->cap.reserved_uars = MLX_GET ( &dev_cap, num_rsvd_uars );
	hermon->cap.dpdp = MLX_GET ( &dev_cap, dpdp );
	hermon->cap.vep_uc_steering		= MLX_GET ( &dev_cap, vep_uc_steering );
	hermon->cap.vep_mc_steering		= MLX_GET ( &dev_cap, vep_mc_steering );
	hermon->cap.nv_mem_access_supported	= MLX_GET( &dev_cap, flash_nv_config);
	hermon->cap.ncsi_lag_mode 		= MLX_GET( &dev_cap, ncsi_lag_mode);
	hermon->cap.cmdif_post_doorbell		= MLX_GET ( &dev_cap, cmdif_post_doorbell );
	if ( hermon->cap.cmdif_post_doorbell ) {
		DBGC ( hermon, "Using the command interface to post doorbells!\n");
	}
	hermon->cap.num_ports = MLX_GET ( &dev_cap, num_ports );
	hermon->cap.port_beacon = MLX_GET ( &dev_cap, port_beacon );
	hermon->cap.max_funix = MLX_GET ( &dev_cap, max_funix );

	hermon->cap.nv_config_flags.nv_config_wol_port1 = MLX_GET ( &dev_cap, wol_port1 );
	hermon->cap.nv_config_flags.nv_config_wol_port2 = MLX_GET ( &dev_cap, wol_port2 );
	dword = MLX_GET ( &dev_cap, nv_config_sriov_en );
	hermon->cap.nv_config_flags.nv_config_sriov_en = dword & (1 << 0);
	hermon->cap.nv_config_flags.nv_config_vpi_port1 = dword & (1 << 3);
	hermon->cap.nv_config_flags.nv_config_vpi_port2 = dword & (1 << 4);
	hermon->cap.nv_config_flags.nv_config_bar_size = dword & (1 << 5);

	return 0;
}

/**
 * Align ICM table
 *
 * @v icm_offset	Current ICM offset
 * @v len		ICM table length
 * @ret icm_offset	ICM offset
 */
static uint64_t icm_align ( uint64_t icm_offset, size_t len ) {

	/* Round up to a multiple of the table size */
	assert ( len == ( 1UL << ( fls ( len ) - 1 ) ) );
	return ( ( icm_offset + len - 1 ) & ~( ( ( uint64_t ) len ) - 1 ) );
}

/**
 * Map ICM (allocating if necessary)
 *
 * @v hermon		ConnectX3 device
 * @v init_hca		INIT_HCA structure to fill in
 * @ret rc		Return status code
 */
static int hermon_map_icm ( struct hermon *hermon,
			    struct hermonprm_init_hca *init_hca ) {
	struct hermonprm_scalar_parameter icm_size;
	struct hermonprm_scalar_parameter icm_aux_size;
	uint64_t icm_offset = 0;
	unsigned int log_num_qps, log_num_srqs, log_num_cqs, log_num_eqs;
	unsigned int log_num_mtts, log_num_mpts, log_num_mcs;
	size_t cmpt_max_len;
	size_t icm_len, icm_aux_len;
	size_t len;
	physaddr_t icm_phys;
	int i;
	int rc;

	/*
	 * Start by carving up the ICM virtual address space
	 *
	 */

	/* Calculate number of each object type within ICM */
	log_num_qps = fls ( hermon->cap.reserved_qps +
			    HERMON_RSVD_SPECIAL_QPS + HERMON_MAX_QPS - 1 );
	log_num_srqs = fls ( hermon->cap.reserved_srqs - 1 );
	log_num_cqs = fls ( hermon->cap.reserved_cqs + HERMON_MAX_CQS - 1 );
	log_num_eqs = fls ( hermon->cap.reserved_eqs + HERMON_MAX_EQS - 1 );
	log_num_mtts = fls ( hermon->cap.reserved_mtts + HERMON_MAX_MTTS - 1 );
	log_num_mpts = fls ( hermon->cap.reserved_mrws + 1 - 1 );
	log_num_mcs = HERMON_LOG_MULTICAST_HASH_SIZE;

	/* ICM starts with the cMPT tables, which are sparse */
	cmpt_max_len = ( HERMON_CMPT_MAX_ENTRIES *
			 ( ( uint64_t ) hermon->cap.cmpt_entry_size ) );
	len = ( ( ( ( 1 << log_num_qps ) * hermon->cap.cmpt_entry_size ) +
		  HERMON_PAGE_SIZE - 1 ) & ~( HERMON_PAGE_SIZE - 1 ) );
	hermon->icm_map[HERMON_ICM_QP_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_QP_CMPT].len = len;
	icm_offset += cmpt_max_len;
	len = ( ( ( ( 1 << log_num_srqs ) * hermon->cap.cmpt_entry_size ) +
		  HERMON_PAGE_SIZE - 1 ) & ~( HERMON_PAGE_SIZE - 1 ) );
	hermon->icm_map[HERMON_ICM_SRQ_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_SRQ_CMPT].len = len;
	icm_offset += cmpt_max_len;
	len = ( ( ( ( 1 << log_num_cqs ) * hermon->cap.cmpt_entry_size ) +
		  HERMON_PAGE_SIZE - 1 ) & ~( HERMON_PAGE_SIZE - 1 ) );
	hermon->icm_map[HERMON_ICM_CQ_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_CQ_CMPT].len = len;
	icm_offset += cmpt_max_len;
	len = ( ( ( ( 1 << log_num_eqs ) * hermon->cap.cmpt_entry_size ) +
		  HERMON_PAGE_SIZE - 1 ) & ~( HERMON_PAGE_SIZE - 1 ) );
	hermon->icm_map[HERMON_ICM_EQ_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_EQ_CMPT].len = len;
	icm_offset += cmpt_max_len;

	hermon->icm_map[HERMON_ICM_OTHER].offset = icm_offset;

	/* Queue pair contexts */
	len = ( ( 1 << log_num_qps ) * hermon->cap.qpc_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 12,
		     qpc_eec_cqc_eqc_rdb_parameters.qpc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 13,
		     qpc_eec_cqc_eqc_rdb_parameters.qpc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_qp,
		     log_num_qps );
	DBGC ( hermon, "ConnectX3 %p ICM QPC is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_qps ), hermon->cap.qpc_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Extended alternate path contexts */
	len = ( ( 1 << log_num_qps ) * hermon->cap.altc_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 24,
		     qpc_eec_cqc_eqc_rdb_parameters.altc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 25,
		     qpc_eec_cqc_eqc_rdb_parameters.altc_base_addr_l,
		     icm_offset );
	DBGC ( hermon, "ConnectX3 %p ICM ALTC is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_qps ), hermon->cap.altc_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Extended auxiliary contexts */
	len = ( ( 1 << log_num_qps ) * hermon->cap.auxc_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 28,
		     qpc_eec_cqc_eqc_rdb_parameters.auxc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 29,
		     qpc_eec_cqc_eqc_rdb_parameters.auxc_base_addr_l,
		     icm_offset );
	DBGC ( hermon, "ConnectX3 %p ICM AUXC is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_qps ), hermon->cap.auxc_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Shared receive queue contexts */
	len = ( ( 1 << log_num_srqs ) * hermon->cap.srqc_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 18,
		     qpc_eec_cqc_eqc_rdb_parameters.srqc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 19,
		     qpc_eec_cqc_eqc_rdb_parameters.srqc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_srq,
		     log_num_srqs );
	DBGC ( hermon, "ConnectX3 %p ICM SRQC is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_srqs ), hermon->cap.srqc_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Completion queue contexts */
	len = ( ( 1 << log_num_cqs ) * hermon->cap.cqc_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 20,
		     qpc_eec_cqc_eqc_rdb_parameters.cqc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 21,
		     qpc_eec_cqc_eqc_rdb_parameters.cqc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_cq,
		     log_num_cqs );
	DBGC ( hermon, "ConnectX3 %p ICM CQC is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_cqs ), hermon->cap.cqc_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Event queue contexts */
	len = ( ( 1 << log_num_eqs ) * hermon->cap.eqc_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 32,
		     qpc_eec_cqc_eqc_rdb_parameters.eqc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 33,
		     qpc_eec_cqc_eqc_rdb_parameters.eqc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_eq,
		     log_num_eqs );
	DBGC ( hermon, "ConnectX3 %p ICM EQC is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_eqs ), hermon->cap.eqc_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Memory translation table */
	len = ( ( 1 << log_num_mtts ) * hermon->cap.mtt_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 64,
		     tpt_parameters.mtt_base_addr_h, ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 65,
		     tpt_parameters.mtt_base_addr_l, icm_offset );
	DBGC ( hermon, "ConnectX3 %p ICM MTT is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_mtts ), hermon->cap.mtt_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Memory protection table */
	len = ( ( 1 << log_num_mpts ) * hermon->cap.dmpt_entry_size );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 60,
		     tpt_parameters.dmpt_base_adr_h, ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 61,
		     tpt_parameters.dmpt_base_adr_l, icm_offset );
	MLX_FILL_1 ( init_hca, 62,
		     tpt_parameters.log_dmpt_sz, log_num_mpts );
	DBGC ( hermon, "ConnectX3 %p ICM DMPT is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_mpts ), hermon->cap.dmpt_entry_size,
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;

	/* Multicast table */
	len = ( ( 1 << log_num_mcs ) * sizeof ( struct hermonprm_mcg_entry ) );
	icm_offset = icm_align ( icm_offset, len );
	MLX_FILL_1 ( init_hca, 48,
		     multicast_parameters.mc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 49,
		     multicast_parameters.mc_base_addr_l, icm_offset );
	MLX_FILL_1 ( init_hca, 52,
		     multicast_parameters.log_mc_table_entry_sz,
		     fls ( sizeof ( struct hermonprm_mcg_entry ) - 1 ) );
	MLX_FILL_1 ( init_hca, 53,
		     multicast_parameters.log_mc_table_hash_sz,
		     log_num_mcs - 1 );
	MLX_FILL_2 ( init_hca, 54,
		     multicast_parameters.log_mc_table_sz, log_num_mcs,
		     multicast_parameters.uc_group_steering,
		     ( hermon->cap.vep_uc_steering ? 1 : 0 ) );
	DBGC ( hermon, "ConnectX3 %p ICM MC is %d x %#zx at [%08llx,%08llx)\n",
	       hermon, ( 1 << log_num_mcs ),
	       sizeof ( struct hermonprm_mcg_entry ),
	       icm_offset, ( icm_offset + len ) );
	icm_offset += len;


	hermon->icm_map[HERMON_ICM_OTHER].len =
		( icm_offset - hermon->icm_map[HERMON_ICM_OTHER].offset );

	/*
	 * Allocate and map physical memory for (portions of) ICM
	 *
	 * Map is:
	 *   ICM AUX area (aligned to its own size)
	 *   cMPT areas
	 *   Other areas
	 */

	/* Calculate physical memory required for ICM */
	icm_len = 0;
	for ( i = 0 ; i < HERMON_ICM_NUM_REGIONS ; i++ ) {
		icm_len += hermon->icm_map[i].len;
	}

	/* Get ICM auxiliary area size */
	memset ( &icm_size, 0, sizeof ( icm_size ) );
	MLX_FILL_1 ( &icm_size, 0, value_hi, ( icm_offset >> 32 ) );
	MLX_FILL_1 ( &icm_size, 1, value, icm_offset );
	if ( ( rc = hermon_cmd_set_icm_size ( hermon, &icm_size,
					      &icm_aux_size ) ) != 0 ) {
		printf ( "ConnectX3 %p could not set ICM size: %s\n",
		       hermon, strerror ( rc ) );
		goto err_set_icm_size;
	}
	icm_aux_len = ( MLX_GET ( &icm_aux_size, value ) * HERMON_PAGE_SIZE );

	/* Allocate ICM data and auxiliary area */
	DBGC ( hermon, "ConnectX3 %p requires %zd kB ICM and %zd kB AUX ICM\n",
	       hermon, ( icm_len / 1024 ), ( icm_aux_len / 1024 ) );
	if ( ! hermon->icm ) {
		hermon->icm_len = icm_len;
		hermon->icm_aux_len = icm_aux_len;
		hermon->icm = umalloc ( hermon->icm_aux_len + hermon->icm_len );
		if ( ! hermon->icm ) {
			rc = -ENOMEM;
			printf ( "ConnectX3 %p could not allocate ICM %s\n",
				hermon, strerror ( rc ) );
			goto err_alloc;
		}
	} else {
		assert ( hermon->icm_len == icm_len );
		assert ( hermon->icm_aux_len == icm_aux_len );
	}
	icm_phys = user_to_phys ( hermon->icm, 0 );

	/* Map ICM auxiliary area */
	DBGC ( hermon, "ConnectX3 %p mapping ICM AUX => %08lx\n",
	       hermon, icm_phys );
	if ( ( rc = hermon_map_vpm ( hermon, hermon_cmd_map_icm_aux,
				     0, icm_phys, icm_aux_len ) ) != 0 ) {
		printf ( "ConnectX3 %p could not map AUX ICM: %s\n",
		       hermon, strerror ( rc ) );
		goto err_map_icm_aux;
	}
	icm_phys += icm_aux_len;

	/* MAP ICM area */
	for ( i = 0 ; i < HERMON_ICM_NUM_REGIONS ; i++ ) {
		DBGC ( hermon, "ConnectX3 %p mapping ICM %llx+%zx => %08lx\n",
		       hermon, hermon->icm_map[i].offset,
		       hermon->icm_map[i].len, icm_phys );
		if ( ( rc = hermon_map_vpm ( hermon, hermon_cmd_map_icm,
					     hermon->icm_map[i].offset,
					     icm_phys,
					     hermon->icm_map[i].len ) ) != 0 ){
			printf ( "ConnectX3 %p could not map ICM: %s\n",
			       hermon, strerror ( rc ) );
			goto err_map_icm;
		}
		icm_phys += hermon->icm_map[i].len;
	}

	return 0;

 err_map_icm:
	assert ( i == 0 ); /* We don't handle partial failure at present */
 err_map_icm_aux:
	hermon_cmd_unmap_icm_aux ( hermon );
 err_alloc:
 err_set_icm_size:
	return rc;
}

/**
 * Unmap ICM
 *
 * @v hermon		ConnectX3 device
 */
static void hermon_unmap_icm ( struct hermon *hermon ) {
	struct hermonprm_scalar_parameter unmap_icm;
	int i;

	for ( i = ( HERMON_ICM_NUM_REGIONS - 1 ) ; i >= 0 ; i-- ) {
		memset ( &unmap_icm, 0, sizeof ( unmap_icm ) );
		MLX_FILL_1 ( &unmap_icm, 0, value_hi,
			     ( hermon->icm_map[i].offset >> 32 ) );
		MLX_FILL_1 ( &unmap_icm, 1, value,
			     hermon->icm_map[i].offset );
		hermon_cmd_unmap_icm ( hermon,
				       ( 1 << fls ( ( hermon->icm_map[i].len /
						      HERMON_PAGE_SIZE ) - 1)),
				       &unmap_icm );
	}
	hermon_cmd_unmap_icm_aux ( hermon );
}

/***************************************************************************
 *
 * Flash access operations
 *
 ***************************************************************************
 */
#define HERMON_TLV_ACCESS_READ		0x1
#define HERMON_TLV_ACCESS_WRITE		0x2
#define HERMON_TLV_ACCESS_INVALIDATE	0x3

static int hermon_access_tlv ( struct hermon *hermon,
		struct driver_tlv_header *tlv_header,
		uint8_t access_method) {
	int rc, op, reg_id;
	unsigned int i, length = 0;
	uint32_t *u32ptr;
	uint8_t *u8ptr;
	uint8_t pad_cnt = 0;
	struct hermonprm_register_access register_access;
	uint32_t op_length = (tlv_header->length + 3) / sizeof(uint32_t);

	if ( ! hermon->cap.nv_mem_access_supported ) {
		return -ENOTSUP;
	}

	memset ( &register_access, 0, sizeof ( register_access ) );
	MLX_FILL_2(&register_access, 0,
				len, 0x4,
				op_type, 0x1);
	MLX_FILL_2(&register_access, 4,
				reg_tlv_len, op_length + 2,
				reg_tlv_type, 0x03);
	MLX_FILL_2(&register_access, 5,
				tlv_hdr_reg_type, tlv_header->type,
				tlv_hdr_length, op_length);

	pad_cnt = op_length * sizeof(uint32_t) - tlv_header->length;
	MLX_FILL_3(&register_access, 6,
				tlv_hdr_type_mod, (tlv_header->type_mod & 0xff),
				tlv_hdr_pad_cnt, (pad_cnt & 0x3),
				tlv_hdr_type_ver, (tlv_header->version & 0xf));

	if (access_method == HERMON_TLV_ACCESS_READ) {
		op = 0x1;
		reg_id = HERMON_REGISTER_ID_SET_GET;
	} else if (access_method == HERMON_TLV_ACCESS_INVALIDATE) {
		op = 0x2;
		reg_id = HERMON_REGISTER_ID_INVALIDATE;
	} else { /* HERMON_TLV_ACCESS_WRITE */
		op = 0x2;
		reg_id = HERMON_REGISTER_ID_SET_GET;
		memcpy((uint8_t *)(&register_access) + 0x1c, tlv_header->data, tlv_header->length);
	}

	MLX_FILL_2 ( &register_access, 1,
			method, op,
			register_id, reg_id);

	if  ( ( rc = hermon_cmd_register_access ( hermon ,
					&register_access ) ) != 0 ) {
		DBGC(hermon, "Error %s:%d [%x] - Failed to run 'Register Access' command\n", __FUNCTION__, __LINE__,rc);
		return rc;
	}

	if ( MLX_GET ( &register_access, status ) != 0 ) {
		DBGC(hermon, "Error %s:%d - 'Register Access' command status %x\n",
			__FUNCTION__, __LINE__, MLX_GET ( &register_access, status ));
		return -EINVAL;
	}

	if (access_method != HERMON_TLV_ACCESS_READ) {
		return 0;
	}

	tlv_header->version = MLX_GET( &register_access, tlv_hdr_type_ver );

	length = MLX_GET ( &register_access, tlv_hdr_length );
	if ( length == 0 ) {
		DBGC(hermon, "%s[%d]: Returned TLV with size 0\n", __FUNCTION__, __LINE__);
		return 0;
	}

	u8ptr = ((uint8_t *)&register_access) + HERMON_REGISTER_ACCESS_DATA_OFFSET;
	u32ptr = (uint32_t *) u8ptr;

	switch (tlv_header->type) {
	case DHCP_VEND_ID			:
	case ISCSI_INITIATOR_IPV4_ADDR		:
	case ISCSI_INITIATOR_SUBNET		:
	case ISCSI_INITIATOR_IPV4_GATEWAY	:
	case ISCSI_INITIATOR_IPV4_PRIM_DNS	:
	case ISCSI_INITIATOR_IPV4_SECDNS	:
	case ISCSI_INITIATOR_NAME		:
	case ISCSI_INITIATOR_CHAP_ID		:
	case ISCSI_INITIATOR_CHAP_PWD		:
	case CONNECT_FIRST_TGT			:
	case FIRST_TGT_IP_ADDRESS		:
	case FIRST_TGT_TCP_PORT			:
	case FIRST_TGT_BOOT_LUN			:
	case FIRST_TGT_ISCSI_NAME		:
	case FIRST_TGT_CHAP_ID			:
	case FIRST_TGT_CHAP_PWD			:
		break;
	default:
		for ( i = 0; i * 4 < tlv_header->length; i++ ) {
			u32ptr[i] = be32_to_cpu(u32ptr[i]);
		}
	}

	pad_cnt = MLX_GET ( &register_access, tlv_hdr_pad_cnt );
	pad_cnt &= 0x3;

	memset(tlv_header->data, 0, tlv_header->length);
	if (tlv_header->length < ((length * sizeof(uint32_t)) - pad_cnt)) {
		DBGC ( hermon, "%s: Buffer overflow. (buffer len = %d ; TLV len = %d ; pad_cnt = %d)\n",
			__FUNCTION__, tlv_header->length, (length * sizeof(uint32_t)), pad_cnt );
		return -EINVAL;
	}
	memcpy ((uint8_t *)tlv_header->data, u8ptr, (length * sizeof(uint32_t)) - pad_cnt);

	return 0;
}

int hermon_read_conf_tlv ( struct hermon *hermon,
				struct driver_tlv_header *tlv_header ) {
	return hermon_access_tlv(hermon, tlv_header, HERMON_TLV_ACCESS_READ);
}

static int hermon_write_conf_tlv ( struct hermon *hermon,
				struct driver_tlv_header *tlv_header ) {
	return hermon_access_tlv(hermon, tlv_header, HERMON_TLV_ACCESS_WRITE);
}

static int hermon_invalidate_conf_tlv ( struct hermon *hermon,
				struct driver_tlv_header *tlv_header ) {
	return hermon_access_tlv(hermon, tlv_header, HERMON_TLV_ACCESS_INVALIDATE);
}

int hermon_flash_write_tlv ( void *priv, void *src,
		uint32_t port_num, uint32_t tlv_type, uint32_t len ) {
	struct hermon *hermon = ( struct hermon * ) priv;
	struct driver_tlv_header tlv;
	int rc;

	if ( ! hermon->cap.nv_mem_access_supported ) {
		return -ENOTSUP;
	}
	memset ( &tlv, 0, sizeof ( tlv ) );
	tlv.type        = tlv_type;
	tlv.length      = len;
	tlv.type_mod    = port_num;
	tlv.version     = driver_settings_get_tlv_version ( tlv_type );
	tlv.data        = src;

	if ( ( rc = hermon_write_conf_tlv ( hermon, &tlv ) ) )
		DBGC ( hermon, "Failed to save tlv type %d in the flash (rc = %d)\n",
			tlv_type, rc );
	return rc;
}

int hermon_flash_invalidate_tlv ( void *priv, uint32_t port_num,
				uint32_t tlv_type ) {
	struct hermon *hermon = ( struct hermon * ) priv;
	struct driver_tlv_header tlv;
	int rc;

	memset ( &tlv, 0, sizeof ( tlv ) );
	tlv.type        = tlv_type;
	tlv.type_mod    = port_num;
	tlv.version     = driver_settings_get_tlv_version ( tlv_type );

	if ( ( rc = hermon_invalidate_conf_tlv ( hermon, &tlv ) ) ) {
		DBGC ( hermon, "Failed to invalidate tlv type %d (rc = %d)\n",
			tlv_type, rc );
	}

	return rc;
}

int hermon_flash_read_tlv_wrapper ( void *drv_priv,
		struct driver_tlv_header *tlv_hdr ) {
	return hermon_read_conf_tlv ( drv_priv, tlv_hdr );
}

void hermon_get_ro_pci_settings ( void *drv_priv ) {
	struct hermon *hermon = ( struct hermon * ) drv_priv;
	struct pci_device *pci = hermon->pci;
    struct firmware_image_props *fw_image_props;
    struct hermonprm_query_fw fw;
	int rc;

	fw_image_props = & ( hermon->hermon_nv_conf.fw_image_props );

	memset ( &fw, 0, sizeof ( fw ) );
	/* Get firmware version */
	if ( ( rc = hermon_cmd_query_fw ( hermon, &fw ) ) != 0 ) {
		DBGC ( hermon, "%s: Failed to query firmware: %d\n", __FUNCTION__, rc );
	}

	snprintf ( fw_image_props->family_fw_version,
			sizeof ( fw_image_props->family_fw_version ),
			"%d.%d.%d", MLX_GET ( &fw, fw_rev_major ),
			MLX_GET ( &fw, fw_rev_minor ),
			MLX_GET ( &fw, fw_rev_subminor ));

	strcpy ( hermon->hermon_nv_conf.bdf_name, pci->dev.name );
	strcpy ( hermon->hermon_nv_conf.device_name, pci->id->name );
	hermon->hermon_nv_conf.desc_dev_id =  pci->dev.desc.device;
	hermon->hermon_nv_conf.driver_name =  pci->dev.driver_name;
}

static int hermon_port_get_defaults ( struct hermon *hermon, unsigned int port_num ) {
	struct hermon_port *port = & ( hermon->port[port_num - 1] );
	union hermonprm_query_defparams def;
	int rc;

	memset ( &def, 0, sizeof ( def ) );
	if ( ( rc = hermon_cmd_query_defparams ( hermon, &def, port_num ) ) ) {
		DBGC ( hermon, "Failed to get port %d default values (rc = %d)\n", rc, port_num );
		return rc;
	} else {
		port->defaults.pptx				= MLX_GET ( &def.port, pptx );
		port->defaults.pprx             		= MLX_GET ( &def.port, pprx );
		port->defaults.boot_option_rom_en		= MLX_GET ( &def.port, boot_option_rom_en );
		port->defaults.boot_vlan_en     		= MLX_GET ( &def.port, boot_vlan_en );
		port->defaults.boot_retry_count 		= MLX_GET ( &def.port, boot_retry_count );
		port->defaults.boot_protocol    		= MLX_GET ( &def.port, boot_protocol );
		port->defaults.boot_vlan        		= MLX_GET ( &def.port, boot_vlan_id );
		port->defaults.boot_pkey			= MLX_GET ( &def.port, boot_pkey );
		port->defaults.en_wol_magic			= MLX_GET ( &def.port, en_wol_magic );
		port->defaults.network_link_type		= MLX_GET ( &def.port, network_link_type );
		port->defaults.iscsi_boot_to_target		= MLX_GET ( &def.port, iscsi_boot_to_target );
		port->defaults.iscsi_vlan_en			= MLX_GET ( &def.port, iscsi_vlan_en );
		port->defaults.iscsi_tcp_timestamps_en		= MLX_GET ( &def.port, iscsi_tcp_timestamps_en );
		port->defaults.iscsi_chap_mutual_auth_en	= MLX_GET ( &def.port, iscsi_chap_mutual_auth_en );
		port->defaults.iscsi_chap_auth_en		= MLX_GET ( &def.port, iscsi_chap_auth_en );
		port->defaults.iscsi_dhcp_params_en		= MLX_GET ( &def.port, iscsi_dhcp_params_en );
		port->defaults.iscsi_ipv4_dhcp_en		= MLX_GET ( &def.port, iscsi_ipv4_dhcp_en );
		port->defaults.iscsi_lun_busy_retry_count	= MLX_GET ( &def.port, iscsi_lun_busy_retry_count );
		port->defaults.iscsi_link_up_delay_time		= MLX_GET ( &def.port, iscsi_link_up_delay_time );
		port->defaults.client_identifier	= MLX_GET ( &def.port, client_identifier );
		port->defaults.mac_admin_bit	= MLX_GET ( &def.port, mac_admin_bit );
		port->defaults.linkup_timeout			= MLX_GET ( &def.port, boot_link_up_timeout );
		port->defaults.ip_ver	= MLX_GET ( &def.port, boot_ip_ver );
	}
	return 0;
}

static int hermon_get_ini_and_defaults ( struct hermon *hermon ) {
	struct hermonprm_query_romini ini;
	union hermonprm_query_defparams def;
	int rc, i;

	memset ( &ini, 0, sizeof ( ini ) );
	if ( ( rc = hermon_cmd_query_romini ( hermon, &ini ) ) ) {
		DBGC ( hermon, "Failed to get ini values (rc = %d)\n", rc );
		return rc;
	} else {
		hermon->ini_configurations.dhcp_user_class[0]	= MLX_GET ( &ini, dhcp_user_class_0 );
		hermon->ini_configurations.dhcp_user_class[1]   = MLX_GET ( &ini, dhcp_user_class_1 );
		hermon->ini_configurations.dhcp_user_class[2]   = MLX_GET ( &ini, dhcp_user_class_2 );
		hermon->ini_configurations.dhcp_user_class[3]   = MLX_GET ( &ini, dhcp_user_class_3 );
		hermon->ini_configurations.dhcp_user_class[4]   = MLX_GET ( &ini, dhcp_user_class_4 );
		hermon->ini_configurations.dhcp_user_class[5]   = MLX_GET ( &ini, dhcp_user_class_5 );
		hermon->ini_configurations.dhcp_user_class[6]   = MLX_GET ( &ini, dhcp_user_class_6 );
		hermon->ini_configurations.dhcp_user_class[7]   = MLX_GET ( &ini, dhcp_user_class_7 );
		hermon->ini_configurations.uri_boot_retry_delay = MLX_GET ( &ini, uri_boot_retry_delay );
		hermon->ini_configurations.uri_boot_retry 	= MLX_GET ( &ini, uri_boot_retry );
		hermon->ini_configurations.option_rom_debug 	= MLX_GET ( &ini, option_rom_debug );
		hermon->ini_configurations.promiscuous_vlan 	= MLX_GET ( &ini, promiscuous_vlan );
	}

	/* Get the global default values */
	memset ( &def, 0, sizeof ( def ) );
	if ( ( rc = hermon_cmd_query_defparams ( hermon, &def, 0 ) ) ) {
		DBGC ( hermon, "Failed to get global default values (rc = %d)\n", rc );
		return rc;
	} else {
		hermon->defaults.total_vfs              = MLX_GET ( &def.global, num_vfs );
		hermon->defaults.sriov_en		= MLX_GET ( &def.global, sriov_en );
		hermon->defaults.maximum_uar_bar_size   = MLX_GET ( &def.global, max_uar_bar_size );
		hermon->defaults.uar_bar_size   	= MLX_GET ( &def.global, uar_bar_size );
		hermon->defaults.flexboot_menu_to   	= MLX_GET ( &def.global, flexboot_menu_to );
	}

        /* Set max virtual functions
         * caps.max_funix contains 1 Pf, so the Vf number is (caps.max_funix - 1)
         */
	hermon->defaults.max_vfs = ( ( hermon->cap.max_funix < 1 ) ? 1 : ( hermon->cap.max_funix - 1 ) );

	/* Get the ports default values */
	for ( i = 0 ; i < hermon->cap.num_ports ; i++ ) {
		if ( ( rc = hermon_port_get_defaults ( hermon, i + 1 ) ) )
			return rc;
	}

	return 0;
}

/***************************************************************************
 *
 * Initialisation and teardown
 *
 ***************************************************************************
 */


/**
 * Reset device
 *
 * @v hermon		ConnectX3 device
 */
static void hermon_reset ( struct hermon *hermon,
			   unsigned int reset_type ) {
	struct pci_device *pci = hermon->pci;
	struct pci_config_backup backup;
	static const uint8_t backup_exclude[] =
		PCI_CONFIG_BACKUP_EXCLUDE ( 0x58, 0x5c );

	mdelay ( HERMON_RESET_WAIT_TIME_MS );
	/* Perform device reset and preserve PCI configuration */
	pci_backup ( pci, &backup, backup_exclude );
	writel ( reset_type,
		 ( hermon->config + HERMON_RESET_OFFSET ) );
	mdelay ( HERMON_RESET_WAIT_TIME_MS );
	pci_restore ( pci, &backup, backup_exclude );

	/* Reset command interface toggle */
	hermon->toggle = 0;
}

/**
 * Set up memory protection table
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
static int hermon_setup_mpt ( struct hermon *hermon ) {
	struct hermonprm_mpt mpt;
	uint32_t key;
	int rc;

	/* Derive key */
	key = ( hermon->cap.reserved_mrws | HERMON_MKEY_PREFIX );
	hermon->lkey = ( ( key << 8 ) | ( key >> 24 ) );

	/* Initialise memory protection table */
	memset ( &mpt, 0, sizeof ( mpt ) );
	MLX_FILL_7 ( &mpt, 0,
		     atomic, 1,
		     rw, 1,
		     rr, 1,
		     lw, 1,
		     lr, 1,
		     pa, 1,
		     r_w, 1 );
	MLX_FILL_1 ( &mpt, 2, mem_key, key );
	MLX_FILL_1 ( &mpt, 3,
		     pd, HERMON_GLOBAL_PD );
	MLX_FILL_1 ( &mpt, 10, len64, 1 );
	if ( ( rc = hermon_cmd_sw2hw_mpt ( hermon,
					   hermon->cap.reserved_mrws,
					   &mpt ) ) != 0 ) {
		printf ( "ConnectX3 %p could not set up MPT: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	return 0;
}

static int hermon_unmap_mpt ( struct hermon *hermon )
{
	int rc;

	if ( ( rc = hermon_cmd_hw2sw_mpt ( hermon, hermon->cap.reserved_mrws ) ) != 0 ) {
		printf ( "ConnectX3 %p could not unmap the MPT: %s\n", hermon, strerror ( rc ) );
		return rc;
	}
	return 0;
}

static void hermon_arm_cq ( struct hermon *hermon, struct net_device *netdev ) {
	struct hermon_port *port = netdev->priv;
	struct ib_completion_queue *cq = port->eth_cq;

	arm_cq ( hermon, cq );
}

/**
 * Configure special queue pairs
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
static int hermon_configure_special_qps ( struct hermon *hermon ) {
	int rc;

	/* Special QP block must be aligned on its own size */
	hermon->special_qpn_base = ( ( hermon->cap.reserved_qps +
				       HERMON_NUM_SPECIAL_QPS - 1 )
				     & ~( HERMON_NUM_SPECIAL_QPS - 1 ) );
	hermon->qpn_base = ( hermon->special_qpn_base +
			     HERMON_NUM_SPECIAL_QPS );
	DBGC ( hermon, "ConnectX3 %p special QPs at [%lx,%lx]\n", hermon,
	       hermon->special_qpn_base, ( hermon->qpn_base - 1 ) );

	/* Issue command to configure special QPs */
	if ( ( rc = hermon_cmd_conf_special_qp ( hermon, 0x00,
					hermon->special_qpn_base ) ) != 0 ) {
		printf ( "ConnectX3 %p could not configure special QPs: "
		       "%s\n", hermon, strerror ( rc ) );
		return rc;
	}

	return 0;
}

static int hermon_init ( struct hermon *hermon ) {
	struct hermonprm_init_hca init_hca;
	unsigned int i;
	int rc;

	/* Allocate and map ICM */
	memset ( &init_hca, 0, sizeof ( init_hca ) );
	if ( ( rc = hermon_map_icm ( hermon, &init_hca ) ) != 0 ) {
		printf ( "ConnectX3 %p could not map ICM %s\n",
			hermon, strerror ( rc ) );
		goto err_map_icm;
	}

	/* Initialise HCA */
	MLX_FILL_1 ( &init_hca, 0, version, 0x02 /* "Must be 0x02" */ );
	MLX_FILL_1 ( &init_hca, 5, udp, 1 );
	MLX_FILL_1 ( &init_hca, 74, uar_parameters.log_max_uars, 8 );
	if ( ( rc = hermon_cmd_init_hca ( hermon, &init_hca ) ) != 0 ) {
		printf ( "ConnectX3 %p could not initialise HCA: %s\n",
		       hermon, strerror ( rc ) );
		goto err_init_hca;
	}

	/* Set up memory protection */
	if ( ( rc = hermon_setup_mpt ( hermon ) ) != 0 ) {
		printf ( "ConnectX3 %p could not setup MPT %s\n",
			hermon, strerror ( rc ) );
		goto err_setup_mpt;
	}
	for ( i = 0 ; i < hermon->num_ports ; i++ ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		hermon->port[i].ibdev->rdma_key = hermon->lkey;
	}

	/* Set up event queue */
	if ( ( rc = hermon_create_eq ( hermon ) ) != 0 ) {
		printf ( "ConnectX3 %p could not create EQ %s\n",
			hermon, strerror ( rc ) );
		goto err_create_eq;
	}

	/* Configure special QPs */
	if ( ( rc = hermon_configure_special_qps ( hermon ) ) != 0 ) {
		printf ( "ConnectX3 %p could not configure SQPs %s\n",
			hermon, strerror ( rc ) );
		goto err_conf_special_qps;
	}

	hermon->mcg_aux_index = ( MC_TABLE_SIZE / 2 );
	INIT_LIST_HEAD ( &hermon->ncsi_mgids );

	return 0;

 err_conf_special_qps:
	hermon_destroy_eq ( hermon );
 err_create_eq:
	hermon_unmap_mpt ( hermon );
 err_setup_mpt:
	hermon_cmd_close_hca ( hermon );
 err_init_hca:
	hermon_unmap_icm ( hermon );
 err_map_icm:
	return rc;
}

/**
 * Start ConnectX3 device
 *
 * @v hermon		ConnectX3 device
 * @v running		Firmware is already running
 * @ret rc		Return status code
 */
static int hermon_start ( struct hermon *hermon, int running ) {
	struct hermonprm_query_adapter query_adapter;
	int rc;

	/* Start firmware if not already running */
	if ( ! running ) {
		hermon_reset ( hermon, HERMON_RESET_START );
		if ( ( rc = hermon_start_firmware ( hermon ) ) != 0 ) {
			printf ( "ConnectX3 %p could not start firmware %s\n",
				hermon, strerror ( rc ) );
			goto err_start_firmware;
		}
	}

	/* Get intapin - for clearing interrupts */
	memset( &query_adapter, 0, sizeof ( query_adapter ) );
	if ( ( rc = hermon_cmd_query_adapter ( hermon, &query_adapter ) ) != 0 ) {
		printf ( "ConnectX3 %p could not query adapter %s\n",
				hermon, strerror ( rc ) );
		goto err_query_adapter;
	}
	hermon->intapin = MLX_GET ( &query_adapter, intapin );

	if ( ! boot_post_shell ) {
		if ( ( rc = hermon_init ( hermon ) ) != 0 ) {
			printf ( "ConnectX3 %p could not init firmware %s\n",
				hermon, strerror ( rc ) );
			goto err_hermon_init;
		}
	}

	return 0;

 err_hermon_init:
 err_query_adapter:
	hermon_stop_firmware ( hermon );
 err_start_firmware:
	return rc;
}

/**
 * Stop ConnectX3 device
 *
 * @v hermon		ConnectX3 device
 */
static void hermon_stop ( struct hermon *hermon ) {
	if ( ! boot_post_shell ) {
		hermon_destroy_eq ( hermon );
		hermon_unmap_mpt ( hermon );
		hermon_cmd_close_hca ( hermon );
		hermon_unmap_icm ( hermon );
	}
	hermon_stop_firmware ( hermon );
	//hermon_reset ( hermon, be32_to_cpu ( HERMON_RESET_END ) );
	/* Reset command interface toggle */
	hermon->toggle = 0;
}

/**
 * Open ConnectX3 device
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
int hermon_open ( struct hermon *hermon ) {
	int rc;

	/* Start device if applicable */
	if ( hermon->open_count == 0 ) {
		if ( ( rc = hermon_start ( hermon, 0 ) ) != 0 )
			return rc;
	}
	/* Increment open counter */
	hermon->open_count++;

	return 0;
}

/**
 * Close ConnectX3 device
 *
 * @v hermon		ConnectX3 device
 */
void hermon_close ( struct hermon *hermon ) {

	/* Decrement open counter */
	assert ( hermon->open_count != 0 );
	hermon->open_count--;

	/* Stop device if applicable */
	if ( hermon->open_count == 0 )
		hermon_stop ( hermon );
}

/***************************************************************************
 *
 * Infiniband link-layer operations
 *
 ***************************************************************************
 */

/**
 * Initialise Infiniband link
 *
 * @v ibdev		Infiniband device
 * @ret rc		Return status code
 */
static int hermon_ib_open ( struct ib_device *ibdev ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermon_port *port = & ( hermon->port[ibdev->port - 1] );
	union hermonprm_set_port set_port;
	uint16_t nvpkey = 0;
	int rc;

	/* Open hardware */
	if ( ( rc = hermon_open ( hermon ) ) != 0 )
		goto err_open;

	/* Set port parameters */
	memset ( &set_port, 0, sizeof ( set_port ) );
	MLX_FILL_8 ( &set_port.ib, 0,
		     mmc, 1,
		     mvc, 1,
		     mp, 1,
		     mg, 1,
		     mtu_cap, IB_MTU_2048,
		     vl_cap, IB_VL_0,
		     rcm, 1,
		     lss, 1 );
	MLX_FILL_2 ( &set_port.ib, 10,
		     max_pkey, 32,
		     max_gid, 1 );
	MLX_FILL_1 ( &set_port.ib, 28,
		     link_speed_supported, 1 );
	if ( ( rc = hermon_cmd_set_port ( hermon, ibdev->port,
					  &set_port, 0) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not set port: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		goto err_set_port;
	}

	/* Initialise port */
	if ( ( rc = hermon_cmd_init_port ( hermon, ibdev->port ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not initialise port: "
		       "%s\n", hermon, ibdev->port, strerror ( rc ) );
		goto err_init_port;
	}

	/* Update MAD parameters */
	nvpkey = driver_settings_get_ib_pkey_val (
			& ( port->driver_settings.generic_settings.settings ) );
	if ( nvpkey )
		ibdev->pkey = nvpkey;
	ib_smc_update ( ibdev, hermon_mad );

	return 0;

 err_init_port:
 err_set_port:
	hermon_close ( hermon );
 err_open:
	return rc;
}

/**
 * Close Infiniband link
 *
 * @v ibdev		Infiniband device
 */
static void hermon_ib_close ( struct ib_device *ibdev ) {
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	int rc;

	/* Close port */
	if ( ( rc = hermon_cmd_close_port ( hermon, ibdev->port ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not close port: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		/* Nothing we can do about this */
	}

	/* Close hardware */
	hermon_close ( hermon );
}

/**
 * Inform embedded subnet management agent of a received MAD
 *
 * @v ibdev		Infiniband device
 * @v mad		MAD
 * @ret rc		Return status code
 */
static int hermon_inform_sma ( struct ib_device *ibdev,
			       union ib_mad *mad ) {
	int rc;

	/* Send the MAD to the embedded SMA */
	if ( ( rc = hermon_mad ( ibdev, mad ) ) != 0 )
		return rc;

	/* Update parameters held in software */
	ib_smc_update ( ibdev, hermon_mad );

	return 0;
}

/** ConnectX3 Infiniband operations */
static struct ib_device_operations hermon_ib_operations = {
	.create_cq	= hermon_create_cq,
	.destroy_cq	= hermon_destroy_cq,
	.create_qp	= hermon_create_qp,
	.modify_qp	= hermon_modify_qp,
	.destroy_qp	= hermon_destroy_qp,
	.post_send	= hermon_post_send,
	.post_recv	= hermon_post_recv,
	.poll_cq	= hermon_poll_cq,
	.poll_eq	= hermon_poll_eq,
	.open		= hermon_ib_open,
	.close		= hermon_ib_close,
	.mcast_attach	= hermon_mcast_attach,
	.mcast_detach	= hermon_mcast_detach,
	.set_port_info	= hermon_inform_sma,
	.set_pkey_table	= hermon_inform_sma,
};

static void hermon_guid2mac ( const void *hw_addr, void *ll_addr ) {
	const uint8_t *guid = hw_addr;
	uint8_t *eth_addr = ll_addr;
	uint8_t mac_admin_bit_mask;

	if ( g_mac_admin_bit < MAC_ADMIN_BIT_FACTORY_MAC ) {
		mac_admin_bit_mask = ( g_mac_admin_bit == 2 ) ? 0x02 : 0x00;
		eth_addr[0] = ( guid[0] | mac_admin_bit_mask );
		eth_addr[1] = guid[1];
		eth_addr[2] = guid[2];
		eth_addr[3] = guid[5];
		eth_addr[4] = guid[6];
		eth_addr[5] = guid[7];
	} else {
		memcpy ( eth_addr, g_fact_mac_addr, ETH_ALEN );
	}

}

/**
 * Register ConnectX3 Infiniband device
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 * @ret rc		Return status code
 */
static int hermon_register_ibdev ( struct hermon *hermon,
				   struct hermon_port *port ) {
	struct ib_device *ibdev = port->ibdev;
	int rc;

	/* Initialize the ipoib_hw_guid2mac function in ipoib in
	 * case we want to use FW GUID2MAC */
	ipoib_hw_guid2mac = NULL;
	if ( g_mac_admin_bit > MAC_ADMIN_BIT_DEFAULT ) {
		ipoib_hw_guid2mac = hermon_guid2mac;
	}

	/* Initialise parameters using SMC */
	ib_smc_init ( ibdev, hermon_mad );

	/* Register Infiniband device */
	if ( ( rc = register_ibdev ( ibdev ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not register IB "
		       "device: %s\n", hermon, ibdev->port, strerror ( rc ) );
		return rc;
	}

	port->netdev = ib_get_ownerdata ( ibdev );

	return 0;
}

/**
 * Handle ConnectX3 Infiniband device port state change
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 * @v link_up		Link is up
 */
static void hermon_state_change_ibdev ( struct hermon *hermon __unused,
					struct hermon_port *port,
					int link_up __unused ) {
	struct ib_device *ibdev = port->ibdev;

	/* Update MAD parameters */
	ib_smc_update ( ibdev, hermon_mad );
}

/**
 * Unregister ConnectX3 Infiniband device
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 */
static void hermon_unregister_ibdev ( struct hermon *hermon __unused,
				      struct hermon_port *port ) {
	struct ib_device *ibdev = port->ibdev;

	unregister_ibdev ( ibdev );
}

/** ConnectX3 Infiniband port type */
static struct hermon_port_type hermon_port_type_ib = {
	.register_dev = hermon_register_ibdev,
	.state_change = hermon_state_change_ibdev,
	.unregister_dev = hermon_unregister_ibdev,
};

/***************************************************************************
 *
 * Ethernet operation
 *
 ***************************************************************************
 */

/** Number of ConnectX3 Ethernet send work queue entries */
#define HERMON_ETH_NUM_SEND_WQES 64

/** Number of ConnectX3 Ethernet receive work queue entries */
#define HERMON_ETH_NUM_RECV_WQES 64

/** Number of ConnectX3 Ethernet completion entries */
#define HERMON_ETH_NUM_CQES (HERMON_ETH_NUM_SEND_WQES + HERMON_ETH_NUM_RECV_WQES)

int hermon_eth_add_steer ( struct ib_device *ibdev,
			   struct ib_queue_pair *eth_qp )
{
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	struct hermonprm_eth_mgi mgi;
	uint32_t in_mod;
	int rc;

	/* Join broadcast mutlicast group */
	memset ( &mgi, 0, sizeof ( mgi ) );
	MLX_FILL_3 ( &mgi, 1, vep_num, hermon_get_port_from_ibdev(ibdev)->vep_number,
		     portnum, ibdev->port,
		     unicast, MLX4_MC_STEER );
	MLX_FILL_1 ( &mgi, 2, mac_h, 0xffff );
	MLX_FILL_1 ( &mgi, 3, mac_l, 0xffffffffUL );
	if ( ( rc = ib_mcast_attach
			  ( ibdev, eth_qp, ( union ib_gid * ) &mgi ) ) != 0 ) {
		printf ( "ConnectX3 %p QPN %lx could not attach to "
		       "broadcast MCG: %s\n", hermon, eth_qp->qpn,
		       strerror ( rc ) );
		return rc;
	}

	/* Add MAC to unicast group */
	memset ( &mgi, 0, sizeof ( mgi ) );
	MLX_FILL_3 ( &mgi, 1, vep_num, hermon_get_port_from_ibdev(ibdev)->vep_number,
		     portnum, ibdev->port,
		     unicast, MLX4_UC_STEER );
	MLX_FILL_1 ( &mgi, 2, mac_h, hermon_get_port_from_ibdev(ibdev)->eth_mac_h );
	MLX_FILL_1 ( &mgi, 3, mac_l, hermon_get_port_from_ibdev(ibdev)->eth_mac_l );
	if ( ( rc = ib_mcast_attach
			  ( ibdev, eth_qp, ( union ib_gid * ) &mgi ) ) != 0 ) {
		printf ( "ConnectX3 %p QPN %lx could not attach to "
		       "unicast MCG: %s\n", hermon, eth_qp->qpn,
		       strerror ( rc ) );
		return rc;
	}

	/* Add default UC MCG */
	in_mod = ( ( MLX4_UC_STEER << 1 ) |
		   ( ibdev->port << 16 ) |
		   ( hermon_get_port_from_ibdev(ibdev)->vep_number << 24 ) );
	if ( ( rc = hermon_default_mcast_attach
			  ( ibdev, eth_qp, in_mod ) != 0 ) ) {
		printf ( "ConnectX3 %p QPN %lx could not attach to "
		       "unicast default MCG: %s\n", hermon, eth_qp->qpn,
		       strerror ( rc ) );
		return rc;
	}

	/* Add default MC MCG */
	in_mod = ( ( MLX4_MC_STEER << 1 ) |
		   ( ibdev->port << 16 ) |
		   ( hermon_get_port_from_ibdev(ibdev)->vep_number << 24 ) );
	if ( ( rc = hermon_default_mcast_attach
			  ( ibdev, eth_qp, in_mod ) ) != 0 ) {
		printf ( "ConnectX3 %p QPN %lx could not attach to "
		       "multicast default MCG: %s\n", hermon, eth_qp->qpn,
		       strerror ( rc ) );
		return rc;
	}

	return 0;
}

void hermon_eth_release_steer ( struct ib_device *ibdev,
				struct ib_queue_pair *eth_qp )
{
	union ib_gid gid;
	struct ib_multicast_gid *mgid;
	struct ib_multicast_gid *tmp;
	uint32_t in_mod;

	list_for_each_entry_safe ( mgid, tmp, &eth_qp->mgids, list ) {
		memcpy ( &gid, &mgid->gid, sizeof ( union ib_gid ) );
		ib_mcast_detach ( ibdev, eth_qp, &gid );
	}

	in_mod = ( ( MLX4_MC_STEER << 1 ) |
		   ( ibdev->port << 16 ) |
		   ( hermon_get_port_from_ibdev(ibdev)->vep_number << 24 ) );
	hermon_default_mcast_detach ( ibdev, eth_qp, in_mod );

	in_mod = ( ( MLX4_UC_STEER << 1 ) |
		   ( ibdev->port << 16 ) |
		   ( hermon_get_port_from_ibdev(ibdev)->vep_number << 24 ) );
	hermon_default_mcast_detach ( ibdev, eth_qp, in_mod );
}

/**
 * Transmit packet via ConnectX3 Ethernet device
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int hermon_eth_transmit ( struct net_device *netdev,
				 struct io_buffer *iobuf ) {
	struct hermon_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	int rc;

	/* Transmit packet */
	if ( ( rc = ib_post_send ( ibdev, port->eth_qp, NULL,
				   iobuf ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p port %d could not transmit: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/** Hermon Ethernet queue pair operations */
static struct ib_queue_pair_operations hermon_eth_qp_op = {
	.alloc_iob = alloc_iob,
};

/**
 * Handle ConnectX3 Ethernet device send completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void hermon_eth_complete_send ( struct ib_device *ibdev __unused,
				       struct ib_queue_pair *qp,
				       struct io_buffer *iobuf, int rc ) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );

	netdev_tx_complete_err ( netdev, iobuf, rc );
}

/**
 * Handle ConnectX3 Ethernet device receive completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector, or NULL
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void hermon_eth_complete_recv ( struct ib_device *ibdev __unused,
				       struct ib_queue_pair *qp,
				       struct ib_address_vector *dest __unused,
				       struct ib_address_vector *source,
				       struct io_buffer *iobuf, int rc ) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );
	struct net_device *vlan;
	int promisc_vlan_enabled;

	if ( rc != 0 ) {
		DBG ( "Received packet with error\n" );
		netdev_rx_err ( netdev, iobuf, rc );
		return;
	}

	if ( ! source ) {
		DBG ( "Received packet without address vector\n" );
		netdev_rx_err ( netdev, iobuf, -ENOTTY );
		return;
	}
	/* Find VLAN device, if applicable */
	if ( source->vlan_present ) {
		if ( ( vlan = vlan_find ( netdev, source->vlan ) ) != NULL ) {
			netdev = vlan;
		} else {
			promisc_vlan_enabled = fetch_intz_setting ( netdev_settings ( netdev ), &promisc_vlan_setting );
			if ( promisc_vlan_enabled == 0 ) {
				netdev_rx_err ( netdev, iobuf, -ENODEV );
				return;
			}
		}
	}

	/* Hand off to network layer */
	netdev_rx ( netdev, iobuf );
}

/** ConnectX3 Ethernet device completion operations */
static struct ib_completion_queue_operations hermon_eth_cq_op = {
	.complete_send = hermon_eth_complete_send,
	.complete_recv = hermon_eth_complete_recv,
};

/**
 * Poll ConnectX3 Ethernet device
 *
 * @v netdev		Network device
 */
static void hermon_eth_poll ( struct net_device *netdev ) {
	struct hermon_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;

	ib_poll_eq ( ibdev );
}

/**
 * Open ConnectX3 Ethernet device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int hermon_eth_open ( struct net_device *netdev ) {
	struct hermon_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	union hermonprm_set_port set_port;
	int rc;

	/* Open hardware */
	if ( ( rc = hermon_open ( hermon ) ) != 0 )
		goto err_open;

	/* Allocate completion queue */
	port->eth_cq = ib_create_cq ( ibdev, HERMON_ETH_NUM_CQES,
				      &hermon_eth_cq_op );
	if ( ! port->eth_cq ) {
		printf ( "ConnectX3 %p port %d could not create completion "
		       "queue\n", hermon, ibdev->port );
		rc = -ENOMEM;
		goto err_create_cq;
	}

	hermon_arm_cq ( hermon, netdev );

	/* Allocate queue pair */
	port->eth_qp = ib_create_qp ( ibdev, IB_QPT_ETH,
				      HERMON_ETH_NUM_SEND_WQES, port->eth_cq,
				      HERMON_ETH_NUM_RECV_WQES, port->eth_cq,
				      &hermon_eth_qp_op );
	if ( ! port->eth_qp ) {
		printf ( "ConnectX3 %p port %d could not create queue "
		       "pair\n", hermon, ibdev->port );
		rc = -ENOMEM;
		goto err_create_qp;
	}
	ib_qp_set_ownerdata ( port->eth_qp, netdev );

	/* Activate queue pair */
	if ( ( rc = ib_modify_qp ( ibdev, port->eth_qp ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not modify queue "
		       "pair: %s\n", hermon, ibdev->port, strerror ( rc ) );
		goto err_modify_qp;
	}

	/* Add steering */
	if ( ( rc = hermon_eth_add_steer ( ibdev, port->eth_qp ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d failed to add steering:"
		       " %s\n", hermon, ibdev->port, strerror ( rc ) );
		goto err_add_steer;
	}

	/* Fill receive rings */
	ib_refill_recv ( ibdev, port->eth_qp );

	/* Set port general parameters */
	memset ( &set_port, 0, sizeof ( set_port ) );
	MLX_FILL_3 ( &set_port.general, 0,
		     v_mtu, 1,
		     v_pprx, 1,
		     v_pptx, 1 );
	MLX_FILL_1 ( &set_port.general, 1,
		     mtu, ( ETH_FRAME_LEN + 40 /* Used by card */ ) );
	MLX_FILL_1 ( &set_port.general, 2,
		     pptx, port->defaults.pptx );
	MLX_FILL_1 ( &set_port.general, 3,
		     pprx, port->defaults.pprx );
	if ( ( rc = hermon_cmd_set_port ( hermon,
					  ( HERMON_SET_PORT_GENERAL_PARAM |
					    ibdev->port ),
					  &set_port, 1 ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not set port general "
		       "parameters: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		goto err_set_port_general_params;
	}

	/* Set port mac table */
	memset ( &set_port, 0, sizeof ( set_port ) );
	MLX_FILL_3 ( set_port.mac_table, 0,
		     mac_vep, hermon_get_port_from_ibdev(ibdev)->vep_number,
		     v, 1,
		     mac_h, hermon_get_port_from_ibdev(ibdev)->eth_mac_h );
	MLX_FILL_1 ( set_port.mac_table, 1,
		     mac_l, hermon_get_port_from_ibdev(ibdev)->eth_mac_l );
	if ( ( rc = hermon_cmd_set_port ( hermon,
					  ( HERMON_SET_PORT_MAC_TABLE |
					    ibdev->port ),
					  &set_port, 1 ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not set port mac "
		       "table: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		goto err_set_port_mac_table;
	}

	/* Initialise port */
	if ( ( rc = hermon_cmd_init_port ( hermon, ibdev->port ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not initialise port: "
		       "%s\n", hermon, ibdev->port, strerror ( rc ) );
		goto err_init_port;
	}

	if (hermon->cap.ncsi_lag_mode) {
		sleep(65);
	}

	port->hermon_is_port_open = TRUE;

	return 0;

 err_init_port:
 err_set_port_general_params:
 err_set_port_mac_table:
 err_add_steer:
	hermon_eth_release_steer ( ibdev, port->eth_qp );
 err_modify_qp:
	ib_destroy_qp ( ibdev, port->eth_qp );
 err_create_qp:
	ib_destroy_cq ( ibdev, port->eth_cq );
 err_create_cq:
	hermon_close ( hermon );
 err_open:
	return rc;
}

/**
 * Close ConnectX3 Ethernet device
 *
 * @v netdev		Network device
 */
static void hermon_eth_close ( struct net_device *netdev ) {
	struct hermon_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;
	struct hermon *hermon = ib_get_drvdata ( ibdev );
	int rc;

	/* set portcopen to false - ignoring the NOP command interrupt (.irq) */
	port->hermon_is_port_open = FALSE;

	/* Close port */
	if ( ( rc = hermon_cmd_close_port ( hermon, ibdev->port ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not close port: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		/* Nothing we can do about this */
	}
	/* Release steering */
	hermon_eth_release_steer ( ibdev, port->eth_qp );
	/* Tear down the queues */
	ib_destroy_qp ( ibdev, port->eth_qp );
	port->eth_qp = NULL;
	ib_destroy_cq ( ibdev, port->eth_cq );
	port->eth_cq = NULL;

	/* Close hardware */
	hermon_close ( hermon );
}

#ifdef HERMON_IRQ_ENABLED
static void hermon_eth_irq ( struct net_device *netdev, int enable ) {
	struct hermon_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;
	struct hermon *hermon = ib_get_drvdata ( ibdev );

	if ( enable ) {
		if ( port->hermon_is_port_open ) {
			hermon_arm_eq ( hermon );
		} else {
			/* do nothing this is NOP cmd interrupt */
		}
	} else {
		hermon_clr_int ( hermon );
	}
}
#endif

/** ConnectX3 Ethernet network device operations */
static struct net_device_operations hermon_eth_operations = {
	.open		= hermon_eth_open,
	.close		= hermon_eth_close,
	.transmit	= hermon_eth_transmit,
	.poll		= hermon_eth_poll,
#ifdef HERMON_IRQ_ENABLED
	.irq		= hermon_eth_irq,
#endif
};

/**
 * Register ConnectX3 Ethernet device
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 * @ret rc		Return status code
 */
static int hermon_register_netdev ( struct hermon *hermon,
				    struct hermon_port *port ) {
	struct net_device *netdev;
	struct ib_device *ibdev = port->ibdev;
	struct hermonprm_query_port_cap query_port;
	union {
		uint8_t bytes[8];
		uint32_t dwords[2];
	} mac;
	int rc;

	/* Set protocol */
	ibdev->protocol = HERMON_PROT_ETH;

	/* Allocate network devices */
	netdev = alloc_etherdev ( 0 );
	if ( ! netdev ) {
		printf ( "ConnectX3 %p port %d could not allocate net device\n",
			hermon, ibdev->port );
		return -ENOMEM;
	}
	port->netdev = netdev;
	netdev_init ( netdev, &hermon_eth_operations );
	netdev->dev = ibdev->dev;
	netdev->priv = port;
	ib_set_ownerdata ( ibdev, netdev );

	/* Retrieve MAC address */
	if ( ( rc = hermon_cmd_query_port ( hermon, ibdev->port,
					    &query_port ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not query port: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		return rc;
	}
	mac.dwords[0] = htonl ( MLX_GET ( &query_port, mac_47_32 ) );
	mac.dwords[1] = htonl ( MLX_GET ( &query_port, mac_31_0 ) );
	memcpy ( netdev->hw_addr,
		 &mac.bytes[ sizeof ( mac.bytes ) - ETH_ALEN ], ETH_ALEN );

	port->eth_mac_l = MLX_GET ( &query_port, mac_31_0 );
	port->eth_mac_h = MLX_GET ( &query_port, mac_47_32 );

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not register network "
		       "device: %s\n", hermon, ibdev->port, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Handle ConnectX3 Ethernet device port state change
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 * @v link_up		Link is up
 */
static void hermon_state_change_netdev ( struct hermon *hermon __unused,
					 struct hermon_port *port,
					 int link_up ) {
	struct net_device *netdev = port->netdev;

	if ( link_up ) {
		netdev_link_up ( netdev );
	} else {
		netdev_link_down ( netdev );
	}
}

/**
 * Unregister ConnectX3 Ethernet device
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 */
static void hermon_unregister_netdev ( struct hermon *hermon __unused,
				       struct hermon_port *port ) {
	struct net_device *netdev = port->netdev;

	unregister_netdev ( netdev );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** ConnectX3 Ethernet port type */
static struct hermon_port_type hermon_port_type_eth = {
	.register_dev = hermon_register_netdev,
	.state_change = hermon_state_change_netdev,
	.unregister_dev = hermon_unregister_netdev,
};

/***************************************************************************
 *
 * Port type detection
 *
 ***************************************************************************
 */

/**
 * Name port type
 *
 * @v port_type		Port type
 * @v port_type_name	Port type name
 */
static inline const char * hermon_name_port_type ( unsigned int port_type ) {
	switch ( port_type ) {
	case HERMON_PORT_TYPE_UNKNOWN:	return "unknown";
	case HERMON_PORT_TYPE_IB:	return "Infiniband";
	case HERMON_PORT_TYPE_ETH:	return "Ethernet";
	default:			return "INVALID";
	}
}

/**
 * Sense port type
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 * @ret port_type	Port type, or negative error
 */
static int hermon_sense_port_type ( struct hermon *hermon,
				    struct hermon_port *port ) {
	struct ib_device *ibdev = port->ibdev;
	struct hermonprm_sense_port sense_port;
	int port_type;
	int rc;

	/* If DPDP is not supported, always assume Infiniband */
	if ( ! hermon->cap.dpdp ) {
		port_type = HERMON_PORT_TYPE_IB;
		DBGC ( hermon, "ConnectX3 %p port %d does not support DPDP; "
		       "assuming an %s network\n", hermon, ibdev->port,
		       hermon_name_port_type ( port_type ) );
		return HERMON_PORT_TYPE_UNKNOWN;
	}

	/* Sense the port type */
	if ( ( rc = hermon_cmd_sense_port ( hermon, ibdev->port,
					    &sense_port ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d sense failed: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		return HERMON_PORT_TYPE_UNKNOWN;
	}
	port_type = MLX_GET ( &sense_port, port_type );

	DBGC ( hermon, "ConnectX3 %p port %d sensed an %s network\n",
	       hermon, ibdev->port, hermon_name_port_type ( port_type ) );
	return port_type;
}

/**
 * Set port type
 *
 * @v hermon		ConnectX3 device
 * @v port		ConnectX3 port
 * @ret rc		Return status code
 */
static int hermon_set_port_type ( struct hermon *hermon,
				  struct hermon_port *port ) {
	struct ib_device *ibdev = port->ibdev;
	struct hermonprm_query_port_cap query_port;
    union mac_addr {
		struct {
			 unsigned long lowByte;
			 uint16_t highByte;
		};
		uint8_t mac_addr[ETH_ALEN];
    } mac_addr_aux;
	int ib_supported, eth_supported, port_type, rc;
	u8 is_connectx2 = 0;
	/* Check to see which types are supported */
	if ( ( rc = hermon_cmd_query_port ( hermon, ibdev->port,
					    &query_port ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not query port: %s\n",
		       hermon, ibdev->port, strerror ( rc ) );
		return rc;
	}

	/* read factory set MAC address */
	port->fact_mac_l = MLX_GET ( &query_port, fac_mac_31_0 );
	port->fact_mac_h = MLX_GET ( &query_port, fac_mac_47_32 );
    mac_addr_aux.lowByte = port->fact_mac_l;
    mac_addr_aux.highByte = port->fact_mac_h;
    port->port_nv_conf.phys_mac[0] = mac_addr_aux.mac_addr[5];
    port->port_nv_conf.phys_mac[1] = mac_addr_aux.mac_addr[4];
    port->port_nv_conf.phys_mac[2] = mac_addr_aux.mac_addr[3];
    port->port_nv_conf.phys_mac[3] = mac_addr_aux.mac_addr[2];
    port->port_nv_conf.phys_mac[4] = mac_addr_aux.mac_addr[1];
    port->port_nv_conf.phys_mac[5] = mac_addr_aux.mac_addr[0];

	ib_supported = MLX_GET ( &query_port, ib );
	eth_supported = MLX_GET ( &query_port, eth );
	DBGC ( hermon, "ConnectX3 %p port %d supports%s%s%s\n",
	       hermon, ibdev->port, ( ib_supported ? " Infiniband" : "" ),
	       ( ( ib_supported && eth_supported ) ? " and" : "" ),
	       ( eth_supported ? " Ethernet" : "" ) );
	DBGC ( hermon, "ConnectX3 %p port %d default_link_protocol %s\n",
			hermon, ibdev->port,
			( MLX_GET ( &query_port, default_link_type )
			? "Ethernet" : "Infiniband" ) );


	if ( eth_supported && ( ! ib_supported ) ) {
		port->type = &hermon_port_type_eth;
		DBGC ( hermon, "ConnectX3 %p port %d link"
		       " protocol %s\n", hermon, ibdev->port,
		       "Ethernet");
	} else if ( ib_supported && ( ! eth_supported ) ) {
		port->type = &hermon_port_type_ib;
		DBGC ( hermon, "ConnectX3 %p port %d link"
			" protocol %s\n", hermon, ibdev->port,
			"Infiniband");
	} else if ( ib_supported && eth_supported ) {
		is_connectx2 = ( cpu_to_be32 ( readl ( hermon->config + 0xf0014 ) )
				== 0x100b190 ? 1 : 0 );
		/* Sense network, if applicable */
		if ( is_connectx2 )
			mdelay ( 2000 );
		/* Try sensing port */
		port_type = hermon_sense_port_type ( hermon, port );
		if ( port_type == HERMON_PORT_TYPE_UNKNOWN ) {
			/* Check to see which types are supported */
			if ( ( rc = hermon_cmd_query_port ( hermon, ibdev->port,
						       &query_port ) ) != 0 ) {
				printf ( "ConnectX3 %p port %d could not "
				       " query port: %s\n", hermon,
				       ibdev->port, strerror ( rc ) );
				return rc;
			}
			DBGC ( hermon, "ConnectX3 %p port %d default_link"
			       " protocol %s\n", hermon, ibdev->port,
			       ( MLX_GET ( &query_port, default_link_type )
				 ? "Ethernet" : "Infiniband" ) );
			port_type = ( ( MLX_GET ( &query_port,
						  default_link_type ) )
				? HERMON_PORT_TYPE_ETH : HERMON_PORT_TYPE_IB );
		}

		/* Set port type based on sensed network, defaulting
		 * to Infiniband if nothing was sensed.
		 */
		switch ( port_type ) {
		case HERMON_PORT_TYPE_ETH:
			port->type = &hermon_port_type_eth;
			break;
		case HERMON_PORT_TYPE_IB:
		case HERMON_PORT_TYPE_UNKNOWN:
			port->type = &hermon_port_type_ib;
			break;
		default:
			return -EINVAL;
		}
	} else { /* eth_supported = 0 && ib_supported = 0*/
		printf ( "ConnectX3 %p port %d - Failed to detect link protocol\n",
				hermon, ibdev->port );
		return -EINVAL;
	}

	assert ( port->type != NULL );
	return 0;
}

/**
 * Get Virtual Ethernet Port number
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
int vep_get_function ( struct hermon *hermon, u8 vep_index,
		       int port, u8 *vep_function ) {
	struct hermonprm_mod_stat_cfg stat_cfg;
	int rc = 0;

	/* Query static configuration */
	if ( ( rc = hermon_mod_stat_cfg ( hermon, port,
				  HERMON_MOD_STAT_CFG_QUERY,
				  HERMON_MOD_STAT_CFG_OFFSET ( pf_net_boot_m ),
				  vep_index, HERMON_SETUP_MODE_VEP,
				  &stat_cfg ) ) != 0 ) {
		printf ( "ConnectX3 %p port %d could not query "
		       "configuration: %s\n", hermon, port, strerror ( rc ) );
		return rc;
	}

	*vep_function = MLX_GET ( &stat_cfg, funix );

	return 0;
}

/**
 * Get number of supported physical functions
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
int hermon_get_num_supported_veps ( struct hermon *hermon,
				    u8 supported_veps[] ) {
	unsigned int port;
	struct hermonprm_mod_stat_cfg stat_cfg;
	int rc = 0;

	for ( port = 1 ; port <= hermon->num_ports ; port++ ) {
		/* Query static configuration */
		if ( ( rc = hermon_mod_stat_cfg ( hermon, port,
				  HERMON_MOD_STAT_CFG_QUERY,
				  HERMON_MOD_STAT_CFG_OFFSET ( num_veps ),
				  0, HERMON_SETUP_MODE_PORT_SETUP,
				  &stat_cfg ) ) != 0 ) {
			printf ( "ConnectX3 %p port %d could not query "
			       "configuration: %s\n", hermon, port, strerror ( rc ) );
			return rc;
		}


		supported_veps[port - 1] = MLX_GET ( &stat_cfg, num_veps );
		DBGC ( hermon, "ConnectX3 %p port %d number supported VEPs %d\n",
		       hermon, port, hermon->num_veps[port - 1] );
	}

	return 0;
}

/**
 * Get supported number of ports
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
static int hermon_get_num_ports ( struct hermon *hermon ) {
	struct hermonprm_mod_stat_cfg stat_cfg;
	int rc = 0;

	/* Query static configuration */
	if ( ( rc = hermon_mod_stat_cfg ( hermon, 0,
					  HERMON_MOD_STAT_CFG_QUERY,
					  HERMON_MOD_STAT_CFG_OFFSET ( num_port ),
					  hermon->physical_function,
					  HERMON_SETUP_MODE_LANE_SETUP,
				  &stat_cfg ) ) != 0 ) {
		printf ( "%s could not query configuration: %d\n", __FUNCTION__, rc );
		return rc;
	}

	hermon->num_ports = MLX_GET ( &stat_cfg, num_port );

	/* Sanity check */
	if ( hermon->num_ports > HERMON_MAX_PORTS ) {
		DBGC ( hermon, "ConnectX3 %p has %d ports (only %d supported)\n",
		       hermon, hermon->num_ports, HERMON_MAX_PORTS );
		hermon->num_ports = HERMON_MAX_PORTS;
	}

	DBGC ( hermon, "ConnectX3 %p supported number of ports %d\n",
	       hermon, hermon->num_ports );

	return 0;
}


/**
 * Set port masking
 *
 * @v hermon		ConnectX3 device
 * @ret rc		Return status code
 */
int set_port_masking ( struct hermon *hermon ) {
	int rc = 0;
	u8 vep = 0;
	unsigned int i;
	u8 boot_enable = 0;
	u8 supported_veps[HERMON_MAX_PORTS];

	/* Get number of supported ports */
	if ( ( rc = hermon_get_num_ports ( hermon ) ) != 0 )
		return rc;

	/* Get number of physical functions */
	if ( ( rc = hermon_get_num_supported_veps
				( hermon, supported_veps ) ) != 0 )
		return rc;

	/* Check CLP configuration */
	hermon->port_mask = 0;
	for ( i = 0; i < hermon->num_ports ; i++ ) {
		hermon->num_veps[i] = 0;
		for ( vep = 0; vep < supported_veps[i]; vep ++ ) {
			/* If we are in POST as a result of pressed CTRL-B - then enable all ports */
			if ( boot_post_shell ) {
				boot_enable = 1;
			} else {
				if ( ( rc = driver_settings_get_nv_boot_en ( hermon,
						i + 1, &boot_enable, hermon_flash_read_tlv_wrapper,
						& ( hermon->port[i].defaults ) ) ) != 0 ) {
					return rc;
				}
			}
			if ( boot_enable ) {
			       hermon->port_mask |= (i + 1);
			       hermon->vep_table[i][hermon->num_veps[i]] = vep;
			       hermon->num_veps[i] ++;
			}
		}
	}

	if ( ! hermon->port_mask ) {
		/* No port was enabled */
		DBGC ( hermon, "ConnectX3 %p No port was enabled for "
		       "booting\n", hermon );
		return -ENETUNREACH;
	}

	return 0;
}

#define SEGMENT_SIZE		8
#define MAX_SEGMENTS_NUM	32
#define EVENT_LENGTH_BIT	8
#define SEGMENT_DATA_LENGTH_BIT	16
#define SEGMENT_DATA_OFFSET_BIT	24
static int hermon_set_ocbb_event_value ( struct hermon *hermon,
				 uint32_t event_id, uint8_t val ) {
	uint64_t in = val;
	uint32_t in_mod = 0;
	int rc;

	in = cpu_to_be64 ( in << 32 );
	in_mod = event_id;
	in_mod |= ( 1 << EVENT_LENGTH_BIT );
	in_mod |= ( 1 << SEGMENT_DATA_LENGTH_BIT );
	in_mod |= ( 0 << SEGMENT_DATA_OFFSET_BIT );

	if ( ( rc = hermon_cmd_init_diagnostic_buffer ( hermon, in_mod, &in ) ) ) {
		DBGC ( hermon, "Failed to send OCBB segment to FW (rc = %d)\n", rc );
		return rc;
	}

	return 0;
}

static void hermon_updater ( void *priv, uint8_t status ) {
	struct hermon *hermon = (struct hermon *) priv;

	switch ( status ) {
		case STATUS_UPDATE_WAIT_ON_LINKUP:
			hermon_set_ocbb_event_value ( hermon, 36, 1 );
			break;
		case STATUS_UPDATE_URI_BOOT:
			hermon_set_ocbb_event_value ( hermon, 36, 4 );
			break;
		default:
			DBGC ( hermon, "%s: Unknown status %d\n", __FUNCTION__, status );
	}
};

/***************************************************************************
 *
 * PCI interface
 *
 ***************************************************************************
 */

/**
 * Allocate ConnectX3 device
 *
 * @v pci		PCI device
 * @v id		PCI ID
 * @ret rc		Return status code
 */
static struct hermon * hermon_alloc ( void ) {
	struct hermon *hermon;

	/* Allocate ConnectX3 device */
	hermon = zalloc ( sizeof ( *hermon ) );
	if ( ! hermon )
		goto err_hermon;

	/* Allocate space for mailboxes */
	hermon->mailbox_in = malloc_dma ( HERMON_MBOX_SIZE,
					  HERMON_MBOX_ALIGN );
	if ( ! hermon->mailbox_in )
		goto err_mailbox_in;
	hermon->mailbox_out = malloc_dma ( HERMON_MBOX_SIZE,
					   HERMON_MBOX_ALIGN );
	if ( ! hermon->mailbox_out )
		goto err_mailbox_out;

	return hermon;

	free_dma ( hermon->mailbox_out, HERMON_MBOX_SIZE );
 err_mailbox_out:
	free_dma ( hermon->mailbox_in, HERMON_MBOX_SIZE );
 err_mailbox_in:
	free ( hermon );
 err_hermon:
	return NULL;
}

/**
 * Free ConnectX3 device
 *
 * @v hermon		ConnectX3 device
 */
static void hermon_free_aux ( struct hermon *hermon ) {
	if ( hermon->icm ) {
		ufree ( hermon->icm );
		hermon->icm = UNULL;
	}

	if ( hermon->firmware_area ) {
		ufree ( hermon->firmware_area );
		hermon->firmware_area = UNULL;
	}

	if ( hermon->mailbox_out ) {
		free_dma ( hermon->mailbox_out, HERMON_MBOX_SIZE );
		hermon->mailbox_out = NULL;
	}

	if ( hermon->mailbox_in ) {
		free_dma ( hermon->mailbox_in, HERMON_MBOX_SIZE );
		hermon->mailbox_in = NULL;
	}
}

static void hermon_free ( struct hermon *hermon ) {
	hermon_free_aux ( hermon );
	free ( hermon );
}

/**
 * Initialise ConnectX3 PCI parameters
 *
 * @v hermon		ConnectX3 device
 */
static void hermon_pci_init ( struct hermon *hermon ) {
	struct pci_device *pci = hermon->pci;

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Get PCI BARs */
	hermon->config = ioremap ( pci_bar_start ( pci, HERMON_PCI_CONFIG_BAR),
				   HERMON_PCI_CONFIG_BAR_SIZE );
	hermon->uar = ioremap ( pci_bar_start ( pci, HERMON_PCI_UAR_BAR ),
				( HERMON_UAR_NON_EQ_PAGE + 1 ) * HERMON_PAGE_SIZE );
	hermon->physical_function = pci->busdevfn & 0x7;
	scratchpad_addr			= hermon->config + SCRATCHPAD_WORDS_START;
}

#define STORE_INT_SETTING( is_enabled, settings, setting )			\
	do {									\
		if ( ( rc = storen_setting ( settings, setting, is_enabled ) ) )\
			DBGC ( hermon, "Failed to set %s with value %d\n",	\
				( setting )->name, is_enabled );		\
		else								\
			DBGC ( hermon, "Successfully stored %s with value %d\n",\
				( setting )->name, is_enabled );		\
	} while ( 0 )

static int hermon_probe_no_nodnic ( struct pci_device *pci, struct hermon *hermon ) {
	struct ib_device *ibdev;
	struct net_device *vlan;
	struct hermon_port *port;
	unsigned int vep;
	unsigned int i;
	int rc;

	/* Start firmware */
	if ( ( rc = hermon_start_firmware ( hermon ) ) != 0 )
		goto err_start_firmware;

	/* Get device limits */
	if ( ( rc = hermon_get_cap ( hermon ) ) != 0 )
		goto err_get_cap;

	/* Get device ini and default configurations */
	if ( ( rc = hermon_get_ini_and_defaults ( hermon ) ) != 0 )
		goto err_get_ini;

	/* Set port mask */
	if ( ( rc = set_port_masking ( hermon ) ) != 0 ) {
		goto err_set_masking;
	}

	/* Allocate Infiniband devices */
	for ( i = 0 ; i < hermon->num_ports ; i++ ) {
		for ( vep = 0; vep < hermon->num_veps[i]; vep++ ) {
			if ( ! ( hermon->port_mask &
						   ( i + HERMON_PORT_BASE ) ) )
				continue;
			ibdev = alloc_ibdev ( 0 );
			if ( ! ibdev ) {
				rc = -ENOMEM;
				goto err_alloc_ibdev;
			}
			hermon->port[i].ibdev = ibdev;
			hermon->port[i].vep_number = hermon->vep_table[i][vep];
			ibdev->op = &hermon_ib_operations;
			ibdev->dev = &pci->dev;
			ibdev->port = ( HERMON_PORT_BASE + i );
			ib_set_drvdata ( ibdev, hermon );
		}
	}

	/* Start device */
	if ( ( rc = hermon_start ( hermon, 1 ) ) != 0 )
		goto err_start;

	/* Determine port types */
	for ( i = 0 ; i < hermon->num_ports ; i++ ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		port = &hermon->port[i];
		if ( ( rc = hermon_set_port_type ( hermon, port ) ) != 0 )
			goto err_set_port_type;
	}

	/* Register device settings */
	hermon_init_settings ( hermon );

	/* Register devices and their settings */
	for ( i = 0 ; i < hermon->num_ports ; i++ ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		port = &hermon->port[i];

		/* We need to initialize the MAC admin bit and the g_fact_mac_addr
		 * before registering IPoIB device due to the fact that IPoIB probe
		 * will use these values
		 */
		if ( port->type == &hermon_port_type_ib ) {
			driver_settings_get_nv_ib_mac_admin_bit( hermon,
						i + 1, hermon_flash_read_tlv_wrapper,
						& ( port->defaults ), &g_mac_admin_bit );
			memcpy ( g_fact_mac_addr, port->port_nv_conf.phys_mac, ETH_ALEN );
		}

		if ( ( rc = port->type->register_dev ( hermon, port ) ) != 0 )
			goto err_register;

		/* Get NvMem port configuration & apply them */
		/* Initialize port settings */
		hermon_init_port_settings ( hermon, port );
		/* TODO: check return value and exit on error */
		driver_settings_get_port_nvdata ( & ( port->driver_settings ),  i + 1 );
		driver_register_port_nv_settings ( & ( port->driver_settings ) );

		STORE_INT_SETTING ( hermon->ini_configurations.uri_boot_retry_delay,
				netdev_settings ( port->netdev ), &uriboot_retry_delay_setting );
		STORE_INT_SETTING ( hermon->ini_configurations.uri_boot_retry,
				netdev_settings ( port->netdev ), &uriboot_retry_setting );
		STORE_INT_SETTING ( hermon->ini_configurations.promiscuous_vlan,
				netdev_settings ( port->netdev ), &promisc_vlan_setting );
		STORE_INT_SETTING ( NETWORK_WAIT_TIMEOUT,
				netdev_settings ( port->netdev ), &network_wait_to_setting );

		/* Only IPv4 is supported in Infiniband */
		if ( port->type == &hermon_port_type_ib ) {
			STORE_INT_SETTING ( 1, netdev_settings ( port->netdev ),
					&dhcpv6_disabled_setting );
			storef_setting ( netdev_settings ( port->netdev ),
					&dhcpv4_disabled_setting, NULL );
		}
	}

	driver_settings_get_nvdata ( & ( hermon->hermon_settings ) );
	driver_register_nv_settings ( & ( hermon->hermon_settings ) );

	if ( ! boot_post_shell ) {
		/* Create VLAN devices if needed */
		for ( i = 0 ; i < hermon->num_ports ; i++ ) {
			if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
				continue;
			port = &hermon->port[i];
			if ( ! port->port_nv_conf.nic.boot_conf.en_vlan )
				continue;
			if ( ( rc = vlan_create ( port->netdev, port->port_nv_conf.nic.boot_conf.vlan_id, 1 ) ) )
				DBGC ( hermon, "Failed to create VLAN device on port %d (rc = %d)\n", rc, i + 1 );
			else if ( ( vlan = vlan_find( port->netdev, port->port_nv_conf.nic.boot_conf.vlan_id ) ) )
				move_trunk_settings_to_vlan ( port->netdev, vlan );
			else
				DBGC ( hermon, "Failed to find the VLAN device (rc = %d)\n", rc );
		}

		/* Register hermon to the status update list */
		if ( ( rc = status_update_register_device ( hermon, hermon_updater ) ) )
			DBGC ( hermon, "Failed to register to the status updaters list (rc = %d)\n", rc );
	}

	/* Leave device quiescent until opened */
	if ( hermon->open_count == 0 ) {
		hermon_stop ( hermon );
	} else {
		printf("hermon->open_count != 0!!!! not stopping...\n");
		assert ( 0 );
	}

	return 0;

	i = hermon->num_ports;
 err_register:
	for ( i-- ; ( signed int ) i >= 0 ; i-- ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		port = &hermon->port[i];
		port->type->unregister_dev ( hermon, port );
	}
 err_set_port_type:
	hermon_stop ( hermon );
 err_start:
	i = hermon->num_ports;
 err_alloc_ibdev:
	for ( i-- ; ( signed int ) i >= 0 ; i-- ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		ibdev_put ( hermon->port[i].ibdev );
	}
 err_set_masking:
 err_get_ini:
 err_get_cap:
	hermon_stop_firmware ( hermon );
 err_start_firmware:
	return rc;
}

static void hermon_remove_no_nodnic ( struct pci_device *pci ) {
	struct hermon *hermon = pci_get_drvdata ( pci );
	struct hermon_port *port;
	int i;

	/* Deallocate all driver settings */
	destroy_driver_settings ();

	if ( ! boot_post_shell ) {
		/* Unregister hermon from the status update list */
		status_update_unregister_device ( hermon );
	}

	for ( i = ( hermon->num_ports - 1 ) ; i >= 0 ; i-- ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		port = &hermon->port[i];
		port->type->unregister_dev ( hermon, port );
	}
	for ( i = ( hermon->num_ports - 1 ) ; i >= 0 ; i-- ) {
		if ( ! ( hermon->port_mask & ( i + HERMON_PORT_BASE ) ) )
			continue;
		ibdev_put ( hermon->port[i].ibdev );
	}
	hermon_free ( hermon );
}

/***************************************************************************
 * NODNIC operations
 **************************************************************************/
static int hermon_nodnic_supported = 0;

static int hermon_read_flash_for_nodnic ( struct hermon *hermon ) {
	struct driver_settings *driver_settings;
	struct hermon_port *port;
	struct hermonprm_query_port_cap query_port;
	int rc;
	unsigned int i;

	if ( ! hermon ) {
		return -ENODEV;
	}

	/* Start firmware */
	if ( ( rc = hermon_start_firmware ( hermon ) ) != 0 )
		goto err_start_firmware;

	/* Get device limits */
	if ( ( rc = hermon_get_cap ( hermon ) ) != 0 )
		goto err_get_cap;

	/* Get device ini and default configurations */
	if ( ( rc = hermon_get_ini_and_defaults ( hermon ) ) != 0 )
		goto err_get_ini;

	driver_settings 							= & hermon->hermon_settings;
	driver_settings->callbacks.tlv_read			= hermon_flash_read_tlv_wrapper;
	driver_settings->drv_priv					= hermon;
	driver_settings->priv_data					= & ( hermon->hermon_nv_conf );
	driver_settings->defaults					= & ( hermon->defaults );
	driver_settings_get_nvdata ( driver_settings );

	for ( i = 0; i < hermon->cap.num_ports; i++ ) {
		port										= & hermon->port[i];
		driver_settings								= & port->driver_settings;
		driver_settings->callbacks.tlv_read			= hermon_flash_read_tlv_wrapper;
		driver_settings->drv_priv					= hermon;
		driver_settings->priv_data					= & ( port->port_nv_conf );
		driver_settings->defaults					= & ( port->defaults );
		driver_settings_get_port_nvdata ( driver_settings, i + 1 );

		if ( ( rc = hermon_cmd_query_port ( hermon, i + 1, &query_port ) ) != 0 ) {
			printf ( "%s: port %d could not query port: %s\n", __FUNCTION__,
			       i + 1, strerror ( rc ) );
		} else {
			flexboot_nodnic_copy_mac ( port->port_nv_conf.phys_mac,
					MLX_GET ( &query_port, fac_mac_31_0 ),
					MLX_GET ( &query_port, fac_mac_47_32 ) );
		}
	}

	DBGC ( hermon, "NODNIC settings read from flash\n" );
	rc = 0;
err_get_ini:
err_get_cap:
	hermon_stop_firmware ( hermon );
err_start_firmware:
	hermon_reset ( hermon, 0xffffffff );
	return rc;
}

static int hermon_nodnic_is_supported ( struct pci_device *pci, struct hermon *hermon ) {
	struct hermonprm_sense_port sense_port;
	int rc, nodnic_supported = 1, disconnected, port_type;
	uint32_t port_index, num_try;

	if ( ( rc = hermon_get_num_ports ( hermon ) ) ) {
		rc = -ENOTSUP;
		goto err_get_num_ports;
	}

	/*
	 * NODNIC driver supports only Ethernet protocol, thus we need to know
	 * if the ports are Ethernet ports or not.
	 */
#define TRIES 0x3
#define PORT_DISCONNECTED 0x4
#define ETH_TYPE 0x2
	for ( port_index = 0; port_index < hermon->num_ports; port_index++ ) {
		port_type = 0;
		disconnected = 0;
		for ( num_try = 0; num_try < TRIES; num_try++ ) {
			if ( ( rc = hermon_cmd_sense_port ( hermon, port_index + 1,
					&sense_port ) ) != 0 ) {
					DBGC ( hermon, "Port %d sense failed: %s\n",
							port_index + 1, strerror ( rc ) );
					if ( rc == PORT_DISCONNECTED ) {
						disconnected = 1;
					}
					break;
			} else {
				port_type = MLX_GET ( &sense_port, port_type );
				if ( port_type != 0 ) {
					break;
				}
			}
			sleep(3);
		}
		if ( ( port_type & ETH_TYPE ) == 0 && !disconnected ) {
				nodnic_supported = 0;
		}
	}

	if ( !nodnic_supported ) {
			rc = -ENOTSUP;
			goto err_bad_type;
	}

	/* Check if NODNIC interface is supported */
	if ( flexboot_nodnic_is_supported ( pci ) == 0 ) {
		rc = -ENOTSUP;
	} else {
		DBGC ( hermon, "NODNIC is supported\n" );
		hermon_read_flash_for_nodnic ( hermon );
		rc = 0;
	}

err_bad_type:
err_get_num_ports:
	if ( rc ) {
		DBGC ( hermon, "NODNIC is not supported\n" );
	}
	return ( rc == 0 );
}

static mlx_status hermon_nodnic_fill_eth_send_wqe ( struct ib_device *ibdev,
			   struct ib_queue_pair *qp, struct ib_address_vector *av __unused,
			   struct io_buffer *iobuf, struct nodnic_send_wqbb *wqbb,
			   unsigned long wqe_idx ) {
	mlx_status status = MLX_SUCCESS;
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct hermonprm_eth_send_wqe *eth_wqe =  NULL;
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp =
			ib_qp_get_drvdata ( qp );
	nodnic_qp *nodnic_qp = flexboot_nodnic_qp->nodnic_queue_pair;
	struct nodnic_send_ring *send_ring = &nodnic_qp->send;
	mlx_uint32 qpn = 0;

	eth_wqe = (struct hermonprm_eth_send_wqe *)wqbb;
	memset ( ( ( ( void * ) eth_wqe ) + 4  ), 0,
			   ( sizeof ( *eth_wqe ) - 4 ) );

	status = nodnic_port_get_qpn ( &port->port_priv, &send_ring->nodnic_ring, &qpn);
	if ( status != MLX_SUCCESS ) {
		printf("nodnic_port_get_qpn failed\n");
		goto err;
	}

	MLX_FILL_2 ( &eth_wqe->ctrl, 0,
			     opcode, 0xa,
			     owner,
				 ( ( wqe_idx & send_ring->nodnic_ring.num_wqes ) ? 1 : 0 ) );
	MLX_FILL_1 ( &eth_wqe->ctrl, 1, ds,
		     0x2 );
	MLX_FILL_2 ( &eth_wqe->ctrl, 2,
		     c, 0x03 /* generate completion */,
		     s, 1 /* inhibit ICRC */ );
	MLX_FILL_1 ( &eth_wqe->data[0], 0,
		     byte_count, iob_len ( iobuf ) );
	MLX_FILL_1 ( &eth_wqe->data[0], 1, l_key, flexboot_nodnic->device_priv.lkey );
	MLX_FILL_H ( &eth_wqe->data[0], 2,
		     local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &eth_wqe->data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );
err:
	return status;
}

static mlx_status hermon_nodnic_fill_completion ( void *cqe,
		struct cqe_data *cqe_data ) {
	union hermonprm_completion_entry *cq_entry;
	uint32_t opcode;
	cq_entry = (union hermonprm_completion_entry *)cqe;
	cqe_data->owner = MLX_GET ( &cq_entry->normal, owner );
	cqe_data->qpn = MLX_GET ( &cq_entry->normal, qpn );
	cqe_data->is_send = MLX_GET ( &cq_entry->normal, s_r );
	opcode = MLX_GET ( &cq_entry->normal, opcode );
#define HERMON_OPCODE_RECV_ERROR	0xfe
#define HERMON_OPCODE_SEND_ERROR	0xff
	cqe_data->is_error =
			( opcode >= HERMON_OPCODE_RECV_ERROR);
	if ( cqe_data->is_error ) {
		cqe_data->syndrome = MLX_GET ( &cq_entry->error, syndrome );
		cqe_data->vendor_err_syndrome =
				MLX_GET ( &cq_entry->error, vendor_error_syndrome );
		cqe_data->is_send =
					(opcode == HERMON_OPCODE_SEND_ERROR);
	} else {
		cqe_data->wqe_counter = MLX_GET ( &cq_entry->normal, wqe_counter );
		cqe_data->byte_cnt = MLX_GET ( &cq_entry->normal, byte_cnt );

	}
	return 0;
}

static mlx_status hermon_nodnic_cqe_set_owner ( void *cq, unsigned int num_cqes ) {
	unsigned int i = 0;
	union hermonprm_completion_entry *cq_list;

	cq_list = (union hermonprm_completion_entry *)cq;
	for ( ; i < num_cqes ; i++ )
		MLX_FILL_1 ( &cq_list[i].normal, 7, owner, 1 );
	return 0;
}

static mlx_size hermon_nodnic_get_cqe_size ( void ) {
	return sizeof ( union hermonprm_completion_entry );
}

static void hermon_nodnic_get_settings ( struct flexboot_nodnic *priv, void *drv_data ) {
	struct hermon *hermon = ( struct hermon * ) drv_data;
	int i = 0;

	if ( ! priv || ! hermon ) {
		printf ( "%s: Bad parameter\n", __FUNCTION__ );
		return;
	}

	memcpy ( & priv->nodnic_nv_conf, & hermon->hermon_nv_conf,
			sizeof ( priv->nodnic_nv_conf ) );
	memcpy ( & priv->ini_configurations, & hermon->ini_configurations,
			sizeof ( priv->ini_configurations ) );
	memcpy ( & priv->defaults, & hermon->defaults,
			sizeof ( priv->defaults ) );

	for ( i = 0; i < hermon->cap.num_ports; i++ ) {
		memcpy ( & priv->port[i].port_nv_conf, & hermon->port[i].port_nv_conf,
				sizeof ( priv->port[i].port_nv_conf ) );
		memcpy ( & priv->port[i].defaults, & hermon->port[i].defaults,
				sizeof ( priv->port[i].defaults ) );
	}
}

static struct flexboot_nodnic_callbacks hermon_nodnic_callbacks = {
	.get_cqe_size = hermon_nodnic_get_cqe_size,
	.fill_send_wqe[IB_QPT_ETH] = hermon_nodnic_fill_eth_send_wqe,
	.fill_completion = hermon_nodnic_fill_completion,
	.cqe_set_owner = hermon_nodnic_cqe_set_owner,
	.get_settings = hermon_nodnic_get_settings,
	.update_settings_ops = hermon_update_setting_ops,
#ifdef NODNIC_IRQ_ENABLED
	.irq = flexboot_nodnic_eth_irq,
#endif
};

/**************************************************************************/
/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @v id		PCI ID
 * @ret rc		Return status code
 */
static int hermon_probe ( struct pci_device *pci ) {
	struct hermon *hermon = NULL;
	int rc;

	DBG ( "%s: start\n", __FUNCTION__ );

	if ( ! pci ) {
		printf ( "%s: PCI is NULL\n", __FUNCTION__ );
		rc = -EINVAL;
		goto probe_done;
	}

	hermon = hermon_alloc();
	if ( ! hermon ) {
		rc = -ENOMEM;
		goto probe_done;
	}

	pci_set_drvdata ( pci, hermon );
	hermon->pci = pci;
	hermon_pci_init ( hermon );

	hermon_reset ( hermon, HERMON_RESET_START );

	/* Use the regular driver for boot menu configuration and if
	 * NODNIC interface is not supported */
	if ( ! boot_post_shell )
		hermon_nodnic_supported = hermon_nodnic_is_supported ( pci, hermon );

	if ( boot_post_shell || ( ! hermon_nodnic_supported ) ) {
		DBG ( "%s: Using no NODNIC driver\n", __FUNCTION__ );
		rc = hermon_probe_no_nodnic ( pci, hermon );
		goto probe_done;
	}

	DBG ( "%s: Using NODNIC driver\n", __FUNCTION__ );

	rc = flexboot_nodnic_probe ( pci, &hermon_nodnic_callbacks, hermon );

probe_done:
	DBG ( "%s: rc = %d\n", __FUNCTION__, rc );
	if ( rc || hermon_nodnic_supported )
		hermon_free ( hermon );
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void hermon_remove ( struct pci_device *pci ) {
	DBG ( "%s: start\n", __FUNCTION__ );

	if ( boot_post_shell || ( ! hermon_nodnic_supported ) ) {
		DBG ( "%s: Using no NODNIC driver remove\n", __FUNCTION__ );
		hermon_remove_no_nodnic ( pci );
		return;
	}

	DBG ( "%s: Using NODNIC driver remove\n", __FUNCTION__ );

	flexboot_nodnic_remove ( pci );

	DBG ( "%s: end\n", __FUNCTION__ );
}

static struct pci_device_id hermon_nics[] = {
	PCI_ROM ( 0x15b3, 0x1003, "ConnectX-3", "ConnectX-3 HCA driver, DevID 4099", 0 ),
	PCI_ROM ( 0x15b3, 0x1007, "ConnectX-3Pro", "ConnectX-3Pro HCA driver, DevID 4103", 0 ),
};

struct pci_driver hermon_driver __pci_driver = {
	.ids = hermon_nics,
	.id_count = ( sizeof ( hermon_nics ) / sizeof ( hermon_nics[0] ) ),
	.probe = hermon_probe,
	.remove = hermon_remove,
};
