/*
 * Copyright (C) 2013-2015 Mellanox Technologies Ltd.
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

#include <errno.h>
#include <strings.h>
#include <ipxe/malloc.h>
#include <ipxe/umalloc.h>
#include <ipxe/infiniband.h>
#include <ipxe/ib_smc.h>
#include <ipxe/iobuf.h>
#include <ipxe/netdevice.h>
#include "flexboot_nodnic.h"
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <usr/ifmgmt.h>
#include <ipxe/boot_menu_ui.h>
#include <ipxe/in.h>
#include <byteswap.h>
#include <mlx_nvconfig.h>
#include <mlx_pci_gw.h>
#include <config/general.h>
#include "mlx_port.h"
#include "prm/nodnic_shomron_prm.h"
#include "golan.h"
#include "golan_settings.h"
#include "mlx_bail.h"


const char *golan_qp_state_as_string[] = {
	"RESET",
	"INIT",
	"RTR",
	"RTS",
	"SQD",
	"SQE",
	"ERR"
};

inline int golan_check_rc_and_cmd_status ( struct golan_cmd_layout *cmd, int rc ) {
	struct golan_outbox_hdr *out_hdr = ( struct golan_outbox_hdr * ) ( cmd->out );
	if ( rc == -EBUSY ) {
		printf ( "HCA is busy (rc = -EBUSY)\n" );
		return rc;
	} else if ( out_hdr->status ) {
		printf("%s status = 0x%x - syndrom = 0x%x\n", __FUNCTION__,
				out_hdr->status, be32_to_cpu(out_hdr->syndrome));
		return out_hdr->status;
	}
	return 0;
}

#define GOLAN_CHECK_RC_AND_CMD_STATUS(_lable)							\
		do {															\
			if ( ( rc = golan_check_rc_and_cmd_status ( cmd, rc ) ) )	\
				goto _lable;											\
		} while (0)

#define GOLAN_PRINT_RC_AND_CMD_STATUS	golan_check_rc_and_cmd_status ( cmd, rc )


struct mbox {
	union {
		struct golan_cmd_prot_block	mblock;
		u8	data[MAILBOX_STRIDE];
		__be64	qdata[MAILBOX_STRIDE >> 3];
	};
};

static inline uint32_t  ilog2(uint32_t mem)
{
	uint32_t reg = 0xffff;
	asm volatile ("bsr %1, %0" : "=r" (reg) : "m" (mem));
	return reg;
}

#define	CTRL_SIG_SZ	(sizeof(mailbox->mblock) - sizeof(mailbox->mblock.bdata) - 2)

static inline u8 xor8_buf(void *buf, int len)
{
	u8 sum = 0;
	int i;
	u8 *ptr = buf;

	for (i = 0; i < len; ++i)
		sum ^= ptr[i];

	return sum;
}

static inline int verify_block_sig(struct golan_cmd_prot_block *block)
{
	if (xor8_buf(block->rsvd0, sizeof(*block) - sizeof(block->data) - 1) != 0xff)
		return -EINVAL;

	if (xor8_buf(block, sizeof(*block)) != 0xff)
		return -EINVAL;
	return 0;
}

static inline const char *cmd_status_str(u8 status)
{
	switch (status) {
		case 0x0:	return "OK";
		case 0x1:	return "internal error";
		case 0x2:	return "bad operation";
		case 0x3:	return "bad parameter";
		case 0x4:	return "bad system state";
		case 0x5:	return "bad resource";
		case 0x6:	return "resource busy";
		case 0x8:	return "limits exceeded";
		case 0x9:	return "bad resource state";
		case 0xa:	return "bad index";
		case 0xf:	return "no resources";
		case 0x50:	return "bad input length";
		case 0x51:	return "bad output length";
		case 0x10:	return "bad QP state";
		case 0x30:	return "bad packet (discarded)";
		case 0x40:	return "bad size too many outstanding CQEs";
		case 0xff:	return "Command Timed Out";
		default:	return "unknown status";
	}
}

static inline uint16_t fw_rev_maj(struct golan *golan)
{
	return be32_to_cpu(readl(&golan->iseg->fw_rev)) & 0xffff;
}

static inline u16 fw_rev_min(struct golan *golan)
{
	return be32_to_cpu(readl(&golan->iseg->fw_rev)) >> 16;
}

static inline u16 fw_rev_sub(struct golan *golan)
{
	return be32_to_cpu(readl(&golan->iseg->cmdif_rev_fw_sub)) & 0xffff;
}

static inline u16 cmdif_rev(struct golan *golan)
{
	return be32_to_cpu(readl(&golan->iseg->cmdif_rev_fw_sub)) >> 16;
}


static inline struct golan_cmd_layout *get_cmd( struct golan *golan, int idx )
{
	return golan->cmd.addr + (idx << golan->cmd.log_stride);
}

static inline void golan_calc_sig(struct golan *golan, uint32_t cmd_idx,
				uint32_t inbox_idx, uint32_t outbox_idx)
{
	struct golan_cmd_layout *cmd	= get_cmd(golan, cmd_idx);
	struct mbox *mailbox = NULL;

	if (inbox_idx != NO_MBOX) {
		mailbox				= GET_INBOX(golan, inbox_idx);
		mailbox->mblock.token		= cmd->token;
		mailbox->mblock.ctrl_sig	= ~xor8_buf(mailbox->mblock.rsvd0,
								CTRL_SIG_SZ);
	}
	if (outbox_idx != NO_MBOX) {
		mailbox				= GET_OUTBOX(golan, outbox_idx);
		mailbox->mblock.token		= cmd->token;
		mailbox->mblock.ctrl_sig	= ~xor8_buf(mailbox->mblock.rsvd0,
								CTRL_SIG_SZ);
	}
	cmd->sig = ~xor8_buf(cmd, sizeof(*cmd));
}

/**
  * Get Golan FW
  */
static int fw_ver_and_cmdif ( struct golan *golan ) {
	DBGC (golan ,"\n[%x:%x]rev maj.min.submin = %x.%x.%x cmdif = %x\n",
		golan->iseg->fw_rev,
		golan->iseg->cmdif_rev_fw_sub,
		fw_rev_maj ( golan ), fw_rev_min ( golan ),
		fw_rev_sub ( golan ), cmdif_rev ( golan));

	if (cmdif_rev ( golan) != PXE_CMDIF_REF) {
		printf ("CMDIF %d not supported current is %d\n",
			cmdif_rev ( golan), PXE_CMDIF_REF);
		return 1;
	}
	return 0;
}

static inline void show_out_status(uint32_t *out)
{
	printf("%x\n", be32_to_cpu(out[0]));
	printf("%x\n", be32_to_cpu(out[1]));
	printf("%x\n", be32_to_cpu(out[2]));
	printf("%x\n", be32_to_cpu(out[3]));
}
/**
  * Check if CMD has finished.
  */
static inline uint32_t is_command_finished( struct golan *golan, int idx)
{
	wmb();
	return !(get_cmd( golan , idx )->status_own & CMD_OWNER_HW);
}

/**
 * Wait for Golan command completion
 *
 * @v golan		Golan device
 * @ret rc		Return status code
 */
static inline int golan_cmd_wait(struct golan *golan, int idx, const char *command)
{
	unsigned int wait;
	int	rc = -EBUSY;

	for ( wait = GOLAN_HCR_MAX_WAIT_MS ; wait ; --wait ) {
		if (is_command_finished(golan, idx)) {
			rc = CMD_STATUS(golan, idx);
			rmb();
			break;
		} else {
			mdelay ( 1 );
		}
	}
	if (rc) {
		printf ("[%s]RC is %s[%x]\n", command, cmd_status_str(rc), rc);
	}

	golan->cmd_bm &= ~(1 << idx);
	return rc;
}

/**
  * Notify the HW that commands are ready
  */
static inline void send_command(struct golan *golan)
{
	wmb(); //Make sure the command is visible in "memory".
	writel(cpu_to_be32(golan->cmd_bm) , &golan->iseg->cmd_dbell);
}

static inline int send_command_and_wait(struct golan *golan, uint32_t cmd_idx,
					uint32_t inbox_idx, uint32_t outbox_idx, const char *command)
{
	golan_calc_sig(golan, cmd_idx, inbox_idx, outbox_idx);
	send_command(golan);
	return golan_cmd_wait(golan, cmd_idx, command);
}

/**
  * Prepare a FW command,
  * In - comamnd idx (Must be valid)
  * writes the command parameters.
  */
static inline struct golan_cmd_layout *write_cmd(struct golan *golan, int idx,
							uint16_t opcode, uint16_t opmod,
							uint16_t inbox_idx,
							uint16_t outbox_idx, uint16_t inlen,
							uint16_t outlen)
{
	struct golan_cmd_layout	*cmd	= get_cmd(golan , idx);
	struct golan_inbox_hdr *hdr	= (struct golan_inbox_hdr *)cmd->in;
	static uint8_t token;

	memset(cmd, 0, sizeof(*cmd));

	cmd->type		= GOLAN_PCI_CMD_XPORT;
	cmd->status_own		= CMD_OWNER_HW;
	cmd->outlen		= cpu_to_be32(outlen);
	cmd->inlen		= cpu_to_be32(inlen);
	hdr->opcode		= cpu_to_be16(opcode);
	hdr->opmod		= cpu_to_be16(opmod);

	if (inbox_idx != NO_MBOX) {
		memset(GET_INBOX(golan, inbox_idx), 0, MAILBOX_SIZE);
		cmd->in_ptr	= VIRT_2_BE64_BUS(GET_INBOX(golan, inbox_idx));
		cmd->token	= ++token;
	}
	if (outbox_idx != NO_MBOX) {
		memset(GET_OUTBOX(golan, outbox_idx), 0, MAILBOX_SIZE);
		cmd->out_ptr = VIRT_2_BE64_BUS(GET_OUTBOX(golan, outbox_idx));
	}

	golan->cmd_bm |= 1 << idx;

	assert ( cmd != NULL );
	return cmd;
}

static inline int golan_core_enable_hca(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	int rc = 0;

	DBGC(golan, "%s\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_ENABLE_HCA, 0x0,
			NO_MBOX, NO_MBOX,
			sizeof(struct golan_enable_hca_mbox_in),
			sizeof(struct golan_enable_hca_mbox_out));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	return rc;
}

static inline void golan_disable_hca(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DISABLE_HCA, 0x0,
					NO_MBOX, NO_MBOX,
				    sizeof(struct golan_disable_hca_mbox_in),
				    sizeof(struct golan_disable_hca_mbox_out));
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
}

static inline int golan_set_hca_cap(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	int rc;

	DBGC(golan, "%s\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_SET_HCA_CAP, 0x0,
			GEN_MBOX, NO_MBOX,
			sizeof(struct golan_cmd_set_hca_cap_mbox_in),
			sizeof(struct golan_cmd_set_hca_cap_mbox_out));

	golan->caps.flags &= ~GOLAN_DEV_CAP_FLAG_CMDIF_CSUM;
	DBGC( golan , "%s caps.uar_sz = %d\n", __FUNCTION__, golan->caps.uar_sz);
	DBGC( golan , "%s caps.log_pg_sz = %d\n", __FUNCTION__, golan->caps.log_pg_sz);
	DBGC( golan , "%s caps.log_uar_sz = %d\n", __FUNCTION__, be32_to_cpu(golan->caps.uar_page_sz));
	golan->caps.uar_page_sz = 0;


	memcpy(((struct golan_hca_cap *)GET_INBOX(golan, GEN_MBOX)),
		   &(golan->caps),
		   sizeof(struct golan_hca_cap));

	//if command failed we should reset the caps in golan->caps
	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	return rc;
}

static inline int golan_qry_hca_cap(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	int rc = 0;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_QUERY_HCA_CAP, 0x1,
					NO_MBOX, GEN_MBOX,
					sizeof(struct golan_cmd_query_hca_cap_mbox_in),
					sizeof(struct golan_cmd_query_hca_cap_mbox_out));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, GEN_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_query_hca_cap );

	memcpy(&(golan->caps),
		   ((struct golan_hca_cap *)GET_OUTBOX(golan, GEN_MBOX)),
		   sizeof(struct golan_hca_cap));
err_query_hca_cap:
	return rc;
}

static inline int golan_take_pages ( struct golan *golan, uint32_t pages, __be16 func_id ) {
	uint32_t out_num_entries = 0;
	int size_ibox =  sizeof(struct golan_manage_pages_inbox);
	int size_obox = sizeof(struct golan_manage_pages_outbox);
	int rc = 0;

	DBGC(golan, "%s\n", __FUNCTION__);

	while ( pages > 0 ) {
		uint32_t pas_num = min(pages, MAX_PASE_MBOX);
		unsigned i;
		struct golan_cmd_layout	*cmd;
		struct golan_manage_pages_inbox *in;
		struct golan_manage_pages_outbox_data *out;

		size_ibox += (pas_num * GOLAN_PAS_SIZE);
		size_obox += (pas_num * GOLAN_PAS_SIZE);

		cmd = write_cmd(golan, MEM_CMD_IDX, GOLAN_CMD_OP_MANAGE_PAGES, GOLAN_PAGES_TAKE,
				MEM_MBOX, MEM_MBOX,
				size_ibox,
				size_obox);

		in = (struct golan_manage_pages_inbox *)cmd->in; /* Warning (WE CANT USE THE LAST 2 FIELDS) */

		in->func_id 	= func_id; /* Already BE */
		in->num_entries = cpu_to_be32(pas_num);

		if ( ( rc = send_command_and_wait(golan, MEM_CMD_IDX, MEM_MBOX, MEM_MBOX, __FUNCTION__) ) == 0 ) {
			out = (struct golan_manage_pages_outbox_data *)GET_OUTBOX(golan, MEM_MBOX);
			out_num_entries = be32_to_cpu(((struct golan_manage_pages_outbox *)(cmd->out))->num_entries);
			for (i = 0; i < out_num_entries; ++i) {
				ufree(BE64_BUS_2_USR(out->pas[i]));
			}
		} else {
			if ( rc == -EBUSY ) {
				printf ( "HCA is busy (rc = -EBUSY)\n" );
			} else {
				printf ("%s: rc =0x%x[%s]<%x> syn 0x%x[0x%x] for %d pages\n",
						__FUNCTION__, rc, cmd_status_str(rc),
						CMD_SYND(golan, MEM_CMD_IDX),
						get_cmd( golan , MEM_CMD_IDX )->status_own,
						be32_to_cpu(CMD_SYND(golan, MEM_CMD_IDX)), pas_num);
			}
			return rc;
		}

		/* TODO: validate RC */
		pages -= out_num_entries;
	}
	DBGC( golan , "%s Pages handled\n", __FUNCTION__);
	return 0;
}

static inline int golan_provide_pages ( struct golan *golan , uint32_t pages, __be16 func_id ) {
	struct mbox *mailbox;
	int size_ibox =  sizeof(struct golan_manage_pages_inbox);
	int size_obox = sizeof(struct golan_manage_pages_outbox);
	int rc = 0;

	DBGC(golan, "%s\n", __FUNCTION__);

	while ( pages > 0 ) {
		uint32_t pas_num = min(pages, MAX_PASE_MBOX);
		unsigned i, j;
		struct golan_cmd_layout	*cmd;
		struct golan_manage_pages_inbox *in;
		userptr_t addr = 0;

		mailbox = GET_INBOX(golan, MEM_MBOX);
		size_ibox += (pas_num * GOLAN_PAS_SIZE);
		size_obox += (pas_num * GOLAN_PAS_SIZE);

		cmd = write_cmd(golan, MEM_CMD_IDX, GOLAN_CMD_OP_MANAGE_PAGES, GOLAN_PAGES_GIVE,
				MEM_MBOX, MEM_MBOX,
				size_ibox,
				size_obox);

		in = (struct golan_manage_pages_inbox *)cmd->in; /* Warning (WE CANT USE THE LAST 2 FIELDS) */

		in->func_id 	= func_id; /* Already BE */
		in->num_entries = cpu_to_be32(pas_num);

		for ( i = 0 , j = MANAGE_PAGES_PSA_OFFSET; i < pas_num; ++i ,++j ) {
			if (!(addr = umalloc(GOLAN_PAGE_SIZE))) {
				rc = -ENOMEM;
				printf ("Couldnt allocated page \n");
				goto malloc_dma_failed;
			}
			if (GOLAN_PAGE_MASK & user_to_phys(addr, 0)) {
				printf ("Addr not Page alligned [%lx %lx]\n", user_to_phys(addr, 0), addr);
			}
			mailbox->mblock.data[j]	= USR_2_BE64_BUS(addr);
		}

		if ( ( rc = send_command_and_wait(golan, MEM_CMD_IDX, MEM_MBOX, MEM_MBOX, __FUNCTION__) ) == 0 ) {
			pages -= pas_num;
			golan->total_dma_pages += pas_num;
		} else {
			if ( rc == -EBUSY ) {
				printf ( "HCA is busy (rc = -EBUSY)\n" );
			} else {
				printf ("%s: rc =0x%x[%s]<%x> syn 0x%x[0x%x] for %d pages\n",
						__FUNCTION__, rc, cmd_status_str(rc),
						CMD_SYND(golan, MEM_CMD_IDX),
						get_cmd( golan , MEM_CMD_IDX )->status_own,
						be32_to_cpu(CMD_SYND(golan, MEM_CMD_IDX)), pas_num);
			}
			ufree ( addr );
			goto err_send_command;
		}
	}
	DBGC( golan , "%s Pages handled\n", __FUNCTION__);
	return 0;

err_send_command:
malloc_dma_failed:
	/* Go over In box and free pages */
	/* Send Error to FW */
	/* What is next - Disable HCA? */
	printf("%s Failed (rc = 0x%x)\n", __FUNCTION__, rc);
	return rc;
}

static inline int golan_handle_pages(struct golan *golan,
					enum golan_qry_pages_mode qry,
					enum golan_manage_pages_mode mode)
{
	struct golan_cmd_layout	*cmd;

	int rc = 0;
	int32_t pages;
	uint16_t total_pages;
	__be16	func_id;

	DBGC(golan, "%s\n", __FUNCTION__);

	cmd = write_cmd(golan, MEM_CMD_IDX, GOLAN_CMD_OP_QUERY_PAGES, qry,
			NO_MBOX, NO_MBOX,
			sizeof(struct golan_query_pages_inbox),
			sizeof(struct golan_query_pages_outbox));

	rc = send_command_and_wait(golan, MEM_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_handle_pages_query );

	pages = be32_to_cpu(QRY_PAGES_OUT(golan, MEM_CMD_IDX)->num_pages);

	DBGC( golan , "%s pages needed: %d\n", __FUNCTION__, pages);

	func_id	= QRY_PAGES_OUT(golan, MEM_CMD_IDX)->func_id;

	total_pages = (( pages >= 0 ) ? pages : ( pages * ( -1 ) ));

	if ( mode == GOLAN_PAGES_GIVE ) {
		rc = golan_provide_pages(golan, total_pages, func_id);
	} else {
		rc = golan_take_pages(golan, golan->total_dma_pages, func_id);
		golan->total_dma_pages = 0;
	}

	if ( rc ) {
		printf ( "Failed to %s pages (rc = %d) - DMA pages allocated = %d\n",
			( ( mode == GOLAN_PAGES_GIVE ) ? "give" : "take" ), rc , golan->total_dma_pages );
		return rc;
	}

	return 0;

err_handle_pages_query:
	printf("%s Qyery pages failed (rc = 0x%x)\n", __FUNCTION__, rc);
	return rc;
}

static inline int golan_set_access_reg ( struct golan *golan __attribute__ (( unused )), uint32_t reg __attribute__ (( unused )))
{
#if 0
	write_cmd(golan, _CMD_IDX, GOLAN_CMD_OP_QUERY_PAGES, 0x0,
			NO_MBOX, NO_MBOX,
			sizeof(struct golan_reg_host_endianess),
			sizeof(struct golan_reg_host_endianess));
        in->arg = cpu_to_be32(arg);
        in->register_id = cpu_to_be16(reg_num);
#endif
	printf (" %s Not implemented yet\n", __FUNCTION__);
	return 0;
}

static inline void golan_cmd_uninit ( struct golan *golan )
{
	free_dma(golan->mboxes.outbox, GOLAN_PAGE_SIZE);
	free_dma(golan->mboxes.inbox, GOLAN_PAGE_SIZE);
	free_dma(golan->cmd.addr, GOLAN_PAGE_SIZE);
}

/**
 * Initialise Golan Command Q parameters
 *	-- Alocate a 4kb page for the Command Q
 *	-- Read the stride and log num commands available
 *	-- Write the address to cmdq_phy_addr in iseg
 * @v golan		Golan device
 */
static inline int golan_cmd_init ( struct golan *golan )
{
	int rc = 0;
	uint32_t addr_l_sz;

	if (!(golan->cmd.addr = malloc_dma(GOLAN_PAGE_SIZE , GOLAN_PAGE_SIZE))) {
		rc = -ENOMEM;
		goto malloc_dma_failed;
	}
	if (!(golan->mboxes.inbox = malloc_dma(GOLAN_PAGE_SIZE , GOLAN_PAGE_SIZE))) {
		rc = -ENOMEM;
		goto malloc_dma_inbox_failed;
	}
	if (!(golan->mboxes.outbox = malloc_dma(GOLAN_PAGE_SIZE , GOLAN_PAGE_SIZE))) {
		rc = -ENOMEM;
		goto malloc_dma_outbox_failed;
	}
	addr_l_sz	= be32_to_cpu(readl(&golan->iseg->cmdq_addr_l_sz));

	golan->cmd.log_stride	= addr_l_sz & 0xf;
	golan->cmd.size		= 1 << (( addr_l_sz >> 4 ) & 0xf);

	addr_l_sz = virt_to_bus(golan->cmd.addr);
	writel(0 /* cpu_to_be32(golan->cmd.addr) >> 32 */, &golan->iseg->cmdq_addr_h);
	writel(cpu_to_be32(addr_l_sz), &golan->iseg->cmdq_addr_l_sz);
	wmb(); //Make sure the addr is visible in "memory".

	addr_l_sz = be32_to_cpu(readl(&golan->iseg->cmdq_addr_l_sz));

	DBGC( golan , "%s Command interface was initialized\n", __FUNCTION__);
	return 0;

malloc_dma_outbox_failed:
	free_dma(golan->mboxes.inbox, GOLAN_PAGE_SIZE);
malloc_dma_inbox_failed:
	free_dma(golan->cmd.addr, GOLAN_PAGE_SIZE);
malloc_dma_failed:
	printf("%s Failed to initialize command interface (rc = 0x%x)\n",
		   __FUNCTION__, rc);
	return rc;
}

static inline int golan_hca_init(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	int rc = 0;

	DBGC(golan, "%s\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_INIT_HCA, 0x0,
			NO_MBOX, NO_MBOX,
			sizeof(struct golan_cmd_init_hca_mbox_in),
			sizeof(struct golan_cmd_init_hca_mbox_out));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	return rc;
}

static inline void golan_teardown_hca(struct golan *golan, enum golan_teardown op_mod)
{
	struct golan_cmd_layout	*cmd;
	int rc;

	DBGC (golan, "%s in\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_TEARDOWN_HCA, op_mod,
			NO_MBOX, NO_MBOX,
			sizeof(struct golan_cmd_teardown_hca_mbox_in),
			sizeof(struct golan_cmd_teardown_hca_mbox_out));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;

	DBGC (golan, "%s HCA teardown compleated\n", __FUNCTION__);
}

static inline int golan_alloc_uar(struct golan *golan)
{
	struct golan_uar *uar = &golan->uar;
	struct golan_cmd_layout *cmd;
	struct golan_alloc_uar_mbox_out *out;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_ALLOC_UAR, 0x0,
			NO_MBOX, NO_MBOX,
			sizeof(struct golan_alloc_uar_mbox_in),
			sizeof(struct golan_alloc_uar_mbox_out));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_alloc_uar_cmd );
	out = (struct golan_alloc_uar_mbox_out *) ( cmd->out );

	uar->index	= be32_to_cpu(out->uarn) & 0xffffff;

	uar->phys = (pci_bar_start(golan->pci, GOLAN_HCA_BAR) + (uar->index << GOLAN_PAGE_SHIFT));
	uar->virt = (void *)(ioremap(uar->phys, 0));

	DBGC( golan , "%s: UAR allocated with index 0x%x\n", __FUNCTION__, uar->index);
	return 0;

err_alloc_uar_cmd:
	printf ("%s [%d] out\n", __FUNCTION__, rc);
	return rc;
}

static void golan_dealloc_uar(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	uint32_t uar_index = golan->uar.index;
	int rc;

	DBGC (golan, "%s in\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DEALLOC_UAR, 0x0,
					NO_MBOX, NO_MBOX,
					sizeof(struct golan_free_uar_mbox_in),
					sizeof(struct golan_free_uar_mbox_out));

	((struct golan_free_uar_mbox_in *)(cmd->in))->uarn = cpu_to_be32(uar_index);
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	golan->uar.index = 0;

	DBGC (golan, "%s UAR (0x%x) was destroyed\n", __FUNCTION__, uar_index);
}

static void golan_eq_update_ci(struct golan_event_queue *eq, int arm)
{
	__be32 *addr = eq->doorbell + (arm ? 0 : 2);
	u32 val = (eq->cons_index & 0xffffff) | (eq->eqn << 24);
	writel(cpu_to_be32(val) , addr);
	/* We still want ordering, just not swabbing, so add a barrier */
	wmb();
}

static int golan_create_eq(struct golan *golan)
{
	struct golan_event_queue *eq = &golan->eq;
	struct golan_create_eq_mbox_in_data *in;
	struct golan_cmd_layout	*cmd;
	struct golan_create_eq_mbox_out *out;
	int rc, i;
	userptr_t addr;

	eq->cons_index	= 0;
	eq->size	= GOLAN_NUM_EQES * sizeof(eq->eqes[0]);
	addr		= umalloc(GOLAN_PAGE_SIZE);
	if (!addr) {
		rc = -ENOMEM;
		goto err_create_eq_eqe_alloc;
	}
	eq->eqes		= (struct golan_eqe *)user_to_virt(addr, 0);

	/* Set EQEs ownership bit to HW ownership */
	for (i = 0; i < GOLAN_NUM_EQES; ++i) {
		eq->eqes[i].owner = GOLAN_EQE_HW_OWNERSHIP;
	}

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_CREATE_EQ, 0x0,
			GEN_MBOX, NO_MBOX,
			sizeof(struct golan_create_eq_mbox_in) + GOLAN_PAS_SIZE,
			sizeof(struct golan_create_eq_mbox_out));

	in = (struct golan_create_eq_mbox_in_data *)GET_INBOX(golan, GEN_MBOX);

	/* Fill the physical address of the page */
	in->pas[0]		= USR_2_BE64_BUS(addr);
	in->ctx.log_sz_usr_page	= cpu_to_be32((ilog2(GOLAN_NUM_EQES)) << 24 | golan->uar.index);
	DBGC( golan , "UAR idx %x (BE %x)\n", golan->uar.index, in->ctx.log_sz_usr_page);
	in->events_mask		= cpu_to_be64(1 << GOLAN_EVENT_TYPE_PORT_CHANGE);

	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_create_eq_cmd );
	out = (struct golan_create_eq_mbox_out *)cmd->out;

	eq->eqn		= out->eq_number;
	eq->doorbell	= ((void *)golan->uar.virt) + GOLAN_EQ_DOORBELL_OFFSET;

	/* EQs are created in ARMED state */
	golan_eq_update_ci(eq, GOLAN_EQ_UNARMED);

	DBGC( golan , "%s: Event queue created (EQN = 0x%x)\n", __FUNCTION__, eq->eqn);
	return 0;

err_create_eq_cmd:
	ufree(virt_to_user(golan->eq.eqes));
err_create_eq_eqe_alloc:
	printf ("%s [%d] out\n", __FUNCTION__, rc);
	return rc;
}

static void golan_destory_eq(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	uint8_t eqn = golan->eq.eqn;
	int rc;

	DBGC (golan, "%s in\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DESTROY_EQ, 0x0,
					NO_MBOX, NO_MBOX,
					sizeof(struct golan_destroy_eq_mbox_in),
					sizeof(struct golan_destroy_eq_mbox_out));

	((struct golan_destroy_eq_mbox_in *)(cmd->in))->eqn = eqn;
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;

	ufree(virt_to_user(golan->eq.eqes));
	golan->eq.eqn = 0;

	DBGC( golan, "%s Event queue (0x%x) was destroyed\n", __FUNCTION__, eqn);
}

static int golan_alloc_pd(struct golan *golan)
{
	struct golan_cmd_layout *cmd;
	struct golan_alloc_pd_mbox_out *out;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_ALLOC_PD, 0x0,
			NO_MBOX, NO_MBOX,
			sizeof(struct golan_alloc_pd_mbox_in),
			sizeof(struct golan_alloc_pd_mbox_out));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_alloc_pd_cmd );
	out = (struct golan_alloc_pd_mbox_out *) ( cmd->out );

	golan->pdn = (be32_to_cpu(out->pdn) & 0xffffff);
	DBGC( golan , "%s: Protection domain created (PDN = 0x%x)\n", __FUNCTION__,
		golan->pdn);
	return 0;

err_alloc_pd_cmd:
	printf ("%s [%d] out\n", __FUNCTION__, rc);
	return rc;
}

static void golan_dealloc_pd(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	uint32_t pdn = golan->pdn;
	int rc;

	DBGC (golan,"%s in\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DEALLOC_PD, 0x0,
					NO_MBOX, NO_MBOX,
					sizeof(struct golan_alloc_pd_mbox_in),
					sizeof(struct golan_alloc_pd_mbox_out));

	((struct golan_dealloc_pd_mbox_in *)(cmd->in))->pdn = cpu_to_be32(pdn);
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	golan->pdn = 0;

	DBGC (golan ,"%s Protection domain (0x%x) was destroyed\n", __FUNCTION__, pdn);
}

static int golan_create_mkey(struct golan *golan)
{
	struct golan_create_mkey_mbox_in_data *in;
	struct golan_cmd_layout	*cmd;
	struct golan_create_mkey_mbox_out *out;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_CREATE_MKEY, 0x0,
					GEN_MBOX, NO_MBOX,
					sizeof(struct golan_create_mkey_mbox_in),
					sizeof(struct golan_create_mkey_mbox_out));

	in = (struct golan_create_mkey_mbox_in_data *)GET_INBOX(golan, GEN_MBOX);

	in->seg.flags			= GOLAN_IB_ACCESS_LOCAL_WRITE | GOLAN_IB_ACCESS_LOCAL_READ;
	in->seg.flags_pd		= cpu_to_be32(golan->pdn | GOLAN_MKEY_LEN64);
	in->seg.qpn_mkey7_0		= cpu_to_be32(0xffffff << GOLAN_CREATE_MKEY_SEG_QPN_BIT);

	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_create_mkey_cmd );
	out = (struct golan_create_mkey_mbox_out *) ( cmd->out );

	golan->mkey = ((be32_to_cpu(out->mkey) & 0xffffff) << 8);
	DBGC( golan , "%s: Got DMA Key for local access read/write (MKEY = 0x%x)\n",
		   __FUNCTION__, golan->mkey);
	return 0;
err_create_mkey_cmd:
	printf ("%s [%d] out\n", __FUNCTION__, rc);
	return rc;
}

static void golan_destroy_mkey(struct golan *golan)
{
	struct golan_cmd_layout	*cmd;
	u32 mkey = golan->mkey;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DESTROY_MKEY, 0x0,
					NO_MBOX, NO_MBOX,
					sizeof(struct golan_destroy_mkey_mbox_in),
					sizeof(struct golan_destroy_mkey_mbox_out));
	((struct golan_destroy_mkey_mbox_in *)(cmd->in))->mkey = cpu_to_be32(mkey >> 8);
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	golan->mkey = 0;

	DBGC( golan , "%s DMA Key (0x%x) for local access write was destroyed\n"
		   , __FUNCTION__, mkey);
}


/**
 * Initialise Golan PCI parameters
 *
 * @v golan		Golan device
 */
static inline void golan_pci_init(struct golan *golan)
{
	struct pci_device *pci = golan->pci;

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Get HCA BAR */
	golan->iseg	= ioremap ( pci_bar_start ( pci, GOLAN_HCA_BAR),
					GOLAN_PCI_CONFIG_BAR_SIZE );// Second var unused
}

static inline struct golan *golan_alloc()
{
	void *golan = zalloc(sizeof(struct golan));
	if ( !golan )
		goto err_zalloc;

	return golan;

err_zalloc:
	return NULL;
}

/**
 * Create completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @ret rc		Return status code
 */
static int golan_create_cq(struct ib_device *ibdev,
				struct ib_completion_queue *cq)
{
	struct golan *golan = ib_get_drvdata(ibdev);
	struct golan_completion_queue *golan_cq;
	struct golan_cmd_layout *cmd;
	struct golan_create_cq_mbox_in_data *in;
	struct golan_create_cq_mbox_out *out;
	int	rc;
	unsigned int i;
	userptr_t addr;

	golan_cq = zalloc(sizeof(*golan_cq));
	if (!golan_cq) {
		rc = -ENOMEM;
		goto err_create_cq;
	}
	golan_cq->size 			= sizeof(golan_cq->cqes[0]) * cq->num_cqes;
	golan_cq->doorbell_record 	= malloc_dma(GOLAN_CQ_DB_RECORD_SIZE,
							GOLAN_CQ_DB_RECORD_SIZE);
	if (!golan_cq->doorbell_record) {
		rc = -ENOMEM;
		goto err_create_cq_db_alloc;
	}

	addr = umalloc(GOLAN_PAGE_SIZE);
	if (!addr) {
		rc = -ENOMEM;
		goto err_create_cq_cqe_alloc;
	}
	golan_cq->cqes = (struct golan_cqe64 *)user_to_virt(addr, 0);

	/* Set CQEs ownership bit to HW ownership */
	for (i = 0; i < cq->num_cqes; ++i) {
		golan_cq->cqes[i].op_own = ((GOLAN_CQE_OPCODE_NOT_VALID <<
								    GOLAN_CQE_OPCODE_BIT) |
								    GOLAN_CQE_HW_OWNERSHIP);
	}

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_CREATE_CQ, 0x0,
					GEN_MBOX, NO_MBOX,
				    sizeof(struct golan_create_cq_mbox_in) + GOLAN_PAS_SIZE,
				    sizeof(struct golan_create_cq_mbox_out));

	in = (struct golan_create_cq_mbox_in_data *)GET_INBOX(golan, GEN_MBOX);

	/* Fill the physical address of the page */
	in->pas[0]		= USR_2_BE64_BUS(addr);
	in->ctx.cqe_sz_flags	= GOLAN_CQE_SIZE_64 << 5;
	in->ctx.log_sz_usr_page = cpu_to_be32(((ilog2(cq->num_cqes)) << 24) | golan->uar.index);
	in->ctx.c_eqn		= cpu_to_be16(golan->eq.eqn);
	in->ctx.db_record_addr	= VIRT_2_BE64_BUS(golan_cq->doorbell_record);

	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_create_cq_cmd );
	out = (struct golan_create_cq_mbox_out *) ( cmd->out );

	cq->cqn	= (be32_to_cpu(out->cqn) & 0xffffff);

	ib_cq_set_drvdata(cq, golan_cq);

	DBGC( golan , "%s CQ created successfully (CQN = 0x%lx)\n", __FUNCTION__, cq->cqn);
	return 0;

err_create_cq_cmd:
	ufree(virt_to_user(golan_cq->cqes));
err_create_cq_cqe_alloc:
	free_dma(golan_cq->doorbell_record, GOLAN_CQ_DB_RECORD_SIZE);
err_create_cq_db_alloc:
	free ( golan_cq );
err_create_cq:
	printf("%s out rc = 0x%x\n", __FUNCTION__, rc);
	return rc;
}

/**
 * Destroy completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void golan_destroy_cq(struct ib_device *ibdev,
				struct ib_completion_queue *cq)
{
	struct golan			*golan		= ib_get_drvdata(ibdev);
	struct golan_completion_queue	*golan_cq	= ib_cq_get_drvdata(cq);
	struct golan_cmd_layout		*cmd;
	uint32_t cqn = cq->cqn;
	int rc;

	DBGC (golan, "%s in\n", __FUNCTION__);

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DESTROY_CQ, 0x0,
					NO_MBOX, NO_MBOX,
				    sizeof(struct golan_destroy_cq_mbox_in),
				    sizeof(struct golan_destroy_cq_mbox_out));
	((struct golan_destroy_cq_mbox_in *)(cmd->in))->cqn = cpu_to_be32(cqn);
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	cq->cqn = 0;

	ib_cq_set_drvdata(cq, NULL);
	ufree(virt_to_user(golan_cq->cqes));
	free_dma(golan_cq->doorbell_record, GOLAN_CQ_DB_RECORD_SIZE);
	free(golan_cq);

	DBGC (golan, "%s CQ number 0x%x was destroyed\n", __FUNCTION__, cqn);
}

static void golan_cq_clean(struct ib_completion_queue *cq)
{
	ib_poll_cq(cq->ibdev, cq);
}

static int golan_qp_type_to_st(enum ib_queue_pair_type type)
{
	int qpt = type;

	switch (qpt) {
	case IB_QPT_RC:
		return GOLAN_QP_ST_RC;
	case IB_QPT_UD:
		return GOLAN_QP_ST_UD;
	case IB_QPT_SMI:
		return GOLAN_QP_ST_QP0;
	case IB_QPT_GSI:
		return GOLAN_QP_ST_QP1;
	case IB_QPT_ETH:
	default:
		return -EINVAL;
	}
}
#if 0
static int golan_is_special_qp(enum ib_queue_pair_type type)
{
	return (type == IB_QPT_GSI || type == IB_QPT_SMI);
}
#endif
static int golan_create_qp_aux(struct ib_device *ibdev,
				struct ib_queue_pair *qp,
				int *qpn)
{
	struct golan *golan = ib_get_drvdata(ibdev);
	struct golan_queue_pair *golan_qp;
	struct golan_create_qp_mbox_in_data *in;
	struct golan_cmd_layout *cmd;
	struct golan_create_qp_mbox_out *out;
	int rc;
	userptr_t addr;

	golan_qp = zalloc(sizeof(*golan_qp));
	if (!golan_qp) {
		rc = -ENOMEM;
		goto err_create_qp;
	}

	/* Calculate receive queue size */
	golan_qp->rq.size = qp->recv.num_wqes * GOALN_RECV_WQE_SIZE;
	if (golan_qp->rq.size > be16_to_cpu(golan->caps.max_wqe_sz_rq)) {	// CHECK THIS
		printf("%s receive wq size [%d] > max size [%d]\n", __FUNCTION__,
				golan_qp->rq.size, be16_to_cpu(golan->caps.max_wqe_sz_rq));
		rc = -EINVAL;
		goto err_create_qp_rq_size;
	}

	/* Calculate send queue size */
	golan_qp->sq.size = GOLAN_WQEBBS_PER_SEND_WQE;
	golan_qp->sq.size = (qp->send.num_wqes * (golan_qp->sq.size * GOLAN_SEND_WQE_BB_SIZE));
	if (golan_qp->sq.size > be16_to_cpu(golan->caps.max_wqe_sz_sq)) {	// CHECK THIS
		printf("%s send wq size [%d] > max size [%d]\n", __FUNCTION__,
				golan_qp->sq.size,
				be16_to_cpu(golan->caps.max_wqe_sz_sq));
		rc = -EINVAL;
		goto err_create_qp_sq_size;
	}

	golan_qp->size = golan_qp->sq.size + golan_qp->rq.size;

	/* allocate dma memory for WQEs (1 page is enough) - should change it */
	addr = umalloc(GOLAN_PAGE_SIZE);
	if (!addr) {
		rc = -ENOMEM;
		goto err_create_qp_wqe_alloc;
	}
	golan_qp->wqes		= user_to_virt(addr, 0);
	golan_qp->rq.wqes	= golan_qp->wqes;
	golan_qp->sq.wqes	= &(((struct golan_recv_wqe_ud *)(golan_qp->wqes))[qp->recv.num_wqes]);

	golan_qp->doorbell_record = malloc_dma(sizeof(struct golan_qp_db),
						sizeof(struct golan_qp_db));
	if (!golan_qp->doorbell_record) {
		rc = -ENOMEM;
		goto err_create_qp_db_alloc;
	}
	memset(golan_qp->doorbell_record, 0, sizeof(struct golan_qp_db));

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_CREATE_QP, 0x0,
			GEN_MBOX, NO_MBOX,
			sizeof(struct golan_create_qp_mbox_in) + GOLAN_PAS_SIZE,
			sizeof(struct golan_create_qp_mbox_out));

	in = (struct golan_create_qp_mbox_in_data *)GET_INBOX(golan, GEN_MBOX);

	/* Fill the physical address of the page */
	in->pas[0]			= USR_2_BE64_BUS(addr);
	in->ctx.qp_counter_set_usr_page	= cpu_to_be32(golan->uar.index);

	in->ctx.flags_pd 	= cpu_to_be32(golan->pdn);
	in->ctx.flags		= cpu_to_be32((golan_qp_type_to_st(qp->type)
						<< GOLAN_QP_CTX_ST_BIT) |
						(GOLAN_QP_PM_MIGRATED <<
						GOLAN_QP_CTX_PM_STATE_BIT));
//	cgs	set to 0, initialy.
//	atomic mode
	in->ctx.rq_size_stride	= (ilog2(qp->recv.num_wqes) << GOLAN_QP_CTX_RQ_SIZE_BIT);
	in->ctx.sq_crq_size		= cpu_to_be16(ilog2(golan_qp->sq.size / GOLAN_SEND_WQE_BB_SIZE)
										  << GOLAN_QP_CTX_SQ_SIZE_BIT);
	in->ctx.cqn_send 		= cpu_to_be32(qp->send.cq->cqn);
	in->ctx.cqn_recv 		= cpu_to_be32(qp->recv.cq->cqn);
	in->ctx.db_rec_addr 	= VIRT_2_BE64_BUS(golan_qp->doorbell_record);

	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_create_qp_cmd );
	out = (struct golan_create_qp_mbox_out *)cmd->out;

	*qpn = (be32_to_cpu(out->qpn) & 0xffffff);
	/*
	* Hardware wants QPN written in big-endian order (after
	* shifting) for send doorbell.  Precompute this value to save
	* a little bit when posting sends.
	*/
	golan_qp->doorbell_qpn	= cpu_to_be32(*qpn << 8);
	golan_qp->state			= GOLAN_IB_QPS_RESET;

	ib_qp_set_drvdata(qp, golan_qp);

	return 0;

err_create_qp_cmd:
	free_dma(golan_qp->doorbell_record, sizeof(struct golan_qp_db));
err_create_qp_db_alloc:
	ufree((userptr_t)golan_qp->wqes);
err_create_qp_wqe_alloc:
err_create_qp_sq_size:
err_create_qp_rq_size:
	free ( golan_qp );
err_create_qp:
	return rc;
}

/**
 * Create queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int golan_create_qp(struct ib_device *ibdev,
				struct ib_queue_pair *qp)
{
	int rc, qpn = -1;

	switch (qp->type) {
	case IB_QPT_UD:
	case IB_QPT_SMI:
	case IB_QPT_GSI:
		rc = golan_create_qp_aux(ibdev, qp, &qpn);
		if (rc) {
			printf("%s Failed to create QP (rc = 0x%x)\n", __FUNCTION__, rc);
			return rc;
		}
		qp->qpn = qpn;

		break;
	case IB_QPT_ETH:
	case IB_QPT_RC:
	default:
		printf("%s unsupported QP type (0x%x)\n", __FUNCTION__, qp->type);
		return -EINVAL;
	}

	return 0;
}

static int golan_modify_qp_rst_to_init(struct ib_device *ibdev,
					struct ib_queue_pair *qp __unused,
					struct golan_modify_qp_mbox_in_data *in)
{
	int rc = 0;

	in->ctx.qkey			= cpu_to_be32((uint32_t)(qp->qkey));

	in->ctx.pri_path.port		= ibdev->port;
	in->ctx.flags			|= cpu_to_be32(GOLAN_QP_PM_MIGRATED << GOLAN_QP_CTX_PM_STATE_BIT);
	in->ctx.pri_path.pkey_index	= ibdev->pkey_index;
	/* QK is 0 */
	/* QP cntr set 0 */
	return rc;
}

static int golan_modify_qp_init_to_rtr(struct ib_device *ibdev __unused,
					struct ib_queue_pair *qp __unused,
					struct golan_modify_qp_mbox_in_data *in)
{
	int rc = 0;

	in->optparam = 0;
	return rc;
}

static int golan_modify_qp_rtr_to_rts(struct ib_device *ibdev __unused,
					struct ib_queue_pair *qp __unused,
					struct golan_modify_qp_mbox_in_data *in __unused)
{
	int rc = 0;

	in->optparam = 0;
	/* In good flow psn in 0 */
	return rc;
}

static int golan_modify_qp_to_rst(struct ib_device *ibdev,
					struct ib_queue_pair *qp)
{
	struct golan *golan = ib_get_drvdata(ibdev);
	struct golan_queue_pair *golan_qp = ib_qp_get_drvdata(qp);
	struct golan_cmd_layout	*cmd;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_2RST_QP, 0x0,
					NO_MBOX, NO_MBOX,
					sizeof(struct golan_modify_qp_mbox_in),
					sizeof(struct golan_modify_qp_mbox_out));
	((struct golan_modify_qp_mbox_in *)(cmd->in))->qpn = cpu_to_be32(qp->qpn);
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_modify_qp_2rst_cmd );

	golan_qp->state = GOLAN_IB_QPS_RESET;
	DBGC( golan , "%s QP number 0x%lx was modified to RESET\n",
		__FUNCTION__, qp->qpn);

	return 0;

err_modify_qp_2rst_cmd:
	printf("%s Failed to modify QP number 0x%lx (rc = 0x%x)\n",
		__FUNCTION__, qp->qpn, rc);
	return rc;
}

static int (*golan_modify_qp_methods[])(struct ib_device *ibdev,
					struct ib_queue_pair *qp,
					struct golan_modify_qp_mbox_in_data *in) = {

	[GOLAN_IB_QPS_RESET]	= golan_modify_qp_rst_to_init,
	[GOLAN_IB_QPS_INIT]	= golan_modify_qp_init_to_rtr,
	[GOLAN_IB_QPS_RTR]	= golan_modify_qp_rtr_to_rts
};

static int golan_modify_qp(struct ib_device *ibdev,
				struct ib_queue_pair *qp)
{
	struct golan *golan = ib_get_drvdata(ibdev);
	struct golan_queue_pair *golan_qp = ib_qp_get_drvdata(qp);
	struct golan_modify_qp_mbox_in_data *in;
	struct golan_cmd_layout	*cmd;
	enum golan_ib_qp_state prev_state;
	int rc;
	int modify_cmd[] = {GOLAN_CMD_OP_RST2INIT_QP,
				GOLAN_CMD_OP_INIT2RTR_QP,
				GOLAN_CMD_OP_RTR2RTS_QP};

	while (golan_qp->state < GOLAN_IB_QPS_RTS) {
		prev_state = golan_qp->state;
		cmd = write_cmd(golan, DEF_CMD_IDX, modify_cmd[golan_qp->state], 0x0,
						GEN_MBOX, NO_MBOX,
						sizeof(struct golan_modify_qp_mbox_in),
						sizeof(struct golan_modify_qp_mbox_out));

		in = (struct golan_modify_qp_mbox_in_data *)GET_INBOX(golan, GEN_MBOX);
		((struct golan_modify_qp_mbox_in *)(cmd->in))->qpn = cpu_to_be32(qp->qpn);
		rc = golan_modify_qp_methods[golan_qp->state](ibdev, qp, in);
		if (rc) {
			goto err_modify_qp_fill_inbox;
		}
//		in->ctx.qp_counter_set_usr_page	= cpu_to_be32(golan->uar.index);
		rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
		GOLAN_CHECK_RC_AND_CMD_STATUS( err_modify_qp_cmd );

		++(golan_qp->state);

		DBGC( golan , "%s QP number 0x%lx was modified from %s to %s\n",
			__FUNCTION__, qp->qpn, golan_qp_state_as_string[prev_state],
			golan_qp_state_as_string[golan_qp->state]);
	}

	DBGC( golan , "%s QP number 0x%lx is ready to receive/send packets.\n",
		__FUNCTION__, qp->qpn);
	return 0;

err_modify_qp_cmd:
err_modify_qp_fill_inbox:
	printf("%s Failed to modify QP number 0x%lx (rc = 0x%x)\n",
		   __FUNCTION__, qp->qpn, rc);
	return rc;
}

/**
 * Destroy queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void golan_destroy_qp(struct ib_device *ibdev,
				struct ib_queue_pair *qp)
{
	struct golan		*golan		= ib_get_drvdata(ibdev);
	struct golan_queue_pair	*golan_qp	= ib_qp_get_drvdata(qp);
	struct golan_cmd_layout			*cmd;
	unsigned long		 qpn = qp->qpn;
	int rc;

	DBGC (golan, "%s in\n", __FUNCTION__);

	if (golan_qp->state != GOLAN_IB_QPS_RESET) {
		if (golan_modify_qp_to_rst(ibdev, qp)) {
			printf("%s Failed to modify QP 0x%lx to RESET\n", __FUNCTION__,
				   qp->qpn);
		}
	}

	if (qp->recv.cq) {
		golan_cq_clean(qp->recv.cq);
	}
	if (qp->send.cq && (qp->send.cq != qp->recv.cq)) {
		golan_cq_clean(qp->send.cq);
	}

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DESTROY_QP, 0x0,
					NO_MBOX, NO_MBOX,
				    sizeof(struct golan_destroy_qp_mbox_in),
				    sizeof(struct golan_destroy_qp_mbox_out));
	((struct golan_destroy_qp_mbox_in *)(cmd->in))->qpn = cpu_to_be32(qpn);
	rc = send_command_and_wait(golan, DEF_CMD_IDX, NO_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;
	qp->qpn = 0;

	ib_qp_set_drvdata(qp, NULL);
	free_dma(golan_qp->doorbell_record, sizeof(struct golan_qp_db));
	ufree((userptr_t)golan_qp->wqes);
	free(golan_qp);

	DBGC( golan ,"%s QP 0x%lx was destroyed\n", __FUNCTION__, qpn);
}

/**
 * Calculate transmission rate
 *
 * @v av		Address vector
 * @ret golan_rate	Golan rate
 */
static unsigned int golan_rate(enum ib_rate rate) {
	return (((rate >= IB_RATE_2_5) && (rate <= IB_RATE_120)) ? (rate + 5) : 0);
}

/**
 * Post send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int golan_post_send(struct ib_device *ibdev,
				struct ib_queue_pair *qp,
				struct ib_address_vector *av,
				struct io_buffer *iobuf)
{
	struct golan			*golan		= ib_get_drvdata(ibdev);
	struct golan_queue_pair		*golan_qp	= ib_qp_get_drvdata(qp);
	struct golan_send_wqe_ud	*wqe		= NULL;
	struct golan_av			*datagram	= NULL;
	unsigned long			wqe_idx_mask;
	unsigned long			wqebb_idx_mask;
	unsigned long			wqe_idx;
	struct golan_wqe_data_seg	*data		= NULL;
	struct golan_wqe_ctrl_seg	*ctrl		= NULL;
//	static uint8_t			toggle		= 0;


	wqe_idx_mask = (qp->send.num_wqes - 1);
	wqe_idx = (qp->send.next_idx & wqe_idx_mask);
	if (qp->send.iobufs[wqe_idx]) {
		printf("%s Send queue of QPN 0x%lx is full\n", __FUNCTION__, qp->qpn);
		return -ENOMEM;
	}

	qp->send.iobufs[wqe_idx] = iobuf;

	// change to this
	//wqe_size_in_octa_words = golan_qp->sq.wqe_size_in_wqebb >> 4;

	wqebb_idx_mask	= (GOLAN_WQEBBS_PER_SEND_WQE * qp->send.num_wqes) - 1;
	wqe 			= golan_qp->sq.wqes +
					  ((golan_qp->sq.next_idx & wqebb_idx_mask) * GOLAN_SEND_WQE_BB_SIZE);

	//CHECK HW OWNERSHIP BIT ???

	memset(wqe, 0, sizeof(*wqe));

	ctrl			= &wqe->ctrl;
	ctrl->opmod_idx_opcode	= cpu_to_be32(GOLAN_SEND_OPCODE |
						  ((u32)(golan_qp->sq.next_idx) <<
						  GOLAN_WQE_CTRL_WQE_IDX_BIT));
	ctrl->qpn_ds		= cpu_to_be32(GOALN_SEND_WQE_SIZE >> 4) |
							  golan_qp->doorbell_qpn;
	ctrl->fm_ce_se		= 0x8;//10 - 0 - 0
	data			= &wqe->data;
	data->byte_count	= cpu_to_be32(iob_len(iobuf));
	data->lkey		= cpu_to_be32(golan->mkey);
	data->addr		= VIRT_2_BE64_BUS(iobuf->data);

	datagram		= &wqe->datagram;
	datagram->key.qkey.qkey	= cpu_to_be32(av->qkey);
	datagram->dqp_dct	= cpu_to_be32((1 << 31) | av->qpn);
	datagram->stat_rate_sl	= ((golan_rate(av->rate) << 4) | av->sl);
	datagram->fl_mlid	= (ibdev->lid & 0x007f); /* take only the 7 low bits of the LID */
	datagram->rlid		= cpu_to_be16(av->lid);
	datagram->grh_gid_fl	= cpu_to_be32(av->gid_present << 30);
	memcpy(datagram->rgid, av->gid.bytes, 16 /* sizeof(datagram->rgid) */);

	/*
	* Make sure that descriptors are written before
	* updating doorbell record and ringing the doorbell
	*/
	++(qp->send.next_idx);
	golan_qp->sq.next_idx = (golan_qp->sq.next_idx + GOLAN_WQEBBS_PER_SEND_WQE);
	golan_qp->doorbell_record->send_db = cpu_to_be16(golan_qp->sq.next_idx);
	writeq(*((__be64 *)ctrl), (uint32_t)golan->uar.virt + 0x800);// +
//			((toggle++ & 0x1) ? 0x100 : 0x0));
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
static int golan_post_recv(struct ib_device *ibdev,
				struct ib_queue_pair *qp,
				struct io_buffer *iobuf)
{
	struct golan		*golan		= ib_get_drvdata(ibdev);
	struct golan_queue_pair	*golan_qp	= ib_qp_get_drvdata(qp);
	struct ib_work_queue		*wq	= &qp->recv;
	struct golan_recv_wqe_ud	*wqe;
	unsigned int wqe_idx_mask;

	/* Allocate work queue entry */
	wqe_idx_mask = (wq->num_wqes - 1);
	if (wq->iobufs[wq->next_idx & wqe_idx_mask]) {
		printf("%s Receive queue of QPN 0x%lx is full\n", __FUNCTION__, qp->qpn);
		return -ENOMEM;
	}

	wq->iobufs[wq->next_idx & wqe_idx_mask] = iobuf;
	wqe = &((struct golan_recv_wqe_ud *)(golan_qp->rq.wqes))[wq->next_idx & wqe_idx_mask];

	memset(wqe, 0, sizeof(*wqe));

	wqe->data.byte_count	= cpu_to_be32(iob_tailroom(iobuf));

	wqe->data.lkey		= cpu_to_be32(golan->mkey);
	wqe->data.addr		= VIRT_2_BE64_BUS(iobuf->data);
	++wq->next_idx;

	/*
	* Make sure that descriptors are written before
	* updating doorbell record and ringing the doorbell
	*/
	wmb();
	golan_qp->doorbell_record->recv_db = cpu_to_be16(qp->recv.next_idx & 0xffff);

	return 0;
}

/**
 * Issue management datagram
 *
 * @v ibdev		Infiniband device
 * @v mad		Management datagram
 * @ret rc		Return status code
 */
static int golan_mad(struct ib_device *ibdev, union ib_mad *mad)
{
	struct golan *golan = ib_get_drvdata(ibdev);
	struct golan_cmd_layout	*cmd;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_MAD_IFC,
			GOLAN_MAD_IFC_NO_VALIDATION, GEN_MBOX, GEN_MBOX,
			sizeof(struct golan_mad_ifc_mbox_in),
			sizeof(struct golan_mad_ifc_mbox_out));

	((struct golan_mad_ifc_mbox_in *)(cmd->in))->port = (u8)ibdev->port;

	linker_assert(sizeof(*mad) == GOLAN_MAD_SIZE, mad_size_mismatch);

	/* Copy in request packet */
	memcpy(GET_INBOX(golan, GEN_MBOX), mad->bytes, GOLAN_MAD_SIZE);

	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, GEN_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_mad_ifc_cmd );

	/* Copy out response packet */
	memcpy(mad->bytes, GET_OUTBOX(golan, GEN_MBOX), GOLAN_MAD_SIZE);

	return 0;
err_mad_ifc_cmd:
	printf ("%s [%d] out\n", __FUNCTION__, rc);
	return rc;
}

static int golan_complete(struct ib_device *ibdev,
				struct ib_completion_queue *cq,
				struct golan_cqe64 *cqe64)
{
	struct golan *golan	= ib_get_drvdata(ibdev);
	struct ib_work_queue *wq;
	struct ib_queue_pair *qp;
	struct io_buffer *iobuf = NULL;
	struct ib_address_vector recv_dest;
	struct ib_address_vector recv_source;
	struct ib_global_route_header *grh;
	struct golan_err_cqe *err_cqe64;
	int gid_present, idx;
	u16 wqe_ctr;
	uint8_t opcode;
	static int error_state;
	uint32_t qpn = be32_to_cpu(cqe64->sop_drop_qpn) & 0xffffff;

	int is_send = 0;
	size_t len;

	opcode = cqe64->op_own >> GOLAN_CQE_OPCODE_BIT;
	DBGC2( golan , "%s completion with opcode 0x%x\n", __FUNCTION__, opcode);

	if (opcode == GOLAN_CQE_REQ || opcode == GOLAN_CQE_REQ_ERR) {
		is_send = 1;
	} else {
		is_send = 0;
	}
	if (opcode == GOLAN_CQE_REQ_ERR || opcode == GOLAN_CQE_RESP_ERR) {
		err_cqe64 = (struct golan_err_cqe *)cqe64;
		int i = 0;
		if (!error_state++) {
			printf("\n");
			for ( i = 0 ; i < 16 ; i += 2 ) {
				printf("%x       %x\n",
						be32_to_cpu(((uint32_t *)(err_cqe64))[i]),
						be32_to_cpu(((uint32_t *)(err_cqe64))[i + 1]));
			}
			printf("CQE with error: Syndrome(0x%x), VendorSynd(0x%x), HW_SYN(0x%x)\n",
					err_cqe64->syndrome, err_cqe64->vendor_err_synd,
					err_cqe64->hw_syndrom);
		}
	}
	/* Identify work queue */
	wq = ib_find_wq(cq, qpn, is_send);
	if (!wq) {
		printf("%s unknown %s QPN 0x%x in CQN 0x%lx\n",
		       __FUNCTION__, (is_send ? "send" : "recv"), qpn, cq->cqn);
		return -EINVAL;
	}
	qp = wq->qp;

	wqe_ctr = be16_to_cpu(cqe64->wqe_counter);
	if (is_send) {
		wqe_ctr &= ((GOLAN_WQEBBS_PER_SEND_WQE * wq->num_wqes) - 1);
		idx = wqe_ctr / GOLAN_WQEBBS_PER_SEND_WQE;
	} else {
		idx = wqe_ctr & (wq->num_wqes - 1);
	}

	iobuf = wq->iobufs[idx];
	if (!iobuf) {
		printf("%s IO Buffer 0x%x not found in QPN 0x%x\n",
			   __FUNCTION__, idx, qpn);
		return -EINVAL;
	}
	wq->iobufs[idx] = NULL;

	if (is_send) {
		ib_complete_send(ibdev, qp, iobuf, (opcode == GOLAN_CQE_REQ_ERR));
	} else {
		switch (qp->type) {
		case IB_QPT_SMI:
		case IB_QPT_GSI:
		case IB_QPT_UD:
			len = be32_to_cpu(cqe64->byte_cnt);
			assert(len <= iob_tailroom(iobuf));
			iob_put(iobuf, len);

			memset(&recv_dest, 0, sizeof(recv_dest));
			recv_dest.qpn = qpn;
			/* Construct address vector */
			memset(&recv_source, 0, sizeof(recv_source));
			recv_source.qpn = be32_to_cpu(cqe64->flags_rqpn) & 0xffffff;
			recv_source.lid = be16_to_cpu(cqe64->slid);
			recv_source.sl	= (be32_to_cpu(cqe64->flags_rqpn) >> 24) & 0xf;
			gid_present = (be32_to_cpu(cqe64->flags_rqpn) >> 28) & 3;
			if (!gid_present) {
				recv_dest.gid_present = recv_source.gid_present = 0;
			} else {
				recv_dest.gid_present = recv_source.gid_present = 1;
				//if (recv_source.gid_present == 0x1) {
				assert(iob_len(iobuf) >= sizeof(*grh));
				grh = iobuf->data;
				memcpy(&recv_source.gid, &grh->sgid, sizeof(recv_source.gid));
				memcpy(&recv_dest.gid, &grh->dgid, sizeof(recv_dest.gid));
				//} else { // recv_source.gid_present = 0x3
					/* GRH is located in the upper 64 byte of the CQE128
					 * currently not supported */
					//;
				//}
			}
			iob_pull(iobuf, sizeof(*grh));
			break;
		case IB_QPT_RC:
		case IB_QPT_ETH:
		default:
			printf("%s Unsupported QP type (0x%x)\n", __FUNCTION__, qp->type);
			return -EINVAL;
		}
		ib_complete_recv(ibdev, qp, &recv_dest, &recv_source, iobuf, (opcode == GOLAN_CQE_RESP_ERR));
	}
	return 0;
}

static int golan_is_hw_ownership(struct ib_completion_queue *cq,
								 struct golan_cqe64 *cqe64)
{
	return ((cqe64->op_own & GOLAN_CQE_OWNER_MASK) !=
			((cq->next_idx >> ilog2(cq->num_cqes)) & 1));
}
static void golan_poll_cq(struct ib_device *ibdev,
				struct ib_completion_queue *cq)
{
	unsigned int		i;
	int			rc = 0;
	unsigned int		cqe_idx_mask;
	struct golan_cqe64	*cqe64;
	struct golan_completion_queue *golan_cq = ib_cq_get_drvdata(cq);
	struct golan		*golan	= ib_get_drvdata(ibdev);

	for (i = 0; i < cq->num_cqes; ++i) {
		/* Look for completion entry */
		cqe_idx_mask = (cq->num_cqes - 1);
		cqe64 = &golan_cq->cqes[cq->next_idx & cqe_idx_mask];
		/* temporary valid only for 64 byte CQE */
		if (golan_is_hw_ownership(cq, cqe64) ||
			((cqe64->op_own >> GOLAN_CQE_OPCODE_BIT) ==
			GOLAN_CQE_OPCODE_NOT_VALID)) {
			break;	/* HW ownership */
		}

		DBGC2( golan , "%s CQN 0x%lx [%ld] \n", __FUNCTION__, cq->cqn, cq->next_idx);
		/*
		 * Make sure we read CQ entry contents after we've checked the
		 * ownership bit. (PRM - 6.5.3.2)
		 */
		rmb();
		rc = golan_complete(ibdev, cq, cqe64);
		if (rc != 0) {
			printf("%s CQN 0x%lx failed to complete\n", __FUNCTION__, cq->cqn);
		}

		/* Update completion queue's index */
		cq->next_idx++;

		/* Update doorbell record */
		*(golan_cq->doorbell_record) = cpu_to_be32(cq->next_idx & 0xffffff);
	}
}

static const char *golan_eqe_type_str(u8 type)
{
	switch (type) {
	case GOLAN_EVENT_TYPE_COMP:
		return "GOLAN_EVENT_TYPE_COMP";
	case GOLAN_EVENT_TYPE_PATH_MIG:
		return "GOLAN_EVENT_TYPE_PATH_MIG";
	case GOLAN_EVENT_TYPE_COMM_EST:
		return "GOLAN_EVENT_TYPE_COMM_EST";
	case GOLAN_EVENT_TYPE_SQ_DRAINED:
		return "GOLAN_EVENT_TYPE_SQ_DRAINED";
	case GOLAN_EVENT_TYPE_SRQ_LAST_WQE:
		return "GOLAN_EVENT_TYPE_SRQ_LAST_WQE";
	case GOLAN_EVENT_TYPE_SRQ_RQ_LIMIT:
		return "GOLAN_EVENT_TYPE_SRQ_RQ_LIMIT";
	case GOLAN_EVENT_TYPE_CQ_ERROR:
		return "GOLAN_EVENT_TYPE_CQ_ERROR";
	case GOLAN_EVENT_TYPE_WQ_CATAS_ERROR:
		return "GOLAN_EVENT_TYPE_WQ_CATAS_ERROR";
	case GOLAN_EVENT_TYPE_PATH_MIG_FAILED:
		return "GOLAN_EVENT_TYPE_PATH_MIG_FAILED";
	case GOLAN_EVENT_TYPE_WQ_INVAL_REQ_ERROR:
		return "GOLAN_EVENT_TYPE_WQ_INVAL_REQ_ERROR";
	case GOLAN_EVENT_TYPE_WQ_ACCESS_ERROR:
		return "GOLAN_EVENT_TYPE_WQ_ACCESS_ERROR";
	case GOLAN_EVENT_TYPE_SRQ_CATAS_ERROR:
		return "GOLAN_EVENT_TYPE_SRQ_CATAS_ERROR";
	case GOLAN_EVENT_TYPE_INTERNAL_ERROR:
		return "GOLAN_EVENT_TYPE_INTERNAL_ERROR";
	case GOLAN_EVENT_TYPE_PORT_CHANGE:
		return "GOLAN_EVENT_TYPE_PORT_CHANGE";
	case GOLAN_EVENT_TYPE_GPIO_EVENT:
		return "GOLAN_EVENT_TYPE_GPIO_EVENT";
	case GOLAN_EVENT_TYPE_REMOTE_CONFIG:
		return "GOLAN_EVENT_TYPE_REMOTE_CONFIG";
	case GOLAN_EVENT_TYPE_DB_BF_CONGESTION:
		return "GOLAN_EVENT_TYPE_DB_BF_CONGESTION";
	case GOLAN_EVENT_TYPE_STALL_EVENT:
		return "GOLAN_EVENT_TYPE_STALL_EVENT";
	case GOLAN_EVENT_TYPE_CMD:
		return "GOLAN_EVENT_TYPE_CMD";
	case GOLAN_EVENT_TYPE_PAGE_REQUEST:
		return "GOLAN_EVENT_TYPE_PAGE_REQUEST";
	default:
		return "Unrecognized event";
	}
}

static const char *golan_eqe_port_subtype_str(u8 subtype)
{
	switch (subtype) {
	case GOLAN_PORT_CHANGE_SUBTYPE_DOWN:
		return "GOLAN_PORT_CHANGE_SUBTYPE_DOWN";
	case GOLAN_PORT_CHANGE_SUBTYPE_ACTIVE:
		return "GOLAN_PORT_CHANGE_SUBTYPE_ACTIVE";
	case GOLAN_PORT_CHANGE_SUBTYPE_INITIALIZED:
		return "GOLAN_PORT_CHANGE_SUBTYPE_INITIALIZED";
	case GOLAN_PORT_CHANGE_SUBTYPE_LID:
		return "GOLAN_PORT_CHANGE_SUBTYPE_LID";
	case GOLAN_PORT_CHANGE_SUBTYPE_PKEY:
		return "GOLAN_PORT_CHANGE_SUBTYPE_PKEY";
	case GOLAN_PORT_CHANGE_SUBTYPE_GUID:
		return "GOLAN_PORT_CHANGE_SUBTYPE_GUID";
	case GOLAN_PORT_CHANGE_SUBTYPE_CLIENT_REREG:
		return "GOLAN_PORT_CHANGE_SUBTYPE_CLIENT_REREG";
	default:
		return "Unrecognized event";
	}
}

static inline void golan_handle_port_event(struct golan *golan, struct golan_eqe *eqe)
{
	struct ib_device *ibdev;
	u8 port;

	port = (eqe->data.port.port >> 4) & 0xf;
	ibdev = golan->ports[port - 1].ibdev;

	if ( ! ib_is_open ( ibdev ) )
		return;

	switch (eqe->sub_type) {
	case GOLAN_PORT_CHANGE_SUBTYPE_CLIENT_REREG:
	case GOLAN_PORT_CHANGE_SUBTYPE_ACTIVE:
		ib_smc_update(ibdev, golan_mad);
	case GOLAN_PORT_CHANGE_SUBTYPE_DOWN:
	case GOLAN_PORT_CHANGE_SUBTYPE_LID:
	case GOLAN_PORT_CHANGE_SUBTYPE_PKEY:
	case GOLAN_PORT_CHANGE_SUBTYPE_GUID:
	case GOLAN_PORT_CHANGE_SUBTYPE_INITIALIZED:
		DBGC( golan , "%s event %s(%d) (sub event %s(%d))arrived on port %d\n",
			   __FUNCTION__, golan_eqe_type_str(eqe->type), eqe->type,
			   golan_eqe_port_subtype_str(eqe->sub_type),
			   eqe->sub_type, port);
		break;
	default:
		printf("%s Port event with unrecognized subtype: port %d, sub_type %d\n",
			   __FUNCTION__, port, eqe->sub_type);
	}
}

static struct golan_eqe *golan_next_eqe_sw(struct golan_event_queue *eq)
{
	uint32_t entry = (eq->cons_index & (GOLAN_NUM_EQES - 1));
	struct golan_eqe *eqe = &(eq->eqes[entry]);
	return ((eqe->owner != ((eq->cons_index >> ilog2(GOLAN_NUM_EQES)) & 1)) ? NULL : eqe);
}


/**
 * Poll event queue
 *
 * @v ibdev		Infiniband device
 */
static void golan_poll_eq(struct ib_device *ibdev)
{
	struct golan		*golan	= ib_get_drvdata(ibdev);
	struct golan_event_queue *eq	= &(golan->eq);
	struct golan_eqe	*eqe;
	u32 cqn;
	int counter = 0;

	while ((eqe = golan_next_eqe_sw(eq)) && (counter < GOLAN_NUM_EQES)) {
		/*
		 * Make sure we read EQ entry contents after we've
		 * checked the ownership bit.
		 */
		rmb();

		DBGC( golan , "%s eqn %d, eqe type %s\n", __FUNCTION__, eq->eqn,
			   golan_eqe_type_str(eqe->type));
		switch (eqe->type) {
		case GOLAN_EVENT_TYPE_COMP:
			/* We dont need to handle completion events since we
			 * poll all the CQs after polling the EQ */
			break;
		case GOLAN_EVENT_TYPE_PATH_MIG:
		case GOLAN_EVENT_TYPE_COMM_EST:
		case GOLAN_EVENT_TYPE_SQ_DRAINED:
		case GOLAN_EVENT_TYPE_SRQ_LAST_WQE:
		case GOLAN_EVENT_TYPE_WQ_CATAS_ERROR:
		case GOLAN_EVENT_TYPE_PATH_MIG_FAILED:
		case GOLAN_EVENT_TYPE_WQ_INVAL_REQ_ERROR:
		case GOLAN_EVENT_TYPE_WQ_ACCESS_ERROR:
		case GOLAN_EVENT_TYPE_SRQ_RQ_LIMIT:
		case GOLAN_EVENT_TYPE_SRQ_CATAS_ERROR:
			DBGC( golan , "%s event %s(%d) arrived\n", __FUNCTION__,
				   golan_eqe_type_str(eqe->type), eqe->type);
			break;
		case GOLAN_EVENT_TYPE_CMD:
//			golan_cmd_comp_handler(be32_to_cpu(eqe->data.cmd.vector));
			break;
		case GOLAN_EVENT_TYPE_PORT_CHANGE:
			golan_handle_port_event(golan, eqe);
			break;
		case GOLAN_EVENT_TYPE_CQ_ERROR:
			cqn = be32_to_cpu(eqe->data.cq_err.cqn) & 0xffffff;
			printf("CQ error on CQN 0x%x, syndrom 0x%x\n",
				   cqn, eqe->data.cq_err.syndrome);
//			mlx5_cq_event(dev, cqn, eqe->type);
			break;
		case GOLAN_EVENT_TYPE_PAGE_REQUEST:
			{
				/* we should check if we get this event while we
				 * waiting for a command */
				u16 func_id = be16_to_cpu(eqe->data.req_pages.func_id);
				s16 npages = be16_to_cpu(eqe->data.req_pages.num_pages);

				printf("%s page request for func 0x%x, napges %d\n",
					   __FUNCTION__, func_id, npages);
				golan_provide_pages(golan, npages, func_id);
			}
			break;
		default:
			printf("%s Unhandled event 0x%x on EQ 0x%x\n", __FUNCTION__,
				   eqe->type, eq->eqn);
			break;
		}

		++eq->cons_index;
		golan_eq_update_ci(eq, GOLAN_EQ_UNARMED);
		++counter;
	}
}

/**
 * Attach to multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 * @ret rc		Return status code
 */
static int golan_mcast_attach(struct ib_device *ibdev,
				struct ib_queue_pair *qp,
				union ib_gid *gid)
{
	struct golan *golan = ib_get_drvdata(ibdev);
	struct golan_cmd_layout	*cmd;
	int rc;

	if ( qp == NULL ) {
		DBGC( golan, "%s: Invalid pointer, could not attach QPN to MCG\n",
			__FUNCTION__ );
		return -EFAULT;
	}

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_ATTACH_TO_MCG, 0x0,
					GEN_MBOX, NO_MBOX,
					sizeof(struct golan_attach_mcg_mbox_in),
					sizeof(struct golan_attach_mcg_mbox_out));
	((struct golan_attach_mcg_mbox_in *)(cmd->in))->qpn = cpu_to_be32(qp->qpn);

	memcpy(GET_INBOX(golan, GEN_MBOX), gid, sizeof(*gid));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_CHECK_RC_AND_CMD_STATUS( err_attach_to_mcg_cmd );

	DBGC( golan , "%s: QPN 0x%lx was attached to MCG\n", __FUNCTION__, qp->qpn);
	return 0;
err_attach_to_mcg_cmd:
	printf ("%s [%d] out\n", __FUNCTION__, rc);
	return rc;
}

/**
 * Detach from multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 * @ret rc		Return status code
 */
static void golan_mcast_detach(struct ib_device *ibdev,
				struct ib_queue_pair *qp,
				union ib_gid *gid)
{
	struct golan *golan = ib_get_drvdata(ibdev);
	struct golan_cmd_layout	*cmd;
	int rc;

	cmd = write_cmd(golan, DEF_CMD_IDX, GOLAN_CMD_OP_DETACH_FROM_MCG, 0x0,
					GEN_MBOX, NO_MBOX,
				    sizeof(struct golan_detach_mcg_mbox_in),
				    sizeof(struct golan_detach_mcg_mbox_out));
	((struct golan_detach_mcg_mbox_in *)(cmd->in))->qpn = cpu_to_be32(qp->qpn);

	memcpy(GET_INBOX(golan, GEN_MBOX), gid, sizeof(*gid));

	rc = send_command_and_wait(golan, DEF_CMD_IDX, GEN_MBOX, NO_MBOX, __FUNCTION__);
	GOLAN_PRINT_RC_AND_CMD_STATUS;

	DBGC( golan , "%s: QPN 0x%lx was detached from MCG\n", __FUNCTION__, qp->qpn);
}

/**
 * Inform embedded subnet management agent of a received MAD
 *
 * @v ibdev		Infiniband device
 * @v mad		MAD
 * @ret rc		Return status code
 */
static int golan_inform_sma(struct ib_device *ibdev,
				union ib_mad *mad)
{
	if (!ibdev || !mad) {
		return 1;
	}

	return 0;
}

static int golan_register_ibdev(struct golan_port *port)
{
	struct ib_device *ibdev = port->ibdev;
	int rc;

	/* Initialise parameters using SMC */
	ib_smc_init(ibdev, golan_mad);
	/* Register Infiniband device */
	if ((rc = register_ibdev(ibdev)) != 0) {
		printf("%s port %d could not register IB device: (rc = %d)\n",
			__FUNCTION__, ibdev->port, rc);
		return rc;
	}

	port->netdev = ib_get_ownerdata ( ibdev );

	return 0;
}

static inline void golan_bring_down(struct golan *golan)
{
	DBGC(golan, "%s\n", __FUNCTION__);
	if (~golan->flags & GOLAN_OPEN)
		return;

	golan_destroy_mkey(golan);
	golan_dealloc_pd(golan);
	golan_destory_eq(golan);
	golan_dealloc_uar(golan);
	golan_teardown_hca(golan, GOLAN_TEARDOWN_GRACEFUL);
	golan_handle_pages(golan, GOLAN_REG_PAGES , GOLAN_PAGES_TAKE);
	golan_disable_hca(golan);
	golan_cmd_uninit(golan);
	golan->flags &= ~GOLAN_OPEN;
}

static inline int golan_bring_up(struct golan *golan)
{
	int rc = 0;
	DBGC(golan, "%s\n", __FUNCTION__);

	if (golan->flags & GOLAN_OPEN)
		return 0;

	if (( rc = golan_cmd_init(golan) ))
		goto out;

	if (( rc = golan_core_enable_hca(golan) ))
		goto cmd_uninit;

	/* Query for need for boot pages */
	if (( rc = golan_handle_pages(golan, GOLAN_BOOT_PAGES, GOLAN_PAGES_GIVE) ))
		goto disable;

	if (( rc = golan_qry_hca_cap(golan) ))
		goto pages;

	if (( rc = golan_set_hca_cap(golan) ))
		goto pages;

	if (( rc = golan_handle_pages(golan, GOLAN_INIT_PAGES, GOLAN_PAGES_GIVE) ))
		goto pages;
	//Reg Init?
	if (( rc = golan_hca_init(golan) ))
		goto pages_2;

	if (( rc = golan_alloc_uar(golan) ))
		goto teardown;

	if (( rc = golan_create_eq(golan) ))
		goto de_uar;

	if (( rc = golan_alloc_pd(golan) ))
		goto de_eq;

	if (( rc = golan_create_mkey(golan) ))
		goto de_pd;

	golan->flags |= GOLAN_OPEN;
	return 0;

	golan_destroy_mkey(golan);
de_pd:
	golan_dealloc_pd(golan);
de_eq:
	golan_destory_eq(golan);
de_uar:
	golan_dealloc_uar(golan);
teardown:
	golan_teardown_hca(golan, GOLAN_TEARDOWN_GRACEFUL);
pages_2:
	golan_handle_pages(golan, GOLAN_INIT_PAGES, GOLAN_PAGES_TAKE);
pages:
	golan_handle_pages(golan, GOLAN_BOOT_PAGES, GOLAN_PAGES_TAKE);
disable:
	golan_disable_hca(golan);
cmd_uninit:
	golan_cmd_uninit(golan);
out:
	return rc;
}

/**
 * Close Infiniband link
 *
 * @v ibdev		Infiniband device
 */
static void golan_ib_close ( struct ib_device *ibdev __unused ) {}

/**
 * Initialise Infiniband link
 *
 * @v ibdev		Infiniband device
 * @ret rc		Return status code
 */
static int golan_ib_open ( struct ib_device *ibdev ) {
	DBG ( "%s start\n", __FUNCTION__ );

	if ( ! ibdev )
		return -EINVAL;

	ib_smc_set_port_info (ibdev, golan_mad);
	ib_smc_update(ibdev, golan_mad);

	DBG ( "%s end\n", __FUNCTION__ );
	return 0;
}

/** Golan Infiniband operations */
static struct ib_device_operations golan_ib_operations = {
	.create_cq	= golan_create_cq,
	.destroy_cq	= golan_destroy_cq,
	.create_qp	= golan_create_qp,
	.modify_qp	= golan_modify_qp,
	.destroy_qp	= golan_destroy_qp,
	.post_send	= golan_post_send,
	.post_recv	= golan_post_recv,
	.poll_cq	= golan_poll_cq,
	.poll_eq	= golan_poll_eq,
	.open		= golan_ib_open,
	.close		= golan_ib_close,
	.mcast_attach	= golan_mcast_attach,
	.mcast_detach	= golan_mcast_detach,
	.set_port_info	= golan_inform_sma,
	.set_pkey_table	= golan_inform_sma,
};

static int golan_probe_normal ( struct pci_device *pci ) {
	struct golan *golan;
	struct ib_device *ibdev;
	struct golan_port *port;
	int i;
	int rc = 0;

	golan = golan_alloc();
	if ( !golan ) {
		rc = -ENOMEM;
		goto err_golan_alloc;
	}
	/* Setup PCI bus and HCA BAR */
	pci_set_drvdata( pci, golan );
	golan->pci = pci;
	golan_pci_init( golan );
	/* config command queues */
	if ( fw_ver_and_cmdif( golan ) ) {
		rc = -1;
		goto err_fw_ver_cmdif;
	}

	if ( golan_bring_up( golan ) ) {
		printf("golan bringup failed\n");
		rc = -1;
		goto err_golan_bringup;
	}

	/* Allocate Infiniband devices */
	for (i = 0; i < golan->caps.num_ports; ++i) {
		ibdev = alloc_ibdev( 0 );
		if ( !ibdev ) {
			rc = -ENOMEM;
			goto err_golan_probe_alloc_ibdev;
		}
		golan->ports[i].ibdev = ibdev;
		golan->ports[i].vep_number = 0;
		ibdev->op = &golan_ib_operations;
		ibdev->dev = &pci->dev;
		ibdev->port = (GOLAN_PORT_BASE + i);
		ib_set_drvdata( ibdev, golan );
	}

	/* Register devices */
	for ( i = 0; i < golan->caps.num_ports; ++i ) {
		port = &golan->ports[i];
		if ((rc = golan_register_ibdev ( port ) ) != 0 )
			goto err_golan_probe_register_ibdev;
		else
			DBGC( golan , "%s port %d was registered.\n", __FUNCTION__,
				i + GOLAN_PORT_BASE);
	}

	return 0;

	i = golan->caps.num_ports;
err_golan_probe_register_ibdev:
	for ( i-- ; ( signed int ) i >= 0 ; i-- )
		unregister_ibdev ( golan->ports[i].ibdev );

	i = golan->caps.num_ports;
err_golan_probe_alloc_ibdev:
	for ( i-- ; ( signed int ) i >= 0 ; i-- )
		ibdev_put ( golan->ports[i].ibdev );

	golan_bring_down ( golan );
err_golan_bringup:
err_fw_ver_cmdif:
	free ( golan );
err_golan_alloc:
	printf ("%s rc = %d\n", __FUNCTION__, rc);
	return rc;
}

static void golan_remove_normal ( struct pci_device *pci ) {
	struct golan	*golan = pci_get_drvdata(pci);
	struct golan_port *port;
	int i;

	DBGC(golan, "%s\n", __FUNCTION__);

	for ( i = ( golan->caps.num_ports - 1 ) ; i >= 0 ; i-- ) {
		port = &golan->ports[i];
		unregister_ibdev ( port->ibdev );
	}
	for ( i = ( golan->caps.num_ports - 1 ) ; i >= 0 ; i-- ) {
		netdev_nullify ( golan->ports[i].netdev );
		netdev_put ( golan->ports[i].netdev );
	}
	for ( i = ( golan->caps.num_ports - 1 ) ; i >= 0 ; i-- ) {
		ibdev_put ( golan->ports[i].ibdev );
	}

	golan_bring_down(golan);
	free(golan);
}

/***************************************************************************
 * NODNIC operations
 **************************************************************************/
static mlx_status shomron_fill_eth_send_wqe ( struct ib_device *ibdev,
			   struct ib_queue_pair *qp, struct ib_address_vector *av __unused,
			   struct io_buffer *iobuf, struct nodnic_send_wqbb *wqbb,
			   unsigned long wqe_index ) {
	mlx_status status = MLX_SUCCESS;
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct shomron_nodnic_eth_send_wqe *eth_wqe =  NULL;
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp =
			ib_qp_get_drvdata ( qp );
	nodnic_qp *nodnic_qp = flexboot_nodnic_qp->nodnic_queue_pair;
	struct nodnic_send_ring *send_ring = &nodnic_qp->send;
	mlx_uint32 qpn = 0;

	eth_wqe = (struct shomron_nodnic_eth_send_wqe *)wqbb;
	memset ( ( ( ( void * ) eth_wqe ) ), 0,
			   ( sizeof ( *eth_wqe ) ) );

	status = nodnic_port_get_qpn(&port->port_priv, &send_ring->nodnic_ring,
			&qpn);
	if ( status != MLX_SUCCESS ) {
		printf("nodnic_port_get_qpn failed\n");
		goto err;
	}

#define SHOMRON_GENERATE_CQE 0x3
#define SHOMRON_INLINE_HEADERS_SIZE 18
#define SHOMRON_INLINE_HEADERS_OFFSET 32
	MLX_FILL_2 ( &eth_wqe->ctrl, 0, opcode, FLEXBOOT_NODNIC_OPCODE_SEND,
			wqe_index, wqe_index & 0xFFFF);
	MLX_FILL_2 ( &eth_wqe->ctrl, 1, ds, 0x4 , qpn, qpn );
	MLX_FILL_1 ( &eth_wqe->ctrl, 2,
		     ce, SHOMRON_GENERATE_CQE /* generate completion */
			 );
	MLX_FILL_2 ( &eth_wqe->ctrl, 7,
			inline_headers1,
			cpu_to_be16(*(mlx_uint16 *)iobuf->data),
			inline_headers_size, SHOMRON_INLINE_HEADERS_SIZE
			 );
	memcpy((void *)&eth_wqe->ctrl + SHOMRON_INLINE_HEADERS_OFFSET,
			iobuf->data + 2, SHOMRON_INLINE_HEADERS_SIZE - 2);
	iob_pull(iobuf, SHOMRON_INLINE_HEADERS_SIZE);
	MLX_FILL_1 ( &eth_wqe->data[0], 0,
		     byte_count, iob_len ( iobuf ) );
	MLX_FILL_1 ( &eth_wqe->data[0], 1, l_key,
			flexboot_nodnic->device_priv.lkey );
	MLX_FILL_H ( &eth_wqe->data[0], 2,
		     local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &eth_wqe->data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );
err:
	return status;
}

static mlx_status shomron_fill_completion( void *cqe, struct cqe_data *cqe_data ) {
	union shomronprm_completion_entry *cq_entry;
	uint32_t opcode;

	cq_entry = (union shomronprm_completion_entry *)cqe;
	cqe_data->owner = MLX_GET ( &cq_entry->normal, owner );
	opcode = MLX_GET ( &cq_entry->normal, opcode );
#define FLEXBOOT_NODNIC_OPCODE_CQ_SEND 0
#define FLEXBOOT_NODNIC_OPCODE_CQ_RECV 2
#define FLEXBOOT_NODNIC_OPCODE_CQ_SEND_ERR 13
#define FLEXBOOT_NODNIC_OPCODE_CQ_RECV_ERR 14
	cqe_data->is_error =
			( opcode >= FLEXBOOT_NODNIC_OPCODE_CQ_RECV_ERR);
	if ( cqe_data->is_error ) {
		cqe_data->syndrome = MLX_GET ( &cq_entry->error, syndrome );
		cqe_data->vendor_err_syndrome =
				MLX_GET ( &cq_entry->error, vendor_error_syndrome );
		cqe_data->is_send =
					(opcode == FLEXBOOT_NODNIC_OPCODE_CQ_SEND_ERR);
	} else {
		cqe_data->is_send =
			(opcode == FLEXBOOT_NODNIC_OPCODE_CQ_SEND);
		cqe_data->wqe_counter = MLX_GET ( &cq_entry->normal, wqe_counter );
		cqe_data->byte_cnt = MLX_GET ( &cq_entry->normal, byte_cnt );

	}
	if ( cqe_data->is_send == TRUE )
		cqe_data->qpn = MLX_GET ( &cq_entry->normal, qpn );
	else
		cqe_data->qpn = MLX_GET ( &cq_entry->normal, srqn );

	return 0;
}

static mlx_status shomron_cqe_set_owner ( void *cq, unsigned int num_cqes ) {
	unsigned int i = 0;
	union shomronprm_completion_entry *cq_list;

	cq_list = (union shomronprm_completion_entry *)cq;
	for ( ; i < num_cqes ; i++ )
		MLX_FILL_1 ( &cq_list[i].normal, 15, owner, 1 );
	return 0;
}

static mlx_size shomron_get_cqe_size () {
	return sizeof ( union shomronprm_completion_entry );
}


static int shomron_flash_access_tlv ( mlx_utils *utils,
		struct driver_tlv_header *tlv_hdr, uint8_t access_method )
{
	uint8_t version = ( uint8_t ) tlv_hdr->version;
	int rc;

	rc = nvconfig_nvdata_access ( utils,  tlv_hdr->type_mod, tlv_hdr->type
			,access_method , tlv_hdr->length, & ( version ), tlv_hdr->data );
	tlv_hdr->version =  ( uint32_t ) version;
	if ( rc ) {
		DBG ( "Failed to %s TLV (rc = %d)\n",
			((access_method == REG_ACCESS_READ) ? "read" : "write" ), rc );
	}

	return rc;
}

static int shomron_flash_write_tlv ( mlx_utils *utils,
		struct driver_tlv_header *tlv_hdr ) {
	return shomron_flash_access_tlv ( utils, tlv_hdr, REG_ACCESS_WRITE );
}

static int shomron_flash_read_tlv ( mlx_utils *utils,
		struct driver_tlv_header *tlv_hdr ) {
	int rc = shomron_flash_access_tlv ( utils, tlv_hdr, REG_ACCESS_READ );
	if ( rc == 0 )
		swab_settings_bits ( tlv_hdr->type, tlv_hdr->data, tlv_hdr->length );
	return rc;
}

static int shomron_flash_invalidate_tlv ( mlx_utils *utils,
		struct driver_tlv_header *tlv_hdr ) {
	return nvconfig_nvdata_invalidate ( utils, tlv_hdr->type_mod,
				tlv_hdr->type );
}

struct flexboot_nodnic_callbacks shomron_nodnic_callbacks = {
	.get_cqe_size = shomron_get_cqe_size,
	.fill_send_wqe[IB_QPT_ETH] = shomron_fill_eth_send_wqe,
	.fill_completion = shomron_fill_completion,
	.cqe_set_owner = shomron_cqe_set_owner,
	.tlv_write_fn = shomron_flash_write_tlv,
	.tlv_read_fn = shomron_flash_read_tlv,
	.tlv_invaidate_fn = shomron_flash_invalidate_tlv,
	.update_settings_ops = golan_update_setting_ops,
};

static int shomron_nodnic_supported = 0;

static int shomron_nodnic_is_supported ( struct pci_device *pci ) {
	if ( pci->device == 0x1011 )
		return 0;

	return flexboot_nodnic_is_supported ( pci );
}
/**************************************************************************/

static int golan_probe ( struct pci_device *pci ) {
	int rc = -ENOTSUP;

	DBG ( "%s: start\n", __FUNCTION__ );

	if ( ! pci ) {
		printf ( "%s: PCI is NULL\n", __FUNCTION__ );
		rc = -EINVAL;
		goto probe_done;
	}

	shomron_nodnic_supported = shomron_nodnic_is_supported ( pci );
	if ( boot_post_shell ) {
		if ( ! shomron_nodnic_supported ) {
			rc = 0;
			goto probe_done;
		}
	}

	if ( shomron_nodnic_supported ) {
		rc = flexboot_nodnic_probe ( pci, &shomron_nodnic_callbacks, NULL );
		if ( rc == 0 ) {
			DBG ( "%s: Using NODNIC driver\n", __FUNCTION__ );
			goto probe_done;
		}
		shomron_nodnic_supported = 0;
	}

	if ( ! boot_post_shell && ! shomron_nodnic_supported ) {
		DBG ( "%s: Using normal driver\n", __FUNCTION__ );
		rc = golan_probe_normal ( pci );
	}

probe_done:
	DBG ( "%s: rc = %d\n", __FUNCTION__, rc );
	return rc;
}

static void golan_remove ( struct pci_device *pci ) {
	DBG ( "%s: start\n", __FUNCTION__ );

	if ( boot_post_shell ) {
		if ( ! shomron_nodnic_supported )
			return;
	}

	if ( ! shomron_nodnic_supported ) {
		DBG ( "%s: Using normal driver remove\n", __FUNCTION__ );
		golan_remove_normal ( pci );
		return;
	}

	DBG ( "%s: Using NODNIC driver remove\n", __FUNCTION__ );

	flexboot_nodnic_remove ( pci );

	DBG ( "%s: end\n", __FUNCTION__ );
}

static struct pci_device_id golan_nics[] = {
	PCI_ROM ( 0x15b3, 0x1011, "ConnectIB", "ConnectIB HCA driver: DevID 4113", 0 ),
	PCI_ROM ( 0x15b3, 0x1013, "ConnectX-4", "ConnectX-4 HCA driver, DevID 4115", 0 ),
	PCI_ROM ( 0x15b3, 0x1015, "ConnectX-4Lx", "ConnectX-4Lx HCA driver, DevID 4117", 0 ),
};

struct pci_driver golan_driver __pci_driver = {
		.ids		= golan_nics,
		.id_count	= (sizeof(golan_nics) / sizeof(golan_nics[0])),
		.probe		= golan_probe,
		.remove		= golan_remove,
};
