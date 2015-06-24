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

#ifndef SRC_DRIVERS_INFINIBAND_MLX_NODNIC_INCLUDE_PRM_NODNIC_HERMON_PRM_H_
#define SRC_DRIVERS_INFINIBAND_MLX_NODNIC_INCLUDE_PRM_NODNIC_HERMON_PRM_H_

#include "nodnic_prm.h"

#define HERMON_MAX_GATHER 2

/* Send wqe segment ctrl */

struct hermonprm_wqe_segment_ctrl_send_st {	/* Little Endian */
    pseudo_bit_t	opcode[0x00005];
    pseudo_bit_t	reserved0[0x0001a];
    pseudo_bit_t	owner[0x00001];
/* -------------- */
    pseudo_bit_t	ds[0x00006];           /* descriptor (wqe) size in 16bytes chunk */
    pseudo_bit_t	f[0x00001];            /* fence */
    pseudo_bit_t	reserved1[0x00019];
/* -------------- */
    pseudo_bit_t	fl[0x00001];           /* Force LoopBack */
    pseudo_bit_t	s[0x00001];            /* Remote Solicited Event */
    pseudo_bit_t	c[0x00002];            /* completion required: 0b00 - no   0b11 - yes */
    pseudo_bit_t	ip[0x00001];           /* When set, InfiniHost III Ex will calculate the IP checksum of the IP header that is present immediately after the IPoverIB encapsulation header. In the case of multiple headers (encapsulation), InfiniHost III Ex will calculate the checksum only for the first IP header following the IPoverIB encapsulation header. Not Valid for IPv6 packets */
    pseudo_bit_t	tcp_udp[0x00001];      /* When set, InfiniHost III Ex will calculate the TCP/UDP checksum of the packet that is present immediately after the IP header. In the case of multiple headers (encapsulation), InfiniHost III Ex will calculate the checksum only for the first TCP header following the IP header. This bit may be set only if the entire TCP/UDP segment is present in one IB packet */
    pseudo_bit_t	reserved2[0x00001];
    pseudo_bit_t	so[0x00001];           /* Strong Ordering - when set, the WQE will be executed only after all previous WQEs have been executed. Can be set for RC WQEs only. This bit must be set in type two BIND, Fast Registration and Local invalidate operations. */
    pseudo_bit_t	src_remote_buf[0x00018];
/* -------------- */
    pseudo_bit_t	immediate[0x00020];    /* If the OpCode encodes an operation with Immediate (RDMA-write/SEND), This field will hold the Immediate data to be sent. If the OpCode encodes send and invalidate operations, this field holds the Invalidation key to be inserted into the packet; otherwise, this field is reserved. */
/* -------------- */
};



/* Completion Queue Entry Format        #### michal - fixed by gdror */

struct hermonprm_completion_queue_entry_st {	/* Little Endian */
    pseudo_bit_t	qpn[0x00018];          /* Indicates the QP for which completion is being reported */
    pseudo_bit_t	reserved0[0x00002];
    pseudo_bit_t	d2s[0x00001];          /* Duplicate to Sniffer. This bit is set if both Send and Receive queues are subject for sniffer queue. The HW delivers
                                                 packet only to send-associated sniffer receive queue. */
    pseudo_bit_t	fcrc_sd[0x00001];      /* FCRC: If set, FC CRC is correct in FC frame encapsulated in payload. Valid for Raw Frame FC receive queue only.
                                                 SD: CQ associated with Sniffer receive queue. If set, packets were skipped due to lack of receive buffers on the Sniffer receive queue */
    pseudo_bit_t	fl[0x00001];           /* Force Loopback Valid for responder RawEth and UD only. */
    pseudo_bit_t	vlan[0x00002];         /* Valid for RawEth and UD over Ethernet only. Applicable for RawEth and UD over Ethernet Receive queue
                                                  00 - No VLAN header was present in the packet
                                                 01 - C-VLAN (802.1q) Header was present in the frame.
                                                 10 - S-VLAN (802.1ad) Header was present in the frame. */
    pseudo_bit_t	dife[0x00001];         /* DIF Error */
/* -------------- */
    pseudo_bit_t	immediate_rssvalue_invalidatekey[0x00020];/* For a responder CQE, if completed WQE Opcode is Send With Immediate or Write With Immediate, this field contains immediate field of the received message.
                                                 For a responder CQE, if completed WQE Opcode is Send With Invalidate, this field contains the R_key that was invalidated.
                                                 For a responder CQE of a GSI packet this filed contains the Pkey Index of the packet.
                                                 For IPoIB (UD) and RawEth CQEs this field contains the RSS hash function value.
                                                 Otherwise, this field is reserved. */
/* -------------- */
    pseudo_bit_t	srq_rqpn[0x00018];     /* For Responder UD QPs, Remote (source) QP number.
                                                 For Responder SRC QPs, SRQ number.
                                                 Otherwise, this field is reserved. */
    pseudo_bit_t	ml_path_mac_index[0x00007];/* For responder UD over IB CQE: These are the lower LMC bits of the DLID in an incoming UD packet, higher bits of this field, that are not part of the LMC bits are zeroed by HW. Invalid if incoming message DLID is the permissive LID or incoming message is multicast.
                                                  For responder UD over Ethernet and RawEth CQEs: Index of the MAC Table entry that the packet DMAC was matched against.
                                                  Otherwise, this field is reserved. */
    pseudo_bit_t	g[0x00001];            /* For responder UD over IB CQE this bit indicates the presence of a GRH
                                                 For responder UD over Ethernet CQE this bit is set if IPv6 L3 header was present in the packet, this bit is cleared if IPv4 L3 Header was present in the packet.
                                                 Otherwise, this field is reserved. */
/* -------------- */
    pseudo_bit_t	slid_smac47_32[0x00010];/* For responder UD over IB CQE it is the source LID of the packet.
                                                 For responder UD over Ethernet and RawEth CQEs it is the source-MAC[47:32] of the packet.
                                                 Otherwise, this field is reserved. */
    pseudo_bit_t	vid[0x0000c];          /* Frame VID, valid for Responder Raw Ethernet and UD over Ethernet QP. Otherwise, this field is reserved. */
    pseudo_bit_t	sl[0x00004];           /* For responder UD over IB - the Service Level of the packet.
                                                  For responder UD over Ethernet and RawEth - it is VLAN-header[15:12]
                                                  Otherwise, this field is reserved. */
/* -------------- */
    pseudo_bit_t	smac31_0_rawether_ipoib_status[0x00020];/* For responder UD over Ethernet - source MAC[31:0] of the packet.
                                                  For responder RawEth and UD over IB - RawEth-IPoIB status {3 reserved, ipok,udp,tcp,ipv4opt,ipv6,ipv4vf,ipv4,rht(6),ipv6extmask(6),reserved(2),l2am,reserved(2),bfcs,reserved(2),enc}
                                                  Otherwise, this field is reserved. */
/* -------------- */
    pseudo_bit_t	byte_cnt[0x00020];     /* Byte count of data transferred. Applicable for RDMA-read, Atomic and all receive operations. completions.
                                                 For Receive Queue that is subject for headers. separation, byte_cnt[31:24] specify number of bytes scattered to the first scatter entry (headers. length). Byte_cnt[23:0] specify total byte count received (including headers). */
/* -------------- */
    pseudo_bit_t	checksum[0x00010];     /* Valid for RawEth and IPoIB only. */
    pseudo_bit_t	wqe_counter[0x00010];
/* -------------- */
    pseudo_bit_t	opcode[0x00005];       /* Send completions - same encoding as WQE.
                                                  Error coding is 0x1F
                                                  Receive:
                                                  0x0 - RDMA-Write with Immediate
                                                  0x1 - Send
                                                  0x2 - Send with Immediate
                                                  0x3 - Send & Invalidate
                                                  */
    pseudo_bit_t	is[0x00001];           /* inline scatter */
    pseudo_bit_t	s_r[0x00001];          /* send 1 / receive 0 */
    pseudo_bit_t	owner[0x00001];        /* HW Flips this bit for every CQ warp around. Initialized to Zero. */
    pseudo_bit_t	reserved1[0x00010];
    pseudo_bit_t	reserved2[0x00008];
/* -------------- */
};


/* Completion with Error CQE             #### michal - gdror fixed */

struct hermonprm_completion_with_error_st {	/* Little Endian */
    pseudo_bit_t	qpn[0x00018];          /* Indicates the QP for which completion is being reported */
    pseudo_bit_t	reserved0[0x00008];
/* -------------- */
    pseudo_bit_t	reserved1[0x000a0];
/* -------------- */
    pseudo_bit_t	syndrome[0x00008];     /* Completion with error syndrome:
                                                         0x01 - Local Length Error
                                                         0x02 - Local QP Operation Error
                                                         0x03 - Local EE Context Operation Error
                                                         0x04 - Local Protection Error
                                                         0x05 - Work Request Flushed Error
                                                         0x06 - Memory Window Bind Error
                                                         0x10 - Bad Response Error
                                                         0x11 - Local Access Error
                                                         0x12 - Remote Invalid Request Error
                                                         0x13 - Remote Access Error
                                                         0x14 - Remote Operation Error
                                                         0x15 - Transport Retry Counter Exceeded
                                                         0x16 - RNR Retry Counter Exceeded
                                                         0x20 - Local RDD Violation Error
                                                         0x21 - Remote Invalid RD Request
                                                         0x22 - Remote Aborted Error
                                                         0x23 - Invalid EE Context Number
                                                         0x24 - Invalid EE Context State
                                                         other - Reserved
                                                 Syndrome is defined according to the IB specification volume 1. For detailed explanation of the syndromes, refer to chapters 10-11 of the IB specification rev 1.1. */
    pseudo_bit_t	vendor_error_syndrome[0x00008];
    pseudo_bit_t	wqe_counter[0x00010];
/* -------------- */
    pseudo_bit_t	opcode[0x00005];       /* The opcode of WQE completion is reported for.

                                                 The following values are reported in case of completion with error:
                                                 0xFE - For completion with error on Receive Queues
                                                 0xFF - For completion with error on Send Queues */
    pseudo_bit_t	reserved2[0x00001];
    pseudo_bit_t	s_r[0x00001];          /* send 1 / receive 0 */
    pseudo_bit_t	owner[0x00001];        /* HW Flips this bit for every CQ warp around. Initialized to Zero. */
    pseudo_bit_t	reserved3[0x00018];
/* -------------- */
};


struct MLX_DECLARE_STRUCT ( hermonprm_wqe_segment_ctrl_send );
struct MLX_DECLARE_STRUCT ( hermonprm_completion_queue_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_completion_with_error );

struct hermon_nodnic_eth_send_wqe {
	struct hermonprm_wqe_segment_ctrl_send ctrl;
	struct nodnic_wqe_segment_data_ptr data[HERMON_MAX_GATHER];
} __attribute__ (( packed ));

union hermonprm_completion_entry {
	struct hermonprm_completion_queue_entry normal;
	struct hermonprm_completion_with_error error;
} __attribute__ (( packed ));

#if 0
/* ConnetX3 eth wqe */
struct hermon_nodnic_eth_send_wqe {
	/* Ctrl */
	/* offset 0x0 */
	mlx_uint32 opcode		: 5;
	mlx_uint32 reserv1		: 26;
	mlx_uint32 owner		: 1;
	/* offset 0x4 */
	mlx_uint32 ds			: 6;
	mlx_uint32 reserv2		: 26;
	/* offset 0x8 */
	mlx_uint32 icrc			: 1;
	mlx_uint32 c			: 2;
	mlx_uint32 reserv3		: 13;
	mlx_uint32 dmac_h		: 16;
	/* offset 0xc */
	mlx_uint32 dmac_l		: 32;
	/* Data segment*/
	/* offset 0x10 */
	mlx_uint32 byte_count	: 31;
	mlx_uint32 reserv10		: 1;
	/* offset 0x14*/
	mlx_uint32 l_key		: 32;
	/* offset 0x18*/
	mlx_uint32 local_address_h	: 32;
	/* offset 0x1c*/
	mlx_uint32 local_address_l	: 32;
};


/* ConnectX3 cqe */
struct hermon_nodnic_completion_queue_entry {
	/*0x0*/
	mlx_uint32 qpn					: 24;
	mlx_uint32 reserve1				: 8;
	/*0x4*/
	mlx_uint32 pkey_index			: 32;
	/*0x8*/
	mlx_uint32 rqpn					: 24;
	mlx_uint32 reserve2				: 7;
	mlx_uint32 g					: 1;
	/*0xc*/
	mlx_uint32 slid					: 16;
	mlx_uint32 reserve3				: 12;
	mlx_uint32 sl					: 4;
	/*0x10*/
	mlx_uint32 reserve4				: 32;
	/*0x14*/
	mlx_uint32 byte_cnt				: 32;
	/*0x18*/
	mlx_uint32 reserve5				: 16;
	mlx_uint32 wqe_counter			: 16;
	/*0x1c*/
	mlx_uint32 opcode				: 5;
	mlx_uint32 reserve6				: 1;
	mlx_uint32 s_r					: 1;
	mlx_uint32 owner				: 1;
	mlx_uint32 reserve7			: 24;
};

/* ConnectX3 cqe with error */
struct hermon_nodnic_completion_queue_err_entry {
	/*0x0*/
	mlx_uint32 qpn					: 24;
	mlx_uint32 reserve1				: 8;
	/*0x4 - 0x14*/
	mlx_uint32 reserve2[5];
	/*0x18*/
	mlx_uint32 syndrome				: 8;
	mlx_uint32 vendor_err_syndrome	: 8;
	mlx_uint32 wqe_counter			: 16;
	/*0x1c*/
	mlx_uint32 opcode				: 5;
	mlx_uint32 reserve3				: 1;
	mlx_uint32 s_r					: 1;
	mlx_uint32 owner				: 1;
	mlx_uint32 reserve4				: 24;
};
#endif

#endif /* SRC_DRIVERS_INFINIBAND_MLX_NODNIC_INCLUDE_PRM_NODNIC_HERMON_PRM_H_ */
