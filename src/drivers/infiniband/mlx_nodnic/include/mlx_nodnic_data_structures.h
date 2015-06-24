#ifndef NODNIC_NODNICDATASTRUCTURES_H_
#define NODNIC_NODNICDATASTRUCTURES_H_

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

#include "mlx_utils.h"
#include "prm/nodnic_prm.h"

/* todo: fix coding convention */
#define NODNIC_MEMORY_ALIGN		0x1000

#define NODNIC_MAX_MAC_FILTERS 5
#define NODNIC_MAX_MGID_FILTERS 4

typedef struct _nodnic_device_priv 				nodnic_device_priv;
typedef struct _nodnic_port_priv 				nodnic_port_priv;
typedef struct _nodnic_device_capabilites 		nodnic_device_capabilites;
typedef struct _nodnic_qp 						nodnic_qp;
typedef struct _nodnic_cq 						nodnic_cq;
typedef struct _nodnic_eq						nodnic_eq;

/* NODNIC Port states
 * Bit 0 - port open/close
 * Bit 1 - port is [not] in disabling DMA
 * 0 - closed and not disabling DMA
 * 1 - opened and not disabling DMA
 * 3 - opened and disabling DMA
 */
#define NODNIC_PORT_OPENED			0b00000001
#define NODNIC_PORT_DISABLING_DMA	0b00000010

typedef enum {
	ConnectX3 = 0,
	Connectx4
}nodnic_hardware_format;


typedef enum {
	NODNIC_QPT_SMI,
	NODNIC_QPT_GSI,
	NODNIC_QPT_UD,
	NODNIC_QPT_RC,
	NODNIC_QPT_ETH,
}nodnic_queue_pair_type;
typedef enum {
	NODNIC_PORT_TYPE_IB = 0,
	NODNIC_PORT_TYPE_ETH,
	NODNIC_PORT_TYPE_UNKNOWN,
}nodnic_port_type;



#define NODNIC_WQBB_SIZE 64
/** A nodnic send wqbb */
struct nodnic_send_wqbb {
	mlx_uint8 force_align[NODNIC_WQBB_SIZE];
};
struct nodnic_ring {
	mlx_uint32 offset;
	/** Work queue entries */
	/* TODO: add to memory entity */
	mlx_physical_address wqe_physical;
	mlx_void *map;
	/** Size of work queue */
	mlx_size wq_size;
	/** Next work queue entry index
	 *
	 * This is the index of the next entry to be filled (i.e. the
	 * first empty entry).  This value is not bounded by num_wqes;
	 * users must logical-AND with (num_wqes-1) to generate an
	 * array index.
	 */
	mlx_uint32 num_wqes;
	mlx_uint32 qpn;
	mlx_uint32 next_idx;
	mlx_uint32	ring_pi;
};

struct nodnic_send_ring{
	struct nodnic_ring nodnic_ring;
	struct nodnic_send_wqbb *wqe_virt;
};


struct nodnic_recv_ring{
	struct nodnic_ring nodnic_ring;
	struct nodnic_recv_wqe *wqe_virt;
};
struct _nodnic_qp{
	nodnic_queue_pair_type	type;
	struct nodnic_send_ring		send;
	struct nodnic_recv_ring		receive;
};

struct _nodnic_cq{
	/** cq entries */
	mlx_void *cq_virt;
	mlx_physical_address cq_physical;
	mlx_void *map;
	/** cq */
	mlx_size cq_size;
};

struct _nodnic_eq{
	mlx_void *eq_virt;
	mlx_physical_address eq_physical;
	mlx_void *map;
	mlx_size eq_size;
};
struct _nodnic_device_capabilites{
	mlx_boolean					support_mac_filters;
	mlx_boolean					support_promisc_filter;
	mlx_boolean					support_promisc_multicast_filter;
	mlx_uint8					log_working_buffer_size;
	mlx_uint8					log_pkey_table_size;
	mlx_boolean					num_ports; // 0 - single port, 1 - dual port
	mlx_uint8					log_max_ring_size;
};

struct _nodnic_device_priv{
	mlx_boolean					is_initiailzied;
	mlx_utils					*utils;

	//nodnic structure offset in init segment
	mlx_uint32					device_offset;

	nodnic_device_capabilites	device_cap;

	mlx_uint8					nodnic_revision;
	nodnic_hardware_format		hardware_format;
	mlx_uint32					pd;
	mlx_uint32					lkey;
	mlx_uint64					device_guid;
	nodnic_port_priv			*ports;
};

struct _nodnic_port_priv{
	nodnic_device_priv		*device;
	mlx_uint32				port_offset;
	mlx_uint8				port_state;
	mlx_boolean				network_state;
	mlx_boolean				dma_state;
	nodnic_port_type		port_type;
	mlx_uint8				port_num;
	nodnic_eq				eq;
	mlx_mac_address			mac_filters[5];
};


#endif /* STUB_NODNIC_NODNICDATASTRUCTURES_H_ */
