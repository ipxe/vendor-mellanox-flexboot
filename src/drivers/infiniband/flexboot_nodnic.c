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

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <ipxe/pci.h>
#include <ipxe/malloc.h>
#include <ipxe/umalloc.h>
#include <ipxe/if_ether.h>
#include <ipxe/ethernet.h>
#include <ipxe/vlan.h>
#include <ipxe/io.h>
#include "flexboot_nodnic.h"
#include "mlx_types.h"
#include "mlx_utils.h"
#include "mlx_bail.h"
#include "mlx_cmd.h"
#include "mlx_memory.h"
#include "mlx_pci.h"
#include "mlx_device.h"
#include "mlx_port.h"
#include <byteswap.h>
#include <ipxe/status_updater.h>
#include <usr/ifmgmt.h>
#include <mlx_nvconfig.h>
#include <mlx_nvconfig_defaults.h>
#include <mlx_pci_gw.h>
#include <mlx_vmac.h>
#include <ipxe/boot_menu_ui.h>
#include "flex_debug_log.h"

/***************************************************************************
 *
 * Completion queue operations
 *
 ***************************************************************************
 */
static int flexboot_nodnic_arm_cq ( struct flexboot_nodnic_port *port ) {
#ifndef DEVICE_CX3
	mlx_uint32 val = ( port->eth_cq->next_idx & 0xffffff );
	if ( nodnic_port_set ( & port->port_priv, nodnic_port_option_arm_cq, val ) ) {
		MLX_DEBUG_ERROR( port->port_priv.device, "Failed to arm the CQ\n" );
		return MLX_FAILED;
	}
#else
	mlx_utils *utils = port->port_priv.device->utils;
	nodnic_port_data_flow_gw *ptr = port->port_priv.data_flow_gw;
	mlx_uint32 data = 0;
	mlx_uint32 val = 0;

	if ( port->port_priv.device->device_cap.crspace_doorbells == 0 ) {
		val = ( port->eth_cq->next_idx & 0xffff );
		if ( nodnic_port_set ( & port->port_priv, nodnic_port_option_arm_cq, val ) ) {
			MLX_DEBUG_ERROR( port->port_priv.device, "Failed to arm the CQ\n" );
			return MLX_FAILED;
		}
	} else {
		/* Arming the CQ with CQ CI should be with this format -
		 * 16 bit - CQ CI - same endianness as the FW (don't swap bytes)
		 * 15 bit - reserved
		 *  1 bit - arm CQ - must correct the endianness with the reserved above */
		data = ( ( ( port->eth_cq->next_idx & 0xffff ) << 16 ) | 0x0080 );
		/* Write the new index and update FW that new data was submitted */
		mlx_pci_mem_write ( utils, MlxPciWidthUint32, 0,
				( mlx_uint64 ) & ( ptr->armcq_cq_ci_dword ), 1, &data );
	}
#endif
	return 0;
}

/**
 * Create completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @ret rc		Return status code
 */
static int flexboot_nodnic_create_cq ( struct ib_device *ibdev ,
			      struct ib_completion_queue *cq ) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct flexboot_nodnic_completion_queue *flexboot_nodnic_cq;
	mlx_status status = MLX_SUCCESS;

	flexboot_nodnic_cq = (struct flexboot_nodnic_completion_queue *)
			zalloc(sizeof(*flexboot_nodnic_cq));
	if ( flexboot_nodnic_cq == NULL ) {
		status = MLX_OUT_OF_RESOURCES;
		goto qp_alloc_err;
	}

	status = nodnic_port_create_cq(&port->port_priv,
			cq->num_cqes *
			flexboot_nodnic->callbacks->get_cqe_size(),
			&flexboot_nodnic_cq->nodnic_completion_queue
			);
	MLX_FATAL_CHECK_STATUS(status, create_err,
				"nodnic_port_create_cq failed");
	flexboot_nodnic->callbacks->cqe_set_owner(
			flexboot_nodnic_cq->nodnic_completion_queue->cq_virt,
			cq->num_cqes);


	ib_cq_set_drvdata ( cq, flexboot_nodnic_cq );
	return status;
create_err:
	free(flexboot_nodnic_cq);
qp_alloc_err:
	return status;
}

/**
 * Destroy completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void flexboot_nodnic_destroy_cq ( struct ib_device *ibdev ,
				struct ib_completion_queue *cq ) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct flexboot_nodnic_completion_queue *flexboot_nodnic_cq = ib_cq_get_drvdata ( cq );

	nodnic_port_destroy_cq(&port->port_priv,
			flexboot_nodnic_cq->nodnic_completion_queue);

	free(flexboot_nodnic_cq);
}

static
struct ib_work_queue * flexboot_nodnic_find_wq ( struct ib_device *ibdev ,
							struct ib_completion_queue *cq,
							unsigned long qpn, int is_send ) {
	struct ib_work_queue *wq;
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp;
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct nodnic_ring  *ring;
	mlx_uint32 out_qpn;
	list_for_each_entry ( wq, &cq->work_queues, list ) {
		flexboot_nodnic_qp = ib_qp_get_drvdata ( wq->qp );
		if( wq->is_send == is_send && wq->is_send == TRUE ) {
			ring = &flexboot_nodnic_qp->nodnic_queue_pair->send.nodnic_ring;
		} else if( wq->is_send == is_send && wq->is_send == FALSE ) {
			ring = &flexboot_nodnic_qp->nodnic_queue_pair->receive.nodnic_ring;
		} else {
			continue;
		}
		nodnic_port_get_qpn(&port->port_priv, ring, &out_qpn);
		if ( out_qpn == qpn )
			return wq;
	}
	return NULL;
}

/**
 * Handle completion
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @v cqe		Hardware completion queue entry
 * @ret rc		Return status code
 */
static int flexboot_nodnic_complete ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq,
				 struct cqe_data *cqe_data ) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct ib_work_queue *wq;
	struct ib_queue_pair *qp;
	struct io_buffer *iobuf;
	struct ib_address_vector recv_dest;
	struct ib_address_vector recv_source;
	unsigned long qpn;
	unsigned long wqe_idx;
	unsigned long wqe_idx_mask;
	size_t len;
	int rc = 0;

	/* Parse completion */
	qpn = cqe_data->qpn;

	if ( cqe_data->is_error == TRUE ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p CQN %#lx syndrome %x vendor %x\n",
				flexboot_nodnic, cq->cqn, cqe_data->syndrome,
				cqe_data->vendor_err_syndrome );
		rc = -EIO;
		/* Don't return immediately; propagate error to completer */
	}

	/* Identify work queue */
	wq = flexboot_nodnic_find_wq( ibdev, cq, qpn, cqe_data->is_send );
	if ( wq == NULL ) {
		DBGC_NODNIC ( flexboot_nodnic,
				"flexboot_nodnic %p CQN %#lx unknown %s QPN %#lx\n",
				flexboot_nodnic, cq->cqn,
				( cqe_data->is_send ? "send" : "recv" ), qpn );
		return -EIO;
	}
	qp = wq->qp;

	/* Identify work queue entry */
	wqe_idx = cqe_data->wqe_counter;
	wqe_idx_mask = ( wq->num_wqes - 1 );
	DBGCP_NODNIC ( flexboot_nodnic,
			"NODNIC %p CQN %#lx QPN %#lx %s WQE %#lx completed:\n",
			flexboot_nodnic, cq->cqn, qp->qpn,
			( cqe_data->is_send ? "send" : "recv" ),
		wqe_idx );

	/* Identify I/O buffer */
	iobuf = wq->iobufs[wqe_idx & wqe_idx_mask];
	if ( iobuf == NULL ) {
		DBGC_NODNIC ( flexboot_nodnic,
				"NODNIC %p CQN %#lx QPN %#lx empty %s WQE %#lx\n",
				flexboot_nodnic, cq->cqn, qp->qpn,
		       ( cqe_data->is_send ? "send" : "recv" ), wqe_idx );
		return -EIO;
	}
	wq->iobufs[wqe_idx & wqe_idx_mask] = NULL;

	if ( cqe_data->is_send == TRUE ) {
		/* Hand off to completion handler */
		ib_complete_send ( ibdev, qp, iobuf, rc );
	} else if ( rc != 0 ) {
		/* Propagate error to receive completion handler */
		ib_complete_recv ( ibdev, qp, NULL, NULL, iobuf, rc );
	} else {
		/* Set received length */
		len = cqe_data->byte_cnt;
		assert ( len <= iob_tailroom ( iobuf ) );
		iob_put ( iobuf, len );
		memset ( &recv_dest, 0, sizeof ( recv_dest ) );
		recv_dest.qpn = qpn;
		memset ( &recv_source, 0, sizeof ( recv_source ) );
		switch ( qp->type ) {
		case IB_QPT_SMI:
		case IB_QPT_GSI:
		case IB_QPT_UD:
		case IB_QPT_RC:
			break;
		case IB_QPT_ETH:
			break;
		default:
			assert ( 0 );
			return -EINVAL;
		}
		/* Hand off to completion handler */
		ib_complete_recv ( ibdev, qp, &recv_dest,
				&recv_source, iobuf, rc );
	}

	return rc;
}
/**
 * Poll completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queues
 */
static void flexboot_nodnic_poll_cq ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_completion_queue *flexboot_nodnic_cq = ib_cq_get_drvdata ( cq );
	void *cqe;
	mlx_size cqe_size;
	struct cqe_data cqe_data;
	unsigned int cqe_idx_mask;
	int rc;

	cqe_size = flexboot_nodnic->callbacks->get_cqe_size();
	while ( TRUE ) {
		/* Look for completion entry */
		cqe_idx_mask = ( cq->num_cqes - 1 );
		cqe = ((uint8_t *)flexboot_nodnic_cq->nodnic_completion_queue->cq_virt) +
				cqe_size * (cq->next_idx & cqe_idx_mask);

		/* TODO: check fill_completion */
		flexboot_nodnic->callbacks->fill_completion(cqe, &cqe_data);
		if ( cqe_data.owner ^
				( ( cq->next_idx & cq->num_cqes ) ? 1 : 0 ) ) {
			/* Entry still owned by hardware; end of poll */
			break;
		}
		/* Handle completion */
		rc = flexboot_nodnic_complete ( ibdev, cq, &cqe_data );
		if ( rc != 0 ) {
			DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p CQN %#lx failed to complete: %s\n",
					flexboot_nodnic, cq->cqn, strerror ( rc ) );
			DBGC_HDA_NODNIC ( flexboot_nodnic, virt_to_phys ( cqe ),
				   cqe, sizeof ( *cqe ) );
		}

		/* Update completion queue's index */
		cq->next_idx++;
	}
}
/***************************************************************************
 *
 * Queue pair operations
 *
 ***************************************************************************
 */


/**
 * Create queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int flexboot_nodnic_create_qp ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp ) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp;
	mlx_status status = MLX_SUCCESS;

	flexboot_nodnic_qp = (struct flexboot_nodnic_queue_pair *)zalloc(sizeof(*flexboot_nodnic_qp));
	if ( flexboot_nodnic_qp == NULL ) {
		status = MLX_OUT_OF_RESOURCES;
		goto qp_alloc_err;
	}

	status = nodnic_port_create_qp(&port->port_priv, qp->type,
			qp->send.num_wqes * sizeof(struct nodnic_send_wqbb),
			qp->send.num_wqes,
			qp->recv.num_wqes * sizeof(struct nodnic_recv_wqe),
			qp->recv.num_wqes,
			&flexboot_nodnic_qp->nodnic_queue_pair);
	MLX_FATAL_CHECK_STATUS(status, create_err,
			"nodnic_port_create_qp failed");
	ib_qp_set_drvdata ( qp, flexboot_nodnic_qp );
	return status;
create_err:
	free(flexboot_nodnic_qp);
qp_alloc_err:
	return status;
}

/**
 * Modify queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int flexboot_nodnic_modify_qp ( struct ib_device *ibdev __unused,
			      struct ib_queue_pair *qp __unused) {
	/*not needed*/
	return 0;
}

/**
 * Destroy queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void flexboot_nodnic_destroy_qp ( struct ib_device *ibdev,
				struct ib_queue_pair *qp ) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp = ib_qp_get_drvdata ( qp );

	nodnic_port_destroy_qp(&port->port_priv, qp->type,
			flexboot_nodnic_qp->nodnic_queue_pair);

	free(flexboot_nodnic_qp);
}

/***************************************************************************
 *
 * Work request operations
 *
 ***************************************************************************
 */

/**
 * Post send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int flexboot_nodnic_post_send ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp,
			      struct ib_address_vector *av,
			      struct io_buffer *iobuf) {

	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp = ib_qp_get_drvdata ( qp );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct ib_work_queue *wq = &qp->send;
	struct nodnic_send_wqbb *wqbb;
	nodnic_qp *nodnic_qp = flexboot_nodnic_qp->nodnic_queue_pair;
	struct nodnic_send_ring *send_ring = &nodnic_qp->send;
	mlx_status status = MLX_SUCCESS;
	unsigned int wqe_idx_mask;
	unsigned long wqe_idx;

	if ( ( port->port_priv.dma_state == FALSE ) ||
		 ( port->port_priv.port_state & NODNIC_PORT_DISABLING_DMA ) ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic DMA disabled\n");
		status = -ENETDOWN;
		goto post_send_done;
	}

	/* Allocate work queue entry */
	wqe_idx = wq->next_idx;
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[wqe_idx & wqe_idx_mask] ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p QPN %#lx send queue full\n",
				flexboot_nodnic, qp->qpn );
		status = -ENOBUFS;
		goto post_send_done;
	}
	wqbb = &send_ring->wqe_virt[wqe_idx & wqe_idx_mask];
	wq->iobufs[wqe_idx & wqe_idx_mask] = iobuf;

	assert ( flexboot_nodnic->callbacks->
			fill_send_wqe[qp->type] != NULL );
	status = flexboot_nodnic->callbacks->
			fill_send_wqe[qp->type] ( ibdev, qp, av, iobuf,
					wqbb, wqe_idx );
	if ( status != 0 ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p QPN %#lx fill send wqe failed\n",
				flexboot_nodnic, qp->qpn );
		goto post_send_done;
	}

	wq->next_idx++;

	status = port->port_priv.send_doorbell ( &port->port_priv,
				&send_ring->nodnic_ring, ( mlx_uint16 ) wq->next_idx );
	if ( status != 0 ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p ring send doorbell failed\n", flexboot_nodnic );
	}

post_send_done:
	return status;
}

/**
 * Post receive work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int flexboot_nodnic_post_recv ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp,
			      struct io_buffer *iobuf ) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp = ib_qp_get_drvdata ( qp );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	struct ib_work_queue *wq = &qp->recv;
	nodnic_qp *nodnic_qp = flexboot_nodnic_qp->nodnic_queue_pair;
	struct nodnic_recv_ring *recv_ring = &nodnic_qp->receive;
	struct nodnic_recv_wqe *wqe;
	unsigned int wqe_idx_mask;
	mlx_status status = MLX_SUCCESS;

	/* Allocate work queue entry */
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[wq->next_idx & wqe_idx_mask] ) {
		DBGC_NODNIC ( flexboot_nodnic,
				"flexboot_nodnic %p QPN %#lx receive queue full\n",
				flexboot_nodnic, qp->qpn );
		status = -ENOBUFS;
		goto post_recv_done;
	}
	wq->iobufs[wq->next_idx & wqe_idx_mask] = iobuf;
	wqe = &((struct nodnic_recv_wqe*)recv_ring->wqe_virt)[wq->next_idx & wqe_idx_mask];

	MLX_FILL_1 ( &wqe->data[0], 0, byte_count, iob_tailroom ( iobuf ) );
	MLX_FILL_1 ( &wqe->data[0], 1, l_key, flexboot_nodnic->device_priv.lkey );
	MLX_FILL_H ( &wqe->data[0], 2,
			 local_address_h, virt_to_bus ( iobuf->data ) );
	MLX_FILL_1 ( &wqe->data[0], 3,
			 local_address_l, virt_to_bus ( iobuf->data ) );

	wq->next_idx++;

	status = port->port_priv.recv_doorbell ( &port->port_priv,
				&recv_ring->nodnic_ring, ( mlx_uint16 ) wq->next_idx );
	if ( status != 0 ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p ring receive doorbell failed\n", flexboot_nodnic );
	}
post_recv_done:
	return status;
}

/***************************************************************************
 *
 * Event queues
 *
 ***************************************************************************
 */

static void flexboot_nodnic_poll_eq ( struct ib_device *ibdev ) {
	struct flexboot_nodnic *flexboot_nodnic;
	struct flexboot_nodnic_port *port;
	struct net_device *netdev;
	nodnic_port_state state = 0;
	mlx_status status;

	if ( ! ibdev ) {
		DBG_NODNIC ( "%s: ibdev = NULL!!!\n", __FUNCTION__ );
		return;
	}

	flexboot_nodnic = ib_get_drvdata ( ibdev );
	port = &flexboot_nodnic->port[ibdev->port - 1];
	netdev = port->netdev;

	if ( ! netdev_is_open ( netdev ) ) {
		DBG2_NODNIC ( "%s: port %d is closed\n", __FUNCTION__, port->ibdev->port );
		return;
	}

	/* we don't poll EQ. Just poll link status if it's not active */
	if ( ! netdev_link_ok ( netdev ) ) {
		status = nodnic_port_get_state ( &port->port_priv, &state );
		MLX_FATAL_CHECK_STATUS(status, state_err, "nodnic_port_get_state failed");

		if ( state == nodnic_port_state_active ) {
			DBG_NODNIC ( "%s: port %d physical link is up\n", __FUNCTION__,
					port->ibdev->port );
			port->type->state_change ( flexboot_nodnic, port, 1 );
		}
	}
state_err:
	return;
}

/***************************************************************************
 *
 * Multicast group operations
 *
 ***************************************************************************
 */
static int flexboot_nodnic_mcast_attach ( struct ib_device *ibdev,
				 struct ib_queue_pair *qp,
				 union ib_gid *gid) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	mlx_mac_address mac;
	mlx_status status = MLX_SUCCESS;

	switch (qp->type) {
	case IB_QPT_ETH:
		memcpy(&mac, &gid, sizeof(mac));
		status = nodnic_port_add_mac_filter(&port->port_priv, mac);
		MLX_CHECK_STATUS(flexboot_nodnic->device_priv, status, mac_err,
				"nodnic_port_add_mac_filter failed");
		break;
	default:
		break;
	}
mac_err:
	return status;
}
static void flexboot_nodnic_mcast_detach ( struct ib_device *ibdev,
				  struct ib_queue_pair *qp,
				  union ib_gid *gid ) {
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	struct flexboot_nodnic_port *port = &flexboot_nodnic->port[ibdev->port - 1];
	mlx_mac_address mac;
	mlx_status status = MLX_SUCCESS;

	switch (qp->type) {
	case IB_QPT_ETH:
		memcpy(&mac, &gid, sizeof(mac));
		status = nodnic_port_remove_mac_filter(&port->port_priv, mac);
		MLX_CHECK_STATUS(flexboot_nodnic->device_priv, status, mac_err,
				"nodnic_port_remove_mac_filter failed");
		break;
	default:
		break;
	}
mac_err:
	return;
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
static int flexboot_nodnic_ib_open ( struct ib_device *ibdev __unused) {
	int rc = 0;

	/*TODO: add implementation*/
	return rc;
}

/**
 * Close Infiniband link
 *
 * @v ibdev		Infiniband device
 */
static void flexboot_nodnic_ib_close ( struct ib_device *ibdev __unused) {
	/*TODO: add implementation*/
}

/**
 * Inform embedded subnet management agent of a received MAD
 *
 * @v ibdev		Infiniband device
 * @v mad		MAD
 * @ret rc		Return status code
 */
static int flexboot_nodnic_inform_sma ( struct ib_device *ibdev __unused,
			       union ib_mad *mad __unused) {
	/*TODO: add implementation*/
	return 0;
}

/** flexboot_nodnic Infiniband operations */
static struct ib_device_operations flexboot_nodnic_ib_operations = {
	.create_cq	= flexboot_nodnic_create_cq,
	.destroy_cq	= flexboot_nodnic_destroy_cq,
	.create_qp	= flexboot_nodnic_create_qp,
	.modify_qp	= flexboot_nodnic_modify_qp,
	.destroy_qp	= flexboot_nodnic_destroy_qp,
	.post_send	= flexboot_nodnic_post_send,
	.post_recv	= flexboot_nodnic_post_recv,
	.poll_cq	= flexboot_nodnic_poll_cq,
	.poll_eq	= flexboot_nodnic_poll_eq,
	.open		= flexboot_nodnic_ib_open,
	.close		= flexboot_nodnic_ib_close,
	.mcast_attach	= flexboot_nodnic_mcast_attach,
	.mcast_detach	= flexboot_nodnic_mcast_detach,
	.set_port_info	= flexboot_nodnic_inform_sma,
	.set_pkey_table	= flexboot_nodnic_inform_sma,
};
/***************************************************************************
 *
 *
 *
 ***************************************************************************
 */

#define FLEX_NODNIC_TX_POLL_TOUT	500000
#define FLEX_NODNIC_TX_POLL_USLEEP	10

static void flexboot_nodnic_complete_all_tx ( struct flexboot_nodnic_port *port ) {
	struct ib_device *ibdev = port->ibdev;
	struct ib_completion_queue *cq;
	struct ib_work_queue *wq;
	int keep_polling = 0;
	int timeout = FLEX_NODNIC_TX_POLL_TOUT;

	list_for_each_entry ( cq, &ibdev->cqs, list ) {
		do {
			ib_poll_cq ( ibdev, cq );
			keep_polling = 0;
			list_for_each_entry ( wq, &cq->work_queues, list ) {
				if ( wq->is_send )
					keep_polling += ( wq->fill > 0 );
			}
			udelay ( FLEX_NODNIC_TX_POLL_USLEEP );
		} while ( keep_polling && ( timeout-- > 0 ) );
	}
}

static void flexboot_nodnic_port_disable_dma ( struct flexboot_nodnic_port *port ) {
	nodnic_port_priv *port_priv = & ( port->port_priv );
	mlx_status status;

	if ( ! ( port_priv->port_state & NODNIC_PORT_OPENED ) )
		return;

	port_priv->port_state |= NODNIC_PORT_DISABLING_DMA;
	flexboot_nodnic_complete_all_tx ( port );
	if ( ( status = nodnic_port_disable_dma ( port_priv ) ) ) {
		MLX_DEBUG_WARN ( port, "Failed to disable DMA %d\n", status );
	}

	port_priv->port_state &= ~NODNIC_PORT_DISABLING_DMA;
}

/***************************************************************************
 *
 * Ethernet operation
 *
 ***************************************************************************
 */

/** Number of flexboot_nodnic Ethernet send work queue entries */
#define FLEXBOOT_NODNIC_ETH_NUM_SEND_WQES 64

/** Number of flexboot_nodnic Ethernet receive work queue entries */
#define FLEXBOOT_NODNIC_ETH_NUM_RECV_WQES 64
/** flexboot nodnic Ethernet queue pair operations */
static struct ib_queue_pair_operations flexboot_nodnic_eth_qp_op = {
	.alloc_iob = alloc_iob,
};

/**
 * Transmit packet via flexboot_nodnic Ethernet device
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int flexboot_nodnic_eth_transmit ( struct net_device *netdev,
				 struct io_buffer *iobuf) {
	struct flexboot_nodnic_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	int rc;

	rc = ib_post_send ( ibdev, port->eth_qp, NULL, iobuf);
	/* Transmit packet */
	if ( rc != 0) {
		DBGC_NODNIC ( flexboot_nodnic, "NODNIC %p port %d could not transmit: %s\n",
				flexboot_nodnic, ibdev->port, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Handle flexboot_nodnic Ethernet device send completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void flexboot_nodnic_eth_complete_send ( struct ib_device *ibdev __unused,
				       struct ib_queue_pair *qp,
				       struct io_buffer *iobuf,
					   int rc) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );

	netdev_tx_complete_err ( netdev, iobuf, rc );
}

/**
 * Handle flexboot_nodnic Ethernet device receive completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector, or NULL
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void flexboot_nodnic_eth_complete_recv ( struct ib_device *ibdev __unused,
				struct ib_queue_pair *qp,
				struct ib_address_vector *dest __unused,
				struct ib_address_vector *source,
				 struct io_buffer *iobuf,
				int rc) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );

	if ( rc != 0 ) {
		DBG_NODNIC ( "Received packet with error\n" );
		netdev_rx_err ( netdev, iobuf, rc );
		return;
	}

	if ( source == NULL ) {
		DBG_NODNIC ( "Received packet without address vector\n" );
		netdev_rx_err ( netdev, iobuf, -ENOTTY );
		return;
	}
	netdev_rx ( netdev, iobuf );
}

/** flexboot_nodnic Ethernet device completion operations */
static struct ib_completion_queue_operations flexboot_nodnic_eth_cq_op = {
	.complete_send = flexboot_nodnic_eth_complete_send,
	.complete_recv = flexboot_nodnic_eth_complete_recv,
};

/**
 * Poll flexboot_nodnic Ethernet device
 *
 * @v netdev		Network device
 */
static void flexboot_nodnic_eth_poll ( struct net_device *netdev) {
	struct flexboot_nodnic_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;

	ib_poll_eq ( ibdev );
}

/**
 * Open flexboot_nodnic Ethernet device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int flexboot_nodnic_eth_open ( struct net_device *netdev ) {
	struct flexboot_nodnic_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	mlx_status status = MLX_SUCCESS;
	struct ib_completion_queue *dummy_cq = NULL;
	struct flexboot_nodnic_queue_pair *flexboot_nodnic_qp = NULL;
	mlx_uint64	cq_size = 0;
	mlx_uint32	qpn = 0;
	nodnic_port_state state = nodnic_port_state_down;

	if ( port->port_priv.port_state & NODNIC_PORT_OPENED ) {
		DBGC_NODNIC ( flexboot_nodnic, "%s: port %d is already opened\n",
				__FUNCTION__, port->ibdev->port );
		return 0;
	}

	port->port_priv.port_state |= NODNIC_PORT_OPENED;

	dummy_cq = zalloc ( sizeof ( struct ib_completion_queue ) );
	if ( dummy_cq == NULL ) {
		DBGC_NODNIC ( flexboot_nodnic, "%s: Failed to allocate dummy CQ\n", __FUNCTION__ );
		status = MLX_OUT_OF_RESOURCES;
		goto err_create_dummy_cq;
	}
	INIT_LIST_HEAD ( &dummy_cq->work_queues );

	port->eth_qp = ib_create_qp ( ibdev, IB_QPT_ETH,
					FLEXBOOT_NODNIC_ETH_NUM_SEND_WQES, dummy_cq,
					FLEXBOOT_NODNIC_ETH_NUM_RECV_WQES, dummy_cq,
					&flexboot_nodnic_eth_qp_op, netdev->name );
	if ( !port->eth_qp ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p port %d could not create queue pair\n",
				 flexboot_nodnic, ibdev->port );
		status = MLX_OUT_OF_RESOURCES;
		goto err_create_qp;
	}

	ib_qp_set_ownerdata ( port->eth_qp, netdev );

	status = nodnic_port_get_cq_size(&port->port_priv, &cq_size);
	MLX_FATAL_CHECK_STATUS(status, get_cq_size_err,
			"nodnic_port_get_cq_size failed");

	port->eth_cq = ib_create_cq ( ibdev, cq_size,
			&flexboot_nodnic_eth_cq_op );
	if ( !port->eth_cq ) {
		DBGC_NODNIC ( flexboot_nodnic,
			"flexboot_nodnic %p port %d could not create completion queue\n",
			flexboot_nodnic, ibdev->port );
		status = MLX_OUT_OF_RESOURCES;
		goto err_create_cq;
	}
	port->eth_qp->send.cq = port->eth_cq;
	list_del(&port->eth_qp->send.list);
	list_add ( &port->eth_qp->send.list, &port->eth_cq->work_queues );
	port->eth_qp->recv.cq = port->eth_cq;
	list_del(&port->eth_qp->recv.list);
	list_add ( &port->eth_qp->recv.list, &port->eth_cq->work_queues );

	status = nodnic_port_allocate_eq(&port->port_priv,
		flexboot_nodnic->device_priv.device_cap.log_working_buffer_size);
	MLX_FATAL_CHECK_STATUS(status, eq_alloc_err,
				"nodnic_port_allocate_eq failed");

	status = nodnic_port_init(&port->port_priv);
	MLX_FATAL_CHECK_STATUS(status, init_err,
					"nodnic_port_init failed");

	/* update qp - qpn */
	flexboot_nodnic_qp = ib_qp_get_drvdata ( port->eth_qp );
	status = nodnic_port_get_qpn(&port->port_priv,
			&flexboot_nodnic_qp->nodnic_queue_pair->send.nodnic_ring,
			&qpn);
	MLX_FATAL_CHECK_STATUS(status, qpn_err,
						"nodnic_port_get_qpn failed");
	port->eth_qp->qpn = qpn;

	/* Fill receive rings */
	ib_refill_recv ( ibdev, port->eth_qp );

	status = nodnic_port_enable_dma(&port->port_priv);
	MLX_FATAL_CHECK_STATUS(status, dma_err,
					"nodnic_port_enable_dma failed");

	if (flexboot_nodnic->device_priv.device_cap.support_promisc_filter) {
		status = nodnic_port_set_promisc(&port->port_priv, TRUE);
		MLX_FATAL_CHECK_STATUS(status, promisc_err,
							"nodnic_port_set_promisc failed");
	}

	status = nodnic_port_get_state(&port->port_priv, &state);
	MLX_FATAL_CHECK_STATUS(status, state_err,
						"nodnic_port_get_state failed");

	port->type->state_change (
			flexboot_nodnic, port, state == nodnic_port_state_active );

	DBGC_NODNIC ( flexboot_nodnic, "%s: port %d opened (link is %s)\n",
			__FUNCTION__, port->ibdev->port,
			( ( state == nodnic_port_state_active ) ? "Up" : "Down" ) );

	free(dummy_cq);
	return 0;
state_err:
promisc_err:
dma_err:
qpn_err:
	nodnic_port_close(&port->port_priv);
init_err:
	nodnic_port_free_eq(&port->port_priv);
eq_alloc_err:
err_create_cq:
get_cq_size_err:
	ib_destroy_qp(ibdev, port->eth_qp );
err_create_qp:
	free(dummy_cq);
err_create_dummy_cq:
	port->port_priv.port_state &= ~NODNIC_PORT_OPENED;
	return status;
}

/**
 * Close flexboot_nodnic Ethernet device
 *
 * @v netdev		Network device
 */
static void flexboot_nodnic_eth_close ( struct net_device *netdev) {
	struct flexboot_nodnic_port *port = netdev->priv;
	struct ib_device *ibdev = port->ibdev;
	struct flexboot_nodnic *flexboot_nodnic = ib_get_drvdata ( ibdev );
	mlx_status status = MLX_SUCCESS;

	if ( ! ( port->port_priv.port_state & NODNIC_PORT_OPENED ) ) {
		DBGC_NODNIC ( flexboot_nodnic, "%s: port %d is already closed\n",
				__FUNCTION__, port->ibdev->port );
		return;
	}

	if (flexboot_nodnic->device_priv.device_cap.support_promisc_filter) {
		if ( ( status = nodnic_port_set_promisc( &port->port_priv, FALSE ) ) ) {
			DBGC_NODNIC ( flexboot_nodnic,
					"nodnic_port_set_promisc failed (status = %d)\n", status );
		}
	}

	flexboot_nodnic_port_disable_dma ( port );

	port->port_priv.port_state &= ~NODNIC_PORT_OPENED;

	port->type->state_change ( flexboot_nodnic, port, FALSE );

	/* Close port */
	status = nodnic_port_close(&port->port_priv);
	if ( status != MLX_SUCCESS ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p port %d could not close port: %s\n",
				flexboot_nodnic, ibdev->port, strerror ( status ) );
		/* Nothing we can do about this */
	}

	ib_destroy_qp ( ibdev, port->eth_qp );
	port->eth_qp = NULL;
	ib_destroy_cq ( ibdev, port->eth_cq );
	port->eth_cq = NULL;

	nodnic_port_free_eq(&port->port_priv);

	DBGC_NODNIC ( flexboot_nodnic, "%s: port %d closed\n", __FUNCTION__, port->ibdev->port );
}

void flexboot_nodnic_eth_irq ( struct net_device *netdev, int enable ) {
	struct flexboot_nodnic_port *port = netdev->priv;

	if ( enable ) {
		if ( ( port->port_priv.port_state & NODNIC_PORT_OPENED ) &&
			 ! ( port->port_priv.port_state & NODNIC_PORT_DISABLING_DMA ) ) {
			flexboot_nodnic_arm_cq ( port );
		} else {
			/* do nothing */
		}
	} else {
		nodnic_device_clear_int( port->port_priv.device );
	}
}

/** flexboot_nodnic Ethernet network device operations */
static struct net_device_operations flexboot_nodnic_eth_operations = {
	.open		= flexboot_nodnic_eth_open,
	.close		= flexboot_nodnic_eth_close,
	.transmit	= flexboot_nodnic_eth_transmit,
	.poll		= flexboot_nodnic_eth_poll,
};

/**
 * Register flexboot_nodnic Ethernet device
 */
static int flexboot_nodnic_register_netdev ( struct flexboot_nodnic *flexboot_nodnic,
				    struct flexboot_nodnic_port *port) {
	mlx_status status = MLX_SUCCESS;
	struct net_device	*netdev;
	struct ib_device	*ibdev = port->ibdev;
	union {
		uint8_t bytes[8];
		uint32_t dwords[2];
	} mac;
	/* Set protocol */

	ibdev->protocol = FLEXBOOT_NODNIC_PROT_ETH;

	/* Allocate network devices */
	netdev = alloc_etherdev ( 0 );
	if ( netdev == NULL ) {
		DBGC_NODNIC ( flexboot_nodnic, "flexboot_nodnic %p port %d could not allocate net device\n",
				flexboot_nodnic, ibdev->port );
		status = MLX_OUT_OF_RESOURCES;
		goto alloc_err;
	}
	port->netdev = netdev;
	netdev_init ( netdev, &flexboot_nodnic_eth_operations );
	netdev->dev = ibdev->dev;
	netdev->priv = port;

	status = nodnic_port_query(&port->port_priv,
			nodnic_port_option_mac_high,
			&mac.dwords[0]);
	MLX_FATAL_CHECK_STATUS(status, mac_err,
			"failed to query mac high");
	status = nodnic_port_query(&port->port_priv,
			nodnic_port_option_mac_low,
			&mac.dwords[1]);
	MLX_FATAL_CHECK_STATUS(status, mac_err,
				"failed to query mac low");
	mac.dwords[0] = htonl(mac.dwords[0]);
	mac.dwords[1] = htonl(mac.dwords[1]);
	memcpy ( netdev->hw_addr,
			 &mac.bytes[2], ETH_ALEN);
	/* Register network device */
	status = register_netdev ( netdev );
	if ( status != MLX_SUCCESS ) {
		DBGC_NODNIC ( flexboot_nodnic,
			"flexboot_nodnic %p port %d could not register network device: %s\n",
			flexboot_nodnic, ibdev->port, strerror ( status ) );
		goto reg_err;
	}
	return status;
reg_err:
mac_err:
	netdev_put ( netdev );
alloc_err:
	return status;
}

/**
 * Handle flexboot_nodnic Ethernet device port state change
 */
static void flexboot_nodnic_state_change_netdev ( struct flexboot_nodnic *flexboot_nodnic __unused,
					 struct flexboot_nodnic_port *port,
					 int link_up ) {
	struct net_device *netdev = port->netdev;

	if ( link_up )
		netdev_link_up ( netdev );
	else
		netdev_link_down ( netdev );

}

/**
 * Unregister flexboot_nodnic Ethernet device
 */
static void flexboot_nodnic_unregister_netdev ( struct flexboot_nodnic *flexboot_nodnic __unused,
				       struct flexboot_nodnic_port *port ) {
	struct net_device *netdev = port->netdev;
	unregister_netdev ( netdev );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** flexboot_nodnic Ethernet port type */
static struct flexboot_nodnic_port_type flexboot_nodnic_port_type_eth = {
	.register_dev = flexboot_nodnic_register_netdev,
	.state_change = flexboot_nodnic_state_change_netdev,
	.unregister_dev = flexboot_nodnic_unregister_netdev,
};

/***************************************************************************
 *
 * PCI interface helper functions
 *
 ***************************************************************************
 */
static
mlx_status
flexboot_nodnic_allocate_infiniband_devices( struct flexboot_nodnic *flexboot_nodnic_priv ) {
	mlx_status status = MLX_SUCCESS;
	nodnic_device_priv *device_priv = &flexboot_nodnic_priv->device_priv;
	struct pci_device *pci = flexboot_nodnic_priv->pci;
	struct ib_device *ibdev = NULL;
	unsigned int i = 0;

	/* Allocate Infiniband devices */
	for (; i < device_priv->device_cap.num_ports; i++) {
		if ( ! ( flexboot_nodnic_priv->port_mask & ( i + 1 ) ) )
			continue;
		ibdev = alloc_ibdev(0);
		if (ibdev == NULL) {
			status = MLX_OUT_OF_RESOURCES;
			goto err_alloc_ibdev;
		}
		flexboot_nodnic_priv->port[i].ibdev = ibdev;
		ibdev->op = &flexboot_nodnic_ib_operations;
		ibdev->dev = &pci->dev;
		ibdev->port = ( FLEXBOOT_NODNIC_PORT_BASE + i);
		ib_set_drvdata(ibdev, flexboot_nodnic_priv);
	}
	return status;
err_alloc_ibdev:
	for ( i-- ; ( signed int ) i >= 0 ; i-- )
		ibdev_put ( flexboot_nodnic_priv->port[i].ibdev );
	return status;
}

static
mlx_status
flexboot_nodnic_thin_init_ports( struct flexboot_nodnic *flexboot_nodnic_priv ) {
	mlx_status status = MLX_SUCCESS;
	nodnic_device_priv *device_priv = &flexboot_nodnic_priv->device_priv;
	nodnic_port_priv *port_priv = NULL;
	unsigned int i = 0;

	for ( i = 0; i < device_priv->device_cap.num_ports; i++ ) {
		if ( ! ( flexboot_nodnic_priv->port_mask & ( i + 1 ) ) )
			continue;
		port_priv = &flexboot_nodnic_priv->port[i].port_priv;
		status = nodnic_port_thin_init( device_priv, port_priv, i );
		MLX_FATAL_CHECK_STATUS(status, thin_init_err,
				"flexboot_nodnic_thin_init_ports failed");
	}
thin_init_err:
	return status;
}


static
mlx_status
flexboot_nodnic_set_ports_type ( struct flexboot_nodnic *flexboot_nodnic_priv ) {
	mlx_status status = MLX_SUCCESS;
	nodnic_device_priv	*device_priv = &flexboot_nodnic_priv->device_priv;
	nodnic_port_priv	*port_priv = NULL;
	nodnic_port_type	type = NODNIC_PORT_TYPE_UNKNOWN;
	unsigned int i = 0;

	for ( i = 0 ; i < device_priv->device_cap.num_ports ; i++ ) {
		if ( ! ( flexboot_nodnic_priv->port_mask & ( i + 1 ) ) )
			continue;
		port_priv = &flexboot_nodnic_priv->port[i].port_priv;
		status = nodnic_port_get_type(port_priv, &type);
		MLX_FATAL_CHECK_STATUS(status, type_err,
				"nodnic_port_get_type failed");
		switch ( type ) {
		case NODNIC_PORT_TYPE_ETH:
			DBGC_NODNIC ( flexboot_nodnic_priv, "Port %d type is Ethernet\n", i );
			flexboot_nodnic_priv->port[i].type = &flexboot_nodnic_port_type_eth;
			break;
		case NODNIC_PORT_TYPE_IB:
			DBGC_NODNIC ( flexboot_nodnic_priv, "Port %d type is Infiniband\n", i );
			status = MLX_UNSUPPORTED;
			goto type_err;
		default:
			DBGC_NODNIC ( flexboot_nodnic_priv, "Port %d type is unknown\n", i );
			status = MLX_UNSUPPORTED;
			goto type_err;
		}
	}
type_err:
	return status;
}

static
mlx_status
flexboot_nodnic_ports_register_dev( struct flexboot_nodnic *flexboot_nodnic_priv ) {
	mlx_status status = MLX_SUCCESS;
	nodnic_device_priv *device_priv = &flexboot_nodnic_priv->device_priv;
	struct flexboot_nodnic_port *port = NULL;
	unsigned int i = 0;

	for (; i < device_priv->device_cap.num_ports; i++) {
		if ( ! ( flexboot_nodnic_priv->port_mask & ( i + 1 ) ) )
			continue;
		port = &flexboot_nodnic_priv->port[i];
		status = port->type->register_dev ( flexboot_nodnic_priv, port );
		MLX_FATAL_CHECK_STATUS(status, reg_err,
				"port register_dev failed");
	}
reg_err:
	return status;
}

static
mlx_status
flexboot_nodnic_ports_unregister_dev ( struct flexboot_nodnic *flexboot_nodnic_priv ) {
	struct flexboot_nodnic_port *port;
	nodnic_device_priv	*device_priv = &flexboot_nodnic_priv->device_priv;
	int i = (device_priv->device_cap.num_ports - 1);

	for (; i >= 0; i--) {
		if ( ! ( flexboot_nodnic_priv->port_mask & ( i + 1 ) ) )
			continue;
		port = &flexboot_nodnic_priv->port[i];
		port->type->unregister_dev(flexboot_nodnic_priv, port);
		ibdev_put(flexboot_nodnic_priv->port[i].ibdev);
	}
	return MLX_SUCCESS;
}

/***************************************************************************
 *
 * flexboot nodnic interface
 *
 ***************************************************************************
 */
static void flexboot_nodnic_enable_dma ( struct flexboot_nodnic *nodnic ) {
	nodnic_port_priv *port_priv;
	mlx_status status;
	int i;

	for ( i = 0; i < nodnic->device_priv.device_cap.num_ports; i++ ) {
		if ( ! ( nodnic->port_mask & ( i + 1 ) ) )
			continue;
		port_priv = & ( nodnic->port[i].port_priv );
		if ( ! ( port_priv->port_state & NODNIC_PORT_OPENED ) )
			continue;

		if ( ( status = nodnic_port_enable_dma ( port_priv ) ) ) {
			MLX_DEBUG_WARN ( nodnic, "Failed to enable DMA %d\n", status );
		}
	}
}

static void flexboot_nodnic_disable_dma ( struct flexboot_nodnic *nodnic ) {
	int i;

	for ( i = 0; i < nodnic->device_priv.device_cap.num_ports; i++ ) {
		if ( ! ( nodnic->port_mask & ( i + 1 ) ) )
			continue;
		flexboot_nodnic_port_disable_dma ( & ( nodnic->port[i] ) );
	}
}

static void flexboot_nodnic_updater ( void *priv, uint8_t status ) {
	struct flexboot_nodnic *nodnic = ( struct flexboot_nodnic * ) priv;

	switch ( status ) {
	case STATUS_UPDATE_PXE_BOOT_START:
	case STATUS_UPDATE_INT13_START:
		flexboot_nodnic_enable_dma ( nodnic );
		break;
	case STATUS_UPDATE_INT13_END:
		flexboot_nodnic_disable_dma ( nodnic );
		break;
	default:
		DBGC_NODNIC ( nodnic, "%s: Unknown status %d\n", __FUNCTION__, status );
	}
}

int flexboot_nodnic_is_supported ( struct pci_device *pci ) {
	mlx_utils utils;
	mlx_pci_gw_buffer buffer;
	mlx_status status;
	int is_supported = 0;

	DBG_NODNIC ( "%s: start\n", __FUNCTION__ );

	memset ( &utils, 0, sizeof ( utils ) );

	status = mlx_utils_init ( &utils, pci );
	MLX_CHECK_STATUS ( pci, status, utils_init_err, "mlx_utils_init failed" );

	status = mlx_pci_gw_init ( &utils );
	MLX_CHECK_STATUS ( pci, status, pci_gw_init_err, "mlx_pci_gw_init failed" );

	status = mlx_pci_gw_read ( &utils, PCI_GW_SPACE_NODNIC,
			NODNIC_NIC_INTERFACE_SUPPORTED_OFFSET, &buffer );

	if ( status == MLX_SUCCESS ) {
		buffer >>= NODNIC_NIC_INTERFACE_SUPPORTED_BIT;
		is_supported = ( buffer & 0x1 );
	}

	mlx_pci_gw_teardown( &utils );

pci_gw_init_err:
	mlx_utils_teardown(&utils);
utils_init_err:
	DBG_NODNIC ( "%s: NODNIC is %s supported (status = %d)\n",
			__FUNCTION__, ( is_supported ? "": "not" ), status );
	return is_supported;
}

#define FLEXBOOT_NODNIC_TLV_ACCESS_READ			0x1
#define FLEXBOOT_NODNIC_TLV_ACCESS_WRITE		0x2
#define FLEXBOOT_NODNIC_TLV_ACCESS_INVALIDATE	0x3

void swab_settings_bits ( uint32 tlv_type, void *data, uint32_t length ) {
	uint32_t * u32ptr = ( uint32_t * ) data;
	uint32_t i = 0;

	switch ( tlv_type ) {
	case DHCP_VEND_ID					:
	case ISCSI_INITIATOR_IPV4_ADDR		:
	case ISCSI_INITIATOR_SUBNET			:
	case ISCSI_INITIATOR_IPV4_GATEWAY	:
	case ISCSI_INITIATOR_IPV4_PRIM_DNS	:
	case ISCSI_INITIATOR_IPV4_SECDNS	:
	case ISCSI_INITIATOR_NAME			:
	case ISCSI_INITIATOR_CHAP_ID		:
	case ISCSI_INITIATOR_CHAP_PWD		:
	case CONNECT_FIRST_TGT				:
	case FIRST_TGT_IP_ADDRESS			:
	case FIRST_TGT_TCP_PORT				:
	case FIRST_TGT_BOOT_LUN				:
	case FIRST_TGT_ISCSI_NAME			:
	case FIRST_TGT_CHAP_ID				:
	case FIRST_TGT_CHAP_PWD				:
		break;
	default:
		for (; i * 4 < length; i++ ) {
			u32ptr[i] = be32_to_cpu( u32ptr[i] );
		}
	}
}

static int flexboot_nodnic_tlv_access ( struct flexboot_nodnic * flexboot_nodnic,
		struct driver_tlv_header *tlv_hdr, uint8_t access_method ) {
	int rc = -1;

	if ( access_method == FLEXBOOT_NODNIC_TLV_ACCESS_WRITE ) {
		rc = flexboot_nodnic->callbacks->tlv_write_fn (
				flexboot_nodnic->device_priv.utils, tlv_hdr );
	} else if ( access_method == FLEXBOOT_NODNIC_TLV_ACCESS_READ ) {
		rc = flexboot_nodnic->callbacks->tlv_read_fn (
				flexboot_nodnic->device_priv.utils, tlv_hdr );
	} else if ( access_method == FLEXBOOT_NODNIC_TLV_ACCESS_INVALIDATE ) {
		rc = flexboot_nodnic->callbacks->tlv_invaidate_fn (
				flexboot_nodnic->device_priv.utils, tlv_hdr );
	}

	return rc;
}

int flexboot_nodnic_invalidate_tlv_wrapper ( void *drv_priv, uint32_t tlv_mod,
		uint32_t tlv_type ) {
	struct driver_tlv_header tlv_hdr;
	struct flexboot_nodnic * flexboot_nodnic = ( struct flexboot_nodnic * ) drv_priv;

	memset ( &tlv_hdr, 0, sizeof ( tlv_hdr ) );
	tlv_hdr.type = tlv_type;
	tlv_hdr.type_mod = tlv_mod;
	tlv_hdr.version     = driver_settings_get_tlv_version ( tlv_type );

	return flexboot_nodnic_tlv_access ( flexboot_nodnic, &tlv_hdr, FLEXBOOT_NODNIC_TLV_ACCESS_INVALIDATE );
}

static int flexboot_nodnic_flash_write_tlv_wrapper ( void *drv_priv, void *src, uint32_t port_num,
        uint32_t tlv_type, uint32_t len ) {
	struct driver_tlv_header tlv_hdr;
	struct flexboot_nodnic * flexboot_nodnic = ( struct flexboot_nodnic * ) drv_priv;

	memset ( &tlv_hdr, 0, sizeof ( tlv_hdr ) );
	tlv_hdr.length = len;
	tlv_hdr.type = tlv_type;
	tlv_hdr.type_mod = port_num;
	tlv_hdr.data = src;
	tlv_hdr.version     = driver_settings_get_tlv_version ( tlv_type );

	return flexboot_nodnic_tlv_access ( flexboot_nodnic, &tlv_hdr, FLEXBOOT_NODNIC_TLV_ACCESS_WRITE );
}

static int flexboot_nodnic_flash_read_tlv_wrapper ( void *drv_priv
		,struct driver_tlv_header *tlv_hdr ) {
	struct flexboot_nodnic * flexboot_nodnic = ( struct flexboot_nodnic * ) drv_priv;
	return flexboot_nodnic_tlv_access ( flexboot_nodnic , tlv_hdr,  FLEXBOOT_NODNIC_TLV_ACCESS_READ );
}

void flexboot_nodnic_copy_mac ( uint8_t mac_addr[], uint32_t low_byte,
		uint16_t high_byte ) {
	union mac_addr {
		struct {
			uint32_t low_byte;
			uint16_t high_byte;
		};
		uint8_t mac_addr[ETH_ALEN];
	} mac_addr_aux;

	mac_addr_aux.high_byte = high_byte;
	mac_addr_aux.low_byte = low_byte;

	mac_addr[0] = mac_addr_aux.mac_addr[5];
	mac_addr[1] = mac_addr_aux.mac_addr[4];
	mac_addr[2] = mac_addr_aux.mac_addr[3];
	mac_addr[3] = mac_addr_aux.mac_addr[2];
	mac_addr[4] = mac_addr_aux.mac_addr[1];
	mac_addr[5] = mac_addr_aux.mac_addr[0];
}

static mlx_status flexboot_nodnic_get_factory_mac (
		struct flexboot_nodnic *flexboot_nodnic_priv, uint8_t port ) {
	struct mlx_vmac_query_virt_mac virt_mac;
	mlx_status status;

	memset ( & virt_mac, 0, sizeof ( virt_mac ) );
	status = mlx_vmac_query_virt_mac ( flexboot_nodnic_priv->device_priv.utils,
			&virt_mac );
	if ( ! status ) {
		flexboot_nodnic_copy_mac (
				flexboot_nodnic_priv->port[port].port_nv_conf.phys_mac,
				virt_mac.parmanent_mac_low, virt_mac.parmanent_mac_high );
	}

	return status;
}

static mlx_status flexboot_nodnic_get_wol_conf (
		struct flexboot_nodnic *flexboot_nodnic_priv)
{
	mlx_boolean read_en = 0;
	mlx_boolean write_en = 0;
	int rc;

	flexboot_nodnic_priv->wol_en = 0;
	if ( ! ( rc = nvconfig_query_capability( flexboot_nodnic_priv->device_priv.utils,
				0, WAKE_ON_LAN_TYPE, &read_en, &write_en ) ) && read_en
			&& write_en ) {
			flexboot_nodnic_priv->wol_en = 1;
	}

	return rc;
}

/**
 * Set port masking
 *
 * @v flexboot_nodnic		nodnic device
 * @ret rc		Return status code
 */
static int flexboot_nodnic_set_port_masking ( struct flexboot_nodnic *flexboot_nodnic ) {
	unsigned int i;
	u8 boot_enable = 0;
	nodnic_device_priv *device_priv = &flexboot_nodnic->device_priv;
	union mlx_nvconfig_nic_boot_conf *boot_con;

	flexboot_nodnic->port_mask = 0;
	for ( i = 0; i < device_priv->device_cap.num_ports; i++ ) {
		/* If we are in POST as a result of pressed CTRL-B - then enable all ports */
		if ( boot_post_shell ) {
			boot_enable = 1;
		} else {
			if ( ! flexboot_nodnic->callbacks->get_settings ) {
				driver_settings_get_nv_boot_en( flexboot_nodnic,
						i + 1, &boot_enable, flexboot_nodnic_flash_read_tlv_wrapper,
						& ( flexboot_nodnic->port[i].defaults ) );
			} else {
				boot_con = & ( flexboot_nodnic->port[i].port_nv_conf.nic.boot_conf );
				if ( ( boot_con->legacy_boot_prot != 0 ) && ( boot_con->en_option_rom != 0 ) )
					boot_enable = 1;
				else
					boot_enable = 0;
			}
		}

		if ( boot_enable )
		flexboot_nodnic->port_mask |= (i + 1);
	}

	if ( ! flexboot_nodnic->port_mask ) {
		/* No port was enabled */
		DBGC_NODNIC ( flexboot_nodnic, "NODNIC %p No port was enabled for "
				"booting\n", flexboot_nodnic );
		return -ENETUNREACH;
	}

	return 0;
}

static void flexboot_nodnic_get_ro_pci_settings ( void *drv_priv ) {
	struct flexboot_nodnic *flexboot_nodnic = ( struct flexboot_nodnic * ) drv_priv;
	struct pci_device *pci = flexboot_nodnic->pci;
	struct firmware_image_props *fw_image_props;
	mlx_uint16 fw_ver_minor = 0, fw_ver_sub_minor = 0, fw_ver_major = 0;

	if ( nodnic_device_get_fw_version ( & flexboot_nodnic->device_priv,
		&fw_ver_minor, &fw_ver_sub_minor, &fw_ver_major ) ) {
		DBGC_NODNIC ( flexboot_nodnic, "%s: Failed to query firmware version\n", __FUNCTION__ );
	} else {
		fw_image_props = & ( flexboot_nodnic->nodnic_nv_conf.fw_image_props );
		snprintf ( fw_image_props->family_fw_version,
			sizeof ( fw_image_props->family_fw_version ),
			"%d.%d.%d", fw_ver_major, fw_ver_minor, fw_ver_sub_minor );
	}

	strcpy( flexboot_nodnic->nodnic_nv_conf.bdf_name,pci->dev.name );
	strcpy ( flexboot_nodnic->nodnic_nv_conf.device_name,pci->id->name );
	flexboot_nodnic->nodnic_nv_conf.desc_dev_id =  pci->dev.desc.device;
	flexboot_nodnic->nodnic_nv_conf.driver_name =  pci->dev.driver_name;
}

static int flexboot_nodnic_init_settings ( struct flexboot_nodnic *flexboot_nodnic ) {
	struct driver_settings *driver_settings = & ( flexboot_nodnic->driver_settings );
	struct generic_settings *gen_settings = & ( driver_settings->generic_settings );

	gen_settings->settings.op = driver_settings_get_operations ();
	gen_settings->settings.default_scope = &main_scope;
	nv_settings_root = gen_settings;

	driver_settings->callbacks.open_dev			= NULL;
	driver_settings->callbacks.close_dev		= NULL;
	driver_settings->callbacks.set_default		= NULL;
	if ( flexboot_nodnic->callbacks->tlv_write_fn )
		driver_settings->callbacks.tlv_write	= flexboot_nodnic_flash_write_tlv_wrapper;
	driver_settings->callbacks.tlv_read			= flexboot_nodnic_flash_read_tlv_wrapper;
	driver_settings->callbacks.tlv_invalidate	= flexboot_nodnic_invalidate_tlv_wrapper;
	driver_settings->callbacks.set_ro_device_settings = flexboot_nodnic_get_ro_pci_settings;
	driver_settings->netdev						= NULL;
	driver_settings->index						= 0;
	driver_settings->drv_priv					= flexboot_nodnic;
	driver_settings->priv_data					= & ( flexboot_nodnic->nodnic_nv_conf );
	driver_settings->defaults					= & ( flexboot_nodnic->defaults );

	if ( flexboot_nodnic->callbacks->update_settings_ops )
		flexboot_nodnic->callbacks->update_settings_ops ();

	return driver_settings_init ( driver_settings );
}

static int flexboot_nodnic_init_port_settings ( struct flexboot_nodnic *flexboot_nodnic, struct flexboot_nodnic_port *port ) {
	struct driver_settings *driver_settings = & ( port->driver_settings );

	driver_settings->callbacks.open_dev			= NULL;
	driver_settings->callbacks.close_dev		= NULL;
	driver_settings->callbacks.set_default		= NULL;
	if ( flexboot_nodnic->callbacks->tlv_write_fn )
		driver_settings->callbacks.tlv_write	= flexboot_nodnic_flash_write_tlv_wrapper;
	driver_settings->callbacks.tlv_read			= flexboot_nodnic_flash_read_tlv_wrapper;
	driver_settings->callbacks.tlv_invalidate	= flexboot_nodnic_invalidate_tlv_wrapper;
	driver_settings->callbacks.set_ro_device_settings = NULL;
	driver_settings->netdev						= port->netdev;
	driver_settings->index						= port->ibdev->port;
	driver_settings->drv_priv					= flexboot_nodnic;
	driver_settings->priv_data					= & ( port->port_nv_conf );
	driver_settings->defaults					= & ( port->defaults );

	return driver_settings_init_port ( driver_settings );
}



static int flexboot_nodnic_port_get_defaults ( struct flexboot_nodnic *flexboot_nodnic, unsigned int port_num ) {
	struct mlx_nvconfig_port_conf_defaults nv_port_conf_def;
	struct flexboot_nodnic_port *port = & ( flexboot_nodnic->port[ port_num - 1 ] );

	memset ( &nv_port_conf_def, 0, sizeof ( nv_port_conf_def ) );
	nvconfig_read_port_default_values ( flexboot_nodnic->device_priv.utils,
				port_num, &nv_port_conf_def );
	port->defaults.boot_protocol			= nv_port_conf_def.boot_protocol;
	port->defaults.boot_option_rom_en		= nv_port_conf_def.boot_option_rom_en;
	port->defaults.boot_vlan        		= nv_port_conf_def.boot_vlan;
	port->defaults.iscsi_dhcp_params_en		= nv_port_conf_def.iscsi_dhcp_params_en;
	port->defaults.iscsi_ipv4_dhcp_en		= nv_port_conf_def.iscsi_ipv4_dhcp_en;
	port->defaults.ip_ver					= nv_port_conf_def.ip_ver;
	port->defaults.linkup_timeout			= nv_port_conf_def.linkup_timeout;

	return 0;
}

static int flexboot_nodnic_get_ini_and_defaults ( struct flexboot_nodnic *flexboot_nodnic ) {
	nodnic_device_priv *device_priv = &flexboot_nodnic->device_priv;
	struct mlx_nvconfig_conf_defaults conf_def;
	struct mlx_nvcofnig_romini rom_ini;
	int rc, i;

	if ( ( rc = nvconfig_read_rom_ini_values ( flexboot_nodnic->device_priv.utils,
				   & rom_ini ) ) ) {
		DBGC_NODNIC ( flexboot_nodnic, "Failed to get ini values (rc = %d)\n", rc );
	} else {
		/* INI configurations  */
		memcpy ( flexboot_nodnic->ini_configurations.dhcp_user_class,
				& rom_ini.dhcp_user_class, sizeof ( rom_ini.dhcp_user_class ) );
		flexboot_nodnic->ini_configurations.uri_boot_retry_delay = rom_ini.uri_boot_retry_delay;
		flexboot_nodnic->ini_configurations.uri_boot_retry = rom_ini.uri_boot_retry;
		flexboot_nodnic->ini_configurations.option_rom_debug = rom_ini.option_rom_debug;
		flexboot_nodnic->ini_configurations.promiscuous_vlan = rom_ini.promiscuous_vlan;
	}
	memset ( &conf_def, 0, sizeof ( conf_def ) );
	/* Get the global default values */
	nvconfig_read_general_default_values ( flexboot_nodnic->device_priv.utils,
				&conf_def );
	/* Defaults configurations  */
	flexboot_nodnic->defaults.flexboot_menu_to = conf_def.flexboot_menu_to;
	/* Set max virtual functions
	* caps.max_funix contains 1 Pf, so the Vf number is (caps.max_funix - 1)
	*/
	flexboot_nodnic->defaults.max_vfs = conf_def.max_vfs;
	/* Get the ports default values */
	for ( i = 0 ; i < device_priv->device_cap.num_ports ; i++ ) {
		if ( ( rc = flexboot_nodnic_port_get_defaults ( flexboot_nodnic, i + 1 ) ) )
			return rc;
	}

	return 0;
}

int flexboot_nodnic_probe ( struct pci_device *pci,
		struct flexboot_nodnic_callbacks *callbacks,
		void *drv_priv ) {
	mlx_status status = MLX_SUCCESS;
	struct flexboot_nodnic *flexboot_nodnic_priv = NULL;
	nodnic_device_priv *device_priv = NULL;
	struct flexboot_nodnic_port *port = NULL;
	struct net_device *vlan = NULL;
	int i = 0;
	int rc = 0;

	if ( ( pci == NULL ) || ( callbacks == NULL ) ) {
		DBGC_NODNIC ( flexboot_nodnic_priv, "%s: Bad Parameter\n", __FUNCTION__ );
		return -EINVAL;
	}

	flexboot_nodnic_priv = zalloc( sizeof ( *flexboot_nodnic_priv ) );
	if ( flexboot_nodnic_priv == NULL ) {
		DBGC_NODNIC ( flexboot_nodnic_priv, "%s: Failed to allocate priv data\n", __FUNCTION__ );
		status = MLX_OUT_OF_RESOURCES;
		goto device_err_alloc;
	}

	if ( callbacks->get_settings && drv_priv )
		callbacks->get_settings ( flexboot_nodnic_priv, drv_priv );
	/* Register settings
	 * Note that pci->priv will be the device private data */
	flexboot_nodnic_priv->pci = pci;
	flexboot_nodnic_priv->callbacks = callbacks;
	pci_set_drvdata ( pci, flexboot_nodnic_priv );

	device_priv = &flexboot_nodnic_priv->device_priv;
	device_priv->utils = (mlx_utils *)zalloc( sizeof ( mlx_utils ) );
	if ( device_priv->utils == NULL ) {
		DBGC_NODNIC ( flexboot_nodnic_priv, "%s: Failed to allocate utils\n", __FUNCTION__ );
		status = MLX_OUT_OF_RESOURCES;
		goto utils_err_alloc;
	}

	status = mlx_utils_init( device_priv->utils, pci );
	MLX_FATAL_CHECK_STATUS(status, utils_init_err,
			"mlx_utils_init failed");

	/* nodnic init*/
	status = mlx_pci_gw_init( device_priv->utils );
	MLX_FATAL_CHECK_STATUS(status, cmd_init_err,
			"mlx_pci_gw_init failed");

	/* init device */
	status = nodnic_device_init( device_priv );
	MLX_FATAL_CHECK_STATUS(status, device_init_err,
				"nodnic_device_init failed");

	status = nodnic_device_get_cap( device_priv );
	MLX_FATAL_CHECK_STATUS(status, get_cap_err,
					"nodnic_device_get_cap failed");

	if ( ! flexboot_nodnic_priv->callbacks->get_settings ) {
		status = flexboot_nodnic_get_ini_and_defaults ( flexboot_nodnic_priv );
		MLX_FATAL_CHECK_STATUS(status, get_ini_and_def_err,
						"flexboot_nodnic_get_ini_and_defaults failed");
	}

	status =  flexboot_nodnic_set_port_masking ( flexboot_nodnic_priv );
	MLX_FATAL_CHECK_STATUS(status, err_set_masking,
						"flexboot_nodnic_set_port_masking failed");

	flexboot_nodnic_init_settings ( flexboot_nodnic_priv );
	MLX_FATAL_CHECK_STATUS(status, err_init_settings,
						"flexboot_nodnic_init_settings failed");

	status = flexboot_nodnic_allocate_infiniband_devices( flexboot_nodnic_priv );
	MLX_FATAL_CHECK_STATUS(status, err_alloc_ibdev,
					"flexboot_nodnic_allocate_infiniband_devices failed");

	/* port init */
	status = flexboot_nodnic_thin_init_ports( flexboot_nodnic_priv );
	MLX_FATAL_CHECK_STATUS(status, err_thin_init_ports,
						"flexboot_nodnic_thin_init_ports failed");

	/* device reg */
	status = flexboot_nodnic_set_ports_type( flexboot_nodnic_priv );
	MLX_CHECK_STATUS( flexboot_nodnic_priv, status, err_set_ports_types,
						"flexboot_nodnic_set_ports_type failed");

	status = flexboot_nodnic_ports_register_dev( flexboot_nodnic_priv );
	MLX_FATAL_CHECK_STATUS(status, reg_err,
					"flexboot_nodnic_ports_register_dev failed");

	for ( i = 0; i < device_priv->device_cap.num_ports; i++ ) {
		if ( ! ( flexboot_nodnic_priv->port_mask & ( i + 1 ) ) )
			continue;
		port = & ( flexboot_nodnic_priv->port[i] );
		flexboot_nodnic_init_port_settings ( flexboot_nodnic_priv, port );
		if ( ! flexboot_nodnic_priv->callbacks->get_settings ) {
			driver_settings_get_port_nvdata ( & ( port->driver_settings ), i + 1 );
			flexboot_nodnic_get_factory_mac ( flexboot_nodnic_priv, i );
			flexboot_nodnic_get_wol_conf ( flexboot_nodnic_priv );
		}
		driver_register_port_nv_settings ( & ( port->driver_settings ) );

		if ( ! boot_post_shell ) {
			DRIVER_STORE_INT_SETTING_EN ( flexboot_nodnic_priv, flexboot_nodnic_priv->ini_configurations.uri_boot_retry_delay,
					netdev_settings ( port->netdev ), &uriboot_retry_delay_setting );
			DRIVER_STORE_INT_SETTING_EN ( flexboot_nodnic_priv, flexboot_nodnic_priv->ini_configurations.uri_boot_retry,
					netdev_settings ( port->netdev ), &uriboot_retry_setting );
			DRIVER_STORE_INT_SETTING_EN ( flexboot_nodnic_priv, flexboot_nodnic_priv->ini_configurations.promiscuous_vlan,
					netdev_settings ( port->netdev ), &promisc_vlan_setting );
			DRIVER_STORE_INT_SETTING_EN (  flexboot_nodnic_priv, NETWORK_WAIT_TIMEOUT,
					netdev_settings ( port->netdev ), &network_wait_to_setting );

			/* Only IPv4 is supported in Infiniband */
			if ( port->port_priv.port_type != NODNIC_PORT_TYPE_ETH ) {
				DRIVER_STORE_INT_SETTING_EN ( flexboot_nodnic_priv, 1,
						netdev_settings ( port->netdev ), &dhcpv6_disabled_setting );
				storef_setting ( netdev_settings ( port->netdev ),
						&dhcpv4_disabled_setting, NULL );
			}
		}
	}

	if ( ! flexboot_nodnic_priv->callbacks->get_settings )
		driver_settings_get_nvdata ( & ( flexboot_nodnic_priv->driver_settings ) );
	driver_register_nv_settings ( & ( flexboot_nodnic_priv->driver_settings ) );

	if ( ! boot_post_shell ) {
		/* Create VLAN devices if needed */
		for ( i = 0 ; i < device_priv->device_cap.num_ports ; i++ ) {
			if ( ! ( flexboot_nodnic_priv->port_mask & ( i + 1 ) ) )
				continue;
			port = & ( flexboot_nodnic_priv->port[i] );

			if ( ( ! port->port_nv_conf.nic.boot_conf.en_vlan ) || ( port->port_priv.port_type != NODNIC_PORT_TYPE_ETH ) )
				continue;
			if ( ( rc = vlan_create ( port->netdev, port->port_nv_conf.nic.boot_conf.vlan_id, 1 ) ) )
				DBGC_NODNIC ( flexboot_nodnic_priv, "Failed to create VLAN device on port %d (rc = %d)\n", rc, i + 1 );
			else if ( ( vlan = vlan_find( port->netdev, port->port_nv_conf.nic.boot_conf.vlan_id ) ) )
				move_trunk_settings_to_vlan ( port->netdev, vlan );
			else
				DBGC_NODNIC ( flexboot_nodnic_priv, "Failed to find the VLAN device (rc = %d)\n", rc );
		}

		/* Register flexboot nodnic to the status update list */
		if ( ( rc = status_update_register_device ( flexboot_nodnic_priv, flexboot_nodnic_updater ) ) ) {
			DBGC_NODNIC ( flexboot_nodnic_priv, "Failed to register to the status updaters list (rc = %d)\n", rc );
		}
	}

	/* Update ETH operations with IRQ function if supported */
	DBGC_NODNIC ( flexboot_nodnic_priv, "%s: %s IRQ function\n",
			__FUNCTION__, ( callbacks->irq ? "Valid" : "No" ) );
	flexboot_nodnic_eth_operations.irq = callbacks->irq;
	return 0;

	flexboot_nodnic_ports_unregister_dev ( flexboot_nodnic_priv );
reg_err:
err_set_ports_types:
err_thin_init_ports:
err_alloc_ibdev:
	destroy_driver_settings ();
err_init_settings:
err_set_masking:
get_ini_and_def_err:
get_cap_err:
	nodnic_device_teardown ( device_priv );
device_init_err:
	mlx_pci_gw_teardown ( device_priv->utils );
cmd_init_err:
	mlx_utils_teardown(device_priv->utils);
utils_init_err:
	free ( device_priv->utils );
utils_err_alloc:
	free ( flexboot_nodnic_priv );
device_err_alloc:
	return status;
}

void flexboot_nodnic_remove ( struct pci_device *pci )
{
	struct flexboot_nodnic *flexboot_nodnic_priv = pci_get_drvdata ( pci );
	nodnic_device_priv *device_priv = & ( flexboot_nodnic_priv->device_priv );

	destroy_driver_settings ();
	status_update_unregister_device ( flexboot_nodnic_priv );
	flexboot_nodnic_ports_unregister_dev ( flexboot_nodnic_priv );
	nodnic_device_teardown( device_priv );
	mlx_pci_gw_teardown( device_priv->utils );
	mlx_utils_teardown(device_priv->utils);
	free( device_priv->utils );
	free( flexboot_nodnic_priv );
}
