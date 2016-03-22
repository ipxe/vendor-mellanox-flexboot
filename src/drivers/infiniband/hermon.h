#ifndef _HERMON_H
#define _HERMON_H

/** @file
 *
 * Mellanox Hermon Infiniband HCA driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdint.h>
#include <ipxe/uaccess.h>
#include <ipxe/ib_packet.h>
#include <ipxe/bofm.h>
#include <ipxe/nvsvpd.h>
#include <ipxe/nvo.h>
#include "mlx_bitops.h"
#include "MT25408_PRM.h"
#include <ipxe/ib_mad.h>
#include <ipxe/boot_menu_ui.h>
#include <ipxe/driver_settings.h>
#include "mlx_nodnic/include/prm/nodnic_hermon_prm.h"

#define TRUE	1
#define FALSE	!TRUE

/*
 * Hardware constants
 *
 */

/* Ports in existence */
#define HERMON_MAX_PORTS		2
#define HERMON_PORT_BASE		1
#define HERMON_MAX_VEPS			8

/* PCI BARs */
#define HERMON_PCI_CONFIG_BAR		PCI_BASE_ADDRESS_0
#define HERMON_PCI_CONFIG_BAR_SIZE	0x100000
#define HERMON_PCI_UAR_BAR		PCI_BASE_ADDRESS_2

/* Device reset */
#define HERMON_RESET_OFFSET		0x0f0010
#define HERMON_RESET_START		0x03000000UL
#define HERMON_RESET_END		0x80000001UL
#define HERMON_RESET_WAIT_TIME_MS	500

/* Work queue entry and completion queue entry opcodes */
#define HERMON_OPCODE_NOP		0x00
#define HERMON_OPCODE_SEND		0x0a
#define HERMON_OPCODE_CQE_ERROR	0x1e

/* HCA command register opcodes */
#define HERMON_HCR_QUERY_DEV_CAP	0x0003
#define HERMON_HCR_QUERY_FW		0x0004
#define HERMON_HCR_QUERY_ADAPTER	0x0006
#define HERMON_HCR_INIT_HCA		0x0007
#define HERMON_HCR_CLOSE_HCA		0x0008
#define HERMON_HCR_INIT_PORT		0x0009
#define HERMON_HCR_CLOSE_PORT		0x000a
#define HERMON_HCR_SET_PORT		0x000c
#define HERMON_HCR_SW2HW_MPT		0x000d
#define HERMON_HCR_HW2SW_MPT		0x000f
#define HERMON_HCR_WRITE_MTT		0x0011
#define HERMON_HCR_MAP_EQ		0x0012
#define HERMON_HCR_SW2HW_EQ		0x0013
#define HERMON_HCR_HW2SW_EQ		0x0014
#define HERMON_HCR_QUERY_EQ		0x0015
#define HERMON_HCR_SW2HW_CQ		0x0016
#define HERMON_HCR_HW2SW_CQ		0x0017
#define HERMON_HCR_QUERY_CQ		0x0018
#define HERMON_HCR_RST2INIT_QP		0x0019
#define HERMON_HCR_INIT2RTR_QP		0x001a
#define HERMON_HCR_RTR2RTS_QP		0x001b
#define HERMON_HCR_RTS2RTS_QP		0x001c
#define HERMON_HCR_2RST_QP		0x0021
#define HERMON_HCR_QUERY_QP		0x0022
#define HERMON_HCR_CONF_SPECIAL_QP	0x0023
#define HERMON_HCR_MAD_IFC		0x0024
#define HERMON_HCR_READ_MCG		0x0025
#define HERMON_HCR_WRITE_MCG		0x0026
#define HERMON_HCR_MGID_HASH		0x0027
#define HERMON_HCR_MOD_STAT_CFG		0x0034
#define HERMON_HCR_REGISTER_ACCESS	0x003b
#define HERMON_HCR_QUERY_PORT		0x0043
#define HERMON_HCR_SENSE_PORT		0x004d
#define HERMON_GET_OP_REQ               0x0059
#define HERMON_HCR_POST_DOORBELL        0x0062
#define HERMON_HCR_RUN_FW		0x0ff6
#define HERMON_HCR_DISABLE_LAM		0x0ff7
#define HERMON_HCR_ENABLE_LAM		0x0ff8
#define HERMON_HCR_UNMAP_ICM		0x0ff9
#define HERMON_HCR_MAP_ICM		0x0ffa
#define HERMON_HCR_UNMAP_ICM_AUX	0x0ffb
#define HERMON_HCR_MAP_ICM_AUX		0x0ffc
#define HERMON_HCR_SET_ICM_SIZE		0x0ffd
#define HERMON_HCR_UNMAP_FA		0x0ffe
#define HERMON_HCR_MAP_FA		0x0fff
#define HERMON_HCR_NOP			0x31
#define HERMON_HCR_MAILBOX_READ		0x71
#define HERMON_HCR_QUERY_ROMINI		0x72
#define HERMON_HCR_QUERY_DEFPARAMS	0x73
#define HERMON_HCR_INIT_DIAG_BUFFER	0x5F

/* Service types */
#define HERMON_ST_RC			0x00
#define HERMON_ST_UD			0x03
#define HERMON_ST_MLX			0x07

/* Port types */
#define HERMON_PORT_TYPE_UNKNOWN	0
#define HERMON_PORT_TYPE_IB		1
#define HERMON_PORT_TYPE_ETH		2

/* Flash access (TLV) */
#define HERMON_REGISTER_ID_SET_GET 		0x9024
#define HERMON_REGISTER_ID_INVALIDATE		0x9025
#define HERMON_REGISTER_ACCESS_DATA_OFFSET	0x1c
#define HERMON_NUM_OF_TLV_PER_PORT		2

/* Port protocol */
enum hermon_protocol {
        HERMON_PROT_IB_IPV6 = 0,
        HERMON_PROT_ETH,
        HERMON_PROT_IB_IPV4,
        HERMON_PROT_FCOE
};

/* Steering types */
enum hermon_steer_type {
        MLX4_MC_STEER = 0,
        MLX4_UC_STEER,
        MLX4_NUM_STEERS
};
#define MC_TABLE_SIZE			128

/* MTUs */
#define HERMON_MTU_2048			0x04
#define HERMON_MTU_ETH			0x07

#define HERMON_INVALID_LKEY		0x00000100UL

#define HERMON_PAGE_SIZE		( ( size_t ) 4096 )

#define HERMON_DB_POST_SND_OFFSET	0x14
#define HERMON_DB_EQ_OFFSET(_eqn)	\
	( 0x800 + HERMON_PAGE_SIZE * ( (_eqn) / 4 ) + 0x08 * ( (_eqn) % 4 ) )

#define HERMON_QP_OPT_PARAM_PM_STATE	0x00000400UL
#define HERMON_QP_OPT_PARAM_QKEY	0x00000020UL
#define HERMON_QP_OPT_PARAM_ALT_PATH	0x00000001UL

#define HERMON_MAP_EQ			( 0UL << 31 )
#define HERMON_UNMAP_EQ			( 1UL << 31 )

#define HERMON_SET_PORT_GENERAL_PARAM	0x0000
#define HERMON_SET_PORT_RECEIVE_QP	0x0100
#define HERMON_SET_PORT_MAC_TABLE	0x0200
#define HERMON_SET_PORT_VLAN_TABLE	0x0300
#define HERMON_SET_PORT_PRIORITY_TABLE	0x0400
#define HERMON_SET_PORT_GID_TABLE	0x0500

/* Hermon event types */
#define HERMON_EV_CQ_COMPLETION		0x00
#define HERMON_EV_PORT_STATE_CHANGE	0x09
#define HERMON_EV_CMD_COMPLETION	0x0a
#define HERMON_EV_TYPE_OP_REQUIRED	0x1a
#define HERMON_EV_PORT_MGMNT_CHANGE     0x1d

/* Hermon operation required types */
#define HERMON_OPREQ_ADD_TO_MCG		0x26

#define HERMON_SCHED_QP0		0x3f
#define HERMON_SCHED_DEFAULT		0x83

#define HERMON_LOG_MULTICAST_HASH_SIZE	7

#define HERMON_PM_STATE_ARMED		0x00
#define HERMON_PM_STATE_REARM		0x01
#define HERMON_PM_STATE_MIGRATED	0x03

#define HERMON_RETRY_MAX		0x07

#define HERMON_MOD_STAT_CFG_SET		0x01
#define HERMON_MOD_STAT_CFG_QUERY	0x03
#define HERMON_MAX_MGM_MEMBERS		8

enum hermon_mod_stat_cfg_setup_mode {
        HERMON_SETUP_MODE_LANE_SETUP = 0,
        HERMON_SETUP_MODE_PORT_SETUP = 1,
        HERMON_SETUP_MODE_VEP = 2,
        HERMON_SETUP_MODE_PORT_PRIORITY = 3,
        HERMON_SETUP_MODE_PORT_PRIORITY_GROUP = 4,
        HERMON_SETUP_MODE_RATE_LIMITER = 5,
        HERMON_SETUP_MODE_SENSOR_SETUP = 6,
};

#define HERMON_VPD_FIELD( port ) \
	PCI_VPD_FIELD ( PCI_VPD_TAG_RW, 'V', ( '5' + (port) - 1 ) )

/*
 * Datatypes that seem to be missing from the autogenerated documentation
 *
 */
struct hermonprm_mgm_hash_st {
	pseudo_bit_t reserved0[0x00020];
/* -------------- */
	pseudo_bit_t hash[0x00010];
	pseudo_bit_t reserved1[0x00010];
} __attribute__ (( packed ));

struct hermonprm_mcg_entry_st {
	struct hermonprm_mcg_hdr_st hdr;
	struct hermonprm_mcg_qp_dw_st qp[8];
} __attribute__ (( packed ));

struct hermonprm_cq_db_record_st {
	pseudo_bit_t update_ci[0x00018];
	pseudo_bit_t reserved0[0x00008];
/* -------------- */
	pseudo_bit_t arm_ci[0x00018];
	pseudo_bit_t cmd[0x00003];
	pseudo_bit_t reserved1[0x00001];
	pseudo_bit_t cmd_sn[0x00002];
	pseudo_bit_t reserved2[0x00002];
} __attribute__ (( packed ));

struct hermonprm_send_db_register_st {
	pseudo_bit_t reserved[0x00008];
	pseudo_bit_t qn[0x00018];
} __attribute__ (( packed ));

struct hermonprm_event_db_register_st {
	pseudo_bit_t ci[0x00018];
	pseudo_bit_t reserver[0x00007];
	pseudo_bit_t a[0x00001];
} __attribute__ (( packed ));

struct hermonprm_scalar_parameter_st {
	pseudo_bit_t value_hi[0x00020];
/* -------------- */
	pseudo_bit_t value[0x00020];
} __attribute__ (( packed ));

struct hermonprm_event_mask_st {
	pseudo_bit_t reserved0[0x00020];
/* -------------- */
	pseudo_bit_t completion[0x00001];
	pseudo_bit_t path_migration_succeeded[0x00001];
	pseudo_bit_t communication_established[0x00001];
	pseudo_bit_t send_queue_drained[0x00001];
	pseudo_bit_t cq_error[0x00001];
	pseudo_bit_t wq_catastrophe[0x00001];
	pseudo_bit_t qpc_catastrophe[0x00001];
	pseudo_bit_t path_migration_failed[0x00001];
	pseudo_bit_t internal_error[0x00001];
	pseudo_bit_t port_state_change[0x00001];
	pseudo_bit_t command_done[0x00001];
	pseudo_bit_t fexch_error[0x00001];
	pseudo_bit_t reserved1[0x00004];
	pseudo_bit_t wq_invalid_request[0x00001];
	pseudo_bit_t wq_access_violation[0x00001];
	pseudo_bit_t srq_catastrophe[0x00001];
	pseudo_bit_t srq_last_wqe[0x00001];
	pseudo_bit_t srq_rq_limit[0x00001];
	pseudo_bit_t gpio[0x00001];
	pseudo_bit_t clientreregister[0x00001];
	pseudo_bit_t reserved2[0x00003];
	pseudo_bit_t operation_required[0x00001];
	pseudo_bit_t reserved3[0x00005];
} __attribute__ (( packed ));

struct hermonprm_port_state_change_event_st {
	pseudo_bit_t reserved[0x00020];
/* -------------- */
	struct hermonprm_port_state_change_st data;
} __attribute__ (( packed ));

struct hermonprm_completion_event_st {
	pseudo_bit_t reserved[0x00020];
/* -------------- */
	struct hermonprm_completion_event_data_st data;
} __attribute__ (( packed ));

struct hermonprm_port_mgmt_change_event_st {
	pseudo_bit_t reserved[0x00020];
/* -------------- */
	pseudo_bit_t       port[0x00008];
	pseudo_bit_t       reserved0[0x00018];
} __attribute__ (( packed ));

struct hermonprm_sense_port_st {
	pseudo_bit_t reserved0[0x00020];
/* -------------- */
	pseudo_bit_t port_type[0x00002];
	pseudo_bit_t reserved1[0x0001e];
} __attribute__ (( packed ));

struct hermonprm_set_port_ib_st {
	pseudo_bit_t rqk[0x00001];
	pseudo_bit_t rcm[0x00001];
	pseudo_bit_t reserved0[0x00002];
	pseudo_bit_t vl_cap[0x00004];
	pseudo_bit_t reserved1[0x00004];
	pseudo_bit_t mtu_cap[0x00004];
	pseudo_bit_t g0[0x00001];
	pseudo_bit_t ng[0x00001];
	pseudo_bit_t sig[0x00001];
	pseudo_bit_t mg[0x00001];
	pseudo_bit_t mp[0x00001];
	pseudo_bit_t mvc[0x00001];
	pseudo_bit_t mmc[0x00001];
	pseudo_bit_t reserved2[0x00004];
	pseudo_bit_t lws[0x00001];
	pseudo_bit_t lss[0x00001];
	pseudo_bit_t reserved3[0x00003];
/* -------------- */
	pseudo_bit_t capability_mask[0x00020];
/* -------------- */
	pseudo_bit_t system_image_guid_h[0x00020];
/* -------------- */
	pseudo_bit_t system_image_guid_l[0x00020];
/* -------------- */
	pseudo_bit_t guid0_h[0x00020];
/* -------------- */
	pseudo_bit_t guid0_l[0x00020];
/* -------------- */
	pseudo_bit_t node_guid_h[0x00020];
/* -------------- */
	pseudo_bit_t node_guid_l[0x00020];
/* -------------- */
	pseudo_bit_t egress_sniff_qpn[0x00018];
	pseudo_bit_t egress_sniff_mode[0x00002];
	pseudo_bit_t reserved4[0x00006];
/* -------------- */
	pseudo_bit_t ingress_sniff_qpn[0x00018];
	pseudo_bit_t ingress_sniff_mode[0x00002];
	pseudo_bit_t reserved5[0x00006];
/* -------------- */
	pseudo_bit_t max_gid[0x00010];
	pseudo_bit_t max_pkey[0x00010];
/* -------------- */
	pseudo_bit_t reserved6[0x00020];
/* -------------- */
	pseudo_bit_t reserved7[0x00020];
/* -------------- */
	pseudo_bit_t reserved8[0x00020];
/* -------------- */
	pseudo_bit_t reserved9[0x00020];
/* -------------- */
	pseudo_bit_t reserved10[0x00020];
/* -------------- */
	pseudo_bit_t reserved11[0x00020];
/* -------------- */
	pseudo_bit_t reserved12[0x00020];
/* -------------- */
	pseudo_bit_t reserved13[0x00020];
/* -------------- */
	pseudo_bit_t reserved14[0x00020];
/* -------------- */
	pseudo_bit_t reserved15[0x00020];
/* -------------- */
	pseudo_bit_t reserved16[0x00020];
/* -------------- */
	pseudo_bit_t reserved17[0x00020];
/* -------------- */
	pseudo_bit_t reserved18[0x00020];
/* -------------- */
	pseudo_bit_t reserved19[0x00020];
/* -------------- */
	pseudo_bit_t reserved20[0x00020];
/* -------------- */
	pseudo_bit_t reserved21[0x00020];
/* -------------- */
	pseudo_bit_t reserved22[0x00020];
/* -------------- */
	pseudo_bit_t link_width_supported[0x00004];
	pseudo_bit_t link_speed_supported[0x00004];
	pseudo_bit_t reserved23[0x00018];
/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_query_port_cap_st {
	pseudo_bit_t eth_mtu[0x00010];
	pseudo_bit_t ib_mtu[0x00004];
	pseudo_bit_t reserved0[0x00004];
	pseudo_bit_t ib[0x00001];
	pseudo_bit_t eth[0x00001];
	pseudo_bit_t fc[0x00001];
	pseudo_bit_t default_link_type[0x00001];
	pseudo_bit_t reserved1[0x00003];
	pseudo_bit_t link_state[0x00001];
/* -------------- */
	pseudo_bit_t log_max_pkey[0x00004];
	pseudo_bit_t log_max_gid[0x00004];
	pseudo_bit_t ib_port_width[0x00004];
	pseudo_bit_t reserved2[0x00004];
	pseudo_bit_t eth_link_speed[0x00004];
	pseudo_bit_t reserved3[0x00004];
	pseudo_bit_t ib_link_speed[0x00004];
	pseudo_bit_t reserved4[0x00004];
/* -------------- */
	pseudo_bit_t max_vl_ib[0x00004];
	pseudo_bit_t reserved5[0x00004];
	pseudo_bit_t log_max_mac[0x00004];
	pseudo_bit_t log_max_vlan[0x00004];
	pseudo_bit_t reserved6[0x00010];
/* -------------- */
	pseudo_bit_t reserved7[0x00020];
/* -------------- */
	pseudo_bit_t mac_47_32[0x00010];
	pseudo_bit_t reserved8[0x00010];
/* -------------- */
	pseudo_bit_t mac_31_0[0x00020];
/* -------------- */
	pseudo_bit_t vendor_oui[0x00018];
	pseudo_bit_t transceiver_type[0x00008];
/* -------------- */
	pseudo_bit_t reserved9[0x00010];
	pseudo_bit_t wavelength[0x00010];
/* -------------- */
	pseudo_bit_t transceiver_code_hi[0x00020];
/* -------------- */
	pseudo_bit_t transceiver_code_lo[0x00020];
/* -------------- */
	pseudo_bit_t fac_mac_47_32[0x00010];
	pseudo_bit_t reserved10[0x00010];
/* -------------- */
	pseudo_bit_t fac_mac_31_0[0x00020];
/* -------------- */
	pseudo_bit_t reserved11[0x00080];
} __attribute__ (( packed ));

struct hermonprm_set_port_general_context_st {
	pseudo_bit_t v_mtu[0x00001];
	pseudo_bit_t v_pprx[0x00001];
	pseudo_bit_t v_pptx[0x00001];
	pseudo_bit_t reserved0[0x0001d];
/* -------------- */
	pseudo_bit_t mtu[0x00010];
	pseudo_bit_t reserved1[0x00010];
/* -------------- */
	pseudo_bit_t reserved2[0x00010];
	pseudo_bit_t pfctx[0x00008];
	pseudo_bit_t reserved3[0x00007];
	pseudo_bit_t pptx[0x00001];
/* -------------- */
	pseudo_bit_t reserved4[0x00010];
	pseudo_bit_t pfcrx[0x00008];
	pseudo_bit_t reserved5[0x00007];
	pseudo_bit_t pprx[0x00001];
/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_set_port_rqp_calc_st {
	pseudo_bit_t base_qpn[0x00018];
	pseudo_bit_t reserved0[0x00008];
/* -------------- */
	pseudo_bit_t n_p[0x00002];
	pseudo_bit_t reserved1[0x00006];
	pseudo_bit_t n_v[0x00003];
	pseudo_bit_t reserved2[0x00005];
	pseudo_bit_t n_m[0x00004];
	pseudo_bit_t reserved3[0x0000c];
/* -------------- */
	pseudo_bit_t mac_miss_index[0x00008];
	pseudo_bit_t reserved4[0x00018];
/* -------------- */
	pseudo_bit_t vlan_miss_index[0x00007];
	pseudo_bit_t reserved5[0x00008];
	pseudo_bit_t intra_miss[0x00001];
	pseudo_bit_t no_vlan_index[0x00007];
	pseudo_bit_t reserved6[0x00008];
	pseudo_bit_t intra_no_vlan[0x00001];
/* -------------- */
	pseudo_bit_t no_vlan_prio[0x00003];
	pseudo_bit_t reserved7[0x0001d];
/* -------------- */
	pseudo_bit_t promisc_qpn[0x00018];
	pseudo_bit_t reserved8[0x00007];
	pseudo_bit_t en_uc_promisc[0x00001];
/* -------------- */
	pseudo_bit_t def_mcast_qpn[0x00018];
	pseudo_bit_t reserved9[0x00005];
	pseudo_bit_t mc_by_vlan[0x00001];
	pseudo_bit_t mc_promisc_mode[0x00002];
/* -------------- */
	pseudo_bit_t reserved10[0x00020];
/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_set_port_mac_table_st {
	pseudo_bit_t mac_h[0x00010];
	pseudo_bit_t mac_vep[0x00008];
	pseudo_bit_t reserved0[0x00007];
	pseudo_bit_t v[0x00001];
/* -------------- */
	pseudo_bit_t mac_l[0x00020];
/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_set_port_vlan_st {
	pseudo_bit_t vlan_id[0x0000c];
	pseudo_bit_t reserved0[0x00012];
	pseudo_bit_t intra[0x00001];
	pseudo_bit_t v[0x00001];
/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_set_port_beacon_st {
	pseudo_bit_t duration[0x0000f];
	pseudo_bit_t reserved0[0x0000e];
	pseudo_bit_t version[0x00002];
/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_mod_stat_cfg_input_mod_st {
	pseudo_bit_t offset[0x00008];
	pseudo_bit_t portnum[0x00008];
	pseudo_bit_t physical_function[0x00008];
	pseudo_bit_t reserved[0x00004];
	pseudo_bit_t setup_mode[0x00004];
} __attribute__ (( packed ));

struct hermonprm_mod_stat_cfg_pf_net_boot_st {
	pseudo_bit_t reserved0[0x00010];
	pseudo_bit_t funix[0x00008];
	pseudo_bit_t reserved1[0x00004];
	pseudo_bit_t pf_net_boot[0x00001];
	pseudo_bit_t reserved2[0x00002];
	pseudo_bit_t pf_net_boot_m[0x00001];
/* -------------- */
	pseudo_bit_t reserved3[0x00020];
} __attribute__ (( packed ));

struct hermonprm_mod_stat_cfg_port_setup_st {
	pseudo_bit_t port_protocol[0x00008];
	pseudo_bit_t reserved0[0x00007];
	pseudo_bit_t port_protocol_m[0x00001];
	pseudo_bit_t num_port[0x00008];
	pseudo_bit_t reserved1[0x00008];
/* -------------- */
	pseudo_bit_t reserved2[0x00020];
} __attribute__ (( packed ));

struct hermonprm_mod_stat_cfg_num_veps_st {
	pseudo_bit_t port_control_vlan[0x0000c];
	pseudo_bit_t reserved0[0x00003];
	pseudo_bit_t port_control_vlan_m[0x00001];
	pseudo_bit_t num_veps[0x00008];
	pseudo_bit_t reserved1[0x00008];
/* -------------- */
	pseudo_bit_t	reserved2[0x00020];
} __attribute__ (( packed ));

struct hermonprm_get_op_req_st {
        pseudo_bit_t	reserved0[0x00020];
        /* -------------- */
        pseudo_bit_t	reserved1[0x00020];
        /* -------------- */
        pseudo_bit_t	request_modifier[0x00020];
        /* -------------- */
        pseudo_bit_t	reserved2[0x00020];
        /* -------------- */
	pseudo_bit_t	reserved3[0x00020];
        /* -------------- */
        pseudo_bit_t	reserved4[0x00010];
        pseudo_bit_t	request_token[0x00010];
        /* -------------- */
        pseudo_bit_t	request_type[0x0000c];
        pseudo_bit_t	request_type_modifier[0x00004];
        /* -------------- */
        pseudo_bit_t	reserved5[0x00020];
	/* -------------- */
	struct 		hermonprm_mcg_entry_st mcg;
} 	__attribute__ (( packed ));

struct hermonprm_eth_mgi_st {
	pseudo_bit_t	reserved0[0x00020];
/* -------------- */
	pseudo_bit_t	vlan_present[0x00001];
	pseudo_bit_t	unicast[0x00001];
	pseudo_bit_t	reserved1[0x0000e];
	pseudo_bit_t	portnum[0x00008];
	pseudo_bit_t	vep_num[0x00008];
/* -------------- */
	pseudo_bit_t	mac_h[0x00010];
	pseudo_bit_t	vlan_id[0x0000c];
	pseudo_bit_t	reserved2[0x00003];
	pseudo_bit_t	vlan_check[0x00001];
/* -------------- */
	pseudo_bit_t	mac_l[0x00020];
/* -------------- */
} 	__attribute__ (( packed ));

struct hermonprm_register_access_st {	/* Little Endian */
/* -------------- */ /* Operation TLV*/
    pseudo_bit_t	reserved0[0x00008];
    pseudo_bit_t	status[0x00007];
	pseudo_bit_t	dr[0x00001];
	pseudo_bit_t	len[0x0000b];
	pseudo_bit_t	op_type[0x0005];

/* -------------- */
    pseudo_bit_t	class[0x00008];
    pseudo_bit_t	method[0x00007];
    pseudo_bit_t	r[0x00001];
    pseudo_bit_t	register_id[0x00010];
/* -------------- */
    pseudo_bit_t	tid_h[0x00020];
    pseudo_bit_t	tid_l[0x00020];
/* -------------- */ /* Register TLV*/
	pseudo_bit_t	reg_tlv_res[0x00010];
	pseudo_bit_t	reg_tlv_len[0x0000b];
	pseudo_bit_t	reg_tlv_type[0x0005];
/* -------------- */ /* TLV Header*/
    pseudo_bit_t	tlv_hdr_reg_type[0x00010];
    pseudo_bit_t	tlv_hdr_header[0x0002];
    pseudo_bit_t	tlv_hdr_length[0x0000c];
    pseudo_bit_t	tlv_hdr_valid[0x00002];

    pseudo_bit_t	tlv_hdr_crc[0x00010];
    pseudo_bit_t	tlv_hdr_type_mod[0x00008];
    pseudo_bit_t	tlv_hdr_shadow[0x00001];
    pseudo_bit_t	tlv_hdr_pad_cnt[0x00002];
    pseudo_bit_t	tlv_hdr_type_res[0x00001];
    pseudo_bit_t	tlv_hdr_type_ver[0x00004];
/* -------------- */
    pseudo_bit_t	data[0x02000];
} __attribute__ (( packed ));

struct hermonprm_query_romini_st {
	pseudo_bit_t	reserved0[0x00018];
	pseudo_bit_t	romini_payload_version[0x00008];
	/* -------------- */
	pseudo_bit_t	reserved1[0x00004];
	pseudo_bit_t	one_adapter[0x00001];
	pseudo_bit_t	ocbb_en[0x00001];
	pseudo_bit_t	ocsd_en[0x00001];
	pseudo_bit_t	bind_driver_only_to_its_device[0x00001];
	pseudo_bit_t	static_component_name_string[0x00001];
	pseudo_bit_t	hii_iscsi_configuration[0x00001];
	pseudo_bit_t	hii_ibm_aim[0x00001];
	pseudo_bit_t	hii_platform_setup[0x00001];
	pseudo_bit_t	hii_bdf_decimal[0x00001];
	pseudo_bit_t	hii_read_only[0x00001];
	pseudo_bit_t	hii_device_name[0x00008];
	pseudo_bit_t	dev_path_fqdd[0x00001];
	pseudo_bit_t	dev_path_enum[0x00001];
	pseudo_bit_t	mac_enum[0x00001];
	pseudo_bit_t	port_enum[0x00001];
	pseudo_bit_t	flash_en[0x00001];
	pseudo_bit_t	fmp_en[0x00001];
	pseudo_bit_t	bofm_en[0x00001];
	pseudo_bit_t	platform_to_driver_en[0x00001];
	pseudo_bit_t	hii_en[0x00001];
	pseudo_bit_t	undi_en[0x00001];
	/* -------------- */
	pseudo_bit_t	dhcp_user_class_3[0x00008];
	pseudo_bit_t	dhcp_user_class_2[0x00008];
	pseudo_bit_t	dhcp_user_class_1[0x00008];
	pseudo_bit_t	dhcp_user_class_0[0x00008];
	/* -------------- */
	pseudo_bit_t	dhcp_user_class_7[0x00008];
	pseudo_bit_t	dhcp_user_class_6[0x00008];
	pseudo_bit_t	dhcp_user_class_5[0x00008];
	pseudo_bit_t	dhcp_user_class_4[0x00008];
	/* -------------- */
	pseudo_bit_t	reserved2[0x00016];
	pseudo_bit_t	uri_boot_retry_delay[0x00004];
	pseudo_bit_t	uri_boot_retry[0x00004];
	pseudo_bit_t	option_rom_debug[0x00001];
	pseudo_bit_t	promiscuous_vlan[0x00001];
	/* -------------- */
	pseudo_bit_t	reserved3[0x00020];
	/* -------------- */
	pseudo_bit_t	reserved4[0x00020];
	/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_query_defparams_global_st {
	pseudo_bit_t	reserved0[0x00018];
	pseudo_bit_t	fw_default_config_payload_version[0x00008];
	/* -------------- */
	pseudo_bit_t	num_vfs[0x00008];
	pseudo_bit_t	uar_bar_size[0x00004];
	pseudo_bit_t	max_uar_bar_size[0x00004];
	pseudo_bit_t	flexboot_menu_to[0x00004];
	pseudo_bit_t	reserved1[0x0000b];
	pseudo_bit_t	sriov_en[0x00001];
	/* -------------- */
	pseudo_bit_t	reserved2[0x00020];
	/* -------------- */
	pseudo_bit_t	reserved3[0x00020];
	/* -------------- */
	pseudo_bit_t	reserved4[0x00020];
	/* -------------- */
} __attribute__ (( packed ));

struct hermonprm_query_defparams_port_st {
	pseudo_bit_t	reserved0[0x00018];
	pseudo_bit_t	fw_default_config_payload_version[0x00008];
	/* -------------- */
	pseudo_bit_t	boot_vlan_id[0x0000c];
	pseudo_bit_t	reserved1[0x00004];
	pseudo_bit_t	boot_protocol[0x00004];
	pseudo_bit_t	boot_retry_count[0x00004];
	pseudo_bit_t	reserved2[0x00001];
	pseudo_bit_t	network_link_type[0x00002];
	pseudo_bit_t	en_wol_magic[0x00001];
	pseudo_bit_t	boot_vlan_en[0x00001];
	pseudo_bit_t	boot_option_rom_en[0x00001];
	pseudo_bit_t	pprx[0x00001];
	pseudo_bit_t	pptx[0x00001];
	/* -------------- */
	pseudo_bit_t	boot_pkey[0x00010];
	pseudo_bit_t	reserved3[0x00010];
	/* -------------- */
	pseudo_bit_t	reserved4[0x00016];
	pseudo_bit_t	iscsi_boot_to_target[0x00002];
	pseudo_bit_t	reserved5[0x00002];
	pseudo_bit_t	iscsi_vlan_en[0x00001];
	pseudo_bit_t	iscsi_tcp_timestamps_en[0x00001];
	pseudo_bit_t	iscsi_chap_mutual_auth_en[0x00001];
	pseudo_bit_t	iscsi_chap_auth_en[0x00001];
	pseudo_bit_t	iscsi_dhcp_params_en[0x00001];
	pseudo_bit_t	iscsi_ipv4_dhcp_en[0x00001];
	/* -------------- */
	pseudo_bit_t	iscsi_lun_busy_retry_count[0x00008];
	pseudo_bit_t	iscsi_link_up_delay_time[0x00008];
	pseudo_bit_t	reserved6[0x00010];
	/* -------------- */
	pseudo_bit_t	reserved7[0x00020];
	/* -------------- */
	pseudo_bit_t	reserved8[0x00020];
	/* -------------- */
} __attribute__ (( packed ));

/*
 * Wrapper structures for hardware datatypes
 *
 */

struct MLX_DECLARE_STRUCT ( hermonprm_completion_queue_context );
struct MLX_DECLARE_STRUCT ( hermonprm_cq_db_record );
struct MLX_DECLARE_STRUCT ( hermonprm_eqc );
struct MLX_DECLARE_STRUCT ( hermonprm_event_db_register );
struct MLX_DECLARE_STRUCT ( hermonprm_event_mask );
struct MLX_DECLARE_STRUCT ( hermonprm_event_queue_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_hca_command_register );
struct MLX_DECLARE_STRUCT ( hermonprm_init_hca );
struct MLX_DECLARE_STRUCT ( hermonprm_mad_ifc );
struct MLX_DECLARE_STRUCT ( hermonprm_mcg_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_mgm_hash );
struct MLX_DECLARE_STRUCT ( hermonprm_mod_stat_cfg );
struct MLX_DECLARE_STRUCT ( hermonprm_mod_stat_cfg_input_mod );
struct MLX_DECLARE_STRUCT ( hermonprm_mod_stat_cfg_num_veps );
struct MLX_DECLARE_STRUCT ( hermonprm_mod_stat_cfg_pf_net_boot );
struct MLX_DECLARE_STRUCT ( hermonprm_mod_stat_cfg_port_setup );
struct MLX_DECLARE_STRUCT ( hermonprm_mpt );
struct MLX_DECLARE_STRUCT ( hermonprm_mtt );
struct MLX_DECLARE_STRUCT ( hermonprm_port_state_change_event );
struct MLX_DECLARE_STRUCT ( hermonprm_completion_event );
struct MLX_DECLARE_STRUCT ( hermonprm_port_mgmt_change_event );
struct MLX_DECLARE_STRUCT ( hermonprm_qp_db_record );
struct MLX_DECLARE_STRUCT ( hermonprm_qp_ee_state_transitions );
struct MLX_DECLARE_STRUCT ( hermonprm_query_dev_cap );
struct MLX_DECLARE_STRUCT ( hermonprm_query_fw );
struct MLX_DECLARE_STRUCT ( hermonprm_query_adapter );
struct MLX_DECLARE_STRUCT ( hermonprm_query_port_cap );
struct MLX_DECLARE_STRUCT ( hermonprm_queue_pair_ee_context_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_scalar_parameter );
struct MLX_DECLARE_STRUCT ( hermonprm_sense_port );
struct MLX_DECLARE_STRUCT ( hermonprm_send_db_register );
struct MLX_DECLARE_STRUCT ( hermonprm_set_port_ib );
struct MLX_DECLARE_STRUCT ( hermonprm_set_port_general_context );
struct MLX_DECLARE_STRUCT ( hermonprm_set_port_mac_table );
struct MLX_DECLARE_STRUCT ( hermonprm_set_port_rqp_calc );
struct MLX_DECLARE_STRUCT ( hermonprm_set_port_vlan );
struct MLX_DECLARE_STRUCT ( hermonprm_set_port_beacon );
struct MLX_DECLARE_STRUCT ( hermonprm_ud_address_vector );
struct MLX_DECLARE_STRUCT ( hermonprm_virtual_physical_mapping );
struct MLX_DECLARE_STRUCT ( hermonprm_wqe_segment_ctrl_mlx );
struct MLX_DECLARE_STRUCT ( hermonprm_wqe_segment_data_ptr );
struct MLX_DECLARE_STRUCT ( hermonprm_wqe_segment_ud );
struct MLX_DECLARE_STRUCT ( hermonprm_get_op_req );
struct MLX_DECLARE_STRUCT ( hermonprm_eth_mgi );
struct MLX_DECLARE_STRUCT ( hermonprm_register_access );
struct MLX_DECLARE_STRUCT ( hermonprm_query_romini );
struct MLX_DECLARE_STRUCT ( hermonprm_query_defparams_global );
struct MLX_DECLARE_STRUCT ( hermonprm_query_defparams_port );

/*
 * Composite hardware datatypes
 *
 */

union hermonprm_query_defparams {
	struct hermonprm_query_defparams_global global;
	struct hermonprm_query_defparams_port port;
};

struct hermonprm_write_mtt {
	struct hermonprm_scalar_parameter mtt_base_addr;
	struct hermonprm_scalar_parameter reserved;
	struct hermonprm_mtt mtt;
} __attribute__ (( packed ));

struct hermonprm_ud_send_wqe {
	struct hermonprm_wqe_segment_ctrl_send ctrl;
	struct hermonprm_wqe_segment_ud ud;
	struct hermonprm_wqe_segment_data_ptr data[HERMON_MAX_GATHER];
} __attribute__ (( packed ));

struct hermonprm_mlx_send_wqe {
	struct hermonprm_wqe_segment_ctrl_mlx ctrl;
	struct hermonprm_wqe_segment_data_ptr data[HERMON_MAX_GATHER];
	uint8_t headers[IB_MAX_HEADER_SIZE];
} __attribute__ (( packed ));

struct hermonprm_rc_send_wqe {
	struct hermonprm_wqe_segment_ctrl_send ctrl;
	struct hermonprm_wqe_segment_data_ptr data[HERMON_MAX_GATHER];
} __attribute__ (( packed ));

struct hermonprm_eth_send_wqe {
	struct hermonprm_wqe_segment_ctrl_send ctrl;
	struct hermonprm_wqe_segment_data_ptr data[HERMON_MAX_GATHER];
} __attribute__ (( packed ));

#define HERMON_MAX_SCATTER 1

struct hermonprm_recv_wqe {
	struct hermonprm_wqe_segment_data_ptr data[HERMON_MAX_SCATTER];
} __attribute__ (( packed ));

union hermonprm_event_entry {
	struct hermonprm_event_queue_entry generic;
	struct hermonprm_port_state_change_event port_state_change;
	struct hermonprm_port_mgmt_change_event port_mgmt_change;
	struct hermonprm_completion_event completion;
} __attribute__ (( packed ));

union hermonprm_doorbell_register {
	struct hermonprm_send_db_register send;
	struct hermonprm_event_db_register event;
	uint32_t dword[1];
} __attribute__ (( packed ));

union hermonprm_mad {
	struct hermonprm_mad_ifc ifc;
	union ib_mad mad;
} __attribute__ (( packed ));

union hermonprm_set_port {
	struct hermonprm_set_port_ib ib;
	struct hermonprm_set_port_general_context general;
	struct hermonprm_set_port_rqp_calc rqp_calc;
	struct hermonprm_set_port_mac_table mac_table[128];
	struct hermonprm_set_port_vlan vlan;
	struct hermonprm_set_port_beacon beacon;
} __attribute__ (( packed ));

struct hermon_wake_on_lan_conf {
	uint32_t			: 9;
	uint32_t	en_wol_phy	: 1;
	uint32_t	en_wol_uc	: 1;
	uint32_t	en_wol_mc	: 1;
	uint32_t	en_wol_bc	: 1;
	uint32_t	en_wol_arp	: 1;
	uint32_t	en_wol_magic	: 1;
	uint32_t	en_wol_passwd	: 1;
	uint32_t			: 16;
	uint32_t			: 32;
};

struct hermon_nv_config_flags {
	uint8_t nv_config_sriov_en      :1;
	uint8_t nv_config_wol_port1     :1;
	uint8_t nv_config_wol_port2     :1;
	uint8_t nv_config_vpi_port1     :1;
	uint8_t nv_config_vpi_port2     :1;
	uint8_t nv_config_bar_size      :1;
	uint8_t reserved                :2;
};

/** Hermon device capabilitiess */
struct hermon_dev_cap {
	/** CMPT entry size */
	size_t cmpt_entry_size;
	/** Number of reserved QPs */
	unsigned int reserved_qps;
	/** QP context entry size */
	size_t qpc_entry_size;
	/** Alternate path context entry size */
	size_t altc_entry_size;
	/** Auxiliary context entry size */
	size_t auxc_entry_size;
	/** Number of reserved SRQs */
	unsigned int reserved_srqs;
	/** SRQ context entry size */
	size_t srqc_entry_size;
	/** Number of reserved CQs */
	unsigned int reserved_cqs;
	/** CQ context entry size */
	size_t cqc_entry_size;
	/** Number of reserved EQs */
	unsigned int reserved_eqs;
	/** EQ context entry size */
	size_t eqc_entry_size;
	/** Number of reserved MTTs */
	unsigned int reserved_mtts;
	/** MTT entry size */
	size_t mtt_entry_size;
	/** Number of reserved MRWs */
	unsigned int reserved_mrws;
	/** DMPT entry size */
	size_t dmpt_entry_size;
	/** Number of reserved UARs */
	unsigned int reserved_uars;
	/** Supported Virtual Ethernet ports */
	u8 supported_veps[HERMON_MAX_PORTS];
	/** Bootable Virtual Ethernet ports */
	u8 num_veps[HERMON_MAX_PORTS];
	/** VEP index table */
	u8 vep_table[HERMON_MAX_PORTS][HERMON_MAX_VEPS];
	/** Dual-port different protocol */
	int dpdp;
	/** Unicast steering */
	unsigned int vep_uc_steering;
	/** Multicast steering */
	unsigned int vep_mc_steering;
	/** NvMem Access **/
	u8 nv_mem_access_supported;
	/**  ncsi_lag_mode **/
	u8 ncsi_lag_mode;
	/**  Support for post doorbell command **/
	u8 cmdif_post_doorbell;
	/** Number of ports **/
	u8 num_ports;
	/** Port beacon support **/
	u8 port_beacon;
	/** The maximum number of function indexes that can be supported by the device **/
	u8 max_funix;
	/** NV configuration flags **/
	struct hermon_nv_config_flags nv_config_flags;
};

/** Number of cMPT entries of each type */
#define HERMON_CMPT_MAX_ENTRIES ( 1 << 24 )

/** Hermon ICM memory map entry */
struct hermon_icm_map {
	/** Offset (virtual address within ICM) */
	uint64_t offset;
	/** Length */
	size_t len;
};

/** Discontiguous regions within Hermon ICM */
enum hermon_icm_map_regions {
	HERMON_ICM_QP_CMPT = 0,
	HERMON_ICM_SRQ_CMPT,
	HERMON_ICM_CQ_CMPT,
	HERMON_ICM_EQ_CMPT,
	HERMON_ICM_OTHER,
	HERMON_ICM_NUM_REGIONS
};

/** UAR page for doorbell accesses
 *
 * Pages 0-127 are reserved for event queue doorbells only, so we use
 * page 128.
 */
#define HERMON_UAR_NON_EQ_PAGE	128

/** Maximum number of allocatable MTT entries

 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_MTTS		64

/** A Hermon MTT descriptor */
struct hermon_mtt {
	/** MTT offset */
	unsigned int mtt_offset;
	/** Number of pages */
	unsigned int num_pages;
	/** MTT base address */
	unsigned int mtt_base_addr;
	/** Offset within page */
	unsigned int page_offset;
};

/** Alignment of Hermon send work queue entries */
#define HERMON_SEND_WQE_ALIGN 128

/** A Hermon send work queue entry */
union hermon_send_wqe {
	struct hermonprm_wqe_segment_ctrl_send ctrl;
	struct hermonprm_ud_send_wqe ud;
	struct hermonprm_mlx_send_wqe mlx;
	struct hermonprm_rc_send_wqe rc;
	struct hermonprm_eth_send_wqe eth;
	uint8_t force_align[HERMON_SEND_WQE_ALIGN];
} __attribute__ (( packed ));

/** A Hermon send work queue */
struct hermon_send_work_queue {
	/** Number of work queue entries, including headroom
	 *
	 * Hermon requires us to leave unused space within the send
	 * WQ, so we create a send WQ with more entries than are
	 * requested in the create_qp() call.
	 */
	unsigned int num_wqes;
	/** Work queue entries */
	union hermon_send_wqe *wqe;
	/** Size of work queue */
	size_t wqe_size;
	/** Doorbell register */
	void *doorbell;
};

/** Alignment of Hermon receive work queue entries */
#define HERMON_RECV_WQE_ALIGN 16

/** A Hermon receive work queue entry */
union hermon_recv_wqe {
	struct hermonprm_recv_wqe recv;
	uint8_t force_align[HERMON_RECV_WQE_ALIGN];
} __attribute__ (( packed ));

/** A Hermon receive work queue */
struct hermon_recv_work_queue {
	/** Work queue entries */
	union hermon_recv_wqe *wqe;
	/** Size of work queue */
	size_t wqe_size;
	/** Doorbell record */
	struct hermonprm_qp_db_record *doorbell;
};

/** Number of special queue pairs */
#define HERMON_NUM_SPECIAL_QPS 8

/** Number of queue pairs reserved for the "special QP" block
 *
 * The special QPs must be within a contiguous block aligned on its
 * own size.
 */
#define HERMON_RSVD_SPECIAL_QPS	( ( HERMON_NUM_SPECIAL_QPS << 1 ) - 1 )

/** Maximum number of allocatable queue pairs
 *
 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_QPS		8

/** Queue pair number randomisation mask */
#define HERMON_QPN_RANDOM_MASK 0xfff000

/** Hermon queue pair state */
enum hermon_queue_pair_state {
	HERMON_QP_ST_RST = 0,
	HERMON_QP_ST_INIT,
	HERMON_QP_ST_RTR,
	HERMON_QP_ST_RTS,
	HERMON_QP_ST_ERR = 6,
};

/** A Hermon queue pair */
struct hermon_queue_pair {
	/** Work queue buffer */
	void *wqe;
	/** Size of work queue buffer */
	size_t wqe_size;
	/** MTT descriptor */
	struct hermon_mtt mtt;
	/** Send work queue */
	struct hermon_send_work_queue send;
	/** Receive work queue */
	struct hermon_recv_work_queue recv;
	/** Queue state */
	enum hermon_queue_pair_state state;
};

/** Maximum number of allocatable completion queues
 *
 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_CQS		8

/** A Hermon completion queue */
struct hermon_completion_queue {
	/** Completion queue entries */
	union hermonprm_completion_entry *cqe;
	/** Size of completion queue */
	size_t cqe_size;
	/** MTT descriptor */
	struct hermon_mtt mtt;
	/** Doorbell record */
	struct hermonprm_cq_db_record *doorbell;
	int arm_sn;
	u32 *set_ci_db;
	u32 *arm_db;
};

enum {
	MLX4_CQ_DB_REQ_NOT_SOL		= 1 << 24,
	MLX4_CQ_DB_REQ_NOT		= 2 << 24,
};

#define MLX4_CQ_DOORBELL      0x20

/** Maximum number of allocatable event queues
 *
 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_EQS		8

/** A Hermon event queue */
struct hermon_event_queue {
	/** Event queue entries */
	union hermonprm_event_entry *eqe;
	/** Size of event queue */
	size_t eqe_size;
	/** MTT descriptor */
	struct hermon_mtt mtt;
	/** Event queue number */
	unsigned long eqn;
	/** Next event queue entry index */
	unsigned long next_idx;
	/** Doorbell register */
	void *doorbell;
};

/** Number of event queue entries
 *
 * This is a policy decision.
 */
#define HERMON_NUM_EQES		8

/** A Hermon resource bitmask */
typedef uint32_t hermon_bitmask_t;

/** Size of a hermon resource bitmask */
#define HERMON_BITMASK_SIZE(max_entries)				     \
	( ( (max_entries) + ( 8 * sizeof ( hermon_bitmask_t ) ) - 1 ) /	     \
	  ( 8 * sizeof ( hermon_bitmask_t ) ) )

struct hermon;
struct hermon_port;

/** A Hermon port type */
struct hermon_port_type {
	/** Register port
	 *
	 * @v hermon		Hermon device
	 * @v port		Hermon port
	 * @ret rc		Return status code
	 */
	int ( * register_dev ) ( struct hermon *hermon,
				 struct hermon_port *port );
	/** Port state changed
	 *
	 * @v hermon		Hermon device
	 * @v port		Hermon port
	 * @v link_up		Link is up
	 */
	void ( * state_change ) ( struct hermon *hermon,
				  struct hermon_port *port,
				  int link_up );
	/** Unregister port
	 *
	 * @v hermon		Hermon device
	 * @v port		Hermon port
	 */
	void ( * unregister_dev ) ( struct hermon *hermon,
				    struct hermon_port *port );
};


/** A Hermon port */
struct hermon_port {
	/** Infiniband device */
	struct ib_device *ibdev;
	/** Network device */
	struct net_device *netdev;
	/** Ethernet completion queue */
	struct ib_completion_queue *eth_cq;
	/** Ethernet queue pair */
	struct ib_queue_pair *eth_qp;
	/** VEP number */
	u8 vep_number;
	/** Ethernet MAC */
	unsigned long eth_mac_l;
	u16 eth_mac_h;
	/** Factory MAC (physical MAC) */
	unsigned long fact_mac_l;
	u16 fact_mac_h;
	/** Port default configurations */
	struct nv_port_conf_defaults defaults;
	/** Port type */
	struct hermon_port_type *type;
	/** Flash (NVmem) Configuration */
	struct nv_port_conf port_nv_conf;
	/** Driver settings */
	struct driver_settings driver_settings;
	/** port open boolean  */
	int hermon_is_port_open;
};

/** A Hermon device */
struct hermon {
	/** PCI device */
	struct pci_device *pci;
	/** PCI configuration registers */
	void *config;
	/** PCI user Access Region */
	void *uar;
	/** PCI clear interrupt register */
	void *clr_int;

	/** Command toggle */
	unsigned int toggle;
	/** Command input mailbox */
	void *mailbox_in;
	/** Command output mailbox */
	void *mailbox_out;

	/** Device open request counter */
	unsigned int open_count;

	/** Firmware size */
	size_t firmware_len;
	/** Firmware area in external memory
	 *
	 * This is allocated when first needed, and freed only on
	 * final teardown, in order to avoid memory map changes at
	 * runtime.
	 */
	userptr_t firmware_area;

	/** Interrupt support */
	u8  clr_int_bar;
	u32 clr_int_bar_offset_l;
	u32 clr_int_bar_offset_h;
	u64 clr_int_bar_offset;
	u8  intapin;

	/** ICM map */
	struct hermon_icm_map icm_map[HERMON_ICM_NUM_REGIONS];
	/** ICM size */
	size_t icm_len;
	/** ICM AUX size */
	size_t icm_aux_len;
	/** ICM area
	 *
	 * This is allocated when first needed, and freed only on
	 * final teardown, in order to avoid memory map changes at
	 * runtime.
	 */
	userptr_t icm;

	/** Event queue */
	struct hermon_event_queue eq;
	/** Unrestricted LKey
	 *
	 * Used to get unrestricted memory access.
	 */
	unsigned long lkey;

	/** Completion queue in-use bitmask */
	hermon_bitmask_t cq_inuse[ HERMON_BITMASK_SIZE ( HERMON_MAX_CQS ) ];
	/** Queue pair in-use bitmask */
	hermon_bitmask_t qp_inuse[ HERMON_BITMASK_SIZE ( HERMON_MAX_QPS ) ];
	/** MTT entry in-use bitmask */
	hermon_bitmask_t mtt_inuse[ HERMON_BITMASK_SIZE ( HERMON_MAX_MTTS ) ];

	/** Device capabilities */
	struct hermon_dev_cap cap;
	/** Special QPN base */
	unsigned long special_qpn_base;
	/** QPN base */
	unsigned long qpn_base;

	/** Ports */
	struct hermon_port port[HERMON_MAX_PORTS];

	/** BOFM device */
	struct bofm_device bofm;

	/** Port masking  */
	u16 port_mask;
	/** PCI physical function */
	u8 physical_function;
	/** Supported number of ports  */
	unsigned int num_ports;
	/** Bootable Virtual Ethernet Ports */
	u8 num_veps[HERMON_MAX_PORTS];
	/** VEP index table */
	u8 vep_table[HERMON_MAX_PORTS][HERMON_MAX_VEPS];
	/** MCG Hash table AUX index */
	unsigned int mcg_aux_index;
	/** List of multicast GIDs */
	struct list_head ncsi_mgids;
	/** Hermon default configurations */
	struct nv_conf_defaults defaults;
	/** Hermon ini configurations */
	struct nv_conf_ini ini_configurations;
	/* Hermon general settings */
	struct driver_settings hermon_settings;
	/* NV configurations */
	struct nv_conf hermon_nv_conf;
};

int hermon_open ( struct hermon *hermon );
void hermon_close ( struct hermon *hermon );
int hermon_flash_invalidate_tlv ( void *priv, uint32_t port_num,
				uint32_t tlv_type );
int hermon_flash_write_tlv ( void *hermon, void *src,
		uint32_t port_num, uint32_t tlv_type, uint32_t len );
int hermon_flash_read_tlv_wrapper ( void *drv_priv,
		struct driver_tlv_header *tlv_hdr );
int hermon_cmd_set_port ( struct hermon *hermon, uint16_t in_mod,
			const union hermonprm_set_port *set_port, unsigned int op_mod );
void hermon_get_ro_pci_settings ( void* driver_data );

/** Global protection domain */
#define HERMON_GLOBAL_PD		0x123456

/** Memory key prefix */
#define HERMON_MKEY_PREFIX		0x77000000UL

/*
 * HCA commands
 *
 */

#define HERMON_HCR_BASE			0x80680
#define HERMON_HCR_REG(x)		( HERMON_HCR_BASE + 4 * (x) )
#define HERMON_HCR_MAX_WAIT_MS		10000
#define HERMON_MBOX_ALIGN		(1 << 12)
#define HERMON_MBOX_SIZE		(1 << 12)

/* HCA command is split into
 *
 * bits  11:0	Opcode
 * bit     12	Input uses mailbox
 * bit     13	Output uses mailbox
 * bits 22:14	Input parameter length (in dwords)
 * bits 31:23	Output parameter length (in dwords)
 *
 * Encoding the information in this way allows us to cut out several
 * parameters to the hermon_command() call.
 */
#define HERMON_HCR_IN_MBOX		0x00001000UL
#define HERMON_HCR_OUT_MBOX		0x00002000UL
#define HERMON_HCR_OPCODE( _command )	( (_command) & 0xfff )
#define HERMON_HCR_IN_LEN( _command )	( ( (_command) >> 12 ) & 0x7fc )
#define HERMON_HCR_OUT_LEN( _command )	( ( (_command) >> 21 ) & 0x7fc )

/** Build HCR command from component parts */
#define HERMON_HCR_INOUT_CMD( _opcode, _in_mbox, _in_len,		     \
			     _out_mbox, _out_len )			     \
	( (_opcode) |							     \
	  ( (_in_mbox) ? HERMON_HCR_IN_MBOX : 0 ) |			     \
	  ( ( (_in_len) / 4 ) << 14 ) |					     \
	  ( (_out_mbox) ? HERMON_HCR_OUT_MBOX : 0 ) |			     \
	  ( ( (_out_len) / 4 ) << 23 ) )

#define HERMON_HCR_IN_CMD( _opcode, _in_mbox, _in_len )			     \
	HERMON_HCR_INOUT_CMD ( _opcode, _in_mbox, _in_len, 0, 0 )

#define HERMON_HCR_OUT_CMD( _opcode, _out_mbox, _out_len )		     \
	HERMON_HCR_INOUT_CMD ( _opcode, 0, 0, _out_mbox, _out_len )

#define HERMON_HCR_VOID_CMD( _opcode )					     \
	HERMON_HCR_INOUT_CMD ( _opcode, 0, 0, 0, 0 )

#endif /* _HERMON_H */
