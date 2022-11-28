/*
 * Cavium Fibre Channel HBA Driver
 * Copyright (c)  2003-2016 QLogic Corporation
 * Copyright (C)  2016-2017 Cavium Inc
 * Copyright (C)  2020-     Marvell Technology Group Ltd.
 *
 * See LICENSE.qla2xxx for copyright and licensing details.
 */
#ifndef __QLA_COMPAT_H
#define __QLA_COMPAT_H

#ifndef DEFINED_FPIN_RCV
#define fc_host_fpin_rcv(_a, _b, _c)
#endif /* DEFINED_FPIN_RCV */

#ifdef SCSI_CHANGE_QDEPTH
#define QLA_SCSI_QUEUE_DEPTH \
	.change_queue_depth	= scsi_change_queue_depth,
#else /* SCSI_CHANGE_QDEPTH */
#include <scsi/scsi_tcq.h>
#define QLA_SCSI_QUEUE_DEPTH \
	.change_queue_depth	= qla2x00_change_queue_depth,
static inline
void qla2x00_adjust_sdev_qdepth_up(struct scsi_device *sdev, int qdepth)
{
	fc_port_t *fcport = sdev->hostdata;
	struct scsi_qla_host *vha = fcport->vha;
	struct req_que *req = NULL;

	req = vha->req;
	if (!req)
		return;

	if (req->max_q_depth <= sdev->queue_depth || req->max_q_depth < qdepth)
		return;

	if (sdev->ordered_tags)
		scsi_adjust_queue_depth(sdev, MSG_ORDERED_TAG, qdepth);
	else
		scsi_adjust_queue_depth(sdev, MSG_SIMPLE_TAG, qdepth);

	ql_dbg(ql_dbg_io, vha, 0x302a,
	    "Queue depth adjusted-up to %d for nexus=%ld:%d:%d.\n",
	    sdev->queue_depth, fcport->vha->host_no, sdev->id, sdev->lun);
}

static inline
void qla2x00_handle_queue_full(struct scsi_device *sdev, int qdepth)
{
	fc_port_t *fcport = (struct fc_port *) sdev->hostdata;

	if (!scsi_track_queue_full(sdev, qdepth))
		return;

	ql_dbg(ql_dbg_io, fcport->vha, 0x3029,
	    "Queue depth adjusted-down to %d for nexus=%ld:%d:%d.\n",
	    sdev->queue_depth, fcport->vha->host_no, sdev->id, sdev->lun);
}

static inline
int qla2x00_change_queue_depth(struct scsi_device *sdev, int qdepth, int reason)
{
	switch (reason) {
	case SCSI_QDEPTH_DEFAULT:
		scsi_adjust_queue_depth(sdev, scsi_get_tag_type(sdev), qdepth);
		break;
	case SCSI_QDEPTH_QFULL:
		qla2x00_handle_queue_full(sdev, qdepth);
		break;
	case SCSI_QDEPTH_RAMP_UP:
		qla2x00_adjust_sdev_qdepth_up(sdev, qdepth);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return sdev->queue_depth;
}
#endif /* SCSI_CHANGE_QDEPTH */

#ifdef SCSI_MARGINAL_PATH_SUPPORT
#define QLA_SCSI_MARGINAL_PATH \
	.eh_should_retry_cmd	= fc_eh_should_retry_cmd,
#else
	#define QLA_SCSI_MARGINAL_PATH
#endif /* SCSI_MARGINAL_PATH_SUPPORT */

#ifdef SCSI_CHANGE_QTYPE
#define QLA_SCSI_QUEUE_TYPE \
	.change_queue_type	= qla2x00_change_queue_type,
static inline int
qla2x00_change_queue_type(struct scsi_device *sdev, int tag_type)
{
	if (sdev->tagged_supported) {
		scsi_set_tag_type(sdev, tag_type);
		if (tag_type)
			scsi_activate_tcq(sdev, sdev->queue_depth);
		else
			scsi_deactivate_tcq(sdev, sdev->queue_depth);
	} else
		tag_type = 0;

	return tag_type;
}

#else /* SCSI_CHANGE_QTYPE */
#define QLA_SCSI_QUEUE_TYPE
#endif /* SCSI_CHANGE_QTYPE */

#ifdef SCSI_MAP_QUEUES
#define QLA_SCSI_MAP_QUEUES \
	.map_queues             = qla2xxx_map_queues,
#include <linux/blk-mq-pci.h>
static inline int qla2xxx_map_queues(struct Scsi_Host *shost)
{
	int rc = -EINVAL;
	scsi_qla_host_t *vha = (scsi_qla_host_t *)shost->hostdata;
#ifdef BLK_MQ_HCTX_TYPE
	struct blk_mq_queue_map *qmap = &shost->tag_set.map[HCTX_TYPE_DEFAULT];

	if (USER_CTRL_IRQ(vha->hw) || !vha->hw->mqiobase)
		rc = blk_mq_map_queues(qmap);
	else
		rc = blk_mq_pci_map_queues(qmap,
				vha->hw->pdev, vha->irq_offset);
#else

	if (USER_CTRL_IRQ(vha->hw) || !vha->hw->mqiobase)
		rc = blk_mq_map_queues(&shost->tag_set);
	else
#ifdef BLK_PCI_MAPQ_3_ARGS
		rc = blk_mq_pci_map_queues(
				(struct blk_mq_tag_set *)&shost->tag_set,
				vha->hw->pdev, vha->irq_offset);
#else /* BLK_PCI_MAPQ_3_ARGS */
		rc = blk_mq_pci_map_queues(
				(struct blk_mq_tag_set *)&shost->tag_set,
				vha->hw->pdev);
#endif /* BLK_PCI_MAPQ_3_ARGS */
#endif
	return rc;
}
#else /* SCSI_MAP_QUEUES */
#define QLA_SCSI_MAP_QUEUES
#endif /* SCSI_MAP_QUEUES */


#ifdef SCSI_HOST_WIDE_TAGS
#define QLA_SCSI_HOST_WIDE_TAGS \
	.use_host_wide_tags = 1,
#else /* SCSI_HOST_WIDE_TAGS */
#define QLA_SCSI_HOST_WIDE_TAGS
#endif /* SCSI_HOST_WIDE_TAGS */

#define lun_cast(_a) (long long)(_a)

#ifdef SCSI_HAS_TCQ
static inline
void qla_scsi_tcq_handler(struct scsi_device *sdev)
{
	scsi_qla_host_t *vha = shost_priv(sdev->host);
	struct req_que *req = vha->req;

	if (sdev->tagged_supported)
		scsi_activate_tcq(sdev, req->max_q_depth);
	else
		scsi_deactivate_tcq(sdev, req->max_q_depth);
}
#else /* SCSI_HAS_TCQ */
#define qla_scsi_tcq_handler(_sdev)
#endif /* SCSI_HAS_TCQ */

#ifdef SCSI_CMD_TAG_ATTR
#include <scsi/scsi_tcq.h>
static inline
int qla_scsi_get_task_attr(struct scsi_cmnd *cmd)
{
	char tag[2];
	if (scsi_populate_tag_msg(cmd, tag)) {
		switch (tag[0]) {
		case HEAD_OF_QUEUE_TAG:
		    return TSK_HEAD_OF_QUEUE;
		case ORDERED_QUEUE_TAG:
		    return TSK_ORDERED;
		default:
		    return TSK_SIMPLE;
		}
	}
	return TSK_SIMPLE;
}
#else /* SCSI_CMD_TAG_ATTR */
#define qla_scsi_get_task_attr(_cmd) (TSK_SIMPLE)
#endif /* SCSI_CMD_TAG_ATTR */

#ifdef SCSI_FC_BSG_JOB
#define fc_bsg_to_shost(_job) (_job)->shost
#define fc_bsg_to_rport(_job) (_job)->rport
#define bsg_job_done(_job, _res, _len) (_job)->job_done(_job)
#define qla_fwsts_ptr(_job) ((uint8_t *)(_job)->req->sense) + \
				sizeof(struct fc_bsg_reply)
#else /* SCSI_FC_BSG_JOB */
#define qla_fwsts_ptr(_job) ((_job)->reply + sizeof(struct fc_bsg_reply))
#endif /* SCSI_FC_BSG_JOB */

#ifdef TIMER_SETUP
#define qla_timer_setup(_tmr, _func, _flags, _cb) \
	timer_setup(_tmr, _func, _flags)
#define qla_from_timer(_var, _timer_arg, _field) \
	(typeof(*_var) *)from_timer(_var, _timer_arg, _field)
#else /* TIMER_SETUP */
#define qla_timer_setup(_tmr, _func, _flags, _cb) \
	init_timer(_tmr); \
	(_tmr)->data = (qla_timer_arg_t) (_cb); \
	(_tmr)->function = (void (*)(unsigned long))_func;
#define qla_from_timer(_var, _timer_arg, _field) \
	(typeof(*_var) *)(_timer_arg)
#endif /* TIMER_SETUP */

#ifdef DMA_ZALLOC_COHERENT
#else /* DMA_ZALLOC_COHERENT */
/* This version of dma_alloc_coherent() does zero out memory. */
#define dma_zalloc_coherent(_dev, _sz, _hdl, _flag) \
	dma_alloc_coherent(_dev, _sz, _hdl, _flag)
#endif /* DMA_ZALLOC_COHERENT */

#ifdef SCSI_USE_CLUSTERING
#define QLA_SCSI_USER_CLUSETERING\
	.use_clustering = ENABLE_CLUSTERING,
#else /* SCSI_USE_CLUSTERING */
#define QLA_SCSI_USER_CLUSETERING
#endif /* SCSI_USE_CLUSTERING */

#ifdef KTIME_GET_REAL_SECONDS
#define qla_get_real_seconds() ktime_get_real_seconds()
#else /* KTIME_GET_REAL_SECONDS */
static inline
u64 qla_get_real_seconds(void)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	return tv.tv_sec;
}
#endif /* KTIME_GET_REAL_SECONDS */

#ifndef  FC_PORTSPEED_64GBIT
#define FC_PORTSPEED_64GBIT             0x1000
#endif
#ifndef  FC_PORTSPEED_128GBIT
#define FC_PORTSPEED_128GBIT            0x2000
#endif

#ifdef BE_ARRAY
static inline void cpu_to_be32_array(__be32 *dst, const u32 *src, size_t len)
{
	int i;

	for (i = 0; i < len; i++)
		dst[i] = cpu_to_be32(src[i]);
}

static inline void be32_to_cpu_array(u32 *dst, const __be32 *src, size_t len)
{
	int i;

	for (i = 0; i < len; i++)
		dst[i] = be32_to_cpu(src[i]);
}
#endif

#ifdef SCSI_USE_BLK_MQ
# ifdef RHEL_DISTRO_VERSION
# define rhel_set_blk_mq(_host) \
	(_host)->use_blk_mq = ql2xmqsupport ? true : false;
# else /* RHEL_DISTRO_VERSION */
# define rhel_set_blk_mq(_host)
# endif /* RHEL_DISTRO_VERSION */
#else /* SCSI_USE_BLK_MQ */
/* Legacy was killed off, return 1 always. */
#define shost_use_blk_mq(_host) 1
#define rhel_set_blk_mq(_host)
#endif /* SCSI_USE_BLK_MQ */

#ifdef NVME_POLL_QUEUE
static inline
void qla_nvme_poll(struct nvme_fc_local_port *lport, void *hw_queue_handle)
{
	struct qla_qpair *qpair = hw_queue_handle;
	unsigned long flags;
	struct scsi_qla_host *vha = lport->private;

	spin_lock_irqsave(&qpair->qp_lock, flags);
	queue_work(vha->hw->wq, &qpair->q_work);
	spin_unlock_irqrestore(&qpair->qp_lock, flags);
}
#define QLA_NVME_POLL_QUEUE \
	.poll_queue	= qla_nvme_poll,
#else /* NVME_POLL_QUEUE */
#define QLA_NVME_POLL_QUEUE
#endif /* NVME_POLL_QUEUE */

#define qla_scsi_templ_compat_entries \
	QLA_SCSI_QUEUE_DEPTH \
	QLA_SCSI_QUEUE_TYPE \
	QLA_SCSI_HOST_WIDE_TAGS \
	QLA_SCSI_USER_CLUSETERING \
	QLA_SCSI_MAP_QUEUES \
	QLA_SCSI_FC_EH_TIMED_OUT \
	QLA_SCSI_TRACK_QUE_DEPTH \
	QLA_SCSI_MARGINAL_PATH

#define qla_nvme_templ_compat_entries \
	QLA_NVME_POLL_QUEUE


#define qla_pci_err_handler_compat_entries \
	QLA_PCI_ERR_RESET_PREPARE \
	QLA_PCI_ERR_RESET_DONE

#ifdef SCSI_CMD_PRIV
typedef scsi_cmd_priv ql_scsi_cmd_priv;
#else /* SCSI_CMD_PRIV */
static inline void *ql_scsi_cmd_priv(struct scsi_cmnd *cmd)
{
	return cmd + 1;
}
#endif /*SCSI_CMD_PRIV */

#ifdef FC_EH_TIMED_OUT
#define QLA_SCSI_FC_EH_TIMED_OUT \
	.eh_timed_out = fc_eh_timed_out,
#else /* FC_EH_TIMED_OUT */
#define QLA_SCSI_FC_EH_TIMED_OUT
#endif /* FC_EH_TIMED_OUT */

#ifdef SCSI_TRACK_QUE_DEPTH
#define QLA_SCSI_TRACK_QUE_DEPTH \
	.track_queue_depth = 1,
#else /* SCSI_TRACK_QUE_DEPTH */
#define QLA_SCSI_TRACK_QUE_DEPTH
#endif /* SCSI_TRACK_QUE_DEPTH */

#ifdef PCI_ERR_RESET_PREPARE
#define QLA_PCI_ERR_RESET_PREPARE \
	.reset_prepare = qla_pci_reset_prepare,

static inline void
qla_pci_reset_prepare(struct pci_dev *pdev)
{
	scsi_qla_host_t *base_vha = pci_get_drvdata(pdev);
	struct qla_hw_data *ha = base_vha->hw;
	struct qla_qpair *qpair;

	ql_log(ql_log_warn, base_vha, 0xffff,
	    "%s.\n", __func__);

	/*
	 * PCI FLR/function reset is about to reset the
	 * slot. Stop the chip to stop all DMA access.
	 * It is assumed that pci_reset_done will be called
	 * after FLR to resume Chip operation.
	 */
	ha->flags.eeh_busy = 1;
	mutex_lock(&ha->mq_lock);
	list_for_each_entry(qpair, &base_vha->qp_list, qp_list_elem)
		qpair->online = 0;
	mutex_unlock(&ha->mq_lock);

	set_bit(ABORT_ISP_ACTIVE, &base_vha->dpc_flags);
	qla2x00_abort_isp_cleanup(base_vha);
	qla2x00_abort_all_cmds(base_vha, DID_RESET << 16);
}
#else /* PCI_ERR_RESET_PREPARE */
#define QLA_PCI_ERR_RESET_PREPARE
#endif /* PCI_ERR_RESET_PREPARE */

#ifdef PCI_ERR_RESET_DONE
#define QLA_PCI_ERR_RESET_DONE\
	.reset_done = qla_pci_reset_done,
static inline void
qla_pci_reset_done(struct pci_dev *pdev)
{
	scsi_qla_host_t *base_vha = pci_get_drvdata(pdev);
	struct qla_hw_data *ha = base_vha->hw;
	struct qla_qpair *qpair;

	ql_log(ql_log_warn, base_vha, 0xffff,
	    "%s.\n", __func__);

	/*
	 * FLR just completed by PCI layer. Resume adapter
	 */
	ha->flags.eeh_busy = 0;
	mutex_lock(&ha->mq_lock);
	list_for_each_entry(qpair, &base_vha->qp_list, qp_list_elem)
		qpair->online = 1;
	mutex_unlock(&ha->mq_lock);

	base_vha->flags.online = 1;
	ha->isp_ops->abort_isp(base_vha);
	clear_bit(ABORT_ISP_ACTIVE, &base_vha->dpc_flags);
}
#else /* PCI_ERR_RESET_DONE */
#define QLA_PCI_ERR_RESET_DONE
#endif /* PCI_ERR_RESET_DONE */

#ifndef MIN_NICE
#define MIN_NICE -20
#endif

#ifdef T10_PI_APP_ESC
#include <linux/t10-pi.h>
#define QL_T10_PI_APP_ESCAPE T10_PI_APP_ESCAPE
#else /* T10_PI_APP_ESC */
#define QL_T10_PI_APP_ESCAPE 0xffff
#endif /* T10_PI_APP_ESC */

#ifdef T10_PI_REF_ESC
#include <linux/t10-pi.h>
#define QL_T10_PI_REF_ESCAPE T10_PI_REF_ESCAPE
#else /* T10_PI_REF_ESC */
#define QL_T10_PI_REF_ESCAPE 0xffffffff
#endif /* T10_PI_REF_ESC */

#ifdef T10_PI_TUPLE
#include <linux/t10-pi.h>
typedef struct t10_pi_tuple QL_T10_PI_TUPLE;
#else /* T10_PI_TUPLE */
/*
 * (sd.h is not exported, hence local inclusion)
 * Data Integrity Field tuple.
 */
struct sd_dif_tuple {
	__be16 guard_tag;	/* Checksum */
	__be16 app_tag;		/* Opaque storage */
	__be32 ref_tag;		/* Target LBA or indirect LBA */
};

typedef struct sd_dif_tuple QL_T10_PI_TUPLE;
#endif /* T10_PI_TUPLE */

#ifdef TGT_FREE_TAG
#define QL_TGT_FREE_TAG(cmd)	(target_free_tag(cmd->sess->se_sess, &cmd->se_cmd))
#else /* TGT_FREE_TAG */
#define QL_TGT_FREE_TAG(cmd)	(tcm_qla2xxx_rel_cmd(cmd))
#endif /* TGT_FREE_TAG */

#ifdef TGT_MAKE_TPG_PARAM_CFG_GROUP
#define TCM_MAKE_TPG_ARGS(_a1, _a2, _a3) _a1, _a2, _a3
#else /* TGT_MAKE_TPG_PARAM_CFG_GROUP */
#define TCM_MAKE_TPG_ARGS(_a1, _a2, _a3) _a1, _a3
#endif /* TGT_MAKE_TPG_PARAM_CFG_GROUP */

#ifdef	TGT_SET_RM_SESSION
#define TARGET_REMOVE_SESSION  target_remove_session(se_sess)
#else /* TGT_SET_RM_SESSION */
#define TARGET_REMOVE_SESSION
#endif /* TGT_SET_RM_SESSION */

#ifdef TGT_FABRIC_OPS_FABRIC_NAME
#define TCM_FABRIC_NAME		.fabric_name = "qla2xxx",
#define TCM_FABRIC_NAME_NPIV	.fabric_name = "qla2xxx_npiv",
#else /* TGT_FABRIC_OPS_FABRIC_NAME */
#define TCM_FABRIC_NAME		.name = "qla2xxx",
#define TCM_FABRIC_NAME_NPIV	.name = "qla2xxx_npiv",
#endif /* TGT_FABRIC_OPS_FABRIC_NAME */

#ifdef FPIN_EVENT_TYPES
#define DECLARE_ENUM2STR_LOOKUP_DELI_EVENT DECLARE_ENUM2STR_LOOKUP( \
		qla_get_dn_event_type, fc_fpin_deli_event_types, \
		QL_FPIN_DELI_EVT_TYPES_INIT);
#define DECLARE_ENUM2STR_LOOKUP_CONGN_EVENT DECLARE_ENUM2STR_LOOKUP( \
		qla_get_congn_event_type, fc_fpin_congn_event_types, \
		FC_FPIN_CONGN_EVT_TYPES_INIT);
#else
#define DECLARE_ENUM2STR_LOOKUP_DELI_EVENT DECLARE_ENUM2STR_LOOKUP( \
		qla_get_dn_event_type, ql_fpin_deli_event_types, \
		QL_FPIN_DELI_EVT_TYPES_INIT);
#define DECLARE_ENUM2STR_LOOKUP_CONGN_EVENT DECLARE_ENUM2STR_LOOKUP( \
		qla_get_congn_event_type, ql_fpin_congn_event_types, \
		QL_FPIN_CONGN_EVT_TYPES_INIT);
/*
 * Delivery event types
 */
enum ql_fpin_deli_event_types {
	FPIN_DELI_UNKNOWN =		0x0,
	FPIN_DELI_TIMEOUT =		0x1,
	FPIN_DELI_UNABLE_TO_ROUTE =	0x2,
	FPIN_DELI_DEVICE_SPEC =		0xF,
};

/*
 * Congestion event types
 */
enum ql_fpin_congn_event_types {
	FPIN_CONGN_CLEAR =		0x0,
	FPIN_CONGN_LOST_CREDIT =	0x1,
	FPIN_CONGN_CREDIT_STALL =	0x2,
	FPIN_CONGN_OVERSUBSCRIPTION =	0x3,
	FPIN_CONGN_DEVICE_SPEC =	0xF,
};

/*
 * Initializer useful for decoding table.
 * Please keep this in sync with the above definitions.
 */
#define QL_FPIN_CONGN_EVT_TYPES_INIT {				\
	{ FPIN_CONGN_CLEAR,		"Clear" },		\
	{ FPIN_CONGN_LOST_CREDIT,	"Lost Credit" },	\
	{ FPIN_CONGN_CREDIT_STALL,	"Credit Stall" },	\
	{ FPIN_CONGN_OVERSUBSCRIPTION,	"Oversubscription" },	\
	{ FPIN_CONGN_DEVICE_SPEC,	"Device Specific" },	\
}

#endif

#define tcm_qla2xxx_ops_compat_entries \
	TCM_FABRIC_NAME

#define tcm_qla2xxx_npiv_ops_compat_entries \
	TCM_FABRIC_NAME_NPIV

#ifdef NVME_FC_PORT_TEMPLATE_HV_MODULE
#define NVME_FC_PORT_TEMPLATE_MODULE .module = THIS_MODULE,
#else
#define NVME_FC_PORT_TEMPLATE_MODULE
#endif

#ifndef fallthrough
#  if defined(__GNUC__) && __GNUC__ >= 7
#    define fallthrough   __attribute__((__fallthrough__))
#  else
#    define fallthrough   do {} while (0)  /* fallthrough */
#  endif
#endif



/* rhel 9.0 support */

#ifndef ioremap_nocache
#define ioremap_nocache ioremap
#endif


#ifndef SET_DRIVER_BYTE
#define DRIVER_SENSE	0x08
static inline void set_driver_byte(struct scsi_cmnd *cmd, char status)
{
	cmd->result = (cmd->result & 0x00ffffff) | (status << 24);
}
#endif

#ifndef LIST_IS_FIRST
/**
 * list_is_first -- tests whether @ list is the first entry in list @head
 * @list: the entry to test
 * @head: the head of the list
 */
static inline int list_is_first(const struct list_head *list,
	const struct list_head *head)
{
	return list->prev == head;
}
#endif

#endif /* __QLA_COMPAT_H */
