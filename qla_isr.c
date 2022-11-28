/*
 * QLogic Fibre Channel HBA Driver
 * Copyright (c)  2003-2014 QLogic Corporation
 *
 * See LICENSE.qla2xxx for copyright and licensing details.
 */
#include "qla_def.h"
#include "qla_target.h"
#include "qla_gbl.h"

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <scsi/scsi_tcq.h>
#include <scsi/scsi_bsg_fc.h>
#include <scsi/scsi_eh.h>
#include <scsi/fc/fc_fs.h>
#include <linux/nvme-fc-driver.h>

static void qla2x00_mbx_completion(scsi_qla_host_t *, uint16_t);
static void qla2x00_status_entry(scsi_qla_host_t *, struct rsp_que *, void *);
static void qla2x00_status_cont_entry(struct rsp_que *, sts_cont_entry_t *);
static void qla27xx_status_cont_type_1(scsi_qla_host_t *, sts_cont_entry_t *);
static int qla2x00_error_entry(scsi_qla_host_t *, struct rsp_que *,
	sts_entry_t *);
static struct purex_item *qla24xx_alloc_purex_item(scsi_qla_host_t *vha,
	uint16_t size);
static struct purex_item *qla24xx_copy_std_pkt(struct scsi_qla_host *vha,
	void *pkt);
static struct purex_item *qla27xx_copy_fpin_pkt(struct scsi_qla_host *vha,
	void **pkt, struct rsp_que **rsp);
static void qla24xx_process_purex_rdp(struct scsi_qla_host *vha,
	struct purex_item *pkt);

const char *const port_state_str[] = {
	"Unknown",
	"UNCONFIGURED",
	"DEAD",
	"LOST",
	"ONLINE"
};


/**
 * __qla_consume_iocb - this routine is used to tell fw driver has processed
 *   or consumed the head IOCB along with the continuation IOCB's from the
 *   provided respond queue.
 * @vha: host adapter pointer
 * @pkt: pointer to current packet.  On return, this pointer shall move
 *       to the next packet.
 * @rsp: respond queue pointer.
 *
 * it is assumed pkt is the head iocb, not the continuation iocbk
 */
void __qla_consume_iocb(struct scsi_qla_host *vha,
    void **pkt, struct rsp_que **rsp)
{
	struct rsp_que *rsp_q = *rsp;
	response_t *new_pkt;
	uint16_t entry_count_remaining;
	struct purex_entry_24xx *purex = *pkt;

	entry_count_remaining = purex->entry_count;
	while (entry_count_remaining > 0) {
		new_pkt = rsp_q->ring_ptr;
		*pkt = new_pkt;

		rsp_q->ring_index++;
		if (rsp_q->ring_index == rsp_q->length) {
			rsp_q->ring_index = 0;
			rsp_q->ring_ptr = rsp_q->ring;
		} else {
			rsp_q->ring_ptr++;
		}

		new_pkt->signature = RESPONSE_PROCESSED;
		/* flush signature */
		wmb();
		--entry_count_remaining;
	}
}

/**
 * __qla_copy_purex_to_buffer - extract ELS payload from Purex IOCB
 *    and save to provided buffer
 * @vha: host adapter pointer
 * @pkt: pointer Purex IOCB
 * @rsp: respond queue
 * @buf: extracted ELS payload copy here
 * @buf_len: buffer length
 */
int __qla_copy_purex_to_buffer(struct scsi_qla_host *vha,
	void **pkt, struct rsp_que **rsp, u8 *buf, u32 buf_len)
{
	struct purex_entry_24xx *purex = *pkt;
	struct rsp_que *rsp_q = *rsp;
	sts_cont_entry_t *new_pkt;
	uint16_t no_bytes = 0, total_bytes = 0, pending_bytes = 0;
	uint16_t buffer_copy_offset = 0;
	uint16_t entry_count_remaining;
	u16 tpad;

	entry_count_remaining = purex->entry_count;
	total_bytes = (le16_to_cpu(purex->frame_size) & 0x0FFF)
		- PURX_ELS_HEADER_SIZE;

	/*
	 * end of payload may not end in 4bytes boundary.  Need to
	 * round up / pad for room to swap, before saving data
	 */
	tpad = roundup(total_bytes, 4);

	if (buf_len < tpad) {
		ql_dbg(ql_dbg_async, vha, 0x5084,
		    "%s buffer is too small %d < %d\n",
		    __func__, buf_len, tpad);
		__qla_consume_iocb(vha, pkt, rsp);
		return -EIO;
	}

	pending_bytes = total_bytes = tpad;
	no_bytes = (pending_bytes > sizeof(purex->els_frame_payload))  ?
	    sizeof(purex->els_frame_payload) : pending_bytes;

	memcpy(buf, &purex->els_frame_payload[0], no_bytes);
	buffer_copy_offset += no_bytes;
	pending_bytes -= no_bytes;
	--entry_count_remaining;

	((response_t *)purex)->signature = RESPONSE_PROCESSED;
	/* flush signature */
	wmb();

	do {
		while ((total_bytes > 0) && (entry_count_remaining > 0)) {
			new_pkt = (sts_cont_entry_t *)rsp_q->ring_ptr;
			*pkt = new_pkt;

			if (new_pkt->entry_type != STATUS_CONT_TYPE) {
				ql_log(ql_log_warn, vha, 0x507a,
				    "Unexpected IOCB type, partial data 0x%x\n",
				    buffer_copy_offset);
				break;
			}

			rsp_q->ring_index++;
			if (rsp_q->ring_index == rsp_q->length) {
				rsp_q->ring_index = 0;
				rsp_q->ring_ptr = rsp_q->ring;
			} else {
				rsp_q->ring_ptr++;
			}
			no_bytes = (pending_bytes > sizeof(new_pkt->data)) ?
			    sizeof(new_pkt->data) : pending_bytes;
			if ((buffer_copy_offset + no_bytes) <= total_bytes) {
				memcpy((buf + buffer_copy_offset), new_pkt->data,
				    no_bytes);
				buffer_copy_offset += no_bytes;
				pending_bytes -= no_bytes;
				--entry_count_remaining;
			} else {
				ql_log(ql_log_warn, vha, 0x5044,
				    "Attempt to copy more that we got, optimizing..%x\n",
				    buffer_copy_offset);
				memcpy((buf + buffer_copy_offset), new_pkt->data,
				    total_bytes - buffer_copy_offset);
			}

			((response_t *)new_pkt)->signature = RESPONSE_PROCESSED;
			/* flush signature */
			wmb();
		}

		if (pending_bytes != 0 || entry_count_remaining != 0) {
			ql_log(ql_log_fatal, vha, 0x508b,
			    "Dropping partial Data, underrun bytes = 0x%x, entry cnts 0x%x\n",
			    total_bytes, entry_count_remaining);
			return -EIO;
		}
	} while (entry_count_remaining > 0);

	be32_to_cpu_array((u32 *)buf, (__be32 *)buf, total_bytes >> 2);

	return 0;
}


/**
 * qla2100_intr_handler() - Process interrupts for the ISP2100 and ISP2200.
 * @irq: interrupt number
 * @dev_id: SCSI driver HA context
 *
 * Called by system whenever the host adapter generates an interrupt.
 *
 * Returns handled flag.
 */
irqreturn_t
qla2100_intr_handler(int irq, void *dev_id)
{
	scsi_qla_host_t	*vha;
	struct qla_hw_data *ha;
	struct device_reg_2xxx __iomem *reg;
	int		status;
	unsigned long	iter;
	uint16_t	hccr;
	uint16_t	mb[8];
	struct rsp_que *rsp;
	unsigned long	flags;

	rsp = (struct rsp_que *) dev_id;
	if (!rsp) {
		ql_log(ql_log_info, NULL, 0x505d,
		    "%s: NULL response queue pointer.\n", __func__);
		return (IRQ_NONE);
	}

	ha = rsp->hw;
	reg = &ha->iobase->isp;
	status = 0;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	vha = pci_get_drvdata(ha->pdev);
	for (iter = 50; iter--; ) {
		hccr = RD_REG_WORD(&reg->hccr);
		if (qla2x00_check_reg16_for_disconnect(vha, hccr))
			break;
		if (hccr & HCCR_RISC_PAUSE) {
			if (pci_channel_offline(ha->pdev))
				break;

			/*
			 * Issue a "HARD" reset in order for the RISC interrupt
			 * bit to be cleared.  Schedule a big hammer to get
			 * out of the RISC PAUSED state.
			 */
			WRT_REG_WORD(&reg->hccr, HCCR_RESET_RISC);
			RD_REG_WORD(&reg->hccr);
			vha->hw_err_cnt++;

			ha->isp_ops->fw_dump(vha, 1);
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
			break;
		} else if ((RD_REG_WORD(&reg->istatus) & ISR_RISC_INT) == 0)
			break;

		if (RD_REG_WORD(&reg->semaphore) & BIT_0) {
			WRT_REG_WORD(&reg->hccr, HCCR_CLR_RISC_INT);
			RD_REG_WORD(&reg->hccr);

			/* Get mailbox data. */
			mb[0] = RD_MAILBOX_REG(ha, reg, 0);
			if (mb[0] > 0x3fff && mb[0] < 0x8000) {
				qla2x00_mbx_completion(vha, mb[0]);
				status |= MBX_INTERRUPT;
			} else if (mb[0] > 0x7fff && mb[0] < 0xc000) {
				mb[1] = RD_MAILBOX_REG(ha, reg, 1);
				mb[2] = RD_MAILBOX_REG(ha, reg, 2);
				mb[3] = RD_MAILBOX_REG(ha, reg, 3);
				qla2x00_async_event(vha, rsp, mb);
			} else {
				/*EMPTY*/
				ql_dbg(ql_dbg_async, vha, 0x5025,
				    "Unrecognized interrupt type (%d).\n",
				    mb[0]);
			}
			/* Release mailbox registers. */
			WRT_REG_WORD(&reg->semaphore, 0);
			RD_REG_WORD(&reg->semaphore);
		} else {
			qla2x00_process_response_queue(rsp);

			WRT_REG_WORD(&reg->hccr, HCCR_CLR_RISC_INT);
			RD_REG_WORD(&reg->hccr);
		}
	}
	qla2x00_handle_mbx_completion(ha, status);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	return (IRQ_HANDLED);
}

bool
qla2x00_check_reg32_for_disconnect(scsi_qla_host_t *vha, uint32_t reg)
{
	/* Check for PCI disconnection */
	if (reg == 0xffffffff && !pci_channel_offline(vha->hw->pdev)) {
		if (!test_and_set_bit(PFLG_DISCONNECTED, &vha->pci_flags) &&
		    !test_bit(PFLG_DRIVER_REMOVING, &vha->pci_flags) &&
		    !test_bit(PFLG_DRIVER_PROBING, &vha->pci_flags)) {
			qla_schedule_eeh_work(vha);
		}
		return true;
	} else
		return false;
}

bool
qla2x00_check_reg16_for_disconnect(scsi_qla_host_t *vha, uint16_t reg)
{
	return qla2x00_check_reg32_for_disconnect(vha, 0xffff0000 | reg);
}

/**
 * qla2300_intr_handler() - Process interrupts for the ISP23xx and ISP63xx.
 * @irq: interrupt number
 * @dev_id: SCSI driver HA context
 *
 * Called by system whenever the host adapter generates an interrupt.
 *
 * Returns handled flag.
 */
irqreturn_t
qla2300_intr_handler(int irq, void *dev_id)
{
	scsi_qla_host_t	*vha;
	struct device_reg_2xxx __iomem *reg;
	int		status;
	unsigned long	iter;
	uint32_t	stat;
	uint16_t	hccr;
	uint16_t	mb[8];
	struct rsp_que *rsp;
	struct qla_hw_data *ha;
	unsigned long	flags;

	rsp = (struct rsp_que *) dev_id;
	if (!rsp) {
		ql_log(ql_log_info, NULL, 0x5058,
		    "%s: NULL response queue pointer.\n", __func__);
		return (IRQ_NONE);
	}

	ha = rsp->hw;
	reg = &ha->iobase->isp;
	status = 0;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	vha = pci_get_drvdata(ha->pdev);
	for (iter = 50; iter--; ) {
		stat = RD_REG_DWORD(&reg->u.isp2300.host_status);
		if (qla2x00_check_reg32_for_disconnect(vha, stat))
			break;
		if (stat & HSR_RISC_PAUSED) {
			if (unlikely(pci_channel_offline(ha->pdev)))
				break;

			hccr = RD_REG_WORD(&reg->hccr);
			vha->hw_err_cnt++;

			if (hccr & (BIT_15 | BIT_13 | BIT_11 | BIT_8))
				ql_log(ql_log_warn, vha, 0x5026,
				    "Parity error -- HCCR=%x, Dumping "
				    "firmware.\n", hccr);
			else
				ql_log(ql_log_warn, vha, 0x5027,
				    "RISC paused -- HCCR=%x, Dumping "
				    "firmware.\n", hccr);

			/*
			 * Issue a "HARD" reset in order for the RISC
			 * interrupt bit to be cleared.  Schedule a big
			 * hammer to get out of the RISC PAUSED state.
			 */
			WRT_REG_WORD(&reg->hccr, HCCR_RESET_RISC);
			RD_REG_WORD(&reg->hccr);

			ha->isp_ops->fw_dump(vha, 1);
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
			break;
		} else if ((stat & HSR_RISC_INT) == 0)
			break;

		switch (stat & 0xff) {
		case 0x1:
		case 0x2:
		case 0x10:
		case 0x11:
			qla2x00_mbx_completion(vha, MSW(stat));
			status |= MBX_INTERRUPT;

			/* Release mailbox registers. */
			WRT_REG_WORD(&reg->semaphore, 0);
			break;
		case 0x12:
			mb[0] = MSW(stat);
			mb[1] = RD_MAILBOX_REG(ha, reg, 1);
			mb[2] = RD_MAILBOX_REG(ha, reg, 2);
			mb[3] = RD_MAILBOX_REG(ha, reg, 3);
			qla2x00_async_event(vha, rsp, mb);
			break;
		case 0x13:
			qla2x00_process_response_queue(rsp);
			break;
		case 0x15:
			mb[0] = MBA_CMPLT_1_16BIT;
			mb[1] = MSW(stat);
			qla2x00_async_event(vha, rsp, mb);
			break;
		case 0x16:
			mb[0] = MBA_SCSI_COMPLETION;
			mb[1] = MSW(stat);
			mb[2] = RD_MAILBOX_REG(ha, reg, 2);
			qla2x00_async_event(vha, rsp, mb);
			break;
		default:
			ql_dbg(ql_dbg_async, vha, 0x5028,
			    "Unrecognized interrupt type (%d).\n", stat & 0xff);
			break;
		}
		WRT_REG_WORD(&reg->hccr, HCCR_CLR_RISC_INT);
		RD_REG_WORD_RELAXED(&reg->hccr);
	}
	qla2x00_handle_mbx_completion(ha, status);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	return (IRQ_HANDLED);
}

/**
 * qla2x00_mbx_completion() - Process mailbox command completions.
 * @vha: SCSI driver HA context
 * @mb0: Mailbox0 register
 */
static void
qla2x00_mbx_completion(scsi_qla_host_t *vha, uint16_t mb0)
{
	uint16_t	cnt;
	uint32_t	mboxes;
	uint16_t __iomem *wptr;
	struct qla_hw_data *ha = vha->hw;
	struct device_reg_2xxx __iomem *reg = &ha->iobase->isp;

	/* Read all mbox registers? */
	WARN_ON_ONCE(ha->mbx_count > 32);
	mboxes = (1ULL << ha->mbx_count) - 1;
	if (!ha->mcp)
		ql_dbg(ql_dbg_async, vha, 0x5001, "MBX pointer ERROR.\n");
	else
		mboxes = ha->mcp->in_mb;

	/* Load return mailbox registers. */
	ha->flags.mbox_int = 1;
	ha->mailbox_out[0] = mb0;
	mboxes >>= 1;
	wptr = (uint16_t __iomem *)MAILBOX_REG(ha, reg, 1);

	for (cnt = 1; cnt < ha->mbx_count; cnt++) {
		if (IS_QLA2200(ha) && cnt == 8)
			wptr = (uint16_t __iomem *)MAILBOX_REG(ha, reg, 8);
		if ((cnt == 4 || cnt == 5) && (mboxes & BIT_0))
			ha->mailbox_out[cnt] = qla2x00_debounce_register(wptr);
		else if (mboxes & BIT_0)
			ha->mailbox_out[cnt] = RD_REG_WORD(wptr);

		wptr++;
		mboxes >>= 1;
	}
}

static void
qla81xx_idc_event(scsi_qla_host_t *vha, uint16_t aen, uint16_t descr)
{
	static char *event[] =
		{ "Complete", "Request Notification", "Time Extension" };
	int rval;
	struct device_reg_24xx __iomem *reg24 = &vha->hw->iobase->isp24;
	struct device_reg_82xx __iomem *reg82 = &vha->hw->iobase->isp82;
	uint16_t __iomem *wptr;
	uint16_t cnt, timeout, mb[QLA_IDC_ACK_REGS];

	/* Seed data -- mailbox1 -> mailbox7. */
	if (IS_QLA81XX(vha->hw) || IS_QLA83XX(vha->hw))
		wptr = (uint16_t __iomem *)&reg24->mailbox1;
	else if (IS_QLA8044(vha->hw))
		wptr = (uint16_t __iomem *)&reg82->mailbox_out[1];
	else
		return;

	for (cnt = 0; cnt < QLA_IDC_ACK_REGS; cnt++, wptr++)
		mb[cnt] = RD_REG_WORD(wptr);

	ql_dbg(ql_dbg_async, vha, 0x5021,
	    "Inter-Driver Communication %s -- "
	    "%04x %04x %04x %04x %04x %04x %04x.\n",
	    event[aen & 0xff], mb[0], mb[1], mb[2], mb[3],
	    mb[4], mb[5], mb[6]);
	switch (aen) {
	/* Handle IDC Error completion case. */
	case MBA_IDC_COMPLETE:
		if (mb[1] >> 15) {
			vha->hw->flags.idc_compl_status = 1;
			if (vha->hw->notify_dcbx_comp && !vha->vp_idx)
				complete(&vha->hw->dcbx_comp);
		}
		break;

	case MBA_IDC_NOTIFY:
		/* Acknowledgement needed? [Notify && non-zero timeout]. */
		timeout = (descr >> 8) & 0xf;
		ql_dbg(ql_dbg_async, vha, 0x5022,
		    "%lu Inter-Driver Communication %s -- ACK timeout=%d.\n",
		    vha->host_no, event[aen & 0xff], timeout);

		if (!timeout)
			return;
		rval = qla2x00_post_idc_ack_work(vha, mb);
		if (rval != QLA_SUCCESS)
			ql_log(ql_log_warn, vha, 0x5023,
			    "IDC failed to post ACK.\n");
		break;
	case MBA_IDC_TIME_EXT:
		vha->hw->idc_extend_tmo = descr;
		ql_dbg(ql_dbg_async, vha, 0x5087,
		    "%lu Inter-Driver Communication %s -- "
		    "Extend timeout by=%d.\n",
		    vha->host_no, event[aen & 0xff], vha->hw->idc_extend_tmo);
		break;
	}
}

#define LS_UNKNOWN	2
const char *
qla2x00_get_link_speed_str(struct qla_hw_data *ha, uint16_t speed)
{
	static const char *const link_speeds[] = {
		"1", "2", "?", "4", "8", "16", "32", "64", "10"
	};
#define	QLA_LAST_SPEED (ARRAY_SIZE(link_speeds) - 1)

	if (IS_QLA2100(ha) || IS_QLA2200(ha))
		return link_speeds[0];
	else if (speed == 0x13)
		return link_speeds[QLA_LAST_SPEED];
	else if (speed < QLA_LAST_SPEED)
		return link_speeds[speed];
	else
		return link_speeds[LS_UNKNOWN];
}

static void
qla83xx_handle_8200_aen(scsi_qla_host_t *vha, uint16_t *mb)
{
	struct qla_hw_data *ha = vha->hw;

	/*
	 * 8200 AEN Interpretation:
	 * mb[0] = AEN code
	 * mb[1] = AEN Reason code
	 * mb[2] = LSW of Peg-Halt Status-1 Register
	 * mb[6] = MSW of Peg-Halt Status-1 Register
	 * mb[3] = LSW of Peg-Halt Status-2 register
	 * mb[7] = MSW of Peg-Halt Status-2 register
	 * mb[4] = IDC Device-State Register value
	 * mb[5] = IDC Driver-Presence Register value
	 */
	ql_dbg(ql_dbg_async, vha, 0x506b, "AEN Code: mb[0] = 0x%x AEN reason: "
	    "mb[1] = 0x%x PH-status1: mb[2] = 0x%x PH-status1: mb[6] = 0x%x.\n",
	    mb[0], mb[1], mb[2], mb[6]);
	ql_dbg(ql_dbg_async, vha, 0x506c, "PH-status2: mb[3] = 0x%x "
	    "PH-status2: mb[7] = 0x%x Device-State: mb[4] = 0x%x "
	    "Drv-Presence: mb[5] = 0x%x.\n", mb[3], mb[7], mb[4], mb[5]);

	if (mb[1] & (IDC_PEG_HALT_STATUS_CHANGE | IDC_NIC_FW_REPORTED_FAILURE |
				IDC_HEARTBEAT_FAILURE)) {
		ha->flags.nic_core_hung = 1;
		ql_log(ql_log_warn, vha, 0x5060,
		    "83XX: F/W Error Reported: Check if reset required.\n");

		if (mb[1] & IDC_PEG_HALT_STATUS_CHANGE) {
			uint32_t protocol_engine_id, fw_err_code, err_level;

			/*
			 * IDC_PEG_HALT_STATUS_CHANGE interpretation:
			 *  - PEG-Halt Status-1 Register:
			 *	(LSW = mb[2], MSW = mb[6])
			 *	Bits 0-7   = protocol-engine ID
			 *	Bits 8-28  = f/w error code
			 *	Bits 29-31 = Error-level
			 *	    Error-level 0x1 = Non-Fatal error
			 *	    Error-level 0x2 = Recoverable Fatal error
			 *	    Error-level 0x4 = UnRecoverable Fatal error
			 *  - PEG-Halt Status-2 Register:
			 *	(LSW = mb[3], MSW = mb[7])
			 */
			protocol_engine_id = (mb[2] & 0xff);
			fw_err_code = (((mb[2] & 0xff00) >> 8) |
			    ((mb[6] & 0x1fff) << 8));
			err_level = ((mb[6] & 0xe000) >> 13);
			ql_log(ql_log_warn, vha, 0x5061, "PegHalt Status-1 "
			    "Register: protocol_engine_id=0x%x "
			    "fw_err_code=0x%x err_level=0x%x.\n",
			    protocol_engine_id, fw_err_code, err_level);
			ql_log(ql_log_warn, vha, 0x5062, "PegHalt Status-2 "
			    "Register: 0x%x%x.\n", mb[7], mb[3]);
			if (err_level == ERR_LEVEL_NON_FATAL) {
				ql_log(ql_log_warn, vha, 0x5063,
				    "Not a fatal error, f/w has recovered itself.\n");
			} else if (err_level == ERR_LEVEL_RECOVERABLE_FATAL) {
				ql_log(ql_log_fatal, vha, 0x5064,
				    "Recoverable Fatal error: Chip reset "
				    "required.\n");
				qla83xx_schedule_work(vha,
				    QLA83XX_NIC_CORE_RESET);
			} else if (err_level == ERR_LEVEL_UNRECOVERABLE_FATAL) {
				ql_log(ql_log_fatal, vha, 0x5065,
				    "Unrecoverable Fatal error: Set FAILED "
				    "state, reboot required.\n");
				qla83xx_schedule_work(vha,
				    QLA83XX_NIC_CORE_UNRECOVERABLE);
			}
		}

		if (mb[1] & IDC_NIC_FW_REPORTED_FAILURE) {
			uint16_t peg_fw_state, nw_interface_link_up;
			uint16_t nw_interface_signal_detect, sfp_status;
			uint16_t htbt_counter, htbt_monitor_enable;
			uint16_t sfp_additional_info, sfp_multirate;
			uint16_t sfp_tx_fault, link_speed, dcbx_status;

			/*
			 * IDC_NIC_FW_REPORTED_FAILURE interpretation:
			 *  - PEG-to-FC Status Register:
			 *	(LSW = mb[2], MSW = mb[6])
			 *	Bits 0-7   = Peg-Firmware state
			 *	Bit 8      = N/W Interface Link-up
			 *	Bit 9      = N/W Interface signal detected
			 *	Bits 10-11 = SFP Status
			 *	  SFP Status 0x0 = SFP+ transceiver not expected
			 *	  SFP Status 0x1 = SFP+ transceiver not present
			 *	  SFP Status 0x2 = SFP+ transceiver invalid
			 *	  SFP Status 0x3 = SFP+ transceiver present and
			 *	  valid
			 *	Bits 12-14 = Heartbeat Counter
			 *	Bit 15     = Heartbeat Monitor Enable
			 *	Bits 16-17 = SFP Additional Info
			 *	  SFP info 0x0 = Unregocnized transceiver for
			 *	  Ethernet
			 *	  SFP info 0x1 = SFP+ brand validation failed
			 *	  SFP info 0x2 = SFP+ speed validation failed
			 *	  SFP info 0x3 = SFP+ access error
			 *	Bit 18     = SFP Multirate
			 *	Bit 19     = SFP Tx Fault
			 *	Bits 20-22 = Link Speed
			 *	Bits 23-27 = Reserved
			 *	Bits 28-30 = DCBX Status
			 *	  DCBX Status 0x0 = DCBX Disabled
			 *	  DCBX Status 0x1 = DCBX Enabled
			 *	  DCBX Status 0x2 = DCBX Exchange error
			 *	Bit 31     = Reserved
			 */
			peg_fw_state = (mb[2] & 0x00ff);
			nw_interface_link_up = ((mb[2] & 0x0100) >> 8);
			nw_interface_signal_detect = ((mb[2] & 0x0200) >> 9);
			sfp_status = ((mb[2] & 0x0c00) >> 10);
			htbt_counter = ((mb[2] & 0x7000) >> 12);
			htbt_monitor_enable = ((mb[2] & 0x8000) >> 15);
			sfp_additional_info = (mb[6] & 0x0003);
			sfp_multirate = ((mb[6] & 0x0004) >> 2);
			sfp_tx_fault = ((mb[6] & 0x0008) >> 3);
			link_speed = ((mb[6] & 0x0070) >> 4);
			dcbx_status = ((mb[6] & 0x7000) >> 12);

			ql_log(ql_log_warn, vha, 0x5066,
			    "Peg-to-Fc Status Register:\n"
			    "peg_fw_state=0x%x, nw_interface_link_up=0x%x, "
			    "nw_interface_signal_detect=0x%x"
			    "\nsfp_statis=0x%x.\n ", peg_fw_state,
			    nw_interface_link_up, nw_interface_signal_detect,
			    sfp_status);
			ql_log(ql_log_warn, vha, 0x5067,
			    "htbt_counter=0x%x, htbt_monitor_enable=0x%x, "
			    "sfp_additional_info=0x%x, sfp_multirate=0x%x.\n ",
			    htbt_counter, htbt_monitor_enable,
			    sfp_additional_info, sfp_multirate);
			ql_log(ql_log_warn, vha, 0x5068,
			    "sfp_tx_fault=0x%x, link_state=0x%x, "
			    "dcbx_status=0x%x.\n", sfp_tx_fault, link_speed,
			    dcbx_status);

			qla83xx_schedule_work(vha, QLA83XX_NIC_CORE_RESET);
		}

		if (mb[1] & IDC_HEARTBEAT_FAILURE) {
			ql_log(ql_log_warn, vha, 0x5069,
			    "Heartbeat Failure encountered, chip reset "
			    "required.\n");

			qla83xx_schedule_work(vha, QLA83XX_NIC_CORE_RESET);
		}
	}

	if (mb[1] & IDC_DEVICE_STATE_CHANGE) {
		ql_log(ql_log_info, vha, 0x506a,
		    "IDC Device-State changed = 0x%x.\n", mb[4]);
		if (ha->flags.nic_core_reset_owner)
			return;
		qla83xx_schedule_work(vha, MBA_IDC_AEN);
	}
}

struct purex_item *
qla24xx_alloc_purex_item(scsi_qla_host_t *vha, uint16_t size)
{
	struct purex_item *item = NULL;
	uint8_t item_hdr_size = sizeof(*item);
	uint8_t default_usable = 0;

	if (size > QLA_DEFAULT_PAYLOAD_SIZE) {
		item = kzalloc(item_hdr_size +
		    (size - QLA_DEFAULT_PAYLOAD_SIZE), GFP_ATOMIC);
	} else {
		item = kzalloc(item_hdr_size, GFP_ATOMIC);
		default_usable = 1;
	}
	if (!item) {
		if (default_usable &&
		    (atomic_inc_return(&vha->default_item.in_use) == 1)) {
			item = &vha->default_item;
			goto initialize_purex_header;
		}
		ql_log(ql_log_warn, vha, 0x5092,
		       ">> Failed to allocate purex list item.\n");

		return NULL;
	}

initialize_purex_header:
	item->vha = vha;
	item->size = size;
	return item;
}

void
qla24xx_queue_purex_item(scsi_qla_host_t *vha, struct purex_item *pkt,
			 void (*process_item)(struct scsi_qla_host *vha,
					      struct purex_item *pkt))
{
	struct purex_list *list = &vha->purex_list;
	ulong flags;

	pkt->process_item = process_item;

	spin_lock_irqsave(&list->lock, flags);
	list_add_tail(&pkt->list, &list->head);
	spin_unlock_irqrestore(&list->lock, flags);

	set_bit(PROCESS_PUREX_IOCB, &vha->dpc_flags);
}

/**
 * qla24xx_copy_std_pkt() - Copy over purex ELS which is
 * contained in a single IOCB.
 * purex packet.
 * @vha: SCSI driver HA context
 * @pkt: ELS packet
 */
static struct purex_item
*qla24xx_copy_std_pkt(struct scsi_qla_host *vha, void *pkt)
{
	struct purex_item *item;

	item = qla24xx_alloc_purex_item(vha,
					QLA_DEFAULT_PAYLOAD_SIZE);
	if (!item)
		return item;

	memcpy(&item->iocb, pkt, sizeof(item->iocb));
	return item;
}

static uint
qla25xx_rdp_port_speed_capability(struct qla_hw_data *ha)
{
	if (IS_CNA_CAPABLE(ha))
		return RDP_PORT_SPEED_10GB;

	if (IS_QLA28XX(ha) || IS_QLA27XX(ha)) {
		uint speeds = 0;
		if (ha->max_supported_speed == 2) {
			if (ha->min_supported_speed <= 6)
				speeds |= RDP_PORT_SPEED_64GB;
		}
		if (ha->max_supported_speed == 2 ||
		    ha->max_supported_speed == 1) {
			if (ha->min_supported_speed <= 5)
				speeds |= RDP_PORT_SPEED_32GB;
		}
		if (ha->max_supported_speed == 2 ||
		    ha->max_supported_speed == 1 ||
		    ha->max_supported_speed == 0) {
			if (ha->min_supported_speed <= 4)
				speeds |= RDP_PORT_SPEED_16GB;
		}
		if (ha->max_supported_speed == 1 ||
		    ha->max_supported_speed == 0) {
			if (ha->min_supported_speed <= 3)
				speeds |= RDP_PORT_SPEED_8GB;
		}
		if (ha->max_supported_speed == 0) {
			if (ha->min_supported_speed <= 2)
				speeds |= RDP_PORT_SPEED_4GB;
		}
		return speeds;
	}

	if (IS_QLA2031(ha))
		return RDP_PORT_SPEED_16GB|RDP_PORT_SPEED_8GB|
		       RDP_PORT_SPEED_4GB;

	if (IS_QLA25XX(ha))
		return RDP_PORT_SPEED_8GB|RDP_PORT_SPEED_4GB|
		       RDP_PORT_SPEED_2GB|RDP_PORT_SPEED_1GB;

	if (IS_QLA24XX_TYPE(ha))
		return RDP_PORT_SPEED_4GB|RDP_PORT_SPEED_2GB|
		       RDP_PORT_SPEED_1GB;

	if (IS_QLA23XX(ha))
		return RDP_PORT_SPEED_2GB|RDP_PORT_SPEED_1GB;

	return RDP_PORT_SPEED_1GB;
}

static uint
qla25xx_rdp_port_speed_currently(struct qla_hw_data *ha)
{
	switch (ha->link_data_rate) {
	case PORT_SPEED_1GB:
		return RDP_PORT_SPEED_1GB;
	case PORT_SPEED_2GB:
		return RDP_PORT_SPEED_2GB;
	case PORT_SPEED_4GB:
		return RDP_PORT_SPEED_4GB;
	case PORT_SPEED_8GB:
		return RDP_PORT_SPEED_8GB;
	case PORT_SPEED_10GB:
		return RDP_PORT_SPEED_10GB;
	case PORT_SPEED_16GB:
		return RDP_PORT_SPEED_16GB;
	case PORT_SPEED_32GB:
		return RDP_PORT_SPEED_32GB;
	case PORT_SPEED_64GB:
		return RDP_PORT_SPEED_64GB;
	default:
		return RDP_PORT_SPEED_UNKNOWN;
	}
}


/**
 * qla27xx_copy_fpin_pkt() - Copy over fpin packets that can
 * span over multiple IOCBs.
 * @vha: SCSI driver HA context
 * @pkt: ELS packet
 * @rsp: Response queue
 */
static struct purex_item *
qla27xx_copy_fpin_pkt(struct scsi_qla_host *vha, void **pkt,
		      struct rsp_que **rsp)
{
	struct purex_entry_24xx *purex = *pkt;
	struct rsp_que *rsp_q = *rsp;
	sts_cont_entry_t *new_pkt;
	uint16_t no_bytes = 0, total_bytes = 0, pending_bytes = 0;
	uint16_t buffer_copy_offset = 0;
	uint16_t entry_count, entry_count_remaining;
	struct purex_item *item;
	void *fpin_pkt = NULL;

	total_bytes = le16_to_cpu(purex->frame_size & 0x0FFF)
	    - PURX_ELS_HEADER_SIZE;
	pending_bytes = total_bytes;
	entry_count = entry_count_remaining = purex->entry_count;
	no_bytes = (pending_bytes > sizeof(purex->els_frame_payload))  ?
		   sizeof(purex->els_frame_payload) : pending_bytes;
	ql_dbg(ql_dbg_async, vha, 0x509a,
	       "FPIN ELS, frame_size 0x%x, entry count %d\n",
	       total_bytes, entry_count);

	item = qla24xx_alloc_purex_item(vha, total_bytes);
	if (!item)
		return item;

	fpin_pkt = &item->iocb;

	memcpy(fpin_pkt, &purex->els_frame_payload[0], no_bytes);
	buffer_copy_offset += no_bytes;
	pending_bytes -= no_bytes;
	--entry_count_remaining;

	((response_t *)purex)->signature = RESPONSE_PROCESSED;
	wmb();

	do {
		while ((total_bytes > 0) && (entry_count_remaining > 0)) {
			if (rsp_q->ring_ptr->signature == RESPONSE_PROCESSED) {
				ql_dbg(ql_dbg_async, vha, 0x5084,
				       "Ran out of IOCBs, partial data 0x%x\n",
				       buffer_copy_offset);
				cpu_relax();
				continue;
			}

			new_pkt = (sts_cont_entry_t *)rsp_q->ring_ptr;
			*pkt = new_pkt;

			if (new_pkt->entry_type != STATUS_CONT_TYPE) {
				ql_log(ql_log_warn, vha, 0x507a,
				       "Unexpected IOCB type, partial data 0x%x\n",
				       buffer_copy_offset);
				break;
			}

			rsp_q->ring_index++;
			if (rsp_q->ring_index == rsp_q->length) {
				rsp_q->ring_index = 0;
				rsp_q->ring_ptr = rsp_q->ring;
			} else {
				rsp_q->ring_ptr++;
			}
			no_bytes = (pending_bytes > sizeof(new_pkt->data)) ?
			    sizeof(new_pkt->data) : pending_bytes;
			if ((buffer_copy_offset + no_bytes) <= total_bytes) {
				memcpy(((uint8_t *)fpin_pkt +
				    buffer_copy_offset), new_pkt->data,
				    no_bytes);
				buffer_copy_offset += no_bytes;
				pending_bytes -= no_bytes;
				--entry_count_remaining;
			} else {
				ql_log(ql_log_warn, vha, 0x5044,
				       "Attempt to copy more that we got, optimizing..%x\n",
				       buffer_copy_offset);
				memcpy(((uint8_t *)fpin_pkt +
				    buffer_copy_offset), new_pkt->data,
				    total_bytes - buffer_copy_offset);
			}

			((response_t *)new_pkt)->signature = RESPONSE_PROCESSED;
			wmb();
		}

		if (pending_bytes != 0 || entry_count_remaining != 0) {
			ql_log(ql_log_fatal, vha, 0x508b,
			       "Dropping partial FPIN, underrun bytes = 0x%x, entry cnts 0x%x\n",
			       total_bytes, entry_count_remaining);
			qla24xx_free_purex_item(item);
			return NULL;
		}
	} while (entry_count_remaining > 0);
	host_to_fcp_swap((uint8_t *)&item->iocb, total_bytes);
	return item;
}

int
qla2x00_is_a_vp_did(scsi_qla_host_t *vha, uint32_t rscn_entry)
{
	struct qla_hw_data *ha = vha->hw;
	scsi_qla_host_t *vp;
	uint32_t vp_did;
	unsigned long flags;
	int ret = 0;

	if (!ha->num_vhosts)
		return ret;

	spin_lock_irqsave(&ha->vport_slock, flags);
	list_for_each_entry(vp, &ha->vp_list, list) {
		vp_did = vp->d_id.b24;
		if (vp_did == rscn_entry) {
			ret = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&ha->vport_slock, flags);

	return ret;
}

fc_port_t *
qla2x00_find_fcport_by_loopid(scsi_qla_host_t *vha, uint16_t loop_id)
{
	fc_port_t *f, *tf;

	f = tf = NULL;
	list_for_each_entry_safe(f, tf, &vha->vp_fcports, list)
		if (f->loop_id == loop_id)
			return f;
	return NULL;
}

fc_port_t *
qla2x00_find_fcport_by_wwpn(scsi_qla_host_t *vha, u8 *wwpn, u8 incl_deleted)
{
	fc_port_t *f, *tf;

	f = tf = NULL;
	list_for_each_entry_safe(f, tf, &vha->vp_fcports, list) {
		if (memcmp(f->port_name, wwpn, WWN_SIZE) == 0) {
			if (incl_deleted)
				return f;
			else if (f->deleted == 0)
				return f;
		}
	}
	return NULL;
}

fc_port_t *
qla2x00_find_fcport_by_nportid(scsi_qla_host_t *vha, port_id_t *id,
	u8 incl_deleted)
{
	fc_port_t *f, *tf;

	f = tf = NULL;
	list_for_each_entry_safe(f, tf, &vha->vp_fcports, list) {
		if (f->d_id.b24 == id->b24) {
			if (incl_deleted)
				return f;
			else if (f->deleted == 0)
				return f;
		}
	}
	return NULL;
}

/* Shall be called only on supported adapters. */
static void
qla27xx_handle_8200_aen(scsi_qla_host_t *vha, uint16_t *mb)
{
	struct qla_hw_data *ha = vha->hw;
	bool reset_isp_needed = 0;

	ql_log(ql_log_warn, vha, 0x02f0,
			"MPI Heartbeat stop. MPI reset is%s needed. "
			"MB0[%xh] MB1[%xh] MB2[%xh] MB3[%xh]\n",
			 mb[1] & BIT_8 ? "" : " not",
			 mb[0], mb[1], mb[2], mb[3]);

	if ((mb[1] & BIT_8) == 0)
		return;

	ql_log(ql_log_warn, vha, 0x02f1,
		"MPI Heartbeat stop. FW dump needed\n");

	if (ql2xfulldump_on_mpifail) {
		ha->isp_ops->fw_dump(vha, 1);
		reset_isp_needed = 1;
	}

	ha->isp_ops->mpi_fw_dump(vha, 1);

	if (reset_isp_needed) {
		vha->hw->flags.fw_init_done = 0;
		set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		qla2xxx_wake_dpc(vha);
	}
}


/**
 * qla2x00_async_event() - Process aynchronous events.
 * @vha: SCSI driver HA context
 * @rsp: response queue
 * @mb: Mailbox registers (0 - 3)
 */
void
qla2x00_async_event(scsi_qla_host_t *vha, struct rsp_que *rsp, uint16_t *mb)
{
	uint16_t	handle_cnt;
	uint16_t	cnt, mbx;
	uint32_t	handles[5];
	struct qla_hw_data *ha = vha->hw;
	struct device_reg_2xxx __iomem *reg = &ha->iobase->isp;
	struct device_reg_24xx __iomem *reg24 = &ha->iobase->isp24;
	struct device_reg_82xx __iomem *reg82 = &ha->iobase->isp82;
	uint32_t	rscn_entry, host_pid;
	unsigned long	flags;
	fc_port_t	*fcport = NULL;

	if (!vha->hw->flags.fw_started)
		return;

	/* Setup to process RIO completion. */
	handle_cnt = 0;
	if (IS_CNA_CAPABLE(ha))
		goto skip_rio;
	switch (mb[0]) {
	case MBA_SCSI_COMPLETION:
		handles[0] = le32_to_cpu((uint32_t)((mb[2] << 16) | mb[1]));
		handle_cnt = 1;
		break;
	case MBA_CMPLT_1_16BIT:
		handles[0] = mb[1];
		handle_cnt = 1;
		mb[0] = MBA_SCSI_COMPLETION;
		break;
	case MBA_CMPLT_2_16BIT:
		handles[0] = mb[1];
		handles[1] = mb[2];
		handle_cnt = 2;
		mb[0] = MBA_SCSI_COMPLETION;
		break;
	case MBA_CMPLT_3_16BIT:
		handles[0] = mb[1];
		handles[1] = mb[2];
		handles[2] = mb[3];
		handle_cnt = 3;
		mb[0] = MBA_SCSI_COMPLETION;
		break;
	case MBA_CMPLT_4_16BIT:
		handles[0] = mb[1];
		handles[1] = mb[2];
		handles[2] = mb[3];
		handles[3] = (uint32_t)RD_MAILBOX_REG(ha, reg, 6);
		handle_cnt = 4;
		mb[0] = MBA_SCSI_COMPLETION;
		break;
	case MBA_CMPLT_5_16BIT:
		handles[0] = mb[1];
		handles[1] = mb[2];
		handles[2] = mb[3];
		handles[3] = (uint32_t)RD_MAILBOX_REG(ha, reg, 6);
		handles[4] = (uint32_t)RD_MAILBOX_REG(ha, reg, 7);
		handle_cnt = 5;
		mb[0] = MBA_SCSI_COMPLETION;
		break;
	case MBA_CMPLT_2_32BIT:
		handles[0] = le32_to_cpu((uint32_t)((mb[2] << 16) | mb[1]));
		handles[1] = le32_to_cpu(
		    ((uint32_t)(RD_MAILBOX_REG(ha, reg, 7) << 16)) |
		    RD_MAILBOX_REG(ha, reg, 6));
		handle_cnt = 2;
		mb[0] = MBA_SCSI_COMPLETION;
		break;
	default:
		break;
	}
skip_rio:
	switch (mb[0]) {
	case MBA_SCSI_COMPLETION:	/* Fast Post */
		if (!vha->flags.online)
			break;

		for (cnt = 0; cnt < handle_cnt; cnt++)
			qla2x00_process_completed_request(vha, rsp->req,
				handles[cnt]);
		break;

	case MBA_RESET:			/* Reset */
		ql_dbg(ql_dbg_async, vha, 0x5002,
		    "Asynchronous RESET.\n");

		set_bit(RESET_MARKER_NEEDED, &vha->dpc_flags);
		break;

	case MBA_SYSTEM_ERR:		/* System Error */
		mbx = 0;

		vha->hw_err_cnt++;

		if (IS_QLA81XX(ha) || IS_QLA83XX(ha) ||
		    IS_QLA27XX(ha) || IS_QLA28XX(ha)) {
			u16 m[4];
			m[0] = RD_REG_WORD(&reg24->mailbox4);
			m[1] = RD_REG_WORD(&reg24->mailbox5);
			m[2] = RD_REG_WORD(&reg24->mailbox6);
			mbx = m[3] = RD_REG_WORD(&reg24->mailbox7);

			ql_log(ql_log_warn, vha, 0x5003,
			    "ISP System Error - mbx1=%xh mbx2=%xh mbx3=%xh "
			    "mbx4=%xh mbx5=%xh mbx6=%xh mbx7=%xh.\n",
			    mb[1], mb[2], mb[3], m[0], m[1], m[2], m[3]);
		} else
			ql_log(ql_log_warn, vha, 0x5003,
			    "ISP System Error - mbx1=%xh mbx2=%xh mbx3=%xh.\n ",
			    mb[1], mb[2], mb[3]);

		if ((IS_QLA27XX(ha) || IS_QLA28XX(ha)) &&
			RD_REG_WORD(&reg24->mailbox7) & BIT_8)
			ha->isp_ops->mpi_fw_dump(vha, 1);
		ha->isp_ops->fw_dump(vha, 1);
		ha->flags.fw_init_done = 0;
		QLA_FW_STOPPED(ha);

		if (IS_FWI2_CAPABLE(ha)) {
			if (mb[1] == 0 && mb[2] == 0) {
				ql_log(ql_log_fatal, vha, 0x5004,
				    "Unrecoverable Hardware Error: adapter "
				    "marked OFFLINE!\n");
				vha->flags.online = 0;
				vha->device_flags |= DFLG_DEV_FAILED;
			} else {
				/* Check to see if MPI timeout occurred */
				if ((mbx & MBX_3) && (ha->port_no == 0))
					set_bit(MPI_RESET_NEEDED,
					    &vha->dpc_flags);

				set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
			}
		} else if (mb[1] == 0) {
			ql_log(ql_log_fatal, vha, 0x5005,
			    "Unrecoverable Hardware Error: adapter marked "
			    "OFFLINE!\n");
			vha->flags.online = 0;
			vha->device_flags |= DFLG_DEV_FAILED;
		} else
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		break;

	case MBA_REQ_TRANSFER_ERR:	/* Request Transfer Error */
		ql_log(ql_log_warn, vha, 0x5006,
		    "ISP Request Transfer Error (%x).\n",  mb[1]);

		vha->hw_err_cnt++;

		set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		break;

	case MBA_RSP_TRANSFER_ERR:	/* Response Transfer Error */
		ql_log(ql_log_warn, vha, 0x5007,
		    "ISP Response Transfer Error (%x).\n", mb[1]);

		vha->hw_err_cnt++;

		set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		break;

	case MBA_WAKEUP_THRES:		/* Request Queue Wake-up */
		ql_dbg(ql_dbg_async, vha, 0x5008,
		    "Asynchronous WAKEUP_THRES (%x).\n", mb[1]);
		break;

	case MBA_LOOP_INIT_ERR:
		ql_log(ql_log_warn, vha, 0x5090,
		    "LOOP INIT ERROR (%x).\n", mb[1]);
		set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		break;

	case MBA_LIP_OCCURRED:		/* Loop Initialization Procedure */
		ha->flags.lip_ae = 1;

		ql_dbg(ql_dbg_async, vha, 0x5009,
		    "LIP occurred (%x).\n", mb[1]);

		if (atomic_read(&vha->loop_state) != LOOP_DOWN) {
			atomic_set(&vha->loop_state, LOOP_DOWN);
			atomic_set(&vha->loop_down_timer, LOOP_DOWN_TIME);
			qla2x00_mark_all_devices_lost(vha);
		}

		if (vha->vp_idx) {
			atomic_set(&vha->vp_state, VP_FAILED);
			fc_vport_set_state(vha->fc_vport, FC_VPORT_FAILED);
		}

		set_bit(REGISTER_FC4_NEEDED, &vha->dpc_flags);
		set_bit(REGISTER_FDMI_NEEDED, &vha->dpc_flags);

		vha->flags.management_server_logged_in = 0;
		qla2x00_post_aen_work(vha, FCH_EVT_LIP, mb[1]);
		break;

	case MBA_LOOP_UP:		/* Loop Up Event */
		if (IS_QLA2100(ha) || IS_QLA2200(ha))
			ha->link_data_rate = PORT_SPEED_1GB;
		else
			ha->link_data_rate = mb[1];

		ql_log(ql_log_info, vha, 0x500a,
		    "LOOP UP detected (%s Gbps).\n",
		    qla2x00_get_link_speed_str(ha, ha->link_data_rate));

		/* Reset Virtual Lane to Normal */
		qla_scm_host_clear_vl_state(vha);

		if (IS_QLA83XX(ha) || IS_QLA27XX(ha) || IS_QLA28XX(ha)) {
			if (mb[2] & BIT_0)
				ql_log(ql_log_info, vha, 0x11a0,
				    "FEC=enabled (link up).\n");
		}

		vha->flags.management_server_logged_in = 0;
		qla2x00_post_aen_work(vha, FCH_EVT_LINKUP, ha->link_data_rate);

		if (vha->link_down_time < vha->hw->port_down_retry_count) {
			vha->short_link_down_cnt++;
			vha->link_down_time = QLA2XX_MAX_LINK_DOWN_TIME;
		}

		break;

	case MBA_LOOP_DOWN:		/* Loop Down Event */
		SAVE_TOPO(ha);
		ha->flags.lip_ae = 0;
		ha->current_topology = 0;
		vha->link_down_time = 0;

		mbx = (IS_QLA81XX(ha) || IS_QLA8031(ha))
			? RD_REG_WORD(&reg24->mailbox4) : 0;
		mbx = (IS_P3P_TYPE(ha)) ? RD_REG_WORD(&reg82->mailbox_out[4])
			: mbx;
		ql_log(ql_log_info, vha, 0x500b,
		    "LOOP DOWN detected (%x %x %x %x).\n",
		    mb[1], mb[2], mb[3], mbx);

		if (atomic_read(&vha->loop_state) != LOOP_DOWN) {
			atomic_set(&vha->loop_state, LOOP_DOWN);
			atomic_set(&vha->loop_down_timer, LOOP_DOWN_TIME);
			/*
			 * In case of loop down, restore WWPN from
			 * NVRAM in case of FA-WWPN capable ISP
			 * Restore for Physical Port only
			 */
			if (!vha->vp_idx) {
				if (ha->flags.fawwpn_enabled &&
				    (ha->current_topology == ISP_CFG_F)) {
					memcpy(vha->port_name, ha->port_name, WWN_SIZE);
					fc_host_port_name(vha->host) =
					    wwn_to_u64(vha->port_name);
					ql_dbg(ql_dbg_init + ql_dbg_verbose,
					    vha, 0x00d8, "LOOP DOWN detected,"
					    "restore WWPN %016llx\n",
					    wwn_to_u64(vha->port_name));
				}

				clear_bit(VP_CONFIG_OK, &vha->vp_flags);
			}

			vha->device_flags |= DFLG_NO_CABLE;
			qla2x00_mark_all_devices_lost(vha);
		}

		if (vha->vp_idx) {
			atomic_set(&vha->vp_state, VP_FAILED);
			fc_vport_set_state(vha->fc_vport, FC_VPORT_FAILED);
		}

		vha->flags.management_server_logged_in = 0;
		ha->link_data_rate = PORT_SPEED_UNKNOWN;
		ha->flags.conn_fabric_cisco_er_rdy = 0;
		ha->flags.conn_fabric_brocade = 0;
		ha->scm.scm_fabric_connection_flags = 0;

		/* Clear SCM stats and throttling, if SCM is enabled */
		if (vha->hw->flags.scm_enabled) {
			qla2xxx_scmr_clear_congn(&ha->sfc);
			qla2xxx_scmr_clear_throttle(&ha->sfc);
			qla_scm_clear_previous_event(vha);
		}

		qla2x00_post_aen_work(vha, FCH_EVT_LINKDOWN, 0);
		break;

	case MBA_LIP_RESET:		/* LIP reset occurred */
		ql_dbg(ql_dbg_async, vha, 0x500c,
		    "LIP reset occurred (%x).\n", mb[1]);

		if (atomic_read(&vha->loop_state) != LOOP_DOWN) {
			atomic_set(&vha->loop_state, LOOP_DOWN);
			atomic_set(&vha->loop_down_timer, LOOP_DOWN_TIME);
			qla2x00_mark_all_devices_lost(vha);
		}

		if (vha->vp_idx) {
			atomic_set(&vha->vp_state, VP_FAILED);
			fc_vport_set_state(vha->fc_vport, FC_VPORT_FAILED);
		}

		set_bit(RESET_MARKER_NEEDED, &vha->dpc_flags);

		ha->operating_mode = LOOP;
		vha->flags.management_server_logged_in = 0;
		qla2x00_post_aen_work(vha, FCH_EVT_LIPRESET, mb[1]);
		break;

	/* case MBA_DCBX_COMPLETE: */
	case MBA_POINT_TO_POINT:	/* Point-to-Point */
		ha->flags.lip_ae = 0;

		if (IS_QLA2100(ha))
			break;

		if (IS_CNA_CAPABLE(ha)) {
			ql_dbg(ql_dbg_async, vha, 0x500d,
			    "DCBX Completed -- %04x %04x %04x.\n",
			    mb[1], mb[2], mb[3]);
			if (ha->notify_dcbx_comp && !vha->vp_idx)
				complete(&ha->dcbx_comp);

		} else
			ql_dbg(ql_dbg_async, vha, 0x500e,
			    "Asynchronous P2P MODE received.\n");

		/*
		 * Until there's a transition from loop down to loop up, treat
		 * this as loop down only.
		 */
		if (atomic_read(&vha->loop_state) != LOOP_DOWN) {
			atomic_set(&vha->loop_state, LOOP_DOWN);
			if (!atomic_read(&vha->loop_down_timer))
				atomic_set(&vha->loop_down_timer,
				    LOOP_DOWN_TIME);
			if (!N2N_TOPO(ha))
				qla2x00_mark_all_devices_lost(vha);
		}

		if (vha->vp_idx) {
			atomic_set(&vha->vp_state, VP_FAILED);
			fc_vport_set_state(vha->fc_vport, FC_VPORT_FAILED);
		}

		if (!(test_bit(ABORT_ISP_ACTIVE, &vha->dpc_flags)))
			set_bit(RESET_MARKER_NEEDED, &vha->dpc_flags);

		set_bit(REGISTER_FC4_NEEDED, &vha->dpc_flags);
		set_bit(REGISTER_FDMI_NEEDED, &vha->dpc_flags);

		vha->flags.management_server_logged_in = 0;
		break;

	case MBA_CHG_IN_CONNECTION:	/* Change in connection mode */
		if (IS_QLA2100(ha))
			break;

		ql_dbg(ql_dbg_async, vha, 0x500f,
		    "Configuration change detected: value=%x.\n", mb[1]);

		if (atomic_read(&vha->loop_state) != LOOP_DOWN) {
			atomic_set(&vha->loop_state, LOOP_DOWN);
			if (!atomic_read(&vha->loop_down_timer))
				atomic_set(&vha->loop_down_timer,
				    LOOP_DOWN_TIME);
			qla2x00_mark_all_devices_lost(vha);
		}

		if (vha->vp_idx) {
			atomic_set(&vha->vp_state, VP_FAILED);
			fc_vport_set_state(vha->fc_vport, FC_VPORT_FAILED);
		}

		set_bit(LOOP_RESYNC_NEEDED, &vha->dpc_flags);
		set_bit(LOCAL_LOOP_UPDATE, &vha->dpc_flags);
		break;

	case MBA_PORT_UPDATE:		/* Port database update */
		/*
		 * Handle only global and vn-port update events
		 *
		 * Relevant inputs:
		 * mb[1] = N_Port handle of changed port
		 * OR 0xffff for global event
		 * mb[2] = New login state
		 * 7 = Port logged out
		 * mb[3] = LSB is vp_idx, 0xff = all vps
		 *
		 * Skip processing if:
		 *       Event is global, vp_idx is NOT all vps,
		 *           vp_idx does not match
		 *       Event is not global, vp_idx does not match
		 */
		if (IS_QLA2XXX_MIDTYPE(ha) &&
		    ((mb[1] == 0xffff && (mb[3] & 0xff) != 0xff) ||
			(mb[1] != 0xffff)) && vha->vp_idx != (mb[3] & 0xff))
			break;

		if (mb[2] == 0x7) {
			ql_dbg(ql_dbg_async, vha, 0x5010,
			    "Port %s %04x %04x %04x.\n",
			    mb[1] == 0xffff ? "unavailable" : "logout",
			    mb[1], mb[2], mb[3]);

			if (mb[1] == 0xffff)
				goto global_port_update;

			if (mb[1] == NPH_F_PORT) {
				if (vha->vp_idx) {
					atomic_set(&vha->vp_state, VP_FAILED);
					fc_vport_set_state(vha->fc_vport,
					    FC_VPORT_FAILED);
				}
				/* F-Port LOGO. Logout from all devices. */
				qla2x00_mark_all_devices_lost(vha);
				vha->flags.management_server_logged_in = 0;

				break;
			}

			if (mb[1] == NPH_SNS_LID(ha)) {
				set_bit(LOOP_RESYNC_NEEDED, &vha->dpc_flags);
				set_bit(LOCAL_LOOP_UPDATE, &vha->dpc_flags);
				break;
			}

			/* use handle_cnt for loop id/nport handle */
			if (IS_FWI2_CAPABLE(ha))
				handle_cnt = NPH_SNS;
			else
				handle_cnt = SIMPLE_NAME_SERVER;
			if (mb[1] == handle_cnt) {
				set_bit(LOOP_RESYNC_NEEDED, &vha->dpc_flags);
				set_bit(LOCAL_LOOP_UPDATE, &vha->dpc_flags);
				break;
			}

			/* Port logout */
			fcport = qla2x00_find_fcport_by_loopid(vha, mb[1]);
			if (!fcport)
				break;
			if (atomic_read(&fcport->state) != FCS_ONLINE)
				break;
			ql_dbg(ql_dbg_async, vha, 0x508a,
			    "Marking port lost loopid=%04x portid=%06x.\n",
			    fcport->loop_id, fcport->d_id.b24);
			if (qla_ini_mode_enabled(vha)) {
				fcport->logout_on_delete = 0;
				qlt_schedule_sess_for_deletion(fcport);
			}
			break;

global_port_update:
			if (atomic_read(&vha->loop_state) != LOOP_DOWN) {
				atomic_set(&vha->loop_state, LOOP_DOWN);
				atomic_set(&vha->loop_down_timer,
				    LOOP_DOWN_TIME);
				vha->device_flags |= DFLG_NO_CABLE;
				qla2x00_mark_all_devices_lost(vha);
			}

			if (vha->vp_idx) {
				atomic_set(&vha->vp_state, VP_FAILED);
				fc_vport_set_state(vha->fc_vport,
				    FC_VPORT_FAILED);
				qla2x00_mark_all_devices_lost(vha);
			}

			vha->flags.management_server_logged_in = 0;
			ha->link_data_rate = PORT_SPEED_UNKNOWN;
			break;
		}

		/*
		 * If PORT UPDATE is global (received LIP_OCCURRED/LIP_RESET
		 * event etc. earlier indicating loop is down) then process
		 * it.  Otherwise ignore it and Wait for RSCN to come in.
		 */
		atomic_set(&vha->loop_down_timer, 0);
		if (atomic_read(&vha->loop_state) != LOOP_DOWN &&
			!ha->flags.n2n_ae  &&
		    atomic_read(&vha->loop_state) != LOOP_DEAD) {
			ql_dbg(ql_dbg_async, vha, 0x5011,
			    "Asynchronous PORT UPDATE ignored %04x/%04x/%04x.\n",
			    mb[1], mb[2], mb[3]);

			qlt_async_event(mb[0], vha, mb);
			break;
		}

		ql_dbg(ql_dbg_async, vha, 0x5012,
		    "Port database changed %04x %04x %04x.\n",
		    mb[1], mb[2], mb[3]);

		/*
		 * Mark all devices as missing so we will login again.
		 */
		atomic_set(&vha->loop_state, LOOP_UP);
		vha->scan.scan_retry = 0;

		set_bit(LOOP_RESYNC_NEEDED, &vha->dpc_flags);
		set_bit(LOCAL_LOOP_UPDATE, &vha->dpc_flags);
		set_bit(VP_CONFIG_OK, &vha->vp_flags);

		qlt_async_event(mb[0], vha, mb);
		break;

	case MBA_RSCN_UPDATE:		/* State Change Registration */
		/* Check if the Vport has issued a SCR */
		if (vha->vp_idx && test_bit(VP_SCR_NEEDED, &vha->vp_flags))
			break;
		/* Only handle SCNs for our Vport index. */
		if (ha->flags.npiv_supported && vha->vp_idx != (mb[3] & 0xff))
			break;

		ql_log(ql_log_warn, vha, 0x5013,
		    "RSCN database changed -- %04x %04x %04x.\n",
		    mb[1], mb[2], mb[3]);

		rscn_entry = ((mb[1] & 0xff) << 16) | mb[2];
		host_pid = (vha->d_id.b.domain << 16) | (vha->d_id.b.area << 8)
				| vha->d_id.b.al_pa;
		if (rscn_entry == host_pid) {
			ql_dbg(ql_dbg_async, vha, 0x5014,
			    "Ignoring RSCN update to local host "
			    "port ID (%06x).\n", host_pid);
			break;
		}

		/* Ignore reserved bits from RSCN-payload. */
		rscn_entry = ((mb[1] & 0x3ff) << 16) | mb[2];

		/* Skip RSCNs for virtual ports on the same physical port */
		if (qla2x00_is_a_vp_did(vha, rscn_entry))
			break;

		atomic_set(&vha->loop_down_timer, 0);
		vha->flags.management_server_logged_in = 0;
		{
			struct event_arg ea;

			memset(&ea, 0, sizeof(ea));
			ea.id.b24 = rscn_entry;
			ea.id.b.rsvd_1 = rscn_entry >> 24;
			qla2x00_handle_rscn(vha, &ea);
			qla2x00_post_aen_work(vha, FCH_EVT_RSCN, rscn_entry);
		}
		break;
	case MBA_CONGESTION_NOTIFICATION_RECV:
		if (!ha->flags.scm_enabled ||
		    mb[1] != QLA_CON_PRIMITIVE_RECEIVED)
			break;
		if (mb[2] == QLA_CONGESTION_ARB_WARNING) {
			ha->scm.sev.cn_warning++;
			ha->sig_sev.cn_warning_sig++;
			atomic_inc(&ha->sfc.num_sig_warning);
			ha->scm.congestion.severity =
				SCM_CONGESTION_SEVERITY_WARNING;
		} else if (mb[2] == QLA_CONGESTION_ARB_ALARM) {
			ha->scm.sev.cn_alarm++;
			ha->sig_sev.cn_alarm_sig++;
			atomic_inc(&ha->sfc.num_sig_alarm);
			ha->scm.congestion.severity =
				SCM_CONGESTION_SEVERITY_ERROR;
		}
		ha->sfc.event_period = 1;
		ha->sfc.throttle_period = 1;
		ha->sfc.event_period_buffer = 0;
		break;
	/* case MBA_RIO_RESPONSE: */
	case MBA_ZIO_RESPONSE:
		ql_dbg(ql_dbg_async, vha, 0x5015,
		    "[R|Z]IO update completion.\n");

		if (IS_FWI2_CAPABLE(ha))
			qla24xx_process_response_queue(vha, rsp);
		else
			qla2x00_process_response_queue(rsp);
		break;

	case MBA_DISCARD_RND_FRAME:
		ql_dbg(ql_dbg_async, vha, 0x5016,
		    "Discard RND Frame -- %04x %04x %04x.\n",
		    mb[1], mb[2], mb[3]);
		vha->interface_err_cnt++;
		break;

	case MBA_TRACE_NOTIFICATION:
		ql_dbg(ql_dbg_async, vha, 0x5017,
		    "Trace Notification -- %04x %04x.\n", mb[1], mb[2]);
		break;

	case MBA_ISP84XX_ALERT:
		ql_dbg(ql_dbg_async, vha, 0x5018,
		    "ISP84XX Alert Notification -- %04x %04x %04x.\n",
		    mb[1], mb[2], mb[3]);

		spin_lock_irqsave(&ha->cs84xx->access_lock, flags);
		switch (mb[1]) {
		case A84_PANIC_RECOVERY:
			ql_log(ql_log_info, vha, 0x5019,
			    "Alert 84XX: panic recovery %04x %04x.\n",
			    mb[2], mb[3]);
			break;
		case A84_OP_LOGIN_COMPLETE:
			ha->cs84xx->op_fw_version = mb[3] << 16 | mb[2];
			ql_log(ql_log_info, vha, 0x501a,
			    "Alert 84XX: firmware version %x.\n",
			    ha->cs84xx->op_fw_version);
			break;
		case A84_DIAG_LOGIN_COMPLETE:
			ha->cs84xx->diag_fw_version = mb[3] << 16 | mb[2];
			ql_log(ql_log_info, vha, 0x501b,
			    "Alert 84XX: diagnostic firmware version %x.\n",
			    ha->cs84xx->diag_fw_version);
			break;
		case A84_GOLD_LOGIN_COMPLETE:
			ha->cs84xx->diag_fw_version = mb[3] << 16 | mb[2];
			ha->cs84xx->fw_update = 1;
			ql_log(ql_log_info, vha, 0x501c,
			    "Alert 84XX: gold firmware version %x.\n",
			    ha->cs84xx->gold_fw_version);
			break;
		default:
			ql_log(ql_log_warn, vha, 0x501d,
			    "Alert 84xx: Invalid Alert %04x %04x %04x.\n",
			    mb[1], mb[2], mb[3]);
		}
		spin_unlock_irqrestore(&ha->cs84xx->access_lock, flags);
		break;
	case MBA_DCBX_START:
		ql_dbg(ql_dbg_async, vha, 0x501e,
		    "DCBX Started -- %04x %04x %04x.\n",
		    mb[1], mb[2], mb[3]);
		break;
	case MBA_DCBX_PARAM_UPDATE:
		ql_dbg(ql_dbg_async, vha, 0x501f,
		    "DCBX Parameters Updated -- %04x %04x %04x.\n",
		    mb[1], mb[2], mb[3]);
		break;
	case MBA_FCF_CONF_ERR:
		ql_dbg(ql_dbg_async, vha, 0x5020,
		    "FCF Configuration Error -- %04x %04x %04x.\n",
		    mb[1], mb[2], mb[3]);
		break;
	case MBA_IDC_NOTIFY:
		if (IS_QLA8031(vha->hw) || IS_QLA8044(ha)) {
			mb[4] = RD_REG_WORD(&reg24->mailbox4);
			if (((mb[2] & 0x7fff) == MBC_PORT_RESET ||
			    (mb[2] & 0x7fff) == MBC_SET_PORT_CONFIG) &&
			    (mb[4] & INTERNAL_LOOPBACK_MASK) != 0) {
				set_bit(ISP_QUIESCE_NEEDED, &vha->dpc_flags);
				/*
				 * Extend loop down timer since port is active.
				 */
				if (atomic_read(&vha->loop_state) == LOOP_DOWN)
					atomic_set(&vha->loop_down_timer,
					    LOOP_DOWN_TIME);
				qla2xxx_wake_dpc(vha);
			}
		}
		fallthrough;
	case MBA_IDC_COMPLETE:
		if (ha->notify_lb_portup_comp && !vha->vp_idx)
			complete(&ha->lb_portup_comp);
		fallthrough;
	case MBA_IDC_TIME_EXT:
		if (IS_QLA81XX(vha->hw) || IS_QLA8031(vha->hw) ||
		    IS_QLA8044(ha))
			qla81xx_idc_event(vha, mb[0], mb[1]);
		break;

	case MBA_IDC_AEN:
		if (IS_QLA27XX(ha) || IS_QLA28XX(ha)) {
			vha->hw_err_cnt++;
			qla27xx_handle_8200_aen(vha, mb);
		} else if (IS_QLA83XX(ha)) {
			mb[4] = RD_REG_WORD(&reg24->mailbox4);
			mb[5] = RD_REG_WORD(&reg24->mailbox5);
			mb[6] = RD_REG_WORD(&reg24->mailbox6);
			mb[7] = RD_REG_WORD(&reg24->mailbox7);
			qla83xx_handle_8200_aen(vha, mb);
		} else {
			ql_dbg(ql_dbg_async, vha, 0x5052,
			    "skip Heartbeat processing mb0-3=[0x%04x] [0x%04x] [0x%04x] [0x%04x]\n",
			    mb[0], mb[1], mb[2], mb[3]);
		}
		break;

	case MBA_DPORT_DIAGNOSTICS:
		if ((mb[1]&0xF) == AEN_DONE_DIAG_TEST_WITH_NOERR ||
			 (mb[1]&0xF) == AEN_DONE_DIAG_TEST_WITH_ERR)
			vha->dport_status &= ~DPORT_DIAG_IN_PROGRESS;
		ql_dbg(ql_dbg_async, vha, 0x5052,
		    "D-Port Diagnostics: %04x %04x %04x %04x\n",
		    mb[0], mb[1], mb[2], mb[3]);
		memcpy(vha->dport_data, mb, sizeof(vha->dport_data));
		if (IS_QLA83XX(ha) || IS_QLA27XX(ha) || IS_QLA28XX(ha)) {
			static char *results[] = {
			    "start", "done(pass)", "done(error)", "undefined" };
			static char *types[] = {
			    "none", "dynamic", "static", "other" };
			uint result = mb[1] >> 0 & 0x3;
			uint type = mb[1] >> 6 & 0x3;
			uint sw = mb[1] >> 15 & 0x1;
			ql_dbg(ql_dbg_async, vha, 0x5052,
			    "D-Port Diagnostics: result=%s type=%s [sw=%u]\n",
			    results[result], types[type], sw);
			if (result == 2) {
				static char *reasons[] = {
				    "reserved", "unexpected reject",
				    "unexpected phase", "retry exceeded",
				    "timed out", "not supported",
				    "user stopped" };
				uint reason = mb[2] >> 0 & 0xf;
				uint phase = mb[2] >> 12 & 0xf;
				ql_dbg(ql_dbg_async, vha, 0x5052,
				    "D-Port Diagnostics: reason=%s phase=%u \n",
				    reason < 7 ? reasons[reason] : "other",
				    phase >> 1);
			}
		}
		break;

	case MBA_TEMPERATURE_ALERT:
		ql_dbg(ql_dbg_async, vha, 0x505e,
		    "TEMPERATURE ALERT: %04x %04x %04x\n", mb[1], mb[2], mb[3]);
		//if (mb[1] == 0x12)
			//schedule_work(&ha->board_disable);
		break;

	case MBA_TRANS_INSERT:
		ql_dbg(ql_dbg_async, vha, 0x5091,
		    "Transceiver Insertion: %04x\n", mb[1]);
		set_bit(DETECT_SFP_CHANGE, &vha->dpc_flags);
		break;

	case MBA_TRANS_REMOVE:
		ql_dbg(ql_dbg_async, vha, 0x5091, "Transceiver Removal\n");
		break;

	default:
		ql_dbg(ql_dbg_async, vha, 0x5057,
		    "Unknown AEN:%04x %04x %04x %04x\n",
		    mb[0], mb[1], mb[2], mb[3]);
	}

	qlt_async_event(mb[0], vha, mb);

	if (!vha->vp_idx && ha->num_vhosts)
		qla2x00_alert_all_vps(rsp, mb);
}

/**
 * qla2x00_process_completed_request() - Process a Fast Post response.
 * @vha: SCSI driver HA context
 * @req: request queue
 * @index: SRB index
 */
void
qla2x00_process_completed_request(struct scsi_qla_host *vha,
				  struct req_que *req, uint32_t index)
{
	srb_t *sp;
	struct qla_hw_data *ha = vha->hw;

	/* Validate handle. */
	if (index >= req->num_outstanding_cmds) {
		ql_log(ql_log_warn, vha, 0x3014,
		    "Invalid SCSI command index (%x).\n", index);

		if (IS_P3P_TYPE(ha))
			set_bit(FCOE_CTX_RESET_NEEDED, &vha->dpc_flags);
		else
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		return;
	}

	sp = req->outstanding_cmds[index];
	if (sp) {
		/* Free outstanding command slot. */
		req->outstanding_cmds[index] = NULL;

		/* Save ISP completion status */
		sp->done(sp, DID_OK << 16);
	} else {
		ql_log(ql_log_warn, vha, 0x3016, "Invalid SCSI SRB.\n");

		if (IS_P3P_TYPE(ha))
			set_bit(FCOE_CTX_RESET_NEEDED, &vha->dpc_flags);
		else
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
	}
}

srb_t *
qla2x00_get_sp_from_handle(scsi_qla_host_t *vha, const char *func,
    struct req_que *req, void *iocb)
{
	struct qla_hw_data *ha = vha->hw;
	sts_entry_t *pkt = iocb;
	srb_t *sp = NULL;
	uint16_t index;

	if (pkt->handle == QLA_SKIP_HANDLE)
		goto done;

	index = LSW(pkt->handle);
	if (index >= req->num_outstanding_cmds) {
		ql_log(ql_log_warn, vha, 0x5031,
			   "Invalid command index (%x) type %8ph.\n",
			   index, iocb);

		if (is_debug(QDBG_FW_DUMP))
			ha->isp_ops->fw_dump(vha, 1);

		BUG_ON(is_debug(QDBG_CRASH_ON_ERR));

		if (IS_P3P_TYPE(ha))
			set_bit(FCOE_CTX_RESET_NEEDED, &vha->dpc_flags);
		else
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		goto done;
	}
	sp = req->outstanding_cmds[index];
	if (!sp) {
		ql_log(ql_log_warn, vha, 0x5032,
		    "Invalid completion handle (%x) -- timed-out.\n", index);
		return sp;
	}
	if (sp->handle != index) {
		ql_log(ql_log_warn, vha, 0x5033,
		    "SRB handle (%x) mismatch %x.\n", sp->handle, index);
		return NULL;
	}

	req->outstanding_cmds[index] = NULL;

	qla_put_fw_resources(sp->qpair, &sp->iores);
done:
	return sp;
}

static void
qla2x00_mbx_iocb_entry(scsi_qla_host_t *vha, struct req_que *req,
    struct mbx_entry *mbx)
{
	const char func[] = "MBX-IOCB";
	const char *type;
	fc_port_t *fcport;
	srb_t *sp;
	struct srb_iocb *lio;
	uint16_t *data;
	uint16_t status;

	sp = qla2x00_get_sp_from_handle(vha, func, req, mbx);
	if (!sp)
		return;

	lio = &sp->u.iocb_cmd;
	type = sp->name;
	fcport = sp->fcport;
	data = lio->u.logio.data;

	data[0] = MBS_COMMAND_ERROR;
	data[1] = lio->u.logio.flags & SRB_LOGIN_RETRIED ?
	    QLA_LOGIO_LOGIN_RETRIED : 0;
	if (mbx->entry_status) {
		ql_dbg(ql_dbg_async, vha, 0x5043,
		    "Async-%s error entry - hdl=%x portid=%02x%02x%02x "
		    "entry-status=%x status=%x state-flag=%x "
		    "status-flags=%x.\n", type, sp->handle,
		    fcport->d_id.b.domain, fcport->d_id.b.area,
		    fcport->d_id.b.al_pa, mbx->entry_status,
		    le16_to_cpu(mbx->status), le16_to_cpu(mbx->state_flags),
		    le16_to_cpu(mbx->status_flags));

		ql_dump_buffer(ql_dbg_async + ql_dbg_buffer, vha, 0x5029,
		    mbx, sizeof(*mbx));

		goto logio_done;
	}

	status = le16_to_cpu(mbx->status);
	if (status == 0x30 && sp->type == SRB_LOGIN_CMD &&
	    le16_to_cpu(mbx->mb0) == MBS_COMMAND_COMPLETE)
		status = 0;
	if (!status && le16_to_cpu(mbx->mb0) == MBS_COMMAND_COMPLETE) {
		ql_dbg(ql_dbg_async, vha, 0x5045,
		    "Async-%s complete - hdl=%x portid=%02x%02x%02x mbx1=%x.\n",
		    type, sp->handle, fcport->d_id.b.domain,
		    fcport->d_id.b.area, fcport->d_id.b.al_pa,
		    le16_to_cpu(mbx->mb1));

		data[0] = MBS_COMMAND_COMPLETE;
		if (sp->type == SRB_LOGIN_CMD) {
			fcport->port_type = FCT_TARGET;
			if (le16_to_cpu(mbx->mb1) & BIT_0)
				fcport->port_type = FCT_INITIATOR;
			else if (le16_to_cpu(mbx->mb1) & BIT_1)
				fcport->flags |= FCF_FCP2_DEVICE;
		}
		goto logio_done;
	}

	data[0] = le16_to_cpu(mbx->mb0);
	switch (data[0]) {
	case MBS_PORT_ID_USED:
		data[1] = le16_to_cpu(mbx->mb1);
		break;
	case MBS_LOOP_ID_USED:
		break;
	default:
		data[0] = MBS_COMMAND_ERROR;
		break;
	}

	ql_log(ql_log_warn, vha, 0x5046,
	    "Async-%s failed - hdl=%x portid=%02x%02x%02x status=%x "
	    "mb0=%x mb1=%x mb2=%x mb6=%x mb7=%x.\n", type, sp->handle,
	    fcport->d_id.b.domain, fcport->d_id.b.area, fcport->d_id.b.al_pa,
	    status, le16_to_cpu(mbx->mb0), le16_to_cpu(mbx->mb1),
	    le16_to_cpu(mbx->mb2), le16_to_cpu(mbx->mb6),
	    le16_to_cpu(mbx->mb7));

logio_done:
	sp->done(sp, 0);
}

static void
qla24xx_mbx_iocb_entry(scsi_qla_host_t *vha, struct req_que *req,
    struct mbx_24xx_entry *pkt)
{
	const char func[] = "MBX-IOCB2";
	srb_t *sp;
	struct srb_iocb *si;
	u16 sz, i;
	int res;

	sp = qla2x00_get_sp_from_handle(vha, func, req, pkt);
	if (!sp)
		return;

	si = &sp->u.iocb_cmd;
	sz = min(ARRAY_SIZE(pkt->mb), ARRAY_SIZE(sp->u.iocb_cmd.u.mbx.in_mb));

	for (i = 0; i < sz; i++)
		si->u.mbx.in_mb[i] = le16_to_cpu(pkt->mb[i]);

	res = (si->u.mbx.in_mb[0] & MBS_MASK);

	sp->done(sp, res);
}

static void
qla24xxx_nack_iocb_entry(scsi_qla_host_t *vha, struct req_que *req,
    struct nack_to_isp *pkt)
{
	const char func[] = "nack";
	srb_t *sp;
	int res = 0;

	sp = qla2x00_get_sp_from_handle(vha, func, req, pkt);
	if (!sp)
		return;

	if (pkt->u.isp2x.status != cpu_to_le16(NOTIFY_ACK_SUCCESS))
		res = QLA_FUNCTION_FAILED;

	sp->done(sp, res);
}

static void
qla2x00_ct_entry(scsi_qla_host_t *vha, struct req_que *req,
    sts_entry_t *pkt, int iocb_type)
{
	const char func[] = "CT_IOCB";
	const char *type;
	srb_t *sp;
	bsg_job_t *bsg_job;
	struct fc_bsg_reply *bsg_reply;
	uint16_t comp_status;
	int res = 0;

	sp = qla2x00_get_sp_from_handle(vha, func, req, pkt);
	if (!sp)
		return;

	switch (sp->type) {
	case SRB_CT_CMD:
	    bsg_job = sp->u.bsg_job;
	    bsg_reply = bsg_job->reply;

	    type = "ct pass-through";

	    comp_status = le16_to_cpu(pkt->comp_status);

	    /*
	     * return FC_CTELS_STATUS_OK and leave the decoding of the ELS/CT
	     * fc payload  to the caller
	     */
	    bsg_reply->reply_data.ctels_reply.status = FC_CTELS_STATUS_OK;
	    bsg_job->reply_len = sizeof(struct fc_bsg_reply);

	    if (comp_status != CS_COMPLETE) {
		    if (comp_status == CS_DATA_UNDERRUN) {
			    res = DID_OK << 16;
			    bsg_reply->reply_payload_rcv_len =
				le16_to_cpu(pkt->rsp_info_len);

			    ql_log(ql_log_warn, vha, 0x5048,
				"CT pass-through-%s error comp_status=0x%x total_byte=0x%x.\n",
				type, comp_status,
				bsg_reply->reply_payload_rcv_len);
		    } else {
			    ql_log(ql_log_warn, vha, 0x5049,
				"CT pass-through-%s error comp_status=0x%x.\n",
				type, comp_status);
			    res = DID_ERROR << 16;
			    bsg_reply->reply_payload_rcv_len = 0;
		    }
		    ql_dump_buffer(ql_dbg_async + ql_dbg_buffer, vha, 0x5035,
			pkt, sizeof(*pkt));
	    } else {
		    res = DID_OK << 16;
		    bsg_reply->reply_payload_rcv_len =
			bsg_job->reply_payload.payload_len;
		    bsg_job->reply_len = 0;
	    }
	    break;
	case SRB_CT_PTHRU_CMD:
	    /*
	     * borrowing sts_entry_24xx.comp_status.
	     * same location as ct_entry_24xx.comp_status
	     */
	     res = qla2x00_chk_ms_status(vha, (ms_iocb_entry_t *)pkt,
		 (struct ct_sns_rsp *)sp->u.iocb_cmd.u.ctarg.rsp,
		 sp->name);
	     break;
	}

	sp->done(sp, res);
}

static void
qla24xx_els_ct_entry(scsi_qla_host_t *v, struct req_que *req,
    struct sts_entry_24xx *pkt, int iocb_type)
{
	const char func[] = "ELS_CT_IOCB";
	const char *type;
	srb_t *sp;
	bsg_job_t *bsg_job;
	struct fc_bsg_reply *bsg_reply;
	uint16_t comp_status;
	uint32_t fw_status[3];
	int res, logit=1;
	struct srb_iocb *els;
	uint n;
	scsi_qla_host_t *vha;
	struct els_sts_entry_24xx *e = (struct els_sts_entry_24xx*)pkt;

	sp = qla2x00_get_sp_from_handle(v, func, req, pkt);
	if (!sp)
		return;
	bsg_job = sp->u.bsg_job;
	vha = sp->vha;

	type = NULL;

	comp_status = fw_status[0] = le16_to_cpu(pkt->comp_status);
	fw_status[1] = le32_to_cpu(((struct els_sts_entry_24xx *)pkt)->error_subcode_1);
	fw_status[2] = le32_to_cpu(((struct els_sts_entry_24xx *)pkt)->error_subcode_2);

	switch (sp->type) {
	case SRB_ELS_CMD_RPT:
	case SRB_ELS_CMD_HST:
		type = "rpt hst";
		break;
	case SRB_ELS_EDC:
	case SRB_ELS_RDF:
		type = "scm els";
		logit = 0;
		break;
	case SRB_ELS_CMD_HST_NOLOGIN:
		type = "els";
		{
			struct els_entry_24xx *els = (void *)pkt;
			struct qla_bsg_auth_els_request *p =
				(struct qla_bsg_auth_els_request *)bsg_job->request;

			ql_dbg(ql_dbg_user, vha, 0x700f,
			     "%s %s complete portid=%02x%02x%02x status %x xchg %x bsg ptr %px\n",
			     __func__, sc_to_str(p->e.sub_cmd),
			     e->d_id[2],e->d_id[1],e->d_id[0],
			     comp_status, p->e.extra_rx_xchg_address, bsg_job);

			if (! (le16_to_cpu(els->control_flags) & ECF_PAYLOAD_DESCR_MASK)) {
				if (sp->remap.remapped) {
					n = sg_copy_from_buffer(
					   bsg_job->reply_payload.sg_list,
					   bsg_job->reply_payload.sg_cnt,
					   sp->remap.rsp.buf, sp->remap.rsp.len);
					ql_dbg(ql_dbg_user+ql_dbg_verbose, vha, 0x700e,
					   "%s: SG copied %x of %x\n",
					   __func__, n, sp->remap.rsp.len);
				} else {
					ql_dbg(ql_dbg_user, vha, 0x700f,
					   "%s: NOT REMAPPED (error)...!!!\n",
					   __func__);
				}
			}
		}
		break;
	case SRB_CT_CMD:
		type = "ct pass-through";
		break;
	case SRB_ELS_DCMD:
		type = "Driver ELS logo";
		if (iocb_type != ELS_IOCB_TYPE) {
			ql_dbg(ql_dbg_user, vha, 0x5047,
			    "Completing %s: (%px) type=%d.\n",
			    type, sp, sp->type);
			sp->done(sp, 0);
			return;
		}
		break;
	case SRB_CT_PTHRU_CMD:
		/* borrowing sts_entry_24xx.comp_status.
		   same location as ct_entry_24xx.comp_status
		 */
		res = qla2x00_chk_ms_status(sp->vha, (ms_iocb_entry_t *)pkt,
			(struct ct_sns_rsp *)sp->u.iocb_cmd.u.ctarg.rsp,
			sp->name);
		sp->done(sp, res);
		return;
	default:
		ql_dbg(ql_dbg_user, vha, 0x503e,
		    "Unrecognized SRB: (%px) type=%d.\n", sp, sp->type);
		return;
	}

	if (iocb_type == ELS_IOCB_TYPE) {
		els = &sp->u.iocb_cmd;
		els->u.els_plogi.fw_status[0] = fw_status[0];
		els->u.els_plogi.fw_status[1] = fw_status[1];
		els->u.els_plogi.fw_status[2] = fw_status[2];
		els->u.els_plogi.comp_status = fw_status[0];
		if (comp_status == CS_COMPLETE) {
			res =  DID_OK << 16;
		} else {
			if (comp_status == CS_DATA_UNDERRUN) {
				res =  DID_OK << 16;
				els->u.els_plogi.len =
				le16_to_cpu(((struct els_sts_entry_24xx *)
					pkt)->total_byte_count);

				if (sp->remap.remapped &&
				    ((u8*)sp->remap.rsp.buf)[0] == ELS_LS_ACC) {
					ql_dbg(ql_dbg_user, vha, 0x503f,
					    "%s IOCB Done LS_ACC %02x%02x%02x -> %02x%02x%02x",
					    __func__, e->s_id[0], e->s_id[2],e->s_id[1],
					    e->d_id[2], e->d_id[1], e->d_id[0]);
					logit = 0;
				}

			} else if (comp_status == CS_PORT_LOGGED_OUT) {
				ql_dbg(ql_dbg_disc, vha, 0x911e,
				    "%s %d sche delete\n", __func__, __LINE__);

				els->u.els_plogi.len = 0;
				res = DID_IMM_RETRY << 16;
				qlt_schedule_sess_for_deletion(sp->fcport);
			} else {
				els->u.els_plogi.len = 0;
				res = DID_ERROR << 16;
			}

			if (sp->remap.remapped &&
			    ((u8 *)sp->remap.rsp.buf)[0] == ELS_LS_RJT) {
				if (logit) {
					ql_dbg(ql_dbg_user, vha, 0x503f,
					    "%s IOCB Done LS_RJT hdl=%x comp_status=0x%x\n",
					    type, sp->handle, comp_status);

					ql_dbg(ql_dbg_user, vha, 0x503f,
					    "subcode 1=0x%x subcode 2=0x%x bytes=0x%x %02x%02x%02x -> %02x%02x%02x\n",
					    fw_status[1], fw_status[2],
					    le32_to_cpu(((struct els_sts_entry_24xx *)
						pkt)->total_byte_count),
					    e->s_id[0], e->s_id[2], e->s_id[1],
					    e->d_id[2], e->d_id[1], e->d_id[0]);
				}
				if (sp->fcport && sp->fcport->flags & FCF_FCSP_DEVICE &&
				    sp->type == SRB_ELS_CMD_HST_NOLOGIN) {
					ql_dbg(ql_dbg_edif, vha, 0x911e,
					    "%s rcv reject. Sched delete\n", __func__);
					qlt_schedule_sess_for_deletion(sp->fcport);
				}
			} else if (logit) {
				ql_log(ql_log_info, vha, 0x503f,
				    "%s IOCB Done hdl=%x comp_status=0x%x\n",
				    type, sp->handle, comp_status);
				ql_log(ql_log_info, vha, 0x503f,
				    "subcode 1=0x%x subcode 2=0x%x bytes=0x%x %02x%02x%02x -> %02x%02x%02x\n",
				    fw_status[1], fw_status[2],
				    le32_to_cpu(((struct els_sts_entry_24xx *)
				    pkt)->total_byte_count),
				    e->s_id[0], e->s_id[2], e->s_id[1],
				    e->d_id[2], e->d_id[1], e->d_id[0]);
			}
		}
		goto els_ct_done;
	}

	/* return FC_CTELS_STATUS_OK and leave the decoding of the ELS/CT
	 * fc payload  to the caller
	 */
	bsg_job = sp->u.bsg_job;
	bsg_reply = bsg_job->reply;
	bsg_reply->reply_data.ctels_reply.status = FC_CTELS_STATUS_OK;
	bsg_job->reply_len = sizeof(struct fc_bsg_reply) + sizeof(fw_status);

	if (comp_status != CS_COMPLETE) {
		if (comp_status == CS_DATA_UNDERRUN) {
			res = DID_OK << 16;
			bsg_reply->reply_payload_rcv_len =
			    le16_to_cpu(((struct els_sts_entry_24xx *)pkt)->total_byte_count);

			ql_dbg(ql_dbg_user, vha, 0x503f,
			    "ELS-CT pass-through-%s error hdl=%x comp_status-status=0x%x "
			    "error subcode 1=0x%x error subcode 2=0x%x total_byte = 0x%x.\n",
			    type, sp->handle, comp_status, fw_status[1], fw_status[2],
			    le16_to_cpu(((struct els_sts_entry_24xx *)
				pkt)->total_byte_count));
		} else {
			ql_dbg(ql_dbg_user, vha, 0x5040,
			    "ELS-CT pass-through-%s error hdl=%x comp_status-status=0x%x "
			    "error subcode 1=0x%x error subcode 2=0x%x.\n",
			    type, sp->handle, comp_status,
			    le16_to_cpu(((struct els_sts_entry_24xx *)
				pkt)->error_subcode_1),
			    le16_to_cpu(((struct els_sts_entry_24xx *)
				    pkt)->error_subcode_2));
			res = DID_ERROR << 16;
			bsg_reply->reply_payload_rcv_len = 0;
		}
		memcpy(bsg_job->reply + sizeof(struct fc_bsg_reply),
		       fw_status, sizeof(fw_status));
		ql_dump_buffer(ql_dbg_user + ql_dbg_buffer, vha, 0x5056,
		    pkt, sizeof(*pkt));
	}
	else {
		res =  DID_OK << 16;
		bsg_reply->reply_payload_rcv_len = bsg_job->reply_payload.payload_len;
		bsg_job->reply_len = 0;
	}
els_ct_done:

	sp->done(sp, res);
}

static void
qla24xx_logio_entry(scsi_qla_host_t *vha, struct req_que *req,
    struct logio_entry_24xx *logio)
{
	const char func[] = "LOGIO-IOCB";
	const char *type;
	fc_port_t *fcport;
	srb_t *sp;
	struct srb_iocb *lio;
	uint16_t *data;
	uint32_t iop[2];
	int logit = 1;

	sp = qla2x00_get_sp_from_handle(vha, func, req, logio);
	if (!sp)
		return;

	lio = &sp->u.iocb_cmd;
	type = sp->name;
	fcport = sp->fcport;
	data = lio->u.logio.data;

	data[0] = MBS_COMMAND_ERROR;
	data[1] = lio->u.logio.flags & SRB_LOGIN_RETRIED ?
		QLA_LOGIO_LOGIN_RETRIED : 0;
	if (logio->entry_status) {
		ql_log(ql_log_warn, fcport->vha, 0x5034,
		    "Async-%s error entry - %8phC hdl=%x"
		    "portid=%02x%02x%02x entry-status=%x.\n",
		    type, fcport->port_name, sp->handle, fcport->d_id.b.domain,
		    fcport->d_id.b.area, fcport->d_id.b.al_pa,
		    logio->entry_status);
		ql_dump_buffer(ql_dbg_async + ql_dbg_buffer, vha, 0x504d,
		    logio, sizeof(*logio));

		goto logio_done;
	}

	if (le16_to_cpu(logio->comp_status) == CS_COMPLETE) {
		ql_dbg(ql_dbg_async, sp->vha, 0x5036, "Async-%s complete: "
		    "handle=%x pid=%06x wwpn=%8phC iop0=%x\n",
		    type, sp->handle, fcport->d_id.b24, fcport->port_name,
		    le32_to_cpu(logio->io_parameter[0]));

		vha->hw->exch_starvation = 0;
		data[0] = MBS_COMMAND_COMPLETE;

		if (sp->type == SRB_PRLI_CMD) {
			lio->u.logio.iop[0] =
			    le32_to_cpu(logio->io_parameter[0]);
			lio->u.logio.iop[1] =
			    le32_to_cpu(logio->io_parameter[1]);
			goto logio_done;
		}

		if (sp->type != SRB_LOGIN_CMD)
			goto logio_done;

                ql_dump_buffer(ql_dbg_edif + ql_dbg_verbose, sp->vha, 0x5055,
                    logio, 64);

		lio->u.logio.iop[1] = le32_to_cpu(logio->io_parameter[5]);
		if (le32_to_cpu(logio->io_parameter[5]) & LIO_COMM_FEAT_FCSP)
			fcport->flags |= FCF_FCSP_DEVICE;

		iop[0] = le32_to_cpu(logio->io_parameter[0]);
		if (iop[0] & BIT_4) {
			fcport->port_type = FCT_TARGET;
			if (iop[0] & BIT_8)
				fcport->flags |= FCF_FCP2_DEVICE;
		} else if (iop[0] & BIT_5)
			fcport->port_type = FCT_INITIATOR;

		if (iop[0] & BIT_7)
			fcport->flags |= FCF_CONF_COMP_SUPPORTED;

		if (logio->io_parameter[7] || logio->io_parameter[8])
			fcport->supported_classes |= FC_COS_CLASS2;
		if (logio->io_parameter[9] || logio->io_parameter[10])
			fcport->supported_classes |= FC_COS_CLASS3;

		goto logio_done;
	}

	iop[0] = le32_to_cpu(logio->io_parameter[0]);
	iop[1] = le32_to_cpu(logio->io_parameter[1]);
	lio->u.logio.iop[0] = iop[0];
	lio->u.logio.iop[1] = iop[1];
	switch (iop[0]) {
	case LSC_SCODE_PORTID_USED:
		data[0] = MBS_PORT_ID_USED;
		data[1] = LSW(iop[1]);
		logit = 0;
		break;
	case LSC_SCODE_NPORT_USED:
		data[0] = MBS_LOOP_ID_USED;
		logit = 0;
		break;
	case LSC_SCODE_CMD_FAILED:
		if (iop[1] == 0x0606) {
			/*
			 * PLOGI/PRLI Completed. We must have Recv PLOGI/PRLI,
			 * Target side acked.
			 */
			data[0] = MBS_COMMAND_COMPLETE;
			goto logio_done;
		}
		data[0] = MBS_COMMAND_ERROR;
		break;
	case LSC_SCODE_NOXCB:
		vha->hw->exch_starvation++;
		if (vha->hw->exch_starvation > 5) {
			ql_log(ql_log_warn, vha, 0xd046,
			    "Exchange starvation. Resetting RISC\n");

			vha->hw->exch_starvation = 0;

			if (IS_P3P_TYPE(vha->hw))
				set_bit(FCOE_CTX_RESET_NEEDED, &vha->dpc_flags);
			else
				set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
			qla2xxx_wake_dpc(vha);
		}
		fallthrough;
	default:
		data[0] = MBS_COMMAND_ERROR;
		break;
	}

	if (logit)
		ql_log(ql_log_warn, sp->vha, 0x5037, "Async-%s failed: "
		    "handle=%x pid=%06x wwpn=%8phC comp_status=%x iop0=%x iop1=%x\n",
		    type, sp->handle, fcport->d_id.b24, fcport->port_name,
		    le16_to_cpu(logio->comp_status),
		    le32_to_cpu(logio->io_parameter[0]),
		    le32_to_cpu(logio->io_parameter[1]));
	else
		ql_dbg(ql_dbg_disc, sp->vha, 0x5037, "Async-%s failed: "
		    "handle=%x pid=%06x wwpn=%8phC comp_status=%x iop0=%x iop1=%x\n",
		    type, sp->handle, fcport->d_id.b24, fcport->port_name,
		    le16_to_cpu(logio->comp_status),
		    le32_to_cpu(logio->io_parameter[0]),
		    le32_to_cpu(logio->io_parameter[1]));

logio_done:
	sp->done(sp, 0);
}

static void
qla24xx_tm_iocb_entry(scsi_qla_host_t *vha, struct req_que *req, void *tsk)
{
	const char func[] = "TMF-IOCB";
	const char *type;
	fc_port_t *fcport;
	srb_t *sp;
	struct srb_iocb *iocb;
	struct sts_entry_24xx *sts = (struct sts_entry_24xx *)tsk;
	u16 comp_status;

	sp = qla2x00_get_sp_from_handle(vha, func, req, tsk);
	if (!sp)
		return;

	comp_status = le16_to_cpu(sts->comp_status);
	iocb = &sp->u.iocb_cmd;
	type = sp->name;
	fcport = sp->fcport;
	iocb->u.tmf.data = QLA_SUCCESS;

	if (sts->entry_status) {
		ql_log(ql_log_warn, fcport->vha, 0x5038,
		    "Async-%s error - hdl=%x entry-status(%x).\n",
		    type, sp->handle, sts->entry_status);
		iocb->u.tmf.data = QLA_FUNCTION_FAILED;
	} else if (sts->comp_status != cpu_to_le16(CS_COMPLETE)) {
		ql_log(ql_log_warn, fcport->vha, 0x5039,
		    "Async-%s error - hdl=%x completion status(%x).\n",
		    type, sp->handle, comp_status);
		iocb->u.tmf.data = QLA_FUNCTION_FAILED;
	} else if ((le16_to_cpu(sts->scsi_status) &
	    SS_RESPONSE_INFO_LEN_VALID)) {
		host_to_fcp_swap(sts->data, sizeof(sts->data));

		if (le32_to_cpu(sts->rsp_data_len) < 4) {
			ql_log(ql_log_warn, fcport->vha, 0x503b,
			    "Async-%s error - hdl=%x not enough response(%d).\n",
			    type, sp->handle, sts->rsp_data_len);
		} else if (sts->data[3]) {
			ql_log(ql_log_warn, fcport->vha, 0x503c,
			    "Async-%s error - hdl=%x response(%x).\n",
			    type, sp->handle, sts->data[3]);
			iocb->u.tmf.data = QLA_FUNCTION_FAILED;
		}
	}

	switch (comp_status) {
	case CS_PORT_LOGGED_OUT:
	case CS_PORT_CONFIG_CHG:
	case CS_PORT_BUSY:
	case CS_INCOMPLETE:
	case CS_PORT_UNAVAILABLE:
	case CS_TIMEOUT:
	case CS_RESET:
		if (atomic_read(&fcport->state) == FCS_ONLINE) {
			ql_dbg(ql_dbg_disc, fcport->vha, 0x3021,
			    "-Port to be marked lost on fcport=%02x%02x%02x, current "
			    "port state= %s comp_status %x.\n", fcport->d_id.b.domain,
			    fcport->d_id.b.area, fcport->d_id.b.al_pa,
			    port_state_str[FCS_ONLINE],
			    comp_status);

			qlt_schedule_sess_for_deletion(fcport);
		}
		break;

	default:
		break;
	}

	if (iocb->u.tmf.data != QLA_SUCCESS)
		ql_dump_buffer(ql_dbg_async + ql_dbg_buffer, sp->vha, 0x5055,
		    sts, sizeof(*sts));

	sp->done(sp, 0);
}

static void qla24xx_nvme_iocb_entry(scsi_qla_host_t *vha, struct req_que *req,
    void *tsk, srb_t *sp)
{
	fc_port_t *fcport;
	struct srb_iocb *iocb;
	struct sts_entry_24xx *sts = (struct sts_entry_24xx *)tsk;
	uint16_t        state_flags;
	struct nvmefc_fcp_req *fd;
	uint16_t        ret = QLA_SUCCESS;
	uint16_t	comp_status = le16_to_cpu(sts->comp_status);
	int		logit = 0;

	iocb = &sp->u.iocb_cmd;
	fcport = sp->fcport;
	iocb->u.nvme.comp_status = comp_status;
	state_flags  = le16_to_cpu(sts->state_flags);
	fd = iocb->u.nvme.desc;


	if (unlikely(iocb->u.nvme.aen_op))
		atomic_dec(&sp->vha->hw->nvme_active_aen_cnt);
	else
		sp->qpair->cmd_completion_cnt++;

	if (unlikely(comp_status != CS_COMPLETE))
		logit = 1;

	fd->transferred_length = fd->payload_length -
	    le32_to_cpu(sts->residual_len);

	/*
	 * State flags: Bit 6 and 0.
	 * If 0 is set, we don't care about 6.
	 * both cases resp was dma'd to host buffer
	 * if both are 0, that is good path case.
	 * if six is set and 0 is clear, we need to
	 * copy resp data from status iocb to resp buffer.
	 */
	if (!(state_flags & (SF_FCP_RSP_DMA | SF_NVME_ERSP))) {
		iocb->u.nvme.rsp_pyld_len = 0;
	} else if ((state_flags & (SF_FCP_RSP_DMA | SF_NVME_ERSP)) ==
			(SF_FCP_RSP_DMA | SF_NVME_ERSP)) {
		/* Response already DMA'd to fd->rspaddr. */
		iocb->u.nvme.rsp_pyld_len = le16_to_cpu(sts->nvme_rsp_pyld_len);
	} else if ((state_flags & SF_FCP_RSP_DMA)) {
		/*
		 * Non-zero value in first 12 bytes of NVMe_RSP IU, treat this
		 * as an error.
		 */
		iocb->u.nvme.rsp_pyld_len = 0;
		fd->transferred_length = 0;
		ql_dbg(ql_dbg_io, fcport->vha, 0x307a,
			"Unexpected values in NVMe_RSP IU.\n");
		logit = 1;
	} else if (state_flags & SF_NVME_ERSP) {
		uint32_t *inbuf, *outbuf;
		uint16_t iter;

		inbuf = (uint32_t *)&sts->nvme_ersp_data;
		outbuf = (uint32_t *)fd->rspaddr;
		iocb->u.nvme.rsp_pyld_len = le16_to_cpu(sts->nvme_rsp_pyld_len);
		if (unlikely(iocb->u.nvme.rsp_pyld_len >
				sizeof(struct nvme_fc_ersp_iu))) {
			if (ql_mask_match(ql_dbg_io)) {
				WARN_ONCE(1, "%8phC: Unexpected response payload length %u.\n",
					fcport->port_name,
					iocb->u.nvme.rsp_pyld_len);
				ql_log(ql_log_warn, fcport->vha, 0x5100,
					"%8phC: Unexpected response payload length %u.\n",
					fcport->port_name,
					iocb->u.nvme.rsp_pyld_len);
				logit = 1;
			}
			iocb->u.nvme.rsp_pyld_len =
				sizeof(struct nvme_fc_ersp_iu);
		}
		iter = iocb->u.nvme.rsp_pyld_len >> 2;
		for (; iter; iter--)
			*outbuf++ = swab32(*inbuf++);
	}

	if (state_flags & SF_NVME_ERSP) {
		struct nvme_fc_ersp_iu *rsp_iu = fd->rspaddr;
		u32 tgt_xfer_len;

		tgt_xfer_len = be32_to_cpu(rsp_iu->xfrd_len);
		if (fd->transferred_length != tgt_xfer_len) {
			ql_log(ql_log_warn, fcport->vha, 0x3079,
				"Dropped frame(s) detected (sent/rcvd=%u/%u).\n",
				tgt_xfer_len, fd->transferred_length);
			logit = 1;
		} else if (comp_status == CS_DATA_UNDERRUN) {
			/*
			 * Do not log if this is just an underflow and there
			 * is no data loss.
			 */
			logit = 0;
		}
	}

	if (unlikely(logit))
		ql_dbg(ql_dbg_io, fcport->vha, 0x5060,
		   "NVME-%s ERR Handling - hdl=%x status(%x) tr_len:%x resid=%x  ox_id=%x\n",
		   sp->name, sp->handle, comp_status,
		   fd->transferred_length, le32_to_cpu(sts->residual_len),
		   sts->ox_id);

	/*
	 * If transport error then Failure (HBA rejects request)
	 * otherwise transport will handle.
	 */
	switch (comp_status) {
	case CS_COMPLETE:
		break;

	case CS_RESET:
	case CS_PORT_UNAVAILABLE:
	case CS_PORT_LOGGED_OUT:
		fcport->nvme_flag |= NVME_FLAG_RESETTING;
		if (atomic_read(&fcport->state) == FCS_ONLINE) {
			ql_dbg(ql_dbg_disc, fcport->vha, 0x3021,
				"Port to be marked lost on fcport=%06x, current "
				"port state= %s comp_status %x.\n", fcport->d_id.b24,
				port_state_str[FCS_ONLINE],
				comp_status);

			qlt_schedule_sess_for_deletion(fcport);
		}
		fallthrough;
	case CS_ABORTED:
	case CS_PORT_BUSY:
		fd->transferred_length = 0;
		iocb->u.nvme.rsp_pyld_len = 0;
		ret = QLA_ABORTED;
		break;
	case CS_DATA_UNDERRUN:
		break;
	default:
		ret = QLA_FUNCTION_FAILED;
		break;
	}
	sp->done(sp, ret);
}

static void qla_ctrlvp_completed(scsi_qla_host_t *vha, struct req_que *req,
    struct vp_ctrl_entry_24xx *vce)
{
	const char func[] = "CTRLVP-IOCB";
	srb_t *sp;
	int rval = QLA_SUCCESS;

	sp = qla2x00_get_sp_from_handle(vha, func, req, vce);
	if (!sp)
		return;

	if (vce->entry_status != 0) {
		ql_dbg(ql_dbg_vport, vha, 0x10c4,
		    "%s: Failed to complete IOCB -- error status (%x)\n",
		    sp->name, vce->entry_status);
		rval = QLA_FUNCTION_FAILED;
	} else if (vce->comp_status != cpu_to_le16(CS_COMPLETE)) {
		ql_dbg(ql_dbg_vport, vha, 0x10c5,
		    "%s: Failed to complete IOCB -- completion status (%x) vpidx %x\n",
		    sp->name, le16_to_cpu(vce->comp_status),
		    le16_to_cpu(vce->vp_idx_failed));
		rval = QLA_FUNCTION_FAILED;
	} else {
		ql_dbg(ql_dbg_vport, vha, 0x10c6,
		    "Done %s.\n", __func__);
	}

	sp->rc = rval;
	sp->done(sp, rval);
}

/* Process a single response queue entry. */
static void qla2x00_process_response_entry(struct scsi_qla_host *vha,
					   struct rsp_que *rsp,
					   sts_entry_t *pkt)
{
	sts21_entry_t *sts21_entry;
	sts22_entry_t *sts22_entry;
	uint16_t handle_cnt;
	uint16_t cnt;

	switch (pkt->entry_type) {
	case STATUS_TYPE:
		qla2x00_status_entry(vha, rsp, pkt);
		break;
	case STATUS_TYPE_21:
		sts21_entry = (sts21_entry_t *)pkt;
		handle_cnt = sts21_entry->handle_count;
		for (cnt = 0; cnt < handle_cnt; cnt++)
			qla2x00_process_completed_request(vha, rsp->req,
						sts21_entry->handle[cnt]);
		break;
	case STATUS_TYPE_22:
		sts22_entry = (sts22_entry_t *)pkt;
		handle_cnt = sts22_entry->handle_count;
		for (cnt = 0; cnt < handle_cnt; cnt++)
			qla2x00_process_completed_request(vha, rsp->req,
						sts22_entry->handle[cnt]);
		break;
	case STATUS_CONT_TYPE:
		qla2x00_status_cont_entry(rsp, (sts_cont_entry_t *)pkt);
		break;
	case MBX_IOCB_TYPE:
		qla2x00_mbx_iocb_entry(vha, rsp->req, (struct mbx_entry *)pkt);
		break;
	case CT_IOCB_TYPE:
		qla2x00_ct_entry(vha, rsp->req, pkt, CT_IOCB_TYPE);
		break;
	default:
		/* Type Not Supported. */
		ql_log(ql_log_warn, vha, 0x504a,
		       "Received unknown response pkt type %x entry status=%x.\n",
		       pkt->entry_type, pkt->entry_status);
		break;
	}
}

/**
 * qla2x00_process_response_queue() - Process response queue entries.
 * @rsp: response queue
 */
void
qla2x00_process_response_queue(struct rsp_que *rsp)
{
	struct scsi_qla_host *vha;
	struct qla_hw_data *ha = rsp->hw;
	struct device_reg_2xxx __iomem *reg = &ha->iobase->isp;
	sts_entry_t	*pkt;

	vha = pci_get_drvdata(ha->pdev);

	if (!vha->flags.online)
		return;

	while (rsp->ring_ptr->signature != RESPONSE_PROCESSED) {
		pkt = (sts_entry_t *)rsp->ring_ptr;

		rsp->ring_index++;
		if (rsp->ring_index == rsp->length) {
			rsp->ring_index = 0;
			rsp->ring_ptr = rsp->ring;
		} else {
			rsp->ring_ptr++;
		}

		if (pkt->entry_status != 0) {
			qla2x00_error_entry(vha, rsp, pkt);
			((response_t *)pkt)->signature = RESPONSE_PROCESSED;
			wmb();
			continue;
		}

		qla2x00_process_response_entry(vha, rsp, pkt);
		((response_t *)pkt)->signature = RESPONSE_PROCESSED;
		wmb();
	}

	/* Adjust ring index */
	WRT_REG_WORD(ISP_RSP_Q_OUT(ha, reg), rsp->ring_index);
}

static inline void
qla2x00_handle_sense(srb_t *sp, uint8_t *sense_data, uint32_t par_sense_len,
		     uint32_t sense_len, struct rsp_que *rsp, int res)
{
	struct scsi_qla_host *vha = sp->vha;
	struct scsi_cmnd *cp = GET_CMD_SP(sp);
	uint32_t track_sense_len;

	if (sense_len >= SCSI_SENSE_BUFFERSIZE)
		sense_len = SCSI_SENSE_BUFFERSIZE;

	SET_CMD_SENSE_LEN(sp, sense_len);
	SET_CMD_SENSE_PTR(sp, cp->sense_buffer);
	track_sense_len = sense_len;

	if (sense_len > par_sense_len)
		sense_len = par_sense_len;

	memcpy(cp->sense_buffer, sense_data, sense_len);

	SET_CMD_SENSE_PTR(sp, cp->sense_buffer + sense_len);
	track_sense_len -= sense_len;
	SET_CMD_SENSE_LEN(sp, track_sense_len);

	if (track_sense_len != 0) {
		rsp->status_srb = sp;
		cp->result = res;
	}

	if (sense_len) {
		ql_dbg(ql_dbg_io + ql_dbg_buffer, vha, 0x301c,
		    "Check condition Sense data, nexus%ld:%d:%llu cmd=%px.\n",
		    sp->vha->host_no, cp->device->id, lun_cast(cp->device->lun),
		    cp);
		ql_dump_buffer(ql_dbg_io + ql_dbg_buffer, vha, 0x302b,
		    cp->sense_buffer, sense_len);
	}
}

struct scsi_dif_tuple {
	__be16 guard;       /* Checksum */
	__be16 app_tag;         /* APPL identifier */
	__be32 ref_tag;         /* Target LBA or indirect LBA */
};

/*
 * Checks the guard or meta-data for the type of error
 * detected by the HBA. In case of errors, we set the
 * ASC/ASCQ fields in the sense buffer with ILLEGAL_REQUEST
 * to indicate to the kernel that the HBA detected error.
 */
static inline int
qla2x00_handle_dif_error(srb_t *sp, struct sts_entry_24xx *sts24)
{
	struct scsi_qla_host *vha = sp->vha;
	struct scsi_cmnd *cmd = GET_CMD_SP(sp);
	uint8_t		*ap = &sts24->data[12];
	uint8_t		*ep = &sts24->data[20];
	uint32_t	e_ref_tag, a_ref_tag;
	uint16_t	e_app_tag, a_app_tag;
	uint16_t	e_guard, a_guard;

	/*
	 * swab32 of the "data" field in the beginning of qla2x00_status_entry()
	 * would make guard field appear at offset 2
	 */
	a_guard   = le16_to_cpu(*(uint16_t *)(ap + 2));
	a_app_tag = le16_to_cpu(*(uint16_t *)(ap + 0));
	a_ref_tag = le32_to_cpu(*(uint32_t *)(ap + 4));
	e_guard   = le16_to_cpu(*(uint16_t *)(ep + 2));
	e_app_tag = le16_to_cpu(*(uint16_t *)(ep + 0));
	e_ref_tag = le32_to_cpu(*(uint32_t *)(ep + 4));

	ql_dbg(ql_dbg_io, vha, 0x3023,
	    "iocb(s) %px Returned STATUS.\n", sts24);

	ql_dbg(ql_dbg_io, vha, 0x3024,
	    "DIF ERROR in cmd 0x%x lba 0x%llx act ref"
	    " tag=0x%x, exp ref_tag=0x%x, act app tag=0x%x, exp app"
	    " tag=0x%x, act guard=0x%x, exp guard=0x%x.\n",
	    cmd->cmnd[0], (u64)scsi_get_lba(cmd), a_ref_tag, e_ref_tag,
	    a_app_tag, e_app_tag, a_guard, e_guard);

	/*
	 * Ignore sector if:
	 * For type     3: ref & app tag is all 'f's
	 * For type 0,1,2: app tag is all 'f's
	 */
	if ((a_app_tag == QL_T10_PI_APP_ESCAPE) &&
	    ((scsi_get_prot_type(cmd) != SCSI_PROT_DIF_TYPE3) ||
	     (a_ref_tag == QL_T10_PI_REF_ESCAPE))) {
		uint32_t blocks_done, resid;
		sector_t lba_s = scsi_get_lba(cmd);

		/* 2TB boundary case covered automatically with this */
		blocks_done = e_ref_tag - (uint32_t)lba_s + 1;

		resid = scsi_bufflen(cmd) - (blocks_done *
		    cmd->device->sector_size);

		scsi_set_resid(cmd, resid);
		cmd->result = DID_OK << 16;

		/* Update protection tag */
		if (scsi_prot_sg_count(cmd)) {
			uint32_t i, j = 0, k = 0, num_ent;
			struct scatterlist *sg;
			QL_T10_PI_TUPLE *spt;

			/* Patch the corresponding protection tags */
			scsi_for_each_prot_sg(cmd, sg,
			    scsi_prot_sg_count(cmd), i) {
				num_ent = sg_dma_len(sg) / 8;
				if (k + num_ent < blocks_done) {
					k += num_ent;
					continue;
				}
				j = blocks_done - k - 1;
				k = blocks_done;
				break;
			}

			if (k != blocks_done) {
				ql_log(ql_log_warn, vha, 0x302f,
				    "unexpected tag values tag:lba=%x:%llx)\n",
				    e_ref_tag, (unsigned long long)lba_s);
				return 1;
			}

			spt = page_address(sg_page(sg)) + sg->offset;
			spt += j;

			spt->app_tag = QL_T10_PI_APP_ESCAPE;
			if (scsi_get_prot_type(cmd) == SCSI_PROT_DIF_TYPE3)
				spt->ref_tag = QL_T10_PI_REF_ESCAPE;
		}

		return 0;
	}

	/* check guard */
	if (e_guard != a_guard) {
		scsi_build_sense_buffer(1, cmd->sense_buffer, ILLEGAL_REQUEST,
		    0x10, 0x1);
		set_driver_byte(cmd, DRIVER_SENSE);
		set_host_byte(cmd, DID_ABORT);
		cmd->result |= SAM_STAT_CHECK_CONDITION;
		return 1;
	}

	/* check ref tag */
	if (e_ref_tag != a_ref_tag) {
		scsi_build_sense_buffer(1, cmd->sense_buffer, ILLEGAL_REQUEST,
		    0x10, 0x3);
		set_driver_byte(cmd, DRIVER_SENSE);
		set_host_byte(cmd, DID_ABORT);
		cmd->result |= SAM_STAT_CHECK_CONDITION;
		return 1;
	}

	/* check appl tag */
	if (e_app_tag != a_app_tag) {
		scsi_build_sense_buffer(1, cmd->sense_buffer, ILLEGAL_REQUEST,
		    0x10, 0x2);
		set_driver_byte(cmd, DRIVER_SENSE);
		set_host_byte(cmd, DID_ABORT);
		cmd->result |= SAM_STAT_CHECK_CONDITION;
		return 1;
	}
	return 1;
}

static void
qla25xx_process_bidir_status_iocb(scsi_qla_host_t *vha, void *pkt,
				  struct req_que *req, uint32_t index)
{
	struct qla_hw_data *ha = vha->hw;
	srb_t *sp;
	uint16_t	comp_status;
	uint16_t	scsi_status;
	uint16_t thread_id;
	uint32_t rval = EXT_STATUS_OK;
	bsg_job_t *bsg_job = NULL;
	struct fc_bsg_request *bsg_request;
	struct fc_bsg_reply *bsg_reply;
	sts_entry_t *sts = pkt;
	struct sts_entry_24xx *sts24 = pkt;

	/* Validate handle. */
	if (index >= req->num_outstanding_cmds) {
		ql_log(ql_log_warn, vha, 0x70af,
		    "Invalid SCSI completion handle 0x%x.\n", index);
		set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		return;
	}

	sp = req->outstanding_cmds[index];
	if (!sp) {
		ql_log(ql_log_warn, vha, 0x70b0,
		    "Req:%d: Invalid ISP SCSI completion handle(0x%x)\n",
		    req->id, index);

		set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		return;
	}

	/* Free outstanding command slot. */
	req->outstanding_cmds[index] = NULL;
	bsg_job = sp->u.bsg_job;
	bsg_request = bsg_job->request;
	bsg_reply = bsg_job->reply;

	if (IS_FWI2_CAPABLE(ha)) {
		comp_status = le16_to_cpu(sts24->comp_status);
		scsi_status = le16_to_cpu(sts24->scsi_status) & SS_MASK;
	} else {
		comp_status = le16_to_cpu(sts->comp_status);
		scsi_status = le16_to_cpu(sts->scsi_status) & SS_MASK;
	}

	thread_id = bsg_request->rqst_data.h_vendor.vendor_cmd[1];
	switch (comp_status) {
	case CS_COMPLETE:
		if (scsi_status == 0) {
			bsg_reply->reply_payload_rcv_len =
					bsg_job->reply_payload.payload_len;
			vha->qla_stats.input_bytes +=
				bsg_reply->reply_payload_rcv_len;
			vha->qla_stats.input_requests++;
			rval = EXT_STATUS_OK;
		}
		goto done;

	case CS_DATA_OVERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b1,
		    "Command completed with data overrun thread_id=%d\n",
		    thread_id);
		rval = EXT_STATUS_DATA_OVERRUN;
		break;

	case CS_DATA_UNDERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b2,
		    "Command completed with data underrun thread_id=%d\n",
		    thread_id);
		rval = EXT_STATUS_DATA_UNDERRUN;
		break;
	case CS_BIDIR_RD_OVERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b3,
		    "Command completed with read data overrun thread_id=%d\n",
		    thread_id);
		rval = EXT_STATUS_DATA_OVERRUN;
		break;

	case CS_BIDIR_RD_WR_OVERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b4,
		    "Command completed with read and write data overrun "
		    "thread_id=%d\n", thread_id);
		rval = EXT_STATUS_DATA_OVERRUN;
		break;

	case CS_BIDIR_RD_OVERRUN_WR_UNDERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b5,
		    "Command completed with read data over and write data "
		    "underrun thread_id=%d\n", thread_id);
		rval = EXT_STATUS_DATA_OVERRUN;
		break;

	case CS_BIDIR_RD_UNDERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b6,
		    "Command completed with read data underrun "
		    "thread_id=%d\n", thread_id);
		rval = EXT_STATUS_DATA_UNDERRUN;
		break;

	case CS_BIDIR_RD_UNDERRUN_WR_OVERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b7,
		    "Command completed with read data under and write data "
		    "overrun thread_id=%d\n", thread_id);
		rval = EXT_STATUS_DATA_UNDERRUN;
		break;

	case CS_BIDIR_RD_WR_UNDERRUN:
		ql_dbg(ql_dbg_user, vha, 0x70b8,
		    "Command completed with read and write data underrun "
		    "thread_id=%d\n", thread_id);
		rval = EXT_STATUS_DATA_UNDERRUN;
		break;

	case CS_BIDIR_DMA:
		ql_dbg(ql_dbg_user, vha, 0x70b9,
		    "Command completed with data DMA error thread_id=%d\n",
		    thread_id);
		rval = EXT_STATUS_DMA_ERR;
		break;

	case CS_TIMEOUT:
		ql_dbg(ql_dbg_user, vha, 0x70ba,
		    "Command completed with timeout thread_id=%d\n",
		    thread_id);
		rval = EXT_STATUS_TIMEOUT;
		break;
	default:
		ql_dbg(ql_dbg_user, vha, 0x70bb,
		    "Command completed with completion status=0x%x "
		    "thread_id=%d\n", comp_status, thread_id);
		rval = EXT_STATUS_ERR;
		break;
	}
	bsg_reply->reply_payload_rcv_len = 0;

done:
	/* Return the vendor specific reply to API */
	bsg_reply->reply_data.vendor_reply.vendor_rsp[0] = rval;
	bsg_job->reply_len = sizeof(struct fc_bsg_reply);
	/* Always return DID_OK, bsg will send the vendor specific response
	 * in this case only */
	sp->done(sp, DID_OK << 16);

}

/**
 * qla2x00_status_entry() - Process a Status IOCB entry.
 * @vha: SCSI driver HA context
 * @rsp: response queue
 * @pkt: Entry pointer
 */
static void
qla2x00_status_entry(scsi_qla_host_t *vha, struct rsp_que *rsp, void *pkt)
{
	srb_t		*sp;
	fc_port_t	*fcport;
	struct scsi_cmnd *cp;
	sts_entry_t *sts = pkt;
	struct sts_entry_24xx *sts24 = pkt;
	uint16_t	comp_status;
	uint16_t	scsi_status;
	uint16_t	ox_id;
	uint8_t		lscsi_status;
	int32_t		resid;
	uint32_t sense_len, par_sense_len, rsp_info_len, resid_len,
	    fw_resid_len;
	uint8_t		*rsp_info, *sense_data;
	struct qla_hw_data *ha = vha->hw;
	uint32_t handle;
	uint16_t que;
	struct req_que *req;
	int logit = 1;
	int res = 0;
	uint16_t state_flags = 0;
	uint16_t retry_delay = 0;

	if (IS_FWI2_CAPABLE(ha)) {
		comp_status = le16_to_cpu(sts24->comp_status);
		scsi_status = le16_to_cpu(sts24->scsi_status) & SS_MASK;
		state_flags = le16_to_cpu(sts24->state_flags);
	} else {
		comp_status = le16_to_cpu(sts->comp_status);
		scsi_status = le16_to_cpu(sts->scsi_status) & SS_MASK;
	}
	handle = (uint32_t) LSW(sts->handle);
	que = MSW(sts->handle);
	req = ha->req_q_map[que];

	/* Check for invalid queue pointer */
	if (req == NULL ||
	    que >= find_first_zero_bit(ha->req_qid_map, ha->max_req_queues)) {
		ql_dbg(ql_dbg_io, vha, 0x3059,
		    "Invalid status handle (0x%x): Bad req pointer. req=%px, "
		    "que=%u.\n", sts->handle, req, que);
		if (is_debug(QDBG_FW_DUMP)) {
			ha->isp_ops->fw_dump(vha, 1);
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
		}

		BUG_ON(is_debug(QDBG_CRASH_ON_ERR));
		return;
	}

	/* Validate handle. */
	if (handle < req->num_outstanding_cmds) {
		sp = req->outstanding_cmds[handle];
		if (!sp) {
			ql_dbg(ql_dbg_io, vha, 0x3075,
			    "%s(%ld): Already returned command for status handle (0x%x).\n",
			    __func__, vha->host_no, sts->handle);
			return;
		}
	} else {
		ql_dbg(ql_dbg_io, vha, 0x3017,
		    "Invalid status handle, out of range (0x%x).\n",
		    sts->handle);

		if (is_debug(QDBG_FW_DUMP))
			ha->isp_ops->fw_dump(vha, 1);

		BUG_ON(is_debug(QDBG_CRASH_ON_ERR));

		if (!test_bit(ABORT_ISP_ACTIVE, &vha->dpc_flags)) {
			if (IS_P3P_TYPE(ha))
				set_bit(FCOE_CTX_RESET_NEEDED, &vha->dpc_flags);
			else
				set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
			qla2xxx_wake_dpc(vha);
		}
		return;
	}

	ql_srb_trace_ext(ql_dbg_io, vha, sp->fcport,
			"sp=%px handle=0x%x type=%d done=%ps",
			sp, sp->handle, sp->type, sp->done);

#ifdef QLA2XXX_LATENCY_MEASURE
	if (sp->type == SRB_SCSI_CMD || sp->type == SRB_NVME_CMD)
		ktime_get_real_ts64(&sp->cmd_from_rsp_q);
#endif
	qla_put_fw_resources(sp->qpair, &sp->iores);

	if (sp->abort)
		sp->aborted = 1;
	else
		sp->completed = 1;

	if (sp->cmd_type != TYPE_SRB) {
		req->outstanding_cmds[handle] = NULL;
		ql_dbg(ql_dbg_io, vha, 0x3015,
		    "Unknown sp->cmd_type %x %px).\n",
		    sp->cmd_type, sp);
		return;
	}

	/* NVME completion. */
	if (sp->type == SRB_NVME_CMD) {
		req->outstanding_cmds[handle] = NULL;
		qla24xx_nvme_iocb_entry(vha, req, pkt, sp);
		return;
	}

	if (unlikely((state_flags & BIT_1) && (sp->type == SRB_BIDI_CMD))) {
		qla25xx_process_bidir_status_iocb(vha, pkt, req, handle);
		return;
	}

	/* Task Management completion. */
	if (sp->type == SRB_TM_CMD) {
		qla24xx_tm_iocb_entry(vha, req, pkt);
		return;
	}

	/* Fast path completion. */
	qla_chk_edif_rx_sa_delete_pending(vha, sp, sts24);
	sp->qpair->cmd_completion_cnt++;

	if (comp_status == CS_COMPLETE && scsi_status == 0) {
		qla2x00_process_completed_request(vha, req, handle);

		return;
	}

	req->outstanding_cmds[handle] = NULL;
	cp = GET_CMD_SP(sp);
	if (cp == NULL) {
		ql_dbg(ql_dbg_io, vha, 0x3018,
		    "Command already returned (0x%x/%px).\n",
		    sts->handle, sp);

		return;
	}

	lscsi_status = scsi_status & STATUS_MASK;

	fcport = sp->fcport;

	ox_id = 0;
	sense_len = par_sense_len = rsp_info_len = resid_len =
	    fw_resid_len = 0;
	if (IS_FWI2_CAPABLE(ha)) {
		if (scsi_status & SS_SENSE_LEN_VALID)
			sense_len = le32_to_cpu(sts24->sense_len);
		if (scsi_status & SS_RESPONSE_INFO_LEN_VALID)
			rsp_info_len = le32_to_cpu(sts24->rsp_data_len);
		if (scsi_status & (SS_RESIDUAL_UNDER | SS_RESIDUAL_OVER))
			resid_len = le32_to_cpu(sts24->rsp_residual_count);
		if (comp_status == CS_DATA_UNDERRUN)
			fw_resid_len = le32_to_cpu(sts24->residual_len);
		rsp_info = sts24->data;
		sense_data = sts24->data;
		host_to_fcp_swap(sts24->data, sizeof(sts24->data));
		ox_id = le16_to_cpu(sts24->ox_id);
		par_sense_len = sizeof(sts24->data);
		/* Valid values of the retry delay timer are 0x1-0xffef */
		if (sts24->retry_delay > 0 && sts24->retry_delay < 0xfff1) {
			retry_delay = sts24->retry_delay & 0x3fff;
			ql_dbg(ql_dbg_io, sp->vha, 0x3033,
			    "%s: scope=%#x retry_delay=%#x\n", __func__,
			    sts24->retry_delay >> 14, retry_delay);
		}
	} else {
		if (scsi_status & SS_SENSE_LEN_VALID)
			sense_len = le16_to_cpu(sts->req_sense_length);
		if (scsi_status & SS_RESPONSE_INFO_LEN_VALID)
			rsp_info_len = le16_to_cpu(sts->rsp_info_len);
		resid_len = le32_to_cpu(sts->residual_length);
		rsp_info = sts->rsp_info;
		sense_data = sts->req_sense_data;
		par_sense_len = sizeof(sts->req_sense_data);
	}

	/* Check for any FCP transport errors. */
	if (scsi_status & SS_RESPONSE_INFO_LEN_VALID) {
		/* Sense data lies beyond any FCP RESPONSE data. */
		if (IS_FWI2_CAPABLE(ha)) {
			sense_data += rsp_info_len;
			par_sense_len -= rsp_info_len;
		}
		if (rsp_info_len > 3 && rsp_info[3]) {
			ql_dbg(ql_dbg_io, fcport->vha, 0x3019,
			    "FCP I/O protocol failure (0x%x/0x%x).\n",
			    rsp_info_len, rsp_info[3]);

			res = DID_BUS_BUSY << 16;
			goto out;
		}
	}

	/* Check for overrun. */
	if (IS_FWI2_CAPABLE(ha) && comp_status == CS_COMPLETE &&
	    scsi_status & SS_RESIDUAL_OVER)
		comp_status = CS_DATA_OVERRUN;

	/*
	 * Check retry_delay_timer value if we receive a busy or
	 * queue full.
	 */
	if (lscsi_status == SAM_STAT_TASK_SET_FULL ||
	    lscsi_status == SAM_STAT_BUSY)
		qla2x00_set_retry_delay_timestamp(fcport, retry_delay);

	/*
	 * Based on Host and scsi status generate status code for Linux
	 */
	switch (comp_status) {
	case CS_COMPLETE:
	case CS_QUEUE_FULL:
		if (scsi_status == 0) {
			res = DID_OK << 16;
			break;
		}
		if (scsi_status & (SS_RESIDUAL_UNDER | SS_RESIDUAL_OVER)) {
			resid = resid_len;
			scsi_set_resid(cp, resid);

			if (!lscsi_status &&
			    ((unsigned)(scsi_bufflen(cp) - resid) <
			     cp->underflow)) {
				ql_dbg(ql_dbg_io, fcport->vha, 0x301a,
				    "Mid-layer underflow detected (0x%x of 0x%x bytes).\n",
				    resid, scsi_bufflen(cp));

				res = DID_ERROR << 16;
				break;
			}
		}
		res = DID_OK << 16 | lscsi_status;

		if (lscsi_status == SAM_STAT_TASK_SET_FULL) {
			ql_dbg(ql_dbg_io, fcport->vha, 0x301b,
			    "QUEUE FULL detected.\n");
			break;
		}
		logit = 0;
		if (lscsi_status != SS_CHECK_CONDITION)
			break;

		memset(cp->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
		if (!(scsi_status & SS_SENSE_LEN_VALID))
			break;

		qla2x00_handle_sense(sp, sense_data, par_sense_len, sense_len,
		    rsp, res);
		break;

	case CS_DATA_UNDERRUN:
		/* Use F/W calculated residual length. */
		resid = IS_FWI2_CAPABLE(ha) ? fw_resid_len : resid_len;
		scsi_set_resid(cp, resid);
		if (scsi_status & SS_RESIDUAL_UNDER) {
			if (IS_FWI2_CAPABLE(ha) && fw_resid_len != resid_len) {
				ql_log(ql_log_warn, fcport->vha, 0x301d,
				    "Dropped frame(s) detected (0x%x of 0x%x bytes).\n",
				    resid, scsi_bufflen(cp));

				res = DID_ERROR << 16 | lscsi_status;
				goto check_scsi_status;
			}

			if (!lscsi_status &&
			    ((unsigned)(scsi_bufflen(cp) - resid) <
			    cp->underflow)) {
				ql_dbg(ql_dbg_io, fcport->vha, 0x301e,
				    "Mid-layer underflow detected (0x%x of 0x%x bytes).\n",
				    resid, scsi_bufflen(cp));

				res = DID_ERROR << 16;
				break;
			}
		} else if (lscsi_status != SAM_STAT_TASK_SET_FULL &&
			    lscsi_status != SAM_STAT_BUSY) {
			/*
			 * scsi status of task set and busy are considered to be
			 * task not completed.
			 */

			ql_log(ql_log_warn, fcport->vha, 0x301f,
			    "Dropped frame(s) detected (0x%x of 0x%x bytes).\n",
			    resid, scsi_bufflen(cp));

			vha->interface_err_cnt++;

			res = DID_ERROR << 16 | lscsi_status;
			goto check_scsi_status;
		} else {
			ql_dbg(ql_dbg_io, fcport->vha, 0x3030,
			    "scsi_status: 0x%x, lscsi_status: 0x%x\n",
			    scsi_status, lscsi_status);
		}

		res = DID_OK << 16 | lscsi_status;
		logit = 0;

check_scsi_status:
		/*
		 * Check to see if SCSI Status is non zero. If so report SCSI
		 * Status.
		 */
		if (lscsi_status != 0) {
			if (lscsi_status == SAM_STAT_TASK_SET_FULL) {
				ql_dbg(ql_dbg_io, fcport->vha, 0x3020,
				    "QUEUE FULL detected.\n");
				logit = 1;
				break;
			}
			if (lscsi_status != SS_CHECK_CONDITION)
				break;

			memset(cp->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
			if (!(scsi_status & SS_SENSE_LEN_VALID))
				break;

			qla2x00_handle_sense(sp, sense_data, par_sense_len,
			    sense_len, rsp, res);
		}
		break;

	case CS_PORT_LOGGED_OUT:
	case CS_PORT_CONFIG_CHG:
	case CS_PORT_BUSY:
	case CS_INCOMPLETE:
	case CS_PORT_UNAVAILABLE:
	case CS_TIMEOUT:
	case CS_RESET:
	case CS_EDIF_INV_REQ:

		/*
		 * We are going to have the fc class block the rport
		 * while we try to recover so instruct the mid layer
		 * to requeue until the class decides how to handle this.
		 */
		res = DID_TRANSPORT_DISRUPTED << 16;

		if (comp_status == CS_TIMEOUT) {
			if (IS_FWI2_CAPABLE(ha))
				break;
			else if ((le16_to_cpu(sts->status_flags) &
			    SF_LOGOUT_SENT) == 0)
				break;
		}

		if (atomic_read(&fcport->state) == FCS_ONLINE) {
			ql_dbg(ql_dbg_disc, fcport->vha, 0x3021,
				"Port to be marked lost on fcport=%02x%02x%02x, current "
				"port state= %s comp_status %x.\n", fcport->d_id.b.domain,
				fcport->d_id.b.area, fcport->d_id.b.al_pa,
				port_state_str[FCS_ONLINE],
				comp_status);

			qlt_schedule_sess_for_deletion(fcport);
		}

		break;

	case CS_ABORTED:
		res = DID_RESET << 16;
		break;

	case CS_DIF_ERROR:
		logit = qla2x00_handle_dif_error(sp, sts24);
		res = cp->result;
		break;

	case CS_TRANSPORT:
		res = DID_ERROR << 16;
		vha->hw_err_cnt++;

		if (!IS_PI_SPLIT_DET_CAPABLE(ha))
			break;

		if (state_flags & BIT_4)
			scmd_printk(KERN_WARNING, cp,
			    "Unsupported device '%s' found.\n",
			    cp->device->vendor);
		break;

	case CS_DMA:
		ql_log(ql_log_info, fcport->vha, 0x3022,
		    "CS_DMA error: 0x%x-0x%x (0x%x) nexus=%ld:%d:%llu portid=%06x oxid=0x%x cdb=%10phN len=0x%x rsp_info=0x%x resid=0x%x fw_resid=0x%x sp=%px cp=%px.\n",
		    comp_status, scsi_status, res, vha->host_no,
		    cp->device->id, lun_cast(cp->device->lun), fcport->d_id.b24,
		    ox_id, cp->cmnd, scsi_bufflen(cp), rsp_info_len,
		    resid_len, fw_resid_len, sp, cp);
		ql_dump_buffer(ql_dbg_tgt + ql_dbg_verbose, vha, 0xe0ee,
		    pkt, sizeof(*sts24));
		res = DID_ERROR << 16;
		vha->hw_err_cnt++;
		break;
	default:
		res = DID_ERROR << 16;
		break;
	}

out:
	if (logit)
		ql_dbg(ql_dbg_io, fcport->vha, 0x3022,
		    "FCP command status: 0x%x-0x%x (0x%x) nexus=%ld:%d:%llu "
		    "portid=%02x%02x%02x oxid=0x%x cdb=%10phN len=0x%x "
		    "rsp_info=0x%x resid=0x%x fw_resid=0x%x sp=%px cp=%px.\n",
		    comp_status, scsi_status, res, vha->host_no,
		    cp->device->id, lun_cast(cp->device->lun), fcport->d_id.b.domain,
		    fcport->d_id.b.area, fcport->d_id.b.al_pa, ox_id,
		    cp->cmnd, scsi_bufflen(cp), rsp_info_len,
		    resid_len, fw_resid_len, sp, cp);

	if (rsp->status_srb == NULL)
		sp->done(sp, res);
}

/**
 * qla2x00_status_cont_entry() - Process a Status Continuations entry.
 * @rsp: response queue
 * @pkt: Entry pointer
 *
 * Extended sense data.
 */
static void
qla2x00_status_cont_entry(struct rsp_que *rsp, sts_cont_entry_t *pkt)
{
	uint8_t	sense_sz = 0;
	struct qla_hw_data *ha = rsp->hw;
	struct scsi_qla_host *vha = pci_get_drvdata(ha->pdev);
	srb_t *sp = rsp->status_srb;
	struct scsi_cmnd *cp;
	uint32_t sense_len;
	uint8_t *sense_ptr;

	if (!sp || !GET_CMD_SENSE_LEN(sp)) {
		return;
	}

	sense_len = GET_CMD_SENSE_LEN(sp);
	sense_ptr = GET_CMD_SENSE_PTR(sp);

	cp = GET_CMD_SP(sp);
	if (cp == NULL) {
		ql_log(ql_log_warn, vha, 0x3025,
		    "cmd is NULL: already returned to OS (sp=%px).\n", sp);

		rsp->status_srb = NULL;
		return;
	}

	if (sense_len > sizeof(pkt->data))
		sense_sz = sizeof(pkt->data);
	else
		sense_sz = sense_len;

	/* Move sense data. */
	if (IS_FWI2_CAPABLE(ha))
		host_to_fcp_swap(pkt->data, sizeof(pkt->data));
	memcpy(sense_ptr, pkt->data, sense_sz);
	ql_dump_buffer(ql_dbg_io + ql_dbg_buffer, vha, 0x302c,
		sense_ptr, sense_sz);

	sense_len -= sense_sz;
	sense_ptr += sense_sz;

	SET_CMD_SENSE_PTR(sp, sense_ptr);
	SET_CMD_SENSE_LEN(sp, sense_len);

	/* Place command on done queue. */
	if (sense_len == 0) {
		rsp->status_srb = NULL;
		sp->done(sp, cp->result);
	}
}

/* Debug code - will be removed */
static void dump_flogi_acc_payld(scsi_qla_host_t *vha, struct qla_hw_data *ha)
{
	int i;

	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
			"page_code = 0x%x\n", ha->flogi_acc.page_code);
	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
			"page_len = 0x%x\n", le16_to_cpu(ha->flogi_acc.page_len));

	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"vendor :%s\n", ha->flogi_acc.vendor_code);

	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"er_rdy_desc_len = 0x%x\n", le16_to_cpu(ha->flogi_acc.er_rdy_desc_len));
	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"er_rdy_desc_tag = 0x%x\n", le16_to_cpu(ha->flogi_acc.er_rdy_desc_tag));

	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"rx_vl_desc_len = 0x%x\n", le16_to_cpu(ha->flogi_acc.rx_vl_desc_len));
	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"rx_vl_desc_tag = 0x%x\n", le16_to_cpu(ha->flogi_acc.rx_vl_desc_tag));

	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"num_rx_vl = 0x%x\n", le16_to_cpu(ha->flogi_acc.num_rx_vl));

	for (i = 0; i < 7; i++ ) {
		ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"Pr_H:0x%x, Pr_L:0x%x, N_C:0x%x \n", ha->flogi_acc.rx_vl[i].prio_hi,
		ha->flogi_acc.rx_vl[i].prio_lo, le16_to_cpu(ha->flogi_acc.rx_vl[i].num_credits));
	}

	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"tx_vl_desc_len = 0x%x\n", le16_to_cpu(ha->flogi_acc.tx_vl_desc_len));
	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"tx_vl_desc_tag = 0x%x\n", le16_to_cpu(ha->flogi_acc.tx_vl_desc_tag));

	ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"num_tx_vl = 0x%x\n", le16_to_cpu(ha->flogi_acc.num_tx_vl));

	for (i = 0; i < 7; i++ ) {
		ql_dbg(ql_dbg_scm + ql_dbg_verbose, vha, 0x502a,
		"Pr_H:0x%x, Pr_L:0x%x, N_C:0x%x \n", ha->flogi_acc.tx_vl[i].prio_hi,
		ha->flogi_acc.tx_vl[i].prio_lo, le16_to_cpu(ha->flogi_acc.tx_vl[i].num_credits));
	}
}
/**
 * qla27xx_status_cont_type_1() - Process a Status Continuation type 1 entry.
 * @ha: SCSI driver HA context
 * @pkt: Entry pointer
 *
 */
static void
qla27xx_status_cont_type_1(scsi_qla_host_t *vha, sts_cont_entry_t *pkt)
{
	uint8_t page_code, vendor_id_data[8];
	uint16_t page_length;
	uint16_t er_rdy = 0;
	uint16_t num_rxvl = 0;
	uint16_t num_txvl = 0;
	struct qla_hw_data *ha = vha->hw;
	struct flogi_acc_payld *fl_p = &ha->flogi_acc;

	if (ha->flags.flogi_acc_pl_in_cont_iocb) {
		if (ha->flogi_acc_curr_offset == 0) {
			// FLOGI ACC payload bytes 4-5
			ha->attached_port_bb_credit = *((uint16_t *)pkt + 4);
			// FLOGI ACC payload bytes 10-11
			ha->flogi_acc_common_features = *((uint16_t *)pkt + 7);
		} else if (ha->flogi_acc_curr_offset ==
		    (sizeof(sts_cont_entry_t) - 4)) {
			// FLOGI ACC payload byte offset 60, Class 3
			// service parameters (dword offs 17,bytes off 68)
			ha->flogi_acc_cl3_sp_options = *((uint16_t *)pkt + 7);
		} else if (ha->flogi_acc_curr_offset ==
		    2*(sizeof(sts_cont_entry_t) - 4)) {
			// FLOGI ACC payload byte offset 120, login extension
			// data length (dword off 31,bytes off 124)
			ha->flogi_acc_login_ex_length = *((uint32_t *)pkt + 2);
		} else if (ha->flogi_acc_curr_offset ==
		    4*(sizeof(sts_cont_entry_t) - 4)) {
			if (ha->flogi_acc_login_ex_length > 0) {// There is login extension data
				// FLOGI ACC payload byte offset 240, login extension
				// data (dword off 64, bytes off 256)

				/* Copy the remaining 44 bytes to the flogi_acc structure */
				memcpy(fl_p, ((uint16_t *)pkt + 10), 44);
				/* Swap the vendor fields */
				be32_to_cpus((uint32_t *)&ha->flogi_acc.vendor_code[0]);
				be32_to_cpus((uint32_t *)&ha->flogi_acc.vendor_code[4]);

				dump_flogi_acc_payld(vha, ha);
			}
		} else if (ha->flogi_acc_curr_offset >
			4*(sizeof(sts_cont_entry_t) - 4)) { // Offset > 300, the last segment.
				uint8_t *ptr;

				ptr = (uint8_t  *)&ha->flogi_acc + 44;
				memcpy(ptr, pkt->data, ha->flogi_acc_pld_remaining);

				dump_flogi_acc_payld(vha, ha);
				page_code = ha->flogi_acc.page_code;
				page_length = le16_to_cpu(ha->flogi_acc.page_len);
				/* Check if VL is enabled */
				if ((page_code == 0xf0) && (page_length > 0)) {
					er_rdy = le16_to_cpu(ha->flogi_acc.er_rdy_desc_tag);
					num_txvl = le16_to_cpu(ha->flogi_acc.num_tx_vl);
					num_rxvl = le16_to_cpu(ha->flogi_acc.num_rx_vl);
					memcpy(vendor_id_data, ha->flogi_acc.vendor_code, 8);
					if ((memcmp(vendor_id_data, "CISCO", 5) == 0) &&
						(er_rdy == ER_RDY_DESC_TAG) &&
						(NUM_VLS_IN_RANGE(num_txvl, num_rxvl))) {
						ha->flags.conn_fabric_cisco_er_rdy = 1;
						ha->scm.scm_fabric_connection_flags
							= SCM_FLAG_CISCO_CONNECTED;
						ql_log(ql_log_info, vha, 0x5075,
							"Port: %8phC connected to Cisco Fabric \n", vha->port_name);
						ql_log(ql_log_info, vha, 0x5076,
							"Num. Rx VLs:%d, Num. Tx VLs:%d\n",num_rxvl,num_txvl);
						ql_log(ql_log_info, vha, 0x5076,
							"ER_RDY/VL supported by Switch and HBA \n");
					} else if (memcmp(vendor_id_data, "BROCADE", 7) == 0) {
						ha->flags.conn_fabric_brocade = 1;
						ha->scm.scm_fabric_connection_flags
							= SCM_FLAG_BROCADE_CONNECTED;
						ql_log(ql_log_info, vha, 0x5075,
						    "Port: %8phC connected to Brocade Fabric \n", vha->port_name);
					}
				}
		}

		if (ha->flogi_acc_pld_remaining >
		    (sizeof(sts_cont_entry_t) - 4)) {
			ha->flogi_acc_pld_remaining -=
			    (sizeof(sts_cont_entry_t) - 4);
			ha->flogi_acc_curr_offset +=
			    (sizeof(sts_cont_entry_t) - 4);
		} else {
			ha->flogi_acc_curr_offset +=
			    (sizeof(sts_cont_entry_t) - 4);
			ha->flogi_acc_pld_remaining -=
			    (sizeof(sts_cont_entry_t) - 4);
			if (ha->flogi_acc_pld_remaining == 0) {
				ha->flags.flogi_acc_pl_in_cont_iocb = 0;
			} else {
				ql_dbg(ql_log_warn, vha, 0x5075,
				    "Un-accounted bytes balance, %d\n",
				    ha->flogi_acc_pld_remaining);
				ha->flags.flogi_acc_pl_in_cont_iocb = 0;
				ha->flogi_acc_pld_remaining = 0;
			}
		}
	}
}

/**
 * qla2x00_error_entry() - Process an error entry.
 * @vha: SCSI driver HA context
 * @rsp: response queue
 * @pkt: Entry pointer
 * return : 1=allow further error analysis. 0=no additional error analysis.
 */
static int
qla2x00_error_entry(scsi_qla_host_t *vha, struct rsp_que *rsp, sts_entry_t *pkt)
{
	srb_t *sp;
	struct qla_hw_data *ha = vha->hw;
	const char func[] = "ERROR-IOCB";
	uint16_t que = MSW(pkt->handle);
	struct req_que *req = NULL;
	int res = DID_ERROR << 16;

	ql_dbg(ql_dbg_async, vha, 0x502a,
	    "iocb type %xh with error status %xh, handle %xh, rspq id %d\n",
	    pkt->entry_type, pkt->entry_status, pkt->handle, rsp->id);

	if (que >= ha->max_req_queues || !ha->req_q_map[que])
		goto fatal;

	req = ha->req_q_map[que];

	if (pkt->entry_status & RF_BUSY)
		res = DID_BUS_BUSY << 16;

	if ((pkt->handle & ~QLA_TGT_HANDLE_MASK) == QLA_TGT_SKIP_HANDLE)
		return 0;

	switch (pkt->entry_type) {
	case NOTIFY_ACK_TYPE:
	case STATUS_TYPE:
	case STATUS_CONT_TYPE:
	case LOGINOUT_PORT_IOCB_TYPE:
	case CT_IOCB_TYPE:
	case ELS_IOCB_TYPE:
	case ABORT_IOCB_TYPE:
	case MBX_IOCB_TYPE:
	default:
		sp = qla2x00_get_sp_from_handle(vha, func, req, pkt);
		if (sp) {
			sp->done(sp, res);
			return 0;
		}
		break;

	case SA_UPDATE_IOCB_TYPE:
	case ABTS_RESP_24XX:
	case CTIO_TYPE7:
	case CTIO_CRC2:
		return 1;
	}
fatal:
	ql_log(ql_log_warn, vha, 0x5030,
	    "Error entry - invalid handle/queue (%04x).\n", que);

	if (is_debug(QDBG_FW_DUMP)) {
		ha->isp_ops->fw_dump(vha, 1);
		set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
	}

	BUG_ON(is_debug(QDBG_CRASH_ON_ERR));

	return 0;
}

/**
 * qla24xx_mbx_completion() - Process mailbox command completions.
 * @vha: SCSI driver HA context
 * @mb0: Mailbox0 register
 */
static void
qla24xx_mbx_completion(scsi_qla_host_t *vha, uint16_t mb0)
{
	uint16_t	cnt;
	uint32_t	mboxes;
	uint16_t __iomem *wptr;
	struct qla_hw_data *ha = vha->hw;
	struct device_reg_24xx __iomem *reg = &ha->iobase->isp24;

	/* Read all mbox registers? */
	WARN_ON_ONCE(ha->mbx_count > 32);
	mboxes = (1ULL << ha->mbx_count) - 1;
	if (!ha->mcp)
		ql_dbg(ql_dbg_async, vha, 0x504e, "MBX pointer ERROR.\n");
	else
		mboxes = ha->mcp->in_mb;

	/* Load return mailbox registers. */
	ha->flags.mbox_int = 1;
	ha->mailbox_out[0] = mb0;
	mboxes >>= 1;
	wptr = (uint16_t __iomem *)&reg->mailbox1;

	for (cnt = 1; cnt < ha->mbx_count; cnt++) {
		if (mboxes & BIT_0)
			ha->mailbox_out[cnt] = RD_REG_WORD(wptr);

		mboxes >>= 1;
		wptr++;
	}
}

static void
qla24xx_abort_iocb_entry(scsi_qla_host_t *vha, struct req_que *req,
	struct abort_entry_24xx *pkt)
{
	const char func[] = "ABT_IOCB";
	srb_t *sp;
	srb_t *orig_sp = NULL;
	struct srb_iocb *abt;
	struct qla_hw_data *ha;
	ha = vha->hw;

	sp = qla2x00_get_sp_from_handle(vha, func, req, pkt);
	if (!sp)
		return;

	abt = &sp->u.iocb_cmd;
	abt->u.abt.comp_status = le16_to_cpu(pkt->comp_status);

	orig_sp = sp->cmd_sp;
	/* Need to pass original sp */
	if(orig_sp)
		qla_nvme_abort_process_comp_status(pkt, orig_sp);

	sp->done(sp, 0);
}

void qla24xx_nvme_ls4_iocb(struct scsi_qla_host *vha,
    struct pt_ls4_request *pkt, struct req_que *req)
{
	srb_t *sp;
	const char func[] = "LS4_IOCB";
	uint16_t comp_status;

	sp = qla2x00_get_sp_from_handle(vha, func, req, pkt);
	if (!sp)
		return;

	comp_status = le16_to_cpu(pkt->status);
	sp->done(sp, comp_status);
}

static void
qla24xx_process_abts(struct scsi_qla_host *vha, struct purex_item *pkt)
{
	struct abts_entry_24xx *abts =
	    (struct abts_entry_24xx *)&pkt->iocb;
	struct qla_hw_data *ha = vha->hw;
	struct els_entry_24xx *rsp_els;
	struct abts_entry_24xx *abts_rsp;
	dma_addr_t dma;
	uint32_t fctl;
	int rval;

	ql_dbg(ql_dbg_init, vha, 0x0286, "%s: entered.\n", __func__);

	ql_log(ql_log_warn, vha, 0x0287,
	    "Processing ABTS xchg=%#x oxid=%#x rxid=%#x seqid=%#x seqcnt=%#x\n",
	    abts->rx_xch_addr_to_abort, abts->ox_id, abts->rx_id,
	    abts->seq_id,abts->seq_cnt);
	ql_dbg(ql_dbg_init + ql_dbg_verbose, vha, 0x0287,
	    "-------- ABTS RCV -------\n");
	ql_dump_buffer(ql_dbg_init + ql_dbg_verbose, vha, 0x0287,
	    (uint8_t *)abts, sizeof(*abts));

	rsp_els = dma_alloc_coherent(&ha->pdev->dev, sizeof(*rsp_els), &dma,
	    GFP_KERNEL);
	if (!rsp_els) {
		ql_log(ql_log_warn, vha, 0x0287,
		    "Failed allocate dma buffer ABTS/ELS RSP.\n");
		return;
	}

	/* terminate exchange */
	memset(rsp_els, 0, sizeof(*rsp_els));
	rsp_els->entry_type = ELS_IOCB_TYPE;
	rsp_els->entry_count = 1;
	rsp_els->nport_handle = ~0;
	rsp_els->rx_xchg_address = abts->rx_xch_addr_to_abort;
	rsp_els->control_flags = EPD_RX_XCHG;
	ql_dbg(ql_dbg_init, vha, 0x0283,
	    "Sending ELS Response to terminate exchange %#x...\n",
	    abts->rx_xch_addr_to_abort);
	ql_dbg(ql_dbg_init + ql_dbg_verbose, vha, 0x0283,
	    "-------- ELS RSP -------\n");
	ql_dump_buffer(ql_dbg_init + ql_dbg_verbose, vha, 0x0283,
	    (uint8_t *)rsp_els, sizeof(*rsp_els));
	rval = qla2x00_issue_iocb(vha, rsp_els, dma, 0);
	if (rval) {
		ql_log(ql_log_warn, vha, 0x0288,
		    "%s: iocb failed to execute -> %x\n", __func__, rval);
	} else if (rsp_els->comp_status) {
		ql_log(ql_log_warn, vha, 0x0289,
		    "%s: iocb failed to complete -> "
		    "completion=%#x subcode=(%#x,%#x)\n",
		    __func__, rsp_els->comp_status,
		    rsp_els->error_subcode_1, rsp_els->error_subcode_2);
	} else {
		ql_dbg(ql_dbg_init, vha, 0x028a,
		    "%s: abort exchange done.\n", __func__);
	}

	/* send ABTS response */
	abts_rsp = (void *)rsp_els;
	memset(abts_rsp, 0, sizeof(*abts_rsp));
	abts_rsp->entry_type = ABTS_RSP_TYPE;
	abts_rsp->entry_count = 1;
	abts_rsp->nport_handle = abts->nport_handle;
	abts_rsp->vp_idx = abts->vp_idx;
	abts_rsp->sof_type = abts->sof_type & 0xf0;
	abts_rsp->rx_xch_addr = abts->rx_xch_addr;
	abts_rsp->d_id[0] = abts->s_id[0];
	abts_rsp->d_id[1] = abts->s_id[1];
	abts_rsp->d_id[2] = abts->s_id[2];
	abts_rsp->r_ctl = FC_ROUTING_BLD | FC_R_CTL_BLD_BA_ACC;
	abts_rsp->s_id[0] = abts->d_id[0];
	abts_rsp->s_id[1] = abts->d_id[1];
	abts_rsp->s_id[2] = abts->d_id[2];
	abts_rsp->cs_ctl = abts->cs_ctl;
	/* include flipping bit23 in fctl */
	fctl = ~(abts->f_ctl[2] | 0x7F) << 16 |
	    FC_F_CTL_LAST_SEQ | FC_F_CTL_END_SEQ | FC_F_CTL_SEQ_INIT;
	abts_rsp->f_ctl[0] = fctl >> 0 & 0xff;
	abts_rsp->f_ctl[1] = fctl >> 8 & 0xff;
	abts_rsp->f_ctl[2] = fctl >> 16 & 0xff;
	abts_rsp->type = FC_TYPE_BLD;
	abts_rsp->rx_id = abts->rx_id;
	abts_rsp->ox_id = abts->ox_id;
	abts_rsp->payload.ba_acc.aborted_rx_id = abts->rx_id;
	abts_rsp->payload.ba_acc.aborted_ox_id = abts->ox_id;
	abts_rsp->payload.ba_acc.high_seq_cnt = ~0;
	abts_rsp->rx_xch_addr_to_abort = abts->rx_xch_addr_to_abort;
	ql_dbg(ql_dbg_init, vha, 0x028b,
	    "Sending BA ACC response to ABTS %#x...\n",
	    abts->rx_xch_addr_to_abort);
	ql_dbg(ql_dbg_init + ql_dbg_verbose, vha, 0x028b,
	    "-------- ELS RSP -------\n");
	ql_dump_buffer(ql_dbg_init + ql_dbg_verbose, vha, 0x028b,
	    (uint8_t *)abts_rsp, sizeof(*abts_rsp));
	rval = qla2x00_issue_iocb(vha, abts_rsp, dma, 0);
	if (rval) {
		ql_log(ql_log_warn, vha, 0x028c,
		    "%s: iocb failed to execute -> %x\n", __func__, rval);
	} else if (abts_rsp->comp_status) {
		ql_log(ql_log_warn, vha, 0x028d,
		    "%s: iocb failed to complete -> "
		    "completion=%#x subcode=(%#x,%#x)\n",
		    __func__, abts_rsp->comp_status,
		    abts_rsp->payload.error.subcode1,
		    abts_rsp->payload.error.subcode2);
	} else {
		ql_dbg(ql_dbg_init, vha, 0x028ea,
		    "%s: done.\n", __func__);
	}

	dma_free_coherent(&ha->pdev->dev, sizeof(*rsp_els), rsp_els, dma);
}

static bool
qla25xx_rdp_rsp_reduce_size(struct scsi_qla_host *vha,
			    struct purex_entry_24xx *purex)
{
	char fwstr[16];
	u32 sid = purex->s_id[2] << 16 | purex->s_id[1] << 8 | purex->s_id[0];
	struct port_database_24xx *pdb;

	/* Domain Controller is always logged-out. */
	/* if RDP request is not from Domain Controller: */
	if (sid != 0xfffc01)
		return false;

	ql_dbg(ql_dbg_init, vha, 0x0181, "%s: s_id=%#x\n", __func__, sid);

	pdb = kzalloc(sizeof(*pdb), GFP_KERNEL);
	if (!pdb) {
		ql_dbg(ql_dbg_init, vha, 0x0181,
		    "%s: Failed allocate pdb\n", __func__);
	} else if (qla24xx_get_port_database(vha, purex->nport_handle, pdb)) {
		ql_dbg(ql_dbg_init, vha, 0x0181,
		    "%s: Failed get pdb sid=%x\n", __func__, sid);
	} else if (pdb->current_login_state != PDS_PLOGI_COMPLETE &&
	    pdb->current_login_state != PDS_PRLI_COMPLETE) {
		ql_dbg(ql_dbg_init, vha, 0x0181,
		    "%s: Port not logged in sid=%#x\n", __func__, sid);
	} else {
		/* RDP request is from logged in port */
		kfree(pdb);
		return false;
	}
	kfree(pdb);

	vha->hw->isp_ops->fw_version_str(vha, fwstr, sizeof(fwstr));
	fwstr[strcspn(fwstr, " ")] = 0;
	/* if FW version allows RDP response length upto 2048 bytes: */
	if (strcmp(fwstr, "8.09.00") > 0 || strcmp(fwstr, "8.05.65") == 0)
		return false;

	ql_dbg(ql_dbg_init, vha, 0x0181, "%s: fw=%s\n", __func__, fwstr);

	/* RDP response length is to be reduced to maximum 256 bytes */
	return true;
}

/*
 * Function Name: qla24xx_process_purex_iocb
 *
 * Description:
 * Prepare a RDP response and send to Fabric switch
 *
 * PARAMETERS:
 * vha:	SCSI qla host
 * purex: RDP request received by HBA
 */
static void
qla24xx_process_purex_rdp(struct scsi_qla_host *vha,
			  struct purex_item *item)
{
	struct qla_hw_data *ha = vha->hw;
	struct purex_entry_24xx *purex =
	    (struct purex_entry_24xx *)&item->iocb;
	dma_addr_t rsp_els_dma;
	dma_addr_t rsp_payload_dma;
	dma_addr_t stat_dma;
	dma_addr_t sfp_dma;
	struct els_entry_24xx *rsp_els = NULL;
	struct rdp_rsp_payload *rsp_payload = NULL;
	struct link_statistics *stat = NULL;
	uint8_t *sfp = NULL;
	uint16_t sfp_flags = 0;
	uint rsp_payload_length = sizeof(*rsp_payload);
	int rval;

	ql_dbg(ql_dbg_init + ql_dbg_verbose, vha, 0x0180,
	    "%s: Enter\n", __func__);

	ql_dbg(ql_dbg_init + ql_dbg_verbose, vha, 0x0181,
	    "-------- ELS REQ -------\n");
	ql_dump_buffer(ql_dbg_init + ql_dbg_verbose, vha, 0x0182,
	    purex, sizeof(*purex));

	if (qla25xx_rdp_rsp_reduce_size(vha, purex)) {
		rsp_payload_length =
		    offsetof(typeof(*rsp_payload), optical_elmt_desc);
		ql_dbg(ql_dbg_init, vha, 0x0181,
		    "Reducing RSP payload length to %u bytes...\n",
		    rsp_payload_length);
	}

	rsp_els = dma_zalloc_coherent(&ha->pdev->dev, sizeof(*rsp_els),
	    &rsp_els_dma, GFP_KERNEL);
	if (!rsp_els) {
		ql_log(ql_log_warn, vha, 0x0183,
		    "Failed to allocate dma buffer ELS RSP.\n");
		goto dealloc;
	}

	rsp_payload = dma_zalloc_coherent(&ha->pdev->dev, sizeof(*rsp_payload),
	    &rsp_payload_dma, GFP_KERNEL);
	if (!rsp_payload) {
		ql_log(ql_log_warn, vha, 0x0184,
		    "Failed allocate dma buffer ELS RSP payload.\n");
		goto dealloc;
	}

	sfp = dma_alloc_coherent(&ha->pdev->dev, SFP_RTDI_LEN,
	    &sfp_dma, GFP_KERNEL);

	stat = dma_alloc_coherent(&ha->pdev->dev, sizeof(*stat),
	    &stat_dma, GFP_KERNEL);

	/* Prepare Response IOCB */
	rsp_els->entry_type = ELS_IOCB_TYPE;
	rsp_els->entry_count = 1;
	rsp_els->sys_define = 0;
	rsp_els->entry_status = 0;
	rsp_els->handle = 0;
	rsp_els->nport_handle = purex->nport_handle;
	rsp_els->tx_dsd_count = 1;
	rsp_els->vp_index = purex->vp_idx;
	rsp_els->sof_type = EST_SOFI3;
	rsp_els->rx_xchg_address = purex->rx_xchg_addr;
	rsp_els->rx_dsd_count = 0;
	rsp_els->opcode = purex->els_frame_payload[0];

	rsp_els->d_id[0] = purex->s_id[0];
	rsp_els->d_id[1] = purex->s_id[1];
	rsp_els->d_id[2] = purex->s_id[2];

	rsp_els->control_flags = EPD_ELS_ACC;
	rsp_els->rx_byte_count = 0;
	rsp_els->tx_byte_count = rsp_payload_length;

	rsp_els->tx_address = rsp_payload_dma;
	rsp_els->tx_len = rsp_els->tx_byte_count;

	rsp_els->rx_address = 0;
	rsp_els->rx_len = 0;

	/* Prepare Response Payload */
	rsp_payload->hdr.cmd = cpu_to_be32(0x2 << 24); /* LS_ACC */
	rsp_payload->hdr.len = cpu_to_be32(
	    rsp_els->tx_byte_count - sizeof(rsp_payload->hdr));

	/* Link service Request Info Descriptor */
	rsp_payload->ls_req_info_desc.desc_tag = cpu_to_be32(0x1);
	rsp_payload->ls_req_info_desc.desc_len =
	    cpu_to_be32(RDP_DESC_LEN(rsp_payload->ls_req_info_desc));
	rsp_payload->ls_req_info_desc.req_payload_word_0 =
	    cpu_to_be32p((uint32_t *)purex->els_frame_payload);

	/* Link service Request Info Descriptor 2 */
	rsp_payload->ls_req_info_desc2.desc_tag = cpu_to_be32(0x1);
	rsp_payload->ls_req_info_desc2.desc_len =
	    cpu_to_be32(RDP_DESC_LEN(rsp_payload->ls_req_info_desc2));
	rsp_payload->ls_req_info_desc2.req_payload_word_0 =
	    cpu_to_be32p((uint32_t *)purex->els_frame_payload);


	rsp_payload->sfp_diag_desc.desc_tag = cpu_to_be32(0x10000);
	rsp_payload->sfp_diag_desc.desc_len =
		cpu_to_be32(RDP_DESC_LEN(rsp_payload->sfp_diag_desc));

	if (sfp) {
		/* SFP Flags */
		memset(sfp, 0, SFP_RTDI_LEN);
		rval = qla2x00_read_sfp(vha, sfp_dma, sfp, 0xa0, 0x7, 2, 0);
		if (!rval) {
			/* SFP Flags bits 3-0: Port Tx Laser Type */
			if (sfp[0] & BIT_2 || sfp[1] & (BIT_6|BIT_5))
				sfp_flags |= BIT_0; /* short wave */
			else if (sfp[0] & BIT_1)
				sfp_flags |= BIT_1; /* long wave 1310nm */
			else if (sfp[1] & BIT_4)
				sfp_flags |= BIT_1|BIT_0; /* long wave 1550nm */
		}

		/* SFP Type */
		memset(sfp, 0, SFP_RTDI_LEN);
		rval = qla2x00_read_sfp(vha, sfp_dma, sfp, 0xa0, 0x0, 1, 0);
		if (!rval) {
			sfp_flags |= BIT_4; /* optical */
			if (sfp[0] == 0x3)
				sfp_flags |= BIT_6; /* sfp+ */
		}

		rsp_payload->sfp_diag_desc.sfp_flags = cpu_to_be16(sfp_flags);

		/* SFP Diagnostics */
		memset(sfp, 0, SFP_RTDI_LEN);
		rval = qla2x00_read_sfp(vha, sfp_dma, sfp, 0xa2, 0x60, 10, 0);
		if (!rval) {
			uint16_t *trx = (void *)sfp; /* already be16 */
			rsp_payload->sfp_diag_desc.temperature = trx[0];
			rsp_payload->sfp_diag_desc.vcc = trx[1];
			rsp_payload->sfp_diag_desc.tx_bias = trx[2];
			rsp_payload->sfp_diag_desc.tx_power = trx[3];
			rsp_payload->sfp_diag_desc.rx_power = trx[4];
		}
	}

	/* Port Speed Descriptor */
	rsp_payload->port_speed_desc.desc_tag = cpu_to_be32(0x10001);
	rsp_payload->port_speed_desc.desc_len =
	    cpu_to_be32(RDP_DESC_LEN(rsp_payload->port_speed_desc));
	rsp_payload->port_speed_desc.speed_capab = cpu_to_be16(
	    qla25xx_rdp_port_speed_capability(ha)); 
	rsp_payload->port_speed_desc.operating_speed = cpu_to_be16(
	    qla25xx_rdp_port_speed_currently(ha));

	/* Link Error Status Descriptor */
	rsp_payload->ls_err_desc.desc_tag = cpu_to_be32(0x10002);
	rsp_payload->ls_err_desc.desc_len =
		cpu_to_be32(RDP_DESC_LEN(rsp_payload->ls_err_desc));

	if (stat) {
		rval = qla24xx_get_isp_stats(vha, stat, stat_dma, 0);
		if (!rval) {
			rsp_payload->ls_err_desc.link_fail_cnt =
			    cpu_to_be32(stat->link_fail_cnt);
			rsp_payload->ls_err_desc.loss_sync_cnt =
			    cpu_to_be32(stat->loss_sync_cnt);
			rsp_payload->ls_err_desc.loss_sig_cnt =
			    cpu_to_be32(stat->loss_sig_cnt);
			rsp_payload->ls_err_desc.prim_seq_err_cnt =
			    cpu_to_be32(stat->prim_seq_err_cnt);
			rsp_payload->ls_err_desc.inval_xmit_word_cnt =
			    cpu_to_be32(stat->inval_xmit_word_cnt);
			rsp_payload->ls_err_desc.inval_crc_cnt =
			    cpu_to_be32(stat->inval_crc_cnt);
			rsp_payload->ls_err_desc.pn_port_phy_type |= BIT_6;
		}
	}

	/* Portname Descriptor */
	rsp_payload->port_name_diag_desc.desc_tag = cpu_to_be32(0x10003);
	rsp_payload->port_name_diag_desc.desc_len =
	    cpu_to_be32(RDP_DESC_LEN(rsp_payload->port_name_diag_desc));
	memcpy(rsp_payload->port_name_diag_desc.WWNN,
	    vha->node_name,
	    sizeof(rsp_payload->port_name_diag_desc.WWNN));
	memcpy(rsp_payload->port_name_diag_desc.WWPN,
	    vha->port_name,
	    sizeof(rsp_payload->port_name_diag_desc.WWPN));

	/* F-Port Portname Descriptor */
	rsp_payload->port_name_direct_desc.desc_tag = cpu_to_be32(0x10003);
	rsp_payload->port_name_direct_desc.desc_len =
	    cpu_to_be32(RDP_DESC_LEN(rsp_payload->port_name_direct_desc));
	memcpy(rsp_payload->port_name_direct_desc.WWNN,
	    vha->fabric_node_name,
	    sizeof(rsp_payload->port_name_direct_desc.WWNN));
	memcpy(rsp_payload->port_name_direct_desc.WWPN,
	    vha->fabric_port_name,
	    sizeof(rsp_payload->port_name_direct_desc.WWPN));

	/* Bufer Credit Descriptor */
	rsp_payload->buffer_credit_desc.desc_tag = cpu_to_be32(0x10006);
	rsp_payload->buffer_credit_desc.desc_len =
		cpu_to_be32(RDP_DESC_LEN(rsp_payload->buffer_credit_desc));
	rsp_payload->buffer_credit_desc.fcport_b2b = 0;
	rsp_payload->buffer_credit_desc.attached_fcport_b2b = cpu_to_be32(0);
	rsp_payload->buffer_credit_desc.fcport_rtt = cpu_to_be32(0);

	if (ha->flags.plogi_template_valid) {
		uint32_t tmp =
		be16_to_cpu(ha->plogi_els_payld.fl_csp.sp_bb_cred);
		rsp_payload->buffer_credit_desc.fcport_b2b = cpu_to_be32(tmp);
	}

	if (rsp_payload_length < sizeof(*rsp_payload))
		goto send;

	/* Optical Element Descriptor, Temperature */
	rsp_payload->optical_elmt_desc[0].desc_tag = cpu_to_be32(0x10007);
	rsp_payload->optical_elmt_desc[0].desc_len =
		cpu_to_be32(RDP_DESC_LEN(*rsp_payload->optical_elmt_desc));
	/* Optical Element Descriptor, Voltage */
	rsp_payload->optical_elmt_desc[1].desc_tag = cpu_to_be32(0x10007);
	rsp_payload->optical_elmt_desc[1].desc_len =
		cpu_to_be32(RDP_DESC_LEN(*rsp_payload->optical_elmt_desc));
	/* Optical Element Descriptor, Tx Bias Current */
	rsp_payload->optical_elmt_desc[2].desc_tag = cpu_to_be32(0x10007);
	rsp_payload->optical_elmt_desc[2].desc_len =
		cpu_to_be32(RDP_DESC_LEN(*rsp_payload->optical_elmt_desc));
	/* Optical Element Descriptor, Tx Power */
	rsp_payload->optical_elmt_desc[3].desc_tag = cpu_to_be32(0x10007);
	rsp_payload->optical_elmt_desc[3].desc_len =
		cpu_to_be32(RDP_DESC_LEN(*rsp_payload->optical_elmt_desc));
	/* Optical Element Descriptor, Rx Power */
	rsp_payload->optical_elmt_desc[4].desc_tag = cpu_to_be32(0x10007);
	rsp_payload->optical_elmt_desc[4].desc_len =
		cpu_to_be32(RDP_DESC_LEN(*rsp_payload->optical_elmt_desc));

	if (sfp) {
		memset(sfp, 0, SFP_RTDI_LEN);
		rval = qla2x00_read_sfp(vha, sfp_dma, sfp, 0xa2, 0, 64, 0);
		if (!rval) {
			uint16_t *trx = (void *)sfp; /* already be16 */

			/* Optical Element Descriptor, Temperature */
			rsp_payload->optical_elmt_desc[0].high_alarm = trx[0];
			rsp_payload->optical_elmt_desc[0].low_alarm = trx[1];
			rsp_payload->optical_elmt_desc[0].high_warn = trx[2];
			rsp_payload->optical_elmt_desc[0].low_warn = trx[3];
			rsp_payload->optical_elmt_desc[0].element_flags =
			    cpu_to_be32(1 << 28);

			/* Optical Element Descriptor, Voltage */
			rsp_payload->optical_elmt_desc[1].high_alarm = trx[4];
			rsp_payload->optical_elmt_desc[1].low_alarm = trx[5];
			rsp_payload->optical_elmt_desc[1].high_warn = trx[6];
			rsp_payload->optical_elmt_desc[1].low_warn = trx[7];
			rsp_payload->optical_elmt_desc[1].element_flags =
			    cpu_to_be32(2 << 28);

			/* Optical Element Descriptor, Tx Bias Current */
			rsp_payload->optical_elmt_desc[2].high_alarm = trx[8];
			rsp_payload->optical_elmt_desc[2].low_alarm = trx[9];
			rsp_payload->optical_elmt_desc[2].high_warn = trx[10];
			rsp_payload->optical_elmt_desc[2].low_warn = trx[11];
			rsp_payload->optical_elmt_desc[2].element_flags =
			    cpu_to_be32(3 << 28);

			/* Optical Element Descriptor, Tx Power */
			rsp_payload->optical_elmt_desc[3].high_alarm = trx[12];
			rsp_payload->optical_elmt_desc[3].low_alarm = trx[13];
			rsp_payload->optical_elmt_desc[3].high_warn = trx[14];
			rsp_payload->optical_elmt_desc[3].low_warn = trx[15];
			rsp_payload->optical_elmt_desc[3].element_flags =
			    cpu_to_be32(4 << 28);

			/* Optical Element Descriptor, Rx Power */
			rsp_payload->optical_elmt_desc[4].high_alarm = trx[16];
			rsp_payload->optical_elmt_desc[4].low_alarm = trx[17];
			rsp_payload->optical_elmt_desc[4].high_warn = trx[18];
			rsp_payload->optical_elmt_desc[4].low_warn = trx[19];
			rsp_payload->optical_elmt_desc[4].element_flags =
			    cpu_to_be32(5 << 28);
		}

		memset(sfp, 0, SFP_RTDI_LEN);
		rval = qla2x00_read_sfp(vha, sfp_dma, sfp, 0xa2, 112, 64, 0);
		if (!rval) {
			/* Temperature high/low alarm/warning */
			rsp_payload->optical_elmt_desc[0].element_flags |=
			    cpu_to_be32(
				(sfp[0] >> 7 & 1) << 3 | (sfp[0] >> 6 & 1) << 2 |
				(sfp[4] >> 7 & 1) << 1 | (sfp[4] >> 6 & 1) << 0);

			/* Voltage high/low alarm/warning */
			rsp_payload->optical_elmt_desc[1].element_flags |=
			    cpu_to_be32(
				(sfp[0] >> 5 & 1) << 3 | (sfp[0] >> 4 & 1) << 2 |
				(sfp[4] >> 5 & 1) << 1 | (sfp[4] >> 4 & 1) << 0);

			/* Tx Bias Current high/low alarm/warning */
			rsp_payload->optical_elmt_desc[2].element_flags |=
			    cpu_to_be32(
				(sfp[0] >> 3 & 1) << 3 | (sfp[0] >> 2 & 1) << 2 |
				(sfp[4] >> 3 & 1) << 1 | (sfp[4] >> 2 & 1) << 0);

			/* Tx Power high/low alarm/warning */
			rsp_payload->optical_elmt_desc[3].element_flags |=
			    cpu_to_be32(
				(sfp[0] >> 1 & 1) << 3 | (sfp[0] >> 0 & 1) << 2 |
				(sfp[4] >> 1 & 1) << 1 | (sfp[4] >> 0 & 1) << 0);

			/* Rx Power high/low alarm/warning */
			rsp_payload->optical_elmt_desc[4].element_flags |=
			    cpu_to_be32(
				(sfp[1] >> 7 & 1) << 3 | (sfp[1] >> 6 & 1) << 2 |
				(sfp[5] >> 7 & 1) << 1 | (sfp[5] >> 6 & 1) << 0);
		}
	}

	/* Optical Product Data Descriptor */
	rsp_payload->optical_prod_desc.desc_tag = cpu_to_be32(0x10008);
	rsp_payload->optical_prod_desc.desc_len =
		cpu_to_be32(RDP_DESC_LEN(rsp_payload->optical_prod_desc));

	if (sfp) {
		memset(sfp, 0, SFP_RTDI_LEN);
		rval = qla2x00_read_sfp(vha, sfp_dma, sfp, 0xa0, 20, 64, 0);
		if (!rval) {
			memcpy(rsp_payload->optical_prod_desc.vendor_name,
			    sfp + 0,
			    sizeof(rsp_payload->optical_prod_desc.vendor_name));
			memcpy(rsp_payload->optical_prod_desc.part_number,
			    sfp + 20,
			    sizeof(rsp_payload->optical_prod_desc.part_number));
			memcpy(rsp_payload->optical_prod_desc.revision,
			    sfp + 36,
			    sizeof(rsp_payload->optical_prod_desc.revision));
			memcpy(rsp_payload->optical_prod_desc.serial_number,
			    sfp + 48,
			    sizeof(rsp_payload->optical_prod_desc.serial_number));
		}

		memset(sfp, 0, SFP_RTDI_LEN);
		rval = qla2x00_read_sfp(vha, sfp_dma, sfp, 0xa0, 84, 8, 0);
		if (!rval) {
			memcpy(rsp_payload->optical_prod_desc.date,
			    sfp + 0,
			    sizeof(rsp_payload->optical_prod_desc.date));
		}
	}

send:
	ql_dbg(ql_dbg_init, vha, 0x0183,
	    "Sending ELS Response to RDP Request...\n");
	ql_dbg(ql_dbg_init + ql_dbg_verbose, vha, 0x0184,
	    "-------- ELS RSP -------\n");
	ql_dump_buffer(ql_dbg_init + ql_dbg_verbose, vha, 0x0185,
	    rsp_els, sizeof(*rsp_els));
	ql_dbg(ql_dbg_init + ql_dbg_verbose, vha, 0x0186,
	    "-------- ELS RSP PAYLOAD -------\n");
	ql_dump_buffer(ql_dbg_init + ql_dbg_verbose, vha, 0x0187,
	    rsp_payload, rsp_payload_length);

	rval = qla2x00_issue_iocb(vha, rsp_els, rsp_els_dma, 0);

	if (rval) {
		ql_log(ql_log_warn, vha, 0x0188,
		    "%s: iocb failed to execute -> %x\n", __func__, rval);
	} else if (rsp_els->comp_status) {
		ql_log(ql_log_warn, vha, 0x0189,
		    "%s: iocb failed to complete -> "
		    "completion=%#x subcode=(%#x,%#x)\n",
		    __func__, rsp_els->comp_status,
		    rsp_els->error_subcode_1, rsp_els->error_subcode_2);
	} else {
		ql_dbg(ql_dbg_init, vha, 0x018a, "%s: done.\n", __func__);
	}

dealloc:
	if (stat)
		dma_free_coherent(&ha->pdev->dev, sizeof(*stat),
		    stat, stat_dma);
	if (sfp)
		dma_free_coherent(&ha->pdev->dev, SFP_RTDI_LEN,
		    sfp, sfp_dma);
	if (rsp_payload)
		dma_free_coherent(&ha->pdev->dev, sizeof(*rsp_payload),
		    rsp_payload, rsp_payload_dma);
	if (rsp_els)
		dma_free_coherent(&ha->pdev->dev, sizeof(*rsp_els),
		    rsp_els, rsp_els_dma);
}

/**
 * qla_chk_cont_iocb_avail - check for all continuation iocbs are available
 *   before iocb processing can start.
 * @vha: host adapter pointer
 * @rsp: respond queue
 * @pkt: head iocb describing how many continuation iocb
 * Return: 0 all iocbs has arrived, xx- all iocbs have not arrived.
*/
static int qla_chk_cont_iocb_avail(struct scsi_qla_host *vha,
	struct rsp_que *rsp, response_t *pkt, u32 rsp_q_in)
{
	int start_pkt_ring_index;
	u32 iocb_cnt = 0;
	int rc = 0;

	if (pkt->entry_count == 1 )
		return rc;

	/* ring_index was pre-increment.  Set it back to current pkt */
	if (rsp->ring_index == 0)
		start_pkt_ring_index = rsp->length - 1;
	else
		start_pkt_ring_index = rsp->ring_index - 1;

	if (rsp_q_in < start_pkt_ring_index)
		/* q in ptr is wrapped */
		iocb_cnt = rsp->length - start_pkt_ring_index + rsp_q_in;
	else
		iocb_cnt = rsp_q_in - start_pkt_ring_index;

	if (iocb_cnt < pkt->entry_count)
		rc = -EIO;

	ql_dbg(ql_dbg_init, vha, 0x5091,
	    "%s - ring %p pkt %p  entry count %d iocb_cnt %d rsp_q_in %d rc %d\n",
	    __func__, rsp->ring, pkt, pkt->entry_count, iocb_cnt, rsp_q_in, rc);

	return rc;
}

static void qla_marker_iocb_entry(scsi_qla_host_t *vha, struct req_que *req,
	struct mrk_entry_24xx *pkt)
{
	const char func[] = "MRK-IOCB";
	srb_t *sp;
	int res = QLA_SUCCESS;

	if (!IS_FWI2_CAPABLE(vha->hw))
		return;

	sp = qla2x00_get_sp_from_handle(vha, func, req, pkt);
	if (!sp)
		return;

	if (pkt->entry_status) {
		ql_dbg(ql_dbg_taskm, vha, 0x8025, "marker failure.\n");
		res = QLA_COMMAND_ERROR;
	}
	sp->u.iocb_cmd.u.tmf.data = res;
	sp->done(sp, res);
}


/**
 * qla24xx_process_response_queue() - Process response queue entries.
 * @vha: SCSI driver HA context
 * @rsp: response queue
 */
void qla24xx_process_response_queue(struct scsi_qla_host *vha,
	struct rsp_que *rsp)
{
	struct sts_entry_24xx *pkt;
	struct qla_hw_data *ha = vha->hw;
	struct purex_entry_24xx *purex_entry;
	struct purex_item *pure_item;
	u16 rsp_in = 0, cur_ring_index;
	int follow_inptr, is_shadow_hba;

	if (!ha->flags.fw_started)
		return;

	if (rsp->qpair->cpuid != smp_processor_id() || !rsp->qpair->rcv_intr) {
		rsp->qpair->rcv_intr = 1;
	}

#define __update_rsp_in(_update, _is_shadow_hba, _rsp, _rsp_in)	\
	if (_update) {						\
		_rsp_in = _is_shadow_hba ? *(_rsp)->in_ptr :	\
			RD_REG_DWORD_RELAXED((_rsp)->rsp_q_in);	\
	}

	is_shadow_hba = IS_SHADOW_REG_CAPABLE(ha);
	follow_inptr = is_shadow_hba ? ql2xrspq_follow_inptr :
				ql2xrspq_follow_inptr_legacy;

	__update_rsp_in(follow_inptr, is_shadow_hba, rsp, rsp_in);

	while ((likely(follow_inptr &&
			rsp->ring_index != rsp_in &&
			rsp->ring_ptr->signature != RESPONSE_PROCESSED)) ||
			(!follow_inptr &&
			 rsp->ring_ptr->signature != RESPONSE_PROCESSED)) {

		pkt = (struct sts_entry_24xx *)rsp->ring_ptr;
		cur_ring_index = rsp->ring_index;

		rsp->ring_index++;
		if (rsp->ring_index == rsp->length) {
			rsp->ring_index = 0;
			rsp->ring_ptr = rsp->ring;
		} else {
			rsp->ring_ptr++;
		}

		if (pkt->entry_status != 0) {
			if (qla2x00_error_entry(vha, rsp, (sts_entry_t *) pkt))
				goto process_err;

			((response_t *)pkt)->signature = RESPONSE_PROCESSED;
			wmb();
			continue;
		}
process_err:

		switch (pkt->entry_type) {
		case STATUS_TYPE:
			qla2x00_status_entry(vha, rsp, pkt);
			break;
		case STATUS_CONT_TYPE:
			qla2x00_status_cont_entry(rsp, (sts_cont_entry_t *)pkt);
			break;
		case VP_RPT_ID_IOCB_TYPE:
			qla24xx_report_id_acquisition(vha,
			    (struct vp_rpt_id_entry_24xx *)pkt);
			break;
		case STATUS_CONT_TYPE_1:
			qla27xx_status_cont_type_1(vha, (sts_cont_entry_t *)pkt);
			break;
		case LOGINOUT_PORT_IOCB_TYPE:
			qla24xx_logio_entry(vha, rsp->req,
			    (struct logio_entry_24xx *)pkt);
			break;
		case CT_IOCB_TYPE:
			qla24xx_els_ct_entry(vha, rsp->req, pkt, CT_IOCB_TYPE);
			break;
		case ELS_IOCB_TYPE:
			qla24xx_els_ct_entry(vha, rsp->req, pkt, ELS_IOCB_TYPE);
			break;
		case ABTS_RECV_24XX:
			if (qla_ini_mode_enabled(vha)) {
				pure_item = qla24xx_copy_std_pkt(vha, pkt);
				if (!pure_item)
					break;

				qla24xx_queue_purex_item(vha, pure_item,
							 qla24xx_process_abts);
				break;
			}
			if (IS_QLA83XX(ha) || IS_QLA27XX(ha) ||
			    IS_QLA28XX(ha)) {
				/* ensure that the ATIO queue is empty */
				qlt_handle_abts_recv(vha, rsp,
				    (response_t *)pkt);
				break;
			} else {
				qlt_24xx_process_atio_queue(vha, 1);
			}
			fallthrough;
		case ABTS_RESP_24XX:
		case CTIO_TYPE7:
		case CTIO_CRC2:
			qlt_response_pkt_all_vps(vha, rsp, (response_t *)pkt);
			break;
		case PT_LS4_REQUEST:
			qla24xx_nvme_ls4_iocb(vha, (struct pt_ls4_request *)pkt,
			    rsp->req);
			break;
		case NOTIFY_ACK_TYPE:
			if (pkt->handle == QLA_TGT_SKIP_HANDLE)
				qlt_response_pkt_all_vps(vha, rsp,
				    (response_t *)pkt);
			else
				qla24xxx_nack_iocb_entry(vha, rsp->req,
					(struct nack_to_isp *)pkt);
			break;
		case MARKER_TYPE:
			qla_marker_iocb_entry(vha, rsp->req, (struct mrk_entry_24xx*)pkt);
			break;
		case ABORT_IOCB_TYPE:
			qla24xx_abort_iocb_entry(vha, rsp->req,
			    (struct abort_entry_24xx *)pkt);
			break;
		case MBX_IOCB_TYPE:
			qla24xx_mbx_iocb_entry(vha, rsp->req,
			    (struct mbx_24xx_entry *)pkt);
			break;
		case VP_CTRL_IOCB_TYPE:
			qla_ctrlvp_completed(vha, rsp->req,
			    (struct vp_ctrl_entry_24xx *)pkt);
			break;
		case PUREX_IOCB_TYPE:
			purex_entry = (void *)pkt;
			switch (purex_entry->els_frame_payload[3]) {
			case ELS_COMMAND_RDP:
				pure_item = qla24xx_copy_std_pkt(vha, pkt);
				if (!pure_item)
					break;
				qla24xx_queue_purex_item(vha, pure_item,
						 qla24xx_process_purex_rdp);
				break;
			case ELS_COMMAND_FPIN:
				if (!vha->hw->flags.scm_enabled) {
					ql_log(ql_log_warn, vha, 0x5094,
					       "SCM not active for this port\n");
					break;
				}
				pure_item = qla27xx_copy_fpin_pkt(vha,
							  (void **)&pkt, &rsp);
				__update_rsp_in(follow_inptr, is_shadow_hba,
								rsp, rsp_in);
				if (!pure_item)
					break;
				qla24xx_queue_purex_item(vha, pure_item,
						 qla27xx_process_purex_fpin);
				break;
			case ELS_AUTH_ELS:
				if (qla_chk_cont_iocb_avail(vha, rsp, (response_t *) pkt, rsp_in)) {
					/*
					 * ring_ptr and ring_index were pre-incremented above.
					 * Reset them back to current.  Wait for next interrupt
					 * with all IOCBs to arrive and re-process.
					 */
					rsp->ring_ptr = (response_t *)pkt;
					rsp->ring_index = cur_ring_index;

					ql_dbg(ql_dbg_init, vha, 0x5091,
					    "Defer processing ELS opcode %#x...\n",
					    purex_entry->els_frame_payload[3]);
					return;
				}
				qla24xx_auth_els(vha, (void**)&pkt, &rsp);
				break;
			case ELS_COMMAND_RDF:
				if (!vha->hw->flags.scm_enabled) {
					ql_log(ql_log_warn, vha, 0x5095,
					       "RDF received when SCM not active for this port\n");
					break;
				}
				pure_item = qla24xx_copy_std_pkt(vha, pkt);
				if (!pure_item)
					break;
				pure_item->qpair = rsp->qpair;
				vha->rdf_retry_cnt = 0;
				qla24xx_queue_purex_item(vha, pure_item,
						 qla2xxx_scm_process_purex_rdf);
				break;
			case ELS_COMMAND_EDC:
				if (!vha->hw->flags.scm_enabled) {
					ql_log(ql_log_warn, vha, 0x5096,
					       "EDC received when SCM not active for this port\n");
					break;
				}
				pure_item = qla24xx_copy_std_pkt(vha, pkt);
				if (!pure_item)
					break;
				pure_item->qpair = rsp->qpair;
				vha->hw->edc_retry_cnt = 0;
				qla24xx_queue_purex_item(vha, pure_item,
						 qla2xx_scm_process_purex_edc);
				break;
			default:
				ql_log(ql_log_warn, vha, 0x509c,
				       "Discarding ELS Request opcode 0x%x\n",
				       purex_entry->els_frame_payload[3]);
			}
			break;
		case SA_UPDATE_IOCB_TYPE:
			qla28xx_sa_update_iocb_entry(vha, rsp->req,
				(struct sa_update_28xx *)pkt);
			break;

		default:
			/* Type Not Supported. */
			ql_dbg(ql_dbg_async, vha, 0x5042,
			    "Received unknown response pkt type %x "
			    "entry status=%x.\n",
			    pkt->entry_type, pkt->entry_status);
			break;
		}

		((response_t *)pkt)->signature = RESPONSE_PROCESSED;
		wmb();
	}

	/* Adjust ring index */
	if (IS_P3P_TYPE(ha)) {
		struct device_reg_82xx __iomem *reg = &ha->iobase->isp82;

		WRT_REG_DWORD(&reg->rsp_q_out[0], rsp->ring_index);
	} else {
		WRT_REG_DWORD(rsp->rsp_q_out, rsp->ring_index);
	}
}

static void
qla2xxx_check_risc_status(scsi_qla_host_t *vha)
{
	uint32_t cnt;
	struct qla_hw_data *ha = vha->hw;
	struct device_reg_24xx __iomem *reg = &ha->iobase->isp24;

	if (!IS_QLA25XX(ha) && !IS_QLA81XX(ha) && !IS_QLA83XX(ha) &&
	    !IS_QLA27XX(ha) && !IS_QLA28XX(ha))
		return;

	WRT_REG_DWORD(&reg->iobase_addr, 0x7C00);
	RD_REG_DWORD(&reg->iobase_addr);
	WRT_REG_DWORD(&reg->iobase_window, 0x0001);
        for (cnt = 10000; cnt; cnt--) {
                if (RD_REG_DWORD(&reg->iobase_window) & BIT_0)
                        goto next_test;
                WRT_REG_DWORD(&reg->iobase_window, 0x0001);
                udelay(10);
        }

        WRT_REG_DWORD(&reg->iobase_window, 0x0003);
        for (cnt = 100; cnt; cnt--) {
                if (RD_REG_DWORD(&reg->iobase_window) & BIT_0)
                        goto next_test;
                WRT_REG_DWORD(&reg->iobase_window, 0x0003);
                udelay(10);
        }

        goto done;
next_test:
	if (RD_REG_DWORD(&reg->iobase_c8) & BIT_3)
		ql_log(ql_log_info, vha, 0x504c,
		    "Additional code -- 0x55AA.\n");

done:
	WRT_REG_DWORD(&reg->iobase_window, 0x0000);
	RD_REG_DWORD(&reg->iobase_window);
}

/**
 * qla24xx_intr_handler() - Process interrupts for the ISP23xx and ISP24xx.
 * @irq: interrupt number
 * @dev_id: SCSI driver HA context
 *
 * Called by system whenever the host adapter generates an interrupt.
 *
 * Returns handled flag.
 */
irqreturn_t
qla24xx_intr_handler(int irq, void *dev_id)
{
	scsi_qla_host_t	*vha;
	struct qla_hw_data *ha;
	struct device_reg_24xx __iomem *reg;
	int		status;
	unsigned long	iter;
	uint32_t	stat;
	uint32_t	hccr;
	uint16_t	mb[8];
	struct rsp_que *rsp;
	unsigned long	flags;
	bool process_atio = false;

	rsp = (struct rsp_que *) dev_id;
	if (!rsp) {
		ql_log(ql_log_info, NULL, 0x5059,
		    "%s: NULL response queue pointer.\n", __func__);
		return IRQ_NONE;
	}

	ha = rsp->hw;
	reg = &ha->iobase->isp24;
	status = 0;

	if (unlikely(pci_channel_offline(ha->pdev)))
		return IRQ_HANDLED;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	vha = pci_get_drvdata(ha->pdev);
	for (iter = 50; iter--; ) {
		stat = RD_REG_DWORD(&reg->host_status);
		if (qla2x00_check_reg32_for_disconnect(vha, stat))
			break;
		if (stat & HSRX_RISC_PAUSED) {
			if (unlikely(pci_channel_offline(ha->pdev)))
				break;

			hccr = RD_REG_DWORD(&reg->hccr);

			ql_log(ql_log_warn, vha, 0x504b,
			    "RISC paused -- HCCR=%x, Dumping firmware.\n",
			    hccr);

			qla2xxx_check_risc_status(vha);
			vha->hw_err_cnt++;

			ha->isp_ops->fw_dump(vha, 1);
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
			break;
		} else if ((stat & HSRX_RISC_INT) == 0)
			break;

		switch (stat & 0xff) {
		case INTR_ROM_MB_SUCCESS:
		case INTR_ROM_MB_FAILED:
		case INTR_MB_SUCCESS:
		case INTR_MB_FAILED:
			qla24xx_mbx_completion(vha, MSW(stat));
			status |= MBX_INTERRUPT;

			break;
		case INTR_ASYNC_EVENT:
			mb[0] = MSW(stat);
			mb[1] = RD_REG_WORD(&reg->mailbox1);
			mb[2] = RD_REG_WORD(&reg->mailbox2);
			mb[3] = RD_REG_WORD(&reg->mailbox3);
			qla2x00_async_event(vha, rsp, mb);
			break;
		case INTR_RSP_QUE_UPDATE:
		case INTR_RSP_QUE_UPDATE_83XX:
			qla24xx_process_response_queue(vha, rsp);
			break;
		case INTR_ATIO_QUE_UPDATE_27XX:
		case INTR_ATIO_QUE_UPDATE:
			process_atio = true;
			break;
		case INTR_ATIO_RSP_QUE_UPDATE:
			process_atio = true;
			qla24xx_process_response_queue(vha, rsp);
			break;
		default:
			ql_dbg(ql_dbg_async, vha, 0x504f,
			    "Unrecognized interrupt type (%d).\n", stat * 0xff);
			break;
		}
		WRT_REG_DWORD(&reg->hccr, HCCRX_CLR_RISC_INT);
		RD_REG_DWORD_RELAXED(&reg->hccr);
		if (unlikely(IS_QLA83XX(ha) && (ha->pdev->revision == 1)))
			ndelay(3500);
	}
	qla2x00_handle_mbx_completion(ha, status);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	if (process_atio) {
		spin_lock_irqsave(&ha->tgt.atio_lock, flags);
		qlt_24xx_process_atio_queue(vha, 0);
		spin_unlock_irqrestore(&ha->tgt.atio_lock, flags);
	}

	return IRQ_HANDLED;
}

static irqreturn_t
qla24xx_msix_rsp_q(int irq, void *dev_id)
{
	struct qla_hw_data *ha;
	struct rsp_que *rsp;
	struct device_reg_24xx __iomem *reg;
	struct scsi_qla_host *vha;
	unsigned long flags;

	rsp = (struct rsp_que *) dev_id;
	if (!rsp) {
		ql_log(ql_log_info, NULL, 0x505a,
		    "%s: NULL response queue pointer.\n", __func__);
		return IRQ_NONE;
	}
	ha = rsp->hw;
	reg = &ha->iobase->isp24;

	spin_lock_irqsave(&ha->hardware_lock, flags);

	vha = pci_get_drvdata(ha->pdev);
	qla24xx_process_response_queue(vha, rsp);
	if (!ha->flags.disable_msix_handshake) {
		WRT_REG_DWORD(&reg->hccr, HCCRX_CLR_RISC_INT);
		RD_REG_DWORD_RELAXED(&reg->hccr);
	}
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t
qla24xx_msix_default(int irq, void *dev_id)
{
	scsi_qla_host_t	*vha;
	struct qla_hw_data *ha;
	struct rsp_que *rsp;
	struct device_reg_24xx __iomem *reg;
	int		status;
	uint32_t	stat;
	uint32_t	hccr;
	uint16_t	mb[8];
	unsigned long flags;
	bool process_atio = false;

	rsp = (struct rsp_que *) dev_id;
	if (!rsp) {
		ql_log(ql_log_info, NULL, 0x505c,
		    "%s: NULL response queue pointer.\n", __func__);
		return IRQ_NONE;
	}
	ha = rsp->hw;
	reg = &ha->iobase->isp24;
	status = 0;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	vha = pci_get_drvdata(ha->pdev);
	do {
		stat = RD_REG_DWORD(&reg->host_status);
		if (qla2x00_check_reg32_for_disconnect(vha, stat))
			break;
		if (stat & HSRX_RISC_PAUSED) {
			if (unlikely(pci_channel_offline(ha->pdev)))
				break;

			hccr = RD_REG_DWORD(&reg->hccr);

			ql_log(ql_log_info, vha, 0x5050,
			    "RISC paused -- HCCR=%x, Dumping firmware.\n",
			    hccr);

			qla2xxx_check_risc_status(vha);
			vha->hw_err_cnt++;

			ha->isp_ops->fw_dump(vha, 1);
			set_bit(ISP_ABORT_NEEDED, &vha->dpc_flags);
			break;
		} else if ((stat & HSRX_RISC_INT) == 0)
			break;

		switch (stat & 0xff) {
		case INTR_ROM_MB_SUCCESS:
		case INTR_ROM_MB_FAILED:
		case INTR_MB_SUCCESS:
		case INTR_MB_FAILED:
			qla24xx_mbx_completion(vha, MSW(stat));
			status |= MBX_INTERRUPT;

			break;
		case INTR_ASYNC_EVENT:
			mb[0] = MSW(stat);
			mb[1] = RD_REG_WORD(&reg->mailbox1);
			mb[2] = RD_REG_WORD(&reg->mailbox2);
			mb[3] = RD_REG_WORD(&reg->mailbox3);
			qla2x00_async_event(vha, rsp, mb);
			break;
		case INTR_RSP_QUE_UPDATE:
		case INTR_RSP_QUE_UPDATE_83XX:
			qla24xx_process_response_queue(vha, rsp);
			break;
		case INTR_ATIO_QUE_UPDATE_27XX:
		case INTR_ATIO_QUE_UPDATE:
			process_atio = true;
			break;
		case INTR_ATIO_RSP_QUE_UPDATE:
			process_atio = true;
			qla24xx_process_response_queue(vha, rsp);
			break;
		default:
			ql_dbg(ql_dbg_async, vha, 0x5051,
			    "Unrecognized interrupt type (%d).\n", stat & 0xff);
			break;
		}
		WRT_REG_DWORD(&reg->hccr, HCCRX_CLR_RISC_INT);
	} while (0);
	qla2x00_handle_mbx_completion(ha, status);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	if (process_atio) {
		spin_lock_irqsave(&ha->tgt.atio_lock, flags);
		qlt_24xx_process_atio_queue(vha, 0);
		spin_unlock_irqrestore(&ha->tgt.atio_lock, flags);
	}

	return IRQ_HANDLED;
}

irqreturn_t
qla2xxx_msix_rsp_q(int irq, void *dev_id)
{
	struct qla_hw_data *ha;
	struct qla_qpair *qpair;

	qpair = dev_id;
	if (!qpair) {
		ql_log(ql_log_info, NULL, 0x505b,
		    "%s: NULL response queue pointer.\n", __func__);
		return IRQ_NONE;
	}
	ha = qpair->hw;

	queue_work_on(smp_processor_id(), ha->wq, &qpair->q_work);

	return IRQ_HANDLED;
}

irqreturn_t
qla2xxx_msix_rsp_q_hs(int irq, void *dev_id)
{
	struct qla_hw_data *ha;
	struct qla_qpair *qpair;
	struct device_reg_24xx __iomem *reg;
	unsigned long flags;

	qpair = dev_id;
	if (!qpair) {
		ql_log(ql_log_info, NULL, 0x505b,
		    "%s: NULL response queue pointer.\n", __func__);
		return IRQ_NONE;
	}
	ha = qpair->hw;

	reg = &ha->iobase->isp24;
	spin_lock_irqsave(&ha->hardware_lock, flags);
	WRT_REG_DWORD(&reg->hccr, HCCRX_CLR_RISC_INT);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);


	queue_work_on(smp_processor_id(), ha->wq, &qpair->q_work);

	return IRQ_HANDLED;
}

/* Interrupt handling helpers. */

struct qla_init_msix_entry {
	const char *name;
	irq_handler_t handler;
};

static const struct qla_init_msix_entry msix_entries[] = {
	{ "default", qla24xx_msix_default },
	{ "rsp_q", qla24xx_msix_rsp_q },
	{ "atio_q", qla83xx_msix_atio_q },
	{ "qpair_multiq", qla2xxx_msix_rsp_q },
	{ "qpair_multiq_hs", qla2xxx_msix_rsp_q_hs },
};

static const struct qla_init_msix_entry qla82xx_msix_entries[] = {
	{ "qla2xxx (default)", qla82xx_msix_default },
	{ "qla2xxx (rsp_q)", qla82xx_msix_rsp_q },
};

static int
qla24xx_enable_msix(struct qla_hw_data *ha, struct rsp_que *rsp)
{
	int i, ret;
	struct qla_msix_entry *qentry;
	scsi_qla_host_t *vha = pci_get_drvdata(ha->pdev);
	int min_vecs = QLA_BASE_VECTORS;
	struct irq_affinity desc = {
		.pre_vectors = QLA_BASE_VECTORS,
	};

	if (QLA_TGT_MODE_ENABLED() && (ql2xenablemsix != 0) &&
	    IS_ATIO_MSIX_CAPABLE(ha)) {
		desc.pre_vectors++;
		min_vecs++;
	}

	if (USER_CTRL_IRQ(ha) || !ha->mqiobase) {
		/* user wants to control IRQ setting for target mode */
		ret = pci_alloc_irq_vectors(ha->pdev, min_vecs,
		    min((u16)ha->msix_count, (u16)(num_online_cpus() + min_vecs)),
		    PCI_IRQ_MSIX);
	} else
		ret = pci_alloc_irq_vectors_affinity(ha->pdev, min_vecs,
		    min((u16)ha->msix_count, (u16)(num_online_cpus() + min_vecs)),
		    PCI_IRQ_MSIX | PCI_IRQ_AFFINITY,
		    &desc);

	if (ret < 0) {
		ql_log(ql_log_fatal, vha, 0x00c7,
		    "MSI-X: Failed to enable support, "
		    "giving   up -- %d/%d.\n",
		    ha->msix_count, ret);
		goto msix_out;
	} else if (ret < ha->msix_count) {
		ql_log(ql_log_info, vha, 0x00c6,
		    "MSI-X: Using %d vectors\n", ret);
		ha->msix_count = ret;
		/* Recalculate queue values */
		if (ha->mqiobase && (ql2xmqsupport || ql2xnvmeenable)) {
			ha->max_req_queues = ha->msix_count - 1;

			/* ATIOQ needs 1 vector. That's 1 less QPair */
			if (QLA_TGT_MODE_ENABLED())
				ha->max_req_queues--;

			ha->max_rsp_queues = ha->max_req_queues;

			ha->max_qpairs = ha->max_req_queues - 1;
			ql_dbg_pci(ql_dbg_init, ha->pdev, 0x0190,
			    "Adjusted Max no of queues pairs: %d.\n", ha->max_qpairs);
		}
	}
	vha->irq_offset = desc.pre_vectors;
	ha->msix_entries = kcalloc(ha->msix_count,
				   sizeof(struct qla_msix_entry),
				   GFP_KERNEL);
	if (!ha->msix_entries) {
		ql_log(ql_log_fatal, vha, 0x00c8,
		    "Failed to allocate memory for ha->msix_entries.\n");
		ret = -ENOMEM;
		goto free_irqs;
	}
	ha->flags.msix_enabled = 1;

	for (i = 0; i < ha->msix_count; i++) {
		qentry = &ha->msix_entries[i];
		qentry->vector = pci_irq_vector(ha->pdev, i);
		qentry->vector_base0 = i;
		qentry->entry = i;
		qentry->have_irq = 0;
		qentry->in_use = 0;
		qentry->handle = NULL;
	}

	/* Enable MSI-X vectors for the base queue */
	for (i = 0; i < QLA_BASE_VECTORS; i++) {
		qentry = &ha->msix_entries[i];
		qentry->handle = rsp;
		rsp->msix = qentry;
		scnprintf(qentry->name, sizeof(qentry->name),
		    "qla2xxx%lu_%s", vha->host_no, msix_entries[i].name);
		if (IS_P3P_TYPE(ha))
			ret = request_irq(qentry->vector,
				qla82xx_msix_entries[i].handler,
				0, qla82xx_msix_entries[i].name, rsp);
		else
			ret = request_irq(qentry->vector,
				msix_entries[i].handler,
				0, qentry->name, rsp);
		if (ret)
			goto msix_register_fail;
		qentry->have_irq = 1;
		qentry->in_use = 1;
	}

	/*
	 * If target mode is enable, also request the vector for the ATIO
	 * queue.
	 */
	if (QLA_TGT_MODE_ENABLED() && (ql2xenablemsix != 0) &&
	    IS_ATIO_MSIX_CAPABLE(ha)) {
		qentry = &ha->msix_entries[QLA_ATIO_VECTOR];
		rsp->msix = qentry;
		qentry->handle = rsp;
		scnprintf(qentry->name, sizeof(qentry->name),
		    "qla2xxx%lu_%s", vha->host_no,
		    msix_entries[QLA_ATIO_VECTOR].name);
		qentry->in_use = 1;
		ret = request_irq(qentry->vector,
			msix_entries[QLA_ATIO_VECTOR].handler,
			0, qentry->name, rsp);
		qentry->have_irq = 1;
	}

msix_register_fail:
	if (ret) {
		ql_log(ql_log_fatal, vha, 0x00cb,
		    "MSI-X: unable to register handler -- %x/%d.\n",
		    qentry->vector, ret);
		qla2x00_free_irqs(vha);
		ha->mqenable = 0;
		goto msix_out;
	}

	/* Enable MSI-X vector for response queue update for queue 0 */
	if (IS_MQUE_CAPABLE(ha) && (ha->msixbase && ha->mqiobase && ha->max_qpairs))
		ha->mqenable = 1;
	else
		ha->mqenable = 0;

	ql_dbg(ql_dbg_multiq, vha, 0xc005,
	    "mqiobase=%px, max_rsp_queues=%d, max_req_queues=%d"
	    "mqenable=%d\n", ha->mqiobase, ha->max_rsp_queues,
	    ha->max_req_queues, ha->mqenable);
	ql_dbg(ql_dbg_init, vha, 0x0055,
	    "mqiobase=%px, max_rsp_queues=%d, max_req_queues=%d"
	    "mqenable=%d\n", ha->mqiobase, ha->max_rsp_queues,
	    ha->max_req_queues, ha->mqenable);

msix_out:
	return ret;

free_irqs:
	pci_free_irq_vectors(ha->pdev);
	goto msix_out;
}

int
qla2x00_request_irqs(struct qla_hw_data *ha, struct rsp_que *rsp)
{
	int ret = QLA_FUNCTION_FAILED;
	device_reg_t *reg = ha->iobase;
	scsi_qla_host_t *vha = pci_get_drvdata(ha->pdev);

	/* If possible, enable MSI-X. */
	if (ql2xenablemsix == 0 || (!IS_QLA2432(ha) && !IS_QLA2532(ha) &&
	    !IS_QLA8432(ha) && !IS_CNA_CAPABLE(ha) && !IS_QLA2031(ha) &&
	    !IS_QLAFX00(ha) && !IS_QLA27XX(ha) && !IS_QLA28XX(ha)))
		goto skip_msi;

	if (ql2xenablemsix == 2)
		goto skip_msix;

	if (ha->pdev->subsystem_vendor == PCI_VENDOR_ID_HP &&
		(ha->pdev->subsystem_device == 0x7040 ||
		ha->pdev->subsystem_device == 0x7041 ||
		ha->pdev->subsystem_device == 0x1705)) {
		ql_log(ql_log_warn, vha, 0x0034,
		    "MSI-X: Unsupported ISP 2432 SSVID/SSDID (0x%X,0x%X).\n",
			ha->pdev->subsystem_vendor,
			ha->pdev->subsystem_device);
		goto skip_msi;
	}

	if (ql2xenablemsix == 2)
		goto skip_msix;

	if (IS_QLA2432(ha)) {
		if (ha->pdev->revision < QLA_MSIX_CHIP_REV_24XX) {
			ql_log(ql_log_warn, vha, 0x0035,
			    "MSI-X; Unsupported ISP2432 (0x%X, 0x%X).\n",
			    ha->pdev->revision, QLA_MSIX_CHIP_REV_24XX);
			goto skip_msix;
		}
	}

	ret = qla24xx_enable_msix(ha, rsp);
	if (!ret) {
		ql_dbg(ql_dbg_init, vha, 0x0036,
		    "MSI-X: Enabled (0x%X, 0x%X).\n",
		    ha->chip_revision, ha->fw_attributes);
		goto clear_risc_ints;
	}

skip_msix:

	ql_log(ql_log_info, vha, 0x0037,
	    "Falling back-to MSI mode -- ret=%d.\n", ret);

	if (!IS_QLA24XX(ha) && !IS_QLA2532(ha) && !IS_QLA8432(ha) &&
	    !IS_QLA8001(ha) && !IS_P3P_TYPE(ha) && !IS_QLAFX00(ha) &&
	    !IS_QLA27XX(ha) && !IS_QLA28XX(ha))
		goto skip_msi;

	ret = pci_alloc_irq_vectors(ha->pdev, 1, 1, PCI_IRQ_MSI);
	if (ret > 0) {
		ql_dbg(ql_dbg_init, vha, 0x0038,
		    "MSI: Enabled.\n");
		ha->flags.msi_enabled = 1;
	} else
		ql_log(ql_log_warn, vha, 0x0039,
		    "Falling back-to INTa mode -- ret=%d.\n", ret);
skip_msi:

	/* Skip INTx on ISP82xx. */
	if (!ha->flags.msi_enabled && IS_QLA82XX(ha))
		return QLA_FUNCTION_FAILED;

	ret = request_irq(ha->pdev->irq, ha->isp_ops->intr_handler,
	    ha->flags.msi_enabled ? 0 : IRQF_SHARED,
	    QLA2XXX_DRIVER_NAME, rsp);
	if (ret) {
		ql_log(ql_log_warn, vha, 0x003a,
		    "Failed to reserve interrupt %d already in use.\n",
		    ha->pdev->irq);
		goto fail;
	} else if (!ha->flags.msi_enabled) {
		ql_dbg(ql_dbg_init, vha, 0x0125,
		    "INTa mode: Enabled.\n");
		ha->flags.mr_intr_valid = 1;
		/* Set max_qpair to 0, as MSI-X and MSI in not enabled */
		ha->max_qpairs = 0;
	}

clear_risc_ints:
	if (IS_FWI2_CAPABLE(ha) || IS_QLAFX00(ha))
		goto fail;

	spin_lock_irq(&ha->hardware_lock);
	WRT_REG_WORD(&reg->isp.semaphore, 0);
	spin_unlock_irq(&ha->hardware_lock);

fail:
	return ret;
}

void
qla2x00_free_irqs(scsi_qla_host_t *vha)
{
	struct qla_hw_data *ha = vha->hw;
	struct rsp_que *rsp;
	struct qla_msix_entry *qentry;
	int i;

	/*
	 * We need to check that ha->rsp_q_map is valid in case we are called
	 * from a probe failure context.
	 */
	if (!ha->rsp_q_map || !ha->rsp_q_map[0])
		goto free_irqs;
	rsp = ha->rsp_q_map[0];

	if (ha->flags.msix_enabled) {
		for (i = 0; i < ha->msix_count; i++) {
			qentry = &ha->msix_entries[i];
			if (qentry->have_irq) {
				irq_set_affinity_notifier(qentry->vector, NULL);
				free_irq(pci_irq_vector(ha->pdev, i), qentry->handle);
			}
		}
		kfree(ha->msix_entries);
		ha->msix_entries = NULL;
		ha->flags.msix_enabled = 0;
		ql_dbg(ql_dbg_init, vha, 0x0042,
			"Disabled MSI-X.\n");
	} else {
		free_irq(pci_irq_vector(ha->pdev, 0), rsp);
	}

free_irqs:
	pci_free_irq_vectors(ha->pdev);
}

int qla25xx_request_irq(struct qla_hw_data *ha, struct qla_qpair *qpair,
	struct qla_msix_entry *msix, int vector_type)
{
	const struct qla_init_msix_entry *intr = &msix_entries[vector_type];
	scsi_qla_host_t *vha = pci_get_drvdata(ha->pdev);
	int ret;

	scnprintf(msix->name, sizeof(msix->name),
	    "qla2xxx%lu_qpair%d", vha->host_no, qpair->id);
	ret = request_irq(msix->vector, intr->handler, 0, msix->name, qpair);
	if (ret) {
		ql_log(ql_log_fatal, vha, 0x00e6,
		    "MSI-X: Unable to register handler -- %x/%d.\n",
		    msix->vector, ret);
		return ret;
	}
	msix->have_irq = 1;
	msix->handle = qpair;
	if (!(IS_SCM_CAPABLE(ha) && (qpair->id == ha->slow_queue_id)))
		qla_mapq_init_qp_cpu_map(ha, msix, qpair);

	return ret;
}
