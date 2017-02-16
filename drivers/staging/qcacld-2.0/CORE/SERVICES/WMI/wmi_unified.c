/*
 * Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */


/*
 * Host WMI unified implementation
 */
#include "athdefs.h"
#include "osapi_linux.h"
#include "a_types.h"
#include "a_debug.h"
#include "ol_if_athvar.h"
#include "ol_defines.h"
#include "ol_fw.h"
#include "htc_api.h"
#include "htc_api.h"
#include "dbglog_host.h"
#include "wmi.h"
#include "wmi_unified_priv.h"
#include "wma_api.h"
#include "wma.h"
#include "macTrace.h"
#if defined(HIF_PCI)
#include "if_pci.h"
#elif defined(HIF_USB)
#include "if_usb.h"
#endif

#define WMI_MIN_HEAD_ROOM 64

#ifdef WMI_INTERFACE_EVENT_LOGGING
/* WMI commands */
u_int32_t g_wmi_command_buf_idx = 0;
struct wmi_command_debug wmi_command_log_buffer[WMI_EVENT_DEBUG_MAX_ENTRY];

/* WMI commands TX completed */
u_int32_t g_wmi_command_tx_cmp_buf_idx = 0;
struct wmi_command_debug wmi_command_tx_cmp_log_buffer[WMI_EVENT_DEBUG_MAX_ENTRY];

/* WMI events when processed */
u_int32_t g_wmi_event_buf_idx = 0;
struct wmi_event_debug wmi_event_log_buffer[WMI_EVENT_DEBUG_MAX_ENTRY];

/* WMI events when queued */
u_int32_t g_wmi_rx_event_buf_idx = 0;
struct wmi_event_debug wmi_rx_event_log_buffer[WMI_EVENT_DEBUG_MAX_ENTRY];

#define WMI_COMMAND_RECORD(a, b) {					\
	if (WMI_EVENT_DEBUG_MAX_ENTRY <= g_wmi_command_buf_idx)		\
		g_wmi_command_buf_idx = 0;				\
	wmi_command_log_buffer[g_wmi_command_buf_idx].command = a;	\
	adf_os_mem_copy(wmi_command_log_buffer[g_wmi_command_buf_idx].data, b, 16);\
	wmi_command_log_buffer[g_wmi_command_buf_idx].time =		\
		adf_get_boottime();					\
	g_wmi_command_buf_idx++;					\
}

#define WMI_COMMAND_TX_CMP_RECORD(a, b) {				\
	if (WMI_EVENT_DEBUG_MAX_ENTRY <= g_wmi_command_tx_cmp_buf_idx)	\
		g_wmi_command_tx_cmp_buf_idx = 0;			\
	wmi_command_tx_cmp_log_buffer[g_wmi_command_tx_cmp_buf_idx].command = a;\
	adf_os_mem_copy(wmi_command_tx_cmp_log_buffer			\
		[g_wmi_command_tx_cmp_buf_idx].data, b, 16);		\
	wmi_command_tx_cmp_log_buffer[g_wmi_command_tx_cmp_buf_idx].time =\
		adf_get_boottime();					\
	g_wmi_command_tx_cmp_buf_idx++;					\
}

#define WMI_EVENT_RECORD(a, b) {					\
	if (WMI_EVENT_DEBUG_MAX_ENTRY <= g_wmi_event_buf_idx)		\
		g_wmi_event_buf_idx = 0;				\
	wmi_event_log_buffer[g_wmi_event_buf_idx].event = a;		\
	adf_os_mem_copy(wmi_event_log_buffer[g_wmi_event_buf_idx].data, b, 16);\
	wmi_event_log_buffer[g_wmi_event_buf_idx].time =		\
		adf_get_boottime();					\
	g_wmi_event_buf_idx++;						\
}

#define WMI_RX_EVENT_RECORD(a,b) {					\
	if (WMI_EVENT_DEBUG_MAX_ENTRY <= g_wmi_rx_event_buf_idx)	\
		g_wmi_rx_event_buf_idx = 0;					\
	wmi_rx_event_log_buffer[g_wmi_rx_event_buf_idx].event = a;	\
	adf_os_mem_copy(wmi_rx_event_log_buffer[g_wmi_rx_event_buf_idx].data, b, 16);\
	wmi_rx_event_log_buffer[g_wmi_rx_event_buf_idx].time =		\
		adf_get_boottime();					\
	g_wmi_rx_event_buf_idx++;					\
}

#endif /*WMI_INTERFACE_EVENT_LOGGING*/


static void __wmi_control_rx(struct wmi_unified *wmi_handle, wmi_buf_t evt_buf);
int wmi_get_host_credits(wmi_unified_t wmi_handle);
/* WMI buffer APIs */

/**
 * wmi_get_max_msg_len() - get maximum WMI message length
 * @wmi_handle: WMI handle.
 *
 * This function returns the maximum WMI message length
 *
 * Return: maximum WMI message length
 */
uint16_t wmi_get_max_msg_len(wmi_unified_t wmi_handle)
{
	return wmi_handle->max_msg_len - WMI_MIN_HEAD_ROOM;
}

wmi_buf_t
wmi_buf_alloc(wmi_unified_t wmi_handle, u_int16_t len)
{
	wmi_buf_t wmi_buf;

	if (roundup(len + WMI_MIN_HEAD_ROOM, 4) >
				wmi_handle->max_msg_len) {
		VOS_ASSERT(0);
		return NULL;
	}
	wmi_buf = adf_nbuf_alloc(NULL, roundup(len + WMI_MIN_HEAD_ROOM, 4),
				 WMI_MIN_HEAD_ROOM, 4, FALSE);
	if (!wmi_buf)
		return NULL;

	/* Clear the wmi buffer */
	OS_MEMZERO(adf_nbuf_data(wmi_buf), len);

	/*
	 * Set the length of the buffer to match the allocation size.
	 */
	adf_nbuf_set_pktlen(wmi_buf, len);
	return wmi_buf;
}

#ifdef FEATURE_RUNTIME_PM
inline bool wmi_get_runtime_pm_inprogress(wmi_unified_t wmi_handle)
{
	return adf_os_atomic_read(&wmi_handle->runtime_pm_inprogress);
}
#endif

static uint16_t wmi_tag_vdev_set_cmd(wmi_unified_t wmi_hdl, wmi_buf_t buf)
{
	wmi_vdev_set_param_cmd_fixed_param *set_cmd;

	set_cmd = (wmi_vdev_set_param_cmd_fixed_param *)wmi_buf_data(buf);

	switch(set_cmd->param_id) {
	case WMI_VDEV_PARAM_LISTEN_INTERVAL:
	case WMI_VDEV_PARAM_DTIM_POLICY:
		return HTC_TX_PACKET_TAG_AUTO_PM;
	default:
		break;
	}

	return 0;
}

static uint16_t wmi_tag_sta_powersave_cmd(wmi_unified_t wmi_hdl, wmi_buf_t buf)
{
	wmi_sta_powersave_param_cmd_fixed_param *ps_cmd;

	ps_cmd = (wmi_sta_powersave_param_cmd_fixed_param *)wmi_buf_data(buf);

	switch(ps_cmd->param) {
	case WMI_STA_PS_ENABLE_QPOWER:
		return HTC_TX_PACKET_TAG_AUTO_PM;
	default:
		break;
	}

	return 0;
}

static uint16_t wmi_tag_common_cmd(wmi_unified_t wmi_hdl, wmi_buf_t buf,
				   WMI_CMD_ID cmd_id)
{
	tp_wma_handle wma = wmi_hdl->scn_handle;

	if (adf_os_atomic_read(&wma->is_wow_bus_suspended))
		return 0;

	switch(cmd_id) {
	case WMI_VDEV_SET_PARAM_CMDID:
		return wmi_tag_vdev_set_cmd(wmi_hdl, buf);
	case WMI_STA_POWERSAVE_PARAM_CMDID:
		return wmi_tag_sta_powersave_cmd(wmi_hdl, buf);
	default:
		break;
	}

	return 0;
}

static uint16_t wmi_tag_fw_hang_cmd(wmi_unified_t wmi_handle)
{
	uint16_t tag = 0;

	if (wmi_handle->tag_crash_inject)
		tag = HTC_TX_PACKET_TAG_AUTO_PM;

	wmi_handle->tag_crash_inject = false;
	return tag;
}

/**
 * wmi_set_htc_tx_tag() - set HTC TX tag for WMI commands
 * @wmi_handle: WMI handle
 * @buf: WMI buffer
 * @cmd_id: WMI command Id
 *
 * Return htc_tx_tag
 */
static uint16_t wmi_set_htc_tx_tag(wmi_unified_t wmi_handle,
				wmi_buf_t buf,
				WMI_CMD_ID cmd_id)
{
	uint16_t htc_tx_tag = 0;

	switch(cmd_id) {
	case WMI_WOW_ENABLE_CMDID:
	case WMI_PDEV_SUSPEND_CMDID:
	case WMI_WOW_ENABLE_DISABLE_WAKE_EVENT_CMDID:
	case WMI_WOW_ADD_WAKE_PATTERN_CMDID:
	case WMI_WOW_HOSTWAKEUP_FROM_SLEEP_CMDID:
	case WMI_PDEV_RESUME_CMDID:
	case WMI_WOW_DEL_WAKE_PATTERN_CMDID:
	case WMI_WOW_SET_ACTION_WAKE_UP_CMDID:
#ifdef FEATURE_WLAN_D0WOW
	case WMI_D0_WOW_ENABLE_DISABLE_CMDID:
#endif
		htc_tx_tag = HTC_TX_PACKET_TAG_AUTO_PM;
		break;
	case WMI_FORCE_FW_HANG_CMDID:
		htc_tx_tag = wmi_tag_fw_hang_cmd(wmi_handle);
		break;
	case WMI_VDEV_SET_PARAM_CMDID:
	case WMI_STA_POWERSAVE_PARAM_CMDID:
		htc_tx_tag = wmi_tag_common_cmd(wmi_handle, buf, cmd_id);
	default:
		break;
	}

	return htc_tx_tag;
}

/* WMI command API */
int wmi_unified_cmd_send(wmi_unified_t wmi_handle, wmi_buf_t buf, int len,
			 WMI_CMD_ID cmd_id)
{
	HTC_PACKET *pkt;
	A_STATUS status;
	void *vos_context;
	struct ol_softc *scn;
	A_UINT16 htc_tag = 0;

	if (vos_is_shutdown_in_progress(VOS_MODULE_ID_WDA, NULL)) {
		adf_os_print("\nERROR: %s: shutdown is in progress so could not send WMI command: %d\n",
			__func__, cmd_id);
		return -EBUSY;
	}

	if (wmi_get_runtime_pm_inprogress(wmi_handle))
		goto skip_suspend_check;

	if (adf_os_atomic_read(&wmi_handle->is_target_suspended) &&
			( (WMI_WOW_HOSTWAKEUP_FROM_SLEEP_CMDID != cmd_id) &&
			  (WMI_PDEV_RESUME_CMDID != cmd_id)) ) {
		adf_os_print("\nERROR: %s: Target is suspended  could not send WMI command: %d\n",
				__func__, cmd_id);
		VOS_ASSERT(0);
		return -EBUSY;
	} else
		goto dont_tag;

skip_suspend_check:
	htc_tag = (A_UINT16) wmi_set_htc_tx_tag(wmi_handle,
						buf, cmd_id);

dont_tag:
	/* Do sanity check on the TLV parameter structure */
	{
		void *buf_ptr = (void *) adf_nbuf_data(buf);

		if (wmitlv_check_command_tlv_params(NULL, buf_ptr, len, cmd_id) != 0)
		{
			adf_os_print("\nERROR: %s: Invalid WMI Parameter Buffer for Cmd:%d\n",
				     __func__, cmd_id);
			return -1;
		}
	}

	if (adf_nbuf_push_head(buf, sizeof(WMI_CMD_HDR)) == NULL) {
		pr_err("%s, Failed to send cmd %x, no memory\n",
		       __func__, cmd_id);
		return -ENOMEM;
	}

	WMI_SET_FIELD(adf_nbuf_data(buf), WMI_CMD_HDR, COMMANDID, cmd_id);

	adf_os_atomic_inc(&wmi_handle->pending_cmds);
	if (adf_os_atomic_read(&wmi_handle->pending_cmds) >= WMI_MAX_CMDS) {
		vos_context = vos_get_global_context(VOS_MODULE_ID_WDA, NULL);
		scn = vos_get_context(VOS_MODULE_ID_HIF, vos_context);
		pr_err("\n%s: hostcredits = %d\n", __func__,
		       wmi_get_host_credits(wmi_handle));
		HTC_dump_counter_info(wmi_handle->htc_handle);
		//dump_CE_register(scn);
		//dump_CE_debug_register(scn->hif_sc);
		pr_err("%s: WMI Pending cmds: %d reached MAX: %d\n",
			__func__, adf_os_atomic_read(&wmi_handle->pending_cmds), WMI_MAX_CMDS);
		adf_os_atomic_dec(&wmi_handle->pending_cmds);
		if (scn && scn->enable_self_recovery) {
			if (vos_is_logp_in_progress(VOS_MODULE_ID_VOSS, NULL)) {
				pr_err("%s- %d: SSR is in progress!!!!\n",
					 __func__, __LINE__);
				return -EBUSY;
			}
			vos_trigger_recovery(true);
		} else
			VOS_BUG(0);
		return -EBUSY;
	}

	pkt = adf_os_mem_alloc(NULL, sizeof(*pkt));
	if (!pkt) {
		adf_os_atomic_dec(&wmi_handle->pending_cmds);
		pr_err("%s, Failed to alloc htc packet %x, no memory\n",
		       __func__, cmd_id);
		return -ENOMEM;
	}

	SET_HTC_PACKET_INFO_TX(pkt,
			NULL,
			adf_nbuf_data(buf),
			len + sizeof(WMI_CMD_HDR),
			/* htt_host_data_dl_len(buf)+20 */
			wmi_handle->wmi_endpoint_id,
			htc_tag);

	SET_HTC_PACKET_NET_BUF_CONTEXT(pkt, buf);

	WMA_LOGD("Send WMI command:%s command_id:%d",
			get_wmi_cmd_string(cmd_id), cmd_id);

#ifdef WMI_INTERFACE_EVENT_LOGGING
	adf_os_spin_lock_bh(&wmi_handle->wmi_record_lock);
        /*Record 16 bytes of WMI cmd data - exclude TLV and WMI headers*/
        WMI_COMMAND_RECORD(cmd_id ,((u_int32_t *)adf_nbuf_data(buf) + 2));
	adf_os_spin_unlock_bh(&wmi_handle->wmi_record_lock);
#endif

	status = HTCSendPkt(wmi_handle->htc_handle, pkt);

	if (A_OK != status) {
		adf_os_atomic_dec(&wmi_handle->pending_cmds);
		pr_err("%s %d, HTCSendPkt failed\n", __func__, __LINE__);
	}


	return ((status == A_OK) ? EOK : -1);
}


/* WMI Event handler register API */
int wmi_unified_get_event_handler_ix(wmi_unified_t wmi_handle,
					WMI_EVT_ID event_id)
{
	u_int32_t idx = 0;
	for (idx = 0; (idx < wmi_handle->max_event_idx &&
		idx < WMI_UNIFIED_MAX_EVENT); ++idx) {
		if (wmi_handle->event_id[idx] == event_id &&
			wmi_handle->event_handler[idx] != NULL ) {
			return idx;
		}
	}
	return  -1;
}

int wmi_unified_register_event_handler(wmi_unified_t wmi_handle,
                                       WMI_EVT_ID event_id,
				       wmi_unified_event_handler handler_func)
{
	u_int32_t idx=0;

    if ( wmi_unified_get_event_handler_ix( wmi_handle, event_id) != -1) {
	printk("%s : event handler already registered 0x%x \n",
		__func__, event_id);
        return -1;
    }
    if ( wmi_handle->max_event_idx == WMI_UNIFIED_MAX_EVENT ) {
	printk("%s : no more event handlers 0x%x \n",
                __func__, event_id);
        return -1;
    }
    idx=wmi_handle->max_event_idx;
    wmi_handle->event_handler[idx] = handler_func;
    wmi_handle->event_id[idx] = event_id;
    wmi_handle->max_event_idx++;

    return 0;
}

int wmi_unified_unregister_event_handler(wmi_unified_t wmi_handle,
                                       WMI_EVT_ID event_id)
{
    u_int32_t idx=0;
    if ( (idx = wmi_unified_get_event_handler_ix( wmi_handle, event_id)) == -1) {
        printk("%s : event handler is not registered: event id 0x%x \n",
                __func__, event_id);
        return -1;
    }
    wmi_handle->event_handler[idx] = NULL;
    wmi_handle->event_id[idx] = 0;
    --wmi_handle->max_event_idx;
    wmi_handle->event_handler[idx] = wmi_handle->event_handler[wmi_handle->max_event_idx];
    wmi_handle->event_id[idx]  = wmi_handle->event_id[wmi_handle->max_event_idx] ;
    return 0;
}

#if 0 /* currently not used */
static int wmi_unified_event_rx(struct wmi_unified *wmi_handle,
				wmi_buf_t evt_buf)
{
	u_int32_t id;
	u_int8_t *event;
	u_int16_t len;
	int status = -1;
	u_int32_t idx = 0;

	ASSERT(evt_buf != NULL);

	id = WMI_GET_FIELD(adf_nbuf_data(evt_buf), WMI_CMD_HDR, COMMANDID);

	if (adf_nbuf_pull_head(evt_buf, sizeof(WMI_CMD_HDR)) == NULL)
		goto end;

	idx = wmi_unified_get_event_handler_ix(wmi_handle, id);
	if (idx == -1) {
		pr_err("%s : event handler is not registered: event id: 0x%x\n",
		       __func__, id);
		goto end;
	}

	event = adf_nbuf_data(evt_buf);
	len = adf_nbuf_len(evt_buf);

	/* Call the WMI registered event handler */
	status = wmi_handle->event_handler[idx](wmi_handle->scn_handle,
						event, len);

end:
	adf_nbuf_free(evt_buf);
	return status;
}
#endif /* 0 */

/*
 * Temporarily added to support older WMI events. We should move all events to unified
 * when the target is ready to support it.
 */
void wmi_control_rx(void *ctx, HTC_PACKET *htc_packet)
{
	struct wmi_unified *wmi_handle = (struct wmi_unified *)ctx;
	wmi_buf_t evt_buf;
	u_int32_t len;
	void *wmi_cmd_struct_ptr = NULL;
	u_int32_t idx = 0;
	int tlv_ok_status = 0;

#if  defined(WMI_INTERFACE_EVENT_LOGGING) || !defined(QCA_CONFIG_SMP)
	u_int32_t id;
	u_int8_t *data;
#endif

	evt_buf = (wmi_buf_t) htc_packet->pPktContext;
	id = WMI_GET_FIELD(adf_nbuf_data(evt_buf), WMI_CMD_HDR, COMMANDID);
	/* TX_PAUSE EVENT should be handled with tasklet context */
	if ((WMI_TX_PAUSE_EVENTID == id) ||
		(WMI_WOW_WAKEUP_HOST_EVENTID == id) ||
		(WMI_D0_WOW_DISABLE_ACK_EVENTID == id)) {
		if (adf_nbuf_pull_head(evt_buf, sizeof(WMI_CMD_HDR)) == NULL)
			return;

		data = adf_nbuf_data(evt_buf);
		len = adf_nbuf_len(evt_buf);
		tlv_ok_status = wmitlv_check_and_pad_event_tlvs(
					wmi_handle->scn_handle,
					data, len, id,
					&wmi_cmd_struct_ptr);
		if (tlv_ok_status != 0) {
			WMA_LOGE("Error: id=0x%x, wmitlv_check_and_pad_tlvs ret=%d",
				id, tlv_ok_status);
			return;
		}

		idx = wmi_unified_get_event_handler_ix(wmi_handle, id);
		if (idx == -1) {
			wmitlv_free_allocated_event_tlvs(id,
				&wmi_cmd_struct_ptr);
			adf_nbuf_free(evt_buf);
			return;
		}
		wmi_handle->event_handler[idx](wmi_handle->scn_handle,
			       wmi_cmd_struct_ptr, len);
		wmitlv_free_allocated_event_tlvs(id, &wmi_cmd_struct_ptr);
		adf_nbuf_free(evt_buf);
		return;
	}

#ifdef WMI_INTERFACE_EVENT_LOGGING
	id = WMI_GET_FIELD(adf_nbuf_data(evt_buf), WMI_CMD_HDR, COMMANDID);
	data = adf_nbuf_data(evt_buf);

	adf_os_spin_lock_bh(&wmi_handle->wmi_record_lock);
	/* Exclude 4 bytes of TLV header */
	WMI_RX_EVENT_RECORD(id, ((u_int8_t *)data + 4));
	adf_os_spin_unlock_bh(&wmi_handle->wmi_record_lock);
#endif
	adf_os_spin_lock_bh(&wmi_handle->eventq_lock);
	adf_nbuf_queue_add(&wmi_handle->event_queue, evt_buf);
	adf_os_spin_unlock_bh(&wmi_handle->eventq_lock);
	schedule_work(&wmi_handle->rx_event_work);
}

void __wmi_control_rx(struct wmi_unified *wmi_handle, wmi_buf_t evt_buf)
{
	u_int32_t id;
	u_int8_t *data;
	u_int32_t len;
	void *wmi_cmd_struct_ptr = NULL;
	int tlv_ok_status = 0;

	id = WMI_GET_FIELD(adf_nbuf_data(evt_buf), WMI_CMD_HDR, COMMANDID);

	if (adf_nbuf_pull_head(evt_buf, sizeof(WMI_CMD_HDR)) == NULL)
		goto end;

	data = adf_nbuf_data(evt_buf);
	len = adf_nbuf_len(evt_buf);

	/* Validate and pad(if necessary) the TLVs */
	tlv_ok_status = wmitlv_check_and_pad_event_tlvs(wmi_handle->scn_handle,
							data, len, id,
							&wmi_cmd_struct_ptr);
	if (tlv_ok_status != 0) {
			pr_err("%s: Error: id=0x%d, wmitlv_check_and_pad_tlvs ret=%d\n",
				__func__, id, tlv_ok_status);
			goto end;
	}

#ifdef FEATURE_WLAN_D0WOW
	if (wmi_get_d0wow_flag(wmi_handle))
		pr_debug("%s: WMI event ID is 0x%x\n", __func__, id);
#endif

	/* This event will be earlier than WMI ready. */
	if (id ==  WMI_PDEV_UTF_SCPC_EVENTID) {
		WMA_LOGD("%s:  get WMI_PDEV_UTF_SCPC_EVENTID\n", __func__);
		wma_scpc_event_handler(wmi_handle->scn_handle,
				wmi_cmd_struct_ptr, len);
		goto end;
	}

	if (id >= WMI_EVT_GRP_START_ID(WMI_GRP_START)) {
		u_int32_t idx = 0;

		idx = wmi_unified_get_event_handler_ix(wmi_handle, id) ;
		if (idx == -1) {
			pr_err("%s : event handler is not registered: event id 0x%x\n",
			       __func__, id);
			goto end;
		}

#ifdef WMI_INTERFACE_EVENT_LOGGING
		adf_os_spin_lock_bh(&wmi_handle->wmi_record_lock);
		/* Exclude 4 bytes of TLV header */
		WMI_EVENT_RECORD(id, ((u_int8_t *)data + 4));
		adf_os_spin_unlock_bh(&wmi_handle->wmi_record_lock);
#endif
		/* Call the WMI registered event handler */
		wmi_handle->event_handler[idx](wmi_handle->scn_handle,
					       wmi_cmd_struct_ptr, len);
		goto end;
	}

	switch (id) {
	default:
		pr_info("%s: Unhandled WMI event %d\n", __func__, id);
		break;
	case WMI_SERVICE_READY_EVENTID:
		pr_info("%s: WMI UNIFIED SERVICE READY event\n", __func__);
		wma_rx_service_ready_event(wmi_handle->scn_handle,
					   wmi_cmd_struct_ptr);
		break;
	case WMI_READY_EVENTID:
		pr_info("%s:  WMI UNIFIED READY event\n", __func__);
		wma_rx_ready_event(wmi_handle->scn_handle, wmi_cmd_struct_ptr);
		break;
	}
end:
	wmitlv_free_allocated_event_tlvs(id, &wmi_cmd_struct_ptr);
	adf_nbuf_free(evt_buf);
}

void __wmi_rx_event_work(struct work_struct *work)
{
	struct wmi_unified *wmi = container_of(work, struct wmi_unified,
					       rx_event_work);
	wmi_buf_t buf;

	adf_os_spin_lock_bh(&wmi->eventq_lock);
	buf = adf_nbuf_queue_remove(&wmi->event_queue);
	adf_os_spin_unlock_bh(&wmi->eventq_lock);
	while (buf) {
		__wmi_control_rx(wmi, buf);
		adf_os_spin_lock_bh(&wmi->eventq_lock);
		buf = adf_nbuf_queue_remove(&wmi->event_queue);
		adf_os_spin_unlock_bh(&wmi->eventq_lock);
	}
}

void wmi_rx_event_work(struct work_struct *work)
{
	vos_ssr_protect(__func__);
	__wmi_rx_event_work(work);
	vos_ssr_unprotect(__func__);
}

/* WMI Initialization functions */

void *
wmi_unified_attach(ol_scn_t scn_handle, wma_wow_tx_complete_cbk func)
{
    struct wmi_unified *wmi_handle;
    wmi_handle = (struct wmi_unified *)OS_MALLOC(NULL, sizeof(struct wmi_unified), GFP_ATOMIC);
    if (wmi_handle == NULL) {
        printk("allocation of wmi handle failed %zu \n", sizeof(struct wmi_unified));
        return NULL;
    }
    OS_MEMZERO(wmi_handle, sizeof(struct wmi_unified));
    wmi_handle->scn_handle = scn_handle;
    adf_os_atomic_init(&wmi_handle->pending_cmds);
    adf_os_atomic_init(&wmi_handle->is_target_suspended);
#ifdef FEATURE_RUNTIME_PM
    adf_os_atomic_init(&wmi_handle->runtime_pm_inprogress);
#endif
    adf_os_spinlock_init(&wmi_handle->eventq_lock);
    adf_nbuf_queue_init(&wmi_handle->event_queue);
    vos_init_work(&wmi_handle->rx_event_work, wmi_rx_event_work);
#ifdef WMI_INTERFACE_EVENT_LOGGING
    adf_os_spinlock_init(&wmi_handle->wmi_record_lock);
#endif
    wmi_handle->wma_wow_tx_complete_cbk = func;
    return wmi_handle;
}

void
wmi_unified_detach(struct wmi_unified* wmi_handle)
{
	wmi_buf_t buf;

	vos_flush_work(&wmi_handle->rx_event_work);
	adf_os_spin_lock_bh(&wmi_handle->eventq_lock);
	buf = adf_nbuf_queue_remove(&wmi_handle->event_queue);
	while (buf) {
		adf_nbuf_free(buf);
		buf = adf_nbuf_queue_remove(&wmi_handle->event_queue);
	}
	adf_os_spin_unlock_bh(&wmi_handle->eventq_lock);

	OS_FREE(wmi_handle);
}

/**
 * wmi_unified_remove_work() - detach for WMI work
 * @wmi_handle: handle to WMI
 *
 * A function that does not fully detach WMI, but just remove work
 * queue items associated with it. This is used to make sure that
 * before any other processing code that may destroy related contexts
 * (HTC, etc), work queue processing on WMI has already been stopped.
 *
 * Return: void.
 */
void
wmi_unified_remove_work(struct wmi_unified* wmi_handle)
{
	wmi_buf_t buf;

	VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_INFO,
		"Enter: %s", __func__);
	vos_flush_work(&wmi_handle->rx_event_work);
	adf_os_spin_lock_bh(&wmi_handle->eventq_lock);
	buf = adf_nbuf_queue_remove(&wmi_handle->event_queue);
	while (buf) {
		adf_nbuf_free(buf);
		buf = adf_nbuf_queue_remove(&wmi_handle->event_queue);
	}
	adf_os_spin_unlock_bh(&wmi_handle->eventq_lock);
	VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_INFO,
		"Done: %s", __func__);
}

void wmi_htc_tx_complete(void *ctx, HTC_PACKET *htc_pkt)
{
	struct wmi_unified *wmi_handle = (struct wmi_unified *)ctx;
	wmi_buf_t wmi_cmd_buf = GET_HTC_PACKET_NET_BUF_CONTEXT(htc_pkt);
#ifdef WMI_INTERFACE_EVENT_LOGGING
	u_int32_t cmd_id;
#endif

	ASSERT(wmi_cmd_buf);
#ifdef WMI_INTERFACE_EVENT_LOGGING
	cmd_id = WMI_GET_FIELD(adf_nbuf_data(wmi_cmd_buf),
		WMI_CMD_HDR, COMMANDID);
	adf_os_spin_lock_bh(&wmi_handle->wmi_record_lock);
	/* Record 16 bytes of WMI cmd tx complete data
	   - exclude TLV and WMI headers */
	WMI_COMMAND_TX_CMP_RECORD(cmd_id,
		((u_int32_t *)adf_nbuf_data(wmi_cmd_buf) + 2));
	adf_os_spin_unlock_bh(&wmi_handle->wmi_record_lock);
#endif
	adf_nbuf_free(wmi_cmd_buf);
	adf_os_mem_free(htc_pkt);
	adf_os_atomic_dec(&wmi_handle->pending_cmds);
}

int
wmi_unified_connect_htc_service(struct wmi_unified * wmi_handle, void *htc_handle)
{

    int status;
    HTC_SERVICE_CONNECT_RESP response;
    HTC_SERVICE_CONNECT_REQ connect;

    OS_MEMZERO(&connect, sizeof(connect));
    OS_MEMZERO(&response, sizeof(response));

    /* meta data is unused for now */
    connect.pMetaData = NULL;
    connect.MetaDataLength = 0;
    /* these fields are the same for all service endpoints */
    connect.EpCallbacks.pContext = wmi_handle;
    connect.EpCallbacks.EpTxCompleteMultiple = NULL /* Control path completion ar6000_tx_complete */;
    connect.EpCallbacks.EpRecv = wmi_control_rx /* Control path rx */;
    connect.EpCallbacks.EpRecvRefill = NULL /* ar6000_rx_refill */;
    connect.EpCallbacks.EpSendFull = NULL /* ar6000_tx_queue_full */;
    connect.EpCallbacks.EpTxComplete = wmi_htc_tx_complete /* ar6000_tx_queue_full */;

    /* connect to control service */
    connect.ServiceID = WMI_CONTROL_SVC;

    if ((status = HTCConnectService(htc_handle, &connect, &response)) != EOK)
    {
        printk(" Failed to connect to WMI CONTROL  service status:%d \n",  status);
        return -1;;
    }
    wmi_handle->wmi_endpoint_id = response.Endpoint;
    wmi_handle->htc_handle = htc_handle;
    wmi_handle->max_msg_len = response.MaxMsgLength;

    return EOK;
}

int wmi_get_host_credits(wmi_unified_t wmi_handle)
{
	int host_credits;

	HTCGetControlEndpointTxHostCredits(wmi_handle->htc_handle,
					   &host_credits);
	return host_credits;
}

int wmi_get_pending_cmds(wmi_unified_t wmi_handle)
{
	return adf_os_atomic_read(&wmi_handle->pending_cmds);
}

void wmi_set_target_suspend(wmi_unified_t wmi_handle, A_BOOL val)
{
	adf_os_atomic_set(&wmi_handle->is_target_suspended, val);
}

/**
 * wmi_set_tgt_assert() - set target assert configuration
 * @wmi_handle: Pointer to WMI handle
 * @val: Target assert config value
 *
 * Return: none
 */
void wmi_set_tgt_assert(wmi_unified_t wmi_handle, bool val)
{
	wmi_handle->tgt_force_assert_enable = val;
}

#ifdef FEATURE_RUNTIME_PM
void wmi_set_runtime_pm_inprogress(wmi_unified_t wmi_handle, A_BOOL val)
{
	adf_os_atomic_set(&wmi_handle->runtime_pm_inprogress, val);
}
#endif

#ifdef FEATURE_WLAN_D0WOW
void wmi_set_d0wow_flag(wmi_unified_t wmi_handle, A_BOOL flag)
{
	tp_wma_handle wma = wmi_handle->scn_handle;
	struct ol_softc *scn =
		vos_get_context(VOS_MODULE_ID_HIF, wma->vos_context);

	if (NULL == scn) {
		WMA_LOGE("%s: Failed to get HIF context", __func__);
		return;
	}
	adf_os_atomic_set(&scn->hif_sc->in_d0wow, flag);
}

A_BOOL wmi_get_d0wow_flag(wmi_unified_t wmi_handle)
{
	tp_wma_handle wma = wmi_handle->scn_handle;
	struct ol_softc *scn =
		vos_get_context(VOS_MODULE_ID_HIF, wma->vos_context);

	if (NULL == scn) {
		WMA_LOGE("%s: Failed to get HIF context", __func__);
		return -EINVAL;
	}

	return adf_os_atomic_read(&scn->hif_sc->in_d0wow);
}
#endif

void wmi_tag_crash_inject(wmi_unified_t wmi_handle, A_BOOL flag)
{
	wmi_handle->tag_crash_inject = flag;
}
