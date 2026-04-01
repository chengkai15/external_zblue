/**
 * @file hogp_device.c
 * @brief HID over GATT Profile Device implementation
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/hogp_device.h>

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
#include <zephyr/bluetooth/iso.h>
#endif

#include <syslog.h>

/* ---- Macros ---- */

#define LOG_MODULE_REGISTER(...)
#undef LOG_ERR
#define LOG_ERR(fmt, ...) syslog(3, "[HOGP_DEV] " fmt "\n", ##__VA_ARGS__)
#undef LOG_WRN
#define LOG_WRN(fmt, ...) syslog(4, "[HOGP_DEV] " fmt "\n", ##__VA_ARGS__)
#undef LOG_DBG
#define LOG_DBG(fmt, ...) syslog(7, "[HOGP_DEV] " fmt "\n", ##__VA_ARGS__)

/* TODO: restore _ENCRYPT after SMP DHKey issue is fixed
#define HOGP_DEVICE_PERM_READ  (BT_GATT_PERM_READ_ENCRYPT)
#define HOGP_DEVICE_PERM_WRITE (BT_GATT_PERM_WRITE_ENCRYPT)
*/
#define HOGP_DEVICE_PERM_READ  (BT_GATT_PERM_READ)
#define HOGP_DEVICE_PERM_WRITE (BT_GATT_PERM_WRITE)
#define HOGP_DEVICE_PERM_RW    (HOGP_DEVICE_PERM_READ | HOGP_DEVICE_PERM_WRITE)

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
#define HIDS_FIXED_ATTR_COUNT 14 /* base(9) + SCI Info(2) + SCI Mode(3) */
#else
#define HIDS_FIXED_ATTR_COUNT                                                                      \
	9 /* primary(1) + protocol_mode(2) + report_map(2) + hid_info(2) + ctrl_point(2) */
#endif

#define HOGP_DEVICE_MAX_ATTRS (HIDS_FIXED_ATTR_COUNT + 4 * BT_HOGP_DEVICE_MAX_REPORTS)

/* SCI placeholder UUIDs (pending SIG assignment) */
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
#ifndef BT_UUID_HIDS_SCI_INFO_VAL
#define BT_UUID_HIDS_SCI_INFO_VAL 0x2C10
#endif
#ifndef BT_UUID_HIDS_SCI_MODE_VAL
#define BT_UUID_HIDS_SCI_MODE_VAL 0x2C11
#endif
#define SCI_MODE_NUM_PRESETS      4
#endif

/* HID ISO Service defines */
#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
#define HID_ISO_OPCODE_SELECT_HYBRID  BT_HOGP_ISO_OPCODE_SELECT_HYBRID
#define HID_ISO_OPCODE_SELECT_DEFAULT BT_HOGP_ISO_OPCODE_SELECT_DEFAULT

#define HID_ISO_ERR_OPCODE_OUTSIDE_RANGE  0x81
#define HID_ISO_ERR_ALREADY_IN_STATE     0x82
#define HID_ISO_ERR_UNSUPPORTED_FEATURE  0x83

#define HID_ISO_SELECT_HYBRID_MIN_LEN                                                              \
	8 /* Opcode(1)+CIG(1)+CIS(1)+Interval(2)+SDU_In(1)+SDU_Out(1)+Enable(1) */
#endif /* CONFIG_BT_HOGP_DEVICE_ISO */

#define HID_INFO_BCDHID_OFFSET       0
#define HID_INFO_COUNTRY_CODE_OFFSET 2
#define HID_INFO_FLAGS_OFFSET        3

/* HID Short Item header bit fields (HID 1.11, Section 6.2.2.2) */

/* ---- Types ---- */

#include "hogp_internal.h"

/* ---- Static variables ---- */

static struct bt_uuid_16 uuid_protocol_mode = BT_UUID_INIT_16(BT_UUID_HIDS_PROTOCOL_MODE_VAL);
static struct bt_uuid_16 uuid_report_map = BT_UUID_INIT_16(BT_UUID_HIDS_REPORT_MAP_VAL);
static struct bt_uuid_16 uuid_report = BT_UUID_INIT_16(BT_UUID_HIDS_REPORT_VAL);
static struct bt_uuid_16 uuid_hids_info = BT_UUID_INIT_16(BT_UUID_HIDS_INFO_VAL);
static struct bt_uuid_16 uuid_ctrl_point = BT_UUID_INIT_16(BT_UUID_HIDS_CTRL_POINT_VAL);
static struct bt_uuid_16 uuid_gatt_primary = BT_UUID_INIT_16(BT_UUID_GATT_PRIMARY_VAL);
static struct bt_uuid_16 uuid_gatt_chrc = BT_UUID_INIT_16(BT_UUID_GATT_CHRC_VAL);
static struct bt_uuid_16 uuid_gatt_ccc = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
static struct bt_uuid_16 uuid_sci_info = BT_UUID_INIT_16(BT_UUID_HIDS_SCI_INFO_VAL);
static struct bt_uuid_16 uuid_sci_mode = BT_UUID_INIT_16(BT_UUID_HIDS_SCI_MODE_VAL);

/* SCI UUIDs */

/*
 * SCI mode preset parameters
 *
 * Mapping: Host "high"→FAST(idx1), "medium"→DEFAULT(idx0),
 *          "low"→LOW_POWER(idx2), "auto"→FULL_RANGE(idx3)
 *
 * For 800Hz+ target: FAST uses interval=1~3 (0.125~0.375ms)
 *                    DEFAULT uses interval=10~20 (1.25~2.5ms) → ~125Hz
 */
static const struct bt_conn_le_conn_rate_param sci_mode_params[SCI_MODE_NUM_PRESETS] = {
	/* DEFAULT → "medium": 1.25~2.5ms, ~125Hz mouse rate */
	[0] = {.conn_interval_min = 10,
	       .conn_interval_max = 20,
	       .max_latency = 0,
	       .subrate_min = 1,
	       .subrate_max = 1,
	       .continuation_number = 0,
	       .supervision_timeout = 2000,
	       .min_ce_length = 1,
	       .max_ce_length = 1},
	/* FAST → "high": 1.25ms fixed (matches BES sim reference log), 800Hz */
	[1] = {.conn_interval_min = 10,
	       .conn_interval_max = 10,
	       .max_latency = 0,
	       .subrate_min = 1,
	       .subrate_max = 1,
	       .continuation_number = 0,
	       .supervision_timeout = 800,
	       .min_ce_length = 0,
	       .max_ce_length = 0},
	/* LOW_POWER → "low": 7.5~11.25ms, subrate 2/5, power saving */
	[2] = {.conn_interval_min = 60,
	       .conn_interval_max = 90,
	       .max_latency = 0,
	       .subrate_min = 2,
	       .subrate_max = 5,
	       .continuation_number = 1,
	       .supervision_timeout = 200,
	       .min_ce_length = 1,
	       .max_ce_length = 1},
	/* FULL_RANGE → "auto": 0.375~11.25ms, widest range */
	[3] = {.conn_interval_min = 3,
	       .conn_interval_max = 90,
	       .max_latency = 0,
	       .subrate_min = 1,
	       .subrate_max = 1,
	       .continuation_number = 0,
	       .supervision_timeout = 2000,
	       .min_ce_length = 1,
	       .max_ce_length = 1},
};

static int sci_mode_to_param_index(uint8_t mode)
{
	switch (mode) {
	case BT_HOGP_DEVICE_SCI_MODE_DEFAULT:
		return 0;
	case BT_HOGP_DEVICE_SCI_MODE_FAST:
		return 1;
	case BT_HOGP_DEVICE_SCI_MODE_LOW_POWER:
		return 2;
	case BT_HOGP_DEVICE_SCI_MODE_FULL_RANGE:
		return 3;
	default:
		return -1;
	}
}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

/* ---- Module context ---- */

static const struct bt_hogp_device_cb *callbacks;
static bool conn_cb_inited;
static struct hogp_device_hid_svc hid_svc;
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
static struct hogp_device_sci sci_ctx;
#endif
#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
static struct hogp_device_iso_svc iso_svc;
#endif
static struct hogp_device_conn connections[HOGP_DEVICE_MAX_CONNECTIONS];

/* UUID variables remain separate (needed as pointers in bt_gatt_attr) */

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
/* HID ISO Service */
static struct bt_uuid_16 uuid_hid_iso_svc = BT_UUID_INIT_16(BT_UUID_HID_ISO_SERVICE_VAL);
static struct bt_uuid_16 uuid_hid_iso_props = BT_UUID_INIT_16(BT_UUID_HID_ISO_PROPERTIES_VAL);
static struct bt_uuid_16 uuid_hid_iso_op_mode = BT_UUID_INIT_16(BT_UUID_HID_ISO_OP_MODE_VAL);

/* ISO UUIDs */

/* ISO data channel */
#define ISO_TX_BUF_CNT   10
#define ISO_SDU_MAX_SIZE 48
#define ISO_PKT_HDR_SIZE 3 /* ReportType+ID(1) + SeqNum(1) + Length(1) */

NET_BUF_POOL_FIXED_DEFINE(hogp_iso_tx_pool, ISO_TX_BUF_CNT, BT_ISO_SDU_BUF_SIZE(ISO_SDU_MAX_SIZE),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

/* ISO server is in iso_svc.server */
#endif /* CONFIG_BT_HOGP_DEVICE_ISO */

/* Current combined mode is now per-connection in struct hogp_device_conn */

static struct hogp_device_conn *find_dev_conn(struct bt_conn *conn)
{
	int i;

	for (i = 0; i < HOGP_DEVICE_MAX_CONNECTIONS; i++) {
		if (connections[i].conn == conn) {
			return &connections[i];
		}
	}

	return NULL;
}

/* ---- Internal: helpers ---- */

static int find_report_index(uint8_t report_id, uint8_t report_type)
{
	for (int i = 0; i < hid_svc.reports_cnt; i++) {
		if (hid_svc.reports[i].id == report_id && hid_svc.reports[i].type == report_type) {
			return i;
		}
	}

	return -1;
}

/* ---- HID ISO data channel ---- */

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
/* Forward declarations for ISO helpers */
static struct hogp_device_conn *find_dev_conn_by_iso(struct bt_iso_chan *chan);
static struct hogp_device_conn *find_dev_conn_by_cig_cis(uint8_t cig_id, uint8_t cis_id);

static void iso_connected_cb(struct bt_iso_chan *chan)
{
	struct hogp_device_conn *dc;

	LOG_DBG("ISO CIS connected");
	dc = find_dev_conn_by_iso(chan);
	if (!dc) {
		return;
	}

	dc->iso_cis_connected = true;

	/* Enter Hybrid mode now that CIS is established (Spec 5.2.1) */
	if (dc->iso_op_mode == ISO_OP_MODE_PENDING) {
		uint8_t new_mode = dc->current_mode;

		dc->iso_op_mode = ISO_OP_MODE_HYBRID;
		new_mode = (dc->current_mode & BT_HOGP_MODE_SCI) | BT_HOGP_MODE_ISO;

		if (new_mode != dc->current_mode) {
			dc->current_mode = new_mode;
			if (callbacks && callbacks->mode_changed) {
				callbacks->mode_changed(dc->conn, dc->current_mode);
			}
		}

		memset(dc->iso_report_seq, 0, sizeof(dc->iso_report_seq));
	}
}

static void iso_disconnected_cb(struct bt_iso_chan *chan, uint8_t reason)
{
	struct hogp_device_conn *dc;

	LOG_DBG("ISO CIS disconnected, reason 0x%02x", reason);
	dc = find_dev_conn_by_iso(chan);
	if (dc) {
		dc->iso_cis_connected = false;
	}
}

/**
 * Send Confirmation packet (Spec Table 5.3): Length=0 + SeqNum + ReportID
 */
static void iso_send_confirmation(struct hogp_device_conn *dc, uint8_t seq_num, uint8_t report_id)
{
	LOG_DBG("iso_confirm_tx: id=%u seq=%u", report_id, seq_num);
	struct net_buf *buf;

	if (!dc->iso_cis_connected) {
		return;
	}

	buf = net_buf_alloc(&hogp_iso_tx_pool, K_NO_WAIT);
	if (!buf) {
		return;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_u8(buf, 0); /* Length = 0 → Confirmation */
	net_buf_add_u8(buf, seq_num);
	net_buf_add_u8(buf, report_id);

	if (bt_iso_chan_send(&dc->iso_chan, buf, dc->iso_seq_num++) < 0) {
		net_buf_unref(buf);
	}
}

static void iso_recv_cb(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
			struct net_buf *buf)
{
	struct hogp_device_conn *dc;
	uint8_t length;
	uint8_t seq_num;
	uint8_t report_id;

	dc = find_dev_conn_by_iso(chan);

	if (buf->len < BT_HOGP_ISO_PKT_HDR_SIZE) {
		return;
	}

	length = net_buf_pull_u8(buf);
	seq_num = net_buf_pull_u8(buf);
	report_id = net_buf_pull_u8(buf);

	if (length == 0) {
		LOG_DBG("ISO Confirmation: id=%u seq=%u", report_id, seq_num);
		return;
	}

	if (buf->len < length) {
		return;
	}

	if (callbacks && callbacks->set_report) {
		callbacks->set_report(dc->conn, BT_HOGP_DEVICE_REPORT_TYPE_OUTPUT, report_id,
				      buf->data, length);
	}

	if (dc) {
		iso_send_confirmation(dc, seq_num, report_id);
	}
}

static struct bt_iso_chan_ops iso_chan_ops = {
	.connected = iso_connected_cb,
	.disconnected = iso_disconnected_cb,
	.recv = iso_recv_cb,
};

static struct bt_iso_chan_qos iso_chan_qos;
static struct hogp_device_conn *find_dev_conn_by_iso(struct bt_iso_chan *chan)
{
	int i;

	for (i = 0; i < HOGP_DEVICE_MAX_CONNECTIONS; i++) {
		if (connections[i].conn && &connections[i].iso_chan == chan) {
			return &connections[i];
		}
	}

	return NULL;
}

static struct hogp_device_conn *find_dev_conn_by_cig_cis(uint8_t cig_id, uint8_t cis_id)
{
	int i;

	for (i = 0; i < HOGP_DEVICE_MAX_CONNECTIONS; i++) {
		if (connections[i].conn && connections[i].iso_op_mode == ISO_OP_MODE_PENDING &&
		    connections[i].iso_cig_id == cig_id && connections[i].iso_cis_id == cis_id) {
			return &connections[i];
		}
	}

	/* Fallback: first in-use conn with hybrid pending */
	for (i = 0; i < HOGP_DEVICE_MAX_CONNECTIONS; i++) {
		if (connections[i].conn && connections[i].iso_op_mode == ISO_OP_MODE_PENDING) {
			return &connections[i];
		}
	}

	return NULL;
}

static struct bt_iso_chan_io_qos iso_chan_tx_qos;
static struct bt_iso_chan_io_qos iso_chan_rx_qos;

static int iso_accept_cb(const struct bt_iso_accept_info *info, struct bt_iso_chan **chan)
{
	struct hogp_device_conn *dc;

	LOG_DBG("ISO CIS request: cig_id %u, cis_id %u", info->cig_id, info->cis_id);
	dc = find_dev_conn_by_cig_cis(info->cig_id, info->cis_id);
	if (!dc) {
		LOG_WRN("No conn for CIG %u CIS %u", info->cig_id, info->cis_id);
		return -ENOENT;
	}

	dc->iso_chan.ops = &iso_chan_ops;
	dc->iso_chan.qos = &iso_chan_qos;
	*chan = &dc->iso_chan;
	return 0;
}

/**
 * Send HID ISO Packet over CIS.
 * Spec Table 5.2: Length(1) + SeqNum(1) + ReportID(1) + Report(N)
 */
static int iso_send_report(struct hogp_device_conn *dc, uint8_t report_id, uint8_t report_type,
			   const uint8_t *data, uint16_t len)
{
	LOG_DBG("iso_send: id=%u type=%u len=%u", report_id, report_type, len);
	struct net_buf *buf;
	int rpt_idx;
	uint8_t seq;
	int ret;

	if (!dc->iso_cis_connected) {
		return -ENOTCONN;
	}

	buf = net_buf_alloc(&hogp_iso_tx_pool, K_NO_WAIT);
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	rpt_idx = find_report_index(report_id, report_type);
	seq = (rpt_idx >= 0) ? dc->iso_report_seq[rpt_idx] : 0;

	net_buf_add_u8(buf, (uint8_t)len);
	net_buf_add_u8(buf, seq);
	net_buf_add_u8(buf, report_id);
	net_buf_add_mem(buf, data, len);

	if (rpt_idx >= 0) {
		dc->iso_report_seq[rpt_idx]++;
	}

	ret = bt_iso_chan_send(&dc->iso_chan, buf, dc->iso_seq_num++);
	if (ret < 0) {
		net_buf_unref(buf);
		return ret;
	}

	return 0;
}
#endif /* CONFIG_BT_HOGP_DEVICE_ISO */

/* ---- HID ISO Service GATT handlers ---- */

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
static ssize_t read_iso_properties(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				   uint16_t len, uint16_t offset)
{
	LOG_DBG("read iso_properties (len=%u)", iso_svc.properties_len);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, iso_svc.properties,
				 iso_svc.properties_len);
}

static ssize_t write_iso_op_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *data = buf;
	struct hogp_device_conn *dc;
	uint8_t opcode;
	uint8_t new_mode;

	dc = find_dev_conn(conn);
	if (!dc) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	new_mode = dc->current_mode;

	if (len < 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	opcode = data[0];

	if (opcode == HID_ISO_OPCODE_SELECT_HYBRID) {
		/* Spec Table 6.11: CIG_ID(1)+CIS_ID(1)+Interval(2)+SDU_In(1)+SDU_Out(1)+Enable(1~2)
		 */
		if (len < HID_ISO_SELECT_HYBRID_MIN_LEN) {
			return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
		}

		if (dc->iso_op_mode != ISO_OP_MODE_DEFAULT) {
			return BT_GATT_ERR(HID_ISO_ERR_ALREADY_IN_STATE);
		}

		/* Parse parameters (Spec Table 6.11) */
		dc->iso_cig_id = data[1];
		dc->iso_cis_id = data[2];
		/* data[3..4] = Report Interval bitmask (LE) */
		/* data[5] = Current SDU Size for Input Reports */
		/* data[6] = Current SDU Size for Output Reports */
		/* data[7..] = Hybrid Mode ISO Reports Enable */

		/* Device stays in Default until CIS established (Spec 5.2.1) */
		LOG_DBG("ISO Op Mode: Select Hybrid (cig=%u cis=%u)", dc->iso_cig_id,
			dc->iso_cis_id);
		dc->iso_op_mode = ISO_OP_MODE_PENDING;
	} else if (opcode == HID_ISO_OPCODE_SELECT_DEFAULT) {
		/* Device immediately switches to Default (Spec 5.2.1) */
		dc->iso_op_mode = ISO_OP_MODE_DEFAULT;
		new_mode = dc->current_mode & ~BT_HOGP_MODE_ISO;
		LOG_DBG("ISO Op Mode: Select Default");
	} else {
		return BT_GATT_ERR(HID_ISO_ERR_OPCODE_OUTSIDE_RANGE);
	}

	if (new_mode != dc->current_mode) {
		dc->current_mode = new_mode;
		if (callbacks && callbacks->mode_changed) {
			callbacks->mode_changed(conn, dc->current_mode);
		}
	}

	return len;
}

static int build_hid_iso_attrs(void)
{
	int idx = 0;
	struct bt_gatt_attr *attrs = iso_svc.attrs;

	memset(attrs, 0, sizeof(iso_svc.attrs));

	/* Primary Service */
	attrs[idx++] = (struct bt_gatt_attr)
		BT_GATT_ATTRIBUTE(&uuid_gatt_primary.uuid, BT_GATT_PERM_READ,
				  bt_gatt_attr_read_service, NULL, &uuid_hid_iso_svc);

	/* HID ISO Properties (Read) */
	iso_svc.props_chrc = (struct bt_gatt_chrc)BT_GATT_CHRC_INIT(
		&uuid_hid_iso_props.uuid, 0, BT_GATT_CHRC_READ);
	attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
		&iso_svc.props_chrc);
	attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_hid_iso_props.uuid, HOGP_DEVICE_PERM_READ, read_iso_properties, NULL, NULL);

	/* LE HID Operation Mode (Write + Indicate) */
	iso_svc.op_mode_chrc = (struct bt_gatt_chrc)BT_GATT_CHRC_INIT(
		&uuid_hid_iso_op_mode.uuid, 0, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE);
	attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
		&iso_svc.op_mode_chrc);
	attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_hid_iso_op_mode.uuid, HOGP_DEVICE_PERM_WRITE, NULL, write_iso_op_mode, NULL);

	iso_svc.op_mode_val_attr = &attrs[idx - 1];

	return idx;
}

static void serialize_iso_properties(const struct bt_hogp_device_init_param *param)
{
	LOG_DBG("serialize_iso_properties");
	uint8_t *p = iso_svc.properties;
	uint8_t features = 0;

	if (param->iso_device_mode_change) {
		features |= 0x01;
	}

	*p++ = features;

	sys_put_le16(param->iso_report_intervals, p);
	p += 2;

	*p++ = param->iso_max_sdu_input;
	*p++ = param->iso_preferred_sdu_input;
	*p++ = param->iso_max_sdu_output;
	*p++ = param->iso_preferred_sdu_output;

	/* Hybrid Mode ISO Reports (Spec Table 6.7):
	 * Each struct: ReportID(1) + AdditionalInfo(1)
	 * AdditionalInfo bit0=ReportType(0=Input,1=Output),
	 *               bit1=Confirmation, bit2=Repetition
	 */
	for (int i = 0; i < hid_svc.reports_cnt; i++) {
		if (hid_svc.reports[i].type == BT_HOGP_DEVICE_REPORT_TYPE_INPUT) {
			*p++ = hid_svc.reports[i].id;
			*p++ = 0x00; /* Input, no Confirmation/Repetition */
		}
	}

	iso_svc.properties_len = p - iso_svc.properties;
}
#endif /* CONFIG_BT_HOGP_DEVICE_ISO */

/* ---- GATT callbacks ---- */

static ssize_t read_protocol_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				  uint16_t len, uint16_t offset)
{
	LOG_DBG("read protocol_mode: %u", hid_svc.protocol_mode);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &hid_svc.protocol_mode,
				 sizeof(hid_svc.protocol_mode));
}

static ssize_t write_protocol_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *val = buf;

	if (len != 1 || offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (*val > BT_HOGP_DEVICE_PROTOCOL_REPORT) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	hid_svc.protocol_mode = *val;
	LOG_DBG("write protocol_mode: %u", *val);

	LOG_DBG("hid_svc.protocol_mode set to %u", hid_svc.protocol_mode);

	if (callbacks && callbacks->set_protocol) {
		callbacks->set_protocol(conn, hid_svc.protocol_mode);
	}

	return len;
}

static ssize_t read_report_map(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	LOG_DBG("read report_map (len=%u)", hid_svc.report_map_len);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, hid_svc.report_map,
				 hid_svc.report_map_len);
}

static ssize_t read_hid_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	LOG_DBG("read hid_info");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, hid_svc.hid_info,
				 sizeof(hid_svc.hid_info));
}

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
static ssize_t read_sci_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	LOG_DBG("read sci_info (len=%u)", sci_ctx.properties_len);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, sci_ctx.properties,
				 sci_ctx.properties_len);
}

static ssize_t read_sci_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	LOG_DBG("read sci_mode");
	struct hogp_device_conn *dc = find_dev_conn(conn);
	uint8_t mode = dc ? dc->sci_mode : BT_HOGP_DEVICE_SCI_MODE_NONE;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &mode, sizeof(mode));
}

static void sci_mode_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("SCI Mode CCC changed: %u", value);
}

static void sci_conn_rate_changed(struct bt_conn *conn,
				  const struct bt_conn_le_conn_rate_changed *params)
{
	struct hogp_device_conn *dc;

	dc = find_dev_conn(conn);
	if (!dc || !dc->sci_pending_mode) {
		return;
	}

	if (params->status != 0) {
		LOG_ERR("SCI conn rate change failed: 0x%02x, "
			"attempting conn_param_update fallback",
			params->status);
		/*
		 * BES workaround: 0x20A1 accepted (Command Status 0x00)
		 * but Connection Rate Change event returns 0x1A
		 * (Unsupported Remote Feature). Fall back to standard
		 * connection parameter update.
		 */
		int idx = sci_mode_to_param_index(dc->sci_pending_mode);
		if (idx >= 0) {
			/* Unit conversion: SCI 0.125ms → standard 1.25ms */
			uint16_t fb_min = sci_mode_params[idx].conn_interval_min / 10;
			uint16_t fb_max = sci_mode_params[idx].conn_interval_max / 10;

			if (fb_min < 6) {
				fb_min = 6;
			}
			if (fb_max < fb_min) {
				fb_max = fb_min;
			}

			const struct bt_le_conn_param cp = {
				.interval_min = fb_min,
				.interval_max = fb_max,
				.latency =
					sci_mode_params[idx].max_latency,
				.timeout =
					sci_mode_params[idx].supervision_timeout,
			};
			int fb_err = bt_conn_le_param_update(conn, &cp);
			if (fb_err) {
				LOG_ERR("fallback conn_param_update failed: %d",
					fb_err);
				dc->sci_pending_mode = 0;
				return;
			}
			LOG_WRN("SCI fallback: conn_param_update sent "
				"(interval %u~%u, 1.25ms units)",
				cp.interval_min, cp.interval_max);
			dc->sci_mode = dc->sci_pending_mode;
			dc->sci_pending_mode = 0;
			bt_gatt_notify(conn, NULL,
				       &dc->sci_mode,
				       sizeof(dc->sci_mode));
			dc->current_mode =
				(dc->current_mode & BT_HOGP_MODE_ISO) |
				BT_HOGP_MODE_SCI;
			if (callbacks && callbacks->mode_changed) {
				callbacks->mode_changed(conn,
							dc->current_mode);
			}
		} else {
			dc->sci_pending_mode = 0;
		}
		return;
	}

	dc->sci_mode = dc->sci_pending_mode;
	dc->sci_pending_mode = 0;

	bt_gatt_notify(conn, NULL, &dc->sci_mode, sizeof(dc->sci_mode));

	dc->current_mode = (dc->current_mode & BT_HOGP_MODE_ISO) |
			   (dc->sci_mode != BT_HOGP_DEVICE_SCI_MODE_NONE ? BT_HOGP_MODE_SCI : 0);
	if (callbacks && callbacks->mode_changed) {
		callbacks->mode_changed(conn, dc->current_mode);
	}
}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *val = buf;
	struct hogp_device_conn *dc;
	int err;

	dc = find_dev_conn(conn);

	if (len != 1 || offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	if (*val >= BT_HOGP_DEVICE_CTRL_SCI_DEFAULT && *val <= BT_HOGP_DEVICE_CTRL_SCI_FULL_RANGE) {
		int idx;

		if (!(sci_ctx.modes & HOGP_DEVICE_SCI_MODE_SUPPORTED)) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}

		if (*val == BT_HOGP_DEVICE_CTRL_SCI_LOW_POWER &&
		    !(sci_ctx.modes & HOGP_DEVICE_SCI_MODE_LP_SUPPORTED)) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}

		idx = sci_mode_to_param_index(*val);
		if (idx < 0) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}

		dc->sci_pending_mode = *val;
		err = bt_conn_le_conn_rate_request(conn, &sci_mode_params[idx]);
		if (err) {
			LOG_WRN("SCI conn_rate_request failed (%d), "
				"fallback to conn_param_update", err);
			/*
			 * BES workaround: 0x20A1 may return -EIO or the
			 * Controller event returns 0x1A. Fall back to
			 * standard LE Connection Parameter Update.
			 *
			 * Unit conversion: SCI intervals are in 0.125ms
			 * units, but bt_le_conn_param uses 1.25ms units.
			 * Convert: standard = SCI_val / 10, clamp to
			 * valid range [6..3200] (7.5ms..4s).
			 */
			uint16_t fb_min = sci_mode_params[idx].conn_interval_min / 10;
			uint16_t fb_max = sci_mode_params[idx].conn_interval_max / 10;

			/* Clamp to BLE spec minimum (6 = 7.5ms) */
			if (fb_min < 6) {
				fb_min = 6;
			}
			if (fb_max < fb_min) {
				fb_max = fb_min;
			}

			const struct bt_le_conn_param cp = {
				.interval_min = fb_min,
				.interval_max = fb_max,
				.latency = sci_mode_params[idx].max_latency,
				.timeout = sci_mode_params[idx].supervision_timeout,
			};
			LOG_WRN("SCI fallback: interval %u~%u (1.25ms units)",
				cp.interval_min, cp.interval_max);
			int fb_err = bt_conn_le_param_update(conn, &cp);
			if (fb_err) {
				dc->sci_pending_mode = 0;
				LOG_ERR("fallback conn_param_update also "
					"failed: %d", fb_err);
			} else {
				LOG_WRN("SCI fallback: conn_param_update "
					"sent (interval %u~%u)",
					cp.interval_min, cp.interval_max);
				dc->sci_mode = *val;
				dc->sci_pending_mode = 0;
				dc->current_mode =
					(dc->current_mode &
					 ~BT_HOGP_MODE_SCI) |
					BT_HOGP_MODE_SCI;
				bt_gatt_notify(conn, NULL,
					       &dc->sci_mode,
					       sizeof(dc->sci_mode));
			}
		}

		if (callbacks && callbacks->ctrl_point) {
			callbacks->ctrl_point(conn, *val);
		}

		return len;
	}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

	if (*val > BT_HOGP_DEVICE_CTRL_EXIT_SUSPEND) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (callbacks && callbacks->ctrl_point) {
		callbacks->ctrl_point(conn, *val);
	}

	return len;
}

static ssize_t read_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	LOG_DBG("read report_ref");
	struct hogp_device_report_ref *ref = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, ref, sizeof(*ref));
}

static ssize_t read_report(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	LOG_DBG("read report");
	struct hogp_device_conn *dc = find_dev_conn(conn);
	uint8_t idx = *(uint8_t *)attr->user_data;

	if (dc) {
		dc->pending_report_id = hid_svc.reports[idx].id;
		dc->pending_report_type = hid_svc.reports[idx].type;
	}

	if (callbacks && callbacks->get_report) {
		callbacks->get_report(conn, hid_svc.reports[idx].type, hid_svc.reports[idx].id,
				      len);
	}

	return -EINPROGRESS;
}

static ssize_t write_report(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			    uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("write report (len=%u)", len);
	uint8_t idx = *(uint8_t *)attr->user_data;

	if (callbacks && callbacks->set_report) {
		callbacks->set_report(conn, hid_svc.reports[idx].type, hid_svc.reports[idx].id, buf,
				      len);
	}

	return len;
}

static void report_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	struct _bt_gatt_ccc *ccc = attr->user_data;
	int idx = (int)(ccc - hid_svc.ccc_data);

	syslog(LOG_WARNING, "[HOGP_DEV] report_ccc_changed: idx=%d value=0x%04x attr_handle=0x%04x\n",
	       idx, value, attr->handle);

	if (idx < 0 || idx >= hid_svc.reports_cnt) {
		syslog(LOG_WARNING, "[HOGP_DEV] CCC idx %d out of range (reports_cnt=%d)\n",
		       idx, hid_svc.reports_cnt);
		return;
	}

	if (value & BT_GATT_CCC_NOTIFY) {
		hid_svc.input_ntf_enabled |= BIT(idx);
		syslog(LOG_WARNING, "[HOGP_DEV] report[%d] notify ENABLED, ntf_mask=0x%x\n",
		       idx, hid_svc.input_ntf_enabled);
	} else {
		hid_svc.input_ntf_enabled &= ~BIT(idx);
		syslog(LOG_WARNING, "[HOGP_DEV] report[%d] notify DISABLED, ntf_mask=0x%x\n",
		       idx, hid_svc.input_ntf_enabled);
	}
}

/* ---- Internal: attribute table builder ---- */

static int build_hids_attrs(void)
{
	static struct bt_uuid_16 hids_uuid = BT_UUID_INIT_16(BT_UUID_HIDS_VAL);
	static struct bt_uuid_16 report_ref_uuid = BT_UUID_INIT_16(BT_UUID_HIDS_REPORT_REF_VAL);
	static struct bt_gatt_chrc prot_mode_chrc;
	static struct bt_gatt_chrc report_map_chrc;
	static struct bt_gatt_chrc report_chrcs[BT_HOGP_DEVICE_MAX_REPORTS];
	static struct bt_gatt_chrc hid_info_chrc;
	static struct bt_gatt_chrc ctrl_point_chrc;
	uint16_t count;
	uint16_t idx = 0;
	uint8_t props;
	int i;

	count = HIDS_FIXED_ATTR_COUNT;
	for (i = 0; i < hid_svc.reports_cnt; i++) {
		count += 3;
		if (hid_svc.reports[i].type == BT_HOGP_DEVICE_REPORT_TYPE_INPUT) {
			count++;
		}
	}

	if (count > HOGP_DEVICE_MAX_ATTRS) {
		return -ENOMEM;
	}

	memset(hid_svc.attrs, 0, count * sizeof(struct bt_gatt_attr));

	/* Primary Service */
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)
		BT_GATT_ATTRIBUTE(&uuid_gatt_primary.uuid, BT_GATT_PERM_READ,
				  bt_gatt_attr_read_service, NULL, &hids_uuid);

	/* Protocol Mode */
	prot_mode_chrc = (struct bt_gatt_chrc)BT_GATT_CHRC_INIT(
		&uuid_protocol_mode.uuid, 0, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
		&prot_mode_chrc);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_protocol_mode.uuid, HOGP_DEVICE_PERM_RW, read_protocol_mode,
		write_protocol_mode, NULL);

	/* Report Map */
	report_map_chrc =
		(struct bt_gatt_chrc)BT_GATT_CHRC_INIT(&uuid_report_map.uuid, 0, BT_GATT_CHRC_READ);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
		&report_map_chrc);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_report_map.uuid, HOGP_DEVICE_PERM_READ, read_report_map, NULL, NULL);

	/* Reports */
	for (i = 0; i < hid_svc.reports_cnt; i++) {
		hid_svc.report_indices[i] = i;
		hid_svc.report_refs[i].id = hid_svc.reports[i].id;
		hid_svc.report_refs[i].type = hid_svc.reports[i].type;

		switch (hid_svc.reports[i].type) {
		case BT_HOGP_DEVICE_REPORT_TYPE_INPUT:
			props = BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY;
			break;
		case BT_HOGP_DEVICE_REPORT_TYPE_OUTPUT:
			props = BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
				BT_GATT_CHRC_WRITE_WITHOUT_RESP;
			break;
		case BT_HOGP_DEVICE_REPORT_TYPE_FEATURE:
			props = BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE;
			break;
		default:
			props = BT_GATT_CHRC_READ;
			break;
		}

		report_chrcs[i] =
			(struct bt_gatt_chrc)BT_GATT_CHRC_INIT(&uuid_report.uuid, 0, props);
		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
			&report_chrcs[i]);
		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&uuid_report.uuid, HOGP_DEVICE_PERM_RW, read_report, write_report,
			&hid_svc.report_indices[i]);

		if (hid_svc.reports[i].type == BT_HOGP_DEVICE_REPORT_TYPE_INPUT) {
			memset(&hid_svc.ccc_data[i], 0, sizeof(hid_svc.ccc_data[i]));
			hid_svc.ccc_data[i].cfg_changed = report_ccc_changed;
			hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
				&uuid_gatt_ccc.uuid,
				HOGP_DEVICE_PERM_READ | HOGP_DEVICE_PERM_WRITE,
				bt_gatt_attr_read_ccc, bt_gatt_attr_write_ccc,
				&hid_svc.ccc_data[i]);
		}

		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&report_ref_uuid.uuid, HOGP_DEVICE_PERM_READ, read_report_ref, NULL,
			&hid_svc.report_refs[i]);
	}

	/* HID Information */
	hid_info_chrc =
		(struct bt_gatt_chrc)BT_GATT_CHRC_INIT(&uuid_hids_info.uuid, 0, BT_GATT_CHRC_READ);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
		&hid_info_chrc);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_hids_info.uuid, HOGP_DEVICE_PERM_READ, read_hid_info, NULL, NULL);

	/* HID Control Point */
	ctrl_point_chrc = (struct bt_gatt_chrc)BT_GATT_CHRC_INIT(&uuid_ctrl_point.uuid, 0,
								 BT_GATT_CHRC_WRITE_WITHOUT_RESP);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
		&ctrl_point_chrc);
	hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
		&uuid_ctrl_point.uuid, HOGP_DEVICE_PERM_WRITE, NULL, write_ctrl_point, NULL);

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	if (sci_ctx.modes & HOGP_DEVICE_SCI_MODE_SUPPORTED) {
		static struct bt_gatt_chrc sci_info_chrc;
		static struct bt_gatt_chrc sci_mode_chrc;

		sci_info_chrc = (struct bt_gatt_chrc)BT_GATT_CHRC_INIT(&uuid_sci_info.uuid, 0,
								       BT_GATT_CHRC_READ);
		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
			&sci_info_chrc);
		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&uuid_sci_info.uuid, HOGP_DEVICE_PERM_READ, read_sci_info, NULL, NULL);

		memset(&sci_ctx.mode_ccc, 0, sizeof(sci_ctx.mode_ccc));
		sci_ctx.mode_ccc.cfg_changed = sci_mode_ccc_changed;

		sci_mode_chrc = (struct bt_gatt_chrc)BT_GATT_CHRC_INIT(
			&uuid_sci_mode.uuid, 0, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY);
		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&uuid_gatt_chrc.uuid, BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
			&sci_mode_chrc);
		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&uuid_sci_mode.uuid, HOGP_DEVICE_PERM_READ, read_sci_mode, NULL, NULL);
		hid_svc.attrs[idx++] = (struct bt_gatt_attr)BT_GATT_ATTRIBUTE(
			&uuid_gatt_ccc.uuid,
			HOGP_DEVICE_PERM_READ | HOGP_DEVICE_PERM_WRITE,
			bt_gatt_attr_read_ccc, bt_gatt_attr_write_ccc,
			&sci_ctx.mode_ccc);
	}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

	hid_svc.attr_count = idx;
	LOG_ERR("build_hids_attrs done: %u attrs built", idx);
	return 0;
}

/* ---- Internal: connection management ---- */

static void hogp_dev_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;

	if (err || !callbacks) {
		return;
	}

	if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) {
		return;
	}

	for (int i = 0; i < HOGP_DEVICE_MAX_CONNECTIONS; i++) {
		if (connections[i].conn == NULL) {
			connections[i].conn = bt_conn_ref(conn);
			LOG_DBG("conn slot %d allocated, conn:%p", i, conn);
			if (callbacks && callbacks->connected) {
				callbacks->connected(conn);
			}
			return;
		}
	}

	LOG_ERR("no free conn slot");
}

static void hogp_dev_disconnected(struct bt_conn *conn, uint8_t reason)
{
	int i;

	for (i = 0; i < HOGP_DEVICE_MAX_CONNECTIONS; i++) {
		struct hogp_device_conn *dc = &connections[i];

		if (dc->conn == conn) {
			bt_conn_unref(dc->conn);
			dc->pending_report_type = 0;
			hid_svc.input_ntf_enabled = 0;
#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
			dc->iso_op_mode = ISO_OP_MODE_DEFAULT;
			dc->current_mode = BT_HOGP_MODE_DEFAULT;
			memset(dc->iso_report_seq, 0, sizeof(dc->iso_report_seq));
#endif
			dc->conn = NULL;
			LOG_DBG("conn slot %d freed, reason:0x%02x", i, reason);
			if (callbacks && callbacks->disconnected) {
				callbacks->disconnected(conn, reason);
			}
			return;
		}
	}
}

static struct bt_conn_cb hogp_dev_conn_cb = {
	.connected = hogp_dev_connected,
	.disconnected = hogp_dev_disconnected,
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	.conn_rate_changed = sci_conn_rate_changed,
#endif
};

/* ---- Public API ---- */

int bt_hogp_device_register(const struct bt_hogp_device_init_param *param)
{
	int err;

	if (callbacks) {
		LOG_ERR("HOGP Device already registered");
		return -EALREADY;
	}

	if (!param || !param->report_map || param->report_map_len == 0) {
		return -EINVAL;
	}

	if (!param->reports || param->report_count == 0) {
		return -EINVAL;
	}

	if (param->report_map_len > sizeof(hid_svc.report_map)) {
		return -ENOMEM;
	}

	if (param->report_count > BT_HOGP_DEVICE_MAX_REPORTS) {
		return -ENOMEM;
	}

	/* Store pre-parsed parameters */
	callbacks = param->cb;
	memcpy(hid_svc.report_map, param->report_map, param->report_map_len);
	hid_svc.report_map_len = param->report_map_len;
	hid_svc.reports_cnt = param->report_count;
	memcpy(hid_svc.reports, param->reports,
	       param->report_count * sizeof(struct bt_hogp_device_report));
	memcpy(hid_svc.input_report_sizes, param->report_sizes, param->report_count);

	hid_svc.hid_info[HID_INFO_BCDHID_OFFSET] = param->info.bcd_hid & 0xFF;
	hid_svc.hid_info[HID_INFO_BCDHID_OFFSET + 1] = (param->info.bcd_hid >> 8) & 0xFF;
	hid_svc.hid_info[HID_INFO_COUNTRY_CODE_OFFSET] = param->info.b_country_code;
	hid_svc.hid_info[HID_INFO_FLAGS_OFFSET] = param->info.flags;

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	sci_ctx.modes = (param->sci_supported ? HOGP_DEVICE_SCI_MODE_SUPPORTED : 0) |
			(param->sci_lp_supported ? HOGP_DEVICE_SCI_MODE_LP_SUPPORTED : 0);
	memset(connections, 0, sizeof(connections));

	if (sci_ctx.modes & HOGP_DEVICE_SCI_MODE_SUPPORTED) {

		hid_svc.hid_info[HID_INFO_FLAGS_OFFSET] |= BT_HOGP_DEVICE_FLAG_SCI_SUPPORTED;
		if (sci_ctx.modes & HOGP_DEVICE_SCI_MODE_LP_SUPPORTED) {
			hid_svc.hid_info[HID_INFO_FLAGS_OFFSET] |=
				BT_HOGP_DEVICE_FLAG_SCI_LP_SUPPORTED;
		}

		/*
		 * Skip 0x20A3 (LE_Read_Min_Supported_Connection_Interval)
		 * at register time. BES Controller reports support in
		 * supported_commands but does not respond, causing HCI
		 * command queue to block and CP core crash.
		 *
		 * SCI mode switching uses 0x20A1 which works independently.
		 * SCI properties will be populated later if needed via
		 * LE_Read_All_Remote_Features (0x2088) after connection.
		 */
		syslog(LOG_WARNING,
		       "SCI: skip 0x20A3 at register (BES workaround), hardcode SCI presets\n");

		/*
		 * BES workaround: 0x20A3 is not supported. Hardcode SCI
		 * Info properties so Host can read valid preset groups.
		 *
		 * Format (HOGP SCI Info characteristic):
		 *   uint16_t min_conn_interval (125μs units)
		 *   uint8_t  num_groups
		 *   group[i]:
		 *     uint16_t min_interval
		 *     uint16_t max_interval
		 *     uint16_t stride
		 *
		 * Presets from sci_mode_params[]:
		 *   [0] DEFAULT:    10~20,  stride=1
		 *   [1] FAST:       3~10,   stride=1
		 *   [2] LOW_POWER:  60~90,  stride=1
		 *   [3] FULL_RANGE: 3~90,   stride=1
		 */
		{
			uint8_t *p = sci_ctx.properties;
			/* min_conn_interval = 3 (0.375ms) */
			*p++ = 3; *p++ = 0;
			/* num_groups = 4 */
			*p++ = 4;
			/* Group 0 (DEFAULT): min=10, max=20, stride=1 */
			*p++ = 10; *p++ = 0; *p++ = 20; *p++ = 0; *p++ = 1; *p++ = 0;
			/* Group 1 (FAST): min=3, max=10, stride=1 */
			*p++ = 3; *p++ = 0; *p++ = 10; *p++ = 0; *p++ = 1; *p++ = 0;
			/* Group 2 (LOW_POWER): min=60, max=90, stride=1 */
			*p++ = 60; *p++ = 0; *p++ = 90; *p++ = 0; *p++ = 1; *p++ = 0;
			/* Group 3 (FULL_RANGE): min=3, max=90, stride=1 */
			*p++ = 3; *p++ = 0; *p++ = 90; *p++ = 0; *p++ = 1; *p++ = 0;
			sci_ctx.properties_len = (uint16_t)(p - sci_ctx.properties);
			syslog(LOG_WARNING,
			       "SCI: hardcoded %u bytes SCI Info properties\n",
			       sci_ctx.properties_len);
		}
	}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

	hid_svc.protocol_mode = BT_HOGP_DEVICE_PROTOCOL_REPORT;
	hid_svc.input_ntf_enabled = 0;

	err = build_hids_attrs();
	if (err) {
		LOG_ERR("build_hids_attrs failed: %d", err);
		return err;
	}

	LOG_ERR("build_hids_attrs: attr_count=%u", hid_svc.attr_count);

	hid_svc.svc.attrs = hid_svc.attrs;
	hid_svc.svc.attr_count = hid_svc.attr_count;

	LOG_ERR("HIDS svc before register: attrs=%p count=%u",
		hid_svc.svc.attrs, hid_svc.svc.attr_count);

	err = bt_gatt_service_register(&hid_svc.svc);
	if (err) {
		LOG_ERR("Failed to register HIDS: %d", err);
		return err;
	}

	LOG_ERR("HIDS registered OK: first_handle=0x%04x count=%u",
		hid_svc.svc.attrs ? bt_gatt_attr_get_handle(&hid_svc.svc.attrs[0]) : 0,
		hid_svc.svc.attr_count);

	if (!conn_cb_inited) {
		bt_conn_cb_register(&hogp_dev_conn_cb);
		conn_cb_inited = true;
	}

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
	if (param->iso_enabled) {
		serialize_iso_properties(param);
		int iso_cnt = build_hid_iso_attrs();

		iso_svc.svc.attrs = iso_svc.attrs;
		iso_svc.svc.attr_count = iso_cnt;
		err = bt_gatt_service_register(&iso_svc.svc);
		if (err) {
			LOG_ERR("Failed to register HID ISO Service: %d", err);
		}

		if (!err) {
			iso_svc.registered = true;
			iso_svc.dev_request = param->iso_device_mode_change;
			LOG_DBG("HID ISO Service registered, props_len=%u", iso_svc.properties_len);
		}

		/* Register ISO server to accept CIS from Host */
		iso_chan_tx_qos.sdu = param->iso_max_sdu_input;
		iso_chan_rx_qos.sdu = param->iso_max_sdu_output;
		iso_chan_qos.tx = &iso_chan_tx_qos;
		iso_chan_qos.rx = &iso_chan_rx_qos;

		iso_svc.server.accept = iso_accept_cb;
		int iso_err = bt_iso_server_register(&iso_svc.server);
		if (iso_err) {
			LOG_ERR("Failed to register ISO server: %d", iso_err);
		}
	}
#endif

	LOG_DBG("HOGP Device registered, %d reports", hid_svc.reports_cnt);
	return 0;
}

void bt_hogp_device_unregister(void)
{
	if (!callbacks) {
		return;
	}

	bt_gatt_service_unregister(&hid_svc.svc);
	hid_svc.attr_count = 0;

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
	if (iso_svc.registered) {
		bt_gatt_service_unregister(&iso_svc.svc);
		iso_svc.registered = false;
		iso_svc.properties_len = 0;
	}
#endif

	for (int i = 0; i < HOGP_DEVICE_MAX_CONNECTIONS; i++) {
		if (connections[i].conn) {
			bt_conn_unref(connections[i].conn);
		}
	}

	memset(connections, 0, sizeof(connections));

	callbacks = NULL;
	hid_svc.reports_cnt = 0;
	hid_svc.input_ntf_enabled = 0;
	hid_svc.report_map_len = 0;
	LOG_DBG("HOGP Device unregistered");
}

int bt_hogp_device_connect(const bt_addr_le_t *peer)
{
	LOG_DBG("connect");
	if (!callbacks) {
		return -ESRCH;
	}

	if (!peer) {
		return -EINVAL;
	}

	return bt_le_adv_start(BT_LE_ADV_CONN_DIR_LOW_DUTY(peer), NULL, 0, NULL, 0);
}

int bt_hogp_device_disconnect(struct bt_conn *conn)
{
	LOG_DBG("disconnect");
	if (!conn) {
		return -EINVAL;
	}

	return bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

int bt_hogp_device_send_report(struct bt_conn *conn, uint8_t report_id, const uint8_t *data,
			       uint16_t len)
{
	// LOG_DBG("send_report: id=%u len=%u", report_id, len);
	struct hogp_device_conn *dc;
	uint8_t fallback_id = 0;
	bool found = false;
	int rpt_idx;
	uint16_t i;
	int j;

	if (!callbacks) {
		return -ESRCH;
	}

	dc = find_dev_conn(conn);

	/* Infer report ID when caller passes 0 */
	if (report_id == 0 && hid_svc.reports_cnt > 0) {
		for (j = 0; j < hid_svc.reports_cnt; j++) {
			if (hid_svc.reports[j].type != BT_HOGP_DEVICE_REPORT_TYPE_INPUT) {
				continue;
			}

			if (hid_svc.input_report_sizes[j] == (uint8_t)len) {
				report_id = hid_svc.reports[j].id;
				found = true;
				break;
			}

			if (!fallback_id && hid_svc.reports[j].id != 0) {
				fallback_id = hid_svc.reports[j].id;
			}
		}

		if (!found && fallback_id) {
			report_id = fallback_id;
		}
	}

	rpt_idx = find_report_index(report_id, BT_HOGP_DEVICE_REPORT_TYPE_INPUT);
	if (rpt_idx < 0) {
		return -ENOENT;
	}

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
	/* Hybrid mode: route Input Report through ISO CIS */
	if (dc && dc->iso_op_mode == ISO_OP_MODE_HYBRID && dc->iso_cis_connected) {
		return iso_send_report(dc, report_id, BT_HOGP_DEVICE_REPORT_TYPE_INPUT, data, len);
	}
#endif

	/* Default mode: GATT Notification */

	for (i = 0; i < hid_svc.attr_count; i++) {
		if (hid_svc.attrs[i].read == read_report &&
		    hid_svc.attrs[i].user_data == &hid_svc.report_indices[rpt_idx]) {
			int ret;
			/* send_report log disabled for 800Hz perf */
			ret = bt_gatt_notify(conn, &hid_svc.attrs[i], data, len);
			if (ret) {
				syslog(LOG_WARNING,
				       "[HOGP_DEV] bt_gatt_notify failed: %d\n", ret);
			}
			return ret;
		}
	}

	return -ENOENT;
}

int bt_hogp_device_get_report_response(struct bt_conn *conn, uint8_t report_id, uint8_t report_type,
				       const uint8_t *data, uint16_t len)
{
	LOG_DBG("get_report_response: id=%u type=%u len=%u", report_id, report_type, len);
	struct hogp_device_conn *dc;
	int rpt_idx;
	uint16_t i;

	if (!conn) {
		return -EINVAL;
	}

	dc = find_dev_conn(conn);
	if (dc) {
		dc->pending_report_type = 0;
	}

	rpt_idx = find_report_index(report_id, report_type);
	if (rpt_idx < 0) {
		return -ENOENT;
	}

	for (i = 0; i < hid_svc.attr_count; i++) {
		if (hid_svc.attrs[i].read == read_report &&
		    hid_svc.attrs[i].user_data == &hid_svc.report_indices[rpt_idx]) {
			return bt_gatt_send_read_rsp(conn, 0, hid_svc.attrs[i].handle, data, len);
		}
	}

	return -ENOENT;
}

int bt_hogp_device_report_error(struct bt_conn *conn, uint8_t error)
{
	LOG_DBG("report_error: 0x%02x", error);
	struct hogp_device_conn *dc;
	int rpt_idx;
	uint16_t i;

	if (!conn) {
		return -EINVAL;
	}

	dc = find_dev_conn(conn);
	if (error == 0) {
		return 0;
	}

	if (!dc || !dc->pending_report_type) {
		return -ENOENT;
	}

	rpt_idx = find_report_index(dc->pending_report_id, dc->pending_report_type);
	dc->pending_report_type = 0;
	if (rpt_idx < 0) {
		return -ENOENT;
	}

	for (i = 0; i < hid_svc.attr_count; i++) {
		if (hid_svc.attrs[i].read == read_report &&
		    hid_svc.attrs[i].user_data == &hid_svc.report_indices[rpt_idx]) {
			return bt_gatt_send_read_rsp(conn, -(int)error, hid_svc.attrs[i].handle,
						     NULL, 0);
		}
	}

	return -ENOENT;
}

int bt_hogp_device_virtual_cable_unplug(struct bt_conn *conn)
{
	LOG_DBG("virtual_cable_unplug");
	const bt_addr_le_t *dst;
	bt_addr_le_t peer;

	if (!conn) {
		return -EINVAL;
	}

	dst = bt_conn_get_dst(conn);
	if (!dst) {
		return -ENOENT;
	}

	bt_addr_le_copy(&peer, dst);
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	return bt_unpair(BT_ID_DEFAULT, &peer);
}

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
int bt_hogp_device_request_mode(struct bt_conn *conn, uint8_t mode)
{
	static struct bt_gatt_indicate_params ind_params;
	static uint8_t ind_buf[HID_ISO_SELECT_HYBRID_MIN_LEN];
	uint8_t len;

	if (!conn || !iso_svc.registered || !iso_svc.dev_request) {
		return -ENOTSUP;
	}

	if (!iso_svc.op_mode_val_attr) {
		return -ENOTSUP;
	}

	if (mode & BT_HOGP_MODE_ISO) {
		/* Request Hybrid: Opcode + preferred interval + first Input report */
		ind_buf[0] = HID_ISO_OPCODE_SELECT_HYBRID;
		ind_buf[1] = 0; /* CIG ID (Host decides) */
		ind_buf[2] = 0; /* CIS ID (Host decides) */
		/* Preferred interval: smallest supported */
		sys_put_le16(iso_svc.properties[1] | (iso_svc.properties[2] << 8), &ind_buf[3]);
		ind_buf[5] = iso_svc.properties[3]; /* SDU Input */
		ind_buf[6] = iso_svc.properties[5]; /* SDU Output */
		ind_buf[7] = 0x00;                  /* Enable first report, index 0 */
		len = HID_ISO_SELECT_HYBRID_MIN_LEN;
	} else {
		ind_buf[0] = HID_ISO_OPCODE_SELECT_DEFAULT;
		len = 1;
	}

	memset(&ind_params, 0, sizeof(ind_params));
	ind_params.attr = iso_svc.op_mode_val_attr;
	ind_params.data = ind_buf;
	ind_params.len = len;

	return bt_gatt_indicate(conn, &ind_params);
}
#endif /* CONFIG_BT_HOGP_DEVICE_ISO */
