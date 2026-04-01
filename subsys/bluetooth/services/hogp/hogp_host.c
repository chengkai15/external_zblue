/**
 * @file hogp_host.c
 * @brief HID over GATT Host (HOGP Host / HIDS Client)
 *
 * GATT client state machine for discovering and interacting with
 * remote HID Service (UUID 0x1812).
 */

#include <string.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <syslog.h>

#include <zephyr/bluetooth/services/hogp_host.h>
#include <zephyr/bluetooth/services/hogp_device.h>

#if defined(CONFIG_BT_HOGP_HOST_ISO)
#include <zephyr/bluetooth/iso.h>
#endif

LOG_MODULE_REGISTER(hogp_host, LOG_LEVEL_DBG);

#define HIDS_DBG(fmt, ...) syslog(7, "[BT] hogp_host: " fmt, ##__VA_ARGS__)
#define HIDS_ERR(fmt, ...) syslog(3, "[BT] hogp_host: " fmt, ##__VA_ARGS__)
#define HIDS_WRN(fmt, ...) syslog(4, "[BT] hogp_host: " fmt, ##__VA_ARGS__)

#define PNP_ID_VID_SRC_OFFSET  0
#define PNP_ID_VID_OFFSET      1
#define PNP_ID_PID_OFFSET      3
#define PNP_ID_VERSION_OFFSET  5

#ifndef BT_UUID_HIDS_SCI_INFO_VAL
#define BT_UUID_HIDS_SCI_INFO_VAL 0x2C10
#endif
#ifndef BT_UUID_HIDS_SCI_MODE_VAL
#define BT_UUID_HIDS_SCI_MODE_VAL 0x2C11
#endif

/* ---- Internal types (see hogp_internal.h) ---- */

#include "hogp_internal.h"
/* ---- Globals ---- */

static struct hogp_host_conn g_conns[BT_HOGP_HOST_MAX_CONNECTIONS];
static uint8_t g_desc_storage[BT_HOGP_HOST_DESC_STORAGE_LEN];
static uint16_t g_desc_used;

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
static uint8_t sci_mode_notify_cb(struct bt_conn *conn,
				  struct bt_gatt_subscribe_params *params,
				  const void *data, uint16_t length);
static struct bt_gatt_subscribe_params sci_mode_sub_params;
#endif

#if defined(CONFIG_BT_HOGP_HOST_ISO)
static int hogp_host_iso_send_report(struct hogp_host_conn *c,
				     uint8_t report_id,
				     const uint8_t *data, uint16_t len);
static void hogp_host_iso_disconnect(struct hogp_host_conn *c);
#endif

static const char *state_name(enum hogp_host_state s)
{
	static const char *names[] = {
		[HOGP_HOST_STATE_IDLE] = "IDLE",
		[HOGP_HOST_STATE_DISCOVER_SERVICE] = "DISCOVER_SERVICE",
		[HOGP_HOST_STATE_DISCOVER_CHARS] = "DISCOVER_CHARS",
		[HOGP_HOST_STATE_SET_PROTOCOL_MODE] = "SET_PROTOCOL_MODE",
		[HOGP_HOST_STATE_READ_HID_INFO] = "READ_HID_INFO",
		[HOGP_HOST_STATE_READ_SCI_INFO] = "READ_SCI_INFO",
		[HOGP_HOST_STATE_READ_REPORT_MAP] = "READ_REPORT_MAP",
		[HOGP_HOST_STATE_DISCOVER_REPORT_MAP_DESCS] = "DISCOVER_REPORT_MAP_DESCS",
		[HOGP_HOST_STATE_READ_EXT_REF_UUID] = "READ_EXT_REF_UUID",
		[HOGP_HOST_STATE_DISCOVER_EXT_CHARS] = "DISCOVER_EXT_CHARS",
		[HOGP_HOST_STATE_FIND_REPORT_DESCS] = "FIND_REPORT_DESCS",
		[HOGP_HOST_STATE_READ_REPORT_ID_TYPE] = "READ_REPORT_ID_TYPE",
		[HOGP_HOST_STATE_ENABLE_NOTIFICATIONS] = "ENABLE_NOTIFICATIONS",
		[HOGP_HOST_STATE_DISCOVER_DIS] = "DISCOVER_DIS",
		[HOGP_HOST_STATE_DISCOVER_DIS_CHARS] = "DISCOVER_DIS_CHARS",
		[HOGP_HOST_STATE_READ_PNP_ID] = "READ_PNP_ID",
		[HOGP_HOST_STATE_DISCOVER_BAS] = "DISCOVER_BAS",
		[HOGP_HOST_STATE_DISCOVER_BAS_CHARS] = "DISCOVER_BAS_CHARS",
		[HOGP_HOST_STATE_ENABLE_BAT_NOTIFY] = "ENABLE_BAT_NOTIFY",
		[HOGP_HOST_STATE_CONNECTED] = "CONNECTED",
		[HOGP_HOST_STATE_GET_REPORT] = "GET_REPORT",
		[HOGP_HOST_STATE_SET_REPORT] = "SET_REPORT",
	};

	if (s < ARRAY_SIZE(names) && names[s]) {
		return names[s];
	}

	return "UNKNOWN";
}

/* ---- Forward declarations ---- */

static void next_step(struct hogp_host_conn *c);
static void finish_connected(struct hogp_host_conn *c, int status);
static void advance_to_finish(struct hogp_host_conn *c);
static void advance_to_dis(struct hogp_host_conn *c);

/* ---- Connection lookup ---- */

static struct hogp_host_conn *find_conn(struct bt_conn *conn)
{
	for (int i = 0; i < BT_HOGP_HOST_MAX_CONNECTIONS; i++) {
		if (g_conns[i].conn == conn) {
			return &g_conns[i];
		}
	}

	return NULL;
}

static struct hogp_host_conn *alloc_conn(struct bt_conn *conn)
{
	for (int i = 0; i < BT_HOGP_HOST_MAX_CONNECTIONS; i++) {
		if (g_conns[i].conn == NULL) {
			memset(&g_conns[i], 0, sizeof(g_conns[i]));
			g_conns[i].conn = bt_conn_ref(conn);
			return &g_conns[i];
		}
	}

	return NULL;
}

static void free_conn(struct hogp_host_conn *c)
{
	if (c->conn) {
		/* Unsubscribe all notifications */
		for (uint8_t i = 0; i < c->num_reports; i++) {
			if (c->reports[i].sub_params.value_handle) {
				bt_gatt_unsubscribe(c->conn, &c->reports[i].sub_params);
			}
		}

		for (uint8_t i = 0; i < c->num_bats; i++) {
			if (c->bats[i].sub_params.value_handle) {
				bt_gatt_unsubscribe(c->conn, &c->bats[i].sub_params);
			}
		}

		bt_conn_unref(c->conn);
	}
	/* Free descriptor storage used by this connection */
	/* Simple approach: just mark freed, compaction not needed for few conns */
	c->conn = NULL;
	c->state = HOGP_HOST_STATE_IDLE;
}

/* ---- Descriptor storage ---- */

static int desc_store_byte(struct hogp_host_conn *c, uint8_t byte)
{
	if (g_desc_used >= BT_HOGP_HOST_DESC_STORAGE_LEN) {
		return -ENOMEM;
	}

	g_desc_storage[g_desc_used++] = byte;
	c->services[c->svc_idx].desc_len++;
	return 0;
}

/* ---- Report helpers ---- */

static uint8_t add_report(struct hogp_host_conn *c, uint16_t value_handle,
			  uint16_t end_handle, uint8_t properties,
			  uint8_t report_id, uint8_t report_type, bool boot)
{
	uint8_t idx;
	struct hogp_host_report *r;

	if (c->num_reports >= BT_HOGP_HOST_MAX_REPORTS) {
		HIDS_WRN("Too many reports, increase BT_HOGP_HOST_MAX_REPORTS");
		return UINT8_MAX;
	}

	idx = c->num_reports++;
	r = &c->reports[idx];

	r->value_handle = value_handle;
	r->end_handle = end_handle;
	r->properties = properties;
	r->service_index = c->svc_idx;
	r->report_id = report_id;
	r->report_type = report_type;
	r->boot_report = boot;
	r->ccc_handle = 0;
	return idx;
}

static struct hogp_host_report *find_report(struct hogp_host_conn *c,
					    uint8_t report_id,
					    uint8_t report_type)
{
	for (uint8_t i = 0; i < c->num_reports; i++) {
		struct hogp_host_report *r = &c->reports[i];
		uint8_t pm = c->services[r->service_index].protocol_mode;

		if (pm == BT_HOGP_HOST_PROTOCOL_BOOT && !r->boot_report) {
			continue;
		}

		if (pm == BT_HOGP_HOST_PROTOCOL_REPORT && r->boot_report) {
			continue;
		}

		if (r->report_id == report_id && r->report_type == report_type) {
			return r;
		}
	}

	return NULL;
}

static struct hogp_host_report *find_report_by_handle(struct hogp_host_conn *c,
						      uint16_t handle)
{
	for (uint8_t i = 0; i < c->num_reports; i++) {
		if (c->reports[i].value_handle == handle) {
			return &c->reports[i];
		}
	}

	return NULL;
}

/* ---- Notification handler ---- */

static uint8_t notify_cb(struct bt_conn *conn,
			 struct bt_gatt_subscribe_params *params,
			 const void *data, uint16_t length)
{
	struct hogp_host_conn *c;
	struct hogp_host_report *r;

	if (!data) {
		HIDS_DBG("notify_cb: unsubscribed handle=0x%04x", params->value_handle);
		return BT_GATT_ITER_STOP;
	}

	c = find_conn(conn);

	if (!c || !c->cb || !c->cb->input_report) {
		return BT_GATT_ITER_CONTINUE;
	}

	r = find_report_by_handle(c, params->value_handle);
	if (!r) {
		HIDS_WRN("notify_cb: no report for handle 0x%04x", params->value_handle);
		return BT_GATT_ITER_CONTINUE;
	}

	// HIDS_DBG("notify_cb: handle=0x%04x svc=%u id=%u len=%u",
//		params->value_handle, r->service_index, r->report_id, length);

	c->cb->input_report(conn, r->service_index, r->report_id, data, length);
	return BT_GATT_ITER_CONTINUE;
}

/* ---- Battery notification handler ---- */

static uint8_t bat_notify_cb(struct bt_conn *conn,
			     struct bt_gatt_subscribe_params *params,
			     const void *data, uint16_t length)
{
	struct hogp_host_conn *c;

	if (!data) {
		return BT_GATT_ITER_STOP;
	}

	c = find_conn(conn);

	if (!c || !c->cb || !c->cb->battery_level || length < 1) {
		return BT_GATT_ITER_CONTINUE;
	}

	for (uint8_t i = 0; i < c->num_bats; i++) {
		if (c->bats[i].sub_params.value_handle == params->value_handle) {
			c->cb->battery_level(conn, i, ((const uint8_t *)data)[0]);
			break;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

/* ---- State machine: iteration helpers ---- */

/* Find next Input Report for notification subscription */
static int next_input_report_idx(struct hogp_host_conn *c, uint8_t start)
{
	for (uint8_t i = start; i < c->num_reports; i++) {
		struct hogp_host_report *r = &c->reports[i];
		uint8_t pm = c->services[r->service_index].protocol_mode;

		if (pm == BT_HOGP_HOST_PROTOCOL_BOOT && !r->boot_report) {
			continue;
		}

		if (pm == BT_HOGP_HOST_PROTOCOL_REPORT && r->boot_report) {
			continue;
		}

		if (r->report_type == BT_HOGP_HOST_REPORT_TYPE_INPUT) {
			return i;
		}
	}

	return -1;
}

/* Find next non-boot Report char that needs Report Reference discovery */
static int next_report_char_idx(struct hogp_host_conn *c, uint8_t start)
{
	for (uint8_t i = start; i < c->num_reports; i++) {
		if (!c->reports[i].boot_report &&
		    c->reports[i].report_type == 0) {
			/* report_type==0 means not yet resolved */
			return i;
		}
	}

	return -1;
}

/* ---- State machine: GATT callbacks ---- */

/* Service discovery callback */
static uint8_t discover_service_cb(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr,
				   struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		/* Discovery complete */
		HIDS_DBG("discover_service_cb: complete, instances=%u", c->num_instances);
		if (c->num_instances == 0) {
			finish_connected(c, -ENOENT);
			return BT_GATT_ITER_STOP;
		}

		c->svc_idx = 0;
		c->state = HOGP_HOST_STATE_DISCOVER_CHARS;
		next_step(c);
		return BT_GATT_ITER_STOP;
	}

	if (c->num_instances < BT_HOGP_HOST_MAX_SERVICES) {
		struct bt_gatt_service_val *sv = attr->user_data;
		uint8_t idx = c->num_instances++;

		c->services[idx].start_handle = attr->handle;
		c->services[idx].end_handle = sv->end_handle;
		c->services[idx].desc_offset = g_desc_used;
		c->services[idx].desc_len = 0;
		HIDS_DBG("HID Service[%u]: 0x%04x-0x%04x", idx,
			attr->handle, sv->end_handle);
	}

	return BT_GATT_ITER_CONTINUE;
}

/* Characteristic discovery callback */
static uint8_t discover_chars_cb(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	struct bt_gatt_chrc *chrc;
	uint16_t uuid16;

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		/* Done with this service's chars */
		if ((c->svc_idx + 1) < c->num_instances) {
			c->svc_idx++;
			c->state = HOGP_HOST_STATE_DISCOVER_CHARS;
			next_step(c);
		} else {
			/* All services' chars discovered */
			if (c->required_protocol_mode == BT_HOGP_HOST_PROTOCOL_BOOT) {
				c->svc_idx = 0;
				c->state = HOGP_HOST_STATE_SET_PROTOCOL_MODE;
				next_step(c);
			} else {
				/* Report mode: set protocol_mode field, go read HID Info */
				for (uint8_t i = 0; i < c->num_instances; i++) {
					c->services[i].protocol_mode =
						BT_HOGP_HOST_PROTOCOL_REPORT;
				}

				c->svc_idx = 0;
				c->state = HOGP_HOST_STATE_READ_HID_INFO;
				next_step(c);
			}
		}

		return BT_GATT_ITER_STOP;
	}

	chrc = attr->user_data;
	uuid16 = BT_UUID_16(chrc->uuid)->val;

	HIDS_DBG("discover_chars_cb: svc[%u] uuid=0x%04x handle=0x%04x props=0x%02x",
		c->svc_idx, uuid16, chrc->value_handle, chrc->properties);

	switch (uuid16) {
	case BT_UUID_HIDS_PROTOCOL_MODE_VAL:
		c->services[c->svc_idx].protocol_mode_handle = chrc->value_handle;
		break;
	case BT_UUID_HIDS_BOOT_KB_IN_REPORT_VAL:
		add_report(c, chrc->value_handle, 0, chrc->properties,
			   1, BT_HOGP_HOST_REPORT_TYPE_INPUT, true);
		break;
	case BT_UUID_HIDS_BOOT_MOUSE_IN_REPORT_VAL:
		add_report(c, chrc->value_handle, 0, chrc->properties,
			   2, BT_HOGP_HOST_REPORT_TYPE_INPUT, true);
		break;
	case BT_UUID_HIDS_BOOT_KB_OUT_REPORT_VAL:
		add_report(c, chrc->value_handle, 0, chrc->properties,
			   1, BT_HOGP_HOST_REPORT_TYPE_OUTPUT, true);
		break;
	case BT_UUID_HIDS_REPORT_VAL:
		/* report_type=0 means unresolved, will be read from Report Reference */
		add_report(c, chrc->value_handle, 0, chrc->properties,
			   0, 0, false);
		break;
	case BT_UUID_HIDS_REPORT_MAP_VAL:
		c->services[c->svc_idx].report_map_handle = chrc->value_handle;
		break;
	case BT_UUID_HIDS_INFO_VAL:
		c->services[c->svc_idx].hid_info_handle = chrc->value_handle;
		break;
	case BT_UUID_HIDS_CTRL_POINT_VAL:
		c->services[c->svc_idx].ctrl_point_handle = chrc->value_handle;
		break;
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	case BT_UUID_HIDS_SCI_INFO_VAL:
		c->services[c->svc_idx].sci_info_handle = chrc->value_handle;
		break;
	case BT_UUID_HIDS_SCI_MODE_VAL:
		c->services[c->svc_idx].sci_mode_handle = chrc->value_handle;
		break;
#endif
	default:
		break;
	}

	/* Track end_handle for the last added report in this service */
	if (c->num_reports > 0) {
		struct hogp_host_report *last = &c->reports[c->num_reports - 1];
		if (last->end_handle == 0 && last->service_index == c->svc_idx) {
			/* Will be updated when next char is found or service ends */
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

/* HID Information read callback */
static uint8_t read_hid_info_cb(struct bt_conn *conn, uint8_t err,
				struct bt_gatt_read_params *params,
				const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!err && data && length >= 4) {
		const uint8_t *v = data;
		struct hogp_host_hid_svc *svc = &c->services[c->svc_idx];
		svc->bcd_hid = v[0] | (v[1] << 8);
		svc->b_country_code = v[2];
		svc->hid_flags = v[3];
		HIDS_DBG("HID Info[%u]: bcdHID=0x%04x country=%u flags=0x%02x",
			c->svc_idx, svc->bcd_hid, svc->b_country_code,
			svc->hid_flags);
	}

	/* Next service's HID Info */
	c->svc_idx++;
	while (c->svc_idx < c->num_instances) {
		if (c->services[c->svc_idx].hid_info_handle) {
			c->state = HOGP_HOST_STATE_READ_HID_INFO;
			next_step(c);
			return BT_GATT_ITER_STOP;
		}

		c->svc_idx++;
	}

	/* All HID Info read — check SCI support */
	c->svc_idx = 0;
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	{
		bool has_sci = false;
		uint8_t i;

		for (i = 0; i < c->num_instances; i++) {
			if ((c->services[i].hid_flags & BT_HOGP_DEVICE_FLAG_SCI_SUPPORTED) &&
			    c->services[i].sci_info_handle) {
				c->services[i].sci_supported = true;
				has_sci = true;
			}
		}

		if (has_sci) {
			c->state = HOGP_HOST_STATE_READ_SCI_INFO;
			next_step(c);
			return BT_GATT_ITER_STOP;
		}
	}
#endif
	c->state = HOGP_HOST_STATE_READ_REPORT_MAP;
	next_step(c);
	return BT_GATT_ITER_STOP;
}

/* Report Map read callback (supports long read) */
static uint8_t read_report_map_cb(struct bt_conn *conn, uint8_t err,
				  struct bt_gatt_read_params *params,
				  const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);
	struct hogp_host_hid_svc *svc;

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (err) {
		HIDS_ERR("Report map read error: %u", err);
		finish_connected(c, -EIO);
		return BT_GATT_ITER_STOP;
	}

	HIDS_DBG("read_report_map_cb: err=%u data=%p len=%u svc_idx=%u",
		err, data, length, c->svc_idx);

	if (data && length > 0) {
		const uint8_t *bytes = data;
		for (uint16_t i = 0; i < length; i++) {
			if (desc_store_byte(c, bytes[i]) < 0) {
				HIDS_WRN("Descriptor storage full");
				break;
			}
		}
		/* Continue long read */
		return BT_GATT_ITER_CONTINUE;
	}

	/* data==NULL: read complete for this service */
	svc = &c->services[c->svc_idx];

	HIDS_DBG("Report map[%u]: %u bytes", c->svc_idx, svc->desc_len);

	/* Notify report map */
	if (c->cb && c->cb->report_map) {
		c->cb->report_map(conn, c->svc_idx,
				  &g_desc_storage[svc->desc_offset],
				  svc->desc_len);
	}

	/* Next service's report map */
	c->svc_idx++;
	while (c->svc_idx < c->num_instances) {
		if (c->services[c->svc_idx].report_map_handle) {
			c->state = HOGP_HOST_STATE_READ_REPORT_MAP;
			next_step(c);
			return BT_GATT_ITER_STOP;
		}

		c->svc_idx++;
	}

	/* All report maps read. Discover External Report Reference descriptors. */
	c->svc_idx = 0;
	c->state = HOGP_HOST_STATE_DISCOVER_REPORT_MAP_DESCS;
	HIDS_DBG("read_report_map_cb: all maps read, -> DISCOVER_REPORT_MAP_DESCS");
	next_step(c);
	return BT_GATT_ITER_STOP;
}

/* External Report Reference descriptor discovery callback */
static uint8_t discover_ext_ref_cb(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr,
				   struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	uint16_t uuid16;

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		/* Next service */
		c->svc_idx++;
		while (c->svc_idx < c->num_instances) {
			if (c->services[c->svc_idx].report_map_handle) {
				c->state = HOGP_HOST_STATE_DISCOVER_REPORT_MAP_DESCS;
				next_step(c);
				return BT_GATT_ITER_STOP;
			}

			c->svc_idx++;
		}
		/* All done, proceed to Report Reference discovery */
		int idx = next_report_char_idx(c, 0);
		if (idx >= 0) {
			c->rpt_idx = idx;
			c->state = HOGP_HOST_STATE_FIND_REPORT_DESCS;
			next_step(c);
		} else {

			idx = next_input_report_idx(c, 0);
			if (idx >= 0) {
				c->rpt_idx = idx;
				c->state = HOGP_HOST_STATE_ENABLE_NOTIFICATIONS;
				next_step(c);
			} else {

				advance_to_dis(c);
			}
		}

		return BT_GATT_ITER_STOP;
	}

	uuid16 = BT_UUID_16(attr->uuid)->val;
	if (uuid16 == BT_UUID_HIDS_EXT_REPORT_VAL &&
	    c->num_ext_reports < BT_HOGP_HOST_MAX_REPORTS) {
		struct hogp_host_ext_report *er =
			&c->ext_reports[c->num_ext_reports++];
		er->desc_handle = attr->handle;
		er->service_index = c->svc_idx;
		HIDS_DBG("Ext Report Ref desc at 0x%04x svc[%u]",
			attr->handle, c->svc_idx);
	}

	return BT_GATT_ITER_CONTINUE;
}

/* Descriptor discovery for Report Reference */
static uint8_t discover_report_desc_cb(struct bt_conn *conn,
				       const struct bt_gatt_attr *attr,
				       struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	uint16_t uuid16;

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		/* Done discovering descriptors for this report char */
		if (c->tmp_handle != 0) {
			/* Found Report Reference, read it */
			c->state = HOGP_HOST_STATE_READ_REPORT_ID_TYPE;
			next_step(c);
		} else {
			/* No Report Reference found, try next report */
			int idx = next_report_char_idx(c, c->rpt_idx + 1);
			if (idx >= 0) {
				c->rpt_idx = idx;
				c->state = HOGP_HOST_STATE_FIND_REPORT_DESCS;
				next_step(c);
			} else {
				/* All done, enable notifications */
				int nidx = next_input_report_idx(c, 0);
				if (nidx >= 0) {
					c->rpt_idx = nidx;
					c->state = HOGP_HOST_STATE_ENABLE_NOTIFICATIONS;
					next_step(c);
				} else {

					advance_to_dis(c);
				}
			}
		}

		return BT_GATT_ITER_STOP;
	}

	uuid16 = BT_UUID_16(attr->uuid)->val;

	if (uuid16 == BT_UUID_HIDS_REPORT_REF_VAL) {
		c->tmp_handle = attr->handle;
		HIDS_DBG("Report Reference desc at 0x%04x for report[%u]",
			attr->handle, c->rpt_idx);
	} else if (uuid16 == BT_UUID_GATT_CCC_VAL) {
		HIDS_DBG("CCC desc at 0x%04x for report[%u] (current=0x%04x)",
			attr->handle, c->rpt_idx, c->reports[c->rpt_idx].ccc_handle);
		/* Only use the first CCC found for this report;
		 * later CCC descriptors belong to other characteristics
		 * (e.g. SCI Mode) sharing the same HIDS service range.
		 */
		if (c->reports[c->rpt_idx].ccc_handle == 0) {
			c->reports[c->rpt_idx].ccc_handle = attr->handle;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

/* Read Report Reference descriptor result */
static uint8_t read_report_ref_cb(struct bt_conn *conn, uint8_t err,
				  struct bt_gatt_read_params *params,
				  const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);
	int idx;

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!err && data && length >= 2) {
		const uint8_t *val = data;

		c->reports[c->rpt_idx].report_id = val[0];
		c->reports[c->rpt_idx].report_type = val[1];
		HIDS_DBG("Report[%u]: id=%u type=%u", c->rpt_idx, val[0], val[1]);
	}

	/* Next report char */
	idx = next_report_char_idx(c, c->rpt_idx + 1);
	if (idx >= 0) {
		c->rpt_idx = idx;
		c->state = HOGP_HOST_STATE_FIND_REPORT_DESCS;
		next_step(c);
	} else {
		/* Enable notifications */
		int nidx = next_input_report_idx(c, 0);
		if (nidx >= 0) {
			c->rpt_idx = nidx;
			c->state = HOGP_HOST_STATE_ENABLE_NOTIFICATIONS;
			next_step(c);
		} else {

			advance_to_dis(c);
		}
	}

	return BT_GATT_ITER_STOP;
}

/* Subscribe callback */
static void subscribe_cb(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_subscribe_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	int idx;

	if (!c) {
		return;
	}

	if (err) {
		HIDS_WRN("Subscribe failed for handle 0x%04x: %u",
			params->value_handle, err);
	}

	/* Next input report */
	idx = next_input_report_idx(c, c->rpt_idx + 1);
	if (idx >= 0) {
		c->rpt_idx = idx;
		c->state = HOGP_HOST_STATE_ENABLE_NOTIFICATIONS;
		next_step(c);
	} else {

		advance_to_dis(c);
	}
}

/* ---- DIS discovery callback ---- */

static uint8_t discover_dis_cb(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	struct bt_gatt_service_val *sv;

	HIDS_DBG("discover_dis_cb: conn=%p attr=%p c=%p", conn, attr, c);
	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		if (c->dis_start_handle) {
			c->state = HOGP_HOST_STATE_DISCOVER_DIS_CHARS;
		} else {

			c->state = HOGP_HOST_STATE_DISCOVER_BAS;
		}

		next_step(c);
		return BT_GATT_ITER_STOP;
	}

	sv = attr->user_data;
	c->dis_start_handle = attr->handle;
	c->dis_end_handle = sv->end_handle;
	HIDS_DBG("DIS: 0x%04x-0x%04x", attr->handle, sv->end_handle);

	/* Advance immediately — ITER_STOP does not trigger a NULL callback */
	c->state = HOGP_HOST_STATE_DISCOVER_DIS_CHARS;
	next_step(c);
	return BT_GATT_ITER_STOP; /* only one DIS instance */
}

/* DIS characteristic discovery callback */
static uint8_t discover_dis_chars_cb(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	struct bt_gatt_chrc *chrc;

	HIDS_DBG("discover_dis_chars_cb: conn=%p attr=%p c=%p", conn, attr, c);
	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		if (c->pnp_id_handle) {
			c->state = HOGP_HOST_STATE_READ_PNP_ID;
		} else {

			c->state = HOGP_HOST_STATE_DISCOVER_BAS;
		}

		next_step(c);
		return BT_GATT_ITER_STOP;
	}

	chrc = attr->user_data;
	if (BT_UUID_16(chrc->uuid)->val == BT_UUID_DIS_PNP_ID_VAL) {
		c->pnp_id_handle = chrc->value_handle;
	}

	return BT_GATT_ITER_CONTINUE;
}

/* PnP ID read callback */
static uint8_t read_pnp_id_cb(struct bt_conn *conn, uint8_t err,
			       struct bt_gatt_read_params *params,
			       const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!err && data && length >= 7 && c->cb && c->cb->pnp_id) {
		const uint8_t *v = data;
		struct bt_hogp_host_pnp_id id;
		id.vid_src = v[PNP_ID_VID_SRC_OFFSET];
		id.vid = v[PNP_ID_VID_OFFSET] | (v[PNP_ID_VID_OFFSET + 1] << 8);
		id.pid = v[PNP_ID_PID_OFFSET] | (v[PNP_ID_PID_OFFSET + 1] << 8);
		id.version = v[PNP_ID_VERSION_OFFSET] | (v[PNP_ID_VERSION_OFFSET + 1] << 8);
		c->cb->pnp_id(conn, &id);
	}

	c->state = HOGP_HOST_STATE_DISCOVER_BAS;
	next_step(c);
	return BT_GATT_ITER_STOP;
}

/* ---- BAS discovery callback ---- */

static uint8_t discover_bas_cb(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);

	HIDS_DBG("discover_bas_cb: conn=%p attr=%p c=%p", conn, attr, c);
	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		if (c->num_bats > 0) {
			c->bat_idx = 0;
			c->state = HOGP_HOST_STATE_DISCOVER_BAS_CHARS;
		} else {

			advance_to_finish(c);
			return BT_GATT_ITER_STOP;
		}

		next_step(c);
		return BT_GATT_ITER_STOP;
	}

	if (c->num_bats < BT_HOGP_HOST_MAX_BAT_INSTANCES) {
		struct bt_gatt_service_val *sv = attr->user_data;
		uint8_t idx = c->num_bats++;
		c->bats[idx].start_handle = attr->handle;
		c->bats[idx].end_handle = sv->end_handle;
		HIDS_DBG("BAS[%u]: 0x%04x-0x%04x", idx,
			attr->handle, sv->end_handle);
	}

	return BT_GATT_ITER_CONTINUE;
}

/* BAS characteristic discovery callback */
static uint8_t discover_bas_chars_cb(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	struct bt_gatt_chrc *chrc;

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	if (!attr) {
		if ((c->bat_idx + 1) < c->num_bats) {
			c->bat_idx++;
			c->state = HOGP_HOST_STATE_DISCOVER_BAS_CHARS;
			next_step(c);
		} else {
			/* Enable battery notifications */
			c->bat_idx = 0;
			c->state = HOGP_HOST_STATE_ENABLE_BAT_NOTIFY;
			next_step(c);
		}

		return BT_GATT_ITER_STOP;
	}

	chrc = attr->user_data;
	if (BT_UUID_16(chrc->uuid)->val == BT_UUID_BAS_BATTERY_LEVEL_VAL) {
		c->bats[c->bat_idx].level_handle = chrc->value_handle;
	}

	return BT_GATT_ITER_CONTINUE;
}

/* BAS subscribe callback */
static void bat_subscribe_cb(struct bt_conn *conn, uint8_t err,
			     struct bt_gatt_subscribe_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (!c) {
		return;
	}

	if ((c->bat_idx + 1) < c->num_bats) {
		c->bat_idx++;
		c->state = HOGP_HOST_STATE_ENABLE_BAT_NOTIFY;
		next_step(c);
	} else {

		advance_to_finish(c);
	}
}

/* ---- advance_to_dis: transition from HID discovery to DIS/BAS ---- */

static void advance_to_finish(struct hogp_host_conn *c)
{
#if defined(CONFIG_BT_HOGP_HOST_ISO)
	/* Discover HID ISO Service before finishing */
	c->state = HOGP_HOST_STATE_DISCOVER_HID_ISO_SERVICE;
	next_step(c);
#else
	finish_connected(c, 0);
#endif
}

static void advance_to_dis(struct hogp_host_conn *c)
{
	c->state = HOGP_HOST_STATE_DISCOVER_DIS;
	next_step(c);
}

/* ---- State machine driver ---- */


#if defined(CONFIG_BT_HOGP_HOST_ISO)
static uint8_t discover_hid_iso_svc_cb(struct bt_conn *conn,
					const struct bt_gatt_attr *attr,
					struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	if (!c) return BT_GATT_ITER_STOP;

	if (!attr) {
		HIDS_DBG("HID ISO Service not found (optional)");
		finish_connected(c, 0);
		return BT_GATT_ITER_STOP;
	}

	struct bt_gatt_service_val *sv = attr->user_data;
	c->iso_svc_start = attr->handle + 1;
	c->iso_svc_end = sv->end_handle;
	c->iso_supported = true;
	HIDS_DBG("HID ISO Service found: 0x%04x-0x%04x", c->iso_svc_start, c->iso_svc_end);

	c->state = HOGP_HOST_STATE_DISCOVER_HID_ISO_CHARS;
	next_step(c);
	return BT_GATT_ITER_STOP;
}

static uint8_t discover_hid_iso_chars_cb(struct bt_conn *conn,
					 const struct bt_gatt_attr *attr,
					 struct bt_gatt_discover_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);
	if (!c) return BT_GATT_ITER_STOP;

	if (!attr) {
		if (c->iso_props_handle) {
			c->state = HOGP_HOST_STATE_READ_ISO_PROPERTIES;
			next_step(c);
		} else {
			finish_connected(c, 0);
		}
		return BT_GATT_ITER_STOP;
	}

	struct bt_gatt_chrc *chrc = attr->user_data;
	uint16_t uuid_val = BT_UUID_16(chrc->uuid)->val;

	if (uuid_val == BT_UUID_HID_ISO_PROPERTIES_VAL)
		c->iso_props_handle = chrc->value_handle;
	else if (uuid_val == BT_UUID_HID_ISO_OP_MODE_VAL)
		c->iso_op_mode_handle = chrc->value_handle;

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t read_iso_properties_cb(struct bt_conn *conn, uint8_t err,
				      struct bt_gatt_read_params *params,
				      const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);
	if (!c) return BT_GATT_ITER_STOP;

	if (err || !data) {
		HIDS_DBG("ISO Properties read done (len=%u)", c->iso_properties_len);
		finish_connected(c, 0);
		return BT_GATT_ITER_STOP;
	}

	uint16_t copy = length;
	if (c->iso_properties_len + copy > sizeof(c->iso_properties_buf))
		copy = sizeof(c->iso_properties_buf) - c->iso_properties_len;
	memcpy(c->iso_properties_buf + c->iso_properties_len, data, copy);
	c->iso_properties_len += copy;

	return BT_GATT_ITER_CONTINUE;
}
#endif /* CONFIG_BT_HOGP_HOST_ISO */
static void finish_connected(struct hogp_host_conn *c, int status)
{
	HIDS_DBG("finish_connected: status=%d, instances=%u, reports=%u, bats=%u",
		status, c->num_instances, c->num_reports, c->num_bats);
	if (status == 0) {
		c->state = HOGP_HOST_STATE_CONNECTED;
		HIDS_DBG("HOGP Host connected: %u services, %u reports",
			c->num_instances, c->num_reports);
		for (uint8_t i = 0; i < c->num_reports; i++) {
			HIDS_DBG("  report[%u]: handle=0x%04x id=%u type=%u boot=%d svc=%u",
				i, c->reports[i].value_handle, c->reports[i].report_id,
				c->reports[i].report_type, c->reports[i].boot_report,
				c->reports[i].service_index);
		}

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
		/* Subscribe to SCI Mode Notify if Device supports SCI */
		if (c->services[0].sci_mode_handle) {
			memset(&sci_mode_sub_params, 0, sizeof(sci_mode_sub_params));
			sci_mode_sub_params.notify = sci_mode_notify_cb;
			sci_mode_sub_params.value_handle = c->services[0].sci_mode_handle;
			sci_mode_sub_params.value = BT_GATT_CCC_NOTIFY;
#if defined(CONFIG_BT_GATT_AUTO_DISCOVER_CCC)
			sci_mode_sub_params.ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE;
			sci_mode_sub_params.end_handle = c->services[0].end_handle;
			sci_mode_sub_params.disc_params = &c->disc_params;
#endif
			int sub_err = bt_gatt_subscribe(c->conn, &sci_mode_sub_params);
			if (sub_err)
				HIDS_WRN("SCI Mode subscribe failed: %d", sub_err);
			else
				HIDS_DBG("SCI Mode subscribed");
		}
#endif
	}

	if (c->cb && c->cb->connected) {
		c->cb->connected(c->conn, status, c->num_instances,
				 c->required_protocol_mode);
	}

	if (status != 0) {
		free_conn(c);
	}
}

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)

static uint8_t read_sci_info_cb(struct bt_conn *conn, uint8_t err,
				struct bt_gatt_read_params *params,
				const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);
	struct hogp_host_hid_svc *svc;

	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	svc = &c->services[c->svc_idx];

	if (!err && data && length >= 3) {
		const uint8_t *d = data;
		uint8_t i;
		uint8_t ng;

		svc->sci_info.min_conn_interval = d[0] | (d[1] << 8);
		ng = d[2];
		if (ng > ARRAY_SIZE(svc->sci_info.groups)) {
			ng = ARRAY_SIZE(svc->sci_info.groups);
		}

		svc->sci_info.num_groups = ng;

		for (i = 0; i < ng && (3 + (i + 1) * 6) <= length; i++) {
			const uint8_t *g = d + 3 + i * 6;

			svc->sci_info.groups[i].min_interval = g[0] | (g[1] << 8);
			svc->sci_info.groups[i].max_interval = g[2] | (g[3] << 8);
			svc->sci_info.groups[i].stride = g[4] | (g[5] << 8);
		}

		HIDS_DBG("SCI Info[%u]: min=%u groups=%u",
			c->svc_idx, svc->sci_info.min_conn_interval, ng);
	}

	/* Next service's SCI Info */
	c->svc_idx++;
	while (c->svc_idx < c->num_instances) {
		if (c->services[c->svc_idx].sci_supported &&
		    c->services[c->svc_idx].sci_info_handle) {
			c->state = HOGP_HOST_STATE_READ_SCI_INFO;
			next_step(c);
			return BT_GATT_ITER_STOP;
		}

		c->svc_idx++;
	}

	/* All SCI Info read, proceed to Report Map */
	c->svc_idx = 0;
	c->state = HOGP_HOST_STATE_READ_REPORT_MAP;
	next_step(c);
	return BT_GATT_ITER_STOP;
}

static uint8_t sci_mode_notify_cb(struct bt_conn *conn,
				  struct bt_gatt_subscribe_params *params,
				  const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (!c || !data || length < 1) {
		return BT_GATT_ITER_CONTINUE;
	}

	if (c->cb && c->cb->mode_changed) {
		const uint8_t *mode = data;

		/* Update current_mode: SCI bit from Device notification */
#if defined(CONFIG_BT_HOGP_HOST_ISO)
		if (*mode != BT_HOGP_DEVICE_SCI_MODE_NONE)
			c->current_mode |= BT_HOGP_MODE_SCI;
		else
			c->current_mode &= ~BT_HOGP_MODE_SCI;
#endif
		c->cb->mode_changed(conn, c->current_mode, 0);
	}

	return BT_GATT_ITER_CONTINUE;
}

static struct bt_gatt_subscribe_params sci_mode_sub_params;
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

static void next_step(struct hogp_host_conn *c)
{
	int err;

	static struct bt_uuid_16 hids_uuid = BT_UUID_INIT_16(BT_UUID_HIDS_VAL);

	HIDS_DBG("next_step: state=%s(%d) svc_idx=%u rpt_idx=%u",
		state_name(c->state), c->state, c->svc_idx, c->rpt_idx);

	switch (c->state) {
	case HOGP_HOST_STATE_DISCOVER_SERVICE:
		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = &hids_uuid.uuid;
		c->disc_params.func = discover_service_cb;
		c->disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		c->disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		c->disc_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		HIDS_DBG("bt_gatt_discover(PRIMARY, uuid=0x1812) ret=%d conn=%p", err, c->conn);
		if (err) {
			HIDS_ERR("Service discovery failed: %d", err);
			finish_connected(c, err);
		}
		break;

	case HOGP_HOST_STATE_DISCOVER_CHARS:
		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = NULL;
		c->disc_params.func = discover_chars_cb;
		c->disc_params.start_handle = c->services[c->svc_idx].start_handle;
		c->disc_params.end_handle = c->services[c->svc_idx].end_handle;
		c->disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_ERR("Char discovery failed: %d", err);
			finish_connected(c, err);
		}
		break;

	case HOGP_HOST_STATE_SET_PROTOCOL_MODE: {
		struct hogp_host_hid_svc *svc = &c->services[c->svc_idx];

		if (svc->protocol_mode_handle == 0) {
			if ((c->svc_idx + 1) < c->num_instances) {
				c->svc_idx++;
				next_step(c);
			} else {

				c->svc_idx = 0;
				c->state = HOGP_HOST_STATE_READ_HID_INFO;
				next_step(c);
			}
			break;
		}

		uint8_t mode = BT_HOGP_HOST_PROTOCOL_BOOT;

		err = bt_gatt_write_without_response(c->conn,
			svc->protocol_mode_handle, &mode, 1, false);
		if (err) {
			HIDS_WRN("Set protocol mode failed: %d", err);
		}

		svc->protocol_mode = BT_HOGP_HOST_PROTOCOL_BOOT;

		if ((c->svc_idx + 1) < c->num_instances) {
			c->svc_idx++;
			next_step(c);
		} else {

			c->svc_idx = 0;
			c->state = HOGP_HOST_STATE_READ_HID_INFO;
			next_step(c);
		}
		break;
	}

	case HOGP_HOST_STATE_READ_HID_INFO: {
		/* Find next service with hid_info_handle */
		while (c->svc_idx < c->num_instances) {
			if (c->services[c->svc_idx].hid_info_handle) {
				break;
			}

			c->svc_idx++;
		}

		if (c->svc_idx >= c->num_instances) {
			/* No HID Info to read, go to Report Map */
			c->svc_idx = 0;
			c->state = HOGP_HOST_STATE_READ_REPORT_MAP;
			next_step(c);
			break;
		}

		memset(&c->read_params, 0, sizeof(c->read_params));
		c->read_params.func = read_hid_info_cb;
		c->read_params.handle_count = 1;
		c->read_params.single.handle =
			c->services[c->svc_idx].hid_info_handle;
		c->read_params.single.offset = 0;

		err = bt_gatt_read(c->conn, &c->read_params);
		if (err) {
			HIDS_WRN("HID Info read failed: %d", err);
			c->svc_idx = 0;
			c->state = HOGP_HOST_STATE_READ_REPORT_MAP;
			next_step(c);
		}
		break;
	}

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	case HOGP_HOST_STATE_READ_SCI_INFO: {
		while (c->svc_idx < c->num_instances) {
			if (c->services[c->svc_idx].sci_supported &&
			    c->services[c->svc_idx].sci_info_handle) {
				break;
			}

			c->svc_idx++;
		}

		if (c->svc_idx >= c->num_instances) {
			c->svc_idx = 0;
			c->state = HOGP_HOST_STATE_READ_REPORT_MAP;
			next_step(c);
			break;
		}

		memset(&c->read_params, 0, sizeof(c->read_params));
		c->read_params.func = read_sci_info_cb;
		c->read_params.handle_count = 1;
		c->read_params.single.handle =
			c->services[c->svc_idx].sci_info_handle;
		c->read_params.single.offset = 0;

		err = bt_gatt_read(c->conn, &c->read_params);
		if (err) {
			HIDS_WRN("SCI Info read failed: %d", err);
			c->svc_idx = 0;
			c->state = HOGP_HOST_STATE_READ_REPORT_MAP;
			next_step(c);
		}
		break;
	}
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

	case HOGP_HOST_STATE_READ_REPORT_MAP: {
		struct hogp_host_hid_svc *svc = &c->services[c->svc_idx];

		if (svc->report_map_handle == 0) {
			/* Skip services without report map */
			c->svc_idx++;
			while (c->svc_idx < c->num_instances) {
				if (c->services[c->svc_idx].report_map_handle) {
					break;
				}

				c->svc_idx++;
			}

			if (c->svc_idx < c->num_instances) {
				next_step(c);
			} else {
				/* No report maps, go to report ref discovery */
				int idx = next_report_char_idx(c, 0);
				if (idx >= 0) {
					c->rpt_idx = idx;
					c->state = HOGP_HOST_STATE_FIND_REPORT_DESCS;
					next_step(c);
				} else {

					int nidx = next_input_report_idx(c, 0);
					if (nidx >= 0) {
						c->rpt_idx = nidx;
						c->state = HOGP_HOST_STATE_ENABLE_NOTIFICATIONS;
						next_step(c);
					} else {

						advance_to_dis(c);
					}
				}
			}
			break;
		}

		svc->desc_offset = g_desc_used;
		svc->desc_len = 0;

		memset(&c->read_params, 0, sizeof(c->read_params));
		c->read_params.func = read_report_map_cb;
		c->read_params.handle_count = 1;
		c->read_params.single.handle = svc->report_map_handle;
		c->read_params.single.offset = 0;

		err = bt_gatt_read(c->conn, &c->read_params);
		if (err) {
			HIDS_ERR("Report map read failed: %d", err);
			finish_connected(c, err);
		}
		break;
	}

	case HOGP_HOST_STATE_DISCOVER_REPORT_MAP_DESCS: {
		/* Find next service with report_map_handle */
		while (c->svc_idx < c->num_instances) {
			if (c->services[c->svc_idx].report_map_handle) {
				break;
			}

			c->svc_idx++;
		}

		if (c->svc_idx >= c->num_instances) {
			/* No more, go to Report Reference discovery */
			int idx = next_report_char_idx(c, 0);
			HIDS_DBG("DISCOVER_REPORT_MAP_DESCS: no more svcs, next_report_char=%d num_reports=%u",
				idx, c->num_reports);
			if (idx >= 0) {
				c->rpt_idx = idx;
				c->state = HOGP_HOST_STATE_FIND_REPORT_DESCS;
				next_step(c);
			} else {

				idx = next_input_report_idx(c, 0);
				if (idx >= 0) {
					c->rpt_idx = idx;
					c->state = HOGP_HOST_STATE_ENABLE_NOTIFICATIONS;
					next_step(c);
				} else {

					advance_to_dis(c);
				}
			}
			break;
		}

		struct hogp_host_hid_svc *svc = &c->services[c->svc_idx];

		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = NULL;
		c->disc_params.func = discover_ext_ref_cb;
		c->disc_params.start_handle = svc->report_map_handle;
		/* end_handle: find next char handle or service end */
		uint16_t end = svc->end_handle;
		for (uint8_t i = 0; i < c->num_reports; i++) {
			if (c->reports[i].service_index == c->svc_idx &&
			    c->reports[i].value_handle > svc->report_map_handle &&

			    c->reports[i].value_handle < end) {
				end = c->reports[i].value_handle - 1;
			}
		}

		c->disc_params.end_handle = end;
		c->disc_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

		HIDS_DBG("DISCOVER_REPORT_MAP_DESCS: svc[%u] discover 0x%04x-0x%04x",
			c->svc_idx, svc->report_map_handle, end);
		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_WRN("Ext ref desc discovery failed: %d", err);
			c->svc_idx++;
			c->state = HOGP_HOST_STATE_DISCOVER_REPORT_MAP_DESCS;
			next_step(c);
		}
		break;
	}

	case HOGP_HOST_STATE_FIND_REPORT_DESCS: {
		struct hogp_host_report *r = &c->reports[c->rpt_idx];

		c->tmp_handle = 0;
		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = NULL;
		c->disc_params.func = discover_report_desc_cb;
		c->disc_params.start_handle = r->value_handle;
		/* end_handle: use next char's handle - 1, or service end */
		uint16_t end = c->services[r->service_index].end_handle;
		/* Find next report in same service with higher handle */
		for (uint8_t i = 0; i < c->num_reports; i++) {
			if (c->reports[i].service_index == r->service_index &&
			    c->reports[i].value_handle > r->value_handle &&

			    c->reports[i].value_handle < end) {
				end = c->reports[i].value_handle - 1;
			}
		}

		c->disc_params.end_handle = end;
		c->disc_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_ERR("Descriptor discovery failed: %d", err);
			finish_connected(c, err);
		}
		break;
	}

	case HOGP_HOST_STATE_READ_REPORT_ID_TYPE:
		memset(&c->read_params, 0, sizeof(c->read_params));
		c->read_params.func = read_report_ref_cb;
		c->read_params.handle_count = 1;
		c->read_params.single.handle = c->tmp_handle;
		c->read_params.single.offset = 0;

		err = bt_gatt_read(c->conn, &c->read_params);
		if (err) {
			HIDS_ERR("Report ref read failed: %d", err);
			finish_connected(c, err);
		}
		break;

	case HOGP_HOST_STATE_ENABLE_NOTIFICATIONS: {
		struct hogp_host_report *r = &c->reports[c->rpt_idx];

		HIDS_DBG("ENABLE_NOTIFICATIONS: rpt_idx=%d value_handle=0x%04x ccc_handle=0x%04x id=%d type=%d",
			c->rpt_idx, r->value_handle, r->ccc_handle, r->report_id, r->report_type);

		memset(&r->sub_params, 0, sizeof(r->sub_params));
		r->sub_params.notify = notify_cb;
		r->sub_params.subscribe = subscribe_cb;
		r->sub_params.value_handle = r->value_handle;
		r->sub_params.ccc_handle = r->ccc_handle;
		r->sub_params.value = BT_GATT_CCC_NOTIFY;
#if defined(CONFIG_BT_GATT_AUTO_DISCOVER_CCC)
		if (r->ccc_handle == 0) {
			r->sub_params.ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE;
			r->sub_params.end_handle =
				c->services[r->service_index].end_handle;
			r->sub_params.disc_params = &c->disc_params;
		}
#endif

		atomic_set_bit(r->sub_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

		err = bt_gatt_subscribe(c->conn, &r->sub_params);
		if (err) {
			HIDS_WRN("Subscribe failed handle 0x%04x: %d",
				r->value_handle, err);
			/* Continue to next */
			int idx = next_input_report_idx(c, c->rpt_idx + 1);
			if (idx >= 0) {
				c->rpt_idx = idx;
				next_step(c);
			} else {

				advance_to_dis(c);
			}
		}
		break;
	}

	case HOGP_HOST_STATE_DISCOVER_DIS: {
		static struct bt_uuid_16 dis_uuid = BT_UUID_INIT_16(BT_UUID_DIS_VAL);

		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = &dis_uuid.uuid;
		c->disc_params.func = discover_dis_cb;
		c->disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		c->disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		c->disc_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_WRN("DIS discovery failed: %d", err);
			c->state = HOGP_HOST_STATE_DISCOVER_BAS;
			next_step(c);
		}
		break;
	}

	case HOGP_HOST_STATE_DISCOVER_DIS_CHARS:
		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = NULL;
		c->disc_params.func = discover_dis_chars_cb;
		c->disc_params.start_handle = c->dis_start_handle;
		c->disc_params.end_handle = c->dis_end_handle;
		c->disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_WRN("DIS char discovery failed: %d", err);
			c->state = HOGP_HOST_STATE_DISCOVER_BAS;
			next_step(c);
		}
		break;

	case HOGP_HOST_STATE_READ_PNP_ID:
		memset(&c->read_params, 0, sizeof(c->read_params));
		c->read_params.func = read_pnp_id_cb;
		c->read_params.handle_count = 1;
		c->read_params.single.handle = c->pnp_id_handle;
		c->read_params.single.offset = 0;

		err = bt_gatt_read(c->conn, &c->read_params);
		if (err) {
			HIDS_WRN("PnP ID read failed: %d", err);
			c->state = HOGP_HOST_STATE_DISCOVER_BAS;
			next_step(c);
		}
		break;

	case HOGP_HOST_STATE_DISCOVER_BAS: {
		static struct bt_uuid_16 bas_uuid = BT_UUID_INIT_16(BT_UUID_BAS_VAL);

		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = &bas_uuid.uuid;
		c->disc_params.func = discover_bas_cb;
		c->disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		c->disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		c->disc_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_WRN("BAS discovery failed: %d", err);
			advance_to_finish(c);
		}
		break;
	}

	case HOGP_HOST_STATE_DISCOVER_BAS_CHARS:
		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = NULL;
		c->disc_params.func = discover_bas_chars_cb;
		c->disc_params.start_handle = c->bats[c->bat_idx].start_handle;
		c->disc_params.end_handle = c->bats[c->bat_idx].end_handle;
		c->disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_WRN("BAS char discovery failed: %d", err);
			advance_to_finish(c);
		}
		break;

	case HOGP_HOST_STATE_ENABLE_BAT_NOTIFY: {
		struct hogp_host_bat *b = &c->bats[c->bat_idx];

		if (b->level_handle == 0) {
			if ((c->bat_idx + 1) < c->num_bats) {
				c->bat_idx++;
				next_step(c);
			} else {

				advance_to_finish(c);
			}
			break;
		}

		memset(&b->sub_params, 0, sizeof(b->sub_params));
		b->sub_params.notify = bat_notify_cb;
		b->sub_params.subscribe = bat_subscribe_cb;
		b->sub_params.value_handle = b->level_handle;
		b->sub_params.ccc_handle = b->ccc_handle;
		b->sub_params.value = BT_GATT_CCC_NOTIFY;
#if defined(CONFIG_BT_GATT_AUTO_DISCOVER_CCC)
		if (b->ccc_handle == 0) {
			b->sub_params.ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE;
			b->sub_params.end_handle = b->end_handle;
			b->sub_params.disc_params = &c->disc_params;
		}
#endif

		atomic_set_bit(b->sub_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

		err = bt_gatt_subscribe(c->conn, &b->sub_params);
		if (err) {
			HIDS_WRN("BAS subscribe failed: %d", err);
			if ((c->bat_idx + 1) < c->num_bats) {
				c->bat_idx++;
				next_step(c);
			} else {

				advance_to_finish(c);
			}
		}
		break;
	}

#if defined(CONFIG_BT_HOGP_HOST_ISO)
	case HOGP_HOST_STATE_DISCOVER_HID_ISO_SERVICE: {
		static struct bt_uuid_16 iso_svc_uuid = BT_UUID_INIT_16(BT_UUID_HID_ISO_SERVICE_VAL);

		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = &iso_svc_uuid.uuid;
		c->disc_params.func = discover_hid_iso_svc_cb;
		c->disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		c->disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		c->disc_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_DBG("HID ISO Service discovery failed: %d (not supported)", err);
			finish_connected(c, 0);
		}
		break;
	}

	case HOGP_HOST_STATE_DISCOVER_HID_ISO_CHARS: {
		memset(&c->disc_params, 0, sizeof(c->disc_params));
		c->disc_params.uuid = NULL;
		c->disc_params.func = discover_hid_iso_chars_cb;
		c->disc_params.start_handle = c->iso_svc_start;
		c->disc_params.end_handle = c->iso_svc_end;
		c->disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(c->conn, &c->disc_params);
		if (err) {
			HIDS_WRN("HID ISO chars discovery failed: %d", err);
			finish_connected(c, 0);
		}
		break;
	}

	case HOGP_HOST_STATE_READ_ISO_PROPERTIES: {
		memset(&c->read_params, 0, sizeof(c->read_params));
		c->read_params.func = read_iso_properties_cb;
		c->read_params.handle_count = 1;
		c->read_params.single.handle = c->iso_props_handle;
		c->read_params.single.offset = 0;

		err = bt_gatt_read(c->conn, &c->read_params);
		if (err) {
			HIDS_WRN("ISO Properties read failed: %d", err);
			finish_connected(c, 0);
		}
		break;
	}
#endif /* CONFIG_BT_HOGP_HOST_ISO */

	default:
		break;
	}
}

/* ---- Disconnect callback ---- */

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	struct hogp_host_conn *c = find_conn(conn);
	const struct bt_hogp_host_cb *cb;

	if (!c) {
		return;
	}

	HIDS_DBG("hogp_host disconnected_cb: conn:%p reason=0x%02x state=%s",
		conn, reason, state_name(c->state));

	cb = c->cb;

#if defined(CONFIG_BT_HOGP_HOST_ISO)
	hogp_host_iso_disconnect(c);
#endif

	free_conn(c);
	if (cb && cb->disconnected) {
		cb->disconnected(conn, reason);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.disconnected = disconnected_cb,
};

static bool g_initialized;

/* ---- Get Report read callback ---- */

static uint8_t get_report_read_cb(struct bt_conn *conn, uint8_t err,
				  struct bt_gatt_read_params *params,
				  const void *data, uint16_t length)
{
	struct hogp_host_conn *c = find_conn(conn);

	HIDS_DBG("get_report_read_cb: err=%u data=%p len=%u c=%p", err, data, length, c);
	if (!c) {
		return BT_GATT_ITER_STOP;
	}

	c->state = HOGP_HOST_STATE_CONNECTED;

	if (!err && data && length > 0 && c->cb && c->cb->get_report_result) {
		struct hogp_host_report *r = find_report_by_handle(
			c, params->single.handle);
		if (r) {
			c->cb->get_report_result(conn, r->report_id,
						 r->report_type, data, length);
		}
	}

	return BT_GATT_ITER_STOP;
}

/* ---- Set Report write callback ---- */

static void set_report_write_cb(struct bt_conn *conn, uint8_t err,
				struct bt_gatt_write_params *params)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (c) {
		c->state = HOGP_HOST_STATE_CONNECTED;
	}

	if (err) {
		HIDS_WRN("Set report write err: %u", err);
	}
}

/* ---- Public API ---- */

void bt_hogp_host_init(void)
{
	if (g_initialized) {
		return;
	}

	memset(g_conns, 0, sizeof(g_conns));
	g_desc_used = 0;
	bt_conn_cb_register(&conn_callbacks);
	g_initialized = true;
}

int bt_hogp_host_connect(struct bt_conn *conn, uint8_t protocol_mode,
			 const struct bt_hogp_host_cb *cb)
{
	struct hogp_host_conn *c;

	HIDS_DBG("bt_hogp_host_connect: conn:%p protocol=%u", conn, protocol_mode);

	if (!conn || !cb) {
		return -EINVAL;
	}

	if (find_conn(conn)) {
		return -EALREADY;
	}

	c = alloc_conn(conn);

	if (!c) {
		return -ENOMEM;
	}

	c->cb = cb;
	c->required_protocol_mode = protocol_mode;
	c->state = HOGP_HOST_STATE_DISCOVER_SERVICE;
	next_step(c);
	return 0;
}

int bt_hogp_host_disconnect(struct bt_conn *conn)
{
	struct hogp_host_conn *c;

	HIDS_DBG("bt_hogp_host_disconnect: conn:%p", conn);
	c = find_conn(conn);

	if (!c) {
		return -ENOTCONN;
	}

	free_conn(c);
	return 0;
}

int bt_hogp_host_get_report(struct bt_conn *conn, uint8_t report_id,
			    uint8_t report_type)
{
	struct hogp_host_conn *c;
	struct hogp_host_report *r;
	int err;

	HIDS_DBG("bt_hogp_host_get_report: id=%u type=%u", report_id, report_type);
	c = find_conn(conn);

	if (!c || c->state != HOGP_HOST_STATE_CONNECTED) {
		return -ENOTCONN;
	}

	r = find_report(c, report_id, report_type);
	if (!r) {
		return -ENOENT;
	}

	c->state = HOGP_HOST_STATE_GET_REPORT;
	memset(&c->read_params, 0, sizeof(c->read_params));
	c->read_params.func = get_report_read_cb;
	c->read_params.handle_count = 1;
	c->read_params.single.handle = r->value_handle;
	c->read_params.single.offset = 0;

	err = bt_gatt_read(c->conn, &c->read_params);
	if (err) {
		c->state = HOGP_HOST_STATE_CONNECTED;
	}

	return err;
}

int bt_hogp_host_set_report(struct bt_conn *conn, uint8_t report_id,
			    uint8_t report_type, const uint8_t *data,
			    uint16_t len)
{
	struct hogp_host_conn *c;
	struct hogp_host_report *r;
	int err;

	HIDS_DBG("bt_hogp_host_set_report: id=%u type=%u len=%u", report_id, report_type, len);
	c = find_conn(conn);

	if (!c || c->state != HOGP_HOST_STATE_CONNECTED) {
		return -ENOTCONN;
	}

#if defined(CONFIG_BT_HOGP_HOST_ISO)
	/* Route Output Report over ISO if in Hybrid mode (Spec 5.2.1) */
	if ((c->current_mode & BT_HOGP_MODE_ISO) && c->iso_cis_connected &&
	    report_type == BT_HOGP_HOST_REPORT_TYPE_OUTPUT) {
		return hogp_host_iso_send_report(c, report_id, data, len);
	}
#endif

	r = find_report(c, report_id, report_type);
	if (!r) {
		return -ENOENT;
	}

	c->state = HOGP_HOST_STATE_SET_REPORT;
	memset(&c->write_params, 0, sizeof(c->write_params));
	c->write_params.func = set_report_write_cb;
	c->write_params.handle = r->value_handle;
	c->write_params.offset = 0;
	c->write_params.data = data;
	c->write_params.length = len;

	err = bt_gatt_write(c->conn, &c->write_params);
	if (err) {
		c->state = HOGP_HOST_STATE_CONNECTED;
	}

	return err;
}

int bt_hogp_host_set_protocol_mode(struct bt_conn *conn, uint8_t service_index,
				   uint8_t protocol_mode)
{
	struct hogp_host_conn *c;
	struct hogp_host_hid_svc *svc;
	int err;

	c = find_conn(conn);

	if (!c || c->state != HOGP_HOST_STATE_CONNECTED) {
		return -ENOTCONN;
	}

	if (service_index >= c->num_instances) {
		return -EINVAL;
	}

	svc = &c->services[service_index];
	if (svc->protocol_mode_handle == 0) {
		return -ENOTSUP;
	}

	err = bt_gatt_write_without_response(c->conn,
		svc->protocol_mode_handle, &protocol_mode, 1, false);
	if (!err) {
		svc->protocol_mode = protocol_mode;
	}

	return err;
}

int bt_hogp_host_suspend(struct bt_conn *conn, uint8_t service_index)
{
	struct hogp_host_conn *c;
	uint8_t val;

	c = find_conn(conn);

	if (!c || c->state != HOGP_HOST_STATE_CONNECTED) {
		return -ENOTCONN;
	}

	if (service_index >= c->num_instances ||
	    c->services[service_index].ctrl_point_handle == 0) {
		return -EINVAL;
	}

	val = 0; /* Suspend */
	return bt_gatt_write_without_response(c->conn,
		c->services[service_index].ctrl_point_handle, &val, 1, false);
}

int bt_hogp_host_exit_suspend(struct bt_conn *conn, uint8_t service_index)
{
	struct hogp_host_conn *c;
	uint8_t val;

	c = find_conn(conn);

	if (!c || c->state != HOGP_HOST_STATE_CONNECTED) {
		return -ENOTCONN;
	}

	if (service_index >= c->num_instances ||
	    c->services[service_index].ctrl_point_handle == 0) {
		return -EINVAL;
	}

	val = 1; /* Exit Suspend */
	return bt_gatt_write_without_response(c->conn,
		c->services[service_index].ctrl_point_handle, &val, 1, false);
}

/* ---- Unified mode management ---- */


#if defined(CONFIG_BT_HOGP_HOST_ISO)
/* ---- ISO CIS management for Host ---- */

#define HOGP_HOST_ISO_SDU_SIZE     48
#define HOGP_HOST_ISO_SDU_INTERVAL 5000 /* 5ms in us */
#define HOGP_HOST_ISO_LATENCY_MS   5
#define HOGP_HOST_ISO_DEV_ID       0
#define HOGP_HOST_SELECT_HYBRID_MAX_LEN 10

static void iso_op_mode_write_cb(struct bt_conn *conn, uint8_t err,
				 struct bt_gatt_write_params *params)
{
	if (err) {
	} else {
		HIDS_DBG("ISO Op Mode write OK");
		HIDS_ERR("ISO Op Mode write error: 0x%02x", err);
	}
}
#define HOGP_HOST_ISO_TX_BUF_CNT   4

NET_BUF_POOL_FIXED_DEFINE(hogp_host_iso_tx_pool, HOGP_HOST_ISO_TX_BUF_CNT,
			  BT_ISO_SDU_BUF_SIZE(HOGP_HOST_ISO_SDU_SIZE),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL); /* Opcode(1)+CIG(1)+CIS(1)+Interval(2)+SDU_In(1)+SDU_Out(1)+Enable(1~2)+pad */

/* Spec Table C.1: Recommended ISO parameters per report interval */
struct hogp_iso_param_preset {
	uint16_t report_interval_us;  /* Report interval in us */
	uint16_t iso_interval_us;     /* ISO_Interval in us */
	uint8_t  nse;                 /* NSE = BN */
	uint8_t  ft;                  /* Flush Timeout */
};

static const struct hogp_iso_param_preset iso_presets[] = {
	{ 1000,  5000, 5,  1 },  /* 1ms    → ISO 5ms,   NSE=5  */
	{ 1250,  5000, 4,  1 },  /* 1.25ms → ISO 5ms,   NSE=4  */
	{ 2000, 10000, 5,  1 },  /* 2ms    → ISO 10ms,  NSE=5  */
	{ 2500,  7500, 3,  1 },  /* 2.5ms  → ISO 7.5ms, NSE=3  */
	{ 3000, 15000, 5,  1 },  /* 3ms    → ISO 15ms,  NSE=5  */
	{ 3750,  7500, 2,  1 },  /* 3.75ms → ISO 7.5ms, NSE=2  */
	{ 4000, 20000, 5,  1 },  /* 4ms    → ISO 20ms,  NSE=5  */
	{ 5000,  5000, 1,  1 },  /* 5ms    → ISO 5ms,   NSE=1  */
	{ 7500,  7500, 1,  1 },  /* 7.5ms  → ISO 7.5ms, NSE=1  */
};

static const struct hogp_iso_param_preset *find_iso_preset(uint16_t interval_us)
{
	for (int i = 0; i < ARRAY_SIZE(iso_presets); i++) {
		if (iso_presets[i].report_interval_us == interval_us)
			return &iso_presets[i];
	}
	return NULL; /* fallback needed */
}

/**
 * Parse Supported Report Intervals bitmask from HID ISO Properties.
 * Returns the smallest supported interval in us, or 5000 as default.
 */
/* Spec Table 6.6: bit→interval mapping */
static const uint16_t iso_interval_map[] = {
	1000,  /* bit 0: 1 ms */
	2000,  /* bit 1: 2 ms */
	3000,  /* bit 2: 3 ms */
	4000,  /* bit 3: 4 ms */
	5000,  /* bit 4: 5 ms */
	1250,  /* bit 5: 1.25 ms */
	2500,  /* bit 6: 2.5 ms */
	3750,  /* bit 7: 3.75 ms */
	7500,  /* bit 8: 7.5 ms */
};

static int parse_iso_intervals_sorted(const uint8_t *props, uint16_t len,
				      uint16_t *out, uint8_t max_count)
{
	uint16_t bitmask;
	int count = 0;
	int i, j;

	if (len < 3)
		return 0;

	bitmask = props[1] | (props[2] << 8);

	for (i = 0; i < 9 && count < max_count; i++) {
		if (bitmask & (1 << i))
			out[count++] = iso_interval_map[i];
	}

	/* Sort ascending */
	for (i = 0; i < count - 1; i++)
		for (j = i + 1; j < count; j++)
			if (out[i] > out[j]) {
				uint16_t tmp = out[i];
				out[i] = out[j];
				out[j] = tmp;
			}

	return count;
}

int bt_hogp_host_get_supported_intervals(struct bt_conn *conn,
					 uint16_t *intervals, uint8_t max_count)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (!c || !c->iso_supported)
		return -ENOTSUP;

	return parse_iso_intervals_sorted(c->iso_properties_buf,
					  c->iso_properties_len,
					  intervals, max_count);
}

/**
 * Parse Max SDU sizes from HID ISO Properties.
 */
static void parse_iso_sdu_sizes(const uint8_t *props, uint16_t len,
				uint8_t *max_sdu_input, uint8_t *max_sdu_output)
{
	*max_sdu_input = (len >= 4) ? props[3] : 48;
	*max_sdu_output = (len >= 6) ? props[5] : 48;
}

static void hogp_host_iso_connected_cb(struct bt_iso_chan *chan)
{
	/* Find conn from iso_chan */
	for (int i = 0; i < BT_HOGP_HOST_MAX_CONNECTIONS; i++) {
		struct hogp_host_conn *c = &g_conns[i];

		if (&c->iso_chan == chan && c->conn) {
			c->iso_cis_connected = true;
			HIDS_DBG("ISO CIS connected");
			memset(c->iso_seq_valid, 0, sizeof(c->iso_seq_valid));
			memset(c->iso_output_seq, 0, sizeof(c->iso_output_seq));
			c->current_mode |= BT_HOGP_MODE_ISO;
			HIDS_DBG("ISO CIS connected, mode=0x%02x", c->current_mode);
			if (c->cb && c->cb->mode_changed)
				c->cb->mode_changed(c->conn, c->current_mode, 0);
			return;
		}
	}
}

static void hogp_host_iso_disconnected_cb(struct bt_iso_chan *chan, uint8_t reason)
{
	for (int i = 0; i < BT_HOGP_HOST_MAX_CONNECTIONS; i++) {
		struct hogp_host_conn *c = &g_conns[i];

		if (&c->iso_chan == chan && c->conn) {
			c->iso_cis_connected = false;
			HIDS_DBG("ISO CIS disconnected (reason=0x%02x)", reason);
			c->current_mode &= ~BT_HOGP_MODE_ISO;
			HIDS_DBG("ISO CIS disconnected reason=0x%02x, mode=0x%02x",
				 reason, c->current_mode);
			if (c->cb && c->cb->mode_changed)
				c->cb->mode_changed(c->conn, c->current_mode, 0);
			return;
		}
	}
}

/**
 * Send Output Report over ISO (Spec Table 5.2): Length + SeqNum + ReportID + Report
 */
static int hogp_host_iso_send_report(struct hogp_host_conn *c,
				     uint8_t report_id,
				     const uint8_t *data, uint16_t len)
{
	HIDS_DBG("iso_send: id=%u len=%u", report_id, len);
	struct net_buf *buf;

	if (!c->iso_cis_connected)
		return -ENOTCONN;

	buf = net_buf_alloc(&hogp_host_iso_tx_pool, K_NO_WAIT);
	if (!buf)
		return -ENOMEM;

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	/* HID ISO Packet (Spec Table 5.2) */
	net_buf_add_u8(buf, (uint8_t)len);
	net_buf_add_u8(buf, c->iso_output_seq[report_id]);
	net_buf_add_u8(buf, report_id);
	net_buf_add_mem(buf, data, len);

	/* Increment SeqNum after creating packet (Spec 5.6.1) */
	c->iso_output_seq[report_id]++;

	if (bt_iso_chan_send(&c->iso_chan, buf, c->iso_tx_seq++) < 0) {
		net_buf_unref(buf);
		return -EIO;
	}

	return 0;
}

/**
 * Send Confirmation packet (Spec Table 5.3): Length=0 + SeqNum + ReportID
 */
static void hogp_host_iso_send_confirm(struct hogp_host_conn *c,
					uint8_t seq_num, uint8_t report_id)
{
	HIDS_DBG("iso_confirm_tx: id=%u seq=%u", report_id, seq_num);
	struct net_buf *buf;

	buf = net_buf_alloc(&hogp_host_iso_tx_pool, K_NO_WAIT);
	if (!buf)
		return;

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_u8(buf, 0);          /* Length = 0 → Confirmation */
	net_buf_add_u8(buf, seq_num);
	net_buf_add_u8(buf, report_id);

	if (bt_iso_chan_send(&c->iso_chan, buf, c->iso_tx_seq++) < 0)
		net_buf_unref(buf);
}

static void hogp_host_iso_recv_cb(struct bt_iso_chan *chan,
				  const struct bt_iso_recv_info *info,
				  struct net_buf *buf)
{
	int i;

	for (i = 0; i < BT_HOGP_HOST_MAX_CONNECTIONS; i++) {
		struct hogp_host_conn *c = &g_conns[i];

		if (&c->iso_chan != chan || !c->conn)
			continue;

		/* Process all HID ISO packets in the SDU (Spec 5.4) */
		while (buf->len >= BT_HOGP_ISO_PKT_HDR_SIZE) {
			uint8_t length = net_buf_pull_u8(buf);
			uint8_t seq_num = net_buf_pull_u8(buf);
			uint8_t report_id = net_buf_pull_u8(buf);

			if (length == 0) {
				/* Confirmation (Spec Table 5.3) */
				continue;
			}

			if (buf->len < length)
				break;

			/* Sequence Number dedup (Spec 5.6.2) */
			if (c->iso_seq_valid[report_id]) {
				int diff = ((int)c->iso_last_seq[report_id] -
					    (int)seq_num) & 0xFF;

				if (diff > 0 && diff <= 7) {
					net_buf_pull(buf, length);
					continue;
				}
			}

			c->iso_seq_valid[report_id] = true;
			c->iso_last_seq[report_id] = seq_num;

			if (c->cb && c->cb->input_report)
				c->cb->input_report(c->conn, 0, report_id,
						    buf->data, length);

			if (c->iso_confirm_enabled)
				hogp_host_iso_send_confirm(c, seq_num,
							   report_id);

			net_buf_pull(buf, length);
		}

		return;
	}
}

static struct bt_iso_chan_ops hogp_host_iso_ops = {
	.connected    = hogp_host_iso_connected_cb,
	.disconnected = hogp_host_iso_disconnected_cb,
	.recv         = hogp_host_iso_recv_cb,
};

/**
 * Convert interval in us to Spec Table 6.6 bitmask (single bit set).
 */
static uint16_t interval_us_to_bit(uint16_t interval_us)
{
	static const uint16_t map[][2] = {
		{1000, 0x0001}, {2000, 0x0002}, {3000, 0x0004},
		{4000, 0x0008}, {5000, 0x0010}, {1250, 0x0020},
		{2500, 0x0040}, {3750, 0x0080}, {7500, 0x0100},
	};
	int i;

	for (i = 0; i < 9; i++) {
		if (map[i][0] == interval_us)
			return map[i][1];
	}

	return 0x0010; /* default: 5ms */
}

static int hogp_host_iso_connect(struct hogp_host_conn *c, uint16_t interval_us)
{
	struct bt_iso_chan *channels[1];
	struct bt_iso_cig_param cig_param;
	struct bt_iso_connect_param connect_param;
	const struct hogp_iso_param_preset *preset;
	uint8_t max_sdu_input, max_sdu_output;
	int err;

	/* Parse Device's ISO SDU sizes from cached properties */
	parse_iso_sdu_sizes(c->iso_properties_buf, c->iso_properties_len,
			    &max_sdu_input, &max_sdu_output);

	/* Check if first ISO report supports Confirmation (Spec Table 6.8 bit1) */
	c->iso_confirm_enabled = false;
	if (c->iso_properties_len >= 8) {
		uint8_t additional_info = c->iso_properties_buf[7];

		if (additional_info & 0x02)
			c->iso_confirm_enabled = true;
	}

	c->iso_tx_seq = 0;

	preset = find_iso_preset(interval_us);
	if (!preset) {
		HIDS_WRN("No ISO preset for interval %uus, using 5ms default", interval_us);
		preset = &iso_presets[7]; /* 5ms */
	}

	HIDS_DBG("ISO params: interval=%uus iso_interval=%uus nse=%u sdu_in=%u sdu_out=%u",
		 interval_us, preset->iso_interval_us, preset->nse,
		 max_sdu_input, max_sdu_output);

	/* Configure QoS from Device properties (Spec Section 5.3) */
	c->iso_rx_qos.sdu = max_sdu_input;
	c->iso_rx_qos.phy = BT_GAP_LE_PHY_2M;
	c->iso_rx_qos.rtn = 2;
	c->iso_tx_qos.sdu = max_sdu_output;
	c->iso_tx_qos.phy = BT_GAP_LE_PHY_2M;
	c->iso_tx_qos.rtn = 2;
	c->iso_qos.tx = &c->iso_tx_qos;
	c->iso_qos.rx = &c->iso_rx_qos;

	c->iso_chan.ops = &hogp_host_iso_ops;
	c->iso_chan.qos = &c->iso_qos;

	channels[0] = &c->iso_chan;

	/* CIG Create (Spec Section 5.3) */
	memset(&cig_param, 0, sizeof(cig_param));
	cig_param.cis_channels    = channels;
	cig_param.num_cis         = 1;
	cig_param.c_to_p_interval = interval_us;
	cig_param.p_to_c_interval = interval_us;
	cig_param.c_to_p_latency  = preset->iso_interval_us / 1000;
	cig_param.p_to_c_latency  = preset->iso_interval_us / 1000;
	cig_param.sca             = BT_GAP_SCA_UNKNOWN;
	cig_param.packing         = BT_ISO_PACKING_SEQUENTIAL;
	cig_param.framing         = BT_ISO_FRAMING_UNFRAMED;

	err = bt_iso_cig_create_mc(HOGP_HOST_ISO_DEV_ID, &cig_param, &c->iso_cig);
	if (err) {
		HIDS_ERR("CIG create failed: %d", err);
		return err;
	}

	/* CIS Connect */
	connect_param.acl     = c->conn;
	connect_param.iso_chan = &c->iso_chan;

	err = bt_iso_chan_connect_mc(HOGP_HOST_ISO_DEV_ID, &connect_param, 1);
	if (err) {
		HIDS_ERR("CIS connect failed: %d", err);
		bt_iso_cig_terminate(c->iso_cig);
		c->iso_cig = NULL;
		return err;
	}

	HIDS_DBG("CIS connecting...");
	return 0;
}

static void hogp_host_iso_disconnect(struct hogp_host_conn *c)
{
	if (c->iso_cis_connected) {
		bt_iso_chan_disconnect(&c->iso_chan);
	}

	if (c->iso_cig) {
		bt_iso_cig_terminate(c->iso_cig);
		c->iso_cig = NULL;
	}

	c->iso_cis_connected = false;
}
#endif /* CONFIG_BT_HOGP_HOST_ISO */

int bt_hogp_host_set_mode(struct bt_conn *conn,
			  const struct bt_hogp_host_mode_param *param)
{
	HIDS_DBG("set_mode: mode=0x%02x policy=%u iso_interval=%u", param->mode, param->policy, param->iso_interval);
	struct hogp_host_conn *c = find_conn(conn);
	struct hogp_host_hid_svc *svc;
	int err = 0;
	uint8_t old_mode;

	if (!c || c->state != HOGP_HOST_STATE_CONNECTED || !param)
		return -ENOTCONN;

	svc = &c->services[0];

#if defined(CONFIG_BT_HOGP_HOST_ISO)
	old_mode = c->current_mode;
#else
	old_mode = BT_HOGP_MODE_DEFAULT;
#endif

	/* Handle SCI bit changes */
	if ((param->mode & BT_HOGP_MODE_SCI) != (old_mode & BT_HOGP_MODE_SCI)) {
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
		if (param->mode & BT_HOGP_MODE_SCI) {
			uint8_t sci_val = param->policy;

			if (svc->ctrl_point_handle) {
				err = bt_gatt_write_without_response(conn,
					svc->ctrl_point_handle, &sci_val, 1, false);
			}
		} else {
			/* Disable SCI */
			uint8_t none = BT_HOGP_DEVICE_SCI_MODE_NONE;
			if (svc->ctrl_point_handle)
				bt_gatt_write_without_response(conn,
					svc->ctrl_point_handle, &none, 1, false);
		}
#endif
	}

#if defined(CONFIG_BT_HOGP_HOST_ISO)
	/* Handle ISO bit changes */
	if ((param->mode & BT_HOGP_MODE_ISO) != (old_mode & BT_HOGP_MODE_ISO)) {
		if (param->mode & BT_HOGP_MODE_ISO) {
			/* Enable ISO: write Select Hybrid + CIG Create + CIS Connect */
			if (!c->iso_supported || !c->iso_op_mode_handle) {
				err = -ENOTSUP;
			} else {
				/* Build Select Hybrid command (Spec Table 6.11):
				 * Opcode(1) + CIG_ID(1) + CIS_ID(1) +
				 * ReportInterval(2) + SDU_Input(1) + SDU_Output(1) +
				 * HybridModeISOReportsEnable(1~2)
				 */
				uint16_t interval_us;
				uint16_t interval_bits;
				uint8_t sdu_in, sdu_out;
				uint8_t cmd_len;

				interval_us = param->iso_interval;
				if (!interval_us)
					interval_us = 5000;
				interval_bits = interval_us_to_bit(interval_us);
				parse_iso_sdu_sizes(c->iso_properties_buf,
						    c->iso_properties_len,
						    &sdu_in, &sdu_out);

				c->iso_cmd_buf[0] = BT_HOGP_ISO_OPCODE_SELECT_HYBRID;
				c->iso_cmd_buf[1] = 0;
				c->iso_cmd_buf[2] = 0;
				sys_put_le16(interval_bits, &c->iso_cmd_buf[3]);
				c->iso_cmd_buf[5] = sdu_in;
				c->iso_cmd_buf[6] = sdu_out;
				c->iso_cmd_buf[7] = 0x00;
				cmd_len = 8;

				memset(&c->write_params, 0, sizeof(c->write_params));
				c->write_params.func = iso_op_mode_write_cb;
				c->write_params.handle = c->iso_op_mode_handle;
				c->write_params.data = c->iso_cmd_buf;
				c->write_params.length = cmd_len;
				err = bt_gatt_write(conn, &c->write_params);
				if (err) {
					HIDS_ERR("Write Op Mode failed: %d", err);
				} else {
					err = hogp_host_iso_connect(c, interval_us);
				}
			}
		} else {
			/* Disable ISO: write Select Default + disconnect CIS */
			if (c->iso_op_mode_handle) {
				c->iso_cmd_buf[0] = BT_HOGP_ISO_OPCODE_SELECT_DEFAULT;
				memset(&c->write_params, 0, sizeof(c->write_params));
				c->write_params.func = iso_op_mode_write_cb;
				c->write_params.handle = c->iso_op_mode_handle;
				c->write_params.data = c->iso_cmd_buf;
				c->write_params.length = 1;
				bt_gatt_write(conn, &c->write_params);
			}
			hogp_host_iso_disconnect(c);
		}
	}

	if (err == 0)
		c->current_mode = param->mode;
#endif

	/* mode_changed is called asynchronously:
	 * - SCI: from sci_mode_notify_cb when Device confirms
	 * - ISO: from CIS established callback (TODO)
	 * Only notify immediately on error */
	if (err && c->cb && c->cb->mode_changed)
		c->cb->mode_changed(conn, param->mode, err);

	return err;
}

int bt_hogp_host_get_mode(struct bt_conn *conn,
			  struct bt_hogp_host_mode_param *param)
{
	struct hogp_host_conn *c = find_conn(conn);

	if (!c || !param)
		return -EINVAL;

	memset(param, 0, sizeof(*param));
#if defined(CONFIG_BT_HOGP_HOST_ISO)
	param->mode = c->current_mode;
#else
	param->mode = BT_HOGP_MODE_DEFAULT;
#endif
	return 0;
}

int bt_hogp_host_get_report_map(struct bt_conn *conn, uint8_t service_index,
				const uint8_t **data, uint16_t *len)
{
	struct hogp_host_conn *c;
	struct hogp_host_hid_svc *svc;

	c = find_conn(conn);

	if (!c) {
		return -ENOTCONN;
	}

	if (service_index >= c->num_instances) {
		return -EINVAL;
	}

	svc = &c->services[service_index];
	*data = &g_desc_storage[svc->desc_offset];
	*len = svc->desc_len;
	return 0;
}
