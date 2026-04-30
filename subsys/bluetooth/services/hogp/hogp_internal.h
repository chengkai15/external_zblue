/*
 * HOGP internal definitions shared between hogp_device.c and hogp_host.c.
 * Not part of the public API.
 */

#ifndef HOGP_INTERNAL_H_
#define HOGP_INTERNAL_H_

#include <zephyr/bluetooth/services/hogp_device.h>
#include <zephyr/bluetooth/services/hogp_host.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>

/* HID Control Point commands (HIDS) */
#define BT_HOGP_DEVICE_CTRL_SUSPEND      0
#define BT_HOGP_DEVICE_CTRL_EXIT_SUSPEND 1

/* HOGP Operation mode (shared by Device and Host) */
#define BT_HOGP_MODE_DEFAULT    0x00  /* GATT only */
#define BT_HOGP_MODE_SCI        0x01  /* GATT + SCI */
#define BT_HOGP_MODE_ISO        0x02  /* GATT + ISO CIS (Hybrid) */
#define BT_HOGP_MODE_ISO_SCI    0x03  /* ISO CIS + SCI */

/* SCI Mode values (BT 6.2 SCI CR) */
#define BT_HOGP_SCI_MODE_NONE       0x00
#define BT_HOGP_SCI_MODE_DEFAULT    0x02
#define BT_HOGP_SCI_MODE_FAST       0x03
#define BT_HOGP_SCI_MODE_LOW_POWER  0x04
#define BT_HOGP_SCI_MODE_FULL_RANGE 0x05

/* HID Information Flags bits (HOGP SCI CR) */
#define BT_HOGP_FLAG_SCI_SUPPORTED    BIT(2)
#define BT_HOGP_FLAG_SCI_LP_SUPPORTED BIT(3)

#if defined(CONFIG_BT_HOGP_DEVICE_ISO) || defined(CONFIG_BT_HOGP_HOST_ISO)
#include <zephyr/bluetooth/iso.h>
#endif

/* ================================================================
 * Shared UUID definitions for HID ISO Service
 * ================================================================ */

#ifndef BT_UUID_HID_ISO_SERVICE_VAL
#define BT_UUID_HID_ISO_SERVICE_VAL    0x185C
#endif
#ifndef BT_UUID_HID_ISO_PROPERTIES_VAL
#define BT_UUID_HID_ISO_PROPERTIES_VAL 0x2C23
#endif
#ifndef BT_UUID_HID_ISO_OP_MODE_VAL
#define BT_UUID_HID_ISO_OP_MODE_VAL    0x2C24
#endif

/* ================================================================
 * Device internal definitions
 * ================================================================ */

#define HOGP_DEVICE_MAX_CONNECTIONS  2
#define REPORT_MAP_MAX_SIZE          512
#define HID_INFO_VAL_SIZE            4
#define SCI_INFO_MAX_SIZE            (2 + 1 + 4 * 6)

#ifndef HOGP_DEVICE_MAX_ATTRS
#define HOGP_DEVICE_MAX_ATTRS        64
#endif

#define HID_ISO_SERVICE_ATTR_COUNT   5

/* HID ISO packet header size (Spec Table 5.2): Length + SeqNum + ReportID */
#define BT_HOGP_ISO_PKT_HDR_SIZE     3

/* HID ISO Properties characteristic field offsets (Spec Table 6.4) */
#define BT_HOGP_ISO_PROPS_FEATURES_OFFSET       0
#define BT_HOGP_ISO_PROPS_INTERVALS_OFFSET      1
#define BT_HOGP_ISO_PROPS_MAX_SDU_IN_OFFSET     3
#define BT_HOGP_ISO_PROPS_PREF_SDU_IN_OFFSET    4
#define BT_HOGP_ISO_PROPS_MAX_SDU_OUT_OFFSET    5
#define BT_HOGP_ISO_PROPS_PREF_SDU_OUT_OFFSET   6
#define BT_HOGP_ISO_PROPS_REPORTS_OFFSET        7

/* HID ISO Properties: Hybrid Mode ISO Reports entry size (Spec Table 6.7) */
#define BT_HOGP_ISO_REPORT_ENTRY_SIZE           2

/* HID ISO Properties: Additional Info bits (Spec Table 6.8) */
#define BT_HOGP_ISO_ADDL_INFO_REPORT_TYPE_BIT   BIT(0)
#define BT_HOGP_ISO_ADDL_INFO_CONFIRM_BIT       BIT(1)
#define BT_HOGP_ISO_ADDL_INFO_REPETITION_BIT    BIT(2)

/* HID ISO default SDU size (Spec Section 6.5.1.1) */
#define BT_HOGP_ISO_DEFAULT_SDU_SIZE            48

/* HID ISO default report interval in microseconds (5 ms) */
#define BT_HOGP_ISO_DEFAULT_INTERVAL_US         5000



/* HID ISO LE HID Operation Mode opcodes (Spec Table 6.10) */
#define BT_HOGP_ISO_OPCODE_SELECT_HYBRID        0x01
#define BT_HOGP_ISO_OPCODE_SELECT_DEFAULT       0x02

/* Sequence Number dedup window (Spec 5.6.2): past 7 values are duplicates */
#define BT_HOGP_ISO_SEQ_DEDUP_WINDOW            7

/* HID ISO Properties buffer size (Spec Table 6.4: 7 fixed + up to 4 report entries × 2) */
#define BT_HOGP_ISO_PROPERTIES_BUF_SIZE         16

/* Device ISO operation mode state */
#define ISO_OP_MODE_DEFAULT   0
#define ISO_OP_MODE_PENDING   1
#define ISO_OP_MODE_HYBRID    2

struct hogp_device_report_ref {
	uint8_t id;
	uint8_t type;
};

/* Device per-connection state */
struct hogp_device_conn {
	struct bt_conn *conn;  /* non-NULL = in use */
	uint8_t current_mode;  /* BT_HOGP_MODE_xxx */

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	uint8_t sci_mode;
	uint8_t sci_pending_mode;  /* 0 = no pending */
#endif

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
	uint8_t iso_op_mode;       /* ISO_OP_MODE_xxx */
	uint8_t iso_cig_id;
	uint8_t iso_cis_id;
	struct bt_iso_chan iso_chan;
	bool iso_cis_connected;
	uint16_t iso_seq_num;
	uint8_t iso_report_seq[BT_HOGP_DEVICE_MAX_REPORTS];
	uint32_t iso_tx_interval_us; /* SDU interval from CIS params */
#endif

	uint8_t pending_report_id;
	uint8_t pending_report_type;  /* 0 = no pending */
};

/* Device HID Service data */
struct hogp_device_hid_svc {
	struct bt_gatt_attr attrs[HOGP_DEVICE_MAX_ATTRS];
	uint16_t attr_count;
	struct bt_gatt_service svc;
	uint8_t protocol_mode;
	uint8_t report_map[REPORT_MAP_MAX_SIZE];
	uint16_t report_map_len;
	uint8_t hid_info[HID_INFO_VAL_SIZE];
	struct bt_hogp_device_report reports[BT_HOGP_DEVICE_MAX_REPORTS];
	uint8_t reports_cnt;
	uint8_t report_indices[BT_HOGP_DEVICE_MAX_REPORTS];
	struct hogp_device_report_ref report_refs[BT_HOGP_DEVICE_MAX_REPORTS];
	uint8_t input_ntf_enabled;
	struct _bt_gatt_ccc ccc_data[BT_HOGP_DEVICE_MAX_REPORTS];
};

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
/* Device SCI data */
#define HOGP_DEVICE_SCI_MODE_SUPPORTED    BIT(0)
#define HOGP_DEVICE_SCI_MODE_LP_SUPPORTED BIT(1)

struct hogp_device_sci {
	uint8_t modes;  /* HOGP_DEVICE_SCI_MODE_xxx bitmask */
	struct _bt_gatt_ccc mode_ccc;
	uint8_t properties[SCI_INFO_MAX_SIZE];
	uint16_t properties_len;
};
#endif

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
/* Device ISO Service data */
struct hogp_device_iso_svc {
	struct bt_gatt_attr attrs[HID_ISO_SERVICE_ATTR_COUNT];
	struct bt_gatt_attr *op_mode_val_attr;
	struct bt_gatt_service svc;
	struct bt_gatt_chrc props_chrc;
	struct bt_gatt_chrc op_mode_chrc;
	bool registered;
	bool dev_request;
	uint8_t properties[32];
	uint16_t properties_len;
	uint8_t max_sdu_input;
	uint8_t max_sdu_output;
	struct bt_iso_server server;
};
#endif

/* ================================================================
 * Host internal definitions
 * ================================================================ */

enum hogp_host_state {
	HOGP_HOST_STATE_IDLE = 0,
	HOGP_HOST_STATE_DISCOVER_SERVICE,
	HOGP_HOST_STATE_DISCOVER_CHARS,
	HOGP_HOST_STATE_SET_PROTOCOL_MODE,
	HOGP_HOST_STATE_READ_HID_INFO,
	HOGP_HOST_STATE_READ_SCI_INFO,
	HOGP_HOST_STATE_READ_REPORT_MAP,
	HOGP_HOST_STATE_DISCOVER_REPORT_MAP_DESCS,
	HOGP_HOST_STATE_READ_EXT_REF_UUID,
	HOGP_HOST_STATE_DISCOVER_EXT_CHARS,
	HOGP_HOST_STATE_FIND_REPORT_DESCS,
	HOGP_HOST_STATE_READ_REPORT_ID_TYPE,
	HOGP_HOST_STATE_ENABLE_NOTIFICATIONS,
	HOGP_HOST_STATE_DISCOVER_DIS,
	HOGP_HOST_STATE_DISCOVER_DIS_CHARS,
	HOGP_HOST_STATE_READ_PNP_ID,
	HOGP_HOST_STATE_DISCOVER_BAS,
	HOGP_HOST_STATE_DISCOVER_BAS_CHARS,
	HOGP_HOST_STATE_DISCOVER_BAS_DESCS,
	HOGP_HOST_STATE_READ_BAT_LEVEL,
	HOGP_HOST_STATE_ENABLE_BAT_NOTIFY,
#if defined(CONFIG_BT_HOGP_HOST_ISO)
	HOGP_HOST_STATE_DISCOVER_HID_ISO_SERVICE,
	HOGP_HOST_STATE_DISCOVER_HID_ISO_CHARS,
	HOGP_HOST_STATE_READ_ISO_PROPERTIES,
#endif
	HOGP_HOST_STATE_CONNECTED,
	HOGP_HOST_STATE_GET_REPORT,
	HOGP_HOST_STATE_SET_REPORT,
};

struct hogp_host_report {
	uint16_t value_handle;
	uint16_t end_handle;
	uint8_t properties;
	uint8_t service_index;
	uint8_t report_id;
	uint8_t report_type;
	bool boot_report;
	uint16_t ccc_handle;
	struct bt_gatt_subscribe_params sub_params;
};

struct hogp_host_ext_report {
	uint16_t desc_handle;
	uint16_t ext_ref_uuid;
	uint8_t service_index;
};

struct hogp_host_hid_svc {
	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t report_map_handle;
	uint16_t report_map_end_handle;
	uint16_t hid_info_handle;
	uint16_t ctrl_point_handle;
	uint16_t protocol_mode_handle;
	uint8_t protocol_mode;
	uint16_t bcd_hid;
	uint8_t b_country_code;
	uint8_t hid_flags;
	uint16_t desc_offset;
	uint16_t desc_len;
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	uint16_t sci_info_handle;
	uint16_t sci_mode_handle;
	uint16_t sci_mode_ccc_handle;
	struct bt_conn_le_min_conn_interval_info sci_info;
	bool sci_supported;
#endif
};

struct hogp_host_bat {
	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t level_handle;
	uint16_t ccc_handle;
	struct bt_gatt_subscribe_params sub_params;
};

/* Host per-connection state */
struct hogp_host_conn {
	struct bt_conn *conn;
	const struct bt_hogp_host_cb *cb;
	enum hogp_host_state state;
	uint8_t required_protocol_mode;

	struct hogp_host_hid_svc services[BT_HOGP_HOST_MAX_SERVICES];
	uint8_t num_instances;

	struct hogp_host_report reports[BT_HOGP_HOST_MAX_REPORTS];
	uint8_t num_reports;

	struct hogp_host_ext_report ext_reports[BT_HOGP_HOST_MAX_REPORTS];
	uint8_t num_ext_reports;

	/* DIS */
	uint16_t dis_start_handle;
	uint16_t dis_end_handle;
	uint16_t pnp_id_handle;

	/* Battery Service */
	struct hogp_host_bat bats[BT_HOGP_HOST_MAX_BAT_INSTANCES];
	uint8_t num_bats;
	uint8_t bat_idx;

	uint8_t svc_idx;
	uint8_t rpt_idx;
	uint16_t tmp_handle;

	struct bt_gatt_discover_params disc_params;
	struct bt_gatt_read_params read_params;
	struct bt_gatt_write_params write_params;

	uint16_t rmap_read_offset;

#if defined(CONFIG_BT_HOGP_HOST_ISO)
	/* HID ISO Service */
	uint16_t iso_svc_start;
	uint16_t iso_svc_end;
	uint16_t iso_props_handle;
	uint16_t iso_op_mode_handle;
	uint8_t iso_cmd_buf[10]; /* LE HID Op Mode write buffer */
	bool iso_supported;
	uint8_t iso_properties_buf[32];
	uint16_t iso_properties_len;

	/* ISO CIS */
	struct bt_iso_chan iso_chan;
	struct bt_iso_cig *iso_cig;
	struct bt_iso_chan_io_qos iso_tx_qos;
	struct bt_iso_chan_io_qos iso_rx_qos;
	struct bt_iso_chan_qos iso_qos;
	bool iso_cis_connected;

	/* ISO Sequence Number tracking (Spec 5.6.2) */
	uint8_t iso_last_seq[256];
	bool iso_seq_valid[256];
	bool iso_confirm_enabled;
	uint16_t iso_tx_seq;
#endif

	/* Mode state (shared by ISO and SCI) */
	uint8_t current_mode;

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	uint8_t sci_policy;
	struct bt_gatt_subscribe_params sci_mode_sub_params;
#endif
};

#endif /* HOGP_INTERNAL_H_ */
