/**
 * @file hogp_device.h
 * @brief HID over GATT Profile Device (HOGP Device)
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_HOGP_DEVICE_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_HOGP_DEVICE_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BT_HOGP_DEVICE_MAX_REPORTS 8

#define BT_HOGP_DEVICE_REPORT_TYPE_INPUT   1
#define BT_HOGP_DEVICE_REPORT_TYPE_OUTPUT  2
#define BT_HOGP_DEVICE_REPORT_TYPE_FEATURE 3

#define BT_HOGP_DEVICE_PROTOCOL_BOOT   0
#define BT_HOGP_DEVICE_PROTOCOL_REPORT 1

#define BT_HOGP_DEVICE_CTRL_SUSPEND      0
#define BT_HOGP_DEVICE_CTRL_EXIT_SUSPEND 1

/* Operation mode (shared with Host) */
#define BT_HOGP_MODE_DEFAULT    0x00  /* GATT only */
#define BT_HOGP_MODE_SCI        0x01  /* GATT + SCI */
#define BT_HOGP_MODE_ISO        0x02  /* GATT + ISO CIS (Hybrid) */
#define BT_HOGP_MODE_ISO_SCI    0x03  /* ISO CIS + SCI */

/* LE HID Operation Mode Opcodes (HOGP v1.1 Table 6.10) */
#define BT_HOGP_ISO_OPCODE_SELECT_HYBRID   0x01
#define BT_HOGP_ISO_OPCODE_SELECT_DEFAULT  0x02

/* HID ISO Packet header size */
#define BT_HOGP_ISO_PKT_HDR_SIZE           3       /* ReportType+ID(1) + SeqNum(1) + Length(1) */

#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
/* SCI Mode values (HOGP CR) */
#define BT_HOGP_DEVICE_SCI_MODE_NONE       0x00
#define BT_HOGP_DEVICE_SCI_MODE_DEFAULT    0x02
#define BT_HOGP_DEVICE_SCI_MODE_FAST       0x03
#define BT_HOGP_DEVICE_SCI_MODE_LOW_POWER  0x04
#define BT_HOGP_DEVICE_SCI_MODE_FULL_RANGE 0x05

/* HID Control Point SCI commands */
#define BT_HOGP_DEVICE_CTRL_SCI_DEFAULT    0x02
#define BT_HOGP_DEVICE_CTRL_SCI_FAST       0x03
#define BT_HOGP_DEVICE_CTRL_SCI_LOW_POWER  0x04
#define BT_HOGP_DEVICE_CTRL_SCI_FULL_RANGE 0x05

/* HID Information Flags bits */
#define BT_HOGP_DEVICE_FLAG_SCI_SUPPORTED    BIT(2)
#define BT_HOGP_DEVICE_FLAG_SCI_LP_SUPPORTED BIT(3)
#endif /* CONFIG_BT_SHORTER_CONNECTION_INTERVALS */

struct bt_hogp_device_report {
	uint8_t id;
	uint8_t type; /* BT_HOGP_DEVICE_REPORT_TYPE_xxx */
};

struct bt_hogp_device_info {
	uint16_t bcd_hid;
	uint8_t  b_country_code;
	uint8_t  flags;
};

struct bt_hogp_device_cb {
	/** Called when a BLE host connects */
	void (*connected)(struct bt_conn *conn);
	/** Called when a BLE host disconnects */
	void (*disconnected)(struct bt_conn *conn, uint8_t reason);
	/** Called on GET_REPORT */
	void (*get_report)(struct bt_conn *conn, uint8_t report_type,
			   uint8_t report_id, uint16_t buf_size);
	/** Called on SET_REPORT */
	void (*set_report)(struct bt_conn *conn, uint8_t report_type,
			   uint8_t report_id, const uint8_t *data, uint16_t len);
	/** Called on Protocol Mode write */
	void (*set_protocol)(struct bt_conn *conn, uint8_t protocol);
	/** Called on HID Control Point write */
	void (*ctrl_point)(struct bt_conn *conn, uint8_t value);
	/** Called when operation mode changes (SCI/ISO/Default) */
	void (*mode_changed)(struct bt_conn *conn, uint8_t mode);
};

struct bt_hogp_device_init_param {
	struct bt_hogp_device_info info;
	const uint8_t *report_map;
	uint16_t report_map_len;
	const struct bt_hogp_device_report *reports;
	const uint8_t *report_sizes;   /* input report byte sizes, indexed by report */
	uint8_t report_count;
	const struct bt_hogp_device_cb *cb;
#if defined(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
	bool sci_supported;
	bool sci_lp_supported;
#endif
#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
	bool     iso_enabled;
	uint16_t iso_report_intervals;      /* Supported Report Intervals bitmask */
	uint8_t  iso_max_sdu_input;         /* Max SDU Size for Input Reports */
	uint8_t  iso_preferred_sdu_input;   /* Preferred SDU Size for Input Reports */
	uint8_t  iso_max_sdu_output;        /* Max SDU Size for Output Reports */
	uint8_t  iso_preferred_sdu_output;  /* Preferred SDU Size for Output Reports */
	bool     iso_device_mode_change;    /* Device Mode Change Supported */
#endif
};

int bt_hogp_device_register(const struct bt_hogp_device_init_param *param);

void bt_hogp_device_unregister(void);

int bt_hogp_device_connect(const bt_addr_le_t *peer);

int bt_hogp_device_disconnect(struct bt_conn *conn);

int bt_hogp_device_send_report(struct bt_conn *conn, uint8_t report_id,
			       const uint8_t *data, uint16_t len);

/**
 * @brief Get ISO TX interval in milliseconds.
 * @return interval in ms, or 0 if ISO not connected.
 */
uint16_t bt_hogp_device_get_iso_interval_ms(struct bt_conn *conn);

int bt_hogp_device_get_report_response(struct bt_conn *conn, uint8_t report_id,
				       uint8_t report_type,
				       const uint8_t *data, uint16_t len);

int bt_hogp_device_report_error(struct bt_conn *conn, uint8_t error);

int bt_hogp_device_virtual_cable_unplug(struct bt_conn *conn);

#if defined(CONFIG_BT_HOGP_DEVICE_ISO)
/**
 * @brief Request operation mode change (Device-initiated, Spec Section 5.2.2).
 *
 * Indicates LE HID Operation Mode to Host requesting a mode switch.
 * Requires Device Mode Change Supported in HID ISO Properties.
 *
 * @param conn BLE connection.
 * @param mode BT_HOGP_MODE_ISO or BT_HOGP_MODE_DEFAULT.
 * @return 0 on success.
 */
int bt_hogp_device_request_mode(struct bt_conn *conn, uint8_t mode);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_HOGP_DEVICE_H_ */
