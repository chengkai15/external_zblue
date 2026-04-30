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

#include <zephyr/bluetooth/hid.h>


struct bt_hogp_device_report {
	uint8_t id;
	uint8_t type; /* BT_HID_REPORT_TYPE_xxx */
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

/**
 * @brief Send an Input Report to the connected Host.
 *
 * Routes through ISO CIS in Hybrid mode, or GATT notification in Default mode.
 *
 * @param conn    Connection object. NULL broadcasts via GATT notification
 *                to all subscribed connections (ISO hybrid mode requires
 *                a specific connection).
 * @param report_id  HID Report ID. Pass 0 only when the device has a single
 *                   Input Report (HID Spec: Report ID field is omitted).
 *                   When multiple Input Reports exist, caller MUST provide
 *                   the correct report_id; passing 0 returns -EINVAL.
 * @param data    Report payload (excluding Report ID byte).
 * @param len     Length of @p data in bytes.
 *
 * @return 0 on success, negative errno on failure.
 */
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
