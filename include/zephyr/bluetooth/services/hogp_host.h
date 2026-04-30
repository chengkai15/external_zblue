/**
 * @file hogp_host.h
 * @brief HID over GATT Host (HOGP Host / HIDS Client)
 *
 * Supports three modes: Default (GATT), SCI (GATT+SCI), ISO (Hybrid), ISO+SCI.
 * Mode management via bt_hogp_host_set_mode() / bt_hogp_host_get_mode().
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_HOGP_HOST_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_HOGP_HOST_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hid.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BT_HOGP_HOST_MAX_SERVICES 3
#define BT_HOGP_HOST_MAX_REPORTS 8
#define BT_HOGP_HOST_MAX_EXT_REPORTS 4
#define BT_HOGP_HOST_MAX_CONNECTIONS 1
#define BT_HOGP_HOST_DESC_STORAGE_LEN 256

#define BT_HOGP_HOST_MAX_BAT_INSTANCES 3
struct bt_hogp_host_pnp_id {
	uint8_t vid_src;
	uint16_t vid;
	uint16_t pid;
	uint16_t version;
};

struct bt_hogp_host_mode_param {
	uint8_t mode;           /* BT_HOGP_MODE_xxx */
	uint8_t policy;         /* SCI: BT_HOGP_SCI_MODE_xxx */
	uint16_t iso_interval;  /* ISO report interval in us, 0=default */
};

/** Query Device's supported ISO report intervals (sorted ascending).
 *  @param conn       Connection
 *  @param intervals  Output array (caller provides)
 *  @param max_count  Size of intervals array
 *  @return number of intervals filled, or negative error
 */
int bt_hogp_host_get_supported_intervals(struct bt_conn *conn,
					 uint16_t *intervals, uint8_t max_count);

/** HOGP Host callbacks */
struct bt_hogp_host_cb {
	/** Service discovery and setup complete */
	void (*connected)(struct bt_conn *conn, int status,
			  uint8_t num_instances, uint8_t protocol_mode);
	/** Disconnected */
	void (*disconnected)(struct bt_conn *conn, int reason);
	/** Report Map received for a service instance */
	void (*report_map)(struct bt_conn *conn, uint8_t service_index,
			   const uint8_t *data, uint16_t len);
	/** Input Report received (via GATT Notification or ISO CIS) */
	void (*input_report)(struct bt_conn *conn, uint8_t service_index,
			     uint8_t report_id, const uint8_t *data,
			     uint16_t len);
	/** Get Report response */
	void (*get_report_result)(struct bt_conn *conn, uint8_t report_id,
				  uint8_t report_type, const uint8_t *data,
				  uint16_t len);
	/** PnP ID read from DIS */
	void (*pnp_id)(struct bt_conn *conn,
		       const struct bt_hogp_host_pnp_id *id);
	/** Battery level notification or read */
	void (*battery_level)(struct bt_conn *conn, uint8_t bat_index,
			      uint8_t level);
	/** Mode changed notification (SCI/ISO/Default) */
	void (*mode_changed)(struct bt_conn *conn, uint8_t mode, int status);
};

/* ---- Connection management ---- */

void bt_hogp_host_init(void);

int bt_hogp_host_connect(struct bt_conn *conn, uint8_t protocol_mode,
			 const struct bt_hogp_host_cb *cb);

int bt_hogp_host_disconnect(struct bt_conn *conn);

/* ---- Mode management ---- */

int bt_hogp_host_set_mode(struct bt_conn *conn,
			  const struct bt_hogp_host_mode_param *param);

int bt_hogp_host_get_mode(struct bt_conn *conn,
			  struct bt_hogp_host_mode_param *param);

/* ---- Report operations ---- */

int bt_hogp_host_get_report(struct bt_conn *conn, uint8_t report_id,
			    uint8_t report_type);

int bt_hogp_host_set_report(struct bt_conn *conn, uint8_t report_id,
			    uint8_t report_type, const uint8_t *data,
			    uint16_t len);

int bt_hogp_host_set_protocol_mode(struct bt_conn *conn, uint8_t service_index,
				   uint8_t protocol_mode);

int bt_hogp_host_suspend(struct bt_conn *conn, uint8_t service_index);

int bt_hogp_host_exit_suspend(struct bt_conn *conn, uint8_t service_index);

int bt_hogp_host_get_report_map(struct bt_conn *conn, uint8_t service_index,
				const uint8_t **data, uint16_t *len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_HOGP_HOST_H_ */
