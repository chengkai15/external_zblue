/** @file
 *  @brief Bluetooth HID common definitions.
 *
 *  Transport-agnostic HID constants shared by Classic HID (HIDP),
 *  BLE HID Service (HIDS), and HID over GATT Profile (HOGP).
 */

/*
 * Copyright 2026 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_HID_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_HID_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HID Report Type values
 *
 * Defined by BT Core Spec Vol 3 Part G (Report Reference Descriptor,
 * UUID 0x2908) and USB HID Spec Section 7.2.1. Used across Classic HID,
 * BLE HIDS, and HOGP ISO.
 */
#define BT_HID_REPORT_TYPE_OTHER   0
#define BT_HID_REPORT_TYPE_INPUT   1
#define BT_HID_REPORT_TYPE_OUTPUT  2
#define BT_HID_REPORT_TYPE_FEATURE 3

/** @brief HID Protocol Mode values (HIDS Protocol Mode characteristic) */
#define BT_HID_PROTOCOL_BOOT   0
#define BT_HID_PROTOCOL_REPORT 1

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_HID_H_ */
