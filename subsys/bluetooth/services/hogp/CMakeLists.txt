# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_BT_HOGP_DEVICE hogp_device.c)
zephyr_sources_ifdef(CONFIG_BT_HOGP_HOST hogp_host.c)
