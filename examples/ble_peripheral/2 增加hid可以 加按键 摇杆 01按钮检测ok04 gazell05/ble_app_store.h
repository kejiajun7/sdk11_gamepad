/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_gzll_gazell_part Gazell part of Multiprotocol Application
 * @{
 * @ingroup ble_sdk_app_gzll
 * @brief Gazell demo application used in the multiprotocol application.
 */

#ifndef BLE_APP_STORE_H__
#define BLE_APP_STORE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble_app_gzll_common.h"

void com_mode_check(void);
void mode_store(radio_mode_t run_mode);

#endif // BLE_APP_GZLL_DEVICE_H__
/** @} */
