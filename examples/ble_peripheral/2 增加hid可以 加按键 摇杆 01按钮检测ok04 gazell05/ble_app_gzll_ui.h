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
 * @defgroup ble_sdk_app_gzll_ui Multiprotocol Application User Interface
 * @{
 * @ingroup ble_sdk_app_gzll
 * @brief User Interface (buttons and LED) handling for the multiprotocol application
 */

#ifndef BLE_APP_GZLL_UI_H__
#define BLE_APP_GZLL_UI_H__

#include <stdbool.h>
#include "bsp.h"

#define BLE_BUTTON_ID              0        /**<  Button used for switching to Bluetooth Heart Rate example. */
#define GZLL_BUTTON_ID             1        /**<  Button used for switching to Gazell example. */

extern bool goto_system_off_flag;
extern uint8_t my_role;
extern bool mode_change_flag;

/**@brief Function for initializing bsp module.
 */
//void bsp_init_app(void);
void buttons_init_Judge(void);//JUDGE
void leds_init_Judge(void);
void cpiote_interrupt_set(void);
void goto_syetem_off(void);
void LED_FALSH_handler(void * p_context);
void timers_init(void);

void key_sense_turbo(void);
void BETOP_switch_check(void);


#endif // BLE_APP_GZLL_UI_H__
/** @} */

