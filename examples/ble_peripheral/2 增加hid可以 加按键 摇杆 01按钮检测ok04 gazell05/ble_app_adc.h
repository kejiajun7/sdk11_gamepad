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

#ifndef BLE_APP_ADC_H__
#define BLE_APP_ADC_H__

#include <stdint.h>
#include <stdbool.h>

#define ADC_NUM_INIT 2
#define ADC_BUFFER_SIZE 6                               //Size of buffer for ADC samples. Buffer size should be multiple of number of adc channels located.


void adc_init(void);
void adc_disinit(void);
void adc_change_data_calculate(void);
//void adc_sampling_event_init(void);
//void adc_sampling_event_enable(void);
//void adc_config(void);

//void mouse_sensor_new_sample_generated_cb();
//uint8_t adc_change_data_get(void);
//void read_mouse_and_send2(void);

//uint8_t btn_read_pin_state(uint8_t button);

//void goto_syetem_off(void);
extern uint16_t value_adc_change[ADC_BUFFER_SIZE];
extern bool time_for_send_flag;//Judge

#endif // BLE_APP_GZLL_DEVICE_H__
/** @} */
