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
 * @defgroup ble_sdk_app_hids_keyboard_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_keyboard
 * @brief HID Keyboard Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Services for implementing a simple keyboard functionality.
 * Pressing Button 0 will send text 'hello' to the connected peer. On receiving output report,
 * it toggles the state of LED 2 on the mother board based on whether or not Caps Lock is on.
 * This application uses the @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "softdevice_handler_appsh.h"
#include "app_timer_appsh.h"
#include "device_manager.h"
#include "app_button.h"
#include "pstorage.h"
#include "app_trace.h"


#include "ble_nus.h"


#include "ble_app_store.h"
#include "ble_app_gzll_device.h"
#include "ble_app_gzll_hid.h"
#include "ble_app_gzll_ui.h"
#include "ble_app_gzll_common.h"


#include "ble_app_adc.h"
#include "io_reinit.h"
//#include "nrf_drv_clock.h"

#define SCHED_MAX_EVENT_DATA_SIZE        MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                             BLE_STACK_HANDLER_SCHED_EVT_SIZE)          /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                 10                                             /**< Maximum number of events in the scheduler queue. */

#define DEAD_BEEF  0xDEADBEEF   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


/**< Battery timer. */
APP_TIMER_DEF(m_led_flash_timer_id);  
#define LED_FLASH_INTERVAL      				 APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)     /**< Battery level measurement interval (ticks). */

//#define BOOTLOADER_BASE_ADDRESS         0x00038000UL
//const uint32_t store_version_address = 101;  

volatile radio_mode_t running_mode = BLE;
        bool  switch_ble_flag=false;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the Power Management.
 */
static void power_manage(void)
{
    if (running_mode == GAZELL)
    {
        // Use directly __WFE and __SEV macros since the SoftDevice is not available in proprietary 
        // mode.
        // Wait for event.
//        __WFE();

//        // Clear Event Register.
//        __SEV();
//        __WFE();
    }
    else if (running_mode == BLE)
    {
        uint32_t  err_code;
        
//        app_sched_execute();
			
				if(time_for_send_flag == true)
				{
					time_for_send_flag = false;
					adc_change_data_calculate();
					read_mouse_and_send_change();
					key_sense_turbo();
				}
				
				if(mode_change_flag == true)//goto GAZELL
				{
					mode_change_flag = false;
					running_mode = GAZELL;    
//					break;
				}
				
				if(goto_system_off_flag)
				{
						if(nrf_gpio_pin_read(BETOP_BUTTON))
						{
								goto_system_off_flag = false;
								goto_syetem_off();
						}
				}
				
//				if(goto_system_off_flag)
//				{
//						err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//						if(err_code == 	NRF_SUCCESS)
//						{
//							goto_system_off_flag = false;
//							goto_disconnect_event_flag = true;
//						}
//						else
//						{
//						}
//				}
				
//				if(goto_off_flag)//
//				{
//						if(btn_read_pin_state(BETOP_BUTTON)==0)
//						{
//								goto_off_flag = false;
//								goto_syetem_off();
//						}
//				}

				if (running_mode == GAZELL)
				{
				}
				else
				{
					if(switch_ble_flag==true)
					{
							// Use SoftDevice API for power_management when in Bluetooth Mode.
							err_code = sd_app_evt_wait();
							APP_ERROR_CHECK(err_code);
					}
				}
			
    }

}

//static void lfclk_config(void)
//{
//    nrf_clock_event_clear(NRF_CLOCK_EVENT_LFCLKSTARTED);
//    nrf_clock_int_enable(NRF_CLOCK_INT_LF_STARTED_MASK);
//    nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);
//}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

void timers_init_and_start(void)
{
		uint32_t  err_code;
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
	
    // Initialize timer module, making it use the scheduler.
//    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
    // Create led timer.
    err_code = app_timer_create(&m_led_flash_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                LED_FALSH_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_led_flash_timer_id, LED_FLASH_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t  err_code;
    radio_mode_t previous_mode = running_mode;

//		if((*(uint32_t *)BOOTLOADER_BASE_ADDRESS) == 0xffffffff)
//			nrf_nvmc_write_word(BOOTLOADER_BASE_ADDRESS,store_version_address);
	
		BETOP_switch_check();	
		com_mode_check();	
    NRF_POWER->DCDCEN = 1;                           //Enabling the DCDC converter for lower current consumption
    // Initialize.
    app_trace_init();
//    scheduler_init();
		NRF_LOG_INIT();

//    buttons_leds_init(&erase_bonds);
//    bsp_init_app();
		buttons_init_Judge();				//Judge
    leds_init_Judge();					//Judge

//		lfclk_config();
		adc_init();									//Judge
		timers_init_and_start();
	
    ble_stack_start();//APP RTC1
    ble_hid_app_start();
//		ble_stack_stop();
	
//		running_mode = GAZELL;    
    // Enter main loop.
    for (;;)
    {
        if (running_mode != previous_mode)
        {
            previous_mode = running_mode;//switch state need restart? store but not restart
            if (running_mode == GAZELL)
            {

                // Stop all heart rate functionality before disabling the SoftDevice.
                ble_hid_app_stop();
                
                // Disable the S110 stack.
                ble_stack_stop();
//                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//                APP_ERROR_CHECK(err_code);
                // Enable Gazell.
								mode_store(GAZELL);	
                gzll_app_start();
								switch_ble_flag=false;
            }
            else if (running_mode == BLE)
            {
                // Disable Gazell.
                gzll_app_stop();
								mode_store(BLE);	
//                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//                APP_ERROR_CHECK(err_code);
                // Re-enable the S110 stack.
                ble_stack_start();
                ble_hid_app_start();
							switch_ble_flag = true;
								
            }
        }
        power_manage();
    }
}


/**
 * @}
 */
