/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
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

/**@file
 *
 * @defgroup ble_sdk_app_bootloader_main main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file.
 *
 * -# Receive start data packet. 
 * -# Based on start packet, prepare NVM area to store received data. 
 * -# Receive data packet. 
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */
#include "dfu_transport.h"
#include "bootloader.h"
#include "bootloader_util.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "nrf.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_error.h"
#include "bsp.h"
#include "softdevice_handler_appsh.h"
#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "nrf_log.h"

#include "nrf_gpio.h"
bool goto_system_off_flag = false;
#include "main_init.h"
#include "ble_conn_params.h"

#if BUTTONS_NUMBER < 1
#error "Not enough buttons on board"
#endif

#if LEDS_NUMBER < 1
#error "Not enough LEDs on board"
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                                       /**< Include the service_changed characteristic. For DFU this should normally be the case. */

#define BOOTLOADER_BUTTON               CLEAR_BUTTON                                            /**< Button used to enter SW update mode. */
#define UPDATE_IN_PROGRESS_LED          LED2_OUTPUT                                               /**< Led used to indicate that DFU is active. */

#define APP_TIMER_PRESCALER             0                                                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                                       /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, 0)                        /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                20                                                      /**< Maximum number of events in the scheduler queue. */


static uint32_t pins_value_temp;
uint8_t btn_read_pin_state(uint8_t button)
{
    return (pins_value_temp & (1 << button)) == 0;
}

static uint32_t BETOP_switch_check(void)
{
	static uint32_t betop_button_flag = 0;
	
	nrf_gpio_cfg_input(BETOP_BUTTON,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(CLEAR_BUTTON,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_output(LED1_OUTPUT);
	nrf_gpio_cfg_output(LED2_OUTPUT);
		nrf_gpio_pin_clear(LED1_OUTPUT);
		nrf_gpio_pin_clear(LED2_OUTPUT);
	
	pins_value_temp = nrf_gpio_pins_read();
	
	while(btn_read_pin_state(BETOP_BUTTON))
	{
		pins_value_temp = nrf_gpio_pins_read();
		betop_button_flag = 1;
	}
	
	while(btn_read_pin_state(CLEAR_BUTTON))
	{
		pins_value_temp = nrf_gpio_pins_read();
	}	
	
//	if(betop_button_flag)
//	{
//		cpiote_interrupt_set();
//		POWER_OFF;
////		sd_power_system_off();	
//	}
//	else
//	{
//	}

}

	static uint32_t betop_button_count;
	static uint32_t turbo_button_count;
	static uint32_t clear_button_count;
	
void cpiote_interrupt_set(void)
{
    nrf_gpio_cfg_sense_input(BETOP_BUTTON,NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);

//		NVIC_EnableIRQ(GPIOTE_IRQn);
//    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;
}

static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);
}
extern  uint16_t               m_conn_handle;            /**< Current connection handle. */

static void reset_prepare()
{
    uint32_t err_code;
       
//		err_code = sd_power_gpregret_clr(0,0x00);
//		APP_ERROR_CHECK(err_code);
//		err_code = sd_power_gpregret_set(0,0xB1);
//		APP_ERROR_CHECK(err_code);       
       
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }     
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }
              
    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
		
}

void goto_syetem_off(void)
{
		CLOSE_M1
		CLOSE_M2
		CLOSE_KEY_VDD	
		cpiote_interrupt_set();
		nrf_gpio_pin_set(LED1_OUTPUT);
		nrf_gpio_pin_set(LED2_OUTPUT);
		//POWER_OFF;
		sd_power_system_off();	
}

static uint32_t key_sense_turbo(void)
{
//#define BETOP_BUTTON 21
//#define CLEAR_BUTTON 13
	
#define  BETOP_BUTTON_VALUE 100
#define  TURBO_BUTTON_VALUE 100
#define  CLEAR_BUTTON_VALUE 100
	
   uint32_t err_code;
	
	
//	if(btn_read_pin_state(TURBO_BUTTON))
//	{
//		turbo_button_count++;
//	}
//	else
//	{
//		turbo_button_count = 0;
//	}
//	
//	if(turbo_button_count > TURBO_BUTTON_VALUE)	// BLE 2.4G
//	{
//			turbo_button_count = 0;
//			
//			led_flash_time();
//		
//			mode_change_flag = !mode_change_flag;
//		
//	}
		nrf_gpio_cfg_input(BETOP_BUTTON,NRF_GPIO_PIN_PULLUP);

		pins_value_temp = nrf_gpio_pins_read();
	
	if(btn_read_pin_state(CLEAR_BUTTON)==1)
	{
		clear_button_count++;
	}
	else
	{
		clear_button_count = 0;
	}
	
	if(clear_button_count > CLEAR_BUTTON_VALUE)	//dfu
	{
			clear_button_count = 0;
//			NRF_POWER->GPREGRET = 0xB1;
			
//			err_code = sd_power_gpregret_clr(0xFF);
//			APP_ERROR_CHECK(err_code);
//			err_code = sd_power_gpregret_set(0xB1);
//			APP_ERROR_CHECK(err_code);
			
//			led_flash_time();
			reset_prepare();
			NVIC_SystemReset();
		
	}
	
	if(btn_read_pin_state(BETOP_BUTTON)==1)
	{
		betop_button_count++;
	}
	else
	{
		betop_button_count = 0;
	}
	
	if(betop_button_count > BETOP_BUTTON_VALUE)	// 
	{
			betop_button_count = 0;
			
//			while(btn_read_pin_state(BETOP_BUTTON));
			goto_system_off_flag = true;
		
	}
				if(goto_system_off_flag)
				{
//					goto_system_off_flag = false;
					if(nrf_gpio_pin_read(BETOP_BUTTON)==1)
					{
						reset_prepare();
						goto_syetem_off();
					}

				}
	
}

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)     /**< Battery level measurement interval (ticks). */
APP_TIMER_DEF(m_battery_timer_id);                                                      /**< Battery timer. */

static void battery_level_meas_timeout_handler(void * p_context)
{
//    UNUSED_PARAMETER(p_context);
		key_sense_turbo();
}

static void timers_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] file_name   File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for initialization of LEDs.
 */
static void leds_init(void)
{
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    nrf_gpio_pins_set(LEDS_MASK);
}


/**@brief Function for initializing the timer handler module (app_timer).
 */
static void timers_init(void)
{
    uint32_t         err_code;
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
	
    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);

}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void sys_evt_dispatch(uint32_t event)
{
    pstorage_sys_event_handler(event);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 *
 * @param[in] init_softdevice  true if SoftDevice should be initialized. The SoftDevice must only 
 *                             be initialized if a chip reset has occured. Soft reset from 
 *                             application must not reinitialize the SoftDevice.
 */
static void ble_stack_init(bool init_softdevice)
{
    uint32_t         err_code;
    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    if (init_softdevice)
    {
        err_code = sd_mbr_command(&com);
        APP_ERROR_CHECK(err_code);
    }
    
    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
    APP_ERROR_CHECK(err_code);
   
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    // Only one connection as a central is used when performing dfu.
    err_code = softdevice_enable_get_default_config(1, 1, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for event scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void leds_init1(void)
{
    uint32_t i;
    const uint32_t BUTTONS[] = INPUT_BUTTON;
    const uint32_t PWMs[] = OUPUTS_PWM;
	
    for (i = 0; i < INPUTS_NUMBER; i++)
    {
				nrf_gpio_cfg_input(BUTTONS[i],NRF_GPIO_PIN_PULLUP);
    }
	
    for (i = 0; i < OUPUTS_PWM_NUMBER; i++)
    {
				nrf_gpio_cfg_output(PWMs[i]);
    }	
		
		CLOSE_M1
		CLOSE_M2
//		OPEN_KEY_VDD		
		CLOSE_KEY_VDD	
		CLOSE_LED1
		CLOSE_LED2		
	
//    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
//    LED_OFF(ADVERTISING_LED_PIN_NO);
//    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
//    LED_OFF(CONNECTED_LED_PIN_NO);
//    nrf_gpio_cfg_output(ASSERT_LED_PIN_NO);
//    LED_OFF(ASSERT_LED_PIN_NO);
	
}

/**@brief Function for bootloader main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     dfu_start = false;
    bool     app_reset = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_START);

		BETOP_switch_check();	

    if (app_reset)
    {
        NRF_POWER->GPREGRET = 0;
    }
    
//    leds_init();
		leds_init1();
		
    // This check ensures that the defined fields in the bootloader corresponds with actual
    // setting in the chip.
    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

    // Initialize.
    timers_init();
//    buttons_init();

    (void)bootloader_init();

    if (bootloader_dfu_sd_in_progress())
    {

        err_code = bootloader_dfu_sd_update_continue();
        APP_ERROR_CHECK(err_code);

        ble_stack_init(!app_reset);
        scheduler_init();

        err_code = bootloader_dfu_sd_update_finalize();
        APP_ERROR_CHECK(err_code);


    }
    else
    {
        // If stack is present then continue initialization of bootloader.
        ble_stack_init(true);
        scheduler_init();
    }

    dfu_start  = app_reset;
    
    
    
    if (dfu_start||(!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
    {
        nrf_gpio_pin_clear(UPDATE_IN_PROGRESS_LED);
				timers_start();

        // Initiate an update of the firmware.
        err_code = bootloader_dfu_start();
        APP_ERROR_CHECK(err_code);

        nrf_gpio_pin_set(UPDATE_IN_PROGRESS_LED);
    }

    if (bootloader_app_is_valid(DFU_BANK_0_REGION_START) && !bootloader_dfu_sd_in_progress())
    {
        // Select a bank region to use as application region.
        // @note: Only applications running from DFU_BANK_0_REGION_START is supported.
        bootloader_app_start(DFU_BANK_0_REGION_START);
    }
    
    NVIC_SystemReset();
}
