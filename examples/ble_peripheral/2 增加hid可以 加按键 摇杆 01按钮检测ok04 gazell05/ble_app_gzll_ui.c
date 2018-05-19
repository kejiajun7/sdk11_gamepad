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

#include "ble_app_gzll_ui.h"
//#include "app_error.h"
#include "app_timer.h"
#include "ble_app_gzll_common.h"

#include "io_reinit.h"
#include "ble_app_adc.h"


#define POWER_OFF		NRF_POWER->SYSTEMOFF = 1



uint8_t my_role = 0;

bool goto_system_off_flag = false;
bool mode_change_flag = false;
extern bool system_addr_received;

static uint32_t pins_value_temp;

/**@brief Function for the Timer module initialization.
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
//    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
	
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

}
 */

void buttons_init_Judge(void)//JUDGE
{
    uint32_t i;
    const uint32_t BUTTONS[] = INPUT_BUTTON;
	
    for (i = 0; i < INPUTS_NUMBER; i++)
    {
				nrf_gpio_cfg_input(BUTTONS[i],NRF_GPIO_PIN_PULLUP);
    }
		
}

void leds_init_Judge(void)
{
    uint32_t i;
    const uint32_t PWMs[] = OUPUTS_PWM;
	
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
	
}

void LED_FALSH_update(void)
{
	static bool flag = false;
	
	flag = !flag;	
	
	if(my_role)
	{
		if(my_role == 2)
		{
				nrf_gpio_pin_clear(LED1_OUTPUT);
				nrf_gpio_pin_set(LED2_OUTPUT);
		}
		
		if(my_role == 3)
		{
				nrf_gpio_pin_clear(LED2_OUTPUT);
				nrf_gpio_pin_set(LED1_OUTPUT);
		}		
	}
	else
	{
			if(running_mode==BLE)	//BLE
			{
					if(flag)
					{
							nrf_gpio_pin_set(LED2_OUTPUT);
					}
					else
					{
							nrf_gpio_pin_clear(LED2_OUTPUT);
					}
					nrf_gpio_pin_set(LED1_OUTPUT);
			}
			else	//2.4G
			{
					if(flag)
					{
							nrf_gpio_pin_set(LED1_OUTPUT);
					}
					else
					{
							nrf_gpio_pin_clear(LED1_OUTPUT);
					}
					nrf_gpio_pin_set(LED2_OUTPUT);
				
			}
	}
	
}

void LED_FALSH_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    LED_FALSH_update();
}


/**@brief Function for handling bsp events.
static void bsp_event_handler(bsp_event_t evt)
{
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
                        running_mode = BLE;
            break;

        case BSP_EVENT_KEY_1:
                        running_mode = GAZELL;
            break;

        default:
                        APP_ERROR_HANDLER((uint8_t)evt);
            break;
    }
}
 */


/**@brief Function for initializing bsp module.
void bsp_init_app(void)
{
    uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}
*/

static bool btn_read_pin_state(uint8_t pin_number)
{
//    return  ((NRF_GPIO->IN >> pin_number) & 1UL);
    return (pins_value_temp & (1 << pin_number)) == 0;
}

void cpiote_interrupt_set(void)
{
    nrf_gpio_cfg_sense_input(BETOP_BUTTON,NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);

		NVIC_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;
}

void goto_syetem_off(void)
{
		adc_disinit();
		CLOSE_M1
		CLOSE_M2
		CLOSE_KEY_VDD	
		cpiote_interrupt_set();
		nrf_gpio_pin_set(LED1_OUTPUT);
		nrf_gpio_pin_set(LED2_OUTPUT);
	
		if (running_mode == GAZELL)
		{
			POWER_OFF;
		}
		else
		{
			sd_power_system_off();	
		}

}

void led_flash_time(void)
{
			nrf_gpio_pin_set(LED1_OUTPUT);
			nrf_gpio_pin_set(LED2_OUTPUT);
			nrf_delay_ms(100);
			
			nrf_gpio_pin_clear(LED1_OUTPUT);
			nrf_gpio_pin_clear(LED2_OUTPUT);
			nrf_delay_ms(100);
			
			nrf_gpio_pin_set(LED1_OUTPUT);
			nrf_gpio_pin_set(LED2_OUTPUT);
			nrf_delay_ms(100);
			
			nrf_gpio_pin_clear(LED1_OUTPUT);
			nrf_gpio_pin_clear(LED2_OUTPUT);
			nrf_delay_ms(100);
			
//			nrf_gpio_pin_set(LED1_OUTPUT);
//			nrf_gpio_pin_set(LED2_OUTPUT);

}

void key_sense_turbo(void)
{
    uint32_t err_code;
static uint32_t  BETOP_BUTTON_VALUE = 300;
static uint32_t  TURBO_BUTTON_VALUE = 300;
static uint32_t  CLEAR_BUTTON_VALUE = 300;
	
	static uint32_t betop_button_count = 0;
	static uint32_t turbo_button_count = 0;
	static uint32_t clear_button_count = 0;
	
		pins_value_temp = nrf_gpio_pins_read();
	
	if(btn_read_pin_state(TURBO_BUTTON))
	{
		turbo_button_count++;
	}
	else
	{
		turbo_button_count = 0;
	}
	
	if ((running_mode == GAZELL)&&(system_addr_received == false))
	{
			BETOP_BUTTON_VALUE = 1;
			TURBO_BUTTON_VALUE = 1;
			CLEAR_BUTTON_VALUE = 1;
	}
	else
	{
			BETOP_BUTTON_VALUE = 300;
			TURBO_BUTTON_VALUE = 300;
			CLEAR_BUTTON_VALUE = 300;
	}


	
	if(turbo_button_count > TURBO_BUTTON_VALUE)	// BLE 2.4G
	{
			turbo_button_count = 0;
			
			led_flash_time();

			mode_change_flag = true;
			my_role = 0;
	}
	
	if(btn_read_pin_state(CLEAR_BUTTON))
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
		
			if (running_mode == GAZELL)
			{
					NRF_POWER->GPREGRET = 0xB1;
			}
			else
			{
					err_code = sd_power_gpregret_clr(0xFF);
					APP_ERROR_CHECK(err_code);
					err_code = sd_power_gpregret_set(0xB1);
					APP_ERROR_CHECK(err_code);
			}
			
			
			led_flash_time();
			
			NVIC_SystemReset();
		
	}
	
	if(btn_read_pin_state(BETOP_BUTTON))
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
			app_timer_stop_all();
			nrf_gpio_pin_set(LED1_OUTPUT);
			nrf_gpio_pin_set(LED2_OUTPUT);
	}
	
}

void BETOP_switch_check(void)
{
	static uint32_t betop_button_flag = 0;
	
	nrf_gpio_cfg_input(BETOP_BUTTON,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(CLEAR_BUTTON,NRF_GPIO_PIN_PULLUP);
	
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



