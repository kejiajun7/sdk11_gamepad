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
 * @{
 * @ingroup ble_app_gzll_gazell_part
 */

#include "ble_app_gzll_device.h"
#include <stdint.h>
#include "nordic_common.h"
#include "app_error.h"
#include "nrf_gzll.h"
#include "ble_app_gzll_ui.h"
#include "bsp.h"

#include "io_reinit.h"
#include "ble_app_adc.h"
#include "ble_app_gzll_common.h"

#include "nrf_gzll.h"
#include "nrf_gzp_JUDGE.h"
#include "nrf_gzllde_params.h"

uint32_t send_data_count = 0;		//Judge
uint32_t send_data_count_temp = 0;		//Judge
uint32_t send_data_count_gazell = 0;		//Judge

uint32_t send_success_count = 0;//Judge 变量异常不能使用很奇怪
uint32_t receive_data_count = 0;//Judge
uint32_t send_fail_count = 0;		//Judge

uint8_t	pipe_num_assign = 0;				//Judge
uint8_t	pipe_use_state = 0;						//Judge
uint8_t	CMD_Value = 0;							//Judge

 bool system_addr_received = false;     ///< System address receivedfrom Host.
static uint8_t send_change_flag = 0;

static uint8_t device_packet[32];

static uint32_t pins_value_temp;
void send_24_mode_communication(void);

static bool btn_read_pin_state(uint8_t pin_number)
{
//    return  ((NRF_GPIO->IN >> pin_number) & 1UL);
    return (pins_value_temp & (1 << pin_number)) == 0;
}

void read_mouse_and_send2(void)
{
//lint -save -e514 Unusual use of a boolean expression (use of &= assignment).
	static uint8_t value_temp = 0,i = 0;
	
	static uint8_t device_packet_temp[7];//Judge
	
    // "Scan" keyboard.
		pins_value_temp = nrf_gpio_pins_read();

		adc_change_data_calculate();

		device_packet[4] = value_adc_change[3 + ADC_NUM_INIT];//y1  up down
		device_packet[3] = value_adc_change[2 + ADC_NUM_INIT];//x1  left right
		device_packet[6] = value_adc_change[1 + ADC_NUM_INIT];//y2	up down
		device_packet[5] = value_adc_change[0 + ADC_NUM_INIT];//x2	left right
		

		device_packet[0] = ((btn_read_pin_state(Y_BUTTON)) << 0) | ((btn_read_pin_state(B_BUTTON)) << 1) \
		|((btn_read_pin_state(A_BUTTON))  << 2)	| ((btn_read_pin_state(X_BUTTON))  << 3) \
		|((btn_read_pin_state(LB_BUTTON)) << 4) | ((btn_read_pin_state(RB_BUTTON)) << 5) \
		|((btn_read_pin_state(LT_BUTTON)) << 6) | ((btn_read_pin_state(RT_BUTTON)) << 7) ;
		
		device_packet[1] = ((btn_read_pin_state(BACK_BUTTON)) << 0) |((btn_read_pin_state(START_BUTTON)) << 1) \
		|((btn_read_pin_state(B1_BUTTON)) << 2) | ((btn_read_pin_state(B2_BUTTON)) << 3) \
		|((btn_read_pin_state(BETOP_BUTTON)) << 4) | (0 << 5) \
		|(0 << 6) | (0 << 7) ;
		
		
		value_temp = ((nrf_gpio_pin_read(LEFT_BUTTON)) << 3) | ((nrf_gpio_pin_read(RIGHT_BUTTON)) << 1) | ((nrf_gpio_pin_read(UP_BUTTON)) << 0) | ((nrf_gpio_pin_read(DOWN_BUTTON)) << 2);
		switch(value_temp)
		{
			case 0x0f:
				device_packet[2] = 0x0f;
				break;
			case 0x0e:
				device_packet[2] = 0x00;
				break;
			case 0x0d:
				device_packet[2] = 0x02;
				break;
			case 0x0b:
				device_packet[2] = 0x04;
				break;
			case 0x07:
				device_packet[2] = 0x06;
				break;
			case 0x0c:
				device_packet[2] = 0x01;
				break;
			case 0x09:
				device_packet[2] = 0x03;
				break;
			case 0x03:
				device_packet[2] = 0x05;
				break;
			case 0x06:
				device_packet[2] = 0x07;
				break;
			default:
				device_packet[2] = 0x0f;
				break;
				
		}
		device_packet[7] = 0xff * btn_read_pin_state(RIGHT_BUTTON);
		device_packet[8] = 0xff * btn_read_pin_state(LEFT_BUTTON);
		device_packet[9] = 0xff * btn_read_pin_state(UP_BUTTON);
		device_packet[10] = 0xff * btn_read_pin_state(DOWN_BUTTON);
		device_packet[11] = 0xff * btn_read_pin_state(Y_BUTTON);
		device_packet[12] = 0xff * btn_read_pin_state(B_BUTTON);
		device_packet[13] = 0xff * btn_read_pin_state(A_BUTTON);
		device_packet[14] = 0xff * btn_read_pin_state(X_BUTTON);
		device_packet[15] = 0xff * btn_read_pin_state(LB_BUTTON);
		device_packet[16] = 0xff * btn_read_pin_state(RB_BUTTON);
		device_packet[17] = 0xff * btn_read_pin_state(LT_BUTTON);
		device_packet[18] = 0xff * btn_read_pin_state(RT_BUTTON);
		
			device_packet[19] ^= 0x01;
			device_packet[20] = 0x02;
			device_packet[21] = 0x90;
			device_packet[22] = 0x01;
			device_packet[23] ^= 0x01;
			device_packet[24] = 0x02;
			device_packet[25] = 0x00;
			device_packet[26] = 0x02;

	for (i = 0; i < 7; i++)
	{
		if(device_packet_temp[i]!=device_packet[i])
		{
			device_packet_temp[i]=device_packet[i];
			send_change_flag = 1;
			send_data_count++;
		}
	}
//	for (i = 7; i < 18; i++)
//	{
//		if(device_packet[i]==0xff)
//		{
//			send_change_flag = 1;
//			break;
//		}
//	}
//			send_change_flag = 1;
	
//lint -restore
}

void gazell_mode_init(void)
{

#define MAX_TX_ATTEMPTS (NRF_GZLL_DEFAULT_TIMESLOTS_PER_CHANNEL_WHEN_DEVICE_OUT_OF_SYNC * \
                         NRF_GZLL_DEFAULT_CHANNEL_TABLE_SIZE)
	
	
//#define NRF_GZLL_JUDGE_CHANNEL_TABLE  {2, 49 ,79}       ///< Default channel table.
//#define NRF_GZLL_JUDGE_CHANNEL_TABLE_SIZE   3                   ///< Default channel table size.
#define NRF_GZLL_JUDGE_CHANNEL_TABLE  {4, 25, 42, 63, 77}       ///< Default channel table.
#define NRF_GZLL_JUDGE_CHANNEL_TABLE_SIZE   5                   ///< Default channel table size.

    bool     result_value = false;
    uint32_t err_code;
		static uint8_t gzll_channel_tab[NRF_GZLL_JUDGE_CHANNEL_TABLE_SIZE] = NRF_GZLL_JUDGE_CHANNEL_TABLE;

	// Initialize and enable Gazell
    result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
//    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Ensure Gazell parameters are configured.
    result_value = nrf_gzll_set_max_tx_attempts(150);
//    result_value = nrf_gzll_set_max_tx_attempts(0);
//    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_device_channel_selection_policy(
        NRF_GZLLDE_DEVICE_CHANNEL_SELECTION_POLICY);
//    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2);
//    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_sync_lifetime(0); // Asynchronous mode, more efficient for pairing.
//    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_tx_power(GZP_POWER);

		result_value = nrf_gzll_set_channel_table(gzll_channel_tab,NRF_GZLL_JUDGE_CHANNEL_TABLE_SIZE);

    gzp_init();//get system address + host id

    result_value = nrf_gzll_enable();
//    GAZELLE_ERROR_CODE_CHECK(result_value);

		
//		NRF_LOG_PRINTF("first time to pair.\r\n");
//		uint32_t count = 0;
		
//		while((mode_change_flag == false)&&(!system_addr_received))
		while(!system_addr_received)
    {
					system_addr_received = gzp_address_req_send();
					if(pipe_num_assign == 0)	//will disturb other device
						system_addr_received = false;
					
				if(time_for_send_flag == true)
				{
					time_for_send_flag = false;
					key_sense_turbo();
				}
				
				if(mode_change_flag == true)//goto ble
				{
					break;
				}
				
				if(goto_system_off_flag)
				{
						if(nrf_gpio_pin_read(BETOP_BUTTON)==1)
						{
								goto_system_off_flag = false;
								goto_syetem_off();
						}
				}
				
//				count ++;
//				if(count > 20)    //10 = 14s  
//					goto_syetem_off();
				
//				communication_fail_check();
    }	
		
//		if((system_addr_received==false)||(pipe_num_assign == 0))
//		{
//		}
		
		if(mode_change_flag == true)//goto ble
		{
				mode_change_flag = false;
				running_mode = BLE;    
		}
		else
		{
			my_role = pipe_num_assign;
			
			// Clean-up and restore parameters temporarily  modified
			nrf_gzp_disable_gzll();
			
//			result_value = nrf_gzll_set_max_tx_attempts(NRF_GZLL_DEFAULT_TIMESLOTS_PER_CHANNEL_WHEN_DEVICE_OUT_OF_SYNC * NRF_GZLL_DEFAULT_CHANNEL_TABLE_SIZE);
//			result_value = nrf_gzll_set_sync_lifetime(NRF_GZLL_DEFAULT_SYNC_LIFETIME);
//			result_value = nrf_gzll_set_timeslots_per_channel_when_device_out_of_sync(NRF_GZLL_DEFAULT_TIMESLOTS_PER_CHANNEL_WHEN_DEVICE_OUT_OF_SYNC);

			result_value = nrf_gzll_set_max_tx_attempts(5);
			result_value = nrf_gzll_set_sync_lifetime(0); // Asynchronous mode, more efficient for pairing.
			result_value = nrf_gzll_set_timeslots_per_channel_when_device_out_of_sync(5);
			nrf_gzll_enable();
		
			send_24_mode_communication();		
		}
		
//		if(pipe_num_assign == 0)
//		{
//		}
//		else
//		{
//		}
	
}

void send_24_mode_communication(void)
{
	static uint16_t time_count_flag = 0;
	bool tx_success = false;
	uint32_t i;
	uint32_t fail_count = 0;
	uint32_t fail_count_total = 0;

				for (;;)
				{
						if(time_for_send_flag == true)
						{
							time_for_send_flag = false;
							time_count_flag++;
							read_mouse_and_send2();
							key_sense_turbo();
						}
						
						if(mode_change_flag == true)//goto ble
						{
							mode_change_flag = false;
							running_mode = BLE;    
							break;
						}
						
						if(goto_system_off_flag)
						{
								if(btn_read_pin_state(BETOP_BUTTON)==0)
								{
										goto_system_off_flag = false;
										goto_syetem_off();
								}
						}
						
							if(time_count_flag>200)
							{
								time_count_flag=0;
								send_change_flag = 1;
							}
				
						if(send_change_flag)
						{
								send_change_flag = 0;
							
								device_packet[24] = (uint8_t)(fail_count_total >> 8);
								device_packet[25] = (uint8_t)fail_count_total;
								device_packet[26] = pipe_num_assign;
						//		device_packet[26] ++;
								
							// Wait in case the FIFOs are full.
								while(!nrf_gzll_ok_to_add_packet_to_tx_fifo(pipe_num_assign))
								{
									i++;
									if(i > 10000)//1000000 = 7s  10000=4s
									{
											i = 0;
											my_role = 0;
											fail_count ++;
											fail_count_total++;
											if(fail_count > 30)//15=15s
											{
												fail_count = 0;
											}
									}										
//									system_off_button_check();											
								}
								tx_success = nrf_gzll_add_packet_to_tx_fifo(pipe_num_assign, device_packet, 32);
								
								if(tx_success)
								{
										my_role = pipe_num_assign;
										fail_count = 0;
										i = 0;
										if(send_data_count_temp!=send_data_count)
										{
											send_data_count_temp=send_data_count;
											send_data_count_gazell++;
										}
								}
								else
								{
								}	
								
						}
						
//						__WFE();

//						// Clear Event Register.
//						__SEV();
//						__WFE();
				}
	
}


/**@brief Function for starting Gazell functionality.
 */
void gzll_app_start(void)
{
		gazell_mode_init();
}


void gzll_app_stop()
{
    // Disable gazell.
		nrf_gzp_disable_gzll();
//    nrf_gzll_disable();
    
    // Wait for Gazell to shut down
//    while (nrf_gzll_is_enabled())
//    {
//        // Do nothing.
//    }
    
    // Clean up after Gazell.
    NVIC_DisableIRQ(RADIO_IRQn);
    NVIC_DisableIRQ(TIMER2_IRQn);   
}




/** 
 * @}
 */
