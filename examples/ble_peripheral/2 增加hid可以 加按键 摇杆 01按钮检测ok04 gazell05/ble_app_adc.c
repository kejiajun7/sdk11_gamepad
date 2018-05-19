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

#include "ble_app_adc.h"
#include "io_reinit.h"
#include "nrf_gpio.h"

#include "app_util_platform.h"
#include "nrf_drv_adc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"


/**
 * @brief ADC initialization.
 */

/*****************************************************************************/
/** @name Configuration  ADC  init                                           */
/*****************************************************************************/
//#define ADC_BUFFER_SIZE 6                               //Size of buffer for ADC samples. Buffer size should be multiple of number of adc channels located.
//#define ADC_NUM_INIT 2
#define ADC_SAMPLE_RATE_DIVIDER				160000;           	//Sets the sampling rate
//#define ADC_SAMPLE_RATE_DIVIDER				16000;           	//Sets the sampling rate
//#define ADC_SAMPLE_RATE_DIVIDER				1600000;           	//Sets the sampling rate

static nrf_adc_value_t         adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
static nrf_ppi_channel_t       m_ppi_channel;
static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(1);

/*****************************************************************************/
/** @name Configuration  ADC  para                                           */
/*****************************************************************************/
//#define NRFR_DEVICE_PACKET_LENGTH 32
//static uint8_t device_packet[NRFR_DEVICE_PACKET_LENGTH];

static uint8_t adc_event_counter = 0;
static uint8_t number_of_adc_channels;

static uint16_t adc_value[ADC_BUFFER_SIZE];
//uint32_t value_pins = 0;
static uint8_t adc_value_state = 0;

bool time_for_send_flag = false;//Judge
uint16_t value_adc_change[ADC_BUFFER_SIZE];

/**
 * @brief ADC interrupt handler.
 */
static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        uint32_t i;
//        NRF_LOG_PRINTF("adc event: %d\r\n", adc_event_counter);
        for (i = 0; i < p_event->data.done.size; i++)
        {
//            NRF_LOG_PRINTF("ADC value channel %d: %d\r\n", (i % number_of_adc_channels), p_event->data.done.p_buffer[i]);
            adc_value[i] = p_event->data.done.p_buffer[i];
        }
				adc_event_counter++;
				CLOSE_KEY_VDD	
				time_for_send_flag = true;//360ms
    }
    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));

}

static void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
		OPEN_KEY_VDD		
}

static void adc_config(void)
{
    ret_code_t ret_code;
	
    //Initialize ADC
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);
	
    //Configure and enable ADC channel 2
    static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2); 
    m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_2_config);
	
    //Configure and enable ADC channel 3
    static nrf_drv_adc_channel_t m_channel_3_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_3); 
    m_channel_3_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_3_config);
	
    //Configure and enable ADC channel 4
    static nrf_drv_adc_channel_t m_channel_4_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_4); 
    m_channel_4_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_4_config);
 
    //Configure and enable ADC channel 5
    static nrf_drv_adc_channel_t m_channel_5_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_5); 
    m_channel_5_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_5_config);
	
    //Configure and enable ADC channel 6
    static nrf_drv_adc_channel_t m_channel_6_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_6);	
    m_channel_6_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_6_config);
 
    //Configure and enable ADC channel 7
    static nrf_drv_adc_channel_t m_channel_7_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_7);	
    m_channel_7_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_7_config);
		
    number_of_adc_channels = ADC_BUFFER_SIZE;    //Set equal to the number of configured ADC channels, for the sake of UART output.
}

static void adc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

static void adc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
	
//		err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);//sdk11 驱动里面默认有初始化
//		APP_ERROR_CHECK(err_code);

		nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG(1);
		timer_cfg.bit_width =  NRF_TIMER_BIT_WIDTH_16;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);
	
    /* setup m_timer for compare event */
    uint32_t ticks = ADC_SAMPLE_RATE_DIVIDER;
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t adc_sample_event_addr = nrf_drv_adc_start_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, adc_sample_event_addr);  //NRF_ADC->TASKS_START);
    APP_ERROR_CHECK(err_code);
}

void adc_init(void)
{
    adc_sampling_event_init();
    adc_config();
    adc_sampling_event_enable();

//    NRF_LOG_PRINTF(" ADC example\r\n");
//    NRF_LOG_FLUSH();

    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));

//    while (1);
}

void adc_disinit(void)
{
    //Initialize ADC
		nrf_drv_timer_uninit(&m_timer);
		nrf_drv_adc_uninit();
	  nrf_drv_ppi_channel_disable(m_ppi_channel);

}

/*
static uint8_t adc_change_data_get(void)
{
		#define DATA_CHANGE 	30
		#define DATA_CHANGE2 	10
		#define ADC_NUM 	ADC_BUFFER_SIZE
		uint8_t i;
//		static uint16_t adc_value_temp_init[ADC_NUM] = {0,0,451,452,430,434};
		static uint16_t adc_value_temp_init[ADC_NUM] = {0,0,431,430,430,430};
		uint8_t adc_value_change = 0;
		static bool adc_value_back[ADC_NUM] = 0;

		for (i = ADC_NUM_INIT; i < ADC_NUM; i++)
		{
			if((adc_value[i] > adc_value_temp_init[i] + DATA_CHANGE) || (adc_value[i] < adc_value_temp_init[i] - DATA_CHANGE))
			{
					adc_value_change |= (1 << i);
					adc_value_back[i] = true;
			}
			else
			{
					if(adc_value_back[i] == true)
					{
						adc_value_back[i] = false;
						adc_value_change |= (1 << i);
					}
			}
		}
		return adc_value_change;
}


static void mouse_sensor_new_sample_generated_cb()
{
		adc_value_state = adc_change_data_get();
}
*/

void adc_change_data_calculate(void)
{
//LEMON
		#define UP_VALUE 650
		#define DOWN_VALUE 180
		#define MIDDLE_LOW_VALUE 400
		#define MIDDLE_HIGH_VALUE 530
			
		#define DATA_CHANGE2 	10
		#define DATA_LIMIT_UP 	30
		#define DATA_LIMIT_DOWN 	30
	
		static uint16_t adc_value_temp[ADC_BUFFER_SIZE] = {0};
		static uint8_t i;
//		static uint16_t adc_value_temp2[ADC_NUM] = {0};

		
		for (i = 0; i < ADC_BUFFER_SIZE; i++)
		{
//			if((adc_value[i] > adc_value_temp[i] + DATA_CHANGE2) || (adc_value[i] < adc_value_temp[i] - DATA_CHANGE2))
//			if(adc_value[i] != adc_value_temp[i])
//			{
//					adc_value_temp[i] = adc_value[i];
//			}
					adc_value_temp[i] = adc_value[i];
		}


//    static uint16_t  battery_value;
//    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
//    battery_level = adc_value[0]*1200/1023*3*4/3;
//	if(adc_value_temp[0]>0)
//		battery_value = adc_value_temp[0]*4.692;//mv
//	else
//		battery_value = 0;//mv
//	
//	battery_level = battery_value/50;//
	
		value_adc_change[0] = adc_value_temp[0];
		value_adc_change[1] = adc_value_temp[1];
		
//		if(adc_value_state)
//		{
			
			i = ADC_NUM_INIT;
			
//			if(adc_value_state & ( 1 << i ))
//			{
						if(adc_value_temp[i] <= DOWN_VALUE)					value_adc_change[i] = 0;
						else if(adc_value_temp[i] >= UP_VALUE)			value_adc_change[i] = 0xFF;
						else if((adc_value_temp[i] >= MIDDLE_LOW_VALUE) && (adc_value_temp[i] <= MIDDLE_HIGH_VALUE))	
								value_adc_change[i] = 0x80;				
						else  if((adc_value_temp[i] < MIDDLE_LOW_VALUE) && (adc_value_temp[i] > DOWN_VALUE))
		//				    value_adc_change[i] = ((adc_value_temp[i] - DOWN_VALUE) / ((float)(MIDDLE_LOW_VALUE - DOWN_VALUE)/128));
								value_adc_change[i] = ((adc_value_temp[i] - DOWN_VALUE)<<7) / 220;
						else  if((adc_value_temp[i] < UP_VALUE) && (adc_value_temp[i] > MIDDLE_HIGH_VALUE))
		//				    value_adc_change[i] = (0x80 + ((adc_value_temp[i] - MIDDLE_HIGH_VALUE) / ((float)(UP_VALUE - MIDDLE_HIGH_VALUE)/128)));
								value_adc_change[i] = ((adc_value_temp[i] - MIDDLE_HIGH_VALUE)<<7 ) / 120 + 0x80;
						
						
//				adc_value_state &= ~( 1 << i );
//			}
//			else
//			{
//				value_adc_change[i] = 0x80;
//			}
			
			i++;
			
//			if(adc_value_state & ( 1 << i ))
//			{
				
				if(adc_value_temp[i] <= DOWN_VALUE)
					value_adc_change[i] = 0xFF;
				else if(adc_value_temp[i] >= UP_VALUE)
					value_adc_change[i] = 0;
				else if((adc_value_temp[i] >= MIDDLE_LOW_VALUE) && (adc_value_temp[i] <= MIDDLE_HIGH_VALUE))	
				  value_adc_change[i] = 0x80;				
				else  if((adc_value_temp[i] < MIDDLE_LOW_VALUE) && (adc_value_temp[i] > DOWN_VALUE))
//				  value_adc_change[i] = (255-((adc_value_temp[i] - DOWN_VALUE) / ((float)(MIDDLE_LOW_VALUE - DOWN_VALUE)/128)));
				  value_adc_change[i] = 255 - (((adc_value_temp[i] - DOWN_VALUE)<<7) / 220);
				else  if((adc_value_temp[i] < UP_VALUE) && (adc_value_temp[i] > MIDDLE_HIGH_VALUE))
//				  value_adc_change[i] = (128-((adc_value_temp[i] - MIDDLE_HIGH_VALUE) / ((float)(UP_VALUE - MIDDLE_HIGH_VALUE) / 128)));
				  value_adc_change[i] = 128-(((adc_value_temp[i] - MIDDLE_HIGH_VALUE)<<7) / 120);

//				adc_value_state &= ~( 1 << i );
//			}
//			else
//			{
//				value_adc_change[i] = 0x80;
//			}
			
			i++;
			
//			if(adc_value_state & ( 1 << i ))
//			{
				
				if(adc_value_temp[i] <= DOWN_VALUE)					value_adc_change[i] = 0;
				else if(adc_value_temp[i] >= UP_VALUE)			value_adc_change[i] = 0xFF;
				else if((adc_value_temp[i] >= MIDDLE_LOW_VALUE) && (adc_value_temp[i] <= MIDDLE_HIGH_VALUE))	
				    value_adc_change[i] = 0x80;				
				else  if((adc_value_temp[i] < MIDDLE_LOW_VALUE) && (adc_value_temp[i] > DOWN_VALUE))
//				    value_adc_change[i] = ((adc_value_temp[i] - DOWN_VALUE) / ((float)(MIDDLE_LOW_VALUE - DOWN_VALUE)/128));
				    value_adc_change[i] = ((adc_value_temp[i] - DOWN_VALUE)<<7) / 220;
				else  if((adc_value_temp[i] < UP_VALUE) && (adc_value_temp[i] > MIDDLE_HIGH_VALUE))
//				    value_adc_change[i] = (0x80 + ((adc_value_temp[i] - MIDDLE_HIGH_VALUE) / ((float)(UP_VALUE - MIDDLE_HIGH_VALUE)/128)));
				    value_adc_change[i] = ((adc_value_temp[i] - MIDDLE_HIGH_VALUE)<<7 ) / 120 + 0x80;

//				adc_value_state &= ~( 1 << i );
//			}
//			else
//			{
//				value_adc_change[i] = 0x80;
//			}
			
			i++;

//			if(adc_value_state & ( 1 << i ))
//			{
				
				if(adc_value_temp[i] <= DOWN_VALUE)
					value_adc_change[i] = 0xFF;
				else if(adc_value_temp[i] >= UP_VALUE)
					value_adc_change[i] = 0;
				else if((adc_value_temp[i] >= MIDDLE_LOW_VALUE) && (adc_value_temp[i] <= MIDDLE_HIGH_VALUE))	
				  value_adc_change[i] = 0x80;				
				else  if((adc_value_temp[i] < MIDDLE_LOW_VALUE) && (adc_value_temp[i] > DOWN_VALUE))
//				  value_adc_change[i] = (255-((adc_value_temp[i] - DOWN_VALUE) / ((float)(MIDDLE_LOW_VALUE - DOWN_VALUE)/128)));
				  value_adc_change[i] = 255 - (((adc_value_temp[i] - DOWN_VALUE)<<7) / 220);
				else  if((adc_value_temp[i] < UP_VALUE) && (adc_value_temp[i] > MIDDLE_HIGH_VALUE))
//				  value_adc_change[i] = (128-((adc_value_temp[i] - MIDDLE_HIGH_VALUE) / ((float)(UP_VALUE - MIDDLE_HIGH_VALUE) / 128)));
				  value_adc_change[i] = 128-(((adc_value_temp[i] - MIDDLE_HIGH_VALUE)<<7) / 120);
				
//				adc_value_state &= ~( 1 << i );
//			}
//			else
//			{
//				value_adc_change[i] = 0x80;
//			}
		
//		}
//		else
//		{
//				for (i = ADC_NUM_INIT; i < ADC_NUM; i++)
//				{
//						value_adc_change[i] = 0x80;
//				}
//		}
				
//		for (i = ADC_NUM_INIT; i < ADC_NUM; i++)
//		{
//				if(value_adc_change[i] != adc_value_temp2[i])
//				{
//						adc_value_temp2[i] = value_adc_change[i];
//						adc_value_state |= (1 << i);
//				}
//				else
//				{
//						adc_value_state &= ~(1 << i);
//				}
//		}				
				
}


//uint8_t btn_read_pin_state(uint8_t button)
//{
//    return (NRF_GPIO->IN & (1 << button)) == 0;
//}






/** 
 * @}
 */
