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

#include "ble_app_store.h"
#include "io_reinit.h"
#include "nrf_gpio.h"
#include "ble_app_gzll_ui.h"


#include "pstorage.h"

#define GZP_INDEX_DB_SIZE 1024   ///<
#define GZP_PARAMS_STORAGE_ADR 	(PSTORAGE_DATA_START_ADDR - PSTORAGE_FLASH_PAGE_SIZE * 3)
#define GZP_INDEX_DB_ADR 	GZP_PARAMS_STORAGE_ADR //SET address

static void gzp_index_db_add(uint8_t val)//xian low hou high
{
    int16_t i;
    uint8_t temp_val;
    uint32_t  addr;

    // Search for unwritten loacation in index DB
    for (i = 0; i < GZP_INDEX_DB_SIZE; i++)
    {
        temp_val = *(uint8_t*)(GZP_INDEX_DB_ADR + i);
			
        // Lower nibble
        if (i != (GZP_INDEX_DB_SIZE - 1))
        {
            if(temp_val == 0xff)
            {
//                temp_val = val;
                break;
            }

        }
        else
        {
            if(temp_val == 0xff)
            {
//                temp_val = val;
                break;
            }
						else
						{
								nrf_nvmc_page_erase((uint32_t)GZP_PARAMS_STORAGE_ADR);
//								temp_val = val;//FULL
								i = 0;
                break;
						}
        }
    }

		temp_val = val;//FULL
    // Write index DB
    addr = (GZP_INDEX_DB_ADR + i);
    nrf_nvmc_write_byte(addr, temp_val);
}

static uint8_t gzp_index_db_read()//xian low hou high
{
    uint8_t retval;
    int16_t i;

    // Search for previously written location
    for (i = (GZP_INDEX_DB_SIZE - 1); i >= 0; i--)
    {
        retval = *(uint8_t*)(GZP_INDEX_DB_ADR + i);

        if (retval != 0xff)
        {
          break;
        }
    }

    if (retval == 0xff)
    {
        retval = 0xff;  // index db empty
    }


    return retval;
}

void mode_store(radio_mode_t run_mode)
{
	if(running_mode == BLE)
	{
			gzp_index_db_add(0xfe);	
	}
	else
	{
			gzp_index_db_add(0xfc);	
	}
}

void com_mode_check(void)
{
//	uint8_t output_dat[4];
	uint8_t i;
//	static pstorage_size_t m_block_num;
//	static pstorage_size_t m_flash_offset;
//	m_block_num=0;//first block
//	m_flash_offset=0;
//	
//	register_flash();
//	load_flash(m_block_num,output_dat,sizeof(output_dat),m_flash_offset);
//	if(output_dat[0] == 0x01)
	
  i = gzp_index_db_read();
	
	if(i == 0xff)//first
	{
		gzp_index_db_add(0xfe);	
//		mode_change_flag = false;//2.4g
//		my_role = 0;
		running_mode = BLE;
	}
	else
	{
		if((i & 0x02) == 0x02)
		{
			running_mode = BLE;
		}
		else
		{
			running_mode = GAZELL;
		}
	}

	
}



/** 
 * @}
 */
