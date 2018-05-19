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
 * @ingroup ble_app_gzll_bluetooth_part
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

//#include "softdevice_handler.h"
//#include "app_timer.h"
//#include "boards.h"
#include "ble_app_gzll_ui.h"
#include "ble_app_gzll_hid.h"
#include "ble_app_gzll_common.h"

#include "ble_app_adc.h"
#include "io_reinit.h"

#if BUTTONS_NUMBER <2
#error "Not enough resources on board"
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                              /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                              /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                              /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/


#define DEVICE_NAME                      "GAMEPAD_XT102"                              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                          /**< Manufacturer. Will be passed to Device Information Service. */


APP_TIMER_DEF(m_battery_timer_id);                                                      /**< Battery timer. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)     /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                81                                             /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                                            /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                              /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE          0x02                                           /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                 0x1915                                         /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                0xEEEE                                         /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION           0x0001                                         /**< Product Version. */

#define APP_ADV_FAST_INTERVAL            0x0028                                         /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL            0x0C80                                         /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT             30                                             /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT             180                                            /**< The duration of the slow advertising period (in seconds). */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(7.5, UNIT_1_25_MS)               /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(30, UNIT_1_25_MS)                /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                    6                                              /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(430, UNIT_10_MS)                 /**< Connection supervisory timeout (430 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)    /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                              /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                              /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                              /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                              /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                           /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                             /**< Maximum encryption key size. */


/*lint hid keyboard report*/
#define INPUT_REPORT_KEYS_INDEX          0                                              /**< Index of Input Report. */
#define INPUT_REP_REF_ID                 1                                              /**< Id of reference to Keyboard Input Report. */
#define INPUT_REPORT_KEYS_MAX_LEN        8                                              /**< Maximum length of the Input Report characteristic. */
#define OUTPUT_REPORT_INDEX              0                                              /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN            1                                              /**< Maximum length of Output Report. */
#define OUTPUT_REP_REF_ID                0                                              /**< Id of reference to Keyboard Output Report. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK 0x02                                           /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */

/*lint hid - keyboard code position */
#define MODIFIER_KEY_POS                 0                                              /**< Position of the modifier byte in the Input Report. */
#define SCAN_CODE_POS                    2                                              /**< This macro indicates the start position of the key scan code in a HID Report. As per the document titled 'Device Class Definition for Human Interface Devices (HID) V1.11, each report shall have one modifier byte followed by a reserved constant byte and then the key scan code. */
#define SHIFT_KEY_CODE                   0x02                                           /**< Key code indicating the press of the Shift Key. */
#define MAX_KEYS_IN_ONE_REPORT           (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS)    /**< Maximum number of key presses that can be sent in one Input Report. */

#define KEY_PRESS_BUTTON_ID              0                                              /**< Button used as Keyboard key press. */
#define SHIFT_BUTTON_ID                  1                                              /**< Button used as 'SHIFT' Key. */

/*lint hid comsumer control report*/
#define INPUT_CCONTROL_KEYS_INDEX		 1								//Judge
#define INPUT_CC_REP_REF_ID					 2								//Judge
#define INPUT_CC_REPORT_KEYS_MAX_LEN			5

/*lint hid gamepad report*/
#define INPUT_GAMEPAD_KEYS_INDEX		 2								//Judge
#define INPUT_GAMEPAD_REP_REF_ID	   3								//Judge	
#define INPUT_GAMEPAD_REPORT_KEYS_MAX_LEN			2				//Judge

/*lint hid gamepad gamesir report*/
#define INPUT_GAMESIR_KEYS_INDEX		 3								//Judge
#define INPUT_GAMESIR_REP_REF_ID	   4								//Judge	
#define INPUT_GAMESIR_REPORT_KEYS_MAX_LEN			12			//Judge
#define OUTPUT_GAMESIR_REPORT_INDEX              1    //Judge                                          /**< Index of Output Report. */
#define OUTPUT_GAMESIR_REPORT_MAX_LEN            1    //Judge                                          /**< Maximum length of Output Report. */
#define OUTPUT_GAMESIR_REP_REF_ID                5    //Judge                                          /**< Id of reference to Keyboard Output Report. */



#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2            /**< Reply when unsupported features are requested. */

#define MAX_BUFFER_ENTRIES               5                                              /**< Number of elements that can be enqueued */

#define BASE_USB_HID_SPEC_VERSION        0x0101                                         /**< Version number of base USB HID Specification implemented by this application. */






#define KEY_PRESS_BUTTON_PIN_NO          BSP_BUTTON_2                                       /**< Button used for sending keyboard text. */
#define VOLDOWN_BUTTON                   BSP_BUTTON_1
#define VOLUP_BUTTON                     BSP_BUTTON_0

#define ADVERTISING_LED_PIN_NO           BSP_LED_0                                          /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO             BSP_LED_1                                          /**< Is on when device has connected. */
#define ASSERT_LED_PIN_NO                BSP_LED_3                                          /**< Is on when application has asserted. */

#define LED_OFF(x)                       nrf_gpio_pin_set(x)
#define LED_ON(x)                        nrf_gpio_pin_clear(x)



/**Buffer queue access macros
 *
 * @{ */
/** Initialization of buffer list */
#define BUFFER_LIST_INIT()                                                                        \
        do                                                                                        \
        {                                                                                         \
            buffer_list.rp = 0;                                                                   \
            buffer_list.wp = 0;                                                                   \
            buffer_list.count = 0;                                                                \
        } while (0)

/** Provide status of data list is full or not */
#define BUFFER_LIST_FULL()\
        ((MAX_BUFFER_ENTRIES == buffer_list.count - 1) ? true : false)

/** Provides status of buffer list is empty or not */
#define BUFFER_LIST_EMPTY()\
        ((0 == buffer_list.count) ? true : false)

#define BUFFER_ELEMENT_INIT(i)\
        do                                                                                        \
        {                                                                                         \
            buffer_list.buffer[(i)].p_data = NULL;                                                \
        } while (0)

/** @} */

/** Abstracts buffer element */
typedef struct hid_key_buffer
{
    uint8_t    data_offset;   /**< Max Data that can be buffered for all entries */
    uint8_t    data_len;      /**< Total length of data */
    uint8_t    * p_data;      /**< Scanned key pattern */
    ble_hids_t * p_instance;  /**< Identifies peer and service instance */
}buffer_entry_t;

STATIC_ASSERT(sizeof(buffer_entry_t) % 4 == 0);

/** Circular buffer list */
typedef struct
{
    buffer_entry_t buffer[MAX_BUFFER_ENTRIES]; /**< Maximum number of entries that can enqueued in the list */
    uint8_t        rp;                         /**< Index to the read location */
    uint8_t        wp;                         /**< Index to write location */
    uint8_t        count;                      /**< Number of elements in the list */
}buffer_list_t;

STATIC_ASSERT(sizeof(buffer_list_t) % 4 == 0);

/** List to enqueue not just data to be sent, but also related information like the handle, connection handle etc */
static buffer_list_t buffer_list;


typedef enum
{
    BLE_NO_ADV,               /**< No advertising running. */
    BLE_DIRECTED_ADV,         /**< Direct advertising to the latest central. */
    BLE_FAST_ADV_WHITELIST,   /**< Advertising with whitelist. */
    BLE_FAST_ADV,             /**< Fast advertising running. */
    BLE_SLOW_ADV,             /**< Slow advertising running. */
    BLE_SLEEP,                /**< Go to system-off. */
} ble_advertising_mode_t;

static ble_hids_t                        m_hids;                                        /**< Structure used to identify the HID service. */
static ble_bas_t                         m_bas;                                         /**< Structure used to identify the battery service. */
static bool                              m_in_boot_mode = false;                        /**< Current protocol mode. */
static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;       /**< Handle of the current connection. */

static sensorsim_cfg_t                   m_battery_sim_cfg;                             /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t                 m_battery_sim_state;                           /**< Battery Level sensor simulator state. */


static dm_application_instance_t         m_app_handle;                                  /**< Application identifier allocated by device manager. */
static dm_handle_t                       m_bonded_peer_handle;                          /**< Device reference handle to the current bonded central. */
static bool                              m_caps_on = false;                             /**< Variable to indicate if Caps Lock is turned on. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}};

static uint8_t m_sample_key_press_scan_str[] =                                          /**< Key pattern to be sent when the key press button has been pushed. */
{
    0x0b, /* Key h */
    0x08, /* Key e */
    0x0f, /* Key l */
    0x0f, /* Key l */
    0x12, /* Key o */
    0x28  /* Key Return */
};

static uint8_t m_caps_on_key_scan_str[] =                                                /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit set. */
{
    0x06, /* Key C */
    0x04, /* Key a */
    0x13, /* Key p */
    0x16, /* Key s */
    0x12, /* Key o */
    0x11, /* Key n */
};

static uint8_t m_caps_off_key_scan_str[] =                                               /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit cleared. */
{
    0x06, /* Key C */
    0x04, /* Key a */
    0x13, /* Key p */
    0x16, /* Key s */
    0x12, /* Key o */
    0x09, /* Key f */
};


static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */


///////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t device_packet[INPUT_GAMESIR_REPORT_KEYS_MAX_LEN];//Judge
uint8_t GPIOTE_PORT_EVENT_FLAG = 0;															//Judge
bool tx_complete_flag;

//static uint8_t data_array[40];
bool goto_disconnect_event_flag = false;
bool goto_off_flag = false;





/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for performing a battery measurement, and updating the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;
    uint16_t  battery_value;

//    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
//    battery_level = adc_value[0]*1200/1023*3*4/3;
	if(value_adc_change[0]>0)
		battery_value = value_adc_change[0]*4.692;//mv
	else
		battery_value = 0;//mv
	
	battery_level = battery_value/50;//

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}



/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, 
                                          (const uint8_t *)DEVICE_NAME, 
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
    APP_ERROR_CHECK(err_code);
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval   = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval   = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency       = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout    = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    uint32_t         err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;
	
#define HW_STRING "XT-GAMEPAD HW V1.0"	//"Hardware Revision String" 
#define SW_STRING "XT-GAMEPAD SW V1.0"	//"Software Revision String" 
#define FW_STRING "XT-GAMEPAD FW V1.0"	//"Firmware Revision String" 
#define SN_STRING "XT-GAMEPAD SN 00000000"	//"Serial Number String" 
#define MN_STRING "XT-GAMEPAD MN 00000000"	//"Model Number String" 
	
//    ble_srv_ascii_to_utf8(&dis_init_obj.hw_rev_str, HW_STRING);
//    ble_srv_ascii_to_utf8(&dis_init_obj.sw_rev_str, SW_STRING);
//    ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, FW_STRING);
//    ble_srv_ascii_to_utf8(&dis_init_obj.serial_num_str, SN_STRING);
//    ble_srv_ascii_to_utf8(&dis_init_obj.model_num_str, MN_STRING);
	
	
//		ble_dis_sys_id_t sys_id;
//		sys_id.manufacturer_id = ;
//		sys_id.organizationally_unique_id = ;
//    dis_init_obj.p_sys_id = &sys_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


static void nus_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    uint32_t                   err_code;
    ble_hids_init_t            hids_init_obj;
    ble_hids_inp_rep_init_t    input_report_array[4];
    ble_hids_inp_rep_init_t  * p_input_report;
    ble_hids_outp_rep_init_t   output_report_array[2];
    ble_hids_outp_rep_init_t * p_output_report;
    uint8_t                    hid_info_flags;

    memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));
    memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));
    
    static uint8_t report_map_data[] =
    {
        0x05, 0x01,                 // Usage Page (Generic Desktop)
        0x09, 0x06,                 // Usage (Keyboard)
        0xA1, 0x01,                 // Collection (Application)
			
				0x85, 0x01,                     //     Report Id (1)
        0x05, 0x07,                 //     Usage Page (Key Codes)
        0x19, 0xe0,                 //     Usage Minimum (224)
        0x29, 0xe7,                 //     Usage Maximum (231)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x01,                 //     Logical Maximum (1)
        0x75, 0x01,                 //     Report Size (1)
        0x95, 0x08,                 //     Report Count (8)
        0x81, 0x02,                 //     Input (Data, Variable, Absolute)

        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x08,                 //     Report Size (8)
        0x81, 0x01,                 //     Input (Constant) reserved byte(1)

        0x05, 0x08,                 //     Usage Page (Page# for LEDs)
        0x95, 0x05,                 //     Report Count (5)
        0x75, 0x01,                 //     Report Size (1)
        0x19, 0x01,                 //     Usage Minimum (1)
        0x29, 0x05,                 //     Usage Maximum (5)
        0x91, 0x02,                 //     Output (Data, Variable, Absolute), Led report
        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x03,                 //     Report Size (3)
        0x91, 0x01,                 //     Output (Data, Variable, Absolute), Led report padding

        0x05, 0x07,                 //     Usage Page (Key codes)
        0x95, 0x06,                 //     Report Count (6)
        0x75, 0x08,                 //     Report Size (8)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x65,                 //     Logical Maximum (101)
        0x19, 0x00,                 //     Usage Minimum (0)
        0x29, 0x65,                 //     Usage Maximum (101)
        0x81, 0x00,                 //     Input (Data, Array) Key array(6 bytes)

        0x09, 0x05,                 //     Usage (Vendor Defined)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x26, 0xFF, 0x00,           //     Logical Maximum (255)
        0x75, 0x08,                 //     Report Count (2)
        0x95, 0x02,                 //     Report Size (8 bit)
        0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)

        0xC0,                        // End Collection (Application)

        // Report ID 2: Advanced buttons
				0x05,0x0C,
				0x09,0x01,
				0xA1,0x01,
				0x85,0x02,
				
				0x09, 0x05,                 //     Usage (Vendor Defined)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x26, 0xFF, 0x00,           //     Logical Maximum (255)
        0x75, 0x08,                 //     Report Count (2)
        0x95, 0x02,                 //     Report Size (8 bit)
        0x81, 0x02,                 //     Input (Data, Variable, Absolute)


				0x15,0x00,
				0x25,0x01,
				0x75,0x01,
				0x95,0x15, 
				0x0A,0x94,0x01,
				0x0A,0x92,0x01,
				0x0A,0x83,0x01,
				0x0A,0x23,0x02,
				0x0A,0x8A,0x01,
				0x0A,0xB1,0x01,
				0x0A,0x21,0x02,
				0x0A,0x24,0x02,
				0x0A,0x25,0x02,
				0x0A,0x2A,0x02,
				0x0A,0x27,0x02,
				0x09,0xB6,
				0x09,0xB5,
				0x09,0xB7,
				0x09,0xB0,
				0x09,0xE9,
				0x09,0xEA,
				0x09,0xE2,
				0x09,0xCD,
				0x09,0x30,
				0x09,0xB8,
				0x81,0x02,
				0x95,0x01,
				0x75,0x03,
				0x81,0x03,
				0xC0,

				// Report ID 3: Gamepad
				0x05, 0x01,                 		// Usage Page (Generic Desktop)
				0x09, 0x07,                 		// Usage (Gamepad)
				0xA1, 0x01,                 		// Collection (Application)
				0x85, 0x03,                     //     Report Id (3)
				0xA1, 0x00,                     //     Collection (Physical)
				0x09, 0x30,                 		// 	   Usage (X)
				0x09, 0x31,                 		// 	   Usage (Y)
				0x15, 0xFF,                     //     Logical Minimum (-1)
				0x25, 0x01,                     //     Logical Maximum (1)
				0x95, 0x02,											//	   Report Count (2)
				0x75, 0x02,											//	   Report Size (2)
				0x81, 0x02,											//     Input (Data, Variable, Absolute)
				0xC0,                           //     End Collection	
				
				0x95, 0x04,											//	   Report Count (4)
				0x75, 0x01,											//	   Report Size (1)
				0x81, 0x03,											//     Input (Constant, Variable, Absolute)
				
				0x05, 0x09,                 		// Usage Page (Button)
				0x19, 0x01,                 		//     Usage Minimum (1)
				0x29, 0x06,                 		//     Usage Maximum (6)
				0x15, 0x00,                     //     Logical Minimum (0)
				0x25, 0x01,                     //     Logical Maximum (1)
				0x95, 0x06,											//	   Report Count (6)
				0x75, 0x01,											//	   Report Size (1)
				0x81, 0x02,											//     Input (Data, Variable, Absolute)
				0x95, 0x02,											//	   Report Count (2)
				0x81, 0x03,											//     Input (Constant, Variable, Absolute)
				0xC0,                        		//End Collection


				// Report ID 4: Gamesir
				0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
				0x09, 0x05,                    // USAGE (Game Pad)
				0xa1, 0x01,                    // COLLECTION (Application)
				0x85, 0x04,                    //     Report Id (4)

				0x09, 0x05,                 //     Usage (Vendor Defined)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x26, 0xFF, 0x00,           //     Logical Maximum (255)
        0x75, 0x08,                 //     Report Count (2)
        0x95, 0x02,                 //     Report Size (8 bit)
        0x81, 0x02,                 //     Input (Data, Variable, Absolute)
				
				0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
				0x46, 0xff, 0x00,              //   PHYSICAL_MAXIMUM (255)
				0x09, 0x30,                    //   USAGE (X)
				0x09, 0x31,                    //   USAGE (Y)
				0x09, 0x32,                    //   USAGE (Z)
				0x09, 0x35,                    //   USAGE (Rz)
				0x09, 0x33,                    //   USAGE (Rx)
				0x09, 0x34,                    //   USAGE (Ry)
				0x75, 0x08,                    //   REPORT_SIZE (8)
				0x95, 0x06,                    //   REPORT_COUNT (6)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)

				0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
				0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
				0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
				0x45, 0x01,                    //   PHYSICAL_MAXIMUM (1)
				0x75, 0x01,                    //   REPORT_SIZE (1)
				0x95, 0x10,                    //   REPORT_COUNT (16)
				0x05, 0x09,                    //   USAGE_PAGE (Button)
				0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
				0x29, 0x10,                    //   USAGE_MAXIMUM (Button 16)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)

				0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
				0x15, 0x01,                    //   LOGICAL_MINIMUM (1)
				0x25, 0x08,                    //   LOGICAL_MAXIMUM (8)
				0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
				0x46, 0x3B, 0x10,              //   PHYSICAL_MAXIMUM (4155)
				0x75, 0x04,                    //   REPORT_SIZE (4)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x66, 0x0E, 0x00,              //   UNIT (None)
				0x09, 0x39,                    //   USAGE (Hat switch)
				0x81, 0x42,                    //   INPUT (Data,Var,Abs,Null)
				0x65, 0x00,                    //   UNIT (None)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
				
        0x75, 0x08,                 //     Report Count (1)
        0x95, 0x02,                 //     Report Size (8 bit)
				0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)

				0x85, 0x05,                    //     Report Id (1)
				0x09, 0x04,                    //   Usage (Joystick)
        0x91, 0x02,                    //   Output (Data, Variable, Absolute), Led report padding
				0xC0                        	 //End Collection

    };

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);

	// 	Initialize HID Service - ConsumerControl	
    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_CCONTROL_KEYS_INDEX];
    p_input_report->max_len             = INPUT_CC_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_CC_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm); 	
	
	// 	Initialize HID Service - Gamepad
    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_GAMEPAD_KEYS_INDEX];
    p_input_report->max_len             = INPUT_GAMEPAD_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_GAMEPAD_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
		
	// 	Initialize HID Service - Gamesir
    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_GAMESIR_KEYS_INDEX];//
    p_input_report->max_len             = INPUT_GAMESIR_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_GAMESIR_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
		
		
    p_output_report                      = &output_report_array[OUTPUT_GAMESIR_REPORT_INDEX];//
    p_output_report->max_len             = OUTPUT_GAMESIR_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_GAMESIR_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);
		
    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = false;
    hids_init_obj.inp_rep_count                  = 4;
    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.outp_rep_count                 = 2;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    dis_init();
    bas_init();
    hids_init();
		nus_init();
}



/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */




/** @brief   Function for checking if the Shift key is pressed.
 *
 *  @returns true if the SHIFT_BUTTON is pressed. false otherwise.
static bool is_shift_key_pressed(void)
{
    bool result;
    uint32_t err_code = bsp_button_is_pressed(SHIFT_BUTTON_ID,&result);
    APP_ERROR_CHECK(err_code);
    return result;
}
 */


/**@brief   Function for transmitting a key scan Press & Release Notification.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_instance     Identifies the service for which Key Notifications are requested.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern. 0 < pattern_len < 7.
 * @param[in]  pattern_offset Offset applied to Key Pattern for transmission.
 * @param[out] actual_len     Provides actual length of Key Pattern transmitted, making buffering of
 *                            rest possible if needed.
 * @return     NRF_SUCCESS on success, BLE_ERROR_NO_TX_PACKETS in case transmission could not be
 *             completed due to lack of transmission buffer or other error codes indicating reason
 *             for failure.
 *
 * @note       In case of BLE_ERROR_NO_TX_PACKETS, remaining pattern that could not be transmitted
 *             can be enqueued \ref buffer_enqueue function.
 *             In case a pattern of 'cofFEe' is the p_key_pattern, with pattern_len as 6 and
 *             pattern_offset as 0, the notifications as observed on the peer side would be
 *             1>    'c', 'o', 'f', 'F', 'E', 'e'
 *             2>    -  , 'o', 'f', 'F', 'E', 'e'
 *             3>    -  ,   -, 'f', 'F', 'E', 'e'
 *             4>    -  ,   -,   -, 'F', 'E', 'e'
 *             5>    -  ,   -,   -,   -, 'E', 'e'
 *             6>    -  ,   -,   -,   -,   -, 'e'
 *             7>    -  ,   -,   -,   -,   -,  -
 *             Here, '-' refers to release, 'c' refers to the key character being transmitted.
 *             Therefore 7 notifications will be sent.
 *             In case an offset of 4 was provided, the pattern notifications sent will be from 5-7
 *             will be transmitted.
static uint32_t send_key_scan_press_release(ble_hids_t *   p_hids,
                                            uint8_t *      p_key_pattern,
                                            uint16_t       pattern_len,
                                            uint16_t       pattern_offset,
                                            uint16_t *     p_actual_len)
{
    uint32_t err_code;
    uint16_t offset;
    uint16_t data_len;
    uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];
    
    // HID Report Descriptor enumerates an array of size 6, the pattern hence shall not be any
    // longer than this.
    STATIC_ASSERT((INPUT_REPORT_KEYS_MAX_LEN - 2) == 6);

    ASSERT(pattern_len <= (INPUT_REPORT_KEYS_MAX_LEN - 2));

    offset   = pattern_offset;
    data_len = pattern_len;

    do
    {
        // Reset the data buffer. 
        memset(data, 0, sizeof(data));
        
        // Copy the scan code.
        memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);
        
        if (is_shift_key_pressed())
        {
            data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
        }

        if (!m_in_boot_mode)
        {
            err_code = ble_hids_inp_rep_send(p_hids, 
                                             INPUT_REPORT_KEYS_INDEX,
                                             INPUT_REPORT_KEYS_MAX_LEN,
                                             data);
        }
        else
        {
            err_code = ble_hids_boot_kb_inp_rep_send(p_hids,
                                                     INPUT_REPORT_KEYS_MAX_LEN,
                                                     data);
        }
        
        if (err_code != NRF_SUCCESS)
        {
            break;
        }
        
        offset++;
    } while (offset <= data_len);

    *p_actual_len = offset;

    return err_code;
}
 */


/**@brief   Function for initializing the buffer queue used to key events that could not be
 *          transmitted
 *
 * @warning This handler is an example only. You need to analyze how you wish to buffer or buffer at
 *          all.
 *
 * @note    In case of HID keyboard, a temporary buffering could be employed to handle scenarios
 *          where encryption is not yet enabled or there was a momentary link loss or there were no
 *          Transmit buffers.
static void buffer_init(void)
{
    uint32_t buffer_count;

    BUFFER_LIST_INIT();

    for (buffer_count = 0; buffer_count < MAX_BUFFER_ENTRIES; buffer_count++)
    {
        BUFFER_ELEMENT_INIT(buffer_count);
    }
}
 */


/**@brief Function for enqueuing key scan patterns that could not be transmitted either completely
 *        or partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_hids         Identifies the service for which Key Notifications are buffered.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern.
 * @param[in]  offset         Offset applied to Key Pattern when requesting a transmission on
 *                            dequeue, @ref buffer_dequeue.
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
static uint32_t buffer_enqueue(ble_hids_t *            p_hids,
                               uint8_t *               p_key_pattern,
                               uint16_t                pattern_len,
                               uint16_t                offset)
{
    buffer_entry_t * element;
    uint32_t         err_code = NRF_SUCCESS;

    if (BUFFER_LIST_FULL())
    {
        // Element cannot be buffered.
        err_code = NRF_ERROR_NO_MEM;
    }
    else
    {
        // Make entry of buffer element and copy data.
        element                 = &buffer_list.buffer[(buffer_list.wp)];
        element->p_instance     = p_hids;
        element->p_data         = p_key_pattern;
        element->data_offset    = offset;
        element->data_len       = pattern_len;

        buffer_list.count++;
        buffer_list.wp++;

        if (buffer_list.wp == MAX_BUFFER_ENTRIES)
        {
            buffer_list.wp = 0;
        }
    }

    return err_code;
}
 */


/**@brief   Function to dequeue key scan patterns that could not be transmitted either completely of
 *          partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  tx_flag   Indicative of whether the dequeue should result in transmission or not.
 * @note       A typical example when all keys are dequeued with transmission is when link is
 *             disconnected.
 *
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
static uint32_t buffer_dequeue(bool tx_flag)
{
    buffer_entry_t * p_element;
    uint32_t         err_code = NRF_SUCCESS;
    uint16_t         actual_len;

    if (BUFFER_LIST_EMPTY()) 
    {
        err_code = NRF_ERROR_NOT_FOUND;
    }
    else
    {
        bool remove_element = true;

        p_element = &buffer_list.buffer[(buffer_list.rp)];

        if (tx_flag)
        {
            err_code = send_key_scan_press_release(p_element->p_instance,
                                                   p_element->p_data,
                                                   p_element->data_len,
                                                   p_element->data_offset,
                                                   &actual_len);
            // An additional notification is needed for release of all keys, therefore check
            // is for actual_len <= element->data_len and not actual_len < element->data_len
            if ((err_code == BLE_ERROR_NO_TX_PACKETS) && (actual_len <= p_element->data_len))
            {
                // Transmission could not be completed, do not remove the entry, adjust next data to
                // be transmitted
                p_element->data_offset = actual_len;
                remove_element         = false;
            }
        }

        if (remove_element)
        {
            BUFFER_ELEMENT_INIT(buffer_list.rp);

            buffer_list.rp++;
            buffer_list.count--;

            if (buffer_list.rp == MAX_BUFFER_ENTRIES)
            {
                buffer_list.rp = 0;
            }
        }
    }

    return err_code;
}
 */


/**@brief Function for sending sample key presses to the peer.
 *
 * @param[in]   key_pattern_len   Pattern length.
 * @param[in]   p_key_pattern     Pattern to be sent.
static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)
{
    uint32_t err_code;
    uint16_t actual_len;

    err_code = send_key_scan_press_release(&m_hids,
                                           p_key_pattern,
                                           key_pattern_len,
                                           0,
                                           &actual_len);
    // An additional notification is needed for release of all keys, therefore check
    // is for actual_len <= key_pattern_len and not actual_len < key_pattern_len.
    if ((err_code == BLE_ERROR_NO_TX_PACKETS) && (actual_len <= key_pattern_len))
    {
        // Buffer enqueue routine return value is not intentionally checked.
        // Rationale: Its better to have a a few keys missing than have a system
        // reset. Recommendation is to work out most optimal value for
        // MAX_BUFFER_ENTRIES to minimize chances of buffer queue full condition
        UNUSED_VARIABLE(buffer_enqueue(&m_hids, p_key_pattern, key_pattern_len, actual_len));
    }


    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}
 */


/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(ble_hids_evt_t *p_evt)//judge
{
    if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT)
    {
        uint32_t err_code;
        uint8_t  report_val;
        uint8_t  report_index = p_evt->params.char_write.char_id.rep_index;

        if (report_index == OUTPUT_REPORT_INDEX)
        {
            // This code assumes that the outptu report is one byte long. Hence the following
            // static assert is made.
            STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);

            err_code = ble_hids_outp_rep_get(&m_hids,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             &report_val);
            APP_ERROR_CHECK(err_code);

            if (!m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) != 0))
            {
                // Caps Lock is turned On.
//                err_code = bsp_indication_set(BSP_INDICATE_ALERT_3);
//                APP_ERROR_CHECK(err_code);

//                keys_send(sizeof(m_caps_on_key_scan_str), m_caps_on_key_scan_str);
                m_caps_on = true;
            }
            else if (m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) == 0))
            {
                // Caps Lock is turned Off .
//                err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
//                APP_ERROR_CHECK(err_code);

//                keys_send(sizeof(m_caps_off_key_scan_str), m_caps_off_key_scan_str);
                m_caps_on = false;
            }
            else
            {
                // The report received is not supported by this application. Do nothing.
            }
        }
				
        else if (report_index == OUTPUT_GAMESIR_REPORT_INDEX)
        {
            // This code assumes that the outptu report is one byte long. Hence the following
            // static assert is made.
            STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);

            err_code = ble_hids_outp_rep_get(&m_hids,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             &report_val);
            APP_ERROR_CHECK(err_code);
//						data_array = &report_val;
						NRF_LOG_PRINTF("output: %d.\r\n",report_val);
						ble_nus_string_send(&m_nus, &report_val, OUTPUT_REPORT_MAX_LEN);
					
        }
   }
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

//		uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            on_hid_rep_char_write(p_evt);
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
        {
            dm_service_context_t   service_context;
            service_context.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
            service_context.context_data.len = 0;
            service_context.context_data.p_data = NULL;

            if (m_in_boot_mode)
            {
                // Protocol mode is Boot Protocol mode.
                if (
                    p_evt->params.notification.char_id.uuid
                    ==
                    BLE_UUID_BOOT_KEYBOARD_INPUT_REPORT_CHAR
                )
                {
                    // The notification of boot keyboard input report has been enabled.
                    // Save the system attribute (CCCD) information into the flash.
                    uint32_t err_code;

                    err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                    else
                    {
                        // The system attributes could not be written to the flash because
                        // the connected central is not a new central. The system attributes
                        // will only be written to flash only when disconnected from this central.
                        // Do nothing now.
                    }
                }
                else
                {
                    // Do nothing.
                }
            }
            else if (p_evt->params.notification.char_id.rep_type == BLE_HIDS_REP_TYPE_INPUT)
            {
                // The protocol mode is Report Protocol mode. And the CCCD for the input report
                // is changed. It is now time to store all the CCCD information (system
                // attributes) into the flash.
                uint32_t err_code;

                err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    // The system attributes could not be written to the flash because
                    // the connected central is not a new central. The system attributes
                    // will only be written to flash only when disconnected from this central.
                    // Do nothing now.
                }
            }
            else
            {
                // The notification of the report that was enabled by the central is not interesting
                // to this application. So do nothing.
            }
            break;
        }

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_SLOW:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_FAST_WHITELIST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_SLOW_WHITELIST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
						goto_syetem_off();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_whitelist_t whitelist;
            ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;

            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_whitelist_reply(&whitelist);
            APP_ERROR_CHECK(err_code);
            break;
        }
        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            ble_gap_addr_t peer_address;

            // Only Give peer address if we have a handle to the bonded peer.
            if(m_bonded_peer_handle.appl_id != DM_INVALID_ID)
            {

                err_code = dm_peer_addr_get(&m_bonded_peer_handle, &peer_address);
                if (err_code != (NRF_ERROR_NOT_FOUND | DEVICE_MANAGER_ERR_BASE))
                {
                    APP_ERROR_CHECK(err_code);

                    err_code = ble_advertising_peer_addr_reply(&peer_address);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)//judge
{
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
//            APP_ERROR_CHECK(err_code);
						my_role = 3;
//						CLOSE_M1
//						CLOSE_M2
//						OPEN_KEY_VDD		
				//		CLOSE_KEY_VDD	

            m_conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_EVT_TX_COMPLETE:
            // Send next key event
//            (void) buffer_dequeue(true);
					tx_complete_flag = true;

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            // Dequeue all keys without transmission.
//            (void) buffer_dequeue(false);

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Reset m_caps_on variable. Upon reconnect, the HID host will re-send the Output 
            // report containing the Caps lock state.
            m_caps_on = false;
            // disabling alert 3. signal - used for capslock ON
//            err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
//            APP_ERROR_CHECK(err_code);
						my_role = 0;
//						CLOSE_M1
//						CLOSE_M2
				//		OPEN_KEY_VDD		
//						CLOSE_KEY_VDD	
						if(goto_disconnect_event_flag)
						{
								goto_disconnect_event_flag = false;
								goto_off_flag = true;
						}
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            if(p_ble_evt->evt.gatts_evt.params.authorize_request.type
               != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_PREP_WRITE_REQ)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type
                        == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle,&auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief   Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
//    bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_hids_on_ble_evt(&m_hids, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
}


/**@brief   Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
//    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
	
//    ble_enable_params.gatts_enable_params.service_changed = 1;
//    ble_enable_params.gatts_enable_params.attr_tab_size = 0X580+0XF00;
//    ble_enable_params.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_MIN<<2;
//		ble_enable_params.common_enable_params.vs_uuid_count   = 2;
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
//static void scheduler_init(void)
//{
//    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
//}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    static uint8_t * p_key = m_sample_key_press_scan_str;
    static uint8_t size = 0;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                keys_send(1, p_key);
                p_key++;
                size++;
                if (size == MAX_KEYS_IN_ONE_REPORT)
                {
                    p_key = m_sample_key_press_scan_str;
                    size = 0;
                }
            }
            break;

        default:
            break;
    }
}
 */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t       err_code;
    uint8_t        adv_flags;
    ble_advdata_t  advdata;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    adv_flags                       = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = adv_flags;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options =
    {
        BLE_ADV_WHITELIST_DISABLED,
        BLE_ADV_DIRECTED_DISABLED,
        BLE_ADV_DIRECTED_SLOW_DISABLED, 0,0,
        BLE_ADV_FAST_ENABLED, APP_ADV_FAST_INTERVAL, APP_ADV_FAST_TIMEOUT,
        BLE_ADV_SLOW_ENABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT
    };

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, ble_advertising_error_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           ret_code_t           event_result)
{
    APP_ERROR_CHECK(event_result);

    switch(p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_SECURITY_SETUP_COMPLETE:
            m_bonded_peer_handle = (*p_handle);
            break;
    }

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t         init_param = {0};
//    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t  register_param;

    // Initialize peer device handle.
    err_code = dm_handle_initialize(&m_bonded_peer_handle);
    APP_ERROR_CHECK(err_code);
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);
    
    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}



static uint32_t home_key_press_send(void)
{
		device_packet[0] = 0xA1;
		device_packet[1] = 0x12;
		device_packet[2] = 0x08;
		device_packet[3] = 0x00;
		device_packet[4] = 0x00;
    return ble_hids_inp_rep_send(&m_hids, INPUT_CCONTROL_KEYS_INDEX, INPUT_CC_REPORT_KEYS_MAX_LEN, device_packet);
}

static uint32_t home_key_release_send(void)
{
		device_packet[0] = 0xA1;
		device_packet[1] = 0x12;
		device_packet[2] = 0x00;
		device_packet[3] = 0x00;
		device_packet[4] = 0x00;
    return ble_hids_inp_rep_send(&m_hids, INPUT_CCONTROL_KEYS_INDEX, INPUT_CC_REPORT_KEYS_MAX_LEN, device_packet);
}

static uint32_t pins_value_temp;
static uint8_t send_change_flag = 0;

static bool btn_read_pin_state(uint8_t pin_number)
{
//    return  ((NRF_GPIO->IN >> pin_number) & 1UL);
    return (pins_value_temp & (1 << pin_number)) == 0;
}

void read_mouse_and_send_change(void)
{
	static uint8_t device_packet_temp[INPUT_GAMESIR_REPORT_KEYS_MAX_LEN];//Judge
	static uint8_t i = 0;
	static uint8_t value_temp = 0,send_change_start_home_key = 0;
//	static uint8_t device_packet_release[INPUT_GAMESIR_REPORT_KEYS_MAX_LEN]={0xA1,0xC4,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00};//Judge

	static uint8_t index = 0;
	uint32_t       err_code;
	
		pins_value_temp = nrf_gpio_pins_read();
	
		device_packet[0] = 0xA1;
		device_packet[1] = 0xC4;

//		if(my_role)
		{
				device_packet[3] = value_adc_change[3 + ADC_NUM_INIT];//y1  up down
				device_packet[2] = value_adc_change[2 + ADC_NUM_INIT];//x1  left right
				device_packet[5] = value_adc_change[1 + ADC_NUM_INIT];//y2	up down
				device_packet[4] = value_adc_change[0 + ADC_NUM_INIT];//x2	left right
		}
//		else
//		{
//				device_packet[3] = 0X80;//y1  up down
//				device_packet[2] = 0X80;//x1  left right
//				device_packet[5] = 0X80;//y2	up down
//				device_packet[4] = 0X80;//x2	left right
//		}

	
		device_packet[8] = ((btn_read_pin_state(A_BUTTON)) << 0) | ((btn_read_pin_state(B_BUTTON)) << 1) \
		|(0 << 2)	| ((btn_read_pin_state(X_BUTTON))  << 3) \
		|((btn_read_pin_state(Y_BUTTON))  << 4) | (0 << 5) \
		|((btn_read_pin_state(LB_BUTTON)) << 6) | ((btn_read_pin_state(RB_BUTTON)) << 7) ;
	
		device_packet[9] = ((btn_read_pin_state(LT_BUTTON)) << 0) |((btn_read_pin_state(RT_BUTTON)) << 1) \
		|((btn_read_pin_state(BACK_BUTTON)) << 2) | ((btn_read_pin_state(START_BUTTON)) << 3) \
		|(0 << 4) | ((btn_read_pin_state(B1_BUTTON)) << 5) \
		|((btn_read_pin_state(B2_BUTTON)) << 6) | (0 << 7) ;
	
//		if(btn_read_pin_state(LT_BUTTON))device_packet[6] = 0xff;
//		else	device_packet[6] = 0;
//		
//		if(btn_read_pin_state(RT_BUTTON))device_packet[7] = 0xff;
//		else	device_packet[7] = 0;
		
		device_packet[6] = device_packet[4];// CROSS FIRE GAME
		device_packet[7] = device_packet[5];

		value_temp = ((nrf_gpio_pin_read(LEFT_BUTTON)) << 3) | ((nrf_gpio_pin_read(RIGHT_BUTTON)) << 1) | ((nrf_gpio_pin_read(UP_BUTTON)) << 0) | ((nrf_gpio_pin_read(DOWN_BUTTON)) << 2);
		switch(value_temp)
		{
			case 0x0f:
				device_packet[10] = 0x00;
				break;
			case 0x0e:
				device_packet[10] = 0x01;
				break;
			case 0x0d:
				device_packet[10] = 0x03;
				break;
			case 0x0b:
				device_packet[10] = 0x05;
				break;
			case 0x07:
				device_packet[10] = 0x07;
				break;
			case 0x0c:
				device_packet[10] = 0x02;
				break;
			case 0x09:
				device_packet[10] = 0x04;
				break;
			case 0x03:
				device_packet[10] = 0x06;
				break;
			case 0x06:
				device_packet[10] = 0x08;
				break;
			default:
				device_packet[10] = 0x00;
				break;
				
		}
		
		device_packet[11] = 0x00;


	for (i = 0; i < INPUT_GAMESIR_REPORT_KEYS_MAX_LEN; i++)
	{
		if(device_packet[i] != device_packet_temp[i])
		{
			device_packet_temp[i] = device_packet[i];
			send_change_flag = 1;
//			break;
		}
	}
//			send_change_flag = 1;
	
		if(send_change_start_home_key == 1)
		{
			if(btn_read_pin_state(BETOP_BUTTON)==0)
			{
				send_change_start_home_key = 0;
				home_key_release_send();
				send_change_flag = 0;
			}
			
		}	
		
		if(send_change_start_home_key == 0)
		{
			if(btn_read_pin_state(BETOP_BUTTON))
			{
				send_change_start_home_key = 1;
				home_key_press_send();
				send_change_flag = 0;
			}
		}
		
		if(send_change_flag)
		{
				send_change_flag = 0;
				if(my_role)
				{
	//			err_code = ble_hids_inp_rep_send(&m_hids, INPUT_GAMESIR_KEYS_INDEX, INPUT_GAMESIR_REPORT_KEYS_MAX_LEN, device_packet_temp);
	//			err_code = ble_nus_string_send(&m_nus, &data_array[4], 20);
					err_code = ble_hids_inp_rep_send(&m_hids, INPUT_GAMESIR_KEYS_INDEX, INPUT_GAMESIR_REPORT_KEYS_MAX_LEN, device_packet);
	//			APP_ERROR_CHECK(err_code);
	//			err_code = ble_nus_string_send(&m_nus, &data_array[4], 20);
	//			APP_ERROR_CHECK(err_code);
			
	//			ble_nus_string_send(&m_nus, &data_array[4], 8);
	//			err_code = ble_nus_string_send(&m_nus, data_array, INPUT_GAMESIR_REPORT_KEYS_MAX_LEN);
				}
		}

}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
 */


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure, m_adv_params, to be passed to the stack 
 *          when starting advertising.
 */
/*
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = 
    {
        {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE}, 
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE}, 
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type         = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr  = NULL;                           // Undirected advertisement.
    m_adv_params.fp           = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval     = APP_ADV_INTERVAL;
    m_adv_params.timeout      = APP_ADV_TIMEOUT_IN_SECONDS;
}
*/

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
    
    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
    
    // Start timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
	
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
	
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
//    err_code = sd_ble_gap_adv_start(&m_adv_params);
//    APP_ERROR_CHECK(err_code);

//    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection Parameters module.
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t connection_params_init;
    
    memset(&connection_params_init, 0, sizeof(connection_params_init));

    connection_params_init.p_conn_params                  = NULL;
    connection_params_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    connection_params_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    connection_params_init.disconnect_on_fail             = true;
    connection_params_init.evt_handler                    = NULL;
    connection_params_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&connection_params_init);
    APP_ERROR_CHECK(err_code);
}
 */


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
/*
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static uint16_t                  s_conn_handle = BLE_CONN_HANDLE_INVALID;
    static ble_gap_sec_keyset_t      s_sec_keyset;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            s_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            s_sec_keyset.keys_peer.p_enc_key  = NULL;
            s_sec_keyset.keys_peer.p_id_key   = NULL;
            s_sec_keyset.keys_peer.p_sign_key = NULL;
            err_code = sd_ble_gap_sec_params_reply(s_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS,        
                                                   &m_sec_params,
                                                   &s_sec_keyset);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GAP_EVT_AUTH_STATUS:
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(s_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            if (s_sec_keyset.keys_own.p_enc_key != NULL)
            {
                p_enc_info = &s_sec_keyset.keys_own.p_enc_key->enc_info;

                err_code = sd_ble_gap_sec_info_reply(s_conn_handle, p_enc_info, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device.
                err_code = sd_ble_gap_sec_info_reply(s_conn_handle, NULL, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            { 
                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                APP_ERROR_CHECK(err_code);

                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);
                
                if (err_code == NRF_SUCCESS)
                {
                    // Configure buttons with sense level low as wakeup source.
                    // err_code = bsp_buttons_enable((1 << BLE_BUTTON_ID) | (1 << GZLL_BUTTON_ID));

//                    nrf_gpio_cfg_sense_input(BLE_BUTTON_PIN_NO,
//                                             BUTTON_PULL,
//                                             NRF_GPIO_PIN_SENSE_LOW);
//                
//                    nrf_gpio_cfg_sense_input(GZLL_BUTTON_PIN_NO,
//                                             BUTTON_PULL,
//                                             NRF_GPIO_PIN_SENSE_LOW);

                    // Go to system-off mode.
                    // (this function will not return; wakeup will cause a reset)
                    err_code = sd_power_system_off();
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
            
        default: 
            // No implementation needed.
            break;
    }
}
*/

void ble_stack_start(void)
{
//    uint32_t err_code;
//    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
//    
//    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

//    // Enable BLE stack 
//    ble_enable_params_t ble_enable_params;
//    err_code = softdevice_enable_get_default_config(0, 1, &ble_enable_params);
//    APP_ERROR_CHECK(err_code);

//    err_code = softdevice_enable(&ble_enable_params);
//    APP_ERROR_CHECK(err_code);
//    
//    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
//    APP_ERROR_CHECK(err_code);
		ble_stack_init();
}

static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);
}

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

void ble_stack_stop(void)
{
    uint32_t err_code;
	
		reset_prepare();     
	
    err_code = sd_nvic_DisableIRQ(SWI2_IRQn);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_handler_sd_disable();
    APP_ERROR_CHECK(err_code);
}


void ble_hid_app_start(void)
{
    bool erase_bonds;
	
//    scheduler_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
//    buffer_init();
	
    application_timers_start();
    advertising_start();
}


void ble_hid_app_stop(void)
{
    uint32_t err_code;
    
    // Stop any impending connection parameters update.
//    err_code = ble_conn_params_stop();
//    APP_ERROR_CHECK(err_code);
    
    // Stop application timers.
    err_code = app_timer_stop(m_battery_timer_id);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_stop(m_led_flash_timer_id);
//    APP_ERROR_CHECK(err_code);

}

/** 
 * @}
 */
