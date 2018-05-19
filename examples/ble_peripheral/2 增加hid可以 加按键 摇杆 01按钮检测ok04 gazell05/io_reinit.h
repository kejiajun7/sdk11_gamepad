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
 * $LastChangedRevision: 15516 $
 */

#ifndef __IO_REINIT_H
#define __IO_REINIT_H

/**
 * @file
 * @brief Mouse Emulator API
 */


/**
 * @defgroup gzp_mouse_emulator Mouse Emulator
 * @{
 * @ingroup gzp_desktop_device_emulator_example
 * @brief  Emulates a mouse using one of the nRF51's timer peripherals.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/*****************************************************************************/
/** @name Configuration                                                      */
/*****************************************************************************/
#define SEND_KEYBOARD_DATA_BUTTON_ID   0  ///< GPIO pin for reading from a button to emulate a keypress.
#define SEND_MOUSE_DATA_BUTTON_ID      1  ///< GPIO pin for reading from a button to emulate a mouse movement.
#define SEND_KEYBOARD_DATA_BUTTON_ID2  2  ///< GPIO pin for reading from a button to emulate a mouse movement.
#define SEND_KEYBOARD_DATA_BUTTON_ID3  3  ///< GPIO pin for reading from a button to emulate a mouse movement.
//////////////////////////////
// output
//////////////////////////////
#define LED1_OUTPUT 7
#define LED2_OUTPUT 8

#define KEY_VDD 30

#define M1_PWM 29
#define M2_PWM 9

//////////////////////////////
// input
//////////////////////////////

#define IO_PAIRING_BTN_PIN BUTTON_1

#define LEFT_BUTTON 17
#define RIGHT_BUTTON 15
#define UP_BUTTON 18
#define DOWN_BUTTON 16

#define BETOP_BUTTON 21

#define JOYSTICK_X2_ADC 3	//up down 2
#define JOYSTICK_Y2_ADC 4	//left right 2
#define JOYSTICK_X1_ADC 5	//up down 1
#define JOYSTICK_Y1_ADC 6	//left right 1

#define B1_BUTTON 10
#define B2_BUTTON 11

#define RB_BUTTON 23
#define RT_BUTTON 22
#define LB_BUTTON 19
#define LT_BUTTON 20
#define X_BUTTON 25
#define Y_BUTTON 24
#define A_BUTTON 28
#define B_BUTTON 27
#define BACK_BUTTON 14
#define TURBO_BUTTON 12
#define CLEAR_BUTTON 13
#define START_BUTTON 26

#define ADC_REF0 0
#define ADC2_BAT 1
#define ADC3_USB 2


#define INPUT_BUTTON {START_BUTTON,CLEAR_BUTTON,TURBO_BUTTON,BACK_BUTTON,\
B_BUTTON,A_BUTTON,Y_BUTTON,X_BUTTON,\
LT_BUTTON,LB_BUTTON,RT_BUTTON,RB_BUTTON,\
B2_BUTTON,B1_BUTTON,BETOP_BUTTON,\
LEFT_BUTTON,RIGHT_BUTTON,UP_BUTTON,DOWN_BUTTON,\
}

#define INPUTS_NUMBER    19

#define INPUTS_ADC {JOYSTICK_X2_ADC,JOYSTICK_Y2_ADC,JOYSTICK_X1_ADC,JOYSTICK_Y1_ADC,ADC2_BAT,ADC3_USB}    
#define INPUTS_ADC_NUMBER    6

#define OUPUTS_PWM {M1_PWM,M2_PWM,LED1_OUTPUT,LED2_OUTPUT,KEY_VDD}    
#define OUPUTS_PWM_NUMBER    5

#define OPEN_M1		nrf_gpio_pin_set(M1_PWM);
#define OPEN_M2		nrf_gpio_pin_set(M2_PWM);
#define CLOSE_M1		nrf_gpio_pin_clear(M1_PWM);
#define CLOSE_M2		nrf_gpio_pin_clear(M2_PWM);
#define OPEN_KEY_VDD		nrf_gpio_pin_clear(KEY_VDD);
#define CLOSE_KEY_VDD		nrf_gpio_pin_set(KEY_VDD);
#define OPEN_LED1		nrf_gpio_pin_clear(LED1_OUTPUT);
#define CLOSE_LED1		nrf_gpio_pin_set(LED1_OUTPUT);
#define OPEN_LED2		nrf_gpio_pin_clear(LED2_OUTPUT);
#define CLOSE_LED2		nrf_gpio_pin_set(LED2_OUTPUT);


// lint -restore

/** @} */

#ifdef __cplusplus
}
#endif

#endif
