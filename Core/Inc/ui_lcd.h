/*
 * ui_lcd.h
 *
 *  Created on: Dec 16, 2024
 *      Author: Her
 */

#ifndef INC_UI_LCD_H_
#define INC_UI_LCD_H_

#include "i2c_lcd.h"
#include "string.h"
#include "utils.h"

#define MAIN_MENU 99
#define SET_SI5351 0
#define SET_LED_BRIGHTNESS 1
#define GET_ADC 2
#define RUN_SI5351 10

uint8_t lcd_fsm_state = MAIN_MENU;
// Menggunakan array yang lebih besar untuk menghindari buffer overflow
uint8_t value_1[4] = { 0 }; // 3 digit + null terminator
uint8_t value_2[4] = { 0 }; // 3 digit + null terminator
uint8_t value_3[4] = { 0 }; // 3 digit + null terminator

void Main_Menu(uint8_t I2C_LCD_InstanceIndex, int8_t* option)
{
    static const uint8_t text_up[16] = { "   MAIN MENU!   " };
    static uint8_t text_down_left[8] = { " " };
    static uint8_t text_down_right[8] = { " " };

    if (*option == 0 || *option == 1) {
        sprintf((char*)text_down_left, " SI5351 ");
        sprintf((char*)text_down_right, " LED PWM");
        if (*option == 0) {
            text_down_left[0] = '>';
            text_down_right[0] = ' ';
        } else {
            text_down_right[0] = '>';
            text_down_left[0] = ' ';
        }
    } else {
        sprintf((char*)text_down_left, " ADC IN ");
        sprintf((char*)text_down_right, "        ");
        if (*option == 2 || *option == 3) {
            text_down_left[0] = '>';
            *option = 2;
        }
    }

    I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 0);
    I2C_LCD_WriteString(I2C_LCD_InstanceIndex, text_up);

    I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 1);
    I2C_LCD_WriteString(I2C_LCD_InstanceIndex, text_down_left);
    I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 8, 1);
    I2C_LCD_WriteString(I2C_LCD_InstanceIndex, text_down_right);
}

void Set_Si5351(uint8_t I2C_LCD_InstanceIndex, int8_t counter, int freq_target, uint8_t flag)
{
    static const uint8_t text[16] = { "   SET SI5351   " };
    static const uint8_t ok[3] = { "-YN" };
    static const uint8_t hz[2] = { "Hz" };

    // Menggunakan snprintf untuk format angka dengan leading zeros
    snprintf((char*)value_1, 4, "%03lu", freq_target / 1000000);
    snprintf((char*)value_2, 4, "%03lu", (freq_target % 1000000) / 1000);
    snprintf((char*)value_3, 4, "%03lu", freq_target % 1000);

    I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 0);
    I2C_LCD_WriteString(I2C_LCD_InstanceIndex, text);

    if (lcd_fsm_state != SET_SI5351) {
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_1);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 3, 1);
        I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 4, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_2);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 7, 1);
        I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 8, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_3);

        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 11, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, hz);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 13, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, ok);
        lcd_fsm_state = SET_SI5351;
    }

    if (flag) {
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_1);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 3, 1);
        I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 4, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_2);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 7, 1);
        I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 8, 1);

        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_3);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, counter, 1);
        I2C_LCD_Blink(I2C_LCD_InstanceIndex);
    } else {
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, counter, 1);
        I2C_LCD_Blink(I2C_LCD_InstanceIndex);
    }
    return;
}

void Run_Si5351(uint8_t I2C_LCD_InstanceIndex)
{
    static const uint8_t text_run[3] = { "F:" };
    static const uint8_t hz[3] = { "Hz" };
    static const uint8_t stop[16] = { "      STOP      " };

    if (lcd_fsm_state != RUN_SI5351) {
        I2C_LCD_Clear(I2C_LCD_InstanceIndex);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 0);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, text_run);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 2, 0);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_1);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 5, 0);
        I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 6, 0);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_2);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 9, 0);
        I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 10, 0);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value_3);
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 13, 0);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, hz);
        lcd_fsm_state = RUN_SI5351;

        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 1);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, stop);
    }

    return;
}

void Set_LED_Brightness(uint8_t I2C_LCD_InstanceIndex, int8_t pwm_counter)
{
    static const uint8_t text[16] = { "  SET LED PWM!  " };

    if (lcd_fsm_state != SET_LED_BRIGHTNESS) {
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 0);
        I2C_LCD_WriteString(I2C_LCD_InstanceIndex, text);
        lcd_fsm_state = SET_LED_BRIGHTNESS;
        I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 1);
    }

    I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 1);

    if (pwm_counter == 0) {
        // Display dots for 0%
        for (uint8_t i = 0; i < 16; i++) {
            I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
        }
    } else if (pwm_counter == 16) {
        // Display full hash line for 100%
        for (uint8_t i = 0; i < 16; i++) {
            I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '#');
        }
    } else {
        // Partial progress bar
        for (uint8_t i = 0; i < 16; i++) {
            if (i < pwm_counter) {
                I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '#');
            } else {
                I2C_LCD_WriteChar(I2C_LCD_InstanceIndex, '.');
            }
        }
    }

    return;
}

void Get_ADC(uint8_t I2C_LCD_InstanceIndex, uint16_t data)
{
    static const uint8_t text[16] = { "    GET ADC IN" };
    static uint8_t value[5] = { "0000" };
    // if (lcd_fsm_state != GET_ADC) {
    //     I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 0);
    //     I2C_LCD_WriteString(I2C_LCD_InstanceIndex, text);
    //     lcd_fsm_state = GET_ADC;
    // }
    I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 0, 1);
    I2C_LCD_WriteString(I2C_LCD_InstanceIndex, "ADC: ");
    I2C_LCD_SetCursor(I2C_LCD_InstanceIndex, 5, 1);
    sprintf((char*)value, "%04d", data);
    I2C_LCD_WriteString(I2C_LCD_InstanceIndex, value);
    return;
}

void LCD_Routine(uint8_t I2C_LCD_InstanceIndex, int8_t counter, int8_t* option, int freq_target, uint8_t flag, int8_t pwm_value, uint16_t data)
{
    switch (fsm_state) {
    case MAIN_MENU:
        Main_Menu(I2C_LCD_InstanceIndex, option);
        break;

    case SET_SI5351:
        Set_Si5351(I2C_LCD_InstanceIndex, counter, freq_target, flag);
        break;

    case RUN_SI5351:
        Run_Si5351(I2C_LCD_InstanceIndex);
        break;

    case SET_LED_BRIGHTNESS:
        Set_LED_Brightness(I2C_LCD_InstanceIndex, pwm_value);
        break;

    case GET_ADC:
        Get_ADC(I2C_LCD_InstanceIndex, data);
        break;

    default:
        break;
    }
}

#endif /* INC_UI_LCD_H_ */
