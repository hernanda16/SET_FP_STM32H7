/*
 * File: I2C_LCD_cfg.h
 * Driver Name: [[ I2C_LCD Display ]]
 * SW Layer:   ECUAL
 * Created on: Jan 28, 2024
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#ifndef I2C_LCD_CFG_H_
#define I2C_LCD_CFG_H_

#include "main.h"

#define I2C_LCD_MAX	1	// Maximum Number of I2C_LCD Modules in Your Project
#define I2C_LCD_1	0	// I2C_LCD Instance Number 1 (Add more if you need)

/*-----------------------[INTERNAL DEFINITIONS]-----------------------*/
// CMD
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80
// DISPLAY ENTRY
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
// DISPLAY CONTROL
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00
// CURSOR MOTION
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00
// FUNCTION SET
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00
// BACKLIGHT CONTROL
#define LCD_BACKLIGHT           0x08
#define LCD_NOBACKLIGHT         0x00
#define EN                      0b00000100  // Enable bit
#define RW                      0b00000010  // Read/Write bit
#define RS                      0b00000001  // Register select bit

/*-----------------------[INTERNAL VARIABLES]-----------------------*/

typedef struct
{
	// I2C LCD Module Instance Index
	uint8_t I2C_LCD_Instance;

	// I2C Hardware Peripheral Handle
	I2C_HandleTypeDef* I2C_Handle;

	// I2C LCD Hardware Device Address
	uint8_t I2C_LCD_Address;

	// I2C LCD Columns Count
	uint8_t I2C_LCD_nCol;

	// I2C LCD Rows Count
	uint8_t I2C_LCD_nRow;

}I2C_LCD_CfgType;

I2C_HandleTypeDef hi2c1;

const I2C_LCD_CfgType I2C_LCD_CfgParam[I2C_LCD_MAX] =
{
	{   /*  Configuration Parameter For I2C_LCD Instance #1   */
		I2C_LCD_1,	/* Index of I2C_LCD Instance #1           */
		&hi2c1,		/* Hardware I2C Module's Handle           */
		0x27,		/* Hardware I2C_LCD Device Address        */
		16,			/* LCD Columns Count                      */
		2			/* LCD Rows Count                         */
	}
};


#endif /* I2C_LCD_CFG_H_ */
