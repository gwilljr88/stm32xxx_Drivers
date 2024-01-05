/*
 * lcd.h
 *
 *  Created on: Oct 30, 2023
 *      Author: geroldwilliams
 * 
 */
#ifndef LCD_H_
#define LCD_H_
#include "stm32f407xx.h"
/**
 * bsp exposed APIs
 */
void LCD_Init(void);
void LCD_CommandSend(uint8_t cmd);
void LCD_PrintChar(uint8_t data);
void LCD_PrintString(char* message);
void LCD_DisplayClear(void);
void LCD_DisplayReturnHome(void);
void LCD_SetCursor(uint8_t row, uint8_t col);

/**
 * Application Configurable Items
 */
#define LCD_GPIO_PORT			GPIOD
#define LCD_GPIO_RS				GPIO_PIN_0 
#define LCD_GPIO_RW				GPIO_PIN_1 
#define LCD_GPIO_EN				GPIO_PIN_2 
#define LCD_GPIO_D4				GPIO_PIN_3 
#define LCD_GPIO_D5				GPIO_PIN_4 
#define LCD_GPIO_D6				GPIO_PIN_5 
#define LCD_GPIO_D7				GPIO_PIN_6 
/**
 * LCD Commands
 */
#define LCD_CMD_4DL_2N_5X8F		0x28  /*Use 4 Data lines, 2 lines on display, 5x8 dots*/
#define LCD_CMD_DON_CURON		0x0E  /*Display ON, Cursor ON*/
#define LCD_CMD_INCADD			0x06  /*Increment DDRAM address, move cursor to right*/ 
#define LCD_CMD_DIS_CLEAR		0x01  /*Clear Display*/
#define LCD_CMD_RETURN_HOME		0x02  /*Return cursor to home position*/
#endif /* LCD_H_ */