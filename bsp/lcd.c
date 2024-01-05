/*
 * lcd.c
 *
 *  Created on: Oct 30, 2023
 *      Author: geroldwilliams
 * 
 */
#include "lcd.h"
static void lcd_enable(void);
static void write_4_bits(uint8_t data);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);
/**
 * @brief      Initialize the LCD and configure the GPIO for use
 */
void LCD_Init(void){

	//Configure GPIO pins used for LCD connection
	GPIO_Handle_t lcd_signal;
	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber      = LCD_GPIO_RS;
	lcd_signal.GPIO_PinConfig.GPIO_PinOType       = GPIO_OUTPUT_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;

	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber      = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber      = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber      = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber      = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber      = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber      = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	//Keep I/O pins at 0 initially
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//LCD initialization
	//Wait for more than 40 ms after Vcc rises to 2.7V
	mdelay(40);

	//RS = 0 for LCD command
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	//RW = 0 for LCD write
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//Write 0010 to D7-D3 Function set
	write_4_bits(0x03);
	// write_4_bits(0x02);

	mdelay(5);

	write_4_bits(0x03);
	// write_4_bits(0x02);

	udelay(100);

	write_4_bits(0x03);
	// write_4_bits(0x0C);
	write_4_bits(0x02);
	// write_4_bits(0x00);
	//Function set command
	LCD_CommandSend(LCD_CMD_4DL_2N_5X8F);

	//Display ON, Cursor ON
	// write_4_bits(0x00);
	LCD_CommandSend(LCD_CMD_DON_CURON);

	//Display clear
	// write_4_bits(0x00);
	LCD_DisplayClear();

	//Entry Mode Set
	// write_4_bits(0x00);
	LCD_CommandSend(LCD_CMD_INCADD);
}
/**
 * @brief      Helper function to latch data on data line
 */
static void lcd_enable(void){
	//Make LCD enable high to low
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	//Wait for instruction execution time
	udelay(40);

}
/**
 * @brief      Writes 4 bits(Nibble) of data to the LCD.
 *
 * @param[in]  data  4 bits of data to write
 */
static void write_4_bits(uint8_t data){
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((data >> 0) & 0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((data >> 1) & 0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((data >> 2) & 0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((data >> 3) & 0x01));

	//Instruct the LCD to latch the data
	lcd_enable();
}
/**
 * @brief      Clears the LCD
 */
void LCD_DisplayClear(void){
	LCD_CommandSend(LCD_CMD_DIS_CLEAR);
	//Wait 2ms for command execution time
	mdelay(2);
}
/**
 * @brief      Returns the LCD cursor to its home position.
 */
void LCD_DisplayReturnHome(void){
	LCD_CommandSend(LCD_CMD_RETURN_HOME);
	//Wait 2ms for command execution time
	mdelay(2);
}
/**
 * @brief      Sends a command to the LCD
 *
 * @param[in]  cmd   LCD command,  See: LCD Commands in header file for possible options
 */
void LCD_CommandSend(uint8_t cmd){
	//Create the command code

	//Make RS pin low
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	//Make RW pin low
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//Send higher nibble(4-bits) of the command code first the lower nibble
	write_4_bits(cmd>>4);
	write_4_bits(cmd & 0x0F);
	//Make LCD enable pin high to low, to latch data on data lines
	//These steps are taken care of by lcd_enable in write_4_bits
}
/**
 * @brief      Prints a single character on the LCD screen
 *
 * @param[in]  data  The character to be displayed on the LCD
 */
void LCD_PrintChar(uint8_t data){
	//Make RS high
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
	//Make RW low
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//Send higher nibble(4-bits) of the command code first the lower nibble
	write_4_bits(data >> 4);
	write_4_bits(data & 0x0F);
	//Make LCD enable pin high to low, to latch data on data lines
	//These steps are taken care of by lcd_enable in write_4_bits

}
/**
 * @brief      Prints a string on the LCD screen
 *
 * @param      message  The message as a string to be printed
 */
void LCD_PrintString(char* message){
	//Loop and send a character to the LCD until we hit the end of string
	do{
		LCD_PrintChar((uint8_t)*message++);
	}while(*message != '\0');
}

static void mdelay(uint32_t cnt){
	for(uint32_t i=0; i<(cnt *1000);i++);
}
static void udelay(uint32_t cnt){
	for(uint32_t i=0; i<(cnt *1);i++);
}
/**
 * @brief      Sets LCD cursor to location specified by row and col
 *
 * @param[in]  row   The row ranges from 1 - 2
 * @param[in]  col   The column ranges from 1 - 16
 */
void LCD_SetCursor(uint8_t row, uint8_t col){
	col--;
	switch(row){
		case 1:
			LCD_CommandSend(col |= 0x80);
			break;
		case 2:
			LCD_CommandSend(col |= 0xC0);
			break;
		default:
			break;
	}
}
