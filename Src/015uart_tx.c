/*
 * 015uart_tx.c
 *
 *  Created on: Oct 24, 2023
 *      Author: geroldwilliams
 *
 * This program will wait on user to press a button to send a message over UART, from STM32
 * to ArduinoUno. The Arduino will display the message, in the Serial Monitor
 * of ArduinoIDE, sent from the STM32F4DISCOVERY board.
 * 
 * 
 * 
 * 
 *
 * Note:
 * Using UART6
 * Baud Rate: 115200 bps
 * Frame Format: 1 stop bit, 8 bits, no parity
 * ALT FUNCTION: 8
 *
 * STM32 <---- USART ----> ArduinoUno
 * PC6    ----  Tx   ----> 0
 * PC7   <----  Rx   ----  1
 *    ----  RTS  ----
 *    ----  CTS  ----
 *
 * Arduino Uno use sketch: 001UARTRxString.ino
 * (PIN 0 must be disconnected while downloading code to Arduino Uno)
 * 
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_gpio_driver.h"

char msgBuffer[] = "Test message using USART from STM32.\n\r"; //Message must end with \n\r
USART_Handle_t USART6_handle; //Let's use UART6 for this test 

void delay(void)
{
	//~200ms delay when the system clock is 16 MHz
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

//Function to configure GPIO for user button
void GPIO_ButtonInit(){
	GPIO_Handle_t GpioBtn;
	//Configure Button GPIO
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioBtn);

}
//Function to configure GPIO as USART
void USART6_GPIO_init(void){
	GPIO_Handle_t GPIOhandle;

	// Configure Port C for USART6 Tx(PC6) and Rx(PC7)
	GPIOhandle.pGPIOx = GPIOC;
	GPIOhandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOhandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOhandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIOhandle.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_PP;
	GPIOhandle.GPIO_PinConfig.GPIO_PinAltFunMode = 8;

	GPIOhandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&GPIOhandle);

	GPIOhandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&GPIOhandle);
}

//Function to configure USART with application settings
void USART6_init(void){
	// Configure USART6
	USART6_handle.pUSARTx = USART6;
	// Configure Application settings:
	// Baud Rate: 115200 bps
	// Frame Format: 1 stop bit, 8 bits, no parity
	USART6_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART6_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART6_handle.USART_Config.USART_NumOfStopBits = USART_STOPBITS_1;
	USART6_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART6_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART6_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USART6_handle);
}

int main(void){
	// Initialize GPIO for User Button
	GPIO_ButtonInit();

	// Initialize GPIO for USART6	
	USART6_GPIO_init();

	// Initialize USART6 with application setting
	USART6_init();

	// Turn on USART6
	USART_PeripheralControl(USART6_handle.pUSARTx, ENABLE);

	while(1){
		// Wait for user to press button
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0));

		// Delay for button de-bounce ~200ms
		delay();

		// Send message to Arduino Uno
		USART_SendData(&USART6_handle, (uint8_t*)msgBuffer,strlen(msgBuffer));
	}
	return 0;
}
