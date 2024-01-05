/*
 * 017uart_caseIT.c
 *
 *  Created on: Oct 24, 2023
 *      Author: geroldwilliams
 *
 * This program will send a message over UART, using interrupt based
 * send and receive API, from STM32 to Arduino Uno
 * For every message the STM32 sends, the Arduino will change the case
 * of the alphabets, lower to upper and upper to lower. Then sends the
 * message back to the STM32. The STM32 will capture the reply and 
 * display it inside of a console.
 * 
 * 
 *
 * Note:
 * Using UART6
 * Baud Rate: 115200 bps
 * Frame Format: 1 stop bit, 8 bits, no parity
 *
 * ALT FUNCTION: 8
 *
 * STM32 <---- USART ----> ArduinoUno
 * PC6    ----  Tx   ----> 0
 * PC7   <----  Rx   ----  1
 *    ----  RTS  ----
 *    ----  CTS  ----
 *
 * Arduino Uno use sketch: 002UARTTxString.ino
 * (PIN 0 must be disconnected while downloading code to Arduino Uno)
 * 
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define msgLen 4
char *msgBuffer[msgLen] = {"Test message using USART from STM32", "PLEASE DONT YELL whisper","Up iS DoWn, dOwN Is Up","sAiPpUaKiVaUpPiAs"}; 
USART_Handle_t USART6_handle; //Let's use UART6 for this test 
uint8_t rxBuffer[1024]; //Buffer for received data
uint8_t cnt = 0; //keep track of the current message we a sending
uint8_t rxCmplt = RESET; //flag to prevent main loop hold main until we are finished receiving the last byte

extern void initialise_monitor_handles(void);
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
	USART6_handle.USART_Config.USART_Mode = USART_MODE_TXRX; // Configure for both Tx and Rx
	USART6_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART6_handle.USART_Config.USART_NumOfStopBits = USART_STOPBITS_1;
	USART6_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART6_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART6_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USART6_handle);
}

int main(void){
	// Initialize Semi-hosting, output info to console using printf
	initialise_monitor_handles();

	// Initialize GPIO for User Button
	GPIO_ButtonInit();

	// Initialize GPIO for USART6	
	USART6_GPIO_init();

	// Initialize USART6 with application setting
	USART6_init();

	// Enable Interrupts here
	USART_IRQInterruptConfig(IRQ_NO_USART6,ENABLE);

	// Turn on USART6
	USART_PeripheralControl(USART6_handle.pUSARTx, ENABLE);

	while(1){
		// Wait for user to press button
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0));

		// Delay for button de-bounce ~200ms
		delay();

		// Receive the message with the case reversed
		while(USART_ReceiveDataIT(&USART6_handle, rxBuffer, strlen(msgBuffer[cnt])) != USART_READY);

		// Send message to Arduino Uno
		USART_SendData(&USART6_handle, (uint8_t*)msgBuffer[cnt], strlen(msgBuffer[cnt]));

		// Wait until we are finished receiving the message from the Arduino
		while(rxCmplt != SET);

		memset(rxBuffer,'\0',sizeof(rxBuffer)); // empty rxBuffer for next message
		rxCmplt = RESET;
		// Increment count to the next message in the buffer
		cnt++;
		// Make sure we don't pass the last message
		cnt = cnt % msgLen;
	}
	return 0;
}

// Here we define the IRQ Handler for USART6, the declaration for this function is found in the startup file
// This function gets call when ever an interrupt is generated
void USART6_IRQHandler(void){
	// Call the API IRQ handler, pass it USART6 handle to take the appropriate action
	USART_IRQHandling(&USART6_handle);
}

// Here we handle the events/error generated by the API see: USART_APPLICATION_CALLBACK_STATUS
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv){
	if(AppEv ==  USART_EVENT_TX_CMPLT){

	}
	else if(AppEv == USART_EVENT_RX_CMPLT){
		rxCmplt = SET;
		printf("Tx'd: %s\n",msgBuffer[cnt]);
		rxBuffer[strlen(msgBuffer[cnt]) + 1] = '\0';
		printf("Rx'd: %s\n", rxBuffer);
	}
	else if(AppEv == USART_EVENT_CTS){
		printf("Event/Error - USART_EVENT_CTS \n");
	}
	else if(AppEv == USART_EVENT_IDLE){
		printf("Event/Error - USART_EVENT_IDLE \n");
	}
	else if(AppEv == USART_EVENT_ORE){
		printf("Event/Error - USART_EVENT_ORE \n");
	}
	else if(AppEv == USART_ERREVENT_FE){
		printf("Event/Error - USART_ERREVENT_FE \n");
	}
	else if(AppEv == USART_ERREVENT_NE){
		printf("Event/Error - USART_ERREVENT_NE \n");
	}
	else if(AppEv == USART_ERREVENT_ORE){
		printf("Event/Error - USART_ERREVENT_ORE \n");
	}

}

