/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Aug 29, 2023
 *      Author: geroldwilliams
 *      Details: This code sends command over SPI2 when the user button
 *      connected to Port A Pin 0 is pressed to an Arduino board
 *      running 002SPISlaveCmdHandling.ino
 *      STM32F407xx SPI2 PINS:
 *      MOSI = PB15
 *      MISO = PB14
 *      SCLK = PB13
 *      NSS  = PB12
 *
 */
//extern void initialise_monitor_handles();
#include "stm32f407xx.h"
#include <string.h>

//Command Codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

//Arduino Analog Pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

//Arduino LED
#define LED_PIN					9

void SPI_GPIOInit(){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}
void SPI2_GPIOInit(){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MASTER;
	//Generate 2Mhz clock
	SPI2handle.SPIConfig.SPI_ClockSpeed = SPI_CLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	//Use hardware to manage SSM
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}
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
uint8_t SPI_VerifyResponse(uint8_t ackData){
	//Received ACK
	if(ackData == 0xF5){
		return 1;
	}
	//Received NACK
	return 0;
}
void delay(void){
	//~200ms delay when the system clock is 16 MHz
	for(uint32_t i=0; i<500000/2; i++);
}

int main(void){
	//initialise_monitor_handles();
	uint8_t dummy_tx = 0xff;
	uint8_t dummy_rx;

	//Initialize any GPIO and SPI bus configuration
	GPIO_ButtonInit();
	SPI_GPIOInit();
	SPI2_GPIOInit();

	printf("Application Running! \n" );
	printf("SPI Init Finished. \n" );
	/*
	 * SSOE = 1 does NSS output enable
	 * The NSS pin is automatically managed by the hardware.
	 * i.e. SPE = 1 NSS will be pulled low
	 *      SPE = 0 NSS will be pulled high
	 * */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		//Wait until the button is pressed before sending data
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Delay for button debounce
		delay();

		//Enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//Send the first command
		// CMD_LED_CTRL		<pin no(1)>		<value(1)>
		uint8_t command_code = COMMAND_LED_CTRL;
		uint8_t args[2];
		uint8_t ackData;
		SPI_DataSend(SPI2, &command_code, 1);

		//Read data after sending command to clear garbage from the Data register
		SPI_DataReceive(SPI2,&dummy_rx,1);

		//Send dummy data to receive data from slave(slave will not send data on its own)
		SPI_DataSend(SPI2,&dummy_tx,1);

		//Receive data from slave
		SPI_DataReceive(SPI2,&ackData,1);

		//Check data for ACK or NACK
		if(SPI_VerifyResponse(ackData)){
			//Received ACK, Send other arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_DataSend(SPI2,args,2);
			//Read data after sending command to clear garbage from the Data register
			SPI_DataReceive(SPI2,&dummy_rx,1);
			printf(" COMMAND_LED_CTRL executed \n" );
		}
		//Send Second Command

		//Wait until the button is pressed before sending data
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Delay for button debounce
		delay();

		// CMD_SENSOR_READ		<Analog pin Number(1)>
		command_code = COMMAND_SENSOR_READ;
		SPI_DataSend(SPI2, &command_code, 1);

		//Read data after sending command to clear garbage from the Data register
		SPI_DataReceive(SPI2,&dummy_rx,1);

		//Send dummy data to receive data from slave(slave will not send data on its own)
		SPI_DataSend(SPI2,&dummy_tx,1);

		//Receive data from slave
		SPI_DataReceive(SPI2,&ackData,1);

		//Check data for ACK or NACK
		if(SPI_VerifyResponse(ackData)){
			//Received ACK, Send other arguments
			args[0] = ANALOG_PIN0;
			SPI_DataSend(SPI2,args,1);

			//Read data after sending command to clear garbage from the Data register
			SPI_DataReceive(SPI2,&dummy_rx,1);

			//Delay for ADC conversion
			delay();

			//Send dummy data to receive data from slave(slave will not send data on its own)
			SPI_DataSend(SPI2,&dummy_tx,1);

			//Read the analog data
			uint8_t analogData;
			//Receive data from slave
			SPI_DataReceive(SPI2,&analogData,1);

			printf(" COMMAND_SENSOR_READ executed! Sensor Value: %d \n", analogData );
		}
		//Send third Command

				//Wait until the button is pressed before sending data
				while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

				//Delay for button debounce
				delay();

				// CMD_LED_READ		<pin no>
				command_code = COMMAND_LED_READ;
				SPI_DataSend(SPI2, &command_code, 1);

				//Read data after sending command to clear garbage from the Data register
				SPI_DataReceive(SPI2,&dummy_rx,1);

				//Send dummy data to receive data from slave(slave will not send data on its own)
				SPI_DataSend(SPI2,&dummy_tx,1);

				//Receive data from slave
				SPI_DataReceive(SPI2,&ackData,1);

				//Check data for ACK or NACK
				if(SPI_VerifyResponse(ackData)){
					//Received ACK, Send other arguments
					args[0] = LED_PIN;
					SPI_DataSend(SPI2,args,1);

					uint8_t ledStatus;

					//Read data after sending command to clear garbage from the Data register
					SPI_DataReceive(SPI2,&dummy_rx,1);

					//Send dummy data to receive data from slave(slave will not send data on its own)
					SPI_DataSend(SPI2,&dummy_tx,1);

					//Receive data from slave
					SPI_DataReceive(SPI2,&ledStatus,1);
					printf(" COMMAND_LED_READ executed, LED value = %d\n",ledStatus );
				}

		//Send Forth Command

		//Wait until the button is pressed before sending data
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Delay for button debounce
		delay();

		// CMD_PRINT		<len(1)>		<message(len)>
		command_code = COMMAND_PRINT;
		SPI_DataSend(SPI2, &command_code, 1);
		char message[] = "Testing, testing, testing... 1,2,3...";

		//Read data after sending command to clear garbage from the Data register
		SPI_DataReceive(SPI2,&dummy_rx,1);

		//Send dummy data to receive data from slave(slave will not send data on its own)
		SPI_DataSend(SPI2,&dummy_tx,1);

		//Receive data from slave
		SPI_DataReceive(SPI2,&ackData,1);

		//Check data for ACK or NACK
		if(SPI_VerifyResponse(ackData)){
			//Received ACK, Send other arguments
			args[0] = strlen(message);
			SPI_DataSend(SPI2,args,1);
			SPI_DataSend(SPI2,(uint8_t*)message,strlen(message));

			//Read data after sending command to clear garbage from the Data register
			SPI_DataReceive(SPI2,&dummy_rx,1);
			printf(" COMMAND_PRINT executed \n");
		}
		//Send Fifth Command

		//Wait until the button is pressed before sending data
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Delay for button debounce
		delay();

		// CMD_ID_READ
		command_code = COMMAND_ID_READ;
		SPI_DataSend(SPI2, &command_code, 1);

		//Read data after sending command to clear garbage from the Data register
		SPI_DataReceive(SPI2,&dummy_rx,1);

		//Send dummy data to receive data from slave(slave will not send data on its own)
		SPI_DataSend(SPI2,&dummy_tx,1);

		//Receive data from slave
		SPI_DataReceive(SPI2,&ackData,1);

		//Check data for ACK or NACK
		if(SPI_VerifyResponse(ackData)){
			//Received ACK, Get ID
			char id[11];
			SPI_DataSend(SPI2,args,1);
			SPI_DataSend(SPI2,(uint8_t*)message,strlen(message));

			//Read data after sending command to clear garbage from the Data register
			SPI_DataReceive(SPI2,&dummy_rx,1);
			for(uint8_t i=0;i<10;i++){
				//Send dummy data to receive data from slave(slave will not send data on its own)
				SPI_DataSend(SPI2,&dummy_tx,1);
				//Receive data from slave
				SPI_DataReceive(SPI2,&id[i],1);
			}
			id[10] = '\0';
			printf(" COMMAND_ID_READ executed, ArduinoUno ID = %s \n", id);
		}

		//Wait until we are finished transmitting
		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY_FLAG));

		//Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
		printf("SPI Comms Closed.");
	}


	return 0;
}

