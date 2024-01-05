/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Aug 28, 2023
 *      Author: geroldwilliams
 */

#include "stm32f407xx.h"
#include <string.h>

void SPI_GPIOInit(){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
}
void SPI2_Init(){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MASTER;
	//Generate 2Mhz clock
	SPI2handle.SPIConfig.SPI_ClockSpeed = SPI_CLK_SPEED_DIV32;
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

void delay(void){
	//~200ms delay when the system clock is 16 MHz
	for(uint32_t i=0; i<500000/2; i++);
}

int main(void){

	char user_data[] = "Hello World!";
	GPIO_ButtonInit();
	SPI_GPIOInit();
	SPI2_Init();
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

		//Arduino expect to receive the length
		uint8_t dataLen = strlen(user_data);
		SPI_DataSend(SPI2, &dataLen, 1);

		//Send user_data
		SPI_DataSend(SPI2, (uint8_t*)user_data, strlen(user_data));

		//Wait until we are finished transmitting
		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY_FLAG));

		//Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}

