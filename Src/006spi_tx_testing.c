/*
 * 006spi_tx_testing.c
 *
 *  Created on: Aug 26, 2023
 *      Author: geroldwilliams
 */
/*  SPI 2
 *  PB12 - NSS
 *  PB13 - SCLK
 *  PB14 - MISO
 *  PB15 - MOSI
 *  ALT function mode : 5
 *  */

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
	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}
void SPI2_GPIOInit(){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MASTER;
	SPI2handle.SPIConfig.SPI_ClockSpeed = SPI_CLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}
int main(void){

	char user_data[] = "Hello World!";
	SPI_GPIOInit();
	SPI2_GPIOInit();

	//TO AVOID MODEF ERROR, Make the NSS bit high internally
	SPI_SSIConfig(SPI2, ENABLE);
	//Enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_DataSend(SPI2, (uint8_t*)user_data, strlen(user_data));
	//Wait until we are finished transmitting
	while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY_FLAG));
	//Disable SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
	return 0;
}
