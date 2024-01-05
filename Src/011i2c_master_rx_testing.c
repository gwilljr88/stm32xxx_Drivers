/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: Sep 26, 2023
 *      Author: geroldwilliams
 *
 * I2C Master(STM32 Discovery) and I2C Slave(Arduino Uno) communication
 * When user button is pressed on the STM32 board, master should read data from the
 * slave(Arduino Uno). The data received from the Arduino board will be displayed
 * on the console.
 *
 * Note:
 *
 * This code Uses I2C1
 * STM32 ----> I2C -----> ArduinoUno
 * PB7   ----> SDA -----> A4
 * PB6   ----> SCL -----> A5
 * ALT FN MODE = 4
 *
 * Arduino Uno use sketch: 002I2CSlaveTxString.ino
 * the sketch uses the Arduino wire library and is limited to 32 bytes
 * send/receive in one transaction
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles(void);

#define MY_ADDR		0x61
#define SLAVE_ADDR	0x68 //Found in Arduino sketch
uint8_t CMD_LEN  =	0x51;
uint8_t CMD_READ =	0x52;

uint8_t RxBuffer[256];
 
I2C_Handle_t I2C1handle;

void delay(void)
{
	//~200ms delay when the system clock is 16 MHz
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

//Function to setup I2C1 peripheral for use
void I2C1_Init(){

	I2C1handle.pI2Cx = I2C1;
	I2C1handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; // be sure to check i2c spec for reserved addresses
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1handle);
}
//Function to setup GPIO pins as I2C
void I2C_GPIOInit(){
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);
}
//Function to setup GPIO for user button
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
int main(void){
  	initialise_monitor_handles();

	printf("Application started.\n");
	//Setup Button GPIO
	GPIO_ButtonInit();

	//Setup GPIO pins for I2C
	I2C_GPIOInit();

	//I2C peripheral setup
	I2C1_Init();
	printf("Finished I2C and GPIO init.\n");

	//Enable the I2C1 peripheral
	I2C_PeripheralControl(I2C1handle.pI2Cx, ENABLE);

	//Enable ACKing
	I2C_AckContol(I2C1handle.pI2Cx, I2C_ACK_ENABLE);

	printf("I2C enabled.\n");

	uint8_t len = 0;

	while(1){
		printf("Waiting for button press.\n");
		//Wait until the button is pressed before sending data
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Delay for button debounce
		delay();

		//Note: printf between transmissions will slow communication ~60 ms per printf
		//Send command to request length to slave(1 byte)
		I2C_MasterSendData(&I2C1handle, &CMD_LEN,1, SLAVE_ADDR, I2C_EN_SR);
		//printf("Requesting length.\n");

		//Receive data length(1 byte)
		I2C_MasterReceiveData(&I2C1handle, &len,1, SLAVE_ADDR, I2C_EN_SR);
		//printf("length is %u.\n",len);

		//Send read command to slave(1 byte)
		I2C_MasterSendData(&I2C1handle, &CMD_READ,1, SLAVE_ADDR, I2C_EN_SR);
		//printf("Requesting data.\n");

		//Receive "len" bytes of data from slave
		I2C_MasterReceiveData(&I2C1handle,(uint8_t*)&RxBuffer,len, SLAVE_ADDR, I2C_DI_SR);
		printf("Data rcvd: %s", (char*)RxBuffer);

	}
	return 0;
}
