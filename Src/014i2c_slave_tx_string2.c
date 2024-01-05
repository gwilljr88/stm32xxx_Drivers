/*
 * 014i2c_slave_tx_string2.c
 *
 *  Created on: Oct 18, 2023
 *      Author: geroldwilliams
 *
 * I2C Slave(STM32 Discovery) and I2C master(Arduino Uno) communication
 * This code will send data to the Arduino when requested using interrupts and
 * has been modified to send more than 32 bytes of data to an Arduino.
 * Using the serial monitor inside of the Arduino IDE,
 * Start by type s inside the console. This will trigger the STM32 to send
 * a message using I2Cx Peripheral. 
 * 
 *
 * Note:
 *
 * This code Uses I2C1
 * STM32 ----> I2C -----> ArduinoUno
 * PB7   ----> SDA -----> A4
 * PB6   ----> SCL -----> A5
 * ALT FN MODE = 4
 *
 * Arduino Uno use sketch: 003I2CMasterRxStringLen.ino
 * the sketch uses the Arduino wire library and the code has been modified to accept > 32 bytes of data.
 * The sketch will send the STM32 a command code to request the length of data, then a command to start sending
 * data.
 * send/receive in one transaction
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

#define SLAVE_ADDR	 0x68 //Found in Arduino sketch
#define MY_ADDR		 SLAVE_ADDR
uint8_t CMD_LEN      =	0x51;
uint8_t CMD_READ     =	0x52;
uint8_t cmd_code     = 0;
uint32_t curPos      = 0; //Maintains the position of the buffer
uint8_t bytePos      = 0; //Maintains the byte position while sending the size to master


//Large buffer
uint8_t TxBuffer[] = "water massive nobody same science lifetime must spirit testimony creation deny can west poet basic convert discuss news be interested furniture point distinguish do admission water massive nobody same science lifetime must spirit testimony creation deny can west poet basic convert discuss news be interested furniture point distinguish do admission water massive nobody same science lifetime must spirit testimony creation deny can west poet basic convert discuss news be interested furniture point distinguish do admission ";
uint32_t buffSize; //Holds the size of Tx buffer
 
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

int main(void){

	//Setup GPIO pins for I2C
	I2C_GPIOInit();

	//I2C peripheral setup
	I2C1_Init();

	//Setup Interrupts for I2C1(Required because slave is always in interrupt mode)
	I2C_IRQITConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQITConfig(IRQ_NO_I2C1_ER, ENABLE);

	//Enable I2Cx Interrupt Control bits
	I2C_SlaveEnableOrDisableCallbackEvents(I2C1handle.pI2Cx, ENABLE);

	//Enable the I2C1 peripheral
	I2C_PeripheralControl(I2C1handle.pI2Cx, ENABLE);

	//Enable ACKing
	I2C_AckContol(I2C1handle.pI2Cx, I2C_ACK_ENABLE);

	//Get the size RxBuffer size
	buffSize = strlen((char*)TxBuffer);

	//Hang here wait for I2C1 interrupts
	while(1);

}
/* Here we define the I2C1 EV and ER IRQ handlers found in startup_stm32f407vgtx.s
*  When an interrupt is triggered these functions will be called 
*  passing the I2C1 handle to the I2C IRQ handling API to take the appropriate action
*/
void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C1handle);
} 
void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C1handle);
}
/*
* Here we check for the EV @I2C_CALLBACK_EVENTS and ER @I2C_CALLBACK_ERRORS interrupt and act accordingly
*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
	if(AppEv == I2C_EVENT_DATA_REQ){
		//Send Data to Master
		if(cmd_code == CMD_LEN){
			//Send length info to master
			//Sending 4 byte of size information
			I2C_SlaveSendData(pI2CHandle->pI2Cx, (buffSize >> (8 * (bytePos++ % 4))) & 0xFF);	 
		}
		else if(cmd_code == CMD_READ ){
			//Send data to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, TxBuffer[curPos++]);
		}
	}
	else if(AppEv == I2C_EVENT_DATA_RCV){
		//Receiving Data, read command codes
		cmd_code = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if(AppEv == I2C_ERROR_AF){
		//This happens only during slave Tx
		//Master hast sent a NACK, we must stop sending data
		//if curPos is buffer size we are finished sending the entire buffer(this is to protect against receiving a NACK before we have completed sending the buffer)
	 	if(curPos == buffSize){
	 		 cmd_code = 0xff;
	 		 curPos = 0;
	 		 bytePos = 0;
	 	}
	}
	else if(AppEv == I2C_EVENT_STOPF){
		//This happens only during slave reception
		//Master has ended the I2C communication
	}
}

