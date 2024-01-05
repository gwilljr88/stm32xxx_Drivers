/*
 * 001LEDToggle.c
 *
 *  Created on: Aug 14, 2023
 *      Author: geroldwilliams
 */
#include "stm32f407xx.h"
#include "stdio.h"
void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void){
	GPIO_Handle_t Gpioled;
	Gpioled.pGPIOx = GPIOD;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClkCtrl(Gpioled.pGPIOx, ENABLE);
	GPIO_Init(&Gpioled);

	while(1){
		GPIO_ToggleOutputPin(Gpioled.pGPIOx, GPIO_PIN_12);
		delay();
	};
	return 0;
}
