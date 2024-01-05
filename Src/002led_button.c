/*
 * 002led_button.c
 *
 *  Created on: Aug 15, 2023
 *      Author: geroldwilliams
 */
#include "stm32f407xx.h"

#define HIGH				ENABLE
#define LOW					DISABLE
#define	BUTTON_PRESSED		HIGH
#define BUTTON_NOT_PRESSED	LOW

void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GpioBtn;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClkCtrl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClkCtrl(GpioBtn.pGPIOx, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GpioBtn.pGPIOx,GpioBtn.GPIO_PinConfig.GPIO_PinNumber) ==
				BUTTON_PRESSED){
			delay();//account for debounce of button
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		}
	};
	return 0;
}

