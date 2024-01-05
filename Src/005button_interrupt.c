/*
 * 002led_button.c
 *
 *  Created on: Aug 15, 2023
 *      Author: geroldwilliams
 */
#include "stm32f407xx.h"
#define HIGH				ENABLE
#define LOW					DISABLE
#define	BUTTON_PRESSED		LOW
#define BUTTON_NOT_PRESSED	HIGH

void delay(void){
	//~200ms delay when the system clock is 16 MHz
	for(uint32_t i=0; i<500000/2; i++);
}

int main(void){
	//Create handler for button and LED
	GPIO_Handle_t GpioLed, GpioBtn;

	//Configure LED GPIO
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Initialize configuration for LED GPIO
	GPIO_PeriClkCtrl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	//Configure Button GPIO
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//Initialize configuration for Button GPIO
	GPIO_PeriClkCtrl(GpioBtn.pGPIOx, ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9, NVIC_IRQ_PRIORITY15);
	GPIO_ITIRQConfig(IRQ_NO_EXTI5_9, ENABLE);

	for(;;){}

	return 0;
}

void EXTI9_5_IRQHandler(void){
	//Delay for button debounce
	delay();
	//Clear the event form EXTI line
	GPIO_IRQHandling(GPIO_PIN_5);
	//Toggle LED
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}
