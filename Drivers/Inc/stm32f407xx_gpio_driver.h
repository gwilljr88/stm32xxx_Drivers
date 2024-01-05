/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Aug 3, 2023
 *      Author: geroldwilliams
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


//GPIOx Pin Configuration Structure
typedef struct{
	uint8_t GPIO_PinNumber;			/*!< possible values @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode; 			/*!< possible values @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/*!< possible values @GPIO_SPEED */
	uint8_t GPIO_PinPuPdControl;	/*!< possible values @GPIO_PUPD_CONFIG */
	uint8_t GPIO_PinOType;			/*!< possible values @GPIO_OUTPUT_TYPES */
	uint8_t GPIO_PinAltFunMode;		/*!< possible values @GPIO_MODE_ALTFN */
}GPIO_PinConfig_t;


//GPIOx Handle Structure
typedef struct{
	GPIO_RegDef_t *pGPIOx; /*!< this holds the base address of the GPIO port to which the pin belongs > */
	GPIO_PinConfig_t GPIO_PinConfig; /*!< holds the GPIO pin configuration settings >*/
}GPIO_Handle_t;
/********************************************************* GPIO CONFIG SETTINGS *************************************************************/
/*
 * @GPIO_PIN_NUMBERS
 * GPIO possible pin numbers
 * */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO possible pin modes
 * */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_OUTPUT_TYPES
 * GPIO Output type
 * */
#define GPIO_OUTPUT_TYPE_PP		0
#define GPIO_OUTPUT_TYPE_OD		1

/*
 * @GPIO_SPEED >
 * GPIO possible speeds
 * */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD_CONFIG
 * GPIO possible Pull up Pull down configuration
 * */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/********************************************************************************************************************************************/


/************************************************************************
 * 						Supported API's
 *
 * 				Check function definition for more info
 * **********************************************************************
 * */
// Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Peripheral Clock Setup
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and ISR Handling
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
