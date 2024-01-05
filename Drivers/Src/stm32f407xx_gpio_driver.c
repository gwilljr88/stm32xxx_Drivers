/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 3, 2023
 *      Author: geroldwilliams
 */

#include "stm32f407xx.h"

/*********************************************************
 * @fn							- GPIO_Init
 *
 * @brief						- Funciton to initialize a GPIO port and pin for use
 *
 * @param[in]					- *pGPIOHandle - pointer to GPIO handler structure that contains the GPIO port base address and pin configuration structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0;
	//Enable the clock
	GPIO_PeriClkCtrl(pGPIOHandle->pGPIOx, ENABLE);
	//Configure the mode
	//Separate Interrupt and Non-Interrupt modes
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//Non-Interrupt Mode
		//left shifted by 2 to place in proper position(2 bits)
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//Clear bit field before Setting to insure the field is clear
		pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//write the value to the register
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0; //reset temp
	}
	else{
		//Interrupt Modes
		//1) Configure the FTSR(Falling Edge Trigger)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//Set the corresponding FTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit for safety
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//1) Configure the RTSR(Rising Edge Trigger)
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//Set the corresponding RTSR bit
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit for safety
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//1) Configure both FTSR and RTSR(Falling and Rising Edge Trigger)
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//Set the corresponding FTSR and RTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2) Configure the GPIO port selection in SYSCFG_EXTICR
		//find the correct configuration register for the given pin
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		//find the start position within the configuration register
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		//Get the Port Code value to store in the configuration register
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		//Must enable the clock before we set the register
		SYSCFG_PCLK_EN();
		//Set the Configuration register "temp1" to portCode shifted by "temp * 4"
		SYSCFG->EXTICR[temp1] = portCode << ( temp2 * 4);

		//3) Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	//Configure the speed
	//left shifted by 2 to place in proper position(2 bits)
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	//Clear bit field before Setting to insure the field is clear
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	//write the value to the register
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0; //reset temp

	//Configure the Pull up Pull down
	//left shifted by 2 to place in proper position(2 bits)
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	//Clear bit field before Setting to insure the field is clear
	pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	//write the value to the register
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//temp = 0; //reset temp

	//Configure the Output Type
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	//Clear bit field before Setting to insure the field is clear
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	//write the value to the register
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0; //reset temp

	//Configure the Alternate Function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		//Figure out which alternate function register the pin is in HIGH(0) or LOW(1)
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		//Figure out the location to within the register for the given pin
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		//Clear bit field before Setting to insure the field is clear
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		//Left shift by 4 bits(Size of pin AltFn register) and set bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}
}

/*********************************************************
 * @fn							- GPIO_DeInit
 *
 * @brief						- Function to Deinitialize and free up resources that are no longer needed
 *
 * @param[in]					- *pGPIOx - pointer peripheral port base address
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}

/*********************************************************
 * @fn							- GPIO_PeriClkCtrl
 *
 * @brief						- Funtion to Enable or Disable the peripheral clock register
 *
 * @param[in]					- *pGPIOx - pointer peripheral port base address
 * @param[in]					- EnorDi - use ENABLE or DISABLE
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}

	}
}

/*********************************************************
 * @fn							- GPIO_ReadFromInputPin
 *
 * @brief						- Reads the input from a GPIO pin
 *
 * @param[in]					- *pGPIOx - pointer peripheral port base address for a given pin
 * @param[in]					- PinNumber - pin to read
 * @param[in]					-
 *
 * @return						- value of PinNumber (0 or 1)
 *
 * @note						- NONE
 *
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value = 0;

	//Right shift the bit into the least significant bit and mask all other Don't Care bits
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return(value);
}

/*********************************************************
 * @fn							- GPIO_ReadFromInputPort
 *
 * @brief						- Reads the input from a given port(16 pins)
 *
 * @param[in]					- *pGPIOx - pointer peripheral port base address for a given pin
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- value of the port @ *pGPIOx (16 bits)
 *
 * @note						- NONE
 *
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value = 0;

	//Right shift the bit into the least significant bit and mask all other Don't Care bits
	value = (uint16_t)pGPIOx->IDR ;

	return(value);
}

/*********************************************************
 * @fn							- GPIO_WriteToOutputPin
 *
 * @brief						- Writes some value to a pin
 *
 * @param[in]					- *pGPIOx - pointer peripheral port base address for a given pin
 * @param[in]					- PinNumber - the data output pin
 * @param[in]					- Value - value to output
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	//Write a 1 to the output data register at the given bit position(PinNumber)
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= ( 1 << PinNumber );
	}
	//Write a 0 to the output data register at the given bit position(PinNumber)
	else{

		pGPIOx->ODR &= ~( 1 << PinNumber );
	}
}

/*********************************************************
 * @fn							- GPIO_WriteToOutputPort
 *
 * @brief						- Writes a given value to a port(16 pins)
 *
 * @param[in]					- *pGPIOx - pointer peripheral port base address for a given pin
 * @param[in]					- Value - the value to output
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	//Write Value to the entire port
	pGPIOx->ODR = Value;
}

/*********************************************************
 * @fn							- GPIO_ToggleOutputPin
 *
 * @brief						- Reads the output data register and toggles the output of a given pin
 *
 * @param[in]					- *pGPIOx - pointer peripheral port base address for a given pin
 * @param[in]					- PinNumber - pin to toggle
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	//Use bitwise XOR to toggle the output of PinNumber
	pGPIOx->ODR ^= ( 1 << PinNumber);
}

/*********************************************************
 * @fn							- GPIO_IRQITConfig
 *
 * @brief						- Enable or disable a given interrupt and sets its' priority
 *
 * @param[in]					- IRQNumber - IRQ number to enable or disable
 * @param[in]					-
 * @param[in]					- ENorDI - use ENABLE or DISABLE
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//ISER0 Register
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= 1 << IRQNumber;
		}
		//ISER1 Register 32 - 63
		else if( (IRQNumber > 31) && (IRQNumber < 64) ){
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		//ISER2 Register 64 - 95
		else if((IRQNumber >= 64) && (IRQNumber < 96)){
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else{
		//ICER0 Register
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		//ICER1 Register 32 - 63
		else if( (IRQNumber > 31) && (IRQNumber < 64) ){
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		//IC ER2 Register  64 - 95
		else if((IRQNumber >= 64) && (IRQNumber < 96)){
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}


/*********************************************************
 * @fn							- GPIO_IRQPriorityConfig
 *
 * @brief						- Sets the priority of a given interrupt
 *
 * @param[in]					- IRQNumber - Number of the given interrupt
 * @param[in]					- IRQPriortiy - priority need for given IRQ
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//Find the IPRx Register
	uint32_t iprx = IRQNumber / 4;
	//Find the correct position for the given IRQ within the register
	uint8_t iprx_section = IRQNumber % 4;
	//Calculate the amount to shift because the lower 4 bits
	//in the 8 bit section for each IRQ priority is not used
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + (iprx) ) |= ( IRQPriority << shift_amount );
}

/*********************************************************
 * @fn							- GPIO_IRQHandling
 *
 * @brief						- Manages the interrupt for a given pin number
 *
 * @param[in]					- PinNumber - interrupt pin number
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void GPIO_IRQHandling(uint8_t PinNumber){
	//Clear the EXTI Priority register corresponding to the pin number
	//if the correspond bit in the Pending Register has been set
	//we must clear it bit writing
	if(EXTI->PR & ( 1 << PinNumber )){
		EXTI->PR |= ( 1 << PinNumber );
	}
}
/*********************************************************
 * @fn							-
 *
 * @brief						-
 *
 * @param[in]					-
 * @param[in]					-
 * @param[in]					-
 *
 * @return						-
 *
 * @note						-
 *
 * */

