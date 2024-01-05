/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 3, 2023
 *      Author: geroldwilliams
 */

#include "stm32f407xx_i2c_driver.h"

/**Private helper functions NOT USED BY USER APPLICATION**/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
/********************************************************/

/*********************************************************
 * @fn							- SPI_Init
 *
 * @brief						- Function to initialize a SPIx peripheral for use
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handler structure that contains the SPIx base address and SPIx configuration structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//Enable the peripheral clock
	SPI_PeriClkCtrl(pSPIHandle->pSPIx, ENABLE);
	//Configure the SPI CR1 register
	uint32_t  tempreg = 0;

	//1) Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2) Configure the bus config
	//Clear BIDI mode bit
	if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
	}
	//Set BIDI mode bit
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		tempreg |= ( 1 << SPI_CR1_BIDIMODE );
	}
	//Clear BIDI mode and Set RxOnly bit
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == 	SPI_BUS_CONFIG_S_RXONLY){
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
		tempreg |= ( 1 << SPI_CR1_RXONLY );
	}
	//3) Configure the clock speed
	tempreg |= ( pSPIHandle->SPIConfig.SPI_ClockSpeed << SPI_CR1_BR );
	//4) Configure the data frame format
	tempreg |= ( pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF );
	//5) Configure the clock polarity
	tempreg |= ( pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL );
	//6) Configure the clock phase
	tempreg |= ( pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA );
	//6) Configure the slave select management
	tempreg |= ( pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM );
	//Write to CR1 Register
	pSPIHandle->pSPIx->CR1  = tempreg;
}

/*********************************************************
 * @fn							- SPI_DeInit
 *
 * @brief						- Function to De-initialize and free up resources that are no longer needed by the SPIx peripheral
 *
 * @param[in]					- *pSPIx - pointer to the base address of SPIx peripheral
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

/*********************************************************
 * @fn							- SPI_PeriClkCtrl
 *
 * @brief						- Function to Enable or Disable the SPI peripheral clock
 *
 * @param[in]					- *pSPIx - pointer to the base address of SPIx peripheral
 * @param[in]					- EnorDi - use ENABLE or DISABLE
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}

	}
}
/*********************************************************
 * @fn							- SPI_DataSend
 *
 * @brief						- Funciton to send 1 byte of data
 *
 * @param[in]					- *pSPIx - pointer to the base address of SPIx peripheral
 * @param[in]					- pTxBuffer - Tx buffer with data to be sent
 * @param[in]					- len - length of the Tx buffer to be sent
 *
 * @return						- NONE
 *
 * @note						- this is a blocking function, will only return when all bytes are sent
 *
 * */

void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t len){
	//loop until there is no more data to send
	while(len > 0){
		//Wait until Tx buffer is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_FLAG) == FLAG_RESET);
		//Check the DFF bit in the CR1 register
		//If 1 we are sending 2 bytes of data
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF )){
			//Load DR(Data register) with 1 byte of data and increment the buffer address
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			//decrease length by 2(Sending 2 bytes)
			len-=2;
			(uint16_t*)pTxBuffer++;
		}

		//Else we are sending 1 byte of data
		else{
			//Load DR(Data register) with 1 byte of data and increment the buffer address
			pSPIx->DR = *pTxBuffer;
			//decrease length
			len--;
			pTxBuffer++;
		}
	}
}

/*********************************************************
 * @fn							- SPI_DataReceive
 *
 * @brief						- Function to receive 1 byte of data
 *
 * @param[in]					- *pSPIx - pointer to the base address of SPIx peripheral
 * @param[in]					- pRxBuffer - Rx buffer to hold data received
 * @param[in]					- len - length of the data to be received
 *
 * @return						- value of the port @ *pGPIOx (16 bits)
 *
 * @note						- NONE
 *
 * */

void SPI_DataReceive(SPI_RegDef_t *pSPIx,  uint8_t* pRxBuffer, uint32_t len){
	//loop until there is no more data to send
		while(len > 0){
			//Wait until Rx buffer is empty
			while(SPI_GetFlagStatus(pSPIx, SPI_SR_RXE_FLAG) == FLAG_RESET);
			//Check the DFF bit in the CR1 register
			//If 1 we are receiving 2 bytes of data
			if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF )){
				//Load 2 bytes of data from DR(Data register) and increment the buffer address
				*(uint16_t*)pRxBuffer = pSPIx->DR;
				(uint16_t*)pRxBuffer++;
				//decrease length by 2(Sending 2 bytes)
				len-=2;
			}
			//Else we are receiving 1 byte of data
			else{
				//Load DR(Data register) with 1 byte of data and increment the buffer address
				*pRxBuffer = pSPIx->DR;
				pRxBuffer++;
				//decrease length
				len--;
			}
		}
}

/*********************************************************
 * @fn							- SPI_DataSendIT
 *
 * @brief						- Function to save user data buffer info and change the SPI status
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handle structure
 * @param[in]					- *pTxBuffer - pointer to user data pxTxBuffer
 * @param[in]					- len - length of user data buffer
 *
 * @return						- Current state of SPIx
 *
 * @note						- Data transmission will be handled be the ISR
 *
 * */
uint8_t SPI_DataSendIT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t len){
	//Get current SPI State
	uint8_t state = pSPIHandle->TxState;
	//We are not busy transmitting we are clear to send
	if(state != SPI_BUSY_IN_TX) {
		//1) Save the Tx buffer address an Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		//2) Mark the SPI state as busy in transmission so that no other code can take
		//   over the same SPI peripheral until transmission is complete
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3) Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
		//4) Data transmission will be handled by the ISR code

	}
	return state;
}

/*********************************************************
 * @fn							- SPI_DataReceiveIT
 *
 * @brief						- Function to save user data buffer info and change the SPI status
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handle structure
 * @param[in]					- *pTxBuffer - pointer to user data pxRxBuffer
 * @param[in]					- len - length of user data buffer
 *
 * @return						- Current state of SPIx
 *
 * @note						- Data transmission will be handled be the ISR
 *
 * */
uint8_t SPI_DataReceiveIT(SPI_Handle_t *pSPIHandle,  uint8_t* pRxBuffer, uint32_t len){
	//Get current SPI State
		uint8_t state = pSPIHandle->RxState;
		//We are not busy receiving we are clear to receive
		if(state != SPI_BUSY_IN_RX) {
			//1) Save the Rx buffer address an Len information in some global variables
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = len;
			//2) Mark the SPI state as busy receiving so that no other code can take
			//   over the same SPI peripheral until receive is complete
			pSPIHandle->RxState = SPI_BUSY_IN_TX;
			//3) Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
			//4) Data transmission will be handled by the ISR code

		}
		return state;
}
/*********************************************************
 * @fn							- SPI_ITIRQConfig
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
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//ISER0 Register
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= ( 1 << IRQNumber );
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
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 32) );
		}
		//IC ER2 Register  64 - 95
		else if((IRQNumber >= 64) && (IRQNumber < 96)){
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}


/*********************************************************
 * @fn							- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
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
 * @brief						- Manages the interrupt for a SPIx peripheral
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
void SPI_IRQHandling(SPI_Handle_t* pHandle){

	uint8_t temp1 , temp2;
	//Check for TXE Error
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE );
	if(temp1 && temp2){
		spi_txe_interrupt_handle(pHandle);
	}
	//Check for RXNE Error
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXE );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE );
	if(temp1 && temp2){
		spi_rxne_interrupt_handle(pHandle);
	}
	//Check for Overrun Error(occurs when the master or slave completes a reception
	//of the next data frame while the read operation of the previous frame from the Rx
	//buffer has not completed)
	//The data received will be discarded
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE );
	if(temp1 && temp2){
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

/*********************************************************
 * @fn							- SPI_GetFlagStatus
 *
 * @brief						- Reads the SR register
 *
 * @param[in]					- *pSPIx - pointer to the base address of SPIx peripheral
 * @param[in]					- FlagName - @SPI_SR_FLAGS
 * @param[in]					-
 *
 * @return						- Returns the status of a given bit/flag in the SR register
 *
 * @note						-
 *
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*********************************************************
 * @fn							- SPI_PeripheralControl
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

void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDI){
	if(EnorDI == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}
/*********************************************************
 * @fn							- SPI_SSIConfig
 *
 * @brief						- Function to enable or disable NSS pin, when SSM bit is set
 *
 * @param[in]					- *pSPIx - pointer to the base address of SPIx peripheral
 * @param[in]					- EnorDI - enable or disable
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */

void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDI){
	if(EnorDI == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}
	else{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}


/*********************************************************
 * @fn							- SPI_SSOEConfig
 *
 * @brief						- Funciton to enable or disable the NSS pin output enable
 *
 * @param[in]					- *pSPIx - pointer to the base address of SPIx peripheral
 * @param[in]					- EnorDI - enable or disable
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDI){
	if(EnorDI == ENABLE){
			pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
		}
		else{
			pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
		}
}

/*********************************************************
 * @fn							- spi_txe_interrupt_handle
 *
 * @brief						- ISR for TXNE interrupt
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- DRIVER ONLY FUNCTION, NOT USED BY APPLICATION
 *
 * */
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//Check the DFF bit in the CR1 register
	//If 1 we are sending 2 bytes of data
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF )){
		//Load DR(Data register) with 1 byte of data and increment the buffer address
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		(uint16_t*)pSPIHandle->pTxBuffer++;
		//decrease length by 2(Sending 2 bytes)
		pSPIHandle->TxLen -= 2;
	}
	//Else we are sending 1 byte of data
	else{
		//Load DR(Data register) with 1 byte of data and increment the buffer address
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer++;
		//decrease length
		pSPIHandle->TxLen--;
	}
	//If the length is 0 we are finished transmitting and must close the SPIx peripheral
	//and inform the application that the Tx is over
	if(!pSPIHandle->TxLen){
		//Clear the TXEIE bit to prevent interrupts from setting the TXE flag
		SPI_CloseTransmission(pSPIHandle);
		//Notify the application the Tx event has been completed
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/*********************************************************
 * @fn							- spi_rxne_interrupt_handle
 *
 * @brief						- ISR for RXNE interrupt
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- DRIVER ONLY FUNCTION, NOT USED BY APPLICATION
 *
 * */
void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//Check the DFF bit in the CR1 register
		//If 1 we are receiving 2 bytes of data
		if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF )){
			//Load DR(Data register) with 1 byte of data and increment the buffer address
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
			(uint16_t*)pSPIHandle->pRxBuffer++;
			//decrease length by 2(Sending 2 bytes)
			pSPIHandle->RxLen-=2;
		}
		//Else we are sending 1 byte of data
		else{
			//Load DR(Data register) with 1 byte of data and increment the buffer address
			*(pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->pRxBuffer++;
			//decrease length
			pSPIHandle->RxLen--;
		}
		//If the length is 0 we are finished transmitting and must close the SPIx peripheral
		//and inform the application that the Rx is over
		if(!pSPIHandle->RxLen){
			//Close Reception
			SPI_CloseReception(pSPIHandle);
			//Notify the application the Rx event has been completed
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//Clear the Overrun flag and inform application
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//Type-cast to avoid compiler warning
	(void)temp;
	//Notify the application an Overrun error has occurred
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx){
	//Clear the Overrun flag and inform application
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	//Type-cast to avoid compiler warning
	(void)temp;
}

/*********************************************************
 * @fn							- SPI_CloseTransmission
 *
 * @brief						- Closes the transmission of the SPIx peripheral
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//Clear the TXEIE bit to prevent interrupts from setting the TXE flag
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
	//Clear Tx buffer
	pSPIHandle->pTxBuffer = NULL;
	//Reset Tx buffer length
	pSPIHandle->TxLen = 0;
	//Clear the Tx state
	pSPIHandle->TxState = SPI_READY;
}

/*********************************************************
 * @fn							- SPI_CloseReception
 *
 * @brief						- Closes the reception of the SPIx peripheral
 *
 * @param[in]					- *pSPIHandle - pointer to SPIx handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//Clear the RXEIE bit to prevent interrupts from setting the TXE flag
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
	//Clear Rx buffer
	pSPIHandle->pRxBuffer = NULL;
	//Reset Rx buffer length
	pSPIHandle->RxLen = 0;
	//Clear the Rx state
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	//This is a weak implementation, the application my override this function
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

