/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Oct 19, 2023
 *      Author: geroldwilliams
 */

#include "stm32f407xx.h"

/*********************************************************
 * @fn							- USART_Init
 *
 * @brief						- function to initialize USARTx peripheral for user application
 *
 * @param[in]					- *pUSARTHandle - pointer to USARTx peripheral handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note							- We enable the Rx engine but never disable it, this should be turned off for low power applications
 *
 * */

void USART_Init(USART_Handle_t *pUSARTHandle){

	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Enable the Clock for given USART peripheral
	USART_PeriClkCtrl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX )
	{
		//Enable the Receiver bit field 
		tempreg |= ( 1 << USART_CR1_RE );
	}else if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX )
	{
		//Enable the Transmitter bit field 
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Enable the both Transmitter and Receiver bit fields 
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

    //Configure the Word length configuration item 
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control 
		tempreg |= ( 1 << USART_CR1_PCE);

		//Enable EVEN parity 
		//Not required because by default EVEN parity will be selected once you enable the parity control 

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Enable the parity control 
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Enable ODD parity 
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register 
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Configure the number of stop bits inserted during USART frame transmission 
	tempreg |= pUSARTHandle->USART_Config.USART_NumOfStopBits << USART_CR2_STOP;

	//Program the CR2 register 
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;
	
	//Configuration of USART hardware flow control 
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Enable CTS flow control 
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Enable RTS flow control 
		tempreg |= ( 1 << USART_CR3_RTSE );

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Enable both CTS and RTS Flow control 
		tempreg |= ( 1 << USART_CR3_CTSE );
		tempreg |= ( 1 << USART_CR3_RTSE );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/
	//Configure Baud Rate with application settings
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}
/*********************************************************
 * @fn							- USART_DeInit
 *
 * @brief						- function to reset the USARTx peripheral
 *
 * @param[in]					- *pUSARTx - pointer to USARTx peripheral base address
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note							- NONE 
 *
 * */

void USART_DeInit(USART_RegDef_t *pUSARTx){
	if(pUSARTx == USART1){
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2){
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3){
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4){
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5){
		UART5_REG_RESET();
	}
	else if(pUSARTx == USART6){
		USART6_REG_RESET();
	}
	
}
/*********************************************************
 * @fn							- USART_PeriClkCtrl
 *
 * @brief						- function to enable of disable USARTx peripheral clock
 *
 * @param[in]					- *pUSARTx - pointer to USARTx peripheral base address
 * @param[in]					- EnorDi - ENABLE or DISABLE
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note							- NONE
 *
 * */

void USART_PeriClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5){
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}
	else{
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3){
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4){
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5){
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}
/*********************************************************
 * @fn							- USART_SendData
 *
 * @brief						- function to send data using USARTx peripheral
 *
 * @param[in]					- *pUSARTHandle - pointer to a structure that holds application USARTx configuration and data
 * @param[in]					- *pTxBuffer - pointer to buffer holding data to be sent
 * @param[in]					- Len - length of the data stored in pTxBuffer
 *
 * @return						- NONE
 *
 * @note							- BLOCKING FUNCTION
 *
 * */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t Len){

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_SR_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//If 9BIT, load the DR with 2 bytes masking the bits other than first 9 bits 
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
			
			//Check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9 bits of user data will be sent
				//Implement the code to increment pTxBuffer twice 
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8 bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8 bit data transfer 
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			
			//Increment the buffer address
			pTxBuffer++;
		}
	}

	//Wait till TC(Transmission complete) flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_SR_TC));
}
/*********************************************************
 * @fn							- USART_ReceiveData
 *
 * @brief						- function to send data using USARTx peripheral
 *
 * @param[in]					- *pUSARTHandle - pointer to structure that holds application configuration and data for USARTx
 * @param[in]					- *pRxBuffer - pointer to buffer that holds data received by USARTx 
 * @param[in]					- Len - length of the data stored in pRxBuffer
 *
 * @return						- NONE
 *
 * @note						- BLOCKING FUNCTION
 *
 * */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_SR_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9 bits of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9 bit data in a frame
			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9 bits will be of user data
				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x1FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8 bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 
				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8 bit data in a frame
			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8 bits will be of user data
				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}
			//Increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************
 * @fn							- USART_SendDataIT
 *
 * @brief						- function to store application Tx data/ enable TXE and TC(transmission complete) interrupt
 *
 * @param[in]					- *pUSARTHandle - pointer to structure that holds application configuration and data for USARTx
 * @param[in]					- *pTxBuffer - pointer to buffer holding data to be sent
 * @param[in]					- Len - length of the data stored in pTxBuffer
 *
 * @return						- returns the status of USARTx communication state
 *
 * @note						- NONE
 *
 * */

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t Len){
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		//Store application info
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE );
		
		//Enable interrupt for TC 
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE );
	}

	return txstate;
}
/*********************************************************
 * @fn							- USART_ReceiveDataIT
 *
 * @brief						- function to store application Rx data and enable RXNE interrupt
 *
 * @param[in]					- *pUSARTHandle - pointer to structure that holds application configuration and data for USARTx
 * @param[in]					- *pRxBuffer - pointer to buffer holding data to be received
 * @param[in]					- Len - length of the data stored in pRxBuffer
 *
 * @return						- returns the status of USARTx communication state see: USART_COMMUNICATION_STATUS
 *
 * @note							- NONE 
 *
 * */

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		//Store application information
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Dummy read to clear DR
		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE );
	}

	return rxstate;
}

/*********************************************************
 * @fn							- USART_IRQInterruptConfig
 *
 * @brief						- Enable or disable USARTx peripheral interrupt
 *
 * @param[in]					- IRQNumber - USARTx IRQ number
 * @param[in]					- EnorDI - use ENABLE or DISABLE
 *
 * @return						- NONE
 *
 * @note						   - NONE
 *
 * */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
 * @fn							- USART_IRQPriorityConfig
 *
 * @brief						- Sets the priority of a given interrupt
 *
 * @param[in]					- IRQNumber - Number of the USARTx interrupt
 * @param[in]					- IRQPriortiy - priority need for given IRQ
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

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
 * @fn							- USART_GetFlagStatus
 *
 * @brief						- Reads the USARTx SR register
 *
 * @param[in]					- *pUSARTx - pointer to the base address of USARTx peripheral
 * @param[in]					- FlagName - @USART_SR_FLAGS
 * @param[in]					-
 *
 * @return						- Returns the status of a given bit/flag in the SR register
 *
 * @note							- NONE
 *
 * */
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t FlagName){
	if(pUSARTx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************
 * @fn							- USART_ClearFlagStatus
 *
 * @brief						- function to clear the status of a given flag in the SR register
 *
 * @param[in]					- *pUSARTx - pointer to the base address of USARTx peripheral
 * @param[in]					- FlagName - @USART_SR_FLAGS
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note							- NONE
 *
 * */
void USART_ClearFlagStatus(USART_RegDef_t* pUSARTx, uint32_t FlagName){
	pUSARTx->SR &= ~( FlagName );
}

/*********************************************************
 * @fn							- USART_PeripheralControl
 *
 * @brief						- function to enable or disable the USARTx peripheral
 *
 * @param[in]					- *pUSARTx - pointer to the USARTx peripheral base address
 * @param[in]					- EnorDI - use ENABLE or DISABLE
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note							- NONE
 *
 * */
void USART_PeripheralControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pUSARTx->CR1 |= ( 1 << USART_CR1_UE );
	}
	else{
		pUSARTx->CR1 &= ~( 1 << USART_CR1_UE );
	}
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - function to set the baud rate of USARTx peripheral, calculate the value of Mantissa and Fraction for BRR register
 *
 * @param[in]         - *pUSARTx - pointer to USARTx peripheral base address
 * @param[in]         - BaudRate - baud rate for USARTx peripheral to operate at
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @note              - USARTDIV = fclk / ((8 x (2 - OVER8)) x Tx/Rx baud rate)
 * 							USARTDIV = Mantissa(Whole number).Fraction(Real number or decimal)
 * 							
 * 							Mantissa = USARTDIV / 100
 * 							
 * 							Fraction = USARTDIV - (Mantissa * 100)
 * 							Fraction = Fraction x ((8 x (2 - OVER8)) + 50(carry)) / 100
 *
 *
 *
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock value
	uint32_t PCLKx;
	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
	}else
	{
	   PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}else
	{
	   //Over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position. refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	}else
	{
	   //Over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//Copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*********************************************************
 * @fn							- USART_ClearOreFlag
 *
 * @brief						- function to clear the ORE flag
 *
 * @param[in]					- *pUSARTHandle - pointer to the USARTx peripheral handle structure
 * @param[in]					- 
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note							- this function should be call by the application in the event of an Overrun detection(ORE flag is set)
 *
 * */
void USART_ClearErrorFlag(USART_Handle_t *pUSARTHandle){
	uint32_t dummy_read = pUSARTHandle->pUSARTx->SR;
	dummy_read = pUSARTHandle->pUSARTx->DR;

	//Typecast to avoid compiler warning
	(void)dummy_read;
}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             - function to be called by processor when a USART interrupt has been generated
 *
 * @param[in]         - *pUSARTHandle - pointer to handle structure for USARTx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @note              - This function should be passed to the IRQ handler for the USARTx peripheral found in the startup file

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata = NULL;
	uint32_t dummy_read;

	
/*************************Check for RXNE flag ********************************************/

	//Check the state of RXNE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR  & ( 1 << USART_SR_RXNE );

	//Check the state of RXNEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE );


	if(temp1 && temp2 ){
		//this interrupt is because of RXNE
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX){
			if(pUSARTHandle->RxLen > 0){
				//Check the USART_WordLength to decide whether we are going to receive 9 bits of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					//We are going to receive 9 bits data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						//No parity is used. so, all 9 bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer twice
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						
						//Decrement Rx length twice
						pUSARTHandle->RxLen-=2;

					}
					else{
						//Parity is used. so, 8 bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 
						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;
						 
						 //Decrement Rx length
						 pUSARTHandle->RxLen-=1;
					}
				}
				else{
					//We are going to receive 8 bits data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						//No parity is used , so all 8 bits will be of user data
						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}
					else{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity
						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					
					//Decrement Rx length
					pUSARTHandle->RxLen-=1;
				}		
			}
			//If RxLen is 0, we are finished receiving
			if(!pUSARTHandle->RxLen)
			{
				//disable the RXNE interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				//Notify the application RX in complete
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}
	
/*************************Check for TC flag ********************************************/

    //Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC );
	
	 //Check the state of TCEIE bit 
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE) ;

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC
		
		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX )
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC );
				
				//Clear TCIE control bit 
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;
				
				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;
				
				//Reset the length to zero
				pUSARTHandle->TxLen = 0;
				
				//Notify the application TX in complete
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE );
	
	//Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE );


	if(temp1 && temp2 ){
		//this interrupt is because of TXE
		
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0){
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					//if 9BIT , load the DR with 2 bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					
					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						//No parity is used in this transfer , so, 9 bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						
						//Decrement Tx length twice
						pUSARTHandle->TxLen-=2;
					}
					else{
						//Parity bit is used in this transfer . so , 8 bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						
						//Decrement Tx length
						pUSARTHandle->TxLen-=1;

					}
				}
				else{				
					//This is 8 bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					
					//Decrement Tx length
					pUSARTHandle->TxLen-=1;
				}
				
			}
			if (pUSARTHandle->TxLen == 0 ){
				//TxLen is zero 
				//Clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &=  ~( 1 << USART_CR1_TXEIE );
			}
		}
	}
	
/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS );
	
	//Check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);
	
	//Check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 && temp3)
	{
		//Clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ( 1 << USART_SR_CTS );
		
		//Notify the application we are Clear to Send
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE );
	
	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE );


	if(temp1 && temp2)
	{
		//Clear the IDLE flag by reading SR register followed by DR register
		dummy_read = pUSARTHandle->pUSARTx->SR; //Could remove as temp1 has already completed a read of SR
		dummy_read = pUSARTHandle->pUSARTx->DR;
		//typecast to avoid compiler warning
		(void)dummy_read;
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;
	
	//Check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag . 
		
		//Notify the application there was an Overrun error 
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multi-buffer communication
//The below code will get executed in only if multi-buffer mode is used. 

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ClearErrorFlag(pUSARTHandle);
			//Notify the application Frame Error has been detected
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ClearErrorFlag(pUSARTHandle);
			//Notify the application hardware has detected noise
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}
} 
/*********************************************************************
 * @fn      		  	 - USART_ApplicationEventCallback
 *
 * @brief             - this function is called after an USARTx interrupt has occurred and should respond according to the AppEV:USART_APPLICATION_CALLBACK_STATUS
 *
 * @param[in]         - *pUSARTHandle - pointer to USARTx peripheral handle structure
 * @param[in]         - AppEv - see: USART_APPLICATION_CALLBACK_STATUS for possible errors and events
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @note              - this is a weak implementation, and should be written by application
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv){

}