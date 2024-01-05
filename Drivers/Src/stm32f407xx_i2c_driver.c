/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 7, 2023
 *      Author: geroldwilliams
 */
#include "stm32f407xx.h"

/****************************************************PRIVATE FUNCTION**********************************************/
static void I2C_GenerateStartCondidtion(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t readWrite);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
#define I2C_MASTER_WRITE 0
#define I2C_MASTER_READ  1
/*********************************************************
 * @fn							- I2C_GenerateStartCondidtion
 *
 * @brief						- Sets the START bit of CR1 register to generate start condition on I2Cx bus
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral base address
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
static void I2C_GenerateStartCondidtion(I2C_RegDef_t *pI2Cx){
	//Set the START bit in CR1 register
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}
/*********************************************************
 * @fn							- I2C_ExecuteAddressPhase
 *
 * @brief						- Sends the 7-bit slave address + R/W bit to the I2Cx data register
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral base address
 * @param[in]					- slaveAddr - 7 bit address
 * @param[in]					- readWrite - sets the R/W bit
 *
 * @return						-
 *
 * @note						-
 *
 * */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t readWrite){
	//Shift slave address to left by 1 to make room for R/W bit
	slaveAddr = slaveAddr << 1;
	if(!readWrite){
		//Clear the LSB/0th bit(R/W) for Write operation
		slaveAddr &= ~(1);
	}
	else{
		//Set the LSB/0th bit(R/W) for Read operation
		slaveAddr |= 1;
	}
	//Put the address into the Data Register
	pI2Cx->DR = slaveAddr;
}
/*********************************************************
 * @fn							- I2C_ClearADDRFlag
 *
 * @brief						- Executes a dummy read of SR1 and SR2 registers
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;
	//Check device mode
	//Master device (MSL = 1)
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )){
		//Master is receiving
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			//Receiving 1 byte
			if(pI2CHandle->RxSize == 1){
				I2C_AckContol(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Clear the ADDR flag, read SR1 followed by SR2
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				//type cast to avoid compiler warning
				(void)dummy_read;
			}
		}
		else{
			//Clear the ADDR flag, read SR1 followed by SR2
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			//type cast to avoid compiler warning
			(void)dummy_read;
		}
	}
	//Slave device
	else{
		//Clear the ADDR flag, read SR1 followed by SR2
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		//type cast to avoid compiler warning
		(void)dummy_read;
	}

}
/*********************************************************
 * @fn							- I2C_MasterHandleTXEInterrupt
 *
 * @brief						- function the handle the TXE interrupt, transmitting data
 * 
 * @param[in]					- pI2CHandle - pointer to I2Cx peripheral handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxLen > 0){
		//Load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		//Decrement TxLen
		pI2CHandle->TxLen--;
		//Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}
/*********************************************************
 * @fn							- I2C_MasterHandleRXNEInterrupt
 *
 * @brief						- function the handle the RXNE interrupt, receiving data
 * 
 * @param[in]					- pI2CHandle - pointer to I2Cx peripheral handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	//Receive a 1 byte
	if(pI2CHandle->RxSize == 1){
		//Store data from DR into RxBuffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

		//Decrement RxLen
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen == 2){
			//Disable ACKing
			I2C_AckContol(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
		}
		//Store data from DR into RxBuffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

		//Increment the buffer address
		pI2CHandle->pRxBuffer++;

		//Decrement RxLen
		pI2CHandle->RxLen--;
	}
	//We are finished receiving bytes, close communication
	if(pI2CHandle->RxLen == 0){
		//Generate STOP condition
		if(pI2CHandle->Sr == I2C_DI_SR){
			I2C_GenerateStopCondidtion(pI2CHandle->pI2Cx);
		}
		//Close communication
		I2C_CloseReceiveData(pI2CHandle);
		//Notify application Rx is complete
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_RX_CMPLT);
	}
}
/******************************************************************************************************************/

/*********************************************************
 * @fn							- I2C_Init
 *
 * @brief						- Function to initialize a I2Cx peripheral for use
 *
 * @param[in]					- *pI2CHandle - pointer to I2Cx handler structure that contains the I2Cx base address and I2Cx configuration structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempReg = 0;

	//Enable I2C peripheral clock
	I2C_PeriClkCtrl(pI2CHandle->pI2Cx, ENABLE);

	//Configure ACK Control bit in CR1
	tempReg |= ( pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempReg;

	//Configure the FREQ field of CR2
	tempReg = 0;
	tempReg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = tempReg & 0x3F;

	//Set the device's own address in OAR
	//Shift the address by 1 to skip the ADD0 bit field
	tempReg = 0;
	tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	//According to the reference manual bit 14 must be kept at 1 by software
	tempReg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempReg;

	//Configure the CCR field in CCR
	//Calculate CCR
	uint16_t ccr_value = 0;
	tempReg = 0;
	//Standard Mode calculation
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){
		// Thigh = CCR * TPCLK1 & Tlow  = CCR * TPCLK1
		// Thigh + Tlow = Tscl
		// Tscl = 2*CCR*Tpclk1
		// CCR = tscl/2*Tpclk1 ---frequency domain----> CCR = fpclk1/2*ftscl
		ccr_value = RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		//Keep the first 12 bits and mask the others
		tempReg  |= ( ccr_value & 0xFFF );
	}
	//Fast Mode calculation
	else{
		//Set F/S bit to fast mode
		tempReg |= (1 << 15);
		//Set the duty cycle
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		//Duty = 0
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			//CCR = tscl/3*Tpclk1 ---frequency domain----> CCR = fpclk1/3*ftscl
			ccr_value = RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			//Keep the first 12 bits and mask the others
			tempReg  |= ( ccr_value & 0xFFF );
		}
		//Duty = 1
		else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9){
		//CCR = tscl/25*Tpclk1 ---frequency domain----> CCR = fpclk1/25*ftscl
			ccr_value = RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			//Keep the first 12 bits and mask the others
			tempReg  |= ( ccr_value & 0xFFF );
		}
	}
	pI2CHandle->pI2Cx->CCR = tempReg;
	//Clear tempReg
	tempReg = 0;
	//Trise Configuration
	//Standard Mode
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){
		// Trise(max) / TPCLK1 => FPCLK1 x Trise(max)
		tempReg = ( RCC_GetPCLK1Value() / 1000000U ) + 1;
	}
	//Fast Mode calculation
	else{
		// Trise(max) / TPCLK1 => FPCLK1 x Trise(max)
		tempReg = ( ( RCC_GetPCLK1Value() * 300 )/ 1000000U ) + 1;
	}
	//Program TRISE with calculated value
	//Masked with 0x3F to preserve upper 10 bits[15:6]
	pI2CHandle->pI2Cx->TRISE = (tempReg & 0x3F);
}
/*********************************************************
 * @fn							- I2C_DeInit
 *
 * @brief						- Function to De-initialize and free up resources that are no longer needed by the I2Cx peripheral
 *
 * @param[in]					- *pI2Cx - pointer to the base address of I2Cx peripheral
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

/*********************************************************
 * @fn							- I2C_PeriClkCtrl
 *
 * @brief						- Function to Enable or Disable the I2Cx peripheral clock
 *
 * @param[in]					- *pI2Cx - pointer to the base address of I2Cx peripheral
 * @param[in]					- EnorDi - use ENABLE or DISABLE
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- NONE
 *
 * */
void I2C_PeriClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}

	}
}
/*********************************************************
 * @fn							- I2C_PeripheralControl
 *
 * @brief						- function to enable or disable I2Cx peripheral
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral base address
 * @param[in]					- EnorDI - ENABLE or DISABLE
 * @param[in]					-
 *
 * @return						-
 *
 * @note						- Control Register 1 bit PE: 0 = Disable, 1 = enable
 *
 * */


void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDI){
	if(EnorDI == ENABLE){
			pI2Cx->CR1 |= ( 1 << I2C_CR1_PE);
		}
	else{
			pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE);
		}
}
/*********************************************************
 * @fn							- I2C_MasterSendData
 *
 * @brief						- Send data using I2Cx peripheral on the I2C bus to a device with the address given
 *
 * @param[in]					- *pI2CHandle - handle structure for I2Cx peripheral
 * @param[in]					- *pTxBuffer  - pointer to buffer address of data to send
 * @param[in]					- Len		  - length(size in bytes) of pTxBuffer
 * @param[in]					- slaveAddr   - address of slave device
 * @param[in]					- Sr   		  - flag for start repeat, if set send start repeat if not send stop @I2C_START_REPEAT
 *
 * @return						-
 *
 * @note						-
 *
 * */


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t slaveAddr, uint8_t Sr){
	//1) Generate START condition
	I2C_GenerateStartCondidtion(pI2CHandle->pI2Cx);

	//2) Wait until start generation is completed by checking the SB flag in the
	//   SR1 register
	//   Note: Until SB is cleared, SCL will be stretched(pulled LOW)
	//wait until SB bit = 0, clears the SB by reading
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_SB));

	//3) Send the address to the slave with r/nw bit set w(0) (8-bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_WRITE);

	//4) Confirm the address phase is complete by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_ADDR));

	//5) Clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6) Send the data until Len becomes 0
	while(Len > 0){
		//Check if TXE = 1 so we can send data
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_TXE));
		//Load the data register with data from Tx buffer
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		//Increment Tx buffer
		pTxBuffer++;
		//Decrement Len
		Len--;
	}

	//7) When Len is 0 wait for TXE = 1 and BTF = 1 before generating the STOP condition
	//   Note: TXE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	//   when BTF = 1 SCL will be stretched(pulled LOW0
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_BTF));

	//8) Generate STOP condition and master need not to wait for the completion of stop condition
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DI_SR){
		I2C_GenerateStopCondidtion(pI2CHandle->pI2Cx);
	}
}

/*********************************************************
 * @fn							- I2C_MasterReceiveData
 *
 * @brief						- Receive data using I2Cx peripheral on the I2C bus from a device with the address given
 *
 * @param[in]					- *pI2CHandle - handle structure for I2Cx peripheral
 * @param[in]					- *pRxBuffer  - pointer to buffer address of data to received
 * @param[in]					- Len		  - length(size in bytes) of pRxBuffer
 * @param[in]					- slaveAddr   - address of slave device
 * @param[in]					- Sr   		  - flag for start repeat, if set send start repeat if not send stop @I2C_START_REPEAT
 *
 * @return						-
 *
 * @note						-
 *
 * */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr){
	//1) Generate the START condition
	I2C_GenerateStartCondidtion(pI2CHandle->pI2Cx);

	//2) Confirm that start generation is completed by checking the SB flag in the SR1 register
	//   Note: Until SB is cleared the SCL line will be stretched(Pulled LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_SB));

	//3) Send the address of the slave with R/W bit set to r(1) (8-bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_READ);

	//4) Wait until address phase is completed by checking the ADDR flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_ADDR));

	//Procedure to read only 1 byte from slave
	if(Len == 1){
		//Disable ACKing
		I2C_AckContol(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_RXNE));

		//Generate STOP condition
		if(Sr == I2C_DI_SR){
			I2C_GenerateStopCondidtion(pI2CHandle->pI2Cx);
		}
		//Read the data from data register into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	if(Len > 1){
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read data until Len becomes 0
		for(uint8_t i = Len; i>0 ; i--){
			//Wait until RXNE is set
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_RXNE));

			//If only 2 bytes are remaining
			if(i == 2){
				//Disable ACKing(Clear the ACK bit)
				pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK );

				//Generate STOP condition
				if(Sr == I2C_DI_SR){
					I2C_GenerateStopCondidtion(pI2CHandle->pI2Cx);
				}
			}
			//Read the data from data register into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//Increment buffer address
			pRxBuffer++;
		}
	}
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		//Enable ACKing(Set the ACK bit)
		I2C_AckContol(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/*********************************************************
 * @fn							- I2C_MasterSendDataIT
 *
 * @brief						- function to store application Tx data and enable error,event, and buffer interrupt to handle Tx data
 *
 * @param[in]					- *pI2CHandle - handle structure for I2Cx peripheral
 * @param[in]					- *pRxBuffer  - pointer to buffer address of data to received
 * @param[in]					- Len		  - length(size in bytes) of pRxBuffer
 * @param[in]					- slaveAddr   - address of slave device
 * @param[in]					- Sr   		  - flag for start repeat, if set send start repeat if not send stop @I2C_START_REPEAT
 *
 * @return						- returns the communication state of the I2Cx peripheral
 *
 * @note						-
 *
 * */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t slaveAddr, uint8_t Sr){
	//Get the communication state of I2Cx peripheral
	uint8_t busyState = pI2CHandle->TxRxState;

	//If not current state is not busy, prepare the I2Cx handle structure
	if( (busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate start condition
		I2C_GenerateStartCondidtion(pI2CHandle->pI2Cx);

		//Enable buffer interrupt, set ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable enable event interrupt, set ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable error interrupt, set ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busyState;
}

/*********************************************************
 * @fn							- I2C_MasterReceiveDataIT
 *
 * @brief						- function to store application Rx data and enable error,event, and buffer interrupt to handle Rx data
 *
 * @param[in]					- *pI2CHandle - handle structure for I2Cx peripheral
 * @param[in]					- *pRxBuffer  - pointer to buffer address of data to received
 * @param[in]					- Len		  - length(size in bytes) of pRxBuffer
 * @param[in]					- slaveAddr   - address of slave device
 * @param[in]					- Sr   		  - flag for start repeat, if set send start repeat if not send stop @I2C_START_REPEAT
 *
 * @return						- returns the communication state of the I2Cx peripheral
 *
 * @note						-
 *
 * */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr){
	//Get the communication state of I2Cx peripheral
	uint8_t busyState = pI2CHandle->TxRxState;

	//If not current state is not busy, prepare the I2Cx handle structure
	if( (busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX)){
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		//RxSize is used by ISR code to manage data reception
		pI2CHandle->RxSize = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = slaveAddr;

		//Generate start condition
		I2C_GenerateStartCondidtion(pI2CHandle->pI2Cx);

		//Enable buffer interrupt, set ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable enable event interrupt, set ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable error interrupt, set ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busyState;
}

/*********************************************************
 * @fn							- I2C_SlaveSendData
 *
 * @brief						- function to send data to master device while acting as slave device during I2Cx communication
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral base address
 * @param[in]					- data - byte of data to send to master
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}

/*********************************************************
 * @fn							- I2C_SlaveReceiveData
 *
 * @brief						- function to store a byte data sent by master device, while acting as slave device
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral base address
 * @param[in]					-
 * @param[in]					-
 *
 * @return						-
 *
 * @note						-
 *
 * */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return((uint8_t)pI2Cx->DR);
}

/*********************************************************
 * @fn							- I2C_GetFlagStatus
 *
 * @brief						- Reads the I2Cx SR1 register
 *
 * @param[in]					- *pI2Cx - pointer to the base address of I2Cx peripheral
 * @param[in]					- FlagName - @I2C_SR_FLAGS
 * @param[in]					-
 *
 * @return						- Returns the status of a given bit/flag in the SR register
 *
 * @note						-
 *
 * */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************
 * @fn							- I2C_AckContol
 *
 * @brief						- ENABLES/DISABLES the ACKing for I2Cx peripheral
 *
 * @param[in]					- *pI2Cx - pointer to the base address of SPIx peripheral
 * @param[in]					- EnorDI - @I2C_ACK_CONTROL
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
void I2C_AckContol(I2C_RegDef_t* pI2Cx, uint8_t EnorDI){
	if(EnorDI == I2C_ACK_ENABLE){
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK );
	}
	else{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK );
	}
}

/*********************************************************
 * @fn							- I2C_IRQITConfig
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
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
 * @fn							- I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//Find the IPRx Register
	uint32_t iprx = IRQNumber / 4;
	//Find the correct position for the given IRQ within the register
	uint8_t iprx_section = IRQNumber % 4;
	//Calculate the amount to shift because the lower 4 bits
	//in the 8 bit section for each IRQ priority is not used
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + (iprx) ) |= ( IRQPriority << shift_amount );
}
/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             - function to handle interrupts for both master and slave mode of a device
 *
 * @param[in]         - *pI2CHandle - pointer to I2Cx peripheral handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Interrupt handling for different I2C events (refer SR1)

 */


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1 , temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3){
		//SB Event
		//This block will not executed in slave mode because, SB is always 0 in slave mode
		//Check I2Cx application state and execute address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			//We are sending data
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_MASTER_WRITE);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			//We are receiving data
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_MASTER_READ);
		}
	}
	
	//2. Handle For interrupt generated by ADDR event 
	//Note : When master mode : Address is sent 
	//		 When Slave mode   : Address matched with own address

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );  
	if(temp1 && temp3){
		//ADDR Event
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event  
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );  
	if(temp1 && temp3){
		//BTG Event
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			//We are in Tx state
			//Check the TXE flag
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				//BTF = 1 and TXE = 1, we are finished transmitting
				//Check Tx Length
				if(pI2CHandle->TxLen == 0){
					//Generate Stop Condition
					if(pI2CHandle->Sr == I2C_DI_SR){
						I2C_GenerateStopCondidtion(pI2CHandle->pI2Cx);
					}
					//Reset all member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//Notify the application the transmission is complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			//We are in Rx state
			//Nothing to do here
		}
	}
	
	//4. Handle For interrupt generated by STOPF event 
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );  
	if(temp1 && temp3){
		//STOF Event
		//Cleared by reading SR1(we have done previously in Step 3) followed by writing to CR1
		//Do a dummy write to avoid corrupting the I2Cx peripheral CR1 register
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify application stop has been detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOPF);
	}
		
	//5. Handle For interrupt generated by TXE event 
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE );  
	if(temp1 && temp2 && temp3){
		//TXE Event
		//If device is master(MSL bit = 1), we transmit
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )){
			//We transmit here only if the application state in I2C_BUSY_IN_TX
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else{
			//We are a slave device sending data
			//Check to TRA bit in SR, if set were are transmitting
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_REQ);
			}
		}
	}
	
	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );
	if(temp1 && temp2 && temp3){
		//RXNE Event
		//Check if device is master
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )){
			//We receive here only if the application state in I2C_BUSY_IN_RX
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else{
			//We are a slave device receiving data
			//Check to TRA bit in SR, if clear were are receiving
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_RCV);
			}
		}
	} 

}

/*********************************************************
 * @fn							- I2C_CloseReceiveData
 *
 * @brief						- function the clear I2Cx event interrupts, application Rx data and application state
 *
 * @param[in]					- *pI2CHandle - pointer to I2Cx peripheral handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	//Clear interrupts to prevent us from closing communications
	//Clear ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Clear ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//Clear application state
	pI2CHandle->TxRxState = I2C_READY;

	//Reset I2C handle structure Rx data
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	//Re-enable ACking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_AckContol(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}
/*********************************************************
 * @fn							- I2C_CloseSendData
 *
 * @brief						- function the clear I2Cx event interrupts, application Tx data and application state
 *
 * @param[in]					- *pI2CHandle - pointer to I2Cx handle structure
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	//Clear interrupts to prevent us from closing communications
	//Clear ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Clear ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//Clear application state
	pI2CHandle->TxRxState = I2C_READY;

	//Reset I2C handle structure Tx data
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

	//Re-enable ACking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_AckContol(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}
/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         - *pI2CHandle - pointer the I2Cx peripheral handler
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @note              - 
 *
 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1,temp2;

    //Get the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


//Check for Bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_BERR);
	if(temp1  && temp2 ){
		//Clear the buss error flag 
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);
		
		//Notify the application about the error 
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

//Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2){
		
		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		
		//Implement the code to notify the application about the error 
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

//Check for ACK failure  error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error 
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

//Check for Overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
	    //Implement the code to clear the Overrun/under-run error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR); 
	}

//Check for Time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		
		//Implement the code to notify the application about the error 
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

/*********************************************************
 * @fn							- I2C_GenerateStopCondidtion
 *
 * @brief						- Sets the STOP bit of CR1 register to generate start condition on I2Cx bus
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral base address
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */
void I2C_GenerateStopCondidtion(I2C_RegDef_t *pI2Cx){
	//Set the STOP bit in CR1 register
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/*********************************************************
 * @fn							- I2C_SlaveEnableOrDisableCallbackEvents
 *
 * @brief						- functions to enable or disable I2Cx peripheral event, error, and buffer interrupts
 *
 * @param[in]					- *pI2Cx - pointer to I2Cx peripheral base address
 * @param[in]					- EnorDi - ENABLE or DISABLE
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						-
 *
 * */


void I2C_SlaveEnableOrDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//Enable Event Interrupts
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		//Enable Error Interrupts
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
		//Enable Buffer Interrupts
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
	}
	else{
		//Disable Event Interrupts
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		//Disable Error Interrupts
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
		//Disable Buffer Interrupts
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	}
}

