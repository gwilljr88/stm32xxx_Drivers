/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Oct 19, 2023
 *      Author: geroldwilliams
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

//USARTx Application Configuration Structure
typedef struct{
	uint8_t  USART_Mode;			/**< see:USART_MODE */
	uint32_t USART_Baud;			/**< see:USART_BAUD_RATE */
	uint8_t  USART_NumOfStopBits;	/**< see:USART_NO_OF_STOPBITS */
	uint8_t  USART_WordLength;		/**< see:USART_WORD_LENGTH */
	uint8_t  USART_ParityControl;	/**< see:USART_PARITY_CONTROL */
	uint8_t  USART_HWFlowControl;	/**< see:USART_HARDWARE_FLOW_CONTROL */
}USART_Config_t;

//USARTx Handle Structure
typedef struct{
	USART_RegDef_t *pUSARTx;		/**< pointer to USARTx peripheral registers */
	USART_Config_t USART_Config;	/**< Application configuration structure */
	uint8_t TxBusyState;			/**< Application communication status see:USART_APPLICATION_STATUS */
	uint8_t *pTxBuffer;				/**< pointer the pTxBuffer, hold application transmit data */
	uint32_t TxLen;					/**< length of data stored in TxBuffer */
	uint8_t RxBusyState;			/**< Application communication status see:USART_APPLICATION_STATUS */
	uint8_t *pRxBuffer;				/**< pointer to pRxBuffer, holds received data */
	uint32_t RxLen;					/**< length of data stored in RxBuffer */
}USART_Handle_t;
/*
 * USART_MODE
 * TE and RE bits in CR1 register
 * */
#define USART_MODE_ONLY_TX			0
#define USART_MODE_ONLY_RX			1
#define USART_MODE_TXRX				2
/*
 * USART_BAUD_RATE
 * BRR register
 * */
#define USART_STD_BAUD_1920 		1200
#define USART_STD_BAUD_2400 		2400
#define USART_STD_BAUD_9600 		9600
#define USART_STD_BAUD_19200 		19200
#define USART_STD_BAUD_38400 		38400
#define USART_STD_BAUD_57600 		57600
#define USART_STD_BAUD_115200 		115200
#define USART_STD_BAUD_230400 		230400
#define USART_STD_BAUD_460800 		460800
#define USART_STD_BAUD_921600 		921600
#define USART_STD_BAUD_2M 			2000000
#define USART_STD_BAUD_3M 			3000000
/*
 * USART_PARITY_CONTROL
 * PCE and PS bit in CR1 register
 * */
#define USART_PARITY_EN_ODD			2
#define USART_PARITY_EN_EVEN		1
#define USART_PARITY_DISABLE		0
/*
 * USART_WORD_LENGTH
 * M bit in CR1 register
 * */
#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1
/*
 * USART_NO_OF_STOPBITS
 * STOP bit in CR2 register
 * */
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_1			0
#define USART_STOPBITS_1_5			3
#define USART_STOPBITS_2			2
/*
 * USART_HARDWARE_FLOW_CONTROL
 * CSTE an RSTE bit in CR3 register
 * */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3
/*
 * USART_SR_FLAGS
 * USART SR register flags
 * */
#define USART_FLAG_SR_PE		( 1 << USART_SR_PE	 )
#define USART_FLAG_SR_FE 		( 1 << USART_SR_FE   )
#define USART_FLAG_SR_NF 		( 1 << USART_SR_NF   )
#define USART_FLAG_SR_ORE		( 1 << USART_SR_ORE  )
#define USART_FLAG_SR_IDLE		( 1 << USART_SR_IDLE )
#define USART_FLAG_SR_RXNE		( 1 << USART_SR_RXNE )
#define USART_FLAG_SR_TC 		( 1 << USART_SR_TC   )
#define USART_FLAG_SR_TXE		( 1 << USART_SR_TXE  )
#define USART_FLAG_SR_LBD		( 1 << USART_SR_LBD  )
#define USART_FLAG_SR_CTS		( 1 << USART_SR_CTS  )
/*
 * USART_COMMUNICATION_STATUS
 * USART communication status
 * */
#define USART_READY					0 
#define USART_RX_READY				1 
#define USART_BUSY_IN_TX 			2
#define USART_BUSY_IN_RX 			3
/*
 * USART_APPLICATION_CALLBACK_STATUS
 * USART application callback status
 * */
#define USART_EVENT_TX_CMPLT 		0
#define USART_EVENT_RX_CMPLT 		1
#define USART_EVENT_CTS				2
#define USART_EVENT_IDLE			3
#define USART_EVENT_ORE				4
#define USART_ERREVENT_FE			5
#define USART_ERREVENT_NE			6
#define USART_ERREVENT_ORE			7

/************************************************************************
 * 						Supported API's
 *
 * 				Check function definition for more info
 * **********************************************************************
 * */
//USART Init and De-init
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

//USART Peripheral Clock Setup
void USART_PeriClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

//USART Data Send and Receive
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

//USART Data Send and Receive (Interrupt Based)
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);


//USART Configuration and ISR Handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

//Helper Functions
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t FlagName);
void USART_ClearFlagStatus(USART_RegDef_t* pUSARTx, uint32_t FlagName);
void USART_PeripheralControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_ClearErrorFlag(USART_Handle_t *pUSARTHandle);

//USART Interrupt Handler
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

//Application callback
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);



#endif /* INC_STM32F407XX_USART_DRIVER_H_ */