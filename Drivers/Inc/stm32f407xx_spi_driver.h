
 /* 
 *
 *  Created on: Aug 3, 2023
 *      Author: geroldwilliams
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

//SPIx Configuration Structure
typedef struct{
	uint8_t SPI_DeviceMode;			/*!< possible values @SPI_DEVICE_MODE >*/
	uint8_t SPI_BusConfig; 			/*!< possible values @SPI_BUS_CONFIG >*/
	uint8_t SPI_ClockSpeed;			/*!< possible values @SPI_CLK_SPEED >*/
	uint8_t SPI_DFF;				/*!< possible values @SPI_DATA_FRAME_FORMAT >*/
	uint8_t SPI_CPOL;				/*!< possible values @SPI_CLK_POLARITY >*/
	uint8_t SPI_CPHA;				/*!< possible values @SPI_CLK_PHASE >*/
	uint8_t SPI_SSM;				/*!< possible values @SPI_SLAVE_SELECT_MGT >*/
}SPI_Config_t;

//SPIx Bus Handle structure
typedef struct{
	SPI_RegDef_t *pSPIx;		/*!< holds the base address of the SPI interface > */
	SPI_Config_t SPIConfig;		/*!< holds the SPIx interface configuration settings >*/
	uint8_t *pTxBuffer;			/*!< holds the address of TxBuffer > */
	uint8_t *pRxBuffer;			/*!< holds the address of RxBuffer >*/
	uint32_t TxLen;				/*!< holds the length of TxBuffer> */
	uint32_t RxLen;				/*!< holds the length of RxBuffer >*/
	uint8_t TxState;			/*!< holds the Tx state @SPI_APPLICATION_STATUS > */
	uint8_t RxState;			/*!< holds the Rx state @SPI_APPLICATION_STATUS >*/
}SPI_Handle_t;


/******************************************************* SPI CONFIG SETTINGS ****************************************************************/

/*
 * @SPI_DEVICE_MODE
 * SPI Modes of operation
 * */
#define SPI_DEVICE_SLAVE			0
#define SPI_DEVICE_MASTER			1
/*
 * @SPI_BUS_CONFIG
 * SPI Bus Configuration options
 * */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_S_RXONLY			3

/*
 * @SPI_CLK_SPEED
 * SPI clock speeds
 * */
#define SPI_CLK_SPEED_DIV2					0
#define SPI_CLK_SPEED_DIV4					1
#define SPI_CLK_SPEED_DIV8					2
#define SPI_CLK_SPEED_DIV16					3
#define SPI_CLK_SPEED_DIV32					4
#define SPI_CLK_SPEED_DIV64					5
#define SPI_CLK_SPEED_DIV128				6
#define SPI_CLK_SPEED_DIV256				7
/*
 * @SPI_DATA_FRAME_FORMAT
 * SPI data frame formats
 * */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1
/*
 * @SPI_CLK_POLARITY
 * SPI clock polarity
 * */
#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1
/*
 * @SPI_CLK_PHASE
 * SPI clock phase
 * */
#define SPI_CPHA_HIGH			0
#define SPI_CPHA_LOW			1
/*
 * @SPI_SLAVE_SELECT_MGT
 * SPI slave select management options
 * */
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1
/*
 * @SPI_SR_FLAGS
 * SPI SR register flags
 * */
#define SPI_SR_TXE_FLAG			( 1 << SPI_SR_TXE)
#define SPI_SR_RXE_FLAG			( 1 << SPI_SR_RXE)
#define SPI_SR_CHSID_FLAG		( 1 << SPI_SR_CHSID)
#define SPI_SR_UDR_FLAG			( 1 << SPI_SR_UDR)
#define SPI_SR_CRCERR_FLAG		( 1 << SPI_SR_CRCERR)
#define SPI_SR_MODF_FLAG		( 1 << SPI_SR_MODF)
#define SPI_SR_OVR_FLAG			( 1 << SPI_SR_OVR)
#define SPI_SR_BSY_FLAG			( 1 << SPI_SR_BSY)
#define SPI_SR_FRE_FLAG			( 1 << SPI_SR_FRE)

/*
 * @SPI_APPLICATION_STATUS
 * Possible SPI States
 * */
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

/*
 * @SPI_CALLBACK_EVENTS
 * Possible SPI Callback Events
 * */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4
/*******************************************************************************************************************************************/
/************************************************************************
 * 						Supported API's
 *
 * 				Check function definition for more info
 * **********************************************************************
 * */
//SPI Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//SPI Peripheral Clock Setup
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//SPI Data Send and Receive
void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t len);
void SPI_DataReceive(SPI_RegDef_t *pSPIx,  uint8_t* pRxBuffer, uint32_t len);
//SPI Data Send and Receive (Interrupt Based)
uint8_t SPI_DataSendIT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t len);
uint8_t SPI_DataReceiveIT(SPI_Handle_t *pSPIHandle,  uint8_t* pRxBuffer, uint32_t len);

//IRQ Configuration and ISR Handling
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

//Flag Status
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName);

void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDI);
void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDI);
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDI);
void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//Application callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
