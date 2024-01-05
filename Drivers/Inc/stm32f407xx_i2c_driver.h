/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Sep 7, 2023
 *      Author: geroldwilliams
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

//I2Cx Configuration Structure
typedef struct{
	uint32_t I2C_SCLSpeed;				/*!< possible values @I2C_SCL_SPEED >*/
	uint8_t  I2C_DeviceAddress;			/*!< defined by user >*/
	uint8_t  I2C_ACKControl;			/*!< possible values @I2C_ACK_CONTROL >*/
	uint8_t I2C_FMDutyCycle;			/*!< possible values @I2C_FM_DUTY_CYCLE >*/
}I2C_Config_t;

//I2Cx Handle Structure
typedef struct{
	I2C_RegDef_t *pI2Cx;				/*!< pointer to I2Cx peripheral register structure >*/
	I2C_Config_t I2C_Config;			/*!< holds the application info for I2Cx peripheral configuration >*/
	uint8_t TxRxState;					/*!< holds the communication state of the I2Cx peripheral @I2C_APPLICATION_STATE>*/
	uint8_t *pTxBuffer;					/*!< holds the address of the application Tx buffer >*/
	uint8_t *pRxBuffer;					/*!< holds the address of the application Rx buffer >*/
	uint32_t TxLen;						/*!< holds the size of the application Tx buffer  >*/
	uint32_t RxLen;						/*!< holds the size of the application Tx buffer >*/
	uint8_t DevAddr;					/*!< holds the address of slave device >*/
	uint32_t RxSize;					/*!< holds the size of Rx data >*/
	uint8_t Sr;							/*!< holds the Start repeat flag @I2C_START_REPEAT>*/
}I2C_Handle_t;

/******************************************************* SPI CONFIG SETTINGS ****************************************************************/
/*
 * @I2C_SCL_SPEED
 */
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM2K			200000
#define I2C_SCL_SPEED_FM4K			400000

/*
 * @I2C_ACK_CONTROL
 */
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

/*
 * @I2C_FM_DUTY_CYCLE
 */
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1
/*
 * @I2C_START_REPEAT
 */
#define I2C_EN_SR					SET
#define I2C_DI_SR 					RESET
/*
 * @I2C_APPLICATION_STATE
 */
#define I2C_READY					0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2
/*
 * @I2C_CALLBACK_EVENTS
 * */
#define I2C_EVENT_TX_CMPLT			1
#define I2C_EVENT_RX_CMPLT			2
#define I2C_EVENT_STOPF				3
#define I2C_EVENT_DATA_REQ			4
#define I2C_EVENT_DATA_RCV			5
/*
 * @I2C_CALLBACK_ERRORS
 * */
#define I2C_ERROR_BERR  			6
#define I2C_ERROR_ARLO 				7
#define I2C_ERROR_AF    			8
#define I2C_ERROR_OVR   			9
#define I2C_ERROR_TIMEOUT 			10
/********************************************************************************************************************************************/


/************************************************************************
 * 						Supported API's
 *
 * 				Check function definition for more info
 * **********************************************************************
 * */
//I2C Init and De-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//I2C Peripheral Clock Setup
void I2C_PeriClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//I2C Master Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t slaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr);

//I2C Data Send and Receive (Interrupt Based)
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t slaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr);

//I2C Slave Data Send and Receive
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

//I2C Configuration and ISR Handling
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

//Flag Status
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName);

void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDI);
void I2C_AckContol(I2C_RegDef_t* pI2Cx, uint8_t EnorDI);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStopCondidtion(I2C_RegDef_t *pI2Cx);

//Application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
void I2C_SlaveEnableOrDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * @I2C_SR_FLAGS
 * I2C SR1 register flags
 * */
#define I2C_FLAG_SR1_SB					( 1 << I2C_SR1_SB )
#define I2C_FLAG_SR1_ADDR				( 1 << I2C_SR1_ADDR )
#define I2C_FLAG_SR1_BTF				( 1 << I2C_SR1_BTF )
#define I2C_FLAG_SR1_STOPF				( 1 << I2C_SR1_STOPF )
#define I2C_FLAG_SR1_RXNE				( 1 << I2C_SR1_RXNE )
#define I2C_FLAG_SR1_TXE				( 1 << I2C_SR1_TXE )
#define I2C_FLAG_SR1_BERR				( 1 << I2C_SR1_BERR )
#define I2C_FLAG_SR1_ARLO				( 1 << I2C_SR1_ARLO )
#define I2C_FLAG_SR1_AF					( 1 << I2C_SR1_AF )
#define I2C_FLAG_SR1_OVR				( 1 << I2C_SR_OVR )
#define I2C_FLAG_SR1_PECERR				( 1 << I2C_SR1_PECERR )
#define I2C_FLAG_SR1_TIMEOUT			( 1 << I2C_SR1_TIMEOUT )
#define I2C_FLAG_SR1_SMBALERT			( 1 << I2C_SR1_SMBALERT )



#endif /* INC_STM32F4XX_I2C_DRIVER_H_ */
