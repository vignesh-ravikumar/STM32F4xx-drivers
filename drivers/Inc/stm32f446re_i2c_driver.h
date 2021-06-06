/*
 * stm32f446re_i2c_driver.h
 *
 *  Created on: 06-Nov-2020
 *      Author: 91883
 */

#ifndef INC_STM32F446RE_I2C_DRIVER_H_
#define INC_STM32F446RE_I2C_DRIVER_H_

#include "stm32f446re.h"

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;		//to store communication state
	uint8_t DevAddr;		//slave address
	uint32_t RxSize;
	uint8_t Sr;
}I2C_Handle_t;

//I2C application states
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

//@I2C_SCLSpeed
#define	I2C_SCL_Speed_SM		100000		//standard mode speed 100K Hz
#define I2C_SCL_Speed_FM4K		400000		//fast mode speed 400K Hz
#define I2C_SCL_Speed_FM2K		200000

//@I2C_ACKControl
#define I2C_ACK_EN				1
#define I2C_ACK_DI				0

//@I2C_FMDutyCycle
#define I2C_FM_DUTY2			0
#define I2C_FM_DUTY16_9			1

//I2C application event macros
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2

//I2C error macros
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF   		 	5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7

#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RCV			9

//api's

//peripheral clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//init and de-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagbit);

//send and receive data
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

//IRQ configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

//I2C peripheral control
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
void I2CSlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F446RE_I2C_DRIVER_H_ */
