/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: 02-Nov-2020
 *      Author: 91883
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "stm32f446re.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

	uint8_t *pTxBuffer;		//to store tx buffer address
	uint8_t *pRxBuffer;		//to store rx buffer address
	uint8_t TxLen;			//to store Tx len
	uint8_t RxLen;			//to store rx len
	uint8_t TxState;		//to store tx state
	uint8_t RxState;		//to store rx state
}SPI_Handle_t;

//@SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER						1
#define SPI_DEVICE_MODE_SLAVE						0

//@SPI_BusConfig
#define SPI_BUS_CONFIG_FD							1		//full duplex
#define SPI_BUS_CONFIG_HD							2		//half duplex
//#define SPI_BUS_CONFIG_SIMPLEX_TXDNLY		simplex transmit only, this is done by removing MISO pin in full duplex mode
#define SPI_BUS_CONFIG_SIMPLEX_RXDNLY				3		//simplex receive only

//@SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV_2						0		//peripheral clock divided by 2
#define SPI_SCLK_SPEED_DIV_4						1		//peripheral clock divided by 4
#define SPI_SCLK_SPEED_DIV_8						2
#define SPI_SCLK_SPEED_DIV_16						3
#define SPI_SCLK_SPEED_DIV_32						4
#define SPI_SCLK_SPEED_DIV_64						5
#define SPI_SCLK_SPEED_DIV_128						6
#define SPI_SCLK_SPEED_DIV_256						7

//@SPI_DFF
#define SPI_DFF_8BITS								0
#define SPI_DFF_16BITS								1

//@SPI_CPOL
#define SPI_CPOL_LOW								0
#define SPI_CPOL_HIGH								1

//@SPI_CPHA
#define SPI_CPHA_LOW								0
#define SPI_CPHA_HIGH								1

//@SPI_SSM
#define SPI_SSM_EN									1		//software Slave select management
#define SPI_SSM_DI									0		//hardware Slave select management

//possible SPI application events
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4


//peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//init and de-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);      //interrupt mode
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);   //interrupt mode

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagbit);

//IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
//SPI peripheral control
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//application callback
void SPI_ApplicaitonEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
