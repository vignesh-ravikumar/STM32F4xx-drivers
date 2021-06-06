/*
 * sstm32f446re_spi_driver.c
 *
 *  Created on: 02-Nov-2020
 *      Author: 91883
 */


//peripheral clock setup

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
#include "stm32f446re_spi_driver.h"

//some helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

//SPI application states
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx== SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx== SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx== SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx== SPI4){
			SPI4_PCLK_EN();
		}
	}
	else{
		if(pSPIx== SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx== SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx== SPI3){
			SPI3_PCLK_DI();
		}
		else if(pSPIx== SPI4){
			SPI4_PCLK_DI();
		}
	}
}

//init and de-init

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){
	//enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//config SPI CR1 register
	uint32_t tempreg = 0;

	//config device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);

	//config bus mode
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_BUS_CONFIG_FD){
		//BIDI mode should be cleared
		tempreg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_BUS_CONFIG_HD){
		//BIDI mode should be set
		tempreg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_BUS_CONFIG_SIMPLEX_RXDNLY){
		//BIDI mode should be cleared
		tempreg &= ~(1 << 15);
		//RXONLY must be set
		tempreg |= (1 << 10);
	}

	//config SPI serial clock speed
	//change values of BR[2:0]
	if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_2){
		tempreg &= ~(1 << 3);
		tempreg &= ~(1 << 4);
		tempreg &= ~(1 << 5);
	}
	else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_4){
		tempreg |=  (1 << 3);
		tempreg &= ~(1 << 4);
		tempreg &= ~(1 << 5);
	}
	else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_8){
		tempreg &= ~(1 << 3);
		tempreg |=  (1 << 4);
		tempreg &= ~(1 << 5);
	}
	else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_16){
		tempreg |=  (1 << 3);
		tempreg |=  (1 << 4);
		tempreg &= ~(1 << 5);
	}
	else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_32){
		tempreg &= ~(1 << 3);
		tempreg &= ~(1 << 4);
		tempreg |=  (1 << 5);
	}
	else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_64){
		tempreg |=  (1 << 3);
		tempreg &= ~(1 << 4);
		tempreg |=  (1 << 5);
	}
	else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_128){
		tempreg &= ~(1 << 3);
		tempreg |=  (1 << 4);
		tempreg |=  (1 << 5);
	}
	else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV_256){
		tempreg |=  (1 << 3);
		tempreg |=  (1 << 4);
		tempreg |=  (1 << 5);
	}

	//configure data frame format
	if(pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_8BITS){
		tempreg &= ~(1 << 11);
	}
	else if(pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_16BITS){
		tempreg |=  (1<<11);
	}

	//config CPOL
	if(pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_LOW){
		tempreg &= ~(1 << 1);
	}
	else if(pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_HIGH){
		tempreg |=  (1 << 1);
	}

	//congif CPHA
	if(pSPIHandle->SPIConfig.SPI_CPHA == SPI_CPHA_LOW){
		tempreg &= ~(1 << 0);
	}
	else if(pSPIHandle->SPIConfig.SPI_CPHA == SPI_CPHA_HIGH){
		tempreg |=  (1 << 0);
	}

	//config slave select management
	if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_DI){
		tempreg &= ~(1 << 9);
	}
	else if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN){
		tempreg |=  (1 << 9);
	}

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx== SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx== SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx== SPI3){
		SPI3_REG_RESET();
	}
	else if(pSPIx== SPI4){
		SPI4_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagbit){
	if(pSPIx->SR & (1 << Flagbit) ){
		return SET;
	}
	return RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -	This is blocking call, non-interrupt based or polling based
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	while(len>0){
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, 1) == 0);

		//2. check the DFF
		if(pSPIx->CR1 & (1 << 11)){				//DFF bit is at position 11 of CR1 register
			//16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len -= 2;
			(uint16_t*)pTxBuffer++;				//increment 16 bit address so the value is incremented by two
		}
		else{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -	This is blocking call, non-interrupt based or polling based
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	while(len > 0){
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, 0) == 0);

		//2. check the DFF
		if(pSPIx->CR1 & (1 << 11)){				//DFF bit is at position 11 of CR1 register
			//16 bit DFF
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len -= 2;
			(uint16_t*)pRxBuffer++;				//increment 16 bit address so the value is incremented by two
		}
		else{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << 8);
	}
	else if(EnorDi == DISABLE){
		pSPIx->CR1 &= ~(1 << 8);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR2 |= (1 << 2);
	}
	else if(EnorDi == DISABLE){
		pSPIx->CR2 &= ~(1 << 2);
	}
}

//SPI peripheral control
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << 6);
	}
	else if(EnorDi == DISABLE){
		pSPIx->CR1 &= ~(1 << 6);
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			//configure ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if( (IRQNumber>31)  & (IRQNumber <=63) ){
			//configure ISER1 register
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
		}
		else if( (IRQNumber>63) & (IRQNumber <= 95) ){
			//configure ISER2 register
			*NVIC_ISER2 |= (1<<(IRQNumber%64));
		}
		else if(IRQNumber == 96){
			//configure ISER3 register
			*NVIC_ISER3 |= (1<<(IRQNumber%96));
		}
	}
	else{
		if(IRQNumber <= 31){
			//configure ICER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if( (IRQNumber>31) & (IRQNumber <=63) ){
			//configure ICER1 register
			*NVIC_ICER1 |= (1<<(IRQNumber%32));
		}
		else if( (IRQNumber>63) & (IRQNumber <= 95) ){
			//configure ICER2 register
			*NVIC_ICER2 |= (1<<(IRQNumber%64));
		}
		else if(IRQNumber == 96){
			//configure ICER3 register
			*NVIC_ICER3 |= (1<<(IRQNumber%96));
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED);
	volatile uint32_t *priority_reg = NVIC_PR_BASE_ADDR + iprx;
	*priority_reg &= ~(0xFF << (8 * iprx_section) );
	*priority_reg |= (IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		//1. save the tx buffer address and len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2. mark the spi status as busy in transmission so that
		//   no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever the flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << 7);

		//4. Data transmission will be handled by the ISR code
	}
	return state;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		//1. save the rx buffer address and len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		//2. mark the spi status as busy in reception so that
		//   no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever the flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << 6);

		//4. Data reception will be handled by the ISR code
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1,temp2;

	//check for TXE and TXEIE
	temp1 = pSPIHandle->pSPIx->SR & (1 << 1);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << 7);

	if(temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for RXNE and RXNEIE
	temp1 = pSPIHandle->pSPIx->SR & (1 << 0);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << 6);

	if(temp1 && temp2){
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//check for overrun(OVR) error and ERRIR
	temp1 = pSPIHandle->pSPIx->SR & (1 << 6);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << 5);

	if(temp1 && temp2){
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

//some helper functions implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF
	if(pSPIHandle->pSPIx->CR1 & (1 << 11)){				//DFF bit is at position 11 of CR1 register
		//16 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;				//increment 16 bit address so the value is incremented by two
	}
	else{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen){
		//close the spi transmission
		//tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicaitonEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF
	if(pSPIHandle->pSPIx->CR1 & (1 << 11)){				//DFF bit is at position 11 of CR1 register
		//16 bit DFF
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;				//increment 16 bit address so the value is incremented by two
	}
	else{
		//8 bit DFF
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen){
		//close the spi transmission
		//rx is over
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicaitonEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//1. clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//2. inform the application
	SPI_ApplicaitonEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << 7);	//this prevents interrupts from setting up of TXE flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << 6);	//this prevents interrupts from setting up of RXNE flag
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicaitonEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	//the application may override this function

}
