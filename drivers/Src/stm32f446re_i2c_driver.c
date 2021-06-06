/*
 * stm32f446re_i2c_driver.c
 *
 *  Created on: 06-Nov-2020
 *      Author: Vignesh R
 */

#include "stm32f446re_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

//peripheral clock setup
/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
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
	if(EnorDi == DISABLE){
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

//init and de-init

/*********************************************************************
 * @fn      		  - I2C_DeInit
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
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;

	//enable clock for I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure FREQ field
	tempreg = 0;
	tempreg = RCC_GETPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_Speed_SM){
		//standard mode
		ccr_value = (RCC_GETPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		tempreg |= (ccr_value & 0x0FFF);
	}
	else{
		//fast mode
		//change F/S bit to 1
		tempreg |= (1 << 15);

		//modify DUTY bit
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY2){
			ccr_value = (RCC_GETPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else{
			ccr_value = (RCC_GETPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		tempreg |= (ccr_value & 0x0FFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_Speed_SM){
		//mode is standard mode
		tempreg = ( (RCC_GETPCLK1Value() / 1000000U) + 1);
	}
	else{
		//mode is fast mode
		tempreg = ( (RCC_GETPCLK1Value() * 300) / 1000000000U ) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/*********************************************************************
 * @fn      		  - I2C_DeInit
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

//IRQ configuration and ISR handling
/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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

//IRQ configuration and ISR handling
/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED);
	volatile uint32_t *priority_reg = NVIC_PR_BASE_ADDR + iprx;
	*priority_reg &= ~(0xFF << (8 * iprx_section) );
	*priority_reg |= (IRQPriority << shift_amount );
}

//I2C peripheral control
//IRQ configuration and ISR handling
/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
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
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pI2Cx->CR1 |= (1 << 0);
	}
	else{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx){
	pI2Cx->CR1 |= (1 << 8);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagbit){
	if(pI2Cx->SR1 & (1 << Flagbit) ){
		return SET;
	}
	return RESET;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);			//slave address + r/nw bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |=  (1);			//slave address + r/nw bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1)){
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

			if(pI2CHandle->RxSize == 1){
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag (read SR1, read SR@)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
			else{
				//clear the ADDR flag (read SR1, read SR@)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else{
			//clear the ADDR flag (read SR1, read SR@)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else{
		//device is in slave mode
		//clear the ADDR flag (read SR1, read SR@)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx){
	pI2Cx->CR1 |= (1 << 9);
}

void I2CSlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pI2Cx->CR2 |= ( 1 << 10);
		pI2Cx->CR2 |= ( 1 << 9);
		pI2Cx->CR2 |= ( 1 << 8);
	}
	else{
		pI2Cx->CR2 &= ~( 1 << 10);
		pI2Cx->CR2 &= ~( 1 << 9);
		pI2Cx->CR2 &= ~( 1 << 8);
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pI2Cx->CR1 |= (1 << 10);
	}
	else if (EnorDi == DISABLE){
		pI2Cx->CR1 &= ~(1 << 10);
	}
}

//send and receive data
//IRQ configuration and ISR handling
/*********************************************************************
 * @fn      		  - I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr){
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: until SB is cleared SCL will be stretched (pulled low)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 0));			//SR1 is read and write to DR is done in next step to clear the SB bit

	//3. send the address of the slave with r/nw bit set to w(0)  ( total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. confirm that address phase is completed by checking the ADDR flag in the SR!
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 1));

	//5. clear the ADDR flag according to its software sequence
	//	 Note: unitl ADDR is cleared SCl will be stretched
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send data until length becomes zero
	while(len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 7));		//wait until TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. when len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 7));		//wait until TXE is set
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 2));		//wait until BTF is set

	//8. Generate STOP condition and master need not to wait for the completion of the stop condition
	//	 Note: generating STOP, automatically clears the BTF
	if(Sr == 0){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

//IRQ configuration and ISR handling
/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr){
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR
	//   Note: Until SB is cleared SCL will be stretched
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 0));			//SR1 is read and write to DR is done in next step to clear the SB bit

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in the SR
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 1));

	//procedure to read only 1 byte from slave
	if(len == 1){
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 6));

		//generate STOP condition
		if(Sr == 0){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read data from slave when len > 1
	if(len >  1){
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until len becomes zero
		for(uint32_t i = len; i > 0; i++){
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, 6));

			if(i == 2){		//if last two bytes are remaining
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//generate STOP condition
				if(Sr == 0){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return (uint8_t)pI2Cx->DR;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
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
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << 10);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << 9);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << 8);

	}
	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
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
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = len; 				//Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 10);

			//Implement the code to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 9);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 8);

		}
		return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxLen > 0){
		//1. load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	//implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 10);

	//implement the code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 9);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	//implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 10);

	//implement the code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 9);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->RxSize == 1){
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	else if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen == 2){
			//clear ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}
		//read DR
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0){
		//close the I2C data reception and notify the application
		//1. generate stop condition
		if(pI2CHandle->Sr == DISABLE){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2. Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	// Interrupt handling for both master and slave mode of device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << 9);		//ITEVTEN status
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << 10);		//ITBUFEN status

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 0);		//SB status
	//1. handle for interrupt generated by SB event
	//   Note: SB flag is only applicable in Master mode
	if(temp1 && temp3){
		//interrupt is generated because of SB event
		//this block will not be executed in slave mode because SB is always zero in slave mode
		//execute address phase

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 1);	//ADDR status
	//2. Handle for interrupt generated by ADDR event
	//   Note: When master : Address is sent
	//         When Slave  : Address is matched with own address
	if(temp1 && temp3){
		//interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 2);	//BTF status
	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3){
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			//make sure TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << 7)){
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0){
					//1. generate STOP condition
					if(pI2CHandle->Sr == 0){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 4);	//STOPF status
	//4. Handle for interrupt generated by STOPF event
	//   Note: stop detection flag is applicable only in slave mode.
	//	 The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3){
		//STOPF flag is set
		//clear the STOPF (i.e. read SR1 and write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 7);	//TXE status
	//5. Handle for interrupt generated by TXE event
	if(temp1 && temp2 && temp3){
		if(pI2CHandle->pI2Cx->SR2 & (1 << 0)){		//check the MSL bit in SR2 and perform the actions only if the mode is master mode
			//TXE bit is set
			//we have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else{
			//slave mode
			//make sure that slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << 2)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 6);	//RXNE status
	//6.Handle for interrupt generated by RXNE event
	if(temp1 && temp2 && temp3){
		//RXNE bit is set
		if(pI2CHandle->pI2Cx->SR2 & (1 << 0)){			//check MSL
			//device is master
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else{
			//slave mode
			//make sure that slave is in receiver mode
			if( !(pI2CHandle->pI2Cx->SR2 & (1 << 2) ) ){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
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

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << 8);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< 8);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 8);

	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 9 );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 9);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 10);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 10);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 11);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 11);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 14);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 14);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
	//the application may override this function

}
