/*
 * stm32f446re_gpio_driver.c
 *
 *  Created on: 30-Oct-2020
 *      Author: Vignesh R
 */

#include "stm32f446re_gpio_driver.h"

//return portcode for given base address
/*********************************************************************
 * @fn      		  - GPIO_BASEADDR_TO_CODE
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  port code
 *
 * @Note              -  none
 */

uint8_t GPIO_BASEADDR_TO_CODE (GPIO_RegDef_t *x){
	  if(x==GPIOA){
		  return 1;
	  }
	  else if(x==GPIOB){
		  return 2;
	  }
	  else if(x==GPIOC){
	  	  return 3;
	  }
	  else if(x==GPIOD){
	  	  return 4;
	  }
	  else if(x==GPIOE){
	  	  return 5;
	  }
	  else if(x==GPIOF){
		  return 6;
	  }
	  else if(x==GPIOG){
		  return 7;
	  }
	  else{
		  return 0;
	  }
}

//peripheral clock setup

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}

//init and de-init

/*********************************************************************
 * @fn      		  - GPIO_Init
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

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	//enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode
	uint32_t temp = 0;
	if((pGPIOHandle->GPIO_PinConfig).GPIO_PinMode <= GPIO_MODE_ANALOG){
		//non-interrupt node

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);						//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;																		//setting

	}
	else{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1. configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4)*4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << temp2;

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	//2. configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp  = 0;
	//3. configure the PuPd settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. configure the output type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//5. configure alternate functionality mode

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure alternate function registers
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8) ) );
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <=7){
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8) ) );
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
		}
		else{
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8) ) );
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
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

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx ==GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx ==GPIOB){
		GPIOB_REG_RESET();
	}
	if(pGPIOx ==GPIOC){
		GPIOC_REG_RESET();
	}
	if(pGPIOx ==GPIOD){
		GPIOD_REG_RESET();
	}
	if(pGPIOx ==GPIOE){
		GPIOE_REG_RESET();
	}
	if(pGPIOx ==GPIOF){
		GPIOF_REG_RESET();
	}
	if(pGPIOx ==GPIOG){
		GPIOG_REG_RESET();
	}
	if(pGPIOx ==GPIOH){
		GPIOH_REG_RESET();
	}
}

//data read and write

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t	value;
	value = (uint8_t)( (pGPIOx->IDR  >>  PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
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

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
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

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		//write 1
		pGPIOx->ODR |= (0x1 << PinNumber);
	}
	else{
		//write 0
		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
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

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
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

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}

//IRQ configuration and ISR handling

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
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

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
 * @fn      		  - GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED);
	volatile uint32_t *priority_reg = NVIC_PR_BASE_ADDR + iprx;
	*priority_reg &= ~(0xFF << (8 * iprx_section) );
	*priority_reg |= (IRQPriority << shift_amount );
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

void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the EXTI PR register corresponding to pin number
	if(EXTI->PR & (1<<PinNumber)){
		EXTI->PR |= (1<<PinNumber);
	}
}
