/*
 * stm32f446re_rcc_driver.h
 *
 *  Created on: 09-Nov-2020
 *      Author: 91883
 */

#ifndef INC_STM32F446RE_RCC_DRIVER_H_
#define INC_STM32F446RE_RCC_DRIVER_H_

#include "stm32f446re.h"

uint32_t RCCGetOutputClock();
uint32_t RCC_GETPCLK1Value(void);
uint32_t RCC_GETPCLK2Value(void);

#endif /* INC_STM32F446RE_RCC_DRIVER_H_ */
