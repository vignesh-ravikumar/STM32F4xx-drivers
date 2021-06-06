/*
 * 004button_interrupt.c
 *
 *  Created on: 01-Nov-2020
 *      Author: Vignesh R
 */

#include<string.h>
#include "stm32f446re.h"

void delay(){
	for(uint32_t i=0; i<500000/2; i++);
}
int main(){
	//external button config
	GPIO_Handle_t GpioButton;
	memset(&GpioButton, 0, sizeof(GpioButton));		//sets all the values of structure to zero
	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//external led config
	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0, sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	//IRQ configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15);

	while(1);
	return 0;
}

void EXTI9_5_IRQHandler(void){
	delay();		//200ms
	GPIO_IRQHandling(GPIO_PIN_NO_5);		//clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}

