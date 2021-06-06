/*
 * 001ledtoggle.c
 *
 *  Created on: 31-Oct-2020
 *      Author: Vignesh R
 */
#include "stm32f446re.h"

void delay(){
	for(uint32_t i=0; i<500000; i++);
}

int main(){
	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0, sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;		//for push pull use GPIO_OP_TYPE_PP
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	while(1){
		delay();
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
	}
}
