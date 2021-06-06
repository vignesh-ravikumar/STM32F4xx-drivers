/*
 * 002led_button.c
 *
 *  Created on: 31-Oct-2020
 *      Author: 91883
 */

#include "stm32f446re.h"

void delay(){
	for(uint32_t i=0; i<500000; i++);
}

int main(){
	//button config
	GPIO_Handle_t GpioButton;
	memset(&GpioButton, 0, sizeof(GpioButton));
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	//on-board led config
	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0, sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1){
		if(! GPIO_ReadFromInputPin(GpioButton.pGPIOx, GPIO_PIN_NO_13)){
			delay();
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		}
	}
}
