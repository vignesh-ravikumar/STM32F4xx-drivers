/*
 * 009i2c_master_tx_testing.c
 *
 *  Created on: 07-Nov-2020
 *      Author: Vignesh R
 */

#include<stdint.h>
#include "stm32f446re.h"
#include<string.h>

void delay(){
	for(uint32_t i=0; i<500000; i++);
}

I2C_Handle_t I2C1Handle;

//data
uint8_t data[] = "Hope it works\n";

#define MY_Addr			0x61
#define SLAVE_ADDR		0x68

/*
 * PB6 -  I2C1_SCL
 * PB9 -  I2C1_SDA
 * ALT Function mode 4
 */

void I2C1_GPIOInits(){
	GPIO_Handle_t I2CPins;
	memset(&I2CPins, 0, sizeof(I2CPins));

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&I2CPins);
}

void I2C1Inits(){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_Addr;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_Speed_SM;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY2;    //this value does not matter in SM

	I2C_Init(&I2C1Handle);
}

void BtnGPIOInits(){
	//button config
	GPIO_Handle_t GpioButton;
	memset(&GpioButton, 0, sizeof(GpioButton));
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
	GPIO_Init(&GpioButton);
}


int main(){

	BtnGPIOInits();

	//i2c pin inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1){
		//wait for buttin press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		//send data
		I2C_MasterSendData(&I2C1Handle, data, strlen((char*)data), SLAVE_ADDR);

	}
}
