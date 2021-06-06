/*
 * 010i2c_master_rx_testing.c
 *
 *  Created on: 07-Nov-2020
 *      Author: Vignesh R
 */

#include<stdint.h>
#include<stdio.h>
#include "stm32f446re.h"
#include<string.h>

void delay(){
	for(uint32_t i=0; i<500000; i++);
}

I2C_Handle_t I2C1Handle;

//data
uint8_t Tx_buffer[32] = "STM32 slave mode testing..";

#define SLAVE_ADDR		0x68
#define MY_Addr			SLAVE_ADDR

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

	//I2C IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2CSlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// acking is made 1 after PE = 1, because any change we make to the ack bit before
	// before making PE = 1 will have no effect
	I2C_ManageAcking(I2C1, ENABLE);

	while(1);
}

void I2C1_EV_IRQHandler(){
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(){
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicaitonEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ){
		//master wants data, slave has to send it
		if(commandCode == 0x51){
			//send the length info to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buffer) );
		}
		else if(commandCode == 0x52){
			//send the contents of Tx buffer
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buffer[Cnt++]);
		}
	}
	else if(AppEv == I2C_EV_DATA_RCV){
		//data is waiting for slave to read
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if(AppEv == I2C_ERROR_AF){
		// happens only during slave txing.
		// at this condition slave must understand that master doesn't need any more data
		commandCode = 0xFF; 		//invalidating
		Cnt = 0;
	}
	else if(AppEv == I2C_EV_STOP){
		// happens only during slave rxing
		// master has ended the I2C communication with the slave
		Cnt = 0;
	}
}
