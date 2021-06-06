/*
 * 005spi_txponly_arduino.c
 *
 *  Created on: 03-Nov-2020
 *      Author: Vignesh R
 */

#include<stdint.h>
#include<string.h>

/*
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCK
 * ALT Function mode 5
 */

#include "stm32f446re.h"

void delay(){
	for(uint32_t i=0; i<500000; i++);
}

void SPI2_GPIOInits(){
	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins));
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		//for SPI output type must be push pull
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2Inits(){
	SPI_Handle_t SPI2handle;
	memset(&SPI2handle, 0, sizeof(SPI2handle));

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV_8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;							//software slave management disabled for NSS pin
	SPI_Init(&SPI2handle);
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
	char user_data[] = "Hello World";		//this is the tx buffer

	//this function is used to initialize GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize SPI2 peripheral parameters
	SPI2Inits();

	//this function is used to initialize GPIO pin for button
	BtnGPIOInits();

	//enable SSOE
	SPI_SSOEConfig(SPI2 , ENABLE);

	while(1){
		if(~GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)){
			delay();
			//enable SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			//send length of data
			uint8_t datalen = strlen(user_data);
			SPI_SendData(SPI2, &datalen, 1);

			//wait until SPI is not busy
			while( SPI_GetFlagStatus(SPI2, 7) );


			//send actual data
			SPI_SendData(SPI2, (uint8_t*)user_data, datalen);

			//wait until SPI is not busy
			while( SPI_GetFlagStatus(SPI2, 7) );

			//enable SPI2 peripheral
			SPI_PeripheralControl(SPI2, DISABLE);
		}
	}

	return 0;
}
