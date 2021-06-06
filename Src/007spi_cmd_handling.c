/*
 * 007spi_cmd_handling.c
 *
 *  Created on: 04-Nov-2020
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

//command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON		1
#define LED_OFF		0

//arduino analog pins
#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

//arduino led
#define LED_PIN				9

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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

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

uint8_t SPI_verifyResponse(uint8_t ackbyte){
	if(ackbyte == 0xF5){
		return 1;
	}
	return 0;
}

int main(){
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//this function is used to initialize GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize SPI2 peripheral parameters
	SPI2Inits();

	//this function is used to initialize GPIO pin for button
	BtnGPIOInits();

	//enable SSOE
	SPI_SSOEConfig(SPI2 , ENABLE);

	while(1){
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();
		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL  <pin no(1)>   <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);


		//send dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_verifyResponse(ackbyte) ){
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);

			printf("LED control executed");
		}
		//END OF CMD_LED_CNTRL



		//COMMAND_SENSOR_READ <pin no(1)>

		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);


		//send dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_verifyResponse(ackbyte) ){
			//send arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can be ready with the data
			delay();

			//send dummy bits (1 byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			//read the ack byte received
			SPI_ReceiveData(SPI2, &analog_read, 1);

			printf("Sensor value: %d\n", analog_read);
		}

		//END of COMMAND_SENSOR_READ



		//CMD_LED_READ	<pin_no>
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_verifyResponse(ackbyte) ){
			//send arguments
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can be ready with the data
			delay();

			//send dummy bits (1 byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t digital_read;
			//read the ack byte received
			SPI_ReceiveData(SPI2, &digital_read, 1);

			printf("LED status: %d\n", digital_read);
		}

		//wait until SPI is not busy
		while( SPI_GetFlagStatus(SPI2, 7) );

		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}
