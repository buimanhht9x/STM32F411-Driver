/*
 * 006spi_tx_only_arduino.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Adam
 */

#include <string.h>
#include "stm32f407xx.h"


/*
 * PB15 --> SPI2_MOSI
 * PB14 --> SPI2_MISO
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT Function mode: 5
 */

void SPI_GPIO_Setup(void);
void GPIO_ButtonInit(void);
void SPI_Setup(void);

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}

int main(void) {

	char userData[] = "Hello, I am robot.";

	//initizalize button
	GPIO_ButtonInit();

	//This function is used to initialize the gpio pins to behave as spi pins
	SPI_GPIO_Setup();

	//This function is used to initialize the SPI peripheral
	SPI_Setup();

	//this makes NSS signal internally high and avoids MODF error
	//NOTE: This should be taken care of in my SPI_Init function
//	SPI_SSIConfig(SPI2, ENABLE);

	//SSOE to 1 does NSS output enable
	//I don't want to enable this in this way, but it is more simple due to the code design
	//i.e. (using shifts instead of masks for registers)
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		//wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000); //used for button debouncing

		//enable the SPI2 peripheral --- according to generated code, this seems to be done in the transmit function
		// I a SET_BIT macro in the transmit function if SPI isn't enabled, make sure to test this
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send the length of data (Arduino expects this as 1 byte)
		uint8_t dataLen = strlen(userData);
		SPI_SendData(SPI2, &dataLen, 1);

		//send data
		SPI_SendData(SPI2, (uint8_t*)userData, dataLen);

		//confirm the SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		//disable the SPI2 peripheral --- according to Generated code, this seems to be done in abort functions
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

void SPI_GPIO_Setup(void) {

	GPIO_Handle_t SPIpins;

	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIpins);


	//MISO
//	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(&SPIpins);

	//SCLK
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIpins);

	//NSS
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIpins);


}
void GPIO_ButtonInit(void){
	GPIO_Handle_t GpioBtn;

	// configure button
	//Set up PA0 to be an input for the button press
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn);
}
void SPI_Setup(void){
	SPI_Handle_t hSPI;

	hSPI.SPIx = SPI2;
	hSPI.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	hSPI.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	hSPI.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generate SCLK of 2 MHz
	hSPI.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	hSPI.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	hSPI.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	hSPI.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management disabled for NSS pin

	SPI_Init(&hSPI);
}
