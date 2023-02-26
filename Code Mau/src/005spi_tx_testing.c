/*
 * 005spi_tx_testing.c
 *
 *  Created on: Nov 13, 2019
 *      Author: pigmo
 */

#include "stm32f407xx.h"


/*
 * PB15 --> SPI2_MOSI
 * PB14 --> SPI2_MISO
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT Function mode: 5
 */

void GPIO_Setup(void);
void SPI_Setup(void);

int main(void) {

	char userData[] = "Hello world";

	//This function is used to initialize the gpio pins to behave as spi pins
	GPIO_Setup();

	//This function is used to initialize the SPI peripheral
	SPI_Setup();

	//enable the SPI2 peripheral --- according to generated code, this seems to be done in the transmit function
	// I a SET_BIT macro in the transmit function if SPI isn't enabled, make sure to test this
	//SPI_PeripheralControl(SPI2, ENABLE);

	//send data
	SPI_SendData(SPI2, (uint8_t*) userData, sizeof(userData));//use strlen?

	//confirm the SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	//disable the SPI2 peripheral --- according to Generated code, this seems to be done in abort functions
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}

void GPIO_Setup(void) {
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t SPIpins;

	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIpins);


	//MISO
//	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(SPIpins);

	//SCLK
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIpins);

	//NSS
//	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
//	GPIO_Init(SPIpins);
}

void SPI_Setup(void){
	SPI_PeriClockControl(SPI2,ENABLE);

	SPI_Handle_t hSPI;

	hSPI.SPIx = SPI2;
	hSPI.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	//in genearted code, this value is or'd with SSI bit
	hSPI.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	hSPI.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //max speed - 8 MHz
	hSPI.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	hSPI.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	hSPI.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	hSPI.SPIConfig.SPI_SSM = SPI_SSM_EN; //NSS not required

	SPI_Init(&hSPI);
}
