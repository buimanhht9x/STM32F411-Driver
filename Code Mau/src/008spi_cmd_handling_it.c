/*
 * 008spi_cmd_handling_it.c
 *
 *  Created on: Apr 8, 2020
 *      Author: Adam
 */

//NOTE: This is code provided by the class and there is no explination
//It is unclear what code this is supposed to interract with
//At this point, I am also not totally sure how this works
//I also feel that the use of global variables here is probably bad practice, but I'm not sure
// this appears only to receive

//#include <stdio.h>
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


#define NACK 0xA5
#define ACK 0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

#define LED_PIN	9

SPI_Handle_t SPI2handle;

uint8_t rcvBuff[100];

uint8_t readByte;

uint8_t rxContFlag = RESET;

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}


uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == ACK) {
		return 1;
	} else
	{
		return 0;
	}
}

int main(void) {



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

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);
	while(1){
		//wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000); //used for button debouncing

		//enable the SPI2 peripheral (this isn't needed because the send and receive functions do this)
		SPI_PeripheralControl(SPI2, ENABLE);

		rxContFlag = SET;

		while(rxContFlag == SET){
			while( SPI_ReceiveDataIT(&SPI2handle, &readByte,1) != SPI_READY );
		}

		//confirm that SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

void SPI2_IRQHandler(void){
	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *hSPI, uint8_t appEvent){
	static uint32_t i = 0;
	static uint8_t rcv_start = 0;
	if(appEvent == SPI_EVENT_RX_CMPLT) {
		if(readByte == 0xF1){
			rcv_start = 1;
		} else {
			if(rcv_start){
				if(readByte == '\r'){
					rxContFlag = RESET;
					rcv_start = 0;
					rcvBuff[i++] = readByte;
					i=0;
				} else {
					rcvBuff[i++] = readByte;
				}
			}
		}
	}
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
//	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
//	GPIO_Init(&SPIpins);


	//MISO
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIpins);

	//SCLK
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIpins);

	//NSS
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIpins);


}
void GPIO_ButtonInit(void){
	GPIO_Handle_t GpioBtn, GpioLed;

	// configure button
	//Set up PA0 to be an input for the button press
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);
}
void SPI_Setup(void){

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_SIMPLEX_RXONLY;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generate SCLK of 2 MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management disabled for NSS pin

	SPI_Init(&SPI2handle);
}


