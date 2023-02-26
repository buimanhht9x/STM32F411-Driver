/*
 * 008spi_cmd_handling_it_alternate.c
 *
 *  Created on: Apr 8, 2020
 *      Author: Adam
 */


//NOTE: This was based off of https://github.com/nemanjadjekic/stm32f446xx_drivers/blob/master/Src/Tests/009_SPI_TxRx_Arduino_IRQ.c
//though it is basically another project adapted to use interrupts


#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


/*
 * Alternate functionality
 *
 * PB15 --> SPI2_MOSI
 * PB14 --> SPI2_MISO
 * PB13 --> SPI2_SLCK
 * PB12 --> SPI2_NSS
 * ALT function mode: AF5
 */

void SPI_GPIO_Setup(void);
void GPIO_ButtonInit(void);
void SPI2_Inits(void);

#define NACK 0xA5
#define ACK 0xF5

/* Command codes */
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

/* Analog pins */
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

/* Arduino LED */
#define LED_PIN					9


void delay(int time)
{
	for(uint32_t i = 0; i < time; i++);
}


uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == ACK)
	{
		return 1;
	}

	return 0;
}

SPI_Handle_t SPI2Handle;

uint8_t dummyWrite = 0xFF;
uint8_t dummyRead;

int main(void)
{
	/* Initialize button */
	GPIO_ButtonInit();

	/* Initialize GPIO pins to behave as SPI2 pins */
	SPI_GPIO_Setup();

	/* Initialize SPI2 peripheral parameters */
	SPI2_Inits();

	/* SPI2 IRQ configurations */
	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		/* Wait till button is pressed */
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );

//		printf("SPI communication started!\n");

		/* 200ms delay */
		delay(250000);

		/* Enable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, ENABLE);

		/* Send SPI data: CMD LED Control */
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;

		/* Send command */
		while(SPI_SendDataIT(&SPI2Handle, &commandCode, 1) != SPI_READY);

		/* Dummy read to clear off the RXNE */
		while(SPI_ReceiveDataIT(&SPI2Handle, &dummyRead, 1) != SPI_READY);

		/* Send dummy bits (byte) to fetch the response from slave */
		while(SPI_SendDataIT(&SPI2Handle, &dummyWrite, 1) != SPI_READY);

		/* Receive Acknowledgment byte */

		while(SPI_ReceiveDataIT(&SPI2Handle, &ackByte, 1) != SPI_READY);

		/* Verify response from SPI slave */
		uint8_t args[2];
		if( SPI_VerifyResponse(ackByte) )
		{
			/* Send arguments */
			args[0] = LED_PIN;
			args[1] = LED_ON;
			while(SPI_SendDataIT(&SPI2Handle, args, 2) != SPI_READY);
			/* Dummy read to clear off the RXNE */
			while(SPI_ReceiveDataIT(&SPI2Handle, &dummyRead, 1) != SPI_READY);
//			printf("Command LED executed!\n");
		}
		/* End of CMD LED Control */

		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );

		delay(250000);

		/* Send SPI data: CMD Sensor Read */
		commandCode = COMMAND_SENSOR_READ;

		/* Send command */
		while(SPI_SendDataIT(&SPI2Handle, &commandCode, 1) != SPI_READY);

		/* Dummy read to clear off the RXNE */
		while(SPI_ReceiveDataIT(&SPI2Handle, &dummyRead, 1) != SPI_READY);

		/* Send dummy bits (byte) to fetch the response from slave */
		while(SPI_SendDataIT(&SPI2Handle, &dummyWrite, 1) != SPI_READY);

		/* Receive Acknowledgment byte */
		while(SPI_ReceiveDataIT(&SPI2Handle, &ackByte, 1) != SPI_READY);

		/* Verify response from SPI slave */
		uint8_t analogRead;
		if( SPI_VerifyResponse(ackByte) )
		{
			/* Send arguments */
			args[0] = ANALOG_PIN0;
			while(SPI_SendDataIT(&SPI2Handle, args, 1) != SPI_READY);
//			printf("Command Sensor Read executed!\n");

			/* Dummy read to clear off the RXNE */
			while(SPI_ReceiveDataIT(&SPI2Handle, &dummyRead, 1) != SPI_READY);

			/* Wait for Slave to be ready with data */
			delay(25000);

			/* Send dummy bits (byte) to fetch the response from slave */
			while(SPI_SendDataIT(&SPI2Handle, &dummyWrite, 1) != SPI_READY);
			while(SPI_ReceiveDataIT(&SPI2Handle, &analogRead, 1) != SPI_READY);
			/* Dummy read to clear off the RXNE */
		}
		/* End of CMD Sensor Read */

		/* Confirm SPI2 not busy */
		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

		/* Disable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, DISABLE);

		//TODO: Implement rest of commands to test if needed
//		printf("SPI communication closed!\n");
	}

	return 0;
}
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	if(AppEv == SPI_EVENT_TX_CMPLT)
	{
//		printf("Tx is complete!\n");
	}
	else if(AppEv == SPI_EVENT_RX_CMPLT)
	{
//		printf("Rx is complete!\n");
	}
	else if(AppEv == SPI_EVENT_OVR_ERR)
	{
//		printf("OVR Error triggered!\n");
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
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIpins);


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

void SPI2_Inits(void)
{
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //HW Slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}


void GPIO_ButtonInit(void)
{
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
