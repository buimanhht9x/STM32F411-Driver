/*
 * 007spi_cmd_handling.c
 *
 *  Created on: Mar 23, 2020
 *      Author: Adam
 */

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

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}

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


uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == ACK) {
		return 1;
	} else
	{
		return 0;
	}
}

int main(void) {

	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;

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
		// Sets NSS to low?
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL	<pin no(1)>		<value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, 1);
		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead,1);

		//send dummy bits (1byte) to fetch response of slave
		SPI_SendData(SPI2, &dummyWrite, 1);
		//receive ackbyte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte)){
			//ack
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			//NOTE: only need to read one byte to clear the RXNE because that's all the DR can hold
			//if I wanted to read more, I would have to do seperate writes
			//I couldn't have a length of greater than 1 because the slave cannot initiate communication
			// and the Arduino would just overwrite it's data register until I read
			// Arduino also waits for transmission complete, how does this work?
			SPI_ReceiveData(SPI2, &dummyRead, 1);

//			printf("Completed LED Write. Wrote value %d to pin %d.", args[0], args[1]);
		}

		// wait for button to be pressed for second command
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000); //used for button debouncing

		//2. COMMAND_SENSOR_READ	<pin no(1)>
		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2, &commandcode, 1);
		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead,1);

		//send dummy bits (1byte) to fetch response of slave
		SPI_SendData(SPI2, &dummyWrite, 1);
		//receive ackbyte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t analogRead;
		if(SPI_VerifyResponse(ackbyte)){
			//ack
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			delay(250000); //allow slave to read analog

			//get the value returned form the slave analog
			SPI_SendData(SPI2, &dummyWrite, 1);
			SPI_ReceiveData(SPI2, &analogRead, 1);

//			printf("Completed Annalog read. Read value %d from analog pin %d.", analogRead, args[0]);
		}

		// wait for button to be pressed for third command
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000); //used for button debouncing

		//3. COMMAND_LED_READ	<pin no(1)>
		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2, &commandcode, 1);
		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead,1);

		//send dummy bits (1byte) to fetch response of slave
		SPI_SendData(SPI2, &dummyWrite, 1);
		//receive ackbyte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t ledRead;
		if(SPI_VerifyResponse(ackbyte)){
			//ack
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			delay(250000); //allow slave to read pin

			//get the value returned form the slave analog
			SPI_SendData(SPI2, &dummyWrite, 1);
			SPI_ReceiveData(SPI2, &ledRead, 1);

//			printf("Completed LED read. Read value %d from pin %d.", ledRead, args[0]);
		}

		// wait for button to be pressed for 4th command
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000); //used for button debouncing

		//4. COMMAND_PRINT	<STRING LEN (1)>
		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2, &commandcode, 1);
		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead,1);

		//send dummy bits (1byte) to fetch response of slave
		SPI_SendData(SPI2, &dummyWrite, 1);
		//receive ackbyte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		char userData[] = "Hello, I am robot.";
		if(SPI_VerifyResponse(ackbyte)){
			//ack
			args[0] = strlen(userData);
			SPI_SendData(SPI2, args, 1);

			//send string
			SPI_SendData(SPI2, (uint8_t*) userData, args[0]);
			SPI_ReceiveData(SPI2, &dummyRead, 1);

//			printf("Completed Print. Sent string %s.", userData);
		}

		// wait for button to be pressed for 5th command
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000); //used for button debouncing

		//4. COMMAND_ID_READ	<STRING LEN (1)>
		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2, &commandcode, 1);
		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead,1);

		//send dummy bits (1byte) to fetch response of slave
		SPI_SendData(SPI2, &dummyWrite, 1);
		//receive ackbyte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t boardID[11];
		uint32_t i =0;
		if(SPI_VerifyResponse(ackbyte)){
			//ack
			for(i=0; i < sizeof(boardID)-2; i++){
				SPI_SendData(SPI2, &dummyWrite, 1);
				SPI_ReceiveData(SPI2, &boardID[i], 1);
			}
			boardID[10]='\0';

//			printf("Completed ID read. Read ID %s.", boardID);
		}

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


