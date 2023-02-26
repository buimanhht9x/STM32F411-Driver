/*
 * 007api_cmd_handling_anon_refactor.c
 *
 *  Created on: Mar 30, 2020
 *      Author: adam
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
void GPIO_LedTestingInit(void);
void SPI_Setup(void);

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}

#define BOARD_ID_LEN	10

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

// I can add multiple types of structs some with the return value, but this would fuck up the iteration of the command chain
typedef struct {
	uint8_t command;
	uint8_t args[2];
	uint8_t *userString;
	uint8_t retLength;
} spiCmd;

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == ACK) {
		return 1;
	} else
	{
		return 0;
	}
}

//This should probably return a success code
void spi_send_dummy_read(SPI_RegDef_t* ppSPIx, uint8_t *pTxBuffer, uint32_t len){
	uint8_t dummyRead;
	//send data
	SPI_SendData(ppSPIx,pTxBuffer, len);
	//dummy read to clear off the RXNE
	SPI_ReceiveData(ppSPIx, &dummyRead, 1);

	//NOTE: only need to read one byte to clear the RXNE because that's all the DR can hold
	//if I wanted to read more, I would have to do seperate writes
	//I couldn't have a length of greater than 1 because the slave cannot initiate communication
	// and the Arduino would just overwrite it's data register until I read
	// Arduino also waits for transmission complete, how does this work?
}


//for now, always read 1 byte
//This should probably return a success code
void spi_read_dummy_send(SPI_RegDef_t* ppSPIx, uint8_t *pRxBuffer){
	uint8_t dummyWrite = 0xff;
	//dummy send
	SPI_SendData(SPI2, &dummyWrite, 1);
	//recieve data
	SPI_ReceiveData(SPI2, pRxBuffer, 1);
}

uint8_t send_command(uint8_t commandcode){
	uint8_t ackbyte;

	spi_send_dummy_read(SPI2, &commandcode, 1);
	spi_read_dummy_send(SPI2, &ackbyte);

	return ackbyte;
}

uint8_t execute_command(SPI_RegDef_t* ppSPIx, spiCmd cmdStruct, uint8_t *retBuffer){
	uint8_t ackbyte;
	uint8_t returnVal = 1; //should figure out how to confirm this return val for each command
	uint8_t commandcode = cmdStruct.command;
	uint8_t *args = cmdStruct.args;
	ackbyte = send_command(commandcode);

	if(SPI_VerifyResponse(ackbyte)){

		//ack
		switch(commandcode){
			case COMMAND_LED_CTRL:
			{
				//should probably check that args are valid
				spi_send_dummy_read(ppSPIx, args, 2);

//				printf("Completed LED Write. Wrote value %d to pin %d.", args[0], args[1]);
				returnVal = 0;
				break;
			}
			case COMMAND_SENSOR_READ:
			{
				//verify arg is analog pin
				spi_send_dummy_read(ppSPIx, args, 1);

				delay(250000); //allow slave to read analog

				//get the value returned form the slave analog
				spi_read_dummy_send(ppSPIx, retBuffer);

//				printf("Completed Annalog read. Read value %d from analog pin %d.", analogRead, args[0]);
				returnVal = 0;
				break;
			}
			case COMMAND_LED_READ:
			{
				//verify arg is led pin
				spi_send_dummy_read(ppSPIx, args, 1);

				delay(250000); //allow slave to read pin

				//get the value returned form the slave led
				spi_read_dummy_send(ppSPIx, retBuffer);

//				printf("Completed LED read. Read value %d from pin %d.", ledRead, args[0]);
				returnVal = 0;
				break;
			}
			case COMMAND_PRINT:
			{
				// check that args are valid
				SPI_SendData(ppSPIx, args, 1);
				//send string
				spi_send_dummy_read(ppSPIx, cmdStruct.userString, args[0]);

//				printf("Completed Print. Sent string %s.", userData);
				returnVal = 0;
				break;
			}
			case COMMAND_ID_READ:
			{
				uint32_t i =0;
				for(i=0; i < BOARD_ID_LEN; i++){
					spi_read_dummy_send(ppSPIx, &retBuffer[i]);
				}
				retBuffer[BOARD_ID_LEN]='\0';

//				printf("Completed ID read. Read ID %s.", boardID);
				returnVal = 0;
				break;
			}
		}
	}
	return returnVal;
}

//returns 1 if command passes test
//these are test cases as outlined below
//led ctrl shouldn't return anything
//led read should return 0
//analog0 read should return 0xff
// id read should return ARDUINOUNO

uint8_t test_command_result(spiCmd cmdStruct, uint8_t *retBuffer){
	uint8_t commandcode = cmdStruct.command;
	uint8_t testResult = 0;
	switch(commandcode){
		case COMMAND_LED_CTRL:
		{
			testResult = 1;
			break;
		}
		case COMMAND_SENSOR_READ:
		{
			uint8_t result;
			result = *retBuffer;
			if (result == 0xff){
				testResult = 1;
			} else {
				testResult = 0;
			}
			break;
		}
		case COMMAND_LED_READ:
		{
			uint8_t result;
			result = *retBuffer;
			if(result == 0){
				testResult = 1;
			} else {
				testResult = 0;
			}
			break;
		}
		case COMMAND_PRINT:
		{
			//shouldn't return anything
			testResult = 1;
			break;
		}
		case COMMAND_ID_READ:
		{
			testResult = !(strcmp("ARDUINOUNO", (char*) retBuffer));
			break;
		}
	}
	return testResult;
}
void testAll(){

	char printString[] = "Hello I am robot";

	spiCmd ledOn;
	ledOn.command = COMMAND_LED_CTRL;
	ledOn.args[0] = LED_PIN;
	ledOn.args[1] = LED_ON;
	ledOn.retLength = 0;

	spiCmd ledOff;
	ledOff.command = COMMAND_LED_CTRL;
	ledOff.args[0] = LED_PIN;
	ledOff.args[1] = LED_OFF;
	ledOff.retLength = 0;

	spiCmd analog0Read;
	analog0Read.command = COMMAND_SENSOR_READ;
	analog0Read.args[0] = ANALOG_PIN0;
	analog0Read.retLength = 1;

	spiCmd ledRead;
	ledRead.command = COMMAND_LED_READ;
	ledRead.args[0] = LED_PIN;
	ledRead.retLength = 1;

	spiCmd slavePrint;
	slavePrint.command = COMMAND_PRINT;
	slavePrint.args[0] = strlen(printString);
	slavePrint.userString = (uint8_t *) printString;
	slavePrint.retLength = 0;

	spiCmd idRead;
	idRead.command = COMMAND_ID_READ;
	idRead.retLength = BOARD_ID_LEN+1; //add one for null byte

	spiCmd commands[] = {ledOn, ledOff, ledRead, analog0Read, slavePrint, idRead};

	uint8_t sent=0;
	uint32_t cmdIndex;
	uint8_t retBuffer[BOARD_ID_LEN];
	uint8_t faildTest = 0;
	while(1){
		//when restarting the test, restart the commands tested
		cmdIndex = 0;
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
		delay(250000); //used for button debouncing
		//orange means test in progress
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		//iterate as long as commands are available
		while(cmdIndex < sizeof(commands)/sizeof(spiCmd)){
			//wait for button to be pressed
			while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
			delay(250000); //used for button debouncing

			sent = execute_command(SPI2,commands[cmdIndex], retBuffer);
			//check if command was succesfully sent
			if(!test_command_result(commands[cmdIndex],retBuffer)){
				GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_13, GPIO_PIN_RESET);
				GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);
				faildTest=1;
				break;
			}
			if(sent == 0) {
				cmdIndex++;
			}
		}
		if(faildTest==0){
			GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_13, GPIO_PIN_RESET);
			GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_12, GPIO_PIN_SET);
		}


		//confirm the SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		//disable the SPI2 peripheral --- according to Generated code, this seems to be done in abort functions
		// this only works because my send and receive functions can enable the spi on their own
		// I'm confuse about how this has been working??
		SPI_PeripheralControl(SPI2, DISABLE);
	}

}

void testToggle (){
	spiCmd ledOn;
	ledOn.command = COMMAND_LED_CTRL;
	ledOn.args[0] = LED_PIN;
	ledOn.args[1] = LED_ON;
	ledOn.retLength = 0;

	spiCmd ledOff;
	ledOff.command = COMMAND_LED_CTRL;
	ledOff.args[0] = LED_PIN;
	ledOff.args[1] = LED_OFF;
	ledOff.retLength = 0;

	spiCmd ledRead;
	ledRead.command = COMMAND_LED_READ;
	ledRead.args[0] = LED_PIN;
	ledRead.retLength = 1;

	uint8_t result=0;
	uint8_t retBuffer[1];
	while(1){
		//wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
		delay(250000); //used for button debouncing

		result = execute_command(SPI2, ledRead, retBuffer);
		delay(2600);// need to wait for the arduino to send serial print, 2500 - 2600 seems to be the cut off, how long is this?
		if(result == 0){
			if(retBuffer[ledRead.retLength-1] == 1){
				execute_command(SPI2, ledOff, retBuffer);
			} else {
				//this happens if the led is off, or anything else happens, should check 0 here and throw exc otherwise?
				execute_command(SPI2, ledOn, retBuffer);
			}
		}
		//confirm the SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		//disable the SPI2 peripheral --- according to Generated code, this seems to be done in abort functions
		SPI_PeripheralControl(SPI2, DISABLE);
	}
}

int main(void) {
	//initalize leds
	GPIO_LedTestingInit();

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

//	testAll();
	testToggle();

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

void GPIO_LedTestingInit(void){
	GPIO_Handle_t GpioLeds;

	//GPIO PIN 12 will be used for a normal push pull led
	GpioLeds.pGPIOx = GPIOD;
	// 12 green will mean success, orange will be in progress, red will be failed
	GpioLeds.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLeds.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLeds.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GpioLeds.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioLeds.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GpioLeds);

	GpioLeds.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&GpioLeds);

	GpioLeds.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GpioLeds);
}
void SPI_Setup(void){
	SPI_Handle_t hSPI;

	hSPI.pSPIx = SPI2;
	hSPI.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	hSPI.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	hSPI.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generate SCLK of 2 MHz
	hSPI.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	hSPI.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	hSPI.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	hSPI.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management disabled for NSS pin

	SPI_Init(&hSPI);
}


