/*
 * 0010i2c_master_tx_testing.c
 *
 *  Created on: May 1, 2020
 *      Author: Adam
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


/*
 * PB6 --> I2C_SCL
 * PB7 --> I2C_SDA
 * ALT Function mode: 4
 */


//STM defines these here as "global variables"
I2C_Handle_t hI2C1;
GPIO_Handle_t I2Cpins;
GPIO_Handle_t GpioBtn;
uint8_t message[32];

void I2C_GPIO_Setup(void);
void GPIO_ButtonInit(void);
void I2C_Setup(void);

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}

#define MY_ADDR		0x61
#define SLAVE_ADDR	0x68

#define COMMAND_READ_LENGTH		0x51
#define COMMAND_READ_DATA		0x52


int main(void) {
	printf("Hello World\n");
	//initizalize button
	GPIO_ButtonInit();

	//This function is used to initialize the gpio pins to behave as I2C pins
	I2C_GPIO_Setup();

	//This function is used to initialize the SPI peripheral
	I2C_Setup();

	//Course uses the PeripheralControl function to enable I2C here, but I implemented this in the read and write functions
	//course also enables acking here, but I do this in the receive function to match the STM code

	uint8_t commandcode;
	uint8_t messageLength;
	while(1){
		//wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000); //used for button debouncing

		//send command code to read the length of data
		commandcode = COMMAND_READ_LENGTH;
		I2C_MasterSendData(&hI2C1, &commandcode, 1, SLAVE_ADDR, I2C_SR_ENABLED);
		//receive the length from the slave
		I2C_MasterReceiveData(&hI2C1, &messageLength, 1, SLAVE_ADDR, I2C_SR_ENABLED);

		//send command code to read the message
		commandcode = COMMAND_READ_DATA;
		I2C_MasterSendData(&hI2C1, &commandcode, 1, SLAVE_ADDR, I2C_SR_ENABLED);
		//receive the message from the slave
		//receive buffer is only 32 bits
		I2C_MasterReceiveData(&hI2C1,  message, messageLength, SLAVE_ADDR, I2C_SR_DISABLED);

		//note: why does this work without adding a null character?
		printf("%s\n", message);


	}

	return 0;
}

void I2C_GPIO_Setup(void) {

	I2Cpins.pGPIOx = GPIOB;
	I2Cpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2Cpins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2Cpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OD;
	I2Cpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	I2Cpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2Cpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2Cpins);

	//SDL
	//changed from pin 9 because of little glitch (not sure if this is happening on my board, but changing anyway)
	I2Cpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2Cpins);


}
void GPIO_ButtonInit(void){

	// configure button
	//Set up PA0 to be an input for the button press
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn);
}
void I2C_Setup(void){
	hI2C1.pI2Cx = I2C1;
	hI2C1.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	hI2C1.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	hI2C1.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	hI2C1.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

	I2C_Init(&hI2C1);
}
