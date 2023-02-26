/*
 * 013i2c_slave_tx_string2_alt.c
 *
 *  Created on: Jun 23, 2020
 *      Author: Adam
 */

//TODO: see if I can make this function mimic the original in terms limited code in the main loop
//		this will be difficult due to the delays in arduino reception
//		i.e. the arduino has send a new request for every 4 bytes

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


/*
 * PB6 --> I2C_SCL
 * PB9 --> I2C_SDA
 * ALT Function mode: 4
 */


//STM defines these here as "global variables"
I2C_Handle_t hI2C1;
GPIO_Handle_t I2Cpins;
GPIO_Handle_t GpioBtn;
uint8_t MESSAGE[] = "HiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHi...123";
volatile uint8_t txCmplt = RESET;

void I2C_GPIO_Setup(void);
void GPIO_ButtonInit(void);
void I2C_Setup(void);

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}

#define SLAVE_ADDR	0x68
#define MY_ADDR SLAVE_ADDR


#define COMMAND_CODE_LENGTH		1
#define COMMAND_READ_LENGTH		0x51
#define COMMAND_READ_DATA		0x52



//NOTE: mishandling if TXE is not cleared, but I2C_MasterReceiveDataIT is not set
int main(void) {
//	printf("Hello World\n");
	uint32_t messageLength = strlen((char*)MESSAGE);
	uint8_t commandCode = 0;

	//initizalize button
	GPIO_ButtonInit();

	//This function is used to initialize the gpio pins to behave as I2C pins
	I2C_GPIO_Setup();

	//This function is used to initialize the SPI peripheral
	I2C_Setup();

	//I2C IRQ Configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	uint32_t remainingBytes = messageLength;
	uint32_t wPtr = 0;
	while(1){
		while( I2C_SlaveReceiveDataIT(&hI2C1, &commandCode, COMMAND_CODE_LENGTH) != I2C_READY);
		while(!commandCode);
		if(commandCode == 0x51)
		{
			//Here we are sending 4 bytes of length information
			while(I2C_SlaveSendDataIT(&hI2C1, (uint8_t*) &messageLength, 4) != I2C_READY);//4 because of the arduino
		} else if (commandCode == 0x52)
		{
			//TODO: need to update the pointer for sending the data, or maybe use a delay?
			//send the contents of MESSAGE
			//the issue with trying to send all the data at once is that the arduino requests data in 32 byte chunks
			//so sending all the data at once will send fast than the arduino can read
			while(remainingBytes > 0){
				if(remainingBytes > 32){
					while(I2C_SlaveSendDataIT(&hI2C1, &MESSAGE[wPtr], 32) != I2C_READY);
					remainingBytes -= 32;
					wPtr += 32;
				} else {
					while(I2C_SlaveSendDataIT(&hI2C1, &MESSAGE[wPtr], remainingBytes) != I2C_READY);
					remainingBytes = 0;
				}
			}
			remainingBytes = messageLength;
			wPtr = 0;
		}
		while(txCmplt == RESET);
		commandCode = 0;
	}

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

void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&hI2C1);
}

void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&hI2C1);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent){
//	static uint32_t wPtr = 0;

	switch(appEvent)
	{
	case I2C_EV_SLAVE_RX_CMPLT:
//		if(COMMAND_CODE == 0x51)
//		{
//			//Here we are sending 4 bytes of length information
////			while(I2C_SlaveSendDataIT(&hI2C1, (uint8_t*) &MESSAGE_LEN, 4) != I2C_READY);//4 because of the arduino
//			I2C_SlaveSendData(&hI2C1, (uint8_t*) &MESSAGE_LEN, 4);//4 because of the arduino
//		} else if (commandCode == 0x52)
//		{
//			I2C_SlaveSendData(&hI2C1, MESSAGE[wPtr++], 1);
//		}
		break;
	case I2C_EV_SLAVE_TX_CMPLT:
		printf("Transmition Complete\n");
		txCmplt = SET;
		break;
	case I2C_EV_STOPF:
		break;
	case I2C_ER_AF:
		break;
	default:
		printf("Something unexpected happened\n");
		while(1);
	}
}
