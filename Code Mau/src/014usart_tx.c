/*
 * 014usart_tx.c
 *
 *  Created on: Jun 2, 2020
 *      Author: Adam
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


/*
 * PA2 --> USART_TX
 * PA3 --> USART_RX
 * ALT Function mode: 7
 */


//STM defines these here as "global variables"
USART_Handle_t hUSART2;
GPIO_Handle_t GpioBtn;
GPIO_Handle_t USARTpins;
char MESSAGE[1024] = "USART Tx Testing...\n\r";


void USART_GPIO_Setup(void);
void GPIO_ButtonInit(void);
void USART_Setup(void);

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}


//NOTE: mishandling if TXE is not cleared, but I2C_MasterReceiveDataIT is not set
int main(void)
{

	USART_GPIO_Setup();

	GPIO_ButtonInit();

	USART_Setup();



	while(1){
		//wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000);//used for button debouncing

		USART_SendData(&hUSART2, (uint8_t*)MESSAGE, strlen(MESSAGE));
	}

	return 0;
}

void USART_GPIO_Setup(void) {

	USARTpins.pGPIOx = GPIOA;
	USARTpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTpins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	USARTpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	USARTpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//tx
	USARTpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&USARTpins);

	//rx
	USARTpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&USARTpins);

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

void USART_Setup(void){
	hUSART2.pUSARTx = USART2;
	hUSART2.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	hUSART2.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	hUSART2.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	hUSART2.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	hUSART2.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	hUSART2.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&hUSART2);
}



//void USART_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent){
//
//}
