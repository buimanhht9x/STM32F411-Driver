/*
 * 015usart_case.c
 *
 *  Created on: Jun 3, 2020
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

char *MESSAGES[3] = {"hihihihihihihi123", "Hello how are you?", "Today is monday !"};
char rxBuffer[1024];

uint8_t rxCmplt = RESET;
uint8_t gData = 0;

void USART_GPIO_Setup(void);
void GPIO_ButtonInit(void);
void USART_Setup(void);

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}


//NOTE: mishandling if TXE is not cleared, but I2C_MasterReceiveDataIT is not set
int main(void)
{
	uint8_t cnt = 0;
	char *msg;

	USART_GPIO_Setup();

	GPIO_ButtonInit();

	USART_Setup();

	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
	printf("Application is running!\n");
	while(1){
		//wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay(250000);//used for button debouncing

		cnt = cnt %3;
		msg = MESSAGES[cnt];

		//enable rx interrupt
		while(USART_ReceiveDataIT(&hUSART2, (uint8_t*) rxBuffer, strlen(msg)) != USART_READY);

		//send alternating data using count
		USART_SendData(&hUSART2, (uint8_t*) msg, strlen(msg));

		printf("Transmitted: %s\n", msg);

		//wait for rxcmplt
		while(rxCmplt != SET);

		rxBuffer[strlen(msg)+1] = '\0';
		//print out data
		printf("Received: %s\n", rxBuffer);

		//reset rxCmplt and increment count by 1
		rxCmplt = RESET;
		cnt++;

	}

	return 0;
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&hUSART2);
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
	hUSART2.USART_Config.USART_Mode = USART_MODE_TXRX;
	hUSART2.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	hUSART2.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	hUSART2.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&hUSART2);
}



void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t appEvent){
	switch(appEvent)
	{
	case USART_EV_TX_CMPLT:
		break;
	case USART_EV_RX_CMPLT:
		rxCmplt = SET;
		break;
//	case USART_EV_CTS:
//	case USART_EV_IDLE:
//	case USART_ER_ORE:
//	case USART_ER_FE:
//	case USART_ER_NF:
	}
}
