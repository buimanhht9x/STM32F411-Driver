/*
 * 002led_button.c
 *
 *  Created on: Oct 2, 2019
 *      Author: pigmo
 */


//Excersize to connect external button to PB12 and external LED to PA14, then have the button toggle the LED

#include "stm32F407xx.h"

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}
void GPIO_setup(void);

int main(void) {
	GPIO_setup();

	while(1){
		//read PA0 associated with the button
		uint8_t pinState = GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_12);
		if(pinState == 0 ){
			delay(250000);
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_8);
//			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//		} else {
//			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		}
	}
	return 0;
}

void GPIO_setup(void){
	//Using two GPIO_Handle_t structs here for readability, though one could be used to save space

	GPIO_Handle_t GpioLedExt, GpioBtnExt;

	//Set up PB12 to be an input for the external button press
	GpioBtnExt.pGPIOx = GPIOB;
	GpioBtnExt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioBtnExt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtnExt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtnExt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtnExt);


	//Set up PA14 to be an output for the external LED
	GpioLedExt.pGPIOx = GPIOA;
	GpioLedExt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GpioLedExt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedExt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLedExt.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GpioLedExt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLedExt);



}
