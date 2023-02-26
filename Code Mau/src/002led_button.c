/*
 * 002led_button.c
 *
 *  Created on: Oct 2, 2019
 *      Author: pigmo
 */


#include "stm32F407xx.h"

void delay(int time){
	for(uint32_t i=0; i<time; i++);
}
void GPIO_setup(void);

int main(void) {
	GPIO_setup();

	while(1){
		//read PA0 associated with the button
		if( GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) ){
			delay(250000);
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_12);
//			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//		} else {
//			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		}
	}
	return 0;
}

void GPIO_setup(void){
	//Using two GPIO_Handle_t structs here for readability, though one could be used to save space

	GPIO_Handle_t GpioLed, GpioBtn;

	//Set up PA0 to be an input for the button press
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);


	//GPIO PIN 12 will be used for a normal push pull led
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);



}
