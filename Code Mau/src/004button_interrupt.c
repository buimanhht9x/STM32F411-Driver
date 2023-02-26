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

	}
	return 0;
}

void GPIO_setup(void){
	//Using two GPIO_Handle_t structs here for readability, though one could be used to save space

	GPIO_Handle_t GpioLed, GpioBtn;


	//Set up PD12 to be an output for the internal LED
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Set up PD5 to be an input for the external button press
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
//	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(NVIC_IRQ_PRI15, IRQ_NO_EXTI9_5);
}


//this is defined as weak in the startup_stm32.s file
void EXTI9_5_IRQHandler(void) {
	delay(200000);
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}
