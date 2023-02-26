/*
 * 001led_toggle.c
 *
 *  Created on: Sep 30, 2019
 *      Author: pigmo
 */


#include "stm32F407xx.h"

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}
void GPIO_setup(void);

int main(void) {
	GPIO_setup();

	while(1){
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_12);
		delay();
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_13);
		delay();
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_14);
		delay();
	}
	return 0;
}

void GPIO_setup(void){
	GPIO_Handle_t GPIOHandler;

	GPIOHandler.pGPIOx = GPIOD;
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//GPIO PIN 12 will be used for a normal push pull led
	GPIOHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOHandler.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOHandler.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GPIOHandler.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOHandler.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOHandler);

	//GPIO PIN 13 will be used as a open drain with internal resistor
	GPIOHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIOHandler.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OD;
	GPIOHandler.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_Init(&GPIOHandler);

	//GPIO PIN 14 will be used as a open drain with external resistor
	//external resistor is connected between a 5V pin and PD14 with a 470 Ohm resistor
	GPIOHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIOHandler.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOHandler);

}
