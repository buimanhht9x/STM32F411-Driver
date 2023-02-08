#include "stm32f411xx.h"

void delay()
{
	for(uint32_t i=0 ; i < 50000;i++);
	for(uint32_t i=0 ; i < 50000;i++);
	for(uint32_t i=0 ; i < 50000;i++);
}

int main()
{
	GPIO_Handle_t LED1;
	LED1.pGPIOx = GPIOD;
	LED1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	LED1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED1.GPIO_PinConfig.GPIO_PinOPType =  GPIO_OPT_PP;
	LED1.GPIO_PinConfig.GPIO_PinPUPD = GPIO_PU;
	LED1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&LED1);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_14);
		delay();
	}
	return 0;
}

