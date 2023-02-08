#include "stm32f411xx.h"

#define BUTTON_PRESSED 		1

void delay()
{
	for(uint32_t i=0 ; i < 50000/2 ;i++);
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

	GPIO_Handle_t Button;
	Button.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPUPD = GPIO_NO_PUPD;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	Button.pGPIOx = GPIOA;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&Button);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_14);
		}
	}
	return 0;
}



