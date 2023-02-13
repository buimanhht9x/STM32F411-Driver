/*
 * Đề bài:
 * Kết nối button PD5 và toggle mỗi khi có trigger falling edge interrupt
 */


#include "stm32f411xx.h"


void delay()
{
	for(uint32_t i=0 ; i < 50000/2 ;i++);
}
void main(void)
{
	GPIO_Handle_t button,led;

	// Clear value 0
	memset(&button,0,sizeof(button));
	memset(&led,0,sizeof(led));

	// Khai báo button PD5
	button.pGPIOx = GPIOD;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FT;
	button.GPIO_PinConfig.GPIO_PinPUPD = GPIO_PU;

	GPIO_Init(&button);
	// Khai báo led PD14
	led.pGPIOx = GPIOD;
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	led.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_MED;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
	led.GPIO_PinConfig.GPIO_PinPUPD = GPIO_NO_PUPD;

	GPIO_Init(&led);

	GPIO_PeriClockControl(GPIOD, ENABLE);

	// Config IRQ
	GPIO_IRQConfigNumber(IRQ_NO_EXTI9_5, ENABLE);

	GPIO_IRQConfigPriority(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO_15);

	while(1);
}

void EXTI9_5_IRQHandler()
{
	delay();
	// Clear the Pending interrupt from the EXTI Line
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}
