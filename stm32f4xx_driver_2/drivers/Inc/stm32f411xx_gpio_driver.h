#ifndef STM32F411XX_GPIO_DRIVER_H
#define STM32F411XX_GPIO_DRIVER_H
#include "stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;		//  @GPIO PIN Number
	uint8_t GPIO_PinMode; 		//  @GPIO port mode register	input, output, adc, alternate...
	uint8_t GPIO_PinSpeed;		//	@GPIO port output speed register
	uint8_t GPIO_PinOPType;		//  @GPIO port output type register
	uint8_t GPIO_PinPUPD;		//  @GPIO port pull-up/pull-down register
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t *pGPIOx; // Base Address của PORT
	GPIO_PinConfig_t GPIO_PinConfig; //Biến chứa thông tin của Pin settings.
}GPIO_Handle_t;

/*
 * Định nghĩa macro cho các thanh ghi GPIO
 */

/*
 * @GPIO port mode register (GPIOx_MODER)
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IN_RT		4
#define GPIO_MODE_IN_FT		5
#define GPIO_MODE_IN_RFT	6

/*
 * @GPIO port output type register (GPIOx_OTYPER)
 */
#define GPIO_OPT_PP			0
#define GPIO_OPT_OD			1


/*
 * @GPIO port output speed register (GPIOx_OSPEEDR)
 */
#define GPIO_OUT_SPEED_LOW	0
#define GPIO_OUT_SPEED_MED	1
#define GPIO_OUT_SPEED_FAST	2
#define GPIO_OUT_SPEED_HIGH 3

/*
 * @GPIO port pull-up/pull-down register (GPIOx_PUPDR)
 */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2

/*
 * @GPIO PIN Number
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15


/*
 *  Clock Control GPIO
 */
void GPIO_PeriClockControl(	GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * GPIO Init and DeInit Pin
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * GPIO IRQ Handle
 */
void GPIO_IRQConfigNumber(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQConfigPriority(uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif
