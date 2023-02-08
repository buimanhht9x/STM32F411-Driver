#include "stm32f411xx_gpio_driver.h"

/*
 *  @fn					-  GPIO_PeriClockControl
 *  @brief				-  enable or disable peripheral clock for given GPIO port
 *
 *  @param[1]			-  GPIO_RegDef_t *pGPIOx // Base Address GPIO Port
 *  @param[2]			-  uint8_t EnorDi        // enable or disable
 *  @param[3]			-
 *
 *  @return				-  none
 *
 *  @note				-  none
 */
void GPIO_PeriClockControl(	GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	switch(EnorDi)
	{
		case ENABLE:
			if(pGPIOx == GPIOA)
				GPIOA_PCLK_EN();
			else if (pGPIOx == GPIOB)
				GPIOB_PCLK_EN();
			else if (pGPIOx == GPIOC)
				GPIOC_PCLK_EN();
			else if (pGPIOx == GPIOD)
				GPIOD_PCLK_EN();
			else if (pGPIOx == GPIOE)
				GPIOE_PCLK_EN();
			else if (pGPIOx == GPIOH)
				GPIOH_PCLK_EN();
			break;
		case DISABLE:
			if(pGPIOx == GPIOA)
				GPIOA_PCLK_DI();
			else if (pGPIOx == GPIOB)
				GPIOB_PCLK_DI();
			else if (pGPIOx == GPIOC)
				GPIOC_PCLK_DI();
			else if (pGPIOx == GPIOD)
				GPIOD_PCLK_DI();
			else if (pGPIOx == GPIOE)
				GPIOE_PCLK_DI();
			else if (pGPIOx == GPIOH)
				GPIOH_PCLK_DI();
			break;
	}

}

/*
 * GPIO Init and DeInit Pin
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	/*
	 * Config the pinMode
	 *   + Chia ra Nomal Mode
	 *   + Interrupt Mode
	 */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		// Clearbit trước khi Set
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		// Setbit
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// Interrupt mode
	}

	temp = 0;

	// Config the pin Speed

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// Clearbit trước khi Set
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// Setbit
	pGPIOHandle->pGPIOx->OSPEEDR |=  temp;

	temp = 0;

	// Config the OPType

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	// Clearbit trước khi Set
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// Setbit
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp =0;

	// Config the PUPD

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPD << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// Clearbit trước khi Set
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// Setbit
	pGPIOHandle->pGPIOx->PUPDR |=  temp;

	temp =0;

	// Config the Alternate Function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Cấu hình Alternate Function đây
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ;

		// Lấy phần nguyên để xem thanh ghi High hay Low
		// Nếu temp1  = 0 thì thanh ghi LOW
		// Nếu temp1 = 1  thì thanh ghi HIGH
		// Lấy phần dư, mỗi ô cấu hình pin có 4 bit, vậy ta cần ghi value << (4 * temp2)

		// ClearBit trước khi set
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~ (0xF << (4 * temp2));
		// SET bit
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2);

	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx ==  GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx ==  GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/*
 * GPIO Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) (pGPIOx->IDR >> PinNumber) & 0x01;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR ;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
		pGPIOx->ODR |= 1 << PinNumber;
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * GPIO IRQ Handle
 */
void GPIO_IRQConfigNumber(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void GPIO_IRQConfigPriority(uint8_t IRQPriority)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}


