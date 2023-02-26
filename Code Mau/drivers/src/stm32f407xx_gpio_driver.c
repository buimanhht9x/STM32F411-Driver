/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 26, 2019
 *      Author: pigmo
 */

#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral clock setup
 */

/******************************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- Enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 * NOTE: pinNumber should probably be uint16_t to make accessing a pin easier
 * 			not chaning yet because course has not instructed so and may later
 * 			Also, probably don't need the read/write functions for the entire ports
 */

/******************************************************
 * @fn					- GPIO_Init
 *
 * @brief				- Initializes a given GPIO peripheral with given configurations
 *
 * @param[in]			- Struct holding base address and desired configurations of GPIO peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0; //temp. register

	//check to see if it's already enabled?
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//this is non-interrupt mode
		//Shift the mode the pin number of bits. Multiple by 2 as a single pin takes two bits
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	} else {
		// if the Pin Mode is an interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//NOTE: clearing of target bits before setting not required because only 1 bit is used
			// 1. Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 2. Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// 3. Configure FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();

		uint8_t extiCrRegIndex, extiCrPinPos, portcode;
		extiCrRegIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		extiCrPinPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		temp = portcode << (4 * extiCrPinPos);
		SYSCFG->EXTICR[extiCrRegIndex] &= ~(0xF <<(4 * extiCrPinPos)); //clearing
		SYSCFG->EXTICR[extiCrRegIndex] |= temp; //setting

		// 3. enable exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0; //is this needed?

	// 2. Configure the pupd settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT) ||
		(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)){
		// 3. Configure the speed
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

		// 4. Configure the optype if mode is output or analog
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp; //setting
	}

	// 5. Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure the alternate function registers
		uint8_t altRegIndex, altRegPinPos;

		altRegIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // pin >8 will return 1, else 0
		altRegPinPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * altRegPinPos);
		pGPIOHandle->pGPIOx->AFR[altRegIndex] &= ~(0xF << (4 * altRegPinPos)); //clearing
		pGPIOHandle->pGPIOx->AFR[altRegIndex] |= temp; //setting

	}

}

/******************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				- Deinitializes a given GPIO periphal and resets register
 *
 * @param[in]			- base address of GPIO peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}

/*
 * Data read write
 */

/******************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- gets the value from a GPIO peripheral pin
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- pin number
 *
 * @return				- PIN_SET or PIN_RESET ( 0 or 1)
 *
 * @Note				- Uses the Input Data Register to retrieve the value
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber){
	//Note: STM code uses a 16 bit value for pin and a simple & (no bit shift needed)
	uint8_t value;
	uint16_t GPIO_value = (1 << pinNumber);
	if (pGPIOx->IDR & GPIO_value){
		value = SET;
	} else {
		value = RESET;
	}
	return value;
}

/******************************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				- gets the value of a GPIO peripheral Input Data Register
 *
 * @param[in]			- base address of GPIO peripheral
 *
 * @return				- uint8_t value of GPIO peripheral input data register
 *
 * @Note				- Retrieves the Input data register
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){
	uint8_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/******************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- writes a value to a GPIO pin
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- pin number
 * @param[in]			- value to write (PIN_SET or PIN_RESET)
 *
 * @return				- none
 *
 * @Note				- uses the Bit Set/Reset Register for atomic writing
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value){
	//NOTE: STM code uses 16 bit value representing the final value and no pin Number. i.e. no shift required
	uint16_t GPIO_value = (1 << pinNumber);
	if (value == GPIO_PIN_SET){
		//write 1 to the output data register at the bit field corresponding to the pin number
		// multiply pinNumber by 2 to get the correct position in the BSy register of BSRR (high 16 bits)
		pGPIOx->BSRR = GPIO_value;
	} else {
		//write 0
		pGPIOx->BSRR = (uint32_t) GPIO_value << 16U;

	}
}

/******************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- writes a value to a GPIO peripheral
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- value to write (16 bits)
 *
 * @return				- none
 *
 * @Note				- uses the Bit Set/Reset Register for atomic writing
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value){
	//Write value to the ODR no matter what it is
	pGPIOx->BSRR = (uint32_t) value << 16U;
}

/******************************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- toggles the value of a GPIO pin
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- pin number to toggle
 *
 * @return				- none
 *
 * @Note				- uses the Bit Set/Reset Register for atomic writing
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber){
	// although XOR is used to toggle, use of BSRR makes this irrelevant
//	pGPIOx->ODR ^= (0x1 << pinNumber);
	uint16_t GPIO_value = (1 << pinNumber);
	if ( pGPIOx->ODR & GPIO_value){
		pGPIOx->BSRR = (uint32_t) GPIO_value << 16U;

	} else {
		pGPIOx->BSRR = GPIO_value;
	}
}

/*
 * IRQ configuration and handling
 */

/******************************************************
 * @fn					- GPIO_IRQITConfig
 *
 * @brief				- enables or disables a GPIO peripherals IRQ functionality
 *
 * @param[in]			- IRQ number of GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- modifies the NVIC peripheral
 * 						- feel like this should just be a general function for NVIC?
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi){
	uint8_t registerIndex, registerBitPos;
	registerIndex = IRQNumber /32;
	registerBitPos = IRQNumber % 32;
	if(enOrDi == ENABLE){
		NVIC->ISER[registerIndex] |= 1 << registerBitPos; //setting, no need to clear
	} else {
		NVIC->ICER[registerIndex] |= 1<< registerBitPos; //setting, no need to clear
	}
}

/******************************************************
 * @fn					- GPIO_IRQPriorityConfig
 *
 * @brief				- sets the priority for a given IRQ number
 *
 * @param[in]			- IRQ priority of GPIO peripheral
 * @param[in]			- IRQ number of GPIO peripheral
 *
 * @return				- none
 *
 * @Note				- modifies the NVIC peripheral
 */
void GPIO_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNumber){
	//NOTE: because we used uint8_t for the IPR register, we can select the desired register directly
	NVIC->IPR[IRQNumber]  = IRQPriority << NO_PR_BITS_IMPLEMENTED; //shift by 4 is required because the lower 4 bits are not implemented
}

/******************************************************
 * @fn					- GPIO_IRQHandling
 *
 * @brief				- deals with a interrupt triggered by a GPIO pin
 *
 * @param[in]			- GPIO pin number
 *
 * @return				- none
 *
 * @Note				- modifies the EXTI peripheral
 */
void GPIO_IRQHandling(uint8_t pinNumber){
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << pinNumber)){
		EXTI->PR |= 1<< pinNumber;
	}
}
