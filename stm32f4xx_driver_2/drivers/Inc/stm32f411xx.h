
#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>


// Một số định nghĩa macro

#define __vo volatile




// Viết macro base address cho BUS domains

#define AHB1_BASEADDR       0x40020000UL
#define AHB2_BASEADDR 	    0x50000000UL
#define APB1_BASEADDR	    0x40000000UL
#define APB2_BASEADDR       0x40010000UL


// Macro base address cho bộ nhớ

#define SRAM_BASEADDR		0x20000000UL
#define FLASH_BASEADDR   	0x08000000UL
#define ROM_BASEADDR	 	0x1FFF0000UL            // System memory


/* Viết macro cho peripherals bus domains */

// APB1 BUS
#define TIM2_BASEADDR 		 APB1_BASEADDR
#define TIM3_BASEADDR		(APB1_BASEADDR + 0x0400)
#define TIM4_BASEADDR		(APB1_BASEADDR + 0x0800)
#define TIM5_BASEADDR		(APB1_BASEADDR + 0x0C00)
#define RTC_BASEADDR		(APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR		(APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR		(APB1_BASEADDR + 0x3000)
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400)
#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR 		(APB1_BASEADDR + 0x5800)
#define I2C3_BASRADDR		(APB1_BASEADDR + 0x5C00)
#define PWR_BASEADDR		(APB1_BASEADDR + 0x7000)

// APB2 BUS
#define TIM1_BASEADDR		 APB2_BASEADDR
#define USART1_BASEADDR 	(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400)
#define ADC1_BASEADDR		(APB2_BASEADDR + 0x2000)
#define SDIO_BASEADDR		(APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR		(APB2_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00)
#define TIM9_BASEADDR		(APB2_BASEADDR + 0x4000)
#define TIM10_BASEADDR		(APB2_BASEADDR + 0x4400)
#define TIM11_BASEADDR		(APB2_BASEADDR + 0x4800)
#define SPI5_BASEADDR		(APB2_BASEADDR + 0x5000)

// AHB1 BUS
#define GPIOA_BASEADDR 		 AHB1_BASEADDR
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR		(AHB1_BASEADDR + 0x1C00)
#define CRC_BASEADDR		(AHB1_BASEADDR + 0x3000)
#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800)
#define FIR_BASEADDR		(AHB1_BASEADDR + 0x3C00)
#define DMA1_BASEADDR		(AHB1_BASEADDR + 0x6000)
#define DMA2_BASEADDR		(AHB1_BASEADDR + 0x6400)

// AHB2 BUS
#define	OTG_BASEADDR		AHB2_BASEADDR


// Định nghĩa Struct cho ngoại vi

// GPIO structure
typedef struct {
	__vo uint32_t 	MODER;			/*	GPIO port mode register										*/
	__vo uint32_t	OTYPER;			/*	GPIO port output type register								*/
	__vo uint32_t	OSPEEDR;		/*	GPIO port output speed register								*/
	__vo uint32_t	PUPDR;			/*	GPIO port pull-up/pull-down register						*/
	__vo uint32_t	IDR;			/*	GPIO port input data register								*/
	__vo uint32_t	ODR;			/*	GPIO port output data register								*/
	__vo uint32_t	BSRR;			/*	GPIO port bit set/reset register							*/
	__vo uint32_t	LCKR;			/*	GPIO port configuration lock register 						*/
	__vo uint32_t	AFR[2];			/*	GPIO alternate function register  [0] 0-7  [1] 8-15			*/
}GPIO_RegDef_t;


/*
 * Cách khai báo chuẩn
 * GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t *) GPIOA_BASEADDR
 */


#define GPIOA 	(GPIO_RegDef_t *)GPIOA_BASEADDR
#define GPIOB 	(GPIO_RegDef_t *)GPIOB_BASEADDR
#define GPIOC 	(GPIO_RegDef_t *)GPIOC_BASEADDR
#define GPIOD 	(GPIO_RegDef_t *)GPIOD_BASEADDR
#define GPIOE 	(GPIO_RegDef_t *)GPIOE_BASEADDR
#define GPIOH 	(GPIO_RegDef_t *)GPIOH_BASEADDR

// RCC structure để enable hoặc disable peripheral clock

typedef struct{
	__vo uint32_t	CR;					// 0x00
	__vo uint32_t	PLLCFGR;			// 0x04
	__vo uint32_t	CFGR;				// 0x08
	__vo uint32_t	CIR;				// 0x0C
	__vo uint32_t	AHB1RSTR;			// 0x10
	__vo uint32_t	AHB2RSTR;			// 0x14
	__vo uint32_t	Reserved1[2];		// 0x18, 0x1C
	__vo uint32_t	APB1RSTR;			// 0x20
	__vo uint32_t	APB2RSTR;			// 0x24
	__vo uint32_t	Reserved2[2];		// 0x28, 0x2C
	__vo uint32_t	AHB1ENR;			// 0x30
	__vo uint32_t	AHB2ENR;			// 0x34
	__vo uint32_t	Reserved3[2];		// 0x38, 0x3C
	__vo uint32_t	APB1ENR;			// 0x40
	__vo uint32_t	APB2ENR;			// 0x44
	__vo uint32_t	Reserved4[2];		// 0x48, 0x4C
	__vo uint32_t	AHB1LPENR;			// 0x50
	__vo uint32_t	AHB2LPENR;			// 0x54
	__vo uint32_t	Reserved5[2];		// 0x58, 0x5C
	__vo uint32_t	APB1LPENR;			// 0x60
	__vo uint32_t	APB2LPENR;			// 0x64
	__vo uint32_t	Reserved6[2];		// 0x68, 0x6C
	__vo uint32_t	BDCR;				// 0x70
	__vo uint32_t	CSR;				// 0x74
	__vo uint32_t	Reserved7[2];		// 0x78, 0x7C
	__vo uint32_t	SSCGR;				// 0x80
	__vo uint32_t	PLLI2SCFGR;			// 0x84
	__vo uint32_t	Reserved8;			// 0x88
	__vo uint32_t	DCKCFGR;			// 0x8C
}RCC_RegDef_t;

#define RCC		((RCC_RegDef_t *)RCC_BASEADDR)


// EXTI structure

typedef struct
{
	__vo uint32_t IMR;		// Interrupt mask register (EXTI_IMR)  				0x00
	__vo uint32_t EMR;		// Event mask register (EXTI_EMR)					0x04
	__vo uint32_t RTSR;		// Rising trigger selection register (EXTI_RTSR)    0x08
	__vo uint32_t FTSR;		// Falling trigger selection register (EXTI_FTSR)   0x0C
	__vo uint32_t SWIER; 	// Software interrupt event register (EXTI_SWIER)   0x10
	__vo uint32_t PR;		// Pending register (EXTI_PR)						0x14

}EXTI_Regdef_t;

#define EXTI	((EXTI_Regdef_t *)EXTI_BASEADDR)

// Định nghĩa IRQ number cho EXTI
#define EXTI_NO_0			6
#define EXTI_NO_1			7
#define EXTI_NO_2			8
#define EXTI_NO_3			9
#define EXTI_NO_4			10
#define EXTI_NO_9_5			23
#define EXTI_NO_15_10		40

// SYSCONFIG structure
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t 	  Reserved1[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

#define SYSCFG   ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

// Viết macro enable và disable clock của peripherals

// Macro ENABLE cho BUS AHB1
// RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
#define GPIOA_PCLK_EN()  		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()  		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()  		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()  		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()  		(RCC->AHB1ENR |= (1 << 7))

// Macro DISABLE cho BUS AHB1
#define GPIOA_PCLK_DI()  		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()  		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()  		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()  		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()  		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()  		(RCC->AHB1ENR &= ~(1 << 7))

// Macro ENABLE cho BUS APB1
// RCC APB1 peripheral clock enable register (RCC_APB1ENR)
#define TIM2_PCLK_EN()			(RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN()			(RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN()			(RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN()			(RCC->APB1ENR |= (1 << 3))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))

// Macro DISABLE cho BUS APB1
#define TIM2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 3))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23))


// Macro ENALBE cho BUS APB2
//RCC APB2 peripheral clock enable register (RCC_APB2ENR)
#define TIM1_PCLK_EN()			(RCC->APB2ENR |= (1 << 0))
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define ADC1_PCLK_EN()			(RCC->APB2ENR |= (1 << 8))
#define SDIO_PCLK_EN()			(RCC->APB2ENR |= (1 << 11))
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))
#define TIM9_PCLK_EN()			(RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN()			(RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN()			(RCC->APB2ENR |= (1 << 18))
#define SPI5_PCLK_EN()			(RCC->APB2ENR |= (1 << 20))


// Macro DISABLE cho BUS APB2
#define TIM1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 0))
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))
#define ADC1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 8))
#define SDIO_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 11))
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13))
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))
#define TIM9_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 16))
#define TIM10_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 17))
#define TIM11_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 18))
#define SPI5_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 20))

/*
 * Macro để RESET GPIOx peripherals
 */

#define GPIOA_REG_RESET()   			do{ (RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()   			do{ (RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()   			do{ (RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()   			do{ (RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()   			do{ (RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOH_REG_RESET()   			do{ (RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1<<7)); }while(0)
// Some MacroName



// PortCode cho GPIO

#define GPIO_PORTCODE(x)    ((x == GPIOA) ? 0 : \
							(x == GPIOB) ? 1 : \
							(x == GPIOC) ? 2 : \
							(x == GPIOD) ? 3 : \
							(x == GPIOE) ? 4 : \
							(x == GPIOH) ? 7 : 0)


// Macro IRQ number

#define  IRQ_NO_EXTI0			6
#define  IRQ_NO_EXTI1			7
#define  IRQ_NO_EXTI2			8
#define  IRQ_NO_EXTI3			9
#define  IRQ_NO_EXTI4			10
#define  IRQ_NO_EXTI9_5			23
#define  IRQ_NO_EXTI15_10		40

// Macro for NVIC register
// NVIC_ISER0 - NVIC_ISER7  Interrupt Set-Enable Registers
// Cấu trúc của NVIC_ISER0, 32 bit, lưu giá trị IRQ 0 đến 31
// Cấu trúc của NVIC_ISER1, 32 bit, lưu giá trị IRQ 32 đến 61
#define NVIC_ISER0_BASEADDR				((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1_BASEADDR				((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2_BASEADDR				((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3_BASEADDR				((__vo uint32_t *)0xE000E10C)
#define NVIC_ISER4_BASEADDR				((__vo uint32_t *)0xE000E110)
#define NVIC_ISER5_BASEADDR				((__vo uint32_t *)0xE000E114)
#define NVIC_ISER6_BASEADDR				((__vo uint32_t *)0xE000E118)
#define NVIC_ISER7_BASEADDR				((__vo uint32_t *)0xE000E11C)

// NVIC_ICER0 - NVIC_ICER7  Interrupt Clear-Enable Registers
#define NVIC_ICER0_BASEADDR				((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1_BASEADDR				((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2_BASEADDR				((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3_BASEADDR				((__vo uint32_t *)0xE000E18C)
#define NVIC_ICER4_BASEADDR				((__vo uint32_t *)0xE000E190)
#define NVIC_ICER5_BASEADDR				((__vo uint32_t *)0xE000E194)
#define NVIC_ICER6_BASEADDR				((__vo uint32_t *)0xE000E198)
#define NVIC_ICER7_BASEADDR				((__vo uint32_t *)0xE000E19C)

// NVIC_IPR0 - NVIC_IPR59	Interrupt Priority Register

#define NVIC_IPR_BASEADDR				((__vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			4

// Define IRQ PRIORITY
#define NVIC_IRQ_PRIO_0			0
#define NVIC_IRQ_PRIO_1			1
#define NVIC_IRQ_PRIO_2			2
#define NVIC_IRQ_PRIO_3			3
#define NVIC_IRQ_PRIO_4			4
#define NVIC_IRQ_PRIO_5			5
#define NVIC_IRQ_PRIO_6			6
#define NVIC_IRQ_PRIO_7			7
#define NVIC_IRQ_PRIO_8			8
#define NVIC_IRQ_PRIO_9			9
#define NVIC_IRQ_PRIO_10		10
#define NVIC_IRQ_PRIO_11		11
#define NVIC_IRQ_PRIO_12		12
#define NVIC_IRQ_PRIO_13		13
#define NVIC_IRQ_PRIO_14		14
#define NVIC_IRQ_PRIO_15		15


// Define some common macros
#define SET 			1
#define RESET 			0
#define ENABLE			1
#define DISABLE			0
#define GPIO_PIN_SET	1
#define GPIO_PIN_RESET  0


#include "stm32f411xx_gpio_driver.h"
#include "string.h"
#endif

