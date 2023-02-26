/*
 * stm32f407xx.h
 *
 *  Created on: Sep 25, 2019
 *      Author: Adam
 */
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>


#define __vo  volatile
#define __weak __attribute__((weak))

/*************************** PROCESSOR SPECIFIC DETAILS *********************/

#define SET_BIT(REG, MASK)		((REG |= MASK))
#define CLEAR_BIT(REG, MASK)	((REG &= ~MASK))

/*
 * Base address of the NVIC
 */
#define NVIC_BASEADDR					0xE000E100

/*
 * NVIC Register structure
 * this should be in another header file specific to the m4 coretx (e.g. core_m4.h)
 */
typedef struct{
	__vo uint32_t ISER[8];		/*!< Interrupt Set-enable registers,		Address offset: 0x00-0x1C	*/
	uint32_t RESERVED0[24];		/*!< Reserved, 0x20-0x7C*/
	__vo uint32_t ICER[8];		/*!< Interrupt Set-clear registers,			Address offset: 0x80-0x9C	*/
	uint32_t RESERVED1[24];		/*!< Reserved, 0xA0-0xFC*/
	__vo uint32_t ISPR[8];		/*!< Interrupt Set-pending registers,		Address offset: 0x100-0x11C	*/
	uint32_t RESERVED2[24];		/*!< Reserved, 0x120-0x17C*/
	__vo uint32_t ICPR[8];		/*!< Interrupt Clear-pending registers,		Address offset: 0x180-0x19C	*/
	uint32_t RESERVED3[24];		/*!< Reserved, 0x1A0-0x1FC*/
	__vo uint32_t IABR[8];		/*!< Interrupt Active Bit registers,		Address offset: 0x200-0x21C	*/
	uint32_t RESERVED4[56];		/*!< Reserved, 0x220-0x2FC*/
	__vo uint8_t IPR[240];		/*!< Interrupt Priority registers,			Address offset: 0x300-0x3EF	*/
	uint32_t RESERVED5[644];	/*!< Reserved, 0x3F0-0xDFC*/
	__vo uint32_t STIR;			/*!< Software Trigger Interrupt register,	Address offset: 0xE00		*/
}NVIC_RegDef_t;

#define NVIC			((NVIC_RegDef_t*)	NVIC_BASEADDR)
//NOTE: the below is what is shown in the video,
// for consistency, I will use the method we have been using below (as shown above)
// In the generated code, these definitions would be stored in the CMSIS Include core_cm4.h file
///*
// * ARM Cortex-M4 Processor NVIC ISERx register addresses
// */
//#define NVIC_ISER0_BASEADDR			((__vo uint32_t*) 0xE000E100)
//#define NVIC_ISER1_BASEADDR			((__vo uint32_t*) 0xE000E104)
//#define NVIC_ISER2_BASEADDR			((__vo uint32_t*) 0xE000E108)
//#define NVIC_ISER3_BASEADDR			((__vo uint32_t*) 0xE000E10C)
//
///*
// * ARM Cortex-M4 Processor NVIC ICERx register addresses
// */
//#define NVIC_ICER0_BASEADDR			((__vo uint32_t*) 0xE000E180)
//#define NVIC_ICER1_BASEADDR			((__vo uint32_t*) 0xE000E184)
//#define NVIC_ICER2_BASEADDR			((__vo uint32_t*) 0xE000E188)
//#define NVIC_ICER3_BASEADDR			((__vo uint32_t*) 0xE000E18C)


/*
 * MCU specific Macros
 */
#define NO_PR_BITS_IMPLEMENTED		4



/*
 * Base addresses of FLASH and SRAM memories
 */
#define FLASH_BASEADDR					0x080000000U //Base address for the flash memory (size: 1024KB)
#define SRAM1_BASEADDR					0x200000000U //Base address to SRAM1 (Size: 112KB)
#define SRAM2_BASEADDR					0x2001C000U //Base address for SRAM 2 (SRAM1_BASEADDR + 112KB)
#define ROM_BASEADDR					0x1FFF0000U //System Memory (size: 30KB)
#define SRAM							SRAM1_BASEADDR

/*
 * Base addresses of peripheral buses
 */
#define PERIPH_BASEADDR					0x40000000U //Base address of peripherals
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR //Base address of ABP bus is peripheral base
#define APB2PERIPH_BASEADDR				0x40010000U //Base address of ABP bus
#define AHB1PERIPH_BASEADDR				0x40020000U //Base address of AHB1 bus
#define AHB2PERIPH_BASEADDR				0x50000000U // Base address of AHB2 bus

/*
 * Base addresses of peripherals hanging on the AHB1 bus
 * Only interested in GPIO
 */
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000UL)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400UL)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800UL)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00UL)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000UL)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400UL)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800UL)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2000UL)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00UL)

#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800UL)

/*
 * Base addresses of peripherals hanging on the APB1 bus
 * (Only interested in I2C, SPI, USART/UART
 */
#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800UL)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00UL)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400UL)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800UL)
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00UL)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000UL)

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400UL)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800UL)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00UL)

/*
 * Base addresses of peripherals hanging on the APB2 bus
 * (Only interested in SPI, USART/UART, EXTI, SYSCFG
 */
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000UL)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1C00UL)

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000UL)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400UL) //reserved

#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800UL)

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00UL)

/*****************************peripheral register definition structures*******************************/
/*
 * Note: Registers of a peripheral are specific to an MCU
 * e.g.: Numbers of registers of SPI peripheral of STM32F4x family MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your device RM
 */

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct {
	__vo uint32_t IMR;	/*!< EXTI interrupt mask register,				Address offset: 0x00	*/
	__vo uint32_t EMR;	/*!< EXTI event mask register,					Address offset: 0x4	*/
	__vo uint32_t RTSR;	/*!< EXTI rising trigger selection register,	Address offset: 0x08	*/
	__vo uint32_t FTSR;	/*!< EXTI falling trigger selection register,	Address offset: 0x0C	*/
	__vo uint32_t SWIER;/*!< EXTI software interrupt event register,	Address offset: 0x10	*/
	__vo uint32_t PR;	/*!< EXTI pending register,						Address offset: 0x14	*/
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct {
	__vo uint32_t MODER;	/*!< GPIO Port mode register,				Address offset: 0x00	*/
	__vo uint32_t OTYPER;	/*!< GPIO Port output type register,		Address offset: 0x04	*/
	__vo uint32_t OSPEEDR;	/*!< GPIO Port output speed register,		Address offset: 0x08	*/
	__vo uint32_t PUPDR;	/*!< GPIO Port pull-up/pull-down register,	Address offset: 0x0C	*/
	__vo uint32_t IDR;		/*!< GPIO Port input data register,			Address offset: 0x10	*/
	__vo uint32_t ODR;		/*!< GPIO Port output data register,		Address offset: 0x14	*/
	__vo uint32_t BSRR;		/*!< GPIO Port bit set/reset register,		Address offset: 0x18	*/
	__vo uint32_t LCKR;		/*!< GPIO Port configuration lock register,	Address offset: 0x1C	*/
	__vo uint32_t AFR[2];	/*!< GPIO Port alternate function registers,Address offset: 0x20-24	*/
} GPIO_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */
typedef struct {
	__vo uint32_t CR1;	/*!< I2C control register 1,		Address offset: 0x00	*/
	__vo uint32_t CR2;	/*!< I2C control register 2,		Address offset: 0x04	*/
	__vo uint32_t OAR1;	/*!< I2C own address register 1,	Address offset: 0x08	*/
	__vo uint32_t OAR2;	/*!< I2C own address register 2,	Address offset: 0x0C	*/
	__vo uint32_t DR;	/*!< I2C data register,				Address offset: 0x10	*/
	__vo uint32_t SR1;	/*!< I2C status register 1,			Address offset: 0x14	*/
	__vo uint32_t SR2;	/*!< I2C status register 2,			Address offset: 0x18	*/
	__vo uint32_t CCR;	/*!< I2C clock control register,	Address offset: 0x1C	*/
	__vo uint32_t TRISE;/*!< I2C TRISE register,			Address offset: 0x20	*/
	__vo uint32_t FLTR;	/*!< I2C FLTR register,				Address offset: 0x24	*/
} I2C_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct {
	__vo uint32_t CR;			/*!< RCC control register,											Address offset: 0x00	*/
	__vo uint32_t PLLCFGR;		/*!< RCC PLL configuration register,								Address offset: 0x04	*/
	__vo uint32_t CFGR;			/*!< RCC clock configuration register,								Address offset: 0x08	*/
	__vo uint32_t CIR;			/*!< RCC clock interrupt register,									Address offset: 0x0C	*/
	__vo uint32_t AHB1RSTR;		/*!< RCC AHB1 peripheral reset register,							Address offset: 0x10	*/
	__vo uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register,							Address offset: 0x14	*/
	__vo uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register,							Address offset: 0x18	*/
	uint32_t RESERVED0;			/*!< Reserved, 0x1C	*/
	__vo uint32_t APB1RSTR;		/*!< RCC ABP1 peripheral reset register,							Address offset: 0x20	*/
	__vo uint32_t APB2RSTR;		/*!< RCC ABP2 peripheral reset register,							Address offset: 0x24	*/
	uint32_t RESREVED1[2];		/*!< Reserved, 0x28-0x2C	*/
	__vo uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock register,							Address offset: 0x30	*/
	__vo uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock register,							Address offset: 0x34	*/
	__vo uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock register,							Address offset: 0x38	*/
	uint32_t RESERVED2;			/*!< Reserved, 0x3C	*/
	__vo uint32_t APB1ENR;		/*!< RCC ABP1 peripheral clock register,							Address offset: 0x40	*/
	__vo uint32_t APB2ENR;		/*!< RCC ABP2 peripheral clock register,							Address offset: 0x44	*/
	uint32_t RESERVED3[2];		/*!< Reserved, 0x48-0x4C	*/
	__vo uint32_t AHBLP1ENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50	*/
	__vo uint32_t AHBLP2ENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54	*/
	__vo uint32_t AHBLP3ENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58	*/
	uint32_t RESERVED4;			/*!< Reserved, 0x5C	*/
	__vo uint32_t ABPLP1ENR;	/*!< RCC ABP1 peripheral clock enable in low power mode register,	Address offset: 0x60	*/
	__vo uint32_t ABPLP2ENR;	/*!< RCC ABP2 peripheral clock enable in low power mode register,	Address offset: 0x64	*/
	__vo uint32_t RESERVED5[2];	/*!< Reserved, 0x68-0x6C	*/
	__vo uint32_t BDCR;			/*!< RCC backup domain control register,							Address offset: 0x70	*/
	__vo uint32_t CSR;			/*!< RCC clock control & status register,							Address offset: 0x74	*/
	__vo uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register,					Address offset: 0x78	*/
	__vo uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register,								Address offset: 0x84	*/
	__vo uint32_t PLLSAICFGR;	/*!< RCC PLL configuration register,								Address offset: 0x88	*/
	__vo uint32_t DCKCFGR;		/*!< RCC dedicated clock configuration register,	Address offset: 0x8C	*/
} RCC_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct {
	__vo uint32_t CR1;		/*!< SPI control register 1,			Address offset: 0x00	*/
	__vo uint32_t CR2;		/*!< SPI control register 2,			Address offset: 0x04	*/
	__vo uint32_t SR;		/*!< SPI status register,				Address offset: 0x08	*/
	__vo uint32_t DR;		/*!< SPI data register,					Address offset: 0x0C	*/
	__vo uint32_t CRCPR;	/*!< SPI CRC polynomial register,		Address offset: 0x10	*/
	__vo uint32_t RXCRCR;	/*!< SPI RX CRC register,				Address offset: 0x14	*/
	__vo uint32_t TXCRCR;	/*!< SPI TX CRC register,				Address offset: 0x18	*/
	__vo uint32_t I2SCFGR;	/*!< SPI_I2S configuration register,	Address offset: 0x1C	*/
	__vo uint32_t I2SPR;	/*!< SPI_I2S prescaler register,		Address offset: 0x20	*/
} SPI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct {
	__vo uint32_t MEMRMP;		/*!< SYSCFG memory remap register,						Address offset: 0x00	*/
	__vo uint32_t PMC;			/*!< SYSCFG peripheral mode configuration register,		Address offset: 0x04	*/
	__vo uint32_t EXTICR[4];	/*!< SYSCFG external interrupt configuration register,	Address offset: 0x08-0x14	*/
	uint32_t RESERVED[2];		/*!< Reserved, 0x18-0x1C	*/
	__vo uint32_t CMPCR;		/*!< SYSCFG compensation cell control register,			Address offset: 0x20	*/
} SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for USART
 */
typedef struct {
	__vo uint32_t SR;	/*!< USART status register,						Address offset: 0x00	*/
	__vo uint32_t DR;	/*!< USART data register,						Address offset: 0x04	*/
	__vo uint32_t BRR;	/*!< USART baud rate register,					Address offset: 0x08	*/
	__vo uint32_t CR1;	/*!< USART control register 1,					Address offset: 0x0C	*/
	__vo uint32_t CR2;	/*!< USART control register 2,					Address offset: 0x10	*/
	__vo uint32_t CR3;	/*!< USART control register 3,					Address offset: 0x14	*/
	__vo uint32_t GTPR;	/*!< USART guard time and prescaler register,	Address offset: 0x18	*/
} USART_RegDef_t;

/*
 * Peripheral definitions (peripheral base addresses typecasted as xxx_RegDef_t)
 */

#define EXTI				((EXTI_RegDef_t*) EXTI_BASEADDR)

#define GPIOA				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define I2C1				((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*) I2C3_BASEADDR)

#define RCC					((RCC_RegDef_t*) RCC_BASEADDR)

#define SPI1				((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*) SPI4_BASEADDR) //reserved

#define SYSCFG				((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define UART4				((USART_RegDef_t*) UART4_BASEADDR)
#define UART5				((USART_RegDef_t*) UART5_BASEADDR)
#define USART1				((USART_RegDef_t*) USART1_BASEADDR)
#define USART2				((USART_RegDef_t*) USART2_BASEADDR)
#define USART3				((USART_RegDef_t*) USART3_BASEADDR)
#define USART6				((USART_RegDef_t*) USART6_BASEADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock enable macros for I2C peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPI peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))//reserved

/*
 * Clock enable macros for USART peripherals
 */
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 14 ))

/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock disable macros for I2C peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPI peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))//reserved

/*
 * Clock disable macros for USART peripherals
 */
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)


#define GPIO_BASEADDR_TO_CODE(addr)		( (addr==GPIOA) ? 0 :\
										(addr==GPIOB) ? 1 :\
										(addr==GPIOC) ? 2 :\
										(addr==GPIOD) ? 3 :\
										(addr==GPIOE) ? 4 :\
										(addr==GPIOF) ? 5 :\
										(addr==GPIOG) ? 6 :\
										(addr==GPIOH) ? 7 : 0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)

/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21)); }while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22)); }while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23)); }while(0)

/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()			do{	(RCC->APB2RSTR |= (1<<4));	(RCC->APB2RSTR &= ~(1<<4));	}while(0)
#define USART2_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<17));	(RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART3_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<18));	(RCC->APB1RSTR &= ~(1<<18));}while(0)
#define UART4_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<19));	(RCC->APB1RSTR &= ~(1<<19));}while(0)
#define UART5_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<20));	(RCC->APB1RSTR &= ~(1<<20));}while(0)
#define USART6_REG_RESET()			do{	(RCC->APB2RSTR |= (1<<5));	(RCC->APB2RSTR &= ~(1<<5));	}while(0)

/*
 * Macros for IRQ numbers
 * maybe move these to a different section
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71

/*
 * macros for all possible NVIC priority levels
 * maybe move these to a different section
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


/***************************************************************************************
 * Bit position definitions for I2C peripheral
 ***************************************************************************************/

/*
 * Bit Definition Macros for I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/*
 * Bit Definition Macros for I2C_CR1
 */

#define I2C_CR2_FREQ		0 //8 bits long
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * Bit Definition Macros for I2C_OAR1
 */
#define I2C_OAR1_ADD0		0 // this is the LSB of the address in 10-bit address mode, NA in 7-bit mode
#define I2C_OAR1_ADD1		1 // 7 or 9 bits; this is the LSB of the address in 7-bit address mode, included in 10-bit mode, 2 MSB are ignored in 7-bit mode
#define I2C_OAR1_ADDMODE	15

/*
 * Bit Definition Macros for I2C_OAR2
 */
//#define I2C_OAR2_ENDUAL		0
//#define I2C_OAR2_ADD2		1 // 7 bits long

/*
 * Bit Definition Macros for I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 * Bit Definition Macros for I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8 //8 bits long

/*
 * Bit Definition Marcos for I2C_CCR
 */
#define I2C_CCR_CCR		0 //12 bits long
#define I2C_CCR_DUTY	14
#define I2C_CCR_FS		15

/*
 * Bit Definition Macros for I2C_TRISE
 */
//#define I2C_TRISE_TRISE		0 //6 bits

/*
 * Bit Definition Macros for I2C_FLTR
 */
//#define I2C_FLTR_DNF	0 //4 bits
//#define I2C_FLTR_ANOFF	4

/***************************************************************************************
 * Bit position definitions for SPI peripheral
 ***************************************************************************************/
/*
 * Bit Definition Macros for SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit Definition Macros for SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit Definition Macros for SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/***************************************************************************************
 * Bit position definitions for USART/UART peripheral
 ***************************************************************************************/
/*
 * Bit Definition Macros for USART_SR
 */
#define USART_SR_PE		0
#define USART_SR_FE		1
#define USART_SR_NF		2
#define USART_SR_ORE	3
#define USART_SR_IDLE	4
#define USART_SR_RXNE	5
#define USART_SR_TC		6
#define USART_SR_TXE	7
#define USART_SR_LBD	8
#define USART_SR_CTS	9

/*
 * Bit Definition Macros for USART_BRR
 */
#define USART_BRR_DIV_FRACTION	0
#define USART_BRR_DIV_MANTISSA	4

/*
 * Bit Definition Macros for USART_CR1
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 * Bit Definition Macros for USART_CR2
 */
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

/*
 * Bit Definition Macros for USART_CR3
 */
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		8
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

/*
 * Bit Definition Macros for USART_GTPR
 */
#define USART_GTPR_PSC	0
#define USART_GTPR_GT	8
/*
 * generic macros
 */
#define ENABLE		1
#define DISABLE		0
#define SET			ENABLE
#define RESET		DISABLE


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_usart_driver.h"

#endif /* INC_STM32F407XX_H_ */
