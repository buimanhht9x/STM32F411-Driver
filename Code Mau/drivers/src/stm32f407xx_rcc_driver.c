/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Apr 30, 2020
 *      Author: Adam
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB_PreScaler[4] = {2,4,8,16};

/******************************************************
 * @fn					- RCC_GetPLLOutputClock
 *
 * @brief				- returns the value of PLL output clock
 *
 * @return				- none
 *
 * @Note				- Currently not implemented as the PLL clock is not used
 */
uint32_t RCC_GetPLLOutputClock(void){
	return 0;
}


/******************************************************
 * @fn					- RCC_GetPCLK1Value
 *
 * @brief				- calculates the value of PCLK 1 using the system clock, ahb and apb1 prescaler
 *
 * @return				- 32 bit PCLK value in Hz
 *
 * @Note				- none
 */
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, systemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);//0's out any bits that we don't want. Probably should create masks/pos vars for this

	if(clksrc == 0){
		systemClk = 16000000;//HSI
	} else if(clksrc == 1){
		systemClk = 8000000;//HSE
	} else if(clksrc == 2){
		systemClk = RCC_GetPLLOutputClock();//PLL (not implemented)
	}

	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB_PreScaler[temp-4];
	}

	pclk1 = (systemClk/ahbp)/apb1p;

	return pclk1;
}


uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t tmp,pclk2;
	uint32_t systemClk = 0;
	uint8_t clksrc = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clksrc == 0)
	{
		systemClk = 16000000;
	}else
	{
		systemClk = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB_PreScaler[tmp-4];
	}

	pclk2 = (systemClk / ahbp )/ apb2p;

	return pclk2;
}
