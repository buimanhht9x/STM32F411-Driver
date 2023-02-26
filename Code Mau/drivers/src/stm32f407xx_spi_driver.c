/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 17, 2019
 *      Author: Adam
 */


#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/**************************************************************************
 * 					APIs supported by this driver
 * For more information about these APIs check the function description
 **************************************************************************/
/*
 * Peripheral clock setup
 */

/******************************************************
 * @fn					- SPI_PeriClockControl
 *
 * @brief				- Enables or disables peripheral clock for the given SPI port
 *
 * @param[in]			- base address of SPI peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- SPI4 is reserved and won't be affected by this function
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
			if(pSPIx == SPI1){
				SPI1_PCLK_EN();
			} else if (pSPIx == SPI2){
				SPI2_PCLK_EN();
			} else if (pSPIx == SPI3){
				SPI3_PCLK_EN();
			}
		} else {
			if(pSPIx == SPI1){
				SPI1_PCLK_DI();
			} else if (pSPIx == SPI2){
				SPI2_PCLK_DI();
			} else if (pSPIx == SPI3){
				SPI3_PCLK_DI();
			}
		}
}

/*
 * Init and De-init
 */

/******************************************************
 * @fn					- SPI_Init
 *
 * @brief				- Initializes a given SPI peripheral with given configurations
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- I don't like the way this is done, but I'm doing it to be consistent with the course
 * 							Instead, I think the SPIConfig should hold masks, not enum type values
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	//enable the spi clock
	//should I check this is already enabled?
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//disable the specific SPI (concept taken from STM generated code) SPI will be enabled when sending or receiving
	SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);

	//first configure the SPI CR1 register
	uint32_t tempReg = 0;

	//1. configure the device mode
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER){
		// if the device is master mode, then the SSI must also be 1 to avoid an error (unless Multi master?)
		// this is only true if SSM is enabled, can we keep this functionality even is SSM is disabled?
		// based on stm generated code, the SSI stays enabled for SSM enabled and disabled
		tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
		tempReg |= (1 << SPI_CR1_SSI); // this should probably be done in another way
	} else {
		tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	}

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bdi should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bdi should be set
		tempReg |= ~(1 << SPI_CR1_BIDIMODE);
	} else {
		//bdi should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY should be set
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	//3. configure the clock speed
	tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. configure the Data frame format
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. configure the clock polarity
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. configure the clock phase
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. determine hardware or software slave management
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempReg;
}

/******************************************************
 * @fn					- SPI_DeInit
 *
 * @brief				- Deinitializes a given SPI peripheral and resets register
 *
 * @param[in]			- base address of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

/*
 * Data send and receive
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t flag){
	if(pSPIx->SR & flag){
		return SET;
	}
	return RESET;
}

/******************************************************
 * @fn					- SPI_SendData
 *
 * @brief				- sends data of length len to transmit buffer
 *
 * @param[in]			- base address of SPI peripheral
 * @param[in]			- address of Tx buffer
 * @param[in]			- length of data to send
 *
 * @return				- none
 *
 * @Note				- This is a blocking call
 */
//actual code seems much more complex, check that out
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t len){
	//if SPi is not enabled, then enable it
	if((pSPIx->CR1 & (1 << SPI_CR1_SPE)) != (1 << SPI_CR1_SPE)){
		//SET_BIT(pSPIx->CR1, (1 << SPI_CR1_SPE)); // this is repetitive, should just have a mask,
		SPI_PeripheralControl(pSPIx, ENABLE);
	}

	while(len > 0) {
		//1. Wait until Tx is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) ==  RESET);

		//2. Check DFF in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) {
			//16 bit dff
			//3. load data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		} else {
			//8 bit dff
			//3. load data into DR
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}

	}
}

/******************************************************
 * @fn					- SPI_ReceiveData
 *
 * @brief				- Receives data of length len to Receive buffer
 *
 * @param[in]			- base address of SPI peripheral
 * @param[in]			- address of Rx buffer
 * @param[in]			- length of data to Receive
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t len){
	//if SPi is not enabled, then enable it
	if((pSPIx->CR1 & (1 << SPI_CR1_SPE)) != (1 << SPI_CR1_SPE)){
		SET_BIT(pSPIx->CR1, (1 << SPI_CR1_SPE)); // this is repetitive, should just have a mask,
	}

	while(len > 0) {
		//1. Wait until Rx is not empty
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) ==  RESET);

		//2. Check DFF in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) {
			//16 bit dff
			//3. load data from the DR to RxBuffer address
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t*)pRxBuffer++;
		} else {
			//8 bit dff
			//3. load data from the DR to RxBuffer address
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}

	}
}

/******************************************************
 * @fn					- SPI_SendDataIT
 *
 * @brief				- Sends data of length len in non-blocking mode
 *
 * @param[in]			- pointer to SPI handle structure
 * @param[in]			- address of Tx buffer
 * @param[in]			- length of data to Send
 *
 * @return				- none
 *
 * @Note				- none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		//1.	Save the Tx Buffer address and len information in some variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2.	Mark the API state as busy in transmission so that
		//		no other code can take over same API peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3.	Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE); //should I also enable error interrupt?
//		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_ERRIE)

		//if SPi is not enabled, then enable it
		//I think this should be in the if, because, otherwise it is assumed that it is enabled
		if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_SPE)) != (1 << SPI_CR1_SPE)){
			//SET_BIT(pSPIx->CR1, (1 << SPI_CR1_SPE)); // this is repetitive, should just have a mask,
			SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);
		}
	}


	//why return state here and not pSPIHandle->TxState?
	//if the TxState is not busy, then we configure to send as an intterupt, which presumably executes
	//in this case, state will return as SPI_NOT_BUSY
	//this is intended because this function will only return once the data is all sent
	//if TxState is busy, then this will return, that it is busy, which will allow the while loop to try again
	return state;
}

/******************************************************
 * @fn					- SPI_ReceiveDataIT
 *
 * @brief				- Receives data of length len in non-blocking mode
 *
 * @param[in]			- pointer to SPI handle structure
 * @param[in]			- address of Rx buffer
 * @param[in]			- length of data to Send
 *
 * @return				- none
 *
 * @Note				- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		//1.	Save the Tx Buffer address and len information in some variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		//2.	Mark the API state as busy in transmission so that
		//		no other code can take over same API peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3.	Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE); //should I also enable error interrupt?

		//if SPi is not enabled, then enable it
		if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_SPE)) != (1 << SPI_CR1_SPE)){
			//SET_BIT(pSPIx->CR1, (1 << SPI_CR1_SPE)); // this is repetitive, should just have a mask,
			SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);
		}
	}



	return state; //should we return this variable or the struct state variable? will this be consistent with the RxState?
}

/*
 * IRQ configuration and handling
 */

/******************************************************
 * @fn					- SPI_IRQInterruptConfig
 *
 * @brief				- enables or disables a SPI peripherals IRQ functionality
 *
 * @param[in]			- IRQ number of SPI peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi){
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
 * @fn					- SPI_IRQPriorityConfig
 *
 * @brief				- sets the priority for a given IRQ number
 *
 * @param[in]			- IRQ priority of SPI peripheral
 * @param[in]			- IRQ number of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- modifies the NVIC peripheral
 */
void SPI_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNumber){
	//NOTE: because we used uint8_t for the IPR register, we can select the desired register directly
	NVIC->IPR[IRQNumber]  = IRQPriority << NO_PR_BITS_IMPLEMENTED; //shift by 4 is required because the lower 4 bits are not implemented
}

/******************************************************
 * @fn					- SPI_IRQHandling
 *
 * @brief				- deals with a interrupt triggered by a SPI pin
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- Usually check for errors as well, but we aren't concerned at this time
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t status, control; //change these?

	//first check for TXE
	status = pSPIHandle->pSPIx->SR & SPI_FLAG_TXE; // will be 1 if TXE is set
	control = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE); // will be 1 if TXEIE is set

	if ( status && control ) {
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// check for RXNE
	status = pSPIHandle->pSPIx->SR & SPI_FLAG_RXNE; // will be 1 if RXNE is set
	control = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE); // will be 1 if RXNEIE is set

	if ( status && control ) {
		//handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// check for Overun flag
	status = pSPIHandle->pSPIx->SR & SPI_FLAG_OVR; // will be 1 if OVR is set
	control = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE); // will be 1 if ERRIE is set

	if ( status && control ) {
		//handle TXE
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

/*
 * Other peripheral control APIs
 */

/******************************************************
 * @fn					- SPI_PeripheralControl
 *
 * @brief				- Enables or Disables SPI
 *
 * @param[in]			- Pointer to a SPI register
 *
 * @return				- none
 *
 * @Note				- I'm implementing this in the transmit function
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/******************************************************
 * @fn					- SPI_SSIConfig
 *
 * @brief				- Sets or Resets SSI bit in SPI CR1 reg
 *
 * @param[in]			- Pointer to a SPI register
 *
 * @return				- none
 *
 * @Note				- This code is used in the course I'm following,
 * 						but I think it is redundant and bad practice. It will not be used.
 * 						Instead, I implemented a check in the SPI init function to enable SSI w/ master
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/******************************************************
 * @fn					- SPI_SSOEConfig
 *
 * @brief				- Sets or Resets SSOE bit in SPI CR2 reg
 *
 * @param[in]			- Pointer to a SPI register
 *
 * @return				- none
 *
 * @Note				- This code is used in the course I'm following,
 * 						but I think it is redundant and bad practice. I will use it bec of consistency
 * 						Otherwise, in the STM generated code, this bit is determine by a NSS value in the init struct
 * 						See: WRITE_REG(hspi->Instance->CR2, (((hspi->Init.NSS >> 16U) & SPI_CR2_SSOE) | hspi->Init.TIMode));
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->TxLen = 0;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->RxLen = 0;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent){

	//This is a weak implementation. The application may override this function.
}

//helper function implementations
//this will run when TXE and TXEIE are 1
//AS long as the TxLen is above 0, then this will get called again as soon as the DR is empty
//When TxLen is 0, then disable TXEIE, reset handle variables, and Set SPI_READY again
//We then use SPI_ApplicationEventCallback to inform the application that it is done
//Note: when we close transmission, pSPIHandle->pRxBuffer is set to  NULL
//however, the buffer that this originally pointed to will still contain the value read
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	// Check DFF in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) {
		//16 bit dff
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {
		//8 bit dff
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen){
		//TxLen is zero, so close the spi communication
		//and inform app that TX is over
		//this precents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	// Check DFF in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) {
		//16 bit dff
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {
		//8 bit dff
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen){
		//TxLen is zero, so close the spi communication
		//and inform app that TX is over
		//this precents interrupts from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//1. clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp;
	//If the SPI is busy in TX, then the error will not be cleared and the application will have to clear bit on own
	//In this case, application should call SPI_ClearOVRFlag
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
