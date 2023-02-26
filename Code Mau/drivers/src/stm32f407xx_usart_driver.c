/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: May 28, 2020
 *      Author: Adam
 */

#include "stm32f407xx_usart_driver.h"

/*
 * Peripheral Clock setup
 */

/******************************************************
 * @fn					- USART_PeriClockControl
 *
 * @brief				- Enables or disables peripheral clock for the given USART port
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	} else {
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		} else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - initializes a given USART peripheral with given configurations
 *
 * @param[in]         - Struct holding base address and desired configurations of SPI peripheral
 *
 * @return            - none
 *
 * @Note              - this function is a good example about why the register values shouldn't just be positions

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//disable the specific USART (will be enabled when sending and receiving
	USART_PeripheralControl(pUSARTHandle->pUSARTx, DISABLE);

	//Temporary variable
	uint32_t tempreg=0;

	/******************************** Configuration of CR1******************************************/


	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2= tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/******************************************************
 * @fn					- USART_DeInit
 *
 * @brief				- De-initializes a given USART peripheral and resets register
 *
 * @param[in]			- base address of USART peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	} else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	} else if (pUSARTx == USART3)
	{
		USART3_REG_RESET();
	} else if (pUSARTx == UART4)
	{
		UART4_REG_RESET();
	} else if (pUSARTx == UART5)
	{
		UART5_REG_RESET();
	} else
	{
		USART6_REG_RESET();
	}
}


/*
 * Data Send and Receive
 */

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - send data of length len using USART in blocking mode
 *
 * @param[in]         - pointer to struct containing USART peripheral and configurations
 * @param[in]         - buffer containing data to send
 * @param[in]         - number of frames to send
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pdata;


	//if I2C is not enabled, then enable it
	if(! (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_UE)))
	{
		USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);
	}

   //Loop over until "len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

        //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		//should we actually check the register instead?
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twic
				//shouldn't we decrement length here?
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - receive data of length len using USART in blocking mode
 *
 * @param[in]         - pointer to struct containing USART peripheral and configurations
 * @param[in]         - buffer containing data to receive
 * @param[in]         - number of frames to receive
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	 //Loop over until "len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! (USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE)));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t) 0x01FF);

				//Now increment the pRxBuffer two times
				//why don't we decrement len here?
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = pUSARTHandle->pUSARTx->DR;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - send data of length len using USART in non-blocking mode
 *
 * @param[in]         - pointer to struct containing USART peripheral and configurations
 * @param[in]         - buffer containing data to send
 * @param[in]         - number of frames to send
 *
 * @return            - none
 *
 * @Note              - none
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txState = pUSARTHandle->TxBusyState;

	//do we not check for busy in RX because these can happen at the same time?
	if(txState != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		if(! (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_UE)))
		{
			USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);
		}

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txState;

}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - receive data of length len using USART in non-blocking mode
 *
 * @param[in]         - pointer to struct containing USART peripheral and configurations
 * @param[in]         - buffer containing data to receive
 * @param[in]         - number of frames to receive
 *
 * @return            - none
 *
 * @Note              - none
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t rxState = pUSARTHandle->RxBusyState;

	if(rxState != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		if(! (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_UE)))
		{
			USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);
		}

//		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxState;

}


/*
 * IRQ Configuration and ISR handling
 */

/******************************************************
 * @fn					- USART_IRQInterruptConfig
 *
 * @brief				- enables or disables a USART peripherals IRQ functionality
 *
 * @param[in]			- IRQ number of USART peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- this function should be made universal as it doesn't change depending on the peripheral
 * 						- or you could make a check for valid USART IRQ numbers
 * 						- (could be in a nvic .h file)
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi)
{
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
 * @fn					- USART_IRQPriorityConfig
 *
 * @brief				- sets the priority for a given IRQ number
 *
 * @param[in]			- IRQ priority of USART peripheral
 * @param[in]			- IRQ number of USART peripheral
 *
 * @return				- none
 *
 * @Note				- modifies the NVIC peripheral
 * 						- this also probably doesn't need to be a peripheral specific function (could be in a nvic .h file)
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//NOTE: because we used uint8_t for the IPR register, we can select the desired register directly
	NVIC->IPR[IRQNumber]  = IRQPriority << NO_PR_BITS_IMPLEMENTED; //shift by 4 is required because the lower 4 bits are not implemented
}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t controlBitIT, controlBit;
	uint16_t usartSR = pUSARTHandle->pUSARTx->SR;
	uint16_t *pdata;
/*************************Check for TC flag ********************************************/

	 //Implement the code to check the state of TCIE bit
	controlBitIT= pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);
	if((usartSR & USART_FLAG_TC) && controlBitIT )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(pUSARTHandle->TxLen == 0)
			{
				//Implement the code to clear the TC flag
				USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EV_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/


	//Implement the code to check the state of TXEIE bit in CR1
	controlBitIT= pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if( (usartSR & USART_FLAG_TXE) &&  controlBitIT)
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxLen--;
				}

			}if(pUSARTHandle->TxLen == 0)
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	controlBitIT = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if((usartSR & USART_FLAG_RXNE) && controlBitIT)
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen--;
				}


			} if(pUSARTHandle->RxLen == 0)
			{
				//RxLen < 0
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EV_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5


	//Implement the code to check the state of CTSE bit in CR1
	controlBit = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	controlBitIT = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if((usartSR & USART_FLAG_CTS) && controlBit && controlBitIT)
	{
		//Implement the code to clear the CTS flag in SR
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EV_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/


	//Implement the code to check the state of IDLEIE bit in CR1
	controlBitIT = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if((usartSR & USART_FLAG_IDLE) && controlBitIT)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EV_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/


	//Implement the code to check the status of RXNEIE  bit in the CR1
	controlBitIT = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if((usartSR & USART_FLAG_ORE)  && controlBitIT )
	{
		//No need to clear the ORE flag here, allow application to handle this

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ER_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	controlBitIT =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(controlBitIT)
	{

		if(usartSR & USART_FLAG_FE)
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ER_FE);
		}

		if(usartSR & USART_FLAG_NF )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ER_NF);
		}

		if(usartSR & USART_FLAG_ORE )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ER_ORE);
		}
	}
}


/*
 * Other Peripheral Control APIs
 */

/******************************************************
 * @fn					- USART_PeripheralControl
 *
 * @brief				- Enables or Disables USART
 *
 * @param[in]			- Pointer to a USART register
 * @param[in]			- enable or disable bit
 *
 * @return				- none
 *
 * @Note				- none (might implement this in the transmit function)
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/******************************************************
 * @fn					- USART_GetFlagStatus
 *
 * @brief				- Obtains value of a specific flag in the Status Register
 *
 * @param[in]			- Pointer to a USART register
 * @param[in]			- flag to access
 *
 * @return				- none
 *
 * @Note				- none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t flag){
	if(pUSARTx->SR & flag)
	{
		return SET;
	}
	return RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag)
{
	pUSARTx->SR &= ~(flag);
}

/*********************************************************************
 * @fn      		 	 - USART_SetBaudRate
 *
 * @brief            	 -
 *
 * @param[in]			- Pointer to a USART register
 * @param[in]        	- baud rate to be programmed for communication
 *
 * @return           	 -
 *
 * @Note             	 -  Note, multiplying by 100, so we don't have to deal with floating point numbers?
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t mantissa, fraction;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *baudRate));
	}else
	{
	   //over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 *baudRate));
	}

	//Calculate the Mantissa part
	mantissa = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= mantissa << 4;

	//Extract the fraction part
	fraction = (usartdiv - ( mantissa* 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	 {
	  //OVER8 = 1 , over sampling by 8
	  fraction = ((( fraction * 8) + 50) / 100)& ((uint8_t)0x07);

	 }else
	 {
	   //over sampling by 16
	   fraction = ((( fraction * 16) + 50) / 100) & ((uint8_t)0x0F);

	 }

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= fraction;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

