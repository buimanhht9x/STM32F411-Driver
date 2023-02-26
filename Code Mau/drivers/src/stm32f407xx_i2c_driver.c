/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: Apr 16, 2020
 *      Author: Adam
 */


//TODO: Implement timeouts for master/slave Tx and Rx

#include "stm32f407xx_i2c_driver.h"

#define I2C_MAX_TRISE_SM	1000
#define I2C_MAX_TRISE_FM	300

#define I2C_WRITE	0
#define I2C_READ	1


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t readOrWrite);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_ClearSTOPFFlag(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEIT(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEIT(I2C_Handle_t *pI2CHandle);

/*
 * I2C Static Functions
 */

/******************************************************
 * @fn					- I2C_GenerateStartCondition
 *
 * @brief				- sets the CR1 START bit to generate the start condition,
 *
 * @param[in]			- base address of I2C peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}



/******************************************************
 * @fn					- I2C_ExecuteAddressPhaseWrite
 *
 * @brief				- loads the address + Write bit into the DR
 *
 * @param[in]			- base address of I2C peripheral
 * @param[in]			- 8 bit address of the slave
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t readOrWrite){
	//make space for the R/nW bit
	slaveAddr = slaveAddr << 1;
	if (readOrWrite == I2C_READ){
		slaveAddr |= 1;//slave address is address plus Read bit (1)
	} else {
		slaveAddr &= ~(1);//slave address is address plus Write bit	(0)
	}
	pI2Cx->DR = slaveAddr;
}


/******************************************************
 * @fn					- I2C_ClearADDRFlag
 *
 * @brief				- dummy reads from the SR1 and SR2 registers to clear the ADDR bit
 *
 * @param[in]			- base address of I2C peripheral
 *
 * @return				- none
 *
 * @Note				- why not use the GetFlagStatus function? we don't need a specific flag
 *						- because we read SR1 before we call this, could this just read SR2? do we need a function?
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx){
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;//type cast to void so the compiler doesn't throw and unused warning
}

static void I2C_ClearSTOPFFlag(I2C_RegDef_t *pI2Cx){
	uint32_t dummyRead;
	uint16_t dummyWrite = (1 << I2C_CR1_PE);
	dummyRead = pI2Cx->SR1;
	(void)dummyRead;//type cast to void so the compiler doesn't throw and unused warning
	pI2Cx->CR1 |= dummyWrite;//need to write to the CR1 to clear the flag. taken from STM generated code
}

/*
 * I2C non-static functions
 */

/*
 * Peripheral clock setup
 */

/******************************************************
 * @fn					- I2C_PeriClockControl
 *
 * @brief				- Enables or disables peripheral clock for the given I2C port
 *
 * @param[in]			- base address of I2C peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
			if(pI2Cx == I2C1){
				I2C1_PCLK_EN();
			} else if (pI2Cx == I2C2){
				I2C2_PCLK_EN();
			} else if (pI2Cx == I2C3){
				I2C3_PCLK_EN();
			}
		} else {
			if(pI2Cx == I2C1){
				I2C1_PCLK_DI();
			} else if (pI2Cx == I2C2){
				I2C2_PCLK_DI();
			} else if (pI2Cx == I2C3){
				I2C3_PCLK_DI();
			}
		}
}

/*
 * Init and De-init
 */

/******************************************************
 * @fn					- I2C_Init
 *
 * @brief				- Initializes a given I2C peripheral with given configurations
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- I don't like the way this is done, but I'm doing it to be consistent with the course
 * 							Instead, I think the SPIConfig should hold masks
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){

	//enable the I2C clock
	//should I check this is already enabled?
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//disable the specific I2C (concept imitated from STM SPI generated code) I2C will be enabled when sending or receiving
	I2C_PeripheralControl(pI2CHandle->pI2Cx, DISABLE);

	uint32_t tempreg = 0;

	//setting this won't do anything because the peripheral is not enabled
	//ack control bit
//	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
//	pI2CHandle->pI2Cx->CR1 = tempreg; // equals or |=

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;//divide by 1 MHz to get 16 (not 16 mil)
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); // mask all bits except first 5

	//configure the devices address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1; //assuming 7-bit slave address
	//if using 10-bit mode, address will not be shifted, and bit 15 of will need to be set
	tempreg |= (1 << 14); // this bit needs to be set according to RM. replace 14 with variable
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccrValue = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <=  I2C_SCL_SPEED_SM){
		//mode is standard mode
		ccrValue = RCC_GetPCLK1Value() / (2 *pI2CHandle->I2C_Config.I2C_SCLSpeed); // CCR = FPCLK1/2*FSCL (frequency)
		tempreg |= ccrValue & 0xFFF; //only use 12 bits (CCR->CCR requirement
	} else {
		//mode is fast mode
		tempreg |= (1 << 15); //program the mode
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14); // set the duty cycle
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccrValue = RCC_GetPCLK1Value() / (3 *pI2CHandle->I2C_Config.I2C_SCLSpeed); // CCR = FPCLK1/3*FSCL (frequency)
		} else {
			ccrValue = RCC_GetPCLK1Value() / (25 *pI2CHandle->I2C_Config.I2C_SCLSpeed); // CCR = FPCLK1/25*FSCL (frequency)
		}
		tempreg |= ccrValue;
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//configure TRISE
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <=  I2C_SCL_SPEED_SM){
		//mode is standard mode
		tempreg = ((RCC_GetPCLK1Value()*I2C_MAX_TRISE_SM)/1000000000U) + 1;//take into account ns
	} else {
		//mode is fast mode
		tempreg = ((RCC_GetPCLK1Value()*I2C_MAX_TRISE_FM)/1000000000U) + 1;// take into account ns
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/******************************************************
 * @fn					- I2C_DeInit
 *
 * @brief				- De-initializes a given I2C peripheral and resets register
 *
 * @param[in]			- base address of I2C peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_DeInit(I2C_RegDef_t* pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2){
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}



/*
 * Data send and receive
 */

/******************************************************
 * @fn					- I2C_MasterSendData
 *
 * @brief				- Sends data over I2C (theoretically in blocking mode?)
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 * @param[in]			- Buffer containing data to be sent
 * @param[in]			- length of data to be sent
 * @param[in]			- 8 bit address of the slave
 * @param[in]			- value determining repeated start generation
 *
 * @return				- none
 *
 * @Note				- what about sending multiple bytes with a repeated start?
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repStart) {

	//if I2C is not enabled, then enable it
	if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
		I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
	}

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//	Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, I2C_WRITE);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	//	Note: until ADDR is cleared SCL will be stretched (pulled to LOW)
	// we already read SR1 above, so all we really need is a read to SR2, is this really worth another function?
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send the data until len becomes 0
	while( len > 0) {
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_TXE)); // Wait until TXE is set
		pI2CHandle->pI2Cx->DR= *pTxBuffer;
		pTxBuffer++;
		len--;

		//check for underrun
		if( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_SR1_BTF) && len >0 )
		{
			//write data into DR
			pI2CHandle->pI2Cx->DR = *pTxBuffer;
			pTxBuffer++;
			len--;
		}
	}

	//7. when len becomes 0, wait for TXE=1 and BTF=1 before generating the STOP condition
	//	Note: TXE=1, BTF=1, means that both SR and SR are empty and next transmission should begin
	//	when BTF=1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	//	Note: generating STOP, automatically clears the BTF
	if (repStart == I2C_SR_DISABLED){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/******************************************************
 * @fn					- I2C_MasterReceiveData
 *
 * @brief				- Sends data over I2C (theoretically in blocking mode?)
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 * @param[in]			- Buffer to store received data
 * @param[in]			- length of data to be received
 * @param[in]			- 8 bit address of the slave
 * @param[in]			- value determining repeated start generation
 *
 * @return				- none
 *
 * @Note				- In comparison to the STM generated code, this code does not seem very robust
 * 						- There are no checks for BTS, and it does not utilize the POS bit
 * 						- I should read the reference manual to fully understand the I2C process
 * 						- also, dealing with 3 bytes should be done
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repStart){
	//if I2C is not enabled, then enable it
	if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
		I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
	}

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//	 NOTE: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_SB));

	//3, Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, I2C_READ);

	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if (len == 1){

		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RxNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_RXNE));

		//generate STOP condition
		if (repStart == I2C_SR_DISABLED){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;


	} else if (len > 1) {//proceed to read data from slave when len > 1
		//clear the ADDR flag
		//Should we be checking the BTF flag?
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//enable acking, this mimics STM code, not the course
		// this isn't for len = 1 because it is not needed for 1 byte
		//should I check to confirm we want acking enabled? I think this should be default
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

		while (len > 0) {

			//wait until RxNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_RXNE));

			if (len == 2){
				//should we check for the BTF flag? BTF set when data in DR and in Shift register
				//I think no because rxne =1 means dr has dataN-1, so whether or not we dataN is in shift reg,
				//we can disable the acking?
				//unless somehow the stop condition is sent before that bit?
//				while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_BTF));

				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);


//				//generate STOP condition
//				//should this be moved to after len == 0?
				if (repStart == I2C_SR_DISABLED){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read data in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;
			len--;

			//check for overrun error
			if( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_BTF) && len > 0 )
			{
				//read data in to buffer
				*pRxBuffer = pI2CHandle->pI2Cx->DR;
				//increment the buffer address
				pRxBuffer++;
				len--;
			}
		}
	}
	//FOR FUTURE: This shouldn't happen here because STOP should be enabled before the last data is read
	//occurs when len == 0
	//originally occurs during len==2 if statement, but I moved it here because I think it makes more sense
	//generate STOP condition
//	if (repStart == I2C_SR_DISABLED){
//		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
//	}

	//re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}


/******************************************************
 * @fn					- I2C_SlaveSendData
 *
 * @brief				- sends data in slave mode (only sends 1 byte at a time)
 *
 * @param[in]			- pointer to I2C register definition
 * @param[in]			- byte of data to send
 *
 * @return				- none
 *
 * @Note				- TODO: implement timeout and error checking
 * 						-		mainly to check for ack failure while length is still greater than 0
 */
//void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){
//	//if I2C is not enabled, then enable it
//	if(! (pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
//		I2C_PeripheralControl(pI2Cx, ENABLE);
//	}
//	pI2Cx->DR = data;
//}
void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len)
{
	//if I2C is not enabled, then enable it
	if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
		I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
	}

	//1. Disable POS
	CLEAR_BIT(pI2CHandle->pI2Cx->CR1, (1 << I2C_CR1_POS));

	//2. Enable Address acknowledge
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

	//3. Wait for address phase is complete
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_ADDR));
	//4. clear addr flag to begin sending data
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//5. Send data until len becomes 0
	while(len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_TXE));

		//load buffer data into DR
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		//increment the buffer address
		pTxBuffer++;
		len--;

		//check for underrun
		if( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_SR1_BTF) && len >0 )
		{
			//write data into DR
			pI2CHandle->pI2Cx->DR = *pTxBuffer;
			pTxBuffer++;
			len--;
		}
	}

	//6. Wait for AF flag to end communication
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_AF));

	//7. Clear the AF flag; write 0 in AF bit of SR1
	CLEAR_BIT(pI2CHandle->pI2Cx->SR1, I2C_FLAG_AF);

	//8. Disable Address acking
	I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
}

/******************************************************
 * @fn					- I2C_SlaveReceiveData
 *
 * @brief				- Receives data in slave mode (only receives 1 byte at a time)
 *
 * @param[in]			- pointer to I2C register definition
 *
 * @return				- DR register contents
 *
 * @Note				- TODO: add error/timeout checking
 * 						- TODO implement a timeout so we can recover from trying to receive the wrong length of data
 */
//uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
//
//
//	//if I2C is not enabled, then enable it
//	if(! (pI2Cx->CR1 & (1 << I2C_CR1_PE) ) )
//	{
//		I2C_PeripheralControl(pI2Cx, ENABLE);
//	}
//
//	return pI2Cx->DR;
//}

void I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len)
{
	//if I2C is not enabled, then enable it
	if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
		I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
	}

	//enable acking
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

	//1. Wait for address phase is complete
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_ADDR));
	//2. clear addr flag to begin receiving data
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	while(len > 0){
		//3. Wait for RXNE to be set
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_RXNE));

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		//increment the buffer address
		pRxBuffer++;
		len--;

		//check for overrun error
		if( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_BTF) && len > 0 )
		{
			//read data in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;
			len--;
		}
	}

	//4. Wait for STOPF flag to end communication
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1, I2C_FLAG_STOPF));
	//5. Clear the STOPF flag
	I2C_ClearSTOPFFlag(pI2CHandle->pI2Cx);

	//disnable ACKing
	I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
}

/*********************************************************************
 * @fn					- I2C_MasterSendDataIT
 *
 * @brief 				- Enables sending of data over I2C in non-blocking mode
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 * @param[in]			- Buffer containing data to send
 * @param[in]			- length of data to be received
 * @param[in]			- 8 bit address of the slave
 * @param[in]			- value determining repeated start generation
 *
 * @return				- state of I2C peripheral
 *
 * @Note				- Should this enable acking?
 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repStart) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->repStart = repStart;

		//if I2C is not enabled, then enable it
		if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
			I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
		}

		//Implement code to Generate START Condition
		//In course code, this is above the interrupt enables, but I moved it here just in case it succeeds before they're enabled
		//EDIT: This should be before interrupt enables because for the start condition clears the TXE flag
		//		If TXE isn't cleared before interrupts are enabled,
		//		then the program will continually interrupt
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;

}

/*********************************************************************
 * @fn					- I2C_MasterReceiveDataIT
 *
 * @brief 				- Enables reception of data over I2C in non-blocking mode
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 * @param[in]			- Buffer to store received data
 * @param[in]			- length of data to be received
 * @param[in]			- 8 bit address of the slave
 * @param[in]			- value determining repeated start generation
 *
 * @return				- state of I2C peripheral
 *
 * @Note				- Should this enable acking?
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t repStart) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->repStart = repStart;

		//if I2C is not enabled, then enable it
		if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
			I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
		}

		//Implement code to Generate START Condition
		//In course code, this is above the interrupt enables, but I moved it here just in case it succeeds before they're enabled
		//EDIT: This should be before interrupt enables because for the start condition clears the TXE flag
		//		If TXE isn't cleared before interupts are enabled,
		//		then the program will continually interrupt
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

uint8_t I2C_SlaveSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;

		//if I2C is not enabled, then enable it
		if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
			I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
		}

		//enable address acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


//NOTE: Do we need a length here?
//Yes: we probably don't want the master to be able to send unlimited data

uint8_t I2C_SlaveReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception

		//if I2C is not enabled, then enable it
		if(! ( pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_PE) ) ){
			I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
		}

		//enable address acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}
/*
 * IRQ configuration and handling
 */

/******************************************************
 * @fn					- I2C_IRQInterruptConfig
 *
 * @brief				- enables or disables a I2C peripherals IRQ functionality
 *
 * @param[in]			- IRQ number of I2C peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi){
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
 * @fn					- I2C_IRQPriorityConfig
 *
 * @brief				- sets the priority for a given IRQ number
 *
 * @param[in]			- IRQ priority of I2C peripheral
 * @param[in]			- IRQ number of I2C peripheral
 *
 * @return				- none
 *
 * @Note				- modifies the NVIC peripheral
 */
void I2C_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNumber){
	//NOTE: because we used uint8_t for the IPR register, we can select the desired register directly
	NVIC->IPR[IRQNumber]  = IRQPriority << NO_PR_BITS_IMPLEMENTED; //shift by 4 is required because the lower 4 bits are not implemented
}

/******************************************************
 * @fn					- I2C_MasterHandleTXEIT
 *
 * @brief				- Handles the sending of data on a TXE interrupt
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_MasterHandleTXEIT(I2C_Handle_t *pI2CHandle){
	//we have to transmit data
	if(pI2CHandle->TxLen > 0){
		//1. load the data in to the DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
	//NOTE: We don't need to handle the end here because BTF IT will do this
}

/******************************************************
 * @fn					- I2C_MasterHandleRXNEIT
 *
 * @brief				- Handles the sending of data on a RXNE interrupt
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- The STM code seems much more robust, study that in order to gain full understanding
 * 						- for transmitting, we let the BTF interrupt handle the stop condition
 * 						- I think we should also let BTF interrupt handle the stop condition for receiving
 *
 */
static void I2C_MasterHandleRXNEIT(I2C_Handle_t *pI2CHandle){
	//data reception
	if(pI2CHandle->RxSize == 1){
		//send stop before reading the last byte
//		if(pI2CHandle->repStart == I2C_SR_DISABLED){
//			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
//		}

		//1. load the data in to the DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		//2. decrement the TxLen
		pI2CHandle->RxLen--;
	} else if (pI2CHandle->RxSize > 1){
		//enable acking, this mimics STM code, not the course
		// this isn't for len = 1 because it is not needed for 1 byte
		if (!(pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR1_ACK)) && (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)){
			//acking isn't enabled, but should be
			I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
		}

		if (pI2CHandle->RxLen == 2){
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
			//don't need to wait for btf because dr has dataN-1 and we don't want to ack dataN
			//send stop before reading the last two bytes
//			if(pI2CHandle->repStart == I2C_SR_DISABLED){
//					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
//				}
		}

		//1. load the data in to the DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		//2. decrement the TxLen
		pI2CHandle->RxLen--;
		//3. increment the buffer address
		pI2CHandle->pRxBuffer++;
	}

	if(pI2CHandle->RxLen == 0){
		//instructor code has this here, but according to the RM, this should be sent when there are 2 bytes left
		//to make the stop conditions above work, instead of the one below, must change the line
		//above from pI2CHandle->RxLen == 2 to pI2CHandle->RxLen == 1
		if(pI2CHandle->repStart == I2C_SR_DISABLED){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		//close the I2C data reception and notify the application
		//2. Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_MASTER_RX_CMPLT);
	}
}

static void I2C_SlaveHandleTXEIT(I2C_Handle_t *pI2CHandle)
{
	//we have to transmit data
	if(pI2CHandle->TxLen > 0){
		//1. load the data in to the DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_SlaveHandleRXNEIT(I2C_Handle_t *pI2CHandle)
{
	//we have to receive data
	if(pI2CHandle->RxLen > 0){
		//1. load the data in to the DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		//2. decrement the TxLen
		pI2CHandle->RxLen--;

		//3. increment the buffer address
		pI2CHandle->pRxBuffer++;
	}
}

/******************************************************
 * @fn					- I2C_CloseReceiveData
 *
 * @brief				- Resets Interrupt Controls and handle variables for rx
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	//Would it makes sense to disable the error interrupts?
	//Implement the code to disable the ITERREN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	//do we need this here if we are enabling acking else where?
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/******************************************************
 * @fn					- I2C_CloseSendData
 *
 * @brief				- Resets Interrupt Controls and handle variables for tx
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	//Would it makes sense to disable the error interrupts?
	//Implement the code to disable the ITERREN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/******************************************************
 * @fn					- I2C_EV_IRQHandling
 *
 * @brief				- Handles an I2C interrupt event
 *
 * @param[in]			- Struct holding base address and desired configurations of SPI peripheral
 *
 * @return				- none
 *
 * @Note				- should check for multiple interrupts at once (as coded below)?
 * 						- ANS: No, code changed to use else if instead of if, (based on STM generated code)
 * 						- STM code seems a lot better than course code
 * 						- uses TRA to determine whether or not to check TXE/RXNE and BTF
 * 						- BTF receive and transmit have their own functions
 * 						- STM code disables the BUF it in reception when data is at 3 bits and relies on BTF ITs to finish reception
 * 						-TODO: implement slave transfer and receive so this doesn't have to occur in the callback code
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Should probably make temporary variables to represent the status register

	//Interrupt handling for both master and slave mode of device
	uint32_t eventIT, bufferIT;
	uint16_t sr1IT, sr2IT;

	eventIT = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	bufferIT = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	sr1IT= pI2CHandle->pI2Cx->SR1;
	sr2IT= pI2CHandle->pI2Cx->SR2;

	//ITEVTEN is enabled

	//1. Handle for interrupt generated by SB event
	//	Note: SB flag is only applicable in Master mode
	if(eventIT && (sr1IT & I2C_FLAG_SB) )
	{
		//Interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block, let's execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_WRITE);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_READ);
		}
	}

	//2. Handle for interrupt generated by ADDR event
	//	Note:	When Master mode: Address is sent
	//			When Slave mode : Address matched with own address
	//			Disable acking if in receiving mode and one byte is received
	// bug in the insturctors code: doesn't clear addr if rxSize is > 1
	else if(eventIT && (sr1IT & I2C_FLAG_ADDR) )
	{
		//ADDR flag is set
		if(sr2IT & I2C_FLAG_MSL)
		{
			//device is master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				//device is Receiving
				if(pI2CHandle->RxSize == 1)
				{
					//only receiving one byte
					//first disable ack
					I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				}
			}
		}
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	}

	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	else if(eventIT && (sr1IT & I2C_FLAG_BTF) )
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(sr2IT & I2C_FLAG_MSL ){
				//make sure that TXE is also set
				if( sr1IT & I2C_FLAG_TXE )
				{
					//BTF, TXE both 1
					if(pI2CHandle->TxLen == 0)
					{
						//1. generate STOP condition
						if(pI2CHandle->repStart == I2C_SR_DISABLED)
						{
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
						}

						//2. rest all member elements of the handle struct
						I2C_CloseSendData(pI2CHandle);

						//3. notify the application about transmission complete
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_MASTER_TX_CMPLT);
					}
				}
			}
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//do nothing in this situation
			//why? don't we need to generate the stop condition for Rx???
			// I think because BTF is set when the DR and shift register are full, which may happen frequently
			//we don't want the communication to end in that situation
			// but we could still handle that here by checking the rx len?
			;
		}
	}

	//4. Handle for interrupt generated by STOPF event
	//	Note: Stop detection flag is applicable only slave mode. For master this flag will never be set
	//RM recommends going through entire process of read SR1 and write CR1 to clear after STOPF is set
	//ADDR flag should also be cleared
	else if(eventIT && (sr1IT & I2C_FLAG_STOPF ))
	{
		//close data reception
		I2C_CloseReceiveData(pI2CHandle);

		//STOPF flag is set
		//Clear the STOPF i.e.[ 1) read SR1 2) Write to CR1 ]
		//Already read SR1 above, do we need to write something specific?
		//NOTE: The RM states that for either STOPF or ADDR set evetns, both STOPF and ADDR should be checked and cleared
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		I2C_ClearSTOPFFlag(pI2CHandle->pI2Cx);

		//disable acking
		//acking might be enabled in close receivedata, should check if this is necessary
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
//		pI2CHandle->pI2Cx->CR1 |= 0x0000;

//		//notify the appliaction that RX is complete is detected
		//should probably implement more checking to confirm this occurs at the correct time.
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX || pI2CHandle->RxSize == 0){
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_SLAVE_RX_CMPLT);
		}
		//else throw an exception probably
	}

	//5. Handle for interrupt generated by TXE event
	else if(eventIT && bufferIT && (sr1IT & I2C_FLAG_TXE))
	{
		//check for device mode
		if(sr2IT & I2C_FLAG_MSL )
		{
			//device mode is master
			//TXE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEIT(pI2CHandle);
			}
		} else
		{
			//device mode is slave
			//should we check TRA in master mode too?
			if( sr2IT & I2C_FLAG_TRA && pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				//device is transmitting
				I2C_SlaveHandleTXEIT(pI2CHandle);
//				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}

		}
	}

	//6. Handle for interrupt generated by RXNE event
	else if(eventIT && bufferIT && (sr1IT & I2C_FLAG_RXNE) ){
		//check for device mode
		if(sr2IT & I2C_FLAG_MSL){
			//device mode is master
			//RXNE is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEIT(pI2CHandle);
			}
		} else
		{
			//device mode is slave
			if((! (sr2IT & I2C_FLAG_TRA)) && (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) )
			{
				//device is receiving
				I2C_SlaveHandleRXNEIT(pI2CHandle);
//				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             - Handles an I2C error event
 *
 * @param[in]         - Struct holding base address and desired configurations of SPI peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t errorIT;
	uint16_t sr1IT, sr2IT;

	//Know the status of  ITERREN control bit in the CR2
	errorIT = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

	//get static value for the SR registers
	sr1IT= pI2CHandle->pI2Cx->SR1;
	sr2IT= pI2CHandle->pI2Cx->SR2;

	if(errorIT){
		/***********************Check for Bus error************************************/
		if(sr1IT & I2C_FLAG_BERR)
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_BERR);
		}

		/***********************Check for arbitration lost error************************************/
		else if(sr1IT & I2C_FLAG_ARLO)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_ARLO);
		}

		/***********************Check for ACK failure  error************************************/

		else if(sr1IT & I2C_FLAG_AF)
		{
			//This is ACK failure error
			if(sr2IT & I2C_FLAG_MSL)
			{
				//I2C is in master mode
				//Implement the code to notify the application about the error
				I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_AF);
			} else
			{
				//I2C is in slave mode
				//probably should make this more robust to confirm the AF failure occured in the right time
				if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				{
					//I2C is transmitting
					if (pI2CHandle->TxLen == 0)
					{
						//all the data was sent and AF occured at the correct time
						I2C_CloseSendData(pI2CHandle);
						I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_SLAVE_TX_CMPLT);
					} else
					{
						I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_AF);
					}
				}
			}

			//Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);


		}

		/***********************Check for Overrun/underrun error************************************/
		else if(sr1IT & I2C_FLAG_OVR)
		{
			//This is Overrun/underrun

			//Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_OVR);
		}

		/***********************Check for Time out error************************************/
		else if(sr1IT & I2C_FLAG_TIMEOUT)
		{
			//This is Time out error

			//Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_TIMEOUT);
		}
	}
}

/*
 * Other peripheral control APIs
 */


/******************************************************
 * @fn					- I2C_PeripheralControl
 *
 * @brief				- Enables or Disables I2C
 *
 * @param[in]			- Pointer to a I2C register
 * @param[in]			- enable or disable bit
 *
 * @return				- none
 *
 * @Note				- none (might implement this in the transmit function)
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/******************************************************
 * @fn					- I2C_GetFlagStatusl
 *
 * @brief				- Obtains value of a specific flag in the Status Register
 *
 * @param[in]			- Pointer to a I2C register
 * @param[in]			- register to access (SR1 or SR2)
 * @param[in]			- flag to access
 *
 * @return				- none
 *
 * @Note				- TODO find a better way to use this with two SRs, should I just have the user pass the specific SR?
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t SRx, uint32_t flag){
	if(SRx == I2C_SR1){
		if(pI2Cx->SR1 & flag) {
				return SET;
			}
		return RESET;
	} else if (SRx == I2C_SR2) {
		if(pI2Cx->SR2 & flag) {
				return SET;
			}
		return RESET;
	}
	return RESET;//return something else to indicate invalid register?

}

/******************************************************
 * @fn					- I2C_ManageAcking
 *
 * @brief				- Enables or Disables I2C Acking
 *
 * @param[in]			- Pointer to a I2C register
 * @param[in]			- enable or disable bit
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t enOrDi){
	if (enOrDi == ENABLE) {
		//enable acking
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		//disable acking
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/******************************************************
 * @fn					- I2C_GenerateStopCondition
 *
 * @brief				- sets the CR1 STOP bit to generate the stop condition
 * 						- should be set during EV8_2 when TXE or BTS is set
 *
 * @param[in]			- base address of I2C peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/******************************************************
 * @fn					- I2C_SlaveControlCallbackEvents
 *
 * @brief				- enables or disables interrupt control bits
 *
 * @param[in]			- base address of I2C peripheral
 * @param[in]			- enable or disable byte
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_SlaveControlCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	} else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}
