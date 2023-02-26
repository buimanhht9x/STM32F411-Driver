/*
 * stm32f4xx_i2c_driver.h
 *
 *  Created on: Apr 16, 2020
 *      Author: Adam
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct {
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint8_t		I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx perpipheral
 */
typedef struct {
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
	uint8_t			*pTxBuffer;	/* !< To store the app. Tx buffer address > */
	uint8_t			*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t		TxLen;		/* !< To store Tx len > */
	uint32_t		RxLen;		/* !< To store Rx len > */
	uint8_t			TxRxState;	/* !< To store Communication state > */
	uint8_t			DevAddr;	/* !< To store slave/device address > */
	uint32_t		RxSize;		/* !< To store Rx size (why?)> */
	uint32_t		repStart;	/* !< To store repeated start value > */
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C status flag definitions
 */

#define I2C_FLAG_SB				(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10			(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF			(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE			(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE			(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR			(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF				(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR			(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT		(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT		(1 << I2C_SR1_SMBALERT)

#define I2C_FLAG_MSL			(1 << I2C_SR2_MSL)
#define I2C_FLAG_BUSY			(1 << I2C_SR2_BUSY)
#define I2C_FLAG_TRA			(1 << I2C_SR2_TRA)
#define I2C_FLAG_GENCALL		(1 << I2C_SR2_GENCALL)
#define I2C_FLAG_SMBDEFAULT		(1 << I2C_SR2_SMBDEFAULT)
#define I2C_FLAG_SMBHOST		(1 << I2C_SR2_SMBHOST)
#define I2C_FLAG_DUALF			(1 << I2C_SR2_DUALF)
#define I2C_FLAG_PEC			(1 << I2C_SR2_PEC)

/*
 * I2C application states
 */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*
 * I2C repeated start values
 */
#define I2C_SR_DISABLED		RESET
#define I2C_SR_ENABLED		SET

/*
 * I2C Status Register variables
 */
#define I2C_SR1		1
#define I2C_SR2		2

/*
 * I2C Application Event Macros
 */
#define I2C_EV_MASTER_TX_CMPLT		0
#define I2C_EV_MASTER_RX_CMPLT		1
#define I2C_EV_SLAVE_TX_CMPLT		2
#define I2C_EV_SLAVE_RX_CMPLT		3
#define I2C_EV_STOPF				4
#define I2C_ER_BERR  				5
#define I2C_ER_ARLO  				6
#define I2C_ER_AF    				7
#define I2C_ER_OVR   				8
#define I2C_ER_TIMEOUT 				9
//#define I2C_EV_DATA_REQ				10
//#define I2C_EV_DATA_RCV				11

/**************************************************************************
 * 					APIs supported by this driver
 * For more information about these APIs check the function description
 **************************************************************************/
/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t enOrDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2Cx);


/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t repStart);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repStart);
//void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len);
//void I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
void I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repStart);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repStart);
uint8_t I2C_SlaveSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len);
uint8_t I2C_SlaveReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ configuration and handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNumber);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 * Other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t SRx, uint32_t flag);
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t enOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveControlCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
