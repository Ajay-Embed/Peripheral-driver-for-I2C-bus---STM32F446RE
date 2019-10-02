/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 24-Sep-2019
 *      Author: blueh
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

//THIS IS A CONFIGURATION STRUCTURE

typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t 	I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;


/*
 * Handle structure for I2cx peripherals
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t *I2C_Config;

	uint8_t *pTxBuffer;		//To store the applications TX buffer address
	uint8_t *pRxBuffer;  //To store the applications RX buffer address
	uint8_t TxRxState; //To store the communication state
	uint8_t DevAddr;	//To store the slave/device address
	uint8_t Sr;   	//To store the repeated start value

	uint32_t TxLen; //To store the Tx len
	uint32_t RxLen; //To store the Rx len
	uint32_t RxSize; //To stoer the Rx size
}I2C_Handle_t;



#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

/*
 * @i2C I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000U
#define I2C_SCL_SPEED_FM4K		400000U
#define I2C_SCL_SPEED_FM2K		200000U


/*
 * @i2C I2C_ACKControl
 */

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET
/*
 * @i2C I2C_DutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


#define I2C_EV_TX_CMPLT 		0
#define I2C_EV_RX_CMPLT 		1
#define I2C_EV_STOP		 		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9



#define I2C_FLAG_TXE 						   (1<< I2C_SR1_TXE)
#define I2C_FLAG_RXNE 						   (1<< I2C_SR1_RXNE)
#define I2C_FLAG_SB 						   (1<< I2C_SR1_SB)
#define I2C_FLAG_OVR 						   (1<< I2C_SR1_OVR)
#define I2C_FLAG_AF 						   (1<< I2C_SR1_AF)
#define I2C_FLAG_ARLO 						   (1<< I2C_SR1_ARLO)
#define I2C_FLAG_BERR 						   (1<< I2C_SR1_BERR)
#define I2C_FLAG_STOPF 						   (1<< I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 						   (1<< I2C_SR1_ADD10)
#define I2C_FLAG_BTF 						   (1<< I2C_SR1_BTF)
#define I2C_FLAG_ADDR 						   (1<< I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 					   (1<< I2C_SR1_TIMEOUT)


/*
 * *************************************API'S SUPPORTED BY THIS DRIVER*****************************************************
 */


/*
 * PERIPHERAL COCK CONTROL
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/*
 * INIT AND DEINIT
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
/*
 * DATA SEND AND RECEIVE
 */

/*
 * IRQ CONFIGURATION AND ISR HANDLING
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi );
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


uint8_t I2C_GetFlag_Status(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Other Peripheral control API's
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//APPLICATION CALLBACK
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPLLOutputClock(void);


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
