/*
 * i2c_master_rx_testing.c
 *
 *  Created on: 27-Sep-2019
 *      Author: blueh
 */


/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Feb 24, 2019
 *      Author: admin
 */


#include<stdio.h>
#include<string.h>
#include "stm32f446xx.h"


#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/6 ; i ++);
}

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[32];
/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config->I2C_ACKControl 		= I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config->I2C_DeviceAddress 	= MY_ADDR;
	I2C1Handle.I2C_Config->I2C_FMDutyCycle 		= I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config->I2C_SCLSpeed 		= I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);

}


int main(void)
{
uint8_t command_code;
uint8_t Len;


	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();



	//i2c peripheral configuration
	I2C1_Inits();

	//Enabling the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);


while(1)
{
	while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
	delay();
command_code = 0x51;
	I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR);
	I2C_MasterReceiveData(&I2C1Handle, &Len, 1, SLAVE_ADDR);

	command_code = 0x52;
		I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR);
		I2C_MasterReceiveData(&I2C1Handle, some_data, Len, SLAVE_ADDR);
}
		//send some data to the slave
}
