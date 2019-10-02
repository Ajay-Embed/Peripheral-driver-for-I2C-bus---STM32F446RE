/*
 * stm32f446xx.h
 *
 *  Created on: 20-Aug-2019
 *      Author: blueh
 */
/*
 * Created on 21/08/2019
 * by - Ajay C Rajan
 * 1 624 057 464
 *
 */
#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>
#include<stddef.h>



/* ******************************START: PROCESSOR SPECIFIC DETAILS************************************************
 *
 * ARM CORTEX Mx PROCESSOR NVIC ISERx REGISTERS
 */
//ISER REGEISTERS


#define NVIC_ISER0 						((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 						((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 						((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 						((__vo uint32_t*)0xE000E10C)

//ICER REGISTERS
#define NVIC_ICER0 						((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1 						((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2 						((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3 						((__vo uint32_t*)0xE000E18C)


#define NVIC_PR_BASEADDR 				((__vo uint32_t*)0xE000E400)

#define NO_OF_BITS_IMPLEMENTED			4




#define __vo  volatile
/* define base addresses for the memories of the micro_controller*/
#define FLASH_BASEADDR 						 0x08000000U //by default these numbers are signed, so unsign them
#define SRAM1_BASEADDR 						 0x20000000U //112KBv
#define SRAM2_BASEADDR						 0x2001C000U //16kB
#define ROM_BASEADDR						 0x1FFF0000U //30kB // System_Memeory
#define SRAM 								 SRAM1_BASEADDR

/*Defining the BUS base addresses AHB1/2 APB1/2 buses*/
#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR 				0x40000000U
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U

//Base address of peripherals being used in the AHB1 bus
#define GPIOA_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)

//Base address of the peripherals hanging on the APB1 bus
#define I2C1_BASEADDR 						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 						(APB1PERIPH_BASEADDR + 0x5C00)
#define UART4_BASEADDR 						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR 						(APB1PERIPH_BASEADDR + 0x5000)
#define USART2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 					(APB1PERIPH_BASEADDR + 0x4800)
#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

//Base address of the peripherals hanging on the APB2 bus
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR					    (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)

//**************************Peripheral register definition structures******************************************


//Note: Registers of the peripherals are specific to the MCU, check the reference manual of yours to write the correct code offset

//REGISTER MAP OF THE GPIO PERIPHERAL
typedef struct
{
__vo uint32_t MODER;   	 // Address offset = 0x00 --> GPIO port mode register (GPIOx_MODER) (x = A..H)
__vo uint32_t OTYPER;  	 // Address offset = 0x04 --> GPIO port output type register (GPIOx_OTYPER) (x = A..H)
__vo uint32_t OSPEEDER;   // Address offset = 0x08 --> GPIO port output speed register (GPIOx_OSPEEDR) (x = A..H)
__vo uint32_t PUPDR;    	 // Address offset = 0x0C --> GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..H)
__vo uint32_t IDR;        // Address offset = 0x10 --> GPIO port input data register (GPIOx_IDR) (x = A..H)
__vo uint32_t ODR;		 // Address offset = 0x14 --> GPIO port output data register (GPIOx_ODR) (x = A..H)
__vo uint32_t BSRR;       // Address offset = 0x18 --> GPIO port bit set/reset register (GPIOx_BSRR) (x = A..H)
__vo uint32_t LCKR;		 // Address offset = 0x1C --> GPIO port configuration lock register (GPIOx_LCKR) (x = A..H)
__vo uint32_t AFR[2];  	 // Address offset AFRL -> AFR[0] = 0x20 || Address offset AFRH -> AFR[1] = 0x24  ---->> GPIO alternate function low/high register (GPIOx_AFRL/H) (x = A..H)

}GPIO_RegDef_t;

//GPIO_RegDef_t *pGPIO = (GPIO_RegDef_t*)0x40020000; This statement can be rewritten as the following MACROS:




//REGISTER MAP OF THE SPI PERIPHERAL
typedef struct
{
	__vo uint32_t CR1;									//Address offset -> 0x00
	__vo uint32_t CR2;									//Address offset -> 0x04
	__vo uint32_t SR;									//Address offset -> 0x08
	__vo uint32_t DR;									//Address offset -> 0x0C
	__vo uint32_t CRCPR;								//Address offset -> 0x10
	__vo uint32_t RXCRCR;								//Address offset -> 0x14
	__vo uint32_t TXCRCR;								//Address offset -> 0x18
	__vo uint32_t I2SCFGR;								//Address offset -> 0x1C
	__vo uint32_t I2SPR;								//Address offset -> 0x20
}SPI_RegDef_t;

//REGISTER MAP OF I2C
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;




//Register map of USART

typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;




//RCC peripheral definitions
typedef struct
{
	__vo uint32_t CR;   				//Address offset = 0x00 -->>
	__vo uint32_t PLLCFGR;   			//Address offset = 0x04 -->>
	__vo uint32_t CFGR;   				//Address offset = 0x08 -->>
	__vo uint32_t CIR;   				//Address offset = 0x0C -->>
	__vo uint32_t AHB1RSTR;   			//Address offset = 0x10 -->>
	__vo uint32_t AHB2RSTR;   			//Address offset = 0x14 -->>
	__vo uint32_t AHB3RSTR;  			//Address offset = 0x18 -->>
	uint32_t RESERVED0;   				//Address offset = 0x1C -->>
	__vo uint32_t APB1RSTR;   			//Address offset = 0x20 -->>
	__vo uint32_t APB2RSTR;   			//Address offset = 0x24 -->>
	uint32_t RESERVED1[2];   			//Address offset = 0x28 - 0x2C -->>
	__vo uint32_t AHB1ENR;  			//Address offset = 0x30 -->>
	__vo uint32_t AHB2ENR;   			//Address offset = 0x34 -->>
	__vo uint32_t AHB3ENR;   			//Address offset = 0x38 -->>
	uint32_t RESERVED2;   				//Address offset = 0x3C -->>
	__vo uint32_t APB1ENR;   			//Address offset = 0x40 -->>
	__vo uint32_t APB2ENR;   			//Address offset = 0x44 -->>
	uint32_t RESERVED3[2];   			//Address offset = 0x48 - 0x4C -->>
	__vo uint32_t AHB1LPENR;  			//Address offset = 0x50 -->>
	__vo uint32_t AHB2LPENR;  			//Address offset = 0x54 -->>
	__vo uint32_t AHB3LPENR;  			//Address offset = 0x58 -->>
	uint32_t RESERVED4;   				//Address offset = 0x5C -->>
	__vo uint32_t APB1LPENR;   			//Address offset = 0x60 -->>
	__vo uint32_t APB2LPENR;   			//Address offset = 0x64 -->>
	uint32_t RESERVED5[2]; 				//Address offset = 0x68 - 0x6C -->>
	__vo uint32_t BDCR;   				//Address offset = 0x70 -->>
	__vo uint32_t CSR;   				//Address offset = 0x74 -->>
	__vo uint32_t RESERVED6[2];   		//Address offset = 0x78 - 0x7C->>
	__vo uint32_t SSCGR;   				//Address offset = 0x80 -->>
	__vo uint32_t PLLI2SCGR;   			//Address offset = 0x84 -->>
	__vo uint32_t PLLSAICFGR;   		//Address offset = 0x88 -->>
	__vo uint32_t DCKCFGR;   			//Address offset = 0x8C -->>
	__vo uint32_t CKGATENR;   			//Address offset = 0x90 -->>
	__vo uint32_t DCKCFGR2;   			//Address offset = 0x94 -->>


}RCC_RegDef_t;



//EXTI peripheral definitions

typedef struct{
	__vo uint32_t IMR;					//Address offset = 0x00 -->>  INTERRUPT MASK REGISTER
	__vo uint32_t EMR;					//Address offset = 0x04 -->>  EVENT MASK REGISTER
	__vo uint32_t RTSR;					//Address offset = 0x08 -->>  RISING TRIGGER SELECTION REGISTER
	__vo uint32_t FTSR;					//Address offset = 0x0C -->>  FALLING TRIGGER SELECTION REGISTER
	__vo uint32_t SWIER;				//Address offset = 0x10 -->>  SOFTWARE INTERRUPT EVENT REGISTER
	__vo uint32_t PR;                   //Address offset = 0x14 -->>  PENDING REGISTER
}EXTI_RegDef_t;



//SYS_CFG peripheral definitions

typedef struct{
	__vo uint32_t MEMRPMP;				//Address offset = 0x00 -->> MEMORY REMAP REGISTER
	__vo uint32_t PMC;					//Address offset = 0x04 -->> PERIPHERAL MODE CONFIGURATION REGISTER
	__vo uint32_t EXTICR[4];			//Address offset = 0x08- 0x14 -->> EXTERNAL INTERRUPT CONFIGURATION REGISTER 1 -4
uint32_t RESERVED1[2];					//0x18 AND 0x1C
	__vo uint32_t CMPCR;				//Address offset = 0x20 -->> COMPENSATION CELL CONTROL REISTER
uint32_t RESERVED2[2];					//0x24 AND 0x28
	__vo uint32_t CFGR;					//Address offset = 0x2C -->> CONFIGURATION REGISTER

}SYSCFG_RegDef_t;

//********************************PERIPHERAL DEFINITIONS*******(Peripheral base address type casted to reg_def structure)

#define GPIOA  (GPIO_RegDef_t*) GPIOA_BASEADDR
#define GPIOB  (GPIO_RegDef_t*) GPIOB_BASEADDR
#define GPIOC  (GPIO_RegDef_t*) GPIOC_BASEADDR
#define GPIOD  (GPIO_RegDef_t*) GPIOD_BASEADDR
#define GPIOE  (GPIO_RegDef_t*) GPIOE_BASEADDR
#define GPIOF  (GPIO_RegDef_t*) GPIOF_BASEADDR
#define GPIOG  (GPIO_RegDef_t*) GPIOG_BASEADDR
#define GPIOH  (GPIO_RegDef_t*) GPIOH_BASEADDR



#define SPI1	(SPI_RegDef_t*)SPI1_BASEADDR
#define SPI2	(SPI_RegDef_t*)SPI2_BASEADDR
#define SPI3	(SPI_RegDef_t*)SPI3_BASEADDR


#define I2C1	(I2C_RegDef_t*)I2C1_BASEADDR
#define I2C2	(I2C_RegDef_t*)I2C2_BASEADDR
#define I2C3	(I2C_RegDef_t*)I2C3_BASEADDR



#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)



#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR) // Also define the EXTI peripheral structure
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


//CLOCK ENABLE MACROS FOR GPIO PERIPHERALS
#define GPIOA_PCLK_EN					(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN					(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN					(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN					(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN					(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN					(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN					(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN					(RCC->AHB1ENR |= (1<<7))


//CLOCK EN MACROS FOR I2Cx PERIPHERALS
#define I2C1_PCLK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<21))
#define I2C2_PCLK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<22))
#define I2C3_PCLK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<23))


//CLK EN MARCORS FOR SpIx PERIPHERALS
#define SPI1_PCLK_EN()					 (RCC->APB2ENR = RCC->APB2ENR | (1<<12))
#define SPI2_PCLK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<14))
#define SPI3_PCLK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<15))

//CLK EN MACROS FOR SYSCFG PERIPHERALS
#define SYSCFG_PCLK_EN()					 (RCC->APB2ENR = RCC->APB2ENR | (1<<14))

//CLK EN MACROS FOR USART PERIPHERALS
#define USART1_PCCK_EN()					 (RCC->APB2ENR = RCC->APB2ENR | (1<<4))
#define USART2_PCCK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<17))
#define USART3_PCCK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<18))
#define UART4_PCCK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<19))
#define UART5_PCCK_EN()					 (RCC->APB1ENR = RCC->APB1ENR | (1<<20))
#define USART6_PCCK_EN()					 (RCC->APB2ENR = RCC->APB2ENR | (1<<5))


//CLK DISABLE MACROS FOR THE GPIOx PERIPHERALS

#define GPIOA_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<0))

#define GPIOB_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<1))

#define GPIOC_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<2))

#define GPIOD_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<3))

#define GPIOE_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<4))

#define GPIOF_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<5))

#define GPIOG_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<6))

#define GPIOH_PCLK_DI()					(RCC->AHB1ENR = RCC->AHB1ENR & ~(1<<7))


//CLK DISABLE MACROS FOR I2Cx PERIPHERALS
#define I2C1_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<21))
#define I2C2_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<22))
#define I2C3_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<23))


//CLK DISABLE MACROSS FOR SPIx PERIPHERALS
#define SPI1_PCLK_DI()					 (RCC->APB2ENR = RCC->APB2ENR & ~(1<<12))
#define SPI2_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<14))
#define SPI3_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<15))


//CLK DISABLE MACROS FOR USARTx PERIPHERALS
#define USART1_PCLK_DI()					 (RCC->APB2ENR = RCC->APB2ENR & ~(1<<4))
#define USART2_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<17))
#define USART3_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<18))
#define UART4_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<19))
#define UART5_PCLK_DI()					 (RCC->APB1ENR = RCC->APB1ENR & ~(1<<20))
#define USART6_PCLK_DI()					 (RCC->APB2ENR = RCC->APB2ENR & ~(1<<5))

//CLK DISABLE MACROS FOR SYSCFG PERIPHERALS
#define SYSCFG_PCLK_DI()					 (RCC->APB2ENR = RCC->APB2ENR & ~(1<<14))

// MACROS TO RESET GPIOX PERIPHERALS

#define GPIOA_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<0));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<0)); }while(0)

#define GPIOB_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<1));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<1)); }while(0)

#define GPIOC_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<2));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<2)); }while(0)

#define GPIOD_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<3));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<3)); }while(0)

#define GPIOE_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<4));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<4)); }while(0)

#define GPIOF_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<5));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<5)); }while(0)

#define GPIOG_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<6));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<6)); }while(0)

#define GPIOH_REG_RESET()                      do{ (RCC->AHB1RSTR = RCC->AHB1RSTR | (1<<7));  (RCC->AHB1RSTR = RCC->AHB1RSTR & ~(1<<7)); }while(0)



#define SPI1_REG_RESET()						do{ (RCC->APB2RSTR = RCC->APB2RSTR | (1<<12));  (RCC->APB2RSTR = RCC->APB2RSTR & ~(1<<12)); }while(0)
#define SPI2_REG_RESET()						do{ (RCC->APB1RSTR = RCC->APB1RSTR | (1<<14));  (RCC->APB1RSTR = RCC->APB1RSTR & ~(1<<14)); }while(0)
#define SPI3_REG_RESET()						do{ (RCC->APB1RSTR = RCC->APB1RSTR | (1<<15));  (RCC->APB1RSTR = RCC->APB1RSTR & ~(1<<15)); }while(0)


#define I2C1_REG_RESET()						do{ (RCC->APB1RSTR |= (1<<21));  (RCC->APB2RSTR &= ~(1<<21)); }while(0)
#define I2C2_REG_RESET()						do{ (RCC->APB1RSTR |= (1<<22));  (RCC->APB2RSTR &= ~(1<<22)); }while(0)
#define I2C3_REG_RESET()						do{ (RCC->APB1RSTR |= (1<<23));  (RCC->APB2RSTR &= ~(1<<23)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x) ((x==GPIOA) ? 0 :\
								  (x==GPIOB) ? 1 :\
								  (x==GPIOC) ? 2 :\
                                  (x==GPIOD) ? 3 :\
                                  (x==GPIOE) ? 4 :\
                                  (x==GPIOF) ? 5 :\
                                  (x==GPIOG) ? 6 :\
								  (x==GPIOH) ? 7 : 0)


//Interrupt request numbers of stm32f4xx family of MCU, update them according tho the family of your mcu

#define IRQ_NO_EXTI0 			6
#define IRQ_NO_EXTI1 			7
#define IRQ_NO_EXTI2 			8
#define IRQ_NO_EXTI3 			9
#define IRQ_NO_EXTI4 			10
#define IRQ_NO_EXTI9_5 			23
#define IRQ_NO_EXTI15_10 		40

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71










#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET 				RESET
#define FLAG_SET				SET





/*
 ***************************************************************************************************************************************
 * 											BIT POSITION DEFINITIONS MACROS
 ****************************************************************************************************************************************
 */

//***************************************************BIT POSITION DEFINITION FOR SPI_SR***************************************************
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BUSY				7
#define SPI_SR_FRE				8
//***************************************************BIT POSITION DEFINITION FOR SPI_CR1***************************************************
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7		3
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15
//***************************************************BIT POSITION DEFINITION FOR SPI_CR2***************************************************
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7




/*
 * *******************************************BIT POSITION DEFINITIONS FOR I2C_CR1 PERIPHERAL***********************************************************
 */
#define I2C_CR1_PE 					0
#define I2C_CR1_NOSTRETCH 			7
#define I2C_CR1_START 				8
#define I2C_CR1_STOP 				9
#define I2C_CR1_ACK 				10
#define I2C_CR1_SWRST 				15


/*
 * *******************************************BIT POSITION DEFINITIONS FOR I2C_CR2 PERIPHERAL***********************************************************
 */
#define I2C_CR2_FREQ 					0
#define I2C_CR2_ITERREN 				8
#define I2C_CR2_ITEVTEN 				9
#define I2C_CR2_ITBUFEN 				10

/*
 * *******************************************BIT POSITION DEFINITIONS FOR I2C_OAR1 PERIPHERAL***********************************************************
 */

#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15

/*
 * *******************************************BIT POSITION DEFINITIONS FOR I2C_SR1 PERIPHERAL***********************************************************
 */
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_TIMEOUT				14


/*
 * *******************************************BIT POSITION DEFINITIONS FOR I2C_SR2 PERIPHERAL***********************************************************
 */
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_DUALF				7


/*
 * *******************************************BIT POSITION DEFINITIONS FOR I2C_CCR PERIPHERAL***********************************************************
 */
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15
/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


/*
 * POSSIBLE SPI APPLICATION EVENT
 */

#define SPI_EVENT_TX_CMPLT 		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4











//5037094277

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"
#endif /* INC_STM32F446XX_H_ */
