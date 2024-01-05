/*
 * stm32f407xx.h
 *
 *  Created on: Aug 1, 2023
 *      Author: geroldwilliams
 */
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

#define ENABLE  		1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/******************PROCESSOR SPECIFIC DETAILS******************/
/**************************CORTEX M4***************************/
//NVIC Register Addresses
//ISER
#define NVIC_ISER0 				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 				((__vo uint32_t*)0xE000E10C)

//ICER
#define NVIC_ICER0 				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1 				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2 				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3 				((__vo uint32_t*)0xE000E18C)

//IRQ Priority Register Base Address
#define NVIC_PR_BASE_ADDR 		((__vo uint32_t*)0xE000E400)

//Number of priority bits
#define NO_PR_BITS_IMPLEMENTED	4

//Priority Levels
#define	NVIC_IRQ_PRIORITY0		0
#define	NVIC_IRQ_PRIORITY1		1
#define	NVIC_IRQ_PRIORITY2		2
#define	NVIC_IRQ_PRIORITY3		3
#define	NVIC_IRQ_PRIORITY4		4
#define	NVIC_IRQ_PRIORITY5		5
#define	NVIC_IRQ_PRIORITY6		6
#define	NVIC_IRQ_PRIORITY7		7
#define	NVIC_IRQ_PRIORITY8		8
#define	NVIC_IRQ_PRIORITY9		9
#define	NVIC_IRQ_PRIORITY10		10
#define	NVIC_IRQ_PRIORITY11		11
#define	NVIC_IRQ_PRIORITY12		12
#define	NVIC_IRQ_PRIORITY13		13
#define	NVIC_IRQ_PRIORITY14		14
#define	NVIC_IRQ_PRIORITY15		15

/**************************************************************/

//Base address of Flash and SRAM
#define FLASH_BASEADDR 			0x08000000U		//Base address of flash
#define	SRAM1_BASEADDR 			0x20000000U		//Base address of SRAM
#define SRAM2_BASEADDR			0x2001C000U		//Base address of AUX SRAM
#define ROM_BASEADDR			0X1FFF0000U		//Base address of ROM
#define SRAM 					SRAM1_BASEADDR

#define PERIPH_BASEADDR			0x40000000U
#define	APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define	APB2PERIPH_BASEADDR		0x40010000U
#define	AHB1PERIPH_BASEADDR		0x40020000U
#define	AHB2PERIPH_BASEADDR		0x50000000U

//Base address of AHB1 Peripherals
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

//Base address of APB1 Peripherals
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)


//Base address of APB2 Peripherals
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

//Peripheral Register Definition Structures
//GPIO
typedef struct{
	__vo uint32_t MODER;		/*!< Mode register >*/
	__vo uint32_t OTYPER;		/*!< Output type register >*/
	__vo uint32_t OSPEEDR;		/*!< Output speed register >*/
	__vo uint32_t PUPDR;		/*!< Pull up Pull down register >*/
	__vo uint32_t IDR;			/*!< Input data register >*/
	__vo uint32_t ODR;			/*!< Output data register >*/
	__vo uint32_t BSRR;			/*!< Bit set/reset register >*/
	__vo uint32_t LCKR;			/*!< Configuration lock register >*/
	__vo uint32_t AFR[2]; 		/*!< Alternate function Registers [LOW(0)][HIGH(1)] >*/
}GPIO_RegDef_t;

//SPI
typedef struct{
	__vo uint32_t CR1;			/*!< Control register 1 >*/
	__vo uint32_t CR2;			/*!< Control register 2 >*/
	__vo uint32_t SR;			/*!< Status register >*/
	__vo uint32_t DR;			/*!< Data register >*/
	__vo uint32_t CRCPR;		/*!< CRC polynomial register >*/
	__vo uint32_t RXCRCR;		/*!< RX CRC register >*/
	__vo uint32_t TXCRCR;		/*!< TX CRC register >*/
	__vo uint32_t I2SCFGR;		/*!< Configuration register >*/
	__vo uint32_t I2SPR;		/*!< Pre-scaler register >*/
}SPI_RegDef_t;

//I2C
typedef struct{
	__vo uint32_t CR1;			/*!< Control register 1 >*/
	__vo uint32_t CR2;			/*!< Control register 2 >*/
	__vo uint32_t OAR1;			/*!< Own address register 1 >*/
	__vo uint32_t OAR2;			/*!< Own address register 2  >*/
	__vo uint32_t DR;			/*!< Data register >*/
	__vo uint32_t SR1;			/*!< Status register 1 >*/
	__vo uint32_t SR2;			/*!< Status register 2 >*/
	__vo uint32_t CCR;			/*!< Clock control register >*/
	__vo uint32_t TRISE;		/*!< Rise time register >*/
}I2C_RegDef_t;

typedef struct{
	__vo uint32_t SR;			/*!< Status register >*/
	__vo uint32_t DR;			/*!< Data register >*/
	__vo uint32_t BRR;			/*!< Baud rate register >*/
	__vo uint32_t CR1;			/*!< Control register 1 >*/
	__vo uint32_t CR2;			/*!< Control register 2 >*/
	__vo uint32_t CR3;			/*!< Control register 3 >*/	
	__vo uint32_t GTPR;			/*!< Guard time and prescaler register >*/
}USART_RegDef_t;

//SYSCFG
typedef struct{
	__vo uint32_t MEMRMP;		/*!< Memory re-map register. >*/
	__vo uint32_t PMC;			/*!< Peripheral mode configuration register. >*/
	__vo uint32_t EXTICR[4];	/*!< Source input for the EXTIx external interrupt. 0 - EXTI0-3, 1 - EXTI4-7, 2 - EXTI8-11, 3 - EXTI12-15 >*/
	     uint32_t RESERVED1[2];	/*!< RESERVED >*/
	__vo uint32_t CMPCR;		/*!< Compensation cell control register. >*/
}SYSCFG_RegDef_t;

//EXTI
typedef struct{
	__vo uint32_t IMR;		/*!< Interrupt mask register >*/
	__vo uint32_t EMR;		/*!< Event mask register >*/
	__vo uint32_t RTSR;		/*!< Rising trigger selection register >*/
	__vo uint32_t FTSR;		/*!< Falling trigger selection register >*/
	__vo uint32_t SWIER;	/*!< Software interrupt event register >*/
	__vo uint32_t PR;		/*!< Pending register >*/
}EXTI_RegDef_t;

//RCC
typedef struct{
	__vo uint32_t CR;					/*!< RCC clock control register >*/
	__vo uint32_t PLLCFGR;				/*!< RCC PLL configuration register >*/
	__vo uint32_t CFGR;					/*!< RCC clock configuration register >*/
	__vo uint32_t CIR;					/*!< RCC clock interrupt register >*/
	__vo uint32_t AHB1RSTR;				/*!< RCC AHB1 peripheral clock reset register >*/
	__vo uint32_t AHB2RSTR;				/*!< RCC AHB2 peripheral clock reset register >*/
	__vo uint32_t AHB3RSTR;				/*!< RCC AHB3 peripheral clock reset register >*/
	     uint32_t RESERVED0;			/*!< RESERVED >*/
	__vo uint32_t APB1RSTR;				/*!< RCC APB1 peripheral clock reset register >*/
	__vo uint32_t APB2RSTR;				/*!< RCC APB2 peripheral clock reset register >*/
	     uint32_t RESERVED1[2];			/*!< RESERVED >*/
	__vo uint32_t AHB1ENR;				/*!< RCC AHB1 peripheral clock enable register >*/
	__vo uint32_t AHB2ENR;				/*!< RCC AHB2 peripheral clock enable register >*/
	__vo uint32_t AHB3ENR;				/*!< RCC AHB3 peripheral clock enable register >*/
	     uint32_t RESERVED2;			/*!< RESERVED >*/
	__vo uint32_t APB1ENR;				/*!< RCC APB1 peripheral clock enable register >*/
	__vo uint32_t APB2ENR;				/*!< RCC APB2 peripheral clock enable register >*/
	     uint32_t RESERVED3[2];			/*!< RESERVED >*/
	__vo uint32_t AHB1LPENR;			/*!< RCC AHB1 low power peripheral clock enable register >*/
	__vo uint32_t AHB2LPENR;			/*!< RCC AHB2 low power peripheral clock enable register >*/
	__vo uint32_t AHB3LPENR;			/*!< RCC AHB3 low power peripheral clock enable register >*/
	     uint32_t RESERVED4;			/*!< RESERVED >*/
	__vo uint32_t APB1LPENR;			/*!< RCC APB1 low power peripheral clock enable register >*/
	__vo uint32_t APB2LPENR;			/*!< RCC APB2 low power peripheral clock enable register >*/
	     uint32_t RESERVED5[2];			/*!< RESERVED >*/
	__vo uint32_t BDCR;					/*!< RCC Backup domain control register >*/
	__vo uint32_t CSR;					/*!< RCC clock control & status register >*/
	     uint32_t RESERVED6[2];			/*!< RESERVED >*/
	__vo uint32_t SSCGR;				/*!< RCC spread spectrum clock generation register  >*/
	__vo uint32_t PLLI2SCFGR;			/*!< RCC PLLI2S configuration register >*/
}RCC_RegDef_t;


//Peripheral definitions (Peripheral base address typecast to xxx_RegDef_t)
//GPIO
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ		((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK		((GPIO_RegDef_t*)GPIOK_BASEADDR)

//SPI
#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)

//I2C
#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

//USART
#define USART1 		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2 		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3 		((USART_RegDef_t*)USART3_BASEADDR)
#define USART6 		((USART_RegDef_t*)USART6_BASEADDR)

//UART
#define UART4 		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5 		((USART_RegDef_t*)UART5_BASEADDR)

//RCC
#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

//EXTI
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

//SYSCFG
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

//Clock enable for GPIOx Peripherals
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 8 ) )

//Clock enable for I2Cx Peripherals
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )

//Clock enable for SPIx Peripherals
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )

//Clock enable for USARTx Peripherals
#define	USART1_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 4  ) )
#define	USART2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 17 ) )
#define	USART3_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 18 ) )
#define	USART6_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 5  ) )

//Clock enable for UARTx Peripherals
#define	UART4_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 19 ) )
#define	UART5_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 20 ) )

//Clock enable for SYSCFG Peripherals
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 14 ) )

//Clock Disable for GPIOx Peripherals
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 8 ) )

//Clock Disable for I2Cx Peripherals
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )

//Clock Disable for SPIx Peripherals
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )

//Clock Disable for USARTx Peripherals
#define	USART1_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 4  ) )
#define	USART2_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 17 ) )
#define	USART3_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 18 ) )
#define	USART6_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 5  ) )

//Clock Disable for UARTx Peripherals
#define	UART4_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 19 ) )
#define	UART5_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 20 ) )

//Clock Disable for SYSCFG Peripherals
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 14 ) )

//Clock Reset for GPIOx peripheral(SET AND CLEAR)
#define GPIOA_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 0 ) );( RCC->AHB1RSTR &= ~( 1 << 0 ) );}while(0)
#define GPIOB_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 1 ) );( RCC->AHB1RSTR &= ~( 1 << 1 ) );}while(0)
#define GPIOC_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 2 ) );( RCC->AHB1RSTR &= ~( 1 << 2 ) );}while(0)
#define GPIOD_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 3 ) );( RCC->AHB1RSTR &= ~( 1 << 3 ) );}while(0)
#define GPIOE_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 4 ) );( RCC->AHB1RSTR &= ~( 1 << 4 ) );}while(0)
#define GPIOF_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 5 ) );( RCC->AHB1RSTR &= ~( 1 << 5 ) );}while(0)
#define GPIOG_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 6 ) );( RCC->AHB1RSTR &= ~( 1 << 6 ) );}while(0)
#define GPIOH_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 7 ) );( RCC->AHB1RSTR &= ~( 1 << 7 ) );}while(0)
#define GPIOI_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 8 ) );( RCC->AHB1RSTR &= ~( 1 << 8 ) );}while(0)

//Clock Reset for  SPIx peripheral(SET AND CLEAR)
#define SPI1_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 12 ) );( RCC->APB2RSTR &= ~( 1 << 12 ) );}while(0)
#define SPI2_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 14 ) );( RCC->APB1RSTR &= ~( 1 << 14 ) );}while(0)
#define SPI3_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 15 ) );( RCC->APB1RSTR &= ~( 1 << 15 ) );}while(0)

//Clock Reset for  I2Cx peripheral(SET AND CLEAR)
#define I2C1_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 21 ) );( RCC->APB2RSTR &= ~( 1 << 21 ) );}while(0)
#define I2C2_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 22 ) );( RCC->APB1RSTR &= ~( 1 << 22 ) );}while(0)
#define I2C3_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 23 ) );( RCC->APB1RSTR &= ~( 1 << 23 ) );}while(0)

//Clock Reset for USARTx and UARTx peripheral(SET AND CLEAR)
#define USART1_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 4  ) );( RCC->APB2RSTR &= ~( 1 << 4  ) );}while(0)
#define USART2_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 17 ) );( RCC->APB1RSTR &= ~( 1 << 17 ) );}while(0)
#define USART3_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 18 ) );( RCC->APB1RSTR &= ~( 1 << 18 ) );}while(0)
#define USART6_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 5  ) );( RCC->APB2RSTR &= ~( 1 << 5  ) );}while(0)
#define UART4_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 19 ) );( RCC->APB1RSTR &= ~( 1 << 19 ) );}while(0)
#define UART5_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 20 ) );( RCC->APB1RSTR &= ~( 1 << 20 ) );}while(0)

//Function to calculate the Port Code using the Port Base address(conditional/ternary operations)
#define GPIO_BASEADDR_TO_CODE(x)		( (x == GPIOA) ? 0 :\
									  	  (x == GPIOB) ? 1 :\
									  	  (x == GPIOC) ? 2 :\
									  	  (x == GPIOD) ? 3 :\
									  	  (x == GPIOE) ? 4 :\
									  	  (x == GPIOF) ? 5 :\
									  	  (x == GPIOG) ? 6 :\
									  	  (x == GPIOH) ? 7 :\
									  	  (x == GPIOI) ? 8 : 0 )

//IRQ(Interrupt Request) Number of STM32F407xx MCU
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI5_9			23
#define IRQ_NO_EXTI10_15		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73
#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53
#define IRQ_NO_USART6			71
/************************** SPI REG BIT FIELDS ***************************/
// SPI_CR1
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

// SPI_CR2
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

// SPI_SR
#define SPI_SR_RXE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSID				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8
/*************************************************************************/

/************************** I2C REG BIT FIELDS ***************************/
//CR1
#define I2C_CR1_PE							0
#define I2C_CR1_SMBUS						1
#define I2C_CR1_SMBTYPE						3
#define I2C_CR1_ENARP						4
#define I2C_CR1_ENPEC						5
#define I2C_CR1_ENGC						6
#define I2C_CR1_NOSTRETCH					7
#define I2C_CR1_START						8
#define I2C_CR1_STOP						9
#define I2C_CR1_ACK							10
#define I2C_CR1_POS							11
#define I2C_CR1_PEC							12
#define I2C_CR1_ALERT						13
#define I2C_CR1_SWRST						15

//OAR1
#define I2C_OAR1_ADD0						0
#define I2C_OAR1_ADD71						1
#define I2C_OAR1_ADD98						8
#define I2C_OAR1_ADDMODE					15

//CR2
#define I2C_CR2_FREQ						0
#define I2C_CR2_ITERREN						8
#define I2C_CR2_ITEVTEN						9
#define I2C_CR2_ITBUFEN						10
#define I2C_CR2_DMAEN						11
#define I2C_CR2_LAST						12

//SR1
#define I2C_SR1_SB							0
#define I2C_SR1_ADDR						1
#define I2C_SR1_BTF							2
#define I2C_SR1_ADD10						3
#define I2C_SR1_STOPF						4
#define I2C_SR1_RXNE						6
#define I2C_SR1_TXE							7
#define I2C_SR1_BERR						8
#define I2C_SR1_ARLO						9
#define I2C_SR1_AF							10
#define I2C_SR1_OVR							11
#define I2C_SR1_PECERR						12
#define I2C_SR1_TIMEOUT						14
#define I2C_SR1_SMBALERT					15

//SR2
#define I2C_SR2_MSL							0
#define I2C_SR2_BUSY						1
#define I2C_SR2_TRA							2
#define I2C_SR2_GENCALL						4
#define I2C_SR2_SMBDEFAULT					5
#define I2C_SR2_SMBHOST						6
#define I2C_SR2_DUALF						7

//CRR
#define I2C_CRR_CCR							0
#define I2C_CRR_DUTY						14
#define I2C_CRR_FS							15
/*************************************************************************/
/************************** USART REG BIT FIELDS ***************************/
//SR
#define USART_SR_PE							0
#define USART_SR_FE 						1
#define USART_SR_NF 						2
#define USART_SR_ORE						3
#define USART_SR_IDLE						4
#define USART_SR_RXNE						5
#define USART_SR_TC 						6
#define USART_SR_TXE						7
#define USART_SR_LBD						8
#define USART_SR_CTS						9

//BRR
#define USART_BRR_DIV_FRACTION30			0
#define USART_BRR_DIV_MANTISSA15_4			4

//CR1
#define USART_CR1_SBK						0
#define USART_CR1_RWU						1
#define USART_CR1_RE						2
#define USART_CR1_TE						3
#define USART_CR1_IDLEIE					4
#define USART_CR1_RXNEIE					5
#define USART_CR1_TCIE						6
#define USART_CR1_TXEIE						7
#define USART_CR1_PEIE						8
#define USART_CR1_PS						9
#define USART_CR1_PCE						10
#define USART_CR1_WAKE						11
#define USART_CR1_M							12
#define USART_CR1_UE						13
#define USART_CR1_OVER8						15

//CR2
#define USART_CR2_ADD03						0
#define USART_CR2_LBDL						5
#define USART_CR2_LBDIE						6
#define USART_CR2_LBCL						8
#define USART_CR2_CPHA						9
#define USART_CR2_CPOL						10
#define USART_CR2_CLKEN						11
#define USART_CR2_STOP						12
#define USART_CR2_LINEN						14

//CR3
#define USART_CR3_EIE						0
#define USART_CR3_IREN						1
#define USART_CR3_IRLP						2
#define USART_CR3_HDSEL						3
#define USART_CR3_NACK						4
#define USART_CR3_SCEN						5
#define USART_CR3_DMAR						6
#define USART_CR3_DMAT						7
#define USART_CR3_RTSE						8
#define USART_CR3_CTSE						9
#define USART_CR3_CTSIE						10
#define USART_CR3_ONEBIT					11

//USART_GTPR
#define USART_GTPR_PSC7_0					0
#define USART_GTPR_GT15_8					8
/*************************************************************************/
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#endif /* INC_STM32F407XX_H_ */
