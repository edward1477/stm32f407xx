/*
 * stm32f407xx.h
 *
 *  Created on: May 16, 2021
 *      Author: Edward
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10C )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1			( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2  		( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3			( (__vo uint32_t*)0XE000E18C )

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/************************************************************************************************************************
 * Define all base addresses of MCU memory including Flask, SRAM, ROM etc
*************************************************************************************************************************/

/*
 * base addresses of Flash and SRAM memories
 */

#define	FLASH_BASEADDR					(uint32_t)	0x08000000
#define	SRAM1_BASEADDR					(uint32_t)	0x20000000
#define SRAM2_BASEADDR					(uint32_t)	0x2001C000
#define ROM_BASEADDR					(uint32_t)	0x1FFF0000
#define SRAM							SRAM1_BASEADDR

/************************************************************************************************************************
 * Define all base addresses of Buses including AHBx, APBx etc
*************************************************************************************************************************/
/*
 * base addresses of AHBx and APBx Bus Peripheral
 */

#define PERIPH_BASEADDR					(uint32_t)	0x40000000
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				(uint32_t)	0x40010000
#define	AHB1PERIPH_BASEADDR				(uint32_t)	0x40020000
#define	AHB2PERIPH_BASEADDR				(uint32_t)	0x50000000

/************************************************************************************************************************
 * Define all base address of peripherals that hanging to AHBx and APBx buses
*************************************************************************************************************************/
/*
 * base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all peripherals
 *
 */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all peripherals
 *
 */

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)

/*
 * base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all peripherals
 *
 */

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)


/************************************************************************************************************************
 * Define peripherals register definition structures
*************************************************************************************************************************/
/*
 * Note: Registers of a peripheral are specific to MCU
 * e.g.: Number of registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM23Lx or STM32F0x family of MCUs
 * Please check your device reference manual
 *
 */

/*
 * Structure definition of GPIOx Register
 */

typedef struct {
	__vo uint32_t	MODER;				/*!< GPIO port mode register,						Address offset: 0x00	*/
	__vo uint32_t	OTYPER;				/*!< GPIO port output type register,				Address offset: 0x04	*/
	__vo uint32_t	OSPEEDR;			/*!< GPIO port output speed register,				Address offset: 0x08	*/
	__vo uint32_t	PUPDR;				/*!< GPIO port pull-up/pull-down register,			Address offset: 0x0C	*/
	__vo uint32_t	IDR;				/*!< GPIO port input data register,					Address offset: 0x10	*/
	__vo uint32_t	ODR;				/*!< GPIO port output data register,				Address offset: 0x14	*/
	__vo uint32_t	BSRR;				/*!< GPIO port bit set/reset register,				Address offset: 0x18	*/
	__vo uint32_t	LCKR;				/*!< GPIO port configuration lock register,			Address offset: 0x1C	*/
	__vo uint32_t	AFR[2];				/*!< AFR[0]: GPIO alternate function low register,	Address offset: 0x20	*/
										/*!< AFR[1]: GPIO alternate function high register,	Address offset: 0x24	*/
}GPIO_RegDef_t;

/*
 * Structure definition of RCC Register
 */

typedef struct {
	__vo uint32_t	CR;
	__vo uint32_t	PLLCFGR;
	__vo uint32_t	CFGR;
	__vo uint32_t	CIR;
	__vo uint32_t	AHB1RSTR;
	__vo uint32_t	AHB2RSTR;
	__vo uint32_t	AHB3RSTR;
	uint32_t		RESERVED0;
	__vo uint32_t	APB1RSTR;
	__vo uint32_t	APB2RSTR;
	uint32_t		RESERVED1[2];
	__vo uint32_t	AHB1ENR;
	__vo uint32_t	AHB2ENR;
	__vo uint32_t	AHB3ENR;
	uint32_t		RESERVED2;
	__vo uint32_t	APB1ENR;
	__vo uint32_t	APB2ENR;
	uint32_t		RESERVED3[2];
	__vo uint32_t	AHB1LPENR;
	__vo uint32_t	AHB2LPENR;
	__vo uint32_t	AHB3LPENR;
	uint32_t		RESERVED4;
	__vo uint32_t	APB1LPENR;
	__vo uint32_t	APB2LPENR;
	uint32_t		RESERVED5[2];
	__vo uint32_t	BDCR;
	__vo uint32_t	CSR;
	uint32_t		RESERVED6[2];
	__vo uint32_t	SSCGR;
	__vo uint32_t	PLLI2SCFGR;

}RCC_RegDef_t;

/*
 * Structure definition of EXTI Register
 */

typedef struct {
	__vo uint32_t	IMR;
	__vo uint32_t	EMR;
	__vo uint32_t	RTSR;
	__vo uint32_t	FTSR;
	__vo uint32_t	SWIER;
	__vo uint32_t	PR;

}EXTI_RegDef_t;

/*
 * Structure definition of SYSCFG Register
 */

typedef struct {
	__vo uint32_t	MEMRMP;
	__vo uint32_t	PMC;
	__vo uint32_t	EXTICR[4];
	uint32_t		RESERVED1[2];
	__vo uint32_t	CMPCR;
	uint32_t		RESERVED2[2];
	__vo uint32_t	CFGR;

}SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t	CR1;
	__vo uint32_t	CR2;
	__vo uint32_t	SR;
	__vo uint32_t	DR;
	__vo uint32_t	CRCPR;
	__vo uint32_t	RXCRCR;
	__vo uint32_t	TXCRCR;
	__vo uint32_t	I2SCFGR;
	__vo uint32_t	I2SPR;
}SPI_RegDef_t;

/************************************************************************************************************************
 * Define peripherals register definition structures
*************************************************************************************************************************/
/*
 * peripheral definition ( Peripheral base addresses typecasted to xxx_RegDef_t )
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)

/************************************************************************************************************************
 * Writing peripheral clock enable and disable marcos
*************************************************************************************************************************/
/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN()			( RCC -> AHB1ENR |= ( 1 << 8 ) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()			( RCC -> APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()			( RCC -> APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()			( RCC -> APB1ENR |= ( 1 << 23 ) )

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()			( RCC -> APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()			( RCC -> APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()			( RCC -> APB1ENR |= ( 1 << 15 ) )

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()			( RCC -> APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()			( RCC -> APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()			( RCC -> APB1ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()				( RCC -> APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN()				( RCC -> APB1ENR |= ( 1 << 20 ) )
#define USART6_PCLK_EN()			( RCC -> APB2ENR |= ( 1 << 5 ) )


/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()			( RCC -> APB2ENR |= ( 1 << 14 ) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()				( RCC -> AHB1ENR &= ~( 1 << 8 ) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()				( RCC -> APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()				( RCC -> APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()				( RCC -> APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()				( RCC -> APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()				( RCC -> APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()				( RCC -> APB1ENR &= ~( 1 << 15 ) )

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()			( RCC -> APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI()			( RCC -> APB1ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI()			( RCC -> APB1ENR &= ~( 1 << 18 ) )
#define UART4_PCLK_DI()				( RCC -> APB1ENR &= ~( 1 << 19 ) )
#define UART5_PCLK_DI()				( RCC -> APB1ENR &= ~( 1 << 20 ) )
#define USART6_PCLK_DI()			( RCC -> APB2ENR &= ~( 1 << 5 ) )


/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()			( RCC -> APB2ENR &= ~( 1 << 14 ) )

/*
 * GPIOx Reset Macros
 * Tricks to execute multiple line code is using do-while(0) loop in C
 * do-while(0) loop will execute all the code exactly 1 time and complete
 */

#define GPIOA_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 0 ) );	( RCC -> AHB1RSTR &= ~( 1 << 0 ) );}while(0)
#define GPIOB_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 1 ) );	( RCC -> AHB1RSTR &= ~( 1 << 1 ) );}while(0)
#define GPIOC_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 2 ) );	( RCC -> AHB1RSTR &= ~( 1 << 2 ) );}while(0)
#define GPIOD_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 3 ) );	( RCC -> AHB1RSTR &= ~( 1 << 3 ) );}while(0)
#define GPIOE_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 4 ) );	( RCC -> AHB1RSTR &= ~( 1 << 4 ) );}while(0)
#define GPIOF_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 5 ) );	( RCC -> AHB1RSTR &= ~( 1 << 5 ) );}while(0)
#define GPIOG_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 6 ) );	( RCC -> AHB1RSTR &= ~( 1 << 6 ) );}while(0)
#define GPIOH_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 7 ) );	( RCC -> AHB1RSTR &= ~( 1 << 7 ) );}while(0)
#define GPIOI_REG_RESET()			do{( RCC -> AHB1RSTR |= ( 1 << 8 ) );	( RCC -> AHB1RSTR &= ~( 1 << 8 ) );}while(0)

/*
 * SPIx Reset Macros
 */
#define SPI1_REG_RESET()			do{( RCC -> APB2RSTR |= ( 1 << 12 ) );	( RCC -> APB2RSTR &= ~( 1 << 12 ) );}while(0)
#define SPI2_REG_RESET()			do{( RCC -> APB1RSTR |= ( 1 << 14 ) );	( RCC -> APB1RSTR &= ~( 1 << 14 ) );}while(0)
#define SPI3_REG_RESET()			do{( RCC -> APB1RSTR |= ( 1 << 15 ) );	( RCC -> APB1RSTR &= ~( 1 << 15 ) );}while(0)

/*
 * Return GPIO port code Macros
 * This macro returns a code (between 0 - 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)		( (x == GPIOA) ? 0 :\
										  (x == GPIOB) ? 1 :\
										  (x == GPIOC) ? 2 :\
										  (x == GPIOD) ? 3 :\
										  (x == GPIOE) ? 4 :\
										  (x == GPIOF) ? 5 :\
										  (x == GPIOG) ? 6 :\
										  (x == GPIOH) ? 7 :\
										  (x == GPIOI) ? 8 :0 )

/*
 * Define IRQ [Interrupt Request] number
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
/*
 * Define some Generic Macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */

