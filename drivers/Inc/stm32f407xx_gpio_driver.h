/*
 * stm32f07xx_gpio_driver.h
 *
 *  Created on: May 16, 2021
 *      Author: Edward
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Define a Configuration structure for a GPIO pin
 * This is a structure variable for user application to define how the pin is being configured
 */

typedef struct{
	uint8_t GPIO_PinNumber;					/*!< possible value from @GPIO_PIN_NUMBER > */
	uint8_t GPIO_PinMode;					/*!< possible value from @GPIO_PIN_MODE > */
	uint8_t GPIO_PinSpeed;					/*!< possible value from @GPIO_PIN_SPEED > */
	uint8_t GPIO_PinPuPdControl;			/*!< possible value from @GPIO_PIN_PUPD > */
	uint8_t GPIO_PinOPType;					/*!< possible value from @GPIO_PIN_OUTPUT_TYPE > */
	uint8_t GPIO_PinAltFunMOde;
}GPIO_PinConfig_t;

/*
 * Define the Handle Structure for a GPIO pin
 */

typedef struct{

	// First step is to define a pointer to hold the base address of the GPIO peripheral

	GPIO_RegDef_t		*pGPIOx;			/* This point variable of type GPIO_RegDef_t hold the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t	GPIO_PinConfig;		/* This holds GPIO pin configuration settings, this structure is created such that as a format for the user application to fill in */

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin numbers
 */
#define	GPIO_PIN_NO_0			0
#define	GPIO_PIN_NO_1			1
#define	GPIO_PIN_NO_2			2
#define	GPIO_PIN_NO_3			3
#define	GPIO_PIN_NO_4			4
#define	GPIO_PIN_NO_5			5
#define	GPIO_PIN_NO_6			6
#define	GPIO_PIN_NO_7			7
#define	GPIO_PIN_NO_8			8
#define	GPIO_PIN_NO_9			9
#define	GPIO_PIN_NO_10			10
#define	GPIO_PIN_NO_11			11
#define	GPIO_PIN_NO_12			12
#define	GPIO_PIN_NO_13			13
#define	GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODE
 * GPIO port mode
 */
#define GPIO_MODE_IN			0			//00 : Input (reset state)
#define	GPIO_MODE_OUT			1			//01 : General purpose output mode
#define GPIO_MODE_ALTFN			2			//10 : Alternate function mode
#define GPIO_MODE_ANALOG		3			//11 : Analog mode
#define GPIO_MODE_IT_FT			4			//100: Interrupt mode, Falling Edge Trigger (User defined)
#define	GPIO_MODE_IT_RT			5			//101: Interrupt mode, Rising Edge Trigger (User defined)
#define GPIO_MODE_IT_RFT		6			//110: Interrupt mode, Rising and Falling Edge Trigger (User defined)

/*
 * @GPIO_PIN_OUTPUT_TYPE
 * GPIO port output type
 */
#define GPIO_OP_TYPE_PP			0			//0: Output push-pull (reset state)
#define GPIO_OP_TYPE_OD			1			//1: Output open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO port output speed
 */
#define GPIO_SPEED_LOW			0			//00: Low speed
#define GPIO_SPEED_MEDIUM		1			//01: Medium speed
#define GPIO_SPEED_FAST			2			//10: High speed
#define GPIO_SPEED_HIGH			3			//11: Very high speed

/*
 * @GPIO_PIN_PUPD
 * GPIO port pull-up/pull-down
 */
#define GPIO_NO_PUPD			0			//00: No pull-up, pull-down
#define GPIO_PIN_PU				1			//01: Pull-up
#define GPIO_PIN_PD				2			//10: Pull-down

/*
 *
 */

/************************************************************************************************************************
 * Define APIs (Functions) supported by this driver
 * For more information about the APIs check the function definitions
 * All APIs(Functions) prototype are first define in this header file first
 * All APIs(Functions) actual definition/implementation will be define in corresponding source file
*************************************************************************************************************************/

/*
 * Peripheral Clock setup
 */
void 		GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void 		GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void 		GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t		GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t	GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void 		GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void		GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void		GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void 		GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void		GPIO_IRQPriorityConfig(uint8_t IRQNumer, uint32_t IRQPriority);
void 		GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
