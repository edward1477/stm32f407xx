/*
 * stm32f07xx_gpio_driver.c
 *
 *  Created on: May 16, 2021
 *      Author: Edward
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void 		GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */

/*********************************************************************
 * @fn      		  - GPIO_Init
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
void 		GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	/*
	 * Initialization of a peripheral is to define all the registers in it
	 */

	uint32_t	temp=0;

	//1.	Configure the mode of GPIO pin (Write value to GPIO Mode Register)

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

		//non interrupt mode

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else {

		//interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {

			//1. Configure the FTSR and clear the RTSR for safe reason
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {

			//1. Configure the RTSR and clear the RTSR for safe reason
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {

			//1. Configure both the FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4 );

		//3. Enable the EXTI interrupt delivery using IMR
			EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2.	Configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3.	Configure the pull-up, pull-down settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4.	Configure the output type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5.	Configure the alternate function (if any)

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMOde == GPIO_MODE_ALTFN) {

		uint8_t	temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMOde << (4 * temp2));

	}

	/*	GPIO port mode register,						Address offset: 0x00	*/
	/*	GPIO port output type register,					Address offset: 0x04	*/
	/*	GPIO port output speed register,				Address offset: 0x08	*/
	/*	GPIO port pull-up/pull-down register,			Address offset: 0x0C	*/
	/*	GPIO port input data register,					Address offset: 0x10	*/
	/*	GPIO port output data register,					Address offset: 0x14	*/
	/*	GPIO port bit set/reset register,				Address offset: 0x18	*/
	/*	GPIO port configuration lock register,			Address offset: 0x1C	*/
	/*	AFR[0]: GPIO alternate function low register,	Address offset: 0x20	*/
	/*	AFR[1]: GPIO alternate function high register,	Address offset: 0x24	*/
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
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
void 		GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
//	if(EnorDi == DISABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_REG_RESET();
		}else if (pGPIOx == GPIOB) {
			GPIOB_REG_RESET();
		}else if (pGPIOx == GPIOC) {
			GPIOC_REG_RESET();
		}else if (pGPIOx == GPIOD) {
			GPIOD_REG_RESET();
		}else if (pGPIOx == GPIOE) {
			GPIOE_REG_RESET();
		}else if (pGPIOx == GPIOF) {
			GPIOF_REG_RESET();
		}else if (pGPIOx == GPIOG) {
			GPIOG_REG_RESET();
		}else if (pGPIOx == GPIOH) {
			GPIOH_REG_RESET();
		}else if (pGPIOx == GPIOI) {
			GPIOI_REG_RESET();
		}
}

/*
 * Data Read and Write
 */

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 0 or 1
 *
 * @Note              -
*/
uint8_t		GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t	value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
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
uint16_t	GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
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
void 		GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if (Value == GPIO_PIN_SET) {

		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else {

		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
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
void		GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
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
void		GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);	//Bitwise XOR operation to toggle bit field
}

/*
 * IRQ Configuration and ISR Handling (Processor side)
 */
void 		GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			//program ISER0 registers
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ISER1 registers
			*NVIC_ISER1 |= ( 1 << ( IRQNumber % 32 ) );
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//program ISER2 registers
			*NVIC_ISER2 |= ( 1 << ( IRQNumber % 64 ) );
		}
	} else {
		if (IRQNumber <= 31) {
			//program ICER0 registers
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ICER1 registers
			*NVIC_ICER1 |= ( 1 << ( IRQNumber % 32 ) );
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//program ICER2 registers
			*NVIC_ICER2 |= ( 1 << ( IRQNumber % 64 ) );
		}
	}
}

/*
 *
 */
void		GPIO_IRQPriorityConfig(uint8_t IRQNumer, uint32_t IRQPriority){
		//1. First find out the IPR register
		uint8_t iprx = IRQNumer / 4;
		uint8_t iprx_section = IRQNumer % 4;
		uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
		*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
}

/*
 *
 */
void 		GPIO_IRQHandling(uint8_t PinNumber){
		//Clear the EXTI PR register corresponding to the pin number
		if (EXTI->PR & ( 1 << PinNumber)) {
			//Clear the PR register
			EXTI->PR |= ( 1 << PinNumber);
		}
}
