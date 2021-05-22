/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: May 21, 2021
 *      Author: Edward
 */

#include "stm32f407xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void	SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void	SPI_Init(SPI_Handle_t *pSPIHandle) {

	// Configure the SPI_CR1 register

	uint32_t tempReg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {

		// BIDIMODE bit should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {

		// BIDIMODE bit should be set
		tempReg |= (1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX) {

		// BIDIMODE bit should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);

		// RXONLY bit must be set
		tempReg |= (1 << SPI_CR1_RXONLY);

	}

	//3. Configure the SPI serial clk speed (baud rate)
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. COnfigure the DFF
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// Write the final value of tempReg to actual SPI_CR1 register
	pSPIHandle->pSPIx->CR1 = tempReg;

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
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
void	SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if(pSPIx == SPI1) {
				SPI1_REG_RESET();
			}else if (pSPIx == SPI2) {
				SPI2_REG_RESET();
			}else if (pSPIx == SPI3) {
				SPI3_REG_RESET();
			}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  1 or 0
 *
 * @Note              -  none
 */
uint8_t	SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is a blocking call or we are polling the TXE flag to set
 */
void	SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

	//Use while loop to keep track of Len variable
	while (Len > 0) {
		//1. wait until TXE is set
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//2. Check DFF bit in CR1
		if ( ( pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ) {
			//16 bit DFF
			//1. load the data to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;

		} else {
			//8 bit DFF
			//1. load the data to the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			*pTxBuffer++;
		}

	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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
void	SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
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
void	SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
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
void	SPI_IRQPriorityConfig(uint8_t IRQNumer, uint32_t IRQPriority) {

}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
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
void	SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

}

/*
 * Other peripheral control APIs
 */

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
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
void	SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
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
void	SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
