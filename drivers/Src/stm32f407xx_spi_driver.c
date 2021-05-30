/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: May 21, 2021
 *      Author: Edward
 */

#include "stm32f407xx_spi_driver.h"

//Private function only use in this source file
// Use the keyword "static" to tell the complier that these function are private to this source file, user or other file could
// not call these function to use, otherwise the complier will return error
static	void	spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static	void	spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static	void	spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
			//1. load the data to DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;

		} else {
			//8 bit DFF
			//1. load the data to the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			(uint8_t*)pTxBuffer++;
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

	//Use while loop to keep track of Len variable
	while (Len > 0) {
		//1. wait until TXE is set
		while ( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		//2. Check DFF bit in CR1
		if ( ( pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ) {
			//16 bit DFF
			//1. Read the data from the DR to Rxbuffer

			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;

		} else {
			//8 bit DFF
			//1. load the data to the DR
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			(uint8_t*)pRxBuffer++;
		}

	}

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
	//1. First find out the IPR register
	uint8_t iprx = IRQNumer / 4;
	uint8_t iprx_section = IRQNumer % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

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

	uint8_t temp1, temp2;

	// Check for Interrupt is due to TXE or not
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if ( temp1 && temp2) {
		// Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// Check for Interrupt is due to RXNE or not
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if ( temp1 && temp2) {
		// Handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// Check for Interrupt is due to RXNE or not
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if ( temp1 && temp2) {
		// Handle OverRun error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

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
uint8_t	SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
	//1. Save the Tx Buffer address and Len information in some global variables (Define inside the SPI_Handle_t)
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;

	//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE Flag is set in SR
	pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);

	//4. Data Transmission will be handled by the ISR code

	}

	return state;

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
uint8_t	SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {

	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {

	//1. Save the Rx Buffer address and Len information in some global variables (Define inside the SPI_Handle_t)
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;

	//2. Mark the SPI state as busy in receive so that no other code can take over same SPI peripheral until receive is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//3. Enable the RXNEIE control bit to get interrupt whenever RXNE Flag is set in SR
	pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);

	//4. Data Transmission will be handled by the ISR code

	}

	return state;

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
void	spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	// Check DFF bit in CR1
	if ( ( pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ) {
		//16 bit DFF
		//1. load the data to DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;

	} else {
		//8 bit DFF
		//1. load the data to the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if ( ! pSPIHandle->TxLen) {
		// If Tx Length = 0, close the SPI communication and inform the application that Tx is over

		//1. Clear the TXEIE bit in SPI CR2 register, preventing interrupt from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
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
void	spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	// Check DFF bit in CR1
	if ( ( pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ) {
		//16 bit DFF
		//1. Read the data from DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;

	} else {
		//8 bit DFF
		//1. Read the data from the DR
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if ( ! pSPIHandle->RxLen) {
		// If Rx Length = 0, close the SPI communication and inform the application that Rx is over

		//1. Clear the RXNEIE bit in SPI CR2 register, preventing interrupt from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
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
void	spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;

	//1. Clear the OVR flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
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
void	SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~ ( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
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
void	SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~ ( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
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
void	SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;			// To avoid "unused -value warning during comply process
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
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {
	//This is a weak implementation. The user application may override this function.
}

