/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: May 29, 2021
 *      Author: Edward
 */

#include "stm32f407xx_i2c_driver.h"

/*
 * Private function prototypes
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void	I2C_MasterHandleTXEInterrupt (I2C_Handle_t *pI2CHandle);
static void	I2C_MasterHandleRXNEInterrupt (I2C_Handle_t *pI2CHandle);

/*
 * private function definitions
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START );
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr) {
	slaveAddr = slaveAddr << 1;		//I2C slave address is 7 bits, we need to right shift this address and reserve the LSB for R/W
	slaveAddr &= ~(1);				// R/W bit = 0: WRITE, 1:READ, this code is clearing R/W (i.e. make R/W = 0)
	pI2Cx->DR = slaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {

	uint8_t dummyRead;

	//Check for device mode
	if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ) ) {
		//Device is in Master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			//Device is in Receiving
			if (pI2CHandle->RxSize == 1) {
				//1. Disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//2. Clear the ADDR Flag (Read SR1 and SR2)
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;

			}
		} else {
			//Device is NOT in I2C receiving
			//Just simply clear the ADDR Flag (Read SR1 and SR2)
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;

		}
	} else {
		//Device is in slave mode
		//Just simply clear the ADDR Flag (Read SR1 and SR2)
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;

	}
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr) {
	slaveAddr = slaveAddr << 1;		//I2C slave address is 7 bits, we need to right shift this address and reserve the LSB for R/W
	slaveAddr |= 1;				// R/W bit = 1: WRITE, 1:READ, this code is clearing R/W (i.e. make R/W = 0)
	pI2Cx->DR = slaveAddr;
}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void	I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             -
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void	I2C_Init(I2C_Handle_t *pI2CHandle) {


	/*
	 * Note: All the below initialization steps must be done when the peripheral is DISABLED
	 * 		 in the control register (By Default, STM32 I2C peripheral is DISBALED,
	 * 		 it might not be TRUE for other Mfg's MCU, you need to check.
	 */
	uint32_t tempReg = 0;

	//1. Enable the peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//2. Configure the mode (Standard/Fast etc), done in step 6

	//3. Configure the speed of the serial clock (SCL)
	/*
	 * To configure SCL, we need to configure CR2 FREQ[5:0] bit field
	 * This FREQ must equal to the APB1 CLK as I2C is hang on APB1 bus
	 *
	 */
	tempReg = 0;	//reset the temp register to all 0
	tempReg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = ( tempReg & 0x3F );

	//4. Configure the device address (Applicable when device is slave)
	tempReg = 0;	//reset the temp register to all 0
	tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempReg |= ( 1 << 14 );		//Bit 14 Should always be kept at 1 by software
	pI2CHandle->pI2Cx->OAR1 = tempReg;

	//5. Enable the Acking (ACK is in CR1 register)
	tempReg = 0;	//reset the temp register to all 0
	tempReg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempReg;

	//6. Configure CCR register
	//(i) Configure the mode (standard, fast etc) and configure CCR register
	//(ii) Calculate CCR value

	uint16_t ccrValue= 0;
	tempReg = 0;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM100K) {

		//mode is standard mode
		ccrValue = ( RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempReg |= ( ccrValue & 0xFFF );

	} else {
		//mode is fast mode
		tempReg |= ( 1 << 15 );		//CCR bit 15 = (0:Standard mode, 1:Fast mode)
		tempReg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14 );

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccrValue = ( RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );

		} else {
			ccrValue = ( RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}

		tempReg |= ( ccrValue & 0xFFF );

	}

	pI2CHandle->pI2Cx->CCR = tempReg;

	//7. Configure the rise time for I2C pins

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM100K) {

			//mode is standard mode
			tempReg = ( RCC_GetPCLK1Value() / 1000000U ) + 1;

		} else {
			//mode is fast mode
			tempReg = (( RCC_GetPCLK1Value() * 300 ) / 1000000000U ) + 1;
		}

	pI2CHandle->pI2Cx->TRISE = ( tempReg & 0x3F );
}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void	I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if(pI2Cx == I2C1) {
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	}else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
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
uint8_t	I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void	I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr) {
	//1. Generate the START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the salve with r/nw bit set to WRITE (0) (total 8bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until Length becomes 0
	while(len > 0) {
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE )) {
			pI2CHandle->pI2Cx->DR = *pTxbuffer;
			pTxbuffer++;
			len--;
		}
	}

	//7. When Length becomes 0, wait for TxE = 1 and BTF = 1 before generating the STOP condition
	//   Note: TxE = 1, BTF = 1, means both SR and DR are empty and next transmission should begin
	//   When BTF = 1, SCL will be stretched (pulled to LOW)
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	//   Note: Generating STOP, automatically clears the BTF
	if (Sr == I2C_DISABLE_SR) {
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void	I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr) {
	//1. Generate the START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the salve with r/nw bit set to READ (1) (total 8bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Procedure to read ONLY 1 BYTE from slave
	if (len == 1) {
		//Disable Acking (i.e. Make ACK = 0)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Generate STOP condition
		if (Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE Flag = 1
		while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Read the data from DR register to Rxbuffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	//6. Procedure to read data from slave when len > 1
	if (len > 1) {


		//Read the data until len become 0
		for ( uint32_t i = len ; i > 0 ; i--) {
			//wait until RXNE becomes 1
			while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (i == 2) {
				//Clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Generate STOP condition
				if (Sr == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//Read the data from data register to pRxbuffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer address
			pRxbuffer++;
		}
	}

	//Re enable Acking
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint8_t	I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr) {
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint8_t	I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr) {
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

void	I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {
	pI2Cx->DR = data;
}

uint8_t	I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {
	return (uint8_t) pI2Cx->DR;
}


/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void	I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
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
 * @fn      		  - I2C_IRQPriorityConfig
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
void	I2C_IRQPriorityConfig(uint8_t IRQNumer, uint32_t IRQPriority) {
	//1. First find out the IPR register
	uint8_t iprx = IRQNumer / 4;
	uint8_t iprx_section = IRQNumer % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}

void I2C_CloseReceiveData (I2C_Handle_t *pI2CHandle) {
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//Reset the pI2CHandle parameters
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData (I2C_Handle_t *pI2CHandle) {
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//Reset the pI2CHandle parameters
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}


void	I2C_MasterHandleTXEInterrupt (I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->TxLen > 0) {				// confirm TxBuffer is not empty
		//1. Load the data to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pRxBuffer);

		//2. Decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the TxBuffer address
		pI2CHandle->pTxBuffer++;
	}
}

void	I2C_MasterHandleRXNEInterrupt (I2C_Handle_t *pI2CHandle) {
	//case of receiving 1 byte ONLY
	if (pI2CHandle->RxSize == 1 ) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}

	//case of receiving more than 1 byte
	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2) {
			//Clear the ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}

		//Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	//case of RxLeb = 0, completion of Reading data
	if (pI2CHandle->RxLen == 0) {
		//Close the I2C data reception and notify the application

		//1. Generate the STOP condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2. Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
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
void	I2C_EV_IRQHadnling(I2C_Handle_t *pI2CHandle) {

	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;	//Create temp. register to check various Bit status

	//First check whether the ITEVTEN and ITBUFFEN is SET to make usre IRQ is really enabled
	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );

/***********************************Check for SB EVENT*****************************************/

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	//SB event is confirmed if ITEVTEN and SB Bit is SET
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );	//Check if interrupt is raised by SB bit SET
	if (temp1 && temp3) {

		//Interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always 0
		//In this block, we could executed the address phase as SB is successfully set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

/***********************************Check for ADDR EVENT*****************************************/

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );	//Check if interrupt is raised by ADDR bit SET
	if (temp1 && temp3) {
		//ADDR flag is set
		//Once ADDR flag is SET, we need to clear it by software
		I2C_ClearADDRFlag(pI2CHandle);
	}

/***********************************Check for BTF EVENT*****************************************/

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );	//Check if interrupt is raised by BTF bit SET
	if (temp1 && temp3) {
		//BTF flag is set
		//BTF status is indicator for closing a Transmission if both TXE and BTF are set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			// BTF, TXE = 1

			if(pI2CHandle->TxLen == 0) {

				//1. Generate the STOP condition
				if (pI2CHandle->Sr == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				//2. reset all the member elements of the handle structure
				I2C_CloseSendData(pI2CHandle);

				//3. notify the application about the transmission complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

			}

		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			;	//nothing need to do in this case
		}
	}

/***********************************Check for STOPF EVENT*****************************************/

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );	//Check if interrupt is raised by STOPF bit SET
	if (temp1 && temp3) {
		//STOPF flag is set
		//The software need to clear the STOPF by 1) Reading SR1 and 2)Write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;	//Just perform a dummy write operation to CR1, bitwise OR operation will retain all the content of CR1

		//Notify the application that STOP generated by the MASTER is being detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

/***********************************Check for TXE EVENT*****************************************/

	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TxE );	//Check if interrupt is raised by TXE bit SET
	if (temp1 && temp2 && temp3) {
		//TXE flag is set
		//Indicating that DR register is empty, the software should write a byte of data to DR for sending out

		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {	// confirm device is in Master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {	// confirm device is in transmission operation

				I2C_MasterHandleTXEInterrupt(pI2CHandle);

			}
		} else {
			//slave
			// Check if slave is really in transmitter mode
			if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA )) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

/***********************************Check for RXNE EVENT*****************************************/

	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RxNE );	//Check if interrupt is raised by RXNE bit SET
	if (temp1 && temp2 && temp3) {
		//RXNE flag is set

		//Check the device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)) {
			// The device mode is Master
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {	// confirm device is in transmission operation

				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else {
			//slave
			// Check if slave is really in receiver mode
			if ( ! ( pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA ) ) ) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void 	I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

/*
 * Other peripheral control APIs
 */

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
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
void	I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
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

uint8_t	I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR2 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}
*/

void 	I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == I2C_ACK_ENABLE) {
		//Enable the ACK
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK );

	} else {
		//Disable the ACK
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}

}

/*********************************************************************
 * @fn      		  - I2C_ApplicationEventCallback
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
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {
	//This is a weak implementation. The user application may override this function.
}
