/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: May 21, 2021
 *      Author: Edward
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Define a Configuration structure for a SPI peripheral
 * This is a structure variable for user application to define how the pin is being configured
 */

typedef struct {

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPIConfig_t;

/*
 * Define the Handle Structure for a SPI peripheral
 */

typedef struct {

	// First step is to define a pointer to hold the base address of the SPI peripheral
	SPI_RegDef_t		*pSPIx;
	SPIConfig_t			SPIConfig;
	uint8_t				*pTxBuffer;		/* !< To store the app. Tx buffer address > */
	uint8_t				*pRxBuffer;		/* !< To store the app. Rx buffer address > */
	uint32_t			TxLen;			/* !< To store Tx buffer length > */
	uint32_t			RxLen;			/* !< To store Rx buffer length > */
	uint8_t				TxState;		/* !< To store Tx state > */
	uint8_t				RxState;		/* !< To store Rx state > */

}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_SIMPLEX_RX	3

/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS				0	//default value
#define SPI_DFF_16BITS				1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0

/*
 * @SPI_SSM
 */

#define SPI_SSM_EN					1
#define SPI_SSM_DI					0

/*
 * @SPI related status flag definition
 */

#define SPI_TXE_FLAG				( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG				( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG				( 1 << SPI_SR_BSY )

/*
 * @SPI Application state definition
 */

#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

/*
 * @Possible SPI Application event definition
 */

#define SPI_EVENT_TX_CMPLT			0
#define SPI_EVENT_RX_CMPLT			1
#define SPI_EVENT_OVR_ERR			2
#define SPI_EVENT_CRC_ERR			3

/************************************************************************************************************************
 * Define APIs (Functions) supported by this driver
 * For more information about the APIs check the function definitions
 * All APIs(Functions) prototype are first define in this header file first
 * All APIs(Functions) actual definition/implementation will be define in corresponding source file
*************************************************************************************************************************/

/*
 * Peripheral Clock setup
 */

void	SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void	SPI_Init(SPI_Handle_t *pSPIHandle);
void	SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */

void	SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void	SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t	SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t	SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */

void	SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void	SPI_IRQPriorityConfig(uint8_t IRQNumer, uint32_t IRQPriority);
void	SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral control APIs
 */

void	SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void	SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void	SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t	SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void	SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void	SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void	SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
