/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: May 29, 2021
 *      Author: Edward
 */

#include "stm32f407xx.h"

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

/*
 * Define a Configuration structure for a I2C peripheral
 * This is a structure variable for user application to define how the pin is being configured
 */

typedef struct {

	uint32_t I2C_SCLSpeed;
	uint8_t	 I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;

}I2C_Config_t;

/*
 * Define the Handle Structure for a I2C peripheral
 */

typedef struct {

	// First step is to define a pointer to hold the base address of the I2C peripheral
	I2C_RegDef_t		*pI2Cx;
	I2C_Config_t		I2C_Config;

}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM100K	100000
#define I2C_SCL_SPEED_FM200K	200000
#define I2C_SCL_SPEED_FM400K	400000

/*
 * @ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/************************************************************************************************************************
 * Define APIs (Functions) supported by this driver
 * For more information about the APIs check the function definitions
 * All APIs(Functions) prototype are first define in this header file first
 * All APIs(Functions) actual definition/implementation will be define in corresponding source file
*************************************************************************************************************************/

/*
 * Peripheral Clock setup
 */

void	I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void	I2C_Init(I2C_Handle_t *pI2CHandle);
void	I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */


/*
 * IRQ Configuration and ISR Handling
 */
void	I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void	I2C_IRQPriorityConfig(uint8_t IRQNumer, uint32_t IRQPriority);

/*
 * Other peripheral control APIs
 */
void	I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t	I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Application Callback
 */
void	I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
