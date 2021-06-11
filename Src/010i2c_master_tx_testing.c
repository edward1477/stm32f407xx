/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Jun 11, 2021
 *      Author: Edward
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define MY_ADDR		0x61;
#define SLAVE_ADDR	0x68

/*
 * From STM32F407xx data sheet GPIO-AF table, we first define which GPIO pins used for SPI2 pinout
 * PB6 --> SCL
 * PB9 --> SDA
 * ALT function mode : 4
 *
*/

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

void I2C1_GPIOInits(void) {

	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMOde = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void) {

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM100K;

	I2C_Init(&I2C1Handle);

}

void GPIO_BtnInit(void) {

	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// This step already implemented in the driver layer
	//GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioBtn);
}

void delay(void){
	for(uint32_t i = 0 ; i < 500000 ; i++);
}

int main(void) {

	GPIO_BtnInit();

	//I2C pin inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while (1) {
		// Wait for button pressed event
		while ( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay();	//Tackle button bouncing issue

		//Master send some data to the slave
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR, I2C_DISABLE_SR);
	}

	return 0;
}
