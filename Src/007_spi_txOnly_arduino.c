/*
 * 007_spi_txOnly_arduino.c
 *
 *  Created on: Jun 6, 2021
 *      Author: Edward
 */


#include <string.h>
#include "stm32f407xx.h"

/*
 * From STM32F407xx data sheet GPIO-AF table, we first define which GPIO pins used for SPI2 pinout
 * PB15 --> SPI2_MOSI
 * PB14 --> SPI2_MISO (Not used in this exercise)
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 *
*/


void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMOde = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void) {

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;			// This ex. will use SCLK 2Mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;							// This ex. will use hardware slave management

	SPI_Init(&SPI2handle);
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
	for(uint32_t i = 0 ; i < 250000 ; i++);
}

int main() {

	char user_data[] = "Hello World";

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// Make NSS signal internally HIGH and avoids MODF error if using SSM to manage NSS pin
	// SPI_SSIConfig(SPI2, ENABLE);

	// This function is used to initialize the GPIOA0 for the button input
	GPIO_BtnInit();

	/*
	 * Making SSOE (Slave Select Output Enable) = 1 does NSS output enable
	 * The NSS pin is automatically managed by HW
	 * i.e. When SPE = 1, NSS = 0
	 * and  When SPE = 0, NSS = 1
	*/

	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {

		// Wait for button pressed event
		while ( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay();	//Tackle button bouncing issue

		// Enable the SPI2 peripheral (This step must be done after all Initialization is done)
		SPI_PeripheralControl(SPI2, ENABLE);

		// First need to send length information to the Slave (Arduino board)
		uint8_t dataLength = strlen(user_data);
		SPI_SendData(SPI2, &dataLength, 1);

		// User application which sending out data in this example
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//Confirm SPI is not busy before closing the peripheral
		//Just use a while loop to wait for SPI BUSY FLAG reset event
		while ( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		// Always disable the peripheral after the communication task is completed
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}

