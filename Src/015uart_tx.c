/*
 * 015uart_tx.c
 *
 *  Created on: Jun 11, 2021
 *      Author: Edward
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

/*
 * From STM32F407xx data sheet GPIO-AF table, we first define which GPIO pins used for SPI2 pinout
 * PA2 --> Tx
 * PA3 --> Rx
 * ALT function mode : 7
 *
*/

USART_Handle_t usart2_handle;

//some data
char msg[1024] = "UART Tx testing...\n\r";

void USART2_GPIOInits(void) {

	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMOde = 7;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//Tx
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//Rx
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);

}

void USART2_Inits(void) {

	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart2_handle);
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
	USART2_GPIOInits();
	USART2_Inits();
	USART_PeripheralControl(USART2, ENABLE);

	while(1) {
		// Wait for button pressed event
		while ( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay();	//Tackle button bouncing issue

		USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));
	}




	return 0;
}

