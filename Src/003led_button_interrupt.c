/*
 * 002led_button.c
 *
 *  Created on: May 18, 2021
 *      Author: Edward
 */

#include <stm32f407xx_gpio_driver.h> /*This code also could be happened at the end of stm32f407xx.h file */
#include <string.h>
#include "stm32f407xx.h"

#define HIGH 		ENABLE
#define BTN_PRESSED HIGH

void delay(void){
	for(uint32_t i = 0 ; i < 300000 ; i++);
}

int main(void){

	//1. Create a structure variable to handle the target peripheral (e.g. GPIO in this case)
	GPIO_Handle_t	Gpioled, GpioBtn;

	memset(&Gpioled,0,sizeof(Gpioled));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	//2. Initialization of GPIO which handle the LED
	Gpioled.pGPIOx = GPIOD;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&Gpioled);

	//3. Initialization of GPIO which handle the Button
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioBtn);

	//IRQ Configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	//IRQ Handler Definition

	//4. After all initialization steps, we could write code to implement toggle LED
	while(1);

	return 0;
}

//IRQ Handler Definition
void EXTI0_IRQHandler(void) {
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	delay();
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}
