/*
 * 001led_toggle.c
 *
 *  Created on: May 18, 2021
 *      Author: Edward
 */

#include "stm32f407xx.h"
#include "stm32f07xx_gpio_driver.h" /*This code also could be happened at the end of stm32f407xx.h file */

void delay(void){
	for(uint32_t i = 0 ; i < 500000 ; i++);
}

int main(void){

	//1. Create a structure variable to handle the target peripheral (e.g. GPIO in this case)
	GPIO_Handle_t	Gpioled;

	//2. Initialize the created structure variable
	Gpioled.pGPIOx = GPIOD;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//3. Enable the clock of target peripheral before Initialize it
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//4. Initialize the peripheral by using API built ourselves
	GPIO_Init(&Gpioled);

	//5. After all initialization steps, we could write code to implement toggle LED
	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
