/*
 * lcd.c
 *
 *  Created on: Jun 12, 2021
 *      Author: Edward
 */

#include "lcd.h"

/* Private function prototypes */
void send_to_lcd(uint8_t value);
void lcd_enable(void);
void lcd_display_clear(void);
void ms_delay(uint32_t delay_in_ms);
void us_delay(uint32_t delay_in_us);

/* API definitions */
void lcd_init(void) {

	//1. Configure the GPIO pins which are used for lcd connect
	GPIO_Handle_t lcd_signal;

	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	// Initialize all the pin to RESET state
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2. Perform LCD initialization in 4 bit mode

	//2-1. 4 bit mode initialization
	// Write RS pin to indicate writing command: RS = 0, data: RS = 1
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	// Write RW pin to indicate READ: RW = 1 or WRITE: RW = 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	ms_delay(100);			//wait for > 40ms
	send_to_lcd(0x03);		//send command 000011 (RS:RW:DB7:DB6:DB5:DB4)
	ms_delay(10);			//wait for > 4.1ms
	send_to_lcd(0x03);		//send command 000011 (RS:RW:DB7:DB6:DB5:DB4)
	us_delay(200);			//wait for > 100us
	send_to_lcd(0x03);		//send command 000011 (RS:RW:DB7:DB6:DB5:DB4)
	us_delay(200);			//might need this delay if the application not work properly
	send_to_lcd(0x02);		//send command 000010 (RS:RW:DB7:DB6:DB5:DB4)

	//2-2. send initialization command

	// Function set command
	lcd_send_cmd(LCD_CMD_4DL_2N_5X8F);

	// Display ON and Cursor ON
	lcd_send_cmd(LCD_CMD_DON_CURON);

	// Display Clear
	lcd_display_clear();

	// Entry mode set
	lcd_send_cmd(LCD_CMD_INCADD);

}

/* Private function definitions */
void send_to_lcd(uint8_t value) {

	// 1. Put the 4 bit data at D4 - D7 port
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, (( value >> 0 ) & 0x1 ));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, (( value >> 1 ) & 0x1 ));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, (( value >> 2 ) & 0x1 ));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, (( value >> 3 ) & 0x1 ));

	// 2. Enable the LCD with a Logic HIGH to LOW transition to latch the data from MCU GPIO Port to LCD
	lcd_enable();
}

void lcd_send_cmd(uint8_t cmd) {

	// 1. Write RS pin to indicate writing command: RS = 0, data: RS = 1
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	// 2. Write RW pin to indicate READ: RW = 1 or WRITE: RW = 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	send_to_lcd(cmd >> 4);
	send_to_lcd(cmd & 0x0F);
}

void lcd_send_data(uint8_t data) {

	// 1. Write RS pin to indicate writing command: RS = 0, data: RS = 1
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	// 2. Write RW pin to indicate READ: RW = 1 or WRITE: RW = 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	send_to_lcd(data >> 4);			// Send upper nibble first
	send_to_lcd(data & 0x0F);		// Then send lower nibble

}

void lcd_enable(void) {

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	us_delay(100);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	us_delay(100);
}

void lcd_display_clear(void) {

	lcd_send_cmd(LCD_CMD_DIS_CLEAR);
	ms_delay(2);
}

void ms_delay(uint32_t delay_in_ms) {
	for(uint32_t i = 0 ; i < delay_in_ms * 1000 ; i++);
}

void us_delay(uint32_t delay_in_us) {
	for(uint32_t i = 0 ; i < delay_in_us * 1 ; i++);
}
