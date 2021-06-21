/*
 * lcd.h
 *
 *  Created on: Jun 12, 2021
 *      Author: Edward
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f407xx.h"

/* Application level configurable items */
#define LCD_GPIO_PORT		GPIOD
#define LCD_GPIO_RS			GPIO_PIN_NO_0
#define LCD_GPIO_RW			GPIO_PIN_NO_1
#define LCD_GPIO_EN			GPIO_PIN_NO_2
#define LCD_GPIO_D4			GPIO_PIN_NO_3
#define LCD_GPIO_D5			GPIO_PIN_NO_4
#define LCD_GPIO_D6			GPIO_PIN_NO_5
#define LCD_GPIO_D7			GPIO_PIN_NO_6

/* LCD command */
#define LCD_CMD_4DL_2N_5X8F			0x28
#define LCD_CMD_DON_CURON			0x0E
#define LCD_CMD_INCADD				0x06
#define LCD_CMD_DIS_CLEAR			0x01
#define LCD_CMD_DIS_RETURN_HOME		0x02


/* bsp exposed APIs */
void lcd_init(void);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);

#endif /* LCD_H_ */
