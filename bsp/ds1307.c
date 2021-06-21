/*
 * ds1307.c
 *
 *  Created on: Jun 12, 2021
 *      Author: Edward
 */

#include <stdint.h>
#include <string.h>
#include "ds1307.h"

/* Private global variables */
I2C_Handle_t g_ds1307_I2CHandle;

/* Private function prototypes */
static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t value);
static uint8_t bcd_to_binary(uint8_t value);

uint8_t ds1307_init(void) {

	//1. Init the i2c pins of STM32 which connected to ds1307
	ds1307_i2c_pin_config();

	//2. Init the i2c peripheral
	ds1307_i2c_config();

	//3. Enable the I2C peripheral
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	//4. Need to write DS1307 00h register bit-8 (CH) = 0, Master(STM32) write to Slave(DS1307) operation
	ds1307_write(0x00,DS1307_ADDR_SEC);

	//5. Read back the DS1307 00h register to confirm whether step 4 is completed successfully
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	return (( clock_state >> 7 ) & 0x1 );	// Shift the 7th bit (MSB) of read-back value to 1st bit(LSB) and test it is 0 or 1

	/*
	 * This function return
	 * 0 : DS1307 init completed successfully
	 * 1 : DS1307 init fail
	 */

}

void ds1307_set_current_time(RTC_time_t *rtc_time) {

	//1. Write the DS1307 00h register with seconds information and make sure 7th bit is 0
	uint8_t seconds, hrs;

	seconds = binary_to_bcd(rtc_time->seconds);		//ds1307 accpet bcd format not binary format
	seconds &= ~( 1 << 7 );							//Make sure 7th bit is 0 by clearing operation
	ds1307_write(seconds, DS1307_ADDR_SEC);

	//2. Write the DS1307 01h register with minutes information
	ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);

	//3. Wire the DS1307 02h register with hours information
	hrs = binary_to_bcd(rtc_time->hours);

	if (rtc_time->time_format == TIME_FORMAT_24HRS) {
		hrs &= ( 1 << 6 );	//24hrs format need to clear 6th bit of the hrs register
	} else {
		hrs |= ( 1 << 6 );	//12 hrs format need to set 6th bit of the hrs register
		hrs = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? hrs | ( 1 << 5 ) : hrs & ~( 1 << 5 );
	}

	ds1307_write(hrs, DS1307_ADDR_HRS);

}

void ds1307_get_current_time(RTC_time_t *rtc_time) {

	uint8_t seconds, hrs;

	//1. Reading seconds info from DS1307
	seconds = ds1307_read(DS1307_ADDR_SEC);

	// clear bit-7 as it is not relevant to the seconds' value
	seconds &= ~( 1 << 7 );

	// store the value to the user define structure variable
	rtc_time->seconds = bcd_to_binary(seconds);

	//2. Reading minutes info from DS1307
	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	//3. Reading hrs info from DS1307
	hrs = ds1307_read(DS1307_ADDR_HRS);

	// Check the time format is 12hrs or 24hrs from bit-6
	if (hrs & ( 1 << 6 )) {
		//12 hr format
		rtc_time->time_format = ! ((hrs & ( 1 << 5 )) == 0);	//Check bit 5  for AM/PM
		hrs &= ~(0x3 << 5 );	//clear bit 5 and 6
	} else {
		//24 hr format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}

	rtc_time->hours = bcd_to_binary(hrs);

}

void ds1307_set_current_date(RTC_date_t *rtc_date) {

	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);

	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);

	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);

	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);

}

void ds1307_get_current_date(RTC_date_t *rtc_date) {

	rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));

	rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));

	rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));

	rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));

}

static void ds1307_i2c_pin_config() {

	/*
	 * I2C1_SCL -> PB6
	 * I2C1_SDA -> PB7 or PB9
	 */

	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_scl,0,sizeof(i2c_scl));
	memset(&i2c_sda,0,sizeof(i2c_sda));

	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMOde = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMOde = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&i2c_scl);

}

static void ds1307_i2c_config() {

	g_ds1307_I2CHandle.pI2Cx = DS1307_I2C;
	g_ds1307_I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_ds1307_I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&g_ds1307_I2CHandle);

}

static void ds1307_write(uint8_t value, uint8_t reg_addr) {

	uint8_t tx[2];		//Create an array of size 2 to store the first 2 bytes' info which are going to be sent to the Slave
	tx[0] = reg_addr;	//tx[0] store the 2nd byte (Slave register address) to be sent
	tx[1] = value;		//tx[1] store the 3rd byte (Actual Data) to be sent
	I2C_MasterSendData(&g_ds1307_I2CHandle, tx, 2, DS1307_I2C_ADDRESS, 0);
}

static uint8_t ds1307_read(uint8_t reg_addr) {

	uint8_t data;
	I2C_MasterSendData(&g_ds1307_I2CHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, 0);
	I2C_MasterReceiveData(&g_ds1307_I2CHandle, &data, 1, DS1307_I2C_ADDRESS, 0);

	return data;


}

static uint8_t binary_to_bcd(uint8_t value) {

	uint8_t m, n;	// m = 10th digit value, n = single digit value
	uint8_t bcd;

	bcd = value;

	if ( value >= 10 ) {
		m = value / 10;
		n = value % 10;
		bcd = (uint8_t) (( m << 4 ) | n);
	}

	return bcd;
}

static uint8_t bcd_to_binary(uint8_t value) {

	uint8_t m, n;	// m = 10th digit value, n = single digit value

	m = (uint8_t)(( value >> 4 ) * 10);
	n = value & (uint8_t)0x0F;

	return m + n;
}









