/*
 * 017rtc_lcd.c
 *
 *  Created on: Jun 12, 2021
 *      Author: Edward
 */

#include <stdio.h>
#include "ds1307.h"

/*
 * Private function prototypes
 */
char* get_day_of_week(uint8_t i);
char* time_to_string(RTC_time_t *rtc_time);
char* date_to_string(RTC_date_t *rtc_date);
void number_to_string(uint8_t num, char *buf);

int main(void) {

	RTC_date_t current_date;
	RTC_time_t current_time;

	printf("RTC test\n");

	//1. Initialization of DS1307

	if (ds1307_init()) {
		/*
		 * Since ds1307_init() will return either 1 or 0, this loop will be executed if ds1307_init() return 1 which mean init failure
		 */

		printf("RTC Init has failed\n");
		while(1);

	}

	//2. After init succeed, set the DS1307 with current time and date
	current_date.day = MONDAY;
	current_date.date = 14;
	current_date.month = 6;
	current_date.year = 21;

	current_time.hours = 6;
	current_time.minutes = 10;
	current_time.seconds = 30;
	current_time.time_format = TIME_FORMAT_12HRS_PM;

	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	//3. Read back the current_time and current_date, then store them into the current_date and current_time variables
	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);

	//4. Create logic to interpret the read back time and date value and convert them to the format of
	//   date: dd/mm/yy <weekday>
	//   time: hh/mm/ss AM (12hr format), just hh/mm/ss (24hr format)


	//5. First interpret the time and convert the read back value to a human readable string format, then print it out
	char *am_pm;
	//Check the time format first
	if (current_time.time_format != TIME_FORMAT_24HRS) {
		// Time format is 12hr with AM/PM
		am_pm = (current_time.time_format) ? "PM" : "AM";	//Since PM is defined as value 1 and AM is defined as value 0
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);

	} else {
		// Time format is 24hr
		printf("Current time = %s\n", time_to_string(&current_time));
	}

	//6. Second interpret the date and convert the read back value to a human readable string format, then print it out
	printf("Current date = %s %s\n", date_to_string(&current_date), get_day_of_week(current_date.day));

	return 0;
}

/*
 * Private function definitions
 */
char* get_day_of_week(uint8_t i) {
	/*
	 * Return "SUNDAY" 		if i = 1
	 * 		  "MONDAY" 		if i = 2
	 * 		  "TUESDAY" 	if i = 3
	 * 		  "WEDNESDAY" 	if i = 4
	 * 		  "THURSDAY" 	if i = 5
	 * 		  "FRIDAY" 		if i = 6
	 * 		  "SATURDAY" 	if i = 7
	 */

	// Create a string array
	char* days[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY" };

	return days[i-1];

}

char* time_to_string(RTC_time_t *rtc_time) {
	/*
	 * Return the time in the format of hh:mm:ss (i.e. total 9 characters as the last character must be a null or \0 char)
	 * this function will assign each character to the buffer and return it to the main
	 * buf[0] and buf[1] = hh
	 * buf[2] = :
	 * buf[3] and buf[4] = mm
	 * buf[5] = :
	 * buf[6] and buf[7] = ss
	 * buf[8] = null character
	 */

	// First create a buffer of char with size = 9
	static char buf[9];

	buf[2] = ':';
	buf[5] = ':';

	number_to_string(rtc_time->hours, buf);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;

}

char* date_to_string(RTC_date_t *rtc_date) {
	// Implementation similar to time_to_string
	// First create a buffer of char with size = 9
	static char buf[9];

	buf[2] = '/';
	buf[5] = '/';

	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	buf[8] = '\0';

	return buf;
}

void number_to_string(uint8_t num, char *buf) {
	if (num < 10) {
		buf[0] = '0';
		buf[1] = num + 48;			//convert a number to its corresponding ASCII value should add 48 to the number
	} else if (num >=10 && num < 99) {
		buf[0] = (num/10) + 48;
		buf[1] = (num%10) + 48;
	}
}

