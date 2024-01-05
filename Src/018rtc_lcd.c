/*
 * 018rtc_lcd.c
 *
 *  Created on: Oct 30, 2023
 *      Author: geroldwilliams
 *      
 * Note:
 *
 * This code Uses I2C1 to communicate with DS1307 RTC
 * to get and set the time and date. Using 7 GPIO pins
 * it will display the time and date to and QAPASS 1602A LCD screen,
 * in 4 bit data mode.
 *
 *      Board Connections
 * STM32 ----> I2C -----> DS1307
 * PB7   ----> SDA -----> A4
 * PB6   ----> SCL -----> A5
 * 
 * STM32 ----> GPIO -----> QAPASS 1602A LCD
 * PD0   ----> GPIO -----> RS(Pin4)
 * PD1   ----> GPIO -----> R/W(Pin5)
 * PD2   ----> GPIO -----> En(Pin6)
 * PD3   ----> GPIO -----> D4(Pin11)
 * PD4   ----> GPIO -----> D5(Pin12)
 * PD5   ----> GPIO -----> D6(Pin13)
 * PD6   ----> GPIO -----> D7(Pin14)
 * 
 */


#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK 	160000000UL //We use the default clock - 16 Mhz

static void mdelay(uint32_t cnt){
	for(uint32_t i=0; i<(cnt *1000);i++);
}
/**
 * @brief      Initializes the systick timer to interrupt at a frequency of tick_hz.
 *
 * @param[in]  tick_hz  - frequency of timer interrupt (.5 sec ==> 1/.5 hz or 2 hz) 
 */
void init_systick_timer(uint32_t tick_hz)
{
	//systick timer registers
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}
/**
 * @brief      Gets the day of week.
 *
 * @param[in]  dayVal - the value of day as a number see: Days of the Week
 *
 * @return     The day of week as a string.
 */
char* get_day_of_week(uint8_t dayVal){
	char* days[] = {"Sun","Mon","Tues","Wed","Thurs","Fri","Sat"};
	return(days[dayVal - 1]);
}
/**
 * @brief      Helper function to convert a number to string
 *
 * @param[in]  num        The number to be converted
 * @param      strBuffer  The string buffer to hold the string version of num
 */
void number_to_string(uint8_t num, char* strBuffer){
	// Add 48 to any number to get the ASCII value and save it to the buffer
	if(num < 10){
		// i.e 4 ==> '04'
		strBuffer[0] = '0';
		strBuffer[1] = num + 48;
	}
	else if(num >= 10 && num < 99){
		strBuffer[0] = (num / 10) + 48;
		strBuffer[1] = (num % 10) + 48;
	}
}
/**
 * @brief      Returns a string representation of a time.
 *
 * @param      rtcTime - RTC time structure
 *
 * @return     String representation of the time.
 */
char* time_to_string(RTC_time_t *rtcTime){
	// hh:mm:ss
	static char timeBuffer[9];
	
	timeBuffer[2] = ':';
	timeBuffer[5] = ':';
	
	number_to_string(rtcTime->hours, &timeBuffer[0]);
	number_to_string(rtcTime->minutes, &timeBuffer[3]);
	number_to_string(rtcTime->seconds, &timeBuffer[6]);
	
	timeBuffer[8] = '\0';
	return timeBuffer;
}
/**
 * @brief      Returns a string representation of a date.
 *
 * @param      rtcDate - RTC date structure
 *
 * @return     String representation of the date.
 */
char* date_to_string(RTC_date_t *rtcDate){
	//mm/dd/yy
	static char dateBuffer[9];

	dateBuffer[2] = '/';
	dateBuffer[5] = '/';

	number_to_string(rtcDate->month, &dateBuffer[0]);
	number_to_string(rtcDate->date, &dateBuffer[3]);
	number_to_string(rtcDate->year, &dateBuffer[6]);

	dateBuffer[8] = '\0';
	return dateBuffer;
}
int main(void){

	// Current time and date variables
	RTC_time_t current_time;
	RTC_date_t current_date;

	//printf("RTC testing!\n");
	if(ds13007_init()){
		// Init has failed
		printf("RTC init has failed!\n");
		// Hang here
		while(1);
	}

	// Initialize the LCD
	LCD_Init();
	//Print test string
	// LCD_PrintString("RTC Testing... ");
	//Wait 2 secs
	mdelay(2000);
	//Clear the LCD and Return the cursor to the home position
	LCD_DisplayClear();
	LCD_DisplayReturnHome(); 

	// generate an interrupt every 1 sec
	init_systick_timer(1);

	printf("RTC has started.\n");

	// Set current time and date
	current_date.day   = SATURDAY;
	current_date.date  = 11;
	current_date.year  = 23;
	current_date.month = 11;

	current_time.hours       = 10;
	current_time.minutes     = 23;
	current_time.seconds     = 20;
	current_time.time_format = TIME_FORMAT_24HRS;

	// Set RTC date and time
	ds13007_set_current_date(&current_date);
	ds13007_set_current_time(&current_time);

	// Get RTC data and time
	ds13007_get_current_date(&current_date);
	ds13007_get_current_time(&current_time);


	// Print current time and date
	// Time
	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		//AM/PM
		//Format 07:00:00 AM 
		am_pm = (current_time.time_format) ? "PM" : "AM";
		// printf("Current time: %s %s\n", time_to_string(&current_time), am_pm); 
		LCD_PrintString("   "); // Using spaces for formatting, should move the cursor position instead
		LCD_PrintString(time_to_string(&current_time));
		LCD_PrintString(am_pm);
	}
	else{
		//24H Format
		//Format 07:00:00
		// printf("Current time = %s\n", time_to_string(&current_time)); 
		LCD_PrintString("    "); // Using spaces for formatting, should move the cursor position instead
		LCD_PrintString(time_to_string(&current_time));
	}
	//Date
	//Format 11/01/23 <Wednesday>
	// printf("Current date: %s <%s>",date_to_string(&current_date), get_day_of_week(current_date.day));
	// Print date information to second row, first column
	LCD_SetCursor(2,1);
	LCD_PrintChar(' ');
	LCD_PrintString(date_to_string(&current_date));
	LCD_PrintString(" <");
	LCD_PrintString(get_day_of_week(current_date.day));
	LCD_PrintChar('>');

	// Hang here, and wait for interrupt
	while(1);
	return 0;
}

void SysTick_Handler(void){

	// Current time and date variables
	RTC_time_t current_time;
	RTC_date_t current_date;

	// Get RTC data and time
	ds13007_get_current_date(&current_date);
	ds13007_get_current_time(&current_time);


	//Clear the LCD and Return the cursor to the home position
	LCD_DisplayClear();
	LCD_DisplayReturnHome(); 

	// Print current time and date
	// Time
	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		//AM/PM
		//Format 07:00:00 AM 
		am_pm = (current_time.time_format) ? "PM" : "AM";
		// printf("Current time: %s %s\n", time_to_string(&current_time), am_pm); 
		LCD_PrintString("   ");
		LCD_PrintString(time_to_string(&current_time));
		LCD_PrintString(am_pm);
	}
	else{
		//24H Format
		//Format 07:00:00
		// printf("Current time: %s\n", time_to_string(&current_time)); 
		LCD_PrintString("    ");
		LCD_PrintString(time_to_string(&current_time));
	}
	//Date
	//Format 11/01/23 <Wednesday>
	// printf("Current date: %s <%s>",date_to_string(&current_date), get_day_of_week(current_date.day));
	// Print date information to second row, first column
	LCD_SetCursor(2,1);
	LCD_PrintChar(' ');
	LCD_PrintString(date_to_string(&current_date));
	LCD_PrintString(" <");
	LCD_PrintString(get_day_of_week(current_date.day));
	LCD_PrintChar('>');
}

