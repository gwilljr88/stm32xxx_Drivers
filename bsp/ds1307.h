/*
 * ds1307.h
 *
 *  Created on: Oct 30, 2023
 *      Author: geroldwilliams
 * 
 */
#ifndef DS1307_H_
#define DS1307_H_

#include "stm32f407xx.h"
/*
* Application Configurable Items
*/
#define DS1307_I2C 				I2C1
#define DS1307_I2C_GPIO_PORT 	GPIOB
#define DS1307_I2C_SDA_PIN 		GPIO_PIN_7
#define DS1307_I2C_SCL_PIN 		GPIO_PIN_6
#define DS1307_I2C_SPEED 		I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD			GPIO_PIN_PU
/*
* Register Addresses
*/
#define DS1307_ADDR_SEC 		0x00
#define DS1307_ADDR_MIN			0x01
#define DS1307_ADDR_HRS 		0x02
#define DS1307_ADDR_DAY 		0x03
#define DS1307_ADDR_DATE 		0x04
#define DS1307_ADDR_MONTH 		0x05
#define DS1307_ADDR_YEAR 		0x06
/*
* Time Formats
*/
#define TIME_FORMAT_12HRS_AM 	0
#define TIME_FORMAT_12HRS_PM 	1
#define TIME_FORMAT_24HRS	 	2
/*
* Device I2C Address
*/
#define DS1307_I2C_ADDRESS	 	0x68
/*
* Days of the Week
*/
#define SUNDAY					1
#define MONDAY					2
#define TUESDAY					3
#define WEDNESDAY				4
#define THURSDAY				5
#define FRIDAY					6
#define SATURDAY				7
/*
* Date Structure
*/
typedef struct{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_date_t;
/*
* Time Structure
*/
typedef struct{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_time_t;
/*
* Init Function
*/
uint8_t ds13007_init(void);
/*
* Get and Set time Functions
*/
void ds13007_set_current_time(RTC_time_t *pRTCtime);
void ds13007_get_current_time(RTC_time_t *pRTCtime);

/*
* Get and Set date Functions
*/
void ds13007_set_current_date(RTC_date_t *pRTCdate);
void ds13007_get_current_date(RTC_date_t *pRTCdate);
/**
 * Register Bit Positions
 */
/**
 * Seconds Bits
 */
#define SECONDS_1_S_BIT03 			0
#define SECONDS_10_S_BIT46 			4
#define SECONDS_CH		 			7
/**
 * Minute Bits
 */
#define MINUTES_1_S_BIT03 			0
#define MINUTES_10_S_BIT46 			4
/**
 * Hours Bits
 */
#define HOURS_1_S_BIT03				0
#define HOURS_10_S_BIT45			4
#define HOURS_AM_PM					5
#define HOURS_MODE_12_24H			6
/**
 * Day Bits
 */
#define DAY_OF_WEEK					0
/**
 * Date Bits
 */
#define DATE_1_S_BIT03				0
#define DATE_10_S_BIT45				4
/**
 * Month Bits
 */
#define MONTH_1_S_BIT03				0
#define MONTH_10_S_BIT				4
/**
 * Year Bits
 */
#define YEAR_1_S_BIT03				0
#define YEAR_10_S_BIT47				4
/**
 * Control Bits
 */
#define CONTROL_RS0					0
#define CONTROL_RS1					1
#define CONTROL_SQWE				4
#define CONTROL_OUT					7
#endif /* DS1307_H_ */