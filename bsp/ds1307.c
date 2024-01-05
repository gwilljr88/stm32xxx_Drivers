/*
 * ds1307.c
 *
 *  Created on: Oct 30, 2023
 *      Author: geroldwilliams
 * 
 */
#include "ds1307.h"
#include <stdio.h>
#include <string.h>

static void gpio_i2c_pins_config(void);
static void ds1307_i2c_config(void);
static void	ds1307_write(uint8_t data, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t binVal);
static uint8_t bcd_to_binary(uint8_t bcdVal);

// Global I2C handler
I2C_Handle_t g_ds1307i2cHandle;

/**
 * @brief      { function to initialize the pins for I2C1 and turn on crystal oscillator for RTC }
 *
 * @return     { Returns the value of CH bit in SECONDS register, if 1 init failed, if 0 init was successful }
 */
uint8_t ds13007_init(void){

	// Initialize the I2C pins
	gpio_i2c_pins_config();

	// Initialize the I2C peripheral
	ds1307_i2c_config();

	// Enable the I2C1 peripheral
	I2C_PeripheralControl(g_ds1307i2cHandle.pI2Cx, ENABLE);

	//Turn on RTC crystal, make clock halt = 0
	ds1307_write(0x00, DS1307_ADDR_SEC);

	//Read back Clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	return( (clock_state >> SECONDS_CH) & 0x01 );
}
/**
 * @brief      { function to write/set time RTC time }
 *
 * @param      pRTCtime - pointer to RTC time structure
 */
void ds13007_set_current_time(RTC_time_t *pRTCtime){
	//Get User settings for time
	uint8_t seconds     = binary_to_bcd(pRTCtime->seconds);
	uint8_t minutes     = binary_to_bcd(pRTCtime->minutes);
	uint8_t hours       = binary_to_bcd(pRTCtime->hours);
	uint8_t time_format = pRTCtime->time_format;

	// Here we make sure not to change SECONDS_CH bit
	seconds &= ~( 1 << SECONDS_CH );

	// Now we write the information to the DS1307 seconds register
	ds1307_write(seconds, DS1307_ADDR_SEC);
	ds1307_write(minutes, DS1307_ADDR_MIN);

	// Check and set the time format for 12H/24H
	if(time_format == TIME_FORMAT_24HRS){
		// 24H time format, clear HOURS_MODE_12_24H bit
		hours &= ~( 1 << HOURS_MODE_12_24H );
	}
	else{
		// 12H time format, set HOURS_MODE_12_24H bit
		hours |= ( 1 << HOURS_MODE_12_24H );

		// Now we muse set/clear the HOURS_AM_PM bit
		if(time_format == TIME_FORMAT_12HRS_PM){
			// Its PM, set HOURS_AM_PM bit in 
			hours |= ( 1 << HOURS_AM_PM );
		}
		else{
			// Its AM, clear HOURS_AM_PM bit
			hours &= ~( 1 << HOURS_AM_PM );
		}
	}
	ds1307_write(hours, DS1307_ADDR_HRS);

}
/**
 * @brief      { function to retrieve and store the time information from the RTC }
 *
 * @param      pRTCtime - pointer to RTC time structure
 */
void ds13007_get_current_time(RTC_time_t *pRTCtime){
	/**
	 * { Get Time information from the RTC }
	 */
	uint8_t seconds = ds1307_read(DS1307_ADDR_SEC);
	uint8_t minutes = ds1307_read(DS1307_ADDR_MIN);
	uint8_t hours   = ds1307_read(DS1307_ADDR_HRS);
	// uint8_t time_format = ds1307_read(DS1307_ADDR_F);

	/**
	 * { Clear SECONDS_CH bit, it is not relevant }
	 */
	seconds &= ~( 1 <<  SECONDS_CH);
	/**
	 * Store current RTC time information
	 */
	pRTCtime->seconds = bcd_to_binary(seconds);
	pRTCtime->minutes = bcd_to_binary(minutes);
	/**
	 * { Check time format }
	 */
	if(hours & ( 1 << HOURS_MODE_12_24H )){
		// 12H format
		// Check HOURS_AM_PM bit
		if( (hours & ( 1 << HOURS_AM_PM )) ){
			// Its PM
			pRTCtime->time_format = TIME_FORMAT_12HRS_PM;
		}
		else{
			// Its AM
			pRTCtime->time_format = TIME_FORMAT_12HRS_AM;
		}
	}
	else{
		// 24H format
		pRTCtime->time_format = TIME_FORMAT_24HRS;
	}
	pRTCtime->hours = bcd_to_binary(hours);
}
/**
 * @brief      { Get user Date information and write it to the RTC }
 *
 * @param      pRTCdate  - pointer to RTC Date structure, contains date information
 */
void ds13007_set_current_date(RTC_date_t *pRTCdate){
	/**
	 * { Set and convert user Date Information }
	 */
	
	uint8_t date  = binary_to_bcd(pRTCdate->date);
	uint8_t day   = binary_to_bcd(pRTCdate->day);
	uint8_t year  = binary_to_bcd(pRTCdate->year);
	uint8_t month = binary_to_bcd(pRTCdate->month);

	/**
	 * { Write the user data to its register  }
	 */
	ds1307_write(date, DS1307_ADDR_DATE);
	ds1307_write(day, DS1307_ADDR_DAY);
	ds1307_write(year, DS1307_ADDR_YEAR);
	ds1307_write(month, DS1307_ADDR_MONTH);
}
/**
 * @brief      { Get the Date information from the RTC }
 *
 * @param      pRTCdate  - pointer to RTC Date structure, contains date information
 */
void ds13007_get_current_date(RTC_date_t *pRTCdate){
	/**
	 * { Get Date information from RTC}
	 */
	uint8_t date  = ds1307_read(DS1307_ADDR_DATE);
	uint8_t day   = ds1307_read(DS1307_ADDR_DAY);
	uint8_t month = ds1307_read(DS1307_ADDR_MONTH);
	uint8_t year  = ds1307_read(DS1307_ADDR_YEAR);

	/**
	 * { Store RTC Date information }
	 */
	pRTCdate->date  = bcd_to_binary(date);
	pRTCdate->day   = bcd_to_binary(day);
	pRTCdate->month = bcd_to_binary(month);
	pRTCdate->year  = bcd_to_binary(year);
}
/**
 * @brief      { function to initialize GPIO pins as I2C }
 */
static void gpio_i2c_pins_config(void){
	// Create a handle for each I2C pin
	GPIO_Handle_t i2c_sda, i2c_scl;

	// Make sure the handles are clear
	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));
	/**
	 * I2C_SDA ==> PB7
	 * I2C_SCL ==> PB6
	 */

	//Here we use the application values to initialize GPIO for I2C see: Application Configurable Items
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode  = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber      = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOType       = GPIO_OUTPUT_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode  = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber      = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOType       = GPIO_OUTPUT_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_scl);
}
/**
 * @brief      { Function to configure the I2C peripheral with application settings }
 */
static void ds1307_i2c_config(void){
	//Here we use the application values to initialize GPIO for I2C see: Application Configurable Items
	g_ds1307i2cHandle.pI2Cx = DS1307_I2C;
	g_ds1307i2cHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_ds1307i2cHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_Init(&g_ds1307i2cHandle);
}
/**
 * @brief      { helper function to write data to the RTC at the given register }
 *
 * @param[in]  data      - data to be written to RTC register
 * @param[in]  reg_addr  - RTC register address
 */
static void	ds1307_write(uint8_t data, uint8_t reg_addr){
	uint8_t txData[2];
	txData[0] = reg_addr;
	txData[1] = data;
	I2C_MasterSendData(&g_ds1307i2cHandle, txData, 2, DS1307_I2C_ADDRESS, I2C_DI_SR);

}
/**
 * @brief      { helper function to read data from the given register }
 *
 * @param[in]  reg_addr -  the RTC register address 
 *
 * @return     { returns the value read at the register address }
 */
static uint8_t ds1307_read(uint8_t reg_addr){
	uint8_t reg_data;
	I2C_MasterSendData(&g_ds1307i2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, I2C_DI_SR);
	I2C_MasterReceiveData(&g_ds1307i2cHandle, &reg_data, 1, DS1307_I2C_ADDRESS, I2C_DI_SR);
	return(reg_data);
}
/**
 * @brief      { converts binary value into bcd value }
 *
 * @param[in]  binVal  The bin value
 *
 * @return     { description_of_the_return_value }
 */
static uint8_t binary_to_bcd(uint8_t binVal){
	uint8_t upper_four = 0;
	uint8_t lower_four = 0;
	uint8_t bcd = 0;
	/**
	 * binary = uuuullll(b) ==>  uuuu | llll (b)
	 * bcd    = uuuullll ÷ 10 | uuuullll % 10
	 *        = (uuuu << 4) | (llll)
	 */
	if(binVal >= 10){
		upper_four = binVal / 10;
		lower_four = binVal % 10;
		bcd = (uint8_t)(upper_four << 4) | (lower_four);
	}
	else{
		bcd = binVal;
	}
	return bcd;
}
/**
 * @brief      { converts bcd into binary value }
 *
 * @param[in]  bcdVal -  The bcd value
 *
 * @return     { the binary value of bcd }
 */
static uint8_t bcd_to_binary(uint8_t bcdVal){
	uint8_t upper_four = 0;
	uint8_t lower_four = 0;
	uint8_t binary = 0;
	/**
	 * bcd = uuuullll(bcd)
	 * binary = (uuuullll >> 4) x 10 + (uuuullll & 0x0F)
	 *        = uuuu + llll
	 * 
	 */
	if(bcdVal >= 10){
		upper_four = (uint8_t)(( bcdVal >> 4 ) & 0x0F) * 10;
		lower_four = (bcdVal & (uint8_t)0x0F);
		binary = upper_four + lower_four;
	}
	else{
		binary = bcdVal;
	}

	return binary;

}


