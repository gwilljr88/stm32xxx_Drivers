/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Oct 24, 2023
 *      Author: geroldwilliams
 */

#include "stm32f407xx_rcc_driver.h"


uint32_t AHB_PreScaler[8]  = {2,4,8,16,32,64,128,512};
uint8_t APBx_PreScaler[4]  = {2,4,8,16};



/*********************************************************
 * @fn							- RCC_GetPCLK1Value
 *
 * @brief						- Function to calculate the and return the value of PCLK1
 *
 * @param[in]					-
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- value of PCLK1
 *
 * @note						- This function is based of the STM32F4DISCOVERY using 8 Mhz external oscillator
 *
 * */
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1;
	uint32_t sysclk=0;
	uint8_t clkSource, prebit, ahbp, apb1p;
	//Find the clock source that is being used
	//Shift right by 2 to bring bit 2 and 3 to 0 and 1 position, mask all other bits
	clkSource = (RCC->CFGR >> 2) & 0x3;
	//if equal to 0, clock source is HSI
	if(clkSource == 0){
		sysclk = 16000000; //16 Mhz
	}
	//if equal to 1, clock source is HSE
	else if(clkSource == 1){
		sysclk = 8000000; //8 Mhz
	}
	//if equal to 1, clock source is PLL
	else if(clkSource == 2){
		sysclk = RCC_GetPLLOutputClock();
	}
	//Find the AHB prescaler amount;
	//Right shift by 4 to bring HPRE bit to 0 position, mask all other bits
	prebit = (RCC->CFGR >> 4) & 0xF;
	if(prebit < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[prebit-8];
	}
	//Find the APB1 prescaler amount;
	//Right shift by 10-12 to bring PPRE1 bit to 0-2 position, mask all other bits
	prebit = (RCC->CFGR >> 10) & 0x7;
	if(prebit < 4){
		apb1p = 1;
	}
	else{
		apb1p = APBx_PreScaler[prebit-4];
	}
	//Calculate PCLK1
	pclk1 = (sysclk / ahbp) / apb1p;
	return pclk1;
}
/*********************************************************
 * @fn							- RCC_GetPCLK2Value
 *
 * @brief						- Function to calculate the and return the value of PCLK2
 *
 * @param[in]					-
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- value of PCLK2
 *
 * @note						- This function is based of the STM32F4DISCOVERY using 8 Mhz external oscillator
 *
 * */
uint32_t RCC_GetPCLK2Value(void){
	uint32_t pclk2;
	uint32_t sysclk=0;
	uint8_t clkSource, prebit, ahbp, apb2p;
	//Find the clock source that is being used
	//Shift right by 2 to bring bit 2 and 3 to 0 and 1 position, mask all other bits
	clkSource = (RCC->CFGR >> 2) & 0x3;
	//if equal to 0, clock source is HSI
	if(clkSource == 0){
		sysclk = 16000000; //16 Mhz internal oscillator
	}
	//if equal to 1, clock source is HSE
	else if(clkSource == 1){
		sysclk = 8000000; //8 Mhz external oscillator on STM32F4DISCOVERY
	}
	//if equal to 1, clock source is PLL
	else if(clkSource == 2){
		sysclk = RCC_GetPLLOutputClock();
	}
	//Find the AHB prescaler amount;
	//Right shift by 4 to bring HPRE bit to 0 position, mask all other bits
	prebit = (RCC->CFGR >> 4) & 0xF;
	if(prebit < 0x08){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[prebit-8];
	}
	//Find the APB1 prescaler amount;
	//Right shift by 13-15 to bring PPRE1 bit to 0-2 position, mask all other bits
	prebit = (RCC->CFGR >> 13) & 0x7;
	if(prebit < 4){
		apb2p = 1;
	}
	else{
		apb2p = APBx_PreScaler[prebit-4];
	}
	//Calculate PCLK2
	pclk2 = (sysclk / ahbp) / apb2p;
	return pclk2;
}
/*********************************************************
 * @fn							- RCC_GetPLLOutputClock
 *
 * @brief						- function to calculate and return the value of PLL clock
 *
 * @param[in]					-
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- NONE
 *
 * @note						- This function is based of the STM32F4DISCOVERY using 8 Mhz external oscillator
 *
 * */
uint32_t RCC_GetPLLOutputClock(void){
	uint32_t pllclk;
	uint32_t srcclk=0;
	uint16_t clkSource, vco, n, m, p;

	//Find the clock source that is being used
	//Shift right by 2 to bring bit 22 to 0 position, mask all other bits
	clkSource = ((RCC->PLLCFGR >> 22) & 0x1);
	if(clkSource){
		//HSE Oscillator is selected
		srcclk = 8000000;  //8 Mhz external oscillator on STM32F4DISCOVERY
	}
	else{
		//HSI Clock is selected
		srcclk = 16000000; //16 Mhz internal oscillator
	}

	//Calculate VCO
	//First find the value of n and p
	//Get the value of n
	//Shift right by 6 to bring bit 6 to 0 position, mask all other bits
	n = (RCC->PLLCFGR >> 6) & 0x1FF;
	//Get the value of m
	//Get the lower 5 bits mask all other bits
	m = RCC->PLLCFGR & 0x1F;
	vco = (srcclk * n) / m;

	//Calculate PLL value
	//First get the value of p
	//Shift right by 16 to bring bit 16 to 0 position, mask all other bits
	p = (RCC->PLLCFGR >> 16) & 0x2;
	pllclk = vco / p;

	return pllclk;
}