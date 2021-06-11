/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jun 7, 2021
 *      Author: Edward
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t	 APB1_PreScaler[4] = {2,4,8,16};
uint8_t	 APB2_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void) {

	uint32_t pclk1, sysclk, ahbPre, apb1Pre;
	uint8_t clksrc, temp1, temp2;

	//1. Figure out which clock source is the system using now (HSI, HSE or PLL)
	/*
	 *Clock source status could be figured out from RCC_CFGR
	 *Bits 3:2 SWS: System clock switch status
	 *Set and cleared by hardware to indicate which clock source is used as the system clock.
	 *00: HSI oscillator used as the system clock
	 *01: HSE oscillator used as the system clock
	 *10: PLL used as the system clock
	 *11: not applicable
	 *
	 */

	clksrc = ( RCC->CFGR >> 2 ) & 0x3;	//Bring those 2 bits to the LSB and MASK it

	if ( clksrc == 0 ) {
		sysclk = 16000000;	//HSI as SYSCLK
	} else if (clksrc == 1) {
		sysclk = 8000000;	///HSE as SYSCLK
	} else if ( clksrc == 2) {
		sysclk = RCC_Get_PLL();
	}

	//2. Figure out the AHB Prescaler (i.e. 1,2,4,8,16,32,64,128,256,512)

	temp1 = ( RCC -> CFGR >> 4 ) & 0xF; //Bring HPER 4bits to the LSB and MASK it

	if ( temp1 < 8 ) {
		ahbPre = 1;
	} else {
		ahbPre = AHB_PreScaler[temp1 - 8];
	}

	//3. Figure out the APB1 or APB2 Prescaler (i.e. 1,2,4,8,16), same code for APB1,2
	temp2 = ( RCC -> CFGR >> 10 ) & 0x7; //Bring PPER1 3bits to the LSB and MASK it

	if ( temp2 < 4 ) {
		apb1Pre = 1;
	} else {
		apb1Pre = APB1_PreScaler[temp2 - 4];
	}

	//4. Final step to calculate the pclk1
	pclk1 = ( sysclk / ahbPre ) / apb1Pre;

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void) {
	uint32_t pclk2, sysclk, ahbPre, apb2Pre;
	uint8_t clksrc, temp1, temp2;

	//1. Figure out which clock source is the system using now (HSI, HSE or PLL)
	/*
	 *Clock source status could be figured out from RCC_CFGR
	 *Bits 3:2 SWS: System clock switch status
	 *Set and cleared by hardware to indicate which clock source is used as the system clock.
	 *00: HSI oscillator used as the system clock
	 *01: HSE oscillator used as the system clock
	 *10: PLL used as the system clock
	 *11: not applicable
	 *
	 */

	clksrc = ( RCC->CFGR >> 2 ) & 0x3;	//Bring those 2 bits to the LSB and MASK it

	if ( clksrc == 0 ) {
		sysclk = 16000000;	//HSI as SYSCLK
	} else if (clksrc == 1) {
		sysclk = 8000000;	///HSE as SYSCLK
	} else if ( clksrc == 2) {
		sysclk = RCC_Get_PLL();
	}

	//2. Figure out the AHB Prescaler (i.e. 1,2,4,8,16,32,64,128,256,512)

	temp1 = ( RCC -> CFGR >> 4 ) & 0xF; //Bring HPER 4bits to the LSB and MASK it

	if ( temp1 < 8 ) {
		ahbPre = 1;
	} else {
		ahbPre = AHB_PreScaler[temp1 - 8];
	}

	//3. Figure out the APB1 or APB2 Prescaler (i.e. 1,2,4,8,16), same code for APB1,2
	temp2 = ( RCC -> CFGR >> 13 ) & 0x7; //Bring PPER1 3bits to the LSB and MASK it

	if ( temp2 < 4 ) {
		apb2Pre = 1;
	} else {
		apb2Pre = APB2_PreScaler[temp2 - 4];
	}

	//4. Final step to calculate the pclk2
	pclk2 = ( sysclk / ahbPre ) / apb2Pre;

	return pclk2;
}

uint32_t RCC_Get_PLL(void) {
	return 0;
}
