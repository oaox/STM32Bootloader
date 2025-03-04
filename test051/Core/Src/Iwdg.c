/*
 * Iwdg.c
 *
 *  Created on: Feb 4, 2025
 *      Author: oao
 */

#include <stdint.h>
//#include <stm32l0xx_hal_iwdg.h>

uint16_t *IWDG_KR= (uint16_t*)0x40003000;
uint16_t *IWDG_PR= (uint16_t*)0x40003004;
uint16_t *IWDG_RLR= (uint16_t*)0x40003008;
uint16_t *IWDG_SR= (uint16_t*)0x4000300c;

const uint16_t IWDG_START= 0xcccc;
const uint16_t IWDG_WRITE_ACCESS= 0x5555;
const uint16_t IWDG_PR_PR_0= 2; // div by 16
const uint16_t IWDG_RELOAD= 2315; // about 1 sec
const uint16_t IWDG_REFRESH= 0xaaaa;

void iwdgStart(void)
{
	*(IWDG_KR) = IWDG_START; /* (1) */
	*(IWDG_KR) = IWDG_WRITE_ACCESS; /* (2) */
	*(IWDG_PR) = IWDG_PR_PR_0; /* (3) */
	*(IWDG_RLR) = IWDG_RELOAD; /* (4) */
	while(*IWDG_SR != 0); /* (5) */
}

void iwdgKick(void)
{
	*IWDG_KR = IWDG_REFRESH; /* (6) */
}
