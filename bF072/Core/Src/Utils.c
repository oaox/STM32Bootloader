/*
 * Utils.c
 *
 *  Created on: Feb 8, 2025
 *      Author: oao
 */
#include <stdint.h>
#include <stdbool.h>
#include "Utils.h"
#include "Flash.h"


__attribute__((naked))
extern void start_application(unsigned long app_reset_location)
{
	asm(" ldr r0, [r0,#4]"); // get the new program counter value from the program's reset vector

	asm(" blx r0");          // jump to the start address

}

uint32_t byteSwap(uint32_t s)
{
	uint32_t d;
	uint8_t *src;
	uint8_t *dest;
	src= (uint8_t*)&s;
	dest= (uint8_t *)&d +3;
	for (int i=0; i<4; i++) *dest--= *src++;
	return d;
}

void uintToHex(uint32_t v, char *buff, uint8_t term)
{
	uint8_t ch;
	int shift;
	for(int i=0; i<8; i++) {
		shift= 4*(7-i);
		ch= (uint8_t)(v>>shift) & 0x0f;
		ch += '0';
		if (ch > '9') ch= ch + ('A'-'9'-1);
		buff[i]= ch;
	}
	buff[8]= term;
}

uint32_t utilDelay(uint32_t delayMs)
{
	static uint32_t counter;
	if (delayMs != 0) counter= delayMs;
	if ( ((*(uint32_t *)0xE000E010) & 0x10000) != 0 ){
		if (counter > 0) counter--;
	}
	return counter;
}

bool utilIsFlash(uint32_t a)
{
	if ((a < AppAddr) || (a > (0x8000000  + utilFlashSize() )) ) return false;
	return true;
}

uint32_t utilFlashSize(void)
{
	uint32_t size;
	size = *(uint16_t*) 0x1ffff7cc; // size in kiBit
	size <<= 10;
	return size;
}

