/*
 * Utils.c
 *
 *  Created on: Feb 8, 2025
 *      Author: oao
 */
#include <stdint.h>
#include "Utils.h"


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
