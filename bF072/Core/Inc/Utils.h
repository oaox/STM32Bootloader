/*
 * Utils.h
 *
 *  Created on: Feb 8, 2025
 *      Author: oao
 */

#ifndef CORE_INC_UTILS_H_
#define CORE_INC_UTILS_H_

void start_application(unsigned long app_reset_location);
uint32_t byteSwap(uint32_t s);
void uintToHex(uint32_t v, char *buff, uint8_t term);
uint32_t utilDelay(uint32_t delayMs);
bool utilIsFlash(uint32_t a);
uint32_t utilFlashSize(void);



#endif /* CORE_INC_UTILS_H_ */
