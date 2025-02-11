/*
 * Hal.h
 *
 *  Created on: Feb 8, 2025
 *      Author: oao
 */

#ifndef CORE_INC_HAL_H_
#define CORE_INC_HAL_H_


extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;
uint32_t halHAL_GetTick(void);
HAL_StatusTypeDef halHAL_Init(void);
HAL_StatusTypeDef halHAL_InitTick(uint32_t TickPriority);
void halHAL_IncTick(void);
#endif /* CORE_INC_HAL_H_ */
