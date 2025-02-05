/*
 * Rcc.h
 *
 *  Created on: Feb 5, 2025
 *      Author: oao
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

uint32_t rccHAL_RCC_GetSysClockFreq(void);
void rccHAL_RCC_GetClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t *pFLatency);
HAL_StatusTypeDef rccHAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
HAL_StatusTypeDef rccHAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
HAL_StatusTypeDef rccHAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency);
void rccHAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
uint32_t rccHAL_RCC_GetPCLK1Freq(void);
uint32_t rccHAL_RCC_GetHCLKFreq(void);
uint32_t rccHAL_RCC_GetPCLK2Freq(void);


#endif /* INC_RCC_H_ */
