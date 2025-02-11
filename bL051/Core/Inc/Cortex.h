/*
 * Cortex.h
 *
 *  Created on: Feb 8, 2025
 *      Author: oao
 */

#ifndef CORE_INC_CORTEX_H_
#define CORE_INC_CORTEX_H_

uint32_t corHAL_SYSTICK_Config(uint32_t TicksNumb);
void corHAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);



#endif /* CORE_INC_CORTEX_H_ */
