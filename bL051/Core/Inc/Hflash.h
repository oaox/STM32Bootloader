/*
 * Hflash.c
 *
 *  Created on: Feb 8, 2025
 *      Author: oao
 */

#ifndef CORE_INC_HFLASH_C_
#define CORE_INC_HFLASH_C_

extern FLASH_ProcessTypeDef pFlash;

HAL_StatusTypeDef hfHAL_FLASH_Lock(void);
HAL_StatusTypeDef hfHAL_FLASH_Unlock(void);



__RAM_FUNC HAL_StatusTypeDef hrfHAL_FLASHEx_HalfPageProgram(uint32_t Address, uint32_t* pBuffer);


void hfxFLASH_PageErase(uint32_t PageAddress);


#endif /* CORE_INC_HFLASH_C_ */
