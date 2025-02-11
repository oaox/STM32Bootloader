/*
 * Crc.h
 *
 *  Created on: Feb 7, 2025
 *      Author: oao
 */

#ifndef CORE_INC_CRC_H_
#define CORE_INC_CRC_H_
uint32_t crcCalcCrc(bool *correct);
HAL_StatusTypeDef crcHAL_CRCEx_Polynomial_Set(CRC_HandleTypeDef *hcrc, uint32_t Pol, uint32_t PolyLength);
HAL_StatusTypeDef crcHAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef crcHAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
uint32_t crcHAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);


#endif /* CORE_INC_CRC_H_ */
