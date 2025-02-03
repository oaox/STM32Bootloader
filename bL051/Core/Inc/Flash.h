/*
 * Flash.h
 *
 *  Created on: Nov 18, 2024
 *      Author: oao
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_


#define MAX_FLASH_LINE_LEN 150
#define FLASH_NOT_HEX -1
#define FLASH_WRONG_LENGTH -2
#define FLASH_NOTIMPLEMENTED -3
#define FLASH_TIMEOUT -4
#define FLASH_FIN -5
#define FLASH_CS_ERROR -6
#define FLASH_ADDR_RANGE -7
#define FLASH_RECORD_SIZE -8


extern const uint32_t AppAddr;

int flashReadOneLine(void);
int flashDecodeLine(void);
int flashWriteData(void);


#endif /* INC_FLASH_H_ */
