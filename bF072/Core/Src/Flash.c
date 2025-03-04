/*
 * Flash.c
 *
 *  Created on: Nov 17, 2024
 *      Author: oao
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"
//#include "stm32f0xx_hal_flash_ramfunc.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_flash_ex.h"
#include "Uart.h"
#include "Flash.h"
#include "Utils.h"
void FLASH_PageErase(uint32_t PageAddress);

//#include "Hal.h"
//#include "Hflash.h"

const uint32_t AppAddr = 0x8005000;
static uint64_t data64[9];
//static uint32_t *data32= (uint32_t *)data64;
//static uint16_t *data16= (uint16_t *)data64;
static uint8_t hexLine[MAX_FLASH_LINE_LEN + 1];
//static uint8_t byteLine[(MAX_FLASH_LINE_LEN)/2+1];
static uint8_t *byteLine = (uint8_t*) data64;
//static uint8_t *data= (uint8_t *)data32;

typedef struct {
	uint32_t address;
	uint32_t extAddr;
	int frameType;
	int byteCount;
	uint8_t cs;
} flashParametres;

flashParametres flashPar;

/**
 * @brief Converts an upper case hex digit string to an unsigned 16 bit value
 * @param s The up to four hex digits string
 * @param length The number of characters to convert
 * @return The converted value, -1 if error i.e. a digit was not a hex digit
 */
int32_t flashHexToU16(char *s, int length) {
	int32_t r;
	int i;
	r = 0;
	for (i = 0; i < length; i++) {
		if (s[i] == 0)
			return r;
		r = r * 16;
		if ((s[i] >= '0') && (s[i] <= '9')) {
			r = r + (s[i] - '0');
		} else if ((s[i] >= 'A') && (s[i] <= 'F'))
			r = r + (s[i] - 'A') + 10;
		else
			return -1;
	}
	return r;
}
/**
 * @brief Converts an intel hex string into a flash line structure
 * @param f A flash line structure to be filled in
 * @param length The length of the Intel hex string
 * @param data The string to be converted
 * @return 0 if OK
 */

/**
 * @brief Converts one Intel hex line and sends to boot loader(s).
 *
 * @param kbd The keyboard to program [1..5]
 * @param card The card of the keyboard to program, not used [1..8]
 * @param length The number of bytes to program
 * @param data The data array to program
 * @return 0 if OK
 */

static int charToInt(uint8_t ch) {
	int v;
	v = (int) (ch - '0');
	if (v > 9)
		v = (int) (ch - 'A') + 10;
	if ((v > 15) || (v < 0))
		return -1;
	return v;
}

int hexToInt(uint8_t *cbuff) {
	int val = 0;
	int tmp;
	for (int i = 0; i < 5; i++) {
		if (cbuff[i] == '\0')
			break;
		tmp = charToInt(cbuff[i]);
		if (tmp < 0)
			return FLASH_NOT_HEX;
		val <<= 4;
		val = val | tmp;
	}
	return val;
}

int hexToByte(uint8_t *b) {
	int val = 0;
	int tmp;
	tmp = charToInt(b[0]);
	if (tmp < 0)
		return FLASH_NOT_HEX;
	val = tmp;
	tmp = charToInt(b[1]);
	if (tmp < 0)
		return FLASH_NOT_HEX;
	val = (val << 4) | tmp;
	return val;
}

int flashReadOneLine(void) {
	int err = 0;
	int idx;
	int r;
	uint32_t timeout;
	hexLine[0] = 0;
	do {
		r = uartGetOneChar(hexLine);
	} while (hexLine[0] != ':');
	//got message start
	utilDelay(10);
	//get data byte count
	idx = 0; // We know we gotr a colon, so just write over it
	while (1) {
		if (utilDelay(0) == 0) {
			err = FLASH_TIMEOUT;
			goto exit;
		}
		r = uartGetOneChar(hexLine + idx);
		if (r < 0)
			continue;
		utilDelay(10);

		if (hexLine[idx] == '\n') {
			err = idx; // length including \n
			goto exit;
		}
		idx++;
	}
	exit: return err;
}

int flashDecodeLine(void) {
	int idx = 0;
	int err;
	uint16_t byteCount;
	uint32_t address;
	uint16_t type;
	uint8_t cs;
	uint8_t sum;
	uint8_t cbuff[5];
	int tmp;

	// data byte count
	cbuff[0] = hexLine[0];
	cbuff[1] = hexLine[1];
	cbuff[2] = '\0';
	byteCount = hexToInt(cbuff);
	if (byteCount < 0) {
		err = FLASH_NOT_HEX;
		goto exit;
	}
	sum = 0;
	for (int i = 0; i < (byteCount + 4); i++) {
		tmp = hexToByte(&hexLine[2 * i]);
		if (tmp < 0)
			return FLASH_NOT_HEX;
		sum += (uint8_t) tmp;
	}

	// Address
	for (int i = 0; i < 4; i++) {
		cbuff[i] = hexLine[i + 2];
	}
	cbuff[4] = '\0';
	address = hexToInt(cbuff);
	if (address < 0) {
		err = FLASH_NOT_HEX;
		goto exit;
	}

	// type
	cbuff[0] = hexLine[6];
	cbuff[1] = hexLine[7];
	cbuff[2] = '\0';
	type = hexToInt(cbuff);
	if (type < 0) {
		err = FLASH_NOT_HEX;
		goto exit;
	}
	if (type == 1) {
		err = FLASH_FIN;
		goto exit;
	}

	// data
	if (byteCount > 0) {
		idx = 8;
		for (int i = 0; i < byteCount; i++) {
			cbuff[0] = hexLine[idx++];
			cbuff[1] = hexLine[idx++];
			cbuff[2] = '\0';
			tmp = hexToInt(cbuff);
			if (tmp < 0) {
				err = FLASH_NOT_HEX;
				goto exit;
			}
			byteLine[i] = (uint8_t) tmp;
		}
	}
	// checksum
	cbuff[0] = hexLine[idx++];
	cbuff[1] = hexLine[idx++];
	cbuff[2] = '\0';
	cs = hexToInt(cbuff);
	if (cs < 0) {
		err = FLASH_NOT_HEX;
		goto exit;
	}
	flashPar.frameType = type;
	flashPar.cs = (uint8_t) cs + sum;
	flashPar.byteCount = byteCount;
	if (type == 4) {
		flashPar.extAddr = (((uint32_t) byteLine[0]) << 8
				| (uint32_t) byteLine[1]) << 16;
	}
	if (type == 0) {
		flashPar.address = address;
	}

	err = 0;
	exit: return err;
}

int flashWriteData(void) {
	int err;
	uint32_t flashSize;
	uint32_t realAddr;
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t pageError;
	flashSize = utilFlashSize();

	HAL_StatusTypeDef rf;
	//HAL_StatusTypeDef rf=  hfHAL_FLASH_Unlock();
	err = 0;
	if (flashPar.frameType != 0) {
		err = 0;
		goto exit;
	}

	realAddr = flashPar.extAddr + flashPar.address;
	if (!utilIsFlash(realAddr + 64)) {
		err = FLASH_ADDR_RANGE;
		goto exit;
	}
	rf = HAL_FLASH_Unlock();
	if (rf != HAL_OK) {
		err = FLASH_UNLOCK_ERROR;
		goto exit;
	}

	if ((realAddr & (FLASH_PAGE_SIZE - 1)) == 0) {
		eraseInit.NbPages = 1;
		eraseInit.PageAddress = realAddr;
		eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
		HAL_FLASHEx_Erase(&eraseInit, &pageError);
		//FLASH_PageErase(realAddr);
		//hfxFLASH_PageErase(realAddr);
	}
	//rf= HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, realAddr, data32[pos]);
	//rf= HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, realAddr, 0x12343210);
	//if (rf != HAL_OK) {
	//	err= FLASH_return rf;
	//}
	//address += 4;
	//}
	if (flashPar.byteCount == 64) {
		for (int idx = 0; idx < 8; idx++) {
			//rf= hrfHAL_FLASHEx_HalfPageProgram(realAddr, data32);
			//FLASH_Program_HalfWord(realAddr + idx, data16[idx/2]);
			rf = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, realAddr,
					data64[idx]);
			if (rf != HAL_OK) {
				err = FLASH_WRITE_ERROR;
				goto exit;
			}
			realAddr += 8;
		}
	} else {
		err = FLASH_RECORD_SIZE;
		goto exit;
	}
	exit: HAL_FLASH_Lock();
	return err;

}

void flashUpdateBootsector(void) {
	int err;
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t pageError;
	HAL_StatusTypeDef rf;
	uint32_t *bootVector;
	uint32_t *appVector;
	bool equal;
	uint32_t bootSP;
	uint32_t bootReset;
	bootVector = (uint32_t*) 0x8000000;
	appVector = (uint32_t*) AppAddr;
	bootVector += 2;
	appVector += 2;
	equal = true;
	while ((uint32_t) bootVector < 0x80000c0) {
		if (*bootVector++ != *appVector++) {
			equal = false;
			break;
		}
	}
	if (equal)
		return;
	bootSP = *(uint32_t*) 0x8000000;
	bootReset = *(uint32_t*) 0x8000004;

	rf = HAL_FLASH_Unlock();
	if (rf != HAL_OK) {
		err = FLASH_UNLOCK_ERROR;
		goto exit;
	}
	eraseInit.NbPages = 1;
	eraseInit.PageAddress = 0x8000000;
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	HAL_FLASHEx_Erase(&eraseInit, &pageError);

	bootVector = (uint32_t*) 0x8000000;
	appVector = (uint32_t*) AppAddr;

	rf = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)bootVector++, (uint64_t)bootSP);
	if (rf != HAL_OK) {
		err = FLASH_WRITE_ERROR;
		goto exit;
	}

	rf = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)bootVector, (uint64_t)bootReset);
	if (rf != HAL_OK) {
		err = FLASH_WRITE_ERROR;
		goto exit;
	}

	bootVector = (uint32_t*) 0x8000000;
	appVector = (uint32_t*) AppAddr;
	bootVector += 2;
	appVector += 2;

	for (int idx = 2; idx < 0xc0 / 4; idx++) {
		rf = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)bootVector,
				(uint64_t)*appVector);
		appVector++;
		bootVector++;
		if (rf != HAL_OK) {
			err = FLASH_WRITE_ERROR;
			goto exit;
		}
	}
	exit: HAL_FLASH_Lock();
}

