/*
 * Uart.c
 *
 *  Created on: Nov 17, 2024
 *      Author: oao
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include "usart.h"
#include "Uart.h"

#define HUART huart2
#define DE_Pin GPIO_PIN_1
#define DE_GPIO_Port GPIOA
#define TX_READY 0x40
#define RX_READY 0x20



int uartGetOneChar(uint8_t *pData)
{
	if ( (HUART.Instance->ISR & RX_READY) != 0) {
		*pData= (uint8_t)HUART.Instance->RDR;
		return 0;
	}
	return -1;
}

void uartSetDe(bool state)
{
	if (state)
	  {
	    DE_GPIO_Port->BSRR = DE_Pin;
	  }
	  else
	  {
		  DE_GPIO_Port->BRR = DE_Pin;
	  }
}

void uartSendResponse(uint8_t *msg)
{
	uartSetDe(true);
	while (*msg != 0) {
		while ((HUART.Instance->ISR & TX_READY) == 0);
		HUART.Instance->TDR= *msg;
		msg++;
	}
	while ((HUART.Instance->ISR & TX_READY ) == 0);
	uartSetDe(false);
}
