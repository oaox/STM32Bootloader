/*
 * Uart.h
 *
 *  Created on: Nov 17, 2024
 *      Author: oao
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#define USART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | \
                                      USART_CR1_OVER8)) /*!< UART or USART CR1 fields of parameters set by UART_SetConfig API */

#define USART_CR3_FIELDS  ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE |\
                                      USART_CR3_ONEBIT)) /*!< UART or USART CR3 fields of parameters set by UART_SetConfig API */

#define LPUART_BRR_MIN  0x00000300U  /* LPUART BRR minimum authorized value */
#define LPUART_BRR_MAX  0x000FFFFFU  /* LPUART BRR maximum authorized value */

#define UART_BRR_MIN    0x10U        /* UART BRR minimum authorized value */
#define UART_BRR_MAX    0x0000FFFFU  /* UART BRR maximum authorized value */



int uartGetOneChar(uint8_t *pData);
void uartSendResponse(char *msg);
HAL_StatusTypeDef uartHAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef uartUART_SetConfig(UART_HandleTypeDef *huart);
void uartUART_AdvFeatureConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef uartUART_CheckIdleState(UART_HandleTypeDef *huart);
HAL_StatusTypeDef uartUART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                              uint32_t Tickstart, uint32_t Timeout);




#endif /* INC_UART_H_ */
