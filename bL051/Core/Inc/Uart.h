/*
 * Uart.h
 *
 *  Created on: Nov 17, 2024
 *      Author: oao
 */

#ifndef INC_UART_H_
#define INC_UART_H_

int uartGetOneChar(uint8_t *pData);
void uartSendResponse(uint8_t *msg);



#endif /* INC_UART_H_ */
