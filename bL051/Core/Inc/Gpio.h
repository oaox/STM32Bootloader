/*
 * Gpio.h
 *
 *  Created on: Feb 5, 2025
 *      Author: oao
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

void gpioHAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
void gpioHAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void rccHAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);


#endif /* INC_GPIO_H_ */
