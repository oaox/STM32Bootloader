/*
 * Gpio.c
 *
 *  Created on: Feb 5, 2025
 *      Author: oao
 */
#include <stdint.h>
#include "stm32l0xx_hal.h"
#include <stm32l0xx_hal_gpio.h>
#include "Gpio.h"


/**
  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
  *                   All port bits are not necessarily available on all GPIOs.
  * @retval None
  */
void gpioHAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
  uint32_t position = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t tmp = 0x00U;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, GPIO_Pin));

  /* Configure the port pins */
  while ((GPIO_Pin >> position) != 0)
  {
    /* Get the IO position */
    iocurrent = (GPIO_Pin) & (1U << position);

    if (iocurrent)
    {
      /*------------------------- EXTI Mode Configuration --------------------*/
      /* Clear the External Interrupt or Event for the current IO */

      tmp = SYSCFG->EXTICR[position >> 2U];
      tmp &= ((0x0FUL) << (4U * (position & 0x03U)));
      if (tmp == (GPIO_GET_INDEX(GPIOx) << (4U * (position & 0x03U))))
      {
        /* Clear EXTI line configuration */
        EXTI->IMR &= ~((uint32_t)iocurrent);
        EXTI->EMR &= ~((uint32_t)iocurrent);

        /* Clear Rising Falling edge configuration */
        EXTI->FTSR &= ~((uint32_t)iocurrent);
        EXTI->RTSR &= ~((uint32_t)iocurrent);

        tmp = (0x0FUL) << (4U * (position & 0x03U));
        SYSCFG->EXTICR[position >> 2U] &= ~tmp;
      }

      /*------------------------- GPIO Mode Configuration --------------------*/
      /* Configure IO Direction in Analog Mode (reset state) */
      GPIOx->MODER |= (GPIO_MODE_ANALOG << (position * 2U));

      /* Configure the default Alternate Function in current IO */
      GPIOx->AFR[position >> 3U] &= ~(0xFUL << ((uint32_t)(position & 0x07UL) * 4U));

      /* Deactivate the Pull-up oand Pull-down resistor for the current IO */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (position * 2U));

      /* Configure the default value IO Output Type */
      GPIOx->OTYPER  &= ~(GPIO_OTYPER_OT_0 << position);

      /* Configure the default value for IO Speed */
      GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEED0 << (position * 2U));
    }
    position++;
  }
}

#if 1
void gpioHAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    GPIOx->OSPEEDR = 0xc000000;
    GPIOx->OTYPER= 0;
    GPIOx->PUPDR= 0x26000000;
    GPIOx->AFR[0]= 0x4400;
    GPIOx->AFR[1]= 0;
    GPIOx->MODER=0xe8dffca7;
    SYSCFG->EXTICR[0]= 0;
    SYSCFG->EXTICR[1]= 0;
    SYSCFG->EXTICR[2]= 0;
    SYSCFG->EXTICR[3]= 0;
    EXTI->RTSR=0;
    EXTI->FTSR= 0;
    EXTI->EMR= 0;
    EXTI->IMR= 0x3f840000;
}
#else
/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  GPIO_Init pointer to a GPIO_InitTypeDef structure that contains
  *                    the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void gpioHAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;

  /* Check the parameters */
  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));
  assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, (GPIO_Init->Pin)));

  /* Configure the port pins */
  while (((GPIO_Init->Pin) >> position) != 0)
  {
    /* Get the IO position */
    iocurrent = (GPIO_Init->Pin) & (1U << position);

    if (iocurrent)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Output or Alternate function mode selection */
      if (((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT) ||
          ((GPIO_Init->Mode & GPIO_MODE) == MODE_AF))
      {
        /* Check the Speed parameter */
        assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEED0 << (position * 2U));
        temp |= (GPIO_Init->Speed << (position * 2U));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
        GPIOx->OTYPER = temp;
      }

      if ((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG)
      {
        /* Check the Pull parameter */
        assert_param(IS_GPIO_PULL(GPIO_Init->Pull));

        /* Activate the Pull-up or Pull down resistor for the current IO */
        temp = GPIOx->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2U));
        temp |= ((GPIO_Init->Pull) << (position * 2U));
        GPIOx->PUPDR = temp;
      }

      /* In case of Alternate function mode selection */
      if ((GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
      {
        /* Check the Alternate function parameters */
        assert_param(IS_GPIO_AF_INSTANCE(GPIOx));
        assert_param(IS_GPIO_AF(GPIO_Init->Alternate));

        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3U];
        temp &= ~(0xFUL << ((uint32_t)(position & 0x07UL) * 4U));
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07U) * 4U));
        GPIOx->AFR[position >> 3U] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODE0 << (position * 2U));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
      GPIOx->MODER = temp;

      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if ((GPIO_Init->Mode & EXTI_MODE) != 0x00U)
      {
        /* Enable SYSCFG Clock */
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EXTICR[position >> 2U];
        CLEAR_BIT(temp, (0x0FUL) << (4U * (position & 0x03U)));
        SET_BIT(temp, (GPIO_GET_INDEX(GPIOx)) << (4 * (position & 0x03U)));
        SYSCFG->EXTICR[position >> 2U] = temp;

        /* Clear Rising Falling edge configuration */
        temp = EXTI->RTSR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & TRIGGER_RISING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->RTSR = temp;

        temp = EXTI->FTSR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & TRIGGER_FALLING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->FTSR = temp;

        temp = EXTI->EMR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & EXTI_EVT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->EMR = temp;

        /* Clear EXTI line configuration */
        temp = EXTI->IMR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & EXTI_IT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->IMR = temp;
      }
    }
    position++;
  }
}
#endif

/**
  * @brief  Sets or clears the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0xx family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
  *                   All port bits are not necessarily available on all GPIOs.
  * @param  PinState specifies the value to be written to the selected bit.
  *                   This parameter can be one of the GPIO_PinState enum values:
  *                        GPIO_PIN_RESET: to clear the port pin
  *                        GPIO_PIN_SET: to set the port pin
  * @retval None
  */
void rccHAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, GPIO_Pin));
  assert_param(IS_GPIO_PIN_ACTION(PinState));

  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin ;
  }
}

