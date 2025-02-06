/*
 * Uart.c
 *
 *  Created on: Nov 17, 2024
 *      Author: oao
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"
//#include "stm32l0xx_ll_lpuart.h"
#include "usart.h"
#include "Uart.h"
#include "Rcc.h"

#define HUART huart2
#define DE_Pin GPIO_PIN_1
#define DE_GPIO_Port GPIOA
#define TX_READY 0x40
#define RX_READY 0x20

static void uartUART_EndRxTransfer(UART_HandleTypeDef *huart);


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

void uartSendResponse(char *msg)
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


/**
  * @brief Initialize the UART mode according to the specified
  *        parameters in the UART_InitTypeDef and initialize the associated handle.
  * @param huart UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef uartHAL_UART_Init(UART_HandleTypeDef *huart)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  if (huart->Init.HwFlowCtl != UART_HWCONTROL_NONE)
  {
    /* Check the parameters */
    assert_param(IS_UART_HWFLOW_INSTANCE(huart->Instance));
  }
  else
  {
    /* Check the parameters */
    assert_param((IS_UART_INSTANCE(huart->Instance)) || (IS_LPUART_INSTANCE(huart->Instance)));
  }

  if (huart->gState == HAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = HAL_UNLOCKED;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    UART_InitCallbacksToDefault(huart);

    if (huart->MspInitCallback == NULL)
    {
      huart->MspInitCallback = HAL_UART_MspInit;
    }

    /* Init the low level hardware */
    huart->MspInitCallback(huart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    HAL_UART_MspInit(huart);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
  }

  huart->gState = HAL_UART_STATE_BUSY;

  __HAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  if (uartUART_SetConfig(huart) == HAL_ERROR)
  {
    return HAL_ERROR;
  }

  if (huart->AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
  {
    uartUART_AdvFeatureConfig(huart);
  }

  /* In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

  __HAL_UART_ENABLE(huart);

  /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
  return (uartUART_CheckIdleState(huart));
}


/**
  * @brief Configure the UART peripheral advanced features.
  * @param huart UART handle.
  * @retval None
  */
void uartUART_AdvFeatureConfig(UART_HandleTypeDef *huart)
{
#if 0
	/* Check whether the set of advanced features to configure is properly set */
  assert_param(IS_UART_ADVFEATURE_INIT(huart->AdvancedInit.AdvFeatureInit));

  /* if required, configure TX pin active level inversion */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_TXINVERT_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_TXINV(huart->AdvancedInit.TxPinLevelInvert));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_TXINV, huart->AdvancedInit.TxPinLevelInvert);
  }

  /* if required, configure RX pin active level inversion */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXINVERT_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_RXINV(huart->AdvancedInit.RxPinLevelInvert));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_RXINV, huart->AdvancedInit.RxPinLevelInvert);
  }

  /* if required, configure data inversion */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DATAINVERT_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_DATAINV(huart->AdvancedInit.DataInvert));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_DATAINV, huart->AdvancedInit.DataInvert);
  }

  /* if required, configure RX/TX pins swap */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_SWAP_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_SWAP(huart->AdvancedInit.Swap));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_SWAP, huart->AdvancedInit.Swap);
  }

  /* if required, configure RX overrun detection disabling */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXOVERRUNDISABLE_INIT))
  {
    assert_param(IS_UART_OVERRUN(huart->AdvancedInit.OverrunDisable));
    MODIFY_REG(huart->Instance->CR3, USART_CR3_OVRDIS, huart->AdvancedInit.OverrunDisable);
  }

  /* if required, configure DMA disabling on reception error */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DMADISABLEONERROR_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_DMAONRXERROR(huart->AdvancedInit.DMADisableonRxError));
    MODIFY_REG(huart->Instance->CR3, USART_CR3_DDRE, huart->AdvancedInit.DMADisableonRxError);
  }

  /* if required, configure auto Baud rate detection scheme */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_AUTOBAUDRATE_INIT))
  {
    assert_param(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(huart->Instance));
    assert_param(IS_UART_ADVFEATURE_AUTOBAUDRATE(huart->AdvancedInit.AutoBaudRateEnable));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_ABREN, huart->AdvancedInit.AutoBaudRateEnable);
    /* set auto Baudrate detection parameters if detection is enabled */
    if (huart->AdvancedInit.AutoBaudRateEnable == UART_ADVFEATURE_AUTOBAUDRATE_ENABLE)
    {
      assert_param(IS_UART_ADVFEATURE_AUTOBAUDRATEMODE(huart->AdvancedInit.AutoBaudRateMode));
      MODIFY_REG(huart->Instance->CR2, USART_CR2_ABRMODE, huart->AdvancedInit.AutoBaudRateMode);
    }
  }

  /* if required, configure MSB first on communication line */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_MSBFIRST_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_MSBFIRST(huart->AdvancedInit.MSBFirst));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_MSBFIRST, huart->AdvancedInit.MSBFirst);
  }
#endif
}


/**
  * @brief Check the UART Idle State.
  * @param huart UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef uartUART_CheckIdleState(UART_HandleTypeDef *huart)
{
  uint32_t tickstart;

  /* Initialize the UART ErrorCode */
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  /* Check if the Transmitter is enabled */
  if ((huart->Instance->CR1 & USART_CR1_TE) == USART_CR1_TE)
  {
    /* Wait until TEACK flag is set */
    if (uartUART_WaitOnFlagUntilTimeout(huart, USART_ISR_TEACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != HAL_OK)
    {
      /* Disable TXE interrupt for the interrupt process */
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE));

      huart->gState = HAL_UART_STATE_READY;

      __HAL_UNLOCK(huart);

      /* Timeout occurred */
      return HAL_TIMEOUT;
    }
  }

  /* Check if the Receiver is enabled */
  if ((huart->Instance->CR1 & USART_CR1_RE) == USART_CR1_RE)
  {
    /* Wait until REACK flag is set */
    if (uartUART_WaitOnFlagUntilTimeout(huart, USART_ISR_REACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != HAL_OK)
    {
      /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error)
      interrupts for the interrupt process */
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
      ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      huart->RxState = HAL_UART_STATE_READY;

      __HAL_UNLOCK(huart);

      /* Timeout occurred */
      return HAL_TIMEOUT;
    }
  }

  /* Initialize the UART State */
  huart->gState = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
  huart->RxEventType = HAL_UART_RXEVENT_TC;

  __HAL_UNLOCK(huart);

  return HAL_OK;
}

#if 1
HAL_StatusTypeDef uartUART_SetConfig(UART_HandleTypeDef *huart)
{
	 MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, 0xc);
	 MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, 0);
	 MODIFY_REG(huart->Instance->CR3, USART_CR3_FIELDS, 0);
	 huart->Instance->BRR = (uint16_t)0xd0;
	return HAL_OK;

}
#else
/**
  * @brief Configure the UART peripheral.
  * @param huart UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef uartUART_SetConfig(UART_HandleTypeDef *huart)
{
	  HAL_StatusTypeDef ret               = HAL_OK;

	uint32_t tmpreg;
  uint16_t brrtemp;
  UART_ClockSourceTypeDef clocksource;
  uint32_t usartdiv;
  uint32_t pclk;

#if 0
  /* Check the parameters */
  assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
  assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  if (UART_INSTANCE_LOWPOWER(huart))
  {
    assert_param(IS_LPUART_STOPBITS(huart->Init.StopBits));
  }
  else
  {
    assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
    assert_param(IS_UART_ONE_BIT_SAMPLE(huart->Init.OneBitSampling));
  }

  assert_param(IS_UART_PARITY(huart->Init.Parity));
  assert_param(IS_UART_MODE(huart->Init.Mode));
  assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
  assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));
#endif
  /*-------------------------- USART CR1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
  *  the UART Word Length, Parity, Mode and oversampling:
  *  set the M bits according to huart->Init.WordLength value
  *  set PCE and PS bits according to huart->Init.Parity value
  *  set TE and RE bits according to huart->Init.Mode value
  *  set OVER8 bit according to huart->Init.OverSampling value */
  tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling ;
  MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, tmpreg);

  /*-------------------------- USART CR2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according
  * to huart->Init.StopBits value */
  MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

  /*-------------------------- USART CR3 Configuration -----------------------*/
  /* Configure
  * - UART HardWare Flow Control: set CTSE and RTSE bits according
  *   to huart->Init.HwFlowCtl value
  * - one-bit sampling method versus three samples' majority rule according
  *   to huart->Init.OneBitSampling (not applicable to LPUART) */
  tmpreg = (uint32_t)huart->Init.HwFlowCtl;

  if (!(UART_INSTANCE_LOWPOWER(huart)))
  {
    tmpreg |= huart->Init.OneBitSampling;
  }
  MODIFY_REG(huart->Instance->CR3, USART_CR3_FIELDS, tmpreg);


  /*-------------------------- USART BRR Configuration -----------------------*/
  UART_GETCLOCKSOURCE(huart, clocksource);

  /* Check LPUART instance */
  if (UART_INSTANCE_LOWPOWER(huart))
  {
    /* Retrieve frequency clock */
    switch (clocksource)
    {
      case UART_CLOCKSOURCE_PCLK1:
        pclk = rccHAL_RCC_GetPCLK1Freq();
        break;
      case UART_CLOCKSOURCE_HSI:
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
        {
          pclk = (uint32_t)(HSI_VALUE >> 2U);
        }
        else
        {
          pclk = (uint32_t) HSI_VALUE;
        }
        break;
      case UART_CLOCKSOURCE_SYSCLK:
        pclk = rccHAL_RCC_GetSysClockFreq();
        break;
      case UART_CLOCKSOURCE_LSE:
        pclk = (uint32_t) LSE_VALUE;
        break;
      default:
        pclk = 0U;
        ret = HAL_ERROR;
        break;
    }

    /* If proper clock source reported */
    if (pclk != 0U)
    {
      /* No Prescaler applicable */
      /* Ensure that Frequency clock is in the range [3 * baudrate, 4096 * baudrate] */
      if ((pclk < (3U * huart->Init.BaudRate)) ||
          (pclk > (4096U * huart->Init.BaudRate)))
      {
        ret = HAL_ERROR;
      }
      else
      {
        usartdiv = (uint32_t)(UART_DIV_LPUART(pclk, huart->Init.BaudRate));
        if ((usartdiv >= LPUART_BRR_MIN) && (usartdiv <= LPUART_BRR_MAX))
        {
          huart->Instance->BRR = usartdiv;
        }
        else
        {
          ret = HAL_ERROR;
        }
      } /* if ( (pclk < (3 * huart->Init.BaudRate) ) || (pclk > (4096 * huart->Init.BaudRate) )) */
    } /* if (pclk != 0) */
  }
  /* Check UART Over Sampling to set Baud Rate Register */
  else if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
  {
    switch (clocksource)
    {
      case UART_CLOCKSOURCE_PCLK1:
        pclk = rccHAL_RCC_GetPCLK1Freq();
        break;
      case UART_CLOCKSOURCE_PCLK2:
        pclk = rccHAL_RCC_GetPCLK2Freq();
        break;
      case UART_CLOCKSOURCE_HSI:
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
        {
          pclk = (uint32_t)(HSI_VALUE >> 2U);
        }
        else
        {
          pclk = (uint32_t) HSI_VALUE;
        }
        break;
      case UART_CLOCKSOURCE_SYSCLK:
        pclk = rccHAL_RCC_GetSysClockFreq();
        break;
      case UART_CLOCKSOURCE_LSE:
        pclk = (uint32_t) LSE_VALUE;
        break;
      default:
        pclk = 0U;
        ret = HAL_ERROR;
        break;
    }

    /* USARTDIV must be greater than or equal to 0d16 */
    if (pclk != 0U)
    {
      usartdiv = (uint32_t)(UART_DIV_SAMPLING8(pclk, huart->Init.BaudRate));
      if ((usartdiv >= UART_BRR_MIN) && (usartdiv <= UART_BRR_MAX))
      {
        brrtemp = (uint16_t)(usartdiv & 0xFFF0U);
        brrtemp |= (uint16_t)((usartdiv & (uint16_t)0x000FU) >> 1U);
        huart->Instance->BRR = brrtemp;
      }
      else
      {
        ret = HAL_ERROR;
      }
    }
  }
  else
  {
    switch (clocksource)
    {
      case UART_CLOCKSOURCE_PCLK1:
        pclk = rccHAL_RCC_GetPCLK1Freq();
        break;
      case UART_CLOCKSOURCE_PCLK2:
        pclk = rccHAL_RCC_GetPCLK2Freq();
        break;
      case UART_CLOCKSOURCE_HSI:
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
        {
          pclk = (uint32_t)(HSI_VALUE >> 2U);
        }
        else
        {
          pclk = (uint32_t) HSI_VALUE;
        }
        break;
      case UART_CLOCKSOURCE_SYSCLK:
        pclk = rccHAL_RCC_GetSysClockFreq();
        break;
      case UART_CLOCKSOURCE_LSE:
        pclk = (uint32_t) LSE_VALUE;
        break;
      default:
        pclk = 0U;
        ret = HAL_ERROR;
        break;
    }

    if (pclk != 0U)
    {
      /* USARTDIV must be greater than or equal to 0d16 */
      usartdiv = (uint32_t)(UART_DIV_SAMPLING16(pclk, huart->Init.BaudRate));
      if ((usartdiv >= UART_BRR_MIN) && (usartdiv <= UART_BRR_MAX))
      {
        huart->Instance->BRR = (uint16_t)usartdiv;
      }
      else
      {
        ret = HAL_ERROR;
      }
    }
  }
return ret;
}
#endif


/**
  * @brief  This function handles UART Communication Timeout. It waits
  *                  until a flag is no longer in the specified status.
  * @param huart     UART handle.
  * @param Flag      Specifies the UART flag to check
  * @param Status    The actual Flag status (SET or RESET)
  * @param Tickstart Tick start value
  * @param Timeout   Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef uartUART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                              uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {

        return HAL_TIMEOUT;
      }

      if (READ_BIT(huart->Instance->CR1, USART_CR1_RE) != 0U)
      {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) == SET)
        {
           /* Clear Overrun Error flag*/
           __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);

           /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts if ongoing */
           uartUART_EndRxTransfer(huart);

           huart->ErrorCode = HAL_UART_ERROR_ORE;

           /* Process Unlocked */
           __HAL_UNLOCK(huart);

           return HAL_ERROR;
        }
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RTOF) == SET)
        {
          /* Clear Receiver Timeout flag*/
          __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_RTOF);

          /* Blocking error : transfer is aborted
          Set the UART state ready to be able to start again the process,
          Disable Rx Interrupts if ongoing */
          uartUART_EndRxTransfer(huart);

          huart->ErrorCode = HAL_UART_ERROR_RTO;

          /* Process Unlocked */
          __HAL_UNLOCK(huart);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

static void uartUART_EndRxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
  if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
  }

  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

  /* Reset RxIsr function pointer */
  huart->RxISR = NULL;
}


