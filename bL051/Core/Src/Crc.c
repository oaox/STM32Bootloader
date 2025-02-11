/*
 * Crc.c
 *
 *  Created on: Feb 7, 2025
 *      Author: oao
 */
#include <stdbool.h>
#include "crc.h"
#include "Crc.h"
#include "Utils.h"

static uint32_t crcCRC_Handle_8(CRC_HandleTypeDef *hcrc, uint8_t pBuffer[], uint32_t BufferLength);;


uint32_t crcCalcCrc(bool *correct)
{
	uint8_t *beg;
	uint8_t *crcPtr;
	uint8_t *end;
	uint32_t tmp;
	uint32_t crc;
	uint32_t crcFlash;
	uint32_t crci;
	uint32_t len;
	beg= (uint8_t*)0x8005000;
	crcPtr= (beg + 0xc0);
	tmp= *(uint32_t*)crcPtr;
	tmp= byteSwap(tmp);
	end= (uint8_t *)tmp;
	crcFlash= *(uint32_t*)end;
	crcFlash= byteSwap(crcFlash);
	len= (uint32_t)(end-beg);
	crc= crcHAL_CRC_Calculate(&hcrc, (uint32_t*)beg, len);
	crci = crc ^ 0xffffffff;
	*correct= false;
	if (crci == crcFlash) *correct= true;

	return crci;
}


/**
  * @brief  Initialize the CRC polynomial if different from default one.
  * @param  hcrc CRC handle
  * @param  Pol CRC generating polynomial (7, 8, 16 or 32-bit long).
  *         This parameter is written in normal representation, e.g.
  *         @arg for a polynomial of degree 7, X^7 + X^6 + X^5 + X^2 + 1 is written 0x65
  *         @arg for a polynomial of degree 16, X^16 + X^12 + X^5 + 1 is written 0x1021
  * @param  PolyLength CRC polynomial length.
  *         This parameter can be one of the following values:
  *          @arg @ref CRC_POLYLENGTH_7B  7-bit long CRC (generating polynomial of degree 7)
  *          @arg @ref CRC_POLYLENGTH_8B  8-bit long CRC (generating polynomial of degree 8)
  *          @arg @ref CRC_POLYLENGTH_16B 16-bit long CRC (generating polynomial of degree 16)
  *          @arg @ref CRC_POLYLENGTH_32B 32-bit long CRC (generating polynomial of degree 32)
  * @retval HAL status
  */
HAL_StatusTypeDef crcHAL_CRCEx_Polynomial_Set(CRC_HandleTypeDef *hcrc, uint32_t Pol, uint32_t PolyLength)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t msb = 31U; /* polynomial degree is 32 at most, so msb is initialized to max value */

  /* Check the parameters */
  assert_param(IS_CRC_POL_LENGTH(PolyLength));

  /* Ensure that the generating polynomial is odd */
  if ((Pol & (uint32_t)(0x1U)) ==  0U)
  {
    status =  HAL_ERROR;
  }
  else
  {
    /* check polynomial definition vs polynomial size:
     * polynomial length must be aligned with polynomial
     * definition. HAL_ERROR is reported if Pol degree is
     * larger than that indicated by PolyLength.
     * Look for MSB position: msb will contain the degree of
     *  the second to the largest polynomial member. E.g., for
     *  X^7 + X^6 + X^5 + X^2 + 1, msb = 6. */
    while ((msb-- > 0U) && ((Pol & ((uint32_t)(0x1U) << (msb & 0x1FU))) == 0U))
    {
    }

    switch (PolyLength)
    {

      case CRC_POLYLENGTH_7B:
        if (msb >= HAL_CRC_LENGTH_7B)
        {
          status =   HAL_ERROR;
        }
        break;
      case CRC_POLYLENGTH_8B:
        if (msb >= HAL_CRC_LENGTH_8B)
        {
          status =   HAL_ERROR;
        }
        break;
      case CRC_POLYLENGTH_16B:
        if (msb >= HAL_CRC_LENGTH_16B)
        {
          status =   HAL_ERROR;
        }
        break;

      case CRC_POLYLENGTH_32B:
        /* no polynomial definition vs. polynomial length issue possible */
        break;
      default:
        status =  HAL_ERROR;
        break;
    }
  }
  if (status == HAL_OK)
  {
    /* set generating polynomial */
    WRITE_REG(hcrc->Instance->POL, Pol);

    /* set generating polynomial size */
    MODIFY_REG(hcrc->Instance->CR, CRC_CR_POLYSIZE, PolyLength);
  }
  /* Return function status */
  return status;
}


static uint32_t crcCRC_Handle_8(CRC_HandleTypeDef *hcrc, uint8_t pBuffer[], uint32_t BufferLength);


/**
  * @brief  Enter 8-bit input data to the CRC calculator.
  *         Specific data handling to optimize processing time.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer
  * @param  BufferLength input data buffer length
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
static uint32_t crcCRC_Handle_8(CRC_HandleTypeDef *hcrc, uint8_t pBuffer[], uint32_t BufferLength)
{
  uint32_t i; /* input data buffer index */
  uint16_t data;
  __IO uint16_t *pReg;

  /* Processing time optimization: 4 bytes are entered in a row with a single word write,
   * last bytes must be carefully fed to the CRC calculator to ensure a correct type
   * handling by the peripheral */
  for (i = 0U; i < (BufferLength / 4U); i++)
  {
    hcrc->Instance->DR = ((uint32_t)pBuffer[4U * i] << 24U) | \
                         ((uint32_t)pBuffer[(4U * i) + 1U] << 16U) | \
                         ((uint32_t)pBuffer[(4U * i) + 2U] << 8U)  | \
                         (uint32_t)pBuffer[(4U * i) + 3U];
  }
  /* last bytes specific handling */ /* will always be word aligned, so the folling code is not needed*/
#if 0
  if ((BufferLength % 4U) != 0U)
  {
    if ((BufferLength % 4U) == 1U)
    {
      *(__IO uint8_t *)(__IO void *)(&hcrc->Instance->DR) = pBuffer[4U * i];         /* Derogation MisraC2012 R.11.5 */
    }
    if ((BufferLength % 4U) == 2U)
    {
      data = ((uint16_t)(pBuffer[4U * i]) << 8U) | (uint16_t)pBuffer[(4U * i) + 1U];
      pReg = (__IO uint16_t *)(__IO void *)(&hcrc->Instance->DR);                    /* Derogation MisraC2012 R.11.5 */
      *pReg = data;
    }
    if ((BufferLength % 4U) == 3U)
    {
      data = ((uint16_t)(pBuffer[4U * i]) << 8U) | (uint16_t)pBuffer[(4U * i) + 1U];
      pReg = (__IO uint16_t *)(__IO void *)(&hcrc->Instance->DR);                    /* Derogation MisraC2012 R.11.5 */
      *pReg = data;

      *(__IO uint8_t *)(__IO void *)(&hcrc->Instance->DR) = pBuffer[(4U * i) + 2U];  /* Derogation MisraC2012 R.11.5 */
    }
  }
#endif
  /* Return the CRC computed value */
  return hcrc->Instance->DR;
}


/**
  * @brief  Initialize the CRC according to the specified
  *         parameters in the CRC_InitTypeDef and create the associated handle.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef crcHAL_CRC_Init(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */

  /* Check the parameters */

  if (hcrc->State == HAL_CRC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcrc->Lock = HAL_UNLOCKED;
    /* Init the low level hardware */
    HAL_CRC_MspInit(hcrc);
  }

  hcrc->State = HAL_CRC_STATE_BUSY;

  /* check whether or not non-default generating polynomial has been
   * picked up by user */
  if (hcrc->Init.DefaultPolynomialUse == DEFAULT_POLYNOMIAL_ENABLE)
  {
    /* initialize peripheral with default generating polynomial */
    WRITE_REG(hcrc->Instance->POL, DEFAULT_CRC32_POLY);
    MODIFY_REG(hcrc->Instance->CR, CRC_CR_POLYSIZE, CRC_POLYLENGTH_32B);
  }
  else {
  }

  /* check whether or not non-default CRC initial value has been
   * picked up by user */
    WRITE_REG(hcrc->Instance->INIT, hcrc->Init.InitValue);


  /* set input data inversion mode */
  assert_param(IS_CRC_INPUTDATA_INVERSION_MODE(hcrc->Init.InputDataInversionMode));
  MODIFY_REG(hcrc->Instance->CR, CRC_CR_REV_IN, hcrc->Init.InputDataInversionMode);

  /* set output data inversion mode */
  assert_param(IS_CRC_OUTPUTDATA_INVERSION_MODE(hcrc->Init.OutputDataInversionMode));
  MODIFY_REG(hcrc->Instance->CR, CRC_CR_REV_OUT, hcrc->Init.OutputDataInversionMode);

  /* makes sure the input data format (bytes, halfwords or words stream)
   * is properly specified by user */
  assert_param(IS_CRC_INPUTDATA_FORMAT(hcrc->InputDataFormat));

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  DeInitialize the CRC peripheral.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef crcHAL_CRC_DeInit(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */
  if (hcrc == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CRC_ALL_INSTANCE(hcrc->Instance));

  /* Check the CRC peripheral state */
  if (hcrc->State == HAL_CRC_STATE_BUSY)
  {
    return HAL_BUSY;
  }

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Reset CRC calculation unit */
  __HAL_CRC_DR_RESET(hcrc);

  /* Reset IDR register content */
  __HAL_CRC_SET_IDR(hcrc, 0);

  /* DeInit the low level hardware */
  HAL_CRC_MspDeInit(hcrc);

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_RESET;

  /* Process unlocked */
  __HAL_UNLOCK(hcrc);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Compute the 7, 8, 16 or 32-bit CRC value of an 8, 16 or 32-bit data buffer
  *         starting with hcrc->Instance->INIT as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer, exact input data format is
  *         provided by hcrc->InputDataFormat.
  * @param  BufferLength input data buffer length (number of bytes if pBuffer
  *         type is * uint8_t, number of half-words if pBuffer type is * uint16_t,
  *         number of words if pBuffer type is * uint32_t).
  * @note  By default, the API expects a uint32_t pointer as input buffer parameter.
  *        Input buffer pointers with other types simply need to be cast in uint32_t
  *        and the API will internally adjust its input data processing based on the
  *        handle field hcrc->InputDataFormat.
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t crcHAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
{
  uint32_t index;      /* CRC input data buffer index */
  uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DR register) */

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Reset CRC Calculation Unit (hcrc->Instance->INIT is
  *  written in hcrc->Instance->DR) */
  __HAL_CRC_DR_RESET(hcrc);

      temp = crcCRC_Handle_8(hcrc, (uint8_t *)pBuffer, BufferLength);
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return the CRC computed value */
  return temp;
}

