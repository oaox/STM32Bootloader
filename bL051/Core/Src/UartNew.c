/*
 * UartNew.c
 *
 *  Created on: Feb 6, 2025
 *      Author: oao
 */

#if 0
   __HAL_FLASH_SET_LATENCY(1);
   tickstart = HAL_GetTick();

   while (__HAL_FLASH_GET_LATENCY() != 1)
   {
     if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
     {
       return HAL_TIMEOUT;
     }
   }
   MODIFY_REG(RCC->CFGR, 0xf0, 0);

   __HAL_RCC_SYSCLK_CONFIG(3);

   tickstart = HAL_GetTick();
   while (__HAL_RCC_GET_SYSCLK_SOURCE() != 8)
    {
      if((HAL_GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }

   while (__HAL_RCC_GET_SYSCLK_SOURCE() != 0xc)
   {
     if((HAL_GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
     {
       return HAL_TIMEOUT;
     }
   }
   __HAL_FLASH_SET_LATENCY(1);

   /* Check that the new number of wait states is taken into account to access the Flash
   memory by polling the FLASH_ACR register */
   tickstart = HAL_GetTick();




int position= 0;
  while (((GPIO_Init->Pin) >> position) != 0)
  {
	    iocurrent = (GPIO_Init->Pin) & (1U << position);
	    if (iocurrent)
	    {
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



		    position++;

#endif
