/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
//#include <stdio.h>
#include <stdbool.h>
//#include <string.h>
#include "Uart.h"
#include "Flash.h"
#include "Rcc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint32_t v[0x10];
} uninitBuff_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uninitBuff_t *uninit;
uint8_t hexLine[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

__attribute__((naked))
extern void start_application(unsigned long app_link_location)
{
	asm(" ldr r0, [r0,#4]"); // get the new program counter value from the program's reset vector

	asm(" blx r0");          // jump to the start address

}

uint32_t byteSwap(uint32_t s)
{
	uint32_t d;
	uint8_t *src;
	uint8_t *dest;
	src= (uint8_t*)&s;
	dest= (uint8_t *)&d +3;
	for (int i=0; i<4; i++) *dest--= *src++;
	return d;
}

void uintToHex(uint32_t v, char *buff, uint8_t term)
{
	uint8_t ch;
	int shift;
	for(int i=0; i<8; i++) {
		shift= 4*(7-i);
		ch= (uint8_t)(v>>shift) & 0x0f;
		ch += '0';
		if (ch > '9') ch= ch + ('A'-'9'-1);
		buff[i]= ch;
	}
	buff[8]= term;
}



uint32_t calcCrc(void) {
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
	crc= HAL_CRC_Calculate(&hcrc, (uint32_t*)beg, len);
	crci = crc ^ 0xffffffff;


	return crci;
}

		  // 	 uint32_t crcinv= crcres ^ 0xffffffff;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// bootloader
	uint32_t resetFlags;
	  char rbuff[15];
	  char cr[3];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  //*(uint32_t*)0xE000E010= *(uint32_t*)0xE000E010 & ~2;
  uninit= (uninitBuff_t *) 0x20000000;
  for (int i=0; i<4; i++){ uninit->v[i]= 0;}

  cr[0]= 0x0a;
  cr[1]= 0x0d;
  cr[2]= 0;

  resetFlags= *(uint32_t*)0x40021050;
  uninit->v[0]= resetFlags;
  //*(uint32_t*)0x40021050= RMVF;
  RCC->CSR |= RCC_CSR_RMVF;
  uintToHex(resetFlags, rbuff, 0x0a);
  //strcat(rbuff, cr);
   uartSendResponse(rbuff);
   resetFlags= *(uint32_t*)0x40021050;


  //for (int i= 0; i< 16; i++) *cp++= crctestdata_bin[i];
  //*cp= (uint8_t*)&crctestdata[1];
  //for (int i= 0; i< 8; i++) *cp++= crctestdata_bin[15-i];

  //uint32_t crcres= HAL_CRC_Calculate(&hcrc, crctestdata, 16);
  //HAL_IWDG_Refresh(&hiwdg);
  bool attf;
  bool app_maybe_present;
  app_maybe_present= *(uint32_t*)AppAddr? true:false;
  uint32_t reg= GPIOA->IDR;

  if (app_maybe_present) {
 // 	 uint32_t crcres= HAL_CRC_Calculate(&hcrc, AppAddr,10384);
 // 	 uint32_t crcinv= crcres ^ 0xffffffff;
   }

  if ((GPIOA->IDR & ATTF_Pin) != (uint32_t)GPIO_PIN_RESET)
	attf= true;
	else
	attf= false;


  /*
  static void (*go_to_app)(void) = 0;
   go_to_app = (void (*)(void))(PROGRAM_VECTOR_TABLE[1]);
   SCB->VTOR = (uint32_t)PROGRAM_VECTOR_TABLE;
   __set_MSP(PROGRAM_VECTOR_TABLE[0]);
   __set_PSP(PROGRAM_VECTOR_TABLE[0]);
   go_to_app();
*/

if (!attf && app_maybe_present) {
	 // static void (*func)(void);
		HAL_UART_MspDeInit(&huart2);
		//*(uint32_t*)0xE000E010= *(uint32_t*)0xE000E010 & ~2;// disable RTC interrupts
	//*vectorTable= (uint32_t )AppAddr; // See Mitchell Jones, https://stackoverflow.com/a/58306399
		//vtor=SCB->VTOR;
	//SCB->VTOR= (uint32_t) AppAddr;//(uint32_t) vectorTable;
	//vtor=SCB->VTOR;
	   //__set_MSP(VectorTable[0]);
	   //__set_PSP(VectorTable[0]);

//	void (*application)(void);
	__disable_irq();
	start_application(AppAddr);
//	application = *(uint32_t*)(AppAddr+4);


//		((func= (void(*)(void)) ((uint32_t*)AppAddr)+1 ; //vectorTable[1];
//		__disable_irq();
	  //func();

}
  //*(uint32_t*)0xE000ED08= 0x8004000;

  //uint32_t *p= (uint32_t *)0x8004004;

  //typedef void func(void);
  //func* f = (func*)(*p & 0xffffffff);
  //f();
  //uint32_t t1= HAL_GetTick();
  //*(uint32_t*)0xE000E010= *(uint32_t*)0xE000E010 & ~2;
  //for(int i=0; i<10000; i++){
//	  while((*(uint32_t*)0xE000E010 & 0x10000) == 0);
 // }
 // uint32_t t2= HAL_GetTick();
 // uint32_t t3= t2-t1;

  //*(uint32_t*)0xE000E010= *(uint32_t*)0xE000E010 | 2;
 // t1= HAL_GetTick();
 // for(int i=0; i<10000; i++){
//	  while((*(uint32_t*)0xE000E010 & 0x10000) == 0);
 // }
 // t2= HAL_GetTick();
 // t3= t2-t1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int r;
	  r= flashReadOneLine();
	  r= flashDecodeLine();
	  if (r == FLASH_FIN) {
		  r= calcCrc();
		    }
	  else {
		  r= flashWriteData();
	  }
	   //r= flashOneLine();
	  uintToHex((uint32_t)r, rbuff, 0x0a);
	  //sprintf(rbuff, "%x", r);
	  //strcat(rbuff, cr);
	   uartSendResponse(rbuff);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (rccHAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (rccHAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (rccHAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
