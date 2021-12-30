/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "ringbuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RingBufferU8 rb_u1, rb_u2;
uint8_t rb_u1_s[128], rb_u2_s[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void  USART1_TX_Callback(void)
{
	if (RingBufferU8_available(&rb_u2)) {
		LL_USART_TransmitData8(USART1, RingBufferU8_readByte(&rb_u2));
		LL_USART_EnableIT_TXE(USART1);
	} else
		LL_USART_DisableIT_TXE(USART1);
}

void  USART2_TX_Callback(void)
{
//	if (RingBufferU8_available(&rb_u1))
//		LL_USART_TransmitData8(USART2, RingBufferU8_readByte(&rb_u1));
//	    LL_USART_EnableIT_TXE(USART2);
}

void  USART3_TX_Callback(void)
{
	if (RingBufferU8_available(&rb_u1)) {
		LL_USART_TransmitData8(USART3, RingBufferU8_readByte(&rb_u1));
		LL_USART_EnableIT_TXE(USART3);
	} else
		LL_USART_DisableIT_TXE(USART3);
}

void  USART1_RX_Callback(void)
{
	RingBufferU8_writeByte(&rb_u1, LL_USART_ReceiveData8(USART1));
	if (LL_USART_IsActiveFlag_TXE(USART3))
		USART3_TX_Callback();
}

void  USART2_RX_Callback(void)
{
	RingBufferU8_writeByte(&rb_u1, LL_USART_ReceiveData8(USART2));
	if (LL_USART_IsActiveFlag_TXE(USART3))
		USART3_TX_Callback();
}

void  USART3_RX_Callback(void)
{
	RingBufferU8_writeByte(&rb_u2, LL_USART_ReceiveData8(USART3));
	if (LL_USART_IsActiveFlag_TXE(USART1))
		USART1_TX_Callback();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  RingBufferU8_init(&rb_u1, rb_u1_s, sizeof(rb_u1_s));
  RingBufferU8_init(&rb_u2, rb_u2_s, sizeof(rb_u2_s));

  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_RXNE(USART3);

  RingBufferU8_write(&rb_u1, (const uint8_t*)"\r\n\r\nHello\r\n", 5+6);
  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();

  if (LL_RCC_IsActiveFlag_LPWRRST()) {
	  RingBufferU8_write(&rb_u1, (const uint8_t*)"LPWRRST\r\n", 9);
	  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();
  }

  if (LL_RCC_IsActiveFlag_IWDGRST()) {
	  RingBufferU8_write(&rb_u1, (const uint8_t*)"IWDGRST\r\n", 9);
	  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();
  }

  if (LL_RCC_IsActiveFlag_PINRST()) {
	  RingBufferU8_write(&rb_u1, (const uint8_t*)"PINRST\r\n", 9);
	  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();
  }

  if (LL_RCC_IsActiveFlag_PORRST()) {
	  RingBufferU8_write(&rb_u1, (const uint8_t*)"PORRST\r\n", 9);
	  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();
  }

  if (LL_RCC_IsActiveFlag_SFTRST()) {
	  RingBufferU8_write(&rb_u1, (const uint8_t*)"SFTRST\r\n", 9);
	  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();
  }

  if (LL_RCC_IsActiveFlag_WWDGRST()) {
	  RingBufferU8_write(&rb_u1, (const uint8_t*)"WWDGRST\r\n", 9);
	  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();
  }

  LL_RCC_ClearResetFlags();

  //Включаем питание модема
  LL_GPIO_SetOutputPin(MODEM_PWR_GPIO_Port, MODEM_PWR_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool modem_ready = false;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //Включаем модем
	if (!LL_GPIO_IsInputPinSet(MODEM_STATUS_GPIO_Port, MODEM_STATUS_Pin) || !LL_GPIO_IsInputPinSet(MODEM_STATUS_2_GPIO_Port, MODEM_STATUS_2_Pin)) {
	  RingBufferU8_write(&rb_u1, (const uint8_t*)"PWR_KEY\r\n", 9);
	  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();

		  LL_GPIO_ResetOutputPin(MODEM_KEY_GPIO_Port, MODEM_KEY_Pin);
  //  	  while (!LL_GPIO_IsInputPinSet(MODEM_STATUS_GPIO_Port, MODEM_STATUS_Pin) && LL_GPIO_IsInputPinSet(MODEM_STATUS_2_GPIO_Port, MODEM_STATUS_2_Pin)) {
			  LL_mDelay(1000);
  //  	  }
		  LL_GPIO_SetOutputPin(MODEM_KEY_GPIO_Port, MODEM_KEY_Pin);

		  RingBufferU8_write(&rb_u1, (const uint8_t*)"STATUS 1\r\n", 10);
		  if (LL_USART_IsActiveFlag_TXE(USART3)) USART3_TX_Callback();
		  LL_mDelay(500);

		  modem_ready = false;
	} else
		LL_mDelay(100);

	if (!modem_ready) {
		for (uint8_t i=0; i<3; i++) {
			RingBufferU8_write(&rb_u2, (const uint8_t*)"AT\r\n", 4);
			if (LL_USART_IsActiveFlag_TXE(USART1)) USART1_TX_Callback();
			LL_mDelay(500);
		}
		modem_ready = true;
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
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

