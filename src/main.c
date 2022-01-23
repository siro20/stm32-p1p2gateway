/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c
  * @author  MCD Application Team
  * @brief   This example shows how to retarget the C library printf function
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup STM32L0xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Private functions ---------------------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);

static volatile int input;
static volatile int tx;
static volatile int tx_valid;
static volatile int state = 0;
static volatile int cnt = 0;


#define STATE_WAIT_FOR_START 0
#define STATE_RX 1

// 0 START
// 1 BIT0
// 2 BIT1
// 3 BIT2
// 4 BIT3
// 5 BIT4
// 6 BIT5
// 7 BIT6
// 8 BIT7
// 9 PARITY
// 10 STOP
#define START_BIT (1 << 0)
#define PARITY_BIT (1 << 9)
#define STOP_BIT (1 << 10)
#define BIT_CNT 11

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  int delay;

  /* STM32L0xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 32 MHz */
  SystemClock_Config();

  /* Initialize BSP Led for LED3 */
  BSP_LED_Init(LED3);

  /*##- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) : 
	                  BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  puts("P1/P2 decoder\r");

  puts("init gpio...\r");

  /* Enable GPIO channels Clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure  (TIMx_Channel) in Alternate function, push-pull and high speed */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  NVIC_SetPriority(EXTI0_1_IRQn, 2);

  puts("init timer...\r");

  /*##- Configure the TIM peripheral #######################################*/

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;
  TimHandle.Init.Period            = (32000000U / 9600) - 1;
  TimHandle.Init.Prescaler         = 0;
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  puts("waiting for idle...\r");
  // Wait about 1 msec (11bits @ 9600 baud) for idle line
  cnt = HAL_GetTick();
  while (cnt + 1 > HAL_GetTick()) {
    if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) {
      cnt = HAL_GetTick();
    }
  }

  state = STATE_WAIT_FOR_START;
  cnt = 0;
  input = 0;
  tx_valid = 0;

  delay = 0;

  int timestamp = 1;

  /* Enable EXTI Interrupt to the highest priority */
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  puts("Starting main loop\r");

  /* Infinite loop */
  while (1)
  {
	if (tx_valid) {
		int parity = 0;
		int tmp = tx;
		tx_valid = 0;
		__DSB();

		if (timestamp) {
			printf("%lu: ", HAL_GetTick());
			timestamp = 0;
		}
		for (int i = 1; i < 10; i++) {
			parity += (tmp >> i) & 1;
		}
		if (parity & 1)
			putchar('P');
		if (tmp & START_BIT)
			putchar('>');
		if (!(tmp & STOP_BIT))
			putchar('<');
		tmp = (tmp >> 1) & 0xff;
		printf("%02x ", tmp);
	} else if (state == STATE_WAIT_FOR_START && cnt == BIT_CNT) {
		printf("\r\n");
		timestamp = 1;
	}
	delay ++;
	if (delay < 10000)
		continue;
	delay = 0;
	BSP_LED_Toggle(LED3);
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the BSP_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);

  return ch;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	int val = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

	if (state == STATE_RX) {
		if (val) {
			input |= (1 << cnt);
		} else {
			input &= ~(1 << cnt);
		}

		cnt++;
		if (cnt == BIT_CNT) {
			tx = input;
			tx_valid = 1;
			__DSB();

			input = 0;
			cnt = 0;
			state = STATE_WAIT_FOR_START;
		}
	} else if (state == STATE_WAIT_FOR_START) {
		if (val)
			cnt ++;
	}
}

/**
  * @brief EXTI line detection callback.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// start sampling in 1/4 baud seconds
	// Every low bit adjustes the timer for better accuracy
	__HAL_TIM_SET_COUNTER(&TimHandle, (32000000U / (9600 * 4 / 3)) - 1);

	if (state == STATE_WAIT_FOR_START) {
		cnt = 0;
		state = STATE_RX;
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 1
  *            Main regulator output voltage  = Scale1 mode
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV      = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("assert(%s:%lu)\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
