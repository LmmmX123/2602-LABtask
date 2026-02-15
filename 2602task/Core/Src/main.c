/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include "stdio.h"
#include "string.h"
/* USER CODE BEGIN Includes */

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
uint8_t slec_flag = 0;
char highli[4]={0,0,0,0};
uint8_t rx_data;
uint8_t cut_down = 0;
uint8_t  rx_buf[64];
uint16_t rx_cnt = 0;
uint8_t  rx_line_done = 0;
uint8_t  rx_temp;

const char menu[4][32] = {
	"1. OpenMV 1eye distance detact",
	"2. PID control MG90 align cam",
	"3. output to flash",
	"4. erase flash"
};

char uart1_rx_buf[128] = {0};
uint16_t uart1_rx_idx = 0;
uint8_t uart1_rx_complete = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void update_highli(void)
{
    for(int i=0; i<4; i++) highli[i] = ' ';
    highli[slec_flag] = '*';
}

void display_menu(void)
{
	printf("\033[2J\033[H");
	printf("========== TALOS ==========\r\n");
	for(int i=0; i<4; i++)
		printf("%c  %s\r\n", highli[i], menu[i]);
	printf("===========================\r\n");
	printf("up down by arrow key, select by enter\r\n");
}

void key_proc(uint8_t ch)
{
    if(ch == 'A' || ch == 'D')
    {
        if(slec_flag > 0)
        {
					slec_flag--;
					update_highli();
        }
        else
        {
					printf("input error,the mode can only be chose between 1~4");
					slec_flag = 0;
        }
				printf("\033[2J\033[H");
        display_menu();
    }
    else if(ch == 'B' || ch == 'C')
    {
        if(slec_flag < 3)
        {
					slec_flag++;
					update_highli();
        }
        else
        {
					slec_flag = 3;
        }
				printf("\033[2J\033[H");
        display_menu();
    }
    else if(ch == '\r')
    {
			printf("\r\nis doing the mode:%d\r\n", slec_flag + 1);
			char see_mode = slec_flag + 48;
			HAL_UART_Transmit(&huart1,(uint8_t*)&see_mode, 1, 100);
    }
		else if (ch == 3)
		{
				slec_flag = 0;
				update_highli();
				display_menu();
				printf("\r\ncut down the chosen mode\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t*)&cut_down, 1, 100);
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == LPUART1)
    {
				uint8_t ch = rx_data;
				HAL_UART_Receive_IT(&hlpuart1, &rx_data, 1);
        if(ch != 0x1B && ch != '[')
        {
            key_proc(ch);
        }
    }
    else if(huart->Instance == USART1)
    {
				HAL_UART_Receive_IT(&huart1, &rx_temp, 1);

				if(rx_temp == 0x00 || rx_temp == 0xFF)
				{
						return;
				}

				if(rx_temp == '\n' || rx_temp == '\r')
				{
						if(rx_cnt > 0)
						{
								rx_buf[rx_cnt] = '\0';
								rx_line_done = 1;
						}
						rx_cnt = 0;
				}
				else
				{
						if(rx_cnt < 63)
						{
								rx_buf[rx_cnt++] = rx_temp;
						}
						else
						{
								rx_cnt = 0;
						}
				}
    }
}
/* USER CODE END 4 */

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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
	update_highli();
	display_menu();
	HAL_UART_Receive_IT(&hlpuart1, &rx_data, 1);
	HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(rx_line_done == 1)
		{
			printf("receive from OpenMV:%s\r\n", rx_buf);
			rx_line_done = 0;
		}
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM15 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM15)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
