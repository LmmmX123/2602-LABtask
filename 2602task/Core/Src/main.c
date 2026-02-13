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
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
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
uint8_t slec_flag = 0;	//storage of the curently change of the mode
char highli[4]={0,0,0,0};	// to provide the interaction,to let the user see what mode they chose
uint8_t rx_data;				// to store the message send from the usart
const char menu[4][32] = {	//which is the main manu of the function instruction
	"1. OpenMV 1eye distance detact",
	"2. PID control MG90 align cam",
	"3. output the data in flash",
	"4. erase flash"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f)	//redirectiion to enable printf function
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void update_highli(void)	//ui:interraction to hilight the chosen one
{
    for(int i=0; i<4; i++) highli[i] = ' ';
    highli[slec_flag] = '*';
}

void display_menu(){	//ui:display the main manu
printf("\033[2J\033[H");
printf("========== TALOS ==========\r\n");
for(int i=0; i<4; i++)
	printf("%c  %s\r\n", highli[i], menu[i]);
printf("===========================\r\n");
printf("up down by arrow key,slect by enter\r\n");
//HAL_Delay(1000);
}






/*
void function_enter(uint8_t flag){//the main entrance of the function
	if (flag == 1){
		char see_mode = flag + 49;
		HAL_UART_Transmit(&huart1,(uint8_t*)&see_mode, 1, 100);
	}
	else if(flag == 2){
	}
}*/







void key_proc(uint8_t ch)	//process the key input to the change of the variable
{
		//if(ch == 0x003);
		//finish the mode 2,because it is continuous,***this place is uncomplete***
    /*else*/if(ch == 'A' || ch == 'D'){  //when the user input the up and left arrow key
        if(slec_flag > 0){
					slec_flag--;
					update_highli();}
				else if(slec_flag == 0){	//the mode can't smaller than 1 
					//printf("input error,the mode can only be chose between 1~4");
					//HAL_Delay(1000);
					slec_flag = 0;
				}
				printf("\033[2J\033[H");
        display_menu();
				//printf("input error,the mode can only be chose between 1~4");
    }
    else if(ch == 'B' || ch == 'C'){  //when the user input the left and down arrow key
        if(slec_flag < 3){
					slec_flag++;
					update_highli();
				}
				else if(slec_flag == 3){	//the mode can't bigger than 4
					//printf("input error,the mode can only be chose between 1~4");		
					//HAL_Delay(1000);
					slec_flag = 3;
				}
				printf("\033[2J\033[H");					
        display_menu();
				//printf("input error,the mode can only be chose between 1~4");
    }
    else if(ch == '\r'){  // when the user input enter key
        printf("\r\nis doing the mode：%d\r\n", slec_flag + 1);		
				//function_enter(slec_flag+1);
				char see_mode = slec_flag + 49;
				HAL_UART_Transmit(&huart1,(uint8_t*)&see_mode, 1, 100);
        //show to the user mode they are doing	
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == LPUART1)
		//to check are the input comes from the LPUART,because thisprogram has two divice use the USART
    {
        if(rx_data != 0x1B && rx_data != '[')
				//the arrow input can divide to 3part,the 0x1B the '[' and the A/B/C/D,
				//so the only thing we need to do is detect the last input
        {
            key_proc(rx_data);
        }
			HAL_UART_Receive_IT(&hlpuart1, &rx_data, 1);//the user may not input once
		}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//uint8_t byteNumber = 1;

	


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
  MX_USART1_UART_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	// init the UI
	update_highli();
	display_menu();
	// 开启接收中断
	HAL_UART_Receive_IT(&hlpuart1, &rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
