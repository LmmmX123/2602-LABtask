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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

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

// Flash's variable
uint32_t flash_ptr = 0;
#define FLASH_ADDR 0x000000
#define SECTOR_SIZE 4096 // W25Q64

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

	// Flash busy wait function 
	uint8_t Flash_CheckBusy(void);
	// Flash write enable
	void Flash_WriteEnable(void);
	// Flash single byte write
	void Flash_WriteByte(uint32_t addr, uint8_t dat);
	// Flash single byte read
	uint8_t Flash_ReadByte(uint32_t addr);
	// Flash sector erase
	void Flash_EraseSector(uint32_t addr);
	// Function 1: Append '1'
	void Func1_Add_1(void);
	// Function 3: Read all data
	void Func3_Read_All(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

			void DELAY_1MS(void)
			{
					for(uint32_t i = 0; i < 56000; i++)
					{
							__NOP();
					}
			}

			// Flash check the status
			uint8_t Flash_CheckBusy(void)
			{
					uint8_t status = 0;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					uint8_t cmd = 0x05; //the reading status command,which means to read the status register
					HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);
					HAL_SPI_Receive(&hspi3, &status, 1, 100);//read the status to the variable"status"
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
					return (status & 0x01); // 1="busy",0="idle";status has a long line ,but the 0x01 & means only read the last byte
			}

			// mainly enable the FLASH torecord the things in the variable
			void Flash_WriteEnable(void)
			{
					uint8_t cmd = 0x06;//the command that enable the FLASH to free the protection of the reloading and erase
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
					DELAY_1MS(); // need to wait for it
			}

			// Flash actually begin to proguamming but only can write 1 byte
			void Flash_WriteByte(uint32_t addr, uint8_t dat)
			{
					Flash_WriteEnable();
					uint8_t cmd[5] = {0x02,//the command that enables the page programming
														(uint8_t)((addr >> 16) & 0xFF), // the last 24~18 bytes,
														(uint8_t)((addr >> 8) & 0xFF),  // the last 16~8 bytes,
														(uint8_t)(addr & 0xFF),         // the last 8~1 bytes,
														dat};//the actually data that need to deliver
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_SPI_Transmit(&hspi3, cmd, 5, 100);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

					// to avoid been stuck ,need to check the status curruntly
					while(Flash_CheckBusy())
					{
							DELAY_1MS(); // 1 for 1ms,so this loop will be fast,to avoid wasting time
					}
			}

			// Flash function that can read a byte from the flash
			uint8_t Flash_ReadByte(uint32_t addr)
			{
					uint8_t cmd[4] = {0x03,//the command that enables it to read data
														(uint8_t)((addr >> 16) & 0xFF),//this are same to the writing functiong
														(uint8_t)((addr >> 8) & 0xFF),
														(uint8_t)(addr & 0xFF)};
					uint8_t dat;//name one to let the receive data has a place to store
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_SPI_Transmit(&hspi3, cmd, 4, 100);
					HAL_SPI_Receive(&hspi3, &dat, 1, 100);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
					return dat;
			}

			//as the fuction name goes,it can transmit a bunch of bytes
			void Flash_WriteBytes(uint32_t addr, const uint8_t *buf, uint16_t len) 
			{
					if (len == 0) return;//no need to use this
					uint16_t bytes_written = 0;
					#define PAGE_SIZE 256 // W25Q64 has 256 pages
					while (bytes_written < len) 
					{
							//check if you cross the line
							uint32_t page_remain = PAGE_SIZE - (addr % PAGE_SIZE);
							//the culculation in()means the offset in the page,so page_remain means as its name
							uint16_t write_len = (len - bytes_written) < page_remain ? (len - bytes_written) : page_remain;
							//compares the smaller one between the "rest bytes need to be write"and"how much the page left"
							
							Flash_WriteEnable();
							uint8_t cmd[4] = {0x02, (addr>>16)&0xFF, (addr>>8)&0xFF, addr&0xFF};
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
							HAL_SPI_Transmit(&hspi3, cmd, 4, 100);
							HAL_SPI_Transmit(&hspi3, (uint8_t*)buf + bytes_written, write_len, 100);//the most important sentense that actually done the transmit
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
							
							while (Flash_CheckBusy()); //waiting for the command been done
							addr += write_len;	//move afterward the address
							bytes_written += write_len;	//update
					}
			}

			// the most effectible fnction that take charge in the FLASH writting
			void Flash_WriteString(uint32_t addr, const char *str) 
			{
					uint16_t str_len = strlen(str); // aculate the length
					// keep the addition in control
					if (addr + str_len + 1 > FLASH_ADDR + SECTOR_SIZE) 
					{
							printf("Flash: No space for string\r\n");
							return;
					}
					// call the other functions
					Flash_WriteBytes(addr, (const uint8_t*)str, str_len);
					Flash_WriteByte(addr + str_len, '\0');
			}

			//  the other most effectible fnction that take charge in the FLASH reading
			uint16_t Flash_ReadString(uint32_t addr, char *buf, uint16_t buf_len) 
			{
					if (buf_len == 0) return 0;
					uint16_t i = 0;		//record the place
					buf[0] = '\0';		//avoid the mess output
					while (i < buf_len - 1 && addr < FLASH_ADDR + SECTOR_SIZE) 
					{
							buf[i] = Flash_ReadByte(addr);
							if (buf[i] == '\0') break; // exit the loop decisively
							i++;
							addr++;
					}
					buf[i] = '\0'; // end the string
					return i; 
			}




			// the main entrance of the function 1
			void Func1_Add_1(void)
			{
					uint16_t str_len = strlen((const char*)rx_buf); 
					if (str_len == 0) 
					{
							printf("Flash: Empty string, skip\r\n");
							return;
					}
					if (flash_ptr + str_len + 1 > FLASH_ADDR + SECTOR_SIZE) 
					{
							printf("Flash: Sector full! Can't write more data\r\n");
							return;
					}
					Flash_WriteString(flash_ptr, (const char*)rx_buf);
					flash_ptr += str_len + 1;
					printf("Flash written: %s | Next addr: %lu\r\n", rx_buf, (unsigned long)flash_ptr);
			}

			// the main entrance of the function 3
			void Func3_Read_All(void)
			{
					printf("\r\n==== Flash All Data ====\r\n");
					if (flash_ptr == 0) 
					{
							printf("No data stored in Flash!\r\n");
					}
					else 
					{
							uint32_t curr_addr = FLASH_ADDR;
							uint16_t str_idx = 1;
							char read_buf[64]; 
							
							while (curr_addr < flash_ptr && curr_addr < FLASH_ADDR + SECTOR_SIZE) 
							{
									uint16_t len = Flash_ReadString(curr_addr, read_buf, sizeof(read_buf));
									if (len == 0) break;
									printf("String %d: %s (addr: %lu, len: %u)\r\n", str_idx++, read_buf, (unsigned long)curr_addr, len);
									curr_addr += len + 1; 
							}
							printf("Total strings: %d | Used bytes: %lu\r\n", str_idx - 1, (unsigned long)flash_ptr);
					}
					printf("\r\n=======================\r\n");
			}

			// fnction that take charge in the FLASH erasing
			void Flash_EraseSector(uint32_t addr)
			{
					Flash_WriteEnable();
					uint8_t cmd[4] = {0x20,
														(uint8_t)((addr >> 16) & 0xFF),
														(uint8_t)((addr >> 8) & 0xFF),
														(uint8_t)(addr & 0xFF)};

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_SPI_Transmit(&hspi3, cmd, 4, 100);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

					uint32_t timeout = 0;
					while(Flash_CheckBusy())
					{
							DELAY_1MS(); 
							timeout++;
							if(timeout > 500) 
							{
									printf("Flash Erase Timeout! Check SPI connection\r\n");
									break;
							}
					}

					flash_ptr = 0;  
					uint8_t check = Flash_ReadByte(0);
					printf("Erase check: addr0 = 0x%02X\r\n", check); 
			}
																		


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
			if(slec_flag == 2) Func3_Read_All();    
			if(slec_flag == 3) Flash_EraseSector(0); 
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
  MX_SPI3_Init();
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
    /* USER CODE END WHILE */
		if(rx_line_done == 1)
			{
				rx_line_done = 0;

				//printf("OpenMV receive:%s\r\n", rx_buf);

				// ====================== Flash ======================
				if(slec_flag == 0)
				{
					Func1_Add_1();   
				}
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
