/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <MPU_9255.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

asm(".global _printf_float"); //for the float supporting of printf

#ifdef __GNUC__
int __io_putchars(char *ptr, int len){
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len,0xFFFF);
  return len;
}
#else
int fputc(int ch, FILE *f){
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,0xFFFF);
  return ch;
}
#endif /* __GNUC__ */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//覆寫MPU9255的延時函數
void MPU_9255_delay(uint32_t ms){
  HAL_Delay(ms);
}

//定義MPU9255的i2c讀指令
uint8_t nmpu1_i2c_read(uint16_t DevAddress, uint8_t *pData, uint16_t size){
  return HAL_I2C_Master_Receive(&hi2c1, DevAddress, pData, size, 100);
}

//定義MPU9255的i2c寫指令
uint8_t nmpu1_i2c_write(uint16_t DevAddress, uint8_t *pData, uint16_t size){
  return HAL_I2C_Master_Transmit(&hi2c1, DevAddress, pData, size, 100);
}

void connecting_MPU_9255(MPU_9255_t *nmpu){
  // Read WHO_AM_I register for MPU-9255
  while(1){
    uint8_t who = MPU_9255_whoami(nmpu); 
    
    if (who == 0x73) // WHO_AM_I should always be 0x73
    {  
      printf("MPU9255 WHO_AM_I is 0x%x\n\r", who);
      printf("MPU9255 is online...\n\r");
      HAL_Delay(1000);
      
      MPU_9255_initSensor(nmpu);
      MPU_9255_printInfo(nmpu);
      break;
    }
    else
    {
      printf("Could not connect to MPU9255: \n\r");
      printf("%#x \n",  who);
      HAL_Delay(1000);
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  MPU_9255_t *nmpu = MPU_9255_new(
      MPU9255_ADDRESS_AD0_1, //AD0為0時的i2c地址
      SMPLRT_DIV_250Hz, 
      MGT_100HZ, 
      AFS_4G, 
      GFS_250DPS, 
      MFS_16BITS, 
      nmpu1_i2c_read, //前面自己定義好的i2c寫指令
      nmpu1_i2c_write //前面自己定義好的i2c讀指令
  );

  //connecting to MPU
  connecting_MPU_9255(nmpu);

  double ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
  
  const double PI = atan(1) * 4;
	double yaw, pitch, roll;
	double deltat = 0.0f;                      // integration interval for both filter schemes
  volatile uint32_t lastUpdate = tim3GetUs();
  volatile uint32_t Now = 0; // used to calculate integration interval                               // used to calculate integration interval

  while (1)
  {
    if(MPU_9255_isDataReady(nmpu)) {  							// On interrupt, check if data ready interrupt
			MPU_9255_readAccelData(nmpu, &ax, &ay, &az);  	// Read the accel x/y/z adc values  
			MPU_9255_readGyroData(nmpu, &gx, &gy, &gz);  	// Read the gyro x/y/z adc values
			MPU_9255_readMagData(nmpu, &mx, &my, &mz);  		// Read the mag x/y/z adc values
      Now = tim3GetUs();
      deltat = ((double)(Now - lastUpdate))/1000000.0f ; // set integration time by time elapsed since last filter update
      lastUpdate = Now;

      MPU_9255_filterUpdate(nmpu, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, ax, ay, az, deltat);
			MPU_9255_getEulerDegreeFilter(nmpu, &yaw, &pitch, &roll);
      printf("Orientation: %f %f %f\n\r", yaw, pitch, roll);
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
