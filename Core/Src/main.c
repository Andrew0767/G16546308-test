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
#include "stdio.h"

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
uint8_t key_value = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//
int fputc(int ch,FILE *p){
	HAL_UART_Transmit(&huart1,(u8 *)&ch,1,HAL_MAX_DELAY);
	return ch;
}
//
uint8_t Key_Scan(){
	static struct{
		uint8_t state;
		uint32_t timestamp;
		uint8_t click_count;  
		uint32_t last_release_time;
	}key_status[4] = {0};
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == 0){//普通消抖
		if(key_status[0].state == 0){
			key_status[0].state = 1;
			key_status[0].timestamp = uwTick;
		}
		else if(key_status[0].state == 1&&(uwTick - key_status[0].timestamp) > 100){
			key_status[0].state = 2;
		}
	}
	else if(key_status[0].state == 2){
		key_status[0].state = 0;
		return 1;
	}
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == 0){
		if(key_status[1].state == 0){
			key_status[1].state = 1;
			key_status[1].timestamp = uwTick;
		}
		else if(key_status[1].state == 1){
			if((uwTick - key_status[1].timestamp) > 100){
				key_status[1].state = 2;
				key_status[1].timestamp = uwTick;
			}
		}
		else if(key_status[1].state == 2){
			if((uwTick - key_status[1].timestamp) > 1000){
				key_status[1].state = 3;
				return 5;
			}
		}
	}
	else if(key_status[1].state >= 2){
		if(key_status[1].state == 2){
			key_status[1].state = 0;
			return 2;
		}
		key_status[1].state = 0;
	}
	
	
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0) {  // 按键3被按下
    if(key_status[2].state == 0) {
        key_status[2].state = 1;  // 按下状态
        key_status[2].timestamp = uwTick;
    }
    else if(key_status[2].state == 1 && (uwTick - key_status[2].timestamp) > 1000) {
        key_status[2].state = 2;  // 长按状态
    }
}
else if(key_status[2].state == 2) {  // 按键3被释放
    key_status[2].state = 0;
    return 3;  // 返回按键3的值（单击）
}
else if(key_status[2].state == 1) {  // 按键3被按下但未释放
    key_status[2].state = 0;
    key_status[2].click_count++;  // 增加点击计数
    key_status[2].last_release_time = uwTick;  // 记录释放时间

    if(key_status[2].click_count == 2 && (uwTick - key_status[2].last_release_time) < 200) {  // 双击检测时间间隔调整为200毫秒
        key_status[2].click_count = 0;  // 重置点击计数
        return 6;  // 返回6表示双击
    }
    else if(key_status[2].click_count == 1&& (uwTick - key_status[2].last_release_time) > 1000) { 
       // 单击检测时间间隔调整为200毫秒
        key_status[2].click_count = 0;  // 重置点击计数
        key_status[2].state = 0;  // 重置状态
        return 3;  // 返回3表示单击
    }
}
	
	return 0;
}
//
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
  /* USER CODE BEGIN 2 */
  LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);
		LCD_Clear(Black);

    while (1)
    {
			key_value = Key_Scan();
			switch(key_value){
				case 1:
					printf("%d\r\n",key_value);
					break;
				case 2:
					printf("%d\r\n",key_value);
					break;
				case 3:
					printf("%d\r\n",key_value);
					break;
				case 5:
					printf("%d\r\n",key_value);
					break;
				case 6:
					printf("%d\r\n",key_value);
					break;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
