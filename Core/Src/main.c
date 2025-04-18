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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "i2c_hal.h"
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
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t key_value = 0;
double adc1_value = 0,adc2_value = 0;
double fp7 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
////
int fputc(int ch,FILE *p){
	HAL_UART_Transmit(&huart1,(u8 *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

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
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == 0){//长按
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
        key_status[2].state = 1;  
        key_status[2].timestamp = uwTick;
    }
    else if(key_status[2].state == 1 && (uwTick - key_status[2].timestamp) > 1000) {
        key_status[2].state = 2;  
    }
	}
	else if(key_status[2].state == 2) {  // 按键3被释放
			key_status[2].state = 0;
			return 3;  // 返回按键3的值（单击）
	}
	else if(key_status[2].state == 1) {  // 按键3被按下但未释放
    key_status[2].state = 0;
    key_status[2].click_count++;  
    key_status[2].last_release_time = uwTick;  

    if(key_status[2].click_count == 2 && (uwTick - key_status[2].last_release_time) < 200) {  // 双击检测时间间隔调整为200毫秒
        key_status[2].click_count = 0;  
        return 6;  // 返回6表示双击
    }
    else if(key_status[2].click_count == 1&& (uwTick - key_status[2].last_release_time) > 1000) { 
       // 单击检测时间间隔调整为200毫秒
        key_status[2].click_count = 0;  
        key_status[2].state = 0;  
        return 3;  // 返回3表示单击
    }
	}

	return 0;
}

double Adc1_Get(){//R38
	double adc = 0;
	HAL_ADC_Start(&hadc1);
	
	adc = HAL_ADC_GetValue(&hadc1);
	
	return adc;
}
double Adc2_Get(){//R37
	double adc = 0;
	HAL_ADC_Start(&hadc2);
	
	adc = HAL_ADC_GetValue(&hadc2);
	
	return adc;
}

void Led_Control(int ledNum,int ledState){
	static int status = 0xFF;
	
	
	if(ledNum > 8 || ledNum < 1) return;
	
	if(ledState == 0){
		status |= 0x01<<(ledNum - 1);
	}
	else if(ledState == 1){
		status &= ~(0x01<<(ledNum-1));
	}
	
	GPIOC->ODR = status<<8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
}

void PwmPro(){
	HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);
	
	TIM2->ARR = 5000-1;//将tim2频率改为1000000/5000=200Hz
	HAL_TIM_Base_Init(&htim2);
	TIM2->CCR2 = 2500;//等于__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,200),将tim2通道2设为占空比为TIM2->CCR2/TIM2->ARR=40%
	HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_2);
	__HAL_TIM_SET_AUTORELOAD(&htim2,500);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,200);
}

uint16_t cap_up_count = 0,cap_down_count = 0;
float cap_duty = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		cap_up_count = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2)+1;
		cap_duty = (float)cap_down_count/cap_up_count;
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		cap_down_count = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1)+1;
	}
}

uint8_t uart_rx_buf[64] = {0};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(uart_rx_buf[0] == 'a'){
		x24c02_write(0,0);
		HAL_Delay(5);
		printf("%d\r\n",x24c02_read(0));
	}
	HAL_UART_Receive_IT(&huart1,uart_rx_buf,1);
}

uint8_t uart_rx_dma_buf[64];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == &huart1){
		if(uart_rx_dma_buf[0] == '1'&&uart_rx_dma_buf[1] == '2'&&uart_rx_dma_buf[4] == '5')
			printf("ok!");
		else
			printf("error!");
				memset(uart_rx_dma_buf,0,sizeof(uart_rx_dma_buf));
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart_rx_dma_buf,sizeof(uart_rx_dma_buf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static int time_ms = 0;
	if(htim == &htim16){
		time_ms++;
		if(time_ms == 1000){
			printf("%d\r\n",time_ms);
			time_ms = 0;
		}
	}
}

uint8_t x24c02_read(uint8_t address){
	unsigned char val;
	
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(address);
	I2CWaitAck();
	I2CStop();
	
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	val = I2CReceiveByte();
	I2CSendNotAck();
	I2CStop();
	
	return val;
}

void x24c02_write(unsigned char address,unsigned char info){
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(address);
	I2CWaitAck();
	I2CSendByte(info);
	I2CWaitAck();
	I2CStop();
}
//
void DAC1_OUT1_Set_Vol(float vol) //设置PA4的输出电压
{
	uint16_t temp;
	temp = (4096*vol/3.3f);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R,temp);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);//启动DAC1 通道1输出
}
//
void DAC1_OUT2_Set_Vol(float vol) //设置PA5的输出电压
{
	uint16_t temp;
	temp = (4096*vol/3.3f);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,DAC_ALIGN_12B_R,temp);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);//启动DAC1 通道2输出
}
//
uint32_t  f39 = 0, f40 = 0; 
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t  cc1_value = 0;
	cc1_value = __HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SetCounter(htim,0);
	
	if(htim == &htim2) //定时器2，对应的是PA15引脚，R40
	{
		f40 = 1000000/cc1_value;
	}
	
	if(htim == &htim3) //定时器3，对应的是PB4引脚，R39
	{
		f39 = 1000000/cc1_value;
	}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
	I2CInit();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
	
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_UART_Receive_IT(&huart1,uart_rx_buf,1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart_rx_dma_buf,sizeof(uart_rx_dma_buf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	HAL_TIM_Base_Start_IT(&htim16);
	
	DAC1_OUT1_Set_Vol(2.3);
	DAC1_OUT2_Set_Vol(1.5);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);
		LCD_Clear(Black);
    while (1)
    {
			printf("%d\r\n",f40);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
