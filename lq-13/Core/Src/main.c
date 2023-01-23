/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LCD/lcd.h"
#include "LCD/delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define transmit(x) HAL_UART_Transmit(&huart1, x, sizeof(x), 0xffff)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t 	f 				= 1; // kHz
uint8_t		d					= 0;
uint8_t 	lck 			= 0;
volatile uint32_t v	= 0;
uint8_t		recv[10]	= {0};
char tosConvBuf[10] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void fDspl(void);
void vDspl(float voltage);
void dDspl(void);
void writeLog(char txt[]);
char* toString(long long num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	/*
	k up - PA0
	k0	 - PC5
	k1	 - PA15
	l1	 - PA8
	l2	 - PD2
	pwm	 - TIM3CH2, f = 1MHz / ARR, d = CCR2 / ARR
	*/
	int i = 0;
	//transmit("Starting...");
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	
	HAL_UART_Init(&huart1);
	
	HAL_TIM_Base_Start(&htim3);
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, &v, 1);
	
	
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
	
	delay_init(72);
	LCD_Init();
	LCD_DisplayOn();
	BACK_COLOR = BLACK;
	POINT_COLOR = WHITE;
	LCD_Clear(BLACK);
	LCD_ShowString(100, 100, 120, 20, 16, (uint8_t *)"DATA");
	LCD_ShowString(10 , 120, 50 , 20, 16, (uint8_t *)"Volt:");
	LCD_ShowString(10 , 140, 50 , 20, 16, (uint8_t *)"D:");
	LCD_ShowString(10 , 160, 50 , 20, 16, (uint8_t *)"F:");
	
	HAL_UART_Receive_IT(&huart1, recv, 10);
	
	fDspl();
	
	//transmit("Init complete");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//writeLog(toString(v));
		float voltage = (float)v / 4096 * 3.3;
		d = voltage < 1 ? 40 : voltage > 2 ? 80 : voltage * 40;
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, d * 10 * f);
		HAL_ADC_Start_DMA(&hadc1, &v, 1);
		vDspl(voltage);
		dDspl();
		if(voltage < 1) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
		if(i++ == 10) {
			if(voltage > 1) HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
			i = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	writeLog("UART rx");
	float volt = 0;
	int decimal = 0;
	for(int i = 0;i < 10; i++) {
		if(recv[i] == 0 || recv[i] == '\n') break;
		if(recv[i] == '.') {
			decimal = 10;
			continue;
		}
		if(recv[i] < '0' || recv[i] > '9') {
			transmit("Format error\n");
			return;
		}
		if(!decimal) {
			volt *= 10;
			volt += recv[i] - '0';
			continue;
		}
		volt += (float)(recv[i] - '0') / decimal;
		decimal *= 10;
	}
	
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, volt / 3.3 * 4096);
	HAL_UART_Receive_IT(&huart1, recv, 10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch(GPIO_Pin) {
		case GPIO_PIN_0: //  lock
			HAL_Delay(100);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) return;
			lck = lck ? 0 : 1;
			//transmit("Lock status:");
			//transmit(lck ? "locked" : "unlocked");
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
			break;
		case GPIO_PIN_5: //  +
			HAL_Delay(100);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) return;
			//writeLog("f+");
			if(lck || f == 10) return;
			++f;
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1000 / f);
			//transmit("f+");
			fDspl();
			break;
		case GPIO_PIN_15: // -
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) return;
			//writeLog("f-");
			if(lck || f == 1) return;
			--f;
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1000 / f);
			
			fDspl();
			break;
	}
}

inline char* toString(long long num) {
  if(sprintf(tosConvBuf, "%lld   ", num) == -1) {
    //ERR
    writeLog("Exception occurred while converting LL value to string");
    return NULL;
  }
  return tosConvBuf;
}

void fDspl() {
	char buf[10] = {0};
	sprintf(buf, "%d000Hz   ", f);
	LCD_ShowString(65, 160, 100, 20, 16, (uint8_t *)buf);
}
void vDspl(float voltage) {
	char buf[10] = {0};
	sprintf(buf, "%4.2fV   ", voltage);
	LCD_ShowString(65, 120, 100, 20, 16, (uint8_t *)buf);
}
void dDspl() {
	char buf[10] = {0};
	sprintf(buf, "%d%%   ", d);
	LCD_ShowString(65, 140, 100, 20, 16, (uint8_t *)buf);
}

void writeLog(char txt[]) {
	LCD_ShowString(200, 10, 200, 16, 16, (uint8_t *)txt);
}
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
