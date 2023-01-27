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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "LCD/lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define findPin(x) (uint16_t)((x + (x == 3)) << (4 * (x / 4) + 8))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int8_t 		upper 		= 24;
volatile int8_t 		lower 		= 12;
uint8_t 	selection = 0;
uint32_t	adcBuf		= 0;
int8_t		upperl		= 1;
int8_t		lowerl		= 2;
char			buf[32]		= {0};
float			voltage		= 0;
bool 			bufLock		= false;
volatile bool	settings 	= false;
bool 			doRerender= false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern void delay_init(uint16_t);
void HAL_GPIO_EXTI_Callback(uint16_t);
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void rerender(void);
void setUpper(int8_t diff);
void setLower(int8_t diff);
void setUpperl(int8_t diff);
void setLowerl(int8_t diff);

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
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	
	/////////////////////////////////////////////////////////
	
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
	
	/////////////////////////////////////////////////////////
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, &adcBuf, 1);
	
	HAL_TIM_Base_Start_IT(&htim3);
	
	delay_init(72);
	LCD_Init();
	BACK_COLOR = BLACK;
	POINT_COLOR = WHITE;
	rerender();
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		voltage = adcBuf * 33.0 / 0x1000;
		if(!settings && !bufLock) {
			memset(buf, 0, 32);
			sprintf(buf, "%.1fV", voltage / 10);
			if(!settings) LCD_ShowString(120, 120, 100, 20, 16, (uint8_t *)buf);
			
		}
		if(doRerender) {
			rerender();
			doRerender = false;
		}
		HAL_ADC_Start_DMA(&hadc1, &adcBuf, 1);
		HAL_Delay(50);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t key) {
	switch(key) {
		case GPIO_PIN_4: //interface
			HAL_Delay(100);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) != GPIO_PIN_RESET) return;
			settings = !settings;
			if(settings) doRerender = true;
			else {
				rerender();
				HAL_TIM_PeriodElapsedCallback(&htim3);
			}
			break;
		case GPIO_PIN_0: //select
			if(!settings) return;
			HAL_Delay(100);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET) return;
			(selection == 3) ? (selection = 0) : (++selection);
			rerender();
			break;
		case GPIO_PIN_5: //+
			if(!settings) return;
			HAL_Delay(100);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) != GPIO_PIN_RESET) return;
			switch(selection) {
				case 0: setUpper(+3); break;
				case 1: setLower(+3); break;
				case 2: setUpperl(+1); break;
				case 3: setLowerl(+1); break;
			}
			break;
		case GPIO_PIN_15://-
			if(!settings) return;
			HAL_Delay(100);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != GPIO_PIN_RESET) return;
			switch(selection) {
				case 0: setUpper(-3); break;
				case 1: setLower(-3); break;
				case 2: setUpperl(-1); break;
				case 3: setLowerl(-1); break;
			}
			break;
	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	
	//HAL_GPIO_WritePin(GPIOC, findPin(upperl), voltage <= upper);
	//HAL_GPIO_WritePin(GPIOC, findPin(lowerl), voltage >= lower);
	
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance != TIM3) return;
	if(voltage > upper) {
		if(!settings) {LCD_ShowString(120, 140, 100, 20, 16, (uint8_t *)"Upper    ");}
		HAL_GPIO_TogglePin(GPIOC, findPin(upperl));
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else HAL_GPIO_WritePin(GPIOC, findPin(upperl), GPIO_PIN_SET);
	if(voltage < lower) {
		HAL_GPIO_TogglePin(GPIOC, findPin(lowerl));
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		if(!settings) {LCD_ShowString(120, 140, 100, 20, 16, (uint8_t *)"Lower    ");}
	}
	else {HAL_GPIO_WritePin(GPIOC, findPin(lowerl), GPIO_PIN_SET);}
	if(!settings && voltage <= upper && voltage >= lower) LCD_ShowString(120, 140, 100, 20, 16, (uint8_t *)"Normal  ");
	//LCD_ShowNum(20, 200, voltage, 6, 16);
}

inline void setUpper(int8_t diff) {
	if(upper + diff > 33 || upper + diff < lower) return;
	upper += diff;
	BACK_COLOR = WHITE;
	POINT_COLOR= BLACK;
	bufLock = true;
	memset(buf, 0, 32);
	sprintf(buf, "Max Volt: %.1fV", upper / 10.0);
	LCD_ShowString( 50, 120, 200, 20, 16, (uint8_t *)buf);
	memset(buf, 0, 32);
	bufLock = false;
	BACK_COLOR = BLACK;
	POINT_COLOR= WHITE;
}

inline void setLower(int8_t diff) {
	if(lower + diff < 0 || lower + diff > upper) return;
	lower += diff;
	BACK_COLOR = WHITE;
	POINT_COLOR= BLACK;
	bufLock = true;
	memset(buf, 0, 32);
	sprintf(buf, "Min Volt: %.1fV", lower / 10.0);
	LCD_ShowString( 50, 140, 200, 20, 16, (uint8_t *)buf);
	memset(buf, 0, 32);
	bufLock = false;
	BACK_COLOR = BLACK;
	POINT_COLOR= WHITE;
}

inline void setUpperl(int8_t diff) {
	HAL_GPIO_WritePin(GPIOC, findPin(upperl), GPIO_PIN_SET);
	upperl += diff;
	if(upperl == 9) upperl = 1;
	if(upperl == 0) upperl = 8;
	if(upperl == lowerl) setUpperl(diff);
	BACK_COLOR = WHITE;
	POINT_COLOR= BLACK;
	bufLock = true;
	memset(buf, 0, 32);
	sprintf(buf, "Upper: LD%d", upperl);
	LCD_ShowString( 50, 160, 200, 20, 16, (uint8_t *)buf);
	memset(buf, 0, 32);
	bufLock = false;
	BACK_COLOR = BLACK;
	POINT_COLOR= WHITE;
}

inline void setLowerl(int8_t diff) {
	HAL_GPIO_WritePin(GPIOC, findPin(lowerl), GPIO_PIN_SET);
	lowerl += diff;
	if(lowerl == 9) lowerl = 1;
	if(lowerl == 0) lowerl = 8;
	if(lowerl == upperl) setLowerl(diff);
	BACK_COLOR = WHITE;
	POINT_COLOR= BLACK;
	bufLock = true;
	memset(buf, 0, 32);
	sprintf(buf, "lower: LD%d", lowerl);
	LCD_ShowString( 50, 180, 200, 20, 16, (uint8_t *)buf);
	memset(buf, 0, 32);
	bufLock = false;
	BACK_COLOR = BLACK;
	POINT_COLOR= WHITE;
}

inline void rerender(void) {
	LCD_Clear(BLACK);
	if(settings) {
		uint8_t i = 0;
		LCD_ShowString(100, 100, 250, 20, 16, (uint8_t *)"Setting");
		BACK_COLOR = (i == selection) ? WHITE : BLACK;
		POINT_COLOR = (i++ != selection) ? WHITE : BLACK;
		bufLock = true;
		memset(buf, 0, 32);
		sprintf(buf, "Max Volt: %.1fV", upper / 10.0);
		LCD_ShowString( 50, 120, 250, 20, 16, (uint8_t *)buf);
		BACK_COLOR = (i == selection) ? WHITE : BLACK;
		POINT_COLOR = (i++ != selection) ? WHITE : BLACK;
		memset(buf, 0, 32);
		sprintf(buf, "Min Volt: %.1fV", lower / 10.0);
		LCD_ShowString( 50, 140, 250, 20, 16, (uint8_t *)buf);
		BACK_COLOR = (i == selection) ? WHITE : BLACK;
		POINT_COLOR = (i++ != selection) ? WHITE : BLACK;
		memset(buf, 0, 32);
		sprintf(buf, "Upper: LD%d", upperl);
		LCD_ShowString( 50, 160, 250, 20, 16, (uint8_t *)buf);
		BACK_COLOR = (i == selection) ? WHITE : BLACK;
		POINT_COLOR = (i++ != selection) ? WHITE : BLACK;
		memset(buf, 0, 32);
		sprintf(buf, "Lower: LD%d", lowerl);
		LCD_ShowString( 50, 180, 250, 20, 16, (uint8_t *)buf);
		bufLock = false;
		BACK_COLOR = BLACK;
		POINT_COLOR = WHITE;
		memset(buf, 0, 32);
		return;
	}
	LCD_ShowString(100, 100, 100, 20, 16, (uint8_t *)"Main");
	LCD_ShowString( 50, 120, 100, 20, 16, (uint8_t *)"Volt:");
	LCD_ShowString( 50, 140, 100, 20, 16, (uint8_t *)"Status:");
	//Data will be updated by ADC and TIM3
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
