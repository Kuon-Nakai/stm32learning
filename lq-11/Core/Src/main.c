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

#define debug 0

#if debug

#include <time.h>

#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define modeUpd() LCD_ShowString(100, 140, 100, 20, 16, (uint8_t *)(autoMode ? "AUTO" : "MANU"))

#define CCR1()  autoMode ? voltage / .4096 - 1 : (pa6Rate << 10) - (pa6Rate << 4) - (pa6Rate << 3) - 1

#define CCR20() autoMode ? (10 - voltage / 409.6) * 250 - 1 : (10 - pa7Rate) * 250 - 1

#define CCR21()  4999

#define CCR22() autoMode ? (30 - voltage / 409.6) * 250 - 1 : (30 - pa7Rate) * 250 - 1

#define CCR23()  9999

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

bool 			paraPage 	= false;
uint32_t	voltage		=	0; // of 4096
bool			autoMode	= true;
char			pa6Rate		= 1; // x10%
char 			pa7Rate		= 1; // x10%
char 			buf[32]		= {0};
uint32_t 	adcBuf		= 0;
char 			pa7Step		= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern void delay_init(uint8_t);
void HAL_GPIO_EXTI_Callback(uint16_t);
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void rerender(void);
void voltUpd(void);
void pa6Upd(void);
void pa7Upd(void);

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, &adcBuf, 1);
	
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1024);

	delay_init(72);
	LCD_Init();
	BACK_COLOR = BLACK;
	POINT_COLOR = WHITE;
	rerender();

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if debug
	int clk;
	int cnt;
	bool in = false;
#endif
  while (1)
  {
#if debug
		clk = clock();
		if(clk >= 2 * CLOCKS_PER_SEC) {
			if(in) {
				LCD_ShowNum(160, 20, clk * 1000 / CLOCKS_PER_SEC, 4, 16);
				in = false;
			}
		}
		else if(clk >= CLOCKS_PER_SEC) {
			if(!in) {
				LCD_ShowNum(160, 20, clk * 1000 / CLOCKS_PER_SEC, 4, 16);
				in = true;
			}
			LCD_ShowNum(160, 40, ++cnt, 5, 16);
		}
#endif
		voltage = adcBuf;
		if(autoMode) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, CCR1());
		}
		if(!paraPage) voltUpd();
		HAL_ADC_Start_DMA(&hadc1, &adcBuf, 1);
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

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	switch(pin) {
		case GPIO_PIN_0: //interface
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET) return;
			paraPage = ! paraPage;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, paraPage ? GPIO_PIN_SET : GPIO_PIN_RESET);
			rerender();
			break;
		case GPIO_PIN_5: //pa6 +
			if(!paraPage) return;
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) != GPIO_PIN_RESET) return;
			if(pa6Rate == 9) {
				pa6Rate = 1;
				if(!autoMode) __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, CCR1());
				//__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pa6Rate * 1000);
				pa6Upd();
				break;
			}
			++pa6Rate;
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (pa6Rate << 10) - (pa6Rate << 4) - (pa6Rate << 3) - 1);
			//__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pa6Rate * 1000);
			pa6Upd();
			break;
		case GPIO_PIN_15://pa7 +
			if(!paraPage) return;
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != GPIO_PIN_RESET) return;
			if(pa7Rate == 9) {
				pa7Rate = 1;
				pa7Upd();   //pa7 does not require updating CCR2 through a function as it is manually controlled
				break;
			}
			++pa7Rate;
			pa7Upd();
			break;
		case GPIO_PIN_1: //mode
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) != GPIO_PIN_RESET) return;
			autoMode = !autoMode;
			modeUpd();
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, autoMode ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	switch(pa7Step) {
		case 0: //+
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CCR20());
			++pa7Step;
			break;
		case 1: //-
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CCR21());
			++pa7Step;
			break;
		case 2: //+
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CCR22());
			++pa7Step;
			break;
		case 3: //-
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CCR23());
			pa7Step = 0;
			break;
	}
}

inline void rerender() {
	LCD_Clear(BLACK);
	if(paraPage) {
		LCD_ShowString(100, 100, 100, 20, 16, (uint8_t *)"Para");
		LCD_ShowString( 20, 120, 100, 20, 16, (uint8_t *)"PA6:");
		LCD_ShowString( 20, 140, 100, 20, 16, (uint8_t *)"PA7:");
		pa6Upd();
		pa7Upd();
		return;
	}
	LCD_ShowString(100, 100, 100, 20, 16, (uint8_t *)"Data");
	LCD_ShowString( 20, 120, 100, 20, 16, (uint8_t *)"V:");
	LCD_ShowString( 20, 140, 100, 20, 16, (uint8_t *)"Mode:");
	voltUpd();
	modeUpd();
}

inline void voltUpd() {
	sprintf(buf, "%.2fV", voltage * 3.3 / 4096);
	LCD_ShowString( 50, 120, 100, 20, 16, (uint8_t *)buf);
	memset(buf, 0, 32);
}

inline void pa6Upd() {
	LCD_ShowNum(100, 120, pa6Rate, 1, 16);
	LCD_ShowString(116, 120, 100, 20, 16, (uint8_t *)"0%");
}

inline void pa7Upd() {
	LCD_ShowNum(100, 140, pa7Rate, 1, 16);
	LCD_ShowString(116, 140, 100, 20, 16, (uint8_t *)"0%");
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
