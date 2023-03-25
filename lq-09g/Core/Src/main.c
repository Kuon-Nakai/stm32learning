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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "i2c.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _adcData {
	uint32_t vin;
	uint32_t key;
} ADCData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define dspl(ln, txt) \
	LCD_DisplayStringLine(ln * 24, (uint8_t *)txt)

#define putKeyBuf(k, s) \
	keyBuf[k] = (keyBuf[k] << 1) | s

#define getKey(r) \
	((r < 2048) ? \
		(r < 128) ? 1 : \
		(r < 1024) ? 2 : \
		(r < 1500) ? 3 : \
		4 : \
	(r < 2500) ? 5 : \
	(r < 3000) ? 6 : \
	(r < 3700) ? 7 : \
	(r < 4000) << 3)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADCData 	adcBuf;
uint8_t		keyBuf		= 0;
uint8_t		keyStreak	= 0;
uint32_t	keyIn		= 0;
bool 		settings 	= false;
uint8_t		goodsCode	= 1; // 1-3
uint32_t	*unitPrices;
uint32_t	changes		= 0;
uint32_t 	weight		= 0;
bool		keyDown[9]	= {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void dsplf(int ln, const char *, ...);
void render(void);
void updateWeight(int wt);
void setGoodsCode(int code);
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
	unitPrices = (uint32_t *)malloc(128);
	unitPrices[1] = 1;
	unitPrices[2] = 2;
	unitPrices[3] = 5;
	
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
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//lcd init
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&adcBuf, 2);
	HAL_TIM_Base_Start_IT(&htim6);
	
	//i2c read & prices fetch
	
	LCD_Init();
	LCD_PowerOn();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	render();
	
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM6) {
		register uint8_t r = getKey(adcBuf.key);
		keyIn = r;
		if(r == keyBuf) {
			if(++keyStreak == 17) keyStreak = 16;
		}
		else {
			keyStreak = 1;
			keyDown[keyBuf] = false;
			keyBuf = r;
		}
		if(!r) goto cplt;
		if(keyStreak > 7) {
			switch(r) {
				case 2: // +
					
					goto cplt;
				case 3: // -
					
					goto cplt;
				default:
					break;
			}
		}
		float cost;
		if(keyStreak > 1) {
			if(r != 2 && r != 3 && keyDown[r]) return;
			keyDown[r] = true;
			switch(r) {
				case 1: //settings
					settings = ! settings;
					render();	
					break;
				case 2: // +
					break;
				case 3: // -
					break;
				case 4: // switch
					break;
				case 5: // g1
				case 6: // g2
				case 7: // g3
					setGoodsCode(r - 4);
					break;
				case 8: // calc
					cost = unitPrices[goodsCode] * weight / 10000.0;
					dsplf(6, "  Total: %.2f yuan   ", cost);
					printf("U.W.1:%.2f\nG.W.:%.2f\nTotal:%.2f\n", unitPrices[goodsCode] / 100.0, weight / 100.0, cost);
					break;
			}
		}
		cplt:
		r = abs((int)adcBuf.vin - (int)weight);
		if(r > 16) updateWeight(r);
		__HAL_TIM_ENABLE(&htim6);
	}
}

inline void setGoodsCode(int code) {
	if(code == goodsCode) return;
	goodsCode = code;
	if(settings) return;
	render();
}

inline void updateWeight(int wt) {
	weight = wt;
	if(settings) return;
	dsplf(5, "  Weight: %.2f kg    ", wt / 100.0);
	
}

inline void render() {
	LCD_Clear(Black);
	if(settings) {
		
		return;
	}
	dspl(1,  "    Weigh & Pay     ");
	dsplf(3, " Number: %d  ", goodsCode);
	dsplf(4, " Unit price: %.2f y/kg  ",unitPrices[goodsCode] / 100.0);
	updateWeight(weight);
}

inline void dsplf(int ln, const char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	uint8_t *buf = (uint8_t *)malloc(256);
	memset(buf, 0, 256);
	vsprintf((char *)buf, fmt, ap);
	dspl(ln, buf);
	free(buf);
	va_end(ap);
}

int fputc(int ch, FILE *f) {
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFF);
	return ch;
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
