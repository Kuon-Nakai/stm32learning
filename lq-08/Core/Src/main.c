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
#include "comp.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef void *Func();

typedef struct {
	bool active;
	uint8_t delay;
	Func *invoke;
	void *then;
} DelayedAction;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ld(x, s)					HAL_GPIO_WritePin(GPIOC, 0x0080 << x, s);\
													HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
													HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#define kdown(x)					((x == 4 ? HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) : HAL_GPIO_ReadPin(GPIOB, 1 << (x - 1))) == GPIO_PIN_RESET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

bool 					b1d				= false;
bool 					b2d				= false;
bool 					b3d				= false;
bool 					b4d				= false;
bool					arrive		= false;
bool					tgt[4]		= {0};
int8_t				dir				= 0;
uint8_t 			level			= 1;
uint8_t				buf[32]		= {0};
DelayedAction delays[9] = {0};
DelayedAction empty;
RTC_TimeTypeDef time;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *);
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);

void updTime(void);
void updLvl(void);
void arriveAt(uint8_t);
uint8_t *format(char *fmt, ...);

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
	empty.active = false;
	empty.delay  = 0;
	empty.invoke = NULL;
	empty.then	 = NULL;
	
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
  MX_COMP3_Init();
  MX_RTC_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	
	
	HAL_COMP_Start(&hcomp3);
	
	HAL_TIM_Base_Start_IT(&htim6);
	
	
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	LCD_DisplayStringLine(Line2, (uint8_t *)"????");
	
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		if(arrive) {
			HAL_GPIO_WritePin(GPIOC, 0x0080 << level | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
			//off
			HAL_Delay(250);
			//on
			HAL_Delay(250);
			//off
			HAL_Delay(250);
			//on
		}
		updTime();
		if(dir == 1) {
			
			continue;
		}
		if(dir == -1) {
			
			continue;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) { //b4
	if(level == 4 || tgt[3]) return;
	HAL_Delay(20);
	if(!kdown(4)) return;
	tgt[3] = true;
	ld(4, 0);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	switch(pin) {
		case GPIO_PIN_0: //b1
			if(level == 1 || tgt[0]) return;
			HAL_Delay(20);
			if(!kdown(1)) return;
			tgt[0] = true;
			ld(1, 0);
			break;
		case GPIO_PIN_1: //b2
			if(level == 2 || tgt[1]) return;
			HAL_Delay(20);
			if(!kdown(2)) return;
			tgt[1] = true;
			ld(2, 0);
			break;
		case GPIO_PIN_2: //b3
			if(level == 3 || tgt[2]) return;
			HAL_Delay(20);
			if(!kdown(3)) return;
			tgt[2] = true;
			ld(3, 0);
			break;
	}
}

void updTime() {
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BCD);
	LCD_DisplayStringLine(Line7, format("    %2d : %2d : %2d", time.Hours, time.Minutes, time.Seconds));
}

void arriveAt(uint8_t l) {
	level = l;
	tgt[l] = false;
	arrive = true;
}

uint8_t *format(char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	memset(buf, 0, 32);
	vsprintf(buf, fmt, ap);
	return buf;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM6) {
		for(int i = 0; i < 9; i++) {
			if(!delays[i].active) continue;
			if(!--delays[i].delay) {
				delays[i].invoke();
				if(delays[i].then != NULL) {
					((DelayedAction *)(delays[i].then))->active = true;
					delays[i] = empty;
				}
			}
		}
		
	}
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
