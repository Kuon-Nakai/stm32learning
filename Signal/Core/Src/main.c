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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef void *Function(void);

typedef struct _DelayedAction {
	uint32_t 	delay;
	Function *	invoke;
	bool		active;
	void *		next;
	void *		prev;
	void *		activate;
} DelayedAction;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define lds(x, s)\
	HAL_GPIO_WritePin(GPIOC, 0x80 << x, s);\
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t segNum[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
DelayedAction *firstAct = NULL;
DelayedAction *lastAct	= NULL;

uint8_t countdown = 0;
uint8_t ldn = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void segShow(uint8_t, uint8_t, uint8_t);
DelayedAction *new_delay(bool, uint32_t, Function, DelayedAction *);
void appendWork(void);
void aPass(void);
void aStop(void);
void bPass(void);
void bStop(void);

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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	LCD_DisplayStringLine(Line3, (uint8_t *)"    Signal info");

	lds(1, 0);
	lds(2, 0);
	lds(3, 0);
	lds(4, 0);
	lds(5, 0);
	lds(6, 1);
	lds(7, 1);
	lds(8, 1);
	
	aPass();
	LCD_DisplayStringLine(Line7, (uint8_t *)"    B: Stop  ");
//	if(!new_delay( true,  1000, (Function *)aPass, NULL)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
	if(!new_delay( true, 44000, (Function *)aStop, NULL)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
	if(!new_delay(false,  5000, (Function *)bPass, lastAct)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
	if(!new_delay(false, 25000, (Function *)bStop, lastAct)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
	
	HAL_Delay(32);
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
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

void appendWork() {
	if(!new_delay(false,  5000, (Function *)aPass, lastAct)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
	if(!new_delay(false, 45000, (Function *)aStop, lastAct)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
	if(!new_delay(false,  5000, (Function *)bPass, lastAct)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
	if(!new_delay(false, 25000, (Function *)bStop, lastAct)) LCD_DisplayStringLine(Line9, (uint8_t *)"MALLOC FAILED");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM6) { //seg update timer, 1Hz
		if(countdown) --countdown;
		int cd10 = countdown / 10;
		segShow(0x00, cd10 ? segNum[cd10] : 0x00, segNum[countdown % 10]);
		if(ldn) lds(ldn--, 1);
		HAL_TIM_Base_Start_IT(&htim6);
	}
	if(htim->Instance == TIM7) { //delay handler timer, 1kHz
		DelayedAction *temp = NULL;
		DelayedAction *act  = NULL;
		if(firstAct == NULL) return;
		if(firstAct->active) {
			if(--(firstAct->delay) == 0) {
				firstAct->invoke();
				if(firstAct->activate) 	((DelayedAction *)firstAct->activate)->active = true;
				if(firstAct->next)		((DelayedAction *)firstAct->next)->prev = NULL;
				else					lastAct = NULL;
				temp = (DelayedAction *)firstAct->next;
				free(firstAct);
				firstAct = temp;
			}
			else temp = (DelayedAction *)firstAct->next;
		}
		else temp = (DelayedAction *)firstAct->next;
		
		while(temp) {
			if(temp->active) {
				if(--(temp->delay) == 0) {
					temp->invoke();
					act = temp;
					if(act->activate) 	((DelayedAction *)act->activate)->active = true;
					if(act->prev)		((DelayedAction *)act->prev)->next = act->next;
					if(act->next)		((DelayedAction *)act->next)->prev = act->prev;
					else				lastAct = NULL;
					temp = (DelayedAction *)act->next;
					free(act);
					act = NULL;
				}
				else temp = (DelayedAction *)temp->next;
			}
			else temp = (DelayedAction *)temp->next;
		}
		HAL_TIM_Base_Start_IT(&htim7);
	}
}

void aPass() {
	LCD_DisplayStringLine(Line6, (uint8_t *)"    A: Pass  ");
	countdown = 45;
	segShow(0x00, segNum[4], segNum[5]);
}

void bPass() {
	LCD_DisplayStringLine(Line7, (uint8_t *)"    B: Pass  ");
	countdown = 25;
	segShow(0x00, segNum[2], segNum[5]);
}

void aStop() {
	LCD_DisplayStringLine(Line6, (uint8_t *)"    A: Stop  ");
	lds(1, 0);
	lds(2, 0);
	lds(3, 0);
	lds(4, 0);
	lds(5, 0);
	ldn = 5;
}

void bStop() {
	LCD_DisplayStringLine(Line7, (uint8_t *)"    B: Stop  ");
	appendWork();
	lds(1, 0);
	lds(2, 0);
	lds(3, 0);
	lds(4, 0);
	lds(5, 0);
	ldn = 5;
}

DelayedAction *new_delay(bool active, uint32_t delay, Function func, DelayedAction *activatedBy) {
	DelayedAction *n = (DelayedAction *)malloc(sizeof(DelayedAction));
	n->active 		= active;
	n->invoke 		= func;
	n->delay		= delay;
	n->prev			= lastAct;
	n->next			= NULL;
	if(lastAct)lastAct->next = n;
	lastAct = n;
	if(!firstAct) firstAct = n;
	if(activatedBy) activatedBy->activate = n;
	return n;
}

void segShow(uint8_t n1, uint8_t n2, uint8_t n3) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	for(int i = 8; i--; ) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, n3 & 128u);
		n3 <<= 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
	for(int i = 8; i--; ) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, n2 & 128u);
		n2 <<= 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
	for(int i = 8; i--; ) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, n1 & 128u);
		n1 <<= 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
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
