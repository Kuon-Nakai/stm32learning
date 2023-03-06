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
#include "comp.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ld(x, s) \
	HAL_GPIO_WritePin(GPIOC, 0x80 << x, s); \
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
	

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t 	adcBuf[2]	= {0};
uint8_t		fmtBuf[32]	= {0};
uint32_t	ic1			= 0;
uint32_t	ic2			= 0;
uint32_t	*v1			= adcBuf;
uint32_t	*v2			= adcBuf + 1;
uint8_t		vl			= 1;
uint8_t		fl			= 2;
uint8_t		vlt			= 1;
uint8_t		flt			= 2;

bool		ric1		= false;
bool 		ric2		= false;
bool		para		= false;
bool		fol1		= true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void render(void);
void setF1(void);
void setF2(void);
void setV1(void);
void setV2(void);
void showVl(void);
void showFl(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *);
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
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
  MX_COMP3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, adcBuf, 2);
  
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  render();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ric1) setF1();
	  if(ric2) setF2();
	  setV1();
	  setV2();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	switch(pin) {
		case GPIO_PIN_0: // b1
			HAL_Delay(32);
			if(HAL_GPIO_ReadPin(GPIOB, pin) != GPIO_PIN_RESET) break;
			para = !para;
			render();
			break;
		case GPIO_PIN_1: // b2
			HAL_Delay(32);
			if(HAL_GPIO_ReadPin(GPIOB, pin) != GPIO_PIN_RESET) break;
			//ld(vl, GPIO_PIN_SET);
			if(++vlt == flt) ++vlt;
			if(vlt == 9) vlt = 1;
			//ld(vl, v1 < v2);
			break;
		case GPIO_PIN_2: // b3
			HAL_Delay(32);
			if(HAL_GPIO_ReadPin(GPIOB, pin) != GPIO_PIN_RESET) break;
			//ld(fl, GPIO_PIN_SET);
			if(++flt == vlt) ++flt;
			if(flt == 9) flt = 1;
			//ld(fl, ic1 < ic2);
			break;
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) { // b4
	if(hcomp->Instance != COMP3) return;
	HAL_Delay(32);
	if(HAL_COMP_GetOutputLevel(&hcomp3) != COMP_OUTPUT_LEVEL_LOW) return;
	fol1 = !fol1;
	if(fol1) setF1();
	else setF2();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) { // PA1
		ic1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
		ric1 = true;
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
		return;
	}
	if(htim->Instance == TIM15) { // PA2
		ic2 = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);
		ric2 = true;
		HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
		return;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	if(!para && hadc->Instance == ADC2) {
//		setV1();
//		setV2();
//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM4) 
		HAL_ADC_Start_DMA(&hadc2, adcBuf, 2);
}

inline void setF1() {
	ld(fl, ic1 < ic2);
	if(fol1) {
		__HAL_TIM_SET_AUTORELOAD(&htim3, ic1);
		__HAL_TIM_SetCounter(&htim3, 0);
	}
	if(para) return;
	LCD_DisplayStringLine(Line5, format("    F1:%dHz     ", 1000000 / ic1));
}
inline void setF2() {
	ld(fl, ic1 < ic2);
	if(!fol1) {
		__HAL_TIM_SET_AUTORELOAD(&htim3, ic2);
		__HAL_TIM_SetCounter(&htim3, 0);
	}
	if(para) return;
	LCD_DisplayStringLine(Line6, format("    F2:%dHz     ", 1000000 / ic2));
}
inline void setV1() {
	ld(vl, *v1 < *v2);
	if(para) return;
	LCD_DisplayStringLine(Line3, format("    V1:%.1fV    ", *v1 * 3.3 / 4096));
}
inline void setV2() {
	ld(vl, *v1 < *v2);
	if(para) return;
	LCD_DisplayStringLine(Line4, format("    V2:%.1fV    ", *v2 * 3.3 / 4096));
}
inline void showVl() {
	if(!para) return;
	LCD_DisplayStringLine(Line3, format("    VD:LD%1d    ", vl));
}
inline void showFl() {
	if(!para) return;
	LCD_DisplayStringLine(Line4, format("    FD:LD%1d    ", fl));
}

void render() {
	if(para) {
		LCD_DisplayStringLine(Line1, (uint8_t *)"    PARA");
		setV1();
		setV2();
		setF1();
		setF2();
	}
	else {
		LCD_DisplayStringLine(Line1, (uint8_t *)"    DATA");
		if(vlt != vl) {
			ld(vl, 1);
			vl = vlt;
			ld(vl, *v1 > *v2);
		}
		if(flt != fl) {
			ld(fl, 1);
			fl = flt;
			ld(fl, ic1 > ic2);
		}
		showVl();
		showFl();
	}
}

uint8_t *format(char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	vsprintf((char *)fmtBuf, fmt, ap);
	return fmtBuf;
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
