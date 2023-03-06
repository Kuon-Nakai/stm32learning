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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRAO_LIM 3500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define lds(x, s) \
	HAL_GPIO_WritePin(GPIOC, 0x0080<<(x), s);\
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t 	fmtBuf[32]	= {0};
bool 		paraPage	= false;
float		a			= 0;
float  		b 			= 0;
float		da			= 0;
float  		db 			= 0;
int 		ax			= 0;
int 		bx			= 0;
int 		pax			= 20;
int 		pbx			= 20;
int 		pf			= 1000;
int 		f			= 0;
int 		ah			= 0;
int 		bh			= 0;
bool		modeB		= false;
bool		rf			= false;
bool		ra			= false;
bool		rb			= false;
uint32_t	adcIn		= {0};
char 		rxBuf[32]	= {0};
bool		rxRdy		= false;
float 		recA[5]		= {0};
int			irA			= 0;
float 		recB[5]		= {0};
int 		irB			= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void render(void);
void updA(void);
void updB(void);
void updF(void);
void updAx(void);
void updBx(void);
void updMode(void);
void incPax(bool);
void incPbx(bool);
void incPf(bool);

float angleA(int, int);
float angleB(int, int);

void refreshAngle(void);

void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *);

int cmp(const void *, const void *);

uint8_t *format(char *, ...);
int fputc(int, FILE *);

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
  MX_USART1_UART_Init();
  MX_COMP3_Init();
  MX_TIM2_Init();
  MX_COMP2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	//HAL_ADC_Start_IT(&hadc2); //trao
	
	HAL_COMP_Start(&hcomp2);
	HAL_COMP_Start(&hcomp3);
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); //freq in
	HAL_TIM_Base_Start_IT(&htim15); //ADC trigger
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	render();
	
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); //pwm a
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); //pwm b
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuf, 31);
	printf("Init complete\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //LCD_DisplayStringLine(Line0, format("%d              ", adcIn)); //debug
	  
	  lds(1, ax < pax);
	  lds(2, bx < pbx);
	  lds(3, f  < pf );
	  lds(4, modeB   ); //move?
	  lds(5, a-b < 80);
	  
	  if(!paraPage && rf) {
		  updF();
		  rf = false;
	  }
	  if(!paraPage && ra) {
		  updA();
		  updAx();
		  recA[irA++] = a;
		  if(irA == 5) irA = 0;
		  ra = false;
	  }
	  if(!paraPage && rb) {
		  updB();
		  updBx();
		  recB[irB++] = b;
		  if(irB == 5) irB = 0;
		  rb = false;
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	if(huart->Instance == USART1) {
		//Cmd handler:
		//LCD_DisplayStringLine(Line0, (uint8_t *)rxBuf);
		if(rxBuf[1] == '?') {
			if(rxBuf[0] == 'a') { //a?
				printf("a:%.1f\n", a);
				goto end;
			}
			if(rxBuf[0] == 'b') { //b?
				printf("b:%.1f\n", b);
				goto end;
			}
		}
		else if(rxBuf[2] == '?') {
			rxBuf[2] = 0;
			if(!strcmp(rxBuf, "aa")) {
				printf("aa:%.1f", recA[irA]);
				for(int i = irA; --i != irA; ) {
					if(i == -1) i = 4;
					printf("-%.1f", recA[i]);
				}
				printf("\n");
				goto end;
			}
			if(!strcmp(rxBuf, "bb")) {
				printf("bb:%.1f", recB[irB]);
				for(int i = irB; --i != irB; ) {
					if(i == -1) i = 4;
					printf("-%.1f", recB[i]);
				}
				printf("\n");
				goto end;
			}
			if(!strcmp(rxBuf, "qa")) {
				float *q = (float *)malloc(sizeof(float)*5);
				memcpy(q, recA, sizeof(float)*5);
				//for(int i = 4; i--; ) *(q + i) = recA[i]; //array copy
				qsort(q, 5, sizeof(float), cmp); //sort
				printf("qa:%.1f", *(q + 4));
				for(int i = 4; i--; ) {
					printf("-%.1f", *(q + i));
				}
				free(q);
				printf("\n");
				goto end;
			}
			if(!strcmp(rxBuf, "qb")) {
				float *q = (float *)malloc(sizeof(float)*5);
				memcpy(q, recB, sizeof(float)*5);
				//for(int i = 4; i--; ) *(q + i) = recB[i]; //array copy
				qsort(q, 5, sizeof(float), cmp); //sort
				printf("qb:%.1f", *(q + 4));
				for(int i = 4; i--; ) {
					printf("-%.1f", *(q + i));
				}
				free(q);
				printf("\n");
				goto end;
			}
		}
		//Illegal format
		printf("error\n");
		
		//handler end
		end:
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuf, 31);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if(hadc->Instance == ADC2) {
		register int adcVal = HAL_ADC_GetValue(&hadc2);
		if(modeB && adcVal > TRAO_LIM && adcIn < TRAO_LIM) refreshAngle();
		adcIn = adcVal;
		//HAL_ADC_Start_IT(&hadc2);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM15) HAL_ADC_Start_IT(&hadc2);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	bool b1 = htim->Instance == TIM3, b2 = htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1;
//	UNUSED(b1);
//	UNUSED(b2);
	if(htim->Instance == TIM2) { //freq
		f = 1000000/ HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
		rf = true;
		return;
	}
	if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { //pwm A
		if(HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1) == 0) return;
		da = angleA(HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2), HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1));
		//ax = a1 - a;
		//a = a1;
//		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
//		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
		//ra = true;
		return;
	}
	if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { //pwm B
		if(HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1) == 0) return;
		db = angleB(HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2), HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1));
		//bx = b1 - b;
		//b = b1;
//		HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
//		HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_2);
		//rb = true;
		return;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	switch(pin) {
		case GPIO_PIN_0: //B1
			HAL_Delay(32);
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) != SET) return;
			paraPage = !paraPage;
			render();
			break;
		case GPIO_PIN_1: //B2
			HAL_Delay(32);
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) != SET) return;
			if(paraPage) {
				incPax(true);
				incPbx(true);
			}
			break;
		case GPIO_PIN_2: //B3
			HAL_Delay(32);
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) != SET) return;
			if(paraPage) incPf(true);
			else {
				modeB = !modeB;
				updMode();
			}
			break;
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
	if(hcomp->Instance == COMP3) { //B4
		if(modeB) return;
		HAL_Delay(32);
		if(HAL_COMP_GetOutputLevel(&hcomp3) == COMP_OUTPUT_LEVEL_HIGH) return;
		refreshAngle();
	}
}

inline float angleA(int h, int t) { //opt possible
	register int d = h * 10000 / t - 1000; //x1000
	if(d > 8000) return 180;
	if(d < 0) return 0; 
	return d * 9 / 400.0;
	
}
inline float angleB(int h, int t) { //opt possible
	register int d = h * 10000 / t - 1000; //x1000
	if(d > 8000) return 90;
	if(d < 0) return 0;
	return d * 9 / 800.0;
}

inline void refreshAngle() {
	ax= da - a;
	bx= db - b;
	a = da;
	b = db;
	ra = true;
	rb = true;
}

void render() {
	LCD_Clear(Black);
	if(paraPage) {
		LCD_DisplayStringLine(Line1, (uint8_t *)"        PARA");
		incPax(0);
		incPbx(0);
		incPf(0);
		return;
	}
	LCD_DisplayStringLine(Line1, (uint8_t *)"        DATA");
	updA();
	updB();
	updF();
	updAx();
	updBx();
	updMode();
}

inline void updA() {
	LCD_DisplayStringLine(Line2, format("   a:%.1f      ", a));
}
inline void updB() {
	LCD_DisplayStringLine(Line3, format("   b:%.1f      ", b));
}
inline void updF() {
	LCD_DisplayStringLine(Line4, format("   f:%dHz       ", f));
}
inline void updAx() {
	LCD_DisplayStringLine(Line6, format("   ax:%d       ", ax));
}
inline void updBx() {
	LCD_DisplayStringLine(Line7, format("   bx:%d     ", bx));
}
inline void updMode() {
	LCD_DisplayStringLine(Line8, format("   mode:%c", 'A' + modeB));
}
inline void incPax(bool inc) {
	if(inc) {
		pax += 10;
		if(pax == 70) pax = 10;
	}
	LCD_DisplayStringLine(Line2, format("   Pax:%d    ", pax));
}
inline void incPbx(bool inc) {
	if(inc) {
		pbx += 10;
		if(pbx == 70) pbx = 10;
	}
	LCD_DisplayStringLine(Line3, format("   Pbx:%d      ", pbx));
}
inline void incPf(bool inc) {
	if(inc) {
		pf += 1000;
		if(pax == 11000) pax = 1000;
	}
	LCD_DisplayStringLine(Line4, format("   Pf:%d      ", pf));
}

uint8_t *format(char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	memset(fmtBuf, 0, 32);
	vsprintf((char *)fmtBuf, fmt, ap);
	return fmtBuf;
}

int fputc(int ch, FILE *f) {
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

int cmp(const void *arg1, const void *arg2) {
	return ((int)(*(float *)arg2*10) - (int)(*(float *)arg1*10));
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
