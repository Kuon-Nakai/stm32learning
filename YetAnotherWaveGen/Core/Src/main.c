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
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _WaveOptions
{
	bool on;
	uint16_t A;  // CON mode ignore
	uint16_t dA; // CON mode ignore, x0.01
	uint16_t F;  // CON mode ignore
	uint16_t T;  // CON mode ignore
	int16_t O;
	int16_t dO;
	uint16_t D;  // PWM only
	uint16_t HT; // PWM only
} WaveOptions;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GRAPH_UPD_F 40 // Hz
#define Pi 3.141593f
#define DARK_MAG /*0x9814*/ White

#define PwmFunc (opts[0].A * (__HAL_TIM_GetCounter(&htim3) < opts[0].HT) + opts[0].O)
//#define TriFunc	(opts[1].A * ((__HAL_TIM_GetCounter(&htim3) % 2) ? (1 - (float)__HAL_TIM_GetCounter(&htim3) / __HAL_TIM_GET_AUTORELOAD(&htim3)) : \
	((float)__HAL_TIM_GetCounter(&htim3) / __HAL_TIM_GET_AUTORELOAD(&htim3))) + opts[1].O)
#define SawFunc (opts[3].A * (float)__HAL_TIM_GetCounter(&htim3) / __HAL_TIM_GET_AUTORELOAD(&htim3) + opts[3].O)
// Triangle wave alg: uses a saw func but twice the size
#define TriFunc (opts[1].A * fabs((float)__HAL_TIM_GetCounter(&htim3) / __HAL_TIM_GET_AUTORELOAD(&htim3) * 2 - 1) + opts[1].O)
#define SinFunc (opts[2].A * sin((float)__HAL_TIM_GetCounter(&htim3) / __HAL_TIM_GET_AUTORELOAD(&htim3) * Pi * 2) / 2 + opts[2].O + 2048)
#define ConFunc opts[4].O

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define lds(x, s)                           \
	HAL_GPIO_WritePin(GPIOC, 0x0080 << x, s); \
	HAL_GPIO_WritePin(GPIOD, 0x0002, 1);      \
	HAL_GPIO_WritePin(GPIOD, 0x0002, 0)

#define transmit(m) \
	HAL_UART_Transmit(&huart1, (uint8_t *)m, sizeof(m), 0x7FFF)

// Do not use
#define SinSend(x, f) /*Call sinSend(cnt+1) right after the calculated value is used to minimize waiting time*/ \
	sinCalcBuf = ((int)(x * f * .5 - 1) << 16) + 1;                                                               \
	memset(sinCalcRes, 0, 2);                                                                                     \
	waitCalc = true;                                                                                              \
	HAL_CORDIC_Calculate_DMA(&hcordic, &sinCalcBuf, (int32_t *)sinCalcRes, 1, CORDIC_DMA_DIR_IN_OUT)

#define getKeyRange(v)                                                                                           \
	((v < 2500) ? (v < 1500) ? 3 - !(v & 0xFF00) - !(v & 0xFC00) : 5 - !(v & 0xF800) : (v < 3600) ? 6 + (v > 3000) \
																																																: (v < 4000) << 3)

#define lcdSideUpdAll()   \
	for (int i = 4; i;)   \
		lcdUpdSide(i--, false)

#define debug(x)                                            \
	uint8_t *t = format("%6d", x); \
	LCD_DisplayStringLine(Line1, t);                          \
	free(t)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const uint8_t topStr[] = "PWM Wave \0Triangle \0Sine Wave\0Sawtooth \0Constant ";
const uint8_t onOffStr[] = "Off \0 On ";
const uint8_t altsStr[] = " DA \0TIMR\0DA_1";

uint16_t adcBuf[2] = {0}; // use as uint32_t in DMA
uint32_t adc1Buf = 0;
// uint32_t	dac1val		= 0;
// uint32_t	dac3val		= 0;
uint8_t topSel = 0;  // 0 - 4 valid
uint8_t sideSel = 0; // 0 - 7/8 valid
uint16_t graphY = 310;

WaveOptions opts[5];
WaveOptions *currentOpts = opts;

int32_t sinCalcBuf = 0;
volatile int16_t sinCalcRes[2] = {0};

uint16_t adcLoopbackLastVal = 220;
uint16_t dacGraphLastVal = 220;

bool waitCalc = false;

uint8_t rxBuf[128] = {0};
bool rxIdle = false;
uint32_t rxLen = 0;

//Analog input vars

bool aInput = false;

//freq max ~32000Hz, min ~700Hz

uint32_t pwmData[5] = {0};
uint32_t f1Data[5] = {0};
uint32_t f2Data[5] = {0};
uint32_t f3Data[5] = {0};

uint32_t npwm = 0;
uint32_t nf1  = 0;
uint32_t nf2  = 0;
uint32_t nf3  = 0;

uint32_t val;

volatile uint32_t pwm1;
volatile uint32_t pwm2;
volatile uint32_t f1;
volatile uint32_t f2;
volatile uint32_t f3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t *format(char *, ...);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *);
void HAL_CORDIC_CalculateCpltCallback(CORDIC_HandleTypeDef *);

void emuSetTim(const uint16_t);
void lcdSideShow(uint8_t, const uint8_t *, bool, bool);
void lcdUpdSide(uint8_t, bool);
void drawPt(uint16_t, uint16_t, uint16_t);
void graphReset(void);
void AAProc(uint16_t, uint16_t);
uint16_t filter(uint32_t *);

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
  MX_DAC3_Init();
  MX_OPAMP1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_COMP3_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

	// options init
	for (int i = 5; --i;)
	{
		currentOpts->on = false;      // str+0, +5
		currentOpts->A = 4095u;        // No display
		currentOpts->dA = 330u;
		currentOpts->D = 0u;
		currentOpts->F = 1u;
		currentOpts->HT = 0u; // No display
		// currentOpts->T	= 1;		//No display
		currentOpts->O = 0; // No display
		currentOpts->dO = 0;
		++currentOpts;
	}
	currentOpts = opts;

	// transmit("Initializing...");
	// uint8_t *txBuf = (uint8_t *)"Init......";
	// HAL_UART_Transmit_DMA(&huart1, txBuf, 11);
	// __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	// HAL_UART_Receive_DMA(&huart1, rxBuf, 128);

	CORDIC_ConfigTypeDef cordicCfg;
	cordicCfg.Function = CORDIC_FUNCTION_SINE;
	cordicCfg.InSize = CORDIC_INSIZE_16BITS;
	cordicCfg.OutSize = CORDIC_OUTSIZE_16BITS;
	cordicCfg.Scale = CORDIC_SCALE_0;
	cordicCfg.NbRead = CORDIC_NBREAD_2;
	cordicCfg.NbWrite = CORDIC_NBWRITE_2;
	cordicCfg.Precision = CORDIC_PRECISION_10CYCLES;
	HAL_CORDIC_Configure(&hcordic, &cordicCfg);

	HAL_OPAMP_SelfCalibrate(&hopamp1);
	HAL_OPAMP_Start(&hopamp1);

	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	LCD_DrawRect(30, 320, 200, 250);
	LCD_SetTextColor(Red);
	LCD_DrawLine(58, 310, 230, Horizontal);  // 0.0V
	LCD_DrawLine(220, 310, 230, Horizontal); // 3.3V
	LCD_SetTextColor(White);
	LCD_DrawLine(73, 310, 230, Horizontal);  // 1.0V
	LCD_DrawLine(122, 310, 230, Horizontal); // 2.0V
	LCD_DrawLine(171, 310, 230, Horizontal); // 3.0V
	LCD_DisplayStringLine(Line0, (uint8_t *)(topStr + topSel * 10));
	lcdUpdSide(1, false);
	lcdUpdSide(2, false);
	lcdUpdSide(3, false);
	sideSel = 0;
	lcdSideShow(Line1, (uint8_t *)"Ampl", false, false);
	lcdSideShow(Line3, (uint8_t *)"Freq", false, false);
	lcdSideShow(Line5, (uint8_t *)"Ofst", false, false);
	lcdSideShow(Line7, (uint8_t *)"Duty\0    " + (topSel != 0), false, false);
	lcdSideShow(Line0, (uint8_t *)"Off ", true, false);

	HAL_COMP_Start(&hcomp3);

	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcBuf, 2);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, &adc1Buf, 1);

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
	//	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &dac1val, 1, DAC_ALIGN_12B_R);
	//	HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_1, &dac3val, 1, DAC_ALIGN_12B_R);

	__HAL_TIM_SetAutoreload(&htim7, 1000000 / GRAPH_UPD_F);

	HAL_TIM_Base_Start_IT(&htim4); // ADC trigger
	HAL_TIM_Base_Start(&htim3);    // DAC clock
	HAL_TIM_Base_Start_IT(&htim6); //DAC updater
	HAL_TIM_Base_Start_IT(&htim2); // new DAC updater
	HAL_TIM_Base_Start_IT(&htim7); //Graphic updater

	//Analog input
	HAL_COMP_Start(&hcomp1);
	HAL_COMP_Start(&hcomp2);
	HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, &pwm1, 1);
	HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2, &pwm2, 1);
	HAL_TIM_IC_Start_DMA(&htim4, TIM_CHANNEL_1, &f1, 1);
	HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1, &f2, 1);
	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, &f3, 1);
	
	HAL_TIM_Base_Start_IT(&htim16);

	register int kr;
	register bool waitLift = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		kr = getKeyRange(adcBuf[0]);
		//		uint8_t *t = format("%4d %4d", kr, adcBuf[0]);
		//		LCD_DisplayStringLine(Line8, t);
		//		free(t);
		
		if (!kr)
		{
			waitLift = false;
			continue;
		}
		if (!waitLift && !aInput && kr == getKeyRange(adcBuf[1]))
		{
			switch (getKeyRange(adcBuf[0]))
			{
			case 1: //-1000
				if (sideSel == 1)
				{
					currentOpts->dA = 0;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F <= 1001)
						currentOpts->F = 1;
					else
						currentOpts->F -= 1000;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					currentOpts->dO = 0;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 4)
				{
					currentOpts->D = 0;
					break;
				}
				break;
			case 2: //-100
				if (sideSel == 1)
				{
					if (currentOpts->dA < 101)
						currentOpts->dA = 0;
					else
						currentOpts->dA -= 100;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F < 102)
						currentOpts->F = 1;
					else
						currentOpts->F -= 100;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					if (currentOpts->dO < 101)
						currentOpts->O = 0;
					else
						currentOpts->dO -= 100;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
				}
				if (sideSel == 4)
				{
					currentOpts->D = 0;
					break;
				}
				break;
			case 3: //-10
				if (sideSel == 1)
				{
					if (currentOpts->dA < 11)
						currentOpts->dA = 0;
					else
						currentOpts->dA -= 10;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F < 12)
						currentOpts->F = 1;
					else
						currentOpts->F -= 10;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					if (currentOpts->dO < 11)
						currentOpts->O = 0;
					else
						currentOpts->dO -= 10;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
				}
				if (sideSel == 4)
				{
					if (currentOpts->D < 11)
						currentOpts->D = 0;
					else
						currentOpts->D -= 10;
					break;
				}
				break;
			case 4: //-1
				if (sideSel == 1)
				{
					if (currentOpts->dA < 1)
						break;
					currentOpts->dA--;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F < 2)
						break;
					currentOpts->F--;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					if (currentOpts->dO < 1)
						break;
					currentOpts->dO--;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
				}
				if (sideSel == 4)
				{
					if (currentOpts->D < 1)
						break;
					currentOpts->D--;
					break;
				}
				break;
			case 5: //+1000
				if (sideSel == 1)
				{
					currentOpts->dA = 330;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F > 8998)
						currentOpts->F = 9999;
					else
						currentOpts->F += 1000;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					currentOpts->dO = 330 - currentOpts->dA;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 4)
				{
					currentOpts->D = 100;
					break;
				}
				break;
			case 6: //+100
				if (sideSel == 1)
				{
					if (currentOpts->dO + currentOpts->dA > 229)
						currentOpts->A = 330 - currentOpts->dA;
					else
						currentOpts->dA += 100;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F > 9898)
						currentOpts->F = 9999;
					else
						currentOpts->F += 100;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					if (currentOpts->dO + currentOpts->dA > 229)
						currentOpts->O = 330 - currentOpts->dA;
					else
						currentOpts->dO += 100;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 4)
				{
					currentOpts->D = 100;
					break;
				}
				break;
			case 7: //+10
				if (sideSel == 1)
				{
					if (currentOpts->dO + currentOpts->dA > 319)
						currentOpts->dA = 330 - currentOpts->dA;
					else
						currentOpts->dA += 10;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F > 9988)
						currentOpts->F = 9999;
					else
						currentOpts->F += 10;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					if (currentOpts->dO + currentOpts->dA > 319)
						currentOpts->O = 330 - currentOpts->dA;
					else
						currentOpts->dO += 10;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 4)
				{
					if (currentOpts->D > 89)
						currentOpts->D = 100;
					else
						currentOpts->D += 10;
					break;
				}
				break;
			case 8: //+1
				if (sideSel == 1)
				{
					if (currentOpts->dO + currentOpts->dA > 329)
						break;
					currentOpts->dA++;
					currentOpts->A = currentOpts->dA / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 2)
				{
					if (currentOpts->F > 9988)
						break;
					currentOpts->F++;
					__HAL_TIM_SetAutoreload(&htim3, 10000 / currentOpts->F);
					break;
				}
				if (sideSel == 3)
				{
					if (currentOpts->dO + currentOpts->dA > 319)
						break;
					currentOpts->dO++;
					currentOpts->O = currentOpts->dO / 330.0 * 0x0FFF;
					break;
				}
				if (sideSel == 4)
				{
					if (currentOpts->D > 99)
						break;
					currentOpts->D++;
					break;
				}
				break;
			}
			if (topSel == 0)
				currentOpts->HT = 100 * currentOpts->D / (float)opts[0].F;
			lcdUpdSide(sideSel, true);
			waitLift = true;
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 80;
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

uint16_t filter(uint32_t *arr) {
	int max = 0, min = 1, sum = 0;
	int *cpy = (int *)malloc(5 * sizeof(int));
	memcpy(cpy, arr, 5 * sizeof(int));
	for(int i = 4; i >= 0; i--) {
		if(*(cpy + i) > *(cpy + max)) max = i;
		if(*(cpy + i) < *(cpy + min)) min = i;
	}
	for(int i = 4; i >= 0; i--) {
		if(i == max || i == min) continue;
		sum += *(cpy + i);
	}
	free(cpy);
	return (sum / 3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM16) {
		pwmData[npwm++] = pwm1 * 100 / pwm2;
		if(npwm == 5) npwm = 0;
		
//		f1Data[nf1++] = 1000000 / f1;
//		if(nf1 == 5) nf1 = 0;
		
		f2Data[nf2++] = f2; // 1250 - 100
		if(nf2 == 5) nf2 = 0;
	
		f3Data[nf3++] = f3; 
		if(nf3 == 5) nf3 = 0;
		
		if(aInput) 
		{
			int x = filter(f3Data) - 100;
			if (x < 0) currentOpts->A = 0;
			else if(x > 1250) currentOpts->A = 4095;
			else currentOpts->A = x * 4095/ 1250;
			currentOpts->dA = currentOpts->A * 330 / 0x0FFF;
			x = filter(f2Data);
			if(currentOpts == &opts[2]) {
				x -= 600;
				//debug(x);
				int ot  = (4095 - currentOpts->A) * x / 1000;
				int max = 2048 - (currentOpts->A / 2);
				int min = -2048 + (currentOpts->A / 2);
				currentOpts->O  = (ot > max) ? max : (ot < min) ? min : ot; 
				//debug(currentOpts->O);
			}
			else {
				x -= 50;
				int ot  = x * 4095 / 1250;
				int max = 4095 - currentOpts-> A;
				currentOpts->O = ot > max ? max : ot < 0 ? 0 : ot;
			}
			currentOpts->dO = currentOpts->O * 165 / 2048;
			currentOpts->D = filter(pwmData);
			currentOpts->HT = 100 * currentOpts->D / (float)opts[0].F;
			lcdSideUpdAll();
		}
		return;
	}
	if (htim->Instance == TIM6)
	{
		HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcBuf, 2);
		HAL_ADC_Start_DMA(&hadc1, &adc1Buf, 1);
		return;
	}
	if (htim->Instance == TIM2)
	{ // DAC update
		if (opts[0].on)
			val = PwmFunc;
		else if (opts[1].on)
			val = TriFunc;
		else if (opts[2].on)
			val = SinFunc;
		else if (opts[3].on)
			val = SawFunc;
		else if (opts[4].on)
			val = ConFunc;
		else
			val = 0;

		if (HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val) != HAL_OK)
			LCD_DisplayStringLine(Line1, (uint8_t *)"ERR-DAC3");
		if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val) != HAL_OK)
			LCD_DisplayStringLine(Line2, (uint8_t *)"ERR-DAC1");
		//		if(HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) LCD_DisplayStringLine(Line2, (uint8_t *)"ERR-DAC1-start");
		//		if(HAL_DAC_Start(&hdac3, DAC_CHANNEL_1) != HAL_OK) LCD_DisplayStringLine(Line1, (uint8_t *)"ERR-DAC3-start");
		//		dac1val = val;
		//		dac3val = val;

		//		debug(val);

		return;
	}
	if (htim->Instance == TIM7)
	{ // Graphics
		// TODO: DAC graph
		if (opts[0].on)
			val = PwmFunc;
		else if (opts[1].on)
			val = TriFunc;
		else if (opts[2].on)
			val = SinFunc;
		else if (opts[3].on)
			val = SawFunc;
		else if (opts[4].on)
			val = ConFunc;

		if (val > 4096)
			val = 4096;

		register uint16_t v = 220 - val * 0.03947754, vl = dacGraphLastVal;
		if (graphY == 310)
		{
			drawPt(v, 310, Magenta);
		}
		else
		{
			if (v > vl)
			{
				while (v >= vl)
				{
					drawPt(vl++, graphY, Magenta);
					//					AAProc(vl, graphY + 1);
					//					AAProc(vl, graphY - 1);
					//					AAProc(vl - 1, graphY);
					//					AAProc(vl + 1, graphY);
				}
			}
			else
			{
				while (v <= vl)
				{
					drawPt(vl--, graphY, Magenta);
					//					AAProc(vl, graphY + 1);
					//					AAProc(vl, graphY - 1);
					//					AAProc(vl + 1, graphY);
					//					AAProc(vl - 1, graphY);
				}
			}
		}
		dacGraphLastVal = v;

		// ADC1 loopback graph - for testing
		//		v = 220 - adc1Buf * 0.03947754, vl = adcLoopbackLastVal;
		//		if(graphY == 310) {
		//			drawPt((uint16_t)v, 310, Yellow);
		//			graphY = 309;
		//		}
		//		else {
		//			if(v > vl) {
		//				while(v >= vl++) drawPt(vl, graphY, Yellow);
		//			}
		//			else {
		//				while(v <= vl--) drawPt(vl, graphY, Yellow);
		//			}
		// drawPt((uint16_t)(220 - adc1Buf * 3.3 / 4096 * 49), graphY--, Yellow);
		if (--graphY == 80)
			graphReset();
		//		}
		adcLoopbackLastVal = v;
	}
}
void AAProc(uint16_t x, uint16_t y)
{
	LCD_SetCursor(x, y);
	if (LCD_ReadRAM() == Magenta || LCD_ReadRAM() == DARK_MAG)
		return;
	int cnt = 2;
	LCD_SetCursor(x, y - 1);
	if (LCD_ReadRAM() == Magenta)
		--cnt;
	LCD_SetCursor(x, y + 1);
	if (LCD_ReadRAM() == Magenta && --cnt == 0)
		goto paint;
	LCD_SetCursor(x - 1, y);
	if (LCD_ReadRAM() == Magenta && --cnt == 0)
		goto paint;
	LCD_SetCursor(x + 1, y);
	if (LCD_ReadRAM() == Magenta && --cnt == 0)
		goto paint;
	return;
paint:
	drawPt(x, y, DARK_MAG);
}
void drawPt(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_SetCursor(x, y);
	LCD_WriteRAM_Prepare();
	LCD_WriteRAM(color);
}
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	switch (pin)
	{
	case GPIO_PIN_0: // b1 : switch waves
		HAL_Delay(128);
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) != GPIO_PIN_SET)
			return;
		if (++topSel == 5)
		{
			topSel = 0;
			currentOpts = opts;
		}
		else
			++currentOpts;
		LCD_DisplayStringLine(Line0, (uint8_t *)(topStr + topSel * 10));
		lcdUpdSide(1, false);
		lcdUpdSide(2, false);
		lcdUpdSide(3, false);
		sideSel = 0;
		lcdSideShow(Line1, (uint8_t *)"Ampl", false, false);
		lcdSideShow(Line3, (uint8_t *)"Freq", false, false);
		lcdSideShow(Line5, (uint8_t *)"Ofst", false, false);
		lcdSideShow(Line7, (uint8_t *)"Duty\0    " + 5 *(topSel != 0), false, false);
		lcdUpdSide(4, false);
		// if (topSel == 2)
		// { // sine, start early calc
		// 	SinSend(0, currentOpts->F);
		// }
		break;
	case GPIO_PIN_1: // b2 : switch options
		HAL_Delay(128);
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) != GPIO_PIN_SET)
			return;
		lcdUpdSide(sideSel, false);
		if (++sideSel == 4 + (!topSel))
			sideSel = 0;
		lcdUpdSide(sideSel, true);
		break;
	case GPIO_PIN_2: // b3 : toggle
		HAL_Delay(32);
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) != GPIO_PIN_SET)
			return;
		currentOpts->on = !currentOpts->on;
		/*if(!currentOpts->altMode)*/ emuSetTim(currentOpts->F);
		// lcdUpdSide(0, true); //FIXME
		if(currentOpts->on) {
			lcdSideShow(Line0, onOffStr + 5 * currentOpts->on, false, true);
		}
		lcdSideShow(Line0, onOffStr + 5 * currentOpts->on, sideSel == 0, false);
		break;
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{ // b4 : Redraw graphs
	if (hcomp->Instance != COMP3) return;
	HAL_Delay(32);
	// if (HAL_COMP_GetOutputLevel(&hcomp3) != COMP_OUTPUT_LEVEL_HIGH)
	// 	return;
	// HAL_TIM_Base_Stop_IT(&htim7);
	// graphReset();
	// __HAL_TIM_SET_COUNTER(&htim7, 0xFFFFu);
	// HAL_TIM_Base_Start_IT(&htim7);

	//new b4: Enable/ Disable analog input
	aInput = !aInput;
}

void graphReset()
{
	for (int i = 10; --i;)
		LCD_DisplayStringLine(i * 24, (uint8_t *)"                ");
	LCD_DrawRect(30, 320, 200, 250);
	LCD_SetTextColor(Red);
	LCD_DrawLine(58, 310, 230, Horizontal);  // 0.0V
	LCD_DrawLine(220, 310, 230, Horizontal); // 3.3V
	LCD_SetTextColor(White);
	LCD_DrawLine(73, 310, 230, Horizontal);  // 1.0V
	LCD_DrawLine(122, 310, 230, Horizontal); // 2.0V
	LCD_DrawLine(171, 310, 230, Horizontal); // 3.0V
	graphY = 310;
}
void lcdUpdSide(uint8_t ln, bool hl)
{
	switch (ln)
	{
	case 0:
		lcdSideShow(Line0, onOffStr + 5 * currentOpts->on, hl, currentOpts->on);
		break;
	case 1: {
		uint8_t *t = format("%4.2f", currentOpts->dA * .01);
		lcdSideShow(Line2, t, hl, false);
		free(t);
		break;
		}
	case 2: {
		uint8_t *t = format("%4d", currentOpts->F);
		lcdSideShow(Line4, t, hl, false);
		free(t);
		break;
		}
	case 3: {
		uint8_t *t = format("%4.2f", currentOpts->dO * .01);
		lcdSideShow(Line6, t, hl, false);
		free(t);
		break;
		}
	case 4:
		if (topSel)
			lcdSideShow(Line8, (uint8_t *)"    ", hl, false);
		else
		{
			uint8_t *t = format("%4d", currentOpts->D);
			lcdSideShow(Line8, t, hl, false);
			free(t);
		}
		break;
	}
	//	LCD_SetBackColor(Black);
	//	LCD_SetTextColor(White);
}

void lcdSideShow(uint8_t Line, const uint8_t *ptr, bool highlight, bool green)
{
	u32 i = 16;
	u16 refcolumn = 63; // 319;
	if (highlight)
	{
		LCD_SetBackColor(Blue2);
		LCD_SetTextColor(Black);
	}
	else if (green)
	{
		LCD_SetBackColor(Green);
		LCD_SetTextColor(Black);
	}
	while ((*ptr != 0) && (i < 20)) //	20
	{
		LCD_DisplayChar(Line, refcolumn, *ptr);
		refcolumn -= 16;
		ptr++;
		i++;
	}
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
}

inline void emuSetTim(const uint16_t f)
{
	HAL_TIM_Base_Stop_IT(&htim3);
	__HAL_TIM_SetAutoreload(&htim3, (uint16_t)(10000 / f));
	__HAL_TIM_SetCounter(&htim3, 65535);
	HAL_TIM_Base_Start_IT(&htim3);
}

void HAL_CORDIC_CalculateCpltCallback(CORDIC_HandleTypeDef *hcordic)
{
	if (hcordic->State == HAL_CORDIC_STATE_READY)
		waitCalc = false;
}

uint8_t *format(char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	uint8_t *buffer = (uint8_t *)malloc(16);
	if (buffer == NULL)
		return NULL;
	memset(buffer, 0, 16);
	vsprintf((char *)buffer, fmt, ap);
	return buffer;
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
