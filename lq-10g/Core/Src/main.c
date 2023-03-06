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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "lcd.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define lcd(x, s) LCD_DisplayStringLine(24 * x, (uint8_t *)s)
#define a1u() lcd(3, format("    A01: %.2f   ", adcIn[0] * 3.3f / 4096))
#define a2u() lcd(4, format("    A02: %.2f   ", adcIn[1] * 3.3f / 4096))
#define p2u() lcd(5, format("    PWM2:%d%%   ", pwmH[0] * 100 / pwmT[0]))
#define cgu() lcd(7, format("    N:   %d     ", changes))
#define lmu() lcd(3, format("    T: %d    ", lim));
#define chu() lcd(4, format("    X: %d    ", (int)adcCh2 + 1));
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char	  	fmtBuf[32] 	= {0};
uint16_t 	adcIn[2]	= {0};
uint32_t	pwmT[10]	= {0};
uint32_t	pwmH[10]	= {0};
uint8_t		rxBuf[32]	= {0};
uint32_t	b1In		= 0xFFFFFFFF;
uint32_t	b2In		= 0xFFFFFFFF;
uint32_t	b3In		= 0xFFFFFFFF;
uint32_t	b4In		= 0xFFFFFFFF;
bool		b1a			= true;
bool		b2a			= true;
bool		b3a			= true;
bool		b4a			= true;

bool		para		= false;
bool		adcCh2		= false;
bool		selT		= true;

uint32_t	changes		= 0;
uint32_t	lim			= 30;

bool 		adcCh2t		= false;
uint32_t	limt		= 30;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void eeAdd(void);
uint16_t eeRead(void);
void num(uint8_t, uint8_t, uint8_t);
void render(void);
char *format(const char *, ...);
uint32_t filter(uint32_t *);

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
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  
	num(0x00, 0x00, 0x00);
  
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcIn, 2);

//	HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, pwmH, 1);
//	HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_2, pwmT, 1); //pwm in
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); //pwm in
	
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	render();

	HAL_TIM_Base_Start_IT(&htim6); //2s
	HAL_TIM_Base_Start_IT(&htim7); //btn in
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuf, 32);

	
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
	//TODO
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		pwmH[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
		pwmT[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM7) { //50Hz
		
		register uint32_t temp = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		lcd(0, format("%d", temp));
		//b1In = (b1In << 1) ^ temp; // 0 = down
		//b2In = b2In << 1 & HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
		//b3In = b3In << 1 & HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
		//b4In = b4In << 1 & HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		b1In = b1In << 1 | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		b2In = b2In << 1 | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
		b3In = b3In << 1 | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
		b4In = b4In << 1 | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		//if(!(b1In & 0x03) && b1a) {
		if(b1a && !(b1In & 0x00000003)) {
			b1a = false;
			if(para && (adcCh2t != adcCh2 || limt != lim) ) {
				eeAdd();
				adcCh2 = adcCh2t;
				lim = limt;
			}
			para = !para;
			render();
		}
		//else b1a = (b1In & 0x03); // released for 20ms+
		else b1a = b1In & 0x00000003;
		
		if(!para) {
			a1u();
			a2u();
			p2u();
			HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcIn, 2);
			return;
		}
		
		if(b2a && !(b2In & 0x00000003)) {
			b2a = false;
			selT = !selT;
		}
		else b2a =b2In & 0x03;
		
		if(!b3In || (b3a && !(b3In & 0x000000FF))) { //0.8s+
			b3a = false;
			if(selT) {
				if(++limt == 41) limt = 40;
				lmu();
			}
			else adcCh2t = !adcCh2t;
			chu();
		}
		else b3a = b3In & 0x03; 
		
		if(!b4In || (b4a && !(b4In & 0x000000FF))) {
			b4a = false;
			if(selT) {
				if(--limt == 19) limt = 20;
				lmu();
			}
			else adcCh2t = !adcCh2t;
			chu();
		}
		else b4a = b4In & 0x03;
		
		
	}
}

inline void render() {
	LCD_Clear(Black);
	if(para) {
		lcd(1, "      Para  ");
		lmu();
		chu();
		return;
	}
	lcd(1, "       Main  ");
	a1u();
	a2u();
	p2u();
	//TODO
	cgu();
}

inline void eeAdd() {
	I2CStart();
	I2CSendByte(0xA0);
	I2CWaitAck();
	I2CSendByte(0x10);
	I2CWaitAck();
	I2CSendByte(++changes & 0x00FF);
	I2CWaitAck();
	I2CSendByte(changes >> 8);
	I2CStop();
}

inline uint16_t eeRead() {
	I2CStart();
	I2CSendByte(0xA0);
	I2CWaitAck();
	I2CSendByte(0x10);
	I2CWaitAck();
	I2CStop();
	I2CStart();
	I2CSendByte(0xA1);
	I2CWaitAck();
	changes = 0;
	changes = I2CReceiveByte();
	I2CSendAck();
	changes |= I2CReceiveByte() << 8;
	I2CSendNotAck();
	I2CStop();
	return changes;
}

void num(uint8_t n1, uint8_t n2, uint8_t n3) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	bool *bit = (bool *)&n3;
	for(int i = 0; i < 8; i++, bit++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, *bit);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
	bit = (bool *)&n2;
	for(int i = 0; i < 8; i++, bit++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, *bit);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
	bit = (bool *)&n1;
	for(int i = 0; i < 8; i++, bit++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, *bit);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}	

char *format(const char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	vsprintf(fmtBuf, fmt, ap);
	return fmtBuf;
}

uint32_t filter(uint32_t *data) {
	uint32_t *d = (uint32_t *)malloc(320);
	uint32_t *max = d, *min = d + 1, *p = d, sum = 0;
	memcpy(d, data, 320);
	for(int i = 0; i < 10; i++, p++) {
		if(*p > *max) max = p;
		if(*p < *min) min = p;
	}
	p = d;
	for(int i = 0; i < 10; i++, p++) {
		if(p == max || p == min) continue;
		sum += *p;
	}
	return sum >> 3;
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
