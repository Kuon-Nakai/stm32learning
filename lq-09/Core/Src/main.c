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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "lcd.h"
#include "i2c.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint8_t h;
	uint8_t m;
	uint8_t s;
}TimeSave;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LEDSet() 											HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, ld1)
#define isLongPress(awt, req, val)		(awt ? false : req ? val[1] + 8000 - val[0] : )

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char 			buf[64] 				= {0};
bool 			mainInterface		= true;
bool 			ticking					= false;
bool 			pause						= false;
bool 			b2AwaitOverflow = false;
bool 			b3AwaitOverflow = false;
bool 			b4AwaitOverflow = false;
bool 			b2ReqOverflow = false;
bool 			b3ReqOverflow = false;
bool 			b4ReqOverflow = false;
TimeSave	saves[5];
TimeSave	currentTime;
uint8_t 	highlight				= 0;
uint8_t		cntSave					= 0;
uint8_t		b2							= 0;
uint8_t		b3							= 0;
uint8_t		b4							= 0;
uint16_t 	b2cap[2]				= {0};
uint16_t 	b4cap[2]				= {0};
bool			ld1							= 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *);

void DrawTime(uint8_t);

char *format(char *, ...);
void render(void);
void *eeRead(uint8_t, uint8_t);
void eeWrite(uint8_t, void *, uint8_t);


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
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	I2CInit();

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim6);
	
	
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	LEDSet();
	
	TimeSave *temp;
	for(uint8_t i = 0; i < 5; i++) {
		temp = ((TimeSave *)eeRead(i << 2, 3));
		saves[i] = *temp;
		free(temp);
	}
	temp = NULL;
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch(b2) {
			case 0:
				
				++b2;
				break;
			case 3:
				break;
		}
		switch(b3) {
			case 0:
				break;
			case 3:
				break;
		}
		switch(b4) {
			case 0:
				memset(b4cap, 0, 2);
				b4AwaitOverflow = false;
				HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
				++b4;
				break;
			case 3:
				break;
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableLSECSS();
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if(pin == GPIO_PIN_0) { //B1 done
		HAL_Delay(32);
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) return;
		if(++cntSave == 5) cntSave = 0;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) { //b4
		switch(b4) {
			case 1:
				b4cap[0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
				TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
				if(
				++b4;
				break;
			case 2:
				b4cap[1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
				if(b4AwaitOverflow || b4cap[1] - b4cap[0] < 30) break;
				TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
				++b4;
				break;
		}
		return;
	}
	if(htim->Instance == TIM3) { //b2 / b3?
		
		return;
	}
}

void render() {
	LCD_DisplayStringLine(Line3, (uint8_t *)format("    No %d", cntSave));
	LCD_DisplayStringLine(Line5, (uint8_t *)format("     %2d : %2d : %2d", currentTime.h, currentTime.m, currentTime.s));
	LCD_DisplayStringLine(Line7, (uint8_t *)(mainInterface ? ticking ? "Running" : pause ? "Pause" : "Standby" : "Setting"));
	DrawTime(0u);
}

void DrawTime(uint8_t filter) {
	
	u32 i = 0;
	u16 refcolumn;//319;
	uint8_t *ptr;
	if(filter == 0 || filter == 1) {
		refcolumn = 239;
		ptr = (uint8_t *)format("%2d", currentTime.h);
		if(highlight == 1) {
			LCD_SetBackColor(White);
			LCD_SetTextColor(Black);
		}
		else {
			LCD_SetBackColor(Black);
			LCD_SetTextColor(White);
		}
		while ((*ptr != 0) && (i < 3))	 //	20
		{
			LCD_DisplayChar(Line5, refcolumn, *ptr);
			refcolumn -= 16;
			ptr++;
			i++;
		}
		free(ptr);
	}
	if(filter == 0 || filter == 2) {
		refcolumn = 159;
		ptr = (uint8_t *)format("%2d", currentTime.m);
		if(highlight == 2) {
			LCD_SetBackColor(White);
			LCD_SetTextColor(Black);
		}
		else {
			LCD_SetBackColor(Black);
			LCD_SetTextColor(White);
		}
		while ((*ptr != 0) && (i < 6))	 //	20
		{
			LCD_DisplayChar(Line5, refcolumn, *ptr);
			refcolumn -= 16;
			ptr++;
			i++;
		}
		free(ptr);
	}
	if(filter == 0 || filter == 3) {
		refcolumn = 79;
		ptr = (uint8_t *)format("%2d", currentTime.s);
		if(highlight == 3) {
			LCD_SetBackColor(White);
			LCD_SetTextColor(Black);
		}
		else {
			LCD_SetBackColor(Black);
			LCD_SetTextColor(White);
		}
		while ((*ptr != 0) && (i < 9))	 //	20
		{
			LCD_DisplayChar(Line5, refcolumn, *ptr);
			refcolumn -= 16;
			ptr++;
			i++;
		}
		free(ptr);
	}
}

void _eeWrite(uint8_t addr, uint8_t data) {
	I2CStart();
	I2CSendByte(0xA0);
	I2CWaitAck();
	I2CSendByte(addr);
	I2CWaitAck();
	I2CSendByte(data);
	I2CWaitAck();
	I2CStop();
}

void eeWrite(uint8_t addr, void *obj, uint8_t size) {
	uint8_t *p = obj;
	while(size--) _eeWrite(addr++, *p++);
}

uint8_t _eeRead(uint8_t addr) {
	uint8_t data;
	I2CStart();
	I2CSendByte(0xA0);
	I2CWaitAck();
	I2CSendByte(addr);
	I2CWaitAck();
	I2CStop();
	I2CStart();
	I2CSendByte(0xA1);
	data = I2CReceiveByte();
	I2CSendNotAck();
	I2CStop();
	return data;
}

void *eeRead(uint8_t addr, uint8_t size) {
	void *r = malloc(size);
	uint8_t *p = r;
	while(size--) *p = _eeRead(addr++);
	return r;
}

char *format(char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	vsprintf(buf, fmt, ap);
	va_end(ap);
	return buf;
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
