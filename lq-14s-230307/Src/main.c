/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file		   : main.c
  * @brief		  : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
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
#define lds(x, s) \
	HAL_GPIO_WritePin(GPIOC, 0x0080 << x, s); \
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)

#define psetf(f_kHz) \
	__HAL_TIM_SetAutoreload((pin7) ? &htim3 : &htim2, 1000 / f_kHz)

#define psetd(dx10) \
	__HAL_TIM_SetCompare((pin7) ? &htim3 : &htim2, TIM_CHANNEL_2, __HAL_TIM_GetAutoreload((pin7) ? &htim3 : &htim2) * dx10 / 10)
	
#define dsp(ln, str) \
	LCD_DisplayStringLine(ln * 24, (uint8_t *)str);
	

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool 		serial	= false;
bool	 	pin7	= false;
uint8_t 	buf[32]	= {0};
uint8_t		rxBuf	= 0;
uint8_t		f1		= 1;
uint8_t		f7		= 1;
uint8_t 	d1		= 1;
uint8_t		d7		= 1;
uint32_t	b4		= 0xFFFFFFFF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void render();
void dspf(uint8_t ln, char *fmt, ...);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
	lds(1, 0);
	lds(2, 0);
	lds(3, 0);
	lds(4, 0);
	lds(5, 0);
	lds(6, 0);
	lds(7, 0);
	lds(8, 0);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_UART_Receive_DMA(&huart1, &rxBuf, 1);

	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	render();
	
	//printf("Init complete!\n");

  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static bool	b4dwn = true;
  while (1)
  {
	  //__ASM("BKPT 0xAB");
	  //dspf(7, "%d", b4dwn);
	  if(b4dwn && !b4) {
		  b4dwn = false;
		  serial = !serial;
	  }
	  else b4dwn |= !~b4;
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
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
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(!serial) {
		printf("KEY CONTROL\n");
		HAL_UART_Receive_DMA(&huart1, &rxBuf, 1);
		return;
	}
	if(rxBuf == '@') {
		pin7 = false;
		render();
	}
	else if(rxBuf == '#') {
		pin7 = true;
		render();
	}
	else printf("ERROR\n");
	HAL_UART_Receive_DMA(&huart1, &rxBuf, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	switch(pin) {
		case 0x0001:
			HAL_Delay(64);
			if(HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_RESET) return;
			if(pin7 && ++f7 == 11) {
				f7 = 1;
				psetf(f7);
			}
			if(!pin7 && ++f1 == 11) {
				f1 = 1;
				psetf(f1);
			}
			psetf(((pin7) ? f7 : f1));
			psetd(((pin7) ? d7 : d1));
			dspf(4, "   F:%d000Hz   ", pin7 ? f7 : f1);
			break;
		case 0x0002:
			HAL_Delay(64);
			if(HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_RESET) return;
			if(pin7 && ++d7 == 10) {
				d7 = 1;
			}
			if(!pin7 && ++d1 == 10) {
				d1 = 1;
			}
			psetd(((pin7) ? d7 : d1));
			dspf(5, "   D:%1d0%%   ", pin7 ? d7 : d1);
			break;
		case 0x0004:
			if(serial) return;
			HAL_Delay(64);
			if(HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_RESET) return;
			pin7 = ! pin7;
			render();
			break;
	}
}

//void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
//	HAL_Delay(32);
//	if(HAL_COMP_GetOutputLevel(&hcomp3) == COMP_OUTPUT_LEVEL_HIGH) return;
//	serial = !serial;
//}

inline void dspf(uint8_t ln, char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	vsprintf((char *)buf, fmt, ap);
	va_end(ap);
	dsp(ln, buf);
}

inline void render() {
	LCD_Clear(Black);
	if(pin7) {
		dsp(2, "      PA7  ");
		dspf(4, "   F:%d000Hz   ", f7);
		dspf(5, "   D:%1d0%%   ", d7);
		return;
	}
	dsp(2, "      PA1  ");
	dspf(4, "   F:%d000Hz   ", f1);
	dspf(5, "   D:%1d0%%   ", d1);
}

int fputc(int ch, FILE *f) {
	HAL_UART_Transmit(&huart1, (const uint8_t *)&ch, 1, 0xFF);
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
