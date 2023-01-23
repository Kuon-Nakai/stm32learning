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
#include <time.h>
#include <string.h>
#include "LCD/lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t type;
	uint8_t code[5];
	time_t  time;
}Car;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PwmEnable() 	{__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 399);\
											HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);}

#define PwmDisable()	{__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 511);\
											HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);}

#define cnDspl()			{sprintf(buf, "%d  ", cnbr);\
											LCD_ShowString(100, 120,  50, 20, 16, (uint8_t *)buf);\
											memset(buf, 0, 32);}

#define vnDspl()			{sprintf(buf, "%d  ", vnbr);\
											LCD_ShowString(100, 140,  50, 20, 16, (uint8_t *)buf);\
											memset(buf, 0, 32);}

#define idDspl()			{sprintf(buf, "%d  ", idle);\
											LCD_ShowString(100, 160,  50, 20, 16, (uint8_t *)buf);\
											memset(buf, 0, 32);}

#define cpDspl()			{sprintf(buf, "%.2f  ", cPrice);\
											LCD_ShowString(100, 120,  50, 20, 16, (uint8_t *)buf);\
											memset(buf, 0, 32);}

#define vpDspl()			{sprintf(buf, "%.2f  ", vPrice);\
											LCD_ShowString(100, 140,  50, 20, 16, (uint8_t *)buf);\
											memset(buf, 0, 32);}

#define rerender() 		{LCD_Clear(BLACK);\
											LCD_ShowString( 50, 120,  50, 20, 16, (uint8_t *)"CNBR:");\
											LCD_ShowString( 50, 140,  50, 20, 16, (uint8_t *)"VNBR:");\
											if(paraInterface) {\
												LCD_ShowString(100, 100, 100, 20, 16, (uint8_t *)"Para");\
												vpDspl();\
												cpDspl();\
											}\
											else {\
												LCD_ShowString(100, 100, 100, 20, 16, (uint8_t *)"Data");\
												LCD_ShowString( 50, 160,  50, 20, 16, (uint8_t *)"IDLE:");\
												cnDspl();\
												vnDspl();\
												idDspl();\
											}\
											if(idle > 0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);\
											else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);}
											
#define err()					HAL_UART_Transmit(&huart1, (uint8_t *)"Error", 6, 0xFFFF);\
											memset(buf, 0, 32);
	
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char paraInterface = 0;
char cnbr = 2, vnbr = 4, idle = 2;
char pwnOn = 1;
float cPrice = 3.5, vPrice = 2.0;
static char buf[32] = {0};
uint8_t rx[23] = {0};
uint8_t *rxType = rx;
uint8_t *rxCode = rx + 5;
uint8_t *rxTime = rx + 10;
Car park[8];
int occupied = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern void delay_init(uint8_t);
//extern void *memset(void *, int, size_t);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
time_t parseTime(void);

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	PwmDisable();
	
	HAL_UART_Receive_IT(&huart1, rx, 22);
	
	delay_init(72);
	LCD_Init();
	BACK_COLOR = BLACK;
	POINT_COLOR= WHITE;
	rerender();

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
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch(GPIO_Pin) {
		case GPIO_PIN_0: // interface
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET) return;
			paraInterface = !paraInterface;
			rerender();
			break;
		case GPIO_PIN_15:// +
			if(!paraInterface) return;
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != GPIO_PIN_RESET) return;
			vPrice += .5;
			cPrice += .5;
			vpDspl();
			cpDspl();
			break;
		case GPIO_PIN_5: // -
			if(!paraInterface) return;
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) != GPIO_PIN_RESET) return;
			vPrice -= .5;
			cPrice -= .5;
			vpDspl();
			cpDspl();
			break;
		case GPIO_PIN_1: // ctrl
			HAL_Delay(100);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) != GPIO_PIN_RESET) return;
			pwnOn = !pwnOn;
			if(pwnOn) {PwmEnable();}
			else PwmDisable();
			break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(rx[4] != ':' || rx[9] != ':') { 
		err();
		goto ret;
	}
	rx[ 4] = 0;
	rx[ 9] = 0;
	rx[22] = 0;
	//uint8_t code[5] = {0};
	//strcpy((char *)code, (char *)rxCode);
	for(int i = 0; i < occupied; i++) {
		if(strcmp((char *)rxCode, (char *)&park[i].code) == 0) { //exiting
			time_t current = parseTime();
			if(current == (time_t)-1 || current <= park[i].time) {
				err();
				goto ret;
			}
			int hrs = difftime(current, park[i].time) / 3600 + 1;
			if(hrs == 0) {
				err();
				goto ret;
			}
			sprintf(buf, "%s:%s:%d:%.2f", *rxType == 'C' ? "CNBT" : "VNBT", rxCode, hrs, hrs * (*rxType == 'C' ? cPrice : vPrice));
			HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf) + 1, 0xFFFF);
			memset(buf, 0, 32);
			--occupied;
			++idle;
			if(*rxType == 'C') --cnbr;
			else --vnbr;
			rerender();
			for(int j = i; j < occupied; j++) park[j] = park[j+1];
			goto ret;
		}
	}
	if(idle < 1 || (rx[0] == 'C' && cnbr < 1) || (rx[0] == 'V' && vnbr < 1)) { // park full
		err();
		goto ret;
	}
	Car nextCar;
	nextCar.type = rx[0];
	strcpy((char *)nextCar.code, (char *)rxCode);
	nextCar.time = parseTime();
	park[occupied++] = nextCar; // add new car to park
	--idle;
	if(*rxType == 'C') ++cnbr;
	else ++vnbr;
	rerender();
	ret:;
	HAL_UART_Receive_IT(&huart1, rx, 22);
}

inline time_t parseTime() {
	struct tm time;
	time.tm_isdst = 0;
	int temp = 0;
	uint8_t *ptr = rxTime;
	for(char i = 0; i < 2; i++) temp = (temp << 3) + (temp << 1) + *ptr++ - '0';
	//time.tm_year = temp + 100;
	time.tm_year = temp;
	temp = 0;
	for(char i = 0; i < 2; i++) temp = (temp << 3) + (temp << 1) + *ptr++ - '0';
	time.tm_mon = temp;
	temp = 0;
	for(char i = 0; i < 2; i++) temp = (temp << 3) + (temp << 1) + *ptr++ - '0';
	time.tm_mday = temp;
	temp = 0;
	for(char i = 0; i < 2; i++) temp = (temp << 3) + (temp << 1) + *ptr++ - '0';
	time.tm_hour = temp;
	temp = 0;
	for(char i = 0; i < 2; i++) temp = (temp << 3) + (temp << 1) + *ptr++ - '0';
	time.tm_min = temp;
	temp = 0;
	for(char i = 0; i < 2; i++) temp = (temp << 3) + (temp << 1) + *ptr++ - '0';
	time.tm_sec = temp;
	return mktime(&time);
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
