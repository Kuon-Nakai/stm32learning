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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "LCD/lcd.h"
#include "LCD/delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LOG_Y_INCREMENT 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// max: 18. WILL OVERFLOW IF WENT OUT OF BOUND
int lineNum = 0;
char buf[128] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**
 * @brief         : Appends a log message to the LCD screen. Main goal of this project.
 * @param level   : Severity of the event. 0 -> Verbose(white), 1 -> Info(blue), 2 -> Warning(yellow), 3 -> Error(red), 4 -> Fatal(white on red)
 * @param msg     : Message to be displayed. 116 characters max.
*/
void log(uint8_t level, char msg[]);

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  delay_init(72);
  LCD_Init();

  LCD_DisplayOn();
  LCD_Clear(BLACK);
  BACK_COLOR = BLACK;
  
  log(1, "LCD boot complete");
  

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  log(2, "DS1 LED is on. Check status.");

  int i = 0;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //HAL_UART_Transmit(&huart1, (uint8_t *) "Testing testing?\n", 18, 0xffff); //To recv: COM6
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    if(i == 16) log(3, "Milti-line long message handling test");
    if(i++ == 12) log(4, "ERROR: Don't want to work any more");
    log(0, "Meaningless log");
    HAL_Delay(1000);
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
void log(uint8_t level, char msg[]) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  //TODO: Overflow handler
  if(lineNum >= 18) { // 18 lines already present
    for(int8_t l = (strlen(msg) + 11) / 37; l >= 0; l--) {
      for(uint16_t i = 10 + LOG_Y_INCREMENT; i < 289; i += LOG_Y_INCREMENT) {
        LCD_Fill(10, i - LOG_Y_INCREMENT - 4, 240, i, BLACK);
        for(uint16_t j = 10; j < 230; j++) {
          for(uint16_t k = 0; k < 12; k++) {
            LCD_Fast_DrawPoint(j, i + k - LOG_Y_INCREMENT, LCD_ReadPoint(j, i + k));
          }
        }
      }
    }
    lineNum = 17 - (strlen(msg) + 11) / 37;
  }
  memset(buf, 0, 128);
  switch(level) {
    case 0: // verbose
      sprintf(buf, "%7.3f[V] %s", HAL_GetTick() / 1000.0,  msg);
      POINT_COLOR = WHITE;
      break;
    case 1: // debug
      sprintf(buf, "%7.3f[D] %s", HAL_GetTick() / 1000.0, msg);
      POINT_COLOR = BLUE;
      break;
    case 2: // warning
      sprintf(buf, "%7.3f[W] %s", HAL_GetTick() / 1000.0, msg);
      POINT_COLOR = YELLOW;
      break;
    case 3: // error
      sprintf(buf, "%7.3f[E] %s", HAL_GetTick() / 1000.0, msg);
      POINT_COLOR = RED;
      break;
    case 4: // fatal
      sprintf(buf, "%7.3f[F] %s", HAL_GetTick() / 1000.0, msg);
      POINT_COLOR = WHITE;
      BACK_COLOR  = RED;
      break;
  }
  LCD_ShowString(10, 10 + lineNum * LOG_Y_INCREMENT, 220, LOG_Y_INCREMENT, 12, (uint8_t *)buf);
  lineNum += strlen(buf) / 37 + 1; // line capacity = 37
  BACK_COLOR = BLACK;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
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
