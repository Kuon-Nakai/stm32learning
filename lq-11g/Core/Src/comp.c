/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comp.c
  * @brief   This file provides code for the configuration
  *          of the COMP instances.
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
#include "comp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

COMP_HandleTypeDef hcomp3;

/* COMP3 init function */
void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp3.Init.InputMinus = COMP_INPUT_MINUS_VREFINT;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}

void HAL_COMP_MspInit(COMP_HandleTypeDef* compHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(compHandle->Instance==COMP3)
  {
  /* USER CODE BEGIN COMP3_MspInit 0 */

  /* USER CODE END COMP3_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP3 GPIO Configuration
    PA0     ------> COMP3_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* COMP3 interrupt Init */
    HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
  /* USER CODE BEGIN COMP3_MspInit 1 */

  /* USER CODE END COMP3_MspInit 1 */
  }
}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef* compHandle)
{

  if(compHandle->Instance==COMP3)
  {
  /* USER CODE BEGIN COMP3_MspDeInit 0 */

  /* USER CODE END COMP3_MspDeInit 0 */

    /**COMP3 GPIO Configuration
    PA0     ------> COMP3_INP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* COMP3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn);
  /* USER CODE BEGIN COMP3_MspDeInit 1 */

  /* USER CODE END COMP3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
