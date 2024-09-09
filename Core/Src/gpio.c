/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RTC_POWER_Pin|SG8_3_Pin|SG5_3_Pin|SG5_2_Pin
                          |MEM_WP_Pin|RST_USR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_INT_EN_GPIO_Port, RS485_INT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SG8_2_Pin|SG8_1_Pin|SG7_3_Pin|SG7_2_Pin
                          |SG7_1_Pin|SG6_3_Pin|SG6_2_Pin|SG6_1_Pin
                          |EN_485_Pin|IND_ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SG5_1_Pin|SG2_3_Pin|SG2_2_Pin|SG2_1_Pin
                          |SG1_3_Pin|SG1_2_Pin|SG1_1_Pin|SG11_3_Pin
                          |SG11_2_Pin|SG11_1_Pin|SG10_3_Pin|SG10_2_Pin
                          |SG10_1_Pin|SG9_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SG4_3_Pin|SG4_2_Pin|SG4_1_Pin|SG3_3_Pin
                          |SG3_2_Pin|SG3_1_Pin|SG9_2_Pin|SG9_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CPU_PROCESS_Pin|IND_GPS_Pin|IND_FLASHER_Pin|IND_HOLD_Pin
                          |IND_SKIP_Pin|DO4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DO3_Pin|DO2_Pin|DO1_Pin|SG12_3_Pin
                          |SG12_2_Pin|SG12_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = FB_SG8_3_Pin|FB_SG8_2_Pin|FB_SG8_1_Pin|FB_SG7_3_Pin
                          |FB_SG7_2_Pin|FB_SG1_3_Pin|FB_SG1_2_Pin|FB_SG1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin PCPin */
  GPIO_InitStruct.Pin = RTC_POWER_Pin|SG8_3_Pin|SG5_3_Pin|SG5_2_Pin
                          |MEM_WP_Pin|RST_USR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PFPin PFPin PFPin PFPin
                           PFPin PFPin PFPin PFPin
                           PFPin PFPin PFPin PFPin
                           PFPin PFPin PFPin */
  GPIO_InitStruct.Pin = FB_SG7_1_Pin|FB_SG6_3_Pin|FB_SG6_2_Pin|FB_SG6_1_Pin
                          |FB_SG5_3_Pin|FB_SG5_2_Pin|FB_SG5_1_Pin|INPUT_FLASHER_Pin
                          |INPUT_HOLD_Pin|INPUT_SKIP_Pin|FB_SG4_1_Pin|FB_SG3_3_Pin
                          |FB_SG3_2_Pin|FB_SG3_1_Pin|FB_SG2_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = RS485_INT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_INT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                           PAPin PAPin PAPin PAPin
                           PAPin PAPin */
  GPIO_InitStruct.Pin = SG8_2_Pin|SG8_1_Pin|SG7_3_Pin|SG7_2_Pin
                          |SG7_1_Pin|SG6_3_Pin|SG6_2_Pin|SG6_1_Pin
                          |EN_485_Pin|IND_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = SG5_1_Pin|SG2_3_Pin|SG2_2_Pin|SG2_1_Pin
                          |SG1_3_Pin|SG1_2_Pin|SG1_1_Pin|SG11_3_Pin
                          |SG11_2_Pin|SG11_1_Pin|SG10_3_Pin|SG10_2_Pin
                          |SG10_1_Pin|SG9_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = FB_SG4_3_Pin|FB_SG4_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PGPin PGPin PGPin PGPin
                           PGPin PGPin PGPin PGPin
                           PGPin PGPin */
  GPIO_InitStruct.Pin = FB_SG2_2_Pin|FB_SG2_1_Pin|DI4_Pin|DI3_Pin
                          |DI2_Pin|DI1_Pin|FB_SG10_1_Pin|FB_SG9_3_Pin
                          |FB_SG9_2_Pin|FB_SG9_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = SG4_3_Pin|SG4_2_Pin|SG4_1_Pin|SG3_3_Pin
                          |SG3_2_Pin|SG3_1_Pin|SG9_2_Pin|SG9_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin */
  GPIO_InitStruct.Pin = CPU_PROCESS_Pin|IND_GPS_Pin|IND_FLASHER_Pin|IND_HOLD_Pin
                          |IND_SKIP_Pin|DO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PGPin PGPin PGPin PGPin
                           PGPin PGPin */
  GPIO_InitStruct.Pin = DO3_Pin|DO2_Pin|DO1_Pin|SG12_3_Pin
                          |SG12_2_Pin|SG12_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = FB_SG12_3_Pin|FB_SG12_2_Pin|FB_SG12_1_Pin|FB_SG11_3_Pin
                          |FB_SG11_2_Pin|FB_SG11_1_Pin|FB_SG10_3_Pin|FB_SG10_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
