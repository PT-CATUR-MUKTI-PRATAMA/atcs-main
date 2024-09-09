/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : baseConvertion.h
  * @brief          : Header for baseConvetion.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @autrhor		: ZenyF
  * @date			: March 04, 2024
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BASECONVERTION_H
#define __BASECONVERTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define BASE 64
/* USER CODE END Private defines */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern const char *Base64Table;
/* USER CODE END PV */

/* USER CODE BEGIN Prototypes */
void ConvertDecimalToBase64(uint32_t numDecimal,char* result);
uint32_t ConvertBase64ToDecimal(const char *encoded);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __BASECONVERTION_H */
