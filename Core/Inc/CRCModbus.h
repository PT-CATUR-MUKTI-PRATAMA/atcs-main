/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : CRCModbus.h
  * @brief          : Header for CRCModbus.c file.
  *                   This file contains the common defines of the application
  ******************************************************************************
  * @autrhor		: ZenyF
  * @date			: March 04, 2024
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRCMODBUS_H
#define __CRCMODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* USER CODE BEGIN Prototypes */

uint16_t calculateCRC16(const uint8_t *data, size_t length);
void stringToByteArray(const char *str, uint8_t *byteArray, size_t length);
uint16_t CRC16(const char *data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CRCMODBUS_H */
