/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : crypto.h
  * @brief          : Header for dataEncrypt.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @autrhor		: ZenyF
  * @date			: March 04, 2024
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRYPTO_H
#define __CRYPTO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
//#define UART	&huart2

#define BUFFER_SIZE				3000
#define MAX_DATA_LENGTH			2000
#define MAX_VARIABLE			400
#define MAX_VARIABLE_CHARACTER	10

/* USER CODE END Private defines */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct ParsedData
{
    char **tokens;
    uint8_t count;
};
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern const char *Base64SeparatorValue;
extern const char *dictionaryTable[];

// Random Data, only for test purpose
extern uint32_t randomNumbers[MAX_VARIABLE];
extern uint32_t numDummy;

// Encrypt variable
extern char topicData;
extern char sourceInfo;
extern char rawData[MAX_DATA_LENGTH];
extern char arrayData[MAX_VARIABLE][MAX_VARIABLE_CHARACTER];
extern char combinedData[MAX_DATA_LENGTH];
extern char encryptedData[BUFFER_SIZE];

// Decrypt variable
extern char receivedData[BUFFER_SIZE];
extern char sectionsData[5][BUFFER_SIZE];
extern char parsedData[MAX_VARIABLE][MAX_VARIABLE_CHARACTER];
extern uint32_t finalValue[MAX_VARIABLE];
/* USER CODE END PV */

/* USER CODE BEGIN Prototypes */
uint8_t generateRandomNumber(uint32_t numDummy);
uint8_t arrayToString(uint32_t *numbers, uint32_t size, char *buffer);

uint32_t GenerateKey(uint32_t timeBase);
uint32_t SeparateData(char dataIn[], char result[][MAX_VARIABLE_CHARACTER]);
void ShuffleData(char data[][MAX_VARIABLE_CHARACTER], uint32_t size, uint32_t key);
void CombineData(char dataIn[][MAX_VARIABLE_CHARACTER], uint32_t size, char *result);
void ReplaceDictionary(char *data, uint32_t key);
void EncryptData(char *data, char topic);

uint8_t SeparateSections (char* dataIn, char result[][BUFFER_SIZE]);
uint16_t CompareCRC(char* data);
void ReverseShuffleData(char data[][MAX_VARIABLE_CHARACTER], uint32_t size, uint32_t key);
void ReverseReplaceDictionary(char *data, uint32_t key);
uint32_t DecryptData(char *data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CRYPTO_H */
