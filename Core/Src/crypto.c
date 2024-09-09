/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   crypto.c
  * @brief  Data encryption and decryption process
  *
  ******************************************************************************
  * @author	: ZenyF
  * @date	: March 06, 2024
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crypto.h"
#include "CRCModbus.h"
#include "baseConvertion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
const char *Base64SeparatorValue = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz.,;";

const char *dictionaryTable[] =
{
  //"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz.,;"
	"itFf543bSsjqQpU8;HnoPAWYLVXya.exCm61uKlhDdz7O9,IwcB0TJgNZrkR2MvEG",
	"Wqmn.71OSR30g,XT8YB4lMxZ2PUIkEeo;NVyFuvzbA9KpfGaL5JiDswjhtd6cHQrC",
	"dQuTKoiRU7aN2rkn1DtSqZ;GMhfyzALFB0jOWe5b,VsmPI6Xlxv8HgYcJC4p.Ew93",
	"MEbNZr6YGFRc;qOgKl5yWw781xPBXU3S9s0,nJHToatkLA4imIjeuQ.CzDdhV2pfv"
};

// Random Data, only for test purpose
uint32_t randomNumbers[MAX_VARIABLE];
uint32_t numDummy = 25;

// Encrypt variable
char topicData = 'S';
char sourceInfo = 'H';
char rawData[MAX_DATA_LENGTH];
char arrayData[MAX_VARIABLE][MAX_VARIABLE_CHARACTER];
char combinedData[MAX_DATA_LENGTH];
char encryptedData[BUFFER_SIZE];

// Decrypt variable
char receivedData[BUFFER_SIZE];
char sectionsData[5][BUFFER_SIZE];
char parsedData[MAX_VARIABLE][MAX_VARIABLE_CHARACTER];
uint32_t finalValue[MAX_VARIABLE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

// Function to generate a random number within a specified range
uint8_t generateRandomNumber(uint32_t numDummy)
{
    // Generate and store 5 random numbers within the specified range
    srand(HAL_GetTick());
    for (int i = 0; i < numDummy; i++)
    {
        randomNumbers[i] = rand() % 999991 + 10;
    }
    return 0;
}

uint8_t arrayToString(uint32_t *numbers, uint32_t size, char *buffer)
{
    uint32_t offset = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        // Use sprintf to convert the integer to a string and store it in the buffer
        uint32_t written = sprintf(buffer + offset, "%lu;", numbers[i]);
        if (written < 0 || offset + written >= BUFFER_SIZE)
        {
            // Check for buffer overflow
            return -1;
        }
        offset += written;
    }

    // Remove the last comma
    if (offset > 0)
        buffer[offset - 1] = '\0'; // Null-terminate the string and remove the last comma

    return 0;
}

/* =================================================================================
 * =================================================================================
 * ENCRYPTION PROCEDURE
 * *) Get random value from 2-9 for shuffleKey and dictionaryKey
 * *) Separate data to two dimension char
 * *) LV-1 : Swipe position of variable data depend on shuffleKey
 * *) Convert it from decimal base to Base64
 * *) Combine swipped data to array of char
 * *) LV-2 : Replace character of data depend on dictionaryKey
 * *) Calculate CRC-16 Modbus based
 * *) Combine all section to frame data below
 *    $encryptionKey64:DATA:CRC#
 */

uint32_t GenerateKey(uint32_t timeBase)
{
    uint8_t number[] = {2, 3, 4, 5, 6, 7, 8, 9};
    size_t size = sizeof(number) / sizeof(number[0]);

    srand(timeBase);

    if (size > 1)
    {
        for (uint8_t i = 0; i < size - 1; i++)
        {
          size_t j = i + rand() / (RAND_MAX / (size - i) + 1);

          uint8_t tempNumber = number[j];
          number[j] = number[i];
          number[i] = tempNumber;
        }
    }

    uint32_t randomNumber = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        randomNumber = randomNumber * 10 + number[i];
    }

    return randomNumber;
}

/*
 * Parsed data to two dimension variable array char
 * origin : D1;D2D3;.....;Dn
 * target : char data[0][] = D1; char data[1][] = D2; .....
 *
 */
uint32_t SeparateData(char dataIn[], char result[][MAX_VARIABLE_CHARACTER])
{
    uint32_t dataIndex = 0;
    char *token = strtok(dataIn, ";");
    while (token != NULL && dataIndex < MAX_VARIABLE)
    {
        strcpy(result[dataIndex], token);
        dataIndex++;
        token = strtok(NULL, ";");
    }
    return dataIndex;
}


/*
 *          ----- ENCRYPTION LEVEL 1 -----
 *          Shuffle value of data position
 *   Function to shuffle the position of elements
 * in the two-dimensional array based on shuffleKey
 *
 */
void ShuffleData(char data[][MAX_VARIABLE_CHARACTER], uint32_t size, uint32_t key)
{
    uint32_t keyDigits[4];
    uint32_t cycleShuffle = size / 10;

    if(cycleShuffle != 0)
    {
        // Extracting the individual digits from shuffleKey
        for (int32_t i = 3; i >= 0; i--)
        {
            keyDigits[i] = key % 10;
            key /= 10;
        }

        // Shuffle loop
		for(uint32_t i=0; i < cycleShuffle; i++)
		{
			uint32_t factor = 10 * i;
			// Swap position
            uint32_t nowIndex = factor;
            for (uint32_t j = 0; j < 4; j++)
            {
                char temp[MAX_VARIABLE_CHARACTER];
                uint32_t thenIndex = factor+keyDigits[j]-1;

                strcpy(temp, data[nowIndex]);
                strcpy(data[nowIndex], data[thenIndex]);
                strcpy(data[thenIndex], temp);

                nowIndex = thenIndex;
            }
		}
    }
}

/*
 * Combine the elements of the two-dimensional array into a single string
 * Default separator used (;)
 *
 */
void CombineData(char dataIn[][MAX_VARIABLE_CHARACTER], uint32_t size, char *result)
{
    strcpy(result, ""); // Clear the combinedData array
    for (int i = 0; i < size; i++)
    {
    	uint32_t tempData = atoi(dataIn[i]);
    	char tempArray[16];

    	// Convert to base 64 before combining the data
    	ConvertDecimalToBase64(tempData, tempArray);
    	for(uint32_t j = 0; j < strlen(tempArray); j++)
    	{
    		dataIn[i][j]=tempArray[j];
    		dataIn[i][j+1]='\0';
    	}

        strcat(result, dataIn[i]);
        if (i < size - 1)
        {
        	// Add a semicolon separator if it's not the last element
            strcat(result, ";");
        }
    }
}


/*
 *          ----- ENCRYPTION LEVEL 2 -----
 *            Replace dictionary of data
 * Pattern of replaced data depend on dictionaryKey
 *
 */
void ReplaceDictionary(char *data, uint32_t key)
{
	uint32_t size = strlen(data);

	uint32_t keyDigits[4];
    for (int32_t i = 3; i >= 0; i--)
    {
        keyDigits[i] = key % 10;
        key /= 10;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        uint32_t currentIndex = keyDigits[i];

        for (uint32_t j = 0; j < size; j++)
        {
            if ((j + 1) % currentIndex == 0)
            {
                // copying memory address of encrypt table
                const char *destinationCharPtr = dictionaryTable[i];
                // Find index character of string compared by base table
                char *ptr = strchr(Base64SeparatorValue, data[j]);
                uint8_t sourceIndex = ptr - Base64SeparatorValue;
                // Save replacement character to temporary char variable
                char destinationChar = destinationCharPtr[sourceIndex];
                data[j]=destinationChar;
            }

        }
    }
}


void EncryptData(char *data, char topic)
{
	// Generate encryptKey
	uint32_t shuffleKey = GenerateKey(HAL_GetTick());
	//HAL_Delay(100);
	uint32_t dictionaryKey = GenerateKey(TIM2->CNT);
	//HAL_Delay(100);
	uint32_t encryptKey = (shuffleKey * 10000) + dictionaryKey;
	char encryptKey64[10];
	ConvertDecimalToBase64(encryptKey, encryptKey64);

	// Separate the data
	uint32_t dataCount = SeparateData(data, arrayData);

	// Shuffle data position
	ShuffleData(arrayData, dataCount, shuffleKey);

	// Combine data
	CombineData(arrayData, dataCount, combinedData);

	// Change dictionary
	ReplaceDictionary(combinedData, dictionaryKey);

	// Add colon and dollar
	sprintf(encryptedData,"$%c:%c:%s:%s:",topic, sourceInfo, encryptKey64, combinedData);

	// Calculate CRC16 Modbus
	uint16_t crc = CRC16(encryptedData);
	sprintf(encryptedData,"$%c:%c:%s:%s:%04X#\n\n",topic, sourceInfo, encryptKey64, combinedData, crc);
}




/* =================================================================================
 * =================================================================================
 * DECRYPTION PROCEDURE
 *
 */


/*
 * Separate full frame data from RS485 into three section
 * Section-1 : topicData
 * Section-2 : source
 * Section-3 : decryptKey
 * Section-4 : data
 * Section-5 : crc
 */
uint8_t SeparateSections (char* dataIn, char result[][BUFFER_SIZE])
{
	//copy data to temp data
	char tempData[BUFFER_SIZE];
	strcpy(tempData,dataIn);

    // Separate data into sections based on the ":" separator
    char *token = strtok(tempData, ":");
    uint8_t sections = 0;
    // Iterate through the sections
    while (token != NULL)
    {
    	strcpy(result[sections],token);
        token = strtok(NULL, ":");
        sections++;
    }

    if(sections != 5)
    {
    	sections = 0;
    }

    return sections;
}

/*
 * Extract section of data
 * Eliminate the colon symbol after calculating CRC16
 */
uint16_t CompareCRC(char* data)
{
    uint16_t getCRC;
    uint16_t calcCRC;
    uint8_t crcOk=0;
	size_t length = strlen(data);

	if(length >= 5)
	{
		// Get CRC value from raw data
    	char temp[6];
        strcpy(temp, data + length - 5);
        temp[4]='\0';
        // convert from hexa to integer
        char *endptr;
        getCRC = strtol(temp, &endptr, 16);
        if (*endptr != '\0')
        {
        	// return error
            return 0;
        }

        // Delete the last 5 characters
        data[length - 5] = '\0';
        calcCRC = CRC16(data);

        // Compare the CRC value
        if(getCRC == calcCRC)
        {
        	crcOk = 1;
        }
	}
    else
    {
        // Handle the case where the data is too short
        return 0;
    }

	return crcOk;
}


/*
 *          ----- DECRYPTION LEVEL 1 -----
 *          Shuffle value of data position
 *   Function to shuffle the position of elements
 * in the two-dimensional array based on shuffleKey
 *
 */
void ReverseShuffleData(char data[][MAX_VARIABLE_CHARACTER], uint32_t size, uint32_t key)
{
    uint32_t keyDigits[4];
    uint32_t cycleShuffle = size / 10;

    if(cycleShuffle != 0)
    {
        // Extracting the individual digits from shuffleKey
        for (int32_t i = 3; i >= 0; i--)
        {
            keyDigits[i] = key % 10;
            key /= 10;
        }

        // Shuffle loop
        for(uint32_t i=0; i < cycleShuffle; i++)
		{
			uint32_t factor = 10 * i;
			// Swap position
            uint32_t nowIndex = factor;
            for (int32_t j = 3; j >= 0; j--)
            {
                char temp[MAX_VARIABLE_CHARACTER];
                uint32_t thenIndex = factor+keyDigits[j]-1;

                strcpy(temp, data[nowIndex]);
                strcpy(data[nowIndex], data[thenIndex]);
                strcpy(data[thenIndex], temp);

                nowIndex = thenIndex;
            }
		}
    }
}

/*
 *          ----- DECRYPTION LEVEL 2 -----
 *            Replace dictionary of data
 * Pattern of replaced data depend on dictionaryKey
 *
 */
void ReverseReplaceDictionary(char *data, uint32_t key)
{
	uint32_t size = strlen(data);

	uint32_t keyDigits[4];
    for (int32_t i = 3; i >= 0 ; i--)
    {
        keyDigits[i] = key % 10;
        key /= 10;
    }

    for (int8_t i = 3; i >= 0; i--)
    {
        uint32_t currentIndex = keyDigits[i];

        for (uint32_t j = 0; j < size; j++)
        {
            if ((j + 1) % currentIndex == 0)
            {
                // copying memory address of encrypt table
                const char *destinationCharPtr = Base64SeparatorValue;
                // Find index character of string compared by base table
                char *ptr = strchr(dictionaryTable[i], data[j]);
                uint8_t sourceIndex = ptr - dictionaryTable[i];
                // Save replacement character to temporary char variable
                char destinationChar = destinationCharPtr[sourceIndex];
                data[j]=destinationChar;
            }

        }
    }
}


uint32_t DecryptData(char *data)
{
	// Separate 3 sections
	uint8_t numSection = SeparateSections (data,sectionsData);
	uint32_t numData;

	// Calculate CRC
	uint8_t isCrcOk = CompareCRC(data);

	if(isCrcOk && (numSection==5))
	{
		// Get shuffleKey and dictionaryKey
		uint32_t numDecimal = ConvertBase64ToDecimal(sectionsData[2]);
		uint32_t shuffleKey = numDecimal / 10000;
		uint32_t dictionaryKey = numDecimal % 10000;

		// Reverse replace dictionary
		ReverseReplaceDictionary(sectionsData[3],dictionaryKey);

		//Separate the data
		numData = SeparateData(sectionsData[3],parsedData);

		// Reverse shuffle data
		ReverseShuffleData(parsedData, numData, shuffleKey);

		// Convert to decimal base
	    for (int i = 0; i < numData; i++)
	    {
	    	finalValue [i] = ConvertBase64ToDecimal(parsedData[i]);
	    }
	}
	return numData;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
