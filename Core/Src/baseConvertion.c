/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    baseConvertion.c
  * @brief   Convert dec to base64 and base64 to dec
  ******************************************************************************
  * @author	: ZenyF
  * @date	: March 06, 2024
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "usart.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
const char *Base64Table = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz.,";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void ConvertDecimalToBase64(uint32_t numDecimal,char* result)
{
    uint8_t indx = 0;
    if(numDecimal > 0)
    {
        while (numDecimal > 0)
        {
        	result[indx++] = Base64Table[numDecimal % 64];
        	numDecimal /= 64;
        }
        result[indx] = '\0';

        // Reverse the string
        for (uint8_t i = 0, j = strlen(result)-1; i < j; i++, j--)
        {
        	static char temp;
            temp = result[i];
            result[i] = result[j];
            result[j] = temp;
        }
    }
    else
    {
    	result[0]=0;
    	result[1]='\0';
    }

}

uint32_t ConvertBase64ToDecimal(const char *encoded)
{
    uint32_t val = 0;
    uint8_t length = strlen(encoded);
    for (uint32_t indx = 0; indx < length; indx++)
    {
        int power = length - (indx + 1);
        const char *pos = strchr(Base64Table, encoded[indx]);
        if (pos == NULL)
        {
            // Character didnt present execution
            return 0;
        }
        val += (uint32_t)(pos - Base64Table) * pow(BASE, power);
    }
    return val;
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
