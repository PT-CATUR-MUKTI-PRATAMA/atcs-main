/*
 * RS485_driver.c
 *
 *  Created on: Apr 30, 2024
 *      Author: mohhi
 *      Project name : Proyek abadi
 */
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"

#define PORT EN_485_GPIO_Port
#define PIN EN_485_Pin

char rxData;

//Variabel arus setiap SG dan tegangan
uint16_t VoltDC[3], VoltAC[3];
uint16_t I_LAMP[36];


void RequestData_Driver(UART_HandleTypeDef *huart, uint8_t driver_id){
	char txbuff[10];
	sprintf(txbuff, "$%d#", driver_id);

	HAL_GPIO_WritePin(PORT, PIN, 1);
	HAL_UART_Transmit(huart, (uint8_t*)txbuff, strlen(txbuff),1000);
	HAL_GPIO_WritePin(PORT, PIN, 0);
	HAL_Delay(10);
}

void Receive_485(UART_HandleTypeDef *huart)
{ //fungsi harus dipanggil di dalam interupt USART
	uint8_t driverID;
	static uint16_t cnt = 0;
	static char bufferData[100];
	char parsedData[20][20];
	uint16_t driverData[20];

	if(rxData == '@')
	{
		for(int p=0; p<20; p++)
		{
			for(int q=0; q<20; q++)
			{
				parsedData[p][q]='\0';
			}
		}
		for(int p=0; p<100; p++)
		{
			bufferData[p] = '\0';
		}
		cnt = 0;
	}

	else if(rxData == '&')
	{
		for(int p=0, q=0, r=0; p < strlen(bufferData); p++)
		{
			if(bufferData[p] != ',')
			{
				parsedData[q][r] = bufferData[p];
				r++;
			}
			else
			{
				driverData[q] = atoi(parsedData[q]);
				q++;
				r=0;
			}
		}

		driverID = driverData[0];
		VoltDC[driverID-1] = driverData[1];
		VoltAC[driverID-1] = driverData[2];
		for(uint8_t f=(3+(driverID-1)*12); f<(15+(driverID-1)*12); f++){
			I_LAMP[f-3] = driverData[f-((driverID-1)*12)];
		}
	}

	else
	{
		bufferData[cnt] = rxData;
		cnt++;
		cnt=cnt%100;
	}

	HAL_UART_Receive_IT(huart, (uint8_t*)&rxData, 1);
}

