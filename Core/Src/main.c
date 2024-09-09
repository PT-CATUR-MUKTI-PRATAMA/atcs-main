/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "baseConvertion.h"
#include "crypto.h"
#include "CRCModbus.h"
#include "RS485_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Variabel Komunikasi UART
char send[10000], send_enc[10000];

uint8_t Rx_data_a[1],Rx_data_i[1];
short int json_detect_a, update_data_a;
int ca;
char data_apps[10000], topic_apps[1];
char send_a[10000];
uint8_t Try_Server=0;


//Variabel Waktu
uint8_t server_weekday,
		server_date,
		server_month,
		server_year,
		server_hour,
		server_minute,
		server_second;

extern int second, lastSecond;
uint16_t Running_Phase,timerun_phs,timerun_cyc;

RTC_TimeTypeDef sTime = {0}, Offset_Time = {0}, Run_Time = {0};
RTC_DateTypeDef sDate = {0};


//Variabel ATCS - Setting Var
struct setPhase phase1, phase2, phase3, phase4, phase5, phase6, phase7, phase8;
uint8_t SG[12][3], SG_set[8][12], SG_set_ext[8][12];

uint8_t flag_bypass_GC[66], flag_bypass_GC_ext[66];

uint16_t PLAN_Cycle[8], PLAN_Offset[8], PLAN_Cycle_skip;
uint16_t PLAN_Cycle_ext[8], PLAN_Offset_ext[8], PLAN_Cycle_skip;

plan PLAN[8][8], PLAN_ext[8][8], plan_skip[8];
sche SCHE[7][9], SCHE_ext[7][9];
HariLibur HARILIBUR[35], HARILIBUR_ext[35];

// Variable ATCS - Running Var
struct state state, command;
uint8_t state_ATCS, old_state_ATCS;

uint8_t flag_flash_y[12], flag_flash_g[12];
uint16_t elapsed_time_green[13];

uint8_t Selected_PLAN;

uint8_t Running_Phase_skip;
uint8_t setting_changed=0, setting_in_update=0;

uint8_t status_GC, status_HWGC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_RTC_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  state.power =1;

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_IT(&huart6, Rx_data_a,1);
  HAL_UART_Receive_IT(&huart3, Rx_data_i,1);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData, 1);

  HAL_GPIO_WritePin(RTC_POWER_GPIO_Port, RTC_POWER_Pin, 1);
  set_RTC();
  kedip_startup();

  Setting_Initial_PLAN_Phase_2();

  state.system=1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // Pengambilan kondisi state dari Saklar menjadi state hardware (agreate state button dan server takeover)

	 if(((Switch_Flasher == 1) || (command.flasher==1)) && state_ATCS<=status_ERROR)
	 {
		 state.normal=0;
		 state.flasher=1;
		 state.hold=0;
		 state.skip=0;
		 state.alarm=0;
	 }
	 else if((Switch_Normal == 1) && state_ATCS<=status_ERROR)
	 {
		 if((Switch_Hold == 1 && command.system == 0) || (command.hold == 1 && command.system == 1))
		 {
			 state.normal=0;
			 state.hold=1;
			 state.skip=0;
		 }
		 else if((Switch_Skip == 1 && command.system == 0) || (command.skip == 1 && command.system == 1))
		 {
			 state.normal=0;
			 state.hold=0;
			 state.skip=1;
		 }
		 else
		 {
			 state.normal=1;
			 state.hold=0;
			 state.skip=0;
		 }
		 state.flasher=0;
		 state.alarm=0;
	 }
	 else
	 {
		 state.normal=0;
		 state.flasher=0;
		 state.hold=0;
		 state.skip=0;
		 state.alarm=1;
	 }

	 // State Hardware digunakan untuk menyalakan LED Indikator Status
	 HAL_GPIO_WritePin(IND_GPS_GPIO_Port, IND_GPS_Pin, state.normal);	// GPS tidak digunakan, digantikan dg indikator normal state.
	 HAL_GPIO_WritePin(IND_FLASHER_GPIO_Port, IND_FLASHER_Pin, state.flasher);
	 HAL_GPIO_WritePin(IND_HOLD_GPIO_Port, IND_HOLD_Pin, state.hold);
	 HAL_GPIO_WritePin(IND_SKIP_GPIO_Port, IND_SKIP_Pin, state.skip);
	 HAL_GPIO_WritePin(IND_ALARM_GPIO_Port, IND_ALARM_Pin, state.alarm);

	 // Pengkondisian state hardware ke state ATCS software
	 old_state_ATCS = state_ATCS;

	 if(state.normal == 1) 			state_ATCS = status_RUN;
	 else if(state.flasher == 1) 	state_ATCS = status_FLASH;
	 else if(state.hold == 1) 		state_ATCS = status_HOLD;
	 else if(state.skip == 1) 		state_ATCS = status_SKIP;
	 else state_ATCS = status_ERROR;

	// Pengkondisian Transisi Status ATCS
	 if(old_state_ATCS==status_RUN && state_ATCS == status_FLASH)
	 {
		 command_lamp_HIL(0, APILL_ALL_YELLOW);
		 HAL_Delay(1000);
	 }
	 if(old_state_ATCS==status_FLASH && state_ATCS==status_RUN)
	 {
		 Reset_cycle_cond();
	 }
	 if(old_state_ATCS==status_HOLD && state_ATCS==status_RUN)
	 {
		 Reset_cycle_cond();
		 command_lamp_HIL(0,APILL_ALL_RED);
		 HAL_Delay(5000);
	 }

	// Mode Run Process
	if(state_ATCS==status_RUN)
	{
		if(Selected_PLAN<=7)
		{
			green_conflict_check();
			update_second();
			if(timerun_cyc<PLAN_Cycle[Selected_PLAN])
			{
				//timerun_phs -> menyatakan lamanya fase sudah berjalan
				//timerun_cyc -> menyatakan lamanya cycle sudah berjalan
				Excute_Red_Time();
				Excute_Green_Time();
				Excute_Yellow_Time();
				Check_Phase_Time();
			}
			else
			{
				// Update PLAN yang akan dijalankan disini berdasarkan Schedulling
				Reset_cycle_cond();
			}
		}
		else
		{
			Reset_cycle_cond();
			command_lamp_HIL(0, APILL_FLASH_Y_ALL);
		}
	}


	else if(state_ATCS==status_FLASH)
	{
		Running_Phase=0;
		timerun_phs=0;
		timerun_cyc=0;
		if(setting_changed)
		{
			update_setting_from_extVar();
			setting_changed=0;
		}
		command_lamp_HIL(0, APILL_FLASH_Y_ALL);
		update_second_FLASH_HOLD_ERROR();
	}

	else if(state_ATCS==status_HOLD)
	{
		update_second_FLASH_HOLD_ERROR();
	}

	else if(state_ATCS==status_ERROR)
	{
		Running_Phase=0;
		timerun_phs=0;
		timerun_cyc=0;
		command_lamp_HIL(0, APILL_FLASH_Y_ALL);
		update_second_FLASH_HOLD_ERROR();
	}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t All_SG_Status[2],status_cond;
char data_send[10000];

void Send_Data_To_Server_Proc(void)
{
	uint8_t Mode_SG[12];
	uint8_t Alarm_status, Error_status;


		All_SG_Status[0] = 0;
		All_SG_Status[1] = 0;

		for(int y=0; y<12; y++)
		{
			if(SG_set[Running_Phase][y]==0) Mode_SG[y] =0;
			else if(SG_set[Running_Phase][y]>0 && SG_set[Running_Phase][y]<=8 && SG_set[Running_Phase][y]!=5) Mode_SG[y]=1;
			else if(SG_set[Running_Phase][y] == 5 || SG_set[Running_Phase][y]>8) Mode_SG[y]=2;
		}


		for(int k=0; k<6; k++)
		{
			for(int l=0; l<3; l++)
			{
				All_SG_Status[0] = (All_SG_Status[0] | (SG[k][l]<<(k*3+l)));
				All_SG_Status[1] = (All_SG_Status[1] | (SG[6+k][l]<<(k*3+l)));
			}
		}

		status_cond = 	(state.power<<0) +
						(state.system<<1) +
						(state.normal<<2) +
						(state.hold<<3) +
						(state.skip<<4) +
						(state.flasher<<5) ;

		HAL_RTC_GetTime(&hrtc, &Run_Time, FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);

		sprintf(data_send,	"%d;%d;%d;%d;%d;%d;" // Status_cond, jam,menit,detik,tgl,bln
							"%d;%d;%d;%d;%d;%d;" // thn,IP1,IP2,IP3,IP4,time_input
							"%d;%d;"	// DI ; DO
							"%d;%d;"    //status lampu SG
							"%d;%d;%d;%d;%d;%d;"
							"%d;%d;%d;%d;%d;%d;"
							"%d;%d;%d;"
							"%d;%d;%d;%d;%d;%d;"
							"%d;%d;%d;%d;%d;%d;"
							"%d;%d;%d;%d;" //TeganganAC,TeganganDC,Baterai,SuhuCPU
							"%d;%d;"       //Alarm, Error
							"",
				status_cond,Run_Time.Hours,Run_Time.Minutes,Run_Time.Seconds,sDate.Date,sDate.Month,
				sDate.Year,129,90,90,1,0,
				0,0,
				All_SG_Status[0],All_SG_Status[1],
				elapsed_time_green[0],elapsed_time_green[1],elapsed_time_green[2],elapsed_time_green[3],elapsed_time_green[4],elapsed_time_green[5],
				elapsed_time_green[6],elapsed_time_green[7],elapsed_time_green[8],elapsed_time_green[9],elapsed_time_green[10],elapsed_time_green[11],
				PLAN_Cycle[Selected_PLAN]-timerun_cyc, Selected_PLAN, Running_Phase+1,
				Mode_SG[0],Mode_SG[1],Mode_SG[2],Mode_SG[3],Mode_SG[4],Mode_SG[5],
				Mode_SG[6],Mode_SG[7],Mode_SG[8],Mode_SG[9],Mode_SG[10],Mode_SG[11],
				223,26,24,30,
				0,0);

		for (int k=0;k<36;k++)
		{
			sprintf(data_send,"%s%d;",data_send,I_LAMP[k]);
		}

		sprintf(data_send,"%s%d;%d;",data_send,command.system,PLAN_Cycle[Selected_PLAN]);

		EncryptData(data_send, 'S');
		sprintf(send_enc, "%s",encryptedData);
		DecryptData(encryptedData);

		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET);
		HAL_UART_Transmit(&huart3, (uint8_t *)send_enc, strlen(send_enc), HAL_MAX_DELAY);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);


}

void command_lamp_HIL(int SG_in, uint8_t warna)
{
	switch(warna) {

	case APILL_RED:
		SG[SG_in][0]=1;
		SG[SG_in][1]=0;
		SG[SG_in][2]=0;
		flag_flash_y[SG_in]=0;
		flag_flash_g[SG_in]=0;
		break;

	case APILL_YELLOW:
		SG[SG_in][0]=0;
		SG[SG_in][1]=1;
		SG[SG_in][2]=0;
		flag_flash_y[SG_in]=0;
		flag_flash_g[SG_in]=0;
		break;

	case APILL_GREEN:
		SG[SG_in][0]=0;
		SG[SG_in][1]=0;
		SG[SG_in][2]=1;
		flag_flash_y[SG_in]=0;
		flag_flash_g[SG_in]=0;
		break;

	case APILL_RED_Y:
		SG[SG_in][0]=1;
		SG[SG_in][1]=1;
		SG[SG_in][2]=0;
		flag_flash_y[SG_in]=0;
		flag_flash_g[SG_in]=0;
		break;

	case APILL_NONE:
		SG[SG_in][0]=0;
		SG[SG_in][1]=0;
		SG[SG_in][2]=0;
		flag_flash_y[SG_in]=0;
		flag_flash_g[SG_in]=0;
		break;

	case APILL_ALL_RED:
		for(int y=0;y<12; y++)
		{
			SG[y][0]=1;
			SG[y][1]=0;
			SG[y][2]=0;
			flag_flash_y[y]=0;
			flag_flash_g[y]=0;
		}
		break;

	case APILL_ALL_YELLOW:
		for(int y=0;y<12; y++)
		{
			SG[y][0]=0;
			SG[y][1]=1;
			SG[y][2]=0;
			flag_flash_y[y]=0;
			flag_flash_g[y]=0;
		}
		break;

	case APILL_FLASH_Y :
		SG[SG_in][0]=0;
		SG[SG_in][2]=0;
		flag_flash_y[SG_in]=1;
		break;

	case APILL_FLASH_G :
		SG[SG_in][0]=0;
		SG[SG_in][1]=0;
		flag_flash_g[SG_in]=1;
		break;

	case APILL_FLASH_Y_ALL :
		for(int y=0;y<12; y++)
		{
			SG[y][0]=0;
			SG[y][2]=0;
			flag_flash_y[y]=1;
		}

		break;

	case APILL_RED_YFlash :
		SG[SG_in][0]=1;
		SG[SG_in][2]=0;
		flag_flash_y[SG_in]=1;

	}

	update_logic_lampu();
}

void command_lamp(int SG, uint8_t channel, uint8_t set_reset)
{
	switch(SG) {
	case 0 :
		if(channel==0 ){ if(set_reset==0) 	HAL_GPIO_WritePin(SG1_1_GPIO_Port, SG1_1_Pin, 0);
			else 						HAL_GPIO_WritePin(SG1_1_GPIO_Port, SG1_1_Pin, 1);}

		if(channel==1 ){ if(set_reset==0) 	HAL_GPIO_WritePin(SG1_2_GPIO_Port, SG1_2_Pin, 0);
			else 						HAL_GPIO_WritePin(SG1_2_GPIO_Port, SG1_2_Pin, 1);}

		if(channel==2 ){ if(set_reset==0) 	HAL_GPIO_WritePin(SG1_3_GPIO_Port, SG1_3_Pin, 0);
			else 						HAL_GPIO_WritePin(SG1_3_GPIO_Port, SG1_3_Pin, 1);}
		break;

	case 1 :
		if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG2_1_GPIO_Port, SG2_1_Pin, 0);
			else 						HAL_GPIO_WritePin(SG2_1_GPIO_Port, SG2_1_Pin, 1);}

		if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG2_2_GPIO_Port, SG2_2_Pin, 0);
			else 						HAL_GPIO_WritePin(SG2_2_GPIO_Port, SG2_2_Pin, 1);}

		if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG2_3_GPIO_Port, SG2_3_Pin, 0);
			else 						HAL_GPIO_WritePin(SG2_3_GPIO_Port, SG2_3_Pin, 1);}
		break;

	case 2 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG3_1_GPIO_Port, SG3_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG3_1_GPIO_Port, SG3_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG3_2_GPIO_Port, SG3_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG3_2_GPIO_Port, SG3_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG3_3_GPIO_Port, SG3_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG3_3_GPIO_Port, SG3_3_Pin, 1);}
			break;
	case 3 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG4_1_GPIO_Port, SG4_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG4_1_GPIO_Port, SG4_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG4_2_GPIO_Port, SG4_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG4_2_GPIO_Port, SG4_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG4_3_GPIO_Port, SG4_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG4_3_GPIO_Port, SG4_3_Pin, 1);}
			break;
	case 4 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG5_1_GPIO_Port, SG5_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG5_1_GPIO_Port, SG5_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG5_2_GPIO_Port, SG5_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG5_2_GPIO_Port, SG5_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG5_3_GPIO_Port, SG5_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG5_3_GPIO_Port, SG5_3_Pin, 1);}
			break;
	case 5 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG6_1_GPIO_Port, SG6_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG6_1_GPIO_Port, SG6_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG6_2_GPIO_Port, SG6_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG6_2_GPIO_Port, SG6_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG6_3_GPIO_Port, SG6_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG6_3_GPIO_Port, SG6_3_Pin, 1);}
			break;
	case 6 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG7_1_GPIO_Port, SG7_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG7_1_GPIO_Port, SG7_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG7_2_GPIO_Port, SG7_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG7_2_GPIO_Port, SG7_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG7_3_GPIO_Port, SG7_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG7_3_GPIO_Port, SG7_3_Pin, 1);}
			break;
	case 7 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG8_1_GPIO_Port, SG8_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG8_1_GPIO_Port, SG8_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG8_2_GPIO_Port, SG8_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG8_2_GPIO_Port, SG8_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG8_3_GPIO_Port, SG8_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG8_3_GPIO_Port, SG8_3_Pin, 1);}
			break;
	case 8 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG9_1_GPIO_Port, SG9_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG9_1_GPIO_Port, SG9_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG9_2_GPIO_Port, SG9_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG9_2_GPIO_Port, SG9_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG9_3_GPIO_Port, SG9_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG9_3_GPIO_Port, SG9_3_Pin, 1);}
			break;
	case 9 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG10_1_GPIO_Port, SG10_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG10_1_GPIO_Port, SG10_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG10_2_GPIO_Port, SG10_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG10_2_GPIO_Port, SG10_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG10_3_GPIO_Port, SG10_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG10_3_GPIO_Port, SG10_3_Pin, 1);}
			break;
	case 10 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG11_1_GPIO_Port, SG11_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG11_1_GPIO_Port, SG11_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG11_2_GPIO_Port, SG11_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG11_2_GPIO_Port, SG11_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG11_3_GPIO_Port, SG11_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG11_3_GPIO_Port, SG11_3_Pin, 1);}
			break;
	case 11 :
			if(channel==0 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG12_1_GPIO_Port, SG12_1_Pin, 0);
				else 						HAL_GPIO_WritePin(SG12_1_GPIO_Port, SG12_1_Pin, 1);}

			if(channel==1 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG12_2_GPIO_Port, SG12_2_Pin, 0);
				else 						HAL_GPIO_WritePin(SG12_2_GPIO_Port, SG12_2_Pin, 1);}

			if(channel==2 ){ if( set_reset==0) 	HAL_GPIO_WritePin(SG12_3_GPIO_Port, SG12_3_Pin, 0);
				else 						HAL_GPIO_WritePin(SG12_3_GPIO_Port, SG12_3_Pin, 1);}
			break;
	}
}

void Excute_Red_Time(void)
{
	if(timerun_phs < PLAN[Selected_PLAN][Running_Phase].r+PLAN[Selected_PLAN][Running_Phase].g+PLAN[Selected_PLAN][Running_Phase].y
			&& timerun_phs >= PLAN[Selected_PLAN][Running_Phase].g+PLAN[Selected_PLAN][Running_Phase].y)
	{
		for(int i=0; i<12;i++)
		{
			switch (SG_set[Running_Phase][i]){
			case (mode_R1):
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_K1) :
				if(SG_set[Running_Phase+1][i]==mode_K1){
					elapsed_time_green[i]= PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r - (timerun_phs - PLAN[Selected_PLAN][Running_Phase].g - PLAN[Selected_PLAN][Running_Phase].y);
				}
				else command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_R2) :
				if(SG_set[Running_Phase+1][i]==mode_R2) command_lamp_HIL(i, APILL_RED);
				else if(SG_set[Running_Phase+1][i]==mode_NonActive)
					if(SG_set[0][i]==mode_R2) command_lamp_HIL(i, APILL_RED);
					else command_lamp_HIL(i, APILL_RED_Y);
				else command_lamp_HIL(i, APILL_RED_Y);
				break;

			case (mode_R3) :
				if(SG_set[Running_Phase+1][i]==mode_R3) command_lamp_HIL(i, APILL_RED);
				else if(SG_set[Running_Phase+1][i]==mode_NonActive)
					if(SG_set[0][i]==mode_R3) command_lamp_HIL(i, APILL_RED);
					else command_lamp_HIL(i, APILL_RED_YFlash);
				else command_lamp_HIL(i, APILL_RED_YFlash);
				break;

			case (mode_R4) :
				if(SG_set[Running_Phase+1][i]==mode_R4) command_lamp_HIL(i, APILL_RED);
				else if(SG_set[Running_Phase+1][i]==mode_NonActive)
					if(SG_set[0][i]==mode_R4) command_lamp_HIL(i, APILL_RED);
					else command_lamp_HIL(i, APILL_YELLOW);
				else command_lamp_HIL(i, APILL_YELLOW);
				break;

			case (mode_RP) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_K2) :
				if(SG_set[Running_Phase+1][i]==mode_K2){
					elapsed_time_green[i]= PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r - (timerun_phs - PLAN[Selected_PLAN][Running_Phase].g - PLAN[Selected_PLAN][Running_Phase].y);
				}
				else command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_K3) :
				if(SG_set[Running_Phase+1][i]==mode_K3){
					elapsed_time_green[i]= PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r - (timerun_phs - PLAN[Selected_PLAN][Running_Phase].g - PLAN[Selected_PLAN][Running_Phase].y);
				}
				else command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_P1) :
				elapsed_time_green[i] = 0;
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_P2) :
				elapsed_time_green[i] = 0;
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_F1) :
				command_lamp_HIL(i, APILL_FLASH_Y);
				break;
			case (mode_NonActive) :
				command_lamp_HIL(i, APILL_NONE);
				break;
			}

		}

	}
}

void Excute_Green_Time(void)
{
	if(timerun_phs < PLAN[Selected_PLAN][Running_Phase].g )
	{
		elapsed_time_green[12] = PLAN[Selected_PLAN][Running_Phase].g -timerun_phs;
		for(int i=0; i<12;i++)
		{
			switch (SG_set[Running_Phase][i]){
			case (mode_R1):
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_K1) :
				command_lamp_HIL(i, APILL_GREEN);		//Mengaktifkan Lampu Hijau

				if(SG_set[Running_Phase+1][i]!=mode_K1)
					elapsed_time_green[i]=elapsed_time_green[12];
				if(SG_set[Running_Phase+1][i]==mode_K1)
					elapsed_time_green[i]=elapsed_time_green[12]+PLAN[Selected_PLAN][Running_Phase].y+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r;
				break;

			case (mode_R2) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_R3) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_R4) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_RP) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_K2) :
				command_lamp_HIL(i, APILL_GREEN);		//Mengaktifkan Lampu Hijau

				if(SG_set[Running_Phase+1][i]!=mode_K2)
					elapsed_time_green[i]=elapsed_time_green[12];
				if(SG_set[Running_Phase+1][i]==mode_K2)
					elapsed_time_green[i]=elapsed_time_green[12]+PLAN[Selected_PLAN][Running_Phase].y+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r;
				break;

			case (mode_K3) :
				if(SG_set[Running_Phase+1][i]!=mode_K3)
				{
					elapsed_time_green[i]=elapsed_time_green[12];
				}

				if(SG_set[Running_Phase+1][i]==mode_K3)
				{
					elapsed_time_green[i]=elapsed_time_green[12]+PLAN[Selected_PLAN][Running_Phase].y+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r;
				}

				if(elapsed_time_green[i]<=5) command_lamp_HIL(i, APILL_FLASH_G);
				else command_lamp_HIL(i, APILL_GREEN);		//Mengaktifkan Lampu Hijau
				break;

			case (mode_P1) :
				command_lamp_HIL(i, APILL_GREEN);		//Mengaktifkan Lampu Hijau

				if(SG_set[Running_Phase+1][i]!=mode_P1)
					elapsed_time_green[i]=elapsed_time_green[12];
				if(SG_set[Running_Phase+1][i]==mode_P1)
					elapsed_time_green[i]=elapsed_time_green[12]+PLAN[Selected_PLAN][Running_Phase].y+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r;

				break;
			case (mode_P2) :
				command_lamp_HIL(i, APILL_GREEN);		//Mengaktifkan Lampu Hijau

				if(SG_set[Running_Phase+1][i]!=mode_P2)
					elapsed_time_green[i]=elapsed_time_green[12];
				if(SG_set[Running_Phase+1][i]==mode_P2)
					elapsed_time_green[i]=elapsed_time_green[12]+PLAN[Selected_PLAN][Running_Phase].y+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].r;
				break;
			case (mode_F1) :
				command_lamp_HIL(i, APILL_FLASH_Y);
				break;
			case (mode_NonActive) :
				command_lamp_HIL(i, APILL_NONE);
				break;
			}

			green_conflict_check();
		}
	}
}

void Excute_Yellow_Time(void)
{
	if(timerun_phs < PLAN[Selected_PLAN][Running_Phase].g+PLAN[Selected_PLAN][Running_Phase].y
			&& timerun_phs >= PLAN[Selected_PLAN][Running_Phase].g)
	{
		for(int i=0; i<12;i++)
		{
			switch (SG_set[Running_Phase][i]){
			case (mode_R1):
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_K1) :
				if(SG_set[Running_Phase+1][i]!=mode_K1)
				{
					command_lamp_HIL(i, APILL_YELLOW);
					elapsed_time_green[i] = 0;
				}
				if(SG_set[Running_Phase+1][i]==mode_K1)
				elapsed_time_green[i]=PLAN[Selected_PLAN][Running_Phase].r+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].y-(timerun_phs-PLAN[Selected_PLAN][Running_Phase].g);

				break;
			case (mode_R2) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_R3) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_R4) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_RP) :
				command_lamp_HIL(i, APILL_RED);
				break;
			case (mode_K2) :
				if(SG_set[Running_Phase+1][i]!=mode_K2)
				{
					command_lamp_HIL(i, APILL_FLASH_Y);
					elapsed_time_green[i] = 0;
				}
				if(SG_set[Running_Phase+1][i]==mode_K2)
				elapsed_time_green[i]=PLAN[Selected_PLAN][Running_Phase].r+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].y-(timerun_phs-PLAN[Selected_PLAN][Running_Phase].g);
				break;
			case (mode_K3) :
				if(SG_set[Running_Phase+1][i]!=mode_K3)
				{
					command_lamp_HIL(i, APILL_YELLOW);
					elapsed_time_green[i] = 0;
				}
				if(SG_set[Running_Phase+1][i]==mode_K3)
				elapsed_time_green[i]=PLAN[Selected_PLAN][Running_Phase].r+PLAN[Selected_PLAN][Running_Phase+1].g+PLAN[Selected_PLAN][Running_Phase].y-(timerun_phs-PLAN[Selected_PLAN][Running_Phase].g);
				break;
			case (mode_P1) :
				command_lamp_HIL(i, APILL_GREEN);
				break;
			case (mode_P2) :
				command_lamp_HIL(i, APILL_FLASH_G);
				break;
			case (mode_F1) :
				command_lamp_HIL(i, APILL_FLASH_Y);
				break;
			case (mode_NonActive) :
				command_lamp_HIL(i, APILL_NONE);
				break;
			}

		}
	}
}

void Check_Phase_Time(void)
{
	if(timerun_phs >= PLAN[Selected_PLAN][Running_Phase].r+PLAN[Selected_PLAN][Running_Phase].g+PLAN[Selected_PLAN][Running_Phase].y)
	{
		Running_Phase+=1;
		timerun_phs=0;
	}
}

void Reset_cycle_cond(void)
{
	Running_Phase=0;
	timerun_phs=0;
	timerun_cyc=0;
//	if(setting_changed)
//	{
//		update_setting_from_extVar();
//		setting_changed=0;
//	}

	Selected_PLAN_Checking();
}

uint32_t schp;
uint32_t timep;
uint8_t last_sch=0;

uint32_t offset_insec, offset_informat;

void Selected_PLAN_Checking(void)
{
	for(int p=0; p<=7; p++)
	{
		offset_insec = PLAN_Cycle[ SCHE[sDate.WeekDay-1][p].plan-1];
		offset_informat = (((int)(offset_insec/60))*100) + (offset_insec%60);

		schp = SCHE[sDate.WeekDay-1][p].jam*10000+(SCHE[sDate.WeekDay-1][p].menit)*100 + (SCHE[sDate.WeekDay-1][p].detik) + offset_informat;
		timep = Run_Time.Hours*10000+Run_Time.Minutes*100+Run_Time.Seconds;

		if(p==0 && timep<schp)
		{
			// Mendeteksi plan terakhir di hari sebelumnya untuk dijadwalkan sebelum jam pertama dihari berjalan.
			for(int r=1;r<=8; r++)
			{
				if(sDate.WeekDay==1) schp = SCHE[7][p].jam*10000+(SCHE[7][p].menit)*100 + (SCHE[7][p].detik);
				else schp = SCHE[sDate.WeekDay-2][p].jam*10000+(SCHE[sDate.WeekDay-2][p].menit)*100 + (SCHE[sDate.WeekDay-2][p].detik);

				if(schp==0)
				{
					last_sch=r-1;
					break;
				}
			}

			if(sDate.WeekDay==1) Selected_PLAN = SCHE[7][last_sch].plan-1;
			else Selected_PLAN = SCHE[sDate.WeekDay-2][last_sch].plan-1;
		}
		else if(timep>=schp)
		{
			if(schp!=0)	Selected_PLAN=SCHE[sDate.WeekDay-1][p].plan-1;
		}

		/*
		 * For Development Purpose
		 */
		//if(Selected_PLAN==0) Selected_PLAN=1;
	}
}

uint8_t green_conflict_check(void)
{
	int jk;
	status_GC=0, status_HWGC=0;

	if(SG[0][2]==1)
	{
		for(jk=1;jk<12;jk++)
		{
			if(SG[jk][2]==1 && (!flag_bypass_GC[jk-1])) status_GC=1*1+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[1][2]==1)
	{
		for(jk=2;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-2+11])) status_GC=1*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[2][2]==1)
	{
		for(jk=3;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-3+21])) status_GC=2*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[3][2]==1)
	{
		for(jk=4;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-4+30])) status_GC=3*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[4][2]==1)
	{
		for(jk=5;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-5+38])) status_GC=4*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[5][2]==1)
	{
		for(jk=6;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-6+45])) status_GC=5*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[6][2]==1)
	{
		for(jk=7;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-7+51])) status_GC=6*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[7][2]==1)
	{
		for(jk=8;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-8+56])) status_GC=7*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[8][2]==1)
	{
		for(jk=9;jk<12;jk++)
		{
			 if(SG[jk][2]==1 && (!flag_bypass_GC[jk-9+60])) status_GC=8*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[9][2]==1)
	{
		for(jk=10;jk<12;jk++)
		{
			if(SG[jk][2]==1 && (!flag_bypass_GC[jk-10+63])) status_GC=9*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
		}
	}

	if(SG[10][2]==1)
		if(SG[11][2]==1 && (!flag_bypass_GC[65])) status_GC=10*10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict

	// Green Conflict Check 1
	 if(status_GC>0)
	 {
		 //command_lamp_HIL(1, APILL_YELLOW);
		 state_ATCS = status_ERROR + status_GC;
	 }

	 status_HWGC = green_conflict_HW_check();
	 if(status_HWGC>0)
	 {
		 //command_lamp_HIL(1, APILL_YELLOW);
		 state_ATCS = status_ERROR + status_HWGC;
	 }

	 return status_GC+(status_HWGC);

}

uint8_t feedback_SG[12][3];
uint8_t status_GC_HW=0;
uint8_t green_conflict_HW_check(void)
{
	int jk;
	status_GC_HW=0;

	feedback_SG[0][2] = FBSG1_3;
	feedback_SG[1][2] = FBSG2_3;
	feedback_SG[2][2] = FBSG3_3;
	feedback_SG[3][2] = FBSG4_3;
	feedback_SG[4][2] = FBSG5_3;
	feedback_SG[5][2] = FBSG6_3;
	feedback_SG[6][2] = FBSG7_3;
	feedback_SG[7][2] = FBSG8_3;
	feedback_SG[8][2] = FBSG9_3;
	feedback_SG[9][2] = FBSG10_3;
	feedback_SG[10][2] = FBSG11_3;
	feedback_SG[11][2] = FBSG12_3;


	if(feedback_SG[0][2]==1)
		{
			for(jk=1;jk<12;jk++)
			{
				if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-1])) status_GC_HW=jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[1][2]==1)
		{
			for(jk=2;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-2+11])) status_GC_HW=10+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[2][2]==1)
		{
			for(jk=3;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-3+21])) status_GC_HW=20+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[3][2]==1)
		{
			for(jk=4;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-4+30])) status_GC_HW=30+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[4][2]==1)
		{
			for(jk=5;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-5+38])) status_GC_HW=40+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[5][2]==1)
		{
			for(jk=6;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-6+45])) status_GC_HW=50+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[6][2]==1)
		{
			for(jk=7;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-7+51])) status_GC_HW=60+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[7][2]==1)
		{
			for(jk=8;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-8+56])) status_GC_HW=70+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[8][2]==1)
		{
			for(jk=9;jk<12;jk++)
			{
				 if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-9+60])) status_GC_HW=80+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[9][2]==1)
		{
			for(jk=10;jk<12;jk++)
			{
				if(feedback_SG[jk][2]==1 && (!flag_bypass_GC[jk-10+63])) status_GC_HW=90+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict
			}
		}

		if(feedback_SG[10][2]==1)
			if(feedback_SG[11][2]==1 && (!flag_bypass_GC[65])) status_GC_HW=100+jk;  //flag_bypass_GC => jika berlogika 0, maka tidak memperbolehkan ada GC,berlogika 1 maka memperbolehkan Green conflict

	return status_GC_HW;
}

void update_second(void)
{
	static uint8_t loop;

	if(second!=lastSecond) {
		lastSecond=second; timerun_cyc++; timerun_phs++;
		Send_Data_To_Server_Proc();

		loop++;
		if(loop==2)
		{
			RequestData_Driver(&huart1, 1);
			RequestData_Driver(&huart1, 2);
			RequestData_Driver(&huart1, 3);
			loop=0;
		}
	}
}

void update_second_FLASH_HOLD_ERROR(void)
{
	static uint8_t loop;

	if(second!=lastSecond) {
		lastSecond=second;
		Send_Data_To_Server_Proc();

		loop++;
		if(loop==2)
		{
			RequestData_Driver(&huart1, 1);
			RequestData_Driver(&huart1, 2);
			RequestData_Driver(&huart1, 3);
			loop=0;
		}
	}
}

uint8_t extVar_range_false;
void check_extVar_range(void)
{
	extVar_range_false=0;
	for(int p=0; p<8; p++)
	{
		for(int q=0; q<12;q++)
		{
			if(SG_set_ext[p][q]>15)
			{
				extVar_range_false=1;
			}
		}
	}
	for(int pq=0; pq<66; pq++)
	{
		if(flag_bypass_GC_ext[pq]>1)
			extVar_range_false =1;
	}
	for(int pq=0; pq<8; pq++)
	{
		for(int qr=0 ;qr<8; qr++)
		{
			if(PLAN_ext[pq][qr].g>300 || PLAN_ext[pq][qr].y>30 || PLAN_ext[pq][qr].r>30)
				extVar_range_false =1;
		}

		if(PLAN_Cycle_ext[pq]>3000 || PLAN_Offset_ext[pq]>180)
		{
			extVar_range_false =1;
		}
	}
	for(int hr=0;hr<7;hr++)
	{
		for(int jw=0; jw<9; jw++)
		{
			if(SCHE_ext[hr][jw].jam>23 || SCHE_ext[hr][jw].menit>59 || SCHE_ext[hr][jw].detik>59 || SCHE_ext[hr][jw].plan>8)
				extVar_range_false =1;
		}
	}
	for(int n=0; n<35; n++)
	{
		if(HARILIBUR_ext[n].tanggal>31 && HARILIBUR_ext[n].bulan>12 && HARILIBUR_ext[n].tahun>99)
			extVar_range_false =1;
	}
}

uint8_t plan_setting_false;
void check_cycle_plan_used(void)
{
	plan_setting_false=0;

	for(int hr=0;hr<7;hr++)
	{
		for(int jw=0; jw<9; jw++)
		{
			if((SCHE_ext[hr][jw].plan-1)>=0 && (SCHE_ext[hr][jw].plan-1)<=7)
			{
				if(PLAN_Cycle_ext[SCHE_ext[hr][jw].plan-1]==0)
					plan_setting_false=1;
			}
			else
				plan_setting_false=1;

		}
	}

}

void update_setting_from_extVar(void)
{
	// Pengecekan Range setiap Variabel yang diterima
	check_extVar_range();

	// Pengecekan cycle pada plan yang digunakan pada penjadwalan
	check_cycle_plan_used();

	if(extVar_range_false ==0 && plan_setting_false==0)
	{
		for(int p=0; p<8; p++)
		{
			for(int q=0; q<12;q++)
			{
				SG_set[p][q] = SG_set_ext[p][q];
			}
		}

		for(int pq=0; pq<66; pq++)
		{
			flag_bypass_GC[pq]=flag_bypass_GC_ext[pq];
		}

		for(int pq=0,sum_cycle=0; pq<8; pq++)
		{
			for(int qr=0 ;qr<8; qr++)
			{
				PLAN[pq][qr].g = PLAN_ext[pq][qr].g;
				PLAN[pq][qr].y = PLAN_ext[pq][qr].y;
				PLAN[pq][qr].r = PLAN_ext[pq][qr].r;

				PLAN_ext[pq][qr].fase_cycle=PLAN_ext[pq][qr].g+PLAN_ext[pq][qr].y+PLAN_ext[pq][qr].r;

				PLAN[pq][qr].fase_cycle = PLAN_ext[pq][qr].fase_cycle ;

				sum_cycle += PLAN_ext[pq][qr].fase_cycle;
			}
			PLAN_Cycle[pq] = PLAN_Cycle_ext[pq];
			PLAN_Offset [pq] = PLAN_Offset_ext[pq];
		}

		for(int hr=0;hr<7;hr++)
		{
			for(int jw=0; jw<9; jw++)
			{
				SCHE[hr][jw] = SCHE_ext[hr][jw];
			}
		}

		for(int n=0; n<35; n++)
		{
			HARILIBUR[n] = HARILIBUR_ext[n];
		}
	}

}

void first_setup_for_extVar(void)
{
	phase1.SG1=SG_set[0][0];
	phase1.SG2=SG_set[0][1];
	phase1.SG3=SG_set[0][2];
	phase1.SG4=SG_set[0][3];
	phase1.SG5=SG_set[0][4];
	phase1.SG6=SG_set[0][5];
	phase1.SG7=SG_set[0][6];
	phase1.SG8=SG_set[0][7];
	phase1.SG9=SG_set[0][8];
	phase1.SG10=SG_set[0][9];
	phase1.SG11=SG_set[0][10];
	phase1.SG12=SG_set[0][11];

	phase2.SG1=SG_set[1][0];
	phase2.SG2=SG_set[1][1];
	phase2.SG3=SG_set[1][2];
	phase2.SG4=SG_set[1][3];
	phase2.SG5=SG_set[1][4];
	phase2.SG6=SG_set[1][5];
	phase2.SG7=SG_set[1][6];
	phase2.SG8=SG_set[1][7];
	phase2.SG9=SG_set[1][8];
	phase2.SG10=SG_set[1][9];
	phase2.SG11=SG_set[1][10];
	phase2.SG12=SG_set[1][11];

	phase3.SG1=SG_set[2][0];
	phase3.SG2=SG_set[2][1];
	phase3.SG3=SG_set[2][2];
	phase3.SG4=SG_set[2][3];
	phase3.SG5=SG_set[2][4];
	phase3.SG6=SG_set[2][5];
	phase3.SG7=SG_set[2][6];
	phase3.SG8=SG_set[2][7];
	phase3.SG9=SG_set[2][8];
	phase3.SG10=SG_set[2][9];
	phase3.SG11=SG_set[2][10];
	phase3.SG12=SG_set[2][11];

	phase4.SG1=SG_set[3][0];
	phase4.SG2=SG_set[3][1];
	phase4.SG3=SG_set[3][2];
	phase4.SG4=SG_set[3][3];
	phase4.SG5=SG_set[3][4];
	phase4.SG6=SG_set[3][5];
	phase4.SG7=SG_set[3][6];
	phase4.SG8=SG_set[3][7];
	phase4.SG9=SG_set[3][8];
	phase4.SG10=SG_set[3][9];
	phase4.SG11=SG_set[3][10];
	phase4.SG12=SG_set[3][11];

	phase5.SG1=SG_set[4][0];
	phase5.SG2=SG_set[4][1];
	phase5.SG3=SG_set[4][2];
	phase5.SG4=SG_set[4][3];
	phase5.SG5=SG_set[4][4];
	phase5.SG6=SG_set[4][5];
	phase5.SG7=SG_set[4][6];
	phase5.SG8=SG_set[4][7];
	phase5.SG9=SG_set[4][8];
	phase5.SG10=SG_set[4][9];
	phase5.SG11=SG_set[4][10];
	phase5.SG12=SG_set[4][11];

	phase6.SG1=SG_set[5][0];
	phase6.SG2=SG_set[5][1];
	phase6.SG3=SG_set[5][2];
	phase6.SG4=SG_set[5][3];
	phase6.SG5=SG_set[5][4];
	phase6.SG6=SG_set[5][5];
	phase6.SG7=SG_set[5][6];
	phase6.SG8=SG_set[5][7];
	phase6.SG9=SG_set[5][8];
	phase6.SG10=SG_set[5][9];
	phase6.SG11=SG_set[5][10];
	phase6.SG12=SG_set[5][11];

	phase7.SG1=SG_set[6][0];
	phase7.SG2=SG_set[6][1];
	phase7.SG3=SG_set[6][2];
	phase7.SG4=SG_set[6][3];
	phase7.SG5=SG_set[6][4];
	phase7.SG6=SG_set[6][5];
	phase7.SG7=SG_set[6][6];
	phase7.SG8=SG_set[6][7];
	phase7.SG9=SG_set[6][8];
	phase7.SG10=SG_set[6][9];
	phase7.SG11=SG_set[6][10];
	phase7.SG12=SG_set[6][11];

	phase8.SG1=SG_set[7][0];
	phase8.SG2=SG_set[7][1];
	phase8.SG3=SG_set[7][2];
	phase8.SG4=SG_set[7][3];
	phase8.SG5=SG_set[7][4];
	phase8.SG6=SG_set[7][5];
	phase8.SG7=SG_set[7][6];
	phase8.SG8=SG_set[7][7];
	phase8.SG9=SG_set[7][8];
	phase8.SG10=SG_set[7][9];
	phase8.SG11=SG_set[7][10];
	phase8.SG12=SG_set[7][11];

	for(int pq=0; pq<66; pq++)
	{
		flag_bypass_GC_ext[pq]=flag_bypass_GC[pq];
	}

	for(int pq=0,sum_cycle=0; pq<8; pq++)
	{
		for(int qr=0 ;qr<8; qr++)
		{
			PLAN_ext[pq][qr].g = PLAN[pq][qr].g;
			PLAN_ext[pq][qr].y = PLAN[pq][qr].y;
			PLAN_ext[pq][qr].r = PLAN[pq][qr].r;

			PLAN[pq][qr].fase_cycle=PLAN[pq][qr].g+PLAN[pq][qr].y+PLAN[pq][qr].r;

			PLAN_ext[pq][qr].fase_cycle = PLAN[pq][qr].fase_cycle ;

			sum_cycle += PLAN[pq][qr].fase_cycle;
		}
		PLAN_Cycle_ext[pq] = sum_cycle;
		PLAN_Offset_ext [pq] = PLAN_Offset[pq];
	}

	for(int hr=0;hr<7;hr++)
	{
		for(int jw=0; jw<5; jw++)
		{
			SCHE_ext[hr][jw] = SCHE[hr][jw];
		}
	}

	for(int n=0; n<35; n++)
	{
		HARILIBUR_ext[n] = HARILIBUR[n];
	}


}

void set_RTC(void)
{
	sDate.Date = server_date;
	sDate.Month = server_month;
	sDate.Year = server_year;
	sDate.WeekDay = server_weekday;

	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);


	Offset_Time.DayLightSaving = 0;
	Offset_Time.Hours = server_hour%24;
	Offset_Time.StoreOperation = 0;
	Offset_Time.TimeFormat = 0;
	Offset_Time.Minutes = ((server_minute + (PLAN_Cycle[Selected_PLAN])/60))%60;
	if(PLAN_Cycle[Selected_PLAN]%60 + server_second>59)
	{
		Offset_Time.Minutes = Offset_Time.Minutes + PLAN_Cycle[Selected_PLAN]/60 + 1;
		Offset_Time.Seconds = PLAN_Cycle[Selected_PLAN]%60 + server_second - 60;
	}
	else
		Offset_Time.Seconds = PLAN_Cycle[Selected_PLAN] + server_second;

	Offset_Time.SubSeconds = sTime.SubSeconds;
	Offset_Time.SecondFraction = sTime.SecondFraction;
	HAL_RTC_SetTime(&hrtc, &Offset_Time, RTC_FORMAT_BIN);
}

void get_json_unsignedint(char json[], char var[], unsigned int *output){
		char *string1, *string2, *string3;
		char nilai[10];
		char nilai1[10];
		int i,j,k;
		string1 = strstr(json,var);
		string2 = strstr(string1, ":");
		string3 = strstr(string1, ",");
		i = (strlen(string1)-strlen(string2))+2;
		j = (strlen(string2)-strlen(string3))-3;
		if(j>0){k = j;}else{k = 0;}
		strncpy (nilai1,string1+i,k);
		if(string1 != NULL){*output = atoi(nilai1);}else{output=output;}
		memset(nilai,0,sizeof nilai1);
}

//For Testing Only
void Setting_Initial_PLAN_Phase_2(void)
{
	PLAN[0][0].g = 15;
	PLAN[0][0].r = 2;
	PLAN[0][0].y = 3;
	PLAN[0][0].fase_cycle =PLAN[0][0].g + PLAN[0][0].r + PLAN[0][0].y;

	PLAN[0][1].g = 15;
	PLAN[0][1].r = 2;
	PLAN[0][1].y = 3;
	PLAN[0][1].fase_cycle =PLAN[0][1].g + PLAN[0][1].r + PLAN[0][1].y;

	PLAN[0][2].g = 15;
	PLAN[0][2].r = 2;
	PLAN[0][2].y = 3;
	PLAN[0][2].fase_cycle =PLAN[0][2].g + PLAN[0][2].r + PLAN[0][2].y;

	PLAN[0][3].g = 15;
	PLAN[0][3].r = 2;
	PLAN[0][3].y = 3;
	PLAN[0][3].fase_cycle =PLAN[0][3].g + PLAN[0][3].r + PLAN[0][3].y;

	PLAN[0][4].g = 15;
	PLAN[0][4].r = 2;
	PLAN[0][4].y = 3;
	PLAN[0][4].fase_cycle =PLAN[0][4].g + PLAN[0][4].r + PLAN[0][4].y;

	PLAN[0][5].g = 15;
	PLAN[0][5].r = 2;
	PLAN[0][5].y = 3;
	PLAN[0][5].fase_cycle =PLAN[0][5].g + PLAN[0][5].r + PLAN[0][5].y;

	PLAN[0][6].g = 15;
	PLAN[0][6].r = 2;
	PLAN[0][6].y = 3;
	PLAN[0][6].fase_cycle =PLAN[0][6].g + PLAN[0][6].r + PLAN[0][6].y;

	PLAN[0][7].g = 15;
	PLAN[0][7].r = 2;
	PLAN[0][7].y = 3;
	PLAN[0][7].fase_cycle =PLAN[0][7].g + PLAN[0][7].r + PLAN[0][7].y;

	for(int jl=0; jl<=7; jl++)
	{
		PLAN_Cycle[0]+=PLAN[0][jl].fase_cycle;
	}
	PLAN_Offset[0]=3;

	PLAN[1][0].g = 10;
	PLAN[1][0].r = 4;
	PLAN[1][0].y = 3;
	PLAN[1][0].fase_cycle =PLAN[1][0].g + PLAN[1][0].r + PLAN[1][0].y;

	PLAN[1][1].g = 10;
	PLAN[1][1].r = 4;
	PLAN[1][1].y = 3;
	PLAN[1][1].fase_cycle =PLAN[1][1].g + PLAN[1][1].r + PLAN[1][1].y;

	PLAN[1][2].g = 10;
	PLAN[1][2].r = 4;
	PLAN[1][2].y = 3;
	PLAN[1][2].fase_cycle =PLAN[1][2].g + PLAN[1][2].r + PLAN[1][2].y;

	PLAN[1][3].g = 10;
	PLAN[1][3].r = 4;
	PLAN[1][3].y = 3;
	PLAN[1][3].fase_cycle =PLAN[1][3].g + PLAN[1][3].r + PLAN[1][1].y;

//	SG_set[0][0]=mode_K1;
//	SG_set[0][1]=mode_K1;
//	SG_set[0][2]=mode_K1;
//	SG_set[0][3]=mode_R1;
//	SG_set[0][4]=mode_R1;
//	SG_set[0][5]=mode_R1;
//	SG_set[0][6]=mode_R1;
//	SG_set[0][7]=mode_R1;
//	SG_set[0][8]=mode_R1;
//	SG_set[0][9]=mode_R1;
//	SG_set[0][10]=mode_R1;
//	SG_set[0][11]=mode_R1;
//
//	SG_set[1][0]=mode_K1;
//	SG_set[1][1]=mode_R1;
//	SG_set[1][2]=mode_R1;
//	SG_set[1][3]=mode_R1;
//	SG_set[1][4]=mode_R1;
//	SG_set[1][5]=mode_R1;
//	SG_set[1][6]=mode_R1;
//	SG_set[1][7]=mode_R1;
//	SG_set[1][8]=mode_R1;
//	SG_set[1][9]=mode_K1;
//	SG_set[1][10]=mode_K1;
//	SG_set[1][11]=mode_K1;
//
//	SG_set[2][0]=mode_R1;
//	SG_set[2][1]=mode_R1;
//	SG_set[2][2]=mode_R1;
//	SG_set[2][3]=mode_R1;
//	SG_set[2][4]=mode_R1;
//	SG_set[2][5]=mode_R1;
//	SG_set[2][6]=mode_K1;
//	SG_set[2][7]=mode_K1;
//	SG_set[2][8]=mode_K1;
//	SG_set[2][9]=mode_K1;
//	SG_set[2][10]=mode_R1;
//	SG_set[2][11]=mode_R1;
//
//	SG_set[3][0]=mode_R1;
//	SG_set[3][1]=mode_R1;
//	SG_set[3][2]=mode_R1;
//	SG_set[3][3]=mode_K1;
//	SG_set[3][4]=mode_K1;
//	SG_set[3][5]=mode_K1;
//	SG_set[3][6]=mode_K1;
//	SG_set[3][7]=mode_R1;
//	SG_set[3][8]=mode_R1;
//	SG_set[3][9]=mode_R1;
//	SG_set[3][10]=mode_R1;
//	SG_set[3][11]=mode_R1;

// Uji 2

//		SG_set[0][0]=mode_K1;
//		SG_set[0][1]=mode_K1;
//		SG_set[0][2]=mode_K1;
//		SG_set[0][3]=mode_R2;
//		SG_set[0][4]=mode_R2;
//		SG_set[0][5]=mode_R2;
//		SG_set[0][6]=mode_R2;
//		SG_set[0][7]=mode_R2;
//		SG_set[0][8]=mode_R2;
//		SG_set[0][9]=mode_R2;
//		SG_set[0][10]=mode_R2;
//		SG_set[0][11]=mode_R2;
//
//		SG_set[1][0]=mode_K1;
//		SG_set[1][1]=mode_R2;
//		SG_set[1][2]=mode_R2;
//		SG_set[1][3]=mode_R2;
//		SG_set[1][4]=mode_R2;
//		SG_set[1][5]=mode_R2;
//		SG_set[1][6]=mode_R2;
//		SG_set[1][7]=mode_R2;
//		SG_set[1][8]=mode_R2;
//		SG_set[1][9]=mode_K1;
//		SG_set[1][10]=mode_K1;
//		SG_set[1][11]=mode_K1;
//
//		SG_set[2][0]=mode_R2;
//		SG_set[2][1]=mode_R2;
//		SG_set[2][2]=mode_R2;
//		SG_set[2][3]=mode_R2;
//		SG_set[2][4]=mode_R2;
//		SG_set[2][5]=mode_R2;
//		SG_set[2][6]=mode_K1;
//		SG_set[2][7]=mode_K1;
//		SG_set[2][8]=mode_K1;
//		SG_set[2][9]=mode_K1;
//		SG_set[2][10]=mode_R2;
//		SG_set[2][11]=mode_R2;
//
//		SG_set[3][0]=mode_R2;
//		SG_set[3][1]=mode_R2;
//		SG_set[3][2]=mode_R2;
//		SG_set[3][3]=mode_K1;
//		SG_set[3][4]=mode_K1;
//		SG_set[3][5]=mode_K1;
//		SG_set[3][6]=mode_K1;
//		SG_set[3][7]=mode_R2;
//		SG_set[3][8]=mode_R2;
//		SG_set[3][9]=mode_R2;
//		SG_set[3][10]=mode_R2;
//		SG_set[3][11]=mode_R2;


	/* uji 3
	 *
	 */
//				SG_set[0][0]=mode_K2;
//				SG_set[0][1]=mode_K2;
//				SG_set[0][2]=mode_K2;
//				SG_set[0][3]=mode_R3;
//				SG_set[0][4]=mode_R3;
//				SG_set[0][5]=mode_R3;
//				SG_set[0][6]=mode_R3;
//				SG_set[0][7]=mode_R3;
//				SG_set[0][8]=mode_R3;
//				SG_set[0][9]=mode_R3;
//				SG_set[0][10]=mode_R3;
//				SG_set[0][11]=mode_R3;
//
//				SG_set[1][0]=mode_K2;
//				SG_set[1][1]=mode_R3;
//				SG_set[1][2]=mode_R3;
//				SG_set[1][3]=mode_R3;
//				SG_set[1][4]=mode_R3;
//				SG_set[1][5]=mode_R3;
//				SG_set[1][6]=mode_R3;
//				SG_set[1][7]=mode_R3;
//				SG_set[1][8]=mode_R3;
//				SG_set[1][9]=mode_K2;
//				SG_set[1][10]=mode_K2;
//				SG_set[1][11]=mode_K2;
//
//				SG_set[2][0]=mode_R3;
//				SG_set[2][1]=mode_R3;
//				SG_set[2][2]=mode_R3;
//				SG_set[2][3]=mode_R3;
//				SG_set[2][4]=mode_R3;
//				SG_set[2][5]=mode_R3;
//				SG_set[2][6]=mode_K2;
//				SG_set[2][7]=mode_K2;
//				SG_set[2][8]=mode_K2;
//				SG_set[2][9]=mode_K2;
//				SG_set[2][10]=mode_R3;
//				SG_set[2][11]=mode_R3;
//
//				SG_set[3][0]=mode_R3;
//				SG_set[3][1]=mode_R3;
//				SG_set[3][2]=mode_R3;
//				SG_set[3][3]=mode_K2;
//				SG_set[3][4]=mode_K2;
//				SG_set[3][5]=mode_K2;
//				SG_set[3][6]=mode_K2;
//				SG_set[3][7]=mode_R3;
//				SG_set[3][8]=mode_R3;
//				SG_set[3][9]=mode_R3;
//				SG_set[3][10]=mode_R3;
//				SG_set[3][11]=mode_R3;



	/*
	 * Uji 4
	 */
					SG_set[0][0]=mode_K3;
					SG_set[0][1]=mode_K3;
					SG_set[0][2]=mode_K3;
					SG_set[0][3]=mode_R1;
					SG_set[0][4]=mode_R1;
					SG_set[0][5]=mode_R1;
					SG_set[0][6]=mode_R1;
					SG_set[0][7]=mode_R1;
					SG_set[0][8]=mode_R1;
					SG_set[0][9]=mode_R1;
					SG_set[0][10]=mode_R1;
					SG_set[0][11]=mode_R1;

					SG_set[1][0]=mode_K3;
					SG_set[1][1]=mode_R1;
					SG_set[1][2]=mode_R1;
					SG_set[1][3]=mode_R1;
					SG_set[1][4]=mode_R1;
					SG_set[1][5]=mode_R1;
					SG_set[1][6]=mode_R1;
					SG_set[1][7]=mode_R1;
					SG_set[1][8]=mode_R1;
					SG_set[1][9]=mode_K3;
					SG_set[1][10]=mode_K3;
					SG_set[1][11]=mode_K3;

					SG_set[2][0]=mode_R1;
					SG_set[2][1]=mode_R1;
					SG_set[2][2]=mode_R1;
					SG_set[2][3]=mode_R1;
					SG_set[2][4]=mode_R1;
					SG_set[2][5]=mode_R1;
					SG_set[2][6]=mode_K3;
					SG_set[2][7]=mode_K3;
					SG_set[2][8]=mode_K3;
					SG_set[2][9]=mode_K3;
					SG_set[2][10]=mode_R1;
					SG_set[2][11]=mode_R1;

					SG_set[3][0]=mode_R1;
					SG_set[3][1]=mode_R1;
					SG_set[3][2]=mode_R1;
					SG_set[3][3]=mode_K3;
					SG_set[3][4]=mode_K3;
					SG_set[3][5]=mode_K3;
					SG_set[3][6]=mode_K3;
					SG_set[3][7]=mode_R1;
					SG_set[3][8]=mode_R1;
					SG_set[3][9]=mode_R1;
					SG_set[3][10]=mode_R1;
					SG_set[3][11]=mode_R1;

/*
	 * Uji 5
	 */
//						SG_set[0][0]=mode_P1;
//						SG_set[0][1]=mode_P1;
//						SG_set[0][2]=mode_P1;
//						SG_set[0][3]=mode_R1;
//						SG_set[0][4]=mode_R1;
//						SG_set[0][5]=mode_R1;
//						SG_set[0][6]=mode_R1;
//						SG_set[0][7]=mode_R1;
//						SG_set[0][8]=mode_R1;
//						SG_set[0][9]=mode_R1;
//						SG_set[0][10]=mode_R1;
//						SG_set[0][11]=mode_R1;
//
//						SG_set[1][0]=mode_P1;
//						SG_set[1][1]=mode_R1;
//						SG_set[1][2]=mode_R1;
//						SG_set[1][3]=mode_R1;
//						SG_set[1][4]=mode_R1;
//						SG_set[1][5]=mode_R1;
//						SG_set[1][6]=mode_R1;
//						SG_set[1][7]=mode_R1;
//						SG_set[1][8]=mode_R1;
//						SG_set[1][9]=mode_P1;
//						SG_set[1][10]=mode_P1;
//						SG_set[1][11]=mode_P1;
//
//						SG_set[2][0]=mode_R1;
//						SG_set[2][1]=mode_R1;
//						SG_set[2][2]=mode_R1;
//						SG_set[2][3]=mode_R1;
//						SG_set[2][4]=mode_R1;
//						SG_set[2][5]=mode_R1;
//						SG_set[2][6]=mode_P1;
//						SG_set[2][7]=mode_P1;
//						SG_set[2][8]=mode_P1;
//						SG_set[2][9]=mode_P1;
//						SG_set[2][10]=mode_R1;
//						SG_set[2][11]=mode_R1;
//
//						SG_set[3][0]=mode_R1;
//						SG_set[3][1]=mode_R1;
//						SG_set[3][2]=mode_R1;
//						SG_set[3][3]=mode_P1;
//						SG_set[3][4]=mode_P1;
//						SG_set[3][5]=mode_P1;
//						SG_set[3][6]=mode_P1;
//						SG_set[3][7]=mode_R1;
//						SG_set[3][8]=mode_R1;
//						SG_set[3][9]=mode_R1;
//						SG_set[3][10]=mode_R1;
//						SG_set[3][11]=mode_R1;

//	UJi6

//							SG_set[0][0]=mode_K1;
//							SG_set[0][1]=mode_K1;
//							SG_set[0][2]=mode_K1;
//							SG_set[0][3]=mode_R4;
//							SG_set[0][4]=mode_R4;
//							SG_set[0][5]=mode_R4;
//							SG_set[0][6]=mode_R4;
//							SG_set[0][7]=mode_R4;
//							SG_set[0][8]=mode_R4;
//							SG_set[0][9]=mode_R4;
//							SG_set[0][10]=mode_R4;
//							SG_set[0][11]=mode_R4;
//
//							SG_set[1][0]=mode_K1;
//							SG_set[1][1]=mode_R4;
//							SG_set[1][2]=mode_R4;
//							SG_set[1][3]=mode_R4;
//							SG_set[1][4]=mode_R4;
//							SG_set[1][5]=mode_R4;
//							SG_set[1][6]=mode_R4;
//							SG_set[1][7]=mode_R4;
//							SG_set[1][8]=mode_R4;
//							SG_set[1][9]=mode_K1;
//							SG_set[1][10]=mode_K1;
//							SG_set[1][11]=mode_K1;
//
//							SG_set[2][0]=mode_R4;
//							SG_set[2][1]=mode_R4;
//							SG_set[2][2]=mode_R4;
//							SG_set[2][3]=mode_R4;
//							SG_set[2][4]=mode_R4;
//							SG_set[2][5]=mode_R4;
//							SG_set[2][6]=mode_K1;
//							SG_set[2][7]=mode_K1;
//							SG_set[2][8]=mode_K1;
//							SG_set[2][9]=mode_K1;
//							SG_set[2][10]=mode_R4;
//							SG_set[2][11]=mode_R4;
//
//							SG_set[3][0]=mode_R4;
//							SG_set[3][1]=mode_R4;
//							SG_set[3][2]=mode_R4;
//							SG_set[3][3]=mode_K1;
//							SG_set[3][4]=mode_K1;
//							SG_set[3][5]=mode_K1;
//							SG_set[3][6]=mode_K1;
//							SG_set[3][7]=mode_R4;
//							SG_set[3][8]=mode_R4;
//							SG_set[3][9]=mode_R4;
//							SG_set[3][10]=mode_R4;
//							SG_set[3][11]=mode_R4;



	SG_set[4][0]=mode_NonActive;
	SG_set[4][1]=mode_NonActive;
	SG_set[4][2]=mode_NonActive;
	SG_set[4][3]=mode_NonActive;
	SG_set[4][4]=mode_NonActive;
	SG_set[4][5]=mode_NonActive;
	SG_set[4][6]=mode_NonActive;
	SG_set[4][7]=mode_NonActive;
	SG_set[4][8]=mode_NonActive;
	SG_set[4][9]=mode_NonActive;
	SG_set[4][10]=mode_NonActive;
	SG_set[4][11]=mode_NonActive;

	flag_bypass_GC[0]=1;
	flag_bypass_GC[1]=1;
	flag_bypass_GC[8]=1;
	flag_bypass_GC[9]=1;
	flag_bypass_GC[10]=1;
	flag_bypass_GC[11]=1;

	flag_bypass_GC[31]=1;
	flag_bypass_GC[32]=1;
	flag_bypass_GC[30]=1;
	flag_bypass_GC[38]=1;
	flag_bypass_GC[39]=1;
	flag_bypass_GC[45]=1;

	flag_bypass_GC[51]=1;
	flag_bypass_GC[52]=1;
	flag_bypass_GC[53]=1;
	flag_bypass_GC[56]=1;
	flag_bypass_GC[57]=1;
	flag_bypass_GC[60]=1;
	flag_bypass_GC[63]=1;
	flag_bypass_GC[64]=1;
	flag_bypass_GC[65]=1;

	Selected_PLAN=1;
	PLAN_Cycle[1]=68;
	PLAN_Offset[1]=1;

}

void kedip_startup(void)
{
	int ulang;

//	sprintf(send,"{\"C1\":%d,\"C2\":%d,\"C3\":%d,\"C4\":%d,\"C5\":%d,\"C6\":%d,\"C7\":%d,\"C8\":%d,\"C9\":%d,\"C10\":%d,\"C11\":%d,\"C12\":%d,\"C13\":%d,\"C14\":%d,\"C15\":%d}\n",
//								elapsed_time_green[0],elapsed_time_green[1],elapsed_time_green[2],elapsed_time_green[3],elapsed_time_green[4],elapsed_time_green[5],elapsed_time_green[6],elapsed_time_green[7],
//								elapsed_time_green[8],elapsed_time_green[9],elapsed_time_green[10],elapsed_time_green[11], PLAN_Cycle[Selected_PLAN]-timerun_cyc, Selected_PLAN+1, Running_Phase+1);
//	HAL_UART_Transmit(&huart1, (uint8_t *)send, strlen(send), HAL_MAX_DELAY);

	for(ulang=0;ulang<3;ulang++)  //Untuk trial kedip=3
	{
		for(int jk=0;jk<12; jk++)
		{
			SG[jk][0]=0;
			SG[jk][1]=1;
			SG[jk][2]=0;
		}
		update_logic_lampu();
		//send_lamp_sig();
		HAL_Delay(500);
		for(int jk=0;jk<12; jk++)
		{
			SG[jk][0]=0;
			SG[jk][1]=0;
			SG[jk][2]=0;
		}
		update_logic_lampu();
		//send_lamp_sig();
		HAL_Delay(500);
	}
	for(int jk=0;jk<12; jk++)
	{
		SG[jk][0]=1;
		SG[jk][1]=0;
		SG[jk][2]=0;
	}
	update_logic_lampu();
	//send_lamp_sig();
	HAL_Delay(2000);
}

void send_lamp_sig(void)
{
	sprintf(send,"{\"L1\":\"%d%d%d\",\"L2\":\"%d%d%d\",\"L3\":\"%d%d%d\",\"L4\":\"%d%d%d\",\"L5\":\"%d%d%d\",\"L6\":\"%d%d%d\",\"L7\":\"%d%d%d\",\"L8\":\"%d%d%d\",\"L9\":\"%d%d%d\",\"L10\":\"%d%d%d\",\"L11\":\"%d%d%d\",\"L12\":\"%d%d%d\"}\n",
										SG[0][0],SG[0][1],SG[0][2],SG[1][0],SG[1][1],SG[1][2],SG[2][0],SG[2][1],SG[2][2],SG[3][0],SG[3][1],SG[3][2],
										SG[4][0],SG[4][1],SG[4][2],SG[5][0],SG[5][1],SG[5][2],SG[6][0],SG[6][1],SG[6][2],SG[7][0],SG[7][1],SG[7][2],
										SG[8][0],SG[8][1],SG[8][2],SG[9][0],SG[9][1],SG[9][2],SG[10][0],SG[10][1],SG[10][2],SG[11][0],SG[11][1],SG[11][2]);
			HAL_UART_Transmit(&huart1, (uint8_t *)send, strlen(send), HAL_MAX_DELAY);
}

void update_logic_lampu(void)
{
	 for(int jk=0; jk<12; jk++)
	  {
		  if(SG[jk][0]==0)					//CEK LAMPU MERAH
		  {
			  command_lamp(jk,0,0);
		  }
		  else
		  {
			  command_lamp(jk,0,1);
		  }

		  if(SG[jk][1]==0)					//CEK LAMPU KUNING
		  {
			  command_lamp(jk,1,0);
		  }
		  else
		  {
			  command_lamp(jk,1,1);
		  }

		  if(SG[jk][2]==0)					//CEK LAMPU HIJAU
		  {
			  command_lamp(jk,2,0);
		  }
		  else
		  {
			  command_lamp(jk,2,1);
		  }
	  }

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
