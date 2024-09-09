/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "crypto.h"
#include "rtc.h"
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
extern struct setPhase phase1, phase2, phase3, phase4, phase5, phase6, phase7, phase8;
extern struct state state;

extern uint8_t Rx_data_a[1],Rx_data_i[1];
extern short int json_detect_a, update_data_a;
extern int ca;
extern char data_apps[10000], topic_apps[1];
extern char send_a[10000];

extern uint8_t flag_flash_y[12], flag_flash_g[12];

extern uint8_t SG[12][3], SG_set[8][12];

extern uint8_t setting_changed;

extern char send[10000];
//Variable Pewaktu
int second, lastSecond;

// Variabel Encrypt
extern char send[10000], send_enc[10000];

//Variabel bantu send to server
int indeks_detect_i = 0;
uint8_t DataIn_detect_i = 0, Process_DataIn=0;
uint8_t DataIn_i[10000];
char send_i[10000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void command_toggle_y(uint8_t SG);
void command_toggle_g(uint8_t SG);
void send_section_D(void);
void send_section_T(void);
void send_feedback_section_D(void);
void send_feedback_section_T(void);
void send_feedback_section_R(void);
void send_feedback_section_Q(void);
void clear_finalValue(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  HAL_GPIO_TogglePin(CPU_PROCESS_GPIO_Port, CPU_PROCESS_Pin);
  //Send_Data_To_Server_Proc();
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  second++;
  if(second>1000) second=0;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	static uint8_t loop;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  for(int kl=0; kl<12; kl++)
  {
	  if(flag_flash_y[kl] == 1)
	  {
		  command_toggle_y(kl);
	  }
	  if(flag_flash_g[kl] == 1)
	  {
		  command_toggle_g(kl);
	  }
  }

  loop++;
  if(loop>=2)
  {

  }
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  Receive_485(&huart1);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  HAL_UART_Receive_IT(&huart3, Rx_data_i,1);
  if(Rx_data_i[0] == '$'){
   		indeks_detect_i = 0;
   		DataIn_detect_i = 1;
   		DataIn_i[indeks_detect_i] = Rx_data_i[0];
  }
  if(DataIn_detect_i == 1 && Rx_data_i[0] != '$' && Rx_data_i[0] != '#'){
	  indeks_detect_i++;
	  DataIn_i[indeks_detect_i] = Rx_data_i[0];
  }
  if(Rx_data_i[0] == '#'){
	  DataIn_detect_i = 0;
	  indeks_detect_i++;
	  DataIn_i[indeks_detect_i] = Rx_data_i[0];
	  Process_DataIn = 1;
  }
  if(Process_DataIn == 1){


	  // Memproses Feedback ke Server
	  if(indeks_detect_i == 4)
	  {
		  if(DataIn_i[1]=='D')
		  {
			  send_section_D();
		  }
		  else if(DataIn_i[1]=='T')
		  {
			  send_section_T();
		  }
	  }

	  if(indeks_detect_i >4)
	  {
		  if(DataIn_i[1]=='D')
		  {
			  DecryptData(DataIn_i);
			  send_feedback_section_D();
		  }
		  else if(DataIn_i[1]=='T')
		  {
			  DecryptData(DataIn_i);
			  send_feedback_section_T();
		  }
		  else if(DataIn_i[1]=='R')
		  {
			  DecryptData(DataIn_i);
			  send_feedback_section_R();
		  }
		  else if(DataIn_i[1]=='Q')
		  {
			  DecryptData(DataIn_i);
			  send_feedback_section_Q();
		  }

	  }

	  for(int i=0;i<sizeof(DataIn_i);i++){
		  DataIn_i[i] = '\0';
	  }

	  Process_DataIn = 0;
  }

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
  HAL_UART_Receive_IT(&huart6, Rx_data_a,1);
 //	if(Rx_data_a[0] == 'I')					{topic_apps[0] = 'I';}
 	if(Rx_data_a[0] == 'O')					{topic_apps[0] = 'O';}
 	if(Rx_data_a[0] == 'S')         		{topic_apps[0] = 'S';}
 	if(Rx_data_a[0] == 'T')                 {topic_apps[0] = 'T';}
 	if(Rx_data_a[0] == 'U')                 {topic_apps[0] = 'U';}
 	if(Rx_data_a[0] == 'J')                 {topic_apps[0] = 'J';}
 	if(Rx_data_a[0] == 'H')                 {topic_apps[0] = 'H';}
 	if(Rx_data_a[0] == 'M')                 {topic_apps[0] = 'M';}
 	if(Rx_data_a[0] == 'N')                 {topic_apps[0] = 'N';}
 	if(Rx_data_a[0] == 'P')                 {topic_apps[0] = 'P';}
 	if(Rx_data_a[0] == 'G')                 {topic_apps[0] = 'G';}
 	if(Rx_data_a[0] == 'B')                 {topic_apps[0] = 'B';}

 	if(Rx_data_a[0] == '{'){
 		ca = 0;
 		json_detect_a = 1;
 	}
 	if(json_detect_a == 1 && Rx_data_a[0] != '{' && Rx_data_a[0] != '}'){
 		data_apps[ca] = Rx_data_a[0];
 		ca++;
 	}
 	if(Rx_data_a[0] == '}'){
 		data_apps[ca] = ',';
 		json_detect_a = 0;
 		update_data_a = 1;
 	}

 	if(update_data_a == 1 && topic_apps[0] == 'P'){
		get_json_unsignedint(data_apps,"P1" ,&phase1.SG1);   get_json_unsignedint(data_apps,"P2"   ,&phase1.SG2);
		get_json_unsignedint(data_apps,"P3" ,&phase1.SG3);   get_json_unsignedint(data_apps,"P4"   ,&phase1.SG4);
		get_json_unsignedint(data_apps,"P5" ,&phase1.SG5);   get_json_unsignedint(data_apps,"P6"   ,&phase1.SG6);
		get_json_unsignedint(data_apps,"P7" ,&phase1.SG7);   get_json_unsignedint(data_apps,"P8"   ,&phase1.SG8);
		get_json_unsignedint(data_apps,"P9" ,&phase1.SG9);   get_json_unsignedint(data_apps,"P10" ,&phase1.SG10);
		get_json_unsignedint(data_apps,"P11" ,&phase1.SG11); get_json_unsignedint(data_apps,"P12" ,&phase1.SG12);

		get_json_unsignedint(data_apps,"P13" ,&phase2.SG1);  get_json_unsignedint(data_apps,"P14"  ,&phase2.SG2);
		get_json_unsignedint(data_apps,"P15" ,&phase2.SG3);  get_json_unsignedint(data_apps,"P16"  ,&phase2.SG4);
		get_json_unsignedint(data_apps,"P17" ,&phase2.SG5);  get_json_unsignedint(data_apps,"P18"  ,&phase2.SG6);
		get_json_unsignedint(data_apps,"P19" ,&phase2.SG7);  get_json_unsignedint(data_apps,"P20"  ,&phase2.SG8);
		get_json_unsignedint(data_apps,"P21" ,&phase2.SG9);  get_json_unsignedint(data_apps,"P22" ,&phase2.SG10);
		get_json_unsignedint(data_apps,"P23" ,&phase2.SG11); get_json_unsignedint(data_apps,"P24" ,&phase2.SG12);

		get_json_unsignedint(data_apps,"P25" ,&phase3.SG1);  get_json_unsignedint(data_apps,"P26"  ,&phase3.SG2);
		get_json_unsignedint(data_apps,"P27" ,&phase3.SG3);  get_json_unsignedint(data_apps,"P28"  ,&phase3.SG4);
		get_json_unsignedint(data_apps,"P29" ,&phase3.SG5);  get_json_unsignedint(data_apps,"P30"  ,&phase3.SG6);
		get_json_unsignedint(data_apps,"P31" ,&phase3.SG7);  get_json_unsignedint(data_apps,"P32"  ,&phase3.SG8);
		get_json_unsignedint(data_apps,"P33" ,&phase3.SG9);  get_json_unsignedint(data_apps,"P34" ,&phase3.SG10);
		get_json_unsignedint(data_apps,"P35" ,&phase3.SG11); get_json_unsignedint(data_apps,"P36" ,&phase3.SG12);

		get_json_unsignedint(data_apps,"P37" ,&phase4.SG1);  get_json_unsignedint(data_apps,"P38"  ,&phase4.SG2);
		get_json_unsignedint(data_apps,"P39" ,&phase4.SG3);  get_json_unsignedint(data_apps,"P40"  ,&phase4.SG4);
		get_json_unsignedint(data_apps,"P41" ,&phase4.SG5);  get_json_unsignedint(data_apps,"P42"  ,&phase4.SG6);
		get_json_unsignedint(data_apps,"P43" ,&phase4.SG7);  get_json_unsignedint(data_apps,"P44"  ,&phase4.SG8);
		get_json_unsignedint(data_apps,"P45" ,&phase4.SG9);  get_json_unsignedint(data_apps,"P46" ,&phase4.SG10);
		get_json_unsignedint(data_apps,"P47" ,&phase4.SG11); get_json_unsignedint(data_apps,"P48" ,&phase4.SG12);

		get_json_unsignedint(data_apps,"P49" ,&phase5.SG1);  get_json_unsignedint(data_apps,"P50"  ,&phase5.SG2);
		get_json_unsignedint(data_apps,"P51" ,&phase5.SG3);  get_json_unsignedint(data_apps,"P52"  ,&phase5.SG4);
		get_json_unsignedint(data_apps,"P53" ,&phase5.SG5);  get_json_unsignedint(data_apps,"P54"  ,&phase5.SG6);
		get_json_unsignedint(data_apps,"P55" ,&phase5.SG7);  get_json_unsignedint(data_apps,"P56"  ,&phase5.SG8);
		get_json_unsignedint(data_apps,"P57" ,&phase5.SG9);  get_json_unsignedint(data_apps,"P58" ,&phase5.SG10);
		get_json_unsignedint(data_apps,"P59" ,&phase5.SG11); get_json_unsignedint(data_apps,"P60" ,&phase5.SG12);

		get_json_unsignedint(data_apps,"P61" ,&phase6.SG1);  get_json_unsignedint(data_apps,"P62"  ,&phase6.SG2);
		get_json_unsignedint(data_apps,"P63" ,&phase6.SG3);  get_json_unsignedint(data_apps,"P64"  ,&phase6.SG4);
		get_json_unsignedint(data_apps,"P65" ,&phase6.SG5);  get_json_unsignedint(data_apps,"P66"  ,&phase6.SG6);
		get_json_unsignedint(data_apps,"P67" ,&phase6.SG7);  get_json_unsignedint(data_apps,"P68"  ,&phase6.SG8);
		get_json_unsignedint(data_apps,"P69" ,&phase6.SG9);  get_json_unsignedint(data_apps,"P70" ,&phase6.SG10);
		get_json_unsignedint(data_apps,"P71" ,&phase6.SG11); get_json_unsignedint(data_apps,"P72" ,&phase6.SG12);

		get_json_unsignedint(data_apps,"P73" ,&phase7.SG1);  get_json_unsignedint(data_apps,"P74"  ,&phase7.SG2);
		get_json_unsignedint(data_apps,"P75" ,&phase7.SG3);  get_json_unsignedint(data_apps,"P76"  ,&phase7.SG4);
		get_json_unsignedint(data_apps,"P77" ,&phase7.SG5);  get_json_unsignedint(data_apps,"P78"  ,&phase7.SG6);
		get_json_unsignedint(data_apps,"P79" ,&phase7.SG7);  get_json_unsignedint(data_apps,"P80"  ,&phase7.SG8);
		get_json_unsignedint(data_apps,"P81" ,&phase7.SG9);  get_json_unsignedint(data_apps,"P82" ,&phase7.SG10);
		get_json_unsignedint(data_apps,"P83" ,&phase7.SG11); get_json_unsignedint(data_apps,"P84" ,&phase7.SG12);

		get_json_unsignedint(data_apps,"P85" ,&phase8.SG1);  get_json_unsignedint(data_apps,"P86"  ,&phase8.SG2);
		get_json_unsignedint(data_apps,"P87" ,&phase8.SG3);  get_json_unsignedint(data_apps,"P88"  ,&phase8.SG4);
		get_json_unsignedint(data_apps,"P89" ,&phase8.SG5);  get_json_unsignedint(data_apps,"P90"  ,&phase8.SG6);
		get_json_unsignedint(data_apps,"P91" ,&phase8.SG7);  get_json_unsignedint(data_apps,"P92"  ,&phase8.SG8);
		get_json_unsignedint(data_apps,"P93" ,&phase8.SG9);  get_json_unsignedint(data_apps,"P94" ,&phase8.SG10);
		get_json_unsignedint(data_apps,"P95" ,&phase8.SG11); get_json_unsignedint(data_apps,"P96" ,&phase8.SG12);
		for(int i=0;i<sizeof(data_apps);i++){
			data_apps[i] = NULL;
		}
		update_data_a = 0;

		setting_changed=1;
	}
 	if(update_data_a == 1 && topic_apps[0] == 'B'){
 		unsigned int status;
 		get_json_unsignedint(data_apps,"B1" ,&status);

 		switch(status){
 		case(1):
 				state.normal=1;
 				state.hold=0;
 				state.skip=0;
 				state.flasher=0;
 				break;
 		case(2):
				state.normal=0;
				state.hold=1;
				state.skip=0;
				state.flasher=0;
				break;
 		case(3):
				state.normal=0;
				state.hold=0;
				state.skip=1;
				state.flasher=0;
				break;
 		case(4):
				state.normal=0;
				state.hold=0;
				state.skip=0;
				state.flasher=1;
				break;

 		}
 		for(int i=0;i<sizeof(data_apps);i++){
 					data_apps[i] = NULL;
 		}

 		sprintf(send,"{\"D1\":%d,\"D2\":%d,\"D3\":%d,\"D4\":%d,\"D5\":%d,\"D6\":%d}\n",
 				state.power,state.system,state.normal,state.hold, state.skip, state.flasher);
 		HAL_UART_Transmit(&huart6, (uint8_t *)send, strlen(send), HAL_MAX_DELAY);

 		update_data_a = 0;
 	}
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void command_toggle_y(uint8_t sig)
{
	if(SG[sig][1] == 0) SG[sig][1]=1;
	else SG[sig][1]=0;
}
void command_toggle_g(uint8_t sig)
{
	if(SG[sig][2] == 0) SG[sig][2]=1;
	else SG[sig][2]=0;
}
uint32_t Green_Conflict22[3];
void send_section_D(void)
{
	Green_Conflict22[0]=0;
	Green_Conflict22[1]=0;
	Green_Conflict22[2]=0;

	for(uint8_t p=0; p<22; p++)
	{
		Green_Conflict22[0] = Green_Conflict22[0]|((flag_bypass_GC[p] & 0x01) << p);
		Green_Conflict22[1] = Green_Conflict22[1]|((flag_bypass_GC[22+p] & 0x01) << p);
		Green_Conflict22[2] = Green_Conflict22[2]|((flag_bypass_GC[44+p] & 0x01) << p); //
	}

	sprintf(send_i,"%d;%d;%d;%d;",
			0,Green_Conflict22[0],Green_Conflict22[1],Green_Conflict22[2]);

	for(uint8_t q=0; q<8;q++)
	{
		for(uint8_t r=0;r<12;r++)
		{
			sprintf(send_i,"%s%d;",send_i,SG_set[q][r]);
		}
	}

	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			sprintf(send_i,"%s%d;",send_i,PLAN[r][s].g);
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			sprintf(send_i,"%s%d;",send_i,PLAN[r][s].y);
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			sprintf(send_i,"%s%d;",send_i,PLAN[r][s].r);
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			sprintf(send_i,"%s%d;",send_i,PLAN[r][s].AddG);
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		sprintf(send_i,"%s%d;",send_i,PLAN_Cycle[r]);
	}
	for(uint8_t r=0; r<8; r++)
	{
		sprintf(send_i,"%s%d;",send_i,PLAN_Offset[r]);
	}


	EncryptData(send_i, 'D');
	sprintf(send_enc, "%s",encryptedData);

	HAL_UART_Transmit(&huart3, (uint8_t *)send_enc, strlen(send_enc), HAL_MAX_DELAY);
}
void send_section_T(void)
{
	sprintf(send_i,"");

	for(uint8_t p=0;p<7;p++)
	{
		for(uint8_t q=0;q<9;q++)
		{
			if(SCHE[p][q].jam<=9) sprintf(send_i,"%s%02d", send_i,SCHE[p][q].jam);
			else sprintf(send_i,"%s%d", send_i,SCHE[p][q].jam);
			if(SCHE[p][q].menit<=9) sprintf(send_i,"%s%02d", send_i,SCHE[p][q].menit);
			else sprintf(send_i,"%s%d", send_i,SCHE[p][q].menit);
			if(SCHE[p][q].detik<=9) sprintf(send_i,"%s%02d;", send_i,SCHE[p][q].detik);
			else sprintf(send_i,"%s%d;", send_i,SCHE[p][q].detik);
		}
	}

	for(uint8_t p=0;p<7;p++)
	{
		for(uint8_t q=0;q<9;q++)
		{
			sprintf(send_i,"%s%d;", send_i,SCHE[p][q].plan);
		}
	}
	for(uint8_t p=0;p<30;p++)
	{
//		sprintf(send_i,"%s%02d;", send_i,HARILIBUR[p]);
		sprintf(send_i,"%s%02d%02d%02d;", send_i,HARILIBUR[p].tanggal,HARILIBUR[p].bulan,HARILIBUR[p].tahun);
	}


	EncryptData(send_i, 'T');
	sprintf(send_enc, "%s",encryptedData);

	HAL_UART_Transmit(&huart3, (uint8_t *)send_enc, strlen(send_enc), HAL_MAX_DELAY);
}
void send_feedback_section_D(void)
{
	uint32_t Green_Conflict22_rcv[3];

	Green_Conflict22_rcv[0] = finalValue[1];
	Green_Conflict22_rcv[1] = finalValue[2];
	Green_Conflict22_rcv[2] = finalValue[3];

	for(uint8_t p=0; p<22; p++)
	{
		flag_bypass_GC_ext[p] = (Green_Conflict22_rcv[0]>>p) & 0x01;
		flag_bypass_GC_ext[22+p] = (Green_Conflict22_rcv[1]>>p) & 0x01;
		flag_bypass_GC_ext[44+p] = (Green_Conflict22_rcv[2]>>p) & 0x01;
	}

	for(uint8_t q=0; q<8;q++)
	{
		for(uint8_t r=0;r<12;r++)
		{
			SG_set_ext[q][r] = finalValue[(12*q)+4+r];
		}
	}

	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			PLAN_ext[r][s].g = finalValue[(8*r)+100+s];
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			PLAN_ext[r][s].y = finalValue[(8*r)+164+s];
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			PLAN_ext[r][s].r = finalValue[(8*r)+228+s];
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		for(uint8_t s=0; s<8; s++)
		{
			PLAN_ext[r][s].AddG = finalValue[(8*r)+292+s];
		}
	}
	for(uint8_t r=0; r<8; r++)
	{
		PLAN_Cycle_ext[r] = finalValue[356+r];
	}
	for(uint8_t r=0; r<8; r++)
	{
		PLAN_Offset_ext[r] = finalValue[364+r];
	}

	clear_finalValue();

	sprintf(send_enc, "$D:H:1#");

	HAL_UART_Transmit(&huart3, (uint8_t *)send_enc, strlen(send_enc), HAL_MAX_DELAY);

	setting_changed = 0; setting_in_update = 1;
}
void send_feedback_section_T(void)
{
	for(uint8_t p=0;p<7;p++)
	{
		for(uint8_t q=0;q<9;q++)
		{
			// MIsal angka A = 112233
			/*
			 * Untuk mengambil angka 11 maka A/10000
			 * Untuk Mengambil angka 22 maka (A-(11*10000))/100
			 * Untuk mengambil angka 33 maka A - (11*10000) - (22*100)
			 */
			SCHE_ext[p][q].jam = finalValue[q+9*p]/10000;
			SCHE_ext[p][q].menit = (finalValue[q+9*p]/100)%100;
			SCHE_ext[p][q].detik = finalValue[q+9*p]%100;
		}
	}

	for(uint8_t p=0;p<7;p++)
	{
		for(uint8_t q=0;q<9;q++)
		{
			SCHE_ext[p][q].plan = finalValue[63+q+(9*p)];
		}
	}
	for(uint8_t p=0;p<30;p++)
	{
		HARILIBUR_ext[p].tanggal = finalValue[126+p]/10000;
		HARILIBUR_ext[p].bulan = (finalValue[126+p]/100)%100;
		HARILIBUR_ext[p].tahun = finalValue[126+p]%100;
	}

	clear_finalValue();

	sprintf(send_enc, "$T:H:1#");

	HAL_UART_Transmit(&huart3, (uint8_t *)send_enc, strlen(send_enc), HAL_MAX_DELAY);

	if(setting_in_update == 1)
	{
		setting_changed =1;
		setting_in_update =0;
	}
}
void send_feedback_section_R(void)
{
	sDate.Date  =  finalValue[0]/10000;
	sDate.Month = (finalValue[0]/100)%100;
	sDate.Year  =  finalValue[0]%100;

	sTime.Hours = (int)finalValue[1]/10000;
	sTime.Minutes = ((int)finalValue[1]/100)%100;
	sTime.Seconds = (int)finalValue[1]%100;

	HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);

	clear_finalValue();

	sprintf(send_enc, "$R:H:1#");
	HAL_UART_Transmit(&huart3, (uint8_t *)send_enc, strlen(send_enc), HAL_MAX_DELAY);
}
void send_feedback_section_Q(void)
{
	command.hold = (finalValue[0]>>1) & 0x01;
	command.skip = (finalValue[0]>>2) & 0x01;
	command.flasher = (finalValue[0]>>3) & 0x01;
	command.system = (finalValue[0]>>4) & 0x01;

	clear_finalValue();

	sprintf(send_enc, "$Q:H:1#");
	HAL_UART_Transmit(&huart3, (uint8_t *)send_enc, strlen(send_enc), HAL_MAX_DELAY);
}

void clear_finalValue(void)
{
	for(int16_t i=0; i<MAX_VARIABLE; i++)
	{
		finalValue[i] = 0;
	}
}
/* USER CODE END 1 */
