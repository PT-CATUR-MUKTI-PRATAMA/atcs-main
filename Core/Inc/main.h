/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
struct state{uint8_t power, system, normal, hold, skip, flasher, alarm;};
struct setPhase{unsigned int SG1,SG2,SG3,SG4,SG5,SG6,SG7,SG8,SG9,SG10,SG11,SG12;};

typedef struct renc {
  uint8_t g;
  uint8_t y;
  uint8_t r;
  uint8_t AddG;
  uint16_t fase_cycle;
} plan;

typedef struct sch {
  uint8_t jam;
  uint8_t menit;
  uint8_t detik;
  uint8_t plan;
} sche;

typedef struct Libur {
	uint8_t tanggal;
	uint8_t bulan;
	uint8_t tahun;
} HariLibur;

typedef struct Gps {
	uint8_t jam;
	uint8_t menit;
	uint8_t sekon;
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t hari;
	float lng;
	float lat;

} GPS;

extern struct state state, command;

extern uint8_t SG[12][3], SG_set[8][12], SG_set_ext[8][12];

extern uint8_t flag_bypass_GC[66], flag_bypass_GC_ext[66];

extern uint16_t PLAN_Cycle[8], PLAN_Offset[8], PLAN_Cycle_skip;
extern uint16_t PLAN_Cycle_ext[8], PLAN_Offset_ext[8], PLAN_Cycle_skip;

extern plan PLAN[8][8], PLAN_ext[8][8], plan_skip[8];
extern sche SCHE[7][9], SCHE_ext[7][9];
extern HariLibur HARILIBUR[35], HARILIBUR_ext[35];

extern RTC_TimeTypeDef sTime, Offset_Time, Run_Time;
extern RTC_DateTypeDef sDate;

extern uint8_t setting_changed, setting_in_update;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Send_Data_To_Server_Proc(void);
void command_lamp_HIL(int SG_in, uint8_t warna);
uint8_t green_conflict_check(void);
uint8_t green_conflict_HW_check(void);
void set_RTC(void);

void Selected_PLAN_Checking(void);
void first_setup_for_extVar(void);
void update_setting_from_extVar(void);
void update_second(void);
void update_second_FLASH_HOLD_ERROR(void);


void Excute_Red_Time(void);
void Excute_Green_Time(void);
void Excute_Yellow_Time(void);
void Check_Phase_Time(void);
void Reset_cycle_cond(void);

void Setting_Initial_PLAN_Phase_2(void); // For Testing Only
void kedip_startup(void);
void send_lamp_sig(void);
void update_logic_lampu(void);

void get_json_unsignedint(char json[], char var[], unsigned int *output);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FB_SG8_3_Pin GPIO_PIN_2
#define FB_SG8_3_GPIO_Port GPIOE
#define FB_SG8_2_Pin GPIO_PIN_3
#define FB_SG8_2_GPIO_Port GPIOE
#define FB_SG8_1_Pin GPIO_PIN_4
#define FB_SG8_1_GPIO_Port GPIOE
#define FB_SG7_3_Pin GPIO_PIN_5
#define FB_SG7_3_GPIO_Port GPIOE
#define FB_SG7_2_Pin GPIO_PIN_6
#define FB_SG7_2_GPIO_Port GPIOE
#define RTC_POWER_Pin GPIO_PIN_13
#define RTC_POWER_GPIO_Port GPIOC
#define FB_SG7_1_Pin GPIO_PIN_0
#define FB_SG7_1_GPIO_Port GPIOF
#define FB_SG6_3_Pin GPIO_PIN_1
#define FB_SG6_3_GPIO_Port GPIOF
#define FB_SG6_2_Pin GPIO_PIN_2
#define FB_SG6_2_GPIO_Port GPIOF
#define FB_SG6_1_Pin GPIO_PIN_3
#define FB_SG6_1_GPIO_Port GPIOF
#define FB_SG5_3_Pin GPIO_PIN_4
#define FB_SG5_3_GPIO_Port GPIOF
#define FB_SG5_2_Pin GPIO_PIN_5
#define FB_SG5_2_GPIO_Port GPIOF
#define FB_SG5_1_Pin GPIO_PIN_6
#define FB_SG5_1_GPIO_Port GPIOF
#define RS485_INT_EN_Pin GPIO_PIN_7
#define RS485_INT_EN_GPIO_Port GPIOF
#define INPUT_FLASHER_Pin GPIO_PIN_8
#define INPUT_FLASHER_GPIO_Port GPIOF
#define INPUT_HOLD_Pin GPIO_PIN_9
#define INPUT_HOLD_GPIO_Port GPIOF
#define INPUT_SKIP_Pin GPIO_PIN_10
#define INPUT_SKIP_GPIO_Port GPIOF
#define SENSOR_GRID_Pin GPIO_PIN_0
#define SENSOR_GRID_GPIO_Port GPIOC
#define SENSOR_BATT_Pin GPIO_PIN_1
#define SENSOR_BATT_GPIO_Port GPIOC
#define VREF_Pin GPIO_PIN_2
#define VREF_GPIO_Port GPIOC
#define SG8_3_Pin GPIO_PIN_3
#define SG8_3_GPIO_Port GPIOC
#define SG8_2_Pin GPIO_PIN_0
#define SG8_2_GPIO_Port GPIOA
#define SG8_1_Pin GPIO_PIN_1
#define SG8_1_GPIO_Port GPIOA
#define SG7_3_Pin GPIO_PIN_2
#define SG7_3_GPIO_Port GPIOA
#define SG7_2_Pin GPIO_PIN_3
#define SG7_2_GPIO_Port GPIOA
#define SG7_1_Pin GPIO_PIN_4
#define SG7_1_GPIO_Port GPIOA
#define SG6_3_Pin GPIO_PIN_5
#define SG6_3_GPIO_Port GPIOA
#define SG6_2_Pin GPIO_PIN_6
#define SG6_2_GPIO_Port GPIOA
#define SG6_1_Pin GPIO_PIN_7
#define SG6_1_GPIO_Port GPIOA
#define SG5_3_Pin GPIO_PIN_4
#define SG5_3_GPIO_Port GPIOC
#define SG5_2_Pin GPIO_PIN_5
#define SG5_2_GPIO_Port GPIOC
#define SG5_1_Pin GPIO_PIN_0
#define SG5_1_GPIO_Port GPIOB
#define FB_SG4_3_Pin GPIO_PIN_1
#define FB_SG4_3_GPIO_Port GPIOB
#define FB_SG4_2_Pin GPIO_PIN_2
#define FB_SG4_2_GPIO_Port GPIOB
#define FB_SG4_1_Pin GPIO_PIN_11
#define FB_SG4_1_GPIO_Port GPIOF
#define FB_SG3_3_Pin GPIO_PIN_12
#define FB_SG3_3_GPIO_Port GPIOF
#define FB_SG3_2_Pin GPIO_PIN_13
#define FB_SG3_2_GPIO_Port GPIOF
#define FB_SG3_1_Pin GPIO_PIN_14
#define FB_SG3_1_GPIO_Port GPIOF
#define FB_SG2_3_Pin GPIO_PIN_15
#define FB_SG2_3_GPIO_Port GPIOF
#define FB_SG2_2_Pin GPIO_PIN_0
#define FB_SG2_2_GPIO_Port GPIOG
#define FB_SG2_1_Pin GPIO_PIN_1
#define FB_SG2_1_GPIO_Port GPIOG
#define FB_SG1_3_Pin GPIO_PIN_7
#define FB_SG1_3_GPIO_Port GPIOE
#define FB_SG1_2_Pin GPIO_PIN_8
#define FB_SG1_2_GPIO_Port GPIOE
#define FB_SG1_1_Pin GPIO_PIN_9
#define FB_SG1_1_GPIO_Port GPIOE
#define SG4_3_Pin GPIO_PIN_10
#define SG4_3_GPIO_Port GPIOE
#define SG4_2_Pin GPIO_PIN_11
#define SG4_2_GPIO_Port GPIOE
#define SG4_1_Pin GPIO_PIN_12
#define SG4_1_GPIO_Port GPIOE
#define SG3_3_Pin GPIO_PIN_13
#define SG3_3_GPIO_Port GPIOE
#define SG3_2_Pin GPIO_PIN_14
#define SG3_2_GPIO_Port GPIOE
#define SG3_1_Pin GPIO_PIN_15
#define SG3_1_GPIO_Port GPIOE
#define SG2_3_Pin GPIO_PIN_10
#define SG2_3_GPIO_Port GPIOB
#define SG2_2_Pin GPIO_PIN_11
#define SG2_2_GPIO_Port GPIOB
#define SG2_1_Pin GPIO_PIN_12
#define SG2_1_GPIO_Port GPIOB
#define SG1_3_Pin GPIO_PIN_13
#define SG1_3_GPIO_Port GPIOB
#define SG1_2_Pin GPIO_PIN_14
#define SG1_2_GPIO_Port GPIOB
#define SG1_1_Pin GPIO_PIN_15
#define SG1_1_GPIO_Port GPIOB
#define RS485_INT_TX_Pin GPIO_PIN_8
#define RS485_INT_TX_GPIO_Port GPIOD
#define RS485_INT_RX_Pin GPIO_PIN_9
#define RS485_INT_RX_GPIO_Port GPIOD
#define CPU_PROCESS_Pin GPIO_PIN_10
#define CPU_PROCESS_GPIO_Port GPIOD
#define IND_GPS_Pin GPIO_PIN_11
#define IND_GPS_GPIO_Port GPIOD
#define IND_FLASHER_Pin GPIO_PIN_12
#define IND_FLASHER_GPIO_Port GPIOD
#define IND_HOLD_Pin GPIO_PIN_13
#define IND_HOLD_GPIO_Port GPIOD
#define IND_SKIP_Pin GPIO_PIN_14
#define IND_SKIP_GPIO_Port GPIOD
#define DO4_Pin GPIO_PIN_15
#define DO4_GPIO_Port GPIOD
#define DO3_Pin GPIO_PIN_2
#define DO3_GPIO_Port GPIOG
#define DO2_Pin GPIO_PIN_3
#define DO2_GPIO_Port GPIOG
#define DO1_Pin GPIO_PIN_4
#define DO1_GPIO_Port GPIOG
#define DI4_Pin GPIO_PIN_5
#define DI4_GPIO_Port GPIOG
#define DI3_Pin GPIO_PIN_6
#define DI3_GPIO_Port GPIOG
#define DI2_Pin GPIO_PIN_7
#define DI2_GPIO_Port GPIOG
#define DI1_Pin GPIO_PIN_8
#define DI1_GPIO_Port GPIOG
#define USB_TX_Pin GPIO_PIN_6
#define USB_TX_GPIO_Port GPIOC
#define USB_RX_Pin GPIO_PIN_7
#define USB_RX_GPIO_Port GPIOC
#define MEM_WP_Pin GPIO_PIN_8
#define MEM_WP_GPIO_Port GPIOC
#define RS485_TX_Pin GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOA
#define EN_485_Pin GPIO_PIN_11
#define EN_485_GPIO_Port GPIOA
#define IND_ALARM_Pin GPIO_PIN_15
#define IND_ALARM_GPIO_Port GPIOA
#define ETH_TX_Pin GPIO_PIN_10
#define ETH_TX_GPIO_Port GPIOC
#define ETH_RX_Pin GPIO_PIN_11
#define ETH_RX_GPIO_Port GPIOC
#define RST_USR_Pin GPIO_PIN_12
#define RST_USR_GPIO_Port GPIOC
#define FB_SG12_3_Pin GPIO_PIN_0
#define FB_SG12_3_GPIO_Port GPIOD
#define FB_SG12_2_Pin GPIO_PIN_1
#define FB_SG12_2_GPIO_Port GPIOD
#define FB_SG12_1_Pin GPIO_PIN_2
#define FB_SG12_1_GPIO_Port GPIOD
#define FB_SG11_3_Pin GPIO_PIN_3
#define FB_SG11_3_GPIO_Port GPIOD
#define FB_SG11_2_Pin GPIO_PIN_4
#define FB_SG11_2_GPIO_Port GPIOD
#define FB_SG11_1_Pin GPIO_PIN_5
#define FB_SG11_1_GPIO_Port GPIOD
#define FB_SG10_3_Pin GPIO_PIN_6
#define FB_SG10_3_GPIO_Port GPIOD
#define FB_SG10_2_Pin GPIO_PIN_7
#define FB_SG10_2_GPIO_Port GPIOD
#define FB_SG10_1_Pin GPIO_PIN_9
#define FB_SG10_1_GPIO_Port GPIOG
#define FB_SG9_3_Pin GPIO_PIN_10
#define FB_SG9_3_GPIO_Port GPIOG
#define FB_SG9_2_Pin GPIO_PIN_11
#define FB_SG9_2_GPIO_Port GPIOG
#define FB_SG9_1_Pin GPIO_PIN_12
#define FB_SG9_1_GPIO_Port GPIOG
#define SG12_3_Pin GPIO_PIN_13
#define SG12_3_GPIO_Port GPIOG
#define SG12_2_Pin GPIO_PIN_14
#define SG12_2_GPIO_Port GPIOG
#define SG12_1_Pin GPIO_PIN_15
#define SG12_1_GPIO_Port GPIOG
#define SG11_3_Pin GPIO_PIN_3
#define SG11_3_GPIO_Port GPIOB
#define SG11_2_Pin GPIO_PIN_4
#define SG11_2_GPIO_Port GPIOB
#define SG11_1_Pin GPIO_PIN_5
#define SG11_1_GPIO_Port GPIOB
#define SG10_3_Pin GPIO_PIN_6
#define SG10_3_GPIO_Port GPIOB
#define SG10_2_Pin GPIO_PIN_7
#define SG10_2_GPIO_Port GPIOB
#define SG10_1_Pin GPIO_PIN_8
#define SG10_1_GPIO_Port GPIOB
#define SG9_3_Pin GPIO_PIN_9
#define SG9_3_GPIO_Port GPIOB
#define SG9_2_Pin GPIO_PIN_0
#define SG9_2_GPIO_Port GPIOE
#define SG9_1_Pin GPIO_PIN_1
#define SG9_1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define APILL_RED 0
#define APILL_YELLOW 1
#define APILL_GREEN 2
#define APILL_RED_Y 3
#define APILL_NONE 4
#define APILL_FLASH 5
#define APILL_FLASH_GY 6
#define APILL_PDTR_G 7
#define APILL_PDTR_FLASH_G 8
#define APILL_ALL_RED 9
#define APILL_ALL_YELLOW 10
#define APILL_FLASH_Y 11
#define APILL_FLASH_G 12
#define APILL_FLASH_Y_ALL 13
#define APILL_RED_YFlash 14


#define status_RUN 		1
#define status_HOLD 	2
#define status_SKIP 	3
#define status_FLASH 	4
#define status_ERROR	100

#define mode_NonActive	0
#define mode_R1			1
#define mode_R2			2
#define mode_R3			3
#define mode_R4			4
#define mode_RP			5
#define mode_K1			6
#define mode_K2			7
#define mode_K3			8
#define mode_P1			9
#define mode_P2			10
#define mode_F1			11

#define Switch_Flasher	HAL_GPIO_ReadPin(INPUT_FLASHER_GPIO_Port, INPUT_FLASHER_Pin)
#define Switch_Normal	(!HAL_GPIO_ReadPin(INPUT_FLASHER_GPIO_Port, INPUT_FLASHER_Pin))
#define Switch_Hold		HAL_GPIO_ReadPin(INPUT_HOLD_GPIO_Port, INPUT_HOLD_Pin)
#define Switch_Skip		HAL_GPIO_ReadPin(INPUT_SKIP_GPIO_Port, INPUT_SKIP_Pin)

#define FBSG1_3 HAL_GPIO_ReadPin(FB_SG1_3_GPIO_Port, FB_SG1_3_Pin)
#define FBSG2_3 HAL_GPIO_ReadPin(FB_SG2_3_GPIO_Port, FB_SG2_3_Pin)
#define FBSG3_3 HAL_GPIO_ReadPin(FB_SG3_3_GPIO_Port, FB_SG3_3_Pin)
#define FBSG4_3 HAL_GPIO_ReadPin(FB_SG4_3_GPIO_Port, FB_SG4_3_Pin)
#define FBSG5_3 HAL_GPIO_ReadPin(FB_SG5_3_GPIO_Port, FB_SG5_3_Pin)
#define FBSG6_3 HAL_GPIO_ReadPin(FB_SG6_3_GPIO_Port, FB_SG6_3_Pin)
#define FBSG7_3 HAL_GPIO_ReadPin(FB_SG7_3_GPIO_Port, FB_SG7_3_Pin)
#define FBSG8_3 HAL_GPIO_ReadPin(FB_SG8_3_GPIO_Port, FB_SG8_3_Pin)
#define FBSG9_3 HAL_GPIO_ReadPin(FB_SG9_3_GPIO_Port, FB_SG9_3_Pin)
#define FBSG10_3 HAL_GPIO_ReadPin(FB_SG10_3_GPIO_Port, FB_SG10_3_Pin)
#define FBSG11_3 HAL_GPIO_ReadPin(FB_SG11_3_GPIO_Port, FB_SG11_3_Pin)
#define FBSG12_3 HAL_GPIO_ReadPin(FB_SG12_3_GPIO_Port, FB_SG12_3_Pin)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
