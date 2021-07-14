/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include <string.h>
#include <stdio.h>


#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void SendChar(uint8_t data);
void SendString(char array[]);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_SCK_Pin 			LL_GPIO_PIN_5
#define SPI1_SCK_GPIO_Port 		GPIOA
#define SPI1_MISO_Pin 			LL_GPIO_PIN_6
#define SPI1_MISO_GPIO_Port 	GPIOA
#define SYN_Pin 				LL_GPIO_PIN_7
#define SYN_GPIO_Port 			GPIOA
#define SPI1_SS_Pin 			LL_GPIO_PIN_4
#define SPI1_SS_GPIO_Port 		GPIOA
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

#define PERIOD_TIME 		(uint32_t)1000000
#define	BUFFER_SIZE			(uint8_t)8

extern uint8_t		TxBuffer[BUFFER_SIZE];
uint8_t 		numberOfPulses;
extern uint32_t 	currTick;
extern char		UART_TxBuffer[256];
extern uint8_t 		byteSent;
typedef enum
{
	TSTD = 0,
	MDIV,
	HSA,
	APL_0,  APL_1,
	TCS,
	FRS,
	FUND,
	ART,
	MSBF,
	ABS_0,	ABS_1,
	LTCH_0,	LTCH_1,
	KMOT_0,	KMOT_1,
	LVS,
	SYS_0,	SYS_1,	SYS_2,
	SCLP,
	PM,
	FR1,
	CCA_0,  CCA_1,  CCA_2,  CCA_3,  CCA_4,  CCA_5,  CCA_6,  CCA_7,  CCA_8,
	CIN_0,	CIN_1,	CIN_2,	CIN_3,	CIN_4,	CIN_5,	CIN_6,	CIN_7,
	CIR_0,	CIR_1,	CIR_2,	CIR_3,	CIR_4,	CIR_5,	CIR_6,	CIR_7,
	CIS_0,	CIS_1,	CIS_2,	CIS_3,	CIS_4,	CIS_5,	CIS_6,	CIS_7,
	CIT_0,	CIT_1,	CIT_2,	CIT_3,	CIT_4,	CIT_5,	CIT_6,	CIT_7,
	CVR_0,	CVR_1,	CVR_2,	CVR_3,	CVR_4,	CVR_5,	CVR_6,	CVR_7,
	CVS_0,	CVS_1,	CVS_2,	CVS_3,	CVS_4,	CVS_5,	CVS_6,	CVS_7,
	CVT_0,	CVT_1,	CVT_2,	CVT_3,	CVT_4,	CVT_5,	CVT_6,	CVT_7,
	CPR_0,	CPR_1,	CPR_2,	CPR_3,
	CPS_0,	CPS_1,	CPS_2,	CPS_3,
	CPT_0,	CPT_1,	CPT_2,	CPT_3,
	CCB_0,	CCB_1,	CCB_2,	CCB_3,	CCB_4,	CCB_5,	CCB_6,	CCB_7,
	CPC_0,	CPC_1,
	ENH,
	CHK,
	TSG_0,  TSG_1,	TSG_2,	TSG_3,
	BANK = 120,
	PUMP,
	TST_0,
	TST1,
	TST2,
	RD,
	WE,
	Precharge
}STPMC1_cfg_bits_address;

typedef enum
{
	temporary = 0,
	permanent

}WritingMode;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
