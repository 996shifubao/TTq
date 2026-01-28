/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#pragma once
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct
{
	uint32_t sum[6];
	
	uint16_t buf[6];
	uint16_t ref[6];//基准
	
	int16_t Cur[6];//电流
	
	int16_t CurA;
	int16_t CurB;
	int16_t CurC;
	int16_t CurD;
	int16_t CurE;
	int16_t CurF;
		
}ADCPara_t;

extern ADCPara_t ADCS;
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
void ADC_CaptureRef_Startup(void);
void ADC_PrintRefLoop(void);

void ADC_LL_MyCallback(void);

extern volatile uint8_t calibrationDone1;
extern volatile uint8_t calibrationDone2;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

