/**
  ******************************************************************************
  * @file           : touch.h
  * @brief          : Header for touch.c file.
  *                   This file contains the common defines of the touch driver.
  ******************************************************************************
  * @attention
  *
  * Author: Kasim Tasdemir
  * Licence: Creative Commons
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TOUCH_H
#define __TOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"



typedef enum
{
  DRIVE_A_B_FOR_X             = 0x00U,  
  ADC_START_FOR_X             ,
  DRIVE_A_B_FOR_Y             ,
  ADC_START_FOR_Y             ,
	GOTO_CMPLT_CALLBACK					,
  WAIT_ONE_MORE_TIME             
} StageType;

void EASYMX_BOARD_TOUCH_Init(ADC_HandleTypeDef *phadc1, ADC_HandleTypeDef *phadc2, TIM_HandleTypeDef *phtim6);
void EASYMX_BOARD_TOUCH_Start(void);
void EASYMX_BOARD_TOUCH_CmpltCallback(uint32_t x, uint32_t y);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/**************************END OF FILE****/
