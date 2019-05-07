/**
  ******************************************************************************
  * @file    touch.c
  * @brief   Touch panel driver for EasyMX Pro V7 board.
  ******************************************************************************
  * @attention
  *
  * Author: Kasim Tasdemir
  * Licence: Creative Commons
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "touch.h"
ADC_HandleTypeDef *phadc1_x;
ADC_HandleTypeDef *phadc2_y;
TIM_HandleTypeDef *phtim6_TIM6;

uint32_t x_tmp = 0;
uint32_t y_tmp = 0;
uint32_t x_raw = 0;
uint32_t y_raw = 0;
StageType stage;

void EASYMX_BOARD_TOUCH_CmpltCallback(uint32_t x, uint32_t y){
	
	uint32_t xssa = x;
	uint32_t yssa = y;
	//You can put your code here or override it in the main file.
	__NOP();
}
void EASYMX_BOARD_TOUCH_Init(ADC_HandleTypeDef *phadc1, ADC_HandleTypeDef *phadc2, TIM_HandleTypeDef *phtim6){
	phadc1_x = phadc1;
	phadc2_y = phadc2;
	phtim6_TIM6 = phtim6;
}

void EASYMX_BOARD_TOUCH_Start(void){
	stage=DRIVE_A_B_FOR_X;
	HAL_ADCEx_Calibration_Start(phadc1_x);
	HAL_ADCEx_Calibration_Start(phadc2_y);
	HAL_TIM_Base_Start_IT(phtim6_TIM6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	switch(stage){
		case DRIVE_A_B_FOR_X:
			stage=ADC_START_FOR_X;// Set the next stage to be run in few ms.
			/* Set DriveA and B pins to get ready to read X position and wait some time*/
			HAL_GPIO_WritePin(DRIVE_A_GPIO_Port, DRIVE_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DRIVE_B_GPIO_Port, DRIVE_B_Pin, GPIO_PIN_RESET);
			break;
		case ADC_START_FOR_X:
			stage=WAIT_ONE_MORE_TIME;// Set the next stage to be run in few ms.
			HAL_ADC_Start_IT(phadc1_x);
			break;
		case DRIVE_A_B_FOR_Y:
			stage=ADC_START_FOR_Y;// Set the next stage to be run in few ms.
			/* Set DriveA and B pins to get ready to read X position and wait some time*/
			HAL_GPIO_WritePin(DRIVE_A_GPIO_Port, DRIVE_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DRIVE_B_GPIO_Port, DRIVE_B_Pin, GPIO_PIN_SET);
			break;
		case ADC_START_FOR_Y:			
			stage=WAIT_ONE_MORE_TIME;// Set the next stage to be run in few ms.
			HAL_ADC_Start_IT(phadc2_y);
			break;
		case GOTO_CMPLT_CALLBACK:
			stage = DRIVE_A_B_FOR_X;
			EASYMX_BOARD_TOUCH_CmpltCallback(x_raw, y_raw);
			break;
		case WAIT_ONE_MORE_TIME:
			break;
		default:
			break;
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == phadc1_x->Instance){
		x_tmp = HAL_ADC_GetValue(phadc1_x);
		stage = DRIVE_A_B_FOR_Y;// Set the next stage to be run in few ms.

	}
	else if (hadc->Instance == phadc2_y->Instance){
		y_raw = HAL_ADC_GetValue(phadc2_y);
		x_raw = x_tmp;
		stage = GOTO_CMPLT_CALLBACK;
	}
	
}

/*****************************END OF FILE****/
