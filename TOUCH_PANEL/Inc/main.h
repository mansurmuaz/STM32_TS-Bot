/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D2_Pin GPIO_PIN_2
#define D2_GPIO_Port GPIOE
#define D3_Pin GPIO_PIN_3
#define D3_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOE
#define TOUCH_X_Pin GPIO_PIN_0
#define TOUCH_X_GPIO_Port GPIOB
#define TOUCH_Y_Pin GPIO_PIN_1
#define TOUCH_Y_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOE
#define LCD_BCK_PWM_Pin GPIO_PIN_9
#define LCD_BCK_PWM_GPIO_Port GPIOE
#define LCD_RD_Pin GPIO_PIN_10
#define LCD_RD_GPIO_Port GPIOE
#define LCD_WR_Pin GPIO_PIN_11
#define LCD_WR_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_12
#define LCD_RS_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOE
#define CLK_Pin GPIO_PIN_0
#define CLK_GPIO_Port GPIOD
#define MOSI_Pin GPIO_PIN_1
#define MOSI_GPIO_Port GPIOD
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOD
#define IRQ_Pin GPIO_PIN_3
#define IRQ_GPIO_Port GPIOD
#define MISO_Pin GPIO_PIN_4
#define MISO_GPIO_Port GPIOD
#define DRIVE_A_Pin GPIO_PIN_8
#define DRIVE_A_GPIO_Port GPIOB
#define DRIVE_B_Pin GPIO_PIN_9
#define DRIVE_B_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_0
#define D0_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_1
#define D1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
