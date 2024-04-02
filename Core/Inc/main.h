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
#define Leg2_Rev2_Pin GPIO_PIN_1
#define Leg2_Rev2_GPIO_Port GPIOA
#define Leg2_Rev1_Pin GPIO_PIN_2
#define Leg2_Rev1_GPIO_Port GPIOA
#define Leg2_Rev4_Pin GPIO_PIN_3
#define Leg2_Rev4_GPIO_Port GPIOA
#define Leg3_Rev2_Pin GPIO_PIN_0
#define Leg3_Rev2_GPIO_Port GPIOB
#define Leg3_Rev3_Pin GPIO_PIN_1
#define Leg3_Rev3_GPIO_Port GPIOB
#define Leg2_Rev3_Pin GPIO_PIN_10
#define Leg2_Rev3_GPIO_Port GPIOB
#define Leg3_Rev1_Pin GPIO_PIN_4
#define Leg3_Rev1_GPIO_Port GPIOB
#define Leg3_Rev4_Pin GPIO_PIN_5
#define Leg3_Rev4_GPIO_Port GPIOB
#define Leg1_Rev1_Pin GPIO_PIN_6
#define Leg1_Rev1_GPIO_Port GPIOB
#define Leg1_Rev2_Pin GPIO_PIN_7
#define Leg1_Rev2_GPIO_Port GPIOB
#define Leg1_Rev3_Pin GPIO_PIN_8
#define Leg1_Rev3_GPIO_Port GPIOB
#define Leg1_Rev4_Pin GPIO_PIN_9
#define Leg1_Rev4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
