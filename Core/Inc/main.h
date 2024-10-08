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
#include "stm32c0xx_hal.h"

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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Oil_Pressure_Pin GPIO_PIN_0
#define Oil_Pressure_GPIO_Port GPIOA
#define Water_Pressure_Pin GPIO_PIN_1
#define Water_Pressure_GPIO_Port GPIOA
#define Water_Flow_Pin GPIO_PIN_2
#define Water_Flow_GPIO_Port GPIOA
#define Jack_Excursion_Pin GPIO_PIN_3
#define Jack_Excursion_GPIO_Port GPIOA
#define Oil_Temp_Pin GPIO_PIN_4
#define Oil_Temp_GPIO_Port GPIOA
#define DCV_POS_B_Pin GPIO_PIN_5
#define DCV_POS_B_GPIO_Port GPIOA
#define DCV_POS_A_Pin GPIO_PIN_6
#define DCV_POS_A_GPIO_Port GPIOA
#define Oil_Low_Pin GPIO_PIN_7
#define Oil_Low_GPIO_Port GPIOA
#define Oil_High_Pin GPIO_PIN_8
#define Oil_High_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
