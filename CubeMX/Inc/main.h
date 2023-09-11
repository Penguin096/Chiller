/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
//void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define P_SENSE_Pin GPIO_PIN_0
#define P_SENSE_GPIO_Port GPIOA
#define T_SENS_Pin GPIO_PIN_1
#define T_SENS_GPIO_Port GPIOA
#define C_SENS_Pin GPIO_PIN_2
#define C_SENS_GPIO_Port GPIOA
#define R_SENS_Pin GPIO_PIN_3
#define R_SENS_GPIO_Port GPIOA
#define EN_4988_Pin GPIO_PIN_1
#define EN_4988_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_10
#define STEP_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_11
#define DIR_GPIO_Port GPIOB
#define FLOW_SENS_Pin GPIO_PIN_12
#define FLOW_SENS_GPIO_Port GPIOB
#define LEVEL_SENS_Pin GPIO_PIN_13
#define LEVEL_SENS_GPIO_Port GPIOB
#define FAN_1_Pin GPIO_PIN_14
#define FAN_1_GPIO_Port GPIOB
#define FAN_2_Pin GPIO_PIN_15
#define FAN_2_GPIO_Port GPIOB
#define FAN_3_Pin GPIO_PIN_8
#define FAN_3_GPIO_Port GPIOA
#define RS_DIR_Pin GPIO_PIN_15
#define RS_DIR_GPIO_Port GPIOA
#define RELAY_Pin GPIO_PIN_3
#define RELAY_GPIO_Port GPIOB
#define VALVE_1_Pin GPIO_PIN_4
#define VALVE_1_GPIO_Port GPIOB
#define VALVE_2_Pin GPIO_PIN_5
#define VALVE_2_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_6
#define FAN_GPIO_Port GPIOB
#define PUMP_Pin GPIO_PIN_7
#define PUMP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
