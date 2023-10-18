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
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "reader.h"
#include "GCodes.h"
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
int GetTicks();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEP_E1_Pin GPIO_PIN_13
#define STEP_E1_GPIO_Port GPIOC
#define DIR_E0_Pin GPIO_PIN_14
#define DIR_E0_GPIO_Port GPIOC
#define STEP_E0_Pin GPIO_PIN_15
#define STEP_E0_GPIO_Port GPIOC
#define DIR_Z_Pin GPIO_PIN_0
#define DIR_Z_GPIO_Port GPIOC
#define STEP_Z_Pin GPIO_PIN_1
#define STEP_Z_GPIO_Port GPIOC
#define DIR_Y_Pin GPIO_PIN_2
#define DIR_Y_GPIO_Port GPIOC
#define Z_MIN_Pin GPIO_PIN_3
#define Z_MIN_GPIO_Port GPIOC
#define HE0_T_Pin GPIO_PIN_0
#define HE0_T_GPIO_Port GPIOA
#define HE1_T_Pin GPIO_PIN_1
#define HE1_T_GPIO_Port GPIOA
#define BED_T_Pin GPIO_PIN_2
#define BED_T_GPIO_Port GPIOA
#define STEP_Y_Pin GPIO_PIN_3
#define STEP_Y_GPIO_Port GPIOA
#define DIR_X_Pin GPIO_PIN_4
#define DIR_X_GPIO_Port GPIOA
#define NZLF_Pin GPIO_PIN_0
#define NZLF_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_1
#define FAN_GPIO_Port GPIOB
#define STEP_X_Pin GPIO_PIN_2
#define STEP_X_GPIO_Port GPIOB
#define FHE0_Pin GPIO_PIN_10
#define FHE0_GPIO_Port GPIOB
#define FHE1_Pin GPIO_PIN_11
#define FHE1_GPIO_Port GPIOB
#define X_STOP_Pin GPIO_PIN_12
#define X_STOP_GPIO_Port GPIOB
#define BED_Pin GPIO_PIN_15
#define BED_GPIO_Port GPIOB
#define HE0_Pin GPIO_PIN_8
#define HE0_GPIO_Port GPIOA
#define TMC_UART_Pin GPIO_PIN_9
#define TMC_UART_GPIO_Port GPIOA
#define HE1_Pin GPIO_PIN_10
#define HE1_GPIO_Port GPIOA
#define Y_STOP_Pin GPIO_PIN_15
#define Y_STOP_GPIO_Port GPIOA
#define TMC_ENN_Pin GPIO_PIN_5
#define TMC_ENN_GPIO_Port GPIOB
#define TMC_S1_Pin GPIO_PIN_6
#define TMC_S1_GPIO_Port GPIOB
#define TMC_S2_Pin GPIO_PIN_7
#define TMC_S2_GPIO_Port GPIOB
#define TMC_S3_Pin GPIO_PIN_3
#define TMC_S3_GPIO_Port GPIOH
#define DIR_E1_Pin GPIO_PIN_8
#define DIR_E1_GPIO_Port GPIOB
#define SERVO_Pin GPIO_PIN_9
#define SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define X_AXIS 			0
#define Y_AXIS 			1
#define Z_AXIS 			2
#define E0_AXIS 		3
#define E1_AXIS			4

#define ADC_CHANNELS 6
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
