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
#define STEP5_Pin GPIO_PIN_13
#define STEP5_GPIO_Port GPIOC
#define DIR4_Pin GPIO_PIN_14
#define DIR4_GPIO_Port GPIOC
#define STEP4_Pin GPIO_PIN_15
#define STEP4_GPIO_Port GPIOC
#define DIR3_Pin GPIO_PIN_0
#define DIR3_GPIO_Port GPIOC
#define STEP3_Pin GPIO_PIN_1
#define STEP3_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_2
#define DIR2_GPIO_Port GPIOC
#define Z__Pin GPIO_PIN_3
#define Z__GPIO_Port GPIOC
#define HE0_T_Pin GPIO_PIN_0
#define HE0_T_GPIO_Port GPIOA
#define HE1_T_Pin GPIO_PIN_1
#define HE1_T_GPIO_Port GPIOA
#define BED_T_Pin GPIO_PIN_2
#define BED_T_GPIO_Port GPIOA
#define STEP1_Pin GPIO_PIN_3
#define STEP1_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_4
#define DIR1_GPIO_Port GPIOA
#define GPU_SCK_Pin GPIO_PIN_5
#define GPU_SCK_GPIO_Port GPIOA
#define GPU_MISO_Pin GPIO_PIN_6
#define GPU_MISO_GPIO_Port GPIOA
#define GPU_MOSI_Pin GPIO_PIN_7
#define GPU_MOSI_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_4
#define ESP_RX_GPIO_Port GPIOC
#define ESP_TX_Pin GPIO_PIN_5
#define ESP_TX_GPIO_Port GPIOC
#define NZLF_Pin GPIO_PIN_0
#define NZLF_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_1
#define FAN_GPIO_Port GPIOB
#define STEP2_Pin GPIO_PIN_2
#define STEP2_GPIO_Port GPIOB
#define FHE0_Pin GPIO_PIN_10
#define FHE0_GPIO_Port GPIOB
#define FHE1_Pin GPIO_PIN_11
#define FHE1_GPIO_Port GPIOB
#define X_STOP_Pin GPIO_PIN_12
#define X_STOP_GPIO_Port GPIOB
#define BED_Pin GPIO_PIN_15
#define BED_GPIO_Port GPIOB
#define E0_Pin GPIO_PIN_6
#define E0_GPIO_Port GPIOC
#define E1_Pin GPIO_PIN_7
#define E1_GPIO_Port GPIOC
#define SD0_Pin GPIO_PIN_8
#define SD0_GPIO_Port GPIOC
#define SD1_Pin GPIO_PIN_9
#define SD1_GPIO_Port GPIOC
#define HE0_Pin GPIO_PIN_8
#define HE0_GPIO_Port GPIOA
#define TMC_UART_Pin GPIO_PIN_9
#define TMC_UART_GPIO_Port GPIOA
#define HE1_Pin GPIO_PIN_10
#define HE1_GPIO_Port GPIOA
#define Y_STOP_Pin GPIO_PIN_15
#define Y_STOP_GPIO_Port GPIOA
#define SD2_Pin GPIO_PIN_10
#define SD2_GPIO_Port GPIOC
#define SD3_Pin GPIO_PIN_11
#define SD3_GPIO_Port GPIOC
#define SD_CLK_Pin GPIO_PIN_12
#define SD_CLK_GPIO_Port GPIOC
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define TMC_DIAG_Pin GPIO_PIN_4
#define TMC_DIAG_GPIO_Port GPIOB
#define TMC_ENN_Pin GPIO_PIN_5
#define TMC_ENN_GPIO_Port GPIOB
#define TMC_S1_Pin GPIO_PIN_6
#define TMC_S1_GPIO_Port GPIOB
#define TMC_S2_Pin GPIO_PIN_7
#define TMC_S2_GPIO_Port GPIOB
#define TMC_S3_Pin GPIO_PIN_3
#define TMC_S3_GPIO_Port GPIOH
#define DIR5_Pin GPIO_PIN_8
#define DIR5_GPIO_Port GPIOB
#define SERVO_Pin GPIO_PIN_9
#define SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
