/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Config.h"
#include "Parser.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
static char msg_buffer[30];
extern uint32_t value[ADC_CHANNELS];
static uint8_t RxBuf[RxBuf_SIZE];

int GetTicks();
void DelayMicrosecond(uint16_t time); 
void SPI_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HOT_BED_Pin GPIO_PIN_3
#define HOT_BED_GPIO_Port GPIOE
#define HOT_END_Pin GPIO_PIN_4
#define HOT_END_GPIO_Port GPIOE
#define HOT_END_FAN_Pin GPIO_PIN_5
#define HOT_END_FAN_GPIO_Port GPIOE
#define NOZZLE_TURBINE_Pin GPIO_PIN_6
#define NOZZLE_TURBINE_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define STEPPER_ENN_Pin GPIO_PIN_0
#define STEPPER_ENN_GPIO_Port GPIOF
#define X_DIR_Pin GPIO_PIN_1
#define X_DIR_GPIO_Port GPIOF
#define Y_DIR_Pin GPIO_PIN_2
#define Y_DIR_GPIO_Port GPIOF
#define E_DIR_Pin GPIO_PIN_3
#define E_DIR_GPIO_Port GPIOF
#define X_STOP_Pin GPIO_PIN_4
#define X_STOP_GPIO_Port GPIOF
#define Y_STOP_Pin GPIO_PIN_5
#define Y_STOP_GPIO_Port GPIOF
#define STEPPER_MPLX_A_Pin GPIO_PIN_6
#define STEPPER_MPLX_A_GPIO_Port GPIOF
#define STEPPER_MPLX_B_Pin GPIO_PIN_7
#define STEPPER_MPLX_B_GPIO_Port GPIOF
#define Z_DIR_Pin GPIO_PIN_8
#define Z_DIR_GPIO_Port GPIOF
#define Z_MIN_Pin GPIO_PIN_9
#define Z_MIN_GPIO_Port GPIOF
#define XSHUT_Pin GPIO_PIN_10
#define XSHUT_GPIO_Port GPIOF
#define BED_TEMP_Pin GPIO_PIN_0
#define BED_TEMP_GPIO_Port GPIOC
#define HOT_END_TEMP_Pin GPIO_PIN_1
#define HOT_END_TEMP_GPIO_Port GPIOC
#define STEPPER_UART_TX_Pin GPIO_PIN_0
#define STEPPER_UART_TX_GPIO_Port GPIOA
#define Z_SERVO_Pin GPIO_PIN_1
#define Z_SERVO_GPIO_Port GPIOA
#define COOLING_FAN_Pin GPIO_PIN_5
#define COOLING_FAN_GPIO_Port GPIOA
#define LCD_MISO_Pin GPIO_PIN_6
#define LCD_MISO_GPIO_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_7
#define Y_STEP_GPIO_Port GPIOA
#define E_STEP_Pin GPIO_PIN_0
#define E_STEP_GPIO_Port GPIOB
#define Z_STEP_Pin GPIO_PIN_1
#define Z_STEP_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOE
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USB_OverCurrent_Pin GPIO_PIN_5
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOG
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOG
#define X_STEP_Pin GPIO_PIN_6
#define X_STEP_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define STEPPER_UART_RX_Pin GPIO_PIN_11
#define STEPPER_UART_RX_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_10
#define SD_CS_GPIO_Port GPIOG
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOG
#define LCD_DC_Pin GPIO_PIN_13
#define LCD_DC_GPIO_Port GPIOG
#define LCD_RST_Pin GPIO_PIN_14
#define LCD_RST_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define VL_SCL_Pin GPIO_PIN_6
#define VL_SCL_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define VL_SDA_Pin GPIO_PIN_9
#define VL_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
