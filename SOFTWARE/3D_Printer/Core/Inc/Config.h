#ifndef CONFIG_H
#define CONFIG_H

//X AXIS
#define X_AXIS_DIR X_DIR_GPIO_Port,X_DIR_Pin
#define X_AXIS_STEP X_STEP_GPIO_Port,X_STEP_Pin
//Y AXIS
#define Y_AXIS_DIR Y_DIR_GPIO_Port,Y_DIR_Pin
#define Y_AXIS_STEP Y_STEP_GPIO_Port,Y_STEP_Pin
//E AXIS
#define E_AXIS_DIR E_DIR_GPIO_Port,E_DIR_Pin
#define E_AXIS_STEP E_STEP_GPIO_Port,E_STEP_Pin
//Z AXIS
#define Z_AXIS_DIR Z_DIR_GPIO_Port,Z_DIR_Pin
#define Z_AXIS_STEP Z_STEP_GPIO_Port,Z_STEP_Pin

#define AXIS_FAULT_DETECT_PIN GPIOF,GPIO_PIN_0
#define X_AXIS 0
#define Y_AXIS 1
#define E_AXIS 2
#define Z_AXIS 3
#define XY_AXIS 4
#define CLOCKWISE 0
#define COUNTERCLOCKWISE 1
#define MPLX_PIN_A GPIOF,GPIO_PIN_6
#define MPLX_PIN_B GPIOF,GPIO_PIN_7
#define HOT_BED_PIN GPIOE,GPIO_PIN_3
#define HOT_END_PIN GPIOE,GPIO_PIN_4
#define HOT_BED_TEMP_PIN GPIOC,GPIO_PIN_0
#define HOT_END_TEMP_PIN GPIOC,GPIO_PIN_1
#define HOT_END_FAN_PIN GPIOE,GPIO_PIN_5
#define ADC_CHANNELS 3
#define ADC_BED 0
#define ADC_HOT_END 1
#define ADC_STM_TEMP 2
#define HOT_END_THERM_A 0.000726
#define HOT_END_THERM_B 0.000215
#define HOT_END_THERM_C 1.055E-07
#define HOT_BED_THERM_A 0.00128
#define HOT_BED_THERM_B 0.00024
#define HOT_BED_THERM_C 8.5E-08
#define ADC_VOLTAGE 3.3
#define ADC_SAMPLING 65536
#define HOT_END_RESISTANCE 10000
#define HOT_BED_RESISTANCE 4700
#define KELVIN_OFFSET 273.15
#define V30 0.76
#define AVG_SLOPE 0.0025
#define HOT_END_FAN 0
#define HOT_END 0
#define HOT_BED 1
#define RxBuf_SIZE	40
#define STEPS_PER_REV 19256.0f
#define E_MM_PER_REV 23.0f
#define X_STOP_PIN GPIOF,GPIO_PIN_4
#define Y_STOP_PIN GPIOF,GPIO_PIN_5
#define OFFSET_P 0.012
#define OFFSET_N -0.012
#define XY_MM_PER_REV 69.115f
#define Z_MM_PER_REV 8.0f
#define XY_STEPS_PER_REV 6500.0f
#define USER_BUTTON GPIOC,GPIO_PIN_13
//#define REDUNDANT_TEMP_SENSOR
#define ENGINE_MIN_SPEED 0.001

// MATHS
#define SQRT_2_BY_2 0.7071067811865476
#define MIN_TO_MS 60000000.0f

#endif