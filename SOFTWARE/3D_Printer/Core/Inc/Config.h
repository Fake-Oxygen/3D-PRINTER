#ifndef CONFIG_H
#define CONFIG_H

#define X_AXIS_DIR GPIOF,GPIO_PIN_1
#define X_AXIS_STEP GPIOC,GPIO_PIN_6
#define Y_AXIS_DIR GPIOF,GPIO_PIN_2
#define Y_AXIS_STEP GPIOA,GPIO_PIN_7
#define E_AXIS_DIR GPIOF,GPIO_PIN_3
#define E_AXIS_STEP GPIOB,GPIO_PIN_0
#define Z_AXIS_DIR GPIOF,GPIO_PIN_8
#define Z_AXIS_STEP GPIOB,GPIO_PIN_1
#define AXIS_FAULT_DETECT_PIN GPIOF,GPIO_PIN_0
#define X_AXIS 0
#define Y_AXIS 1
#define E_AXIS 2
#define Z_AXIS 3
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
#define RxBuf_SIZE	20
#define STEPS_PER_REV 19256
#define MM_PER_REV 23
//#define REDUNDANT_TEMP_SENSOR

#endif