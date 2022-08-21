#include "main.h"
#include "Config.h"

#define CHANGE_MOTOR_DIR(axis_dir, dir)     HAL_GPIO_WritePin(axis_dir,dir);
#define MAKE_MOTOR_STEP(axis_step)           HAL_GPIO_TogglePin(axis_step);
#define IS_MOTOR_RUNNING()              HAL_GPIO_ReadPin(AXIS_FAULT_DETECT_PIN);
#define WRITE_PIN(_pin, _value)		    HAL_GPIO_WritePin(_pin, _value == 0? GPIO_PIN_RESET:GPIO_PIN_SET)
#define READ_PIN(_pin)					HAL_GPIO_ReadPin(_pin)

void DelayMicrosecond(TIM_HandleTypeDef *timer, uint8_t time); 
void SelectDriver(uint8_t axis);
double GetTemperature();