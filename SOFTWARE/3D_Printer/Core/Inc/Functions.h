#ifndef FUNTIONS_H
#define FUNTIONS_H

#include "Config.h"
#include <math.h>
#include "main.h"

#define CHANGE_MOTOR_DIR(axis_dir, dir)         HAL_GPIO_WritePin(axis_dir,dir);
#define MAKE_MOTOR_STEP(axis_step)              HAL_GPIO_TogglePin(axis_step);
#define IS_MOTOR_RUNNING()                      HAL_GPIO_ReadPin(AXIS_FAULT_DETECT_PIN);
#define WRITE_PIN(_pin, _value)		            HAL_GPIO_WritePin(_pin, _value == 0? GPIO_PIN_RESET:GPIO_PIN_SET)
#define READ_PIN(_pin)					        HAL_GPIO_ReadPin(_pin)

void SelectDriver(uint16_t axis);
double GetTemperature(uint16_t adc, uint16_t value);
void SetFanSpeed(uint16_t fan, uint16_t speed);
void SetHeating(uint16_t heater, uint16_t power);
void void MoveXY(double dir_x, double dir_y, uint32_t speed, uint32_t last_tick_A, unit32_t last_tick_B);
void Move(double dif, uint32_t last_time, uint16_t axis, uint16_t speed);
void MoveAndWait(uint16_t state, uint16_t speed, uint16_t axis, uint16_t dir_X, uint16_t dir_Y);
// void line_append(uint8_t value, UART_HandleTypeDef *hlpuart);

#endif