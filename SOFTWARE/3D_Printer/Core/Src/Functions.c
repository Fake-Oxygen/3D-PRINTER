#include "Functions.h"

void DelayMicrosecond(TIM_HandleTypeDef *timer, uint16_t time)
{
    __HAL_TIM_SET_COUNTER(timer,0);
	while (__HAL_TIM_GET_COUNTER(timer) < time);
}
void SelectDriver(uint16_t axis)
{
    switch(axis)
    {
        case X_AXIS:
            WRITE_PIN(MPLX_PIN_A, 0);
            WRITE_PIN(MPLX_PIN_B, 0);
            break;
        case Y_AXIS:
            WRITE_PIN(MPLX_PIN_A, 1);
            WRITE_PIN(MPLX_PIN_B, 0);
            break;
        case E_AXIS:
            WRITE_PIN(MPLX_PIN_A, 0);
            WRITE_PIN(MPLX_PIN_B, 1);
            break;
        case Z_AXIS:
            WRITE_PIN(MPLX_PIN_A, 1);
            WRITE_PIN(MPLX_PIN_B, 1);
            break;
    }
}

double GetTemperature(uint16_t adc, uint16_t value)
{
    double resistance, val;
    switch (adc)
    {
        case ADC_HOT_END:
            val = (ADC_VOLTAGE/ADC_SAMPLING)*value;
            resistance = val/((ADC_VOLTAGE-val)/HOT_END_RESISTANCE);
            return 1/(HOT_END_THERM_A+HOT_END_THERM_B*log(resistance)+HOT_END_THERM_C*pow(log(resistance),3))-KELVIN_OFFSET;
            break;
        case ADC_STM_TEMP:
            return ((V30-(ADC_VOLTAGE/ADC_SAMPLING*value))/AVG_SLOPE+30);
            break;
    }
}

void SetHeating(uint16_t heater, uint16_t power)
{
    switch(heater)
    {
        case HOT_END:
            TIM3->CCR2 = power;
            break;
    }
}

void SetFanSpeed(uint16_t fan, uint16_t speed)
{
    switch(fan)
    {
        case HOT_END_FAN:
            TIM3->CCR3 = speed;
            break;
    }
}