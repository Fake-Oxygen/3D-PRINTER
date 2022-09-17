#include "Functions.h"

void SelectDriver(uint16_t axis)
{
    switch (axis)
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
        val = (ADC_VOLTAGE / ADC_SAMPLING) * value;
        resistance = val / ((ADC_VOLTAGE - val) / HOT_END_RESISTANCE);
        return 1 / (HOT_END_THERM_A + HOT_END_THERM_B * log(resistance) + HOT_END_THERM_C * pow(log(resistance), 3)) - KELVIN_OFFSET;
        break;
    case ADC_STM_TEMP:
        return ((V30 - (ADC_VOLTAGE / ADC_SAMPLING * value)) / AVG_SLOPE + 30);
        break;
    }
}

void SetHeating(uint16_t heater, uint16_t power)
{
    switch (heater)
    {
    case HOT_END:
        TIM3->CCR2 = power;
        break;
    }
}

void SetFanSpeed(uint16_t fan, uint16_t speed)
{
    switch (fan)
    {
    case HOT_END_FAN:
        TIM3->CCR3 = speed;
        break;
    }
}

void Move(double dif, uint32_t last_time, uint16_t axis, uint16_t speed)
{
    if (dif >= OFFSET_P || dif <= OFFSET_N)
    {
        if (GetTicks() - last_time >= speed && speed > 0)
        {
            isRunning = true;
            if (dif < 0)
            {
                switch (axis)
                {
                case E_AXIS:
                    CHANGE_MOTOR_DIR(E_AXIS_DIR, COUNTERCLOCKWISE);
                    Cur_E -= E_MM_PER_REV / STEPS_PER_REV;
                    MAKE_MOTOR_STEP(E_AXIS_STEP);
                    last_time_E = GetTicks();
                    break;
                case X_AXIS:
                    CHANGE_MOTOR_DIR(X_AXIS_DIR, COUNTERCLOCKWISE);
                    CHANGE_MOTOR_DIR(Y_AXIS_DIR, COUNTERCLOCKWISE);
                    Cur_X -= XY_MM_PER_REV / STEPS_PER_REV;
                    MAKE_MOTOR_STEP(X_AXIS_STEP);
                    MAKE_MOTOR_STEP(Y_AXIS_STEP);
                    last_time_X = GetTicks();
                    break;
                default:
                    break;
                }
            }
            else
            {
                switch (axis)
                {
                case E_AXIS:
                    CHANGE_MOTOR_DIR(E_AXIS_DIR, CLOCKWISE);
                    Cur_E += E_MM_PER_REV / STEPS_PER_REV;
                    MAKE_MOTOR_STEP(E_AXIS_STEP);
                    last_time_E = GetTicks();
                    break;
                case X_AXIS:
                    CHANGE_MOTOR_DIR(X_AXIS_DIR, CLOCKWISE);
                    CHANGE_MOTOR_DIR(Y_AXIS_DIR, CLOCKWISE);
                    Cur_X += XY_MM_PER_REV / STEPS_PER_REV;
                    MAKE_MOTOR_STEP(X_AXIS_STEP);
                    MAKE_MOTOR_STEP(Y_AXIS_STEP);
                    last_time_X = GetTicks();
                    break;
                default:
                    break;
                }
            }

            // DelayMicrosecond(speed);
        }
    }
    else
    {
        isRunning = false;
    }
}