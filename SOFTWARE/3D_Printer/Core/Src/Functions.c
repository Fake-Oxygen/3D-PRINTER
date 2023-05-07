#include "Functions.h"

uint16_t CurStepsA = 1;
uint16_t CurStepsB = 1;
uint16_t CurStepsE = 1;
uint16_t CurStepsZ = 1;

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
    case ADC_BED:
        val = (ADC_VOLTAGE / ADC_SAMPLING) * value;
        resistance = val / ((ADC_VOLTAGE - val) / HOT_BED_RESISTANCE);
        return 1 / (HOT_BED_THERM_A + HOT_BED_THERM_B * log(resistance) + HOT_BED_THERM_C * pow(log(resistance), 3)) - KELVIN_OFFSET;
        break;
    case ADC_STM_TEMP:
        return ((V30 - (ADC_VOLTAGE / ADC_SAMPLING * value)) / AVG_SLOPE + 30);
        break;
    case 6:
        val = (ADC_VOLTAGE / ADC_SAMPLING) * value;
        return val / ((ADC_VOLTAGE - val) / HOT_END_RESISTANCE);
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
    case HOT_BED:
        TIM3->CCR1 = power;
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

void MoveAndWait(uint16_t state, uint16_t speed, uint16_t axis, uint16_t dir_X, uint16_t dir_Y)
{
    switch (axis)
    {
    case X_AXIS:
        CHANGE_MOTOR_DIR(X_AXIS_DIR, dir_X);
        CHANGE_MOTOR_DIR(Y_AXIS_DIR, dir_Y);
        while (READ_PIN(X_STOP_PIN) != state)
        {
            MAKE_MOTOR_STEP(X_AXIS_STEP);
            MAKE_MOTOR_STEP(Y_AXIS_STEP);
            DelayMicrosecond(speed);
        }
        break;
    case Y_AXIS:
        CHANGE_MOTOR_DIR(X_AXIS_DIR, dir_X);
        CHANGE_MOTOR_DIR(Y_AXIS_DIR, dir_Y);
        while (READ_PIN(Y_STOP_PIN) != state)
        {
            MAKE_MOTOR_STEP(X_AXIS_STEP);
            MAKE_MOTOR_STEP(Y_AXIS_STEP);
            DelayMicrosecond(speed);
        }
        break;
    }
}

void Move(double difx, double dify, double dife, double difz)
{
    // To muszą być stałe
    double difA = difx + dify;
    double difB = difx - dify;
    double feedrateA = fabs(difA) / (fabs(difA) + fabs(difB));
    double feedrateB = fabs(difB) / (fabs(difA) + fabs(difB));
    uint16_t speed_A = (double)1 / (double)F * XY_MM_PER_REV / XY_STEPS_PER_REV * MIN_TO_US / (double)feedrateA;
    uint16_t speed_B = (double)1 / (double)F * XY_MM_PER_REV / XY_STEPS_PER_REV * MIN_TO_US / (double)feedrateB;
    uint16_t speed_Z = (double)1 / (double)F * Z_MM_PER_REV / XY_STEPS_PER_REV * MIN_TO_US;
    uint16_t stepsA = abs(difA / XY_MM_PER_REV * XY_STEPS_PER_REV);
    uint16_t stepsB = abs(difB / XY_MM_PER_REV * XY_STEPS_PER_REV);
    uint16_t stepsE = abs(dife / E_MM_PER_REV * STEPS_PER_REV);
    uint16_t stepsZ = abs(difz / Z_MM_PER_REV * XY_STEPS_PER_REV);
    uint32_t execution_time = stepsA * speed_A * feedrateA + stepsB * speed_B * feedrateB;
    uint16_t speed_E = execution_time / stepsE;

    if (stepsA >= CurStepsA)
    {
        if (GetTicks() - last_time_A >= speed_A)
        {
            if (difA < 0)
            {
                CHANGE_MOTOR_DIR(X_AXIS_DIR, COUNTERCLOCKWISE);
            }
            else
            {
                CHANGE_MOTOR_DIR(X_AXIS_DIR, CLOCKWISE);
            }
            MAKE_MOTOR_STEP(X_AXIS_STEP);
            last_time_A = GetTicks();
            CurStepsA++;
        }
    }
    if (stepsB >= CurStepsB)
    {
        if (GetTicks() - last_time_B >= speed_B)
        {
            if (difB < 0)
            {
                CHANGE_MOTOR_DIR(Y_AXIS_DIR, COUNTERCLOCKWISE);
            }
            else
            {
                CHANGE_MOTOR_DIR(Y_AXIS_DIR, CLOCKWISE);
            }
            MAKE_MOTOR_STEP(Y_AXIS_STEP);
            last_time_B = GetTicks();
            CurStepsB++;
        }
    }
    if (stepsE >= CurStepsE)
    {
        if (GetTicks() - last_time_E >= speed_E)
        {
            if (dife < 0)
            {
                CHANGE_MOTOR_DIR(E_AXIS_DIR, CLOCKWISE);
            }
            else
            {
                CHANGE_MOTOR_DIR(E_AXIS_DIR, COUNTERCLOCKWISE);
            }
            MAKE_MOTOR_STEP(E_AXIS_STEP);
            last_time_E = GetTicks();
            CurStepsE++;
        }
    }
    if (stepsZ >= CurStepsZ)
    {
        if (GetTicks() - last_time_Z >= speed_Z)
        {
            if (difz < 0)
            {
                CHANGE_MOTOR_DIR(Z_AXIS_DIR, COUNTERCLOCKWISE);
            }
            else
            {
                CHANGE_MOTOR_DIR(Z_AXIS_DIR, CLOCKWISE);
            }
            MAKE_MOTOR_STEP(Z_AXIS_STEP);
            last_time_Z = GetTicks();
            CurStepsZ++;
        }
    }

    if (stepsB == CurStepsB - 1 && stepsA == CurStepsA - 1 && stepsE == CurStepsE - 1 && stepsZ == CurStepsZ - 1)
    {
        Cur_X = X;
        Cur_Y = Y;
        Cur_E = E;
        Cur_Z = Z;
    }
}